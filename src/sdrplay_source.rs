//! An [`IqSource`] for an SDRplay RSP driven through the vendor's API service
//! by the native driver in `sdroxide-sdrplay` — no SoapySDR in the path.
//!
//! Receive only: the trait's transmit methods already default to errors, which
//! is the correct answer for this hardware.
//!
//! On an RSPduo with both tuners running, this is also where the two aerials
//! meet: the driver hands back a sample-aligned pair and one of three
//! `sdroxide_dsp` combiners — chosen by [`DiversityTechnique`] — turns it
//! into one stream: a noise source nulled, or two fading paths combined
//! (issue #153).

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use sdroxide_dsp::{Diversity, DiversityAlgorithm, WidebandDecorrelator};
use sdroxide_radio::{Complex32, IqSource, Result, lo_offset_for};
use sdroxide_sdrplay::SdrPlayHandle;
use sdroxide_types::{
    DiversityMode, DiversityTechnique, SdrPlayAgc, SdrPlayConfig, SdrPlayDiversity, SdrPlayModel,
};

/// How long the receiver may deliver nothing before the connection counts as
/// dead. The service delivers continuously while healthy, so three seconds of
/// silence means the session is gone, same as the other local backends.
const SILENCE_BEFORE_REOPEN: Duration = Duration::from_secs(3);

/// How often the diversity filter's achieved null depth reaches the log.
const DEPTH_LOG_INTERVAL: Duration = Duration::from_secs(10);

/// [`DiversityTechnique::WidebandDecorrelate`]'s FFT size. Not exposed as a
/// setting (yet) — 2048 gives a few hundred hertz of frequency resolution
/// across the whole rate range dual-tuner mode can reach (2 Msps or the
/// decimated rates below it), with well under a millisecond of added
/// latency either way.
const WB_FFT_SIZE: usize = 2048;

/// [`DiversityTechnique::WidebandDecorrelate`]'s per-bin covariance
/// smoothing time constant. Not exposed as a setting (yet); the gate
/// threshold is the tunable the wideband technique actually needs — see
/// `DECORRELATION_PLAN.md`.
const WB_AVG_TC_SECS: f32 = 0.5;

pub struct SdrPlaySource {
    handle: SdrPlayHandle,
    center: f64,
    rx_scratch: Vec<f32>,
    label: String,
    lo_offset: f64,
    /// Last values the operator asked for, reported back until the hardware
    /// says otherwise (the GainChange event keeps `IF` honest under AGC).
    if_gr_db: i32,
    agc: SdrPlayAgc,
    bias_tee: bool,
    antenna: String,
    /// The ports this model (and, on the RSPduo, this tuner) offers — what
    /// the capabilities advertise. Empty hides the selector.
    antennas: Vec<String>,
    /// The second tuner's settings as configured, so the panel's live controls
    /// have somewhere to land.
    div_cfg: SdrPlayDiversity,
    /// `Some` when both tuners are running and
    /// [`DiversityTechnique::Adaptive`] or [`DiversityTechnique::Decorrelate`]
    /// is selected — the two techniques that share one component, differing
    /// only in [`Diversity::algorithm`]. Mutually exclusive with
    /// [`Self::wideband`]; [`Self::rebuild_combiner`] is what keeps it so.
    diversity: Option<Diversity>,
    /// `Some` when both tuners are running and
    /// [`DiversityTechnique::WidebandDecorrelate`] is selected.
    wideband: Option<WidebandDecorrelator>,
    /// This call's raw main-channel samples, when [`Self::wideband`] is
    /// active. `buf` itself is reserved for draining [`Self::wb_out`] in
    /// that case, since the decorrelator's overlap-add latency means what
    /// comes out of one call is not what went into it.
    wb_main_scratch: Vec<Complex32>,
    /// [`WidebandDecorrelator::process`]'s own per-call output scratch,
    /// reused rather than allocated fresh every call.
    wb_produced: Vec<Complex32>,
    /// Combined samples [`Self::wideband`] has produced but `read` has not
    /// yet handed to its caller. A `VecDeque` specifically: draining from
    /// the front is O(1) amortised, unlike a `Vec`'s, which would have to
    /// shift every remaining element down on every call.
    wb_out: VecDeque<Complex32>,
    /// The second tuner's samples, as the filter wants them.
    aux_scratch: Vec<f32>,
    aux_buf: Vec<Complex32>,
    last_depth_log: Instant,
}

impl SdrPlaySource {
    pub fn open(cfg: &SdrPlayConfig, center_hz: f64) -> anyhow::Result<Self> {
        let handle = sdroxide_sdrplay::spawn(cfg, center_hz)?;
        let rate = handle.out_rate_hz();
        let dual = handle.dual_tuner();
        let label = format!(
            "{} @ {:.3} Msps{}",
            handle.label(),
            rate / 1e6,
            if dual { ", both tuners" } else { "" }
        );
        // Zero-IF part: park the LO off the VFO where the analog filter
        // allows it — see `sdroxide_radio::lo_offset_for`. A low IF has no DC
        // spike in the middle of the span to dodge, so it wants none: the
        // offset would only push the wanted signal towards the skirt of an
        // analog filter that is already the narrowest part of this mode.
        let lo_offset =
            if handle.low_if_khz() != 0 { 0.0 } else { lo_offset_for(rate, handle.analog_bw_hz()) };
        let antennas: Vec<String> =
            handle.model().antennas(cfg.duo_tuner).iter().map(|s| s.to_string()).collect();
        let antenna = if cfg.antenna.is_empty() {
            antennas.first().cloned().unwrap_or_default()
        } else {
            cfg.antenna.clone()
        };
        tracing::info!(
            "SDRplay source ready: {label}, center {center_hz:.0} Hz, \
             LO offset {lo_offset:.0} Hz (0 = LO on the VFO)"
        );
        let mut src = SdrPlaySource {
            center: center_hz,
            rx_scratch: Vec::new(),
            label,
            lo_offset,
            if_gr_db: cfg.if_gr_db,
            agc: cfg.agc,
            bias_tee: cfg.bias_tee,
            antenna,
            antennas,
            div_cfg: cfg.diversity.clone(),
            diversity: None,
            wideband: None,
            wb_main_scratch: Vec::new(),
            wb_produced: Vec::new(),
            wb_out: VecDeque::new(),
            aux_scratch: Vec::new(),
            aux_buf: Vec::new(),
            last_depth_log: Instant::now(),
            handle,
        };
        if dual {
            src.rebuild_combiner();
            let pairing_note = if src.handle.pair_stamped() {
                ""
            } else {
                " (the service is not numbering its blocks, so the two tuners are being paired \
                 by arrival order)"
            };
            match cfg.diversity.technique {
                DiversityTechnique::Adaptive => tracing::info!(
                    "diversity is on: {} adaptive filter, {} taps{pairing_note}",
                    mode_word(cfg.diversity.mode),
                    cfg.diversity.taps,
                ),
                DiversityTechnique::Decorrelate => tracing::info!(
                    "diversity is on: {} (decorrelate, whole span){pairing_note}",
                    mode_word(cfg.diversity.mode),
                ),
                DiversityTechnique::WidebandDecorrelate => tracing::info!(
                    "diversity is on: {} (decorrelate per bin, {WB_FFT_SIZE}-point FFT, \
                     {:.0} dB gate){pairing_note}",
                    mode_word(cfg.diversity.mode),
                    cfg.diversity.gate_db,
                ),
            }
        }
        Ok(src)
    }

    /// (Re)build whichever combiner [`Self::div_cfg`]'s technique calls for,
    /// discarding whatever was running before. Used both at
    /// [`Self::open`] and whenever the technique itself changes live — a
    /// pure software swap, so unlike [`SdrPlayDiversity::enabled`] it needs
    /// no reopen.
    fn rebuild_combiner(&mut self) {
        self.wb_out.clear();
        match self.div_cfg.technique {
            DiversityTechnique::Adaptive | DiversityTechnique::Decorrelate => {
                let mut d = Diversity::new(
                    div_mode(self.div_cfg.mode),
                    usize::from(self.div_cfg.taps),
                    self.div_cfg.rate,
                );
                if self.div_cfg.technique == DiversityTechnique::Decorrelate {
                    d.set_algorithm(DiversityAlgorithm::Decorrelate);
                }
                d.set_frozen(self.div_cfg.frozen);
                self.diversity = Some(d);
                self.wideband = None;
            }
            DiversityTechnique::WidebandDecorrelate => {
                let mut wb = WidebandDecorrelator::new(
                    WB_FFT_SIZE,
                    self.handle.out_rate_hz(),
                    WB_AVG_TC_SECS,
                    div_mode(self.div_cfg.mode),
                );
                wb.set_gate_db(self.div_cfg.gate_db);
                wb.set_frozen(self.div_cfg.frozen);
                self.wideband = Some(wb);
                self.diversity = None;
            }
        }
    }

    pub fn model(&self) -> SdrPlayModel {
        self.handle.model()
    }

    pub fn antennas(&self) -> &[String] {
        &self.antennas
    }

    /// Whether both tuners are running and being combined.
    pub fn dual_tuner(&self) -> bool {
        self.handle.dual_tuner()
    }

    /// Say how the combiner is doing, occasionally.
    ///
    /// The null depth is the one number that separates "the second aerial
    /// hears the noise" from "the second aerial hears nothing the first one
    /// does", and no amount of adjusting the filter fixes the second case.
    /// For the per-bin technique, the active-bin count answers the same
    /// question a different way: there is no single convergence to watch,
    /// so "is this doing anything at all" needs its own number.
    fn log_depth(&mut self) {
        if self.last_depth_log.elapsed() < DEPTH_LOG_INTERVAL {
            return;
        }
        self.last_depth_log = Instant::now();
        let slips = self.handle.pair_slips();
        let slip_note =
            if slips > 0 { format!(", {slips} pairing restart(s)") } else { String::new() };
        if let Some(d) = self.diversity.as_ref() {
            match d.depth_db() {
                Some(db) => tracing::info!(
                    "diversity: {db:.1} dB of the main aerial's signal is being \
                     cancelled{}{slip_note}",
                    if d.frozen() { ", filter held" } else { "" },
                ),
                None if slips > 0 => tracing::debug!("diversity: combining{slip_note}"),
                None => {}
            }
        } else if let Some(wb) = self.wideband.as_ref() {
            let (active, total) = (wb.active_bins(), wb.fft_size());
            // depth_db() alone reads misleadingly shallow here: it is the
            // whole span's own average, diluted by however many of `total`
            // bins had nothing to remove -- peak_depth_db() is the number
            // that actually answers "how deep is the null on whatever is
            // actually being nulled." Both together tell the fuller story;
            // neither alone does.
            match (wb.depth_db(), wb.peak_depth_db()) {
                (Some(avg), Some(peak)) => tracing::info!(
                    "diversity: {peak:.1} dB peak null ({avg:.1} dB span average), \
                     {active}/{total} bins active{}{slip_note}",
                    if wb.frozen() { ", held" } else { "" },
                ),
                _ => tracing::info!("diversity: combining, {active}/{total} bins active{slip_note}"),
            }
        }
        if self.handle.aux_stalled() {
            tracing::warn!(
                "the RSPduo's second tuner is not delivering, so blocks are going through \
                 uncombined — the receiver is still working, but diversity is not"
            );
        }
    }
}

/// The configuration's mode, as the DSP crate spells it.
fn div_mode(mode: DiversityMode) -> sdroxide_dsp::DiversityMode {
    match mode {
        DiversityMode::Cancel => sdroxide_dsp::DiversityMode::Cancel,
        DiversityMode::Combine => sdroxide_dsp::DiversityMode::Combine,
    }
}

fn mode_word(mode: DiversityMode) -> &'static str {
    match mode {
        DiversityMode::Cancel => "cancelling",
        DiversityMode::Combine => "combining",
    }
}

impl IqSource for SdrPlaySource {
    /// The engine is transmitting and has stopped reading, or has started
    /// again. Passed straight through to the stream thread, which keeps
    /// receiving either way — this only decides whether a full ring is
    /// reported as an overrun or as the ordinary cost of an over. See
    /// [`IqSource::set_rx_paused`].
    fn set_rx_paused(&mut self, paused: bool) {
        self.handle.set_rx_paused(paused);
    }

    fn sample_rate(&self) -> f64 {
        self.handle.out_rate_hz()
    }

    fn center_hz(&self) -> f64 {
        self.center
    }

    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        self.handle.set_center_hz(hz);
        Ok(())
    }

    fn lo_offset_hz(&self) -> f64 {
        self.lo_offset
    }

    /// One block from the receiver — and, with both tuners running, the second
    /// aerial combined with the first.
    ///
    /// The pair the driver hands back is sample-aligned by construction, so
    /// there is nothing to check here; what there is to respect is the second
    /// tuner having stopped, in which case the block goes through uncombined
    /// rather than combined against silence.
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        let need = buf.len() * 2;
        if self.rx_scratch.len() < need {
            self.rx_scratch.resize(need, 0.0);
        }
        let dual = self.handle.dual_tuner();
        if dual && self.aux_scratch.len() < need {
            self.aux_scratch.resize(need, 0.0);
        }
        // Disjoint fields, so both landing buffers can be handed to the handle
        // while the handle borrows itself.
        let (main, aux) = (&mut self.rx_scratch[..need], &mut self.aux_scratch[..]);
        let pairs = self.handle.read_pair(main, if dual { aux } else { &mut [] });

        if pairs > 0 {
            if self.wideband.is_some() {
                // `buf` is reserved for draining `wb_out` below, which is
                // not necessarily what this call fetched — the overlap-add
                // has its own latency. Raw main samples land in scratch
                // instead.
                if self.wb_main_scratch.len() < pairs {
                    self.wb_main_scratch.resize(pairs, Complex32::new(0.0, 0.0));
                }
                for p in 0..pairs {
                    self.wb_main_scratch[p] =
                        Complex32::new(self.rx_scratch[2 * p], self.rx_scratch[2 * p + 1]);
                }
            } else {
                for p in 0..pairs {
                    buf[p] = Complex32::new(self.rx_scratch[2 * p], self.rx_scratch[2 * p + 1]);
                }
            }

            if dual && !self.handle.aux_stalled() {
                if self.aux_buf.len() < pairs {
                    self.aux_buf.resize(pairs, Complex32::new(0.0, 0.0));
                }
                for p in 0..pairs {
                    self.aux_buf[p] =
                        Complex32::new(self.aux_scratch[2 * p], self.aux_scratch[2 * p + 1]);
                }
                if let Some(d) = self.diversity.as_mut() {
                    d.process(&mut buf[..pairs], &self.aux_buf[..pairs]);
                } else if let Some(wb) = self.wideband.as_mut() {
                    self.wb_produced.clear();
                    wb.process(
                        &self.wb_main_scratch[..pairs],
                        &self.aux_buf[..pairs],
                        &mut self.wb_produced,
                    );
                    self.wb_out.extend(self.wb_produced.drain(..));
                }
            } else if self.wideband.is_some() {
                // Second tuner stalled: the docs above's "goes through
                // uncombined" applies here too, straight into the drain
                // queue so the caller still sees the samples in order.
                self.wb_out.extend(self.wb_main_scratch[..pairs].iter().copied());
            }

            if dual {
                self.log_depth();
            }
        }

        if self.wideband.is_some() {
            let n = self.wb_out.len().min(buf.len());
            for (i, v) in self.wb_out.drain(..n).enumerate() {
                buf[i] = v;
            }
            if n == 0 {
                // Nothing ready yet -- either the overlap-add is still
                // filling its first block, or the hardware itself has
                // nothing this cycle.
                std::thread::sleep(Duration::from_millis(2));
            }
            return Ok(n);
        }

        if pairs == 0 {
            // Nothing yet — brief nap so the DSP loop doesn't spin hot.
            std::thread::sleep(Duration::from_millis(2));
            return Ok(0);
        }
        Ok(pairs)
    }

    fn describe(&self) -> String {
        self.label.clone()
    }

    /// The two real gain elements, plus pseudo-elements for everything else.
    ///
    /// Both real elements are carried negated — `IF` is −(gain reduction),
    /// `LNA` is −(state) — so that on the sliders more is louder, like every
    /// other backend. The switches ride `SetGain` for the usual reason: no
    /// new `Command` variant for settings only this backend has.
    fn set_gain_element(&mut self, name: &str, db: f64) -> Result<()> {
        match name {
            SdrPlayConfig::IF_GAIN_ELEMENT => {
                let gr = (-db).round() as i32;
                self.if_gr_db = gr.clamp(SdrPlayConfig::IF_GR_MIN, SdrPlayConfig::IF_GR_MAX);
                self.handle.set_if_gr_db(self.if_gr_db);
            }
            SdrPlayConfig::LNA_ELEMENT => {
                let state = (-db).round().clamp(0.0, 255.0) as u8;
                self.handle.set_lna_state(state);
            }
            SdrPlayConfig::AGC_ELEMENT => {
                self.agc = SdrPlayAgc::from_code(db);
                self.handle.set_agc(self.agc.code() as i32);
            }
            SdrPlayConfig::AGC_SETPOINT_ELEMENT => {
                self.handle.set_agc_setpoint(db.round() as i32);
            }
            SdrPlayConfig::PPM_ELEMENT => {
                self.handle.set_ppm(db);
            }
            SdrPlayConfig::BIAS_TEE_ELEMENT => {
                self.bias_tee = db >= 0.5;
                self.handle.set_bias_tee(self.bias_tee);
            }
            SdrPlayConfig::RF_NOTCH_ELEMENT => {
                self.handle.set_rf_notch(db >= 0.5);
            }
            SdrPlayConfig::DAB_NOTCH_ELEMENT => {
                self.handle.set_dab_notch(db >= 0.5);
            }
            SdrPlayConfig::HDR_ELEMENT => {
                self.handle.set_hdr(db >= 0.5);
            }
            // The second tuner and its filter. Everything here is live: a null
            // is found by adjusting and listening, so a control that needed a
            // reconnect would be no use at all.
            SdrPlayConfig::AUX_LNA_ELEMENT => {
                let state = (-db).round().clamp(0.0, 255.0) as u8;
                self.div_cfg.lna_state = state;
                self.handle.set_aux_lna_state(state);
            }
            SdrPlayConfig::AUX_IF_GAIN_ELEMENT => {
                let gr = (-db).round() as i32;
                self.div_cfg.if_gr_db =
                    gr.clamp(SdrPlayConfig::IF_GR_MIN, SdrPlayConfig::IF_GR_MAX);
                self.handle.set_aux_if_gr_db(self.div_cfg.if_gr_db);
            }
            SdrPlayConfig::DIV_MODE_ELEMENT => {
                self.div_cfg.mode =
                    if db >= 0.5 { DiversityMode::Combine } else { DiversityMode::Cancel };
                // Kept across the switch, both combiners: it is the same
                // estimate either way, and throwing away a converged one
                // would cost the operator the convergence they watched.
                if let Some(d) = self.diversity.as_mut() {
                    d.set_mode(div_mode(self.div_cfg.mode));
                }
                if let Some(wb) = self.wideband.as_mut() {
                    wb.set_mode(div_mode(self.div_cfg.mode));
                }
            }
            SdrPlayConfig::DIV_RATE_ELEMENT => {
                // Adaptive only -- decorrelation, whole-span or per-bin, has
                // nothing that converges for a rate to govern.
                self.div_cfg.rate = db as f32;
                if let Some(d) = self.diversity.as_mut() {
                    d.set_rate(self.div_cfg.rate);
                }
            }
            SdrPlayConfig::DIV_TAPS_ELEMENT => {
                // Adaptive only, same reasoning as the rate above.
                let taps =
                    db.round().clamp(1.0, f64::from(sdroxide_types::DIVERSITY_MAX_TAPS)) as u8;
                self.div_cfg.taps = taps;
                if let Some(d) = self.diversity.as_mut() {
                    d.set_taps(usize::from(taps));
                }
            }
            SdrPlayConfig::DIV_FREEZE_ELEMENT => {
                self.div_cfg.frozen = db >= 0.5;
                if let Some(d) = self.diversity.as_mut() {
                    d.set_frozen(self.div_cfg.frozen);
                }
                if let Some(wb) = self.wideband.as_mut() {
                    wb.set_frozen(self.div_cfg.frozen);
                }
            }
            SdrPlayConfig::DIV_RESET_ELEMENT => {
                if db >= 0.5 {
                    if let Some(d) = self.diversity.as_mut() {
                        d.reset();
                    }
                    if let Some(wb) = self.wideband.as_mut() {
                        wb.reset();
                        self.wb_out.clear();
                    }
                }
            }
            SdrPlayConfig::DIV_TECHNIQUE_ELEMENT => {
                self.div_cfg.technique = match db.round() as i64 {
                    1 => DiversityTechnique::Decorrelate,
                    2 => DiversityTechnique::WidebandDecorrelate,
                    _ => DiversityTechnique::Adaptive,
                };
                // A pure software swap -- discards whatever the previous
                // combiner had converged or solved, but needs no reopen.
                self.rebuild_combiner();
            }
            SdrPlayConfig::DIV_GATE_ELEMENT => {
                self.div_cfg.gate_db = db as f32;
                if let Some(wb) = self.wideband.as_mut() {
                    wb.set_gate_db(self.div_cfg.gate_db);
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// What the hardware is actually doing. `IF` prefers the GainChange
    /// event's answer, which follows the AGC; `LNA` is the programmed state
    /// after any per-band clamp.
    fn current_gains(&self) -> Vec<(String, f64)> {
        let gr = self.handle.effective_if_gr_db().unwrap_or(self.if_gr_db);
        vec![
            (SdrPlayConfig::LNA_ELEMENT.to_string(), -(self.handle.lna_state() as f64)),
            (SdrPlayConfig::IF_GAIN_ELEMENT.to_string(), -(gr as f64)),
        ]
    }

    fn set_antenna(&mut self, name: &str) -> Result<()> {
        self.antenna = name.to_string();
        self.handle.set_antenna(name);
        Ok(())
    }

    fn current_antenna(&self) -> String {
        self.antenna.clone()
    }

    /// An RSP whose service session died — unplug, service restart — reports
    /// as needing a reopen so the engine reconnects on its own.
    fn needs_reopen(&self) -> bool {
        !self.handle.is_alive() || self.handle.silent_for() >= SILENCE_BEFORE_REOPEN
    }

    /// Hand the device back to the service before the engine opens its
    /// replacement; without this, Apply fails with "in use" — by us.
    fn release(&mut self) {
        self.handle.release();
    }

    /// Standing conditions an operator needs to see: a degraded enumeration,
    /// DC on the antenna port, and an ADC being overloaded right now.
    fn open_status(&self) -> Option<String> {
        let mut notes = Vec::new();
        // A device the service lists without an identity streams but often
        // hears nothing — and nothing else about the session looks wrong, so
        // this note is the only thing standing between the operator and a
        // deaf receiver with no explanation.
        if let Some(w) =
            sdroxide_types::SdrPlayDevice::degraded_warning(self.handle.serial(), self.model())
        {
            notes.push(w);
        }
        if self.bias_tee {
            notes.push(format!("{}: bias tee is ON — ~4.7 V DC on the antenna port", self.label));
        }
        if self.handle.overloaded() {
            notes.push(
                "RF overload — raise the LNA slider (more attenuation), lower IF gain, or \
                 enable AGC"
                    .to_string(),
            );
        }
        // A setting that quietly did nothing is worse than one that is
        // refused: only an RSPduo has a second tuner, and only the API's
        // dual-tuner mode runs it.
        if self.div_cfg.enabled && !self.handle.dual_tuner() {
            notes.push(if self.model() == SdrPlayModel::RspDuo {
                "Both tuners were asked for but the device opened with one — another \
                 application may be holding the RSPduo, or holding it in single-tuner mode. \
                 Diversity is off."
                    .to_string()
            } else {
                format!(
                    "Diversity needs two tuners and an {} has one, so it is off.",
                    self.model().label()
                )
            });
        } else if self.handle.dual_tuner() {
            let what = match self.div_cfg.mode {
                DiversityMode::Cancel => "cancelling a noise source",
                DiversityMode::Combine => "combining two aerials",
            };
            let how = match self.div_cfg.technique {
                DiversityTechnique::Adaptive => "adaptive filter".to_string(),
                DiversityTechnique::Decorrelate => "decorrelate, whole span".to_string(),
                DiversityTechnique::WidebandDecorrelate => {
                    let (active, total) =
                        self.wideband.as_ref().map(|wb| (wb.active_bins(), wb.fft_size())).unwrap_or((0, 0));
                    format!("decorrelate per bin, {active}/{total} bins active")
                }
            };
            notes.push(format!(
                "Diversity is running on the RSPduo's second tuner — {what} ({how}). Watch the \
                 log for the depth it is reaching.",
            ));
            // The second tuner's ladder is shorter on some bands than others,
            // exactly like the first one's — and unlike the first one's, no
            // gain readout on screen would ever show it.
            let programmed = self.handle.aux_lna_state();
            if programmed != self.div_cfg.lna_state {
                notes.push(format!(
                    "The second tuner's LNA state is clamped to {programmed} in this band \
                     (you asked for {}); your choice comes back when you tune somewhere it \
                     fits.",
                    self.div_cfg.lna_state
                ));
            }
        }
        if notes.is_empty() { None } else { Some(notes.join("\n")) }
    }
}
