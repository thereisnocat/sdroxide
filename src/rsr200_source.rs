//! An [`IqSource`] for a Reuter RSR200(B) driven by the native driver in
//! `sdroxide-rsr200`, over either its LAN interface or (Linux/macOS) its
//! USB one — the choice lives in [`Rsr200Config::transport`], not in this
//! file: both ride the same [`Rsr200Handle`], because nothing about it was
//! ever specific to one transport.
//!
//! Receive only: the trait's transmit methods already default to errors,
//! which is the correct answer for this hardware (it has none).
//!
//! In `Rsr200ChannelMode::Separate` this is also where the two ADCs meet:
//! [`Rsr200Handle::read_pair`] hands back a sample-aligned pair — see that
//! method's own doc for why, unlike the RSPduo's two independently-arriving
//! tuner callbacks, there is nothing to reconcile here — and one of three
//! `sdroxide_dsp` combiners, chosen by [`DiversityTechnique`], turns it into
//! one stream: a noise source nulled, or two fading paths combined. The
//! *same* component the SDRplay RSPduo's own second-tuner mode uses,
//! reused rather than reimplemented — see `RSR200_PLAN.md` §3, and
//! `sdrplay_source.rs`, which this file mirrors closely for exactly that
//! reason.
//!
//! Single channel, 16-bit — the only wire shapes `sdroxide-rsr200` streams
//! yet (`RSR200_PLAN.md` steps 1–4, 7). 24-bit and the radio's own
//! *hardware* combiner (a third, distinct wire shape, step 6) are real
//! capabilities of the radio with no host-side wiring for them here yet.
//!
//! Verified working against a real RSR200: real spectrum on screen, tuning
//! and the attenuators all live. LAN (2026-08-24) over both WiFi and a
//! wired connection — clean through ÷8 decimation, ÷4/÷2 broke up even
//! wired, reading as a wire-speed ceiling rather than a bug. USB
//! (2026-08-24, same day) on Linux/macOS — clean through ÷4 (confirmed
//! against the real app, not just the standalone probe), real loss only
//! at ÷2, the highest rate (its own, narrower throughput ceiling), and
//! one real shutdown segfault found and fixed by that testing. See
//! `RSR200_PLAN.md`'s own step 3 and step 7 entries for the full account of
//! each.
//!
//! Separate mode (step 4), confirmed on real air the same day, on two real
//! antennas: [`DiversityTechnique::Decorrelate`] nulls well, as expected —
//! this is the milestone `RSR200_PLAN.md` §4 was written to reach. But
//! [`DiversityTechnique::WidebandDecorrelate`] does not work on this radio
//! as tested: rather than nulling specific interferers, it wipes out the
//! entire band — no carriers survive, only noise. Not yet root-caused; see
//! [`Rsr200Source::open_status`] and `RSR200_PLAN.md`'s own step 4 entry.
//! [`Self::log_depth`] now reports active-bin counts and null depth to the
//! log for whoever investigates next.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use anyhow::Context;
use sdroxide_dsp::{Diversity, DiversityAlgorithm, WidebandDecorrelator};
use sdroxide_radio::{Complex32, IqSource, Result};
use sdroxide_rsr200::Rsr200Handle;
use sdroxide_types::{DiversityMode, DiversityTechnique, Rsr200Config, Rsr200Diversity};

/// How long the radio may deliver nothing before the connection counts as
/// dead. LAN, so more generous than a local USB device's three seconds —
/// matching `sdroxide-rsr200::stream`'s own silence budget.
const SILENCE_BEFORE_REOPEN: Duration = Duration::from_secs(5);

/// [`DiversityTechnique::WidebandDecorrelate`]'s FFT size. Not exposed as a
/// setting (yet), matching `sdrplay_source.rs`'s own constant and reasoning:
/// gives a few hundred hertz of resolution across the RSR200's own rate
/// range with well under a millisecond of added latency.
const WB_FFT_SIZE: usize = 2048;

/// [`DiversityTechnique::WidebandDecorrelate`]'s per-bin covariance
/// smoothing time constant. Not exposed as a setting (yet); the gate
/// threshold is the tunable that technique actually needs.
const WB_AVG_TC_SECS: f32 = 0.5;

/// How often the diversity filter's achieved null depth reaches the log —
/// matching `sdrplay_source.rs`'s own interval and reasoning.
const DEPTH_LOG_INTERVAL: Duration = Duration::from_secs(10);

pub struct Rsr200Source {
    handle: Rsr200Handle,
    center: f64,
    rx_scratch: Vec<f32>,
    label: String,
    /// Mirrors of the settings the panel drives live, so `current_gains`
    /// can answer without a round trip to the stream thread.
    attenuator1: i32,
    attenuator2: i32,
    /// The second channel's settings as configured, so the panel's live
    /// controls have somewhere to land.
    div_cfg: Rsr200Diversity,
    /// `Some` when `Rsr200ChannelMode::Separate` is running and
    /// [`DiversityTechnique::Adaptive`] or [`DiversityTechnique::Decorrelate`]
    /// is selected — the two techniques that share one component, differing
    /// only in [`Diversity::algorithm`]. Mutually exclusive with
    /// [`Self::wideband`]; [`Self::rebuild_combiner`] is what keeps it so.
    diversity: Option<Diversity>,
    /// `Some` when Separate mode is running and
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
    /// yet handed to its caller.
    wb_out: VecDeque<Complex32>,
    /// The second ADC's samples, as the filter wants them.
    aux_scratch: Vec<f32>,
    aux_buf: Vec<Complex32>,
    dual: bool,
    last_depth_log: Instant,
}

impl Rsr200Source {
    pub fn open(cfg: &Rsr200Config, center_hz: f64) -> anyhow::Result<Self> {
        let handle = Rsr200Handle::open(cfg, center_hz)
            .with_context(|| format!("opening the RSR200 at {}:{}", cfg.host, cfg.port))?;
        let label = handle.label.clone();
        let dual = handle.dual();
        tracing::info!(
            "RSR200 source ready: {label}, center {center_hz:.0} Hz{}",
            if dual { ", Separate mode" } else { "" }
        );
        let mut src = Rsr200Source {
            center: center_hz,
            rx_scratch: Vec::new(),
            label,
            attenuator1: cfg.attenuator1,
            attenuator2: cfg.attenuator2,
            div_cfg: cfg.diversity.clone(),
            diversity: None,
            wideband: None,
            wb_main_scratch: Vec::new(),
            wb_produced: Vec::new(),
            wb_out: VecDeque::new(),
            aux_scratch: Vec::new(),
            aux_buf: Vec::new(),
            dual,
            last_depth_log: Instant::now(),
            handle,
        };
        if dual {
            src.rebuild_combiner();
            match cfg.diversity.technique {
                DiversityTechnique::Adaptive => tracing::info!(
                    "diversity is on: {} adaptive filter, {} taps",
                    mode_word(cfg.diversity.mode),
                    cfg.diversity.taps,
                ),
                DiversityTechnique::Decorrelate => tracing::info!(
                    "diversity is on: {} (decorrelate, whole span)",
                    mode_word(cfg.diversity.mode),
                ),
                DiversityTechnique::WidebandDecorrelate => tracing::info!(
                    "diversity is on: {} (decorrelate per bin, {WB_FFT_SIZE}-point FFT, {:.0} dB gate)",
                    mode_word(cfg.diversity.mode),
                    cfg.diversity.gate_db,
                ),
            }
        }
        Ok(src)
    }

    /// (Re)build whichever combiner [`Self::div_cfg`]'s technique calls
    /// for, discarding whatever was running before. Used both at
    /// [`Self::open`] and whenever the technique itself changes live — a
    /// pure software swap, so unlike `Rsr200ChannelMode::Separate` itself
    /// it needs no reopen.
    fn rebuild_combiner(&mut self) {
        self.wb_out.clear();
        match self.div_cfg.technique {
            DiversityTechnique::Adaptive | DiversityTechnique::Decorrelate => {
                let mut d =
                    Diversity::new(div_mode(self.div_cfg.mode), usize::from(self.div_cfg.taps), self.div_cfg.rate);
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
                    self.handle.sample_rate_hz,
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

    /// Say how the combiner is doing, occasionally.
    ///
    /// The null depth is the one number that separates "the second ADC
    /// hears the noise" from "the second ADC hears nothing the first one
    /// does", and no amount of adjusting the filter fixes the second case.
    /// For the per-bin technique, the active-bin count answers the same
    /// question a different way: there is no single convergence to watch,
    /// so "is this doing anything at all" needs its own number.
    fn log_depth(&mut self) {
        if self.last_depth_log.elapsed() < DEPTH_LOG_INTERVAL {
            return;
        }
        self.last_depth_log = Instant::now();
        if let Some(d) = self.diversity.as_ref() {
            if let Some(db) = d.depth_db() {
                tracing::info!(
                    "diversity: {db:.1} dB of the main ADC's signal is being cancelled{}",
                    if d.frozen() { ", filter held" } else { "" },
                );
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
                    "diversity: {peak:.1} dB peak null ({avg:.1} dB span average), {active}/{total} \
                     bins active{}",
                    if wb.frozen() { ", held" } else { "" },
                ),
                _ => tracing::info!("diversity: combining, {active}/{total} bins active"),
            }
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

impl IqSource for Rsr200Source {
    /// The engine is transmitting and has stopped reading, or has started
    /// again. This radio has no transmitter of its own, but a panadapter
    /// receiver fed alongside a different transmitting radio still wants
    /// its ring accounted for correctly during an over — see
    /// [`IqSource::set_rx_paused`]. There is currently nothing to actually
    /// pass this through to: `sdroxide-rsr200`'s stream thread has no
    /// paused-accounting of its own yet (unlike the USB backends'), so a
    /// full ring during an over is reported the same as any other overrun.
    /// Worth fixing if this radio is ever run as a panadapter for a
    /// transmitting station; not yet done because nothing has needed it.
    fn set_rx_paused(&mut self, _paused: bool) {}

    fn sample_rate(&self) -> f64 {
        self.handle.sample_rate_hz
    }

    fn center_hz(&self) -> f64 {
        self.center
    }

    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        self.handle.set_center_hz(hz);
        Ok(())
    }

    /// One block from the receiver — and, in Separate mode, the second ADC
    /// combined with the first.
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        let need = buf.len() * 2;
        if self.rx_scratch.len() < need {
            self.rx_scratch.resize(need, 0.0);
        }
        if self.dual && self.aux_scratch.len() < need {
            self.aux_scratch.resize(need, 0.0);
        }
        // Disjoint fields, so both landing buffers can be handed to the
        // handle while the handle borrows itself.
        let (main, aux) = (&mut self.rx_scratch[..need], &mut self.aux_scratch[..]);
        let pairs = self.handle.read_pair(main, if self.dual { aux } else { &mut [] });

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
                    self.wb_main_scratch[p] = Complex32::new(self.rx_scratch[2 * p], self.rx_scratch[2 * p + 1]);
                }
            } else {
                for p in 0..pairs {
                    buf[p] = Complex32::new(self.rx_scratch[2 * p], self.rx_scratch[2 * p + 1]);
                }
            }

            if self.dual {
                if self.aux_buf.len() < pairs {
                    self.aux_buf.resize(pairs, Complex32::new(0.0, 0.0));
                }
                for p in 0..pairs {
                    self.aux_buf[p] = Complex32::new(self.aux_scratch[2 * p], self.aux_scratch[2 * p + 1]);
                }
                if let Some(d) = self.diversity.as_mut() {
                    d.process(&mut buf[..pairs], &self.aux_buf[..pairs]);
                } else if let Some(wb) = self.wideband.as_mut() {
                    self.wb_produced.clear();
                    wb.process(&self.wb_main_scratch[..pairs], &self.aux_buf[..pairs], &mut self.wb_produced);
                    self.wb_out.extend(self.wb_produced.drain(..));
                }
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
                // filling its first block, or the radio has nothing this
                // cycle.
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

    /// The two front-end attenuators, plus (in Separate mode) the software
    /// diversity filter's own controls, riding `SetGain` for the usual
    /// reason: no new `Command` variant for settings only this backend has.
    fn set_gain_element(&mut self, name: &str, db: f64) -> Result<()> {
        match name {
            Rsr200Config::ATT1_ELEMENT => {
                self.attenuator1 = (-db).round().clamp(0.0, f64::from(Rsr200Config::ATTENUATOR_MAX_DB)) as i32;
                self.handle.set_attenuator1_db(self.attenuator1);
            }
            Rsr200Config::ATT2_ELEMENT => {
                self.attenuator2 = (-db).round().clamp(0.0, f64::from(Rsr200Config::ATTENUATOR_MAX_DB)) as i32;
                self.handle.set_attenuator2_db(self.attenuator2);
            }
            Rsr200Config::DIV_MODE_ELEMENT => {
                self.div_cfg.mode = if db >= 0.5 { DiversityMode::Combine } else { DiversityMode::Cancel };
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
            Rsr200Config::DIV_RATE_ELEMENT => {
                // Adaptive only -- decorrelation, whole-span or per-bin, has
                // nothing that converges for a rate to govern.
                self.div_cfg.rate = db as f32;
                if let Some(d) = self.diversity.as_mut() {
                    d.set_rate(self.div_cfg.rate);
                }
            }
            Rsr200Config::DIV_TAPS_ELEMENT => {
                let taps = db.round().clamp(1.0, f64::from(sdroxide_types::DIVERSITY_MAX_TAPS)) as u8;
                self.div_cfg.taps = taps;
                if let Some(d) = self.diversity.as_mut() {
                    d.set_taps(usize::from(taps));
                }
            }
            Rsr200Config::DIV_FREEZE_ELEMENT => {
                self.div_cfg.frozen = db >= 0.5;
                if let Some(d) = self.diversity.as_mut() {
                    d.set_frozen(self.div_cfg.frozen);
                }
                if let Some(wb) = self.wideband.as_mut() {
                    wb.set_frozen(self.div_cfg.frozen);
                }
            }
            Rsr200Config::DIV_RESET_ELEMENT => {
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
            Rsr200Config::DIV_TECHNIQUE_ELEMENT => {
                self.div_cfg.technique = match db.round() as i64 {
                    1 => DiversityTechnique::Decorrelate,
                    2 => DiversityTechnique::WidebandDecorrelate,
                    _ => DiversityTechnique::Adaptive,
                };
                // A pure software swap -- discards whatever the previous
                // combiner had converged or solved, but needs no reopen.
                if self.dual {
                    self.rebuild_combiner();
                }
            }
            Rsr200Config::DIV_GATE_ELEMENT => {
                self.div_cfg.gate_db = db as f32;
                if let Some(wb) = self.wideband.as_mut() {
                    wb.set_gate_db(self.div_cfg.gate_db);
                }
            }
            _ => {}
        }
        Ok(())
    }

    /// Carried negated, like every other backend here: on the sliders more
    /// is louder, and an attenuator is the opposite sense from a gain.
    fn current_gains(&self) -> Vec<(String, f64)> {
        vec![
            (Rsr200Config::ATT1_ELEMENT.to_string(), -f64::from(self.attenuator1)),
            (Rsr200Config::ATT2_ELEMENT.to_string(), -f64::from(self.attenuator2)),
        ]
    }

    /// A radio whose connection has dropped, or whose thread has died, is
    /// reported as needing a reopen so the engine reconnects on its own.
    fn needs_reopen(&self) -> bool {
        !self.handle.is_alive() || self.handle.silent_for() >= SILENCE_BEFORE_REOPEN
    }

    /// Disconnect before the engine opens a replacement — without this,
    /// changing anything in Settings → Radio on a running RSR200 leaves the
    /// old connection dangling rather than actually reconfiguring the
    /// radio, since the new session's own commands would race the old
    /// one's.
    fn release(&mut self) {
        self.handle.release();
    }

    fn open_status(&self) -> Option<String> {
        let mut msg = String::from(
            "Reuter RSR200 support is new: verified against real hardware over both LAN \
             (wired and WiFi) and USB (Linux/macOS — Windows needs its own driver research \
             first). 16-bit only — 24-bit is not wired up yet. LAN's ÷8 decimation and \
             coarser is solid over ordinary gigabit Ethernet; ÷4/÷2 broke up even wired. \
             USB held up cleanly through ÷4, with real loss only at ÷2 (the highest rate) \
             — its own, narrower throughput ceiling. Either way, expect the link — not \
             this driver — to be what limits the top end.",
        );
        if self.dual {
            match self.div_cfg.technique {
                DiversityTechnique::WidebandDecorrelate => msg.push_str(
                    " Separate mode's decorrelate-per-bin technique does not work on this \
                     radio as tested: confirmed on real air to wipe out the entire band \
                     rather than null specific interferers — no carriers survive, only \
                     noise. Not yet root-caused. Whole-span decorrelate nulls well; \
                     consider that instead until this is understood.",
                ),
                DiversityTechnique::Decorrelate => msg.push_str(
                    " Separate mode's whole-span decorrelate is confirmed on real air, \
                     nulling well on the current frequency.",
                ),
                DiversityTechnique::Adaptive => msg.push_str(
                    " Separate mode's adaptive filter has not yet been judged against two \
                     real antennas — only whole-span decorrelate has.",
                ),
            }
        }
        Some(msg)
    }
}
