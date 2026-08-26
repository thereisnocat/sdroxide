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
//! Single channel, Serial (both ADCs time-interleaved into one stream),
//! Separate (two ADCs, combined here in software), or hardware diversity
//! (two ADCs, combined by the radio itself before a sample reaches the host
//! — channel A carries the result, channel B is read off the wire but
//! unused) — [`Rsr200Config::channel_mode`] picks which, and
//! [`Rsr200Config::bits24`] the sample width, independently (`RSR200_PLAN.md`
//! steps 1–8, all done). Hardware diversity's own weight is solved once in software
//! (whole-span decorrelate, while running in Separate mode — see
//! [`Rsr200Source::log_hardware_diversity_solve`]) and applied by
//! reopening into `Rsr200ChannelMode::HardwareDiversity`, not adjusted
//! live — the round trip through the command channel is too slow for a
//! control loop, confirmed in the SDR++ sibling implementation this whole
//! flow (including a real channel-2-needs-an-explicit-unity-weight bug
//! that implementation found and fixed) is drawn from.
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
//! Separate mode (step 4) and hardware diversity (step 6) were both
//! believed confirmed on real air, on two real antennas, the day they were
//! built. **Step 8 found that belief was wrong**: reading the DP directly
//! for Serial mode's own switch-register requirements turned up that
//! `SW_ADC2_TO_HF2` — the bit that actually routes ADC2 to the physical HF2
//! connector — had never been set in either dual-channel mode, so every
//! "confirmed on air" run through step 7 had both ADCs listening to the
//! same HF1 antenna, not two genuinely independent aerials. Fixed in step 8;
//! see `RSR200_PLAN.md`'s own step 4/6 entries and their step-8 correction
//! notes for the full account. Retested the same day against two real,
//! physically separate antennas: [`DiversityTechnique::Decorrelate`], left
//! running continuously rather than frozen once converged, now shows
//! audible distortion and a wandering null rather than the clean result the
//! pre-fix (same-antenna) test found — not a regression, the first
//! non-degenerate test this technique has had on this radio, and it needs a
//! follow-up with Hold/Freeze engaged before it can be judged either way.
//! [`DiversityTechnique::WidebandDecorrelate`] still does not work on this
//! radio as tested: rather than nulling specific interferers, it wipes out
//! the entire band — no carriers survive, only noise. Not yet root-caused,
//! and not yet retested against the routing fix either; see
//! [`Rsr200Source::open_status`] and `RSR200_PLAN.md`'s own step 4 entry.
//! [`Self::log_depth`] reports active-bin counts and null depth to the log
//! for whoever investigates next.
//!
//! Hardware diversity (step 6) follows the SDR++ sibling implementation's
//! own already-tested design exactly, including a real bug that
//! implementation found and fixed live (channel 2's weight has to be
//! explicitly set to unity in Separate mode too, or it silently inherits a
//! stale weight from a previous hardware-diversity session and reads as an
//! exact, clean zero) — and was confirmed against the real RSR200:
//! `OpMode::Diversity` and the channel-2 weight command both accepted at
//! unity and at a real non-unity weight (magnitude 0.5, phase 45°), real
//! samples streaming cleanly afterward either way. What that run does *not*
//! prove, and still has not proven even after step 8's routing fix, is that
//! the *combining* itself is correct — which channel actually carries the
//! result, whether a solved weight actually nulls or combines something
//! real — that needs its own retest with two genuinely independent aerials
//! now that they are, for the first time, actually in the signal path.
//!
//! Step 8 itself (Auto-ATT, Serial mode, VHF/preamp switching,
//! swap-channels) shipped on protocol-level confidence — the underlying
//! commands were already hardware-verified at step 1 — and on the DP read
//! directly rather than a fresh real-hardware probe of its own; none exists
//! yet for this step's own new surface area.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use anyhow::Context;
use sdroxide_dsp::{Diversity, DiversityAlgorithm, WidebandDecorrelator};
use sdroxide_radio::{Complex32, IqSource, Result};
use sdroxide_rsr200::Rsr200Handle;
use sdroxide_types::{DiversityMode, DiversityTechnique, Rsr200ChannelMode, Rsr200Config, Rsr200Diversity};

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

/// How often the status header (temperature, GPS-corrected clock offset)
/// reaches the log — `RSR200_PLAN.md` step 5. Coarser than
/// [`DEPTH_LOG_INTERVAL`]: this is background telemetry, not something an
/// operator is actively watching converge.
const STATUS_LOG_INTERVAL: Duration = Duration::from_secs(30);

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
    /// Wire is 2-channel — true for both `Rsr200ChannelMode::Separate` and
    /// `Rsr200ChannelMode::HardwareDiversity`.
    dual: bool,
    /// `Rsr200ChannelMode::HardwareDiversity`: the radio has already
    /// combined the channels by the time a sample arrives (channel A
    /// carries the result), so [`Self::diversity`]/[`Self::wideband`] stay
    /// `None` and `read()`'s existing "no combiner running" path — already
    /// correct for a single ADC — is exactly the right behaviour here too,
    /// with no changes of its own needed.
    hw_diversity: bool,
    /// Whether the radio was told to discipline its ADC clock from GPS —
    /// [`sdroxide_rsr200::protocol::freq_correction_hz`]'s resolution
    /// depends on it (0.5 Hz/LSB disciplining, 0.1 Hz/LSB only measuring).
    /// A reopen-trigger in the config, so a plain mirror needs no live
    /// updates.
    gps_discipline: bool,
    last_depth_log: Instant,
    last_status_log: Instant,
}

impl Rsr200Source {
    pub fn open(cfg: &Rsr200Config, center_hz: f64) -> anyhow::Result<Self> {
        let handle = Rsr200Handle::open(cfg, center_hz)
            .with_context(|| format!("opening the RSR200 at {}:{}", cfg.host, cfg.port))?;
        let label = handle.label.clone();
        let dual = handle.dual();
        let hw_diversity = cfg.channel_mode == Rsr200ChannelMode::HardwareDiversity;
        tracing::info!(
            "RSR200 source ready: {label}, center {center_hz:.0} Hz{}",
            match cfg.channel_mode {
                Rsr200ChannelMode::Single => "",
                Rsr200ChannelMode::Separate => ", Separate mode",
                Rsr200ChannelMode::HardwareDiversity => ", hardware diversity",
                Rsr200ChannelMode::Serial => ", Serial mode",
            }
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
            hw_diversity,
            gps_discipline: cfg.gps_discipline,
            last_depth_log: Instant::now(),
            last_status_log: Instant::now(),
            handle,
        };
        if dual && !hw_diversity {
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
        } else if hw_diversity {
            tracing::info!(
                "hardware diversity active: magnitude {:.3}, phase {:+.1} deg",
                cfg.hw_div_magnitude,
                cfg.hw_div_phase_deg
            );
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
                // See `Self::refresh_ref_band`'s own doc: harmless when the
                // technique is Adaptive (unused there), and needs `self.diversity`
                // to already be `Some`, which it now is.
                self.refresh_ref_band();
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

    /// Push [`Self::div_cfg`]'s current reference-band settings into the live
    /// combiner — a pure parameter update, like [`Rsr200Config::DIV_GATE_ELEMENT`]'s
    /// own handler, not a rebuild: `decorr_k0`/`decorr_k1` (whatever is
    /// currently solved or frozen) survive it. A no-op when the active
    /// technique isn't [`DiversityTechnique::Decorrelate`] (`self.diversity`
    /// is `None` for `WidebandDecorrelate`, and `set_ref_band` on an
    /// `Adaptive`-algorithm `Diversity` is harmless but unused — see
    /// [`Rsr200Diversity::ref_band_enabled`]'s own doc for why it applies to
    /// neither).
    fn refresh_ref_band(&mut self) {
        if let Some(d) = self.diversity.as_mut() {
            d.set_ref_band(
                self.div_cfg.ref_band_enabled,
                self.handle.sample_rate_hz,
                self.div_cfg.ref_band_freq_hz - self.center,
                self.div_cfg.ref_band_width_hz,
            );
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
            let whitening = if d.is_capturing_noise() {
                ", noise calibration in progress"
            } else if d.has_whitening() {
                ", whitened"
            } else {
                ""
            };
            if let Some(db) = d.depth_db() {
                tracing::info!(
                    "diversity: {db:.1} dB of the main ADC's signal is being cancelled{}{whitening}",
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

    /// Temperature and GPS-corrected clock offset, occasionally —
    /// `RSR200_PLAN.md` step 5's "GPS discipline/correction readout". The
    /// protocol-level parsing (`sdroxide_rsr200::protocol::Status`,
    /// `freq_correction_hz`) was already built and tested in step 1; this
    /// is what was missing to actually read it once the radio is running.
    fn log_status(&mut self) {
        if self.last_status_log.elapsed() < STATUS_LOG_INTERVAL {
            return;
        }
        self.last_status_log = Instant::now();
        let s = self.handle.status();
        let correction = if s.freq_correction_valid {
            format!(
                "{:+.1} Hz clock correction ({})",
                sdroxide_rsr200::protocol::freq_correction_hz(&s, self.gps_discipline),
                if self.gps_discipline { "disciplining" } else { "measuring only" }
            )
        } else {
            "no GPS fix".to_string()
        };
        // The temperature byte doubles as the Auto-ATT indicator (DP's own
        // 0x80 sentinel) — `Status::temperature_c` is forced to 0 while
        // engaged, which is not a real reading and must not be printed as
        // one.
        if s.auto_att_active {
            tracing::info!("RSR200: Auto-ATT engaged, {correction}");
        } else {
            tracing::info!("RSR200: {}°C, {correction}", s.temperature_c);
        }
    }

    /// Solve the current whole-span decorrelation weight for the radio's
    /// own hardware combiner and log it — `RSR200_PLAN.md` §4's "solve,
    /// then apply" flow. Only meaningful in `Rsr200ChannelMode::Separate`
    /// with `DiversityTechnique::Decorrelate` selected: that is the one
    /// combination whose running [`Diversity`] already has a fresh, whole-
    /// span-scalar `decorrelated_weight()` on every block — `Adaptive`'s
    /// multi-tap filter has no single complex weight to read out at all,
    /// and `WidebandDecorrelate`'s one weight per bin is exactly what the
    /// radio's own combiner (one weight, full stop) cannot use (see
    /// `RSR200_PLAN.md` §4's own note on why the wideband technique does
    /// not apply to hardware diversity).
    ///
    /// Logged rather than written back into a settings field: there is no
    /// wire from a running `IqSource` back into the settings dialog for
    /// any backend yet (`RSR200_PLAN.md` step 5 hit the same gap for the
    /// GPS/status readout) — copy the numbers from the log into the
    /// Hardware diversity magnitude/phase fields, then switch Channels to
    /// apply them.
    fn log_hardware_diversity_solve(&self) {
        if self.hw_diversity || self.div_cfg.technique != DiversityTechnique::Decorrelate {
            tracing::warn!(
                "RSR200: hardware-diversity solve needs Separate mode with whole-span \
                 decorrelate selected — nothing to solve from the current settings."
            );
            return;
        }
        let Some((k0, k1)) = self.diversity.as_ref().and_then(Diversity::decorrelated_weight) else {
            tracing::warn!("RSR200: nothing solved yet — give the filter a block or two, then try again.");
            return;
        };
        let to64 = |c: Complex32| sdroxide_rsr200::protocol::Complex64::new(f64::from(c.re), f64::from(c.im));
        let h = sdroxide_rsr200::protocol::hardware_weight_for(to64(k0), to64(k1));
        if !h.representable {
            let extra = if h.suggest_swap {
                " (channel A contributes essentially nothing — this ratio wants the aerials \
                 the other way round, but sdroxide has no channel-swap control yet)"
            } else {
                ""
            };
            tracing::warn!(
                "RSR200: hardware-diversity solve is not representable in the radio's \
                 0.001..8x range{extra}."
            );
            return;
        }
        tracing::info!(
            "RSR200: hardware-diversity solve: magnitude {:.3}, phase {:+.1} deg — copy into the \
             Hardware diversity fields and switch Channels to apply.",
            h.magnitude,
            h.phase_degrees
        );
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
        self.log_status();
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
                if self.dual && !self.hw_diversity {
                    self.rebuild_combiner();
                }
            }
            Rsr200Config::DIV_GATE_ELEMENT => {
                self.div_cfg.gate_db = db as f32;
                if let Some(wb) = self.wideband.as_mut() {
                    wb.set_gate_db(self.div_cfg.gate_db);
                }
            }
            Rsr200Config::DIV_HW_SOLVE_ELEMENT if db >= 0.5 => self.log_hardware_diversity_solve(),
            Rsr200Config::DIV_REFBAND_ENABLED_ELEMENT => {
                self.div_cfg.ref_band_enabled = db >= 0.5;
                self.refresh_ref_band();
            }
            Rsr200Config::DIV_REFBAND_FREQ_ELEMENT => {
                self.div_cfg.ref_band_freq_hz = db;
                self.refresh_ref_band();
            }
            Rsr200Config::DIV_REFBAND_WIDTH_ELEMENT => {
                self.div_cfg.ref_band_width_hz = db.max(1.0);
                self.refresh_ref_band();
            }
            Rsr200Config::DIV_CAPTURE_NOISE_ELEMENT if db >= 0.5 => {
                if let Some(d) = self.diversity.as_mut() {
                    d.capture_noise(1.0, self.handle.sample_rate_hz);
                    tracing::info!("RSR200: noise calibration armed — keep the radio on a quiet channel for ~1 s");
                }
            }
            Rsr200Config::DIV_CLEAR_WHITENING_ELEMENT if db >= 0.5 => {
                if let Some(d) = self.diversity.as_mut() {
                    d.clear_whitening();
                    tracing::info!("RSR200: noise calibration cleared — back to the un-whitened solve");
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
             first). LAN's ÷8 decimation and coarser is solid over ordinary gigabit \
             Ethernet; ÷4/÷2 broke up even wired. USB held up cleanly through ÷4, with real \
             loss only at ÷2 (the highest rate) — its own, narrower throughput ceiling. \
             Either way, expect the link — not this driver — to be what limits the top end.",
        );
        if self.hw_diversity {
            msg.push_str(
                " Hardware diversity (the radio's own combiner) is confirmed against real \
                 hardware: the mode switch and the channel-2 weight command both work, at \
                 unity and at a real non-unity weight, with clean streaming afterward. Not \
                 yet confirmed: that the combining itself is correct — which channel \
                 carries the result, whether a solved weight actually nulls or combines \
                 something real — that needs two real aerials and a human listening.",
            );
        } else if self.dual {
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
        // Overload is worth a standing note the way sdrplay_source.rs's own
        // is: nothing else about the session looks wrong when the front
        // end is being driven into overload, and reading it costs nothing
        // — the status header rides every block already.
        let s = self.handle.status();
        if s.overload_ch1 {
            msg.push_str(" ADC1 is overloaded — raise the attenuator.");
        }
        if self.dual && s.overload_ch2 {
            msg.push_str(" ADC2 is overloaded — raise its attenuator.");
        }
        Some(msg)
    }
}
