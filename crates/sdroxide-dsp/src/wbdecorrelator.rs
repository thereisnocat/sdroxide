//! Per-bin ("wideband") decorrelation: the same 2×2 covariance null/combine
//! [`covariance_eigen`](crate::diversity::covariance_eigen) solves, run
//! independently in every bin of a short-time Fourier transform instead of
//! once over the whole passband.
//!
//! # Why this exists, when [`crate::Diversity`] already has a decorrelation mode
//!
//! [`crate::Diversity`]'s own `DiversityAlgorithm::Decorrelate` computes
//! *one* complex weight from the whole passband's covariance — a single
//! knob-and-potentiometer null, exactly like a single-tap adaptive filter:
//! it can null one interferer, or find a middling compromise across
//! several, but nothing forces every frequency to want the same answer.
//! Solving the *same* eigendecomposition independently in every FFT bin
//! removes that constraint: each interferer gets nulled in whichever bin(s)
//! it actually occupies, simultaneously, because nothing ties one bin's
//! solve to another's. On real HF recordings this was the difference
//! between roughly 22–26 dB (one global weight) and 28–38 dB (per bin) —
//! see `DECORRELATION_PLAN.md`.
//!
//! # The instability this fixes
//!
//! Solved naively — one covariance matrix *per bin, per frame, from a
//! single sample pair* — this does not work: a lone `(main, aux)` sample is
//! always exactly rank one, so every bin's "null" is trivially perfect and
//! points in an essentially arbitrary direction set by that instant's phase
//! noise. Reconstructed across thousands of noise-floor bins, that shows up
//! as the null wandering and refusing to hold, even though any *individual*
//! bin with a real signal in it is stable on its own. Two things fix it,
//! both required:
//!
//! * **Time-smoothed covariance**, not a single frame's outer product — the
//!   same exponential-average idiom [`crate::SpectrumAnalyzer`] already
//!   uses for display power, applied to `raa`/`rbb`/`rab` instead.
//! * **A per-bin power gate**: a bin sitting far enough below the *median*
//!   bin's power is left untouched (identity weight) rather than solved at
//!   all — there is nothing there for a "direction" to be measuring, only
//!   noise. [`WidebandDecorrelator::set_gate_db`], default 20 dB, the number
//!   that worked on real material in the original work; worth retuning
//!   against this chain's own noise floor, not importing unquestioned.
//!
//! # Shape
//!
//! A weighted overlap-add analysis/resynthesis pipeline, structurally the
//! same as [`crate::WbDdc`]'s (periodic Hann, 50 % hop — which is what makes
//! the overlapped windows reassemble into a continuous signal) but without
//! that module's band-selection/decimation: this filters across the same
//! span it is given, rather than downconverting out of a larger one.

use std::sync::Arc;

use rustfft::{Fft, FftPlanner};

use crate::Complex32;
use crate::diversity::{DiversityMode, Eigenpair, cancel_weight, covariance_eigen};

/// Default power gate — see the module doc.
const DEFAULT_GATE_DB: f32 = 20.0;

/// How much of the previous frame's output-power estimate survives into the
/// next one. Only [`WidebandDecorrelator::depth_db`] depends on this.
const POWER_DECAY: f32 = 0.9;

pub struct WidebandDecorrelator {
    fft_fwd: Arc<dyn Fft<f32>>,
    fft_inv: Arc<dyn Fft<f32>>,
    n: usize,
    hop: usize,
    window: Vec<f32>,
    scale: f32,

    mode: DiversityMode,
    gate_db: f32,
    frozen: bool,
    alpha: f32,

    main_in: Vec<Complex32>,
    aux_in: Vec<Complex32>,
    main_block: Vec<Complex32>,
    aux_block: Vec<Complex32>,
    scratch: Vec<Complex32>,
    tail: Vec<Complex32>,

    /// Per-bin smoothed covariance, natural FFT order.
    raa: Vec<f32>,
    rbb: Vec<f32>,
    rab: Vec<Complex32>,
    primed: bool,

    /// Per-bin solved weight, held across frames while [`Self::frozen`].
    k0: Vec<Complex32>,
    k1: Vec<Complex32>,
    /// Whether each bin passed the power gate on the last solve — kept
    /// separately from "`k0`/`k1` happen to sit at identity" so
    /// [`Self::peak_depth_db`] can tell a gated-out bin (nothing there to
    /// null) apart from a solved bin whose own answer genuinely was unity.
    active: Vec<bool>,
    power_scratch: Vec<f32>,
    active_bins: usize,

    in_pow: f32,
    out_pow: f32,
}

impl WidebandDecorrelator {
    /// `fft_size` must be a power of two — the finer the frequency
    /// resolution, the more precisely an interferer can be nulled without
    /// taking the wanted signal in the same bin with it, at the cost of a
    /// coarser *time* resolution (each solve now averages over a longer
    /// span) and more per-frame arithmetic. A few thousand is a reasonable
    /// starting point for an HF-width span.
    pub fn new(fft_size: usize, sample_rate: f64, avg_tc_secs: f32, mode: DiversityMode) -> Self {
        assert!(fft_size >= 64 && fft_size.is_power_of_two(), "fft_size must be a power of two");

        let mut planner = FftPlanner::<f32>::new();
        let fft_fwd = planner.plan_fft_forward(fft_size);
        let fft_inv = planner.plan_fft_inverse(fft_size);
        let hop = fft_size / 2;

        // Periodic Hann: at 50 % hop this sums to exactly 1.0, which is what
        // lets the overlapped blocks reassemble into a continuous signal —
        // same reasoning as `WbDdc`'s own analysis window, and the same
        // reason it is *not* `window::hann`, which is the symmetric form
        // built for spectral display rather than exact reconstruction.
        let window: Vec<f32> = (0..fft_size)
            .map(|i| {
                let t = std::f64::consts::TAU * i as f64 / fft_size as f64;
                (0.5 - 0.5 * t.cos()) as f32
            })
            .collect();

        // rustfft normalises neither direction, so a forward+inverse round
        // trip on an unmodified spectrum returns fft_size times the input —
        // see `WbDdc`'s own note on the same arithmetic. Unlike `WbDdc` there
        // is no decimation here (the inverse FFT is the same size as the
        // forward one), so the correction is the plain `1/n`, not `2/n`.
        let scale = (1.0 / fft_size as f64) as f32;

        let scratch_len =
            fft_fwd.get_inplace_scratch_len().max(fft_inv.get_inplace_scratch_len());

        let mut d = WidebandDecorrelator {
            fft_fwd,
            fft_inv,
            n: fft_size,
            hop,
            window,
            scale,
            mode,
            gate_db: DEFAULT_GATE_DB,
            frozen: false,
            alpha: 1.0,
            main_in: Vec::with_capacity(fft_size * 2),
            aux_in: Vec::with_capacity(fft_size * 2),
            main_block: vec![Complex32::default(); fft_size],
            aux_block: vec![Complex32::default(); fft_size],
            scratch: vec![Complex32::default(); scratch_len],
            tail: vec![Complex32::default(); hop],
            raa: vec![0.0; fft_size],
            rbb: vec![0.0; fft_size],
            rab: vec![Complex32::default(); fft_size],
            primed: false,
            k0: vec![Complex32::new(1.0, 0.0); fft_size],
            k1: vec![Complex32::default(); fft_size],
            active: vec![false; fft_size],
            power_scratch: vec![0.0; fft_size],
            active_bins: 0,
            in_pow: 0.0,
            out_pow: 0.0,
        };
        d.set_avg_tc(avg_tc_secs, sample_rate);
        d
    }

    pub fn fft_size(&self) -> usize {
        self.n
    }

    pub fn mode(&self) -> DiversityMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: DiversityMode) {
        self.mode = mode;
    }

    pub fn gate_db(&self) -> f32 {
        self.gate_db
    }

    /// Bins more than this far below the frame's median bin power are left
    /// untouched rather than solved. See the module doc.
    pub fn set_gate_db(&mut self, gate_db: f32) {
        self.gate_db = gate_db.max(0.0);
    }

    pub fn frozen(&self) -> bool {
        self.frozen
    }

    /// Stop re-solving, holding every bin's weight where it is — the same
    /// meaning [`crate::Diversity::set_frozen`] has for the scalar filter.
    pub fn set_frozen(&mut self, frozen: bool) {
        self.frozen = frozen;
    }

    pub fn set_avg_tc(&mut self, tc_secs: f32, sample_rate: f64) {
        let hop_time = self.hop as f32 / sample_rate as f32;
        self.alpha = if tc_secs <= 0.0 { 1.0 } else { 1.0 - (-hop_time / tc_secs).exp() };
    }

    /// How many of this frame's bins passed the power gate and were solved,
    /// out of [`Self::fft_size`] — a cheap "is this actually doing anything"
    /// indicator for a technique with no single number like the scalar
    /// filter's convergence to watch.
    pub fn active_bins(&self) -> usize {
        self.active_bins
    }

    /// Same meaning as [`crate::Diversity::depth_db`]: how much quieter the
    /// output is than `main` was, in dB, `Cancel` only.
    ///
    /// Averaged over the *whole* span, which is a genuinely different
    /// number from [`Self::peak_depth_db`], not just a rougher version of
    /// it: a narrow, deep null in the handful of bins one interferer
    /// actually occupies gets diluted here by however many of the other
    /// [`Self::fft_size`] bins had nothing to remove at all — measured
    /// against real interference, this read a couple of dB while the
    /// per-bin peak read as a solid null on the thing actually being
    /// nulled. Reach for [`Self::peak_depth_db`] to answer "how deep is the
    /// null on what I'm nulling"; this one answers "how much of the whole
    /// span's own power changed."
    pub fn depth_db(&self) -> Option<f32> {
        if self.mode != DiversityMode::Cancel || self.in_pow <= 0.0 || self.out_pow <= 0.0 {
            return None;
        }
        Some(10.0 * (self.in_pow / self.out_pow).log10())
    }

    /// The single deepest null among this frame's *active* (gated-in) bins,
    /// in dB — `Cancel` only, `None` if nothing has been solved or nothing
    /// passed the gate. See [`Self::depth_db`]'s doc for why this is a
    /// genuinely different, usually much larger, number.
    ///
    /// Computed from the closed-form output variance for the rescaled
    /// Cancel weight (`k0` fixed at 1 — see `cancel_weight`), not by
    /// re-running the FFT on anything: `main`'s own coefficient is always
    /// 1, so the output power at bin `i` is `raa[i] + |k1[i]|²·rbb[i] +
    /// 2·Re[conj(k1[i])·rab[i]]`, the same quadratic form
    /// `covariance_eigen`'s own tests already verify reproduces an
    /// eigenvalue for the *unrescaled* eigenvector — here evaluated at the
    /// rescaled one instead, which is no different in kind.
    pub fn peak_depth_db(&self) -> Option<f32> {
        if self.mode != DiversityMode::Cancel {
            return None;
        }
        let mut best: Option<f32> = None;
        for i in 0..self.n {
            if !self.active[i] || self.raa[i] <= 0.0 {
                continue;
            }
            let out_p = self.raa[i]
                + self.k1[i].norm_sqr() * self.rbb[i]
                + 2.0 * (self.k1[i].conj() * self.rab[i]).re;
            if out_p <= 0.0 {
                continue;
            }
            let depth = 10.0 * (self.raa[i] / out_p).log10();
            best = Some(best.map_or(depth, |b| b.max(depth)));
        }
        best
    }

    /// Clear all overlap, smoothing and solved-weight state — the answer
    /// when the aerials or the interference have changed under a frozen
    /// solve, or across a retune.
    pub fn reset(&mut self) {
        self.main_in.clear();
        self.aux_in.clear();
        self.tail.fill(Complex32::default());
        self.raa.fill(0.0);
        self.rbb.fill(0.0);
        self.rab.fill(Complex32::default());
        self.primed = false;
        self.k0.fill(Complex32::new(1.0, 0.0));
        self.k1.fill(Complex32::default());
        self.active.fill(false);
        self.active_bins = 0;
        self.in_pow = 0.0;
        self.out_pow = 0.0;
    }

    /// Consume one block of sample-aligned `main`/`aux` IQ and append
    /// combined output to `out`. Latency of roughly half the FFT size is
    /// inherent to the overlap-add: no output at all is produced until the
    /// first full block has accumulated, and every call after that lags by
    /// about that much.
    ///
    /// As with [`crate::Diversity::process`], a short `aux` limits how much
    /// of `main` is consumed this call rather than processing against
    /// samples that are not really there.
    pub fn process(&mut self, main: &[Complex32], aux: &[Complex32], out: &mut Vec<Complex32>) {
        let take = main.len().min(aux.len());
        self.main_in.extend_from_slice(&main[..take]);
        self.aux_in.extend_from_slice(&aux[..take]);

        let mut pos = 0usize;
        while pos + self.n <= self.main_in.len() && pos + self.n <= self.aux_in.len() {
            for (b, (x, w)) in
                self.main_block.iter_mut().zip(self.main_in[pos..pos + self.n].iter().zip(&self.window))
            {
                *b = x * w;
            }
            for (b, (x, w)) in
                self.aux_block.iter_mut().zip(self.aux_in[pos..pos + self.n].iter().zip(&self.window))
            {
                *b = x * w;
            }

            self.fft_fwd.process_with_scratch(&mut self.main_block, &mut self.scratch);
            self.fft_fwd.process_with_scratch(&mut self.aux_block, &mut self.scratch);

            if !self.frozen {
                self.update_weights();
            }

            let mut in_acc = 0.0f32;
            let mut out_acc = 0.0f32;
            for i in 0..self.n {
                let m = self.main_block[i];
                let a = self.aux_block[i];
                let y = self.k0[i] * m + self.k1[i] * a;
                self.main_block[i] = y;
                in_acc += m.norm_sqr();
                out_acc += y.norm_sqr();
            }
            let inv_n = 1.0 / self.n as f32;
            self.in_pow = self.in_pow * POWER_DECAY + in_acc * inv_n * (1.0 - POWER_DECAY);
            self.out_pow = self.out_pow * POWER_DECAY + out_acc * inv_n * (1.0 - POWER_DECAY);

            self.fft_inv.process_with_scratch(&mut self.main_block, &mut self.scratch);

            for i in 0..self.hop {
                out.push(self.main_block[i] * self.scale + self.tail[i]);
            }
            for i in 0..self.hop {
                self.tail[i] = self.main_block[self.hop + i] * self.scale;
            }

            pos += self.hop;
        }
        if pos > 0 {
            self.main_in.drain(..pos);
            self.aux_in.drain(..pos);
        }
    }

    /// Update the smoothed per-bin covariance from `main_block`/`aux_block`
    /// (already the current frame's spectra), then re-solve every bin that
    /// passes the power gate.
    fn update_weights(&mut self) {
        if !self.primed {
            for i in 0..self.n {
                self.raa[i] = self.main_block[i].norm_sqr();
                self.rbb[i] = self.aux_block[i].norm_sqr();
                self.rab[i] = self.main_block[i] * self.aux_block[i].conj();
            }
            self.primed = true;
        } else {
            for i in 0..self.n {
                let raa_i = self.main_block[i].norm_sqr();
                let rbb_i = self.aux_block[i].norm_sqr();
                let rab_i = self.main_block[i] * self.aux_block[i].conj();
                self.raa[i] += self.alpha * (raa_i - self.raa[i]);
                self.rbb[i] += self.alpha * (rbb_i - self.rbb[i]);
                let rab_delta = self.alpha * (rab_i - self.rab[i]);
                self.rab[i] += rab_delta;
            }
        }

        // Median bin power, for the gate. select_nth_unstable is O(n)
        // average rather than a full O(n log n) sort -- this runs every
        // frame, so it is worth not paying for an order this doesn't need.
        for i in 0..self.n {
            self.power_scratch[i] = self.raa[i] + self.rbb[i];
        }
        let mid = self.n / 2;
        self.power_scratch.select_nth_unstable_by(mid, |a, b| a.total_cmp(b));
        let median = self.power_scratch[mid];
        let gate = median * 10f32.powf(-self.gate_db / 10.0);

        let mut active = 0usize;
        for i in 0..self.n {
            let power = self.raa[i] + self.rbb[i];
            if power < gate {
                // Below the noise floor's own median: nothing here for a
                // direction to be measuring. Leave it alone rather than let
                // it contribute an arbitrary momentary null -- see the
                // module doc.
                self.k0[i] = Complex32::new(1.0, 0.0);
                self.k1[i] = Complex32::default();
                self.active[i] = false;
                continue;
            }
            active += 1;
            self.active[i] = true;
            let (null, combine): (Eigenpair, Eigenpair) =
                covariance_eigen(self.raa[i], self.rbb[i], self.rab[i]);
            let (k0, k1) = match self.mode {
                DiversityMode::Cancel => cancel_weight(null),
                DiversityMode::Combine => (combine.k0, combine.k1),
            };
            self.k0[i] = k0;
            self.k1[i] = k1;
        }
        self.active_bins = active;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic complex noise, matching `diversity`'s own generator so
    /// a failure here is reproducible the same way.
    fn noise(n: usize, seed: u64) -> Vec<Complex32> {
        let mut s = seed | 1;
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            ((s >> 40) as f32 / 8_388_608.0) - 1.0
        };
        (0..n).map(|_| Complex32::new(next(), next())).collect()
    }

    fn power(x: &[Complex32]) -> f32 {
        x.iter().map(|s| s.norm_sqr()).sum::<f32>() / x.len() as f32
    }

    /// Passing the same noise as both channels, ungated (gate at 0 dB so
    /// every bin with any power at all is solved), `Cancel` should reduce
    /// the whole thing to close to silence: every bin sees a perfectly
    /// correlated pair, which is as nullable as it gets.
    #[test]
    fn identical_channels_are_nulled_almost_entirely() {
        let n = 8192;
        let x = noise(n, 1);
        let mut d = WidebandDecorrelator::new(256, 100_000.0, 0.01, DiversityMode::Cancel);
        // A gate threshold gates out bins *below* the frame's median power --
        // for white noise, close to half of them, by definition of median.
        // A very high threshold is what "don't gate anything" looks like.
        d.set_gate_db(200.0);

        let mut out = Vec::new();
        d.process(&x, &x, &mut out);

        let tail = &out[out.len() - 2048..];
        let residual = power(tail);
        let original = power(&x);
        let depth = 10.0 * (original / residual.max(1e-20)).log10();
        assert!(depth > 40.0, "only {depth:.1} dB of the identical signal was removed");
    }

    /// A single interferer confined to one bin should be nulled there
    /// without disturbing a wanted tone sitting in a completely different
    /// bin that `aux` cannot hear at all -- the whole point of doing this
    /// per bin instead of with one global weight. Modulating *noise* by a
    /// carrier only wraps its already-full-band spectrum rather than
    /// confining it, so both signals here are tones, landing exactly on
    /// the FFT's bin grid to keep window-leakage out of the way of what
    /// this test is actually checking.
    #[test]
    fn an_interferer_in_one_bin_is_nulled_without_touching_a_tone_in_another() {
        let fft = 512;
        let hop = fft / 2;
        let frames = 400;
        let n = frames * hop + fft;
        let fs = 200_000.0f32;
        let bin_hz = fs / fft as f32;

        let tone = |freq: f32, amp: f32| -> Vec<Complex32> {
            (0..n)
                .map(|i| {
                    let ph = std::f32::consts::TAU * freq * i as f32 / fs;
                    Complex32::new(ph.cos(), ph.sin()) * amp
                })
                .collect()
        };

        // Interference: bin 100, correlated between channels via a fixed
        // complex gain -- aux hears it directly, main hears it through h.
        let h = Complex32::from_polar(0.7, 1.1);
        let qrm = tone(100.0 * bin_hz, 0.5);
        // Wanted tone: bin -150, main channel only -- aux hears none of it.
        let want = tone(-150.0 * bin_hz, 0.4);

        let main: Vec<Complex32> = (0..n).map(|i| want[i] + qrm[i] * h).collect();
        let aux = qrm.clone();

        let mut d = WidebandDecorrelator::new(fft, fs as f64, 0.02, DiversityMode::Cancel);
        // Default 20 dB gate: only the handful of bins the two tones'
        // window main-lobes actually spread into should ever get solved.

        let mut out = Vec::new();
        d.process(&main, &aux, &mut out);
        assert!(out.len() > fft, "expected steady-state output, got {}", out.len());

        // Skip the startup transient while the smoothing settles.
        let settle = out.len() / 2;
        let tail = &out[settle..];
        let want_tail = &want[settle..settle + tail.len()];
        let qrm_in_main_tail: Vec<Complex32> =
            (settle..settle + tail.len()).map(|i| qrm[i] * h).collect();

        let kept_fraction = |target: &[Complex32], target_amp: f32| -> f32 {
            let corr: Complex32 = tail
                .iter()
                .zip(target)
                .map(|(o, t)| o * t.conj())
                .fold(Complex32::new(0.0, 0.0), |a, b| a + b);
            corr.norm() / (tail.len() as f32 * target_amp * target_amp)
        };

        let want_kept = kept_fraction(want_tail, 0.4);
        assert!(want_kept > 0.85, "the wanted tone survived at only {want_kept:.3} of its amplitude");

        let qrm_kept = kept_fraction(&qrm_in_main_tail, (0.5 * h.norm()).max(1e-6));
        assert!(qrm_kept < 0.2, "the interferer survived at {qrm_kept:.3} of its amplitude, not nulled");

        // This is the exact shape depth_db()'s own doc warns about: one
        // narrow, deep null diluted by ~500 other bins with nothing to
        // remove. peak_depth_db() should read like a real null (the
        // amplitude check above already implies >14 dB); depth_db()'s
        // whole-span average should read much smaller, not because the
        // null is shallow but because it is one bin out of many.
        let peak = d.peak_depth_db().expect("a bin was actively nulled");
        let avg = d.depth_db().expect("Cancel mode reports a whole-span average");
        assert!(peak > 14.0, "peak per-bin depth only {peak:.1} dB");
        assert!(peak > avg + 6.0, "peak {peak:.1} dB isn't meaningfully deeper than the {avg:.1} dB average");
    }

    /// Gating out the quiet bins should leave a genuinely dead `aux`
    /// span untouched rather than let it contribute an arbitrary null --
    /// the per-bin analogue of `diversity`'s own
    /// `a_dead_auxiliary_channel_leaves_main_untouched_in_cancel_mode`.
    #[test]
    fn a_silent_auxiliary_channel_leaves_main_close_to_untouched() {
        let n = 8192;
        let x = noise(n, 0xd00d);
        let silence = vec![Complex32::default(); n];
        let mut d = WidebandDecorrelator::new(256, 100_000.0, 0.01, DiversityMode::Cancel);

        let mut out = Vec::new();
        d.process(&x, &silence, &mut out);

        let settle = out.len() / 2;
        let orig_aligned = &x[settle..out.len()];
        let out_tail = &out[settle..];
        let mut max_err = 0.0f32;
        for (o, m) in out_tail.iter().zip(orig_aligned) {
            max_err = max_err.max((o - m).norm());
        }
        assert!(max_err < 0.05, "a dead aux channel altered main by up to {max_err:.4}");
    }

    /// Freezing holds every bin's weight, mirroring
    /// `diversity::tests::freezing_stops_the_filter_moving`.
    #[test]
    fn freezing_holds_every_bins_weight() {
        let n = 8192;
        let x = noise(n, 5);
        let y = noise(n, 6);
        let mut d = WidebandDecorrelator::new(256, 100_000.0, 0.01, DiversityMode::Cancel);
        d.set_gate_db(0.0);

        let mut out = Vec::new();
        d.process(&x, &y, &mut out);
        let held_k0 = d.k0.clone();
        let held_k1 = d.k1.clone();

        d.set_frozen(true);
        let x2 = noise(n, 55);
        let y2 = noise(n, 66);
        let mut out2 = Vec::new();
        d.process(&x2, &y2, &mut out2);
        assert_eq!(d.k0, held_k0, "k0 moved while frozen");
        assert_eq!(d.k1, held_k1, "k1 moved while frozen");
    }
}
