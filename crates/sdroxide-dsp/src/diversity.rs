//! Two receive antennas, one output: null a local noise source, or combine the
//! pair for the better signal-to-noise ratio of the two.
//!
//! # What "diversity" means here
//!
//! Two things, which is why this has a mode switch. Both start from the same
//! place — a second receiver, coherent with the first because the two chains
//! share one synthesiser and one sample clock — and they differ only in what
//! is done with the answer.
//!
//! * **[`DiversityMode::Cancel`]** is the QRM/QRN killer, the DSP form of what
//!   an MFJ-1026 or an X-Phase does with a knob: point a second aerial at the
//!   noise, and subtract what it hears from what the main aerial hears. The
//!   filter below finds the gain, phase and delay that make the two versions of
//!   the noise line up, and what is left is the band without it. What it costs
//!   is that anything *both* aerials hear equally goes with it, so the second
//!   one wants to be the one that hears the interference and not much else — a
//!   short whip by the offending switched-mode supply, a loop pointed at it, or
//!   simply the noisier of two aerials.
//!
//! * **[`DiversityMode::Combine`]** is diversity reception proper: two aerials
//!   on the same signal, added in the phase that makes them reinforce, weighted
//!   so the one with the better signal counts for more. On HF, where the two
//!   fade independently, this fills in the fades; the theoretical gain over the
//!   better branch alone is `1 + |r|²`, where `r` is the amplitude ratio
//!   between them — 3 dB for two equal aerials, and much more when one of them
//!   is momentarily in a null.
//!
//! # One filter, two arithmetics
//!
//! Both modes run the same adaptive filter and differ in one line at the end.
//!
//! The filter is a normalised LMS transversal filter `W` fed by the auxiliary
//! channel and driven by the error against the main one:
//!
//! ```text
//! y[n] = Σ_k w[k] · aux[n−k]
//! e[n] = main[n] − y[n]
//! w[k] += mu · e[n] · conj(aux[n−k]) / (Σ|aux|² + ε)
//! ```
//!
//! It converges on the Wiener solution: `W` becomes the transfer function that
//! turns the auxiliary channel into whatever part of the main channel can be
//! predicted from it — which is exactly the common signal, whether that is a
//! noise source or a station.
//!
//! **Multi-tap on purpose.** A single complex weight is a gain and a phase, and
//! that is all a knob-and-potentiometer canceller has; it nulls at one
//! frequency and gets steadily worse either side of it. Over the megahertz of
//! span an SDR shows, two aerials with different feedlines differ in *delay*
//! too, and a delay is a phase that turns with frequency. Each tap buys one
//! sample period of the difference the filter can equalise, so a handful of
//! them is the difference between a notch and a band that is quiet all the way
//! across. They are not free — see [`Diversity::cost_note`].
//!
//! Then:
//!
//! * `Cancel` outputs the error itself, `main − W·aux`: the common part
//!   removed.
//! * `Combine` outputs `(‖W‖²·main + W·aux) / (1 + ‖W‖²)`. That is
//!   maximal-ratio combining written in terms of the same filter: `W·aux` is
//!   the auxiliary channel referred to the main one's scale, and weighting the
//!   two branches by the inverse of their noise powers — which is what the
//!   `‖W‖²` and the `1` are — is what makes the sum optimal rather than merely
//!   in phase. It assumes the two branches have **comparable noise floors**,
//!   so the auxiliary chain's gain is a real setting and not a convenience:
//!   set it so the band noise reads about the same on both.
//!
//! # What it cannot do
//!
//! Nothing here separates a wanted signal from an unwanted one; the filter only
//! knows what the two channels have in common. Point both aerials at the same
//! thing in `Cancel` and it will dutifully cancel the station you are listening
//! to. The honest workflow is to watch the waterfall while it converges, and
//! [`Diversity::set_frozen`] the moment it has: a converged null holds while
//! the band changes around it, and a filter left adapting will re-aim itself at
//! whatever is loudest.

use crate::Complex32;

/// What to do with the two channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiversityMode {
    /// Subtract what the auxiliary aerial hears: kill the noise source.
    Cancel,
    /// Add the two in phase, weighted by branch quality: diversity reception.
    Combine,
}

/// How the combining weight in [`DiversityMode`] gets computed. Orthogonal to
/// the mode: both answer *what* the two channels should become, this answers
/// *how* the weight that gets them there is found.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DiversityAlgorithm {
    /// The adaptive filter documented at the top of this module: multi-tap,
    /// so it can follow a delay as well as a gain and phase, at the cost of
    /// having to converge toward the answer rather than landing on it.
    #[default]
    Adaptive,
    /// A closed-form solve of the two channels' 2×2 covariance matrix — see
    /// [`covariance_eigen`]. One complex weight, no delay compensation, but
    /// no convergence to wait for either: [`DiversityMode::Cancel`] and
    /// [`DiversityMode::Combine`] both fall out of the *same* solve (the
    /// smaller and larger eigenvalue respectively), rather than needing two
    /// different filters the way `Adaptive` effectively does. See
    /// `DECORRELATION_PLAN.md` for the fuller case for this, including the
    /// real-air numbers that motivated it.
    Decorrelate,
}

/// One eigenpair of a 2×2 Hermitian covariance matrix: a unit-norm combining
/// weight `(k0, k1)` — `y = k0·main + k1·aux` — together with the variance
/// that combination produces. Unit norm is what makes that variance come out
/// exactly equal to `eigenvalue`: [`Diversity::depth_db`]'s existing
/// `10·log10(in/out)` needs no extra scaling to work for this weight too.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Eigenpair {
    pub eigenvalue: f32,
    pub k0: Complex32,
    pub k1: Complex32,
}

/// Eigendecomposition of the 2×2 Hermitian matrix `[[raa, rab], [conj(rab),
/// rbb]]` — `raa`/`rbb` the two channels' own (real, non-negative) mean
/// powers, `rab` their cross-covariance `E[main·conj(aux)]`. Closed-form: a
/// 2×2 Hermitian matrix's eigenvalues are always real, and are the roots of
/// its characteristic polynomial, `((raa+rbb) ± √((raa−rbb)² + 4|rab|²)) / 2`;
/// each eigenvector is `(rab, λ−raa)` (normalised), except when `rab` is
/// (numerically) zero, where the channels are already uncorrelated and the
/// eigenvectors are the trivial `(1,0)`/`(0,1)` basis.
///
/// Returns `(min, max)`: the smaller eigenvalue's pair first — the
/// combination as decorrelated from itself as this pair of channels allows,
/// i.e. the null — then the larger — the combination as correlated as
/// possible, i.e. maximal-ratio combining.
pub fn covariance_eigen(raa: f32, rbb: f32, rab: Complex32) -> (Eigenpair, Eigenpair) {
    let sum = raa + rbb;
    let disc = ((raa - rbb) * (raa - rbb) + 4.0 * rab.norm_sqr()).max(0.0).sqrt();
    let lo = (sum - disc) * 0.5;
    let hi = (sum + disc) * 0.5;

    if rab.norm_sqr() < EPS {
        // Already diagonal: the trivial basis vectors, ordered by which
        // channel is quieter (that one's own axis is the null).
        let (one, zero) = (Complex32::new(1.0, 0.0), Complex32::new(0.0, 0.0));
        return if raa <= rbb {
            (
                Eigenpair { eigenvalue: lo, k0: one, k1: zero },
                Eigenpair { eigenvalue: hi, k0: zero, k1: one },
            )
        } else {
            (
                Eigenpair { eigenvalue: lo, k0: zero, k1: one },
                Eigenpair { eigenvalue: hi, k0: one, k1: zero },
            )
        };
    }

    let vec_for = |lambda: f32| -> (Complex32, Complex32) {
        let k1 = lambda - raa;
        let norm = (rab.norm_sqr() + k1 * k1).sqrt().max(EPS);
        // `(rab, k1)` solves `M·v = λ·v` (verified against the characteristic
        // polynomial in the module's own commit history). But the
        // combination that actually *produces* variance `λ` is the Hermitian
        // form `v^H·M·v`, i.e. `conj(v0)·main + conj(v1)·aux` -- so what gets
        // returned here, meant to be applied directly as `k0·main + k1·aux`,
        // is that conjugate: `k1` is already real (self-conjugate), only
        // `k0` needs it.
        (rab.conj() / norm, Complex32::new(k1 / norm, 0.0))
    };
    let (k0_lo, k1_lo) = vec_for(lo);
    let (k0_hi, k1_hi) = vec_for(hi);
    (
        Eigenpair { eigenvalue: lo, k0: k0_lo, k1: k1_lo },
        Eigenpair { eigenvalue: hi, k0: k0_hi, k1: k1_hi },
    )
}

/// Turns a null [`Eigenpair`] into a `Cancel`-style weight: unity gain on
/// `main`, i.e. `main + k1·aux` rather than the raw jointly-unit-norm
/// `null.k0·main + null.k1·aux`. See [`Diversity::process`]'s own comment on
/// why the rescale matters — a signal `aux` has no part of stays untouched
/// only in this form, not the raw eigenvector.
///
/// Shared by [`Diversity`] and [`crate::wbdecorrelator::WidebandDecorrelator`]
/// (one global weight, one per FFT bin) so the degenerate case below is one
/// piece of reasoning, not two copies that could quietly drift apart.
///
/// Degenerate case: when `null.k0` is itself (numerically) zero, `main`
/// carries essentially none of the null direction — `aux` is the quieter,
/// uncorrelated channel here. The rescale is undefined (dividing by zero),
/// and the tempting answer — "output `aux` alone, it *is* the null" — is a
/// trap: it is only a sensible null if `aux` actually has power to offer.
/// Nothing here distinguishes "`aux` is quiet because it is genuinely a
/// clean reference" from "`aux` is quiet because it is dead" — for the
/// global scalar case a dead `aux` is a whole-band silence an operator
/// would notice immediately, but for the per-bin case it would show up as
/// scattered, easy-to-miss silent holes wherever `aux` happens to be a
/// touch quieter than `main`. Identity — leave `main` alone — is the safe
/// default in both cases: failing to null a bin is a smaller failure than
/// silencing it.
///
/// The guard is on the *ratio* `k1`, not on `null.k0` against some absolute
/// floor like [`EPS`] — a first cut of this function did that and it broke
/// under real use: a per-bin caller (`WidebandDecorrelator`) can hand in a
/// `k0` that is only *window-sidelobe-leakage* small, not truly zero —
/// bigger than any fixed epsilon, but still small enough that dividing by
/// it amplifies numerical noise into a wild, effectively garbage `k1`,
/// which then injects that noise into the reconstructed bin. `k1` is a
/// dimensionless gain ratio, so a fixed bound on *it* is scale-invariant in
/// a way a threshold on `k0` alone can never be: dividing by an exact zero
/// yields a non-finite `k1` (caught by `is_finite`), and dividing by a
/// merely-tiny-but-nonzero one yields a `k1` so large it is not physically
/// a real antenna pairing's gain ratio either way (caught by the bound).
pub fn cancel_weight(null: Eigenpair) -> (Complex32, Complex32) {
    let k1 = null.k1 / null.k0;
    if k1.norm_sqr().is_finite() && k1.norm_sqr() <= MAX_CANCEL_RATIO * MAX_CANCEL_RATIO {
        (Complex32::new(1.0, 0.0), k1)
    } else {
        (Complex32::new(1.0, 0.0), Complex32::new(0.0, 0.0))
    }
}

/// Bound on `|k1|` in [`cancel_weight`]'s rescaled output: a 60 dB gain
/// ratio between two aerials is already implausible for any real pairing,
/// so treating anything past it as numerical noise rather than a real
/// solve costs nothing genuine.
const MAX_CANCEL_RATIO: f32 = 1.0e3;

// -----------------------------------------------------------------------
// Whitening -- a real gap [`Diversity::process_decorrelate`] had against two
// independent, real-world implementations (SDR++ and the Perseus22's own
// vendor software), both of which get a materially cleaner null on
// identical antennas than the un-whitened solve below did. Ported from the
// SDR++ sibling's own `dsp::combine` (`decorrelator.h`'s `inverseSqrt`/
// `transform`/`combineCoefficients`, `phaser.h`'s `captureNoise`), whose own
// doc names the mechanism plainly: solving for maximum power favours
// whichever channel is noisiest unless the two are first normalised to
// equal, uncorrelated noise -- "whitened". [`covariance_eigen`] has no
// equivalent; it solves the raw covariance directly, which is fine when the
// two channels already have comparable noise floors and biased otherwise.
//
// f64 throughout, and a second, un-conjugated eigendecomposition
// ([`raw_eigen`]) rather than reusing [`covariance_eigen`]: building a
// whitening matrix needs the actual eigenvectors, not that function's own
// f32/"already conjugated, ready to apply" convention, and round-tripping
// through the conjugate twice per calibration is more room for a sign error
// than a second, independently-verified, closed-form solve costs.
// -----------------------------------------------------------------------

/// A 2×2 complex matrix, row-major — the whitening transform's own shape.
/// Built once per calibration, not per-sample, so f64 costs nothing real.
#[derive(Debug, Clone, Copy)]
struct Matrix2 {
    m: [[num_complex::Complex64; 2]; 2],
}

/// Unit-norms a raw (2-component) eigenvector — [`raw_eigen`]'s own helper,
/// not [`Eigenpair`]'s: this one has no conjugate applied and no eigenvalue
/// attached, since [`Matrix2`] construction needs the vector itself.
fn normalize2(mut v: [num_complex::Complex64; 2]) -> [num_complex::Complex64; 2] {
    let n = (v[0].norm_sqr() + v[1].norm_sqr()).sqrt();
    if n > 1e-300 {
        v[0] /= n;
        v[1] /= n;
    } else {
        v = [num_complex::Complex64::new(1.0, 0.0), num_complex::Complex64::new(0.0, 0.0)];
    }
    v
}

/// The same closed-form 2×2 Hermitian solve [`covariance_eigen`] does, in
/// f64 and returning the *raw*, un-conjugated, unit-norm eigenvectors —
/// what building or applying a whitening transform needs. Returns
/// `(λ_min, u_min, λ_max, u_max)`.
fn raw_eigen(
    raa: f64,
    rbb: f64,
    rab: num_complex::Complex64,
) -> (f64, [num_complex::Complex64; 2], f64, [num_complex::Complex64; 2]) {
    let trace = raa + rbb;
    let det = raa * rbb - rab.norm_sqr();
    let disc = (trace * trace - 4.0 * det).max(0.0).sqrt();
    let lambda_max = 0.5 * (trace + disc);
    let lambda_min = 0.5 * (trace - disc);

    let one = num_complex::Complex64::new(1.0, 0.0);
    let zero = num_complex::Complex64::new(0.0, 0.0);
    let (u_min, u_max) = if rab.norm_sqr() > 1e-300 {
        (
            normalize2([rab, num_complex::Complex64::new(lambda_min - raa, 0.0)]),
            normalize2([rab, num_complex::Complex64::new(lambda_max - raa, 0.0)]),
        )
    } else if raa <= rbb {
        ([one, zero], [zero, one])
    } else {
        ([zero, one], [one, zero])
    };
    (lambda_min, u_min, lambda_max, u_max)
}

/// `R^(-1/2)`, for whitening. Applied to a channel pair it produces two
/// outputs of equal power that are uncorrelated — the manual's own
/// "orthonormalisation". Calibrated on noise-only data (see
/// [`Diversity::capture_noise`]), it is what turns a maximum-power
/// combination into a maximum-SNR one: without it, combining for power
/// alone favours whichever channel is noisiest.
fn inverse_sqrt(raa: f64, rbb: f64, rab: num_complex::Complex64) -> Matrix2 {
    let (lambda_min, u_min, lambda_max, u_max) = raw_eigen(raa.max(0.0), rbb.max(0.0), rab);
    let s_min = 1.0 / lambda_min.max(1e-300).sqrt();
    let s_max = 1.0 / lambda_max.max(1e-300).sqrt();
    let mut m = [[num_complex::Complex64::new(0.0, 0.0); 2]; 2];
    for (r, row) in m.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = s_max * u_max[r] * u_max[c].conj() + s_min * u_min[r] * u_min[c].conj();
        }
    }
    Matrix2 { m }
}

/// `R' = W·R·Wᴴ` — a covariance measured on the raw channels, expressed in
/// whitened coordinates, without touching a single sample.
///
/// A genuine 2×2 matrix multiply, where each output element reads from both
/// input matrices at different indices — an iterator over one side's own
/// elements has nothing to enumerate against the other side, so a plain
/// index loop is the clearer way to write it, not a range clippy would
/// rather see turned into one.
#[allow(clippy::needless_range_loop)]
fn transform(raa: f64, rbb: f64, rab: num_complex::Complex64, w: &Matrix2) -> (f64, f64, num_complex::Complex64) {
    let r = [[num_complex::Complex64::new(raa, 0.0), rab], [rab.conj(), num_complex::Complex64::new(rbb, 0.0)]];
    let mut wr = [[num_complex::Complex64::new(0.0, 0.0); 2]; 2];
    for i in 0..2 {
        for j in 0..2 {
            wr[i][j] = w.m[i][0] * r[0][j] + w.m[i][1] * r[1][j];
        }
    }
    let mut out = [[num_complex::Complex64::new(0.0, 0.0); 2]; 2];
    for i in 0..2 {
        for j in 0..2 {
            out[i][j] = wr[i][0] * w.m[j][0].conj() + wr[i][1] * w.m[j][1].conj();
        }
    }
    (out[0][0].re, out[1][1].re, out[0][1])
}

/// A whitened-space eigenvector `u`, folded back into coefficients for the
/// *raw* channels: `y = k0·main + k1·aux`. Whitening never touches a
/// sample — it only reshapes the covariance the solve runs on — so the
/// transform has to be folded into the coefficients instead. For a
/// Hermitian `W` (which [`inverse_sqrt`] always produces), `conj(u)·W` and
/// `conj(W^H·u)` are the same value — the identity the SDR++ sibling's own
/// `combineCoefficients(..., whitened=true, ...)` uses, verified against
/// this file's own [`covariance_eigen`] convention (`k0`/`k1` already
/// conjugated, ready to apply directly) rather than assumed to match it.
fn whitened_to_raw(u: [num_complex::Complex64; 2], w: &Matrix2) -> (Complex32, Complex32) {
    let cu = [u[0].conj(), u[1].conj()];
    let k0 = cu[0] * w.m[0][0] + cu[1] * w.m[1][0];
    let k1 = cu[0] * w.m[0][1] + cu[1] * w.m[1][1];
    (Complex32::new(k0.re as f32, k0.im as f32), Complex32::new(k1.re as f32, k1.im as f32))
}

/// A noise-only calibration in progress — see [`Diversity::capture_noise`].
/// Accumulates the same `raa`/`rbb`/`rab` a normal solve would, over
/// [`Self::samples_wanted`] samples, then finalises into
/// [`Diversity::whitening`] and clears itself.
struct NoiseCapture {
    raa: f64,
    rbb: f64,
    rab: num_complex::Complex64,
    terms: u64,
    samples_seen: u64,
    samples_wanted: u64,
}

/// Slowest and fastest normalised step size the rate control reaches.
///
/// The bottom is a filter that takes tens of seconds to settle and then sits
/// still — what a stationary noise source wants. The top converges inside a
/// fraction of a second and visibly hunts, which is what makes it useful for
/// *finding* the null before freezing it. Above about 1.0 an NLMS filter is no
/// longer guaranteed stable, so the ceiling is well short of it.
const MU_MIN: f32 = 1.0e-4;
const MU_MAX: f32 = 0.5;

/// Keeps the normalisation finite on a dead auxiliary channel.
const EPS: f32 = 1.0e-12;

/// A weight vector longer than the signal it is fitting will wander; a
/// divergent one is worse, because it turns the output into noise louder than
/// the input. Anything past this is a reset rather than a slow recovery.
const MAX_WEIGHT_ENERGY: f32 = 1.0e6;

/// One over the time constant, in samples, of the input-power average that
/// normalises the step size. About four thousand samples: long enough to
/// average out a modulation envelope, short enough to follow a gain change.
const POWER_TAU_INV: f32 = 1.0 / 4096.0;

/// How often, in samples, the combining weight `‖W‖²` is recomputed inside a
/// block. Short enough that a filter converging mid-block is followed, long
/// enough that the recomputation is not part of the per-sample cost.
const WNORM_INTERVAL: usize = 256;

/// How much of the previous block's power estimate survives into the next one.
/// Only the reported depth depends on this, not the filter.
const POWER_DECAY: f32 = 0.9;

/// Accumulates the correlations [`Diversity::process_decorrelate`] needs —
/// `Σ main·conj(aux)`, `Σ|aux|²`, `Σ|main|²` — optionally restricted to a
/// slice of spectrum rather than the whole received span. Ported from the
/// SDR++ sibling implementation's own `dsp::combine::RefBand`
/// (`core/src/dsp/combine/ref_band.h`), which is where this whole idea comes
/// from — its own comment is worth keeping verbatim as the reason it exists:
///
/// > Minimising total output power nulls whatever is loudest, which when the
/// > DX peaks is the DX. Pointing the adaptation at a stretch of spectrum
/// > containing only the interferer... lets the weight be solved from the
/// > pest alone and then applied to the whole band. It is the thing an SDR
/// > can do that an analogue phasing box cannot.
///
/// Real-air evidence this matters, not just theory: whole-span
/// `covariance_eigen` (no restriction) left far more of a real interferer
/// (WNYC, 820 kHz) audible than the SDR++ sibling's own reference-band-
/// restricted solve on the identical antennas and frequency — see the
/// commit that added this struct.
///
/// The restriction is two cascaded boxcar decimators, not a designed filter
/// — matching the original's own reasoning: a sharp filter at these ratios
/// would need hundreds of taps at the full sample rate, while a two-stage
/// cascade costs about one add per sample and gives a triangular response
/// with roughly −26 dB sidelobes, ample for weighting the correlation toward
/// a chosen part of the spectrum. This is not a channel filter — nearby
/// strong signals still pull on the estimate somewhat.
///
/// Both channels are mixed by the *same* rotation and decimated by the
/// *same* cascade, so their relative phase — the entire quantity being
/// measured — survives untouched.
struct RefBand {
    sample_rate_hz: f64,
    rot: num_complex::Complex64,
    rot_step: num_complex::Complex64,
    acc1a: num_complex::Complex64,
    acc1b: num_complex::Complex64,
    acc2a: num_complex::Complex64,
    acc2b: num_complex::Complex64,
    len1: usize,
    len2: usize,
    n1: usize,
    n2: usize,
}

impl Default for RefBand {
    fn default() -> Self {
        RefBand {
            sample_rate_hz: 1.0,
            rot: num_complex::Complex64::new(1.0, 0.0),
            rot_step: num_complex::Complex64::new(1.0, 0.0),
            acc1a: num_complex::Complex64::new(0.0, 0.0),
            acc1b: num_complex::Complex64::new(0.0, 0.0),
            acc2a: num_complex::Complex64::new(0.0, 0.0),
            acc2b: num_complex::Complex64::new(0.0, 0.0),
            len1: 1,
            len2: 1,
            n1: 0,
            n2: 0,
        }
    }
}

impl RefBand {
    /// `offset_hz` is where the reference band sits relative to the centre
    /// of whatever `main`/`aux` are baseband IQ around (positive = above
    /// centre); `width_hz` is its nominal width — the cascade's decimation
    /// is chosen so the first null of its response sits at
    /// `sample_rate_hz / D`, i.e. `D ≈ sample_rate_hz / width_hz`.
    fn configure(&mut self, sample_rate_hz: f64, offset_hz: f64, width_hz: f64) {
        self.sample_rate_hz = sample_rate_hz;
        let inc = -2.0 * std::f64::consts::PI * offset_hz / sample_rate_hz.max(1.0);
        self.rot_step = num_complex::Complex64::from_polar(1.0, inc);

        let w = width_hz.max(1.0);
        let d = (sample_rate_hz / w).round().clamp(1.0, 65536.0) as usize;
        self.len1 = d.min(32);
        self.len2 = (d / self.len1.max(1)).max(1);
        self.reset();
    }

    fn reset(&mut self) {
        self.rot = num_complex::Complex64::new(1.0, 0.0);
        self.acc1a = num_complex::Complex64::new(0.0, 0.0);
        self.acc1b = num_complex::Complex64::new(0.0, 0.0);
        self.acc2a = num_complex::Complex64::new(0.0, 0.0);
        self.acc2b = num_complex::Complex64::new(0.0, 0.0);
        self.n1 = 0;
        self.n2 = 0;
    }

    /// Correlation restricted to the configured band, added into
    /// `raa`/`rbb`/`rab`. State persists across calls so the decimator does
    /// not restart mid-stream; returns how many decimated terms this call
    /// contributed, so a caller normalising by term count (as
    /// [`covariance_eigen`] wants) knows the true denominator — a narrow
    /// band yields far fewer terms than samples fed in, and a block shorter
    /// than the cascade's own period can legitimately contribute zero.
    fn accumulate(
        &mut self,
        main: &[Complex32],
        aux: &[Complex32],
        raa: &mut f64,
        rbb: &mut f64,
        rab: &mut num_complex::Complex64,
    ) -> usize {
        let mut terms = 0;
        for (a, b) in main.iter().zip(aux.iter()) {
            // One rotation, applied to both channels, so the relative phase
            // survives.
            let av = num_complex::Complex64::new(f64::from(a.re), f64::from(a.im)) * self.rot;
            let bv = num_complex::Complex64::new(f64::from(b.re), f64::from(b.im)) * self.rot;
            self.rot *= self.rot_step;

            self.acc1a += av;
            self.acc1b += bv;
            self.n1 += 1;
            if self.n1 < self.len1 {
                continue;
            }
            let y1a = self.acc1a / self.len1 as f64;
            let y1b = self.acc1b / self.len1 as f64;
            self.acc1a = num_complex::Complex64::new(0.0, 0.0);
            self.acc1b = num_complex::Complex64::new(0.0, 0.0);
            self.n1 = 0;

            self.acc2a += y1a;
            self.acc2b += y1b;
            self.n2 += 1;
            if self.n2 < self.len2 {
                continue;
            }
            let y2a = self.acc2a / self.len2 as f64;
            let y2b = self.acc2b / self.len2 as f64;
            self.acc2a = num_complex::Complex64::new(0.0, 0.0);
            self.acc2b = num_complex::Complex64::new(0.0, 0.0);
            self.n2 = 0;

            *rab += y2a * y2b.conj();
            *rbb += y2b.norm_sqr();
            *raa += y2a.norm_sqr();
            terms += 1;
        }
        // Keep the mixer on the unit circle.
        let m = self.rot.norm();
        if m > 0.0 {
            self.rot /= m;
        }
        terms
    }
}

pub struct Diversity {
    mode: DiversityMode,
    algorithm: DiversityAlgorithm,
    mu: f32,
    frozen: bool,
    /// The filter, `w[0]` multiplying the newest auxiliary sample.
    w: Vec<Complex32>,
    /// The auxiliary delay line, written at [`Self::pos`].
    hist: Vec<Complex32>,
    pos: usize,
    /// `Σ|aux|²` over the delay line, maintained incrementally: recomputing it
    /// per sample would double the work.
    energy: f32,
    /// [`Self::energy`] smoothed over [`POWER_TAU`] samples, and the actual
    /// step-size normaliser — see the note on the update in [`Self::process`].
    pow: f32,
    in_pow: f32,
    out_pow: f32,
    /// The last weight [`Self::process_decorrelate`] solved — held across
    /// blocks so a frozen decorrelator has something to reuse, the same
    /// meaning [`Self::frozen`] already has for `w` above.
    decorr_k0: Complex32,
    decorr_k1: Complex32,
    /// Distinguishes "never solved yet" from a genuinely all-zero weight.
    decorr_solved: bool,
    /// Restricts [`Self::process_decorrelate`]'s covariance measurement to a
    /// chosen slice of spectrum instead of the whole received span — see
    /// [`RefBand`]'s own doc for why this exists. Always present, only
    /// consulted when [`Self::ref_enabled`] is set — matching the SDR++
    /// sibling's own "always construct it, gate it with a flag" shape.
    ref_band: RefBand,
    ref_enabled: bool,
    /// The whitening transform [`Self::capture_noise`] calibrates, or
    /// `None` before that has ever been done — in which case
    /// [`Self::process_decorrelate`] solves the raw covariance directly,
    /// exactly as it always has.
    whitening: Option<Matrix2>,
    /// A calibration in progress — see [`Self::capture_noise`].
    noise_capture: Option<NoiseCapture>,
}

impl Diversity {
    /// `taps` is clamped to 1..=[`Self::MAX_TAPS`]; `rate` is the 0..1 control,
    /// not a step size — see [`Self::mu_for_rate`].
    pub fn new(mode: DiversityMode, taps: usize, rate: f32) -> Diversity {
        let taps = taps.clamp(1, Self::MAX_TAPS);
        Diversity {
            mode,
            algorithm: DiversityAlgorithm::Adaptive,
            mu: Self::mu_for_rate(rate),
            frozen: false,
            w: vec![Complex32::new(0.0, 0.0); taps],
            hist: vec![Complex32::new(0.0, 0.0); taps],
            pos: 0,
            energy: 0.0,
            pow: 0.0,
            in_pow: 0.0,
            out_pow: 0.0,
            // Unity on main, nothing from aux -- i.e. "do nothing yet",
            // not "null everything". A real bug this default's own history
            // has now hit twice: `(0, 0)` here means a `Diversity` asked to
            // start `frozen` before it has ever solved once -- a real
            // sequence, not hypothetical: settings persist `frozen: true`
            // from a previous session, and a fresh rebuild honors it from
            // the first block -- would multiply every sample by zero
            // forever, since being frozen is exactly what stops it from
            // ever getting to solve. Found live: Separate mode opened with
            // Hold already on produced total silence and a blank spectrum,
            // and turning Hold off (letting it solve at least once) fixed
            // it instantly. `decorr_solved` still distinguishes "never
            // solved" from "solved to a real answer" for anything that
            // reads it (`decorrelated_weight()`); this only changes what
            // gets *applied* to samples in the meantime.
            decorr_k0: Complex32::new(1.0, 0.0),
            decorr_k1: Complex32::new(0.0, 0.0),
            decorr_solved: false,
            ref_band: RefBand::default(),
            ref_enabled: false,
            whitening: None,
            noise_capture: None,
        }
    }

    /// Restrict [`DiversityAlgorithm::Decorrelate`]'s covariance solve to a
    /// slice of spectrum — see [`RefBand`]'s own doc for the mechanism and
    /// why it matters. `offset_hz` is where the reference band sits relative
    /// to the centre of the IQ [`Self::process`] receives (positive = above
    /// centre); `width_hz` is its nominal width. Disabling reverts to the
    /// original whole-span measurement; re-enabling (or changing the
    /// offset/width while enabled) resets the internal decimator, since its
    /// state has no meaning across a configuration change.
    pub fn set_ref_band(&mut self, enabled: bool, sample_rate_hz: f64, offset_hz: f64, width_hz: f64) {
        self.ref_enabled = enabled;
        if enabled {
            self.ref_band.configure(sample_rate_hz, offset_hz, width_hz);
        }
    }

    /// Arm a noise-only calibration: point the radio at a quiet channel
    /// first — whatever is on the air while this runs becomes "noise".
    /// The next `seconds` of covariance — measured the same way a normal
    /// solve is, respecting [`Self::set_ref_band`] if it is on — is taken
    /// as the receive chain's own noise floor and inverted into a
    /// whitening transform. After this, [`DiversityAlgorithm::Decorrelate`]
    /// solves in whitened coordinates: a maximum-power combination becomes
    /// a maximum-SNR one, correcting for whatever gain/noise-floor mismatch
    /// exists between the two channels instead of assuming they are
    /// already matched. See this module's own "Whitening" section for the
    /// mechanism and why it was added — two independent, real-world
    /// implementations (SDR++, and the Perseus22's own vendor software)
    /// got a materially cleaner null than the un-whitened solve did, on
    /// identical antennas.
    ///
    /// Capturing takes priority over [`Self::frozen`] — it is a deliberate,
    /// one-off operator action, not something Hold should be able to block
    /// — and leaves whatever was already being applied untouched while it
    /// runs, the same as being frozen already does.
    pub fn capture_noise(&mut self, seconds: f64, sample_rate_hz: f64) {
        let samples_wanted = (seconds.max(0.05) * sample_rate_hz).round().max(1.0) as u64;
        self.noise_capture = Some(NoiseCapture {
            raa: 0.0,
            rbb: 0.0,
            rab: num_complex::Complex64::new(0.0, 0.0),
            terms: 0,
            samples_seen: 0,
            samples_wanted,
        });
    }

    /// Whether a [`Self::capture_noise`] calibration is still accumulating.
    pub fn is_capturing_noise(&self) -> bool {
        self.noise_capture.is_some()
    }

    /// Whether [`Self::capture_noise`] has ever completed — `false` means
    /// [`Self::process_decorrelate`] is solving the raw, un-whitened
    /// covariance, exactly as it always has.
    pub fn has_whitening(&self) -> bool {
        self.whitening.is_some()
    }

    /// Discard a calibration (or one in progress) and go back to solving
    /// the raw covariance directly — the comparison this whole feature was
    /// motivated by needs an easy way back to the un-whitened baseline.
    pub fn clear_whitening(&mut self) {
        self.whitening = None;
        self.noise_capture = None;
    }

    /// This block's `raa`/`rbb`/`rab`, either the whole-span raw sum or
    /// [`Self::ref_band`]'s own restricted measurement — the same
    /// accumulation a normal solve and [`Self::capture_noise`] both want,
    /// so there is one implementation of "measure this block" rather than
    /// two that could quietly drift apart.
    fn measure_covariance(
        &mut self,
        main: &[Complex32],
        aux: &[Complex32],
    ) -> (f64, f64, num_complex::Complex64, usize) {
        if self.ref_enabled {
            let mut raa = 0.0f64;
            let mut rbb = 0.0f64;
            let mut rab = num_complex::Complex64::new(0.0, 0.0);
            let terms = self.ref_band.accumulate(main, aux, &mut raa, &mut rbb, &mut rab);
            (raa, rbb, rab, terms)
        } else {
            let mut raa = 0.0f64;
            let mut rbb = 0.0f64;
            let mut rab = num_complex::Complex64::new(0.0, 0.0);
            let n = main.len().min(aux.len());
            for i in 0..n {
                raa += f64::from(main[i].norm_sqr());
                rbb += f64::from(aux[i].norm_sqr());
                rab += num_complex::Complex64::new(f64::from(main[i].re), f64::from(main[i].im))
                    * num_complex::Complex64::new(f64::from(aux[i].re), -f64::from(aux[i].im));
            }
            (raa, rbb, rab, n)
        }
    }

    /// The longest filter this will build. At the top sample rates it is
    /// already more arithmetic than the rest of the receive chain put
    /// together, and a filter this long is equalising a path difference no
    /// pair of aerials on one site has. Kept in step with
    /// `sdroxide_types::LimeAuxConfig::MAX_TAPS`, which is what the settings
    /// panel bounds its control by; this one is the backstop.
    pub const MAX_TAPS: usize = 64;

    /// The 0..1 rate control mapped to a normalised step size, logarithmically
    /// — the useful range spans three and a half decades, and a linear slider
    /// would spend nine tenths of its travel in the part that hunts.
    pub fn mu_for_rate(rate: f32) -> f32 {
        let r = rate.clamp(0.0, 1.0);
        MU_MIN * (MU_MAX / MU_MIN).powf(r)
    }

    pub fn mode(&self) -> DiversityMode {
        self.mode
    }

    /// Changing mode keeps the filter: it is the same estimate either way, and
    /// throwing away a converged one to answer a switch would cost the operator
    /// the convergence they just watched happen.
    pub fn set_mode(&mut self, mode: DiversityMode) {
        self.mode = mode;
    }

    pub fn algorithm(&self) -> DiversityAlgorithm {
        self.algorithm
    }

    /// Switching does *not* reset either state: `w` (the adaptive filter) and
    /// `decorr_k0`/`decorr_k1` (the decorrelator) are unrelated
    /// representations, so crossing between them just leaves whichever one
    /// is not currently selected alone, unused, wherever it was.
    pub fn set_algorithm(&mut self, algorithm: DiversityAlgorithm) {
        self.algorithm = algorithm;
    }

    pub fn taps(&self) -> usize {
        self.w.len()
    }

    /// Resize the filter, which necessarily starts it again — the taps mean
    /// different delays now.
    pub fn set_taps(&mut self, taps: usize) {
        let taps = taps.clamp(1, Self::MAX_TAPS);
        if taps == self.w.len() {
            return;
        }
        self.w = vec![Complex32::new(0.0, 0.0); taps];
        self.hist = vec![Complex32::new(0.0, 0.0); taps];
        self.pos = 0;
        self.energy = 0.0;
        self.pow = 0.0;
    }

    pub fn set_rate(&mut self, rate: f32) {
        self.mu = Self::mu_for_rate(rate);
    }

    /// Stop adapting, holding the filter where it is. The whole point of the
    /// control: a null found while the band was quiet is worth keeping when it
    /// is not.
    pub fn set_frozen(&mut self, frozen: bool) {
        self.frozen = frozen;
    }

    pub fn frozen(&self) -> bool {
        self.frozen
    }

    /// Zero the filter and start again — the answer when the aerials, the band
    /// or the noise source have changed under a frozen null.
    pub fn reset(&mut self) {
        self.w.fill(Complex32::new(0.0, 0.0));
        self.hist.fill(Complex32::new(0.0, 0.0));
        self.pos = 0;
        self.energy = 0.0;
        self.pow = 0.0;
        self.in_pow = 0.0;
        self.out_pow = 0.0;
        // Unity on main, nothing from aux -- see `Self::new`'s own doc on
        // this exact pair of values: `(0, 0)` here means "Restart" while
        // frozen would silence everything until Hold is turned off, the
        // same real bug construction-time freezing had.
        self.decorr_k0 = Complex32::new(1.0, 0.0);
        self.decorr_k1 = Complex32::new(0.0, 0.0);
        self.decorr_solved = false;
    }

    /// The weight [`Self::process_decorrelate`] last solved, `(k0, k1)` such
    /// that `y = k0·main + k1·aux` — `None` when [`Self::algorithm`] is not
    /// [`DiversityAlgorithm::Decorrelate`] or nothing has been solved yet.
    /// What a hardware combiner (an RSR200's, say) would read to turn a
    /// software solve into a command of its own.
    pub fn decorrelated_weight(&self) -> Option<(Complex32, Complex32)> {
        if self.algorithm != DiversityAlgorithm::Decorrelate || !self.decorr_solved {
            return None;
        }
        Some((self.decorr_k0, self.decorr_k1))
    }

    /// How much quieter the output is than the main channel was, in dB.
    ///
    /// In `Cancel` this is the null depth, and it is the number to watch: a
    /// converged canceller on a real noise source reads 15–30 dB, and one
    /// reading a fraction of a decibel is one whose auxiliary aerial cannot
    /// hear what it is being asked to subtract. In `Combine` it is not a
    /// figure of merit at all — the output is meant to be about as loud as the
    /// input — so it is reported only for the one mode that means something.
    pub fn depth_db(&self) -> Option<f32> {
        if self.mode != DiversityMode::Cancel || self.in_pow <= 0.0 || self.out_pow <= 0.0 {
            return None;
        }
        Some(10.0 * (self.in_pow / self.out_pow).log10())
    }

    /// Combine one block. `main` is replaced by the result; `aux` must be the
    /// **same samples in time** from the second chain, which is the caller's
    /// side of the bargain — a block pair that is not sample-aligned produces a
    /// filter that fits a delay that is not there.
    ///
    /// A short `aux` processes only the overlap and leaves the rest of `main`
    /// as it came, which is the right answer for a chain that has stalled: no
    /// cancellation beats cancellation against the wrong samples.
    pub fn process(&mut self, main: &mut [Complex32], aux: &[Complex32]) {
        if self.algorithm == DiversityAlgorithm::Decorrelate {
            self.process_decorrelate(main, aux);
            return;
        }
        let n = main.len().min(aux.len());
        if n == 0 {
            return;
        }
        let taps = self.w.len();
        let mut wnorm = 0.0f32;
        let mut in_acc = 0.0f32;
        let mut out_acc = 0.0f32;

        let mut pos = self.pos;
        for i in 0..n {
            // `‖W‖²` is only needed by the combining arithmetic, and it moves
            // at the adaptation's pace — thousands of samples slow. Refreshing
            // it every [`WNORM_INTERVAL`] samples rather than every one keeps
            // it a rounding error on the cost, while still tracking a filter
            // that converges inside a block. Doing it once per *block* would
            // not: a caller that hands over a whole second at a time would
            // combine the entire first block against a weight of zero.
            if i % WNORM_INTERVAL == 0 {
                wnorm = self.w.iter().map(|w| w.norm_sqr()).sum();
                if !wnorm.is_finite() || wnorm > MAX_WEIGHT_ENERGY {
                    // Diverged. The output would be louder than the input and
                    // made of nothing, so start again and let the rest of the
                    // block through untouched — silently, because a DSP block
                    // on the sample path has no business logging, and the
                    // caller is watching [`Self::depth_db`] anyway.
                    self.reset();
                    return;
                }
            }
            let x = aux[i];
            let m = main[i];
            // Slide the delay line one sample on, keeping its energy with it.
            let dropped = self.hist[pos];
            self.hist[pos] = x;
            self.energy += x.norm_sqr() - dropped.norm_sqr();
            if self.energy < 0.0 {
                // Rounding, over millions of samples of adds and subtracts.
                self.energy = self.hist.iter().map(|h| h.norm_sqr()).sum();
            }
            let newest = pos;
            pos = if pos + 1 == taps { 0 } else { pos + 1 };

            let mut y = Complex32::new(0.0, 0.0);
            for k in 0..taps {
                let idx = (newest + taps - k) % taps;
                y += self.w[k] * self.hist[idx];
            }
            let e = m - y;

            // Normalised by the *smoothed* input power rather than this
            // sample's, and that is not a refinement — it is what makes the
            // filter converge on the right answer. Dividing by the
            // instantaneous energy, as textbook NLMS does, weights every
            // sample by `1/|aux|²`, so the quiet samples dominate the average
            // and the filter settles somewhere other than the least-squares
            // fit; on two aerials hearing one station that showed up as a
            // combining weight half again too big. The `max` keeps the step
            // no larger than plain NLMS would have taken, so a burst cannot
            // make it unstable.
            self.pow += (self.energy - self.pow) * POWER_TAU_INV;
            if !self.frozen {
                let step = self.mu / (self.pow.max(self.energy) + EPS);
                for k in 0..taps {
                    let idx = (newest + taps - k) % taps;
                    self.w[k] += e * self.hist[idx].conj() * step;
                }
            }

            main[i] = match self.mode {
                DiversityMode::Cancel => e,
                // Maximal ratio: each branch weighted by the inverse of its
                // noise power, with `W` carrying the scale between them.
                DiversityMode::Combine => (m * wnorm + y) / (1.0 + wnorm),
            };
            in_acc += m.norm_sqr();
            out_acc += main[i].norm_sqr();
        }

        self.pos = pos;
        let inv = 1.0 / n as f32;
        self.in_pow = self.in_pow * POWER_DECAY + in_acc * inv * (1.0 - POWER_DECAY);
        self.out_pow = self.out_pow * POWER_DECAY + out_acc * inv * (1.0 - POWER_DECAY);
    }

    /// The [`DiversityAlgorithm::Decorrelate`] half of [`Self::process`].
    /// Unlike the adaptive filter, there is nothing to converge: one
    /// covariance matrix over the whole block, solved once, applied
    /// uniformly to every sample in it. While [`Self::frozen`] the solve is
    /// skipped and the last weight reused instead — the same meaning
    /// freezing already has for the adaptive filter's `w`.
    fn process_decorrelate(&mut self, main: &mut [Complex32], aux: &[Complex32]) {
        let n = main.len().min(aux.len());
        if n == 0 {
            return;
        }

        if self.noise_capture.is_some() {
            // Measured first, before the mutable borrow of `noise_capture`
            // below -- `measure_covariance` needs `&mut self.ref_band`,
            // which would conflict with holding `noise_capture` mutably at
            // the same time.
            let (raa, rbb, rab, terms) = self.measure_covariance(&main[..n], &aux[..n]);
            let capture = self.noise_capture.as_mut().expect("checked above");
            capture.raa += raa;
            capture.rbb += rbb;
            capture.rab += rab;
            capture.terms += terms as u64;
            capture.samples_seen += n as u64;
            if capture.samples_seen >= capture.samples_wanted {
                if capture.terms > 0 {
                    let inv = 1.0 / capture.terms as f64;
                    self.whitening = Some(inverse_sqrt(capture.raa * inv, capture.rbb * inv, capture.rab * inv));
                }
                self.noise_capture = None;
            }
            // Calibrating is not itself a reason to change what is heard --
            // whatever weight was already solved (or the safe unity
            // default) stays exactly where it is, the same as `frozen`.
        } else if !self.frozen {
            let (raa, rbb, rab, terms) = self.measure_covariance(&main[..n], &aux[..n]);
            // A block shorter than the reference band's own decimator period
            // legitimately produces zero terms -- nothing to solve from yet,
            // not a reason to solve on all-zero and null the whole signal.
            // Skipping leaves whatever was last solved (or nothing, if this
            // is the first block) exactly as `frozen` already does.
            if terms > 0 {
                let inv = 1.0 / terms as f64;
                let (raa, rbb, rab) = (raa * inv, rbb * inv, rab * inv);

                let (k0, k1) = if let Some(w) = self.whitening {
                    // Solved in whitened coordinates -- see this module's
                    // own "Whitening" section -- then folded back into
                    // coefficients for the raw channels.
                    let (raa_w, rbb_w, rab_w) = transform(raa, rbb, rab, &w);
                    let (lambda_min, u_min, _lambda_max, u_max) = raw_eigen(raa_w.max(0.0), rbb_w.max(0.0), rab_w);
                    match self.mode {
                        DiversityMode::Cancel => {
                            let (k0, k1) = whitened_to_raw(u_min, &w);
                            cancel_weight(Eigenpair { eigenvalue: lambda_min as f32, k0, k1 })
                        }
                        DiversityMode::Combine => whitened_to_raw(u_max, &w),
                    }
                } else {
                    let raa = raa as f32;
                    let rbb = rbb as f32;
                    let rab = Complex32::new(rab.re as f32, rab.im as f32);
                    let (null, combine) = covariance_eigen(raa, rbb, rab);
                    match self.mode {
                        // The raw null eigenvector is jointly unit-norm across
                        // *both* channels, so applied directly it scales `main`
                        // itself by less than one -- unlike the adaptive filter's
                        // `main - W*aux`, which leaves a signal `aux` cannot hear
                        // untouched by construction. Rescaling to unity gain on
                        // `main` restores that guarantee -- see `cancel_weight`.
                        DiversityMode::Cancel => cancel_weight(null),
                        // Combining has no such expectation -- both branches are
                        // meant to be weighted by quality, `main` included -- so
                        // the raw maximal-ratio eigenvector is applied as solved.
                        DiversityMode::Combine => (combine.k0, combine.k1),
                    }
                };
                self.decorr_k0 = k0;
                self.decorr_k1 = k1;
                self.decorr_solved = true;
            }
        }

        let mut in_acc = 0.0f32;
        let mut out_acc = 0.0f32;
        for i in 0..n {
            let m = main[i];
            main[i] = self.decorr_k0 * m + self.decorr_k1 * aux[i];
            in_acc += m.norm_sqr();
            out_acc += main[i].norm_sqr();
        }
        let inv = 1.0 / n as f32;
        self.in_pow = self.in_pow * POWER_DECAY + in_acc * inv * (1.0 - POWER_DECAY);
        self.out_pow = self.out_pow * POWER_DECAY + out_acc * inv * (1.0 - POWER_DECAY);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A deterministic complex noise source, so a failure is reproducible.
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

    /// A pure complex exponential at `freq_hz`, sampled at `sample_rate_hz` —
    /// a synthetic "one carrier, one frequency" signal to correlate against.
    fn tone(n: usize, freq_hz: f64, sample_rate_hz: f64) -> Vec<Complex32> {
        let inc = 2.0 * std::f64::consts::PI * freq_hz / sample_rate_hz;
        (0..n)
            .map(|i| {
                let (s, c) = (inc * i as f64).sin_cos();
                Complex32::new(c as f32, s as f32)
            })
            .collect()
    }

    /// How much of `reference` (a known pure tone) is present in `signal`, in
    /// amplitude — a matched-filter correlation, which isolates one tone's
    /// own contribution even when `signal` also carries other, sufficiently
    /// far-apart frequencies (their own correlation against `reference`
    /// averages toward zero over enough samples).
    fn tone_amplitude(signal: &[Complex32], reference: &[Complex32]) -> f32 {
        let n = signal.len().min(reference.len());
        let sum: Complex32 =
            signal[..n].iter().zip(&reference[..n]).map(|(&s, &r)| s * r.conj()).sum();
        (sum / n as f32).norm()
    }

    /// The case the mode exists for: one noise source reaching two aerials
    /// through different paths — a complex gain *and* a delay, which is what a
    /// single-weight canceller cannot follow across a span.
    ///
    /// Measured on the tail of the run, after convergence, and on the
    /// interference alone: what matters is how much of the noise is left, not
    /// how much total power went away.
    #[test]
    fn a_delayed_and_rotated_noise_source_is_nulled() {
        let n = 200_000;
        let qrm = noise(n, 0x1234);
        // The path from the noise to the main aerial: 3 samples further and
        // 0.6 of the amplitude, rotated 40°.
        let h = Complex32::from_polar(0.6, 0.7);
        let delay = 3usize;
        let mut main: Vec<Complex32> = (0..n)
            .map(|i| if i >= delay { qrm[i - delay] * h } else { Complex32::new(0.0, 0.0) })
            .collect();

        let mut d = Diversity::new(DiversityMode::Cancel, 8, 0.8);
        d.process(&mut main, &qrm);

        let tail = &main[n - 20_000..];
        let before: Vec<Complex32> = (n - 20_000..n).map(|i| qrm[i - delay] * h).collect();
        let depth = 10.0 * (power(&before) / power(tail)).log10();
        assert!(depth > 30.0, "only {depth:.1} dB of the noise source was removed");
    }

    /// The real-air case whitening exists for, reproduced synthetically: two
    /// antennas hearing the same interferer, but one channel's own front-end
    /// noise is much louder than the other's — exactly a real gain/noise-
    /// floor mismatch between two different aerials, not anything about the
    /// interferer itself. Without whitening, the raw covariance solve is
    /// dominated by whichever channel is loudest overall (mostly its own
    /// noise here, not the correlated interferer) and finds essentially no
    /// useful null. Calibrating on noise-only data with the same mismatch
    /// (what actually tuning to a quiet channel first gives) and solving in
    /// the resulting whitened coordinates finds the interferer's *true*
    /// gain/phase relationship almost exactly and nulls deep — matching what
    /// SDR++ and the Perseus22's own vendor software get on real antennas
    /// that sdroxide's un-whitened solve did not.
    #[test]
    fn whitening_finds_the_null_a_channel_noise_floor_mismatch_hides() {
        let n = 200_000;
        let qrm = noise(n, 0xA1);
        // The path from the interferer to main: 0.6 of the amplitude,
        // rotated 40°. Aux hears it at unity gain.
        let h = Complex32::from_polar(0.6, 0.7);
        // Aux's own front end is thirty times noisier than main's -- a real
        // mismatch, not a subtle one.
        let noise_main: Vec<Complex32> = noise(n, 0xB2).iter().map(|s| s * 0.1).collect();
        let noise_aux: Vec<Complex32> = noise(n, 0xC3).iter().map(|s| s * 3.0).collect();
        let build_main = |nm: &[Complex32]| -> Vec<Complex32> { (0..n).map(|i| qrm[i] * h + nm[i]).collect() };
        let build_aux = |na: &[Complex32]| -> Vec<Complex32> { (0..n).map(|i| qrm[i] + na[i]).collect() };

        let mut d_raw = Diversity::new(DiversityMode::Cancel, 1, 0.8);
        d_raw.set_algorithm(DiversityAlgorithm::Decorrelate);
        let mut main_raw = build_main(&noise_main);
        d_raw.process(&mut main_raw, &build_aux(&noise_aux));

        let mut d_w = Diversity::new(DiversityMode::Cancel, 1, 0.8);
        d_w.set_algorithm(DiversityAlgorithm::Decorrelate);
        d_w.capture_noise(1.0, n as f64);
        // Calibration is a fresh realisation of the same per-channel noise
        // mismatch, not a replay of `noise_main`/`noise_aux` -- what tuning
        // to a genuinely quiet channel actually gives.
        let mut calib_main: Vec<Complex32> = noise(n, 0xD4).iter().map(|s| s * 0.1).collect();
        let calib_aux: Vec<Complex32> = noise(n, 0xE5).iter().map(|s| s * 3.0).collect();
        d_w.process(&mut calib_main, &calib_aux);
        assert!(d_w.has_whitening(), "calibration did not complete in one block");

        let mut main_w = build_main(&noise(n, 0xF6).iter().map(|s| s * 0.1).collect::<Vec<_>>());
        d_w.process(&mut main_w, &build_aux(&noise(n, 0x17).iter().map(|s| s * 3.0).collect::<Vec<_>>()));

        // Matched-filter correlation against the known interferer waveform:
        // how much of it specifically remains, independent of how much of
        // aux's own much-louder noise also rides along in the output --
        // raw output power would conflate the two.
        let before: Vec<Complex32> = qrm.iter().map(|q| q * h).collect();
        let qrm_power = power(&before);
        let residual_db = |out: &[Complex32]| -> f32 {
            let corr: Complex32 = out.iter().zip(&before).map(|(&o, &b)| o * b.conj()).sum::<Complex32>() / n as f32;
            20.0 * (qrm_power / corr.norm()).log10()
        };
        let depth_raw = residual_db(&main_raw);
        let depth_w = residual_db(&main_w);
        assert!(depth_raw < 10.0, "raw solve should barely null under this mismatch, got {depth_raw:.1} dB");
        assert!(depth_w > 40.0, "whitened solve should null deep, got only {depth_w:.1} dB");
        assert!(
            depth_w > depth_raw + 30.0,
            "whitening should be a large, unambiguous improvement: raw {depth_raw:.1} dB, whitened {depth_w:.1} dB"
        );
    }

    /// The real-air case a reference band exists for, reproduced
    /// synthetically: two interferers reach both aerials, at different
    /// frequencies with different gain/phase relationships, and one is far
    /// stronger than the other. A whole-span [`DiversityAlgorithm::Decorrelate`]
    /// solve is dominated by whichever one contributes more covariance —
    /// the strong one — and does a mediocre job on the weak one, whatever it
    /// actually is. Restricting the solve to a band around the weak one
    /// (`RefBand`) targets it specifically and nulls it properly, regardless
    /// of what the strong one is doing elsewhere in the span. This is the
    /// synthetic version of what real air showed on 820 kHz the day this was
    /// added: sdroxide's whole-span Decorrelate left far more of a real
    /// interferer audible than the SDR++ sibling implementation's own
    /// reference-band-restricted solve on the identical antennas and
    /// frequency.
    #[test]
    fn a_reference_band_nulls_the_weak_interferer_the_whole_span_solve_misses() {
        let n = 40_000;
        let sr = 1_000_000.0;
        // Strong: 200 kHz, unity amplitude, gain (1.0 ∠ 0.3 rad) to aux.
        let strong = tone(n, 200_000.0, sr);
        let g_strong = Complex32::from_polar(1.0, 0.3);
        // Weak: -350 kHz, 5% the amplitude, a *different* gain/phase to aux --
        // a single compromise weight fit to the strong one does not also fit
        // this one.
        let weak = tone(n, -350_000.0, sr);
        let weak_amp = 0.05;
        let g_weak = Complex32::from_polar(1.0, -1.2);

        let build_main = || -> Vec<Complex32> {
            strong.iter().zip(&weak).map(|(&s, &w)| s + w * weak_amp).collect()
        };
        let build_aux = || -> Vec<Complex32> {
            strong.iter().zip(&weak).map(|(&s, &w)| s * g_strong + w * weak_amp * g_weak).collect()
        };

        let weak_before = tone_amplitude(&build_main(), &weak);
        assert!((weak_before - weak_amp).abs() < 0.005, "sanity: {weak_before} vs {weak_amp}");

        let mut whole_span_out = build_main();
        let mut d_whole = Diversity::new(DiversityMode::Cancel, 1, 0.8);
        d_whole.set_algorithm(DiversityAlgorithm::Decorrelate);
        d_whole.process(&mut whole_span_out, &build_aux());
        let weak_after_whole = tone_amplitude(&whole_span_out, &weak);

        let mut ref_band_out = build_main();
        let mut d_ref = Diversity::new(DiversityMode::Cancel, 1, 0.8);
        d_ref.set_algorithm(DiversityAlgorithm::Decorrelate);
        // 50 kHz wide, centred on the weak interferer -- the strong one at
        // +200 kHz is 550 kHz away, well outside this band's rolloff.
        d_ref.set_ref_band(true, sr, -350_000.0, 50_000.0);
        d_ref.process(&mut ref_band_out, &build_aux());
        let weak_after_ref = tone_amplitude(&ref_band_out, &weak);

        let depth_whole = 20.0 * (weak_amp / weak_after_whole).log10();
        let depth_ref = 20.0 * (weak_amp / weak_after_ref).log10();
        assert!(
            depth_ref > depth_whole + 15.0,
            "reference band ({depth_ref:.1} dB) should null the weak interferer much deeper \
             than the whole-span solve ({depth_whole:.1} dB), not comparably"
        );
        assert!(depth_ref > 25.0, "reference band only nulled the weak interferer {depth_ref:.1} dB");
    }

    /// And the wanted signal survives it, as long as the auxiliary aerial
    /// cannot hear it — which is the whole instruction for using this mode.
    #[test]
    fn a_signal_only_the_main_aerial_hears_survives_the_null() {
        let n = 200_000;
        let qrm = noise(n, 0xbeef);
        let h = Complex32::from_polar(0.8, -1.2);
        // A tone at a twelfth of the sample rate, on the main aerial only.
        let want: Vec<Complex32> = (0..n)
            .map(|i| Complex32::from_polar(0.3, std::f32::consts::TAU * i as f32 / 12.0))
            .collect();
        let mut main: Vec<Complex32> = (0..n).map(|i| want[i] + qrm[i] * h).collect();

        let mut d = Diversity::new(DiversityMode::Cancel, 8, 0.8);
        d.process(&mut main, &qrm);

        // Correlate the tail against the tone: how much of it came through.
        let tail = n - 20_000..n;
        let corr: Complex32 = tail
            .clone()
            .map(|i| main[i] * want[i].conj())
            .fold(Complex32::new(0.0, 0.0), |a, b| a + b);
        let kept = corr.norm() / (tail.len() as f32 * 0.3 * 0.3);
        assert!(kept > 0.9, "the wanted tone came through at {kept:.3} of its amplitude");
    }

    /// Two aerials on the same signal with independent noise: the textbook
    /// 3 dB, which is what says the branch weighting is right and not merely
    /// in phase.
    #[test]
    fn two_equal_branches_combine_for_about_three_decibels() {
        let n = 400_000;
        let sig = noise(n, 0xa11ce);
        let n1 = noise(n, 0x1111);
        let n2 = noise(n, 0x2222);
        // Equal amplitude, different phase — two aerials at different distances
        // from the same station.
        let h1 = Complex32::from_polar(1.0, 0.3);
        let h2 = Complex32::from_polar(1.0, -2.1);
        // About 10 dB per branch: enough that the filter's estimate of the
        // ratio between them is the ratio and not half of it. The formula's
        // 3 dB is the high-SNR limit, and a branch at 0 dB is a branch whose
        // ratio cannot be measured to better than a factor of two.
        let noise_amp = 0.316f32;
        let mut main: Vec<Complex32> = (0..n).map(|i| sig[i] * h1 + n1[i] * noise_amp).collect();
        let aux: Vec<Complex32> = (0..n).map(|i| sig[i] * h2 + n2[i] * noise_amp).collect();

        let mut d = Diversity::new(DiversityMode::Combine, 1, 0.6);
        d.process(&mut main, &aux);

        // Measure on the tail: project the output onto the signal, and call
        // what is left the noise.
        let tail = n - 100_000..n;
        let refs: Vec<Complex32> = tail.clone().map(|i| sig[i] * h1).collect();
        let out: Vec<Complex32> = tail.map(|i| main[i]).collect();
        let g: Complex32 = out
            .iter()
            .zip(&refs)
            .map(|(o, r)| o * r.conj())
            .fold(Complex32::new(0.0, 0.0), |a, b| a + b)
            / refs.iter().map(|r| r.norm_sqr()).sum::<f32>();
        let resid: Vec<Complex32> = out.iter().zip(&refs).map(|(o, r)| o - r * g).collect();
        let snr_out = 10.0 * ((g.norm_sqr() * power(&refs)) / power(&resid)).log10();
        // The same measurement on the main branch alone: its residual after
        // the signal is projected out is exactly its own noise.
        let own: Vec<Complex32> = (n - 100_000..n).map(|i| n1[i] * noise_amp).collect();
        let snr_in = 10.0 * (power(&refs) / power(&own)).log10();
        let gain = snr_out - snr_in;
        assert!(
            (2.5..3.5).contains(&gain),
            "combining two equal branches gained {gain:.2} dB, not about 3"
        );
    }

    /// Freezing holds the filter where it is, which is what makes a null found
    /// on a quiet band survive a loud one.
    #[test]
    fn freezing_stops_the_filter_moving() {
        let n = 40_000;
        let qrm = noise(n, 7);
        let mut main: Vec<Complex32> = qrm.iter().map(|q| q * 0.5).collect();
        let mut d = Diversity::new(DiversityMode::Cancel, 4, 0.9);
        d.process(&mut main, &qrm);
        let held: Vec<Complex32> = d.w.clone();

        d.set_frozen(true);
        let other = noise(n, 99);
        let mut main2: Vec<Complex32> = other.iter().map(|q| q * 3.0).collect();
        d.process(&mut main2, &other);
        assert_eq!(d.w, held, "the filter moved while frozen");
    }

    /// The rate control spans the documented range and is monotonic, because a
    /// slider that goes the wrong way at one end is worse than no slider.
    #[test]
    fn the_rate_control_spans_its_range_in_order() {
        assert!((Diversity::mu_for_rate(0.0) - MU_MIN).abs() < 1e-9);
        assert!((Diversity::mu_for_rate(1.0) - MU_MAX).abs() < 1e-6);
        let mut last = 0.0;
        for i in 0..=10 {
            let mu = Diversity::mu_for_rate(i as f32 / 10.0);
            assert!(mu > last, "rate {i} went backwards");
            last = mu;
        }
        // Out of range is clamped rather than extrapolated.
        assert_eq!(Diversity::mu_for_rate(-1.0), Diversity::mu_for_rate(0.0));
        assert_eq!(Diversity::mu_for_rate(9.0), Diversity::mu_for_rate(1.0));
    }

    /// A stalled auxiliary chain hands over fewer samples than the main one;
    /// the rest of the block must come through untouched rather than be
    /// cancelled against silence.
    #[test]
    fn a_short_auxiliary_block_passes_the_remainder_through() {
        let mut main: Vec<Complex32> = (0..16).map(|i| Complex32::new(i as f32, 0.0)).collect();
        let aux = vec![Complex32::new(1.0, 0.0); 4];
        let mut d = Diversity::new(DiversityMode::Cancel, 2, 0.5);
        d.process(&mut main, &aux);
        for i in 4..16 {
            assert_eq!(main[i], Complex32::new(i as f32, 0.0), "sample {i} was touched");
        }
    }

    // --- covariance_eigen: pure math, hand-computed answers -----------------

    /// An already-diagonal matrix (uncorrelated channels): the trivial basis,
    /// no division by `rab` involved at all.
    #[test]
    fn covariance_eigen_of_a_diagonal_matrix_is_the_trivial_basis() {
        let (lo, hi) = covariance_eigen(4.0, 9.0, Complex32::new(0.0, 0.0));
        assert!((lo.eigenvalue - 4.0).abs() < 1e-6);
        assert_eq!(lo.k0, Complex32::new(1.0, 0.0));
        assert_eq!(lo.k1, Complex32::new(0.0, 0.0));
        assert!((hi.eigenvalue - 9.0).abs() < 1e-6);
        assert_eq!(hi.k0, Complex32::new(0.0, 0.0));
        assert_eq!(hi.k1, Complex32::new(1.0, 0.0));

        // Same matrix, channels swapped: the null follows the quieter one.
        let (lo2, _) = covariance_eigen(9.0, 4.0, Complex32::new(0.0, 0.0));
        assert_eq!(lo2.k0, Complex32::new(0.0, 0.0));
        assert_eq!(lo2.k1, Complex32::new(1.0, 0.0));
    }

    /// Two equal-power channels correlated by a real 0.5: hand-computable by
    /// the textbook 2×2 formula, worked through in `DECORRELATION_PLAN.md`'s
    /// own commit message. `Var[(A−B)/√2] = (Var A + Var B − 2·Re[Cov])/2 =
    /// (1+1−1)/2 = 0.5`, which is exactly the smaller eigenvalue.
    #[test]
    fn covariance_eigen_matches_a_hand_computed_pair() {
        let (lo, hi) = covariance_eigen(1.0, 1.0, Complex32::new(0.5, 0.0));
        assert!((lo.eigenvalue - 0.5).abs() < 1e-6, "{lo:?}");
        assert!((hi.eigenvalue - 1.5).abs() < 1e-6, "{hi:?}");
        let s = std::f32::consts::FRAC_1_SQRT_2;
        assert!((lo.k0 - Complex32::new(s, 0.0)).norm() < 1e-5, "{lo:?}");
        assert!((lo.k1 - Complex32::new(-s, 0.0)).norm() < 1e-5, "{lo:?}");
        assert!((hi.k0 - Complex32::new(s, 0.0)).norm() < 1e-5, "{hi:?}");
        assert!((hi.k1 - Complex32::new(s, 0.0)).norm() < 1e-5, "{hi:?}");
    }

    /// General correctness check, not tied to one hand-worked case: for any
    /// covariance matrix, applying eigenpair `(k0, k1)` to samples with that
    /// exact covariance must reproduce its own eigenvalue as the output
    /// power, and the two eigenvectors must be orthogonal (a property of any
    /// Hermitian matrix's eigenvectors, for distinct eigenvalues).
    #[test]
    fn covariance_eigen_pairs_are_orthogonal_and_reproduce_their_eigenvalue() {
        for &(raa, rbb, rab) in &[
            (1.0f32, 1.0f32, Complex32::new(0.3, 0.4)),
            (5.0, 1.0, Complex32::new(-0.2, 1.1)),
            (0.2, 7.0, Complex32::new(0.9, -0.1)),
        ] {
            let (lo, hi) = covariance_eigen(raa, rbb, rab);
            let inner = lo.k0.conj() * hi.k0 + lo.k1.conj() * hi.k1;
            assert!(inner.norm() < 1e-5, "eigenvectors not orthogonal: {inner:?}");
            for pair in [lo, hi] {
                let unit = pair.k0.norm_sqr() + pair.k1.norm_sqr();
                assert!((unit - 1.0).abs() < 1e-5, "not unit norm: {pair:?}");
                // y = k0*A + k1*B has variance k0*conj(k0)*raa + k1*conj(k1)*rbb
                //   + k0*conj(k1)*rab + conj(k0)*k1*conj(rab) -- which for the
                // pre-conjugated (k0, k1) this function returns comes out to
                // exactly the eigenvalue, by construction.
                let var = pair.k0.norm_sqr() * raa
                    + pair.k1.norm_sqr() * rbb
                    + (pair.k0 * pair.k1.conj() * rab).re
                    + (pair.k0.conj() * pair.k1 * rab.conj()).re;
                assert!(
                    (var - pair.eigenvalue).abs() < 1e-3,
                    "eigenpair {pair:?} produced variance {var}, not its own eigenvalue"
                );
            }
        }
    }

    // --- Diversity, DiversityAlgorithm::Decorrelate --------------------------

    /// The case scalar decorrelation *can* handle: a noise source reaching
    /// both aerials with a pure complex gain between them, no delay. Unlike
    /// `a_delayed_and_rotated_noise_source_is_nulled`, there is no `delay`
    /// here — a single weight has no way to equalise one, which is exactly
    /// the trade this algorithm makes for not having to converge.
    #[test]
    fn a_pure_gain_noise_source_is_nulled_by_decorrelation() {
        let n = 50_000;
        let qrm = noise(n, 0x1234);
        let h = Complex32::from_polar(0.6, 0.7);
        let mut main: Vec<Complex32> = qrm.iter().map(|q| q * h).collect();

        let mut d = Diversity::new(DiversityMode::Cancel, 1, 0.5);
        d.set_algorithm(DiversityAlgorithm::Decorrelate);
        d.process(&mut main, &qrm);

        let depth = d.depth_db().expect("Cancel mode reports a depth");
        assert!(depth > 60.0, "only {depth:.1} dB of the noise source was removed");
    }

    /// A wanted signal the auxiliary aerial cannot hear survives the null,
    /// same requirement as the adaptive filter's own version of this test.
    #[test]
    fn a_signal_only_the_main_aerial_hears_survives_decorrelation() {
        let n = 50_000;
        let qrm = noise(n, 0xbeef);
        let h = Complex32::from_polar(0.8, -1.2);
        let want: Vec<Complex32> = (0..n)
            .map(|i| Complex32::from_polar(0.3, std::f32::consts::TAU * i as f32 / 12.0))
            .collect();
        let mut main: Vec<Complex32> = (0..n).map(|i| want[i] + qrm[i] * h).collect();

        let mut d = Diversity::new(DiversityMode::Cancel, 1, 0.5);
        d.set_algorithm(DiversityAlgorithm::Decorrelate);
        d.process(&mut main, &qrm);

        let corr: Complex32 = (0..n)
            .map(|i| main[i] * want[i].conj())
            .fold(Complex32::new(0.0, 0.0), |a, b| a + b);
        let kept = corr.norm() / (n as f32 * 0.3 * 0.3);
        assert!(kept > 0.9, "the wanted tone came through at {kept:.3} of its amplitude");
    }

    /// Same measurement as `two_equal_branches_combine_for_about_three_decibels`,
    /// checked against the decorrelator instead of the adaptive filter — the
    /// claim in `DECORRELATION_PLAN.md` that `Cancel` and `Combine` are the
    /// same solve read two different ways, not two different filters.
    #[test]
    fn two_equal_branches_decorrelate_combine_for_about_three_decibels() {
        let n = 400_000;
        let sig = noise(n, 0xa11ce);
        let n1 = noise(n, 0x1111);
        let n2 = noise(n, 0x2222);
        let h1 = Complex32::from_polar(1.0, 0.3);
        let h2 = Complex32::from_polar(1.0, -2.1);
        let noise_amp = 0.316f32;
        let mut main: Vec<Complex32> = (0..n).map(|i| sig[i] * h1 + n1[i] * noise_amp).collect();
        let aux: Vec<Complex32> = (0..n).map(|i| sig[i] * h2 + n2[i] * noise_amp).collect();

        let mut d = Diversity::new(DiversityMode::Combine, 1, 0.5);
        d.set_algorithm(DiversityAlgorithm::Decorrelate);
        d.process(&mut main, &aux);

        let refs: Vec<Complex32> = (0..n).map(|i| sig[i] * h1).collect();
        let g: Complex32 = main
            .iter()
            .zip(&refs)
            .map(|(o, r)| o * r.conj())
            .fold(Complex32::new(0.0, 0.0), |a, b| a + b)
            / refs.iter().map(|r| r.norm_sqr()).sum::<f32>();
        let resid: Vec<Complex32> = main.iter().zip(&refs).map(|(o, r)| o - r * g).collect();
        let snr_out = 10.0 * ((g.norm_sqr() * power(&refs)) / power(&resid)).log10();
        let own: Vec<Complex32> = (0..n).map(|i| n1[i] * noise_amp).collect();
        let snr_in = 10.0 * (power(&refs) / power(&own)).log10();
        let gain = snr_out - snr_in;
        assert!(
            (2.5..3.5).contains(&gain),
            "combining two equal branches gained {gain:.2} dB, not about 3"
        );
    }

    /// Freezing holds the decorrelation weight, the same guarantee
    /// `freezing_stops_the_filter_moving` makes for the adaptive filter.
    #[test]
    fn freezing_stops_the_decorrelation_weight_moving() {
        let n = 20_000;
        let qrm = noise(n, 7);
        let mut main: Vec<Complex32> = qrm.iter().map(|q| q * 0.5).collect();
        let mut d = Diversity::new(DiversityMode::Cancel, 1, 0.5);
        d.set_algorithm(DiversityAlgorithm::Decorrelate);
        d.process(&mut main, &qrm);
        let held = d.decorrelated_weight().expect("solved once");

        d.set_frozen(true);
        let other = noise(n, 99);
        let mut main2: Vec<Complex32> = other.iter().map(|q| q * 3.0).collect();
        d.process(&mut main2, &other);
        assert_eq!(d.decorrelated_weight(), Some(held), "the weight moved while frozen");
    }

    /// A real bug found live, not a hypothetical: a fresh `Diversity` asked
    /// to start `frozen` before it has ever solved once (settings persist
    /// `frozen: true` from a previous session, and honoring it from the
    /// first block is the normal, expected behavior) must not silence
    /// everything forever, since being frozen is exactly what stops it from
    /// ever getting the chance to solve. Separate mode opened with Hold
    /// already on produced total silence and a blank spectrum on real
    /// hardware; turning Hold off (letting it solve at least once) fixed it
    /// instantly, which is what pinned this down to the starting weight
    /// rather than anything about the solve itself.
    #[test]
    fn freezing_before_ever_solving_leaves_main_untouched_not_silenced() {
        let n = 20_000;
        let qrm = noise(n, 11);
        let mut main: Vec<Complex32> = (0..n).map(|i| Complex32::new(i as f32, -(i as f32))).collect();
        let before = main.clone();

        let mut d = Diversity::new(DiversityMode::Cancel, 1, 0.5);
        d.set_algorithm(DiversityAlgorithm::Decorrelate);
        d.set_frozen(true);
        d.process(&mut main, &qrm);

        assert_eq!(d.decorrelated_weight(), None, "frozen before ever solving has no answer yet");
        assert_eq!(main, before, "main was silenced instead of passed through untouched");
    }

    /// `decorrelated_weight()` is honest about not having an answer yet, and
    /// about which algorithm it belongs to.
    #[test]
    fn decorrelated_weight_is_none_until_solved_and_for_the_other_algorithm() {
        let mut d = Diversity::new(DiversityMode::Cancel, 4, 0.5);
        assert_eq!(d.decorrelated_weight(), None, "adaptive filter has no decorrelation weight");

        d.set_algorithm(DiversityAlgorithm::Decorrelate);
        assert_eq!(d.decorrelated_weight(), None, "nothing solved yet");

        let n = 100;
        let mut main: Vec<Complex32> = noise(n, 1);
        let aux = noise(n, 2);
        d.process(&mut main, &aux);
        assert!(d.decorrelated_weight().is_some(), "a block was processed");
    }

    /// The degenerate case `cancel_weight` exists to get right: a dead `aux`
    /// gives a null direction with essentially no `main` component. The safe
    /// answer is to leave `main` alone, not to output the (silent) `aux`.
    #[test]
    fn a_dead_auxiliary_channel_leaves_main_untouched_in_cancel_mode() {
        let n = 20_000;
        let original = noise(n, 0xd00d);
        let mut main = original.clone();
        let aux = vec![Complex32::new(0.0, 0.0); n];

        let mut d = Diversity::new(DiversityMode::Cancel, 1, 0.5);
        d.set_algorithm(DiversityAlgorithm::Decorrelate);
        d.process(&mut main, &aux);

        assert_eq!(main, original, "a dead aux channel should leave main untouched");
    }
}
