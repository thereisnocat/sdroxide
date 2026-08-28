//! Decimators: fast half-band /2 stages plus a generic windowed-sinc
//! FIR decimator for the residual integer factor.

use std::f64::consts::PI;

use crate::Complex32;

fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 { 1.0 } else { (PI * x).sin() / (PI * x) }
}

fn blackman_harris_f64(n: usize, i: usize) -> f64 {
    const A: [f64; 4] = [0.35875, 0.48829, 0.14128, 0.01168];
    let x = std::f64::consts::TAU * i as f64 / (n as f64 - 1.0);
    A[0] - A[1] * x.cos() + A[2] * (2.0 * x).cos() - A[3] * (3.0 * x).cos()
}

/// Windowed-sinc lowpass, DC gain 1. `cutoff` is normalized to the input
/// sample rate (0.5 = Nyquist).
pub fn lowpass_taps(ntaps: usize, cutoff: f64) -> Vec<f32> {
    let center = (ntaps - 1) as f64 / 2.0;
    let mut taps: Vec<f64> = (0..ntaps)
        .map(|i| {
            2.0 * cutoff * sinc(2.0 * cutoff * (i as f64 - center)) * blackman_harris_f64(ntaps, i)
        })
        .collect();
    let sum: f64 = taps.iter().sum();
    taps.iter_mut().for_each(|t| *t /= sum);
    taps.into_iter().map(|t| t as f32).collect()
}

/// Taps in the half-band prototype. Fixed, because the filter is: a cutoff of
/// exactly 0.25 is what zeroes every other tap, and 23 is where the stopband is
/// deep enough for a /2 stage without the dot product growing.
const HB_TAPS: usize = 23;

/// Non-zero taps below the centre one, and so the number of symmetric pairs the
/// dot product below is written out as. `(HB_TAPS / 2 + 1) / 2` for a 23-tap
/// prototype: indices 0, 2, 4, 6, 8, 10.
const HB_PAIRS: usize = 6;

/// Half-band decimator (factor 2).
///
/// Two properties of the prototype are spent here rather than left on the
/// table, because this is the busiest filter in the tree — a receive chain on a
/// wide front end runs a ladder of these at the device rate, 32.4 Msps on an
/// RX-888, and the first stage alone was measured at 5 % of a core.
///
/// * Every second tap is zero (that is what a half-band *is*), so half the
///   products never existed.
/// * The rest are symmetric about the centre, so a pair of samples can share
///   one multiply: `x[a]·t + x[b]·t` is `(x[a] + x[b])·t`. That is a complex
///   add standing in for a complex multiply, and it halves the multiplies
///   again — seven where the straightforward loop does thirteen.
///
/// Measured together at 1.7× the loop they replace.
pub struct HalfbandDecim {
    /// The surviving taps, as `(low index, its mirror, value)`. Built from the
    /// prototype rather than written down, so the numbers stay derived.
    pairs: [(usize, usize, f32); HB_PAIRS],
    /// The centre tap, which has no partner.
    center: (usize, f32),
    buf: Vec<Complex32>,
}

impl HalfbandDecim {
    pub fn new() -> Self {
        let taps = lowpass_taps(HB_TAPS, 0.25);
        let center = HB_TAPS / 2;
        let mut pairs = [(0usize, 0usize, 0.0f32); HB_PAIRS];
        let mut found = 0;
        // The zeros are what the cutoff put there; recognising them rather than
        // assuming their positions keeps this honest if the prototype is ever
        // re-cut.
        for (k, &tap) in taps.iter().enumerate().take(center).filter(|(_, t)| t.abs() > 1e-12) {
            pairs[found] = (k, HB_TAPS - 1 - k, tap);
            found += 1;
        }
        assert_eq!(found, HB_PAIRS, "the half-band prototype changed shape");
        HalfbandDecim { pairs, center: (center, taps[center]), buf: Vec::new() }
    }

    pub fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        self.buf.extend_from_slice(input);
        if self.buf.len() < HB_TAPS {
            return;
        }
        let count = (self.buf.len() - HB_TAPS) / 2 + 1;
        out.reserve(count);
        let p = self.pairs;
        let (ci, ct) = self.center;
        for o in 0..count {
            // One slice, so the bounds are checked once for the whole window
            // instead of once per tap.
            let w = &self.buf[o * 2..o * 2 + HB_TAPS];
            // Written out in two halves rather than looped: six terms summed
            // in sequence is one dependency chain the length of the filter,
            // and two chains of three run in half the time.
            let a = (w[p[0].0] + w[p[0].1]) * p[0].2
                + (w[p[1].0] + w[p[1].1]) * p[1].2
                + (w[p[2].0] + w[p[2].1]) * p[2].2;
            let b = (w[p[3].0] + w[p[3].1]) * p[3].2
                + (w[p[4].0] + w[p[4].1]) * p[4].2
                + (w[p[5].0] + w[p[5].1]) * p[5].2;
            out.push(w[ci] * ct + (a + b));
        }
        self.buf.drain(..count * 2);
    }
}

/// Cascade of half-band stages: decimation by any power of two, keeping the
/// span centred where it already was.
///
/// This is the front-end decimator the operator sets from the RX box, not a
/// downconverter — there is no NCO, so DC in is DC out and the hardware LO
/// stays the middle of the span. What it buys is everything downstream running
/// at a fraction of the device rate.
///
/// Half-band stages are what make a deep factor affordable: each costs about a
/// dozen multiply-accumulates per *output* sample, and every stage halves the
/// rate the next one is fed at, so the whole cascade costs less than twice its
/// first stage no matter how far down it goes.
pub struct Decimator {
    stages: Vec<HalfbandDecim>,
    tmp_a: Vec<Complex32>,
    tmp_b: Vec<Complex32>,
}

impl Decimator {
    /// `factor` is rounded *down* to a power of two; 1 (or 0) builds a
    /// pass-through, so a caller does not have to special-case "off".
    pub fn new(factor: u32) -> Self {
        let stages = (0..factor.max(1).ilog2()).map(|_| HalfbandDecim::new()).collect();
        Decimator { stages, tmp_a: Vec::new(), tmp_b: Vec::new() }
    }

    /// The factor actually built — `1 << stages`, which is [`Decimator::new`]'s
    /// argument rounded down to a power of two.
    pub fn factor(&self) -> u32 {
        1 << self.stages.len()
    }

    /// Appends the decimated samples to `out`.
    ///
    /// Each stage keeps its own tail of unconsumed samples, so a block whose
    /// length is not a multiple of the factor is carried over rather than
    /// dropped — the output is continuous across calls.
    pub fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        let Some((last, rest)) = self.stages.split_last_mut() else {
            out.extend_from_slice(input);
            return;
        };
        // The first stage reads the caller's buffer and the last writes into
        // `out`, so only the middle ones ever touch the scratch pair — a /2 and
        // a /4 copy nothing at all.
        let mut middle = rest.iter_mut();
        let Some(first) = middle.next() else {
            last.process(input, out);
            return;
        };
        self.tmp_a.clear();
        first.process(input, &mut self.tmp_a);
        for hb in middle {
            self.tmp_b.clear();
            hb.process(&self.tmp_a, &mut self.tmp_b);
            std::mem::swap(&mut self.tmp_a, &mut self.tmp_b);
        }
        last.process(&self.tmp_a, out);
    }
}

/// Independent running sums a filter window is walked with.
///
/// A dot product written as one accumulator is a chain: every
/// multiply-accumulate waits on the one before it, so a filter costs its tap
/// count times the *latency* of an add and the machine's other pipelines stand
/// idle. Splitting the window across several accumulators that are summed at
/// the end computes exactly the same thing with the chain cut into that many
/// independent pieces.
///
/// Eight is measured, not assumed: on the 97-tap /8 stage a zoom lane builds at
/// 32.4 Msps, two accumulators buy 1.1×, four 1.3×, eight 1.7×, and twelve
/// falls off a cliff (more live values than there are registers to hold them).
const FIR_ACCUMULATORS: usize = 8;

/// `window · taps`, walked with [`FIR_ACCUMULATORS`] independent sums.
#[inline]
fn dot(window: &[Complex32], taps: &[f32]) -> Complex32 {
    let mut acc = [Complex32::default(); FIR_ACCUMULATORS];
    let mut w = window.chunks_exact(FIR_ACCUMULATORS);
    let mut t = taps.chunks_exact(FIR_ACCUMULATORS);
    for (xs, ts) in (&mut w).zip(&mut t) {
        for i in 0..FIR_ACCUMULATORS {
            acc[i] += xs[i] * ts[i];
        }
    }
    let mut sum = Complex32::default();
    for a in acc {
        sum += a;
    }
    // The taps are an odd count by construction, so there is always a tail.
    for (x, &tap) in w.remainder().iter().zip(t.remainder()) {
        sum += x * tap;
    }
    sum
}

/// Generic FIR decimator by an integer factor.
pub struct FirDecim {
    taps: Vec<f32>,
    factor: usize,
    buf: Vec<Complex32>,
}

impl FirDecim {
    pub fn new(factor: usize) -> Self {
        assert!(factor >= 1);
        let ntaps = (12 * factor).clamp(24, 768) | 1; // odd
        let taps = lowpass_taps(ntaps, 0.45 / factor as f64);
        FirDecim { taps, factor, buf: Vec::new() }
    }

    pub fn process(&mut self, input: &[Complex32], out: &mut Vec<Complex32>) {
        self.buf.extend_from_slice(input);
        let n = self.taps.len();
        if self.buf.len() < n {
            return;
        }
        let count = (self.buf.len() - n) / self.factor + 1;
        out.reserve(count);
        for o in 0..count {
            let base = o * self.factor;
            out.push(dot(&self.buf[base..base + n], &self.taps));
        }
        self.buf.drain(..count * self.factor);
    }
}

/// [`dot`] for a real-valued window — the audio counterpart.
#[inline]
fn real_dot(window: &[f32], taps: &[f32]) -> f32 {
    let mut acc = [0.0f32; FIR_ACCUMULATORS];
    let mut w = window.chunks_exact(FIR_ACCUMULATORS);
    let mut t = taps.chunks_exact(FIR_ACCUMULATORS);
    for (xs, ts) in (&mut w).zip(&mut t) {
        for i in 0..FIR_ACCUMULATORS {
            acc[i] += xs[i] * ts[i];
        }
    }
    let mut sum = 0.0;
    for a in acc {
        sum += a;
    }
    for (x, &tap) in w.remainder().iter().zip(t.remainder()) {
        sum += x * tap;
    }
    sum
}

/// Real-valued decimating FIR: the counterpart to [`FirDecim`] for audio.
///
/// Only the kept outputs are computed, so a long anti-alias filter costs
/// `ntaps / factor` multiply-accumulates per output sample — which is what
/// makes the two 15 kHz low-passes of the WFM stereo decoder cheaper than the
/// single non-decimating one they replace.
///
/// The cutoff is given in Hz at the *input* rate; the caller is responsible for
/// keeping it below the output Nyquist.
pub struct RealFirDecim {
    taps: Vec<f32>,
    factor: usize,
    buf: Vec<f32>,
}

impl RealFirDecim {
    pub fn new(ntaps: usize, cutoff_hz: f64, sample_rate: f64, factor: usize) -> Self {
        assert!(factor >= 1);
        RealFirDecim { taps: lowpass_taps(ntaps, cutoff_hz / sample_rate), factor, buf: Vec::new() }
    }

    /// Group delay in samples at the *output* rate.
    pub fn delay(&self) -> f64 {
        (self.taps.len() - 1) as f64 / 2.0 / self.factor as f64
    }

    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        self.buf.extend_from_slice(input);
        let n = self.taps.len();
        if self.buf.len() < n {
            return;
        }
        let count = (self.buf.len() - n) / self.factor + 1;
        out.reserve(count);
        for o in 0..count {
            let base = o * self.factor;
            out.push(real_dot(&self.buf[base..base + n], &self.taps));
        }
        self.buf.drain(..count * self.factor);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    /// A unit-amplitude complex tone at `f` cycles per sample.
    fn tone(n: usize, f: f32) -> Vec<Complex32> {
        (0..n)
            .map(|i| {
                let p = TAU * f * i as f32;
                Complex32::new(p.cos(), p.sin())
            })
            .collect()
    }

    /// Level at `f` cycles per sample, in dB relative to a unit-amplitude tone.
    fn tone_db(x: &[Complex32], f: f32) -> f32 {
        let mut acc = Complex32::default();
        for (i, s) in x.iter().enumerate() {
            let p = -TAU * f * i as f32;
            acc += *s * Complex32::new(p.cos(), p.sin());
        }
        20.0 * (acc.norm() / x.len() as f32).max(1e-12).log10()
    }

    /// Decimate one long tone in blocks, the way the engine feeds it, and hand
    /// back the steady state (the first samples cover the filters' warm-up).
    fn decimate_tone(factor: u32, f: f32) -> Vec<Complex32> {
        let input = tone(1 << 17, f);
        let mut d = Decimator::new(factor);
        let mut out = Vec::new();
        for block in input.chunks(4096) {
            d.process(block, &mut out);
        }
        out.split_off(1024)
    }

    #[test]
    fn a_decimator_rounds_its_factor_down_to_a_power_of_two() {
        assert_eq!(Decimator::new(0).factor(), 1);
        assert_eq!(Decimator::new(1).factor(), 1);
        assert_eq!(Decimator::new(6).factor(), 4);
        assert_eq!(Decimator::new(64).factor(), 64);
    }

    /// Factor 1 is the "decimation off" case the engine leaves in place, and it
    /// has to be a pass-through rather than a filter with a unit factor.
    #[test]
    fn a_pass_through_decimator_hands_the_block_straight_back() {
        let input = tone(64, 0.1);
        let mut out = Vec::new();
        Decimator::new(1).process(&input, &mut out);
        assert_eq!(out, input);
    }

    /// Nothing is lost across the block boundaries: every stage carries its own
    /// tail, so N input samples produce N/factor output samples give or take
    /// the filters' warm-up, however the input is chopped up.
    #[test]
    fn the_output_rate_is_the_input_rate_over_the_factor() {
        for factor in [2u32, 4, 8, 16, 32] {
            let input = tone(1 << 16, 0.001);
            let mut d = Decimator::new(factor);
            let mut out = Vec::new();
            for block in input.chunks(1000) {
                d.process(block, &mut out);
            }
            let want = input.len() / factor as usize;
            let slack = 22 * factor as usize; // 11 input samples of group delay per stage
            assert!(
                want - out.len() < slack,
                "factor {factor}: {} samples out, wanted ~{want}",
                out.len()
            );
        }
    }

    /// The point of the anti-alias filtering: a signal from the part of the span
    /// being thrown away must not reappear inside the part being kept. Unfiltered,
    /// the 0.2 tone would land at -0.4 in the output at full strength and read as
    /// a real signal on the panadapter.
    #[test]
    fn what_falls_outside_the_kept_span_does_not_fold_into_it() {
        // 0.01 of the input rate is 0.08 of the /8 output rate: comfortably
        // inside, and it has to come through at full amplitude.
        let kept = decimate_tone(8, 0.01);
        let db = tone_db(&kept, 0.08);
        assert!(db > -0.2, "kept tone lost {db} dB");

        // 0.2 of the input rate is outside the ±0.0625 that /8 keeps. It would
        // alias to 1.6 cycles/sample at the output rate, i.e. -0.4.
        let folded = decimate_tone(8, 0.2);
        let db = tone_db(&folded, -0.4);
        assert!(db < -70.0, "alias only {db} dB down");
    }
}
