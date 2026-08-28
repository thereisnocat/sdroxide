//! G3RUH 9600 baud: scrambled NRZ baseband, straight onto the carrier.
//!
//! The other packet speed, and a different animal from [`crate::afsk`]. There
//! are no tones. The bit stream is scrambled, shaped, and applied to the
//! transmitter's modulator as baseband — which on FM means the deviation
//! follows the data directly. A receiver takes it from the discriminator.
//!
//! # The chain, and why the scrambler is here rather than in the framer
//!
//! ```text
//! HDLC bits ──▶ stuff ──▶ NRZI ──▶ scramble ──▶ shape ──▶ FM modulator
//!   (sdroxide-ax25::Framer does the first three)     (this module)
//! ```
//!
//! Direwolf's order, confirmed against its source: `hdlc_send.c` stuffs and
//! NRZI-encodes, then `gen_tone.c` scrambles what comes out. So the scrambler
//! sits *below* the framer, which is why it lives with the modem and not with
//! the HDLC — 300 and 1200 baud AFSK use the same framer and no scrambler at
//! all.
//!
//! # What the scrambler is for
//!
//! Not secrecy — the polynomial is published and every station uses the same
//! one. An FM transmitter cannot carry DC, and a run of identical bits is DC.
//! Scrambling turns any input, including a long idle run of flags, into
//! something that looks random and therefore has no DC content and plenty of
//! transitions for the receiver's clock.
//!
//! It is *multiplicative* (self-synchronising): the shift register is fed the
//! transmitted bit, so a receiver locks on after seventeen bits with no
//! agreement about where the data started. The cost is error multiplication —
//! one bad bit on the air becomes three after descrambling — which is why the
//! FCS matters even more here than at 1200.

use crate::fir::RealFir;

/// Symbol rate. The one speed this modem runs at; 4800 and 19200 exist in the
/// wild but reach no Winlink gateway.
pub const BAUD: f64 = 9600.0;

/// The G3RUH polynomial, x¹⁷ + x¹² + 1, as shift-register tap positions.
///
/// Taken from Direwolf's `gen_tone.c`:
/// `x = (dat ^ (lfsr >> 16) ^ (lfsr >> 11)) & 1; lfsr = (lfsr << 1) | x;`
/// At the moment bit `n` is processed the register holds bit `n-1-k` at
/// position `k`, so position 16 is bit `n-17` and position 11 is bit `n-12` —
/// which is the polynomial, and is worth writing down because the off-by-one
/// between "tap position" and "polynomial exponent" is not visible in the code.
const TAP_A: u32 = 16;
const TAP_B: u32 = 11;

/// Scramble and descramble share the register; only which bit is fed back
/// differs — the transmitter feeds back what it sent, the receiver what it
/// heard, which is the same sequence when nothing is corrupted.
#[derive(Debug, Default, Clone, Copy)]
pub struct Scrambler {
    lfsr: u32,
}

impl Scrambler {
    #[must_use]
    pub fn new() -> Self {
        Scrambler::default()
    }

    /// Transmit direction: returns the bit to put on the air.
    pub fn scramble(&mut self, bit: bool) -> bool {
        let out = bit ^ (self.lfsr >> TAP_A & 1 != 0) ^ (self.lfsr >> TAP_B & 1 != 0);
        self.lfsr = (self.lfsr << 1) | u32::from(out);
        out
    }

    /// Receive direction: returns the recovered data bit.
    pub fn descramble(&mut self, bit: bool) -> bool {
        let out = bit ^ (self.lfsr >> TAP_A & 1 != 0) ^ (self.lfsr >> TAP_B & 1 != 0);
        self.lfsr = (self.lfsr << 1) | u32::from(bit);
        out
    }
}

// ─────────────────────────────── transmit ───────────────────────────────

/// Symbol level before shaping. Half scale, as everywhere else in the transmit
/// chain — the level the signal goes out at is the transmit chain's to set.
const SYMBOL: f32 = 0.5;

/// Peak amplitude [`G3ruhTx::next_block`] reaches, [`SYMBOL`] plus the shaping
/// filter's overshoot.
///
/// Higher than the plain symbol level because a low-pass fed a run of hard
/// transitions rings past them, and that ring is part of the waveform: the
/// transmit chain divides this back out to reach full scale
/// (`DigiEngine::tx_peak`), so using the symbol level here instead would drive
/// the shaping peaks a third of the way into the limiter and close the eye the
/// filter was widening. Measured against random data — see
/// `the_shaped_peak_stays_inside_what_is_declared`.
pub const G3RUH_TX_PEAK: f32 = 0.67;

/// Shaped, scrambled baseband. Bits in, modulating signal out.
pub struct G3ruhTx {
    spb: f64,
    scram: Scrambler,
    lpf: RealFir,
    bits: std::collections::VecDeque<bool>,
    left: f64,
    level: f32,
    scratch: Vec<f32>,
    shaped: Vec<f32>,
}

impl G3ruhTx {
    #[must_use]
    pub fn new(rate: f64) -> Self {
        G3ruhTx {
            spb: rate / BAUD,
            scram: Scrambler::new(),
            // Shaping. Rectangular NRZ at 9600 baud would splatter well past
            // the channel; a low-pass a little above half the symbol rate keeps
            // the eye open while bounding the occupied bandwidth.
            lpf: RealFir::lowpass(65, 0.6 * BAUD, rate),
            bits: std::collections::VecDeque::new(),
            left: 0.0,
            level: 0.0,
            scratch: Vec::new(),
            shaped: Vec::new(),
        }
    }

    /// Queue line levels from the HDLC framer — already stuffed and NRZI'd.
    /// Scrambling happens here.
    pub fn push_bits(&mut self, bits: &[bool]) {
        self.bits.extend(bits.iter().copied());
    }

    #[must_use]
    pub fn idle(&self) -> bool {
        self.bits.is_empty() && self.left <= 0.0
    }

    #[must_use]
    pub fn pending_bits(&self) -> usize {
        self.bits.len()
    }

    /// Fill `out` with baseband. Returns how many samples carry signal.
    pub fn next_block(&mut self, out: &mut [f32]) -> usize {
        self.scratch.clear();
        self.scratch.reserve(out.len());
        let mut n = 0;
        for _ in 0..out.len() {
            if self.left <= 0.0 {
                match self.bits.pop_front() {
                    Some(b) => {
                        self.level = if self.scram.scramble(b) { SYMBOL } else { -SYMBOL };
                        // Accumulated rather than assigned, for the reason
                        // `AfskTx::next_block` gives at length: assigning
                        // rounds every symbol up to a whole sample and puts the
                        // over on the air at `rate / spb.ceil()` baud.
                        self.left += self.spb;
                    }
                    None => {
                        self.scratch.push(0.0);
                        continue;
                    }
                }
            }
            self.scratch.push(self.level);
            self.left -= 1.0;
            n += 1;
        }
        self.shaped.clear();
        let mut shaped = std::mem::take(&mut self.shaped);
        self.lpf.process(&self.scratch, &mut shaped);
        for (slot, &s) in out.iter_mut().zip(shaped.iter()) {
            *slot = s;
        }
        self.shaped = shaped;
        n
    }
}

// ─────────────────────────────── receive ────────────────────────────────

/// Clock loop gain. Higher than the AFSK detector's: at 9600 baud into 48 kHz
/// there are only five samples per bit, so the clock has less resolution and
/// has to move decisively on the transitions it does see. Scrambled data
/// guarantees plenty of them.
const CLOCK_TRACK_GAIN: f32 = 0.18;

/// Where a transition is steered to, given the strobe fires at the wrap: half a
/// bit away puts the strobe in the middle of the eye.
const STROBE_PHASE: f32 = 0.5;

/// Slicer-threshold smoothing, about 30 ms at 48 kHz. Scrambled data is
/// DC-free over any useful window, so its mean *is* the right threshold — and
/// tracking it absorbs a mistuned carrier, which on FM shows up as exactly a
/// DC offset in the discriminator output.
const DC_SMOOTH: f32 = 7.0e-4;

/// Envelope smoothing for the carrier-detect figure.
const MAG_SMOOTH: f32 = 1.0e-3;

/// G3RUH receiver: discriminator audio in, line levels out.
///
/// Output is NRZI line levels with the scrambler already removed, ready for
/// `sdroxide_ax25::Deframer`.
pub struct G3ruhRx {
    spb: f32,
    lpf: RealFir,
    descram: Scrambler,

    dc: f32,
    mag: f32,

    clk: f32,
    last_bit: bool,
    have_last: bool,

    filtered: Vec<f32>,
}

impl G3ruhRx {
    #[must_use]
    pub fn new(rate: f64) -> Self {
        G3ruhRx {
            spb: (rate / BAUD) as f32,
            // Matched to the transmit shaping. Wider would collect noise the
            // eye does not need; narrower would close it.
            lpf: RealFir::lowpass(65, 0.6 * BAUD, rate),
            descram: Scrambler::new(),
            dc: 0.0,
            mag: 0.0,
            clk: 0.0,
            last_bit: false,
            have_last: false,
            filtered: Vec::new(),
        }
    }

    /// Smoothed signal level, for the meter.
    #[must_use]
    pub fn magnitude(&self) -> f32 {
        self.mag
    }

    /// Data-carrier detect. Unlike the AFSK detector there is no tone pair to
    /// compare, so this is the eye opening: how far the signal sits from its
    /// own mean, relative to the noise on an idle channel.
    #[must_use]
    pub fn separation(&self) -> f32 {
        (self.mag * 4.0).min(1.0)
    }

    /// Demodulate, appending one line level per recovered bit.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<bool>) {
        self.filtered.clear();
        let mut filtered = std::mem::take(&mut self.filtered);
        self.lpf.process(audio, &mut filtered);

        for &s in &filtered {
            // Track the mean and slice against it.
            self.dc += DC_SMOOTH * (s - self.dc);
            let d = s - self.dc;
            self.mag += MAG_SMOOTH * (d.abs() - self.mag);
            let bit = d > 0.0;

            if self.have_last && bit != self.last_bit {
                let mut e = STROBE_PHASE - self.clk;
                if e > 0.5 {
                    e -= 1.0;
                } else if e < -0.5 {
                    e += 1.0;
                }
                self.clk = (self.clk + CLOCK_TRACK_GAIN * e).rem_euclid(1.0);
            }
            self.last_bit = bit;
            self.have_last = true;

            self.clk += 1.0 / self.spb;
            if self.clk < 1.0 {
                continue;
            }
            self.clk -= 1.0;
            out.push(self.descram.descramble(bit));
        }
        self.filtered = filtered;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The declared peak has to be the real one: the transmit chain divides it
    /// out to reach full scale, so a figure below the waveform clips the
    /// shaping ring and one far above it transmits quiet.
    #[test]
    fn the_shaped_peak_stays_inside_what_is_declared() {
        let mut tx = G3ruhTx::new(48_000.0);
        let mut seed = 7u32;
        let bits: Vec<bool> = (0..8000)
            .map(|_| {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                seed >> 20 & 1 != 0
            })
            .collect();
        tx.push_bits(&bits);
        let mut peak = 0.0f32;
        let mut blk = [0.0f32; 480];
        while !tx.idle() {
            tx.next_block(&mut blk);
            peak = blk.iter().fold(peak, |a, s| a.max(s.abs()));
        }
        assert!(peak <= G3RUH_TX_PEAK, "shaped baseband peaks at {peak}");
        // …and not so far inside it that the declaration is throwing power away.
        assert!(peak > G3RUH_TX_PEAK * 0.9, "declared {G3RUH_TX_PEAK}, measured only {peak}");
    }

    /// The 9600 baud half of issue #180's fault, and the same rule: the symbol
    /// clock keeps its fractional remainder, because a software radio's channel
    /// rate is rarely a whole number of samples a symbol. See
    /// `AfskTx::next_block`.
    #[test]
    fn the_baud_rate_survives_a_rate_that_is_not_whole_samples_a_symbol() {
        for rate in [48_000.0, 50_000.0, 44_642.857_142_857_145, 62_500.0] {
            let mut tx = G3ruhTx::new(rate);
            let bits = 4_000usize;
            tx.push_bits(&vec![true; bits]);
            let mut sent = 0usize;
            loop {
                let mut blk = [0.0f32; 480];
                let n = tx.next_block(&mut blk);
                if n == 0 {
                    break;
                }
                sent += n;
                if tx.idle() {
                    break;
                }
            }
            let got = rate / (sent as f64 / bits as f64);
            assert!((got / BAUD - 1.0).abs() < 0.001, "{rate} Hz: {got:.1} baud against {BAUD}");
        }
    }

    /// Scramble then descramble is the identity, once the register has filled.
    ///
    /// The first seventeen bits are *not* recoverable and are not expected to
    /// be: a self-synchronising descrambler starts from a register it has not
    /// been told, and the opening flags are what pay for that.
    #[test]
    fn the_scrambler_is_its_own_inverse_after_seventeen_bits() {
        let mut tx = Scrambler::new();
        // Deliberately start the receiver's register somewhere else, which is
        // the real situation: it has never seen the bits before the ones it is
        // hearing now.
        let mut rx = Scrambler { lfsr: 0xdead_beef };
        let data: Vec<bool> = (0..200).map(|i| (i * 5 + i / 7) % 3 == 0).collect();
        let recovered: Vec<bool> = data.iter().map(|&b| rx.descramble(tx.scramble(b))).collect();
        assert_eq!(&recovered[17..], &data[17..], "descrambler never synchronised");
    }

    /// A long run of identical bits — the worst case an FM transmitter can be
    /// handed — must come out looking like noise, which is the entire reason
    /// the scrambler is here.
    #[test]
    fn a_constant_input_scrambles_to_something_balanced() {
        let mut s = Scrambler::new();
        let out: Vec<bool> = (0..2000).map(|_| s.scramble(true)).collect();
        let ones = out.iter().filter(|b| **b).count();
        assert!(
            (900..=1100).contains(&ones),
            "{ones} ones in 2000 bits — a constant input is still DC after scrambling"
        );
        // And it must not simply repeat quickly: the sequence's period is
        // 2^17-1, far longer than any frame.
        let longest_run = out
            .windows(2)
            .fold((0usize, 0usize), |(best, run), w| {
                let run = if w[0] == w[1] { run + 1 } else { 0 };
                (best.max(run), run)
            })
            .0;
        assert!(longest_run < 30, "a run of {longest_run} identical bits is not DC-free");
    }

    /// The taps are the published polynomial. Pinned because a plausible edit
    /// here produces a modem that works perfectly against itself and against
    /// nothing else on the air.
    #[test]
    fn the_taps_are_the_g3ruh_polynomial() {
        // x^17 + x^12 + 1, as Direwolf writes it.
        assert_eq!(TAP_A, 16, "x^17 is tap position 16");
        assert_eq!(TAP_B, 11, "x^12 is tap position 11");
    }

    /// Modulate and demodulate, comparing the settled tail — the same posture
    /// as the AFSK loopback, and for the same reason: the clock and the
    /// descrambler both need a run-up, which on the air is the preamble.
    #[test]
    fn baseband_round_trips() {
        let rate = 48_000.0;
        let mut tx = G3ruhTx::new(rate);
        let preamble: Vec<bool> = (0..128).map(|i| i % 2 == 0).collect();
        let bits: Vec<bool> = (0..400).map(|i| (i * 7 + i / 3) % 5 != 0).collect();
        tx.push_bits(&preamble);
        tx.push_bits(&bits);

        let spb = rate / BAUD;
        let n = ((preamble.len() + bits.len()) as f64 * spb).ceil() as usize + 2048;
        let mut audio = vec![0.0f32; n];
        tx.next_block(&mut audio);

        let mut rx = G3ruhRx::new(rate);
        let mut got = Vec::new();
        rx.process(&audio, &mut got);

        let want = &bits[bits.len() / 2..];
        let mut best = usize::MAX;
        for off in 0..got.len().saturating_sub(want.len()) {
            let wrong = got[off..off + want.len()].iter().zip(want).filter(|(a, b)| a != b).count();
            best = best.min(wrong);
            if best == 0 {
                break;
            }
        }
        assert_eq!(best, 0, "{best} of {} bits wrong at 9600 baud", want.len());
    }
}
