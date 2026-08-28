//! Pairing an RSPduo's two tuners, sample for sample (issue #153).
//!
//! With both tuners running, the service calls *two* stream callbacks — one
//! per tuner — from a thread of its own. What comes out is two aerials on the
//! same span, sampled by one ADC clock from one reference, so their relative
//! phase is fixed by the feedlines rather than by chance. That is the property
//! the diversity filter in `sdroxide_dsp` is built on, and it is worth nothing
//! at all unless the two blocks handed to that filter are *the same samples in
//! time*. A filter fed a pair that is off by even a handful of samples fits a
//! delay that is not there: it converges on nothing, or worse, on the wanted
//! signal.
//!
//! # Why this is a producer-side job here
//!
//! The LimeSDR backend pairs on the *read* side, because LimeSuite hands its
//! two channels back through two FIFOs that the caller drains itself. Here the
//! service pushes, so the two streams meet in the callbacks — and once they
//! have met, the cheapest way to keep them together is to never let them
//! apart: [`Pairer::drain`] writes both tuners into one ring as interleaved
//! *quadruples* (main I, main Q, aux I, aux Q). One ring cannot desynchronise
//! with itself, so the reader has nothing to check.
//!
//! # What the pairing is against
//!
//! Each block arrives stamped with `firstSampleNum`, the hardware sample
//! counter of its first sample, and in dual-tuner mode both tuners are counted
//! by the same clock. So the two staged queues are aligned by that number,
//! whatever order the callbacks happen to run in and whatever sizes they bring.
//!
//! **None of this has been run against an RSPduo.** Two failures are therefore
//! planned for rather than assumed away:
//!
//! * A build that does not fill the stamp in. Noticed the first time a block
//!   arrives stamped the same as the last one, after which pairing falls back
//!   to arrival order — which is what the callbacks give anyway when nothing
//!   has been dropped.
//! * One tuner that stops delivering. After [`Pairer::stall_samples`] of the
//!   other tuner have piled up with nothing to pair them against, they are
//!   released against silence and [`Pairer::stalled`] is set: the operator
//!   loses the diversity filter and keeps the receiver, which is the right way
//!   round. Nothing here may be able to make the second tuner work, but
//!   nothing here is allowed to take the first one down with it.

use rtrb::Producer;

use crate::handle::{RxStats, push_iq};

/// 16-bit wire samples to full-scale ±1.0.
const SCALE: f32 = 1.0 / 32768.0;

/// Floats per complex sample in a staged queue, and per paired sample in the
/// ring: one tuner is I and Q, the pair is both tuners' I and Q.
const IQ: usize = 2;
pub(crate) const QUAD: usize = 4;

/// Which tuner a block came from. Not the API's `TunerSelect`: which of A and
/// B is the main aerial is the operator's choice, and it is resolved once when
/// the callbacks are installed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Side {
    Main,
    Aux,
}

/// One tuner's samples, staged until the other's arrive.
#[derive(Default)]
struct Staged {
    /// Interleaved I/Q, oldest first.
    buf: Vec<f32>,
    /// Index of the oldest sample still wanted, in floats — always even.
    head: usize,
    /// The hardware sample number of the sample at [`Self::head`].
    num: u32,
}

impl Staged {
    /// Complex samples held.
    fn len(&self) -> usize {
        (self.buf.len() - self.head) / IQ
    }

    /// The sample number one past the end of what is held.
    fn end(&self) -> u32 {
        self.num.wrapping_add(self.len() as u32)
    }

    fn clear(&mut self) {
        self.buf.clear();
        self.head = 0;
    }

    /// Throw away the `n` oldest complex samples.
    fn skip(&mut self, n: usize) {
        let n = n.min(self.len());
        self.head += n * IQ;
        self.num = self.num.wrapping_add(n as u32);
        if self.head == self.buf.len() {
            self.clear();
        }
    }

    fn extend(&mut self, xi: &[i16], xq: &[i16]) {
        // Compact before growing, so a long session cannot grow the buffer
        // without bound.
        if self.head > 0 {
            self.buf.drain(..self.head);
            self.head = 0;
        }
        self.buf.reserve(xi.len() * IQ);
        for (&i, &q) in xi.iter().zip(xq) {
            self.buf.push(f32::from(i) * SCALE);
            self.buf.push(f32::from(q) * SCALE);
        }
    }

    fn samples(&self) -> &[f32] {
        &self.buf[self.head..]
    }
}

/// The two tuners' staged samples, and the arithmetic that puts them together.
pub(crate) struct Pairer {
    main: Staged,
    aux: Staged,
    /// The interleaved quadruples on their way to the ring.
    out: Vec<f32>,
    /// Whether the sample numbers are believed. Cleared the first time a block
    /// arrives stamped the same as the last one, which is what a build that
    /// does not fill them in looks like.
    stamped: bool,
    /// How many times the pairing had to be abandoned and restarted — a gap in
    /// one tuner's stream and not the other's. Reported, because a pairing
    /// that keeps restarting is one whose sample rate the host cannot keep up
    /// with.
    slips: u64,
    /// The second tuner has stopped delivering and the first is going through
    /// unpaired. Cleared by the next real pair.
    stalled: bool,
    /// How much of one tuner may pile up unpaired before it is released
    /// against silence — about 50 ms, so a hiccup costs a stutter in the
    /// filter rather than a gap in the audio.
    stall_samples: usize,
}

impl Pairer {
    pub(crate) fn new(rate_hz: f64) -> Pairer {
        Pairer {
            main: Staged::default(),
            aux: Staged::default(),
            out: Vec::new(),
            stamped: true,
            slips: 0,
            stalled: false,
            stall_samples: ((rate_hz * 0.05) as usize).clamp(4_096, 65_536),
        }
    }

    pub(crate) fn slips(&self) -> u64 {
        self.slips
    }

    pub(crate) fn stalled(&self) -> bool {
        self.stalled
    }

    pub(crate) fn believes_sample_numbers(&self) -> bool {
        self.stamped
    }

    /// Throw both queues away and start pairing from the next blocks.
    ///
    /// What a radio letting go of the pair leaves behind: whatever was staged
    /// belongs to a span nobody is listening to any more, and holding it would
    /// only make the first block after the next attach a slip.
    pub(crate) fn restart(&mut self) {
        self.main.clear();
        self.aux.clear();
        self.stalled = false;
    }

    /// Stage one block from one tuner.
    ///
    /// A block that does not continue where that tuner's last one ended means
    /// the stream dropped something, so what is held is no longer contiguous
    /// and is thrown away rather than spliced — a splice would be a delay the
    /// filter cannot see and would fit against.
    pub(crate) fn push(&mut self, side: Side, xi: &[i16], xq: &[i16], first_num: u32) {
        let n = xi.len().min(xq.len());
        if n == 0 {
            return;
        }
        let stamped = self.stamped;
        let mut slipped = false;
        let mut unstamped = false;
        let q = match side {
            Side::Main => &mut self.main,
            Side::Aux => &mut self.aux,
        };
        if q.len() == 0 {
            q.clear();
            q.num = first_num;
        } else if stamped && first_num != q.end() {
            // Either a gap, or a build that stamps everything the same. Told
            // apart by the exact repeat: a service that does not fill the
            // number in hands back the same one every time, which is the one
            // value that cannot be a gap. Anything else — including the
            // counter starting again after a retune — is a discontinuity, and
            // treating *that* as an unstamped build would throw the pairing's
            // one real tool away the first time the operator moved the dial.
            if first_num == q.num {
                unstamped = true;
            }
            slipped = true;
            q.clear();
            q.num = first_num;
        }
        q.extend(&xi[..n], &xq[..n]);
        if unstamped {
            self.stamped = false;
        }
        if slipped {
            self.slips += 1;
        }
    }

    /// Write every sample pair both tuners have into the ring.
    ///
    /// Returns `(offered, dropped)` in complex samples: what the pairing
    /// produced, and how much of it the ring could not take — the second is
    /// accounted for exactly as a single-tuner overrun, and the first is the
    /// caller's evidence that the pairing is alive at all. A pairing that
    /// offers nothing while blocks keep arriving must *not* look like a
    /// healthy receiver, or the watchdog that reopens a silent session would
    /// never fire.
    pub(crate) fn drain(
        &mut self,
        ring: &mut Producer<f32>,
        stats: &mut RxStats,
        paused: bool,
    ) -> (usize, usize) {
        if self.stamped {
            // The two tuners are counted by one clock, so the skew is a plain
            // wrapping difference — the streams are milliseconds apart at
            // most, nowhere near the half-counter the sign would need.
            let skew = self.main.num.wrapping_sub(self.aux.num) as i32;
            if self.main.len() > 0 && self.aux.len() > 0 {
                match skew.cmp(&0) {
                    std::cmp::Ordering::Greater => self.aux.skip(skew as usize),
                    std::cmp::Ordering::Less => self.main.skip(skew.unsigned_abs() as usize),
                    std::cmp::Ordering::Equal => {}
                }
            }
        }
        let n = self.main.len().min(self.aux.len());
        if n > 0 {
            self.stalled = false;
            self.out.clear();
            self.out.reserve(n * QUAD);
            let (m, a) = (self.main.samples(), self.aux.samples());
            for i in 0..n {
                self.out.extend_from_slice(&m[i * IQ..i * IQ + IQ]);
                self.out.extend_from_slice(&a[i * IQ..i * IQ + IQ]);
            }
            self.main.skip(n);
            self.aux.skip(n);
            return (n, push_iq(ring, &self.out, stats, paused, QUAD));
        }
        // Nothing to pair. The second tuner piling up against a first that is
        // not arriving is a wait; the *first* piling up is a second tuner that
        // has stopped, and the receiver must not stop with it.
        if self.aux.len() > self.stall_samples {
            let excess = self.aux.len() - self.stall_samples;
            self.aux.skip(excess);
            self.slips += 1;
        }
        if self.main.len() > self.stall_samples {
            self.stalled = true;
            let n = self.main.len();
            self.out.clear();
            self.out.resize(n * QUAD, 0.0);
            let m = self.main.samples();
            for i in 0..n {
                self.out[i * QUAD] = m[i * IQ];
                self.out[i * QUAD + 1] = m[i * IQ + 1];
            }
            self.main.skip(n);
            // The pairing is broken, not merely late: keeping the aux queue's
            // idea of where it is would have the alignment above throw the
            // first tuner away when the second one comes back.
            self.aux.clear();
            return (n, push_iq(ring, &self.out, stats, paused, QUAD));
        }
        (0, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rtrb::RingBuffer;

    /// A block of `n` samples whose I is a ramp from `from` and whose Q is the
    /// side's marker, so a mispairing is visible rather than merely wrong.
    fn block(from: i16, n: usize, marker: i16) -> (Vec<i16>, Vec<i16>) {
        ((0..n).map(|i| from + i as i16).collect(), vec![marker; n])
    }

    fn drained(p: &mut Pairer, ring: &mut Producer<f32>) -> usize {
        let mut stats = RxStats::new(2.0e6);
        p.drain(ring, &mut stats, false).0
    }

    /// One sample number, two tuners: the ring gets both, in step.
    #[test]
    fn the_two_tuners_land_in_the_ring_as_quadruples() {
        let (mut prod, mut cons) = RingBuffer::<f32>::new(1024);
        let mut p = Pairer::new(2.0e6);
        let (mi, mq) = block(0, 4, 1);
        let (ai, aq) = block(100, 4, 2);
        p.push(Side::Main, &mi, &mq, 1000);
        assert_eq!(drained(&mut p, &mut prod), 0, "nothing to pair with yet");
        assert_eq!(cons.slots(), 0);
        p.push(Side::Aux, &ai, &aq, 1000);
        assert_eq!(drained(&mut p, &mut prod), 4);
        assert_eq!(cons.slots(), 4 * QUAD);
        let chunk = cons.read_chunk(QUAD).unwrap();
        let (a, _) = chunk.as_slices();
        assert_eq!(a[1], SCALE, "main Q carries the main marker");
        assert_eq!(a[3], 2.0 * SCALE, "aux Q carries the aux marker");
        assert!((a[2] - 100.0 * SCALE).abs() < 1e-9, "aux I is the aux ramp");
    }

    /// The tuner that starts later decides the span: what the other heard
    /// before it is older than anything that can be paired, and goes.
    #[test]
    fn samples_older_than_the_other_tuner_are_discarded() {
        let (mut prod, mut cons) = RingBuffer::<f32>::new(1024);
        let mut p = Pairer::new(2.0e6);
        let (mi, mq) = block(0, 8, 1);
        let (ai, aq) = block(0, 8, 2);
        p.push(Side::Main, &mi, &mq, 1000);
        p.push(Side::Aux, &ai, &aq, 1004);
        assert_eq!(drained(&mut p, &mut prod), 4, "only the overlapping four pair");
        assert_eq!(cons.slots(), 4 * QUAD);
        let chunk = cons.read_chunk(QUAD).unwrap();
        let (a, _) = chunk.as_slices();
        assert!((a[0] - 4.0 * SCALE).abs() < 1e-9, "the main stream skipped to 1004");
    }

    /// A gap in one tuner's stream restarts that side rather than splicing
    /// across it, and says so.
    #[test]
    fn a_gap_restarts_the_pairing_instead_of_splicing() {
        let (mut prod, _cons) = RingBuffer::<f32>::new(1024);
        let mut p = Pairer::new(2.0e6);
        let (mi, mq) = block(0, 8, 1);
        p.push(Side::Main, &mi, &mq, 1000);
        p.push(Side::Main, &mi, &mq, 2000); // 992 samples went missing
        assert_eq!(p.slips(), 1);
        assert!(p.believes_sample_numbers());
        let (ai, aq) = block(0, 8, 2);
        p.push(Side::Aux, &ai, &aq, 2000);
        drained(&mut p, &mut prod);
        assert_eq!(_cons.slots(), 8 * QUAD, "the pairing picked up at the new number");
    }

    /// The sample counter starting again — what a retune looks like — is a
    /// discontinuity like any other, and must not be mistaken for a service
    /// that does not fill the number in at all.
    #[test]
    fn a_counter_that_restarts_is_a_gap_not_an_unstamped_service() {
        let (mut prod, cons) = RingBuffer::<f32>::new(1024);
        let mut p = Pairer::new(2.0e6);
        let (mi, mq) = block(0, 8, 1);
        let (ai, aq) = block(0, 8, 2);
        p.push(Side::Main, &mi, &mq, 1_000_000);
        p.push(Side::Main, &mi, &mq, 0);
        assert!(p.believes_sample_numbers(), "a restart is not a missing stamp");
        assert_eq!(p.slips(), 1);
        p.push(Side::Aux, &ai, &aq, 0);
        assert_eq!(drained(&mut p, &mut prod), 8, "and pairing carries on from the new number");
        assert_eq!(cons.slots(), 8 * QUAD);
    }

    /// A service that does not stamp its blocks: the counter never moves,
    /// which is noticed once and then pairing falls back to arrival order.
    #[test]
    fn an_unstamped_service_falls_back_to_arrival_order() {
        let (mut prod, cons) = RingBuffer::<f32>::new(1024);
        let mut p = Pairer::new(2.0e6);
        let (mi, mq) = block(0, 4, 1);
        let (ai, aq) = block(0, 4, 2);
        p.push(Side::Main, &mi, &mq, 0);
        p.push(Side::Main, &mi, &mq, 0);
        assert!(!p.believes_sample_numbers());
        p.push(Side::Aux, &ai, &aq, 0);
        p.push(Side::Aux, &ai, &aq, 0);
        drained(&mut p, &mut prod);
        // Four samples survived the restart on each side and pair by order.
        assert_eq!(cons.slots(), 4 * QUAD);
    }

    /// A second tuner that stops delivering must not take the first one down
    /// with it: after the backlog builds, the main stream goes through
    /// unpaired and says so.
    #[test]
    fn a_dead_second_tuner_releases_the_first_one() {
        let (mut prod, mut cons) = RingBuffer::<f32>::new(1 << 16);
        let mut p = Pairer::new(2.0e6);
        let mut num = 0u32;
        let mut pushed = 0usize;
        while !p.stalled() {
            let (mi, mq) = block(0, 512, 1);
            p.push(Side::Main, &mi, &mq, num);
            num = num.wrapping_add(512);
            pushed += 512;
            drained(&mut p, &mut prod);
            assert!(pushed < 200_000, "the stall valve never opened");
        }
        assert!(cons.slots() > 0, "the first tuner reached the ring");
        let chunk = cons.read_chunk(QUAD).unwrap();
        let (a, _) = chunk.as_slices();
        assert_eq!(a[1], SCALE, "the main tuner's samples are there");
        assert_eq!(a[2], 0.0, "and the missing tuner reads as silence");
        assert_eq!(a[3], 0.0);
    }

    /// ...and when it comes back, pairing resumes without dragging the stale
    /// backlog in behind it.
    #[test]
    fn pairing_resumes_after_a_stall() {
        let (mut prod, mut cons) = RingBuffer::<f32>::new(1 << 16);
        let mut p = Pairer::new(2.0e6);
        let mut num = 0u32;
        while !p.stalled() {
            let (mi, mq) = block(0, 512, 1);
            p.push(Side::Main, &mi, &mq, num);
            num = num.wrapping_add(512);
            drained(&mut p, &mut prod);
        }
        while cons.slots() > 0 {
            let n = cons.slots();
            cons.read_chunk(n).unwrap().commit_all();
        }
        let (mi, mq) = block(0, 64, 1);
        let (ai, aq) = block(0, 64, 2);
        p.push(Side::Main, &mi, &mq, num);
        p.push(Side::Aux, &ai, &aq, num);
        drained(&mut p, &mut prod);
        assert!(!p.stalled(), "a real pair clears the stall");
        assert_eq!(cons.slots(), 64 * QUAD);
        let chunk = cons.read_chunk(QUAD).unwrap();
        let (a, _) = chunk.as_slices();
        assert_eq!(a[3], 2.0 * SCALE, "the second tuner is back in the stream");
    }
}
