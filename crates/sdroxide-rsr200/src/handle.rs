//! The handle the rest of the program holds, and the accounting behind it.
//!
//! Same shape as every other native backend in this workspace (see
//! `sdroxide-rtlsdr`'s own `handle.rs`): one blocking thread owns the
//! transport and the [`crate::device::Device`] state machine, control goes
//! in over a `crossbeam_channel`, samples come back out through an `rtrb`
//! ring of interleaved `f32`.
//!
//! [`Rsr200Handle::read_pair`] mirrors `SdrPlayHandle`'s own method of the
//! same name (`RSR200_PLAN.md` step 4): with both ADCs running the ring
//! holds quadruples (main I, main Q, aux I, aux Q) rather than pairs, same
//! as an RSPduo's two tuners — and, unlike the RSPduo, needs no `Pairer`
//! (`sdroxide-sdrplay::pair`) to get there: [`crate::device::Device::pump`]
//! already delivers both channels from the very same parsed frame in one
//! call, so they cannot come apart on the way into the ring the way two
//! independently-arriving tuner callbacks could.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;
use rtrb::{Consumer, Producer, RingBuffer};
use sdroxide_types::Rsr200Config;

use crate::error::Result;

/// A control message for the stream thread.
///
/// Deliberately short: [`Rsr200Config::adc_clock_hz`],
/// `decimation_exp` and `gps_discipline` are not here — changing any of
/// them moves the sample rate, so [`crate::rsr200_source`] (outside this
/// crate) treats them the same way `sdrplay_source.rs` treats its own
/// sample-rate-affecting settings: a reopen, not a live message. Only what
/// can genuinely change without disturbing the ring's own sizing lands here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Ctrl {
    Center(f64),
    Attenuator1(i32),
    Attenuator2(i32),
    Shutdown,
}

/// Control messages accumulated over one pass of the thread loop — last
/// value wins per field, the same collapsing `sdroxide-rtlsdr`'s own
/// `Pending` does and for the same reason: a dial drag emits far more
/// messages than the radio needs applied.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct Pending {
    pub center: Option<f64>,
    pub attenuator1: Option<i32>,
    pub attenuator2: Option<i32>,
    pub shutdown: bool,
}

impl Pending {
    pub(crate) fn absorb(&mut self, c: Ctrl) {
        match c {
            Ctrl::Center(v) => self.center = Some(v),
            Ctrl::Attenuator1(v) => self.attenuator1 = Some(v),
            Ctrl::Attenuator2(v) => self.attenuator2 = Some(v),
            Ctrl::Shutdown => self.shutdown = true,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        *self == Pending::default()
    }
}

/// Shared state the stream thread publishes and the handle reads.
pub(crate) struct Shared {
    pub alive: AtomicBool,
    /// Milliseconds since the thread started, at the last sample delivered.
    pub last_rx_ms: AtomicU64,
}

/// Push interleaved I/Q into the RX ring, keeping I and Q paired.
///
/// All-or-nothing: if the ring cannot take the whole block it is dropped
/// whole, for the same reason every other backend here does it — a partial
/// write would leave the ring one float out of step, swapping I with Q for
/// the rest of the session.
pub(crate) fn push_iq(rx: &mut Producer<f32>, iq: &[f32]) -> bool {
    let Ok(mut chunk) = rx.write_chunk(iq.len()) else {
        return false;
    };
    let (head, tail) = chunk.as_mut_slices();
    head.copy_from_slice(&iq[..head.len()]);
    tail.copy_from_slice(&iq[head.len()..]);
    chunk.commit_all();
    true
}

/// Floats per sample in the ring: I and Q, or (with both ADCs running) both
/// channels' I and Q — matching `sdroxide-sdrplay`'s own `pair::QUAD`.
pub(crate) const QUAD: usize = 4;

/// Half a second of interleaved floats, rounded up to a power of two — same
/// formula as every other native backend's ring here. `stride` is 2 for one
/// ADC, [`QUAD`] for two.
pub(crate) fn ring_for(rate_hz: f64, stride: usize) -> (Producer<f32>, Consumer<f32>) {
    let cap = ((rate_hz * stride as f64 * 0.5) as usize).next_power_of_two().max(1 << 16);
    RingBuffer::<f32>::new(cap)
}

/// A connected RSR200, streaming over LAN or USB.
pub struct Rsr200Handle {
    rx: Consumer<f32>,
    ctrl: Sender<Ctrl>,
    shared: Arc<Shared>,
    opened_at: Instant,
    join: Option<JoinHandle<()>>,
    /// Description for logs and the UI, filled in at open time.
    pub label: String,
    /// The rate actually achieved — `adc_clock_hz / 2^(decimation_exp+1)`,
    /// read back from the device rather than recomputed, so it can never
    /// disagree with what is actually on the wire.
    pub sample_rate_hz: f64,
    /// Both ADCs are running (`Rsr200ChannelMode::Separate`) — the ring
    /// holds [`QUAD`]-wide quadruples rather than pairs. Fixed for the life
    /// of the handle: the wire shape is chosen at open time.
    dual: bool,
}

impl Rsr200Handle {
    /// Connect to the radio and start streaming. Blocks until that has
    /// either succeeded or failed.
    pub fn open(cfg: &Rsr200Config, center_hz: f64) -> Result<Rsr200Handle> {
        crate::stream::spawn(cfg, center_hz)
    }

    #[allow(clippy::too_many_arguments)]
    pub(crate) fn from_parts(
        rx: Consumer<f32>,
        ctrl: Sender<Ctrl>,
        shared: Arc<Shared>,
        join: JoinHandle<()>,
        label: String,
        sample_rate_hz: f64,
        dual: bool,
    ) -> Rsr200Handle {
        Rsr200Handle { rx, ctrl, shared, opened_at: Instant::now(), join: Some(join), label, sample_rate_hz, dual }
    }

    /// Whether the stream thread is still running.
    pub fn is_alive(&self) -> bool {
        self.shared.alive.load(Ordering::Relaxed)
    }

    /// How long the radio has gone without delivering samples, measured
    /// from the last block or — if none ever arrived — from when it was
    /// opened.
    pub fn silent_for(&self) -> Duration {
        let since_open = self.opened_at.elapsed();
        let last = Duration::from_millis(self.shared.last_rx_ms.load(Ordering::Relaxed));
        since_open.saturating_sub(last)
    }

    /// Both ADCs are running and the ring holds quadruples — see
    /// [`Self::read_pair`], which is what `rsr200_source.rs` calls either
    /// way.
    pub fn dual(&self) -> bool {
        self.dual
    }

    /// Drain interleaved I,Q floats into `out`. Always returns an even
    /// count. Zero means nothing is available yet.
    ///
    /// With both ADCs running the ring holds quadruples, not pairs — this
    /// is the wrong door then; use [`Self::read_pair`].
    pub fn rx_read(&mut self, out: &mut [f32]) -> usize {
        let take = self.rx.slots().min(out.len()) & !1;
        let mut n = 0;
        while n < take {
            match self.rx.pop() {
                Ok(v) => {
                    out[n] = v;
                    n += 1;
                }
                Err(_) => break,
            }
        }
        n
    }

    /// Read both ADCs at once, as interleaved complex samples, and return
    /// how many **complex samples** landed in each. With one ADC running
    /// `aux` is left untouched and this is the ordinary read.
    ///
    /// The pair is sample-aligned by construction — see this module's own
    /// doc for why, unlike an RSPduo's two independently-arriving tuner
    /// callbacks, there is nothing to reconcile here.
    pub fn read_pair(&mut self, main: &mut [f32], aux: &mut [f32]) -> usize {
        if !self.dual {
            return self.rx_read(main) / 2;
        }
        let want = (main.len() / 2).min(aux.len() / 2).min(self.rx.slots() / QUAD);
        if want == 0 {
            return 0;
        }
        let Ok(chunk) = self.rx.read_chunk(want * QUAD) else {
            return 0;
        };
        let (head, tail) = chunk.as_slices();
        for (idx, &v) in head.iter().chain(tail.iter()).enumerate() {
            let (sample, lane) = (idx / QUAD, idx % QUAD);
            match lane {
                0 => main[2 * sample] = v,
                1 => main[2 * sample + 1] = v,
                2 => aux[2 * sample] = v,
                _ => aux[2 * sample + 1] = v,
            }
        }
        chunk.commit_all();
        want
    }

    fn send(&self, c: Ctrl) {
        // A closed channel means the thread has exited; needs_reopen (in
        // rsr200_source.rs) picks that up from is_alive, so there is
        // nothing useful to do here.
        let _ = self.ctrl.send(c);
    }

    pub fn set_center_hz(&self, hz: f64) {
        self.send(Ctrl::Center(hz));
    }

    pub fn set_attenuator1_db(&self, db: i32) {
        self.send(Ctrl::Attenuator1(db));
    }

    pub fn set_attenuator2_db(&self, db: i32) {
        self.send(Ctrl::Attenuator2(db));
    }

    /// Stop the stream thread and disconnect, without dropping the handle.
    /// Blocks until the thread has closed the connection. Idempotent.
    pub fn release(&mut self) {
        let _ = self.ctrl.send(Ctrl::Shutdown);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

impl Drop for Rsr200Handle {
    fn drop(&mut self) {
        self.release();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pending_keeps_only_the_last_value_per_field() {
        let mut p = Pending::default();
        for hz in [10e6, 10.1e6, 10.2e6] {
            p.absorb(Ctrl::Center(hz));
        }
        p.absorb(Ctrl::Attenuator1(6));
        p.absorb(Ctrl::Attenuator1(12));
        assert_eq!(p.center, Some(10.2e6));
        assert_eq!(p.attenuator1, Some(12));
        assert!(!p.shutdown);
    }

    #[test]
    fn pending_shutdown_is_sticky() {
        let mut p = Pending::default();
        p.absorb(Ctrl::Shutdown);
        p.absorb(Ctrl::Center(10e6));
        assert!(p.shutdown, "a later message must not cancel a shutdown");
    }

    #[test]
    fn empty_pending_is_detected() {
        let mut p = Pending::default();
        assert!(p.is_empty());
        p.absorb(Ctrl::Attenuator2(0));
        assert!(!p.is_empty(), "even a zero-valued request is a request");
    }

    #[test]
    fn ring_capacity_is_even_and_at_least_half_a_second() {
        for rate in [130_560.0, 500_000.0, 2_000_000.0] {
            for stride in [2, QUAD] {
                let (p, _c) = ring_for(rate, stride);
                let cap = p.buffer().capacity();
                assert_eq!(cap % stride, 0, "capacity not a whole number of samples at {rate}, stride {stride}");
                assert!(
                    cap as f64 >= rate * stride as f64 * 0.5,
                    "ring holds {cap} floats, less than 0.5 s of {rate} sps at stride {stride}"
                );
            }
        }
    }

    #[test]
    fn push_iq_drops_whole_blocks_rather_than_splitting_pairs() {
        let (mut prod, cons) = RingBuffer::<f32>::new(8);
        assert!(push_iq(&mut prod, &[1.0, 2.0, 3.0, 4.0]));
        assert_eq!(cons.slots(), 4);

        assert!(!push_iq(&mut prod, &[5.0, 6.0, 7.0, 8.0, 9.0, 10.0]), "does not fit, must be refused whole");
        assert_eq!(cons.slots(), 4, "a partial write would desynchronise I and Q");
    }
}
