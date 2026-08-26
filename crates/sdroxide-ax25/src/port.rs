//! The seam between a connected-mode link and whatever wants a byte stream.
//!
//! A Winlink forwarding session is written against `Read`/`Write` and blocks on
//! its own worker thread. The link layer lives on the engine's audio thread,
//! because that is where PTT and the sample clock are. This is the pair of ends
//! that joins them, and nothing but bytes and intentions crosses it.
//!
//! ```text
//! worker thread                         engine thread
//! ─────────────                         ─────────────
//! session::run_into(&mut transport)     PacketController
//!         │                                  ├─ ax25::state + timers
//!         └── PortHandle ⇄ channels ⇄ PortEndpoint ─┤
//!             Connect/Data/Disconnect            ├─ modem
//!             Connected/Data/Disconnected/Failed └─ CSMA + PTT
//! ```
//!
//! # Two deliberate choices
//!
//! **The request channel is bounded.** `Write::write` blocking when the window
//! is full *is* the backpressure: without it, B2F would hand over a whole
//! compressed message in a moment and the session would believe it had been
//! sent while nothing had left the antenna.
//!
//! **A lease, not a mutex.** Two owners interleaving on one link would produce
//! traffic neither understands, so [`PortHandle::try_claim`] hands out one lease
//! at a time and the second caller gets a clear refusal — the same shape as the
//! engine's transmit gate.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};

use crate::addr::Addr;

/// How many outstanding writes the session may run ahead by.
///
/// Eight frames' worth: enough that the link never starves waiting for the
/// session between overs, small enough that a session which stops reading
/// cannot bank a message the radio has not sent.
const WINDOW: usize = 8;

/// What the session asks the link to do.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PortRequest {
    Connect { peer: Addr, via: Vec<Addr>, ext: bool },
    Data(Vec<u8>),
    Disconnect,
}

/// What the link tells the session.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PortEvent {
    Connected,
    Data(Vec<u8>),
    /// A clean shutdown — DISC/UA, or the peer went away politely.
    Disconnected,
    /// The link gave up: retries exhausted, the mode changed under it, the
    /// operator aborted. Carries something worth putting in the session log.
    Failed(String),
}

/// Settings the link needs that are not the state machine's business.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LinkConfig {
    /// Our own address. A link cannot be opened without one.
    pub me: Addr,
    /// Longest information field we send.
    pub paclen: u16,
    /// Frames outstanding before an acknowledgement is required.
    pub maxframe: u8,
}

/// The session's end. Cloneable so a manager can hold one while a worker uses
/// another, but only one lease may be live at a time.
#[derive(Debug, Clone)]
pub struct PortHandle {
    tx: Sender<PortRequest>,
    rx: Receiver<PortEvent>,
    claimed: Arc<AtomicBool>,
}

/// Proof of exclusive use. Releases on drop.
#[derive(Debug)]
pub struct PortLease {
    claimed: Arc<AtomicBool>,
}

impl Drop for PortLease {
    fn drop(&mut self) {
        self.claimed.store(false, Ordering::SeqCst);
    }
}

/// Take exclusive use of a link, or `None` if somebody already has it.
///
/// Shared by both ends: the lease is one flag and either end may take it, so
/// the operator's terminal and a forwarding session exclude each other without
/// either knowing the other exists.
fn claim(flag: &Arc<AtomicBool>) -> Option<PortLease> {
    if flag.swap(true, Ordering::SeqCst) {
        return None;
    }
    Some(PortLease { claimed: Arc::clone(flag) })
}

impl PortHandle {
    /// Take exclusive use of the link, or `None` if somebody already has it.
    #[must_use]
    pub fn try_claim(&self) -> Option<PortLease> {
        claim(&self.claimed)
    }

    /// Whether two handles name the same link.
    ///
    /// A mode change destroys the controller and builds a fresh pair, so a
    /// holder of the old handle is holding a link whose other end is gone —
    /// and a handle carries no other identity to tell that from the new one.
    #[must_use]
    pub fn same_link(&self, other: &PortHandle) -> bool {
        Arc::ptr_eq(&self.claimed, &other.claimed)
    }

    /// Ask the link to do something. `Err` means the engine end is gone — the
    /// operator changed mode, or the radio was closed.
    pub fn send(&self, req: PortRequest) -> Result<(), PortClosed> {
        self.tx.send(req).map_err(|_| PortClosed)
    }

    /// Non-blocking send, for a caller that must not stall.
    pub fn try_send(&self, req: PortRequest) -> Result<(), TrySendError<PortRequest>> {
        self.tx.try_send(req)
    }

    /// Block for the next event. `Err` means the link is gone.
    pub fn recv(&self) -> Result<PortEvent, PortClosed> {
        self.rx.recv().map_err(|_| PortClosed)
    }

    /// Block for the next event, giving up after `timeout`.
    ///
    /// `Ok(None)` is "nothing yet"; `Err` is "the link is gone". Keeping those
    /// apart matters: a caller polling in short hops so it can notice an abort
    /// would otherwise loop for ever against a controller that has already been
    /// destroyed by a mode change, unable to tell that from a quiet channel.
    pub fn recv_timeout(
        &self,
        timeout: std::time::Duration,
    ) -> Result<Option<PortEvent>, PortClosed> {
        use crossbeam_channel::RecvTimeoutError;
        match self.rx.recv_timeout(timeout) {
            Ok(ev) => Ok(Some(ev)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(PortClosed),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error("the packet link is gone")]
pub struct PortClosed;

/// The engine's end, held by the controller.
#[derive(Debug)]
pub struct PortEndpoint {
    rx: Receiver<PortRequest>,
    tx: Sender<PortEvent>,
    claimed: Arc<AtomicBool>,
    pub cfg: LinkConfig,
}

impl PortEndpoint {
    /// Take whatever the session has asked for. Never blocks — this runs on the
    /// audio thread, which owes the transmitter a block every 10 ms.
    pub fn take_requests(&self) -> Vec<PortRequest> {
        self.rx.try_iter().collect()
    }

    /// Take the link for this end — the operator, rather than a session.
    ///
    /// The same flag [`PortHandle::try_claim`] uses, so whichever of the two
    /// asks first gets it and the other is refused. One radio, one channel:
    /// two owners interleaving would produce traffic neither understands.
    #[must_use]
    pub fn try_claim(&self) -> Option<PortLease> {
        claim(&self.claimed)
    }

    /// Whether anybody holds the link right now.
    ///
    /// Also the answer to "should this station be answering calls?": while a
    /// forwarding session has the link, an incoming connect has nowhere to go.
    #[must_use]
    pub fn is_claimed(&self) -> bool {
        self.claimed.load(Ordering::SeqCst)
    }

    /// Tell the session something. **Never blocks.**
    ///
    /// `false` means the queue was full and the event is gone. It has to be
    /// checked: this runs on the audio thread, and the caller's answer to a
    /// full queue must be to fail the link, because a gap in a B2F stream is
    /// not noticed until the CRC at the end of the whole message.
    ///
    /// Blocking here instead — which is what `send` on a bounded channel does,
    /// and what this used to do — wedges the audio thread outright: the engine
    /// holds the receiving handle for the whole life of a packet mode, so the
    /// channel never disconnects and the send never returns.
    pub fn emit(&self, ev: PortEvent) -> bool {
        self.tx.try_send(ev).is_ok()
    }
}

/// Make a linked pair.
#[must_use]
pub fn port_pair(cfg: LinkConfig) -> (PortHandle, PortEndpoint) {
    let (req_tx, req_rx) = bounded(WINDOW);
    // Events are bounded too, and more generously: a burst of received frames
    // must not block the audio thread, but an unbounded queue would let a
    // stalled session grow one without limit.
    let (ev_tx, ev_rx) = bounded(WINDOW * 8);
    // One flag, cloned into both ends: see `PortEndpoint::try_claim`.
    let claimed = Arc::new(AtomicBool::new(false));
    (
        PortHandle { tx: req_tx, rx: ev_rx, claimed: Arc::clone(&claimed) },
        PortEndpoint { rx: req_rx, tx: ev_tx, claimed, cfg },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> LinkConfig {
        LinkConfig { me: Addr::new("OE3JJS-10").unwrap(), paclen: 128, maxframe: 4 }
    }

    #[test]
    fn requests_and_events_cross() {
        let (h, e) = port_pair(cfg());
        h.send(PortRequest::Data(b"hello".to_vec())).unwrap();
        assert_eq!(e.take_requests(), vec![PortRequest::Data(b"hello".to_vec())]);
        assert!(e.emit(PortEvent::Connected));
        assert_eq!(h.recv().unwrap(), PortEvent::Connected);
    }

    /// The window is the backpressure. Without it a session believes a whole
    /// message has been sent while none of it has left the antenna.
    #[test]
    fn the_request_window_is_bounded() {
        let (h, _e) = port_pair(cfg());
        for _ in 0..WINDOW {
            h.try_send(PortRequest::Data(vec![0; 8])).expect("within the window");
        }
        assert!(h.try_send(PortRequest::Data(vec![0; 8])).is_err(), "the window did not bound");
    }

    /// One owner at a time, and the lease comes back on drop.
    #[test]
    fn only_one_owner_at_a_time() {
        let (h, _e) = port_pair(cfg());
        let lease = h.try_claim().expect("first claim");
        assert!(h.try_claim().is_none(), "two owners on one link");
        drop(lease);
        assert!(h.try_claim().is_some(), "the lease did not come back");
    }

    /// The operator and a forwarding session are two owners of one radio, and
    /// they hold the link from opposite ends. Whichever asks first gets it.
    #[test]
    fn either_end_can_take_the_lease_and_the_other_is_refused() {
        let (h, e) = port_pair(cfg());
        assert!(!e.is_claimed());
        let lease = e.try_claim().expect("the controller's end may claim");
        assert!(e.is_claimed());
        assert!(h.try_claim().is_none(), "a session took a link the operator holds");
        drop(lease);
        assert!(!e.is_claimed());
        let lease = h.try_claim().expect("and back the other way");
        assert!(e.try_claim().is_none(), "the operator took a link a session holds");
        assert!(e.is_claimed(), "the controller cannot see that the link is busy");
        drop(lease);
    }

    /// `emit` runs on the audio thread. A session that stops reading must cost
    /// a refused event, not a stalled receiver — and the channel never
    /// disconnects, so a blocking send here would never return at all.
    #[test]
    fn a_full_event_queue_is_refused_not_a_block() {
        let (_h, e) = port_pair(cfg());
        // Run it on a thread and join with a deadline: a regression reports
        // "emit blocked" rather than hanging the whole test binary.
        let done = std::thread::spawn(move || {
            let mut refused = 0;
            for _ in 0..1000 {
                if !e.emit(PortEvent::Data(vec![0; 4])) {
                    refused += 1;
                }
            }
            refused
        });
        let start = std::time::Instant::now();
        while !done.is_finished() {
            assert!(start.elapsed() < std::time::Duration::from_secs(5), "emit blocked");
            std::thread::yield_now();
        }
        assert!(done.join().unwrap() > 0, "a bounded queue accepted a thousand events");
    }

    /// A mode change builds a fresh pair. Whoever kept the old handle is
    /// holding a link whose other end is gone, and nothing else says so.
    #[test]
    fn same_link_tells_a_replaced_port_from_the_original() {
        let (a, _ea) = port_pair(cfg());
        let (b, _eb) = port_pair(cfg());
        assert!(a.same_link(&a.clone()));
        assert!(!a.same_link(&b));
    }

    /// A dropped controller must surface as an error, not a hang: the mode
    /// changed under the session and it needs to unwind with a transcript.
    #[test]
    fn a_dropped_link_is_an_error_not_a_hang() {
        let (h, e) = port_pair(cfg());
        drop(e);
        assert_eq!(h.send(PortRequest::Disconnect), Err(PortClosed));
        assert_eq!(h.recv(), Err(PortClosed));
    }
}
