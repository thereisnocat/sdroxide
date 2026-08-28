//! The station's transmit interlock: at most one radio keyed at any moment.
//!
//! One shared [`TxGate`] is handed to every engine in the process. It is not a
//! hardware arbiter — each engine still talks to its own front end — it is the
//! operator-level rule that keying radio B while radio A is on the air is a
//! mistake, refused with a notice rather than acted on.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

/// Change signal for the station-shared stores (memories, folders, band
/// stacks, digi operator config), shared by every engine in the process the
/// way [`TxGate`] is.
///
/// Each engine holds those stores as whole in-memory copies and writes them
/// back whole, so an engine that saved must tell the others to re-read or the
/// next engine to save would clobber the change. The writer bumps the
/// generation; every engine compares it against the last value it saw once per
/// loop tick — an atomic load — and reloads from disk when it moved. Purely
/// engine-side, so it also covers saves no UI drove, like a band change from a
/// CAT dial spun on a radio whose tab is not focused.
#[derive(Debug, Default)]
pub struct StoreSync {
    generation: AtomicU64,
}

impl StoreSync {
    pub fn new() -> Self {
        StoreSync::default()
    }

    /// Announce a completed write. Returns the new generation, which the
    /// writer records as already-seen so it does not reload its own save.
    pub fn bump(&self) -> u64 {
        self.generation.fetch_add(1, Ordering::AcqRel) + 1
    }

    pub fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }
}

/// See the module doc. `0` = free; otherwise the holder's radio id + 1, so
/// radio id 0 is representable.
#[derive(Debug, Default)]
pub struct TxGate {
    owner: AtomicU32,
}

impl TxGate {
    pub fn new() -> Self {
        TxGate::default()
    }

    /// Claim the gate for `radio`. True when it was free or already ours —
    /// re-acquiring is normal (PTT while tuning, the digi sequencer keying
    /// again mid-over), so it must not deadlock the holder out.
    pub fn try_acquire(&self, radio: u32) -> bool {
        let tag = radio + 1;
        self.owner
            .compare_exchange(0, tag, Ordering::AcqRel, Ordering::Acquire)
            .map(|_| true)
            .unwrap_or_else(|held| held == tag)
    }

    /// Release the gate if `radio` holds it. Releasing someone else's claim is
    /// refused silently: an unkey on a radio that never keyed must not free the
    /// gate out from under the radio that did.
    pub fn release(&self, radio: u32) {
        let tag = radio + 1;
        let _ = self.owner.compare_exchange(tag, 0, Ordering::AcqRel, Ordering::Acquire);
    }

    /// The radio currently holding the gate, if any.
    pub fn holder(&self) -> Option<u32> {
        match self.owner.load(Ordering::Acquire) {
            0 => None,
            tag => Some(tag - 1),
        }
    }
}

/// Which radio, if any, is on FreeDV — shared by every engine in the process
/// the way [`TxGate`] is.
///
/// FreeDV Reporter is a *station's* entry on a website, and only the primary
/// engine holds the session (see `SpotManager::stand_down`). But the radio in
/// RADE need not be that one, and the reporter has to be told the frequency and
/// transmit state of whichever radio is actually working FreeDV. Without this,
/// an operator running RADE on their second radio pushed "in RADE" into a spot
/// manager with no session in it, while the primary engine — sitting in SSB —
/// kept the station hidden.
///
/// First claim wins and holds until it leaves RADE, so two radios in RADE at
/// once report the one that got there first rather than flapping between them.
/// `0` = nobody; otherwise the holder's radio id + 1, as in [`TxGate`].
#[derive(Debug, Default)]
pub struct RadeWatch {
    owner: AtomicU32,
    /// The holder's transmit frequency in Hz — where the site should show it.
    freq_hz: AtomicU64,
    tx: AtomicBool,
}

impl RadeWatch {
    pub fn new() -> Self {
        RadeWatch::default()
    }

    /// Publish one radio's FreeDV state. Called every engine tick, so it must
    /// stay to a couple of atomics in the common case.
    pub fn publish(&self, radio: u32, in_rade: bool, tx_freq_hz: u64, transmitting: bool) {
        let tag = radio + 1;
        if !in_rade {
            // Only ever our own claim: releasing another radio's would hide a
            // station that is still on the air.
            let _ = self.owner.compare_exchange(tag, 0, Ordering::AcqRel, Ordering::Acquire);
            return;
        }
        let ours = match self.owner.compare_exchange(0, tag, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => true,
            Err(held) => held == tag,
        };
        if ours {
            self.freq_hz.store(tx_freq_hz, Ordering::Relaxed);
            self.tx.store(transmitting, Ordering::Relaxed);
        }
    }

    /// What to report: the holder's `(frequency, transmitting)`, or `None` when
    /// no radio is in RADE and the station should be hidden.
    pub fn reported(&self) -> Option<(u64, bool)> {
        (self.owner.load(Ordering::Acquire) != 0)
            .then(|| (self.freq_hz.load(Ordering::Relaxed), self.tx.load(Ordering::Relaxed)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_radio_at_a_time_and_reacquiring_is_not_a_deadlock() {
        let gate = TxGate::new();
        assert_eq!(gate.holder(), None);
        assert!(gate.try_acquire(0), "a free gate must key");
        assert!(gate.try_acquire(0), "the holder keying again must not be refused");
        assert!(!gate.try_acquire(1), "a second radio must be refused while the first is keyed");
        assert_eq!(gate.holder(), Some(0));

        gate.release(1);
        assert_eq!(gate.holder(), Some(0), "an unkey on a radio that never keyed frees nothing");
        gate.release(0);
        assert_eq!(gate.holder(), None);
        assert!(gate.try_acquire(1), "released, the gate serves the next radio");
        assert_eq!(gate.holder(), Some(1));
    }

    #[test]
    fn radio_id_zero_is_distinguishable_from_free() {
        let gate = TxGate::new();
        assert!(gate.try_acquire(0));
        assert_eq!(gate.holder(), Some(0), "id 0 must not read back as \"nobody\"");
    }

    /// The case this type exists for: the radio in RADE is not the radio that
    /// holds the FreeDV Reporter session.
    #[test]
    fn a_second_radio_in_rade_is_what_gets_reported() {
        let w = RadeWatch::new();
        // Radio 0 (the primary, which owns the session) is in SSB.
        w.publish(0, false, 14_200_000, false);
        assert_eq!(w.reported(), None, "nobody on FreeDV yet");

        // Radio 1 goes to RADE. The station is on the air on *its* frequency.
        w.publish(1, true, 14_236_000, false);
        assert_eq!(w.reported(), Some((14_236_000, false)));

        // The primary keeps saying "not me" every tick; that must not hide it.
        w.publish(0, false, 14_200_000, false);
        assert_eq!(w.reported(), Some((14_236_000, false)));

        // Radio 1 keys, then retunes.
        w.publish(1, true, 14_236_000, true);
        assert_eq!(w.reported(), Some((14_236_000, true)));
        w.publish(1, true, 7_177_000, false);
        assert_eq!(w.reported(), Some((7_177_000, false)));

        // And leaves RADE: hidden again.
        w.publish(1, false, 7_177_000, false);
        assert_eq!(w.reported(), None);
    }

    #[test]
    fn the_first_radio_into_rade_holds_the_report() {
        let w = RadeWatch::new();
        w.publish(0, true, 14_236_000, false);
        w.publish(1, true, 7_177_000, false);
        assert_eq!(w.reported(), Some((14_236_000, false)), "the second must not take it over");

        // Once the first leaves, the second's next tick claims it.
        w.publish(0, false, 14_236_000, false);
        assert_eq!(w.reported(), None);
        w.publish(1, true, 7_177_000, false);
        assert_eq!(w.reported(), Some((7_177_000, false)));
    }
}
