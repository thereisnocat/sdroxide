//! Where a backend keeps the last session trace of each radio it has talked to.
//!
//! The trace has to outlive the source that produced it: a connection that
//! fails or misbehaves is usually replaced within seconds — by the engine's
//! background retry, or by the operator pressing Apply — and the trace of the
//! *interesting* session would go with it. So it is kept here, where Settings →
//! Radio can still offer it afterwards.
//!
//! One slot per radio, not one for the whole process. A station can have two of
//! the same kind of radio on it (two Icoms on the LAN is the ordinary case), and
//! a single slot hands whichever of them last hung up to whoever asks — so the
//! IC-9700's tab answers with the IC-7300's trace, which is worse than
//! answering with nothing: it reads as this radio's own conversation.
//!
//! The key is the address the session was dialled at, which is what tells two
//! radios of one kind apart and is the same string on both sides of the
//! question: the source knows what it dialled, and the tab that asks knows what
//! its own `radio.json` says.

use std::sync::Mutex;

/// How many radios' traces to keep. A station has a handful, and a key changes
/// only when an address is retyped; past that the oldest is the one nobody is
/// going to ask about.
const KEEP: usize = 8;

/// One backend's traces, newest first.
pub struct TraceStore {
    inner: Mutex<Vec<(String, String)>>,
}

impl TraceStore {
    pub const fn new() -> TraceStore {
        TraceStore { inner: Mutex::new(Vec::new()) }
    }

    /// File the trace of the session dialled at `key`, replacing that radio's
    /// previous one.
    pub fn record(&self, key: &str, dump: String) {
        let mut list = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        list.retain(|(k, _)| k != key);
        list.insert(0, (key.to_string(), dump));
        list.truncate(KEEP);
    }

    /// The trace of the radio at `key`, if one has run. Never another radio's:
    /// see the module doc.
    pub fn get(&self, key: &str) -> Option<String> {
        let list = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        list.iter().find(|(k, _)| k == key).map(|(_, dump)| dump.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn each_radio_gets_its_own_trace_and_neither_answers_for_the_other() {
        let store = TraceStore::new();
        store.record("192.168.0.10:50001", "the 7300's session".into());
        store.record("192.168.0.124:50001", "the 9700's session".into());

        assert_eq!(store.get("192.168.0.10:50001").as_deref(), Some("the 7300's session"));
        assert_eq!(store.get("192.168.0.124:50001").as_deref(), Some("the 9700's session"));
        // The radio nobody has connected to answers with nothing at all, rather
        // than with whichever session happened to be the most recent.
        assert_eq!(store.get("192.168.0.7:50001"), None);
    }

    #[test]
    fn a_radios_latest_session_replaces_its_previous_one() {
        let store = TraceStore::new();
        store.record("radio:1", "first".into());
        store.record("radio:1", "second".into());
        assert_eq!(store.get("radio:1").as_deref(), Some("second"));
        assert_eq!(store.inner.lock().unwrap().len(), 1, "one slot per radio");
    }

    #[test]
    fn the_oldest_radio_falls_off_rather_than_growing_without_bound() {
        let store = TraceStore::new();
        for i in 0..KEEP + 3 {
            store.record(&format!("radio:{i}"), format!("session {i}"));
        }
        assert_eq!(store.inner.lock().unwrap().len(), KEEP);
        assert_eq!(store.get("radio:0"), None, "the first one asked about long ago");
        assert!(store.get(&format!("radio:{}", KEEP + 2)).is_some(), "the newest is kept");
    }
}
