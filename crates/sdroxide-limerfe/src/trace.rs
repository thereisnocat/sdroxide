//! A bounded record of what the front end was told, and what it said back.
//!
//! The same idea as `sdroxide_lime::trace` next door, and here for a reason
//! that crate cannot cover: the LimeRFE's own USB cable is the link most people
//! use, and on it nothing this board does passes through LimeSuite at all. A
//! report from that setup would otherwise contain the radio's whole session and
//! not one word about the amplifier in front of it.
//!
//! What it records is the *decision and its outcome* rather than the bytes:
//! this driver deduplicates on the resolved state, so "nothing was sent" is
//! frequently the correct behaviour and always the confusing one. A report that
//! shows one configuration at open and one relay change per over is a report
//! that answers "why is there no output" — the channel, the connector and the
//! relay position are all in it.

use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

/// How many entries to keep. Enough for an opening configuration, a band
/// change or two and a run of overs; a long healthy session is not what
/// anybody is reporting.
const CAP: usize = 128;

#[derive(Clone)]
struct Entry {
    at_ms: u128,
    what: String,
    outcome: String,
}

#[derive(Clone)]
pub struct Trace {
    inner: Arc<Mutex<Inner>>,
}

struct Inner {
    started: Instant,
    link: String,
    entries: std::collections::VecDeque<Entry>,
    dropped: u64,
}

impl Default for Trace {
    fn default() -> Self {
        Self::new()
    }
}

impl Trace {
    pub fn new() -> Trace {
        Trace {
            inner: Arc::new(Mutex::new(Inner {
                started: Instant::now(),
                link: String::new(),
                entries: std::collections::VecDeque::with_capacity(CAP),
                dropped: 0,
            })),
        }
    }

    /// Which cable this board is on, and what it said it is.
    pub fn set_link(&self, link: impl AsRef<str>) {
        let mut i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        i.link = link.as_ref().to_string();
    }

    /// Record one transaction, or one thing that happened to the link.
    pub fn note(&self, what: impl AsRef<str>, outcome: impl AsRef<str>) {
        let mut i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let at_ms = i.started.elapsed().as_millis();
        if i.entries.len() == CAP {
            i.entries.pop_front();
            i.dropped += 1;
        }
        i.entries.push_back(Entry {
            at_ms,
            what: what.as_ref().to_string(),
            outcome: outcome.as_ref().to_string(),
        });
    }

    pub fn dump(&self) -> String {
        let i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut out = String::new();
        if !i.link.is_empty() {
            out.push_str(&format!("{}\n", i.link));
        }
        if i.dropped > 0 {
            out.push_str(&format!("({} earlier entries dropped)\n", i.dropped));
        }
        for e in &i.entries {
            out.push_str(&format!("{:>7} ms  {:<64} {}\n", e.at_ms, e.what, e.outcome));
        }
        out
    }
}

fn last() -> &'static Mutex<Option<Trace>> {
    static T: OnceLock<Mutex<Option<Trace>>> = OnceLock::new();
    T.get_or_init(|| Mutex::new(None))
}

/// Remember this board's trace as the one a report should carry.
pub fn remember(trace: &Trace) {
    *last().lock().unwrap_or_else(|e| e.into_inner()) = Some(trace.clone());
}

/// What to put in a bug report about the front end. `None` before any board
/// has been opened, which is the ordinary case for the great majority of
/// operators and not worth a heading of its own.
pub fn diagnostics() -> Option<String> {
    let t = last().lock().unwrap_or_else(|e| e.into_inner()).clone()?;
    let dump = t.dump();
    if dump.is_empty() { None } else { Some(dump) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dump_carries_the_link_and_the_transactions() {
        let t = Trace::new();
        t.set_link("LimeRFE on /dev/ttyUSB0 (firmware 5, hardware 2)");
        t.note("2 m in on J3 (TX/RX), 2 m out on J4 (TX), Receive", "ok");
        t.note("relays: Transmit", "ok");
        let d = t.dump();
        assert!(d.contains("ttyUSB0"), "{d}");
        assert!(d.contains("J4 (TX)"), "{d}");
        assert!(d.contains("relays: Transmit"), "{d}");
    }

    #[test]
    fn an_overlong_session_reports_what_it_dropped() {
        let t = Trace::new();
        for i in 0..(CAP + 5) {
            t.note(format!("state {i}"), "ok");
        }
        let d = t.dump();
        assert!(d.contains("5 earlier entries dropped"), "{d}");
        assert!(!d.contains("state 0 "), "the oldest are gone: {d}");
    }
}
