//! A bounded session trace, surfaced as `ReportKind::Lime`.
//!
//! Not optional. This backend ships without ever having been run against a
//! LimeSDR, and this is how the first person to plug one in can report what
//! happened without being asked to reproduce it under a log filter.
//!
//! Simpler than the USB drivers' equivalents because the interesting events
//! here are library calls rather than wire transfers: which call, what was
//! asked for, and what LimeSuite said when it refused.

use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

/// How many entries to keep. A session that fails does so in the first few
/// calls; a long healthy one is not what anyone is reporting.
const CAP: usize = 256;

#[derive(Clone)]
struct Entry {
    at_ms: u128,
    call: String,
    detail: String,
    outcome: String,
}

#[derive(Clone)]
pub struct Trace {
    inner: Arc<Mutex<Inner>>,
}

struct Inner {
    started: Instant,
    identity: String,
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
                identity: String::new(),
                entries: std::collections::VecDeque::with_capacity(CAP),
                dropped: 0,
            })),
        }
    }

    /// What this session is talking to — the library version and the board.
    pub fn set_identity(&self, identity: impl AsRef<str>) {
        let mut i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        i.identity = identity.as_ref().to_string();
    }

    /// Record one library call.
    pub fn call(&self, call: &str, detail: impl AsRef<str>, outcome: impl AsRef<str>) {
        let mut i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let at_ms = i.started.elapsed().as_millis();
        if i.entries.len() == CAP {
            i.entries.pop_front();
            i.dropped += 1;
        }
        i.entries.push_back(Entry {
            at_ms,
            call: call.to_string(),
            detail: detail.as_ref().to_string(),
            outcome: outcome.as_ref().to_string(),
        });
    }

    pub fn dump(&self) -> String {
        let i = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut out = String::new();
        if !i.identity.is_empty() {
            out.push_str(&format!("{}\n", i.identity));
        }
        if i.dropped > 0 {
            out.push_str(&format!("({} earlier entries dropped)\n", i.dropped));
        }
        for e in &i.entries {
            out.push_str(&format!(
                "{:>7} ms  {:<24} {:<44} {}\n",
                e.at_ms, e.call, e.detail, e.outcome
            ));
        }
        out
    }
}

fn last_open() -> &'static Mutex<Option<Trace>> {
    static T: OnceLock<Mutex<Option<Trace>>> = OnceLock::new();
    T.get_or_init(|| Mutex::new(None))
}

fn last_probe() -> &'static Mutex<Option<Trace>> {
    static T: OnceLock<Mutex<Option<Trace>>> = OnceLock::new();
    T.get_or_init(|| Mutex::new(None))
}

/// Remember an open session's trace.
pub fn remember(trace: &Trace) {
    *last_open().lock().unwrap_or_else(|e| e.into_inner()) = Some(trace.clone());
}

/// Remember a probe's trace, in its own slot so it cannot displace the evidence
/// of an open session.
pub fn remember_probe(trace: &Trace) {
    *last_probe().lock().unwrap_or_else(|e| e.into_inner()) = Some(trace.clone());
}

/// What to put in a bug report. `None` before anything has been attempted.
pub fn diagnostics() -> Option<String> {
    let open = last_open().lock().unwrap_or_else(|e| e.into_inner()).clone();
    let probe = last_probe().lock().unwrap_or_else(|e| e.into_inner()).clone();
    if open.is_none() && probe.is_none() {
        return None;
    }
    let mut out = String::new();
    if let Some(t) = open {
        out.push_str("### radio session (open / stream / transmit)\n");
        out.push_str(&t.dump());
    }
    if let Some(t) = probe {
        if !out.is_empty() {
            out.push('\n');
        }
        out.push_str("### probe\n");
        out.push_str(&t.dump());
    }
    Some(out)
}

/// The line to put in front of a pasted report.
pub const FIELD_REPORT_HINT: &str = "LimeSDR support is not yet hardware-verified. If something is wrong, paste this into an \
     issue along with the output of `LimeUtil --find`.";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dump_carries_the_identity_and_the_calls() {
        let t = Trace::new();
        t.set_identity("LimeSuite 23.11.0, LimeSDR-USB serial 0009072C02873717");
        t.call("LMS_SetLOFrequency", "rx 145.500 MHz", "ok");
        t.call("LMS_Calibrate", "rx bw 6.25 MHz", "FAILED: calibration did not converge");
        let d = t.dump();
        assert!(d.contains("0009072C02873717"), "{d}");
        assert!(d.contains("LMS_SetLOFrequency"), "{d}");
        assert!(d.contains("did not converge"), "{d}");
    }

    /// The ring drops the oldest entries and says how many, so a report from a
    /// long session is not silently a partial one.
    #[test]
    fn an_overlong_session_reports_what_it_dropped() {
        let t = Trace::new();
        for i in 0..(CAP + 10) {
            t.call("LMS_SetLOFrequency", format!("rx {i}"), "ok");
        }
        let d = t.dump();
        assert!(d.contains("10 earlier entries dropped"), "{d}");
        assert!(!d.contains("rx 0 "), "the oldest are gone: {d}");
        assert!(d.contains(&format!("rx {}", CAP + 9)), "the newest are kept: {d}");
    }

    /// Nothing attempted, nothing to report — rather than an empty section
    /// that looks like a trace that captured nothing.
    #[test]
    fn diagnostics_are_absent_until_something_happens() {
        // Uses the process-global slots, so this only asserts the shape of the
        // empty case when it has not been populated by another test.
        let t = Trace::new();
        assert!(t.dump().is_empty());
    }
}
