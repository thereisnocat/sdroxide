//! The AX.25 lane behind the `Transport` seam, with no radio.
//!
//! `session::run_into` is generic over `Read + Write` and has no idea a radio
//! exists, so a fake gateway on the other end of a `port_pair` exercises
//! `Ax25Transport` exactly as a real link would — the blocking read, the
//! clean-disconnect path, the failure path — while the B2F conversation itself
//! stays the one already pinned by `session.rs`'s own tests.
//!
//! What this does **not** prove is that a real RMS gateway answers. That needs
//! the air, and is the operator's step.

use std::io::{Read, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use sdroxide_ax25::{Addr, LinkConfig, PortEvent, PortRequest, port_pair};
use sdroxide_winlink::transport::{Ax25Transport, Transport};

/// A cancel flag nobody ever sets.
fn quiet() -> Arc<AtomicBool> {
    Arc::new(AtomicBool::new(false))
}

fn link() -> (sdroxide_ax25::PortHandle, sdroxide_ax25::PortEndpoint) {
    port_pair(LinkConfig { me: Addr::new("OE3JJS-10").unwrap(), paclen: 128, maxframe: 4 })
}

/// Wait for the client's connect request, then report the link up.
fn answer(endpoint: &sdroxide_ax25::PortEndpoint) {
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while std::time::Instant::now() < deadline {
        if endpoint.take_requests().iter().any(|r| matches!(r, PortRequest::Connect { .. })) {
            endpoint.emit(PortEvent::Connected);
            return;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    panic!("no connect request arrived");
}

/// The transport connects, carries bytes both ways, and names the gateway —
/// which is what `SessionConfig::target_call` now uses for the greeting instead
/// of the hard-coded `wl2k`.
#[test]
fn the_transport_carries_a_conversation() {
    let (handle, endpoint) = link();
    let g = std::thread::spawn(move || {
        answer(&endpoint);
        endpoint.emit(PortEvent::Data(b"hello from the gateway\r".to_vec()));

        let mut seen = String::new();
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while !seen.contains("MY TURN") && std::time::Instant::now() < deadline {
            for r in endpoint.take_requests() {
                if let PortRequest::Data(d) = r {
                    seen.push_str(&String::from_utf8_lossy(&d));
                }
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(seen.contains("MY TURN"), "the gateway never heard us: {seen:?}");
        endpoint.emit(PortEvent::Data(b"ack\r".to_vec()));
        endpoint.emit(PortEvent::Disconnected);
    });

    let mut t = Ax25Transport::connect(handle, "OE1XAR-10", &[], Duration::from_secs(5), quiet())
        .expect("connect");
    assert_eq!(t.target_call(), "OE1XAR-10");
    assert!(t.describe().contains("OE1XAR-10"));

    let mut buf = [0u8; 64];
    let n = t.read(&mut buf).expect("read");
    assert_eq!(&buf[..n], b"hello from the gateway\r");

    t.write_all(b"MY TURN\r").expect("write");
    let n = t.read(&mut buf).expect("read");
    assert_eq!(&buf[..n], b"ack\r");

    drop(t);
    g.join().unwrap();
}

/// A clean disconnect must surface as `Ok(0)` *after* the buffered bytes, not
/// instead of them. A gateway that sends its last block and hangs up in the
/// same breath is ordinary, and losing that block would lose the end of a
/// message — silently, because `Ok(0)` reads as a tidy end of session.
#[test]
fn a_clean_hangup_delivers_the_last_bytes_first() {
    let (handle, endpoint) = link();
    std::thread::spawn(move || {
        answer(&endpoint);
        endpoint.emit(PortEvent::Data(b"FQ\r".to_vec()));
        endpoint.emit(PortEvent::Disconnected);
    });

    let mut t = Ax25Transport::connect(handle, "OE1XAR-10", &[], Duration::from_secs(5), quiet())
        .expect("connect");
    let mut buf = [0u8; 16];
    let n = t.read(&mut buf).expect("read");
    assert_eq!(&buf[..n], b"FQ\r", "the last block was lost to the hangup");
    assert_eq!(t.read(&mut buf).expect("read"), 0, "a clean hangup must read as Ok(0)");
}

/// A link that gives up is an error, not an end of stream. The session has to
/// tell "the gateway said goodbye" from "the path died", and only the second is
/// worth reporting as a failure.
#[test]
fn a_failed_link_is_an_error_not_an_eof() {
    let (handle, endpoint) = link();
    std::thread::spawn(move || {
        answer(&endpoint);
        endpoint.emit(PortEvent::Failed("retries exhausted".into()));
    });

    let mut t = Ax25Transport::connect(handle, "OE1XAR-10", &[], Duration::from_secs(5), quiet())
        .expect("connect");
    let mut buf = [0u8; 16];
    let e = t.read(&mut buf).expect_err("a dead link must not read as EOF");
    assert_eq!(e.kind(), std::io::ErrorKind::BrokenPipe);
    assert!(e.to_string().contains("retries exhausted"), "the reason must survive: {e}");
}

/// A gateway that never answers times out with something the operator can read,
/// rather than blocking the worker for ever.
#[test]
fn an_unanswered_call_times_out() {
    let (handle, _endpoint) = link();
    let e = Ax25Transport::connect(handle, "OE1XAR-10", &[], Duration::from_millis(200), quiet())
        .expect_err("an unanswered call must not succeed");
    assert!(e.to_string().contains("OE1XAR-10"), "the error should name the gateway: {e}");
}

/// One session at a time on one radio.
#[test]
fn a_second_session_is_refused() {
    let (handle, endpoint) = link();
    std::thread::spawn(move || {
        answer(&endpoint);
        std::thread::sleep(Duration::from_secs(2));
    });

    let first =
        Ax25Transport::connect(handle.clone(), "OE1XAR-10", &[], Duration::from_secs(5), quiet())
            .expect("first");
    let e = Ax25Transport::connect(handle, "OE1XAR-10", &[], Duration::from_millis(200), quiet())
        .expect_err("two sessions took one radio");
    assert!(e.to_string().contains("already in use"), "{e}");
    drop(first);
}

/// A mistyped callsign is caught before anything is keyed.
#[test]
fn a_bad_gateway_callsign_is_refused_up_front() {
    let (handle, _endpoint) = link();
    let e = Ax25Transport::connect(handle, "not a callsign", &[], Duration::from_secs(1), quiet())
        .expect_err("garbage accepted as a callsign");
    assert!(e.to_string().to_lowercase().contains("callsign"), "{e}");
}

// ─────────────────────────── stopping a call ───────────────────────────

/// The operator changes their mind while a gateway is being called.
///
/// Reported from the field: there was no way to stop a call, and a dial that
/// takes two minutes to time out is two minutes of watching a window that says
/// "calling OE1XIK…". The flag is checked every quarter second, so this must
/// return in well under a second rather than at the deadline.
#[test]
fn an_abort_stops_a_call_in_progress() {
    let (handle, _endpoint) = link();
    let cancel = quiet();

    let c = Arc::clone(&cancel);
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(150));
        c.store(true, Ordering::SeqCst);
    });

    let started = std::time::Instant::now();
    // A dial timeout long enough that finishing early can only be the abort.
    let e = Ax25Transport::connect(handle, "OE1XAR-10", &[], Duration::from_secs(60), cancel)
        .expect_err("the abort did not stop the call");
    let took = started.elapsed();
    assert!(took < Duration::from_secs(2), "the abort took {took:?} to take effect");
    assert_eq!(e.to_string().to_lowercase().contains("stopped"), true, "{e}");
}

/// And once connected: a session stuck waiting for a gateway that has gone
/// quiet must still be stoppable, because that is where a real session spends
/// most of its time.
#[test]
fn an_abort_stops_a_session_waiting_on_a_quiet_link() {
    let (handle, endpoint) = link();
    let cancel = quiet();
    std::thread::spawn(move || {
        answer(&endpoint);
        // Then say nothing at all, holding the endpoint alive.
        std::thread::sleep(Duration::from_secs(30));
    });

    let mut t = Ax25Transport::connect(
        handle,
        "OE1XAR-10",
        &[],
        Duration::from_secs(5),
        Arc::clone(&cancel),
    )
    .expect("connect");

    let c = Arc::clone(&cancel);
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(150));
        c.store(true, Ordering::SeqCst);
    });

    let started = std::time::Instant::now();
    let mut buf = [0u8; 16];
    let e = t.read(&mut buf).expect_err("a stopped read must not block for ever");
    assert!(started.elapsed() < Duration::from_secs(2), "the abort was not noticed promptly");
    assert_eq!(e.kind(), std::io::ErrorKind::Interrupted);
}

/// A controller destroyed under a running session — the operator changed mode —
/// must fail the read rather than wait for ever on a link with no other end.
#[test]
fn a_vanished_controller_fails_the_read() {
    let (handle, endpoint) = link();
    std::thread::spawn(move || {
        answer(&endpoint);
        // Drop the endpoint: the mode changed and the controller is gone.
    });

    let mut t = Ax25Transport::connect(handle, "OE1XAR-10", &[], Duration::from_secs(5), quiet())
        .expect("connect");
    let mut buf = [0u8; 16];
    let e = t.read(&mut buf).expect_err("a vanished controller must not read as EOF");
    assert_eq!(e.kind(), std::io::ErrorKind::BrokenPipe);
}

/// One radio, one link. When the operator's terminal has it, a forwarding
/// session must be refused rather than interleaving on it.
#[test]
fn the_transport_is_refused_when_the_controller_holds_the_lease() {
    let (handle, endpoint) = link();
    // What `PacketController` does when the operator presses CONNECT.
    let _theirs = endpoint.try_claim().expect("the link was already claimed");
    let e = Ax25Transport::connect(handle, "OE1XAB-8", &[], Duration::from_secs(2), quiet())
        .expect_err("a session took a link the operator is using");
    assert!(format!("{e}").contains("already in use"), "the refusal did not say why: {e}");
}

/// `PortEndpoint::emit` no longer blocks, so a session that stops reading loses
/// events instead of stalling the audio thread. Losing them silently is the
/// worse failure of the two: B2F would carry on against a stream with a hole
/// in it and not find out until the CRC at the end of the whole message. The
/// link has to die loudly instead.
#[test]
fn a_full_event_queue_is_refused_rather_than_blocking_the_link() {
    let (handle, endpoint) = link();
    let _lease = handle.try_claim().expect("the link was already claimed");
    // Nobody reads `handle`, so the queue fills and stays full.
    let mut refused = 0;
    let start = std::time::Instant::now();
    for _ in 0..500 {
        if !endpoint.emit(PortEvent::Data(vec![0; 32])) {
            refused += 1;
        }
        assert!(start.elapsed() < Duration::from_secs(5), "emit blocked the caller");
    }
    assert!(refused > 0, "a bounded queue accepted five hundred events");
}
