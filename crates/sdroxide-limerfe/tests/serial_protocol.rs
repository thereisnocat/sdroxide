//! The serial protocol, against a fake LimeRFE on a pty.
//!
//! This test exists because of a field report: the first release of this
//! backend could not talk to a real board at all, and every unit test passed.
//! They passed because they checked the bytes against *my reading* of
//! LimeSuite's protocol rather than against something that behaves like the
//! firmware — and the reading was wrong in five places, the fatal one being
//! that the hello handshake is a single byte rather than a 16-byte frame.
//!
//! So the fake below is written to be **strict about lengths**: it answers a
//! one-byte hello with one byte, a two-byte `MODE`/`FAN` with two, and
//! everything else with sixteen. Anything the driver sends beyond what a
//! command calls for stays in the input and desynchronises the next exchange,
//! which is exactly how the real thing failed.
//!
//! Unix only: it needs a pty.

#![cfg(unix)]

use std::io::{Read, Write};
use std::os::fd::{FromRawFd, OwnedFd};
use std::time::Duration;

use sdroxide_limerfe::{RfeState, RfeTransport, SerialTransport};
use sdroxide_types::{RfeChannel, RfeMode, RfePort};

const HELLO: u8 = 0x00;
const MODE: u8 = 0xd1;
const CONFIG: u8 = 0xd2;
const FAN: u8 = 0xc1;
const GET_INFO: u8 = 0xe1;
const GET_CONFIG: u8 = 0xe3;

/// Firmware 4 on purpose: 4 is also `RFE_ERROR_CELL_WRONG_MODE`, so a driver
/// that tests `buf[1]` of a `GET_INFO` reply as a status turns this board's
/// version into a refusal about cellular bands.
const FW: u8 = 4;
const HW: u8 = 7;

fn reply_len(cmd: u8) -> usize {
    if cmd == MODE || cmd == FAN { 2 } else { 16 }
}

/// Open a pty pair, returning the master and the slave's path.
fn open_pty() -> (std::fs::File, String) {
    unsafe {
        let mut master: libc::c_int = 0;
        let mut slave: libc::c_int = 0;
        let rc = libc::openpty(
            &mut master,
            &mut slave,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        assert_eq!(rc, 0, "openpty failed");
        let mut name = [0i8; 256];
        assert_eq!(libc::ttyname_r(slave, name.as_mut_ptr(), name.len()), 0);
        let path = std::ffi::CStr::from_ptr(name.as_ptr()).to_string_lossy().into_owned();
        // The slave stays open for the pty's lifetime; the driver opens the
        // path itself.
        let keep = OwnedFd::from_raw_fd(slave);
        std::mem::forget(keep);
        // Non-blocking, so the server thread can notice it has been asked to
        // stop. A blocking read on a pty nothing writes to never returns, and
        // the test harness would wait for that thread forever.
        let flags = libc::fcntl(master, libc::F_GETFL);
        libc::fcntl(master, libc::F_SETFL, flags | libc::O_NONBLOCK);
        (std::fs::File::from_raw_fd(master), path)
    }
}

/// Serve the fake board until the channel says to stop.
fn serve(mut master: std::fs::File, stop: std::sync::Arc<std::sync::atomic::AtomicBool>) {
    use std::sync::atomic::Ordering;
    let mut buf: Vec<u8> = Vec::new();
    let mut state = [0u8; 16];
    let mut chunk = [0u8; 256];
    while !stop.load(Ordering::Relaxed) {
        match master.read(&mut chunk) {
            Ok(0) => break,
            Ok(n) => buf.extend_from_slice(&chunk[..n]),
            // Nothing waiting: the pty is non-blocking so this is the idle
            // case, not a failure.
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                std::thread::sleep(Duration::from_millis(2));
                continue;
            }
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(_) => break,
        }
        loop {
            let Some(&cmd) = buf.first() else { break };
            if cmd == HELLO {
                buf.remove(0);
                let _ = master.write_all(&[HELLO]);
                continue;
            }
            let need = reply_len(cmd);
            if buf.len() < need {
                break;
            }
            let frame: Vec<u8> = buf.drain(..need).collect();
            let mut out = vec![0u8; need];
            out[0] = cmd;
            match cmd {
                CONFIG => {
                    state[..frame.len()].copy_from_slice(&frame);
                    out[1] = 0;
                }
                MODE => {
                    state[5] = frame[1];
                    out[1] = 0;
                }
                FAN => out[1] = 0,
                GET_INFO => {
                    out[1] = FW;
                    out[2] = HW;
                }
                GET_CONFIG => out[1..10].copy_from_slice(&state[1..10]),
                _ => {}
            }
            let _ = master.write_all(&out);
        }
    }
}

struct Fake {
    path: String,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl Fake {
    fn start() -> Fake {
        let (master, path) = open_pty();
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let s = std::sync::Arc::clone(&stop);
        let join = std::thread::spawn(move || serve(master, s));
        Fake { path, stop, join: Some(join) }
    }
}

impl Drop for Fake {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

/// The whole point: a board that answers the way LimeSuite's source says it
/// does is reachable. A 16-byte hello would get one byte back and hang here.
#[test]
fn a_board_that_behaves_like_the_firmware_is_reachable() {
    let fake = Fake::start();
    let rfe = SerialTransport::open(&fake.path).expect("the handshake completes");
    assert!(rfe.describe().contains("firmware 4"), "{}", rfe.describe());
}

/// Firmware 4 must read as a version, not as `RFE_ERROR_CELL_WRONG_MODE`.
/// Only `CONFIG` and `MODE` carry a status in `buf[1]`.
#[test]
fn a_firmware_version_is_not_mistaken_for_an_error_code() {
    let fake = Fake::start();
    let mut rfe = SerialTransport::open(&fake.path).unwrap();
    let info = rfe.info().expect("GET_INFO has no status byte to trip over");
    assert_eq!((info.firmware, info.hardware), (FW, HW));
}

/// `MODE` is a two-byte frame. Sending sixteen leaves fourteen bytes in the
/// board's input, and the *next* command is then read as garbage — so this
/// asserts the exchange after it still works, which is what catches the length
/// error rather than merely the first symptom.
#[test]
fn a_mode_change_is_two_bytes_and_leaves_the_link_in_step() {
    let fake = Fake::start();
    let mut rfe = SerialTransport::open(&fake.path).unwrap();
    rfe.set_mode(RfeMode::Tx).expect("mode accepted");
    // If MODE had been sent as 16 bytes, the fake would still be chewing
    // through the padding and this would time out.
    let info = rfe.info().expect("the link is still in step after a mode change");
    assert_eq!(info.firmware, FW);
    rfe.set_fan(true).expect("fan is the same two-byte shape");
    assert_eq!(rfe.info().unwrap().firmware, FW, "and still in step after that");
}

/// A full configuration round trip, and the state read back through the
/// board's own reply layout.
#[test]
fn a_configuration_survives_the_round_trip() {
    let fake = Fake::start();
    let mut rfe = SerialTransport::open(&fake.path).unwrap();
    let state = RfeState {
        channel_rx: RfeChannel::Ham0145,
        channel_tx: RfeChannel::Ham0435,
        port_rx: RfePort::J3,
        port_tx: RfePort::J4,
        mode: RfeMode::TxRx,
        notch: true,
        atten_steps: 3,
        swr_enable: false,
        swr_source_cell: false,
    };
    rfe.configure(state).expect("configure accepted");
    // And the link is still usable afterwards.
    assert_eq!(rfe.info().unwrap().firmware, FW);
}

/// Nothing on the other end is a clean refusal within the handshake's own
/// patience, not a hang.
#[test]
fn a_silent_port_gives_up_rather_than_hanging() {
    let (_master, path) = open_pty();
    // No server thread: the pty accepts writes and never answers.
    let started = std::time::Instant::now();
    let err = match SerialTransport::open(&path) {
        Ok(_) => panic!("nothing is listening, so this must not succeed"),
        Err(e) => e,
    };
    assert!(err.to_string().contains("no LimeRFE answered"), "{err}");
    // Ten attempts, 200 ms apart, plus the reads between them.
    assert!(started.elapsed() < Duration::from_secs(15), "gave up in {:?}", started.elapsed());
}

/// An over, from the driver's own thread, ends up in the diagnostic report.
///
/// The field report this answers is "I receive on every band but the power
/// meter never moves", and the only way to act on it is to know what the front
/// end was told: which channel, which connector, and which way the relays went
/// at key-down. On the board's own serial cable none of that passes through
/// LimeSuite, so this record is the only one there is.
#[test]
fn what_the_board_was_told_survives_for_a_report() {
    let fake = Fake::start();
    let transport = SerialTransport::open(&fake.path).expect("the handshake completes");
    let cfg = sdroxide_types::LimeRfeConfig {
        link: sdroxide_types::RfeLink::Serial,
        serial: sdroxide_types::SerialConfig { path: fake.path.clone(), ..Default::default() },
        ..Default::default()
    };
    let handle = sdroxide_limerfe::spawn(Box::new(transport), cfg);
    handle.set_rx_hz(145.5e6);
    handle.set_tx_hz(145.5e6);
    // The opening configuration goes out at once; the relays wait for the
    // key-down. Both are one short transaction on this link.
    std::thread::sleep(Duration::from_millis(400));
    handle.set_keyed(true);
    std::thread::sleep(Duration::from_millis(400));
    handle.set_keyed(false);
    std::thread::sleep(Duration::from_millis(400));

    let report = sdroxide_limerfe::diagnostics().expect("a board has been driven");
    assert!(report.contains("firmware 4"), "the link identifies itself:\n{report}");
    assert!(report.contains("2 m (140 – 150 MHz)"), "the band it resolved to:\n{report}");
    assert!(report.contains("J4 (TX)"), "the connector transmit leaves by:\n{report}");
    assert!(report.contains("relays Transmit"), "and the key-down itself:\n{report}");
    assert!(report.contains("keyed"), "with the operator's request beside it:\n{report}");
    drop(handle);
}
