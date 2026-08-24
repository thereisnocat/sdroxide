//! TCP transport for the RSR200's LAN interface. See `RSR200_PLAN.md` §3.2/4.2:
//! commands and, in this transport, the IQ stream too, share one TCP
//! connection to [`crate::protocol::LAN_TCP_PORT`]. (UDP for the IQ stream
//! is a distinct transport — kept separate because framing is what differs
//! between them, per [`crate::device`]'s own reasoning for why [`Transport`]
//! exists at all — and is not built here.)
//!
//! TCP resynchronisation is the one genuinely transport-specific piece of
//! work here: a LAN block can begin anywhere within a stream of `read()`'d
//! bytes, so this accumulates a byte buffer and uses the protocol layer's
//! own [`find_block_start`]/[`block_trailer_valid`] — the same functions
//! `device`'s own tests already exercise — to locate a complete block
//! rather than trusting the first bytes read to be a boundary.
//!
//! Uses `std::net::TcpStream` rather than raw BSD sockets — the C++
//! reference implementation this crate is ported from reaches for raw
//! sockets because Winsock's API is close enough to BSD's that one
//! implementation covers every platform SDR++ targets without a vendor SDK
//! or an install step; `std::net` already gives Rust that same one-body,
//! every-platform property for free. Modelled on
//! `sdroxide-rtlsdr`'s own `tcp/mod.rs`, per `RSR200_PLAN.md`'s own
//! suggestion — same read-timeout-then-check-a-flag loop shape, same
//! `WouldBlock`/`TimedOut` platform split.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use crate::device::{Transport, TransportKind};
use crate::protocol::{BlockLayout, find_block_start};

/// A read that hit its timeout with nothing to show for it.
///
/// Two kinds, and both have to be caught: a socket read timeout surfaces as
/// `WouldBlock` on Unix and as `TimedOut` on Windows. Missing one turns
/// every quiet moment on that platform into a dropped connection. Also
/// catches `Interrupted` (a signal interrupting the read syscall), which
/// wants the same answer: try again, this was not a failure.
fn would_block_or_interrupted(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut | std::io::ErrorKind::Interrupted
    )
}

/// A handle that can interrupt a blocked [`LanTcpTransport::next_frame`]/
/// [`LanTcpTransport::read_packet`] from another thread, without a full
/// [`LanTcpTransport::close`] first.
///
/// Closing first (what an early version of this did, mirroring the naive
/// approach) is wrong: it leaves the caller unable to send a Stop Stream
/// command afterward, since sending needs a live socket too. That mattered
/// in the original C++ implementation's own live testing — closing first
/// meant Stop Stream never actually reached the radio, leaving it streaming
/// indefinitely and contaminating the next session's connection with data
/// left over from one that was never told to stop. `shutdown(Read)` makes
/// any pending or future read return immediately (as if the peer closed)
/// while leaving the write side open for exactly that Stop Stream command;
/// a real [`LanTcpTransport::close`] is still required afterward.
pub struct StopHandle {
    stopped: Arc<AtomicBool>,
    stream: Option<TcpStream>,
}

impl StopHandle {
    pub fn stop(&self) {
        self.stopped.store(true, Ordering::Relaxed);
        if let Some(s) = &self.stream {
            let _ = s.shutdown(std::net::Shutdown::Read);
        }
    }
}

impl Clone for StopHandle {
    /// `TcpStream` has no `Clone`, only a fallible `try_clone` (a `dup()`
    /// of the underlying file descriptor/handle) — so this clone can, in
    /// principle, end up without a stream if the OS refuses the duplicate.
    /// `stop()` still sets the flag in that case; only the immediate
    /// `shutdown(Read)` kick is lost, and the blocked read still notices
    /// `stopped` on its own next timeout.
    fn clone(&self) -> Self {
        StopHandle { stopped: Arc::clone(&self.stopped), stream: self.stream.as_ref().and_then(|s| s.try_clone().ok()) }
    }
}

pub struct LanTcpTransport {
    stream: Option<TcpStream>,
    connected: bool,
    stopped: Arc<AtomicBool>,
    recv_buf: Vec<u8>,
    layout: Option<BlockLayout>,
    last_error: String,
}

impl LanTcpTransport {
    pub fn new() -> Self {
        LanTcpTransport {
            stream: None,
            connected: false,
            stopped: Arc::new(AtomicBool::new(false)),
            recv_buf: Vec::new(),
            layout: None,
            last_error: String::new(),
        }
    }

    /// A cross-thread handle to interrupt a blocked read — see
    /// [`StopHandle`]'s own doc. `None` before [`Self::connect`] has ever
    /// succeeded, since there is nothing yet to interrupt.
    pub fn stop_handle(&self) -> StopHandle {
        StopHandle { stopped: Arc::clone(&self.stopped), stream: self.stream.as_ref().and_then(|s| s.try_clone().ok()) }
    }

    /// Connects and leaves the socket in blocking mode — `next_frame()` is
    /// meant to block, matching the contract [`crate::device::Device::pump`]
    /// already assumes. `recv_timeout` bounds how long a single read can
    /// stall, so a dead link is detectable rather than hanging `next_frame()`
    /// forever; it is not a connect timeout.
    pub fn connect(&mut self, host: &str, port: u16, recv_timeout: Duration) -> bool {
        self.close();

        let addr = format!("{host}:{port}");
        let stream = match TcpStream::connect(&addr) {
            Ok(s) => s,
            // A bare "connect() failed" gives no way to tell "nothing
            // listening" from "firewalled" from "network unreachable"
            // apart, which matters a lot when bringing this up against
            // real hardware for the first time -- io::Error's own Display
            // already carries that detail (the OS error text), so it is
            // threaded straight through rather than discarded.
            Err(e) => return self.set_error(format!("connect() to {addr} failed: {e}")),
        };

        // Nagle batches small writes, which is exactly wrong for a command
        // channel where a caller wants a short packet on the wire
        // immediately.
        if let Err(e) = stream.set_nodelay(true) {
            return self.set_error(format!("set_nodelay failed: {e}"));
        }
        if let Err(e) = stream.set_read_timeout(Some(recv_timeout)) {
            return self.set_error(format!("set_read_timeout failed: {e}"));
        }

        self.stream = Some(stream);
        self.connected = true;
        self.stopped.store(false, Ordering::Relaxed);
        self.recv_buf.clear();
        true
    }

    /// Drops the socket outright. Call after [`Self::stop`] (or a
    /// [`StopHandle::stop`]) has already stopped any in-flight read and any
    /// last command (e.g. Stop Stream) has been sent — see [`StopHandle`]'s
    /// own doc for why the ordering matters.
    pub fn close(&mut self) {
        self.stream = None; // dropping a TcpStream closes it
        self.connected = false;
    }

    /// The local equivalent of [`StopHandle::stop`] — see its doc. Prefer
    /// [`Self::stop_handle`] when the caller that needs to interrupt a
    /// blocked read is on a different thread than the one driving this
    /// transport, which is the situation this exists for.
    pub fn stop(&mut self) {
        self.stopped.store(true, Ordering::Relaxed);
        if let Some(s) = &self.stream {
            let _ = s.shutdown(std::net::Shutdown::Read);
        }
    }

    pub fn is_connected(&self) -> bool {
        self.connected
    }

    fn set_error(&mut self, msg: String) -> bool {
        self.last_error = msg;
        false
    }

    /// Shared by [`Transport::next_frame`] and [`Transport::read_packet`]:
    /// pull more bytes into `recv_buf`, treating a read timeout as "nothing
    /// yet, keep waiting" rather than a failure. `false` means stopped or
    /// the connection is gone.
    fn fill_recv_buf(&mut self) -> bool {
        let mut chunk = [0u8; 65536];
        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return false;
            }
            let Some(stream) = self.stream.as_mut() else { return false };
            match stream.read(&mut chunk) {
                Ok(0) => {
                    if !self.stopped.load(Ordering::Relaxed) {
                        self.last_error = "connection closed by radio".to_string();
                    }
                    self.connected = false;
                    return false;
                }
                Ok(n) => {
                    self.recv_buf.extend_from_slice(&chunk[..n]);
                    return true;
                }
                Err(e) if would_block_or_interrupted(&e) => continue,
                Err(e) => {
                    if !self.stopped.load(Ordering::Relaxed) {
                        self.last_error = format!("read failed: {e}");
                    }
                    self.connected = false;
                    return false;
                }
            }
        }
    }
}

impl Default for LanTcpTransport {
    fn default() -> Self {
        LanTcpTransport::new()
    }
}

impl Transport for LanTcpTransport {
    fn kind(&self) -> TransportKind {
        TransportKind::LanTcp
    }

    fn send_command(&mut self, data: &[u8]) -> bool {
        let Some(stream) = self.stream.as_mut() else { return false };
        if let Err(e) = stream.write_all(data) {
            self.last_error = format!("send failed: {e}");
            self.connected = false;
            return false;
        }
        true
    }

    /// Blocks until a full, validated block is available. False on stop or
    /// a dead connection — [`crate::device::Device::pump`] treats false as
    /// "the transport has stopped", exactly what both of those are.
    fn next_frame(&mut self, out: &mut Vec<u8>) -> bool {
        let Some(layout) = self.layout else { return false };

        loop {
            if self.stopped.load(Ordering::Relaxed) {
                return false;
            }

            if let Some(start) = find_block_start(&self.recv_buf, &layout) {
                out.clear();
                out.extend_from_slice(&self.recv_buf[start..start + layout.block_bytes]);
                self.recv_buf.drain(..start + layout.block_bytes);
                return true;
            }

            // No complete, valid block yet. Keep at most one block's worth
            // of trailing bytes -- find_block_start needs a full block
            // from a sync candidate onward, so anything older than that
            // can never complete one and would otherwise make this buffer
            // grow without bound on a link that is producing garbage.
            if self.recv_buf.len() > layout.block_bytes * 2 {
                let keep_from = self.recv_buf.len() - layout.block_bytes;
                self.recv_buf.drain(..keep_from);
            }

            if !self.fill_recv_buf() {
                return false;
            }
        }
    }

    fn set_layout(&mut self, layout: BlockLayout) {
        self.layout = Some(layout);
        // A format change mid-stream invalidates whatever partial data was
        // buffered under the old layout; resync from whatever arrives
        // next.
        self.recv_buf.clear();
    }

    /// Reads exactly `expected_bytes` — the fixed size of every LAN
    /// packet-mode reply. Shares `recv_buf` with [`Self::next_frame`]
    /// deliberately, so bytes read here and bytes read once streaming has
    /// switched over to block mode are never duplicated or dropped at the
    /// boundary between the two — whatever's left over (there should be
    /// nothing, if every packet-mode reply this call is used for is read
    /// before the next command is sent) just becomes the start of
    /// `next_frame()`'s own resync search.
    fn read_packet(&mut self, out: &mut Vec<u8>, expected_bytes: usize) -> bool {
        while self.recv_buf.len() < expected_bytes {
            if self.stopped.load(Ordering::Relaxed) {
                return false;
            }
            if !self.fill_recv_buf() {
                return false;
            }
        }
        out.clear();
        out.extend_from_slice(&self.recv_buf[..expected_bytes]);
        self.recv_buf.drain(..expected_bytes);
        true
    }

    fn last_error(&self) -> Option<&str> {
        if self.last_error.is_empty() { None } else { Some(&self.last_error) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{SYNC_BYTES, StreamFormat, lan_layout, write_u32};
    use std::net::TcpListener;
    use std::thread;

    /// A block with valid sync words and a matching counter/~counter pair
    /// — the minimum needed to pass `next_frame()`'s own resync check.
    fn make_valid_block(l: &BlockLayout, counter: u32) -> Vec<u8> {
        let mut b = vec![0u8; l.block_bytes];
        write_u32(&mut b[l.counter_offset..], counter);
        write_u32(&mut b[l.inv_counter_offset..], !counter);
        b[l.sync_offset..l.sync_offset + SYNC_BYTES.len()].copy_from_slice(&SYNC_BYTES);
        b
    }

    #[test]
    fn connect_fails_cleanly_against_a_closed_port() {
        // Port 0 asks the OS for any free port and never listens on it --
        // guaranteed nothing is there to accept a connection, without
        // depending on any specific port being free system-wide.
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener); // free the port again; nothing will be listening on it now

        let mut t = LanTcpTransport::new();
        assert!(!t.connect("127.0.0.1", port, Duration::from_millis(200)), "connecting to a closed port fails");
        assert!(t.last_error().is_some(), "and leaves an explanatory error");
        assert!(!t.is_connected());
    }

    #[test]
    fn next_frame_resyncs_mid_stream_and_delivers_a_validated_block() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let l = lan_layout(StreamFormat { channels: 1, bits: 16 });

        let server = thread::spawn(move || {
            let (mut sock, _) = listener.accept().unwrap();
            // Junk bytes before the block, as a receiver would see
            // mid-stream -- next_frame() has to find the boundary, not
            // assume the first bytes read are one.
            sock.write_all(&[0xAAu8; 37]).unwrap();
            sock.write_all(&make_valid_block(&l, 0x1234_5678)).unwrap();
        });

        let mut t = LanTcpTransport::new();
        assert!(t.connect("127.0.0.1", port, Duration::from_secs(2)), "connect: {:?}", t.last_error());
        t.set_layout(l);

        let mut out = Vec::new();
        assert!(t.next_frame(&mut out), "a validated block is found and returned");
        assert_eq!(out.len(), l.block_bytes);
        assert_eq!(&out[l.sync_offset..l.sync_offset + SYNC_BYTES.len()], &SYNC_BYTES[..]);

        server.join().unwrap();
    }

    #[test]
    fn stop_unblocks_a_pending_next_frame_without_a_full_close() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let l = lan_layout(StreamFormat { channels: 1, bits: 16 });

        // Accept and then send nothing at all -- next_frame() would
        // otherwise block on this connection indefinitely (bounded only by
        // the read timeout, which this test sets generously long
        // specifically to prove stop() is what unblocks it, not a
        // timeout). Left to finish and exit on its own in the background
        // once its sleep ends; the assertions below don't depend on it.
        let _server = thread::spawn(move || {
            let (sock, _) = listener.accept().unwrap();
            thread::sleep(Duration::from_secs(5));
            drop(sock);
        });

        let mut t = LanTcpTransport::new();
        assert!(t.connect("127.0.0.1", port, Duration::from_secs(30)), "connect: {:?}", t.last_error());
        t.set_layout(l);
        let handle = t.stop_handle();

        let joiner = thread::spawn(move || {
            let mut out = Vec::new();
            t.next_frame(&mut out)
        });

        // Give next_frame() a moment to actually enter its blocking read,
        // then interrupt it -- this is the whole property under test: stop()
        // must return next_frame() promptly, not after the 30s timeout.
        thread::sleep(Duration::from_millis(100));
        handle.stop();

        let result = joiner.join().unwrap();
        assert!(!result, "stop() unblocks next_frame(), which reports itself stopped");
    }
}
