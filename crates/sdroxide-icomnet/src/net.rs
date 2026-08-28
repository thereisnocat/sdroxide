//! One session with one Icom over its LAN port.
//!
//! # Why a single thread for three sockets
//!
//! The three streams are not independent in time: the control stream's status
//! packet is what names the CI-V and audio ports, so those two sockets cannot
//! exist until the first has got that far, and all three then have to be
//! serviced together. Running them on one non-blocking thread makes that
//! sequencing ordinary code rather than a handshake between threads, and the
//! whole session — sequence state, token renewal, teardown — stays in one
//! place with no locking.
//!
//! The thread drains all three sockets each turn and sleeps a millisecond only
//! when every one of them is empty, so a busy stream is never waiting on a
//! timer.
//!
//! # What crosses the boundary
//!
//! Audio moves through `rtrb` rings, because it is a steady sample flow and
//! must not allocate. CI-V frames move through `crossbeam_channel`, because
//! they are bursty, small, and want a queue rather than a ring.

use std::net::{Ipv4Addr, SocketAddrV4, ToSocketAddrs, UdpSocket};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TryRecvError, bounded, unbounded};

use crate::protocol::{self as proto, AudioCodec, Model, RadioCap, StreamRequest, timing};
use crate::stream::{Event, Stream};
use crate::trace::Trace;

/// Everything the operator chooses about a connection.
#[derive(Debug, Clone)]
pub struct IcomNetOptions {
    /// Hostname or dotted-quad. A name is resolved once, at connect.
    pub address: String,
    pub control_port: u16,
    pub username: String,
    pub password: String,
    /// How we appear in the radio's connection list.
    pub client_name: String,
    pub rx_sample_rate: u32,
    pub tx_sample_rate: u32,
    /// How much audio the radio should buffer before modulating, in ms.
    pub tx_buffer_ms: u32,
    /// Use this radio from the capability list rather than the first, matched
    /// on CI-V address. Only an RS-BA1 server ever offers more than one.
    pub civ_address_override: Option<u8>,
    /// How long to wait for the streams to open.
    pub timeout: Duration,
}

impl Default for IcomNetOptions {
    fn default() -> Self {
        IcomNetOptions {
            address: String::new(),
            control_port: 50_001,
            username: String::new(),
            password: String::new(),
            client_name: "sdroxide".into(),
            rx_sample_rate: 48_000,
            tx_sample_rate: 48_000,
            tx_buffer_ms: 150,
            civ_address_override: None,
            timeout: Duration::from_secs(10),
        }
    }
}

/// What the radio turned out to be, once it had introduced itself.
#[derive(Debug, Clone)]
pub struct SessionInfo {
    pub radio_name: String,
    pub civ_address: u8,
    pub model: Model,
    pub rx_sample_rate: u32,
    pub tx_sample_rate: u32,
    /// Whether this radio has a transmitter — [`RadioCap::has_transmitter`],
    /// not the raw offer the stream request is built from. An IC-R8600
    /// advertises a transmit stream and has nothing to transmit with.
    pub can_transmit: bool,
    /// The link description the radio reported at login ("FTTH", …).
    pub connection: String,
    pub baud_rate: u32,
    /// The offered-rate bit sets, verbatim. Icom does not document the
    /// encoding and the reference client does not decode it either, so this is
    /// surfaced raw for a bug report rather than interpreted.
    pub offered_rx_rates: u16,
    pub offered_tx_rates: u16,
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    Io(#[from] std::io::Error),
    #[error("cannot resolve {0}")]
    Resolve(String),
    #[error("the radio is only reachable over IPv4")]
    NotIpv4,
    #[error("no answer from the radio — check the address and that Network Control is on")]
    NoAnswer,
    #[error("the radio rejected the username or password")]
    BadCredentials,
    #[error("the radio refused the connection; it may need a power cycle")]
    Refused,
    #[error("{0} is already streaming to {1}")]
    Busy(String, String),
    #[error("the radio offered no receiver")]
    NoRadios,
    #[error("timed out after {0:?} waiting for the audio and CI-V streams")]
    Timeout(Duration),
    #[error("the radio closed the connection")]
    Closed,
    #[error(
        "the radio never answered on its data ports — it may still be holding an \
         earlier session; waiting a minute, or powering the radio off and on, clears it"
    )]
    StreamsSilent,
    #[error("the session hit a bug ({0}) — please attach the session trace to a report")]
    Internal(String),
}

/// A message for the session thread.
enum Ctrl {
    /// A CI-V frame to put on the wire.
    Civ(Vec<u8>),
    Shutdown,
}

struct Inner {
    alive: Arc<AtomicBool>,
    /// Received PCM, measured over the last second — the only way to notice
    /// that the radio granted a rate other than the one we asked for.
    measured_rate: Arc<AtomicU32>,
    ctrl: Sender<Ctrl>,
    civ_rx: Receiver<Vec<u8>>,
    audio_rx: Mutex<Option<rtrb::Consumer<f32>>>,
    audio_tx: Mutex<Option<rtrb::Producer<f32>>>,
    info: SessionInfo,
    trace: Trace,
    join: Mutex<Option<JoinHandle<()>>>,
}

/// A live session. Dropping it, or calling [`IcomNetDevice::shutdown`], tells
/// the radio we are going and waits for the thread to stop.
pub struct IcomNetDevice {
    inner: Arc<Inner>,
}

impl IcomNetDevice {
    /// Connect, log in, and open the CI-V and audio streams. Blocks until the
    /// radio is streaming or `opts.timeout` runs out.
    pub fn connect(opts: IcomNetOptions) -> Result<IcomNetDevice, Error> {
        IcomNetDevice::connect_traced(opts, Trace::new())
    }

    /// [`IcomNetDevice::connect`], recording into a trace the caller supplied
    /// and keeps. The point is the failure case: a connect that never becomes a
    /// device would otherwise take its trace down with it, and the trace of a
    /// *failed* connect is exactly the one a bug report needs.
    pub fn connect_traced(opts: IcomNetOptions, trace: Trace) -> Result<IcomNetDevice, Error> {
        let peer = resolve(&opts.address, opts.control_port)?;
        let local_ip = local_ip_towards(peer);
        trace.note(format!("connecting to {peer} from {local_ip}"));

        // 1 s of audio each way at the highest rate we ever ask for. The RX
        // ring absorbs a scheduling hiccup in the engine; the TX ring is what
        // the engine pushes into and is drained on the radio's own clock.
        let (audio_prod, audio_cons) = rtrb::RingBuffer::<f32>::new(48_000);
        let (tx_prod, tx_cons) = rtrb::RingBuffer::<f32>::new(48_000);
        let (ctrl_tx, ctrl_rx) = unbounded::<Ctrl>();
        let (civ_tx, civ_rx) = unbounded::<Vec<u8>>();
        let (ready_tx, ready_rx) = bounded::<Result<SessionInfo, Error>>(1);

        let alive = Arc::new(AtomicBool::new(true));
        let measured = Arc::new(AtomicU32::new(0));

        let session = SessionThread {
            opts: opts.clone(),
            peer,
            local_ip,
            trace: trace.clone(),
            alive: alive.clone(),
            measured: measured.clone(),
            ctrl_rx,
            civ_tx,
            audio_out: audio_prod,
            tx_in: tx_cons,
            ready: Some(ready_tx),
            connection: String::new(),
        };

        let join = std::thread::Builder::new()
            .name("sdroxide-icomnet".into())
            .spawn(move || session.run())?;

        // The thread reports the outcome of the handshake once; anything after
        // that is a runtime event, not a connect failure.
        let info = match ready_rx.recv_timeout(opts.timeout + Duration::from_secs(1)) {
            Ok(Ok(info)) => info,
            Ok(Err(e)) => {
                alive.store(false, Ordering::Relaxed);
                let _ = join.join();
                return Err(e);
            }
            Err(_) => {
                alive.store(false, Ordering::Relaxed);
                let _ = join.join();
                return Err(Error::Timeout(opts.timeout));
            }
        };

        Ok(IcomNetDevice {
            inner: Arc::new(Inner {
                alive,
                measured_rate: measured,
                ctrl: ctrl_tx,
                civ_rx,
                audio_rx: Mutex::new(Some(audio_cons)),
                audio_tx: Mutex::new(Some(tx_prod)),
                info,
                trace,
                join: Mutex::new(Some(join)),
            }),
        })
    }

    pub fn info(&self) -> &SessionInfo {
        &self.inner.info
    }

    pub fn trace(&self) -> Trace {
        self.inner.trace.clone()
    }

    /// The receive sample rate as actually measured off the wire, or 0 before
    /// the first second of audio. A figure well away from the requested rate
    /// means the radio granted something else.
    pub fn measured_rx_rate(&self) -> u32 {
        self.inner.measured_rate.load(Ordering::Relaxed)
    }

    pub fn is_alive(&self) -> bool {
        self.inner.alive.load(Ordering::Relaxed)
    }

    /// Take the receive audio ring. Available once.
    pub fn take_audio_rx(&self) -> Option<rtrb::Consumer<f32>> {
        self.inner.audio_rx.lock().unwrap_or_else(|e| e.into_inner()).take()
    }

    /// Take the transmit audio ring. `None` when the radio said it cannot
    /// accept audio, and once it has already been taken.
    pub fn take_audio_tx(&self) -> Option<rtrb::Producer<f32>> {
        if !self.inner.info.can_transmit {
            return None;
        }
        self.inner.audio_tx.lock().unwrap_or_else(|e| e.into_inner()).take()
    }

    /// Put a CI-V frame on the wire. Silently dropped once the session is gone,
    /// which is what a rig that has been unplugged mid-command looks like.
    pub fn send_civ(&self, frame: Vec<u8>) {
        let _ = self.inner.ctrl.send(Ctrl::Civ(frame));
    }

    /// Frames the radio has sent. Each is one complete CI-V frame.
    pub fn civ_frames(&self) -> &Receiver<Vec<u8>> {
        &self.inner.civ_rx
    }

    /// Close the session and wait for the thread.
    pub fn shutdown(&self) {
        if self.inner.alive.swap(false, Ordering::Relaxed) {
            let _ = self.inner.ctrl.send(Ctrl::Shutdown);
        }
        if let Some(j) = self.inner.join.lock().unwrap_or_else(|e| e.into_inner()).take() {
            let _ = j.join();
        }
    }
}

impl Drop for IcomNetDevice {
    fn drop(&mut self) {
        // Only the last handle tears the session down; the device registry
        // hands the same session to two radio tabs.
        if Arc::strong_count(&self.inner) == 1 {
            self.shutdown();
        }
    }
}

impl std::fmt::Debug for IcomNetDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("IcomNetDevice")
            .field("radio", &self.inner.info.radio_name)
            .field("civ", &format_args!("{:02X}h", self.inner.info.civ_address))
            .field("alive", &self.is_alive())
            .finish()
    }
}

impl Clone for IcomNetDevice {
    fn clone(&self) -> Self {
        IcomNetDevice { inner: self.inner.clone() }
    }
}

// ---------------------------------------------------------------------------
// The session thread
// ---------------------------------------------------------------------------

/// Where the control stream has got to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Probing, then waiting for the peer to say it is ready.
    Handshake,
    /// Login sent, waiting for the token.
    LoggingIn,
    /// Token claimed, waiting for the radio to describe itself.
    Capabilities,
    /// Stream request sent, waiting for the ports.
    Opening,
    /// CI-V and audio are up.
    Streaming,
}

struct SessionThread {
    opts: IcomNetOptions,
    peer: SocketAddrV4,
    local_ip: Ipv4Addr,
    trace: Trace,
    alive: Arc<AtomicBool>,
    measured: Arc<AtomicU32>,
    ctrl_rx: Receiver<Ctrl>,
    civ_tx: Sender<Vec<u8>>,
    audio_out: rtrb::Producer<f32>,
    tx_in: rtrb::Consumer<f32>,
    ready: Option<Sender<Result<SessionInfo, Error>>>,
    /// The link description the radio reported at login.
    connection: String,
}

/// How many CI-V frames to hold while the CI-V stream comes up.
///
/// A session opens with about a dozen — the offset clear, the reads, the menu
/// writes, the scope enable — and then polls two or three every 200 ms, so this
/// is several seconds of stall. Past it the stream is not coming up and the
/// queue is not the problem.
const CIV_PENDING_MAX: usize = 64;

/// Put one CI-V frame on the CI-V stream.
fn send_civ(c: &mut Stream, frame: &[u8], inner_seq: &mut u16, trace: &Trace) {
    trace.civ_tx(frame);
    let (a, b) = c.ids();
    let p = proto::civ_data(frame, *inner_seq, a, b);
    *inner_seq = inner_seq.wrapping_add(1);
    c.send_tracked(p);
}

impl SessionThread {
    fn run(mut self) {
        // A panic must not leave a zombie: without the catch, `alive` stays
        // true forever, the source never reports `needs_reopen`, and the
        // operator is left with a dead session that says it is connected.
        let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.drive()));
        match outcome {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                self.trace.note(format!("session ended: {e}"));
                tracing::warn!(target: "icomnet", error = %e, "session ended");
                self.report(Err(e));
            }
            Err(panic) => {
                let what = panic_text(panic.as_ref());
                self.trace.note(format!("session thread panicked: {what}"));
                tracing::error!(target: "icomnet", panic = %what, "session thread panicked");
                self.report(Err(Error::Internal(what)));
            }
        }
        self.alive.store(false, Ordering::Relaxed);
    }

    /// Hand the connect outcome back, once.
    fn report(&mut self, r: Result<SessionInfo, Error>) {
        if let Some(tx) = self.ready.take() {
            let _ = tx.send(r);
        }
    }

    fn drive(&mut self) -> Result<(), Error> {
        // Both data ports have to be named in the stream request, which goes
        // out before the radio replies with its own — so reserve them now and
        // bind for real when the reply arrives.
        let civ_port = Stream::reserve_port()?;
        let audio_port = Stream::reserve_port()?;

        let mut control =
            Stream::bind(self.local_ip, self.peer, 0, true, self.trace.clone(), "control")?;
        let mut civ: Option<Stream> = None;
        let mut audio: Option<Stream> = None;

        let mut phase = Phase::Handshake;
        let mut auth_seq: u16 = 0;
        let mut token_request: u16 = 0;
        let mut token: u32 = 0;
        let mut civ_inner: u16 = 0;
        let mut audio_inner: u16 = 0;
        let mut radios: Vec<RadioCap> = Vec::new();
        let mut chosen: Option<RadioCap> = None;
        let mut info: Option<SessionInfo> = None;

        let started = Instant::now();
        let mut next_token = Instant::now() + Duration::from_millis(timing::TOKEN_RENEWAL);
        let mut next_civ_open = Instant::now();
        let mut last_civ_data = Instant::now();
        // CI-V frames the source handed over before the radio's CI-V stream was
        // in a state to carry them — see `CIV_PENDING_MAX`.
        let mut civ_pending: Vec<Vec<u8>> = Vec::new();
        let mut civ_preamble: Vec<Vec<u8>> = Vec::new();
        let mut civ_ready = false;
        let mut civ_overflowed = false;
        let mut audio_ready = false;
        let mut rate_window = Instant::now();
        let mut rate_frames: u32 = 0;
        let mut pcm = Vec::<f32>::with_capacity(4096);
        let mut tx_bytes = Vec::<u8>::with_capacity(proto::AUDIO_CHUNK);
        let mut tx_scratch = vec![0f32; proto::AUDIO_CHUNK / 2];
        let mut buf = Vec::<u8>::with_capacity(2048);
        let codec = AudioCodec::Lpcm16Mono;

        // Every way out of the loop — orderly shutdown, an error, the radio
        // hanging up, the application going away — funnels through the one
        // `break 'session`, so `teardown` below runs on all of them. A session
        // that leaves without saying goodbye leaves the radio holding a dead
        // session, and the next connect finds a control port that answers late
        // and data ports that never answer at all (a field-reported IC-R8600
        // wedged exactly this way).
        let outcome: Result<(), Error> = 'session: loop {
            if !self.alive.load(Ordering::Relaxed) {
                self.trace.note("closing");
                break 'session Ok(());
            }
            let mut worked = false;

            // --- commands from the source -----------------------------------
            loop {
                match self.ctrl_rx.try_recv() {
                    Ok(Ctrl::Civ(frame)) => {
                        match civ.as_mut() {
                            Some(c) if civ_ready => {
                                send_civ(c, &frame, &mut civ_inner, &self.trace);
                            }
                            // The socket exists from the moment the radio names
                            // its port, but it is not yet a session: the radio
                            // discards a datagram carrying an id it never
                            // issued. Hold on to the frame instead of writing it
                            // into a conversation that has not started.
                            _ if civ_pending.len() < CIV_PENDING_MAX => {
                                civ_pending.push(frame);
                            }
                            _ => {
                                if !civ_overflowed {
                                    civ_overflowed = true;
                                    self.trace.note(
                                        "the CI-V stream has not come up; commands sent \
                                         meanwhile are being dropped",
                                    );
                                }
                            }
                        }
                        worked = true;
                    }
                    Ok(Ctrl::Shutdown) => {
                        self.trace.note("shutdown requested");
                        break 'session Ok(());
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        self.trace.note("the application dropped the session");
                        break 'session Ok(());
                    }
                }
            }

            // --- control stream ---------------------------------------------
            loop {
                match control.poll(&mut buf) {
                    Event::Idle => break,
                    Event::Ready => {
                        if phase == Phase::Handshake {
                            phase = Phase::LoggingIn;
                            self.trace.note("logging in");
                            let (a, b) = control.ids();
                            let p = proto::login(
                                &self.opts.username,
                                &self.opts.password,
                                &self.opts.client_name,
                                next_seq(&mut auth_seq),
                                {
                                    token_request = fresh_token_request(control.my_id());
                                    token_request
                                },
                                a,
                                b,
                            );
                            control.send_tracked(p);
                        }
                        worked = true;
                    }
                    Event::Disconnect => {
                        break 'session Err(Error::Closed);
                    }
                    Event::Data => {
                        worked = true;
                        let status = match self.on_control_packet(
                            &buf,
                            &mut control,
                            &mut phase,
                            &mut auth_seq,
                            &mut token_request,
                            &mut token,
                            &mut radios,
                            &mut chosen,
                            civ_port,
                            audio_port,
                            codec,
                        ) {
                            Ok(s) => s,
                            Err(e) => break 'session Err(e),
                        };
                        // A status packet naming both ports is the go-ahead to
                        // bring the data streams up. The radio repeats it
                        // whenever another client comes or goes, so this only
                        // acts the first time.
                        if let (Some(status), None) = (status, civ.as_ref()) {
                            let Some(radio) = chosen.clone() else {
                                break 'session Err(Error::NoRadios);
                            };
                            civ = Some(
                                match Stream::bind(
                                    self.local_ip,
                                    SocketAddrV4::new(*self.peer.ip(), status.civ_port),
                                    civ_port,
                                    true,
                                    self.trace.clone(),
                                    "civ",
                                ) {
                                    Ok(s) => s,
                                    Err(e) => break 'session Err(e.into()),
                                },
                            );
                            audio = Some(
                                match Stream::bind(
                                    self.local_ip,
                                    SocketAddrV4::new(*self.peer.ip(), status.audio_port),
                                    audio_port,
                                    false,
                                    self.trace.clone(),
                                    "audio",
                                ) {
                                    Ok(s) => s,
                                    Err(e) => break 'session Err(e.into()),
                                },
                            );
                            phase = Phase::Streaming;
                            let model = proto::model_for_radio(&radio.name, radio.civ_address);
                            let i = SessionInfo {
                                radio_name: radio.name.clone(),
                                civ_address: radio.civ_address,
                                model,
                                rx_sample_rate: self.opts.rx_sample_rate,
                                tx_sample_rate: self.opts.tx_sample_rate,
                                can_transmit: radio.has_transmitter(),
                                connection: self.connection.clone(),
                                baud_rate: radio.baud_rate,
                                offered_rx_rates: radio.rx_sample,
                                offered_tx_rates: radio.tx_sample,
                            };
                            self.trace.note(format!(
                                "streaming: {} (CI-V {:02X}h, driven as {}), \
                                 civ port {}, audio port {}",
                                i.radio_name,
                                i.civ_address,
                                model.name,
                                status.civ_port,
                                status.audio_port
                            ));
                            info = Some(i.clone());
                            self.report(Ok(i));
                        }
                    }
                    _ => worked = true,
                }
            }

            // --- CI-V stream --------------------------------------------------
            if let Some(c) = civ.as_mut() {
                loop {
                    match c.poll(&mut buf) {
                        Event::Idle => break,
                        Event::Ready => {
                            // The radio stays silent until asked to open.
                            let (a, b) = c.ids();
                            let p = proto::open_close(true, next_seq(&mut civ_inner), a, b);
                            c.send_tracked(p);
                            next_civ_open = Instant::now() + Duration::from_millis(100);
                            // Everything the source asked for while the stream
                            // was coming up goes out now, in the order it was
                            // asked for. These are the one-shot frames — the
                            // menu writes, the offset clear, the scope enable —
                            // and nothing re-sends them, so dropping them on the
                            // floor cost the session its scope for good.
                            if !civ_ready {
                                civ_ready = true;
                                civ_preamble = std::mem::take(&mut civ_pending);
                                for f in &civ_preamble {
                                    send_civ(c, f, &mut civ_inner, &self.trace);
                                }
                            }
                            worked = true;
                        }
                        Event::Data => {
                            worked = true;
                            if let Some(frame) = proto::parse_civ_data(&buf) {
                                last_civ_data = Instant::now();
                                // The radio is answering, so the open request
                                // and everything after it landed.
                                civ_preamble.clear();
                                self.trace.civ_rx(frame);
                                if self.civ_tx.send(frame.to_vec()).is_err() {
                                    self.trace.note("the application dropped the session");
                                    break 'session Ok(());
                                }
                            }
                        }
                        Event::Disconnect => break 'session Err(Error::Closed),
                        _ => worked = true,
                    }
                }
                // The open request is the one packet the radio can lose without
                // saying so — nothing arrives and nothing complains. Repeat it
                // until CI-V data appears.
                let now = Instant::now();
                if now >= next_civ_open
                    && now.duration_since(last_civ_data) > Duration::from_secs(2)
                {
                    next_civ_open = now + Duration::from_millis(500);
                    let (a, b) = c.ids();
                    let p = proto::open_close(true, next_seq(&mut civ_inner), a, b);
                    c.send_tracked(p);
                    // The open request is not the only thing that can have been
                    // lost. Nothing has answered yet, so the session's opening
                    // commands are still unaccounted for — send them again. All
                    // of them are idempotent, which is what makes this safe.
                    for f in &civ_preamble {
                        send_civ(c, f, &mut civ_inner, &self.trace);
                    }
                }
                c.tick();
                // The control stream proving healthy says nothing about this
                // one: a radio still holding an earlier session answers control
                // and leaves its data ports dead, and only the CI-V stream can
                // notice. Probes unanswered → wedged; answered once but silent
                // since (its ping replies count as traffic) → the link died.
                // Either way, fail with a reason rather than sit half-open.
                if !c.handshaken() && c.quiet_for() > Duration::from_millis(timing::HANDSHAKE_MS) {
                    self.trace.note("the radio named its CI-V port but never answered on it");
                    break 'session Err(Error::StreamsSilent);
                }
                if c.handshaken() && c.is_stale() {
                    self.trace.note("the radio's CI-V stream went silent");
                    break 'session Err(Error::StreamsSilent);
                }
            }

            // --- audio stream -------------------------------------------------
            if let Some(a) = audio.as_mut() {
                loop {
                    match a.poll(&mut buf) {
                        Event::Idle => break,
                        // Either answer means the radio has issued a session
                        // id and will accept what we send. `IAmHere` is enough
                        // on its own, and taking it rather than only `Ready`
                        // keeps transmit working on a radio that skips one.
                        Event::IAmHere | Event::Ready => {
                            audio_ready = true;
                            worked = true;
                        }
                        Event::Data => {
                            worked = true;
                            audio_ready = true;
                            if let Some(payload) = proto::parse_audio_data(&buf) {
                                self.trace.audio_packet(payload.len());
                                pcm.clear();
                                proto::decode_pcm(codec, payload, &mut pcm);
                                rate_frames += pcm.len() as u32;
                                for &s in &pcm {
                                    // A full ring means the consumer stalled;
                                    // dropping the newest keeps the session
                                    // alive and the loss audible rather than
                                    // stalling the whole thread.
                                    if self.audio_out.push(s).is_err() {
                                        break;
                                    }
                                }
                            }
                        }
                        Event::Disconnect => break 'session Err(Error::Closed),
                        _ => worked = true,
                    }
                }

                // Transmit whatever the engine has queued — but not before the
                // stream has handshaken. Audio written into a session the radio
                // has not issued an id for is discarded by it, and unlike a CI-V
                // command there is nothing sensible to replay later: the ring
                // holds it instead, which is what a transmit ring is for.
                let want = if audio_ready { self.tx_in.slots().min(tx_scratch.len()) } else { 0 };
                if want > 0 {
                    for slot in tx_scratch.iter_mut().take(want) {
                        *slot = self.tx_in.pop().unwrap_or(0.0);
                    }
                    tx_bytes.clear();
                    proto::encode_pcm(codec, &tx_scratch[..want], &mut tx_bytes);
                    for chunk in tx_bytes.chunks(proto::AUDIO_CHUNK) {
                        let (x, y) = a.ids();
                        let p = proto::audio_data(chunk, next_seq(&mut audio_inner), x, y);
                        a.send_tracked(p);
                    }
                    worked = true;
                }
                a.tick();
            }

            control.tick();

            // Measure what the radio is actually sending us.
            if rate_window.elapsed() >= Duration::from_secs(1) {
                self.measured.store(rate_frames, Ordering::Relaxed);
                rate_frames = 0;
                rate_window = Instant::now();
            }

            // Token renewal keeps the session alive; without it the radio drops
            // us after a minute.
            if phase == Phase::Streaming && Instant::now() >= next_token {
                next_token = Instant::now() + Duration::from_millis(timing::TOKEN_RENEWAL);
                let (a, b) = control.ids();
                let p = proto::token(0x05, next_seq(&mut auth_seq), token_request, token, a, b);
                control.send_tracked(p);
            }

            if info.is_none() && started.elapsed() > self.opts.timeout {
                break 'session Err(Error::Timeout(self.opts.timeout));
            }
            if control.is_stale() {
                break 'session Err(Error::NoAnswer);
            }

            if !worked {
                std::thread::sleep(Duration::from_millis(1));
            }
        };
        teardown(
            &mut control,
            &mut civ,
            &mut audio,
            civ_inner,
            token_request,
            token,
            &mut auth_seq,
            &self.trace,
        );
        outcome
    }

    /// Handle one application packet on the control stream. Returns the status
    /// packet that names the data ports, when that is what arrived.
    #[allow(clippy::too_many_arguments)]
    fn on_control_packet(
        &mut self,
        pkt: &[u8],
        control: &mut Stream,
        phase: &mut Phase,
        auth_seq: &mut u16,
        token_request: &mut u16,
        token: &mut u32,
        radios: &mut Vec<RadioCap>,
        chosen: &mut Option<RadioCap>,
        civ_port: u16,
        audio_port: u16,
        codec: AudioCodec,
    ) -> Result<Option<proto::Status>, Error> {
        match pkt.len() {
            proto::LOGIN_RESPONSE_SIZE => {
                let Some(r) = proto::LoginResponse::parse(pkt) else { return Ok(None) };
                if r.error == proto::LoginResponse::BAD_CREDENTIALS {
                    return Err(Error::BadCredentials);
                }
                if *phase == Phase::LoggingIn && r.token_request == *token_request {
                    *token = r.token;
                    *phase = Phase::Capabilities;
                    self.trace.note(format!("login accepted, link reported as {:?}", r.connection));
                    self.connection = r.connection.clone();
                    let (a, b) = control.ids();
                    let p = proto::token(0x02, next_seq(auth_seq), *token_request, *token, a, b);
                    control.send_tracked(p);
                }
                Ok(None)
            }
            proto::TOKEN_SIZE => {
                let Some(t) = proto::TokenReply::parse(pkt) else { return Ok(None) };
                if t.response == proto::TokenReply::REJECTED {
                    // A renewal the radio would not honour: the token we hold
                    // is stale, and it has just handed us a fresh one.
                    self.trace.note("token renewal refused, re-requesting the stream");
                    *token_request = t.token_request;
                    *token = t.token;
                    let (a, b) = control.ids();
                    if let Some(radio) = chosen.clone() {
                        let p = self.build_stream_request(
                            &radio,
                            auth_seq,
                            *token_request,
                            *token,
                            civ_port,
                            audio_port,
                            codec,
                            a,
                            b,
                        );
                        control.send_tracked(p);
                    }
                }
                Ok(None)
            }
            proto::STATUS_SIZE => {
                let Some(s) = proto::Status::parse(pkt) else { return Ok(None) };
                if s.error == proto::Status::FAILED {
                    return Err(Error::Refused);
                }
                if s.disconnected {
                    self.trace.note("the radio dropped the stream");
                    return Ok(None);
                }
                if s.civ_port != 0 && s.audio_port != 0 {
                    return Ok(Some(s));
                }
                Ok(None)
            }
            proto::CONNINFO_SIZE => {
                let Some(c) = proto::ConnInfo::parse(pkt) else { return Ok(None) };
                if *phase == Phase::Capabilities && c.busy && !c.computer.is_empty() {
                    // Somebody else has it. Say who, rather than timing out.
                    return Err(Error::Busy(c.name, c.computer));
                }
                Ok(None)
            }
            _ => {
                // Only a capabilities packet is a header plus whole radios.
                let Some(caps) = proto::parse_capabilities(pkt) else { return Ok(None) };
                if caps.is_empty() {
                    return Err(Error::NoRadios);
                }
                if !radios.is_empty() {
                    return Ok(None);
                }
                for r in &caps {
                    self.trace.note(format!(
                        "radio: {} audio={:?} CI-V={:02X}h rx_rates={:#06x} tx_rates={:#06x}",
                        r.name, r.audio, r.civ_address, r.rx_sample, r.tx_sample
                    ));
                }
                *radios = caps;
                let pick = match self.opts.civ_address_override {
                    Some(a) => radios.iter().find(|r| r.civ_address == a).cloned(),
                    None => radios.first().cloned(),
                };
                let Some(radio) = pick else { return Err(Error::NoRadios) };
                *chosen = Some(radio.clone());
                *phase = Phase::Opening;
                let (a, b) = control.ids();
                let p = self.build_stream_request(
                    &radio,
                    auth_seq,
                    *token_request,
                    *token,
                    civ_port,
                    audio_port,
                    codec,
                    a,
                    b,
                );
                control.send_tracked(p);
                Ok(None)
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn build_stream_request(
        &self,
        radio: &RadioCap,
        auth_seq: &mut u16,
        token_request: u16,
        token: u32,
        civ_port: u16,
        audio_port: u16,
        codec: AudioCodec,
        sent_id: u32,
        rcvd_id: u32,
    ) -> Vec<u8> {
        proto::stream_request(&StreamRequest {
            radio,
            username: &self.opts.username,
            rx_codec: codec,
            tx_codec: radio.can_transmit().then_some(codec),
            rx_sample_rate: self.opts.rx_sample_rate,
            tx_sample_rate: self.opts.tx_sample_rate,
            civ_local_port: civ_port,
            audio_local_port: audio_port,
            tx_buffer_ms: self.opts.tx_buffer_ms,
            inner_seq: next_seq(auth_seq),
            token_request,
            token,
            sent_id,
            rcvd_id,
        })
    }
}

/// Say goodbye on all three streams, giving the token back so the radio does
/// not hold the session open for the next client.
///
/// This runs on *every* way out of a session, errors included. Skipping it on
/// the failure paths is what used to wedge the radio: it kept the dead
/// session's token and streams, and the next connect found a control port that
/// answered seconds late and data ports that never answered at all.
#[allow(clippy::too_many_arguments)]
fn teardown(
    control: &mut Stream,
    civ: &mut Option<Stream>,
    audio: &mut Option<Stream>,
    civ_inner: u16,
    token_request: u16,
    token: u32,
    auth_seq: &mut u16,
    trace: &Trace,
) {
    trace.note(if token != 0 {
        "goodbye: token returned, streams disconnected"
    } else {
        "goodbye: streams disconnected"
    });
    if let Some(c) = civ.as_mut() {
        let (a, b) = c.ids();
        let p = proto::open_close(false, civ_inner, a, b);
        c.send_tracked(p);
        c.disconnect();
    }
    if let Some(a) = audio.as_mut() {
        a.disconnect();
    }
    if token != 0 {
        let (a, b) = control.ids();
        let p = proto::token(0x01, next_seq(auth_seq), token_request, token, a, b);
        control.send_tracked(p);
    }
    control.disconnect();
}

fn next_seq(seq: &mut u16) -> u16 {
    let v = *seq;
    *seq = seq.wrapping_add(1);
    v
}

/// The message inside a panic payload, for the trace. A payload is almost
/// always one of the two string forms; anything else gets a placeholder.
fn panic_text(p: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = p.downcast_ref::<&str>() {
        (*s).into()
    } else if let Some(s) = p.downcast_ref::<String>() {
        s.clone()
    } else {
        "unknown panic".into()
    }
}

/// Icom's own client picks this at random; the radio only checks that our
/// login and our token requests agree on it. Deriving it from the session id
/// keeps it unpredictable enough for that purpose without pulling in an RNG.
fn fresh_token_request(my_id: u32) -> u16 {
    let t = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(0);
    ((t ^ my_id) & 0xffff) as u16 | 1
}

/// Resolve a host or dotted quad to an IPv4 socket address.
pub(crate) fn resolve(address: &str, port: u16) -> Result<SocketAddrV4, Error> {
    let host = address.trim();
    if host.is_empty() {
        return Err(Error::Resolve(address.to_string()));
    }
    let mut any = false;
    for addr in (host, port).to_socket_addrs().map_err(|_| Error::Resolve(host.to_string()))? {
        any = true;
        if let std::net::SocketAddr::V4(v4) = addr {
            return Ok(v4);
        }
    }
    Err(if any { Error::NotIpv4 } else { Error::Resolve(host.to_string()) })
}

/// Which of our addresses the radio will see us coming from.
///
/// A connected UDP socket costs nothing and makes the kernel do the routing
/// decision for us, which is the only way to get this right on a host with a
/// VPN, a second NIC, or both. Falling back to the loopback address is safe:
/// the value only seeds a session identifier.
pub(crate) fn local_ip_towards(peer: SocketAddrV4) -> Ipv4Addr {
    UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, 0))
        .and_then(|s| {
            s.connect(peer)?;
            s.local_addr()
        })
        .ok()
        .and_then(|a| match a {
            std::net::SocketAddr::V4(v4) => Some(*v4.ip()),
            _ => None,
        })
        .unwrap_or(Ipv4Addr::LOCALHOST)
}

/// Connect, report what the radio said, and disconnect — the settings dialog's
/// "Test connection" button.
pub fn test_connection(opts: IcomNetOptions) -> Result<String, String> {
    let dev = IcomNetDevice::connect(opts).map_err(|e| e.to_string())?;
    let i = dev.info();
    let mut out = format!(
        "{} — CI-V {:02X}h, driven as {}\naudio {} Hz{}",
        i.radio_name,
        i.civ_address,
        i.model.name,
        i.rx_sample_rate,
        if i.can_transmit { ", transmit available" } else { ", receive only" },
    );
    if i.model.lan_mod_input.is_none() {
        out.push_str(
            "\n\nThis model is not in the menu table, so sdroxide cannot switch \
             the modulation input to LAN for you — set it on the radio.",
        );
    }
    dev.shutdown();
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dotted_quad_resolves_without_dns() {
        let a = resolve("192.0.2.7", 50_001).unwrap();
        assert_eq!(a, SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 7), 50_001));
    }

    #[test]
    fn an_empty_address_is_an_error_rather_than_a_wildcard_bind() {
        assert!(matches!(resolve("   ", 50_001), Err(Error::Resolve(_))));
    }

    #[test]
    fn the_local_address_for_a_loopback_peer_is_loopback() {
        let ip = local_ip_towards(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 50_001));
        assert_eq!(ip, Ipv4Addr::LOCALHOST);
    }

    #[test]
    fn a_token_request_is_never_zero() {
        // Zero is the "no request outstanding" marker in the reply matcher.
        for id in [0u32, 1, 0xffff_ffff, 0x1234_5678] {
            assert_ne!(fresh_token_request(id), 0);
        }
    }

    #[test]
    fn sequences_wrap_rather_than_panic() {
        let mut s = u16::MAX;
        assert_eq!(next_seq(&mut s), u16::MAX);
        assert_eq!(s, 0);
    }
}
