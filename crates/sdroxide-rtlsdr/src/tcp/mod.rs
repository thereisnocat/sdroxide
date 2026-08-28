//! The `rtl_tcp` client: an RTL-SDR on another machine, reached over TCP.
//!
//! Same shape as the USB backend next door — one blocking thread owns the
//! connection, control arrives over a crossbeam channel and is coalesced
//! ([`Pending`]), samples leave through an `rtrb` ring — and it hands back the
//! very same [`RtlSdrHandle`], because nothing in that handle was ever about
//! USB. What changes is only where the register writes happen: here they are
//! five-byte commands the server performs on our behalf.
//!
//! Two consequences run through everything below.
//!
//! *Nothing is ever answered.* The protocol has no replies, so this end cannot
//! read back an achieved sample rate, a snapped gain or an error; what it
//! reports is what it asked for. Every place that matters says so.
//!
//! *The socket must be drained whether or not anyone wants the samples.*
//! `rtl_tcp` gives up on a client that stops reading and closes the
//! connection, so a full RX ring drops blocks (as it does over USB) but never
//! stops draining the socket. Backing up into the kernel's receive queue would
//! also add latency that never comes back.

pub mod proto;

use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use rtrb::Producer;
use sdroxide_types::{RtlSdrAgc, RtlSdrHfMode, RtlTcpConfig};

use crate::error::{Error, Result};
use crate::handle::{Ctrl, Pending, RtlSdrHandle, RxStats, Shared, push_iq, ring_for};
use crate::stream::sample_lut;
use proto::{
    Cmd, GREETING_LEN, Greeting, RSP_CAPABILITIES_LEN, RspCapabilities, RspSampleFormat, Tuner,
};

/// How long to wait for the TCP connection itself. A server that is running
/// answers a LAN connect in milliseconds; this length is for the case where
/// the host is up but nothing is listening on a filtered port, which is a
/// timeout rather than a refusal.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// How long the greeting may take once connected. Generous: `rtl_tcp` sends it
/// only after it has opened and configured the dongle, which on a Raspberry Pi
/// with a cold USB stack is not instant.
const GREETING_TIMEOUT: Duration = Duration::from_secs(5);

/// How long to wait for the optional `rsp_tcp` capability block after the
/// greeting.
///
/// This is a *peek*, and it has to be one: the block is only sent by an SDRplay
/// server started with `-E`, and an ordinary `rtl_tcp` server sends samples
/// there instead. Four bytes are read either way and kept if they turn out not
/// to be the magic, so nothing is lost — but a server that has not started
/// streaming yet would block here forever without a bound. Half a second is far
/// longer than either case takes on a LAN and is paid once, at connect.
const RSP_PEEK_TIMEOUT: Duration = Duration::from_millis(500);

/// How long a read may block before the loop goes back to serve control.
/// Bounds how long a dial drag waits when the stream has gone quiet; while
/// samples are flowing, reads return as soon as anything has arrived and this
/// never comes into play.
const READ_TIMEOUT: Duration = Duration::from_millis(20);

/// Socket read size. Only a ceiling — a TCP read returns whatever has arrived
/// — so this costs nothing at low sample rates and saves syscalls at high
/// ones. 64 KiB is 13.6 ms at 2.4 Msps.
const READ_BYTES: usize = 64 * 1024;

/// Hysteresis above [`crate::TUNER_MIN_HZ`], so a dial parked next to the
/// crossover cannot flip the far end between direct sampling and its tuner on
/// every nudge. Wider than the USB backend's 500 kHz: each switch costs a tuner
/// re-init *and* a network round trip, and the samples in flight when it
/// happens are already stale. Above the floor and never straddling it, for the
/// reason the USB backend's copy gives.
const HF_HYSTERESIS_HZ: f64 = 1_000_000.0;

/// [`Client::direct`] before anything has been asked of the far end. Whatever
/// a previous client left the server in is unknowable, so the opening sequence
/// states the direct-sampling mode either way rather than assuming it is off.
const DIRECT_UNKNOWN: u32 = u32::MAX;

/// Connect, configure, and start the stream thread.
///
/// Blocks until the connection is up and the far end has been configured, or
/// has failed — so a wrong address or a server that is not running comes back
/// as an ordinary error rather than as a stream that never starts.
pub(crate) fn spawn(cfg: &RtlTcpConfig, center_hz: f64) -> Result<RtlSdrHandle> {
    let (ctrl_tx, ctrl_rx) = crossbeam_channel::unbounded::<Ctrl>();
    let (ready_tx, ready_rx) =
        crossbeam_channel::bounded::<Result<(Greeting, Option<RspCapabilities>)>>(1);

    let shared = Arc::new(Shared {
        alive: AtomicBool::new(true),
        last_rx_ms: AtomicU64::new(0),
        rate_milli_hz: AtomicU64::new(0),
        effective_gain_tenth_db: AtomicI64::new(-1),
        rx_paused: AtomicBool::new(false),
    });

    let (rx_prod, rx_cons) = ring_for(cfg.sample_rate_hz);

    // Read out before the config moves onto the thread: the handle is built
    // here, from what was asked for, because the far end reports none of it.
    let endpoint = cfg.endpoint();
    let rate_hz = cfg.sample_rate_hz;
    let hf_mode = cfg.hf_mode;

    let cfg = cfg.clone();
    let thread_shared = Arc::clone(&shared);
    let join = std::thread::Builder::new()
        .name("sdroxide-rtltcp".into())
        .spawn(move || {
            run(cfg, center_hz, ctrl_rx, rx_prod, Arc::clone(&thread_shared), ready_tx);
            thread_shared.alive.store(false, Ordering::Relaxed);
        })
        .map_err(|e| Error::Access(format!("could not start the rtl_tcp thread: {e}")))?;

    match ready_rx.recv() {
        Ok(Ok((greeting, rsp))) => Ok(RtlSdrHandle::from_parts(
            rx_cons,
            ctrl_tx,
            shared,
            join,
            // An SDRplay server in extended mode is the one case where the far
            // end says something real about itself, so the label says it back
            // rather than repeating the R820T it is pretending to be.
            match &rsp {
                Some(c) => format!("rsp_tcp {endpoint} ({})", c.describe()),
                None => format!("rtl_tcp {endpoint} ({})", greeting.describe()),
            },
            greeting.tuner.name().to_string(),
            // "Upconverts HF in hardware" is what this flag decides, and for a
            // remote dongle the tuner is the only evidence there is; see
            // [`Tuner::upconverts_hf`].
            greeting.tuner.upconverts_hf(),
            hf_capable(greeting.tuner, hf_mode),
            // The nominal rate: the server never reports what its resampler
            // actually achieved. The difference is the same handful of ppm it
            // is over USB, and it lands in the measured clock error.
            rate_hz,
            // The gain *values* the far end can produce are not carried by the
            // protocol — only how many there are. Nothing here can list them.
            Vec::new(),
        )),
        Ok(Err(e)) => {
            let _ = join.join();
            Err(e)
        }
        Err(_) => {
            let _ = join.join();
            Err(Error::Net("the rtl_tcp thread stopped before it connected".into()))
        }
    }
}

fn run(
    cfg: RtlTcpConfig,
    center_hz: f64,
    ctrl: crossbeam_channel::Receiver<Ctrl>,
    mut rx: Producer<f32>,
    shared: Arc<Shared>,
    ready: crossbeam_channel::Sender<Result<(Greeting, Option<RspCapabilities>)>>,
) {
    let mut client = match Client::connect(&cfg, center_hz) {
        Ok(c) => c,
        Err(e) => {
            let _ = ready.send(Err(e));
            return;
        }
    };

    shared.rate_milli_hz.store((cfg.sample_rate_hz * 1000.0) as u64, Ordering::Relaxed);
    publish_gain(&shared, &client);
    let _ = ready.send(Ok((client.greeting, client.rsp.clone())));

    if let Err(e) = pump(&mut client, &ctrl, &mut rx, &shared, &cfg) {
        tracing::warn!("rtl_tcp stream stopped: {e}");
    }
    client.shutdown();
}

/// The far end, and everything this end believes about it.
struct Client {
    sock: TcpStream,
    endpoint: String,
    greeting: Greeting,
    center: f64,
    hf_mode: RtlSdrHfMode,
    /// Which direct-sampling branch the far end was last told to use. Tracked
    /// rather than read back, because it cannot be read back.
    direct: u32,
    agc: RtlSdrAgc,
    gain_db: f64,
    bias_tee: bool,
    /// What an `rsp_tcp` server in extended mode said about itself, if this is
    /// one. `None` for an ordinary `rtl_tcp` server *and* for an `rsp_tcp`
    /// server started without `-E` — the two are indistinguishable on the wire,
    /// which is exactly why the RSP-specific controls stay hidden without it.
    rsp: Option<RspCapabilities>,
    /// Bytes read while peeking for the capability block that turned out to be
    /// samples. Drained by the pump before it touches the socket again.
    prefix: Vec<u8>,
}

impl Client {
    /// Open the connection, read the greeting, and apply the whole
    /// configuration once.
    fn connect(cfg: &RtlTcpConfig, center_hz: f64) -> Result<Client> {
        let endpoint = cfg.endpoint();
        let sock = dial(&endpoint)?;
        // Commands are five bytes and are the only thing this end sends;
        // letting Nagle sit on them would add a round trip to every retune.
        let _ = sock.set_nodelay(true);

        sock.set_read_timeout(Some(GREETING_TIMEOUT))
            .map_err(|e| Error::Net(format!("cannot set a timeout on {endpoint}: {e}")))?;
        let mut buf = [0u8; GREETING_LEN];
        (&sock).read_exact(&mut buf).map_err(|e| {
            Error::Net(format!(
                "{endpoint} accepted the connection but sent no rtl_tcp greeting: {e}"
            ))
        })?;
        let greeting = Greeting::parse(&buf)?;

        // An `rsp_tcp` server in extended mode follows the greeting with a
        // 45-byte block; everything else sends samples. Both greet with "RTL0",
        // so the only way to tell is to look.
        let (rsp, prefix) = peek_rsp(&sock, &endpoint)?;

        sock.set_read_timeout(Some(READ_TIMEOUT))
            .map_err(|e| Error::Net(format!("cannot set a timeout on {endpoint}: {e}")))?;

        match &rsp {
            Some(c) => tracing::info!(
                "rsp_tcp {endpoint}: {} — requesting {:.3} Msps ({:.1} Mbit/s on the link)",
                c.describe(),
                cfg.sample_rate_hz / 1e6,
                RtlTcpConfig::link_mbit(cfg.sample_rate_hz),
            ),
            None => tracing::info!(
                "rtl_tcp {endpoint}: {} — requesting {:.3} Msps ({:.1} Mbit/s on the link)",
                greeting.describe(),
                cfg.sample_rate_hz / 1e6,
                RtlTcpConfig::link_mbit(cfg.sample_rate_hz),
            ),
        }

        let mut client = Client {
            sock,
            endpoint,
            greeting,
            center: center_hz,
            hf_mode: cfg.hf_mode,
            direct: DIRECT_UNKNOWN,
            agc: cfg.agc,
            gain_db: cfg.tuner_gain_db,
            bias_tee: cfg.bias_tee,
            rsp,
            prefix,
        };

        // Opening order, for the same reasons as the USB backend's `apply`:
        // ppm first because every frequency is computed against the corrected
        // crystal, then the rate, then the HF path (which may re-init the far
        // end's tuner), then the centre, then the gains that a re-init would
        // have reset, then the bias tee, which depends on nothing.
        client.send(Cmd::Ppm, cfg.ppm as u32)?;
        client.send(Cmd::SampleRate, cfg.sample_rate_hz as u32)?;
        client.apply_hf_path(center_hz)?;
        client.send(Cmd::Freq, center_hz.max(0.0) as u32)?;
        client.apply_gains()?;
        client.send(Cmd::BiasTee, u32::from(cfg.bias_tee))?;
        Ok(client)
    }

    fn send(&mut self, cmd: Cmd, arg: u32) -> Result<()> {
        self.sock.write_all(&proto::frame(cmd, arg)).map_err(|e| {
            Error::Net(format!("lost the connection to {} while sending: {e}", self.endpoint))
        })
    }

    /// Send a raw five-byte command.
    ///
    /// Exists for the `rsp_tcp` opcodes, which share this channel and its
    /// framing but live in their own numbering above the rtl_tcp ones. Kept
    /// separate from [`Self::send`] so the two spaces cannot be confused at a
    /// call site — an antenna change sent as a frequency would retune the
    /// radio to 2 Hz and look like a dead receiver.
    fn send_raw(&mut self, cmd: proto::RspCmd, arg: u32) -> Result<()> {
        self.sock.write_all(&proto::frame_rsp(cmd, arg)).map_err(|e| {
            Error::Net(format!("lost the connection to {} while sending: {e}", self.endpoint))
        })
    }

    /// Tuner gain mode, tuner gain and the demodulator's digital AGC.
    ///
    /// Sent as a group because they have to be: the gain mode must precede the
    /// gain (a server running its tuner AGC discards the value), and both have
    /// to be restated after anything that re-initialises the far end's tuner.
    fn apply_gains(&mut self) -> Result<()> {
        self.send(Cmd::GainMode, u32::from(!self.agc.tuner_auto()))?;
        if !self.agc.tuner_auto() {
            self.send(Cmd::Gain, (self.gain_db * 10.0).round().max(0.0) as u32)?;
        }
        self.send(Cmd::RtlAgc, u32::from(self.agc.rtl_auto()))
    }

    /// Decide whether this frequency needs the far end's direct-sampling path,
    /// and say so if the answer has changed.
    ///
    /// The policy is the USB backend's, with one difference forced by what the
    /// protocol does not carry: a dongle whose tuner suggests it upconverts
    /// (an R828D — see [`Tuner::upconverts_hf`]) is left alone in `Auto`,
    /// because a Blog V4 handles HF inside the server's own tuning call and
    /// switching it to direct sampling would take away the band it can hear.
    /// `DirectQ` is still obeyed for it, which is the escape hatch for a plain
    /// R828D that does not upconvert.
    fn apply_hf_path(&mut self, hz: f64) -> Result<()> {
        let want = match self.hf_mode {
            RtlSdrHfMode::Off => 0,
            RtlSdrHfMode::DirectQ => 2,
            RtlSdrHfMode::Auto if self.greeting.tuner.upconverts_hf() => 0,
            RtlSdrHfMode::Auto => {
                let threshold = if self.direct == 0 {
                    crate::TUNER_MIN_HZ
                } else {
                    crate::TUNER_MIN_HZ + HF_HYSTERESIS_HZ
                };
                u32::from(hz < threshold) * 2
            }
        };
        if want == self.direct {
            return Ok(());
        }
        let switching = self.direct != DIRECT_UNKNOWN;
        if switching {
            tracing::info!(
                "rtl_tcp {}: switching the far end to {}",
                self.endpoint,
                if want == 0 { "its tuner" } else { "direct sampling (Q branch)" }
            );
        }
        self.direct = want;
        self.send(Cmd::DirectSampling, want)?;
        // A server that has just re-initialised its tuner is back at that
        // tuner's defaults, so the gains have to be restated. Free here —
        // three more five-byte writes — and the symptom otherwise is a band
        // that is quietly 20 dB down after crossing 28.8 MHz. Not on the
        // opening call, where the sequence in `connect` states them anyway.
        if switching { self.apply_gains() } else { Ok(()) }
    }

    fn set_center(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        self.apply_hf_path(hz)?;
        // While the far end is direct sampling, its tuning call moves the
        // demodulator's DDC instead of a tuner — the same command either way.
        self.send(Cmd::Freq, hz.max(0.0) as u32)
    }

    /// Best knowledge of the gain in effect, in dB — which is the requested
    /// value, snapped to nothing, because the protocol reports nothing back.
    /// `None` while the far end's tuner AGC owns it.
    fn effective_gain_db(&self) -> Option<f64> {
        (!self.agc.tuner_auto()).then_some(self.gain_db)
    }

    /// Leave the far end safe. A bias tee outlives the connection — `rtl_tcp`
    /// keeps the dongle open for the next client — so an operator who
    /// disconnects with it on would otherwise leave DC on a mast-head coax
    /// with nothing on screen to say so.
    fn shutdown(&mut self) {
        if self.bias_tee {
            if let Err(e) = self.send(Cmd::BiasTee, 0) {
                tracing::warn!("could not turn the remote bias tee off: {e}");
            }
        }
    }
}

/// Resolve and connect, with a timeout on each candidate address.
fn dial(endpoint: &str) -> Result<TcpStream> {
    let addrs: Vec<_> = endpoint
        .to_socket_addrs()
        .map_err(|e| {
            Error::Net(format!(
                "cannot resolve {endpoint}: {e} — the address is host:port, \
                 and the port defaults to {}",
                RtlTcpConfig::DEFAULT_PORT
            ))
        })?
        .collect();
    let mut last = None;
    for addr in &addrs {
        match TcpStream::connect_timeout(addr, CONNECT_TIMEOUT) {
            Ok(s) => return Ok(s),
            Err(e) => last = Some(e),
        }
    }
    Err(Error::Net(match last {
        Some(e) => format!(
            "cannot reach the rtl_tcp server at {endpoint}: {e} — check that \
             rtl_tcp is running there (rtl_tcp -a 0.0.0.0) and that nothing is \
             filtering the port"
        ),
        None => format!("{endpoint} resolved to no addresses"),
    }))
}

fn publish_gain(shared: &Arc<Shared>, client: &Client) {
    let v = client.effective_gain_db().map(|db| (db * 10.0).round() as i64).unwrap_or(-1);
    shared.effective_gain_tenth_db.store(v, Ordering::Relaxed);
}

/// Apply coalesced control changes. Order matches the USB backend's `apply`.
fn apply(client: &mut Client, p: &Pending, carry: &mut Vec<u8>) -> Result<()> {
    if let Some(ppm) = p.ppm {
        client.send(Cmd::Ppm, ppm as u32)?;
    }
    if let Some(mode) = p.hf_mode {
        client.hf_mode = mode;
        client.apply_hf_path(client.center)?;
        // The far end restarts its stream on a path change, so a byte held
        // back from before it is no longer half of anything.
        carry.clear();
    }
    if let Some(hz) = p.center {
        client.set_center(hz)?;
    }
    if p.agc.is_some() || p.gain.is_some() {
        if let Some(agc) = p.agc {
            client.agc = agc;
        }
        if let Some(db) = p.gain {
            client.gain_db = db;
        }
        client.apply_gains()?;
    }
    if let Some(on) = p.bias_tee {
        client.bias_tee = on;
        client.send(Cmd::BiasTee, u32::from(on))?;
    }
    // Last, because they depend on nothing. A plain `rtl_tcp` server ignores
    // these opcodes and says nothing about it, which is the protocol working as
    // designed rather than a failure to detect.
    for (op, arg) in &p.rsp {
        client.send_raw(*op, *arg)?;
    }
    Ok(())
}

/// Convert a socket read into interleaved `f32`, in whichever format the far
/// end is sending.
///
/// The 8-bit path is the RTL-SDR's own table, shared byte for byte with the USB
/// backend. The 16-bit one exists only for an `rsp_tcp` server started with
/// `-b 16`: an RSP's ADC is 14-bit, so 8-bit throws away six bits of dynamic
/// range on the one receiver family here that has range worth keeping.
///
/// `carry` holds the bytes of a partial I/Q pair left by the previous read.
/// A TCP segment ends wherever it ends, so this is the normal case rather than
/// a rare one — and the pair is **two** bytes at 8-bit and **four** at 16-bit,
/// which is why the carry is a small buffer rather than the single byte the USB
/// path needs. Losing track of it does not corrupt one sample; it swaps I with
/// Q for the rest of the session, which reads as a mirrored spectrum rather
/// than as the framing slip it is.
fn convert_stream(
    format: RspSampleFormat,
    bytes: &[u8],
    lut: &[f32; 256],
    carry: &mut Vec<u8>,
    out: &mut Vec<f32>,
) {
    out.clear();
    // Bytes in one complete I/Q pair.
    let pair = format.bytes_per_component() * 2;

    let joined: &[u8] = if carry.is_empty() {
        bytes
    } else {
        carry.extend_from_slice(bytes);
        carry.as_slice()
    };

    let whole = joined.len() / pair * pair;
    out.reserve(whole / format.bytes_per_component());
    match format {
        RspSampleFormat::Uint8 => {
            for b in &joined[..whole] {
                out.push(lut[*b as usize]);
            }
        }
        RspSampleFormat::Int16 => {
            for w in joined[..whole].chunks_exact(2) {
                out.push(i16::from_le_bytes([w[0], w[1]]) as f32 / 32768.0);
            }
        }
    }

    // Keep whatever did not make up a whole pair. Written this way round —
    // build the tail, then replace the carry — because `joined` may be
    // borrowing the carry itself.
    let tail = joined[whole..].to_vec();
    *carry = tail;
}

/// Look for the `rsp_tcp` capability block that may follow the greeting.
///
/// Returns what was found and any bytes read that turned out to be samples.
///
/// The awkwardness is unavoidable and worth stating. There is no framing here:
/// after the greeting the socket carries either 45 bytes of capabilities and
/// then samples, or samples immediately, and both start with whatever the next
/// four bytes happen to be. So four bytes are read and compared against the
/// magic. If they do not match they are *samples* and are handed back to be
/// prepended to the stream — discarding them would put a two-sample hole at the
/// head of every ordinary `rtl_tcp` session, and reading them as capabilities
/// would put 45 bytes of noise at the head of every extended one.
fn peek_rsp(sock: &TcpStream, endpoint: &str) -> Result<(Option<RspCapabilities>, Vec<u8>)> {
    sock.set_read_timeout(Some(RSP_PEEK_TIMEOUT))
        .map_err(|e| Error::Net(format!("cannot set a timeout on {endpoint}: {e}")))?;

    let mut magic = [0u8; 4];
    match (&*sock).read_exact(&mut magic) {
        Ok(()) => {}
        // Nothing arrived. A server that has not started streaming yet is not
        // an extended one — the block comes immediately or not at all.
        Err(e) if would_block(&e) => return Ok((None, Vec::new())),
        Err(e) => {
            return Err(Error::Net(format!("{endpoint} closed after the greeting: {e}")));
        }
    }
    if &magic != proto::RSP_MAGIC {
        return Ok((None, magic.to_vec()));
    }

    let mut rest = [0u8; RSP_CAPABILITIES_LEN - 4];
    (&*sock).read_exact(&mut rest).map_err(|e| {
        Error::Net(format!(
            "{endpoint} sent an rsp_tcp capability header but only part of the \
             block that follows it: {e}"
        ))
    })?;
    let mut whole = magic.to_vec();
    whole.extend_from_slice(&rest);
    // The magic matched and the length is exact, so this cannot fail — but
    // returning the bytes as samples rather than panicking is the safe way to
    // be wrong.
    match RspCapabilities::parse(&whole) {
        Some(c) => Ok((Some(c), Vec::new())),
        None => Ok((None, whole)),
    }
}

fn pump(
    client: &mut Client,
    ctrl: &crossbeam_channel::Receiver<Ctrl>,
    rx: &mut Producer<f32>,
    shared: &Arc<Shared>,
    cfg: &RtlTcpConfig,
) -> Result<()> {
    let lut = sample_lut();
    let mut stats = RxStats::network(cfg.sample_rate_hz);
    let started = Instant::now();
    // A TCP read ends wherever the segment did, so an odd byte count is the
    // normal case here rather than the rare one it is over USB. Carrying it
    // into the next read is what keeps I with Q; dropping it would mirror the
    // spectrum for the rest of the session.
    let mut carry: Vec<u8> = Vec::new();
    let mut buf = vec![0u8; READ_BYTES];
    let mut scratch: Vec<f32> = Vec::with_capacity(READ_BYTES + 1);
    // 8-bit unsigned unless an extended `rsp_tcp` server said otherwise. There
    // is no way to discover a 16-bit stream without that block, so a server on
    // `-b 16` without `-E` is undetectable and reads as noise — said plainly in
    // the manual rather than guessed at here.
    let format = client.rsp.as_ref().map(|c| c.sample_format).unwrap_or_default();
    // Samples read while peeking for the capability block. They are the head of
    // the stream and have to go through the same conversion as everything else.
    if !client.prefix.is_empty() {
        let head = std::mem::take(&mut client.prefix);
        convert_stream(format, &head, &lut, &mut carry, &mut scratch);
        if !scratch.is_empty() {
            stats.on_iq(scratch.len() / 2);
            push_iq(rx, &scratch, &mut stats, shared.rx_paused.load(Ordering::Relaxed));
        }
    }

    loop {
        let mut pending = Pending::default();
        while let Ok(c) = ctrl.try_recv() {
            pending.absorb(c);
        }
        if pending.shutdown {
            break;
        }
        if !pending.is_empty() {
            // A failed write means the link is gone, which is a reason to stop
            // — unlike the USB backend, where a rejected setting is local and
            // the stream carries on.
            apply(client, &pending, &mut carry)?;
            publish_gain(shared, client);
        }

        match client.sock.read(&mut buf) {
            Ok(0) => {
                return Err(Error::Net(format!(
                    "the rtl_tcp server at {} closed the connection — it does \
                     that when a client stops keeping up, and when the dongle \
                     is unplugged from it",
                    client.endpoint
                )));
            }
            Ok(n) => {
                convert_stream(format, &buf[..n], &lut, &mut carry, &mut scratch);
                if !scratch.is_empty() {
                    stats.on_iq(scratch.len() / 2);
                    push_iq(rx, &scratch, &mut stats, shared.rx_paused.load(Ordering::Relaxed));
                    shared
                        .last_rx_ms
                        .store(started.elapsed().as_millis() as u64, Ordering::Relaxed);
                }
            }
            Err(e) if would_block(&e) => {}
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => {
                return Err(Error::Net(format!("lost the connection to {}: {e}", client.endpoint)));
            }
        }

        stats.tick();
    }
    Ok(())
}

/// A read that hit [`READ_TIMEOUT`] with nothing to show for it.
///
/// Two kinds, and both have to be caught: a socket read timeout surfaces as
/// `WouldBlock` on Unix and as `TimedOut` on Windows. Missing one turns every
/// quiet moment on that platform into a dropped connection.
fn would_block(e: &std::io::Error) -> bool {
    matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut)
}

/// Whether this connection can be tuned below the tuner's 24 MHz floor.
fn hf_capable(tuner: Tuner, hf_mode: RtlSdrHfMode) -> bool {
    tuner.upconverts_hf() || hf_mode != RtlSdrHfMode::Off
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A [`Client`] wired to a socket with a live peer, so the command writes
    /// the policy below performs go somewhere real. The peer never reads them;
    /// five bytes at a time, they fit in the socket buffer many times over.
    struct Peer {
        client: Client,
        _server: TcpStream,
        _listener: std::net::TcpListener,
    }

    fn peer(tuner: Tuner, hf_mode: RtlSdrHfMode) -> Peer {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("addr");
        let sock = TcpStream::connect(addr).expect("connect");
        let (server, _) = listener.accept().expect("accept");
        Peer {
            client: Client {
                sock,
                endpoint: addr.to_string(),
                greeting: Greeting { tuner, gain_count: 29 },
                center: 100e6,
                hf_mode,
                direct: 0,
                agc: RtlSdrAgc::Manual,
                gain_db: 30.0,
                bias_tee: false,
                rsp: None,
                prefix: Vec::new(),
            },
            _server: server,
            _listener: listener,
        }
    }

    /// The whole point of the hysteresis: a dial sitting on the crossover must
    /// not flip the far end's front end back and forth. The crossover itself is
    /// the tuner's floor, and the hysteresis is spent entirely above it.
    #[test]
    fn auto_hf_switches_once_and_holds() {
        let mut p = peer(Tuner::R820T, RtlSdrHfMode::Auto);
        let c = &mut p.client;
        // Above the crossover: the tuner.
        c.apply_hf_path(40e6).expect("write");
        assert_eq!(c.direct, 0);
        // 12 m and 10 m are the tuner's own — it reaches them, unaliased.
        c.apply_hf_path(24.915e6).expect("write");
        assert_eq!(c.direct, 0, "12 m belongs to the tuner");
        // Below the floor, where the ADC is the only way: direct sampling.
        c.apply_hf_path(21.074e6).expect("write");
        assert_eq!(c.direct, 2, "15 m has nowhere else to go");
        // Back up, but not clear of the band yet.
        c.apply_hf_path(24.5e6).expect("write");
        assert_eq!(c.direct, 2, "flipped back inside the hysteresis band");
        c.apply_hf_path(28.074e6).expect("write");
        assert_eq!(c.direct, 0);
    }

    /// A Blog V4 upconverts on the far end; asking it to direct sample would
    /// take away the HF it can actually hear.
    #[test]
    fn an_upconverting_dongle_is_left_alone_in_auto() {
        let mut p = peer(Tuner::R828D, RtlSdrHfMode::Auto);
        let c = &mut p.client;
        c.apply_hf_path(7.1e6).expect("write");
        assert_eq!(c.direct, 0);
    }

    /// ...but an explicit request is still obeyed, which is what a plain
    /// R828D dongle needs.
    #[test]
    fn direct_q_is_obeyed_even_on_an_r828d() {
        let mut p = peer(Tuner::R828D, RtlSdrHfMode::DirectQ);
        let c = &mut p.client;
        c.apply_hf_path(7.1e6).expect("write");
        assert_eq!(c.direct, 2);
        // And it stays there at VHF, because it was asked for.
        c.apply_hf_path(145e6).expect("write");
        assert_eq!(c.direct, 2);
    }

    #[test]
    fn hf_off_never_leaves_the_tuner() {
        let mut p = peer(Tuner::R820T, RtlSdrHfMode::Off);
        let c = &mut p.client;
        c.apply_hf_path(3.5e6).expect("write");
        assert_eq!(c.direct, 0);
        assert!(!hf_capable(Tuner::R820T, RtlSdrHfMode::Off));
        assert!(hf_capable(Tuner::R820T, RtlSdrHfMode::DirectQ));
        // An upconverting dongle reaches HF whatever this setting says.
        assert!(hf_capable(Tuner::R828D, RtlSdrHfMode::Off));
    }

    #[test]
    fn the_reported_gain_is_the_requested_one_unless_the_far_end_owns_it() {
        let mut p = peer(Tuner::R820T, RtlSdrHfMode::Auto);
        let c = &mut p.client;
        assert_eq!(c.effective_gain_db(), Some(30.0));
        c.agc = RtlSdrAgc::Tuner;
        assert_eq!(c.effective_gain_db(), None, "no figure to report under the far end's AGC");
    }
}
