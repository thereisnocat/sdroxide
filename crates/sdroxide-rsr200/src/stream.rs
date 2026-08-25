//! The worker thread: owns the transport and the [`Device`], drains one,
//! feeds the other, and pushes converted samples through an `rtrb` ring —
//! same shape as every other native backend's stream thread in this
//! workspace.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use rtrb::Producer;
use sdroxide_types::{Rsr200Config, Rsr200Transport};

use crate::device::{Config, Device, Transport, TransportKind};
use crate::error::{Error, Result};
use crate::handle::{Ctrl, Pending, Rsr200Handle, Shared, push_iq, ring_for};
use crate::lan::LanTcpTransport;
use crate::protocol::{BlockLayout, OpMode, StreamFormat};
use crate::usb::UsbTransport;

/// Whichever transport this session opened, behind one [`Transport`] impl
/// that just delegates — an enum rather than `Box<dyn Transport>` because
/// [`RunTransport::close`] needs each variant's own inherent teardown (LAN's
/// own `stop()` unblocks a read from another thread before `close()`; USB's
/// [`UsbTransport::close`] is self-contained), which is not part of the
/// [`Transport`] trait itself.
enum RunTransport {
    Lan(LanTcpTransport),
    Usb(UsbTransport),
}

impl RunTransport {
    fn close(&mut self) {
        match self {
            RunTransport::Lan(t) => {
                t.stop();
                t.close();
            }
            RunTransport::Usb(t) => t.close(),
        }
    }
}

impl Transport for RunTransport {
    fn kind(&self) -> TransportKind {
        match self {
            RunTransport::Lan(t) => t.kind(),
            RunTransport::Usb(t) => t.kind(),
        }
    }

    fn send_command(&mut self, data: &[u8]) -> bool {
        match self {
            RunTransport::Lan(t) => t.send_command(data),
            RunTransport::Usb(t) => t.send_command(data),
        }
    }

    fn next_frame(&mut self, out: &mut Vec<u8>) -> bool {
        match self {
            RunTransport::Lan(t) => t.next_frame(out),
            RunTransport::Usb(t) => t.next_frame(out),
        }
    }

    /// USB ignores block layout entirely — it reads whole 4096-byte packets
    /// from a fixed endpoint, nothing to frame — matching
    /// [`Transport::set_layout`]'s own default no-op.
    fn set_layout(&mut self, layout: BlockLayout) {
        if let RunTransport::Lan(t) = self {
            t.set_layout(layout);
        }
    }

    /// Only LAN has a standalone-packet concept at all — see
    /// [`Transport::read_packet`]'s own doc for why USB never calls this.
    fn read_packet(&mut self, out: &mut Vec<u8>, expected_bytes: usize) -> bool {
        match self {
            RunTransport::Lan(t) => t.read_packet(out, expected_bytes),
            RunTransport::Usb(_) => false,
        }
    }

    fn last_error(&self) -> Option<&str> {
        match self {
            RunTransport::Lan(t) => t.last_error(),
            RunTransport::Usb(t) => t.last_error(),
        }
    }
}

/// How long a single LAN read may block before the loop goes back to serve
/// control and the acknowledgement timer. Bounds how long a retune or a
/// shutdown request waits when the stream has gone quiet; while samples are
/// flowing, reads return as soon as anything has arrived and this never
/// comes into play.
const READ_TIMEOUT: Duration = Duration::from_millis(200);

/// How often [`Device::service`] gets a chance to notice and retry an
/// unacknowledged command.
const SERVICE_INTERVAL: Duration = Duration::from_millis(100);

/// How long the radio may deliver nothing before the connection counts as
/// dead. LAN, so more generous than a local USB device's three seconds —
/// there is a real network between here and the radio.
const SILENCE_BEFORE_DROP: Duration = Duration::from_secs(5);

/// Connect, configure, and start the stream thread. Blocks until the
/// connection is up and the radio has been configured, or has failed — so a
/// wrong address or a radio that is not listening comes back as an ordinary
/// error rather than as a stream that never starts.
pub(crate) fn spawn(cfg: &Rsr200Config, center_hz: f64) -> Result<Rsr200Handle> {
    if cfg.transport == Rsr200Transport::Lan && cfg.host.is_empty() {
        return Err(Error::Net("no RSR200 host configured".to_string()));
    }

    let (ctrl_tx, ctrl_rx) = crossbeam_channel::unbounded::<Ctrl>();
    let (ready_tx, ready_rx) = crossbeam_channel::bounded::<Result<f64>>(1);
    let shared = Arc::new(Shared { alive: AtomicBool::new(true), last_rx_ms: AtomicU64::new(0) });
    let (rx_prod, rx_cons) = ring_for(cfg.sample_rate_hz());

    let cfg = cfg.clone();
    let where_from = match cfg.transport {
        Rsr200Transport::Lan => format!("{}:{}", cfg.host, cfg.port),
        Rsr200Transport::Usb if cfg.usb_serial.is_empty() => "USB".to_string(),
        Rsr200Transport::Usb => format!("USB {}", cfg.usb_serial),
    };
    let thread_shared = Arc::clone(&shared);
    let join = std::thread::Builder::new()
        .name("sdroxide-rsr200".into())
        .spawn(move || {
            run(cfg, center_hz, ctrl_rx, rx_prod, Arc::clone(&thread_shared), ready_tx);
            thread_shared.alive.store(false, Ordering::Relaxed);
        })
        .map_err(|e| Error::Net(format!("could not start the RSR200 stream thread: {e}")))?;

    match ready_rx.recv() {
        Ok(Ok(rate_hz)) => {
            let label = format!("Reuter RSR200 @ {where_from}, {:.3} Msps", rate_hz / 1e6);
            Ok(Rsr200Handle::from_parts(rx_cons, ctrl_tx, shared, join, label, rate_hz))
        }
        Ok(Err(e)) => {
            let _ = join.join();
            Err(e)
        }
        Err(_) => {
            let _ = join.join();
            Err(Error::Net("the stream thread exited before connecting".to_string()))
        }
    }
}

fn run(
    cfg: Rsr200Config,
    center_hz: f64,
    ctrl_rx: Receiver<Ctrl>,
    mut rx: Producer<f32>,
    shared: Arc<Shared>,
    ready_tx: Sender<Result<f64>>,
) {
    let mut transport = match cfg.transport {
        Rsr200Transport::Lan => {
            let mut t = LanTcpTransport::new();
            if !t.connect(&cfg.host, cfg.port, READ_TIMEOUT) {
                let msg = t.last_error().unwrap_or("connect failed").to_string();
                let _ = ready_tx.send(Err(Error::Net(msg)));
                return;
            }
            RunTransport::Lan(t)
        }
        Rsr200Transport::Usb => match UsbTransport::open(&cfg.usb_serial) {
            Ok(t) => RunTransport::Usb(t),
            Err(e) => {
                let _ = ready_tx.send(Err(Error::Usb(e)));
                return;
            }
        },
    };

    // Single channel, 16 bit -- the only wire shape either transport
    // produces yet. `OpMode::Independent` on a one-channel format is ADC1
    // alone, unmixed with ADC2 -- the plain single-receiver behaviour this
    // backend wants for now.
    let mut dev_cfg = Config {
        adc_clock_hz: cfg.adc_clock_hz,
        gps_discipline: cfg.gps_discipline,
        decimation_exp: cfg.decimation_exp,
        format: StreamFormat { channels: 1, bits: 16 },
        op_mode: OpMode::Independent,
        tuned_hz: center_hz,
        attenuator1: cfg.attenuator1,
        attenuator2: cfg.attenuator2,
        ..Config::default()
    };

    let mut device = Device::new();
    let started = Instant::now();

    if let Err(e) = device.apply_config(&mut transport, &dev_cfg, now_ms(started)) {
        let _ = ready_tx.send(Err(Error::Net(format!("configuring the radio failed: {e}"))));
        return;
    }
    if let Err(e) = device.start_stream(&mut transport, now_ms(started)) {
        let _ = ready_tx.send(Err(Error::Net(format!("starting the stream failed: {e}"))));
        return;
    }

    if ready_tx.send(Ok(device.sample_rate())).is_err() {
        // The caller gave up waiting (should not happen — spawn() blocks on
        // this very channel — but a dropped receiver is not a reason to
        // leak a streaming connection).
        let _ = device.stop_stream(&mut transport, now_ms(started));
        return;
    }

    let mut pending = Pending::default();
    let mut out_a: Vec<f32> = Vec::new();
    let mut out_b: Vec<f32> = Vec::new(); // unused (single channel); pump() still wants a slot
    let mut iq_scratch: Vec<f32> = Vec::new();
    let mut last_service = Instant::now();

    loop {
        while let Ok(c) = ctrl_rx.try_recv() {
            pending.absorb(c);
        }
        if pending.shutdown {
            break;
        }
        if !pending.is_empty() {
            let mut cfg_changed = false;
            if let Some(v) = pending.attenuator1.take() {
                dev_cfg.attenuator1 = v;
                cfg_changed = true;
            }
            if let Some(v) = pending.attenuator2.take() {
                dev_cfg.attenuator2 = v;
                cfg_changed = true;
            }
            if cfg_changed
                && let Err(e) = device.apply_config(&mut transport, &dev_cfg, now_ms(started))
            {
                tracing::warn!("RSR200: reconfigure failed: {e}");
            }
            if let Some(hz) = pending.center.take() {
                dev_cfg.tuned_hz = hz;
                if let Err(e) = device.tune(&mut transport, hz, now_ms(started)) {
                    tracing::warn!("RSR200: retune failed: {e}");
                }
            }
            pending = Pending::default();
        }

        if last_service.elapsed() >= SERVICE_INTERVAL {
            if let Err(e) = device.service(&mut transport, now_ms(started)) {
                tracing::warn!("RSR200: {e}");
            }
            last_service = Instant::now();
        }

        match device.pump(&mut transport, &mut out_a, &mut out_b) {
            Some(outcome) => {
                if let Some(err) = &outcome.error {
                    tracing::warn!("RSR200: {err}");
                }
                if let Some(sb) = outcome.samples {
                    let need = sb.frames * 2;
                    iq_scratch.clear();
                    iq_scratch.extend_from_slice(&out_a[..need]);
                    if !push_iq(&mut rx, &iq_scratch) {
                        tracing::debug!("RSR200: RX ring full, {} sample(s) dropped", sb.frames);
                    }
                    shared.last_rx_ms.store(started.elapsed().as_millis() as u64, Ordering::Relaxed);
                }
            }
            None => {
                let msg = transport.last_error().unwrap_or("connection lost").to_string();
                tracing::warn!("RSR200: {msg}");
                break;
            }
        }

        // A dead link that keeps handing back *something* (a well-behaved
        // TCP connection should not, but see `Transport::next_frame`'s own
        // "stopped or failed look the same" note) is caught here rather
        // than trusted to `pump()` alone.
        if shared.last_rx_ms.load(Ordering::Relaxed) > 0
            && Duration::from_millis(started.elapsed().as_millis() as u64 - shared.last_rx_ms.load(Ordering::Relaxed))
                > SILENCE_BEFORE_DROP
        {
            tracing::warn!("RSR200: no samples for {SILENCE_BEFORE_DROP:?}, giving up");
            break;
        }
    }

    let _ = device.stop_stream(&mut transport, now_ms(started));
    transport.close();
}

fn now_ms(started: Instant) -> u64 {
    started.elapsed().as_millis() as u64
}
