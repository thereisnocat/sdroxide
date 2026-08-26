//! An [`IqSource`] for a SmartSDR radio (FlexRadio FLEX-6000 / FLEX-8000).
//!
//! Receive is wideband IQ over a DAX IQ stream → the engine's normal DDC/demod
//! path (`audio_mode = false`); transmit is 24 kHz audio (`tx_write_audio`)
//! which the radio modulates (`caps.tx_audio`). Control (frequency, mode, PTT,
//! power) rides the ASCII command protocol on TCP 4992.
//!
//! Shaped like [`crate::tci_source`], because the two rigs present the same way:
//! somebody else's transceiver, streaming us IQ and modulating audio we send
//! back. The differences are the ceiling — DAX IQ stops at 192 kHz — and that
//! this one has never been run against real hardware, which is why every
//! connection carries a diagnostic trace.

use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, ControlUpdate, IqSource, Result};
use sdroxide_smartsdr::{FlexHandle, FlexUpdate};
use sdroxide_types::TxTelemetry;

use crate::session_trace::TraceStore;

/// The last session's trace of each radio, kept after the source is dropped.
/// See [`crate::session_trace`] — including why there is one per radio rather
/// than one for the process.
static TRACES: TraceStore = TraceStore::new();

/// Which FlexRadio a session was with: the address it was dialled at, the one
/// thing both the source and the tab asking for the report can spell.
fn session_key(cfg: &sdroxide_types::SmartSdrConfig) -> String {
    cfg.target().unwrap_or("").trim().to_ascii_lowercase()
}

fn record_trace(key: &str, trace: &sdroxide_smartsdr::Trace) {
    TRACES.record(key, trace.dump());
}

pub struct SmartSdrSource {
    handle: FlexHandle,
    /// Which radio this session is with, for [`TRACES`] — see the field of the
    /// same name on [`crate::icomnet_source::IcomNetSource`].
    key: String,
    center: f64,
    scratch: Vec<f32>,
    label: String,
    /// Last VFO-within-span offset pushed to the radio (dedup).
    if_offset: f64,
    /// Latest telemetry the radio reported while keyed, latched for the engine's
    /// meter poll. Cleared on unkey.
    last_telem: Option<TxTelemetry>,
    /// Kept so the trace survives the handle being dropped.
    trace: sdroxide_smartsdr::Trace,
    /// Warning captured while opening, surfaced in the UI.
    warning: Option<String>,
}

impl SmartSdrSource {
    pub fn open(
        cfg: &sdroxide_types::SmartSdrConfig,
        center_hz: f64,
    ) -> anyhow::Result<SmartSdrSource> {
        let address = cfg
            .target()
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "no FlexRadio selected — press Discover in Settings → Radio, or enter \
                     the radio's address"
                )
            })?
            .to_string();

        let handle =
            FlexHandle::connect(&address, cfg.iq_sample_rate_hz, cfg.iq_channel, &cfg.station)?;
        let trace = handle.trace.clone();

        // The radio creates every DAX IQ stream at 48 kHz and only moves on a
        // follow-up command; if it refused that, the engine would run its whole
        // DSP chain at a rate the samples are not arriving at, which sounds like
        // a badly detuned receiver rather than like an error. Say so instead.
        let mut warning = None;
        let actual = wait_for_rate(&handle, cfg.iq_sample_rate_hz);
        if (actual - cfg.iq_sample_rate_hz).abs() > 1.0 {
            warning = Some(format!(
                "the radio is streaming {:.0} kHz IQ, not the {:.0} kHz requested — \
                 DAX IQ channel {} may be shared, or the radio declined the rate",
                actual / 1000.0,
                cfg.iq_sample_rate_hz / 1000.0,
                cfg.iq_channel
            ));
            tracing::warn!(target: "smartsdr", "{}", warning.as_deref().unwrap_or(""));
        }

        let label = format!(
            "{} @ {address} ({:.0} kHz DAX IQ)",
            handle.ident.display_name(),
            actual / 1000.0,
        );
        handle.set_center(center_hz);

        Ok(SmartSdrSource {
            center: center_hz,
            scratch: Vec::new(),
            label,
            handle,
            key: session_key(cfg),
            if_offset: 0.0,
            last_telem: None,
            trace,
            warning,
        })
    }

    pub fn model(&self) -> &str {
        &self.handle.ident.model
    }
}

/// Wait briefly for the radio's first packets, which are what report the rate it
/// really chose, then return it.
///
/// Bounded rather than open-ended: a radio that never streams is a fault to
/// report, not a reason to hang the whole startup.
fn wait_for_rate(handle: &FlexHandle, requested: f64) -> f64 {
    let deadline = Instant::now() + Duration::from_millis(1500);
    while Instant::now() < deadline {
        let rate = handle.actual_rate_hz();
        if (rate - requested).abs() < 1.0 {
            return rate;
        }
        std::thread::sleep(Duration::from_millis(25));
    }
    handle.actual_rate_hz()
}

impl Drop for SmartSdrSource {
    fn drop(&mut self) {
        record_trace(&self.key, &self.trace);
    }
}

impl IqSource for SmartSdrSource {
    fn sample_rate(&self) -> f64 {
        self.handle.actual_rate_hz()
    }

    fn center_hz(&self) -> f64 {
        self.center
    }

    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        self.handle.set_center(hz);
        Ok(())
    }

    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        let need = buf.len() * 2;
        if self.scratch.len() < need {
            self.scratch.resize(need, 0.0);
        }
        let n = self.handle.rx_read(&mut self.scratch[..need]);
        let pairs = n / 2;
        if pairs == 0 {
            std::thread::sleep(Duration::from_millis(2));
            return Ok(0);
        }
        for p in 0..pairs {
            buf[p] = Complex32::new(self.scratch[2 * p], self.scratch[2 * p + 1]);
        }
        Ok(pairs)
    }

    fn describe(&self) -> String {
        self.label.clone()
    }

    fn open_status(&self) -> Option<String> {
        self.warning.clone()
    }

    fn set_control_mode(&mut self, mode: sdroxide_types::Mode) -> Result<()> {
        self.handle.set_mode(mode);
        Ok(())
    }

    fn poll_control(&mut self) -> Vec<ControlUpdate> {
        self.handle
            .poll_updates()
            .into_iter()
            .map(|u| match u {
                FlexUpdate::Freq(hz) => ControlUpdate::Freq(hz),
                FlexUpdate::Mode(m) => ControlUpdate::Mode(m),
                FlexUpdate::TxDrive(f) => ControlUpdate::TxDrive(f),
                FlexUpdate::TuneDrive(f) => ControlUpdate::TuneDrive(f),
            })
            .collect()
    }

    fn tx_begin(&mut self, center_hz: f64, _rate: f64) -> Result<f64> {
        Ok(self.handle.tx_begin(center_hz))
    }

    fn tx_write_audio(&mut self, audio: &[f32]) -> Result<()> {
        self.handle.tx_write(audio);
        Ok(())
    }

    /// Let the queued audio reach the radio before PTT drops. The engine hands
    /// us a burst faster than real time and the radio plays it out at 24 kHz, so
    /// unkeying immediately would cut the tail (FT8 needs every symbol).
    fn tx_drain(&mut self) {
        let deadline = Instant::now() + Duration::from_secs(2);
        while self.handle.tx_pending() > 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
        }
        // The last packet handed over is still playing out in the radio.
        std::thread::sleep(Duration::from_millis(40));
    }

    fn tx_end(&mut self) -> Result<()> {
        self.handle.tx_end();
        self.last_telem = None; // drop the stale SWR reading on unkey
        Ok(())
    }

    fn discard_pending_rx(&mut self) {
        self.handle.discard_pending_rx();
    }

    fn tx_telemetry(&mut self) -> Option<TxTelemetry> {
        if let Some(t) = self.handle.poll_telemetry() {
            self.last_telem = Some(t);
        }
        self.last_telem
    }

    fn set_tx_drive(&mut self, frac: f64) {
        self.handle.set_drive(frac);
    }

    fn set_tune_drive(&mut self, frac: f64) {
        self.handle.set_tune_drive(frac);
    }

    /// `transmit set rfpower=` is the radio's real power control, so the TX
    /// audio we stream must stay at full scale.
    fn commands_tx_power(&self) -> bool {
        true
    }

    /// The control thread stops when the radio closes the TCP session (powered
    /// off, or another client evicted ours); the engine then reconnects on its
    /// own.
    fn needs_reopen(&self) -> bool {
        !self.handle.is_alive()
    }

    fn set_if_offset(&mut self, hz: f64) {
        if (hz - self.if_offset).abs() > 0.5 {
            self.if_offset = hz;
            self.handle.set_if_offset(hz);
        }
    }

    /// Hand the radio's streams back before the engine builds a replacement.
    ///
    /// Not strictly required — a FLEX reaps a client's streams when its TCP
    /// session ends — but the DAX IQ channel is a limited resource, and a
    /// reconnect that raced its own predecessor for channel 1 would fail with
    /// "channel in use" against a stream we ourselves had just abandoned.
    fn release(&mut self) {
        record_trace(&self.key, &self.trace);
        self.handle.tx_end();
    }
}

/// Discover FlexRadios and map them to the settings UI's device type.
pub fn discover() -> Vec<sdroxide_types::SmartSdrDevice> {
    sdroxide_smartsdr::discover_default()
        .into_iter()
        .map(|r| sdroxide_types::SmartSdrDevice {
            joinable: r.joinable(),
            ip: r.ip,
            port: r.port,
            model: r.model,
            serial: r.serial,
            nickname: r.nickname,
            version: r.version,
            gui_clients: r.gui_clients,
        })
        .collect()
}

/// Shared with the settings UI's "Copy diagnostic report" button: the trace of
/// the radio *this* configuration names, never another one's.
pub fn diagnostics_or_hint(cfg: &sdroxide_types::SmartSdrConfig) -> String {
    match TRACES.get(&session_key(cfg)) {
        Some(t) => format!("{t}\n{}\n", sdroxide_smartsdr::FIELD_REPORT_HINT),
        None => format!(
            "No SmartSDR session has run yet for {} — connect to that radio first.\n\n{}\n",
            cfg.target().unwrap_or("this radio"),
            sdroxide_smartsdr::FIELD_REPORT_HINT
        ),
    }
}
