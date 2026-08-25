//! An [`IqSource`] for a Reuter RSR200(B) driven over LAN by the native
//! driver in `sdroxide-rsr200` — no vendor SDK, no ExtIO.
//!
//! Receive only: the trait's transmit methods already default to errors,
//! which is the correct answer for this hardware (it has none).
//!
//! Single channel, 16-bit, LAN — the only wire shape `sdroxide-rsr200`
//! streams yet (`RSR200_PLAN.md` steps 1–3). USB, 24-bit and the
//! dual-channel Separate/Diversity modes are real capabilities of the radio
//! with no host-side wiring for them here yet.
//!
//! Verified working against a real RSR200 (2026-08-24), so far only over
//! WiFi: real spectrum on screen, tuning and the attenuators all live. One
//! caveat from that first run, recorded in `RSR200_PLAN.md`'s own step 3
//! entry rather than repeated here: brief dropouts at the lowest decimation
//! setting, consistent with WiFi's own bandwidth headroom rather than
//! anything wrong in this crate — not yet tried over a wired LAN.

use std::time::Duration;

use anyhow::Context;
use sdroxide_radio::{Complex32, IqSource, Result};
use sdroxide_rsr200::Rsr200Handle;
use sdroxide_types::Rsr200Config;

/// How long the radio may deliver nothing before the connection counts as
/// dead. LAN, so more generous than a local USB device's three seconds —
/// matching `sdroxide-rsr200::stream`'s own silence budget.
const SILENCE_BEFORE_REOPEN: Duration = Duration::from_secs(5);

pub struct Rsr200Source {
    handle: Rsr200Handle,
    center: f64,
    rx_scratch: Vec<f32>,
    label: String,
    /// Mirrors of the settings the panel drives live, so `current_gains`
    /// can answer without a round trip to the stream thread.
    attenuator1: i32,
    attenuator2: i32,
}

impl Rsr200Source {
    pub fn open(cfg: &Rsr200Config, center_hz: f64) -> anyhow::Result<Self> {
        let handle = Rsr200Handle::open(cfg, center_hz)
            .with_context(|| format!("opening the RSR200 at {}:{}", cfg.host, cfg.port))?;
        let label = handle.label.clone();
        tracing::info!("RSR200 source ready: {label}, center {center_hz:.0} Hz");
        Ok(Rsr200Source {
            center: center_hz,
            rx_scratch: Vec::new(),
            label,
            attenuator1: cfg.attenuator1,
            attenuator2: cfg.attenuator2,
            handle,
        })
    }
}

impl IqSource for Rsr200Source {
    /// The engine is transmitting and has stopped reading, or has started
    /// again. This radio has no transmitter of its own, but a panadapter
    /// receiver fed alongside a different transmitting radio still wants
    /// its ring accounted for correctly during an over — see
    /// [`IqSource::set_rx_paused`]. There is currently nothing to actually
    /// pass this through to: `sdroxide-rsr200`'s stream thread has no
    /// paused-accounting of its own yet (unlike the USB backends'), so a
    /// full ring during an over is reported the same as any other overrun.
    /// Worth fixing if this radio is ever run as a panadapter for a
    /// transmitting station; not yet done because nothing has needed it.
    fn set_rx_paused(&mut self, _paused: bool) {}

    fn sample_rate(&self) -> f64 {
        self.handle.sample_rate_hz
    }

    fn center_hz(&self) -> f64 {
        self.center
    }

    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        self.handle.set_center_hz(hz);
        Ok(())
    }

    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        let need = buf.len() * 2;
        if self.rx_scratch.len() < need {
            self.rx_scratch.resize(need, 0.0);
        }
        let n = self.handle.rx_read(&mut self.rx_scratch[..need]);
        let pairs = n / 2;
        if pairs == 0 {
            // Nothing yet — brief nap so the DSP loop doesn't spin hot.
            std::thread::sleep(Duration::from_millis(2));
            return Ok(0);
        }
        for (p, out) in buf.iter_mut().enumerate().take(pairs) {
            *out = Complex32::new(self.rx_scratch[2 * p], self.rx_scratch[2 * p + 1]);
        }
        Ok(pairs)
    }

    fn describe(&self) -> String {
        self.label.clone()
    }

    /// The two front-end attenuators. Both real elements — unlike, say,
    /// HydraSDR's curve-and-switches split, there is nothing here that
    /// isn't a plain dB value the radio already expresses as one.
    fn set_gain_element(&mut self, name: &str, db: f64) -> Result<()> {
        match name {
            Rsr200Config::ATT1_ELEMENT => {
                self.attenuator1 = (-db).round().clamp(0.0, f64::from(Rsr200Config::ATTENUATOR_MAX_DB)) as i32;
                self.handle.set_attenuator1_db(self.attenuator1);
            }
            Rsr200Config::ATT2_ELEMENT => {
                self.attenuator2 = (-db).round().clamp(0.0, f64::from(Rsr200Config::ATTENUATOR_MAX_DB)) as i32;
                self.handle.set_attenuator2_db(self.attenuator2);
            }
            _ => {}
        }
        Ok(())
    }

    /// Carried negated, like every other backend here: on the sliders more
    /// is louder, and an attenuator is the opposite sense from a gain.
    fn current_gains(&self) -> Vec<(String, f64)> {
        vec![
            (Rsr200Config::ATT1_ELEMENT.to_string(), -f64::from(self.attenuator1)),
            (Rsr200Config::ATT2_ELEMENT.to_string(), -f64::from(self.attenuator2)),
        ]
    }

    /// A radio whose connection has dropped, or whose thread has died, is
    /// reported as needing a reopen so the engine reconnects on its own.
    fn needs_reopen(&self) -> bool {
        !self.handle.is_alive() || self.handle.silent_for() >= SILENCE_BEFORE_REOPEN
    }

    /// Disconnect before the engine opens a replacement — without this,
    /// changing anything in Settings → Radio on a running RSR200 leaves the
    /// old connection dangling rather than actually reconfiguring the
    /// radio, since the new session's own commands would race the old
    /// one's.
    fn release(&mut self) {
        self.handle.release();
    }

    fn open_status(&self) -> Option<String> {
        Some(
            "Reuter RSR200 support is new: verified against real hardware over WiFi, not yet \
             over a wired LAN. Single channel, 16-bit only — 24-bit, dual channel and USB are \
             not wired up yet. Brief dropouts at low decimation are a known WiFi bandwidth \
             limitation, not a bug — a wired connection is expected to do better."
                .to_string(),
        )
    }
}
