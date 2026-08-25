//! The LimeRFE RF front end: band filters, an LNA, a power amplifier and the
//! transmit/receive relay, in front of whatever radio is driving it.
//!
//! # Why this is its own crate
//!
//! The LimeRFE is not a LimeSDR accessory. It has its own micro-USB port and
//! its own microcontroller, and it sits in front of transceivers and other
//! SDRs as readily as in front of the board it was designed for. Keeping it
//! here — pure Rust, no LimeSuite, no libusb — means a LimeRFE works whatever
//! this program happens to be receiving with.
//!
//! The board's *other* control path, bit-banged I²C through the LimeSDR's GPIO
//! header, does need an open LimeSuite device. That implementation lives in
//! `sdroxide-lime` and reaches this crate through [`RfeTransport`]; the
//! dependency runs that way round so this crate stays usable without LimeSuite
//! installed at all.
//!
//! # Shape
//!
//! [`frame`] holds the wire with no serial port in it, which is what makes the
//! fiddly half testable with nothing plugged in — the same split every native
//! USB driver here makes. [`driver::Follower`] decides what to say and how
//! often, and is pure and clock-injected for the same reason. [`spawn`] puts a
//! transport on a thread, because both links block for far longer than the
//! engine's loop can spare.
//!
//! The frequency-to-channel map is *not* here: it lives in `sdroxide-types`, so
//! the settings panel can show which channel a dial resolves to while compiled
//! to wasm.
//!
//! NATIVE ONLY — links `serialport`; must never be a dependency of any
//! wasm-targeted crate.

pub mod driver;
pub mod error;
pub mod frame;
pub mod serial;
pub mod trace;
pub mod transport;

pub use driver::{Ctrl, Follower, LimeRfeHandle, Presence, spawn};
pub use error::{Error, Result};
pub use frame::{RfeInfo, RfeState};
pub use serial::SerialTransport;
pub use trace::diagnostics;
pub use transport::RfeTransport;

use sdroxide_types::{LimeRfeConfig, RfeLink};

/// Open the LimeRFE this configuration describes over its own serial port.
///
/// Returns `Ok(None)` when no board is configured — the ordinary case, and not
/// a failure. The board link ([`RfeLink::Board`]) is not reachable from here:
/// it needs an open LimeSuite device, so `sdroxide-lime` builds that transport
/// and calls [`spawn`] itself.
pub fn open_serial(cfg: &LimeRfeConfig) -> Result<Option<LimeRfeHandle>> {
    if cfg.link != RfeLink::Serial {
        return Ok(None);
    }
    let path = cfg.serial.path.trim();
    if path.is_empty() {
        return Ok(None);
    }
    let transport = SerialTransport::open(path)?;
    Ok(Some(spawn(Box::new(transport), cfg.clone())))
}
