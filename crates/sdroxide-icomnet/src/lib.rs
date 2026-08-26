//! Icom IP-remote (RS-BA1) client — the LAN and WiFi ports on modern Icoms.
//!
//! NATIVE ONLY — plain UDP sockets and an OS thread; must never be a
//! dependency of any wasm-targeted crate.
//!
//! # What the radio offers, and what it does not
//!
//! Three things cross the LAN port: **CI-V control**, tunnelled over UDP with
//! the whole command set intact; **audio**, bidirectional PCM at up to 48 kHz;
//! and the radio's own **spectrum scope**, which arrives as CI-V `27 00`
//! sweeps like any other CI-V reply.
//!
//! There is no I/Q. No Icom offers it over a network port — the IC-7760's RF
//! deck has a USB 3.0 socket that does, through a manufacturer-supplied FTDI
//! driver, and none of it reaches this protocol. Two things stand in for it,
//! and this crate carries both:
//!
//! * The scope sweep — finished magnitude bins covering up to ±500 kHz, 475 of
//!   them on most models and 689 on an IC-7760. Not I/Q, but the engine
//!   already has a lane for pre-computed bins.
//! * `SET > Connectors > LAN AF/IF Output → Output Select = IF`, which swaps
//!   the audio stream from demodulated AF to a real 12 kHz IF. At 48 kHz that
//!   is genuine spectrum a receiver can be built on, about ±12 kHz of it.
//!
//! Which of those two the audio stream carries is a menu setting on the radio;
//! this crate ships the CI-V command to change it (see
//! [`protocol::Model::lan_afif_select`]) but takes no view on which is right.
//!
//! # Status
//!
//! Written against the published packet layout and the IC-7300MK2 CI-V
//! reference, and exercised end to end against [`sim`] — but **not verified on
//! hardware**. [`Trace`] is always on so that a first user with a real radio
//! can produce a report worth reading.
//!
//! # Example
//!
//! ```no_run
//! use sdroxide_icomnet::{IcomNetDevice, IcomNetOptions};
//!
//! let dev = IcomNetDevice::connect(IcomNetOptions {
//!     address: "192.168.1.50".into(),
//!     username: "operator".into(),
//!     password: "hunter2".into(),
//!     ..Default::default()
//! })?;
//! println!("{} on CI-V {:02X}h", dev.info().radio_name, dev.info().civ_address);
//! # Ok::<(), sdroxide_icomnet::Error>(())
//! ```

pub mod protocol;
pub mod sim;
mod stream;
mod trace;

mod net;

pub use net::{Error, IcomNetDevice, IcomNetOptions, SessionInfo, test_connection};
pub use protocol::{AudioCodec, Model, RadioCap};
pub use trace::{Counters, Trace};

/// What to ask a user for when this backend misbehaves.
///
/// Kept here rather than in the UI so the CLI, the settings dialog and the logs
/// all say the same thing.
pub const FIELD_REPORT_HINT: &str = "Only an IC-705 and an IC-7760 have been on \
    the other end of this so far, and only receiving. If it misbehaves, please \
    attach the session trace (Settings → Radio → Copy diagnostic report) to a bug \
    report — it contains the whole handshake and every CI-V frame exchanged with \
    the radio.";
