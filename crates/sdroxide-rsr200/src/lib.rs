//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Steps 1–5 and 7 of that plan's suggested build order are done: protocol
//! (`protocol`), the transport-agnostic device layer (`device`), USB and
//! LAN transports (`usb`/`ffi`, `lan`), `Backend::Rsr200`'s registration in
//! `sdroxide-types`/`sdroxide-ui`, Separate mode
//! (`handle::Rsr200Handle::read_pair`, both ADCs interleaved through one
//! ring — see that method's own doc for why no `Pairer` like
//! `sdroxide-sdrplay`'s is needed here), and 24-bit/status readout (step 5
//! — `handle::Shared::status`, `Rsr200Handle::status`). Step 7 was done out
//! of order, ahead of steps 4–6, at Ralph's request; Windows USB still
//! needs its own research spike, see `RSR200_PLAN.md` §6.
//!
//! Verified against a real RSR200: LAN wired and over WiFi, USB on
//! Linux/macOS single-channel, Separate mode, and 24-bit, all three. Separate
//! mode has since been confirmed on real air, on two real antennas:
//! whole-span decorrelate nulls well, as intended — but decorrelate-per-bin
//! does not work on this radio as tested, wiping out the entire band rather
//! than nulling specific interferers. Not yet root-caused. Step 5's status
//! readout showed both a "no GPS fix" run and, on a later run of the
//! identical config, a genuine valid correction — GPS acquisition settling
//! after Start Stream is the obvious guess, not confirmed. See
//! `RSR200_PLAN.md`'s own step 3, 4, 5 and 7 entries for the full account of
//! each, including a real shutdown segfault USB testing found and fixed.
//! Not yet: the radio's own *hardware* combiner (a third, distinct wire
//! shape, step 6).

pub mod device;
pub mod error;
pub mod ffi;
pub mod handle;
pub mod lan;
pub mod protocol;
mod stream;
pub mod usb;

pub use handle::Rsr200Handle;
pub use usb::{UsbDeviceInfo, list_devices as list_usb_devices};
