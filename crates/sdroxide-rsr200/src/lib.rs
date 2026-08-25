//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Steps 1–4 and 7 of that plan's suggested build order are done: protocol
//! (`protocol`), the transport-agnostic device layer (`device`), USB and
//! LAN transports (`usb`/`ffi`, `lan`), `Backend::Rsr200`'s registration in
//! `sdroxide-types`/`sdroxide-ui`, and Separate mode
//! (`handle::Rsr200Handle::read_pair`, both ADCs interleaved through one
//! ring — see that method's own doc for why no `Pairer` like
//! `sdroxide-sdrplay`'s is needed here). Step 7 was done out of order,
//! ahead of steps 4–6, at Ralph's request; Windows USB still needs its own
//! research spike, see `RSR200_PLAN.md` §6.
//!
//! Verified against a real RSR200: LAN wired and over WiFi, USB on
//! Linux/macOS single-channel and Separate mode both. Separate mode has
//! since been confirmed on real air, on two real antennas: whole-span
//! decorrelate nulls well, as intended — but decorrelate-per-bin does not
//! work on this radio as tested, wiping out the entire band rather than
//! nulling specific interferers. Not yet root-caused. See `RSR200_PLAN.md`'s
//! own step 3, 4 and 7 entries for what each run turned up, including a
//! real shutdown segfault USB testing found and fixed. Not yet: 24-bit, or
//! the radio's own *hardware* combiner (a third, distinct wire shape, step
//! 6).

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
