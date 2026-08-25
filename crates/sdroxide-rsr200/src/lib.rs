//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Steps 1 (`protocol`) through 3 (`stream`/`handle`, the public API
//! `src/rsr200_source.rs` outside this crate calls, plus `Backend::Rsr200`'s
//! registration in `sdroxide-types`/`sdroxide-ui`) of that plan's suggested
//! build order are done and verified against a real RSR200 — LAN wired and
//! over WiFi, USB on Linux/macOS (`ffi`/`usb`, done out of order ahead of
//! steps 4–6 at Ralph's request; Windows still needs its own research spike,
//! see `RSR200_PLAN.md` §6). See `RSR200_PLAN.md`'s own step 3 and step 7
//! entries for what each run turned up, including a real shutdown segfault
//! USB testing found and fixed. Not yet: 24-bit or dual channel (Separate
//! mode + `sdroxide_dsp::Diversity` wiring, step 4) — single channel,
//! 16-bit is the whole of what streams today, over either transport.

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
