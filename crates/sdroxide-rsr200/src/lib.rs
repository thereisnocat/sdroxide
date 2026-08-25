//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Steps 1–7 of that plan's suggested build order are done — only step 8's
//! own lowest-priority extras (Auto-ATT, Serial mode, VHF/preamp switching)
//! remain: protocol (`protocol`), the transport-agnostic device layer
//! (`device`), USB and LAN transports (`usb`/`ffi`, `lan`),
//! `Backend::Rsr200`'s registration in `sdroxide-types`/`sdroxide-ui`,
//! Separate mode (`handle::Rsr200Handle::read_pair`, both ADCs interleaved
//! through one ring — see that method's own doc for why no `Pairer` like
//! `sdroxide-sdrplay`'s is needed here), 24-bit/status readout
//! (`handle::Shared::status`, `Rsr200Handle::status`), and hardware
//! diversity (the radio's own combiner, `OpMode::Diversity`,
//! `device::Device::set_hardware_diversity[_from]`, `protocol::hardware_weight_for`
//! — all built and tested at the protocol/device layer since step 1, only
//! wired up to a `Backend::Rsr200` channel mode in step 6). Steps 6 and 7
//! were both done out of order, ahead of 4–6's own suggested sequence, at
//! Ralph's request; Windows USB still needs its own research spike, see
//! `RSR200_PLAN.md` §6.
//!
//! Verified against a real RSR200: LAN wired and over WiFi, USB on
//! Linux/macOS single-channel, Separate mode, hardware diversity, and
//! 24-bit, all four. Separate mode has been confirmed on real air, on two
//! real antennas: whole-span decorrelate nulls well, as intended — but
//! decorrelate-per-bin does not work on this radio as tested, wiping out
//! the entire band rather than nulling specific interferers. Not yet
//! root-caused. Hardware diversity's own mode switch and weight command
//! are confirmed against real hardware, at unity and at a real non-unity
//! weight — but whether the *combining* itself is correct (which channel
//! carries the result, whether a solved weight actually nulls or combines
//! something real) still needs two real aerials and a human listening, the
//! same milestone Separate mode already reached. Step 5's status readout
//! showed both a "no GPS fix" run and, on a later run of the identical
//! config, a genuine valid correction — GPS acquisition settling after
//! Start Stream is the obvious guess, not confirmed. See `RSR200_PLAN.md`'s
//! own step 3–7 entries for the full account of each, including a real
//! shutdown segfault USB testing found and fixed, and a real
//! channel-2-needs-an-explicit-unity-weight bug (found in the SDR++ sibling
//! implementation, fixed here proactively before it could bite) that step
//! 6 carried forward.

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
