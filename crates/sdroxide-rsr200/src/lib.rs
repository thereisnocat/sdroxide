//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! All eight steps of that plan's suggested build order are done: protocol
//! (`protocol`), the transport-agnostic device layer (`device`), USB and LAN
//! transports (`usb`/`ffi`, `lan`), `Backend::Rsr200`'s registration in
//! `sdroxide-types`/`sdroxide-ui`, Separate mode (`handle::Rsr200Handle::read_pair`,
//! both ADCs interleaved through one ring — see that method's own doc for why
//! no `Pairer` like `sdroxide-sdrplay`'s is needed here), 24-bit/status
//! readout (`handle::Shared::status`, `Rsr200Handle::status`), hardware
//! diversity (the radio's own combiner, `OpMode::Diversity`,
//! `device::Device::set_hardware_diversity[_from]`, `protocol::hardware_weight_for`
//! — all built and tested at the protocol/device layer since step 1, only
//! wired up to a `Backend::Rsr200` channel mode in step 6), and step 8's own
//! Auto-ATT, Serial mode, VHF/preamp switching and swap-channels. Steps 6, 7
//! and 8 were all done out of their own originally-suggested order, at
//! Ralph's request. Windows USB bindings (`ffi::Api`'s Windows-specific
//! signatures, `usb.rs`'s three cfg-gated call sites) are now written too,
//! against the vendor header and the SDR++ sibling's own hardware-verified
//! Windows implementation directly rather than guessed at — see `ffi.rs`'s
//! own module doc for the three real ABI differences from Linux/macOS. Not
//! yet run against a real Windows machine with the radio attached, unlike
//! everything else this doc comment claims as verified below.
//!
//! Verified against a real RSR200: LAN wired and over WiFi, USB on
//! Linux/macOS single-channel, Separate mode, hardware diversity, and
//! 24-bit, all four. Step 8 has **not** had its own real-hardware probe run
//! yet (Serial mode, Auto-ATT, VHF/preamp, swap-channels) — it shipped on
//! protocol-level confidence, the underlying commands already having been
//! hardware-verified at step 1, and on the DP itself, which is also how step
//! 8 found and fixed two real bugs in already-shipped code: `Single` channel
//! mode had been sending a documented-invalid `op_mode`/wire-format
//! combination since step 4, and `SW_ADC2_TO_HF2` — the switch-register bit
//! that actually routes ADC2 to the physical HF2 connector rather than
//! leaving it paralleled onto HF1 — had never been set in any dual-channel
//! mode, also since step 4. **That second bug means every "confirmed on
//! air" real-hardware result below, for Separate mode and hardware
//! diversity alike, was almost certainly testing two ADCs on the same HF1
//! antenna, not two genuinely independent aerials** — see `RSR200_PLAN.md`'s
//! own step 4/6 entries and their step-8 correction notes for the full
//! account, including what changed on retest against two real, physically
//! separate antennas once the routing bug was fixed: whole-span decorrelate
//! and the adaptive filter, both left running continuously rather than
//! frozen once converged, now show real instability against genuine
//! interference rather than the clean, stable result the pre-fix
//! (same-antenna) testing found — not a regression, but the first
//! non-degenerate test these techniques have had on this radio, and it
//! needs its own follow-up with Hold/Freeze engaged. Decorrelate-per-bin
//! still does not work on this radio as tested (wipes out the entire band
//! rather than nulling specific interferers), not yet root-caused, and not
//! yet retested against the routing fix either. Hardware diversity's own
//! mode switch and weight command are confirmed against real hardware, at
//! unity and at a real non-unity weight — but whether the *combining*
//! itself is correct (which channel carries the result, whether a solved
//! weight actually nulls or combines something real) still needs its own
//! retest with two genuinely independent aerials now that the routing bug
//! is fixed. Step 5's status readout showed both a "no GPS fix" run and, on
//! a later run of the identical config, a genuine valid correction — GPS
//! acquisition settling after Start Stream is the obvious guess, not
//! confirmed. See `RSR200_PLAN.md`'s own step 3–8 entries for the full
//! account of each, including a real shutdown segfault USB testing found
//! and fixed, and a real channel-2-needs-an-explicit-unity-weight bug
//! (found in the SDR++ sibling implementation, fixed here proactively
//! before it could bite) that step 6 carried forward.

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
