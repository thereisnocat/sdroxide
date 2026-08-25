//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Steps 1 (`protocol`) through 3 (`stream`/`handle`, the public API
//! `src/rsr200_source.rs` outside this crate calls, plus `Backend::Rsr200`'s
//! registration in `sdroxide-types`/`sdroxide-ui`) of that plan's suggested
//! build order are done, and step 3 has been verified against a real RSR200
//! (2026-08-24, over WiFi — real spectrum, tuning and the attenuators all
//! working; see `RSR200_PLAN.md`'s own step 3 entry for the one caveat that
//! run turned up). Not yet: 24-bit, dual channel (Separate mode +
//! `sdroxide_dsp::Diversity` wiring, step 4), hardware diversity (step 6), or
//! USB (step 7) — single channel, 16-bit, LAN is the whole of what streams
//! today.

pub mod device;
pub mod error;
pub mod handle;
pub mod lan;
pub mod protocol;
mod stream;

pub use handle::Rsr200Handle;
