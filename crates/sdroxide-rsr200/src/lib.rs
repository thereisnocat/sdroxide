//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Steps 1 (`protocol`) through 3 (`stream`/`handle`, the public API
//! `src/rsr200_source.rs` outside this crate calls, plus `Backend::Rsr200`'s
//! registration in `sdroxide-types`/`sdroxide-ui`) of that plan's suggested
//! build order are done. Not yet: 24-bit, dual channel (Separate mode +
//! `sdroxide_dsp::Diversity` wiring, step 4), hardware diversity (step 6), or
//! USB (step 7) — single channel, 16-bit, LAN is the whole of what streams
//! today, and none of it has been run against a real RSR200 yet.

pub mod device;
pub mod error;
pub mod handle;
pub mod lan;
pub mod protocol;
mod stream;

pub use handle::Rsr200Handle;
