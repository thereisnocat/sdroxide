//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Step 1 (`protocol`) and step 2 (`device` + `lan`) of that plan's
//! suggested build order are done — "LAN transport + device.rs, single
//! channel, 16-bit — first light," per the plan's own words. Step 3
//! (`Backend::Rsr200` registration in the main binary) onward is not yet
//! started; there is no `stream.rs`/`handle.rs` here yet either, since
//! those are what step 3 needs and nothing before it does.

pub mod device;
pub mod error;
pub mod lan;
pub mod protocol;
