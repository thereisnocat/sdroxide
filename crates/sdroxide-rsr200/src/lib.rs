//! Reuter RSR200(B) support. See `RSR200_PLAN.md` at the workspace root for the
//! full plan this crate is built against.
//!
//! Step 1 of that plan's suggested build order: [`protocol`] only. Transport
//! (`usb`/`lan`), device sequencing and the rest of the backend are later
//! steps, not yet started.

pub mod protocol;
