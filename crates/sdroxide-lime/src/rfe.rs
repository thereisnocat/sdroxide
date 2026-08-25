//! The LimeRFE reached through the SDR board rather than its own USB port.
//!
//! LimeSuite drives it by bit-banging I²C on the LimeSDR's GPIO pins, which
//! means every exchange is hundreds of USB control transfers. It is the slow
//! path by a wide margin — the serial link in `sdroxide-limerfe` costs tens of
//! milliseconds and this costs the better part of a second — and that number is
//! reported honestly through [`RfeTransport::round_trip`] so the rate limit
//! above can be derived from it rather than guessed.
//!
//! This is also the one LimeRFE path that needs LimeSuite at all, which is why
//! it lives here and not in `sdroxide-limerfe`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use sdroxide_limerfe::{Error as RfeError, Result as RfeResult, RfeInfo, RfeState, RfeTransport};
use sdroxide_types::RfeMode;

use crate::device::DevCtl;
use crate::ffi;

/// One exchange over the bit-banged bus.
///
/// An estimate, not a measurement: a 16-byte frame is ~1150 USB control
/// transfers at four GPIO operations per bit, and a control transfer on a
/// healthy bus is around half a millisecond. It is deliberately generous — the
/// cost of over-estimating is a slower rate limit, and the cost of
/// under-estimating is a queue that never drains.
const BOARD_ROUND_TRIP: Duration = Duration::from_millis(700);

pub struct BoardTransport {
    api: Arc<ffi::Api>,
    /// Held so the radio cannot be closed out from under the bit-banged I²C:
    /// these calls reach the board through the *device's* GPIO pins, so the
    /// device has to outlive them. Locked for the duration of each call, which
    /// is the boundary LimeSuite itself draws — stream calls take a stream
    /// pointer and never come here.
    ctl: Arc<Mutex<DevCtl>>,
    rfe: ffi::RfeDev,
    label: String,
}

// The handle is only reachable through `&mut self` and the type is not `Clone`.
unsafe impl Send for BoardTransport {}

impl BoardTransport {
    /// Open the LimeRFE bolted to an already-open LimeSDR.
    ///
    /// Takes the shared device rather than a bare pointer, so the radio cannot
    /// be closed while this transport still exists — which is what makes this
    /// safe to call rather than an `unsafe fn` with a contract to remember.
    pub fn open(ctl: Arc<Mutex<DevCtl>>, label: &str) -> RfeResult<BoardTransport> {
        let api = {
            let guard = ctl.lock().unwrap_or_else(|e| e.into_inner());
            Arc::clone(guard.api())
        };
        if !api.has_rfe() {
            return Err(RfeError::NoLibrarySupport);
        }
        let open = api.rfe_open.ok_or(RfeError::NoLibrarySupport)?;
        // NULL port plus a device handle is LimeSuite's way of saying "through
        // the board".
        let rfe = {
            let guard = ctl.lock().unwrap_or_else(|e| e.into_inner());
            unsafe { open(std::ptr::null(), guard.raw()) }
        };
        if rfe.is_null() {
            return Err(RfeError::NoAnswer { path: format!("{label} GPIO header") });
        }
        let mut t = BoardTransport { api, ctl, rfe, label: label.to_string() };
        // Confirm something is actually on the bus rather than trusting a
        // non-null handle: the GPIO pins exist whether or not a board is wired
        // to them.
        let info = t.info()?;
        t.label = format!(
            "LimeRFE through {label} (firmware {}, hardware {})",
            info.firmware, info.hardware
        );
        Ok(t)
    }

    fn check(&self, rc: std::ffi::c_int) -> RfeResult<()> {
        if rc == 0 {
            Ok(())
        } else {
            // The board's own refusal codes arrive here as small negatives and
            // positives; `from_board` turns each into the fix for it.
            Err(RfeError::from_board(rc as i8 as u8))
        }
    }

    /// Put one LimeRFE transaction in the radio's diagnostic report.
    ///
    /// The front end is the half of this path that has no report of its own on
    /// the board link — it *is* the radio, electrically — and "the amplifier
    /// answered and passed nothing" is not answerable without knowing which
    /// channel and which connector it was told to use.
    fn record(&self, call: &'static str, detail: impl AsRef<str>, out: &RfeResult<()>) {
        let dev = self.ctl.lock().unwrap_or_else(|e| e.into_inner());
        dev.trace().call(
            call,
            detail,
            match out {
                Ok(()) => "ok".to_string(),
                Err(e) => format!("FAILED: {e}"),
            },
        );
    }
}

impl RfeTransport for BoardTransport {
    fn info(&mut self) -> RfeResult<RfeInfo> {
        let f = self.api.rfe_get_info.ok_or(RfeError::NoLibrarySupport)?;
        let mut buf = [0u8; 4];
        let rc = {
            let _dev = self.ctl.lock().unwrap_or_else(|e| e.into_inner());
            unsafe { f(self.rfe, buf.as_mut_ptr()) }
        };
        self.check(rc)?;
        // `cinfo[0]` is the firmware version here, where the *raw wire* puts it
        // at `buf[1]` — see `sdroxide_limerfe::frame::decode_info`. Not an
        // inconsistency to tidy away: the C API has already stripped the echoed
        // command byte that the serial transport still has to skip. Making the
        // two agree would break one of them.
        Ok(RfeInfo { firmware: buf[0], hardware: buf[1] })
    }

    fn configure(&mut self, state: RfeState) -> RfeResult<()> {
        let f = self.api.rfe_configure_state.ok_or(RfeError::NoLibrarySupport)?;
        let st = ffi::RfeBoardState {
            channel_id_rx: state.channel_rx.code() as std::ffi::c_char,
            channel_id_tx: state.channel_tx.code() as std::ffi::c_char,
            sel_port_rx: state.port_rx.code() as std::ffi::c_char,
            sel_port_tx: state.port_tx.code() as std::ffi::c_char,
            mode: state.mode.code() as std::ffi::c_char,
            notch_on_off: std::ffi::c_char::from(state.notch),
            att_value: state.atten_steps.min(7) as std::ffi::c_char,
            enable_swr: std::ffi::c_char::from(state.swr_enable),
            source_swr: std::ffi::c_char::from(state.swr_source_cell),
        };
        let rc = {
            let _dev = self.ctl.lock().unwrap_or_else(|e| e.into_inner());
            unsafe { f(self.rfe, st) }
        };
        let out = self.check(rc);
        self.record(
            "RFE_ConfigureState",
            format!(
                "{} in on {}, {} out on {}, {}",
                state.channel_rx.label(),
                state.port_rx.label(),
                state.channel_tx.label(),
                state.port_tx.label(),
                state.mode.label()
            ),
            &out,
        );
        out
    }

    fn set_mode(&mut self, mode: RfeMode) -> RfeResult<()> {
        let f = self.api.rfe_mode.ok_or(RfeError::NoLibrarySupport)?;
        let rc = {
            let _dev = self.ctl.lock().unwrap_or_else(|e| e.into_inner());
            unsafe { f(self.rfe, std::ffi::c_int::from(mode.code())) }
        };
        let out = self.check(rc);
        self.record("RFE_Mode", mode.label(), &out);
        out
    }

    fn set_fan(&mut self, on: bool) -> RfeResult<()> {
        let f = self.api.rfe_fan.ok_or(RfeError::NoLibrarySupport)?;
        let rc = {
            let _dev = self.ctl.lock().unwrap_or_else(|e| e.into_inner());
            unsafe { f(self.rfe, std::ffi::c_int::from(on)) }
        };
        let out = self.check(rc);
        self.record("RFE_Fan", if on { "on" } else { "off" }, &out);
        out
    }

    fn round_trip(&self) -> Duration {
        BOARD_ROUND_TRIP
    }

    fn describe(&self) -> String {
        self.label.clone()
    }
}

impl Drop for BoardTransport {
    fn drop(&mut self) {
        if let Some(close) = self.api.rfe_close
            && !self.rfe.is_null()
        {
            let _dev = self.ctl.lock().unwrap_or_else(|e| e.into_inner());
            unsafe { close(self.rfe) };
            self.rfe = std::ptr::null_mut();
        }
    }
}
