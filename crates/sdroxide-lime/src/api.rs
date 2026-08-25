//! The process-global handle on LimeSuite.
//!
//! One dlopen serves every board, and LimeSuite's error reporting is
//! process-global too (`LMS_GetLastErrorMessage` has no device argument), so
//! this module owns the library behind one mutex and reads the error text while
//! still holding it.
//!
//! There is no background service to connect to, unlike the SDRplay backend —
//! but there *is* one-time global setup: LimeSuite writes its own chatter to
//! stderr unless a log handler is installed, which on a terminal UI lands in
//! the middle of whatever is on screen.

use std::ffi::{CStr, CString, c_char, c_int};
use std::sync::{Arc, Mutex, OnceLock};

use sdroxide_types::LimeDevice;

use crate::error::{Error, Result};
use crate::ffi;
use crate::trace::{self, Trace};

struct ApiState {
    api: Option<Arc<ffi::Api>>,
    /// One log line per absence, not one per rescan tick.
    complained: bool,
}

fn state() -> &'static Mutex<ApiState> {
    static STATE: OnceLock<Mutex<ApiState>> = OnceLock::new();
    STATE.get_or_init(|| Mutex::new(ApiState { api: None, complained: false }))
}

/// LimeSuite's log callback.
///
/// `catch_unwind` is not defensive decoration: this is invoked from LimeSuite's
/// own threads, and unwinding across the FFI boundary is undefined behaviour.
/// A panic formatting a log line must not take the process with it.
unsafe extern "C" fn log_handler(level: c_int, message: *const c_char) {
    let _ = std::panic::catch_unwind(|| {
        if message.is_null() {
            return;
        }
        let text = unsafe { CStr::from_ptr(message) }.to_string_lossy();
        let text = text.trim();
        if text.is_empty() {
            return;
        }
        match level {
            ffi::LOG_CRITICAL | ffi::LOG_ERROR => tracing::warn!("LimeSuite: {text}"),
            ffi::LOG_WARNING => tracing::debug!("LimeSuite: {text}"),
            _ => tracing::trace!("LimeSuite: {text}"),
        }
    });
}

/// Load the library, idempotent.
fn ensure_loaded(s: &mut ApiState) -> Result<Arc<ffi::Api>> {
    if let Some(a) = &s.api {
        return Ok(Arc::clone(a));
    }
    match ffi::Api::load() {
        Ok(a) => {
            let a = Arc::new(a);
            // Take LimeSuite's chatter off stderr before anything can produce
            // any.
            unsafe { (a.register_log_handler)(Some(log_handler)) };
            let version = a.version();
            tracing::info!("LimeSuite loaded, version {version}");
            if !a.has_rfe() {
                tracing::info!(
                    "this LimeSuite has no LimeRFE support; a LimeRFE on its own USB port still \
                     works"
                );
            }
            s.api = Some(Arc::clone(&a));
            s.complained = false;
            Ok(a)
        }
        Err(e) => {
            if !s.complained {
                s.complained = true;
                tracing::debug!("LimeSDR backend unavailable: {e}");
            }
            Err(Error::LibMissing(e))
        }
    }
}

/// The loaded library, for callers that need to reach it directly.
pub(crate) fn api() -> Result<Arc<ffi::Api>> {
    let mut s = state().lock().expect("lime api state poisoned");
    ensure_loaded(&mut s)
}

/// Everything LimeSuite enumerated, including entries this backend will not
/// open. The second list is what `--probe` prints so a device that was filtered
/// out is explicable rather than simply missing.
pub struct Enumeration {
    pub devices: Vec<LimeDevice>,
    pub rejected: Vec<String>,
}

/// Ask LimeSuite what is attached, or say why that is impossible — the
/// distinction `--probe` needs between "no library" and "no devices".
pub fn try_list() -> Result<Enumeration> {
    // Its own trace, in its own slot: a scan that finds nothing is the report
    // for "sdroxide does not see my board", and it must not overwrite the
    // record of a session that did open one.
    let t = Trace::new();
    trace::remember_probe(&t);
    let mut s = state().lock().expect("lime api state poisoned");
    let api = match ensure_loaded(&mut s) {
        Ok(a) => a,
        Err(e) => {
            t.call("dlopen", "LimeSuite", format!("FAILED: {e}"));
            return Err(e);
        }
    };
    t.set_identity(format!(
        "LimeSuite {}{}",
        api.version(),
        if api.has_rfe() { "" } else { " (no LimeRFE support in this build)" }
    ));

    // Two-call: a null pointer asks only for the count.
    let n = unsafe { (api.get_device_list)(std::ptr::null_mut()) };
    if n < 0 {
        let text = api.err_text();
        t.call("LMS_GetDeviceList", "count", format!("FAILED: {text}"));
        return Err(Error::api("LMS_GetDeviceList", text));
    }
    if n == 0 {
        t.call("LMS_GetDeviceList", "count", "0 devices");
        return Ok(Enumeration { devices: Vec::new(), rejected: Vec::new() });
    }
    let mut buf = vec![[0 as c_char; ffi::INFO_STR_LEN]; n as usize];
    let n = unsafe { (api.get_device_list)(buf.as_mut_ptr()) };
    if n < 0 {
        let text = api.err_text();
        t.call("LMS_GetDeviceList", "entries", format!("FAILED: {text}"));
        return Err(Error::api("LMS_GetDeviceList", text));
    }

    let mut devices = Vec::new();
    let mut rejected = Vec::new();
    for entry in buf.iter().take(n as usize) {
        let info = ffi::c_field(entry);
        if info.is_empty() {
            continue;
        }
        let dev = LimeDevice::parse(&info);
        // The allow-list, and the reason for it: LimeSuite claims the bare
        // Cypress FX3 id that an unprogrammed RX-888 also presents. Offering
        // that as a Lime board would hand back a receiver that hears nothing
        // and floods the log with transfer errors on the way.
        if LimeDevice::name_is_known(&dev.name) {
            t.call("LMS_GetDeviceList", &info, "a Lime board");
            devices.push(dev);
        } else {
            t.call("LMS_GetDeviceList", &info, "not a Lime board, ignored");
            rejected.push(info);
        }
    }
    if !rejected.is_empty() {
        tracing::debug!(
            "LimeSuite listed {} device(s) that are not Lime boards, ignoring them: {}",
            rejected.len(),
            rejected.join("; ")
        );
    }
    Ok(Enumeration { devices, rejected })
}

/// The boards LimeSuite reports. Best-effort: no library and no devices are the
/// same answer to a Rescan button.
pub fn list() -> Vec<LimeDevice> {
    match try_list() {
        Ok(e) => e.devices,
        Err(e) => {
            tracing::debug!("LimeSDR enumeration failed: {e}");
            Vec::new()
        }
    }
}

/// Open the board `want` names — a serial suffix, a whole device string, or
/// empty for the first one found.
pub(crate) fn open(want: &str) -> Result<(Arc<ffi::Api>, ffi::Device, LimeDevice)> {
    let found = try_list()?;
    let chosen = found.devices.iter().find(|d| d.matches(want)).cloned().ok_or_else(|| {
        let want = want.trim();
        if found.devices.is_empty() {
            let mut msg = "no LimeSDR found — is one plugged in, and does LimeUtil --find see \
                           it?"
            .to_string();
            if !found.rejected.is_empty() {
                msg.push_str(&format!(
                    " (LimeSuite listed {} non-Lime device(s), which were ignored)",
                    found.rejected.len()
                ));
            }
            Error::NotFound(msg)
        } else {
            Error::NotFound(format!(
                "no LimeSDR matching {want:?} — the boards found are: {}",
                found.devices.iter().map(|d| d.label()).collect::<Vec<_>>().join(", ")
            ))
        }
    })?;

    let api = api()?;
    let info = CString::new(chosen.info.as_str())
        .map_err(|_| Error::NotFound("the device string contains a NUL byte".into()))?;
    let mut dev: ffi::Device = std::ptr::null_mut();
    let rc = unsafe { (api.open)(&mut dev, info.as_ptr(), std::ptr::null_mut()) };
    if rc != ffi::OK || dev.is_null() {
        let text = api.err_text();
        // LimeSuite does not distinguish "busy" in its return code, and a board
        // the enumeration just listed nearly always failed to open because
        // something else has it.
        let lowered = text.to_ascii_lowercase();
        if lowered.contains("busy") || lowered.contains("in use") || lowered.contains("access") {
            return Err(Error::InUse(format!("{} — {text}", chosen.label())));
        }
        return Err(Error::api("LMS_Open", format!("{}: {text}", chosen.label())));
    }
    Ok((api, dev, chosen))
}
