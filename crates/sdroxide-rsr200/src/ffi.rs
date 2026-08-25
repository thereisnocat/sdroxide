//! Hand-written bindings for FTDI's closed D3XX driver (`libftd3xx`), which
//! the RSR200's USB interface needs — the FT601Q "SuperSpeed-FIFO" bridge
//! chip is not a standard USB-class device generic bulk/control transfers
//! can drive. See `RSR200_PLAN.md` section 6 for why this exists at all and
//! what was tried and ruled out first.
//!
//! Loaded with `dlopen` at runtime, the same as `sdroxide-sdrplay`'s own
//! `ffi.rs`: nothing is linked at build time, so this crate (and everything
//! that depends on it) builds and ships everywhere, and merely finds USB
//! support missing where the driver is not installed.
//!
//! Linux/macOS only for now. Windows ships a genuinely different D3XX SDK —
//! different async-call names, and `FT_ReadPipeEx` there is the *overlapped*
//! call rather than a synchronous one, an inversion from this file's own
//! `FT_ReadPipeEx`/`FT_ReadPipeAsync` split — and the plan's own section 6
//! leaves open whether a WinUSB-based `nusb` path might avoid the vendor SDK
//! there entirely. That is its own research spike, not assumed here:
//! [`Api::load`] fails cleanly with an explanatory message on Windows rather
//! than guess at an unverified binding.
//!
//! # A second, undocumented split beyond Windows vs. Linux/macOS
//!
//! On the Linux/macOS SDK, `FT_ReadPipeAsync` (this file's [`Api::read_pipe_async`])
//! takes a logical FIFO *channel* index (0–3), not the raw USB endpoint
//! address every other pipe call in this file takes — confirmed against real
//! hardware in the SDR++ port this is drawn from, not assumed:
//! `FT_ReadPipeAsync(h, 0x82, ...)` returns `FT_INVALID_PARAMETER` every
//! time; `FT_ReadPipeAsync(h, 0, ...)` returns `FT_IO_PENDING` and completes
//! normally. The FT600/FT601's bulk IN endpoints are 0x82/0x83/0x84/0x85 for
//! channels 0–3, so the conversion is the low nibble minus 2 — see
//! [`crate::usb`], which is the only place that needs to know.

#![allow(dead_code)]

use std::ffi::c_void;

/// `FT_HANDLE` — an opaque device handle owned by the driver.
pub type Handle = *mut c_void;

/// `FT_STATUS` — `typedef ULONG FT_STATUS`.
pub type Status = u32;
pub const OK: Status = 0;
pub const IO_PENDING: Status = 24;

pub fn failed(st: Status) -> bool {
    st != OK
}

/// `BOOL` — `typedef unsigned int BOOL` on this driver's own `Types.h`, not
/// a C `bool`.
pub type Bool = u32;
pub const FALSE: Bool = 0;
pub const TRUE: Bool = 1;

pub const OPEN_BY_SERIAL_NUMBER: u32 = 0x0000_0001;
pub const OPEN_BY_INDEX: u32 = 0x0000_0010;

pub const FLAGS_SUPERSPEED: u32 = 4;

/// `_OVERLAPPED` from the driver's own `Types.h`. Rust never reads or writes
/// its fields directly — only ever zeroed, passed by pointer to the driver,
/// and eventually released — so the `{Offset,OffsetHigh}`/`Pointer` union
/// the real struct has is represented here by the pointer alone, which
/// matches its size and alignment exactly.
#[repr(C)]
pub struct Overlapped {
    pub internal: u32,
    pub internal_high: u32,
    pub pointer: *mut c_void,
    pub h_event: *mut c_void,
}

impl Default for Overlapped {
    fn default() -> Self {
        Overlapped { internal: 0, internal_high: 0, pointer: std::ptr::null_mut(), h_event: std::ptr::null_mut() }
    }
}

/// `_FT_DEVICE_LIST_INFO_NODE`, one entry per enumerated D3XX device.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct DeviceListInfoNode {
    pub flags: u32,
    pub device_type: u32,
    pub id: u32,
    pub loc_id: u32,
    pub serial_number: [u8; 16],
    pub description: [u8; 32],
    pub ft_handle: Handle,
}

impl Default for DeviceListInfoNode {
    fn default() -> Self {
        DeviceListInfoNode {
            flags: 0,
            device_type: 0,
            id: 0,
            loc_id: 0,
            serial_number: [0; 16],
            description: [0; 32],
            ft_handle: std::ptr::null_mut(),
        }
    }
}

/// A `char[N]` field that may or may not be NUL-terminated within its own
/// width — the header makes no promise either way — trimmed at the first
/// NUL if there is one, and at trailing NULs otherwise.
pub fn field_to_string(raw: &[u8]) -> String {
    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
    String::from_utf8_lossy(&raw[..end]).into_owned()
}

#[cfg(target_pointer_width = "64")]
mod layout_asserts {
    use super::*;
    use std::mem::size_of;

    const _: () = assert!(size_of::<Overlapped>() == 24);
    const _: () = assert!(size_of::<DeviceListInfoNode>() == 72);
}

/// The handful of D3XX entry points this crate needs, resolved once at load
/// time. Everything else the driver offers (GPIO, flash, the descriptor
/// queries) is real but unused here.
pub struct Api {
    pub create_device_info_list: unsafe extern "C" fn(*mut u32) -> Status,
    pub get_device_info_list: unsafe extern "C" fn(*mut DeviceListInfoNode, *mut u32) -> Status,
    pub create: unsafe extern "C" fn(*mut c_void, u32, *mut Handle) -> Status,
    pub close: unsafe extern "C" fn(Handle) -> Status,
    pub set_pipe_timeout: unsafe extern "C" fn(Handle, u8, u32) -> Status,
    pub set_stream_pipe: unsafe extern "C" fn(Handle, Bool, Bool, u8, u32) -> Status,
    pub clear_stream_pipe: unsafe extern "C" fn(Handle, Bool, Bool, u8) -> Status,
    pub abort_pipe: unsafe extern "C" fn(Handle, u8) -> Status,
    pub initialize_overlapped: unsafe extern "C" fn(Handle, *mut Overlapped) -> Status,
    pub release_overlapped: unsafe extern "C" fn(Handle, *mut Overlapped) -> Status,
    /// The Linux/macOS async read — see this module's own doc for the
    /// FIFO-channel-not-endpoint quirk. Named `read_pipe_async` here to
    /// match the driver's own name for it, even though `usb.rs` is the only
    /// caller and always passes a channel already converted from an
    /// endpoint address.
    pub read_pipe_async: unsafe extern "C" fn(Handle, u8, *mut u8, u32, *mut u32, *mut Overlapped) -> Status,
    /// A plain blocking write with a millisecond timeout — this driver's own
    /// `FT_WritePipe` signature on Linux/macOS, unlike Windows' overlapped
    /// one. Commands are rare and small, so nothing here needs the
    /// overlapped machinery the read side does for throughput.
    pub write_pipe: unsafe extern "C" fn(Handle, u8, *mut u8, u32, *mut u32, u32) -> Status,
    pub get_overlapped_result: unsafe extern "C" fn(Handle, *mut Overlapped, *mut u32, Bool) -> Status,
    _lib: libloading::Library,
}

/// Library names/paths to try, most specific first. The absolute path is
/// the one actually installed and verified on Ralph's Mac (see
/// `RSR200_PLAN.md` section 6 and the `sdrpp-antenna-phasing` history this
/// was ported from) — after `install_name_tool -id` durably fixed the
/// dylib's own install name to this exact path, so every future load of it,
/// this crate included, resolves the same way. The bare name is kept first
/// in case a future install puts it somewhere the dynamic linker already
/// searches.
#[cfg(target_os = "macos")]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    ["libftd3xx.dylib", "/usr/local/lib/libftd3xx.dylib"].iter().map(Into::into).collect()
}
#[cfg(target_os = "linux")]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    ["libftd3xx.so", "/usr/local/lib/libftd3xx.so"].iter().map(Into::into).collect()
}
#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    Vec::new()
}

impl Api {
    /// Load the vendor driver, or say why not (used verbatim in the UI).
    /// Windows always fails here — see this module's own doc — rather than
    /// load an unverified binding.
    pub fn load() -> Result<Api, String> {
        #[cfg(target_os = "windows")]
        {
            return Err(
                "USB support for the RSR200 needs its own Windows research spike (see \
                 RSR200_PLAN.md section 6) and is not implemented on this platform yet — use \
                 the LAN transport instead."
                    .to_string(),
            );
        }
        #[cfg(not(target_os = "windows"))]
        {
            let mut last = String::new();
            for name in lib_candidates() {
                match unsafe { libloading::Library::new(&name) } {
                    Ok(lib) => return unsafe { Api::from_lib(lib) },
                    Err(e) => last = e.to_string(),
                }
            }
            Err(format!(
                "the FTDI D3XX driver was not found ({last}) — install it from ftdichip.com \
                 (libftd3xx / FTD3XXWU), then rescan"
            ))
        }
    }

    #[cfg(not(target_os = "windows"))]
    unsafe fn from_lib(lib: libloading::Library) -> Result<Api, String> {
        // Resolve every symbol up front: a library that is missing one is
        // the wrong library (or too old a version), and finding out now
        // beats finding out mid-stream.
        macro_rules! sym {
            ($name:literal) => {
                *unsafe { lib.get(concat!($name, "\0").as_bytes()) }
                    .map_err(|e| format!("{} missing from the D3XX library: {e}", $name))?
            };
        }
        Ok(Api {
            create_device_info_list: sym!("FT_CreateDeviceInfoList"),
            get_device_info_list: sym!("FT_GetDeviceInfoList"),
            create: sym!("FT_Create"),
            close: sym!("FT_Close"),
            set_pipe_timeout: sym!("FT_SetPipeTimeout"),
            set_stream_pipe: sym!("FT_SetStreamPipe"),
            clear_stream_pipe: sym!("FT_ClearStreamPipe"),
            abort_pipe: sym!("FT_AbortPipe"),
            initialize_overlapped: sym!("FT_InitializeOverlapped"),
            release_overlapped: sym!("FT_ReleaseOverlapped"),
            read_pipe_async: sym!("FT_ReadPipeAsync"),
            write_pipe: sym!("FT_WritePipe"),
            get_overlapped_result: sym!("FT_GetOverlappedResult"),
            _lib: lib,
        })
    }
}
