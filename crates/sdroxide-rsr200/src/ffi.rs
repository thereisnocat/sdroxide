//! Hand-written bindings for FTDI's closed D3XX driver (`libftd3xx` on
//! Linux/macOS, `FTD3XXWU.dll` on Windows), which the RSR200's USB interface
//! needs — the FT601Q "SuperSpeed-FIFO" bridge chip is not a standard
//! USB-class device generic bulk/control transfers can drive. See
//! `RSR200_PLAN.md` section 6 for why this exists at all and what was tried
//! and ruled out first.
//!
//! Loaded with `dlopen`/`LoadLibrary` at runtime (via `libloading`, on every
//! platform), the same as `sdroxide-sdrplay`'s own `ffi.rs`: nothing is
//! linked at build time, so this crate (and everything that depends on it)
//! builds and ships everywhere, and merely finds USB support missing where
//! the driver is not installed.
//!
//! # Windows: a genuinely different D3XX SDK, not just a different file name
//!
//! Windows ships its own D3XX SDK (`FTD3XX.h` / `FTD3XXWU.dll`, from FTDI's
//! *WinUSB* driver package — not the older, deprecated WDF-based one, which
//! has no `FTD3XXWU` import library at all). Bindings below were written
//! against that header directly (extracted into the SDR++ sibling project's
//! own `third_party/ftd3xx_winusb/FTD3XX.h`, not guessed), and against the
//! SDR++ sibling's own `transport_usb.cpp`, whose Windows path is confirmed
//! working on real RSR200 hardware (0.00% packet loss, `test/test_usb_live.cpp`
//! — see that project's `RSR200_PLAN.md` §6/entry 6). Three real ABI
//! differences from the Linux/macOS SDK, not one:
//!
//! 1. **The overlapped read call has a different name**: `FT_ReadPipeEx` on
//!    Windows, `FT_ReadPipeAsync` on Linux/macOS — where, confusingly,
//!    Windows' own `FT_ReadPipeEx` is a *synchronous* call with no async
//!    equivalent under that name on the other platform's SDK (see point 3).
//!    Its Rust-level signature is otherwise identical to the Linux/macOS
//!    call, though: [`Api::read_pipe_async`] is one field, resolved to
//!    whichever symbol the running platform's SDK actually exports.
//! 2. **`FT_WritePipe`'s last parameter has a different type**: an
//!    `LPOVERLAPPED` on Windows (`NULL` for a plain blocking write — the
//!    documented way to get synchronous behavior from an otherwise-async
//!    call) versus a millisecond timeout `DWORD` on Linux/macOS. Genuinely
//!    different width and meaning, not just a different name — cfg-gated
//!    on the [`Api::write_pipe`] field itself, and at its one call site in
//!    [`crate::usb`].
//! 3. **`FT_SetStreamPipe`/`FT_ClearStreamPipe`'s two flag parameters are
//!    `BOOLEAN` (1 byte) on Windows, not the 4-byte `BOOL` this driver's own
//!    Linux/macOS `Types.h` uses everywhere (including for these same two
//!    calls, and for `FT_GetOverlappedResult`'s `bWait`, which stays 4 bytes
//!    — real `BOOL` — on *both* platforms; only the stream-pipe flags are
//!    the narrower `BOOLEAN`).** [`Boolean`]/[`TRUE_B`]/[`FALSE_B`] exist
//!    only for this.
//! 4. **`OVERLAPPED` itself is a different, larger struct** — the real
//!    Win32 one from `<windows.h>` (32 bytes on x86_64: two 8-byte
//!    `ULONG_PTR`s, an 8-byte union collapsed to its `Pointer` member the
//!    same way the Linux/macOS side already does, then an 8-byte `HANDLE`),
//!    not the Linux/macOS SDK's own smaller 24-byte one this file already
//!    modeled ([`Overlapped`] is cfg-gated on `target_os = "windows"` for
//!    exactly this).
//!
//! The FIFO-channel-not-endpoint-address quirk documented below is
//! Linux/macOS-only — Windows' `FT_ReadPipeEx` takes the raw endpoint
//! address directly, confirmed against real hardware in the SDR++ sibling's
//! own comment ("verified there against real hardware: 0.00% packet loss"),
//! so [`crate::usb`] skips [`to_fifo_channel`]'s conversion on Windows.
//!
//! **Not yet verified against real hardware from *this* codebase** — ported
//! carefully from the vendor header and the SDR++ sibling's own
//! already-hardware-verified implementation, and type-checked against the
//! `x86_64-pc-windows-gnu` target, but no Windows machine with the radio
//! attached has run it yet. The runtime DLL search in [`Api::load`]'s
//! Windows path is a best-effort guess at where a real end-user driver
//! install leaves `FTD3XXWU.dll` (the bare name catches the app's own
//! directory, `System32`, and `PATH`; the absolute fallback matches where
//! FTDI's SDK doc and the SDR++ sibling's own `CMakeLists.txt`/packaging
//! script both point when *building against* the SDK) — flag if it turns
//! out an installed driver leaves the DLL somewhere else.
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

/// `FT_HANDLE` — an opaque device handle owned by the driver. `typedef PVOID
/// FT_HANDLE` on every D3XX SDK, Windows included — identical layout.
pub type Handle = *mut c_void;

/// `FT_STATUS` — `typedef ULONG FT_STATUS` on every D3XX SDK, Windows
/// included.
pub type Status = u32;
pub const OK: Status = 0;
pub const IO_PENDING: Status = 24;

pub fn failed(st: Status) -> bool {
    st != OK
}

/// `BOOL` — `typedef unsigned int BOOL` (4 bytes) on this driver's own
/// `Types.h` on Linux/macOS, and on Windows' own `<windows.h>` too (`BOOL`
/// there is `int`, same 4-byte width) — used identically on both platforms
/// for `FT_GetOverlappedResult`'s `bWait` and, on Linux/macOS only, for
/// `FT_SetStreamPipe`/`FT_ClearStreamPipe`'s two flags as well. Not a C
/// `bool`. See [`Boolean`] for the narrower type Windows uses for that
/// second job instead.
pub type Bool = u32;
pub const FALSE: Bool = 0;
pub const TRUE: Bool = 1;

/// `BOOLEAN` — `typedef unsigned char BOOLEAN` (1 byte) on Windows'
/// `<windows.h>`. Only exists as its own type here because Windows' own
/// `FT_SetStreamPipe`/`FT_ClearStreamPipe` take this narrower type for their
/// two flag parameters where the Linux/macOS SDK uses the wider [`Bool`]
/// instead — a real, easy-to-miss width mismatch between the two vendor
/// SDKs for the exact same two calls, not a stylistic difference.
#[cfg(target_os = "windows")]
pub type Boolean = u8;
#[cfg(target_os = "windows")]
pub const TRUE_B: Boolean = 1;
#[cfg(target_os = "windows")]
pub const FALSE_B: Boolean = 0;

pub const OPEN_BY_SERIAL_NUMBER: u32 = 0x0000_0001;
pub const OPEN_BY_INDEX: u32 = 0x0000_0010;

pub const FLAGS_SUPERSPEED: u32 = 4;

/// `_OVERLAPPED` from the Linux/macOS SDK's own `Types.h`. Rust never reads
/// or writes its fields directly — only ever zeroed, passed by pointer to
/// the driver, and eventually released — so the `{Offset,OffsetHigh}`/
/// `Pointer` union the real struct has is represented here by the pointer
/// alone, which matches its size and alignment exactly. **Not the same
/// layout as Windows' own `OVERLAPPED`** — see the `target_os = "windows"`
/// version of this same type just below, and this module's own top-level
/// doc for why they differ (32 bytes vs. 24: two 8-byte `ULONG_PTR`s on
/// Windows where this one has two 4-byte fields).
#[cfg(not(target_os = "windows"))]
#[repr(C)]
pub struct Overlapped {
    pub internal: u32,
    pub internal_high: u32,
    pub pointer: *mut c_void,
    pub h_event: *mut c_void,
}

#[cfg(not(target_os = "windows"))]
impl Default for Overlapped {
    fn default() -> Self {
        Overlapped { internal: 0, internal_high: 0, pointer: std::ptr::null_mut(), h_event: std::ptr::null_mut() }
    }
}

/// The real Win32 `OVERLAPPED` from `<windows.h>`, as every D3XX call on
/// Windows expects — `Internal`/`InternalHigh` are `ULONG_PTR` (8 bytes each
/// on x86_64, not the Linux/macOS SDK's own 4-byte fields of the same name),
/// then the `{Offset,OffsetHigh}`/`Pointer` union (collapsed to the pointer
/// alone, the same simplification the non-Windows version above makes, and
/// for the same reason — never read or written from Rust, only handed to the
/// driver by pointer), then `HANDLE hEvent` (8 bytes). Total 32 bytes on
/// x86_64, checked below.
#[cfg(target_os = "windows")]
#[repr(C)]
pub struct Overlapped {
    pub internal: usize,
    pub internal_high: usize,
    pub pointer: *mut c_void,
    pub h_event: *mut c_void,
}

#[cfg(target_os = "windows")]
impl Default for Overlapped {
    fn default() -> Self {
        Overlapped { internal: 0, internal_high: 0, pointer: std::ptr::null_mut(), h_event: std::ptr::null_mut() }
    }
}

/// `_FT_DEVICE_LIST_INFO_NODE`, one entry per enumerated D3XX device.
/// Identical field order/types on every D3XX SDK, Windows included —
/// confirmed directly against `FTD3XX.h`, not assumed.
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

#[cfg(all(target_pointer_width = "64", not(target_os = "windows")))]
mod layout_asserts {
    use super::*;
    use std::mem::size_of;

    const _: () = assert!(size_of::<Overlapped>() == 24);
    const _: () = assert!(size_of::<DeviceListInfoNode>() == 72);
}

#[cfg(all(target_pointer_width = "64", target_os = "windows"))]
mod layout_asserts {
    use super::*;
    use std::mem::size_of;

    const _: () = assert!(size_of::<Overlapped>() == 32);
    const _: () = assert!(size_of::<DeviceListInfoNode>() == 72);
}

/// The handful of D3XX entry points this crate needs, resolved once at load
/// time. Everything else the driver offers (GPIO, flash, the descriptor
/// queries) is real but unused here.
///
/// Field types are uniform across platforms except the three noted in this
/// module's own top-level doc (`write_pipe`, `set_stream_pipe`,
/// `clear_stream_pipe`), which are declared twice below with `#[cfg]` on
/// each field individually — same field name either way, so
/// [`crate::usb`]'s call sites differ only where the argument *values*
/// (not the field access) need to differ.
pub struct Api {
    pub create_device_info_list: unsafe extern "C" fn(*mut u32) -> Status,
    pub get_device_info_list: unsafe extern "C" fn(*mut DeviceListInfoNode, *mut u32) -> Status,
    pub create: unsafe extern "C" fn(*mut c_void, u32, *mut Handle) -> Status,
    pub close: unsafe extern "C" fn(Handle) -> Status,
    pub set_pipe_timeout: unsafe extern "C" fn(Handle, u8, u32) -> Status,
    #[cfg(not(target_os = "windows"))]
    pub set_stream_pipe: unsafe extern "C" fn(Handle, Bool, Bool, u8, u32) -> Status,
    #[cfg(target_os = "windows")]
    pub set_stream_pipe: unsafe extern "system" fn(Handle, Boolean, Boolean, u8, u32) -> Status,
    #[cfg(not(target_os = "windows"))]
    pub clear_stream_pipe: unsafe extern "C" fn(Handle, Bool, Bool, u8) -> Status,
    #[cfg(target_os = "windows")]
    pub clear_stream_pipe: unsafe extern "system" fn(Handle, Boolean, Boolean, u8) -> Status,
    pub abort_pipe: unsafe extern "C" fn(Handle, u8) -> Status,
    pub initialize_overlapped: unsafe extern "C" fn(Handle, *mut Overlapped) -> Status,
    pub release_overlapped: unsafe extern "C" fn(Handle, *mut Overlapped) -> Status,
    /// The async read — `FT_ReadPipeAsync` on Linux/macOS (channel-indexed,
    /// see [`crate::usb::to_fifo_channel`]), `FT_ReadPipeEx` on Windows
    /// (raw-endpoint-addressed, no conversion needed) — resolved to
    /// whichever symbol the running platform's SDK exports under
    /// [`Api::load`]/[`Api::from_lib`]/[`Api::from_dll`]. Named
    /// `read_pipe_async` here to match the Linux/macOS driver's own name
    /// for it (the one platform where this file's own conversion quirk
    /// applies) even though the Rust-level signature is identical on both
    /// platforms.
    pub read_pipe_async: unsafe extern "C" fn(Handle, u8, *mut u8, u32, *mut u32, *mut Overlapped) -> Status,
    /// A plain blocking write — `FT_WritePipe`'s own signature differs by
    /// platform (see this module's own top-level doc, point 2): a
    /// millisecond timeout on Linux/macOS, an `LPOVERLAPPED` (`NULL` for
    /// synchronous) on Windows. [`crate::usb`]'s one call site is
    /// cfg-gated to pass the right kind of value either way.
    #[cfg(not(target_os = "windows"))]
    pub write_pipe: unsafe extern "C" fn(Handle, u8, *mut u8, u32, *mut u32, u32) -> Status,
    #[cfg(target_os = "windows")]
    pub write_pipe: unsafe extern "system" fn(Handle, u8, *mut u8, u32, *mut u32, *mut Overlapped) -> Status,
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
/// **Unverified against a real driver install** — see this module's own
/// top-level doc. The bare DLL name is tried first, which is what resolves
/// if the driver install (or a bundled copy sitting beside `sdroxide.exe`,
/// the way the SDR++ sibling's own Windows packaging script does it) put
/// `FTD3XXWU.dll` somewhere Windows' own DLL search order already looks
/// (the app's own directory, `System32`, `PATH`). The absolute fallback is
/// where FTDI's SDK doc and the SDR++ sibling's `CMakeLists.txt` both point
/// for *building against* the SDK, on the chance an install leaves the
/// runtime DLL in the same place — not confirmed for a real end-user driver
/// install specifically.
#[cfg(target_os = "windows")]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    ["FTD3XXWU.dll", "C:/Program Files/FTD3XX/FTD3XXWU.dll", "C:/Program Files/FTD3XX/lib/FTD3XXWU.dll"]
        .iter()
        .map(Into::into)
        .collect()
}
#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
fn lib_candidates() -> Vec<std::ffi::OsString> {
    Vec::new()
}

impl Api {
    /// Load the vendor driver, or say why not (used verbatim in the UI).
    pub fn load() -> Result<Api, String> {
        let mut last = String::new();
        for name in lib_candidates() {
            match unsafe { libloading::Library::new(&name) } {
                Ok(lib) => return unsafe { Api::from_lib(lib) },
                Err(e) => last = e.to_string(),
            }
        }
        #[cfg(target_os = "windows")]
        {
            Err(format!(
                "the FTDI D3XX WinUSB driver was not found ({last}) — install the \"WinUSB \
                 D3XX driver\" package from ftdichip.com (not the older WDF-based one), then \
                 rescan; if FTD3XXWU.dll isn't on PATH or beside sdroxide.exe, copy it there"
            ))
        }
        #[cfg(not(target_os = "windows"))]
        {
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

    /// Same idea as [`Api::from_lib`], against `FTD3XXWU.dll`'s own symbol
    /// names — `FT_ReadPipeEx` in place of `FT_ReadPipeAsync` (see this
    /// module's own top-level doc, point 1); everything else shares a name
    /// with the Linux/macOS SDK even where its signature differs.
    #[cfg(target_os = "windows")]
    unsafe fn from_lib(lib: libloading::Library) -> Result<Api, String> {
        macro_rules! sym {
            ($name:literal) => {
                *unsafe { lib.get(concat!($name, "\0").as_bytes()) }
                    .map_err(|e| format!("{} missing from FTD3XXWU.dll: {e}", $name))?
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
            read_pipe_async: sym!("FT_ReadPipeEx"),
            write_pipe: sym!("FT_WritePipe"),
            get_overlapped_result: sym!("FT_GetOverlappedResult"),
            _lib: lib,
        })
    }
}
