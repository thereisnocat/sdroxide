//! USB transport for the RSR200's FT601Q SuperSpeed-FIFO bridge, over
//! FTDI's D3XX driver ([`crate::ffi`]). See `RSR200_PLAN.md` section 6 for
//! why this needs a vendor driver rather than `nusb` the way every sibling
//! USB backend in this workspace does, and the `sdrpp-antenna-phasing`
//! project memory for the three real macOS bugs this port carries forward
//! fixes for (a bare `dlopen` install name, an invalid code signature after
//! `install_name_tool`, and the FIFO-channel-not-endpoint-address quirk —
//! the last one lives in [`crate::ffi`]'s own doc, the first two are
//! operator-side driver install steps, not anything this file can fix).
//!
//! A direct, line-by-line port of the already-hardware-verified
//! `transport_usb.h`/`.cpp` from the SDR++ sibling implementation this
//! whole crate is drawn from — including its queue-depth/chunk-size
//! constants, which were arrived at empirically against the real radio (see
//! the comment on [`QUEUE_DEPTH`]), not re-derived here.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::device::{Transport, TransportKind};
use crate::ffi::{self, Api};
use crate::protocol::{USB_ENDPOINT_IN, USB_ENDPOINT_OUT, USB_PACKET_BYTES};

/// Each queued read asks for this many radio packets at once rather than
/// one, so `QUEUE_DEPTH` chunk-sized calls are in flight rather than
/// `QUEUE_DEPTH` packet-sized ones. Ported as-is from `transport_usb.h`:
/// raising queue depth alone (8 → 64 → 256) plateaued well under the
/// FT601's SuperSpeed ceiling on the real radio — throughput capped at the
/// same ~11000 reads/sec regardless of buffer count, which points at fixed
/// per-call overhead rather than a buffering shortfall. Batching packets
/// into fewer, larger calls is the standard answer to that, and is what
/// FTDI's own D3XX SDK sample (`WU_DataStreamerApp`) does for the same
/// reason.
const QUEUE_DEPTH: usize = 16;
const PACKETS_PER_READ: usize = 8;
const CHUNK_BYTES: usize = USB_PACKET_BYTES * PACKETS_PER_READ;

/// The FIFO-channel-not-endpoint-address conversion — see [`crate::ffi`]'s
/// own doc for why this exists and how it was confirmed. Scoped to the one
/// call that needs it, matching the C++ original's own `toFifoChannel`.
fn to_fifo_channel(endpoint_address: u8) -> u8 {
    (endpoint_address & 0x0F) - 2
}

/// One connected D3XX device, for a settings-tab device list.
pub struct UsbDeviceInfo {
    pub description: String,
    pub serial: String,
    pub superspeed: bool,
}

/// Loads the driver and lists every connected D3XX device. The empty list
/// on a load failure carries no reason on its own — callers that want to
/// explain "no driver" versus "driver found, nothing attached" should call
/// [`Api::load`] themselves first, the way [`UsbTransport::open`] does.
pub fn list_devices() -> Result<Vec<UsbDeviceInfo>, String> {
    let api = Api::load()?;
    Ok(unsafe { list_devices_with(&api) })
}

/// # Safety
/// `api` must be a successfully loaded [`Api`].
unsafe fn list_devices_with(api: &Api) -> Vec<UsbDeviceInfo> {
    let mut count: u32 = 0;
    if ffi::failed(unsafe { (api.create_device_info_list)(&mut count) }) || count == 0 {
        return Vec::new();
    }
    let mut nodes = vec![ffi::DeviceListInfoNode::default(); count as usize];
    if ffi::failed(unsafe { (api.get_device_info_list)(nodes.as_mut_ptr(), &mut count) }) {
        return Vec::new();
    }
    nodes[..count as usize]
        .iter()
        .map(|n| UsbDeviceInfo {
            description: ffi::field_to_string(&n.description),
            serial: ffi::field_to_string(&n.serial_number),
            superspeed: (n.flags & ffi::FLAGS_SUPERSPEED) != 0,
        })
        .collect()
}

/// Interrupts a pending read without touching the overlapped/buffer pool —
/// call this from another thread before [`UsbTransport::close`] runs on the
/// thread that owns the transport, so its blocked [`Api::get_overlapped_result`]
/// call returns and that thread stops touching the pool before close
/// releases it. Ported from `UsbTransport::abortReads` for API parity with
/// [`crate::lan::LanTcpTransport::stop_handle`]; not yet wired into
/// `stream.rs`'s own shutdown path, matching that same LAN method's current
/// state — see its own doc.
#[derive(Clone)]
pub struct UsbStopHandle {
    api: Arc<Api>,
    handle: SendableHandle,
    stopped: Arc<AtomicBool>,
}

impl UsbStopHandle {
    pub fn stop(&self) {
        self.stopped.store(true, Ordering::Relaxed);
        if !self.handle.0.is_null() {
            unsafe { (self.api.abort_pipe)(self.handle.0, USB_ENDPOINT_IN) };
        }
    }
}

/// `FT_HANDLE` is `*mut c_void`, not `Send` by default. FTDI's own D3XX
/// documentation for `FT_AbortPipe` describes exactly this pattern — calling
/// it from a thread other than the one blocked in a read on the same handle
/// — as the intended way to interrupt a pending read, so sharing the handle
/// across threads for that one call is the API's own documented contract,
/// not an assumption made here.
#[derive(Clone, Copy)]
struct SendableHandle(ffi::Handle);
unsafe impl Send for SendableHandle {}
unsafe impl Sync for SendableHandle {}

pub struct UsbTransport {
    api: Arc<Api>,
    handle: ffi::Handle,
    buffers: Vec<Vec<u8>>,
    overlapped: Vec<ffi::Overlapped>,
    cursor: usize,
    drain_slot: usize,
    packet_in_chunk: usize,
    packets_in_current_chunk: usize,
    have_chunk: bool,
    stream_pipe_set: bool,
    stopped: Arc<AtomicBool>,
    last_error: String,
}

impl UsbTransport {
    /// Opens the device whose serial matches `serial` exactly, or — when
    /// `serial` is empty — the first D3XX device enumerated, matching every
    /// other USB backend's own "empty means first found" convention (e.g.
    /// [`sdroxide_types::HydraSdrConfig::serial`]). Starts the queued read
    /// pipe before returning, matching DP 2.1's recommendation to keep reads
    /// outstanding at all times — so a successful `open` is already
    /// streaming.
    pub fn open(serial: &str) -> Result<UsbTransport, String> {
        let api = Arc::new(Api::load()?);

        let mut count: u32 = 0;
        if ffi::failed(unsafe { (api.create_device_info_list)(&mut count) }) {
            return Err("FT_CreateDeviceInfoList failed".to_string());
        }
        if count == 0 {
            return Err("no D3XX device found".to_string());
        }

        let handle = if serial.is_empty() {
            let mut h: ffi::Handle = std::ptr::null_mut();
            // FT_OPEN_BY_INDEX packs the index into the pointer-sized
            // argument directly rather than pointing at it — index 0 here
            // ("first found"), which happens to share null's bit pattern,
            // not a null-pointer dereference.
            let st = unsafe { (api.create)(std::ptr::null_mut(), ffi::OPEN_BY_INDEX, &mut h) };
            if ffi::failed(st) || h.is_null() {
                return Err(format!("FT_Create failed (status {st})"));
            }
            h
        } else {
            // FT_Create's PVOID argument is read, not retained, so the
            // CString only needs to outlive this one call.
            let cserial = std::ffi::CString::new(serial)
                .map_err(|_| "the USB serial contains an embedded NUL".to_string())?;
            let mut h: ffi::Handle = std::ptr::null_mut();
            let st = unsafe {
                (api.create)(cserial.as_ptr() as *mut std::ffi::c_void, ffi::OPEN_BY_SERIAL_NUMBER, &mut h)
            };
            if ffi::failed(st) || h.is_null() {
                return Err(format!("no D3XX device with serial \"{serial}\" (status {st})"));
            }
            h
        };

        let mut t = UsbTransport {
            api,
            handle,
            buffers: Vec::new(),
            overlapped: Vec::new(),
            cursor: 0,
            drain_slot: 0,
            packet_in_chunk: 0,
            packets_in_current_chunk: 0,
            have_chunk: false,
            stream_pipe_set: false,
            stopped: Arc::new(AtomicBool::new(false)),
            last_error: String::new(),
        };
        if let Err(e) = t.start_streaming() {
            t.close();
            return Err(e);
        }
        Ok(t)
    }

    pub fn stop_handle(&self) -> UsbStopHandle {
        UsbStopHandle {
            api: Arc::clone(&self.api),
            handle: SendableHandle(self.handle),
            stopped: Arc::clone(&self.stopped),
        }
    }

    fn start_streaming(&mut self) -> Result<(), String> {
        // DP 2.1: no host-side read timeout. The radio paces the stream on
        // its own; a timeout here would only produce spurious short reads
        // while nothing is wrong.
        unsafe { (self.api.set_pipe_timeout)(self.handle, USB_ENDPOINT_IN, 0) };

        let st = unsafe {
            (self.api.set_stream_pipe)(self.handle, ffi::FALSE, ffi::FALSE, USB_ENDPOINT_IN, CHUNK_BYTES as u32)
        };
        if ffi::failed(st) {
            return Err(format!("FT_SetStreamPipe failed (status {st})"));
        }
        self.stream_pipe_set = true;

        self.buffers = (0..QUEUE_DEPTH).map(|_| vec![0u8; CHUNK_BYTES]).collect();
        self.overlapped = (0..QUEUE_DEPTH).map(|_| ffi::Overlapped::default()).collect();
        for ov in &mut self.overlapped {
            let st = unsafe { (self.api.initialize_overlapped)(self.handle, ov) };
            if ffi::failed(st) {
                return Err(format!("FT_InitializeOverlapped failed (status {st})"));
            }
        }
        for slot in 0..QUEUE_DEPTH {
            self.queue_read(slot)?;
        }
        Ok(())
    }

    fn queue_read(&mut self, slot: usize) -> Result<(), String> {
        let mut got: u32 = 0;
        let st = unsafe {
            (self.api.read_pipe_async)(
                self.handle,
                to_fifo_channel(USB_ENDPOINT_IN),
                self.buffers[slot].as_mut_ptr(),
                CHUNK_BYTES as u32,
                &mut got,
                &mut self.overlapped[slot],
            )
        };
        // Overlapped reads report their real completion through
        // FT_GetOverlappedResult; FT_IO_PENDING here is success, not an
        // error.
        if st != ffi::IO_PENDING && ffi::failed(st) {
            return Err(format!("queued read failed (status {st})"));
        }
        Ok(())
    }

    pub fn close(&mut self) {
        if self.handle.is_null() {
            return;
        }
        if self.stream_pipe_set {
            unsafe { (self.api.abort_pipe)(self.handle, USB_ENDPOINT_IN) };
            // Wait for each queued read's own cancellation/completion
            // before releasing its OVERLAPPED. The C++ reference this was
            // ported from releases immediately after FT_AbortPipe with no
            // drain, matching FTDI's own sample code — but that exact
            // sequence produced a real, reproducible segfault against this
            // radio (2026-08-24): FT_AbortPipe returns before every queued
            // read has actually finished cancelling on this driver, despite
            // its synchronous-looking signature, so FT_ReleaseOverlapped
            // could race a read that was still in flight. Draining first
            // fixed it; discovered by testing against real hardware, not
            // documented anywhere in the D3XX headers.
            for ov in &mut self.overlapped {
                let mut got: u32 = 0;
                unsafe { (self.api.get_overlapped_result)(self.handle, ov, &mut got, ffi::TRUE) };
                unsafe { (self.api.release_overlapped)(self.handle, ov) };
            }
            unsafe { (self.api.clear_stream_pipe)(self.handle, ffi::FALSE, ffi::FALSE, USB_ENDPOINT_IN) };
        }
        unsafe { (self.api.close)(self.handle) };
        self.handle = std::ptr::null_mut();
        self.stream_pipe_set = false;
        self.buffers.clear();
        self.overlapped.clear();
        self.cursor = 0;
        self.drain_slot = 0;
        self.packet_in_chunk = 0;
        self.packets_in_current_chunk = 0;
        self.have_chunk = false;
    }
}

impl Drop for UsbTransport {
    fn drop(&mut self) {
        self.close();
    }
}

impl Transport for UsbTransport {
    fn kind(&self) -> TransportKind {
        TransportKind::Usb
    }

    fn send_command(&mut self, data: &[u8]) -> bool {
        if self.handle.is_null() {
            return false;
        }
        let mut written: u32 = 0;
        // A plain blocking write with a 1-second timeout. Commands are rare
        // (config changes, acks) and small (8/12/16 bytes per DP 4), so
        // there is no need for the overlapped machinery the read side needs
        // for throughput.
        const WRITE_TIMEOUT_MS: u32 = 1000;
        let mut buf = data.to_vec();
        let st = unsafe {
            (self.api.write_pipe)(
                self.handle,
                USB_ENDPOINT_OUT,
                buf.as_mut_ptr(),
                buf.len() as u32,
                &mut written,
                WRITE_TIMEOUT_MS,
            )
        };
        if ffi::failed(st) {
            self.last_error = format!("USB write failed (status {st})");
            return false;
        }
        written as usize == data.len()
    }

    /// Drains completed reads one packet at a time before waiting on the
    /// next chunk, so the fixed per-call overhead is paid once per chunk
    /// rather than once per packet. False means the device is gone or the
    /// read failed — DP 3.3's warning that `Stop stream` closes the USB
    /// endpoint entirely shows up here as exactly this, matching
    /// [`crate::lan::LanTcpTransport::next_frame`]'s own "stopped or failed
    /// look the same" contract.
    fn next_frame(&mut self, out: &mut Vec<u8>) -> bool {
        if self.handle.is_null() {
            return false;
        }

        while !self.have_chunk {
            if self.stopped.load(Ordering::Relaxed) {
                return false;
            }
            let mut got: u32 = 0;
            let st = unsafe {
                (self.api.get_overlapped_result)(self.handle, &mut self.overlapped[self.cursor], &mut got, ffi::TRUE)
            };
            if ffi::failed(st) {
                self.last_error = format!("USB read failed (status {st})");
                return false;
            }

            self.drain_slot = self.cursor;
            self.cursor = (self.cursor + 1) % QUEUE_DEPTH;
            self.packets_in_current_chunk = got as usize / USB_PACKET_BYTES;
            self.packet_in_chunk = 0;

            if self.packets_in_current_chunk == 0 {
                // A short/empty completion — seen occasionally around Start
                // Stream on the real radio. Nothing to hand out; requeue
                // this buffer and wait on the next one.
                if let Err(e) = self.queue_read(self.drain_slot) {
                    self.last_error = e;
                    return false;
                }
                continue;
            }
            self.have_chunk = true;
        }

        let start = self.packet_in_chunk * USB_PACKET_BYTES;
        out.clear();
        out.extend_from_slice(&self.buffers[self.drain_slot][start..start + USB_PACKET_BYTES]);
        self.packet_in_chunk += 1;

        if self.packet_in_chunk >= self.packets_in_current_chunk {
            if let Err(e) = self.queue_read(self.drain_slot) {
                self.last_error = e;
                return false;
            }
            self.have_chunk = false;
        }

        true
    }

    fn last_error(&self) -> Option<&str> {
        if self.last_error.is_empty() { None } else { Some(&self.last_error) }
    }
}
