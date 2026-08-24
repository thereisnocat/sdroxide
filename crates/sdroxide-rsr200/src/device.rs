//! Transport-agnostic device layer for the RSR200. See `RSR200_PLAN.md`.
//!
//! Everything that is true of the radio regardless of how it is connected
//! lives here: configuration ordering, command numbering, the
//! acknowledgement and retry rules, frame parsing, and sample delivery. A
//! transport supplies whole frames and carries command bytes; it knows
//! nothing about what they mean.
//!
//! Ported from the already-tested `rsr200_device.h`. Written against a fake
//! transport (see this module's own tests) so the sequencing can be tested
//! before either the radio or a socket exists — matching the C++ original's
//! own stated reason for existing.
//!
//! Two structural adaptations from the C++ original:
//!
//! * That version stores a non-owning `Transport*` set once via
//!   `setTransport()`. Rust's ownership rules make a *stored* trait object
//!   awkward for exactly the case that matters here — a caller (a test, or
//!   later `stream.rs`) needing to keep its own handle to the transport for
//!   inspection or re-feeding frames while `Device` is also using it. So
//!   `Device` here holds no transport at all; every method that needs one
//!   takes `&mut dyn Transport` as an explicit parameter instead. No
//!   behaviour changes — only who holds the reference.
//! * That version delivers samples and replies through `std::function`
//!   callbacks (`onSamples`/`onReply`/`onError`). This one has
//!   [`Device::pump`] return what happened directly — a `SampleBlock`'s
//!   worth of metadata, the replies embedded in the frame, and an error
//!   string if the frame was malformed — into caller-supplied output
//!   buffers, for the same reason: a callback stored alongside the buffers
//!   it would need to read is a self-referential-borrow problem in Rust
//!   that the C++ version's raw pointers never faced.

use crate::error::{Error, Result};
use crate::protocol::{
    AUTO_ATT_GAIN, BlockLayout, Complex64, GenSelect, HardwareWeight, Interface, OpMode, Reply,
    Status, StreamFormat, StreamPort, Tuning, Variable, auto_att_gain_lsb,
    auto_att_hold_time_clocks, block_trailer_valid, cmd_set_adc_clock, cmd_set_auto_attenuator,
    cmd_set_data_transmission, cmd_set_generator, cmd_set_lo_both, cmd_set_variable,
    cmd_start_stream, cmd_stop_stream, dsp_mode_byte, hardware_weight_for, lan_layout,
    pack_magnitude_phase, parse_embedded_command, parse_status, port_mode_byte, read_u32,
    tune_for, unpack, usb_samples_per_packet, write_u32,
};

// ---------------------------------------------------------------------------
// Transport
//
// Framing belongs to the transport because framing *is* the difference
// between the interfaces: USB reads whole 4096-byte packets from an
// endpoint, TCP has to resync a byte stream against the block trailer, and
// UDP has to reassemble indexed fragments. Above this line those are all
// just "a frame arrived".
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportKind {
    Usb,
    LanTcp,
    LanUdp,
}

pub trait Transport {
    fn kind(&self) -> TransportKind;

    /// Command encoding is shortened on LAN and fixed-length on USB.
    fn is_lan(&self) -> bool {
        self.kind() != TransportKind::Usb
    }

    /// The stream port to name in Start/Stop stream.
    fn stream_port(&self) -> StreamPort {
        match self.kind() {
            TransportKind::Usb => StreamPort::Usb,
            TransportKind::LanUdp => StreamPort::Udp,
            TransportKind::LanTcp => StreamPort::Tcp,
        }
    }

    fn send_command(&mut self, data: &[u8]) -> bool;

    /// Blocks until a whole frame is available. False means stopped or
    /// failed — see [`Self::last_error`] to tell them apart.
    fn next_frame(&mut self, out: &mut Vec<u8>) -> bool;

    /// LAN transports need the block geometry to frame at all; USB ignores
    /// it.
    fn set_layout(&mut self, _layout: BlockLayout) {}

    /// Reads exactly one standalone, fixed-size reply packet, blocking with
    /// whatever timeout the transport itself is configured with. Only ever
    /// used for one specific case: the "Read version numbers" query, when
    /// sent before streaming has started on LAN — that command's reply (DP
    /// section 3.2, "Report version numbers LAN", 12 bytes) genuinely
    /// arrives as its own standalone packet on real hardware, confirmed by
    /// packet capture. Nothing else does: every configuration command's own
    /// confirmation only ever shows up once streaming has actually started,
    /// embedded in a block — [`Device::send`]'s own ordinary embedded-reply
    /// path handles everything except the version query, which callers use
    /// this directly for instead. USB has no equivalent concept at all — it
    /// starts streaming continuously right at power-up — so the default
    /// implementation always fails, and nothing should call this for USB.
    fn read_packet(&mut self, _out: &mut Vec<u8>, _expected_bytes: usize) -> bool {
        false
    }

    fn last_error(&self) -> Option<&str> {
        None
    }
}

// ---------------------------------------------------------------------------
// What comes out
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SampleBlock {
    pub frames: usize,
    /// Whether a second channel was delivered (into `pump`'s `out_b`).
    pub dual: bool,
    pub status: Status,
    /// Packet or block counter.
    pub sequence: u32,
    /// Counter skipped: data was lost. USB only — see [`Device::note_sequence`]'s
    /// own note on why LAN cannot derive this reliably from its own counter
    /// field.
    pub sequence_gap: bool,
}

/// What one [`Device::pump`] call found, if anything.
#[derive(Debug, Clone, Default)]
pub struct PumpOutcome {
    /// `Some` when a block/packet's samples were unpacked into the
    /// caller's buffers this call.
    pub samples: Option<SampleBlock>,
    /// Every reply embedded in this frame, in the order the radio sent
    /// them — usually 0 or 1; the protocol allows more.
    pub replies: Vec<Reply>,
    /// A malformed frame the transport itself did not already filter out
    /// (a well-behaved transport already resyncs on its own — see
    /// `crate::lan::LanTcpTransport` — so this is mostly a defence against
    /// a misbehaving or fake one, exercised in this module's own tests).
    pub error: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Config {
    pub adc_clock_hz: f64,
    pub gps_discipline: bool,
    /// Rate = `2^(decimation_exp+1)`.
    pub decimation_exp: i32,
    pub format: StreamFormat,
    pub op_mode: OpMode,
    pub swap_channels: bool,
    /// Only meaningful in [`OpMode::Serial`]. Should match the parity of
    /// `tune_for(tuned_hz, adc_clock_hz).zone` — odd zone → `false` (SerL),
    /// even zone → `true` (SerU). A manual field regardless: easy to notice
    /// and correct by ear while tuning, not worth removing the choice over.
    pub upper_sideband: bool,
    pub tuned_hz: f64,
    pub switch_register: u16,
    /// 0..35 normally, 0..19 whenever Auto-ATT is on (the automatic +16dB
    /// step needs headroom above it).
    pub attenuator1: i32,
    pub attenuator2: i32,

    /// 0 = off, 1..5 = -6dB..-30dB in fixed 6dB steps. "Enabled" is
    /// `auto_att_threshold > 0`; there is no separate on/off field, so the
    /// two can never disagree with each other.
    pub auto_att_threshold: i32,
    pub auto_att_hold_time_sec: f64,
    /// Per-channel calibration multipliers, nominal 6.3096x (the DP's own
    /// worked value, = the attenuator's nominal 16dB).
    pub auto_att_gain_ch1: f32,
    pub auto_att_gain_ch2: f32,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            adc_clock_hz: 125e6,
            gps_discipline: true,
            decimation_exp: 3,
            format: StreamFormat { channels: 1, bits: 16 },
            op_mode: OpMode::ParallelAdd,
            swap_channels: false,
            upper_sideband: false,
            tuned_hz: 10e6,
            switch_register: 0,
            attenuator1: 0,
            attenuator2: 0,
            auto_att_threshold: 0,
            auto_att_hold_time_sec: 0.2,
            auto_att_gain_ch1: 6.3096,
            auto_att_gain_ch2: 6.3096,
        }
    }
}

impl Config {
    pub fn sample_rate_hz(&self) -> f64 {
        crate::protocol::sample_rate_hz(self.adc_clock_hz, self.decimation_exp)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Default)]
struct PendingCommand {
    has_command: bool,
    number: u32,
    instruction: u8,
    bytes: Vec<u8>,
    sent_at_ms: u64,
    attempts: u32,
}

pub struct Device {
    cfg: Config,
    tuning: Tuning,
    pending: PendingCommand,

    command_counter: u32,
    last_command_no: u8,
    last_sequence: u32,
    expect_sequence: bool,
    streaming: bool,
    configured_once: bool,

    frame_buf: Vec<u8>,
    last_block: SampleBlock,
}

impl Device {
    /// How long to wait for an acknowledgement, and how many times to try.
    pub const ACK_TIMEOUT_MS: u64 = 500;
    pub const MAX_ATTEMPTS: u32 = 3;

    pub fn new() -> Self {
        Device {
            cfg: Config::default(),
            tuning: Tuning { zone: 1, lo_hz: 0.0, spectrum_inverted: false, alias_below_hz: 0.0, alias_above_hz: 0.0 },
            pending: PendingCommand::default(),
            command_counter: 0,
            last_command_no: 0,
            last_sequence: 0,
            expect_sequence: false,
            streaming: false,
            configured_once: false,
            frame_buf: Vec::new(),
            last_block: SampleBlock::default(),
        }
    }

    /// Clear configured/streaming/pending state. Call after discarding a
    /// transport (a dead connection, a deliberate reconnect): DP 3.3 says
    /// Stop Stream closes the USB endpoint entirely, so whatever the
    /// radio's own switch/clock/format state was carries no guarantee into
    /// the *next* Start — the next Start has to reopen and reconfigure from
    /// scratch either way (see [`Self::apply_config`]'s own comment).
    pub fn reset_for_new_transport(&mut self) {
        self.configured_once = false;
        self.pending.has_command = false;
        self.streaming = false;
        self.expect_sequence = false;
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn sample_rate(&self) -> f64 {
        self.cfg.sample_rate_hz()
    }

    pub fn current_tuning(&self) -> Tuning {
        self.tuning
    }

    pub fn layout(&self) -> BlockLayout {
        lan_layout(self.cfg.format)
    }

    pub fn is_streaming(&self) -> bool {
        self.streaming
    }

    pub fn awaiting_ack(&self) -> bool {
        self.pending.has_command
    }

    pub fn pending_number(&self) -> u32 {
        self.pending.number
    }

    pub fn pending_attempts(&self) -> u32 {
        self.pending.attempts
    }

    pub fn last_command_number(&self) -> u32 {
        self.command_counter
    }

    // -----------------------------------------------------------------
    // Configuration
    //
    // Order matters and is dictated by the documents:
    //  - Changing the transmission settings on LAN stops any running
    //    stream, and the restart has to name a size code matching the new
    //    format (DP 3.3).
    //  - Changing the ADC clock or the transmission settings triggers a
    //    synchronisation event, which is what puts the two channels back in
    //    phase (DP 4.6). So the clock is set before the LOs, never after.
    // -----------------------------------------------------------------

    pub fn apply_config(&mut self, transport: &mut dyn Transport, next: &Config, now_ms: u64) -> Result<()> {
        let format_changed = next.format != self.cfg.format
            || next.decimation_exp != self.cfg.decimation_exp
            || next.op_mode != self.cfg.op_mode
            || next.swap_channels != self.cfg.swap_channels
            || next.upper_sideband != self.cfg.upper_sideband;
        let clock_changed =
            next.adc_clock_hz != self.cfg.adc_clock_hz || next.gps_discipline != self.cfg.gps_discipline;
        // Auto-ATT's own command (0xB1), independent of the transmission one
        // format_changed tracks — not itself a reason to stop and restart
        // the stream. clock_changed is included too: hold time is expressed
        // in raw ADC clock cycles, so the DP's own caution ("must be
        // reloaded each time the ADC clock frequency is changed") means a
        // clock change alone has to resend this even if nothing about
        // Auto-ATT's own settings changed.
        let auto_att_changed = next.auto_att_threshold != self.cfg.auto_att_threshold
            || next.auto_att_hold_time_sec != self.cfg.auto_att_hold_time_sec
            || next.auto_att_gain_ch1 != self.cfg.auto_att_gain_ch1
            || next.auto_att_gain_ch2 != self.cfg.auto_att_gain_ch2;

        if self.streaming && (format_changed || clock_changed) {
            self.stop_stream(transport, now_ms)?;
        }

        self.cfg = next.clone();
        let is_lan = transport.is_lan();

        if clock_changed || !self.configured_once {
            let no = self.next_number();
            let cmd = cmd_set_adc_clock(no, is_lan, self.cfg.adc_clock_hz, self.cfg.gps_discipline, 0);
            self.send(transport, cmd, now_ms, true)?;
        }

        if format_changed || !self.configured_once {
            let pm = port_mode_byte(
                self.cfg.decimation_exp,
                self.cfg.format.channels == 2,
                self.cfg.format.bits == 16,
                self.cfg.swap_channels,
            );
            let dm = dsp_mode_byte(self.cfg.op_mode, self.cfg.upper_sideband);
            let iface = if is_lan { Interface::Lan } else { Interface::Usb };
            let no = self.next_number();
            let cmd = cmd_set_data_transmission(no, is_lan, iface, pm, dm, 0);
            self.send(transport, cmd, now_ms, true)?;
        }

        let no = self.next_number();
        let cmd = cmd_set_variable(no, is_lan, Variable::Switch, self.cfg.switch_register, 0);
        self.send(transport, cmd, now_ms, true)?;

        let no = self.next_number();
        let cmd = cmd_set_variable(no, is_lan, Variable::AttenuatorAdc1, self.cfg.attenuator1.clamp(0, 35) as u16, 0);
        self.send(transport, cmd, now_ms, true)?;

        let no = self.next_number();
        let cmd = cmd_set_variable(no, is_lan, Variable::AttenuatorAdc2, self.cfg.attenuator2.clamp(0, 35) as u16, 0);
        self.send(transport, cmd, now_ms, true)?;

        if auto_att_changed || clock_changed || !self.configured_once {
            let hold_clocks = auto_att_hold_time_clocks(self.cfg.auto_att_hold_time_sec, self.cfg.adc_clock_hz);
            let g1 = auto_att_gain_lsb(f64::from(self.cfg.auto_att_gain_ch1));
            let g2 = auto_att_gain_lsb(f64::from(self.cfg.auto_att_gain_ch2));
            let no = self.next_number();
            let cmd = cmd_set_auto_attenuator(
                no,
                is_lan,
                self.cfg.auto_att_threshold.clamp(0, 5) as u8,
                hold_clocks,
                g1,
                g2,
                0,
            );
            self.send(transport, cmd, now_ms, true)?;
        }

        // Tuning last, so it follows the synchronisation event rather than
        // preceding it.
        let tuned_hz = self.cfg.tuned_hz;
        self.tune(transport, tuned_hz, now_ms)?;

        self.configured_once = true;
        let layout = self.layout();
        transport.set_layout(layout);
        Ok(())
    }

    /// Always both oscillators in one command. DP 4.6: tuning them
    /// separately and later returning them to a common frequency leaves the
    /// phase relationship undefined until the next synchronisation event,
    /// which silently ruins phasing.
    pub fn tune(&mut self, transport: &mut dyn Transport, rf_hz: f64, now_ms: u64) -> Result<()> {
        self.cfg.tuned_hz = rf_hz;
        self.tuning = tune_for(rf_hz, self.cfg.adc_clock_hz);
        let is_lan = transport.is_lan();
        let no = self.next_number();
        let cmd = cmd_set_lo_both(no, is_lan, self.tuning.lo_hz);
        self.send(transport, cmd, now_ms, true)
    }

    /// The hardware diversity weight, for users who would rather the radio
    /// combined the channels than send both. Not usable as an adaptive
    /// control: the round trip through the command channel is far too slow
    /// for a loop.
    pub fn set_hardware_diversity(
        &mut self,
        transport: &mut dyn Transport,
        magnitude: f64,
        phase_degrees: f64,
        now_ms: u64,
    ) -> Result<()> {
        let is_lan = transport.is_lan();
        let no = self.next_number();
        let cmd = cmd_set_generator(no, is_lan, GenSelect::MagPhaseCh2, pack_magnitude_phase(magnitude, phase_degrees), 0);
        self.send(transport, cmd, now_ms, true)
    }

    /// Hand a combination worked out in software to the radio's own
    /// combiner. The caller is expected to be in Separate mode while
    /// solving and to switch to [`OpMode::Diversity`] afterwards; this only
    /// carries the weight across. Returns the computed weight either way,
    /// so a caller can act on `suggest_swap` even when nothing was sent.
    pub fn set_hardware_diversity_from(
        &mut self,
        transport: &mut dyn Transport,
        k0: Complex64,
        k1: Complex64,
        now_ms: u64,
    ) -> (HardwareWeight, Result<()>) {
        let h = hardware_weight_for(k0, k1);
        if !h.representable {
            return (h, Err(Error::NotRepresentable));
        }
        let r = self.set_hardware_diversity(transport, h.magnitude, h.phase_degrees, now_ms);
        (h, r)
    }

    pub fn start_stream(&mut self, transport: &mut dyn Transport, now_ms: u64) -> Result<()> {
        let layout = self.layout();
        transport.set_layout(layout);
        self.expect_sequence = false;
        let is_lan = transport.is_lan();
        let port = transport.stream_port();
        let no = self.next_number();
        let cmd = cmd_start_stream(no, is_lan, port, layout.start_stream_size_code);
        // Start and Stop are not acknowledged (DP 3.3), so nothing to wait
        // for.
        self.send(transport, cmd, now_ms, false)?;
        self.streaming = true;
        Ok(())
    }

    pub fn stop_stream(&mut self, transport: &mut dyn Transport, now_ms: u64) -> Result<()> {
        let is_lan = transport.is_lan();
        let port = transport.stream_port();
        let no = self.next_number();
        let cmd = cmd_stop_stream(no, is_lan, port, 0);
        self.send(transport, cmd, now_ms, false)?;
        self.streaming = false;
        Ok(())
    }

    // -----------------------------------------------------------------
    // Running
    // -----------------------------------------------------------------

    /// Take one frame from the transport and unpack any samples into
    /// `out_a`/`out_b` (the latter ignored unless the current format is
    /// dual channel). `None` means the transport has stopped; `Some` with
    /// everything empty means a frame arrived but carried nothing new (an
    /// unchanged command number and no samples worth delivering would be
    /// unusual but is not itself an error).
    pub fn pump(&mut self, transport: &mut dyn Transport, out_a: &mut Vec<f32>, out_b: &mut Vec<f32>) -> Option<PumpOutcome> {
        let mut frame = std::mem::take(&mut self.frame_buf);
        let ok = transport.next_frame(&mut frame);
        let kind = transport.kind();
        let outcome = if ok {
            if kind == TransportKind::Usb {
                self.parse_usb_packet(&frame, out_a, out_b)
            } else {
                self.parse_lan_block(&frame, out_a, out_b)
            }
        } else {
            None
        };
        self.frame_buf = frame;
        if !ok {
            return None;
        }
        Some(outcome.unwrap_or_default())
    }

    /// Re-issue anything that has gone unacknowledged. Call periodically.
    ///
    /// DP 3.5 documents a repeat counter for this, then says firmware 22x
    /// ignores it and simply executes the command again. So a retry goes
    /// out under a *fresh* number and the caller must tolerate the original
    /// having landed as well — bumping the repeat field would achieve
    /// nothing.
    pub fn service(&mut self, transport: &mut dyn Transport, now_ms: u64) -> Result<()> {
        if !self.pending.has_command {
            return Ok(());
        }
        if now_ms.saturating_sub(self.pending.sent_at_ms) < Self::ACK_TIMEOUT_MS {
            return Ok(());
        }

        if self.pending.attempts >= Self::MAX_ATTEMPTS {
            self.pending.has_command = false;
            return Err(Error::NoAck { instruction: self.pending.instruction, attempts: self.pending.attempts });
        }

        self.pending.attempts += 1;
        let no = self.next_number();
        self.pending.number = no;
        write_u32(&mut self.pending.bytes[0..4], no);
        self.pending.sent_at_ms = now_ms;
        transport.send_command(&self.pending.bytes);
        Ok(())
    }

    // -----------------------------------------------------------------
    // Internals
    // -----------------------------------------------------------------

    /// Never 0: the radio reserves 0 to mark commands it generated itself,
    /// so a PC command numbered 0 would be indistinguishable from one.
    fn next_number(&mut self) -> u32 {
        self.command_counter = self.command_counter.wrapping_add(1);
        if self.command_counter == 0 {
            self.command_counter = 1;
        }
        self.command_counter
    }

    fn send(&mut self, transport: &mut dyn Transport, bytes: Vec<u8>, now_ms: u64, expect_ack: bool) -> Result<()> {
        if !transport.send_command(&bytes) {
            return Err(Error::CommandRejected);
        }
        if expect_ack {
            // Deliberately always the async/embedded-reply path, never a
            // synchronous packet-mode read, even for LAN before streaming
            // has started — see `Transport::read_packet`'s own doc for why.
            // `pending` just sits registered (only the most recently sent
            // command, since it's a single slot, not a queue) until the
            // first streaming block's embedded reply satisfies it, or until
            // enough calls to `service()` after streaming starts time it
            // out.
            self.pending.has_command = true;
            self.pending.number = read_u32(&bytes);
            self.pending.instruction = bytes[4];
            self.pending.bytes = bytes;
            self.pending.sent_at_ms = now_ms;
            self.pending.attempts = 0;
        }
        Ok(())
    }

    /// `check_gap` is false for LAN. USB packets can genuinely be lost on
    /// the bus, so "the counter didn't advance by exactly 1" is a real,
    /// meaningful signal there. LAN's TCP connection can't lose or reorder
    /// bytes the same way — if `next_frame()` hands back a block at all,
    /// its sync words and inverted counter have both already validated,
    /// meaning it is genuinely the next chunk of bytes the radio sent, full
    /// stop; a dead connection is caught separately, by `next_frame()`
    /// itself returning false. What isn't reliable on LAN is the counter
    /// *field*'s own value: live testing found its steady-state per-block
    /// delta isn't a constant +1 at all — a decimation-dependent, sometimes
    /// non-monotonic quirk of the radio's own firmware, not lost data.
    /// Treating it as a gap indicator on LAN just produces constant false
    /// positives, so LAN skips the check and only stores the raw counter
    /// (still exposed via [`SampleBlock::sequence`]) rather than deriving
    /// `sequence_gap` from it.
    fn note_sequence(&mut self, counter: u32, check_gap: bool) -> (u32, bool) {
        let gap = check_gap && self.expect_sequence && counter != self.last_sequence.wrapping_add(1);
        self.last_sequence = counter;
        self.expect_sequence = true;
        (counter, gap)
    }

    fn deliver(&mut self, iq: &[u8], frames: usize, out_a: &mut Vec<f32>, out_b: &mut Vec<f32>) -> SampleBlock {
        // Constant 2-bit (4x) headroom shift whenever Auto-ATT is enabled at
        // all (threshold > 0), independent of whether it is *currently*
        // engaged — OM: "the entire level range is shifted by 2 bits... as
        // soon as Auto ATT is turned on." A materially wider condition than
        // `status.auto_att_active` (the momentary engaged-state flag).
        let mut gain_a = if self.cfg.auto_att_threshold > 0 { AUTO_ATT_GAIN } else { 1.0 };
        let mut gain_b = gain_a;
        // Gated on `auto_att_threshold` too, not just the status flag alone
        // — the DP's own caution that the -128C/"active" indicator persists
        // for up to ~0.5s after the attenuator itself has actually released
        // means the status flag is not a trustworthy sole signal right at a
        // transition.
        if self.cfg.auto_att_threshold > 0 && self.last_block.status.auto_att_active {
            // An additional, variable, *per-channel* correction for the
            // attenuator's own ~16dB while actually engaged, using the
            // calibrated gain multiplier from the last Set Auto-ATT
            // command.
            gain_a *= self.cfg.auto_att_gain_ch1;
            gain_b *= self.cfg.auto_att_gain_ch2;
        }

        let dual = self.cfg.format.channels == 2;
        let need = frames * 2;
        if out_a.len() < need {
            out_a.resize(need, 0.0);
        }
        if dual && out_b.len() < need {
            out_b.resize(need, 0.0);
        }

        if dual {
            unpack(iq, frames, self.cfg.format, gain_a, gain_b, &mut out_a[..need], &mut out_b[..need]);
        } else {
            unpack(iq, frames, self.cfg.format, gain_a, gain_b, &mut out_a[..need], &mut []);
        }

        // `tune_for()` already works out, per the DP's own Nyquist-zone
        // arithmetic, exactly when the current tuning lands in an even zone
        // and therefore comes off the ADC mirrored. Negating the Q sample
        // of every complex pair conjugates the signal, the standard
        // correction for a mirrored spectrum, applied automatically and
        // only when the current tuning actually needs it.
        if self.tuning.spectrum_inverted {
            for i in (1..need).step_by(2) {
                out_a[i] = -out_a[i];
            }
            if dual {
                for i in (1..need).step_by(2) {
                    out_b[i] = -out_b[i];
                }
            }
        }

        self.last_block.frames = frames;
        self.last_block.dual = dual;
        self.last_block
    }

    /// A command is only new when the command number changes; the same
    /// data repeats in every frame until the radio writes something else
    /// (DP 3.1).
    fn handle_command_number(&mut self, number: u8, commands: &[u8], count: usize) -> Vec<Reply> {
        if number == 0 || number == self.last_command_no {
            return Vec::new();
        }
        self.last_command_no = number;
        let mut replies = Vec::with_capacity(count);
        for i in 0..count {
            let r = parse_embedded_command(&commands[i * 8..]);
            // Only a reply carrying our own number clears a pending
            // command; a self-generated report (number 0) must not.
            if self.pending.has_command && !r.self_generated && r.confirmed_command == self.pending.number {
                self.pending.has_command = false;
            }
            replies.push(r);
        }
        replies
    }

    fn parse_usb_packet(&mut self, p: &[u8], out_a: &mut Vec<f32>, out_b: &mut Vec<f32>) -> Option<PumpOutcome> {
        use crate::protocol::{USB_CMD_NO_OFFSET, USB_COMMAND_OFFSET, USB_GPS_OFFSET, USB_IQ_OFFSET, USB_PACKET_BYTES, USB_TEMP_OFFSET};
        if p.len() < USB_PACKET_BYTES {
            return None;
        }
        self.last_block.status = parse_status(p[USB_TEMP_OFFSET], p[USB_GPS_OFFSET], p[USB_GPS_OFFSET + 1]);
        let (seq, gap) = self.note_sequence(read_u32(p), true);
        self.last_block.sequence = seq;
        self.last_block.sequence_gap = gap;
        let replies = self.handle_command_number(p[USB_CMD_NO_OFFSET], &p[USB_COMMAND_OFFSET..], 1);
        let format = self.cfg.format;
        let sample_block = self.deliver(&p[USB_IQ_OFFSET..], usb_samples_per_packet(format), out_a, out_b);
        Some(PumpOutcome { samples: Some(sample_block), replies, error: None })
    }

    fn parse_lan_block(&mut self, b: &[u8], out_a: &mut Vec<f32>, out_b: &mut Vec<f32>) -> Option<PumpOutcome> {
        let l = self.layout();
        if b.len() < l.block_bytes {
            return None;
        }
        if !block_trailer_valid(b, &l) {
            return Some(PumpOutcome {
                samples: None,
                replies: Vec::new(),
                error: Some("LAN block failed its sync check".to_string()),
            });
        }

        self.last_block.status = parse_status(b[l.temp_offset], b[l.gps_offset], b[l.gps_offset + 1]);
        let (seq, gap) = self.note_sequence(read_u32(&b[l.counter_offset..]), false);
        self.last_block.sequence = seq;
        self.last_block.sequence_gap = gap;

        let mut count = read_u32(&b[l.cmd_count_offset..]) as usize;
        count = count.min(l.command_space / 8);
        let replies = self.handle_command_number(b[l.cmd_no_offset], &b[l.commands_offset..], count);

        let sample_block = self.deliver(b, l.samples_per_channel, out_a, out_b);
        Some(PumpOutcome { samples: Some(sample_block), replies, error: None })
    }
}

impl Default for Device {
    fn default() -> Self {
        Device::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{SYNC_BYTES, instr};
    use std::collections::VecDeque;

    /// Records what the device sends and hands back frames on demand.
    #[derive(Default)]
    struct FakeTransport {
        kind: TransportKindOpt,
        sent: Vec<Vec<u8>>,
        frames: VecDeque<Vec<u8>>,
        last_layout: Option<BlockLayout>,
        refuse: bool,
    }

    /// A `TransportKind` with a `Default` (`LanTcp`), since the real enum
    /// deliberately has none — every real transport knows its own kind at
    /// construction, so a fallback would only ever paper over a bug there.
    #[derive(Clone, Copy)]
    struct TransportKindOpt(TransportKind);
    impl Default for TransportKindOpt {
        fn default() -> Self {
            TransportKindOpt(TransportKind::LanTcp)
        }
    }

    impl FakeTransport {
        fn new() -> Self {
            FakeTransport::default()
        }

        fn usb() -> Self {
            FakeTransport { kind: TransportKindOpt(TransportKind::Usb), ..Default::default() }
        }

        fn count_of(&self, instruction: u8) -> usize {
            self.sent.iter().filter(|c| c[4] == instruction).count()
        }

        fn first_of(&self, instruction: u8) -> Option<&Vec<u8>> {
            self.sent.iter().find(|c| c[4] == instruction)
        }

        fn index_of(&self, instruction: u8) -> Option<usize> {
            self.sent.iter().position(|c| c[4] == instruction)
        }
    }

    impl Transport for FakeTransport {
        fn kind(&self) -> TransportKind {
            self.kind.0
        }

        fn send_command(&mut self, data: &[u8]) -> bool {
            if self.refuse {
                return false;
            }
            self.sent.push(data.to_vec());
            true
        }

        fn next_frame(&mut self, out: &mut Vec<u8>) -> bool {
            match self.frames.pop_front() {
                Some(f) => {
                    *out = f;
                    true
                }
                None => false,
            }
        }

        fn set_layout(&mut self, layout: BlockLayout) {
            self.last_layout = Some(layout);
        }
    }

    /// Build a LAN block carrying a given counter, command number and one
    /// embedded reply.
    fn make_lan_block(l: &BlockLayout, counter: u32, cmd_no: u8, reply: Option<&[u8; 8]>, temp: u8) -> Vec<u8> {
        let mut b = vec![0u8; l.block_bytes];
        write_u32(&mut b[l.counter_offset..], counter);
        write_u32(&mut b[l.inv_counter_offset..], !counter);
        b[l.sync_offset..l.sync_offset + SYNC_BYTES.len()].copy_from_slice(&SYNC_BYTES);
        b[l.temp_offset] = temp;
        b[l.cmd_no_offset] = cmd_no;
        write_u32(&mut b[l.cmd_count_offset..], if reply.is_some() { 1 } else { 0 });
        if let Some(r) = reply {
            b[l.commands_offset..l.commands_offset + 8].copy_from_slice(r);
        }
        b
    }

    fn make_confirmation(for_command: u32) -> [u8; 8] {
        let mut out = [0u8; 8];
        out[4..8].copy_from_slice(&for_command.to_le_bytes());
        out
    }

    fn poke_sample(block: &mut [u8], i_val: i16, q_val: i16) {
        block[0..2].copy_from_slice(&i_val.to_le_bytes());
        block[2..4].copy_from_slice(&q_val.to_le_bytes());
    }

    fn poke_dual(block: &mut [u8], i1: i16, q1: i16, i2: i16, q2: i16) {
        block[0..2].copy_from_slice(&i1.to_le_bytes());
        block[2..4].copy_from_slice(&q1.to_le_bytes());
        block[4..6].copy_from_slice(&i2.to_le_bytes());
        block[6..8].copy_from_slice(&q2.to_le_bytes());
    }

    #[test]
    fn command_numbers_are_never_zero_and_always_advance() {
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        d.apply_config(&mut t, &Config::default(), 0).unwrap();

        assert!(t.sent.iter().all(|c| read_u32(c) != 0), "no command is ever numbered 0");

        let ascending = t.sent.windows(2).all(|w| read_u32(&w[1]) > read_u32(&w[0]));
        assert!(ascending, "numbers advance with each command");
    }

    #[test]
    fn configuration_order_resynchronises_before_tuning() {
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        let c = Config {
            adc_clock_hz: 125e6,
            format: StreamFormat { channels: 2, bits: 16 },
            op_mode: OpMode::Independent,
            tuned_hz: 9.5e6,
            ..Config::default()
        };
        d.apply_config(&mut t, &c, 0).expect("a first configuration goes out");

        let clk = t.index_of(instr::SET_ADC_CLOCK);
        let xmit = t.index_of(instr::SET_DATA_TRANSMISSION);
        let lo = t.index_of(instr::SET_GENERATORS);
        assert!(clk.is_some() && xmit.is_some() && lo.is_some(), "clock, transmission and tuning are all sent");

        // DP 4.6: changing the clock or the transmission settings
        // resynchronises the channels. Tuning has to come after, or the
        // LOs are set and then reset underneath.
        assert!(clk < lo && xmit < lo, "tuning follows the commands that resynchronise the channels");

        let x = t.first_of(instr::SET_DATA_TRANSMISSION).expect("data transmission was sent");
        assert_eq!(x[5], Interface::Lan as u8, "the LAN interface is named");
        assert!((x[6] & (1 << 4)) != 0, "port mode says dual channel");
        assert!((x[6] & (1 << 5)) != 0, "port mode says 16 bit");
        assert_eq!(x[7] & 0x03, OpMode::Independent as u8, "DSP mode says Separate");
    }

    #[test]
    fn reconfiguring_stops_the_stream_first() {
        // DP 3.3: changing the LAN transmission settings stops streaming,
        // and the restart has to name a size code matching the new format.
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        let c = Config::default();
        d.apply_config(&mut t, &c, 0).unwrap();
        d.start_stream(&mut t, 0).unwrap();
        t.sent.clear();

        let c2 = Config { format: StreamFormat { channels: 2, bits: 24 }, ..c.clone() };
        d.apply_config(&mut t, &c2, 100).unwrap();
        assert_eq!(t.index_of(instr::STOP_STREAM), Some(0), "the stream is stopped before anything is changed");
        assert!(!d.is_streaming(), "the device knows it is no longer streaming");

        t.sent.clear();
        d.start_stream(&mut t, 200).unwrap();
        let s = t.first_of(instr::START_STREAM).expect("start stream was sent");
        assert_eq!(
            s[6],
            lan_layout(StreamFormat { channels: 2, bits: 24 }).start_stream_size_code,
            "the restart names the size code for the new format"
        );
        assert_eq!(s[5], StreamPort::Tcp as u8, "and the transport's own port");
    }

    #[test]
    fn tuning_sets_both_oscillators_with_one_command() {
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        let c = Config { adc_clock_hz: 125e6, ..Config::default() };
        d.apply_config(&mut t, &c, 0).unwrap();
        t.sent.clear();

        d.tune(&mut t, 95e6, 0).expect("tuning is accepted");
        let g = t.first_of(instr::SET_GENERATORS).expect("generators were set");
        // DP 4.6: separate per-channel tuning leaves the phase relationship
        // undefined.
        assert_eq!(g[5], GenSelect::LoBoth as u8, "both oscillators are set with one command");
        assert_eq!(read_u32(&g[6..]), 30_000_000, "95 MHz at a 125 MHz clock tunes the LO to 30 MHz");

        let tn = d.current_tuning();
        assert!(tn.zone == 2 && tn.spectrum_inverted, "and reports zone 2 with the spectrum reversed");

        assert_eq!(t.count_of(instr::SET_GENERATORS), 1, "no separate per-channel commands are sent");
    }

    #[test]
    fn acknowledgement_and_retry() {
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        d.apply_config(&mut t, &Config::default(), 0).unwrap();

        assert!(d.awaiting_ack(), "the last command is awaiting acknowledgement");
        let first = d.pending_number();

        // Nothing yet: too soon to retry.
        d.service(&mut t, 100).unwrap();
        assert!(d.pending_number() == first && d.pending_attempts() == 0, "no retry before the timeout");

        // DP 3.5: firmware 22x ignores the repeat counter, so a retry must
        // go out under a fresh number rather than repeating the old one.
        let before = t.sent.len();
        d.service(&mut t, 1000).unwrap();
        assert_eq!(t.sent.len(), before + 1, "a timeout re-issues the command");
        assert_ne!(d.pending_number(), first, "the retry carries a NEW command number");
        assert_eq!(read_u32(t.sent.last().unwrap()), d.pending_number(), "and that number is what went on the wire");
        assert_eq!(t.sent.last().unwrap()[4], t.sent[before - 1][4], "the instruction is unchanged");

        // Acknowledging the current number clears it.
        let l = d.layout();
        let reply = make_confirmation(d.pending_number());
        t.frames.push_back(make_lan_block(&l, 1, 5, Some(&reply), 25));
        let mut a = Vec::new();
        let mut b = Vec::new();
        d.pump(&mut t, &mut a, &mut b);
        assert!(!d.awaiting_ack(), "a matching confirmation clears the pending command");

        // Give up after the documented number of attempts.
        let mut t2 = FakeTransport::new();
        let mut d2 = Device::new();
        d2.tune(&mut t2, 10e6, 0).unwrap();
        let mut last_err = None;
        for i in 1..=(Device::MAX_ATTEMPTS + 1) {
            if let Err(e) = d2.service(&mut t2, u64::from(i) * 1000) {
                last_err = Some(e);
            }
        }
        assert!(last_err.is_some(), "it eventually gives up and reports");
        assert!(!d2.awaiting_ack(), "and stops retrying");
    }

    #[test]
    fn embedded_replies_are_taken_once_per_command_number_change() {
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        d.apply_config(&mut t, &Config::default(), 0).unwrap();
        let l = d.layout();

        let reply = make_confirmation(999);

        // DP 3.1: the same command data repeats in every block until the
        // number changes.
        t.frames.push_back(make_lan_block(&l, 1, 7, Some(&reply), 25));
        t.frames.push_back(make_lan_block(&l, 2, 7, Some(&reply), 25));
        t.frames.push_back(make_lan_block(&l, 3, 8, Some(&reply), 25));
        let mut a = Vec::new();
        let mut b = Vec::new();
        let mut total_replies = 0;
        for _ in 0..3 {
            if let Some(o) = d.pump(&mut t, &mut a, &mut b) {
                total_replies += o.replies.len();
            }
        }
        assert_eq!(total_replies, 2, "a reply is taken once per change of command number, not once per block");

        // A self-generated report must not clear a command we are waiting
        // on.
        let mut t2 = FakeTransport::new();
        let mut d2 = Device::new();
        d2.tune(&mut t2, 10e6, 0).unwrap();
        let waiting = d2.pending_number();
        let self_gen = [instr::SET_ADC_CLOCK, 0xD0, 0x04, 0x00, 0, 0, 0, 0];
        t2.frames.push_back(make_lan_block(&d2.layout(), 1, 3, Some(&self_gen), 25));
        d2.pump(&mut t2, &mut a, &mut b);
        assert!(
            d2.awaiting_ack() && d2.pending_number() == waiting,
            "a self-generated report does not satisfy a pending command"
        );
    }

    #[test]
    fn frame_parsing_delivers_status_and_gap_state() {
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        let c = Config { format: StreamFormat { channels: 2, bits: 16 }, ..Config::default() };
        d.apply_config(&mut t, &c, 0).unwrap();
        let l = d.layout();
        let mut a = Vec::new();
        let mut b = Vec::new();

        t.frames.push_back(make_lan_block(&l, 10, 1, None, 25));
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        let sb = o.samples.expect("a sample block was delivered");
        assert_eq!(sb.frames, l.samples_per_channel, "a block delivers its full sample count");
        assert!(sb.dual, "dual channel delivers a second channel");
        assert!(!sb.sequence_gap, "the first block is not a gap");
        assert_eq!(sb.status.temperature_c, 25, "the status header comes through");

        t.frames.push_back(make_lan_block(&l, 11, 1, None, 25));
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        assert!(!o.samples.unwrap().sequence_gap, "consecutive counters are not a gap");

        // LAN doesn't derive sequence_gap from the counter at all (see
        // `Device::note_sequence`'s own comment): TCP can't lose or
        // reorder bytes, so a block that makes it through `next_frame()`'s
        // sync/inverted-counter check is genuinely the next one the radio
        // sent regardless of what its counter field says.
        t.frames.push_back(make_lan_block(&l, 20, 1, None, 25));
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        assert!(!o.samples.unwrap().sequence_gap, "a jump in the LAN block counter is not treated as lost data");

        // Auto-ATT scales the stream down 2 bits; the device has to scale
        // it back.
        t.frames.push_back(make_lan_block(&l, 21, 1, None, 0x80));
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        assert!(o.samples.unwrap().status.auto_att_active, "the Auto-ATT indicator is recognised");

        // A corrupted trailer is refused rather than delivered as garbage.
        let mut bad = make_lan_block(&l, 22, 1, None, 25);
        bad[l.sync_offset] ^= 0xFF;
        t.frames.push_back(bad);
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        assert!(o.error.is_some(), "a block failing its sync check is rejected");
    }

    #[test]
    fn spectrum_inversion_follows_the_current_tuning() {
        // A wanted frequency's Nyquist zone -- and therefore whether it
        // comes off the ADC mirrored -- depends on the exact frequency and
        // the ADC clock, not on which of the radio's inputs it happens to
        // be reached through. 125 MHz clock, half = 62.5 MHz: 30 MHz falls
        // in zone 1 (odd, not inverted) and 80 MHz in zone 2 (even,
        // inverted), so one Device exercises both without needing separate
        // setups.
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        let c = Config { format: StreamFormat { channels: 1, bits: 16 }, adc_clock_hz: 125e6, ..Config::default() };
        d.apply_config(&mut t, &c, 0).unwrap();
        let l = d.layout();
        let mut a = Vec::new();
        let mut b = Vec::new();

        d.tune(&mut t, 30e6, 0).unwrap();
        assert!(d.current_tuning().zone == 1 && !d.current_tuning().spectrum_inverted, "30 MHz at a 125 MHz clock is zone 1, not inverted");
        let mut not_inverted = make_lan_block(&l, 1, 1, None, 25);
        poke_sample(&mut not_inverted, 1000, 2000);
        t.frames.push_back(not_inverted);
        d.pump(&mut t, &mut a, &mut b);
        assert!((a[1] - 2000.0 / 32768.0).abs() < 1e-6, "not inverted: Q comes through unchanged");

        d.tune(&mut t, 80e6, 0).unwrap();
        assert!(d.current_tuning().zone == 2 && d.current_tuning().spectrum_inverted, "80 MHz at the same clock is zone 2, inverted");
        let mut inverted = make_lan_block(&l, 2, 1, None, 25);
        poke_sample(&mut inverted, 1000, 2000);
        t.frames.push_back(inverted);
        d.pump(&mut t, &mut a, &mut b);
        assert!((a[0] - 1000.0 / 32768.0).abs() < 1e-6, "inverted: I is untouched");
        assert!((a[1] + 2000.0 / 32768.0).abs() < 1e-6, "inverted: Q is negated to conjugate the spectrum");

        // Retuning back out of the even zone stops correcting again --
        // this isn't a sticky per-session setting, it tracks the tuning
        // live.
        d.tune(&mut t, 30e6, 0).unwrap();
        let mut back_to_odd = make_lan_block(&l, 3, 1, None, 25);
        poke_sample(&mut back_to_odd, 1000, 2000);
        t.frames.push_back(back_to_odd);
        d.pump(&mut t, &mut a, &mut b);
        assert!((a[1] - 2000.0 / 32768.0).abs() < 1e-6, "retuning back to an odd zone stops inverting again");
    }

    #[test]
    fn auto_att_gain_compensation_is_per_channel_and_gated_on_engagement() {
        // OM: "the entire level range is shifted by 2 bits... as soon as
        // Auto ATT is turned on" -- a materially wider condition than the
        // momentary `auto_att_active` status flag.
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        let c = Config {
            format: StreamFormat { channels: 2, bits: 16 },
            auto_att_threshold: 3, // enabled, not yet engaged
            auto_att_gain_ch1: 2.0,
            auto_att_gain_ch2: 5.0,
            ..Config::default()
        };
        d.apply_config(&mut t, &c, 0).unwrap();
        let l = d.layout();
        let mut a = Vec::new();
        let mut b = Vec::new();

        // Enabled but not (yet) engaged (an ordinary temperature byte, not
        // 0x80): the constant 4x headroom shift applies to both channels,
        // but neither channel's own calibration multiplier does -- that
        // only applies once actually engaged.
        let mut not_engaged = make_lan_block(&l, 1, 1, None, 25);
        poke_dual(&mut not_engaged, 1000, 0, 1000, 0);
        t.frames.push_back(not_engaged);
        d.pump(&mut t, &mut a, &mut b);
        assert!(
            (a[0] - 1000.0 * AUTO_ATT_GAIN / 32768.0).abs() < 1e-5,
            "enabled-but-not-engaged: channel A gets the constant 4x shift only"
        );
        assert!(
            (b[0] - 1000.0 * AUTO_ATT_GAIN / 32768.0).abs() < 1e-5,
            "enabled-but-not-engaged: channel B gets the same constant shift, no calibration yet"
        );

        // Engaged (temperature byte 0x80, the DP/OM's own Auto-ATT-active
        // indicator): each channel's own calibration multiplier stacks on
        // top of the constant shift, and the two channels' calibration
        // must not cross.
        let mut engaged = make_lan_block(&l, 2, 1, None, 0x80);
        poke_dual(&mut engaged, 1000, 0, 1000, 0);
        t.frames.push_back(engaged);
        d.pump(&mut t, &mut a, &mut b);
        assert!(
            (a[0] - 1000.0 * AUTO_ATT_GAIN * 2.0 / 32768.0).abs() < 1e-5,
            "engaged: channel A additionally takes its own calibrated gain"
        );
        assert!(
            (b[0] - 1000.0 * AUTO_ATT_GAIN * 5.0 / 32768.0).abs() < 1e-5,
            "engaged: channel B takes its own, different calibrated gain"
        );

        // Off (threshold 0): no shift at all, even with the engaged status
        // flag set -- the two conditions are independent, not one derived
        // from the other.
        let c2 = Config { auto_att_threshold: 0, ..c };
        d.apply_config(&mut t, &c2, 100).unwrap();
        let mut off = make_lan_block(&l, 3, 1, None, 0x80);
        poke_dual(&mut off, 1000, 0, 1000, 0);
        t.frames.push_back(off);
        d.pump(&mut t, &mut a, &mut b);
        assert!((a[0] - 1000.0 / 32768.0).abs() < 1e-5, "Auto-ATT off: no gain shift at all, regardless of the status flag");
    }

    #[test]
    fn auto_att_command_is_sent_and_resent_at_the_right_times() {
        let mut t = FakeTransport::new();
        let mut d = Device::new();
        let c = Config {
            adc_clock_hz: 125e6,
            auto_att_threshold: 3,
            auto_att_hold_time_sec: 0.05,
            auto_att_gain_ch1: 2.0,
            auto_att_gain_ch2: 3.0,
            ..Config::default()
        };
        d.apply_config(&mut t, &c, 0).unwrap();

        let a = t.first_of(instr::SET_AUTO_ATT).expect("Auto-ATT is sent as part of the first configuration");
        assert_eq!(a[5], 3, "with the configured threshold");
        let hold = u32::from(a[6]) | (u32::from(a[7]) << 8) | (u32::from(a[8]) << 16);
        assert_eq!(hold, auto_att_hold_time_clocks(0.05, 125e6), "and the hold time converted from the current ADC clock");

        // Unrelated settings changing must not resend it.
        t.sent.clear();
        let c2 = Config { tuned_hz: 20e6, ..c.clone() };
        d.apply_config(&mut t, &c2, 100).unwrap();
        assert!(t.index_of(instr::SET_AUTO_ATT).is_none(), "retuning alone does not resend Auto-ATT settings");

        // Its own settings changing must resend it.
        t.sent.clear();
        let c3 = Config { auto_att_threshold: 2, ..c2.clone() };
        d.apply_config(&mut t, &c3, 200).unwrap();
        assert!(t.index_of(instr::SET_AUTO_ATT).is_some(), "changing the threshold resends Auto-ATT settings");

        // The DP says hold time must be reloaded whenever the ADC clock
        // changes, since it's expressed in raw clock cycles -- an ADC
        // clock change alone (nothing about Auto-ATT itself) still has to
        // resend it.
        t.sent.clear();
        let c4 = Config { adc_clock_hz: 100e6, ..c3 };
        d.apply_config(&mut t, &c4, 300).unwrap();
        let resent = t.first_of(instr::SET_AUTO_ATT).expect("an ADC clock change alone still resends Auto-ATT settings");
        let hold2 = u32::from(resent[6]) | (u32::from(resent[7]) << 8) | (u32::from(resent[8]) << 16);
        assert_eq!(hold2, auto_att_hold_time_clocks(0.05, 100e6), "and the resend carries the hold time reconverted for the new clock");
    }

    #[test]
    fn usb_framing_uses_the_same_device() {
        let mut t = FakeTransport::usb();
        let mut d = Device::new();
        let c = Config { format: StreamFormat { channels: 1, bits: 16 }, ..Config::default() };
        d.apply_config(&mut t, &c, 0).unwrap();

        // Commands take their USB lengths without the device knowing or
        // caring.
        let g = t.first_of(instr::SET_GENERATORS).expect("generators were set");
        assert_eq!(g.len(), 12, "commands use the USB length on a USB transport");

        d.start_stream(&mut t, 0).unwrap();
        let s = t.first_of(instr::START_STREAM).expect("start stream was sent");
        assert_eq!(s[5], StreamPort::Usb as u8, "and the USB stream port");

        let mut a = Vec::new();
        let mut b = Vec::new();
        let mut pkt = vec![0u8; crate::protocol::USB_PACKET_BYTES];
        write_u32(&mut pkt, 1);
        pkt[crate::protocol::USB_TEMP_OFFSET] = 30;
        pkt[crate::protocol::USB_CMD_NO_OFFSET] = 0;
        t.frames.push_back(pkt.clone());
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        let sb = o.samples.unwrap();
        assert_eq!(sb.frames, 1020, "a USB packet delivers 1020 samples at 1 channel 16 bit");
        assert!(!sb.sequence_gap, "the first USB packet is not a gap");

        // Unlike LAN, USB packets can genuinely be lost on the bus, so gap
        // detection still applies to the counter here.
        write_u32(&mut pkt, 2);
        t.frames.push_back(pkt.clone());
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        assert!(!o.samples.unwrap().sequence_gap, "consecutive USB counters are not a gap");

        write_u32(&mut pkt, 9);
        t.frames.push_back(pkt);
        let o = d.pump(&mut t, &mut a, &mut b).unwrap();
        assert!(o.samples.unwrap().sequence_gap, "a jump in the USB packet counter is reported as lost data");
    }
}
