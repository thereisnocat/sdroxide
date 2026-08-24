//! Wire protocol for the Reuter RSR200B, from `RSR200_DP_ENG_V52.pdf` (firmware 225).
//! See `RSR200_PLAN.md` at the workspace root.
//!
//! Ported from the already-tested C++ implementation in SDR++'s own
//! `source_modules/rsr200_source/src/rsr200_protocol.h` — see that plan
//! document's §1 for why this crate exists at all rather than starting from a
//! blank page. Block geometry, command construction, sample unpacking and
//! Nyquist-zone arithmetic are all pure functions over bytes, deliberately
//! free of any I/O or `sdroxide-types` dependency (this crate is the one
//! `sdroxide-types` must never depend on, not the other way round — see that
//! crate's own boundary note on `DiversityMode`), so they can be exercised
//! against the documented layouts without a radio on the desk. Everything
//! here is little-endian on the wire.
//!
//! The USB and LAN interfaces share this command set and differ only in
//! framing, which is why the two transports (not yet built) sit above this
//! rather than each carrying their own copy.
//!
//! A few adaptations from the C++ original, all mechanical rather than
//! semantic: `u32::to/from_le_bytes` replaces hand-rolled byte shuffling,
//! `Option<Reply>` replaces an out-parameter-plus-`bool` return, and enums
//! that were plain `int`-castable in C++ are `#[repr(u8)]` Rust enums cast
//! with `as u8` at the same call sites. The wire encoding, every constant,
//! and every worked example this module is tested against are unchanged.

use num_complex::Complex;

/// `hardware_weight_for`'s domain — the software solve is done in `f64`
/// throughout in the C++ original, and there is no reason to lose precision
/// converting to the radio's own units.
pub type Complex64 = Complex<f64>;

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Interface {
    Usb = 1,
    Lan = 2,
    /// Change DSP mode without touching an interface.
    DspOnly = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamPort {
    Udp = 0,
    Tcp = 1,
    Usb = 2,
}

/// DSP mode bits 0-1. `Independent` is what dual-channel phasing needs; it
/// requires the port mode to be dual-channel as well.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpMode {
    /// "Separate": two unrelated channels.
    Independent = 0,
    /// ADC1 + ADC2 summed.
    ParallelAdd = 1,
    /// Time-interleaved sampling, doubles the Nyquist zones.
    Serial = 2,
    /// ADC1 + ADC2 with the hardware magnitude/phase weight.
    Diversity = 3,
}

/// "Set frequency generators or IP address" (`0xB0`) selector.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GenSelect {
    LoCh1 = 0,
    LoCh2 = 1,
    /// The only phase-safe way to tune both channels.
    LoBoth = 2,
    MagPhaseCh2 = 9,
    IpAddress = 10,
}

/// "Set variable 16 bit value" (`0xF5`) variable numbers.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Variable {
    ClockCorrection = 0,
    AttenuatorAdc1 = 1,
    AttenuatorAdc2 = 2,
    AntennaHf1Vhf = 3,
    AntennaHf2 = 4,
    Switch = 5,
    AntennaFreqHf1Vhf = 6,
    AntennaFreqHf2 = 7,
}

/// Bits of the [`Variable::Switch`] register.
pub const SW_ADC2_CLK_INVERTED: u8 = 1 << 0;
/// Clear: ADC1 to HF1.
pub const SW_ADC1_TO_VHF: u8 = 1 << 1;
/// Clear: ADC2 parallel with ADC1.
pub const SW_ADC2_TO_HF2: u8 = 1 << 2;
pub const SW_REMOTE_PWR_CH1: u8 = 1 << 3;
/// Set: control signalling, clear: plain +12 V.
pub const SW_REMOTE_CTRL_CH1: u8 = 1 << 4;
pub const SW_REMOTE_PWR_CH2: u8 = 1 << 5;
pub const SW_REMOTE_CTRL_CH2: u8 = 1 << 6;
pub const SW_VHF_PREAMP: u8 = 1 << 7;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamFormat {
    /// 1 or 2.
    pub channels: usize,
    /// 16 or 24.
    pub bits: usize,
}

/// Decimation is `2^(exp+1)` for `exp` in `0..=5`, so 2 to 64. Sample rate =
/// ADC clock / rate.
pub fn decimation_rate(exp: i32) -> u32 {
    1 << (exp.clamp(0, 5) + 1)
}

pub fn decimation_exponent_for(rate: u32) -> i32 {
    for e in 0..=5 {
        if decimation_rate(e) == rate {
            return e;
        }
    }
    -1
}

// ---------------------------------------------------------------------------
// LAN block geometry
//
// Blocks carry a fixed 130560 samples, so the byte length varies with the
// format -- except at 24 bit, where one and two channel blocks are the same
// length and the dual channel case therefore carries only half as many
// samples per channel.
// ---------------------------------------------------------------------------

pub const LAN_SAMPLES_PER_BLOCK: usize = 130560;
pub const UDP_PACKET_BYTES: usize = 1458; // 2 byte index + 1456 payload
pub const UDP_PAYLOAD_BYTES: usize = 1456;
pub const LAN_TCP_PORT: u16 = 55557;
pub const LAN_UDP_PORT: u16 = 55558;

/// Little-endian on the wire: `78 56 34 12 F0 DE BC 9A`.
pub const SYNC_BYTES: [u8; 8] = [0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockLayout {
    pub format: StreamFormat,
    pub samples_per_channel: usize,
    /// One sample across all channels.
    pub bytes_per_frame: usize,
    pub iq_bytes: usize,
    pub block_bytes: usize,

    pub counter_offset: usize,
    pub inv_counter_offset: usize,
    pub sync_offset: usize,
    pub temp_offset: usize,
    pub gps_offset: usize,
    pub cmd_no_offset: usize,
    pub cmd_count_offset: usize,
    pub commands_offset: usize,
    pub command_space: usize,

    pub start_stream_size_code: u8,
    pub udp_packets: usize,
}

pub fn lan_layout(fmt: StreamFormat) -> BlockLayout {
    let bytes_per_frame = (fmt.bits / 8) * 2 * fmt.channels;

    // 24 bit blocks are a fixed length regardless of channel count, so dual
    // channel halves the samples rather than doubling the block.
    let samples_per_channel = if fmt.bits == 24 {
        if fmt.channels == 2 { LAN_SAMPLES_PER_BLOCK / 2 } else { LAN_SAMPLES_PER_BLOCK }
    } else {
        LAN_SAMPLES_PER_BLOCK
    };
    let iq_bytes = samples_per_channel * bytes_per_frame;

    let counter_offset = iq_bytes;
    let inv_counter_offset = counter_offset + 4;
    let sync_offset = inv_counter_offset + 4;
    let temp_offset = sync_offset + 8;
    let gps_offset = temp_offset + 1;
    let cmd_no_offset = gps_offset + 2;
    let cmd_count_offset = cmd_no_offset + 1;
    let commands_offset = cmd_count_offset + 4;

    // Documented block lengths. They are all exact multiples of the UDP
    // payload size, which is what makes fragmentation come out even.
    let (block_bytes, start_stream_size_code): (usize, u8) =
        if fmt.bits == 16 && fmt.channels == 1 {
            (522704, 7)
        } else if fmt.bits == 16 && fmt.channels == 2 {
            (1045408, 15)
        } else if fmt.bits == 24 && fmt.channels == 2 {
            (784784, 5)
        } else {
            // "All other values" mean 1 channel 24 bit.
            (784784, 0)
        };

    let command_space = block_bytes - commands_offset;
    let udp_packets = block_bytes.div_ceil(UDP_PAYLOAD_BYTES);

    BlockLayout {
        format: fmt,
        samples_per_channel,
        bytes_per_frame,
        iq_bytes,
        block_bytes,
        counter_offset,
        inv_counter_offset,
        sync_offset,
        temp_offset,
        gps_offset,
        cmd_no_offset,
        cmd_count_offset,
        commands_offset,
        command_space,
        start_stream_size_code,
        udp_packets,
    }
}

// ---------------------------------------------------------------------------
// USB packet geometry: a fixed 4096 bytes carrying exactly one command.
// ---------------------------------------------------------------------------

pub const USB_PACKET_BYTES: usize = 4096;
pub const USB_IQ_OFFSET: usize = 4;
pub const USB_IQ_BYTES: usize = 4080;
pub const USB_TEMP_OFFSET: usize = 4084;
pub const USB_GPS_OFFSET: usize = 4085;
pub const USB_CMD_NO_OFFSET: usize = 4087;
pub const USB_COMMAND_OFFSET: usize = 4088;
pub const USB_COMMAND_BYTES: usize = 8;
pub const USB_ENDPOINT_IN: u8 = 0x82;
pub const USB_ENDPOINT_OUT: u8 = 0x02;

pub fn usb_samples_per_packet(fmt: StreamFormat) -> usize {
    USB_IQ_BYTES / ((fmt.bits / 8) * 2 * fmt.channels)
}

// ---------------------------------------------------------------------------
// Status header (temperature / GPS correction / overload), shared by both
// framings
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Status {
    /// Temperature reads `0x80` while the attenuator is in.
    pub auto_att_active: bool,
    /// Meaningless when `auto_att_active`.
    pub temperature_c: i32,
    pub freq_correction_valid: bool,
    /// Signed 14 bit.
    pub freq_correction_raw: i32,
    pub overload_ch1: bool,
    pub overload_ch2: bool,
}

/// Frequency correction resolution depends on whether the radio is
/// disciplining its own clock: 0.5 Hz per LSB when it is, 0.1 Hz when it is
/// only measuring.
pub fn freq_correction_hz(s: &Status, internal_control_on: bool) -> f64 {
    f64::from(s.freq_correction_raw) * if internal_control_on { 0.5 } else { 0.1 }
}

pub fn parse_status(temp: u8, gps_lo: u8, gps_hi: u8) -> Status {
    let mut s = Status::default();

    // 0x80 in the temperature byte is the Auto-ATT indicator, not -128
    // degrees.
    s.auto_att_active = temp == 0x80;
    s.temperature_c = if s.auto_att_active { 0 } else { i32::from(temp as i8) };

    s.overload_ch1 = (gps_hi & 0x40) != 0;
    s.overload_ch2 = (gps_hi & 0x80) != 0;

    let mut raw = i32::from(gps_lo) | ((i32::from(gps_hi) & 0x3F) << 8);
    if raw & 0x2000 != 0 {
        raw |= !0x3FFF; // sign extend from 14 bits
    }

    // 0x2000 is the largest negative value and means "no valid measurement",
    // either because GPS is not being received or because the frequency has
    // just changed.
    s.freq_correction_valid = raw != -8192;
    s.freq_correction_raw = raw;
    s
}

// ---------------------------------------------------------------------------
// Sample unpacking
//
// Outputs interleaved (re, im) float pairs. `out_b` is only touched for a
// dual channel format.
// ---------------------------------------------------------------------------

/// Enabling Auto-ATT scales the whole data stream down by 2 bits to make
/// headroom, so everything downstream has to be scaled back up by the same
/// amount.
pub const AUTO_ATT_GAIN: f32 = 4.0;

pub fn full_scale_for(bits: usize) -> f32 {
    if bits == 16 { 32768.0 } else { 8_388_608.0 }
}

/// 24-bit sign extension across the three-byte boundary: shift the bytes
/// into the *top* of a 32-bit word, then an arithmetic right shift by 8
/// sign-extends in the same step that repositions the value.
#[inline]
pub fn read24(p: &[u8]) -> i32 {
    let v = (u32::from(p[0]) << 8) | (u32::from(p[1]) << 16) | (u32::from(p[2]) << 24);
    (v as i32) >> 8
}

#[inline]
pub fn read16(p: &[u8]) -> i16 {
    i16::from_le_bytes([p[0], p[1]])
}

/// Returns the number of samples written per channel. `gain_a`/`gain_b` are
/// separate — not one shared gain — because Auto-ATT's own engaged-state
/// compensation is a *per-channel* calibration factor (DP: "the gain values
/// can be used to compensate the attenuator's attenuation").
pub fn unpack(
    iq: &[u8],
    frames: usize,
    fmt: StreamFormat,
    gain_a: f32,
    gain_b: f32,
    out_a: &mut [f32],
    out_b: &mut [f32],
) -> usize {
    let scale_a = gain_a / full_scale_for(fmt.bits);
    let scale_b = gain_b / full_scale_for(fmt.bits);
    let step = (fmt.bits / 8) * 2; // one complex sample of one channel

    if fmt.channels == 1 {
        if fmt.bits == 16 {
            for i in 0..frames {
                let p = &iq[i * step..];
                out_a[2 * i] = f32::from(read16(p)) * scale_a;
                out_a[2 * i + 1] = f32::from(read16(&p[2..])) * scale_a;
            }
        } else {
            for i in 0..frames {
                let p = &iq[i * step..];
                out_a[2 * i] = read24(p) as f32 * scale_a;
                out_a[2 * i + 1] = read24(&p[3..]) as f32 * scale_a;
            }
        }
        return frames;
    }

    // Dual channel is interleaved per sample: I1 Q1 I2 Q2.
    if fmt.bits == 16 {
        for i in 0..frames {
            let p = &iq[i * step * 2..];
            out_a[2 * i] = f32::from(read16(p)) * scale_a;
            out_a[2 * i + 1] = f32::from(read16(&p[2..])) * scale_a;
            out_b[2 * i] = f32::from(read16(&p[4..])) * scale_b;
            out_b[2 * i + 1] = f32::from(read16(&p[6..])) * scale_b;
        }
    } else {
        for i in 0..frames {
            let p = &iq[i * step * 2..];
            out_a[2 * i] = read24(p) as f32 * scale_a;
            out_a[2 * i + 1] = read24(&p[3..]) as f32 * scale_a;
            out_b[2 * i] = read24(&p[6..]) as f32 * scale_b;
            out_b[2 * i + 1] = read24(&p[9..]) as f32 * scale_b;
        }
    }
    frames
}

// ---------------------------------------------------------------------------
// LAN block validation and resynchronisation
// ---------------------------------------------------------------------------

pub fn read_u32(p: &[u8]) -> u32 {
    u32::from_le_bytes([p[0], p[1], p[2], p[3]])
}

pub fn write_u32(p: &mut [u8], v: u32) {
    p[0..4].copy_from_slice(&v.to_le_bytes());
}

/// DP 3.2/3.3 describe the firmware version field as "4 digit hexadecimal
/// value firmware version" — that's packed BCD (each nibble is one decimal
/// digit), not a plain integer. Verified live: a real unit reports the raw
/// 32-bit field as `0x0225`, which read straight (binary) is decimal 549 —
/// but BCD-decoded (nibbles 5, 2, 2, 0 from the LSB, i.e. digits "0225") it's
/// 225, the version this project's protocol documentation
/// (`RSR200_DP_ENG_V52.pdf`) was itself written against. Generic over any
/// number of packed digits, so it doesn't assume firmware versions stay 3
/// digits forever.
pub fn bcd_to_decimal(bcd: u32) -> u32 {
    let mut result = 0u32;
    let mut multiplier = 1u32;
    let mut bcd = bcd;
    while bcd != 0 {
        result += (bcd & 0xF) * multiplier;
        multiplier *= 10;
        bcd >>= 4;
    }
    result
}

/// A block is credible when the sync words are present and the counter
/// matches its own ones' complement. Both checks together make a false
/// positive very unlikely, which is what lets a receiver find block
/// boundaries in an arbitrary byte stream.
pub fn block_trailer_valid(block: &[u8], l: &BlockLayout) -> bool {
    if block.len() < l.sync_offset + SYNC_BYTES.len() {
        return false;
    }
    if block[l.sync_offset..l.sync_offset + SYNC_BYTES.len()] != SYNC_BYTES {
        return false;
    }
    let c = read_u32(&block[l.counter_offset..]);
    let inv = read_u32(&block[l.inv_counter_offset..]);
    (c ^ inv) == 0xFFFF_FFFF
}

/// Offset of the first complete, valid block in `data`, or `None`. Scans for
/// the sync words rather than trying every offset, then validates the
/// candidate.
pub fn find_block_start(data: &[u8], l: &BlockLayout) -> Option<usize> {
    if data.len() < l.block_bytes {
        return None;
    }
    let mut p = l.sync_offset;
    while p + SYNC_BYTES.len() <= data.len() {
        if data[p] == SYNC_BYTES[0] && data[p..p + SYNC_BYTES.len()] == SYNC_BYTES {
            let start = p - l.sync_offset;
            if start + l.block_bytes <= data.len() && block_trailer_valid(&data[start..], l) {
                return Some(start);
            }
        }
        p += 1;
    }
    None
}

// ---------------------------------------------------------------------------
// Commands, PC -> RSR200
//
// Every command starts with a 32-bit number of our choosing, which comes
// back in the acknowledgement. Never send 0: the radio uses 0 to mark
// commands it generated itself.
//
// LAN truncates the unused tail of each command; USB always sends the full
// length.
//
// The "repeat counter" is documented for retries but firmware 22x ignores
// it, so a retry should be a fresh command number rather than a repeat.
// ---------------------------------------------------------------------------

pub mod instr {
    pub const ENABLE_FW_UPDATE: u8 = 0x0B;
    pub const START_FW_UPDATE: u8 = 0x0C;
    pub const FW_DATA: u8 = 0x0D;
    pub const READ_VERSION: u8 = 0x12;
    pub const START_STREAM: u8 = 0x15;
    pub const STOP_STREAM: u8 = 0x16;
    pub const SET_GENERATORS: u8 = 0xB0;
    pub const SET_AUTO_ATT: u8 = 0xB1;
    pub const RESET: u8 = 0xB2;
    pub const SET_DATA_TRANSMISSION: u8 = 0xB4;
    pub const SET_ADC_CLOCK: u8 = 0xF2;
    pub const SET_VARIABLE: u8 = 0xF5;
}

/// `usb_len`/`lan_len` are the documented sizes; this emits whichever
/// applies.
pub fn make_command(
    number: u32,
    instruction: u8,
    params: &[u8],
    usb_len: usize,
    lan_len: usize,
    lan: bool,
) -> Vec<u8> {
    let mut c = vec![0u8; if lan { lan_len } else { usb_len }];
    c[0..4].copy_from_slice(&number.to_le_bytes());
    c[4] = instruction;
    for (i, &b) in params.iter().enumerate() {
        if 5 + i < c.len() {
            c[5 + i] = b;
        }
    }
    c
}

pub fn cmd_reset(no: u32, lan: bool) -> Vec<u8> {
    make_command(no, instr::RESET, &[], 8, 8, lan)
}

pub fn cmd_read_version(no: u32, lan: bool, repeat: u8) -> Vec<u8> {
    make_command(no, instr::READ_VERSION, &[repeat], 8, 6, lan)
}

pub fn cmd_start_stream(no: u32, lan: bool, port: StreamPort, size_code: u8) -> Vec<u8> {
    make_command(no, instr::START_STREAM, &[port as u8, size_code], 8, 7, lan)
}

pub fn cmd_stop_stream(no: u32, lan: bool, port: StreamPort, repeat: u8) -> Vec<u8> {
    make_command(no, instr::STOP_STREAM, &[port as u8, repeat], 8, 7, lan)
}

/// `clock_hz` is rounded to the radio's 0.1 MHz step. RSR200B accepts
/// 70..200 MHz.
pub fn cmd_set_adc_clock(
    no: u32,
    lan: bool,
    clock_hz: f64,
    gps_discipline_on: bool,
    repeat: u8,
) -> Vec<u8> {
    let units = (clock_hz / 100_000.0).round() as i32; // 0.1 MHz per unit
    let units = units.clamp(700, 2000);
    let lsb = (units & 0xFF) as u8;
    let mut msb = ((units >> 8) & 0x7F) as u8;
    if !gps_discipline_on {
        msb |= 0x80; // bit 7 is "GPS-Dis"
    }
    make_command(no, instr::SET_ADC_CLOCK, &[lsb, msb, repeat], 8, 8, lan)
}

pub fn cmd_set_generator(
    no: u32,
    lan: bool,
    select: GenSelect,
    value: u32,
    repeat: u8,
) -> Vec<u8> {
    let mut p = Vec::with_capacity(6);
    p.push(select as u8);
    p.extend_from_slice(&value.to_le_bytes());
    p.push(repeat);
    make_command(no, instr::SET_GENERATORS, &p, 12, 11, lan)
}

/// Tuning both channels with one command is the only way to keep them phase
/// locked.
pub fn cmd_set_lo_both(no: u32, lan: bool, lo_hz: f64) -> Vec<u8> {
    cmd_set_generator(no, lan, GenSelect::LoBoth, lo_hz.round() as i32 as u32, 0)
}

/// Hardware diversity weight: magnitude 1/8192 per LSB, phase spanning
/// ±180 degrees.
pub fn pack_magnitude_phase(magnitude: f64, phase_degrees: f64) -> u32 {
    let mag = (magnitude.clamp(0.0, 7.9999) * 8192.0).round() as i32;
    let mag = mag.clamp(0, 65535) as u16;
    // 0x8000 is -180 degrees and 0x7FFF is +180 minus one LSB, so a full
    // circle spans 65536 steps. Clamp after scaling, not before: clamping
    // the angle to 179.99 first lands a step short of the top of the range.
    let phi = (phase_degrees / 360.0 * 65536.0).round() as i32;
    let phi = phi.clamp(-32768, 32767) as i16;
    (u32::from(phi as u16) << 16) | u32::from(mag)
}

// -----------------------------------------------------------------------------
// Handing a software-derived combination to the hardware combiner
//
// In Diversity mode the radio computes Y = A + g*B, with g the magnitude and
// phase set for channel 2. That is enough to null one arrival, and it costs
// no PC time and half the data rate -- but the weight cannot be *found* in
// that mode, because the radio returns only the combined result. So the
// workflow is necessarily two-step: solve in Separate mode with both
// channels available, then switch to Diversity and hand the answer over.
//
// Note the sign convention. The hardware *adds*, while a subtractive phaser
// weight (Y = A - wB) would need g = -w. An additive coefficient pair
// (y = k0*A + k1*B) -- what `sdroxide_dsp`'s decorrelator produces, per
// `Diversity::decorrelated_weight()` -- converts directly as g = k1/k0.
// -----------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HardwareWeight {
    /// 0.001..8, the radio's expressible range.
    pub magnitude: f64,
    pub phase_degrees: f64,
    /// `false` when the ratio falls outside that range.
    pub representable: bool,
    /// `true` when swapping the channels would bring it inside.
    pub suggest_swap: bool,
}

impl Default for HardwareWeight {
    fn default() -> Self {
        HardwareWeight { magnitude: 1.0, phase_degrees: 0.0, representable: false, suggest_swap: false }
    }
}

/// The radio's magnitude spans 0 to just under 8 (16 bits at 1/8192 per
/// LSB), so a combination needing more than 8x on channel 2 cannot be
/// expressed. Swapping the channels inverts the ratio and usually brings it
/// back into range, which the port mode can do with a single bit.
pub fn hardware_weight_for(k0: Complex64, k1: Complex64) -> HardwareWeight {
    if k0.norm() < 1e-30 {
        // Channel A contributes nothing: not expressible as A + g*B at any
        // gain.
        return HardwareWeight { suggest_swap: true, ..Default::default() };
    }

    let g = k1 / k0;
    let mag = g.norm();

    HardwareWeight {
        magnitude: mag,
        phase_degrees: g.arg() * 180.0 / std::f64::consts::PI,
        representable: (0.001..8.0).contains(&mag),
        suggest_swap: mag >= 8.0,
    }
}

pub fn cmd_set_variable(no: u32, lan: bool, v: Variable, value: u16, repeat: u8) -> Vec<u8> {
    let p = [v as u8, (value & 0xFF) as u8, (value >> 8) as u8, repeat];
    make_command(no, instr::SET_VARIABLE, &p, 12, 9, lan)
}

pub fn port_mode_byte(decimation_exp: i32, dual_channel: bool, bits16: bool, swap_channels: bool) -> u8 {
    let mut b = (decimation_exp.clamp(0, 5) as u8) & 0x07;
    if swap_channels {
        b |= 1 << 3;
    }
    if dual_channel {
        b |= 1 << 4;
    }
    if bits16 {
        b |= 1 << 5;
    }
    b
}

pub fn dsp_mode_byte(mode: OpMode, upper_sideband: bool) -> u8 {
    let mut b = (mode as u8) & 0x03;
    if upper_sideband {
        b |= 1 << 3;
    }
    b
}

pub fn cmd_set_data_transmission(
    no: u32,
    lan: bool,
    iface: Interface,
    port_mode: u8,
    dsp_mode: u8,
    repeat: u8,
) -> Vec<u8> {
    let p = [iface as u8, port_mode, dsp_mode, repeat];
    make_command(no, instr::SET_DATA_TRANSMISSION, &p, 12, 9, lan)
}

/// The command's own gain fields are raw 1/1024-LSB counts (DP's own worked
/// value: nominal 16dB attenuation = 6.3096x = set value 6461), but that's
/// not a number anyone would want to type in or reason about — callers work
/// in the plain multiplier instead and this is the one place it becomes wire
/// bytes. A device-to-device calibration knob (DP: "There are device
/// tolerances that can be compensated for with appropriate values
/// (calibrate!)"), not something with a fixed "correct" value — so no
/// assertion here on how far from 6.3096 it's allowed to drift, only the
/// field's own 16-bit wire range.
pub fn auto_att_gain_lsb(multiplier: f64) -> u16 {
    (multiplier * 1024.0).round().clamp(0.0, 65535.0) as u16
}

/// Hold time is documented in raw ADC clock cycles (`0..0xFFFFFF`, 24 bits),
/// not seconds — "the hold time must be reloaded each time the ADC clock
/// frequency is changed." Seconds is what a person actually reasons about,
/// so that's the unit everywhere except the wire itself; this is the one
/// conversion point, mirroring [`auto_att_gain_lsb`] just above.
pub fn auto_att_hold_time_clocks(seconds: f64, adc_clock_hz: f64) -> u32 {
    (seconds * adc_clock_hz).round().clamp(0.0, 0xFF_FFFF as f64) as u32
}

pub fn cmd_set_auto_attenuator(
    no: u32,
    lan: bool,
    threshold: u8,
    hold_time_clocks: u32,
    gain_ch1: u16,
    gain_ch2: u16,
    repeat: u8,
) -> Vec<u8> {
    let p = [
        threshold,
        (hold_time_clocks & 0xFF) as u8,
        ((hold_time_clocks >> 8) & 0xFF) as u8,
        ((hold_time_clocks >> 16) & 0xFF) as u8,
        (gain_ch1 & 0xFF) as u8,
        (gain_ch1 >> 8) as u8,
        (gain_ch2 & 0xFF) as u8,
        (gain_ch2 >> 8) as u8,
        repeat,
    ];
    make_command(no, instr::SET_AUTO_ATT, &p, 16, 14, lan)
}

// ---------------------------------------------------------------------------
// Replies, RSR200 -> PC
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReplyKind {
    #[default]
    None,
    /// Command executed, nothing to report.
    Confirmation,
    /// Command executed, with feedback data.
    Special,
    /// Serial number and firmware version.
    Version,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Reply {
    pub kind: ReplyKind,
    /// 0 also means "self-generated by the radio".
    pub confirmed_command: u32,
    pub echoed_instruction: u8,
    pub data: [u8; 3],
    pub serial: u32,
    pub firmware: u32,
    pub self_generated: bool,
}

/// An 8-byte command as embedded in a USB packet or a LAN block.
pub fn parse_embedded_command(c: &[u8]) -> Reply {
    if c[0] == instr::READ_VERSION {
        return Reply {
            kind: ReplyKind::Version,
            serial: u32::from(c[1]) | (u32::from(c[2]) << 8) | (u32::from(c[3]) << 16),
            firmware: bcd_to_decimal(read_u32(&c[4..])),
            ..Default::default()
        };
    }
    let confirmed_command = read_u32(&c[4..]);
    let self_generated = confirmed_command == 0;
    if c[0] == 0 && c[1] == 0 && c[2] == 0 && c[3] == 0 {
        Reply { kind: ReplyKind::Confirmation, confirmed_command, self_generated, ..Default::default() }
    } else {
        Reply {
            kind: ReplyKind::Special,
            confirmed_command,
            self_generated,
            echoed_instruction: c[0],
            data: [c[1], c[2], c[3]],
            ..Default::default()
        }
    }
}

/// The 12-byte standalone packet the radio sends over LAN when not
/// streaming. `None` for a wrong length or an unrecognised header — an
/// `Option` here rather than the C++ original's out-parameter-plus-`bool`.
pub fn parse_lan_version_packet(p: &[u8]) -> Option<Reply> {
    if p.len() < 12 || read_u32(p) != 12 || p[4] != instr::READ_VERSION {
        return None;
    }
    Some(Reply {
        kind: ReplyKind::Version,
        serial: u32::from(p[5]) | (u32::from(p[6]) << 8) | (u32::from(p[7]) << 16),
        firmware: bcd_to_decimal(read_u32(&p[8..])),
        ..Default::default()
    })
}

// ---------------------------------------------------------------------------
// Tuning and Nyquist zones
//
// The ADC digitises everything; anything above half the clock folds back.
// So a wanted RF frequency does not map directly onto the mixer setting --
// it has to be reduced to its position within the digitised baseband, and
// every even zone arrives with its spectrum reversed.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tuning {
    /// 1-based Nyquist zone.
    pub zone: i32,
    /// Mixer setting, within `0..adc_clock/2`.
    pub lo_hz: f64,
    pub spectrum_inverted: bool,
    /// Nearest image from the zone below.
    pub alias_below_hz: f64,
    /// Nearest image from the zone above.
    pub alias_above_hz: f64,
}

pub fn tune_for(rf_hz: f64, adc_clock_hz: f64) -> Tuning {
    let half = adc_clock_hz * 0.5;
    if half <= 0.0 {
        return Tuning { zone: 1, lo_hz: 0.0, spectrum_inverted: false, alias_below_hz: 0.0, alias_above_hz: 0.0 };
    }

    let mut zone = (rf_hz / half).floor() as i32 + 1;
    if zone < 1 {
        zone = 1;
    }

    let (lo_hz, spectrum_inverted) = if zone % 2 == 1 {
        (rf_hz - f64::from(zone - 1) * half, false)
    } else {
        (f64::from(zone) * half - rf_hz, true)
    };

    // Frequencies that land on the same baseband position from the
    // neighbouring zones, i.e. what an inadequate anti-alias filter will
    // let through on top of the signal.
    let (mut alias_below_hz, alias_above_hz) = if zone % 2 == 0 {
        (f64::from(zone - 1) * half + lo_hz, f64::from(zone) * half - lo_hz + half)
    } else {
        (f64::from(zone - 1) * half - lo_hz, f64::from(zone) * half + lo_hz)
    };
    if alias_below_hz < 0.0 {
        alias_below_hz = 0.0;
    }

    Tuning { zone, lo_hz, spectrum_inverted, alias_below_hz, alias_above_hz }
}

pub fn sample_rate_hz(adc_clock_hz: f64, decimation_exp: i32) -> f64 {
    adc_clock_hz / f64::from(decimation_rate(decimation_exp))
}

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------
    // LAN block geometry matches the documented lengths
    // -----------------------------------------------------------------

    #[test]
    fn lan_block_bytes_match_the_documented_lengths() {
        let a = lan_layout(StreamFormat { channels: 1, bits: 16 });
        let b = lan_layout(StreamFormat { channels: 2, bits: 16 });
        let c = lan_layout(StreamFormat { channels: 1, bits: 24 });
        let d = lan_layout(StreamFormat { channels: 2, bits: 24 });

        assert_eq!(a.block_bytes, 522704, "1 channel 16 bit block");
        assert_eq!(b.block_bytes, 1045408, "2 channel 16 bit block");
        assert_eq!(c.block_bytes, 784784, "1 channel 24 bit block");
        assert_eq!(d.block_bytes, 784784, "2 channel 24 bit block");

        // DP 2.2.1.4: "At 24 bit, block lengths for both 1 and 2-channel
        // mode are the same. Unlike all other formats, 24 bit 2-channel
        // mode therefore transmits only half the usual number of samples
        // per block."
        assert_eq!(a.samples_per_channel, 130560, "1x16 carries 130560 samples");
        assert_eq!(b.samples_per_channel, 130560, "2x16 carries 130560 samples per channel");
        assert_eq!(c.samples_per_channel, 130560, "1x24 carries 130560 samples");
        assert_eq!(d.samples_per_channel, 65280, "2x24 carries only 65280 samples per channel");

        assert_eq!((a.counter_offset, a.commands_offset), (522240, 522264));
        assert_eq!((b.counter_offset, b.commands_offset), (1044480, 1044504));
        assert_eq!((c.counter_offset, c.commands_offset), (783360, 783384));
        assert_eq!(d.counter_offset, 783360);

        assert_eq!(a.start_stream_size_code, 7);
        assert_eq!(b.start_stream_size_code, 15);
        assert_eq!(d.start_stream_size_code, 5);
    }

    #[test]
    fn block_lengths_divide_evenly_into_udp_packets() {
        // DP 2.2.2.1 gives 359 packets for the 1x16 block. That every block
        // length is an exact multiple of the 1456-byte payload is clearly
        // deliberate, and worth pinning down: a remainder would mean a
        // short final packet to handle.
        for fmt in [
            StreamFormat { channels: 1, bits: 16 },
            StreamFormat { channels: 2, bits: 16 },
            StreamFormat { channels: 1, bits: 24 },
            StreamFormat { channels: 2, bits: 24 },
        ] {
            let l = lan_layout(fmt);
            assert_eq!(l.block_bytes % UDP_PAYLOAD_BYTES, 0, "{fmt:?} divides into whole packets");
        }
        assert_eq!(
            lan_layout(StreamFormat { channels: 1, bits: 16 }).udp_packets,
            359,
            "1x16 block is 359 UDP packets, as documented"
        );
    }

    // -----------------------------------------------------------------
    // USB packet geometry
    // -----------------------------------------------------------------

    #[test]
    fn usb_packet_geometry_matches_the_documented_sample_counts() {
        // DP 2.1.1 - 2.1.4 give these sample counts explicitly.
        assert_eq!(usb_samples_per_packet(StreamFormat { channels: 1, bits: 16 }), 1020);
        assert_eq!(usb_samples_per_packet(StreamFormat { channels: 2, bits: 16 }), 510);
        assert_eq!(usb_samples_per_packet(StreamFormat { channels: 1, bits: 24 }), 680);
        assert_eq!(usb_samples_per_packet(StreamFormat { channels: 2, bits: 24 }), 340);
    }

    // -----------------------------------------------------------------
    // Status header
    // -----------------------------------------------------------------

    #[test]
    fn status_header_decodes_temperature_overload_and_frequency_correction() {
        let s = parse_status(25, 0x00, 0x00);
        assert!(s.temperature_c == 25 && !s.auto_att_active, "an ordinary temperature reads through");

        // DP 3.2: "the reading -128 C (0x80) is reserved to indicate that
        // the attenuator is active. This value must not be used as a
        // temperature reading."
        let s = parse_status(0x80, 0x00, 0x00);
        assert!(s.auto_att_active, "0x80 is the Auto-ATT flag, not a temperature");

        let s = parse_status(0, 0x00, 0xC0);
        assert!(s.overload_ch1 && s.overload_ch2, "overload bits are 6 and 7 of the GPS high byte");

        // Signed 14-bit, so 0x3FFF is -1.
        let s = parse_status(0, 0xFF, 0x3F);
        assert_eq!(s.freq_correction_raw, -1, "frequency correction is a signed 14 bit value");

        let s = parse_status(0, 0x64, 0x00);
        assert_eq!(s.freq_correction_raw, 100, "positive corrections read correctly");
        assert_eq!(freq_correction_hz(&s, true), 50.0, "0.5 Hz per LSB while disciplining");
        assert_eq!(freq_correction_hz(&s, false), 10.0, "0.1 Hz per LSB while only measuring");

        // DP 3.2: "If GPS reception is not possible, the highest possible
        // negative value is output (0x2000)."
        let s = parse_status(0, 0x00, 0x20);
        assert!(!s.freq_correction_valid, "0x2000 means no valid measurement");

        // Overload bits must not corrupt the correction value.
        let s = parse_status(0, 0x64, 0xC0);
        assert!(
            s.freq_correction_raw == 100 && s.overload_ch1 && s.overload_ch2,
            "overload bits are masked out of the correction"
        );
    }

    // -----------------------------------------------------------------
    // Sample unpacking
    // -----------------------------------------------------------------

    #[test]
    fn sample_unpacking_scales_sign_extends_and_keeps_channels_independent() {
        // 16 bit, single channel: full scale positive and negative.
        let iq16 = [0x00u8, 0x40, 0x00, 0xC0]; // +16384, -16384
        let mut a = [0.0f32; 4];
        let mut b = [0.0f32; 4];
        unpack(&iq16, 1, StreamFormat { channels: 1, bits: 16 }, 1.0, 1.0, &mut a, &mut b);
        assert!((a[0] - 0.5).abs() < 1e-6 && (a[1] + 0.5).abs() < 1e-6, "16 bit scales to +/-1.0 at full scale");

        // 24 bit sign extension across the three-byte boundary.
        let iq24 = [0x00u8, 0x00, 0x40, 0x00, 0x00, 0xC0];
        unpack(&iq24, 1, StreamFormat { channels: 1, bits: 24 }, 1.0, 1.0, &mut a, &mut b);
        assert!((a[0] - 0.5).abs() < 1e-6 && (a[1] + 0.5).abs() < 1e-6, "24 bit sign extends and scales correctly");

        // Dual channel interleave is I1 Q1 I2 Q2, and must not get crossed.
        let dual16 = [0x00u8, 0x40, 0x00, 0x20, 0x00, 0xC0, 0x00, 0xE0];
        unpack(&dual16, 1, StreamFormat { channels: 2, bits: 16 }, 1.0, 1.0, &mut a, &mut b);
        assert!((a[0] - 0.5).abs() < 1e-6 && (a[1] - 0.25).abs() < 1e-6, "channel 1 lands in A");
        assert!((b[0] + 0.5).abs() < 1e-6 && (b[1] + 0.25).abs() < 1e-6, "channel 2 lands in B");

        // DP 4.7: enabling Auto-ATT scales the stream down 2 bits, so it
        // has to be scaled back up or every level downstream is 12 dB
        // wrong.
        unpack(&iq16, 1, StreamFormat { channels: 1, bits: 16 }, AUTO_ATT_GAIN, AUTO_ATT_GAIN, &mut a, &mut b);
        assert!((a[0] - 2.0).abs() < 1e-5, "Auto-ATT compensation is a factor of 4");

        // Auto-ATT's per-channel calibration gain is genuinely different
        // per channel (device tolerances, DP: "calibrate!") -- gain_a and
        // gain_b must be applied independently, not averaged or
        // cross-applied.
        unpack(&dual16, 1, StreamFormat { channels: 2, bits: 16 }, 2.0, 3.0, &mut a, &mut b);
        assert!(
            (a[0] - 1.0).abs() < 1e-6 && (b[0] + 1.5).abs() < 1e-6,
            "channel A and channel B take their own, independent gain"
        );
    }

    // -----------------------------------------------------------------
    // Block resynchronisation
    // -----------------------------------------------------------------

    #[test]
    fn block_resync_finds_a_boundary_and_rejects_a_corrupted_counter() {
        let l = lan_layout(StreamFormat { channels: 1, bits: 16 });
        // A partial block, then a complete one, as a receiver would see
        // mid-stream.
        let junk = 1234usize;
        let mut stream = vec![0xAAu8; junk + l.block_bytes];
        {
            let blk = &mut stream[junk..];
            write_u32(&mut blk[l.counter_offset..], 0x1234_5678);
            write_u32(&mut blk[l.inv_counter_offset..], !0x1234_5678u32);
            blk[l.sync_offset..l.sync_offset + SYNC_BYTES.len()].copy_from_slice(&SYNC_BYTES);
        }

        assert_eq!(
            find_block_start(&stream, &l),
            Some(junk),
            "finds a block boundary in a stream that starts mid-block"
        );
        assert!(block_trailer_valid(&stream[junk..], &l), "a well-formed trailer validates");

        // A corrupted counter must be rejected even though the sync words
        // are intact -- that pairing is what makes a false positive
        // unlikely.
        write_u32(&mut stream[junk + l.inv_counter_offset..], 0);
        assert!(!block_trailer_valid(&stream[junk..], &l), "counter and its inverse must agree");
        assert_eq!(find_block_start(&stream, &l), None, "no false positive on a bad counter");
    }

    // -----------------------------------------------------------------
    // Command construction
    // -----------------------------------------------------------------

    #[test]
    fn command_lengths_match_the_dp_tables() {
        // Lengths, from the tables in DP 3.3.
        assert_eq!((cmd_reset(1, false).len(), cmd_reset(1, true).len()), (8, 8), "Reset is 8 bytes both ways");
        assert_eq!(
            (cmd_read_version(1, false, 0).len(), cmd_read_version(1, true, 0).len()),
            (8, 6),
            "Read version 8 / 6"
        );
        assert_eq!(
            (cmd_start_stream(1, false, StreamPort::Tcp, 7).len(), cmd_start_stream(1, true, StreamPort::Tcp, 7).len()),
            (8, 7),
            "Start stream 8 / 7"
        );
        assert_eq!(
            (
                cmd_set_generator(1, false, GenSelect::LoBoth, 0, 0).len(),
                cmd_set_generator(1, true, GenSelect::LoBoth, 0, 0).len()
            ),
            (12, 11),
            "Set generators 12 / 11"
        );
        assert_eq!(
            (
                cmd_set_variable(1, false, Variable::Switch, 0, 0).len(),
                cmd_set_variable(1, true, Variable::Switch, 0, 0).len()
            ),
            (12, 9),
            "Set variable 12 / 9"
        );
        assert_eq!(
            (
                cmd_set_data_transmission(1, false, Interface::Lan, 0, 0, 0).len(),
                cmd_set_data_transmission(1, true, Interface::Lan, 0, 0, 0).len()
            ),
            (12, 9),
            "Set data transmission 12 / 9"
        );
        assert_eq!(
            (
                cmd_set_auto_attenuator(1, false, 0, 0, 0, 0, 0).len(),
                cmd_set_auto_attenuator(1, true, 0, 0, 0, 0, 0).len()
            ),
            (16, 14),
            "Set auto attenuator 16 / 14"
        );
    }

    #[test]
    fn command_layout_places_number_then_instruction_then_params() {
        let c = cmd_start_stream(0xDEAD_BEEF, true, StreamPort::Tcp, 15);
        assert_eq!(read_u32(&c), 0xDEAD_BEEF, "command number occupies bytes 0-3");
        assert!(
            c[4] == instr::START_STREAM && c[5] == StreamPort::Tcp as u8 && c[6] == 15,
            "instruction and parameters follow"
        );
    }

    #[test]
    fn adc_clock_command_encodes_units_and_gps_discipline_and_clamps() {
        // ADC clock is in 0.1 MHz units with GPS discipline in bit 7 of the
        // high byte.
        let c = cmd_set_adc_clock(1, false, 125_000_000.0, true, 0);
        assert!(c[5] == (1250u32 & 0xFF) as u8 && c[6] == (1250u32 >> 8) as u8, "125 MHz encodes as 1250 units");

        let c = cmd_set_adc_clock(1, false, 125_000_000.0, false, 0);
        assert!((c[6] & 0x80) != 0, "GPS-Dis is bit 7 of the high byte");

        let c = cmd_set_adc_clock(1, false, 300_000_000.0, true, 0);
        assert_eq!(i32::from(c[5]) | ((i32::from(c[6]) & 0x7F) << 8), 2000, "clock clamps to the 200 MHz maximum");
    }

    #[test]
    fn lo_both_command_is_a_32_bit_little_endian_hz_value() {
        let c = cmd_set_lo_both(7, true, 10_000_000.0);
        assert_eq!(c[5], GenSelect::LoBoth as u8, "tuning both channels uses selector 2");
        assert_eq!(read_u32(&c[6..]), 10_000_000, "LO is a 32 bit little endian value in Hz");
    }

    // -----------------------------------------------------------------
    // Port and DSP mode bytes
    // -----------------------------------------------------------------

    #[test]
    fn decimation_spans_the_documented_range() {
        assert!(decimation_rate(0) == 2 && decimation_rate(5) == 64, "decimation spans 2 to 64");
        assert_eq!(decimation_exponent_for(16), 3, "a rate of 16 is exponent 3");
    }

    #[test]
    fn port_and_dsp_mode_bytes_place_their_bits_correctly() {
        assert_eq!(port_mode_byte(3, false, false, false), 0x03, "single channel 24 bit, decimation 16");
        assert!((port_mode_byte(0, true, false, false) & (1 << 4)) != 0, "bit 4 selects dual channel");
        assert!((port_mode_byte(0, false, true, false) & (1 << 5)) != 0, "bit 5 selects 16 bit");
        assert!((port_mode_byte(0, false, false, true) & (1 << 3)) != 0, "bit 3 swaps the channels");

        assert_eq!(dsp_mode_byte(OpMode::Independent, false), 0, "Separate is operating mode 0");
        assert_eq!(dsp_mode_byte(OpMode::Diversity, false), 3, "Diversity is operating mode 3");
        assert!((dsp_mode_byte(OpMode::Serial, true) & (1 << 3)) != 0, "bit 3 picks the upper sideband");
    }

    // -----------------------------------------------------------------
    // Hardware diversity weight packing
    // -----------------------------------------------------------------

    #[test]
    fn magnitude_phase_packing_matches_the_documented_encoding() {
        // DP 3.3: magnitude 1 LSB = 1/8192; phase 0x8000 = -180, 0x7FFF =
        // +180 - 1 LSB.
        let v = pack_magnitude_phase(1.0, 0.0);
        assert_eq!(v & 0xFFFF, 8192, "unity magnitude is 8192");
        assert_eq!(v >> 16, 0, "zero phase is zero");

        let v = pack_magnitude_phase(1.0, 180.0);
        assert_eq!((v >> 16) as i16, 32767, "+180 degrees saturates at 0x7FFF");

        let v = pack_magnitude_phase(1.0, -180.0);
        assert_eq!((v >> 16) as i16, -32768, "-180 degrees is 0x8000");
    }

    // -----------------------------------------------------------------
    // Auto-ATT command encoding and unit conversions
    // -----------------------------------------------------------------

    #[test]
    fn auto_att_unit_conversions_match_the_dp_worked_values() {
        // DP's own worked value: nominal 16dB attenuation = 6.3096x = raw
        // set value 6461.
        assert_eq!(auto_att_gain_lsb(6.3096), 6461, "6.3096x is the DP's own worked value, 6461 raw LSBs");
        assert_eq!(auto_att_gain_lsb(0.0), 0, "zero multiplier is zero LSBs");
        assert_eq!(auto_att_gain_lsb(1000.0), 65535, "clamps to the field's 16 bit range rather than wrapping");

        // "0 ... 0xFFFFFF = 1 ... 2^24 ADC CLK" -- a plain seconds*Hz
        // conversion, clamped to the field's 24 bits, and (per DP's own
        // caution) meant to be re-derived from the *current* ADC clock
        // every time either changes, not carried as a fixed raw count.
        assert_eq!(auto_att_hold_time_clocks(0.05, 125e6), 6_250_000, "0.05s at 125 MHz is 6,250,000 clocks");
        assert_eq!(auto_att_hold_time_clocks(0.0, 125e6), 0, "zero hold time is zero clocks");

        // A real, easy-to-miss consequence of the field only being 24
        // bits: 0xFFFFFF clocks caps out at barely over 134ms at a 125 MHz
        // ADC clock (2^24-1 / 125e6), and less still at higher clock rates
        // -- so a "look and see" hold-time default in the low hundreds of
        // milliseconds, which reads as perfectly reasonable next to the
        // field's *seconds* framing, can silently be requesting more than
        // the wire format can actually carry.
        assert_eq!(
            auto_att_hold_time_clocks(1.0, 125e6),
            0xFF_FFFF,
            "a full second at 125 MHz overflows the 24 bit field and clamps rather than wraps"
        );
        assert_eq!(
            auto_att_hold_time_clocks(0.2, 125e6),
            0xFF_FFFF,
            "even 200ms overflows the field at a 125 MHz clock -- see the comment above"
        );
    }

    #[test]
    fn auto_att_command_places_its_fields_at_the_documented_offsets() {
        // Field layout, DP's own "Set automatic attenuator" table:
        // threshold (byte 5), hold time (24 bit LE, bytes 6-8), channel 1
        // gain (16 bit LE, bytes 9-10), channel 2 gain (16 bit LE, bytes
        // 11-12), repeat (byte 13).
        let c = cmd_set_auto_attenuator(1, false, 3, 0x123456, 6461, 500, 9);
        assert_eq!(c[5], 3, "threshold occupies byte 5");
        assert!(c[6] == 0x56 && c[7] == 0x34 && c[8] == 0x12, "hold time is a 24 bit little endian value in ADC clock cycles");
        assert!(c[9] == (6461u16 & 0xFF) as u8 && c[10] == (6461u16 >> 8) as u8, "channel 1 gain is a 16 bit little endian value");
        assert!(
            c[11] == (500u16 & 0xFF) as u8 && c[12] == (500u16 >> 8) as u8,
            "channel 2 gain is its own, independent 16 bit little endian value"
        );
        assert_eq!(c[13], 9, "repeat counter occupies byte 13");
    }

    // -----------------------------------------------------------------
    // Nyquist zone mapping
    // -----------------------------------------------------------------

    #[test]
    fn nyquist_zone_mapping_matches_the_operating_manual_examples() {
        // OM 5.1 example 1: at a 125 MHz clock, "Frequencies around 125 MHz
        // +/- 30 MHz are mapped to the 0 - 30 MHz range", and 95 MHz
        // appears at 30 MHz.
        let t = tune_for(10e6, 125e6);
        assert!(t.zone == 1 && (t.lo_hz - 10e6).abs() < 1.0 && !t.spectrum_inverted, "10 MHz at a 125 MHz clock is zone 1, straight through");

        let t = tune_for(95e6, 125e6);
        assert!(t.zone == 2 && (t.lo_hz - 30e6).abs() < 1.0, "95 MHz maps to 30 MHz baseband");
        assert!(t.spectrum_inverted, "zone 2 arrives with its spectrum reversed");

        // OM 5.1 example 2: 155 MHz is in the 3rd zone and collides with 95
        // MHz reception, which only works if both land on the same
        // baseband frequency.
        let t = tune_for(155e6, 125e6);
        assert!(t.zone == 3 && (t.lo_hz - 30e6).abs() < 1.0, "155 MHz also maps to 30 MHz");
        assert!(!t.spectrum_inverted, "zone 3 is the right way up again");

        // OM 5.1 example 3: with a 166 MHz clock the 3rd zone is 166 - 249
        // MHz, chosen to cover the 174 - 240 MHz DAB band.
        let t = tune_for(174e6, 166e6);
        assert_eq!(t.zone, 3, "174 MHz sits in zone 3 at a 166 MHz clock");
        let t = tune_for(240e6, 166e6);
        assert_eq!(t.zone, 3, "240 MHz is still in zone 3, so the band fits");

        assert!((sample_rate_hz(70e6, 5) - 1_093_750.0).abs() < 1.0, "70 MHz over 64 is 1.09375 MSp/s");
        assert!((sample_rate_hz(200e6, 0) - 100e6).abs() < 1.0, "200 MHz over 2 is 100 MSp/s");
    }

    // -----------------------------------------------------------------
    // BCD decoding
    // -----------------------------------------------------------------

    #[test]
    fn bcd_decoding_matches_worked_examples() {
        assert_eq!(bcd_to_decimal(0x0225), 225, "0x0225 BCD-decodes to 225, not 549");
        assert_eq!(bcd_to_decimal(0), 0, "zero decodes to zero");
        assert_eq!(bcd_to_decimal(0x9999), 9999, "all-nines round-trips");
        assert_eq!(bcd_to_decimal(0x0007), 7, "single low digit, no leading-zero artefacts");
    }

    // -----------------------------------------------------------------
    // Reply parsing
    // -----------------------------------------------------------------

    #[test]
    fn reply_parsing_covers_confirmation_special_self_generated_and_version() {
        // Plain confirmation: four zero bytes then the command number being
        // confirmed.
        let conf = [0u8, 0, 0, 0, 0x2A, 0x00, 0x00, 0x00];
        let r = parse_embedded_command(&conf);
        assert!(r.kind == ReplyKind::Confirmation && r.confirmed_command == 42, "plain confirmation parses");

        // Special confirmation echoes the instruction and carries three
        // data bytes.
        let spec = [instr::SET_ADC_CLOCK, 0xE2, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00];
        let r = parse_embedded_command(&spec);
        assert!(r.kind == ReplyKind::Special && r.echoed_instruction == instr::SET_ADC_CLOCK, "special confirmation parses");
        assert!(r.data[0] == 0xE2 && r.data[1] == 0x04 && r.confirmed_command == 7, "its feedback data comes through");

        // DP 3.2: "In self-generated commands, the RSR200 always sends
        // number 0 to the PC."
        let self_gen = [instr::SET_ADC_CLOCK, 0xD0, 0x04, 0x00, 0, 0, 0, 0];
        let r = parse_embedded_command(&self_gen);
        assert!(r.self_generated, "command number 0 marks a self-generated report");

        // DP 3.2/3.3 describe the firmware field as a "4 digit hexadecimal
        // value" -- packed BCD -- not a plain binary count.
        let ver = [instr::READ_VERSION, 0x34, 0x12, 0x00, 0x25, 0x02, 0x00, 0x00];
        let r = parse_embedded_command(&ver);
        assert!(
            r.kind == ReplyKind::Version && r.serial == 0x1234 && r.firmware == 225,
            "USB version report parses (BCD-decoded)"
        );

        // The 12-byte standalone LAN version packet, which is the only
        // reply sent outside the stream.
        let lanver = [12u8, 0, 0, 0, instr::READ_VERSION, 0x34, 0x12, 0x00, 0x25, 0x02, 0x00, 0x00];
        let r = parse_lan_version_packet(&lanver).expect("well-formed LAN version packet");
        assert!(r.serial == 0x1234 && r.firmware == 225, "LAN version packet parses (BCD-decoded)");

        let mut bad_len = lanver;
        bad_len[0] = 8;
        assert!(parse_lan_version_packet(&bad_len).is_none(), "a wrong length is rejected");
    }

    // -----------------------------------------------------------------
    // Handing a software combination to the hardware combiner
    // -----------------------------------------------------------------

    #[test]
    fn hardware_weight_conversion_round_trips_through_quantisation() {
        // The radio computes Y = A + g*B. A signal whose channel-B copy is
        // r times its channel-A copy is cancelled by g = -1/r.
        let r = Complex64::from_polar(0.7, 2.39); // -3 dB, 137 degrees
        let g = -1.0 / r;

        // An additive coefficient pair that nulls it, as the decorrelator
        // would produce: any scaling of (1, g) is the same combination.
        let k0 = Complex64::from_polar(0.31, 1.1);
        let k1 = k0 * g;

        let h = hardware_weight_for(k0, k1);
        assert!(h.representable, "the ratio is inside the radio's range");
        assert!((h.magnitude - g.norm()).abs() < 1e-9, "magnitude survives the overall scaling");
        assert!(
            (h.phase_degrees - g.arg() * 180.0 / std::f64::consts::PI).abs() < 1e-9,
            "as does phase"
        );

        // The real test: quantise through the wire format, then check the
        // weight the radio would actually apply still cancels.
        let packed = pack_magnitude_phase(h.magnitude, h.phase_degrees);
        let q_mag = f64::from(packed & 0xFFFF) / 8192.0;
        let q_phase = f64::from((packed >> 16) as i16) / 32768.0 * 180.0;
        let qg = Complex64::from_polar(q_mag, q_phase * std::f64::consts::PI / 180.0);
        let residual = 20.0 * (1.0 + qg * r).norm().log10();
        assert!(residual < -60.0, "quantisation is not what limits the null: {residual:.1} dB");

        // Out of range: channel B needs more than 8x, which the 16 bit
        // magnitude cannot express. Swapping the channels inverts the
        // ratio and brings it back.
        let big = hardware_weight_for(Complex64::new(1.0, 0.0), Complex64::new(20.0, 0.0));
        assert!(!big.representable && big.suggest_swap, "too large a ratio asks for a channel swap");
        let swapped = hardware_weight_for(Complex64::new(20.0, 0.0), Complex64::new(1.0, 0.0));
        assert!(swapped.representable, "and swapping brings it back into range");

        // A combination that ignores channel A entirely cannot be written
        // as A + g*B.
        let none = hardware_weight_for(Complex64::new(0.0, 0.0), Complex64::new(1.0, 0.0));
        assert!(!none.representable && none.suggest_swap, "a channel B only combination asks for a swap");

        // The sign convention is the easy thing to get wrong: the hardware
        // adds where a subtractive phaser weight would subtract.
        let hw = hardware_weight_for(Complex64::new(1.0, 0.0), Complex64::new(-0.5, 0.0));
        let additive = Complex64::from_polar(hw.magnitude, hw.phase_degrees * std::f64::consts::PI / 180.0);
        assert!(
            (additive - Complex64::new(-0.5, 0.0)).norm() < 1e-9,
            "a negative coefficient becomes a 180 degree phase, not a negative magnitude"
        );
    }
}
