//! LimeRFE vocabulary: which channel a frequency wants, which connector that
//! channel is reachable on, and the persisted settings for the board.
//!
//! This lives in the types crate rather than beside the driver because the
//! settings panel compiles to wasm for the browser client and shows the
//! operator which channel the current dial resolves to. A rule the UI and the
//! driver both apply has to have exactly one copy, for the same reason
//! [`crate::hackrf_serial_matches`] does.
//!
//! The band table and the two port rules are transcribed from LimeSuite's own
//! `FreqToBand`, `RxPortCheck` and `TxPortCheck` (`src/limeRFE/RFE_Device.cpp`).
//! Deliberately the same numbers rather than better ones: a LimeRFE configured
//! by sdroxide and one configured by LimeSuiteGUI should put the same filters
//! in circuit, and a disagreement would show up as "it works in their software"
//! with nothing to point at.

use serde::{Deserialize, Serialize};

use crate::SerialConfig;

/// The LimeRFE's I²C address on the SDR board's GPIO header.
pub const RFE_I2C_ADDRESS: u8 = 0x51;

/// The serial link's baud rate. Fixed in the board's firmware; not a setting.
pub const RFE_BAUD: u32 = 9600;

/// Every command and reply is this many bytes, always. Not a maximum — short
/// frames are padded, and a reply of any other length is a desynchronised link
/// rather than a small answer.
pub const RFE_BUFFER_SIZE: usize = 16;

/// One of the board's filtered signal paths.
///
/// The discriminants are the wire values (`RFE_CID_*`), so this casts straight
/// into a frame. `NotSelected` (100) is the board's own "nothing chosen" code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum RfeChannel {
    /// 1 MHz – 1 GHz unfiltered. No PA, no LNA — a straight-through path.
    Wb1000 = 1,
    /// 1 – 4 GHz unfiltered.
    Wb4000 = 2,
    /// HF, up to 30 MHz. The only channel that reaches the J5 connector.
    Ham0030 = 3,
    /// 6 m / 4 m.
    Ham0070 = 4,
    /// 2 m.
    Ham0145 = 5,
    /// 1.25 m.
    Ham0220 = 6,
    /// 70 cm.
    Ham0435 = 7,
    /// 33 cm.
    Ham0920 = 8,
    /// 23 cm.
    Ham1280 = 9,
    /// 13 cm.
    Ham2400 = 10,
    /// 9 cm.
    Ham3500 = 11,
    CellBand01 = 12,
    CellBand02 = 13,
    CellBand03 = 14,
    CellBand07 = 15,
    CellBand38 = 16,
    /// The board's power-on state: no path selected at all.
    NotSelected = 100,
}

impl RfeChannel {
    /// Every channel an operator may pick by hand, in the order the combo
    /// offers them. `NotSelected` is not here — it is a state to read back,
    /// never one to ask for.
    pub const ALL: [RfeChannel; 16] = [
        RfeChannel::Wb1000,
        RfeChannel::Wb4000,
        RfeChannel::Ham0030,
        RfeChannel::Ham0070,
        RfeChannel::Ham0145,
        RfeChannel::Ham0220,
        RfeChannel::Ham0435,
        RfeChannel::Ham0920,
        RfeChannel::Ham1280,
        RfeChannel::Ham2400,
        RfeChannel::Ham3500,
        RfeChannel::CellBand01,
        RfeChannel::CellBand02,
        RfeChannel::CellBand03,
        RfeChannel::CellBand07,
        RfeChannel::CellBand38,
    ];

    pub fn label(self) -> &'static str {
        match self {
            RfeChannel::Wb1000 => "Wideband 1 – 1000 MHz",
            RfeChannel::Wb4000 => "Wideband 1 – 4 GHz",
            RfeChannel::Ham0030 => "HF, up to 30 MHz",
            RfeChannel::Ham0070 => "6 m / 4 m (50 – 70 MHz)",
            RfeChannel::Ham0145 => "2 m (140 – 150 MHz)",
            RfeChannel::Ham0220 => "1.25 m (220 – 225 MHz)",
            RfeChannel::Ham0435 => "70 cm (400 – 450 MHz)",
            RfeChannel::Ham0920 => "33 cm (902 – 928 MHz)",
            RfeChannel::Ham1280 => "23 cm (1220 – 1420 MHz)",
            RfeChannel::Ham2400 => "13 cm (2.3 – 2.5 GHz)",
            RfeChannel::Ham3500 => "9 cm (3.3 – 3.7 GHz)",
            RfeChannel::CellBand01 => "Cellular band 1",
            RfeChannel::CellBand02 => "Cellular band 2",
            RfeChannel::CellBand03 => "Cellular band 3",
            RfeChannel::CellBand07 => "Cellular band 7",
            RfeChannel::CellBand38 => "Cellular band 38",
            RfeChannel::NotSelected => "None selected",
        }
    }

    /// The wire code.
    pub fn code(self) -> u8 {
        self as u8
    }

    /// Read a code back off the wire. Anything unrecognised reads as
    /// `NotSelected` rather than failing: this only ever decodes a *report* of
    /// what the board is doing, and a firmware that gains a channel should not
    /// make the state readback error out.
    pub fn from_code(code: u8) -> RfeChannel {
        RfeChannel::ALL.into_iter().find(|c| c.code() == code).unwrap_or(RfeChannel::NotSelected)
    }

    /// Whether this channel has a power amplifier behind it. The two wideband
    /// paths do not — they are filters and relays only — so a transmit through
    /// one comes out at whatever the LimeSDR produced.
    pub fn has_pa(self) -> bool {
        !matches!(self, RfeChannel::Wb1000 | RfeChannel::Wb4000 | RfeChannel::NotSelected)
    }
}

/// A connector on the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(u8)]
pub enum RfePort {
    /// J3, labelled "TX/RX" — the usual receive connector.
    J3 = 1,
    /// J4, labelled "TX" — transmit only.
    J4 = 2,
    /// J5, labelled "30 MHz TX/RX" — the HF connector, and shared between the
    /// two directions, which is what forces mode switching on HF. See
    /// [`LimeRfeConfig::needs_ptt_switching`].
    J5 = 3,
}

impl RfePort {
    pub const RX_PORTS: [RfePort; 2] = [RfePort::J3, RfePort::J5];
    pub const TX_PORTS: [RfePort; 3] = [RfePort::J3, RfePort::J4, RfePort::J5];

    pub fn label(self) -> &'static str {
        match self {
            RfePort::J3 => "J3 (TX/RX)",
            RfePort::J4 => "J4 (TX)",
            RfePort::J5 => "J5 (30 MHz TX/RX)",
        }
    }

    pub fn code(self) -> u8 {
        self as u8
    }
}

/// Which directions the board's relays have enabled — the wire value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[repr(u8)]
pub enum RfeMode {
    /// Receive enabled, transmit disabled.
    #[default]
    Rx = 0,
    /// Transmit enabled, receive disabled.
    Tx = 1,
    /// Both disabled — the board passes nothing.
    None = 2,
    /// Both enabled at once. **Only legal when receive and transmit are on
    /// different connectors**; the board answers `RXTX_SAME_CONN` otherwise.
    TxRx = 3,
}

impl RfeMode {
    pub fn label(self) -> &'static str {
        match self {
            RfeMode::Rx => "Receive",
            RfeMode::Tx => "Transmit",
            RfeMode::None => "Both off",
            RfeMode::TxRx => "Receive + transmit",
        }
    }

    pub fn code(self) -> u8 {
        self as u8
    }
}

/// What the operator asked for, which is not the same question as what the
/// board is doing right now.
///
/// This is one control rather than a "follow PTT" checkbox beside a mode
/// selector, and deliberately so: those two can contradict each other, and the
/// contradiction resolves to a transmitter that produces nothing while the
/// panel says it should be working.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RfeModeControl {
    /// Receive, and transmit only while the operator is keyed.
    ///
    /// [`RfeMode::Rx`] whenever the transmitter is not running — **whatever the
    /// cabling**, and that last part is the correction issue #94 turned on.
    /// `TxRx` looks like the tidier answer on split connectors because it costs
    /// no round trip at key-down, and on this board it is the wrong one: the
    /// amateur channels have a single filter with a transmit/receive switch
    /// either side of it (`RFE_MCU_BYTE_TXRX0_BIT`, `TXRX1_BIT`), so a board
    /// asked for both at once puts that switch in the transmit position and the
    /// receive path stops passing anything. It answers the command and goes
    /// deaf, which is exactly what was reported. LimeSuite's own GUI and
    /// SDRangel both leave a receiving board in `Rx` and reach for `TxRx` only
    /// on the cellular bands, which are the ones with duplexers.
    ///
    /// So an over costs one relay transaction either side of it — see
    /// [`LimeRfeConfig::switches_at_key_down`], which is what the source waits
    /// for before letting drive out.
    #[default]
    Auto,
    /// Pinned to receive. A key-down is refused rather than sent into a closed
    /// relay — see the interlock in the source.
    Rx,
    /// Pinned to transmit. Bench use: the board stays keyed.
    Tx,
    /// Pinned to both at once. What the cellular bands want — they have the
    /// duplexer for it, and bands 1, 2, 3 and 7 accept nothing else. On an
    /// amateur channel it stops receive, for the reason [`Self::Auto`] gives,
    /// so the settings panel says so rather than leaving it to be discovered.
    /// Asking for it on a shared connector is caught here rather than by the
    /// board's error code.
    TxRx,
}

impl RfeModeControl {
    pub const ALL: [RfeModeControl; 4] =
        [RfeModeControl::Auto, RfeModeControl::Rx, RfeModeControl::Tx, RfeModeControl::TxRx];

    pub fn label(self) -> &'static str {
        match self {
            RfeModeControl::Auto => "Automatic (follow the cabling)",
            RfeModeControl::Rx => "Always receive",
            RfeModeControl::Tx => "Always transmit",
            RfeModeControl::TxRx => "Always both (split connectors only)",
        }
    }
}

/// How the board is reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RfeLink {
    /// Nothing attached. The default, and the reason it is the default is that
    /// this board switches a power amplifier: an accessory that could be wired
    /// to anything must come up inert and be declared, exactly as
    /// [`crate::HpsdrFilterBoard::None`] does for the open-collector outputs.
    #[default]
    Off,
    /// The board's own micro-USB port, as a serial device. Independent of the
    /// radio, so this link works whatever is driving the I/Q.
    Serial,
    /// Through the SDR board's GPIO header — bit-banged I²C at
    /// [`RFE_I2C_ADDRESS`]. One cable fewer, but each frame is hundreds of USB
    /// round trips and it only exists while the LimeSDR itself is open.
    Board,
}

impl RfeLink {
    pub const ALL: [RfeLink; 3] = [RfeLink::Off, RfeLink::Serial, RfeLink::Board];

    pub fn label(self) -> &'static str {
        match self {
            RfeLink::Off => "Not connected",
            RfeLink::Serial => "Its own USB cable (serial)",
            RfeLink::Board => "Through the LimeSDR (GPIO / I²C)",
        }
    }
}

/// The receive attenuator's step. The board takes a count, not decibels.
pub const RFE_ATTEN_STEP_DB: u8 = 2;
/// The largest attenuator count the board accepts — 7 steps, so 14 dB.
pub const RFE_ATTEN_MAX_STEPS: u8 = 7;

/// LimeRFE settings, persisted inside [`crate::LimeConfig`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct LimeRfeConfig {
    /// How the board is reached; [`RfeLink::Off`] means "no LimeRFE here" and
    /// nothing is sent at all.
    pub link: RfeLink,
    /// The serial port, when [`Self::link`] is [`RfeLink::Serial`]. The baud is
    /// fixed at [`RFE_BAUD`] by the board's firmware, so only the path is
    /// really a setting; the rest of [`SerialConfig`] is carried so the port
    /// picker can reuse the CAT widgets unchanged.
    pub serial: SerialConfig,
    /// Which connector receive comes in on.
    pub port_rx: RfePort,
    /// Which connector transmit goes out of.
    pub port_tx: RfePort,
    /// Follow the operating frequency: resolve the channel from the dial and
    /// switch before any RF appears. On by default — it is the reason most
    /// people want this at all.
    pub follow_band: bool,
    /// The channel to use when [`Self::follow_band`] is off.
    pub channel: RfeChannel,
    /// What the relays should do. See [`RfeModeControl`] — one control rather
    /// than a mode plus a "follow PTT" flag that could disagree with it.
    pub mode: RfeModeControl,
    /// The notch filter, on the channels that have one.
    pub notch: bool,
    /// Receive attenuator, in steps of [`RFE_ATTEN_STEP_DB`] dB.
    pub atten_steps: u8,
    /// The board's fan. Worth having on for any sustained transmit.
    pub fan: bool,
}

impl Default for LimeRfeConfig {
    fn default() -> Self {
        LimeRfeConfig {
            link: RfeLink::Off,
            serial: SerialConfig::default(),
            port_rx: RfePort::J3,
            port_tx: RfePort::J4,
            follow_band: true,
            channel: RfeChannel::Wb1000,
            mode: RfeModeControl::Auto,
            notch: false,
            atten_steps: 0,
            fan: false,
        }
    }
}

impl LimeRfeConfig {
    /// Whether this cabling forces the board to be switched at key-down.
    ///
    /// True when receive and transmit share a connector, because the board
    /// refuses [`RfeMode::TxRx`] there — the `RXTX_SAME_CONN` error. Two very
    /// ordinary setups land here: one antenna on J3 for both directions, and
    /// anything on HF, where J5 is the only transmit path to the HF amplifier
    /// and is one jack.
    pub fn needs_ptt_switching(&self) -> bool {
        self.port_rx == self.port_tx
    }

    /// What the board should sit in while receiving.
    ///
    /// [`RfeMode::Rx`] on Automatic, on either cabling — see
    /// [`RfeModeControl::Auto`] for why a standing `TxRx` goes deaf on the
    /// amateur channels. And never `TxRx` on a shared connector whatever was
    /// asked for: putting an impossible request on the wire gets an error code
    /// back and leaves the board in whatever it was, which is a worse outcome
    /// than quietly asking for the reachable thing.
    pub fn rx_mode(&self) -> RfeMode {
        match self.mode {
            RfeModeControl::Rx | RfeModeControl::Auto => RfeMode::Rx,
            RfeModeControl::Tx => RfeMode::Tx,
            RfeModeControl::TxRx => {
                if self.needs_ptt_switching() {
                    RfeMode::Rx
                } else {
                    RfeMode::TxRx
                }
            }
        }
    }

    /// What the board should be in for an over, or `None` when it is already
    /// in a mode that transmits and nothing need be sent at key-down.
    ///
    /// `None` is now only the pinned cases: a board held in transmit has
    /// nothing to switch, and one held in receive refuses the over outright
    /// (see [`Self::tx_refusal`]).
    pub fn tx_mode(&self) -> Option<RfeMode> {
        match self.mode {
            // Pinned to receive: nothing to switch to. The caller refuses the
            // key-down rather than transmitting into a closed relay — see
            // `tx_refusal`.
            RfeModeControl::Rx => None,
            RfeModeControl::Tx => None,
            RfeModeControl::Auto => Some(RfeMode::Tx),
            RfeModeControl::TxRx => self.needs_ptt_switching().then_some(RfeMode::Tx),
        }
    }

    /// Whether an over moves the board's relays, and so has to wait for them.
    ///
    /// What the source reads before letting drive out: a transmitter that
    /// starts before the switch has thrown is driving into the receive path
    /// with the amplifier bypassed. Not the same question as
    /// [`Self::needs_ptt_switching`], which is about the *cabling* — this one
    /// is about the mode actually in force, and on Automatic the answer is yes
    /// on both cablings.
    pub fn switches_at_key_down(&self) -> bool {
        self.tx_mode().is_some()
    }

    /// Why a key-down cannot be honoured with this configuration, if it cannot.
    ///
    /// The one case is a board pinned to receive: nothing downstream will open
    /// the transmit relay, so the drive would go into the receive path with the
    /// amplifier bypassed. Refusing early is the same discipline as an unarmed
    /// HackRF publishing no transmit channel — the point is that no path can
    /// key it, not that most paths remember to check.
    pub fn tx_refusal(&self) -> Option<String> {
        (self.link != RfeLink::Off && self.mode == RfeModeControl::Rx).then(|| {
            format!(
                "the LimeRFE is pinned to receive, so its transmit relay stays closed — set \
                 the LimeRFE mode to Automatic in Settings → Radio, or move transmit to {}",
                if self.port_tx == RfePort::J4 { "another connector" } else { "J4" }
            )
        })
    }

    /// The standing-cost sentence for the settings panel and `open_status`.
    /// `None` when there is nothing worth saying, which is the ordinary case.
    ///
    /// Only the *surprising* configuration gets a sentence. Automatic switching
    /// the relays around an over is what everybody's LimeRFE does and costs one
    /// short transaction, so it is described in the panel's own prose rather
    /// than painted as a warning; being pinned to both at once is neither
    /// ordinary nor safe on an amateur channel, so it is.
    pub fn switching_note(&self) -> Option<String> {
        if self.link == RfeLink::Off {
            return None;
        }
        if self.mode == RfeModeControl::TxRx && !self.needs_ptt_switching() {
            return Some(
                "The LimeRFE is pinned to receive and transmit at once. Only the cellular \
                 bands have the duplexer that needs — on an amateur channel the board puts \
                 its transmit/receive switch in the transmit position and receive goes \
                 silent. Set the relays to Automatic unless this really is a cellular band."
                    .to_string(),
            );
        }
        self.needs_ptt_switching().then(|| {
            format!(
                "Receive and transmit are both on {}, so the board is switched to transmit at \
                 key-down and back at key-up and there is no arrangement that avoids it. \
                 Above 30 MHz, wiring transmit to J4 lets the switch happen without the \
                 receive path sharing a connector with a live amplifier.",
                self.port_rx.label()
            )
        })
    }

    /// Which connector each direction is on, when they are not the same one.
    ///
    /// A statement rather than a warning, and it earns its place because the
    /// default cabling is the split one and the failure it produces is silent:
    /// receive comes in on J3 and transmit leaves by J4, so a station with one
    /// antenna in J3 hears the band perfectly and radiates into an open
    /// connector. Nothing refuses, nothing errors, and every meter downstream
    /// of the antenna reads zero — which is exactly the report this exists to
    /// answer.
    ///
    /// `None` on a shared connector, where [`Self::switching_note`] has more
    /// to say and says it, and none of this arises.
    pub fn connector_note(&self) -> Option<String> {
        if self.link == RfeLink::Off || self.needs_ptt_switching() {
            return None;
        }
        Some(format!(
            "Receive comes in on {} and transmit goes out of {} — two different connectors, \
             which is what the board is for. With a single antenna it has to be on {}, and \
             transmit moved there with it: nothing transmitted reaches a connector the \
             amplifier is not driving.",
            self.port_rx.label(),
            self.port_tx.label(),
            self.port_rx.label(),
        ))
    }

    pub fn atten_db(&self) -> f64 {
        f64::from(self.atten_steps.min(RFE_ATTEN_MAX_STEPS) * RFE_ATTEN_STEP_DB)
    }

    /// The whole configuration as one string, for `Command::SetDeviceSetting`.
    ///
    /// One door rather than a pseudo-gain per control, because these settings
    /// only mean anything together: the channel a dial resolves to depends on
    /// the connectors, and the mode the board may be asked for depends on
    /// both. Five separate elements pushed one at a time would put states on
    /// the wire that no configuration ever asked for.
    ///
    /// [`Self::link`] and [`Self::serial`] ride along and are ignored on the
    /// far side: which cable the board is on is decided when it is opened.
    pub fn to_setting(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    /// Read one back. `None` for anything that is not a configuration, so a
    /// stale or hand-typed value leaves the board where it is.
    pub fn from_setting(value: &str) -> Option<LimeRfeConfig> {
        serde_json::from_str(value).ok()
    }
}

/// The channel whose filter covers `hz`.
///
/// Transcribed from LimeSuite's `FreqToBand`, first match wins — so the amateur
/// channels take precedence over the two wideband paths, whose ranges overlap
/// all of them. Anything above 4 GHz falls out as [`RfeChannel::Wb4000`], which
/// is LimeSuite's answer too; it is out of the board's range either way, and
/// returning the widest path is better than returning nothing.
pub fn channel_for(hz: f64) -> RfeChannel {
    const RANGES: [(RfeChannel, f64, f64); 11] = [
        (RfeChannel::Ham0030, 0.0, 30e6),
        (RfeChannel::Ham0070, 50e6, 70e6),
        (RfeChannel::Ham0145, 140e6, 150e6),
        (RfeChannel::Ham0220, 220e6, 225e6),
        (RfeChannel::Ham0435, 400e6, 450e6),
        (RfeChannel::Ham0920, 902e6, 928e6),
        (RfeChannel::Ham1280, 1220e6, 1420e6),
        (RfeChannel::Ham2400, 2.3e9, 2.5e9),
        (RfeChannel::Ham3500, 3.3e9, 3.7e9),
        (RfeChannel::Wb1000, 1.0, 1e9),
        (RfeChannel::Wb4000, 100.0, 4e9),
    ];
    for (ch, lo, hi) in RANGES {
        if hz >= lo && hz <= hi {
            return ch;
        }
    }
    RfeChannel::Wb4000
}

/// Narrow a receive channel to one the chosen connector can actually reach.
///
/// LimeSuite's `RxPortCheck`: J5 is wired only to the low-band filters, so
/// anything above 70 cm asked for there falls back to the wideband path.
pub fn rx_port_check(port: RfePort, ch: RfeChannel) -> RfeChannel {
    if port == RfePort::J5
        && !matches!(
            ch,
            RfeChannel::Ham0030
                | RfeChannel::Ham0070
                | RfeChannel::Ham0145
                | RfeChannel::Ham0220
                | RfeChannel::Ham0435
        )
    {
        return RfeChannel::Wb1000;
    }
    ch
}

/// Narrow a transmit channel to one the chosen connector can actually reach.
///
/// LimeSuite's `TxPortCheck`, and the asymmetry with receive is real: J5 is the
/// *only* transmit path to the HF and 6 m amplifiers, so anything else asked
/// for there becomes one of those two; and asking for HF or 6 m anywhere else
/// gets the wideband path, because those amplifiers are not wired to J3 or J4.
pub fn tx_port_check(port: RfePort, ch: RfeChannel) -> RfeChannel {
    if port == RfePort::J5 {
        return if ch == RfeChannel::Ham0070 { RfeChannel::Ham0070 } else { RfeChannel::Ham0030 };
    }
    if matches!(ch, RfeChannel::Ham0030 | RfeChannel::Ham0070) {
        return RfeChannel::Wb1000;
    }
    ch
}

/// The pair of channels a receive and a transmit frequency resolve to on this
/// cabling — what actually gets sent to the board.
pub fn resolve(cfg: &LimeRfeConfig, rx_hz: f64, tx_hz: f64) -> (RfeChannel, RfeChannel) {
    if !cfg.follow_band {
        return (rx_port_check(cfg.port_rx, cfg.channel), tx_port_check(cfg.port_tx, cfg.channel));
    }
    (rx_port_check(cfg.port_rx, channel_for(rx_hz)), tx_port_check(cfg.port_tx, channel_for(tx_hz)))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The band table, at the edges rather than in the middles — an off-by-one
    /// on a boundary is what puts a 30 MHz transmit into a wideband path with
    /// no amplifier behind it, or a 50 MHz one into an HF filter.
    #[test]
    fn the_band_table_matches_limesuite_at_every_edge() {
        // Ham ranges are inclusive at both ends.
        assert_eq!(channel_for(0.0), RfeChannel::Ham0030);
        assert_eq!(channel_for(14.2e6), RfeChannel::Ham0030);
        assert_eq!(channel_for(30e6), RfeChannel::Ham0030);
        assert_eq!(channel_for(50e6), RfeChannel::Ham0070);
        assert_eq!(channel_for(70e6), RfeChannel::Ham0070);
        assert_eq!(channel_for(144e6), RfeChannel::Ham0145);
        assert_eq!(channel_for(150e6), RfeChannel::Ham0145);
        assert_eq!(channel_for(222e6), RfeChannel::Ham0220);
        assert_eq!(channel_for(432e6), RfeChannel::Ham0435);
        assert_eq!(channel_for(915e6), RfeChannel::Ham0920);
        assert_eq!(channel_for(1296e6), RfeChannel::Ham1280);
        assert_eq!(channel_for(2400e6), RfeChannel::Ham2400);
        assert_eq!(channel_for(3400e6), RfeChannel::Ham3500);

        // Between the ham channels the wideband paths take over — below 1 GHz
        // the low one, above it the high one.
        assert_eq!(channel_for(40e6), RfeChannel::Wb1000);
        assert_eq!(channel_for(100e6), RfeChannel::Wb1000);
        assert_eq!(channel_for(1e9), RfeChannel::Wb1000);
        assert_eq!(channel_for(1.1e9), RfeChannel::Wb4000);
        assert_eq!(channel_for(5e9), RfeChannel::Wb4000, "past the board's reach, widest path");
    }

    /// The two port rules, which are not symmetric and are easy to get wrong.
    #[test]
    fn port_rules_narrow_to_what_the_connector_reaches() {
        // J5 receives the low bands and nothing else.
        assert_eq!(rx_port_check(RfePort::J5, RfeChannel::Ham0030), RfeChannel::Ham0030);
        assert_eq!(rx_port_check(RfePort::J5, RfeChannel::Ham0435), RfeChannel::Ham0435);
        assert_eq!(rx_port_check(RfePort::J5, RfeChannel::Ham0920), RfeChannel::Wb1000);
        // J3 receives anything.
        assert_eq!(rx_port_check(RfePort::J3, RfeChannel::Ham2400), RfeChannel::Ham2400);

        // J5 is the *only* HF/6 m transmit path, so everything there collapses
        // onto one of those two...
        assert_eq!(tx_port_check(RfePort::J5, RfeChannel::Ham0030), RfeChannel::Ham0030);
        assert_eq!(tx_port_check(RfePort::J5, RfeChannel::Ham0070), RfeChannel::Ham0070);
        assert_eq!(tx_port_check(RfePort::J5, RfeChannel::Ham0145), RfeChannel::Ham0030);
        // ...and asking for HF anywhere else gets the wideband path, because
        // that amplifier is not wired to J3 or J4.
        assert_eq!(tx_port_check(RfePort::J4, RfeChannel::Ham0030), RfeChannel::Wb1000);
        assert_eq!(tx_port_check(RfePort::J3, RfeChannel::Ham0070), RfeChannel::Wb1000);
        assert_eq!(tx_port_check(RfePort::J4, RfeChannel::Ham0145), RfeChannel::Ham0145);
    }

    /// The field report this exists for (issue #94): a receiving board must be
    /// left in `Rx`, on **either** cabling. Standing in `TxRx` because the
    /// connectors happen to be split is what put the board's transmit/receive
    /// switch in the transmit position and left the receive path passing
    /// nothing at all.
    #[test]
    fn a_receiving_board_is_never_left_standing_in_both_on() {
        for (rx, tx) in
            [(RfePort::J3, RfePort::J4), (RfePort::J3, RfePort::J3), (RfePort::J5, RfePort::J5)]
        {
            let cfg = LimeRfeConfig {
                port_rx: rx,
                port_tx: tx,
                link: RfeLink::Serial,
                ..Default::default()
            };
            assert_eq!(cfg.mode, RfeModeControl::Auto, "the default");
            assert_eq!(cfg.rx_mode(), RfeMode::Rx, "receiving means receiving");
            assert_eq!(cfg.tx_mode(), Some(RfeMode::Tx), "and an over switches the relays");
            assert!(cfg.switches_at_key_down(), "which the source has to wait for");
        }
    }

    /// The cabling question the transmit path reads. Two ordinary setups share
    /// a connector — one antenna on J3, and anything on HF where J5 is the only
    /// transmit path — so this is the common case, not an exotic one.
    #[test]
    fn a_shared_connector_forbids_the_standing_both_on_mode() {
        let split = LimeRfeConfig {
            port_rx: RfePort::J3,
            port_tx: RfePort::J4,
            mode: RfeModeControl::TxRx,
            link: RfeLink::Serial,
            ..Default::default()
        };
        assert!(!split.needs_ptt_switching());
        assert_eq!(split.rx_mode(), RfeMode::TxRx, "asked for by hand, and reachable here");
        assert_eq!(split.tx_mode(), None, "nothing to send at key-down");
        assert!(split.switching_note().is_some(), "but it is not a good idea and says so");

        for shared in [RfePort::J3, RfePort::J5] {
            let cfg = LimeRfeConfig {
                port_rx: shared,
                port_tx: shared,
                link: RfeLink::Serial,
                ..Default::default()
            };
            assert!(cfg.needs_ptt_switching());
            assert_eq!(cfg.rx_mode(), RfeMode::Rx, "TxRx would be refused by the board");
            assert_eq!(cfg.tx_mode(), Some(RfeMode::Tx), "so an over has to switch");
            assert!(cfg.switching_note().is_some(), "and the operator is told");
        }
    }

    /// The whole configuration survives the round trip through
    /// `SetDeviceSetting`, which is the only way anything the operator changes
    /// reaches a board that is already open.
    #[test]
    fn a_configuration_survives_the_settings_door() {
        let cfg = LimeRfeConfig {
            link: RfeLink::Serial,
            port_rx: RfePort::J5,
            port_tx: RfePort::J5,
            follow_band: false,
            channel: RfeChannel::Ham0435,
            mode: RfeModeControl::Rx,
            notch: true,
            atten_steps: 5,
            fan: true,
            ..Default::default()
        };
        assert_eq!(LimeRfeConfig::from_setting(&cfg.to_setting()), Some(cfg));
        assert_eq!(LimeRfeConfig::from_setting("LNAW"), None, "not a configuration");
    }

    /// Asking for both-on where the board cannot do it resolves to something
    /// reachable rather than going onto the wire to be refused.
    #[test]
    fn pinning_txrx_on_a_shared_connector_never_reaches_the_wire() {
        let cfg = LimeRfeConfig {
            port_rx: RfePort::J3,
            port_tx: RfePort::J3,
            mode: RfeModeControl::TxRx,
            link: RfeLink::Serial,
            ..Default::default()
        };
        assert_eq!(cfg.rx_mode(), RfeMode::Rx);
        assert_eq!(cfg.tx_mode(), Some(RfeMode::Tx));
    }

    /// The default cabling splits the two directions across two connectors,
    /// and a station with one antenna gets no warning from the hardware — so
    /// it gets one here. The report it answers: receives on every band,
    /// transmits into nothing.
    #[test]
    fn split_connectors_are_stated_because_one_antenna_cannot_be_on_both() {
        let split = LimeRfeConfig { link: RfeLink::Serial, ..Default::default() };
        assert_eq!(split.port_rx, RfePort::J3, "the default");
        assert_eq!(split.port_tx, RfePort::J4, "the default");
        let note = split.connector_note().expect("the default cabling says which is which");
        assert!(note.contains("J3 (TX/RX)"), "{note}");
        assert!(note.contains("J4 (TX)"), "{note}");
        assert_eq!(split.switching_note(), None, "and it is not the switching note's business");

        // One connector for both directions has nothing to say here — the
        // switching note covers that cabling in full.
        let shared =
            LimeRfeConfig { port_tx: RfePort::J3, link: RfeLink::Serial, ..Default::default() };
        assert_eq!(shared.connector_note(), None);
        assert!(shared.switching_note().is_some());

        // And no board means no notes at all.
        assert_eq!(LimeRfeConfig::default().connector_note(), None, "no board, nothing to say");
    }

    /// A board pinned to receive cannot transmit, and says so before anything
    /// is keyed rather than after the drive has gone into the receive path.
    #[test]
    fn a_board_pinned_to_receive_refuses_the_key_down() {
        let pinned =
            LimeRfeConfig { mode: RfeModeControl::Rx, link: RfeLink::Serial, ..Default::default() };
        let refusal = pinned.tx_refusal().expect("refused");
        assert!(refusal.contains("pinned to receive"), "{refusal}");

        // Automatic never refuses — on either cabling.
        for (rx, tx) in [(RfePort::J3, RfePort::J4), (RfePort::J5, RfePort::J5)] {
            let auto = LimeRfeConfig {
                mode: RfeModeControl::Auto,
                link: RfeLink::Serial,
                port_rx: rx,
                port_tx: tx,
                ..Default::default()
            };
            assert_eq!(auto.tx_refusal(), None);
        }

        // Nor does a pinned board that is not connected at all: with no
        // LimeRFE in the path there is nothing to stand in the way.
        let absent = LimeRfeConfig { mode: RfeModeControl::Rx, ..Default::default() };
        assert_eq!(absent.link, RfeLink::Off);
        assert_eq!(absent.tx_refusal(), None);
    }

    /// A 2 m contact resolves to the 2 m filters on the normal split cabling,
    /// and a split-frequency transmit follows the transmit leg rather than the
    /// receive one.
    #[test]
    fn resolve_follows_each_direction_separately() {
        let cfg = LimeRfeConfig { follow_band: true, ..Default::default() };
        let (rx, tx) = resolve(&cfg, 145.5e6, 145.5e6);
        assert_eq!((rx, tx), (RfeChannel::Ham0145, RfeChannel::Ham0145));

        // Cross-band: listening on 70 cm, transmitting on 2 m.
        let (rx, tx) = resolve(&cfg, 435.0e6, 145.0e6);
        assert_eq!((rx, tx), (RfeChannel::Ham0435, RfeChannel::Ham0145));
    }

    /// With follow-band off the operator's choice still goes through the port
    /// rules — picking a channel the connector cannot reach must not silently
    /// send an impossible request the board will refuse.
    #[test]
    fn a_manual_channel_is_still_narrowed_to_the_connector() {
        let cfg = LimeRfeConfig {
            follow_band: false,
            channel: RfeChannel::Ham2400,
            port_rx: RfePort::J5,
            port_tx: RfePort::J4,
            ..Default::default()
        };
        let (rx, tx) = resolve(&cfg, 0.0, 0.0);
        assert_eq!(rx, RfeChannel::Wb1000, "J5 does not reach 13 cm");
        assert_eq!(tx, RfeChannel::Ham2400);
    }

    /// Round-tripping a channel code, and the fallback for one this build does
    /// not know.
    #[test]
    fn channel_codes_round_trip() {
        for ch in RfeChannel::ALL {
            assert_eq!(RfeChannel::from_code(ch.code()), ch);
        }
        assert_eq!(RfeChannel::from_code(0), RfeChannel::NotSelected);
        assert_eq!(RfeChannel::from_code(99), RfeChannel::NotSelected);
    }

    /// The wideband paths are filters and relays only. Anything that decides
    /// whether there is an amplifier to warm up depends on this.
    #[test]
    fn only_the_band_specific_channels_have_an_amplifier() {
        assert!(!RfeChannel::Wb1000.has_pa());
        assert!(!RfeChannel::Wb4000.has_pa());
        assert!(!RfeChannel::NotSelected.has_pa());
        assert!(RfeChannel::Ham0030.has_pa());
        assert!(RfeChannel::Ham2400.has_pa());
    }

    #[test]
    fn attenuator_steps_are_two_db_apiece_and_clamped() {
        let mut cfg = LimeRfeConfig::default();
        assert_eq!(cfg.atten_db(), 0.0);
        cfg.atten_steps = 7;
        assert_eq!(cfg.atten_db(), 14.0);
        cfg.atten_steps = 200;
        assert_eq!(cfg.atten_db(), 14.0, "clamped rather than wrapped");
    }
}

#[cfg(test)]
mod config_tests {
    use super::*;
    use crate::{Backend, LimeConfig, RadioConfig};

    /// A `radio.json` written before this interface existed has no `lime` block
    /// at all, and must still load — serde fails a `RadioConfig` whole, and the
    /// default backend would then go and grab whatever hardware it found first.
    #[test]
    fn a_config_from_before_this_interface_still_loads() {
        let before = serde_json::to_value(RadioConfig::default()).unwrap();
        let mut before = before.as_object().unwrap().clone();
        assert!(before.remove("lime").is_some(), "the field is in the written form");

        let loaded: RadioConfig = serde_json::from_value(before.into()).unwrap();
        assert_eq!(loaded.lime, LimeConfig::default());
        assert_eq!(loaded, RadioConfig::default(), "and nothing else moved");
    }

    /// A hand-written block naming only what the operator cared about fills the
    /// rest in rather than failing — which is what `#[serde(default)]` is for,
    /// and worth pinning because the nested LimeRFE block needs it too.
    #[test]
    fn a_partial_block_fills_in_the_rest() {
        let json = r#"{
            "backend": "Lime",
            "lime": {
                "tx_enabled": true,
                "rfe": { "link": "Serial", "port_tx": "J3" }
            }
        }"#;
        let cfg: RadioConfig = serde_json::from_str(json).unwrap();
        assert_eq!(cfg.backend, Backend::Lime);
        assert!(cfg.lime.tx_enabled);
        // Untouched fields are the defaults...
        assert_eq!(cfg.lime.sample_rate_hz, LimeConfig::default().sample_rate_hz);
        assert_eq!(cfg.lime.rfe.link, RfeLink::Serial);
        assert_eq!(cfg.lime.rfe.mode, RfeModeControl::Auto);
        // ...and the cabling this describes is the shared-connector one, so the
        // relays have to switch at key-down.
        assert!(cfg.lime.rfe.needs_ptt_switching());
        assert_eq!(cfg.lime.rfe.tx_mode(), Some(RfeMode::Tx));
    }

    /// Every enum in this block is externally tagged, which postcard requires —
    /// an internally or adjacently tagged one would encode fine as JSON and be
    /// refused on the wire to a remote client.
    #[test]
    fn the_enums_survive_a_json_round_trip_by_name() {
        for link in RfeLink::ALL {
            let j = serde_json::to_string(&link).unwrap();
            assert!(j.starts_with('"'), "{link:?} is not a plain name: {j}");
            assert_eq!(serde_json::from_str::<RfeLink>(&j).unwrap(), link);
        }
        for m in RfeModeControl::ALL {
            let j = serde_json::to_string(&m).unwrap();
            assert_eq!(serde_json::from_str::<RfeModeControl>(&j).unwrap(), m);
        }
        for c in RfeChannel::ALL {
            let j = serde_json::to_string(&c).unwrap();
            assert_eq!(serde_json::from_str::<RfeChannel>(&j).unwrap(), c);
        }
    }
}
