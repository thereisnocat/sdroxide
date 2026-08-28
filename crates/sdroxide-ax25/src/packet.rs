//! The AX.25 packet codec: every frame type, parsed and serialised.
//!
//! Vendored from rax25 `src/lib.rs`; see `vendor/rax25/PROVENANCE.md`. Changes
//! from upstream:
//!
//! * `anyhow` becomes [`crate::Ax25Error`], so a caller can tell a truncated
//!   frame from an unparseable one;
//! * the `USE_FCS` branches are gone. Upstream had them compiled out because a
//!   KISS TNC does the FCS itself; here [`crate::hdlc`] owns it, checks it, and
//!   strips it before a frame reaches this module. A frame arriving here has
//!   already passed.
//! * [`Packet::parse`] cannot panic. Upstream indexed the address path and the
//!   information field directly and ended in `todo!()` on a control field it
//!   did not implement, which is safe against a TNC's vetted output and not
//!   against a channel; see the note on that function.
//! * an I frame keeps the PID it arrived with, rather than being assumed to
//!   carry no layer 3.
//!
//! Section numbers in the comments are the AX.25 specification's, kept from
//! upstream — they are the most useful thing in the file.

use crate::Ax25Error;
use crate::addr::Addr;

type Result<T, E = Ax25Error> = std::result::Result<T, E>;

/// An AX.25 packet, of any type.
#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    pub(crate) src: Addr,
    pub(crate) dst: Addr,
    pub(crate) digipeater: Vec<Addr>,
    pub(crate) rr_extseq: bool,
    pub(crate) command_response: bool,
    pub(crate) command_response_la: bool,
    pub(crate) rr_dist1: bool,
    #[allow(clippy::struct_field_names)]
    pub(crate) packet_type: PacketType,
}

/// All packet types.
#[derive(Clone, Debug, PartialEq)]
pub enum PacketType {
    Sabm(Sabm),
    Sabme(Sabme),
    Ua(Ua),
    Dm(Dm),
    Disc(Disc),
    Iframe(Iframe),
    Rr(Rr),
    Rnr(Rnr),
    Rej(Rej),
    Srej(Srej),
    Frmr(Frmr),
    Xid(Xid),
    Ui(Ui),
    Test(Test),
}

/// SABM - Set Asynchronous Balanced Mode (4.3.3.1, page 23)
#[derive(Clone, Debug, PartialEq)]
pub struct Sabm {
    pub poll: bool,
}

/// SAMBE - Set Asynchronous Balanced Mode Extended (4.3.3.2, page 23)
#[derive(Clone, Debug, PartialEq)]
pub struct Sabme {
    pub poll: bool,
}

/// RR - Receiver Ready (4.3.2.1, page 21)
///
/// Basically an ACK.
#[derive(Debug, PartialEq, Clone)]
pub struct Rr {
    pub poll: bool,
    pub nr: u8,
}

/// REJ - Reject (4.3.2.3, page 21)
///
/// Unclear why this is even needed. Couldn't RR with NR older than last sent
/// be equally eager to retransmit?
#[derive(Debug, PartialEq, Clone)]
pub struct Rej {
    pub poll: bool,
    pub nr: u8,
}

/// SREJ - Selective reject (4.3.2.4, page 21)
///
/// Request retransmissions of a single iframe.
#[derive(Debug, PartialEq, Clone)]
pub struct Srej {
    pub poll: bool,
    pub nr: u8,
}

/// FRMR - A deprecated error signaling (4.3.3.9, page 28)
///
/// The AX.25 2.2 spec deprecates this, and says to not generate these frames. But
/// it does specify what to do when receiving one.
#[derive(Debug, PartialEq, Clone)]
pub struct Frmr {
    pub poll: bool,
}

/// Test - Test frame (4.3.3.8, page 28)
///
/// It's ping, basically. The payload is mirrorred back, or (if there's not
/// enough room to store the payload), an empty response is returned.
///
/// The intended use of the poll flag here is unclear.
#[derive(Debug, PartialEq, Clone)]
pub struct Test {
    pub poll: bool,
    pub payload: Vec<u8>,
}

/// XID - Exchange Identification (4.3.3.7, page 24)
///
/// ISO 8885 exchange of capabilities, like extended sequence numbers,
/// max IFRAME size ("MTU"), and lots of other stuff.
///
/// TODO: Currently not implemented.
#[derive(Debug, PartialEq, Clone)]
pub struct Xid {
    pub poll: bool,
}

/// RNR - Receiver Not Ready (4.3.2.2, page 21)
///
/// Like RR, but asks the sender to not send more data for now.
/// The TCP version of this would be a closed receiver window.
#[derive(Debug, PartialEq, Clone)]
pub struct Rnr {
    pub poll: bool,
    pub nr: u8,
}

/// UA - Unnumbered Ack (4.3.3.4, page 23)
///
/// Acknowledge of things that don't have sequence numbers. Like SABM(E)
/// and DISC.
///
/// The equivalent of both the replying FIN and the SYN|ACK in TCP.
/// Probably not a good idea to use the same message for these two very
/// different events, since it's more "yeah, whatever, I hear you", but
/// not acknowledging if you heard "let's go" or "close down".
#[derive(Debug, PartialEq, Clone)]
pub struct Ua {
    pub poll: bool,
}

/// IFRAME - Information Frame (4.3.1, page 19)
///
/// Carries information. Obviously. Really, this could probably have been
/// merged with RR/RNR, even if it means empty payload. That's what
/// TCP does.
#[derive(Clone, Debug, PartialEq)]
pub struct Iframe {
    pub nr: u8,
    pub ns: u8,
    pub poll: bool,
    pub pid: u8,
    pub payload: Vec<u8>,
}

/// UI - Unnumbered Information (4.3.3.6, page 24)
///
/// Information frames outside of the sequential data flow.
/// Can be used whether a connection is established or not.
/// Disconnected UIs power APRS.
///
/// APRS doesn't use "push" for ACKs, but when unicasted
/// it could. A DM should be returned when push is set.
#[derive(Clone, Debug, PartialEq)]
pub struct Ui {
    pub pid: u8,
    pub push: bool,
    pub payload: Vec<u8>,
}

/// DM - Disconnected Mode (4.3.3.5, page 23)
///
/// The reply if the incoming packet implies a connection is active, or if
/// a SABM(E) was received and nothing was ready to receive it.
///
/// Basically a TCP RST.
#[derive(Clone, Debug, PartialEq)]
pub struct Dm {
    pub poll: bool,
}

/// DISC - Disconnect (4.3.3.3, page 23)
///
/// End the connection. A DISC is acked with a UA packet, which seems
/// silly. Replying with DISC would make more sense, but hey ho.
#[derive(Clone, Debug, PartialEq)]
pub struct Disc {
    pub poll: bool,
}

// Unnumbered frames. Ending in 11.
#[allow(clippy::unusual_byte_groupings)]
const CONTROL_SABM: u8 = 0b001_0_11_11;
#[allow(clippy::unusual_byte_groupings)]
const CONTROL_SABME: u8 = 0b011_0_11_11;
#[allow(clippy::unusual_byte_groupings)]
const CONTROL_UI: u8 = 0b000_0_00_11;
#[allow(clippy::unusual_byte_groupings)]
const CONTROL_DISC: u8 = 0b010_0_00_11;
#[allow(clippy::unusual_byte_groupings)]
const CONTROL_DM: u8 = 0b0000_1111;
const CONTROL_UA: u8 = 0b0110_0011;
const CONTROL_TEST: u8 = 0b1110_0011;
const CONTROL_XID: u8 = 0b1010_1111;
const CONTROL_FRMR: u8 = 0b1000_0111;

// Supervisor frames. Ending in 01.
const CONTROL_RR: u8 = 0b0000_0001;
const CONTROL_RNR: u8 = 0b0000_0101;
const CONTROL_REJ: u8 = 0b0000_1001;
const CONTROL_SREJ: u8 = 0b0000_1101;

// Iframes end in 0.
const CONTROL_IFRAME: u8 = 0b0000_0000;

// Masks.
const CONTROL_POLL: u8 = 0b0001_0000;
const NR_MASK: u8 = 0b1110_0000;
const TYPE_MASK: u8 = 0b0000_0011;
const NO_L3: u8 = 0xF0;

/// Longest digipeater path a frame may claim.
///
/// AX.25 2.0 allows eight and 2.2 allows two; eight is therefore the generous
/// reading and still bounds the address field. Without a limit a frame whose
/// end-of-address bit never arrives walks the information field seven bytes at
/// a time, calling arbitrary payload a path of callsigns.
const MAX_DIGIPEATERS: usize = 8;

/// PID: no layer 3 — the information field is what it looks like.
///
/// What a keyboard-to-keyboard session, a BBS and a node command line all use,
/// and the only value whose payload is safe to print as text.
pub const PID_NO_L3: u8 = NO_L3;

/// PID: NET/ROM. The payload is a routing header, not something to print.
pub const PID_NETROM: u8 = 0xCF;

/// PID: one segment of a frame too long for the link's packet length. Only
/// meaningful reassembled, which nothing here does.
pub const PID_SEGMENT: u8 = 0x08;

impl Packet {
    /// Construct a UI frame.
    // TODO: allow setting all the other fields.
    #[must_use]
    pub fn ui<T: Into<Vec<u8>>>(src: Addr, dst: Addr, payload: T) -> Self {
        Self {
            src,
            dst,
            digipeater: vec![],
            rr_extseq: false,
            command_response: false,
            command_response_la: true,
            rr_dist1: false,
            packet_type: PacketType::Ui(Ui { pid: 0xf0, push: false, payload: payload.into() }),
        }
    }
    /// Construct a UI frame with a digipeater path.
    ///
    /// Upstream's [`Packet::ui`] builds a direct frame, which is all a
    /// connected-mode station needs — a link is negotiated with a fixed path
    /// and the state machine carries it. APRS is the opposite: the path is
    /// chosen per transmission, it is the only thing deciding how far the
    /// frame goes, and it is what `WIDE1-1,WIDE2-1` means.
    #[must_use]
    pub fn ui_via<T: Into<Vec<u8>>>(src: Addr, dst: Addr, via: Vec<Addr>, payload: T) -> Self {
        let mut p = Self::ui(src, dst, payload);
        p.digipeater = via;
        // Sent as a command, which is what every APRS station on the channel
        // sends: destination C bit set, source C bit clear. `Packet::ui` builds
        // the other pairing — a *response* — which is right for a connected
        // session answering an I-frame and wrong for a broadcast nobody asked
        // for. The bits are one each and no decoder here cares, but a frame
        // that does not look like everyone else's is a frame some digipeater
        // firmware is entitled to ignore.
        p.command_response = true;
        p.command_response_la = false;
        p
    }

    /// Get the packet type.
    #[must_use]
    pub fn packet_type(&self) -> &PacketType {
        &self.packet_type
    }

    /// Who sent it.
    ///
    /// Upstream exposes only `packet_type`, because its callers already know
    /// the addresses — they are the ones that built the connection. A monitor
    /// does not: it is reading other people's traffic, and the addresses are
    /// most of what it has to show. Added here rather than making the fields
    /// public, so the struct keeps the shape upstream gave it.
    #[must_use]
    pub fn src(&self) -> &Addr {
        &self.src
    }

    /// Who it was addressed to.
    #[must_use]
    pub fn dst(&self) -> &Addr {
        &self.dst
    }

    /// The digipeater path, in order. Empty for a direct frame.
    #[must_use]
    pub fn digipeaters(&self) -> &[Addr] {
        &self.digipeater
    }

    /// The command/response bit. Several state-machine events need it, and it
    /// is not recoverable from the frame type alone.
    #[must_use]
    pub fn command_response(&self) -> bool {
        self.command_response
    }
    /// Serialize a packet, either as standard mod-8, or extended mod-128.
    #[must_use]
    #[allow(clippy::too_many_lines)]
    pub fn serialize(&self, ext: bool) -> Vec<u8> {
        let mut ret = Vec::with_capacity(
            14 + 2
                + if let PacketType::Iframe(s) = &self.packet_type {
                    s.payload.len() + 1
                } else {
                    0
                },
        );
        ret.extend(self.dst.serialize(false, self.command_response, self.rr_dist1, false));
        // Upstream's assertion, kept, with the reason it is safe written down:
        // every packet this crate *builds* sets the two bits opposite, and an
        // AX.25 v1 frame off the air has both clear — so re-serialising a
        // parsed packet would fire it. Nothing does that today. Anything that
        // starts to (a digipeater, a frame editor) has to decide what the two
        // bits should become before it reaches here.
        assert_ne!(self.command_response, self.command_response_la);
        ret.extend(self.src.serialize(
            self.digipeater.is_empty(),
            self.command_response_la,
            self.rr_extseq, // Setting this bit for extseq seems to be a de facto standard.
            false,
        ));

        {
            let l = self.digipeater.len();
            for (n, digi) in self.digipeater.iter().enumerate() {
                ret.extend(digi.serialize(
                    n == l - 1, // last repeater?
                    false,      // "has been repeated"
                    false,
                    false,
                ));
            }
        }

        match &self.packet_type {
            // U frames. Control always one byte.
            PacketType::Sabm(s) => {
                if ext {
                    ret.push(CONTROL_SABME | if s.poll { CONTROL_POLL } else { 0 });
                } else {
                    ret.push(CONTROL_SABM | if s.poll { CONTROL_POLL } else { 0 });
                }
            }
            PacketType::Sabme(s) => ret.push(CONTROL_SABME | if s.poll { CONTROL_POLL } else { 0 }),
            PacketType::Ua(s) => ret.push(CONTROL_UA | if s.poll { CONTROL_POLL } else { 0 }),
            PacketType::Disc(disc) => {
                ret.push(CONTROL_DISC | if disc.poll { CONTROL_POLL } else { 0 });
            }
            PacketType::Dm(s) => ret.push(CONTROL_DM | if s.poll { CONTROL_POLL } else { 0 }),
            // TODO: FRMR data too.
            PacketType::Frmr(s) => ret.push(CONTROL_FRMR | if s.poll { CONTROL_POLL } else { 0 }),
            PacketType::Ui(s) => {
                ret.push(CONTROL_UI | if s.push { CONTROL_POLL } else { 0 });
                ret.push(s.pid);
                ret.extend(&s.payload);
            }
            // TODO: XID data too.
            PacketType::Xid(s) => ret.push(CONTROL_XID | if s.poll { CONTROL_POLL } else { 0 }),
            PacketType::Test(s) => {
                ret.push(CONTROL_TEST | if s.poll { CONTROL_POLL } else { 0 });
                ret.extend(&s.payload);
            }

            // S frames.
            PacketType::Rr(s) => {
                if ext {
                    ret.push(CONTROL_RR);
                    ret.push((s.nr << 1) & 0xFE | u8::from(s.poll));
                } else {
                    ret.push(
                        CONTROL_RR
                            | if s.poll { CONTROL_POLL } else { 0 }
                            | ((s.nr << 5) & NR_MASK),
                    );
                }
            }
            PacketType::Rnr(s) => {
                if ext {
                    ret.push(CONTROL_RNR);
                    ret.push((s.nr << 1) & 0xFE | u8::from(s.poll));
                } else {
                    ret.push(
                        CONTROL_RNR
                            | if s.poll { CONTROL_POLL } else { 0 }
                            | ((s.nr << 5) & NR_MASK),
                    );
                }
            }
            PacketType::Rej(s) => {
                if ext {
                    ret.push(CONTROL_REJ);
                    ret.push((s.nr << 1) & 0xFE | u8::from(s.poll));
                } else {
                    ret.push(CONTROL_REJ | if s.poll { CONTROL_POLL } else { 0 });
                }
            }
            PacketType::Srej(s) => {
                if ext {
                    ret.push(CONTROL_SREJ);
                    ret.push((s.nr << 1) & 0xFE | u8::from(s.poll));
                } else {
                    ret.push(CONTROL_SREJ | if s.poll { CONTROL_POLL } else { 0 });
                }
            }
            PacketType::Iframe(iframe) => {
                if ext {
                    ret.push(CONTROL_IFRAME | ((iframe.ns << 1) & 0xFE));
                    ret.push((iframe.nr << 1) & 0xFE | u8::from(iframe.poll));
                } else {
                    ret.push(
                        CONTROL_IFRAME
                            | if iframe.poll { CONTROL_POLL } else { 0 }
                            | ((iframe.nr << 5) & 0b1110_0000)
                            | ((iframe.ns << 1) & 0b0000_1110),
                    );
                }
                ret.push(iframe.pid);
                ret.extend(&iframe.payload);
            }
        }
        ret
    }

    /// Parse packet from bytes.
    ///
    /// A packet with sequence numbers in it (S and I frames) cannot be parsed
    /// without knowing if it's extended or not, because the control field is
    /// either one or two bytes.
    ///
    /// Ideally you already know, because you sent, received, or at least saw
    /// the SABM (not extended) or SABME (extended).
    ///
    /// The Linux stack uses the `rbit_ext` reserved bit in the source address
    /// to indicate extended mode. This is the used by the other Linux tooling
    /// like `axlisten`. But it's nonstandard.
    ///
    /// In theory other heuristics can be provided, to try to brute force the
    /// mode. But really, it's best if you know ahead of time.
    ///
    /// This code supports using the Linux bit, by providing `None` as `ext`, as
    /// opposed to `Some(bool)`.
    ///
    /// # Nothing here may panic
    ///
    /// Upstream indexed the address path and the information field directly and
    /// ended the unknown-control arm in `todo!()`, which is fine for frames a
    /// KISS TNC has already vetted and wrong for these callers: this parses
    /// every frame heard on the channel — on the audio thread — and every frame
    /// *sent*, including the verbatim bytes a KISS host handed over. A frame
    /// that is truncated, or carries a control field nobody has implemented,
    /// still passed a 16-bit check sequence, so it is somebody's real traffic
    /// and it must come back as an error rather than as a dead receiver.
    #[allow(clippy::too_many_lines)]
    pub fn parse(bytes: &[u8], ext: Option<bool>) -> Result<Self> {
        // The FCS is `crate::hdlc`'s business and is already gone by here.
        if bytes.len() < 15 {
            return Err(Ax25Error::Short(bytes.len()));
        }
        let dst = Addr::parse(&bytes[0..7])?;
        let src = Addr::parse(&bytes[7..14])?;

        let ext = match ext {
            Some(v) => v,
            None => src.rbit_ext,
        };

        let mut digipeater = Vec::new();
        let mut pos = 14;
        let mut more_addresses = !src.lowbit;
        while more_addresses {
            // Both bounds matter and for different reasons. The length check
            // catches a frame whose source address says "digipeaters follow"
            // and then stops; the count catches one where the end-of-address
            // bit never arrives at all, which would otherwise walk the whole
            // frame seven bytes at a time and call the information field a
            // path of callsigns.
            if pos + 7 > bytes.len() {
                return Err(Ax25Error::Parse("address field ends mid-digipeater".into()));
            }
            if digipeater.len() >= MAX_DIGIPEATERS {
                return Err(Ax25Error::Parse(format!(
                    "more than {MAX_DIGIPEATERS} digipeaters in the path"
                )));
            }
            let digi = Addr::parse(&bytes[pos..pos + 7])?;
            more_addresses = !digi.lowbit;
            pos += 7;
            digipeater.push(digi);
        }

        if pos >= bytes.len() {
            return Err(Ax25Error::Parse("frame ends before its control field".into()));
        }
        let rest = &bytes[pos..];
        let control1 = rest[0];
        let (poll, nr, ns, bytes) = {
            if !ext || control1 & TYPE_MASK == 3 {
                // NOTE: ns/nr will be nonsense for U frames.
                // ns will be nonsense for S frames.
                (
                    control1 & CONTROL_POLL == CONTROL_POLL,
                    (control1 >> 5) & 7,
                    (control1 >> 1) & 7,
                    &rest[1..],
                )
            } else {
                if rest.len() < 2 {
                    return Err(Ax25Error::Parse(
                        "AX.25 in ext mode, but S/U frame is too short".into(),
                    ));
                }
                let control2 = rest[1];
                (control2 & 1 == 1, (control2 >> 1) & 127, (control1 >> 1) & 127, &rest[2..])
            }
        };
        // Bound before the struct literal, so an arm that cannot make sense of
        // the frame can return an error rather than having to produce one.
        let packet_type = match control1 & TYPE_MASK {
            // I frames. Second control byte, with NR and NS.
            0 | 2 => {
                // The PID is kept rather than assumed. Upstream hard-coded
                // `NO_L3` with a TODO beside it; the byte is right here and it
                // is the only thing that tells a terminal's text (0xF0) from
                // NET/ROM (0xCF) or a segment of a longer frame (0x08), which
                // would otherwise be printed as line noise.
                let (&pid, payload) = bytes
                    .split_first()
                    .ok_or_else(|| Ax25Error::Parse("I frame with no PID".into()))?;
                PacketType::Iframe(Iframe { ns, nr, poll, pid, payload: payload.to_vec() })
            }
            // S frames. Second control byte, with NR.
            1 => match control1 & !NR_MASK & !CONTROL_POLL {
                CONTROL_RR => PacketType::Rr(Rr { poll, nr }),
                CONTROL_RNR => PacketType::Rnr(Rnr { poll, nr }),
                CONTROL_REJ => PacketType::Rej(Rej { poll, nr }),
                CONTROL_SREJ => PacketType::Srej(Srej { poll, nr }),
                // The mask leaves bits 2 and 3 and the two type bits, and all
                // four of those combinations are named above.
                _ => unreachable!("{control1:#010b} matched S but named no S frame"),
            },
            // U frames. No second control byte.
            3 => match !CONTROL_POLL & control1 {
                CONTROL_SABME => PacketType::Sabme(Sabme { poll }),
                CONTROL_SABM => PacketType::Sabm(Sabm { poll }),
                CONTROL_UA => PacketType::Ua(Ua { poll }),
                CONTROL_DISC => PacketType::Disc(Disc { poll }),
                CONTROL_DM => PacketType::Dm(Dm { poll }),
                CONTROL_FRMR => PacketType::Frmr(Frmr { poll }),
                CONTROL_UI => {
                    let (&pid, payload) = bytes
                        .split_first()
                        .ok_or_else(|| Ax25Error::Parse("UI frame with no PID".into()))?;
                    PacketType::Ui(Ui { push: poll, pid, payload: payload.to_vec() })
                }
                CONTROL_XID => PacketType::Xid(Xid { poll }),
                CONTROL_TEST => PacketType::Test(Test { poll, payload: bytes.to_vec() }),
                c => {
                    return Err(Ax25Error::Parse(format!(
                        "unimplemented U frame control {c:#010b}"
                    )));
                }
            },
            // `& 3` is 0..=3 and every value is an arm above.
            _ => unreachable!("{control1} & 3 > 3"),
        };
        Ok(Packet {
            src: src.clone(),
            dst: dst.clone(),
            command_response: dst.highbit,
            command_response_la: src.highbit,
            rr_dist1: dst.rbit_ext,
            rr_extseq: ext,
            digipeater,
            packet_type,
        })
    }
}

/// Hub packet serializer/deserializer.
///
/// Hub reads and writes packets. Normally to a KISS serial port. But
/// ideally something more clevel with priority queues and mux-capability.
///
/// Then again that more clever system could just be freestanding, and expose
/// KISS as an interface to it.
pub trait Hub {
    /// Send frame. May block.
    ///
    /// The provided frame must be a complete AX.25 frame, without FEND or
    /// escaping.
    fn send(&mut self, frame: &[u8]) -> Result<()>;

    /// Try receiving a frame.
    ///
    /// Ok(None) means timeout.
    fn recv_timeout(&mut self, timeout: std::time::Duration) -> Result<Option<Vec<u8>>>;

    /// Clone a kisser.
    /// All packets get delivered to all clones.
    fn clone(&self) -> Box<dyn Hub>;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame the way it comes off the air: destination, source, control, and
    /// whatever follows. Written by hand rather than through `serialize`, which
    /// is the point — these are the frames `serialize` would never build.
    fn frame(dst: &str, src: &str, last_addr_is_src: bool, tail: &[u8]) -> Vec<u8> {
        let mut f = Addr::new(dst).unwrap().serialize(false, true, false, false);
        f.extend(Addr::new(src).unwrap().serialize(last_addr_is_src, false, false, false));
        f.extend_from_slice(tail);
        f
    }

    /// A source address that says "digipeaters follow" and then stops. It
    /// passed a 16-bit check sequence, so it is somebody's real traffic — and
    /// upstream sliced straight past the end of it.
    #[test]
    fn a_truncated_digipeater_path_is_an_error_not_a_panic() {
        // Three bytes where a seven-byte address should be, then nothing.
        let f = frame("APRS", "OE3JJS", false, &[0x03, 0xF0, b'h']);
        assert!(matches!(Packet::parse(&f, None), Err(Ax25Error::Parse(_))));
    }

    /// A path with no end-of-address bit anywhere. Without a limit this walks
    /// the information field seven bytes at a time, reading payload as
    /// callsigns until it runs off the end.
    #[test]
    fn an_endless_digipeater_path_is_an_error_not_a_panic() {
        let mut f = frame("APRS", "OE3JJS", false, &[]);
        for _ in 0..12 {
            f.extend(Addr::new("WIDE1-1").unwrap().serialize(false, false, false, false));
        }
        f.extend_from_slice(&[0x03, 0xF0, b'x']);
        assert!(matches!(Packet::parse(&f, None), Err(Ax25Error::Parse(_))));
    }

    /// Addresses, then nothing. `hdlc` delivers frames down to thirteen bytes,
    /// so this length reaches the codec in practice.
    #[test]
    fn a_frame_with_no_control_field_is_an_error_not_a_panic() {
        let f = frame("APRS", "OE3JJS", true, &[0; 1]);
        // Fifteen bytes, one of which is the control field: truncate it away.
        assert!(Packet::parse(&f[..14], None).is_err());
    }

    /// An I frame whose information field ends where the PID should be.
    #[test]
    fn an_i_frame_with_no_pid_is_an_error_not_a_panic() {
        // Control 0x00 is I, N(S)=0, N(R)=0, and then the frame stops.
        let f = frame("OE3JJS-1", "OE3JJS-10", true, &[0x00]);
        assert!(matches!(Packet::parse(&f, Some(false)), Err(Ax25Error::Parse(_))));
    }

    /// A U-frame control field nobody has implemented. Upstream ended in
    /// `todo!()`, which on the audio thread is a dead receiver.
    #[test]
    fn an_unknown_u_frame_control_byte_is_an_error_not_a_panic() {
        // 0b0011_0011: ends in 11 so it is a U frame, and names none of them.
        let f = frame("OE3JJS-1", "OE3JJS-10", true, &[0b0011_0011, 0xF0]);
        assert!(matches!(Packet::parse(&f, Some(false)), Err(Ax25Error::Parse(_))));
    }

    /// The PID says whether the payload is text at all. Assuming `NO_L3`, as
    /// upstream did, prints a NET/ROM routing header as line noise.
    #[test]
    fn an_i_frames_pid_survives_the_parse() {
        for pid in [PID_NO_L3, PID_NETROM, PID_SEGMENT] {
            let f = frame("OE3JJS-1", "OE3JJS-10", true, &[0x00, pid, b'h', b'i']);
            match Packet::parse(&f, Some(false)).expect("frame does not parse").packet_type() {
                PacketType::Iframe(i) => {
                    assert_eq!(i.pid, pid);
                    assert_eq!(i.payload, b"hi");
                }
                other => panic!("expected an I frame, got {other:?}"),
            }
        }
    }

    /// The whole point, pinned: whatever arrives, `parse` returns.
    ///
    /// One in every 65536 corrupted frames passes a 16-bit check sequence by
    /// chance, so on a busy channel this is not a hypothetical — it is a
    /// receiver that dies after an afternoon, from somebody else's traffic.
    #[test]
    fn parse_never_panics_on_a_mutated_frame() {
        let good = [
            // A UI frame with a digipeater path, the APRS shape.
            Packet::ui_via(
                Addr::new("OE3JJS-7").unwrap(),
                Addr::new("APRS").unwrap(),
                vec![Addr::new("WIDE1-1").unwrap(), Addr::new("WIDE2-1").unwrap()],
                &b"=4812.00N/01620.00E-test"[..],
            )
            .serialize(false),
            // An I frame, the connected-mode shape.
            Packet {
                src: Addr::new("OE3JJS-10").unwrap(),
                dst: Addr::new("OE3JJS-1").unwrap(),
                digipeater: vec![Addr::new("OE3XLR-1").unwrap()],
                rr_extseq: false,
                command_response: true,
                command_response_la: false,
                rr_dist1: false,
                packet_type: PacketType::Iframe(Iframe {
                    nr: 3,
                    ns: 2,
                    poll: false,
                    pid: PID_NO_L3,
                    payload: b"hello from a BBS>".to_vec(),
                }),
            }
            .serialize(false),
        ];
        for f in &good {
            // Every truncation.
            for n in 0..=f.len() {
                let _ = Packet::parse(&f[..n], None);
                let _ = Packet::parse(&f[..n], Some(false));
                let _ = Packet::parse(&f[..n], Some(true));
            }
            // Every single-bit flip.
            for byte in 0..f.len() {
                for bit in 0..8 {
                    let mut m = f.clone();
                    m[byte] ^= 1 << bit;
                    let _ = Packet::parse(&m, None);
                    let _ = Packet::parse(&m, Some(false));
                    let _ = Packet::parse(&m, Some(true));
                }
            }
        }
    }
}
