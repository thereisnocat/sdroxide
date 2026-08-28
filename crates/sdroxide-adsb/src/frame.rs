//! Which messages may be believed, and on what grounds.
//!
//! # Two kinds of claim
//!
//! Mode S overlays its parity differently depending on what the reply is, and
//! that difference decides how much a passive receiver may believe:
//!
//! * **Extended squitters (DF17, DF18) and all-call replies (DF11)** carry a
//!   plain check sequence. The syndrome comes out zero or the message is wrong.
//!   One frame in sixteen million passes by luck, and at the handful of
//!   candidates a second this correlator produces, that is never. These prove
//!   themselves.
//!
//! * **Surveillance and Comm-B replies (DF0, DF4, DF5, DF16, DF20, DF21)** have
//!   the aircraft's address XORed into the parity, because they are answers to
//!   an interrogation that already knew who had been asked. The syndrome of one
//!   of these is not zero — it *is* the address. There is nothing inside the
//!   frame to check it against, so taken alone it is unfalsifiable: any 56 bits
//!   of noise "decode" to some address.
//!
//! The rule enforced here is the one every serious receiver uses. A
//! surveillance reply is accepted only when the address its parity yields is
//! already being tracked from a verified squitter, and recently. That turns an
//! unfalsifiable claim into a falsifiable one — the address had to come out
//! right from sixteen million, and that aircraft had to be in the air now — and
//! it is what buys the squawk codes and the faster altitude updates that the
//! extended squitter alone does not give.
//!
//! # Deliberately absent: error correction
//!
//! dump1090 and its descendants will flip one bit, or two, looking for a
//! syndrome of zero. It recovers a useful fraction of weak frames and it also
//! invents aircraft — a 112-bit message has 112 single-bit neighbours, and on a
//! noisy band some of them check out. An invented aircraft on a map looks
//! exactly like a real one, so this decoder does not guess.

use crate::crc;
use crate::message::{self, Message};

/// A message that passed, and on what grounds.
#[derive(Debug, Clone, PartialEq)]
pub enum Accepted {
    /// Check sequence verified outright — DF17/18 squitter, or DF11 all-call.
    Verified(Message),
    /// A surveillance reply whose overlaid address matched a tracked aircraft.
    Matched(Message),
}

impl Accepted {
    pub fn message(&self) -> &Message {
        match self {
            Accepted::Verified(m) | Accepted::Matched(m) => m,
        }
    }
    pub fn icao(&self) -> u32 {
        self.message().icao
    }
    pub fn is_verified(&self) -> bool {
        matches!(self, Accepted::Verified(_))
    }
}

/// Why a candidate was thrown away, so the panel can say which kind of failure
/// the band is producing — "busy with something else" and "nothing there" look
/// identical from an empty aircraft list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rejected {
    /// The syndrome was not zero on a format whose syndrome must be.
    BadCrc,
    /// A surveillance reply for an address nobody has heard a squitter from.
    Unmatched,
    /// A downlink format this decoder does not read.
    Unsupported,
    /// The length and the format disagree, so there is nothing to check.
    Malformed,
}

/// Decide what to do with one demodulated message.
///
/// `known` answers whether an address is already being tracked from a verified
/// frame; that is the caller's aircraft table, not this module's business.
pub fn accept(bytes: &[u8], known: impl Fn(u32) -> bool) -> Result<Accepted, Rejected> {
    let df = message::downlink_format(bytes);
    if bytes.len() != message::message_len(df) {
        return Err(Rejected::Malformed);
    }
    let syn = crc::syndrome(bytes);
    match df {
        11 | 17 | 18 => {
            if syn != 0 {
                return Err(Rejected::BadCrc);
            }
            let icao = u32::from(bytes[1]) << 16 | u32::from(bytes[2]) << 8 | u32::from(bytes[3]);
            Ok(Accepted::Verified(message::decode(bytes, icao)))
        }
        0 | 4 | 5 | 16 | 20 | 21 => {
            let icao = syn & 0x00FF_FFFF;
            if icao == 0 || !known(icao) {
                return Err(Rejected::Unmatched);
            }
            Ok(Accepted::Matched(message::decode(bytes, icao)))
        }
        _ => Err(Rejected::Unsupported),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::Body;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len() / 2).map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()).collect()
    }

    /// A published extended squitter passes on its own, and one bit flipped in
    /// it does not.
    #[test]
    fn a_squitter_proves_itself_and_a_corrupted_one_does_not() {
        let good = hex("8D40621D58C382D690C8AC2863A7");
        let a = accept(&good, |_| false).expect("a published frame is accepted");
        assert!(a.is_verified());
        assert_eq!(a.icao(), 0x40_621D);

        let mut bad = good.clone();
        bad[6] ^= 0x08;
        assert_eq!(accept(&bad, |_| false).unwrap_err(), Rejected::BadCrc);
    }

    /// The whole point of the second half of this module: the same reply is
    /// refused when nobody is tracking that address and accepted when somebody
    /// is. Without the rule these bytes are indistinguishable from noise.
    #[test]
    fn a_surveillance_reply_is_only_believed_for_an_aircraft_already_heard() {
        let want = 0x48_4148u32;
        let mut msg = hex("20001910000000");
        crc::seal(&mut msg, want);

        assert_eq!(accept(&msg, |_| false).unwrap_err(), Rejected::Unmatched);
        let a = accept(&msg, |icao| icao == want).expect("accepted once the address is known");
        assert_eq!(a.icao(), want);
        assert!(!a.is_verified(), "an address match is not a check sequence");
        assert!(matches!(a.message().body, Body::Altitude { .. }));
    }

    /// A length that disagrees with the format cannot be checked at all:
    /// running a 112-bit division over 56 bits of message and 56 bits of the
    /// next aircraft's silence would reject everything for the wrong reason.
    #[test]
    fn a_length_that_disagrees_with_the_format_is_malformed() {
        assert_eq!(accept(&hex("8D40621D58C382"), |_| true).unwrap_err(), Rejected::Malformed);
        assert_eq!(
            accept(&hex("2000191000000000000000000000"), |_| true).unwrap_err(),
            Rejected::Malformed
        );
    }

    /// An all-call reply is a verified frame with nothing in it but an address,
    /// which is how an aircraft whose ADS-B is off or broken still gets a row.
    #[test]
    fn an_all_call_reply_is_an_address_and_nothing_else() {
        let mut msg = hex("5D4CA1FA000000");
        crc::seal(&mut msg, 0);
        let a = accept(&msg, |_| false).expect("a sealed all-call is verified");
        assert!(a.is_verified());
        assert_eq!(a.icao(), 0x4C_A1FA);
        assert_eq!(a.message().body, Body::AllCall);
    }
}
