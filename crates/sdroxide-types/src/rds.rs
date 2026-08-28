//! RDS and RBDS: the data a broadcast FM station carries beside its audio.
//!
//! A 57 kHz subcarrier — the third harmonic of the 19 kHz stereo pilot — carries
//! 1187.5 bps of data organised into *groups* of four 16-bit blocks. Different
//! group types carry different things: the eight-character station name, the
//! programme type, a 64-character free-text field, the alternative frequencies
//! the same programme is on, and the station's clock.
//!
//! RDS (IEC 62106) and RBDS (NRSC-4-B) are the *same signal*. They differ in two
//! places that matter here: the 32-entry programme-type table means different
//! things in each, and North American stations derive their programme
//! identification code from their call letters. Neither difference is visible in
//! the bits, so which table to read them against is a decision made outside the
//! decoder — see [`RdsStandard`].
//!
//! Everything on the wire is therefore a **raw code**, not a rendered string: the
//! programme type travels as the 5-bit number the station sent, and the name it
//! is given is chosen when it is drawn. That is what lets the operator flip
//! between the two tables and have everything already received re-label itself,
//! rather than waiting out another cycle of groups from the station.

use serde::{Deserialize, Serialize};

/// Which of the two programme-type tables to read a station against, and whether
/// to try its programme identification as a call sign.
///
/// [`RdsStandard::Auto`] is the default and resolves per station — see
/// [`RdsStandard::resolve`], which is candid about the guess it is making.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RdsStandard {
    #[default]
    Auto,
    /// IEC 62106, used everywhere outside North America.
    Rds,
    /// NRSC-4-B, used in the United States, Canada and Mexico.
    Rbds,
}

impl RdsStandard {
    /// Settle [`RdsStandard::Auto`] against what this station has said about
    /// itself. Never returns `Auto`.
    ///
    /// The Extended Country Code, when the station sends one (group 1A), is
    /// decisive: its high nibble is the ITU region, and `0xA` is the Americas.
    /// Absent one, the fallback is whether the programme identification decodes
    /// to a North American call sign under the NRSC-4-B algorithm.
    ///
    /// That fallback is a *guess*, and it is worth being clear about why. The
    /// call-letter range `0x1000..=0x994F` covers PI country codes 1 through 9,
    /// and those are real country codes elsewhere in the world — a European
    /// station can land inside it and read as a plausible American call sign.
    /// The ECC is the only thing that settles it, most stations outside North
    /// America do send one, and the manual override exists for the rest.
    pub fn resolve(self, pi: Option<u16>, ecc: Option<u8>) -> RdsStandard {
        match self {
            RdsStandard::Rds | RdsStandard::Rbds => self,
            RdsStandard::Auto => match ecc {
                Some(e) => {
                    if e >> 4 == 0xA {
                        RdsStandard::Rbds
                    } else {
                        RdsStandard::Rds
                    }
                }
                None if pi.and_then(pi_callsign).is_some() => RdsStandard::Rbds,
                None => RdsStandard::Rds,
            },
        }
    }

    /// Whether this station's programme identification may be read out *as a
    /// call sign*. Asked of what the operator selected, not of what
    /// [`RdsStandard::resolve`] made of it.
    ///
    /// The distinction is the whole point. `resolve`'s fallback is a guess, and
    /// a guess is good enough to pick between two programme-type tables — one
    /// of the two labels is wrong and the other is right, and the operator can
    /// see which. It is *not* good enough to print four letters that look like
    /// the station's name: `pi_callsign` accepts `0x1000..=0x994F`, which is
    /// nine countries' worth of RDS country codes, so YLE Radio Suomi on
    /// `0x6201` reads as "WFBL" and DR P1 on `0x9101` as "WWWF" until their
    /// first group 1A arrives — and on a signal poor enough that 1A takes a
    /// while, the fabricated name is what sits on screen.
    ///
    /// So the name is only ever spelled out on evidence: the operator said
    /// RBDS, or the station's own extended country code put it in the Americas.
    /// Guessed RBDS still picks the RBDS programme-type table; it just does not
    /// get to name the station.
    pub fn names_from_pi(self, ecc: Option<u8>) -> bool {
        match self {
            RdsStandard::Rbds => true,
            RdsStandard::Rds => false,
            RdsStandard::Auto => ecc.is_some_and(|e| e >> 4 == 0xA),
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            RdsStandard::Auto => "Auto",
            RdsStandard::Rds => "RDS",
            RdsStandard::Rbds => "RBDS",
        }
    }

    pub const ALL: [RdsStandard; 3] = [RdsStandard::Auto, RdsStandard::Rds, RdsStandard::Rbds];
}

/// The 32 RDS programme types of IEC 62106.
const PTY_RDS: [&str; 32] = [
    "None",
    "News",
    "Current Affairs",
    "Information",
    "Sport",
    "Education",
    "Drama",
    "Culture",
    "Science",
    "Varied",
    "Pop Music",
    "Rock Music",
    "Easy Listening",
    "Light Classical",
    "Serious Classical",
    "Other Music",
    "Weather",
    "Finance",
    "Children's Programmes",
    "Social Affairs",
    "Religion",
    "Phone In",
    "Travel",
    "Leisure",
    "Jazz Music",
    "Country Music",
    "National Music",
    "Oldies Music",
    "Folk Music",
    "Documentary",
    "Alarm Test",
    "Alarm",
];

/// The 32 RBDS programme types of NRSC-4-B. Same five bits on the air, an
/// entirely different table: 5 is "Education" in Europe and "Rock" in the US.
const PTY_RBDS: [&str; 32] = [
    "None",
    "News",
    "Information",
    "Sports",
    "Talk",
    "Rock",
    "Classic Rock",
    "Adult Hits",
    "Soft Rock",
    "Top 40",
    "Country",
    "Oldies",
    "Soft",
    "Nostalgia",
    "Jazz",
    "Classical",
    "Rhythm and Blues",
    "Soft Rhythm and Blues",
    "Foreign Language",
    "Religious Music",
    "Religious Talk",
    "Personality",
    "Public",
    "College",
    "Spanish Talk",
    "Spanish Music",
    "Hip Hop",
    "Unassigned",
    "Unassigned",
    "Weather",
    "Emergency Test",
    "Emergency",
];

/// What programme type `pty` means under `standard`. Pass an already-resolved
/// standard — [`RdsStandard::Auto`] is read as [`RdsStandard::Rds`] here rather
/// than guessed at a second time.
pub fn pty_name(pty: u8, standard: RdsStandard) -> &'static str {
    let table = match standard {
        RdsStandard::Rbds => &PTY_RBDS,
        _ => &PTY_RDS,
    };
    table[(pty & 0x1f) as usize]
}

/// The call sign behind a North American programme identification code, if the
/// code is one of the call-letter-derived ones.
///
/// NRSC-4-B maps four-letter calls arithmetically: `K` occupies
/// `0x1000..=0x54A7` and `W` the block straight after it, each letter after the
/// first being a base-26 digit.
///
/// Three-letter calls (`KYW`, and the rest of that generation) are not derived
/// but assigned from a fixed table published in the standard, and that table is
/// not reproduced here. Those stations get `None` and are shown as a bare PI
/// code. A partial table would be worse than no table: it would put a confident,
/// wrong call sign on screen for every station it does not actually cover, and
/// there is nothing in the signal to contradict it.
pub fn pi_callsign(pi: u16) -> Option<String> {
    const K_BASE: u16 = 0x1000;
    const W_BASE: u16 = 0x54A8;
    // 26³ − 1: the largest offset three base-26 letters can express.
    const SPAN: u16 = 17_575;

    let (first, mut n) = if (K_BASE..=K_BASE + SPAN).contains(&pi) {
        ('K', pi - K_BASE)
    } else if (W_BASE..=W_BASE + SPAN).contains(&pi) {
        ('W', pi - W_BASE)
    } else {
        return None;
    };

    let mut call = String::with_capacity(4);
    call.push(first);
    for place in [676u16, 26, 1] {
        call.push((b'A' + (n / place) as u8) as char);
        n %= place;
    }
    Some(call)
}

/// The station's clock, from a group 4A.
///
/// Broadcast as a Modified Julian Date plus a UTC time and the local offset, so
/// the date is already decoded to a calendar here and the offset is kept as the
/// station sent it — some stations run their clock minutes out, and rendering a
/// single "local time" would hide which half of the pair was wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RdsClock {
    pub year: i32,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    /// Local time's offset from UTC in half-hours, signed. India and Newfoundland
    /// are why this is not whole hours.
    pub offset_half_hours: i8,
}

impl RdsClock {
    /// `2026-08-17 14:32 UTC +02:00`.
    pub fn label(&self) -> String {
        let sign = if self.offset_half_hours < 0 { '-' } else { '+' };
        let mins = self.offset_half_hours.unsigned_abs() as u32 * 30;
        format!(
            "{:04}-{:02}-{:02} {:02}:{:02} UTC {sign}{:02}:{:02}",
            self.year,
            self.month,
            self.day,
            self.hour,
            self.minute,
            mins / 60,
            mins % 60,
        )
    }
}

/// RadioText+: the station tagging substrings of its own RadioText so a receiver
/// can show "now playing" as fields rather than as a sentence.
///
/// Title and artist are lifted out because they are what a listener wants and
/// what a display has room for; everything else the station tags is kept as it
/// arrived rather than being dropped or guessed at.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct RtPlus {
    /// Content type 1, `ITEM.TITLE`.
    pub title: Option<String>,
    /// Content type 4, `ITEM.ARTIST`.
    pub artist: Option<String>,
    /// Every other tag the station sent, as `(content type, text)`.
    pub other: Vec<(u8, String)>,
}

impl RtPlus {
    pub fn is_empty(&self) -> bool {
        self.title.is_none() && self.artist.is_none() && self.other.is_empty()
    }
}

/// The RadioText+ content types this build puts a name to.
///
/// The class list runs to 64 entries; only the ones that can be stated with
/// confidence are named, and the rest are shown as their number. A wrong label
/// on a "now playing" field is indistinguishable from a decoding fault, so an
/// unnamed class says so.
pub fn rt_plus_class(code: u8) -> Option<&'static str> {
    Some(match code {
        1 => "Title",
        2 => "Album",
        3 => "Track number",
        4 => "Artist",
        5 => "Composition",
        6 => "Movement",
        7 => "Conductor",
        8 => "Composer",
        9 => "Band",
        10 => "Comment",
        11 => "Genre",
        31 => "Station name (short)",
        32 => "Station name (long)",
        33 => "Programme now",
        34 => "Programme next",
        35 => "Programme part",
        36 => "Programme host",
        39 => "Homepage",
        _ => return None,
    })
}

/// One group as it came off the air, for the diagnostics view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RdsGroupLog {
    /// Blocks A–D. A block that did not check out is left as it was received.
    pub blocks: [u16; 4],
    /// Which blocks checked out, bit 0 = A through bit 3 = D.
    pub valid: u8,
    /// Which of those needed the error corrector to get there, same bit order.
    pub corrected: u8,
}

impl RdsGroupLog {
    /// The group type, as `0A`, `2B`, `15B` — the label the standard uses.
    /// `None` when block B was not recovered, since that is where it is carried.
    pub fn kind(&self) -> Option<String> {
        (self.valid & 0b0010 != 0).then(|| {
            let b = self.blocks[1];
            format!("{}{}", b >> 12, if b & 0x0800 != 0 { 'B' } else { 'A' })
        })
    }
}

/// Counters for the diagnostics view: how much arrived and how much of it was
/// intact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct RdsStats {
    pub groups: u64,
    pub blocks_ok: u64,
    pub blocks_corrected: u64,
    pub blocks_bad: u64,
    /// Groups seen of each type, indexed `type * 2 + version` — so `0A` is 0,
    /// `0B` is 1, `15B` is 31.
    pub group_types: [u32; 32],
}

impl RdsStats {
    /// Fraction of blocks that did not check out, in `0.0..=1.0`.
    ///
    /// Corrected blocks count as errors here, deliberately: they are how close
    /// the signal is running to the edge, and a rising corrected count is the
    /// first sign that a station is about to stop decoding.
    pub fn block_error_rate(&self) -> f32 {
        let seen = self.blocks_ok + self.blocks_corrected + self.blocks_bad;
        if seen == 0 {
            return 0.0;
        }
        (self.blocks_corrected + self.blocks_bad) as f32 / seen as f32
    }
}

/// Everything decoded from one station's RDS, plus the groups that arrived since
/// this snapshot was last taken.
///
/// A snapshot, latest-wins, in the shape of the rest of the engine's state — with
/// one exception. [`RdsData::groups`] is a *delta*: the groups decoded since the
/// previous snapshot, not a running log. A log long enough to be useful is far
/// larger than the rest of this struct and would be resent in full several times
/// a second, so the receiving end accumulates it into its own bounded ring, using
/// [`RdsData::group_seq`] to tell a dropped batch from a decoder that restarted.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct RdsData {
    /// Block sync is held and groups are arriving.
    #[serde(default)]
    pub sync: bool,
    /// Programme identification: the station's identity, and the only field that
    /// is in every group.
    #[serde(default)]
    pub pi: Option<u16>,
    /// Programme type, 0–31. Read against [`RdsStandard`] — see [`pty_name`].
    #[serde(default)]
    pub pty: Option<u8>,
    /// Programme service: the eight-character station name.
    #[serde(default)]
    pub ps: Option<String>,
    /// Programme type name, from group 10A — eight characters the station uses to
    /// say what the bare programme type could not.
    #[serde(default)]
    pub ptyn: Option<String>,
    /// RadioText: 64 characters of free text, whatever the station wants.
    #[serde(default)]
    pub radiotext: Option<String>,
    /// RadioText+ tags into [`RdsData::radiotext`], when the station sends them.
    #[serde(default)]
    pub rt_plus: Option<RtPlus>,
    /// This station carries traffic announcements at all.
    #[serde(default)]
    pub tp: bool,
    /// A traffic announcement is on the air *right now*.
    #[serde(default)]
    pub ta: bool,
    /// Music rather than speech, when the station bothers to say.
    #[serde(default)]
    pub music: Option<bool>,
    /// Alternative frequencies for the same programme, in Hz.
    #[serde(default)]
    pub af: Vec<u32>,
    /// The station's clock, from the last group 4A.
    #[serde(default)]
    pub clock: Option<RdsClock>,
    /// Extended country code, from group 1A. Settles [`RdsStandard::Auto`].
    #[serde(default)]
    pub ecc: Option<u8>,
    #[serde(default)]
    pub stats: RdsStats,
    /// Groups decoded since the previous snapshot — a delta, not a log.
    #[serde(default)]
    pub groups: Vec<RdsGroupLog>,
    /// Absolute index of the first entry in [`RdsData::groups`], counted from
    /// when the decoder started.
    #[serde(default)]
    pub group_seq: u64,
}

impl RdsData {
    /// Nothing has been decoded worth showing. Distinct from "no sync": a
    /// station being tuned through holds sync for a moment without ever
    /// completing a name.
    pub fn is_empty(&self) -> bool {
        self.pi.is_none() && self.ps.is_none() && self.radiotext.is_none() && self.pty.is_none()
    }

    /// The call sign this station's programme identification spells out, or
    /// `None` when that reading is not to be trusted.
    ///
    /// `chosen` is the operator's selection, *not* the resolved standard — see
    /// [`RdsStandard::names_from_pi`] for why the difference matters here and
    /// nowhere else.
    pub fn call_sign(&self, chosen: RdsStandard) -> Option<String> {
        if !chosen.names_from_pi(self.ecc) {
            return None;
        }
        self.pi.and_then(pi_callsign)
    }

    /// What to head the display with: the station name if one has arrived, else
    /// the call sign its PI decodes to under RBDS, else the PI in hex.
    ///
    /// Takes the operator's selection rather than the resolved standard,
    /// because the call sign is the one thing a guessed RBDS may not produce.
    pub fn title(&self, chosen: RdsStandard) -> Option<String> {
        if let Some(ps) = self.ps.as_ref().filter(|s| !s.trim().is_empty()) {
            return Some(ps.trim().to_string());
        }
        let pi = self.pi?;
        if let Some(call) = self.call_sign(chosen) {
            return Some(call);
        }
        Some(format!("{pi:04X}"))
    }
}

/// An AF code as broadcast in group 0A, resolved to a frequency in Hz.
///
/// Codes 1–204 are the VHF band in 100 kHz steps from 87.6 MHz. Everything else
/// is a filler, a count marker, or the LF/MF escape, and yields `None` — an AF
/// list padded with 87.5 MHz because the count byte was read as a frequency is a
/// well-known way to get this wrong.
pub fn af_code_hz(code: u8) -> Option<u32> {
    (1..=204).contains(&code).then(|| 87_600_000 + (code as u32 - 1) * 100_000)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_two_programme_type_tables_disagree_where_they_should() {
        // Same five bits, different meaning — the whole reason the standard is a
        // display-side choice.
        assert_eq!(pty_name(5, RdsStandard::Rds), "Education");
        assert_eq!(pty_name(5, RdsStandard::Rbds), "Rock");
        assert_eq!(pty_name(31, RdsStandard::Rds), "Alarm");
        assert_eq!(pty_name(31, RdsStandard::Rbds), "Emergency");
        // Auto is not a table; it reads as the international one.
        assert_eq!(pty_name(5, RdsStandard::Auto), "Education");
        // Every code in range must name something, including the top bit set.
        for pty in 0..=255u8 {
            assert!(!pty_name(pty, RdsStandard::Rds).is_empty());
            assert!(!pty_name(pty, RdsStandard::Rbds).is_empty());
        }
    }

    #[test]
    fn call_letters_come_back_out_of_the_programme_identification() {
        // The two block bases, and the arithmetic between them.
        assert_eq!(pi_callsign(0x1000).as_deref(), Some("KAAA"));
        assert_eq!(pi_callsign(0x54A8).as_deref(), Some("WAAA"));
        // Last code in each block is ZZZ.
        assert_eq!(pi_callsign(0x1000 + 17_575).as_deref(), Some("KZZZ"));
        assert_eq!(pi_callsign(0x54A8 + 17_575).as_deref(), Some("WZZZ"));
        // A round trip through the standard's own arithmetic: the three letters
        // after the W are base-26 digits, so B, C, D is 1, 2, 3.
        let (b, c, d) = (1u16, 2u16, 3u16);
        assert_eq!(pi_callsign(0x54A8 + 676 * b + 26 * c + d).as_deref(), Some("WBCD"));
        // Outside both blocks: a European PI, and the range above W.
        assert_eq!(pi_callsign(0xD3C2), None);
        assert_eq!(pi_callsign(0x0FFF), None);
        assert_eq!(pi_callsign(0x54A8 + 17_576), None);
    }

    #[test]
    fn every_derived_call_sign_is_four_letters() {
        // 0x1000 through 0x994F is the two blocks back to back, with no gap.
        for pi in 0x1000u16..=0x994F {
            let call = pi_callsign(pi).expect("inside both derived blocks");
            assert_eq!(call.len(), 4, "{pi:04X} gave {call}");
            assert!(call.starts_with('K') || call.starts_with('W'), "{call}");
            assert!(call[1..].bytes().all(|b| b.is_ascii_uppercase()), "{call}");
        }
    }

    #[test]
    fn the_country_code_settles_the_standard_and_the_call_sign_only_guesses() {
        // An ECC in the Americas block wins outright…
        assert_eq!(RdsStandard::Auto.resolve(Some(0xD3C2), Some(0xA0)), RdsStandard::Rbds);
        // …and one outside it overrides a PI that reads as a call sign, which is
        // exactly the collision the fallback cannot resolve on its own.
        assert_eq!(RdsStandard::Auto.resolve(Some(0x54A8), Some(0xE0)), RdsStandard::Rds);
        // No ECC: the call-sign guess is all there is.
        assert_eq!(RdsStandard::Auto.resolve(Some(0x54A8), None), RdsStandard::Rbds);
        assert_eq!(RdsStandard::Auto.resolve(Some(0xD3C2), None), RdsStandard::Rds);
        assert_eq!(RdsStandard::Auto.resolve(None, None), RdsStandard::Rds);
        // A manual choice is never second-guessed.
        assert_eq!(RdsStandard::Rds.resolve(Some(0x54A8), Some(0xA0)), RdsStandard::Rds);
        assert_eq!(RdsStandard::Rbds.resolve(Some(0xD3C2), Some(0xE0)), RdsStandard::Rbds);
    }

    #[test]
    fn a_guessed_rbds_picks_a_table_but_never_spells_out_a_call_sign() {
        // The guess is welcome to choose the programme-type table — one of the
        // two labels is right and the operator can see which. It is not welcome
        // to put four letters on screen that look like the station's name.
        assert!(!RdsStandard::Auto.names_from_pi(None));
        assert!(!RdsStandard::Auto.names_from_pi(Some(0xE0)));
        // Evidence, from either direction, does settle it.
        assert!(RdsStandard::Auto.names_from_pi(Some(0xA0)));
        assert!(RdsStandard::Rbds.names_from_pi(None));
        assert!(!RdsStandard::Rds.names_from_pi(Some(0xA0)));
    }

    #[test]
    fn a_nordic_station_is_not_given_an_american_call_sign() {
        // Issue #173. The derived-call-sign blocks run 0x1000..=0x994F, which is
        // PI country codes 1 through 9 — and Finland is 6, Denmark 9. Before
        // either sends the group 1A that settles it, the fallback reads their
        // identity as a call sign, and on a signal poor enough that 1A takes a
        // while it is the fabricated name that sits on screen.
        for (pi, invented) in [(0x6201u16, "WFBL"), (0x9101, "WWWF")] {
            assert_eq!(pi_callsign(pi).as_deref(), Some(invented), "the trap itself");
            let d = RdsData { pi: Some(pi), ..Default::default() };
            assert_eq!(d.call_sign(RdsStandard::Auto), None);
            assert_eq!(d.title(RdsStandard::Auto).as_deref(), Some(&format!("{pi:04X}")[..]));
        }
        // A North American station that never sends an ECC is the case this
        // costs, and the selector is what it costs: chosen RBDS still names it.
        let d = RdsData { pi: Some(0x54A8), ..Default::default() };
        assert_eq!(d.call_sign(RdsStandard::Rbds).as_deref(), Some("WAAA"));
        // …as does one that does send it.
        let d = RdsData { pi: Some(0x54A8), ecc: Some(0xA0), ..Default::default() };
        assert_eq!(d.call_sign(RdsStandard::Auto).as_deref(), Some("WAAA"));
    }

    #[test]
    fn alternative_frequencies_span_the_broadcast_band_and_reject_the_markers() {
        assert_eq!(af_code_hz(1), Some(87_600_000));
        assert_eq!(af_code_hz(204), Some(107_900_000));
        assert_eq!(af_code_hz(0), None);
        // 224 is "no AF", 225-249 count the list that follows, 250 is the LF/MF
        // escape. None of them is a frequency.
        assert_eq!(af_code_hz(224), None);
        assert_eq!(af_code_hz(225), None);
        assert_eq!(af_code_hz(250), None);
    }

    #[test]
    fn the_block_error_rate_counts_corrections_as_errors() {
        let mut s = RdsStats::default();
        assert_eq!(s.block_error_rate(), 0.0, "nothing seen is not a perfect signal");
        s.blocks_ok = 90;
        s.blocks_corrected = 5;
        s.blocks_bad = 5;
        assert!((s.block_error_rate() - 0.10).abs() < 1e-6);
    }

    #[test]
    fn a_group_names_its_own_type_only_when_block_b_survived() {
        let mut g = RdsGroupLog { blocks: [0x1234, 0x0800, 0, 0], valid: 0b1111, corrected: 0 };
        assert_eq!(g.kind().as_deref(), Some("0B"));
        g.blocks[1] = 0x2000;
        assert_eq!(g.kind().as_deref(), Some("2A"));
        g.blocks[1] = 0xF800;
        assert_eq!(g.kind().as_deref(), Some("15B"));
        g.valid = 0b1101;
        assert_eq!(g.kind(), None, "block B is where the type lives");
    }

    #[test]
    fn the_display_title_falls_back_the_way_a_receiver_does() {
        let mut d = RdsData { pi: Some(0x54A8), ..Default::default() };
        // No name yet: a settled RBDS can name the station from its PI, RDS
        // cannot, and neither can a bare `Auto` — see the test above.
        assert_eq!(d.title(RdsStandard::Rbds).as_deref(), Some("WAAA"));
        assert_eq!(d.title(RdsStandard::Rds).as_deref(), Some("54A8"));
        assert_eq!(d.title(RdsStandard::Auto).as_deref(), Some("54A8"));
        // A name, once it arrives, wins under either.
        d.ps = Some("ROCK FM ".to_string());
        assert_eq!(d.title(RdsStandard::Rbds).as_deref(), Some("ROCK FM"));
        // A blank name is not a name.
        d.ps = Some("        ".to_string());
        assert_eq!(d.title(RdsStandard::Rds).as_deref(), Some("54A8"));
        assert_eq!(RdsData::default().title(RdsStandard::Rds), None);
    }

    #[test]
    fn the_clock_shows_utc_and_the_offset_it_was_sent_with() {
        let c =
            RdsClock { year: 2026, month: 8, day: 17, hour: 14, minute: 32, offset_half_hours: 4 };
        assert_eq!(c.label(), "2026-08-17 14:32 UTC +02:00");
        // Half-hour zones and the negative side both have to read right.
        assert_eq!(RdsClock { offset_half_hours: 11, ..c }.label(), "2026-08-17 14:32 UTC +05:30");
        assert_eq!(RdsClock { offset_half_hours: -7, ..c }.label(), "2026-08-17 14:32 UTC -03:30");
    }
}
