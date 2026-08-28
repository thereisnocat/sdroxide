//! Digital Radio Mondiale receive status.
//!
//! What the decoder knows about the multiplex it is listening to, as one
//! latest-wins snapshot. Unlike [`crate::RdsData`] nothing here is a delta: DRM
//! carries a service label and a scrolling text message that are simply
//! *current*, and a set of sync lights whose whole value is that they show the
//! present state of each stage.

use serde::{Deserialize, Serialize};

/// How one stage of the receive chain is doing, in the four states Dream's own
/// indicators use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum DrmSync {
    /// Nothing arriving at this stage at all.
    #[default]
    Absent,
    /// Arriving, but failing its CRC — the stage is locked onto something wrong.
    CrcError,
    /// Arriving with errors the FEC could not fully repair.
    DataError,
    /// Good.
    Ok,
}

impl DrmSync {
    /// Dream reports these as a plain 0–3; anything else is treated as absent
    /// rather than trusted.
    pub fn from_raw(v: i32) -> Self {
        match v {
            1 => DrmSync::CrcError,
            2 => DrmSync::DataError,
            3 => DrmSync::Ok,
            _ => DrmSync::Absent,
        }
    }

    pub fn is_ok(self) -> bool {
        self == DrmSync::Ok
    }

    /// Single-character indicator, as Dream's console display draws it.
    pub fn glyph(self) -> char {
        match self {
            DrmSync::Absent => '-',
            DrmSync::CrcError => 'x',
            DrmSync::DataError => '!',
            DrmSync::Ok => '•',
        }
    }
}

/// DRM robustness mode: how much guard interval the transmission spends on
/// multipath, from A (a ground-wave channel, most capacity) to D (a badly
/// scattered sky-wave path, least).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DrmRobustness {
    A,
    B,
    C,
    D,
    E,
}

impl DrmRobustness {
    pub fn from_raw(v: i32) -> Option<Self> {
        match v {
            0 => Some(DrmRobustness::A),
            1 => Some(DrmRobustness::B),
            2 => Some(DrmRobustness::C),
            3 => Some(DrmRobustness::D),
            4 => Some(DrmRobustness::E),
            _ => None,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            DrmRobustness::A => "A",
            DrmRobustness::B => "B",
            DrmRobustness::C => "C",
            DrmRobustness::D => "D",
            DrmRobustness::E => "E",
        }
    }
}

/// The six channel widths DRM30/DRM+ are allowed to occupy, in kHz. 9 and
/// 10 kHz — one broadcast channel raster — are what nearly everything on the
/// air actually uses.
pub fn spectrum_occupancy_khz(raw: i32) -> Option<f32> {
    match raw {
        0 => Some(4.5),
        1 => Some(5.0),
        2 => Some(9.0),
        3 => Some(10.0),
        4 => Some(18.0),
        5 => Some(20.0),
        _ => None,
    }
}

/// Audio coding of the selected service.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DrmCodec {
    Aac,
    /// Dream's own extension, not part of the DRM standard.
    Opus,
    /// xHE-AAC (USAC), what most surviving broadcasters moved to.
    XheAac,
    Unknown,
}

impl DrmCodec {
    /// The order in `CAudioParam::EAudCod`, which is
    /// `{AC_AAC=0, AC_OPUS=1, AC_RESERVED=2, AC_xHE_AAC=3, AC_NONE=4}`.
    ///
    /// This used to read 1 and 2 as CELP and HVXC, which is the *original*
    /// DRM ordering. Both speech codecs were withdrawn from the standard and
    /// Dream cannot signal either, so those slots were re-used: an Opus
    /// service was labelled "CELP" and a service with no audio at all was
    /// labelled "Opus". 2 is reserved and 4 is "no audio", and neither is
    /// invented into a codec name here.
    pub fn from_raw(v: i32) -> Self {
        match v {
            0 => DrmCodec::Aac,
            1 => DrmCodec::Opus,
            3 => DrmCodec::XheAac,
            _ => DrmCodec::Unknown,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            DrmCodec::Aac => "AAC",
            DrmCodec::Opus => "Opus",
            DrmCodec::XheAac => "xHE-AAC",
            DrmCodec::Unknown => "?",
        }
    }
}

/// The broadcaster's own clock, when the multiplex carries one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct DrmTime {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
}

/// One service of the multiplex — in practice the one being listened to.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct DrmService {
    /// The station's name for itself, up to 16 characters.
    #[serde(default)]
    pub label: String,
    /// The scrolling text message the audio stream carries alongside the sound.
    #[serde(default)]
    pub text: String,
    /// ISO country and ISO 639 language codes, when signalled.
    #[serde(default)]
    pub country: String,
    #[serde(default)]
    pub language: String,
    #[serde(default)]
    pub service_id: u32,
    #[serde(default)]
    pub bitrate_kbps: f32,
    #[serde(default)]
    pub codec: Option<DrmCodec>,
    /// Whether this build can decode `codec`.
    ///
    /// False means the receiver is locked and reading the multiplex, and the
    /// audio is going nowhere - which every other field on this struct reports
    /// as a perfectly healthy station. xHE-AAC needs libfdk-aac on the system;
    /// see `vendor/fdk-aac/PROVENANCE.md` for why it cannot be built in.
    #[serde(default)]
    pub codec_supported: bool,
    #[serde(default)]
    pub stereo: bool,
}

/// Which of the multiplex's three logical channels a constellation came from.
///
/// They are decoded in this order and carry progressively more: the FAC says
/// what the transmission is, the SDC what its services are, and the MSC the
/// programme itself. The FAC is always 4-QAM; the other two carry whatever the
/// transmission signalled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum DrmChannel {
    Fac,
    Sdc,
    #[default]
    Msc,
}

impl DrmChannel {
    pub const ALL: [DrmChannel; 3] = [DrmChannel::Fac, DrmChannel::Sdc, DrmChannel::Msc];

    pub fn label(self) -> &'static str {
        match self {
            DrmChannel::Fac => "FAC",
            DrmChannel::Sdc => "SDC",
            DrmChannel::Msc => "MSC",
        }
    }

    /// What this channel carries, for the picker's hover text.
    pub fn describes(self) -> &'static str {
        match self {
            DrmChannel::Fac => "Fast Access Channel — what the transmission is. Always 4-QAM",
            DrmChannel::Sdc => "Service Description Channel — what services the multiplex carries",
            DrmChannel::Msc => "Main Service Channel — the programme itself",
        }
    }

    /// The order the C side numbers them in.
    pub fn as_raw(self) -> i32 {
        match self {
            DrmChannel::Fac => 0,
            DrmChannel::Sdc => 1,
            DrmChannel::Msc => 2,
        }
    }
}

/// A snapshot of one logical channel's equalised symbols — the constellation.
///
/// This is the picture of *how well* the signal is being decoded, as opposed to
/// whether it is: tight clusters on the ideal points mean margin, a smeared
/// cloud means the decoder is working near its limit, and a ring means the
/// equaliser has not resolved the channel's phase.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct DrmConstellation {
    #[serde(default)]
    pub channel: DrmChannel,
    /// 4, 16 or 64.
    #[serde(default)]
    pub qam: u8,
    /// Interleaved real/imaginary pairs, normalised the way the standard
    /// defines the constellations — see [`DrmConstellation::ideal_levels`].
    ///
    /// Flat rather than a vector of pairs so the wire carries one
    /// length-prefixed run of floats instead of a length per point.
    #[serde(default)]
    pub points: Vec<f32>,
}

impl DrmConstellation {
    pub fn len(&self) -> usize {
        self.points.len() / 2
    }

    pub fn is_empty(&self) -> bool {
        self.points.len() < 2
    }

    pub fn iter(&self) -> impl Iterator<Item = (f32, f32)> + '_ {
        self.points.chunks_exact(2).map(|p| (p[0], p[1]))
    }

    /// The positive coordinates an ideal symbol of this constellation sits on,
    /// per axis. Every ideal point is a pair drawn from these and their
    /// negatives.
    ///
    /// The standard normalises each constellation to unit average power, so the
    /// levels are the odd integers divided by the RMS of the whole set: 1/√2
    /// for 4-QAM, {1,3}/√10 for 16-QAM and {1,3,5,7}/√42 for 64-QAM.
    pub fn ideal_levels(&self) -> Vec<f32> {
        let (levels, norm): (&[f32], f32) = match self.qam {
            4 => (&[1.0], 2.0),
            16 => (&[1.0, 3.0], 10.0),
            _ => (&[1.0, 3.0, 5.0, 7.0], 42.0),
        };
        let scale = norm.sqrt();
        levels.iter().map(|l| l / scale).collect()
    }

    /// Half-width of a plot that shows the whole constellation with a margin —
    /// one level's worth beyond the outermost point, which is the scale Dream's
    /// own display uses.
    pub fn plot_extent(&self) -> f32 {
        let (outer, norm): (f32, f32) = match self.qam {
            4 => (2.0, 2.0),
            16 => (4.0, 10.0),
            _ => (8.0, 42.0),
        };
        outer / norm.sqrt()
    }
}

/// Everything the DRM decoder knows right now.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct DrmStatus {
    /// The five stages of the receive chain, in the order they lock: the
    /// sample-rate/IO interface, time synchronisation, frame synchronisation,
    /// then the two decoded channels — FAC (which says what the transmission
    /// is) and SDC (which says what the services are) — and finally the audio.
    #[serde(default)]
    pub io: DrmSync,
    #[serde(default)]
    pub time_sync: DrmSync,
    #[serde(default)]
    pub frame_sync: DrmSync,
    #[serde(default)]
    pub fac: DrmSync,
    #[serde(default)]
    pub sdc: DrmSync,
    #[serde(default)]
    pub audio: DrmSync,

    /// Acquisition has finished and the receiver believes it has a signal.
    /// Everything below is only meaningful while this holds.
    #[serde(default)]
    pub locked: bool,
    #[serde(default)]
    pub snr_db: f32,
    #[serde(default)]
    pub if_level_db: f32,
    /// Weighted and plain modulation error ratio of the main service channel.
    #[serde(default)]
    pub wmer_db: f32,
    #[serde(default)]
    pub mer_db: f32,
    /// Where the DRM carrier actually sits inside the decoder's own 48 kHz
    /// window, which is not where the dial is — see the mode's I.F. offset.
    #[serde(default)]
    pub dc_offset_hz: f32,
    /// Residual sample-clock error against the transmitter, in Hz.
    #[serde(default)]
    pub sample_offset_hz: f32,
    /// Doppler spread and delay spread of the path, when the channel estimator
    /// has enough to say.
    #[serde(default)]
    pub doppler_hz: Option<f32>,
    #[serde(default)]
    pub delay_ms: f32,

    #[serde(default)]
    pub robustness: Option<DrmRobustness>,
    #[serde(default)]
    pub bandwidth_khz: Option<f32>,
    /// Two seconds of time interleaving rather than 400 ms: better against
    /// fading, worse to acquire.
    #[serde(default)]
    pub interleaver_long: bool,
    /// Protection levels of the two multiplex parts.
    #[serde(default)]
    pub protection_a: u8,
    #[serde(default)]
    pub protection_b: u8,

    #[serde(default)]
    pub audio_services: u8,
    #[serde(default)]
    pub data_services: u8,
    /// Which service of the multiplex is being decoded, 0-based.
    #[serde(default)]
    pub current_service: u8,
    #[serde(default)]
    pub service: DrmService,
    #[serde(default)]
    pub time: Option<DrmTime>,

    /// The equalised symbols of one logical channel, when the operator has a
    /// constellation on screen.
    ///
    /// `None` the rest of the time, and deliberately: this is hundreds of
    /// floats several times a second, which is worth sending to a remote
    /// client while somebody is watching it and pure waste when nobody is.
    #[serde(default)]
    pub constellation: Option<DrmConstellation>,
}

impl DrmStatus {
    /// The receiver is decoding audio, not merely holding sync on a carrier.
    ///
    /// `audio` on its own is not enough. A codec this build has no decoder for
    /// still produces audio frames, and the null codec Dream substitutes fails
    /// every one of them - which registers as `CrcError`, not `Absent`. So the
    /// whole chain reads healthy while the speaker is silent, and anything
    /// built on this (the top bar's chip, and its "DRM is decoding" hover) says
    /// so. A service whose codec cannot be decoded is not decoding, however
    /// good the signal.
    pub fn decoding(&self) -> bool {
        self.locked
            && self.fac.is_ok()
            && self.service.codec_supported
            && self.audio != DrmSync::Absent
    }

    /// A one-line summary for a status bar: the label if the multiplex has
    /// named itself, else how far the chain has got.
    pub fn summary(&self) -> String {
        if !self.service.label.is_empty() {
            return self.service.label.clone();
        }
        if self.locked {
            "acquiring service".to_string()
        } else if self.time_sync.is_ok() {
            "syncing".to_string()
        } else {
            "no signal".to_string()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `CAudioParam::EAudCod` is `{AC_AAC=0, AC_OPUS=1, AC_RESERVED=2,
    /// AC_xHE_AAC=3, AC_NONE=4}`.
    ///
    /// The table used to be the original DRM ordering, where 1 and 2 were the
    /// CELP and HVXC speech codecs. Both were withdrawn from the standard and
    /// Dream cannot signal either, so an Opus service read as "CELP" and a
    /// service with no audio read as "Opus" — a wrong answer with no way to
    /// tell it was wrong, since only 0 and 3 were ever exercised on air.
    #[test]
    fn the_codec_table_follows_dream_not_the_original_standard() {
        assert_eq!(DrmCodec::from_raw(0), DrmCodec::Aac);
        assert_eq!(DrmCodec::from_raw(1), DrmCodec::Opus);
        assert_eq!(DrmCodec::from_raw(3), DrmCodec::XheAac);
        // Reserved, "no audio", and anything a corrupt SDC invents.
        assert_eq!(DrmCodec::from_raw(2), DrmCodec::Unknown);
        assert_eq!(DrmCodec::from_raw(4), DrmCodec::Unknown);
        assert_eq!(DrmCodec::from_raw(-1), DrmCodec::Unknown);
    }

    /// A codec with no decoder is not "decoding", however good the signal.
    ///
    /// Dream substitutes a null codec, which fails every audio frame — and a
    /// failed frame is `CrcError`, not `Absent`, so the sync row and the top
    /// bar's chip both used to report a healthy decode into a silent speaker.
    #[test]
    fn a_codec_with_no_decoder_is_not_decoding() {
        let mut s = DrmStatus {
            locked: true,
            fac: DrmSync::Ok,
            audio: DrmSync::CrcError,
            ..Default::default()
        };
        s.service.codec = Some(DrmCodec::XheAac);

        s.service.codec_supported = false;
        assert!(!s.decoding(), "no decoder for the signalled codec");

        s.service.codec_supported = true;
        assert!(s.decoding(), "everything else about this station is healthy");
    }
}
