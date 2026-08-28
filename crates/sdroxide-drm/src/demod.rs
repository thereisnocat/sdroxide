//! [`DrmDemod`] — DRM as a demodulator: channel-rate complex baseband in,
//! decoded programme audio out.
//!
//! It sits where `AmDemod` or `WfmDemod` would, because that is what it is —
//! RF in, sound out, no keyboard, no transmit, no QSO. The differences from an
//! analog demod are all consequences of the decode running on its own thread:
//!
//! * the audio for a block is not made from that block's samples. DRM's time
//!   interleaving alone is 400 ms or 2 s, so what comes back now was
//!   transmitted seconds ago;
//! * the decoder produces audio at the *transmitter's* rate, not ours, so the
//!   two are rate-matched here rather than assumed equal;
//! * there is a status snapshot to hand up, which is what [`take_drm`] is for.
//!
//! [`take_drm`]: sdroxide_dsp::Demodulator::take_drm

use num_complex::Complex32;
use sdroxide_dsp::{ComplexResampler, Demodulator};
use sdroxide_types::{DrmChannel, DrmCodec, DrmStatus};
use tracing::{debug, warn};

use crate::{AUDIO_RATE, DrmWorker, SIGNAL_RATE};

/// Where the DRM channel's own reference carrier sits relative to the dial.
///
/// Zero, and worth saying so: for the 9 and 10 kHz occupancies that every
/// broadcast on the air uses, the carrier set is symmetric about the reference
/// (Kmin = −Kmax in the standard's tables), so the reference *is* the dial and
/// the channel needs no offset. Dream then shifts the baseband we hand it up to
/// its own 6 kHz virtual intermediate frequency internally — which is why this
/// is not that 6 kHz.
pub const DRM_IF_OFFSET_HZ: f32 = 0.0;

/// Target RMS of the samples handed to the decoder, as a fraction of full
/// scale.
///
/// The decoder takes 16-bit samples, so the level it is fed decides how much of
/// that word the signal uses. Too low and quantisation eats the weak carriers;
/// too high and the OFDM peaks clip, which is worse — a DRM waveform is the sum
/// of a couple of hundred carriers and runs about 10 dB peak-to-RMS. This
/// leaves ~26 dB of headroom above RMS.
const TARGET_RMS: f32 = 0.05;

/// One-pole coefficient for the level estimate, ~0.5 s at 48 kHz. Slow on
/// purpose: it must not follow the fading it is there to ride out.
const LEVEL_ALPHA: f32 = 1.0 / (0.5 * SIGNAL_RATE as f32);

/// Audio held back before playback, and the point at which a backlog is
/// dropped rather than allowed to become latency. In steady state the decoder
/// produces almost exactly real time and neither applies.
const MAX_BACKLOG_FRAMES: usize = (0.6 * AUDIO_RATE) as usize;
const TARGET_BACKLOG_FRAMES: usize = (0.3 * AUDIO_RATE) as usize;

pub struct DrmDemod {
    /// `None` when the decoder could not start — the mode then behaves as a
    /// silent receiver rather than taking the radio down.
    worker: Option<DrmWorker>,
    channel_rate: f64,
    resampler: Option<ComplexResampler>,

    /// Channel IQ resampled to the decoder's rate.
    rs_buf: Vec<Complex32>,
    /// ...and the same, as the interleaved 16-bit pairs the decoder reads.
    iq_buf: Vec<i16>,
    /// Interleaved stereo audio taken back from the decoder.
    audio_buf: Vec<i16>,
    /// The stereo difference matching the samples the last `process` appended.
    side: Vec<f32>,

    /// Mean square of the resampled input, for the level normalisation.
    level: f32,
    /// Post-filter signal power for the S-meter.
    power: f32,
    /// Fractional part of the input-to-audio sample accounting.
    frame_debt: f64,

    status: DrmStatus,
    /// Cleared by `take_drm`, so the engine only republishes what has moved.
    status_dirty: bool,
    /// The codec last warned about, so the log gets one line per station rather
    /// than one every 250 ms.
    warned_codec: Option<DrmCodec>,
}

impl DrmDemod {
    pub fn new(channel_rate: f64) -> Self {
        let worker = match DrmWorker::new(true, false) {
            Ok(w) => {
                debug!(channel_rate, "DRM decoder attached to the receive chain");
                Some(w)
            }
            Err(e) => {
                warn!(?e, "could not start the DRM decoder; the mode will be silent");
                None
            }
        };
        DrmDemod {
            worker,
            channel_rate,
            resampler: ComplexResampler::new(channel_rate, SIGNAL_RATE),
            rs_buf: Vec::new(),
            iq_buf: Vec::new(),
            audio_buf: Vec::new(),
            side: Vec::new(),
            level: 0.0,
            power: 0.0,
            frame_debt: 0.0,
            status: DrmStatus::default(),
            status_dirty: true,
            warned_codec: None,
        }
    }

    /// Re-acquire. The demodulator cannot see a retune — the DDC ahead of it
    /// absorbs that — so the engine says.
    pub fn restart(&mut self) {
        if let Some(w) = self.worker.as_ref() {
            w.restart();
        }
    }

    /// Decode a different service of the multiplex.
    pub fn select_service(&mut self, service: u8) {
        if let Some(w) = self.worker.as_ref() {
            w.select_service(service);
        }
    }

    /// Resample the block to the decoder's rate, normalise it and queue it.
    fn feed(&mut self, iq: &[Complex32]) {
        self.rs_buf.clear();
        match self.resampler.as_mut() {
            Some(rs) => rs.push(iq, &mut self.rs_buf),
            None => self.rs_buf.extend_from_slice(iq),
        }
        if self.rs_buf.is_empty() {
            return;
        }

        for z in &self.rs_buf {
            let p = z.re * z.re + z.im * z.im;
            self.level += LEVEL_ALPHA * (p - self.level);
        }
        // A silent input would divide by zero and then clip on the first real
        // sample; hold the gain where it was until there is something to measure.
        let rms = self.level.sqrt();
        let gain = if rms > 1e-9 { (TARGET_RMS / rms).clamp(1.0e-3, 1.0e4) } else { 0.0 };

        self.iq_buf.clear();
        self.iq_buf.reserve(self.rs_buf.len() * 2);
        for z in &self.rs_buf {
            self.iq_buf.push(to_i16(z.re * gain));
            self.iq_buf.push(to_i16(z.im * gain));
        }
        if let Some(w) = self.worker.as_ref() {
            let dropped = w.push(&self.iq_buf);
            if dropped > 0 {
                warn!(dropped, "the DRM decoder fell behind; samples were dropped");
            }
        }
    }
}

#[inline]
fn to_i16(v: f32) -> i16 {
    (v * 32_767.0).clamp(-32_767.0, 32_767.0) as i16
}

impl Demodulator for DrmDemod {
    fn process(&mut self, iq: &[Complex32], out: &mut Vec<f32>) {
        self.side.clear();
        if iq.is_empty() {
            return;
        }

        let mut p = 0.0f32;
        for z in iq {
            p += z.re * z.re + z.im * z.im;
        }
        self.power = p / iq.len() as f32;

        self.feed(iq);

        let Some(worker) = self.worker.as_ref() else {
            return;
        };

        // How much audio this block is worth in real time. The decoder's output
        // is paced by the broadcast, so this is what keeps the two clocks
        // together instead of playing back whatever happens to have arrived.
        self.frame_debt += iq.len() as f64 * AUDIO_RATE / self.channel_rate;
        let want = self.frame_debt as usize;
        self.frame_debt -= want as f64;
        if want == 0 {
            return;
        }

        // A backlog is latency, and DRM has more than enough of that already.
        // It only builds if the decoder catches up in a burst after acquiring.
        let mut available = worker.audio_available() / 2;
        if available > MAX_BACKLOG_FRAMES {
            let drop = available - TARGET_BACKLOG_FRAMES;
            self.audio_buf.resize(drop * 2, 0);
            let popped = worker.pop(&mut self.audio_buf);
            debug!(frames = popped / 2, "dropped a DRM audio backlog");
            available -= popped / 2;
        }

        let take = want.min(available);
        out.reserve(want);
        self.side.reserve(want);
        if take > 0 {
            self.audio_buf.resize(take * 2, 0);
            let got = worker.pop(&mut self.audio_buf) / 2;
            for f in 0..got {
                let l = self.audio_buf[2 * f] as f32 / 32_768.0;
                let r = self.audio_buf[2 * f + 1] as f32 / 32_768.0;
                out.push((l + r) * 0.5);
                self.side.push((l - r) * 0.5);
            }
            // Whatever the queue could not supply is silence, not a gap: the
            // block still has to be as long as real time says.
            for _ in got..want {
                out.push(0.0);
                self.side.push(0.0);
            }
        } else {
            out.resize(out.len() + want, 0.0);
            self.side.resize(want, 0.0);
        }
    }

    /// Nothing to do: the DRM channel's width is fixed by the transmission's
    /// spectrum occupancy, which the decoder reads out of the FAC rather than
    /// being told. The operator's filter edges still set what the panadapter
    /// draws and what the S-meter measures.
    fn set_filter(&mut self, _lo_hz: f32, _hi_hz: f32) {}

    fn audio_rate(&self) -> f64 {
        AUDIO_RATE
    }

    fn power_dbfs(&self) -> f32 {
        if self.power <= 1e-20 { -200.0 } else { 10.0 * self.power.log10() }
    }

    fn take_side(&mut self, out: &mut Vec<f32>) -> bool {
        if !self.status.service.stereo || self.side.is_empty() {
            return false;
        }
        out.extend_from_slice(&self.side);
        true
    }

    fn stereo_locked(&self) -> bool {
        self.status.service.stereo
    }

    fn reset_drm(&mut self) {
        self.restart();
    }

    fn set_drm_service(&mut self, service: u8) {
        self.select_service(service);
    }

    fn set_drm_constellation(&mut self, channel: Option<DrmChannel>) {
        if let Some(w) = self.worker.as_ref() {
            w.set_constellation(channel);
        }
    }

    fn take_drm(&mut self) -> Option<DrmStatus> {
        if let Some(w) = self.worker.as_ref() {
            let now = w.status();
            if now != self.status {
                self.status = now;
                self.status_dirty = true;
            }
            /* Nothing else in the receive chain reports this. Every indicator
            reads healthy - locked, FAC, SDC, a service label - and the audio
            is silence, so without a line here the log of a working install
            and the log of a station nobody can hear are identical. */
            match self.status.service.codec {
                Some(c) if self.status.locked && !self.status.service.codec_supported => {
                    if self.warned_codec != Some(c) {
                        self.warned_codec = Some(c);
                        warn!(
                            codec = c.label(),
                            service = %self.status.service.label,
                            "this DRM service's audio codec has no decoder here; \
                             xHE-AAC needs libfdk-aac installed"
                        );
                    }
                }
                _ => self.warned_codec = None,
            }
        }
        if !std::mem::take(&mut self.status_dirty) {
            return None;
        }
        Some(self.status.clone())
    }
}
