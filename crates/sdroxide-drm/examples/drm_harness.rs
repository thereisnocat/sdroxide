//! Bench harness: decode a DRM recording and print what the receiver makes of it.
//!
//! Recordings of DRM broadcasts are 48 kHz real signals off a receiver's I.F.,
//! which is not the zero-IF I/Q the radio feeds [`DrmDemod`]. Both paths are
//! driven here — `--real` puts the recording straight into the decoder the way
//! Dream's own file input would, and the default converts it to baseband first
//! so the path the application actually uses is the one under test.
//!
//!     cargo run -p sdroxide-drm --example drm_harness -- recording.wav
//!
//! `--out decoded.wav` writes what came out of the decoder, which is the only
//! way to tell audio that decoded from audio the codec concealed its way
//! through: a misframed stream still returns samples, just not the broadcast's.
//!
//! Sample recordings live at
//! <https://sourceforge.net/projects/drm/files/samples/DRM%20sample%20recordings/>.

use num_complex::Complex32;
use rustfft::FftPlanner;
use sdroxide_drm::{DrmDemod, DrmWorker};
use sdroxide_dsp::Demodulator;
use sdroxide_types::DrmChannel;

const RATE: f64 = 48_000.0;

fn main() {
    tracing_subscriber_init();
    let mut args = std::env::args().skip(1);
    let mut path = None;
    let mut real = false;
    let mut shift_hz = f64::NAN;
    let mut out_path = None;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--real" => real = true,
            "--shift" => shift_hz = args.next().and_then(|v| v.parse().ok()).unwrap_or(f64::NAN),
            "--out" => out_path = args.next(),
            _ => path = Some(a),
        }
    }
    let path =
        path.expect("usage: drm_harness [--real] [--shift HZ] [--out decoded.wav] recording.wav");

    let mut reader = hound::WavReader::open(&path).expect("open the recording");
    let spec = reader.spec();
    let samples: Vec<f32> =
        reader.samples::<i16>().map(|s| s.expect("read sample") as f32 / 32_768.0).collect();
    let channels = spec.channels as usize;
    let mono: Vec<f32> = samples.chunks(channels).map(|c| c[0]).collect();
    eprintln!(
        "{path}: {} ch, {} Hz, {:.1} s",
        spec.channels,
        spec.sample_rate,
        mono.len() as f64 / spec.sample_rate as f64
    );
    assert_eq!(spec.sample_rate as f64, RATE, "the harness expects a 48 kHz recording");

    if real {
        run_real(&mono, out_path.as_deref());
    } else {
        run_iq(&mono, shift_hz);
    }
}

/// Write the decoder's stereo output, so it can be listened to or measured.
fn write_wav(path: &str, interleaved: &[i16]) {
    let spec = hound::WavSpec {
        channels: 2,
        sample_rate: RATE as u32,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut w = hound::WavWriter::create(path, spec).expect("create the output");
    for &s in interleaved {
        w.write_sample(s).expect("write");
    }
    w.finalize().expect("finalize");
    eprintln!("wrote {} samples per channel to {path}", interleaved.len() / 2);
}

/// Feed the recording as Dream's own file input would: a real signal in both
/// channels, with the decoder left to find the carrier anywhere in the band.
fn run_real(mono: &[f32], out_path: Option<&str>) {
    let worker = DrmWorker::new(false, false).expect("start the decoder");
    worker.set_constellation(Some(DrmChannel::Msc));
    let mut interleaved = Vec::with_capacity(mono.len() * 2);
    for &s in mono {
        let v = (s * 32_767.0) as i16;
        interleaved.push(v);
        interleaved.push(v);
    }
    let mut audio = vec![0i16; 8192];
    let mut frames = 0usize;
    let mut decoded = Vec::new();
    for block in interleaved.chunks(4800) {
        while worker.push(block) > 0 {
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        loop {
            let n = worker.pop(&mut audio);
            if n == 0 {
                break;
            }
            if out_path.is_some() {
                decoded.extend_from_slice(&audio[..n]);
            }
            frames += n / 2;
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    report(&worker.status(), frames);
    if let Some(p) = out_path {
        write_wav(p, &decoded);
    }
}

/// Convert the recording to the zero-IF baseband the receive chain produces,
/// then drive the real demodulator with it.
///
/// The recording's carrier is wherever the original receiver's I.F. put it, so
/// it is measured first — a pass of the real path reports it as
/// `DrmStatus::dc_offset_hz`, relative to Dream's own 6 kHz virtual I.F.
fn run_iq(mono: &[f32], shift_hz: f64) {
    let carrier = if shift_hz.is_finite() { shift_hz } else { measure_carrier(mono) };
    eprintln!("carrier at {carrier:.1} Hz in the recording; shifting to baseband");

    let baseband = to_baseband(mono, carrier);
    let mut demod = DrmDemod::new(RATE);
    Demodulator::set_drm_constellation(&mut demod, Some(DrmChannel::Msc));
    let mut out = Vec::new();
    let mut frames = 0usize;
    let mut status = Default::default();
    for block in baseband.chunks(4800) {
        out.clear();
        demod.process(block, &mut out);
        frames += out.len();
        if let Some(s) = demod.take_drm() {
            status = s;
        }
        // The decoder runs on its own thread; give it the wall-clock time a
        // live receiver would have.
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    if let Some(s) = demod.take_drm() {
        status = s;
    }
    report(&status, frames);
}

/// One pass of the real path, to find where in the 48 kHz band the DRM signal
/// sits. Dream reports the figure against its own virtual I.F.
fn measure_carrier(mono: &[f32]) -> f64 {
    let worker = DrmWorker::new(false, false).expect("start the decoder");
    let mut interleaved = Vec::new();
    // Twelve seconds is comfortably more than acquisition needs.
    for &s in mono.iter().take(12 * RATE as usize) {
        let v = (s * 32_767.0) as i16;
        interleaved.push(v);
        interleaved.push(v);
    }
    let mut sink = vec![0i16; 8192];
    for block in interleaved.chunks(4800) {
        while worker.push(block) > 0 {
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        while worker.pop(&mut sink) > 0 {}
        std::thread::sleep(std::time::Duration::from_millis(4));
    }
    let st = worker.status();
    assert!(st.locked, "the decoder did not lock onto the recording");
    st.dc_offset_hz as f64
}

/// Real signal to complex baseband: the analytic signal, mixed down so the
/// carrier lands at DC.
fn to_baseband(mono: &[f32], carrier_hz: f64) -> Vec<Complex32> {
    let n = mono.len().next_power_of_two();
    let mut planner = FftPlanner::<f32>::new();
    let fwd = planner.plan_fft_forward(n);
    let inv = planner.plan_fft_inverse(n);

    let mut buf: Vec<Complex32> = mono.iter().map(|&s| Complex32::new(s, 0.0)).collect();
    buf.resize(n, Complex32::new(0.0, 0.0));
    fwd.process(&mut buf);
    // Zero the negative frequencies and double the positive ones — the analytic
    // signal, with the same real part and no image to alias on the mixdown.
    for (k, v) in buf.iter_mut().enumerate() {
        if k == 0 || (n.is_multiple_of(2) && k == n / 2) {
            // leave as is
        } else if k < n / 2 {
            *v *= 2.0;
        } else {
            *v = Complex32::new(0.0, 0.0);
        }
    }
    inv.process(&mut buf);
    let scale = 1.0 / n as f32;
    let w = -2.0 * std::f64::consts::PI * carrier_hz / RATE;
    buf.truncate(mono.len());
    buf.iter()
        .enumerate()
        .map(|(i, z)| {
            let ph = w * i as f64;
            *z * scale * Complex32::new(ph.cos() as f32, ph.sin() as f32)
        })
        .collect()
}

/// How far the decoded symbols sit from the constellation they should land on.
///
/// This is the check on the scaling: the standard normalises each constellation
/// to unit average power, and if that assumption were wrong the cloud would
/// come out at some other radius entirely and the error here would be of the
/// same order as the spacing rather than a small fraction of it.
fn report_constellation(c: &sdroxide_types::DrmConstellation) {
    let levels = c.ideal_levels();
    let axis =
        |v: f32| levels.iter().map(|&l| (v - l.copysign(v)).abs()).fold(f32::INFINITY, f32::min);
    let mut sum_sq = 0.0f64;
    let mut worst = 0.0f32;
    let mut rms_mag = 0.0f64;
    for (re, im) in c.iter() {
        let (dx, dy) = (axis(re), axis(im));
        let d = (dx * dx + dy * dy).sqrt();
        sum_sq += f64::from(d) * f64::from(d);
        worst = worst.max(d);
        rms_mag += f64::from(re * re + im * im);
    }
    let n = c.len().max(1) as f64;
    let rms = (sum_sq / n).sqrt();
    let half_spacing = levels.first().copied().unwrap_or(0.5);
    println!(
        "constel {}-QAM {} pts \u{00b7} rms |z|={:.3} (ideal 1.000) \u{00b7}          rms error={:.3} worst={:.3} (half-spacing {:.3})",
        c.qam,
        c.len(),
        (rms_mag / n).sqrt(),
        rms,
        worst,
        half_spacing,
    );
}

fn report(st: &sdroxide_types::DrmStatus, audio_frames: usize) {
    println!("\n--- decode ---");
    println!(
        "sync   IO:{} time:{} frame:{} FAC:{} SDC:{} audio:{}",
        st.io.glyph(),
        st.time_sync.glyph(),
        st.frame_sync.glyph(),
        st.fac.glyph(),
        st.sdc.glyph(),
        st.audio.glyph()
    );
    println!(
        "signal locked={} SNR={:.1} dB  mode={} bw={} kHz  offset={:.1} Hz",
        st.locked,
        st.snr_db,
        st.robustness.map(|r| r.label()).unwrap_or("?"),
        st.bandwidth_khz.map(|b| b.to_string()).unwrap_or_else(|| "?".into()),
        st.dc_offset_hz,
    );
    println!(
        "service \"{}\" [{}/{}] {:.2} kbps {} {}",
        st.service.label,
        st.service.country,
        st.service.language,
        st.service.bitrate_kbps,
        st.service.codec.map(|c| c.label()).unwrap_or("?"),
        if st.service.stereo { "stereo" } else { "mono" },
    );
    // The distinction between "no audio because the signal is bad" and "no
    // audio because nothing here can decode this codec".
    if st.service.codec.is_some() && !st.service.codec_supported {
        println!("codec   NOT DECODABLE — xHE-AAC needs libfdk-aac on the system");
    }
    if !st.service.text.is_empty() {
        println!("text    {}", st.service.text);
    }
    println!("audio   {:.1} s decoded", audio_frames as f64 / RATE);
    if let Some(c) = st.constellation.as_ref() {
        report_constellation(c);
    }
}

/// The harness printed nothing from `tracing`, so every `warn!` the decode path
/// emits — including the one that says a codec has no decoder — was invisible
/// exactly where it would have been most use.
fn tracing_subscriber_init() {
    if std::env::var("RUST_LOG").is_err() {
        // SAFETY: single-threaded, before anything else starts.
        unsafe { std::env::set_var("RUST_LOG", "sdroxide_drm=debug") };
    }
    let _ = tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_writer(std::io::stderr)
        .try_init();
}
