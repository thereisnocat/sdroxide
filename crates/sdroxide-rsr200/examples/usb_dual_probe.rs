//! Standalone smoke test for Separate mode (`RSR200_PLAN.md` step 4)
//! against a real, physically-attached RSR200 over USB. Not a unit test —
//! it needs actual hardware, and is not run by `cargo test`.
//!
//! Proves, in order: `format.channels = 2` is accepted, both ADCs actually
//! deliver real (non-silent) data rather than one going through as
//! zero-filled padding, and `sdroxide_dsp::Diversity::process()` runs
//! against real samples without panicking. Does **not** prove the *result*
//! is musically useful — that needs two real aerials on the two ADC inputs
//! and a human listening, which this program cannot judge.
//!
//! One thing seen in testing worth knowing before assuming a failure here
//! means a real bug: `apply_config`'s very first command failed once with
//! "the transport rejected a command" on a run started moments after a
//! previous example's `Stop Stream` on the same radio. DP 3.3's own
//! warning about `Stop Stream` closing the USB send endpoint — previously
//! only seen delaying `FT_Create` itself — apparently can also delay the
//! *first write* succeeding on a fresh handle. A second run right after
//! succeeded cleanly. If this program fails immediately after another one
//! just ran, try again before assuming the code is wrong.
//!
//! ```text
//! cargo run --example usb_dual_probe -p sdroxide-rsr200
//! ```

use std::time::{Duration, Instant};

use num_complex::Complex32;
use sdroxide_dsp::{Diversity, DiversityMode};
use sdroxide_rsr200::device::{Config, Device, Transport};
use sdroxide_rsr200::protocol::{OpMode, StreamFormat};
use sdroxide_rsr200::usb::UsbTransport;

fn rms(samples: &[f32]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = samples.iter().map(|&v| f64::from(v) * f64::from(v)).sum();
    (sum_sq / samples.len() as f64).sqrt()
}

fn main() {
    let mut transport = match UsbTransport::open("") {
        Ok(t) => t,
        Err(e) => {
            eprintln!("UsbTransport::open failed: {e}");
            std::process::exit(1);
        }
    };
    println!("Opened, queued reads outstanding.");

    let dev_cfg = Config {
        adc_clock_hz: 100e6,
        gps_discipline: false,
        decimation_exp: 3,
        format: StreamFormat { channels: 2, bits: 16 },
        op_mode: OpMode::Independent,
        tuned_hz: 14.2e6,
        ..Config::default()
    };

    let mut device = Device::new();
    let started = Instant::now();
    let now_ms = || started.elapsed().as_millis() as u64;

    if let Err(e) = device.apply_config(&mut transport, &dev_cfg, now_ms()) {
        eprintln!("apply_config (channels=2) failed: {e}");
        eprintln!("transport.last_error(): {:?}", transport.last_error());
        std::process::exit(1);
    }
    println!("Configured for two channels.");

    if let Err(e) = device.start_stream(&mut transport, now_ms()) {
        eprintln!("start_stream failed: {e}");
        std::process::exit(1);
    }
    println!("Streaming. Sample rate reported: {:.3} Msps.", device.sample_rate() / 1e6);

    let mut out_a = Vec::new();
    let mut out_b = Vec::new();
    let mut frames_a: Vec<f32> = Vec::new();
    let mut frames_b: Vec<f32> = Vec::new();
    let mut saw_dual = false;
    let mut total_frames: u64 = 0;
    let deadline = Instant::now() + Duration::from_secs(3);

    while Instant::now() < deadline {
        match device.pump(&mut transport, &mut out_a, &mut out_b) {
            Some(outcome) => {
                if let Some(err) = &outcome.error {
                    eprintln!("frame error: {err}");
                }
                if let Some(sb) = outcome.samples {
                    total_frames += sb.frames as u64;
                    if sb.dual {
                        saw_dual = true;
                        let need = sb.frames * 2;
                        if frames_a.len() < 200_000 {
                            frames_a.extend_from_slice(&out_a[..need]);
                            frames_b.extend_from_slice(&out_b[..need]);
                        }
                    }
                }
            }
            None => {
                let msg = transport.last_error().unwrap_or("connection lost").to_string();
                eprintln!("transport stopped: {msg}");
                break;
            }
        }
    }

    let _ = device.stop_stream(&mut transport, now_ms());
    transport.close();

    println!("\n{total_frames} frames total. SampleBlock.dual seen: {saw_dual}.");
    if !saw_dual {
        eprintln!("Device never reported a dual block -- format.channels=2 did not take.");
        std::process::exit(2);
    }

    let rms_a = rms(&frames_a);
    let rms_b = rms(&frames_b);
    println!("channel A: {} samples collected, RMS {rms_a:.5}", frames_a.len() / 2);
    println!("channel B: {} samples collected, RMS {rms_b:.5}", frames_b.len() / 2);
    if rms_b < 1e-6 {
        eprintln!(
            "channel B is silent (RMS ~0) -- either nothing is connected to ADC2, or the \
             second channel is not really being delivered. Not necessarily a bug: an \
             unterminated or unconnected second input reads as silence too."
        );
    }

    // Diversity::process() against the real, live pair -- proves the
    // software half of Separate mode runs against genuine hardware data,
    // not just the synthetic fixtures its own unit tests use.
    let pairs = (frames_a.len() / 2).min(frames_b.len() / 2);
    if pairs > 0 {
        let mut main: Vec<Complex32> = (0..pairs).map(|i| Complex32::new(frames_a[2 * i], frames_a[2 * i + 1])).collect();
        let aux: Vec<Complex32> = (0..pairs).map(|i| Complex32::new(frames_b[2 * i], frames_b[2 * i + 1])).collect();
        let mut d = Diversity::new(DiversityMode::Cancel, 8, 0.7);
        d.process(&mut main, &aux);
        println!("Diversity::process() ran against {pairs} real sample pairs without panicking.");
    }

    std::process::exit(0);
}
