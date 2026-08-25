//! Standalone smoke test against a real, physically-attached RSR200 over
//! USB. Not a unit test — it needs actual hardware, and is not run by
//! `cargo test`. Run by hand while bringing up the USB transport
//! (`RSR200_PLAN.md` step 7), the same way the SDR++ sibling
//! implementation's own `test_usb_live.cpp` was — this is a direct port of
//! it, minus the raw open/Start-Stream/Read-Version calls it makes by hand,
//! since this crate's [`Device`] already owns command numbering and
//! embedded-reply parsing.
//!
//! Proves, in order: the D3XX driver enumerates the radio,
//! [`UsbTransport::open`] can open it and keep reads queued, [`Device`] can
//! configure it and start the stream, and real packets come back with a
//! plausible sample rate and — usually within the first second — a version
//! reply.
//!
//! ```text
//! cargo run --example usb_live_probe -p sdroxide-rsr200
//! ```
//!
//! DP 3.3: Stop Stream closes the USB send endpoint entirely, so after this
//! program's clean exit the radio may need a moment (or a replug) before a
//! second run's `FT_Create` succeeds.

use std::time::{Duration, Instant};

use sdroxide_rsr200::device::{Config, Device, Transport};
use sdroxide_rsr200::protocol::{OpMode, ReplyKind, StreamFormat};
use sdroxide_rsr200::usb::UsbTransport;

fn main() {
    let devices = match sdroxide_rsr200::list_usb_devices() {
        Ok(d) => d,
        Err(e) => {
            eprintln!("driver load failed: {e}");
            std::process::exit(1);
        }
    };
    println!("D3XX devices found: {}", devices.len());
    for d in &devices {
        println!("  {} ({}){}", d.description, d.serial, if d.superspeed { ", SuperSpeed" } else { "" });
    }
    if devices.is_empty() {
        eprintln!("No D3XX device enumerated -- is the radio connected and powered?");
        std::process::exit(1);
    }

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
        format: StreamFormat { channels: 1, bits: 16 },
        op_mode: OpMode::Independent,
        tuned_hz: 14.2e6,
        ..Config::default()
    };

    let mut device = Device::new();
    let started = Instant::now();
    let now_ms = || started.elapsed().as_millis() as u64;

    if let Err(e) = device.apply_config(&mut transport, &dev_cfg, now_ms()) {
        eprintln!("apply_config failed: {e}");
        std::process::exit(1);
    }
    println!("Configured (requested {:.3} Msps).", dev_cfg.adc_clock_hz / f64::from(1u32 << (dev_cfg.decimation_exp + 1)) / 1e6);

    if let Err(e) = device.start_stream(&mut transport, now_ms()) {
        eprintln!("start_stream failed: {e}");
        std::process::exit(1);
    }
    println!("Streaming. Sample rate reported: {:.3} Msps.", device.sample_rate() / 1e6);

    let mut out_a = Vec::new();
    let mut out_b = Vec::new();
    let mut frames: u64 = 0;
    let mut gap_events: u64 = 0;
    let mut saw_version = false;
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut last_print = Instant::now();

    while Instant::now() < deadline {
        match device.pump(&mut transport, &mut out_a, &mut out_b) {
            Some(outcome) => {
                if let Some(err) = &outcome.error {
                    eprintln!("frame error: {err}");
                }
                if let Some(sb) = outcome.samples {
                    frames += sb.frames as u64;
                    if sb.sequence_gap {
                        gap_events += 1;
                    }
                }
                for r in &outcome.replies {
                    if r.kind == ReplyKind::Version && !saw_version {
                        saw_version = true;
                        println!("Version reply: serial={} firmware={}", r.serial, r.firmware);
                    }
                }
            }
            None => {
                let msg = transport.last_error().unwrap_or("connection lost").to_string();
                eprintln!("transport stopped: {msg}");
                break;
            }
        }
        if last_print.elapsed() >= Duration::from_millis(500) {
            println!("frames so far: {frames} (gap events: {gap_events})");
            last_print = Instant::now();
        }
    }

    let elapsed = started.elapsed();
    let _ = device.stop_stream(&mut transport, now_ms());
    transport.close();

    let sps = frames as f64 / elapsed.as_secs_f64();
    println!(
        "\n{frames} frames in {:.2} s ({sps:.0} sps). Gap events: {gap_events}. Version reply seen: {saw_version}.",
        elapsed.as_secs_f64()
    );
    std::process::exit(if frames > 0 { 0 } else { 2 });
}
