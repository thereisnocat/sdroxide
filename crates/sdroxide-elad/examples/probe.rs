//! Identify an ELAD receiver and print everything a first bug report needs.
//!
//! ```text
//! cargo run -p sdroxide-elad --example probe
//! PROBE_FREQ=9750000 cargo run -p sdroxide-elad --example probe
//! PROBE_RATE=384000 cargo run -p sdroxide-elad --example probe
//! ```
//!
//! This backend was written from ELAD's own GNU Radio module rather than on a
//! bench, so this tool exists to be *pasted into an issue*. It lists the bus,
//! opens the first device, streams briefly, and dumps the whole session trace —
//! every vendor request with its reply length, the EEPROM calibration, the
//! tuning arithmetic, and the first bulk bytes decoded as `(re, im)` pairs.
//!
//! Two questions this driver cannot answer for itself are what it is really for:
//!
//! * **What rate is the device in?** On an FDM-S1 or FDM-S2 that is which FPGA
//!   image ELAD's `elad-firmware` loader put in it, and `PROBE_RATE` asks for
//!   one; on an FDM-DUO nothing selects the decimation at all. Either way the
//!   measured throughput printed below is the evidence.
//! * **Do the samples arrive I before Q?** Point the receiver at a known strong
//!   carrier (`PROBE_FREQ`, in Hz — a local broadcast station is ideal); the
//!   decoded sample line in the trace is what settles it.

use std::time::{Duration, Instant};

use sdroxide_types::{ELAD_SAMPLE_RATES, EladConfig};

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_elad=info".into()),
        )
        .init();

    let devices = sdroxide_elad::list();
    if devices.is_empty() {
        println!("No ELAD devices found on USB.");
        println!();
        println!("If one is plugged in, this is usually a permissions problem:");
        println!("  Linux   — install 60-sdroxide-elad.rules, then re-plug it");
        println!("  Windows — bind the device to WinUSB with Zadig (this stops");
        println!("            ELAD's FDM-SW2 seeing it until the driver is put back)");
        println!("  macOS   — nothing to install; quit any other software first");
        println!();
        println!("Note that only the *receive* USB port is this device. An FDM-DUO's");
        println!("CAT port is an FTDI serial bridge and its audio port a sound card;");
        println!("neither appears here.");
        return;
    }

    println!("{} ELAD device(s):", devices.len());
    for (i, d) in devices.iter().enumerate() {
        println!("  {i}: {}  [usb 1721:{:04x}]", d.label(), d.pid);
    }

    let center =
        std::env::var("PROBE_FREQ").ok().and_then(|s| s.parse::<f64>().ok()).unwrap_or(9_750_000.0);
    let rate = std::env::var("PROBE_RATE")
        .ok()
        .and_then(|s| s.parse::<u32>().ok())
        .filter(|r| ELAD_SAMPLE_RATES.contains(r))
        .unwrap_or(sdroxide_types::ELAD_DEFAULT_RATE_HZ);

    println!();
    println!(
        "Opening at {:.3} kHz, reading the stream as {:.0} kHz…",
        center / 1e3,
        rate as f64 / 1e3
    );
    let cfg = EladConfig { sample_rate_hz: rate, ..EladConfig::default() };
    let mut handle = match sdroxide_elad::EladHandle::open(&cfg, center) {
        Ok(h) => h,
        Err(e) => {
            println!("FAILED: {e}");
            if let Some(d) = sdroxide_elad::diagnostics() {
                println!("\n{d}");
            }
            std::process::exit(1);
        }
    };

    println!("  device   : {}", handle.label);
    println!("  model    : {}", handle.model.name());
    println!("  serial   : {}", handle.serial.as_deref().unwrap_or("(unreadable)"));
    println!(
        "  hardware : {}",
        handle.hw_version.map(|(a, b)| format!("{a}.{b}")).unwrap_or_else(|| "unknown".into())
    );
    println!(
        "  firmware : {}",
        handle.firmware.map(|(a, b)| format!("{a}.{b}")).unwrap_or_else(|| "unknown".into())
    );
    println!("  clock    : {:.3} MHz nominal", handle.model.clock_hz() / 1e6);
    for w in &handle.warnings {
        println!("  warning  : {w}");
    }

    let secs = 8.0;
    println!();
    println!("Streaming for {secs:.0} s…");
    let mut buf = vec![0f32; 1 << 16];
    let mut total = 0usize;
    let mut peak = 0f32;
    let started = Instant::now();
    while started.elapsed().as_secs_f64() < secs {
        let n = handle.rx_read(&mut buf);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(2));
            continue;
        }
        total += n / 2;
        for v in &buf[..n] {
            peak = peak.max(v.abs());
        }
    }
    let dt = started.elapsed().as_secs_f64();
    let measured = total as f64 / dt;
    println!("  {total} samples in {dt:.2} s = {:.1} kHz", measured / 1e3);
    println!("  read as {:.1} kHz", rate as f64 / 1e3);
    // The whole point of this tool. Named against the list rather than left as
    // a raw figure, because the list is what the setting offers.
    let nearest = ELAD_SAMPLE_RATES
        .iter()
        .copied()
        .min_by(|a, b| {
            let d = |r: u32| (r as f64 - measured).abs();
            d(*a).total_cmp(&d(*b))
        })
        .unwrap_or(rate);
    if total > 0 {
        if nearest == rate {
            println!("  → the device really is in its {:.0} kHz mode", rate as f64 / 1e3);
        } else {
            println!(
                "  → THE RATE IS WRONG: the device looks to be in its {:.0} kHz mode. \
                 Re-run with PROBE_RATE={nearest}.",
                nearest as f64 / 1e3
            );
        }
    }
    println!("  peak |sample| {peak:.4} (1.0 is full scale)");
    if total == 0 {
        println!();
        println!("NO SAMPLES ARRIVED.");
        println!("  {}", sdroxide_elad::fpga::silence_hint(handle.model));
    }

    handle.release();

    println!();
    println!("--- paste everything below into the bug report ---");
    println!();
    match sdroxide_elad::diagnostics() {
        Some(d) => println!("{d}"),
        None => println!("(no trace was recorded, which should not happen)"),
    }
}
