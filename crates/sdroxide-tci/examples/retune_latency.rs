//! How long after a `dds:` command does the IQ stream actually arrive on the new
//! centre? That delay times the speed of a panadapter drag is exactly how far
//! the picture slides while the drag is in progress — which is what
//! `TciConfig::stream_delay_ms` is set from, so this is how to calibrate it for
//! a rig or a link that is not the one the default was measured on.
//!
//! Tune to a steady carrier, step the centre by a known amount, and watch where
//! the carrier sits in the arriving samples. Point ExpertSDR3 at a strong
//! station first, and give a centre a few tens of kHz off it — the carrier must
//! not sit on DC, and it must stay inside the span after the step.
//!
//! Measured this way on a SunSDR2DX over the loopback interface: 109–131 ms at
//! 192 kHz, 129 ms at 96 kHz, 169 ms at 48 kHz, of which only about 21 ms was
//! buffered on this side.
//!
//! `cargo run --release -p sdroxide-tci --example retune_latency -- [addr] [center_hz] [step_hz] [rate_hz]`

use std::time::{Duration, Instant};

static RATE_CELL: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
fn rate() -> f64 {
    *RATE_CELL.get().expect("rate set in main")
}
/// Samples per measurement window: fine enough in frequency, short in time.
const WIN: usize = 4096;

/// Offset of the strongest bin from DC, in Hz, by a plain DFT search.
fn peak_offset_hz(iq: &[f32]) -> (f64, f32) {
    let n = iq.len() / 2;
    let mut best = (0.0f64, 0.0f32);
    // Coarse over the whole span, then refine.
    let scan = |lo: f64, hi: f64, steps: usize, best: &mut (f64, f32)| {
        for s in 0..=steps {
            let f = lo + (hi - lo) * s as f64 / steps as f64;
            let (mut re, mut im) = (0.0f32, 0.0f32);
            for k in 0..n {
                let ph = -std::f64::consts::TAU * f * k as f64 / rate();
                let (c, s) = (ph.cos() as f32, ph.sin() as f32);
                let (xr, xi) = (iq[2 * k], iq[2 * k + 1]);
                re += xr * c - xi * s;
                im += xr * s + xi * c;
            }
            let mag = (re * re + im * im).sqrt() / n as f32;
            if mag > best.1 {
                *best = (f, mag);
            }
        }
    };
    scan(-rate() / 2.0, rate() / 2.0, 256, &mut best);
    let step = rate() / 256.0;
    scan(best.0 - step, best.0 + step, 64, &mut best);
    best
}

fn main() {
    let addr = std::env::args().nth(1).unwrap_or_else(|| "127.0.0.1:50001".into());
    let center: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(14_100_000.0);
    let step: f64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(40_000.0);
    let r: f64 = std::env::args().nth(4).and_then(|s| s.parse().ok()).unwrap_or(192_000.0);
    RATE_CELL.set(r).ok();
    println!("iq rate: {r} Hz");

    let mut h = match sdroxide_tci::TciHandle::connect(&addr, rate()) {
        Ok(h) => h,
        Err(e) => {
            println!("connect failed: {e}");
            return;
        }
    };
    println!("device: {}", h.device);
    h.set_center(center);

    let mut buf = vec![0f32; WIN * 2];
    let read_win = |h: &mut sdroxide_tci::TciHandle, buf: &mut Vec<f32>| -> bool {
        let mut got = 0usize;
        let deadline = Instant::now() + Duration::from_secs(2);
        while got < WIN * 2 {
            if Instant::now() > deadline {
                return false;
            }
            let n = h.rx_read(&mut buf[got..]);
            if n == 0 {
                std::thread::sleep(Duration::from_millis(1));
                continue;
            }
            got += n;
        }
        true
    };

    // Settle, then learn where the carrier is before the step.
    std::thread::sleep(Duration::from_millis(1500));
    while h.rx_read(&mut buf) > 0 {} // drain
    if !read_win(&mut h, &mut buf) {
        println!("no IQ arriving — is the receiver running in ExpertSDR3?");
        return;
    }
    let (before, mag) = peak_offset_hz(&buf);
    println!("before: strongest tone at {before:+.0} Hz from centre (mag {mag:.5})");
    if mag < 1e-4 {
        println!(
            "  NOTE: nothing much there. Point the rig at a strong carrier for a clean answer."
        );
    }
    let want = before - step; // the centre moves up by `step`, so the tone moves down

    // How much of the delay is ours? Drain what is sitting in the ring right
    // now: anything already buffered here is latency we could account for
    // locally, and the rest belongs to the rig and the socket.
    {
        let mut held = 0usize;
        let mut scratch = vec![0f32; 1 << 16];
        loop {
            let n = h.rx_read(&mut scratch);
            if n == 0 {
                break;
            }
            held += n;
        }
        println!(
            "ring held {} samples at the moment of the step = {:.1} ms of local buffering",
            held / 2,
            held as f64 / 2.0 / rate() * 1e3
        );
    }

    // Step the centre, then time how long the arriving samples take to follow.
    let t0 = Instant::now();
    h.set_center(center + step);
    let mut elapsed_first = None;
    let mut settled = None;
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        if !read_win(&mut h, &mut buf) {
            break;
        }
        let (now, _) = peak_offset_hz(&buf);
        let t = t0.elapsed();
        // "Moved at all" and "arrived", as fractions of the commanded step.
        if elapsed_first.is_none() && (now - before).abs() > step * 0.1 {
            elapsed_first = Some(t);
        }
        if (now - want).abs() < step * 0.1 {
            settled = Some(t);
            println!("after:  tone at {now:+.0} Hz (expected {want:+.0}) at t = {:?}", t);
            break;
        }
    }
    println!();
    match (elapsed_first, settled) {
        (Some(a), Some(b)) => println!(
            "retune latency: first movement {:.0} ms, on frequency {:.0} ms",
            a.as_secs_f64() * 1e3,
            b.as_secs_f64() * 1e3
        ),
        _ => println!("the tone never reached the new offset — check the signal and the step size"),
    }
    println!(
        "\nA drag moves the centre about once per displayed frame. At 60 fps and\n\
         {:.0} ms of latency the picture runs {:.0} frames behind the label the\n\
         engine puts on it.",
        settled.unwrap_or_default().as_secs_f64() * 1e3,
        settled.unwrap_or_default().as_secs_f64() * 60.0
    );
    h.set_center(center);
}
