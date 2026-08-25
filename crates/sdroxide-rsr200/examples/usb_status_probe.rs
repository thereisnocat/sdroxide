//! Standalone smoke test for step 5 (24-bit + status readout) against a
//! real, physically-attached RSR200 over USB. Not a unit test — it needs
//! actual hardware, and is not run by `cargo test`. Goes through the real
//! path the app uses (`Rsr200Handle::open`, `stream.rs`), unlike
//! `usb_live_probe.rs`/`usb_dual_probe.rs`, which drive `Device` directly —
//! this is what actually proves `Rsr200Config::bits24` and
//! `Rsr200Handle::status()` work end to end, not just that the lower layer
//! can produce 24-bit blocks in isolation.
//!
//! ```text
//! cargo run --example usb_status_probe -p sdroxide-rsr200
//! ```

use std::time::{Duration, Instant};

use sdroxide_rsr200::Rsr200Handle;
use sdroxide_types::{Rsr200ChannelMode, Rsr200Config, Rsr200Transport};

fn main() {
    let cfg = Rsr200Config {
        adc_clock_hz: 100e6,
        gps_discipline: true,
        decimation_exp: 3,
        transport: Rsr200Transport::Usb,
        channel_mode: Rsr200ChannelMode::Single,
        bits24: true,
        ..Rsr200Config::default()
    };

    let mut handle = match Rsr200Handle::open(&cfg, 14.2e6) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("Rsr200Handle::open failed: {e}");
            std::process::exit(1);
        }
    };
    println!("Opened: {}", handle.label);
    println!("Sample rate: {:.3} Msps", handle.sample_rate_hz / 1e6);

    let mut buf = vec![0f32; 1 << 16];
    let mut total_pairs: u64 = 0;
    let mut last_status = Instant::now();
    let deadline = Instant::now() + Duration::from_secs(5);

    while Instant::now() < deadline {
        let n = handle.rx_read(&mut buf);
        total_pairs += (n / 2) as u64;
        if n == 0 {
            std::thread::sleep(Duration::from_millis(5));
        }
        if last_status.elapsed() >= Duration::from_secs(1) {
            let s = handle.status();
            println!(
                "status: temp={}C auto_att={} overload1={} overload2={} freq_corr_valid={} freq_corr_raw={} \
                 ({:+.1} Hz)",
                s.temperature_c,
                s.auto_att_active,
                s.overload_ch1,
                s.overload_ch2,
                s.freq_correction_valid,
                s.freq_correction_raw,
                sdroxide_rsr200::protocol::freq_correction_hz(&s, cfg.gps_discipline),
            );
            last_status = Instant::now();
        }
    }

    handle.release();
    println!("\n{total_pairs} complex pairs read over the run. Exiting.");
    std::process::exit(if total_pairs > 0 { 0 } else { 2 });
}
