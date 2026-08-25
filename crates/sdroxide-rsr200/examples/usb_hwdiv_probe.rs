//! Standalone smoke test for step 6 (hardware diversity) against a real,
//! physically-attached RSR200 over USB. Not a unit test — it needs actual
//! hardware, and is not run by `cargo test`. Goes through the real
//! `Rsr200Handle`/`stream.rs` path, the same as `usb_status_probe.rs`.
//!
//! Proves, in order: `OpMode::Diversity` is accepted, the channel-2 weight
//! command (`SET_GENERATORS` selector 9) is accepted, and real samples
//! stream afterward — i.e. that switching the radio into its own hardware
//! combiner does not itself break the connection. Does **not** prove the
//! *combining* is correct (which channel carries the result, whether a
//! non-unity weight actually nulls or combines anything real) — that needs
//! two real aerials and a human listening, the same as Separate mode's own
//! "confirmed on air" milestone.
//!
//! ```text
//! cargo run --example usb_hwdiv_probe -p sdroxide-rsr200
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
        channel_mode: Rsr200ChannelMode::HardwareDiversity,
        // Unity first -- the safest possible weight to confirm the mode
        // switch itself works before trying anything that actually
        // combines the channels differently.
        hw_div_magnitude: 1.0,
        hw_div_phase_deg: 0.0,
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
    println!("Sample rate: {:.3} Msps, dual (wire is 2-channel): {}", handle.sample_rate_hz / 1e6, handle.dual());

    // Hardware diversity still wants the 2-channel ring -- channel A
    // carries the combined result, channel B is read but unused, exactly
    // the way Rsr200Source::read() handles it.
    let mut main = vec![0f32; 1 << 15];
    let mut aux = vec![0f32; 1 << 15];
    let mut total_pairs: u64 = 0;
    let mut last_status = Instant::now();
    let deadline = Instant::now() + Duration::from_secs(5);

    while Instant::now() < deadline {
        let pairs = handle.read_pair(&mut main, &mut aux);
        total_pairs += pairs as u64;
        if pairs == 0 {
            std::thread::sleep(Duration::from_millis(5));
        }
        if last_status.elapsed() >= Duration::from_secs(1) {
            let s = handle.status();
            println!(
                "status: temp={}C auto_att={} overload1={} overload2={}",
                s.temperature_c, s.auto_att_active, s.overload_ch1, s.overload_ch2
            );
            last_status = Instant::now();
        }
    }

    handle.release();
    println!("\n{total_pairs} complex pairs read over the run. Exiting.");
    std::process::exit(if total_pairs > 0 { 0 } else { 2 });
}
