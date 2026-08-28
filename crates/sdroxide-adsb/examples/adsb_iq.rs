//! Synthesise a sky to a `.iq` file, so the decoder and the panel can be
//! exercised without an antenna.
//!
//! ```text
//! cargo run --release -p sdroxide-adsb --example adsb_iq -- /tmp/sky.iq
//! sdroxide --file /tmp/sky.iq --rate 2400000 --freq 1090000000 --mode ADS-B
//! ```
//!
//! Interleaved little-endian `f32` pairs (CF32) at 2.4 Msps, which is what
//! `FileSource` reads and what an RTL-SDR delivers. The file loops, so a few
//! seconds of it is a sky that keeps flying.
//!
//! # What it is for, and what it is not
//!
//! It proves the plumbing: that a burst modulated the way a transponder
//! modulates one comes back out of the whole chain as a row on the panel, with
//! the identity and the position it was built from. It does **not** prove the
//! decoder works on air — the transmitter here and the receiver there were
//! written by the same hand and agree with each other by construction. For that
//! there is no substitute for an aerial and a comparison against another
//! decoder.
//!
//! The aircraft fly: each one is given a position, a track and a speed, and its
//! squitters are re-encoded every second from where it has got to. Six of them
//! at descending amplitudes, so the weakest is near where the correlator gives
//! up — which is the part of the picture worth watching.

use std::io::Write;

use sdroxide_adsb::{crc, modulate};

const RATE: f64 = 2_400_000.0;
/// Seconds of traffic before the file loops.
const SECONDS: usize = 20;

/// One aeroplane, and where it is going.
struct Plane {
    icao: u32,
    call: &'static str,
    lat: f64,
    lon: f64,
    track_deg: f64,
    speed_kt: f64,
    altitude_ft: i32,
    /// Amplitude of its bursts, full scale being 1.0.
    amp: f32,
}

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "sky.iq".to_string());

    let mut planes = vec![
        Plane {
            icao: 0x40_621D,
            call: "SDX0001",
            lat: 48.20,
            lon: 16.37,
            track_deg: 75.0,
            speed_kt: 460.0,
            altitude_ft: 37_000,
            amp: 0.70,
        },
        Plane {
            icao: 0x3C_6444,
            call: "SDX0002",
            lat: 48.55,
            lon: 15.90,
            track_deg: 190.0,
            speed_kt: 410.0,
            altitude_ft: 24_000,
            amp: 0.45,
        },
        Plane {
            icao: 0x48_4148,
            call: "SDX0003",
            lat: 47.90,
            lon: 16.80,
            track_deg: 300.0,
            speed_kt: 300.0,
            altitude_ft: 11_000,
            amp: 0.28,
        },
        Plane {
            icao: 0x4C_A1FA,
            call: "SDX0004",
            lat: 48.35,
            lon: 17.30,
            track_deg: 250.0,
            speed_kt: 480.0,
            altitude_ft: 41_000,
            amp: 0.18,
        },
        Plane {
            icao: 0x39_8568,
            call: "SDX0005",
            lat: 49.10,
            lon: 16.10,
            track_deg: 155.0,
            speed_kt: 350.0,
            altitude_ft: 8_000,
            amp: 0.12,
        },
        Plane {
            icao: 0x50_0123,
            call: "SDX0006",
            lat: 47.55,
            lon: 15.40,
            track_deg: 40.0,
            speed_kt: 520.0,
            altitude_ft: 33_000,
            amp: 0.08,
        },
    ];

    let mut out: Vec<u8> = Vec::new();
    let mut seed = 0x2B_7E15_1628u64;
    let block = (RATE / 2.0) as usize; // half a second

    for tick in 0..SECONDS * 2 {
        // Half a second of noise, with every aircraft's squitters written into
        // it. Each transmits a position twice a second (alternating even and
        // odd, as a real transponder does) and its identification every fifth.
        let mut samples = vec![num_complex::Complex::<f32>::new(0.0, 0.0); block];
        let mut at = 0usize;
        for p in planes.iter() {
            let odd = tick % 2 == 1;
            let mut frames: Vec<Vec<u8>> = vec![position(p, odd)];
            if tick % 10 == p.icao as usize % 10 {
                frames.push(identification(p));
            }
            if tick % 4 == 1 {
                frames.push(velocity(p));
            }
            for f in frames {
                seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                let burst = modulate(&f, RATE, p.amp, 0.004, seed);
                // Spread the traffic across the block rather than stacking it
                // at the start: on air two aircraft overlapping is a thing that
                // happens, and a generator that never does it would hide how
                // the correlator behaves when they do.
                at = (at + block / 17) % (block - burst.len());
                for (i, z) in burst.iter().enumerate() {
                    let s = &mut samples[at + i];
                    s.re += z.re;
                    s.im += z.im;
                }
            }
        }
        for z in &samples {
            out.extend_from_slice(&z.re.to_le_bytes());
            out.extend_from_slice(&z.im.to_le_bytes());
        }
        // Fly them on half a second. A knot is a nautical mile an hour and a
        // nautical mile is a minute of latitude, so this needs no earth radius.
        for p in planes.iter_mut() {
            let d = p.speed_kt * (0.5 / 3600.0) / 60.0;
            let r = p.track_deg.to_radians();
            p.lat += d * r.cos();
            p.lon += d * r.sin() / p.lat.to_radians().cos();
        }
    }

    std::fs::File::create(&path)
        .and_then(|mut f| f.write_all(&out))
        .unwrap_or_else(|e| panic!("cannot write {path}: {e}"));
    println!(
        "wrote {path}: {:.1} MB, {SECONDS} s at {:.1} Msps, {} aircraft",
        out.len() as f64 / 1e6,
        RATE / 1e6,
        planes.len()
    );
    println!("  sdroxide --file {path} --rate {} --freq 1090000000", RATE as u64);
    for p in &planes {
        println!(
            "  {:06X} {}  {:>6} ft  {:>3} kt  track {:>3}°  {:.0} dBFS",
            p.icao,
            p.call,
            p.altitude_ft,
            p.speed_kt as i32,
            p.track_deg as i32,
            20.0 * f64::from(p.amp).log10()
        );
    }
}

/// Write `len` bits of `v` into `msg` starting at bit `at`, most significant
/// first — the order every field in this format is written in.
fn put(msg: &mut [u8], at: usize, len: usize, v: u32) {
    for i in 0..len {
        if v >> (len - 1 - i) & 1 == 1 {
            msg[(at + i) / 8] |= 0x80 >> ((at + i) % 8);
        }
    }
}

/// The header every extended squitter shares: DF17, capability 5, the address.
fn squitter(icao: u32) -> Vec<u8> {
    let mut m = vec![0u8; 14];
    put(&mut m, 0, 5, 17);
    put(&mut m, 5, 3, 5);
    put(&mut m, 8, 24, icao);
    m
}

/// A barometric position squitter, in the requested parity's grid.
fn position(p: &Plane, odd: bool) -> Vec<u8> {
    let mut m = squitter(p.icao);
    put(&mut m, 32, 5, 11); // type code 11: airborne position, barometric
    // The 25-foot altitude encoding, which is what everything civil uses.
    let n = ((p.altitude_ft + 1000) / 25) as u32;
    let ac = ((n & 0x7F0) << 1) | 0x10 | (n & 0x00F);
    put(&mut m, 40, 12, ac);
    put(&mut m, 53, 1, u32::from(odd));
    let (lat, lon) = cpr_encode(p.lat, p.lon, odd);
    put(&mut m, 54, 17, lat);
    put(&mut m, 71, 17, lon);
    crc::seal(&mut m, 0);
    m
}

/// An identification squitter carrying the callsign.
fn identification(p: &Plane) -> Vec<u8> {
    let mut m = squitter(p.icao);
    put(&mut m, 32, 5, 4); // type code 4: category set A
    put(&mut m, 37, 3, 3); // A3, a large aeroplane
    let call = p.call.as_bytes();
    for i in 0..8 {
        let c = call.get(i).copied().unwrap_or(b' ');
        let v = match c {
            b'A'..=b'Z' => c - b'A' + 1,
            b'0'..=b'9' => c - b'0' + 48,
            _ => 32,
        };
        put(&mut m, 40 + i * 6, 6, u32::from(v));
    }
    crc::seal(&mut m, 0);
    m
}

/// A ground-velocity squitter.
fn velocity(p: &Plane) -> Vec<u8> {
    let mut m = squitter(p.icao);
    put(&mut m, 32, 5, 19); // type code 19: airborne velocity
    put(&mut m, 37, 3, 1); // subtype 1: subsonic ground speed
    let r = p.track_deg.to_radians();
    let ew = p.speed_kt * r.sin();
    let ns = p.speed_kt * r.cos();
    // Zero means "no information" and one means zero, so every magnitude is
    // offset by one.
    put(&mut m, 45, 1, u32::from(ew < 0.0));
    put(&mut m, 46, 10, (ew.abs().round() as u32 + 1).min(1023));
    put(&mut m, 56, 1, u32::from(ns < 0.0));
    put(&mut m, 57, 10, (ns.abs().round() as u32 + 1).min(1023));
    // Level: vertical rate zero, which is the field's value 1.
    put(&mut m, 69, 9, 1);
    crc::seal(&mut m, 0);
    m
}

/// Encode a position into the 17-bit CPR fields of the requested parity.
///
/// The forward direction of the algorithm in `sdroxide_adsb::cpr` — a straight
/// reading of ICAO Doc 9871 D.2.4.7, which states the encoder and leaves the
/// decoder to be derived from it.
fn cpr_encode(lat: f64, lon: f64, odd: bool) -> (u32, u32) {
    const NZ: f64 = 15.0;
    let i = if odd { 1.0 } else { 0.0 };
    let d_lat = 360.0 / (4.0 * NZ - i);
    let yz = (131_072.0 * modpos(lat, d_lat) / d_lat + 0.5).floor();
    let rlat = d_lat * (yz / 131_072.0 + (lat / d_lat).floor());

    let nlv = sdroxide_adsb::cpr::nl(rlat) - i;
    let d_lon = if nlv > 0.0 { 360.0 / nlv } else { 360.0 };
    let xz = (131_072.0 * modpos(lon, d_lon) / d_lon + 0.5).floor();
    ((yz as u32) & 0x1_FFFF, (xz as u32) & 0x1_FFFF)
}

fn modpos(x: f64, y: f64) -> f64 {
    let r = x % y;
    if r < 0.0 { r + y } else { r }
}
