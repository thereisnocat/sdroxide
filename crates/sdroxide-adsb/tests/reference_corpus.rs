//! Decode somebody else's off-air recordings and check against their answers.
//!
//! Every other test in this crate synthesises its own signal, which proves the
//! arithmetic and the plumbing but cannot prove recall: a transmitter and a
//! receiver written by the same hand agree with each other by construction —
//! and in this crate they once did, at length, while the decoder recovered a
//! twentieth of real bursts. See `demod::modulate_at`.
//!
//! These are real 1090 MHz captures from the **rsadsb/dump1090_rs** project,
//! with the messages *its* demodulator finds in them. Both halves are somebody
//! else's: the samples were recorded on hardware this project has never seen,
//! and the expected output is another implementation's, written from the same
//! standard but sharing no code. Agreeing with it is the only evidence here
//! that is not self-referential.
//!
//! The recordings are not redistributed, so this test **skips** when they are
//! absent rather than failing. Point it at a checkout of
//! <https://github.com/rsadsb/dump1090_rs>:
//!
//! ```text
//! SDROXIDE_ADSB_IQ=/path/to/dump1090_rs/test_iq \
//!   cargo test -p sdroxide-adsb --test reference_corpus -- --nocapture
//! ```
//!
//! Format: 2.4 Msps, interleaved little-endian `i16`, **imaginary part first**.
//!
//! When this was written the decoder found all fourteen, and nothing the
//! reference did not also find. Any message here going missing is a real loss
//! of sensitivity; an extra one is a false decode, which matters more.

use std::collections::BTreeSet;
use std::path::PathBuf;

use sdroxide_adsb::{Demod, accept};
use sdroxide_dsp::Complex32;

/// (file, the messages `dump1090_rs` reports in it).
const CORPUS: [(&str, &[&str]); 3] = [
    (
        "test_1641427457780.iq",
        &[
            "8DAD929358B9C6273F002169C02E",
            "8DAA2BC4F82100020049B8DB9449",
            "02E1971CE17C84",
            "8DA0AAA058BF163FCF860013E840",
        ],
    ),
    (
        "test_1641428165033.iq",
        &[
            "8DA79DE99909932F780C9E2F2F8F",
            "8DAC04D358A7820A86AC3709E689",
            "8DAC04D3EA4288669B5C082751D4",
            "8DA79DE958BDF59C85104874ADAD",
            "5DAD92936265F5",
        ],
    ),
    (
        "test_1641428106243.iq",
        &[
            "8DA8AAC8990C30B51808AA24E573",
            "02E19838BFF1D9",
            "8DADA6B9990CF61E4848AF2A8656",
            "8DA4BA025885462008FA0A4A6EB2",
            "8DA4BA0299115F301074A72DB6FF",
        ],
    ),
];

fn corpus_dir() -> Option<PathBuf> {
    let d = PathBuf::from(std::env::var("SDROXIDE_ADSB_IQ").ok()?);
    d.is_dir().then_some(d)
}

/// Read one capture as complex baseband.
fn read(path: &PathBuf) -> Vec<Complex32> {
    let raw = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    raw.chunks_exact(4)
        .map(|c| {
            // Imaginary first — the order the reference writes them in.
            let im = i16::from_le_bytes([c[0], c[1]]);
            let re = i16::from_le_bytes([c[2], c[3]]);
            Complex32::new(f32::from(re) / 32768.0, f32::from(im) / 32768.0)
        })
        .collect()
}

/// Everything this decoder accepts from one capture.
fn decode(iq: &[Complex32]) -> BTreeSet<String> {
    let mut d = Demod::new(2_400_000.0);
    let mut cands = Vec::new();
    // Engine-sized blocks, so the block-boundary handling is exercised too.
    for chunk in iq.chunks(32_768) {
        d.push(chunk, &mut cands);
    }
    let mut seen: BTreeSet<u32> = BTreeSet::new();
    let mut out = BTreeSet::new();
    for c in &cands {
        let Ok(a) = accept(&c.bytes, |i| seen.contains(&i)) else { continue };
        if a.is_verified() {
            seen.insert(a.icao());
        }
        out.insert(c.bytes.iter().map(|b| format!("{b:02X}")).collect::<String>());
    }
    out
}

#[test]
fn the_reference_recordings_decode_to_what_the_reference_says() {
    let Some(dir) = corpus_dir() else {
        eprintln!("SDROXIDE_ADSB_IQ is not set to a directory of captures; skipping");
        return;
    };
    let (mut want_total, mut got_total, mut common_total) = (0usize, 0usize, 0usize);
    let mut missing: Vec<String> = Vec::new();
    let mut extra: Vec<String> = Vec::new();

    for (name, expected) in CORPUS {
        let path = dir.join(name);
        if !path.is_file() {
            eprintln!("{name} is missing from the corpus; skipping it");
            continue;
        }
        let want: BTreeSet<String> = expected.iter().map(|s| (*s).to_string()).collect();
        let got = decode(&read(&path));
        want_total += want.len();
        got_total += got.len();
        common_total += want.intersection(&got).count();
        missing.extend(want.difference(&got).map(|m| format!("{name}: {m}")));
        extra.extend(got.difference(&want).map(|m| format!("{name}: {m}")));
        eprintln!("{name}: reference {}, here {}", want.len(), got.len());
    }
    if want_total == 0 {
        eprintln!("no captures found; skipping");
        return;
    }

    // An extra message is worse than a missing one: a decode nothing else sees
    // is either a false positive on the map or evidence the reference is being
    // out-decoded, and only one of those is good news.
    assert!(extra.is_empty(), "decoded messages the reference did not: {extra:#?}");
    assert_eq!(
        common_total,
        want_total,
        "missed {} of {want_total} reference messages: {missing:#?}",
        want_total - common_total
    );
}
