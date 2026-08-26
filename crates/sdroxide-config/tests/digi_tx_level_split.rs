//! What happens to a `digi.json` written before the transmit-audio level split.
//!
//! One test per file on purpose: `SDROXIDE_CONFIG_DIR` is process-global, and
//! setting it from a `#[test]` that shares a binary with others would race them.

/// The one level that became two carries into **both**, so no station's signal
/// changes level on an update.
///
/// It matters which way round that goes. The single `tx_audio_level` was doing
/// two unrelated jobs — deviation on FM, drive into the modulator on sideband —
/// and a station that had turned it down to 40 % for 1200 baud packet was
/// transmitting its FT8 8 dB down as well without being told. Splitting it is
/// the fix, but *defaulting the new sideband level to full scale* would put
/// that station on the air 8 dB hotter the first time they updated, which is
/// not a change an operator should discover by being reported for it. So the
/// old value stays in force on both sides until the operator raises the one
/// they can now see.
#[test]
fn an_old_single_level_carries_into_both_of_the_new_ones() {
    let root = std::env::temp_dir().join(format!("sdroxide-digi-split-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).expect("scratch dir");
    // SAFETY: this is the only test in this binary; nothing races the setter.
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };

    // A first run has no file at all: both levels wide open, as every digital
    // mode was before either of them existed.
    let fresh = sdroxide_config::load_digi_config();
    assert_eq!(fresh.tx_audio_level_fm, 1.0);
    assert_eq!(fresh.tx_audio_level_ssb, 1.0);

    // A file from before the split, with the level a packet station would set.
    std::fs::write(
        root.join("digi.json"),
        r#"{"my_call":"OE1XYZ","my_grid":"JN88","tx_audio_level":0.4}"#,
    )
    .expect("write the old config");
    let migrated = sdroxide_config::load_digi_config();
    assert_eq!(migrated.tx_audio_level_fm, 0.4, "the FM level lost the operator's setting");
    assert_eq!(migrated.tx_audio_level_ssb, 0.4, "the sideband level came up hotter than it was");
    assert_eq!(migrated.my_call, "OE1XYZ", "the rest of the config did not survive the migration");

    // A file written since the split says what it means, and the dead key left
    // beside it — an older version is still writing one — does not overrule it.
    std::fs::write(
        root.join("digi.json"),
        r#"{"tx_audio_level":0.4,"tx_audio_level_fm":0.4,"tx_audio_level_ssb":0.9}"#,
    )
    .expect("write the new config");
    let current = sdroxide_config::load_digi_config();
    assert_eq!(current.tx_audio_level_fm, 0.4);
    assert_eq!(current.tx_audio_level_ssb, 0.9, "the migration overwrote a level that was set");

    // And a file that never mentioned any of them is still the default.
    std::fs::write(root.join("digi.json"), r#"{"my_call":"OE1XYZ"}"#).expect("write");
    let plain = sdroxide_config::load_digi_config();
    assert_eq!(plain.tx_audio_level_fm, 1.0);
    assert_eq!(plain.tx_audio_level_ssb, 1.0);

    // SAFETY: as above.
    unsafe { std::env::remove_var("SDROXIDE_CONFIG_DIR") };
}
