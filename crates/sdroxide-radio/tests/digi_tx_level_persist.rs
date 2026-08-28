//! The transmit-audio level an operator sets survives the session (issue #186).
//!
//! Its own test binary, and deliberately so. `SDROXIDE_CONFIG_DIR` is
//! process-global, this is the one test here that reads `digi.json` back rather
//! than measuring what reached the radio, and every engine in a process shares
//! that file — so run beside `digi_tx_level.rs` it would be asserting against a
//! file three other engines are writing. Cargo gives each integration test file
//! a process of its own, which is the isolation this needs.
//!
//! What is being pinned is the debounce. `SetDigiTxLevel` deliberately does not
//! save: the control is a rail, one command per frame while it is dragged, and
//! an atomic write per frame on the thread pacing transmit blocks is a cost
//! that lands exactly while the operator is transmitting. So the write is
//! deferred to the periodic tick and to shutdown — and a setting that is only
//! ever in memory is not a setting.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RxId};

const DIAL_HZ: f64 = 14_090_000.0;

struct SoundCardRig {
    tx: Arc<Mutex<usize>>,
}

impl IqSource for SoundCardRig {
    fn sample_rate(&self) -> f64 {
        48_000.0
    }
    fn center_hz(&self) -> f64 {
        DIAL_HZ
    }
    fn set_center_hz(&mut self, _hz: f64) -> Result<()> {
        Ok(())
    }
    fn center_is_dial(&self) -> bool {
        true
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(1024);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn tx_write_audio(&mut self, audio: &[f32]) -> Result<()> {
        *self.tx.lock().unwrap() += audio.len();
        Ok(())
    }
    fn describe(&self) -> String {
        "mock CAT rig on a sound card".into()
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "mock".into(),
        label: "mock CAT rig".into(),
        rx_channels: 1,
        tx_channels: 1,
        tx_audio: true,
        ..DeviceCaps::default()
    }
}

#[test]
fn a_level_the_operator_set_is_on_disk_after_the_session() {
    let root =
        std::env::temp_dir().join(format!("sdroxide-tx-level-persist-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };

    // Nothing set yet: every mode is on its carrier default.
    let before = sdroxide_config::load_digi_config();
    assert!(before.tx_audio_levels.is_empty(), "the isolated config was not empty");

    let tx = Arc::new(Mutex::new(0usize));
    let src = SoundCardRig { tx: Arc::clone(&tx) };
    let cfg = EngineConfig { tx_ham_only: false, ..Default::default() };
    let mut h = start_engine(Box::new(src), caps(), cfg);
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(200));

    for c in [
        Command::SetMode { rx: RxId::Main, mode: Mode::Rtty },
        Command::SetDigiTxLevel { mode: Mode::Rtty, level: 0.25 },
        Command::SetDigiTxLevel { mode: Mode::Ft8, level: 0.5 },
    ] {
        h.cmd_tx.send(c).expect("engine gone");
        std::thread::sleep(Duration::from_millis(60));
    }

    // Still only in memory: the tick is ten seconds away and the rail does not
    // save as it goes. Read through a fresh load rather than the handle, which
    // is what a second engine in the station would see.
    let mid = sdroxide_config::load_digi_config();
    assert!(
        mid.tx_audio_levels.is_empty(),
        "the rail wrote the file per command after all: {:?}",
        mid.tx_audio_levels
    );

    // End the session. `Drop` is the other half of the debounce, and the common
    // case: an operator who trims their level and quits has set it.
    drop(h.cmd_tx);
    drop(h.event_rx);
    if let Some(t) = thread {
        let _ = t.join();
    }

    let after = sdroxide_config::load_digi_config();
    assert_eq!(after.tx_level_for(Mode::Rtty), 0.25, "RTTY's level did not survive the session");
    assert_eq!(after.tx_level_for(Mode::Ft8), 0.5, "FT8's level did not survive the session");
    // And a mode nobody set still inherits, from the file as from memory.
    assert_eq!(after.tx_level_for(Mode::Psk), 1.0, "an unset mode came back with an entry");

    let _ = std::fs::remove_dir_all(&root);
}
