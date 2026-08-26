//! What level a digital over reaches the radio at.
//!
//! Field report ([issue #131]): a Kenwood on its I/Q output made 100 % of its
//! power under TUNE and a quarter of it on FT8, with the Drive slider moving
//! nothing. The rig modulates the audio we send it, and the modems synthesise
//! their modulating signal at half scale — 6 dB of headroom that belongs to the
//! modem's own arithmetic, not to the transmitter. Nothing divided it out
//! again, so the radio was asked for a quarter of the power a TUNE at the same
//! slider setting asks for, and no power command could make up the difference:
//! the output was riding on the audio, not on the power register.
//!
//! So the two levels are compared against each other rather than against a
//! constant. Whatever a tune tone is worth to a rig, a digital over has to be
//! worth the same.
//!
//! [issue #131]: https://github.com/dividebysandwich/sdroxide/issues/131

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, DigiConfig, Mode, RxId};

const DIAL_HZ: f64 = 14_090_000.0;

/// Point the whole process at a config directory of its own, once.
///
/// `SDROXIDE_CONFIG_DIR` is process-global, and an engine that comes up without
/// it reads the *operator's* live configuration — including the transmit-audio
/// level, which is one of the two numbers this file multiplies together. A
/// station that had turned its data input down to 40 % for FM packet failed
/// this test at 0.4 against the tune tone's 1.0, and the failure looked exactly
/// like the regression it exists to catch. It was measuring the operator's
/// settings, not the code.
fn isolate_config() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let root =
            std::env::temp_dir().join(format!("sdroxide-digi-tx-level-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };
    });
}

/// The station this test transmits from: stock, and explicitly at a full-scale
/// data input.
///
/// Sent rather than left to the defaults because it is the premise of the
/// comparison and not an incidental setting. The transmit-audio levels are the
/// operator's own, into a radio that modulates what we send it — deviation on
/// FM, drive into the modulator on sideband — and a digital over only reaches
/// the rig at the level a tune does when the one in play is wide open. Turned
/// down, the over is *supposed* to arrive quieter, which
/// [`the_sideband_level_is_the_one_a_data_mode_uses`] and
/// `aprs_tx::the_transmit_audio_level_scales_what_the_radio_is_given` pin from
/// the other side.
fn full_scale_station() -> DigiConfig {
    DigiConfig { tx_audio_level_fm: 1.0, tx_audio_level_ssb: 1.0, ..DigiConfig::default() }
}

/// The loudest sample the rig's sound card was given, and how many blocks it
/// took.
#[derive(Default)]
struct Heard {
    peak: f32,
    blocks: usize,
}

/// A CAT rig on a sound card: it modulates the audio we hand it and has its own
/// power control, which is what nearly every rig this backend drives looks
/// like — a Kenwood among them.
struct SoundCardRig {
    heard: Arc<Mutex<Heard>>,
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
    fn describe(&self) -> String {
        "mock CAT rig on a sound card".into()
    }
    /// The rig's power register is the level control; the audio is only the
    /// modulating signal. Exactly the case the report came from.
    fn commands_tx_power(&self) -> bool {
        true
    }
    fn tx_begin(&mut self, _center_hz: f64, rate: f64) -> Result<f64> {
        Ok(rate)
    }
    fn tx_write_audio(&mut self, audio: &[f32]) -> Result<()> {
        let mut h = self.heard.lock().unwrap();
        h.blocks += 1;
        for &a in audio {
            h.peak = h.peak.max(a.abs());
        }
        Ok(())
    }
    fn tx_end(&mut self) -> Result<()> {
        Ok(())
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "mock-cat".into(),
        label: "mock CAT rig".into(),
        rx_channels: 1,
        tx_channels: 1,
        // Quadrature receive, audio transmit — the shape of the rig in the
        // report.
        audio_mode: false,
        tx_audio: true,
        freq_ranges_rx: vec![(10_000.0, 148_000_000.0)],
        freq_ranges_tx: vec![(1_800_000.0, 54_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// Run an over built by `cmds` and report the loudest audio the rig was given.
/// Gives up once the transmission is plainly under way, since the level is
/// settled by then and a keyboard mode would otherwise send until stopped.
fn peak_of(cmds: Vec<Command>) -> f32 {
    isolate_config();
    let heard = Arc::new(Mutex::new(Heard::default()));
    let src = SoundCardRig { heard: Arc::clone(&heard) };
    let cfg = EngineConfig { tx_ham_only: false, ..Default::default() };
    let mut h = start_engine(Box::new(src), caps(), cfg);
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(200));
    for c in std::iter::once(Command::SetDigiConfig(full_scale_station())).chain(cmds) {
        h.cmd_tx.send(c).expect("engine gone");
        std::thread::sleep(Duration::from_millis(60));
    }

    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        while h.event_rx.try_recv().is_ok() {}
        if heard.lock().unwrap().blocks > 20 {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let out = std::mem::take(&mut *heard.lock().unwrap());

    drop(h.cmd_tx);
    drop(h.event_rx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    assert!(out.blocks > 0, "nothing ever reached the rig's sound card");
    out.peak
}

/// The fix. A keyboard-mode over and a tune tone are both modulating signals
/// for the same transmitter, and they arrive at the same level; before this the
/// over arrived 6 dB down, which is the quarter of the power the report
/// measured.
#[test]
fn a_digital_over_reaches_the_rig_at_the_level_a_tune_does() {
    let tune = peak_of(vec![Command::SetTune(true)]);
    let over = peak_of(vec![
        Command::SetMode { rx: RxId::Main, mode: Mode::Rtty },
        Command::DigiTxActive(true),
        Command::DigiTxText("cq cq de w1aw w1aw k".into()),
    ]);
    assert!(tune > 0.9, "the tune tone is full scale: {tune}");
    assert!(
        (over - tune).abs() < 0.1,
        "a digital over is {over} against the tune tone's {tune} — the rig is being asked for \
         {:.0}% of the power",
        (over / tune).powi(2) * 100.0
    );
}

/// RTTY is audio on a sideband, so it takes the sideband level and never the FM
/// one.
///
/// The two were a single number until a station set 40 % for 1200 baud packet
/// deviation and found it had taken 8 dB off their FT8 as well — issue #131's
/// symptom by another road, and invisible, because the control was only drawn
/// in the APRS panel. So the level that applies follows the carrier the mode
/// goes out on: this is the sideband end of it, and
/// `aprs_tx::the_transmit_audio_level_scales_what_the_radio_is_given` is the FM
/// end.
#[test]
fn the_sideband_level_is_the_one_a_data_mode_uses() {
    let rtty_at = |fm: f32, ssb: f32| {
        peak_of(vec![
            Command::SetDigiConfig(DigiConfig {
                tx_audio_level_fm: fm,
                tx_audio_level_ssb: ssb,
                ..DigiConfig::default()
            }),
            Command::SetMode { rx: RxId::Main, mode: Mode::Rtty },
            Command::DigiTxActive(true),
            Command::DigiTxText("cq cq de w1aw w1aw k".into()),
        ])
    };
    // Turning the *FM* level down does nothing to a sideband over...
    let fm_down = rtty_at(0.4, 1.0);
    assert!(fm_down > 0.9, "an FM deviation setting reached an SSB over: {fm_down}");
    // ...and turning the sideband level down scales it, because on sideband
    // this is the only drive control there is.
    let ssb_down = rtty_at(1.0, 0.5);
    assert!(
        (ssb_down / fm_down - 0.5).abs() < 0.08,
        "half the sideband level should be half the amplitude: {ssb_down} against {fm_down}"
    );
}

// ── the same over on a radio we modulate ourselves ───────────────────────────

/// An I/Q transmitter: no sound card, no power register of its own, so Drive is
/// the level and the modulated baseband is what goes on the air.
struct IqRadio {
    heard: Arc<Mutex<Heard>>,
}

impl IqSource for IqRadio {
    fn sample_rate(&self) -> f64 {
        48_000.0
    }
    fn center_hz(&self) -> f64 {
        DIAL_HZ
    }
    fn set_center_hz(&mut self, _hz: f64) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(1024);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock I/Q transmitter".into()
    }
    fn tx_begin(&mut self, _center_hz: f64, rate: f64) -> Result<f64> {
        Ok(rate)
    }
    fn tx_write(&mut self, iq: &[Complex32]) -> Result<()> {
        let mut h = self.heard.lock().unwrap();
        h.blocks += 1;
        for z in iq {
            h.peak = h.peak.max(z.norm());
        }
        Ok(())
    }
    fn tx_end(&mut self) -> Result<()> {
        Ok(())
    }
}

/// As [`peak_of`], for the modulated-I/Q path: the strongest baseband magnitude
/// the converter was handed.
fn iq_peak_of(cmds: Vec<Command>) -> f32 {
    isolate_config();
    let heard = Arc::new(Mutex::new(Heard::default()));
    let src = IqRadio { heard: Arc::clone(&heard) };
    let cfg = EngineConfig { tx_ham_only: false, ..Default::default() };
    let mut h = start_engine(Box::new(src), DeviceCaps { tx_audio: false, ..caps() }, cfg);
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(200));
    for c in cmds {
        h.cmd_tx.send(c).expect("engine gone");
        std::thread::sleep(Duration::from_millis(60));
    }

    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        while h.event_rx.try_recv().is_ok() {}
        if heard.lock().unwrap().blocks > 20 {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let out = std::mem::take(&mut *heard.lock().unwrap());

    drop(h.cmd_tx);
    drop(h.event_rx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    assert!(out.blocks > 0, "nothing was ever transmitted");
    out.peak
}

/// On a radio sdroxide modulates itself, Drive is the level — so a hundred
/// percent of it has to be a hundred percent of the transmitter, the same as a
/// tune at a hundred percent. The modem's headroom used to eat two thirds of
/// the power the slider was promising, with nothing to show it.
#[test]
fn drive_spends_the_whole_transmitter_on_a_digital_over() {
    let tune = iq_peak_of(vec![Command::SetTuneDrive(1.0), Command::SetTune(true)]);
    let over = iq_peak_of(vec![
        Command::SetTxDrive(1.0),
        Command::SetMode { rx: RxId::Main, mode: Mode::Rtty },
        Command::DigiTxActive(true),
        Command::DigiTxText("cq cq de w1aw w1aw k".into()),
    ]);
    assert!(tune > 0.9, "a full tune is a full-scale carrier: {tune}");
    assert!(
        (over - tune).abs() < 0.1,
        "an over at full Drive is {over} against the full tune's {tune} — {:.0}% of the power",
        (over / tune).powi(2) * 100.0
    );
}
