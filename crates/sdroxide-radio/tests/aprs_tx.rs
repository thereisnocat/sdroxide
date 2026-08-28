//! What an APRS over actually sounds like by the time it leaves the engine.
//!
//! Field report (issue #150): a beacon and a message went out on an IC-705 —
//! the transmission was there, and briefer than it should have been — and no
//! receiver could decode it. The controller's own tests could not see it: they
//! stop at `DigiEngine::fill_tx_block`, and what reaches a radio is what the
//! engine does with those blocks afterwards.
//!
//! ⚠️ `SetDigiConfig` makes the engine save, and `SDROXIDE_CONFIG_DIR` is
//! process-global: see [`isolate_config`]. A test here that forgets it writes
//! its fixtures over the operator's real station settings.
//!
//! So this stands the whole engine up on an audio-modulated rig — the shape an
//! IC-705 on its LAN or USB connector presents — captures every sample handed
//! to `tx_write_audio`, and puts it back through a real Bell 202 modem and
//! deframer. Anything between the modem and the sound card that loses,
//! duplicates, reorders or truncates a sample shows up here as a frame that
//! will not decode, which is exactly what the operator on the far end sees.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_ax25::{Deframer, Packet, PacketType};
use sdroxide_dsp::{AfskProfile, AfskRx};
use sdroxide_radio::{AudioParams, Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, DigiConfig, Mode, RxId};

/// The APRS channel for Region 1, which is where the engine tunes itself.
const DIAL_HZ: f64 = 144_800_000.0;

/// Every sample the rig's sound card was given, in order.
#[derive(Default)]
struct Heard {
    audio: Vec<f32>,
    blocks: usize,
}

/// A transceiver that modulates the audio we send it: quadrature receive, audio
/// transmit, its own power control. An IC-705 over its LAN port is this shape,
/// and so is any CAT rig on a sound card.
struct SoundCardRig {
    heard: Arc<Mutex<Heard>>,
    /// What this radio receives at, and what it plays transmit audio at. They
    /// are not always the same number: an Icom on its 12 kHz IF output hands
    /// back a stream decimated to 24 kHz while taking transmit audio at the
    /// session's 48. See `rx_and_tx_rates_may_differ`.
    rx_rate: f64,
    tx_rate: f64,
    /// A little noise on the receive side. Not decoration: the packet channel
    /// counts its CSMA slots on the receive sample clock, so a source that
    /// hands back nothing at all is a station that can never decide to key.
    rng: u32,
}

impl IqSource for SoundCardRig {
    fn sample_rate(&self) -> f64 {
        self.rx_rate
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
        // Audio in the I component, which is what a demod-audio rig hands
        // back. Noise rather than silence: the packet channel counts its CSMA
        // slots on the receive sample clock, so a source that hands back
        // nothing is a station that can never decide to key.
        for z in &mut buf[..n] {
            self.rng = self.rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let a = (self.rng >> 16) as f32 / 32_768.0 - 1.0;
            *z = Complex32::new(a * 0.02, 0.0);
        }
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock CAT rig on a sound card".into()
    }
    fn display_bandwidth(&self) -> Option<f64> {
        Some(4000.0)
    }
    fn commands_tx_power(&self) -> bool {
        true
    }
    fn tx_begin(&mut self, _center_hz: f64, _rate: f64) -> Result<f64> {
        Ok(self.tx_rate)
    }
    fn tx_write_audio(&mut self, audio: &[f32]) -> Result<()> {
        let mut h = self.heard.lock().unwrap();
        h.blocks += 1;
        h.audio.extend_from_slice(audio);
        Ok(())
    }
    fn tx_end(&mut self) -> Result<()> {
        Ok(())
    }
}

/// The two shapes a transceiver presents, which are two different paths
/// through the engine's receive side and have to be tested apart.
///
/// `audio_mode` is a radio that demodulates for us and hands back audio — an
/// IC-705 on its LAN port, a CAT rig on a sound card. The other is a radio
/// that hands back I/Q and takes audio to modulate, which is every SDR with a
/// transmitter. The digital-mode tap is fed from a different place in each,
/// and a packet station counts its channel-access slots on that tap — so a
/// fault in either is a station that never keys at all.
fn caps(audio_mode: bool) -> DeviceCaps {
    DeviceCaps {
        driver: "mock-cat".into(),
        label: "mock CAT rig".into(),
        rx_channels: 1,
        tx_channels: 1,
        // The shape an IC-705 on its LAN port presents: the rig demodulates
        // and hands back audio, and modulates the audio we hand it.
        audio_mode,
        tx_audio: true,
        freq_ranges_rx: vec![(10_000.0, 500_000_000.0)],
        freq_ranges_tx: vec![(144_000_000.0, 148_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// A station that will actually transmit: a callsign and somewhere to say it is.
fn station() -> DigiConfig {
    DigiConfig {
        my_call: "OE3JJS".into(),
        my_grid: "JN88ec".into(),
        // Take the channel as soon as a slot has passed; the dice are not what
        // is under test.
        packet_persist: 255,
        packet_slottime_ms: 10,
        ..DigiConfig::default()
    }
}

/// Point the whole process at a config directory of its own, once.
///
/// `SetDigiConfig` makes the engine *save* — and `SDROXIDE_CONFIG_DIR` is
/// process-global, so a test that does not set it writes into the operator's
/// real configuration. This one sends a digi config on every over, so without
/// this it would overwrite a live station's callsign, position and channel
/// settings with its own fixtures. It has done exactly that once.
fn isolate_config() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        let root = std::env::temp_dir().join(format!("sdroxide-aprs-tx-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };
    });
}

/// Run `cmds` on an APRS station and return every sample the rig was given.
fn over_for(audio_mode: bool, cmds: Vec<Command>) -> Vec<f32> {
    over_at(audio_mode, 48_000.0, 48_000.0, cmds)
}

/// The same on a radio whose receive and transmit rates differ.
fn over_at(audio_mode: bool, rx_rate: f64, tx_rate: f64, cmds: Vec<Command>) -> Vec<f32> {
    isolate_config();
    let heard = Arc::new(Mutex::new(Heard::default()));
    let src = SoundCardRig { heard: Arc::clone(&heard), rng: 0x1234_5678, rx_rate, tx_rate };
    // A ring nothing reads. Without somewhere to play audio the engine never
    // builds the main receive chain, and the digital-mode tap it feeds is what
    // a packet station counts its channel-access slots on — so an engine with
    // no audio sink is a station that can never decide to key.
    let (producer, _consumer) = rtrb::RingBuffer::<f32>::new(48_000);
    let cfg = EngineConfig {
        tx_ham_only: false,
        remember_session: false,
        audio: Some(AudioParams { producer, out_rate: 48_000.0 }),
        ..Default::default()
    };
    let mut h = start_engine(Box::new(src), caps(audio_mode), cfg);
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(200));
    for c in cmds {
        h.cmd_tx.send(c).expect("engine gone");
        std::thread::sleep(Duration::from_millis(80));
    }

    // Wait for the over to finish rather than for a fixed time: a packet burst
    // is about a second, and cutting the capture short would fake the very
    // symptom under test.
    let deadline = Instant::now() + Duration::from_secs(8);
    let mut quiet_since: Option<Instant> = None;
    while Instant::now() < deadline {
        while h.event_rx.try_recv().is_ok() {}
        let blocks = heard.lock().unwrap().blocks;
        std::thread::sleep(Duration::from_millis(50));
        let after = heard.lock().unwrap().blocks;
        if after > 0 && after == blocks {
            // Nothing new for a beat once something has gone out: the over is
            // over.
            match quiet_since {
                Some(t) if t.elapsed() > Duration::from_millis(250) => break,
                Some(_) => {}
                None => quiet_since = Some(Instant::now()),
            }
        } else {
            quiet_since = None;
        }
    }
    let out = std::mem::take(&mut *heard.lock().unwrap());

    drop(h.cmd_tx);
    drop(h.event_rx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    assert!(out.blocks > 0, "nothing ever reached the rig's sound card");
    out.audio
}

/// Demodulate what the rig was given and return every frame that survived its
/// check sequence.
fn frames_in(audio: &[f32]) -> Vec<Vec<u8>> {
    frames_in_at(audio, 48_000.0)
}

fn frames_in_at(audio: &[f32], rate: f64) -> Vec<Vec<u8>> {
    let mut modem = AfskRx::new(rate, AfskProfile::Vhf1200);
    let mut deframer = Deframer::new();
    let (mut levels, mut frames) = (Vec::new(), Vec::new());
    for chunk in audio.chunks(480) {
        levels.clear();
        modem.process(chunk, &mut levels);
        for lvl in levels.drain(..) {
            deframer.push_level(lvl, &mut frames);
        }
    }
    frames
}

/// A beacon, all the way from the button to the sound card and back through a
/// receiver.
#[test]
fn a_beacon_reaches_the_rig_as_a_decodable_frame() {
    for audio_mode in [true, false] {
        beacon_on(audio_mode);
    }
}

fn beacon_on(audio_mode: bool) {
    let audio = over_for(
        audio_mode,
        vec![
            Command::SetMode { rx: RxId::Main, mode: Mode::Aprs },
            Command::SetDigiConfig(station()),
            Command::AprsBeacon,
        ],
    );
    let secs = audio.len() as f32 / 48_000.0;
    // 500 ms of TXDELAY flags plus a position report; materially less than that
    // means the over was cut off.
    assert!(secs > 0.5, "audio_mode={audio_mode}: the over was only {secs:.3} s");

    let frames = frames_in(&audio);
    assert_eq!(
        frames.len(),
        1,
        "audio_mode={audio_mode}: {secs:.3} s of audio reached the rig and {} frames came back \
         out of it",
        frames.len()
    );
    let p = Packet::parse(&frames[0], None).expect("the frame does not parse");
    assert_eq!(p.src().call(), "OE3JJS");
    let PacketType::Ui(ui) = p.packet_type() else { panic!("a beacon must be a UI frame") };
    let data = sdroxide_aprs::parse(p.dst().call(), &ui.payload).expect("not APRS");
    let pos = data.position().expect("no position in the beacon");
    assert!((pos.pos.lat - 48.2).abs() < 0.2, "{}", pos.pos.lat);
    assert!((pos.pos.lon - 16.4).abs() < 0.4, "{}", pos.pos.lon);
}

/// ...and a message, which is what the report was actually about.
#[test]
fn a_message_reaches_the_rig_as_a_decodable_frame() {
    let audio = over_for(
        true,
        vec![
            Command::SetMode { rx: RxId::Main, mode: Mode::Aprs },
            Command::SetDigiConfig(station()),
            Command::AprsSendMessage { to: "VK2ABC".into(), text: "hello from sdroxide".into() },
        ],
    );
    let secs = audio.len() as f32 / 48_000.0;
    assert!(secs > 0.5, "the over reached the rig as only {secs:.3} s of audio");

    let frames = frames_in(&audio);
    assert!(!frames.is_empty(), "{secs:.3} s of audio reached the rig and nothing decoded");
    let p = Packet::parse(&frames[0], None).expect("the frame does not parse");
    let PacketType::Ui(ui) = p.packet_type() else { panic!("must be a UI frame") };
    let data = sdroxide_aprs::parse(p.dst().call(), &ui.payload).expect("not APRS");
    match data {
        sdroxide_aprs::AprsData::Message(m) => {
            assert_eq!(m.addressee, "VK2ABC");
            assert_eq!(m.text, "hello from sdroxide");
        }
        other => panic!("{other:?}"),
    }
}

/// Control: the same harness driving the *packet* mode, which shares the modem
/// and the channel-access rules. Tells an APRS-specific fault from one in the
/// AX.25 layer under it.
#[test]
fn a_packet_beacon_reaches_the_rig() {
    let cfg = DigiConfig {
        packet_mycall: "OE3JJS".into(),
        packet_beacon_text: "test".into(),
        packet_persist: 255,
        packet_slottime_ms: 10,
        ..DigiConfig::default()
    };
    let audio = over_for(
        false,
        vec![
            Command::SetMode { rx: RxId::Main, mode: Mode::Packet },
            Command::SetDigiConfig(cfg),
            Command::PacketBeacon,
        ],
    );
    let secs = audio.len() as f32 / 48_000.0;
    assert!(secs > 0.5, "the over reached the rig as only {secs:.3} s of audio");
}

/// A radio that receives at one rate and transmits at another (issue #150).
///
/// An IC-705 on its 12 kHz IF output is exactly this: the IF arrives decimated
/// to 24 kHz, so the digital modes are built at 24 kHz — a `DigiEngine` keeps
/// one clock for both directions — while transmit audio goes back at the
/// session's 48. Field report: the burst was structurally perfect, half as
/// long, at twice the baud rate, and no receiver on the channel could read it.
/// Confirmed off the air with a recording: 2400 and 4400 Hz where Bell 202
/// wants 1200 and 2200.
#[test]
fn a_beacon_is_rate_matched_when_the_radio_transmits_at_another_rate() {
    let rx_rate = 24_000.0;
    let tx_rate = 48_000.0;
    let audio = over_at(
        false,
        rx_rate,
        tx_rate,
        vec![
            Command::SetMode { rx: RxId::Main, mode: Mode::Aprs },
            Command::SetDigiConfig(station()),
            Command::AprsBeacon,
        ],
    );
    // Read back at the rate the *radio* plays, which is the whole point: the
    // frame has to be right in the radio's own time base, not the modem's.
    let secs = audio.len() as f32 / tx_rate as f32;
    assert!(
        secs > 0.5,
        "the over lasted {secs:.3} s in the radio's time base — half length is the fault this \
         test exists for"
    );
    let frames = frames_in_at(&audio, tx_rate);
    assert_eq!(
        frames.len(),
        1,
        "{secs:.3} s reached the radio at {tx_rate} Hz and {} frames came back out of it",
        frames.len()
    );
    let p = Packet::parse(&frames[0], None).expect("the frame does not parse");
    assert_eq!(p.src().call(), "OE3JJS");
}

/// The transmit audio level reaches the radio, and only where the radio is the
/// thing doing the modulating. APRS is FM data, so it is the FM level of the
/// two — `digi_tx_level::the_sideband_level_is_the_one_a_data_mode_uses` holds
/// the other end of that.
///
/// On FM that level is the deviation and nothing else sets it: an over that
/// over-deviates sounds completely normal and decodes for nobody, which is a
/// failure with no symptom an operator can act on. Full scale stays the
/// default, so this changes nothing for anyone who does not reach for it.
#[test]
fn the_transmit_audio_level_scales_what_the_radio_is_given() {
    let peak_at = |level: f32| {
        let audio = over_for(
            true,
            vec![
                Command::SetMode { rx: RxId::Main, mode: Mode::Aprs },
                Command::SetDigiConfig(DigiConfig { tx_audio_level_fm: level, ..station() }),
                Command::AprsBeacon,
            ],
        );
        audio.iter().fold(0.0f32, |m, s| m.max(s.abs()))
    };
    let full = peak_at(1.0);
    let half = peak_at(0.5);
    assert!(full > 0.9, "the default is full scale into the radio: {full}");
    assert!(
        (half / full - 0.5).abs() < 0.08,
        "half the level should be half the amplitude: {half} against {full}"
    );
    // ...and what came out at half level is still a frame, not a fainter mess.
    let audio = over_for(
        true,
        vec![
            Command::SetMode { rx: RxId::Main, mode: Mode::Aprs },
            Command::SetDigiConfig(DigiConfig { tx_audio_level_fm: 0.5, ..station() }),
            Command::AprsBeacon,
        ],
    );
    assert_eq!(frames_in(&audio).len(), 1, "turning the level down broke the frame");
}

#[test]
#[ignore = "measurement, not an assertion"]
fn measure_headroom() {
    for (name, rx, tx) in [("48k straight", 48_000.0, 48_000.0), ("24k -> 48k", 24_000.0, 48_000.0)]
    {
        let audio = over_at(
            false,
            rx,
            tx,
            vec![
                Command::SetMode { rx: RxId::Main, mode: Mode::Aprs },
                Command::SetDigiConfig(station()),
                Command::AprsBeacon,
            ],
        );
        let loud: Vec<f32> = audio.iter().copied().filter(|s| s.abs() > 0.05).collect();
        let peak = loud.iter().fold(0.0f32, |m, s| m.max(s.abs()));
        let at_full = loud.iter().filter(|s| s.abs() >= 0.999).count();
        let rms = (loud.iter().map(|s| s * s).sum::<f32>() / loud.len() as f32).sqrt();
        eprintln!(
            "{name}: peak={peak:.4} rms/peak={:.4} at_full={:.2}%",
            rms / peak,
            100.0 * at_full as f32 / loud.len() as f32
        );
    }
}

// ---------------------------------------------------------------------------
// The third shape: a radio that takes modulated I/Q.
// ---------------------------------------------------------------------------

/// Every I/Q sample the transmitter was handed, in order.
#[derive(Default)]
struct Radiated {
    iq: Vec<Complex32>,
    blocks: usize,
}

/// A transmitting SDR: quadrature in, quadrature out, half duplex, a zero-IF
/// LO parked clear of the dial.
///
/// A HackRF is this shape, and so is every other SDR that transmits. The engine
/// modulates for it — `Engine::tx_block_digi`, modulator and DUC — which is a
/// third path through the transmit side, and the only one nothing above
/// reaches: both radios above hand the *rig* audio and let it modulate.
struct TransmittingSdr {
    heard: Arc<Mutex<Radiated>>,
    rate: f64,
    rng: u32,
}

impl IqSource for TransmittingSdr {
    fn sample_rate(&self) -> f64 {
        self.rate
    }
    fn center_hz(&self) -> f64 {
        DIAL_HZ
    }
    fn set_center_hz(&mut self, _hz: f64) -> Result<()> {
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(4096);
        // Noise in both components, as a quadrature front end delivers it: the
        // packet channel counts its channel-access slots on the receive sample
        // clock, so a source that hands back nothing is a station that can
        // never decide to key.
        for z in &mut buf[..n] {
            self.rng = self.rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let i = (self.rng >> 16) as f32 / 32_768.0 - 1.0;
            self.rng = self.rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let q = (self.rng >> 16) as f32 / 32_768.0 - 1.0;
            *z = Complex32::new(i * 0.05, q * 0.05);
        }
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock transmitting SDR".into()
    }
    fn lo_offset_hz(&self) -> f64 {
        sdroxide_radio::lo_offset_for(self.rate, self.rate * 0.75)
    }
    fn tx_begin(&mut self, _center_hz: f64, _rate: f64) -> Result<f64> {
        Ok(self.rate)
    }
    fn tx_write(&mut self, samples: &[Complex32]) -> Result<()> {
        let mut h = self.heard.lock().unwrap();
        h.blocks += 1;
        h.iq.extend_from_slice(samples);
        Ok(())
    }
    fn tx_end(&mut self) -> Result<()> {
        Ok(())
    }
}

fn sdr_caps() -> DeviceCaps {
    DeviceCaps {
        driver: "mock-sdr".into(),
        label: "mock transmitting SDR".into(),
        rx_channels: 1,
        tx_channels: 1,
        full_duplex: false,
        freq_ranges_rx: vec![(1_000_000.0, 6_000_000_000.0)],
        freq_ranges_tx: vec![(1_000_000.0, 6_000_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// Run `cmds` on an APRS station whose radio takes I/Q, and hand back
/// everything that reached the transmitter.
fn radiated_for(rate: f64, cmds: Vec<Command>) -> Vec<Complex32> {
    isolate_config();
    let heard = Arc::new(Mutex::new(Radiated::default()));
    let src = TransmittingSdr { heard: Arc::clone(&heard), rate, rng: 0x2345_6789 };
    let (producer, _consumer) = rtrb::RingBuffer::<f32>::new(48_000);
    let cfg = EngineConfig {
        // The shipped default rather than the test-friendly one: an APRS
        // channel is inside an amateur band, so the lockout has no business
        // refusing it, and a station whose only radio is an SDR meets that
        // lockout on every over.
        tx_ham_only: true,
        remember_session: false,
        audio: Some(AudioParams { producer, out_rate: 48_000.0 }),
        ..Default::default()
    };
    let mut h = start_engine(Box::new(src), sdr_caps(), cfg);
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(200));
    for c in cmds {
        h.cmd_tx.send(c).expect("engine gone");
        std::thread::sleep(Duration::from_millis(80));
    }
    let deadline = Instant::now() + Duration::from_secs(8);
    let mut quiet_since: Option<Instant> = None;
    while Instant::now() < deadline {
        while h.event_rx.try_recv().is_ok() {}
        let blocks = heard.lock().unwrap().blocks;
        std::thread::sleep(Duration::from_millis(50));
        let after = heard.lock().unwrap().blocks;
        if after > 0 && after == blocks {
            match quiet_since {
                Some(t) if t.elapsed() > Duration::from_millis(250) => break,
                Some(_) => {}
                None => quiet_since = Some(Instant::now()),
            }
        } else {
            quiet_since = None;
        }
    }
    let out = std::mem::take(&mut *heard.lock().unwrap());
    drop(h.cmd_tx);
    drop(h.event_rx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    out.iq
}

/// FM-demodulate radiated I/Q back to audio at `rate / decim`, the way a
/// receiver on the channel would.
///
/// Scaled by the deviation the mode transmits at, so full scale here means
/// [`sdroxide_dsp::PACKET_DEVIATION_HZ`] and the modem is handed the same
/// numbers a rig's discriminator would give it. A raw phase step would shrink
/// with the sample rate instead — 0.002 radians at 10 Msps — and read as a
/// station that had barely modulated.
///
/// The mean is removed because the signal does not sit on the centre of the
/// span: a carrier offset is a constant term out of a discriminator, and a real
/// receiver tunes it out.
fn discriminate(iq: &[Complex32], rate: f64, decim: usize) -> Vec<f32> {
    let scale = (rate / (std::f64::consts::TAU * sdroxide_dsp::PACKET_DEVIATION_HZ)) as f32;
    let mut audio: Vec<f32> = Vec::with_capacity(iq.len() / decim);
    for w in iq.chunks(decim) {
        let mut acc = 0.0f32;
        for p in w.windows(2) {
            acc += (p[1] * p[0].conj()).arg();
        }
        audio.push(acc / w.len().max(1) as f32 * scale);
    }
    let mean = audio.iter().sum::<f32>() / audio.len().max(1) as f32;
    for a in &mut audio {
        *a -= mean;
    }
    audio
}

/// A beacon from the button, through the modulator and the upconverter, off the
/// transmitter and back through a receiver on the channel.
fn beacon_over_iq(rate: f64, decim: usize) {
    let iq = radiated_for(
        rate,
        vec![
            Command::SetMode { rx: RxId::Main, mode: Mode::Aprs },
            Command::SetDigiConfig(station()),
            Command::AprsBeacon,
        ],
    );
    let msps = rate / 1e6;
    assert!(!iq.is_empty(), "{msps} Msps: nothing reached the transmitter at all");
    let secs = iq.len() as f32 / rate as f32;
    assert!(secs > 0.5, "{msps} Msps: the over was only {secs:.3} s");

    let audio = discriminate(&iq, rate, decim);
    let frames = frames_in_at(&audio, rate / decim as f64);
    assert_eq!(
        frames.len(),
        1,
        "{msps} Msps: {secs:.3} s went out and {} frames came back",
        frames.len()
    );
    let p = Packet::parse(&frames[0], None).expect("the frame does not parse");
    assert_eq!(p.src().call(), "OE3JJS");
}

/// The whole transmit path on a radio that takes I/Q, at the rate a HackRF
/// comes up on.
#[test]
fn a_beacon_reaches_a_transmitting_sdr() {
    beacon_over_iq(2_000_000.0, 40);
}

/// Issue #180: the same station on a wider sample rate put nothing on the air
/// that anyone could read.
///
/// 10 Msps is not incidental. The decimation chain lands the channel on
/// 44642.857 Hz there — 37.2 samples to a 1200 baud bit, where 48 kHz gives
/// exactly 40 — and the transmit modem was rounding every bit up to a whole
/// sample. The over went out at 1174.8 baud, 2.1 % slow, which is past what a
/// receiver's clock recovery will follow: a transmission that looked perfect on
/// a spectrum display and decoded for nobody.
///
/// Every rate whose channel works out to a whole number of samples a bit still
/// passed throughout, which is why a sound card never showed this and the test
/// above would not have caught it.
#[test]
fn a_beacon_survives_a_channel_rate_that_is_not_whole_samples_a_bit() {
    beacon_over_iq(10_000_000.0, 200);
}
