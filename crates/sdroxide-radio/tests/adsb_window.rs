//! The ADS-B lane, from a mock front end to an aircraft on the panel's table
//! (issue #160).
//!
//! Two things are pinned here, and neither can be seen from inside the decoder
//! crate:
//!
//! * **The plumbing carries a decode.** A stand-in receiver on 1090 MHz
//!   transmits a real extended squitter twice a second; selecting the mode has
//!   to end with that aircraft in an `AdsbStatus`, with the identity and the
//!   position the frames were built from. Every piece between the source and
//!   the event — the tap in the audio pass, the window's downconverter, the
//!   worker thread, the snapshot — is in that path.
//!
//! * **A receiver that cannot do it says so.** Mode S needs at least two
//!   megasamples a second and there is no way to manufacture them downstream,
//!   so a narrow front end must produce a sentence rather than an empty list.
//!   "No aircraft" and "this receiver was never going to hear any" look
//!   identical otherwise, and only one of them is the operator's to fix.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_radio::{AudioParams, Complex32, EngineConfig, IqSource, Result, rtrb, start_engine};
use sdroxide_types::{AdsbStatus, Command, DeviceCaps, Mode, RadioEvent, RxId, Vfo};

const ADSB_HZ: f64 = 1_090_000_000.0;

/// The published even/odd position pair for one aircraft, and the published
/// identification squitter for another. Between them they carry everything a
/// row needs: an address, a callsign, an altitude and — from the pair — a
/// place.
const POSITIONS: [&str; 2] = ["8D40621D58C382D690C8AC2863A7", "8D40621D58C386435CC412692AD6"];
const IDENT: &str = "8D4840D6202CC371C32CE0576098";

fn bytes_of(hex: &str) -> Vec<u8> {
    (0..hex.len() / 2).map(|k| u8::from_str_radix(&hex[k * 2..k * 2 + 2], 16).unwrap()).collect()
}

/// The three frames this aeroplane transmits, all under one address.
///
/// The identification squitter is the published one with its address changed to
/// the position pair's, and then **re-sealed**: the check sequence covers the
/// address, so editing the bytes without recomputing it produces a frame the
/// decoder is right to throw away. Doing that by hand once, and having the test
/// fail, is how this comment came to be here.
fn traffic() -> Vec<Vec<u8>> {
    let mut ident = bytes_of(IDENT);
    let addr = bytes_of(POSITIONS[0]);
    ident[1..4].copy_from_slice(&addr[1..4]);
    sdroxide_adsb::crc::seal(&mut ident, 0);
    let mut out: Vec<Vec<u8>> = POSITIONS.iter().map(|h| bytes_of(h)).collect();
    out.push(ident);
    out
}

/// A front end on 1090 MHz that transmits one aeroplane.
///
/// The bursts are pre-modulated once and handed out a block at a time, so the
/// source costs nothing per block and the test is not racing its own generator.
struct Sky {
    center: Arc<Mutex<f64>>,
    rate: f64,
    /// The whole loop of traffic, already modulated.
    samples: Vec<Complex32>,
    pos: usize,
}

impl Sky {
    fn new(rate: f64, center: Arc<Mutex<f64>>) -> Sky {
        let mut samples = Vec::new();
        for (i, bytes) in traffic().iter().enumerate() {
            // The same transmitter the decoder's own tests use, so a
            // disagreement between the two cannot hide here.
            samples.extend(sdroxide_adsb::modulate(bytes, rate, 0.6, 0.01, 0x51_23 + i as u64));
        }
        Sky { center, rate, samples, pos: 0 }
    }
}

impl IqSource for Sky {
    fn sample_rate(&self) -> f64 {
        self.rate
    }
    fn center_hz(&self) -> f64 {
        *self.center.lock().unwrap()
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        *self.center.lock().unwrap() = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        // Not real time: a trickle of blocks keeps the decoder and its status
        // clock talking without spending a core on 2.4 Msps of mostly silence.
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(4096);
        for z in buf[..n].iter_mut() {
            *z = self.samples[self.pos];
            self.pos = (self.pos + 1) % self.samples.len();
        }
        Ok(n)
    }
    fn describe(&self) -> String {
        "one aeroplane".into()
    }
}

fn caps(rate: f64) -> DeviceCaps {
    DeviceCaps {
        driver: "mock".into(),
        label: "mock".into(),
        rx_channels: 1,
        sample_rates: vec![rate],
        freq_ranges_rx: vec![(1_000_000.0, 2_000_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// Wait for a status that satisfies `f`, or say what the last one was.
fn status(
    h: &sdroxide_radio::EngineHandles,
    what: &str,
    mut f: impl FnMut(&AdsbStatus) -> bool,
) -> AdsbStatus {
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut last: Option<AdsbStatus> = None;
    while Instant::now() < deadline {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::AdsbStatus(s) = ev {
                if f(&s) {
                    return *s;
                }
                last = Some(*s);
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    panic!("the ADS-B decoder never reported {what}; last status: {last:#?}");
}

fn isolate_config(name: &str) {
    let root = std::env::temp_dir().join(format!("sdroxide-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    // An engine test that saves anything writes the operator's real config
    // directory unless this is set: the variable is process-global and unset
    // means the live one.
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };
}

fn engine(rate: f64) -> sdroxide_radio::EngineHandles {
    // The lanes below the speaker — the skimmers, the ISM window, this one —
    // are fed from the same pass as the main receiver, so an engine with
    // nowhere to play audio never reaches them. Hence a ring nothing reads.
    let (producer, _consumer) = rtrb::RingBuffer::<f32>::new(48_000);
    let center = Arc::new(Mutex::new(ADSB_HZ));
    start_engine(
        Box::new(Sky::new(rate, center)),
        caps(rate),
        EngineConfig {
            remember_session: false,
            audio: Some(AudioParams { producer, out_rate: 48_000.0 }),
            ..Default::default()
        },
    )
}

#[test]
fn selecting_the_mode_puts_the_aeroplane_on_the_table() {
    isolate_config("adsb-window");
    let mut h = engine(2_400_000.0);
    let thread = h.thread.take();
    let send = |c: Command| h.cmd_tx.send(c).unwrap();

    send(Command::SetVfo { vfo: Vfo::A, hz: ADSB_HZ });
    send(Command::SetMode { rx: RxId::Main, mode: Mode::Adsb });

    let st = status(&h, "an aircraft with a position and a callsign", |s| {
        s.aircraft.iter().any(|a| a.has_position() && !a.callsign.is_empty())
    });
    assert!(st.unavailable.is_none(), "the lane should be running: {:?}", st.unavailable);
    assert!(st.frames > 0, "frames were accepted");

    let a = st.aircraft.iter().find(|a| a.icao == 0x40_621D).expect("the transmitted address");
    assert_eq!(a.callsign, "KLM1023", "the identification squitter reached the same row");
    assert_eq!(a.altitude_ft, Some(38_000));
    let (lat, lon) = (a.lat.unwrap(), a.lon.unwrap());
    assert!((lat - 52.26).abs() < 0.05, "latitude {lat}");
    assert!((lon - 3.93).abs() < 0.05, "longitude {lon}");

    // The whole stream, undecimated: 2.4 Msps is what the front end delivers
    // and there is no slack in this waveform to give away.
    assert!(
        (st.window_rate_hz - 2_400_000.0).abs() < 1.0,
        "the window should be the whole stream, not {:.0} Hz",
        st.window_rate_hz
    );

    // ...and leaving the mode stops it, because a receiver on 1090 MHz at
    // 2.4 Msps is not listening to anything else and the lane costs a core.
    // Nothing arrives after it goes: silence is what standing down looks like
    // from out here.
    send(Command::SetMode { rx: RxId::Main, mode: Mode::Nfm });
    std::thread::sleep(Duration::from_millis(400));
    while h.event_rx.try_recv().is_ok() {}
    let quiet_until = Instant::now() + Duration::from_millis(1500);
    while Instant::now() < quiet_until {
        while let Ok(ev) = h.event_rx.try_recv() {
            assert!(
                !matches!(ev, RadioEvent::AdsbStatus(_)),
                "the lane was still reporting after the mode changed"
            );
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    drop(h);
    if let Some(t) = thread {
        let _ = t.join();
    }
}

#[test]
fn a_receiver_too_narrow_for_mode_s_says_so_rather_than_going_quiet() {
    isolate_config("adsb-narrow");
    // Under two megasamples a second there is not one sample per chip, and no
    // amount of processing downstream can put one there.
    let mut h = engine(1_024_000.0);
    let thread = h.thread.take();
    let send = |c: Command| h.cmd_tx.send(c).unwrap();

    send(Command::SetVfo { vfo: Vfo::A, hz: ADSB_HZ });
    send(Command::SetMode { rx: RxId::Main, mode: Mode::Adsb });

    let st = status(&h, "the reason it cannot run", |s| s.unavailable.is_some());
    let why = st.unavailable.unwrap();
    assert!(why.contains("Msps"), "the sentence should name the rate: {why}");
    assert!(st.aircraft.is_empty());

    drop(h);
    if let Some(t) = thread {
        let _ = t.join();
    }
}
