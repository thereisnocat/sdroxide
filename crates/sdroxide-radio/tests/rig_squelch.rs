//! The squelch on a rig that hands sdroxide audio it has already gated
//! (issue #192).
//!
//! On such a radio the software squelch is not the one the operator hears.
//! What reaches the sound card has already been through the rig's own gate, so
//! a threshold applied on this side can close further on what got through and
//! can never open up a weak station the radio muted — which is how an operator
//! ended up with a squelch control that could not reach the thing quietening
//! their receiver. A front end that says it has one gets the SQL rail; every
//! other one keeps the engine's own dBFS gate, which is the honest one there.

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, ControlUpdate, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, RadioEvent, RxId};

const RATE: f64 = 48_000.0;

/// A stand-in for a CAT rig on a sound card: audio in, and a squelch in the
/// radio that sdroxide can command. `adopt` is the level the radio reports when
/// its control link opens, delivered once.
struct SquelchRig {
    center: f64,
    told: Arc<Mutex<Vec<f32>>>,
    adopt: Option<f32>,
}

impl IqSource for SquelchRig {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        self.center
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(2048);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock rig with its own squelch".into()
    }
    fn display_bandwidth(&self) -> Option<f64> {
        Some(4000.0)
    }
    fn commands_squelch(&self) -> bool {
        true
    }
    fn set_squelch(&mut self, frac: f32) {
        self.told.lock().unwrap().push(frac);
    }
    fn poll_control(&mut self) -> Vec<ControlUpdate> {
        self.adopt.take().map(ControlUpdate::Squelch).into_iter().collect()
    }
}

/// The same rig with no squelch of its own — an SDR, as far as this is
/// concerned.
struct SilentRig {
    center: f64,
    told: Arc<Mutex<Vec<f32>>>,
}

impl IqSource for SilentRig {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        self.center
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(2048);
        buf[..n].fill(Complex32::new(0.0, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock front end with no squelch of its own".into()
    }
    fn set_squelch(&mut self, frac: f32) {
        self.told.lock().unwrap().push(frac);
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "cat".into(),
        label: "mock CAT rig".into(),
        rx_channels: 1,
        tx_channels: 1,
        audio_mode: true,
        freq_ranges_rx: vec![(10_000.0, 148_000_000.0)],
        freq_ranges_tx: vec![(1_800_000.0, 54_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// Run the engine on `src`, feed it `cmds`, and return the last state it
/// broadcast alongside everything the rig was told.
fn run(src: Box<dyn IqSource>, cmds: Vec<Command>) -> sdroxide_types::RadioState {
    let mut h = start_engine(src, caps(), EngineConfig::default());
    let thread = h.thread.take();
    std::thread::sleep(Duration::from_millis(300));
    for c in cmds {
        h.cmd_tx.send(c).expect("engine gone");
        std::thread::sleep(Duration::from_millis(60));
    }
    let mut last = None;
    let deadline = Instant::now() + Duration::from_millis(300);
    while Instant::now() < deadline {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::State(s) = ev {
                last = Some(s);
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    drop(h.cmd_tx);
    drop(h.event_rx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    last.expect("the engine never broadcast a state")
}

/// The rail reaches the radio, and the state carries what was asked for so
/// every client's rail reads the same figure.
#[test]
fn the_squelch_the_operator_sets_is_sent_to_the_radio() {
    let told = Arc::new(Mutex::new(Vec::new()));
    let src = SquelchRig { center: 145_500_000.0, told: Arc::clone(&told), adopt: None };
    let state = run(Box::new(src), vec![Command::SetRigSquelch { frac: 0.4 }]);

    let told = told.lock().unwrap().clone();
    assert!(told.contains(&0.4), "the radio was never given the level: {told:?}");
    assert!((state.rig_squelch - 0.4).abs() < 1e-6, "state says {}", state.rig_squelch);
    // And the engine's own gate was not moved instead: the two are different
    // controls on different scales, and writing one from the other would put a
    // dBFS threshold at 0.4 dB below full scale.
    assert_eq!(
        state.rx[0].squelch_db,
        sdroxide_types::SQUELCH_OPEN_DB,
        "the software gate was moved by a command meant for the radio"
    );
}

/// Out-of-range levels are clamped rather than passed on: this is a fraction of
/// the radio's own scale, and a client is a file away from sending anything.
#[test]
fn a_level_outside_the_radios_scale_is_clamped() {
    let told = Arc::new(Mutex::new(Vec::new()));
    let src = SquelchRig { center: 145_500_000.0, told: Arc::clone(&told), adopt: None };
    let state = run(Box::new(src), vec![Command::SetRigSquelch { frac: 7.5 }]);
    assert_eq!(state.rig_squelch, 1.0);
    assert_eq!(told.lock().unwrap().last().copied(), Some(1.0));
}

/// The level the radio reports when its control link opens is *adopted*, not
/// overridden — the operator set it at the front panel, and it is what the
/// audio arriving is already being gated by. Nothing is sent back at the rig.
#[test]
fn the_level_the_radio_reports_is_adopted_rather_than_overridden() {
    let told = Arc::new(Mutex::new(Vec::new()));
    let src = SquelchRig { center: 145_500_000.0, told: Arc::clone(&told), adopt: Some(0.62) };
    let state = run(Box::new(src), Vec::new());
    assert!((state.rig_squelch - 0.62).abs() < 1e-6, "state says {}", state.rig_squelch);
    assert!(
        told.lock().unwrap().is_empty(),
        "the radio's own setting was answered with a command: {:?}",
        told.lock().unwrap()
    );
}

/// A front end with no squelch of its own says so, and the capability the UI
/// picks the rail from follows the source rather than the configuration.
#[test]
fn a_front_end_without_one_reports_that_it_has_none() {
    let told = Arc::new(Mutex::new(Vec::new()));
    let src = SilentRig { center: 145_500_000.0, told: Arc::clone(&told) };
    let mut h = start_engine(Box::new(src), caps(), EngineConfig::default());
    let thread = h.thread.take();
    let deadline = Instant::now() + Duration::from_millis(600);
    let mut caps_seen = None;
    while Instant::now() < deadline {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::Capabilities(c) = ev {
                caps_seen = Some(c);
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    // The rail the client draws follows this, so a source that cannot command a
    // squelch must not be given the control that does.
    let c = caps_seen.expect("the engine never published its capabilities");
    assert!(!c.commands_squelch, "a front end with no squelch claimed one");

    h.cmd_tx.send(Command::SetSquelch { rx: RxId::Main, db: -80.0 }).expect("engine gone");
    std::thread::sleep(Duration::from_millis(120));
    drop(h.cmd_tx);
    drop(h.event_rx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    assert!(
        told.lock().unwrap().is_empty(),
        "the engine's own squelch was pushed at a radio that has none"
    );
}
