//! The main panadapter on a radio that sends audio and draws its own spectrum.
//!
//! An Icom over its LAN port with the output set to AF hands over demodulated
//! audio and nothing else — there is no I/Q on any Icom — so the main lane used
//! to be an FFT of that audio, mapped USB-style from the dial upwards. That is
//! not a picture of the band: it is one-sided by construction, so the whole
//! display sat above the dial with the signal jammed against the left edge, and
//! it was never wider than the rig's own filter however wide the display was
//! set. The radio's `27 00` scope is the only real spectrum such a session has,
//! it is centred on the dial, and it is what every other client of these radios
//! draws — so it is what the main lane shows.
//!
//! Except in the digital modes, where the waterfall *is* the audio band: FT8
//! places signals by their offset inside the rig's passband, and a band-wide
//! scope at a few hundred Hz a bin cannot show one at all.

use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, EngineConfig, IqSource, Result, start_engine};
use sdroxide_types::{Command, DeviceCaps, Mode, RadioEvent, RxId, SpectrumConfig, SpectrumFrame};

/// The audio-band lane reaches its window through the analyser's fractional
/// viewport, so its edges land a few parts in 10^15 off the round number. Close
/// enough that a hertz of slack tells the two lanes apart with room to spare.
#[track_caller]
fn about(got: f64, want: f64, what: &str) {
    assert!((got - want).abs() < 1.0, "{what}: got {got}, expected {want}");
}

/// The sound card's rate — what the radio actually sends.
const RATE: f64 = 48_000.0;
/// The narrow window the audio FFT would be mapped onto.
const AUDIO_BW: f64 = 4_000.0;
const DIAL: f64 = 14_074_000.0;
/// What the radio's own scope is sweeping: ±100 kHz, its centre on the dial.
const SCOPE_SPAN: f64 = 200_000.0;
/// Bins in one sweep, as an Icom sends them.
const SCOPE_BINS: usize = 475;
/// Which bin carries the marker, and how far off the scope centre that is.
const HOT_BIN: usize = 100;

/// A demod-audio rig that also publishes the radio's own scope: audio in the
/// real component, a finished sweep in the full-band lane.
struct ScopeRig {
    center: f64,
    /// Set to stop publishing sweeps, so the fall-back to the audio FFT can be
    /// exercised without waiting out the staleness window.
    scope: bool,
}

impl IqSource for ScopeRig {
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
    fn center_is_dial(&self) -> bool {
        true
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        std::thread::sleep(Duration::from_millis(5));
        let n = buf.len().min(512);
        buf[..n].fill(Complex32::new(0.01, 0.0));
        Ok(n)
    }
    fn describe(&self) -> String {
        "mock Icom over LAN (AF)".into()
    }
    /// Demodulated audio: this is what puts the engine in audio mode.
    fn display_bandwidth(&self) -> Option<f64> {
        Some(AUDIO_BW)
    }
    /// One finished sweep per poll — a noise floor with a single strong bin, so
    /// where it lands in the emitted frame says which lane built it.
    fn wide_spectrum_db(&mut self, out: &mut Vec<f32>) -> Option<(f64, f64)> {
        if !self.scope {
            return None;
        }
        out.clear();
        out.resize(SCOPE_BINS, -110.0);
        out[HOT_BIN] = -30.0;
        Some((self.center, SCOPE_SPAN))
    }
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "icomnet".into(),
        label: "mock Icom over LAN".into(),
        rx_channels: 1,
        audio_mode: true,
        freq_ranges_rx: vec![(30_000.0, 10_500_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// Run the engine for long enough to settle, optionally sending commands first,
/// and return the last main-lane frame and the last state it published.
fn run(scope: bool, cmds: &[Command]) -> (SpectrumFrame, sdroxide_types::RadioState) {
    let mut h =
        start_engine(Box::new(ScopeRig { center: DIAL, scope }), caps(), EngineConfig::default());
    let thread = h.thread.take();

    std::thread::sleep(Duration::from_millis(150));
    for c in cmds {
        h.cmd_tx.send(c.clone()).unwrap();
    }

    let mut frame = None;
    let mut state = None;
    let deadline = Instant::now() + Duration::from_millis(900);
    while Instant::now() < deadline {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::State(s) = ev {
                state = Some(s);
            }
        }
        if h.spectrum_out.update() {
            frame = Some(h.spectrum_out.output_buffer().clone());
        }
        std::thread::sleep(Duration::from_millis(20));
    }

    drop(h.cmd_tx);
    if let Some(t) = thread {
        let _ = t.join();
    }
    (frame.expect("the engine published no spectrum"), state.expect("no state published"))
}

#[test]
fn the_radios_own_scope_is_the_main_panadapter() {
    let (frame, _) = run(true, &[Command::SetMode { rx: RxId::Main, mode: Mode::Cw }]);
    assert_eq!(frame.span_hz, SCOPE_SPAN, "the main lane is not the scope's span");
    assert_eq!(frame.center_hz, DIAL, "the scope is centred on the dial");
    // Centred, not jammed against an edge: the dial sits in the middle of the
    // window, which is the whole complaint this replaces.
    let lo = frame.center_hz - frame.span_hz / 2.0;
    assert!(lo < DIAL && DIAL < frame.center_hz + frame.span_hz / 2.0);

    // And the trace is the scope's, not the audio's: the marker bin lands where
    // the sweep put it, a fifth of the way across.
    let peak = frame
        .bins
        .iter()
        .enumerate()
        .max_by_key(|(_, v)| **v)
        .map(|(i, _)| i)
        .expect("an empty frame");
    let want = HOT_BIN * frame.bins.len() / SCOPE_BINS;
    assert!(
        peak.abs_diff(want) <= frame.bins.len() / SCOPE_BINS + 1,
        "marker at bin {peak}, expected about {want}"
    );
}

#[test]
fn the_state_describes_the_window_the_panadapter_is_showing() {
    // The client's zoom clamp and sub-receiver limits are built on these two
    // numbers; left describing the audio band they would pin the view to 4 kHz
    // of a panadapter two hundred wide.
    let (_, state) = run(true, &[Command::SetMode { rx: RxId::Main, mode: Mode::Cw }]);
    assert_eq!(state.sample_rate, SCOPE_SPAN);
    assert_eq!(state.center_hz, DIAL);
}

#[test]
fn a_digital_mode_gets_the_audio_band_back() {
    // FT8 places stations by their audio offset within the passband. A 200 kHz
    // sweep at 421 Hz a bin cannot show one, so the scope stands aside.
    let (frame, state) = run(true, &[Command::SetMode { rx: RxId::Main, mode: Mode::Ft8 }]);
    about(frame.span_hz, AUDIO_BW, "a digital mode must keep the audio-band window");
    assert_eq!(state.sample_rate, AUDIO_BW);
    // USB-side: audio f maps to dial + f, so the window hangs off the dial.
    about(frame.center_hz, DIAL + AUDIO_BW / 2.0, "the audio window hangs off the dial");
}

/// Zoom in far enough and the scope stops being the better picture.
///
/// A serial CAT rig's sweep is a fixed number of points across whatever span it
/// was told to cover — 475 on an IC-705 — so past a point the operator is
/// magnifying rather than resolving, and a signal stays one block wide however
/// far they go. The audio the same rig is already sending covers exactly the
/// passband at some three hertz a bin, so inside that window it draws instead.
#[test]
fn a_view_inside_the_passband_is_drawn_from_the_audio_and_not_the_scope() {
    // A kilohertz inside the upper-sideband passband, well clear of its edges.
    let (lo, hi) = (DIAL + 700.0, DIAL + 1_700.0);
    let cfg = SpectrumConfig { viewport: Some((lo, hi)), ..SpectrumConfig::default() };
    let (frame, state) = run(
        true,
        &[Command::SetMode { rx: RxId::Main, mode: Mode::Usb }, Command::SetSpectrumCfg(cfg)],
    );

    about(frame.span_hz, hi - lo, "the frame is the window that was asked for");
    about(frame.center_hz, (lo + hi) / 2.0, "centred on that window");

    // The display axis stays the scope's, so zooming back out is the gesture it
    // always was rather than a jump to a four-kilohertz panadapter.
    assert_eq!(
        state.sample_rate, SCOPE_SPAN,
        "the axis followed the picture out from under the view"
    );
    assert_eq!(state.center_hz, DIAL);

    // And the waterfall gets its rows clocked again: a continuous analyser is
    // drawing now, not a sweep that arrives finished.
    assert!(frame.rows_clocked, "the audio lane has to clock its own rows");
}

/// Wider than the passband there is nothing to switch to — the audio is a
/// picture of what the rig demodulated, not of the band — so the scope keeps it.
#[test]
fn a_view_wider_than_the_passband_stays_on_the_scope() {
    let (lo, hi) = (DIAL - 20_000.0, DIAL + 20_000.0);
    let cfg = SpectrumConfig { viewport: Some((lo, hi)), ..SpectrumConfig::default() };
    let (frame, _) = run(
        true,
        &[Command::SetMode { rx: RxId::Main, mode: Mode::Usb }, Command::SetSpectrumCfg(cfg)],
    );
    about(frame.span_hz, hi - lo, "the scope serves the window it was asked for");
    // The scope arrives finished, so the client scrolls it on its own clock.
    assert!(!frame.rows_clocked, "a finished sweep must not claim to clock rows");
    // ...and it must not carry any either: the client repeats the spectrum on
    // its own wall clock for such a lane, and rows sent alongside that
    // instruction are a picture it would draw twice.
    assert!(frame.rows.is_empty(), "a lane that does not clock rows handed some out");
}

#[test]
fn a_session_without_a_scope_is_left_as_it_was() {
    // The scope is an operator setting on this backend and not every model has
    // one. With no sweeps at all the main lane is the audio FFT, exactly as
    // before — a front end that publishes nothing must not lose its panadapter.
    let (frame, state) = run(false, &[Command::SetMode { rx: RxId::Main, mode: Mode::Cw }]);
    about(frame.span_hz, AUDIO_BW, "the audio FFT is still the panadapter");
    assert_eq!(state.sample_rate, AUDIO_BW);
}
