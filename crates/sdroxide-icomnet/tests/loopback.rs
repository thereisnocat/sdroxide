//! The client against the simulator, over real UDP sockets.
//!
//! Every byte here is the byte that would go to a transceiver: the handshake,
//! the login, the token dance, the port negotiation, the CI-V tunnel and the
//! PCM stream. With no hardware to test against, this is the evidence that the
//! backend does what it claims.

use std::time::{Duration, Instant};

use sdroxide_icomnet::sim::{Sim, SimOptions};
use sdroxide_icomnet::{Error, IcomNetDevice, IcomNetOptions};

/// Connect to a simulator on the loopback interface.
fn connect(sim: &Sim, tweak: impl FnOnce(&mut IcomNetOptions)) -> Result<IcomNetDevice, Error> {
    let mut opts = IcomNetOptions {
        address: "127.0.0.1".into(),
        control_port: sim.port(),
        username: "operator".into(),
        password: "hunter2".into(),
        timeout: Duration::from_secs(5),
        ..Default::default()
    };
    tweak(&mut opts);
    IcomNetDevice::connect(opts)
}

/// Spin until `f` holds or the deadline passes. Everything here is driven by a
/// real socket and a real clock, so a fixed sleep would be either flaky or slow.
fn wait_for(what: &str, timeout: Duration, mut f: impl FnMut() -> bool) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if f() {
            return;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("timed out waiting for {what}");
}

#[test]
fn the_handshake_completes_and_the_radio_introduces_itself() {
    let sim = Sim::start(SimOptions::default()).unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");

    let info = dev.info();
    assert_eq!(info.radio_name, "IC-7300MK2");
    assert_eq!(info.civ_address, 0xB6);
    // The CI-V address the radio reported is what selects the model entry —
    // nothing was configured.
    assert_eq!(info.model.name, "IC-7300MK2");
    assert!(info.model.lan_mod_input.is_some());
    assert!(info.can_transmit);
    assert_eq!(info.connection, "FTTH");
    assert_eq!(info.baud_rate, 115_200);
    assert!(dev.is_alive());
    assert!(sim.is_streaming());
}

#[test]
fn a_wrong_password_is_reported_as_such_rather_than_timing_out() {
    let sim = Sim::start(SimOptions::default()).unwrap();
    let err = connect(&sim, |o| o.password = "wrong".into()).unwrap_err();
    assert!(matches!(err, Error::BadCredentials), "got {err}");
}

#[test]
fn an_unknown_model_still_connects_but_gets_no_menu_writes() {
    // An IC-7100's CI-V address. It has no network port at all, so it will
    // never be in the table — unlike the IC-9700 that used to stand in here,
    // which is now in it.
    let sim = Sim::start(SimOptions {
        civ_address: 0x88,
        radio_name: "IC-7100".into(),
        ..Default::default()
    })
    .unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");
    assert_eq!(dev.info().civ_address, 0x88);
    assert_eq!(dev.info().model.name, "Icom (LAN)");
    assert!(dev.info().model.lan_mod_input.is_none());
    assert!(dev.info().model.lan_afif_select.is_none());
}

#[test]
fn a_receive_only_radio_offers_no_transmit_ring() {
    let sim = Sim::start(SimOptions { can_transmit: false, ..Default::default() }).unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");
    assert!(!dev.info().can_transmit);
    assert!(dev.take_audio_tx().is_none());
    assert!(dev.take_audio_rx().is_some());
}

#[test]
fn audio_flows_and_carries_the_tone_the_radio_is_sending() {
    let sim = Sim::start(SimOptions { tone_hz: Some(1_000.0), scope: false, ..Default::default() })
        .unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");
    let mut rx = dev.take_audio_rx().expect("an rx ring");

    let mut got = Vec::new();
    wait_for("audio", Duration::from_secs(3), || {
        while let Ok(s) = rx.pop() {
            got.push(s);
        }
        got.len() > 4_800
    });

    // Half scale, so the PCM round trip has not rescaled anything.
    let peak = got.iter().fold(0f32, |m, s| m.max(s.abs()));
    assert!((0.4..0.6).contains(&peak), "peak was {peak}");
    // And the energy really is at 1 kHz — a stuck or repeated buffer would
    // pass an amplitude check but not this one.
    let at_1k = goertzel(&got, 1_000.0, 48_000.0);
    let at_3k = goertzel(&got, 3_000.0, 48_000.0);
    assert!(at_1k > 20.0 * at_3k, "1 kHz {at_1k:.3} against 3 kHz {at_3k:.3}");
}

/// Energy at one frequency, without pulling in an FFT.
fn goertzel(x: &[f32], hz: f64, rate: f64) -> f64 {
    let w = std::f64::consts::TAU * hz / rate;
    let coeff = 2.0 * w.cos();
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for &v in x {
        let s0 = f64::from(v) + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    ((s1 * s1 + s2 * s2 - coeff * s1 * s2).max(0.0)).sqrt() / x.len() as f64
}

#[test]
fn the_measured_rate_matches_what_was_asked_for() {
    let sim =
        Sim::start(SimOptions { sample_rate: 48_000, scope: false, ..Default::default() }).unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");
    let mut rx = dev.take_audio_rx().unwrap();

    // Keep the ring drained, or a full ring throttles what the session counts.
    wait_for("a rate measurement", Duration::from_secs(4), || {
        while rx.pop().is_ok() {}
        dev.measured_rx_rate() > 0
    });
    let measured = dev.measured_rx_rate();
    assert!(
        (43_000..53_000).contains(&measured),
        "measured {measured} Hz against a requested 48000"
    );
}

#[test]
fn civ_goes_out_and_the_reply_comes_back() {
    let sim = Sim::start(SimOptions { scope: false, freq_hz: 14_074_000.0, ..Default::default() })
        .unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");

    // Read frequency: FE FE B6 E0 03 FD.
    dev.send_civ(vec![0xfe, 0xfe, 0xb6, 0xe0, 0x03, 0xfd]);

    let mut reply = None;
    wait_for("a CI-V reply", Duration::from_secs(3), || {
        while let Ok(f) = dev.civ_frames().try_recv() {
            if f.get(4) == Some(&0x03) {
                reply = Some(f);
                return true;
            }
        }
        false
    });
    // 14.074 MHz as Icom's little-endian BCD.
    let f = reply.unwrap();
    assert_eq!(&f[5..10], &[0x00, 0x40, 0x07, 0x14, 0x00]);

    // And the radio saw what we sent.
    wait_for("the radio to record our frame", Duration::from_secs(1), || {
        sim.civ_frames().iter().any(|f| f.get(4) == Some(&0x03))
    });
}

#[test]
fn the_scope_sweep_arrives_whole_the_way_a_lan_icom_sends_it() {
    let sim = Sim::start(SimOptions::default()).unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");

    // Scope data output on: 27 11 01.
    dev.send_civ(vec![0xfe, 0xfe, 0xb6, 0xe0, 0x27, 0x11, 0x01, 0xfd]);

    let mut sweep = None;
    wait_for("a scope sweep", Duration::from_secs(3), || {
        while let Ok(f) = dev.civ_frames().try_recv() {
            if f.get(4) == Some(&0x27) && f.get(5) == Some(&0x00) {
                sweep = Some(f);
                return true;
            }
        }
        false
    });
    let s = sweep.unwrap();
    // FE FE E0 B6 27 00 | vfo div/divmax mode | 10 freq | range | 475 bins | FD
    assert_eq!(s.len(), 6 + 3 + 1 + 10 + 1 + 475 + 1);
    assert_eq!(s[7], 0x01, "division 1");
    assert_eq!(s[8], 0x01, "of 1 — over LAN the sweep is not fragmented");
    let bins = &s[21..21 + 475];
    assert!(bins.iter().all(|&b| b <= 160), "the scale runs 0..160");
    assert!(bins.iter().any(|&b| b > 100), "the simulator plants a peak");
}

/// Issue #183. The IC-7760's CI-V reference gives its LAN sweep as a single
/// 704-byte division — 689 points on a 0 ~ C8 scale — where the IC-7300
/// generation sends 475 on 0 ~ A0. The byte count is the manufacturer's own,
/// so asserting it pins every field ahead of the waveform data as well.
#[test]
fn an_ic7760_sends_the_704_byte_sweep_its_reference_guide_documents() {
    let sim = Sim::start(SimOptions {
        civ_address: 0xB2,
        radio_name: "IC-7760".into(),
        ..Default::default()
    })
    .unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");
    dev.send_civ(vec![0xfe, 0xfe, 0xb2, 0xe0, 0x27, 0x11, 0x01, 0xfd]);

    let mut sweep = None;
    wait_for("a scope sweep", Duration::from_secs(3), || {
        while let Ok(f) = dev.civ_frames().try_recv() {
            if f.get(4) == Some(&0x27) && f.get(5) == Some(&0x00) {
                sweep = Some(f);
                return true;
            }
        }
        false
    });
    let s = sweep.unwrap();
    // FE FE E0 B2 27 00 | 704 bytes of payload | FD
    assert_eq!(s.len(), 6 + 704 + 1);
    assert_eq!(s[8], 0x01, "of 1 — over LAN the sweep is not fragmented");
    let bins = &s[21..21 + 689];
    assert!(bins.iter().all(|&b| b <= 200), "the scale runs 0..200");
    assert!(bins.iter().any(|&b| b > 160), "and the peak goes above where an IC-705 tops out");
}

#[test]
fn the_opening_burst_survives_the_streams_still_coming_up() {
    // `connect` reports the moment the radio names its CI-V port, which is
    // several round trips before that stream is a session the radio will accept
    // anything on. A client configures the radio right then — the menu writes,
    // the offset clear, the scope enable — and nothing ever re-sends those.
    // Written straight out they carry a session id the radio has not issued and
    // are discarded: on loopback the race was usually won, over WiFi never, so
    // the scope never started on a real radio.
    let sim = Sim::start(SimOptions { scope: false, ..Default::default() }).unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");

    // Menu *reads*, so the burst is inert whatever it is pointed at.
    let burst: Vec<Vec<u8>> =
        (0..8).map(|i| vec![0xFE, 0xFE, 0xB6, 0xE0, 0x1A, 0x05, 0x00, i, 0xFD]).collect();
    for f in &burst {
        dev.send_civ(f.clone());
    }

    wait_for("the whole opening burst", Duration::from_secs(5), || {
        let got = sim.civ_frames();
        burst.iter().all(|f| got.contains(f))
    });
    // And in the order the client asked for: a menu write that overtakes the
    // read of the item it is writing would report the old value.
    let got = sim.civ_frames();
    let at: Vec<usize> =
        burst.iter().map(|f| got.iter().position(|g| g == f).expect("present")).collect();
    assert!(at.windows(2).all(|w| w[0] < w[1]), "the burst was reordered: {at:?}");
}

#[test]
fn transmit_audio_reaches_the_radio() {
    let sim = Sim::start(SimOptions { scope: false, ..Default::default() }).unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");
    let mut tx = dev.take_audio_tx().expect("a tx ring");

    // A quarter-scale DC block is trivially recognisable after the PCM round
    // trip, and cannot be confused with the radio's own silence.
    for _ in 0..4_800 {
        let _ = tx.push(0.25);
    }
    wait_for("transmitted audio", Duration::from_secs(3), || sim.tx_audio().len() > 4_000);

    let got = sim.tx_audio();
    let mean = got.iter().sum::<f32>() / got.len() as f32;
    assert!((mean - 0.25).abs() < 0.01, "mean was {mean}");
}

#[test]
fn a_lost_packet_is_noticed_and_asked_for_again() {
    // Every eighth packet the radio sends never arrives. The client must spot
    // the sequence gap and ask; nothing else recovers the stream.
    let sim = Sim::start(SimOptions {
        drop_every: 8,
        tone_hz: Some(1_000.0),
        scope: false,
        ..Default::default()
    })
    .unwrap();
    let dev = connect(&sim, |_| {}).expect("connect despite loss");
    let mut rx = dev.take_audio_rx().unwrap();

    wait_for("a retransmit request", Duration::from_secs(5), || {
        while rx.pop().is_ok() {}
        sim.served_retransmit()
    });
    assert!(dev.is_alive(), "the session must survive the loss it recovered from");
}

#[test]
fn a_radio_that_is_not_there_fails_quickly_and_says_why() {
    let opts = IcomNetOptions {
        // Reserved for documentation; nothing answers.
        address: "192.0.2.1".into(),
        control_port: 50_001,
        username: "a".into(),
        password: "b".into(),
        timeout: Duration::from_millis(600),
        ..Default::default()
    };
    let started = Instant::now();
    let err = IcomNetDevice::connect(opts).unwrap_err();
    assert!(matches!(err, Error::Timeout(_)), "got {err}");
    assert!(started.elapsed() < Duration::from_secs(5));
}

#[test]
fn shutting_down_stops_the_thread_and_tells_the_radio() {
    let sim = Sim::start(SimOptions::default()).unwrap();
    let dev = connect(&sim, |_| {}).expect("connect");
    assert!(dev.is_alive());
    dev.shutdown();
    assert!(!dev.is_alive());
    // The goodbye must actually reach the radio: a DISCONNECT on the streams
    // and the token handed back, or the radio holds the session and the next
    // client finds ports that never answer.
    wait_for("the goodbye", Duration::from_secs(2), || {
        sim.client_disconnected() && sim.token_returned()
    });
    // Idempotent: the source calls `release()` and then drops.
    dev.shutdown();
}

#[test]
fn dead_data_ports_fail_the_session_with_a_reason_and_still_say_goodbye() {
    // The wedged-radio case from the field: control answers and login succeeds,
    // but the CI-V and audio ports — still held by an earlier session — never
    // answer. The session must not sit half-open behind the healthy control
    // stream: it fails with a reason, and it still cleans up on the way out.
    let sim = Sim::start(SimOptions { mute_data_ports: true, ..Default::default() }).unwrap();
    let dev = connect(&sim, |_| {}).expect("the control-plane handshake still completes");
    wait_for("the data-port watchdog", Duration::from_secs(6), || !dev.is_alive());

    let dump = dev.trace().dump();
    assert!(dump.contains("never answered on it"), "{dump}");
    assert!(dump.contains("session ended"), "{dump}");
    // The goodbye ran despite the error exit — this is what un-wedges the
    // radio for the *next* attempt.
    assert!(dump.contains("goodbye: token returned"), "{dump}");
    wait_for("the goodbye", Duration::from_secs(2), || {
        sim.client_disconnected() && sim.token_returned()
    });
}

#[test]
fn the_audio_stream_is_pinged_so_a_receive_only_session_is_never_silent() {
    // Nothing is ever *sent* on the audio stream of a receive-only session —
    // no TX audio, no idles — so the pings are the only sign of life the radio
    // gets on that port. The reference clients ping all three streams.
    let sim = Sim::start(SimOptions { scope: false, ..Default::default() }).unwrap();
    let _dev = connect(&sim, |_| {}).expect("connect");
    wait_for("an audio-stream ping", Duration::from_secs(3), || sim.audio_pinged());
}

#[test]
fn test_connection_reports_the_radio_in_one_line() {
    let sim = Sim::start(SimOptions::default()).unwrap();
    let report = sdroxide_icomnet::test_connection(IcomNetOptions {
        address: "127.0.0.1".into(),
        control_port: sim.port(),
        username: "operator".into(),
        password: "hunter2".into(),
        timeout: Duration::from_secs(5),
        ..Default::default()
    })
    .expect("a report");
    assert!(report.contains("IC-7300MK2"), "{report}");
    assert!(report.contains("B6h"), "{report}");
    assert!(report.contains("transmit available"), "{report}");
}

#[test]
fn an_unknown_model_says_the_operator_must_set_the_menu_item() {
    // A radio whose `1A 05` numbering is not in the table. It has to be one
    // with no network port — every Icom that can actually reach this backend
    // is now in the table, which is the point of the table.
    let sim = Sim::start(SimOptions {
        civ_address: 0x88,
        radio_name: "IC-7100".into(),
        ..Default::default()
    })
    .unwrap();
    let report = sdroxide_icomnet::test_connection(IcomNetOptions {
        address: "127.0.0.1".into(),
        control_port: sim.port(),
        username: "operator".into(),
        password: "hunter2".into(),
        timeout: Duration::from_secs(5),
        ..Default::default()
    })
    .expect("a report");
    assert!(report.contains("set it on the radio"), "{report}");
}
