//! Threading wrapper around [`CwSkimmer`], mirroring `DigiController`: the
//! realtime engine thread ships IQ blocks to a worker over a bounded channel
//! (dropping on backpressure) and drains emitted spots non-blocking via
//! [`SkimmerController::poll`]. All the heavy DSP runs on the worker thread.
//!
//! Control messages (re-center, settings, stop) ride a separate unbounded
//! channel: they are rare, must never be dropped, and must still arrive while
//! the IQ queue is backed up — which is exactly when the operator is likely to
//! be switching a skimmer off.

use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender, bounded, select, unbounded};
use sdroxide_dsp::Complex32 as C32;
use sdroxide_types::{SkimmerKind, SkimmerSettings, SkimmerSpot};

use crate::cw::CwSkimmer;
use crate::digi::DigiSkimmer;

/// Actions drained from the skimmer each engine tick.
pub enum SkimmerAction {
    Spots(Vec<SkimmerSpot>),
}

/// Realtime data, dropped on backpressure.
struct Iq(Vec<C32>);

/// Control traffic, never dropped.
enum Ctl {
    /// Where the skim window now sits, and where the front end's DC spike falls
    /// inside it. One message rather than two because placing the window
    /// settles both: the spike is at a fixed frequency, so moving the window
    /// moves it within the window.
    Window {
        center_hz: f64,
        dc_off_hz: f64,
    },
    Config(SkimmerSettings),
    View(Option<(f64, f64)>),
    Stop,
}

pub struct SkimmerController {
    iq_tx: Sender<Iq>,
    ctl_tx: Sender<Ctl>,
    res_rx: Receiver<Vec<SkimmerSpot>>,
    worker: Option<JoinHandle<()>>,
}

impl SkimmerController {
    pub fn new(skim_rate: f64, skim_center_hz: f64, dc_off_hz: f64, cfg: SkimmerSettings) -> Self {
        let (iq_tx, iq_rx) = bounded::<Iq>(64);
        let (ctl_tx, ctl_rx) = unbounded::<Ctl>();
        let (res_tx, res_rx) = unbounded::<Vec<SkimmerSpot>>();
        // Emit a fresh spot snapshot roughly 4×/second.
        let emit_every = (skim_rate * 0.25) as usize;
        let worker = std::thread::Builder::new()
            .name("sdroxide-skimmer".into())
            .spawn(move || {
                // One shared worker runs the CW skimmer plus the continuous
                // PSK/RTTY skimmers; each gates its own spots to its band
                // segments, so their spot sets never overlap. A skimmer the
                // operator switched off is skipped entirely — no DSP, no spots.
                let mut cfg = cfg;
                let mut cw = CwSkimmer::with_config(skim_rate, skim_center_hz, &cfg);
                let mut psk = DigiSkimmer::new(SkimmerKind::Psk, skim_rate, skim_center_hz);
                let mut rtty = DigiSkimmer::new(SkimmerKind::Rtty, skim_rate, skim_center_hz);
                cw.set_dc_offset_hz(dc_off_hz);
                psk.set_dc_offset_hz(dc_off_hz);
                rtty.set_dc_offset_hz(dc_off_hz);
                let mut since = 0usize;
                loop {
                    select! {
                        recv(ctl_rx) -> msg => match msg {
                            Ok(Ctl::Window { center_hz, dc_off_hz }) => {
                                cw.set_center(center_hz);
                                psk.set_center(center_hz);
                                rtty.set_center(center_hz);
                                cw.set_dc_offset_hz(dc_off_hz);
                                psk.set_dc_offset_hz(dc_off_hz);
                                rtty.set_dc_offset_hz(dc_off_hz);
                            }
                            Ok(Ctl::Config(next)) => {
                                // The CW skimmer has settings of its own beyond
                                // whether it runs — which decoder reads its
                                // signals, and how many at once.
                                cw.set_config(&next);
                                // Drop the tracks of a skimmer being switched
                                // off, so switching it back on starts from a
                                // clean band instead of replaying a stale one.
                                if !next.enabled(SkimmerKind::Cw) {
                                    cw.reset();
                                }
                                if !next.enabled(SkimmerKind::Psk) {
                                    psk.reset();
                                }
                                if !next.enabled(SkimmerKind::Rtty) {
                                    rtty.reset();
                                }
                                cfg = next;
                            }
                            Ok(Ctl::View(v)) => {
                                cw.set_view(v);
                                psk.set_view(v);
                                rtty.set_view(v);
                            }
                            Ok(Ctl::Stop) | Err(_) => break,
                        },
                        recv(iq_rx) -> msg => match msg {
                            Ok(Iq(iq)) => {
                                since += iq.len();
                                if cfg.enabled(SkimmerKind::Cw) {
                                    cw.process(&iq);
                                }
                                if cfg.enabled(SkimmerKind::Psk) {
                                    psk.process(&iq);
                                }
                                if cfg.enabled(SkimmerKind::Rtty) {
                                    rtty.process(&iq);
                                }
                                if since >= emit_every {
                                    since = 0;
                                    let mut spots = Vec::new();
                                    if cfg.enabled(SkimmerKind::Cw) {
                                        spots.extend(cw.spots());
                                    }
                                    if cfg.enabled(SkimmerKind::Psk) {
                                        spots.extend(psk.spots());
                                    }
                                    if cfg.enabled(SkimmerKind::Rtty) {
                                        spots.extend(rtty.spots());
                                    }
                                    // Squelch: a track has to stand this far
                                    // above its skimmer's noise floor to be
                                    // worth a box on the waterfall.
                                    spots.retain(|s| s.snr_db >= cfg.squelch_db(s.kind));
                                    let _ = res_tx.send(spots);
                                }
                            }
                            Err(_) => break,
                        },
                    }
                }
            })
            .expect("spawn skimmer worker");
        SkimmerController { iq_tx, ctl_tx, res_rx, worker: Some(worker) }
    }

    /// Realtime path: hand a block of skim-rate IQ to the worker. Non-blocking;
    /// drops the block if the worker is behind (backpressure).
    pub fn on_rx_iq(&self, iq: &[C32]) {
        let _ = self.iq_tx.try_send(Iq(iq.to_vec()));
    }

    /// Re-place the skim window: its absolute centre, and where the front end's
    /// DC spike now falls inside it. A centre that actually moved clears the
    /// tracks — they were measured against the old axis.
    pub fn set_window(&self, center_hz: f64, dc_off_hz: f64) {
        let _ = self.ctl_tx.send(Ctl::Window { center_hz, dc_off_hz });
    }

    /// Apply new per-kind enables / squelches to the running worker.
    pub fn set_config(&self, cfg: SkimmerSettings) {
        let _ = self.ctl_tx.send(Ctl::Config(cfg));
    }

    /// Narrow every skimmer to the operator's visible waterfall window, in
    /// absolute Hz. `None` skims the whole window. Rides the control channel
    /// because a pan must not be dropped behind a backed-up IQ queue: the
    /// skimmers would carry on decoding a stretch of band nobody is looking at.
    pub fn set_view(&self, view: Option<(f64, f64)>) {
        let _ = self.ctl_tx.send(Ctl::View(view));
    }

    /// Drain any spot snapshots produced since the last poll. Non-blocking.
    pub fn poll(&self) -> Vec<SkimmerAction> {
        let mut out = Vec::new();
        while let Ok(spots) = self.res_rx.try_recv() {
            out.push(SkimmerAction::Spots(spots));
        }
        out
    }
}

impl Drop for SkimmerController {
    fn drop(&mut self) {
        let _ = self.ctl_tx.send(Ctl::Stop);
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
    }
}
