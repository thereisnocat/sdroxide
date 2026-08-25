//! Deciding what to tell the board, and how often.
//!
//! Two halves. [`Follower`] is pure and clock-injected: it holds what the board
//! was last told and answers "is there anything to say right now?". [`spawn`]
//! puts it on a thread with a transport, because both links block — the serial
//! one for tens of milliseconds and the board/I²C one for the better part of a
//! second, and neither belongs on the engine's loop.
//!
//! The discipline is the HL2IOBoard's, next door in `sdroxide-hpsdr`:
//!
//! * **Deduplicate on the resolved state, never on the frequency.** A dial
//!   dragged across 14.100–14.350 resolves to the same channel and puts nothing
//!   on the control bus at all; a jump to 145 MHz always does.
//! * **Rate-limit the sending, not the value.** Whatever the state is when the
//!   interval expires is what goes out, so a band change during a fast tune is
//!   never the one that gets dropped.
//! * **A half-sent state is not a state.** A failed transaction forgets what
//!   was sent entirely and starts again from the top.
//! * **Give up rather than retry forever.** A board that has stopped answering
//!   is left alone for the rest of the session and said so, once.

use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TryRecvError};
use sdroxide_types::{LimeRfeConfig, RfeLink, RfeMode};

use crate::error::Result;
use crate::frame::RfeState;
use crate::trace::{self, Trace};
use crate::transport::RfeTransport;

/// The floor on how often anything is sent, whatever the transport says. The
/// HL2IOBoard's `FREQ_INTERVAL`, and for the same reason: a band change is not
/// a thing that happens twice a second.
const MIN_INTERVAL: Duration = Duration::from_millis(500);

/// Consecutive failures before the board is declared gone.
const MAX_TRIES: u32 = 3;

/// How long the board's thread sleeps between looks at its channel. Short
/// enough that a key-down is acted on promptly, long enough that an idle board
/// costs nothing — and it sleeps *on* the channel, so a key-down wakes it
/// rather than waiting this out. The rate limit, not this, is what bounds how
/// often anything is actually sent.
const TICK: Duration = Duration::from_millis(20);

/// Whether the board is there. Not configuration: it either answers or it does
/// not, and an operator should not have to tell us which.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Presence {
    Present,
    /// Stopped answering, and left alone for the rest of the session.
    Absent,
}

/// What the board should be doing, given the dial and the configuration.
///
/// Pure — no transport, no clock of its own. Everything that decides *what* to
/// send lives here so it can be tested with nothing plugged in.
#[derive(Debug)]
pub struct Follower {
    cfg: LimeRfeConfig,
    /// The last state the board acknowledged. `None` means "unknown", which is
    /// both the starting state and what a failed transaction resets it to.
    acked: Option<RfeState>,
    /// When the last transaction went out, for the rate limit.
    last_sent: Option<Instant>,
    interval: Duration,
    consecutive_failures: u32,
    presence: Presence,
    /// Latest receive and transmit frequencies, as the source last reported.
    ///
    /// `None` until it has. Nothing is sent while the receive one is unknown,
    /// because zero resolves to the *HF* channel: the opening transaction would
    /// otherwise put the 30 MHz filter in circuit on a station tuned nowhere
    /// near it, and on a shared connector move the relays to do it.
    rx_hz: Option<f64>,
    tx_hz: Option<f64>,
    /// Whether the operator is keyed. Always matters now: a receiving board is
    /// left in receive whatever the cabling — see [`sdroxide_types::RfeModeControl::Auto`].
    keyed: bool,
}

impl Follower {
    pub fn new(cfg: LimeRfeConfig, round_trip: Duration) -> Follower {
        Follower {
            cfg,
            acked: None,
            last_sent: None,
            // Four round trips, so a slow link is never asked to run flat out
            // and never has more than a quarter of its time taken by us.
            interval: MIN_INTERVAL.max(round_trip * 4),
            consecutive_failures: 0,
            presence: Presence::Present,
            rx_hz: None,
            tx_hz: None,
            keyed: false,
        }
    }

    pub fn presence(&self) -> Presence {
        self.presence
    }

    pub fn interval(&self) -> Duration {
        self.interval
    }

    pub fn set_config(&mut self, cfg: LimeRfeConfig) {
        if cfg != self.cfg {
            self.cfg = cfg;
            // Not a reset of `acked`: the resolved state may well be identical,
            // and `want()` comparing them is what decides. Changing the notch
            // and changing the band cost the same one transaction either way.
        }
    }

    pub fn set_rx_hz(&mut self, hz: f64) {
        self.rx_hz = Some(hz);
    }

    pub fn set_tx_hz(&mut self, hz: f64) {
        self.tx_hz = Some(hz);
    }

    pub fn set_keyed(&mut self, keyed: bool) {
        self.keyed = keyed;
    }

    /// The state the board should be in right now.
    ///
    /// Transmit is resolved from the transmit frequency and receive from the
    /// receive one, so a cross-band or split contact puts the right filter in
    /// each path rather than one band's filter in both.
    pub fn want(&self) -> RfeState {
        let rx_hz = self.rx_hz.unwrap_or(0.0);
        // A transceiver whose transmit frequency has not been reported yet is
        // on the receive one, not on zero — which is HF, and would put the
        // wideband path in the transmit chain for a station on 2 m.
        let tx_hz = self.tx_hz.or(self.rx_hz).unwrap_or(0.0);
        let (channel_rx, channel_tx) = sdroxide_types::rfe_resolve(&self.cfg, rx_hz, tx_hz);
        let mode = if self.keyed {
            // `tx_mode` is None only where the operator has pinned the board:
            // held in transmit there is nothing to switch to, and held in
            // receive the source has already refused the over.
            self.cfg.tx_mode().unwrap_or_else(|| self.cfg.rx_mode())
        } else {
            self.cfg.rx_mode()
        };
        RfeState {
            channel_rx,
            channel_tx,
            port_rx: self.cfg.port_rx,
            port_tx: self.cfg.port_tx,
            mode,
            notch: self.cfg.notch,
            atten_steps: self.cfg.atten_steps,
            // The SWR subsystem is left off: sdroxide does not read the board's
            // ADCs, and enabling a detector nothing looks at only adds a way
            // for the configuration to be refused.
            swr_enable: false,
            swr_source_cell: false,
        }
    }

    /// What to send now, if anything.
    ///
    /// `None` covers every quiet case: nothing changed, the rate limit has not
    /// expired, no board is configured, or it has been given up on.
    pub fn due(&self, now: Instant) -> Option<Action> {
        if self.cfg.link == RfeLink::Off || self.presence == Presence::Absent {
            return None;
        }
        // Hold everything until the dial is known, where the dial is what
        // decides. See `rx_hz`. With the band pinned by hand there is nothing
        // to wait for and the opening state goes out at once as before.
        if self.cfg.follow_band && self.rx_hz.is_none() {
            return None;
        }
        let want = self.want();
        let Some(acked) = self.acked else {
            // Nothing acknowledged yet — the opening configuration goes out
            // immediately rather than waiting out an interval nothing has used.
            return Some(Action::Configure(want));
        };
        if want == acked {
            return None;
        }
        // Anything involving the relays is the key-down path and never waits:
        // an over that had to sit out a rate limit would transmit into a closed
        // relay. The rate limit exists to keep a dragged dial off the control
        // bus, not to delay a transmitter.
        if want.mode != acked.mode {
            // Only the relays moved: the cheap one-byte command will do.
            if (RfeState { mode: acked.mode, ..want }) == acked {
                return Some(Action::Mode(want.mode));
            }
            // The band moved too — keying straight after a band change. Send
            // the whole state rather than the mode alone, because transmitting
            // late is better than transmitting through the previous band's
            // filter, and a `Configure` carries both.
            return Some(Action::Configure(want));
        }
        if let Some(last) = self.last_sent
            && now.duration_since(last) < self.interval
        {
            return None;
        }
        Some(Action::Configure(want))
    }

    /// Record that a transaction succeeded.
    pub fn on_ack(&mut self, action: Action, now: Instant) {
        self.consecutive_failures = 0;
        self.last_sent = Some(now);
        match action {
            Action::Configure(st) => self.acked = Some(st),
            // A mode change moves one field of what the board holds. Recording
            // the whole `want()` instead would claim credit for a band change
            // that never went out.
            Action::Mode(mode) => {
                if let Some(acked) = self.acked.as_mut() {
                    acked.mode = mode;
                }
            }
        }
    }

    /// Record that a transaction failed.
    ///
    /// Forgets what was acknowledged entirely: a board that refused or missed
    /// one request is not in a state we know, and re-sending the whole
    /// configuration is cheaper than reasoning about which half arrived.
    pub fn on_error(&mut self, now: Instant) {
        self.acked = None;
        self.last_sent = Some(now);
        self.consecutive_failures += 1;
        if self.consecutive_failures >= MAX_TRIES {
            self.presence = Presence::Absent;
        }
    }
}

/// One transaction the follower wants performed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    Configure(RfeState),
    Mode(RfeMode),
}

/// One line for a report: what this transaction asked the board for.
///
/// Deliberately the same words the log uses. Somebody comparing a pasted
/// report against a pasted log should not have to work out that they describe
/// the same thing.
fn describe_action(action: Action) -> String {
    match action {
        Action::Configure(st) => format!(
            "{} in on {}, {} out on {}, relays {}{}{}",
            st.channel_rx.label(),
            st.port_rx.label(),
            st.channel_tx.label(),
            st.port_tx.label(),
            st.mode.label(),
            if st.notch { ", notch in" } else { "" },
            match st.atten_steps {
                0 => String::new(),
                n => format!(", {} dB of attenuation", u16::from(n) * 2),
            }
        ),
        Action::Mode(m) => format!("relays {}", m.label()),
    }
}

/// Messages into the board's thread. Every one is last-value-wins.
#[derive(Debug, Clone)]
pub enum Ctrl {
    Config(Box<LimeRfeConfig>),
    RxFreq(f64),
    TxFreq(f64),
    Keyed(bool),
    Fan(bool),
    Shutdown,
}

/// A handle on the thread driving one LimeRFE.
pub struct LimeRfeHandle {
    tx: Sender<Ctrl>,
    join: Option<std::thread::JoinHandle<()>>,
    describe: String,
    status: std::sync::Arc<std::sync::Mutex<Option<String>>>,
    relay_settle: Duration,
}

impl LimeRfeHandle {
    pub fn set_config(&self, cfg: LimeRfeConfig) {
        let _ = self.tx.send(Ctrl::Config(Box::new(cfg)));
    }
    pub fn set_rx_hz(&self, hz: f64) {
        let _ = self.tx.send(Ctrl::RxFreq(hz));
    }
    pub fn set_tx_hz(&self, hz: f64) {
        let _ = self.tx.send(Ctrl::TxFreq(hz));
    }
    pub fn set_keyed(&self, keyed: bool) {
        let _ = self.tx.send(Ctrl::Keyed(keyed));
    }
    pub fn set_fan(&self, on: bool) {
        let _ = self.tx.send(Ctrl::Fan(on));
    }
    pub fn describe(&self) -> &str {
        &self.describe
    }
    /// A standing condition worth showing the operator — a board that stopped
    /// answering, or a refusal it keeps giving. `None` when all is well.
    pub fn status(&self) -> Option<String> {
        self.status.lock().ok().and_then(|s| s.clone())
    }

    /// How long to allow for a relay to have thrown after
    /// [`Self::set_keyed`] — the thread's own wake-up plus one transaction on
    /// this transport.
    ///
    /// Asked rather than assumed because the two links are two orders of
    /// magnitude apart: the board's own serial port answers in tens of
    /// milliseconds, and the bit-banged I²C path through the LimeSDR's GPIO
    /// header in the better part of a second. A caller that guessed the first
    /// number would let drive out into a receive path on the second.
    pub fn relay_settle(&self) -> Duration {
        self.relay_settle
    }
}

impl Drop for LimeRfeHandle {
    fn drop(&mut self) {
        let _ = self.tx.send(Ctrl::Shutdown);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

/// Put a transport on its own thread and drive it from the follower.
pub fn spawn(mut transport: Box<dyn RfeTransport>, cfg: LimeRfeConfig) -> LimeRfeHandle {
    let (tx, rx) = crossbeam_channel::unbounded::<Ctrl>();
    let describe = transport.describe();
    let relay_settle = transport.round_trip() + TICK;
    let status = std::sync::Arc::new(std::sync::Mutex::new(None));
    let status_thread = std::sync::Arc::clone(&status);
    // What this board was told, kept for a report. A front end that answers
    // every command and passes no signal is diagnosed from exactly this, and
    // on the serial link there is no other record of it anywhere.
    let t = Trace::new();
    t.set_link(&describe); // the transport's own description of the link
    trace::remember(&t);

    let join = std::thread::Builder::new()
        .name("sdroxide-limerfe".into())
        .spawn(move || {
            let mut follower = Follower::new(cfg, transport.round_trip());
            run(&mut *transport, &mut follower, &rx, &status_thread, &t);
            // Leave the board receiving rather than keyed, whatever happened
            // above — the same "shutdown is best-effort but unconditional"
            // rule the USB drivers apply to their radios.
            let stood_down = transport.set_mode(RfeMode::Rx);
            t.note(
                "shutdown: relays back to Receive",
                match &stood_down {
                    Ok(()) => "ok".to_string(),
                    Err(e) => format!("FAILED: {e}"),
                },
            );
        })
        .expect("spawn sdroxide-limerfe thread");

    LimeRfeHandle { tx, join: Some(join), describe, status, relay_settle }
}

fn run(
    transport: &mut dyn RfeTransport,
    follower: &mut Follower,
    rx: &Receiver<Ctrl>,
    status: &std::sync::Mutex<Option<String>>,
    trace: &Trace,
) {
    let tick = TICK;
    loop {
        // Drain the whole channel before acting: a band change and a key-down
        // that arrived together should produce one decision, not two.
        loop {
            match rx.try_recv() {
                Ok(Ctrl::Config(c)) => follower.set_config(*c),
                Ok(Ctrl::RxFreq(hz)) => follower.set_rx_hz(hz),
                Ok(Ctrl::TxFreq(hz)) => follower.set_tx_hz(hz),
                Ok(Ctrl::Keyed(k)) => {
                    follower.set_keyed(k);
                    trace.note(if k { "keyed" } else { "unkeyed" }, "");
                }
                Ok(Ctrl::Fan(on)) => {
                    if let Err(e) = transport.set_fan(on) {
                        tracing::warn!("LimeRFE fan: {e}");
                    }
                }
                Ok(Ctrl::Shutdown) | Err(TryRecvError::Disconnected) => return,
                Err(TryRecvError::Empty) => break,
            }
        }

        let now = Instant::now();
        if let Some(action) = follower.due(now) {
            let result: Result<()> = match action {
                Action::Configure(st) => transport.configure(st),
                Action::Mode(m) => transport.set_mode(m),
            };
            match result {
                Ok(()) => {
                    follower.on_ack(action, Instant::now());
                    if let Ok(mut s) = status.lock() {
                        *s = None;
                    }
                    // Loud on purpose, and affordable because the whole design
                    // is about not saying much: a band change, a key-down, and
                    // nothing in between. A front end that answers every
                    // command and passes no signal is diagnosed from exactly
                    // this line — what it was told, and that it agreed.
                    match action {
                        Action::Configure(st) => tracing::info!(
                            "LimeRFE set to {} in, {} out, receiving on {}, transmitting on \
                             {}, relays {}{}{}",
                            st.channel_rx.label(),
                            st.channel_tx.label(),
                            st.port_rx.label(),
                            st.port_tx.label(),
                            st.mode.label(),
                            if st.notch { ", notch in" } else { "" },
                            match st.atten_steps {
                                0 => String::new(),
                                n => format!(", {} dB of attenuation", u16::from(n) * 2),
                            }
                        ),
                        Action::Mode(m) => tracing::info!("LimeRFE relays: {}", m.label()),
                    }
                    trace.note(describe_action(action), "ok");
                }
                Err(e) => {
                    follower.on_error(Instant::now());
                    trace.note(describe_action(action), format!("FAILED: {e}"));
                    let gone = follower.presence() == Presence::Absent;
                    if let Ok(mut s) = status.lock() {
                        *s = Some(if gone {
                            format!("the LimeRFE stopped answering and has been left alone: {e}")
                        } else {
                            e.to_string()
                        });
                    }
                    if gone {
                        // Said once, not once per tick.
                        tracing::warn!("LimeRFE gave up after {MAX_TRIES} failures: {e}");
                    } else {
                        tracing::debug!("LimeRFE transaction failed, will retry: {e}");
                    }
                }
            }
        }

        // Sleep on the channel so a key-down wakes the thread rather than
        // waiting out the tick.
        match rx.recv_timeout(tick) {
            Ok(Ctrl::Shutdown) | Err(crossbeam_channel::RecvTimeoutError::Disconnected) => return,
            Ok(msg) => {
                // Put it back at the head of the next drain by handling it now.
                match msg {
                    Ctrl::Config(c) => follower.set_config(*c),
                    Ctrl::RxFreq(hz) => follower.set_rx_hz(hz),
                    Ctrl::TxFreq(hz) => follower.set_tx_hz(hz),
                    Ctrl::Keyed(k) => {
                        follower.set_keyed(k);
                        trace.note(if k { "keyed" } else { "unkeyed" }, "");
                    }
                    Ctrl::Fan(on) => {
                        if let Err(e) = transport.set_fan(on) {
                            tracing::warn!("LimeRFE fan: {e}");
                        }
                    }
                    Ctrl::Shutdown => return,
                }
            }
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::{RfeChannel, RfeModeControl, RfePort};

    fn split_cabling() -> LimeRfeConfig {
        LimeRfeConfig {
            link: RfeLink::Serial,
            port_rx: RfePort::J3,
            port_tx: RfePort::J4,
            ..Default::default()
        }
    }

    fn shared_cabling() -> LimeRfeConfig {
        LimeRfeConfig {
            link: RfeLink::Serial,
            port_rx: RfePort::J3,
            port_tx: RfePort::J3,
            ..Default::default()
        }
    }

    /// The opening configuration goes out at once rather than waiting out an
    /// interval nothing has used yet.
    #[test]
    fn the_first_state_is_sent_immediately() {
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        f.set_rx_hz(145.5e6);
        assert!(matches!(f.due(Instant::now()), Some(Action::Configure(_))));
    }

    /// ...but not before there is a dial to send it for.
    ///
    /// An unreported receive frequency is zero, and zero resolves to the *HF*
    /// channel — so a board configured before the source has said where it is
    /// gets the 30 MHz filter put in circuit under a station on 2 m, and on a
    /// shared connector has its relays moved to do it.
    #[test]
    fn nothing_is_sent_until_the_dial_has_been_reported() {
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        let now = Instant::now();
        assert_eq!(f.due(now), None, "no dial, nothing to say");
        // Even keying does not force a state out of a follower that does not
        // know where the radio is.
        f.set_keyed(true);
        assert_eq!(f.due(now + Duration::from_secs(5)), None);

        f.set_rx_hz(145.5e6);
        let Some(Action::Configure(st)) = f.due(now + Duration::from_secs(5)) else {
            panic!("and once it knows, it speaks")
        };
        assert_eq!(st.channel_rx, RfeChannel::Ham0145);
        // The transmit path follows the receive dial rather than falling to
        // zero and taking the HF rule with it.
        assert_eq!(st.channel_tx, RfeChannel::Ham0145);
    }

    /// A hand-pinned band has nothing to wait for, so the hold above must not
    /// apply to it — otherwise a board with *Follow the dial* off would sit
    /// unconfigured on a receive-only station that never reports a frequency.
    #[test]
    fn a_pinned_band_is_sent_without_waiting_for_a_dial() {
        let f = Follower::new(
            LimeRfeConfig { follow_band: false, channel: RfeChannel::Ham0435, ..split_cabling() },
            Duration::from_millis(45),
        );
        let Some(Action::Configure(st)) = f.due(Instant::now()) else {
            panic!("a pinned band needs no dial")
        };
        assert_eq!(st.channel_rx, RfeChannel::Ham0435);
    }

    /// The whole point of the design: a dial dragged inside one band puts
    /// nothing on the control bus.
    #[test]
    fn tuning_within_a_band_says_nothing_but_changing_band_says_something() {
        let now = Instant::now();
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        f.set_rx_hz(144.100e6);
        f.set_tx_hz(144.100e6);
        let Some(first) = f.due(now) else { panic!("the opening state") };
        f.on_ack(first, now);

        // Two hundred kilohertz later, and a long time later, still nothing.
        let later = now + Duration::from_secs(5);
        f.set_rx_hz(144.300e6);
        f.set_tx_hz(144.300e6);
        assert_eq!(f.due(later), None, "same channel, nothing to say");

        // A band away, and it speaks.
        f.set_rx_hz(432.100e6);
        f.set_tx_hz(432.100e6);
        assert!(matches!(f.due(later), Some(Action::Configure(_))), "a band change always goes");
    }

    /// A band change inside the rate-limit window is *held*, not dropped —
    /// whatever the state is when the window opens is what goes out.
    #[test]
    fn a_change_inside_the_rate_limit_is_held_and_then_sent() {
        let now = Instant::now();
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        f.set_rx_hz(144.1e6);
        f.set_tx_hz(144.1e6);
        let first = f.due(now).unwrap();
        f.on_ack(first, now);

        f.set_rx_hz(432.1e6);
        f.set_tx_hz(432.1e6);
        assert_eq!(f.due(now + Duration::from_millis(100)), None, "too soon");

        // Another band change arrives while we were waiting. The one that goes
        // out is the latest, not the one that was pending first.
        f.set_rx_hz(1296.0e6);
        f.set_tx_hz(1296.0e6);
        let Some(Action::Configure(st)) = f.due(now + f.interval() + Duration::from_millis(1))
        else {
            panic!("the window opened")
        };
        assert_eq!(st.channel_rx, RfeChannel::Ham1280, "the latest band, not the stale one");
    }

    /// Key-down must not wait out a rate limit — a mode change is the one
    /// transaction that is late if it is late at all.
    #[test]
    fn a_key_down_is_never_rate_limited() {
        let now = Instant::now();
        let mut f = Follower::new(shared_cabling(), Duration::from_millis(45));
        f.set_rx_hz(14.2e6);
        f.set_tx_hz(14.2e6);
        let first = f.due(now).unwrap();
        f.on_ack(first, now);

        f.set_keyed(true);
        // Immediately after a transaction, well inside the interval.
        let Some(Action::Mode(mode)) = f.due(now + Duration::from_millis(1)) else {
            panic!("key-down must go out at once")
        };
        assert_eq!(mode, RfeMode::Tx);
    }

    /// Split connectors switch the relays around an over exactly as a shared
    /// one does, and the standing state is *receive* (issue #94). Leaving the
    /// board in both-on because the connectors happen to be split saved a
    /// round trip at key-down and cost the receiver: the amateur channels have
    /// one filter with a transmit/receive switch either side, so both-on puts
    /// that switch on the transmitter.
    #[test]
    fn keying_on_split_connectors_still_moves_the_relays() {
        let now = Instant::now();
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        f.set_rx_hz(144.2e6);
        f.set_tx_hz(144.2e6);
        let first = f.due(now).unwrap();
        f.on_ack(first, now);
        assert!(matches!(first, Action::Configure(st) if st.mode == RfeMode::Rx));

        f.set_keyed(true);
        let Some(Action::Mode(m)) = f.due(now + Duration::from_millis(1)) else {
            panic!("key-down switches to transmit")
        };
        assert_eq!(m, RfeMode::Tx);
        f.on_ack(Action::Mode(m), now + Duration::from_millis(1));

        f.set_keyed(false);
        let Some(Action::Mode(m)) = f.due(now + Duration::from_millis(2)) else {
            panic!("and key-up switches back")
        };
        assert_eq!(m, RfeMode::Rx);
    }

    /// A failed transaction forgets everything and re-sends the whole state.
    #[test]
    fn a_failed_transaction_is_resent_from_the_top() {
        let now = Instant::now();
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        f.set_rx_hz(144.2e6);
        f.set_tx_hz(144.2e6);
        let first = f.due(now).unwrap();
        f.on_ack(first, now);
        assert_eq!(f.due(now + Duration::from_secs(5)), None, "settled");

        f.on_error(now + Duration::from_secs(5));
        assert!(
            matches!(f.due(now + Duration::from_secs(6)), Some(Action::Configure(_))),
            "a half-sent state is not a state"
        );
    }

    /// Three failures and the board is left alone — not retried forever on a
    /// bus that may have something else on it.
    #[test]
    fn a_board_that_stops_answering_is_given_up_on() {
        let now = Instant::now();
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        assert_eq!(f.presence(), Presence::Present);
        for i in 0..MAX_TRIES {
            f.on_error(now + Duration::from_secs(i as u64));
        }
        assert_eq!(f.presence(), Presence::Absent);
        assert_eq!(f.due(now + Duration::from_secs(60)), None, "and stays quiet");
    }

    /// No board configured means nothing is ever sent, however the dial moves.
    #[test]
    fn an_unconfigured_board_is_never_spoken_to() {
        let mut f = Follower::new(LimeRfeConfig::default(), Duration::from_millis(45));
        assert_eq!(f.cfg.link, RfeLink::Off);
        f.set_rx_hz(144.2e6);
        f.set_tx_hz(432.1e6);
        f.set_keyed(true);
        assert_eq!(f.due(Instant::now()), None);
    }

    /// A mode acknowledgement moves one field, and must not claim credit for a
    /// band change that never went out.
    #[test]
    fn acking_a_mode_does_not_mark_a_pending_band_change_as_done() {
        let now = Instant::now();
        let mut f = Follower::new(shared_cabling(), Duration::from_millis(45));
        f.set_rx_hz(14.2e6);
        f.set_tx_hz(14.2e6);
        let first = f.due(now).unwrap();
        f.on_ack(first, now);

        // Key down with the band unchanged: the cheap one-byte mode command.
        f.set_keyed(true);
        let Some(Action::Mode(m)) = f.due(now + Duration::from_millis(1)) else {
            panic!("a mode-only change uses the mode command")
        };
        f.on_ack(Action::Mode(m), now + Duration::from_millis(1));
        assert_eq!(f.due(now + Duration::from_millis(2)), None, "and nothing else is owed");

        // Now the band moves while still keyed. That is an ordinary band
        // change and waits for the window like any other.
        f.set_rx_hz(144.2e6);
        f.set_tx_hz(144.2e6);
        assert_eq!(f.due(now + Duration::from_millis(3)), None, "rate limited");
        let Some(Action::Configure(st)) = f.due(now + f.interval() + Duration::from_millis(2))
        else {
            panic!("the band change is still pending")
        };
        assert_eq!(st.channel_rx, RfeChannel::Ham0145);
        assert_eq!(st.mode, RfeMode::Tx, "and carries the mode it is already in");
    }

    /// Keying immediately after a band change must not wait out the rate
    /// limit — but must not transmit through the previous band's filter
    /// either, so the whole state goes rather than the mode alone.
    #[test]
    fn keying_straight_after_a_band_change_sends_both_at_once() {
        let now = Instant::now();
        let mut f = Follower::new(shared_cabling(), Duration::from_millis(45));
        f.set_rx_hz(14.2e6);
        f.set_tx_hz(14.2e6);
        let first = f.due(now).unwrap();
        f.on_ack(first, now);

        f.set_rx_hz(144.2e6);
        f.set_tx_hz(144.2e6);
        f.set_keyed(true);
        // Well inside the rate-limit window.
        let Some(Action::Configure(st)) = f.due(now + Duration::from_millis(1)) else {
            panic!("a key-down is never rate limited, whatever else moved")
        };
        assert_eq!(st.mode, RfeMode::Tx);
        assert_eq!(st.channel_tx, RfeChannel::Ham0145, "on the new band's filter, not the old");
    }

    /// Cross-band operation puts a different filter in each path.
    #[test]
    fn each_direction_gets_its_own_filter() {
        let mut f = Follower::new(split_cabling(), Duration::from_millis(45));
        f.set_rx_hz(435.0e6);
        f.set_tx_hz(145.0e6);
        let st = f.want();
        assert_eq!(st.channel_rx, RfeChannel::Ham0435);
        assert_eq!(st.channel_tx, RfeChannel::Ham0145);
    }

    /// A board pinned to receive never resolves to a transmitting mode, keyed
    /// or not. The source refuses the key-down; this is the second line.
    #[test]
    fn a_pinned_receive_board_never_resolves_to_transmit() {
        let mut f = Follower::new(
            LimeRfeConfig { mode: RfeModeControl::Rx, ..shared_cabling() },
            Duration::from_millis(45),
        );
        f.set_keyed(true);
        assert_eq!(f.want().mode, RfeMode::Rx);
    }

    /// A slow transport gets a proportionally slower rate limit, and a fast one
    /// still never goes below the floor.
    #[test]
    fn the_rate_limit_follows_the_transport() {
        let fast = Follower::new(split_cabling(), Duration::from_millis(45));
        assert_eq!(fast.interval(), MIN_INTERVAL, "the floor holds");

        let slow = Follower::new(split_cabling(), Duration::from_millis(700));
        assert_eq!(slow.interval(), Duration::from_millis(2800));
    }
}
