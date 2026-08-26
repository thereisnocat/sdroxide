//! Loading the samplers' down-converter, which is what makes them stream at all.
//!
//! # The bug this module exists for
//!
//! An FDM-S1 or FDM-S2 is a bus-powered front end, and the two halves of it come
//! up very differently. The Cypress bridge's firmware lives in an EEPROM, so the
//! moment the device is plugged in it enumerates, answers every vendor request,
//! reports its serial and its hardware version out of that EEPROM, and
//! acknowledges the FIFO start with the `0xE9` it is documented to answer with.
//! The FPGA behind it is SRAM-configured and comes up **empty**. There is no
//! down-converter in there to start, so the bulk endpoint simply never produces
//! a byte — no error, no stall, no short transfer, nothing to see anywhere
//! ([issue #178]).
//!
//! ELAD ship the image loader as a separate program, `elad-firmware`, in their
//! Linux download area, and it has to be run after every power-up. Their own
//! GNU Radio module does not run it — it assumes something else already has,
//! which is why nothing in `gr-elad` hints that any of this is needed, and why
//! a driver written from `gr-elad` inherits the gap.
//!
//! # The sample rate is which image is loaded
//!
//! This is also the answer to the other ELAD puzzle. The six rates are not a
//! register: each is a *different FPGA image*, selected by the speed code given
//! to the loader ([`speed_code`]). That is why no request in the vendor
//! protocol sets the rate, why `gr-elad` takes the rate as a parameter and only
//! ever scales by it, and why ELAD's own software appears to "remember" a rate
//! across programs. So on a sampler `EladConfig::sample_rate_hz` really is a
//! command, and this module is where it is issued.
//!
//! An FDM-DUO is a radio with its own controller and boots its own FPGA; none
//! of this applies to it.
//!
//! # Running somebody else's binary
//!
//! Deliberate, and deliberately narrow. The images are ELAD's and cannot be
//! redistributed, so their loader is the only way to put one in the device, and
//! a receiver that cannot receive is not much of a backend. Nothing happens
//! unless the operator has installed that loader themselves: the paths searched
//! are the one ELAD's instructions name, the one a package manager would use,
//! and whatever [`LOADER_ENV`] points at. When it is not there, the sentence
//! that goes on screen says exactly what to install and where — which is worth
//! more than the load itself, because it turns an invisible silent receiver
//! into a five-minute fix.
//!
//! [issue #178]: https://github.com/dividebysandwich/sdroxide/issues/178

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use crate::protocol::Model;
use crate::trace::Trace;

/// The name ELAD distribute their FPGA loader under.
pub const LOADER: &str = "elad-firmware";

/// An environment variable naming the loader, for an installation that put it
/// somewhere else — or for a build of it under another name.
pub const LOADER_ENV: &str = "SDROXIDE_ELAD_FIRMWARE";

/// Where the loader is looked for, in order.
///
/// The first is the path ELAD's own Linux instructions tell an operator to copy
/// it to and the one every third-party recipe repeats; the rest are where a
/// packager might reasonably put it instead.
const CANDIDATES: &[&str] =
    &["/usr/local/bin/elad-firmware", "/usr/bin/elad-firmware", "/opt/elad/elad-firmware"];

/// How long the loader may take before it is given up on and killed.
///
/// Programming takes about six seconds. A minute is not a guess at that, it is
/// a guess at the worst case for a program that is talking to hardware over
/// USB, and the only thing it protects against is a loader that has hung and
/// would otherwise hold the stream thread for ever.
const LOADER_TIMEOUT: Duration = Duration::from_secs(60);

/// How long the device is given to come back after being programmed, and how
/// often the claim is retried in the meantime.
///
/// It re-enumerates: ELAD's loader leaves it looking like a freshly plugged-in
/// device, so the handle we had is stale and the new one may not exist yet.
const RECLAIM_WINDOW: Duration = Duration::from_secs(10);
const RECLAIM_INTERVAL: Duration = Duration::from_millis(250);

/// The speed code of the image loaded in this process, if one was.
///
/// Process-wide rather than per-device because that is the shape of the thing
/// being remembered: an FPGA image is state in the hardware, it outlives every
/// handle we open onto it, and the reopen loop must not pay six seconds every
/// four. It is cleared by [`forget`] the moment a stream turns out to be silent
/// anyway, which is what makes unplugging and replugging a sampler recover on
/// its own rather than leaving it dead for the session.
static LOADED: Mutex<Option<u8>> = Mutex::new(None);

/// What the loader has to be asked for, if anything.
pub enum Load {
    /// Nothing to do: the model boots its own FPGA, or this image is already in.
    NotNeeded,
    /// Run this, with the interface **not** claimed.
    Run(Run),
    /// It cannot be done, and this is the sentence the operator needs.
    Unavailable(String),
}

/// A loader run that is ready to go.
pub struct Run {
    loader: PathBuf,
    code: u8,
    rate_hz: u32,
}

/// The speed code for `rate_hz`: which of ELAD's six FPGA images to load.
///
/// One per rate, lowest first, exactly as their loader numbers them. A rate
/// between two of the six takes the lower image rather than rounding, so an
/// unknown value can only ever ask for less bandwidth than it meant, never for
/// a stream the link cannot carry.
pub fn speed_code(rate_hz: u32) -> u8 {
    match rate_hz {
        r if r >= 6_144_000 => 6,
        r if r >= 3_072_000 => 5,
        r if r >= 1_536_000 => 4,
        r if r >= 768_000 => 3,
        r if r >= 384_000 => 2,
        _ => 1,
    }
}

/// Where ELAD's loader is, if it is anywhere.
pub fn loader() -> Option<PathBuf> {
    pick_loader(std::env::var_os(LOADER_ENV).map(PathBuf::from), CANDIDATES)
}

/// The search itself, with the environment passed in rather than read, so it
/// can be tested without a process-wide variable two threads would fight over.
fn pick_loader(from_env: Option<PathBuf>, candidates: &[&str]) -> Option<PathBuf> {
    if let Some(p) = from_env {
        // An override that names nothing is a refusal, not a hint: the operator
        // has said where theirs is, and quietly running a different binary from
        // somewhere else is the last thing they asked for.
        return p.is_file().then_some(p);
    }
    candidates.iter().map(PathBuf::from).find(|p| p.is_file()).or_else(|| which(LOADER))
}

/// The first `name` on `PATH`, without pulling in a crate to ask.
fn which(name: &str) -> Option<PathBuf> {
    std::env::var_os("PATH").and_then(|path| {
        std::env::split_paths(&path).map(|dir| dir.join(name)).find(|p| p.is_file())
    })
}

/// Whether this device needs an FPGA image loading before it can stream, and
/// what it would take.
pub fn wanted(model: Model, rate_hz: u32) -> Load {
    if !model.needs_fpga_load() {
        return Load::NotNeeded;
    }
    let code = speed_code(rate_hz);
    if *LOADED.lock().unwrap_or_else(|e| e.into_inner()) == Some(code) {
        return Load::NotNeeded;
    }
    match loader() {
        Some(loader) => Load::Run(Run { loader, code, rate_hz }),
        None => Load::Unavailable(missing_loader(model)),
    }
}

/// Forget what was loaded, so the next open loads it again.
///
/// Called when a stream has delivered nothing at all: whatever this module
/// believed about the hardware is not true any more — the likeliest reason
/// being that the device has been unplugged and plugged back in, which empties
/// the FPGA again.
pub fn forget() {
    *LOADED.lock().unwrap_or_else(|e| e.into_inner()) = None;
}

impl Run {
    /// Program the FPGA. Returns the sentence to put in front of the operator
    /// if it did not work.
    ///
    /// The device must not be claimed: the loader opens it itself, and a
    /// claimed interface is refused to everybody including us.
    pub fn execute(self, trace: &Trace) -> std::result::Result<(), String> {
        let what = format!(
            "{} + {} ({:.0} kHz)",
            self.loader.display(),
            self.code,
            self.rate_hz as f64 / 1000.0
        );
        tracing::info!("loading the ELAD's FPGA image: {what}");
        trace.note(format!("fpga: running {what}"));

        let started = Instant::now();
        let mut child = match Command::new(&self.loader)
            .arg("+")
            .arg(self.code.to_string())
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
        {
            Ok(c) => c,
            Err(e) => {
                let w = format!(
                    "sdroxide could not run ELAD's FPGA loader {} ({e}). Until the \
                     image is loaded the receiver answers every command and sends no \
                     samples at all — make the loader executable, or run \
                     `{LOADER} + {}` yourself before starting sdroxide",
                    self.loader.display(),
                    self.code,
                );
                trace.note(format!("fpga: {w}"));
                return Err(w);
            }
        };

        // `wait` with a deadline. std has no timed wait, and a loader that has
        // hung would otherwise hold the stream thread — and with it the open
        // this is running inside — for as long as the program is up.
        let status = loop {
            match child.try_wait() {
                Ok(Some(s)) => break Some(s),
                Ok(None) if started.elapsed() < LOADER_TIMEOUT => {
                    std::thread::sleep(Duration::from_millis(100));
                }
                Ok(None) => {
                    let _ = child.kill();
                    let _ = child.wait();
                    break None;
                }
                Err(_) => break None,
            }
        };

        match status {
            Some(s) if s.success() => {
                trace.note(format!("fpga: loaded in {:.1}s", started.elapsed().as_secs_f64()));
                *LOADED.lock().unwrap_or_else(|e| e.into_inner()) = Some(self.code);
                Ok(())
            }
            other => {
                let outcome = match other {
                    Some(s) => format!("exited with {s}"),
                    None => format!("did not finish within {}s", LOADER_TIMEOUT.as_secs()),
                };
                let w = format!(
                    "ELAD's FPGA loader {} {outcome}. The receiver will answer every \
                     command and send no samples until an image is loaded — try \
                     `sudo {LOADER} + {}` in a terminal and see what it says",
                    self.loader.display(),
                    self.code,
                );
                trace.note(format!("fpga: {w}"));
                Err(w)
            }
        }
    }
}

/// Reclaim a device that has just been reprogrammed.
///
/// It re-enumerates on the way through, so the first few attempts are expected
/// to find nothing; only the last failure is worth reporting.
pub fn reclaim(serial: &str, trace: &Trace) -> crate::error::Result<crate::usb::UsbDev> {
    let deadline = Instant::now() + RECLAIM_WINDOW;
    loop {
        match crate::usb::UsbDev::open(serial, trace) {
            Ok(dev) => return Ok(dev),
            Err(e) if Instant::now() < deadline => {
                trace.note(format!("fpga: waiting for the device to come back ({e})"));
                std::thread::sleep(RECLAIM_INTERVAL);
            }
            Err(e) => return Err(e),
        }
    }
}

/// One line for the diagnostic report: whether the loader is here, and what
/// this process has put in the device with it.
///
/// Near the top of the dump on purpose. It is the first thing to check on any
/// report of a sampler that will not stream, and the answer is invisible from
/// everywhere else — nothing in the USB exchange below it looks any different
/// against an empty FPGA.
pub fn status_line() -> String {
    let loaded = match *LOADED.lock().unwrap_or_else(|e| e.into_inner()) {
        Some(code) => format!("image {code} loaded this session"),
        None => "no image loaded this session".to_string(),
    };
    match loader() {
        Some(p) => format!("{} — {loaded}", p.display()),
        None => format!("{LOADER} not found — {loaded}"),
    }
}

/// What to say when the loader is not installed.
fn missing_loader(model: Model) -> String {
    format!(
        "the {}'s FPGA is not loaded at power-up, and until it is the receiver \
         answers every command and sends no samples at all. sdroxide can load it \
         for you but could not find ELAD's `{LOADER}` — download it from ELAD's \
         Linux area (eladit.com → Download → SDR/Linux), copy it to \
         /usr/local/bin/{LOADER}, and make it executable. Set {LOADER_ENV} if you \
         keep it somewhere else",
        model.name(),
    )
}

/// What to say when a stream has run and delivered nothing at all.
///
/// The one symptom this backend can produce with no error anywhere in it, so it
/// gets a sentence of its own rather than being left to the silence watchdog —
/// which reopens the device every three seconds, for ever, without a word.
pub fn silence_hint(model: Model) -> String {
    if model.needs_fpga_load() {
        format!(
            "the {} has not delivered a single sample. That is what an unloaded \
             FPGA looks like: it enumerates, reports its serial, and acknowledges \
             the start of the stream, with no down-converter in it to produce one. \
             Install ELAD's `{LOADER}` loader (eladit.com → Download → SDR/Linux) \
             as /usr/local/bin/{LOADER} and sdroxide will run it at every open",
            model.name(),
        )
    } else {
        format!(
            "the {} has not delivered a single sample. Check that the cable is in \
             the radio's RX socket rather than CAT or USB Audio, and that nothing \
             else (FDM-SW2, a gr-elad flowgraph) is holding the receiver",
            model.name(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One image per rate, and every rate on the list has to name a distinct
    /// one — they are six different files, not one register with six values.
    #[test]
    fn every_sample_rate_has_its_own_image() {
        let codes: Vec<u8> =
            sdroxide_types::ELAD_SAMPLE_RATES.iter().map(|&r| speed_code(r)).collect();
        assert_eq!(codes, vec![1, 2, 3, 4, 5, 6]);
    }

    /// A rate that is not one of the six takes the image below it, never the
    /// one above: asking for more bandwidth than was meant is the failure that
    /// the USB link cannot carry.
    #[test]
    fn an_unlisted_rate_rounds_down_rather_than_up() {
        assert_eq!(speed_code(0), 1);
        assert_eq!(speed_code(48_000), 1);
        assert_eq!(speed_code(191_999), 1);
        assert_eq!(speed_code(383_999), 1);
        assert_eq!(speed_code(5_000_000), 5);
        // Above the top rate there is nothing higher to pick.
        assert_eq!(speed_code(12_288_000), 6);
    }

    /// The transceiver boots its own FPGA, so nothing here may run for it — a
    /// loader aimed at a radio that is mid-QSO is not a thing to be casual
    /// about.
    #[test]
    fn only_the_samplers_are_ever_programmed() {
        assert!(matches!(wanted(Model::Duo, 192_000), Load::NotNeeded));
    }

    /// The second open of a session must not pay six seconds for an image that
    /// is already in the device — the silence watchdog reopens every three.
    #[test]
    fn an_image_that_is_already_loaded_is_not_loaded_again() {
        forget();
        *LOADED.lock().unwrap() = Some(speed_code(3_072_000));
        assert!(matches!(wanted(Model::S2, 3_072_000), Load::NotNeeded));
        // A different rate is a different image, so it is not already in.
        assert!(!matches!(wanted(Model::S2, 192_000), Load::NotNeeded));
        // And a device that turned out to be silent is not to be trusted.
        forget();
        assert!(!matches!(wanted(Model::S2, 3_072_000), Load::NotNeeded));
    }

    /// An override that names nothing finds nothing — it must not fall through
    /// to a binary somewhere else that happens to have the right name.
    #[test]
    fn an_override_is_taken_at_its_word() {
        let missing = PathBuf::from("/nonexistent/elad-firmware");
        assert_eq!(pick_loader(Some(missing), CANDIDATES), None);

        // One that does name something is used whatever is installed.
        let here = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("src/fpga.rs");
        assert!(here.is_file(), "{}", here.display());
        assert_eq!(pick_loader(Some(here.clone()), CANDIDATES), Some(here.clone()));

        // With no override, a candidate that exists wins over the search path.
        let candidate = here.to_str().unwrap();
        assert_eq!(pick_loader(None, &[candidate]), Some(here));
        assert_eq!(pick_loader(None, &["/nonexistent/one", "/nonexistent/two"]), which(LOADER));
    }

    /// Both sentences have to name the loader and the model, because they are
    /// read on screen with no other context around them.
    #[test]
    fn the_operator_is_told_what_to_install() {
        let w = missing_loader(Model::S2);
        assert!(w.contains("FDM-S2"), "{w}");
        assert!(w.contains(LOADER), "{w}");
        let s = silence_hint(Model::S2);
        assert!(s.contains(LOADER), "{s}");
        // The transceiver's version must not send anybody after a loader it
        // does not need.
        let duo = silence_hint(Model::Duo);
        assert!(!duo.contains(LOADER), "{duo}");
    }
}
