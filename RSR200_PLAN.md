# RSR200 support in sdroxide — implementation plan

Reuter RSR200B, a two-ADC (dual-channel) HF/VHF direct-sampling receiver over USB or LAN,
GPS-disciplined, with a hardware combiner for antenna diversity/notching built into the radio
itself. This plan is grounded in a completed, live-hardware-tested implementation of the same
radio for a different SDR program (SDR++, a C++ project) — every protocol fact below was
verified against the manufacturer's own documents (`RSR200_DP_ENG_V52.pdf`, the wire protocol;
`RSR200_OM_V225.pdf`, the operations manual and its ExtIO reference implementation notes) and,
for most of it, against the real radio. Where something is untested against real sdroxide-side
hardware, that's called out explicitly rather than presented as settled.

Ralph's own request for this plan: work out whether `sdroxide-dsp`'s existing `Diversity` filter
(built for the SDRplay RSPduo's two tuners) is usable as-is for the RSR200. Short answer, with
the reasoning in its own section below: **yes, for the RSR200's own two-channel "Separate" mode
— and arguably a *better* match for it than the RSPduo is**, but it is the wrong tool entirely
for the radio's *other*, separate diversity capability (a fixed hardware combiner inside the
RSR200 itself). Both are covered.

## 1. What already exists in sdroxide that this can build on

Surveyed directly (not assumed) before writing any of the rest of this plan:

- **`IqSource` trait** (`crates/sdroxide-radio/src/source.rs`) is the whole contract: a single
  `read(&mut self, buf: &mut [Complex32]) -> Result<usize>` plus gain/antenna/setting hooks, all
  but a handful defaulted. It has no concept of "channels" — a backend that owns two ADCs (the
  RSPduo, and the RSR200) is responsible for turning them into the one stream this trait expects,
  internally.
- **The established shape for a native (non-SoapySDR) backend** — `sdroxide-hydrasdr` and
  `sdroxide-airspy` both split as `device.rs` / `error.rs` / `handle.rs` / `lib.rs` / `protocol.rs`
  / `stream.rs` / `trace.rs` / `usb.rs`. This is not a coincidence or convention I'm inferring —
  the two crates are near-identical in layout (hydrasdr's own doc comment says as much: the RFOne
  is "a *fork* of the Airspy R2"). It maps closely onto the three-layer split the SDR++ module
  already uses (`rsr200_protocol.h`: pure wire-format functions, no I/O; `rsr200_device.h`:
  transport-agnostic command sequencing/state machine; a transport underneath) — `protocol.rs` and
  `device.rs` here would carry almost the same responsibilities.
- **A native backend with *two* transports already exists**: `sdroxide-rtlsdr` has both `usb.rs`
  and a `tcp/` submodule (the `rtl_tcp` client), and its own doc comment states the intended
  pattern explicitly: "Same shape as the USB backend next door... it hands back the very same
  [`RtlSdrHandle`], because nothing in that handle was ever about USB. What changes is only where
  the register writes happen." This is exactly the RSR200's own situation (USB and LAN share one
  command protocol, differing only in framing) and exactly what `rsr200_device.h`'s own
  transport-agnostic `Device` class already does in the SDR++ code this plan is drawing from — the
  same design, arrived at independently on both sides, already has a home in this workspace.
- **Threading model**: plain `std::thread::spawn` + `crossbeam_channel` + an `rtrb` ring buffer
  (confirmed in `sdroxide-rx888`/`sdroxide-hydrasdr`'s own `handle.rs` files), not async/tokio.
  One thread owns the transport and does as little as possible on it; sample conversion happens
  either inline (RTL-SDR's rate) or on a second thread (RX-888's rate, gated by a comment
  explaining exactly why). The RSR200 at its higher decimation settings/24-bit mode is closer to
  RX-888 territory than RTL-SDR's, worth deciding explicitly rather than defaulting to inline (see
  §6).
- **Registration is four touchpoints, all found directly, none guessed**: a `Backend` enum variant
  (`crates/sdroxide-types/src/radio.rs`), a per-backend `open_<name>_source()` function and match
  arm in `src/main.rs` (`fn open` around line 1360), a config struct in
  `crates/sdroxide-types/src/radio.rs` (`#[serde(default)]`, `Serialize + Deserialize`, so it's
  safe in the wasm settings client), and a `settings_<name>_tab()` function in
  `crates/sdroxide-ui/src/app/settings/radio.rs`.

## 2. The RSR200 itself, compressed to what a new backend needs

Full detail lives in the already-tested SDR++ module (`source_modules/rsr200_source/` in that
repo) — this is the load-bearing summary, not a replacement for reading the protocol header
directly when actually writing `protocol.rs`.

**Two transports, one protocol.** USB is an FTDI FT601Q "SuperSpeed-FIFO" bridge; LAN is TCP port
**55557** (`LAN_TCP_PORT` in the SDR++ source, confirmed against the DP manual). Every command is
`[4-byte LE command number][1-byte instruction][params...][repeat byte]`, fixed-length on USB (8,
12 or 16 bytes depending on instruction) and a few bytes shorter on LAN (the trailing bytes are
"not transmitted via LAN" per the DP's own tables — not a different protocol, just a different
wire length for the identical logical command). A handful of instructions actually used:
`SET_ADC_CLOCK (0xF2)`, `SET_DATA_TRANSMISSION (0xB4)`, `SET_VARIABLE (0xF5)` (switch register,
per-channel attenuators), `SET_AUTO_ATT (0xB1)`, `SET_GENERATORS (0xB0)` (LO tuning, and — selector
9 — the hardware diversity weight for channel 2), `START_STREAM`/`STOP_STREAM`, `READ_VERSION`,
`RESET`.

**Acknowledgement is not synchronous.** DP 3.1/3.2: replies arrive *embedded in the next streaming
block*, not as a standalone reply to the command that just went out — except the version-query
reply before streaming has started, which (LAN only) *does* arrive as its own standalone packet.
This was a real, live-hardware-confirmed gap the first time it was implemented (a synchronous
packet-mode read was tried and disproven by packet capture) — a fresh implementation should treat
"reply embedded in the next block, one outstanding command tracked with a timeout+retry, LAN's
version-query as the one standalone-packet exception" as settled rather than re-discovering it.

**Framing.** LAN blocks carry sync words + a 32-bit counter + its bitwise inverse, letting a
receiver resync inside an arbitrary byte stream (find sync bytes, confirm counter/~counter agree,
declare a block boundary). USB has no such resync problem — packets are already framed by the
transport. **A real trap, confirmed live**: at 24-bit, block length is the *same* for 1- and
2-channel mode, so 24-bit 2-channel transmits half the samples per block that 1-channel does — a
geometry calculation that doesn't account for this will misframe silently rather than error out.

**Sequence/gap detection is USB-only.** live testing found the LAN block counter's own steady-state
behaviour isn't a reliable "+1 per block" at all (a decimation-dependent, sometimes non-monotonic
delta) — treating it as a loss indicator on LAN produces constant false positives. USB's own packet
counter *is* meaningful for this. Don't build one gap-detection code path and assume it's right for
both transports.

**Tuning is Nyquist-zone folding**, not a direct synthesizer — `tuneFor(rfHz, adcClockHz)`
computes which zone the wanted frequency falls in, the LO frequency that brings it to baseband, and
whether the zone is even (spectrum arrives mirrored, needs conjugating) or odd. Both oscillators
are set together in one command (`GEN_LO_BOTH`) — DP 4.6 is explicit that setting them separately
and later reconverging leaves the inter-channel phase undefined until the next resync event, which
is exactly the property dual-channel/diversity work below depends on.

**Command ordering inside a full reconfigure matters and is not obvious from the command
descriptions alone**: ADC clock, then data-transmission/port mode (both trigger a channel
resynchronisation event per DP 4.6), then switch register + attenuators + Auto-ATT (order among
these doesn't matter), then tuning *last* — so retuning follows the resync event rather than being
wiped out by it.

**Two independent per-channel-count-relevant modes**, both DSP-mode-byte selections (not
mutually exclusive with each other in the protocol's own bit layout, but mutually exclusive in
practice/UI): **Separate** (`OP_INDEPENDENT`, mode 0) — both ADCs delivered as two genuinely
independent, phase-coherent, sample-aligned channels, port-mode bit 4 (dual-channel) required set;
and **Diversity** (`OP_DIVERSITY`, mode 3) — the radio itself combines the two ADCs into *one*
stream using a fixed complex weight the host sets, port-mode bit 4 clear (single-channel wire
format — confirmed both by the SDR++ implementation's own live-hardware bug/fix cycle, documented
in its `RSR200_PLAN.md`, and independently by the OM's changelog entry for the vendor software's
own "AntDiv" preset). §4 covers what each means for this plan specifically.

**Everything else** (GPS discipline + frequency-correction readout, Auto-ATT's threshold/hold-
time/per-channel calibration gain and its own gain-compensation math, Serial/time-interleaved
mode, 16-/24-bit format, swap-channels, VHF input + preamp switching) is real, already-solved,
and out of scope for *this* plan to re-derive — §7 has the phased build order that defers all of
it past first light.

## 3. Is `sdroxide-dsp::Diversity` usable as-is? (Ralph's actual question)

Read `crates/sdroxide-dsp/src/diversity.rs` in full before answering this. It's an adaptive,
multi-tap NLMS filter over a *pair of coherent input streams* — `process(&mut self, main: &mut
[Complex32], aux: &[Complex32])`, called once per block from `SdrPlaySource::read()`
(`src/sdrplay_source.rs`) after the driver hands back a sample-aligned pair from the RSPduo's two
tuners. Its own doc comment is explicit about what it needs to work at all: "a second receiver,
coherent with the first because the two chains share one synthesiser and one sample clock."

**That's a description of the RSR200's own Separate mode, arguably more precisely than it
describes the RSPduo.** The two ADCs share a genuine synchronisation event (DP 4.6 — the same
event a shared LO tune triggers), and unlike the RSPduo — whose own dual-tuner phase offset is
stable only *within* a run and is redrawn on every restart (an actual finding from the earlier
phasing work this plan draws on, not an assumption) — the RSR200's inter-channel phase relationship
is a documented, repeatable property of that resync event, not a per-session accident. So:

- **In Separate mode, the RSR200 backend should read both channels and run
  `sdroxide_dsp::Diversity::process()` on the pair, exactly the way `SdrPlaySource::read()` does**:
  read both ADCs into `main`/`aux` scratch buffers, convert to `Complex32`, call `process()`,
  return `main` (now combined) as the `IqSource::read()` result. No changes to `sdroxide-dsp`
  needed — this really is reuse, not "inspired by."
- **In Diversity (hardware combiner) mode, `Diversity` has nothing to do and must not be
  wired in.** The radio has already summed the two ADCs, weighted by a magnitude/phase pair the
  host set once via `SET_GENERATORS` selector 9, before a single sample reaches the host. There is
  no second channel arriving to feed an adaptive filter with — calling `Diversity::process()` here
  would either be a silent no-op (nothing to combine) or, worse, misleadingly present a fixed,
  already-applied hardware weight as if it were the *software* filter's own adaptive state.
  This needs its own, separate UI/control surface instead: a way to set the radio's magnitude/phase
  weight directly (§4's "solve, then apply" flow — the *radio-side* half of this was already
  built and tested in SDR++; the *software-solve* half it reads from is now built and tested in
  sdroxide too, see §4), not an instance of `Diversity`.

One real, deliberately-unsolved combination, carried over honestly rather than glossed: the
SDR++ implementation's own plan document flags that Auto-ATT's per-channel calibration gain
interacts with hardware diversity's own weight when *both* are active on channel 2 simultaneously,
and that interaction has never been derived, let alone tested. If sdroxide ever offers Auto-ATT and
hardware diversity together, that gap is inherited, not newly introduced.

## 4. Two-channel handling, concretely

The RSR200 backend needs to expose **three** distinct operating shapes, matching what the SDR++
module already settled on (with the reasoning already argued through, live-hardware-corrected
once, and now stable) — not because sdroxide *needs* to replicate that exact taxonomy, but because
each shape maps to a genuinely different wire configuration and there is no simpler decomposition
that stays honest about what the radio is actually doing:

1. **Single channel** — one ADC in use, `format.channels = 1` on the wire, plain `IqSource`
   passthrough, nothing dual-channel-shaped touches this path at all.
2. **Separate (Diversity filter, software)** — both ADCs, `format.channels = 2` on the wire,
   `sdroxide_dsp::Diversity` combines them as described in §3. This is the one genuinely reusing
   existing sdroxide code, and the one to build and prove first (§7).
3. **Diversity (hardware combiner)** — both ADCs on the wire in the *same* 2-channel format as
   Separate mode (confirmed the hard way in SDR++: the first attempt assumed hardware-combined
   output meant a 1-channel wire format, which is wrong and produces a live, audible
   channel-deinterleaving artifact — a comb of spurs across the whole band — see that project's
   own `RSR200_PLAN.md` for the live-testing account), but with the DSP mode byte set to
   `OP_DIVERSITY` instead of `OP_INDEPENDENT` and a magnitude/phase weight sent via `SET_GENERATORS`
   selector 9. Which of the two received channels carries the combined result was *also* only
   settled by live testing in the SDR++ work (channel A, per the DP's own "channel 2 is added to
   channel 1" wording and confirmed on real hardware) — worth re-confirming rather than assuming
   it transfers, since it's a fact about the radio's firmware, not about either host program.

   **The "solve, then apply" workflow this needs now exists** (branch `decorrelation`,
   `crates/sdroxide-dsp/src/diversity.rs`, built and tested against the RSPduo's own settings —
   see `DECORRELATION_PLAN.md`): `Diversity::decorrelated_weight()` returns exactly `Option<(k0,
   k1)>` for a `Diversity` set to `DiversityAlgorithm::Decorrelate`, solved as `y = k0·main +
   k1·aux` — the *same* convention the radio's own additive weight uses (`y = k0·A + k1·B`), not a
   coincidence so much as the natural closed form for "one complex weight combining two coherent
   channels," found independently on both sides. The RSR200 backend's own hardware-diversity mode
   would run a `Diversity` instance (or call `covariance_eigen`/`cancel_weight` directly — both
   `pub` in that module) against the two ADC channels *before* switching the radio into
   `OP_DIVERSITY`, read `decorrelated_weight()` (or the raw eigenpair, for a `Combine`-style
   weight rather than a null), convert `k0`/`k1` into the radio's magnitude/phase units (1 LSB =
   1/8192 up to just under 8×, phase a signed 16-bit degrees value — `k0` is fixed at 1 in the
   `Cancel` case per `Diversity`'s own unity-gain-on-main convention, so only `k1` needs encoding
   there), and send it once via `SET_GENERATORS`. Not a live control loop — the round trip through
   the command channel is too slow for one, confirmed in the SDR++ implementation. The wideband,
   per-bin technique (`WidebandDecorrelator`, same branch) does **not** apply here: the radio's
   hardware combiner takes exactly one weight, not one per bin, so there is nothing for a per-bin
   solve to hand it that the scalar one doesn't already provide more directly.

## 5. Crate layout

```
crates/sdroxide-rsr200/
  Cargo.toml           # sdroxide-types, sdroxide-dsp (Complex32; Diversity is used by the
                        # *source* glue in src/, not by this crate itself — see §3/§6)
  src/
    lib.rs
    protocol.rs         # pure wire-format: command construction, block/packet geometry,
                        # Nyquist-zone tuning math, hardware-diversity weight packing,
                        # Auto-ATT gain/hold-time conversions. No I/O, no sdroxide-radio
                        # dependency — mirrors rsr200_protocol.h's own "written and tested
                        # against the documented layouts before the radio is on the desk"
                        # design intent, and should be just as unit-testable in isolation.
    device.rs           # transport-agnostic config/command sequencing and the
                        # acknowledgement/retry state machine (§2's "not synchronous"
                        # section) — the direct analogue of rsr200_device.h's Device class.
                        # Takes a Transport trait object; knows nothing about USB or TCP.
    usb.rs               # FTDI D3XX transport — see §6 for why this is the hard part
    lan.rs               # TCP transport, closely modelled on sdroxide-rtlsdr's tcp/mod.rs
    stream.rs             # the worker thread(s): own the transport, drain it, hand blocks
                          # to device.rs for parsing, push converted samples through an
                          # rtrb ring — same shape as every other native crate here
    handle.rs             # the public API src/rsr200_source.rs actually calls: spawn(),
                          # read_pair() (mirroring SdrPlayHandle's own read_pair(), since
                          # Separate/Diversity modes both need the two-ADC shape at this
                          # layer even when only one ends up in the final IqSource stream)
    error.rs
    trace.rs
```

```
src/rsr200_source.rs     # Rsr200Source: IqSource — the glue, modelled directly on
                          # sdrplay_source.rs: owns an sdroxide_dsp::Diversity in Separate
                          # mode exactly as SdrPlaySource does, owns nothing DSP-shaped in
                          # Diversity (hardware) mode, sends the SET_GENERATORS weight
                          # command directly for that mode instead.
```

## 6. The hard problem this plan cannot make easy: USB transport

Every native USB backend in this workspace (`sdroxide-hydrasdr`, `sdroxide-airspy`,
`sdroxide-rx888`, `sdroxide-rtlsdr`) uses **`nusb`** — pure Rust, no libusb, no vendor SDK. That
is not available for the RSR200's own USB interface as-is, and this is a real, hard-won finding
from the SDR++ work, not a guess: the FT601Q "SuperSpeed-FIFO" bridge chip needs FTDI's own D3XX
driver stack and shared library (`libftd3xx` / `FTD3XXWU.dll`), because it is not a standard
USB-class device generic bulk/control transfers can drive — this was tried and disproven on macOS
specifically (`FT_CreateDeviceInfoList` reported zero devices even though the OS's own USB log
showed the chip enumerating) before the vendor driver dependency was accepted as unavoidable.

Concretely, for sdroxide:

- **Linux and macOS**: FFI bindings to `libftd3xx`'s async pipe read/write calls
  (`FT_ReadPipeAsync`/`FT_WritePipeAsync` on those platforms specifically — Windows' own D3XX SDK
  uses differently-named/shaped calls, `FT_ReadPipeEx` plus `LPOVERLAPPED`, and a genuinely
  separate, previously-undocumented difference: **the Linux/macOS async calls address a logical
  FIFO channel 0–3, not the raw USB endpoint address every other D3XX pipe call and the Windows
  read call use** — found by a standalone probe, not assumed, in the SDR++ work). This is real
  `unsafe` FFI work, a first for a native driver crate in this workspace as far as this survey
  found, and it's a hard *build-time* dependency too (the SDK headers/libs have to be present and
  linkable), not just a runtime one.
- **Windows** *might* have an easier path: the OM's own changelog documents a newer
  "FTD3XXWU" driver "compatible with FTDI Superspeed-FIFO Bridge USB devices," explicitly
  contrasted with the old FT601-class driver, described as behaving "much more stable (no
  'freezing' when the USB connection is interrupted)." Whether that WU variant is reachable through
  a generic WinUSB-based stack (which `nusb` could plausibly drive) rather than needing its own
  vendor DLL is genuinely unknown from this desk — worth a short, dedicated research spike before
  committing to either the FFI path or a `nusb` path for Windows specifically, rather than assuming
  either way.
- **LAN has none of this problem.** Plain TCP, `std::net::TcpStream`, directly analogous to
  `sdroxide-rtlsdr/src/tcp/mod.rs`. This is also where the SDR++ implementation's own hardest
  bring-up problems concentrated once USB was working (finding the radio on the network at all —
  its LAN LED indicates physical SFP link, not that it's joined the network; a DHCP-assigned
  address rather than the documented static default; the packet-mode/embedded-reply distinction in
  §2) — all now-solved, all directly transferable as *knowledge*, none of it requiring anything
  sdroxide doesn't already have (a `TcpStream`, a byte-stream resync loop for the block framing).

**Recommendation, stated plainly rather than left implicit**: build and prove LAN first. It's
architecturally simple, has no new build-time dependency, and is where §3's actual point (the
`Diversity` reuse) can be fully exercised and shown working end-to-end. Treat USB as a second
phase with its own research spike for the Windows question above, and accept from the outset that
Linux/macOS USB is real FFI work against a proprietary vendor library — not a corner that can be
cut by reaching for `nusb` the way every sibling crate here did.

## 7. Suggested build order

Each stage independently useful and independently testable, matching this workspace's own
apparent convention (and the SDR++ project's, which phased USB/LAN/dual-channel/extras
separately and shipped each as it landed):

1. **Done. `sdroxide-rsr200::protocol`** (branch `rsr200`) — a faithful port of the already-tested
   `rsr200_protocol.h`, all 19 of its worked-example checks transliterated 1:1 as Rust `#[test]`s
   and passing on the first run: block geometry (LAN and USB), status header parsing, sample
   unpacking (16/24-bit, sign extension, per-channel Auto-ATT gain), block resync, every command
   builder, reply parsing (including the BCD firmware-version decode), Nyquist-zone tuning against
   the OM's own three worked examples, and the hardware-diversity weight conversion (§4) including
   its quantisation-through-the-wire-format round trip. No hardware needed, none used — exactly
   the point of doing this piece first. `cargo test -p sdroxide-rsr200`: 19/19 pass; `cargo clippy`:
   clean. Mechanical adaptations from the C++ original only (`u32::to/from_le_bytes` instead of
   hand-rolled byte shuffling, `Option<Reply>` instead of an out-parameter, `#[repr(u8)]` enums
   cast with `as u8` at the call sites) — no wire-format or semantic changes.
2. **Done. LAN transport + `device.rs`** (branch `rsr200`) — `sdroxide-rsr200::device` (the
   `Transport` trait, `Config`, `Device`'s full configuration-ordering/command-numbering/
   acknowledgement-and-retry/frame-parsing state machine) and `sdroxide-rsr200::lan`
   (`LanTcpTransport`, over `std::net::TcpStream` per `sdroxide-rtlsdr`'s own `tcp/mod.rs`
   precedent, not the C++ original's raw BSD sockets). One structural adaptation from the C++
   reference worth recording: `Device` takes `&mut dyn Transport` as a parameter on every method
   that needs one rather than storing it, and returns what `pump()` found directly rather than
   invoking stored `onSamples`/`onReply`/`onError` closures — both are genuine Rust ownership
   constraints (a stored transport can't also be held by a caller wanting to inspect it; a stored
   closure can't safely read the buffers it would need to while also being called from inside
   `self`), not stylistic changes, and were only found once the direct 1:1 port stopped compiling.
   All 11 of the reference `test_device.cpp`'s test scenarios ported 1:1 (configuration order,
   stream-stop-before-reconfigure, tuning, ack/retry/give-up, embedded-reply de-duplication,
   frame parsing, spectrum-inversion tracking the live tuning, Auto-ATT gain compensation and
   resend timing, USB framing sharing the same `Device`) plus three new tests against a real
   `TcpListener` on localhost for `lan.rs` itself (connect failure, mid-stream resync, cross-thread
   `stop()` unblocking a pending read without a full close) — 33/33 pass, `cargo clippy`: clean,
   the three socket tests stable across repeated runs. Single channel, 16-bit is what's been
   exercised (no radio, and no USB transport, exists yet to try 24-bit or dual-channel against);
   the format-handling code itself is generic across both, per the plan's own crate-layout intent.
3. **Done. `Backend::Rsr200` registration** (branch `rsr200`) — `sdroxide-rsr200::handle`
   (`Ctrl`/`Pending`/`Shared`/`push_iq`/`ring_for`/`Rsr200Handle`, the same shape as
   `sdroxide-rtlsdr::handle`) and `sdroxide-rsr200::stream` (the worker thread: owns the
   transport and the `Device`, drains one, feeds the other, pushes converted samples through an
   `rtrb` ring — connect-and-configure-or-fail blocking synchronously inside `spawn()` so a wrong
   address comes back as an ordinary open error, not a stream that silently never starts); then,
   outside the crate, `src/rsr200_source.rs` (the `IqSource` impl, modelled on
   `hydrasdr_source.rs`), `Rsr200Config`/`Backend::Rsr200`/`RadioConfig.rsr200` in
   `sdroxide-types` (postcard-positional append, `PROTO_VERSION` 90 → 91), and
   `settings_rsr200_tab` in `sdroxide-ui` (modelled on `settings_tci_tab`/`settings_icomnet_tab`:
   no Discover button, no device list — the radio neither announces itself nor sits on this
   machine's USB bus).
   One deliberate design split, decided here rather than inherited from the C++ original (which
   has no such distinction because it owns the ring's sizing dynamically): the two attenuators are
   *live*, riding `Command::SetGain` straight to the running stream thread, because they're real
   front-end elements the radio can move without touching the socket; host, port, ADC clock,
   decimation and GPS discipline are all *reopen-triggers* instead — a `before != after` check in
   `settings_rsr200_tab` sets `apply`, the same convention `sdrplay_source.rs` already uses for its
   own device/rate/bandwidth fields — because any of the latter changes the sample rate, and
   `sdroxide-rsr200` does not resize its `rtrb` ring on the fly. (`ADC_CLOCK_ELEMENT`/
   `GPS_DISCIPLINE_ELEMENT`/`DECIMATION_ELEMENT` gain-element consts were drafted for the opposite,
   all-live design before this split was settled, then removed as dead code once it wasn't.)
   `cargo build -p sdroxide`: clean. `cargo test` across `sdroxide`, `sdroxide-ui`,
   `sdroxide-rsr200`, `sdroxide-types`, `sdroxide-proto`: all green, `sdroxide-rsr200` itself now
   38/38 (the 33 from steps 1–2 plus 5 new for `handle::Pending`/`push_iq`/`ring_for`). `cargo
   clippy --all-targets` on the same five crates: no new warnings — the handful clippy reports are
   all pre-existing, in unrelated files this work never touched. Single channel, 16-bit, LAN is the
   whole of what streams.

   **Verified against a real RSR200** (2026-08-24, over WiFi rather than wired LAN): registering
   `Backend::Rsr200` in `RadioConfig`, the settings-tab dispatch and `open_configured_source` was
   not enough on its own — it never reached `settings/mod.rs`'s own hand-maintained `iface_opts`
   list, a *second*, separate enumeration that actually populates the Interface dropdown
   (`Backend::ALL` is not it). Built, opened a settings tab, accepted a host/port — but was not
   selectable until that list got the same one-line addition. Fixed, rebuilt, confirmed selectable
   and working: real spectrum on screen, closing the loop step 3 set out to close. One real,
   expected caveat surfaced by an actual radio rather than the protocol-level test harness: brief
   dropouts at the ÷64 decimation setting (`decimation_exp = 5`, the *lowest* of the six rates,
   still on the order of a couple of Msps at 16 bits — a nontrivial continuous TCP payload) —
   consistent with WiFi's own bandwidth headroom under real household loss patterns, not a protocol
   or threading bug.

   **Confirmed wired-LAN clean through ÷8, breaking up at ÷4 and ÷2** the same day — settling the
   WiFi question above (the WiFi dropouts really were WiFi's, not this crate's) and turning up a
   real ceiling of its own: ÷4 and ÷2, the two highest-rate settings, both had "bad breakups" even
   wired. Matches plain throughput arithmetic well enough to be the likely explanation rather than
   a coincidence: at a 100 MHz ADC clock, ÷2 and ÷4 want roughly 1.6 Gbps and 800 Mbps of sustained
   payload — at or above what 1GbE can carry — while ÷8's ~400 Mbps sits comfortably inside it,
   matching exactly where clean stopped and breakup started. Unverified in the sense that this
   crate does nothing to confirm the link speed or the radio's actual configured ADC clock, so
   treat the specific numbers as illustrative, not measured — but the boundary landing precisely at
   ÷8/÷4 is a strong signal it is a wire-speed ceiling, not a bug. A practical takeaway for now:
   ÷8 and coarser (the four lower rates) are the range to expect solid results in over ordinary
   1GbE; ÷2/÷4 likely need a faster link, not a driver fix.
4. **Done, software path verified against real hardware; not yet judged against two real
   antennas. Separate mode + `sdroxide_dsp::Diversity` wiring** (branch `rsr200`) — the part
   this plan exists to answer the question about.

   `stream.rs` sets `format.channels = 2` (still `OpMode::Independent` — "Separate" is the
   wire shape, not a different op mode; the radio's own hardware combiner is
   `OpMode::Diversity`, step 6, still not selected here) whenever
   `Rsr200Config::channel_mode` is `Rsr200ChannelMode::Separate`, a new enum (`Single`/
   `Separate` today; `RSR200_PLAN.md` deliberately leaves room to append a third variant for
   step 6's hardware combiner later without disturbing either). `handle.rs` gained
   `Rsr200Handle::read_pair`, a direct port of `SdrPlayHandle::read_pair` — one ring holding
   `QUAD`-wide quadruples instead of pairs when dual, de-interleaved on read the same way. One
   genuine simplification over the RSPduo case, not just a smaller port: `sdroxide-sdrplay`
   needs its own `Pairer` (`pair.rs`) to reconcile two *independently-arriving* tuner
   callbacks by hardware sample number; the RSR200's `Device::pump` already delivers both
   channels from the very same parsed frame in one call, so they cannot come apart on the way
   into the ring at all — nothing to reconcile, so nothing built to do it.

   `src/rsr200_source.rs` owns the combiner exactly the way `sdrplay_source.rs` does — all
   three `DiversityTechnique` variants (Adaptive, Decorrelate, WidebandDecorrelate; `sdroxide-
   dsp` itself needed zero changes, genuine reuse as promised in §3), a `Rsr200Diversity`
   config (`sdroxide-types`) field-for-field the same shape as `SdrPlayDiversity`'s own filter
   settings minus its two SDRplay-specific second-tuner gain fields (this radio's second-ADC
   gain is `attenuator2`, already its own top-level field). `settings_rsr200_tab` grew a
   Channels selector (reopen-triggering, like the transport/decimation fields) and, when
   Separate is selected, a full "Second ADC" controls section closely modelled on
   `settings_sdrplay_tab`'s own "Second aerial" one. `PROTO_VERSION` 92 → 93.

   **Verified against the real, physically-attached RSR200 the same day** (2026-08-24): a new
   standalone example, `crates/sdroxide-rsr200/examples/usb_dual_probe.rs`, configures the
   real `Device` for two channels, confirms `SampleBlock.dual` is actually set, collects
   ~100k real sample pairs from *both* ADCs (non-silent, changing between runs — genuinely
   live data, not zero-filled padding), and runs `Diversity::process()` against them without
   panicking. Both channels read the same RMS on every run, which reads as expected rather
   than suspicious — nothing indicates two genuinely different aerials were on the two inputs
   during this test. One transient failure seen and recorded in the example's own doc rather
   than chased further: the very first command write failed once, moments after a previous
   example's own `Stop Stream` on the same radio — the same class of "the radio needs a
   moment after Stop Stream" quirk DP 3.3 already documents for `FT_Create`, apparently also
   reachable on a plain write; a retry succeeded cleanly, twice.

   **Confirmed on air the same day, on two real antennas** — the RSPduo work's own "confirmed
   on air" milestone, reached: `DiversityTechnique::Decorrelate` (whole-span, one weight)
   "works as expected, nulling well on the current frequency." But
   `DiversityTechnique::WidebandDecorrelate` (per-bin) does **not** work on this radio as
   tested — Ralph's own words: "basically wipes out the entire band, nothing but noise, no
   carriers, nothing." Not the same failure the RSPduo work found and fixed in
   `WidebandDecorrelator` (instability across windows, fixed by the per-bin power gate,
   `DECORRELATION_PLAN.md`) — this is a full-band wipeout, not a wandering null, and the
   default 20 dB gate is already in effect. Not yet root-caused. One plausible reading, worth
   testing rather than trusted outright: `usb_dual_probe.rs`'s own two-channel test found the
   two ADCs reading identical RMS on every run (§7's own step 4 entry) — if the two aerials as
   connected are more similar/coupled than the RSPduo's own two independent tuners typically
   are, a per-bin solve could find "these two channels agree" in nearly every bin and null all
   of it, while the whole-span solve only locks onto the single dominant coherent
   relationship (likely the interferer) and leaves weaker, differently-phased signals
   elsewhere in the band comparatively alone. Untested against this specific hypothesis.
   `Rsr200Source::log_depth` (mirroring `sdrplay_source.rs`'s own, previously omitted here) now
   reports active-bin count and peak/average null depth to the log every ten seconds, so a next
   investigation has real numbers to read rather than needing to add logging first.
   `open_status()` and the settings tab both report per-technique status now: Decorrelate
   confirmed good, WidebandDecorrelate flagged in red as confirmed broken, Adaptive still
   unjudged on real antennas.
5. **Done. 24-bit, decimation range, GPS discipline/correction readout** (branch `rsr200`) —
   turned out to be smaller than the three items in its own title suggested, once actually
   checked against what steps 1–4 had already done.

   **Decimation range needed nothing.** `port_mode_byte`'s own clamp (`0..=5`) and
   `Rsr200Config::DECIMATION_EXPS` already cover the full documented range — confirmed against
   the C++ reference's own `portModeByte`, which clamps to the identical `0..=5`. Nothing to
   add; the plan's own title bullet was already satisfied by step 3's work, not a gap.

   **GPS discipline needed nothing either** — `gps_discipline` has been a real, wired-up
   `Rsr200Config` field and settings-tab checkbox since step 3, sent on every `SET_ADC_CLOCK`
   command. What was actually missing was the second half of the bullet: the *correction
   readout*.

   **24-bit** turned out to be a one-field addition, not new protocol work: `unpack`,
   `lan_layout` and `usb_samples_per_packet` in `sdroxide_rsr200::protocol` already handle both
   widths correctly — including the DP's own 24-bit/dual-channel block-length trap (§2) — since
   step 1, tested, unused. `Rsr200Config` gained `bits24: bool`; `stream.rs` sets
   `dev_cfg.format.bits` from it. `settings_rsr200_tab` grew a Sample width selector next to
   Channels, both reopen-triggering fields. `PROTO_VERSION` 93 → 94.

   **GPS/status readout** is the one piece that needed real new plumbing:
   `sdroxide_rsr200::protocol::Status`/`parse_status`/`freq_correction_hz` were already built
   and tested in step 1, but nothing between the wire and the app ever read them — `SampleBlock.status`
   reached `stream.rs`'s pump loop and was discarded every single time. Fixed: `handle::Shared`
   gained a `Mutex<Status>`, updated on every block; `Rsr200Handle::status()` reads it back.
   `Rsr200Source` gained `log_status()` (mirroring `log_depth()`'s own periodic-logging
   pattern, 30s interval) reporting temperature and GPS-corrected clock offset — correctly
   suppressing the temperature figure while Auto-ATT is engaged, since the DP's own `0x80`
   sentinel in that byte means "Auto-ATT active," not a real reading, a trap `parse_status`
   already avoided but `log_status` still had to know about. `open_status()` gained standing
   overload warnings for both ADCs, the same pattern `sdrplay_source.rs` already uses for its
   own front end. No live numeric readout in the settings dialog itself — that would need a
   wire from the running `IqSource` back into the settings UI that does not exist yet for *any*
   backend, a materially bigger change than "a small, mostly independent addition" calls for;
   the log is the honest answer for now, and the settings tab's own closing hint says so.

   `cargo build -p sdroxide`: clean. Tests across the same five crates as steps 3/4: all green.
   `cargo clippy --all-targets`: only the same pre-existing `for p in 0..pairs` pattern every
   sibling backend's own `read()` already has.

   **Verified against the real, physically-attached RSR200 the same day**, both pieces: a new
   standalone example, `crates/sdroxide-rsr200/examples/usb_status_probe.rs` — unlike
   `usb_live_probe.rs`/`usb_dual_probe.rs`, which drive `Device` directly, this one goes through
   `Rsr200Handle::open` and `stream.rs`, the actual path the app uses — opened with `bits24:
   true` and streamed 31M+ complex pairs over a 5-second run at the same 6.25 Msps the 16-bit
   runs got (correct: bit depth changes sample size, not sample rate), confirming the geometry
   math handles 24-bit end to end, not just in isolation. `handle.status()` read real, live
   values throughout: temperature settling around 70–71°C, no overload on either channel. GPS
   correction validity turned out to differ between two back-to-back runs of the identical
   config — the first read `freq_correction_valid: false` (raw `-8192`, the documented "no
   valid measurement" sentinel) for its whole 5 seconds; the second read a genuine valid
   correction throughout (`-75.0 Hz`, `gps_discipline: true`'s 0.5 Hz/LSB resolution). Not
   root-caused — GPS acquisition settling a few seconds after Start Stream is the obvious guess,
   not confirmed — but it does mean both of `log_status`'s branches (valid and invalid) have now
   genuinely been observed against real hardware, not just reasoned about from the protocol
   tests. `gps_discipline: false`'s own 0.1 Hz/LSB "measuring only" resolution was not
   separately exercised.
6. **Done. Hardware diversity (§3/§4's third mode)** (branch `rsr200`) — done last, per the
   plan's own original sequencing note that it needs Separate mode's own solve-from-software
   step to be meaningful.

   Genuinely less new protocol work than the section title suggests: `Device::set_hardware_diversity`/
   `set_hardware_diversity_from` and `protocol::hardware_weight_for`/`HardwareWeight` were already
   built and tested at step 1 — "the hardware-diversity weight conversion (§4) including its
   quantisation-through-the-wire-format round trip" was one of that step's original 19 protocol
   tests. What step 6 actually added was wiring a `Backend::Rsr200` channel mode up to that
   already-tested machinery. `Rsr200ChannelMode` gained a third variant, `HardwareDiversity` —
   `format.channels` stays 2, the *same* wire shape `Separate` uses (a real trap in the SDR++
   sibling implementation's own live testing: the first attempt there assumed a hardware-combined
   result meant a 1-channel wire format, which produced a live, audible channel-deinterleaving
   comb of spurs instead), and only `op_mode` changes, to `OpMode::Diversity`. `Rsr200Config`
   gained `hw_div_magnitude`/`hw_div_phase_deg` (`f64`, reopen-triggering — sent once at stream
   start, not adjustable live, since the round trip through the command channel is too slow for a
   control loop, confirmed in that same sibling implementation). `stream.rs` sends the channel-2
   weight via `Device::set_hardware_diversity` right after `apply_config` and before
   `start_stream`, matching that implementation's own order.

   **A real, proactive fix carried forward from the SDR++ sibling implementation's own live
   testing**, not something this session's own testing found: OM §6.2 documents that channel 2's
   magnitude/phase weight sits in the signal path even in Separate mode, and the vendor software
   sets it to unity when switching there — without that, a Separate-mode session following a
   hardware-diversity one on the same radio would inherit that session's own non-unity weight, and
   channel 2 would read as a clean, exact zero (real ADC2 data multiplied by a zero weight looks
   identical to no data at all). `stream.rs` now sends unity (1.0, 0°) whenever `channel_mode` is
   `Separate` too, for exactly this reason — fixed before it could ever bite here, not after.

   `src/rsr200_source.rs` gained `hw_diversity: bool` and `log_hardware_diversity_solve()` — the
   plan's own "solve, then apply" flow, adapted to what `sdroxide_dsp::Diversity` actually offers:
   unlike the SDR++ reference's `sigpath::phasing` (a global subsystem that can report a scalar
   weight even for its own adaptive/manual modes), `Diversity`'s `decorrelated_weight()` only ever
   has an answer in `DiversityTechnique::Decorrelate` — `Adaptive`'s multi-tap NLMS filter has no
   single complex weight to read out at all, and `WidebandDecorrelate`'s one weight *per bin* is
   exactly what a hardware combiner with one weight, full stop, cannot use (§4's own note on why
   that technique doesn't apply to hardware diversity). So "solve" is simpler here than in the
   reference: gated on Separate mode with Decorrelate selected, it reads the *already continuously
   solving* live filter's own `decorrelated_weight()` directly — no separate capture step needed.
   Logged rather than written back into a settings field: there is no wire from a running
   `IqSource` back into the settings dialog for any backend yet (step 5 hit the same gap for its
   own status readout) — copy the logged magnitude/phase into the new Hardware weight fields, then
   switch Channels to apply them. `settings_rsr200_tab` grew a third Channels entry, the Hardware
   weight fields (shown only in that mode), and a "Solve for hardware diversity" button (shown
   only in Separate mode with Decorrelate selected). `PROTO_VERSION` 94 → 95.

   **Verified against the real, physically-attached RSR200 the same day** (2026-08-24): a new
   standalone example, `crates/sdroxide-rsr200/examples/usb_hwdiv_probe.rs`, opened through the
   real `Rsr200Handle`/`stream.rs` path with `channel_mode: HardwareDiversity` — `OpMode::Diversity`
   and the channel-2 weight command were both accepted, first at unity (1.0, 0°) and then at a
   real non-unity weight (magnitude 0.5, phase 45°), with clean streaming (31M+ pairs per 5-second
   run) and readable status either way. The proactive Separate-mode unity-weight fix was exercised
   too, via the same real path (`usb_status_probe.rs` reconfigured to `Separate`): opened and
   streamed cleanly (62M+ pairs), with a valid GPS correction reading throughout, confirming the
   new `set_hardware_diversity` call in that mode does not itself break anything. **What these
   runs do not prove**: that the *combining* itself is correct — which channel actually carries
   the result (channel A, per DP/OM's own wording and the SDR++ reference's own live-hardware
   confirmation, not independently re-confirmed here), and whether a solved weight actually nulls
   or combines something real. That needs two real aerials and a human listening — the same
   "confirmed on air" milestone Separate mode already reached for its own software path.
   `open_status()` and the settings tab both say so plainly until it has been.
7. **Done on Linux/macOS, Windows still open. USB transport** (branch `rsr200`) — done out of
   order, ahead of steps 4–6, at Ralph's request. `sdroxide-rsr200::ffi` (hand-written D3XX
   bindings, loaded with `dlopen` at runtime via `libloading` — same pattern as
   `sdroxide-sdrplay`'s own `ffi.rs`, so this crate still builds and ships everywhere and
   merely finds USB missing where the driver is not installed) and `sdroxide-rsr200::usb`
   (`UsbTransport: Transport`, a direct port of the already-hardware-verified
   `transport_usb.h`/`.cpp` from the SDR++ sibling implementation — same `QUEUE_DEPTH`/
   `PACKETS_PER_READ` constants, empirically arrived at there, not re-derived here). One
   config, not a second `Backend`: `Rsr200Config` gained `transport` (`Rsr200Transport::Lan`/
   `Usb`) and `usb_serial`, and `settings_rsr200_tab` grew a Connection selector — matching
   `sdroxide-rtlsdr`'s own USB-and-`tcp/`-in-one-crate precedent and the SDR++ reference's own
   "Transport combo" UI shape (§1), not the `RtlSdr`/`RtlTcp`-style split-Backend convention,
   since USB and LAN really are the same radio with the same command protocol here. Windows is
   deliberately unimplemented — `Api::load()` fails there with a clear message pointing at this
   section — rather than guess at the differently-shaped Windows D3XX SDK (`FT_ReadPipeEx` as
   the *overlapped* call there, an inversion from Linux/macOS) without the research spike §6
   itself called for. `PROTO_VERSION` 91 → 92.

   **Verified against the real, physically-attached RSR200 the same day** (2026-08-24), not
   just built: a standalone example (`crates/sdroxide-rsr200/examples/usb_live_probe.rs`, a
   direct port of the reference's own `test_usb_live.cpp`, kept in the repo for future
   hardware bring-up the same way that file was) enumerates the D3XX device, opens it,
   configures and starts the stream through the real `Device`, and pumps samples for a fixed
   window — run directly against the connected radio rather than only reasoned about. At ÷8
   decimation (6.25 Msps): 31M+ frames delivered per 5-second run, essentially the exact
   requested rate, 0–1 gap events per run (the occasional one matching the reference's own
   note about transients right at Start Stream). At ÷2 (the highest rate, ~50 Msps
   requested): real, measurable loss — ~41.3 Msps actually delivered with over 5000 gap
   events in one run.

   **Confirmed clean through ÷4 the same day, against the real app rather than just the
   standalone probe**: Ralph reports "even 4x decimation had no breakups." So the USB
   ceiling found above is narrower than it first looked — real loss only at ÷2, the single
   highest rate, not the whole top half of the range the way LAN's own ÷4-and-÷2 ceiling
   works. A genuine USB-side throughput ceiling of its own regardless, distinct from LAN's
   gigabit one but the same shape of finding: this transport's own useful range tops out
   somewhere below its nominal maximum, not a bug so much as a fixed per-call overhead the
   reference's own `QUEUE_DEPTH`/`PACKETS_PER_READ` tuning already documented running into on
   Windows.

   **A real bug found and fixed by that same testing, not by inspection**: the first run
   streamed cleanly for the full window and then **segfaulted during shutdown**. The C++
   reference's own `close()` — `FT_AbortPipe`, then `FT_ReleaseOverlapped` on every queued
   buffer immediately, no drain in between — is exactly what `UsbTransport::close` had ported
   1:1. On this driver, `FT_AbortPipe` returns before every queued read has actually finished
   cancelling despite its synchronous-looking signature, so releasing immediately could race a
   read still genuinely in flight. Fixed by calling `FT_GetOverlappedResult` (waiting) on each
   buffer between the abort and the release, so the driver has actually finished with it
   first; confirmed fixed across two clean, crash-free runs afterward. Not documented anywhere
   in the D3XX headers — found only because this was run against real hardware rather than
   left as "ported faithfully, should be fine."
8. **Auto-ATT, Serial mode, VHF/preamp switching** — lowest priority; each is real but
   self-contained, and none of them blocks anything else on this list.

## 8. Open questions

- **Answered and built, see `DECORRELATION_PLAN.md`**: sdroxide's manual-null / decorrelation-style
  software combiner exists now (branch `decorrelation`) — both the scalar (whole-block, closed-form)
  half this plan's §4 needs, as `Diversity::decorrelated_weight()`, and a wideband/per-bin half that
  turned out not to apply here (§4). The scalar half is exactly what hardware-diversity's "solve
  from current phasing" step should read from — no separate state to invent, as hoped. It has also
  landed in the RSPduo's own settings UI (`crates/sdroxide-ui/src/app/settings/radio.rs`) as a
  selectable technique, which is worth a look as the template for whatever this plan's own
  RSR200-side controls end up looking like, once §4's hardware-diversity mode is built. Sequencing
  turned out moot rather than needing a decision: the scalar piece landed well before this plan's
  own step 6 (hardware diversity) has been started at all, and before step 4 (Separate mode) too —
  nothing in this plan has begun implementation yet.
- Priority between LAN and USB as the very first milestone — this plan recommends LAN for the
  reasons in §6, but that's a recommendation, not something to lock in without your sign-off.
- Whether the Windows `FTD3XXWU`/WinUSB question in §6 is worth spiking *before* LAN is proven,
  in case it changes how much of the USB phase is genuinely new work versus reachable through
  `nusb` the way every sibling crate manages.
