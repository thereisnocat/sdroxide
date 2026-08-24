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
2. **LAN transport + `device.rs`**, single channel, 16-bit — first light. Proves command
   sequencing, the not-synchronous-acknowledgement handling, block resync/framing.
3. **`Backend::Rsr200` registration**: config struct, `open_rsr200_source()`, settings tab —
   even a minimal one, to get real spectrum on screen and close the loop on whether everything
   above actually works against the real radio, not just against a protocol-level test harness.
4. **Separate mode + `sdroxide_dsp::Diversity` wiring** — the part this plan exists to answer
   the question about. Prove it against real antennas the way the RSPduo work already did.
5. **24-bit, decimation range, GPS discipline/correction readout** — each is a small, mostly
   independent addition to `device.rs`/`protocol.rs` once the above is solid.
6. **Hardware diversity (§3/§4's third mode)** — needs Separate mode's own solve-from-software
   step to be meaningful, so it belongs after step 4, not before it.
7. **USB transport** — its own phase, per §6, including the Windows research spike before
   deciding the implementation approach.
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
