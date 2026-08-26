# rax25 — provenance

`crates/sdroxide-ax25` carries code from **rax25**, an AX.25 connected-mode
implementation in Rust by Thomas Habets.

| | |
|---|---|
| Upstream | <https://github.com/ThomasHabets/rax25> |
| Commit | `d67469166f8c34f380a2e2c4bc15c0a50c9ff897` (2026-05-29, crates.io 0.2.6) |
| Author | Thomas Habets `<thomas@habets.se>` |
| Licence | MIT |

**The upstream repository ships no `LICENSE` file.** MIT is declared in its
`Cargo.toml` (`license = "MIT"`) and in the crates.io metadata, and nowhere
else. `crates/sdroxide-ax25/LICENSE` carries the standard MIT text with Thomas
Habets named as the copyright holder, which is the licence he granted; it is
reproduced by us rather than copied from him, and this note exists so nobody
later mistakes it for an upstream file.

The crate is MIT for that reason, unlike the rest of this workspace, which is
GPL-3.0-or-later. MIT is inbound-compatible with GPL, so the built binary is
GPL as before; keeping this one crate under upstream's terms means upstream's
code stays under upstream's terms.

## Why vendored rather than depended on

`rax25`'s `state` module is a pure, I/O-free `Event` → `Action` machine, which
is exactly the shape needed to drive from an audio-thread tick. Three things
argued against taking it as a crates.io dependency:

1. **Dependency weight.** `clap`, `regex`, `bus` and `anyhow` are all
   non-optional in its manifest, and `default = ["serial"]` pulls `serialport`.
   A link layer has no business dragging an argument parser into the build.
2. **We need to change it.** Upstream's own README records REJ and SREJ as
   "untested / probably broken". Over a KISS TNC on a good VHF channel that
   rarely shows; over a fading HF path at 300 baud it is the difference between
   a session and a stall.
3. **It assumes KISS.** Everything below a bare frame — flags, bit stuffing,
   NRZI, the FCS — is done by the TNC's firmware upstream. We own the modem, so
   that layer had to be written here regardless.

## What was taken

| Here | From | Changes |
|---|---|---|
| `src/fcs.rs` | `src/fcs.rs` | The 256-entry CRC-16/X.25 table, verbatim. `fcs()` reworked to expose the pre-XOR register so a receiver can check by residue; `check()` and the tests are ours. |
| `src/addr.rs` | `src/lib.rs` (`Addr`) | `regex` validation hand-rolled to the same grammar, `^[A-Z0-9]{3,6}(?:-(?:[0-9]|1[0-5]))?$`. SSID stored as a field instead of re-parsed on each `serialize`, removing an `unwrap` whose safety rested on validation done elsewhere. `anyhow` → `Ax25Error`. Fields are `pub(crate)` because upstream had `Addr` and `Packet` in one module and we do not. Tests ours. |
| `src/packet.rs` | `src/lib.rs` (`Packet`, `PacketType`, the frame structs, the control-field constants) | `anyhow` → `Ax25Error`. The `USE_FCS` branches removed rather than left compiled out: `crate::hdlc` owns the FCS, checks it and strips it, so a frame reaching this module has already passed. **`parse` cannot panic** and **an I frame keeps its PID** — see below. `Packet::ui_via` and the `src`/`dst`/`digipeaters`/`command_response` accessors are ours, for the monitor and for APRS. Specification section numbers kept — they are the most useful comments in the file. |
| `src/port.rs` | *(new)* | The thread bridge between the link and a byte-stream consumer, and the lease that decides whether the operator's terminal or a forwarding session is driving it. Not vendored — upstream's equivalents are its `sync`/`async` I/O drivers, which we deliberately did not take. |
| `src/state.rs` | `src/state.rs` | Kept as close to verbatim as it compiles, **including upstream's tests and its commentary on the specification's bugs** — that commentary is the most valuable thing in the file and the hardest to reconstruct. `anyhow::Result` → a two-parameter `Result<T, E = Ax25Error>` alias (anyhow's own shape, so upstream's `Result<(), std::fmt::Error>` in a `Display` impl still compiles), `log` → `tracing`, imports follow this crate's module split, and the `clap::ValueEnum` derive on `Experiment` is dropped. `State` gains a `Send` bound (the engine holds the controller as `Box<dyn DigiEngine: Send>`), `Data` gains `peer()`, `set_accept_incoming()`, `set_via()`/`via()`, `maxframe()`, `retries()`, `unacked()` and `pending_out()` accessors, the peer-unwrap panic below is fixed, the digipeater path and the window default changed as described below, and the 20 `eprintln!`s became `debug!` (they run on the audio thread, once per link-layer error and once per T1 retry). The state transitions themselves are untouched. |
| `src/kiss.rs` | `src/lib.rs` (`escape`/`unescape`) | Rewritten around a streaming `Decoder`, with the command nibble decoded. **`unescape` returns a `Result` instead of panicking** — see below. |

Everything upstream had is now vendored except the I/O drivers, which we do not
want.

### The module split, and what it cost

Upstream keeps `Addr`, `Packet` and the frame structs in `lib.rs` and puts the
state machine in a child module, so `state.rs` reaches their private fields by
ordinary Rust privacy — a child module can see its ancestors' privates. Split
into sibling modules, that stops working, so those fields are `pub(crate)` here.
That is the same visibility upstream had in practice; it just has to be said out
loud now.

The codec is exercised against frames that came off Direwolf's own
transmission, not merely against frames built here — see
`crates/sdroxide-digi/tests/packet_interop.rs`. The state machine is covered by
upstream's own tests, carried across and passing.

## What was *not* taken, and why

* **`src/sync.rs`, `src/async.rs`** — the I/O drivers. Upstream's own README
  says "the sync API is not great" and the async one wants Tokio for a single
  connection. Our equivalent is a channel pair driven from the engine tick.
* **`src/pcap.rs`** — writes pcap files. Interesting, not needed.
* **`BusHub`, `BusKiss`, `Kiss`** — the `bus`/`serialport` plumbing.
* **`parse_duration`** — a `clap` argument helper.
* **`regex`-based callsign validation** — replaced by a hand-rolled check, to
  avoid a regex engine in the dependency graph for one pattern.

## Things found in upstream that mattered

Recorded because they changed what we wrote, and because anyone diffing against
upstream later will want to know they were deliberate.

1. **The FCS has never run upstream.** `lib.rs:39` is
   `const USE_FCS: bool = false;`, both call sites are behind `if USE_FCS`, and
   the parser carries a literal `// TODO: check FCS.`. This is entirely
   reasonable there — a KISS TNC computes and checks the FCS itself — but it
   means the table arrived here untested by its author. It is load-bearing for
   us on every frame in both directions, so `src/fcs.rs` is tested against the
   published CRC-16/X.25 check value (`0x906E` over `123456789`), against every
   single-bit error in a real frame, and against byte-order reversal. A
   round-trip test would have passed with the wrong polynomial.
2. **`unescape` panics on malformed input** (`panic!("TODO: kiss unescape
   error…")`, and an `assert!` on a trailing escape). Reasonable where the bytes
   come from TNC firmware over a serial cable; ours come from whatever connected
   to a TCP port, and a panic would take the operator's receiver down mid-QSO
   with no explanation. `kiss::unescape` returns a `Result`, and a frame that
   fails to unescape is dropped without stopping the stream.
3. **Every packet-emitting action unwraps `data.peer`, and Disconnected has
   none.** `Disconnected::ui` and `Disconnected::disc` both answer a received
   frame with a DM, and neither handler is given the sender's address — so they
   *cannot* record a peer, and the DM's `dst: data.peer.clone().unwrap()`
   panics. Any UI or DISC frame from any station on a shared channel takes the
   process down. Upstream does not hit it because a KISS client is usually
   pointed at one peer; we monitor everything on the channel.
   `state::handle` now drops a response it cannot address, and
   `sabm_and_sabme` records the sender before refusing with a DM so the caller
   is told no instead of waiting out a timeout. This is the case the "we need
   to change it" argument above was about.
4. **`able_to_establish` defaults to `false`** and upstream's sync and async
   server paths both set it explicitly, with a `// TODO: implement some sort of
   listen()`. Surfaced here as the `packet_accept_incoming` setting, off by
   default — a Winlink client dials out and should not answer calls.
5. **`Addr::serialize` unwraps the SSID parse.** Safe as written because
   `Addr::new` validates first, but the invariant lives a long way from the
   `unwrap`; ours makes the SSID a field rather than re-parsing the string.
6. **`Packet::parse` panics on a frame it does not like** — `todo!()` on an
   unimplemented U-frame control field, and unchecked slices for the digipeater
   path, the PID and the payload. Safe against a TNC's vetted output; ours
   parses every frame heard on the channel, on the audio thread, *and* every
   frame sent, including the verbatim bytes a KISS host handed over. One
   corrupted frame in 65536 passes a 16-bit check sequence by chance, so on a
   busy channel this is a receiver that dies after an afternoon from somebody
   else's traffic. `parse` now returns errors, bounds the address path at eight
   digipeaters, and is swept over every single-bit flip and every truncation of
   two real frames.
7. **An I frame's PID is discarded**, hard-coded to `NO_L3` with a
   `// TODO: confirm pid is NO_L3` beside it. The byte is already read and
   skipped over, so keeping it costs nothing — and it is the only thing that
   tells a terminal's text from a NET/ROM routing header or a segment of a
   longer frame, which would otherwise be printed as line noise.
8. **Every emitted frame is direct and says mod-8.** All eight packet-emitting
   arms of `state::handle` carried `digipeater: vec![]` and `rr_extseq: false`.
   The first makes a digipeated link impossible — a connect goes out addressed
   to a station the far end cannot hear and retries N2 times against silence,
   which is most of the reason a BBS two hops away is unreachable. The second
   means our own extended traffic reads back as mod-8 through the very
   heuristic upstream's own parser offers. Both are now the link's own, in one
   `addressed()` helper, so the fact is written once rather than eight times.
9. **There is no T2.** The acknowledgement timer is a `// TODO: self.t2.set(3000)`
   in both `set_version_2` and `set_version_2_2`, and no timer exists to set. A
   receiver therefore never acknowledges on its own: it answers a poll, and
   otherwise waits. Against a real TNC this works, because a TNC polls on the
   last frame of its window and our `check_need_for_response` answers a P=1
   immediately — but between two sdroxide stations a sender that fills its
   window stalls until *its* T1 expires, which is seconds. Left as it is: it
   costs throughput on long transfers and nothing else, and a third timer inside
   the vendored machine is a change that cannot be tested here against the gear
   it would have to interoperate with. `paclen_and_maxframe_bound_the_i_frames_that_go_out`
   is sized to one window because of it.
10. **The window depends on who dialled.** `Data::new` sets `k: 7`, and
   `set_version_2` sets `k: 4` — but only on a *received* SABM;
   `Disconnected::connect` does not call it. So a link upstream dialled ran a
   window of seven and one it answered ran four, the same two stations behaving
   differently depending on which called. **This is a behaviour change, not a
   port:** `Data::new` now starts at 4, the specification's mod-8 figure and
   the one Linux uses, and an operator's `maxframe` preference is kept in
   `k_pref` and re-applied inside `set_version_2`/`set_version_2_2` — those run
   again on every received SABM(E), including a reset mid-session, so a
   preference applied once would silently go back to the default.

## Upstream references

`state.rs` cites the AX.25 specification (1998, 2006 and 2017 revisions) and
resolves its ambiguities against Direwolf and `ax25embed`. Those citations are
kept in place in the vendored file — they are the most valuable comments in it.
