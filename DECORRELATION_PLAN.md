# Decorrelation for the Diversity module — a plan

Ralph: "We should also consider adding a decorrelation function to the phasing functionality;
that does not appear to be part of the software currently." Confirmed by reading
`crates/sdroxide-dsp/src/diversity.rs` in full: `Diversity` currently offers exactly one
technique — an adaptive NLMS transversal filter, in two arithmetics (`Cancel`/`Combine`).
Decorrelation is a genuinely different technique, not a variation on that one, and it's real,
already-built, already-tested value from the other program this plan (and `RSR200_PLAN.md`
next to it) draws on — not a speculative addition.

This is independent of the RSR200 work: it improves the *existing* RSPduo diversity feature
today, with no new hardware and no new backend. It also happens to be the exact mechanism
`RSR200_PLAN.md` §4 needs for hardware-diversity's "solve, then apply" step, so building it now
pays for itself twice.

## What decorrelation is, and why it's a different tool, not a better `Cancel`

`Diversity`'s NLMS filter *predicts* the main channel from the auxiliary one and subtracts the
prediction — an inherently asymmetric formulation (one channel is "main," fixed; the other is
filtered against it) that has to *adapt* toward an answer over time, sample by sample.

Decorrelation instead builds the 2×2 covariance matrix of the two channels over a window of
samples —

```text
R = [ E[A·A*]  E[A·B*] ]
    [ E[B·A*]  E[B·B*] ]
```

— and eigendecomposes it. For a 2×2 Hermitian matrix this is closed-form (no iteration, no
library needed — a handful of lines of complex arithmetic: trace, determinant, the standard 2×2
eigenvalue formula, then solve for each eigenvector directly). The two eigenvectors are the two
weighted combinations of A and B that are, respectively, **as decorrelated from each other as
this pair of channels allows** (smallest eigenvalue — the null) and **as correlated as
possible** (largest eigenvalue — the combine). One solve produces *both* answers; `Cancel` and
`Combine` fall out of the same computation rather than needing two different filters. This is a
generalization of what `DiversityMode` already offers, not a competitor to it — the honest
framing is "a third technique," not "a replacement for the first two," because the NLMS filter's
own real virtue (continuous adaptation to a drifting scenario without ever needing to re-solve)
is exactly what a one-shot eigendecomposition doesn't do on its own.

**Evidence this is worth building, not just an interesting idea**: the eigendecomposition
approach, extended to solve independently *per FFT bin* rather than once globally, was measured
on real HF recordings (five previously-erratic windows of a real interference case) at **28.8,
30.2, 25.5, 29.1, and 38.0 dB** of nulling depth — every one clear of the single-global-weight
("scalar") method's own **22–26 dB** range on the same material. Those numbers are from the
other program's own signal chain, not a guarantee sdroxide's will reproduce them exactly (worth
re-measuring once ported, not assuming), but they're real, live-air numbers, not a simulation —
strong evidence the *technique* is worth the implementation effort, independent of the exact dB
figures a different codebase's DSP chain will land on.

## Two additions, not one

They're different enough in shape that they should land as two separate pieces of work, in this
order:

### 1. Scalar decorrelation — fits inside `Diversity` as it exists today

One covariance matrix, computed over a whole processed block (or an operator-selected reference
sub-band — see "open questions"), eigendecomposed once per block (or once on demand — see
below), producing an instant complex weight. This slots into the *existing* `Diversity` struct
almost exactly where its own doc comment already says the two current modes differ: "Both modes
run the same adaptive filter and differ in one line at the end." Same `process(&mut self, main:
&mut [Complex32], aux: &[Complex32])` call shape, same struct, no new public type needed for the
covariance solve itself.

**Built** (branch `decorrelation`, `crates/sdroxide-dsp/src/diversity.rs`) as
`DiversityAlgorithm::Decorrelate`, one deviation from the paragraph above worth recording: not a
third `DiversityMode` variant. `Cancel`/`Combine` already say *what* the operator wants; the new
enum is a second, orthogonal axis saying *how* the weight is found (`Adaptive`, the existing NLMS
filter, or `Decorrelate`, the closed-form solve) — cheaper to reason about than a flat third mode,
and it means a `Combine`-style decorrelated output was never a separate follow-up: both fall out
of the one eigendecomposition (`covariance_eigen`) exactly as this document originally argued,
selected by whichever `DiversityMode` is already set.

**A real limitation found while testing, not anticipated above**: applying the raw null
eigenvector to `Cancel` directly — `y = k0·main + k1·aux` with `(k0, k1)` jointly unit-norm — does
*not* have the "a signal `aux` cannot hear survives untouched" guarantee the existing NLMS
`Cancel` arithmetic has by construction. `k0` alone is a data-dependent value less than 1 in
general (verified with `h = 0.6∠0.7°`, no wanted signal at all: `|k0| ≈ 0.86` even for a perfect
null), so the raw eigenvector *attenuates* anything in `main` — wanted signal included — in
proportion to how much of `main`'s own power it represents, since the solve has no way to tell
"predictable from `aux`" apart from "just adds power." In the degenerate case where `aux` is
silent, the null eigenvector for `Cancel` is `(0, 1)` — it zeroes `main` entirely, wanted signal
and all, which the NLMS filter's own `Cancel` (`main − W·aux`, `W → 0` when `aux` is silent) does
not do.

The fix shipped: for `DiversityMode::Cancel` specifically, rescale the solved null eigenvector to
unity gain on `main` before applying it — `main + (k1/k0)·aux`, the same null direction, just
re-anchored so `main`'s own coefficient is exactly 1 regardless of what ends up multiplying `aux`.
That restores the untouched-if-unpredictable guarantee (confirmed by test:
`a_signal_only_the_main_aerial_hears_survives_decorrelation`), at the cost of being undefined
when the solved `k0` is itself (numerically) zero — handled by falling back to outputting `aux`
alone in that case, since there is no rescaling of `main` that reaches an answer when `main` is
the channel being rejected. `DiversityMode::Combine` keeps the raw jointly-unit-norm eigenvector
as solved — maximal-ratio combining has no "leave one branch alone" expectation to preserve, `main`
included, so no rescale applies there.

Practical upshot: `Decorrelate`+`Cancel` wants the noise being nulled to genuinely dominate
`main`'s own power to behave like a proper null rather than a lossy blend — more so than the
adaptive filter needs, since the adaptive filter's asymmetry (predict-and-subtract from a fixed
`main`) has no equivalent notion of "how much of main's power is the wanted signal" to begin with.
Worth surfacing in whatever UI exposes this — a `depth_db()`-style readout matters even more here
than for the adaptive filter, per the "where this lands" section below.

Two ways to expose it, worth deciding rather than defaulting to one:

- **Continuous**: re-solve every block, replacing the NLMS filter's iterative convergence with
  an instant one — appropriate for `Combine`-style diversity reception where the operator wants
  the best current combination, tracked live, no "watch it converge" step at all.
- **One-shot "Solve"**: a button, not a running mode — compute the weight once from the samples
  in flight right now, then hold it (mirroring `Diversity::set_frozen`, and *exactly* mirroring
  `RSR200_PLAN.md` §4's "solve from current phasing, then apply" flow for the RSR200's hardware
  combiner, which needs precisely this: a weight computed once from software state, then handed
  to something else — there, a radio command; here, potentially nothing further at all, since the
  RSPduo has no hardware combiner to hand it to and the solved weight just becomes `Diversity`'s
  own held state).

Both are worth having — continuous for "keep it optimal as conditions drift," one-shot for
"give me tonight's answer and stop touching it" — and both are cheap once the eigendecomposition
itself exists, since they differ only in *when* `process()` (or a new `solve()`) gets called.

### 2. Wideband (per-bin) decorrelation — the actual technical leap, and a new component

Solving the same eigendecomposition **independently per FFT bin**, on an STFT of both channels,
is what actually earns the 28–38 dB numbers above rather than the 22–26 dB a single global weight
manages — a fixed complex weight can null *one* interferer, or find a compromise across several;
a per-bin solve nulls each one in the bin(s) it actually occupies, simultaneously, because
nothing forces the answer to be the same frequency to frequency.

This is not a mode of `Diversity` — it needs its own overlap-add STFT pipeline (block in, window,
FFT both channels, per-bin 2×2 solve, apply, inverse FFT, overlap-add out), which is a
structurally different kind of component from `Diversity`'s simple in-place block filter. The
crate already has the right building blocks to build it *from* — `rustfft` is already a
dependency, and `spectrum.rs`'s own `SpectrumAnalyzer` is a working, reviewable example of this
exact "overlapped windowed FFT with a Blackman-Harris window" shape already in the tree, just
computing power for display instead of a covariance solve for combining. A
`WidebandDecorrelator` would follow the same STFT skeleton and do a different thing with each
bin's pair of complex values.

**The one real, hard-won pitfall, worth building the fix in from the start rather than
rediscovering it**: solved naively, per-bin decorrelation is *unstable* on real air — thousands
of noise-floor bins across the observed span each contribute an essentially arbitrary momentary
direction to the solve, which (via the inverse FFT reconstructing from all of them) shows up as
the null wandering and refusing to hold, even though any *individual* bin with a real signal in
it is stable on its own. The fix that worked: **a per-bin power gate** — exclude a bin from the
solve whenever it sits far enough below the median bin's power (20 dB was the number that worked
on real material; worth treating as a starting point to tune against sdroxide's own signal
chain, not a constant to import unquestioned). A frequency-window/reference-band gate was
considered and rejected in the original work, because it would have been a "point at the
interferer" control by another name and broken exactly the multi-interferer case that's the
whole point of doing this per-bin at all — the power gate keeps the general case general.

**Built** (branch `decorrelation`, `crates/sdroxide-dsp/src/wbdecorrelator.rs`, struct
`WidebandDecorrelator`, exactly as sketched above): a periodic-Hann, 50 %-hop weighted
overlap-add analysis/resynthesis pipeline structurally identical to `WbDdc`'s own (see that
module's doc comment for the reconstruction-gain derivation this reuses, minus its
band-selection/decimation, which does not apply here), reusing `covariance_eigen` and the scalar
piece's own `cancel_weight` (below) per bin. Per-bin covariance is *time-smoothed* across STFT
frames with the same exponential-average idiom `SpectrumAnalyzer` already uses for display power
— a single frame's instantaneous per-bin outer product is always exactly rank one (any lone
sample pair is), which is precisely the naive-and-unstable case described above; smoothing is
what gives the gate something meaningful to threshold. Gate default kept at 20 dB, exposed via
`set_gate_db`.

Two things found while testing this, both worth recording rather than rediscovering:

- **`cancel_weight` is shared with the scalar piece**, not reimplemented — one piece of reasoning
  about the Cancel-mode rescale (see the "real limitation" note above) rather than two copies
  that could quietly drift apart. Building the wideband piece is what actually exercised its
  degenerate-fallback branch hard enough to break it (below), which the scalar piece's own tests
  never happened to reach.
- **The degenerate-fallback threshold was wrong, and per-bin use is what exposed it**: the first
  cut gated the Cancel rescale (`k1 = null.k1 / null.k0`) on `null.k0.norm_sqr() > EPS` — an
  *absolute* floor (`1e-12`) on a *scale-dependent* quantity. A per-bin `null.k0` that is merely
  window-sidelobe-leakage small (not truly zero, but many orders of magnitude below the bin's own
  signal) slips past that floor, and dividing by it amplifies numerical noise into a wild,
  effectively garbage `k1` — which then injects that noise straight into the reconstructed bin. A
  test built to check "an interferer in one bin doesn't touch a wanted tone in a completely
  separate, uncorrelated bin" caught this directly (the tone's own bin has `rbb ≈ 0` and
  `rab ≈ 0` from leakage alone, exactly the failure condition). The fix: bound the *ratio* `k1`
  itself, not `k0` — `k1` is a dimensionless gain ratio between two aerials, so a fixed cap on it
  (60 dB — already an implausible ratio for any real pairing) is scale-invariant in a way a
  threshold on `k0` alone can never be, and catches both a literal divide-by-zero (non-finite
  `k1`) and a merely-tiny-enough-to-be-garbage one in the same check.

Tests (`crates/sdroxide-dsp/src/wbdecorrelator.rs`): identical channels null to >40 dB; a dead
`aux` leaves `main` close to untouched (the per-bin analogue of the scalar piece's own such
test); an interferer confined to one bin is nulled without attenuating a wanted tone confined to
a completely different, uncorrelated bin; freezing holds every bin's weight. `cargo test -p
sdroxide-dsp`: all pass, full crate suite unaffected.

**Real-air verification has now happened** (RSPduo, both tuners, serial 1905037B32, 1130 kHz
local mediumwave broadcast, `2048`-point FFT at 2 Msps) and confirms the core claim this whole
document rests on: scalar decorrelation and the adaptive filter behave the same way on real
interference (one compromise weight for the whole span, same limitation a single-tap analogue
phaser has — "not very useful for the most part," in the tester's own words, matching what the
per-branch trade-off in "Two ways to expose it" above predicted), while decorrelate *per bin*
produced "a massive null vs. strong signal" by ear — the qualitative version of the 22–26 dB vs.
28–38 dB gap this document opened with, now reproduced on different hardware and a different
signal chain than the one the numbers were originally measured on.

One real gap the test surfaced, since fixed: the log's `depth_db()` reads misleadingly shallow
for the per-bin technique — 1.4–2.4 dB on the session that produced an audibly "massive" null.
The reason is exactly what `depth_db()`'s own doc now says: it averages *the whole span* (all
2048 bins), so one narrow, deep null gets diluted by however many of the other ~2000 bins had
nothing to remove at all. Fixed by adding `WidebandDecorrelator::peak_depth_db()` — the single
deepest null among the bins that actually passed the power gate, computed from the same
closed-form output-variance quadratic form `covariance_eigen`'s own tests already verify, just
evaluated at the rescaled Cancel weight instead of the raw eigenvector. `SdrPlaySource::log_depth`
now reports both numbers together (`"N dB peak null (M dB span average)"`), since they answer
genuinely different questions rather than one being a rougher version of the other. Regression
test added in `wbdecorrelator.rs`'s existing one-bin-interferer test: peak reads a real null
(&gt;14 dB) while the whole-span average reads at least 6 dB lower for the identical scenario —
the synthetic version of exactly what real air showed.

## Where this lands, concretely

- **Done**: `crates/sdroxide-dsp/src/diversity.rs` — `DiversityAlgorithm::Decorrelate` (an
  orthogonal axis to `DiversityMode`, not a third variant of it — see the note above) plus the
  2×2 eigendecomposition as a free function, `covariance_eigen`, unit-tested against synthetic
  pairs with hand-computed answers.
- **Done**: `crates/sdroxide-dsp/src/wbdecorrelator.rs`, struct `WidebandDecorrelator` — the
  STFT-based per-bin version, genuinely new infrastructure, not an extension of anything
  existing. Reuses `covariance_eigen` and `cancel_weight` from `diversity.rs` rather than
  duplicating either.
- **Done**: `sdroxide_types::SdrPlayDuo` gained `technique` (`DiversityTechnique`:
  `Adaptive`/`Decorrelate`/`WidebandDecorrelate` — a new, RSPduo-config-level enum, since the
  distinction spans two different DSP *components*, not one setting either takes) and `gate_db`.
  `Rsr200Diversity`, per the other plan, does not exist yet — nothing has started there.
- **Done**: `crates/sdroxide-ui/src/app/settings/radio.rs`'s `settings_sdrplay_tab` — a "How to
  find it" technique selector under the existing mode selector, a gate-dB slider (wideband only),
  Filter length/Adaptation rate (adaptive only), Hold/Restart shared by all three. RSR200's own
  settings tab still doesn't exist (the backend itself hasn't been started — see
  `RSR200_PLAN.md`), but this is now the template for it.
- **Done, then found wanting, then fixed**: a `depth_db()`-style readout existed for the wideband
  technique from the start, but real-air testing showed it alone is actively misleading (see
  above) — `peak_depth_db()` is the fix, and the pair of them together is what actually answers
  "is this doing anything."

## Suggested order

1. **Done. Scalar decorrelation inside `Diversity`** — smallest, self-contained, immediately
   useful on the RSPduo today, and unblocks `RSR200_PLAN.md`'s own hardware-diversity step
   regardless of how much of the rest of that plan has landed yet.
2. **Done, out of order — real-air verification actually happened *after* the wideband version was
   already built and wired into the settings UI, not before it as this list originally suggested.**
   That turned out fine: the wideband implementation's own synthetic tests (one-bin-interferer,
   dead-aux) gave enough confidence to build and ship the UI ahead of hardware time, and real air
   confirmed the core claim once it was available rather than catching a fundamental problem that
   would have been cheaper to find first. Worth recording as a real data point on how load-bearing
   "verify on real air before building the next piece" actually was here — not very, this time —
   without overgeneralizing from a single instance.
3. **Done. Wideband/per-bin decorrelation**, power-gated from the start rather than discovering
   the instability the hard way a second time.

## Open questions

- **Answered and built, 2026-08-25 (branch `rsr200`)**: a global scalar solve over an
  operator-selected reference sub-band — real-air evidence made the decision, not a deliberation
  in the abstract. Ralph ran a direct A/B on 820 kHz (WNYC) with two working antennas: sdroxide's
  whole-span `DiversityAlgorithm::Decorrelate` left far more of WNYC audible, and sounded choppier,
  than the SDR++ sibling's own automatic decorrelate on the identical antennas and frequency — the
  same technique, worse result. Root cause, confirmed by reading the SDR++ implementation directly
  rather than guessing: its own `dsp::combine::RefBand` restricts the covariance measurement the
  solve is based on to a slice of spectrum the operator points at the interferer, so the weight is
  solved from the interferer specifically rather than from whatever the whole span happens to make
  loudest and most correlated. Ported as `sdroxide_dsp::diversity::RefBand` (private to the crate;
  `Diversity::set_ref_band(enabled, sample_rate_hz, offset_hz, width_hz)` is the public surface) —
  two cascaded boxcar decimators, exactly the original's own design, feeding the same `raa`/`rbb`/
  `rab` inputs `covariance_eigen` already took. `Rsr200Diversity`/`SdrPlayDuo` both gained
  `ref_band_enabled`/`ref_band_freq_hz`/`ref_band_width_hz` (`PROTO_VERSION` 101 → 102 — was 96 → 97 before this branch's own version chain got renumbered to sit after upstream's, reconciling the two), exposed in
  both settings tabs as a checkbox + absolute frequency (MHz) + width (Hz) — no "centre on VFO"
  convenience yet (see `RSR200_PLAN.md`'s own note on why: `Diversity::process` runs on raw
  wideband IQ, before any VFO/demod tuning exists to read; superseded the same night, see
  "Reference frequency field removed" below). `sdroxide-dsp`'s new
  `a_reference_band_nulls_the_weak_interferer_the_whole_span_solve_misses` test reproduces the
  failure and the fix synthetically: two interferers at different frequencies and gains, one far
  stronger than the other — the whole-span solve, dominated by the strong one, does a mediocre job
  on the weak one; the reference band, pointed at the weak one specifically, nulls it deep
  regardless of what the strong one is doing elsewhere.

  **Real-air retest the same night: the reference band alone was not the fix.** Ralph tried it at
  widths from 100 Hz to 20 kHz, pointed at WNYC — "essentially no difference," still nowhere near
  SDR++'s own null, and not because a single weight is inherently too weak either: single-frequency
  decorrelation on **two independent implementations** (SDR++ *and* a Perseus22 with its own vendor
  software) nulled cleanly on the same antennas. That ruled out "inherent one-weight ceiling" and
  pointed at something structurally missing from sdroxide's own solve.

  **Found by reading SDR++'s `dsp::combine::decorrelator.h` directly: whitening.** Solving the raw
  covariance for maximum power/minimum variance is biased by whichever channel is noisier —
  `covariance_eigen` had no correction for that at all. Whitening, calibrated from a genuine
  noise-only capture ("point the radio at a quiet channel first"), normalises the two channels to
  equal, uncorrelated noise before solving, removing that bias. Ported the same night:
  `Matrix2`/`raw_eigen`/`inverse_sqrt`/`transform`/`whitened_to_raw` (f64, a second
  eigendecomposition alongside the existing f32 `covariance_eigen` rather than round-tripping
  through that one's own already-conjugated convention twice per calibration) and
  `Diversity::capture_noise(seconds, sample_rate_hz)`/`has_whitening()`/`clear_whitening()`. Two new
  momentary controls per settings tab ("Capture noise (1 s)" / "Clear"), not persisted fields — the
  calibration is receiver-environment-specific and would go stale the moment conditions change, so
  no `PROTO_VERSION` bump for this one. New synthetic test,
  `whitening_finds_the_null_a_channel_noise_floor_mismatch_hides`: two antennas hearing the same
  interferer, one channel's own front-end noise 30× louder than the other's — raw solve finds
  essentially no null (0.9 dB), whitened finds a near-perfect one (53.7 dB), with the solved weight
  landing almost exactly on the true cancellation gain and phase.

  **Confirmed on real hardware the same night**: Ralph captured noise, then nulled WNYC (820 kHz)
  completely, then retuned and nulled a Toronto station on 860 kHz completely too — both after a
  fresh capture on each frequency. A residual "little noisy, with pops" quality remained; ruled out
  as coming from the decorrelation weight itself (Hold made no difference — the weight was
  literally frozen and pops persisted unchanged) and from packet loss (no drops in the debug log
  during a live test). SDR++'s own recording has "some similar popping, to a lesser extent" at the
  same task — most likely a shared, largely inherent characteristic (real atmospheric noise exposed
  once the dominant carrier is gone, or a downstream AGC/audio-chain difference) rather than a
  defect in sdroxide's decorrelation math specifically, though sdroxide's being more pronounced is
  a real, smaller gap worth a closer look another time.

  **Reference frequency field removed; the reference band now follows the VFO transparently.**
  Ralph: "I will never want to decorrelate against a different frequency than the one I'm tuned
  to. The reference frequency adds needless friction." Confirmed by SDR++ itself — its own
  `misc_modules/phasing/src/main.cpp` centres the reference band on the VFO automatically
  ("Centred on VFO: %+.3f kHz"), never taking a typed frequency at all. Solved the
  `Diversity::process`-runs-on-raw-wideband-IQ problem noted above not by teaching `Diversity`
  about tuning, but by pushing the VFO frequency down into the source instead: `IqSource` gained
  `set_vfo_hz(&mut self, hz: f64)` (no-op default), called by a new
  `Engine::poll_ref_band_vfo()` every tick `state.rx_freq_hz()` (RIT-inclusive) actually changes.
  `Rsr200Source`/`SdrPlaySource` track `vfo_hz` and recompute the reference band's offset from it
  in `refresh_ref_band()` on every call, in place of the old `ref_band_freq_hz - center`
  calculation. `ref_band_freq_hz` and `DIV_REFBAND_FREQ_ELEMENT` removed outright from
  `Rsr200Diversity`/`SdrPlayDuo` (`PROTO_VERSION` 102 → 103 — was 97 → 98 before the same renumbering) — safe as a hard removal rather
  than a deprecation because v97 was never released. The "Reference frequency" field is gone from
  both settings tabs; only "Reference band" (enable) and "Reference width" remain.
- Continuous-resolve vs. one-shot-then-freeze as the *default* behavior for scalar decorrelation
  in `Combine`-style use — both are worth having per the section above, but which one an operator
  gets without touching a setting is a real UX choice, not obviously either way.
- Whether the 20 dB power-gate default is even the right starting point for sdroxide's own noise
  floor/AGC chain, which may not match the other program's closely enough to inherit the number
  unchanged. First real-air data point: left at the 20 dB default, on the RSPduo, it produced an
  audibly deep null on a real mediumwave interferer — evidence the default is *usable*, not
  evidence it is *optimal*. No deliberate A/B against other thresholds has happened yet.
