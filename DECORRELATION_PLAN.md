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

Real-air verification (this section's own suggested order, step 2) has not happened — everything
above is synthetic-signal testing only, same caveat as the scalar piece.

## Where this lands, concretely

- **Done**: `crates/sdroxide-dsp/src/diversity.rs` — `DiversityAlgorithm::Decorrelate` (an
  orthogonal axis to `DiversityMode`, not a third variant of it — see the note above) plus the
  2×2 eigendecomposition as a free function, `covariance_eigen`, unit-tested against synthetic
  pairs with hand-computed answers.
- **Done**: `crates/sdroxide-dsp/src/wbdecorrelator.rs`, struct `WidebandDecorrelator` — the
  STFT-based per-bin version, genuinely new infrastructure, not an extension of anything
  existing. Reuses `covariance_eigen` and `cancel_weight` from `diversity.rs` rather than
  duplicating either.
- **Not started**: `sdroxide_types::SdrPlayDiversity` (and any future `Rsr200Diversity`, per the
  other plan) — gains whatever fields expose the two additions to config: at minimum an algorithm
  selector (`Adaptive`/`Decorrelate`) alongside the existing mode selector, a gate-threshold field
  for the wideband version, and — for the one-shot "Solve" flow — probably nothing new at all,
  since freezing a computed weight is already `Diversity::set_frozen`'s (and
  `WidebandDecorrelator::set_frozen`'s) job.
- **Not started**: `crates/sdroxide-ui/src/app/settings/radio.rs`'s `settings_sdrplay_tab` (and,
  later, RSR200's own settings tab) — an algorithm selector, and the wideband version's own
  controls (gate dB, active-bin-count readout) once wired up. A `depth_db()`-style readout is
  exactly as valuable here as it already is for `Cancel`, probably more so, since "is this
  actually working" matters even more once there's no adaptive convergence to visually watch
  happen — both `Diversity` and `WidebandDecorrelator` already expose one.

## Suggested order

1. **Scalar decorrelation inside `Diversity`** — smallest, self-contained, immediately useful on
   the RSPduo today, and unblocks `RSR200_PLAN.md`'s own hardware-diversity step regardless of
   how much of the rest of that plan has landed yet.
2. **Real-air verification against actual antennas** — before touching the wideband version,
   confirm the ported math produces a real, holdable null on real interference here, the same way
   the original work verified synthetically first and then on real air before ever building the
   per-bin version on top of it.
3. **Wideband/per-bin decorrelation**, power-gated from the start rather than discovering the
   instability the hard way a second time.

## Open questions

- Whole-band solve, or an operator-selected reference sub-band? The original work used the
  whole visible/processed span for the wideband version (no reference band needed — that's what
  makes it handle multiple simultaneous interferers without being pointed at any one of them);
  a global scalar solve over a *narrow* band the operator picks might behave differently and is
  worth deciding deliberately rather than defaulting to "whatever's easiest to wire up."
- Continuous-resolve vs. one-shot-then-freeze as the *default* behavior for scalar decorrelation
  in `Combine`-style use — both are worth having per the section above, but which one an operator
  gets without touching a setting is a real UX choice, not obviously either way.
- Whether the 20 dB power-gate default is even the right starting point for sdroxide's own noise
  floor/AGC chain, which may not match the other program's closely enough to inherit the number
  unchanged.
