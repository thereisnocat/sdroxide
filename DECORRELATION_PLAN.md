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
run the same adaptive filter and differ in one line at the end." A third `DiversityMode::
Decorrelate` variant does the same job with a fundamentally different (non-adaptive, closed-form)
computation instead — same `process(&mut self, main: &mut [Complex32], aux: &[Complex32])`
call shape, same struct, no new public type needed for this half.

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

## Where this lands, concretely

- `crates/sdroxide-dsp/src/diversity.rs`: extend `DiversityMode` with `Decorrelate`; add the 2×2
  eigendecomposition as a free function or an associated method (pure math, easily unit-tested
  against synthetic pairs with a known, hand-computed answer — no hardware, no FFT, no async, the
  cheapest kind of test to write and the kind worth writing first).
- New file, `crates/sdroxide-dsp/src/wbdecorrelator.rs` (or similar — `wbddc.rs`/`wbspectrum.rs`
  are the existing "wideband X" naming precedent in this crate): the STFT-based per-bin version,
  genuinely new infrastructure, not an extension of anything existing.
- `sdroxide_types::SdrPlayDiversity` (and any future `Rsr200Diversity`, per the other plan):
  gains whatever fields the two additions need — at minimum a mode selector wide enough for a
  third value, a power-gate threshold for the wideband version, and — for the one-shot "Solve"
  flow — probably nothing new at all, since freezing a computed weight is already
  `Diversity::set_frozen`'s job.
- `crates/sdroxide-ui/src/app/settings/radio.rs`'s `settings_sdrplay_tab` (and, later, RSR200's
  own settings tab): a third mode option, and the wideband version's own controls once it exists
  — a `depth_db()`-style readout is exactly as valuable here as it already is for `Cancel`,
  probably more so, since "is this actually working" matters even more once there's no adaptive
  convergence to visually watch happen.

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
