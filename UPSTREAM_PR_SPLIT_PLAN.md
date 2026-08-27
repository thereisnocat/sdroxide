# Splitting the fork into two clean upstream PRs: decorrelation, and RSR200 support

## The strategic call this plan is executing

Ralph's framing, recorded verbatim because it's the reasoning everything below serves: *"sdroxide
is under active development by the upstream developer. Given that, chasing after their changes
while also implementing ours is probably a loser's game, involving regularly merging their
upstream changes into our code base and refactoring, updating PROTO_VERSIONs and the like. As the
number of features we add grows, the overhead of maintaining that increases."* Confirmed real,
not assumed: `dividebysandwich/sdroxide` is genuinely active (37 open issues, PRs merged as
recently as two days before this plan, `#175`/`#149`/`#115`/`#114`/`#113`) — this fork already
paid that exact cost once, reconciling a 34-commit gap in a single session
(`7037df2`). Submitting real, reviewable PRs upstream — rather than maintaining a permanently
diverging fork — is the sound alternative to either repeating that cost indefinitely or
abandoning the work. This plan is the mechanics of doing that for the two features that exist:
decorrelation and RSR200 support.

## The complication found by actually checking, not assumed: RSR200 depends on decorrelation

The two features are not independent. `Rsr200Diversity` (`crates/sdroxide-types/src/radio.rs`)
has a `technique: DiversityTechnique` field from the moment RSR200 Separate mode was built
(commit `2acb8ef`, RSR200_PLAN.md step 4) — `DiversityTechnique` is decorrelation's own type.
RSR200's software-diversity path was *deliberately* built to reuse `sdroxide_dsp::Diversity`
rather than duplicate it — "no changes to `sdroxide-dsp` needed, genuine reuse," per that step's
own commit and `RSR200_PLAN.md`'s account of it. Concretely: **RSR200 support does not compile
without decorrelation's `DiversityTechnique`/`WidebandDecorrelator`/whitening code already
present.** This isn't a flaw to route around — it's the actual shape of the work — but it means
the two PRs are sequenced, not parallel: decorrelation first (or at minimum, RSR200 built as a
stack on top of it), not two independently-orderable submissions.

## What's in each PR, verified by walking the actual commits rather than guessed at

### PR 1: Decorrelation (scalar + wideband, on both LimeSDR-family and SDRplay RSPduo)

Original build, already the `decorrelation` branch's own content (8 commits, `b5264d6`..`ff28ab9`,
merged into local `main` at `2e1da2d`):

- `b5264d6` — scalar decorrelation (`DiversityAlgorithm::Decorrelate`) in `sdroxide-dsp`.
- `6a3a9f1` — wideband per-bin decorrelation (`WidebandDecorrelator`).
- `49e4333` — wiring both into the RSPduo's own settings/UI.
- `175cf65` — `WidebandDecorrelator::peak_depth_db`.
- `cc9e785`, `51a5033`, `ff28ab9` — doc/plan updates alongside the above.
- `01e29df` — unrelated (Opus cmake fix that rode along); drop when rebuilding the branch fresh.

Plus the six commits that exist **only on `rsr200`**, found by walking `decorrelation..rsr200`'s
own history for anything touching `crates/sdroxide-dsp/src/diversity.rs`/`wbdecorrelator.rs` — the
real fixes, not yet on `decorrelation` at all:

- `545da3c` — reference-band restriction for Decorrelate.
- `29d6adc` — fix: silence forever if Hold was already on before ever solving.
- `f7b7e6d` — **whitening** — the fix that actually made real-antenna nulling work (WNYC and a
  Toronto station both fully nulled, from "essentially no difference" beforehand). This is the
  one commit in the whole fork most worth getting upstream; everything else here supports it.
- `5006d88` — doc: real-hardware confirmation.
- `15ea26b` — reference band follows the VFO transparently (drops a separate typed frequency).
- `ccd4699` — doc cleanup for the above.

**Ralph's own question — "possibly also including changes we made to the SDRPlay RSPduo code to
support it" — confirmed yes, and necessarily.** The whitening/reference-band/VFO-tracking work
touched `src/sdrplay_source.rs` and `SdrPlayDuo`'s own fields (`technique`, `gate_db`,
`ref_band_enabled`, `ref_band_width_hz`) directly — there's no way to submit the DSP fix without
its RSPduo wiring, since that's the only backend any of this has been tested against on real
hardware so far. RSR200's *own* copy of the same fields (`Rsr200Diversity`) stays out of this PR
entirely — it belongs in PR 2, and depends on PR 1's types existing first.

### PR 2: RSR200 support

Everything RSR200-specific: `6274f9f` through `0ccef3f` (protocol, device/LAN, USB transport
including the Windows D3XX bindings confirmed on real hardware, Separate mode, hardware
diversity, 24-bit/status, step 8's Auto-ATT/Serial/VHF/swap-channels), plus the real bugs found
and fixed along the way (`1a8aa4d` Serial-mode scrambled audio, `c052782` VHF frequency display).
Depends on PR 1's `DiversityTechnique` for its own Separate-mode diversity — built *on top of* PR
1's branch, not independently.

### Explicitly not going into either PR

- The desktop double-click-to-type-a-frequency fix (`208a444`) — real, but a general UI fix with
  nothing RSR200- or decorrelation-specific about it; a candidate for its own tiny PR, or to drop
  if upstream has since fixed the same gap another way.
- `IQ_RECORDER_PLAN.md`/`IQ_PLAYBACK_PLAN.md` — plans, not yet implemented; not part of this split
  at all.
- The 34-commit upstream-reconciliation merge itself (`7037df2`) and everything it pulled in —
  that's exactly the *other side* of the problem this plan exists to stop needing again. The new
  PR branches start from a fresh `upstream/main`, not from anything already reconciled once.

## Construction: fresh branches off `upstream/main`, not off the fork's own tangled history

Cherry-pick, not merge, and start from `upstream/main` refetched at the moment this actually gets
built — "under active development" means the commit this plan's research used (`PROTO_VERSION`
94, tip `30a6c12`) may already be behind by the time this runs, the same lesson the 34-commit
reconciliation just taught directly.

1. `git fetch upstream && git checkout -b upstream-pr-decorrelation upstream/main`.
2. Cherry-pick the 7 real `decorrelation`-branch commits (dropping `01e29df`), then the 6
   `rsr200`-only DSP/RSPduo commits, in the order above — chronological order matters here since
   later commits assume earlier ones' types exist. Expect the same `SdrPlayDiversity`→
   `SdrPlayDuo` collision this session's own upstream reconciliation already hit and resolved once
   (upstream renamed it in `885dbdb`, after every commit in this list was originally written
   against the old name) — same fix, already known: rename references, keep the added fields.
   `PROTO_VERSION`: renumber against upstream's own current tip at cherry-pick time, not the
   fork's diverged 95–103 — these are new contributions to upstream's own version lineage, not a
   continuation of the fork's local one.
3. Full build, full test suite, clippy — clean, the same bar every change in this fork has been
   held to — against this fresh branch specifically, not assumed to still hold from the fork.
4. `git checkout -b upstream-pr-rsr200 upstream-pr-decorrelation`, cherry-pick the RSR200-only
   commits on top. Same verification pass, including the Windows target check
   (`cargo check -p sdroxide-rsr200 --target x86_64-pc-windows-gnu`) this crate has relied on
   throughout since there's still no local Windows machine to build on directly.
5. Write real PR descriptions from scratch rather than pasting `DECORRELATION_PLAN.md`/
   `RSR200_PLAN.md` in — those are working notes in a conversational, first-person-plural,
   dated-log style meant for this fork's own history, not upstream's PR-description conventions.
   The `.md` plan files themselves are still worth offering as supplementary design docs (real
   engineering record, hard-won findings like the whitening fix's whole diagnostic path) — as an
   attached/linked reference, not the PR body itself.

## Open questions, not resolved here

- **Submit PR 2 only after PR 1 actually merges upstream, or as a stacked/dependent PR
  immediately?** Waiting is cleaner for the maintainer to review (nothing to review out of
  order); stacking is faster but means PR 2 sits un-mergeable until PR 1 lands, and needs
  rebasing if PR 1 changes during review. Ralph's call, not decided here.
- **Whether to open an issue first** describing the whitening finding before the PR, the way
  several of upstream's own recent merges reference an issue number — matches the project's own
  visible convention, not required by anything technical.
- **The standalone frequency-dial double-click fix** (`208a444`) — its own tiny PR, dropped
  entirely, or folded into whichever of the two lands first as an unrelated drive-by fix (the
  last option is generally poor PR hygiene and probably worth avoiding).
