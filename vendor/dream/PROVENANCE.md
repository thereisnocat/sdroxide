# dream — provenance

`vendor/dream` is a source subset of the **Dream** AM/DRM receiver, the
long-running open-source Digital Radio Mondiale implementation begun at
Technische Universität Darmstadt. It is built into a static library by
`crates/sdroxide-drm`, which wraps it behind a small C API.

| | |
|---|---|
| Upstream | <https://sourceforge.net/projects/drm/> |
| Version | `2.2` (released 2019-05-08, the newest release) |
| Tarball | `dream_2.2.orig.tar.gz` |
| SHA-256 | `f7211ee3c19b42116b6d1f999d45007c1a9e62fee92906aa37d56eb00219ef56` |
| Author | Volker Fischer, Julian Cable, Stéphane Fillod, David Flamand and contributors |
| Licence | GPL-2.0-or-later |

`COPYING`, `AUTHORS` and `README` are upstream's own files, copied unchanged.
GPL-2.0-**or-later** matters: the built binary also links mfsk-core
(GPL-3.0-or-later) and the DeepCW model (AGPL-3.0-only), and only the
"or later" makes that combination possible at all.

Upstream is a Subversion repository with no git remote, which is why this is a
copied tree rather than a submodule — the same situation as `vendor/soapysdr`.
The tarball above is the complete original; what is here is the part that is
built.

## What this links against

`crates/sdroxide-drm` also builds **faad2** (`vendor/faad2`, a git submodule
pinned to `2.11.2`, GPL-2.0-or-later) with `DRM_SUPPORT`, and links it directly.
Dream normally `dlopen`s a `libfaad_drm` at runtime instead, which most systems
do not have — leaving a receiver that acquires the signal, reads the service
label, and plays silence. Building it in is what makes the feature work out of
the box.

## What was removed

Nothing that is compiled. The Qt user interface (`src/GUI-QT`, `src/main-Qt`,
`src/util-QT`), the Android sound backends (`src/android`), the empty
`src/macx`, and the sound-card and console code under `src/linux` and
`src/windows` that the shim replaces — `alsa*`, `jack*`, `ConsoleIO*`,
`shmsoundin*`, `pa_shm_ringbuffer*`, `Sound.*`. `Pacer.cpp` and
`platform_util.*` are kept because they are built. The top-level `debian`,
`windows`, `macx`, `linux`, `DreamTests`, `libs` and the qmake project are not
copied.

## What was added

**`src/sourcedecoders/fdk_aac_dll.h`** is not upstream's. It loads the FDK-AAC
*decoder* at run time instead of linking it, because the Fraunhofer licence
cannot be combined with GPL-3.0-or-later. See `vendor/fdk-aac/PROVENANCE.md`,
and the `fdk_aac_codec.cpp` entry below.

## What is patched

Eighteen files, and no upstream line is edited except as described. Five of
them are there because Dream was written for one receiver on one thread and
sdroxide runs one per radio, on a thread each, fed by whatever a shortwave
broadcast happens to contain.

**`src/sound/sound.h`** — one added branch. Under `USE_SDROXIDE_SOUND` the
`CSoundIn`/`CSoundOut` typedefs resolve to the ring-buffer shims in
`crates/sdroxide-drm/include/sdrx_sound.h` instead of a sound card, and the two
fallback conditions (the null interface, and the Windows mmsystem branch) are
guarded so they no longer win. sdroxide feeds the decoder samples from its own
receive chain, so there is no sound card in this picture at all.

**`src/sourcedecoders/aac_codec.cpp`** — two changes.

1. *A bug fix.* `AacCodec::Decode` copied a fixed `AUD_DEC_TRANSFROM_LENGTH`
   (960) samples per channel out of faad2's internal buffer, ignoring
   `NeAACDecFrameInfo::samples` — and fell off the end of the function without
   returning a value, which is undefined behaviour in its own right. faad2 2.10
   and later report a *successful* decode with `samples == 0` for the first SBR
   frame of a stream, so the copy runs off the end of the decoder's buffer and
   segfaults within seconds of acquiring any real broadcast. It now takes the
   length faad2 reports, bounded by the destination, zero-fills any remainder,
   and returns a proper `EDecError`. This is why upstream Dream 2.2 crashes
   against a current faad2, and the fix is not sdroxide-specific.
2. Under `SDROXIDE_NO_AAC_ENCODER`, the constructor no longer `dlopen`s a FAAC
   *encoder*. sdroxide never transmits DRM, and the probe only produced a
   failure message on stderr.

**`src/sourcedecoders/opus_codec.cpp`** — under `SDROXIDE_QUIET`, the two
messages announcing whether libopus was found are not printed. A library has no
business writing to stderr in a GUI application; sdroxide logs through
`tracing`.

**`src/DrmReceiver.cpp`** — one removed `cerr` line that printed every
enumerated input device name while selecting one.

**`src/sourcedecoders/AudioCodec.{h,cpp}`** — the codec list is per thread, and
building it is serialised.

Upstream shares one list of codec objects across every `CAudioSourceDecoder` in
the program, reference-counted with a plain `int` and no lock. Two receivers
starting at the same moment double-free the vector's storage — reproducible in
seconds under AddressSanitizer, and a SIGSEGV about once in forty tries in an
ordinary build. That is only the first thing to go wrong: `GetDecoder` hands
both receivers the *same* `AacCodec`, which owns one faad2 handle, so one
receiver's `DecClose` frees the decoder state the other is still decoding
through. `thread_local` fixes both, because the invariant already holds — a
Dream receiver may only be touched from the thread that built it, so its audio
decoder and its codec live and die together on that thread. Construction and
destruction still take a mutex, because a codec constructor can `dlopen` a
library and fill a table of *global* function pointers from it. Nothing on the
decode path takes that lock.

**`src/matlib/MatlibStdToolbox.cpp`** — the FFT plan mutex is a function-local
static instead of a raw pointer lazily assigned in `CFftPlans`' constructor.
Two threads building their first plan both saw it null, both allocated, and one
then locked a mutex the other was not using. C++11 guarantees exactly one
thread runs a function-local static's initializer, which is the deferred
construction the original comment ("static initialization of CMutex not working
on Mac OS X") was reaching for.

**`src/datadecoding/DABMOT.cpp`** — the MOT reassembler is bounded, and one
out-of-bounds write is fixed.

Every offset it computes is *segment number × segment size*, and both come
straight off the air: the segment number is a 15-bit field in the segmentation
header and the segment size is whatever the last packet happened to carry.
Nothing upstream checks the product. Two things follow. A corrupt header asks
for an allocation of hundreds of megabytes — and since `CVector::Enlarge` takes
an `int`, past `INT_MAX` the size wraps negative, the vector *shrinks*, and the
copy that follows writes far outside it. Separately, `copylast()` grew the
vector by the last segment's own length while writing at the offset the last
segment's *number* implies, which only coincides when every earlier segment
arrived and was exactly full — one short segment and the write runs past the
end. Both reassemblers (`CReassembler`, `CBitReassembler`) now bound the offset
and size the vector from the offset. The identical bug in
`src/util/Reassemble.cpp`'s `CReassemblerN` is left alone: it is only used by
PFT, which is only reached over an RSCI network input this build never enables.

This matters because it is broadcast data reaching a parser with no bounds
checks: a station carrying a slideshow or an EPG runs thousands of lines that a
plain audio-only station never touches, which is exactly the shape of "it works
on most stations and kills the radio on one".

**`src/datadecoding/DataDecoder.cpp`** — under `SDROXIDE_NO_DATA_FILES`, a
decoded EPG object is no longer written out, and the `fwrite` of an empty
object's `front()` is guarded.

This was the only place in the whole receiver that touched the filesystem, and
it took the filename from the broadcast. With the data directory set to `"."`
— which is what the shim asks for — a station carrying an EPG would create
`./EPG/…` in whatever the host's working directory happened to be, under a
name the transmission chose. sdroxide surfaces no EPG, so there was nothing to
write in the first place.

**`src/datadecoding/journaline/NML.cpp`** — `#include <zlib.h>` made
conditional on `HAVE_LIBZ`, which is how its sibling `DABMOT.cpp` already
guards the same header, and the one function that uses zlib
(`Inflate`, for *compressed* Journaline objects) returns failure when it is not
built in. The caller already handles that — "could not uncompress NML body" —
and uncompressed objects are unaffected.

This is the only thing in the tree that wanted a system library nothing else
here needs. It built anywhere `zlib.h` happened to be installed, which included
every developer machine and the GitHub runner images, and failed on a clean
distribution container; the symbols then resolved only because some unrelated
dependency had pulled `-lz` onto the link line. sdroxide surfaces no Journaline
at all, so the whole question is moot for this build — but it is exactly the
kind of accidental dependency that shows up first in somebody else's release.

**`src/MSC/xheaacsuperframe.{h,cpp}`** and **`src/MSC/aacsuperframe.cpp`** —
both audio super frame parsers are bounded, and one uninitialised local is
fixed. *(This entry was missing: the work went in with the super frame fuzzer
and was never written down here.)*

These sit between the MSC and the audio decoder and do unsigned arithmetic on
lengths the broadcast supplies, reached long before any CRC has vouched for
them. In the xHE-AAC parser a single flipped bit in the 4 bit frame border
count, or in a 12 bit border index, produced a frame size larger than the
payload buffered so far, and the copy loop drained a `deque` past its end —
undefined behaviour that reads through a null block pointer on libstdc++ and
libc++ alike, and the segfault a station on 13730 kHz died of within minutes of
a fade. The parser now rejects a directory that does not fit the super frame,
rejects borders that are out of order or outside the payload, refuses to cut
more bytes than it holds, caps the payload it carries between super frames, and
has a `reset()` that drops that carry so alignment is regained on the next
super frame rather than never. `AACSuperFrame::init` left `numFrames`
uninitialised for a mode and rate pair with no super frame, and
`header()`/`parse()` subtracted SDC-supplied header and CRC lengths from an
SDC-supplied frame length without checking the sign. `init()` also fixes an
upstream bug of its own: `audioParam.AM_MONO ? 1 : 2` tests the *enumerator
constant*, never the field, so `numChannels` was always the same value whatever
the broadcast said.
`crates/sdroxide-drm/src/decoder_tests.rs` fuzzes both parsers through
`sdrx_drm_test_parse_superframe` and keeps the three super frames that used to
crash, by name.

**`src/MSC/xheaacsuperframe.cpp`**, again — an xHE-AAC frame border is counted
from the first byte of the Payload section, not of the super frame.

Upstream subtracted two from every border, "header not in payload". The parser
contradicts itself about that. The two special border values are `0xFFE` for
"two bytes back into the previous super frame's payload" and `0xFFF` for "one
byte back" — and with the subtraction, an ordinary index of `0x000` means
exactly what `0xFFE` already means. A standard does not spell one border two
ways. Without it the encoding runs straight through: `0xFFE`, `0xFFF`, `0x000`,
`0x001`.

The visible cost was two bytes on the *first* frame only, because the
subtraction cancels in the differences between later borders. But those two
bytes are never consumed, so the payload stays two bytes ahead and every frame
after the first is cut short as well, for as long as the receiver stays tuned.
Measured on `FMGold_xHE_ModeB_9khz.flac`: the library's own decode error count
over 65 seconds goes from **220 to 2**, and sixteen half-second dropouts
disappear. The frame *count* is the declared border count either way, which is
why nothing noticed — `xhe_aac_frame_borders_are_counted_from_the_payload` in
`decoder_tests.rs` asserts the frame *sizes* for that reason. Reported and
confirmed on air by several people at
<https://sourceforge.net/p/drm/discussion/general/thread/01c6e64c3b/>; the
argument above is the reason to believe them.

**`src/sourcedecoders/AudioSourceDecoder.cpp`** — the super frame parser's
lifetime, and the output block's bound.

`pAudioSuperFrame` was never initialised, never deleted, and never checked.
`InitInternal()` runs again on every mode, service and audio parameter change
and `new`s a fresh parser each time, so upstream leaked one per change and
dereferenced an indeterminate pointer if an init threw before assigning.
*(That much was also missing from this file.)*

Separately, the conversion loop that writes the decoded block into the output
buffer had no bound at all, and the buffer was sized from "an audio frame
always corresponds to 400 ms". That holds for AAC. For xHE-AAC the number of
audio frames in a super frame is a 4 bit field off the air and their length
comes out of the USAC config — up to 4096 samples each — so the product can
exceed it, and the write ran past the end of the cyclic buffer's write window.
The buffer is now sized for what those two fields can actually express, and the
loop drops what will not fit rather than running off the end.

**`src/sourcedecoders/reverb.{h,cpp}`** — three bugs, one of which xHE-AAC makes
reachable.

`apply()` took its two audio buffers **by value**. Everything it computes — the
fade-out of the last good block, the mute of a bad one, the fade-in on recovery
— was written into the copies and thrown away at the semicolon, so a failed
audio frame re-emitted the *previous* frame's samples instead of falling
silent. By reference, as the code plainly intends.

That alone would have made a marginal signal sound worse, because the three
fade coefficients were `_REAL(i / iResOutBlockSize)` — integer division, zero
for every sample. The fade-out did nothing and the fade-*in* multiplied the
whole of the first good block after every dropout by zero. Now computed in
floating point.

`bAudioWasOK` was initialised nowhere — not in the constructor, not in `Init()`
— and `apply()` reads it on its first call, so which of those three branches
ran on the first block after start-up was whatever the stack held. Initialised
in both, since `Init()` runs again on every service change.

And the old-block buffers were only re-sized when they were *empty*: a
mismatched size disabled the reverb branches but not the fade-out loop or the
copy back to the caller, both of which index them to the new block's length.
AAC never grows — its frame length is fixed — but xHE-AAC takes its from the
USAC config and starts at zero, because the frame count is not known until the
first super frame has been parsed. They are re-sized on any change now.

**`src/sourcedecoders/fdk_aac_codec.{h,cpp}`** — the xHE-AAC decoder: loaded at
run time, decoder only, restricted to xHE-AAC, and given an output buffer it
can actually use.

*Loaded at run time* through the added `fdk_aac_dll.h`, because the Fraunhofer
licence cannot be combined with this project's — see
`vendor/fdk-aac/PROVENANCE.md`. `CanDecode` answers false when no library was
found, so an xHE-AAC service degrades to Dream's null codec exactly as an Opus
service does without libopus.

*Decoder only*: `SDROXIDE_NO_AAC_ENCODER` removes the include of
`aacenc_lib.h`, the `hEncoder` member and the bodies of all six encoder
methods, which become inert stubs because `CAudioCodec` declares them pure
virtual. sdroxide never transmits DRM; DRM is a broadcast system with no
amateur allocation.

*Restricted to xHE-AAC*, and this one is not cosmetic. `InitCodecList` puts
this codec **ahead** of `AacCodec`, and upstream's `CanDecode(AC_AAC)` tests
three capability bits that belong to the SBR module against the flag word of
the *AAC decoder* module — `CAPF_SBR_DRM_BS` is `0x4`, which in that word means
`CAPF_ER_AAC_SCAL`, and `CAPF_SBR_PS_DRM` is `0x40`, which means
`CAPF_AAC_960`. All three are set unconditionally by any fdk-aac 2.x, so the
function answers yes, and merely installing libfdk-aac would have moved every
ordinary DRM broadcast off the statically linked faad2 this receiver was
verified against. It now answers only for `AC_xHE_AAC`.

*An output buffer it can use.* `Decode` passed `frameSize * numChannels` as the
`timeDataSize` argument of `aacDecoder_DecodeFrame`, which the library
documents as the **capacity** of the output buffer — and took those two fields
from a `CStreamInfo` read *before* the frame was decoded. `aacDecoder_GetStreamInfo`
describes the last decoded frame, and a USAC stream reports zero channels until
one has been decoded, so xHE-AAC offered a zero sample buffer on every frame,
got `AAC_DEC_OUTPUT_BUFFER_TOO_SMALL` back on every frame, and produced silence
from a signal it had otherwise decoded perfectly — the service label, the bit
rate and the scrolling text all arrive. Plain AAC gets away with it because its
frame size is right by the second frame. The capacity is now the buffer's own,
and the stream info is read after the decode.

Four dereferences a fading broadcast reaches are also guarded: a null decoder
handle, an empty audio frame (a super frame directory may place two borders at
the same byte, and `&audio_frame[0]` on an empty vector is undefined), a null
`CStreamInfo`, and a frame size or channel count that would overrun
`decode_buf`. `aacinfo`'s `LIB_INFO` array is `FDK_MODULE_LAST` long rather
than a round 12, because each module the library chains through scans the
caller's array to that length looking for a free slot. `DecOpen` closes any
handle it is about to replace, and prefers `CStreamInfo::sampleRate` — the rate
of the PCM the decoder actually returns, and the field that goes with the
`frameSize` the copy uses — over `extSamplingRate`, an ASC field which for USAC
is often zero and which sent the resampler ratio wrong without saying so.
Finally, under `SDROXIDE_QUIET` the file's `cerr` goes to a null sink: it
narrates every decode, and on a fading signal a line per frame.
