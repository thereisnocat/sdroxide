# fdk-aac — provenance

`vendor/fdk-aac` holds **five public headers of the Fraunhofer FDK AAC codec
library, and nothing else**. No source, no library, and nothing here reaches
the linker. They exist so that `vendor/dream/src/sourcedecoders/fdk_aac_codec.cpp`
— the only xHE-AAC (USAC) decoder there is — can be compiled, while the library
it calls is looked up at run time instead of being linked in.

| | |
|---|---|
| Upstream | <https://github.com/mstorsjo/fdk-aac> |
| Version | `2.0.3` (tag `v2.0.3`, `716f4394641d53f0d79c9ddac3fa93b03a49f278`) |
| Author | Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. |
| Licence | Software License for The Fraunhofer FDK AAC Codec Library for Android (`NOTICE`) |

| File | SHA-256 |
|---|---|
| `include/fdk-aac/aacdecoder_lib.h` | `0c13c094ca7c3756e6c0c9d0872d1fb72ad869bd770e998813fa35103db75725` |
| `include/fdk-aac/FDK_audio.h` | `956ecbfc03c77b43100640c09fadb846d6d03bf13c94bce3693af8cf0dcfd9b8` |
| `include/fdk-aac/genericStds.h` | `cd8d8fae2a8546850a4765511cab97a12f45fb4fbe8fe8a8502ceaf0688ecfeb` |
| `include/fdk-aac/machine_type.h` | `3997ac1dbdc9c6efc4ce1225345415bff10298cc5282a8f9b604de288ceee147` |
| `include/fdk-aac/syslib_channelMapDescr.h` | `d2a7cfd231346ba58b3b967956ef43d3076227df7224c046ccf4eb143da048c4` |
| `NOTICE` | `95ec80da40b4af12ad4c4f3158c9cfb80f2479f3246e4260cb600827cc8c7836` |

Those five are the whole include graph: `aacdecoder_lib.h` pulls in the other
four and nothing beyond `<stddef.h>`, `<sys/types.h>` and `<assert.h>`. Each
carries the complete licence text in its own header comment, which is what the
licence asks be retained, and `NOTICE` is upstream's own copy of it.

## Why the library is not linked

The FDK licence permits redistribution of source and binaries, but it forbids
charging a copyright licence fee and it grants **no patent licence at all**.
Both are restrictions the GNU GPL does not permit to be added, which is why
Debian ships `libfdk-aac2` in `non-free` and why ffmpeg has to be built
`--enable-nonfree` to use it. sdroxide is GPL-3.0-or-later, so a binary with
FDK-AAC linked into it could not be distributed.

So it is not linked. `vendor/dream/src/sourcedecoders/fdk_aac_dll.h` loads the
seven decoder entry points through `CLibraryLoader` at run time, the way Dream
already loads libopus, and `FdkAacCodec::CanDecode` answers `false` when there
is no library — which leaves an xHE-AAC service falling back to Dream's null
codec, exactly where this build stood before. **No Fraunhofer object code is
produced by this build**, and `crates/sdroxide-drm/build.rs` emits no
`-lfdk-aac`.

## Why headers rather than hand-written declarations

`CStreamInfo` is read through a pointer the library returns — `frameSize`,
`numChannels`, `sampleRate`, `aot` and `flags` are all taken from the middle of
a twenty-member struct. A member transposed by hand would not fail to link; it
would return a plausible wrong integer with no diagnostic at all. Taking the
layout from Fraunhofer's own header removes the question.

These four `libSYS` headers are byte-identical across 2.0.2, 2.0.3 and current
`master`, and `aacdecoder_lib.h` differs between 2.0.3 and master only in a
comment — so this is not a version-tracking liability.

## One thing not to do

`FDK_audio.h` contains three `static inline` functions — `FDKinitLibInfo`,
`FDKlibInfo_getCapabilities` and `FDKlibInfo_lookup`. They are the only
executable code anywhere in this directory. **Do not call them.** Doing so
would compile Fraunhofer code into the binary, which is the whole thing this
arrangement exists to avoid. `fdk_aac_codec.cpp` already uses `memset` where it
would otherwise have used `FDKinitLibInfo`.

## Which library is loaded

Version 2 only. `libfdk-aac.so.1` is fdk-aac 0.1.x, which has no USAC decoder
and whose `CStreamInfo` is not the struct these headers describe, so loading it
would read the wrong fields rather than fail cleanly. See `FdkAacLibNames` in
`fdk_aac_dll.h` for the per-platform list.
