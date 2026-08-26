//! Builds the vendored Dream DRM receiver, faad2 and the C++ shim between them
//! and Rust, then generates bindings for the shim's C API.
//!
//! Everything is compiled straight with `cc` rather than through Dream's qmake
//! project or faad2's CMake. Neither upstream build system is usable here —
//! Dream's needs Qt even for its console build, and going through a generated
//! makefile is what broke the Windows CI job for the other vendored C in this
//! tree (see `vendor/rade_c`). One source list and two `cc::Build`s avoid both.
//!
//! Three dependencies Dream normally takes from the system are not taken here:
//!
//! * **fftw3** — `include/fftw3.h` plus `src/fftw_compat.c` provide the six
//!   entry points Dream's matlib uses.
//! * **libsndfile, speexdsp, pcap, hamlib, gps, Qt** — all optional in Dream,
//!   all left out, which is what the missing `HAVE_*` defines below select.
//! * **faad2 at runtime** — Dream dlopens `libfaad_drm.so.2`, which most
//!   systems do not have. It is built in from `vendor/faad2` instead, so DRM
//!   audio decodes out of the box, which is the whole point of the feature.

use std::path::{Path, PathBuf};

/// Dream's own source list for a console-mode receiver, taken from the
/// `SOURCES` block of upstream's `dream.pro`, minus `resample/speexresampler.cpp`
/// (guarded everywhere by `HAVE_SPEEX`, which would pull in libspeexdsp).
const DREAM_SOURCES: &[&str] = &[
    "AMDemodulation.cpp",
    "AMSSDemodulation.cpp",
    "chanest/ChanEstTime.cpp",
    "chanest/ChannelEstimation.cpp",
    "chanest/IdealChannelEstimation.cpp",
    "chanest/TimeLinear.cpp",
    "chanest/TimeWiener.cpp",
    "creceivedata.cpp",
    "ctransmitdata.cpp",
    "datadecoding/DABMOT.cpp",
    "datadecoding/DataDecoder.cpp",
    "datadecoding/DataEncoder.cpp",
    "datadecoding/epgutil.cpp",
    "datadecoding/Experiment.cpp",
    "datadecoding/Journaline.cpp",
    "datadecoding/journaline/crc_8_16.c",
    "datadecoding/journaline/dabdgdec_impl.c",
    "datadecoding/journaline/log.c",
    "datadecoding/journaline/newsobject.cpp",
    "datadecoding/journaline/newssvcdec_impl.cpp",
    "datadecoding/journaline/NML.cpp",
    "datadecoding/journaline/Splitter.cpp",
    "datadecoding/MOTSlideShow.cpp",
    "DataIO.cpp",
    "drmchannel/ChannelSimulation.cpp",
    "DrmReceiver.cpp",
    "DrmSimulation.cpp",
    "DrmTransmitter.cpp",
    "FAC/FAC.cpp",
    "InputResample.cpp",
    "interleaver/BlockInterleaver.cpp",
    "interleaver/SymbolInterleaver.cpp",
    "IQInputFilter.cpp",
    "matlib/MatlibSigProToolbox.cpp",
    "matlib/MatlibStdToolbox.cpp",
    "MDI/AFPacketGenerator.cpp",
    "MDI/MDIDecode.cpp",
    "MDI/MDIInBuffer.cpp",
    "MDI/MDIRSCI.cpp",
    "MDI/MDITagItemDecoders.cpp",
    "MDI/MDITagItems.cpp",
    "MDI/PacketSinkFile.cpp",
    "MDI/PacketSocket.cpp",
    "MDI/PacketSourceFile.cpp",
    "MDI/Pft.cpp",
    "MDI/RCITagItems.cpp",
    "MDI/RSCITagItemDecoders.cpp",
    "MDI/RSISubscriber.cpp",
    "MDI/TagPacketDecoder.cpp",
    "MDI/TagPacketDecoderMDI.cpp",
    "MDI/TagPacketDecoderRSCIControl.cpp",
    "MDI/TagPacketGenerator.cpp",
    "mlc/BitInterleaver.cpp",
    "mlc/ChannelCode.cpp",
    "mlc/ConvEncoder.cpp",
    "mlc/EnergyDispersal.cpp",
    "mlc/Metric.cpp",
    "mlc/MLC.cpp",
    "mlc/QAMMapping.cpp",
    "mlc/TrellisUpdateMMX.cpp",
    "mlc/TrellisUpdateSSE2.cpp",
    "mlc/ViterbiDecoder.cpp",
    "MSCMultiplexer.cpp",
    "ofdmcellmapping/CellMappingTable.cpp",
    "ofdmcellmapping/OFDMCellMapping.cpp",
    "OFDM.cpp",
    "Parameter.cpp",
    "PlotManager.cpp",
    "ReceptLog.cpp",
    "resample/Resample.cpp",
    "resample/ResampleFilter.cpp",
    "Scheduler.cpp",
    "SDC/SDCReceive.cpp",
    "SDC/SDCTransmit.cpp",
    "SDC/audioparam.cpp",
    "ServiceInformation.cpp",
    "SimulationParameters.cpp",
    "sound/audiofilein.cpp",
    "sourcedecoders/aac_codec.cpp",
    "sourcedecoders/AudioCodec.cpp",
    "sourcedecoders/AudioSourceDecoder.cpp",
    "sourcedecoders/AudioSourceEncoder.cpp",
    "sourcedecoders/null_codec.cpp",
    "sourcedecoders/opus_codec.cpp",
    "spectrumanalyser.cpp",
    "sync/FreqSyncAcq.cpp",
    "sync/SyncUsingPil.cpp",
    "sync/TimeSync.cpp",
    "sync/TimeSyncFilter.cpp",
    "sync/TimeSyncTrack.cpp",
    "tables/TableCarMap.cpp",
    "tables/TableFAC.cpp",
    "tables/TableStations.cpp",
    "TextMessage.cpp",
    "util/CRC.cpp",
    "util/FileTyper.cpp",
    "util/LogPrint.cpp",
    "util/Reassemble.cpp",
    "util/Settings.cpp",
    "util/Utilities.cpp",
    "Version.cpp",
    "sound/soundnull.cpp",
    "DrmTransceiver.cpp",
    "sound/soundinterface.cpp",
    "sound/selectioninterface.cpp",
    "MSC/logicalframe.cpp",
    "MSC/audiosuperframe.cpp",
    "MSC/aacsuperframe.cpp",
    "MSC/xheaacsuperframe.cpp",
    "MSC/frameborderdescription.cpp",
    "resample/cspectrumresample.cpp",
    "resample/caudioresample.cpp",
    "sourcedecoders/reverb.cpp",
    "sourcedecoders/caudioreverb.cpp",
];

/// Whether the *target* is the MinGW-w64 flavour of Windows — the
/// `x86_64-pc-windows-gnu` triple the release build uses — rather than MSVC.
///
/// Unlike the `cfg!` tests elsewhere in this file this one has to come from the
/// environment: a build script is compiled for the host, and the Windows CI
/// host toolchain is the MSVC one while the target it builds is the GNU one.
fn target_is_windows_gnu() -> bool {
    let os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let env = std::env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
    os == "windows" && env == "gnu"
}

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let dream = manifest.join("../../vendor/dream");
    let faad2 = manifest.join("../../vendor/faad2");

    if !dream.join("src/DrmReceiver.h").exists() {
        panic!("vendored Dream sources are missing at {}", dream.display());
    }
    if !faad2.join("include/neaacdec.h").exists() {
        panic!(
            "vendored faad2 is missing at {}\n\
             run: git submodule update --init --recursive",
            faad2.display()
        );
    }

    build_faad2(&faad2);
    build_dream(&manifest, &dream, &faad2);
    build_shim(&manifest, &dream, &faad2);
    generate_bindings(&manifest, &out);

    println!("cargo:rerun-if-changed=src/drm_shim.cpp");
    println!("cargo:rerun-if-changed=src/drm_shim.h");
    println!("cargo:rerun-if-changed=src/fftw_compat.c");
    println!("cargo:rerun-if-changed=include");
    println!("cargo:rerun-if-changed={}", dream.join("src").display());
}

/// The DRM build of faad2: the same sources as the stock library with
/// `DRM_SUPPORT` added, which is what brings in `NeAACDecInitDRM` and the DRM
/// entry into the decoder. Upstream ships this as a second library,
/// `libfaad_drm`, precisely because the plain one cannot decode DRM at all.
fn build_faad2(faad2: &Path) {
    let mut build = cc::Build::new();
    build
        .include(faad2.join("libfaad"))
        .include(faad2.join("include"))
        .define("HAVE_INTTYPES_H", "1")
        .define("HAVE_MEMCPY", "1")
        .define("HAVE_STRING_H", "1")
        .define("HAVE_STRINGS_H", "1")
        .define("HAVE_SYS_STAT_H", "1")
        .define("HAVE_SYS_TYPES_H", "1")
        .define("PACKAGE_VERSION", "\"2.11.2\"")
        .define("APPLY_DRC", None)
        .define("DRM_SUPPORT", None)
        .opt_level(2)
        .warnings(false);
    if !cfg!(target_env = "msvc") {
        build.define("HAVE_LRINTF", "1");
    }
    for entry in std::fs::read_dir(faad2.join("libfaad")).expect("read faad2/libfaad") {
        let path = entry.expect("faad2 dir entry").path();
        if path.extension().is_some_and(|e| e == "c") {
            build.file(path);
        }
    }
    build.compile("sdroxide_faad2_drm");
}

/// The `HAVE_*` set upstream's `dream.pro` defines on unix, which is really a
/// statement about the C library rather than the platform — the same set is
/// right for every target here. What is *not* defined matters more: no
/// `HAVE_LIBSNDFILE`, `HAVE_SPEEX`, `HAVE_LIBPCAP`, `HAVE_LIBHAMLIB`,
/// `HAVE_LIBGPS`, `HAVE_LIBZ`, and no Qt, so every optional dependency
/// compiles out.
fn common_defines(build: &mut cc::Build) {
    for def in [
        "HAVE_DLFCN_H",
        "HAVE_MEMORY_H",
        "HAVE_STDINT_H",
        "HAVE_STDLIB_H",
        "HAVE_STRINGS_H",
        "HAVE_STRING_H",
        "STDC_HEADERS",
        "HAVE_INTTYPES_H",
        "HAVE_SYS_STAT_H",
        "HAVE_SYS_TYPES_H",
        "HAVE_UNISTD_H",
    ] {
        build.define(def, None);
    }
    // Selects the sound shim in `include/sdrx_sound.h` over a sound card, and
    // links faad2 directly instead of dlopening it.
    build.define("USE_SDROXIDE_SOUND", None);
    build.define("USE_FAAD2_LIBRARY", None);
    build.define("SDROXIDE_NO_AAC_ENCODER", None);
    // The EPG decoder is the one place in the receiver that writes to disk, and
    // it names the file from the broadcast. Nothing here reads those files back.
    build.define("SDROXIDE_NO_DATA_FILES", None);
    // Dream announces every codec library it probes for on stderr; the
    // host has its own logging and a GUI has no terminal to print to.
    build.define("SDROXIDE_QUIET", None);
    build.define("EXECUTABLE_NAME", "sdroxide");
}

fn build_dream(manifest: &Path, dream: &Path, faad2: &Path) {
    let src = dream.join("src");

    // Dream is C++ but three of the journaline files are C, and their
    // declarations are `extern "C"` — compiling them as C++ leaves `logit` and
    // friends undefined at link time.
    let mut cxx = cc::Build::new();
    let mut c = cc::Build::new();
    for build in [&mut cxx, &mut c] {
        build
            .include(&src)
            .include(manifest.join("include"))
            .include(faad2.join("include"))
            .opt_level(2)
            .warnings(false);
        common_defines(build);
    }
    cxx.cpp(true).std("gnu++11");
    if target_is_windows_gnu() {
        // Suppresses cc's own `-lstdc++`; `build_shim` links the C++ runtime
        // statically instead. See the note there.
        cxx.cpp_link_stdlib(None::<&str>);
    }

    for rel in DREAM_SOURCES {
        if rel.ends_with(".c") {
            c.file(src.join(rel));
        } else {
            cxx.file(src.join(rel));
        }
    }
    // CPacer, which `CAudioFileIn` paces file input with.
    if cfg!(target_os = "windows") {
        cxx.file(src.join("windows/Pacer.cpp"));
        cxx.file(src.join("windows/platform_util.cpp"));
    } else {
        cxx.file(src.join("linux/Pacer.cpp"));
    }

    c.compile("sdroxide_dream_c");
    cxx.compile("sdroxide_dream");

    // The FFTW stand-in. Its own translation unit so it stays independent of
    // Dream's build flags.
    let mut fft = cc::Build::new();
    fft.file(manifest.join("src/fftw_compat.c"))
        .include(manifest.join("include"))
        .opt_level(3)
        .warnings(false)
        .compile("sdroxide_fftw_compat");
}

fn build_shim(manifest: &Path, dream: &Path, faad2: &Path) {
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("gnu++11")
        .file(manifest.join("src/drm_shim.cpp"))
        .include(dream.join("src"))
        .include(manifest.join("include"))
        .include(faad2.join("include"))
        .opt_level(2)
        .warnings(false);
    common_defines(&mut build);
    if target_is_windows_gnu() {
        build.cpp_link_stdlib(None::<&str>);
    }
    build.compile("sdroxide_drm_shim");

    // cc links the C++ runtime for the objects it builds; the Dream and shim
    // archives both need it, and on unix Dream's LibraryLoader needs dlopen.
    if cfg!(unix) {
        // Both upstreams link the maths library themselves (`dream.pro`'s
        // `-lm`, faad2's CMake `MATH_LIBRARY`), and this build has to as well.
        // It happens to be redundant on a current glibc, where libm was folded
        // into libc in 2.34 — which is exactly why it is easy to leave out and
        // only notice on an older target.
        println!("cargo:rustc-link-lib=dylib=m");
    }
    if cfg!(unix) && !cfg!(target_os = "macos") {
        println!("cargo:rustc-link-lib=dylib=dl");
    }
    // MinGW keeps its C++ runtime in libstdc++-6.dll and cc asks for it by
    // name, so the shipped .exe refused to start on any Windows box without
    // MSYS2 or PothosSDR on PATH ("libstdc++-6.dll not found"). Link it in
    // instead — this is the only C++ in the tree, so it is the only thing that
    // pulls the DLL in.
    //
    // `static:-bundle` rather than plain `static` is the load-bearing part:
    // both make rustc bracket the library with `-Wl,-Bstatic` at the final
    // link, but `static` alone would also try to copy libstdc++.a into this
    // crate's rlib, and rustc has no `-L` that finds it. libwinpthread follows
    // because MSYS2 builds libstdc++ against it, and it is the next DLL the
    // loader would ask for.
    if target_is_windows_gnu() {
        println!("cargo:rustc-link-lib=static:-bundle=stdc++");
        println!("cargo:rustc-link-lib=static:-bundle=winpthread");
    }
    if cfg!(target_os = "windows") {
        println!("cargo:rustc-link-lib=dylib=ws2_32");
    }
}

fn generate_bindings(manifest: &Path, out: &Path) {
    bindgen::Builder::default()
        .header(manifest.join("src/drm_shim.h").to_string_lossy())
        .allowlist_function("sdrx_drm_.*")
        .allowlist_type("sdrx_drm_.*")
        .allowlist_var("SDRX_DRM_.*")
        .layout_tests(false)
        .derive_debug(false)
        .generate()
        .expect("bindgen drm_shim.h")
        .write_to_file(out.join("bindings.rs"))
        .expect("write bindings.rs");
}
