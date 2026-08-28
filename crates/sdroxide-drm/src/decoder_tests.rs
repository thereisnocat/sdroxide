//! The decoder across the C boundary.
//!
//! Unit tests rather than tests under `tests/`, for the reason given in
//! [`crate::fftw_tests`].
//!
//! What runs everywhere is the lifecycle: that a receiver can be built, fed,
//! read and shut down without deadlocking. That is worth a test on its own —
//! the decoder blocks on its input by design, so a mistake in the stop sequence
//! hangs the radio on every mode change rather than failing visibly.
//!
//! Decoding a real broadcast needs a recording, which cannot live in the
//! repository: they are minutes of somebody's copyrighted programme material,
//! and megabytes of it. Point `SDROXIDE_DRM_SAMPLE` at one to run that test —
//! see the harness in `examples/drm_harness.rs`, which is the fuller tool.

use crate::{AUDIO_RATE, DrmWorker, SIGNAL_RATE};

/// A deterministic pseudo-random fill, so a failure reproduces exactly.
fn noise(n: usize) -> Vec<i16> {
    let mut state = 0x1234_5678_9abc_def0u64;
    (0..n)
        .map(|_| {
            state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((state >> 40) as i32 - 8192) as i16
        })
        .collect()
}

#[test]
fn a_decoder_starts_takes_samples_and_stops() {
    let worker = DrmWorker::new(true, false).expect("start the decoder");

    // A second of noise, as interleaved I/Q pairs.
    let block = noise(2 * SIGNAL_RATE as usize);
    for chunk in block.chunks(4800) {
        worker.push(chunk);
    }

    // Nothing to find in noise, so the interesting assertion is that asking is
    // safe and answers something coherent rather than that it locks.
    let status = worker.status();
    assert!(!status.locked, "the decoder claimed a lock on noise");
    assert!(status.service.label.is_empty(), "noise produced a service label");

    // The real assertion: dropping the worker while its thread is blocked
    // waiting for more input still joins. A regression here hangs forever
    // rather than failing, which the test harness reports as a timeout.
    drop(worker);
}

#[test]
fn a_decoder_that_was_never_fed_still_shuts_down() {
    // The thread spends its whole life blocked in the read, which is the case
    // the stop flag and the ring's own release have to cover between them.
    let worker = DrmWorker::new(true, false).expect("start the decoder");
    drop(worker);
}

#[test]
fn two_decoders_do_not_share_a_ring() {
    // Each decoder finds its queues through a thread-local set on its own
    // worker thread. If that ever became one global, a second receiver would
    // silently steal the first's samples — which is exactly what a split-view
    // or multi-radio session would do.
    let a = DrmWorker::new(true, false).expect("start the first decoder");
    let b = DrmWorker::new(true, false).expect("start the second decoder");
    a.push(&noise(4800));
    assert_eq!(b.audio_available(), 0, "samples pushed to one decoder reached the other");
    drop(a);
    drop(b);
}

/// Two decoders may be built at the same time on two threads.
///
/// Dream shares its audio codecs through a `static` list with a plain `int`
/// reference count and no lock, which is safe for the single-threaded console
/// receiver it was written for and not for a host that runs one receiver per
/// radio. Two constructors racing double-free the list's storage; worse, both
/// receivers would then be handed the *same* `AacCodec`, so one's `DecClose`
/// frees the faad2 handle the other is decoding through. The vendored list is
/// per-thread now (see `vendor/PROVENANCE.md`).
///
/// This is not a hypothetical pair: a split view or a second radio makes it
/// permanent, and `RxChain::build_for_mode` used to make it momentarily every
/// time the mode was set — which a CAT rig reporting its own mode back does on
/// its own schedule.
///
/// Like the exception test, a failure here does not report an assertion. The
/// process dies, or ASan does the reporting.
#[test]
fn two_decoders_can_start_at_the_same_time() {
    use std::sync::{Arc, Barrier};

    // Several rounds: the window is the few milliseconds inside the two
    // constructors, so one attempt proves very little.
    for _ in 0..8 {
        let gate = Arc::new(Barrier::new(2));
        let threads: Vec<_> = (0..2)
            .map(|_| {
                let gate = Arc::clone(&gate);
                std::thread::spawn(move || {
                    gate.wait();
                    let w = DrmWorker::new(true, false).expect("start a decoder");
                    w.push(&noise(4800));
                    drop(w);
                })
            })
            .collect();
        for t in threads {
            t.join().expect("a decoder thread panicked");
        }
    }
}

/// Decode a real off-air recording, when one is available.
///
/// Recordings are 48 kHz *real* signals off a receiver's I.F., which is not the
/// zero-IF baseband the receive chain produces — so this drives the decoder's
/// real-signal input rather than [`crate::DrmDemod`]. The harness example
/// covers the baseband path as well.
#[test]
fn a_recording_decodes() {
    let Ok(path) = std::env::var("SDROXIDE_DRM_SAMPLE") else {
        eprintln!("set SDROXIDE_DRM_SAMPLE to a 48 kHz DRM recording to run this");
        return;
    };

    let mut reader = hound::WavReader::open(&path).expect("open the recording");
    let spec = reader.spec();
    assert_eq!(spec.sample_rate as f64, SIGNAL_RATE, "the recording must be 48 kHz");
    let channels = spec.channels as usize;
    let mono: Vec<i16> = reader
        .samples::<i16>()
        .map(|s| s.expect("read sample"))
        .collect::<Vec<_>>()
        .chunks(channels)
        .map(|c| c[0])
        .collect();

    let worker = DrmWorker::new(false, false).expect("start the decoder");
    let mut interleaved = Vec::with_capacity(mono.len() * 2);
    for &s in &mono {
        interleaved.push(s);
        interleaved.push(s);
    }

    let mut sink = vec![0i16; 8192];
    let mut audio_frames = 0usize;
    for chunk in interleaved.chunks(4800) {
        while worker.push(chunk) > 0 {
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        loop {
            let n = worker.pop(&mut sink);
            if n == 0 {
                break;
            }
            audio_frames += n / 2;
        }
        std::thread::sleep(std::time::Duration::from_millis(4));
    }

    let status = worker.status();
    assert!(status.locked, "the decoder did not lock onto {path}");
    assert!(status.fac.is_ok(), "the FAC never decoded, so nothing else can be believed");
    assert!(!status.service.label.is_empty(), "no service label was decoded");
    assert!(status.snr_db > 0.0, "a locked decode reported {} dB SNR", status.snr_db);
    // Well under real time would mean the audio chain stalled even though the
    // demodulator was working.
    let seconds = audio_frames as f64 / AUDIO_RATE;
    let expected = mono.len() as f64 / SIGNAL_RATE;
    assert!(
        seconds > expected * 0.5,
        "only {seconds:.1} s of audio came out of a {expected:.1} s recording"
    );
}

/// Nothing thrown inside the shim may unwind into Rust.
///
/// This is the property that matters most for a radio that has to stay up: the
/// FFI declarations are plain `extern "C"`, and Rust cannot catch a foreign
/// exception crossing one — it aborts the whole process, `catch_unwind` and
/// all. Dream's deliberate throws are `CGenErr` and `std::string`, which the
/// shim always caught; the ones that actually reach the boundary from a real
/// broadcast are implicit — `std::bad_alloc` and `std::length_error` out of the
/// `resize()` calls its over-the-air parsers make with lengths the transmission
/// supplied.
///
/// A failure here is not a failed assertion. The test binary dies.
#[test]
fn no_exception_escapes_the_c_boundary() {
    for kind in 0..5 {
        // SAFETY: the hook exists to be called; it throws and catches inside C++.
        let rc = unsafe { crate::sys::sdrx_drm_test_throw(kind) };
        assert_eq!(rc, -1, "kind {kind} was not reported as a failure");
        assert!(
            crate::last_error().contains("test throw"),
            "kind {kind} left no reason behind: {:?}",
            crate::last_error()
        );
    }
}

/// One audio super frame, straight into the parser named by `kind`.
///
/// Returns the number of audio frames it yielded, -2 if the parser rejected the
/// super frame, or -1 if the call threw.
fn parse_superframe(kind: i32, part_a: usize, part_b: usize, bytes: &[u8]) -> i32 {
    // SAFETY: the hook exists to be called, and `bytes`/`len` describe a live
    // slice that the shim only reads for the duration of the call.
    unsafe {
        crate::sys::sdrx_drm_test_parse_superframe(
            kind,
            part_a as i32,
            part_b as i32,
            bytes.as_ptr(),
            bytes.len() as i32,
            std::ptr::null_mut(),
            0,
        )
    }
}

/// The same, but reporting how long each audio frame came out.
///
/// The frame count is the declared border count whether or not the borders were
/// read correctly, so it is the sizes that say where they fell.
fn parse_superframe_sizes(kind: i32, part_a: usize, part_b: usize, bytes: &[u8]) -> Vec<i32> {
    let mut sizes = [0i32; 16];
    // SAFETY: as above, plus `sizes` is a live array of the length passed.
    let n = unsafe {
        crate::sys::sdrx_drm_test_parse_superframe(
            kind,
            part_a as i32,
            part_b as i32,
            bytes.as_ptr(),
            bytes.len() as i32,
            sizes.as_mut_ptr(),
            sizes.len() as i32,
        )
    };
    assert!(n >= 0, "parser rejected a super frame it should have accepted");
    sizes[..n as usize].to_vec()
}

/// Point the parser named by `kind` at a stream of `part_a + part_b` byte frames.
fn init_superframe(kind: i32, part_a: usize, part_b: usize) {
    // SAFETY: a null `bytes` is the documented "initialise only" form.
    let rc = unsafe {
        crate::sys::sdrx_drm_test_parse_superframe(
            kind,
            part_a as i32,
            part_b as i32,
            std::ptr::null(),
            0,
            std::ptr::null_mut(),
            0,
        )
    };
    assert_eq!(rc, 0, "kind {kind} would not initialise");
}

/// A corrupt audio super frame may not take the process down.
///
/// The header a DRM audio super frame starts with is four bits of frame border
/// count and four of bit reservoir level, followed by a CRC the parser
/// deliberately ignores — it would rather trust a damaged count than lose the
/// super frame. Everything after that is arithmetic on unsigned lengths the
/// broadcast supplied, so a single flipped bit used to walk a `deque` past its
/// end (a null block pointer, reported from a station on 13730 kHz) or index an
/// empty frame vector.
///
/// Noise is the honest input here: a DRM signal fading out delivers exactly
/// this, and the MSC has no CRC standing between it and these parsers.
///
/// A failure is not a failed assertion. The test binary dies.
#[test]
fn a_corrupt_audio_superframe_cannot_crash_the_parser() {
    let kinds = [
        crate::sys::SDRX_DRM_SF_XHE_AAC,
        crate::sys::SDRX_DRM_SF_AAC_12KHZ,
        crate::sys::SDRX_DRM_SF_AAC_24KHZ,
        crate::sys::SDRX_DRM_SF_AAC_MODE_E,
    ];
    // Real super frames run from a few dozen bytes for a 4 kbps voice service
    // to a couple of thousand for 20 kHz DRM+; the degenerate sizes matter
    // because that is where the length arithmetic wraps.
    let sizes = [1usize, 2, 3, 4, 31, 32, 33, 100, 300, 301, 1000, 2000];

    let mut state = 0x1234_5678_9abc_def0u64;
    let mut byte = || {
        state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
        (state >> 40) as u8
    };

    for kind in kinds {
        for size in sizes {
            for part_a in [0, size / 3, size] {
                init_superframe(kind as i32, part_a, size - part_a);
                // Long enough that the payload a parser carries between super
                // frames gets a chance to grow, which is what the border
                // arithmetic is measured against.
                for _ in 0..200 {
                    let frame: Vec<u8> = (0..size).map(|_| byte()).collect();
                    let frames = parse_superframe(kind as i32, part_a, size - part_a, &frame);
                    // Rejection (-2) is the expected answer to noise. The
                    // assertion is that nothing threw, and - the point of the
                    // test - that the process is still here to assert it.
                    assert_ne!(
                        frames,
                        -1,
                        "kind {kind} size {size} part A {part_a} threw: {:?}",
                        crate::last_error()
                    );
                }
            }
        }
    }
}

/// The three xHE-AAC super frames that used to segfault, kept by name.
///
/// Each is a minimal header that a parser trusting its own arithmetic follows
/// off the end of something. They are cheap to check and they say what broke,
/// which the noise sweep above cannot.
#[test]
fn the_xhe_aac_superframes_that_used_to_segfault() {
    const XHE: i32 = crate::sys::SDRX_DRM_SF_XHE_AAC as i32;
    let size = 300usize;

    // A directory pointing 4093 bytes into a 300 byte super frame: the frame
    // size that comes out is larger than the payload the parser is holding, and
    // the copy loop used to keep popping an empty deque.
    let mut far = vec![0x5au8; size];
    far[0] = 0x10; // frame border count 1, bit reservoir level 0
    far[size - 2] = 0xff; // frame border index 0xffd ...
    far[size - 1] = 0xd1; // ... and the count repeated as 1
    init_superframe(XHE, 0, size);
    assert_eq!(parse_superframe(XHE, 0, size, &far), -2, "far border");

    // No frame borders at all — legal, and the spec says so explicitly — but
    // the copy loop ran anyway and indexed an empty frame vector.
    let mut none = vec![0x5au8; size];
    none[0] = 0x00;
    init_superframe(XHE, 0, size);
    assert_eq!(parse_superframe(XHE, 0, size, &none), 0, "no frame borders");

    // Fifteen frame borders need 30 bytes of directory, which does not fit in a
    // ten byte super frame: the offset of the directory used to wrap.
    let mut tiny = vec![0x5au8; 10];
    tiny[0] = 0xf0;
    init_superframe(XHE, 0, 10);
    assert_eq!(parse_superframe(XHE, 0, 10, &tiny), -2, "directory too large");
}

/// A well-formed super frame still parses, on both codecs.
///
/// The counterweight to the checks above: every one of them rejects a super
/// frame, and a rejection too eager would take a working broadcast off the air
/// far more thoroughly than the crash did. Both frames here are built by hand
/// from the layout in ETSI ES 201 980 - the xHE-AAC Header and Directory of
/// clause 5.3.1.3, the AAC frame border table of clause 5.3.1.1.
#[test]
fn a_well_formed_superframe_parses() {
    const XHE: i32 = crate::sys::SDRX_DRM_SF_XHE_AAC as i32;
    const AAC: i32 = crate::sys::SDRX_DRM_SF_AAC_12KHZ as i32;
    let size = 300usize;

    // xHE-AAC: two frame borders, 2 bytes of Header, 294 bytes of Payload and a
    // 4 byte Directory. The Directory is read last border first, and each entry
    // is a 12 bit index into the Payload followed by the border count repeated.
    // Borders at 102 and 202 cut the payload into frames of 102 and 100 bytes
    // and leave 92 bytes running on into the next super frame. See
    // `xhe_aac_frame_borders_are_counted_from_the_payload` for why those
    // numbers and not 100/100/94.
    let mut good = vec![0x5au8; size];
    good[0] = 0x20; // frame border count 2, bit reservoir level 0
    good[size - 4] = 0x0c; // index 202 = 0x0ca ...
    good[size - 3] = 0xa2; // ... then the count, 2
    good[size - 2] = 0x06; // index 102 = 0x066 ...
    good[size - 1] = 0x62; // ... then the count, 2
    init_superframe(XHE, 0, size);
    assert_eq!(parse_superframe(XHE, 0, size, &good), 2, "two frame borders");

    // And the 92 bytes left over are carried: the next super frame declares one
    // border at 102, which with the carry lands 194 bytes into the payload.
    let mut next = vec![0x5au8; size];
    next[0] = 0x10; // frame border count 1
    next[size - 2] = 0x06; // index 102 ...
    next[size - 1] = 0x61; // ... then the count, 1
    assert_eq!(parse_superframe(XHE, 0, size, &next), 1, "carried payload");

    // AAC at 12 kHz in modes A-D: 5 audio frames, so 4 frame borders in 6 bytes
    // of header, then one CRC byte per frame and 289 bytes of audio. The
    // borders are cumulative byte offsets - 50, 100, 150, 200 - packed as 12 bit
    // fields, which puts four frames of 50 bytes ahead of a last one of 89.
    let mut aac = vec![0x5au8; size];
    aac[..6].copy_from_slice(&[0x03, 0x20, 0x64, 0x09, 0x60, 0xc8]);
    init_superframe(AAC, 0, size);
    assert_eq!(parse_superframe(AAC, 0, size, &aac), 5, "five AAC frames");

    // The same frame under unequal protection, which is what the two length
    // checks added to the AAC parser actually bound. Part A holds the 6 byte
    // header, one CRC byte per frame and a 10 byte share of each frame:
    // 6 + 5 + 50 = 61, leaving 239 bytes of Part B.
    init_superframe(AAC, 61, size - 61);
    assert_eq!(parse_superframe(AAC, 61, size - 61, &aac), 5, "five UEP AAC frames");
}

/// An xHE-AAC frame border is counted from the first byte of the Payload
/// section, not from the first byte of the super frame.
///
/// Dream 2.2 subtracted two from every border "because the header is not in the
/// payload". The parser contradicts itself about that: the two special border
/// values are 0xFFE for "two bytes back into the previous super frame's
/// payload" and 0xFFF for "one byte back", and with the subtraction an ordinary
/// index of 0x000 means exactly what 0xFFE already means. A standard does not
/// spell one border two ways. Without the subtraction the encoding runs
/// straight through - 0xFFE, 0xFFF, 0x000, 0x001 - which is what it is for.
///
/// The cost of getting it wrong is not two bytes in one frame. Those two bytes
/// are never consumed, so the payload stays two bytes ahead and every audio
/// frame after the first is cut two bytes early, for as long as the receiver
/// stays tuned - which is a locked receiver, a service label, and silence.
///
/// The frame *count* is the declared border count either way, which is why
/// `a_well_formed_superframe_parses` cannot see this and this test exists.
#[test]
fn xhe_aac_frame_borders_are_counted_from_the_payload() {
    const XHE: i32 = crate::sys::SDRX_DRM_SF_XHE_AAC as i32;
    let size = 300usize;

    // 2 byte Header, 294 byte Payload, 4 byte Directory; borders at 102 and 202.
    let mut good = vec![0x5au8; size];
    good[0] = 0x20; // frame border count 2, bit reservoir level 0
    good[size - 4] = 0x0c; // index 202 = 0x0ca ...
    good[size - 3] = 0xa2; // ... then the count, 2
    good[size - 2] = 0x06; // index 102 = 0x066 ...
    good[size - 1] = 0x62; // ... then the count, 2
    init_superframe(XHE, 0, size);
    assert_eq!(
        parse_superframe_sizes(XHE, 0, size, &good),
        vec![102, 100],
        "the first frame runs from the start of the Payload to border 0, so it \
         is 102 bytes - 100 is the two byte Header wrongly deducted"
    );

    // 294 - 202 = 92 bytes are carried, not 94. The next super frame has one
    // border at 102, so its frame is 92 + 102 = 194 bytes; that figure is the
    // same under either reading, because the carry absorbs the error - which is
    // exactly how a two byte slip goes unnoticed on every frame after the first.
    let mut next = vec![0x5au8; size];
    next[0] = 0x10; // frame border count 1
    next[size - 2] = 0x06; // index 102 ...
    next[size - 1] = 0x61; // ... then the count, 1
    assert_eq!(parse_superframe_sizes(XHE, 0, size, &next), vec![194]);

    // A border at index 0 is now representable and must not be rejected: the
    // guards that used to reject it existed only to stop the subtraction
    // wrapping an unsigned through zero.
    let mut zero = vec![0x5au8; size];
    zero[0] = 0x10; // one border
    zero[size - 2] = 0x00; // index 0 ...
    zero[size - 1] = 0x01; // ... then the count, 1
    init_superframe(XHE, 0, size);
    assert_eq!(
        parse_superframe_sizes(XHE, 0, size, &zero),
        vec![0],
        "a border at the very start of the payload is a zero length frame, not \
         a rejection"
    );
}

/// The AAC decoder is still faad2, and the xHE-AAC decoder is reported
/// separately when the host has libfdk-aac.
///
/// `InitCodecList` puts `FdkAacCodec` ahead of `AacCodec`, and upstream's
/// `CanDecode(AC_AAC)` answers yes on any fdk-aac 2.x — it tests SBR capability
/// bits against the AAC decoder module's flag word — so without the guard in
/// `fdk_aac_codec.cpp` installing libfdk-aac would silently move every ordinary
/// DRM broadcast off the statically linked faad2 this receiver was verified on.
#[test]
fn aac_stays_with_faad2_whatever_else_is_installed() {
    // The codec list is thread-local and built by the first decoder.
    let ring = crate::Ring::new(4096, 4096);
    let _d = crate::Decoder::new(&ring, true, false).expect("decoder");
    let v = crate::codec_version();
    assert!(v.contains("Nero AAC"), "AAC should still be faad2, got {v:?}");
    // Whether the xHE-AAC half is there depends on the host, so this only
    // reports it — but it must never be the *only* entry.
    eprintln!("DRM audio decoders: {v}");
}

/// An xHE-AAC broadcast decodes to audio.
///
/// The regression gate for the whole xHE-AAC change, and the one test that
/// exercises libfdk-aac at all. Two independent faults had to be fixed before
/// this could pass, and each hid the other:
///
/// * `FdkAacCodec::Decode` told the library its output buffer was
///   `frameSize * numChannels` samples long, read from the stream info *before*
///   the frame was decoded — and a USAC stream reports zero channels until one
///   has been. Every frame asked for a zero-sample buffer, so every frame came
///   back `AAC_DEC_OUTPUT_BUFFER_TOO_SMALL` and nothing at all was decoded.
/// * `XHEAACSuperFrame::parse` cut every audio frame two bytes short, so the
///   frames that reached the decoder were misaligned. That one does not show in
///   the sample count — the codec conceals its way through and returns samples
///   either way — but it took the library's own error count on a clean 65
///   second recording from 2 to 220, with audible dropouts to match.
///
/// So a count of decoded samples catches the first and a listen catches the
/// second. Both are asserted here as far as they can be.
///
/// Set `SDROXIDE_DRM_XHE_SAMPLE` to a 48 kHz recording of an xHE-AAC broadcast.
/// `FMGold_xHE_ModeB_9khz.flac` from
/// <https://sourceforge.net/projects/drm/files/samples/DRM%20sample%20recordings/>
/// is one; convert it with `ffmpeg -i x.flac -ar 48000 -ac 1 x.wav`.
#[test]
fn an_xhe_aac_recording_decodes() {
    let Ok(path) = std::env::var("SDROXIDE_DRM_XHE_SAMPLE") else {
        eprintln!("set SDROXIDE_DRM_XHE_SAMPLE to a 48 kHz xHE-AAC recording to run this");
        return;
    };

    let mut reader = hound::WavReader::open(&path).expect("open the recording");
    let spec = reader.spec();
    assert_eq!(spec.sample_rate as f64, SIGNAL_RATE, "the recording must be 48 kHz");
    let channels = spec.channels as usize;
    let mono: Vec<i16> = reader
        .samples::<i16>()
        .map(|s| s.expect("read sample"))
        .collect::<Vec<_>>()
        .chunks(channels)
        .map(|c| c[0])
        .collect();

    let worker = DrmWorker::new(false, false).expect("start the decoder");
    let mut interleaved = Vec::with_capacity(mono.len() * 2);
    for &s in &mono {
        interleaved.push(s);
        interleaved.push(s);
    }

    let mut sink = vec![0i16; 8192];
    let mut audio_samples = 0usize;
    for chunk in interleaved.chunks(4800) {
        while worker.push(chunk) > 0 {
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        loop {
            let n = worker.pop(&mut sink);
            if n == 0 {
                break;
            }
            audio_samples += n / 2;
        }
        std::thread::sleep(std::time::Duration::from_millis(4));
    }

    let status = worker.status();
    assert!(status.locked, "the decoder did not lock onto {path}");
    assert_eq!(
        status.service.codec,
        Some(sdroxide_types::DrmCodec::XheAac),
        "{path} is not an xHE-AAC broadcast"
    );
    assert!(
        status.service.codec_supported,
        "no xHE-AAC decoder registered — this host needs libfdk-aac installed"
    );
    assert!(!status.service.label.is_empty(), "no service label was decoded");

    // The assertion the buffer-size fix exists for: before it, this was zero.
    let seconds = audio_samples as f64 / AUDIO_RATE;
    let expected = mono.len() as f64 / SIGNAL_RATE;
    assert!(
        seconds > expected * 0.5,
        "only {seconds:.1} s of audio came out of a {expected:.1} s recording"
    );
}
