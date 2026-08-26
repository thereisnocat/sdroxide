/* A C API over Dream's CDRMReceiver, sized to what a host radio needs: push
   baseband samples in, pull decoded audio and a status snapshot out.
 *
 * Threading: the ring is the only object that crosses threads. Everything
 * taking an `sdrx_drm*` — new, process, status, restart, free — must be called
 * on one and the same thread, because Dream's receiver is not internally
 * synchronised and the sound shims find their ring through a thread-local. */
#ifndef SDRX_DRM_SHIM_H
#define SDRX_DRM_SHIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sdrx_drm_ring sdrx_drm_ring;
typedef struct sdrx_drm sdrx_drm;

/* Robustness mode, spectrum occupancy and coding scheme are reported as the
   raw enum values Dream uses; the Rust side names them. */
typedef struct {
    /* Per-stage receive status, 0..3 = not present / CRC error / data error / ok. */
    int32_t io_status;
    int32_t time_sync_status;
    int32_t frame_sync_status;
    int32_t fac_status;
    int32_t sdc_status;
    int32_t audio_status;

    int32_t has_signal;          /* acquisition finished and a signal is locked */
    double  if_level_db;
    double  snr_db;
    double  wmer_db;
    double  mer_db;
    double  dc_frequency_hz;     /* where in the 48 kHz band the DRM carrier sits */
    double  sample_offset_hz;
    double  doppler_hz;          /* < 0 when not estimated */
    double  delay_ms;

    int32_t robustness_mode;     /* 0=A 1=B 2=C 3=D, -1 unknown */
    int32_t spectrum_occupancy;  /* 0..5 = 4.5/5/9/10/18/20 kHz, -1 unknown */
    int32_t interleaver_long;    /* 1 = 2 s, 0 = 400 ms */
    int32_t sdc_scheme;          /* CS_1_SM.. */
    int32_t msc_scheme;
    int32_t prot_level_a;
    int32_t prot_level_b;

    int32_t num_audio_services;
    int32_t num_data_services;
    int32_t cur_service;
    double  bitrate_kbps;
    int32_t audio_codec;         /* CAudioParam::EAudCod */
    int32_t audio_mode;          /* mono / p-stereo / stereo */
    int32_t audio_sample_rate;
    int32_t is_stereo;
    int32_t audio_sample_rate_out; /* rate the decoded audio is delivered at */

    /* UTF-8, NUL-terminated. */
    char    label[65];
    char    text_message[257];
    char    country_code[8];
    char    language_code[8];
    int32_t service_id;

    /* Broadcast time, all zero when the multiplex carries none. */
    int32_t year, month, day, utc_hour, utc_minute;
} sdrx_drm_status;

typedef struct {
    int32_t sig_sample_rate;   /* rate of the samples pushed in, 24/48/96/192k */
    int32_t aud_sample_rate;   /* rate decoded audio comes back at */
    int32_t iq_input;          /* 1 = zero-IF I/Q pairs, 0 = a real signal in both channels */
    int32_t flip_spectrum;
} sdrx_drm_config;

/* Capacities are in samples, i.e. two per interleaved frame. */
sdrx_drm_ring* sdrx_drm_ring_new(size_t in_capacity, size_t out_capacity);
void sdrx_drm_ring_free(sdrx_drm_ring*);
/* Returns how many samples had to be dropped to make room. */
size_t sdrx_drm_ring_push(sdrx_drm_ring*, const int16_t* interleaved, size_t samples);
size_t sdrx_drm_ring_pop(sdrx_drm_ring*, int16_t* interleaved, size_t samples);
size_t sdrx_drm_ring_out_available(sdrx_drm_ring*);
/* Unblocks a decoder waiting for input so its thread can be joined. */
void sdrx_drm_ring_stop(sdrx_drm_ring*);

sdrx_drm* sdrx_drm_new(sdrx_drm_ring*, const sdrx_drm_config*);
void sdrx_drm_free(sdrx_drm*);
/* One pass of Dream's receive chain. Blocks until the ring has a block of
   input. Returns 0 normally, -1 if the chain threw. */
int32_t sdrx_drm_process(sdrx_drm*);
/* Re-acquire from scratch — after a retune, or a mode change. */
void sdrx_drm_restart(sdrx_drm*);
void sdrx_drm_get_status(sdrx_drm*, sdrx_drm_status*);
/* Pick which service of the multiplex to decode. */
void sdrx_drm_select_service(sdrx_drm*, int32_t service);

/* Which logical channel's equalised symbols to read back. */
#define SDRX_DRM_CHANNEL_FAC 0
#define SDRX_DRM_CHANNEL_SDC 1
#define SDRX_DRM_CHANNEL_MSC 2

/* Copy the constellation of one logical channel into `out` as interleaved
 * re/im pairs, writing at most `max_points` of them.
 *
 * The MSC carries a couple of thousand cells per frame, far more than a plot
 * needs, so the points are taken at an even stride across the whole frame
 * rather than from the front of it — a prefix would be one corner of the time
 * -frequency grid and would not show fading spread across the rest.
 *
 * Returns the number of points written, or 0 when the channel has not been
 * decoded yet. `qam` receives 4, 16 or 64. */
int32_t sdrx_drm_constellation(sdrx_drm*, int32_t channel, float* out,
                               int32_t max_points, int32_t* qam);
/* "faad2 2.11.2" or "" when no AAC decoder is present. */
const char* sdrx_drm_codec_version(void);

/* Why the last call failed, on this thread, or "" if none has. No call in this
   header lets a C++ exception reach the caller — Rust declares them all as
   plain `extern "C"`, across which an unwind is undefined behaviour and in
   practice takes the process down. `sdrx_drm_process` returning non-zero is
   the signal that something threw; this says what. Valid until the next call
   on the same thread. */
const char* sdrx_drm_last_error(void);

/* Throw one exception of the given kind from inside the shim, and report
 * whether it was contained: 0 if nothing escaped, -1 if the guard caught it.
 * Reaching the caller at all is the assertion — an unwind past this point
 * aborts the process. Exists only for the test of that property.
 * 0 = std::bad_alloc, 1 = CGenErr, 2 = std::string, 3 = const char*, else int. */
int32_t sdrx_drm_test_throw(int32_t kind);

/* Which of Dream's audio super frame parsers `sdrx_drm_test_parse_superframe`
   drives, and with what the receiver would have configured it. */
#define SDRX_DRM_SF_XHE_AAC   0  /* xHE-AAC, any robustness mode */
#define SDRX_DRM_SF_AAC_12KHZ 1  /* AAC, modes A-D at 12 kHz: 5 frames */
#define SDRX_DRM_SF_AAC_24KHZ 2  /* AAC, modes A-D at 24 kHz: 10 frames */
#define SDRX_DRM_SF_AAC_MODE_E 3 /* AAC, mode E at 24 kHz: 5 frames */

/* Drive one audio super frame parser directly, with no receiver around it, so
 * a test can throw corrupt headers and directories at it the way a fading
 * broadcast does. The parsers work on lengths the broadcast itself supplies and
 * are reached long before any CRC has vouched for them, which is what makes
 * them worth fuzzing on their own.
 *
 * `bytes == NULL` (re)initialises the parser named by `kind` for a stream of
 * `len_part_a` + `len_part_b` byte super frames, and returns 0. Otherwise
 * `bytes`/`len` is one super frame to parse, and the return is the number of
 * audio frames it yielded - each of which is also read back out, so the frame
 * accessor is covered too. A rejected super frame is -2 and a call that threw
 * is -1; a healthy broadcast never sees either, noise sees -2 constantly.
 *
 * The parsers carry payload between super frames, so successive calls
 * accumulate until the next initialisation. One parser of each kind per thread.
 * Exists only for the test that a corrupt super frame cannot take the process
 * down. */
int32_t sdrx_drm_test_parse_superframe(int32_t kind, int32_t len_part_a,
                                       int32_t len_part_b, const uint8_t* bytes,
                                       int32_t len);

#ifdef __cplusplus
}
#endif

#endif
