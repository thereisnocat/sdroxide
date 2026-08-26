/* See drm_shim.h. Drives Dream's CDRMReceiver from ring buffers and flattens
   what it knows into one status struct. */

#include "drm_shim.h"
#include "sdrx_sound.h"

#include "DrmReceiver.h"
#include "GlobalDefinitions.h"
#include "MSC/aacsuperframe.h"
#include "MSC/xheaacsuperframe.h"
#include "Parameter.h"
#include "util/Settings.h"
#include "sourcedecoders/AudioCodec.h"

#include <cstring>
#include <string>

/* Which ring the sound objects Dream allocates should attach to. Dream news
   them up deep inside CReceiveData/CWriteData, so there is nowhere to pass it;
   a thread-local works because one decoder owns one thread. */
static thread_local CSdrxRing* tls_ring = nullptr;

CSdrxRing* sdrx_current_ring() { return tls_ring; }
void sdrx_set_current_ring(CSdrxRing* r) { tls_ring = r; }

const char* CSoundInSdrx::SDRX_DEVICE = "sdroxide";

/* Nothing may unwind out of this file.
 *
 * Every entry point below is `extern "C"` and is called from Rust through a
 * plain `extern "C"` declaration, across which a C++ exception is undefined
 * behaviour — in practice the process aborts, with no Rust backtrace and
 * nothing in the log. The two Dream throws that were caught before are the
 * deliberate ones; the ones that actually reach here are implicit. Dream's
 * over-the-air parsers size their buffers from lengths the broadcast supplies
 * (the MOT reassembler is the worst of them), so `std::bad_alloc` and
 * `std::length_error` out of a `resize()` are the realistic escapees, and a
 * shortwave broadcast is exactly the sort of input that produces them.
 *
 * The reason is kept in a thread-local so the worker can log what happened,
 * which is the difference between a user reporting "sdroxide vanished" and
 * reporting a decode that stopped. */
static thread_local std::string tls_error;

const char* sdrx_drm_last_error(void)
{
    return tls_error.c_str();
}

/* Run `body`, converting any exception into `false` plus a recorded reason. */
template <typename F>
static bool sdrx_guard(const char* what, F&& body)
{
    try
    {
        body();
        return true;
    }
    catch (CGenErr& e)
    {
        tls_error = std::string(what) + ": " + e.strError;
    }
    catch (std::exception& e)
    {
        tls_error = std::string(what) + ": " + e.what();
    }
    catch (std::string& e)
    {
        tls_error = std::string(what) + ": " + e;
    }
    catch (const char* e)
    {
        tls_error = std::string(what) + ": " + e;
    }
    catch (...)
    {
        tls_error = std::string(what) + ": unknown exception";
    }
    return false;
}

/* Releases the parameter lock however the scope is left. Dream's own code locks
   and unlocks by hand, which is fine until something in between throws — the
   lock would then stay held and the next `process` would deadlock the decoder
   rather than fail it. */
class CParamLock
{
public:
    explicit CParamLock(CParameter& p) : param(p) { param.Lock(); }
    ~CParamLock() { param.Unlock(); }

private:
    CParameter& param;
    CParamLock(const CParamLock&);
    CParamLock& operator=(const CParamLock&);
};

struct sdrx_drm_ring
{
    sdrx_drm_ring(size_t in_cap, size_t out_cap) : ring(in_cap, out_cap) {}
    CSdrxRing ring;
};

struct sdrx_drm
{
    CSettings settings;
    CDRMReceiver receiver;
    CSdrxRing* ring;

    explicit sdrx_drm(CSdrxRing* r) : settings(), receiver(&settings), ring(r) {}
};

/* --- ring ---------------------------------------------------------------- */

sdrx_drm_ring* sdrx_drm_ring_new(size_t in_capacity, size_t out_capacity)
{
    sdrx_drm_ring* r = nullptr;
    sdrx_guard("ring_new", [&] { r = new sdrx_drm_ring(in_capacity, out_capacity); });
    return r;
}

/* `sdrx_drm_ring_new` can now return NULL — the allocation is guarded like
   everything else here — so each of these tolerates one. */
void sdrx_drm_ring_free(sdrx_drm_ring* r)
{
    if (r == nullptr)
        return;
    sdrx_guard("ring_free", [&] { delete r; });
}

/* The four below only lock a mutex and copy shorts, so the guard is there to
   make "nothing in this file unwinds into Rust" true without exception rather
   than nearly true — a throwing `std::mutex::lock` is the only way in. */
size_t sdrx_drm_ring_push(sdrx_drm_ring* r, const int16_t* data, size_t n)
{
    if (r == nullptr)
        return 0;
    size_t dropped = 0;
    sdrx_guard("ring_push", [&] { dropped = r->ring.in.push(data, n); });
    return dropped;
}

size_t sdrx_drm_ring_pop(sdrx_drm_ring* r, int16_t* data, size_t n)
{
    if (r == nullptr)
        return 0;
    size_t got = 0;
    sdrx_guard("ring_pop", [&] { got = r->ring.out.pop(data, n); });
    return got;
}

size_t sdrx_drm_ring_out_available(sdrx_drm_ring* r)
{
    if (r == nullptr)
        return 0;
    size_t n = 0;
    sdrx_guard("ring_available", [&] { n = r->ring.out.available(); });
    return n;
}

void sdrx_drm_ring_stop(sdrx_drm_ring* r)
{
    if (r == nullptr)
        return;
    sdrx_guard("ring_stop", [&] { r->ring.in.stop(); });
}

/* --- receiver ------------------------------------------------------------ */

sdrx_drm* sdrx_drm_new(sdrx_drm_ring* r, const sdrx_drm_config* cfg)
{
    if (r == nullptr)
        return nullptr;
    sdrx_set_current_ring(&r->ring);

    sdrx_drm* h = nullptr;
    const bool ok = sdrx_guard("open", [&] {
        h = new sdrx_drm(&r->ring);

        CSettings& s = h->settings;
        s.Put("Receiver", "sampleratesig", int(cfg->sig_sample_rate));
        s.Put("Receiver", "samplerateaud", int(cfg->aud_sample_rate));
        /* CS_IQ_POS_ZERO: I and Q as they come off a zero-IF receiver, which
           Dream shifts to its own 6 kHz virtual IF. CS_MIX_CHAN averages the
           two channels, which is what a real-valued signal wants. */
        s.Put("Receiver", "inchansel", int(cfg->iq_input ? CS_IQ_POS_ZERO : CS_MIX_CHAN));
        s.Put("Receiver", "flipspectrum", cfg->flip_spectrum != 0);
        /* Empty means "the default device", which is the only device our
           sound shim enumerates. Anything else and CDRMReceiver::SetInputDevice
           would decide this is an RSCI network source instead. */
        s.Put("Receiver", "snddevin", std::string());
        s.Put("Receiver", "snddevout", std::string());
        /* Nothing here should touch the filesystem: no station schedules, no
           reception log, no MOT object cache. */
        s.Put("Receiver", "datafilesdirectory", std::string("."));
        s.Put("command", "mode", std::string("receive"));

        h->receiver.LoadSettings();
        h->receiver.SetReceiverMode(RM_DRM);
        h->receiver.InitReceiverMode();
        h->receiver.SetInStartMode();
    });
    if (!ok)
    {
        /* The destructor can throw in its own right once construction has gone
           wrong, and there is nothing left to salvage if it does. */
        sdrx_guard("open cleanup", [&] { delete h; });
        return nullptr;
    }
    return h;
}

void sdrx_drm_free(sdrx_drm* h)
{
    if (h == nullptr)
        return;
    sdrx_set_current_ring(h->ring);
    sdrx_guard("close", [&] { h->receiver.CloseSoundInterfaces(); });
    sdrx_guard("free", [&] { delete h; });
    sdrx_set_current_ring(nullptr);
}

int32_t sdrx_drm_process(sdrx_drm* h)
{
    sdrx_set_current_ring(h->ring);
    const bool ok = sdrx_guard("process", [&] {
        h->receiver.updatePosition();
        h->receiver.process();
    });
    return ok ? 0 : -1;
}

void sdrx_drm_restart(sdrx_drm* h)
{
    sdrx_set_current_ring(h->ring);
    sdrx_guard("restart", [&] {
        h->receiver.InitReceiverMode();
        h->receiver.SetInStartMode();
    });
}

void sdrx_drm_select_service(sdrx_drm* h, int32_t service)
{
    if (service < 0 || service >= MAX_NUM_SERVICES)
        return;
    sdrx_guard("select service", [&] {
        CParameter& p = *h->receiver.GetParameters();
        CParamLock lock(p);
        p.SetCurSelAudioService(int(service));
    });
}

/* 4-QAM, 16-QAM or 64-QAM, from the coding scheme the transmission signalled. */
static int32_t qam_order(ECodScheme scheme)
{
    switch (scheme)
    {
    case CS_1_SM: return 4;
    case CS_2_SM: return 16;
    default:      return 64;
    }
}

/* The body of sdrx_drm_constellation, so that entry point is just the guard. */
static int32_t constellation_body(sdrx_drm* h, int32_t channel, float* out,
                                  int32_t max_points, int32_t* qam)
{
    CVector<_COMPLEX> cells;
    CParameter& p = *h->receiver.GetParameters();

    ECodScheme sdc, msc;
    {
        CParamLock lock(p);
        sdc = p.eSDCCodingScheme;
        msc = p.eMSCCodingScheme;
    }

    switch (channel)
    {
    case SDRX_DRM_CHANNEL_FAC:
        /* The FAC is 4-QAM in every transmission there is; it has to be
           readable before anything says what the rest of the multiplex uses. */
        h->receiver.GetFACMLC()->GetVectorSpace(cells);
        if (qam) *qam = 4;
        break;
    case SDRX_DRM_CHANNEL_SDC:
        h->receiver.GetSDCMLC()->GetVectorSpace(cells);
        if (qam) *qam = qam_order(sdc);
        break;
    default:
        h->receiver.GetMSCMLC()->GetVectorSpace(cells);
        if (qam) *qam = qam_order(msc);
        break;
    }

    const int32_t have = int32_t(cells.Size());
    if (have <= 0)
        return 0;

    const int32_t want = have < max_points ? have : max_points;
    for (int32_t i = 0; i < want; i++)
    {
        /* Even stride over the whole frame — see the header. */
        const int32_t src = int32_t((int64_t(i) * have) / want);
        out[2 * i] = float(cells[src].real());
        out[2 * i + 1] = float(cells[src].imag());
    }
    return want;
}

int32_t sdrx_drm_constellation(sdrx_drm* h, int32_t channel, float* out,
                               int32_t max_points, int32_t* qam)
{
    if (out == nullptr || max_points <= 0)
        return 0;

    int32_t want = 0;
    sdrx_guard("constellation",
               [&] { want = constellation_body(h, channel, out, max_points, qam); });
    return want;
}

static void copy_str(char* dst, size_t cap, const std::string& src)
{
    size_t n = src.size() < cap - 1 ? src.size() : cap - 1;
    std::memcpy(dst, src.data(), n);
    dst[n] = '\0';
}

/* The body of sdrx_drm_get_status; the entry point is just the guard. */
static void status_body(sdrx_drm* h, sdrx_drm_status* out)
{
    CParameter& p = *h->receiver.GetParameters();
    CParamLock lock(p);

    ETypeRxStatus in_st = p.ReceiveStatus.InterfaceI.GetStatus();
    ETypeRxStatus out_st = p.ReceiveStatus.InterfaceO.GetStatus();
    /* Dream shows one IO light: the input's problem if it has one, else the
       output's. */
    out->io_status = int32_t(out_st == NOT_PRESENT ||
                             (in_st != NOT_PRESENT && in_st != RX_OK) ? in_st : out_st);
    out->time_sync_status = int32_t(p.ReceiveStatus.TSync.GetStatus());
    out->frame_sync_status = int32_t(p.ReceiveStatus.FSync.GetStatus());
    out->fac_status = int32_t(p.ReceiveStatus.FAC.GetStatus());
    out->sdc_status = int32_t(p.ReceiveStatus.SDC.GetStatus());
    out->audio_status = int32_t(p.ReceiveStatus.SLAudio.GetStatus());

    out->if_level_db = double(p.GetIFSignalLevel());
    out->has_signal = h->receiver.GetAcquiState() == AS_WITH_SIGNAL ? 1 : 0;
    out->audio_sample_rate_out = int32_t(p.GetAudSampleRate());

    if (out->has_signal)
    {
        out->snr_db = double(p.GetSNR());
        out->wmer_db = double(p.rWMERMSC);
        out->mer_db = double(p.rMER);
        out->dc_frequency_hz = double(p.GetDCFrequency());
        out->sample_offset_hz = double(p.rResampleOffset);
        if (p.rSigmaEstimate >= 0.0)
        {
            out->doppler_hz = double(p.rSigmaEstimate);
            out->delay_ms = double(p.rMinDelay);
        }

        ERobMode rm = p.GetWaveMode();
        if (rm != RM_NO_MODE_DETECTED)
            out->robustness_mode = int32_t(rm);
        out->spectrum_occupancy = int32_t(p.GetSpectrumOccup());
        out->interleaver_long = p.eSymbolInterlMode == CParameter::SI_LONG ? 1 : 0;
        out->sdc_scheme = int32_t(p.eSDCCodingScheme);
        out->msc_scheme = int32_t(p.eMSCCodingScheme);
        out->prot_level_a = int32_t(p.MSCPrLe.iPartA);
        out->prot_level_b = int32_t(p.MSCPrLe.iPartB);
        out->num_audio_services = int32_t(p.iNumAudioService);
        out->num_data_services = int32_t(p.iNumDataService);
        out->year = int32_t(p.iYear);
        out->month = int32_t(p.iMonth);
        out->day = int32_t(p.iDay);
        out->utc_hour = int32_t(p.iUTCHour);
        out->utc_minute = int32_t(p.iUTCMin);
    }

    const int cur = p.GetCurSelAudioService();
    out->cur_service = int32_t(cur);
    if (cur >= 0 && cur < MAX_NUM_SERVICES)
    {
        const CService& service = p.Service[cur];
        if (service.IsActive())
        {
            copy_str(out->label, sizeof(out->label), service.strLabel);
            copy_str(out->country_code, sizeof(out->country_code), service.strCountryCode);
            copy_str(out->language_code, sizeof(out->language_code), service.strLanguageCode);
            copy_str(out->text_message, sizeof(out->text_message),
                     service.AudioParam.strTextMessage);
            out->service_id = int32_t(service.iServiceID);
            out->bitrate_kbps =
                double(p.GetBitRateKbps(cur, service.eAudDataFlag != CService::SF_AUDIO));
            out->audio_codec = int32_t(service.AudioParam.eAudioCoding);
            out->audio_mode = int32_t(service.AudioParam.eAudioMode);
            out->audio_sample_rate = int32_t(service.AudioParam.eAudioSamplRate);
            out->is_stereo = service.AudioParam.eAudioMode == CAudioParam::AM_STEREO ? 1 : 0;
        }
    }
}

void sdrx_drm_get_status(sdrx_drm* h, sdrx_drm_status* out)
{
    std::memset(out, 0, sizeof(*out));
    out->robustness_mode = -1;
    out->spectrum_occupancy = -1;
    out->doppler_hz = -1.0;
    /* A failure leaves the zeroed struct, which reads as "nothing decoded". */
    sdrx_guard("status", [&] { status_body(h, out); });
}

int32_t sdrx_drm_test_throw(int32_t kind)
{
    const bool ok = sdrx_guard("test throw", [&] {
        switch (kind)
        {
        /* The one that matters: Dream's over-the-air parsers ask for
           allocations sized by the broadcast. */
        case 0: throw std::bad_alloc();
        case 1: throw CGenErr("deliberate");
        case 2: throw std::string("deliberate");
        case 3: throw "deliberate";
        default: throw 42;
        }
    });
    return ok ? 0 : -1;
}

/* One parser of each kind, kept between calls. The xHE-AAC parser buffers
   payload across super frames and does its border arithmetic relative to what
   it is still holding, so a parser rebuilt for every call would never reach
   half of the code the caller wants to fuzz. */
static thread_local XHEAACSuperFrame tls_xhe_superframe;
static thread_local AACSuperFrame tls_aac_superframe;

int32_t sdrx_drm_test_parse_superframe(int32_t kind, int32_t len_part_a,
                                       int32_t len_part_b, const uint8_t* bytes,
                                       int32_t len)
{
    int32_t frames = -1;
    const bool ok = sdrx_guard("test parse superframe", [&] {
        if (len_part_a < 0 || len_part_b < 0 || len < 0)
        {
            return;
        }

        CAudioParam param;
        ERobMode mode = RM_ROBUSTNESS_MODE_B;
        AudioSuperFrame* sf = nullptr;
        switch (kind)
        {
        case SDRX_DRM_SF_AAC_12KHZ:
            param.eAudioSamplRate = CAudioParam::AS_12KHZ;
            sf = &tls_aac_superframe;
            break;
        case SDRX_DRM_SF_AAC_24KHZ:
            param.eAudioSamplRate = CAudioParam::AS_24KHZ;
            sf = &tls_aac_superframe;
            break;
        case SDRX_DRM_SF_AAC_MODE_E:
            param.eAudioSamplRate = CAudioParam::AS_24KHZ;
            mode = RM_ROBUSTNESS_MODE_E;
            sf = &tls_aac_superframe;
            break;
        default:
            sf = &tls_xhe_superframe;
            break;
        }

        if (bytes == nullptr)
        {
            if (sf == &tls_xhe_superframe)
            {
                /* What CAudioSourceDecoder passes: the two protection parts
                   together are the super frame. */
                tls_xhe_superframe.init(param, unsigned(len_part_a + len_part_b));
            }
            else
            {
                tls_aac_superframe.init(param, mode, unsigned(len_part_a),
                                        unsigned(len_part_b));
            }
            frames = 0;
            return;
        }

        /* Dream's parsers read bits, MSB of each byte first. */
        CVectorEx<_BINARY> asf;
        asf.Init(len * 8);
        for (int32_t i = 0; i < len; i++)
        {
            for (int b = 0; b < 8; b++)
            {
                asf[i * 8 + b] = (bytes[i] >> (7 - b)) & 1;
            }
        }
        asf.ResetBitAccess();

        if (!sf->parse(asf))
        {
            frames = -2;
            return;
        }

        /* Read every frame back: getFrame indexes what parse() produced, and
           an index that outruns it is the same class of bug. */
        std::vector<uint8_t> frame;
        uint8_t crc = 0;
        const unsigned n = sf->getNumFrames();
        for (unsigned i = 0; i < n; i++)
        {
            sf->getFrame(frame, crc, i);
        }
        frames = int32_t(n);
    });
    return ok ? frames : -1;
}

const char* sdrx_drm_codec_version(void)
{
    /* Per thread, like the codec list it reads and like sdrx_drm_last_error:
       two decoders starting at once would otherwise race on this string. */
    static thread_local std::string version;
    if (version.empty())
    {
        /* GetDecoder indexes the codec list, which is empty until a receiver
           has been built — hence the null check, and hence the guard. */
        sdrx_guard("codec version", [&] {
            CAudioCodec* codec = CAudioCodec::GetDecoder(CAudioParam::AC_AAC, true);
            version = codec != nullptr ? codec->DecGetVersion() : std::string();
        });
    }
    return version.c_str();
}
