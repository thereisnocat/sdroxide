/******************************************************************************\
 *
 * Description:
 *  Runtime loader for the FDK-AAC *decoder*, in the shape of Dream's own
 *  opus_dll.h.
 *
 *  This file is not part of upstream Dream. sdroxide adds it because the
 *  Fraunhofer FDK AAC licence cannot be combined with GPL-3.0-or-later: its
 *  no-fee clause and its explicit refusal of a patent grant are additional
 *  restrictions the GPL does not permit, which is why Debian ships the library
 *  in non-free. Linking it in would make the built binary undistributable, so
 *  sdroxide ships no Fraunhofer object code at all and looks the library up at
 *  run time instead. Present on the host, xHE-AAC decodes; absent, CanDecode
 *  says so and Dream falls back to its null codec exactly as it does for a
 *  missing libopus.
 *
 *  Only the seven decoder entry points are loaded. sdroxide never transmits
 *  DRM, so the encoder half of fdk_aac_codec.cpp is compiled out.
 *
 *  The declarations themselves come from the real upstream headers under
 *  vendor/fdk-aac/include, so the CStreamInfo layout this reads through is
 *  Fraunhofer's own rather than something transcribed by hand.
 *
\******************************************************************************/

#ifndef FDK_AAC_DLL_H_
#define FDK_AAC_DLL_H_

#include <fdk-aac/aacdecoder_lib.h>
#include "../util/LibraryLoader.h"

typedef HANDLE_AACDECODER(aacDecoder_Open_t)(TRANSPORT_TYPE transportFmt,
                                             UINT nrOfLayers);
typedef void(aacDecoder_Close_t)(HANDLE_AACDECODER self);
typedef AAC_DECODER_ERROR(aacDecoder_ConfigRaw_t)(HANDLE_AACDECODER self,
                                                  UCHAR *conf[],
                                                  const UINT length[]);
typedef AAC_DECODER_ERROR(aacDecoder_Fill_t)(HANDLE_AACDECODER self,
                                             UCHAR *pBuffer[],
                                             const UINT bufferSize[],
                                             UINT *bytesValid);
typedef AAC_DECODER_ERROR(aacDecoder_DecodeFrame_t)(HANDLE_AACDECODER self,
                                                    INT_PCM *pTimeData,
                                                    const INT timeDataSize,
                                                    const UINT flags);
typedef CStreamInfo *(aacDecoder_GetStreamInfo_t)(HANDLE_AACDECODER self);
typedef INT(aacDecoder_GetLibInfo_t)(LIB_INFO *info);

static void *hFdkAacLib;
static aacDecoder_Open_t *p_aacDecoder_Open;
static aacDecoder_Close_t *p_aacDecoder_Close;
static aacDecoder_ConfigRaw_t *p_aacDecoder_ConfigRaw;
static aacDecoder_Fill_t *p_aacDecoder_Fill;
static aacDecoder_DecodeFrame_t *p_aacDecoder_DecodeFrame;
static aacDecoder_GetStreamInfo_t *p_aacDecoder_GetStreamInfo;
static aacDecoder_GetLibInfo_t *p_aacDecoder_GetLibInfo;

static const LIBFUNC FdkAacLibFuncs[] = {
    {"aacDecoder_Close", (void **)&p_aacDecoder_Close, (void *)nullptr},
    {"aacDecoder_ConfigRaw", (void **)&p_aacDecoder_ConfigRaw, (void *)nullptr},
    {"aacDecoder_DecodeFrame", (void **)&p_aacDecoder_DecodeFrame, (void *)nullptr},
    {"aacDecoder_Fill", (void **)&p_aacDecoder_Fill, (void *)nullptr},
    {"aacDecoder_GetLibInfo", (void **)&p_aacDecoder_GetLibInfo, (void *)nullptr},
    {"aacDecoder_GetStreamInfo", (void **)&p_aacDecoder_GetStreamInfo, (void *)nullptr},
    {"aacDecoder_Open", (void **)&p_aacDecoder_Open, (void *)nullptr},
    {nullptr, nullptr, nullptr}};

/* Only the version 2 sonames. Version 1 has no USAC decoder at all, and its
   CStreamInfo is not the struct these headers describe - loading it would read
   the wrong fields rather than fail. */
#if defined(_WIN32)
static const char *FdkAacLibNames[] = {"libfdk-aac-2.dll", "libfdk-aac.dll",
                                       "fdk-aac.dll", nullptr};
#elif defined(__APPLE__)
/* LOADLIB is a bare dlopen(), and neither Homebrew prefix is on the default
   dyld search path, so both are named outright. */
static const char *FdkAacLibNames[] = {"libfdk-aac.2.dylib",
                                       "/opt/homebrew/lib/libfdk-aac.2.dylib",
                                       "/usr/local/lib/libfdk-aac.2.dylib",
                                       nullptr};
#else
static const char *FdkAacLibNames[] = {"libfdk-aac.so.2", nullptr};
#endif

/* Idempotent: the first codec constructed on this thread loads the library,
   and it is never unloaded - the function pointers are global and another
   receiver may still be decoding through them. */
static inline bool FdkAacDllLoad()
{
    if (hFdkAacLib == nullptr)
        hFdkAacLib = CLibraryLoader::Load(FdkAacLibNames, FdkAacLibFuncs);
    return hFdkAacLib != nullptr;
}

#define aacDecoder_Open (*p_aacDecoder_Open)
#define aacDecoder_Close (*p_aacDecoder_Close)
#define aacDecoder_ConfigRaw (*p_aacDecoder_ConfigRaw)
#define aacDecoder_Fill (*p_aacDecoder_Fill)
#define aacDecoder_DecodeFrame (*p_aacDecoder_DecodeFrame)
#define aacDecoder_GetStreamInfo (*p_aacDecoder_GetStreamInfo)
#define aacDecoder_GetLibInfo (*p_aacDecoder_GetLibInfo)

#endif // FDK_AAC_DLL_H_
