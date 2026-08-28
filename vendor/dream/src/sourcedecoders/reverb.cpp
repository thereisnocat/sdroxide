#include "reverb.h"

Reverb::Reverb() : bAudioWasOK(false), bUseReverbEffect(false)
{
    /* Neither flag was initialised here or in Init(), and apply() reads
       bAudioWasOK on its very first call - so which of the fade-out, the
       fade-in and the mute ran on the first block after start-up was
       whatever the stack happened to hold. */
}

void Reverb::Init(int outputSampleRate, bool bUse)
{
    /* Clear reverberation object */
    AudioRev.Init(1.0 /* seconds delay */, outputSampleRate);
    AudioRev.Clear();
    bUseReverbEffect = bUse;
    bAudioWasOK = false;
    OldLeft.Init(0);
    OldRight.Init(0);
}

ETypeRxStatus Reverb::apply(bool bCurBlockOK, bool bCurBlockFaulty, CVector<_REAL>& CurLeft, CVector<_REAL>& CurRight)
{
    int iResOutBlockSize = CurLeft.Size();
    /* Every loop below indexes OldLeft/OldRight to *this* call's block size,
       and only the reverb branches were guarded against a mismatch - the
       fade-out at the top and the copy back to the caller at the bottom run
       either way. The buffers are re-sized to the current block at the end of
       each call, so a block that grows between calls used to read and write
       past their end. AAC never grows: its frame length is fixed. xHE-AAC
       takes it from the USAC config, and starts at zero because the frame
       count is not known until the first super frame has been parsed. */
    if(OldLeft.Size()!=iResOutBlockSize) OldLeft.Init(iResOutBlockSize, 0.0);
    if(OldRight.Size()!=iResOutBlockSize) OldRight.Init(iResOutBlockSize, 0.0);

    bool okToReverb = bUseReverbEffect;

    vector<_REAL> tempLeft, tempRight;
    tempLeft.resize(unsigned(iResOutBlockSize));
    tempRight.resize(unsigned(iResOutBlockSize));
    for (int i = 0; i < iResOutBlockSize; i++)
    {
        tempLeft[unsigned(i)] = CurLeft[i];
        tempRight[unsigned(i)] = CurRight[i];
    }

    ETypeRxStatus status = DATA_ERROR;
    if (bCurBlockOK == false)
    {
        if (bAudioWasOK)
        {
            /* Post message to show that CRC was wrong (yellow light) */
            status = DATA_ERROR;

            /* Fade-out old block to avoid "clicks" in audio. We use linear
               fading which gives a log-fading impression */
            for (int i = 0; i < iResOutBlockSize; i++)
            {
                /* Linear attenuation with time of OLD buffer */
                const _REAL rAtt = 1.0 - _REAL(i) / _REAL(iResOutBlockSize);

                OldLeft[i] *= rAtt;
                OldRight[i] *= rAtt;

                if (okToReverb)
                {
                    /* Fade in input signal for reverberation to avoid clicks */
                    const _REAL rAttRev = _REAL(i) / _REAL(iResOutBlockSize);

                    /* Cross-fade reverberation effect */
                    const _REAL rRevSam = (1.0 - rAtt) * AudioRev.ProcessSample(OldLeft[i] * rAttRev, OldRight[i] * rAttRev);

                    /* Mono reverbration signal */
                    OldLeft[i] += rRevSam;
                    OldRight[i] += rRevSam;
                }
            }

            /* Set flag to show that audio block was bad */
            bAudioWasOK = false;
        }
        else
        {
            status = CRC_ERROR;

            if (okToReverb)
            {
                /* Add Reverberation effect */
                for (int i = 0; i < iResOutBlockSize; i++)
                {
                    /* Mono reverberation signal */
                    OldLeft[i] = OldRight[i] = AudioRev.ProcessSample(0, 0);
                }
            }
        }

        /* Write zeros in current output buffer */
        for (int i = 0; i < iResOutBlockSize; i++)
        {
            tempLeft[unsigned(i)] = 0.0;
            tempRight[unsigned(i)] = 0.0;
        }
    }
    else
    {
        /* Increment correctly decoded audio blocks counter */
        if (bCurBlockFaulty) {
            status = DATA_ERROR;
        }
        else {
            status = RX_OK;
        }

        if (bAudioWasOK == false)
        {
            if (okToReverb)
            {
                /* Add "last" reverbration only to old block */
                for (int i = 0; i < iResOutBlockSize; i++)
                {
                    /* Mono reverberation signal */
                    OldLeft[i] = OldRight[i] = AudioRev.ProcessSample(OldLeft[i], OldRight[i]);
                }
            }

            /* Fade-in new block to avoid "clicks" in audio. We use linear
               fading which gives a log-fading impression */
            for (int i = 0; i < iResOutBlockSize; i++)
            {
                /* Linear attenuation with time */
                /* `i / iResOutBlockSize` is integer division and was zero for
                   every sample, so the fade-in multiplied the whole of the
                   first good block after a dropout by nothing at all. It never
                   showed because apply() took its buffers by value and the
                   result was discarded. */
                const _REAL rAtt = _REAL(i) / _REAL(iResOutBlockSize);

                tempLeft[unsigned(i)] *= rAtt;
                tempRight[unsigned(i)] *= rAtt;

                if (okToReverb)
                {
                    /* Cross-fade reverberation effect */
                    const _REAL rRevSam = (1.0 - rAtt) * AudioRev.ProcessSample(0, 0);

                    /* Mono reverberation signal */
                    tempLeft[unsigned(i)] += rRevSam;
                    tempRight[unsigned(i)] += rRevSam;
                }
            }

            /* Reset flag */
            bAudioWasOK = true;
        }
    }

    /* Store reverberated block into output */
    for (int i = 0; i < iResOutBlockSize; i++)
    {
        CurLeft[i] = OldLeft[i];
        CurRight[i] = OldRight[i];
    }

    /* Store current audio block for next time */
    OldLeft.Init(iResOutBlockSize);
    OldRight.Init(iResOutBlockSize);
    for (int i = 0; i < iResOutBlockSize; i++)
    {
        OldLeft[i] = tempLeft[i];
        OldRight[i] = tempRight[i];
    }

    return status;
}
