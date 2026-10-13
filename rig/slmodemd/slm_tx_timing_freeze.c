/* SLM_FREEZE_TX_TIMING=<value> for the SmartLink DSP blob (dsplibs.o, i386).
 *
 * V92Modulator::progress hands its upstream resampler the timing offset its
 * downstream receiver estimated, every ~18 ms block, through
 * ResamplerTimingOffset::setTimingOffset(float) (.text 0x35510, 12 call
 * sites).  On the SIP rig both modems run off one clock, so the true offset
 * is zero; slmodemd estimates ~+0.57 ppm, and the slow transmit-phase drift
 * that follows makes its 8000->9600 transmit resampler's error near Nyquist
 * grow and fade over tens of seconds -- against its own transmitted symbols
 * the best linear fit of our received upstream went 0.033 -> 0.13 LU, 81% of
 * the residual in 3800-4000 Hz, and the upstream lost most frames.  With the
 * offset held at 0: 35 DS0 for the whole call, 55 rejected frames in 90 s.
 * Unset, the blob's own offset passes through. */
#include <stdlib.h>

extern void orig_sto(void *self, float v) __asm__("_ZN21ResamplerTimingOffset15setTimingOffsetEf_original");
void hook_sto(void *self, float v) __asm__("_ZN21ResamplerTimingOffset15setTimingOffsetEf");

void hook_sto(void *self, float v)
{
    static int freeze = -1;
    static float frozen;

    if (freeze < 0) {
        const char *e = getenv("SLM_FREEZE_TX_TIMING");

        freeze = e != NULL && *e != 0;
        frozen = freeze ? (float)atof(e) : 0.0f;
    }
    orig_sto(self, freeze ? frozen : v);
}
