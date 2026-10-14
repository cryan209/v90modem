/* Dumps V90Equalizer::calcMeanErrorStatistics() of the SmartLink DSP blob
 * (dsplibs.o, i386; .text 0x388c0).  It averages the equaliser's
 * error-energy history (float array at this+0x98, this+0x9c entries, or 300
 * when this+0xa0 is set) into this+0x84.  V90Equalizer::process calls it twice
 * in Phase 4: when the Phase 4 demodulator raises its first event the mean is
 * kept as the "before" energy (this+0x13c) and the history is cleared; at the
 * second, the new mean is the "after" energy, and before/after is
 * ph4MeanErrorEnergyBeforeToAfterUpdateRatio.  Each call appends to
 * /tmp/p4err.txt: a header (call number, entries, mean, saved before) and the
 * history itself. */
#include <stdio.h>
#include <stdint.h>

extern float orig_cmes(void *self) __asm__("_ZN12V90Equalizer23calcMeanErrorStatisticsEv_original");
float hook_cmes(void *self) __asm__("_ZN12V90Equalizer23calcMeanErrorStatisticsEv");

#define F(p, o) (*(float *)((char *)(p) + (o)))
#define I(p, o) (*(int32_t *)((char *)(p) + (o)))

float hook_cmes(void *self)
{
    static FILE *f;
    static int calls;
    int n9c = I(self, 0x9c), na0 = I(self, 0xa0);
    int n = n9c ? n9c : (na0 ? 300 : 0);
    float r = orig_cmes(self);
    const float *h = *(const float **)((char *)self + 0x98);

    if (!f)
        f = fopen("/tmp/p4err.txt", "a");
    if (f) {
        if (n > 300 || n < 0)
            n = 300;
        fprintf(f, "call %d entries %d (0x9c %d, 0xa0 %d) mean %.4f min %.4f max %.4f saved-before %.4f ret %.4f\n",
                ++calls, n, n9c, na0, F(self, 0x84), F(self, 0x8c), F(self, 0x88), F(self, 0x13c), r);
        for (int i = 0; i < n; i++)
            fprintf(f, "%.3f%c", h[i], (i % 20 == 19 || i == n - 1) ? '\n' : ' ');
        fflush(f);
    }
    return r;
}
