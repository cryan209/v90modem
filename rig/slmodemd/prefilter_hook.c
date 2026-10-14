/* Logs, and optionally overrides, the receive pre-filter the SmartLink DSP
 * blob (dsplibs.o, i386) puts ahead of its V.90/V.92 downstream equaliser.
 *
 * V90Demodulator::progress runs FloatFIR::process (this pre-filter) -> Agc ->
 * V90Resampler -> V90Equalizer.  V90PreFilter is a FloatFIR whose
 * setFilter(unsigned index) (.text 0x45090; called from 0x1d552 and 0x1e046)
 * loads one of preFilterCoefType1/2 (20 taps, index 0..30) or Type3 (40 taps,
 * index 10..50), chosen by the codec entry in V90PreFilter::dataBase.  Index 0
 * is the identity; higher indices are increasingly strong high-band boosts
 * (Type1 index 30: x12 at 3.6 kHz at 9600 Hz), meant to undo an analogue
 * codec's roll-off -- which a SIP rig does not have.
 *
 * Every call is appended to /tmp/prefilter.txt (index asked for, index used,
 * this+0x14 codec entry, this+0x20 previous index, this+0x24 selection) and
 * echoed to stderr.  PREFILTER_FORCE=<n> replaces every requested index. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

extern void orig_set_filter(void *self, unsigned index) __asm__("_ZN12V90PreFilter9setFilterEj_original");
void hook_set_filter(void *self, unsigned index) __asm__("_ZN12V90PreFilter9setFilterEj");

#define I(p, o) (*(int32_t *)((char *)(p) + (o)))

void hook_set_filter(void *self, unsigned index)
{
    static FILE *f;
    const char *e = getenv("PREFILTER_FORCE");
    unsigned used = (e && *e) ? (unsigned)atoi(e) : index;

    if (!f)
        f = fopen("/tmp/prefilter.txt", "a");
    if (f) {
        fprintf(f, "setFilter asked %u used %u codec(0x14) %d prev(0x20) %d sel(0x24) %d\n",
                index, used, I(self, 0x14), I(self, 0x20), I(self, 0x24));
        fflush(f);
    }
    fprintf(stderr, "<prefilter asked %u used %u>\n", index, used);
    orig_set_filter(self, used);
}
