/* Dumps the taps of V90Equalizer, the SmartLink DSP blob's downstream PCM
 * equaliser (dsplibs.o, i386), by wrapping
 * V90Equalizer::process(float*, unsigned, short*, float*, unsigned&)
 * (.text 0x38d80, one call site, in V90Demodulator::progress; returns void).
 *
 * Layout, from zeroLinearEquCoefs / zeroDfeCoefs / setLinearEquCoeff /
 * setDfeCoeff (.text 0x36800-0x3695e):
 *   +0x0c  linear tap count      +0x14  float *linear taps
 *   +0x38  DFE tap count         +0x40  float *DFE taps
 *   +0xb0  MMX mode; then int16 copies at +0xd4/+0xd8 (linear) and
 *          +0x114/+0x118 (DFE), count + 8 each, and the float taps may be stale.
 *
 * Every EQ_DUMP_EVERY calls (default 10) after the call returns, appends to
 * /tmp/eqtaps.txt a header line and the float taps (and the int16 copies in
 * MMX mode), and writes "<eqdump N>" to stderr so each dump can be ordered
 * against slmodemd's own log lines (Jd Detected, DIL, findPadGain).
 * EQ_DUMP_MAX (default 3000) bounds the file. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

extern void orig_eq_process(void *self, float *in, unsigned n, short *out, float *err, unsigned *cnt)
    __asm__("_ZN12V90Equalizer7processEPfjPsS0_Rj_original");
void hook_eq_process(void *self, float *in, unsigned n, short *out, float *err, unsigned *cnt)
    __asm__("_ZN12V90Equalizer7processEPfjPsS0_Rj");

#define U(p, o) (*(uint32_t *)((char *)(p) + (o)))
#define P(p, o) (*(void **)((char *)(p) + (o)))

static void dump_f(FILE *f, const char *tag, const float *t, unsigned n)
{
    fprintf(f, "%s", tag);
    for (unsigned i = 0; t && i < n; i++)
        fprintf(f, " %.6g", t[i]);
    fputc('\n', f);
}

static void dump_s(FILE *f, const char *tag, const int16_t *t, unsigned n)
{
    fprintf(f, "%s", tag);
    for (unsigned i = 0; t && i < n; i++)
        fprintf(f, " %d", t[i]);
    fputc('\n', f);
}

void hook_eq_process(void *self, float *in, unsigned n, short *out, float *err, unsigned *cnt)
{
    static FILE *f;
    static unsigned calls, dumps, every, max;
    unsigned nlin, ndfe, mmx;

    orig_eq_process(self, in, n, out, err, cnt);
    if (!every) {
        const char *e = getenv("EQ_DUMP_EVERY");
        const char *m = getenv("EQ_DUMP_MAX");

        every = (e && atoi(e) > 0) ? (unsigned)atoi(e) : 10;
        max = (m && atoi(m) > 0) ? (unsigned)atoi(m) : 3000;
    }
    if (calls++ % every || dumps >= max)
        return;
    if (!f)
        f = fopen("/tmp/eqtaps.txt", "a");
    if (!f)
        return;
    nlin = U(self, 0x0c);
    ndfe = U(self, 0x38);
    mmx = U(self, 0xb0);
    if (nlin > 512 || ndfe > 512)
        return;
    fprintf(f, "dump %u call %u n %u cnt %u self %p nlin %u ndfe %u mmx %u\n",
            dumps, calls - 1, n, cnt ? *cnt : 0, self, nlin, ndfe, mmx);
    dump_f(f, "lin", (const float *)P(self, 0x14), nlin);
    dump_f(f, "dfe", (const float *)P(self, 0x40), ndfe);
    if (mmx) {
        dump_s(f, "lin16a", (const int16_t *)P(self, 0xd4), nlin + 8);
        dump_s(f, "lin16b", (const int16_t *)P(self, 0xd8), nlin + 8);
        dump_s(f, "dfe16a", (const int16_t *)P(self, 0x114), ndfe + 8);
        dump_s(f, "dfe16b", (const int16_t *)P(self, 0x118), ndfe + 8);
    }
    fflush(f);
    fprintf(stderr, "<eqdump %u>\n", dumps);
    dumps++;
}
