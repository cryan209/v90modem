/* Phase-4 transmit-symbol hook for the SmartLink DSP blob (dsplibs.o, i386).
 *
 * Interposes on V92Phase4Modulator::generateSymbol(), which returns each
 * upstream symbol (int16, 8 kHz) of V.92 Phase 4 and data -- E1u, TRN2u,
 * SUVu, CPu, E2u, B1u and data -- before slmodemd's 8000 -> 9600 resampler
 * and transmit FIR.  State is this+0 (30-way switch).  Three R_386_PC32 call
 * sites (V92Modulator::progress and ::reset), so the hook catches all.
 *
 *   /tmp/p4sym.bin  int16 pairs [state, symbol], one per call (P4SYM_MAX
 *                   pairs, default 4000000)
 *   /tmp/p4sym.txt  one line per state change
 *
 * Data mode does not go through generateSymbol: V92Modulator::progress calls
 * V92BitsToSymbol::process, which buffers and calls
 * V92Transmitter::process(this, uint8_t *bits, unsigned nbytes, short *out,
 * unsigned &written) (.text 0x54590, two call sites); `written` is exactly
 * the number of symbols stored.  Those are logged as state 100.  (Dumping
 * BitsToSymbol's `count` instead reads stale buffer: it is a request, not a
 * result.)
 *
 * Offsets for dsplibs.o e64b4a7 (AonCyberLabs): generateSymbol at .text 0x18050.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

extern short orig_gen(void *self) __asm__("_ZN18V92Phase4Modulator14generateSymbolEv_original");
short hook_gen(void *self) __asm__("_ZN18V92Phase4Modulator14generateSymbolEv");
extern void orig_tx(void *self, uint8_t *bits, unsigned nbytes, short *out, unsigned *written) __asm__("_ZN14V92Transmitter7processEPhjPsRj_original");
void hook_tx(void *self, uint8_t *bits, unsigned nbytes, short *out, unsigned *written) __asm__("_ZN14V92Transmitter7processEPhjPsRj");

static FILE *txt, *bin;
static long max_pairs = -1, n;
static int last_state = -1;

static void open_files(void)
{
    if (max_pairs < 0) {
        const char *e = getenv("P4SYM_MAX");
        max_pairs = e ? atol(e) : 4000000;
        txt = fopen("/tmp/p4sym.txt", "w");
        bin = fopen("/tmp/p4sym.bin", "wb");
    }
}

short hook_gen(void *self)
{
    int state = *(int32_t *)self;
    short sym = orig_gen(self);

    open_files();
    if (state != last_state && txt) {
        fprintf(txt, "%ld state %d -> %d (symbol %d)\n", n, last_state, state, sym);
        fflush(txt);
        last_state = state;
    }
    if (bin && n < max_pairs) {
        int16_t pair[2] = { (int16_t)state, sym };
        fwrite(pair, sizeof pair, 1, bin);
        if ((n & 255) == 0)
            fflush(bin);
    }
    n++;
    return sym;
}

void hook_tx(void *self, uint8_t *bits, unsigned nbytes, short *out, unsigned *written)
{
    orig_tx(self, bits, nbytes, out, written);
    open_files();
    if (last_state != 100 && txt) {
        fprintf(txt, "%ld data via V92Transmitter::process (%u bytes -> %u symbols)\n",
                n, nbytes, *written);
        fflush(txt);
        last_state = 100;
    }
    for (unsigned i = 0; bin && i < *written && n < max_pairs; i++, n++) {
        int16_t pair[2] = { 100, out[i] };
        fwrite(pair, sizeof pair, 1, bin);
    }
    if (bin && *written)
        fflush(bin);
}
