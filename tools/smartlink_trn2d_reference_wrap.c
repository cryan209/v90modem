#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * Read-only link-time diagnostic hook for SmartLink's analogue-side V.90
 * Phase 4 receiver.
 *
 * trn2dKnownDemod() is the reference generator the equalizer trains against
 * during TRN2d.  Disassembly (2026-07-23, dsplibs.o .text 0x25de0) shows it
 * is DECISION-DIRECTED and pure:
 *
 *   reference = sign(received) * table[(idx - 1) mod 6][law(|received|)]
 *
 * with law() = linear2alaw/linear2ulaw (A-law: xor 0xD5; u-law: 0xFF - x)
 * and table = the per-interval 128-entry int16 level table produced by the
 * TRN2 design (findPadGain), at [[this+0x3514]] with the law flag at
 * +0xa95c of the same object.  The peer does NOT regenerate our transmitted
 * symbol sequence - signs come from the received samples themselves - so
 * Phase-4 Error Energy is a per-symbol slicer residual:
 * |received - nearest designed level|^2.
 *
 * The compiler inlined trn2dKnownDemod into getV90Decision/getV92Decision
 * (zero relocations against it in dsplibs.o; the out-of-line copy is dead
 * code), so interposing on it directly can never fire.  This wrap instead
 * interposes on the small dispatcher V90Phase4Demodulator::getDecision(short)
 * - called via R_386_PC32 relocations from V90Equalizer::process - and,
 * because trn2dKnownDemod is pure, calls the blob's own dead-but-linkable
 * out-of-line copy to compute the reference BEFORE forwarding, while the
 * symbol-index member still holds the value the inlined copy will use.
 *
 * Records are interleaved int16 pairs [received, reference] streamed to
 * /tmp/smartlink-trn2d-pairs.s16 across the whole call (every Phase 4
 * attempt and every Phase 4 state, concatenated; the reference is only
 * meaningful during TRN2 - segment offline against the call log and our
 * live-tx.g711).  TRN2D_REF_SYMBOLS in the environment caps the record
 * count (default 200000 = ~25 s of Phase 4 at 8000 symbols/s).
 *
 * Link recipe - alias the ORIGINAL dispatcher, then link this object first:
 *
 *   objcopy --add-symbol \
 *     _ZN20V90Phase4Demodulator11getDecisionEs_original=.text:0x27780,global,function \
 *     dsplibs.o dsplibs_trnref.o
 *   gcc -m32 -O2 -c smartlink_trn2d_reference_wrap.c -o trn2d_wrap.o
 *   ... link with trn2d_wrap.o before dsplibs_trnref.o and
 *   -Wl,--allow-multiple-definition so this getDecision definition wins.
 */

#define TRN2D_REF_SYMBOLS_DEFAULT 200000UL

extern int smartlink_getdecision_original(void *demodulator, int16_t received)
    __asm__("_ZN20V90Phase4Demodulator11getDecisionEs_original");

/* The blob's own out-of-line trn2dKnownDemod: dead code on the live path
 * (inlined at every use) but a complete, pure function we can call. */
extern int16_t smartlink_trn2d_known(void *demodulator, int16_t received)
    __asm__("_ZN20V90Phase4Demodulator15trn2dKnownDemodEs");

int smartlink_getdecision_hook(void *demodulator, int16_t received)
    __asm__("_ZN20V90Phase4Demodulator11getDecisionEs");

static FILE *record_file;
static int record_disabled;
static unsigned long record_count;
static unsigned long record_limit = TRN2D_REF_SYMBOLS_DEFAULT;

int
smartlink_getdecision_hook(void *demodulator, int16_t received)
{
    int16_t pair[2];

    if (!record_disabled) {
        if (!record_file) {
            const char *limit = getenv("TRN2D_REF_SYMBOLS");

            if (limit && *limit) {
                char *end;
                unsigned long parsed = strtoul(limit, &end, 10);

                if (end != limit && *end == '\0' && parsed > 0)
                    record_limit = parsed;
            }
            record_file = fopen("/tmp/smartlink-trn2d-pairs.s16", "wb");
            if (!record_file) {
                record_disabled = 1;
                fprintf(stderr,
                        "TRN2REF cannot open /tmp/smartlink-trn2d-pairs.s16\n");
            } else {
                fprintf(stderr,
                        "TRN2REF capturing up to %lu [received,reference] pairs\n",
                        record_limit);
            }
        }
        if (record_file) {
            pair[0] = received;
            pair[1] = smartlink_trn2d_known(demodulator, received);
            (void)fwrite(pair, sizeof(pair[0]), 2, record_file);
            record_count++;
            /* Flush often: calls end by retrain/timeout, not clean
             * shutdown. */
            if (record_count % 2048 == 0)
                fflush(record_file);
            if (record_count % 12000 == 0)
                fprintf(stderr, "TRN2REF %lu pairs captured\n", record_count);
            if (record_count >= record_limit) {
                fclose(record_file);
                record_file = NULL;
                record_disabled = 1;
                fprintf(stderr, "TRN2REF capture complete (%lu pairs)\n",
                        record_count);
            }
        }
    }
    return smartlink_getdecision_original(demodulator, received);
}
