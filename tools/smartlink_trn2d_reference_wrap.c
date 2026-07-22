#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * Read-only link-time diagnostic hook for SmartLink's analogue-side V.90
 * Phase 4 receiver.  trn2dKnownDemod() calls SmartLink's own digital-side
 * Phase 4 generator and returns the reference sample that the equalizer
 * expects for the current TRN2d symbol.  Capturing those return values gives
 * us an implementation-independent oracle for the negotiated CPt mapper.
 *
 * Records are interleaved int16 pairs [received, reference] streamed to
 * /tmp/smartlink-trn2d-pairs.s16 across the whole call (every Phase 4
 * attempt, concatenated in call order; attempt boundaries show up as the
 * discontinuities where a fresh TRN2d restarts).  The received stream aligns
 * the capture against our live-tx.g711 TRN2d window; the reference stream is
 * the peer's symbol-by-symbol prediction of our transmitter, and
 * (received - reference)^2 reproduces its Error Energy in its own units — so
 * one capture separates sign disagreement (reference sign-flipped vs our
 * shaper), level error (scalar/per-Ucode offset) and frame misalignment.
 * TRN2D_REF_SYMBOLS in the environment caps the record count (default
 * 200000 ≈ 16 full 12000-symbol attempts).
 *
 * Add an alias for the original method before linking this object first:
 *
 *   objcopy --add-symbol \
 *     _ZN20V90Phase4Demodulator15trn2dKnownDemodEs_original=.text:0x25de0,global,function \
 *     dsplibs.o dsplibs_trnref.o
 */

#define TRN2D_REF_SYMBOLS_DEFAULT 200000UL

extern int16_t smartlink_trn2d_known_original(void *demodulator,
                                              int16_t received)
    __asm__("_ZN20V90Phase4Demodulator15trn2dKnownDemodEs_original");

int16_t smartlink_trn2d_known_hook(void *demodulator, int16_t received)
    __asm__("_ZN20V90Phase4Demodulator15trn2dKnownDemodEs");

static FILE *record_file;
static int record_disabled;
static unsigned long record_count;
static unsigned long record_limit = TRN2D_REF_SYMBOLS_DEFAULT;

int16_t
smartlink_trn2d_known_hook(void *demodulator, int16_t received)
{
    int16_t reference = smartlink_trn2d_known_original(demodulator, received);
    int16_t pair[2];

    if (record_disabled)
        return reference;
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
            return reference;
        }
        fprintf(stderr,
                "TRN2REF capturing up to %lu [received,reference] pairs\n",
                record_limit);
    }
    pair[0] = received;
    pair[1] = reference;
    (void)fwrite(pair, sizeof(pair[0]), 2, record_file);
    record_count++;
    /* Flush often: calls end by retrain/timeout, not clean shutdown. */
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
    return reference;
}
