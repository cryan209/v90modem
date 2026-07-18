#include <stdint.h>
#include <stdio.h>

/*
 * Read-only link-time diagnostic hook for SmartLink's analogue-side V.90
 * Phase 4 receiver.  trn2dKnownDemod() calls SmartLink's own digital-side
 * Phase 4 generator and returns the reference sample that the equalizer
 * expects for the current TRN2d symbol.  Capturing those return values gives
 * us an implementation-independent oracle for the negotiated CPt mapper.
 *
 * Add an alias for the original method before linking this object first:
 *
 *   objcopy --add-symbol \
 *     _ZN20V90Phase4Demodulator15trn2dKnownDemodEs_original=.text:0x25de0,global,function \
 *     dsplibs.o dsplibs_trnref.o
 */

#define TRN2D_REFERENCE_SYMBOLS 6

extern int16_t smartlink_trn2d_known_original(void *demodulator,
                                              int16_t received)
    __asm__("_ZN20V90Phase4Demodulator15trn2dKnownDemodEs_original");

int16_t smartlink_trn2d_known_hook(void *demodulator, int16_t received)
    __asm__("_ZN20V90Phase4Demodulator15trn2dKnownDemodEs");

static int16_t reference_samples[TRN2D_REFERENCE_SYMBOLS];
static unsigned reference_count;
static int reference_written;

static void write_reference(void)
{
    FILE *fp;

    if (reference_written || reference_count == 0)
        return;
    fp = fopen("/tmp/smartlink-trn2d-reference.s16", "wb");
    if (!fp)
        return;
    (void)fwrite(reference_samples,
                 sizeof(reference_samples[0]),
                 reference_count,
                 fp);
    fclose(fp);
    reference_written = 1;
    fprintf(stderr,
            "TRN2REF wrote %u SmartLink reference symbols\n",
            reference_count);
}

int16_t
smartlink_trn2d_known_hook(void *demodulator, int16_t received)
{
    int16_t reference = smartlink_trn2d_known_original(demodulator, received);

    if (reference_count < TRN2D_REFERENCE_SYMBOLS)
        reference_samples[reference_count++] = reference;
    if (reference_count == TRN2D_REFERENCE_SYMBOLS)
        write_reference();
    return reference;
}
