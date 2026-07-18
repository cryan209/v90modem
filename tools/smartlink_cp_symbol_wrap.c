#include <stdint.h>
#include <stdio.h>

/*
 * Link-time diagnostic wrapper for SmartLink's closed-source V.90 CP symbol
 * reader.  The object layout offsets below come from disassembling
 * VPcmFloModem::getV90CpBits(short *) in the SL8200 dsplibs object currently
 * used by the hardware interop rig.
 *
 * The SmartLink object calls this method internally, so GNU ld's --wrap does
 * not intercept it.  Add an alias for the original method to the SmartLink
 * object, then link this hook before that object with --allow-multiple-definition:
 *
 *   objcopy --add-symbol \
 *     _ZN12VPcmFloModem12getV90CpBitsEPs_original=.text:0xd2e0,global,function \
 *     dsplibs.o dsplibs_cpdiag.o
 *
 * The wrapper is deliberately read-only apart from calling the real method.
 */

extern int smartlink_get_cp_bits_original(void *modem, int16_t *packed_bits)
    __asm__("_ZN12VPcmFloModem12getV90CpBitsEPs_original");

int smartlink_get_cp_bits_hook(void *modem, int16_t *packed_bits)
    __asm__("_ZN12VPcmFloModem12getV90CpBitsEPs");

int
smartlink_get_cp_bits_hook(void *modem, int16_t *packed_bits)
{
    const uint8_t *state = (const uint8_t *) modem;
    uint16_t before_pos = *(const uint16_t *) (state + 0x1738);
    int16_t sequence_len = *(const int16_t *) (state + 0x7dcc);
    uint8_t bits_per_call = *(const uint8_t *) (state + 0x7dd2);
    int done;

    done = smartlink_get_cp_bits_original(modem, packed_bits);

    if (packed_bits && sequence_len > 0 && bits_per_call > 0 &&
        bits_per_call <= 16 && before_pos < (uint16_t) sequence_len) {
        unsigned packed = (uint16_t) *packed_bits;
        unsigned take = bits_per_call;

        if (before_pos + take > (uint16_t) sequence_len)
            take = (unsigned) sequence_len - before_pos;

        fprintf(stderr,
                "CPBIT pos=%u len=%d width=%u take=%u packed=0x%04x bits=",
                before_pos, sequence_len, bits_per_call, take, packed);
        for (unsigned i = 0; i < take; ++i)
            fputc((packed & (1u << i)) ? '1' : '0', stderr);
        fprintf(stderr, " done=%d\n", done);
    }

    if (done)
        fprintf(stderr, "CPBIT END len=%d\n", sequence_len);

    return done;
}
