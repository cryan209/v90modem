/* V.FC signature bit framing. See vfc_signature.h and docs/vfc_implementation.md.
 * Original firmware aa18..aa42 uses SPM=1: the 0xc0 product is doubled.
 */
#include "vfc_signature.h"

bool vfc_signature_encode(unsigned code, uint8_t dibits[32])
{
    if (!dibits || code > 15)
        return false;
    uint64_t packet = UINT64_C(0x4b434f52)
                    | ((uint64_t)(code * UINT32_C(0x01010101)) << 32);
    for (unsigned k = 0; k < 32; k++)
        dibits[k] = (packet >> (2 * k)) & 3;
    return true;
}

void vfc_signature_rx_init(vfc_signature_rx_t *s)
{
    if (s) *s = (vfc_signature_rx_t){0};
}

bool vfc_signature_rx_put(vfc_signature_rx_t *s, unsigned dibit,
                          bool v8_priority, vfc_signature_t *result)
{
    if (!s || !result || dibit > 3 || s->matched)
        return false;
    /* aa44 shifts the incoming pair through a 64-bit sliding window. */
    s->window = (s->window >> 2) | ((uint64_t)dibit << 62);
    if (s->dibits < 32) s->dibits++;
    if (s->dibits < 32 || (uint32_t)s->window != UINT32_C(0x4b434f52))
        return false;
    uint32_t body = s->window >> 32;
    unsigned byte = body & 255;
    if (body != byte * UINT32_C(0x01010101) || v8_priority)
        return false;
    /* aa35 masks the received byte to four bits; it does not require the
     * upper nibble to be zero. aa3d rejects a negative timer calculation. */
    unsigned code = byte & 15;
    if (code > 3)
        return false;
    *result = (vfc_signature_t){code, 2700 - 384 * code};
    s->matched = true;
    return true;
}
