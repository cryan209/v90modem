/* V.FC c749..c788 symbol mapper; see vfc_mapper.h and docs/vfc_implementation.md.
 * Firmware addresses identify the proprietary rules; this is not ITU V.34.
 */
#include "vfc_mapper.h"

#include "vfc_mapper_tables.h"

bool vfc_mapper_init(vfc_mapper_t *s, unsigned code)
{
    if (!s || (code != 8 && code != 32))
        return false;
    *s = (vfc_mapper_t){ .code = code };
    return true;
}

bool vfc_mapper_put(vfc_mapper_t *s, unsigned index, unsigned dibit,
                    vfc_point_t *point)
{
    /* cb2d: output subset depends on the OLD trellis state and NEW quadrant. */
    static const uint8_t subsets[8] = {0, 5, 6, 1, 2, 7, 4, 3};
    static const uint16_t taps8[7] = {0x64, 0x101, 2, 0, 0, 0x82, 0x103};
    static const uint16_t taps32[7] = {0x70, 0x101, 2, 4, 10, 0x8a, 0x103};
    if (!s || !point || (s->code != 8 && s->code != 32)
        || s->trellis > 31 || s->quadrant > 3 || index > 159 || dibit > 3)
        return false;

    unsigned quadrant = (s->quadrant + dibit) & 3;
    unsigned subset = subsets[2 * quadrant + (s->trellis & 1)];
    uint16_t packed = vfc_quadrant[2 * index + (subset >> 2)];
    /* Signed high/low bytes, c761..c766. Avoid implementation-defined
     * conversion of unsigned bytes to int8_t. */
    int i = packed >> 8, q = packed & 255;
    if (i >= 128) i -= 256;
    if (q >= 128) q -= 256;
    switch (subset & 3) {
    case 0: *point = (vfc_point_t){i, q}; break;
    case 1: *point = (vfc_point_t){q, -i}; break;
    case 2: *point = (vfc_point_t){-i, -q}; break;
    case 3: *point = (vfc_point_t){-q, i}; break;
    }

    /* c76f..c788: one step per 2D symbol, including the nonlinear term.
     * cb57/cb5e are selected by negotiated 0873 bit 1, not by bit rate. */
    const uint16_t *taps = s->code == 32 ? taps32 : taps8;
    unsigned input = s->trellis | (quadrant << 5), b = 0;
    for (unsigned bit = 0; bit < 7; bit++)
        if (input & (1u << bit)) b ^= taps[bit];
    s->trellis = (b ^ ((b >> 7) & (b >> 5) & 3)) & 31;
    s->quadrant = quadrant;
    return true;
}
