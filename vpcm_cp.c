/*
 * vpcm_cp.c - Shared CP helpers for V.PCM-family modems
 */

#include "vpcm_cp.h"

#include <spandsp.h>
#include <spandsp/crc.h>

#include <stdio.h>
#include <string.h>

static void vpcm_cp_set_bit(uint8_t *bits, int bit, int value)
{
    bits[bit] = (uint8_t) (value & 1);
}

static void vpcm_cp_set_bits(uint8_t *bits, int first, int nbits, uint32_t value)
{
    int i;

    for (i = 0; i < nbits; i++)
        bits[first + i] = (uint8_t) ((value >> i) & 1U);
}

static uint32_t vpcm_cp_get_bits(const uint8_t *bits, int first, int nbits)
{
    uint32_t value;
    int i;

    value = 0;
    for (i = 0; i < nbits; i++)
        value |= (uint32_t) (bits[first + i] & 1U) << i;
    return value;
}

static bool vpcm_cp_crc_excluded_bit(int bit)
{
    if (bit <= 17)
        return true;  /* frame sync 0:16 and start bit 17 */
    if (bit == 34 || bit == 51 || bit == 68 || bit == 85
        || bit == 102 || bit == 119) {
        return true;
    }
    /* Each constellation-mask Uchord is a start bit plus 16 payload bits.
     * The final start bit before the CRC lies on the same 17-bit cadence. */
    return bit >= 136 && ((bit - 136) % 17) == 0;
}

static uint16_t vpcm_cp_crc_information(const uint8_t *bits, int crc_start)
{
    uint16_t crc;
    int i;

    crc = 0xFFFF;
    for (i = 0; i < crc_start; i++) {
        if (!vpcm_cp_crc_excluded_bit(i))
            crc = crc_itu16_bits(bits[i], 1, crc);
    }
    return crc;
}

static uint16_t vpcm_cp_crc_remainder(const uint8_t *bits, int crc_start)
{
    uint16_t crc = vpcm_cp_crc_information(bits, crc_start);

    for (int bit = 0; bit < 16; bit++)
        crc = crc_itu16_bits(bits[crc_start + bit], 1, crc);
    return crc;
}

void vpcm_cp_init(vpcm_cp_frame_t *cp)
{
    if (!cp)
        return;
    memset(cp, 0, sizeof(*cp));
    cp->v90_compatibility = true;
    cp->constellation_count = 1;
}

bool vpcm_cp_frames_equal(const vpcm_cp_frame_t *a, const vpcm_cp_frame_t *b)
{
    if (!a || !b)
        return false;
    return a->transparent_mode_granted == b->transparent_mode_granted
        && a->v90_compatibility == b->v90_compatibility
        && a->drn == b->drn
        && a->acknowledge == b->acknowledge
        && a->codec_alaw == b->codec_alaw
        && a->shaping_redundancy == b->shaping_redundancy
        && a->shaping_lookahead == b->shaping_lookahead
        && a->trn1d_gain_q3_13 == b->trn1d_gain_q3_13
        && a->shaping_a1_q1_6 == b->shaping_a1_q1_6
        && a->shaping_a2_q1_6 == b->shaping_a2_q1_6
        && a->shaping_b1_q1_6 == b->shaping_b1_q1_6
        && a->shaping_b2_q1_6 == b->shaping_b2_q1_6
        && a->upstream_rate_mask == b->upstream_rate_mask
        && a->constellation_count == b->constellation_count
        && a->codec_constellations_differ
           == b->codec_constellations_differ
        && memcmp(a->dfi, b->dfi, sizeof(a->dfi)) == 0
        && memcmp(a->masks,
                  b->masks,
                  (size_t)a->constellation_count * VPCM_CP_MASK_BYTES) == 0
        && (!a->codec_constellations_differ
            || memcmp(a->codec_masks,
                      b->codec_masks,
                      (size_t)a->constellation_count
                          * VPCM_CP_MASK_BYTES) == 0);
}

bool vpcm_cp_validate(const vpcm_cp_frame_t *cp, char *reason, size_t reason_len)
{
    const char *msg;
    int i;

    msg = NULL;
    if (!cp) {
        msg = "null CP frame";
    } else if (cp->drn > 28) {
        msg = "drn exceeds 28";
    } else if (cp->shaping_redundancy > 3) {
        msg = "shaping_redundancy out of range";
    } else if (cp->shaping_lookahead > 3) {
        msg = "shaping_lookahead out of range";
    } else if ((int8_t)cp->shaping_a1_q1_6 < -64
               || (int8_t)cp->shaping_a1_q1_6 > 64
               || (int8_t)cp->shaping_a2_q1_6 < -64
               || (int8_t)cp->shaping_a2_q1_6 > 64
               || (int8_t)cp->shaping_b1_q1_6 < -64
               || (int8_t)cp->shaping_b1_q1_6 > 64
               || (int8_t)cp->shaping_b2_q1_6 < -64
               || (int8_t)cp->shaping_b2_q1_6 > 64) {
        /* V.90 5.4.5.6 limits every signed-Q1.6 spectral-shape
         * coefficient to an absolute value no greater than one. */
        msg = "spectral shaping coefficient exceeds unity";
    } else if (cp->constellation_count < 1 || cp->constellation_count > VPCM_CP_MAX_CONSTELLATIONS) {
        msg = "constellation_count out of range";
    } else {
        for (i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
            if (cp->dfi[i] >= cp->constellation_count) {
                msg = "dfi index exceeds constellation_count";
                break;
            }
        }
    }

    if (reason && reason_len > 0) {
        if (msg)
            snprintf(reason, reason_len, "%s", msg);
        else
            reason[0] = '\0';
    }
    return (msg == NULL);
}

static int vpcm_cp_bit_length_fixed_fill(const vpcm_cp_frame_t *cp)
{
    int nbits;

    if (!cp || cp->constellation_count < 1 || cp->constellation_count > VPCM_CP_MAX_CONSTELLATIONS)
        return 0;

    nbits = 136 + 136 * cp->constellation_count;
    if (cp->codec_constellations_differ)
        nbits += 136 * cp->constellation_count;
    nbits += 17; /* CRC start bit + 16 CRC bits */
    nbits += 3;  /* V.90 Table 14 bits 289+d through 291+d */
    return nbits;
}

int vpcm_cp_bit_length(const vpcm_cp_frame_t *cp)
{
    return vpcm_cp_bit_length_fixed_fill(cp);
}

int vpcm_cp_modulated_bit_length(const vpcm_cp_frame_t *cp, int constellation_points)
{
    if (constellation_points != 4 && constellation_points != 16)
        return 0;
    return vpcm_cp_bit_length_fixed_fill(cp);
}

static bool vpcm_cp_encode_bits_fixed_fill(const vpcm_cp_frame_t *cp,
                                           uint8_t *bits_out,
                                           int *nbits_out)
{
    int nbits;
    int max_idx;
    int pos;
    int c;
    int u;
    uint16_t crc;

    if (!cp || !bits_out || !nbits_out || !vpcm_cp_validate(cp, NULL, 0))
        return false;

    nbits = vpcm_cp_bit_length_fixed_fill(cp);
    if (nbits <= 0 || nbits > VPCM_CP_MAX_BITS)
        return false;
    memset(bits_out, 0, (size_t) nbits);

    for (pos = 0; pos <= 16; pos++)
        bits_out[pos] = 1;
    vpcm_cp_set_bit(bits_out, 17, 0);
    vpcm_cp_set_bit(bits_out, 18, cp->transparent_mode_granted ? 1 : 0);
    vpcm_cp_set_bit(bits_out, 19, cp->v90_compatibility ? 1 : 0);
    vpcm_cp_set_bits(bits_out, 20, 5, cp->drn);
    vpcm_cp_set_bit(bits_out, 30, 0);
    vpcm_cp_set_bits(bits_out, 31, 2, cp->shaping_redundancy);
    vpcm_cp_set_bit(bits_out, 33, cp->acknowledge ? 1 : 0);
    vpcm_cp_set_bit(bits_out, 34, 0);
    vpcm_cp_set_bit(bits_out, 35, cp->codec_alaw ? 1 : 0);
    vpcm_cp_set_bits(bits_out, 36, 13, cp->upstream_rate_mask & 0x1FFFU);
    vpcm_cp_set_bits(bits_out, 49, 2, cp->shaping_lookahead);
    vpcm_cp_set_bit(bits_out, 51, 0);
    vpcm_cp_set_bits(bits_out, 52, 16, cp->trn1d_gain_q3_13);
    vpcm_cp_set_bit(bits_out, 68, 0);
    vpcm_cp_set_bits(bits_out, 69, 8, cp->shaping_a1_q1_6);
    vpcm_cp_set_bits(bits_out, 77, 8, cp->shaping_a2_q1_6);
    vpcm_cp_set_bit(bits_out, 85, 0);
    vpcm_cp_set_bits(bits_out, 86, 8, cp->shaping_b1_q1_6);
    vpcm_cp_set_bits(bits_out, 94, 8, cp->shaping_b2_q1_6);
    vpcm_cp_set_bit(bits_out, 102, 0);
    for (c = 0; c < VPCM_CP_FRAME_INTERVALS; c++)
        vpcm_cp_set_bits(bits_out, 103 + 4 * c + (c >= 4 ? 1 : 0), 4, cp->dfi[c]);
    vpcm_cp_set_bit(bits_out, 119, 0);
    vpcm_cp_set_bit(bits_out,
                    128,
                    cp->codec_constellations_differ ? 1 : 0);

    max_idx = cp->constellation_count - 1;
    pos = 136;
    for (c = 0; c <= max_idx; c++) {
        int uchord;

        for (uchord = 0; uchord < 8; uchord++) {
            vpcm_cp_set_bit(bits_out, pos++, 0);
            for (u = 0; u < 16; u++)
                vpcm_cp_set_bit(bits_out, pos++, vpcm_cp_mask_get(cp->masks[c], 16 * uchord + u) ? 1 : 0);
        }
    }
    if (cp->codec_constellations_differ) {
        for (c = 0; c <= max_idx; c++) {
            int uchord;

            for (uchord = 0; uchord < 8; uchord++) {
                vpcm_cp_set_bit(bits_out, pos++, 0);
                for (u = 0; u < 16; u++) {
                    vpcm_cp_set_bit(
                        bits_out,
                        pos++,
                        vpcm_cp_mask_get(cp->codec_masks[c],
                                         16 * uchord + u) ? 1 : 0);
                }
            }
        }
    }

    vpcm_cp_set_bit(bits_out, pos++, 0);
    crc = vpcm_cp_crc_information(bits_out, pos);
    vpcm_cp_set_bits(bits_out, pos, 16, crc);
    pos += 16;
    for (int fill = 0; fill < 3; fill++)
        bits_out[pos++] = 0;

    *nbits_out = pos;
    return true;
}


bool vpcm_cp_encode_bits(const vpcm_cp_frame_t *cp, uint8_t *bits_out, int *nbits_out)
{
    return vpcm_cp_encode_bits_fixed_fill(cp, bits_out, nbits_out);
}

bool vpcm_cp_encode_modulated_bits(const vpcm_cp_frame_t *cp,
                                   int constellation_points,
                                   uint8_t *bits_out,
                                   int *nbits_out)
{
    if (constellation_points != 4 && constellation_points != 16)
        return false;
    return vpcm_cp_encode_bits_fixed_fill(cp, bits_out, nbits_out);
}

bool vpcm_cp_decode_bits(const uint8_t *bits, int nbits, vpcm_cp_frame_t *cp_out)
{
    vpcm_cp_diag_t diag;

    if (!cp_out)
        return false;
    if (!vpcm_cp_decode_diag(bits, nbits, &diag))
        return false;
    *cp_out = diag.frame;
    return true;
}

bool vpcm_cp_build_diag(const vpcm_cp_frame_t *cp, vpcm_cp_diag_t *diag)
{
    if (!diag)
        return false;

    memset(diag, 0, sizeof(*diag));
    if (!vpcm_cp_encode_bits(cp, diag->bits, &diag->nbits))
        return false;
    diag->frame = *cp;
    /* Recompute the variable fields explicitly for clarity. */
    {
        int crc_start;

        crc_start = 136
                  + 136 * cp->constellation_count
                    * (cp->codec_constellations_differ ? 2 : 1)
                  + 1;
        diag->crc_field = (uint16_t) vpcm_cp_get_bits(diag->bits, crc_start, 16);
        diag->crc_remainder = vpcm_cp_crc_remainder(diag->bits, crc_start);
    }
    diag->frame_sync_ok = true;
    for (int i = 0; i <= 16; i++) {
        if (!diag->bits[i]) {
            diag->frame_sync_ok = false;
            break;
        }
    }
    diag->start_bits_ok = (diag->bits[17] == 0
                           && diag->bits[34] == 0
                           && diag->bits[51] == 0
                           && diag->bits[68] == 0
                           && diag->bits[85] == 0
                           && diag->bits[102] == 0
                           && diag->bits[119] == 0);
    diag->reserved_bits_ok = true;
    for (int i = 25; i <= 29; i++)
        diag->reserved_bits_ok = diag->reserved_bits_ok && (diag->bits[i] == 0);
    for (int i = 129; i <= 135; i++)
        diag->reserved_bits_ok = diag->reserved_bits_ok && (diag->bits[i] == 0);
    diag->v90_compat_ok = (diag->bits[19] == 1
                           && diag->bits[30] == 0);
    diag->fill_ok = true;
    for (int i = diag->nbits - 3; i < diag->nbits; i++) {
        if (diag->bits[i] != 0) {
            diag->fill_ok = false;
            break;
        }
    }
    diag->valid = (diag->frame_sync_ok
                   && diag->start_bits_ok
                   && diag->reserved_bits_ok
                   && diag->v90_compat_ok
                   && diag->fill_ok
                   && diag->crc_remainder == 0);
    return true;
}

bool vpcm_cp_decode_diag(const uint8_t *bits, int nbits, vpcm_cp_diag_t *diag)
{
    int i;
    int max_idx;
    int pos;

    if (!bits || !diag || nbits <= 0 || nbits > VPCM_CP_MAX_BITS)
        return false;

    memset(diag, 0, sizeof(*diag));
    memcpy(diag->bits, bits, (size_t) nbits);
    diag->nbits = nbits;
    vpcm_cp_init(&diag->frame);

    diag->frame_sync_ok = true;
    for (i = 0; i <= 16; i++) {
        if (!bits[i]) {
            diag->frame_sync_ok = false;
            break;
        }
    }

    diag->start_bits_ok = (bits[17] == 0
                           && bits[34] == 0
                           && bits[51] == 0
                           && bits[68] == 0
                           && bits[85] == 0
                           && bits[102] == 0
                           && bits[119] == 0);
    diag->reserved_bits_ok = true;
    for (i = 25; i <= 29; i++)
        diag->reserved_bits_ok = diag->reserved_bits_ok && (bits[i] == 0);
    for (i = 129; i <= 135; i++)
        diag->reserved_bits_ok = diag->reserved_bits_ok && (bits[i] == 0);
    diag->v90_compat_ok = (bits[30] == 0);

    diag->frame.transparent_mode_granted = (bits[18] != 0);
    diag->frame.v90_compatibility = (bits[19] != 0);
    diag->frame.drn = (uint8_t) vpcm_cp_get_bits(bits, 20, 5);
    diag->frame.acknowledge = (bits[33] != 0);
    diag->frame.codec_alaw = (bits[35] != 0);
    diag->frame.shaping_redundancy = (uint8_t)vpcm_cp_get_bits(bits, 31, 2);
    diag->frame.upstream_rate_mask = (uint16_t)vpcm_cp_get_bits(bits, 36, 13);
    diag->frame.shaping_lookahead = (uint8_t)vpcm_cp_get_bits(bits, 49, 2);
    diag->frame.trn1d_gain_q3_13 = (uint16_t)vpcm_cp_get_bits(bits, 52, 16);
    diag->frame.shaping_a1_q1_6 = (uint8_t)vpcm_cp_get_bits(bits, 69, 8);
    diag->frame.shaping_a2_q1_6 = (uint8_t)vpcm_cp_get_bits(bits, 77, 8);
    diag->frame.shaping_b1_q1_6 = (uint8_t)vpcm_cp_get_bits(bits, 86, 8);
    diag->frame.shaping_b2_q1_6 = (uint8_t)vpcm_cp_get_bits(bits, 94, 8);
    diag->frame.dfi[0] = (uint8_t) vpcm_cp_get_bits(bits, 103, 4);
    diag->frame.dfi[1] = (uint8_t) vpcm_cp_get_bits(bits, 107, 4);
    diag->frame.dfi[2] = (uint8_t) vpcm_cp_get_bits(bits, 111, 4);
    diag->frame.dfi[3] = (uint8_t) vpcm_cp_get_bits(bits, 115, 4);
    diag->frame.dfi[4] = (uint8_t) vpcm_cp_get_bits(bits, 120, 4);
    diag->frame.dfi[5] = (uint8_t) vpcm_cp_get_bits(bits, 124, 4);
    max_idx = 0;
    for (i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        if (diag->frame.dfi[i] > max_idx)
            max_idx = diag->frame.dfi[i];
    }
    diag->frame.constellation_count = (uint8_t) (max_idx + 1);
    diag->frame.codec_constellations_differ = bits[128] != 0;
    if (nbits != 156
                 + 136 * diag->frame.constellation_count
                   * (diag->frame.codec_constellations_differ ? 2 : 1))
        return false;

    pos = 136;
    for (i = 0; i <= max_idx; i++) {
        int uchord;
        int u;

        for (uchord = 0; uchord < 8; uchord++) {
            if (bits[pos++] != 0)
                diag->start_bits_ok = false;
            for (u = 0; u < 16; u++)
                vpcm_cp_mask_set(diag->frame.masks[i], 16 * uchord + u, bits[pos++] != 0);
        }
    }
    if (diag->frame.codec_constellations_differ) {
        for (i = 0; i <= max_idx; i++) {
            int uchord;
            int u;

            for (uchord = 0; uchord < 8; uchord++) {
                if (bits[pos++] != 0)
                    diag->start_bits_ok = false;
                for (u = 0; u < 16; u++) {
                    vpcm_cp_mask_set(diag->frame.codec_masks[i],
                                     16 * uchord + u,
                                     bits[pos++] != 0);
                }
            }
        }
    }
    if (bits[pos++] != 0)
        diag->start_bits_ok = false;
    diag->crc_field = (uint16_t) vpcm_cp_get_bits(bits, pos, 16);
    diag->crc_remainder = vpcm_cp_crc_remainder(bits, pos);
    pos += 16;
    diag->fill_ok = true;
    while (pos < nbits) {
        if (bits[pos] != 0)
            diag->fill_ok = false;
        pos++;
    }

    diag->valid = (diag->frame_sync_ok
                   && diag->start_bits_ok
                   && diag->reserved_bits_ok
                   && diag->v90_compat_ok
                   && diag->fill_ok
                   && diag->crc_remainder == 0
                   && vpcm_cp_validate(&diag->frame, NULL, 0));
    return diag->valid;
}

double vpcm_cp_drn_to_bps(uint8_t drn)
{
    if (drn == 0)
        return 0.0;
    return ((double) (drn + 20U) * 8000.0) / 6.0;
}

int vpcm_cp_drn_to_k(uint8_t drn)
{
    if (drn == 0 || drn > 28)
        return 0;
    return (int) drn + 14;
}

double vpcm_cp_robbed_bit_ceiling_bps(void)
{
    return (47.0 * 8000.0) / 6.0;
}

uint8_t vpcm_cp_recommended_robbed_bit_drn(void)
{
    /*
     * One stolen bit every sixth PCM codeword leaves a theoretical
     * ceiling of 47 bits per 6-codeword interval, but real robbed-bit
     * trunks are conventionally treated as 56 kbps-safe paths.
     */
    return 22;
}

void vpcm_cp_enable_all_ucodes(uint8_t mask[VPCM_CP_MASK_BYTES])
{
    if (!mask)
        return;
    memset(mask, 0xFF, VPCM_CP_MASK_BYTES);
}

void vpcm_cp_enable_odd_ucodes(uint8_t mask[VPCM_CP_MASK_BYTES])
{
    int ucode;

    if (!mask)
        return;
    memset(mask, 0, VPCM_CP_MASK_BYTES);
    for (ucode = 1; ucode < VPCM_CP_MASK_BITS; ucode += 2)
        vpcm_cp_mask_set(mask, ucode, true);
}

void vpcm_cp_init_robbed_bit_safe_profile(vpcm_cp_frame_t *cp,
                                          uint8_t drn,
                                          bool transparent_mode_granted)
{
    int ucode;

    if (!cp)
        return;

    vpcm_cp_init(cp);
    cp->transparent_mode_granted = transparent_mode_granted;
    cp->drn = drn;
    cp->constellation_count = 2;
    memset(cp->dfi, 0, sizeof(cp->dfi));
    cp->dfi[VPCM_CP_FRAME_INTERVALS - 1] = 1;
    memset(cp->masks[0], 0, VPCM_CP_MASK_BYTES);
    for (ucode = 0; ucode < VPCM_CP_MASK_BITS; ucode++)
        vpcm_cp_mask_set(cp->masks[0], ucode, true);
    /*
     * Robbed-bit trunks steal the LSB on every sixth PCM codeword, so
     * the robbed slot uses only odd Ucodes. Those survive LSB clearing
     * in both A-law and u-law while still leaving 64 choices (6 bits).
     * This profile is suitable family-wide for V.90/V.91/V.92-style
     * 56 kbps-safe CP negotiation on clean robbed-bit paths.
     */
    vpcm_cp_enable_odd_ucodes(cp->masks[1]);
}

bool vpcm_cp_apply_robbed_bit_safe_slots(vpcm_cp_frame_t *cp, uint8_t slot_mask)
{
    uint8_t odd_only[VPCM_CP_MASK_BYTES];
    uint8_t restricted[VPCM_CP_MASK_BYTES];
    int slot;
    int i;
    int idx;

    if (!cp || cp->constellation_count < 1 || cp->constellation_count > VPCM_CP_MAX_CONSTELLATIONS)
        return false;

    vpcm_cp_enable_odd_ucodes(odd_only);
    for (slot = 0; slot < VPCM_CP_FRAME_INTERVALS; slot++) {
        const uint8_t *current;

        if (!(slot_mask & (1U << slot)))
            continue;
        if (cp->dfi[slot] >= cp->constellation_count)
            return false;
        current = cp->masks[cp->dfi[slot]];
        for (i = 0; i < VPCM_CP_MASK_BYTES; i++)
            restricted[i] = (uint8_t) (current[i] & odd_only[i]);
        if (vpcm_cp_mask_population(restricted) <= 0)
            return false;
        if (memcmp(restricted, current, VPCM_CP_MASK_BYTES) == 0)
            continue;
        idx = -1;
        for (i = 0; i < cp->constellation_count; i++) {
            if (memcmp(cp->masks[i], restricted, VPCM_CP_MASK_BYTES) == 0) {
                idx = i;
                break;
            }
        }
        if (idx < 0) {
            if (cp->constellation_count >= VPCM_CP_MAX_CONSTELLATIONS)
                return false;
            idx = cp->constellation_count++;
            memcpy(cp->masks[idx], restricted, VPCM_CP_MASK_BYTES);
        }
        cp->dfi[slot] = (uint8_t) idx;
    }
    return true;
}

int vpcm_cp_select_ucode(const vpcm_cp_frame_t *cp, int frame_interval, bool prefer_high)
{
    int constellation_idx;
    int ucode;

    if (!cp || frame_interval < 0 || frame_interval >= VPCM_CP_FRAME_INTERVALS)
        return -1;
    if (cp->constellation_count < 1 || cp->constellation_count > VPCM_CP_MAX_CONSTELLATIONS)
        return -1;
    constellation_idx = cp->dfi[frame_interval];
    if (constellation_idx >= cp->constellation_count)
        return -1;

    if (prefer_high) {
        for (ucode = VPCM_CP_MASK_BITS - 1; ucode >= 0; ucode--) {
            if (vpcm_cp_mask_get(cp->masks[constellation_idx], ucode))
                return ucode;
        }
    } else {
        for (ucode = 0; ucode < VPCM_CP_MASK_BITS; ucode++) {
            if (vpcm_cp_mask_get(cp->masks[constellation_idx], ucode))
                return ucode;
        }
    }
    return -1;
}

void vpcm_cp_mask_set(uint8_t mask[VPCM_CP_MASK_BYTES], int ucode, bool enabled)
{
    int byte_idx;
    int bit_idx;

    if (ucode < 0 || ucode >= VPCM_CP_MASK_BITS)
        return;
    byte_idx = ucode >> 3;
    bit_idx = ucode & 7;
    if (enabled)
        mask[byte_idx] |= (uint8_t) (1U << bit_idx);
    else
        mask[byte_idx] &= (uint8_t) ~(1U << bit_idx);
}

bool vpcm_cp_mask_get(const uint8_t mask[VPCM_CP_MASK_BYTES], int ucode)
{
    int byte_idx;
    int bit_idx;

    if (ucode < 0 || ucode >= VPCM_CP_MASK_BITS)
        return false;
    byte_idx = ucode >> 3;
    bit_idx = ucode & 7;
    return ((mask[byte_idx] >> bit_idx) & 1U) != 0;
}

int vpcm_cp_mask_population(const uint8_t mask[VPCM_CP_MASK_BYTES])
{
    int count;
    int i;

    count = 0;
    for (i = 0; i < VPCM_CP_MASK_BYTES; i++)
        count += __builtin_popcount((unsigned int) mask[i]);
    return count;
}
