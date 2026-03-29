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

static uint16_t vpcm_cp_crc_bits(const uint8_t *bits, int nbits)
{
    uint16_t crc;
    int i;

    crc = 0xFFFF;
    for (i = 0; i < nbits; i++)
        crc = crc_itu16_bits(bits[i], 1, crc);
    return crc;
}

void vpcm_cp_init(vpcm_cp_frame_t *cp)
{
    memset(cp, 0, sizeof(*cp));
    cp->v90_compatibility = true;
    cp->constellation_count = 1;
}

bool vpcm_cp_validate(const vpcm_cp_frame_t *cp, char *reason, size_t reason_len)
{
    const char *msg;
    int i;

    msg = NULL;
    if (cp->drn > 28) {
        msg = "drn exceeds 28";
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

int vpcm_cp_bit_length(const vpcm_cp_frame_t *cp)
{
    int nbits;

    if (!cp || cp->constellation_count < 1 || cp->constellation_count > VPCM_CP_MAX_CONSTELLATIONS)
        return 0;

    nbits = 136 + 136 * cp->constellation_count;
    nbits += 17; /* CRC start bit + 16 CRC bits */
    while ((nbits % 6) != 0)
        nbits++;
    return nbits;
}

bool vpcm_cp_encode_bits(const vpcm_cp_frame_t *cp, uint8_t *bits_out, int *nbits_out)
{
    int nbits;
    int max_idx;
    int pos;
    int c;
    int u;
    uint16_t crc;

    if (!cp || !bits_out || !nbits_out || !vpcm_cp_validate(cp, NULL, 0))
        return false;

    nbits = vpcm_cp_bit_length(cp);
    if (nbits <= 0 || nbits > VPCM_CP_MAX_BITS)
        return false;
    memset(bits_out, 0, (size_t) nbits);

    for (pos = 0; pos <= 16; pos++)
        bits_out[pos] = 1;
    vpcm_cp_set_bit(bits_out, 17, 0);
    vpcm_cp_set_bit(bits_out, 18, cp->transparent_mode_granted ? 1 : 0);
    vpcm_cp_set_bit(bits_out, 19, cp->v90_compatibility ? 1 : 0);
    vpcm_cp_set_bits(bits_out, 20, 5, cp->drn);
    vpcm_cp_set_bits(bits_out, 30, 3, 0);
    vpcm_cp_set_bit(bits_out, 33, cp->acknowledge ? 1 : 0);
    vpcm_cp_set_bit(bits_out, 34, 0);
    vpcm_cp_set_bit(bits_out, 51, 0);
    vpcm_cp_set_bit(bits_out, 68, 0);
    vpcm_cp_set_bit(bits_out, 85, 0);
    vpcm_cp_set_bit(bits_out, 102, 0);
    for (c = 0; c < VPCM_CP_FRAME_INTERVALS; c++)
        vpcm_cp_set_bits(bits_out, 103 + 4 * c + (c >= 4 ? 1 : 0), 4, cp->dfi[c]);
    vpcm_cp_set_bit(bits_out, 119, 0);
    vpcm_cp_set_bit(bits_out, 128, 0);

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

    vpcm_cp_set_bit(bits_out, pos++, 0);
    crc = vpcm_cp_crc_bits(bits_out, pos);
    vpcm_cp_set_bits(bits_out, pos, 16, crc);
    pos += 16;
    while ((pos % 6) != 0)
        bits_out[pos++] = 0;

    *nbits_out = pos;
    return true;
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
    diag->crc_field = (uint16_t) vpcm_cp_get_bits(diag->bits, diag->nbits - 16 - ((diag->nbits % 6) ? 0 : 0) - ((diag->nbits > 0) ? ((6 - ((diag->nbits - 1) % 6 + 1)) % 6) : 0), 16);
    /* Recompute the variable fields explicitly for clarity. */
    {
        int crc_start;

        crc_start = 136 + 136 * cp->constellation_count + 1;
        diag->crc_field = (uint16_t) vpcm_cp_get_bits(diag->bits, crc_start, 16);
        diag->crc_remainder = vpcm_cp_crc_bits(diag->bits, crc_start + 16);
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
                           && diag->bits[119] == 0
                           && diag->bits[128] == 0);
    diag->v90_compat_ok = (diag->bits[19] == 1
                           && diag->bits[30] == 0
                           && diag->bits[31] == 0
                           && diag->bits[32] == 0
                           && diag->bits[128] == 0);
    diag->fill_ok = true;
    for (int i = diag->nbits - 1; i >= 0 && (i % 6) != 5; i--) {
        if (diag->bits[i] != 0) {
            diag->fill_ok = false;
            break;
        }
    }
    diag->valid = (diag->frame_sync_ok
                   && diag->start_bits_ok
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

    if (!bits || !diag || nbits <= 0 || nbits > VPCM_CP_MAX_BITS || (nbits % 6) != 0)
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
                           && bits[119] == 0
                           && bits[128] == 0);
    diag->v90_compat_ok = (bits[19] == 1
                           && bits[30] == 0
                           && bits[31] == 0
                           && bits[32] == 0
                           && bits[128] == 0);

    diag->frame.transparent_mode_granted = (bits[18] != 0);
    diag->frame.v90_compatibility = (bits[19] != 0);
    diag->frame.drn = (uint8_t) vpcm_cp_get_bits(bits, 20, 5);
    diag->frame.acknowledge = (bits[33] != 0);
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
    if (bits[pos++] != 0)
        diag->start_bits_ok = false;
    diag->crc_field = (uint16_t) vpcm_cp_get_bits(bits, pos, 16);
    diag->crc_remainder = vpcm_cp_crc_bits(bits, pos + 16);
    pos += 16;
    diag->fill_ok = true;
    while (pos < nbits) {
        if (bits[pos] != 0)
            diag->fill_ok = false;
        pos++;
    }

    diag->valid = (diag->frame_sync_ok
                   && diag->start_bits_ok
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
