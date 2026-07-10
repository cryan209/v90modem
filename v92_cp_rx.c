/*
 * v92_cp_rx.c — Native V.92 upstream Phase 4 control-frame receiver
 */

#include "v92_cp_rx.h"

#include <spandsp.h>
#include <spandsp/crc.h>

#include <string.h>

/* ---- Shared bit helpers ---- */

static uint16_t v92_cp_crc_through(const uint8_t *bits, int last_bit)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i <= last_bit; i++)
        crc = crc_itu16_bits(bits[i] & 1U, 1, crc);
    return crc;
}

static uint32_t v92_cp_get_bits(const uint8_t *bits, int first, int count)
{
    uint32_t value = 0;

    for (int i = 0; i < count; i++)
        value |= (uint32_t)(bits[first + i] & 1U) << i;
    return value;
}

static void v92_cp_set_bits(uint8_t *bits, int first, int count, uint32_t value)
{
    for (int i = 0; i < count; i++)
        bits[first + i] = (uint8_t)((value >> i) & 1U);
}

static bool v92_cp_bits_binary(const uint8_t *bits, int count)
{
    for (int i = 0; i < count; i++) {
        if (bits[i] > 1)
            return false;
    }
    return true;
}

/* Bits per symbol for the 4- or 8-point TRN2u constellations (Tables 28/29);
 * frames are filled to the next multiple of 12 symbols. */
static int v92_cp_fill_alignment(int constellation_points)
{
    if (constellation_points == 4)
        return 24;
    if (constellation_points == 8)
        return 36;
    return 0;
}

static int v92_cp_align_up(int nbits, int alignment)
{
    return ((nbits + alignment - 1) / alignment) * alignment;
}

static int v92_cp_max_dfi(const uint8_t dfi[VPCM_CP_FRAME_INTERVALS])
{
    int max_idx = 0;

    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        if (dfi[i] > max_idx)
            max_idx = dfi[i];
    }
    return max_idx;
}

/* delta parameter from Table 23: gamma covers constellations 1..max_idx,
 * doubled plus one more 136-bit set when the codec constellations differ. */
static int v92_cp_delta(int max_dfi, bool codec_differ)
{
    int gamma = 136 * max_dfi;

    return codec_differ ? (2 * gamma + 136) : gamma;
}

/* Raw Table 23 length: fields through bit 271, delta extension bits, the
 * final start bit, 16 CRC bits, and the mandatory fill bit at 289 + delta. */
static int v92_cp_raw_length(int max_dfi, bool codec_differ)
{
    return 290 + v92_cp_delta(max_dfi, codec_differ);
}

/* ---- Table 23 CPt/CPu ---- */

static bool v92_cp_frame_valid(const v92_cp_frame_t *cp)
{
    if (!cp || cp->type > V92_CP_TYPE_CPU || cp->drn > 22
        || cp->shaping_redundancy > 3 || cp->shaping_lookahead > 3)
        return false;
    if (cp->constellation_count < 1
        || cp->constellation_count > V92_CP_MAX_CONSTELLATIONS)
        return false;
    if (cp->constellation_count != v92_cp_max_dfi(cp->dfi) + 1)
        return false;
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        if (cp->dfi[i] >= cp->constellation_count)
            return false;
    }
    return true;
}

int v92_cp_bit_length(const v92_cp_frame_t *cp, int constellation_points)
{
    int alignment = v92_cp_fill_alignment(constellation_points);

    if (!v92_cp_frame_valid(cp) || alignment == 0)
        return 0;
    return v92_cp_align_up(v92_cp_raw_length(cp->constellation_count - 1,
                                             cp->codec_constellations_differ),
                           alignment);
}

static int v92_cp_put_mask(uint8_t *bits,
                           int pos,
                           const uint8_t mask[V92_CP_MASK_BYTES])
{
    for (int uchord = 0; uchord < 8; uchord++) {
        bits[pos++] = 0;
        for (int u = 0; u < 16; u++)
            bits[pos++] = vpcm_cp_mask_get(mask, 16 * uchord + u) ? 1 : 0;
    }
    return pos;
}

bool v92_cp_encode(const v92_cp_frame_t *cp,
                   int constellation_points,
                   uint8_t *bits_out,
                   int bits_max,
                   int *nbits_out)
{
    int nbits;
    int pos;
    uint16_t crc;

    nbits = v92_cp_bit_length(cp, constellation_points);
    if (nbits <= 0 || !bits_out || bits_max < nbits)
        return false;
    memset(bits_out, 0, (size_t)nbits);

    for (pos = 0; pos <= 16; pos++)
        bits_out[pos] = 1;
    v92_cp_set_bits(bits_out, 19, 2, cp->type);
    v92_cp_set_bits(bits_out, 21, 5, cp->drn);
    v92_cp_set_bits(bits_out, 31, 2, cp->shaping_redundancy);
    bits_out[33] = cp->acknowledge ? 1 : 0;
    bits_out[35] = cp->codec_alaw ? 1 : 0;
    v92_cp_set_bits(bits_out, 49, 2, cp->shaping_lookahead);
    v92_cp_set_bits(bits_out, 52, 16, cp->trn1d_gain_q3_13);
    v92_cp_set_bits(bits_out, 69, 8, cp->shaping_a1_q1_6);
    v92_cp_set_bits(bits_out, 77, 8, cp->shaping_a2_q1_6);
    v92_cp_set_bits(bits_out, 86, 8, cp->shaping_b1_q1_6);
    v92_cp_set_bits(bits_out, 94, 8, cp->shaping_b2_q1_6);
    v92_cp_set_bits(bits_out, 103, 4, cp->dfi[0]);
    v92_cp_set_bits(bits_out, 107, 4, cp->dfi[1]);
    v92_cp_set_bits(bits_out, 111, 4, cp->dfi[2]);
    v92_cp_set_bits(bits_out, 115, 4, cp->dfi[3]);
    v92_cp_set_bits(bits_out, 120, 4, cp->dfi[4]);
    v92_cp_set_bits(bits_out, 124, 4, cp->dfi[5]);
    bits_out[128] = cp->codec_constellations_differ ? 1 : 0;

    pos = 136;
    for (int c = 0; c < cp->constellation_count; c++)
        pos = v92_cp_put_mask(bits_out, pos, cp->masks[c]);
    if (cp->codec_constellations_differ) {
        for (int c = 0; c < cp->constellation_count; c++)
            pos = v92_cp_put_mask(bits_out, pos, cp->codec_masks[c]);
    }

    bits_out[pos++] = 0;
    crc = v92_cp_crc_through(bits_out, pos - 1);
    v92_cp_set_bits(bits_out, pos, 16, crc);

    if (nbits_out)
        *nbits_out = nbits;
    return true;
}

bool v92_cp_decode_diag(const uint8_t *bits, int nbits, v92_cp_diag_t *diag)
{
    v92_cp_diag_t local;
    int max_dfi;
    int pos;
    int crc_start;

    if (!bits || !diag || nbits < 136 || nbits > V92_CP_RX_MAX_BITS
        || (nbits % 12) != 0)
        return false;

    memset(&local, 0, sizeof(local));
    memcpy(local.bits, bits, (size_t)nbits);
    local.nbits = nbits;

    local.binary_bits_ok = v92_cp_bits_binary(bits, nbits);
    local.frame_sync_ok = true;
    for (int i = 0; i <= 16; i++) {
        if ((bits[i] & 1U) != 1U)
            local.frame_sync_ok = false;
    }
    local.frame.type = (uint8_t)v92_cp_get_bits(bits, 19, 2);
    local.identifier_ok = (bits[18] & 1U) == 0U
                       && local.frame.type <= V92_CP_TYPE_CPU;
    local.start_bits_ok = (bits[17] & 1U) == 0U
                       && (bits[34] & 1U) == 0U
                       && (bits[51] & 1U) == 0U
                       && (bits[68] & 1U) == 0U
                       && (bits[85] & 1U) == 0U
                       && (bits[102] & 1U) == 0U
                       && (bits[119] & 1U) == 0U;
    local.reserved_ok = true;
    for (int i = 26; i <= 30; i++)
        local.reserved_ok = local.reserved_ok && (bits[i] & 1U) == 0U;
    for (int i = 36; i <= 48; i++)
        local.reserved_ok = local.reserved_ok && (bits[i] & 1U) == 0U;
    for (int i = 129; i <= 135; i++)
        local.reserved_ok = local.reserved_ok && (bits[i] & 1U) == 0U;

    local.frame.drn = (uint8_t)v92_cp_get_bits(bits, 21, 5);
    local.frame.shaping_redundancy = (uint8_t)v92_cp_get_bits(bits, 31, 2);
    local.frame.acknowledge = (bits[33] & 1U) != 0U;
    local.frame.codec_alaw = (bits[35] & 1U) != 0U;
    local.frame.shaping_lookahead = (uint8_t)v92_cp_get_bits(bits, 49, 2);
    local.frame.trn1d_gain_q3_13 = (uint16_t)v92_cp_get_bits(bits, 52, 16);
    local.frame.shaping_a1_q1_6 = (uint8_t)v92_cp_get_bits(bits, 69, 8);
    local.frame.shaping_a2_q1_6 = (uint8_t)v92_cp_get_bits(bits, 77, 8);
    local.frame.shaping_b1_q1_6 = (uint8_t)v92_cp_get_bits(bits, 86, 8);
    local.frame.shaping_b2_q1_6 = (uint8_t)v92_cp_get_bits(bits, 94, 8);
    local.frame.dfi[0] = (uint8_t)v92_cp_get_bits(bits, 103, 4);
    local.frame.dfi[1] = (uint8_t)v92_cp_get_bits(bits, 107, 4);
    local.frame.dfi[2] = (uint8_t)v92_cp_get_bits(bits, 111, 4);
    local.frame.dfi[3] = (uint8_t)v92_cp_get_bits(bits, 115, 4);
    local.frame.dfi[4] = (uint8_t)v92_cp_get_bits(bits, 120, 4);
    local.frame.dfi[5] = (uint8_t)v92_cp_get_bits(bits, 124, 4);
    local.frame.codec_constellations_differ = (bits[128] & 1U) != 0U;

    max_dfi = v92_cp_max_dfi(local.frame.dfi);
    local.parameters_ok = local.frame.drn <= 22
                       && max_dfi < V92_CP_MAX_CONSTELLATIONS;
    if (!local.parameters_ok || !local.binary_bits_ok) {
        *diag = local;
        return false;
    }
    local.frame.constellation_count = (uint8_t)(max_dfi + 1);
    if (nbits < v92_cp_raw_length(max_dfi,
                                  local.frame.codec_constellations_differ)) {
        *diag = local;
        return false;
    }

    pos = 136;
    for (int c = 0; c <= max_dfi; c++) {
        for (int uchord = 0; uchord < 8; uchord++) {
            if ((bits[pos++] & 1U) != 0U)
                local.start_bits_ok = false;
            for (int u = 0; u < 16; u++)
                vpcm_cp_mask_set(local.frame.masks[c], 16 * uchord + u,
                                 (bits[pos++] & 1U) != 0U);
        }
    }
    if (local.frame.codec_constellations_differ) {
        for (int c = 0; c <= max_dfi; c++) {
            for (int uchord = 0; uchord < 8; uchord++) {
                if ((bits[pos++] & 1U) != 0U)
                    local.start_bits_ok = false;
                for (int u = 0; u < 16; u++)
                    vpcm_cp_mask_set(local.frame.codec_masks[c],
                                     16 * uchord + u,
                                     (bits[pos++] & 1U) != 0U);
            }
        }
    }

    if ((bits[pos] & 1U) != 0U)
        local.start_bits_ok = false;
    crc_start = pos + 1;
    local.crc_field = (uint16_t)v92_cp_get_bits(bits, crc_start, 16);
    local.crc_expected = v92_cp_crc_through(bits, crc_start - 1);
    local.crc_ok = local.crc_field == local.crc_expected;

    local.fill_bits_ok = true;
    for (int i = crc_start + 16; i < nbits; i++) {
        if ((bits[i] & 1U) != 0U)
            local.fill_bits_ok = false;
    }

    local.valid = local.binary_bits_ok
               && local.frame_sync_ok
               && local.identifier_ok
               && local.start_bits_ok
               && local.reserved_ok
               && local.parameters_ok
               && local.fill_bits_ok
               && local.crc_ok;
    *diag = local;
    return local.valid;
}

/* ---- Table 24 CPus ---- */

int v92_cpus_bit_length(int constellation_points)
{
    int alignment = v92_cp_fill_alignment(constellation_points);

    return alignment ? v92_cp_align_up(52, alignment) : 0;
}

bool v92_cpus_encode(const v92_cpus_frame_t *frame,
                     int constellation_points,
                     uint8_t *bits_out,
                     int bits_max,
                     int *nbits_out)
{
    int nbits = v92_cpus_bit_length(constellation_points);
    uint16_t crc;

    if (!frame || !bits_out || nbits <= 0 || bits_max < nbits
        || frame->drn > 22)
        return false;
    memset(bits_out, 0, (size_t)nbits);
    for (int i = 0; i <= 16; i++)
        bits_out[i] = 1;
    v92_cp_set_bits(bits_out, 19, 2, V92_CP_TYPE_CPUS);
    v92_cp_set_bits(bits_out, 21, 5, frame->drn);
    bits_out[33] = frame->acknowledge ? 1 : 0;
    crc = v92_cp_crc_through(bits_out, 34);
    v92_cp_set_bits(bits_out, 35, 16, crc);
    if (nbits_out)
        *nbits_out = nbits;
    return true;
}

bool v92_cpus_decode_diag(const uint8_t *bits, int nbits, v92_cpus_diag_t *diag)
{
    v92_cpus_diag_t local;

    if (!bits || !diag || nbits < 52 || nbits > V92_CPUS_MAX_BITS
        || (nbits % 12) != 0)
        return false;
    memset(&local, 0, sizeof(local));
    local.binary_bits_ok = v92_cp_bits_binary(bits, nbits);
    local.frame_sync_ok = true;
    for (int i = 0; i <= 16; i++) {
        if ((bits[i] & 1U) != 1U)
            local.frame_sync_ok = false;
    }
    local.identifier_ok = (bits[18] & 1U) == 0U
                       && v92_cp_get_bits(bits, 19, 2) == V92_CP_TYPE_CPUS;
    local.start_bits_ok = (bits[17] & 1U) == 0U
                       && (bits[34] & 1U) == 0U;
    local.reserved_ok = true;
    for (int i = 26; i <= 32; i++)
        local.reserved_ok = local.reserved_ok && (bits[i] & 1U) == 0U;
    local.frame.drn = (uint8_t)v92_cp_get_bits(bits, 21, 5);
    local.frame.acknowledge = (bits[33] & 1U) != 0U;
    local.parameters_ok = local.frame.drn <= 22;
    local.crc_field = (uint16_t)v92_cp_get_bits(bits, 35, 16);
    local.crc_expected = v92_cp_crc_through(bits, 34);
    local.crc_ok = local.crc_field == local.crc_expected;
    local.fill_bits_ok = true;
    for (int i = 51; i < nbits; i++) {
        if ((bits[i] & 1U) != 0U)
            local.fill_bits_ok = false;
    }
    local.valid = local.binary_bits_ok
               && local.frame_sync_ok
               && local.identifier_ok
               && local.start_bits_ok
               && local.reserved_ok
               && local.parameters_ok
               && local.fill_bits_ok
               && local.crc_ok;
    *diag = local;
    return local.valid;
}

/* ---- Table 27 SUVu ---- */

int v92_suvu_bit_length(int constellation_points)
{
    int alignment = v92_cp_fill_alignment(constellation_points);

    return alignment ? v92_cp_align_up(52, alignment) : 0;
}

bool v92_suvu_encode(const v92_suvu_frame_t *frame,
                     int constellation_points,
                     uint8_t *bits_out,
                     int bits_max,
                     int *nbits_out)
{
    int nbits = v92_suvu_bit_length(constellation_points);
    uint16_t crc;

    if (!frame || !bits_out || nbits <= 0 || bits_max < nbits
        || frame->prefilter_level_q2_2 > 31)
        return false;
    memset(bits_out, 0, (size_t)nbits);
    for (int i = 0; i <= 16; i++)
        bits_out[i] = 1;
    bits_out[18] = 1;
    bits_out[26] = frame->wait_for_cpu ? 1 : 0;
    v92_cp_set_bits(bits_out, 27, 5, frame->prefilter_level_q2_2);
    bits_out[32] = frame->silent_period_requested ? 1 : 0;
    bits_out[33] = frame->acknowledge ? 1 : 0;
    crc = v92_cp_crc_through(bits_out, 34);
    v92_cp_set_bits(bits_out, 35, 16, crc);
    if (nbits_out)
        *nbits_out = nbits;
    return true;
}

bool v92_suvu_decode_diag(const uint8_t *bits, int nbits, v92_suvu_diag_t *diag)
{
    v92_suvu_diag_t local;

    if (!bits || !diag || nbits < 52 || nbits > V92_SUVU_MAX_BITS
        || (nbits % 12) != 0)
        return false;
    memset(&local, 0, sizeof(local));
    local.binary_bits_ok = v92_cp_bits_binary(bits, nbits);
    local.frame_sync_ok = true;
    for (int i = 0; i <= 16; i++) {
        if ((bits[i] & 1U) != 1U)
            local.frame_sync_ok = false;
    }
    local.identifier_ok = (bits[18] & 1U) == 1U;
    local.start_bits_ok = (bits[17] & 1U) == 0U
                       && (bits[34] & 1U) == 0U;
    local.reserved_ok = true;
    for (int i = 19; i <= 25; i++)
        local.reserved_ok = local.reserved_ok && (bits[i] & 1U) == 0U;
    local.frame.wait_for_cpu = (bits[26] & 1U) != 0U;
    local.frame.prefilter_level_q2_2 = (uint8_t)v92_cp_get_bits(bits, 27, 5);
    local.frame.silent_period_requested = (bits[32] & 1U) != 0U;
    local.frame.acknowledge = (bits[33] & 1U) != 0U;
    local.crc_field = (uint16_t)v92_cp_get_bits(bits, 35, 16);
    local.crc_expected = v92_cp_crc_through(bits, 34);
    local.crc_ok = local.crc_field == local.crc_expected;
    local.fill_bits_ok = true;
    for (int i = 51; i < nbits; i++) {
        if ((bits[i] & 1U) != 0U)
            local.fill_bits_ok = false;
    }
    local.valid = local.binary_bits_ok
               && local.frame_sync_ok
               && local.identifier_ok
               && local.start_bits_ok
               && local.reserved_ok
               && local.fill_bits_ok
               && local.crc_ok;
    *diag = local;
    return local.valid;
}

/* ---- Conversion to the shared V.PCM CP representation ---- */

bool v92_cp_frame_to_vpcm(const v92_cp_frame_t *in, vpcm_cp_frame_t *out)
{
    if (!v92_cp_frame_valid(in) || !out)
        return false;
    vpcm_cp_init(out);
    out->transparent_mode_granted = false;
    out->v90_compatibility = (in->type == V92_CP_TYPE_CPU);
    out->drn = in->drn;
    out->acknowledge = in->acknowledge;
    out->codec_alaw = in->codec_alaw;
    out->shaping_redundancy = in->shaping_redundancy;
    out->shaping_lookahead = in->shaping_lookahead;
    out->trn1d_gain_q3_13 = in->trn1d_gain_q3_13;
    out->shaping_a1_q1_6 = in->shaping_a1_q1_6;
    out->shaping_a2_q1_6 = in->shaping_a2_q1_6;
    out->shaping_b1_q1_6 = in->shaping_b1_q1_6;
    out->shaping_b2_q1_6 = in->shaping_b2_q1_6;
    out->upstream_rate_mask = 0;
    out->constellation_count = in->constellation_count;
    memcpy(out->dfi, in->dfi, sizeof(out->dfi));
    memcpy(out->masks, in->masks, sizeof(out->masks));
    return true;
}

/* ---- Bit-level receiver ---- */

void v92_cp_rx_init(v92_cp_rx_t *rx,
                    int constellation_points,
                    bool expected_alaw,
                    v92_cp_rx_handler_t handler,
                    void *user_data)
{
    if (!rx)
        return;
    memset(rx, 0, sizeof(*rx));
    rx->constellation_points = constellation_points;
    rx->expected_alaw = expected_alaw;
    rx->handler = handler;
    rx->user_data = user_data;
}

void v92_cp_rx_reset(v92_cp_rx_t *rx)
{
    if (!rx)
        return;
    rx->bit_count = 0;
    rx->target_bits = 0;
    rx->sync_ones = 0;
    rx->collecting = false;
}

static bool v92_cp_rx_dispatch(v92_cp_rx_t *rx)
{
    if (rx->bits[18]) {
        v92_suvu_diag_t diag;

        if (v92_suvu_decode_diag(rx->bits, rx->bit_count, &diag)) {
            rx->valid_frames++;
            if (rx->handler)
                rx->handler(rx->user_data, V92_P4U_KIND_SUVU,
                            NULL, NULL, &diag);
            return true;
        }
        return false;
    }
    if (v92_cp_get_bits(rx->bits, 19, 2) == V92_CP_TYPE_CPUS) {
        v92_cpus_diag_t diag;

        if (v92_cpus_decode_diag(rx->bits, rx->bit_count, &diag)) {
            rx->valid_frames++;
            if (rx->handler)
                rx->handler(rx->user_data, V92_P4U_KIND_CPUS,
                            NULL, &diag, NULL);
            return true;
        }
        return false;
    }
    {
        v92_cp_diag_t diag;

        if (v92_cp_decode_diag(rx->bits, rx->bit_count, &diag)
            && diag.frame.codec_alaw == rx->expected_alaw) {
            rx->valid_frames++;
            if (rx->handler)
                rx->handler(rx->user_data,
                            diag.frame.type == V92_CP_TYPE_CPU
                                ? V92_P4U_KIND_CPU : V92_P4U_KIND_CPT,
                            &diag, NULL, NULL);
            return true;
        }
        return false;
    }
}

bool v92_cp_rx_put_bit(v92_cp_rx_t *rx, int bit)
{
    int alignment;
    bool accepted = false;

    if (!rx)
        return false;
    alignment = v92_cp_fill_alignment(rx->constellation_points);
    if (alignment == 0)
        return false;
    bit = bit ? 1 : 0;
    rx->input_bits++;

    if (!rx->collecting) {
        if (bit) {
            if (rx->sync_ones < 17)
                rx->sync_ones++;
            return false;
        }
        if (rx->sync_ones < 17) {
            rx->sync_ones = 0;
            return false;
        }
        memset(rx->bits, 0, sizeof(rx->bits));
        for (int i = 0; i < 17; i++)
            rx->bits[i] = 1;
        rx->bits[17] = 0;
        rx->bit_count = 18;
        rx->sync_ones = 0;
        rx->collecting = true;
        return false;
    }

    if (rx->bit_count >= V92_CP_RX_MAX_BITS) {
        rx->rejected_frames++;
        v92_cp_rx_reset(rx);
        return false;
    }
    rx->bits[rx->bit_count++] = (uint8_t)bit;

    if (rx->target_bits == 0 && rx->bit_count == 21) {
        if (rx->bits[18]) {
            rx->target_bits = v92_cp_align_up(52, alignment);
        } else {
            uint32_t type = v92_cp_get_bits(rx->bits, 19, 2);

            if (type == V92_CP_TYPE_CPUS) {
                rx->target_bits = v92_cp_align_up(52, alignment);
            } else if (type > V92_CP_TYPE_CPUS) {
                rx->rejected_frames++;
                v92_cp_rx_reset(rx);
                return false;
            }
            /* CPt/CPu length is known once the dfi fields arrive. */
        }
    }

    if (rx->target_bits == 0 && rx->bit_count == 129) {
        uint8_t dfi[VPCM_CP_FRAME_INTERVALS];
        int max_dfi;

        dfi[0] = (uint8_t)v92_cp_get_bits(rx->bits, 103, 4);
        dfi[1] = (uint8_t)v92_cp_get_bits(rx->bits, 107, 4);
        dfi[2] = (uint8_t)v92_cp_get_bits(rx->bits, 111, 4);
        dfi[3] = (uint8_t)v92_cp_get_bits(rx->bits, 115, 4);
        dfi[4] = (uint8_t)v92_cp_get_bits(rx->bits, 120, 4);
        dfi[5] = (uint8_t)v92_cp_get_bits(rx->bits, 124, 4);
        max_dfi = v92_cp_max_dfi(dfi);
        if (max_dfi >= V92_CP_MAX_CONSTELLATIONS) {
            rx->rejected_frames++;
            v92_cp_rx_reset(rx);
            return false;
        }
        rx->target_bits =
            v92_cp_align_up(v92_cp_raw_length(max_dfi, rx->bits[128] != 0),
                            alignment);
    }

    if (rx->target_bits > 0 && rx->bit_count == rx->target_bits) {
        accepted = v92_cp_rx_dispatch(rx);
        if (!accepted)
            rx->rejected_frames++;
        v92_cp_rx_reset(rx);
    }
    return accepted;
}
