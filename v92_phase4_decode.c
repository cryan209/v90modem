/* V.92 Phase 4 messages and procedure analysis. */

#include "v92_phase4_decode.h"

#include <spandsp.h>

#include <string.h>

static uint16_t v92_crc_through(const uint8_t *bits, int last_bit)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i <= last_bit; i++)
        crc = crc_itu16_bits(bits[i] & 1U, 1, crc);
    return crc;
}

static uint32_t v92_get_bits(const uint8_t *bits, int first, int count)
{
    uint32_t value = 0;

    for (int i = 0; i < count; i++)
        value |= (uint32_t)(bits[first + i] & 1U) << i;
    return value;
}

static void v92_set_bits(uint8_t *bits, int first, int count, uint32_t value)
{
    for (int i = 0; i < count; i++)
        bits[first + i] = (uint8_t)((value >> i) & 1U);
}

static bool v92_bits_binary(const uint8_t *bits, int count)
{
    for (int i = 0; i < count; i++) {
        if (bits[i] > 1)
            return false;
    }
    return true;
}

int v92_suvd_bit_length(int alignment)
{
    if (alignment < 1 || alignment > V92_SUVD_MAX_BITS)
        return 0;
    return ((52 + alignment - 1) / alignment) * alignment;
}

bool v92_suvd_encode_aligned(const v92_suvd_frame_t *frame,
                             int alignment,
                             uint8_t *bits,
                             int bits_max,
                             int *nbits_out)
{
    int nbits = v92_suvd_bit_length(alignment);
    uint16_t crc;

    if (!frame || !bits || nbits < 52 || bits_max < nbits)
        return false;
    memset(bits, 0, (size_t)nbits);
    for (int i = 0; i <= 16; i++)
        bits[i] = 1;
    bits[18] = 1;
    bits[32] = frame->silent_period_requested ? 1 : 0;
    bits[33] = frame->acknowledge ? 1 : 0;
    crc = v92_crc_through(bits, 34);
    for (int i = 0; i < 16; i++)
        bits[35 + i] = (uint8_t)((crc >> i) & 1U);
    if (nbits_out)
        *nbits_out = nbits;
    return true;
}

bool v92_suvd_encode(const v92_suvd_frame_t *frame,
                     uint8_t *bits,
                     int bits_max)
{
    return v92_suvd_encode_aligned(frame, 6, bits, bits_max, NULL);
}

bool v92_suvd_decode(const uint8_t *bits,
                     int bit_count,
                     v92_suvd_frame_t *frame,
                     v92_suvd_diag_t *diag)
{
    if (bit_count != V92_SUVD_BITS)
        return false;
    return v92_suvd_decode_bits(bits, bit_count, frame, diag);
}

bool v92_suvd_decode_bits(const uint8_t *bits,
                          int bit_count,
                          v92_suvd_frame_t *frame,
                          v92_suvd_diag_t *diag)
{
    v92_suvd_diag_t local;
    uint16_t crc_field = 0;

    if (!bits || bit_count < 52 || bit_count > V92_SUVD_MAX_BITS)
        return false;
    memset(&local, 0, sizeof(local));
    local.binary_bits_ok = v92_bits_binary(bits, bit_count);
    local.frame_sync_ok = true;
    for (int i = 0; i <= 16; i++) {
        if ((bits[i] & 1U) != 1U)
            local.frame_sync_ok = false;
    }
    local.identifier_ok = (bits[18] & 1U) == 1U;
    local.reserved_ok = true;
    for (int i = 19; i <= 31; i++) {
        if ((bits[i] & 1U) != 0U)
            local.reserved_ok = false;
    }
    local.start_bits_ok = (bits[17] & 1U) == 0U
                       && (bits[34] & 1U) == 0U;
    local.fill_bits_ok = true;
    for (int i = 51; i < bit_count; i++) {
        if ((bits[i] & 1U) != 0U)
            local.fill_bits_ok = false;
    }
    for (int i = 0; i < 16; i++)
        crc_field |= (uint16_t)(bits[35 + i] & 1U) << i;
    local.crc_field = crc_field;
    local.crc_expected = v92_crc_through(bits, 34);
    local.crc_ok = local.crc_field == local.crc_expected;
    local.valid = local.binary_bits_ok
               && local.frame_sync_ok
               && local.identifier_ok
               && local.reserved_ok
               && local.start_bits_ok
               && local.fill_bits_ok
               && local.crc_ok;
    if (frame) {
        frame->silent_period_requested = (bits[32] & 1U) != 0U;
        frame->acknowledge = (bits[33] & 1U) != 0U;
    }
    if (diag)
        *diag = local;
    return local.valid;
}

bool v92_cpd_base_encode(const v92_cpd_base_frame_t *frame,
                         uint8_t *bits,
                         int bits_max)
{
    uint16_t crc;

    if (!frame || !bits || bits_max < V92_CPD_BASE_BITS
        || frame->selected_upstream_drn > 19
        || frame->trellis_select > 2
        || frame->gain_q0_16 == 0)
        return false;
    memset(bits, 0, V92_CPD_BASE_BITS);
    for (int i = 0; i <= 16; i++)
        bits[i] = 1;
    v92_set_bits(bits, 22, 5, frame->selected_upstream_drn);
    v92_set_bits(bits, 27, 2, frame->trellis_select);
    bits[29] = frame->extend_e2u ? 1 : 0;
    bits[33] = frame->acknowledge ? 1 : 0;
    v92_set_bits(bits, 35, 16, frame->gain_q0_16);
    crc = v92_crc_through(bits, 51);
    v92_set_bits(bits, 52, 16, crc);
    return true;
}

bool v92_cpd_base_decode(const uint8_t *bits,
                         int bit_count,
                         v92_cpd_base_frame_t *frame,
                         v92_cpd_base_diag_t *diag)
{
    v92_cpd_base_diag_t local;
    v92_cpd_base_frame_t parsed;

    if (!bits || bit_count != V92_CPD_BASE_BITS)
        return false;
    memset(&local, 0, sizeof(local));
    memset(&parsed, 0, sizeof(parsed));
    local.binary_bits_ok = v92_bits_binary(bits, bit_count);
    local.frame_sync_ok = true;
    for (int i = 0; i <= 16; i++) {
        if ((bits[i] & 1U) != 1U)
            local.frame_sync_ok = false;
    }
    local.identifier_ok = (bits[18] & 1U) == 0U;
    local.optional_parts_absent = (bits[19] & 1U) == 0U
                               && (bits[20] & 1U) == 0U
                               && (bits[21] & 1U) == 0U;
    local.reserved_ok = (bits[30] & 1U) == 0U
                     && (bits[31] & 1U) == 0U
                     && (bits[32] & 1U) == 0U;
    local.start_bits_ok = (bits[17] & 1U) == 0U
                       && (bits[34] & 1U) == 0U
                       && (bits[51] & 1U) == 0U;
    parsed.selected_upstream_drn = (uint8_t)v92_get_bits(bits, 22, 5);
    parsed.trellis_select = (uint8_t)v92_get_bits(bits, 27, 2);
    parsed.extend_e2u = (bits[29] & 1U) != 0U;
    parsed.acknowledge = (bits[33] & 1U) != 0U;
    parsed.gain_q0_16 = (uint16_t)v92_get_bits(bits, 35, 16);
    local.parameters_ok = parsed.selected_upstream_drn <= 19
                       && parsed.trellis_select <= 2
                       && parsed.gain_q0_16 != 0;
    local.fill_bits_ok = true;
    for (int i = 68; i < V92_CPD_BASE_BITS; i++) {
        if ((bits[i] & 1U) != 0U)
            local.fill_bits_ok = false;
    }
    local.crc_field = (uint16_t)v92_get_bits(bits, 52, 16);
    local.crc_expected = v92_crc_through(bits, 51);
    local.crc_ok = local.crc_field == local.crc_expected;
    local.valid = local.binary_bits_ok
               && local.frame_sync_ok
               && local.identifier_ok
               && local.optional_parts_absent
               && local.reserved_ok
               && local.start_bits_ok
               && local.parameters_ok
               && local.fill_bits_ok
               && local.crc_ok;
    if (frame)
        *frame = parsed;
    if (diag)
        *diag = local;
    return local.valid;
}

/* ---- Full Table 30 CPd ---- */

static bool v92_cpd_frame_valid(const v92_cpd_frame_t *f)
{
    int nonzero_sets = 0;

    if (!f || f->selected_upstream_drn > 19 || f->trellis_select > 2
        || f->gain_q0_16 == 0)
        return false;
    if (f->coeffs_present) {
        if (f->lz1 > V92_CPD_MAX_TAPS || f->lp1 > V92_CPD_MAX_TAPS
            || f->lz2 > V92_CPD_MAX_TAPS || f->lp2 > V92_CPD_MAX_TAPS)
            return false;
    }
    if (f->constellations_present) {
        bool zero_seen = false;

        for (int i = 0; i < V92_CPD_MAX_SETS; i++) {
            if (f->set_sizes[i] > V92_CPD_MAX_POINTS)
                return false;
            if (f->set_sizes[i] == 0) {
                zero_seen = true;
            } else {
                /* Non-zero sets shall be listed first. */
                if (zero_seen)
                    return false;
                nonzero_sets++;
                /* Points listed smallest first; no zero point. */
                if (f->points[i][0] == 0)
                    return false;
                for (int p = 1; p < f->set_sizes[i]; p++) {
                    if (f->points[i][p] <= f->points[i][p - 1])
                        return false;
                }
            }
        }
        if (nonzero_sets == 0)
            return false;
        for (int i = 0; i < 6; i++) {
            if (f->dfi[i] >= nonzero_sets)
                return false;
        }
    }
    return true;
}

static int v92_cpd_raw_length(const v92_cpd_frame_t *f)
{
    int nbits = 51;

    if (f->modulus_present)
        nbits += 102;
    if (f->coeffs_present)
        nbits += 68 + 17 * (f->lz1 + f->lp1 + f->lz2 + f->lp2);
    if (f->constellations_present) {
        int total_points = 0;

        for (int i = 0; i < V92_CPD_MAX_SETS; i++)
            total_points += f->set_sizes[i];
        nbits += 85 + 17 * total_points;
    }
    /* Final start bit, CRC, and the mandatory fill bit. */
    return nbits + 18;
}

int v92_cpd_bit_length(const v92_cpd_frame_t *frame, int alignment)
{
    int raw;

    if (!v92_cpd_frame_valid(frame) || alignment < 1)
        return 0;
    raw = v92_cpd_raw_length(frame);
    raw = ((raw + alignment - 1) / alignment) * alignment;
    return (raw <= V92_CPD_MAX_BITS) ? raw : 0;
}

bool v92_cpd_encode(const v92_cpd_frame_t *frame,
                    int alignment,
                    uint8_t *bits,
                    int bits_max,
                    int *nbits_out)
{
    int nbits;
    int pos;
    uint16_t crc;

    nbits = v92_cpd_bit_length(frame, alignment);
    if (nbits <= 0 || !bits || bits_max < nbits)
        return false;
    memset(bits, 0, (size_t)nbits);

    for (pos = 0; pos <= 16; pos++)
        bits[pos] = 1;
    bits[19] = frame->modulus_present ? 1 : 0;
    bits[20] = frame->coeffs_present ? 1 : 0;
    bits[21] = frame->constellations_present ? 1 : 0;
    v92_set_bits(bits, 22, 5, frame->selected_upstream_drn);
    v92_set_bits(bits, 27, 2, frame->trellis_select);
    bits[29] = frame->extend_e2u ? 1 : 0;
    bits[33] = frame->acknowledge ? 1 : 0;
    v92_set_bits(bits, 35, 16, frame->gain_q0_16);
    pos = 51;

    if (frame->modulus_present) {
        for (int w = 0; w < 6; w++) {
            pos++;   /* start bit */
            v92_set_bits(bits, pos, 8, frame->moduli[2 * w]);
            v92_set_bits(bits, pos + 8, 8, frame->moduli[2 * w + 1]);
            pos += 16;
        }
    }

    if (frame->coeffs_present) {
        const uint16_t lengths[4] = {
            frame->lz1, frame->lp1, frame->lz2, frame->lp2
        };
        const int16_t *taps[4] = {
            frame->precoder_ff, frame->precoder_fb,
            frame->prefilter_ff, frame->prefilter_fb
        };

        for (int w = 0; w < 4; w++) {
            pos++;   /* start bit */
            v92_set_bits(bits, pos, 9, lengths[w]);
            pos += 16;   /* 9 length bits + 7 reserved */
        }
        for (int group = 0; group < 4; group++) {
            for (int t = 0; t < lengths[group]; t++) {
                pos++;   /* start bit */
                v92_set_bits(bits, pos, 16, (uint16_t)taps[group][t]);
                pos += 16;
            }
        }
    }

    if (frame->constellations_present) {
        pos++;   /* start bit */
        for (int i = 0; i < 4; i++) {
            v92_set_bits(bits, pos, 4, frame->dfi[i]);
            pos += 4;
        }
        pos++;   /* start bit */
        v92_set_bits(bits, pos, 4, frame->dfi[4]);
        v92_set_bits(bits, pos + 4, 4, frame->dfi[5]);
        pos += 16;   /* two dfi fields + 8 reserved bits */
        for (int w = 0; w < 3; w++) {
            pos++;   /* start bit */
            v92_set_bits(bits, pos, 8, frame->set_sizes[2 * w]);
            v92_set_bits(bits, pos + 8, 8, frame->set_sizes[2 * w + 1]);
            pos += 16;
        }
        for (int i = 0; i < V92_CPD_MAX_SETS; i++) {
            for (int p = 0; p < frame->set_sizes[i]; p++) {
                pos++;   /* start bit */
                v92_set_bits(bits, pos, 16, frame->points[i][p]);
                pos += 16;
            }
        }
    }

    pos++;   /* start bit before the CRC */
    crc = v92_crc_through(bits, pos - 1);
    v92_set_bits(bits, pos, 16, crc);

    if (nbits_out)
        *nbits_out = nbits;
    return true;
}

bool v92_cpd_decode(const uint8_t *bits,
                    int bit_count,
                    v92_cpd_frame_t *frame,
                    v92_cpd_diag_t *diag)
{
    v92_cpd_diag_t local;
    v92_cpd_frame_t *f;
    int pos;
    int crc_start;

    if (!bits || !diag || bit_count < 69 || bit_count > V92_CPD_MAX_BITS)
        return false;
    memset(&local, 0, sizeof(local));
    local.nbits = bit_count;
    f = &local.frame;

    local.binary_bits_ok = v92_bits_binary(bits, bit_count);
    if (!local.binary_bits_ok) {
        *diag = local;
        return false;
    }
    local.frame_sync_ok = true;
    for (int i = 0; i <= 16; i++) {
        if (bits[i] != 1U)
            local.frame_sync_ok = false;
    }
    local.identifier_ok = bits[18] == 0U;
    local.start_bits_ok = bits[17] == 0U && bits[34] == 0U;
    local.reserved_ok = bits[30] == 0U && bits[31] == 0U && bits[32] == 0U;

    f->modulus_present = bits[19] != 0U;
    f->coeffs_present = bits[20] != 0U;
    f->constellations_present = bits[21] != 0U;
    f->selected_upstream_drn = (uint8_t)v92_get_bits(bits, 22, 5);
    f->trellis_select = (uint8_t)v92_get_bits(bits, 27, 2);
    f->extend_e2u = bits[29] != 0U;
    f->acknowledge = bits[33] != 0U;
    f->gain_q0_16 = (uint16_t)v92_get_bits(bits, 35, 16);
    pos = 51;

    if (f->modulus_present) {
        if (pos + 102 > bit_count) {
            *diag = local;
            return false;
        }
        for (int w = 0; w < 6; w++) {
            if (bits[pos++] != 0U)
                local.start_bits_ok = false;
            f->moduli[2 * w] = (uint8_t)v92_get_bits(bits, pos, 8);
            f->moduli[2 * w + 1] = (uint8_t)v92_get_bits(bits, pos + 8, 8);
            pos += 16;
        }
    }

    if (f->coeffs_present) {
        uint16_t *lengths[4] = { &f->lz1, &f->lp1, &f->lz2, &f->lp2 };
        int16_t *taps[4] = {
            f->precoder_ff, f->precoder_fb,
            f->prefilter_ff, f->prefilter_fb
        };

        if (pos + 68 > bit_count) {
            *diag = local;
            return false;
        }
        for (int w = 0; w < 4; w++) {
            if (bits[pos++] != 0U)
                local.start_bits_ok = false;
            *lengths[w] = (uint16_t)v92_get_bits(bits, pos, 9);
            for (int i = 9; i < 16; i++) {
                if (bits[pos + i] != 0U)
                    local.reserved_ok = false;
            }
            pos += 16;
            if (*lengths[w] > V92_CPD_MAX_TAPS) {
                *diag = local;
                return false;
            }
        }
        for (int group = 0; group < 4; group++) {
            if (pos + 17 * *lengths[group] > bit_count) {
                *diag = local;
                return false;
            }
            for (int t = 0; t < *lengths[group]; t++) {
                if (bits[pos++] != 0U)
                    local.start_bits_ok = false;
                taps[group][t] = (int16_t)v92_get_bits(bits, pos, 16);
                pos += 16;
            }
        }
    }

    if (f->constellations_present) {
        if (pos + 85 > bit_count) {
            *diag = local;
            return false;
        }
        if (bits[pos++] != 0U)
            local.start_bits_ok = false;
        for (int i = 0; i < 4; i++) {
            f->dfi[i] = (uint8_t)v92_get_bits(bits, pos, 4);
            pos += 4;
        }
        if (bits[pos++] != 0U)
            local.start_bits_ok = false;
        f->dfi[4] = (uint8_t)v92_get_bits(bits, pos, 4);
        f->dfi[5] = (uint8_t)v92_get_bits(bits, pos + 4, 4);
        for (int i = 8; i < 16; i++) {
            if (bits[pos + i] != 0U)
                local.reserved_ok = false;
        }
        pos += 16;
        for (int w = 0; w < 3; w++) {
            if (bits[pos++] != 0U)
                local.start_bits_ok = false;
            f->set_sizes[2 * w] = (uint8_t)v92_get_bits(bits, pos, 8);
            f->set_sizes[2 * w + 1] = (uint8_t)v92_get_bits(bits, pos + 8, 8);
            pos += 16;
        }
        for (int i = 0; i < V92_CPD_MAX_SETS; i++) {
            if (f->set_sizes[i] > V92_CPD_MAX_POINTS
                || pos + 17 * f->set_sizes[i] > bit_count) {
                *diag = local;
                return false;
            }
            for (int p = 0; p < f->set_sizes[i]; p++) {
                if (bits[pos++] != 0U)
                    local.start_bits_ok = false;
                f->points[i][p] = (uint16_t)v92_get_bits(bits, pos, 16);
                pos += 16;
            }
        }
    }

    if (pos + 18 > bit_count) {
        *diag = local;
        return false;
    }
    if (bits[pos] != 0U)
        local.start_bits_ok = false;
    crc_start = pos + 1;
    local.crc_field = (uint16_t)v92_get_bits(bits, crc_start, 16);
    local.crc_expected = v92_crc_through(bits, crc_start - 1);
    local.crc_ok = local.crc_field == local.crc_expected;
    local.fill_bits_ok = true;
    for (int i = crc_start + 16; i < bit_count; i++) {
        if (bits[i] != 0U)
            local.fill_bits_ok = false;
    }

    local.parameters_ok = v92_cpd_frame_valid(f);
    local.valid = local.binary_bits_ok
               && local.frame_sync_ok
               && local.identifier_ok
               && local.start_bits_ok
               && local.reserved_ok
               && local.parameters_ok
               && local.fill_bits_ok
               && local.crc_ok;
    if (frame)
        *frame = local.frame;
    *diag = local;
    return local.valid;
}

bool v92_phase4_analyze(const v92_phase4_observation_t *obs,
                        v92_phase4_result_t *out)
{
    if (!obs || !out)
        return false;

    memset(out, 0, sizeof(*out));
    out->phase4_sample = obs->phase4_sample;
    out->started = obs->phase4_seen && obs->phase4_sample >= 0;
    out->suvd_seen = obs->suvd_seen;
    out->suvd_valid = obs->suvd_valid;
    out->suvd_acknowledge_seen = obs->suvd_acknowledge_seen;
    out->cpd_seen = obs->cpd_seen;
    out->cpd_valid = obs->cpd_valid;
    out->ed_seen = obs->ed_seen;
    out->b1d_seen = obs->b1d_seen;
    out->data_seen = obs->data_seen;

    if (!out->started) {
        out->status = "waiting_phase4";
    } else if (obs->training_failed) {
        out->status = "failed";
    } else if (!obs->suvd_seen) {
        out->status = "waiting_suvd";
    } else if (!obs->suvd_valid) {
        out->status = "invalid_suvd";
    } else if (!obs->cpd_seen) {
        out->status = "waiting_cpd";
    } else if (!obs->cpd_valid) {
        out->status = "invalid_cpd";
    } else if (!obs->suvd_acknowledge_seen) {
        out->status = "waiting_suvd_ack";
    } else if (!obs->ed_seen) {
        out->status = "waiting_ed";
    } else if (!obs->b1d_seen) {
        out->status = "waiting_b1d";
    } else if (!obs->data_seen) {
        out->status = "waiting_data";
    } else {
        out->status = "complete";
    }

    out->complete = out->started
                 && !obs->training_failed
                 && obs->suvd_seen
                 && obs->suvd_valid
                 && obs->cpd_seen
                 && obs->cpd_valid
                 && obs->suvd_acknowledge_seen
                 && obs->ed_seen
                 && obs->b1d_seen
                 && obs->data_seen;
    out->valid = true;
    return true;
}
