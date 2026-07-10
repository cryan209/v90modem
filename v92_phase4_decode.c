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

bool v92_suvd_encode(const v92_suvd_frame_t *frame,
                     uint8_t *bits,
                     int bits_max)
{
    uint16_t crc;

    if (!frame || !bits || bits_max < V92_SUVD_BITS)
        return false;
    memset(bits, 0, V92_SUVD_BITS);
    for (int i = 0; i <= 16; i++)
        bits[i] = 1;
    bits[18] = 1;
    bits[32] = frame->silent_period_requested ? 1 : 0;
    bits[33] = frame->acknowledge ? 1 : 0;
    crc = v92_crc_through(bits, 34);
    for (int i = 0; i < 16; i++)
        bits[35 + i] = (uint8_t)((crc >> i) & 1U);
    return true;
}

bool v92_suvd_decode(const uint8_t *bits,
                     int bit_count,
                     v92_suvd_frame_t *frame,
                     v92_suvd_diag_t *diag)
{
    v92_suvd_diag_t local;
    uint16_t crc_field = 0;

    if (!bits || bit_count != V92_SUVD_BITS)
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
    for (int i = 51; i < V92_SUVD_BITS; i++) {
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
