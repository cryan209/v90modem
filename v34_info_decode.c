#include "v34_info_decode.h"

#include <string.h>

static void v34_info_unpack_bits(uint8_t bits[], int nbits, const uint8_t buf[])
{
    for (int i = 0; i < nbits; i++) {
        uint8_t octet;

        octet = bit_reverse8(buf[i >> 3]);
        bits[i] = (octet >> (7 - (i & 7))) & 1;
    }
}

static void v34_info_pack_bits(uint8_t buf[V34_INFO_MAX_BUF_BYTES], const uint8_t bits[], int nbits)
{
    memset(buf, 0, V34_INFO_MAX_BUF_BYTES);
    for (int i = 0; i < nbits; i++) {
        if (bits[i])
            buf[i >> 3] |= (1 << (i & 7));
    }
}

static uint16_t v34_info_crc_from_bits(const uint8_t bits[], int nbits)
{
    uint16_t crc;

    crc = 0xFFFF;
    for (int i = 0; i < nbits; i++)
        crc = crc_itu16_bits(bits[i], 1, crc);
    return crc;
}

static void v34_info_build_full_frame_bytes(uint8_t *out,
                                            int out_len,
                                            const uint8_t *payload,
                                            int target_bits)
{
    uint8_t bits[256];
    int total_bits;

    if (!out || out_len <= 0)
        return;

    memset(out, 0, (size_t) out_len);
    if (!payload || target_bits <= 0)
        return;

    total_bits = V34_INFO_SYNC_BITS + target_bits;
    if (total_bits > (int) (sizeof(bits)/sizeof(bits[0])))
        return;

    for (int i = 0; i < V34_INFO_SYNC_BITS; i++)
        bits[i] = (uint8_t) ((V34_INFO_SYNC_CODE >> (V34_INFO_SYNC_BITS - 1 - i)) & 1);
    v34_info_unpack_bits(bits + V34_INFO_SYNC_BITS, target_bits, payload);

    memset(out, 0, (size_t) out_len);
    for (int i = 0; i < total_bits; i++) {
        if (bits[i])
            out[i >> 3] |= (uint8_t) (1U << (i & 7));
    }
}

void v34_info_collector_init(v34_info_collector_t *collector, int target_bits)
{
    if (!collector)
        return;
    memset(collector, 0, sizeof(*collector));
    collector->target_bits = target_bits;
}

void v34_info_collector_reset(v34_info_collector_t *collector, int target_bits)
{
    if (!collector)
        return;
    collector->bitstream = 0;
    collector->crc = 0;
    collector->bit_count = 0;
    collector->target_bits = target_bits;
    memset(collector->info_buf, 0, sizeof(collector->info_buf));
}

void v34_info_collector_load_snapshot(v34_info_collector_t *collector,
                                      uint16_t bitstream,
                                      uint16_t crc,
                                      int bit_count,
                                      int target_bits,
                                      const uint8_t *info_buf,
                                      int info_buf_len)
{
    if (!collector)
        return;

    collector->bitstream = bitstream;
    collector->crc = crc;
    collector->bit_count = bit_count;
    collector->target_bits = target_bits;
    memset(collector->info_buf, 0, sizeof(collector->info_buf));
    if (info_buf && info_buf_len > 0) {
        if (info_buf_len > (int) sizeof(collector->info_buf))
            info_buf_len = (int) sizeof(collector->info_buf);
        memcpy(collector->info_buf, info_buf, (size_t) info_buf_len);
    }
}

bool v34_info_frame_from_collector(v34_info_frame_t *frame,
                                   const v34_info_collector_t *collector)
{
    int payload_bytes;
    int total_bytes;

    if (!frame)
        return false;
    memset(frame, 0, sizeof(*frame));
    if (!collector || collector->target_bits <= 0)
        return false;

    payload_bytes = (collector->target_bits + 7) / 8;
    if (payload_bytes > V34_INFO_MAX_BUF_BYTES)
        payload_bytes = V34_INFO_MAX_BUF_BYTES;
    total_bytes = (V34_INFO_SYNC_BITS + collector->target_bits + 7) / 8;
    if (total_bytes > V34_INFO_MAX_BUF_BYTES)
        total_bytes = V34_INFO_MAX_BUF_BYTES;
    frame->target_bits = collector->target_bits;
    frame->payload_bytes = payload_bytes;
    frame->total_bits = V34_INFO_SYNC_BITS + collector->target_bits;
    frame->total_bytes = total_bytes;
    memcpy(frame->payload, collector->info_buf, (size_t) payload_bytes);
    v34_info_build_full_frame_bytes(frame->bytes,
                                    frame->total_bytes,
                                    frame->payload,
                                    frame->target_bits);
    frame->valid = v34_info_validate_frame_bytes(frame->payload, frame->target_bits);
    return frame->valid;
}

bool v34_info_collector_push_bit(v34_info_collector_t *collector,
                                 int bit,
                                 uint8_t *frame_out,
                                 int frame_out_len)
{
    return v34_info_collector_push_bit_verbose(collector,
                                               bit,
                                               frame_out,
                                               frame_out_len,
                                               NULL,
                                               NULL);
}

bool v34_info_collector_push_bit_verbose(v34_info_collector_t *collector,
                                         int bit,
                                         uint8_t *frame_out,
                                         int frame_out_len,
                                         bool *frame_complete_out,
                                         uint16_t *crc_out)
{
    int bytes_to_copy;

    if (!collector)
        return false;
    if (frame_complete_out)
        *frame_complete_out = false;
    if (crc_out)
        *crc_out = 0;

    collector->bitstream = (uint16_t) ((collector->bitstream << 1) | (bit & 1));
    if (collector->bit_count == 0) {
        if ((collector->bitstream & 0x3FFU) == V34_INFO_SYNC_CODE) {
            collector->crc = 0xFFFF;
            collector->bit_count = 1;
        }
        return false;
    }

    if ((collector->bit_count & 0x07) == 0) {
        int idx = (collector->bit_count >> 3) - 1;

        if (idx >= 0 && idx < (int) sizeof(collector->info_buf))
            collector->info_buf[idx] = bit_reverse8((uint8_t) (collector->bitstream & 0xFF));
    }
    collector->crc = crc_itu16_bits((uint8_t) (bit & 1), 1, collector->crc);
    if (collector->bit_count++ != collector->target_bits)
        return false;
    if (frame_complete_out)
        *frame_complete_out = true;
    if (crc_out)
        *crc_out = collector->crc;

    bytes_to_copy = (collector->target_bits + 7) / 8;
    if (bytes_to_copy > frame_out_len)
        bytes_to_copy = frame_out_len;
    if (bytes_to_copy > (int) sizeof(collector->info_buf))
        bytes_to_copy = (int) sizeof(collector->info_buf);
    if (collector->crc == 0 && frame_out && bytes_to_copy > 0)
        memcpy(frame_out, collector->info_buf, (size_t) bytes_to_copy);

    collector->bit_count = 0;
    return collector->crc == 0;
}

bool v34_info_try_boundary_recovery(uint8_t *out,
                                    const uint8_t *in,
                                    int nbits,
                                    int *shift_out)
{
    uint8_t bits[128];
    uint8_t trial[128];

    if (!out || !in || nbits <= 0 || nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return false;

    v34_info_unpack_bits(bits, nbits, in);
    for (int shift = -2; shift <= 2; shift++) {
        uint16_t crc;

        if (shift == 0)
            continue;
        for (int i = 0; i < nbits; i++) {
            int src = i + shift;

            trial[i] = (src >= 0 && src < nbits) ? bits[src] : 0;
        }
        crc = v34_info_crc_from_bits(trial, nbits);
        if (crc == 0) {
            v34_info_pack_bits(out, trial, nbits);
            if (shift_out)
                *shift_out = shift;
            return true;
        }
    }
    return false;
}

bool v34_info_try_local_slip_recovery(uint8_t *out,
                                      const uint8_t *in,
                                      int nbits,
                                      int *pivot_out,
                                      int *shift_out)
{
    uint8_t bits[128];
    uint8_t trial[128];

    if (!out || !in || nbits <= 0 || nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return false;

    v34_info_unpack_bits(bits, nbits, in);
    for (int pivot = 4; pivot < nbits - 4; pivot++) {
        for (int shift = -1; shift <= 1; shift += 2) {
            uint16_t crc;

            for (int i = 0; i < nbits; i++) {
                int src;

                if (i < pivot)
                    src = i;
                else
                    src = i + shift;
                trial[i] = (src >= 0 && src < nbits) ? bits[src] : 0;
            }
            crc = v34_info_crc_from_bits(trial, nbits);
            if (crc == 0) {
                v34_info_pack_bits(out, trial, nbits);
                if (pivot_out)
                    *pivot_out = pivot;
                if (shift_out)
                    *shift_out = shift;
                return true;
            }
        }
    }
    return false;
}

bool v34_info_try_bit_error_recovery(uint8_t *out,
                                     const uint8_t *in,
                                     int nbits,
                                     int *bit_a_out,
                                     int *bit_b_out)
{
    uint8_t bits[128];
    uint8_t trial[128];

    if (!out || !in || nbits <= 0 || nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return false;

    v34_info_unpack_bits(bits, nbits, in);

    memcpy(trial, bits, (size_t) nbits);
    for (int a = 0; a < nbits; a++) {
        uint16_t crc;

        trial[a] ^= 1;
        crc = v34_info_crc_from_bits(trial, nbits);
        if (crc == 0) {
            v34_info_pack_bits(out, trial, nbits);
            if (bit_a_out)
                *bit_a_out = a;
            if (bit_b_out)
                *bit_b_out = -1;
            return true;
        }
        trial[a] ^= 1;
    }

    memcpy(trial, bits, (size_t) nbits);
    for (int a = 0; a < nbits; a++) {
        trial[a] ^= 1;
        for (int b = a + 1; b < nbits; b++) {
            uint16_t crc;

            trial[b] ^= 1;
            crc = v34_info_crc_from_bits(trial, nbits);
            if (crc == 0) {
                v34_info_pack_bits(out, trial, nbits);
                if (bit_a_out)
                    *bit_a_out = a;
                if (bit_b_out)
                    *bit_b_out = b;
                return true;
            }
            trial[b] ^= 1;
        }
        trial[a] ^= 1;
    }

    return false;
}

bool v34_info_validate_frame_bytes(const uint8_t *buf, int target_bits)
{
    v34_info_collector_t collector;
    uint8_t replay[V34_INFO_MAX_BUF_BYTES];
    int payload_bits_remaining;
    int total_bytes;

    if (!buf || target_bits <= 0)
        return false;

    v34_info_collector_init(&collector, target_bits);
    for (int bit = 9; bit >= 0; bit--) {
        (void) v34_info_collector_push_bit(&collector,
                                           (V34_INFO_SYNC_CODE >> bit) & 1,
                                           replay,
                                           (int) sizeof(replay));
    }

    payload_bits_remaining = target_bits;
    total_bytes = (target_bits + 7) / 8;
    if (total_bytes > V34_INFO_MAX_BUF_BYTES)
        total_bytes = V34_INFO_MAX_BUF_BYTES;
    for (int i = 0; i < total_bytes; i++) {
        int bits_this_byte = payload_bits_remaining;

        if (bits_this_byte > 8)
            bits_this_byte = 8;
        for (int bit = 0; bit < bits_this_byte; bit++) {
            if (v34_info_collector_push_bit(&collector,
                                            (buf[i] >> bit) & 1,
                                            replay,
                                            (int) sizeof(replay))) {
                int compare_bytes = (target_bits + 7) / 8;

                if (compare_bytes > V34_INFO_MAX_BUF_BYTES)
                    compare_bytes = V34_INFO_MAX_BUF_BYTES;
                return memcmp(replay, buf, (size_t) compare_bytes) == 0;
            }
        }
        payload_bits_remaining -= bits_this_byte;
    }
    return false;
}

bool v34_map_received_info0a(v90_info0a_t *dst, const v34_v90_info0a_t *src)
{
    if (!dst || !src)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->support_2743 = src->support_2743;
    dst->support_2800 = src->support_2800;
    dst->support_3429 = src->support_3429;
    dst->support_3000_low = src->support_3000_low;
    dst->support_3000_high = src->support_3000_high;
    dst->support_3200_low = src->support_3200_low;
    dst->support_3200_high = src->support_3200_high;
    dst->rate_3429_allowed = src->rate_3429_allowed;
    dst->support_power_reduction = src->support_power_reduction;
    dst->max_baud_rate_difference = src->max_baud_rate_difference;
    dst->from_cme_modem = src->from_cme_modem;
    dst->support_1664_point_constellation = src->support_1664_point_constellation;
    dst->tx_clock_source = src->tx_clock_source;
    dst->acknowledge_info0d = src->acknowledge_info0d;
    return v90_info0a_validate(dst);
}

bool v34_map_received_info1a(v90_info1a_t *dst, const v34_v90_info1a_t *src)
{
    if (!dst || !src)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->md = (uint8_t) src->md;
    dst->u_info = (uint8_t) src->u_info;
    dst->upstream_symbol_rate_code = (uint8_t) src->upstream_symbol_rate_code;
    dst->downstream_rate_code = (uint8_t) src->downstream_rate_code;
    dst->freq_offset = (int16_t) src->freq_offset;
    return v90_info1a_validate(dst);
}

bool v34_info_parse_info0a_v90(const uint8_t *buf,
                               v34_v90_info0a_t *raw_out,
                               v90_info0a_t *mapped_out)
{
    bitstream_state_t bs;
    const uint8_t *t;
    v34_v90_info0a_t parsed;
    int target_bits;

    if (!buf)
        return false;

    if (v34_info_validate_frame_bytes(buf, 49 - (4 + 8 + 4))) {
        target_bits = 49 - (4 + 8 + 4);
    } else if (v34_info_validate_frame_bytes(buf, 62 - (4 + 8 + 4))) {
        target_bits = 62 - (4 + 8 + 4);
    } else {
        return false;
    }

    memset(&parsed, 0, sizeof(parsed));
    bitstream_init(&bs, true);
    t = buf;
    parsed.support_2743 = bitstream_get(&bs, &t, 1);
    parsed.support_2800 = bitstream_get(&bs, &t, 1);
    parsed.support_3429 = bitstream_get(&bs, &t, 1);
    parsed.support_3000_low = bitstream_get(&bs, &t, 1);
    parsed.support_3000_high = bitstream_get(&bs, &t, 1);
    parsed.support_3200_low = bitstream_get(&bs, &t, 1);
    parsed.support_3200_high = bitstream_get(&bs, &t, 1);
    parsed.rate_3429_allowed = bitstream_get(&bs, &t, 1);
    parsed.support_power_reduction = bitstream_get(&bs, &t, 1);
    parsed.max_baud_rate_difference = (uint8_t) bitstream_get(&bs, &t, 3);
    parsed.from_cme_modem = bitstream_get(&bs, &t, 1);
    parsed.support_1664_point_constellation = bitstream_get(&bs, &t, 1);
    if (target_bits == (49 - (4 + 8 + 4))) {
        parsed.raw_26_27 = (uint8_t) bitstream_get(&bs, &t, 2);
        parsed.tx_clock_source = parsed.raw_26_27;
        parsed.acknowledge_info0d = bitstream_get(&bs, &t, 1);
        parsed.info0d_extensions_valid = false;
    } else {
        parsed.raw_26_27 = (uint8_t) bitstream_get(&bs, &t, 2);
        parsed.acknowledge_info0d = bitstream_get(&bs, &t, 1);
        parsed.info0d_nominal_power_code = (uint8_t) bitstream_get(&bs, &t, 4);
        parsed.info0d_max_power_code = (uint8_t) bitstream_get(&bs, &t, 5);
        parsed.info0d_power_measured_at_codec_output = bitstream_get(&bs, &t, 1);
        parsed.tx_clock_source = (uint8_t) bitstream_get(&bs, &t, 1);
        parsed.info0d_pcm_alaw = (parsed.tx_clock_source != 0);
        parsed.info0d_upstream_3429_support = bitstream_get(&bs, &t, 1);
        parsed.info0d_reserved_41 = (uint8_t) bitstream_get(&bs, &t, 1);
        parsed.info0d_extensions_valid = true;
    }

    if (mapped_out && !v34_map_received_info0a(mapped_out, &parsed))
        return false;
    if (raw_out)
        *raw_out = parsed;
    return true;
}

bool v34_info_parse_info1a_v90(const uint8_t *buf,
                               v34_v90_info1a_t *raw_out,
                               v90_info1a_t *mapped_out)
{
    bitstream_state_t bs;
    const uint8_t *t;
    uint16_t raw_freq;
    v34_v90_info1a_t parsed;

    if (!buf || !v34_info_validate_frame_bytes(buf, 70 - (4 + 8 + 4)))
        return false;

    memset(&parsed, 0, sizeof(parsed));
    bitstream_init(&bs, true);
    t = buf;
    parsed.raw_12_17 = (uint8_t) bitstream_get(&bs, &t, 6);
    parsed.md = bitstream_get(&bs, &t, 7);
    parsed.u_info = bitstream_get(&bs, &t, 7);
    parsed.raw_32_33 = (uint8_t) bitstream_get(&bs, &t, 2);
    parsed.upstream_symbol_rate_code = bitstream_get(&bs, &t, 3);
    parsed.downstream_rate_code = bitstream_get(&bs, &t, 3);
    raw_freq = (uint16_t) bitstream_get(&bs, &t, 10);
    parsed.raw_40_49 = raw_freq;
    parsed.freq_offset = (int) raw_freq;
    if ((parsed.freq_offset & 0x200) != 0)
        parsed.freq_offset = -(parsed.freq_offset ^ 0x3FF) - 1;

    if (mapped_out && !v34_map_received_info1a(mapped_out, &parsed))
        return false;
    if (raw_out)
        *raw_out = parsed;
    return true;
}

bool v34_info_parse_info1d_v90(const uint8_t *buf,
                               v34_v90_info1d_t *raw_out)
{
    bitstream_state_t bs;
    const uint8_t *t;
    v34_v90_info1d_t parsed;

    if (!buf || !raw_out || !v34_info_validate_frame_bytes(buf, 109 - (4 + 8 + 4)))
        return false;

    memset(&parsed, 0, sizeof(parsed));
    bitstream_init(&bs, true);
    t = buf;
    parsed.power_reduction = bitstream_get(&bs, &t, 3);
    parsed.additional_power_reduction = bitstream_get(&bs, &t, 3);
    parsed.md = bitstream_get(&bs, &t, 7);
    for (int i = 0; i < 6; i++) {
        parsed.rate_data[i].use_high_carrier = bitstream_get(&bs, &t, 1);
        parsed.rate_data[i].pre_emphasis = bitstream_get(&bs, &t, 4);
        parsed.rate_data[i].max_bit_rate = bitstream_get(&bs, &t, 4);
    }
    parsed.freq_offset = bitstream_get(&bs, &t, 10);
    if ((parsed.freq_offset & 0x200) != 0)
        parsed.freq_offset = -(parsed.freq_offset ^ 0x3FF) - 1;
    *raw_out = parsed;
    return true;
}

bool v34_info_parse_info1a_v34(const uint8_t *buf,
                               v34_info1a_generic_t *out)
{
    bitstream_state_t bs;
    const uint8_t *t;
    uint16_t raw_freq;
    v34_info1a_generic_t parsed;

    if (!buf || !out || !v34_info_validate_frame_bytes(buf, 70 - (4 + 8 + 4)))
        return false;

    memset(&parsed, 0, sizeof(parsed));
    bitstream_init(&bs, true);
    t = buf;
    parsed.power_reduction = bitstream_get(&bs, &t, 3);
    parsed.additional_power_reduction = bitstream_get(&bs, &t, 3);
    parsed.md = bitstream_get(&bs, &t, 7);
    parsed.use_high_carrier = bitstream_get(&bs, &t, 1);
    parsed.preemphasis_filter = bitstream_get(&bs, &t, 4);
    parsed.max_data_rate = bitstream_get(&bs, &t, 4);
    parsed.baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
    parsed.baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
    raw_freq = (uint16_t) bitstream_get(&bs, &t, 10);
    parsed.freq_offset = (int) raw_freq;
    if ((parsed.freq_offset & 0x200) != 0)
        parsed.freq_offset = -(parsed.freq_offset ^ 0x3FF) - 1;

    *out = parsed;
    return true;
}

bool v34_info_parse_info1c_v34(const uint8_t *buf,
                               v34_info1c_generic_t *out)
{
    bitstream_state_t bs;
    const uint8_t *t;
    v34_info1c_generic_t parsed;

    if (!buf || !out || !v34_info_validate_frame_bytes(buf, 109 - (4 + 8 + 4)))
        return false;

    memset(&parsed, 0, sizeof(parsed));
    bitstream_init(&bs, true);
    t = buf;
    parsed.power_reduction = bitstream_get(&bs, &t, 3);
    parsed.additional_power_reduction = bitstream_get(&bs, &t, 3);
    parsed.md = bitstream_get(&bs, &t, 7);
    for (int i = 0; i < 6; i++) {
        parsed.rate_data[i].use_high_carrier = bitstream_get(&bs, &t, 1);
        parsed.rate_data[i].pre_emphasis = bitstream_get(&bs, &t, 4);
        parsed.rate_data[i].max_bit_rate = bitstream_get(&bs, &t, 4);
    }
    parsed.freq_offset = bitstream_get(&bs, &t, 10);
    if ((parsed.freq_offset & 0x200) != 0)
        parsed.freq_offset = -(parsed.freq_offset ^ 0x3FF) - 1;

    *out = parsed;
    return true;
}

bool v34_info_parse_info0a_v90_frame(const v34_info_frame_t *frame,
                                     v34_v90_info0a_t *raw_out,
                                     v90_info0a_t *mapped_out)
{
    if (!frame || !frame->valid)
        return false;
    if (frame->target_bits != (49 - (4 + 8 + 4))
        && frame->target_bits != (62 - (4 + 8 + 4))) {
        return false;
    }
    return v34_info_parse_info0a_v90(frame->payload, raw_out, mapped_out);
}

bool v34_info_parse_info0a_v34_frame(const v34_info_frame_t *frame,
                                     v34_v90_info0a_t *raw_out,
                                     v90_info0a_t *mapped_out)
{
    if (!frame || !frame->valid || frame->target_bits != (49 - (4 + 8 + 4)))
        return false;
    return v34_info_parse_info0a_v90(frame->payload, raw_out, mapped_out);
}

bool v34_info_parse_info0c_v34_frame(const v34_info_frame_t *frame,
                                     v34_v90_info0a_t *raw_out,
                                     v90_info0a_t *mapped_out)
{
    if (!frame || !frame->valid || frame->target_bits != (49 - (4 + 8 + 4)))
        return false;
    return v34_info_parse_info0a_v90(frame->payload, raw_out, mapped_out);
}

bool v34_info_parse_info1a_v90_frame(const v34_info_frame_t *frame,
                                     v34_v90_info1a_t *raw_out,
                                     v90_info1a_t *mapped_out)
{
    if (!frame || !frame->valid || frame->target_bits != (70 - (4 + 8 + 4)))
        return false;
    return v34_info_parse_info1a_v90(frame->payload, raw_out, mapped_out);
}

bool v34_info_parse_info1d_v90_frame(const v34_info_frame_t *frame,
                                     v34_v90_info1d_t *raw_out)
{
    if (!frame || !frame->valid || frame->target_bits != (109 - (4 + 8 + 4)))
        return false;
    return v34_info_parse_info1d_v90(frame->payload, raw_out);
}

bool v34_info_parse_info1a_v34_frame(const v34_info_frame_t *frame,
                                     v34_info1a_generic_t *out)
{
    if (!frame || !frame->valid || frame->target_bits != (70 - (4 + 8 + 4)))
        return false;
    return v34_info_parse_info1a_v34(frame->payload, out);
}

bool v34_info_parse_info1c_v34_frame(const v34_info_frame_t *frame,
                                     v34_info1c_generic_t *out)
{
    if (!frame || !frame->valid || frame->target_bits != (109 - (4 + 8 + 4)))
        return false;
    return v34_info_parse_info1c_v34(frame->payload, out);
}
