#include "v34_info_decode.h"

#include <string.h>

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

bool v34_info_collector_push_bit(v34_info_collector_t *collector,
                                 int bit,
                                 uint8_t *frame_out,
                                 int frame_out_len)
{
    int bytes_to_copy;

    if (!collector)
        return false;

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

    if (!buf)
        return false;

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

    if (!buf)
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

    if (!buf || !raw_out)
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
