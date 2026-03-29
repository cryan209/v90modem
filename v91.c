/*
 * v91.c - V.91 PCM modem helpers
 */

#include "v91.h"

#include <spandsp.h>

#include <spandsp/crc.h>

#include <stdio.h>
#include <string.h>

/* Positive A-law codewords indexed by Ucode. Shared with the V.90 mapping. */
static const uint8_t v91_ucode_to_alaw[128] = {
    0xD5, 0xD4, 0xD7, 0xD6, 0xD1, 0xD0, 0xD3, 0xD2,
    0xDD, 0xDC, 0xDF, 0xDE, 0xD9, 0xD8, 0xDB, 0xDA,
    0xC5, 0xC4, 0xC7, 0xC6, 0xC1, 0xC0, 0xC3, 0xC2,
    0xCD, 0xCC, 0xCF, 0xCE, 0xC9, 0xC8, 0xCB, 0xCA,
    0xF5, 0xF4, 0xF7, 0xF6, 0xF1, 0xF0, 0xF3, 0xF2,
    0xFD, 0xFC, 0xFF, 0xFE, 0xF9, 0xF8, 0xFB, 0xFA,
    0xE5, 0xE4, 0xE7, 0xE6, 0xE1, 0xE0, 0xE3, 0xE2,
    0xED, 0xEC, 0xEF, 0xEE, 0xE9, 0xE8, 0xEB, 0xEA,
    0x95, 0x94, 0x97, 0x96, 0x91, 0x90, 0x93, 0x92,
    0x9D, 0x9C, 0x9F, 0x9E, 0x99, 0x98, 0x9B, 0x9A,
    0x85, 0x84, 0x87, 0x86, 0x81, 0x80, 0x83, 0x82,
    0x8D, 0x8C, 0x8F, 0x8E, 0x89, 0x88, 0x8B, 0x8A,
    0xB5, 0xB4, 0xB7, 0xB6, 0xB1, 0xB0, 0xB3, 0xB2,
    0xBD, 0xBC, 0xBF, 0xBE, 0xB9, 0xB8, 0xBB, 0xBA,
    0xA5, 0xA4, 0xA7, 0xA6, 0xA1, 0xA0, 0xA3, 0xA2,
    0xAD, 0xAC, 0xAF, 0xAE, 0xA9, 0xA8, 0xAB, 0xAA,
};

#define V91_INFO_FILL_AND_SYNC_BITS 0x72F
#define V91_INFO_PAYLOAD_START_BIT 12
#define V91_INFO_CRC_START_BIT 42
#define V91_INFO_TOTAL_BITS 62

static uint16_t v91_crc_bits(const uint8_t *bits, int nbits)
{
    uint16_t crc;
    int i;

    crc = 0xFFFF;
    for (i = 0; i < nbits; i++)
        crc = crc_itu16_bits(bits[i], 1, crc);
    return crc;
}

static uint16_t v91_bits_to_u16(const uint8_t *bits, int start, int nbits)
{
    uint16_t value;
    int i;

    value = 0;
    for (i = 0; i < nbits; i++)
        value |= (uint16_t) (bits[start + i] & 1U) << i;
    return value;
}

bool v91_info_frame_validate(const v91_info_frame_t *info, char *reason, size_t reason_len)
{
    const char *msg;

    msg = NULL;
    if ((info->reserved_12_25 & ~0x3FFFU) != 0)
        msg = "reserved_12_25 exceeds 14 bits";
    else if ((info->reserved_29_32 & ~0x0FU) != 0)
        msg = "reserved_29_32 exceeds 4 bits";
    else if ((info->max_tx_power & ~0x1FU) != 0)
        msg = "max_tx_power exceeds 5 bits";
    else if (info->cleardown_if_transparent_denied && !info->request_transparent_mode)
        msg = "cleardown_if_transparent_denied requires request_transparent_mode";

    if (reason && reason_len > 0) {
        if (msg)
            snprintf(reason, reason_len, "%s", msg);
        else
            reason[0] = '\0';
    }
    return (msg == NULL);
}

static void v91_info_to_bits(uint8_t bits[V91_INFO_TOTAL_BITS], const v91_info_frame_t *info)
{
    uint16_t crc;
    int i;

    memset(bits, 0, V91_INFO_TOTAL_BITS);
    bits[0] = 1;
    bits[1] = 1;
    bits[2] = 1;
    bits[3] = 1;

    for (i = 0; i < 8; i++)
        bits[4 + i] = (uint8_t) ((0x72U >> i) & 1U);

    for (i = 0; i < 14; i++)
        bits[12 + i] = (uint8_t) ((info->reserved_12_25 >> i) & 1U);
    bits[26] = info->request_default_dil ? 0 : 1;
    bits[27] = info->request_control_channel ? 1 : 0;
    bits[28] = info->acknowledge_info_frame ? 1 : 0;
    for (i = 0; i < 4; i++)
        bits[29 + i] = (uint8_t) ((info->reserved_29_32 >> i) & 1U);

    for (i = 0; i < 5; i++)
        bits[33 + i] = (uint8_t) ((info->max_tx_power >> i) & 1U);

    bits[38] = info->power_measured_after_digital_impairments ? 1 : 0;
    bits[39] = info->tx_uses_alaw ? 1 : 0;
    bits[40] = info->request_transparent_mode ? 1 : 0;
    bits[41] = info->cleardown_if_transparent_denied ? 1 : 0;

    crc = v91_crc_bits(bits + V91_INFO_PAYLOAD_START_BIT,
                       V91_INFO_CRC_START_BIT - V91_INFO_PAYLOAD_START_BIT);
    for (i = 0; i < 16; i++)
        bits[V91_INFO_CRC_START_BIT + i] = (uint8_t) ((crc >> i) & 1U);

    bits[58] = 1;
    bits[59] = 1;
    bits[60] = 1;
    bits[61] = 1;
}

static bool v91_info_from_bits(const uint8_t bits[V91_INFO_TOTAL_BITS], v91_info_frame_t *info_out)
{
    uint16_t crc;
    int i;

    if (!(bits[0] && bits[1] && bits[2] && bits[3]))
        return false;
    for (i = 0; i < 8; i++) {
        if (bits[4 + i] != ((0x72U >> i) & 1U))
            return false;
    }
    if (!(bits[58] && bits[59] && bits[60] && bits[61]))
        return false;

    crc = v91_crc_bits(bits + V91_INFO_PAYLOAD_START_BIT,
                       (V91_INFO_CRC_START_BIT + 16) - V91_INFO_PAYLOAD_START_BIT);
    if (crc != 0)
        return false;

    memset(info_out, 0, sizeof(*info_out));
    info_out->reserved_12_25 = v91_bits_to_u16(bits, 12, 14);
    info_out->request_default_dil = (bits[26] == 0);
    info_out->request_control_channel = (bits[27] != 0);
    info_out->acknowledge_info_frame = (bits[28] != 0);
    info_out->reserved_29_32 = (uint8_t) v91_bits_to_u16(bits, 29, 4);
    for (i = 0; i < 5; i++)
        info_out->max_tx_power |= (uint8_t) (bits[33 + i] << i);
    info_out->power_measured_after_digital_impairments = (bits[38] != 0);
    info_out->tx_uses_alaw = (bits[39] != 0);
    info_out->request_transparent_mode = (bits[40] != 0);
    info_out->cleardown_if_transparent_denied = (bits[41] != 0);
    return true;
}

void v91_init(v91_state_t *s, v91_law_t law, v91_mode_t mode)
{
    s->law = law;
    s->mode = mode;
}

uint8_t v91_idle_codeword(v91_law_t law)
{
    return (law == V91_LAW_ALAW) ? (uint8_t) 0xD5 : (uint8_t) 0xFF;
}

uint8_t v91_ucode_to_codeword(v91_law_t law, int ucode, bool positive)
{
    uint8_t codeword;

    if (law == V91_LAW_ALAW)
        codeword = v91_ucode_to_alaw[ucode & 0x7F];
    else
        codeword = (uint8_t) (0xFF - (ucode & 0x7F));

    if (!positive)
        codeword &= 0x7F;
    return codeword;
}

int16_t v91_codeword_to_linear(v91_law_t law, uint8_t codeword)
{
    if (law == V91_LAW_ALAW)
        return alaw_to_linear(codeword);
    return ulaw_to_linear(codeword);
}

uint8_t v91_linear_to_codeword(v91_law_t law, int16_t sample)
{
    if (law == V91_LAW_ALAW)
        return linear_to_alaw(sample);
    return linear_to_ulaw(sample);
}

int v91_tx_phase1_silence_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max)
{
    int i;
    int count;
    uint8_t silence;

    if (g711_max <= 0)
        return 0;

    count = (V91_PHASE1_SILENCE_SYMBOLS < g711_max) ? V91_PHASE1_SILENCE_SYMBOLS : g711_max;
    silence = v91_ucode_to_codeword(s->law, 0, true);
    for (i = 0; i < count; i++)
        g711_out[i] = silence;
    return count;
}

int v91_tx_ez_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max)
{
    int i;
    int count;
    uint8_t ez_symbol;

    if (g711_max <= 0)
        return 0;

    count = (V91_EZ_SYMBOLS < g711_max) ? V91_EZ_SYMBOLS : g711_max;
    ez_symbol = v91_ucode_to_codeword(s->law, 66, false);
    for (i = 0; i < count; i++)
        g711_out[i] = ez_symbol;
    return count;
}

int v91_tx_info_codewords(v91_state_t *s,
                          uint8_t *g711_out,
                          int g711_max,
                          const v91_info_frame_t *info)
{
    uint8_t bits[V91_INFO_TOTAL_BITS];
    int i;
    int sign;

    if (g711_max < V91_INFO_TOTAL_BITS)
        return 0;
    if (!v91_info_frame_validate(info, NULL, 0))
        return 0;

    v91_info_to_bits(bits, info);
    sign = 0;
    for (i = 0; i < V91_INFO_TOTAL_BITS; i++) {
        sign ^= bits[i];
        g711_out[i] = v91_ucode_to_codeword(s->law, 66, sign != 0);
    }
    return V91_INFO_TOTAL_BITS;
}

bool v91_rx_info_codewords(v91_state_t *s,
                           const uint8_t *g711_in,
                           int g711_len,
                           v91_info_frame_t *info_out)
{
    uint8_t bits[V91_INFO_TOTAL_BITS];
    int sign;
    int prev_sign;
    int i;

    (void) s;

    if (g711_len < V91_INFO_TOTAL_BITS)
        return false;

    prev_sign = 0;
    for (i = 0; i < V91_INFO_TOTAL_BITS; i++) {
        sign = (g711_in[i] & 0x80) ? 1 : 0;
        bits[i] = (uint8_t) (sign ^ prev_sign);
        prev_sign = sign;
    }
    return v91_info_from_bits(bits, info_out);
}

bool v91_info_build_diag(v91_state_t *s, const v91_info_frame_t *info, v91_info_diag_t *diag)
{
    int i;
    int sign;

    if (!diag || !v91_info_frame_validate(info, NULL, 0))
        return false;

    memset(diag, 0, sizeof(*diag));
    diag->frame = *info;
    v91_info_to_bits(diag->bits, info);
    sign = 0;
    for (i = 0; i < V91_INFO_TOTAL_BITS; i++) {
        sign ^= diag->bits[i];
        diag->codewords[i] = v91_ucode_to_codeword(s->law, 66, sign != 0);
    }
    diag->crc_field = v91_bits_to_u16(diag->bits, V91_INFO_CRC_START_BIT, 16);
    diag->crc_remainder = v91_crc_bits(diag->bits + V91_INFO_PAYLOAD_START_BIT,
                                       (V91_INFO_CRC_START_BIT + 16) - V91_INFO_PAYLOAD_START_BIT);
    diag->fill_ok = (diag->bits[0] && diag->bits[1] && diag->bits[2] && diag->bits[3]
                     && diag->bits[58] && diag->bits[59] && diag->bits[60] && diag->bits[61]);
    diag->sync_ok = true;
    for (i = 0; i < 8; i++) {
        if (diag->bits[4 + i] != ((0x72U >> i) & 1U)) {
            diag->sync_ok = false;
            break;
        }
    }
    diag->valid = (diag->fill_ok && diag->sync_ok && diag->crc_remainder == 0);
    return true;
}

bool v91_info_decode_diag(v91_state_t *s,
                          const uint8_t *g711_in,
                          int g711_len,
                          v91_info_diag_t *diag)
{
    int sign;
    int prev_sign;
    int i;

    if (!diag || g711_len < V91_INFO_TOTAL_BITS)
        return false;

    (void) s;

    memset(diag, 0, sizeof(*diag));
    memcpy(diag->codewords, g711_in, V91_INFO_TOTAL_BITS);
    prev_sign = 0;
    for (i = 0; i < V91_INFO_TOTAL_BITS; i++) {
        sign = (g711_in[i] & 0x80) ? 1 : 0;
        diag->bits[i] = (uint8_t) (sign ^ prev_sign);
        prev_sign = sign;
    }
    diag->crc_field = v91_bits_to_u16(diag->bits, V91_INFO_CRC_START_BIT, 16);
    diag->crc_remainder = v91_crc_bits(diag->bits + V91_INFO_PAYLOAD_START_BIT,
                                       (V91_INFO_CRC_START_BIT + 16) - V91_INFO_PAYLOAD_START_BIT);
    diag->fill_ok = (diag->bits[0] && diag->bits[1] && diag->bits[2] && diag->bits[3]
                     && diag->bits[58] && diag->bits[59] && diag->bits[60] && diag->bits[61]);
    diag->sync_ok = true;
    for (i = 0; i < 8; i++) {
        if (diag->bits[4 + i] != ((0x72U >> i) & 1U)) {
            diag->sync_ok = false;
            break;
        }
    }
    diag->valid = (diag->fill_ok && diag->sync_ok && diag->crc_remainder == 0);
    if (!diag->valid)
        return false;
    return v91_info_from_bits(diag->bits, &diag->frame);
}

int v91_tx_codewords(v91_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len)
{
    (void) s;

    if (g711_max <= 0 || data_len <= 0)
        return 0;

    if (data_len > g711_max)
        data_len = g711_max;

    memcpy(g711_out, data_in, (size_t) data_len);
    return data_len;
}

int v91_rx_codewords(v91_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len)
{
    (void) s;

    if (data_max <= 0 || g711_len <= 0)
        return 0;

    if (g711_len > data_max)
        g711_len = data_max;

    memcpy(data_out, g711_in, (size_t) g711_len);
    return g711_len;
}

int v91_tx_linear(v91_state_t *s,
                  int16_t *amp_out,
                  int amp_max,
                  const uint8_t *data_in,
                  int data_len)
{
    int i;
    int count;

    (void) s;

    if (amp_max <= 0 || data_len <= 0)
        return 0;

    count = (data_len < amp_max) ? data_len : amp_max;
    for (i = 0; i < count; i++)
        amp_out[i] = v91_codeword_to_linear(s->law, data_in[i]);
    return count;
}

int v91_rx_linear(v91_state_t *s,
                  uint8_t *data_out,
                  int data_max,
                  const int16_t *amp_in,
                  int amp_len)
{
    int i;
    int count;

    if (data_max <= 0 || amp_len <= 0)
        return 0;

    count = (amp_len < data_max) ? amp_len : data_max;
    for (i = 0; i < count; i++)
        data_out[i] = v91_linear_to_codeword(s->law, amp_in[i]);
    return count;
}

void v91_tx_idle(v91_state_t *s, int16_t *amp_out, int amp_len)
{
    int16_t sample;
    int i;

    sample = v91_codeword_to_linear(s->law, v91_idle_codeword(s->law));
    for (i = 0; i < amp_len; i++)
        amp_out[i] = sample;
}
