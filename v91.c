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

static int v91_clamp_positive(int value, int max_value)
{
    if (value < 1)
        return 1;
    if (value > max_value)
        return max_value;
    return value;
}

static int v91_dil_uchord_index(int training_ucode)
{
    int idx;

    idx = training_ucode >> 4;
    if (idx < 0)
        idx = 0;
    if (idx > 7)
        idx = 7;
    return idx;
}

static uint8_t v91_count_distinct_train_u(const v91_dil_desc_t *desc)
{
    bool seen[128];
    uint8_t count;
    int i;

    memset(seen, 0, sizeof(seen));
    count = 0;
    for (i = 0; i < desc->n; i++) {
        int ucode;

        ucode = desc->train_u[i] & 0x7F;
        if (!seen[ucode]) {
            seen[ucode] = true;
            count++;
        }
    }
    return count;
}

bool v91_analyse_dil_descriptor(const v91_dil_desc_t *desc, v91_dil_analysis_t *analysis_out)
{
    v91_dil_analysis_t analysis;
    bool seen_uchord[8];
    int i;

    if (!desc || !analysis_out)
        return false;

    memset(&analysis, 0, sizeof(analysis));
    memset(seen_uchord, 0, sizeof(seen_uchord));
    analysis.n = desc->n;
    analysis.lsp = desc->lsp;
    analysis.ltp = desc->ltp;
    analysis.unique_train_u = v91_count_distinct_train_u(desc);

    for (i = 0; i < 8; i++) {
        if (desc->ref[i] != 0)
            analysis.non_default_refs++;
        if (desc->h[i] != 1)
            analysis.non_default_h++;
    }
    for (i = 0; i < desc->n; i++) {
        int uchord_idx;

        uchord_idx = v91_dil_uchord_index(desc->train_u[i] & 0x7F);
        seen_uchord[uchord_idx] = true;
    }
    for (i = 0; i < 8; i++) {
        if (seen_uchord[i])
            analysis.repeated_uchords++;
    }

    analysis.default_like = (desc->n == V91_DEFAULT_DIL_SEGMENTS
                             && desc->lsp == 12
                             && desc->ltp == 12
                             && analysis.unique_train_u >= 120
                             && analysis.non_default_refs == 0
                             && analysis.non_default_h == 0);

    if (desc->n < V91_DEFAULT_DIL_SEGMENTS)
        analysis.impairment_score++;
    if (desc->n < 100)
        analysis.impairment_score++;
    if (desc->lsp != 12 || desc->ltp != 12)
        analysis.impairment_score++;
    if (desc->lsp < 12 || desc->ltp < 12)
        analysis.impairment_score++;
    if (analysis.unique_train_u < 64)
        analysis.impairment_score++;
    if (analysis.unique_train_u < 16)
        analysis.impairment_score++;
    if (analysis.non_default_refs != 0)
        analysis.impairment_score++;
    if (analysis.non_default_h != 0)
        analysis.impairment_score++;

    analysis.echo_limited = (analysis.impairment_score >= 3);
    if (analysis.impairment_score >= 5) {
        analysis.recommended_downstream_drn = 13;
        analysis.recommended_upstream_drn = 7;
    } else if (analysis.echo_limited) {
        analysis.recommended_downstream_drn = 16;
        analysis.recommended_upstream_drn = 10;
    } else {
        analysis.recommended_downstream_drn = 19;
        analysis.recommended_upstream_drn = 16;
    }

    *analysis_out = analysis;
    return true;
}

bool v91_note_received_dil(v91_state_t *s,
                           const v91_dil_desc_t *desc,
                           v91_dil_analysis_t *analysis_out)
{
    v91_dil_analysis_t analysis;

    if (!s || !desc)
        return false;
    if (!v91_analyse_dil_descriptor(desc, &analysis))
        return false;

    s->last_rx_dil = *desc;
    s->last_rx_dil_valid = true;
    s->last_rx_dil_analysis = analysis;
    s->last_rx_dil_analysis_valid = true;
    if (analysis_out)
        *analysis_out = analysis;
    return true;
}

static int v91_scramble_bit(v91_state_t *s, int in_bit)
{
    int out_bit;

    /* V.91 uses the V.34 GPC scrambler for PCM startup sequences. */
    out_bit = (in_bit ^ (int) (s->scramble_reg >> 17) ^ (int) (s->scramble_reg >> (23 - 1))) & 1;
    s->scramble_reg = (s->scramble_reg << 1) | (uint32_t) out_bit;
    return out_bit;
}

static int v91_descramble_reg_bit(uint32_t *reg, int in_bit)
{
    int out_bit;

    out_bit = (in_bit ^ (int) (*reg >> 17) ^ (int) (*reg >> (23 - 1))) & 1;
    *reg = (*reg << 1) | (uint32_t) in_bit;
    return out_bit;
}

static void v91_complete_alignment_signal(v91_state_t *s)
{
    s->frame_aligned = true;
    s->retrain_requested = false;
    s->circuit_107_on = true;
    s->next_frame_interval = 0;
}

static int v91_tx_scrambled_ones_codewords(v91_state_t *s,
                                           uint8_t *g711_out,
                                           int g711_max,
                                           int nsymbols,
                                           bool reset_coders)
{
    int i;
    int sign;

    if (g711_max <= 0 || nsymbols <= 0 || nsymbols > g711_max)
        return 0;

    if (reset_coders) {
        s->scramble_reg = 0;
        s->diff_sign = 0;
    }
    sign = s->diff_sign;
    for (i = 0; i < nsymbols; i++) {
        sign ^= v91_scramble_bit(s, 1);
        g711_out[i] = v91_ucode_to_codeword(s->law, 66, sign != 0);
    }
    s->diff_sign = sign;
    return nsymbols;
}

static int v91_tx_diff_bits_codewords(v91_state_t *s,
                                      uint8_t *g711_out,
                                      int g711_max,
                                      const uint8_t *bits,
                                      int nbits,
                                      bool reset_coders,
                                      bool scramble_bits)
{
    int i;
    int sign;

    if (!s || !g711_out || !bits || nbits <= 0 || nbits > g711_max)
        return 0;

    if (reset_coders) {
        s->scramble_reg = 0;
        s->diff_sign = 0;
    }
    sign = s->diff_sign;
    for (i = 0; i < nbits; i++) {
        int bit;

        bit = bits[i] & 1U;
        if (scramble_bits)
            bit = v91_scramble_bit(s, bit);
        sign ^= bit;
        g711_out[i] = v91_ucode_to_codeword(s->law, 66, sign != 0);
    }
    s->diff_sign = sign;
    return nbits;
}

static int v91_tx_zeros_mapped_as_scr(v91_state_t *s,
                                      uint8_t *g711_out,
                                      int g711_max,
                                      int nsymbols)
{
    uint8_t bits[VPCM_CP_MAX_BITS];
    int i;

    if (nsymbols <= 0 || nsymbols > VPCM_CP_MAX_BITS)
        return 0;
    for (i = 0; i < nsymbols; i++)
        bits[i] = 0;
    return v91_tx_diff_bits_codewords(s, g711_out, g711_max, bits, nsymbols, false, true);
}

static int v91_select_b1_ucode(const vpcm_cp_frame_t *cp, int frame_interval, int k, int symbol_index)
{
    int constellation_idx;
    int population;
    int ordinal;
    int direction;
    int ucode;

    if (!cp || frame_interval < 0 || frame_interval >= VPCM_CP_FRAME_INTERVALS)
        return -1;
    if (cp->constellation_count < 1 || cp->constellation_count > VPCM_CP_MAX_CONSTELLATIONS)
        return -1;
    constellation_idx = cp->dfi[frame_interval];
    if (constellation_idx >= cp->constellation_count)
        return -1;

    population = vpcm_cp_mask_population(cp->masks[constellation_idx]);
    if (population <= 0)
        return -1;

    /* Bring-up mapper: choose a deterministic active codeword within the
       negotiated constellation. The ordinal depends on K, frame interval,
       and symbol position so different negotiated rates produce distinct B1
       patterns while staying within the selected CP mask set. */
    ordinal = (k - 15 + frame_interval * 5 + symbol_index * 7) % population;
    if (ordinal < 0)
        ordinal += population;
    direction = ((k + frame_interval + symbol_index) & 1) ? -1 : 1;

    if (direction > 0) {
        for (ucode = 0; ucode < VPCM_CP_MASK_BITS; ucode++) {
            if (vpcm_cp_mask_get(cp->masks[constellation_idx], ucode)) {
                if (ordinal-- == 0)
                    return ucode;
            }
        }
    } else {
        for (ucode = VPCM_CP_MASK_BITS - 1; ucode >= 0; ucode--) {
            if (vpcm_cp_mask_get(cp->masks[constellation_idx], ucode)) {
                if (ordinal-- == 0)
                    return ucode;
            }
        }
    }
    return -1;
}

static void v91_bits_per_interval(int k, int bits_per_interval[VPCM_CP_FRAME_INTERVALS])
{
    int base;
    int rem;
    int i;

    base = k / VPCM_CP_FRAME_INTERVALS;
    rem = k % VPCM_CP_FRAME_INTERVALS;
    for (i = 0; i < VPCM_CP_FRAME_INTERVALS; i++)
        bits_per_interval[i] = base + ((i < rem) ? 1 : 0);
}

static int v91_select_active_ucode_by_ordinal(const uint8_t mask[VPCM_CP_MASK_BYTES], int ordinal)
{
    int ucode;

    for (ucode = 0; ucode < VPCM_CP_MASK_BITS; ucode++) {
        if (vpcm_cp_mask_get(mask, ucode)) {
            if (ordinal-- == 0)
                return ucode;
        }
    }
    return -1;
}

static int v91_ucode_ordinal_in_mask(const uint8_t mask[VPCM_CP_MASK_BYTES], int ucode)
{
    int ordinal;
    int i;

    ordinal = 0;
    for (i = 0; i < VPCM_CP_MASK_BITS; i++) {
        if (vpcm_cp_mask_get(mask, i)) {
            if (i == ucode)
                return ordinal;
            ordinal++;
        }
    }
    return -1;
}

static bool v91_data_mode_supported(const vpcm_cp_frame_t *cp, int k)
{
    int bits_per_interval[VPCM_CP_FRAME_INTERVALS];
    int i;

    if (!cp)
        return false;
    if (!vpcm_cp_validate(cp, NULL, 0))
        return false;
    if (k <= 0)
        return false;

    v91_bits_per_interval(k, bits_per_interval);
    for (i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        int constellation_idx;
        int population;
        int needed;

        constellation_idx = cp->dfi[i];
        if (constellation_idx >= cp->constellation_count)
            return false;
        population = vpcm_cp_mask_population(cp->masks[constellation_idx]);
        needed = 1 << bits_per_interval[i];
        if (population < needed)
            return false;
    }
    return true;
}

static int v91_primary_bits_per_frame(const vpcm_cp_frame_t *cp, int k)
{
    if (cp && cp->transparent_mode_granted)
        return 48;
    return k + 6;
}

static int v91_rx_diff_bits_stateful(v91_state_t *s,
                                     const uint8_t *g711_in,
                                     int g711_len,
                                     uint8_t *bits_out,
                                     bool reset_coders,
                                     bool scramble_bits)
{
    int i;

    if (!s || !g711_in || !bits_out || g711_len <= 0 || g711_len > VPCM_CP_MAX_BITS)
        return 0;

    if (reset_coders) {
        s->rx_prev_sign = 0;
        s->rx_scramble_reg = 0;
    }
    for (i = 0; i < g711_len; i++) {
        int sign;
        int bit;

        sign = (g711_in[i] & 0x80) ? 1 : 0;
        bit = sign ^ s->rx_prev_sign;
        s->rx_prev_sign = sign;
        if (scramble_bits)
            bit = v91_descramble_reg_bit(&s->rx_scramble_reg, bit);
        bits_out[i] = (uint8_t) bit;
    }
    return g711_len;
}

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
    memset(s, 0, sizeof(*s));
    s->law = law;
    s->mode = mode;
    s->scramble_reg = 0;
    s->rx_scramble_reg = 0;
    s->diff_sign = 0;
    s->rx_prev_sign = 0;
    s->frame_aligned = false;
    s->retrain_requested = false;
    s->circuit_107_on = false;
    s->next_frame_interval = -1;
    s->data_mode_active = false;
    s->active_k = 0;
    s->active_s = 0;
    s->active_primary_bits_per_frame = 0;
    s->active_transparent_mode = false;
    s->tx_bit_accum = 0;
    s->tx_bit_count = 0;
    s->rx_bit_accum = 0;
    s->rx_bit_count = 0;
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

static int v91_codeword_to_ucode(v91_law_t law, uint8_t codeword)
{
    int ucode;

    if (law == V91_LAW_ULAW)
        return (0xFF - codeword) & 0x7F;

    for (ucode = 0; ucode < 128; ucode++) {
        if (v91_ucode_to_alaw[ucode] == codeword || (v91_ucode_to_alaw[ucode] & 0x7F) == (codeword & 0x7F))
            return ucode;
    }
    return -1;
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

int v91_tx_eu_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max)
{
    int i;
    int count;

    if (g711_max <= 0)
        return 0;

    count = (V91_EU_SYMBOLS < g711_max) ? V91_EU_SYMBOLS : g711_max;
    for (i = 0; i < count; i++)
        g711_out[i] = v91_ucode_to_codeword(s->law, 66, s->diff_sign != 0);

    if (count == V91_EU_SYMBOLS)
        v91_complete_alignment_signal(s);
    return count;
}

int v91_tx_em_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max)
{
    int i;
    int count;
    int sign;

    if (g711_max <= 0)
        return 0;

    count = (V91_EM_SYMBOLS < g711_max) ? V91_EM_SYMBOLS : g711_max;
    sign = s->diff_sign;
    for (i = 0; i < count; i++) {
        sign ^= v91_scramble_bit(s, 0);
        g711_out[i] = v91_ucode_to_codeword(s->law, 66, sign != 0);
    }
    s->diff_sign = sign;
    if (count == V91_EM_SYMBOLS)
        v91_complete_alignment_signal(s);
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
    s->scramble_reg = 0;
    s->rx_scramble_reg = 0;
    s->diff_sign = 0;
    s->rx_prev_sign = 0;
    s->frame_aligned = false;
    s->circuit_107_on = false;
    s->next_frame_interval = -1;
    return count;
}

int v91_tx_phil_codewords(v91_state_t *s,
                          uint8_t *g711_out,
                          int g711_max,
                          int nsymbols,
                          bool continue_from_current)
{
    /* PHIL resets its line coders unless it follows J, which we have not
       implemented yet. The caller can still request state continuity for
       future J->PHIL use. */
    return v91_tx_scrambled_ones_codewords(s,
                                           g711_out,
                                           g711_max,
                                           nsymbols,
                                           !continue_from_current);
}

int v91_tx_scr_codewords(v91_state_t *s,
                         uint8_t *g711_out,
                         int g711_max,
                         int nsymbols)
{
    if (nsymbols <= 0 || (nsymbols % 6) != 0)
        return 0;
    return v91_tx_scrambled_ones_codewords(s,
                                           g711_out,
                                           g711_max,
                                           nsymbols,
                                           true);
}

bool v91_rx_scr_codewords(v91_state_t *s,
                          const uint8_t *g711_in,
                          int g711_len,
                          bool continue_from_current)
{
    uint8_t bits[VPCM_CP_MAX_BITS];
    int i;

    if (!s || !g711_in || g711_len <= 0 || g711_len > VPCM_CP_MAX_BITS || (g711_len % 6) != 0)
        return false;
    if (v91_rx_diff_bits_stateful(s, g711_in, g711_len, bits, !continue_from_current, true) != g711_len)
        return false;
    for (i = 0; i < g711_len; i++) {
        if (bits[i] != 1)
            return false;
    }
    return true;
}

int v91_tx_es_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max)
{
    return v91_tx_zeros_mapped_as_scr(s, g711_out, g711_max, V91_ES_SYMBOLS);
}

bool v91_rx_es_codewords(v91_state_t *s,
                         const uint8_t *g711_in,
                         int g711_len,
                         bool continue_from_cp)
{
    uint8_t bits[VPCM_CP_MAX_BITS];
    int i;

    if (!s || !g711_in || g711_len != V91_ES_SYMBOLS)
        return false;
    if (v91_rx_diff_bits_stateful(s, g711_in, g711_len, bits, !continue_from_cp, true) != g711_len)
        return false;
    for (i = 0; i < g711_len; i++) {
        if (bits[i] != 0)
            return false;
    }
    return true;
}

int v91_tx_cp_codewords(v91_state_t *s,
                        uint8_t *g711_out,
                        int g711_max,
                        const vpcm_cp_frame_t *cp,
                        bool continue_from_scr)
{
    uint8_t bits[VPCM_CP_MAX_BITS];
    int nbits;

    if (!s || !cp)
        return 0;
    if (!vpcm_cp_encode_bits(cp, bits, &nbits))
        return 0;
    if (nbits > g711_max)
        return 0;

    s->last_tx_cp = *cp;
    s->last_tx_cp_valid = true;
    return v91_tx_diff_bits_codewords(s,
                                      g711_out,
                                      g711_max,
                                      bits,
                                      nbits,
                                      !continue_from_scr,
                                      true);
}

bool v91_rx_cp_codewords(v91_state_t *s,
                         const uint8_t *g711_in,
                         int g711_len,
                         vpcm_cp_frame_t *cp_out,
                         bool continue_from_scr)
{
    uint8_t bits[VPCM_CP_MAX_BITS];
    vpcm_cp_frame_t cp;

    if (!s || !g711_in || !cp_out || g711_len <= 0 || g711_len > VPCM_CP_MAX_BITS)
        return false;

    if (v91_rx_diff_bits_stateful(s, g711_in, g711_len, bits, !continue_from_scr, true) != g711_len)
        return false;
    if (!vpcm_cp_decode_bits(bits, g711_len, &cp)) {
        s->last_rx_cp_valid = false;
        return false;
    }
    s->last_rx_cp = cp;
    s->last_rx_cp_valid = true;
    *cp_out = cp;
    return true;
}

int v91_tx_b1_codewords(v91_state_t *s,
                        uint8_t *g711_out,
                        int g711_max,
                        const vpcm_cp_frame_t *cp)
{
    int i;
    int ucode;
    int k;

    if (!s || !g711_out || !cp || g711_max < V91_B1_SYMBOLS)
        return 0;

    /* Bring-up implementation: choose active PCM codewords from the
       negotiated CP masks with a deterministic K-sensitive selector. */
    k = vpcm_cp_drn_to_k(cp->drn);
    if (k <= 0)
        return 0;
    for (i = 0; i < V91_B1_SYMBOLS; i++) {
        int fi;

        fi = i % VPCM_CP_FRAME_INTERVALS;
        ucode = v91_select_b1_ucode(cp, fi, k, i);
        if (ucode < 0)
            return 0;
        g711_out[i] = v91_ucode_to_codeword(s->law, ucode, true);
    }
    return V91_B1_SYMBOLS;
}

bool v91_rx_b1_codewords(v91_state_t *s,
                         const uint8_t *g711_in,
                         int g711_len,
                         const vpcm_cp_frame_t *cp)
{
    uint8_t expected[V91_B1_SYMBOLS];

    if (!s || !g711_in || !cp || g711_len != V91_B1_SYMBOLS)
        return false;
    if (v91_tx_b1_codewords(s, expected, V91_B1_SYMBOLS, cp) != V91_B1_SYMBOLS)
        return false;
    return memcmp(expected, g711_in, V91_B1_SYMBOLS) == 0;
}

int v91_tx_info_codewords(v91_state_t *s,
                          uint8_t *g711_out,
                          int g711_max,
                          const v91_info_frame_t *info)
{
    uint8_t bits[V91_INFO_TOTAL_BITS];
    int count;

    if (g711_max < V91_INFO_TOTAL_BITS)
        return 0;
    if (!v91_info_frame_validate(info, NULL, 0))
        return 0;

    s->last_tx_info = *info;
    s->last_tx_info_valid = true;
    v91_info_to_bits(bits, info);
    count = v91_tx_diff_bits_codewords(s, g711_out, g711_max, bits, V91_INFO_TOTAL_BITS, true, false);
    if (count != V91_INFO_TOTAL_BITS)
        return 0;
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
    if (!v91_info_from_bits(bits, info_out)) {
        s->last_rx_info_valid = false;
        return false;
    }
    s->last_rx_info = *info_out;
    s->last_rx_info_valid = true;
    return true;
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
    if (!diag->valid) {
        s->last_rx_info_valid = false;
        return false;
    }
    if (!v91_info_from_bits(diag->bits, &diag->frame)) {
        s->last_rx_info_valid = false;
        return false;
    }
    s->last_rx_info = diag->frame;
    s->last_rx_info_valid = true;
    return true;
}

void v91_default_dil_init(v91_dil_desc_t *desc)
{
    int i;
    int pos;

    memset(desc, 0, sizeof(*desc));
    desc->n = V91_DEFAULT_DIL_SEGMENTS;
    desc->lsp = 12;
    desc->ltp = 12;
    for (i = 0; i < 12; i++) {
        desc->sp[i] = (uint8_t) ((0x0FC0U >> i) & 1U);
        desc->tp[i] = (uint8_t) ((0x0FFFU >> i) & 1U);
    }
    for (i = 0; i < 8; i++) {
        desc->h[i] = 1;
        desc->ref[i] = 0;
    }

    pos = 0;
    for (i = 0; i < 62; i++) {
        desc->train_u[pos++] = (uint8_t) (124 - i);
        desc->train_u[pos++] = (uint8_t) i;
    }
    desc->train_u[pos++] = 62;
}

int v91_dil_symbol_count(const v91_dil_desc_t *desc)
{
    int count;
    int i;

    if (!desc)
        return 0;

    count = 0;
    for (i = 0; i < desc->n; i++) {
        int uchord_idx = v91_dil_uchord_index(desc->train_u[i] & 0x7F);
        count += (int) (desc->h[uchord_idx] + 1) * 6;
    }
    return count;
}

int v91_tx_dil_codewords(v91_state_t *s,
                         uint8_t *g711_out,
                         int g711_max,
                         const v91_dil_desc_t *desc)
{
    int n;
    int lsp;
    int ltp;
    int out_pos;
    int seg_idx;

    if (!s || !g711_out || !desc)
        return 0;

    n = desc->n;
    if (n <= 0 || n > 255)
        return 0;
    lsp = v91_clamp_positive(desc->lsp, 128);
    ltp = v91_clamp_positive(desc->ltp, 128);

    out_pos = 0;
    for (seg_idx = 0; seg_idx < n; seg_idx++) {
        int training_ucode;
        int uchord_idx;
        int seg_len;
        int pos;

        training_ucode = desc->train_u[seg_idx] & 0x7F;
        uchord_idx = v91_dil_uchord_index(training_ucode);
        seg_len = (int) (desc->h[uchord_idx] + 1) * 6;
        if (out_pos + seg_len > g711_max)
            return 0;

        for (pos = 0; pos < seg_len; pos++) {
            int sp_bit;
            int tp_bit;
            int ucode;

            sp_bit = desc->sp[pos % lsp] ? 1 : 0;
            tp_bit = desc->tp[pos % ltp] ? 1 : 0;
            ucode = tp_bit ? training_ucode : (desc->ref[uchord_idx] & 0x7F);
            g711_out[out_pos++] = v91_ucode_to_codeword(s->law, ucode, sp_bit != 0);
            if (s->frame_aligned)
                s->next_frame_interval = (s->next_frame_interval + 1) % 6;
        }
    }

    s->last_tx_dil = *desc;
    s->last_tx_dil_valid = true;
    return out_pos;
}

int v91_tx_default_dil_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max)
{
    v91_dil_desc_t desc;

    v91_default_dil_init(&desc);
    return v91_tx_dil_codewords(s, g711_out, g711_max, &desc);
}

int v91_tx_startup_dil_sequence_codewords(v91_state_t *s,
                                          uint8_t *g711_out,
                                          int g711_max,
                                          const v91_dil_desc_t *peer_dil,
                                          v91_align_signal_t *align_out)
{
    v91_dil_desc_t default_dil;
    const v91_dil_desc_t *chosen_dil;
    bool both_default_dil;
    int emitted;
    int dil_len;

    if (!s || !g711_out || g711_max <= 0)
        return 0;
    if (!s->last_tx_info_valid || !s->last_rx_info_valid)
        return 0;

    both_default_dil = s->last_tx_info.request_default_dil && s->last_rx_info.request_default_dil;
    chosen_dil = peer_dil;
    if (chosen_dil == NULL) {
        v91_default_dil_init(&default_dil);
        chosen_dil = &default_dil;
    }
    dil_len = v91_dil_symbol_count(chosen_dil);
    if (dil_len <= 0 || g711_max < (V91_EU_SYMBOLS + dil_len))
        return 0;

    if (both_default_dil) {
        emitted = v91_tx_eu_codewords(s, g711_out, g711_max);
        if (align_out)
            *align_out = V91_ALIGN_EU;
    } else {
        emitted = v91_tx_em_codewords(s, g711_out, g711_max);
        if (align_out)
            *align_out = V91_ALIGN_EM;
    }
    if (emitted != V91_EU_SYMBOLS)
        return 0;

    dil_len = v91_tx_dil_codewords(s, g711_out + emitted, g711_max - emitted, chosen_dil);
    if (dil_len <= 0)
        return 0;
    return emitted + dil_len;
}

void v91_note_frame_sync_loss(v91_state_t *s)
{
    s->frame_aligned = false;
    s->retrain_requested = true;
    s->next_frame_interval = -1;
    s->data_mode_active = false;
    s->active_k = 0;
    s->active_s = 0;
    s->active_primary_bits_per_frame = 0;
    s->active_transparent_mode = false;
    s->tx_bit_accum = 0;
    s->tx_bit_count = 0;
    s->rx_bit_accum = 0;
    s->rx_bit_count = 0;
}

bool v91_activate_data_mode(v91_state_t *s, const vpcm_cp_frame_t *cp)
{
    int k;

    if (!s || !cp)
        return false;
    k = vpcm_cp_drn_to_k(cp->drn);
    if (!v91_data_mode_supported(cp, k))
        return false;

    s->active_cp = *cp;
    s->active_k = k;
    s->active_s = 6;
    s->active_primary_bits_per_frame = v91_primary_bits_per_frame(cp, k);
    s->active_transparent_mode = (cp->transparent_mode_granted && cp->drn == 28);
    s->data_mode_active = true;
    s->tx_bit_accum = 0;
    s->tx_bit_count = 0;
    s->rx_bit_accum = 0;
    s->rx_bit_count = 0;
    return true;
}

void v91_deactivate_data_mode(v91_state_t *s)
{
    if (!s)
        return;
    s->data_mode_active = false;
    s->active_k = 0;
    s->active_s = 0;
    s->active_primary_bits_per_frame = 0;
    s->active_transparent_mode = false;
    s->tx_bit_accum = 0;
    s->tx_bit_count = 0;
    s->rx_bit_accum = 0;
    s->rx_bit_count = 0;
}

int v91_tx_codewords(v91_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len)
{
    int data_pos;
    int out_pos;

    if (g711_max <= 0 || data_len <= 0)
        return 0;

    if (!s || !s->data_mode_active) {
        if (data_len > g711_max)
            data_len = g711_max;

        memcpy(g711_out, data_in, (size_t) data_len);
        return data_len;
    }

    if (s->active_transparent_mode) {
        if (data_len > g711_max)
            data_len = g711_max;
        memcpy(g711_out, data_in, (size_t) data_len);
        return data_len;
    }

    data_pos = 0;
    out_pos = 0;
    while (out_pos + VPCM_CP_FRAME_INTERVALS <= g711_max) {
        int bits_per_interval[VPCM_CP_FRAME_INTERVALS];
        int frame_interval;

        while (s->tx_bit_count < s->active_k && data_pos < data_len) {
            s->tx_bit_accum |= ((uint64_t) data_in[data_pos++]) << s->tx_bit_count;
            s->tx_bit_count += 8;
        }
        if (s->tx_bit_count < s->active_k)
            break;

        v91_bits_per_interval(s->active_k, bits_per_interval);
        for (frame_interval = 0; frame_interval < VPCM_CP_FRAME_INTERVALS; frame_interval++) {
            int constellation_idx;
            int nbits;
            int ordinal;
            int sign_bit;
            int ucode;
            uint64_t mask;

            constellation_idx = s->active_cp.dfi[frame_interval];
            nbits = bits_per_interval[frame_interval];
            while (s->tx_bit_count < (nbits + 1) && data_pos < data_len) {
                s->tx_bit_accum |= ((uint64_t) data_in[data_pos++]) << s->tx_bit_count;
                s->tx_bit_count += 8;
            }
            if (s->tx_bit_count < (nbits + 1))
                return out_pos;
            mask = (((uint64_t) 1) << nbits) - 1U;
            ordinal = (int) (s->tx_bit_accum & mask);
            s->tx_bit_accum >>= nbits;
            s->tx_bit_count -= nbits;
            sign_bit = (int) (s->tx_bit_accum & 1U);
            s->tx_bit_accum >>= 1;
            s->tx_bit_count -= 1;
            ucode = v91_select_active_ucode_by_ordinal(s->active_cp.masks[constellation_idx], ordinal);
            if (ucode < 0)
                return out_pos;
            g711_out[out_pos++] = v91_ucode_to_codeword(s->law, ucode, sign_bit != 0);
        }
    }
    return out_pos;
}

int v91_rx_codewords(v91_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len)
{
    int in_pos;
    int out_pos;

    if (data_max <= 0 || g711_len <= 0)
        return 0;

    if (!s || !s->data_mode_active) {
        if (g711_len > data_max)
            g711_len = data_max;

        memcpy(data_out, g711_in, (size_t) g711_len);
        return g711_len;
    }

    if (s->active_transparent_mode) {
        if (g711_len > data_max)
            g711_len = data_max;
        memcpy(data_out, g711_in, (size_t) g711_len);
        return g711_len;
    }

    in_pos = 0;
    out_pos = 0;
    while (in_pos + VPCM_CP_FRAME_INTERVALS <= g711_len) {
        int bits_per_interval[VPCM_CP_FRAME_INTERVALS];
        int frame_interval;

        v91_bits_per_interval(s->active_k, bits_per_interval);
        for (frame_interval = 0; frame_interval < VPCM_CP_FRAME_INTERVALS; frame_interval++) {
            int constellation_idx;
            int nbits;
            int ucode;
            int ordinal;
            int sign_bit;

            constellation_idx = s->active_cp.dfi[frame_interval];
            nbits = bits_per_interval[frame_interval];
            sign_bit = (g711_in[in_pos] & 0x80) ? 1 : 0;
            ucode = v91_codeword_to_ucode(s->law, g711_in[in_pos++]);
            if (ucode < 0)
                return out_pos;
            ordinal = v91_ucode_ordinal_in_mask(s->active_cp.masks[constellation_idx], ucode);
            if (ordinal < 0 || ordinal >= (1 << nbits))
                return out_pos;
            s->rx_bit_accum |= ((uint64_t) ordinal) << s->rx_bit_count;
            s->rx_bit_count += nbits;
            s->rx_bit_accum |= ((uint64_t) sign_bit) << s->rx_bit_count;
            s->rx_bit_count += 1;
            while (s->rx_bit_count >= 8 && out_pos < data_max) {
                data_out[out_pos++] = (uint8_t) (s->rx_bit_accum & 0xFFU);
                s->rx_bit_accum >>= 8;
                s->rx_bit_count -= 8;
            }
        }
    }
    return out_pos;
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
