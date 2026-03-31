/*
 * v90.c — V.90 digital modem module
 *
 * ITU-T V.90 digital (server) side implementation.
 *
 * Phase 2: Wraps SpanDSP V.34 for INFO0d/INFO1d exchange.
 * Phase 3: Generates PCM codewords (Jd, J'd, Sd, S̄d, TRN1d) directly.
 * Phase 4: V.90-specific MP/CP exchange (TODO).
 * Data:    Modulus encoder → PCM codewords at 8 kHz.
 */

#include "v90.h"
#include "vpcm_cp.h"

#include <spandsp.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* V.90 downstream encoder constants (ITU-T V.90 §5) */
#define V90_MI          128     /* Default constellation points per frame interval */
#define V90_FRAME_LEN   6       /* Symbols per data frame */

/* Jd frame is 72 bits (Table 13): 17 sync + 51 data + 4 fill */
#define V90_JD_BITS         72

/* Phase 4 timing constants (ITU-T V.90 §9.4.1) */
#define V90_RI_SYMBOLS   64   /* Ri duration: ≥8 ms at 8 kHz */
#define V90_B1D_SYMBOLS  48   /* B1d: 6 frame-intervals of 6 symbols each */

/* Sd: 64 repetitions of 6-symbol pattern = 384 symbols */
#define V90_SD_REPS     64
#define V90_SD_BAR_REPS 8

/* TRN1d: multiple of 6 symbols; spec requires ≥2040T (§9.3.1.4).
 * Use 2046 = 341×6, nearest multiple of 6 above 2040. */
#define V90_TRN1D_LEN   2046

#define V90_DIL_MAX_PAT_BITS 128
#define V90_DIL_MAX_SEGMENTS 255

/* Ucode-to-PCM codeword mapping (ITU-T V.90 Table 1/V.90) */
/* A-law positive codewords indexed by Ucode */
static const uint8_t v90_ucode_to_alaw[128] = {
    /* Ucode   0-  7 */ 0xD5, 0xD4, 0xD7, 0xD6, 0xD1, 0xD0, 0xD3, 0xD2,
    /* Ucode   8- 15 */ 0xDD, 0xDC, 0xDF, 0xDE, 0xD9, 0xD8, 0xDB, 0xDA,
    /* Ucode  16- 23 */ 0xC5, 0xC4, 0xC7, 0xC6, 0xC1, 0xC0, 0xC3, 0xC2,
    /* Ucode  24- 31 */ 0xCD, 0xCC, 0xCF, 0xCE, 0xC9, 0xC8, 0xCB, 0xCA,
    /* Ucode  32- 39 */ 0xF5, 0xF4, 0xF7, 0xF6, 0xF1, 0xF0, 0xF3, 0xF2,
    /* Ucode  40- 47 */ 0xFD, 0xFC, 0xFF, 0xFE, 0xF9, 0xF8, 0xFB, 0xFA,
    /* Ucode  48- 55 */ 0xE5, 0xE4, 0xE7, 0xE6, 0xE1, 0xE0, 0xE3, 0xE2,
    /* Ucode  56- 63 */ 0xED, 0xEC, 0xEF, 0xEE, 0xE9, 0xE8, 0xEB, 0xEA,
    /* Ucode  64- 71 */ 0x95, 0x94, 0x97, 0x96, 0x91, 0x90, 0x93, 0x92,
    /* Ucode  72- 79 */ 0x9D, 0x9C, 0x9F, 0x9E, 0x99, 0x98, 0x9B, 0x9A,
    /* Ucode  80- 87 */ 0x85, 0x84, 0x87, 0x86, 0x81, 0x80, 0x83, 0x82,
    /* Ucode  88- 95 */ 0x8D, 0x8C, 0x8F, 0x8E, 0x89, 0x88, 0x8B, 0x8A,
    /* Ucode  96-103 */ 0xB5, 0xB4, 0xB7, 0xB6, 0xB1, 0xB0, 0xB3, 0xB2,
    /* Ucode 104-111 */ 0xBD, 0xBC, 0xBF, 0xBE, 0xB9, 0xB8, 0xBB, 0xBA,
    /* Ucode 112-119 */ 0xA5, 0xA4, 0xA7, 0xA6, 0xA1, 0xA0, 0xA3, 0xA2,
    /* Ucode 120-127 */ 0xAD, 0xAC, 0xAF, 0xAE, 0xA9, 0xA8, 0xAB, 0xAA,
};

/* V.90 scrambler (V.34 polynomial GPC, x^23 + x^5 + 1) */
typedef struct {
    uint32_t sr;
} v90_scrambler_t;

static void v90_scrambler_init(v90_scrambler_t *sc)
{
    sc->sr = 0;  /* V.90 §8.4: scrambler initialized to zero */
}

static int v90_scramble_bit(v90_scrambler_t *sc, int in_bit)
{
    int fb = ((sc->sr >> 22) ^ (sc->sr >> 4)) & 1;
    int out_bit = in_bit ^ fb;
    sc->sr = ((sc->sr << 1) | out_bit) & 0x7FFFFF;
    return out_bit;
}

static uint8_t v90_scramble_byte(v90_scrambler_t *sc, uint8_t in)
{
    uint8_t out = 0;
    for (int i = 0; i < 8; i++) {
        int in_bit = (in >> i) & 1;
        int out_bit = v90_scramble_bit(sc, in_bit);
        out |= (uint8_t)(out_bit << i);
    }
    return out;
}

static int v90_descramble_reg_bit(uint32_t *reg, int in_bit)
{
    int out_bit;

    out_bit = (in_bit ^ (int) (*reg >> 22) ^ (int) (*reg >> 4)) & 1;
    *reg = (*reg << 1) | (uint32_t) in_bit;
    return out_bit;
}

struct v90_state_s {
    v34_state_t     *v34;
    v90_law_t        law;

    /* Phase 3/4 TX state */
    v90_tx_phase_t   tx_phase;
    int              u_info;        /* U_INFO Ucode from analog modem's INFO1a */
    v90_scrambler_t  scrambler;
    int              diff_enc;      /* Differential encoder state (last sign bit) */
    int              sample_count;  /* Sample counter within current sub-state */
    int              rep_count;     /* Repetition counter (for Jd, Sd, etc.) */
    bool             phase4_hold_logged;
    bool             jd_terminate_requested;
    bool             training_complete;
    bool             dil_requested;
    bool             dil_terminate_requested;
    bool             use_internal_v34_tx;

    /* Jd frame data */
    uint8_t          jd_bits[16];   /* Jd frame packed into bytes (72 bits) */
    int              jd_bit_pos;    /* Current bit position in Jd frame */

    /* DIL descriptor/state */
    v90_dil_desc_t   dil;
    int              dil_segment_index;
    int              dil_pos_in_segment;

    /* Phase 4 CP state */
    bool             cp_ready;                  /* TRN2d→CP transition armed */
    vpcm_cp_frame_t  cp_frame;                  /* CP frame to transmit */
    uint8_t          cp_bits[VPCM_CP_MAX_BITS]; /* Encoded CP bits (one per byte) */
    int              cp_nbits;                  /* Total encoded CP bits */
    int              cp_bit_pos;                /* Current bit index in cp_bits */

    /* Downstream PCM encoder state (data mode) */
    int              prev_sign;     /* §5.4.5.1 differential sign coding */
    uint32_t         rx_scramble_reg;
    int              rx_prev_sign;

    bool             owns_v34;      /* true if we allocated v34 (v90_init), false if external */
};

/* ---- PCM codeword helpers ---- */

static inline uint8_t ucode_to_pcm_positive(v90_law_t law, int ucode)
{
    if (law == V90_LAW_ALAW)
        return v90_ucode_to_alaw[ucode & 0x7F];
    return (uint8_t)(0xFF - ucode);  /* µ-law */
}

static inline int16_t v90_pcm_to_linear(v90_law_t law, uint8_t codeword)
{
    if (law == V90_LAW_ALAW)
        return alaw_to_linear(codeword);
    return ulaw_to_linear(codeword);
}

static inline uint8_t v90_pcm_idle(v90_law_t law)
{
    return (law == V90_LAW_ALAW) ? (uint8_t)0xD5 : (uint8_t)0xFF;
}

/* Generate a signed PCM sample from a Ucode and sign bit.
 * sign=1 → positive, sign=0 → negative. */
static inline int16_t v90_pcm_signed(v90_law_t law, int ucode, int sign)
{
    uint8_t pcm = ucode_to_pcm_positive(law, ucode);
    pcm = (uint8_t) ((pcm & 0x7F) | (sign ? 0x80 : 0x00));  /* bit7 = polarity */
    return v90_pcm_to_linear(law, pcm);
}

static void v90_bits_put(uint8_t *buf, int *bit_pos, uint32_t value, int bits)
{
    int i;

    for (i = 0; i < bits; i++) {
        int pos = *bit_pos + i;
        if (value & (1U << i))
            buf[pos >> 3] |= (uint8_t) (1U << (pos & 7));
    }
    *bit_pos += bits;
}

static int v90_bits_get(const uint8_t *buf, int bit_pos, int bits)
{
    int i;
    int value;

    value = 0;
    for (i = 0; i < bits; i++) {
        if (buf[(bit_pos + i) >> 3] & (1U << ((bit_pos + i) & 7)))
            value |= 1U << i;
    }
    return value;
}

static uint16_t v90_crc_bit_block(const uint8_t buf[], int first_bit, int last_bit, uint16_t crc)
{
    int pre;
    int post;

    last_bit++;
    pre = first_bit & 0x7;
    first_bit >>= 3;
    if (pre) {
        crc = crc_itu16_bits(buf[first_bit] >> pre, (8 - pre), crc);
        first_bit++;
    }
    post = last_bit & 0x7;
    last_bit >>= 3;
    if ((last_bit - first_bit) != 0)
        crc = crc_itu16_calc(buf + first_bit, last_bit - first_bit, crc);
    if (post)
        crc = crc_itu16_bits(buf[last_bit], post, crc);
    return crc;
}

static bool v90_info_fill_and_sync_ok(const uint8_t *bits, int expected_bits)
{
    return bits
        && expected_bits >= 12
        && v90_bits_get(bits, 0, 12) == V90_INFO_FILL_AND_SYNC_BITS;
}

static int v90_codeword_to_ucode(v90_law_t law, uint8_t codeword)
{
    int ucode;

    if (law == V90_LAW_ULAW)
        return (0xFF - codeword) & 0x7F;

    for (ucode = 0; ucode < 128; ucode++) {
        if (v90_ucode_to_alaw[ucode] == codeword
            || (v90_ucode_to_alaw[ucode] & 0x7F) == (codeword & 0x7F)) {
            return ucode;
        }
    }
    return -1;
}

static uint8_t v90_encode_octet_to_codeword(v90_state_t *s, uint8_t in_octet)
{
    uint8_t sc;
    uint8_t mag;
    int s_bit;
    int sign;
    uint8_t pcm;

    sc = v90_scramble_byte(&s->scrambler, in_octet);
    mag = sc & 0x7F;
    s_bit = (sc >> 7) & 1;

    sign = s_bit ^ s->prev_sign;
    s->prev_sign = sign;

    pcm = ucode_to_pcm_positive(s->law, mag);
    if (sign == 0)
        pcm &= 0x7F;
    return pcm;
}

static bool v90_decode_codeword_to_octet(v90_state_t *s, uint8_t codeword, uint8_t *out_octet)
{
    int sign;
    int scrambled_sign;
    int mag;
    uint8_t scrambled_octet;
    uint8_t plain_octet;
    int bit_idx;

    if (!out_octet)
        return false;

    sign = (codeword & 0x80) ? 1 : 0;
    scrambled_sign = sign ^ s->rx_prev_sign;
    s->rx_prev_sign = sign;

    mag = v90_codeword_to_ucode(s->law, codeword);
    if (mag < 0)
        return false;

    scrambled_octet = (uint8_t) ((mag & 0x7F) | ((scrambled_sign & 1) << 7));
    plain_octet = 0;
    for (bit_idx = 0; bit_idx < 8; bit_idx++) {
        int in_bit;
        int out_bit;

        in_bit = (scrambled_octet >> bit_idx) & 1;
        out_bit = v90_descramble_reg_bit(&s->rx_scramble_reg, in_bit);
        plain_octet |= (uint8_t) (out_bit << bit_idx);
    }
    *out_octet = plain_octet;
    return true;
}

void v90_info0a_init(v90_info0a_t *info)
{
    if (!info)
        return;
    memset(info, 0, sizeof(*info));
    info->support_2743 = true;
    info->support_2800 = true;
    info->support_3429 = true;
    info->support_3000_low = true;
    info->support_3000_high = true;
    info->support_3200_low = true;
    info->support_3200_high = true;
    info->rate_3429_allowed = true;
    info->support_power_reduction = true;
    info->max_baud_rate_difference = 0;
    info->from_cme_modem = false;
    info->support_1664_point_constellation = true;
    info->tx_clock_source = 0;
    info->acknowledge_info0d = false;
}

void v90_info1a_init(v90_info1a_t *info)
{
    if (!info)
        return;
    memset(info, 0, sizeof(*info));
    info->md = 0;
    info->u_info = 78;
    info->upstream_symbol_rate_code = 4;
    info->downstream_rate_code = 6;
    info->freq_offset = 0;
}

bool v90_info0a_validate(const v90_info0a_t *info)
{
    if (!info)
        return false;
    return info->max_baud_rate_difference <= 7
        && info->tx_clock_source <= 3;
}

bool v90_info1a_validate(const v90_info1a_t *info)
{
    if (!info)
        return false;
    return info->md <= 0x7F
        && info->u_info <= 0x7F
        && info->upstream_symbol_rate_code <= 0x7
        && info->downstream_rate_code <= 0x7
        && info->freq_offset >= -512
        && info->freq_offset <= 511;
}

bool v90_build_info0a_bits(uint8_t *buf, int buf_len, const v90_info0a_t *info)
{
    int bit_pos;
    uint16_t crc;

    if (!buf || !v90_info0a_validate(info) || buf_len < ((V90_INFO0A_BITS + 7) / 8))
        return false;
    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    v90_bits_put(buf, &bit_pos, V90_INFO_FILL_AND_SYNC_BITS, 12);
    v90_bits_put(buf, &bit_pos, info->support_2743 ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_2800 ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3429 ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3000_low ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3000_high ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3200_low ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_3200_high ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->rate_3429_allowed ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_power_reduction ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->max_baud_rate_difference & 0x7U, 3);
    v90_bits_put(buf, &bit_pos, info->from_cme_modem ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->support_1664_point_constellation ? 1U : 0U, 1);
    v90_bits_put(buf, &bit_pos, info->tx_clock_source & 0x3U, 2);
    v90_bits_put(buf, &bit_pos, info->acknowledge_info0d ? 1U : 0U, 1);
    crc = v90_crc_bit_block(buf, 12, 28, 0xFFFF);
    v90_bits_put(buf, &bit_pos, crc, 16);
    v90_bits_put(buf, &bit_pos, 0xFU, 4);
    return bit_pos == V90_INFO0A_BITS;
}

bool v90_build_info1a_bits(uint8_t *buf, int buf_len, const v90_info1a_t *info)
{
    int bit_pos;
    uint16_t crc;
    uint16_t freq_bits;

    if (!buf || !v90_info1a_validate(info) || buf_len < ((V90_INFO1A_BITS + 7) / 8))
        return false;
    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    v90_bits_put(buf, &bit_pos, V90_INFO_FILL_AND_SYNC_BITS, 12);
    v90_bits_put(buf, &bit_pos, 0, 6);
    v90_bits_put(buf, &bit_pos, info->md & 0x7FU, 7);
    v90_bits_put(buf, &bit_pos, info->u_info & 0x7FU, 7);
    v90_bits_put(buf, &bit_pos, 0, 2);
    v90_bits_put(buf, &bit_pos, info->upstream_symbol_rate_code & 0x7U, 3);
    v90_bits_put(buf, &bit_pos, info->downstream_rate_code & 0x7U, 3);
    freq_bits = (uint16_t) info->freq_offset;
    if (info->freq_offset < 0)
        freq_bits = (uint16_t) (0x400 + info->freq_offset);
    v90_bits_put(buf, &bit_pos, freq_bits & 0x3FFU, 10);
    crc = v90_crc_bit_block(buf, 12, 49, 0xFFFF);
    v90_bits_put(buf, &bit_pos, crc, 16);
    v90_bits_put(buf, &bit_pos, 0xFU, 4);
    return bit_pos == V90_INFO1A_BITS;
}

bool v90_parse_info0a_bits(v90_info0a_t *out, const uint8_t *bits, int bit_len)
{
    v90_info0a_t parsed;
    uint16_t crc_field;
    uint16_t crc_remainder;

    if (!out || !bits || bit_len < V90_INFO0A_BITS)
        return false;
    if (!v90_info_fill_and_sync_ok(bits, bit_len))
        return false;
    if (v90_bits_get(bits, 45, 4) != 0xF)
        return false;

    memset(&parsed, 0, sizeof(parsed));
    parsed.support_2743 = v90_bits_get(bits, 12, 1) != 0;
    parsed.support_2800 = v90_bits_get(bits, 13, 1) != 0;
    parsed.support_3429 = v90_bits_get(bits, 14, 1) != 0;
    parsed.support_3000_low = v90_bits_get(bits, 15, 1) != 0;
    parsed.support_3000_high = v90_bits_get(bits, 16, 1) != 0;
    parsed.support_3200_low = v90_bits_get(bits, 17, 1) != 0;
    parsed.support_3200_high = v90_bits_get(bits, 18, 1) != 0;
    parsed.rate_3429_allowed = v90_bits_get(bits, 19, 1) != 0;
    parsed.support_power_reduction = v90_bits_get(bits, 20, 1) != 0;
    parsed.max_baud_rate_difference = (uint8_t) v90_bits_get(bits, 21, 3);
    parsed.from_cme_modem = v90_bits_get(bits, 24, 1) != 0;
    parsed.support_1664_point_constellation = v90_bits_get(bits, 25, 1) != 0;
    parsed.tx_clock_source = (uint8_t) v90_bits_get(bits, 26, 2);
    parsed.acknowledge_info0d = v90_bits_get(bits, 28, 1) != 0;

    if (!v90_info0a_validate(&parsed))
        return false;
    crc_field = (uint16_t) v90_bits_get(bits, 29, 16);
    crc_remainder = v90_crc_bit_block(bits, 12, 28, 0xFFFF);
    if (crc_field != crc_remainder)
        return false;

    *out = parsed;
    return true;
}

bool v90_parse_info1a_bits(v90_info1a_t *out, const uint8_t *bits, int bit_len)
{
    v90_info1a_t parsed;
    int raw_freq;
    uint16_t crc_field;
    uint16_t crc_remainder;

    if (!out || !bits || bit_len < V90_INFO1A_BITS)
        return false;
    if (!v90_info_fill_and_sync_ok(bits, bit_len))
        return false;
    if (v90_bits_get(bits, 66, 4) != 0xF)
        return false;
    if (v90_bits_get(bits, 12, 6) != 0 || v90_bits_get(bits, 32, 2) != 0)
        return false;

    memset(&parsed, 0, sizeof(parsed));
    parsed.md = (uint8_t) v90_bits_get(bits, 18, 7);
    parsed.u_info = (uint8_t) v90_bits_get(bits, 25, 7);
    parsed.upstream_symbol_rate_code = (uint8_t) v90_bits_get(bits, 34, 3);
    parsed.downstream_rate_code = (uint8_t) v90_bits_get(bits, 37, 3);
    raw_freq = v90_bits_get(bits, 40, 10);
    if (raw_freq & 0x200)
        raw_freq -= 0x400;
    parsed.freq_offset = (int16_t) raw_freq;

    if (!v90_info1a_validate(&parsed))
        return false;
    crc_field = (uint16_t) v90_bits_get(bits, 50, 16);
    crc_remainder = v90_crc_bit_block(bits, 12, 49, 0xFFFF);
    if (crc_field != crc_remainder)
        return false;

    *out = parsed;
    return true;
}

bool v90_info0a_build_diag(const v90_info0a_t *info, v90_info0a_diag_t *diag)
{
    int i;
    uint8_t packed[(V90_INFO0A_BITS + 7) / 8];

    if (!diag || !v90_build_info0a_bits(packed, (int) sizeof(packed), info))
        return false;
    memset(diag, 0, sizeof(*diag));
    diag->frame = *info;
    for (i = 0; i < V90_INFO0A_BITS; i++)
        diag->bits[i] = (uint8_t) ((packed[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(packed, 29, 16);
    diag->crc_remainder = v90_crc_bit_block(packed, 12, 28, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(packed, V90_INFO0A_BITS)
                          && v90_bits_get(packed, 45, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    return true;
}

bool v90_info1a_build_diag(const v90_info1a_t *info, v90_info1a_diag_t *diag)
{
    int i;
    uint8_t packed[(V90_INFO1A_BITS + 7) / 8];

    if (!diag || !v90_build_info1a_bits(packed, (int) sizeof(packed), info))
        return false;
    memset(diag, 0, sizeof(*diag));
    diag->frame = *info;
    for (i = 0; i < V90_INFO1A_BITS; i++)
        diag->bits[i] = (uint8_t) ((packed[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(packed, 50, 16);
    diag->crc_remainder = v90_crc_bit_block(packed, 12, 49, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(packed, V90_INFO1A_BITS)
                          && v90_bits_get(packed, 66, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    return true;
}

bool v90_info0a_decode_diag(const uint8_t *bits, int bit_len, v90_info0a_diag_t *diag)
{
    int i;
    v90_info0a_t parsed;

    if (!diag || !bits || bit_len < V90_INFO0A_BITS)
        return false;
    memset(diag, 0, sizeof(*diag));
    for (i = 0; i < V90_INFO0A_BITS; i++)
        diag->bits[i] = (uint8_t) ((bits[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(bits, 29, 16);
    diag->crc_remainder = v90_crc_bit_block(bits, 12, 28, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(bits, bit_len)
                          && v90_bits_get(bits, 45, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    if (!diag->valid || !v90_parse_info0a_bits(&parsed, bits, bit_len))
        return false;
    diag->frame = parsed;
    return true;
}

bool v90_info1a_decode_diag(const uint8_t *bits, int bit_len, v90_info1a_diag_t *diag)
{
    int i;
    v90_info1a_t parsed;

    if (!diag || !bits || bit_len < V90_INFO1A_BITS)
        return false;
    memset(diag, 0, sizeof(*diag));
    for (i = 0; i < V90_INFO1A_BITS; i++)
        diag->bits[i] = (uint8_t) ((bits[i >> 3] >> (i & 7)) & 1U);
    diag->crc_field = (uint16_t) v90_bits_get(bits, 50, 16);
    diag->crc_remainder = v90_crc_bit_block(bits, 12, 49, 0xFFFF);
    diag->fill_and_sync_ok = v90_info_fill_and_sync_ok(bits, bit_len)
                          && v90_bits_get(bits, 66, 4) == 0xF;
    diag->valid = diag->fill_and_sync_ok && diag->crc_field == diag->crc_remainder;
    if (!diag->valid || !v90_parse_info1a_bits(&parsed, bits, bit_len))
        return false;
    diag->frame = parsed;
    return true;
}

/* ---- Jd frame construction (Table 13) ---- */

static void v90_build_jd(v90_state_t *s)
{
    /* Build the 72-bit Jd frame per V.90 Table 13.
     * Bits 0:16   = Frame sync (17 ones)
     * Bit  17     = Start bit (0)
     * Bits 18:33  = Data signalling rate capability mask
     * Bit  34     = Start bit (0)
     * Bits 35:46  = Rate capability mask continued + reserved
     * Bit  47     = Constellation size for training (0=4pt, 1=16pt)
     * Bit  48     = Constellation size for renegotiation
     * Bits 49:50  = Spectral shaping lookahead (1-3)
     * Bit  51     = Start bit (0)
     * Bits 52:67  = CRC
     * Bits 68:71  = Fill (0000)
     */
    memset(s->jd_bits, 0, sizeof(s->jd_bits));

    /* We pack bit-by-bit, LSB first within each byte */
    int pos = 0;

    /* Bits 0:16 — 17 sync bits (all 1) */
    for (int i = 0; i < 17; i++)
        s->jd_bits[pos/8] |= (1 << (pos%8)), pos++;

    /* Bit 17 — start bit (0) */
    pos++;

    /* Bits 18:33 — data signalling rate capability mask.
     * Support 28-56 kbps (bits 18:28 = rates 28k through 56k).
     * Bit 18:28 = 000 (28k disabled) through 56k.
     * For now, enable all rates 28k-56k (bits 18:40).
     * Actually the mask is: bit N = rate (N-18+20)*8000/6 / 1000
     * Let's enable rates corresponding to common V.90 speeds.
     * Bit 18 = 28000, bit 19 = 29333, ..., bit 33 = 48000 (first group)
     * Simple approach: enable all. */
    for (int i = 18; i <= 33; i++)
        s->jd_bits[pos/8] |= (1 << (pos%8)), pos++;

    /* Bit 34 — start bit (0) */
    pos++;

    /* Bits 35:46 — continued rate mask + reserved.
     * Bits 35:40 = rates 49333-56000. Enable all.
     * Bits 41:46 = reserved (0) */
    for (int i = 35; i <= 40; i++)
        s->jd_bits[pos/8] |= (1 << (pos%8)), pos++;
    pos += 6; /* bits 41:46 reserved = 0 */

    /* Bit 47 — constellation size for training: 0=4-point */
    pos++;

    /* Bit 48 — constellation size for renegotiation: 0=4-point */
    pos++;

    /* Bits 49:50 — spectral shaping lookahead: 1 (minimum mandatory) */
    s->jd_bits[pos/8] |= (1 << (pos%8)), pos++;
    pos++; /* bit 50 = 0 → value is 1 */

    /* Bit 51 — start bit (0) */
    pos++;

    /* Bits 52:67 — CRC (16 bits). Computed over bits 0:51.
     * Using the CRC generator from V.34 §10.1.2.3.2. */
    {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < 52; i++) {
            int bit = (s->jd_bits[i/8] >> (i%8)) & 1;
            int fb = ((crc >> 15) ^ bit) & 1;
            crc <<= 1;
            if (fb)
                crc ^= 0x8005;  /* CRC-16 polynomial */
            crc &= 0xFFFF;
        }
        for (int i = 0; i < 16; i++) {
            if ((crc >> (15 - i)) & 1)
                s->jd_bits[(52+i)/8] |= (1 << ((52+i)%8));
        }
    }

    /* Bits 68:71 — fill (0000), already zero */
}

/* ---- Phase 3 TX sample generation ---- */

/* Get next Jd bit, wrap around for continuous repetition */
static int v90_get_jd_bit(v90_state_t *s)
{
    int bit = (s->jd_bits[s->jd_bit_pos / 8] >> (s->jd_bit_pos % 8)) & 1;
    s->jd_bit_pos++;
    if (s->jd_bit_pos >= V90_JD_BITS)
        s->jd_bit_pos = 0;
    return bit;
}

static int v90_get_packed_bit(const uint8_t *bits, int bit_pos)
{
    return (bits[bit_pos / 8] >> (bit_pos % 8)) & 1;
}

static uint32_t v90_get_packed_bits(const uint8_t *bits, int bit_pos, int bit_count)
{
    uint32_t value = 0;

    for (int i = 0; i < bit_count; i++)
        value |= (uint32_t)(v90_get_packed_bit(bits, bit_pos + i) << i);
    return value;
}

static bool v90_expect_zero_bit(const uint8_t *bits, int bit_len, int bit_pos)
{
    return bit_pos < bit_len && v90_get_packed_bit(bits, bit_pos) == 0;
}

static bool v90_expect_zero_range(const uint8_t *bits, int bit_len, int bit_pos, int bit_count)
{
    if (bit_pos + bit_count > bit_len)
        return false;
    for (int i = 0; i < bit_count; i++) {
        if (v90_get_packed_bit(bits, bit_pos + i) != 0)
            return false;
    }
    return true;
}

static bool v90_copy_framed_pattern(uint8_t *out,
                                    int out_len,
                                    const uint8_t *bits,
                                    int bit_len,
                                    int start_bit_pos)
{
    int pos = start_bit_pos;
    int copied = 0;

    while (copied < out_len) {
        int chunk = out_len - copied;
        if (chunk > 16)
            chunk = 16;
        if (!v90_expect_zero_bit(bits, bit_len, pos))
            return false;
        pos++;
        if (pos + chunk > bit_len)
            return false;
        for (int i = 0; i < chunk; i++)
            out[copied + i] = (uint8_t)v90_get_packed_bit(bits, pos + i);
        pos += chunk;
        copied += chunk;
    }
    return true;
}

static bool v90_parse_table12_byte_pairs(uint8_t *out,
                                         int out_count,
                                         const uint8_t *bits,
                                         int bit_len,
                                         int start_bit_pos)
{
    int pos = start_bit_pos;
    int index = 0;

    while (index < out_count) {
        if (!v90_expect_zero_bit(bits, bit_len, pos))
            return false;
        pos++;
        if (pos + 8 > bit_len)
            return false;
        out[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
        pos += 7;
        if (index < out_count) {
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
            if (pos + 8 > bit_len)
                return false;
            out[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
            pos += 7;
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
        } else {
            if (!v90_expect_zero_range(bits, bit_len, pos, 9))
                return false;
            pos += 9;
        }
    }

    return true;
}

static bool v90_parse_table12_training_ucodes(v90_dil_desc_t *out,
                                              const uint8_t *bits,
                                              int bit_len,
                                              int start_bit_pos)
{
    int pos = start_bit_pos;
    int index = 0;

    while (index < out->n) {
        if (!v90_expect_zero_bit(bits, bit_len, pos))
            return false;
        pos++;
        if (pos + 8 > bit_len)
            return false;
        out->train_u[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
        pos += 7;
        if (index < out->n) {
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
            if (pos + 8 > bit_len)
                return false;
            out->train_u[index++] = (uint8_t)v90_get_packed_bits(bits, pos, 7);
            pos += 7;
            if (!v90_expect_zero_bit(bits, bit_len, pos))
                return false;
            pos++;
        } else {
            if (!v90_expect_zero_range(bits, bit_len, pos, 9))
                return false;
            pos += 9;
        }
    }

    if (!v90_expect_zero_bit(bits, bit_len, pos))
        return false;
    pos++;
    if (pos + 16 > bit_len)
        return false;

    return true;
}

static uint16_t v90_crc16_bits(const uint8_t *bits, int bit_count)
{
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < bit_count; i++) {
        int bit = v90_get_packed_bit(bits, i);
        int fb = ((crc >> 15) ^ bit) & 1;
        crc <<= 1;
        if (fb)
            crc ^= 0x8005;
        crc &= 0xFFFF;
    }
    return crc;
}

static void v90_put_zero_range(uint8_t *buf, int *bit_pos, int count)
{
    if (!buf || !bit_pos || count <= 0)
        return;
    *bit_pos += count;
}

static void v90_put_framed_pattern(uint8_t *buf, int *bit_pos, const uint8_t *pattern, int pattern_len)
{
    int copied;

    if (!buf || !bit_pos || !pattern || pattern_len < 0)
        return;

    copied = 0;
    while (copied < pattern_len) {
        int chunk;

        chunk = pattern_len - copied;
        if (chunk > 16)
            chunk = 16;
        v90_bits_put(buf, bit_pos, 0, 1);
        for (int i = 0; i < chunk; i++)
            v90_bits_put(buf, bit_pos, pattern[copied + i] ? 1U : 0U, 1);
        v90_put_zero_range(buf, bit_pos, 16 - chunk);
        copied += chunk;
    }
}

static void v90_put_framed_byte_pairs(uint8_t *buf, int *bit_pos, const uint8_t *values, int value_count)
{
    int index;

    if (!buf || !bit_pos || !values || value_count < 0)
        return;

    index = 0;
    while (index < value_count) {
        v90_bits_put(buf, bit_pos, 0, 1);
        v90_bits_put(buf, bit_pos, values[index++] & 0x7FU, 7);
        if (index < value_count) {
            v90_bits_put(buf, bit_pos, 0, 1);
            v90_bits_put(buf, bit_pos, values[index++] & 0x7FU, 7);
            v90_bits_put(buf, bit_pos, 0, 1);
        } else {
            v90_put_zero_range(buf, bit_pos, 9);
        }
    }
}

static void v90_put_framed_training_ucodes(uint8_t *buf, int *bit_pos, const v90_dil_desc_t *desc)
{
    int index;

    if (!buf || !bit_pos || !desc)
        return;

    index = 0;
    while (index < desc->n) {
        int have_second;

        v90_bits_put(buf, bit_pos, 0, 1);
        v90_bits_put(buf, bit_pos, desc->train_u[index++] & 0x7FU, 7);
        have_second = (index < desc->n);
        if (have_second) {
            v90_bits_put(buf, bit_pos, 0, 1);
            v90_bits_put(buf, bit_pos, desc->train_u[index++] & 0x7FU, 7);
            v90_bits_put(buf, bit_pos, 0, 1);
        } else {
            v90_put_zero_range(buf, bit_pos, 9);
        }
    }
}

static inline int v90_clamp_positive(int v, int max_v)
{
    if (v < 1)
        return 1;
    if (v > max_v)
        return max_v;
    return v;
}

static inline int v90_dil_uchord_index(int training_ucode)
{
    int idx = (training_ucode >> 4);
    if (idx < 0)
        idx = 0;
    if (idx > 7)
        idx = 7;
    return idx;
}

static uint8_t v90_count_distinct_train_u(const v90_dil_desc_t *desc)
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

static void v90_dil_reset_tx(v90_state_t *s)
{
    s->dil_segment_index = 0;
    s->dil_pos_in_segment = 0;
    s->dil_terminate_requested = false;
}

static int16_t v90_dil_sample(v90_state_t *s)
{
    int seg_idx;
    int n;
    int training_ucode;
    int uchord_idx;
    int lsp;
    int ltp;
    int seg_len;
    int pos;
    int sp_bit;
    int tp_bit;
    int ucode;

    n = s->dil.n;
    if (n <= 0) {
        s->tx_phase = V90_TX_RI;
        s->sample_count = 0;
        s->phase4_hold_logged = false;
        return v90_pcm_to_linear(s->law, v90_pcm_idle(s->law));
    }

    seg_idx = s->dil_segment_index % n;
    training_ucode = s->dil.train_u[seg_idx] & 0x7F;
    uchord_idx = v90_dil_uchord_index(training_ucode);
    lsp = v90_clamp_positive(s->dil.lsp, V90_DIL_MAX_PAT_BITS);
    ltp = v90_clamp_positive(s->dil.ltp, V90_DIL_MAX_PAT_BITS);
    seg_len = (int)(s->dil.h[uchord_idx] + 1) * 6;
    pos = s->dil_pos_in_segment;

    sp_bit = s->dil.sp[pos % lsp] ? 1 : 0;
    tp_bit = s->dil.tp[pos % ltp] ? 1 : 0;
    ucode = tp_bit ? training_ucode : (s->dil.ref[uchord_idx] & 0x7F);

    s->sample_count++;
    s->dil_pos_in_segment++;
    if (s->dil_pos_in_segment >= seg_len) {
        s->dil_pos_in_segment = 0;
        s->dil_segment_index++;
        if (s->dil_terminate_requested) {
            fprintf(stderr, "[V90] Phase 3: DIL termination requested, completed segment %d/%d and entering Phase 4\n",
                    seg_idx + 1, n);
            s->tx_phase = V90_TX_RI;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
        } else if ((s->dil_segment_index % n) == 0) {
            fprintf(stderr, "[V90] Phase 3: completed one full DIL cycle (%d segments), repeating\n", n);
        }
    }

    return v90_pcm_signed(s->law, ucode, sp_bit);
}

bool v90_parse_dil_descriptor(v90_dil_desc_t *out, const uint8_t *bits, int bit_len)
{
    int alpha;
    int beta;
    int training_start;
    int training_bits;
    int crc_start;
    int descriptor_bits;
    uint16_t expected_crc;
    uint16_t actual_crc;

    if (!out || !bits || bit_len < 206)
        return false;

    memset(out, 0, sizeof(*out));

    for (int i = 0; i < 17; i++) {
        if (v90_get_packed_bit(bits, i) == 0)
            return false;
    }
    if (!v90_expect_zero_bit(bits, bit_len, 17))
        return false;

    out->n = (uint8_t)v90_get_packed_bits(bits, 18, 8);
    if (!v90_expect_zero_range(bits, bit_len, 26, 8))
        return false;
    if (!v90_expect_zero_bit(bits, bit_len, 34))
        return false;

    out->lsp = (uint8_t)(v90_get_packed_bits(bits, 35, 7) + 1);
    if (!v90_expect_zero_bit(bits, bit_len, 42))
        return false;
    out->ltp = (uint8_t)(v90_get_packed_bits(bits, 43, 7) + 1);
    if (!v90_expect_zero_bit(bits, bit_len, 50))
        return false;

    if (out->n == 0) {
        if (out->lsp != 1 || out->ltp != 1)
            return false;
    }

    alpha = ((int)out->lsp + 15) / 16 * 17;
    beta = alpha + (((int)out->ltp + 15) / 16) * 17;
    training_start = 187 + beta;
    training_bits = (((int)out->n + 1) / 2) * 17;
    crc_start = training_start + training_bits;
    descriptor_bits = crc_start + 18;

    if (bit_len < descriptor_bits)
        return false;

    if (!v90_copy_framed_pattern(out->sp, out->lsp, bits, bit_len, 51))
        return false;
    if (!v90_copy_framed_pattern(out->tp, out->ltp, bits, bit_len, 51 + alpha))
        return false;
    if (!v90_parse_table12_byte_pairs(out->h, 8, bits, bit_len, 51 + beta))
        return false;
    if (!v90_parse_table12_byte_pairs(out->ref, 8, bits, bit_len, 119 + beta))
        return false;
    if (!v90_parse_table12_training_ucodes(out, bits, bit_len, training_start))
        return false;

    expected_crc = (uint16_t)v90_get_packed_bits(bits, crc_start + 1, 16);
    actual_crc = v90_crc16_bits(bits, crc_start);
    if (expected_crc != actual_crc)
        return false;

    if (!v90_expect_zero_bit(bits, bit_len, crc_start + 17))
        return false;
    if ((crc_start + 18) < bit_len && v90_get_packed_bit(bits, crc_start + 18) != 0)
        return false;

    return true;
}

int v90_dil_descriptor_bit_len(const v90_dil_desc_t *desc)
{
    int alpha;
    int beta;
    int training_bits;

    if (!desc)
        return 0;
    if (desc->n == 0 && (desc->lsp != 1 || desc->ltp != 1))
        return 0;
    if (desc->lsp > V90_DIL_MAX_PAT_BITS || desc->ltp > V90_DIL_MAX_PAT_BITS)
        return 0;
    if (desc->lsp < 1 || desc->ltp < 1)
        return 0;

    alpha = ((int) desc->lsp + 15) / 16 * 17;
    beta = ((int) desc->ltp + 15) / 16 * 17;
    training_bits = (((int) desc->n + 1) / 2) * 17;
    return 187 + alpha + beta + training_bits + 18;
}

bool v90_build_dil_descriptor_bits(uint8_t *buf,
                                   int buf_len,
                                   int *bit_len_out,
                                   const v90_dil_desc_t *desc)
{
    int bit_len;
    int bit_pos;
    int crc_start;
    uint16_t crc;

    if (!buf || !desc || buf_len <= 0)
        return false;

    bit_len = v90_dil_descriptor_bit_len(desc);
    if (bit_len <= 0 || buf_len < ((bit_len + 7) / 8))
        return false;

    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    v90_bits_put(buf, &bit_pos, 0x1FFFFU, 17);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_bits_put(buf, &bit_pos, desc->n, 8);
    v90_put_zero_range(buf, &bit_pos, 8);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_bits_put(buf, &bit_pos, (uint32_t) (desc->lsp - 1), 7);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_bits_put(buf, &bit_pos, (uint32_t) (desc->ltp - 1), 7);
    v90_bits_put(buf, &bit_pos, 0, 1);
    v90_put_framed_pattern(buf, &bit_pos, desc->sp, desc->lsp);
    v90_put_framed_pattern(buf, &bit_pos, desc->tp, desc->ltp);
    v90_put_framed_byte_pairs(buf, &bit_pos, desc->h, 8);
    v90_put_framed_byte_pairs(buf, &bit_pos, desc->ref, 8);
    v90_put_framed_training_ucodes(buf, &bit_pos, desc);
    crc_start = bit_pos;
    v90_bits_put(buf, &bit_pos, 0, 1);
    crc = v90_crc16_bits(buf, crc_start);
    v90_bits_put(buf, &bit_pos, crc, 16);
    v90_bits_put(buf, &bit_pos, 0, 1);

    if (bit_pos != bit_len) {
        return false;
    }
    if (bit_len_out)
        *bit_len_out = bit_len;
    return true;
}

bool v90_analyse_dil_descriptor(const v90_dil_desc_t *desc, v90_dil_analysis_t *analysis_out)
{
    v90_dil_analysis_t analysis;
    bool seen_uchord[8];
    int i;

    if (!desc || !analysis_out)
        return false;

    memset(&analysis, 0, sizeof(analysis));
    memset(seen_uchord, 0, sizeof(seen_uchord));
    analysis.n = desc->n;
    analysis.lsp = desc->lsp;
    analysis.ltp = desc->ltp;
    analysis.unique_train_u = v90_count_distinct_train_u(desc);

    for (i = 0; i < 8; i++) {
        if (desc->ref[i] != 0)
            analysis.non_default_refs++;
        if (desc->h[i] != 1)
            analysis.non_default_h++;
    }
    for (i = 0; i < desc->n; i++) {
        int uchord_idx;

        uchord_idx = v90_dil_uchord_index(desc->train_u[i] & 0x7F);
        seen_uchord[uchord_idx] = true;
    }
    for (i = 0; i < 8; i++) {
        if (seen_uchord[i])
            analysis.used_uchords++;
    }

    analysis.looks_default_125x12 = (desc->n == 125
                                     && desc->lsp == 12
                                     && desc->ltp == 12
                                     && analysis.used_uchords >= 6
                                     && analysis.non_default_h == 0);
    analysis.robbed_bit_limited = (desc->n == 125
                                   && desc->lsp == 12
                                   && desc->ltp == 6
                                   && analysis.used_uchords >= 6
                                   && analysis.non_default_h == 0);

    if (desc->n < 125)
        analysis.impairment_score++;
    if (desc->n < 100)
        analysis.impairment_score++;
    if (desc->lsp != 12 || desc->ltp != 12)
        analysis.impairment_score++;
    if (desc->lsp < 12 || desc->ltp < 12)
        analysis.impairment_score++;
    if (analysis.used_uchords < 6)
        analysis.impairment_score++;
    if (analysis.used_uchords < 3)
        analysis.impairment_score++;
    if (analysis.unique_train_u < 32)
        analysis.impairment_score++;
    if (analysis.unique_train_u < 12)
        analysis.impairment_score++;
    if (analysis.non_default_h != 0)
        analysis.impairment_score++;

    analysis.echo_limited = (!analysis.robbed_bit_limited && analysis.impairment_score >= 3);
    if (analysis.robbed_bit_limited) {
        analysis.recommended_downstream_drn = 22;
        analysis.recommended_upstream_drn = 22;
    } else if (analysis.impairment_score >= 5) {
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

/* Generate one Phase 3 TX sample */
static int16_t v90_phase3_sample(v90_state_t *s)
{
    int sign;

    switch (s->tx_phase) {
    case V90_TX_SD:
        /* §8.4.4: Sd = 64 reps of {+W, +0, +W, -W, -0, -W}
         * W = Ucode(16 + U_INFO), 0 = Ucode 0
         * §9.3.1.3: Sent first after receiving analog modem's Ja */
        {
            int w_ucode = 16 + s->u_info;
            int pos_in_pattern = s->sample_count % 6;
            s->sample_count++;

            switch (pos_in_pattern) {
            case 0: return v90_pcm_signed(s->law, w_ucode, 1); /* +W */
            case 1: return v90_pcm_signed(s->law, 0, 1);       /* +0 */
            case 2: return v90_pcm_signed(s->law, w_ucode, 1); /* +W */
            case 3: return v90_pcm_signed(s->law, w_ucode, 0); /* -W */
            case 4: return v90_pcm_signed(s->law, 0, 0);       /* -0 */
            case 5:
                s->rep_count++;
                if (s->rep_count >= V90_SD_REPS) {
                    fprintf(stderr, "[V90] Phase 3: Sd complete (%d reps), starting S̄d\n",
                            s->rep_count);
                    s->tx_phase = V90_TX_SD_BAR;
                    s->sample_count = 0;
                    s->rep_count = 0;
                }
                return v90_pcm_signed(s->law, w_ucode, 0); /* -W */
            }
            break;  /* unreachable */
        }

    case V90_TX_SD_BAR:
        /* §8.4.4: S̄d = 8 reps of {-W, -0, -W, +W, +0, +W} */
        {
            int w_ucode = 16 + s->u_info;
            int pos_in_pattern = s->sample_count % 6;
            s->sample_count++;

            switch (pos_in_pattern) {
            case 0: return v90_pcm_signed(s->law, w_ucode, 0); /* -W */
            case 1: return v90_pcm_signed(s->law, 0, 0);       /* -0 */
            case 2: return v90_pcm_signed(s->law, w_ucode, 0); /* -W */
            case 3: return v90_pcm_signed(s->law, w_ucode, 1); /* +W */
            case 4: return v90_pcm_signed(s->law, 0, 1);       /* +0 */
            case 5:
                s->rep_count++;
                if (s->rep_count >= V90_SD_BAR_REPS) {
                    fprintf(stderr, "[V90] Phase 3: S̄d complete, starting TRN1d\n");
                    s->tx_phase = V90_TX_TRN1D;
                    s->sample_count = 0;
                    /* §8.4.5: scrambler initialized to zero for TRN1d */
                    v90_scrambler_init(&s->scrambler);
                }
                return v90_pcm_signed(s->law, w_ucode, 1); /* +W */
            }
            break;  /* unreachable */
        }

    case V90_TX_TRN1D:
        /* §8.4.5: TRN1d = U_INFO codeword with signs from scrambled ones.
         * Scrambler initialized to zero.
         * §9.3.1.4: ≥2040T, then Jd within 4000ms of starting TRN1d */
        {
            int scrambled = v90_scramble_bit(&s->scrambler, 1);
            sign = scrambled;  /* sign=0 → negative, sign=1 → positive */
            s->sample_count++;
            if (s->sample_count >= V90_TRN1D_LEN) {
                fprintf(stderr, "[V90] Phase 3: TRN1d complete (%d symbols), starting Jd\n",
                        s->sample_count);
                s->tx_phase = V90_TX_JD;
                s->sample_count = 0;
                /* §8.4.2: differential encoder initialized with final symbol of TRN1d */
                s->diff_enc = sign;
                s->jd_bit_pos = 0;
                /* Scrambler continues from TRN1d into Jd (not reinitialized) */
            }
            return v90_pcm_signed(s->law, s->u_info, sign);
        }

    case V90_TX_JD:
        /* §8.4.2: Jd bits are scrambled and differentially encoded,
         * transmitted as sign of U_INFO PCM codeword.
         * §9.3.1.4/§9.3.1.5: Sent after TRN1d and repeated until S is seen.
         * Once S is detected, complete the current Jd repetition and send J'd. */
        {
            int bit = v90_get_jd_bit(s);
            int scrambled = v90_scramble_bit(&s->scrambler, bit);
            /* Differential encoding: sign = scrambled XOR previous sign */
            s->diff_enc ^= scrambled;
            sign = s->diff_enc;
            s->sample_count++;
            if (s->jd_terminate_requested && s->jd_bit_pos == 0) {
                fprintf(stderr, "[V90] Phase 3: S detected, completed current Jd repetition after %d symbols, starting J'd\n",
                        s->sample_count);
                s->tx_phase = V90_TX_JD_PRIME;
                s->sample_count = 0;
            }
            return v90_pcm_signed(s->law, s->u_info, sign);
        }

    case V90_TX_JD_PRIME:
        /* §8.4.3: J'd = 12 scrambled zeros as sign of U_INFO */
        {
            int scrambled = v90_scramble_bit(&s->scrambler, 0);
            s->diff_enc ^= scrambled;
            sign = s->diff_enc;
            s->sample_count++;
            if (s->sample_count >= 12) {
                if (s->dil_requested) {
                    fprintf(stderr, "[V90] Phase 3: J'd complete, entering DIL placeholder state\n");
                    s->tx_phase = V90_TX_DIL;
                } else {
                    fprintf(stderr, "[V90] Phase 3: J'd complete, entering Phase 4\n");
                    s->tx_phase = V90_TX_RI;
                }
                s->sample_count = 0;
                s->phase4_hold_logged = false;
            }
            return v90_pcm_signed(s->law, s->u_info, sign);
        }

    case V90_TX_DIL:
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 3: sending DIL (%d segments, LSP=%u, LTP=%u)\n",
                    s->dil.n, s->dil.lsp, s->dil.ltp);
            s->phase4_hold_logged = true;
        }
        return v90_dil_sample(s);

    case V90_TX_RI:
        /* §9.4.1.1 Ri: retrain init — send idle codewords for V90_RI_SYMBOLS */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: Ri (%d symbols)\n", V90_RI_SYMBOLS);
            s->phase4_hold_logged = true;
        }
        s->sample_count++;
        if (s->sample_count >= V90_RI_SYMBOLS) {
            s->tx_phase = V90_TX_TRN2D;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
        }
        return v90_pcm_to_linear(s->law, v90_pcm_idle(s->law));

    case V90_TX_TRN2D:
        /* §9.4.1.2 TRN2d: scrambled ones at U_INFO until cp_ready is set */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: TRN2d at U_INFO=%d\n", s->u_info);
            s->phase4_hold_logged = true;
        }
        if (s->cp_ready) {
            s->tx_phase = V90_TX_CP;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            s->diff_enc = 0;  /* reset differential encoder for CP */
        }
        sign = v90_scramble_bit(&s->scrambler, 1);
        s->sample_count++;
        return v90_pcm_signed(s->law, s->u_info, sign);

    case V90_TX_CP:
        /* §9.4.1.3 CP: call parameters frame as differentially-sign-encoded
           PCM codewords at U_INFO. Each CP bit is scrambled then XORed into
           the running sign. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: CP (%d bits)\n", s->cp_nbits);
            s->phase4_hold_logged = true;
        }
        if (s->cp_bit_pos >= s->cp_nbits) {
            s->tx_phase = V90_TX_B1D;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            return v90_pcm_to_linear(s->law, v90_pcm_idle(s->law));
        }
        {
            int cp_bit = s->cp_bits[s->cp_bit_pos++] & 1;
            cp_bit = v90_scramble_bit(&s->scrambler, cp_bit);
            s->diff_enc ^= cp_bit;
            return v90_pcm_signed(s->law, s->u_info, s->diff_enc);
        }

    case V90_TX_B1D:
        /* §9.4.1.4 B1d: data-mode entry marker — brief idle burst then data */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: B1d (%d symbols), entering data mode\n", V90_B1D_SYMBOLS);
            s->phase4_hold_logged = true;
            s->training_complete = true;
        }
        s->sample_count++;
        if (s->sample_count >= V90_B1D_SYMBOLS) {
            s->tx_phase = V90_TX_DATA;
            s->sample_count = 0;
        }
        return v90_pcm_to_linear(s->law, v90_pcm_idle(s->law));

    default:
        break;
    }

    return v90_pcm_to_linear(s->law, v90_pcm_idle(s->law));
}

/* ---- Public API ---- */

v90_state_t *v90_init_with_v34(v34_state_t *v34, v90_law_t law)
{
    v90_state_t *s = (v90_state_t *)calloc(1, sizeof(*s));
    if (!s)
        return NULL;

    s->v34 = v34;
    s->law = law;
    s->tx_phase = V90_TX_PHASE2;
    s->u_info = 80;
    s->owns_v34 = false;
    v90_scrambler_init(&s->scrambler);
    s->diff_enc = 0;
    s->prev_sign = 0;
    s->rx_scramble_reg = 0;
    s->rx_prev_sign = 0;
    s->phase4_hold_logged = false;
    s->jd_terminate_requested = false;
    s->training_complete = false;
    s->dil_requested = false;
    s->dil_terminate_requested = false;
    memset(&s->dil, 0, sizeof(s->dil));
    v90_dil_reset_tx(s);

    return s;
}

v90_state_t *v90_init_data_pump(v90_law_t law)
{
    return v90_init_with_v34(NULL, law);
}

v90_state_t *v90_init(int baud_rate,
                      int bit_rate,
                      bool calling_party,
                      v90_law_t law,
                      span_get_bit_func_t get_bit,
                      void *get_bit_user_data,
                      span_put_bit_func_t put_bit,
                      void *put_bit_user_data)
{
    v90_state_t *s = (v90_state_t *)calloc(1, sizeof(*s));
    if (!s)
        return NULL;

    s->law = law;
    s->tx_phase = V90_TX_PHASE2;
    s->u_info = 80;  /* Default U_INFO (safe mid-range value) */
    v90_scrambler_init(&s->scrambler);
    s->diff_enc = 0;
    s->prev_sign = 0;
    s->rx_scramble_reg = 0;
    s->rx_prev_sign = 0;
    s->jd_terminate_requested = false;
    s->training_complete = false;
    s->dil_requested = false;
    s->dil_terminate_requested = false;
    memset(&s->dil, 0, sizeof(s->dil));
    v90_dil_reset_tx(s);

    s->owns_v34 = true;
    s->v34 = v34_init(NULL, baud_rate, bit_rate, calling_party, true,
                       get_bit, get_bit_user_data,
                       put_bit, put_bit_user_data);
    if (!s->v34) {
        free(s);
        return NULL;
    }

    /* Enable V.90 INFO0d frame generation */
    v34_set_v90_mode(s->v34, (law == V90_LAW_ALAW) ? 1 : 0);

    return s;
}

void v90_free(v90_state_t *s)
{
    if (!s)
        return;
    if (s->v34 && s->owns_v34)
        v34_free(s->v34);
    free(s);
}

v34_state_t *v90_get_v34(v90_state_t *s)
{
    return s->v34;
}

v90_tx_phase_t v90_get_tx_phase(v90_state_t *s)
{
    return s->tx_phase;
}

bool v90_phase3_active(v90_state_t *s)
{
    return s->tx_phase >= V90_TX_SD && s->tx_phase <= V90_TX_JD_PRIME;
}

bool v90_using_internal_v34_tx(v90_state_t *s)
{
    return s ? s->use_internal_v34_tx : false;
}

void v90_start_phase3(v90_state_t *s, int u_info)
{
    if (u_info > 0 && u_info < 128)
        s->u_info = u_info;

    fprintf(stderr, "[V90] Starting Phase 3 TX (U_INFO=%d, law=%s)\n",
            s->u_info, s->law == V90_LAW_ALAW ? "A-law" : "u-law");

    /* Build Jd frame (used later after TRN1d) */
    v90_build_jd(s);

    /* V.90 §9.3.1.3: After receiving analog Ja, send Sd first.
     * Sd does not use scrambler or differential encoder.
     * Scrambler is initialized to zero for TRN1d (done at Sd→S̄d→TRN1d transition). */
    s->sample_count = 0;
    s->rep_count = 0;
    s->diff_enc = 0;
    s->jd_bit_pos = 0;
    s->phase4_hold_logged = false;
    s->jd_terminate_requested = false;
    s->training_complete = false;
    s->dil_terminate_requested = false;
    s->use_internal_v34_tx = false;
    s->prev_sign = 0;
    s->rx_scramble_reg = 0;
    s->rx_prev_sign = 0;
    v90_dil_reset_tx(s);

    s->tx_phase = V90_TX_SD;
}

void v90_set_dil_descriptor(v90_state_t *s, const v90_dil_desc_t *desc)
{
    if (!s)
        return;

    memset(&s->dil, 0, sizeof(s->dil));
    s->dil_requested = false;
    s->dil_terminate_requested = false;
    v90_dil_reset_tx(s);

    if (!desc)
        return;

    s->dil.n = desc->n;
    s->dil.lsp = (uint8_t)v90_clamp_positive(desc->lsp, V90_DIL_MAX_PAT_BITS);
    s->dil.ltp = (uint8_t)v90_clamp_positive(desc->ltp, V90_DIL_MAX_PAT_BITS);
    memcpy(s->dil.sp, desc->sp, sizeof(s->dil.sp));
    memcpy(s->dil.tp, desc->tp, sizeof(s->dil.tp));
    memcpy(s->dil.h, desc->h, sizeof(s->dil.h));
    memcpy(s->dil.ref, desc->ref, sizeof(s->dil.ref));
    memcpy(s->dil.train_u, desc->train_u, sizeof(s->dil.train_u));
    s->dil_requested = (s->dil.n > 0);
}

void v90_notify_s_detected(v90_state_t *s)
{
    if (!s)
        return;
    if (s->tx_phase == V90_TX_JD && !s->jd_terminate_requested) {
        fprintf(stderr, "[V90] Phase 3: far-end S detected, terminating Jd at the next frame boundary\n");
        s->jd_terminate_requested = true;
    } else if (s->tx_phase == V90_TX_DIL && !s->dil_terminate_requested) {
        fprintf(stderr, "[V90] Phase 3: far-end S detected during DIL, terminating at the next segment boundary\n");
        s->dil_terminate_requested = true;
    }
}

bool v90_training_complete(v90_state_t *s)
{
    return s ? s->training_complete : false;
}

bool v90_set_phase4_cp(v90_state_t *s, const vpcm_cp_frame_t *cp)
{
    if (!s || !cp)
        return false;
    if (!vpcm_cp_encode_bits(cp, s->cp_bits, &s->cp_nbits))
        return false;
    s->cp_frame = *cp;
    s->cp_bit_pos = 0;
    return true;
}

void v90_notify_cp_ready(v90_state_t *s)
{
    if (s && s->tx_phase == V90_TX_TRN2D)
        s->cp_ready = true;
}

int v90_phase3_tx(v90_state_t *s, int16_t amp[], int len)
{
    for (int i = 0; i < len; i++)
        amp[i] = v90_phase3_sample(s);
    return len;
}

uint8_t v90_idle_codeword(v90_law_t law)
{
    return v90_pcm_idle(law);
}

/*
 * Encode one 6-symbol data frame.
 * Fills pcm_out[0..5] with G.711 codewords.
 */
static void v90_encode_frame(v90_state_t *s, const uint8_t *data_in,
                             uint8_t *pcm_out)
{
    for (int i = 0; i < V90_FRAME_LEN; i++)
        pcm_out[i] = v90_encode_octet_to_codeword(s, data_in[i]);
}

int v90_tx_codewords(v90_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len)
{
    int i;
    int count;

    if (!s || !g711_out || !data_in || g711_max <= 0 || data_len <= 0)
        return 0;

    count = (data_len < g711_max) ? data_len : g711_max;
    for (i = 0; i < count; i++)
        g711_out[i] = v90_encode_octet_to_codeword(s, data_in[i]);
    return count;
}

int v90_rx_codewords(v90_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len)
{
    int i;
    int count;

    if (!s || !data_out || !g711_in || data_max <= 0 || g711_len <= 0)
        return 0;

    count = (g711_len < data_max) ? g711_len : data_max;
    for (i = 0; i < count; i++) {
        if (!v90_decode_codeword_to_octet(s, g711_in[i], &data_out[i]))
            return i;
    }
    return count;
}

int v90_tx_data(v90_state_t *s, int16_t amp[], int len,
                const uint8_t *data_in, int data_len)
{
    int pos = 0;
    int consumed = 0;

    while (pos + V90_FRAME_LEN <= len) {
        if (consumed + V90_FRAME_LEN > data_len) {
            /* Not enough data — fill with idle */
            uint8_t idle = v90_idle_codeword(s->law);
            for (int i = 0; i < V90_FRAME_LEN && pos < len; i++)
                amp[pos++] = v90_pcm_to_linear(s->law, idle);
            continue;
        }

        uint8_t pcm_out[V90_FRAME_LEN];
        v90_encode_frame(s, &data_in[consumed], pcm_out);
        for (int i = 0; i < V90_FRAME_LEN; i++)
            amp[pos++] = v90_pcm_to_linear(s->law, pcm_out[i]);
        consumed += V90_FRAME_LEN;
    }

    return consumed;
}

void v90_tx_idle(v90_state_t *s, int16_t amp[], int len)
{
    uint8_t idle = v90_idle_codeword(s->law);
    int16_t sample = v90_pcm_to_linear(s->law, idle);
    for (int i = 0; i < len; i++)
        amp[i] = sample;
}

logging_state_t *v90_get_logging_state(v90_state_t *s)
{
    return v34_get_logging_state(s->v34);
}
