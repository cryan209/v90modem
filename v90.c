/*
 * v90.c — V.90 digital modem module
 *
 * ITU-T V.90 digital (server) side implementation.
 *
 * Phase 2: Wraps SpanDSP V.34 for INFO0d/INFO1d exchange.
 * Phase 3: Generates PCM codewords (Jd, J'd, Sd, S̄d, TRN1d) directly.
 * Phase 4: V.90-specific CP receive and MP/MP' transmit.
 * Data:    Modulus encoder → PCM codewords at 8 kHz.
 */

#include "v90.h"
#include "vpcm_cp.h"
#include "v92_phase4_decode.h"

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
#define V90_RI_SYMBOLS   192  /* Ri duration: at least 192T (§9.4.1.1) */
#define V90_RI_POST_CP_SYMBOLS 24
#define V90_TRN2D_SYMBOLS 2040
#define V90_B1D_FRAMES    48
#define V90_B1D_SYMBOLS   (V90_B1D_FRAMES * V90_FRAME_LEN)
#define V90_MP_MAX_BITS  256
/* Native V.92 mapped SUVd/CPd queue: a profile CPd (base + modulus +
 * one 64-point constellation set) is ~1.4k bits after frame fill. */
#define V90_V92_TX_QUEUE_BITS 2048

/* V.92 Phase 4 constants (ITU-T V.92 §8.8.5 Table 31) */
/* SUVd: 17 sync + 1 start + 1 id + 13 rsv + 1 silent + 1 ack + 1 start
 *       + 16 CRC + 1 fill = 52 bits → round to next multiple of 6 = 54 */
/* Ed: 2 downstream data frames × 6 symbols/frame = 12 codewords (§8.8.2/V.92) */
#define V90_ED_SYMBOLS   12

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

typedef struct {
    int prev_odd;
    uint8_t prev_t[V90_FRAME_LEN];
    int trellis_state;
    double x1;
    double y1;
    double v1;
    bool pending_valid;
    int pending_ucodes[V90_FRAME_LEN];
    uint8_t pending_signs[V90_FRAME_LEN];
} v90_shaper_state_t;

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
    bool             cp_ready;                  /* TRN2d→CP/SUVd transition armed */
    vpcm_cp_frame_t  cp_frame;                  /* CP frame to transmit */
    uint8_t          cp_bits[VPCM_CP_MAX_BITS]; /* Encoded CP bits (one per byte) */
    int              cp_nbits;                  /* Total encoded CP bits */
    int              cp_bit_pos;                /* Current bit index in cp_bits */
    bool             cp_ack_received;
    bool             e_received;
    bool             b1_received;

    /* V.90 Phase 4 Type-0 MP and CPt-selected modulus/shaping mapper. */
    uint8_t          mp_bits[V90_MP_MAX_BITS];
    int              mp_nbits;
    int              mp_bit_pos;
    bool             mp_acknowledge;
    bool             phase4_mapper_ready;
    int              phase4_k;
    int              phase4_d;
    int              phase4_s;
    int              phase4_sr;
    v90_scrambler_t  phase4_scrambler;
    int              phase4_prev_sign;
    v90_shaper_state_t phase4_shaper;
    uint8_t          phase4_frame[V90_FRAME_LEN];
    int              phase4_frame_pos;

    /* V.90 data-mode CP and negotiated modulus/shaping mapper. */
    vpcm_cp_frame_t  data_cp_frame;
    bool             data_cp_received;
    bool             data_mapper_ready;
    int              data_mapper_k;
    int              data_mapper_d;
    int              data_mapper_s;
    int              data_mapper_sr;
    v90_scrambler_t  data_mapper_scrambler;
    int              data_mapper_prev_sign;
    v90_shaper_state_t data_shaper;
    uint8_t          data_mapper_frame[V90_FRAME_LEN];
    int              data_mapper_frame_pos;
    uint64_t         data_input_bits;
    int              data_input_bit_count;

    /* V.92 Phase 4 state */
    bool             v92_mode;                  /* V.92 Phase 4 enabled */
    uint8_t          suv_bits[V92_SUVD_BITS];   /* Encoded SUVd bit stream (one bit per byte) */
    int              suv_bit_pos;               /* Current bit index in suv_bits */

    /* V.92 native upstream Phase 4 gating (§9.6.1.1), driven by real
     * SUVu/CPu/CPu' receive events instead of the compatibility sequence. */
    bool             v92_native_cpu_rx;
    bool             v92_suvu_received;
    bool             v92_cpu_received;
    bool             v92_remote_ack_received;   /* CPu'/SUVu' ack, or E2u */
    bool             v92_cpd_sent;
    bool             v92_ack_sent;              /* sent >= 1 SUVd' (ack=1) */

    /* V.92 native mapped Phase 4 TX: SUVd/CPd bit queue transmitted through
     * the CPt-negotiated TRN2d mapper, and the CPd upstream profile. */
    uint8_t          v92_tx_bits[V90_V92_TX_QUEUE_BITS];
    int              v92_tx_nbits;
    int              v92_tx_pos;
    uint8_t          v92_upstream_drn;          /* Table 30 drn, 0..19 */
    uint8_t          v92_trellis_select;
    uint16_t         v92_gain_q0_16;

    /* Downstream PCM encoder state (data mode) */
    v90_scrambler_t  data_scrambler;
    int              prev_sign;     /* §5.4.5.1 differential sign coding */
    uint32_t         rx_scramble_reg;
    int              rx_prev_sign;

    bool             owns_v34;      /* true if we allocated v34 (v90_init), false if external */
};

static void v90_reset_data_pump_state(v90_state_t *s)
{
    if (!s)
        return;
    /* Preserve the legacy live mapper's all-ones data scrambler seed while
     * keeping the Phase 3/4 scrambler independent. */
    s->data_scrambler.sr = 0x7FFFFF;
    s->prev_sign = 0;
    s->rx_scramble_reg = 0x7FFFFF;
    s->rx_prev_sign = 0;
}

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

/* Generate a signed G.711 codeword from a Ucode and sign bit.
 * sign=1 → positive, sign=0 → negative. */
static inline uint8_t v90_pcm_signed_codeword(v90_law_t law, int ucode, int sign)
{
    uint8_t pcm = ucode_to_pcm_positive(law, ucode);
    pcm = (uint8_t) ((pcm & 0x7F) | (sign ? 0x80 : 0x00));  /* bit7 = polarity */
    return pcm;
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

/*
 * Build a SUVd bit stream (Table 31/V.92) into a flat byte array (one bit per byte).
 * 54 bits total: 17 sync ones + frame fields + 16-bit CRC + fill to multiple of 6.
 * The ack bit (bit 33) is set when the digital modem has received CPu.
 */
static void v90_build_suvd(uint8_t bits[V92_SUVD_BITS], bool ack)
{
    v92_suvd_frame_t frame = {
        .silent_period_requested = false,
        .acknowledge = ack
    };

    if (!v92_suvd_encode(&frame, bits, V92_SUVD_BITS))
        memset(bits, 0, V92_SUVD_BITS);
}

/*
 * Queue an SUVd sequence for native V.92 TRN2d-mapped transmission.  The
 * Table 31 fill rule extends the frame to the next multiple of 6 symbols,
 * which is one d-bit data frame of the CPt-negotiated mapper.
 */
static bool v90_build_v92_suvd_mapped(v90_state_t *s, bool ack)
{
    v92_suvd_frame_t frame = {
        .silent_period_requested = false,
        .acknowledge = ack
    };

    if (!s || s->phase4_d < 1)
        return false;
    if (!v92_suvd_encode_aligned(&frame, s->phase4_d,
                                 s->v92_tx_bits,
                                 (int)sizeof(s->v92_tx_bits),
                                 &s->v92_tx_nbits))
        return false;
    s->v92_tx_pos = 0;
    if (ack)
        s->v92_ack_sent = true;
    return true;
}

/*
 * Queue a native Table 30 CPd for TRN2d-mapped transmission.  The base part
 * carries the selected upstream rate, trellis, and prefilter gain from the
 * V.92 CPd profile; the modulus parameters and a single robbed-bit-safe
 * constellation set (odd Ucodes, smallest magnitude first) describe the
 * upstream data-mode modulation.  Precoder/prefilter coefficients stay
 * absent until real upstream channel measurements exist.  The acknowledge
 * bit reflects whether a valid CPu has been received.
 */
bool v90_build_v92_cpd_frame(const v90_state_t *s, v92_cpd_frame_t *out)
{
    int points = 0;

    if (!s || !out)
        return false;
    memset(out, 0, sizeof(*out));
    out->modulus_present = true;
    out->constellations_present = true;
    out->selected_upstream_drn = s->v92_upstream_drn;
    out->trellis_select = s->v92_trellis_select;
    out->extend_e2u = false;
    out->acknowledge = s->v92_cpu_received;
    out->gain_q0_16 = s->v92_gain_q0_16;
    for (int ucode = 1; ucode < 128 && points < V92_CPD_MAX_POINTS;
         ucode += 2) {
        int16_t linear = v90_pcm_to_linear(
            s->law, ucode_to_pcm_positive(s->law, ucode));

        if (linear <= 0)
            continue;
        out->points[0][points++] = (uint16_t)linear;
    }
    out->set_sizes[0] = (uint8_t)points;
    for (int i = 0; i < 12; i++)
        out->moduli[i] = (uint8_t)(2 * points > 255 ? 255 : 2 * points);
    return points > 0;
}

static bool v90_build_v92_cpd_native(v90_state_t *s)
{
    v92_cpd_frame_t cpd;

    if (!s || s->phase4_d < 1 || !v90_build_v92_cpd_frame(s, &cpd))
        return false;
    if (!v92_cpd_encode(&cpd, s->phase4_d,
                        s->v92_tx_bits,
                        (int)sizeof(s->v92_tx_bits),
                        &s->v92_tx_nbits))
        return false;
    s->v92_tx_pos = 0;
    return true;
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

    sc = v90_scramble_byte(&s->data_scrambler, in_octet);
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

static int v90_cp_constellation_ucode(const vpcm_cp_frame_t *cp,
                                      int frame_interval,
                                      int label)
{
    int constellation;

    if (!cp || frame_interval < 0 || frame_interval >= V90_FRAME_LEN || label < 0)
        return -1;
    constellation = cp->dfi[frame_interval];
    if (constellation < 0 || constellation >= cp->constellation_count)
        return -1;
    for (int ucode = VPCM_CP_MASK_BITS - 1; ucode >= 0; ucode--) {
        if (!vpcm_cp_mask_get(cp->masks[constellation], ucode))
            continue;
        if (label-- == 0)
            return ucode;
    }
    return -1;
}

static int v90_cp_max_upstream_drn(const vpcm_cp_frame_t *cp)
{
    if (!cp)
        return 0;
    for (int bit = 12; bit >= 0; bit--) {
        if (cp->upstream_rate_mask & (1U << bit))
            return bit + 2;
    }
    return 0;
}

static bool v90_build_mp_type0(v90_state_t *s, bool acknowledge)
{
    uint16_t crc;
    int upstream_drn;
    int pos = 0;

    if (!s || s->phase4_d <= 0)
        return false;
    upstream_drn = v90_cp_max_upstream_drn(&s->cp_frame);
    if (upstream_drn < 2 || upstream_drn > 14)
        return false;

    memset(s->mp_bits, 0, sizeof(s->mp_bits));
    for (int i = 0; i < 17; i++)
        s->mp_bits[pos++] = 1;
    s->mp_bits[pos++] = 0; /* bit 17: start */
    s->mp_bits[pos++] = 0; /* bit 18: Type 0 */
    pos += 5;              /* bits 19:23 reserved */
    for (int i = 0; i < 4; i++)
        s->mp_bits[pos++] = (uint8_t)((upstream_drn >> i) & 1);
    s->mp_bits[pos++] = 0; /* bit 28 reserved */
    s->mp_bits[pos++] = 0; /* bits 29:30, 16-state trellis */
    s->mp_bits[pos++] = 0;
    s->mp_bits[pos++] = 0; /* bit 31, Q=0 */
    s->mp_bits[pos++] = 0; /* bit 32, minimum shaping */
    s->mp_bits[pos++] = acknowledge ? 1 : 0;
    s->mp_bits[pos++] = 0; /* bit 34: start */
    s->mp_bits[pos++] = 0; /* bit 35 reserved */
    for (int i = 0; i < 13; i++)
        s->mp_bits[pos++] = (uint8_t)((s->cp_frame.upstream_rate_mask >> i) & 1);
    s->mp_bits[pos++] = 0; /* bit 49 reserved */
    s->mp_bits[pos++] = 0; /* bit 50 reserved */
    s->mp_bits[pos++] = 0; /* bit 51: start */
    pos += 16;             /* bits 52:67 reserved */
    s->mp_bits[pos++] = 0; /* bit 68: start */

    crc = 0xFFFF;
    for (int i = 0; i < pos; i++)
        crc = crc_itu16_bits(s->mp_bits[i], 1, crc);
    for (int i = 0; i < 16; i++)
        s->mp_bits[pos++] = (uint8_t)((crc >> i) & 1);
    s->mp_bits[pos++] = 0; /* mandatory fill bit 85 */
    while ((pos % s->phase4_d) != 0 && pos < V90_MP_MAX_BITS)
        s->mp_bits[pos++] = 0;
    if (pos > V90_MP_MAX_BITS || (pos % s->phase4_d) != 0)
        return false;
    s->mp_nbits = pos;
    s->mp_bit_pos = 0;
    s->mp_acknowledge = acknowledge;
    return true;
}

static bool v90_configure_phase4_mapper(v90_state_t *s,
                                        const vpcm_cp_frame_t *cp)
{
    uint64_t product = 1;

    if (!s || !cp || !vpcm_cp_validate(cp, NULL, 0)
        || cp->v90_compatibility
        || cp->acknowledge
        || cp->codec_alaw != (s->law == V90_LAW_ALAW)
        || cp->shaping_redundancy > 3
        || (cp->shaping_redundancy != 0 && cp->shaping_lookahead > 1)
        || cp->drn < 4 || cp->drn > 22)
        return false;

    s->phase4_d = cp->drn + 8;
    s->phase4_sr = cp->shaping_redundancy;
    s->phase4_s = V90_FRAME_LEN - s->phase4_sr;
    s->phase4_k = s->phase4_d - s->phase4_s;
    if (s->phase4_k < 6 || s->phase4_k > 24)
        return false;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m;

        if (constellation < 0 || constellation >= cp->constellation_count)
            return false;
        m = vpcm_cp_mask_population(cp->masks[constellation]);
        if (m <= 0)
            return false;
        product *= (uint64_t)m;
    }
    if (product < (1ULL << s->phase4_k))
        return false;

    s->cp_frame = *cp;
    v90_scrambler_init(&s->phase4_scrambler);
    s->phase4_prev_sign = 0;
    memset(&s->phase4_shaper, 0, sizeof(s->phase4_shaper));
    s->phase4_frame_pos = V90_FRAME_LEN;
    s->phase4_mapper_ready = true;
    s->cp_ack_received = false;
    /* Native V.92 follows TRN2d with SUVd/CPd and has no V.90 Type-0 MP.
       Table 23 CPt therefore carries no V.90 upstream-rate mask. */
    if (!(s->v92_mode && s->v92_native_cpu_rx)
        && !v90_build_mp_type0(s, false)) {
        s->phase4_mapper_ready = false;
        return false;
    }
    return true;
}

static void v90_reset_negotiated_data_mapper(v90_state_t *s)
{
    if (!s)
        return;
    v90_scrambler_init(&s->data_mapper_scrambler);
    s->data_mapper_prev_sign = 0;
    memset(&s->data_shaper, 0, sizeof(s->data_shaper));
    s->data_mapper_frame_pos = V90_FRAME_LEN;
    s->data_input_bits = 0;
    s->data_input_bit_count = 0;
}

static bool v90_configure_data_mapper(v90_state_t *s,
                                      const vpcm_cp_frame_t *cp)
{
    uint64_t product = 1;

    if (!s || !cp || !vpcm_cp_validate(cp, NULL, 0)
        || !cp->v90_compatibility
        || cp->acknowledge
        || cp->codec_alaw != (s->law == V90_LAW_ALAW)
        || cp->shaping_redundancy > 3
        || (cp->shaping_redundancy != 0 && cp->shaping_lookahead > 1)
        || cp->drn < 1 || cp->drn > 22)
        return false;

    s->data_mapper_d = cp->drn + 20;
    s->data_mapper_sr = cp->shaping_redundancy;
    s->data_mapper_s = V90_FRAME_LEN - s->data_mapper_sr;
    s->data_mapper_k = s->data_mapper_d - s->data_mapper_s;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m;

        if (constellation < 0 || constellation >= cp->constellation_count)
            return false;
        m = vpcm_cp_mask_population(cp->masks[constellation]);
        if (m <= 0)
            return false;
        product *= (uint64_t)m;
    }
    if (product < (1ULL << s->data_mapper_k))
        return false;

    s->data_cp_frame = *cp;
    s->data_cp_received = true;
    s->data_mapper_ready = true;
    v90_reset_negotiated_data_mapper(s);
    return true;
}

static bool v90_map_scrambled_frame(v90_state_t *s,
                                    const vpcm_cp_frame_t *cp,
                                    int k,
                                    const uint8_t *scrambled,
                                    int *previous_sign,
                                    uint8_t frame[V90_FRAME_LEN])
{
    uint64_t r = 0;
    int sign;

    if (!s || !cp || !scrambled || !previous_sign || !frame
        || k < 0 || k > 56)
        return false;
    for (int i = 0; i < k; i++)
        r |= (uint64_t)scrambled[6 + i] << i;

    sign = *previous_sign;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m = vpcm_cp_mask_population(cp->masks[constellation]);
        int label = (int)(r % (uint64_t)m);
        int ucode;

        r /= (uint64_t)m;
        ucode = v90_cp_constellation_ucode(cp, i, label);
        if (ucode < 0)
            return false;
        sign = (scrambled[i] & 1) ^ sign;
        frame[i] = v90_pcm_signed_codeword(s->law, ucode, sign);
    }
    if (r != 0)
        return false;
    *previous_sign = sign;
    return true;
}

typedef enum {
    V90_PHASE4_INPUT_ONES,
    V90_PHASE4_INPUT_MP,
    V90_PHASE4_INPUT_ZEROS,
    V90_PHASE4_INPUT_V92    /* native V.92 SUVd/CPd bit queue */
} v90_phase4_input_t;

static bool v90_map_shaped_scrambled_frame(
    v90_state_t *s,
    const vpcm_cp_frame_t *cp,
    int shaping_redundancy,
    int sign_bits,
    int modulus_bits,
    const uint8_t *scrambled,
    v90_shaper_state_t *shaper,
    uint8_t frame[V90_FRAME_LEN]);

static bool v90_phase4_fill_frame(v90_state_t *s, v90_phase4_input_t input)
{
    uint8_t scrambled[32];

    if (!s || !s->phase4_mapper_ready || s->phase4_d > (int)sizeof(scrambled))
        return false;
    for (int i = 0; i < s->phase4_d; i++) {
        int bit;

        if (input == V90_PHASE4_INPUT_ONES) {
            bit = 1;
        } else if (input == V90_PHASE4_INPUT_ZEROS) {
            bit = 0;
        } else if (input == V90_PHASE4_INPUT_V92) {
            if (s->v92_tx_pos >= s->v92_tx_nbits)
                return false;
            bit = s->v92_tx_bits[s->v92_tx_pos++] & 1;
        } else {
            if (s->mp_bit_pos >= s->mp_nbits)
                return false;
            bit = s->mp_bits[s->mp_bit_pos++] & 1;
        }
        scrambled[i] = (uint8_t)v90_scramble_bit(&s->phase4_scrambler, bit);
    }
    if (s->phase4_sr == 0) {
        if (!v90_map_scrambled_frame(s,
                                     &s->cp_frame,
                                     s->phase4_k,
                                     scrambled,
                                     &s->phase4_prev_sign,
                                     s->phase4_frame))
            return false;
    } else if (!v90_map_shaped_scrambled_frame(s,
                                                &s->cp_frame,
                                                s->phase4_sr,
                                                s->phase4_s,
                                                s->phase4_k,
                                                scrambled,
                                                &s->phase4_shaper,
                                                s->phase4_frame)) {
        return false;
    }
    s->phase4_frame_pos = 0;
    return true;
}

static uint8_t v90_phase4_codeword(v90_state_t *s, v90_phase4_input_t input)
{
    if (s->phase4_frame_pos >= V90_FRAME_LEN) {
        int attempts = 0;

        while (!v90_phase4_fill_frame(s, input)) {
            attempts++;
            if (s->cp_frame.shaping_lookahead != 1 || attempts >= 2)
                return v90_pcm_idle(s->law);
        }
    }
    return s->phase4_frame[s->phase4_frame_pos++];
}

typedef struct {
    double x1;
    double y1;
    double v1;
    double metric;
} v90_shaper_filter_state_t;

static bool v90_shaper_rule_inverts(int rule, int position)
{
    switch (rule) {
    case 1: return true;                 /* B: all */
    case 2: return (position & 1) == 0;  /* C: even */
    case 3: return (position & 1) != 0;  /* D: odd */
    default: return false;               /* A: none */
    }
}

static v90_shaper_filter_state_t v90_evaluate_shaper_rule(
    v90_state_t *s,
    const vpcm_cp_frame_t *cp,
    const int *ucodes,
    const uint8_t *initial_signs,
    int length,
    int rule,
    v90_shaper_filter_state_t filter)
{
    const double a1 = (double)(int8_t)cp->shaping_a1_q1_6 / 64.0;
    const double a2 = (double)(int8_t)cp->shaping_a2_q1_6 / 64.0;
    const double b1 = (double)(int8_t)cp->shaping_b1_q1_6 / 64.0;
    const double b2 = (double)(int8_t)cp->shaping_b2_q1_6 / 64.0;

    filter.metric = 0.0;
    for (int i = 0; i < length; i++) {
        int sign = initial_signs[i] ^ v90_shaper_rule_inverts(rule, i);
        double x = (double)v90_pcm_to_linear(
            s->law, v90_pcm_signed_codeword(s->law, ucodes[i], sign));
        double y = x - b1 * filter.x1 + a1 * filter.y1;
        double v = y - b2 * filter.y1 + a2 * filter.v1;

        filter.metric += v * v;
        filter.x1 = x;
        filter.y1 = y;
        filter.v1 = v;
    }
    return filter;
}

static void v90_build_initial_shaping_signs(v90_shaper_state_t *shaper,
                                            int shaping_redundancy,
                                            const uint8_t *sign_bits,
                                            uint8_t signs[V90_FRAME_LEN])
{
    int frame_length = V90_FRAME_LEN / shaping_redundancy;
    int sign_pos = 0;

    for (int frame = 0; frame < shaping_redundancy; frame++) {
        uint8_t p[V90_FRAME_LEN] = {0};

        for (int k = 1; k < frame_length; k++)
            p[k] = sign_bits[sign_pos++];
        for (int k = 0; k < frame_length; k++) {
            int p_prime = p[k];
            int t;

            if (k & 1) {
                p_prime ^= shaper->prev_odd;
                shaper->prev_odd = p_prime;
            }
            t = p_prime ^ shaper->prev_t[k];
            shaper->prev_t[k] = (uint8_t)t;
            signs[frame * frame_length + k] = (uint8_t)t;
        }
    }
}

static int v90_select_shaper_rule(v90_state_t *s,
                                  const vpcm_cp_frame_t *cp,
                                  const v90_shaper_state_t *shaper,
                                  const int *ucodes,
                                  const uint8_t *initial,
                                  const int *next_ucodes,
                                  const uint8_t *next_initial,
                                  int frame_length,
                                  v90_shaper_filter_state_t *selected_filter)
{
    v90_shaper_filter_state_t base = {
        shaper->x1,
        shaper->y1,
        shaper->v1,
        0.0
    };
    double best_metric = HUGE_VAL;
    int first_rules[2];
    int best_rule = 0;

    first_rules[0] = (shaper->trellis_state == 0) ? 0 : 2;
    first_rules[1] = (shaper->trellis_state == 0) ? 1 : 3;
    for (int first_idx = 0; first_idx < 2; first_idx++) {
        int first_rule = first_rules[first_idx];
        int next_state = (first_rule == 1 || first_rule == 3) ? 1 : 0;
        v90_shaper_filter_state_t current;
        double metric;

        current = v90_evaluate_shaper_rule(s,
                                           cp,
                                           ucodes,
                                           initial,
                                           frame_length,
                                           first_rule,
                                           base);
        metric = current.metric;
        if (next_ucodes && next_initial) {
            int next_rules[2] = {
                next_state == 0 ? 0 : 2,
                next_state == 0 ? 1 : 3
            };
            double next_best = HUGE_VAL;

            for (int next_idx = 0; next_idx < 2; next_idx++) {
                v90_shaper_filter_state_t preview;

                preview = v90_evaluate_shaper_rule(s,
                                                   cp,
                                                   next_ucodes,
                                                   next_initial,
                                                   frame_length,
                                                   next_rules[next_idx],
                                                   current);
                if (preview.metric < next_best)
                    next_best = preview.metric;
            }
            metric += next_best;
        }
        if (metric < best_metric) {
            best_metric = metric;
            best_rule = first_rule;
            *selected_filter = current;
        }
    }
    return best_rule;
}

static void v90_shape_data_signs(v90_state_t *s,
                                 const vpcm_cp_frame_t *cp,
                                 int shaping_redundancy,
                                 v90_shaper_state_t *shaper,
                                 const int ucodes[V90_FRAME_LEN],
                                 const uint8_t initial[V90_FRAME_LEN],
                                 const int lookahead_ucodes[V90_FRAME_LEN],
                                 const uint8_t lookahead_initial[V90_FRAME_LEN],
                                 uint8_t signs[V90_FRAME_LEN])
{
    int frame_length = V90_FRAME_LEN / shaping_redundancy;

    for (int frame = 0; frame < shaping_redundancy; frame++) {
        int offset = frame * frame_length;
        v90_shaper_filter_state_t selected;
        const int *next_ucodes = NULL;
        const uint8_t *next_initial = NULL;
        int rule;

        if (cp->shaping_lookahead == 1) {
            if (frame + 1 < shaping_redundancy) {
                next_ucodes = ucodes + offset + frame_length;
                next_initial = initial + offset + frame_length;
            } else {
                next_ucodes = lookahead_ucodes;
                next_initial = lookahead_initial;
            }
        }
        rule = v90_select_shaper_rule(s,
                                      cp,
                                      shaper,
                                      ucodes + offset,
                                      initial + offset,
                                      next_ucodes,
                                      next_initial,
                                      frame_length,
                                      &selected);
        for (int k = 0; k < frame_length; k++)
            signs[offset + k] = initial[offset + k]
                              ^ v90_shaper_rule_inverts(rule, k);
        shaper->trellis_state = (rule == 1 || rule == 3) ? 1 : 0;
        shaper->x1 = selected.x1;
        shaper->y1 = selected.y1;
        shaper->v1 = selected.v1;
    }
}

static bool v90_map_shaped_scrambled_frame(
    v90_state_t *s,
    const vpcm_cp_frame_t *cp,
    int shaping_redundancy,
    int sign_bits,
    int modulus_bits,
    const uint8_t *scrambled,
    v90_shaper_state_t *shaper,
    uint8_t frame[V90_FRAME_LEN])
{
    uint8_t initial_signs[V90_FRAME_LEN];
    uint8_t shaped_signs[V90_FRAME_LEN];
    int ucodes[V90_FRAME_LEN];
    uint64_t r = 0;

    if (!s || !cp || !scrambled || !shaper || !frame
        || shaping_redundancy < 1 || shaping_redundancy > 3
        || sign_bits != V90_FRAME_LEN - shaping_redundancy
        || modulus_bits < 0 || modulus_bits > 56)
        return false;
    for (int i = 0; i < modulus_bits; i++)
        r |= (uint64_t)scrambled[sign_bits + i] << i;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int m = vpcm_cp_mask_population(cp->masks[constellation]);
        int label = (int)(r % (uint64_t)m);

        r /= (uint64_t)m;
        ucodes[i] = v90_cp_constellation_ucode(cp, i, label);
        if (ucodes[i] < 0)
            return false;
    }
    if (r != 0)
        return false;
    v90_build_initial_shaping_signs(shaper,
                                    shaping_redundancy,
                                    scrambled,
                                    initial_signs);
    if (cp->shaping_lookahead == 1) {
        if (!shaper->pending_valid) {
            memcpy(shaper->pending_ucodes, ucodes,
                   sizeof(shaper->pending_ucodes));
            memcpy(shaper->pending_signs, initial_signs,
                   sizeof(shaper->pending_signs));
            shaper->pending_valid = true;
            return false;
        }
        v90_shape_data_signs(s,
                             cp,
                             shaping_redundancy,
                             shaper,
                             shaper->pending_ucodes,
                             shaper->pending_signs,
                             ucodes,
                             initial_signs,
                             shaped_signs);
        for (int i = 0; i < V90_FRAME_LEN; i++) {
            frame[i] = v90_pcm_signed_codeword(s->law,
                                                shaper->pending_ucodes[i],
                                                shaped_signs[i]);
        }
        memcpy(shaper->pending_ucodes, ucodes,
               sizeof(shaper->pending_ucodes));
        memcpy(shaper->pending_signs, initial_signs,
               sizeof(shaper->pending_signs));
    } else {
        v90_shape_data_signs(s,
                             cp,
                             shaping_redundancy,
                             shaper,
                             ucodes,
                             initial_signs,
                             NULL,
                             NULL,
                             shaped_signs);
        for (int i = 0; i < V90_FRAME_LEN; i++) {
            frame[i] = v90_pcm_signed_codeword(s->law,
                                                ucodes[i],
                                                shaped_signs[i]);
        }
    }
    return true;
}

static bool v90_data_mapper_fill_frame(v90_state_t *s, uint64_t input_bits)
{
    uint8_t scrambled[48];

    if (!s || !s->data_mapper_ready
        || s->data_mapper_d <= 0
        || s->data_mapper_d > (int)sizeof(scrambled))
        return false;
    for (int i = 0; i < s->data_mapper_d; i++) {
        int bit = (int)((input_bits >> i) & 1U);

        scrambled[i] = (uint8_t)v90_scramble_bit(&s->data_mapper_scrambler, bit);
    }
    if (s->data_mapper_sr == 0) {
        if (!v90_map_scrambled_frame(s,
                                     &s->data_cp_frame,
                                     s->data_mapper_k,
                                     scrambled,
                                     &s->data_mapper_prev_sign,
                                     s->data_mapper_frame))
            return false;
    } else if (!v90_map_shaped_scrambled_frame(s,
                                                &s->data_cp_frame,
                                                s->data_mapper_sr,
                                                s->data_mapper_s,
                                                s->data_mapper_k,
                                                scrambled,
                                                &s->data_shaper,
                                                s->data_mapper_frame)) {
        return false;
    }
    s->data_mapper_frame_pos = 0;
    return true;
}

static uint8_t v90_data_mapper_ones_codeword(v90_state_t *s)
{
    if (s->data_mapper_frame_pos >= V90_FRAME_LEN) {
        uint64_t ones = (1ULL << s->data_mapper_d) - 1ULL;

        int attempts = 0;

        while (!v90_data_mapper_fill_frame(s, ones)) {
            attempts++;
            if (s->data_cp_frame.shaping_lookahead != 1 || attempts >= 2)
                return v90_pcm_idle(s->law);
        }
    }
    return s->data_mapper_frame[s->data_mapper_frame_pos++];
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
        int pad;
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
        pad = 16 - chunk;
        if (pad > 0) {
            if (!v90_expect_zero_range(bits, bit_len, pos, pad))
                return false;
            pos += pad;
        }
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

static uint8_t v90_dil_codeword(v90_state_t *s)
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
        return v90_pcm_idle(s->law);
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

    return v90_pcm_signed_codeword(s->law, ucode, sp_bit);
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
    /*
     * V.90 Table 12 ends after this fill bit, but V.92 Table 20 extends the
     * DIL descriptor with additional capability fields after the base V.90
     * descriptor. Do not require the next bit to be zero here.
     */

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

/* Generate one raw G.711 codeword for the Phase 3/4 transmit sequence. */
static uint8_t v90_phase3_codeword(v90_state_t *s)
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
            case 0: return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
            case 1: return v90_pcm_signed_codeword(s->law, 0, 1);       /* +0 */
            case 2: return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
            case 3: return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
            case 4: return v90_pcm_signed_codeword(s->law, 0, 0);       /* -0 */
            case 5:
                s->rep_count++;
                if (s->rep_count >= V90_SD_REPS) {
                    fprintf(stderr, "[V90] Phase 3: Sd complete (%d reps), starting S̄d\n",
                            s->rep_count);
                    s->tx_phase = V90_TX_SD_BAR;
                    s->sample_count = 0;
                    s->rep_count = 0;
                }
                return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
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
            case 0: return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
            case 1: return v90_pcm_signed_codeword(s->law, 0, 0);       /* -0 */
            case 2: return v90_pcm_signed_codeword(s->law, w_ucode, 0); /* -W */
            case 3: return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
            case 4: return v90_pcm_signed_codeword(s->law, 0, 1);       /* +0 */
            case 5:
                s->rep_count++;
                if (s->rep_count >= V90_SD_BAR_REPS) {
                    fprintf(stderr, "[V90] Phase 3: S̄d complete, starting TRN1d\n");
                    s->tx_phase = V90_TX_TRN1D;
                    s->sample_count = 0;
                    /* §8.4.5: scrambler initialized to zero for TRN1d */
                    v90_scrambler_init(&s->scrambler);
                }
                return v90_pcm_signed_codeword(s->law, w_ucode, 1); /* +W */
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
            return v90_pcm_signed_codeword(s->law, s->u_info, sign);
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
            return v90_pcm_signed_codeword(s->law, s->u_info, sign);
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
            return v90_pcm_signed_codeword(s->law, s->u_info, sign);
        }

    case V90_TX_DIL:
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 3: sending DIL (%d segments, LSP=%u, LTP=%u)\n",
                    s->dil.n, s->dil.lsp, s->dil.ltp);
            s->phase4_hold_logged = true;
        }
        return v90_dil_codeword(s);

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
        return v90_pcm_idle(s->law);

    case V90_TX_TRN2D:
        /* §9.4.1.1/§9.4.1.2: remain in Ri while acquiring CPt.  After
         * accepting CPt, send 24T more Ri followed by at least 2040T of
         * TRN2d using the negotiated six-interval modulus mapper. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: waiting for valid CPt%s\n",
                    s->v92_mode ? " (V.92 compatibility path)" : "");
            s->phase4_hold_logged = true;
        }
        if (!s->cp_ready)
            return v90_pcm_idle(s->law);

        if (s->v92_mode && !s->v92_native_cpu_rx) {
            /* Compatibility path: no negotiated Phase 4 mapper; jump
             * straight to sign-modulated SUVd at U_INFO. */
            v90_build_suvd(s->suv_bits, false);
            s->suv_bit_pos = 0;
            s->tx_phase = V90_TX_SUVD;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            return v90_pcm_idle(s->law);
        }

        if (s->sample_count < V90_RI_POST_CP_SYMBOLS) {
            s->sample_count++;
            return v90_pcm_idle(s->law);
        }
        if (s->sample_count == V90_RI_POST_CP_SYMBOLS) {
            fprintf(stderr,
                    "[V90] Phase 4: CPt accepted; TRN2d (%d mapped symbols, D=%d, K=%d)\n",
                    V90_TRN2D_SYMBOLS, s->phase4_d, s->phase4_k);
        }
        {
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_ONES);

            s->sample_count++;
            if (s->sample_count >= V90_RI_POST_CP_SYMBOLS + V90_TRN2D_SYMBOLS) {
                if (s->v92_mode) {
                    /* §9.6.1.1.1/V.92: TRN2d done; condition for SUVu and
                     * transmit SUVd sequences over the TRN2d mapper. */
                    (void)v90_build_v92_suvd_mapped(s, false);
                    s->tx_phase = V90_TX_SUVD;
                } else {
                    s->tx_phase = V90_TX_MP;
                }
                s->sample_count = 0;
                s->phase4_hold_logged = false;
            }
            return codeword;
        }

    case V90_TX_MP:
        /* §9.4.1.3: repeat MP with acknowledge=0 until data-mode CP is
         * received, then repeat MP' until the analogue modem returns CP'. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: MP%s (%d bits, D=%d)\n",
                    s->mp_acknowledge ? "'" : "",
                    s->mp_nbits, s->phase4_d);
            s->phase4_hold_logged = true;
        }
        {
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_MP);

            if (s->mp_bit_pos >= s->mp_nbits
                && s->phase4_frame_pos >= V90_FRAME_LEN) {
                s->phase4_hold_logged = false;
                if (!s->mp_acknowledge) {
                    if (s->data_cp_received)
                        (void)v90_build_mp_type0(s, true);
                    else
                        (void)v90_build_mp_type0(s, false);
                } else if ((s->cp_ack_received || s->e_received)
                           && s->data_mapper_ready) {
                    s->tx_phase = V90_TX_ED;
                    s->sample_count = 0;
                } else {
                    (void)v90_build_mp_type0(s, true);
                }
            }
            return codeword;
        }

    case V90_TX_SUVD:
        /* V.92 §9.6.1.1: SUVd — Short Update Values (digital→analogue).
         * Native mode transmits over the CPt-negotiated TRN2d mapper;
         * the compatibility path sign-modulates at U_INFO. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4 V.92: SUVd (%s, ack=%d)\n",
                    s->v92_native_cpu_rx ? "TRN2d mapped" : "U_INFO",
                    (s->v92_native_cpu_rx && s->v92_cpu_received) ? 1 : 0);
            s->phase4_hold_logged = true;
        }
        if (s->v92_native_cpu_rx) {
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_V92);

            if (s->v92_tx_pos >= s->v92_tx_nbits
                && s->phase4_frame_pos >= V90_FRAME_LEN) {
                s->phase4_hold_logged = false;
                /* §9.6.1.1.4: after sending an acknowledged sequence and
                 * receiving CPu'/SUVu' (or E2u), move to Ed. */
                if (s->v92_ack_sent && s->v92_remote_ack_received) {
                    s->tx_phase = V90_TX_ED;
                    s->sample_count = 0;
                } else if (!s->v92_cpd_sent
                           && (s->v92_suvu_received || s->v92_cpu_received)) {
                    /* §9.6.1.1.2: a single CPd per received SUVu/CPu. */
                    if (v90_build_v92_cpd_native(s)) {
                        s->tx_phase = V90_TX_CP;
                        s->sample_count = 0;
                    } else {
                        (void)v90_build_v92_suvd_mapped(s,
                                                        s->v92_cpu_received);
                    }
                } else {
                    /* Otherwise repeat SUVd; ack tracks CPu receipt. */
                    (void)v90_build_v92_suvd_mapped(s, s->v92_cpu_received);
                }
            }
            return codeword;
        }
        if (s->suv_bit_pos >= V92_SUVD_BITS) {
            /* Compatibility path: SUVd complete → CPd; diff_enc continues */
            s->tx_phase = V90_TX_CP;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            s->cp_bit_pos = 0;
            return v90_pcm_idle(s->law);
        }
        {
            int suv_bit = s->suv_bits[s->suv_bit_pos++] & 1;
            suv_bit = v90_scramble_bit(&s->scrambler, suv_bit);
            s->diff_enc ^= suv_bit;
            return v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
        }

    case V90_TX_CP:
        /* Legacy V.92 harness payload. This is a V.90-shaped CP frame, not a
         * native Table 30 CPd; native CPd remains gated on real CPu-derived
         * rate, gain, filter, and constellation parameters. */
        if (!s->phase4_hold_logged) {
            if (s->v92_native_cpu_rx)
                fprintf(stderr,
                        "[V90] Phase 4 V.92: native Table 30 CPd (%d bits, TRN2d mapped, ack=%d)\n",
                        s->v92_tx_nbits, s->v92_cpu_received ? 1 : 0);
            else
                fprintf(stderr,
                        "[V90] Phase 4 V.92 compatibility: legacy CP payload (%d bits; not native CPd)\n",
                        s->cp_nbits);
            s->phase4_hold_logged = true;
        }
        if (s->v92_native_cpu_rx) {
            /* Native Table 30 CPd over the TRN2d mapper. */
            uint8_t codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_V92);

            if (s->v92_tx_pos >= s->v92_tx_nbits
                && s->phase4_frame_pos >= V90_FRAME_LEN) {
                /* §9.6.1.1.2: single CPd, then more SUVd sequences whose
                 * ack bit is gated on a real CPu having been received. */
                s->v92_cpd_sent = true;
                s->phase4_hold_logged = false;
                s->sample_count = 0;
                (void)v90_build_v92_suvd_mapped(s, s->v92_cpu_received);
                s->tx_phase = V90_TX_SUVD;
            }
            return codeword;
        }
        if (s->cp_bit_pos >= s->cp_nbits) {
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            /* Compatibility: CPd → SUVd' (with ack bit set). */
            v90_build_suvd(s->suv_bits, true);
            s->suv_bit_pos = 0;
            s->tx_phase = V90_TX_SUVD_ACK;
            return v90_pcm_idle(s->law);
        }
        {
            int cp_bit = s->cp_bits[s->cp_bit_pos++] & 1;
            cp_bit = v90_scramble_bit(&s->scrambler, cp_bit);
            s->diff_enc ^= cp_bit;
            return v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
        }

    case V90_TX_SUVD_ACK:
        /* V.92 §9.6.1.1: SUVd' — Short Update Values with acknowledge bit set.
         * Signals to analogue that digital has received CPu. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4 V.92: SUVd' (%d bits, ack=1)\n", V92_SUVD_BITS);
            s->phase4_hold_logged = true;
        }
        if (s->suv_bit_pos >= V92_SUVD_BITS) {
            /* SUVd' complete → Ed */
            s->tx_phase = V90_TX_ED;
            s->sample_count = 0;
            s->phase4_hold_logged = false;
            return v90_pcm_idle(s->law);
        }
        {
            int suv_bit = s->suv_bits[s->suv_bit_pos++] & 1;
            suv_bit = v90_scramble_bit(&s->scrambler, suv_bit);
            s->diff_enc ^= suv_bit;
            return v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
        }

    case V90_TX_ED:
        /* Ed is two downstream mapping frames of scrambled binary zeros. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4%s: Ed (%d symbols, scrambled zeros)\n",
                    s->v92_mode ? " V.92" : "", V90_ED_SYMBOLS);
            s->phase4_hold_logged = true;
        }
        {
            uint8_t codeword;
            int tx_symbols = V90_ED_SYMBOLS;

            if (s->v92_mode && s->v92_native_cpu_rx) {
                /* §8.8.2/V.92: Ed uses the corresponding TRN2d modulation. */
                if (s->cp_frame.shaping_lookahead == 1)
                    tx_symbols += V90_FRAME_LEN;
                codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_ZEROS);
            } else if (s->v92_mode) {
                int zero_bit = v90_scramble_bit(&s->scrambler, 0);

                s->diff_enc ^= zero_bit;
                codeword = v90_pcm_signed_codeword(s->law, s->u_info, s->diff_enc);
            } else {
                /* ld=1 keeps the final MP frame pending. Feed one additional
                 * zero frame so the wire sees that MP frame followed by both
                 * required Ed frames before switching to the independent B1d
                 * mapper. */
                if (s->cp_frame.shaping_lookahead == 1)
                    tx_symbols += V90_FRAME_LEN;
                codeword = v90_phase4_codeword(s, V90_PHASE4_INPUT_ZEROS);
            }
            s->sample_count++;
            if (s->sample_count >= tx_symbols) {
                s->tx_phase = V90_TX_B1D;
                s->sample_count = 0;
                s->phase4_hold_logged = false;
                if (!s->v92_mode
                    || (s->v92_native_cpu_rx && s->data_mapper_ready))
                    v90_reset_negotiated_data_mapper(s);
            }
            return codeword;
        }

    case V90_TX_B1D:
        /* §8.6.1/§9.4.1.5: 48 complete data frames of scrambled ones,
         * starting from zeroed data-mode scrambler and differential state. */
        if (!s->phase4_hold_logged) {
            fprintf(stderr, "[V90] Phase 4: B1d (%d data frames, %d symbols)\n",
                    V90_B1D_FRAMES, V90_B1D_SYMBOLS);
            s->phase4_hold_logged = true;
        }
        /* §9.6.1.1.5/V.92: in native mode B1d runs at the negotiated rate
         * using the data-mode constellation parameters received in CPu. */
        if (s->data_mapper_ready
            && (!s->v92_mode || s->v92_native_cpu_rx)) {
            uint8_t codeword = v90_data_mapper_ones_codeword(s);

            s->sample_count++;
            if (s->sample_count >= V90_B1D_SYMBOLS) {
                s->tx_phase = V90_TX_DATA;
                s->sample_count = 0;
                s->training_complete = true;
            }
            return codeword;
        }
        /* V.92 compatibility path retains the legacy marker until its
         * separate data-mode mapper is implemented. */
        s->sample_count++;
        if (s->sample_count >= V90_B1D_SYMBOLS) {
            s->tx_phase = V90_TX_DATA;
            s->sample_count = 0;
            v90_reset_data_pump_state(s);
            s->training_complete = true;
        }
        return v90_pcm_idle(s->law);

    default:
        break;
    }

    return v90_pcm_idle(s->law);
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
    v90_reset_data_pump_state(s);
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
    v90_reset_data_pump_state(s);
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
    s->cp_ready = false;
    s->cp_ack_received = false;
    s->e_received = false;
    s->b1_received = false;
    s->phase4_mapper_ready = false;
    s->phase4_k = 0;
    s->phase4_d = 0;
    s->phase4_s = 0;
    s->phase4_sr = 0;
    memset(&s->phase4_shaper, 0, sizeof(s->phase4_shaper));
    s->phase4_frame_pos = V90_FRAME_LEN;
    s->mp_nbits = 0;
    s->mp_bit_pos = 0;
    s->data_cp_received = false;
    s->data_mapper_ready = false;
    s->data_mapper_k = 0;
    s->data_mapper_d = 0;
    s->data_mapper_frame_pos = V90_FRAME_LEN;
    s->data_input_bits = 0;
    s->data_input_bit_count = 0;
    s->v92_suvu_received = false;
    s->v92_cpu_received = false;
    s->v92_remote_ack_received = false;
    s->v92_cpd_sent = false;
    s->v92_ack_sent = false;
    s->v92_tx_nbits = 0;
    s->v92_tx_pos = 0;
    v90_reset_data_pump_state(s);
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

const char *v90_rx_event_name(v90_rx_event_t event)
{
    switch (event) {
    case V90_RX_EVENT_NONE:           return "NONE";
    case V90_RX_EVENT_INFO1A_VALID:   return "INFO1A_VALID";
    case V90_RX_EVENT_INFO1A_INVALID: return "INFO1A_INVALID";
    case V90_RX_EVENT_S:              return "S";
    case V90_RX_EVENT_TRN_LOCK:       return "TRN_LOCK";
    case V90_RX_EVENT_J:              return "J";
    case V90_RX_EVENT_J_PRIME:        return "J_PRIME";
    case V90_RX_EVENT_CP_VALID:       return "CP_VALID";
    case V90_RX_EVENT_CP_INVALID:     return "CP_INVALID";
    case V90_RX_EVENT_E:              return "E";
    case V90_RX_EVENT_B1:             return "B1";
    case V90_RX_EVENT_FAILURE:        return "FAILURE";
    case V90_RX_EVENT_RETRAIN:        return "RETRAIN";
    case V90_RX_EVENT_TIMEOUT:        return "TIMEOUT";
    }
    return "UNKNOWN";
}

bool v90_handle_rx_event(v90_state_t *s, v90_rx_event_t event)
{
    if (!s)
        return false;

    switch (event) {
    case V90_RX_EVENT_S:
        if (s->tx_phase == V90_TX_JD && !s->jd_terminate_requested) {
            fprintf(stderr, "[V90] Phase 3: far-end S detected, terminating Jd at the next frame boundary\n");
            s->jd_terminate_requested = true;
            return true;
        }
        if (s->tx_phase == V90_TX_DIL && !s->dil_terminate_requested) {
            fprintf(stderr, "[V90] Phase 3: subsequent far-end S detected during DIL, terminating at the next segment boundary\n");
            s->dil_terminate_requested = true;
            return true;
        }
        return false;

    case V90_RX_EVENT_CP_VALID:
        if (s->tx_phase == V90_TX_TRN2D
            && (s->v92_mode
                ? (s->v92_native_cpu_rx ? s->phase4_mapper_ready
                                        : (s->cp_nbits > 0))
                : s->phase4_mapper_ready)
            && !s->cp_ready) {
            fprintf(stderr, "[V90] Phase 4: valid far-end CPt received\n");
            s->cp_ready = true;
            s->sample_count = 0;
            return true;
        }
        if (!s->v92_mode && s->data_cp_received
            && (s->tx_phase == V90_TX_TRN2D || s->tx_phase == V90_TX_MP)) {
            fprintf(stderr, "[V90] Phase 4: valid far-end %s received\n",
                    s->cp_ack_received ? "CP'" : "data-mode CP");
            return true;
        }
        return false;

    case V90_RX_EVENT_E:
        if (s->v92_mode && s->v92_native_cpu_rx) {
            /* §9.6.1.1.4/V.92: E2u counts as remote acknowledgement. */
            if (s->tx_phase == V90_TX_SUVD || s->tx_phase == V90_TX_CP) {
                fprintf(stderr,
                        "[V90] Phase 4 V.92: far-end E2u received; completing current sequence\n");
                s->e_received = true;
                s->v92_remote_ack_received = true;
                return true;
            }
            return false;
        }
        if (s->tx_phase == V90_TX_MP && s->mp_acknowledge
            && s->data_mapper_ready) {
            fprintf(stderr,
                    "[V90] Phase 4: far-end 20-bit E received; completing current MP'\n");
            s->e_received = true;
            return true;
        }
        return false;

    case V90_RX_EVENT_B1:
        if (s->tx_phase == V90_TX_B1D || s->tx_phase == V90_TX_DATA) {
            fprintf(stderr, "[V90] Phase 4: far-end B1 received\n");
            s->b1_received = true;
            return true;
        }
        return false;

    default:
        return false;
    }
}

void v90_notify_s_detected(v90_state_t *s)
{
    (void)v90_handle_rx_event(s, V90_RX_EVENT_S);
}

bool v90_training_complete(v90_state_t *s)
{
    return s ? s->training_complete : false;
}

bool v90_set_phase4_cp(v90_state_t *s, const vpcm_cp_frame_t *cp)
{
    vpcm_cp_frame_t expected;
    vpcm_cp_frame_t received;

    if (!s || !cp)
        return false;
    if (!vpcm_cp_encode_bits(cp, s->cp_bits, &s->cp_nbits))
        return false;
    s->cp_bit_pos = 0;

    if (s->v92_mode) {
        /* Native mode: a CPt (training parameters) configures the TRN2d
         * mapper used for the mapped SUVd/CPd/Ed transmit path. */
        if (s->v92_native_cpu_rx && !cp->v90_compatibility)
            return v90_configure_phase4_mapper(s, cp);
        s->cp_frame = *cp;
        return true;
    }

    /* Table 14 bit 19: 0 = CPt training parameters, 1 = data-mode CP. */
    if (!cp->v90_compatibility) {
        if (!s->phase4_mapper_ready)
            return v90_configure_phase4_mapper(s, cp);

        expected = s->cp_frame;
        received = *cp;
        expected.acknowledge = false;
        received.acknowledge = false;
        return !cp->acknowledge && vpcm_cp_frames_equal(&expected, &received);
    }

    if (!s->phase4_mapper_ready)
        return false;
    if (!s->data_mapper_ready)
        return v90_configure_data_mapper(s, cp);

    /* Repeated data-mode CP/CP' frames may change only acknowledge. */
    expected = s->data_cp_frame;
    received = *cp;
    expected.acknowledge = false;
    received.acknowledge = false;
    if (!vpcm_cp_frames_equal(&expected, &received))
        return false;
    if (cp->acknowledge && s->tx_phase != V90_TX_MP)
        return false;
    if (cp->acknowledge)
        s->cp_ack_received = true;
    s->data_cp_received = true;
    s->data_cp_frame.acknowledge = cp->acknowledge;
    return true;
}

int v90_copy_phase4_mp_bits(const v90_state_t *s, uint8_t *bits, int max_bits)
{
    if (!s || !bits || !s->phase4_mapper_ready
        || s->mp_nbits <= 0 || max_bits < s->mp_nbits)
        return 0;
    memcpy(bits, s->mp_bits, (size_t)s->mp_nbits);
    return s->mp_nbits;
}

int v90_data_bits_per_frame(const v90_state_t *s)
{
    return (s && s->data_mapper_ready) ? s->data_mapper_d : 0;
}

int v90_data_input_bytes_needed(const v90_state_t *s)
{
    int missing;

    if (!s || !s->data_mapper_ready)
        return 0;
    missing = s->data_mapper_d - s->data_input_bit_count;
    return (missing > 0) ? (missing + 7) / 8 : 0;
}

int v90_tx_data_frame_codewords(v90_state_t *s,
                                uint8_t codewords[V90_FRAME_LEN],
                                const uint8_t *data,
                                int data_len,
                                int *data_consumed,
                                bool fill_with_ones)
{
    uint64_t frame_bits;
    uint64_t frame_mask;
    int needed;
    int consumed = 0;

    if (data_consumed)
        *data_consumed = 0;
    if (!s || !codewords || !s->data_mapper_ready || data_len < 0
        || (data_len > 0 && !data))
        return 0;

    needed = v90_data_input_bytes_needed(s);
    while (consumed < data_len && consumed < needed) {
        s->data_input_bits |= (uint64_t)data[consumed] << s->data_input_bit_count;
        s->data_input_bit_count += 8;
        consumed++;
    }
    if (data_consumed)
        *data_consumed = consumed;

    if (s->data_input_bit_count < s->data_mapper_d) {
        int missing;

        if (!fill_with_ones)
            return 0;
        missing = s->data_mapper_d - s->data_input_bit_count;
        s->data_input_bits |= ((1ULL << missing) - 1ULL)
                              << s->data_input_bit_count;
        s->data_input_bit_count = s->data_mapper_d;
    }

    frame_mask = (1ULL << s->data_mapper_d) - 1ULL;
    frame_bits = s->data_input_bits & frame_mask;
    s->data_input_bits >>= s->data_mapper_d;
    s->data_input_bit_count -= s->data_mapper_d;
    if (!v90_data_mapper_fill_frame(s, frame_bits))
        return 0;
    memcpy(codewords, s->data_mapper_frame, V90_FRAME_LEN);
    s->data_mapper_frame_pos = V90_FRAME_LEN;
    return V90_FRAME_LEN;
}

void v90_notify_cp_ready(v90_state_t *s)
{
    (void)v90_handle_rx_event(s, V90_RX_EVENT_CP_VALID);
}

static int v90_codeword_to_ucode_law(v90_law_t law, uint8_t codeword)
{
    uint8_t magnitude = (uint8_t)(codeword & 0x7F);

    if (law == V90_LAW_ULAW)
        return 0x7F - magnitude;
    for (int ucode = 0; ucode < 128; ucode++) {
        if ((v90_ucode_to_alaw[ucode] & 0x7F) == magnitude)
            return ucode;
    }
    return -1;
}

int v90_demap_mapped_frame(v90_law_t law,
                           const vpcm_cp_frame_t *cp,
                           int bits_per_frame,
                           uint32_t *descramble_reg,
                           int *prev_sign,
                           const uint8_t codewords[V90_FRAME_LEN],
                           uint8_t bits_out[])
{
    int labels[V90_FRAME_LEN];
    int moduli[V90_FRAME_LEN];
    uint8_t scrambled[64];
    uint64_t r;
    int sign_prev;
    int k;

    if (!cp || !descramble_reg || !prev_sign || !codewords || !bits_out
        || cp->shaping_redundancy != 0
        || bits_per_frame < V90_FRAME_LEN
        || bits_per_frame > (int)sizeof(scrambled))
        return 0;
    k = bits_per_frame - V90_FRAME_LEN;

    sign_prev = *prev_sign;
    for (int i = 0; i < V90_FRAME_LEN; i++) {
        int constellation = cp->dfi[i];
        int sign = (codewords[i] & 0x80) ? 1 : 0;
        int ucode = v90_codeword_to_ucode_law(law, codewords[i]);
        int label = 0;
        int m = 0;

        if (ucode < 0 || constellation >= cp->constellation_count
            || !vpcm_cp_mask_get(cp->masks[constellation], ucode))
            return 0;
        for (int u = 0; u < VPCM_CP_MASK_BITS; u++) {
            if (vpcm_cp_mask_get(cp->masks[constellation], u)) {
                /* The transmitter assigns labels in descending U-code
                   order; count selected values above this U-code. */
                if (u > ucode)
                    label++;
                m++;
            }
        }
        labels[i] = label;
        moduli[i] = m;
        scrambled[i] = (uint8_t)((sign ^ sign_prev) & 1);
        sign_prev = sign;
    }

    r = 0;
    for (int i = V90_FRAME_LEN - 1; i >= 0; i--)
        r = (uint64_t)moduli[i] * r + (uint64_t)labels[i];
    if (k < 64 && (r >> k) != 0)
        return 0;
    for (int i = 0; i < k; i++)
        scrambled[V90_FRAME_LEN + i] = (uint8_t)((r >> i) & 1);

    for (int i = 0; i < bits_per_frame; i++)
        bits_out[i] = (uint8_t)v90_descramble_reg_bit(descramble_reg,
                                                      scrambled[i]);
    *prev_sign = sign_prev;
    return bits_per_frame;
}

void v90_enable_v92_mode(v90_state_t *s)
{
    if (!s)
        return;
    s->v92_mode = true;
    /* Default Table 30 CPd profile until real upstream measurements or an
     * explicit v90_set_v92_cpd_profile() call refine it. */
    if (s->v92_gain_q0_16 == 0) {
        s->v92_upstream_drn = 14;      /* (14 + 17) x 8000 / 6 bps */
        s->v92_trellis_select = 0;     /* 16-state */
        s->v92_gain_q0_16 = 0x8000;    /* 4G = 0.5 -> G = 0.125 */
    }
}

bool v90_set_v92_cpd_profile(v90_state_t *s,
                             uint8_t upstream_drn,
                             uint8_t trellis_select,
                             uint16_t gain_q0_16)
{
    if (!s || upstream_drn > 19 || trellis_select > 2 || gain_q0_16 == 0)
        return false;
    s->v92_upstream_drn = upstream_drn;
    s->v92_trellis_select = trellis_select;
    s->v92_gain_q0_16 = gain_q0_16;
    return true;
}

void v90_enable_v92_native_cpu_rx(v90_state_t *s)
{
    if (s && s->v92_mode)
        s->v92_native_cpu_rx = true;
}

bool v90_set_v92_suvu(v90_state_t *s, bool acknowledge)
{
    if (!s || !s->v92_mode || !s->v92_native_cpu_rx)
        return false;
    s->v92_suvu_received = true;
    if (acknowledge)
        s->v92_remote_ack_received = true;
    return true;
}

bool v90_set_v92_cpu(v90_state_t *s, const vpcm_cp_frame_t *cpu)
{
    vpcm_cp_frame_t expected;
    vpcm_cp_frame_t received;

    if (!s || !cpu || !s->v92_mode || !s->v92_native_cpu_rx)
        return false;
    /* CPu carries data-mode parameters (d = drn + 20). */
    if (!cpu->v90_compatibility)
        return false;

    if (!s->v92_cpu_received) {
        expected = *cpu;
        expected.acknowledge = false;
        if (!v90_configure_data_mapper(s, &expected))
            return false;
        s->data_cp_frame.acknowledge = cpu->acknowledge;
        s->v92_cpu_received = true;
        if (cpu->acknowledge)
            s->v92_remote_ack_received = true;
        fprintf(stderr,
                "[V90] Phase 4 V.92: CPu%s accepted (drn=%u, D=%d, K=%d, Sr=%d)\n",
                cpu->acknowledge ? "'" : "", (unsigned)cpu->drn,
                s->data_mapper_d, s->data_mapper_k, s->data_mapper_sr);
        return true;
    }

    /* Repeated CPu/CPu' frames may change only the acknowledge bit. */
    expected = s->data_cp_frame;
    received = *cpu;
    expected.acknowledge = false;
    received.acknowledge = false;
    if (!vpcm_cp_frames_equal(&expected, &received))
        return false;
    if (cpu->acknowledge) {
        s->v92_remote_ack_received = true;
        fprintf(stderr, "[V90] Phase 4 V.92: CPu' acknowledgement received\n");
    }
    s->data_cp_frame.acknowledge = cpu->acknowledge;
    return true;
}

bool v90_get_v92_cpu(const v90_state_t *s, vpcm_cp_frame_t *out)
{
    if (!s || !out || !s->v92_cpu_received)
        return false;
    *out = s->data_cp_frame;
    return true;
}

void v90_reset_data_mode(v90_state_t *s)
{
    if (!s)
        return;
    if (s->data_mapper_ready)
        v90_reset_negotiated_data_mapper(s);
    else
        v90_reset_data_pump_state(s);
}

int v90_phase3_tx(v90_state_t *s, int16_t amp[], int len)
{
    for (int i = 0; i < len; i++) {
        uint8_t codeword = v90_phase3_codeword(s);
        amp[i] = v90_pcm_to_linear(s->law, codeword);
    }
    return len;
}

int v90_phase3_tx_codewords(v90_state_t *s, uint8_t codewords[], int len)
{
    if (!s || !codewords || len <= 0)
        return 0;
    for (int i = 0; i < len; i++)
        codewords[i] = v90_phase3_codeword(s);
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
