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

#include <spandsp.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* V.90 downstream encoder constants (ITU-T V.90 §5) */
#define V90_MI          128     /* Default constellation points per frame interval */
#define V90_FRAME_LEN   6       /* Symbols per data frame */

/* Jd frame is 72 bits (Table 13): 17 sync + 51 data + 4 fill */
#define V90_JD_BITS     72

/* Sd: 64 repetitions of 6-symbol pattern = 384 symbols */
#define V90_SD_REPS     64
#define V90_SD_BAR_REPS 8

/* TRN1d: multiple of 6 symbols; use 768 symbols (128ms @ 8kHz) */
#define V90_TRN1D_LEN   768

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

    /* Jd frame data */
    uint8_t          jd_bits[16];   /* Jd frame packed into bytes (72 bits) */
    int              jd_bit_pos;    /* Current bit position in Jd frame */

    /* Downstream PCM encoder state (data mode) */
    int              prev_sign;     /* §5.4.5.1 differential sign coding */
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
    if (!sign)
        pcm &= 0x7F;  /* Clear sign bit → negative polarity */
    return v90_pcm_to_linear(law, pcm);
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

/* Generate one Phase 3 TX sample */
static int16_t v90_phase3_sample(v90_state_t *s)
{
    int sign;
    int ucode;

    switch (s->tx_phase) {
    case V90_TX_JD:
        /* §8.4.2: Jd bits are scrambled and differentially encoded,
         * transmitted as sign of U_INFO PCM codeword. */
        {
            int bit = v90_get_jd_bit(s);
            int scrambled = v90_scramble_bit(&s->scrambler, bit);
            /* Differential encoding: sign = scrambled XOR previous sign */
            s->diff_enc ^= scrambled;
            sign = s->diff_enc;
            s->sample_count++;
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
                /* Transition to Sd */
                fprintf(stderr, "[V90] Phase 3: J'd complete, starting Sd\n");
                s->tx_phase = V90_TX_SD;
                s->sample_count = 0;
                s->rep_count = 0;
            }
            return v90_pcm_signed(s->law, s->u_info, sign);
        }

    case V90_TX_SD:
        /* §8.4.4: Sd = 64 reps of {+W, +0, +W, -W, -0, -W}
         * W = Ucode(16 + U_INFO), 0 = Ucode 0 */
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
         * Scrambler initialized to zero. */
        {
            int scrambled = v90_scramble_bit(&s->scrambler, 1);
            sign = scrambled;  /* sign=0 → negative, sign=1 → positive */
            s->sample_count++;
            if (s->sample_count >= V90_TRN1D_LEN) {
                fprintf(stderr, "[V90] Phase 3: TRN1d complete (%d symbols), entering Phase 4\n",
                        s->sample_count);
                s->tx_phase = V90_TX_PHASE4;
                s->sample_count = 0;
            }
            return v90_pcm_signed(s->law, s->u_info, sign);
        }

    case V90_TX_PHASE4:
        /* TODO: Phase 4 (B1d, TRN2d, MP, R, E exchanges) */
        return v90_pcm_signed(s->law, 0, 1);  /* idle for now */

    default:
        break;
    }

    return v90_pcm_to_linear(s->law, v90_pcm_idle(s->law));
}

/* ---- Public API ---- */

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
    if (s->v34)
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
    return s->tx_phase >= V90_TX_JD && s->tx_phase <= V90_TX_TRN1D;
}

void v90_start_phase3(v90_state_t *s, int u_info)
{
    if (u_info > 0 && u_info < 128)
        s->u_info = u_info;

    fprintf(stderr, "[V90] Starting Phase 3 TX (U_INFO=%d, law=%s)\n",
            s->u_info, s->law == V90_LAW_ALAW ? "A-law" : "u-law");

    /* Build Jd frame */
    v90_build_jd(s);

    /* Initialize scrambler and differential encoder for Jd.
     * §8.4.2: differential encoder initialized with final symbol of TRN1d.
     * Since we haven't sent TRN1d yet, initialize to 0.
     * Actually, Jd comes BEFORE TRN1d. The scrambler is initialized to zero. */
    v90_scrambler_init(&s->scrambler);
    s->diff_enc = 0;
    s->jd_bit_pos = 0;
    s->sample_count = 0;
    s->rep_count = 0;

    s->tx_phase = V90_TX_JD;
}

int v90_phase3_tx(v90_state_t *s, int16_t amp[], int len)
{
    for (int i = 0; i < len; i++)
        amp[i] = v90_phase3_sample(s);
    return len;
}

/*
 * Encode one 6-symbol data frame.
 * Fills pcm_out[0..5] with G.711 codewords.
 */
static void v90_encode_frame(v90_state_t *s, const uint8_t *data_in,
                             uint8_t *pcm_out)
{
    int sign = s->prev_sign;

    for (int i = 0; i < V90_FRAME_LEN; i++) {
        uint8_t sc = v90_scramble_byte(&s->scrambler, data_in[i]);
        uint8_t mag   = sc & 0x7F;
        int     s_bit = (sc >> 7) & 1;

        /* §5.4.5.1 differential sign coding (Sr=0) */
        sign = s_bit ^ sign;

        uint8_t mu = ucode_to_pcm_positive(s->law, mag);
        if (sign == 0)
            mu &= 0x7F;  /* negative polarity */

        pcm_out[i] = mu;
    }

    s->prev_sign = sign;
}

int v90_tx_data(v90_state_t *s, int16_t amp[], int len,
                const uint8_t *data_in, int data_len)
{
    int pos = 0;
    int consumed = 0;

    while (pos + V90_FRAME_LEN <= len) {
        if (consumed + V90_FRAME_LEN > data_len) {
            /* Not enough data — fill with idle */
            uint8_t idle = v90_pcm_idle(s->law);
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
    uint8_t idle = v90_pcm_idle(s->law);
    int16_t sample = v90_pcm_to_linear(s->law, idle);
    for (int i = 0; i < len; i++)
        amp[i] = sample;
}

logging_state_t *v90_get_logging_state(v90_state_t *s)
{
    return v34_get_logging_state(s->v34);
}
