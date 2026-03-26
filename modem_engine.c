/*
 * modem_engine.c — V.34/V.90 modem state machine
 *
 * Digital (server) side modem over SIP/G.711:
 *
 *   Primary mode: V.34 full duplex (up to 33.6 kbps)
 *     SpanDSP 3.0.0 V.34 handles all training phases internally:
 *     Phase 2 (DPSK INFO exchange), Phase 3 (equalizer training),
 *     Phase 4 (final training and rate negotiation).
 *
 *   V.90 mode: downstream PCM (up to 56 kbps) + upstream V.34
 *     ITU-T V.90 §5 PCM codeword injection into the G.711 RTP stream.
 *     Training uses V.34 Phases 2-4, then TX switches to direct PCM injection.
 *     Upstream (analog→digital) uses V.34 demodulation.
 *
 *   Fallback: V.22bis full duplex (2400 bps)
 *
 *   Handshake:
 *     SpanDSP V.8 negotiation; V.34 preferred, V.22bis fallback.
 */

#include "modem_engine.h"
#include "data_interface.h"
#include "clock_recovery.h"

#include <spandsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <stdarg.h>
#include <sys/time.h>

/* V.34 RX/TX stage enums — copied from spandsp/private/v34.h to avoid
   pulling in SpanDSP's full internal header chain.  We access g_v34->rx.stage
   and g_v34->tx.stage by casting through a minimal struct overlay. */
enum v34_rx_stages_e {
    V34_RX_STAGE_INFO0 = 1,
    V34_RX_STAGE_INFOH,
    V34_RX_STAGE_INFO1C,
    V34_RX_STAGE_INFO1A,
    V34_RX_STAGE_TONE_A,
    V34_RX_STAGE_TONE_B,
    V34_RX_STAGE_L1_L2,
    V34_RX_STAGE_CC,
    V34_RX_STAGE_PRIMARY_CHANNEL,
    V34_RX_STAGE_PHASE3_WAIT_S,
    V34_RX_STAGE_PHASE3_TRAINING,
    V34_RX_STAGE_PHASE3_DONE,
    V34_RX_STAGE_PHASE4_S,
    V34_RX_STAGE_PHASE4_S_BAR,
    V34_RX_STAGE_PHASE4_TRN,
    V34_RX_STAGE_PHASE4_MP,
    V34_RX_STAGE_DATA,
};

enum v34_tx_stages_e {
    V34_TX_STAGE_INITIAL_PREAMBLE = 1,
    V34_TX_STAGE_INFO0,
    V34_TX_STAGE_INITIAL_A,
    V34_TX_STAGE_FIRST_A,
    V34_TX_STAGE_FIRST_NOT_A,
    V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN,
    V34_TX_STAGE_SECOND_A,
    V34_TX_STAGE_L1,
    V34_TX_STAGE_L2,
    V34_TX_STAGE_POST_L2_A,
    V34_TX_STAGE_POST_L2_NOT_A,
    V34_TX_STAGE_A_SILENCE,
    V34_TX_STAGE_PRE_INFO1_A,
    V34_TX_STAGE_INFO1,
    V34_TX_STAGE_FIRST_B,
    V34_TX_STAGE_FIRST_B_INFO_SEEN,
    V34_TX_STAGE_FIRST_NOT_B_WAIT,
    V34_TX_STAGE_FIRST_NOT_B,
    V34_TX_STAGE_FIRST_B_SILENCE,
    V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE,
    V34_TX_STAGE_SECOND_B,
    V34_TX_STAGE_SECOND_B_WAIT,
    V34_TX_STAGE_SECOND_NOT_B,
    V34_TX_STAGE_INFO0_RETRY,
    V34_TX_STAGE_FIRST_S,
    V34_TX_STAGE_FIRST_NOT_S,
    V34_TX_STAGE_MD,
    V34_TX_STAGE_SECOND_S,
    V34_TX_STAGE_SECOND_NOT_S,
    V34_TX_STAGE_TRN,
    V34_TX_STAGE_J,
    V34_TX_STAGE_J_DASHED,
    V34_TX_STAGE_PHASE4_WAIT,
    V34_TX_STAGE_PHASE4_S,
    V34_TX_STAGE_PHASE4_NOT_S,
    V34_TX_STAGE_PHASE4_TRN,
    V34_TX_STAGE_MP,
};

/* ------------------------------------------------------------------ */
/* V.90 downstream encoder constants (ITU-T V.90 §5)                  */
/* ------------------------------------------------------------------ */

/*
 * Number of positive constellation points per data frame interval.
 * V.90 uses 6 data frame intervals (i=0..5), each with Mi points.
 * For simplicity we use Mi=128 (7 magnitude bits per symbol),
 * giving a theoretical max of 7+1 (sign) = 8 bits/symbol @ 8000 sym/s.
 *
 * K=42 magnitude bits + S=6 sign bits per 6-symbol frame.
 * Scrambled throughput ≈ 48 bits/frame × (8000/6 frames/s) ≈ 64 kbps;
 * after framing/overhead the effective rate is ≈ 56 kbps.
 *
 * NOTE: In a conforming implementation, Mi is negotiated during training.
 */
#define V90_MI          128     /* Constellation points per frame interval */
#define V90_K_BITS      42      /* Magnitude data bits per 6-symbol frame */
#define V90_FRAME_LEN   6       /* Symbols per data frame */
#define V90_RATE_BPS    56000   /* Advertised downstream rate */

/*
 * Ucode-to-PCM codeword mapping (ITU-T V.90 Table 1/V.90).
 *
 * µ-law: simple formula: positive codeword = 0xFF - Ucode.
 * A-law: independent mapping defined in Table 1 (not derivable via linear).
 * The polarity (sign) is applied separately via the MSB of the G.711 byte.
 */

/* A-law positive codewords indexed by Ucode (from ITU-T V.90 Table 1) */
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

/* Current G.711 law (set by sip_modem.c after codec negotiation) */
static me_law_t g_law = ME_LAW_ULAW;

/*
 * Convert a Ucode (0-127) to the positive G.711 codeword for the
 * currently negotiated law.
 */
static inline uint8_t ucode_to_pcm_positive(int ucode)
{
    if (g_law == ME_LAW_ALAW)
        return v90_ucode_to_alaw[ucode & 0x7F];
    return (uint8_t)(0xFF - ucode);  /* µ-law */
}

/*
 * Convert a positive G.711 codeword to linear PCM, using the
 * currently negotiated law.
 */
static inline int16_t pcm_to_linear(uint8_t codeword)
{
    if (g_law == ME_LAW_ALAW)
        return alaw_to_linear(codeword);
    return ulaw_to_linear(codeword);
}

/*
 * Return the idle (silence) codeword for the current law.
 */
static inline uint8_t pcm_idle(void)
{
    return (g_law == ME_LAW_ALAW) ? (uint8_t)0xD5 : (uint8_t)0xFF;
}

/* ------------------------------------------------------------------ */
/* V.90 scrambler (V.34 polynomial GPC, ITU-T V.34 §7)                */
/* ------------------------------------------------------------------ */

/*
 * Self-synchronising scrambler using the V.34 generator polynomial:
 *   GPC(x) = x^23 + x^5 + 1   (23-stage LFSR)
 */
typedef struct {
    uint32_t sr; /* shift register, 23 bits significant */
} v90_scrambler_t;

static void v90_scrambler_init(v90_scrambler_t *s)
{
    s->sr = 0x7FFFFF; /* all-ones start state */
}

static uint8_t v90_scramble_byte(v90_scrambler_t *s, uint8_t in)
{
    uint8_t out = 0;
    for (int i = 0; i < 8; i++) {
        int in_bit  = (in >> i) & 1;
        int fb      = ((s->sr >> 22) ^ (s->sr >> 4)) & 1; /* x^23 XOR x^5 */
        int out_bit = in_bit ^ fb;
        s->sr = ((s->sr << 1) | out_bit) & 0x7FFFFF;
        out  |= (uint8_t)(out_bit << i);
    }
    return out;
}

/* ------------------------------------------------------------------ */
/* V.90 downstream encoder state                                       */
/* ------------------------------------------------------------------ */

typedef struct {
    v90_scrambler_t scrambler;

    /* Modulus encoder residue across frames */
    uint64_t R;

    /* Data frame interval index (0..5) */
    int frame_idx;

    /* Sign bit differential coding state (§5.4.5.1) */
    int prev_sign; /* $5 of previous data frame */

    /* Bit accumulator for pulling whole bytes from the upstream data buffer */
    uint8_t  byte_acc;
    int      bits_in_acc;
} v90_enc_t;

static void v90_enc_init(v90_enc_t *e)
{
    memset(e, 0, sizeof(*e));
    v90_scrambler_init(&e->scrambler);
    e->prev_sign = 0;
}

/*
 * Encode one 6-symbol data frame from the downstream data buffer.
 * Fills pcm_out[0..5] with µ-law codewords ready for the RTP stream.
 *
 * Uses simplified encoding (Mi=128 for all intervals, Sr=0 spectral shaping):
 *   1. Scramble 6 raw data bytes (one per symbol, 48 bits total)
 *   2. Split into 6 × (7 magnitude bits + 1 sign bit)
 *   3. Map magnitude → Ucode → µ-law positive code
 *   4. Apply sign via differential coding (§5.4.5.1)
 *   5. Set µ-law MSB from the differentially-coded sign
 */
static void v90_encode_frame(v90_enc_t *e, const uint8_t *data_in,
                              uint8_t *pcm_out)
{
    int sign = e->prev_sign;

    for (int i = 0; i < V90_FRAME_LEN; i++) {
        /* Scramble one byte */
        uint8_t s = v90_scramble_byte(&e->scrambler, data_in[i]);

        /* 7 magnitude bits (b0..b6) select the constellation point */
        uint8_t mag   = s & 0x7F;          /* Ucode: 0..127             */
        int     s_bit = (s >> 7) & 1;      /* raw sign bit from data    */

        /* §5.4.5.1 differential coding for sign (Sr=0):
         *   $i = s_i XOR $_{i-1}  (where $_{-1} = $5 of previous frame) */
        sign = s_bit ^ sign;

        /* Map Ucode → positive PCM codeword (µ-law or A-law per negotiated law) */
        uint8_t mu = ucode_to_pcm_positive(mag);

        /* Apply polarity: G.711 MSB=1 → positive, MSB=0 → negative
         * (same convention for both µ-law and A-law).
         * ucode_to_pcm_positive returns positive codewords (MSB=1).
         * For negative: clear the MSB. */
        if (sign == 0)
            mu &= 0x7F; /* make negative */

        pcm_out[i] = mu;
    }

    e->prev_sign = sign; /* save $5 for next frame */
}

/* ------------------------------------------------------------------ */
/* Ring buffers for data exchange between threads                      */
/* ------------------------------------------------------------------ */

#define DATA_RING_SIZE 16384

typedef struct {
    uint8_t  buf[DATA_RING_SIZE];
    volatile int head;
    volatile int tail;
    pthread_mutex_t mtx;
} data_ring_t;

static void dring_init(data_ring_t *r) {
    r->head = r->tail = 0;
    pthread_mutex_init(&r->mtx, NULL);
}

static int dring_write(data_ring_t *r, const uint8_t *d, int len) {
    pthread_mutex_lock(&r->mtx);
    int n = 0;
    for (int i = 0; i < len; i++) {
        int next = (r->head + 1) % DATA_RING_SIZE;
        if (next == r->tail) break;
        r->buf[r->head] = d[i];
        r->head = next;
        n++;
    }
    pthread_mutex_unlock(&r->mtx);
    return n;
}

static int dring_read(data_ring_t *r, uint8_t *buf, int max) {
    pthread_mutex_lock(&r->mtx);
    int n = 0;
    while (n < max && r->tail != r->head) {
        buf[n++] = r->buf[r->tail];
        r->tail  = (r->tail + 1) % DATA_RING_SIZE;
    }
    pthread_mutex_unlock(&r->mtx);
    return n;
}

static int dring_available(data_ring_t *r) {
    pthread_mutex_lock(&r->mtx);
    int n = (r->head - r->tail + DATA_RING_SIZE) % DATA_RING_SIZE;
    pthread_mutex_unlock(&r->mtx);
    return n;
}

/* ------------------------------------------------------------------ */
/* V.22bis get_bit / put_bit callbacks for SpanDSP                    */
/* ------------------------------------------------------------------ */

static data_ring_t downstream_ring; /* data → modem → SIP (downstream TX) */
static data_ring_t upstream_ring;   /* SIP → modem → data (upstream RX) */
static void on_training_complete(me_modulation_t mod, int rate, const char *name);

/* ------------------------------------------------------------------ */
/* Phase trace helpers                                                 */
/* ------------------------------------------------------------------ */

static uint64_t g_trace_start_ms = 0;

static uint64_t trace_now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000ULL);
}

static const char *me_mod_to_str(me_modulation_t mod)
{
    switch (mod) {
    case ME_MOD_NONE:   return "NONE";
    case ME_MOD_V90:    return "V90";
    case ME_MOD_V34:    return "V34";
    case ME_MOD_V22BIS: return "V22BIS";
    default:            return "UNKNOWN";
    }
}

static void v8_mod_mask_to_str(int mask, char *buf, size_t size)
{
    int off = 0;
    if (size == 0)
        return;
    buf[0] = '\0';

    if (mask & V8_MOD_V90)
        off += snprintf(buf + off, size - (size_t)off, "%sV90", off ? "|" : "");
    if (mask & V8_MOD_V34)
        off += snprintf(buf + off, size - (size_t)off, "%sV34", off ? "|" : "");
    if (mask & V8_MOD_V22)
        off += snprintf(buf + off, size - (size_t)off, "%sV22", off ? "|" : "");
    if (mask & V8_MOD_V32)
        off += snprintf(buf + off, size - (size_t)off, "%sV32", off ? "|" : "");
    if (mask & V8_MOD_V21)
        off += snprintf(buf + off, size - (size_t)off, "%sV21", off ? "|" : "");
    if (off == 0)
        snprintf(buf, size, "none");
}

static void trace_phase(const char *fmt, ...)
{
    uint64_t now = trace_now_ms();
    if (g_trace_start_ms == 0)
        g_trace_start_ms = now;

    fprintf(stderr, "[TRACE +%llums] ", (unsigned long long)(now - g_trace_start_ms));

    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static int parse_env_int(const char *name, int fallback)
{
    const char *s = getenv(name);
    if (!s || !*s)
        return fallback;
    char *endp = NULL;
    long v = strtol(s, &endp, 10);
    if (endp == s || *endp != '\0')
        return fallback;
    return (int)v;
}

static const char *v34_rx_stage_name(int stage)
{
    switch (stage) {
    case 0:                              return "IDLE";
    case V34_RX_STAGE_INFO0:             return "INFO0";
    case V34_RX_STAGE_INFOH:             return "INFOH";
    case V34_RX_STAGE_INFO1C:            return "INFO1C";
    case V34_RX_STAGE_INFO1A:            return "INFO1A";
    case V34_RX_STAGE_TONE_A:            return "TONE_A";
    case V34_RX_STAGE_TONE_B:            return "TONE_B";
    case V34_RX_STAGE_L1_L2:             return "L1_L2";
    case V34_RX_STAGE_CC:                return "CC";
    case V34_RX_STAGE_PRIMARY_CHANNEL:   return "PRIMARY_CHANNEL";
    case V34_RX_STAGE_PHASE3_WAIT_S:     return "PHASE3_WAIT_S";
    case V34_RX_STAGE_PHASE3_TRAINING:   return "PHASE3_TRAINING";
    case V34_RX_STAGE_PHASE3_DONE:       return "PHASE3_DONE";
    case V34_RX_STAGE_PHASE4_S:          return "PHASE4_S";
    case V34_RX_STAGE_PHASE4_S_BAR:      return "PHASE4_S_BAR";
    case V34_RX_STAGE_PHASE4_TRN:        return "PHASE4_TRN";
    case V34_RX_STAGE_PHASE4_MP:         return "PHASE4_MP";
    case V34_RX_STAGE_DATA:              return "DATA";
    default:                             return "UNKNOWN";
    }
}

static const char *v34_tx_stage_name(int stage)
{
    switch (stage) {
    case 0:                                        return "IDLE";
    case V34_TX_STAGE_INITIAL_PREAMBLE:            return "INITIAL_PREAMBLE";
    case V34_TX_STAGE_INFO0:                       return "INFO0";
    case V34_TX_STAGE_INITIAL_A:                   return "INITIAL_A";
    case V34_TX_STAGE_FIRST_A:                     return "FIRST_A";
    case V34_TX_STAGE_FIRST_NOT_A:                 return "FIRST_NOT_A";
    case V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN:   return "FIRST_NOT_A_REVERSAL";
    case V34_TX_STAGE_SECOND_A:                    return "SECOND_A";
    case V34_TX_STAGE_L1:                          return "L1";
    case V34_TX_STAGE_L2:                          return "L2";
    case V34_TX_STAGE_POST_L2_A:                   return "POST_L2_A";
    case V34_TX_STAGE_POST_L2_NOT_A:               return "POST_L2_NOT_A";
    case V34_TX_STAGE_A_SILENCE:                   return "A_SILENCE";
    case V34_TX_STAGE_PRE_INFO1_A:                 return "PRE_INFO1_A";
    case V34_TX_STAGE_INFO1:                       return "INFO1";
    case V34_TX_STAGE_FIRST_B:                     return "FIRST_B";
    case V34_TX_STAGE_FIRST_B_INFO_SEEN:           return "FIRST_B_INFO_SEEN";
    case V34_TX_STAGE_FIRST_NOT_B_WAIT:            return "FIRST_NOT_B_WAIT";
    case V34_TX_STAGE_FIRST_NOT_B:                 return "FIRST_NOT_B";
    case V34_TX_STAGE_FIRST_B_SILENCE:             return "FIRST_B_SILENCE";
    case V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE: return "FIRST_B_POST_REVERSAL_SILENCE";
    case V34_TX_STAGE_SECOND_B:                    return "SECOND_B";
    case V34_TX_STAGE_SECOND_B_WAIT:               return "SECOND_B_WAIT";
    case V34_TX_STAGE_SECOND_NOT_B:                return "SECOND_NOT_B";
    case V34_TX_STAGE_INFO0_RETRY:                 return "INFO0_RETRY";
    case V34_TX_STAGE_FIRST_S:                     return "FIRST_S";
    case V34_TX_STAGE_FIRST_NOT_S:                 return "FIRST_NOT_S";
    case V34_TX_STAGE_MD:                          return "MD";
    case V34_TX_STAGE_SECOND_S:                    return "SECOND_S";
    case V34_TX_STAGE_SECOND_NOT_S:                return "SECOND_NOT_S";
    case V34_TX_STAGE_TRN:                         return "TRN";
    case V34_TX_STAGE_J:                           return "J";
    case V34_TX_STAGE_J_DASHED:                    return "J_DASHED";
    case V34_TX_STAGE_PHASE4_WAIT:                 return "PHASE4_WAIT";
    case V34_TX_STAGE_PHASE4_S:                    return "PHASE4_S";
    case V34_TX_STAGE_PHASE4_NOT_S:                return "PHASE4_NOT_S";
    case V34_TX_STAGE_PHASE4_TRN:                  return "PHASE4_TRN";
    case V34_TX_STAGE_MP:                          return "MP";
    default:                                       return "UNKNOWN";
    }
}

static bool valid_v34_baud(int baud)
{
    return baud == 2400 || baud == 2743 || baud == 2800 ||
           baud == 3000 || baud == 3200 || baud == 3429;
}

static int max_v34_bps_for_baud(int baud)
{
    /* V.34 Table 2: max bit rate per symbol rate */
    switch (baud) {
    case 2400: return 21600;
    case 2743: case 2800: return 26400;
    case 3000: return 28800;
    case 3200: return 31200;
    case 3429: return 33600;
    default:   return 21600;
    }
}

static bool valid_v34_bps(int bps)
{
    return bps >= 2400 && bps <= 33600 && (bps % 2400) == 0;
}

/* Bit accumulator for V.22bis TX (one byte at a time) */
static uint8_t  v22bis_tx_byte = 0;
static int      v22bis_tx_bits = 0;

static int v22bis_get_bit_cb(void *user_data)
{
    (void)user_data;
    if (v22bis_tx_bits == 0) {
        /* Fetch the next byte from the downstream ring */
        uint8_t b;
        if (dring_read(&downstream_ring, &b, 1) == 1) {
            v22bis_tx_byte = b;
            v22bis_tx_bits = 8;
        } else {
            return SIG_STATUS_END_OF_DATA; /* Mark for silence */
        }
    }
    int bit = v22bis_tx_byte & 1;
    v22bis_tx_byte >>= 1;
    v22bis_tx_bits--;
    return bit;
}

/* Bit accumulator for V.22bis RX */
static uint8_t  v22bis_rx_byte = 0;
static int      v22bis_rx_bits = 0;

static void v22bis_put_bit_cb(void *user_data, int bit)
{
    (void)user_data;
    if (bit < 0) {
        if (bit == SIG_STATUS_CARRIER_UP || bit == SIG_STATUS_TRAINING_SUCCEEDED)
            on_training_complete(ME_MOD_V22BIS, 2400, "V.22bis");
        return;
    }

    v22bis_rx_byte |= (uint8_t)((bit & 1) << v22bis_rx_bits);
    v22bis_rx_bits++;
    if (v22bis_rx_bits == 8) {
        dring_write(&upstream_ring, &v22bis_rx_byte, 1);
        v22bis_rx_byte = 0;
        v22bis_rx_bits = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static me_state_t      g_state     = ME_IDLE;
static me_modulation_t g_mod       = ME_MOD_NONE;
static pthread_mutex_t g_state_mtx;
static bool            g_calling_party = false; /* false=answerer, true=caller */
static bool            g_invert_v34_role = false; /* debug override via env */

/* SpanDSP modem contexts */
static v8_state_t     *g_v8      = NULL;
static v22bis_state_t *g_v22bis  = NULL;
static v34_state_t    *g_v34     = NULL;

/* V.90 downstream encoder */
static v90_enc_t      g_v90_enc;

/* Echo canceller for full-duplex V.34.
   The FXS hybrid in the AudioCodes gateway leaks our TX signal back into
   RX.  Without cancellation, the ~20-30 dB return loss causes ~30-50% bit
   errors in the V.34 demodulator during Phase 4 and data mode.
   SpanDSP's modem_echo canceller uses LMS — ideal for constant-amplitude
   modem signals.  512 taps = 64 ms at 8 kHz, covering hybrid echo + RTP
   packetization delay (~40ms round-trip). */
#define ECHO_CAN_TAPS 512
static modem_echo_can_segment_state_t *g_echo_can = NULL;
/* Echo canceller is only useful during Phase 3/4 when TX (1600 Hz) and RX (1800 Hz)
   overlap in the RRC passband.  During Phase 2, TX is at 2400 Hz and RX at 1200 Hz —
   well-separated, and the LMS diverges if active.  We track RX frame count since
   ME_TRAINING started and only activate after a delay (Phase 2 takes 2-5s). */
/* EC disabled — notch filter used instead (see g_notch) */
static const bool g_advertise_v90 = true; /* Advertise V.90 — PCM downstream active */
static int        g_v34_start_baud = 2400;   /* 3200 has 91 Hz separation (notch unusable); 2400 has 200 Hz */
static int        g_v34_start_bps  = 0;     /* 0 = auto (max for baud rate) */
static int        g_training_tx_samples = 0; /* Sample counter for TX silencing echo test */

/* Handshake timeouts (in milliseconds) */
#define V8_TIMEOUT_MS       10000   /* V.8 negotiation: 10 seconds */
#define TRAINING_TIMEOUT_MS 30000   /* V.34 training (Phase 2-4): 30 seconds */
static uint64_t g_phase_start_ms = 0;      /* When current phase started */

/* V.34 RX stage tracking — used for notch filter activation and diagnostics */
static int g_last_rx_stage = 0;            /* Last logged RX stage */
static int g_last_tx_stage = 0;            /* Last logged TX stage */

/* TX sample ring buffer (kept for future use but EC is disabled).
   Size must be a power of 2. */
#define TX_BUF_SIZE 4096
#define TX_BUF_MASK (TX_BUF_SIZE - 1)
static int16_t g_tx_buf[TX_BUF_SIZE];
static int     g_tx_buf_wr = 0;  /* write position (updated by me_tx_audio) */
static int     g_tx_buf_rd = 0;  /* read position (updated by me_rx_audio) */

/* Notch filter to remove our own TX carrier echo from the RX signal.
   During V.34 Phase 3/4, we TX at 1600 Hz (answerer low carrier) and RX at 1800 Hz.
   The FXS hybrid leaks our TX back into RX. A second-order IIR notch filter at
   1600 Hz removes this echo without significantly affecting the 1800 Hz signal.
   Design: f0=1600 Hz, fs=8000 Hz, Q=15.
     ω0 = 2π×1600/8000 = 0.4π
     r = 1 - π×(f0/Q)/fs = 1 - π×106.67/8000 = 0.9581
     cos(ω0) = cos(72°) = 0.30902
   Transfer function: H(z) = (1 - 2cos(ω0)z⁻¹ + z⁻²) / (1 - 2r·cos(ω0)z⁻¹ + r²z⁻²) */
typedef struct {
    float b0, b1, b2;  /* numerator (zeros on unit circle at ω0) */
    float a1, a2;       /* denominator (poles at radius r) */
    float x1, x2;       /* input delay line */
    float y1, y2;       /* output delay line */
    bool  active;
} notch_filter_t;

static notch_filter_t g_notch = {0};

/* Clock recovery */
static cr_state_t     g_cr;

/* Audio diagnostics: accumulated energy and sample count for V.8 logging */
static int64_t g_v8_rx_energy;
static int64_t g_training_rx_energy;
static int     g_training_rx_count;
static int     g_v8_rx_count;

/* Pending SIP URI for outgoing calls (set by me_dial) */
static char g_dial_uri[256];

static void notch_filter_init(notch_filter_t *nf, float freq, float q, float fs)
{
    float w0 = 2.0f * M_PI * freq / fs;
    float cosw0 = cosf(w0);
    float r = 1.0f - (M_PI * freq / q) / fs;
    nf->b0 = 1.0f;
    nf->b1 = -2.0f * cosw0;
    nf->b2 = 1.0f;
    nf->a1 = -2.0f * r * cosw0;
    nf->a2 = r * r;
    nf->x1 = nf->x2 = 0.0f;
    nf->y1 = nf->y2 = 0.0f;
    nf->active = true;
}

static void notch_filter_apply(notch_filter_t *nf, int16_t *samples, int len)
{
    if (!nf->active)
        return;
    for (int i = 0; i < len; i++) {
        float x0 = (float)samples[i];
        float y0 = nf->b0*x0 + nf->b1*nf->x1 + nf->b2*nf->x2
                  - nf->a1*nf->y1 - nf->a2*nf->y2;
        nf->x2 = nf->x1;
        nf->x1 = x0;
        nf->y2 = nf->y1;
        nf->y1 = y0;
        /* Clamp to int16 range */
        if (y0 > 32767.0f) y0 = 32767.0f;
        if (y0 < -32768.0f) y0 = -32768.0f;
        samples[i] = (int16_t)y0;
    }
}

static void on_training_complete(me_modulation_t mod, int rate, const char *name)
{
    pthread_mutex_lock(&g_state_mtx);
    if (g_state == ME_TRAINING && g_mod == mod) {
        g_state = ME_DATA;
        g_phase_start_ms = 0;
        pthread_mutex_unlock(&g_state_mtx);
        fprintf(stderr, "[ME] %s training complete (%d bps)\n", name, rate);
        trace_phase("%s training complete: rate=%d mod=%s", name, rate, me_mod_to_str(mod));
        di_on_connected(rate);
        return;
    }
    pthread_mutex_unlock(&g_state_mtx);
}

/* ------------------------------------------------------------------ */
/* V.34 get_bit / put_bit callbacks for SpanDSP                       */
/* ------------------------------------------------------------------ */

/* Forward declarations */
static void start_v22bis_training(void);

/* Bit accumulators for V.34 TX and RX (one byte at a time) */
static uint8_t  v34_tx_byte = 0;
static int      v34_tx_bits = 0;
static uint8_t  v34_rx_byte = 0;
static int      v34_rx_bits = 0;

static int v34_get_bit_cb(void *user_data)
{
    (void)user_data;
    if (v34_tx_bits == 0) {
        uint8_t b;
        if (dring_read(&downstream_ring, &b, 1) == 1) {
            v34_tx_byte = b;
            v34_tx_bits = 8;
        } else {
            return SIG_STATUS_END_OF_DATA;
        }
    }
    int bit = v34_tx_byte & 1;
    v34_tx_byte >>= 1;
    v34_tx_bits--;
    return bit;
}

static void v34_put_bit_cb(void *user_data, int bit)
{
    (void)user_data;
    if (bit < 0) {
        /* Status event from V.34 training state machine */
        trace_phase("V34 rx status=%s (%d)", signal_status_to_str(bit), bit);
        if (bit == SIG_STATUS_CARRIER_UP || bit == SIG_STATUS_TRAINING_SUCCEEDED) {
            if (g_state == ME_TRAINING) {
                g_state = ME_DATA;
                g_phase_start_ms = 0;
                int rate = v34_get_current_bit_rate(g_v34);
                if (g_mod == ME_MOD_V90) {
                    fprintf(stderr, "[ME] V.90 training complete (upstream V.34 %d bps, downstream PCM %d bps)\n",
                            rate, V90_RATE_BPS);
                    trace_phase("V90 enter DATA: upstream=%d downstream=%d", rate, V90_RATE_BPS);
                    /* Re-init V.90 encoder for clean data mode start */
                    v90_enc_init(&g_v90_enc);
                    di_on_connected(V90_RATE_BPS);
                } else {
                    fprintf(stderr, "[ME] V.34 training complete (%d bps)\n", rate);
                    trace_phase("V34 enter DATA: rate=%d", rate);
                    di_on_connected(rate);
                }
                return;
            }
        }
        if (bit == SIG_STATUS_TRAINING_FAILED || bit == SIG_STATUS_CARRIER_DOWN) {
            if (g_state == ME_TRAINING && (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90)) {
                fprintf(stderr, "[ME] V.34 training failed (%s), falling back to V.22bis\n",
                        signal_status_to_str(bit));
                trace_phase("V34 training failed (%s) -> fallback V22BIS",
                            signal_status_to_str(bit));
                /* Clean up V.34 state */
                if (g_v34) {
                    v34_free(g_v34);
                    g_v34 = NULL;
                }
                g_notch.active = false;
                g_last_rx_stage = 0;
                g_last_tx_stage = 0;
                /* Fall back to V.22bis */
                pthread_mutex_lock(&g_state_mtx);
                start_v22bis_training();
                pthread_mutex_unlock(&g_state_mtx);
                return;
            }
        }
        fprintf(stderr, "[ME] V.34 status: %s (%d)\n", signal_status_to_str(bit), bit);
        return;
    }
    /* Normal data bit — accumulate into bytes, write to upstream ring */
    v34_rx_byte |= (uint8_t)((bit & 1) << v34_rx_bits);
    v34_rx_bits++;
    if (v34_rx_bits == 8) {
        dring_write(&upstream_ring, &v34_rx_byte, 1);
        v34_rx_byte = 0;
        v34_rx_bits = 0;
    }
}

/* ------------------------------------------------------------------ */
/* V.8 result callback                                                 */
/* ------------------------------------------------------------------ */

/* Start V.22bis training — shared helper used by V.8 result handler */
static void start_v22bis_training(void)
{
    /* Must be called with g_state_mtx held */
    g_mod   = ME_MOD_V22BIS;
    g_state = ME_TRAINING;
    trace_phase("enter TRAINING: mod=V22BIS role=%s", g_calling_party ? "caller" : "answerer");
    if (g_v22bis) {
        v22bis_free(g_v22bis);
        g_v22bis = NULL;
    }

    g_v22bis = v22bis_init(NULL, 2400, V22BIS_GUARD_TONE_NONE, g_calling_party,
                           v22bis_get_bit_cb, NULL,
                           v22bis_put_bit_cb, NULL);
    if (!g_v22bis)
        fprintf(stderr, "[ME] v22bis_init failed\n");
}

/* Start V.34 training — used when V.8 negotiates V.34 */
static void start_v34_training(void)
{
    /* Must be called with g_state_mtx held */
    g_mod   = ME_MOD_V34;
    g_state = ME_TRAINING;
    g_phase_start_ms = trace_now_ms();
    trace_phase("enter TRAINING: mod=V34 role=%s", g_calling_party ? "caller" : "answerer");
    g_training_rx_energy = 0;
    g_training_rx_count  = 0;
    g_training_tx_samples = 0;
    g_last_rx_stage = 0;
    g_last_tx_stage = 0;

    /* Reset bit accumulators */
    v34_tx_byte = 0;
    v34_tx_bits = 0;
    v34_rx_byte = 0;
    v34_rx_bits = 0;

    if (g_v34) {
        v34_free(g_v34);
        g_v34 = NULL;
    }

    /*
     * Init V.34 in caller/answerer role matching SIP call direction.
     * Start with a conservative profile that is typically more robust over
     * gateway+RTP paths, then iterate upward once baseline connectivity is proven.
     */
    int bps = g_v34_start_bps ? g_v34_start_bps : max_v34_bps_for_baud(g_v34_start_baud);
    g_v34 = v34_init(NULL,
                     g_v34_start_baud,
                     bps,
                     g_calling_party,
                     true,          /* full duplex */
                     v34_get_bit_cb, NULL,
                     v34_put_bit_cb, NULL);
    if (!g_v34) {
        fprintf(stderr, "[ME] v34_init failed, falling back to V.22bis\n");
        start_v22bis_training();
        return;
    }

    /* Enable SpanDSP logging for V.34 training diagnostics */
    logging_state_t *log = v34_get_logging_state(g_v34);
    if (log) {
        span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL |
                                SPAN_LOG_FLOW);
    }
    v34_tx_power(g_v34, -10.0f);  /* Boosted from -14; caller modem not detecting our Phase 2 */

    /* Echo canceller disabled — LMS diverges because far-end signal is too weak
       by the time it's safe to activate (after Phase 2). Use notch filter instead. */
    if (g_echo_can) {
        modem_echo_can_segment_free(g_echo_can);
        g_echo_can = NULL;
    }

    /* Initialize notch filter at our TX carrier to remove FXS hybrid echo.
       Carrier frequency depends on baud rate (V.34 Table 2):
         baud  low_carrier(d/e)   high_carrier(d/e)
         2400  1600 (2/3)         1800 (3/4)
         2743  1646 (3/5)         1829 (2/3)
         2800  1680 (3/5)         1867 (2/3)
         3000  1800 (3/5)         2000 (2/3)
         3200  1829 (4/7)         1920 (3/5)
         3429  1959 (4/7)         1959 (4/7)  -- same! unusable
       In duplex: answerer TX=low, caller TX=high. */
    {
        /* Compute exact baud rate and carrier from V.34 tables.
           exact_baud = 2400 * a/c (from baud_rate_parameters) */
        float exact_baud;
        float our_tx_carrier;
        switch (g_v34_start_baud) {
        case 2400: exact_baud = 2400.0f; break;
        case 2743: exact_baud = 2400.0f * 8.0f / 7.0f; break;
        case 2800: exact_baud = 2400.0f * 7.0f / 6.0f; break;
        case 3000: exact_baud = 2400.0f * 5.0f / 4.0f; break;
        case 3200: exact_baud = 2400.0f * 4.0f / 3.0f; break;
        case 3429: exact_baud = 2400.0f * 10.0f / 7.0f; break;
        default:   exact_baud = 2400.0f; break;
        }
        if (g_calling_party) {
            /* Caller TX = high carrier */
            switch (g_v34_start_baud) {
            case 2400: our_tx_carrier = exact_baud * 3.0f / 4.0f; break;
            case 2743: case 2800: case 3000:
                       our_tx_carrier = exact_baud * 2.0f / 3.0f; break;
            case 3200: our_tx_carrier = exact_baud * 3.0f / 5.0f; break;
            case 3429: our_tx_carrier = exact_baud * 4.0f / 7.0f; break;
            default:   our_tx_carrier = exact_baud * 3.0f / 4.0f; break;
            }
        } else {
            /* Answerer TX = low carrier */
            switch (g_v34_start_baud) {
            case 2400: our_tx_carrier = exact_baud * 2.0f / 3.0f; break;
            case 2743: case 3000:
                       our_tx_carrier = exact_baud * 3.0f / 5.0f; break;
            case 2800: our_tx_carrier = exact_baud * 3.0f / 5.0f; break;
            case 3200: our_tx_carrier = exact_baud * 4.0f / 7.0f; break;
            case 3429: our_tx_carrier = exact_baud * 4.0f / 7.0f; break;
            default:   our_tx_carrier = exact_baud * 2.0f / 3.0f; break;
            }
        }
        /* Compute RX carrier to check separation */
        float rx_carrier;
        if (g_calling_party) {
            /* Caller RX = low carrier */
            switch (g_v34_start_baud) {
            case 2400: rx_carrier = exact_baud * 2.0f / 3.0f; break;
            case 2743: case 3000:
                       rx_carrier = exact_baud * 3.0f / 5.0f; break;
            case 2800: rx_carrier = exact_baud * 3.0f / 5.0f; break;
            case 3200: rx_carrier = exact_baud * 4.0f / 7.0f; break;
            case 3429: rx_carrier = exact_baud * 4.0f / 7.0f; break;
            default:   rx_carrier = exact_baud * 2.0f / 3.0f; break;
            }
        } else {
            /* Answerer RX = high carrier */
            switch (g_v34_start_baud) {
            case 2400: rx_carrier = exact_baud * 3.0f / 4.0f; break;
            case 2743: case 2800: case 3000:
                       rx_carrier = exact_baud * 2.0f / 3.0f; break;
            case 3200: rx_carrier = exact_baud * 3.0f / 5.0f; break;
            case 3429: rx_carrier = exact_baud * 4.0f / 7.0f; break;
            default:   rx_carrier = exact_baud * 3.0f / 4.0f; break;
            }
        }
        float separation = fabsf(rx_carrier - our_tx_carrier);
        if (separation < 150.0f) {
            /* Carriers too close — notch would attenuate RX signal.
               SpanDSP V.34 has its own internal echo management. */
            g_notch.active = false;
            fprintf(stderr, "[ME] Notch filter DISABLED: TX=%.1f Hz, RX=%.1f Hz, "
                    "separation=%.1f Hz too narrow for %d baud\n",
                    our_tx_carrier, rx_carrier, separation, g_v34_start_baud);
        } else {
            notch_filter_init(&g_notch, our_tx_carrier, 30.0f, 8000.0f);
            fprintf(stderr, "[ME] Notch filter at %.1f Hz (Q=30) for %d baud, "
                    "RX carrier=%.1f Hz (sep=%.1f Hz)\n",
                    our_tx_carrier, g_v34_start_baud, rx_carrier, separation);
        }
    }

    fprintf(stderr, "[ME] V.34 training started (%s, %d baud, up to %d bps)\n",
            g_calling_party ? "caller" : "answerer", g_v34_start_baud, bps);
}

static void v8_result_handler(void *user_data, v8_parms_t *result)
{
    (void)user_data;
    char mod_str[96];
    v8_mod_mask_to_str(result->jm_cm.modulations, mod_str, sizeof(mod_str));

    fprintf(stderr, "[ME] V.8 result: status=%d, modulations=0x%X pstn_access=0x%X\n",
            result->status, result->jm_cm.modulations, result->jm_cm.pstn_access);
    trace_phase("V8 result: status=%s (%d) mods=%s (0x%X) protocol=0x%X pstn=0x%X pcm=0x%X",
                v8_status_to_str(result->status), result->status,
                mod_str, result->jm_cm.modulations, result->jm_cm.protocols,
                result->jm_cm.pstn_access, result->jm_cm.pcm_modem_availability);

    /* V8_STATUS_V8_OFFERED just means the other end offered V.8 — still in progress */
    if (result->status == V8_STATUS_IN_PROGRESS ||
        result->status == V8_STATUS_V8_OFFERED) {
        fprintf(stderr, "[ME] V.8 in progress (status=%d)\n", result->status);
        return;
    }

    if (result->status == V8_STATUS_NON_V8_CALL) {
        /*
         * Remote end doesn't support V.8 (e.g. plain V.22bis modem or
         * ATA auto-answer with no V.8).  Fall back to V.22bis directly.
         */
        fprintf(stderr, "[ME] Non-V.8 call detected, falling back to V.22bis\n");
        pthread_mutex_lock(&g_state_mtx);
        start_v22bis_training();
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }

    if (result->status != V8_STATUS_V8_CALL) {
        fprintf(stderr, "[ME] V.8 failed (status=%d), hanging up\n", result->status);
        me_hangup();
        return;
    }

    /* V8_STATUS_V8_CALL — negotiation complete, inspect agreed modulation */
    pthread_mutex_lock(&g_state_mtx);

    if ((result->jm_cm.modulations & V8_MOD_V90) && g_advertise_v90
        && (result->jm_cm.modulations & V8_MOD_V34)) {
        /*
         * V.90 selected: downstream = PCM codeword injection (up to 56 kbps),
         * upstream = V.34 modulation.  Training uses V.34 Phases 2-4; after
         * training completes, TX switches from V.34 to direct PCM injection.
         */
        fprintf(stderr, "[ME] V.8 negotiated V.90 (PCM downstream + V.34 upstream)\n");
        trace_phase("V8 selected V90");
        g_mod = ME_MOD_V90;
        start_v34_training();
        /* start_v34_training sets g_mod = ME_MOD_V34; override back to V90 */
        g_mod = ME_MOD_V90;
        /* Enable V.90 INFO0d frame generation in SpanDSP V.34 */
        if (g_v34)
            v34_set_v90_mode(g_v34, (g_law == ME_LAW_ALAW) ? 1 : 0);

    } else if (result->jm_cm.modulations & V8_MOD_V34) {
        fprintf(stderr, "[ME] V.8 negotiated V.34 (full duplex, up to 33.6 kbps)\n");
        trace_phase("V8 selected V34");
        start_v34_training();

    } else if (result->jm_cm.modulations & V8_MOD_V22) {
        fprintf(stderr, "[ME] V.8 negotiated V.22bis fallback\n");
        trace_phase("V8 selected V22BIS");
        start_v22bis_training();

    } else {
        fprintf(stderr, "[ME] V.8 no usable modulation (0x%X), hanging up\n",
                result->jm_cm.modulations);
        trace_phase("V8 selected no usable modulation -> hangup");
        pthread_mutex_unlock(&g_state_mtx);
        me_hangup();
        return;
    }

    pthread_mutex_unlock(&g_state_mtx);
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

void me_init(void)
{
    pthread_mutex_init(&g_state_mtx, NULL);
    dring_init(&downstream_ring);
    dring_init(&upstream_ring);
    cr_init(&g_cr, 8000);
    v90_enc_init(&g_v90_enc);
    {
        const char *inv = getenv("ME_V34_INVERT_ROLE");
        g_invert_v34_role = (inv && (inv[0] == '1' || inv[0] == 'y' || inv[0] == 'Y' ||
                                     inv[0] == 't' || inv[0] == 'T'));
        if (g_invert_v34_role)
            fprintf(stderr, "[ME] DEBUG: role inversion enabled (ME_V34_INVERT_ROLE)\n");
    }
    {
        int env_baud = parse_env_int("ME_V34_BAUD", g_v34_start_baud);
        int env_bps  = parse_env_int("ME_V34_BPS", 0);
        if (valid_v34_baud(env_baud))
            g_v34_start_baud = env_baud;
        if (env_bps > 0 && valid_v34_bps(env_bps))
            g_v34_start_bps = env_bps;
        int effective_bps = g_v34_start_bps ? g_v34_start_bps : max_v34_bps_for_baud(g_v34_start_baud);
        fprintf(stderr, "[ME] V.34 start profile: %d baud / %d bps\n",
                g_v34_start_baud, effective_bps);
    }
    g_state = ME_IDLE;
    g_mod   = ME_MOD_NONE;
}

void me_destroy(void)
{
    if (g_v8)       { v8_free(g_v8);                             g_v8       = NULL; }
    if (g_v22bis)   { v22bis_free(g_v22bis);                     g_v22bis   = NULL; }
    if (g_v34)      { v34_free(g_v34);                           g_v34      = NULL; }
    if (g_echo_can) { modem_echo_can_segment_free(g_echo_can);   g_echo_can = NULL; }
    pthread_mutex_destroy(&g_state_mtx);
}

/* ------------------------------------------------------------------ */
/* Control                                                             */
/* ------------------------------------------------------------------ */

void me_dial(const char *sip_uri)
{
    pthread_mutex_lock(&g_state_mtx);
    if (g_state == ME_IDLE) {
        snprintf(g_dial_uri, sizeof(g_dial_uri), "%s", sip_uri);
        g_state = ME_DIALING;
        /* Actual SIP call is initiated by sip_modem.c which polls me_get_state() */
    }
    pthread_mutex_unlock(&g_state_mtx);
}

void me_answer(void)
{
    pthread_mutex_lock(&g_state_mtx);
    /* SIP answer is triggered by sip_modem.c */
    pthread_mutex_unlock(&g_state_mtx);
}

void me_hangup(void)
{
    pthread_mutex_lock(&g_state_mtx);
    g_state = ME_HANGUP;
    pthread_mutex_unlock(&g_state_mtx);
    /* sip_modem.c will detect ME_HANGUP and hang up the SIP call */
}

/* Called by sip_modem.c when the SIP call media becomes active */
void me_on_sip_connected(void)
{
    pthread_mutex_lock(&g_state_mtx);

    cr_reset(&g_cr);
    g_trace_start_ms = trace_now_ms();
    g_v8_rx_energy     = 0;
    g_v8_rx_count      = 0;

    /* Outgoing dial = caller role; incoming auto-answer = answerer role. */
    g_calling_party = (g_state == ME_DIALING);
    if (g_invert_v34_role)
        g_calling_party = !g_calling_party;
    trace_phase("SIP media connected: role=%s", g_calling_party ? "caller" : "answerer");

    /* Initialise V.8 */
    v8_parms_t v8_parms;
    memset(&v8_parms, 0, sizeof(v8_parms));
    v8_parms.modem_connect_tone = g_calling_party ? MODEM_CONNECT_TONES_NONE
                                                  : MODEM_CONNECT_TONES_ANSAM_PR;
    v8_parms.send_ci            = g_calling_party;  /* Caller must send CI so answerer knows we want V.8 */
    v8_parms.v92                = -1;    /* don't send V.92 extension */
    v8_parms.jm_cm.call_function      = V8_CALL_V_SERIES;
    /* Advertise V.90 + V.34 + V.22bis.  V.90 training starts with V.34
     * Phase 2, so we can safely advertise it; the actual V.90 PCM switch
     * happens after training (not yet implemented). */
    v8_parms.jm_cm.modulations        = V8_MOD_V34 | V8_MOD_V22;
    if (g_advertise_v90)
        v8_parms.jm_cm.modulations   |= V8_MOD_V90;
    v8_parms.jm_cm.protocols          = V8_PROTOCOL_LAPM_V42;
    if (g_advertise_v90) {
        v8_parms.jm_cm.pstn_access            = V8_PSTN_ACCESS_DCE_ON_DIGITAL;
        v8_parms.jm_cm.pcm_modem_availability = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;
    } else {
        v8_parms.jm_cm.pstn_access            = 0;
        v8_parms.jm_cm.pcm_modem_availability = 0;
    }
    v8_parms.jm_cm.nsf                = -1;   /* don't send NSF */
    v8_parms.jm_cm.t66                = -1;   /* don't send T.66 */

    if (g_v8) { v8_free(g_v8); g_v8 = NULL; }
    g_v8 = v8_init(NULL, g_calling_party, &v8_parms,
                   v8_result_handler, NULL);
    if (!g_v8) {
        fprintf(stderr, "[ME] v8_init failed\n");
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }

    /* Enable SpanDSP logging for V.8 diagnostics */
    {
        logging_state_t *log = v8_get_logging_state(g_v8);
        if (log)
            span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL |
                                    SPAN_LOG_FLOW);
    }

    g_state = ME_V8;
    g_mod   = ME_MOD_NONE;
    g_phase_start_ms = trace_now_ms();
    pthread_mutex_unlock(&g_state_mtx);
    trace_phase("enter V8: advertised mods=%s", g_advertise_v90 ? "V90|V34|V22" : "V34|V22");

    fprintf(stderr, "[ME] SIP connected as %s, starting V.8 handshake\n",
            g_calling_party ? "caller" : "answerer");
}

/* Called by sip_modem.c when the SIP call is torn down */
void me_on_sip_disconnected(void)
{
    pthread_mutex_lock(&g_state_mtx);

    if (g_v8)       { v8_free(g_v8);                             g_v8       = NULL; }
    if (g_v22bis)   { v22bis_free(g_v22bis);                     g_v22bis   = NULL; }
    if (g_v34)      { v34_free(g_v34);                           g_v34      = NULL; }
    if (g_echo_can) { modem_echo_can_segment_free(g_echo_can);   g_echo_can = NULL; }

    me_state_t prev = g_state;
    g_state = ME_IDLE;
    g_mod   = ME_MOD_NONE;
    g_calling_party = false;
    pthread_mutex_unlock(&g_state_mtx);
    trace_phase("SIP disconnected: prev_state=%d -> IDLE", prev);

    if (prev == ME_DATA || prev == ME_TRAINING || prev == ME_V8)
        di_on_disconnected();
}

/* ------------------------------------------------------------------ */
/* Audio I/O — called from PJSIP media thread (real-time)             */
/* ------------------------------------------------------------------ */

void me_rx_audio(const int16_t *amp, int len)
{
    pthread_mutex_lock(&g_state_mtx);
    me_state_t state = g_state;
    pthread_mutex_unlock(&g_state_mtx);

    /* Check for phase timeouts */
    if (g_phase_start_ms > 0) {
        uint64_t elapsed = trace_now_ms() - g_phase_start_ms;
        if (state == ME_V8 && elapsed > V8_TIMEOUT_MS) {
            fprintf(stderr, "[ME] V.8 negotiation timed out after %llu ms\n",
                    (unsigned long long)elapsed);
            trace_phase("V8 timeout after %llums", (unsigned long long)elapsed);
            g_phase_start_ms = 0;
            me_hangup();
            return;
        }
        if (state == ME_TRAINING && elapsed > TRAINING_TIMEOUT_MS) {
            fprintf(stderr, "[ME] Training timed out after %llu ms (mod=%s)\n",
                    (unsigned long long)elapsed, me_mod_to_str(g_mod));
            trace_phase("TRAINING timeout after %llums mod=%s",
                        (unsigned long long)elapsed, me_mod_to_str(g_mod));
            g_phase_start_ms = 0;
            if (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) {
                /* V.34/V.90 training failed — fall back to V.22bis */
                fprintf(stderr, "[ME] %s training timeout, falling back to V.22bis\n",
                        g_mod == ME_MOD_V90 ? "V.90" : "V.34");
                trace_phase("%s timeout -> fallback V22BIS",
                            g_mod == ME_MOD_V90 ? "V90" : "V34");
                if (g_v34) {
                    v34_free(g_v34);
                    g_v34 = NULL;
                }
                g_notch.active = false;
                g_last_rx_stage = 0;
                g_last_tx_stage = 0;
                pthread_mutex_lock(&g_state_mtx);
                start_v22bis_training();
                pthread_mutex_unlock(&g_state_mtx);
                return;
            }
            /* V.22bis timeout — give up */
            me_hangup();
            return;
        }
    }

    switch (state) {
    case ME_V8:
        /* Accumulate received energy for a 1-second diagnostic log */
        for (int i = 0; i < len; i++)
            g_v8_rx_energy += (int64_t)amp[i] * amp[i];
        g_v8_rx_count += len;
        if (g_v8_rx_count >= 8000) {
            double rms = sqrt((double)g_v8_rx_energy / g_v8_rx_count);
            fprintf(stderr, "[ME] V.8 rx: RMS=%.1f (%d samples) — %s\n",
                    rms, g_v8_rx_count,
                    rms < 10.0 ? "WARNING: near-silence, check conference bridge" : "audio OK");
            g_v8_rx_energy = 0;
            g_v8_rx_count  = 0;
        }
        /* Feed received audio to V.8 receiver */
        if (g_v8)
            v8_rx(g_v8, amp, len);
        break;

    case ME_TRAINING:
    case ME_DATA:
        /* RX energy diagnostic — log every second during training */
        if (state == ME_TRAINING) {
            for (int i = 0; i < len; i++)
                g_training_rx_energy += (int64_t)amp[i] * amp[i];
            g_training_rx_count += len;
            if (g_training_rx_count >= 8000) {
                double rms = sqrt((double)g_training_rx_energy / g_training_rx_count);
                /* Keep this diagnostic for abnormal levels only. */
                if (rms < 20.0 || rms > 2000.0) {
                    fprintf(stderr, "[ME] Training rx: RMS=%.1f (%d samples)\n",
                            rms, g_training_rx_count);
                }
                g_training_rx_energy = 0;
                g_training_rx_count  = 0;
            }
        }
        /* Feed audio to the appropriate modem receiver.
           Apply notch filter to remove our TX carrier echo from FXS hybrid. */
        if (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) {
            pthread_mutex_lock(&g_state_mtx);
            if ((g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) && g_v34) {
                /* Log RX/TX stage transitions for training diagnostics */
                int rx_stage = v34_get_rx_stage(g_v34);
                int tx_stage = v34_get_tx_stage(g_v34);
                if (rx_stage != g_last_rx_stage || tx_stage != g_last_tx_stage) {
                    trace_phase("V34 stage: rx=%s(%d) tx=%s(%d)",
                                v34_rx_stage_name(rx_stage), rx_stage,
                                v34_tx_stage_name(tx_stage), tx_stage);
                    g_last_rx_stage = rx_stage;
                    g_last_tx_stage = tx_stage;
                }

                /* Apply notch filter once Phase 3 begins (TX/RX use separated
                   carriers).  During Phase 2, both sides share 1200 Hz DPSK —
                   the notch at 1600+ Hz could affect Phase 2 through its skirts,
                   and there's no echo to cancel since TX/RX overlap.
                   Previously this used v34_get_primary_channel_active() which
                   only returns true after Phase 4 completes — far too late.
                   The equalizer needs clean RX from Phase 3 onset. */
                bool notch_active = (rx_stage >= V34_RX_STAGE_PHASE3_WAIT_S) ||
                                    (tx_stage >= V34_TX_STAGE_FIRST_S);
                int16_t filtered[len];
                memcpy(filtered, amp, len * sizeof(int16_t));
                if (notch_active)
                    notch_filter_apply(&g_notch, filtered, len);
                v34_rx(g_v34, filtered, len);
                /* RX PCM dump during training */
                {
                    static FILE *rx_dump = NULL;
                    if (!rx_dump) {
                        rx_dump = fopen("/tmp/v34_rx.raw", "wb");
                        if (rx_dump)
                            fprintf(stderr, "[ME] RX PCM dump: /tmp/v34_rx.raw (s16le 8000Hz mono)\n");
                    }
                    if (rx_dump) {
                        fwrite(filtered, sizeof(int16_t), len, rx_dump);
                        static int rx_dump_count = 0;
                        rx_dump_count += len;
                        if (rx_dump_count >= 8000) {
                            fflush(rx_dump);
                            rx_dump_count = 0;
                        }
                    }
                }
            }
            pthread_mutex_unlock(&g_state_mtx);
        } else if (g_v22bis)
            v22bis_rx(g_v22bis, amp, len);

        /* In DATA mode, flush received bytes to the PTY */
        if (state == ME_DATA) {
            uint8_t buf[256];
            int n;
            while ((n = dring_read(&upstream_ring, buf, sizeof(buf))) > 0)
                di_write_data(buf, n);
        }
        break;

    default:
        break;
    }
}

void me_tx_audio(int16_t *amp, int len)
{
    pthread_mutex_lock(&g_state_mtx);
    me_state_t state = g_state;
    pthread_mutex_unlock(&g_state_mtx);

    memset(amp, 0, sizeof(int16_t) * (size_t)len);

    switch (state) {
    case ME_V8:
        /* Generate V.8 negotiation audio */
        if (g_v8)
            v8_tx(g_v8, amp, len);
        break;

    case ME_TRAINING:
        /*
         * V.34/V.22bis training: SpanDSP handles the training state machine
         * internally (Phase 2 DPSK INFO exchange, tone probing, Phase 3/4
         * equalizer training). The put_bit callback fires SIG_STATUS_CARRIER_UP
         * when training completes, transitioning us to ME_DATA.
         */
        if (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) {
            pthread_mutex_lock(&g_state_mtx);
            if ((g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) && g_v34)
                v34_tx(g_v34, amp, len);
            pthread_mutex_unlock(&g_state_mtx);
            /* TX PCM dump + RMS diagnostic during training */
            {
                static FILE *tx_dump = NULL;
                static int64_t tx_energy = 0;
                static int tx_count = 0;
                if (!tx_dump) {
                    tx_dump = fopen("/tmp/v34_tx.raw", "wb");
                    if (tx_dump)
                        fprintf(stderr, "[ME] TX PCM dump: /tmp/v34_tx.raw (s16le 8000Hz mono)\n");
                }
                if (tx_dump)
                    fwrite(amp, sizeof(int16_t), len, tx_dump);
                for (int i = 0; i < len; i++)
                    tx_energy += (int64_t)amp[i] * amp[i];
                tx_count += len;
                if (tx_count >= 8000) {
                    double rms = sqrt((double)tx_energy / tx_count);
                    fprintf(stderr, "[ME] Training TX: RMS=%.1f (%d samples)\n",
                            rms, tx_count);
                    if (tx_dump) fflush(tx_dump);
                    tx_energy = 0;
                    tx_count = 0;
                }
            }
        } else if (g_v22bis)
            v22bis_tx(g_v22bis, amp, len);
        break;

    case ME_DATA:
        if (g_mod == ME_MOD_V34) {
            /* V.34 full duplex data TX */
            pthread_mutex_lock(&g_state_mtx);
            if (g_mod == ME_MOD_V34 && g_v34)
                v34_tx(g_v34, amp, len);
            pthread_mutex_unlock(&g_state_mtx);
        } else if (g_mod == ME_MOD_V90) {
            /*
             * V.90 downstream: encode 6 bytes per frame into 6 µ-law
             * codewords and convert to linear PCM for the RTP stream.
             */
            int pos = 0;
            while (pos + V90_FRAME_LEN <= len) {
                uint8_t data_in[V90_FRAME_LEN];
                uint8_t pcm_out[V90_FRAME_LEN];

                if (dring_read(&downstream_ring, data_in, V90_FRAME_LEN)
                    < V90_FRAME_LEN) {
                    uint8_t idle = pcm_idle();
                    for (int i = 0; i < V90_FRAME_LEN; i++)
                        amp[pos + i] = pcm_to_linear(idle);
                    pos += V90_FRAME_LEN;
                    continue;
                }

                v90_encode_frame(&g_v90_enc, data_in, pcm_out);
                for (int i = 0; i < V90_FRAME_LEN; i++)
                    amp[pos + i] = pcm_to_linear(pcm_out[i]);
                pos += V90_FRAME_LEN;
            }
        } else {
            /* V.22bis duplex downstream TX */
            if (g_v22bis)
                v22bis_tx(g_v22bis, amp, len);
        }
        break;

    default:
        break;
    }

    /* Buffer TX samples for the echo canceller.
       Must happen after TX generation so me_rx_audio can subtract
       our echo from the received signal. */
    if (g_mod == ME_MOD_V34) {
        pthread_mutex_lock(&g_state_mtx);
        if (g_echo_can && g_mod == ME_MOD_V34) {
        for (int i = 0; i < len; i++) {
            g_tx_buf[g_tx_buf_wr] = amp[i];
            g_tx_buf_wr = (g_tx_buf_wr + 1) & TX_BUF_MASK;
        }
        }
        pthread_mutex_unlock(&g_state_mtx);
    }
}

/* ------------------------------------------------------------------ */
/* Data I/O                                                            */
/* ------------------------------------------------------------------ */

int me_put_data(const uint8_t *buf, int len)
{
    /* Upstream data: application → modem → SIP (for V.22bis TX or V.90 stub) */
    return dring_write(&downstream_ring, buf, len);
}

int me_get_data(uint8_t *buf, int max_len)
{
    /* Downstream data: SIP → modem → application */
    return dring_read(&upstream_ring, buf, max_len);
}

/* ------------------------------------------------------------------ */
/* State query                                                         */
/* ------------------------------------------------------------------ */

me_state_t me_get_state(void)
{
    pthread_mutex_lock(&g_state_mtx);
    me_state_t s = g_state;
    pthread_mutex_unlock(&g_state_mtx);
    return s;
}

me_modulation_t me_get_modulation(void)
{
    pthread_mutex_lock(&g_state_mtx);
    me_modulation_t m = g_mod;
    pthread_mutex_unlock(&g_state_mtx);
    return m;
}

void me_set_law(me_law_t law)
{
    pthread_mutex_lock(&g_state_mtx);
    g_law = law;
    pthread_mutex_unlock(&g_state_mtx);
    fprintf(stderr, "[ME] PCM law set to %s\n",
            law == ME_LAW_ALAW ? "A-law (PCMA)" : "u-law (PCMU)");
}

me_law_t me_get_law(void)
{
    return g_law;
}

/* Expose the dial URI for sip_modem.c to pick up */
const char *me_get_dial_uri(void)
{
    return g_dial_uri;
}
