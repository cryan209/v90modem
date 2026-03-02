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
 *   Future: V.90 downstream (up to 56 kbps)
 *     ITU-T V.90 §5 PCM codeword injection into the G.711 RTP stream.
 *     V.90 encoder code is retained but not yet active (requires V.90-specific
 *     Phase 3/4 training which differs from standard V.34).
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
    if (bit < 0) return; /* SIG_STATUS_* sentinel, ignore */

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

/* SpanDSP modem contexts */
static v8_state_t     *g_v8      = NULL;
static v22bis_state_t *g_v22bis  = NULL;
static v34_state_t    *g_v34     = NULL;

/* V.90 downstream encoder */
static v90_enc_t      g_v90_enc;

/* Clock recovery */
static cr_state_t     g_cr;

/* Audio diagnostics: accumulated energy and sample count for V.8 logging */
static int64_t g_v8_rx_energy;
static int64_t g_training_rx_energy;
static int     g_training_rx_count;
static int     g_v8_rx_count;

/* Pending SIP URI for outgoing calls (set by me_dial) */
static char g_dial_uri[256];

/* ------------------------------------------------------------------ */
/* V.34 get_bit / put_bit callbacks for SpanDSP                       */
/* ------------------------------------------------------------------ */

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
        if (bit == SIG_STATUS_CARRIER_UP) {
            pthread_mutex_lock(&g_state_mtx);
            if (g_state == ME_TRAINING) {
                g_state = ME_DATA;
                pthread_mutex_unlock(&g_state_mtx);
                int rate = v34_get_current_bit_rate(g_v34);
                fprintf(stderr, "[ME] V.34 training complete (%d bps)\n", rate);
                di_on_connected(rate);
                return;
            }
            pthread_mutex_unlock(&g_state_mtx);
        }
        fprintf(stderr, "[ME] V.34 status: %d\n", bit);
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

    if (!g_v22bis) {
        g_v22bis = v22bis_init(NULL, 2400, V22BIS_GUARD_TONE_NONE, false,
                               v22bis_get_bit_cb, NULL,
                               v22bis_put_bit_cb, NULL);
        if (!g_v22bis)
            fprintf(stderr, "[ME] v22bis_init failed\n");
    } else {
        v22bis_restart(g_v22bis, 2400);
    }
}

/* Start V.34 training — used when V.8 negotiates V.34 */
static void start_v34_training(void)
{
    /* Must be called with g_state_mtx held */
    g_mod   = ME_MOD_V34;
    g_state = ME_TRAINING;
    g_training_rx_energy = 0;
    g_training_rx_count  = 0;

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
     * Init V.34 as answerer (calling_party=false), full duplex.
     * Use 3429 baud, 33600 bps as maximum; V.34 will negotiate down
     * during training based on line conditions.
     * Note: 33600 bps requires 3429 baud (3200 baud max is 31200 bps).
     */
    g_v34 = v34_init(NULL,
                     2800,          /* baud rate */
                     9600,         /* bit rate */
                     false,         /* answerer */
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

    fprintf(stderr, "[ME] V.34 training started (answerer, 3429 baud, up to 33600 bps)\n");
}

static void v8_result_handler(void *user_data, v8_parms_t *result)
{
    (void)user_data;

    fprintf(stderr, "[ME] V.8 result: status=%d, modulations=0x%X pstn_access=0x%X\n",
            result->status, result->jm_cm.modulations, result->jm_cm.pstn_access);

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

    if (result->jm_cm.modulations & V8_MOD_V34) {
        fprintf(stderr, "[ME] V.8 negotiated V.34 (full duplex, up to 33.6 kbps)\n");
        start_v34_training();

    } else if (result->jm_cm.modulations & V8_MOD_V22) {
        fprintf(stderr, "[ME] V.8 negotiated V.22bis fallback\n");
        start_v22bis_training();

    } else {
        fprintf(stderr, "[ME] V.8 no usable modulation (0x%X), hanging up\n",
                result->jm_cm.modulations);
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
    g_state = ME_IDLE;
    g_mod   = ME_MOD_NONE;
}

void me_destroy(void)
{
    if (g_v8)     { v8_free(g_v8);         g_v8     = NULL; }
    if (g_v22bis) { v22bis_free(g_v22bis); g_v22bis = NULL; }
    if (g_v34)    { v34_free(g_v34);       g_v34    = NULL; }
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
    g_v8_rx_energy     = 0;
    g_v8_rx_count      = 0;

    /* Initialise V.8 — we are the answering side (not calling party) */
    v8_parms_t v8_parms;
    memset(&v8_parms, 0, sizeof(v8_parms));
    v8_parms.modem_connect_tone = MODEM_CONNECT_TONES_ANSAM_PR; /* ANSam */
    v8_parms.send_ci            = false; /* answerer doesn't send CI */
    v8_parms.v92                = true;
    v8_parms.jm_cm.call_function      = V8_CALL_V_SERIES;
    /* Advertise V.34 + V.22bis. We don't advertise V.90 yet because our
     * training uses standard V.34 phases (not V.90-modified Phase 3/4). */
    v8_parms.jm_cm.modulations        = V8_MOD_V34 | V8_MOD_V22;
    v8_parms.jm_cm.protocols          = V8_PROTOCOL_LAPM_V42;
    v8_parms.jm_cm.pstn_access        = V8_PSTN_ACCESS_DCE_ON_DIGITAL;
    v8_parms.jm_cm.pcm_modem_availability = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;

    if (g_v8) { v8_free(g_v8); g_v8 = NULL; }
    g_v8 = v8_init(NULL, false /* answerer */, &v8_parms,
                   v8_result_handler, NULL);
    if (!g_v8) {
        fprintf(stderr, "[ME] v8_init failed\n");
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }

    g_state = ME_V8;
    g_mod   = ME_MOD_NONE;
    pthread_mutex_unlock(&g_state_mtx);

    fprintf(stderr, "[ME] SIP connected, starting V.8 handshake\n");
}

/* Called by sip_modem.c when the SIP call is torn down */
void me_on_sip_disconnected(void)
{
    pthread_mutex_lock(&g_state_mtx);

    if (g_v8)     { v8_free(g_v8);         g_v8     = NULL; }
    if (g_v22bis) { v22bis_free(g_v22bis); g_v22bis = NULL; }
    if (g_v34)    { v34_free(g_v34);       g_v34    = NULL; }

    me_state_t prev = g_state;
    g_state = ME_IDLE;
    g_mod   = ME_MOD_NONE;
    pthread_mutex_unlock(&g_state_mtx);

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
                fprintf(stderr, "[ME] Training rx: RMS=%.1f (%d samples)\n",
                        rms, g_training_rx_count);
                g_training_rx_energy = 0;
                g_training_rx_count  = 0;
            }
        }
        /* Feed audio to the appropriate modem receiver */
        if (g_mod == ME_MOD_V34 && g_v34)
            v34_rx(g_v34, amp, len);
        else if (g_v22bis)
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
        if (g_mod == ME_MOD_V34 && g_v34)
            v34_tx(g_v34, amp, len);
        else if (g_v22bis)
            v22bis_tx(g_v22bis, amp, len);
        break;

    case ME_DATA:
        if (g_mod == ME_MOD_V34 && g_v34) {
            /* V.34 full duplex data TX */
            v34_tx(g_v34, amp, len);
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
