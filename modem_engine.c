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
#include "data_stack.h"
#include "clock_recovery.h"
#include "v90.h"
#include "v91.h"
#include "v90_cp_live.h"
#include "v90_cp_rx.h"
#include "v92_cp_rx.h"
#include "v92_trn2u.h"

#include <spandsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
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
    V34_TX_STAGE_V90_WAIT_TONE_A,
    V34_TX_STAGE_V90_WAIT_INFO1A,
    V34_TX_STAGE_V90_WAIT_RX_L2,
    V34_TX_STAGE_V90_WAIT_TONE_A_REV,
    V34_TX_STAGE_V90_B_REV_DELAY,
    V34_TX_STAGE_V90_B_REV_10MS,
    V34_TX_STAGE_V90_PHASE2_B,
    V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN,
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

/* MUST stay in sync, value for value, with enum v34_events_e in
 * spandsp-master/src/spandsp/private/v34.h -- these are compared against
 * values produced inside spandsp, so any drift silently misroutes events
 * rather than failing to build. Append only, and append in both places. */
enum v34_events_e {
    V34_EVENT_NONE = 0,
    V34_EVENT_TONE_SEEN,
    V34_EVENT_REVERSAL_1,
    V34_EVENT_REVERSAL_2,
    V34_EVENT_REVERSAL_3,
    V34_EVENT_INFO0_OK,
    V34_EVENT_INFO0_BAD,
    V34_EVENT_INFO1_OK,
    V34_EVENT_INFO1_BAD,
    V34_EVENT_INFOH_OK,
    V34_EVENT_INFOH_BAD,
    V34_EVENT_L2_SEEN,
    V34_EVENT_S,
    V34_EVENT_J,
    V34_EVENT_J_DASHED,
    V34_EVENT_PHASE4_TRN_READY,
    V34_EVENT_TRAINING_FAILED,
    V34_EVENT_E,
    V34_EVENT_PEER_RETRAIN,
};

/* Current G.711 law (set by sip_modem.c after codec negotiation). */
static me_law_t g_law = ME_LAW_ULAW;

#define V90_DATA_FRAME_LEN 6
#define V90_RATE_BPS       56000

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

static inline uint8_t linear_to_pcm(int16_t sample)
{
    if (g_law == ME_LAW_ALAW)
        return linear_to_alaw(sample);
    return linear_to_ulaw(sample);
}

/*
 * Return the idle (silence) codeword for the current law.
 */
static inline uint8_t pcm_idle(void)
{
    return v90_idle_codeword(g_law == ME_LAW_ALAW ? V90_LAW_ALAW : V90_LAW_ULAW);
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

/* ------------------------------------------------------------------ */
/* V.22bis get_bit / put_bit callbacks for SpanDSP                    */
/* ------------------------------------------------------------------ */

static data_ring_t downstream_ring; /* data → modem → SIP (downstream TX) */
static data_ring_t upstream_ring;   /* SIP → modem → data (upstream RX) */
static data_stack_t g_data_stack;
static ds_framing_t g_data_framing = DS_FRAMING_V14;
static bool g_data_lapm_detect = true;
static int g_data_connect_rate = 0;
static bool g_data_connect_reported = false;
static volatile bool g_data_link_failed = false;
static void on_training_complete(me_modulation_t mod, int rate, const char *name);
static void v8_result_handler(void *user_data, v8_parms_t *result);

/* ------------------------------------------------------------------ */
/* Phase trace helpers                                                 */
/* ------------------------------------------------------------------ */

static uint64_t g_trace_start_ms = 0;
static int      g_me_verbose = -1;  /* -1 = unset; 0 = quiet; 1 = verbose */

static bool me_verbose_enabled(void)
{
    if (g_me_verbose < 0)
        g_me_verbose = (getenv("VPCM_ME_VERBOSE") != NULL) ? 1 : 0;
    return g_me_verbose != 0;
}

/* Call from sip_modem.c after argument parsing to force verbose on */
void me_set_verbose(int v) { g_me_verbose = v ? 1 : 0; }

#define ME_LOG(fmt, ...) \
    do { if (me_verbose_enabled()) fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)

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
    case ME_MOD_V91:    return "V91";
    case ME_MOD_V90:    return "V90";
    case ME_MOD_V34:    return "V34";
    case ME_MOD_V22BIS: return "V22BIS";
    default:            return "UNKNOWN";
    }
}

const char *me_state_to_str(me_state_t state)
{
    switch (state) {
    case ME_IDLE:     return "IDLE";
    case ME_DIALING:  return "DIALING";
    case ME_V8:       return "V8";
    case ME_TRAINING: return "TRAINING";
    case ME_DATA:     return "DATA";
    case ME_HANGUP:   return "HANGUP";
    default:          return "UNKNOWN";
    }
}

const char *me_modulation_to_str(me_modulation_t modulation)
{
    return me_mod_to_str(modulation);
}

const char *me_law_to_str(me_law_t law)
{
    switch (law) {
    case ME_LAW_ULAW: return "ULAW";
    case ME_LAW_ALAW: return "ALAW";
    default:          return "UNKNOWN";
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

    if (!me_verbose_enabled())
        return;

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

static int parse_v8_answer_tone_env(const char *name, int fallback)
{
    const char *s = getenv(name);

    if (!s || !*s)
        return fallback;
    if (strcmp(s, "ansam") == 0 || strcmp(s, "ANSAM") == 0)
        return MODEM_CONNECT_TONES_ANSAM;
    if (strcmp(s, "ansam_pr") == 0 || strcmp(s, "ANSAM_PR") == 0 ||
        strcmp(s, "ansam-pr") == 0 || strcmp(s, "ANSAM-PR") == 0 ||
        strcmp(s, "ansam/") == 0 || strcmp(s, "ANSAM/") == 0)
        return MODEM_CONNECT_TONES_ANSAM_PR;

    fprintf(stderr,
            "[ME] Ignoring invalid %s=%s (expected ansam or ansam_pr)\n",
            name, s);
    return fallback;
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
    case V34_TX_STAGE_V90_WAIT_TONE_A:             return "V90_WAIT_TONE_A";
    case V34_TX_STAGE_V90_WAIT_INFO1A:             return "V90_WAIT_INFO1A";
    case V34_TX_STAGE_V90_WAIT_RX_L2:              return "V90_WAIT_RX_L2";
    case V34_TX_STAGE_V90_WAIT_TONE_A_REV:         return "V90_WAIT_TONE_A_REV";
    case V34_TX_STAGE_V90_B_REV_DELAY:             return "V90_B_REV_DELAY";
    case V34_TX_STAGE_V90_B_REV_10MS:              return "V90_B_REV_10MS";
    case V34_TX_STAGE_V90_PHASE2_B:                return "V90_PHASE2_B";
    case V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN:     return "V90_PHASE2_B_INFO0_SEEN";
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

static int data_stack_pull_dte_byte(void *user_data)
{
    uint8_t byte;

    (void)user_data;
    if (dring_read(&downstream_ring, &byte, 1) != 1)
        return -1;
    return byte;
}

static void data_stack_push_dte_byte(void *user_data, uint8_t byte)
{
    (void)user_data;
    if (dring_write(&upstream_ring, &byte, 1) != 1)
        ME_LOG("[ME] DTE RX ring overrun; byte discarded\n");
}

static void data_stack_link_event(void *user_data, ds_link_event_t event)
{
    (void)user_data;
    switch (event) {
    case DS_LINK_DETECTING:
        ME_LOG("[ME] V.42 detection started\n");
        break;
    case DS_LINK_XID_NEGOTIATED:
        ME_LOG("[ME] V.42 XID negotiated\n");
        break;
    case DS_LINK_CONNECTED:
        ME_LOG("[ME] V.42 LAPM connected\n");
        if (!g_data_connect_reported) {
            g_data_connect_reported = true;
            di_on_connected(g_data_connect_rate);
        }
        break;
    case DS_LINK_UNSUPPORTED:
        ME_LOG("[ME] V.42 detection reported unsupported peer\n");
        g_data_link_failed = true;
        break;
    case DS_LINK_DISCONNECTED:
        ME_LOG("[ME] V.42 LAPM disconnected\n");
        g_data_link_failed = true;
        break;
    case DS_LINK_ERROR:
        ME_LOG("[ME] V.42 LAPM link error\n");
        g_data_link_failed = true;
        break;
    }
}

/* Install a harmless framing source while the physical modem trains. */
static void data_stack_prepare(int bit_rate)
{
    ds_framing_t framing = (g_data_framing == DS_FRAMING_V42)
                         ? DS_FRAMING_V14 : g_data_framing;

    ds_release(&g_data_stack);
    ds_init(&g_data_stack,
            framing,
            data_stack_pull_dte_byte, NULL,
            data_stack_push_dte_byte, NULL);
    ds_set_v14_rates(&g_data_stack, bit_rate, bit_rate);
}

/* Start the selected link protocol once the datapump data clock is stable. */
static int data_stack_start_online(int bit_rate, bool calling_party)
{
    int result = 0;

    ds_release(&g_data_stack);
    g_data_connect_rate = bit_rate;
    g_data_connect_reported = false;
    g_data_link_failed = false;
    if (g_data_framing == DS_FRAMING_V42) {
        result = ds_init_v42(&g_data_stack,
                             calling_party,
                             g_data_lapm_detect,
                             bit_rate,
                             data_stack_pull_dte_byte, NULL,
                             data_stack_push_dte_byte, NULL,
                             data_stack_link_event, NULL);
        if (result != 0)
            g_data_link_failed = true;
    } else {
        ds_init(&g_data_stack,
                g_data_framing,
                data_stack_pull_dte_byte, NULL,
                data_stack_push_dte_byte, NULL);
        ds_set_v14_rates(&g_data_stack, bit_rate, bit_rate);
    }
    return result;
}

static int v22bis_get_bit_cb(void *user_data)
{
    int bit;

    (void)user_data;
    bit = ds_tx_get_bit(&g_data_stack);
    return (bit == DS_TX_NO_DATA) ? SIG_STATUS_END_OF_DATA : bit;
}

static void v22bis_put_bit_cb(void *user_data, int bit)
{
    (void)user_data;
    if (bit < 0) {
        if (bit == SIG_STATUS_CARRIER_UP || bit == SIG_STATUS_TRAINING_SUCCEEDED)
            on_training_complete(ME_MOD_V22BIS, 2400, "V.22bis");
        else if (bit == SIG_STATUS_TRAINING_FAILED || bit == SIG_STATUS_CARRIER_DOWN) {
            ME_LOG("[ME] V.22bis fallback failed (%s), hanging up\n",
                    signal_status_to_str(bit));
            trace_phase("V22BIS training failed (%s) -> hangup",
                        signal_status_to_str(bit));
            me_hangup();
            return;
        }
        return;
    }

    ds_rx_put_bit(&g_data_stack, bit);
}

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static me_state_t      g_state     = ME_IDLE;
static me_modulation_t g_mod       = ME_MOD_NONE;
static pthread_mutex_t g_state_mtx;
static bool            g_calling_party = false; /* false=answerer, true=caller */
static bool            g_invert_v34_role = false; /* debug override via env */
static int             g_v8_answer_tone = MODEM_CONNECT_TONES_ANSAM;
static int             g_v8_active_answer_tone = MODEM_CONNECT_TONES_ANSAM;
static bool            g_v8_answer_tone_retry_done = false;

/* SpanDSP modem contexts */
static v8_state_t     *g_v8      = NULL;
static v22bis_state_t *g_v22bis  = NULL;
static v34_state_t    *g_v34     = NULL;

/* V.90 state (Phase 3/4 TX and data mode) */
static v90_state_t   *g_v90     = NULL;
static bool           g_v90_phase3_started = false;
static bool           g_v90_completion_deferred_logged = false;
static bool           g_v90_wait_info1_logged = false;
static bool           g_v90_fallback_v34_logged = false;
static bool           g_v90_fallback_phase4_released = false;
static int            g_v90_phase3_s_events = 0;
static unsigned        g_v90_phase2_restarts = 0;
static v90_cp_rx_t    g_v90_cp_rx;
static bool           g_v92_active = false;
static bool           g_v92_trn2u_active = false;
static int            g_v92_trn2u_points = 4;
static double         g_v92_trn2u_lu = 8000.0;
static v92_cp_rx_t    g_v92_cp_rx;
static v92_trn2u_demod_t g_v92_trn2u_demod;
static uint8_t        g_v90_data_frame[V90_DATA_FRAME_LEN];
static int            g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
static bool           g_v34_fallback_to_v22bis_pending = false;
static int            g_v34_fallback_status = 0;
static int            g_last_v90_bridge_rx_stage = -1;
static int            g_last_v90_bridge_tx_stage = -1;
static int            g_last_v90_bridge_rx_event = -1;
static v90_dil_desc_t g_v90_pending_dil;
static bool           g_v90_pending_dil_valid = false;
static uint64_t       g_phase_start_ms = 0;

/* Independent strict V.90 CP receiver.  SpanDSP remains the primary live
 * path; this worker snapshots the unfiltered upstream PCM and runs the same
 * CRC-selected front end that recovered the hardware capture offline.  It
 * never runs on the PJSIP media callback. */
#define V90_CP_LIVE_MAX_SAMPLES (40 * 8000)
static pthread_mutex_t g_v90_cp_live_mtx;
static pthread_cond_t  g_v90_cp_live_cond;
static pthread_t       g_v90_cp_live_thread;
static bool            g_v90_cp_live_thread_started = false;
static bool            g_v90_cp_live_shutdown = false;
static bool            g_v90_cp_live_pending = false;
static bool            g_v90_cp_live_running = false;
static int16_t         g_v90_cp_live_samples[V90_CP_LIVE_MAX_SAMPLES];
static int             g_v90_cp_live_sample_count = 0;
static int             g_v90_cp_live_phase4_hint = -1;
static int             g_v90_cp_live_next_request = -1;
static int             g_v90_cp_live_expected_compatibility = 0;
static unsigned        g_v90_cp_live_generation = 0;

/* V.91 symmetric raw-G.711 startup and data path.
 *
 * TX streams a fixed script through the DIL, then repeats SCR until the
 * local rate decision is available (absorbing startup skew between the
 * two directions), then sends CP/Es/B1 built from that decision.
 * RX is a staged parser: it decodes the peer's INFO frames, receives the
 * DIL at the codeword level (which performs robbed-bit detection), finds
 * the CP inside the peer's variable-length SCR run via the descrambled
 * ones-run, and verifies Es/B1.  The final rate is the minimum of both
 * sides' selections. */
#define V91_LIVE_STARTUP_MAX 4096
#define V91_LIVE_DATA_FRAMES 8
#define V91_LIVE_DATA_CODEWORDS (V91_LIVE_DATA_FRAMES * VPCM_CP_FRAME_INTERVALS)
#define V91_LIVE_SCR_FIRST_SYMBOLS 18
#define V91_LIVE_SCR_MAX_SYMBOLS 24000
typedef enum {
    V91_LIVE_TX_SCRIPT = 0,
    V91_LIVE_TX_SCR,
    V91_LIVE_TX_TAIL,
    V91_LIVE_TX_DONE
} v91_live_tx_stage_t;
typedef enum {
    V91_LIVE_RX_HUNT_EZ = 0,
    V91_LIVE_RX_INFO,
    V91_LIVE_RX_INFO_ACK,
    V91_LIVE_RX_EU,
    V91_LIVE_RX_DIL,
    V91_LIVE_RX_SCR_CP,
    V91_LIVE_RX_ES,
    V91_LIVE_RX_B1,
    V91_LIVE_RX_DONE
} v91_live_rx_stage_t;
static v91_state_t g_v91_tx;
static v91_state_t g_v91_rx;
static vpcm_cp_frame_t g_v91_cp_template;
static vpcm_cp_frame_t g_v91_cp_ack;
static vpcm_cp_frame_t g_v91_peer_cp;
static v91_dil_desc_t g_v91_default_dil;
static uint8_t g_v91_local_drn;
static uint8_t g_v91_peer_drn;
static uint8_t g_v91_startup_tx[V91_LIVE_STARTUP_MAX];
static int g_v91_startup_tx_len;
static int g_v91_startup_tx_pos;
static v91_live_tx_stage_t g_v91_tx_stage;
static int g_v91_scr_sent;
static uint8_t g_v91_rx_accum[V91_LIVE_STARTUP_MAX];
static int g_v91_rx_accum_len;
static v91_live_rx_stage_t g_v91_rx_stage;
static uint8_t g_v91_cp_bits[VPCM_CP_MAX_BITS];
static int g_v91_cp_bit_pos;
static int g_v91_cp_total_bits;
static bool g_v91_cp_started;
static int g_v91_scr_ones_run;
static int g_v91_es_bits;
static int g_v91_ez_run;
static uint8_t g_v91_data_tx[V91_LIVE_DATA_CODEWORDS];
static int g_v91_data_tx_pos = V91_LIVE_DATA_CODEWORDS;
static uint8_t g_v91_data_rx[V91_LIVE_DATA_CODEWORDS];
static int g_v91_data_rx_pos;
static int g_v91_data_bytes;
static bool g_v91_data_tx_guard;
static bool g_v91_data_rx_synced;
/* Robbed-cadence frame alignment.  A robbed-bit trunk steals the LSB of
 * every sixth codeword at a fixed cadence; the data mapper protects only
 * frame interval 5 (odd Ucodes), so each transmitter must align its data
 * framing to the cadence its receiver observed.  The receiver measures
 * the cadence during DIL reception, converts it to peer-stream
 * coordinates (the DIL always starts at position 760 of the peer's
 * startup script), and requests alignment through the low bits of the
 * CP upstream_rate_mask field (0 = none, 1..6 = phase + 1). */
#define V91_LIVE_DIL_PEER_START 760
static long long g_v91_tx_abs_pos;
static long long g_v91_rx_abs_pos;
static long long g_v91_rx_dil_start_abs;
static int g_v91_rx_robbed_phase;   /* robbed cadence on our RX path, -1 = none */
static int g_v91_tx_align_phase;    /* peer-requested cadence for our TX, -1 = none */
static bool g_v91_data_tx_aligned;
static int g_v91_sim_robbed_phase;  /* env V91_SIM_ROBBED_RX, -1 = off */

static int v91_live_append(uint8_t *dst, int cap, int pos,
                           const uint8_t *src, int len)
{
    if (!dst || !src || pos < 0 || len < 0 || pos + len > cap)
        return -1;
    memcpy(dst + pos, src, (size_t)len);
    return pos + len;
}

/* Fixed part of the startup script: everything through the DIL.  SCR and
 * the rate-dependent CP/Es/B1 tail are generated later, once the received
 * DIL has been analysed and the local rate selected. */
static int v91_live_build_startup(v91_law_t law,
                                  v91_state_t *state,
                                  uint8_t *out,
                                  int out_cap)
{
    v91_info_frame_t info;
    uint8_t tmp[2048];
    int pos = 0;
    int len;

    v91_init(state, law, V91_MODE_TRANSPARENT);
    memset(&info, 0, sizeof(info));
    info.request_default_dil = true;
    info.tx_uses_alaw = (law == V91_LAW_ALAW);
    info.power_measured_after_digital_impairments = true;

#define V91_APPEND_EXPR(expr) do { \
        len = (expr); \
        if (len <= 0 || (pos = v91_live_append(out, out_cap, pos, tmp, len)) < 0) \
            return 0; \
    } while (0)
    V91_APPEND_EXPR(v91_tx_phase1_silence_codewords(state, tmp, sizeof(tmp)));
    V91_APPEND_EXPR(v91_tx_ez_codewords(state, tmp, sizeof(tmp)));
    V91_APPEND_EXPR(v91_tx_info_codewords(state, tmp, sizeof(tmp), &info));
    info.acknowledge_info_frame = true;
    V91_APPEND_EXPR(v91_tx_info_codewords(state, tmp, sizeof(tmp), &info));
    V91_APPEND_EXPR(v91_tx_eu_codewords(state, tmp, sizeof(tmp)));
    V91_APPEND_EXPR(v91_tx_dil_codewords(state, tmp, sizeof(tmp), &g_v91_default_dil));
#undef V91_APPEND_EXPR
    return pos;
}

/* Rate-dependent tail: CP with the locally selected drn, then Es and B1. */
static int v91_live_build_tail(bool calling_party,
                               v91_state_t *state,
                               uint8_t drn,
                               uint8_t *out,
                               int out_cap)
{
    vpcm_cp_frame_t cp;
    uint8_t tmp[2048];
    int pos = 0;
    int len;

    cp = g_v91_cp_template;
    cp.drn = drn;
    cp.acknowledge = calling_party ? false : true;
    /* Ask the peer to align its data framing to the robbed cadence we
     * observed on our receive path (in peer-stream coordinates). */
    if (g_v91_rx.rx_robbed_bit_detected && g_v91_rx.rx_robbed_slot_mask != 0) {
        int slot = 0;

        while (slot < VPCM_CP_FRAME_INTERVALS
               && !(g_v91_rx.rx_robbed_slot_mask & (1U << slot)))
            slot++;
        cp.upstream_rate_mask =
            (uint16_t)(((V91_LIVE_DIL_PEER_START + slot) % 6) + 1);
    }
#define V91_APPEND_EXPR(expr) do { \
        len = (expr); \
        if (len <= 0 || (pos = v91_live_append(out, out_cap, pos, tmp, len)) < 0) \
            return 0; \
    } while (0)
    V91_APPEND_EXPR(v91_tx_cp_codewords(state, tmp, sizeof(tmp), &cp, true));
    V91_APPEND_EXPR(v91_tx_es_codewords(state, tmp, sizeof(tmp)));
    V91_APPEND_EXPR(v91_tx_b1_codewords(state, tmp, sizeof(tmp), &cp));
#undef V91_APPEND_EXPR
    return pos;
}

static void v91_live_reset(void)
{
    memset(&g_v91_tx, 0, sizeof(g_v91_tx));
    memset(&g_v91_rx, 0, sizeof(g_v91_rx));
    g_v91_startup_tx_len = 0;
    g_v91_startup_tx_pos = 0;
    g_v91_tx_stage = V91_LIVE_TX_SCRIPT;
    g_v91_scr_sent = 0;
    g_v91_rx_accum_len = 0;
    g_v91_rx_stage = V91_LIVE_RX_HUNT_EZ;
    g_v91_cp_bit_pos = 0;
    g_v91_cp_total_bits = 0;
    g_v91_cp_started = false;
    g_v91_scr_ones_run = 0;
    g_v91_es_bits = 0;
    g_v91_local_drn = 0;
    g_v91_peer_drn = 0;
    g_v91_ez_run = 0;
    g_v91_data_tx_pos = V91_LIVE_DATA_CODEWORDS;
    g_v91_data_rx_pos = 0;
    g_v91_data_bytes = 0;
    g_v91_data_tx_guard = false;
    g_v91_data_rx_synced = false;
    g_v91_tx_abs_pos = 0;
    g_v91_rx_abs_pos = 0;
    g_v91_rx_dil_start_abs = 0;
    g_v91_rx_robbed_phase = -1;
    g_v91_tx_align_phase = -1;
    g_v91_data_tx_aligned = false;
    g_v91_sim_robbed_phase = -1;
}

static bool v91_live_start_locked(void)
{
    v91_law_t law = (g_law == ME_LAW_ALAW) ? V91_LAW_ALAW : V91_LAW_ULAW;
    const char *sim_env;

    v91_live_reset();
    /* Test hook: simulate a robbed-bit trunk on our receive path by
     * clearing the LSB of every sixth incoming codeword at the given
     * cadence phase (0..5). */
    sim_env = getenv("V91_SIM_ROBBED_RX");
    if (sim_env && *sim_env) {
        int phase = atoi(sim_env);

        if (phase >= 0 && phase <= 5)
            g_v91_sim_robbed_phase = phase;
    }
    /* Offer ceiling with a robbed-bit-safe constellation set; the actual
     * rate is selected from the received DIL analysis and robbed-bit
     * detection once the peer's DIL has arrived. */
    vpcm_cp_init_robbed_bit_safe_profile(&g_v91_cp_template, 28, false);
    g_v91_cp_template.codec_alaw = (law == V91_LAW_ALAW);
    g_v91_cp_total_bits = vpcm_cp_bit_length(&g_v91_cp_template);
    v91_default_dil_init(&g_v91_default_dil);
    g_v91_startup_tx_len = v91_live_build_startup(law,
                                                  &g_v91_tx,
                                                  g_v91_startup_tx,
                                                  sizeof(g_v91_startup_tx));
    v91_init(&g_v91_rx, law, V91_MODE_TRANSPARENT);
    if (g_v91_startup_tx_len <= V91_PHASE1_SILENCE_SYMBOLS
        || g_v91_cp_total_bits <= 0
        || g_v91_cp_total_bits > VPCM_CP_MAX_BITS)
        return false;
    data_stack_prepare((int)vpcm_cp_drn_to_bps(vpcm_cp_recommended_robbed_bit_drn()));
    g_mod = ME_MOD_V91;
    g_state = ME_TRAINING;
    g_phase_start_ms = trace_now_ms();
    trace_phase("enter TRAINING: mod=V91 role=%s startup_tx=%d ceiling_drn=%u",
                g_calling_party ? "caller" : "answerer",
                g_v91_startup_tx_len, g_v91_cp_template.drn);
    return true;
}

static void v91_live_try_enter_data_locked(void)
{
    uint8_t final_drn;
    int rate;

    if (g_state != ME_TRAINING || g_mod != ME_MOD_V91
        || g_v91_tx_stage != V91_LIVE_TX_DONE
        || g_v91_startup_tx_pos < g_v91_startup_tx_len
        || g_v91_rx_stage != V91_LIVE_RX_DONE)
        return;
    /* Both sides computed a selection from their own received DIL; the
     * connection runs at the smaller of the two. */
    final_drn = (g_v91_peer_drn < g_v91_local_drn) ? g_v91_peer_drn : g_v91_local_drn;
    if (final_drn == 0) {
        g_state = ME_HANGUP;
        return;
    }
    g_v91_cp_ack = g_v91_cp_template;
    g_v91_cp_ack.drn = final_drn;
    g_v91_cp_ack.acknowledge = true;
    if (!v91_activate_data_mode(&g_v91_tx, &g_v91_cp_ack)
        || !v91_activate_data_mode(&g_v91_rx, &g_v91_cp_ack)) {
        g_state = ME_HANGUP;
        return;
    }
    /* 8 frames per data block keep the byte count exact for every drn. */
    g_v91_data_bytes = (int)final_drn + 20;
    rate = (int)vpcm_cp_drn_to_bps(final_drn);
    data_stack_start_online(rate, g_calling_party);
    g_state = ME_DATA;
    g_phase_start_ms = 0;
    g_v91_data_tx_pos = V91_LIVE_DATA_CODEWORDS;
    g_v91_data_rx_pos = 0;
    g_v91_data_tx_guard = true;
    /* The peer's B1 tail and explicit guard can end at arbitrary RTP-frame
     * offsets.  The receiver acquires the mapper boundary at the first
     * non-idle primary-channel codeword instead of guessing packet counts. */
    g_v91_data_rx_synced = false;
    g_data_connect_reported = true;
    di_on_connected(rate);
    trace_phase("V91 enter DATA: rate=%d (local drn=%u peer drn=%u robbed=%d rx_phase=%d tx_align=%d)",
                rate, g_v91_local_drn, g_v91_peer_drn,
                g_v91_rx.rx_robbed_bit_detected ? 1 : 0,
                g_v91_rx_robbed_phase, g_v91_tx_align_phase);
}

/* Refill the TX startup queue when it drains.  Returns false when the
 * current stage is finished and nothing further can be produced yet. */
static bool v91_live_tx_refill_locked(void)
{
    int len;

    switch (g_v91_tx_stage) {
    case V91_LIVE_TX_SCRIPT:
        /* Fixed script drained: start the SCR run (GPC reset). */
        len = v91_tx_scr_codewords(&g_v91_tx, g_v91_startup_tx,
                                   (int)sizeof(g_v91_startup_tx),
                                   V91_LIVE_SCR_FIRST_SYMBOLS);
        if (len != V91_LIVE_SCR_FIRST_SYMBOLS)
            return false;
        g_v91_startup_tx_len = len;
        g_v91_startup_tx_pos = 0;
        g_v91_scr_sent = len;
        g_v91_tx_stage = V91_LIVE_TX_SCR;
        return true;
    case V91_LIVE_TX_SCR:
        if (g_v91_local_drn > 0) {
            /* Rate decided from the received DIL: send CP/Es/B1. */
            len = v91_live_build_tail(g_calling_party, &g_v91_tx,
                                      g_v91_local_drn,
                                      g_v91_startup_tx,
                                      (int)sizeof(g_v91_startup_tx));
            if (len <= 0)
                return false;
            g_v91_startup_tx_len = len;
            g_v91_startup_tx_pos = 0;
            g_v91_tx_stage = V91_LIVE_TX_TAIL;
            return true;
        }
        if (g_v91_scr_sent >= V91_LIVE_SCR_MAX_SYMBOLS)
            return false;
        /* Keep the line filled with SCR (no GPC reset) while the peer's
         * DIL is still arriving. */
        len = v91_tx_phil_codewords(&g_v91_tx, g_v91_startup_tx,
                                    (int)sizeof(g_v91_startup_tx),
                                    VPCM_CP_FRAME_INTERVALS, true);
        if (len != VPCM_CP_FRAME_INTERVALS)
            return false;
        g_v91_startup_tx_len = len;
        g_v91_startup_tx_pos = 0;
        g_v91_scr_sent += len;
        return true;
    case V91_LIVE_TX_TAIL:
        g_v91_tx_stage = V91_LIVE_TX_DONE;
        return false;
    case V91_LIVE_TX_DONE:
    default:
        return false;
    }
}

static bool v91_live_generate_codewords_locked(uint8_t *out, int len)
{
    long long base;
    int pos = 0;

    if (!out || len <= 0 || g_mod != ME_MOD_V91)
        return false;
    base = g_v91_tx_abs_pos;
    g_v91_tx_abs_pos = base + len;
    memset(out, v91_idle_codeword(g_v91_tx.law), (size_t)len);
    if (g_state == ME_TRAINING) {
        while (pos < len) {
            int remain = g_v91_startup_tx_len - g_v91_startup_tx_pos;
            int copy;

            if (remain <= 0) {
                if (!v91_live_tx_refill_locked()) {
                    if (g_v91_tx_stage == V91_LIVE_TX_SCR) {
                        ME_LOG("[ME] V.91 SCR fill exhausted waiting for peer DIL\n");
                        g_state = ME_HANGUP;
                        return false;
                    }
                    break;
                }
                remain = g_v91_startup_tx_len - g_v91_startup_tx_pos;
            }
            copy = remain < (len - pos) ? remain : (len - pos);
            memcpy(out + pos, g_v91_startup_tx + g_v91_startup_tx_pos, (size_t)copy);
            g_v91_startup_tx_pos += copy;
            pos += copy;
        }
        v91_live_try_enter_data_locked();
        /* Do not mix B1 and primary-channel data in one RTP packet.  Starting
         * data on the next pull gives both receivers an unambiguous mapper
         * boundary after discarding the peer's final startup packet. */
        return true;
    }
    if (g_state == ME_DATA && g_v91_data_tx_guard) {
        g_v91_data_tx_guard = false;
        return true;
    }
    if (!g_v91_data_tx_aligned) {
        if (g_v91_tx_align_phase >= 0) {
            /* Pad with idle so the first data codeword (frame interval 0)
             * puts interval 5 on the robbed cadence the peer reported. */
            int want = (g_v91_tx_align_phase + 1) % 6;
            int pad = (int)((want - (base + pos) % 6 + 6) % 6);

            if (pad > len - pos)
                return true;
            pos += pad;
        }
        g_v91_data_tx_aligned = true;
        if (g_v91_tx_align_phase >= 0)
            trace_phase("V91 data TX aligned to robbed cadence (phase=%d)",
                        g_v91_tx_align_phase);
    }
    while (pos < len) {
        int available;
        int copy;

        if (g_v91_data_tx_pos >= V91_LIVE_DATA_CODEWORDS) {
            uint8_t data[V91_LIVE_DATA_CODEWORDS];

            if (g_v91_data_bytes <= 0 || g_v91_data_bytes > (int)sizeof(data))
                return false;
            ds_tx_fill_bytes(&g_data_stack, data, g_v91_data_bytes);
            if (v91_tx_codewords(&g_v91_tx,
                                 g_v91_data_tx,
                                 sizeof(g_v91_data_tx),
                                 data,
                                 g_v91_data_bytes) != V91_LIVE_DATA_CODEWORDS)
                return false;
            g_v91_data_tx_pos = 0;
        }
        available = V91_LIVE_DATA_CODEWORDS - g_v91_data_tx_pos;
        copy = available < (len - pos) ? available : (len - pos);
        memcpy(out + pos, g_v91_data_tx + g_v91_data_tx_pos, (size_t)copy);
        g_v91_data_tx_pos += copy;
        pos += copy;
    }
    return true;
}

static void v91_live_rx_fail_locked(const char *what)
{
    ME_LOG("[ME] V.91 startup receive failed at %s\n", what);
    v91_request_retrain(&g_v91_rx);
    g_state = ME_HANGUP;
}

/* Accumulate codewords for the current RX stage; true once `need` are held. */
static bool v91_live_rx_collect(uint8_t cw, int need)
{
    if (g_v91_rx_accum_len < need && g_v91_rx_accum_len < (int)sizeof(g_v91_rx_accum))
        g_v91_rx_accum[g_v91_rx_accum_len++] = cw;
    return g_v91_rx_accum_len >= need;
}

static void v91_live_rx_enter_stage(v91_live_rx_stage_t stage)
{
    g_v91_rx_stage = stage;
    g_v91_rx_accum_len = 0;
}

/* Robbed-bit trunks clear the LSB of the transported codeword. */
static bool v91_live_codeword_matches(uint8_t cw, uint8_t expected)
{
    return cw == expected || cw == (uint8_t)(expected & 0xFE);
}

static void v91_live_rx_training_codeword_locked(uint8_t cw, long long abs_pos)
{
    uint8_t ez = v91_ucode_to_codeword(g_v91_rx.law, 66, false);

    switch (g_v91_rx_stage) {
    case V91_LIVE_RX_HUNT_EZ:
        if (v91_live_codeword_matches(cw, ez))
            g_v91_ez_run++;
        else
            g_v91_ez_run = 0;
        if (g_v91_ez_run == V91_EZ_SYMBOLS) {
            trace_phase("V91 RX synchronized at Ez");
            v91_live_rx_enter_stage(V91_LIVE_RX_INFO);
        }
        return;
    case V91_LIVE_RX_INFO:
    case V91_LIVE_RX_INFO_ACK: {
        v91_info_frame_t info;
        bool want_ack = (g_v91_rx_stage == V91_LIVE_RX_INFO_ACK);

        if (!v91_live_rx_collect(cw, V91_INFO_SYMBOLS))
            return;
        if (!v91_rx_info_codewords(&g_v91_rx, g_v91_rx_accum, V91_INFO_SYMBOLS, &info)
            || info.acknowledge_info_frame != want_ack) {
            v91_live_rx_fail_locked(want_ack ? "INFO'" : "INFO");
            return;
        }
        v91_live_rx_enter_stage(want_ack ? V91_LIVE_RX_EU : V91_LIVE_RX_INFO_ACK);
        return;
    }
    case V91_LIVE_RX_EU: {
        int ucode = v91_codeword_to_ucode(g_v91_rx.law, cw);

        /* An LSB-robbed Eu codeword decodes to the adjacent Ucode. */
        if (ucode != 66 && ucode != 67) {
            v91_live_rx_fail_locked("Eu");
            return;
        }
        if (v91_live_rx_collect(cw, V91_EU_SYMBOLS))
            v91_live_rx_enter_stage(V91_LIVE_RX_DIL);
        return;
    }
    case V91_LIVE_RX_DIL: {
        int dil_len = v91_dil_symbol_count(&g_v91_default_dil);

        if (g_v91_rx_accum_len == 0)
            g_v91_rx_dil_start_abs = abs_pos;
        if (!v91_live_rx_collect(cw, dil_len))
            return;
        if (!v91_rx_dil_codewords(&g_v91_rx, g_v91_rx_accum, dil_len,
                                  &g_v91_default_dil)) {
            v91_live_rx_fail_locked("DIL");
            return;
        }
        if (g_v91_rx.rx_robbed_bit_detected && g_v91_rx.rx_robbed_slot_mask != 0) {
            int slot = 0;

            while (slot < VPCM_CP_FRAME_INTERVALS
                   && !(g_v91_rx.rx_robbed_slot_mask & (1U << slot)))
                slot++;
            g_v91_rx_robbed_phase =
                (int)((g_v91_rx_dil_start_abs + slot) % 6);
        }
        g_v91_local_drn = v91_select_drn(&g_v91_rx, &g_v91_cp_template, false);
        if (g_v91_local_drn == 0) {
            v91_live_rx_fail_locked("rate selection");
            return;
        }
        trace_phase("V91 DIL analysed: selected drn=%u (%d bps)%s",
                    g_v91_local_drn,
                    (int)vpcm_cp_drn_to_bps(g_v91_local_drn),
                    g_v91_rx.rx_robbed_bit_detected
                        ? " robbed-bit detected" : "");
        v91_rx_diff_reset(&g_v91_rx);
        g_v91_scr_ones_run = 0;
        g_v91_cp_started = false;
        g_v91_cp_bit_pos = 0;
        v91_live_rx_enter_stage(V91_LIVE_RX_SCR_CP);
        return;
    }
    case V91_LIVE_RX_SCR_CP: {
        int bit = v91_rx_diff_scrambled_bit(&g_v91_rx, cw);

        if (!g_v91_cp_started) {
            if (bit != 0) {
                if (++g_v91_scr_ones_run > V91_LIVE_SCR_MAX_SYMBOLS + g_v91_cp_total_bits)
                    v91_live_rx_fail_locked("SCR (no CP)");
                return;
            }
            /* First zero after the SCR ones-run is CP bit 17 of the
             * 17-ones frame-sync pattern. */
            if (g_v91_scr_ones_run < 17) {
                v91_live_rx_fail_locked("CP frame sync");
                return;
            }
            memset(g_v91_cp_bits, 1, 17);
            g_v91_cp_bits[17] = 0;
            g_v91_cp_bit_pos = 18;
            g_v91_cp_started = true;
            return;
        }
        g_v91_cp_bits[g_v91_cp_bit_pos++] = (uint8_t)bit;
        if (g_v91_cp_bit_pos < g_v91_cp_total_bits)
            return;
        {
            vpcm_cp_frame_t expected;
            unsigned align_req;

            if (!vpcm_cp_decode_bits(g_v91_cp_bits, g_v91_cp_total_bits,
                                     &g_v91_peer_cp)) {
                v91_live_rx_fail_locked("CP decode");
                return;
            }
            expected = g_v91_cp_template;
            expected.drn = g_v91_peer_cp.drn;
            /* The answerer's CP carries the acknowledge flag; the
             * upstream_rate_mask field carries the peer's alignment
             * request for our transmit direction. */
            expected.acknowledge = g_calling_party;
            expected.upstream_rate_mask = g_v91_peer_cp.upstream_rate_mask;
            align_req = g_v91_peer_cp.upstream_rate_mask;
            if (g_v91_peer_cp.drn == 0
                || !v91_cp_supports_drn(&g_v91_cp_template, g_v91_peer_cp.drn)
                || align_req > 6
                || !vpcm_cp_frames_equal(&expected, &g_v91_peer_cp)) {
                v91_live_rx_fail_locked("CP contents");
                return;
            }
            g_v91_tx_align_phase = (align_req >= 1) ? (int)(align_req - 1) : -1;
            g_v91_peer_drn = g_v91_peer_cp.drn;
            trace_phase("V91 peer CP: drn=%u (%d bps) tx_align=%d",
                        g_v91_peer_drn,
                        (int)vpcm_cp_drn_to_bps(g_v91_peer_drn),
                        g_v91_tx_align_phase);
        }
        g_v91_es_bits = 0;
        v91_live_rx_enter_stage(V91_LIVE_RX_ES);
        return;
    }
    case V91_LIVE_RX_ES: {
        int bit = v91_rx_diff_scrambled_bit(&g_v91_rx, cw);

        if (bit != 0) {
            v91_live_rx_fail_locked("Es");
            return;
        }
        if (++g_v91_es_bits >= V91_ES_SYMBOLS)
            v91_live_rx_enter_stage(V91_LIVE_RX_B1);
        return;
    }
    case V91_LIVE_RX_B1: {
        v91_state_t b1_state;
        uint8_t expected[V91_B1_SYMBOLS];
        int i;

        if (!v91_live_rx_collect(cw, V91_B1_SYMBOLS))
            return;
        v91_init(&b1_state, g_v91_rx.law, V91_MODE_TRANSPARENT);
        if (v91_tx_b1_codewords(&b1_state, expected, V91_B1_SYMBOLS,
                                &g_v91_peer_cp) != V91_B1_SYMBOLS) {
            v91_live_rx_fail_locked("B1 reference");
            return;
        }
        for (i = 0; i < V91_B1_SYMBOLS; i++) {
            if (!v91_live_codeword_matches(g_v91_rx_accum[i], expected[i])) {
                v91_live_rx_fail_locked("B1");
                return;
            }
        }
        trace_phase("V91 RX startup complete");
        v91_live_rx_enter_stage(V91_LIVE_RX_DONE);
        v91_live_try_enter_data_locked();
        return;
    }
    case V91_LIVE_RX_DONE:
    default:
        return;
    }
}

static void v91_live_receive_chunk_locked(const uint8_t *in, int len, long long base)
{
    int pos = 0;

    while (pos < len && g_mod == ME_MOD_V91) {
        if (g_state == ME_TRAINING) {
            /* The peer may finish a few codewords before our final RTP pull.
             * Startup is already validated; discard the short overlap until
             * our local B1 has also been transmitted. */
            if (g_v91_rx_stage == V91_LIVE_RX_DONE) {
                pos = len;
                v91_live_try_enter_data_locked();
                continue;
            }
            v91_live_rx_training_codeword_locked(in[pos], base + pos);
            pos++;
            if (g_state == ME_HANGUP)
                return;
            if (g_v91_rx_stage == V91_LIVE_RX_DONE)
                pos = len;
            continue;
        }
        if (g_state == ME_DATA) {
            uint8_t idle = v91_idle_codeword(g_v91_rx.law);
            int copy = V91_LIVE_DATA_CODEWORDS - g_v91_data_rx_pos;

            if (!g_v91_data_rx_synced) {
                /* On a robbed path, the peer aligns its data framing so
                 * frame interval 5 sits on the robbed cadence: the first
                 * data codeword arrives at phase (robbed + 1), and a
                 * robbed idle (LSB cleared) never sits on that phase. */
                while (pos < len) {
                    if (g_v91_rx_robbed_phase >= 0) {
                        if ((int)((base + pos) % 6)
                                != (g_v91_rx_robbed_phase + 1) % 6
                            || in[pos] == idle) {
                            pos++;
                            continue;
                        }
                    } else if (in[pos] == idle) {
                        pos++;
                        continue;
                    }
                    break;
                }
                if (pos == len)
                    continue;
                g_v91_data_rx_synced = true;
                g_v91_data_rx_pos = 0;
            }

            if (copy > len - pos)
                copy = len - pos;
            memcpy(g_v91_data_rx + g_v91_data_rx_pos, in + pos, (size_t)copy);
            g_v91_data_rx_pos += copy;
            pos += copy;
            if (g_v91_data_rx_pos == V91_LIVE_DATA_CODEWORDS) {
                uint8_t data[V91_LIVE_DATA_CODEWORDS];
                int all_idle = 1;
                int decoded;

                for (int i = 0; i < V91_LIVE_DATA_CODEWORDS; i++) {
                    /* Robbed idles arrive with the LSB cleared. */
                    if (g_v91_rx_robbed_phase >= 0
                            ? !v91_live_codeword_matches(g_v91_data_rx[i], idle)
                            : g_v91_data_rx[i] != idle) {
                        all_idle = 0;
                        break;
                    }
                }
                if (all_idle) {
                    g_v91_data_rx_pos = 0;
                    continue;
                }
                decoded = v91_rx_codewords(&g_v91_rx,
                                                data,
                                                sizeof(data),
                                                g_v91_data_rx,
                                                sizeof(g_v91_data_rx));
                if (decoded != g_v91_data_bytes) {
                    ME_LOG("[ME] V.91 data frame decode failed: decoded=%d/%d codewords=",
                           decoded, g_v91_data_bytes);
                    if (me_verbose_enabled()) {
                        for (int i = 0; i < V91_LIVE_DATA_CODEWORDS; i++)
                            fprintf(stderr, "%02X%s", g_v91_data_rx[i],
                                    i + 1 == V91_LIVE_DATA_CODEWORDS ? "\n" : " ");
                    }
                    v91_note_frame_sync_loss(&g_v91_rx);
                    g_state = ME_HANGUP;
                    return;
                }
                ds_rx_push_bytes(&g_data_stack, data, decoded);
                g_v91_data_rx_pos = 0;
            }
            continue;
        }
        break;
    }
}

static void v91_live_receive_codewords_locked(const uint8_t *in, int len)
{
    uint8_t buf[512];

    while (len > 0 && g_mod == ME_MOD_V91) {
        long long base = g_v91_rx_abs_pos;
        int chunk = len < (int)sizeof(buf) ? len : (int)sizeof(buf);
        int i;

        memcpy(buf, in, (size_t)chunk);
        if (g_v91_sim_robbed_phase >= 0) {
            for (i = 0; i < chunk; i++) {
                if ((int)((base + i) % 6) == g_v91_sim_robbed_phase)
                    buf[i] &= 0xFE;
            }
        }
        g_v91_rx_abs_pos = base + chunk;
        v91_live_receive_chunk_locked(buf, chunk, base);
        in += chunk;
        len -= chunk;
    }
}
static bool           g_v90_dil_parse_logged = false;

#define V90_DIL_CAPTURE_MAX_BITS 65536
static uint8_t        g_v90_dil_capture[(V90_DIL_CAPTURE_MAX_BITS + 7) / 8];
static int            g_v90_dil_capture_bits = 0;
static int            g_v90_dil_capture_search = 0;
static int            g_v90_dil_hyp_last_bits = 0;
static bool           g_v90_dil_hyp_dumped = false;

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
#define V8_TIMEOUT_MS       15000   /* V.8 negotiation: 15 s (2003-era
                                       SmartLink DSPs need ~6 s just to
                                       validate JM before sending CJ) */
#define TRAINING_TIMEOUT_MS 60000   /* V.90 hardware may spend ~10 seconds
                                       retrying INFO before Phases 3/4. */

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

/* V.22bis guard tone (ITU-T V.22bis §2.1/2.2): an 1800 Hz (or, as a national
   option, 550 Hz) tone transmitted continuously alongside the "high
   channel" carrier, 6 dB (1800 Hz) or 3 dB (550 Hz) below the data signal
   level, to stop PSTN loading-coil relays from mis-triggering on the
   carrier. spandsp's own V.8/V.34 code never generates it (v34tx.c
   explicitly disables it, on the assumption only the analogue modem needs
   to send it), but at least one NZ-market USR modem transmits its own
   1800 Hz guard tone throughout V.8 and never proceeds past CM/JM without
   apparently expecting one back from us. Mixed continuously into our own
   V.8 TX (ANSam/JM) at a fixed level; tunable for interop testing without
   a rebuild. */
static double g_guard_tone_phase = 0.0;

static double v8_guard_tone_hz(void)
{
    static int initialized = 0;
    static double hz = 1800.0;
    if (!initialized) {
        const char *value = getenv("V8_GUARD_TONE_HZ");
        if (value && value[0] != '\0') {
            char *end = NULL;
            double parsed = strtod(value, &end);
            if (end != value && end && *end == '\0' && parsed > 0.0)
                hz = parsed;
        }
        initialized = 1;
    }
    return hz;
}

static int32_t v8_guard_tone_amplitude(void)
{
    /* Disabled by default: live testing against an NZ-market USR modem
       that itself transmits a continuous 1800 Hz guard tone showed no
       change in its behaviour when we also added one on our own TX (the
       guard tone is a one-side signal, not something both ends send —
       the fix belongs on our RX side: tolerate/ignore the peer's guard
       tone while still finding the real CM signal, not mirror it back).
       Left available via env var for future experiments. ANSam's own
       reference level runs ~4100 RMS (~5800 peak); a national 1800 Hz
       guard tone at -6 dB below that would be ~2900 peak. */
    static int initialized = 0;
    static int32_t amplitude = 0;
    if (!initialized) {
        const char *value = getenv("V8_GUARD_TONE_LEVEL");
        if (value && value[0] != '\0') {
            char *end = NULL;
            long parsed = strtol(value, &end, 10);
            if (end != value && end && *end == '\0' && parsed >= 0)
                amplitude = (int32_t) parsed;
        }
        initialized = 1;
    }
    return amplitude;
}

static void mix_v8_guard_tone(int16_t *amp, int len)
{
    int32_t amplitude = v8_guard_tone_amplitude();
    double phase_inc;
    int i;

    if (amplitude <= 0)
        return;
    phase_inc = 2.0 * M_PI * v8_guard_tone_hz() / 8000.0;
    for (i = 0; i < len; i++) {
        int32_t mixed = (int32_t) amp[i]
                       + (int32_t) lround(amplitude * sin(g_guard_tone_phase));
        if (mixed > 32767) mixed = 32767;
        if (mixed < -32768) mixed = -32768;
        amp[i] = (int16_t) mixed;
        g_guard_tone_phase += phase_inc;
        if (g_guard_tone_phase >= 2.0 * M_PI)
            g_guard_tone_phase -= 2.0 * M_PI;
    }
}

/* Clock recovery. cr_update() runs on the SIP/RTP transport thread (each
   incoming RTP packet); cr_get_adjustment() runs on the PJSIP media thread
   (each pulled audio frame) — these are different threads, hence the mutex. */
static cr_state_t     g_cr;
static pthread_mutex_t g_cr_mtx = PTHREAD_MUTEX_INITIALIZER;

/* Raw G.711 bearer diagnostics and optional live taps. */
static uint64_t       g_g711_rx_octets = 0;
static uint64_t       g_g711_tx_octets = 0;
static uint64_t       g_g711_raw_v90_tx_octets = 0;
static uint64_t       g_g711_linear_tx_octets = 0;
static FILE          *g_g711_rx_tap = NULL;
static FILE          *g_g711_tx_tap = NULL;

/* Audio diagnostics: accumulated energy and sample count for V.8 logging */
static int64_t g_v8_rx_energy;
static int64_t g_v8_tx_energy;
static int64_t g_training_rx_energy;
static int     g_training_rx_count;
static int     g_v8_rx_count;
static int     g_v8_tx_count;

/* Pending SIP URI for outgoing calls (set by me_dial) */
static char g_dial_uri[256];

static int v8_alternate_answer_tone(int tone)
{
    return (tone == MODEM_CONNECT_TONES_ANSAM_PR)
        ? MODEM_CONNECT_TONES_ANSAM
        : MODEM_CONNECT_TONES_ANSAM_PR;
}

static void g711_taps_close(void)
{
    if (g_g711_rx_tap) {
        fclose(g_g711_rx_tap);
        g_g711_rx_tap = NULL;
    }
    if (g_g711_tx_tap) {
        fclose(g_g711_tx_tap);
        g_g711_tx_tap = NULL;
    }
}

static void g711_taps_init(void)
{
    const char *dir = getenv("VPCM_G711_TAP_DIR");
    char rx_path[1024];
    char tx_path[1024];

    g711_taps_close();
    if (!dir || !dir[0])
        return;
    if (snprintf(rx_path, sizeof(rx_path), "%s/live-rx.g711", dir) >= (int)sizeof(rx_path)
        || snprintf(tx_path, sizeof(tx_path), "%s/live-tx.g711", dir) >= (int)sizeof(tx_path)) {
        fprintf(stderr, "[ME] VPCM_G711_TAP_DIR path is too long; live taps disabled\n");
        return;
    }
    g_g711_rx_tap = fopen(rx_path, "wb");
    g_g711_tx_tap = fopen(tx_path, "wb");
    if (!g_g711_rx_tap || !g_g711_tx_tap) {
        fprintf(stderr, "[ME] Unable to open live G.711 taps in %s\n", dir);
        g711_taps_close();
        return;
    }
    ME_LOG("[ME] Live G.711 taps: RX=%s TX=%s\n", rx_path, tx_path);
}

/* Voice-mode PCM fidelity testing (tools/voice_pcm_fidelity.py).
 *
 * ME_VOICE_CAPTURE_HOLD=1 keeps a call up past V.8 timeout instead of hanging
 * up, so we can capture a peer that answered in AT+FCLASS=8 voice mode (no
 * V.8 CM will ever arrive). ME_VOICE_TEST_TX_FILE=<path> replaces our own TX
 * (ANSam/V.8 tones) with raw G.711 codewords read verbatim from a file, so
 * the far end's AT+VRX capture sees a byte-exact known signal instead of our
 * negotiation tones. Both are no-ops unless their env var is set. */
static bool     g_voice_capture_hold;
static bool     g_voice_capture_hold_checked;
static uint8_t *g_voice_tx_buf;
static long     g_voice_tx_len;
static long     g_voice_tx_pos;
static bool     g_voice_tx_checked;

static bool voice_capture_hold_enabled(void)
{
    if (!g_voice_capture_hold_checked) {
        const char *v = getenv("ME_VOICE_CAPTURE_HOLD");
        g_voice_capture_hold = (v && v[0] && strcmp(v, "0") != 0);
        g_voice_capture_hold_checked = true;
    }
    return g_voice_capture_hold;
}

static void voice_tx_test_file_load(void)
{
    const char *path;
    FILE *f;
    long len;

    g_voice_tx_checked = true;
    path = getenv("ME_VOICE_TEST_TX_FILE");
    if (!path || !path[0])
        return;
    f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[ME] ME_VOICE_TEST_TX_FILE: unable to open %s\n", path);
        return;
    }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0) {
        fclose(f);
        return;
    }
    g_voice_tx_buf = malloc((size_t)len);
    if (!g_voice_tx_buf) {
        fclose(f);
        return;
    }
    if (fread(g_voice_tx_buf, 1, (size_t)len, f) != (size_t)len) {
        free(g_voice_tx_buf);
        g_voice_tx_buf = NULL;
        fclose(f);
        return;
    }
    fclose(f);
    g_voice_tx_len = len;
    g_voice_tx_pos = 0;
    ME_LOG("[ME] ME_VOICE_TEST_TX_FILE loaded: %s (%ld raw G.711 octets)\n", path, len);
}

/* Fills codewords[0..count) from the loaded test file (single pass, then
 * pads with G.711 mu-law silence 0xFF). Returns true if it handled the
 * request (file loaded), false if the caller should fall back to normal TX. */
static bool voice_tx_test_fill(uint8_t *codewords, int count)
{
    int i;

    if (!g_voice_tx_checked)
        voice_tx_test_file_load();
    if (!g_voice_tx_buf)
        return false;
    for (i = 0; i < count; i++) {
        if (g_voice_tx_pos < g_voice_tx_len)
            codewords[i] = g_voice_tx_buf[g_voice_tx_pos++];
        else
            codewords[i] = 0xFF;
    }
    return true;
}

static const char *me_v92_anspcm_level_to_str(int level)
{
    switch (level & 0x03)
    {
    case 0:
        return "-9.5 dBm0";
    case 1:
        return "-12 dBm0";
    case 2:
        return "-15 dBm0";
    default:
        return "-18 dBm0";
    }
}

static void me_log_v8_peer_summary(const v8_parms_t *result)
{
    fprintf(stderr,
            "[ME] V.8 peer summary: protocol=%s, PSTN=%s, PCM=%s, NSF=%s, T.66=%d\n",
            v8_protocol_to_str(result->jm_cm.protocols),
            v8_pstn_access_to_str(result->jm_cm.pstn_access),
            v8_pcm_modem_availability_to_str(result->jm_cm.pcm_modem_availability),
            (result->jm_cm.nsf >= 0) ? "present" : "absent",
            result->jm_cm.t66);

    if (result->v92 >= 0)
    {
        const bool from_digital = (result->v92 & 0x01) != 0;
        const bool qca = (result->v92 & 0x02) != 0;
        const bool lapm = (result->v92 & 0x04) != 0;
        const int level = (result->v92 >> 6) & 0x03;

        fprintf(stderr,
                "[ME] V.8 peer V.92 packet: %s from %s modem, LAPM=%s, ANSpcm=%s (0x%02X)\n",
                qca ? "QCA" : "QC",
                from_digital ? "digital" : "analogue",
                lapm ? "yes" : "no",
                me_v92_anspcm_level_to_str(level),
                result->v92);
    }
}

static int me_start_or_restart_v8_locked(int answer_tone)
{
    v8_parms_t v8_parms;
    memset(&v8_parms, 0, sizeof(v8_parms));
    v8_parms.modem_connect_tone = g_calling_party ? MODEM_CONNECT_TONES_NONE
                                                  : answer_tone;
    v8_parms.send_ci            = g_calling_party;
    /* V.92 Table 5 QC/QCA: this endpoint is always the digital modem.
       The current Jp profile selects the mandatory 4-point TRN2u channel.
       When ME_V92_ENABLE=0, omit the V.92 octet entirely: peers such as the
       Conexant CX93001 commit to V.92 start-up procedures on seeing it in JM
       and then wait silently for V.92 short-phase signals instead of sending
       V.90 INFO0a (observed live 2026-07-19). */
    if (parse_env_int("ME_V92_ENABLE", 1) != 0)
        v8_parms.v92            = g_calling_party ? 0x45 : 0x47;
    else
        v8_parms.v92            = -1;
    v8_parms.jm_cm.call_function      = V8_CALL_V_SERIES;
    v8_parms.jm_cm.modulations        = V8_MOD_V34 | V8_MOD_V22;
    if (g_advertise_v90)
        v8_parms.jm_cm.modulations   |= V8_MOD_V90;
    v8_parms.jm_cm.protocols          = V8_PROTOCOL_LAPM_V42;
    if (g_advertise_v90) {
        v8_parms.jm_cm.pstn_access            = V8_PSTN_ACCESS_DCE_ON_DIGITAL;
        v8_parms.jm_cm.pcm_modem_availability = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;
        /* Advertising V8_PSTN_PCM_MODEM_V91 here makes 2003-era SmartLink
           V.8 parsers (slmodemd dsplibs) discard the whole JM; keep the
           interop-safe subset unless ME_V8_ADVERTISE_V91 is set. */
        if (getenv("ME_V8_ADVERTISE_V91"))
            v8_parms.jm_cm.pcm_modem_availability |= V8_PSTN_PCM_MODEM_V91;
    } else {
        v8_parms.jm_cm.pstn_access            = 0;
        v8_parms.jm_cm.pcm_modem_availability = 0;
        if (getenv("ME_V8_ADVERTISE_V91"))
            v8_parms.jm_cm.pcm_modem_availability = V8_PSTN_PCM_MODEM_V91;
    }
    v8_parms.jm_cm.nsf                = -1;
    v8_parms.jm_cm.t66                = -1;

    if (g_v8) {
        if (v8_restart(g_v8, g_calling_party, &v8_parms) != 0)
            return -1;
    } else {
        g_v8 = v8_init(NULL, g_calling_party, &v8_parms, v8_result_handler, NULL);
        if (!g_v8)
            return -1;
        {
            logging_state_t *log = v8_get_logging_state(g_v8);
            if (log)
                span_log_set_level(log, SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL |
                                        SPAN_LOG_FLOW);
        }
    }

    g_v8_active_answer_tone = g_calling_party ? MODEM_CONNECT_TONES_NONE : answer_tone;
    g_v8_rx_energy = 0;
    g_v8_rx_count  = 0;
    g_v8_tx_energy = 0;
    g_v8_tx_count  = 0;
    g_state = ME_V8;
    g_mod   = ME_MOD_NONE;
    g_phase_start_ms = trace_now_ms();
    return 0;
}

static bool me_retry_v8_with_alternate_tone_locked(const char *reason, int status)
{
    int retry_tone;

    if (g_calling_party || g_state != ME_V8 || g_v8_answer_tone_retry_done)
        return false;

    retry_tone = v8_alternate_answer_tone(g_v8_active_answer_tone);
    fprintf(stderr,
            "[ME] V.8 failed before CM (%s, status=%d); retrying answer tone with %s\n",
            reason, status, modem_connect_tone_to_str(retry_tone));
    trace_phase("V8 retry: %s status=%d tone=%s",
                reason, status, modem_connect_tone_to_str(retry_tone));

    if (me_start_or_restart_v8_locked(retry_tone) != 0) {
        ME_LOG("[ME] V.8 retry restart failed\n");
        trace_phase("V8 retry restart failed");
        return false;
    }

    g_v8_answer_tone_retry_done = true;
    return true;
}

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
        data_stack_start_online(rate, g_calling_party);
        g_state = ME_DATA;
        g_phase_start_ms = 0;
        pthread_mutex_unlock(&g_state_mtx);
        ME_LOG("[ME] %s training complete (%d bps)\n", name, rate);
        trace_phase("%s training complete: rate=%d mod=%s", name, rate, me_mod_to_str(mod));
        if (g_data_framing != DS_FRAMING_V42) {
            g_data_connect_reported = true;
            di_on_connected(rate);
        } else {
            ME_LOG("[ME] Physical carrier ready; waiting for V.42 LAPM\n");
        }
        return;
    }
    pthread_mutex_unlock(&g_state_mtx);
}

/* ------------------------------------------------------------------ */
/* V.34 get_bit / put_bit callbacks for SpanDSP                       */
/* ------------------------------------------------------------------ */

/* Forward declarations */
static void start_v22bis_training(void);
static void v34_put_aux_bit_cb(void *user_data, int bit);
static bool v90_accept_cp_diag_locked(const vpcm_cp_diag_t *diag,
                                      const char *source);
void me_hangup(void);

static void v90_cp_live_capture_reset_locked(void)
{
    pthread_mutex_lock(&g_v90_cp_live_mtx);
    g_v90_cp_live_generation++;
    g_v90_cp_live_sample_count = 0;
    g_v90_cp_live_phase4_hint = -1;
    g_v90_cp_live_next_request = -1;
    g_v90_cp_live_expected_compatibility = 0;
    g_v90_cp_live_pending = false;
    pthread_mutex_unlock(&g_v90_cp_live_mtx);
}

static void v90_cp_live_capture_append(const int16_t *samples, int count)
{
    int available;

    if (!samples || count <= 0 || !g_v90_cp_live_thread_started)
        return;
    pthread_mutex_lock(&g_v90_cp_live_mtx);
    available = V90_CP_LIVE_MAX_SAMPLES - g_v90_cp_live_sample_count;
    if (count > available)
        count = available;
    if (count > 0) {
        memcpy(g_v90_cp_live_samples + g_v90_cp_live_sample_count,
               samples,
               (size_t)count * sizeof(*samples));
        g_v90_cp_live_sample_count += count;
    }
    if (!g_v90_cp_live_shutdown
        && g_v90_cp_live_phase4_hint >= 0
        && g_v90_cp_live_next_request >= 0
        && g_v90_cp_live_sample_count >= g_v90_cp_live_next_request
        && !g_v90_cp_live_pending
        && !g_v90_cp_live_running) {
        g_v90_cp_live_pending = true;
        g_v90_cp_live_next_request = -1;
        pthread_cond_signal(&g_v90_cp_live_cond);
    }
    pthread_mutex_unlock(&g_v90_cp_live_mtx);
}

/* Called at the exact downstream DIL->Ri transition with g_state_mtx held. */
static void v90_cp_live_note_phase4_hint_locked(void)
{
    pthread_mutex_lock(&g_v90_cp_live_mtx);
    g_v90_cp_live_phase4_hint = g_v90_cp_live_sample_count;
    g_v90_cp_live_expected_compatibility = 0;
    /* SmartLink can enter Phase 4 roughly 300 ms after our downstream Ri
     * marker.  Wait 800 ms so the first batch contains several complete
     * 89-ms CPt copies even with callback and RTP framing skew. */
    g_v90_cp_live_next_request = g_v90_cp_live_sample_count + 6400;
    pthread_mutex_unlock(&g_v90_cp_live_mtx);
    ME_LOG("[ME] V.90 strict batch CP receiver armed at upstream sample %d\n",
           g_v90_cp_live_phase4_hint);
}

/* Called after an actually accepted strict frame with g_state_mtx held. */
static void v90_cp_live_mark_accepted_locked(const vpcm_cp_diag_t *diag)
{
    if (!diag)
        return;
    pthread_mutex_lock(&g_v90_cp_live_mtx);
    if (diag->frame.v90_compatibility
        && diag->frame.acknowledge) {
        /* CP' is the last Table-14 frame needed during startup. */
        g_v90_cp_live_next_request = -1;
    } else {
        g_v90_cp_live_expected_compatibility = 1;
        g_v90_cp_live_next_request = g_v90_cp_live_sample_count + 3200;
    }
    pthread_mutex_unlock(&g_v90_cp_live_mtx);
}

static void v90_dil_capture_reset(void)
{
    memset(g_v90_dil_capture, 0, sizeof(g_v90_dil_capture));
    g_v90_dil_capture_bits = 0;
    g_v90_dil_capture_search = 0;
    g_v90_dil_hyp_last_bits = 0;
    g_v90_dil_hyp_dumped = false;
    g_v90_pending_dil_valid = false;
    g_v90_dil_parse_logged = false;
    g_v90_phase3_s_events = 0;
    g_last_v90_bridge_rx_stage = -1;
    g_last_v90_bridge_tx_stage = -1;
    g_last_v90_bridge_rx_event = -1;
    memset(&g_v90_pending_dil, 0, sizeof(g_v90_pending_dil));
}

/* Runs with g_state_mtx held, either synchronously inside v34_rx() or from
 * the independent strict batch receiver after its worker reacquires state. */
static bool v90_accept_cp_diag_locked(const vpcm_cp_diag_t *diag,
                                      const char *source)
{
    bool accepted;

    if (!diag || !g_v90 || g_mod != ME_MOD_V90)
        return false;
    if (diag->frame.codec_alaw != (g_law == ME_LAW_ALAW)) {
        ME_LOG("[ME] V.90 %s codec-law bridge: peer=%s SIP=%s; translating requested analogue levels\n",
               diag->frame.v90_compatibility ? "CP" : "CPt",
               diag->frame.codec_alaw ? "A-law" : "mu-law",
               g_law == ME_LAW_ALAW ? "A-law" : "mu-law");
    }
    accepted = v90_set_phase4_cp(g_v90, &diag->frame)
        && v90_handle_rx_event(g_v90, V90_RX_EVENT_CP_VALID);
    ME_LOG("[ME] V.90 strict RX event=CP_VALID source=%s kind=%s bits=%d drn=%u ack=%d constellations=%u accepted=%d\n",
           source ? source : "unknown",
           diag->frame.v90_compatibility ? "CP" : "CPt",
           diag->nbits,
           (unsigned)diag->frame.drn,
           diag->frame.acknowledge ? 1 : 0,
           (unsigned)diag->frame.constellation_count,
           accepted ? 1 : 0);
    trace_phase("V90 strict RX event=CP_VALID source=%s kind=%s bits=%d drn=%u accepted=%d",
                source ? source : "unknown",
                diag->frame.v90_compatibility ? "CP" : "CPt",
                diag->nbits, (unsigned)diag->frame.drn, accepted ? 1 : 0);
    if (accepted)
        v90_cp_live_mark_accepted_locked(diag);
    return accepted;
}

/* Runs synchronously inside v34_rx() while g_state_mtx is already held. */
static void v90_live_cp_frame(void *user_data, const vpcm_cp_diag_t *diag)
{
    (void)user_data;
    (void)v90_accept_cp_diag_locked(diag, "spandsp");
}

static void v90_live_cp_bit(void *user_data, int bit)
{
    uint32_t rejected_before;

    (void)user_data;
    rejected_before = g_v90_cp_rx.rejected_frames;
    (void)v90_cp_rx_put_bit(&g_v90_cp_rx, bit);
    if (g_v34 && g_v90_cp_rx.rejected_frames != rejected_before)
        v34_reject_v90_phase4_hypothesis(g_v34);
}

static void *v90_cp_live_worker(void *user_data)
{
    (void)user_data;
    for (;;) {
        int16_t *snapshot;
        int sample_count;
        int phase4_hint;
        int expected_compatibility;
        unsigned generation;
        vpcm_cp_diag_t diag;
        v90_cp_live_meta_t meta;
        bool found;
        bool accepted = false;

        pthread_mutex_lock(&g_v90_cp_live_mtx);
        while (!g_v90_cp_live_pending && !g_v90_cp_live_shutdown)
            pthread_cond_wait(&g_v90_cp_live_cond, &g_v90_cp_live_mtx);
        if (g_v90_cp_live_shutdown) {
            pthread_mutex_unlock(&g_v90_cp_live_mtx);
            break;
        }
        sample_count = g_v90_cp_live_sample_count;
        phase4_hint = g_v90_cp_live_phase4_hint;
        expected_compatibility =
            g_v90_cp_live_expected_compatibility;
        generation = g_v90_cp_live_generation;
        snapshot = malloc((size_t)sample_count * sizeof(*snapshot));
        if (snapshot) {
            memcpy(snapshot,
                   g_v90_cp_live_samples,
                   (size_t)sample_count * sizeof(*snapshot));
        }
        g_v90_cp_live_pending = false;
        g_v90_cp_live_running = true;
        pthread_mutex_unlock(&g_v90_cp_live_mtx);

        memset(&diag, 0, sizeof(diag));
        memset(&meta, 0, sizeof(meta));
        found = snapshot
             && v90_cp_live_recover(snapshot,
                                    sample_count,
                                    phase4_hint,
                                    expected_compatibility,
                                    g_law == ME_LAW_ALAW,
                                    &diag,
                                    &meta);
        if (found) {
            pthread_mutex_lock(&g_state_mtx);
            pthread_mutex_lock(&g_v90_cp_live_mtx);
            bool current = generation == g_v90_cp_live_generation;
            pthread_mutex_unlock(&g_v90_cp_live_mtx);
            if (current && g_state == ME_TRAINING
                && g_mod == ME_MOD_V90 && g_v90) {
                fprintf(stderr,
                        "[ME] V.90 strict batch recovered %s%s: bits=%d "
                        "frame=%d carrier=%s timing=%d step=%d pll=%.3f "
                        "drn=%u mask=0x%04X crc=%u map=%d order=%d vote=%d/%d%%\n",
                        diag.frame.v90_compatibility ? "CP" : "CPt",
                        diag.frame.acknowledge ? "'" : "",
                        diag.nbits,
                        meta.frame_sample,
                        meta.carrier_sel ? "high" : "low",
                        meta.timing_index,
                        meta.carrier_step,
                        meta.pll_gain,
                        (unsigned)diag.frame.drn,
                        diag.frame.upstream_rate_mask,
                        (unsigned)diag.crc_remainder,
                        meta.map_index,
                        meta.bit_order,
                        meta.voted_frames,
                        meta.agreement_pct);
                accepted = v90_accept_cp_diag_locked(&diag, "batch");
            }
            pthread_mutex_unlock(&g_state_mtx);
        }
        free(snapshot);

        pthread_mutex_lock(&g_v90_cp_live_mtx);
        g_v90_cp_live_running = false;
        if (!accepted
            && generation == g_v90_cp_live_generation
            && expected_compatibility
                 == g_v90_cp_live_expected_compatibility
            && g_v90_cp_live_phase4_hint >= 0) {
            /* Retry on a fresh 500 ms of waveform.  A failed snapshot is not
             * evidence for a frame; only a complete CRC-valid observation
             * can advance the modem state. */
            g_v90_cp_live_next_request =
                g_v90_cp_live_sample_count + 4000;
        }
        pthread_mutex_unlock(&g_v90_cp_live_mtx);
    }
    return NULL;
}

static void v90_cp_live_worker_start(void)
{
    pthread_mutex_init(&g_v90_cp_live_mtx, NULL);
    pthread_cond_init(&g_v90_cp_live_cond, NULL);
    g_v90_cp_live_shutdown = false;
    if (pthread_create(&g_v90_cp_live_thread,
                       NULL,
                       v90_cp_live_worker,
                       NULL) == 0) {
        g_v90_cp_live_thread_started = true;
    } else {
        fprintf(stderr,
                "[ME] WARNING: unable to start strict V.90 CP worker\n");
    }
}

static void v90_cp_live_worker_stop(void)
{
    if (g_v90_cp_live_thread_started) {
        pthread_mutex_lock(&g_v90_cp_live_mtx);
        g_v90_cp_live_shutdown = true;
        pthread_cond_signal(&g_v90_cp_live_cond);
        pthread_mutex_unlock(&g_v90_cp_live_mtx);
        pthread_join(g_v90_cp_live_thread, NULL);
        g_v90_cp_live_thread_started = false;
    }
    pthread_cond_destroy(&g_v90_cp_live_cond);
    pthread_mutex_destroy(&g_v90_cp_live_mtx);
}

/* Runs synchronously inside me_rx_g711() while g_state_mtx is held. */
static void v92_live_p4u_frame(void *user_data,
                               v92_p4u_kind_t kind,
                               const v92_cp_diag_t *cp,
                               const v92_cpus_diag_t *cpus,
                               const v92_suvu_diag_t *suvu)
{
    bool accepted = false;
    const char *name = "unknown";
    int bits = 0;
    unsigned drn = 0;
    int ack = 0;

    (void)user_data;
    if (!g_v90 || g_mod != ME_MOD_V90 || !g_v92_active)
        return;

    if ((kind == V92_P4U_KIND_CPT || kind == V92_P4U_KIND_CPU) && cp) {
        vpcm_cp_frame_t vp;

        name = (kind == V92_P4U_KIND_CPT) ? "CPt" : "CPu";
        bits = cp->nbits;
        drn = cp->frame.drn;
        ack = cp->frame.acknowledge ? 1 : 0;
        accepted = v92_cp_frame_to_vpcm(&cp->frame, &vp);
        if (accepted && kind == V92_P4U_KIND_CPT) {
            accepted = v90_set_phase4_cp(g_v90, &vp)
                && v90_handle_rx_event(g_v90, V90_RX_EVENT_CP_VALID);
        } else if (accepted) {
            accepted = v90_set_v92_cpu(g_v90, &vp);
        }
    } else if (kind == V92_P4U_KIND_SUVU && suvu) {
        name = "SUVu";
        bits = v92_suvu_bit_length(g_v92_trn2u_points);
        ack = suvu->frame.acknowledge ? 1 : 0;
        accepted = v90_set_v92_suvu(g_v90, suvu->frame.acknowledge);
    } else if (kind == V92_P4U_KIND_CPUS && cpus) {
        /* CPus belongs to rate renegotiation, not initial Phase 4. */
        name = "CPus";
        bits = v92_cpus_bit_length(g_v92_trn2u_points);
        drn = cpus->frame.drn;
        ack = cpus->frame.acknowledge ? 1 : 0;
    }

    ME_LOG("[ME] V.92 strict RX frame=%s bits=%d drn=%u ack=%d accepted=%d\n",
           name, bits, drn, ack, accepted ? 1 : 0);
    trace_phase("V92 strict RX frame=%s bits=%d drn=%u ack=%d accepted=%d",
                name, bits, drn, ack, accepted ? 1 : 0);
}

static void cleanup_v34_v90_training_locked(void)
{
    if (g_v90) {
        v90_free(g_v90);
        g_v90 = NULL;
    }
    if (g_v34) {
        v34_free(g_v34);
        g_v34 = NULL;
    }
    g_v90_phase3_started = false;
    g_v90_completion_deferred_logged = false;
    g_v90_wait_info1_logged = false;
    g_v90_fallback_v34_logged = false;
    g_v90_fallback_phase4_released = false;
    g_v90_phase2_restarts = 0;
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    v90_cp_rx_reset(&g_v90_cp_rx);
    v92_cp_rx_reset(&g_v92_cp_rx);
    memset(&g_v92_trn2u_demod, 0, sizeof(g_v92_trn2u_demod));
    g_v92_trn2u_active = false;
    g_v34_fallback_to_v22bis_pending = false;
    g_v34_fallback_status = 0;
    v90_dil_capture_reset();
    if (g_v90_cp_live_thread_started)
        v90_cp_live_capture_reset_locked();
    g_notch.active = false;
    g_last_rx_stage = 0;
    g_last_tx_stage = 0;
}

/*
 * The SmartLink analogue peer goes back to V.90 Phase 2 when it does not
 * acquire our Sd/S-bar sequence.  In that case it expects a fresh INFO0d,
 * Tone B, and L1/L2 exchange.  Merely going quiet at the end of Jd leaves the
 * peer in its Phase-1/2 retrain, while continuing Jd contaminates its RTD
 * estimate.  Reinitialise the existing V.34 context as a V.90 digital
 * answerer and let its normal Phase-2 transmitter rejoin the peer.
 *
 * This is called only from the TX media callback with g_state_mtx held.  It
 * intentionally preserves the overall training timeout: a broken bearer
 * still falls back rather than retrying indefinitely.
 */
static bool restart_v90_phase2_locked(void)
{
    int bps;

    if (g_mod != ME_MOD_V90 || !g_v34)
        return false;

    bps = g_v34_start_bps ? g_v34_start_bps : max_v34_bps_for_baud(3200);
    if (v34_restart(g_v34, 3200, bps, true) != 0) {
        ME_LOG("[ME] V.90 Phase 2 restart failed (%d baud, %d bps)\n", 3200, bps);
        trace_phase("V90 Phase2 restart failed");
        return false;
    }

    /* The old PCM-side state belongs to the failed Phase-3 attempt.  The
       restarted V.34 receiver will create a new state only after a new,
       CRC-valid INFO1a. */
    if (g_v90) {
        v90_free(g_v90);
        g_v90 = NULL;
    }
    g_v90_phase3_started = false;
    g_v90_completion_deferred_logged = false;
    g_v90_wait_info1_logged = false;
    g_v90_fallback_v34_logged = false;
    g_v90_fallback_phase4_released = false;
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    g_v92_trn2u_active = false;
    memset(&g_v92_trn2u_demod, 0, sizeof(g_v92_trn2u_demod));
    v90_cp_rx_reset(&g_v90_cp_rx);
    v92_cp_rx_reset(&g_v92_cp_rx);
    v90_dil_capture_reset();

    /* v34_restart preserves the selected V.90 mode, but reapply the mode and
       receiver callbacks explicitly because this is also where the modified
       SpanDSP tree re-primes INFO0a framing for the answerer. */
    v34_set_v90_mode(g_v34, (g_law == ME_LAW_ALAW) ? 1 : 0);
    v34_tx_power(g_v34, -10.0f);
    v34_set_put_aux_bit(g_v34, v34_put_aux_bit_cb, NULL);
    if (!g_v92_active)
        v34_set_put_phase4_bit(g_v34, v90_live_cp_bit, NULL);
    notch_filter_init(&g_notch, 1200.0f, 30.0f, 8000.0f);

    g_v90_phase2_restarts++;
    trace_phase("V90 restart Phase2: attempt=%u profile=3200/%d",
                g_v90_phase2_restarts, bps);
    ME_LOG("[ME] V.90: no S after Jd; restarting Phase 2 (%u, 3200 baud / %d bps)\n",
           g_v90_phase2_restarts, bps);
    return true;
}

static int v90_dil_capture_get_bit(int pos)
{
    return (g_v90_dil_capture[pos / 8] >> (pos % 8)) & 1;
}

static void v90_dil_capture_set_bit(int pos, int bit)
{
    if (bit)
        g_v90_dil_capture[pos / 8] |= (uint8_t)(1U << (pos % 8));
}

static void v90_dil_capture_set_bit_in_buf(uint8_t *buf, int pos, int bit)
{
    if (bit)
        buf[pos / 8] |= (uint8_t)(1U << (pos % 8));
}

static bool v90_dil_capture_has_preamble(int start)
{
    for (int i = 0; i < 17; i++) {
        if (v90_dil_capture_get_bit(start + i) == 0)
            return false;
    }
    return v90_dil_capture_get_bit(start + 17) == 0;
}

static bool v90_dil_capture_try_parse_at(int start)
{
    uint8_t shifted[(V90_DIL_CAPTURE_MAX_BITS + 7) / 8];
    v90_dil_desc_t desc;
    int shifted_bits;

    shifted_bits = g_v90_dil_capture_bits - start;
    if (shifted_bits < 206)
        return false;

    memset(shifted, 0, sizeof(shifted));
    for (int i = 0; i < shifted_bits; i++)
        v90_dil_capture_set_bit_in_buf(shifted, i, v90_dil_capture_get_bit(start + i));

    if (!v90_parse_dil_descriptor(&desc, shifted, shifted_bits))
        return false;

    g_v90_pending_dil = desc;
    g_v90_pending_dil_valid = true;
    if (g_v90)
        v90_set_dil_descriptor(g_v90, &g_v90_pending_dil);
    ME_LOG("[ME] V.90: parsed Ja DIL descriptor (N=%u LSP=%u LTP=%u)\n",
            desc.n, desc.lsp, desc.ltp);
    trace_phase("V90 parsed Ja DIL descriptor: N=%u LSP=%u LTP=%u",
                desc.n, desc.lsp, desc.ltp);
    g_v90_dil_parse_logged = true;
    return true;
}

static int v90_ja_dump_min_bits(void)
{
    const char *value = getenv("ME_V90_JA_DUMP_MIN_BITS");
    char *end;
    long parsed;

    if (value && *value) {
        parsed = strtol(value, &end, 10);
        if (end != value && *end == '\0' && parsed >= 206 && parsed <= INT_MAX)
            return (int) parsed;
    }
    return 32000;
}

static bool v90_dil_capture_try_v34_hypotheses(void)
{
    uint8_t unpacked[V90_DIL_CAPTURE_MAX_BITS];
    int first_bits;

    if (!g_v34 || g_v90_pending_dil_valid)
        return g_v90_pending_dil_valid;

    first_bits = v34_v90_copy_phase3_ja_bits(g_v34,
                                             0,
                                             unpacked,
                                             V90_DIL_CAPTURE_MAX_BITS);
    if (first_bits < 206)
        return false;
    /* This is called every RX audio frame (~20ms), but the 24-hypothesis x
       sliding-window search below is expensive, so re-attempts are
       throttled to once per this many newly-captured bits. Measured live
       against the d-modem/slmodemd rig: successful decode needed ~16 of
       these throttle cycles (~1.7s at the default 512) after the first
       206-bit-eligible window, even though the winning hypothesis (8) was
       consistent across repeated calls -- i.e. most of that time was spent
       waiting for the next throttled attempt, not for more real signal.
       Env-tunable for measurement; default kept at 512. */
    int retry_bits = parse_env_int("ME_V90_DIL_HYP_RETRY_BITS", 512);
    if (retry_bits < 0)
        retry_bits = 512;
    if (first_bits < V90_DIL_CAPTURE_MAX_BITS
        && first_bits < g_v90_dil_hyp_last_bits + retry_bits)
        return false;
    g_v90_dil_hyp_last_bits = first_bits;

    for (int hypothesis = 0; hypothesis < 24; hypothesis++) {
        int bits;

        if (hypothesis == 0) {
            bits = first_bits;
        } else {
            bits = v34_v90_copy_phase3_ja_bits(g_v34,
                                               hypothesis,
                                               unpacked,
                                               V90_DIL_CAPTURE_MAX_BITS);
        }

        if (bits < 206)
            continue;

        memset(g_v90_dil_capture, 0, sizeof(g_v90_dil_capture));
        for (int i = 0; i < bits; i++)
            v90_dil_capture_set_bit(i, unpacked[i] & 1);
        g_v90_dil_capture_bits = bits;
        g_v90_dil_capture_search = 0;

        while (g_v90_dil_capture_search + 206 <= g_v90_dil_capture_bits) {
            int start = g_v90_dil_capture_search++;

            if (v90_dil_capture_has_preamble(start)
                && v90_dil_capture_try_parse_at(start)) {
                ME_LOG("[ME] V.90: Ja descriptor recovered with V.34 hypothesis %d\n",
                       hypothesis);
                trace_phase("V90 Ja descriptor recovered with V.34 hypothesis %d",
                            hypothesis);
                return true;
            }
        }
    }

    /* The dump threshold used to be a hard 32000 bits.  Live calls against the
     * d-modem rig only ever accumulate ~19000 Ja bits before the peer gives up,
     * so the dump never fired on exactly the runs worth investigating.  Keep
     * 32000 as the default and let ME_V90_JA_DUMP_MIN_BITS lower it. */
    if (!g_v90_dil_hyp_dumped && first_bits >= v90_ja_dump_min_bits()) {
        const char *prefix = getenv("ME_V90_JA_DUMP_PREFIX");

        if (prefix && *prefix) {
            char path[1024];

            for (int hypothesis = 0; hypothesis < 24; hypothesis++) {
                FILE *fp;
                int bits;

                bits = v34_v90_copy_phase3_ja_bits(g_v34,
                                                    hypothesis,
                                                    unpacked,
                                                    V90_DIL_CAPTURE_MAX_BITS);
                snprintf(path, sizeof(path), "%s-hyp%d.bits", prefix, hypothesis);
                fp = fopen(path, "wb");
                if (fp) {
                    fwrite(unpacked, 1, (size_t)bits, fp);
                    fclose(fp);
                }

                bits = v34_v90_copy_phase3_ja_raw_bits(g_v34,
                                                        hypothesis,
                                                        unpacked,
                                                        V90_DIL_CAPTURE_MAX_BITS);
                snprintf(path, sizeof(path), "%s-hyp%d.rawbits", prefix, hypothesis);
                fp = fopen(path, "wb");
                if (fp) {
                    fwrite(unpacked, 1, (size_t)bits, fp);
                    fclose(fp);
                }
            }
            ME_LOG("[ME] V.90: dumped %d Ja hypothesis bits to %s-*\n",
                   first_bits, prefix);
        }
        g_v90_dil_hyp_dumped = true;
    }

    return false;
}

static int v34_get_bit_cb(void *user_data)
{
    int bit;

    (void)user_data;
    bit = ds_tx_get_bit(&g_data_stack);
    return (bit == DS_TX_NO_DATA) ? SIG_STATUS_END_OF_DATA : bit;
}

static void v34_put_aux_bit_cb(void *user_data, int bit)
{
    (void)user_data;

    if (bit < 0 || bit > 1)
        return;
    if (g_mod != ME_MOD_V90 || g_v90_pending_dil_valid)
        return;
    if (g_v90_dil_capture_bits >= V90_DIL_CAPTURE_MAX_BITS)
        return;

    v90_dil_capture_set_bit(g_v90_dil_capture_bits, bit & 1);
    g_v90_dil_capture_bits++;

    if (g_v90_dil_capture_bits <= 16 || (g_v90_dil_capture_bits % 256) == 0) {
        ME_LOG("[ME] V.90 Ja capture: buffered %d bits%s\n",
                g_v90_dil_capture_bits,
                g_v90_pending_dil_valid ? " (descriptor already parsed)" : "");
    }

    if (g_v90_dil_capture_bits < 206)
        return;

    while (g_v90_dil_capture_search + 206 <= g_v90_dil_capture_bits) {
        int start = g_v90_dil_capture_search++;

        if (!v90_dil_capture_has_preamble(start))
            continue;
        if (v90_dil_capture_try_parse_at(start))
            break;
    }
}

static void v34_put_bit_cb(void *user_data, int bit)
{
    (void)user_data;
    if (bit < 0) {
        /* Status event from V.34 training state machine */
        trace_phase("V34 rx status=%s (%d)", signal_status_to_str(bit), bit);
        if (bit == SIG_STATUS_CARRIER_UP || bit == SIG_STATUS_TRAINING_SUCCEEDED) {
            if (g_state == ME_TRAINING) {
                int rate = v34_get_current_bit_rate(g_v34);
                if (g_mod == ME_MOD_V90) {
                    if (g_v90 && (v90_training_complete(g_v90) || v90_using_internal_v34_tx(g_v90))) {
                        int downstream_rate = (v90_data_bits_per_frame(g_v90) * 8000) / 6;

                        if (downstream_rate <= 0)
                            downstream_rate = V90_RATE_BPS;
                        data_stack_start_online(downstream_rate, g_calling_party);
                        g_state = ME_DATA;
                        g_phase_start_ms = 0;
                        ME_LOG("[ME] V.90 training complete (upstream V.34 %d bps, downstream PCM %d bps)\n",
                                rate, downstream_rate);
                        trace_phase("V90 enter DATA: upstream=%d downstream=%d", rate, downstream_rate);
                        v90_reset_data_mode(g_v90);
                        g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
                        if (g_data_framing != DS_FRAMING_V42) {
                            g_data_connect_reported = true;
                            di_on_connected(downstream_rate);
                        }
                    } else if (!g_v90_completion_deferred_logged) {
                        ME_LOG("[ME] V.90 received generic training success from V.34, but V.90 startup is not complete yet; remaining in TRAINING\n");
                        trace_phase("V90 deferred DATA entry: V34 success before V90 startup complete");
                        g_v90_completion_deferred_logged = true;
                    }
                } else {
                    data_stack_start_online(rate, g_calling_party);
                    g_state = ME_DATA;
                    g_phase_start_ms = 0;
                    ME_LOG("[ME] V.34 training complete (%d bps)\n", rate);
                    trace_phase("V34 enter DATA: rate=%d", rate);
                    if (g_data_framing != DS_FRAMING_V42) {
                        g_data_connect_reported = true;
                        di_on_connected(rate);
                    }
                }
                return;
            }
        }
        if (bit == SIG_STATUS_TRAINING_FAILED || bit == SIG_STATUS_CARRIER_DOWN) {
            if (g_state == ME_TRAINING && (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90)) {
                ME_LOG("[ME] V.34 training failed (%s), falling back to V.22bis\n",
                        signal_status_to_str(bit));
                trace_phase("V34 training failed (%s) -> fallback V22BIS",
                            signal_status_to_str(bit));
                /* This callback runs from inside v34_rx().  Defer cleanup and
                   the V.22bis handoff until me_rx_audio() regains control so
                   we don't free the active V.34 context out from under SpanDSP. */
                g_v34_fallback_to_v22bis_pending = true;
                g_v34_fallback_status = bit;
                g_phase_start_ms = 0;
                return;
            }
        }
        ME_LOG("[ME] V.34 status: %s (%d)\n", signal_status_to_str(bit), bit);
        return;
    }
    ds_rx_put_bit(&g_data_stack, bit);
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
    g_phase_start_ms = trace_now_ms();
    trace_phase("enter TRAINING: mod=V22BIS role=%s", g_calling_party ? "caller" : "answerer");
    data_stack_prepare(2400);
    if (g_v22bis) {
        v22bis_free(g_v22bis);
        g_v22bis = NULL;
    }

    g_v22bis = v22bis_init(NULL, 2400, V22BIS_GUARD_TONE_NONE, g_calling_party,
                           v22bis_get_bit_cb, NULL,
                           v22bis_put_bit_cb, NULL);
    if (!g_v22bis)
        ME_LOG("[ME] v22bis_init failed\n");
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
    v90_dil_capture_reset();

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
    data_stack_prepare(bps);
    g_v34 = v34_init(NULL,
                     g_v34_start_baud,
                     bps,
                     g_calling_party,
                     true,          /* full duplex */
                     v34_get_bit_cb, NULL,
                     v34_put_bit_cb, NULL);
    if (!g_v34) {
        ME_LOG("[ME] v34_init failed, falling back to V.22bis\n");
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

    /* Echo canceller: essential for V.90 where our broadband PCM TX echoes back
       through the FXS hybrid and overwhelms the upstream V.34 signal.
       512 taps = 64ms covers typical SIP round-trip echo delay.
       For pure V.34, the notch filter below handles narrowband echo. */
    if (g_echo_can) {
        modem_echo_can_segment_free(g_echo_can);
        g_echo_can = NULL;
    }
    if (g_advertise_v90) {
        g_echo_can = modem_echo_can_segment_init(ECHO_CAN_TAPS);
        if (g_echo_can) {
            modem_echo_can_adaption_mode(g_echo_can, 1);
            ME_LOG("[ME] Echo canceller enabled for V.90 (%d taps = %dms)\n",
                    ECHO_CAN_TAPS, ECHO_CAN_TAPS * 1000 / 8000);
        }
        g_tx_buf_wr = 0;
        g_tx_buf_rd = 0;
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
            ME_LOG("[ME] Notch filter DISABLED: TX=%.1f Hz, RX=%.1f Hz, "
                    "separation=%.1f Hz too narrow for %d baud\n",
                    our_tx_carrier, rx_carrier, separation, g_v34_start_baud);
        } else {
            notch_filter_init(&g_notch, our_tx_carrier, 30.0f, 8000.0f);
            ME_LOG("[ME] Notch filter at %.1f Hz (Q=30) for %d baud, "
                    "RX carrier=%.1f Hz (sep=%.1f Hz)\n",
                    our_tx_carrier, g_v34_start_baud, rx_carrier, separation);
        }
    }

    ME_LOG("[ME] V.34 training started (%s, %d baud, up to %d bps)\n",
            g_calling_party ? "caller" : "answerer", g_v34_start_baud, bps);
}

static void v8_result_handler(void *user_data, v8_parms_t *result)
{
    (void)user_data;
    char mod_str[96];
    v8_mod_mask_to_str(result->jm_cm.modulations, mod_str, sizeof(mod_str));

    ME_LOG("[ME] V.8 result: status=%d, modulations=0x%X pstn_access=0x%X\n",
            result->status, result->jm_cm.modulations, result->jm_cm.pstn_access);
    trace_phase("V8 result: status=%s (%d) mods=%s (0x%X) protocol=0x%X pstn=0x%X pcm=0x%X",
                v8_status_to_str(result->status), result->status,
                mod_str, result->jm_cm.modulations, result->jm_cm.protocols,
                result->jm_cm.pstn_access, result->jm_cm.pcm_modem_availability);
    me_log_v8_peer_summary(result);

    /* V8_STATUS_V8_OFFERED just means the other end offered V.8 — still in progress */
    if (result->status == V8_STATUS_IN_PROGRESS
        || result->status == V8_STATUS_V8_OFFERED
        || result->status == V8_STATUS_CALL_FUNCTION_RECEIVED
        || result->status == V8_STATUS_CALLING_TONE_RECEIVED) {
        ME_LOG("[ME] V.8 in progress (status=%d)\n", result->status);
        return;
    }

    if (result->status == V8_STATUS_NON_V8_CALL) {
        /*
         * Remote end doesn't support V.8 (e.g. plain V.22bis modem or
         * ATA auto-answer with no V.8).  Fall back to V.22bis directly.
         */
        ME_LOG("[ME] Non-V.8 call detected, falling back to V.22bis\n");
        pthread_mutex_lock(&g_state_mtx);
        start_v22bis_training();
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }

    if (result->status != V8_STATUS_V8_CALL) {
        pthread_mutex_lock(&g_state_mtx);
        if (me_retry_v8_with_alternate_tone_locked("result failure", result->status)) {
            pthread_mutex_unlock(&g_state_mtx);
            ME_LOG("[ME] V.8 answer tone in use: %s\n",
                    modem_connect_tone_to_str(g_v8_active_answer_tone));
            return;
        }
        pthread_mutex_unlock(&g_state_mtx);
        if (voice_capture_hold_enabled()) {
            ME_LOG("[ME] V.8 failed (status=%d), ME_VOICE_CAPTURE_HOLD set: holding call open\n",
                    result->status);
            g_phase_start_ms = 0;
            return;
        }
        ME_LOG("[ME] V.8 failed (status=%d), hanging up\n", result->status);
        me_hangup();
        return;
    }

    /* V8_STATUS_V8_CALL — negotiation complete, inspect agreed modulation */
    pthread_mutex_lock(&g_state_mtx);

    if ((result->jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V91) != 0) {
        ME_LOG("[ME] V.8 negotiated V.91 symmetric PCM mode\n");
        trace_phase("V8 selected V91");
        if (!v91_live_start_locked()) {
            pthread_mutex_unlock(&g_state_mtx);
            me_hangup();
            return;
        }
    } else if ((result->jm_cm.modulations & V8_MOD_V90) && g_advertise_v90
        && (result->jm_cm.modulations & V8_MOD_V34)) {
        /*
         * V.90 selected: downstream = PCM codeword injection (up to 56 kbps),
         * upstream = V.34 modulation.  Training uses V.34 Phases 2-4; after
         * training completes, TX switches from V.34 to direct PCM injection.
         */
        /* A V.92 capability octet does not guarantee that the caller selected
         * a V.92 data pump for this call.  In particular, SmartLink still
         * advertises V.92 QC while AT+MS=90 forces its long start-up down the
         * V.90 CPt path.  Permit the interop rig (and deployments that need a
         * strict V.90 answer) to suppress native V.92 Phase 4 explicitly. */
        g_v92_active = result->v92 >= 0
                    && parse_env_int("ME_V92_ENABLE", 1) != 0;
        if (result->v92 >= 0 && !g_v92_active)
            ME_LOG("[ME] V.92 capability present, but ME_V92_ENABLE=0; using V.90 Phase 4\n");
        ME_LOG("[ME] V.8 negotiated %s (PCM downstream + V.34 upstream)\n",
               g_v92_active ? "V.92" : "V.90");
        trace_phase("V8 selected %s", g_v92_active ? "V92" : "V90");
        g_mod = ME_MOD_V90;
        /* V.90 §6.2: analog modem only supports 3200 baud (mandatory) */
        int saved_baud = g_v34_start_baud;
        g_v34_start_baud = 3200;
        start_v34_training();
        g_v34_start_baud = saved_baud;
        /* start_v34_training sets g_mod = ME_MOD_V34; override back to V90 */
        g_mod = ME_MOD_V90;
        v90_cp_live_capture_reset_locked();
        /* Enable V.90 INFO0d frame generation and carrier swap in SpanDSP V.34.
           v34_set_v90_mode also updates CC carrier frequencies (§8.2.3.1). */
        if (g_v34)
            v34_set_v90_mode(g_v34, (g_law == ME_LAW_ALAW) ? 1 : 0);
        v90_cp_rx_init(&g_v90_cp_rx,
                       4,
                       g_law == ME_LAW_ALAW,
                       v90_live_cp_frame,
                       NULL);
        if (g_v34 && !g_v92_active)
            v34_set_put_phase4_bit(g_v34, v90_live_cp_bit, NULL);
        if (g_v34)
            v34_set_put_aux_bit(g_v34, v34_put_aux_bit_cb, NULL);
        g_v90_phase3_started = false;
        g_v90_completion_deferred_logged = false;
        g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
        v90_dil_capture_reset();
        /* Phase 2 CC carriers: V.90 digital answerer TX=1200 Hz, RX=2400 Hz.
           The data carriers at 3200 baud are only 91 Hz apart so start_v34_training()
           disabled the notch — but we need it for Phase 2 CC echo removal. */
        notch_filter_init(&g_notch, 1200.0f, 30.0f, 8000.0f);
        ME_LOG("[ME] V.90 notch filter at 1200 Hz (our CC TX), RX CC at 2400 Hz\n");

    } else if (result->jm_cm.modulations & V8_MOD_V34) {
        ME_LOG("[ME] V.8 negotiated V.34 (full duplex, up to 33.6 kbps)\n");
        trace_phase("V8 selected V34");
        start_v34_training();

    } else if (result->jm_cm.modulations & V8_MOD_V22) {
        ME_LOG("[ME] V.8 negotiated V.22bis fallback\n");
        trace_phase("V8 selected V22BIS");
        start_v22bis_training();

    } else {
        ME_LOG("[ME] V.8 no usable modulation (0x%X), hanging up\n",
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
    v90_cp_live_worker_start();
    dring_init(&downstream_ring);
    dring_init(&upstream_ring);
    {
        const char *framing = getenv("ME_DATA_FRAMING");

        if (framing && strcmp(framing, "raw") == 0) {
            g_data_framing = DS_FRAMING_RAW;
        } else if (framing && strcmp(framing, "lapm") == 0) {
            g_data_framing = DS_FRAMING_V42;
            g_data_lapm_detect = true;
        } else if (framing && strcmp(framing, "lapm-bypass") == 0) {
            g_data_framing = DS_FRAMING_V42;
            g_data_lapm_detect = false;
        } else {
            g_data_framing = DS_FRAMING_V14;
        }
        ME_LOG("[ME] DTE framing: %s%s\n",
               g_data_framing == DS_FRAMING_V14 ? "V.14 8N1" :
               g_data_framing == DS_FRAMING_RAW ? "RAW" :
               g_data_lapm_detect ? "V.42 LAPM" : "V.42 LAPM (ODP/ADP bypass)",
               g_data_framing == DS_FRAMING_RAW ? " (diagnostic only)" : "");
    }
    cr_init(&g_cr, 8000);
    g_g711_rx_octets = 0;
    g_g711_tx_octets = 0;
    g_g711_raw_v90_tx_octets = 0;
    g_g711_linear_tx_octets = 0;
    {
        int env_lu = parse_env_int("V92_TRN2U_LU", (int)g_v92_trn2u_lu);
        if (env_lu >= 500 && env_lu <= 30000)
            g_v92_trn2u_lu = (double)env_lu;
        ME_LOG("[ME] V.92 TRN2u receiver: %d-point PAM, L_U=%.0f\n",
               g_v92_trn2u_points, g_v92_trn2u_lu);
    }
    g711_taps_init();
    {
        const char *inv = getenv("ME_V34_INVERT_ROLE");
        g_invert_v34_role = (inv && (inv[0] == '1' || inv[0] == 'y' || inv[0] == 'Y' ||
                                     inv[0] == 't' || inv[0] == 'T'));
        if (g_invert_v34_role)
            ME_LOG("[ME] DEBUG: role inversion enabled (ME_V34_INVERT_ROLE)\n");
    }
    g_v8_answer_tone = parse_v8_answer_tone_env("ME_V8_ANSWER_TONE",
                                                MODEM_CONNECT_TONES_ANSAM);
    ME_LOG("[ME] V.8 answer tone: %s\n",
            modem_connect_tone_to_str(g_v8_answer_tone));
    {
        int env_baud = parse_env_int("ME_V34_BAUD", g_v34_start_baud);
        int env_bps  = parse_env_int("ME_V34_BPS", 0);
        if (valid_v34_baud(env_baud))
            g_v34_start_baud = env_baud;
        if (env_bps > 0 && valid_v34_bps(env_bps))
            g_v34_start_bps = env_bps;
        int effective_bps = g_v34_start_bps ? g_v34_start_bps : max_v34_bps_for_baud(g_v34_start_baud);
        ME_LOG("[ME] V.34 start profile: %d baud / %d bps\n",
                g_v34_start_baud, effective_bps);
    }
    g_state = ME_IDLE;
    g_mod   = ME_MOD_NONE;
    v91_live_reset();
}

void me_destroy(void)
{
    v90_cp_live_worker_stop();
    if (g_v8)       { v8_free(g_v8);                             g_v8       = NULL; }
    if (g_v22bis)   { v22bis_free(g_v22bis);                     g_v22bis   = NULL; }
    cleanup_v34_v90_training_locked();
    v91_live_reset();
    if (g_echo_can) { modem_echo_can_segment_free(g_echo_can);   g_echo_can = NULL; }
    ds_release(&g_data_stack);
    g711_taps_close();
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
    g_v8_tx_energy     = 0;
    g_v8_tx_count      = 0;
    g_v8_active_answer_tone = g_v8_answer_tone;
    g_v8_answer_tone_retry_done = false;
    g_v92_active = false;
    g_v92_trn2u_active = false;
    g_data_link_failed = false;
    g_data_connect_reported = false;

    /* Outgoing dial = caller role; incoming auto-answer = answerer role. */
    g_calling_party = (g_state == ME_DIALING);
    if (g_invert_v34_role)
        g_calling_party = !g_calling_party;
    trace_phase("SIP media connected: role=%s", g_calling_party ? "caller" : "answerer");

    if (g_v8) {
        v8_free(g_v8);
        g_v8 = NULL;
    }
    if (me_start_or_restart_v8_locked(g_v8_answer_tone) != 0) {
        ME_LOG("[ME] v8_init failed\n");
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }
    pthread_mutex_unlock(&g_state_mtx);
    trace_phase("enter V8: advertised mods=%s", g_advertise_v90 ? "V90|V34|V22" : "V34|V22");

    ME_LOG("[ME] SIP connected as %s, starting V.8 handshake\n",
            g_calling_party ? "caller" : "answerer");
    if (!g_calling_party) {
        ME_LOG("[ME] V.8 answer tone in use: %s\n",
                modem_connect_tone_to_str(g_v8_answer_tone));
    }
}

/* Called by sip_modem.c when the SIP call is torn down */
void me_on_sip_disconnected(void)
{
    pthread_mutex_lock(&g_state_mtx);

    if (g_v8)       { v8_free(g_v8);                             g_v8       = NULL; }
    if (g_v22bis)   { v22bis_free(g_v22bis);                     g_v22bis   = NULL; }
    cleanup_v34_v90_training_locked();
    v91_live_reset();
    if (g_echo_can) { modem_echo_can_segment_free(g_echo_can);   g_echo_can = NULL; }
    ds_release(&g_data_stack);
    g_v8_active_answer_tone = g_v8_answer_tone;
    g_v8_answer_tone_retry_done = false;

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

/* V.90 WAIT_JA energy-gap Ja detector.
 *
 * The analogue modem's Phase 3 upstream is S/PP/TRN, then a silent gap
 * (observed ~600 ms on SmartLink), then Ja — and it enters its Sd search
 * window (WaitForSd) at the exact moment Ja starts.  Bit-level Ja decode
 * from the 4-point sliced stream is unreliable (scores ~22/32 on live
 * captures) and scrambled TRN payload can fake the J pattern, so instead
 * key on the unmissable physical marker: sustained Phase 3 energy, then
 * a silent gap, then energy returning.  Fire the Ja event shortly after
 * the energy returns so Sd lands inside the analogue modem's window.
 * Runs with g_state_mtx held. */
static void v90_wait_ja_energy_gate_locked(const int16_t *amp, int len)
{
    /* 10 ms energy windows at 8000 Hz */
    enum { JA_GATE_WIN = 80 };
    static int64_t acc;
    static int     acc_n;
    static int     state;          /* 0=await signal, 1=in signal, 2=in gap, 3=fired */
    static int     ms_in_state;

    if (!g_v90 || v90_get_tx_phase(g_v90) != V90_TX_WAIT_JA) {
        acc = 0;
        acc_n = 0;
        state = 0;
        ms_in_state = 0;
        return;
    }

    for (int i = 0; i < len; i++) {
        acc += (int64_t)amp[i] * amp[i];
        if (++acc_n < JA_GATE_WIN)
            continue;

        {
            double rms = sqrt((double)acc / acc_n);
            /* Phase 3 upstream runs ~-21 dBFS (rms ~2900); the gap sits at
               or below ~-40 dBFS (rms ~330). */
            int signal_now = (rms > 1000.0);
            int silent_now = (rms < 400.0);

            acc = 0;
            acc_n = 0;
            ms_in_state += 10;

            switch (state) {
            case 0:                     /* wait for sustained Phase 3 energy */
                if (signal_now) {
                    if (ms_in_state >= 300) {
                        state = 1;
                        ms_in_state = 0;
                    }
                } else {
                    ms_in_state = 0;
                }
                break;
            case 1:                     /* in S/PP/TRN — wait for the gap */
                if (silent_now) {
                    /* SmartLink's TRN-to-Ja gap is as short as 80 ms on
                       some attempts (600 ms on others) — trigger fast. */
                    if (ms_in_state >= 50) {
                        ME_LOG("[ME] V.90 WAIT_JA: TRN-to-Ja silence gap detected (%d ms)\n",
                               ms_in_state);
                        state = 2;
                        ms_in_state = 0;
                    }
                } else {
                    ms_in_state = 0;
                }
                break;
            case 2:                     /* in gap — energy return means Ja */
                if (signal_now) {
                    if (ms_in_state >= 30) {
                        bool accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_J);

                        ME_LOG("[ME] V.90 WAIT_JA: energy returned after gap; treating as Ja start (accepted=%d)\n",
                               accepted ? 1 : 0);
                        if (accepted) {
                            (void)v90_dil_capture_try_v34_hypotheses();
                            if (g_v34)
                                v34_v90_arm_phase3_s_detector(g_v34);
                        }
                        state = 3;
                        ms_in_state = 0;
                    }
                } else {
                    ms_in_state = 0;
                }
                break;
            default:                    /* fired — phase change resets us */
                break;
            }
        }
    }
}

/* Fed by sip_modem.c's RTP transport tap on packet arrival (transport
   thread). Updates the DPLL's estimate of the remote sender's sample-clock
   rate relative to our own wall clock. */
void me_cr_update(uint32_t rtp_ts, int64_t local_ns)
{
    static int calls = 0;
    pthread_mutex_lock(&g_cr_mtx);
    cr_update(&g_cr, rtp_ts, local_ns);
    if (calls == 0)
        ME_LOG("[ME] DIAG: cr_update first call rtp_ts=%u local_ns=%lld\n",
               rtp_ts, (long long) local_ns);
    if (++calls % 100 == 0)
        ME_LOG("[ME] DIAG: cr_update calls=%d phase_err=%.3f samples\n",
               calls, cr_get_phase_error(&g_cr));
    pthread_mutex_unlock(&g_cr_mtx);
}

/* Called once per RX audio frame (media thread), before feeding samples to
   me_rx_audio()/me_rx_g711(). Returns +1/-1/0 per clock_recovery.h. */
int me_cr_get_adjustment(void)
{
    int adj;
    pthread_mutex_lock(&g_cr_mtx);
    adj = cr_get_adjustment(&g_cr);
    pthread_mutex_unlock(&g_cr_mtx);
    if (adj != 0)
        ME_LOG("[ME] DIAG: cr_get_adjustment returned %d\n", adj);
    return adj;
}

void me_rx_audio(const int16_t *amp, int len)
{
    pthread_mutex_lock(&g_state_mtx);
    me_state_t state = g_state;
    me_modulation_t mod = g_mod;
    pthread_mutex_unlock(&g_state_mtx);

    /* Linear PCM is a debugging fallback for V.91.  Real carriage enters via
     * me_rx_g711(), but keeping this path functional makes failures explicit
     * instead of silently feeding V.91 codewords into the V.34 receiver. */
    if (mod == ME_MOD_V91 && (state == ME_TRAINING || state == ME_DATA)) {
        int offset = 0;

        while (offset < len) {
            uint8_t codewords[320];
            int chunk = len - offset;

            if (chunk > (int)sizeof(codewords))
                chunk = (int)sizeof(codewords);
            for (int i = 0; i < chunk; i++)
                codewords[i] = linear_to_pcm(amp[offset + i]);
            pthread_mutex_lock(&g_state_mtx);
            v91_live_receive_codewords_locked(codewords, chunk);
            pthread_mutex_unlock(&g_state_mtx);
            offset += chunk;
        }
        return;
    }

    /* Check for phase timeouts */
    if (g_phase_start_ms > 0) {
        uint64_t elapsed = trace_now_ms() - g_phase_start_ms;
        if (state == ME_V8 && elapsed > V8_TIMEOUT_MS) {
            ME_LOG("[ME] V.8 negotiation timed out after %llu ms\n",
                    (unsigned long long)elapsed);
            trace_phase("V8 timeout after %llums", (unsigned long long)elapsed);
            pthread_mutex_lock(&g_state_mtx);
            if (me_retry_v8_with_alternate_tone_locked("timeout", V8_STATUS_FAILED)) {
                pthread_mutex_unlock(&g_state_mtx);
                ME_LOG("[ME] V.8 answer tone in use: %s\n",
                        modem_connect_tone_to_str(g_v8_active_answer_tone));
                return;
            }
            pthread_mutex_unlock(&g_state_mtx);
            g_phase_start_ms = 0;
            if (voice_capture_hold_enabled()) {
                ME_LOG("[ME] ME_VOICE_CAPTURE_HOLD set: holding call open past V.8 timeout for voice-mode capture\n");
                return;
            }
            me_hangup();
            return;
        }
        if (state == ME_TRAINING && elapsed > TRAINING_TIMEOUT_MS) {
            ME_LOG("[ME] Training timed out after %llu ms (mod=%s)\n",
                    (unsigned long long)elapsed, me_mod_to_str(g_mod));
            trace_phase("TRAINING timeout after %llums mod=%s",
                        (unsigned long long)elapsed, me_mod_to_str(g_mod));
            g_phase_start_ms = 0;
            if (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) {
                /* V.34/V.90 training failed — fall back to V.22bis */
                ME_LOG("[ME] %s training timeout, falling back to V.22bis\n",
                        g_mod == ME_MOD_V90 ? "V.90" : "V.34");
                trace_phase("%s timeout -> fallback V22BIS",
                            g_mod == ME_MOD_V90 ? "V90" : "V34");
                pthread_mutex_lock(&g_state_mtx);
                cleanup_v34_v90_training_locked();
                start_v22bis_training();
                pthread_mutex_unlock(&g_state_mtx);
                return;
            }
            /* V.22bis timeout — give up */
            if (voice_capture_hold_enabled()) {
                ME_LOG("[ME] ME_VOICE_CAPTURE_HOLD set: holding call open past V.22bis timeout for voice-mode capture\n");
                return;
            }
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
            ME_LOG("[ME] V.8 rx: RMS=%.1f (%d samples) — %s\n",
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
        if (state == ME_TRAINING && mod == ME_MOD_V90)
            v90_cp_live_capture_append(amp, len);
        /* RX energy diagnostic — log every second during training */
        if (state == ME_TRAINING) {
            for (int i = 0; i < len; i++)
                g_training_rx_energy += (int64_t)amp[i] * amp[i];
            g_training_rx_count += len;
            if (g_training_rx_count >= 8000) {
                double rms = sqrt((double)g_training_rx_energy / g_training_rx_count);
                /* Keep this diagnostic for abnormal levels only. */
                if (rms < 20.0 || rms > 2000.0) {
                    ME_LOG("[ME] Training rx: RMS=%.1f (%d samples)\n",
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
                int rx_event = v34_get_rx_event(g_v34);
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
                /* NLMS echo canceller: subtract our TX echo from the RX signal.
                   Critical for V.90 where broadband PCM TX overwhelms the
                   upstream V.34 signal through the FXS hybrid.

                   Uses a single 512-tap FIR (64ms) with no separate delay search.
                   The FIR naturally learns the echo impulse response including
                   the bulk delay (~120 samples / 15ms through the SIP/FXS path).

                   TX reference comes from the ring buffer with a fixed lookback
                   so that tap[0] corresponds to the most recent TX sample and
                   tap[N] corresponds to N samples ago. */
                if (g_echo_can && notch_active) {
                    #define EC_TAPS     1024    /* 128ms — covers delay + full tail */
                    static float ec_h[EC_TAPS];   /* adaptive FIR coefficients */
                    static int ec_init_done = 0;
                    static int ec_samples = 0;    /* total samples processed */
                    #define EC_MU_FAST  0.15f    /* NLMS step size — fast convergence */
                    #define EC_MU_SLOW  0.03f    /* NLMS step size — steady state */
                    #define EC_FAST_SAMPLES 16000 /* fast phase: 2 seconds */
                    #define EC_DELTA    1000.0f  /* regularization */

                    if (!ec_init_done) {
                        memset(ec_h, 0, sizeof(ec_h));
                        ec_init_done = 1;
                        ec_samples = 0;
                    }

                    /* Need at least EC_TAPS samples in TX buffer */
                    int tx_avail = (g_tx_buf_wr - g_tx_buf_rd) & TX_BUF_MASK;
                    if (tx_avail >= EC_TAPS) {
                        for (int i = 0; i < len; i++) {
                            /* Current RX sample index in the TX timeline:
                               RX[i] was received at the same time as TX was being
                               sent. The echo of TX[t-d] appears in RX[t].
                               We reference TX from the read pointer forward. */
                            int tx_base = (g_tx_buf_rd + i) & TX_BUF_MASK;

                            /* Compute echo estimate and input power */
                            float echo_est = 0.0f;
                            float x_pow = EC_DELTA;
                            for (int j = 0; j < EC_TAPS; j++) {
                                int tx_idx = (tx_base - j + TX_BUF_SIZE) & TX_BUF_MASK;
                                float xj = (float)g_tx_buf[tx_idx];
                                echo_est += ec_h[j] * xj;
                                x_pow += xj * xj;
                            }

                            /* Subtract echo estimate */
                            float rx_f = (float)filtered[i];
                            float error = rx_f - echo_est;

                            /* NLMS tap update — fast mu during initial convergence */
                            float mu = (ec_samples < EC_FAST_SAMPLES) ? EC_MU_FAST : EC_MU_SLOW;
                            float mu_norm = mu / x_pow;
                            for (int j = 0; j < EC_TAPS; j++) {
                                int tx_idx = (tx_base - j + TX_BUF_SIZE) & TX_BUF_MASK;
                                ec_h[j] += mu_norm * error * (float)g_tx_buf[tx_idx];
                            }

                            /* Clamp output */
                            if (error > 32767.0f) error = 32767.0f;
                            if (error < -32768.0f) error = -32768.0f;
                            filtered[i] = (int16_t)error;
                        }
                        ec_samples += len;
                    }

                    /* Advance TX buffer read pointer only when NLMS ran.
                       Otherwise let the buffer accumulate until we have EC_TAPS. */
                    if (tx_avail >= EC_TAPS)
                        g_tx_buf_rd = (g_tx_buf_rd + len) & TX_BUF_MASK;

                    /* Log performance periodically */
                    {
                        static int ec_log_count = 0;
                        ec_log_count += len;
                        if (ec_log_count >= 8000) {
                            ec_log_count = 0;
                            double pre_rms = 0, post_rms = 0;
                            for (int i = 0; i < len; i++) {
                                pre_rms += (double)amp[i] * amp[i];
                                post_rms += (double)filtered[i] * filtered[i];
                            }
                            pre_rms = sqrt(pre_rms / len);
                            post_rms = sqrt(post_rms / len);
                            ME_LOG("[ME] Echo cancel: samples=%d pre_rms=%.0f post_rms=%.0f (%.1f dB)\n",
                                    ec_samples, pre_rms, post_rms,
                                    (pre_rms > 1.0 && post_rms > 1.0) ? 20.0*log10(pre_rms/post_rms) : 0.0);
                        }
                    }
                } else if (notch_active) {
                    notch_filter_apply(&g_notch, filtered, len);
                }
                v90_wait_ja_energy_gate_locked(filtered, len);
                v34_rx(g_v34, filtered, len);
                if (g_v34_fallback_to_v22bis_pending) {
                    int status = g_v34_fallback_status;
                    fprintf(stderr,
                            "[ME] Completing deferred V.34/V.90 fallback after %s; starting V.22bis outside v34_rx()\n",
                            signal_status_to_str(status));
                    cleanup_v34_v90_training_locked();
                    start_v22bis_training();
                    pthread_mutex_unlock(&g_state_mtx);
                    return;
                }
                rx_stage = v34_get_rx_stage(g_v34);
                tx_stage = v34_get_tx_stage(g_v34);
                rx_event = v34_get_rx_event(g_v34);

                if (g_mod == ME_MOD_V90
                    && (rx_stage != g_last_v90_bridge_rx_stage
                        || tx_stage != g_last_v90_bridge_tx_stage
                        || rx_event != g_last_v90_bridge_rx_event))
                {
                    bool new_e_event = (rx_event == V34_EVENT_E
                                        && g_last_v90_bridge_rx_event != V34_EVENT_E);
                    bool new_j_event = (rx_event == V34_EVENT_J
                                        && g_last_v90_bridge_rx_event != V34_EVENT_J);
                    bool new_retrain_event = (rx_event == V34_EVENT_PEER_RETRAIN
                                              && g_last_v90_bridge_rx_event != V34_EVENT_PEER_RETRAIN);

                    fprintf(stderr,
                            "[ME] V.90 bridge: phase3_started=%d v90=%d rx=%s(%d) tx=%s(%d) event=%d s_events=%d\n",
                            g_v90_phase3_started ? 1 : 0,
                            g_v90 ? 1 : 0,
                            v34_rx_stage_name(rx_stage), rx_stage,
                            v34_tx_stage_name(tx_stage), tx_stage,
                            rx_event,
                            g_v90_phase3_s_events);
                    g_last_v90_bridge_rx_stage = rx_stage;
                    g_last_v90_bridge_tx_stage = tx_stage;
                    g_last_v90_bridge_rx_event = rx_event;
                    if (new_retrain_event && g_v90) {
                        /* The peer gave up on Phase 3/4 and is restarting its
                         * handshake.  Follow it: drop our Phase 3/4 state and
                         * go back to waiting for Ja, so the next attempt runs
                         * cleanly instead of us pouring Phase 4 over the
                         * peer's fresh Phase 1/2 (which also corrupts its
                         * bulk-delay/RTD estimate -- see rig/README.md). */
                        bool accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_RETRAIN);

                        g_v90_phase3_s_events = 0;
                        g_v90_pending_dil_valid = false;
                        g_v90_dil_capture_bits = 0;
                        g_v90_dil_capture_search = 0;
                        g_v90_dil_hyp_last_bits = 0;
                        ME_LOG("[ME] V.90: peer retrain detected; following it back to Phase 2 "
                               "(accepted=%d)\n", accepted ? 1 : 0);
                        trace_phase("V90 peer retrain detected, following (accepted=%d)",
                                    accepted ? 1 : 0);
                    }
                    if (new_e_event && g_v90) {
                        bool accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_E);

                        fprintf(stderr,
                                "[ME] V.90 strict RX event=E tx_phase=%d accepted=%d\n",
                                (int)v90_get_tx_phase(g_v90), accepted ? 1 : 0);
                        trace_phase("V90 strict RX event=E accepted=%d",
                                    accepted ? 1 : 0);
                    }
                    if (new_j_event && g_v90) {
                        bool accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_J);

                        if (accepted) {
                            (void)v90_dil_capture_try_v34_hypotheses();
                            v34_v90_arm_phase3_s_detector(g_v34);
                        }

                        fprintf(stderr,
                                "[ME] V.90 strict RX event=J tx_phase=%d accepted=%d\n",
                                (int)v90_get_tx_phase(g_v90), accepted ? 1 : 0);
                        trace_phase("V90 strict RX event=J accepted=%d",
                                    accepted ? 1 : 0);
                    }
                }

                if (g_mod == ME_MOD_V90 && g_v90) {
                    int s_events = v34_get_phase3_s_event_count(g_v34);

                    while (g_v90_phase3_s_events < s_events) {
                        bool accepted;

                        g_v90_phase3_s_events++;
                        accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_S);
                        fprintf(stderr,
                                "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d accepted=%d\n",
                                g_v90_phase3_s_events,
                                (int)v90_get_tx_phase(g_v90),
                                accepted ? 1 : 0);
                        trace_phase("V90 strict RX event=S count=%d accepted=%d",
                                    g_v90_phase3_s_events, accepted ? 1 : 0);
                    }

                    if (!g_v90_pending_dil_valid)
                        (void)v90_dil_capture_try_v34_hypotheses();
                }
                /* RX PCM dump during training */
                {
                    static FILE *rx_dump = NULL;
                    if (!rx_dump) {
                        rx_dump = fopen("/tmp/v34_rx.raw", "wb");
                        if (rx_dump)
                            ME_LOG("[ME] RX PCM dump: /tmp/v34_rx.raw (s16le 8000Hz mono)\n");
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

static bool get_strict_v90_info1a_locked(v90_info1a_t *info)
{
    v34_v90_info1a_t received;

    if (!info || !g_v34
        || !v34_get_v90_received_info1a(g_v34, &received))
        return false;

    info->md = (uint8_t)received.md;
    info->u_info = (uint8_t)received.u_info;
    info->upstream_symbol_rate_code = (uint8_t)received.upstream_symbol_rate_code;
    info->downstream_rate_code = (uint8_t)received.downstream_rate_code;
    info->freq_offset = (int16_t)received.freq_offset;

    /* V.90 Table 10: reserved fields are zero, upstream is one of the six
     * V.34 symbol-rate codes (0=2400 through 5=3429), and downstream PCM is
     * code 6 (8000). A 2400-symbol/s upstream remains a valid V.90 call. */
    return received.raw_12_17 == 0
        && received.raw_32_33 == 0
        && received.u_info > 0
        && received.upstream_symbol_rate_code >= 0
        && received.upstream_symbol_rate_code <= 5
        && received.downstream_rate_code == 6
        && v90_info1a_validate(info);
}

/* Called with g_state_mtx held. */
static void prepare_v90_phase3_locked(void)
{
    v90_info1a_t info1a;

    if (g_mod != ME_MOD_V90 || !g_v34 || g_v90_phase3_started)
        return;

    if (get_strict_v90_info1a_locked(&info1a)) {
        const char *dil_profile = getenv("ME_V90_DIL_PROFILE");

        ME_LOG("[ME] V.90 strict RX event: valid INFO1a U_INFO=%u MD=%u upstream_code=%u downstream_code=%u\n",
               (unsigned)info1a.u_info,
               (unsigned)info1a.md,
               (unsigned)info1a.upstream_symbol_rate_code,
               (unsigned)info1a.downstream_rate_code);
        if (!g_v90_pending_dil_valid && dil_profile) {
            bool loaded = false;

            if (strcmp(dil_profile, "smartlink-adi-qc") == 0)
                loaded = v90_dil_load_smartlink_adi_qc(&g_v90_pending_dil);
            else if (strcmp(dil_profile, "smartlink-adi") == 0)
                loaded = v90_dil_load_smartlink_adi(&g_v90_pending_dil);
            if (loaded) {
                g_v90_pending_dil_valid = true;
                ME_LOG("[ME] V.90: installed %s DIL fallback "
                       "(N=%u LSP=%u LTP=%u)\n",
                       dil_profile,
                       (unsigned)g_v90_pending_dil.n,
                       (unsigned)g_v90_pending_dil.lsp,
                       (unsigned)g_v90_pending_dil.ltp);
                trace_phase("V90 %s DIL fallback installed: N=%u LSP=%u LTP=%u",
                            dil_profile,
                            (unsigned)g_v90_pending_dil.n,
                            (unsigned)g_v90_pending_dil.lsp,
                            (unsigned)g_v90_pending_dil.ltp);
            } else if (strcmp(dil_profile, "smartlink-adi-qc") == 0
                       || strcmp(dil_profile, "smartlink-adi") == 0) {
                ME_LOG("[ME] V.90: %s DIL fallback failed validation\n", dil_profile);
            }
        }
        if (!g_v90) {
            v90_law_t law = (g_law == ME_LAW_ALAW) ? V90_LAW_ALAW : V90_LAW_ULAW;
            g_v90 = v90_init_with_v34(g_v34, law);
            if (g_v90 && g_v90_pending_dil_valid)
                v90_set_dil_descriptor(g_v90, &g_v90_pending_dil);
        }
        if (g_v90) {
            if (g_v92_active) {
                v90_enable_v92_mode(g_v90);
                v90_enable_v92_native_cpu_rx(g_v90);
                v92_cp_rx_init(&g_v92_cp_rx,
                               g_v92_trn2u_points,
                               g_law == ME_LAW_ALAW,
                               v92_live_p4u_frame,
                               NULL);
                v92_trn2u_demod_init(&g_v92_trn2u_demod,
                                     g_v92_trn2u_points,
                                     g_v92_trn2u_lu,
                                     g_law == ME_LAW_ALAW,
                                     &g_v92_cp_rx);
                g_v92_trn2u_active = true;
                ME_LOG("[ME] V.92 native Phase 4 RX enabled: %d-point TRN2u, L_U=%.0f\n",
                       g_v92_trn2u_points, g_v92_trn2u_lu);
            }
            v90_start_phase3(g_v90, info1a.u_info);
            g_v90_phase3_started = true;
            g_v90_completion_deferred_logged = false;
            g_v90_wait_info1_logged = false;
            trace_phase("V90 strict RX event=INFO1A_VALID u_info=%u -> Phase3",
                        (unsigned)info1a.u_info);
        }
    } else {
        v34_v90_info1a_t received;

        /* This runs every tick while we wait, so the "still waiting" message
           is gated behind the log-once flag -- but a genuinely decoded,
           CRC-valid-but-declined INFO1a is a one-time event that must be
           acted on the instant it appears, regardless of whether "still
           waiting" already logged on an earlier tick (it almost always
           has, since Phase 2 takes many ticks). Gate that action on its own
           flag instead of reusing the "waiting" log-once flag. */
        if (v34_get_v90_received_info1a(g_v34, &received)) {
            if (!g_v90_wait_info1_logged) {
                ME_LOG("[ME] V.90 strict RX event: rejecting INFO1a reserved=%02X/%02X U_INFO=%d upstream_code=%d downstream_code=%d\n",
                       received.raw_12_17,
                       received.raw_32_33,
                       received.u_info,
                       received.upstream_symbol_rate_code,
                       received.downstream_rate_code);
                trace_phase("V90 strict RX event=INFO1A_INVALID -> remain Phase2");
                g_v90_wait_info1_logged = true;
            }
            if (!g_v90_fallback_phase4_released
                && received.downstream_rate_code >= 0 && received.downstream_rate_code <= 5
                && g_v34) {
                /* Declined V.90: SpanDSP hands Phase 4 frame ownership
                   entirely to our put_phase4_bit callback the instant one is
                   registered (v34rx.c: "the project-owned Table 14 framer
                   owns CP length, CRC, fill and semantics from this boundary
                   on"), bypassing its own native MP-frame CRC/semantic
                   checks. Our CP/Table-14 framer only understands V.90's CP
                   frame layout, so it rejects every genuine V.34 MP frame
                   the peer sends here -- and each rejection also drops
                   SpanDSP's own MP hypothesis lock via
                   v34_reject_v90_phase4_hypothesis(), even though SpanDSP's
                   native semantics might have accepted the same frame. Live
                   interop showed exactly this: TRN converged cleanly (70%
                   ones-lock) and MP hypotheses kept locking, but every one
                   was rejected in a loop. Unregistering the callback lets
                   SpanDSP decode Phase 4 MP frames the standard V.34 way.
                   Gated on its own flag (not the "waiting" log-once flag)
                   since that flag is set on the very first tick, long
                   before INFO1a actually arrives. */
                v34_set_put_phase4_bit(g_v34, NULL, NULL);
                ME_LOG("[ME] V.90 declined by peer; releasing Phase 4 MP frame decode to standard V.34 (was routed to the V.90 CP framer)\n");
                g_v90_fallback_phase4_released = true;
            }
        } else if (!g_v90_wait_info1_logged) {
            ME_LOG("[ME] V.90: waiting for CRC-valid INFO1a before Phase 3 TX\n");
            g_v90_wait_info1_logged = true;
        }
    }
}

/* Preserve six-symbol data-frame boundaries across arbitrary RTP pull sizes. */
static void generate_v90_data_codewords_locked(uint8_t *codewords, int len)
{
    int out_pos = 0;

    while (out_pos < len) {
        int available;

        if (g_v90_data_frame_pos >= V90_DATA_FRAME_LEN) {
            uint8_t data_in[V90_DATA_FRAME_LEN];
            int needed;
            int consumed;

            memset(g_v90_data_frame, pcm_idle(), sizeof(g_v90_data_frame));
            needed = v90_data_input_bytes_needed(g_v90);
            ds_tx_fill_bytes(&g_data_stack, data_in, needed);
            consumed = 0;
            if (v90_tx_data_frame_codewords(g_v90,
                                            g_v90_data_frame,
                                            data_in,
                                            needed,
                                            &consumed,
                                            true) != V90_DATA_FRAME_LEN
                || consumed != needed) {
                ME_LOG("[ME] V.90 data mapper failed to produce one frame\n");
                memset(g_v90_data_frame, pcm_idle(), sizeof(g_v90_data_frame));
            }
            g_v90_data_frame_pos = 0;
        }

        available = V90_DATA_FRAME_LEN - g_v90_data_frame_pos;
        if (available > len - out_pos)
            available = len - out_pos;
        memcpy(codewords + out_pos,
               g_v90_data_frame + g_v90_data_frame_pos,
               (size_t)available);
        g_v90_data_frame_pos += available;
        out_pos += available;
    }
}

static void enter_v90_data_locked(void)
{
    int downstream_rate;
    int upstream_rate;

    if (g_state != ME_TRAINING || !g_v90 || !g_v34)
        return;
    upstream_rate = v34_get_current_bit_rate(g_v34);
    downstream_rate = (v90_data_bits_per_frame(g_v90) * 8000) / 6;
    if (downstream_rate <= 0)
        downstream_rate = V90_RATE_BPS;
    data_stack_start_online(downstream_rate, g_calling_party);
    g_state = ME_DATA;
    g_phase_start_ms = 0;
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    ME_LOG("[ME] V.90 startup complete (upstream V.34 %d bps, downstream PCM %d bps)\n",
           upstream_rate, downstream_rate);
    trace_phase("V90 enter DATA after B1d: upstream=%d downstream=%d",
                upstream_rate, downstream_rate);
    if (g_data_framing != DS_FRAMING_V42) {
        g_data_connect_reported = true;
        di_on_connected(downstream_rate);
    } else {
        ME_LOG("[ME] V.90 carrier ready; waiting for V.42 LAPM\n");
    }
}

/* Called with g_state_mtx held. Returns true when codewords were generated
 * without a linear-PCM round trip. */
static bool generate_v90_raw_codewords_locked(uint8_t *codewords, int len)
{
    if (!codewords || len <= 0 || g_mod != ME_MOD_V90)
        return false;

    if (g_state == ME_TRAINING && g_v34) {
        int pos = 0;

        prepare_v90_phase3_locked();
        if (!g_v90_phase3_started || !g_v90 || v90_using_internal_v34_tx(g_v90))
            return false;

        while (pos < len && v90_get_tx_phase(g_v90) != V90_TX_DATA) {
            v90_tx_phase_t phase_before = v90_get_tx_phase(g_v90);

            if (v90_phase3_tx_codewords(g_v90, codewords + pos, 1) != 1)
                return false;
            pos++;

            /* The project-owned V.90 transmitter replaces SpanDSP's Phase 3
               waveform, so SpanDSP cannot observe the downstream DIL -> Ri
               boundary itself.  Hand its primary-channel receiver into the
               native Phase 4 path at that exact boundary; otherwise it stays
               parked in PHASE3_WAIT_S and never delivers the peer's CPt bits
               to v90_live_cp_bit(). */
            if (phase_before < V90_TX_RI
                && v90_get_tx_phase(g_v90) >= V90_TX_RI) {
                ME_LOG("[ME] V.90 Phase 3 complete; enabling native upstream Phase 4 receiver\n");
                v90_cp_live_note_phase4_hint_locked();
                v34_force_phase4(g_v34);
                v34_force_v90_phase4_cp_rx(g_v34);
            }

            /* v90.c returns to WAIT_JA after a bounded Jd-without-S interval.
               That state change is the recovery request for the live engine:
               re-enter the V.34/V.90 Phase-2 control exchange immediately,
               instead of sending silence while the peer expects INFO0d. */
            if (phase_before == V90_TX_JD
                && v90_get_tx_phase(g_v90) == V90_TX_WAIT_JA) {
                (void) restart_v90_phase2_locked();
                return false;
            }
        }

        /* Keep the wrapped V.34 state machine advancing while its waveform is
         * replaced by the authoritative V.90 codeword stream. */
        {
            int16_t discard[len];
            v34_tx(g_v34, discard, len);
        }

        if (v90_get_tx_phase(g_v90) == V90_TX_DATA) {
            enter_v90_data_locked();
            if (pos < len)
                generate_v90_data_codewords_locked(codewords + pos, len - pos);
        }
        return true;
    }

    if (g_state == ME_DATA) {
        if (!g_v90)
            return false;
        generate_v90_data_codewords_locked(codewords, len);
        return true;
    }

    return false;
}

static void buffer_tx_samples_for_echo(const int16_t *amp, int len)
{
    if (!amp || len <= 0 || (g_mod != ME_MOD_V34 && g_mod != ME_MOD_V90))
        return;
    pthread_mutex_lock(&g_state_mtx);
    if (g_echo_can) {
        for (int i = 0; i < len; i++) {
            g_tx_buf[g_tx_buf_wr] = amp[i];
            g_tx_buf_wr = (g_tx_buf_wr + 1) & TX_BUF_MASK;
        }
    }
    pthread_mutex_unlock(&g_state_mtx);
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
        mix_v8_guard_tone(amp, len);
        for (int i = 0; i < len; i++)
            g_v8_tx_energy += (int64_t)amp[i] * amp[i];
        g_v8_tx_count += len;
        if (g_v8_tx_count >= 8000) {
            double rms = sqrt((double)g_v8_tx_energy / g_v8_tx_count);
            ME_LOG("[ME] V.8 tx: RMS=%.1f (%d samples)\n",
                    rms, g_v8_tx_count);
            g_v8_tx_energy = 0;
            g_v8_tx_count  = 0;
        }
        break;

    case ME_TRAINING:
        /*
         * V.34/V.22bis training: SpanDSP handles the training state machine
         * internally (Phase 2 DPSK INFO exchange, tone probing, Phase 3/4
         * equalizer training). The put_bit callback fires SIG_STATUS_CARRIER_UP
         * when training completes, transitioning us to ME_DATA.
         */
        if (g_mod == ME_MOD_V91) {
            uint8_t pcm_out[len];

            pthread_mutex_lock(&g_state_mtx);
            if (v91_live_generate_codewords_locked(pcm_out, len)) {
                for (int i = 0; i < len; i++)
                    amp[i] = pcm_to_linear(pcm_out[i]);
            }
            pthread_mutex_unlock(&g_state_mtx);
        } else if (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) {
            pthread_mutex_lock(&g_state_mtx);
            if ((g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) && g_v34) {
                /* V.90: detect Phase 2→3 and create the raw-codeword state. */
                prepare_v90_phase3_locked();

                if (g_v90_phase3_started && g_v90) {
                    if (v90_using_internal_v34_tx(g_v90)) {
                        v34_tx(g_v34, amp, len);
                    } else {
                        /* V.90 Phase 3: generate PCM codewords for actual RTP output */
                        v90_phase3_tx(g_v90, amp, len);
                        /* Run v34_tx into discard but protect RX state from being
                           overwritten by V.34 TX Phase 3/4 transitions */
                        int16_t discard[len];
                        v34_tx(g_v34, discard, len);
                        /* V.34 TX may have clobbered RX stage — don't let it;
                           the V.90 RX flow controls its own stage transitions. */
                        /* (RX stage restoration handled by v34rx directly) */
                    }
                } else if (g_mod == ME_MOD_V90
                           && v34_get_tx_stage(g_v34) >= V34_TX_STAGE_FIRST_S) {
                    /* tx_stage only reaches FIRST_S after SpanDSP has accepted
                       a CRC-valid INFO1a and called s_not_s_baud_init(); the
                       only way g_v90_phase3_started can still be false here is
                       that our own strict validator (get_strict_v90_info1a_locked)
                       rejected that same INFO1a — i.e. the analogue modem's
                       Table 10 downstream code was not 6, meaning it declined
                       V.90 PCM and committed to plain V.34 at the negotiated
                       baud (V.90 §9.2.1.1.8). SpanDSP is already generating a
                       genuine V.34 Phase 3/4 waveform for this; previously we
                       discarded it and sent silence instead, so the analogue
                       modem never received anything back and every call in
                       this state ran out the clock to NO CARRIER (observed
                       live with a CX93001, 2026-07-19). Transmit it for real. */
                    if (!g_v90_fallback_v34_logged) {
                        ME_LOG("[ME] V.90 declined by peer INFO1a; continuing as plain V.34 (transmitting Phase 3/4 waveform instead of muting)\n");
                        trace_phase("V90 declined -> V34 fallback: transmitting Phase3/4");
                        g_v90_fallback_v34_logged = true;
                    }
                    v34_tx(g_v34, amp, len);
                } else {
                    v34_tx(g_v34, amp, len);
                }
            }
            pthread_mutex_unlock(&g_state_mtx);
            /* TX PCM dump + RMS diagnostic during training */
            {
                static FILE *tx_dump = NULL;
                static int64_t tx_energy = 0;
                static int tx_count = 0;
                if (!tx_dump) {
                    tx_dump = fopen("/tmp/v34_tx.raw", "wb");
                    if (tx_dump)
                        ME_LOG("[ME] TX PCM dump: /tmp/v34_tx.raw (s16le 8000Hz mono)\n");
                }
                if (tx_dump)
                    fwrite(amp, sizeof(int16_t), len, tx_dump);
                for (int i = 0; i < len; i++)
                    tx_energy += (int64_t)amp[i] * amp[i];
                tx_count += len;
                if (tx_count >= 8000) {
                    double rms = sqrt((double)tx_energy / tx_count);
                    ME_LOG("[ME] Training TX: RMS=%.1f (%d samples)\n",
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
        if (g_mod == ME_MOD_V91) {
            uint8_t pcm_out[len];

            pthread_mutex_lock(&g_state_mtx);
            if (v91_live_generate_codewords_locked(pcm_out, len)) {
                for (int i = 0; i < len; i++)
                    amp[i] = pcm_to_linear(pcm_out[i]);
            }
            pthread_mutex_unlock(&g_state_mtx);
        } else if (g_mod == ME_MOD_V34) {
            /* V.34 full duplex data TX */
            pthread_mutex_lock(&g_state_mtx);
            if (g_mod == ME_MOD_V34 && g_v34)
                v34_tx(g_v34, amp, len);
            pthread_mutex_unlock(&g_state_mtx);
        } else if (g_mod == ME_MOD_V90) {
            uint8_t pcm_out[len];
            bool generated;

            pthread_mutex_lock(&g_state_mtx);
            generated = generate_v90_raw_codewords_locked(pcm_out, len);
            pthread_mutex_unlock(&g_state_mtx);
            if (generated) {
                for (int i = 0; i < len; i++)
                    amp[i] = pcm_to_linear(pcm_out[i]);
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
    buffer_tx_samples_for_echo(amp, len);
}

void me_rx_g711(const uint8_t *codewords, int count)
{
    int offset;
    bool raw_v91;

    if (!codewords || count <= 0)
        return;

    pthread_mutex_lock(&g_state_mtx);
    g_g711_rx_octets += (uint64_t)count;
    raw_v91 = (g_mod == ME_MOD_V91
               && (g_state == ME_TRAINING || g_state == ME_DATA));
    if (raw_v91)
        v91_live_receive_codewords_locked(codewords, count);
    if (g_v92_trn2u_active && g_v92_active && g_v90
        && g_state == ME_TRAINING
        && v90_get_tx_phase(g_v90) >= V90_TX_TRN2D
        && v90_get_tx_phase(g_v90) < V90_TX_DATA) {
        (void)v92_trn2u_demod_feed(&g_v92_trn2u_demod, codewords, count);
    }
    pthread_mutex_unlock(&g_state_mtx);
    if (g_g711_rx_tap)
        (void)fwrite(codewords, 1, (size_t)count, g_g711_rx_tap);

    if (raw_v91) {
        uint8_t buf[256];
        int n;

        while ((n = dring_read(&upstream_ring, buf, sizeof(buf))) > 0)
            di_write_data(buf, n);
        return;
    }

    for (offset = 0; offset < count; ) {
        int16_t linear[320];
        int chunk = count - offset;

        if (chunk > (int)(sizeof(linear) / sizeof(linear[0])))
            chunk = (int)(sizeof(linear) / sizeof(linear[0]));
        for (int i = 0; i < chunk; i++)
            linear[i] = pcm_to_linear(codewords[offset + i]);
        me_rx_audio(linear, chunk);
        offset += chunk;
    }
}

int me_tx_g711(uint8_t *codewords, int count)
{
    int offset;
    uint64_t raw_octets = 0;
    uint64_t linear_octets = 0;

    if (!codewords || count <= 0)
        return 0;

    if (voice_tx_test_fill(codewords, count)) {
        pthread_mutex_lock(&g_state_mtx);
        g_g711_tx_octets += (uint64_t)count;
        pthread_mutex_unlock(&g_state_mtx);
        if (g_g711_tx_tap)
            (void)fwrite(codewords, 1, (size_t)count, g_g711_tx_tap);
        return count;
    }

    for (offset = 0; offset < count; ) {
        int16_t linear[320];
        int chunk = count - offset;
        bool raw_pcm;

        if (chunk > (int)(sizeof(linear) / sizeof(linear[0])))
            chunk = (int)(sizeof(linear) / sizeof(linear[0]));

        pthread_mutex_lock(&g_state_mtx);
        raw_pcm = v91_live_generate_codewords_locked(codewords + offset, chunk)
               || generate_v90_raw_codewords_locked(codewords + offset, chunk);
        pthread_mutex_unlock(&g_state_mtx);

        if (raw_pcm) {
            for (int i = 0; i < chunk; i++)
                linear[i] = pcm_to_linear(codewords[offset + i]);
            buffer_tx_samples_for_echo(linear, chunk);
            raw_octets += (uint64_t)chunk;
        } else {
            me_tx_audio(linear, chunk);
            for (int i = 0; i < chunk; i++)
                codewords[offset + i] = linear_to_pcm(linear[i]);
            linear_octets += (uint64_t)chunk;
        }
        offset += chunk;
    }

    pthread_mutex_lock(&g_state_mtx);
    g_g711_tx_octets += (uint64_t)count;
    g_g711_raw_v90_tx_octets += raw_octets;
    g_g711_linear_tx_octets += linear_octets;
    pthread_mutex_unlock(&g_state_mtx);
    if (g_g711_tx_tap)
        (void)fwrite(codewords, 1, (size_t)count, g_g711_tx_tap);
    return count;
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
    if (g_data_link_failed && g_state == ME_DATA) {
        g_data_link_failed = false;
        ME_LOG("[ME] V.42 failure requested call teardown\n");
        trace_phase("V42 link failure -> HANGUP");
        g_state = ME_HANGUP;
    }
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

void me_get_diag_snapshot(me_diag_snapshot_t *snapshot)
{
    if (!snapshot)
        return;

    pthread_mutex_lock(&g_state_mtx);
    snapshot->state = g_state;
    snapshot->modulation = g_mod;
    snapshot->law = g_law;
    snapshot->calling_party = g_calling_party ? 1 : 0;
    snapshot->v34_rx_stage = g_last_rx_stage;
    snapshot->v34_tx_stage = g_last_tx_stage;
    snapshot->v90_bridge_rx_stage = g_last_v90_bridge_rx_stage;
    snapshot->v90_bridge_tx_stage = g_last_v90_bridge_tx_stage;
    snapshot->v90_bridge_rx_event = g_last_v90_bridge_rx_event;
    snapshot->v90_phase3_started = g_v90_phase3_started ? 1 : 0;
    snapshot->v90_phase3_s_events = g_v90_phase3_s_events;
    snapshot->v90_dil_valid = g_v90_pending_dil_valid ? 1 : 0;
    snapshot->v90_cp_input_bits = g_v90_cp_rx.input_bits;
    snapshot->v90_cp_valid_frames = g_v90_cp_rx.valid_frames;
    snapshot->v90_cp_rejected_frames = g_v90_cp_rx.rejected_frames;
    snapshot->v92_active = g_v92_active ? 1 : 0;
    snapshot->v92_trn2u_active = g_v92_trn2u_active ? 1 : 0;
    snapshot->v92_trn2u_symbols = g_v92_trn2u_demod.symbols;
    snapshot->v92_trn2u_longest_ones =
        g_v92_trn2u_demod.longest_descrambled_one_run;
    snapshot->v92_cp_input_bits = g_v92_cp_rx.input_bits;
    snapshot->v92_cp_valid_frames = g_v92_cp_rx.valid_frames;
    snapshot->v92_cp_rejected_frames = g_v92_cp_rx.rejected_frames;
    snapshot->phase_elapsed_ms = (g_phase_start_ms != 0 && g_state != ME_IDLE)
        ? (trace_now_ms() - g_phase_start_ms)
        : 0;
    snapshot->g711_rx_octets = g_g711_rx_octets;
    snapshot->g711_tx_octets = g_g711_tx_octets;
    snapshot->g711_raw_v90_tx_octets = g_g711_raw_v90_tx_octets;
    snapshot->g711_linear_tx_octets = g_g711_linear_tx_octets;
    pthread_mutex_unlock(&g_state_mtx);
}

void me_set_law(me_law_t law)
{
    pthread_mutex_lock(&g_state_mtx);
    g_law = law;
    pthread_mutex_unlock(&g_state_mtx);
    ME_LOG("[ME] PCM law set to %s\n",
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
