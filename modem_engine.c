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
#include "v90_analogue_phase3.h"
#include "v90_analogue_linear.h"
#include "v90_analogue_fse.h"
#include "v90_analogue_sd.h"
#include "v90_sounder.h"
#include "v90_dil_presets.h"
#include "p3_demod.h"
#include "v92_cp_rx.h"
#include "v92_p3_rx.h"
#include "v92_trn2u.h"
#include "v92_upstream_rx.h"

#include <spandsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <stdarg.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <errno.h>

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
    V34_RX_STAGE_V90_CP,
};

/* MUST stay in sync, value for value, with enum v34_tx_stages_e in
 * spandsp-master/src/spandsp/private/v34.h -- v34_get_tx_stage() returns a
 * value produced inside spandsp, so any drift silently shifts every stage
 * comparison rather than failing to build. A missing INFOMARKSA put every
 * name from V90_WAIT_TONE_A on one low, which made the ">= FIRST_S" V.34
 * fallback test fire a stage early, on INFO0_RETRY (observed live 2026-07-21).
 * The HDX_* stages that follow MP in spandsp are deliberately not copied --
 * nothing here refers to them, and they sit above every value we compare. */
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
    V34_TX_STAGE_POST_INFO0_RESUME_A,
    V34_TX_STAGE_POST_L2_WAIT_TONE_B,
    V34_TX_STAGE_POST_L2_A,
    V34_TX_STAGE_POST_L2_NOT_A,
    V34_TX_STAGE_A_SILENCE,
    V34_TX_STAGE_PRE_INFO1_A,
    V34_TX_STAGE_INFOMARKSA,
    V34_TX_STAGE_V90_WAIT_TONE_A,
    V34_TX_STAGE_V90_WAIT_INFO1A,
    V34_TX_STAGE_V90_WAIT_RX_L2,
    V34_TX_STAGE_V90_WAIT_TONE_A_REV,
    V34_TX_STAGE_V90_B_REV_DELAY,
    V34_TX_STAGE_V90_B_REV_10MS,
    V34_TX_STAGE_V90_PHASE2_B,
    V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN,
    /* §9.5.1.2 retrain response: a Phase 2 state.  It must stay below
       V34_TX_STAGE_FIRST_S here and in spandsp's private/v34.h or the
       "declined V.90" demotion guard misreads it as late training. */
    V34_TX_STAGE_V90_RETRAIN_SILENCE,
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
    /* V.90 §9.2.1.1.8 V.34 fallback silent wait (call-modem role).  In
       spandsp's enum this sits after the 18 uncopied HDX_* stages, so the
       value is pinned explicitly here: MP(47) + 18 HDX + 1 = 66. */
    V34_TX_STAGE_V34_FALLBACK_WAIT_J = 66,
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
    V34_EVENT_INFOMARKSA_SEEN,
    V34_EVENT_PEER_RENEG_S,
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

static int me_span_flow_level(void);

/* In-place Sd retries already used on the current Phase-3 attempt. */
static unsigned g_v90_jd_resync_retries = 0;

/* ME_V90_UPSTREAM_MAX_BPS caps the upstream rate we offer in MP.  It has to
 * reach the receiver's own preparation as well: cap the MP mask alone and the
 * peer transmits at the capped rate while our B1 template is still built for
 * the uncapped one, so acquisition never correlates and the upstream is dead
 * for the whole call. */
static int me_v90_upstream_cap(int rate)
{
    const char *cap = getenv("ME_V90_UPSTREAM_MAX_BPS");

    if (cap && atoi(cap) > 0 && atoi(cap) < rate)
        return atoi(cap);
    return rate;
}

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
    case V34_RX_STAGE_V90_CP:            return "V90_CP";
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
    case V34_TX_STAGE_POST_INFO0_RESUME_A:         return "POST_INFO0_RESUME_A";
    case V34_TX_STAGE_POST_L2_WAIT_TONE_B:         return "POST_L2_WAIT_TONE_B";
    case V34_TX_STAGE_POST_L2_A:                   return "POST_L2_A";
    case V34_TX_STAGE_POST_L2_NOT_A:               return "POST_L2_NOT_A";
    case V34_TX_STAGE_A_SILENCE:                   return "A_SILENCE";
    case V34_TX_STAGE_PRE_INFO1_A:                 return "PRE_INFO1_A";
    case V34_TX_STAGE_INFOMARKSA:                  return "INFOMARKSA";
    case V34_TX_STAGE_V90_WAIT_TONE_A:             return "V90_WAIT_TONE_A";
    case V34_TX_STAGE_V90_WAIT_INFO1A:             return "V90_WAIT_INFO1A";
    case V34_TX_STAGE_V90_WAIT_RX_L2:              return "V90_WAIT_RX_L2";
    case V34_TX_STAGE_V90_WAIT_TONE_A_REV:         return "V90_WAIT_TONE_A_REV";
    case V34_TX_STAGE_V90_B_REV_DELAY:             return "V90_B_REV_DELAY";
    case V34_TX_STAGE_V90_B_REV_10MS:              return "V90_B_REV_10MS";
    case V34_TX_STAGE_V90_PHASE2_B:                return "V90_PHASE2_B";
    case V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN:     return "V90_PHASE2_B_INFO0_SEEN";
    case V34_TX_STAGE_V90_RETRAIN_SILENCE:         return "V90_RETRAIN_SILENCE";
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
    case V34_TX_STAGE_V34_FALLBACK_WAIT_J:         return "V34_FALLBACK_WAIT_J";
    default:                                       return "UNKNOWN";
    }
}

static bool valid_v34_baud(int baud)
{
    return baud == 2400 || baud == 2743 || baud == 2800 ||
           baud == 3000 || baud == 3200 || baud == 3429;
}

static float v34_primary_carrier_hz(int baud, bool high)
{
    float exact_baud;
    float ratio;

    /* V.34 §5.2-5.3/Table 2.  Keep the rational symbol rates: the labels
       2743 and 3429 are rounded names, not the DSP clock values. */
    switch (baud) {
    case 2400: exact_baud = 2400.0f; break;
    case 2743: exact_baud = 2400.0f*8.0f/7.0f; break;
    case 2800: exact_baud = 2400.0f*7.0f/6.0f; break;
    case 3000: exact_baud = 2400.0f*5.0f/4.0f; break;
    case 3200: exact_baud = 2400.0f*4.0f/3.0f; break;
    case 3429: exact_baud = 2400.0f*10.0f/7.0f; break;
    default: return 0.0f;
    }
    if (baud == 2400)
        ratio = high ? 3.0f/4.0f : 2.0f/3.0f;
    else if (baud == 2743 || baud == 2800 || baud == 3000)
        ratio = high ? 2.0f/3.0f : 3.0f/5.0f;
    else if (baud == 3200)
        ratio = high ? 3.0f/5.0f : 4.0f/7.0f;
    else
        ratio = 4.0f/7.0f;
    return exact_baud*ratio;
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

/* V.90 data-mode character bit order.  V.14 mandates data bits LSB-first
 * within each start/stop-framed character, ds_tx_load follows that, and
 * live A/B against SmartLink (2026-07-23, artifacts/v90-hardware/
 * 20260723T055443Z-pty_soak) confirmed the spec order is CORRECT: with the
 * experimental msb setting the far DTE receives every byte bit-reversed
 * ('D' 0x44 arrives as 0x22) in otherwise perfect sequence, i.e. SmartLink
 * assembles characters LSB-first like the spec says.  Two traps recorded so
 * nobody re-introduces this: (1) slmodemd's DSP-layer "rx pattern" debug
 * bytes print bit-reversed relative to what its DTE receives -- do not infer
 * the wire convention from that log; (2) the missing DTE CONNECT that
 * started this chase was slmodemd's V.42 auto-detect waiting forever on raw
 * test data, fixed by AT\N0 in the dial string, not a bit-order problem.
 * (3) reversing at the mapper-byte boundary is doubly wrong -- it reverses
 * 8-bit windows of the *framed* stream, relocating start bits.
 * ME_V90_DATA_BIT_ORDER=msb keeps the experiment available; scoped to V.90
 * so the V.22bis/V.34 fallback paths always keep spec order. */
static bool me_v90_data_bit_order_msb(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *value = getenv("ME_V90_DATA_BIT_ORDER");

        cached = (value && strcasecmp(value, "msb") == 0) ? 1 : 0;
    }
    return cached != 0;
}

static uint8_t me_reverse8(uint8_t v)
{
    v = (uint8_t)(((v & 0xF0) >> 4) | ((v & 0x0F) << 4));
    v = (uint8_t)(((v & 0xCC) >> 2) | ((v & 0x33) << 2));
    v = (uint8_t)(((v & 0xAA) >> 1) | ((v & 0x55) << 1));
    return v;
}

static me_modulation_t g_mod;   /* defined below with the engine state */

static bool me_v90_reverse_dte_bytes_locked(void)
{
    return g_mod == ME_MOD_V90 && me_v90_data_bit_order_msb();
}

static int data_stack_pull_dte_byte(void *user_data)
{
    uint8_t byte;

    (void)user_data;
    if (dring_read(&downstream_ring, &byte, 1) != 1)
        return -1;
    if (me_v90_reverse_dte_bytes_locked())
        byte = me_reverse8(byte);
    return byte;
}

static void data_stack_push_dte_byte(void *user_data, uint8_t byte)
{
    (void)user_data;
    if (me_v90_reverse_dte_bytes_locked())
        byte = me_reverse8(byte);
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
    case DS_LINK_DETECTED:
        ME_LOG("[ME] V.42 detection succeeded; entering LAPM establishment\n");
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

/* Set once V.22bis has actually trained, so a later carrier drop is read as a
 * lost connection rather than as part of the handshake. */
static bool g_v22bis_trained;

static void v22bis_put_bit_cb(void *user_data, int bit)
{
    (void)user_data;
    if (bit < 0) {
        /* CARRIER_UP is not a connection.  V.22bis brings the carrier up at
         * the start of its own training sequence, and taking that as the
         * connect reported CONNECT to the DTE while the modem had not trained
         * -- measured over a real analogue line, where both ends reported
         * "training complete" 8 ms after V.8 and then dropped 60 ms later on
         * the CARRIER_DOWN that follows the V.8 tail.  Only
         * TRAINING_SUCCEEDED means trained. */
        if (bit == SIG_STATUS_CARRIER_UP) {
            ME_LOG("[ME] V.22bis carrier up\n");
        } else if (bit == SIG_STATUS_TRAINING_SUCCEEDED) {
            g_v22bis_trained = true;
            on_training_complete(ME_MOD_V22BIS, 2400, "V.22bis");
        } else if (bit == SIG_STATUS_TRAINING_FAILED
                   || (bit == SIG_STATUS_CARRIER_DOWN && g_v22bis_trained)) {
            ME_LOG("[ME] V.22bis fallback failed (%s), hanging up\n",
                    signal_status_to_str(bit));
            trace_phase("V22BIS training failed (%s) -> hangup",
                        signal_status_to_str(bit));
            me_hangup();
            return;
        } else if (bit == SIG_STATUS_CARRIER_DOWN) {
            /* Before training: the far end has stopped transmitting for the
             * moment.  Keep waiting -- the training timeout still bounds it. */
            ME_LOG("[ME] V.22bis carrier down before training; still waiting\n");
        }
        /*endif*/
        return;
    }

    ds_rx_put_bit(&g_data_stack, bit);
}

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static me_state_t      g_state     = ME_IDLE;
static me_modulation_t g_mod       = ME_MOD_NONE;   /* tentatively declared above the DTE callbacks */
static pthread_mutex_t g_state_mtx;
static bool            g_calling_party = false; /* false=answerer, true=caller */
static bool            g_invert_v34_role = false; /* debug override via env */
static int             g_v8_answer_tone = MODEM_CONNECT_TONES_ANSAM_PR;
static int             g_v8_active_answer_tone = MODEM_CONNECT_TONES_ANSAM_PR;
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
static bool           g_v90_reject_info1a_logged = false;
static bool           g_v90_fallback_v34_logged = false;
/* Consecutive Phase 3 attempts that ended without a CRC-valid Ja descriptor.
 * Per call, not per process -- this server runs many calls in one. */
static int            g_v90_ja_failed_attempts = 0;
/* g_rx_audio_samples at the FIRST Phase 3 entry of this call, 0 if not yet.
 * Deliberately the AUDIO clock and not the wall clock: it is the same quantity
 * in a live call and in a replay, so this trigger can be tested offline at all
 * -- a wall-clock deadline never elapses under `--fast` and would have been
 * verifiable only against the rig.  The
 * Ja deadline is cumulative across retrains rather than per attempt: the peer
 * retrains at 1.88-2.24 s (§34) and a healthy descriptor arrives a median
 * 0.3 s after Phase 3 entry, so any per-attempt deadline long enough to be
 * safe is longer than the peer's own patience and can never fire first. */
static uint64_t       g_v90_phase3_first_samples = 0;

/* How many consecutive Phase 3 attempts may end without a CRC-valid Ja
 * descriptor before we stop asking for V.90 and continue as plain V.34.
 *
 * §9.3.1.3 has the digital modem transmit Sd once it detects Ja, and gives it
 * no way to give up -- so this bound is ours, and it is set from what the peer
 * does.  Measured on this rig (docs/v90_phase3_s_and_rbs_false_positive.md
 * §34): the peer waits 1.88-2.24 s in WaitForSd, retrains when no Sd arrives,
 * and on the THIRD such cycle gives up itself -- `drop to V34 requested` --
 * after which its INFO1a is a plain V.34 offer that our strict validator
 * rejects, which is the misleading "V.90 declined by peer INFO1a" line.
 *
 * The threshold is 3, and it was measured rather than reasoned.  Every capture
 * in artifacts/ predates this code and so records exactly the behaviour that
 * prices it: 1345 calls, of which 325 reach two consecutive Ja failures.
 * Conceding at each threshold would abandon this many calls that went on to
 * recover a descriptor anyway (docs §35l, tools/ja_concede_cost.py):
 *
 *     2 -> 59 of 325 (18.2%), SEVEN of which reached V.90 data mode
 *     3 ->  1 of 171 ( 0.6%), none of which reached data mode
 *     4 ->  0 of  67
 *
 * So 2 -- the value this shipped with for an afternoon, chosen because the
 * peer gives up at 3 and it looked like a free cycle -- throws away nearly a
 * fifth of the calls it fires on.  Attempt 3 recovers Ja often enough to be
 * worth waiting for, and this modem's own corpus says so.
 *
 * At 3 the concession no longer saves time: it fires about when the peer gives
 * up itself.  What it still buys is that the decision, and the log line, are
 * ours -- instead of the peer dropping to V.34 and our strict validator
 * rejecting its INFO1a, reported as "V.90 declined by peer INFO1a" (§34),
 * which is three retrains downstream of the actual cause.
 *
 * ME_V90_JA_CONCEDE_ATTEMPTS=0 disables conceding entirely. */
/* Seconds of Phase 3, cumulative over a call, after which we stop asking for
 * V.90 if no CRC-valid Ja descriptor has arrived.
 *
 * This is the trigger that works, and the attempt count below is the one that
 * does not: an attempt ends when the PEER retrains, so that counter runs on
 * the peer's clock and cannot decide before it does -- measured live, at a
 * threshold of 3 our concession never fired once in four calls because the
 * peer declared first every time (docs §35m).
 *
 * Priced on the corpus the same way as §35l, anchored at Phase 3 entry, over
 * 1286 calls that entered Phase 3 (tools/ja_deadline_cost.py).  A healthy
 * descriptor arrives a median 0.3 s after Phase 3 entry (p90 6.6 s, max
 * 40.6 s), and conceding at T would abandon:
 *
 *      8 s -> 56 calls that recovered anyway, FOUR of which reached data mode
 *     15 s ->  8, one of which reached data mode
 *     20 s ->  3, NONE of which reached data mode      <- default
 *     30 s ->  1, none
 *     45 s ->  0
 *
 * 20 s is the knee: the last value that loses no call which would have reached
 * V.90 data mode, while still firing on 167 calls that were never going to get
 * a descriptor, a median 10.4 s before they gave up by themselves.  0 disables
 * the deadline. */
static int v90_ja_deadline_seconds(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *env = getenv("ME_V90_JA_DEADLINE_SEC");

        cached = 20;
        if (env && *env) {
            char *end;
            long parsed = strtol(env, &end, 10);

            if (end != env && *end == '\0' && parsed >= 0 && parsed <= 600)
                cached = (int) parsed;
        }
    }
    return cached;
}

static int v90_ja_concede_attempts(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *env = getenv("ME_V90_JA_CONCEDE_ATTEMPTS");

        cached = 3;
        if (env && *env) {
            char *end;
            long parsed = strtol(env, &end, 10);

            if (end != env && *end == '\0' && parsed >= 0 && parsed <= 32)
                cached = (int) parsed;
        }
    }
    return cached;
}

static bool           g_v90_fallback_phase4_released = false;
static int            g_v90_phase3_s_events = 0;
static unsigned        g_v90_phase2_restarts = 0;
/* Set when a Phase 2 restart is entered from data mode -- a V.90 §9.5 /
 * V.34 §11.5 retrain rather than a failed startup.  §11.5 says only "turn
 * OFF circuit 106, clamp circuit 104 to binary one": the DTE data is held,
 * the physical layer retrains, and the error-control link above it survives.
 * Re-running data_stack_start_online() on the way back in would tear LAPM
 * down and force a fresh XID against a peer whose link is still up, so the
 * re-entry path consults this instead. */
static bool            g_retrain_from_data = false;
static v90_cp_rx_t    g_v90_cp_rx;
/* V.92 is only selected after both INFO0 frames explicitly advertise it.
 * V.8's QC packet is a hint, not a data-pump selection. */
static bool           g_v92_v8_offered = false;
static bool           g_v92_info0_local_advertised = false;
static bool           g_v92_info0_peer_capable = false;
static bool           g_v92_info0_peer_short_phase2 = false;
static bool           g_v92_info0_mutual = false;
static bool           g_v92_info0_peer_logged = false;
static bool           g_v92_active = false;

/* Stop asking for V.90 and let the restarted Phase 2 be plain V.34.
 * §9.5.1.2 is answered by the caller either way -- the peer may be holding
 * Tone A and something has to reply to it; all this changes is whether the
 * next Phase 2 still offers V.90. */
static void v90_concede_to_v34_locked(const char *why)
{
    if (g_mod != ME_MOD_V90 || g_v92_active)
        return;
    ME_LOG("[ME] V.90: %s; conceding V.90 and continuing as plain V.34 "
           "(this modem's corpus says a call in this state does not recover; "
           "ME_V90_JA_DEADLINE_SEC=0 ME_V90_JA_CONCEDE_ATTEMPTS=0 disable)\n",
           why);
    trace_phase("V90 conceded -> plain V.34: %s", why);
    g_mod = ME_MOD_V34;
}

static v92_p3_rx_t    g_v92_p3_rx;
static bool           g_v92_p3_rx_active = false;
static bool           g_v92_p3_rx_result_applied = false;
static bool           g_v92_p3_rx_failure_logged = false;
static v92_cp_rx_t    g_v92_p3_cpt_rx;
static v92_trn2u_demod_t g_v92_p3_cpt_demod;
static bool           g_v92_p3_cpt_active = false;

/* Strict raw-codeword Su/S-bar-u recogniser (V.92 §8.5.6).  It is armed
 * only after a strict Ja decode; twelve hypotheses cover phase and polarity. */
typedef struct {
    int run[12];
    int last_polarity;
    int transitions;
} v92_su_rx_t;
static v92_su_rx_t    g_v92_su_rx;
static bool           g_v92_su_rx_active = false;
static bool           g_v92_su_final_pending = false;
static bool           g_v92_trn2u_active = false;
static int            g_v92_trn2u_points = 4;
static double         g_v92_trn2u_lu = 8000.0;
static v92_cp_rx_t    g_v92_cp_rx;
static v92_trn2u_demod_t g_v92_trn2u_demod;
static v92_upstream_rx_t g_v92_upstream_rx;
static bool           g_v92_upstream_rx_active = false;
static bool           g_v92_upstream_lock_logged = false;
static uint8_t        g_v90_data_frame[V90_DATA_FRAME_LEN];
static int            g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
static bool           g_v34_fallback_to_v22bis_pending = false;

/* INFO1d Table 17 bit 70.  PCM upstream is V.92's only data-pump gain over
 * V.90, so a zero here makes an analogue peer select V.90 even with both
 * INFO0 capability bits set -- slmodemd reports "V92 capabilities: local=1 ,
 * remote=1 , selected=90" the instant it consumes our INFO1d.  Off by default
 * because the PCM-upstream receiver is still experimental: its B1u and
 * 16-state data path now exist, but foreign-bearer timing/equalizer coverage
 * does not.  Set ME_V92_PCM_UPSTREAM=1 to exercise it explicitly. */
static bool v92_pcm_upstream_advertised(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("ME_V92_PCM_UPSTREAM");
        cached = (v && *v && *v != '0') ? 1 : 0;
    }
    return cached != 0;
}
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
/* V.90 §9.4.1.1 requires CPt reception while Ri is being sent.  The
 * synchronous SpanDSP path is primary; this independent strict fallback
 * starts at 100 ms and retries every 40 ms so a short, single CPt is examined
 * before the analogue modem can abandon Phase 4.  An incomplete snapshot
 * cannot advance state because v90_cp_live_recover() requires a complete
 * Table-14 frame with valid CRC and semantics. */
#define V90_CP_LIVE_FIRST_ATTEMPT_SAMPLES 800
#define V90_CP_LIVE_RETRY_SAMPLES 320
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
static int             g_v90_cp_live_baud_code = 4;
static unsigned        g_v90_cp_live_generation = 0;
typedef struct {
    uint64_t input_bits;
    uint32_t sync_candidates;
    uint32_t valid_frames;
    uint32_t rejected_frames;
    uint32_t crc_rejected_frames;
    uint32_t structure_rejected_frames;
    uint32_t semantic_rejected_frames;
} v90_cp_live_rx_counters_t;
static int             g_v90_cp_live_cpt_accept_sample = -1;
static unsigned        g_v90_cp_live_post_cpt_attempts = 0;
static v90_cp_live_rx_counters_t g_v90_cp_live_rx_baseline;

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

/* A usable DIL descriptor and a descriptor decoded from the live Ja are
 * deliberately separate facts.  ME_V90_DIL_PROFILE may preload a known-good
 * interoperability fallback, making g_v90_pending_dil_valid true before Ja,
 * but that must not suppress the live decoder: V.90 §9.3.1.3 requires the
 * received Ja transition to trigger Sd. */
static bool           g_v90_dil_parse_logged = false;

#define V90_DIL_CAPTURE_MAX_BITS 65536
static uint8_t        g_v90_dil_capture[(V90_DIL_CAPTURE_MAX_BITS + 7) / 8];
static int            g_v90_dil_capture_bits = 0;
static int            g_v90_dil_capture_search = 0;
static int            g_v90_dil_hyp_last_bits = 0;
static bool           g_v90_dil_hyp_dumped = false;
static bool           g_v90_wait_ja_tone_a_logged = false;

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
/* Selected once in me_init() from ME_MODE.  Keeping this at the V.8 offer
 * boundary lets plain V.34 exercise SpanDSP without entering any V.90/V.92
 * branches.  Values: v34, v90 (default), v92. */
/* When the plain-V.34 receiver entered V34_RX_STAGE_PHASE3_WAIT_S, for
 * ME_V34_PHASE3_S_TIMEOUT_MS. */
static uint64_t g_v34_phase3_wait_s_ms;
static bool g_advertise_v90 = true;
/* ME_MODE=v22 offers V.22bis alone in CM/JM.  It exists for a bearer whose far
 * end is a real analogue line rather than a digital G.711 path: V.22bis has
 * its own timing recovery and does not assume the peer's symbol clock is
 * phase-locked to our sample grid, which the V.34 Phase 3 acquisition does. */
static bool g_advertise_v34 = true;
static bool g_enable_v92 = false;
static const char *g_mode_name = "v90";

/*
 * Which side of a V.90 call this endpoint offers to be.
 *
 * V.90 is asymmetric: the digital modem injects PCM codewords downstream and
 * the analogue modem is the one with a D/A -> loop -> A/D hop, so it is the
 * side that measures the line and tells the digital modem what constellation
 * to use (§5.4.3).  This software has always been the digital side, and V.8
 * has always said so.
 *
 * The analogue role is opt-in.  It runs the analogue branches of V.8 and
 * Phases 2-4, then V.34 upstream and PCM downstream data, retrain, rate
 * renegotiation and §9.7 cleardown.  See docs/v90_analogue_role.md.
 */
static bool g_v90_analogue_role = false;

static bool me_v90_analogue_role(void)
{
    return g_v90_analogue_role;
}

/*
 * Analogue-role Phase 3 (§9.3.2).  v90_analogue_phase3.c owns both directions;
 * the engine's job is to start it at the Phase 2/3 seam, hand it the received
 * codewords, and take its samples for the upstream.
 *
 * g_v90 stays NULL for the whole of an analogue-role call.  Every digital
 * Phase 3/4 site in this file is guarded on it, so leaving it unbuilt is what
 * keeps the digital transmitter out of a call that announced itself analogue —
 * rather than a flag each of those sites would have to remember to check.
 */
static v90_analogue_phase3_t *g_v90a = NULL;
static bool     g_v90a_started = false;
/*
 * The slicer that lets the analogue role run over a real analogue line.
 *
 * On a digital bearer the codewords arrive byte-exact and this stays NULL --
 * me_rx_g711() feeds the receiver directly.  On a two-wire line the samples
 * reach me_rx_audio() as levels instead, and this turns them back into §8.4's
 * codeword stream (v90_analogue_linear.h).
 */
static v90a_linear_t *g_v90a_linear = NULL;
/*
 * The T/2 equaliser, when the caller supplies its own 16 kHz stream
 * (v90_analogue_fse.h).  On a two-wire line this is what actually recovers the
 * downstream: the slicer alone cannot, because the sampling phase and the
 * line's intersymbol interference are both outside its reach.  NULL on a
 * digital bearer, where there is nothing to equalise.
 */
static v90a_fse_t *g_v90a_fse = NULL;
/* Set by the first me_rx_v90a_16k() of a call.  The 8 kHz path then stops
 * slicing: the two would otherwise feed the same receiver twice, once well and
 * once badly. */
static bool     g_v90a_16k = false;
/* Whether the G.711 ladder has been calibrated against the TRN1d Ucode the
 * receiver learned; until it has, a level means nothing in absolute terms. */
static bool     g_v90a_ladder_set = false;
/*
 * §8.4.4's Sd acquisition, before anything is calibrated.
 *
 * The blind equaliser cannot be used here: CMA drives every symbol to one
 * modulus and Sd is four slots at W and two at zero, so it erases exactly the
 * structure the hunt looks for (measured: real Sd through a dispersive channel
 * comes out ±1 on all six slots).  So the taps are FITTED to §8.4.4's known
 * sequence instead, over a window buffered here, and the blind loop is not
 * started until §8.4.5's TRN1d, which is constant modulus and is what it was
 * written for.
 */
/*
 * The fit window, and it is bounded by how SHORT Sd is.
 *
 * §8.4.4's Sd is 64 six-symbol repetitions -- 384 DS0 intervals, **48 ms** --
 * and it is followed immediately by S-bar-d and TRN1d.  A window longer than
 * that can never contain only Sd, and the fit is then being asked to map a
 * mixture onto the Sd reference, which it correctly refuses.  Measured live
 * against a digital modem that transmitted Sd, S-bar-d and TRN1d exactly as it
 * should: at 2048 samples (128 ms) the held-out score never rose above 0.014
 * in eight consecutive windows.
 *
 * 512 samples is 256 DS0 intervals, 32 ms, two thirds of Sd -- short enough
 * that a window fits inside it with room to land, long enough that the
 * held-out half is still 20 six-symbol repetitions.
 */
#define V90A_SD_FIT_SAMPLES 512
/*
 * And slide by a quarter of a window rather than a half, so that a window
 * lands wholly inside 48 ms of Sd wherever the hunt happens to start.
 */
#define V90A_SD_FIT_SLIDE   (V90A_SD_FIT_SAMPLES/4)
static int16_t  g_v90a_sd_buf[V90A_SD_FIT_SAMPLES];
static int      g_v90a_sd_fill = 0;
static bool     g_v90a_sd_fitted = false;
static int      g_v90a_sd_score_logged = 0;
static v90a_sd_t *g_v90a_sd = NULL;
static bool     g_v90a_dil_tracking = false;
/*
 * The equaliser's unit modulus, in the slicer's input units.  Arbitrary -- it
 * cancels out of the calibration -- so it is chosen for headroom: §8.4.4's Sd
 * is 6.6 dB above the TRN1d level this is pinned to, and §8.6's constellations
 * reach higher still.
 */
#define V90A_FSE_SCALE  3000.0
/* Set while me_rx_g711() is forwarding its own linear copy to me_rx_audio(),
 * so the codeword path is not run twice on one call's samples. */
static bool     g_rx_from_g711 = false;
static void me_v90_analogue_rx_codewords_locked(const uint8_t *codewords, int count);
static bool     g_v90a_complete_logged = false;
static bool     g_v90a_failed_logged = false;
static bool     g_v90a_retrain_logged = false;
static uint64_t g_v90a_data_start_samples = 0;
static bool     g_v90a_rr_triggered = false;
static bool     g_v90a_rr_deadline_logged = false;
static bool     g_v90a_cleardown = false;
static uint64_t g_v90a_rx_codewords = 0;
static int      g_v90a_data_diag_bits = 0;
static int      g_v90a_data_diag_zeros = 0;
static uint64_t g_v90a_data_bits_seen = 0;
static uint64_t g_v90a_data_adp_window = 0;
static int      g_v90a_data_adp_window_bits = 0;
static int      g_v90a_data_adp_count = 0;
static int      g_v90a_u_info = 78;
static v90_dil_desc_t g_v90a_dil;
static bool     g_v90a_dil_valid = false;

/*
 * U_INFO for our INFO1a (Table 10 bits 25:31): the Ucode the digital modem
 * will train on.  §8.2.3.2 requires it to be greater than 66; Phase 2 clamps
 * it further when INFO0d's maximum transmit power cannot carry the point.
 */
static int me_v90a_u_info(void)
{
    int v = parse_env_int("ME_V90_ANALOGUE_UINFO", 78);

    return (v >= 67  &&  v < 128) ? v : 78;
}

/* The DIL descriptor to request in Ja.  §9.3.2.9 will measure what comes back
 * of exactly this, so the choice decides what can be learned about the line
 * (docs/v90_constellation_selection.md). */
static bool me_v90a_load_dil(v90_dil_desc_t *desc)
{
    const char *name = getenv("ME_V90_ANALOGUE_DIL");
    v90_dil_preset_t preset = V90_DIL_PRESET_MEASUREMENT;
    v90_dil_desc_check_t check;

    if (name  &&  *name) {
        if (strcmp(name, "none") == 0) {
            memset(desc, 0, sizeof(*desc));
            ME_LOG("[ME] V.90 analogue: requesting a zero-length DIL\n");
            return true;
        } else if (strcmp(name, "default-ja") == 0) {
            preset = V90_DIL_PRESET_DEFAULT_JA;
        } else if (strcmp(name, "courier-style") == 0) {
            preset = V90_DIL_PRESET_COURIER_STYLE;
        } else if (strcmp(name, "smartlink-adi") == 0) {
            preset = V90_DIL_PRESET_SMARTLINK_ADI;
        } else if (strcmp(name, "smartlink-adi-qc") == 0) {
            preset = V90_DIL_PRESET_SMARTLINK_ADI_QC;
        } else if (strcmp(name, "measurement") != 0) {
            ME_LOG("[ME] V.90 analogue: unknown ME_V90_ANALOGUE_DIL '%s'; using measurement\n",
                   name);
        }
    }
    if (!v90_dil_preset_load(preset, desc)) {
        ME_LOG("[ME] V.90 analogue: could not load DIL preset %s\n",
               v90_dil_preset_name(preset));
        return false;
    }
    /* A descriptor can satisfy every Table 12 constraint and still leave a
     * data-frame interval unprobed, which is silent at the far end and costs
     * a sixth of the constellation.  Say so now rather than discovering it in
     * the measurement. */
    if (v90_dil_desc_validate(desc, &check)) {
        ME_LOG("[ME] V.90 analogue DIL: %s — %d segments, %d symbols (%.1f ms), "
               "Ucodes %d..%d, intervals probed 0x%02X%s\n",
               v90_dil_preset_name(preset),
               check.segments, check.cycle_symbols, check.cycle_ms,
               check.lowest_ucode, check.highest_ucode,
               check.intervals_probed,
               check.ok ? "" : " — INCOMPLETE, some interval will not be measured");
    }
    return true;
}
static int        g_v34_start_baud = 2400;   /* 3200 has 91 Hz separation (notch unusable); 2400 has 200 Hz */
static int        g_v34_start_bps  = 0;     /* 0 = auto (max for baud rate) */
static int        g_training_tx_samples = 0; /* Sample counter for TX silencing echo test */

/* Handshake timeouts (in milliseconds) */
#define V8_TIMEOUT_MS       15000   /* V.8 negotiation: 15 s (2003-era
                                       SmartLink DSPs need ~6 s just to
                                       validate JM before sending CJ) */
#define TRAINING_TIMEOUT_MS 60000   /* V.90 hardware may spend ~10 seconds
                                       retrying INFO before Phases 3/4.
                                       ME_TRAINING_TIMEOUT_MS overrides for
                                       multi-retrain interop calls, where 60 s
                                       caps Phase 4 attempts at ~3 per call. */

static uint64_t me_training_timeout_ms(void)
{
    static uint64_t cached;

    if (cached == 0) {
        const char *v = getenv("ME_TRAINING_TIMEOUT_MS");

        cached = TRAINING_TIMEOUT_MS;
        if (v && *v) {
            char *end;
            long parsed = strtol(v, &end, 10);

            if (end != v && *end == '\0' && parsed >= 1000)
                cached = (uint64_t)parsed;
        }
    }
    return cached;
}

/* V.34 RX stage tracking — used for notch filter activation and diagnostics */
static int g_last_rx_stage = 0;            /* Last logged RX stage */
static int g_last_tx_stage = 0;            /* Last logged TX stage */
static bool g_v34hdx_fax_control_started = false;
static int g_v34hdx_fax_mode = V34_HALF_DUPLEX_CONTROL_CHANNEL;

/* TX sample ring buffer.  Feeds the optional NLMS canceller and the Phase 3
   far-end-S echo gate (which needs it even when the canceller is off).
   Size must be a power of 2. */
#define TX_BUF_SIZE 4096
#define TX_BUF_MASK (TX_BUF_SIZE - 1)
static int16_t g_tx_buf[TX_BUF_SIZE];
static int     g_tx_buf_wr = 0;  /* write position (updated by me_tx_audio) */
static int     g_tx_buf_rd = 0;  /* read position (updated by me_rx_audio) */

/* Mirror of the RX samples as the V.34/V.90 receiver sees them (post notch /
   post EC), so the echo gate correlates exactly the signal the S detector
   decided on. */
static int16_t  g_rx_ref_buf[TX_BUF_SIZE];
/* Pre-echo-canceller samples for adaptive Phase-3 acquisition. */
static int16_t  g_rx_raw_buf[TX_BUF_SIZE];
static int      g_rx_ref_wr = 0;
static uint64_t g_rx_ref_samples = 0;   /* monotonic; aligns gate logs to the tap */

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
static int  g_notch_tx_baud = -1;
static int  g_notch_tx_high = -1;
static int  g_notch_rx_baud = -1;
static int  g_notch_rx_high = -1;
static bool g_v34_use_echo_can = false;
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
/* Samples actually handed to v34_rx(), against the samples that arrived.
 *
 * These must be equal from the moment the V.34 receiver starts, and if they
 * are not, the receiver's symbol clock runs slow against the wire by exactly
 * the shortfall -- which its timing loop reports as a frequency offset it
 * cannot explain.  That is the open question of 2026-08-24: a live 28800 call
 * walks to -64 ppm and loses the constellation 0.9 s after B1, while both
 * replays of that call's OWN RECORDING hold the eye for 19.7 s at +/-2 ppm.
 * The tap is written before anything else touches the buffer, so a frame the
 * live path fails to feed on is invisible to every recording -- and this
 * counter is the one place the difference can show.  ME_RX_ACCOUNTING=0
 * silences it. */
static uint64_t       g_v34_rx_samples = 0;
/* One-shot: see the arm-phase log in me_rx_audio(). */
static bool           g_v90_t3_arm_logged = false;
/* Samples that reached me_rx_audio(), between the two above: it separates
 * "me_rx_g711() did not pass them on" from "the state machine routed them
 * somewhere other than the V.34 receiver". */
static uint64_t       g_rx_audio_samples = 0;
static bool           g_v90_dil_capture_start_logged = false;
static uint64_t       g_rx_audio_started_at = 0;
static uint64_t       g_v34_rx_started_at = 0;
static bool           g_v34_rx_accounting_logged = false;
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
    if (mkdir(dir, 0755) != 0 && errno != EEXIST) {
        /* Only the leaf is created; a missing parent still fails fopen below. */
        fprintf(stderr, "[ME] VPCM_G711_TAP_DIR mkdir %s: %s\n",
                dir, strerror(errno));
    }
    g_g711_rx_tap = fopen(rx_path, "wb");
    g_g711_tx_tap = fopen(tx_path, "wb");
    if (!g_g711_rx_tap || !g_g711_tx_tap) {
        fprintf(stderr, "[ME] Unable to open live G.711 taps in %s\n", dir);
        g711_taps_close();
        return;
    }
    /* The tap fwrites run on the media clock; with default 4 KB stdio
     * buffers each flush is a disk write() on that path, and an occasional
     * stall past the 20 ms frame deadline slips a frame — fatal for the
     * sample-exact V.90 PCM stages while leaving tones/DPSK fine (observed
     * live 2026-07-23: taps-on 0/9 attempts decode Jd, taps-off 4/4).
     * 64 MB per tap holds >2 h of G.711, so no write() lands mid-call. */
    setvbuf(g_g711_rx_tap, NULL, _IOFBF, 64 * 1024 * 1024);
    setvbuf(g_g711_tx_tap, NULL, _IOFBF, 64 * 1024 * 1024);
    ME_LOG("[ME] Live G.711 taps: RX=%s TX=%s\n", rx_path, tx_path);
}

void me_flush_g711_taps(void)
{
    if (g_g711_rx_tap)
        fflush(g_g711_rx_tap);
    if (g_g711_tx_tap)
        fflush(g_g711_tx_tap);
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
    /* The call function and the modulations the peer offered are the two
       fields that say what the far end actually wants, and neither was
       reported.  A Canon TR7560 calling in to send a V.34 fax was summarised
       here as "protocol=None, PSTN=unknown, PCM=PCM unavailable" -- nothing
       that would tell anyone it had asked for T.30 Tx FAX over V.34
       half-duplex.  peer_modulations, not modulations: the latter is the
       intersection this modem will OFFER, which on a fax call is empty. */
    {
        char mods[256];
        int n = 0;
        int i;

        mods[0] = '\0';
        for (i = 0;  i < 32;  i++)
        {
            if ((result->jm_cm.peer_modulations & (1u << i)))
                n += snprintf(mods + n, (n < (int) sizeof(mods)) ? sizeof(mods) - n : 0,
                              "%s%s", (n > 0) ? "," : "",
                              v8_modulation_to_str(result->jm_cm.peer_modulations & (1u << i)));
            /*endif*/
        }
        /*endfor*/
        fprintf(stderr, "[ME] V.8 peer offer: call function=%s, modulations=%s\n",
                v8_call_function_to_str(result->jm_cm.call_function),
                (mods[0] != '\0') ? mods : "none");
    }
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

/* V.34 fax probe (T.30 Annex F / V.34 clause 12), opt-in.
 *
 * Default OFF, and it must stay that way: this is the shared production call
 * path, and offering V.34 half-duplex changes what every DATA peer sees in
 * our JM.  With it set the answerer adds V8_MOD_V34HDX to its JM offer, so a
 * calling fax's CM -- a Canon TR7560 sends 81 85 d4 90 07, offering exactly
 * that -- intersects with something instead of nothing, and the far end
 * proceeds into V.34 Phase 2 where its INFOh, PPh, ALT and MPh can be
 * recorded.
 *
 * The same opt-in now also keeps a selected fax service class from taking the
 * legacy linear-audio shortcut: after clause 12 start-up, T.30 attaches to
 * the V.34 control-channel bit callbacks instead. */
static bool me_v34_fax_probe(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("ME_V34_FAX_PROBE");
        cached = (v && atoi(v) != 0);
    }
    return cached != 0;
}

static bool me_v8_no_ci(void)
{
    const char *value = getenv("ME_V8_NO_CI");

    return value && atoi(value) != 0;
}

static int me_start_or_restart_v8_locked(int answer_tone)
{
    v8_parms_t v8_parms;
    memset(&v8_parms, 0, sizeof(v8_parms));
    v8_parms.modem_connect_tone = g_calling_party ? MODEM_CONNECT_TONES_NONE
                                                  : answer_tone;
    /* CI announces a data call to the NETWORK, before the answer.  A caller
     * that starts V.8 because it has ALREADY heard ANSam has nothing to
     * announce, and SpanDSP's CI path costs it the 1000 ms V8_WAIT_1S plus
     * however long its own ansam_rx takes to re-detect a tone we detected for
     * it -- measured against the HT802 leg, 3.7 s from starting V.8 to the CM
     * leaving, against an answerer whose CM-wait budget is 200 + 5000 ms.
     * ME_V8_NO_CI=1 skips CI so V8_AWAIT_ANSAM is entered directly. */
    v8_parms.send_ci            = g_calling_party && !me_v8_no_ci();
    /* V.92 Tables 5/14 QC/QCA: this endpoint is always the digital modem.
       The current Jp profile selects the mandatory 4-point TRN2u channel.
       v8.c gates the QCA response on a received QC per V.92 9.2.4.1/.2;
       sending this configured QCA after ordinary CM made the Conexant
       CX93001 wait for QTS/ANSpcm instead of entering full Phase 2.  Even
       when QCA is suppressed, the CM/JM PCM-availability field below still
       lets the full-startup path advertise V.92 in INFO0. */
    /* Keep V.92 opt-in until its start-up path is interoperable end to end.
       Advertising it and later demoting to V.90 leaves some analogue modems
       waiting for QTs in their V.92 Phase 3 state machine. */
    if (g_enable_v92)
        v8_parms.v92            = g_calling_party ? 0x45 : 0x47;
    else
        v8_parms.v92            = -1;
    v8_parms.jm_cm.call_function      = V8_CALL_V_SERIES;
    v8_parms.jm_cm.modulations        = V8_MOD_V22;
    if (g_advertise_v34)
        v8_parms.jm_cm.modulations   |= V8_MOD_V34;
    if (g_advertise_v90)
        v8_parms.jm_cm.modulations   |= V8_MOD_V90;
    if (me_v34_fax_probe() && !g_calling_party) {
        /* Answerer only.  A calling fax is the SOURCE in V.34 12.2.1 ("call
           modem as source modem"), which makes this end the RECIPIENT, and
           that is the one pairing the clause 12 code has ever run. */
        v8_parms.jm_cm.modulations   |= V8_MOD_V34HDX;
        ME_LOG("[ME] V.34 fax probe: offering V.34 half-duplex in JM "
               "(ME_V34_FAX_PROBE)\n");
    }
    v8_parms.jm_cm.protocols          = V8_PROTOCOL_LAPM_V42;
    if (g_advertise_v90 && me_v90_analogue_role()) {
        /*
         * The two fields answer different questions, and only the second one
         * is about the role.
         *
         * pstn_access describes how this DCE reaches the network, and the
         * answer does not change with the role we play: the media here is
         * G.711 over SIP, so V8_PSTN_ACCESS_DCE_ON_DIGITAL stays set because
         * it is true.  pcm_modem_availability is the field that offers a
         * V.90/V.92 role, and that is the one that flips.
         *
         * (A peer could in principle object that a digital access has no
         * analogue loop to learn impairment on.  That is an empirical
         * question, and the lab case this exists for -- dialling the Eicon
         * card, both ends on G.711 -- is exactly where to answer it.)
         */
        v8_parms.jm_cm.pstn_access            = V8_PSTN_ACCESS_DCE_ON_DIGITAL;
        v8_parms.jm_cm.pcm_modem_availability = V8_PSTN_PCM_MODEM_V90_V92_ANALOGUE;
        /* The V.92 QC/QCA octet below encodes the *digital* modem's
         * capabilities; there is no analogue equivalent wired up here, so V.92
         * stays out of an analogue-role offer rather than being guessed. */
        v8_parms.v92 = -1;
    } else if (g_advertise_v90) {
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
                span_log_set_level(log, me_span_flow_level());
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

/* Which echo control plain V.34 uses.  "auto" is the default and is decided
 * by v34_update_echo_policy() below; "none", "notch" and "canceller" force
 * one for an A/B. */
enum { V34_ECHO_AUTO = 0, V34_ECHO_NONE, V34_ECHO_NOTCH, V34_ECHO_CANCELLER };

static int v34_echo_policy_mode(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *e = getenv("ME_V34_ECHO");

        cached = V34_ECHO_AUTO;
        if (e && strcmp(e, "none") == 0)           cached = V34_ECHO_NONE;
        else if (e && strcmp(e, "notch") == 0)     cached = V34_ECHO_NOTCH;
        else if (e && strcmp(e, "canceller") == 0) cached = V34_ECHO_CANCELLER;
    }
    return cached;
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

/* V.90's Phase 2 CC notch has to come out before Phase 3.
 *
 * v8_result_handler() puts a 30 Hz notch on 1200 Hz, our own CC transmit
 * frequency, and against the CC tones that is exactly right: both signals are
 * narrowband and 1200 Hz apart, so the notch removes our echo and costs the
 * 2400 Hz we are listening to nothing.  From Phase 3 on it is the opposite.
 * The upstream is then a wideband V.34 signal -- at 3200 baud on the low
 * carrier it spans about 36 to 3620 Hz -- and 1200 Hz is deep inside it.  A
 * 30 Hz notch is an impulse response of some 266 samples, about 170 symbols at
 * 3200 baud, against the 63 symbols the receiver's equalizer spans, so the
 * receiver cannot undo it.
 *
 * Nothing retired it: start_v34_training() disables the notch at 3200 baud
 * (the 91 Hz carrier separation is too narrow to filter), v8_result_handler()
 * then re-enables it at 1200 Hz for Phase 2, and the only later clear is on
 * the fallback to plain V.34.  Every V.90 soak log shows the pair
 * "Notch filter DISABLED ... 91.4 Hz too narrow" then "V.90 notch filter at
 * 1200 Hz" and no third line.
 *
 * The same mistake in the plain-V.34 path cost 17 dB of receive SNR
 * (docs/v34_data_mode_rates.md).  ME_V34_ECHO=notch keeps the old behaviour
 * for an A/B.  The wideband NLMS canceller is untouched: it is gated on
 * g_mod == ME_MOD_V90 and is the right tool against a broadband PCM echo.
 */
static void v90_retire_phase2_cc_notch(void)
{
    /* g_notch.active is the once-per-call guard: this clears it, and
       v8_result_handler() re-arms it for the next call.  A static "already
       retired" latch was tried here and is wrong -- the server runs many calls
       in one process, so the first call retired the notch and every later call
       re-armed it at V.8 and could never retire it again.  The call that
       reaches data mode is rarely the first, so the notch was live in the
       receive path for exactly the calls that mattered: measured on
       artifacts/v90-cap-20260822T130820Z-9600, the one call that reached data
       mode never logged a retirement and ran its whole upstream at 0.139 from
       the lattice -- the same signature the plain-V.34 notch left. */
    if (!g_notch.active)
        return;
    if (v34_echo_policy_mode() == V34_ECHO_NOTCH)
        return;
    g_notch.active = false;
    ME_LOG("[ME] V.90 Phase 2 CC notch retired at Phase 3: 1200 Hz is inside "
           "the wideband upstream this receiver is now listening to\n");
}
/*- End of function --------------------------------------------------------*/

static void v34_update_echo_policy(void)
{
    static const int baud_by_code[6] = {2400, 2743, 2800, 3000, 3200, 3429};
    int tx_code;
    int rx_code;
    int tx_high;
    int rx_high;
    int tx_baud;
    int rx_baud;
    float tx_carrier;
    float rx_carrier;
    float separation;

    if (!g_v34 || g_mod != ME_MOD_V34)
        return;
    tx_code = v34_get_tx_baud_rate(g_v34);
    rx_code = v34_get_rx_baud_rate(g_v34);
    tx_high = v34_get_tx_high_carrier(g_v34);
    rx_high = v34_get_rx_high_carrier(g_v34);
    if (tx_code < 0 || tx_code >= 6 || rx_code < 0 || rx_code >= 6)
        return;
    if (tx_code == g_notch_tx_baud && tx_high == g_notch_tx_high
        && rx_code == g_notch_rx_baud && rx_high == g_notch_rx_high)
        return;

    tx_baud = baud_by_code[tx_code];
    rx_baud = baud_by_code[rx_code];
    tx_carrier = v34_primary_carrier_hz(tx_baud, tx_high != 0);
    rx_carrier = v34_primary_carrier_hz(rx_baud, rx_high != 0);
    separation = fabsf(tx_carrier - rx_carrier);
    g_notch_tx_baud = tx_code;
    g_notch_tx_high = tx_high;
    g_notch_rx_baud = rx_code;
    g_notch_rx_high = rx_high;

    /* V.34 §10.1.2.3.5 selects each direction in INFO1a.  A startup
       profile is therefore not authoritative here.  A notch is safe only
       when the selected carriers are separated; 3429 has coincident
       carriers and 3200 is normally too close, so use the adaptive echo
       canceller instead of declaring either negotiated profile unusable. */
    /* ME_V34_ECHO=none disables both.  Neither is free: the notch is 30 Hz
       wide, which is an impulse response of about 266 samples -- 114 symbols
       at 3000 baud, against the 63 symbols the receiver's equalizer spans --
       so where the notch frequency lands inside the received signal's band
       the receiver cannot undo it, and the damage is invisible to the RX
       G.711 tap because the tap is upstream of this filter. */
    /* Is our transmit carrier inside the band the receiver is listening to?
     * V.34's shaping is 12% excess bandwidth, so the received signal occupies
     * the RX carrier +/- 0.56 of the RX symbol rate.  At every symbol rate
     * this pairing reaches, that band is most of the 300-3400 channel and the
     * TX carrier lands inside it.  Carrier *separation* is the wrong test: at
     * 3000 baud the two carriers are 200 Hz apart, which passed the old
     * >= 150 Hz test, and the notch then went in at 2000 Hz -- 200 Hz from
     * the middle of a signal spanning 120 to 3480. */
    float rx_lo = rx_carrier - 0.56f*rx_baud;
    float rx_hi = rx_carrier + 0.56f*rx_baud;
    bool tx_in_rx_band = (tx_carrier > rx_lo  &&  tx_carrier < rx_hi);
    int mode = v34_echo_policy_mode();

    if (mode == V34_ECHO_AUTO)
        mode = tx_in_rx_band ? V34_ECHO_NONE : V34_ECHO_NOTCH;

    if (mode != V34_ECHO_NOTCH) {
        /* A 30 Hz notch is an impulse response of about 266 samples, 114
         * symbols at 3000 baud, against the 63 symbols this receiver's
         * equalizer spans -- so an in-band notch cannot be undone, and the
         * damage is invisible to the RX G.711 tap because the tap is upstream
         * of the filter.  Measured live at 3000 baud/9600 against the
         * SmartLink rig: with the notch the receiver reads 17.4 dB, without
         * it 34.7 dB, on a wire whose own least-squares bound is 38 dB. */
        g_notch.active = false;
        g_v34_use_echo_can = (mode == V34_ECHO_CANCELLER);
        ME_LOG("[ME] V.34 negotiated echo policy: TX=%d/%s %.1f Hz, "
               "RX=%d/%s %.1f Hz -> %s (TX carrier %s the %.0f-%.0f Hz "
               "receive band)\n",
               tx_baud, tx_high ? "high" : "low", tx_carrier,
               rx_baud, rx_high ? "high" : "low", rx_carrier,
               g_v34_use_echo_can ? "adaptive canceller" : "no echo filter",
               tx_in_rx_band ? "is inside" : "is outside", rx_lo, rx_hi);
    } else if (separation < 150.0f  &&  v34_echo_policy_mode() == V34_ECHO_AUTO) {
        g_notch.active = false;
        g_v34_use_echo_can = true;
        ME_LOG("[ME] V.34 negotiated echo policy: TX=%d/%s %.1f Hz, "
               "RX=%d/%s %.1f Hz, separation %.1f Hz -> adaptive canceller\n",
               tx_baud, tx_high ? "high" : "low", tx_carrier,
               rx_baud, rx_high ? "high" : "low", rx_carrier, separation);
    } else {
        g_v34_use_echo_can = false;
        notch_filter_init(&g_notch, tx_carrier, 30.0f, 8000.0f);
        ME_LOG("[ME] V.34 negotiated echo policy: TX=%d/%s %.1f Hz, "
               "RX=%d/%s %.1f Hz, separation %.1f Hz -> notch\n",
               tx_baud, tx_high ? "high" : "low", tx_carrier,
               rx_baud, rx_high ? "high" : "low", rx_carrier, separation);
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
static int v90_selected_upstream_baud_locked(void);
static void v92_su_rx_reset_locked(void);
void me_hangup(void);

/* Format constellation `c`'s transmitter Ucodes (descending, §5.4.4) into buf,
 * for interop diagnostics -- e.g. to check a recovered CPt against the
 * constellation the peer's V90TRN2Designer reported it designed. */
static void v90_cp_format_constellation(const vpcm_cp_frame_t *cp,
                                        int c,
                                        char *buf,
                                        size_t buf_len)
{
    size_t pos = 0;

    if (buf_len == 0)
        return;
    buf[0] = '\0';
    if (!cp || c < 0 || c >= cp->constellation_count)
        return;
    for (int ucode = VPCM_CP_MASK_BITS - 1; ucode >= 0; ucode--) {
        if (!vpcm_cp_mask_get(cp->masks[c], ucode))
            continue;
        pos += (size_t)snprintf(buf + pos, buf_len - pos,
                                "%s%d", pos ? "," : "", ucode);
        if (pos >= buf_len - 1)
            break;
    }
}

/* Caller holds g_state_mtx. */
static void v90_cp_live_read_rx_counters_locked(
    v90_cp_live_rx_counters_t *counters)
{
    if (!counters)
        return;
    counters->input_bits = g_v90_cp_rx.input_bits;
    counters->sync_candidates = g_v90_cp_rx.sync_candidates;
    counters->valid_frames = g_v90_cp_rx.valid_frames;
    counters->rejected_frames = g_v90_cp_rx.rejected_frames;
    counters->crc_rejected_frames = g_v90_cp_rx.crc_rejected_frames;
    counters->structure_rejected_frames =
        g_v90_cp_rx.structure_rejected_frames;
    counters->semantic_rejected_frames =
        g_v90_cp_rx.semantic_rejected_frames;
}

static uint32_t v90_cp_live_counter_delta(uint32_t current,
                                          uint32_t baseline)
{
    return current >= baseline ? current - baseline : current;
}

static uint64_t v90_cp_live_counter_delta64(uint64_t current,
                                            uint64_t baseline)
{
    return current >= baseline ? current - baseline : current;
}

static void v90_cp_live_capture_reset_locked(void)
{
    pthread_mutex_lock(&g_v90_cp_live_mtx);
    g_v90_cp_live_generation++;
    g_v90_cp_live_sample_count = 0;
    g_v90_cp_live_phase4_hint = -1;
    g_v90_cp_live_next_request = -1;
    g_v90_cp_live_expected_compatibility = 0;
    g_v90_cp_live_baud_code = 4;
    g_v90_cp_live_cpt_accept_sample = -1;
    g_v90_cp_live_post_cpt_attempts = 0;
    memset(&g_v90_cp_live_rx_baseline,
           0,
           sizeof(g_v90_cp_live_rx_baseline));
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
    g_v90_cp_live_baud_code = v90_selected_upstream_baud_locked();
    /* Attempt as soon as one observed 428-bit CPt could be complete.  If the
     * peer starts later, short strict retries below catch its first frame. */
    g_v90_cp_live_next_request =
        g_v90_cp_live_sample_count + V90_CP_LIVE_FIRST_ATTEMPT_SAMPLES;
    pthread_mutex_unlock(&g_v90_cp_live_mtx);
    ME_LOG("[ME] V.90 strict batch CP receiver armed at upstream sample %d (%d baud)\n",
           g_v90_cp_live_phase4_hint,
           g_v90_cp_live_baud_code == 3 ? 3000 : 3200);
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
        if (!diag->frame.v90_compatibility) {
            /* This is the waveform boundary at which the digital modem
             * accepts CPt and starts Ri/barred-Ri/TRN2d.  Retain both it and
             * the synchronous receiver's counters so later failed CP hunts
             * can describe what the analogue modem actually did next. */
            g_v90_cp_live_cpt_accept_sample =
                g_v90_cp_live_sample_count;
            g_v90_cp_live_post_cpt_attempts = 0;
            v90_cp_live_read_rx_counters_locked(
                &g_v90_cp_live_rx_baseline);
        }
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
    g_v90_wait_ja_tone_a_logged = false;
    g_v90_phase3_s_events = 0;
    g_last_v90_bridge_rx_stage = -1;
    g_last_v90_bridge_tx_stage = -1;
    g_last_v90_bridge_rx_event = -1;
    memset(&g_v90_pending_dil, 0, sizeof(g_v90_pending_dil));
}

/* Upstream (analogue -> digital) V.34 data-path arming.  V.90's upstream
 * Phase 4 is the CP dance, not a V.34 MP exchange, so SpanDSP's own
 * mp_seen-gated E detector can never fire for the answerer -- the RX would
 * sit in its CP receive stage forever and no data bit would reach put_bit (the
 * 2026-07-23 soak measured exactly 0 upstream bytes at the PTY).  Instead:
 * when the peer's acknowledged CP (CP') is accepted we prepare the RX data
 * parameters (v34_v90_prepare_upstream_data -- deliberately does NOT touch
 * the hypothesis lock that keeps the phase-4 bit tap alive), then watch the
 * same tap for the peer's E (20 consecutive ones, 9.4.2: CP' CP' E B1 ->
 * data) and flip the RX into DATA mode at that boundary. */
static bool g_v34_upstream_data_armed = false;
static bool g_v34_upstream_data_started = false;
static int g_v90_upstream_e_run = 0;

/* How many §9.6 rate renegotiations one call may run.  Each costs the best
 * part of a second of downstream (384T of Rd, 24T of R̄d, TRN2d, the MP
 * exchange and 48 data frames of B1d), so a link that needs one every few
 * seconds is not being helped by them and should be left alone rather than
 * spending the rest of the call renegotiating.  ME_V90_MAX_RENEG overrides. */
#define ME_V90_MAX_RENEGOTIATIONS_DEFAULT 8

static int me_v90_max_renegotiations(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *e = getenv("ME_V90_MAX_RENEG");

        cached = (e && atoi(e) >= 0) ? atoi(e)
                                     : ME_V90_MAX_RENEGOTIATIONS_DEFAULT;
    }
    return cached;
}
#define ME_V90_MAX_RENEGOTIATIONS (me_v90_max_renegotiations())

/* Off by default -- but NOT for the reason this comment used to give, which
 * was wrong, and wrong in the direction that blamed the peer.
 *
 * It used to read: "this rig's analogue modem does not answer a §9.6 rate
 * renegotiation.  Two live calls sent Rd for 384T on a data frame boundary
 * and received no CP at all."  We never sent Rd for 384T.  §9.6.1.1.1 makes
 * Rd exactly 384T terminated by 24T of R̄d, and the only code that advanced
 * that state lived in the CP receive path, gated on a far-end CPt -- which is
 * §9.4.1.2's STARTUP rule, where the barred Ri acknowledges the peer's CPt.
 * In a renegotiation the peer cannot send CP until it has seen Rd, R̄d and
 * MP, so each side waited for the other.  Demodulated out of the transmit
 * taps of the very two calls the default was set from
 * (artifacts/goal-v90-reneg-112546Z and -b-113038Z, via
 * tools/v90_rd_verify.py): 89120T and 24160T of unterminated Rd -- 11.1 s and
 * 3.0 s -- and not one barred symbol.  The peer was never given the signal
 * §9.6.1.2 requires it to detect, so those calls say nothing about whether it
 * implements the clause.  Fixed in v90.c; the same recording now produces
 * 384T + 24T exactly.
 *
 * The repaired exchange is now verified against the SmartLink peer: all
 * healthy probe-triggered attempts completed.  This broad knob remains off
 * because chronic carrier-loss triggers mostly started too late and spent
 * downstream without restoring the upstream.  Abrupt discontinuities have
 * their own standards-required early path below and do not depend on this
 * broad loss policy.  ME_V90_RENEG_AFTER_MS remains the healthy-call probe. */
static int me_v90_reneg_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *e = getenv("ME_V90_RENEG");

        cached = (e && atoi(e) != 0) ? 1 : 0;
    }
    return cached;
}

/* Retrain-on-loss bounds, shared by V.90 §9.5.1.1 and plain V.34 §11.5.
 *
 * A retrain costs the whole Phase 2/3/4 startup -- seconds of downstream --
 * so it is worth taking only where the alternative is a dead link, and only a
 * few times.  Two bounds: a per-call cap, and a dwell measured from the last
 * one, so a link that collapses again immediately is left alone rather than
 * spending the call retraining.  The dwell also covers the case where the
 * retrain itself fails to reach data mode: the counter advances whether or
 * not the attempt succeeds.
 *
 * THE TWO CAPS ARE DIFFERENT, and deliberately.
 *
 * Plain V.34 is symmetric: one modulation, one receiver each way, so a
 * receiver that has stopped decoding means half the link is dead and there is
 * nothing to protect by waiting.  That cap stays at 4.
 *
 * V.90 is NOT symmetric.  The downstream is PCM at 52000 and the upstream is
 * V.34 at 31200, they fail independently, and on this rig it is always the
 * upstream that fails -- the frame-phase/eye collapse in
 * docs/v90_upstream_data_path.md, which a fresh handshake does not fix.  The
 * comment that used to sit at the call site argued that the alternative to a
 * retrain was "to keep transmitting downstream into a receiver whose eye is
 * shut for the rest of the call".  Measured over 600 s calls, that is exactly
 * backwards, because the receiver whose eye is shut is at OUR end and the
 * downstream is fine:
 *
 *   artifacts/lossretrain-ab-*, arms alternated, three 600 s calls each,
 *   ME_V90_RETRAIN_ON_LOSS the only variable --
 *     cap 4:  1/0/4 retrains, carried 373/629/629 s,
 *             downstream 490775 lines with 491 missing, upstream 185229
 *     cap 0:  0/0/0 retrains, carried 629/629/629 s,
 *             downstream 667084 lines with 107 missing, upstream 156321
 *
 * So holding the link delivers 36% more downstream lines with a fifth of the
 * losses -- all three calls at the ceiling of what the schedule can send --
 * against 16% fewer upstream lines, on a direction that is more than 45%
 * incomplete in BOTH arms.  The retrain was buying nothing and costing the
 * clean, faster direction.  §9.5.1.1 says the digital modem MAY retrain at
 * any time, so both policies conform.
 *
 * There is also a responder for the case this gives up: §9.5.2.1 has the
 * ANALOGUE modem retrain if ITS receiver fails, and we follow that (the
 * peer-retrain path).  So a genuinely two-directional failure is still
 * recovered; what is no longer done is tearing down a working downstream on
 * our own receiver's account.
 *
 * ME_V90_RETRAIN_ON_LOSS=4 restores the old V.90 behaviour. */
#define ME_V90_MAX_LOSS_RETRAINS_DEFAULT 0
#define ME_V34_MAX_LOSS_RETRAINS_DEFAULT 4
#define ME_V90_LOSS_RETRAIN_DWELL_MS_DEFAULT 20000

static unsigned g_loss_retrains = 0;
static int64_t  g_last_loss_retrain_ms = 0;

static int me_v90_max_loss_retrains(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V90_RETRAIN_ON_LOSS",
                               ME_V90_MAX_LOSS_RETRAINS_DEFAULT);
    return cached;
}

static int me_v34_max_loss_retrains(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V34_RETRAIN_ON_LOSS",
                               ME_V34_MAX_LOSS_RETRAINS_DEFAULT);
    return cached;
}

static int me_loss_retrain_dwell_ms(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V90_RETRAIN_ON_LOSS_DWELL_MS",
                               ME_V90_LOSS_RETRAIN_DWELL_MS_DEFAULT);
    return cached;
}

/* True when a retrain for a receiver that has stopped decoding is both
 * allowed and due -- V.90 §9.5.1.1 or plain V.34 §11.5.1.1/§11.5.2.1.
 * The caller passes its own cap because the two are different; see above. */
static bool retrain_on_loss_due(int cap)
{
    int64_t now;

    if (cap <= 0)
        return false;
    if ((int) g_loss_retrains >= cap)
        return false;
    now = trace_now_ms();
    if (g_last_loss_retrain_ms != 0
        && now - g_last_loss_retrain_ms < me_loss_retrain_dwell_ms())
        return false;
    return true;
}

/* V.34 §11.6 rate renegotiation, engine side.
 *
 * §11.6 is the cheap resynchronisation -- it keeps the call in data mode and
 * costs the S/S-bar/TRN/MP exchange rather than a whole startup -- but it
 * needs a modem at the other end that implements §11.6.1.2.  A retrain needs
 * only the peer's tone detector, which every V.34 modem has.
 *
 * INITIATING one is ON by default, and the reason is measured live rather
 * than inherited.  The V.90 §9.6 note -- "this rig's analogue modem answers
 * Rd with nothing" -- does NOT transfer: that is a different procedure with a
 * different signal, and asked the §11.6 way this same peer answers.  Two live
 * calls, 2026-08-26 (artifacts/reneg-live-r1, -r2): it responded to our S in
 * both.  In r1 the exchange completed, the rate moved 9600 -> 12000, B1
 * re-acquired at correlation 1.000, and 1203 numbered lines reached its DTE
 * with ZERO gaps across the renegotiation.  In r2 its MP never CRC-validated
 * and §11.6.2.1's timeout fell back to a §11.5 retrain.
 *
 * One of two is thin, but the trade is not: the failure path lands exactly
 * where the code would have gone without §11.6, and retrain_on_loss_due()
 * already bounds the whole thing to four attempts a call.  So the worst case
 * is a few seconds of renegotiation before the same retrain, and the best
 * case keeps the link and the error-control layer up.  ME_V34_RENEG=0
 * disables it.
 *
 * RESPONDING to one is not gated here -- §11.6.1.2 is a "shall", and the
 * detector behind the event is itself off unless ME_V90_RENEG_RESPOND=1.
 *
 * §11.6.2.1: "If after transmitting the S-to-S-bar transition, the modem has
 * not received sequence E for the following timeout period, it shall initiate
 * the retrain procedure.  If bit 24 in INFO0 is set to 1 (the CME bit) the
 * timeout period shall be 30 seconds.  If bit 24 in INFO0 is set to 0, the
 * timeout period shall be 2500 ms plus two round trip delays."  CME is not
 * advertised here, so the short one applies; the round trip on a SIP bearer
 * is well under the margin this leaves. */
#define ME_V34_RENEG_E_TIMEOUT_MS_DEFAULT 4000

static int64_t g_v34_reneg_start_ms = 0;

static int me_v34_reneg_enabled(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V34_RENEG", 1) ? 1 : 0;
    return cached;
}

static int me_v34_reneg_timeout_ms(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V34_RENEG_TIMEOUT_MS",
                               ME_V34_RENEG_E_TIMEOUT_MS_DEFAULT);
    return cached;
}

static void v34_reneg_begin_locked(void)
{
    g_v34_reneg_start_ms = trace_now_ms();
}

/* ME_V34_RENEG_AFTER_MS=<n> opens a §11.6 rate renegotiation n ms after the
 * call reaches data mode, once.  A TEST HOOK, and it exists because the only
 * unanswered question about §11.6 is whether a real peer implements
 * §11.6.1.2 -- and the engine's own trigger is a receiver that has stopped
 * decoding, which a healthy call never produces.  The analogue role has had
 * the same knob (ME_V90_ANALOGUE_RATE_RENEGOTIATE_MS) for the same reason.
 * Zero, the default, means never. */
static int64_t g_v34_data_entry_ms = 0;
/* g_rx_audio_samples at the same instant.  The *_AFTER_MS test hooks below
 * are the only way to open a renegotiation or a retrain on a healthy call,
 * and every one of them was measured off trace_now_ms() -- the host's wall
 * clock.  That makes them unusable in the one harness this work depends on:
 * v90_engine_replay --fast consumes the recording faster than real time, so
 * a 20000 ms probe simply never fires, and without --fast a 600 s call costs
 * 600 s of desk time per experiment.  The received-audio counter is the
 * media clock -- 8 kHz on this bearer live and in replay alike -- so these
 * hooks now measure the elapsed audio, which is what "n ms into data mode"
 * was always meant to name.  Live behaviour is unchanged: the two clocks
 * agree on a call that is running in real time. */
static uint64_t g_v34_data_entry_samples = 0;
static bool    g_v34_reneg_probe_done = false;

/* Milliseconds of received audio since this call entered data mode, or 0 if
 * it has not.  Declared here because the probe hooks below all need it;
 * g_rx_audio_samples is defined above. */
static int64_t data_mode_elapsed_ms(void)
{
    if (g_v34_data_entry_ms == 0)
        return 0;
    if (g_rx_audio_samples <= g_v34_data_entry_samples)
        return 0;
    return (int64_t)((g_rx_audio_samples - g_v34_data_entry_samples) / 8u);
}

static int me_v34_reneg_after_ms(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V34_RENEG_AFTER_MS", 0);
    return cached;
}

static bool v34_reneg_probe_due_locked(void)
{
    if (me_v34_reneg_after_ms() <= 0 || g_v34_reneg_probe_done)
        return false;
    if (g_v34_data_entry_ms == 0)
        return false;
    return data_mode_elapsed_ms() >= me_v34_reneg_after_ms();
}

/* ME_V34_RETRAIN_AFTER_MS=<n> initiates a §11.5.1.1/§11.5.2.1 retrain n ms
 * after data mode, once.  The sibling of the knob above and a TEST HOOK for
 * the same reason: the engine's own trigger is a receiver that has stopped
 * decoding, and a healthy call never produces one. */
static bool g_v34_retrain_probe_done = false;

static int me_v34_retrain_after_ms(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V34_RETRAIN_AFTER_MS", 0);
    return cached;
}

/* ME_V90_RENEG_AFTER_MS=<n> opens a §9.6 rate renegotiation n ms after the
 * call reaches V.90 data mode, once.  A TEST HOOK, exactly like plain V.34's
 * ME_V34_RENEG_AFTER_MS above and for the same reason: the engine's own
 * trigger is an upstream receiver that has stopped decoding, so a healthy
 * call never produces one, and the open question is what a real peer does
 * with Rd.
 *
 * The claim this exists to test is that this rig's analogue modem "answers
 * 384T of Rd with nothing at all".  That was inferred from two calls in which
 * the peer retrained and its log said SILENCERETRAIN -- which is the name of
 * the state where the peer TRANSMITS silence before its own Tone A, not a
 * report that it heard silence from us.  The same peer does implement §11.6
 * (verified live: it answered our S, changed the rate and kept LAPM up), so
 * the inference is worth re-testing against our own transmitted Rd.
 *
 * Uses the shared DATA-entry epoch, which the native V.90 handover sets.
 * Zero, the default, means never. */
static bool g_v90_reneg_probe_done = false;
/* §9.6.1.1.1: set while the receiver is watching for the peer's S/S-bar
 * answer to Rd, cleared when it hands over to the CP search. */
static bool g_v90_reneg_await_s = false;

/* §9.6's CP window, measured rather than inferred.  "The peer does not send
 * CP" and "we do not decode it" look identical from the engine's own logs,
 * which say nothing at all while the renegotiation runs.  Snapshot the
 * Table-14 framer's counters when the CP search is armed and report the
 * delta when the renegotiation ends, so the window is classified by what the
 * framer actually saw: no sync candidate at all (nothing CP-shaped on the
 * wire), sync but structural rejects (frame boundary wrong), CRC rejects
 * (boundary right, symbols wrong), or semantic rejects (frame intact, a
 * field this endpoint cannot use). */
static v90_cp_rx_t g_v90_reneg_cp_rx_mark;
static bool g_v90_reneg_cp_rx_marked = false;
/* Whether the CP' that ENDS §9.6.1.2.3 was decoded in the current window. */
static bool g_v90_reneg_cp_ack_seen = false;
/* Blocks of digital silence handed to the receiver while the CP window was
 * armed.  A lost packet is concealed as silence, and 192 symbols of it takes
 * the constellation and does not give it back -- measured on
 * artifacts/reneg-ab-225015Z/reneg-r1, whose RTP trace carries exactly one
 * loss in 32868 packets, three packets, on the frame where §9.6.1.2.3's CP'
 * is due.  A recording cannot show this on its own: the tap is written before
 * the feed, so the count has to be taken here. */
static unsigned g_v90_reneg_dead_blocks = 0;
static unsigned g_v90_reneg_dead_samples = 0;


static void v90_reneg_cp_mark_locked(void)
{
    g_v90_reneg_cp_rx_mark = g_v90_cp_rx;
    g_v90_reneg_cp_rx_marked = true;
    g_v90_reneg_cp_ack_seen = false;
    g_v90_reneg_dead_blocks = 0;
    g_v90_reneg_dead_samples = 0;
}


static void v90_reneg_cp_report_locked(const char *why)
{
    const v90_cp_rx_t *a = &g_v90_reneg_cp_rx_mark;
    const v90_cp_rx_t *b = &g_v90_cp_rx;

    if (!g_v90_reneg_cp_rx_marked)
        return;
    g_v90_reneg_cp_rx_marked = false;
    ME_LOG("[ME] V.90 §9.6 CP window (%s): bits=%llu sync=%u valid=%u "
           "cp_ack=%d rejected=%u (crc=%u structure=%u semantic=%u) voted=%u "
           "dead_blocks=%u dead_samples=%u\n",
           why ? why : "end",
           (unsigned long long)(b->input_bits - a->input_bits),
           (unsigned)(b->sync_candidates - a->sync_candidates),
           (unsigned)(b->valid_frames - a->valid_frames),
           g_v90_reneg_cp_ack_seen ? 1 : 0,
           (unsigned)(b->rejected_frames - a->rejected_frames),
           (unsigned)(b->crc_rejected_frames - a->crc_rejected_frames),
           (unsigned)(b->structure_rejected_frames - a->structure_rejected_frames),
           (unsigned)(b->semantic_rejected_frames - a->semantic_rejected_frames),
           (unsigned)(b->voted_frames_accepted - a->voted_frames_accepted),
           g_v90_reneg_dead_blocks,
           g_v90_reneg_dead_samples);
}

static int me_v90_reneg_after_ms(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_V90_RENEG_AFTER_MS", 0);
    return cached;
}

static bool v90_reneg_probe_due_locked(void)
{
    if (me_v90_reneg_after_ms() <= 0 || g_v90_reneg_probe_done)
        return false;
    if (g_v34_data_entry_ms == 0)
        return false;
    return data_mode_elapsed_ms() >= me_v90_reneg_after_ms();
}

static bool v34_retrain_probe_due_locked(void)
{
    if (me_v34_retrain_after_ms() <= 0 || g_v34_retrain_probe_done)
        return false;
    if (g_v34_data_entry_ms == 0)
        return false;
    return data_mode_elapsed_ms() >= me_v34_retrain_after_ms();
}

/* ME_TX_DISRUPT_AFTER_MS / ME_TX_DISRUPT_MS transmit silence for a window,
 * n ms after data mode.  A TEST HOOK, and the only way to exercise the half
 * of §9.5/§11.5 that matters most: a peer whose receiver has failed holding
 * its retrain tone at us while we are in data mode.  Nothing else provokes
 * it -- the rig will not fail on demand -- and that is the path where three
 * defects were stacked, so it should not stay untested against a real modem.
 * Silence rather than noise: it is what a real carrier loss looks like, and
 * it cannot be mistaken for a signal we meant to send. */
static int64_t g_tx_disrupt_logged = 0;

static int me_tx_disrupt_after_ms(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_TX_DISRUPT_AFTER_MS", 0);
    return cached;
}

static int me_tx_disrupt_ms(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = parse_env_int("ME_TX_DISRUPT_MS", 1500);
    return cached;
}

static bool tx_disrupt_active(void)
{
    int64_t since;

    /* The test hook exists to remove the DATA carrier until the peer asks for
       a retrain.  Once that request moves the engine to TRAINING it must get
       out of the way: otherwise it also erases the §9.5/§11.5 response that
       the test is meant to put on the wire.  The first live peer-response run
       exposed exactly that -- internal logs showed Tone A while the G.711 tap
       still carried the hook's silence, leaving SmartLink in RX_PHASE1_CALL. */
    if (g_state != ME_DATA
        || g_tx_disrupt_logged < 0
        || me_tx_disrupt_after_ms() <= 0
        || g_v34_data_entry_ms == 0)
        return false;
    since = data_mode_elapsed_ms();
    if (since < me_tx_disrupt_after_ms())
        return false;
    return since < (int64_t) me_tx_disrupt_after_ms() + me_tx_disrupt_ms();
}

static void v34_reneg_clear_locked(void)
{
    g_v34_reneg_start_ms = 0;
}

static bool v34_reneg_timed_out_locked(void)
{
    if (g_v34_reneg_start_ms == 0)
        return false;
    return (trace_now_ms() - g_v34_reneg_start_ms)
               >= me_v34_reneg_timeout_ms();
}

static void v90_reset_upstream_data_arming(void)
{
    g_v34_upstream_data_armed = false;
    g_v34_upstream_data_started = false;
    g_v90_upstream_e_run = 0;
}

/* V.90 §6.2 requires the digital modem to support both 3000 and 3200 baud;
 * Table 10 carries the analogue modem's selection.  Keep that protocol value
 * authoritative after the CP receiver temporarily switches to 2400 baud. */
static int v90_selected_upstream_baud_locked(void)
{
    v34_v90_info1a_t info1a;

    if (g_v34 && v34_get_v90_received_info1a(g_v34, &info1a)
        && (info1a.upstream_symbol_rate_code == 3
            || info1a.upstream_symbol_rate_code == 4))
        return info1a.upstream_symbol_rate_code;
    return 4;  /* Defensive fallback to mandatory analogue-modem support. */
}

static int v90_upstream_baud_max_bps(int baud_code)
{
    return baud_code == 3 ? 28800 : 31200;
}

/* Runs with g_state_mtx held, either synchronously inside v34_rx() or from
 * the independent strict batch receiver after its worker reacquires state. */
static bool v90_accept_cp_diag_locked(const vpcm_cp_diag_t *diag,
                                      const char *source)
{
    vpcm_cp_frame_t repaired_frame;
    const vpcm_cp_frame_t *frame;
    bool accepted;

    if (!diag || !g_v90 || g_mod != ME_MOD_V90)
        return false;
    frame = &diag->frame;
    if (parse_env_int("ME_V90_SMARTLINK_DUMMY_CPT", 0) != 0) {
        repaired_frame = diag->frame;
        if (v90_repair_smartlink_dummy_cpt(&repaired_frame)) {
            frame = &repaired_frame;
            ME_LOG("[ME] V.90 SmartLink dummy CPt fingerprint matched; "
                   "removed leaked Ucodes and restored the intended "
                   "8-point 71..78 constellation\n");
        }
    }
    if (frame->codec_alaw != (g_law == ME_LAW_ALAW)) {
        ME_LOG("[ME] V.90 %s codec-law interworking: codec-output=%s SIP=%s; transmitting primary-mask codes and using corresponding output mask for shaping\n",
               frame->v90_compatibility ? "CP" : "CPt",
               frame->codec_alaw ? "A-law" : "mu-law",
               g_law == ME_LAW_ALAW ? "A-law" : "mu-law");
    }
    accepted = v90_set_phase4_cp(g_v90, frame)
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
    if (accepted) {
        v90_cp_live_mark_accepted_locked(diag);
        /* §9.6.1.2.3's CP' is what releases Ed, and whether it was decoded
         * is what separates an attempt that completes from one that takes
         * §9.6.1's timeout.  Reported per window, and deliberately NOT acted
         * on: gating further attempts on it was tried and refuted live --
         * artifacts/renegviab-ab-010014Z/ungated-r3 took the timeout with no
         * CP', and its NEXT attempt completed. */
        if (g_v90_reneg_cp_rx_marked && frame->acknowledge)
            g_v90_reneg_cp_ack_seen = true;
    }
    /* The peer's data-mode CP carries everything the upstream receiver needs
     * (baud, rate, constellation); its acknowledge bit is about the handshake,
     * not about the parameters.  Waiting for an acknowledged CP' meant that
     * against slmodemd -- whose CP' we do not always decode -- the upstream
     * receiver was never prepared, so B1 was hunted with default parameters,
     * correlated at 5%, and no upstream byte ever reached the DTE. */
    if (accepted && (frame->acknowledge || frame->v90_compatibility)
        && !g_v34_upstream_data_armed && g_v34) {
        int baud = v90_selected_upstream_baud_locked();
        int rate = v34_get_current_bit_rate(g_v34);
        int max_rate = v90_upstream_baud_max_bps(baud);

        if (rate > max_rate)
            rate = max_rate;
        rate = me_v90_upstream_cap(rate);
        /* trellis code 0 = V34_TRELLIS_16 (v34_tables.h, not exported); the
         * peer's own decode of our Type-0 MP confirms 16-state upstream. */
        if (v34_v90_prepare_upstream_data(g_v34, baud,
                                           v34_get_rx_high_carrier(g_v34),
                                           rate, 0) == 0) {
            g_v34_upstream_data_armed = true;
            g_v90_upstream_e_run = 0;
            ME_LOG("[ME] V.90 upstream RX data prepared (%d baud, %d bps, trellis 16); watching for E\n",
                   baud == 3 ? 3000 : 3200, rate);
        } else {
            ME_LOG("[ME] V.90 upstream RX data prepare FAILED (rate %d)\n", rate);
        }
    }
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
    uint32_t sync_before;

    (void)user_data;
    /* After CP' is accepted, the next thing on this channel is the peer's E
     * (20 consecutive ones) followed by B1 and data.  CP preambles are only
     * 17 ones + 0, so a run of 20 is unambiguous. */
    if (g_v34_upstream_data_armed && !g_v34_upstream_data_started && g_v34) {
        if (bit)
            g_v90_upstream_e_run++;
        else
            g_v90_upstream_e_run = 0;
        if (g_v90_upstream_e_run >= 20) {
            if (v34_begin_rx_data(g_v34) == 0) {
                g_v34_upstream_data_started = true;
                ME_LOG("[ME] V.90 upstream E detected; V.34 RX entering data mode\n");
                trace_phase("V90 upstream E -> V34 RX DATA");
            }
            return;
        }
    }
    /* V90_RENEG_BIT_DUMP: the recovered Phase-4 bit stream while a §9.6
     * renegotiation's CP search is armed, one ASCII '0'/'1' per bit.  The
     * framer's counters say whether a frame was found; this says what the
     * stream looked like, which is what separates "the peer sent SCR and
     * nothing else" from "CP was there and we demodulated it wrongly". */
    if (g_v90_reneg_cp_rx_marked) {
        static FILE *reneg_dump = NULL;
        static int reneg_dump_checked = 0;

        if (!reneg_dump_checked) {
            const char *path = getenv("V90_RENEG_BIT_DUMP");

            reneg_dump_checked = 1;
            if (path && path[0])
                reneg_dump = fopen(path, "wb");
        }
        if (reneg_dump) {
            fputc(bit ? '1' : '0', reneg_dump);
            fflush(reneg_dump);
        }
    }
    rejected_before = g_v90_cp_rx.rejected_frames;
    sync_before = g_v90_cp_rx.sync_candidates;
    (void)v90_cp_rx_put_bit(&g_v90_cp_rx, bit);
    /* A CP-shaped frame boundary on the wire is an answer even where the S
     * detector missed it -- §9.6.1.2.3's CP is the substance of the reply. */
    if (g_v90 && g_v90_reneg_cp_rx_marked
        && g_v90_cp_rx.sync_candidates != sync_before)
        v90_rate_renegotiation_note_answer(g_v90);
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
        int baud_code;
        int cpt_accept_sample;
        unsigned post_cpt_attempt;
        unsigned generation;
        v90_cp_live_rx_counters_t rx_baseline;
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
        baud_code = g_v90_cp_live_baud_code;
        cpt_accept_sample = g_v90_cp_live_cpt_accept_sample;
        post_cpt_attempt = 0;
        if (expected_compatibility)
            post_cpt_attempt = ++g_v90_cp_live_post_cpt_attempts;
        rx_baseline = g_v90_cp_live_rx_baseline;
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
                                    baud_code,
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
                {
                    /* Log the recovered transmitter (and, if different, codec)
                     * constellation Ucodes so a live capture can be checked
                     * against the peer's V90TRN2Designer "ADI design report".
                     * A mismatch here means our TRN2d/MP is shaped to the wrong
                     * levels and the peer's Phase-4 equalizer will diverge. */
                    char tx_set[256];
                    char codec_set[256];

                    v90_cp_format_constellation(&diag.frame, 0,
                                                tx_set, sizeof(tx_set));
                    fprintf(stderr,
                            "[ME] V.90 recovered %s constellation[0] tx-Ucodes={%s} count=%d\n",
                            diag.frame.v90_compatibility ? "CP" : "CPt",
                            tx_set,
                            vpcm_cp_mask_population(diag.frame.masks[0]));
                    if (diag.frame.codec_constellations_differ) {
                        for (int ucode = VPCM_CP_MASK_BITS - 1, pos = 0;
                             ucode >= 0; ucode--) {
                            if (!vpcm_cp_mask_get(diag.frame.codec_masks[0], ucode))
                                continue;
                            pos += snprintf(codec_set + pos,
                                            sizeof(codec_set) - pos,
                                            "%s%d", pos ? "," : "", ucode);
                            if (pos >= (int)sizeof(codec_set) - 1)
                                break;
                        }
                        fprintf(stderr,
                                "[ME] V.90 recovered %s constellation[0] codec-Ucodes={%s}\n",
                                diag.frame.v90_compatibility ? "CP" : "CPt",
                                codec_set);
                    }
                }
                accepted = v90_accept_cp_diag_locked(&diag, "batch");
            }
            pthread_mutex_unlock(&g_state_mtx);
        } else if (snapshot && expected_compatibility
                   && (post_cpt_attempt == 1
                       || (post_cpt_attempt % 2) == 0)) {
            vpcm_cp_diag_t repeated_cpt_diag;
            v90_cp_live_meta_t repeated_cpt_meta;
            v90_cp_live_rx_counters_t rx_current;
            uint64_t energy = 0;
            uint64_t input_bits_delta;
            uint32_t sync_delta;
            uint32_t valid_delta;
            uint32_t rejected_delta;
            uint32_t crc_delta;
            uint32_t structure_delta;
            uint32_t semantic_delta;
            double recent_rms = 0.0;
            int recent_start = sample_count - 4000;
            int recent_count;
            int repeated_cpt_sample = -1;
            bool repeated_cpt;
            bool current;
            const char *classification;

            memset(&repeated_cpt_diag, 0, sizeof(repeated_cpt_diag));
            memset(&repeated_cpt_meta, 0, sizeof(repeated_cpt_meta));
            repeated_cpt = v90_cp_live_recover(snapshot,
                                                sample_count,
                                                phase4_hint,
                                                baud_code,
                                                0,
                                                g_law == ME_LAW_ALAW,
                                                &repeated_cpt_diag,
                                                &repeated_cpt_meta);
            if (repeated_cpt)
                repeated_cpt_sample = repeated_cpt_meta.frame_sample;

            if (recent_start < cpt_accept_sample)
                recent_start = cpt_accept_sample;
            if (recent_start < 0)
                recent_start = 0;
            if (recent_start > sample_count)
                recent_start = sample_count;
            recent_count = sample_count - recent_start;
            for (int i = recent_start; i < sample_count; i++)
                energy += (uint64_t)((int64_t)snapshot[i] * snapshot[i]);
            if (recent_count > 0)
                recent_rms = sqrt((double)energy / recent_count);

            pthread_mutex_lock(&g_state_mtx);
            v90_cp_live_read_rx_counters_locked(&rx_current);
            pthread_mutex_lock(&g_v90_cp_live_mtx);
            current = generation == g_v90_cp_live_generation;
            pthread_mutex_unlock(&g_v90_cp_live_mtx);
            current = current && g_state == ME_TRAINING
                && g_mod == ME_MOD_V90 && g_v90;
            pthread_mutex_unlock(&g_state_mtx);

            input_bits_delta = v90_cp_live_counter_delta64(
                rx_current.input_bits, rx_baseline.input_bits);
            sync_delta = v90_cp_live_counter_delta(
                rx_current.sync_candidates, rx_baseline.sync_candidates);
            valid_delta = v90_cp_live_counter_delta(
                rx_current.valid_frames, rx_baseline.valid_frames);
            rejected_delta = v90_cp_live_counter_delta(
                rx_current.rejected_frames, rx_baseline.rejected_frames);
            crc_delta = v90_cp_live_counter_delta(
                rx_current.crc_rejected_frames,
                rx_baseline.crc_rejected_frames);
            structure_delta = v90_cp_live_counter_delta(
                rx_current.structure_rejected_frames,
                rx_baseline.structure_rejected_frames);
            semantic_delta = v90_cp_live_counter_delta(
                rx_current.semantic_rejected_frames,
                rx_baseline.semantic_rejected_frames);

            if (repeated_cpt
                && repeated_cpt_sample >= recent_start) {
                /* The batch search deliberately retains pre-transition CPt.
                 * Do not call that historical frame "repeating" after the
                 * newest 500 ms has moved into optional §9.4.2.2 SCR. */
                classification = "repeating-CPt";
            } else if (recent_rms < 64.0) {
                classification = "silence";
            } else if (crc_delta > 0) {
                classification = "CP-like-crc-reject";
            } else if (semantic_delta > 0) {
                classification = "CP-like-semantic-reject";
            } else if (structure_delta > 0 || rejected_delta > 0) {
                classification = "CP-like-structure-reject";
            } else if (sync_delta > 0) {
                classification = "CP-sync-candidate-incomplete";
            } else if (input_bits_delta == 0) {
                classification = "upstream-carrier-not-demodulated";
            } else {
                classification = "SCR-or-unresolved-QAM";
            }

            if (current) {
                fprintf(stderr,
                        "[ME] V.90 post-CPt classifier: class=%s "
                        "attempt=%u samples=%d recent-rms=%.0f "
                        "newest-CPt=%d transition=%d "
                        "live-bits=%llu sync=%u valid=%u rejected=%u "
                        "(crc=%u structure=%u semantic=%u)\n",
                        classification,
                        post_cpt_attempt,
                        sample_count,
                        recent_rms,
                        repeated_cpt_sample,
                        cpt_accept_sample,
                        (unsigned long long)input_bits_delta,
                        sync_delta,
                        valid_delta,
                        rejected_delta,
                        crc_delta,
                        structure_delta,
                        semantic_delta);
            }
        }
        free(snapshot);

        pthread_mutex_lock(&g_v90_cp_live_mtx);
        g_v90_cp_live_running = false;
        if (!accepted
            && generation == g_v90_cp_live_generation
            && expected_compatibility
                 == g_v90_cp_live_expected_compatibility
            && g_v90_cp_live_phase4_hint >= 0) {
            /* Retry on a fresh 40 ms of waveform.  A failed snapshot is not
             * evidence for a frame; only a complete CRC-valid observation
             * can advance the modem state. */
            g_v90_cp_live_next_request =
                g_v90_cp_live_sample_count + V90_CP_LIVE_RETRY_SAMPLES;
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

static void v92_upstream_live_byte(void *user_data, uint8_t byte)
{
    (void)user_data;
    ds_rx_push_bytes(&g_data_stack, &byte, 1);
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
            if (accepted) {
                v92_cpd_frame_t cpd;

                /* §9.6.1.1.6: our CPd defines the waveform that follows
                 * E2u.  Arm the B1u correlator as soon as that profile is
                 * fixed; its 48-frame validation supplies the E2u-to-B1u
                 * boundary without treating a random zero run as E2u. */
                if (v90_build_v92_cpd_frame(g_v90, &cpd)
                    && v92_upstream_b1_rx_init(
                        &g_v92_upstream_rx, &cpd,
                        v92_upstream_live_byte, NULL)) {
                    g_v92_upstream_rx_active = true;
                    g_v92_upstream_lock_logged = false;
                    ME_LOG("[ME] V.92 PCM-upstream B1u receiver armed: drn=%u rate=%d bps\n",
                           (unsigned)cpd.selected_upstream_drn,
                           ((int)cpd.selected_upstream_drn + 17)*8000/6);
                } else {
                    g_v92_upstream_rx_active = false;
                    ME_LOG("[ME] V.92 PCM-upstream CPd profile cannot arm B1u receiver\n");
                }
            }
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
    /* Before g_v34: the analogue Phase 3 borrows it as its modulator. */
    if (g_v90a) {
        v90_analogue_phase3_free(g_v90a);
        v90a_linear_free(g_v90a_linear);
        g_v90a_linear = NULL;
        v90a_fse_free(g_v90a_fse);
        g_v90a_fse = NULL;
        g_v90a_16k = false;
        g_v90a_ladder_set = false;
        g_v90a_sd_fill = 0;
        g_v90a_sd_fitted = false;
        g_v90a_sd_score_logged = 0;
        if (g_v90a_sd) {
            v90a_sd_free(g_v90a_sd);
            g_v90a_sd = NULL;
        }
        g_v90a_dil_tracking = false;
        g_v90a = NULL;
    }
    g_v90a_started = false;
    g_v90a_complete_logged = false;
    g_v90a_failed_logged = false;
    g_v90a_retrain_logged = false;
    g_v90a_data_start_samples = 0;
    g_v90a_rr_triggered = false;
    g_v90a_rr_deadline_logged = false;
    g_v90a_cleardown = false;
    g_v90a_rx_codewords = 0;
    g_v90a_data_diag_bits = 0;
    g_v90a_data_diag_zeros = 0;
    g_v90a_data_bits_seen = 0;
    g_v90a_data_adp_window = 0;
    g_v90a_data_adp_window_bits = 0;
    g_v90a_data_adp_count = 0;
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
    g_v90_reject_info1a_logged = false;
    g_v90_fallback_v34_logged = false;
    g_v90_fallback_phase4_released = false;
    g_v90_ja_failed_attempts = 0;
    g_v90_phase3_first_samples = 0;
    g_v90_dil_capture_start_logged = false;
    g_v90_phase2_restarts = 0;
    /* Per call, not per process: this server runs many calls in one. */
    g_loss_retrains = 0;
    g_last_loss_retrain_ms = 0;
    g_retrain_from_data = false;
    g_v34_reneg_start_ms = 0;
    g_v34_data_entry_ms = 0;
    g_v34_data_entry_samples = 0;
    g_v34_reneg_probe_done = false;
    g_v90_reneg_probe_done = false;
    g_v90_reneg_await_s = false;
    g_v90_reneg_cp_rx_marked = false;
    g_v90_reneg_cp_ack_seen = false;
    g_v34_retrain_probe_done = false;
    g_tx_disrupt_logged = 0;
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    v90_cp_rx_reset(&g_v90_cp_rx);
    v90_cp_rx_clear_votes(&g_v90_cp_rx);
    v92_cp_rx_reset(&g_v92_cp_rx);
    v92_p3_rx_init(&g_v92_p3_rx);
    v92_cp_rx_reset(&g_v92_p3_cpt_rx);
    memset(&g_v92_trn2u_demod, 0, sizeof(g_v92_trn2u_demod));
    memset(&g_v92_p3_cpt_demod, 0, sizeof(g_v92_p3_cpt_demod));
    g_v92_p3_rx_active = false;
    g_v92_p3_rx_result_applied = false;
    g_v92_p3_rx_failure_logged = false;
    g_v92_p3_cpt_active = false;
    v92_su_rx_reset_locked();
    g_v92_trn2u_active = false;
    g_v92_upstream_rx_active = false;
    g_v92_upstream_lock_logged = false;
    g_v92_active = false;
    g_v92_v8_offered = false;
    g_v92_info0_local_advertised = false;
    g_v92_info0_peer_capable = false;
    g_v92_info0_peer_short_phase2 = false;
    g_v92_info0_mutual = false;
    g_v92_info0_peer_logged = false;
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
 * Called from the TX media callback (Jd-without-S recovery) or the RX path
 * (peer retrain per §9.5.1.2), always with g_state_mtx held.  It
 * intentionally preserves the overall training timeout: a broken bearer
 * still falls back rather than retrying indefinitely.
 */
/* Plain V.34 §11.5: back to Phase 2 from wherever we are, keeping the DTE
 * link.  The V.90 path has restart_v90_phase2_locked(); this is its plain
 * V.34 counterpart, and like it, §11.5 says only "turn OFF circuit 106,
 * clamp circuit 104 to binary one" -- the error-control link above the
 * physical layer survives a retrain, so the data stack is not touched. */
static bool restart_v34_phase2_locked(const char *reason)
{
    int baud;
    int bps;

    if (g_mod != ME_MOD_V34 || !g_v34)
        return false;

    baud = g_v34_start_baud ? g_v34_start_baud : 3200;
    bps = g_v34_start_bps ? g_v34_start_bps : max_v34_bps_for_baud(baud);
    /* Keep the half-duplex role across a retrain.  The `true` here was
       unconditional, so a clause 12 call that took a retrain for any reason
       came back as a full-duplex V.34 data modem: measured against a Canon
       TR7560, the stage trace goes HDX_INITIAL_A ... HDX_SECOND_A_WAIT and
       then, after the retrain, INITIAL_A / FIRST_A / L1_L2 / PRE_INFO1_A --
       the plain V.34 answer-modem timetable, which a fax machine cannot
       complete.  A retrain is a fresh start-up of the SAME call, and V.34
       12.8 keeps it half-duplex. */
    if (v34_restart(g_v34, baud, bps, v34_is_duplex(g_v34)) != 0) {
        ME_LOG("[ME] V.34 Phase 2 restart failed (%d baud, %d bps)\n",
               baud, bps);
        trace_phase("V34 Phase2 restart failed");
        return false;
    }
    if (g_state == ME_DATA) {
        g_retrain_from_data = true;
        g_state = ME_TRAINING;
        g_phase_start_ms = trace_now_ms();
    }
    /* §11.5 always puts silence and then this role's tone on the wire, NOT
       the INFO0 exchange v34_restart() re-enters at.  This must not be scoped
       to ME_DATA: a peer may initiate a retrain during Phases 2-4, and a
       retrain already in progress has g_state == ME_TRAINING while retaining
       g_retrain_from_data.  In either case sending INFO0 here leaves the peer
       waiting for the tone required by §11.5.1.2/§11.5.2.2. */
    v34_start_retrain(g_v34);
    ME_LOG("[ME] V.34: %s; restarting Phase 2 (%d baud / %d bps)\n",
           reason ? reason : "restart requested", baud, bps);
    trace_phase("V34 restart Phase2: %d/%d", baud, bps);
    return true;
}

static bool restart_v90_phase2_locked(const char *reason)
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
    g_v90_reject_info1a_logged = false;
    g_v90_fallback_v34_logged = false;
    g_v90_fallback_phase4_released = false;
    /* NOT g_v90_ja_failed_attempts: this runs on every retrain restart, and
     * the whole point of that counter is to survive them and count how many
     * Phase 3 attempts in a row failed.  It resets per call, beside
     * g_v90_phase2_restarts, which is scoped the same way for the same
     * reason. */
    g_v90_dil_capture_start_logged = false;
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    v90_reset_upstream_data_arming();
    g_v92_active = false;
    g_v92_info0_peer_capable = false;
    g_v92_info0_peer_short_phase2 = false;
    g_v92_info0_mutual = false;
    g_v92_info0_peer_logged = false;
    g_v92_trn2u_active = false;
    g_v92_upstream_rx_active = false;
    g_v92_upstream_lock_logged = false;
    memset(&g_v92_trn2u_demod, 0, sizeof(g_v92_trn2u_demod));
    v92_p3_rx_init(&g_v92_p3_rx);
    v92_cp_rx_reset(&g_v92_p3_cpt_rx);
    memset(&g_v92_p3_cpt_demod, 0, sizeof(g_v92_p3_cpt_demod));
    g_v92_p3_rx_active = false;
    g_v92_p3_rx_result_applied = false;
    g_v92_p3_rx_failure_logged = false;
    g_v92_p3_cpt_active = false;
    v92_su_rx_reset_locked();
    v90_cp_rx_reset(&g_v90_cp_rx);
    v90_cp_rx_clear_votes(&g_v90_cp_rx);
    v92_cp_rx_reset(&g_v92_cp_rx);
    v90_dil_capture_reset();
    /* A Phase-4 retrain arrives with the strict batch CP receiver armed and
       mid-capture; its Phase-4 hint and counters belong to the failed
       attempt. */
    if (g_v90_cp_live_thread_started)
        v90_cp_live_capture_reset_locked();

    /* v34_restart preserves the selected V.90 mode, but reapply the mode and
       receiver callbacks explicitly because this is also where the modified
       SpanDSP tree re-primes INFO0a framing for the answerer. */
    v34_set_v90_mode(g_v34, (g_law == ME_LAW_ALAW) ? 1 : 0);
    v34_set_v92_info0_capabilities(g_v34,
                                    g_v92_info0_local_advertised ? 1 : 0,
                                    0);
    v34_set_v92_pcm_upstream_capability(g_v34,
                                        v92_pcm_upstream_advertised() ? 1 : 0);
    v34_tx_power(g_v34, -10.0f);
    v34_set_put_aux_bit(g_v34, v34_put_aux_bit_cb, NULL);
    v34_set_put_phase4_bit(g_v34, v90_live_cp_bit, NULL);
    notch_filter_init(&g_notch, 1200.0f, 30.0f, 8000.0f);

    /* §9.5/§11.5 puts a retrain at "any time", data mode included, and the
       transmitter must leave the data mapper for the Phase 2 tone exchange.
       Nothing else moves the engine state back, so a retrain taken from data
       mode used to leave g_state at ME_DATA: the V.90 transmit path then hit
       its `if (!g_v90) return false` and the call went quiet in both
       directions rather than resynchronising. */
    if (g_state == ME_DATA) {
        g_retrain_from_data = true;
        g_state = ME_TRAINING;
        g_phase_start_ms = trace_now_ms();
        ME_LOG("[ME] V.90: retraining out of data mode; DTE data clamped, "
               "error-control link retained (§9.5/§11.5)\n");
    }

    /* Every caller of this helper is a §9.5 retrain, not a fresh startup:
       peer request, failed Jd/S exchange, failed §9.6, or carrier loss.
       §9.5.1.1 and §9.5.1.2 both require 70 ms silence followed by Tone B
       from the digital modem and omit INFO0.  Centralising that seam also
       keeps timeout and training-stage retrains from accidentally emitting
       v34_restart()'s INITIAL_PREAMBLE/INFO0d instead. */
    v34_v90_start_retrain_response(g_v34);

    g_v90_phase2_restarts++;
    trace_phase("V90 restart Phase2: attempt=%u profile=3200/%d",
                g_v90_phase2_restarts, bps);
    ME_LOG("[ME] V.90: %s; restarting Phase 2 (%u, 3200 baud / %d bps)\n",
           reason ? reason : "restart requested",
           g_v90_phase2_restarts, bps);
    return true;
}

/* V.90 §9.5.2, analogue role.  Unlike a fresh call, retrain skips INFO0 and
 * resumes at the Tone A/Tone B ranging exchange (§9.2.2.1.3-.4). */
static bool restart_v90_analogue_phase2_locked(const char *reason)
{
    int bps;

    if (g_mod != ME_MOD_V90 || !me_v90_analogue_role() || !g_v34)
        return false;
    bps = g_v34_start_bps ? g_v34_start_bps : max_v34_bps_for_baud(3200);

    if (g_v90a) {
        v90_analogue_phase3_free(g_v90a);
        v90a_linear_free(g_v90a_linear);
        g_v90a_linear = NULL;
        v90a_fse_free(g_v90a_fse);
        g_v90a_fse = NULL;
        g_v90a_16k = false;
        g_v90a_ladder_set = false;
        g_v90a_sd_fill = 0;
        g_v90a_sd_fitted = false;
        g_v90a_sd_score_logged = 0;
        if (g_v90a_sd) {
            v90a_sd_free(g_v90a_sd);
            g_v90a_sd = NULL;
        }
        g_v90a_dil_tracking = false;
        g_v90a = NULL;
    }
    g_v90a_started = false;
    g_v90a_complete_logged = false;
    g_v90a_failed_logged = false;
    g_v90a_retrain_logged = false;
    g_v90a_data_start_samples = 0;
    g_v90a_rr_triggered = false;
    g_v90a_rr_deadline_logged = false;
    g_v90a_cleardown = false;
    g_v90a_rx_codewords = 0;
    g_v90a_data_bits_seen = 0;
    g_v90a_data_adp_count = 0;

    if (v34_restart(g_v34, 3200, bps, true) != 0) {
        ME_LOG("[ME] V.90 analogue retrain restart failed (3200/%d)\n", bps);
        return false;
    }
    v34_set_v90_mode(g_v34, (g_law == ME_LAW_ALAW) ? 1 : 0);
    v34_set_v90_u_info(g_v34, g_v90a_u_info);
    v34_tx_power(g_v34, -10.0f);
    v34_v90_start_analogue_retrain(g_v34);

    data_stack_prepare(bps);
    g_state = ME_TRAINING;
    g_notch.active = false;
    notch_filter_init(&g_notch, 2400.0f, 30.0f, 8000.0f);
    g_v90_phase2_restarts++;
    ME_LOG("[ME] V.90 analogue: %s; §9.5.2 retrain %u (3200/%d)\n",
           reason ? reason : "retrain requested", g_v90_phase2_restarts, bps);
    trace_phase("V90a 9.5.2 retrain attempt=%u profile=3200/%d",
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

static void v90_note_ja_confirmed_by_descriptor(void);

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

    /* Dump the frame that actually parsed, at the offset it parsed from.
     * Comparing a SUCCESSFUL frame against the in-tree fixture is what decides
     * whether that fixture is the right reference for this peer at all -- see
     * docs/v90_phase3_s_and_rbs_false_positive.md 35i.  A dump taken on any
     * other attempt, or at any other offset, answers a different question. */
    {
        const char *pfx = getenv("ME_V90_JA_DUMP_ON_PARSE");

        if (pfx && *pfx) {
            char path[1024];
            FILE *fp;

            snprintf(path, sizeof(path), "%s-parsed.bits", pfx);
            fp = fopen(path, "wb");
            if (fp) {
                for (int i = 0; i < shifted_bits; i++) {
                    uint8_t bit = (uint8_t) v90_dil_capture_get_bit(start + i);

                    fwrite(&bit, 1, 1, fp);
                }
                fclose(fp);
            }
            ME_LOG("[ME] V.90 Ja: parsed frame dumped from capture offset %d "
                   "(%d bits available)\n", start, shifted_bits);
        }
    }

    g_v90_pending_dil = desc;
    g_v90_pending_dil_valid = true;
    if (g_v90)
        v90_set_dil_descriptor(g_v90, &g_v90_pending_dil);
    ME_LOG("[ME] V.90: parsed Ja DIL descriptor (N=%u LSP=%u LTP=%u)\n",
            desc.n, desc.lsp, desc.ltp);
    trace_phase("V90 parsed Ja DIL descriptor: N=%u LSP=%u LTP=%u",
                desc.n, desc.lsp, desc.ltp);
    g_v90_dil_parse_logged = true;
    v90_note_ja_confirmed_by_descriptor();
    return true;
}

/* Should a heuristic Ja detector be allowed to start Sd?
 *
 * §9.3.1.3 has the digital modem transmit Sd once it detects Ja, and V.92
 * §9.5.1.1.3 says what "detects" has to mean: "after receiving a DIL descriptor
 * of Ja" -- a CRC-valid descriptor, not an energy edge.  The V.92 path already
 * enforces exactly that (v92_apply_p3_ja_locked refuses a non-strict Ja).
 *
 * It matters because starting Sd is destructive to the thing we still need.
 * §9.3.2.4 has the analogue modem stop Ja as soon as it sees our Sd→S̄d, and
 * measured against the Courier (2026-08-12, both captures, bit-identical
 * descriptors) the DIL descriptor sits in the *last 20%* of Ja: the peer trains
 * for ~2.2 s and the only CRC-valid copy starts at bit ~14290 of ~17000, with
 * one frame (2060 bits) of margin.  So any detector that fires before the
 * descriptor parses truncates the one frame that carries DIL -- one live run
 * missed it by 152 bits.  July's runs decoded it only because the heuristics
 * happened to fire late.
 *
 * The heuristics stay available for the case where no descriptor can ever
 * arrive: a peer that declined V.90 (Table 10 downstream code 0-5) is doing
 * plain V.34, whose J carries no DIL descriptor at all.
 *
 * ME_V90_JA_HEURISTICS=1 restores the old always-on behaviour.
 * ME_V90_JA_HEURISTIC_FALLBACK_MS>0 re-enables them that many ms after the
 * first suppressed attempt, as a bounded escape hatch.  Default 0: never. */
static bool v90_ja_heuristic_allowed(const char *source)
{
    static uint64_t first_suppressed_ms = 0;
    static bool     logged[4];
    v34_v90_info1a_t received;
    int  idx;
    long fallback_ms;

    /* Descriptor already in hand -- J has fired, nothing left to protect. */
    if (g_v90_dil_parse_logged)
        return true;
    if (parse_env_int("ME_V90_JA_HEURISTICS", 0) != 0)
        return true;
    /* Peer declined V.90: plain V.34 J, no descriptor will ever parse. */
    if (g_v34
        && v34_get_v90_received_info1a(g_v34, &received)
        && received.u_info > 0
        && received.downstream_rate_code >= 0
        && received.downstream_rate_code <= 5)
        return true;

    /* §9.3.1.3: "After receiving Ja, the digital modem may wait for up to
     * 500 ms and shall then transmit signal Sd for 384T".  The condition is
     * RECEIVING Ja -- a signal -- not decoding a CRC-valid DIL descriptor out
     * of it, and the wait it allows is bounded.  Suppressing the heuristic
     * with no bound waits instead for the 6000 ms interop fallback in
     * v90.c, which is 12x the clause's allowance and far past this peer's
     * patience: measured 2026-08-27, SmartLink enters WaitForSd, receives
     * digital silence from us (its own log: "Error Energy = -0.000" for the
     * whole window) and retrains after ~1.9 s.  On a rig sitting in the §34
     * Ja-parse blocker that cost 0 of 25 calls any V.90 data mode at all;
     * with the clause's own 500 ms, calls reached data mode again.
     *
     * It does not disturb a healthy call, which is what makes 500 the right
     * default rather than a rescue knob: §35k measured a healthy descriptor
     * arriving a median 0.3 s into Phase 3, so the bound is only reached when
     * the descriptor is late or never coming -- exactly the case where the
     * old behaviour deadlocked.  0 restores the unbounded wait. */
    fallback_ms = parse_env_int("ME_V90_JA_HEURISTIC_FALLBACK_MS", 500);
    if (first_suppressed_ms == 0)
        first_suppressed_ms = trace_now_ms();
    else if (fallback_ms > 0
             && trace_now_ms() - first_suppressed_ms >= (uint64_t) fallback_ms) {
        ME_LOG("[ME] V.90 Ja: no descriptor after %ld ms; allowing %s heuristic "
               "(ME_V90_JA_HEURISTIC_FALLBACK_MS)\n", fallback_ms, source);
        return true;
    }

    idx = (source[0] == 'p') ? 0 : (source[0] == 'e') ? 1 : 2;
    if (!logged[idx]) {
        logged[idx] = true;
        ME_LOG("[ME] V.90 Ja: suppressing %s heuristic; §9.3.1.3/V.92 §9.5.1.1.3 "
               "start Sd only on a CRC-valid DIL descriptor, and starting early "
               "makes the peer stop Ja (§9.3.2.4) before the descriptor arrives\n",
               source);
        trace_phase("V90 Ja heuristic suppressed (%s), awaiting descriptor", source);
    }
    return false;
}

/* A CRC-valid DIL descriptor is proof the peer is transmitting Ja right now --
 * strictly better evidence than the energy-gap heuristic or the J look-ahead
 * timer that currently drive the Ja event. Use it as soon as we have it.
 *
 * Measured live, the descriptor parses well before either of those fire (in one
 * run the parse landed ~670 log lines ahead of "analogue Ja detected", and the
 * look-ahead configured for 3000 bits did not actually trigger until bit
 * 22484). Waiting for them after we already know the answer is what makes the
 * whole of Phase 3 run late and on timers rather than in step with the peer. */
static void v90_note_ja_confirmed_by_descriptor(void)
{
    /* Restart the deadline clock: it measures Phase 3 time WITHOUT a
     * descriptor, so a call that has just had one starts again from zero.
     * Anchored once per call it would instead measure time since the first
     * Phase 3 entry ever, and a call that ran data mode for a minute and then
     * retrained would be past the deadline on its first failed attempt. */
    g_v90_phase3_first_samples = 0;
    bool accepted;

    if (!g_v90 || !g_v34)
        return;
    if (v90_get_tx_phase(g_v90) != V90_TX_WAIT_JA)
        return;
    accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_J);
    ME_LOG("[ME] V.90: Ja confirmed by CRC-valid DIL descriptor; starting Phase 3 "
           "without waiting for the energy gap or look-ahead (accepted=%d)\n",
           accepted ? 1 : 0);
    trace_phase("V90 Ja confirmed by descriptor (accepted=%d)", accepted ? 1 : 0);
    if (accepted)
        v34_v90_arm_phase3_s_detector(g_v34);
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

/* Recover a DIL descriptor from repeated Ja transmissions.
 *
 * The analogue modem repeats its Ja descriptor continuously; measured on the
 * d-modem rig the period is 1702 bits and two clean repeats differed in only
 * 6 of 562 descriptor bits, all in one burst. So rather than needing any
 * single repeat to arrive perfect, combine them: where the repeats agree take
 * the agreed bit, and where they disagree brute-force the (few) combinations
 * against v90_parse_dil_descriptor(), which CRC-checks. That is the same
 * CRC-guided repeated-frame idea already used for CP frames.
 *
 * Deliberately keyed on two repeats rather than a three-way majority vote:
 * the peer's Ja window is only ~1.62 s (~7800 bits), and two repeats cost
 * ~0.71 s against ~1.06 s for three -- majority voting would eat most of the
 * budget for no benefit when the disagreement count is this small.
 * V90_JA_VOTE_MAX_DIFFS bounds the brute force at 2^n parses. */
#define V90_JA_VOTE_MAX_REPEATS  6
#define V90_JA_VOTE_MAX_DIFFS    12

static bool v90_dil_vote_and_parse(const uint8_t *bits,
                                   int total_bits,
                                   const int *starts,
                                   int start_count,
                                   int period)
{
    uint8_t voted[V90_DIL_CAPTURE_MAX_BITS];
    uint8_t packed[(V90_DIL_CAPTURE_MAX_BITS + 7) / 8];
    int diff_pos[V90_JA_VOTE_MAX_DIFFS];
    v90_dil_desc_t desc;
    int diff_count = 0;
    int span;
    int i;

    if (start_count < 2 || period < 206)
        return false;
    /* Vote over a whole period. Do NOT clamp to what the last repeat has
     * left: a full descriptor is up to ~1701 bits (N=144 on this peer) and
     * the final repeat is usually truncated by the end of the capture, so
     * clamping to it silently produces a short buffer that fails the
     * bit_len < descriptor_bits check and looks like a decode failure. */
    span = period;
    if (starts[0] + span > total_bits)
        span = total_bits - starts[0];
    if (span < 206)
        return false;

    for (i = 0; i < span; i++) {
        int ones = 0;
        int available = 0;
        int k;

        /* Only count repeats that actually reach this offset, so early
         * repeats still carry the tail of the descriptor. */
        for (k = 0; k < start_count; k++) {
            if (starts[k] + i >= total_bits)
                continue;
            ones += bits[starts[k] + i] & 1;
            available++;
        }
        if (available == 0) {
            span = i;
            break;
        }
        if (ones == 0) {
            voted[i] = 0;
        } else if (ones == available) {
            voted[i] = 1;
        } else if (ones * 2 > available) {
            voted[i] = 1;                     /* clear majority */
        } else if (ones * 2 < available) {
            voted[i] = 0;
        } else {
            /* Genuine tie (only possible with an even repeat count). Record
             * it for the brute force rather than guessing. */
            voted[i] = 0;
            if (diff_count < V90_JA_VOTE_MAX_DIFFS)
                diff_pos[diff_count] = i;
            diff_count++;
        }
    }

    if (diff_count > V90_JA_VOTE_MAX_DIFFS)
        return false;

    for (unsigned int combo = 0; combo < (1u << diff_count); combo++) {
        memset(packed, 0, sizeof(packed));
        for (i = 0; i < diff_count; i++)
            voted[diff_pos[i]] = (uint8_t) ((combo >> i) & 1u);
        /* LSB-first within each byte, matching v90_get_packed_bit() and the
         * rest of the capture path. Packing this MSB-first silently reverses
         * every multi-bit field -- N=144 reads back as 9 -- while leaving
         * palindromic fields like LSP/LTP=120 looking correct, so it fails in
         * a way that looks like bit errors rather than a format bug. */
        for (i = 0; i < span; i++)
            if (voted[i] & 1)
                packed[i >> 3] |= (uint8_t) (1U << (i & 7));
        if (v90_parse_dil_descriptor(&desc, packed, span)) {
            g_v90_pending_dil = desc;
            g_v90_pending_dil_valid = true;
            if (g_v90)
                v90_set_dil_descriptor(g_v90, &g_v90_pending_dil);
            ME_LOG("[ME] V.90: Ja DIL descriptor recovered by repeated-frame voting "
                   "(%d repeats, period=%d, %d tie bits, combo=%u): N=%u LSP=%u LTP=%u\n",
                   start_count, period, diff_count, combo,
                   desc.n, desc.lsp, desc.ltp);
            trace_phase("V90 Ja descriptor via voting: repeats=%d period=%d N=%u",
                        start_count, period, desc.n);
            g_v90_dil_parse_logged = true;
            v90_note_ja_confirmed_by_descriptor();
            return true;
        }
    }
    return false;
}

/* Find repeated descriptor preambles in one hypothesis' bitstream and try to
 * combine them. Returns true if a descriptor was recovered. */
static bool v90_dil_try_repeated_frames(const uint8_t *bits, int total_bits)
{
    int starts[V90_JA_VOTE_MAX_REPEATS * 4];
    int count = 0;
    int i;
    int a;

    if (total_bits < 2 * 206)
        return false;
    for (i = 0; i + 206 <= total_bits && count < (int) (sizeof(starts)/sizeof(starts[0])); i++) {
        int k;
        int ok = 1;

        for (k = 0; k < 17; k++) {
            if ((bits[i + k] & 1) == 0) {
                ok = 0;
                break;
            }
        }
        if (!ok || (bits[i + 17] & 1))
            continue;
        starts[count++] = i;
        i += 205;                    /* a descriptor cannot start inside itself */
    }
    if (count < 2)
        return false;

    /* Group starts sharing a consistent period, largest group first. */
    for (a = 0; a < count - 1; a++) {
        int b;

        for (b = a + 1; b < count; b++) {
            int period = starts[b] - starts[a];
            int group[V90_JA_VOTE_MAX_REPEATS];
            int gcount = 0;
            int c;

            if (period < 206)
                continue;
            for (c = a; c < count && gcount < V90_JA_VOTE_MAX_REPEATS; c++) {
                int delta = starts[c] - starts[a];

                if (delta % period == 0)
                    group[gcount++] = starts[c];
            }
            if (gcount >= 2
                && v90_dil_vote_and_parse(bits, total_bits, group, gcount, period)) {
                return true;
            }
        }
    }
    return false;
}

static bool v90_dil_capture_try_v34_hypotheses(void)
{
    uint8_t unpacked[V90_DIL_CAPTURE_MAX_BITS];
    int first_bits;

    /* g_v90_pending_dil_valid can mean either "decoded from this Ja" or
     * "preloaded by ME_V90_DIL_PROFILE".  Only the former makes another
     * decode unnecessary.  Treating a fallback as live receive evidence
     * disabled this entire path and left SmartLink waiting for Sd until the
     * unrelated J-lookahead timer expired. */
    /* What the descriptor parser is actually being handed.  v34rx has three
     * Ja sinks and only phase3_ja_capture_hyp[] (filled in PHASE3_WAIT_S)
     * reaches this search; the "Ja capture: emitted N bits" span_log comes
     * from a different buffer, so it can report tens of thousands of bits that
     * this function never sees.  Chasing that discrepancy cost a whole session
     * -- log the parser's own input, and log the RX stage with it, because the
     * arrays are legitimately empty in PHASE3_TRAINING (stage 11) and only
     * fill in PHASE3_WAIT_S (stage 10). */
    {
        static int calls = 0;

        if ((calls++ % 25) == 0 && me_verbose_enabled()) {
            int longest = 0, longest_h = -1;

            for (int h = 0; g_v34 && h < 24; h++) {
                int n = v34_v90_copy_phase3_ja_bits(g_v34, h, unpacked,
                                                    V90_DIL_CAPTURE_MAX_BITS);
                if (n > longest) { longest = n; longest_h = h; }
            }
            ME_LOG("[ME] V.90 Ja search input: t=%.3fs v34=%d parsed=%d rx_stage=%d "
                   "longest_hyp_len=%d (hyp %d)\n",
                   (double)g_rx_audio_samples / 8000.0,
                   g_v34 ? 1 : 0, g_v90_dil_parse_logged ? 1 : 0,
                   g_v34 ? v34_get_rx_stage(g_v34) : -1, longest, longest_h);
        }
    }
    /* V90_JA_BIT_DUMP=<path> writes every hypothesis' accumulated Ja bits as
     * one ASCII line of '0'/'1' per hypothesis, rewritten each time this runs.
     * The parser's verdict is a single bool; when it says no, the only way to
     * find out WHY is to look at the bits it was handed against Table 12. */
    {
        const char *dump = getenv("V90_JA_BIT_DUMP");

        if (dump && *dump && g_v34) {
            FILE *f = fopen(dump, "w");

            if (f) {
                for (int h = 0; h < 24; h++) {
                    int n = v34_v90_copy_phase3_ja_bits(g_v34, h, unpacked,
                                                        V90_DIL_CAPTURE_MAX_BITS);
                    if (n <= 0)
                        continue;
                    fprintf(f, "hyp%d %d ", h, n);
                    for (int b = 0; b < n; b++)
                        fputc((unpacked[b] & 1) ? '1' : '0', f);
                    fputc('\n', f);
                }
                fclose(f);
            }
        }
    }
    if (!g_v34 || g_v90_dil_parse_logged)
        return g_v90_dil_parse_logged;

    first_bits = v34_v90_copy_phase3_ja_bits(g_v34,
                                             0,
                                             unpacked,
                                             V90_DIL_CAPTURE_MAX_BITS);
    /* Exact anchor for the capture.  The periodic line below is sampled every
     * 25 RX frames, i.e. 500 ms, which is far too coarse to say where in the
     * peer's Phase 3 the capture begins -- and that is the difference between
     * "these bits are the peer's J" and "these bits are its Ja".  Log the
     * first bit's arrival to the frame. */
    if (first_bits > 0 && !g_v90_dil_capture_start_logged) {
        g_v90_dil_capture_start_logged = true;
        ME_LOG("[ME] V.90 Ja capture: first bits at t=%.3fs (%d bits)\n",
               (double)g_rx_audio_samples / 8000.0, first_bits);
    }
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
    if (!g_v90_dil_hyp_dumped && first_bits >= v90_ja_dump_min_bits()
        && getenv("ME_V90_JA_DUMP_EARLY")) {
        const char *pfx = getenv("ME_V90_JA_DUMP_PREFIX");
        if (pfx && *pfx) {
            char path[1024];
            for (int h = 0; h < 24; h++) {
                int nb = v34_v90_copy_phase3_ja_bits(g_v34, h, unpacked,
                                                     V90_DIL_CAPTURE_MAX_BITS);
                snprintf(path, sizeof(path), "%s-hyp%d.bits", pfx, h);
                FILE *fp = fopen(path, "wb");
                if (fp) { fwrite(unpacked, 1, (size_t)nb, fp); fclose(fp); }
                nb = v34_v90_copy_phase3_ja_raw_bits(g_v34, h, unpacked,
                                                     V90_DIL_CAPTURE_MAX_BITS);
                snprintf(path, sizeof(path), "%s-hyp%d.rawbits", pfx, h);
                fp = fopen(path, "wb");
                if (fp) { fwrite(unpacked, 1, (size_t)nb, fp); fclose(fp); }
            }
            g_v90_dil_hyp_dumped = true;
        }
    }
    int retry_bits = parse_env_int("ME_V90_DIL_HYP_RETRY_BITS", 512);
    if (retry_bits < 0)
        retry_bits = 512;
    /* Throttle on the TOTAL bits ever captured, not on what the copy-out
     * returned.  The capture is a ring now, so the returned length saturates
     * at the window size while bits keep arriving; keying the throttle on it
     * would satisfy "no new bits since last time" forever and stop the search
     * the moment the ring filled. */
    {
        int total_bits = v34_v90_phase3_ja_total_bits(g_v34, 0);

        if (total_bits < g_v90_dil_hyp_last_bits + retry_bits)
            return false;
        g_v90_dil_hyp_last_bits = total_bits;
    }

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

        /* No single repeat parsed cleanly. Combine repeats before moving on --
         * measured live, individual Ja repeats carry ~1% residual bit errors
         * in short bursts, which is exactly what repeated-frame recovery is
         * for. Runs per hypothesis, right after its single-frame attempt, so
         * it costs nothing extra in captured signal. */
        if (v90_dil_try_repeated_frames(unpacked, bits)) {
            ME_LOG("[ME] V.90: Ja descriptor recovered by voting on V.34 hypothesis %d\n",
                   hypothesis);
            return true;
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
    if (g_v34hdx_fax_control_started && di_fax_active())
        return di_fax_v34hdx_get_bit();
    bit = ds_tx_get_bit(&g_data_stack);
    return (bit == DS_TX_NO_DATA) ? SIG_STATUS_END_OF_DATA : bit;
}

static void v34_put_aux_bit_cb(void *user_data, int bit)
{
    (void)user_data;

    if (bit < 0 || bit > 1)
        return;
    /* A preloaded DIL profile supplies recovery data, not proof that Ja has
     * arrived.  Continue collecting until a live, CRC-valid descriptor has
     * actually parsed (V.90 §8.3.1 and §9.3.1.3). */
    if (g_mod != ME_MOD_V90 || g_v92_active || g_v90_dil_parse_logged)
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

/* SpanDSP FLOW logging is written synchronously from the media thread, and
 * during a Phase-3 retrain carousel the J detector alone emits hundreds of
 * lines a second -- measured at 126 MB in 40 minutes, about 50 KB/s.  That is
 * enough to stall the real-time path, and a stall shows up at the far end as
 * a discontinuity: the peer's Error Energy jumping from ~24 to +1600 and a
 * retrain request.  Default to the quiet level and let ME_V34_SPAN_FLOW_LOG=1
 * turn the diagnostics back on when a capture needs them. */
static int me_span_flow_level(void)
{
    const char *value = getenv("ME_V34_SPAN_FLOW_LOG");
    int level = SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL;

    /* The low bits of the level are a severity *threshold*, not a flag: ORing
     * in nothing leaves SPAN_LOG_NONE and silences every message, warnings
     * included.  Quiet means "warnings and worse", which is what the V.90
     * upstream probes are logged at. */
    level |= (value && atoi(value) != 0) ? SPAN_LOG_FLOW : SPAN_LOG_WARNING;
    return level;
}

static void v34_put_bit_cb(void *user_data, int bit)
{
    (void)user_data;
    if (g_v34hdx_fax_control_started && di_fax_active()) {
        di_fax_v34hdx_put_bit(bit);
        return;
    }
    if (bit < 0) {
        /* Status event from V.34 training state machine */
        trace_phase("V34 rx status=%s (%d)", signal_status_to_str(bit), bit);
        if (bit == SIG_STATUS_CARRIER_UP || bit == SIG_STATUS_TRAINING_SUCCEEDED) {
            if (g_state == ME_TRAINING) {
                int rate = v34_get_current_bit_rate(g_v34);
                if (g_mod == ME_MOD_V90) {
                    if (g_v90 && v90_training_complete(g_v90)) {
                        int downstream_rate = (v90_data_bits_per_frame(g_v90) * 8000) / 6;

                        if (downstream_rate <= 0)
                            downstream_rate = V90_RATE_BPS;
                        /* The second way into data mode, and it needs the
                           same §9.5/§11.5 treatment as
                           enter_v90_data_locked(): a retrain clamps 104 and
                           nothing more, so the error-control link above the
                           physical layer survives it. */
                        if (g_retrain_from_data) {
                            g_retrain_from_data = false;
                            g_data_connect_rate = downstream_rate;
                            ME_LOG("[ME] V.90 retrain complete; resuming the "
                                   "existing data link (%d bps)\n",
                                   downstream_rate);
                        } else {
                            data_stack_start_online(downstream_rate,
                                                    g_calling_party);
                        }
                        g_state = ME_DATA;
                        g_phase_start_ms = 0;
                        g_v34_data_entry_ms = trace_now_ms();
                        g_v34_data_entry_samples = g_rx_audio_samples;
                        ME_LOG("[ME] V.90 training complete (upstream V.34 %d bps, downstream PCM %d bps)\n",
                                rate, downstream_rate);
                        trace_phase("V90 enter DATA: upstream=%d downstream=%d", rate, downstream_rate);
                        v90_reset_data_mode(g_v90);
                        g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
                        if (g_data_framing != DS_FRAMING_V42
                            && !g_data_connect_reported) {
                            g_data_connect_reported = true;
                            di_on_connected(downstream_rate);
                        }
                    } else if (!g_v90_completion_deferred_logged) {
                        ME_LOG("[ME] V.90 received generic training success from V.34, but V.90 startup is not complete yet; remaining in TRAINING\n");
                        trace_phase("V90 deferred DATA entry: V34 success before V90 startup complete");
                        g_v90_completion_deferred_logged = true;
                    }
                } else {
                    /* §11.5 clamps 104 for the duration of a retrain and
                       nothing more: the error-control link above the physical
                       layer survives it, so a re-entry after a retrain keeps
                       the data stack rather than restarting LAPM against a
                       peer still in the middle of its own connection. */
                    if (g_retrain_from_data) {
                        g_retrain_from_data = false;
                        g_data_connect_rate = rate;
                        ME_LOG("[ME] V.34 retrain complete; resuming the "
                               "existing data link (%d bps)\n", rate);
                    } else {
                        data_stack_start_online(rate, g_calling_party);
                    }
                    g_state = ME_DATA;
                    g_phase_start_ms = 0;
                    g_v34_data_entry_ms = trace_now_ms();
                    g_v34_data_entry_samples = g_rx_audio_samples;
                    ME_LOG("[ME] V.34 training complete (%d bps)\n", rate);
                    trace_phase("V34 enter DATA: rate=%d", rate);
                    /* §11.5 never takes CONNECT back; a retrain only clamps
                       104 while it runs, so do not re-report it. */
                    if (g_data_framing != DS_FRAMING_V42
                        && !g_data_connect_reported) {
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
    g_v22bis_trained = false;
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
/* V.34 12.2/12.3/12.4 as the RECIPIENT, for the fax probe.  Deliberately a
   separate function rather than a flag through start_v34_training(): that one
   carries the V.90 upstream rate cap, the echo canceller and the notch policy,
   none of which belong to a half-duplex fax call, and threading a duplex flag
   through all of it would put a second meaning on paths this project has
   tuned against three different data peers. */
static void start_v34hdx_training(void)
{
    /* Must be called with g_state_mtx held */
    int bps;

    g_mod   = ME_MOD_V34;
    g_state = ME_TRAINING;
    g_phase_start_ms = trace_now_ms();
    g_training_rx_energy = 0;
    g_training_rx_count  = 0;
    g_training_tx_samples = 0;
    g_last_rx_stage = 0;
    g_last_tx_stage = 0;
    g_v34hdx_fax_control_started = false;
    g_v34hdx_fax_mode = V34_HALF_DUPLEX_CONTROL_CHANNEL;

    if (g_v34) {
        v34_free(g_v34);
        g_v34 = NULL;
    }
    bps = g_v34_start_bps ? g_v34_start_bps : max_v34_bps_for_baud(g_v34_start_baud);
    g_v34 = v34_init(NULL,
                     g_v34_start_baud,
                     bps,
                     g_calling_party,
                     false,         /* half duplex: V.34 clause 12 */
                     v34_get_bit_cb, NULL,
                     v34_put_bit_cb, NULL);
    if (!g_v34) {
        ME_LOG("[ME] V.34 fax probe: v34_init(half duplex) failed\n");
        me_hangup();
        return;
    }
    {
        logging_state_t *log = v34_get_logging_state(g_v34);

        if (log)
            span_log_set_level(log, me_span_flow_level());
    }
    v34_tx_power(g_v34, -10.0f);
    /* 12.2.1 is "call modem as source modem": the calling fax transmits the
       image on the primary channel, so it is the source and we are the
       recipient.  12.2.1.2.6 then has this end send INFOh, and 12.3.2.1 has
       it go silent and wait for the source's S. */
    v34_half_duplex_change_mode(g_v34, V34_HALF_DUPLEX_RECIPIENT);

    /* No echo canceller and no notch.  Both are tuned for the V.90/V.34 data
       paths, both are applied AFTER the RX G.711 tap, and both have been the
       answer to a "the line is bad" investigation in this project twice
       already.  A probe whose entire output is a recording must not have an
       unretired filter in front of it. */
    if (g_echo_can) {
        modem_echo_can_segment_free(g_echo_can);
        g_echo_can = NULL;
    }
    g_notch.active = 0;
    g_v34_use_echo_can = false;
    g_tx_buf_wr = 0;
    g_tx_buf_rd = 0;

    ME_LOG("[ME] V.34 fax probe: half-duplex clause 12 started as RECIPIENT "
           "(%d baud, ceiling %d bps)\n", g_v34_start_baud, bps);
    trace_phase("enter TRAINING: mod=V34HDX role=recipient");
}

static void start_v34_training(void)
{
    /* Must be called with g_state_mtx held */
    /* The V.90 call site sets g_mod before calling in, and this function
     * overwrites it a line below (the caller puts it back).  Read it first:
     * on a V.90 call the V.34 engine carries the UPSTREAM, and its rate
     * ceiling has to be established here rather than at MP.  See the cap
     * below. */
    bool v90_upstream = (g_mod == ME_MOD_V90);

    g_mod   = ME_MOD_V34;
    g_state = ME_TRAINING;
    g_phase_start_ms = trace_now_ms();
    /* g_mod is overwritten to ME_MOD_V34 a line above because V.90's Phases
     * 2-4 ARE V.34's, so an unqualified "mod=V34" here reads as a V.90 call
     * falling back -- which cost one session a wrong conclusion from its own
     * log.  This only covers the DIGITAL role, where g_mod is already V90 on
     * entry; the analogue role sets it after this call and is disambiguated by
     * the "V8 selected V90 analogue role" trace immediately above. */
    trace_phase("enter TRAINING: mod=%s role=%s",
                v90_upstream ? "V90 (via V.34 phases)" : "V34",
                g_calling_party ? "caller" : "answerer");
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

    /* ME_V90_UPSTREAM_MAX_BPS has to bind HERE, not at MP.
     *
     * Capping only the MP upstream-rate mask does not work against the
     * SmartLink rig: measured 2026-08-23, at a 24000 cap it replied to none
     * of our MP -- 22 Phase-4 entries, 14806 MP frames, zero MP' -- while an
     * uncapped control on the same binary reached data mode on attempt 1 with
     * 73 MP'.  The MP frame itself was consistent (bits 24:27 carry the max
     * drn of the same mask that goes into bits 36:48, and the mask is an
     * intersection with the peer's own offered set), which leaves the rate
     * already being fixed by what this V.34 engine was trained for: the call
     * trains "up to 31200 bps" and MP may select within that, not below it.
     *
     * Capping the engine's ceiling makes every later consumer agree by
     * construction -- the INFO1 capability, v34_get_current_bit_rate() which
     * both MP sites derive from, and the receiver's own B1 preparation --
     * instead of the MP mask disagreeing with all of them. */
    if (v90_upstream) {
        int capped = me_v90_upstream_cap(bps);

        if (capped != bps) {
            ME_LOG("[ME] V.90 upstream: V.34 ceiling capped %d -> %d bps at "
                   "Phase 2 (ME_V90_UPSTREAM_MAX_BPS)\n", bps, capped);
            bps = capped;
        }
        /*endif*/
    }
    /*endif*/
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
        span_log_set_level(log, me_span_flow_level());
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
    /* Keep the canceller available for every duplex V.34-family call.  The
       negotiated INFO1 rates may leave too little carrier separation for a
       notch (V.34 §5.3/Table 2); v34_update_echo_policy() chooses it only
       when the final TX/RX parameters require it. */
    g_echo_can = modem_echo_can_segment_init(ECHO_CAN_TAPS);
    if (g_echo_can) {
        modem_echo_can_adaption_mode(g_echo_can, 1);
        ME_LOG("[ME] Echo canceller available for V.34-family training (%d taps = %dms)\n",
                ECHO_CAN_TAPS, ECHO_CAN_TAPS * 1000 / 8000);
    }
    g_tx_buf_wr = 0;
    g_tx_buf_rd = 0;
    g_notch_tx_baud = -1;
    g_notch_tx_high = -1;
    g_notch_rx_baud = -1;
    g_notch_rx_high = -1;
    g_v34_use_echo_can = false;
    /* Unconditional: the echo gate correlates these rings on every V.90 call,
       and stale audio from a previous training attempt would correlate as
       nonsense. */
    memset(g_tx_buf, 0, sizeof(g_tx_buf));
    memset(g_rx_ref_buf, 0, sizeof(g_rx_ref_buf));
    memset(g_rx_raw_buf, 0, sizeof(g_rx_raw_buf));
    g_tx_buf_wr = 0;
    g_tx_buf_rd = 0;
    g_rx_ref_wr = 0;

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
        && me_v90_analogue_role()) {
        /*
         * We offered the analogue role and the peer offered V.90.
         *
         * V.90 puts the analogue modem on the calling side (§9.2 has it send
         * INFO0a and INFO1a), and SpanDSP's Phase 2 keys the analogue INFO
         * variants off calling_party.  An answering call cannot take this role
         * without lying about which INFO it will send, so it falls back.
         */
        if (!g_calling_party) {
            ME_LOG("[ME] V.90 analogue role requires the calling side (§9.2); "
                   "this call is answering — falling back to V.34\n");
            trace_phase("V8 selected V90 analogue role on answer side; using V34");
            g_mod = ME_MOD_V34;
            start_v34_training();
        } else if (!me_v90a_load_dil(&g_v90a_dil)) {
            ME_LOG("[ME] V.90 analogue role: no DIL descriptor — falling back to V.34\n");
            trace_phase("V8 selected V90 analogue role; no DIL, using V34");
            g_mod = ME_MOD_V34;
            start_v34_training();
        } else {
            int saved_baud = g_v34_start_baud;

            g_v90a_dil_valid = true;
            g_v90a_u_info = me_v90a_u_info();
            ME_LOG("[ME] V.8 negotiated V.90 with this end as the ANALOGUE modem "
                   "(U_INFO=%d)\n", g_v90a_u_info);
            trace_phase("V8 selected V90 analogue role");

            /* §6.2: the analogue modem must support 3200; it may select 3000
             * after INFO1d.  Start with the mandatory capability ceiling. */
            g_v34_start_baud = 3200;
            start_v34_training();
            g_v34_start_baud = saved_baud;
            g_mod = ME_MOD_V90;

            /*
             * SpanDSP's V.90 mode plus calling_party is already the analogue
             * side of Phase 2: INFO0a is the plain V.34 INFO0 (Table 8) and
             * info1_baud_init() picks the Table 10 INFO1a that carries U_INFO.
             * Nothing else of its V.90 machinery is wanted — Phase 3 is ours.
             */
            if (g_v34) {
                v34_set_v90_mode(g_v34, (g_law == ME_LAW_ALAW) ? 1 : 0);
                v34_set_v90_u_info(g_v34, g_v90a_u_info);
            }
            g_v90a_started = false;
            g_v90a_complete_logged = false;
            g_v90a_failed_logged = false;
            g_v90a_retrain_logged = false;
            g_v90a_data_start_samples = 0;
            g_v90a_rr_triggered = false;
            g_v90a_rr_deadline_logged = false;
            g_v90a_rx_codewords = 0;

            /* Phase 2 CC carriers are the mirror of the digital role's: §8.2.3.1
               puts the analogue modem's INFO at 2400 Hz and the digital modem's
               at 1200 Hz, so the notch goes on *our* 2400 Hz, not on 1200. */
            notch_filter_init(&g_notch, 2400.0f, 30.0f, 8000.0f);
            ME_LOG("[ME] V.90 analogue: notch at 2400 Hz (our CC TX), RX CC at 1200 Hz\n");
        }
    } else if ((result->jm_cm.modulations & V8_MOD_V90) && g_advertise_v90
        && (result->jm_cm.modulations & V8_MOD_V34)) {
        /*
         * V.90 selected: downstream = PCM codeword injection (up to 56 kbps),
         * upstream = V.34 modulation.  Training uses V.34 Phases 2-4; after
         * training completes, TX switches from V.34 to direct PCM injection.
         */
        /* V.8 QC only says V.92 is available.  The actual data-pump choice
         * follows the INFO0d/INFO0a capability exchange in Phase 2. */
        /* Some V.8 peers, including d-modem, complete the CM/JM exchange
           before SpanDSP delivers their trailing QC/QCA V.92 octet to this
           callback.  The negotiated PCM-availability field is already
           authoritative at this point: its analogue bit means the peer
           offers the V.90/V.92 analogue role.  INFO0 remains the actual
           V.92 selector below, so this merely enables our INFO0d bit 27. */
        g_v92_v8_offered = (result->v92 >= 0
                          || (result->jm_cm.pcm_modem_availability
                              & V8_PSTN_PCM_MODEM_V90_V92_ANALOGUE) != 0)
                        && g_enable_v92;
        g_v92_info0_local_advertised = g_v92_v8_offered;
        g_v92_info0_peer_capable = false;
        g_v92_info0_peer_short_phase2 = false;
        g_v92_info0_mutual = false;
        g_v92_info0_peer_logged = false;
        g_v92_active = false;
        if ((result->v92 >= 0
             || (result->jm_cm.pcm_modem_availability
                 & V8_PSTN_PCM_MODEM_V90_V92_ANALOGUE) != 0)
            && !g_v92_v8_offered)
            ME_LOG("[ME] V.92 capability present, but modem mode is %s; using V.90 Phase 4\n",
                   g_mode_name);
        ME_LOG("[ME] V.8 negotiated V.90 PCM downstream + V.34 upstream%s\n",
               g_v92_v8_offered ? "; V.92 pending INFO0 confirmation" : "");
        trace_phase("V8 selected V90%s", g_v92_v8_offered ? "; V92 INFO0 pending" : "");
        g_mod = ME_MOD_V90;
        /* V.90 §6.2: the digital modem must receive both 3000 and 3200;
         * initialize at 3200 so INFO1d can enable both rows. */
        int saved_baud = g_v34_start_baud;
        g_v34_start_baud = 3200;
        start_v34_training();
        g_v34_start_baud = saved_baud;
        /* start_v34_training sets g_mod = ME_MOD_V34; override back to V90 */
        g_mod = ME_MOD_V90;
        v90_cp_live_capture_reset_locked();
        /* Enable V.90 INFO0d frame generation and carrier swap in SpanDSP V.34.
           v34_set_v90_mode also updates CC carrier frequencies (§8.2.3.1). */
        if (g_v34) {
            v34_set_v90_mode(g_v34, (g_law == ME_LAW_ALAW) ? 1 : 0);
            v34_set_v92_info0_capabilities(g_v34,
                                            g_v92_info0_local_advertised ? 1 : 0,
                                            0);
            v34_set_v92_pcm_upstream_capability(g_v34,
                                                v92_pcm_upstream_advertised() ? 1 : 0);
        }
        v90_cp_rx_init(&g_v90_cp_rx,
                       4,
                       g_law == ME_LAW_ALAW,
                       v90_live_cp_frame,
                       NULL);
        v90_reset_upstream_data_arming();
        if (g_v34)
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

    } else if (result->jm_cm.modulations & V8_MOD_V34HDX) {
        /* V.34 12.4 half-duplex, i.e. a T.30 Annex F fax.  Only reachable
           with ME_V34_FAX_PROBE set, because that is the only way
           V8_MOD_V34HDX gets into our JM offer in the first place. */
        ME_LOG("[ME] V.8 negotiated V.34 HALF-DUPLEX (clause 12 / T.30 Annex F), "
               "peer call function=%s\n",
               v8_call_function_to_str(result->jm_cm.call_function));
        trace_phase("V8 selected V34 half-duplex");
        start_v34hdx_training();

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
    /* Report the arithmetic libspandsp was built with, not the flag this
       object saw: an A/B of the two datapaths has to be able to confirm from
       the log which arm ran, and a -D in CFLAGS can leave the library and the
       application disagreeing without any other symptom. */
    fprintf(stderr, "[ME] V.34 datapath: %s\n",
            v34_datapath_is_fixed_point() ? "fixed point" : "floating point");
    v90_cp_live_worker_start();
    {
        const char *mode = getenv("ME_MODE");
        const char *role = getenv("ME_V90_ROLE");

        /* ME_V92_ENABLE remains a compatibility alias when ME_MODE is absent. */
        if (!mode || !*mode || strcmp(mode, "auto") == 0) {
            g_advertise_v90 = true;
            g_enable_v92 = parse_env_int("ME_V92_ENABLE", 0) != 0;
            g_mode_name = g_enable_v92 ? "v92" : "v90";
        } else if (strcmp(mode, "v34") == 0) {
            g_advertise_v90 = false;
            g_enable_v92 = false;
            g_mode_name = "v34";
        } else if (strcmp(mode, "v22") == 0) {
            g_advertise_v90 = false;
            g_advertise_v34 = false;
            g_enable_v92 = false;
            g_mode_name = "v22";
        } else if (strcmp(mode, "v90") == 0) {
            g_advertise_v90 = true;
            g_enable_v92 = false;
            g_mode_name = "v90";
        } else if (strcmp(mode, "v92") == 0) {
            g_advertise_v90 = true;
            g_enable_v92 = true;
            g_mode_name = "v92";
        } else {
            ME_LOG("[ME] Unknown ME_MODE '%s'; using v90\n", mode);
            g_advertise_v90 = true;
            g_enable_v92 = false;
            g_mode_name = "v90";
        }

        g_v90_analogue_role = g_advertise_v90
                           && role && strcmp(role, "analogue") == 0;
        ME_LOG("[ME] Modem mode: %s (V.8 offer %s)\n", g_mode_name,
               g_advertise_v90 ? "V90|V34|V22"
                               : (g_advertise_v34 ? "V34|V22" : "V22"));
        if (!g_advertise_v90 && role && strcmp(role, "analogue") == 0)
            ME_LOG("[ME] ME_V90_ROLE=analogue ignored in v34 mode\n");
        if (g_v90_analogue_role)
            ME_LOG("[ME] V.90 role: ANALOGUE (opt-in; Phase 4 B1/B1d and "
                   "bidirectional data mappers enabled)\n");
    }
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
    g_v34_rx_samples = 0;
    g_v90_t3_arm_logged = false;
    g_v34_rx_started_at = 0;
    g_rx_audio_samples = 0;
    g_rx_audio_started_at = 0;
    g_v34_rx_accounting_logged = false;
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
                                                MODEM_CONNECT_TONES_ANSAM_PR);
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
    /* Every knob this build reads is an environment variable, and a capture
       that does not record which ones were set cannot be replayed against.
       The live/replay work of 2026-08-24 lost time to exactly that: the
       server's own log named one knob, in passing, in a line about a rate
       cap.  Print the whole set once, at startup, into the same log the
       capture keeps. */
    {
        extern char **environ;
        int n = 0;

        for (char **e = environ;  e && *e;  e++) {
            if (strncmp(*e, "ME_", 3) == 0
                || strncmp(*e, "V34_", 4) == 0
                || strncmp(*e, "VPCM_", 5) == 0
                || strncmp(*e, "SIP_", 4) == 0
                || strncmp(*e, "SPANDSP_", 8) == 0) {
                ME_LOG("[ME] env: %s\n", *e);
                n++;
            }
        }
        if (n == 0)
            ME_LOG("[ME] env: no ME_/V34_/VPCM_/SIP_/SPANDSP_ knobs set\n");
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
    /* V.90 §9.7 requires a data-mode disconnect to send drn=0 in a rate
     * sequence.  Keep SIP up until one complete CP has gone on the wire; a
     * second request, or a call outside stable analogue V.90 data, is hard. */
    if (!g_v90a_cleardown && g_v90a && g_state == ME_DATA
        && v90_analogue_phase3_data_ready(g_v90a)
        && v90_analogue_phase3_start_cleardown(g_v90a)) {
        g_v90a_cleardown = true;
        ME_LOG("[ME] V.90 analogue: initiating §9.7 cleardown (CP drn=0)\n");
        trace_phase("V90a cleardown initiated");
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }
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
    g_v92_v8_offered = false;
    g_v92_info0_local_advertised = false;
    g_v92_info0_peer_capable = false;
    g_v92_info0_peer_short_phase2 = false;
    g_v92_info0_mutual = false;
    v92_p3_rx_init(&g_v92_p3_rx);
    v92_cp_rx_reset(&g_v92_p3_cpt_rx);
    memset(&g_v92_p3_cpt_demod, 0, sizeof(g_v92_p3_cpt_demod));
    g_v92_p3_rx_active = false;
    g_v92_p3_rx_result_applied = false;
    g_v92_p3_rx_failure_logged = false;
    g_v92_p3_cpt_active = false;
    v92_su_rx_reset_locked();
    g_v92_trn2u_active = false;
    g_v92_upstream_rx_active = false;
    g_v92_upstream_lock_logged = false;
    g_data_link_failed = false;
    g_data_connect_reported = false;

    /* Outgoing dial = caller role; incoming auto-answer = answerer role. */
    g_calling_party = (g_state == ME_DIALING);
    if (g_invert_v34_role)
        g_calling_party = !g_calling_party;
    trace_phase("SIP media connected: role=%s", g_calling_party ? "caller" : "answerer");

    /* A selected legacy fax service class owns the bearer from this point.  T.31
     * Class 1 drives T.30 from the DTE and Class 2.0 drives it in
     * fax_class2.c; in either case V.8/V.34 negotiation is a data-modem
     * startup sequence and would overwrite the fax tones. */
    if (di_fax_active() && !me_v34_fax_probe()) {
        g_state = ME_DATA;
        g_mod = ME_MOD_NONE;
        g_phase_start_ms = 0;
        pthread_mutex_unlock(&g_state_mtx);
        trace_phase("SIP media connected: fax service class owns audio");
        ME_LOG("[ME] SIP connected as %s, handing audio to fax service class\n",
               g_calling_party ? "caller" : "answerer");
        di_on_connected(0);
        return;
    }

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
    trace_phase("enter V8: mode=%s advertised mods=%s", g_mode_name,
                g_advertise_v90 ? "V90|V34|V22" : "V34|V22");

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

/* ---- V.90 Phase 3 far-end-S echo gate -------------------------------------
 *
 * The rotation branch of the S detector (v34rx.c, phase3_s_dom_windows) fires
 * on ANY sustained single-frequency tone, because a pure tone differentially
 * demodulates to a constant dibit.  Our own Phase 3/4 downstream is exactly
 * that: DIL is tonal per segment and Ri is a dead-flat 8000/6 = 1333 Hz
 * (§8.6.4 signal R at U_INFO).  Leaked back through the SIP/FXS/ATA hybrid it
 * reads as a textbook far-end S.
 *
 * Measured on artifacts/v90-hardware/20260724T094048Z-courier-uscan-21210-
 * firegate, over the stuck-Ri window t=14..16 s: RX was 1333.5 Hz at -29.4 dB
 * relative to our TX, cross-correlating 0.868 with our own TX at 2 samples
 * lag, while the Courier transmitted nothing at all.  Both "far-end S" events
 * in that call were that echo.  The first one truncated Jd to 135 ms, so we
 * ran the whole Phase 3 tail ~4.6 s ahead of the Courier's §9.3.2.7 schedule
 * and it retrained to Tone A at its deadline.
 *
 * Raw power does NOT separate the two cases (see the note at v34rx.c:5915),
 * but provenance does: the echo is a coherent copy of our own transmitter, and
 * a real far-end S is not.  So gate on normalised cross-correlation against
 * the TX reference ring rather than on level.  Discrimination is wide — echo
 * ~0.87 vs an expected ~0.04 when the far end is actually transmitting (its
 * signal is ~29 dB above the echo, so it dominates the correlation).
 *
 * Runs with g_state_mtx held.  Returns [0,1], or -1.0 when there is not enough
 * signal on either side to have an opinion (caller then accepts the event, so
 * an unusable reference fails towards the old behaviour rather than deadlocking
 * Phase 3).
 */
enum {
    S_ECHO_WIN     = 800,   /* 100 ms correlation window */
    S_ECHO_LAG_MIN = -320,  /* -40 ms: TX ring may trail RX by a frame or two */
    S_ECHO_LAG_MAX = 640,   /*  80 ms: covers SIP + ATA hybrid round trip */
    S_ECHO_COARSE  = 4,     /* coarse lag step, then refined +/- S_ECHO_COARSE */
};

/* Absolute signal-presence floor, complementary to the correlation gate above.
 *
 * The correlation gate only catches an S that is OUR OWN transmitter coming
 * back.  On a low-echo path there is nothing to correlate against, and the same
 * rotation detector instead fires on a carrier-offset rotation of near-silent
 * noise — §9.3.2.9 explicitly permits the analogue modem to send *silence*
 * throughout DIL, so this is a state we are guaranteed to sit in on every call.
 *
 * Measured over the Cisco FXS path
 * (artifacts/v90-hardware/20260725T015104Z-courier-echogate-novad): far-end RX
 * is RMS **8** through both DIL cycles while the Courier is silent, versus
 * **625–634** during TRN1d/Jd when it is genuinely transmitting — ~38 dB of
 * separation.  On the AudioCodes path a real far-end signal ran ~3671.  The
 * default floor of 64 sits ~18 dB above observed silence and ~20 dB below the
 * weakest real signal seen.
 *
 * Deliberately kept as a SEPARATE test from the correlation gate rather than
 * replacing it: on the AudioCodes path our own echo measured RMS ~132, which is
 * comfortably above any sane floor, so energy alone would not have caught it
 * (this is the trap the note at v34rx.c:5915 describes, and why the earlier
 * `info_rx_carrier_ref/64` attempt regressed).  Echo needs provenance; silence
 * needs level.  Both, or neither works.
 */
static double me_v90_s_min_rx_rms(void)
{
    static double cached = -1.0;

    if (cached < 0.0) {
        const char *v = getenv("ME_V90_S_MIN_RX_RMS");

        cached = 64.0;
        if (v && *v) {
            char *end;
            double parsed = strtod(v, &end);

            /* 0 disables the floor. */
            if (end != v && *end == '\0' && parsed >= 0.0)
                cached = parsed;
        }
    }
    return cached;
}

/* RMS of the same window the correlation gate scores. */
static double v90_s_rx_rms_locked(void)
{
    double energy = 0.0;

    for (int k = 0; k < S_ECHO_WIN; k++) {
        double r = (double)g_rx_ref_buf[(g_rx_ref_wr - 1 - k) & TX_BUF_MASK];

        energy += r * r;
    }
    return sqrt(energy / (double)S_ECHO_WIN);
}

/* Fraction of 10 ms sub-windows in the last DIL_S_ACTIVE_WIN samples whose RMS
 * clears the signal floor.
 *
 * A single 100 ms RMS reading is not enough to qualify the DIL-terminating S.
 * §9.3.2.10 has the analogue modem send S for 128T then S̄ for 16T — at 3200
 * baud that is only ~45 ms — but for it to mean anything the far end must
 * actually be transmitting *around* that point, not producing one isolated
 * window of energy in an otherwise dead line.  Live against the Courier
 * (trn2500-1) the far end sat at RMS 8 continuously from 13.6 s to 16.2 s, i.e.
 * across the whole of DIL and our Ri, yet a single-window reading still let a
 * DIL-terminating S through and we left DIL ~2.9 s before its §9.3.2.10
 * deadline.  Requiring the line to be live across a majority of a 300 ms span
 * rejects that without needing a precise threshold on any one window. */
enum { DIL_S_ACTIVE_WIN = 2400, DIL_S_SUB = 80 };   /* 300 ms in 10 ms steps */

static double v90_s_rx_active_fraction_locked(double floor_rms)
{
    int active = 0;
    int subs = DIL_S_ACTIVE_WIN / DIL_S_SUB;

    for (int w = 0; w < subs; w++) {
        double energy = 0.0;

        for (int k = 0; k < DIL_S_SUB; k++) {
            int idx = (g_rx_ref_wr - 1 - (w * DIL_S_SUB + k)) & TX_BUF_MASK;
            double r = (double)g_rx_ref_buf[idx];

            energy += r * r;
        }
        if (sqrt(energy / (double)DIL_S_SUB) >= floor_rms)
            active++;
    }
    return (double)active / (double)subs;
}

/* Fraction of the S window's energy that sits in the V.34 Tone A band.
 *
 * The third false-S source, and the only one the other three gates cannot
 * reach.  Tone A (§11.2.1.2 / §9.5.2.1, 2400 Hz) is a real, loud, far-end
 * signal, so the silence floor, the RX/TX ratio test and the echo correlator
 * all correctly pass it: it is genuinely not our echo and genuinely not a dead
 * line.  A pure tone differentially demodulates to a constant dibit, which the
 * rotation detector reads as a sustained S -- and p3_demod's structural check
 * lets it through too, because a constant dibit stream *is* 6-symbol periodic.
 *
 * Measured on the Courier capture 20260812T073702Z (µ-law, 100 ms windows,
 * Hann): the retrain tone holds 0.79-0.80 of its energy in 2300-2500 Hz for
 * 32 s, while every window in which the Courier was really transmitting
 * (Ja, PP/TRN) scores 0.03-0.09.  The separation is an order of magnitude, and
 * it is structural rather than calibrated: the real far-end S is V.34-modulated
 * around the 1829/1920 Hz carriers and cannot concentrate its energy in a
 * 200 Hz band at the top of the passband.
 *
 * Bins are evaluated at exact DFT frequencies k*fs/N so Parseval applies and
 * the ratio is comparable against the window's total energy.  Costs 21
 * Goertzel passes over 800 samples, only on an S event that has already
 * cleared every other gate. */
enum { TONE_A_LO_HZ = 2300, TONE_A_HI_HZ = 2500, TONE_A_FS = 8000 };

static double v90_s_tone_a_fraction_locked(void)
{
    double total = 0.0;
    double band = 0.0;
    int klo = (TONE_A_LO_HZ * S_ECHO_WIN) / TONE_A_FS;
    int khi = (TONE_A_HI_HZ * S_ECHO_WIN) / TONE_A_FS;

    /* Hann-window the same 100 ms the other gates score, so leakage from the
     * strong out-of-band content of a real S does not inflate the ratio. */
    for (int n = 0; n < S_ECHO_WIN; n++) {
        int idx = (g_rx_ref_wr - S_ECHO_WIN + n) & TX_BUF_MASK;
        double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)n / (double)(S_ECHO_WIN - 1));
        double x = (double)g_rx_ref_buf[idx] * w;

        total += x * x;
    }
    if (total <= 0.0)
        return 0.0;

    for (int k = klo; k <= khi; k++) {
        double omega = 2.0 * M_PI * (double)k / (double)S_ECHO_WIN;
        double coeff = 2.0 * cos(omega);
        double s0 = 0.0, s1 = 0.0, s2 = 0.0;

        for (int n = 0; n < S_ECHO_WIN; n++) {
            int idx = (g_rx_ref_wr - S_ECHO_WIN + n) & TX_BUF_MASK;
            double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)n / (double)(S_ECHO_WIN - 1));

            s0 = (double)g_rx_ref_buf[idx] * w + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        /* |X_k|^2 from the Goertzel recurrence, doubled for the mirrored
         * negative-frequency bin so the sum is comparable with total energy. */
        band += 2.0 * (s1 * s1 + s2 * s2 - coeff * s1 * s2);
    }
    return band / (total * (double)S_ECHO_WIN);
}

static double me_v90_s_tone_a_max_fraction(void)
{
    static double cached = -1.0;

    if (cached < 0.0) {
        const char *v = getenv("ME_V90_S_TONE_A_MAX_FRACTION");

        /* Midway (in log terms) between the measured 0.09 real-signal ceiling
         * and the 0.79 Tone A floor.  0 disables the gate. */
        cached = 0.35;
        if (v && *v) {
            char *end;
            double parsed = strtod(v, &end);

            if (end != v && *end == '\0' && parsed >= 0.0 && parsed <= 1.0)
                cached = parsed;
        }
    }
    return cached;
}

/* RMS of our OWN transmit over the same window, for the RX/TX ratio test. */
static double v90_s_tx_rms_locked(void)
{
    double energy = 0.0;

    for (int k = 0; k < S_ECHO_WIN; k++) {
        double t = (double)g_tx_buf[(g_tx_buf_wr - 1 - k) & TX_BUF_MASK];

        energy += t * t;
    }
    return sqrt(energy / (double)S_ECHO_WIN);
}

/* Minimum RX/TX level ratio for an S to be believable.
 *
 * This is the discriminator that actually separates the cases, and unlike an
 * absolute floor it is independent of units — both sides are measured from the
 * same rings, so any scaling cancels.  That matters: the gate's RMS readings and
 * the G.711 tap disagree by several times because they are different timelines
 * and processing points, which made an absolute threshold impossible to
 * calibrate confidently.
 *
 * Measured on the Cisco path with the FXS echo canceller disabled
 * (`*gate25-*` runs), RX RMS as a fraction of our own TX RMS:
 *   genuine far-end signal (the Courier's Ja):   0.27 – 0.53
 *   our echo + line noise while it is silent:    0.026 – 0.042
 * A 10x gap.  0.15 sits in the middle of it.
 *
 * This is what catches the failure the correlation gate could not: at the
 * moment we accepted a "far-end S" 108 Jd symbols in, the Courier had been
 * silent since it saw our Sd→S̄d, but the window was mostly line *noise* rather
 * than a clean copy of our transmitter, so it correlated at only 0.129 and
 * passed.  Acting on it made us send J'd ~5 s early; per §9.3.2.8 the analogue
 * modem must detect J'd before it will receive DIL at all, so it never entered
 * the DIL-reception state and ignored the whole sequence.
 *
 * When we are not transmitting, tx_rms is ~0 and the test trivially passes —
 * correct, since there is no echo to mistake for signal. */
static double me_v90_s_min_rx_tx_ratio(void)
{
    static double cached = -1.0;

    if (cached < 0.0) {
        const char *v = getenv("ME_V90_S_MIN_RX_TX_RATIO");

        cached = 0.15;
        if (v && *v) {
            char *end;
            double parsed = strtod(v, &end);

            /* 0 disables the test. */
            if (end != v && *end == '\0' && parsed >= 0.0 && parsed <= 10.0)
                cached = parsed;
        }
    }
    return cached;
}

static double me_v90_dil_s_active_fraction(void)
{
    static double cached = -1.0;

    if (cached < 0.0) {
        const char *v = getenv("ME_V90_DIL_S_ACTIVE_FRACTION");

        cached = 0.5;
        if (v && *v) {
            char *end;
            double parsed = strtod(v, &end);

            /* 0 disables the requirement. */
            if (end != v && *end == '\0' && parsed >= 0.0 && parsed <= 1.0)
                cached = parsed;
        }
    }
    return cached;
}

static int me_v90_s_echo_gate_pct(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("ME_V90_S_ECHO_GATE_PCT");

        cached = 50;    /* reject an S whose window is >50% our own TX */
        if (v && *v) {
            char *end;
            long parsed = strtol(v, &end, 10);

            /* 0 disables the gate entirely. */
            if (end != v && *end == '\0' && parsed >= 0 && parsed <= 100)
                cached = (int)parsed;
        }
    }
    return cached;
}

/* Sum of rx[k]*tx[k+lag] over the window, plus the TX energy at that lag.
   Index 0 is the newest sample in each ring. */
static double v90_s_echo_corr_at_locked(int lag, double rx_energy)
{
    double num = 0.0, tx_energy = 0.0;

    for (int k = 0; k < S_ECHO_WIN; k++) {
        int ri = (g_rx_ref_wr - 1 - k) & TX_BUF_MASK;
        int ti = (g_tx_buf_wr - 1 - k - lag) & TX_BUF_MASK;
        double r = (double)g_rx_ref_buf[ri];
        double t = (double)g_tx_buf[ti];

        num += r * t;
        tx_energy += t * t;
    }
    if (tx_energy < 1.0)
        return 0.0;
    return fabs(num) / sqrt(rx_energy * tx_energy);
}

static double v90_s_echo_correlation_locked(void)
{
    double rx_energy = 0.0;
    double best = 0.0;
    int best_lag = 0;

    for (int k = 0; k < S_ECHO_WIN; k++) {
        double r = (double)g_rx_ref_buf[(g_rx_ref_wr - 1 - k) & TX_BUF_MASK];

        rx_energy += r * r;
    }
    /* < ~1 LSB RMS on either side: no usable reference, express no opinion. */
    if (rx_energy < (double)S_ECHO_WIN)
        return -1.0;

    for (int lag = S_ECHO_LAG_MIN; lag <= S_ECHO_LAG_MAX; lag += S_ECHO_COARSE) {
        double c = v90_s_echo_corr_at_locked(lag, rx_energy);

        if (c > best) {
            best = c;
            best_lag = lag;
        }
    }
    for (int lag = best_lag - S_ECHO_COARSE; lag <= best_lag + S_ECHO_COARSE; lag++) {
        double c = v90_s_echo_corr_at_locked(lag, rx_energy);

        if (c > best)
            best = c;
    }
    return best;
}

static void v90_s_echo_record_rx_locked(const int16_t *filtered,
                                         const int16_t *raw,
                                         int len)
{
    if (!filtered || !raw || len <= 0)
        return;
    for (int i = 0; i < len; i++) {
        g_rx_ref_buf[g_rx_ref_wr] = filtered[i];
        g_rx_raw_buf[g_rx_ref_wr] = raw[i];
        g_rx_ref_wr = (g_rx_ref_wr + 1) & TX_BUF_MASK;
    }
    /* Monotonic count of RX samples that reached the V.34/V.90 receiver.  Logged
     * with every S event so gate readings can be aligned against the live-rx
     * G.711 tap: the two timelines are NOT the same, because the tap records
     * from call answer while this ring only fills once the V.34 branch runs.
     * Comparing a [TRACE +Nms] stamp directly against a tap offset produced
     * contradictory RMS readings during the 2026-07-25 Courier analysis. */
    g_rx_ref_samples += (uint64_t)len;
}

/* Structural cross-check for the live V.34 Phase-3 S/J detectors.
 *
 * The SpanDSP V.34 receiver (spandsp/src/v34rx.c) fires V34_EVENT_S on a
 * sustained-rotation / alternation heuristic and V34_EVENT_J on 8 sustained
 * near-perfect canonical windows.  Both false-positive on near-silent noise and
 * DIL turnaround: a carrier-offset rotation of noise reads as a sustained S
 * (the AGC normalises magnitude so only raw power discriminates), and that
 * same false S during DIL cuts the descriptor short before the far end has
 * converged on it (observed live against the USR Courier: DIL terminated at
 * 0.25-0.5 of a cycle on a faulty S).
 *
 * p3_demod classifies the actual 6-symbol repeating structure (3+ unique
 * dibits, >=80% periodic match for S; 16-bit repeating pattern for J) which
 * noise and silence cannot produce.  Running it on a recent RX window and
 * requiring it to agree turns the event gate from a raw-threshold detector
 * into a confidence-gated one without weakening the strict path.
 *
 * Returns true when the signal is confirmed (or when confirmation is
 * disabled / p3_demod cannot run, so the gate never blocks a real event on a
 * p3_demod defect).  Returns false only when p3_demod successfully ran and
 * did NOT find the expected pattern — the case that rejects a false S. */
enum { P3_CONFIRM_SAMPLES = 1600 };   /* 200 ms at 8 kHz */

static bool v90_p3_confirm_signal_locked(p3_signal_type_t want)
{
    static int disabled = -1;
    int16_t window[P3_CONFIRM_SAMPLES];
    int baud_code;
    int n;
    bool found = false;
    bool ran = false;

    if (disabled < 0)
        disabled = getenv("ME_V90_P3_CONFIRM") &&
                   !strcmp(getenv("ME_V90_P3_CONFIRM"), "0") ? 1 : 0;
    if (disabled)
        return true;
    /* The J gate can be independently disabled: the SmartLink interop rig
     * uses ME_V90_J_LOOKAHEAD_BITS to fire a synthetic J early (before the
     * strict canonical path would confirm it), and p3_demod may not have a
     * clear enough window at that point.  The S gate is the one that
     * directly fixes the false-S DIL cutoff; the J detector is already very
     * strict. */
    if (want == P3_SIGNAL_J
        && getenv("ME_V90_P3_CONFIRM_J")
        && !strcmp(getenv("ME_V90_P3_CONFIRM_J"), "0"))
        return true;
    if (!g_v34)
        return true;

    baud_code = v34_get_rx_baud_rate(g_v34);
    if (baud_code < P3_BAUD_2400 || baud_code >= P3_BAUD_COUNT)
        baud_code = P3_BAUD_2400;

    /* Copy the most recent samples from the RX ring buffer. */
    n = P3_CONFIRM_SAMPLES;
    if (n > TX_BUF_SIZE)
        n = TX_BUF_SIZE;
    for (int i = 0; i < n; i++)
        window[i] = g_rx_ref_buf[(g_rx_ref_wr - n + i) & TX_BUF_MASK];

    /* V.90 §8.2.3.2 Tables 9/10 make the carrier an INFO1d per-rate choice;
     * it is not necessarily the ordinary V.34 caller/high default.  Try the
     * negotiated carrier first and the alternate as a robustness fallback,
     * matching the primary Ja scanner.  Use the PP-trained variant: the fast
     * equalizer (SPRA159 §3.2.3) converges where blind CMA fails. */
    for (int attempt = 0; attempt < 2 && !found; attempt++) {
        int preferred = v34_get_rx_high_carrier(g_v34)
                      ? P3_CARRIER_HIGH : P3_CARRIER_LOW;
        int carrier = attempt == 0 ? preferred : !preferred;
        p3_result_t *result = p3_demod_run_pp_trained(
            window, n, 0, baud_code, carrier, 8000);

        if (!result)
            continue;
        ran = true;
        for (int i = 0; i < result->segment_count; i++) {
            p3_signal_type_t type = result->segments[i].type;
            if (want == P3_SIGNAL_S) {
                /* Accept either polarity: v34rx reports S and S-bar through
                 * the same event. */
                if (type == P3_SIGNAL_S || type == P3_SIGNAL_S_BAR) {
                    found = true;
                    break;
                }
            } else if (type == want) {
                found = true;
                break;
            }
        }
        p3_result_free(result);
    }
    return ran ? found : true;  /* A p3_demod defect must not block a real event. */
}

/* Primary Ja detector: structurally identifies the 16-bit repeating J
 * pattern in the live RX stream using p3_demod and fires V90_RX_EVENT_J
 * directly, bypassing the slow v34rx canonical path (1.25 s for 8
 * sustained windows) and the ME_V90_J_LOOKAHEAD_BITS fixed bit-count
 * shortcut (625 ms).
 *
 * p3_demod classifies the J pattern from the actual 16-bit Table 18
 * structure — scrambled TRN cannot produce a sustained canonical J match —
 * so detection is both faster and more reliable than threshold counting.
 * At 2400 baud the J period is 8 symbols (3.3 ms); two periods fit in
 * ~7 ms, so a 100 ms scan window has ample data.
 *
 * Throttled to every 80 ms.  The 800 ms history lets a weak foreign
 * convention trade Table-18 score for sustained periodicity; a clean J still
 * takes the immediate short-evidence path.  Both remain within WaitForSd.
 * With this as the primary detector, ME_V90_SD_DELAY_MS and
 * ME_V90_SD_DELAY_RETRAIN_MS can both default to 0 (§9.3.1.3 allows
 * 0–500 ms; 0 is valid when Ja is detected structurally rather than
 * by bit count).
 *
 * Once J fires, the TX phase moves past V90_TX_WAIT_JA and this scanner
 * becomes a no-op.  On retrain resync back to WAIT_JA, scanning resumes
 * automatically (the sample counter resets when the phase changes).
 *
 * Disabled by ME_V90_P3_CONFIRM=0 (same kill switch as the S/J
 * confirmation gates). */
enum { P3_JA_SCAN_THROTTLE = 640 };  /* 80 ms at 8 kHz */
enum { P3_JA_SCAN_WINDOW  = 6400 };  /* 800 ms: retain a full weak foreign-J run */
enum { P3_JA_SCAN_LOG_INTERVAL = 2400 };  /* log every 300 ms */

static int g_v90_p3_ja_scan_samples = 0;
static int g_v90_p3_ja_scan_total = 0;   /* total samples scanned since WAIT_JA */
static bool g_v90_p3_ja_fired = false;

static void v90_p3_scan_ja_locked(int len)
{
    int16_t window[P3_JA_SCAN_WINDOW];
    int baud_code;
    int n;
    p3_result_t *result;

    if (!g_v90 || g_v92_active || !g_v34)
        return;
    /* Only scan while waiting for Ja.  Once J fires, the phase moves to
     * SD_DELAY/SD and this function becomes a no-op.  On retrain resync
     * back to WAIT_JA, scanning resumes.  The counter reset on phase
     * change ensures we don't carry stale throttle state across retrains. */
    if (v90_get_tx_phase(g_v90) != V90_TX_WAIT_JA) {
        g_v90_p3_ja_scan_samples = 0;
        g_v90_p3_ja_scan_total = 0;
        g_v90_p3_ja_fired = false;
        return;
    }
    g_v90_p3_ja_scan_samples += len;
    g_v90_p3_ja_scan_total += len;
    if (g_v90_p3_ja_scan_samples < P3_JA_SCAN_THROTTLE)
        return;
    g_v90_p3_ja_scan_samples = 0;

    /* Reuse the confirmation gate's env kill switch. */
    {
        static int disabled = -1;

        if (disabled < 0)
            disabled = getenv("ME_V90_P3_CONFIRM") &&
                       !strcmp(getenv("ME_V90_P3_CONFIRM"), "0") ? 1 : 0;
        if (disabled)
            return;
    }

    baud_code = v90_selected_upstream_baud_locked();
    if (baud_code != P3_BAUD_3000 && baud_code != P3_BAUD_3200)
        baud_code = P3_BAUD_3200;

    n = P3_JA_SCAN_WINDOW;
    if (n > TX_BUF_SIZE)
        n = TX_BUF_SIZE;
    /* Grade Ja on the pre-echo-canceller observation.  With INFO1d selecting
     * 3200-low, SmartLink's raw 1829 Hz Ja remains 94% periodic while the
     * ordinary V.34 echo front end erases it.  Payload RX remains filtered;
     * only this bounded acquisition decision uses the raw companion ring. */
    for (int i = 0; i < n; i++)
        window[i] = g_rx_raw_buf[(g_rx_ref_wr - n + i) & TX_BUF_MASK];

    /* V.90 §6.2 limits this upstream to 3000/3200 (3429 is optional and is
     * not offered by the current profile).  Try INFO1a's selected rate first
     * and the other required digital-modem rate as fallback.  This bounds the
     * 800 ms grader to four rate/carrier passes per 80 ms tick. */
    {
        int try_bauds[2];
        int n_try = 0;

        try_bauds[n_try++] = baud_code;
        try_bauds[n_try++] = baud_code == P3_BAUD_3000
                           ? P3_BAUD_3200 : P3_BAUD_3000;

        for (int bi = 0; bi < n_try; bi++) {
            int b = try_bauds[bi];

            for (int ci = 0; ci <= 1; ci++) {
                int carrier = ci ? P3_CARRIER_LOW : P3_CARRIER_HIGH;

                result = p3_demod_run_pp_trained(window, n, 0, b, carrier, 8000);
                if (!result)
                    continue;

                for (int i = 0; i < result->segment_count; i++) {
                    const p3_segment_t *seg = &result->segments[i];

                    if (seg->type == P3_SIGNAL_J
                        && seg->end_sample >= n - P3_JA_SCAN_THROTTLE
                        && p3_is_adaptive_ja_candidate(
                               seg, (int)(result->baud_rate_estimate + 0.5f))) {
                        bool accepted;
                        int start_sample = seg->start_sample;
                        int end_sample = seg->end_sample;
                        int length = seg->length;
                        int match_pct = seg->j_table_match_pct;
                        int periodic_pct = seg->j_periodic_match_pct;

                        /* The rolling analysis window overlaps PP/TRN and old
                         * Ja on every invocation.  Accepting a J segment
                         * anywhere in it can fire on stale training hundreds
                         * of milliseconds before the peer enters WaitForSd;
                         * SmartLink then reports zero Sd energy and retrains.
                         * Require J evidence in the newest 80 ms, matching the
                         * scan cadence.  p3_is_adaptive_ja_candidate() accepts
                         * either a strong short Table 18 match or a weaker run
                         * only after duration and periodicity independently
                         * prove sustained J.  Live false TRN matches scored
                         * 69-71% but lasted only 48 symbols; the foreign
                         * 3200-low Ja scores 63% yet persists for 600+ ms. */
                        p3_result_free(result);
                        /* Fire J event directly.  The post-processing (DIL
                         * capture, S-detector arming) mirrors the v34rx J
                         * event handler so the downstream state is identical
                         * regardless of which detector fired first. */
                        g_v90_p3_ja_fired = true;
                        /* Keep decoding Ja either way -- it is the descriptor
                           parse that is now allowed to fire J. */
                        (void)v90_dil_capture_try_v34_hypotheses();
                        accepted = v90_ja_heuristic_allowed("p3_demod")
                                 ? v90_handle_rx_event(g_v90, V90_RX_EVENT_J)
                                 : false;
                        if (accepted) {
                            (void)v90_dil_capture_try_v34_hypotheses();
                            v34_v90_arm_phase3_s_detector(g_v34);
                        }
                        fprintf(stderr,
                                "[ME] V.90 p3_demod: current J detected (adaptive structural, %d ms window, "
                                "segment=%d..%d samples/%d symbols table=%d%% periodic=%d%%, "
                                "baud=%d carrier=%s); accepted=%d\n",
                                n * 1000 / 8000, start_sample, end_sample, length,
                                match_pct, periodic_pct, b,
                                carrier == P3_CARRIER_HIGH ? "high" : "low",
                                accepted ? 1 : 0);
                        trace_phase("V90 p3_demod J detected accepted=%d", accepted ? 1 : 0);
                        return;
                    }
                }
                p3_result_free(result);
            }
        }
    }

    /* Diagnostic: log periodically so we can see p3_demod is running but
     * not finding J (useful for noisy-signal debugging). */
    if ((g_v90_p3_ja_scan_total % P3_JA_SCAN_LOG_INTERVAL) < P3_JA_SCAN_THROTTLE) {
        fprintf(stderr,
                "[ME] V.90 p3_demod: J scan at %d ms, no J found "
                "(baud=%d tried all carriers)\n",
                g_v90_p3_ja_scan_total * 1000 / 8000, baud_code);
    }
}

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

    /* V.90 only.  This is an energy heuristic -- sustained signal, silent gap,
       energy returns => call it Ja -- tuned to the V.90 upstream, which runs
       S/PP/TRN straight into Ja.  The V.92 Phase 3 upstream is
       Ru/uR/[MD]/Ru/uR/TRN1u/Ja (Figure 10), so it has gaps *before* Ja that
       this fires on, and V.92 9.5.1.1.3 anyway requires Sd to start only
       "after receiving a DIL descriptor of Ja" -- a CRC-valid descriptor, not
       an energy edge.  Under V.92 the strict decode in v92_p3_rx owns Ja
       (v92_apply_p3_ja_locked); leaving this armed transmitted Sd seconds
       early, during the peer's own training, so the peer -- which only looks
       for Sd after it sends Ja -- reported Error Energy = -0.000 for the whole
       call. */
    if (!g_v90 || g_v92_active || v90_get_tx_phase(g_v90) != V90_TX_WAIT_JA) {
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
                        bool accepted;
                        double tone_a_max = me_v90_s_tone_a_max_fraction();
                        double tone_a = (tone_a_max > 0.0)
                                      ? v90_s_tone_a_fraction_locked() : 0.0;

                        /* "Energy came back" is also true when the peer gives
                           up and starts its §9.5.2.1 Tone A retrain, and that
                           is exactly the shape this heuristic sees: the peer
                           stops Ja, there is a gap, then a loud steady tone.
                           Live (20260812T073702Z) the Courier's real Ja ran
                           9.0-12.1 s and this gate fired at 12.20 s on the
                           retrain tone instead -- so Sd started against a
                           modem that had already left Phase 3, and the Ja that
                           carried the DIL descriptor was never decoded.
                           Treating a retrain as Ja is strictly worse than not
                           firing: WAIT_JA still has its own timeout. */
                        if (tone_a_max > 0.0 && tone_a >= tone_a_max) {
                            /* Tone A persists, so this retests every 30 ms for
                               the rest of the call -- say it once. */
                            if (!g_v90_wait_ja_tone_a_logged) {
                                g_v90_wait_ja_tone_a_logged = true;
                                ME_LOG("[ME] V.90 WAIT_JA: energy returned after gap but it is the "
                                       "far-end Tone A retrain (band2400=%.3f >= %.3f); not Ja\n",
                                       tone_a, tone_a_max);
                                trace_phase("V90 WAIT_JA energy return rejected as Tone A band=%.3f",
                                            tone_a);
                            }
                            ms_in_state = 0;
                            break;
                        }
                        if (!v90_ja_heuristic_allowed("energy-gap")) {
                            /* Not a state change: leave the gate armed so a
                               later, descriptor-backed Ja is still seen. */
                            ms_in_state = 0;
                            break;
                        }
                        accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_J);

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

bool me_rx_g711_slip_permitted(void)
{
    static int allowed = -1;

    /* See modem_engine.h: splicing a sample into the received stream costs
     * the connection and closes no loop, so it is off unless asked for. */
    if (allowed < 0) {
        const char *v = getenv("ME_RX_CLOCK_SLIP");

        allowed = (v && atoi(v) != 0) ? 1 : 0;
        /* Process-scoped on purpose: this is a configuration readout, not
         * per-call state.  It is logged because a silent regression here
         * costs every long call and shows up nowhere else -- the splice is
         * applied after the RX tap, so no recording of a live call can
         * contain the evidence. */
        ME_LOG("[ME] RX clock-recovery slips: %s\n",
               allowed ? "ENABLED (ME_RX_CLOCK_SLIP) — expect collapses"
                       : "disabled (default)");
    }
    /*endif*/
    if (!allowed)
        return false;
    /*endif*/

    /* Even when asked for, never on a DS0 stream: in the V.90 analogue role
     * the received octets *are* the constellation, so an inserted one shifts
     * every 5.4 data frame after it and desynchronises the 5.3 scrambler. */
    {
        bool ds0;

        pthread_mutex_lock(&g_state_mtx);
        ds0 = (g_v90a != NULL);
        pthread_mutex_unlock(&g_state_mtx);
        return !ds0;
    }
}

/* See g_v34_rx_samples.  Reports the first shortfall and then every further
 * 8000 samples of it, so a steady leak shows its rate rather than one line. */
static void me_rx_accounting_check(void)
{
    static uint64_t last_reported;
    uint64_t arrived;
    uint64_t missing;

    if (g_v34_rx_samples == 0)
        return;
    /*endif*/
    arrived = g_g711_rx_octets - g_v34_rx_started_at;
    if (arrived <= g_v34_rx_samples)
        return;
    /*endif*/
    missing = arrived - g_v34_rx_samples;
    if (!g_v34_rx_accounting_logged || missing >= last_reported + 8000) {
        const char *v = getenv("ME_RX_ACCOUNTING");

        if (!v || atoi(v) != 0) {
            ME_LOG("[ME] RX sample accounting: %llu of %llu samples never "
                   "reached v34_rx (%.1f ppm of the wire); %llu reached "
                   "me_rx_audio\n",
                   (unsigned long long)missing,
                   (unsigned long long)arrived,
                   arrived ? 1.0e6*(double)missing/(double)arrived : 0.0,
                   (unsigned long long)(g_rx_audio_samples
                                        - g_rx_audio_started_at));
        }
        g_v34_rx_accounting_logged = true;
        last_reported = missing;
    }
    /*endif*/
}

/* ------------------------------------------------------------------ */
/* Fax (T.31 class 1) audio                                            */
/* ------------------------------------------------------------------ */

/*
 * With a fax service class selected (AT+FCLASS=1), the call's audio belongs
 * to T.31's fax datapumps, not to the data modem: T.30 negotiation happens in
 * the DTE above the +FTM/+FRM/+FTH/+FRH commands, so there is no V.8 exchange
 * and no V.34/V.90 startup to run.  These wrappers convert the G.711 the
 * bearer carries to the linear PCM SpanDSP's fax modems take, in the chunk
 * size the rest of this file uses.
 */
static bool me_fax_rx_g711(const uint8_t *codewords, int count)
{
    int offset;

    /* Annex F keeps the bearer in V.8/V.34; T.30 is attached to the V.34
       control-channel bit callbacks after clause 12 start-up. */
    if (!di_fax_active() || me_v34_fax_probe())
        return false;

    for (offset = 0; offset < count; ) {
        int16_t linear[320];
        int chunk = count - offset;

        if (chunk > (int)(sizeof(linear) / sizeof(linear[0])))
            chunk = (int)(sizeof(linear) / sizeof(linear[0]));
        for (int i = 0; i < chunk; i++)
            linear[i] = pcm_to_linear(codewords[offset + i]);
        di_fax_rx(linear, chunk);
        offset += chunk;
    }
    return true;
}

static bool me_fax_tx_g711(uint8_t *codewords, int count)
{
    int offset;

    if (!di_fax_active() || me_v34_fax_probe())
        return false;

    for (offset = 0; offset < count; ) {
        int16_t linear[320];
        int chunk = count - offset;

        if (chunk > (int)(sizeof(linear) / sizeof(linear[0])))
            chunk = (int)(sizeof(linear) / sizeof(linear[0]));
        di_fax_tx(linear, chunk);
        for (int i = 0; i < chunk; i++)
            codewords[offset + i] = linear_to_pcm(linear[i]);
        offset += chunk;
    }
    return true;
}

void me_rx_audio(const int16_t *amp, int len)
{
    g_rx_audio_samples += (uint64_t)len;

    if (di_fax_active() && !me_v34_fax_probe()) {
        di_fax_rx(amp, len);
        return;
    }

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

    /*
     * The analogue role over an analogue bearer.
     *
     * §8.4's Phase 3 signals and §8.6's Phase 4 ones are one G.711 level per
     * DS0 interval, so the receiver is a codeword state machine -- but on a
     * two-wire line nothing hands us codewords, only the levels they produced.
     * Slice them back (v90_analogue_linear.c) and run exactly the receiver the
     * digital bearer runs.  Skipped when these samples are me_rx_g711()'s own
     * linear copy, where the real codewords have already been fed.
     */
    if (!g_rx_from_g711 && !g_v90a_16k && g_v90a_started && g_v90a && g_v90a_linear
        && (state == ME_TRAINING || state == ME_DATA)) {
        int offset = 0;

        while (offset < len) {
            uint8_t codewords[576];
            int chunk = len - offset;
            int n;

            if (chunk > 256)
                chunk = 256;
            n = v90a_linear_put(g_v90a_linear, amp + offset, chunk,
                                codewords, (int)sizeof(codewords));
            if (n > 0) {
                pthread_mutex_lock(&g_state_mtx);
                if (g_v90a_started && g_v90a)
                    me_v90_analogue_rx_codewords_locked(codewords, n);
                pthread_mutex_unlock(&g_state_mtx);
                /* §8.4.4's W is learned off the wire and everything after Sd
                 * is identified by its level relative to it, so the gain that
                 * acquired Sd is the one the rest of Phase 3 must keep. */
                if (!v90a_linear_locked(g_v90a_linear)
                    && v90_analogue_phase3_rx_stage(g_v90a) != V90A_RX_HUNT_SD) {
                    v90a_linear_lock(g_v90a_linear);
                    ME_LOG("[ME] V.90 analogue: level slicer locked, "
                           "line level %.0f, gain %.4f\n",
                           v90a_linear_level(g_v90a_linear),
                           v90a_linear_gain(g_v90a_linear));
                }
            }
            offset += chunk;
        }
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
        if (state == ME_TRAINING && elapsed > me_training_timeout_ms()) {
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

                /* T.30 Annex F/F.3.1.4: once PPh/MPh/E has completed, HDLC
                   frames replace V.21 on V.34's 1200 bit/s control channel.
                   Keep the probe behaviour when no Class 2 T.30 terminal is
                   active; that path intentionally records idle user bits. */
                if (!g_v34hdx_fax_control_started
                    && di_fax_active()
                    && v34_get_hdx_control_channel_ready(g_v34)) {
                    int primary_rate = v34_get_hdx_negotiated_bit_rate(g_v34);

                    di_on_connected(primary_rate);
                    if (di_fax_v34hdx_start_control(primary_rate) == 0) {
                        g_v34hdx_fax_control_started = true;
                        g_v34hdx_fax_mode = V34_HALF_DUPLEX_CONTROL_CHANNEL;
                        g_phase_start_ms = 0;
                        ME_LOG("[ME] T.30 Annex F attached to V.34 control channel "
                               "(primary %d bit/s)\n", primary_rate);
                        trace_phase("V34HDX enter T30 control: primary=%d",
                                    primary_rate);
                    }
                }

                if (g_v34hdx_fax_control_started) {
                    int requested_mode = di_fax_v34hdx_get_mode();

                    if (requested_mode != g_v34hdx_fax_mode) {
                        ME_LOG("[ME] T.30 Annex F channel request: %s\n",
                               requested_mode == V34_HALF_DUPLEX_PRIMARY_CHANNEL
                                   ? "primary" : "control");
                        v34_half_duplex_change_mode(g_v34, requested_mode);
                        g_v34hdx_fax_mode = requested_mode;
                    }
                }

                /* Begin echo control at Phase 3.  Plain V.34 retunes this from
                   INFO1a's negotiated per-direction parameters, not the
                   startup profile (V.34 10.1.2.3.5/Table 16). */
                bool notch_active = (rx_stage >= V34_RX_STAGE_PHASE3_WAIT_S) ||
                                    (tx_stage >= V34_TX_STAGE_FIRST_S);
                if (notch_active) {
                    if (g_mod == ME_MOD_V90)
                        v90_retire_phase2_cc_notch();
                    else
                        v34_update_echo_policy();
                }
                int16_t filtered[len];
                memcpy(filtered, amp, len * sizeof(int16_t));
                /* A block of digital silence inside an armed §9.6 CP window.
                 * See g_v90_reneg_dead_blocks. */
                if (g_v90_reneg_cp_rx_marked) {
                    int nonzero = 0;

                    for (int i = 0; i < len; i++) {
                        if (amp[i] != 0) {
                            nonzero = 1;
                            break;
                        }
                    }
                    if (!nonzero) {
                        g_v90_reneg_dead_blocks++;
                        g_v90_reneg_dead_samples += (unsigned)len;
                    }
                }
                /* NLMS echo canceller: subtract our TX echo from the RX signal.
                   Critical for V.90 where broadband PCM TX overwhelms the
                   upstream V.34 signal through the FXS hybrid.

                   Uses a single 512-tap FIR (64ms) with no separate delay search.
                   The FIR naturally learns the echo impulse response including
                   the bulk delay (~120 samples / 15ms through the SIP/FXS path).

                   TX reference comes from the ring buffer with a fixed lookback
                   so that tap[0] corresponds to the most recent TX sample and
                   tap[N] corresponds to N samples ago. */
                /* ME_V34_ECHO=none takes the canceller out as well, V.90
                   included.  It is not free: with no echo to cancel, an NLMS
                   filter's taps random-walk around zero and it injects its own
                   misadjustment into the receive path, driven by the transmit
                   reference -- and in V.90 that reference is the downstream
                   PCM, measured 8.4 dB louder than the upstream we are trying
                   to receive.  Measured on this rig's taps, the echo it exists
                   to remove is not there: peak normalised cross-correlation
                   between our transmit and our receive is 0.0068 over lags 0
                   to 1024, and a 512-tap least-squares fit of transmit into
                   receive removes 0.307% of the receive power -- which is
                   512/160000, exactly the bias of fitting that many taps to
                   that many samples, i.e. nothing. */
                /* On V.90 the canceller is now opt-in (ME_V34_ECHO=canceller)
                   rather than unconditional.  The log it prints is its own
                   verdict: across the soak runs in artifacts/ its effect on
                   the receive RMS oscillates around zero (+0.2 to -0.4 dB,
                   mean nil), and on the batches where the received signal is
                   digital silence -- pre_rms=0 -- it emits post_rms 60 to 159.
                   A filter that turns silence into RMS 159 is not removing an
                   echo, it is adding its own transmit-driven misadjustment,
                   and against a received upstream of RMS ~1240 that alone
                   pins the receive SNR at 18-22 dB.  A 31200 bit/s upstream at
                   3200 baud is 9.75 bits/symbol and wants far more than that,
                   so the canceller was capping the very thing it sat in front
                   of -- the same shape of defect as the Phase 2 notch, and
                   invisible for the same reason: it is applied after the RX
                   G.711 tap, so every recording exonerates the wire. */
                if (g_echo_can && notch_active
                    && v34_echo_policy_mode() != V34_ECHO_NONE
                    && ((g_mod == ME_MOD_V90
                         && v34_echo_policy_mode() == V34_ECHO_CANCELLER)
                        || g_v34_use_echo_can)) {
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
                /* Acquisition markers must survive a legal INFO1d carrier
                 * choice that confuses the ordinary V.34 echo front end.
                 * Grade the physical signal/gap/Ja edge on unmodified RX. */
                v90_wait_ja_energy_gate_locked(amp, len);
                v90_s_echo_record_rx_locked(filtered, amp, len);
                /*
                 * Once the analogue role's Phase 3 is running, what arrives is
                 * the digital modem's PCM downstream — Sd, TRN1d, Jd, DIL —
                 * which is not a V.34 signal.  me_rx_g711() hands those to the
                 * codeword receiver; putting them through a V.34 demodulator
                 * as well only produces events about signals that are not
                 * there, and its Phase 3 detectors act on them.
                 */
                if (!g_v90a_started) {
                    if (g_v34_rx_samples == 0) {
                        g_v34_rx_started_at = g_g711_rx_octets;
                        g_rx_audio_started_at =
                            g_rx_audio_samples - (uint64_t)len;
                    }
                    /*endif*/
                    g_v34_rx_samples += (uint64_t)len;
                    v34_rx(g_v34, filtered, len);
                    me_rx_accounting_check();
                    /* The T/3 interpolator is exact-rational 6/5 and begins
                     * counting input samples at the arm, so which of its five
                     * phases a call runs on is fixed by the bearer index it
                     * was armed at.  Offline that residue decides the call:
                     * swept over eqoff-28800-r1's own recording, two of the
                     * five phases wind the timing loop to -70 ppm and lose
                     * the constellation 0.9 s after B1 while three hold for
                     * the whole file, and the outcome tracks the residue and
                     * not the handover instant -- k and k+5 agree to every
                     * printed digit.  Nothing recorded it live, so the five
                     * 28800 captures of 2026-08-24 cannot say whether the
                     * live phase is pinned or a lottery.  Both counters are
                     * exact and both include this block, so their difference
                     * is the arm index itself. */
                    if (g_v34_upstream_data_started
                            && !g_v90_t3_arm_logged) {
                        int64_t input_8k = 0;
                        int64_t output_t3 = 0;

                        v34_v90_upstream_sample_counts(g_v34, &input_8k,
                                                       &output_t3);
                        if (input_8k > 0) {
                            int64_t arm = (int64_t)g_g711_rx_octets - input_8k;

                            g_v90_t3_arm_logged = true;
                            ME_LOG("[ME] V.90 upstream T/3 armed at bearer "
                                   "sample %lld, interpolator phase %lld of 5 "
                                   "(%lld samples in, %lld T/3 out)\n",
                                   (long long)arm,
                                   (long long)(((arm % 5) + 5) % 5),
                                   (long long)input_8k,
                                   (long long)output_t3);
                        }
                    }
                }
                /* p3_demod J scanner runs AFTER v34_rx so the real-time V.34
                   receiver processes samples first.  The scanner reads the
                   raw companion ring filled beside g_rx_ref_buf, not from
                   v34_rx's internal state, so ordering is safe. */
                v90_p3_scan_ja_locked(len);
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

                /* Drive the Ja descriptor search off the capture, not off J.
                 *
                 * Every other call site for this is a J/Ja event handler, so
                 * once those stop firing the capture keeps growing with nothing
                 * parsing it.  Measured live against the Courier (2026-08-13,
                 * commit 1d31353): the capture reached 49152 bits while the
                 * search never saw past 15850, and this peer's descriptor needs
                 * ~16340 -- so the frame was in the buffer, ~500 bits beyond
                 * where anyone looked, and Phase 3 then moved on without DIL.
                 *
                 * §9.3.2.9 has the analogue modem send the descriptor inside
                 * Ja, so the right trigger is "Ja is still arriving", which is
                 * exactly these two stages.  The function's own
                 * ME_V90_DIL_HYP_RETRY_BITS throttle (512) already paces the
                 * 24-hypothesis sliding search, and it early-returns once a
                 * descriptor is logged, so calling it per RX frame here costs a
                 * bit-count comparison in the common case. */
                if (g_mod == ME_MOD_V90
                    && g_v90
                    && (rx_stage == V34_RX_STAGE_PHASE3_WAIT_S
                        || rx_stage == V34_RX_STAGE_PHASE3_TRAINING))
                {
                    (void)v90_dil_capture_try_v34_hypotheses();
                }

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
                    /* Our own deadline, cumulative over the call.  Checked
                     * here because this poll runs per RX frame and the
                     * attempt-count trigger below can only fire when the PEER
                     * retrains -- which is why it never fires first (§35m). */
                    /* Not gated on g_v90_phase3_started: the clock measures
                     * time spent trying V.90 since the first Phase 3 entry,
                     * which is what the corpus priced, and a call that keeps
                     * failing spends most of it back in Phase 2 restarts --
                     * gated, the deadline can only be tested during the very
                     * attempts that are failing and routinely misses. */
                    if (v90_ja_deadline_seconds() > 0
                        && !g_v90_pending_dil_valid
                        && g_v90_phase3_first_samples != 0
                        && g_rx_audio_samples - g_v90_phase3_first_samples
                           >= (uint64_t) v90_ja_deadline_seconds() * 8000u) {
                        char why[128];

                        snprintf(why, sizeof(why),
                                 "%d s of Phase 3 with no CRC-valid Ja descriptor",
                                 v90_ja_deadline_seconds());
                        v90_concede_to_v34_locked(why);
                    }
                    if (new_retrain_event && g_v90 && !g_v92_active) {
                        /* The peer gave up on Phase 3/4 and initiated a
                         * retrain: 70 ms silence then Tone A, waiting for our
                         * Tone B (V.90 §9.5.2.1).  §9.4.1/§9.3.1 require the
                         * digital modem to respond per §9.5.1.2 -- back into
                         * the Phase 2 tone/INFO exchange.  Dropping only the
                         * v90 object to WAIT_JA is not enough: the V.34
                         * receiver stays parked in a Phase 3/4 stage, nothing
                         * ever answers Tone A, and the SmartLink peer declares
                         * a link error after ~3.1 s of unanswered Tone A
                         * (observed live 2026-07-22). */
                        bool accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_RETRAIN);
                        /* Did this attempt ever recover a descriptor?  Read it
                         * before the reset below clears the flag. */
                        bool had_descriptor = g_v90_pending_dil_valid;

                        if (had_descriptor)
                            g_v90_ja_failed_attempts = 0;
                        else
                            g_v90_ja_failed_attempts++;

                        g_v90_phase3_s_events = 0;
                        g_v90_pending_dil_valid = false;
                        g_v90_dil_capture_bits = 0;
                        g_v90_dil_capture_search = 0;
                        g_v90_dil_hyp_last_bits = 0;
                        ME_LOG("[ME] V.90: peer retrain detected; following it back to Phase 2 "
                               "(accepted=%d)\n", accepted ? 1 : 0);
                        trace_phase("V90 peer retrain detected, following (accepted=%d)",
                                    accepted ? 1 : 0);
                        /* PEER_RETRAIN is an application-owned event, unlike
                         * the ordinary V.34 TX/RX handshake events.  A rejected
                         * notification used to stay sticky and suppress the
                         * later J/Ja detector indefinitely. */
                        v34_v90_clear_peer_retrain_event(g_v34);
                        /* §9.5.1.2 response: restart the answerer Phase 2 flow.
                         * This frees g_v90; later blocks in this poll are all
                         * guarded on g_v90. */
                        (void) restart_v90_phase2_locked(
                            "peer retrain (Tone A/silence) during Phase 3/4; "
                            "responding per 9.5.1.2");
                        /* §9.5.1.2 is answered above whatever we decide next --
                         * the peer is holding Tone A and something has to reply
                         * to it.  What changes here is only whether the Phase 2
                         * we just restarted still asks for V.90. */
                        if (v90_ja_concede_attempts() > 0
                            && g_v90_ja_failed_attempts >= v90_ja_concede_attempts()) {
                            char why[128];

                            snprintf(why, sizeof(why),
                                     "%d Phase 3 attempts ended with no CRC-valid "
                                     "Ja descriptor", g_v90_ja_failed_attempts);
                            v90_concede_to_v34_locked(why);
                        }
                    }
                    if (rx_event == V34_EVENT_PEER_RENEG_S
                        && g_v90 && !g_v92_active
                        && !v90_rate_renegotiation_active(g_v90)) {
                        /* §9.6.1.2 responds to a renegotiation the PEER
                         * opened.  While one of OURS is running, the same S
                         * is Figure 8/V.90's answer to our Rd, not a new
                         * request -- responding to it would open a second
                         * renegotiation inside the first. */
                        /* §9.6.1.2: "After detecting S, the digital modem
                         * shall clamp circuit 104 ... After detecting the
                         * S-to-S-bar transition, the digital modem shall
                         * transmit signal Rd for 384T and R-bar-d for 24T ...
                         * and condition its receiver to receive CP."
                         *
                         * That transmit sequence is identical to the one
                         * §9.6.1.1.1 sends when we initiate, so it is the
                         * same machinery: arm the request, and the transmit
                         * path starts it at the next data frame boundary,
                         * which §9.6 requires anyway.  The S-to-S-bar
                         * transition is not separately detected -- S is 128T
                         * and S-bar 24T, and the peer then runs SCR and CP
                         * for up to 2 s, so the boundary we start on is well
                         * inside the window it is waiting in.
                         *
                         * Not gated on me_v90_reneg_enabled(): that knob says
                         * whether to START one, and §9.6.1.2 is a "shall"
                         * regardless.  The detector behind the event is
                         * itself off unless ME_V90_RENEG_RESPOND=1. */
                        v34_clear_peer_reneg_s_event(g_v34);
                        if (!v90_rate_renegotiation_active(g_v90)) {
                            ME_LOG("[ME] V.90: peer opened a §9.6 rate "
                                   "renegotiation; answering with Rd at the "
                                   "next data frame boundary\n");
                            trace_phase("V90 answering peer rate renegotiation");
                            (void) v90_request_rate_renegotiation(g_v90);
                        }
                    }
                    if (new_e_event && g_v90 && !g_v92_active) {
                        bool accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_E);

                        fprintf(stderr,
                                "[ME] V.90 strict RX event=E tx_phase=%d accepted=%d\n",
                                (int)v90_get_tx_phase(g_v90), accepted ? 1 : 0);
                        trace_phase("V90 strict RX event=E accepted=%d",
                                    accepted ? 1 : 0);
                    }
                    if (new_j_event && g_v90 && !g_v92_active) {
                        /* Accept v34rx J directly.  p3_demod runs as an
                           independent fast-path scanner (v90_p3_scan_ja)
                           that fires J in ~80 ms when it can classify the
                           16-bit pattern; the v34rx canonical path (8
                           sustained windows) and energy-gap heuristic are
                           slower but work on noisier signals where p3_demod's
                           structural classifier may not find a clean J
                           segment.  Neither path should block the other.
                           The p3_demod S gate (below) is the one that
                           matters — it prevents false-S DIL cutoff.

                           All three are now subordinate to the descriptor:
                           they detect that Ja is present, which is true and
                           useful, but presence is not what §9.3.1.3 needs
                           before Sd may start.  Decode first, then fire. */
                        bool accepted;

                        (void)v90_dil_capture_try_v34_hypotheses();
                        accepted = v90_ja_heuristic_allowed("v34rx")
                                 ? v90_handle_rx_event(g_v90, V90_RX_EVENT_J)
                                 : false;

                        if (accepted) {
                            (void)v90_dil_capture_try_v34_hypotheses();
                            v34_v90_arm_phase3_s_detector(g_v34);
                        }

                        fprintf(stderr,
                                "[ME] V.90 strict RX event=J tx_phase=%d accepted=%d"
                                " p3_demod_fired=%d\n",
                                (int)v90_get_tx_phase(g_v90), accepted ? 1 : 0,
                                g_v90_p3_ja_fired ? 1 : 0);
                        trace_phase("V90 strict RX event=J accepted=%d p3_demod_fired=%d",
                                    accepted ? 1 : 0, g_v90_p3_ja_fired ? 1 : 0);
                    }
                }

                /* Plain V.34 §11.5/§11.6.  The V.90 path above has had a
                 * retrain response since 2026-07-22; plain V.34 never did,
                 * so a peer that gave up and held its tone was answered with
                 * a data mapper still running over the top of it, and the
                 * call died where §11.5.1.2 says to resynchronise. */
                if (g_mod == ME_MOD_V34 && g_v34) {
                    /* Plain V.34 can sit in PHASE3_WAIT_S for the whole call
                     * when the far end's symbol clock is not phase-locked to
                     * our sample grid -- measured over a Conexant HSF DAA
                     * against this project's own digital modem, where about a
                     * tenth of the receive sampling phases acquire and the
                     * rest never see 10.1.3.7's S at all.  Nothing in the
                     * default path bounds that wait: the peer completes its
                     * own Phase 3, goes to data mode and stops sending S, so
                     * the one attempt is the whole call.  ME_V34_PHASE3_S_
                     * TIMEOUT_MS restarts Phase 2 instead, which gives the
                     * next attempt a fresh draw.  Default 0 (off) -- on a
                     * digital bearer this never fires and the wait is bounded
                     * by the training timeout as before. */
                    if (rx_stage == V34_RX_STAGE_PHASE3_WAIT_S) {
                        int limit = parse_env_int("ME_V34_PHASE3_S_TIMEOUT_MS", 0);

                        if (g_v34_phase3_wait_s_ms == 0)
                            g_v34_phase3_wait_s_ms = trace_now_ms();
                        else if (limit > 0
                                 && trace_now_ms() - g_v34_phase3_wait_s_ms
                                    > (uint64_t) limit) {
                            g_v34_phase3_wait_s_ms = 0;
                            ME_LOG("[ME] V.34: no far-end S after %d ms in "
                                   "PHASE3_WAIT_S; restarting Phase 2\n", limit);
                            (void) restart_v34_phase2_locked(
                                "no far-end S (ME_V34_PHASE3_S_TIMEOUT_MS)");
                            pthread_mutex_unlock(&g_state_mtx);
                            return;
                        }
                    } else {
                        g_v34_phase3_wait_s_ms = 0;
                    }
                    if (rx_event == V34_EVENT_PEER_RETRAIN) {
                        /* §11.5.1.2/§11.5.2.2: silence, then our own tone,
                         * then back to §11.2.1.  v34_restart() re-enters
                         * Phase 2 from INFO0, which is the branch both
                         * §11.5.1.1 and §11.5.2.1 name for a peer that sends
                         * INFO0 rather than a bare tone ("If INFO0a is
                         * received, the modem shall proceed in accordance
                         * with 11.8.1"). */
                        v34_clear_peer_retrain_event(g_v34);
                        v34_reneg_clear_locked();
                        (void) restart_v34_phase2_locked(
                            "peer retrain tone detected; responding per 11.5");
                    } else if (rx_event == V34_EVENT_PEER_RENEG_S) {
                        /* §11.6.1.2: the peer has opened a rate
                         * renegotiation.  Responding is the same sequence it
                         * is sending -- S, S-bar, TRN, MP -- and by here its
                         * S is already detected, which is all §11.6.1.2.1
                         * asks for before §11.6.1.2.2 starts ours. */
                        v34_clear_peer_reneg_s_event(g_v34);
                        if (!v34_rate_renegotiation_active(g_v34)
                            && v34_start_rate_renegotiation(g_v34) == 0) {
                            ME_LOG("[ME] V.34: peer opened a §11.6 rate "
                                   "renegotiation; answering with S/S-bar/TRN/MP\n");
                            trace_phase("V34 answering peer rate renegotiation");
                            v34_reneg_begin_locked();
                        }
                    } else if (v34_rate_renegotiation_active(g_v34)) {
                        /* §11.6.2: "If after transmitting the S-to-S-bar
                         * transition, the modem has not received sequence E
                         * for the following timeout period, it shall initiate
                         * the retrain procedure." */
                        if (v34_reneg_timed_out_locked()) {
                            v34_reneg_clear_locked();
                            ME_LOG("[ME] V.34 §11.6 rate renegotiation "
                                   "produced no E; falling back to a §11.5 "
                                   "retrain\n");
                            (void) restart_v34_phase2_locked(
                                "rate renegotiation timeout");
                        }
                    } else if (v34_retrain_probe_due_locked()) {
                        g_v34_retrain_probe_done = true;
                        ME_LOG("[ME] V.34: ME_V34_RETRAIN_AFTER_MS probe; "
                               "initiating a §11.5 retrain\n");
                        v34_reneg_clear_locked();
                        (void) restart_v34_phase2_locked(
                            "retrain probe (ME_V34_RETRAIN_AFTER_MS)");
                    } else if (v34_reneg_probe_due_locked()) {
                        g_v34_reneg_probe_done = true;
                        if (v34_start_rate_renegotiation(g_v34) == 0) {
                            ME_LOG("[ME] V.34: ME_V34_RENEG_AFTER_MS probe; "
                                   "opening a §11.6 rate renegotiation\n");
                            trace_phase("V34 reneg probe");
                            v34_reneg_begin_locked();
                        }
                    } else if (v34_data_carrier_lost(g_v34)
                               && retrain_on_loss_due(
                                      me_v34_max_loss_retrains())) {
                        /* §11.6 first where the peer implements it -- it is
                         * the cheap resynchronisation and keeps the call in
                         * data mode -- and §11.5.1.1/§11.5.2.1 otherwise.  A
                         * retrain needs only the peer's tone detector, which
                         * every V.34 modem has; a renegotiation needs a
                         * responding modem.  Either way bounded by the same
                         * per-call cap and dwell as the V.90 path, so a line
                         * that is simply too poor is left alone rather than
                         * recovered in a loop. */
                        v34_clear_data_carrier_lost(g_v34);
                        g_loss_retrains++;
                        g_last_loss_retrain_ms = trace_now_ms();
                        if (me_v34_reneg_enabled()
                            && v34_start_rate_renegotiation(g_v34) == 0) {
                            ME_LOG("[ME] V.34 data mode has stopped decoding; "
                                   "initiating a §11.6 rate renegotiation "
                                   "(%u of %d)\n",
                                   g_loss_retrains, me_v34_max_loss_retrains());
                            v34_reneg_begin_locked();
                        } else {
                            ME_LOG("[ME] V.34 data mode has stopped decoding; "
                                   "initiating a §11.5 retrain (%u of %d)\n",
                                   g_loss_retrains, me_v34_max_loss_retrains());
                            (void) restart_v34_phase2_locked(
                                "data mode stopped decoding; retraining "
                                "per 11.5");
                        }
                    }
                }

                if (g_mod == ME_MOD_V90 && g_v90 && !g_v92_active) {
                    int s_events = v34_get_phase3_s_event_count(g_v34);

                    while (g_v90_phase3_s_events < s_events) {
                        bool accepted;
                        int  gate_pct = me_v90_s_echo_gate_pct();
                        /* Measure even when the gate is off, so a capture can
                           always be used to calibrate the threshold. */
                        double echo = v90_s_echo_correlation_locked();
                        double rx_rms = v90_s_rx_rms_locked();
                        double min_rms = me_v90_s_min_rx_rms();
                        bool  is_echo = (gate_pct > 0 && echo >= 0.0
                                         && echo >= gate_pct / 100.0);
                        bool  is_silence = (min_rms > 0.0 && rx_rms < min_rms);

                        g_v90_phase3_s_events++;
                        {
                            double tx_rms = v90_s_tx_rms_locked();
                            double want   = me_v90_s_min_rx_tx_ratio();

                            if (want > 0.0 && tx_rms > 1.0
                                && rx_rms < want * tx_rms) {
                                /* The line is carrying our own transmission and
                                   little else.  Acting on this sends J'd or ends
                                   DIL while the peer is still legitimately
                                   silent (§9.3.2.7 allows it 5000 ms), and it
                                   will then never see the 12-symbol J'd it must
                                   detect per §9.3.2.8. */
                                fprintf(stderr,
                                        "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d "
                                        "REJECTED as own-line noise (rx/tx=%.3f < %.2f, "
                                        "rx_rms=%.1f tx_rms=%.1f corr=%.3f rx_sample=%llu)\n",
                                        g_v90_phase3_s_events,
                                        (int)v90_get_tx_phase(g_v90),
                                        rx_rms / tx_rms, want, rx_rms, tx_rms, echo,
                                        (unsigned long long)g_rx_ref_samples);
                                trace_phase("V90 strict RX event=S count=%d rejected_ratio rx/tx=%.3f",
                                            g_v90_phase3_s_events, rx_rms / tx_rms);
                                continue;
                            }
                        }
                        if (is_silence) {
                            /* The far end is not transmitting at all, so this
                               cannot be its S.  §9.3.2.9 lets it stay silent for
                               the whole of DIL, so acting on this would end DIL
                               before it ever asked us to. */
                            fprintf(stderr,
                                    "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d "
                                    "REJECTED as silence (rx_rms=%.1f < %.1f, corr=%.3f)\n",
                                    g_v90_phase3_s_events,
                                    (int)v90_get_tx_phase(g_v90),
                                    rx_rms, min_rms, echo);
                            trace_phase("V90 strict RX event=S count=%d rejected_silence rms=%.1f",
                                        g_v90_phase3_s_events, rx_rms);
                            continue;
                        }
                        /* §9.3.2.10 DIL terminator: the far end must actually be
                           on the line around the event, not just for one
                           window.  Only applied while transmitting DIL — Jd's
                           terminator is a different, longer signal. */
                        if ((int)v90_get_tx_phase(g_v90) == V90_TX_DIL
                            && me_v90_dil_s_active_fraction() > 0.0) {
                            double want = me_v90_dil_s_active_fraction();
                            double got  = v90_s_rx_active_fraction_locked(min_rms);

                            if (got < want) {
                                fprintf(stderr,
                                        "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d "
                                        "REJECTED as dead line during DIL (active=%.2f < %.2f over 300ms, "
                                        "rx_rms=%.1f corr=%.3f rx_sample=%llu)\n",
                                        g_v90_phase3_s_events,
                                        (int)v90_get_tx_phase(g_v90),
                                        got, want, rx_rms, echo,
                                        (unsigned long long)g_rx_ref_samples);
                                trace_phase("V90 strict RX event=S count=%d rejected_dead_line active=%.2f",
                                            g_v90_phase3_s_events, got);
                                continue;
                            }
                        }
                        if (is_echo) {
                            /* Our own DIL/Ri leaking back, not the analogue
                               modem.  Swallow it: forwarding it would cut Jd or
                               DIL short and desynchronise us from the peer's
                               §9.3.2.7 / §9.3.2.10 schedule. */
                            fprintf(stderr,
                                    "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d "
                                    "REJECTED as own-TX echo (corr=%.3f >= %d%%, rx_rms=%.1f)\n",
                                    g_v90_phase3_s_events,
                                    (int)v90_get_tx_phase(g_v90),
                                    echo, gate_pct, rx_rms);
                            trace_phase("V90 strict RX event=S count=%d rejected_echo corr=%.3f",
                                        g_v90_phase3_s_events, echo);
                            continue;
                        }
                        /* Structural confirmation protects DIL's early-exit
                           seam, where one false event can discard the rest of
                           a long impairment study.  Do NOT make it a second
                           veto during Jd.  v34rx has already required at least
                           64 equalized symbols of §10.1.3.7's alternating or
                           dominant-rotation structure before publishing this
                           event; §9.3.2.7's S itself is only 128T.  Re-running
                           p3_demod over a 200 ms window (mostly the preceding
                           silence/Jd interval) rejected SmartLink's real,
                           weaker S at RMS 652 while accepting the same signal
                           at RMS 845.  The peer had reached waitForJd and then
                           waited for our J'd until we timed out, making the
                           supposed confidence gate the intermittent failure.
                           Echo, silence, RX/TX ratio and Tone-A gates still
                           apply in Jd; retain this extra classifier only for
                           the DIL termination it was introduced to protect. */
                        if ((int)v90_get_tx_phase(g_v90) == V90_TX_DIL
                            && !v90_p3_confirm_signal_locked(P3_SIGNAL_S)) {
                            fprintf(stderr,
                                    "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d "
                                    "REJECTED by p3_demod structural check (no 6-symbol S pattern) "
                                    "echo_corr=%.3f rx_rms=%.1f rx_sample=%llu\n",
                                    g_v90_phase3_s_events,
                                    (int)v90_get_tx_phase(g_v90),
                                    echo, rx_rms, (unsigned long long)g_rx_ref_samples);
                            trace_phase("V90 strict RX event=S count=%d rejected_p3_structural",
                                        g_v90_phase3_s_events);
                            continue;
                        }
                        /* Last gate: a far-end retrain tone.  Runs after the
                           structural check because a constant dibit stream
                           satisfies that check -- 6-symbol periodicity is
                           exactly what a pure tone produces -- so this is the
                           only test that separates Tone A from a real S. */
                        {
                            double tone_a_max = me_v90_s_tone_a_max_fraction();
                            double tone_a = (tone_a_max > 0.0)
                                          ? v90_s_tone_a_fraction_locked() : 0.0;

                            if (tone_a_max > 0.0 && tone_a >= tone_a_max) {
                                fprintf(stderr,
                                        "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d "
                                        "REJECTED as far-end Tone A retrain (band2400=%.3f >= %.3f, "
                                        "echo_corr=%.3f rx_rms=%.1f rx_sample=%llu)\n",
                                        g_v90_phase3_s_events,
                                        (int)v90_get_tx_phase(g_v90),
                                        tone_a, tone_a_max, echo, rx_rms,
                                        (unsigned long long)g_rx_ref_samples);
                                trace_phase("V90 strict RX event=S count=%d rejected_tone_a band=%.3f",
                                            g_v90_phase3_s_events, tone_a);
                                continue;
                            }
                            accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_S);
                            fprintf(stderr,
                                    "[ME] V.90 strict RX event: index=%d event=S tx_phase=%d accepted=%d "
                                    "echo_corr=%.3f rx_rms=%.1f band2400=%.3f rx_sample=%llu\n",
                                    g_v90_phase3_s_events,
                                    (int)v90_get_tx_phase(g_v90),
                                    accepted ? 1 : 0, echo, rx_rms, tone_a,
                                    (unsigned long long)g_rx_ref_samples);
                        }
                        trace_phase("V90 strict RX event=S count=%d accepted=%d echo_corr=%.3f rms=%.1f",
                                    g_v90_phase3_s_events, accepted ? 1 : 0, echo, rx_rms, (unsigned long long)g_rx_ref_samples);
                    }

                    /* Decode live Ja even when an interoperability profile
                     * already supplied the DIL contents.  The decoded frame is
                     * the standards-driven timing event; the profile is only a
                     * fallback if the frame cannot be recovered. */
                    if (!g_v90_dil_parse_logged)
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
                        /* Mark the dump against engine state every 100 ms.
                         * The file is opened once per process and appended
                         * across every call, with no timestamps of its own, so
                         * without these a capture cannot be tied to a phase --
                         * spectral analysis of it can say what tones are on the
                         * line but not which handshake stage was running, which
                         * is the only question worth asking of it. Carrying the
                         * stages here makes each mark self-contained. */
                        static long rx_dump_total = 0;
                        static long rx_dump_marked = 0;

                        fwrite(filtered, sizeof(int16_t), len, rx_dump);
                        rx_dump_total += len;
                        if (rx_dump_total - rx_dump_marked >= 800) {
                            int gc_valid = 0;
                            float gc_db = g_v34
                                        ? v34_get_guard_carrier_db(g_v34, &gc_valid)
                                        : 0.0f;

                            rx_dump_marked = rx_dump_total;
                            /* guard_db is the peer's 1800 Hz guard tone
                             * relative to its 2400 Hz carrier: about +1 dB
                             * under Tone A, about -6 dB under an INFO
                             * sequence (V.34 10.1.2.1/10.1.2.3). Logged on
                             * every mark so the prediction can be checked on
                             * ordinary calls, not just the rare ones that
                             * stall at the 9.2.1.2.6 deadline. */
                            ME_LOG("[ME] RX dump mark: sample=%ld rx_stage=%d tx_stage=%d mod=%d guard_db=%.1f guard_valid=%d\n",
                                   rx_dump_total,
                                   g_v34 ? v34_get_rx_stage(g_v34) : -1,
                                   g_v34 ? v34_get_tx_stage(g_v34) : -1,
                                   (int)g_mod,
                                   gc_db, gc_valid);
                        }
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

static bool get_strict_v90_info1a_locked(v90_info1a_t *info, bool *v92_selected)
{
    v34_v90_info1a_t received;

    if (v92_selected)
        *v92_selected = false;
    if (!info || !g_v34
        || !v34_get_v90_received_info1a(g_v34, &received))
        return false;

    info->md = (uint8_t)received.md;
    info->u_info = (uint8_t)received.u_info;
    info->upstream_symbol_rate_code = (uint8_t)received.upstream_symbol_rate_code;
    info->downstream_rate_code = (uint8_t)received.downstream_rate_code;
    info->freq_offset = (int16_t)received.freq_offset;

    /* V.92 Table 19, against a real one captured 2026-07-21 (raw bytes
     * 3f c0 89 fd 0f 09 44):
     *
     *   raw_12_17=63  md=0  U_INFO=78  raw_32_33=0  upstream=6  downstream=6
     *
     * Upstream rate code 6 is 8000 sym/s -- the PCM upstream itself, since
     * V.92 replaces the V.34 upstream carrier rather than naming one.  That
     * makes 6 the unambiguous marker that the peer selected V.92; a V.34
     * upstream code cannot distinguish V.92 from V.90 here, and a peer that
     * declines PCM upstream reports V.90 anyway ("selected=90" in slmodemd),
     * so anything else is handled as V.90 below.
     *
     * The earlier expectations here were written from the spec text with no
     * ground truth and were wrong three ways: they required raw_12_17 == 0
     * (the peer sets all six bits), raw_32_33 == 0x2 (it sends 0), and a
     * 3000/3200/3429 upstream code (it sends 8000).  Do not reinstate them
     * without a capture that shows them. */
    if (g_v92_info0_mutual
        && received.u_info > 66
        && received.upstream_symbol_rate_code == 6
        && received.downstream_rate_code == 6
        && v90_info1a_validate(info)) {
        if (v92_selected)
            *v92_selected = true;
        return true;
    }
    /* Fall through to the V.90 rules even under a mutual-V.92 INFO0 contract:
     * this peer confirms V.92 capability at INFO0 and can still answer a plain
     * Table 10 INFO1a, and rejecting that stalls Phase 2 on a frame we could
     * have run as V.90.  Reaching here with g_v92_info0_mutual set is the
     * demotion path; the caller keys g_v92_active off *v92_selected, not off
     * the INFO0 contract, so a demoted call runs as ordinary V.90. */
    return received.raw_12_17 == 0
        && received.raw_32_33 == 0
        && received.u_info > 0
        && received.upstream_symbol_rate_code >= 0
        && received.upstream_symbol_rate_code <= 5
        && received.downstream_rate_code == 6
        && v90_info1a_validate(info);
}

/* Read the CRC-validated INFO0a captured by SpanDSP.  Table 16 bit 26 is
 * the analogue V.92 capability and bit 27 requests short Phase 2.  We only
 * implement the long procedure, so a short-Phase-2 request is a deliberate
 * V.90 fallback rather than a partial V.92 start-up. */
static void v92_refresh_info0_confirmation_locked(void)
{
    v34_v90_info0a_t info0a;
    bool old_mutual = g_v92_info0_mutual;

    g_v92_info0_peer_capable = false;
    g_v92_info0_peer_short_phase2 = false;
    if (g_v92_info0_local_advertised
        && g_v34
        && v34_get_v90_received_info0a(g_v34, &info0a)) {
        g_v92_info0_peer_capable = (info0a.raw_26_27 & 0x1) != 0;
        g_v92_info0_peer_short_phase2 = (info0a.raw_26_27 & 0x2) != 0;
        if (!g_v92_info0_peer_logged) {
            ME_LOG("[ME] V.92 INFO0a flags: raw26_27=0x%X, capability(bit26)=%d, short-phase2(bit27)=%d\n",
                   info0a.raw_26_27,
                   g_v92_info0_peer_capable ? 1 : 0,
                   g_v92_info0_peer_short_phase2 ? 1 : 0);
            g_v92_info0_peer_logged = true;
        }
    }
    g_v92_info0_mutual = g_v92_info0_local_advertised
                      && g_v92_info0_peer_capable
                      && !g_v92_info0_peer_short_phase2;
    if (!old_mutual && g_v92_info0_mutual) {
        ME_LOG("[ME] V.92 INFO0 confirmed mutually (INFO0d bit27=1, INFO0a bit26=1); selecting long Phase 2/3\n");
        trace_phase("V92 INFO0 mutual confirmation: long Phase2 selected");
    } else if (g_v92_info0_local_advertised
               && g_v92_info0_peer_short_phase2) {
        ME_LOG("[ME] V.92 INFO0a requested short Phase 2; not implemented, retaining V.90 long-startup path\n");
    }
}

static void v92_su_rx_reset_locked(void)
{
    memset(&g_v92_su_rx, 0, sizeof(g_v92_su_rx));
    g_v92_su_rx.last_polarity = -1;
    g_v92_su_rx_active = false;
    g_v92_su_final_pending = false;
}

/* Return the expected Su class for phase p: +1, 0, or -1.  The inverse
 * polarity is S-bar-u. */
static int v92_su_expected_class(int phase, int polarity)
{
    static const int pattern[6] = {1, 0, 1, -1, 0, -1};
    int value = pattern[phase % 6];

    return polarity ? -value : value;
}

static void v92_su_rx_feed_locked(uint8_t codeword, uint64_t sample_index)
{
    int sample;
    int observed;

    if (!g_v92_su_rx_active || !g_v92_active || !g_v90)
        return;
    sample = pcm_to_linear(codeword);
    if (sample > -900 && sample < 900)
        observed = 0;
    else
        observed = sample > 0 ? 1 : -1;

    for (int h = 0; h < 12; h++) {
        int polarity = h / 6;
        int phase_offset = h % 6;
        int phase = (int)((sample_index + (uint64_t)phase_offset) % 6U);
        int expected = v92_su_expected_class(phase, polarity);

        if (observed == expected)
            g_v92_su_rx.run[h]++;
        else
            g_v92_su_rx.run[h] = 0;
        if (g_v92_su_rx.run[h] < 24 || polarity == g_v92_su_rx.last_polarity)
            continue;

        g_v92_su_rx.last_polarity = polarity;
        if (g_v92_su_rx.transitions == 0) {
            if (v90_handle_rx_event(g_v90, V90_RX_EVENT_SU))
                g_v92_su_rx.transitions++;
        } else if (g_v92_su_rx.transitions == 1) {
            if (v90_handle_rx_event(g_v90, V90_RX_EVENT_SU_BAR))
                g_v92_su_rx.transitions++;
        } else {
            g_v92_su_final_pending = true;
            (void)v90_handle_rx_event(g_v90, V90_RX_EVENT_SU_FINAL);
            g_v92_su_rx.transitions++;
        }
        break;
    }
}

static void v92_start_phase3_cpt_rx_locked(void)
{
    if (!g_v92_active || !g_v90 || g_v92_p3_cpt_active)
        return;
    v92_cp_rx_init(&g_v92_p3_cpt_rx,
                   2,
                   g_law == ME_LAW_ALAW,
                   v92_live_p4u_frame,
                   NULL);
    v92_trn2u_demod_init(&g_v92_p3_cpt_demod,
                         2,
                         g_v92_trn2u_lu,
                         g_law == ME_LAW_ALAW,
                         &g_v92_p3_cpt_rx);
    g_v92_p3_cpt_active = true;
    ME_LOG("[ME] V.92 Phase 3: armed 2-point TRN1u/CPt receiver after Jp'\n");
    trace_phase("V92 Phase3 CPt receiver armed (2-point TRN1u)");
}

static void v92_apply_p3_ja_locked(void)
{
    const ja_dil_decode_t *ja;

    if (!g_v92_p3_rx_active || !g_v90)
        return;
    if (!v92_p3_rx_ja_ok(&g_v92_p3_rx)) {
        if (v92_p3_rx_get_state(&g_v92_p3_rx) == V92_P3_RX_FAILED
            && !g_v92_p3_rx_failure_logged) {
            int sample = -1;
            int metric0 = 0;
            int metric1 = 0;
            v92_p3_rx_reject_t reason =
                v92_p3_rx_last_reject(&g_v92_p3_rx, &sample, &metric0, &metric1);

            g_v92_p3_rx_failure_logged = true;
            ME_LOG("[ME] V.92 Phase 3 receiver rejected before Ja: reason=%s sample=%d m0=%d m1=%d\n",
                   v92_p3_rx_reject_name(reason), sample, metric0, metric1);
            trace_phase("V92 Phase3 receive rejected: %s", v92_p3_rx_reject_name(reason));
        }
        return;
    }
    if (g_v92_p3_rx_result_applied)
        return;
    ja = v92_p3_rx_get_ja(&g_v92_p3_rx);
    if (!ja || !ja->ok || !ja->parsed_v92) {
        /* Neither result_applied nor rx_active changes here, so this runs
           again for every audio block until the call ends -- log it once
           (129360 copies in one live call before this was gated), and say
           which half failed so the next pass knows where to look. */
        if (!g_v92_p3_rx_failure_logged) {
            g_v92_p3_rx_failure_logged = true;
            ME_LOG("[ME] V.92 Phase 3 receiver produced non-strict Ja (ja=%d ok=%d parsed_v92=%d); "
                   "refusing V.90 fallback handoff\n",
                   ja ? 1 : 0,
                   (ja && ja->ok) ? 1 : 0,
                   (ja && ja->parsed_v92) ? 1 : 0);
            trace_phase("V92 Phase3 non-strict Ja: ok=%d parsed_v92=%d",
                        (ja && ja->ok) ? 1 : 0,
                        (ja && ja->parsed_v92) ? 1 : 0);
        }
        return;
    }

    g_v90_pending_dil = ja->desc;
    g_v90_pending_dil_valid = true;
    v90_set_dil_descriptor(g_v90, &ja->desc);
    g_v92_p3_rx_result_applied = true;
    g_v92_p3_rx_active = false;
    v92_su_rx_reset_locked();
    g_v92_su_rx_active = true;
    ME_LOG("[ME] V.92 Phase 3 strict Ja accepted: sample=%d bits=%d N=%u LSP=%u LTP=%u; starting Sd\n",
           ja->start_sample, ja->descriptor_bits,
           (unsigned)ja->desc.n, (unsigned)ja->desc.lsp, (unsigned)ja->desc.ltp);
    trace_phase("V92 Phase3 Ja valid -> project-owned TX: N=%u LSP=%u LTP=%u",
                (unsigned)ja->desc.n, (unsigned)ja->desc.lsp,
                (unsigned)ja->desc.ltp);
    if (!v90_handle_rx_event(g_v90, V90_RX_EVENT_J))
        ME_LOG("[ME] V.92 Phase 3: strict Ja arrived outside WAIT_JA\n");
}

/*
 * The analogue role's Phase 2/3 seam, called with g_state_mtx held.
 *
 * SpanDSP's transmit stage reaching FIRST_S is the moment its Phase 2 is over:
 * it has sent INFO1a and would start V.34's own S/S̄/PP/TRN next.  §9.3.2.1
 * starts from the same point but with V.90's sequence, so the modulator is
 * taken over here and SpanDSP's Phase 3 never runs.
 *
 * Its *receiver* stops being fed at the same moment (see me_rx_audio): what
 * arrives from here on is the digital modem's PCM downstream, which is not a
 * V.34 signal at all, and feeding it to a V.34 demodulator produces events
 * about signals that are not there.
 */
static void prepare_v90_analogue_phase3_locked(void)
{
    v90_analogue_phase3_config_t cfg;

    if (!me_v90_analogue_role() || g_mod != ME_MOD_V90 || !g_v34 || g_v90a_started)
        return;
    if (v34_get_tx_stage(g_v34) < V34_TX_STAGE_FIRST_S)
        return;

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = (g_law == ME_LAW_ALAW) ? V90_LAW_ALAW : V90_LAW_ULAW;
    /* Table 10 bits 34:36 were selected from INFO1d when INFO1a was built.
     * 3200 is mandatory; 3000/3429 are used only when the peer enabled them. */
    cfg.baud_rate_code = v34_get_tx_baud_rate(g_v34);
    if (cfg.baud_rate_code < 3 || cfg.baud_rate_code > 5)
        cfg.baud_rate_code = 4;
    cfg.round_trip_delay_samples = v34_get_round_trip_delay_samples(g_v34);
    /*
     * The upstream carrier is the digital modem's choice, not ours.  §8.2.3.2
     * Table 9 makes INFO1d identical to V.34's INFO1c, and V.34 §10.1.2.3.4
     * has INFO1c's per-symbol-rate block carry the carrier and pre-emphasis
     * for the *answer* modem's transmitter — which here is us, since
     * §9.2.2.1.9 puts the analogue modem in the V.34 answer-modem role.
     *
     * This was hardcoded true, with a comment claiming INFO1d directs the
     * upstream high.  It does not: an Eicon Diva Server asks for the low
     * carrier at 3200 baud, and transmitting the high one puts our whole
     * Phase 3 91 Hz off what its receiver is tuned to (1920 against
     * 1829 Hz).  Its Ja detector then has nothing to find, so §9.3.1.3 never
     * fires and it transmits no Sd at all — measured as 4.5 s of pure 0xFF
     * on the wire while it sat in page 14 (run 56).
     */
    {
        v34_v90_info1d_t info1d;

        cfg.high_carrier = true;
        if (v34_get_v90_received_info1d(g_v34, &info1d)) {
            cfg.high_carrier = info1d.rate_data[cfg.baud_rate_code].use_high_carrier;
            ME_LOG("[ME] V.90 analogue upstream: INFO1d selects the %s carrier "
                   "at symbol-rate code %d (max bit rate code %d)\n",
                   cfg.high_carrier ? "high" : "low", cfg.baud_rate_code,
                   info1d.rate_data[cfg.baud_rate_code].max_bit_rate);
        } else {
            ME_LOG("[ME] V.90 analogue upstream: no INFO1d decoded; defaulting "
                   "to the high carrier at symbol-rate code %d\n",
                   cfg.baud_rate_code);
        }
    }
    /* prepare_v90_info1a() clamps the configured preference to Table 10's
     * U_INFO > 66 and INFO0d power requirements.  The PCM receiver must use
     * the effective value that actually went on the wire. */
    cfg.u_info = v34_get_v90_tx_u_info(g_v34);
    if (cfg.u_info < 67 || cfg.u_info > 127)
        cfg.u_info = 78;
    g_v90a_u_info = cfg.u_info;
    cfg.md_units = 0;                   /* INFO1a announces no MD */
    cfg.digital_max_tx_dbm0 = 0.0;
    {
        v34_v90_info0a_t info0d;

        /* V.90 §8.2.1/Table 7 bits 33:37: code 0 is -0.5 dBm0 and code
         * 31 is -16 dBm0.  §8.5.2/Table 15 requires CP's average power to
         * obey this value; omitting it made clean digital calls offer every
         * measured Ucode at 56 kbit/s regardless of the peer's limit. */
        if (v34_get_v90_received_info0a(g_v34, &info0d)
            && info0d.info0d_extensions_valid) {
            cfg.digital_max_tx_dbm0 = -0.5*(info0d.info0d_max_power_code + 1);
            ME_LOG("[ME] V.90 analogue downstream: INFO0d maximum transmit "
                   "power %.1f dBm0 (code %u, measured at %s)\n",
                   cfg.digital_max_tx_dbm0,
                   (unsigned) info0d.info0d_max_power_code,
                   info0d.info0d_power_measured_at_codec_output
                       ? "codec output" : "digital-modem terminals");
        } else {
            ME_LOG("[ME] V.90 analogue downstream: INFO0d power extension "
                   "unavailable; CP power cap disabled\n");
        }
    }
    cfg.scr_during_dil = parse_env_int("ME_V90_ANALOGUE_SCR", 0) != 0;
    /*
     * §5.4.5's Sr, which this side chooses and Phase 4's CPt/CP carry.  The
     * default is 0 — spectral shaping disabled — because it is the only value
     * that needs nothing agreed with the digital modem: no filter parameters
     * to signal in Table 14 bits 69:101, no ld to keep consistent with Jd, and
     * S = 6, so every one of the six sign bits carries data.  1 to 3 enable
     * shaping and cost rate twice over (§5.4.1: S falls and K rises).
     */
    cfg.shaping_redundancy = parse_env_int("ME_V90_ANALOGUE_SR", 0);
    cfg.shaping_lookahead = parse_env_int("ME_V90_ANALOGUE_LD", 0);
    cfg.upstream_max_n = parse_env_int("ME_V90_ANALOGUE_UPSTREAM_MAX_N", 0);
    if (cfg.upstream_max_n >= 2 && cfg.upstream_max_n <= 14)
        ME_LOG("[ME] V.90 analogue diagnostic upstream ceiling: N=%d (%d bps)\n",
               cfg.upstream_max_n, cfg.upstream_max_n*2400);
    else
        cfg.upstream_max_n = 0;
    cfg.v34 = g_v34;                    /* borrow the modulator Phase 2 configured */
    if (g_v90a_dil_valid)
        cfg.dil = g_v90a_dil;

    g_v90a = v90_analogue_phase3_init(&cfg);
    if (!g_v90a) {
        ME_LOG("[ME] V.90 analogue: Phase 3 failed to start; the call cannot continue\n");
        trace_phase("V90 analogue Phase3 init failed");
        return;
    }
    g_v90a_started = true;
    v90a_linear_free(g_v90a_linear);
    g_v90a_linear = v90a_linear_init(cfg.law);
    ME_LOG("[ME] V.90 analogue Phase 3 started: symbol-rate code %d, %s carrier, U_INFO=%d, "
           "Ja descriptor N=%u (%d bits), RTD=%d samples\n",
           cfg.baud_rate_code, cfg.high_carrier ? "high" : "low",
           g_v90a_u_info, (unsigned) cfg.dil.n,
           v90_analogue_tx_ja_bits(v90_analogue_phase3_tx_state(g_v90a)),
           cfg.round_trip_delay_samples);
    trace_phase("V90 analogue Phase3 start: U_INFO=%d N=%u",
                g_v90a_u_info, (unsigned) cfg.dil.n);
}

/*
 * Follow the analogue Phase 3 along, with g_state_mtx held: log each §9.3.2
 * stage as it is reached, act on the deadlines, and say what the measurement
 * came to when it ends.
 */
static void me_v90_analogue_phase4_progress_locked(void);

static void me_v90_analogue_progress_locked(void)
{
    static v90_analogue_tx_stage_t last_tx = (v90_analogue_tx_stage_t) -1;
    static v90_analogue_rx_stage_t last_rx = (v90_analogue_rx_stage_t) -1;
    const v90_analogue_rx_t *rx;
    v90_analogue_tx_stage_t tx_stage;
    v90_analogue_rx_stage_t rx_stage;

    if (!g_v90a)
        return;

    tx_stage = v90_analogue_phase3_tx_stage(g_v90a);
    rx_stage = v90_analogue_phase3_rx_stage(g_v90a);
    rx = v90_analogue_phase3_rx_state(g_v90a);

    if (tx_stage != last_tx) {
        last_tx = tx_stage;
        ME_LOG("[ME] V.90 analogue TX: %s\n", v90_analogue_tx_stage_name(tx_stage));
        trace_phase("V90a TX %s", v90_analogue_tx_stage_name(tx_stage));
    }
    if (rx_stage != last_rx) {
        last_rx = rx_stage;
        ME_LOG("[ME] V.90 analogue RX: %s (Sd %d reps, S̄d %d reps, TRN1d %dT, "
               "Jd %d frames)\n",
               v90_analogue_rx_stage_name(rx_stage),
               v90_analogue_rx_sd_reps(rx), v90_analogue_rx_sd_bar_reps(rx),
               v90_analogue_rx_trn1d_symbols(rx), v90_analogue_rx_jd_frames(rx));
        trace_phase("V90a RX %s", v90_analogue_rx_stage_name(rx_stage));
    }

    /* §9.3.2.4 and §9.3.2.7 both expire into a retrain (§9.5.2.1), which does
     * not exist for this role yet.  Report it rather than sitting on a dead
     * Phase 3 until the training timeout, so a lab run says what happened. */
    if (v90_analogue_phase3_retrain_due(g_v90a)) {
        if (!g_v90a_retrain_logged) {
            g_v90a_retrain_logged = true;
            ME_LOG("[ME] V.90 analogue: §9.3.2 deadline passed in %s with no answer "
                   "from the digital modem; initiating §9.5.2.1 retrain\n",
                   v90_analogue_tx_stage_name(tx_stage));
            trace_phase("V90a deadline expired in %s",
                        v90_analogue_tx_stage_name(tx_stage));
        }
        if (parse_env_int("ME_V90_ANALOGUE_HOLD", 0) != 0)
            return;
        if (!restart_v90_analogue_phase2_locked("Phase 3 deadline"))
            g_state = ME_HANGUP;
        return;
    }

    /*
     * Phase 4 reporting cannot hang off v90_analogue_phase3_complete(), which
     * is "the transmitter is sitting in V90A_TX_PHASE4" and therefore stops
     * being true the moment CPt starts — i.e. exactly when Phase 4 becomes
     * worth reporting.  Follow it from here instead, unconditionally.
     */
    me_v90_analogue_phase4_progress_locked();

    if (v90_analogue_phase3_phase4_retrain_due(g_v90a)) {
        if (!g_v90a_retrain_logged) {
            g_v90a_retrain_logged = true;
            ME_LOG("[ME] V.90 analogue: §9.4.2 B1d/constellation deadline; initiating §9.5.2.1 retrain\n");
            trace_phase("V90a Phase4 retrain required");
        }
        if (parse_env_int("ME_V90_ANALOGUE_HOLD", 0) == 0
            && !restart_v90_analogue_phase2_locked("Phase 4 failure/deadline"))
            g_state = ME_HANGUP;
        return;
    }

    /* Opt-in trigger for exercising the analogue-initiated half of §9.6.
     * Zero (the default) never initiates autonomously. */
    if (v90_analogue_phase3_data_ready(g_v90a)) {
        int delay_ms = parse_env_int("ME_V90_ANALOGUE_RATE_RENEGOTIATE_MS", 0);

        if (g_v90a_data_start_samples == 0)
            g_v90a_data_start_samples = g_rx_ref_samples;
        if (!g_v90a_rr_triggered && delay_ms > 0
            && g_rx_ref_samples - g_v90a_data_start_samples
               >= (uint64_t) delay_ms*8U) {
            bool silence = parse_env_int(
                "ME_V90_ANALOGUE_RATE_RENEGOTIATE_SILENCE", 0) != 0;

            g_v90a_rr_triggered = true;
            if (v90_analogue_phase3_start_rate_renegotiation(g_v90a, silence)) {
                ME_LOG("[ME] V.90 analogue: initiating §9.6 rate renegotiation%s\n",
                       silence ? " with CPs echo reconditioning" : "");
                trace_phase("V90a initiate rate renegotiation%s",
                            silence ? " CPs" : "");
            } else {
                ME_LOG("[ME] V.90 analogue: could not initiate §9.6 rate renegotiation\n");
            }
        }
    }
    if (g_v90a_cleardown
        && v90_analogue_phase3_cleardown_complete(g_v90a)) {
        ME_LOG("[ME] V.90 analogue: §9.7 cleardown sequence sent; releasing SIP bearer\n");
        trace_phase("V90a cleardown CP complete");
        g_state = ME_HANGUP;
        return;
    }
    if (v90_analogue_phase3_rate_retrain_due(g_v90a)) {
        if (g_v90a_cleardown) {
            ME_LOG("[ME] V.90 analogue: §9.7 cleardown peer did not answer Rd; releasing bearer\n");
            g_state = ME_HANGUP;
            return;
        }
        if (!g_v90a_rr_deadline_logged) {
            g_v90a_rr_deadline_logged = true;
            ME_LOG("[ME] V.90 analogue: §9.6.2 Ed deadline expired; retrain required\n");
            trace_phase("V90a rate renegotiation deadline expired");
        }
        if (parse_env_int("ME_V90_ANALOGUE_HOLD", 0) == 0
            && !restart_v90_analogue_phase2_locked(
                    "rate-renegotiation Ed deadline"))
            g_state = ME_HANGUP;
        return;
    }

    /*
     * Say why Phase 4 did not arm, and say it from here.
     *
     * This used to sit at the end of the block below, which is one-shot on
     * "Phase 3 complete" -- and the handover happens on a *later* receive
     * batch than the one that completes Phase 3, so the flag was always still
     * false when it was read and the message could never print.  Measured
     * live (run 77): a call whose measurement yielded no constellation logged
     * nothing at all between "DIL measured" and the hold, which is the one
     * case where the engine most needs to speak.
     */
    if (v90_analogue_phase3_phase4_failed(g_v90a)  &&  !g_v90a_failed_logged) {
        g_v90a_failed_logged = true;
        ME_LOG("[ME] V.90 analogue: the measurement yielded no constellation "
               "§8.5.2 would let us offer; Phase 4 cannot start (§9.4.2.1 has "
               "nothing to put in a CPt)\n");
        trace_phase("V90a Phase4 build failed");
    }
    /*endif*/

    if (!v90_analogue_phase3_complete(g_v90a)  ||  g_v90a_complete_logged)
        return;
    g_v90a_complete_logged = true;

    {
        const v90_dil_measurement_t *m = v90_analogue_phase3_measurement(g_v90a);
        v90_dil_rate_plan_t plan;

        ME_LOG("[ME] V.90 analogue Phase 3 complete: Sd %d reps, S̄d %d reps, "
               "TRN1d %dT, %d Jd frames (%s-point), DIL %d symbols\n",
               v90_analogue_rx_sd_reps(rx), v90_analogue_rx_sd_bar_reps(rx),
               v90_analogue_rx_trn1d_symbols(rx), v90_analogue_rx_jd_frames(rx),
               v90_analogue_rx_jd_trn16(rx) ? "16" : "4",
               v90_analogue_rx_dil_symbols(rx));
        if (m) {
            ME_LOG("[ME] V.90 analogue DIL measured: %d Ucodes, %d usable, "
                   "gain %.2f dB, RBS slots 0x%02X, coverage %.0f%%\n",
                   m->ucodes_measured, m->usable_count, m->gain_db,
                   m->rbs_slot_mask, 100.0*m->coverage);
            /* What the line would actually carry.  This is the whole point of
               taking the analogue role: the constellation decision is ours. */
            if (v90_dil_measure_plan_rate(m, 0, 3.0, 0.0,
                                          (g_law == ME_LAW_ALAW) ? V90_LAW_ALAW
                                                                 : V90_LAW_ULAW,
                                          &plan)) {
                ME_LOG("[ME] V.90 analogue constellation: Mi = %d %d %d %d %d %d, "
                       "drn=%u, %.0f bps%s%s\n",
                       plan.mi[0], plan.mi[1], plan.mi[2], plan.mi[3],
                       plan.mi[4], plan.mi[5],
                       (unsigned) plan.drn, plan.bps,
                       plan.robbed_bit_limited ? " (robbed-bit limited)" : "",
                       plan.noise_limited ? " (noise limited)" : "");
            } else {
                v90_dil_rate_plan_t raw;
                bool any;

                /*
                 * No constellation at the 3-sigma margin.  Say what the line
                 * would carry without it, because the two answers point at
                 * different problems: a measurement that yields nothing either
                 * way is a bad DIL or a bad receiver, while one that yields a
                 * rate at 0 sigma and nothing at 3 is a noisy line meeting a
                 * margin policy with no floor.  Run 77 was the second, and
                 * logged neither.
                 */
                any = v90_dil_measure_plan_rate(m, 0, 0.0, 0.0,
                                                (g_law == ME_LAW_ALAW)
                                                    ? V90_LAW_ALAW
                                                    : V90_LAW_ULAW,
                                                &raw);
                ME_LOG("[ME] V.90 analogue constellation: none at a 3-sigma "
                       "noise margin; ignoring noise it would be %s\n",
                       any ? "offerable" : "still empty");
                if (any) {
                    ME_LOG("[ME] V.90 analogue constellation (0 sigma): "
                           "Mi = %d %d %d %d %d %d, drn=%u, %.0f bps\n",
                           raw.mi[0], raw.mi[1], raw.mi[2], raw.mi[3],
                           raw.mi[4], raw.mi[5], (unsigned) raw.drn, raw.bps);
                }
                /*endif*/
            }
        } else {
            ME_LOG("[ME] V.90 analogue: Phase 3 ended with no DIL measurement\n");
        }
        trace_phase("V90a Phase3 complete");
    }

}

/*
 * Follow §9.4 along, with g_state_mtx held.  Phase 4's transmit stages come
 * from the same transmitter Phase 3 used, so they are already reported by the
 * TX line above; what is new is the receive side and, at the end, what the
 * digital modem said in MP.
 */
static void me_v90_analogue_phase4_progress_locked(void)
{
    static v90_analogue_phase4_rx_stage_t last = (v90_analogue_phase4_rx_stage_t) -1;
    static int last_mp_frames = -1;
    static bool started_logged = false;
    const v90_analogue_phase4_t *p4;
    v90_analogue_phase4_rx_stage_t stage;

    if (!g_v90a  ||  (p4 = v90_analogue_phase3_phase4_state(g_v90a)) == NULL)
        return;

    if (!started_logged) {
        const vpcm_cp_frame_t *cpt = v90_analogue_phase3_cpt(g_v90a);
        const vpcm_cp_frame_t *cp = v90_analogue_phase3_cp(g_v90a);

        started_logged = true;
        if (cpt  &&  cp) {
            /* K is the number the digital modem's mapper is built from, so it
             * is the one worth logging beside the rate: Table 17 caps a CPt's
             * at 24 and §5.4.3 caps both at log2(prod(Mi)). */
            ME_LOG("[ME] V.90 analogue Phase 4 started (§9.4.2.1): "
                   "CPt drn=%u (%d bits/frame, K=%d), CP drn=%u "
                   "(%d bits/frame, K=%d, %.0f bps), Sr=%u, ld=%u, %s\n",
                   cpt->drn, cpt->drn + 8, v90_analogue_phase4_cp_k(cpt),
                   cp->drn, cp->drn + 20, v90_analogue_phase4_cp_k(cp),
                   vpcm_cp_drn_to_bps(cp->drn),
                   cp->shaping_redundancy, cp->shaping_lookahead,
                   cp->codec_alaw ? "A-law" : "u-law");
            ME_LOG("[ME] V.90 analogue Phase 4 constellations: "
                   "CPt Mi=%d/%d/%d/%d/%d/%d; CP Mi=%d/%d/%d/%d/%d/%d\n",
                   vpcm_cp_mask_population(cpt->masks[cpt->dfi[0]]),
                   vpcm_cp_mask_population(cpt->masks[cpt->dfi[1]]),
                   vpcm_cp_mask_population(cpt->masks[cpt->dfi[2]]),
                   vpcm_cp_mask_population(cpt->masks[cpt->dfi[3]]),
                   vpcm_cp_mask_population(cpt->masks[cpt->dfi[4]]),
                   vpcm_cp_mask_population(cpt->masks[cpt->dfi[5]]),
                   vpcm_cp_mask_population(cp->masks[cp->dfi[0]]),
                   vpcm_cp_mask_population(cp->masks[cp->dfi[1]]),
                   vpcm_cp_mask_population(cp->masks[cp->dfi[2]]),
                   vpcm_cp_mask_population(cp->masks[cp->dfi[3]]),
                   vpcm_cp_mask_population(cp->masks[cp->dfi[4]]),
                   vpcm_cp_mask_population(cp->masks[cp->dfi[5]]));
            trace_phase("V90a Phase4 start: CPt drn=%u CP drn=%u",
                        cpt->drn, cp->drn);
        }
        /*endif*/
    }
    /*endif*/

    stage = v90_analogue_phase4_stage(p4);
    if (stage != last) {
        last = stage;
        ME_LOG("[ME] V.90 analogue Phase 4 RX: %s (Ri %dT, TRN2d %dT/%d ones, "
               "MP %d frames, B1d %d/48 frames with %d bit errors, "
               "%d demap failures)\n",
               v90_analogue_phase4_stage_name(stage),
               v90_analogue_phase4_r_symbols(p4),
               v90_analogue_phase4_trn2d_symbols(p4),
               v90_analogue_phase4_trn2d_ones(p4),
               v90_analogue_phase4_mp_frames(p4),
               v90_analogue_phase4_b1d_frames(p4),
               v90_analogue_phase4_b1d_bit_errors(p4),
               v90_analogue_phase4_demap_failures(p4));
        trace_phase("V90a P4 RX %s", v90_analogue_phase4_stage_name(stage));
    }
    /*endif*/
    if (v90_analogue_phase4_mp_frames(p4) != last_mp_frames) {
        const v90_analogue_mp_t *mp = v90_analogue_phase4_mp(p4);

        last_mp_frames = v90_analogue_phase4_mp_frames(p4);
        if (mp) {
            /* §9.4.2.4: the upstream rate is the maximum enabled in both
             * modems and no more than MP's cap. */
            ME_LOG("[ME] V.90 analogue MP: Type %d, max upstream drn=%u (%d bps), "
                   "trellis=%u, rate mask 0x%04X, %s\n",
                   mp->type1 ? 1 : 0, mp->max_drn, mp->max_drn*2400,
                   mp->trellis, mp->rate_mask,
                   mp->acknowledge ? "MP' (acknowledged)" : "MP");
        }
        /*endif*/
    }
    /*endif*/
}

/* Called with g_state_mtx held. */
static void prepare_v90_phase3_locked(void)
{
    v90_info1a_t info1a;
    bool v92_selected = false;

    if (g_mod != ME_MOD_V90 || !g_v34 || g_v90_phase3_started)
        return;
    /* The digital transmitter has no business in a call that told the peer it
     * was the analogue modem.  It would not get far — this waits on an INFO1a
     * that the analogue role sends rather than receives — but the guard states
     * the intent instead of relying on that. */
    if (me_v90_analogue_role())
        return;

    v92_refresh_info0_confirmation_locked();
    if (get_strict_v90_info1a_locked(&info1a, &v92_selected)) {
        const char *dil_profile = getenv("ME_V90_DIL_PROFILE");

        if (g_v92_info0_mutual && !v92_selected)
            ME_LOG("[ME] V.92 INFO0 was mutual but the peer answered a V.90 INFO1a "
                   "(upstream_code=%u, not 6/8000); demoting to V.90\n",
                   (unsigned)info1a.upstream_symbol_rate_code);
        g_v92_active = v92_selected;
        ME_LOG("[ME] %s strict RX event: valid INFO1a U_INFO=%u MD=%u upstream_code=%u downstream_code=%u\n",
               g_v92_active ? "V.92" : "V.90",
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
            if (g_v90 && g_v90_phase2_restarts > 0) {
                const char *configured = getenv("ME_V90_SD_DELAY_RETRAIN_MS");
                int retrain_delay;

                /* §9.3.1.3 permits the digital modem to wait 0..500 ms after
                 * detecting Ja before Sd.  SmartLink's WaitForSd acquisition
                 * has a narrow, attempt-dependent window: repeating the
                 * initial ME_V90_SD_DELAY_MS on every retrain produced three
                 * identical misses live (2026-08-11).  Probe the legal window
                 * deterministically on successive retrains unless an explicit
                 * fixed interoperability value was requested. */
                if (configured && *configured) {
                    retrain_delay = parse_env_int("ME_V90_SD_DELAY_RETRAIN_MS", 0);
                    if (retrain_delay < 0)
                        retrain_delay = 0;
                    if (retrain_delay > 500)
                        retrain_delay = 500;
                    ME_LOG("[ME] V.90: retrained attempt %u; fixed Sd delay %d ms "
                           "(ME_V90_SD_DELAY_RETRAIN_MS)\n",
                           g_v90_phase2_restarts, retrain_delay);
                } else {
                    /* The peer normally permits two V.90 retries before its
                     * third failure drops to V.34.  With Ja now gated to the
                     * current strong Table 18 match, try the immediate legal
                     * transition first, then the middle and late windows. */
                    static const int legal_delays_ms[] = {0, 250, 500};
                    unsigned int index = (g_v90_phase2_restarts - 1U)
                                         % (sizeof(legal_delays_ms)/sizeof(legal_delays_ms[0]));

                    retrain_delay = legal_delays_ms[index];
                    ME_LOG("[ME] V.90: retrained attempt %u; adaptive §9.3.1.3 Sd delay %d ms\n",
                           g_v90_phase2_restarts, retrain_delay);
                }
                /* Set zero explicitly.  Leaving the override unset here used
                 * to inherit ME_V90_SD_DELAY_MS, despite the old log/comment
                 * claiming that the retrained-attempt default was zero. */
                v90_set_sd_delay_ms(g_v90, retrain_delay);
            }
        }
        if (g_v90) {
            if (g_v92_active) {
                v90_enable_v92_phase3(g_v90);
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
            if (g_v90_phase3_first_samples == 0)
                g_v90_phase3_first_samples = g_rx_audio_samples;
            g_v90_completion_deferred_logged = false;
            g_v90_wait_info1_logged = false;
            g_v90_reject_info1a_logged = false;
            if (g_v92_active) {
                v92_p3_rx_init(&g_v92_p3_rx);
                v92_p3_rx_start(&g_v92_p3_rx, (int)g_g711_rx_octets);
                /* INFO1a codes MD in units of 35 ms (v34rx.c prints md*35);
                   at 8000 sym/s that is md*35*8 symbols.  MD = 0 is the
                   common case and selects the V.92 9.5.1.1.1 short flow --
                   without this the receiver always took the MD-bearing path
                   and burned its 8000-sample MD timeout waiting for a second
                   Ru/uR pair the peer never sends. */
                v92_p3_rx_set_md_length(&g_v92_p3_rx, (int)info1a.md * 35 * 8);
                g_v92_p3_rx_active = true;
                g_v92_p3_rx_result_applied = false;
                g_v92_p3_rx_failure_logged = false;
                g_v92_p3_cpt_active = false;
                memset(&g_v92_p3_cpt_demod, 0, sizeof(g_v92_p3_cpt_demod));
                v92_cp_rx_reset(&g_v92_p3_cpt_rx);
                v92_su_rx_reset_locked();
                trace_phase("V92 strict RX event=INFO1A_VALID u_info=%u -> Phase3 Ru/TRN1u/Ja",
                            (unsigned)info1a.u_info);
                ME_LOG("[ME] V.92 Phase 3 raw receiver armed at G.711 sample %llu\n",
                       (unsigned long long)g_g711_rx_octets);
            } else {
                trace_phase("V90 strict RX event=INFO1A_VALID u_info=%u -> Phase3",
                            (unsigned)info1a.u_info);
            }
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
        /* SpanDSP exposes the INFO1a snapshot as soon as V.90 mode is
           enabled, before a CRC-valid INFO1a has populated it.  A zero
           U_INFO is that reset snapshot, not a peer selecting plain V.34. */
        if (v34_get_v90_received_info1a(g_v34, &received)
            && received.u_info > 0) {
            /* Own log-once flag, not the "still waiting" one: that has almost
               always fired by now, which silently swallowed the only line that
               names the field the validator objected to. */
            if (!g_v90_reject_info1a_logged) {
                ME_LOG("[ME] V.90 strict RX event: rejecting INFO1a reserved=%02X/%02X U_INFO=%d upstream_code=%d downstream_code=%d (v92_contract=%d)\n",
                       received.raw_12_17,
                       received.raw_32_33,
                       received.u_info,
                       received.upstream_symbol_rate_code,
                       received.downstream_rate_code,
                       g_v92_info0_mutual ? 1 : 0);
                trace_phase("V90 strict RX event=INFO1A_INVALID -> remain Phase2");
                g_v90_reject_info1a_logged = true;
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
    /* V.92 §9.6.1.1.6 does not expose carrier/data until B1u has conditioned
     * the PCM-upstream receiver.  Downstream B1d can finish first. */
    if (g_v92_active
        && (!g_v92_upstream_rx_active || !g_v92_upstream_rx.locked))
        return;
    upstream_rate = g_v92_active
        ? ((int)g_v92_upstream_rx.cpd.selected_upstream_drn + 17)*8000/6
        : v34_get_current_bit_rate(g_v34);
    downstream_rate = (v90_data_bits_per_frame(g_v90) * 8000) / 6;
    if (downstream_rate <= 0)
        downstream_rate = V90_RATE_BPS;
    if (g_retrain_from_data) {
        /* §9.5/§11.5 retrain: the physical layer has re-trained under a link
           that never went down.  Re-arming the data stack here would restart
           LAPM against a peer still in the middle of its own connection, so
           only the rate is refreshed. */
        g_retrain_from_data = false;
        g_data_connect_rate = downstream_rate;
        ME_LOG("[ME] V.90 retrain complete; resuming the existing data link "
               "(downstream %d bps)\n", downstream_rate);
    } else {
        data_stack_start_online(downstream_rate, g_calling_party);
    }
    g_state = ME_DATA;
    g_phase_start_ms = 0;
    /* Shared DATA-entry epoch for the §9.5/§11.5 disruption probe.  Plain
       V.34 set it at its handover, but the native V.90 handover did not, so
       ME_TX_DISRUPT_AFTER_MS could never exercise a V.90 peer response. */
    g_v34_data_entry_ms = trace_now_ms();
    g_v34_data_entry_samples = g_rx_audio_samples;
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    ME_LOG("[ME] %s startup complete (upstream %s %d bps, downstream PCM %d bps)\n",
           g_v92_active ? "V.92" : "V.90",
           g_v92_active ? "PCM" : "V.34",
           upstream_rate, downstream_rate);
    trace_phase("%s enter DATA after B1: upstream=%d downstream=%d",
                g_v92_active ? "V92" : "V90",
                upstream_rate, downstream_rate);
    if (g_data_connect_reported) {
        /* A retrain, not a new connection: the DTE was told CONNECT once and
           §11.5 never takes it back -- it only clamps 104 for the duration. */
    } else if (g_data_framing != DS_FRAMING_V42) {
        g_data_connect_reported = true;
        di_on_connected(downstream_rate);
    } else {
        ME_LOG("[ME] V.90 carrier ready; waiting for V.42 LAPM\n");
    }
}

/* At the V.90 DIL -> Ri boundary, V.90 owns the downstream transmitter.
 * SpanDSP is still needed for the V.34 upstream receiver, but its generic
 * Phase 4 handoff also starts the V.34 S/S-bar/TRN transmitter.  That is not
 * part of the V.90 Phase 4 sequence: the answerer sends Ri, then TRN2d. */
static void enter_v90_phase4_rx_locked(void)
{
    if (!g_v90 || !g_v34)
        return;

    g_v90_jd_resync_retries = 0;
    ME_LOG("[ME] %s Phase 3 complete; enabling native upstream Phase 4 receiver\n",
           g_v92_active ? "V.92" : "V.90");
    {
        int baud = v90_selected_upstream_baud_locked();
        int limit = v34_get_current_bit_rate(g_v34);
        int baud_limit = v90_upstream_baud_max_bps(baud);

        if (limit > baud_limit)
            limit = baud_limit;
        /* The peer picks its upstream rate from the mask our MP offers, and
         * at 31200 the symbols arrive with 3-4 units of error against a
         * constellation spacing of 2 -- too dense for this path to decode. */
        limit = me_v90_upstream_cap(limit);
        v90_set_upstream_rate_limit(g_v90, limit);
        ME_LOG("[ME] V.90 upstream selection: %d baud, rate cap %d bps, %s carrier\n",
               baud == 3 ? 3000 : 3200, limit,
               v34_get_rx_high_carrier(g_v34) ? "high" : "low");
    }
    if (!g_v92_active) {
        v90_cp_live_note_phase4_hint_locked();
        v34_force_v90_phase4_cp_rx(g_v34);
    } else {
        v34_force_phase4(g_v34);
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
        if (!g_v90_phase3_started || !g_v90)
            return false;

        while (pos < len && v90_get_tx_phase(g_v90) != V90_TX_DATA) {
            v90_tx_phase_t phase_before = v90_get_tx_phase(g_v90);

            if (v90_phase3_tx_codewords(g_v90, codewords + pos, 1) != 1)
                return false;
            pos++;

            /* §9.3.2.4/§9.3.2.7 make the analogue modem silent for exactly as
               long as we are sending Jd, so energy there means it has left
               V.90.  DIL is excluded: §9.3.2.9 lets it send SCR. */
            if (phase_before != v90_get_tx_phase(g_v90))
                v34_v90_set_phase3_expect_silence(
                    g_v34, v90_get_tx_phase(g_v90) == V90_TX_JD);
            /*endif*/

            if (v90_get_tx_phase(g_v90) == V90_TX_JP
                && g_v92_su_final_pending) {
                /* The final S-bar-u transition can arrive during the last
                 * Jd frame.  Preserve it until Jp is live, then terminate
                 * Jp at its next frame boundary as §9.5.1.1.9 requires. */
                if (v90_handle_rx_event(g_v90, V90_RX_EVENT_SU_FINAL))
                    g_v92_su_final_pending = false;
            }

            if (phase_before == V90_TX_JP_PRIME
                && (v90_get_tx_phase(g_v90) == V90_TX_DIL
                    || v90_get_tx_phase(g_v90) == V90_TX_SCR)) {
                v92_start_phase3_cpt_rx_locked();
            }

            /* The project-owned V.90 transmitter replaces SpanDSP's Phase 3
               waveform, so SpanDSP cannot observe the downstream DIL -> Ri
               boundary itself.  Hand its primary-channel receiver into the
               native Phase 4 path at that exact boundary; otherwise it stays
               parked in PHASE3_WAIT_S and never delivers the peer's CPt bits
               to v90_live_cp_bit(). */
            if (phase_before < V90_TX_RI
                && v90_get_tx_phase(g_v90) >= V90_TX_RI) {
                enter_v90_phase4_rx_locked();
            }

            /* v90.c returns to WAIT_JA after a bounded Jd-without-S interval.
               That state change is the recovery request for the live engine:
               re-enter the V.34/V.90 Phase-2 control exchange immediately,
               instead of sending silence while the peer expects INFO0d. */
            if (phase_before == V90_TX_JD
                && v90_get_tx_phase(g_v90) == V90_TX_WAIT_JA) {
                /* Not every Jd-without-S means the peer went back to Phase 2.
                 * Measured against slmodemd, the common case is that it is
                 * sitting in WaitForSd waiting for an Sd it missed, and will
                 * give up in about 2 s -- and restarting Phase 2 here
                 * guarantees it never gets one, which its own log shows as
                 * "Error Energy = -0.000" right up to the retrain.  Let
                 * v90.c hold its short silence and re-emit Sd once; only fall
                 * back to the Phase-2 exchange if that is ignored too. */
                if (g_v90_jd_resync_retries < 1) {
                    g_v90_jd_resync_retries++;
                    ME_LOG("[ME] V.90: no S after Jd; re-emitting Sd in place "
                           "(peer may still be in WaitForSd)\n");
                } else {
                    (void) restart_v90_phase2_locked("no S after Jd");
                    return false;
                }
            }
        }

        if (v90_get_tx_phase(g_v90) == V90_TX_DATA) {
            enter_v90_data_locked();
            if (pos < len)
                generate_v90_data_codewords_locked(codewords + pos, len - pos);
        }
        return true;
    }

    if (g_state == ME_DATA) {
        int pos = 0;

        if (!g_v90)
            return false;

        /* §9.6.1: a rate renegotiation that does not produce an E is
         * abandoned for a full retrain.  The engine owns the retrain, so it
         * is the engine that acts on the flag. */
        /* Report the window on the way out either way: a renegotiation that
         * completed and one that timed out differ by exactly one frame, and
         * only logging the failures hides that. */
        if (g_v90_reneg_cp_rx_marked
            && !v90_rate_renegotiation_active(g_v90)
            && !v90_rate_renegotiation_timed_out(g_v90))
            v90_reneg_cp_report_locked("complete");

        if (v90_rate_renegotiation_timed_out(g_v90)) {
            v90_reneg_cp_report_locked("timeout");
            (void) restart_v90_phase2_locked("rate renegotiation timeout");
            return false;
        }

        /* ME_V90_RENEG_AFTER_MS test hook: open a §9.6 renegotiation on a
         * HEALTHY call, which is the only way to see what this peer does
         * with Rd.  Deliberately not gated on me_v90_reneg_enabled(): the
         * point of the probe is to answer the question that knob's default
         * rests on. */
        if (v90_reneg_probe_due_locked()
            && !v90_rate_renegotiation_active(g_v90)
            && !v90_rate_renegotiation_pending(g_v90)) {
            g_v90_reneg_probe_done = true;
            ME_LOG("[ME] V.90: ME_V90_RENEG_AFTER_MS probe; requesting a "
                   "§9.6 rate renegotiation\n");
            trace_phase("V90 reneg probe");
            (void) v90_request_rate_renegotiation(g_v90);
        }

        /* A receiver that was settled and then stepped abruptly off the
         * constellation gets one bounded local timing search.  If that
         * cannot find a viable solution, do not wait for the one-second
         * carrier-loss timer: by then the analogue modem may no longer
         * decode Rd.  V.90 §9.6 permits renegotiation at any time in DATA,
         * requires both sides to preserve data-frame synchronization, and
         * starts the actual signal on the next data-frame boundary below.
         * V.34 §11.6 explicitly defines this exchange as receiver
         * resynchronization: training, MP/E, B1, then a new superframe.
         *
         * This narrow trigger is mandatory protocol recovery, independent
         * of ME_V90_RENEG, which only governs the broad chronic-loss policy.
         * It is bounded by the same per-call cap so a bad line cannot spend
         * the whole call renegotiating. */
        if (g_v34 && v34_v90_upstream_resync_required(g_v34)
            && !v90_rate_renegotiation_active(g_v90)
            && !v90_rate_renegotiation_pending(g_v90)) {
            if (v90_rate_renegotiation_count(g_v90)
                    < ME_V90_MAX_RENEGOTIATIONS) {
                ME_LOG("[ME] V.90 upstream discontinuity; requesting early "
                       "§9.6/§11.6 training-and-B1 resynchronization\n");
                trace_phase("V90 upstream discontinuity resync");
                (void) v90_request_rate_renegotiation(g_v90);
            } else {
                ME_LOG("[ME] V.90 upstream discontinuity; §9.6 per-call cap "
                       "reached\n");
            }
            v34_v90_upstream_clear_resync_required(g_v34);
            v34_v90_upstream_clear_carrier_lost(g_v34);
        }

        /* Has the upstream lost carrier?  §9.6 allows a rate renegotiation
         * "at any time during data mode", and it is the only recovery with
         * the reach this needs: it ends in a fresh B1 from the analogue
         * modem, which is what our upstream receiver acquires against.
         * Nudging the receiver's existing state does not work -- measured
         * twice, no sampling offset within half a symbol and no rotation
         * recovers the constellation after one of this peer's one-sample
         * timing slips (docs/v90_upstream_data_path.md). */
        if (g_v34 && v34_v90_upstream_carrier_lost(g_v34)
            && !v90_rate_renegotiation_active(g_v90)
            && !v90_rate_renegotiation_pending(g_v90)) {
            if (me_v90_reneg_enabled()
                && v90_rate_renegotiation_count(g_v90) < ME_V90_MAX_RENEGOTIATIONS) {
                ME_LOG("[ME] V.90 upstream carrier lost; requesting a §9.6 "
                       "rate renegotiation to re-acquire\n");
                (void) v90_request_rate_renegotiation(g_v90);
                /* Clear it: at the cap, the condition would otherwise be
                 * re-raised on every block for the rest of the call. */
                v34_v90_upstream_clear_carrier_lost(g_v34);
            } else if (retrain_on_loss_due(me_v90_max_loss_retrains())) {
                /* §9.5.1.1.  The renegotiation above is the cheaper of the
                 * two recoveries and is preferred where the peer implements
                 * §9.6.2.  ("This rig's analogue modem answers Rd with
                 * nothing at all" used to stand here; it was our own
                 * unterminated Rd -- see me_v90_reneg_enabled().)  With §9.6
                 * unavailable the alternative to
                 * a retrain is not "leave the link alone" -- it is to keep
                 * transmitting downstream into a receiver whose eye is shut
                 * for the rest of the call.  §9.5 puts a retrain at any time,
                 * and it ends where the upstream receiver can actually start
                 * again: a fresh B1 off a fresh Phase 4.
                 *
                 * retrain_on_loss_due() carries the bounds -- a per-call
                 * cap and a dwell since the last one -- so a line that is
                 * simply too poor degrades to the old behaviour of leaving it
                 * alone rather than retraining in a loop. */
                v34_v90_upstream_clear_carrier_lost(g_v34);
                ME_LOG("[ME] V.90 upstream carrier lost and §9.6 is not "
                       "available; initiating a §9.5.1.1 retrain (%u of %d)\n",
                       g_loss_retrains + 1, me_v90_max_loss_retrains());
                g_loss_retrains++;
                g_last_loss_retrain_ms = trace_now_ms();
                (void) restart_v90_phase2_locked(
                    "upstream carrier lost in data mode; retraining "
                    "per 9.5.1.1");
                return false;
            } else {
                v34_v90_upstream_clear_carrier_lost(g_v34);
            }
        }

        /* §9.6: "Rate renegotiation shall be initiated by the digital modem's
         * transmitter only on the boundary of a data frame."  This is that
         * boundary -- the point at which the next frame would be mapped. */
        if (v90_rate_renegotiation_pending(g_v90)
            && g_v90_data_frame_pos >= V90_DATA_FRAME_LEN) {
            if (v90_rate_renegotiation_start(g_v90)) {
                /* §9.6.1.1.1: "condition its receiver to detect S, S-bar,
                 * and CP".  Figure 8/V.90 is why all three are named: the
                 * analogue modem answers Rd with S for 128T, S-bar for 16T
                 * and an optional SCR of up to 2000 ms, and only THEN sends
                 * CP.  enter_v90_phase4_rx_locked() is the STARTUP
                 * conditioning and goes straight to the CP search, which is
                 * right there -- after DIL the peer begins repeated CPt at
                 * once and startup Phase 4 contains no S at all -- but here
                 * it leaves the receiver hunting CP through the peer's S, so
                 * the S-to-S-bar transition that re-anchors the frame goes by
                 * unnoticed.  Measured (artifacts/reneg-mp): the peer's S is
                 * on the wire 258 ms after our R-bar-d and 40 ms long -- 128T
                 * at 3200 baud, §9.6.2.1.1 to the symbol -- with 98.7% of the
                 * block energy in §10.1.3.7's three bins, and we logged no S
                 * detection of any kind. */
                enter_v90_phase4_rx_locked();
                if (!g_v92_active) {
                    /* Leave the receiver in DATA and watch for S spectrally.
                     * The constellation detector in V34_RX_STAGE_PHASE4_S
                     * declared S on ordinary data-mode symbols 140 ms after
                     * being armed, 170 ms before the peer's real S was on the
                     * wire, and the CP search then ran straight through the
                     * transition it exists to find. */
                    v34_v90_watch_reneg_s(g_v34, 1);
                    g_v90_reneg_await_s = true;
                    ME_LOG("[ME] V.90 §9.6.1.1.1: watching for the peer's S "
                           "answer to Rd before the CP search\n");
                    trace_phase("V90 reneg watching for peer S");
                }
                /* The analogue modem's answer ends in a fresh E/B1, so let
                 * the upstream receiver acquire it exactly as it did at
                 * startup rather than carrying the state that just failed.
                 * Reset STARTED as well as ARMED: §9.6.1.2.3 puts E/B1 after
                 * CP', and v90_live_cp_bit() deliberately ignores E while
                 * STARTED remains true.  Leaving the pre-renegotiation value
                 * set completed the downstream exchange but stranded the
                 * upstream receiver forever in V90_CP. */
                v90_reset_upstream_data_arming();
                if (g_v34)
                    v34_v90_upstream_clear_resync_required(g_v34);
                if (g_v34)
                    v34_v90_upstream_clear_carrier_lost(g_v34);
            }
        }

        /* §9.6.1.1.1's hand-off: the receiver was conditioned to detect the
         * peer's S and the S-to-S-bar transition; once that has gone by,
         * Figure 8/V.90 puts an optional SCR of up to 2000 ms and then CP on
         * the wire, so switch to the CP search.  v34rx advances PHASE4_S to
         * PHASE4_TRN at the junction because that is where plain V.34 §11.6
         * goes; in V.90 the same junction is where CP begins. */
        if (g_v90_reneg_await_s && g_v34
            && v34_get_rx_event(g_v34) == V34_EVENT_PEER_RENEG_S) {
            g_v90_reneg_await_s = false;
            v34_v90_watch_reneg_s(g_v34, 0);
            ME_LOG("[ME] V.90 §9.6.1.1.1: peer S answer detected; "
                   "conditioning the receiver for CP\n");
            trace_phase("V90 reneg S answer seen, CP search");
            v34_v90_force_reneg_cp_rx(g_v34);
            v90_reneg_cp_mark_locked();
            /* The peer is answering, so this attempt gets §9.6.1's full E
             * window rather than the unanswered-attempt bound. */
            v90_rate_renegotiation_note_answer(g_v90);
        }

        /* While the renegotiation runs, the downstream is Rd/R̄d/TRN2d/MP/Ed/
         * B1d, which is the Phase 4 transmitter, not the data mapper. */
        if (v90_rate_renegotiation_active(g_v90)
            && !v90_rate_renegotiation_pending(g_v90)) {
            while (pos < len && v90_get_tx_phase(g_v90) != V90_TX_DATA) {
                if (v90_phase3_tx_codewords(g_v90, codewords + pos, 1) != 1)
                    return false;
                pos++;
            }
            if (pos < len)
                generate_v90_data_codewords_locked(codewords + pos, len - pos);
            return true;
        }

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
    /* Fill unconditionally: the NLMS canceller below is optional (g_echo_can),
       but the Phase 3 far-end-S echo gate needs this TX reference on every
       call, including the ones where the canceller is not allocated. */
    for (int i = 0; i < len; i++) {
        g_tx_buf[g_tx_buf_wr] = amp[i];
        g_tx_buf_wr = (g_tx_buf_wr + 1) & TX_BUF_MASK;
    }
    pthread_mutex_unlock(&g_state_mtx);
}

void me_tx_audio(int16_t *amp, int len)
{
    if (di_fax_active() && !me_v34_fax_probe()) {
        di_fax_tx(amp, len);
        return;
    }

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
        } else if (g_mod == ME_MOD_V90 && me_v90_analogue_role()) {
            pthread_mutex_lock(&g_state_mtx);
            if (g_v34) {
                /* Take the modulator over at the Phase 2/3 seam (§9.3.2.1). */
                prepare_v90_analogue_phase3_locked();
                if (g_v90a_started && g_v90a) {
                    v90_analogue_phase3_tx(g_v90a, amp, len);
                    me_v90_analogue_progress_locked();
                } else {
                    /* Still in Phase 2: SpanDSP is sending INFO0a/INFO1a. */
                    v34_tx(g_v34, amp, len);
                }
            }
            pthread_mutex_unlock(&g_state_mtx);
        } else if (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) {
            pthread_mutex_lock(&g_state_mtx);
            if ((g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) && g_v34) {
                /* V.90: detect Phase 2→3 and create the raw-codeword state. */
                prepare_v90_phase3_locked();

                if (g_v90_phase3_started && g_v90) {
                    /* V.90 Phase 3: generate PCM codewords for actual RTP output */
                    v90_tx_phase_t phase_before = v90_get_tx_phase(g_v90);
                    v90_phase3_tx(g_v90, amp, len);
                    if (phase_before < V90_TX_RI
                        && v90_get_tx_phase(g_v90) >= V90_TX_RI)
                        enter_v90_phase4_rx_locked();
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

                        /* Commit the modulation, not just the waveform.  Emitting
                           the V.34 Phase 3/4 signal while g_mod stayed V90 could
                           never complete: v34_put_bit_cb() gates ME_DATA entry on
                           v90_training_complete(), which can only be true if V.90
                           startup ran — so training success logged "remaining in
                           TRAINING" and the call ran out TRAINING_TIMEOUT_MS into
                           the V.22bis fallback.  Had it entered ME_DATA anyway,
                           the tx dispatch keys on g_mod and would have put V.90
                           PCM codewords on a V.34 link.  Switching here routes
                           both through the plain-V.34 paths that already work
                           when V.8 selects V.34 outright.  Every other V.90-only
                           site is Phase 3/4 machinery we no longer want or a
                           diagnostic, and each is guarded on g_mod == ME_MOD_V90,
                           so this one assignment retires them all — including
                           prepare_v90_phase3_locked(), which no-ops from the next
                           call on. */
                        g_mod = ME_MOD_V34;

                        /* The 1200 Hz notch belongs to V.90's Phase 2 CC echo
                           removal.  For plain V.34 at 3200 baud it sits at
                           neither carrier (1829/1920 Hz), so it is pure loss in
                           the RX path.  start_v34_training() disables the notch
                           at this baud because the 91 Hz separation is too narrow
                           to filter; restore that decision.  The NLMS echo
                           canceller stays — it is wideband and still earns its
                           keep against the FXS hybrid. */
                        g_notch.active = false;
                        ME_LOG("[ME] Notch filter disabled: V.90 Phase 2 CC notch does not apply to plain V.34\n");
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
                if (tx_dump) {
                    /* The TX dump is written from the transmit callback and
                     * the RX dump from the receive callback: different call
                     * sites, different rates, and the two files end up
                     * different lengths for the same call.  So TX needs its
                     * own marks -- indexing this file with the RX marks silently
                     * reads the wrong instant, which is exactly the error that
                     * produced a confident "our transmitter is silent during
                     * the stall" from an 8 s misalignment. */
                    static long tx_dump_total = 0;
                    static long tx_dump_marked = 0;

                    fwrite(amp, sizeof(int16_t), len, tx_dump);
                    tx_dump_total += len;
                    if (tx_dump_total - tx_dump_marked >= 800) {
                        tx_dump_marked = tx_dump_total;
                        ME_LOG("[ME] TX dump mark: sample=%ld rx_stage=%d tx_stage=%d mod=%d\n",
                               tx_dump_total,
                               g_v34 ? v34_get_rx_stage(g_v34) : -1,
                               g_v34 ? v34_get_tx_stage(g_v34) : -1,
                               (int)g_mod);
                    }
                }
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
            if (me_v90_analogue_role()) {
                /* The analogue role's data direction remains V.34 upstream;
                 * keep using the modulator whose mapper was seeded from MP. */
                pthread_mutex_lock(&g_state_mtx);
                if (g_v90a) {
                    v90_analogue_phase3_tx(g_v90a, amp, len);
                    me_v90_analogue_progress_locked();
                }
                pthread_mutex_unlock(&g_state_mtx);
            } else {
                uint8_t pcm_out[len];
                bool generated;

                pthread_mutex_lock(&g_state_mtx);
                generated = generate_v90_raw_codewords_locked(pcm_out, len);
                pthread_mutex_unlock(&g_state_mtx);
                if (generated) {
                    for (int i = 0; i < len; i++)
                        amp[i] = pcm_to_linear(pcm_out[i]);
                }
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

    if (tx_disrupt_active()) {
        /* ME_TX_DISRUPT_*: deliberate carrier loss, to make the far end's
           receiver fail and start a retrain we can then be seen to answer. */
        memset(amp, 0, sizeof(int16_t) * (size_t) len);
        if (g_tx_disrupt_logged == 0) {
            g_tx_disrupt_logged = trace_now_ms();
            ME_LOG("[ME] TX DISRUPT: transmitting silence for %d ms "
                   "(ME_TX_DISRUPT_MS)\n", me_tx_disrupt_ms());
        }
    } else if (g_tx_disrupt_logged > 0) {
        /* Negative means this once-per-call probe has completed.  A retrain
           updates g_v34_data_entry_ms when it returns to DATA; without this
           latch the hook opened another disruption 20 s later and retrained
           repeatedly for the rest of the call. */
        g_tx_disrupt_logged = -1;
        ME_LOG("[ME] TX DISRUPT: ended, carrier restored\n");
    }

    /* Buffer TX samples for the echo canceller.
       Must happen after TX generation so me_rx_audio can subtract
       our echo from the received signal. */
    buffer_tx_samples_for_echo(amp, len);
}

/*
 * ME_G711_CAPTURE=<prefix> records the received G.711 octets to
 * <prefix>.rx.ulaw, byte for byte as the engine consumed them.
 *
 * In the analogue role this stream *is* the far end's DS0 output, so a capture
 * of it can be diffed against what the digital modem is known to have
 * transmitted.  That difference is the only way to tell a decoder fault from a
 * transport fault, and the two look identical in the log.
 */
static void me_g711_capture_rx(const uint8_t *codewords, int count)
{
    static FILE *f;
    static bool tried;

    if (!tried) {
        const char *prefix = getenv("ME_G711_CAPTURE");
        char path[512];

        tried = true;
        if (prefix == NULL  ||  prefix[0] == '\0')
            return;
        snprintf(path, sizeof(path), "%s.rx.ulaw", prefix);
        if ((f = fopen(path, "wb")) == NULL) {
            ME_LOG("[ME] G.711 capture: cannot write %s\n", path);
            return;
        }
        ME_LOG("[ME] G.711 capture: recording received octets to %s\n", path);
    }
    if (f) {
        fwrite(codewords, 1, (size_t) count, f);
        fflush(f);
    }
}

/*
 * The analogue role's Phase 3/4 receive work, in the codeword domain.
 *
 * Factored out of me_rx_g711() because a digital bearer is not the only way
 * these codewords arrive: on a two-wire analogue line the samples reach
 * me_rx_audio() as levels, and v90_analogue_linear.c slices them back into
 * the same stream.  Both bearers run the same receiver.  g_state_mtx is held.
 */
static void me_v90_analogue_rx_codewords_locked(const uint8_t *codewords, int count)
{
    unsigned events;
    uint8_t data_bits[2048];
    int nbits;


    /* Phase 3 and Phase 4/data are one byte-exact DS0 stream.  The Phase 4
     * receiver changes from CPt to CP at Ed and retains that mapper into
     * data, so it must continue to receive after ME_DATA is entered. */
    events = v90_analogue_phase3_rx(g_v90a, codewords, count);
    g_v90a_rx_codewords += (uint64_t) count;
    /*
     * The stage report beside this is edge-triggered, so a receiver that is
     * fed correctly and never advances says NOTHING for the whole of Phase 3
     * -- which is indistinguishable in a log from one that is being fed
     * nothing at all.  V90A_RX_TRACE=1 reports the stage and its counters on
     * a period whether or not anything changed.
     */
    {
        static int trace = -1;
        if (trace < 0) {
            const char *v = getenv("V90A_RX_TRACE");
            trace = (v && *v && *v != '0') ? 1 : 0;
        }
        if (trace && (g_v90a_rx_codewords % 8000) < (uint64_t) count) {
            const void *rx = v90_analogue_phase3_rx_state(g_v90a);
            ME_LOG("[ME] V.90 analogue RX trace: %s at %llu codewords "
                   "(Sd %d reps, S-bar-d %d reps, TRN1d %dT, Jd %d frames)\n",
                   v90_analogue_rx_stage_name(
                       v90_analogue_phase3_rx_stage(g_v90a)),
                   (unsigned long long) g_v90a_rx_codewords,
                   v90_analogue_rx_sd_reps(rx), v90_analogue_rx_sd_bar_reps(rx),
                   v90_analogue_rx_trn1d_symbols(rx),
                   v90_analogue_rx_jd_frames(rx));
        }
    }
    if (events & V90A4_RX_EVENT_CLEARDOWN) {
        /* §9.7: MP drn=0 is a completed cleardown indication, not a
         * malformed MP and not a retrain request. */
        ME_LOG("[ME] V.90 analogue: peer requested §9.7 cleardown (MP drn=0)\n");
        trace_phase("V90a peer cleardown");
        g_state = ME_HANGUP;
    }
    if (events & V90A_EVENT_TONE_B_RETRAIN) {
        /* §9.3.2/§9.4.2/§9.6.2: sustained Tone B is the digital modem's
         * retrain request; answer with §9.5.2.2, not a fresh V.8 call. */
        ME_LOG("[ME] V.90 analogue: Tone B >50 ms; responding with §9.5.2.2 retrain\n");
        (void) restart_v90_analogue_phase2_locked("peer Tone B");
    }
    me_v90_analogue_progress_locked();
    if ((events & V90A4_RX_EVENT_DATA)
        && g_state == ME_TRAINING
        && v90_analogue_phase3_data_ready(g_v90a)) {
        const vpcm_cp_frame_t *cp = v90_analogue_phase3_cp(g_v90a);
        int downstream_rate = cp ? (int)vpcm_cp_drn_to_bps(cp->drn) : 0;
        int upstream_rate = v90_analogue_phase3_upstream_rate(g_v90a);

        if (downstream_rate <= 0)
            downstream_rate = V90_RATE_BPS;
        /* The stack's line bit rate clocks its transmitted V.42/V.14
         * stream.  In the analogue role that stream is the V.34 upstream,
         * not the faster PCM downstream; using 56 kbit/s here made V.42's
         * detection timers expire while bits left at 12–26.4 kbit/s. */
        data_stack_start_online(upstream_rate > 0 ? upstream_rate
                                                  : downstream_rate,
                                g_calling_party);
        g_state = ME_DATA;
        g_phase_start_ms = 0;
        ME_LOG("[ME] V.90 analogue startup complete after B1d "
               "(upstream V.34 %d bps, downstream PCM %d bps)\n",
               upstream_rate, downstream_rate);
        trace_phase("V90a enter DATA after B1d: upstream=%d downstream=%d",
                    upstream_rate, downstream_rate);
        if (g_data_framing != DS_FRAMING_V42) {
            g_data_connect_reported = true;
            di_on_connected(downstream_rate);
        } else {
            ME_LOG("[ME] V.90 analogue carrier ready; waiting for V.42 LAPM\n");
        }
    }
    while ((nbits = v90_analogue_phase3_get_data_bits(
                g_v90a, data_bits, (int)sizeof(data_bits))) > 0) {
        for (int i = 0; i < nbits; i++) {
            int data_bit = data_bits[i] & 1U;

            if (g_v90a_data_diag_bits < 4096) {
                g_v90a_data_diag_bits++;
                if (!data_bit)
                    g_v90a_data_diag_zeros++;
                if (g_v90a_data_diag_bits == 4096) {
                    ME_LOG("[ME] V.90 analogue first 4096 downstream data "
                           "bits: %d zeroes, %d marks\n",
                           g_v90a_data_diag_zeros,
                           4096 - g_v90a_data_diag_zeros);
                }
            }
            if (data_bit == 0 && g_v90a_data_bits_seen > 0
                && g_v90a_data_adp_window_bits == 44
                && g_v90a_data_adp_window == ((1ULL << 44) - 1ULL)) {
                ME_LOG("[ME] V.90 analogue first downstream zero at bit %llu\n",
                       (unsigned long long)g_v90a_data_bits_seen);
            }
            /* V.42 §7.2.1 Table 3: (E), 12 marks, (C), 12 marks. */
            g_v90a_data_adp_window =
                (g_v90a_data_adp_window >> 1) | ((uint64_t)data_bit << 43);
            if (g_v90a_data_adp_window_bits < 44)
                g_v90a_data_adp_window_bits++;
            if (g_v90a_data_adp_window_bits == 44
                && g_v90a_data_adp_window == 0xFFFA1BFFE8AULL) {
                g_v90a_data_adp_count++;
                ME_LOG("[ME] V.90 analogue decoded V.42 ADP #%d at "
                       "downstream bit %llu\n",
                       g_v90a_data_adp_count,
                       (unsigned long long)(g_v90a_data_bits_seen - 43));
            }
            g_v90a_data_bits_seen++;
            ds_rx_put_bit(&g_data_stack, data_bit);
        }
    }
}

/*
 * The analogue role fed from the caller's own 16 kHz stream.
 *
 * Two samples per DS0 interval is what makes the downstream recoverable over a
 * two-wire line at all.  Sampled at T/2 the line's response and the caller's
 * sampling phase are one linear filter, so a fractionally-spaced equaliser
 * inverts both together -- and it has to, because at 8 kHz neither is
 * reachable: the bearer has zero excess bandwidth, so there is no timing tone
 * to find the instant with, and the channel's first neighbours are coarser than
 * the µ-law ladder's steps near the top (docs/hsf_analogue_v90_coupler.md).
 *
 * The equalised symbols come out at one per DS0 interval with their modulus
 * pinned to §8.4.5's TRN1d level, which is exactly what the level slicer above
 * already knows how to turn into codewords, so the rest of the path is
 * unchanged from the digital bearer's.
 */
void me_rx_v90a_16k(const int16_t *amp, int len)
{
    if (amp == NULL  ||  len <= 0)
        return;
    pthread_mutex_lock(&g_state_mtx);
    if (!g_v90a_started  ||  g_v90a == NULL
        ||  (g_state != ME_TRAINING  &&  g_state != ME_DATA)) {
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }
    if (g_v90a_fse == NULL) {
        g_v90a_fse = v90a_fse_init(0, 0.0);
        if (g_v90a_fse == NULL) {
            pthread_mutex_unlock(&g_state_mtx);
            return;
        }
        /*
         * FROZEN, not CMA.  The first thing on the line is §8.4.4's Sd, which
         * is not constant modulus -- four slots at W and two at zero -- so the
         * blind loop would drive the zero slots up to W and destroy the only
         * structure the hunt has.  The taps are fitted to Sd below instead;
         * CMA is right for §8.4.5's TRN1d and starts there.
         */
        v90a_fse_set_mode(g_v90a_fse, V90A_FSE_FROZEN);
        g_v90a_sd = v90a_sd_init(0);
        ME_LOG("[ME] V.90 analogue: T/2 equaliser on the 16 kHz stream, "
               "awaiting the Sd fit\n");
    }
    /*
     * Acquire the equaliser on Sd.  Buffer a window, fit §8.4.4's reference,
     * install the taps -- and only then let anything downstream see a symbol,
     * because before the fit the output is the raw channel at whatever
     * sampling phase the caller happened to hand over, which is precisely what
     * the codeword slicer cannot use.
     */
    if (!g_v90a_sd_fitted) {
        int taps = v90a_fse_tap_count(g_v90a_fse);
        int i;

        for (i = 0; i < len  &&  g_v90a_sd_fill < V90A_SD_FIT_SAMPLES; i++)
            g_v90a_sd_buf[g_v90a_sd_fill++] = amp[i];
        if (g_v90a_sd_fill >= V90A_SD_FIT_SAMPLES) {
            double h[V90A_SD_MAX_TAPS], score = 0.0, level = 0.0;
            int parity = 0;

            if (taps <= V90A_SD_MAX_TAPS
                &&  v90a_sd_fit(g_v90a_sd_buf, g_v90a_sd_fill, taps, &parity,
                                h, &score, &level)) {
                v90a_fse_set_taps(g_v90a_fse, h, taps, parity);
                g_v90a_sd_fitted = true;
                ME_LOG("[ME] V.90 analogue: Sd fit accepted (held-out score "
                       "%.3f, T/2 parity %d, level %.3f); equaliser acquired "
                       "on §8.4.4's own sequence\n", score, parity, level);
            } else {
                /* Slide so a fit that straddles the start of Sd -- or its end,
                 * where S-bar-d begins -- gets a clean one next time rather
                 * than repeating the same straddle. */
                memmove(g_v90a_sd_buf, g_v90a_sd_buf + V90A_SD_FIT_SLIDE,
                        (size_t) (V90A_SD_FIT_SAMPLES - V90A_SD_FIT_SLIDE)
                            *sizeof(g_v90a_sd_buf[0]));
                g_v90a_sd_fill = V90A_SD_FIT_SAMPLES - V90A_SD_FIT_SLIDE;
                if (g_v90a_sd_score_logged < 24) {
                    g_v90a_sd_score_logged++;
                    ME_LOG("[ME] V.90 analogue: no Sd in this window "
                           "(held-out score %.3f)\n", score);
                }
            }
        }
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }
    if (!g_v90a_16k) {
        g_v90a_16k = true;
        if (g_v90a_linear == NULL)
            g_v90a_linear = v90a_linear_init((g_law == ME_LAW_ALAW)
                                             ? V90_LAW_ALAW : V90_LAW_ULAW);
    }
    /*
     * Calibrate the ladder as soon as TRN1d has been acquired.
     *
     * CMA pins the equaliser's output modulus to §8.4.5's TRN1d level, and the
     * receiver has just learned which Ucode that is off the wire -- so the
     * absolute scale of the whole ladder follows, without measuring a peak and
     * without trusting the U_INFO we asked for.  That is what §8.4.1's DIL and
     * §8.6's Phase 4 need: the training signals are one level with a sign on
     * them and survive an arbitrary scale, and nothing after them does.
     */
    if (!g_v90a_ladder_set) {
        const v90_analogue_rx_t *rx = v90_analogue_phase3_rx_state(g_v90a);
        int trn1d = (rx != NULL) ? v90_analogue_rx_trn1d_ucode(rx) : 0;

        if (trn1d > 0) {
            uint8_t one = (uint8_t) trn1d;

            v90a_linear_set_reference(g_v90a_linear, trn1d, V90A_FSE_SCALE);
            /*
             * And say what is on the line, which right now is that one Ucode
             * and nothing else.  Slicing §8.4.5's TRN1d onto all 128 Ucodes is
             * not merely wasteful: it makes the decision region meaningless,
             * and the region is what tells the equaliser whether to believe a
             * decision.  Measured, full-ladder tolerances refused 29567
             * decisions out of 30000 on a signal where every one of them was
             * certain.
             */
            v90a_linear_set_constellation(g_v90a_linear, &one, 1);
            g_v90a_ladder_set = true;
            /*
             * Hand CMA over here rather than at the DIL.  This is the one
             * stretch of the call where a decision-directed loop is not a
             * gamble -- two levels, a long way apart -- so it is where the
             * equaliser should be converged, at a step near unity, before the
             * signal stops being that easy.  Handing over at the DIL instead
             * leaves CMA's residual in the taps, and CMA's residual is larger
             * than the ladder's steps: on the test channel, exact codeword
             * recovery afterwards is 52% that way and 92% this way.
             */
            v90a_fse_set_mu(g_v90a_fse, V90A_FSE_MU_TRAIN);
            v90a_fse_set_mode(g_v90a_fse, V90A_FSE_DD);
            ME_LOG("[ME] V.90 analogue: ladder calibrated on TRN1d Ucode %d, "
                   "equaliser decision-directed (dispersion %.4f)\n",
                   trn1d, v90a_fse_dispersion(g_v90a_fse));
        }
    }
    /*
     * §8.4.1's DIL is a level ladder, so from here the decisions are ordinary
     * ones and the step comes down to match -- and the slicer is told which
     * levels those are.
     *
     * It can be told exactly, because this side AUTHORED the descriptor: §8.4.1
     * sends it to the digital modem in Ja and §9.3.2.9 has the analogue modem
     * "receive the DIL sequence it requested", so every Ucode that is about to
     * arrive is known before it does.  Without that the slicer falls back to
     * all 128 and spends the DIL deciding between levels the far end is not
     * transmitting -- and, worse, its decision regions become so fine that the
     * equaliser refuses nearly every decision and effectively freezes.
     */
    if (v90_analogue_phase3_rx_stage(g_v90a) >= V90A_RX_DIL
        &&  !g_v90a_dil_tracking) {
        uint8_t set[128];
        int n = 0;

        g_v90a_dil_tracking = true;
        if (g_v90a_dil_valid)
            n = v90_dil_ucode_set((g_law == ME_LAW_ALAW) ? V90_LAW_ALAW
                                                         : V90_LAW_ULAW,
                                  &g_v90a_dil, set, (int) sizeof(set));
        v90a_linear_set_constellation(g_v90a_linear, n > 0 ? set : NULL, n);
        v90a_fse_set_mu(g_v90a_fse, V90A_FSE_MU_TRACK);
        if (v90a_fse_mode(g_v90a_fse) == V90A_FSE_CMA) {
            /* TRN1d never gave us a Ucode, so nothing is calibrated and a
             * decision-directed loop would be adapting on levels that mean
             * nothing.  Stop instead. */
            v90a_fse_set_mode(g_v90a_fse, V90A_FSE_FROZEN);
        }
        /*endif*/
        ME_LOG("[ME] V.90 analogue: DIL over %d requested Ucodes; equaliser %s, "
               "dispersion %.4f, tap centre %.2f symbols, "
               "%d decisions used and %d refused\n",
               n,
               v90a_fse_mode(g_v90a_fse) == V90A_FSE_DD ? "tracking"
                                                        : "frozen",
               v90a_fse_dispersion(g_v90a_fse),
               v90a_fse_centre(g_v90a_fse),
               v90a_fse_dd_used(g_v90a_fse),
               v90a_fse_dd_rejected(g_v90a_fse));
    }
    /*
     * One symbol at a time, deliberately.
     *
     * The decision an equaliser adapts on has to be the decision for the symbol
     * whose delay line is still in place, so the equaliser, the slicer and the
     * feedback run in lockstep rather than in batches.  At 8000 symbols a
     * second the call overhead is nothing and the alternative is silently
     * adapting every symbol of a block on the last one's decision.
     */
    for (int offset = 0; offset + 1 < len; offset += 2) {
        double sym;
        int16_t scaled;
        uint8_t codeword;
        double v;

        if (v90a_fse_put(g_v90a_fse, amp + offset, 2, &sym, 1) != 1)
            continue;
        v = sym*V90A_FSE_SCALE;
        if (v > 32000.0)
            v = 32000.0;
        else if (v < -32000.0)
            v = -32000.0;
        scaled = (int16_t) v;
        /*
         * A structural confirmation of the fit on the equalised stream, once.
         * The fit's own score is held out and is the gate; this is the reading
         * that says what the receiver is actually looking at -- the grid it
         * settled on and W's level in the equaliser's units -- and it is what
         * a live log needs when the fit passes and nothing downstream works.
         */
        if (g_v90a_sd != NULL  &&  !v90a_sd_acquired(g_v90a_sd)
            &&  v90a_sd_put(g_v90a_sd, sym)) {
            ME_LOG("[ME] V.90 analogue: Sd structure confirmed on the "
                   "equalised stream (slot %d, level %.3f, score %.3f)\n",
                   v90a_sd_next_slot(g_v90a_sd), v90a_sd_level(g_v90a_sd),
                   v90a_sd_score(g_v90a_sd));
        }
        if (v90a_linear_put(g_v90a_linear, &scaled, 1, &codeword, 1) == 1) {
            me_v90_analogue_rx_codewords_locked(&codeword, 1);
            if (v90a_fse_mode(g_v90a_fse) == V90A_FSE_DD)
                v90a_fse_decide(g_v90a_fse,
                                v90a_linear_last_decision(g_v90a_linear)
                                    /V90A_FSE_SCALE,
                                v90a_linear_last_tolerance(g_v90a_linear)
                                    /V90A_FSE_SCALE);
        }
        /*endif*/
        if (!v90a_linear_locked(g_v90a_linear)
            &&  v90_analogue_phase3_rx_stage(g_v90a) != V90A_RX_HUNT_SD)
            v90a_linear_lock(g_v90a_linear);
    }
    pthread_mutex_unlock(&g_state_mtx);
}

void me_rx_g711(const uint8_t *codewords, int count)
{
    int offset;
    bool raw_v91;
    uint64_t first_sample;

    if (!codewords || count <= 0)
        return;

    me_g711_capture_rx(codewords, count);

    if (me_fax_rx_g711(codewords, count)) {
        pthread_mutex_lock(&g_state_mtx);
        g_g711_rx_octets += (uint64_t)count;
        pthread_mutex_unlock(&g_state_mtx);
        return;
    }

    pthread_mutex_lock(&g_state_mtx);
    first_sample = g_g711_rx_octets;
    g_g711_rx_octets += (uint64_t)count;
    raw_v91 = (g_mod == ME_MOD_V91
               && (g_state == ME_TRAINING || g_state == ME_DATA));
    if (raw_v91)
        v91_live_receive_codewords_locked(codewords, count);
    if (g_v90a_started && g_v90a
        && (g_state == ME_TRAINING || g_state == ME_DATA)) {
        me_v90_analogue_rx_codewords_locked(codewords, count);
    }
    if (g_v92_p3_rx_active && g_v92_active && g_state == ME_TRAINING) {
        for (int i = 0; i < count; i++) {
            (void)v92_p3_rx_feed(&g_v92_p3_rx,
                                 codewords[i],
                                 (int)(first_sample + (uint64_t)i));
            v92_apply_p3_ja_locked();
            if (!g_v92_p3_rx_active)
                break;
        }
    }
    if (g_v92_su_rx_active && g_v92_active && g_state == ME_TRAINING) {
        for (int i = 0; i < count; i++)
            v92_su_rx_feed_locked(codewords[i], first_sample + (uint64_t)i);
    }
    if (g_v92_p3_cpt_active && g_v92_active && g_state == ME_TRAINING) {
        (void)v92_trn2u_demod_feed(&g_v92_p3_cpt_demod, codewords, count);
    }
    if (g_v92_trn2u_active && g_v92_active && g_v90
        && g_state == ME_TRAINING
        && v90_get_tx_phase(g_v90) >= V90_TX_TRN2D
        && v90_get_tx_phase(g_v90) < V90_TX_DATA) {
        (void)v92_trn2u_demod_feed(&g_v92_trn2u_demod, codewords, count);
    }
    if (g_v92_upstream_rx_active && g_v92_active
        && (g_state == ME_TRAINING || g_state == ME_DATA)) {
        for (int i = 0; i < count; i++) {
            int16_t linear = pcm_to_linear(codewords[i]);
            bool was_locked = g_v92_upstream_rx.locked;

            (void)v92_upstream_b1_rx_feed(&g_v92_upstream_rx, &linear, 1);
            if (!was_locked && g_v92_upstream_rx.locked) {
                if (!g_v92_upstream_lock_logged) {
                    g_v92_upstream_lock_logged = true;
                    ME_LOG("[ME] V.92 PCM upstream: B1u locked corr=%.6f gain=%.6f offset=%.2f eq_delay=%d\n",
                           g_v92_upstream_rx.correlation,
                           g_v92_upstream_rx.gain,
                           g_v92_upstream_rx.offset,
                           g_v92_upstream_rx.equalizer_delay);
                    trace_phase("V92 B1u locked corr=%.6f",
                                g_v92_upstream_rx.correlation);
                }
                if (g_v90 && v90_get_tx_phase(g_v90) == V90_TX_DATA)
                    enter_v90_data_locked();
            }
        }
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

    /* The codeword work above has already been done for these samples; say so,
     * or me_rx_audio() slices them a second time (v90_analogue_linear.h). */
    g_rx_from_g711 = true;
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
    g_rx_from_g711 = false;
}

/* Cached: this is read on every transmit frame. */
static bool me_sounder_active(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("ME_SOUNDER");

        cached = (v != NULL  &&  v[0] != '\0'  &&  v[0] != '0') ? 1 : 0;
        if (cached)
            ME_LOG("[ME] ME_SOUNDER set: transmitting the channel sounder "
                   "instead of a modem signal for this whole call\n");
    }
    return cached != 0;
}

int me_tx_g711(uint8_t *codewords, int count)
{
    int offset;
    uint64_t raw_octets = 0;
    uint64_t linear_octets = 0;

    if (!codewords || count <= 0)
        return 0;

    /* Which transmit path a call actually takes, once a second.  A Canon
       TR7560 was sent EXACTLY zero for twenty seconds while the V.34 state
       machine walked HDX_SECOND_A, HDX_SECOND_A_WAIT and INFOh, and neither
       "Training TX: RMS" nor "TX PCM dump" appeared in the log -- so the
       samples were not being discarded downstream, the branch that generates
       them was never reached.  me_tx_audio() memsets the buffer first, so
       every way of missing it produces silence that looks identical on the
       wire.  Name the path instead of inferring it. */
    {
        static uint64_t path_samples = 0;
        static int last_path = -1;
        int path;

        path_samples += (uint64_t)count;
        path = di_fax_active() && !me_v34_fax_probe() ? 3 : 0;
        if (path_samples >= 8000  ||  path != last_path) {
            path_samples = 0;
            last_path = path;
            ME_LOG("[ME] TX path: state=%d mod=%d fax_active=%d v34=%p v22bis=%p\n",
                   (int)g_state, (int)g_mod, di_fax_active() ? 1 : 0,
                   (void *)g_v34, (void *)g_v22bis);
        }
    }
    /*
     * ME_SOUNDER=1 replaces this side's transmit with the channel sounder
     * (v90_sounder.h) for the whole call.  It is a measurement mode, not a
     * modem: no handshake happens, and whichever end sets it is the end doing
     * the transmitting, so the same flag sounds the downstream from the
     * digital side and the upstream from the analogue one.
     *
     * Pair it with ME_G711_CAPTURE, which records these exact codewords: the
     * response is then RX/TX and the transmit path's own µ-law quantisation is
     * inside the reference rather than inside the answer.
     */
    if (me_sounder_active()) {
        static int phase;
        static uint64_t logged;

        v90_sounder_fill((g_law == ME_LAW_ALAW) ? V90_LAW_ALAW : V90_LAW_ULAW,
                         codewords, count, &phase);
        logged += (uint64_t) count;
        if (logged >= 8000) {
            int n = 0;

            logged = 0;
            (void) v90_sounder_tones(&n);
            ME_LOG("[ME] sounder: transmitting %d tones to 3950 Hz\n", n);
        }
        /*endif*/
        if (g_g711_tx_tap)
            (void) fwrite(codewords, 1, (size_t) count, g_g711_tx_tap);
        return count;
    }
    /*endif*/
    if (me_fax_tx_g711(codewords, count)) {
        pthread_mutex_lock(&g_state_mtx);
        g_g711_tx_octets += (uint64_t)count;
        pthread_mutex_unlock(&g_state_mtx);
        if (g_g711_tx_tap)
            (void)fwrite(codewords, 1, (size_t)count, g_g711_tx_tap);
        return count;
    }

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
            /* The V.PCM paths bypass me_tx_audio(), so apply the retrain test
               hook in the byte-exact G.711 domain as well.  The old hook
               silently worked only for plain V.34: native V.90 reached DATA
               but could never provoke the peer retrain it was meant to test.
               pcm_idle() is the law's digital-silence codeword; no transcode
               or gain operation is introduced into the normal path. */
            if (tx_disrupt_active()) {
                memset(codewords + offset, pcm_idle(), (size_t)chunk);
                if (g_tx_disrupt_logged == 0) {
                    g_tx_disrupt_logged = trace_now_ms();
                    ME_LOG("[ME] TX DISRUPT: transmitting G.711 silence for "
                           "%d ms (ME_TX_DISRUPT_MS)\n", me_tx_disrupt_ms());
                }
            } else if (g_tx_disrupt_logged > 0) {
                g_tx_disrupt_logged = -1;
                ME_LOG("[ME] TX DISRUPT: ended, G.711 carrier restored\n");
            }
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

/* ME_DATA_HOLD=1 keeps a call up after a V.42 failure.  The physical modem is
   still in data mode at that point, and tearing down 750 ms into T400 destroys
   the evidence needed to tell why detection failed. */
static bool me_data_hold_locked(void)
{
    static int hold = -1;

    if (hold < 0) {
        const char *v = getenv("ME_DATA_HOLD");

        hold = (v && *v && *v != '0');
    }
    return hold != 0;
}

me_state_t me_get_state(void)
{
    pthread_mutex_lock(&g_state_mtx);
    if (g_data_link_failed && g_state == ME_DATA && !me_data_hold_locked()) {
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
