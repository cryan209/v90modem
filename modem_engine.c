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
#include "v90_cp_rx.h"
#include "v92_cp_rx.h"
#include "v92_trn2u.h"

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
static data_stack_t g_data_stack;
static ds_framing_t g_data_framing = DS_FRAMING_V14;
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
static int            g_v90_phase3_s_events = 0;
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
static bool           g_v90_dil_parse_logged = false;

#define V90_DIL_CAPTURE_MAX_BITS 8192
static uint8_t        g_v90_dil_capture[(V90_DIL_CAPTURE_MAX_BITS + 7) / 8];
static int            g_v90_dil_capture_bits = 0;
static int            g_v90_dil_capture_search = 0;

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
       The current Jp profile selects the mandatory 4-point TRN2u channel. */
    v8_parms.v92                = g_calling_party ? 0x45 : 0x47;
    v8_parms.jm_cm.call_function      = V8_CALL_V_SERIES;
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
        g_state = ME_DATA;
        g_phase_start_ms = 0;
        pthread_mutex_unlock(&g_state_mtx);
        ME_LOG("[ME] %s training complete (%d bps)\n", name, rate);
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
void me_hangup(void);

/* Bit accumulators for V.34 TX and RX (one byte at a time) */
static uint8_t  v34_tx_byte = 0;
static int      v34_tx_bits = 0;
static uint8_t  v34_rx_byte = 0;
static int      v34_rx_bits = 0;

static void v90_dil_capture_reset(void)
{
    memset(g_v90_dil_capture, 0, sizeof(g_v90_dil_capture));
    g_v90_dil_capture_bits = 0;
    g_v90_dil_capture_search = 0;
    g_v90_pending_dil_valid = false;
    g_v90_dil_parse_logged = false;
    g_v90_phase3_s_events = 0;
    g_last_v90_bridge_rx_stage = -1;
    g_last_v90_bridge_tx_stage = -1;
    g_last_v90_bridge_rx_event = -1;
    memset(&g_v90_pending_dil, 0, sizeof(g_v90_pending_dil));
}

/* Runs synchronously inside v34_rx() while g_state_mtx is already held. */
static void v90_live_cp_frame(void *user_data, const vpcm_cp_diag_t *diag)
{
    bool accepted;

    (void)user_data;
    if (!diag || !g_v90 || g_mod != ME_MOD_V90)
        return;
    accepted = v90_set_phase4_cp(g_v90, &diag->frame)
        && v90_handle_rx_event(g_v90, V90_RX_EVENT_CP_VALID);
    ME_LOG("[ME] V.90 strict RX event=CP_VALID kind=%s bits=%d drn=%u ack=%d constellations=%u accepted=%d\n",
           diag->frame.v90_compatibility ? "CP" : "CPt",
           diag->nbits,
           (unsigned)diag->frame.drn,
           diag->frame.acknowledge ? 1 : 0,
           (unsigned)diag->frame.constellation_count,
           accepted ? 1 : 0);
    trace_phase("V90 strict RX event=CP_VALID kind=%s bits=%d drn=%u accepted=%d",
                diag->frame.v90_compatibility ? "CP" : "CPt",
                diag->nbits, (unsigned)diag->frame.drn, accepted ? 1 : 0);
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
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    v90_cp_rx_reset(&g_v90_cp_rx);
    v92_cp_rx_reset(&g_v92_cp_rx);
    memset(&g_v92_trn2u_demod, 0, sizeof(g_v92_trn2u_demod));
    g_v92_trn2u_active = false;
    g_v34_fallback_to_v22bis_pending = false;
    g_v34_fallback_status = 0;
    v90_dil_capture_reset();
    g_notch.active = false;
    g_last_rx_stage = 0;
    g_last_tx_stage = 0;
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
                        g_state = ME_DATA;
                        g_phase_start_ms = 0;
                        ME_LOG("[ME] V.90 training complete (upstream V.34 %d bps, downstream PCM %d bps)\n",
                                rate, V90_RATE_BPS);
                        trace_phase("V90 enter DATA: upstream=%d downstream=%d", rate, V90_RATE_BPS);
                        v90_reset_data_mode(g_v90);
                        g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
                        di_on_connected(V90_RATE_BPS);
                    } else if (!g_v90_completion_deferred_logged) {
                        ME_LOG("[ME] V.90 received generic training success from V.34, but V.90 startup is not complete yet; remaining in TRAINING\n");
                        trace_phase("V90 deferred DATA entry: V34 success before V90 startup complete");
                        g_v90_completion_deferred_logged = true;
                    }
                } else {
                    g_state = ME_DATA;
                    g_phase_start_ms = 0;
                    ME_LOG("[ME] V.34 training complete (%d bps)\n", rate);
                    trace_phase("V34 enter DATA: rate=%d", rate);
                    di_on_connected(rate);
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
    g_phase_start_ms = trace_now_ms();
    trace_phase("enter TRAINING: mod=V22BIS role=%s", g_calling_party ? "caller" : "answerer");
    ds_init(&g_data_stack,
            g_data_framing,
            data_stack_pull_dte_byte, NULL,
            data_stack_push_dte_byte, NULL);
    ds_set_v14_rates(&g_data_stack, 2400, 2400);
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
    if (result->status == V8_STATUS_IN_PROGRESS ||
        result->status == V8_STATUS_V8_OFFERED) {
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
        ME_LOG("[ME] V.8 failed (status=%d), hanging up\n", result->status);
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
        g_v92_active = result->v92 >= 0;
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
    dring_init(&downstream_ring);
    dring_init(&upstream_ring);
    {
        const char *framing = getenv("ME_DATA_FRAMING");

        if (framing && strcmp(framing, "raw") == 0)
            g_data_framing = DS_FRAMING_RAW;
        else
            g_data_framing = DS_FRAMING_V14;
        ME_LOG("[ME] DTE framing: %s%s\n",
               g_data_framing == DS_FRAMING_V14 ? "V.14 8N1" : "RAW",
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
}

void me_destroy(void)
{
    if (g_v8)       { v8_free(g_v8);                             g_v8       = NULL; }
    if (g_v22bis)   { v22bis_free(g_v22bis);                     g_v22bis   = NULL; }
    cleanup_v34_v90_training_locked();
    if (g_echo_can) { modem_echo_can_segment_free(g_echo_can);   g_echo_can = NULL; }
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
    if (g_echo_can) { modem_echo_can_segment_free(g_echo_can);   g_echo_can = NULL; }
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

void me_rx_audio(const int16_t *amp, int len)
{
    pthread_mutex_lock(&g_state_mtx);
    me_state_t state = g_state;
    pthread_mutex_unlock(&g_state_mtx);

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
                    if (new_e_event && g_v90) {
                        bool accepted = v90_handle_rx_event(g_v90, V90_RX_EVENT_E);

                        fprintf(stderr,
                                "[ME] V.90 strict RX event=E tx_phase=%d accepted=%d\n",
                                (int)v90_get_tx_phase(g_v90), accepted ? 1 : 0);
                        trace_phase("V90 strict RX event=E accepted=%d",
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

    /* V.90 Table 10: reserved fields are zero, upstream is one of the
     * specified 3000/3200/3429 codes, and downstream PCM is code 6 (8000). */
    return received.raw_12_17 == 0
        && received.raw_32_33 == 0
        && received.u_info > 0
        && received.upstream_symbol_rate_code >= 3
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
        ME_LOG("[ME] V.90 strict RX event: valid INFO1a U_INFO=%u MD=%u upstream_code=%u downstream_code=%u\n",
               (unsigned)info1a.u_info,
               (unsigned)info1a.md,
               (unsigned)info1a.upstream_symbol_rate_code,
               (unsigned)info1a.downstream_rate_code);
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
    } else if (!g_v90_wait_info1_logged) {
        v34_v90_info1a_t received;

        if (v34_get_v90_received_info1a(g_v34, &received)) {
            ME_LOG("[ME] V.90 strict RX event: rejecting INFO1a reserved=%02X/%02X U_INFO=%d upstream_code=%d downstream_code=%d\n",
                   received.raw_12_17,
                   received.raw_32_33,
                   received.u_info,
                   received.upstream_symbol_rate_code,
                   received.downstream_rate_code);
            trace_phase("V90 strict RX event=INFO1A_INVALID -> remain Phase2");
        } else {
            ME_LOG("[ME] V.90: waiting for CRC-valid INFO1a before Phase 3 TX\n");
        }
        g_v90_wait_info1_logged = true;
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
            int input_len;
            int consumed;

            memset(g_v90_data_frame, pcm_idle(), sizeof(g_v90_data_frame));
            needed = v90_data_input_bytes_needed(g_v90);
            input_len = dring_available(&downstream_ring);
            if (input_len > needed)
                input_len = needed;
            if (input_len > 0
                && dring_read(&downstream_ring, data_in, input_len) != input_len) {
                ME_LOG("[ME] V.90 data ring read failed\n");
                input_len = 0;
            }
            consumed = 0;
            if (v90_tx_data_frame_codewords(g_v90,
                                            g_v90_data_frame,
                                            data_in,
                                            input_len,
                                            &consumed,
                                            true) != V90_DATA_FRAME_LEN
                || consumed != input_len) {
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
    g_state = ME_DATA;
    g_phase_start_ms = 0;
    g_v90_data_frame_pos = V90_DATA_FRAME_LEN;
    ME_LOG("[ME] V.90 startup complete (upstream V.34 %d bps, downstream PCM %d bps)\n",
           upstream_rate, downstream_rate);
    trace_phase("V90 enter DATA after B1d: upstream=%d downstream=%d",
                upstream_rate, downstream_rate);
    di_on_connected(downstream_rate);
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
            if (v90_phase3_tx_codewords(g_v90, codewords + pos, 1) != 1)
                return false;
            pos++;
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
        if (g_mod == ME_MOD_V34 || g_mod == ME_MOD_V90) {
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
                    /* V.90 §9.2.1.1.8: send silence while waiting for INFO1a.
                       V.34 TX still runs into discard to keep RX advancing.
                       Protect RX stage from V.34 TX overwrites. */
                    memset(amp, 0, sizeof(int16_t) * (size_t)len);
                    int16_t discard[len];
                    v34_tx(g_v34, discard, len);
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
        if (g_mod == ME_MOD_V34) {
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

    if (!codewords || count <= 0)
        return;

    pthread_mutex_lock(&g_state_mtx);
    g_g711_rx_octets += (uint64_t)count;
    if (g_v92_trn2u_active && g_v92_active && g_v90
        && g_state == ME_TRAINING
        && v90_get_tx_phase(g_v90) >= V90_TX_TRN2D
        && v90_get_tx_phase(g_v90) < V90_TX_DATA) {
        (void)v92_trn2u_demod_feed(&g_v92_trn2u_demod, codewords, count);
    }
    pthread_mutex_unlock(&g_state_mtx);
    if (g_g711_rx_tap)
        (void)fwrite(codewords, 1, (size_t)count, g_g711_rx_tap);

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

    for (offset = 0; offset < count; ) {
        int16_t linear[320];
        int chunk = count - offset;
        bool raw_v90;

        if (chunk > (int)(sizeof(linear) / sizeof(linear[0])))
            chunk = (int)(sizeof(linear) / sizeof(linear[0]));

        pthread_mutex_lock(&g_state_mtx);
        raw_v90 = generate_v90_raw_codewords_locked(codewords + offset, chunk);
        pthread_mutex_unlock(&g_state_mtx);

        if (raw_v90) {
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
