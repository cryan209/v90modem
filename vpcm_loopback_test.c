/*
 * vpcm_loopback_test.c - Local PCM-modem self-test
 *
 * This harness is intended for the V.PCM modem family under a single
 * "vpcm" umbrella:
 *   - V.91 has a dedicated G.711 bearer/data-plane helper in v91.[ch]
 *   - V.90/V.92 startup should ultimately be driven by the SpanDSP-backed
 *     V.8/V.34/V.90 family code already present in the tree
 *
 * The current V.90/V.92 session coverage below is therefore a startup
 * contract/transport placeholder layered on top of shared PCM helpers.
 * It is useful for end-to-end harness work, but it is not a spec-faithful
 * replacement for the real V.90/V.92 Phase 2/3 engine.
 */

#include "v91.h"

#include <spandsp.h>
#include <pjsua-lib/pjsua.h>
#include <pjmedia-codec/passthrough.h>
#include <pjmedia/frame.h>
#include <pjmedia/transport.h>
#include <pjmedia/rtp.h>
#include <pjmedia/stream.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#define TEST_PAYLOAD_LEN 4096
#define TEST_CHUNK_MAX    257
#define VPCM_CHUNK_SAMPLES 160
#define VPCM_V8_MAX_CHUNKS 1500

typedef enum {
    VPCM_PATH_ANALOG_G711 = 0,
    VPCM_PATH_PCM_G711 = 1
} vpcm_path_mode_t;

typedef struct {
    v91_law_t law;
    vpcm_path_mode_t mode;
} vpcm_channel_t;

typedef struct {
    bool seen;
    v8_parms_t result;
} vpcm_v8_result_t;

/*
 * Minimal V.34/V.90 stage and event enums copied from SpanDSP internals.
 * We intentionally avoid including the full private header tree here.
 */
enum vpcm_v34_rx_stages_e {
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
    V34_RX_STAGE_DATA
};

enum vpcm_v34_tx_stages_e {
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
    V34_TX_STAGE_MP
};

enum vpcm_v34_events_e {
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
    V34_EVENT_TRAINING_FAILED
};

static void vpcm_transport_robbed_bit_codewords(uint8_t *dst,
                                                const uint8_t *src,
                                                int len);

static bool g_vpcm_verbose = false;
static bool g_vpcm_session_diag = false;
static bool g_vpcm_experimental_v90_info = false;
static bool g_vpcm_realtime = false;
static bool g_vpcm_compact_e2e = false;
static volatile sig_atomic_t g_vpcm_stop_requested = 0;

typedef enum {
    VPCM_TRANSPORT_LOOPBACK = 0,
    VPCM_TRANSPORT_PJ_SIP = 1
} vpcm_transport_backend_t;

typedef enum {
    VPCM_PJSIP_ROLE_CALLER = 0,
    VPCM_PJSIP_ROLE_ANSWERER = 1
} vpcm_pjsip_role_t;

static vpcm_transport_backend_t g_vpcm_transport_backend = VPCM_TRANSPORT_LOOPBACK;

typedef struct {
    pjsua_call_id call_id;
    pjsua_conf_port_id call_conf_port;
    vpcm_pjsip_role_t role;
    pjmedia_port *stream_port;
    pjmedia_transport *media_tp;
    bool stream_ready;
    bool media_tp_ready;
    bool call_confirmed;
    bool call_disconnected;
    bool media_ready;
    bool media_codec_checked;
    bool media_codec_ok;
    bool media_clock_wired;
    int disconnect_code;
    v91_law_t expected_law;
} vpcm_pjsip_runtime_t;

static vpcm_pjsip_runtime_t g_vpcm_pjsip_runtime;

typedef struct {
    bool initialized;
    bool failed;
    bool completed;
    bool v8_complete;
    bool data_started;
    bool data_mode_active;
    bool drain_logged;
    bool remote_data_logged;
    v91_law_t law;
    int packet_codewords_per_pull;
    int packets_per_second;
    int v91_codewords_per_packet;
    int v8_negotiation_timeout_packets;
    int post_v8_zero_preroll_packets;
    int drain_packets;
    int bytes_per_packet;
    int total_packets;
    int total_bytes;
    int total_codewords;
    int startup_tx_pkt;
    int data_tx_pkt;
    int v8_pkt_count;
    int v8_restart_count;
    int v8_rx_packets;
    int v8_rx_bytes;
    int tx_len;
    int rx_len;
    int rx_codewords;
    int drain_seen_packets;
    int last_v8_status;
    int startup_timeout_packets;
    int startup_pkt_count;
    int startup_rx_stage;
    int startup_remote_rx_len;
    int startup_local_tx_len;
    int startup_local_tx_pos;
    int startup_remote_total_len;
    int startup_stage_offsets[8];
    int startup_stage_lengths[8];
    uint8_t idle_codeword;
    uint64_t bit_errors;
    uint64_t bits_checked;
    uint8_t last_rx_pcm_sample[6];
    uint8_t last_rx_decoded_sample[6];
    uint8_t last_expected_sample[6];
    bool have_last_rx_pcm;
    bool have_last_rx_decoded;
    uint8_t *tx_data;
    uint8_t *expected_rx;
    uint8_t *rx_data;
    uint8_t *tx_codeword_stream;
    uint8_t *startup_local_stream;
    uint8_t *startup_remote_stream;
    uint8_t *startup_remote_rx_stream;
    vpcm_cp_frame_t cp;
    vpcm_cp_frame_t cp_ack;
    vpcm_cp_frame_t expected_remote_cp;
    v91_dil_desc_t default_dil;
    v91_state_t tx_state;
    v91_state_t rx_state;
    v8_parms_t v8_parms;
    v8_state_t *v8_state;
    vpcm_v8_result_t v8_result;
} vpcm_pjsip_modem_state_t;

static vpcm_pjsip_modem_state_t g_vpcm_pjsip_modem;
static pj_mutex_t *g_vpcm_pjsip_modem_lock;
static pj_pool_t *g_vpcm_pjsip_modem_pool;

typedef struct {
    pjmedia_port base;
    pj_pool_t *pool;
    pjmedia_port *downstream_port;
    unsigned payload_samples_per_frame;
    uint8_t tx_payload[320];
    uint8_t tx_ext_buf[1024];
    uint8_t rx_ext_buf[2048];
} vpcm_pjsip_passthrough_port_t;

static char g_vpcm_passthrough_port_marker;

typedef enum {
    VPCM_STARTUP_STAGE_PHASE1 = 0,
    VPCM_STARTUP_STAGE_EZ = 1,
    VPCM_STARTUP_STAGE_INFO = 2,
    VPCM_STARTUP_STAGE_STARTUP_DIL = 3,
    VPCM_STARTUP_STAGE_SCR = 4,
    VPCM_STARTUP_STAGE_CP = 5,
    VPCM_STARTUP_STAGE_ES = 6,
    VPCM_STARTUP_STAGE_B1 = 7,
    VPCM_STARTUP_STAGE_COUNT = 8
} vpcm_startup_stage_t;

static void vpcm_log(const char *fmt, ...);
static void vpcm_log_e2e_phase(const char *phase, const char *fmt, ...);
static void fill_pattern(uint8_t *buf, int len, uint32_t seed);
static bool expect_equal(const char *label,
                         const uint8_t *expected,
                         const uint8_t *actual,
                         int len);
static void vpcm_log_data_sample(const uint8_t *data_tx,
                                 int data_tx_len,
                                 const uint8_t *pcm_tx,
                                 int pcm_tx_len,
                                 const uint8_t *remote_pcm_rx,
                                 int remote_pcm_rx_len,
                                 const uint8_t *remote_data_rx,
                                 int remote_data_rx_len);
static void vpcm_log_data_sample_named(const char *h1,
                                       const char *h2,
                                       const char *h3,
                                       const char *h4,
                                       const uint8_t *data_tx,
                                       int data_tx_len,
                                       const uint8_t *pcm_tx,
                                       int pcm_tx_len,
                                       const uint8_t *remote_pcm_rx,
                                       int remote_pcm_rx_len,
                                       const uint8_t *remote_data_rx,
                                       int remote_data_rx_len);
static void vpcm_bytes_to_hex(char *out, size_t out_len, const uint8_t *buf, int len);
static uint64_t vpcm_count_bit_errors(const uint8_t *a, const uint8_t *b, int len);
static void vpcm_cp_enable_all_ucodes(uint8_t mask[VPCM_CP_MASK_BYTES]);
static bool test_vpcm_cp_robbed_bit_safe_profile(void);
static bool run_vpcm_session_suite(void);
static bool run_vpcm_primitive_suite(void);
static bool test_spandsp_v90_info_startup_over_analog_g711(v91_law_t law);
static int run_v91_e2e_mode(v91_law_t law, int data_seconds);
static bool run_v91_single_e2e_call_pjsip(v91_law_t law, int data_seconds, uint32_t seed);
static const char *vpcm_pjsip_role_to_str(vpcm_pjsip_role_t role);
static vpcm_pjsip_role_t vpcm_pjsip_parse_role(const char *role_env);
static bool vpcm_run_pjsip_payload_loop(v91_law_t law, int data_seconds, uint32_t seed);
static void vpcm_pjsip_modem_reset(void);
static void vpcm_pjsip_modem_cleanup(void);
static bool vpcm_pjsip_modem_prepare(v91_law_t law, int data_seconds, uint32_t seed,
                                     int packet_codewords_per_pull);
static void vpcm_pjsip_modem_handle_rtp_rx_payload(const uint8_t *payload, int payload_len);
static void vpcm_pjsip_modem_fill_rtp_tx_payload(uint8_t *payload, int payload_len);
static pj_status_t vpcm_pjsip_passthrough_port_create(const pjmedia_port *source_port,
                                                      unsigned payload_samples_per_frame,
                                                      pjmedia_port **p_port);

static void vpcm_handle_sigint(int sig)
{
    (void) sig;
    g_vpcm_stop_requested = 1;
}

static double vpcm_monotonic_seconds(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0.0;
    return (double) ts.tv_sec + ((double) ts.tv_nsec / 1000000000.0);
}

static void vpcm_maybe_realtime_pace_samples(int samples)
{
    struct timespec req;
    struct timespec rem;
    long long total_ns;

    if (!g_vpcm_realtime || samples <= 0)
        return;
    total_ns = ((long long) samples * 1000000000LL) / 8000LL;
    if (total_ns <= 0)
        return;
    req.tv_sec = (time_t) (total_ns / 1000000000LL);
    req.tv_nsec = (long) (total_ns % 1000000000LL);
    while (nanosleep(&req, &rem) != 0) {
        if (errno != EINTR)
            break;
        if (g_vpcm_stop_requested)
            break;
        req = rem;
    }
}

static int vpcm_data_frames_from_seconds(int seconds)
{
    long long frames;

    if (seconds <= 0)
        return 0;
    frames = ((long long) seconds * 8000LL) / (long long) VPCM_CP_FRAME_INTERVALS;
    if (frames < 1)
        frames = 1;
    /* Keep frame count byte-aligned for the current V.91 payload mapping. */
    frames -= (frames % 4LL);
    if (frames < 4)
        frames = 4;
    if (frames > 2000000000LL)
        frames = 2000000000LL;
    return (int) frames;
}

static const char *vpcm_law_to_str(v91_law_t law)
{
    return (law == V91_LAW_ALAW) ? "A-law" : "u-law";
}

static const char *vpcm_path_mode_to_str(vpcm_path_mode_t mode)
{
    return (mode == VPCM_PATH_ANALOG_G711) ? "analog-over-G.711" : "raw-G.711";
}

static const char *vpcm_pjsip_role_to_str(vpcm_pjsip_role_t role)
{
    return (role == VPCM_PJSIP_ROLE_ANSWERER) ? "answerer" : "caller";
}

static const char *vpcm_pjsip_inv_state_to_str(pjsip_inv_state state)
{
    switch (state) {
    case PJSIP_INV_STATE_NULL: return "NULL";
    case PJSIP_INV_STATE_CALLING: return "CALLING";
    case PJSIP_INV_STATE_INCOMING: return "INCOMING";
    case PJSIP_INV_STATE_EARLY: return "EARLY";
    case PJSIP_INV_STATE_CONNECTING: return "CONNECTING";
    case PJSIP_INV_STATE_CONFIRMED: return "CONFIRMED";
    case PJSIP_INV_STATE_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
    }
}

static vpcm_pjsip_role_t vpcm_pjsip_parse_role(const char *role_env)
{
    if (role_env && role_env[0] != '\0') {
        if (strcasecmp(role_env, "answerer") == 0)
            return VPCM_PJSIP_ROLE_ANSWERER;
        if (strcasecmp(role_env, "callee") == 0)
            return VPCM_PJSIP_ROLE_ANSWERER;
    }
    return VPCM_PJSIP_ROLE_CALLER;
}

static bool vpcm_str_ieq_prefix_n(const char *s, int slen, const char *prefix)
{
    int i;
    int plen;

    if (!s || !prefix)
        return false;
    plen = (int) strlen(prefix);
    if (slen < plen)
        return false;
    for (i = 0; i < plen; i++) {
        unsigned char a;
        unsigned char b;

        a = (unsigned char) s[i];
        b = (unsigned char) prefix[i];
        if (a >= 'A' && a <= 'Z')
            a = (unsigned char) (a + ('a' - 'A'));
        if (b >= 'A' && b <= 'Z')
            b = (unsigned char) (b + ('a' - 'A'));
        if (a != b)
            return false;
    }
    return true;
}

static const char *vpcm_expected_codec_name(v91_law_t law)
{
    return (law == V91_LAW_ALAW) ? "PCMA" : "PCMU";
}

static bool vpcm_is_expected_law_codec(const pjmedia_codec_info *fmt, v91_law_t law)
{
    if (!fmt || !fmt->encoding_name.ptr || fmt->encoding_name.slen <= 0)
        return false;
    return vpcm_str_ieq_prefix_n(fmt->encoding_name.ptr,
                                 fmt->encoding_name.slen,
                                 (law == V91_LAW_ALAW) ? "PCMA" : "PCMU");
}

static bool vpcm_is_g711_codec(const pjmedia_codec_info *fmt)
{
    if (!fmt || !fmt->encoding_name.ptr || fmt->encoding_name.slen <= 0)
        return false;
    return vpcm_str_ieq_prefix_n(fmt->encoding_name.ptr, fmt->encoding_name.slen, "PCMU")
        || vpcm_str_ieq_prefix_n(fmt->encoding_name.ptr, fmt->encoding_name.slen, "PCMA");
}

static pj_status_t vpcm_pjsip_register_g711_passthrough(v91_law_t law)
{
    pjmedia_codec_passthrough_setting setting;
    pjmedia_format fmts[2];
    unsigned cnt;

#if !PJMEDIA_HAS_PASSTHROUGH_CODECS
    (void) law;
    return PJ_ENOTSUP;
#else
    cnt = 0;
    memset(&setting, 0, sizeof(setting));
    memset(fmts, 0, sizeof(fmts));

    if (law == V91_LAW_ALAW || law == V91_LAW_ULAW) {
        if (law == V91_LAW_ULAW) {
            pjmedia_format_init_audio(&fmts[cnt++],
                                      PJMEDIA_FORMAT_PCMU,
                                      8000,
                                      1,
                                      8,
                                      20000,
                                      64000,
                                      64000);
        } else {
            pjmedia_format_init_audio(&fmts[cnt++],
                                      PJMEDIA_FORMAT_PCMA,
                                      8000,
                                      1,
                                      8,
                                      20000,
                                      64000,
                                      64000);
        }
    }

    setting.fmt_cnt = cnt;
    setting.fmts = fmts;
    setting.ilbc_mode = 20;

    /* Re-register explicitly for null-sound operation where ext fmt list is empty. */
    pjmedia_codec_passthrough_deinit();
    return pjmedia_codec_passthrough_init2(pjsua_get_pjmedia_endpt(), &setting);
#endif
}

static void vpcm_pjsip_on_call_state(pjsua_call_id call_id, pjsip_event *e)
{
    pjsua_call_info ci;
    pj_status_t st;

    (void) e;
    st = pjsua_call_get_info(call_id, &ci);
    if (st != PJ_SUCCESS)
        return;
    vpcm_log("PJSIP call state: call_id=%d state=%s status=%d",
             call_id,
             vpcm_pjsip_inv_state_to_str(ci.state),
             ci.last_status);
    if (ci.state == PJSIP_INV_STATE_CONFIRMED)
        g_vpcm_pjsip_runtime.call_confirmed = true;
    if (ci.state == PJSIP_INV_STATE_DISCONNECTED) {
        g_vpcm_pjsip_runtime.call_disconnected = true;
        g_vpcm_pjsip_runtime.disconnect_code = ci.last_status;
    }
}

static void vpcm_pjsip_on_incoming_call(pjsua_acc_id acc_id,
                                        pjsua_call_id call_id,
                                        pjsip_rx_data *rdata)
{
    pj_status_t st;
    pjsua_call_info ci;

    (void) acc_id;
    (void) rdata;

    st = pjsua_call_get_info(call_id, &ci);
    if (st == PJ_SUCCESS) {
        vpcm_log("PJSIP incoming INVITE: from=%.*s to=%.*s role=%s",
                 (int) ci.remote_info.slen,
                 ci.remote_info.ptr ? ci.remote_info.ptr : "",
                 (int) ci.local_info.slen,
                 ci.local_info.ptr ? ci.local_info.ptr : "",
                 vpcm_pjsip_role_to_str(g_vpcm_pjsip_runtime.role));
    } else {
        vpcm_log("PJSIP incoming INVITE: call_id=%d role=%s",
                 call_id,
                 vpcm_pjsip_role_to_str(g_vpcm_pjsip_runtime.role));
    }

    if (g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_ANSWERER
        && g_vpcm_pjsip_runtime.call_id == PJSUA_INVALID_ID) {
        g_vpcm_pjsip_runtime.call_id = call_id;
        st = pjsua_call_answer(call_id, 200, NULL, NULL);
        if (st != PJ_SUCCESS) {
            vpcm_log("PJSIP answer failed: status=%d", st);
            pjsua_call_hangup(call_id, 500, NULL, NULL);
        } else {
            vpcm_log("PJSIP answerer accepted call_id=%d", call_id);
        }
    } else {
        pjsua_call_answer(call_id, 486, NULL, NULL);
        vpcm_log("PJSIP incoming call rejected (busy): active_call=%d role=%s",
                 g_vpcm_pjsip_runtime.call_id,
                 vpcm_pjsip_role_to_str(g_vpcm_pjsip_runtime.role));
    }
}

static pj_status_t vpcm_pjsip_passthrough_put_frame(pjmedia_port *this_port,
                                                    pjmedia_frame *frame)
{
    vpcm_pjsip_passthrough_port_t *port = (vpcm_pjsip_passthrough_port_t *) this_port;
    pjmedia_frame_ext *tx_ext;

    PJ_UNUSED_ARG(frame);
    if (!port || !port->downstream_port)
        return PJ_EINVAL;

    vpcm_pjsip_modem_fill_rtp_tx_payload(port->tx_payload, (int) port->payload_samples_per_frame);

    /* Build the frame_ext directly in tx_ext_buf.  codec_encode() casts
     * the pjmedia_frame* argument straight to pjmedia_frame_ext* and
     * reads subframe_cnt / appends subframes from that address, so we must
     * pass the frame_ext buffer itself as the frame pointer. */
    memset(port->tx_ext_buf, 0, sizeof(port->tx_ext_buf));
    tx_ext = (pjmedia_frame_ext *) port->tx_ext_buf;
    tx_ext->base.type = PJMEDIA_FRAME_TYPE_EXTENDED;
    pjmedia_frame_ext_append_subframe(tx_ext,
                                      port->tx_payload,
                                      port->payload_samples_per_frame * 8U,
                                      port->payload_samples_per_frame);

    return pjmedia_port_put_frame(port->downstream_port, (pjmedia_frame *) tx_ext);
}

static pj_status_t vpcm_pjsip_passthrough_get_frame(pjmedia_port *this_port,
                                                    pjmedia_frame *frame)
{
    vpcm_pjsip_passthrough_port_t *port = (vpcm_pjsip_passthrough_port_t *) this_port;
    pj_status_t st;
    pjmedia_frame_ext *rx_ext;

    if (!port || !frame)
        return PJ_EINVAL;
    if (!port->downstream_port)
        return PJ_EINVAL;

    /* get_frame_ext() casts the pjmedia_frame* argument to pjmedia_frame_ext*
     * and appends subframes starting at that address + sizeof(pjmedia_frame_ext).
     * We must pass the rx_ext_buf buffer itself as the frame pointer so that
     * subframe data lands inside rx_ext_buf rather than past the end of a
     * stack-local pjmedia_frame. */
    rx_ext = (pjmedia_frame_ext *) port->rx_ext_buf;
    pj_bzero(rx_ext, sizeof(pjmedia_frame_ext));

    st = pjmedia_port_get_frame(port->downstream_port, (pjmedia_frame *) rx_ext);
    if (st == PJ_SUCCESS && rx_ext->base.type == PJMEDIA_FRAME_TYPE_EXTENDED) {
        unsigned i;
        for (i = 0; i < rx_ext->subframe_cnt; i++) {
            pjmedia_frame_ext_subframe *sf = pjmedia_frame_ext_get_subframe(rx_ext, i);
            unsigned sf_bytes;
            if (!sf)
                continue;
            sf_bytes = ((unsigned) sf->bitlen + 7U) >> 3;
            if (sf_bytes > 0)
                vpcm_pjsip_modem_handle_rtp_rx_payload((const uint8_t *) sf->data, (int) sf_bytes);
        }
    } else if (st == PJ_SUCCESS && rx_ext->base.type == PJMEDIA_FRAME_TYPE_AUDIO
               && rx_ext->base.buf && rx_ext->base.size > 0) {
        vpcm_pjsip_modem_handle_rtp_rx_payload((const uint8_t *) rx_ext->base.buf,
                                               (int) rx_ext->base.size);
    }

    frame->type = PJMEDIA_FRAME_TYPE_NONE;
    frame->size = 0;
    return PJ_SUCCESS;
}

static pj_status_t vpcm_pjsip_passthrough_on_destroy(pjmedia_port *this_port)
{
    vpcm_pjsip_passthrough_port_t *port = (vpcm_pjsip_passthrough_port_t *) this_port;
    if (port && port->pool)
        pj_pool_release(port->pool);
    return PJ_SUCCESS;
}

static pj_status_t vpcm_pjsip_passthrough_port_create(const pjmedia_port *source_port,
                                                      unsigned payload_samples_per_frame,
                                                      pjmedia_port **p_port)
{
    pjmedia_endpt *endpt = pjsua_get_pjmedia_endpt();
    pj_pool_t *pool;
    vpcm_pjsip_passthrough_port_t *port;

    if (!source_port || !p_port || !endpt)
        return PJ_EINVAL;
    if (source_port->port_data.pdata == &g_vpcm_passthrough_port_marker) {
        *p_port = (pjmedia_port *) source_port;
        return PJ_SUCCESS;
    }
    if (payload_samples_per_frame == 0 || payload_samples_per_frame > 320)
        return PJ_EINVAL;

    pool = pjmedia_endpt_create_pool(endpt, "vpcm-pass-port", 1024, 1024);
    if (!pool)
        return PJ_ENOMEM;

    port = PJ_POOL_ZALLOC_T(pool, vpcm_pjsip_passthrough_port_t);
    port->pool = pool;
    port->downstream_port = (pjmedia_port *) source_port;
    port->payload_samples_per_frame = payload_samples_per_frame;
    port->base.info = source_port->info;
    port->base.port_data.pdata = &g_vpcm_passthrough_port_marker;
    port->base.put_frame = &vpcm_pjsip_passthrough_put_frame;
    port->base.get_frame = &vpcm_pjsip_passthrough_get_frame;
    port->base.on_destroy = &vpcm_pjsip_passthrough_on_destroy;
    *p_port = &port->base;
    return PJ_SUCCESS;
}

static void vpcm_pjsip_on_stream_created2(pjsua_call_id call_id,
                                          pjsua_on_stream_created_param *param)
{
    pjmedia_port *pass_port = NULL;
    unsigned payload_samples_per_frame = 160;

    if (!param)
        return;
    if (call_id != g_vpcm_pjsip_runtime.call_id)
        return;
    if (param->stream_idx != 0)
        return;

    if (param->port
        && param->port->info.fmt.type == PJMEDIA_TYPE_AUDIO
        && param->port->info.fmt.detail_type == PJMEDIA_FORMAT_DETAIL_AUDIO
        && PJMEDIA_PIA_SPF(&param->port->info) > 0
        && PJMEDIA_PIA_SPF(&param->port->info) <= 320) {
        payload_samples_per_frame = PJMEDIA_PIA_SPF(&param->port->info);
    }

    if (param->port
        && param->port->port_data.pdata != &g_vpcm_passthrough_port_marker
        && vpcm_pjsip_passthrough_port_create(param->port,
                                              payload_samples_per_frame,
                                              &pass_port) == PJ_SUCCESS) {
        param->port = pass_port;
        param->destroy_port = PJ_TRUE;
    }

    g_vpcm_pjsip_runtime.stream_port = param->port;
    g_vpcm_pjsip_runtime.stream_ready = (param->port != NULL);
    if (param->stream) {
        g_vpcm_pjsip_runtime.media_tp = pjmedia_stream_get_transport(param->stream);
        g_vpcm_pjsip_runtime.media_tp_ready = (g_vpcm_pjsip_runtime.media_tp != NULL);
    }
    vpcm_log("PJSIP stream created: call_id=%d stream_idx=%u stream_port=%s",
             call_id,
             param->stream_idx,
             param->port ? "ready" : "none");
}

static void vpcm_pjsip_on_call_media_state(pjsua_call_id call_id)
{
    pjsua_call_info ci;
    unsigned i;

    if (pjsua_call_get_info(call_id, &ci) != PJ_SUCCESS)
        return;

    for (i = 0; i < ci.media_cnt; i++) {
        pjsua_stream_info si;
        char codec_name[32];
        int n;

        if (ci.media[i].type != PJMEDIA_TYPE_AUDIO)
            continue;
        if (ci.media[i].status != PJSUA_CALL_MEDIA_ACTIVE
            && ci.media[i].status != PJSUA_CALL_MEDIA_REMOTE_HOLD) {
            continue;
        }
        if (pjsua_call_get_stream_info(call_id, i, &si) != PJ_SUCCESS)
            continue;
        if (si.type != PJMEDIA_TYPE_AUDIO)
            continue;

        n = si.info.aud.fmt.encoding_name.slen;
        if (n < 0)
            n = 0;
        if (n >= (int) sizeof(codec_name))
            n = (int) sizeof(codec_name) - 1;
        memcpy(codec_name, si.info.aud.fmt.encoding_name.ptr, (size_t) n);
        codec_name[n] = '\0';

        g_vpcm_pjsip_runtime.media_codec_checked = true;
        if (!vpcm_is_g711_codec(&si.info.aud.fmt)
            || !vpcm_is_expected_law_codec(&si.info.aud.fmt, g_vpcm_pjsip_runtime.expected_law)) {
            g_vpcm_pjsip_runtime.media_codec_ok = false;
            vpcm_log("PJSIP media reject: negotiated codec=%s pt=%u (expected %s passthrough)",
                     codec_name,
                     si.info.aud.fmt.pt,
                     vpcm_expected_codec_name(g_vpcm_pjsip_runtime.expected_law));
            pjsua_call_hangup(call_id, 488, NULL, NULL);
            return;
        }

        g_vpcm_pjsip_runtime.media_codec_ok = true;
        g_vpcm_pjsip_runtime.media_ready = true;
        g_vpcm_pjsip_runtime.call_conf_port = pjsua_call_get_conf_port(call_id);
        if (!g_vpcm_pjsip_runtime.media_clock_wired
            && g_vpcm_pjsip_runtime.call_conf_port != PJSUA_INVALID_ID) {
            pj_status_t st_a;
            pj_status_t st_b;

            st_a = pjsua_conf_connect(g_vpcm_pjsip_runtime.call_conf_port, 0);
            st_b = pjsua_conf_connect(0, g_vpcm_pjsip_runtime.call_conf_port);
            if (st_a == PJ_SUCCESS && st_b == PJ_SUCCESS) {
                g_vpcm_pjsip_runtime.media_clock_wired = true;
                vpcm_log("PJSIP media clock wired: conf %d <-> 0",
                         g_vpcm_pjsip_runtime.call_conf_port);
            } else {
                vpcm_log("PJSIP media clock wiring failed: conf=%d st_a=%d st_b=%d",
                         g_vpcm_pjsip_runtime.call_conf_port, st_a, st_b);
            }
        }
        vpcm_log("PJSIP media active: codec=%s pt=%u (G.711 passthrough)",
                 codec_name,
                 si.info.aud.fmt.pt);
        return;
    }
}

static pj_status_t vpcm_pjsip_set_g711_only_priorities(v91_law_t law)
{
    pjsua_codec_info codecs[64];
    unsigned count;
    unsigned i;
    bool matched = false;

    count = (unsigned) PJ_ARRAY_SIZE(codecs);
    if (pjsua_enum_codecs(codecs, &count) != PJ_SUCCESS)
        return PJ_EUNKNOWN;
    if (g_vpcm_verbose || g_vpcm_session_diag)
        vpcm_log("PJSIP codec enum count=%u", count);
    for (i = 0; i < count; i++) {
        pj_str_t cid;
        bool wanted;

        cid = codecs[i].codec_id;
        if (g_vpcm_verbose || g_vpcm_session_diag)
            vpcm_log("PJSIP codec[%u]=%.*s", i, cid.slen, cid.ptr ? cid.ptr : "");
        wanted = (cid.ptr != NULL
                  && cid.slen > 0
                  && vpcm_str_ieq_prefix_n(cid.ptr, cid.slen, vpcm_expected_codec_name(law)));
        if (wanted)
            matched = true;
        if (pjsua_codec_set_priority(&cid, wanted ? 255 : 0) != PJ_SUCCESS)
            return PJ_EUNKNOWN;
    }
    if (!matched) {
        vpcm_log("PJSIP codec policy could not find expected codec prefix %s",
                 vpcm_expected_codec_name(law));
    }
    return matched ? PJ_SUCCESS : PJ_ENOTFOUND;
}

static const char *vpcm_v34_rx_stage_to_str(int stage)
{
    switch (stage)
    {
    case V34_RX_STAGE_INFO0: return "INFO0";
    case V34_RX_STAGE_TONE_A: return "TONE_A";
    case V34_RX_STAGE_TONE_B: return "TONE_B";
    case V34_RX_STAGE_L1_L2: return "L1_L2";
    case V34_RX_STAGE_INFO1C: return "INFO1C";
    case V34_RX_STAGE_INFO1A: return "INFO1A";
    case V34_RX_STAGE_PHASE3_TRAINING: return "PHASE3_TRAINING";
    default: return "other";
    }
}

static const char *vpcm_v34_tx_stage_to_str(int stage)
{
    switch (stage)
    {
    case V34_TX_STAGE_INITIAL_PREAMBLE: return "INITIAL_PREAMBLE";
    case V34_TX_STAGE_INFO0: return "INFO0";
    case V34_TX_STAGE_INITIAL_A: return "INITIAL_A";
    case V34_TX_STAGE_FIRST_A: return "FIRST_A";
    case V34_TX_STAGE_FIRST_NOT_A: return "FIRST_NOT_A";
    case V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN: return "FIRST_NOT_A_REV_SEEN";
    case V34_TX_STAGE_SECOND_A: return "SECOND_A";
    case V34_TX_STAGE_L1: return "L1";
    case V34_TX_STAGE_L2: return "L2";
    case V34_TX_STAGE_POST_L2_A: return "POST_L2_A";
    case V34_TX_STAGE_POST_L2_NOT_A: return "POST_L2_NOT_A";
    case V34_TX_STAGE_A_SILENCE: return "A_SILENCE";
    case V34_TX_STAGE_PRE_INFO1_A: return "PRE_INFO1_A";
    case V34_TX_STAGE_V90_WAIT_RX_L2: return "V90_WAIT_RX_L2";
    case V34_TX_STAGE_V90_WAIT_TONE_A: return "V90_WAIT_TONE_A";
    case V34_TX_STAGE_V90_WAIT_TONE_A_REV: return "V90_WAIT_TONE_A_REV";
    case V34_TX_STAGE_V90_B_REV_DELAY: return "V90_B_REV_DELAY";
    case V34_TX_STAGE_V90_B_REV_10MS: return "V90_B_REV_10MS";
    case V34_TX_STAGE_V90_WAIT_INFO1A: return "V90_WAIT_INFO1A";
    case V34_TX_STAGE_V90_PHASE2_B: return "V90_PHASE2_B";
    case V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN: return "V90_PHASE2_B_INFO0_SEEN";
    case V34_TX_STAGE_INFO1: return "INFO1";
    case V34_TX_STAGE_FIRST_B: return "FIRST_B";
    case V34_TX_STAGE_FIRST_B_INFO_SEEN: return "FIRST_B_INFO_SEEN";
    case V34_TX_STAGE_FIRST_NOT_B_WAIT: return "FIRST_NOT_B_WAIT";
    case V34_TX_STAGE_FIRST_NOT_B: return "FIRST_NOT_B";
    case V34_TX_STAGE_FIRST_B_SILENCE: return "FIRST_B_SILENCE";
    case V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE: return "FIRST_B_POST_REV_SILENCE";
    case V34_TX_STAGE_SECOND_B: return "SECOND_B";
    case V34_TX_STAGE_SECOND_B_WAIT: return "SECOND_B_WAIT";
    case V34_TX_STAGE_SECOND_NOT_B: return "SECOND_NOT_B";
    case V34_TX_STAGE_INFO0_RETRY: return "INFO0_RETRY";
    case V34_TX_STAGE_FIRST_S: return "FIRST_S";
    default: {
        static char buf[32];
        snprintf(buf, sizeof(buf), "stage_%d", stage);
        return buf;
    }
    }
}

static int vpcm_v34_dummy_get_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void vpcm_v34_dummy_put_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

#define VPCM_V90_INFO_FILL_AND_SYNC_BITS  0x4EF
#define VPCM_V90_INFO0A_BITS              49
#define VPCM_V90_INFO1A_BITS              70

typedef struct {
    bool support_2743;
    bool support_2800;
    bool support_3429;
    bool support_3000_low;
    bool support_3000_high;
    bool support_3200_low;
    bool support_3200_high;
    bool rate_3429_allowed;
    bool support_power_reduction;
    uint8_t max_baud_rate_difference;
    bool from_cme_modem;
    bool support_1664_point_constellation;
    uint8_t tx_clock_source;
    bool acknowledge_info0d;
} vpcm_v90_info0a_t;

typedef struct {
    uint8_t md;
    uint8_t u_info;
    uint8_t upstream_symbol_rate_code;
    uint8_t downstream_rate_code;
    int16_t freq_offset;
} vpcm_v90_info1a_t;

static void vpcm_bits_put(uint8_t *buf, int *bit_pos, uint32_t value, int bits)
{
    int i;

    for (i = 0; i < bits; i++) {
        int pos = *bit_pos + i;
        if (value & (1U << i))
            buf[pos >> 3] |= (uint8_t) (1U << (pos & 7));
    }
    *bit_pos += bits;
}

static uint16_t vpcm_crc_bit_block(const uint8_t buf[], int first_bit, int last_bit, uint16_t crc)
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

static void vpcm_packed_bits_to_str(const uint8_t *buf, int bit_count, char *out, size_t out_size)
{
    int i;
    size_t pos;

    pos = 0;
    if (out_size == 0)
        return;
    for (i = 0; i < bit_count && pos + 1 < out_size; i++)
        out[pos++] = (buf[i >> 3] & (1U << (i & 7))) ? '1' : '0';
    out[pos] = '\0';
}

static void vpcm_v90_info0a_init(vpcm_v90_info0a_t *info)
{
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
    info->tx_clock_source = 0; /* internal */
    info->acknowledge_info0d = false;
}

static void vpcm_v90_info1a_init(vpcm_v90_info1a_t *info)
{
    memset(info, 0, sizeof(*info));
    info->md = 0;
    info->u_info = 78;
    info->upstream_symbol_rate_code = 4; /* 3200 baud */
    info->downstream_rate_code = 6;      /* 8000 PCM */
    info->freq_offset = 0;
}

static bool vpcm_v90_build_info0a_bits(uint8_t *buf, int buf_len, const vpcm_v90_info0a_t *info)
{
    int bit_pos;
    uint16_t crc;

    if (buf_len < ((VPCM_V90_INFO0A_BITS + 7) / 8))
        return false;
    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    vpcm_bits_put(buf, &bit_pos, VPCM_V90_INFO_FILL_AND_SYNC_BITS, 12);
    vpcm_bits_put(buf, &bit_pos, info->support_2743 ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_2800 ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_3429 ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_3000_low ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_3000_high ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_3200_low ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_3200_high ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->rate_3429_allowed ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_power_reduction ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->max_baud_rate_difference & 0x7U, 3);
    vpcm_bits_put(buf, &bit_pos, info->from_cme_modem ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->support_1664_point_constellation ? 1U : 0U, 1);
    vpcm_bits_put(buf, &bit_pos, info->tx_clock_source & 0x3U, 2);
    vpcm_bits_put(buf, &bit_pos, info->acknowledge_info0d ? 1U : 0U, 1);
    crc = vpcm_crc_bit_block(buf, 12, 28, 0xFFFF);
    vpcm_bits_put(buf, &bit_pos, crc, 16);
    vpcm_bits_put(buf, &bit_pos, 0xFU, 4);
    return bit_pos == VPCM_V90_INFO0A_BITS;
}

static bool vpcm_v90_build_info1a_bits(uint8_t *buf, int buf_len, const vpcm_v90_info1a_t *info)
{
    int bit_pos;
    uint16_t crc;
    uint16_t freq_bits;

    if (buf_len < ((VPCM_V90_INFO1A_BITS + 7) / 8))
        return false;
    memset(buf, 0, (size_t) buf_len);
    bit_pos = 0;
    vpcm_bits_put(buf, &bit_pos, VPCM_V90_INFO_FILL_AND_SYNC_BITS, 12);
    vpcm_bits_put(buf, &bit_pos, 0, 6); /* reserved */
    vpcm_bits_put(buf, &bit_pos, info->md & 0x7FU, 7);
    vpcm_bits_put(buf, &bit_pos, info->u_info & 0x7FU, 7);
    vpcm_bits_put(buf, &bit_pos, 0, 2); /* reserved */
    vpcm_bits_put(buf, &bit_pos, info->upstream_symbol_rate_code & 0x7U, 3);
    vpcm_bits_put(buf, &bit_pos, info->downstream_rate_code & 0x7U, 3);
    freq_bits = (uint16_t) info->freq_offset;
    if (info->freq_offset < 0)
        freq_bits = (uint16_t) (0x400 + info->freq_offset);
    vpcm_bits_put(buf, &bit_pos, freq_bits & 0x3FFU, 10);
    crc = vpcm_crc_bit_block(buf, 12, 49, 0xFFFF);
    vpcm_bits_put(buf, &bit_pos, crc, 16);
    vpcm_bits_put(buf, &bit_pos, 0xFU, 4);
    return bit_pos == VPCM_V90_INFO1A_BITS;
}

static void vpcm_log(const char *fmt, ...)
{
    va_list ap;

    fprintf(stderr, "[VPCM] ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void vpcm_log_e2e_phase(const char *phase, const char *fmt, ...)
{
    va_list ap;

    if (!g_vpcm_compact_e2e && !g_vpcm_session_diag && !g_vpcm_verbose)
        return;

    fprintf(stderr, "[VPCM] [E2E:%s] ", phase);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static bool vpcm_info_frames_equal(const v91_info_frame_t *a, const v91_info_frame_t *b)
{
    return a->reserved_12_25 == b->reserved_12_25
        && a->request_default_dil == b->request_default_dil
        && a->request_control_channel == b->request_control_channel
        && a->acknowledge_info_frame == b->acknowledge_info_frame
        && a->reserved_29_32 == b->reserved_29_32
        && a->max_tx_power == b->max_tx_power
        && a->power_measured_after_digital_impairments == b->power_measured_after_digital_impairments
        && a->tx_uses_alaw == b->tx_uses_alaw
        && a->request_transparent_mode == b->request_transparent_mode
        && a->cleardown_if_transparent_denied == b->cleardown_if_transparent_denied;
}

static bool vpcm_cp_frames_equal(const vpcm_cp_frame_t *a, const vpcm_cp_frame_t *b)
{
    return a->transparent_mode_granted == b->transparent_mode_granted
        && a->v90_compatibility == b->v90_compatibility
        && a->drn == b->drn
        && a->acknowledge == b->acknowledge
        && a->constellation_count == b->constellation_count
        && memcmp(a->dfi, b->dfi, sizeof(a->dfi)) == 0
        && memcmp(a->masks, b->masks, sizeof(a->masks)) == 0;
}

static bool test_vpcm_cp_robbed_bit_safe_profile(void)
{
    vpcm_cp_frame_t tx;
    vpcm_cp_frame_t rx;
    uint8_t bits[VPCM_CP_MAX_BITS];
    int nbits;
    int idx;

    vpcm_log("Test: VPCM robbed-bit-safe CP profile");

    vpcm_cp_init_robbed_bit_safe_profile(&tx, vpcm_cp_recommended_robbed_bit_drn(), false);
    if (!vpcm_cp_validate(&tx, NULL, 0)) {
        fprintf(stderr, "robbed-bit-safe CP profile validation failed\n");
        return false;
    }
    if (tx.constellation_count != 2 || tx.dfi[VPCM_CP_FRAME_INTERVALS - 1] != 1) {
        fprintf(stderr, "robbed-bit-safe CP profile shape mismatch\n");
        return false;
    }
    if (vpcm_cp_mask_population(tx.masks[0]) != 128 || vpcm_cp_mask_population(tx.masks[1]) != 64) {
        fprintf(stderr, "robbed-bit-safe CP profile population mismatch\n");
        return false;
    }
    for (idx = 0; idx < VPCM_CP_MASK_BITS; idx++) {
        bool expected;

        expected = ((idx & 1) != 0);
        if (vpcm_cp_mask_get(tx.masks[1], idx) != expected) {
            fprintf(stderr, "robbed-bit-safe CP profile odd-Ucode mask mismatch at %d\n", idx);
            return false;
        }
    }
    if (!vpcm_cp_encode_bits(&tx, bits, &nbits) || !vpcm_cp_decode_bits(bits, nbits, &rx)) {
        fprintf(stderr, "robbed-bit-safe CP profile encode/decode failed\n");
        return false;
    }
    if (!vpcm_cp_frames_equal(&tx, &rx)) {
        fprintf(stderr, "robbed-bit-safe CP profile did not round-trip\n");
        return false;
    }

    vpcm_log("PASS: VPCM robbed-bit-safe CP profile (drn=%u, rate=%.0f bps, robbed slot population=%d)",
             tx.drn,
             vpcm_cp_drn_to_bps(tx.drn),
             vpcm_cp_mask_population(tx.masks[1]));
    return true;
}

static void vpcm_bits_to_str(char *out, size_t out_len, const uint8_t *bits, int nbits)
{
    int i;
    size_t pos;

    if (out_len == 0)
        return;
    pos = 0;
    for (i = 0; i < nbits && pos + 1 < out_len; i++)
        out[pos++] = bits[i] ? '1' : '0';
    out[pos] = '\0';
}

static const char *vpcm_info_law_to_str(bool tx_uses_alaw)
{
    return tx_uses_alaw ? "A-law" : "u-law";
}

static void vpcm_log_info_compare_row(const char *field, const char *tx, const char *rx)
{
    vpcm_log("| %-10s | %-18s | %-18s |", field, tx, rx);
}

static void vpcm_format_info_status(char *buf, size_t len, bool ok)
{
    snprintf(buf, len, "%s", ok ? "correct" : "incorrect");
}

static void vpcm_format_info_reserved14(char *buf, size_t len, uint16_t value)
{
    snprintf(buf, len, "0x%04x", value);
}

static void vpcm_format_info_reserved4(char *buf, size_t len, uint8_t value)
{
    snprintf(buf, len, "0x%x", value & 0x0F);
}

static void vpcm_format_info_dil(char *buf, size_t len, bool request_default_dil)
{
    snprintf(buf, len, "%s", request_default_dil ? "default" : "non-default");
}

static void vpcm_format_info_request(char *buf, size_t len, bool requested, const char *true_value, const char *false_value)
{
    snprintf(buf, len, "%s", requested ? true_value : false_value);
}

static void vpcm_format_info_power(char *buf, size_t len, uint8_t max_tx_power)
{
    if (max_tx_power == 0) {
        snprintf(buf, len, "unspecified");
    } else {
        snprintf(buf, len, "%u (%.1f dBm0)", max_tx_power, -0.5 * (double) (max_tx_power + 1));
    }
}

static void vpcm_log_info_diag_compare(const v91_info_diag_t *tx, const v91_info_diag_t *rx)
{
    char tx_bits[V91_INFO_SYMBOLS + 1];
    char rx_bits[V91_INFO_SYMBOLS + 1];
    char tx_buf[64];
    char rx_buf[64];

    vpcm_bits_to_str(tx_bits, sizeof(tx_bits), tx->bits, V91_INFO_SYMBOLS);
    vpcm_bits_to_str(rx_bits, sizeof(rx_bits), rx->bits, V91_INFO_SYMBOLS);

    vpcm_log("+------------+--------------------+--------------------+");
    vpcm_log("| Field      | TX                 | RX                 |");
    vpcm_log("+------------+--------------------+--------------------+");
    vpcm_format_info_status(tx_buf, sizeof(tx_buf), tx->fill_ok);
    vpcm_format_info_status(rx_buf, sizeof(rx_buf), rx->fill_ok);
    vpcm_log_info_compare_row("Fill", tx_buf, rx_buf);
    vpcm_format_info_status(tx_buf, sizeof(tx_buf), tx->sync_ok);
    vpcm_format_info_status(rx_buf, sizeof(rx_buf), rx->sync_ok);
    vpcm_log_info_compare_row("FS", tx_buf, rx_buf);
    vpcm_format_info_reserved14(tx_buf, sizeof(tx_buf), tx->frame.reserved_12_25);
    vpcm_format_info_reserved14(rx_buf, sizeof(rx_buf), rx->frame.reserved_12_25);
    vpcm_log_info_compare_row("RSVD12:25", tx_buf, rx_buf);
    vpcm_format_info_dil(tx_buf, sizeof(tx_buf), tx->frame.request_default_dil);
    vpcm_format_info_dil(rx_buf, sizeof(rx_buf), rx->frame.request_default_dil);
    vpcm_log_info_compare_row("DIL", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.request_control_channel, "request", "none");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.request_control_channel, "request", "none");
    vpcm_log_info_compare_row("CC", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.acknowledge_info_frame, "yes", "no");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.acknowledge_info_frame, "yes", "no");
    vpcm_log_info_compare_row("ACK", tx_buf, rx_buf);
    vpcm_format_info_reserved4(tx_buf, sizeof(tx_buf), tx->frame.reserved_29_32);
    vpcm_format_info_reserved4(rx_buf, sizeof(rx_buf), rx->frame.reserved_29_32);
    vpcm_log_info_compare_row("RSVD29:32", tx_buf, rx_buf);
    vpcm_format_info_power(tx_buf, sizeof(tx_buf), tx->frame.max_tx_power);
    vpcm_format_info_power(rx_buf, sizeof(rx_buf), rx->frame.max_tx_power);
    vpcm_log_info_compare_row("MaxPwr", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.power_measured_after_digital_impairments, "receiver", "terminals");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.power_measured_after_digital_impairments, "receiver", "terminals");
    vpcm_log_info_compare_row("PwrRef", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%s", vpcm_info_law_to_str(tx->frame.tx_uses_alaw));
    snprintf(rx_buf, sizeof(rx_buf), "%s", vpcm_info_law_to_str(rx->frame.tx_uses_alaw));
    vpcm_log_info_compare_row("PCM", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.request_transparent_mode, "transparent", "normal");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.request_transparent_mode, "transparent", "normal");
    vpcm_log_info_compare_row("Mode", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.cleardown_if_transparent_denied, "cleardown", "continue");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.cleardown_if_transparent_denied, "cleardown", "continue");
    vpcm_log_info_compare_row("Deny", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "0x%04x", tx->crc_field);
    snprintf(rx_buf, sizeof(rx_buf), "0x%04x", rx->crc_field);
    vpcm_log_info_compare_row("CRC", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "0x%04x", tx->crc_remainder);
    snprintf(rx_buf, sizeof(rx_buf), "0x%04x", rx->crc_remainder);
    vpcm_log_info_compare_row("CRC rem", tx_buf, rx_buf);
    vpcm_format_info_status(tx_buf, sizeof(tx_buf), tx->valid);
    vpcm_format_info_status(rx_buf, sizeof(rx_buf), rx->valid);
    vpcm_log_info_compare_row("INFO", tx_buf, rx_buf);
    vpcm_log("+------------+--------------------+--------------------+");
    vpcm_log("INFO bits TX: %s", tx_bits);
    vpcm_log("INFO bits RX: %s", rx_bits);
    if (g_vpcm_verbose)
        vpcm_log("INFO codeword[0..7] TX/RX: %02x %02x %02x %02x %02x %02x %02x %02x / %02x %02x %02x %02x %02x %02x %02x %02x",
                 tx->codewords[0], tx->codewords[1], tx->codewords[2], tx->codewords[3],
                 tx->codewords[4], tx->codewords[5], tx->codewords[6], tx->codewords[7],
                 rx->codewords[0], rx->codewords[1], rx->codewords[2], rx->codewords[3],
                 rx->codewords[4], rx->codewords[5], rx->codewords[6], rx->codewords[7]);
}

static void vpcm_trace(const char *fmt, ...)
{
    va_list ap;

    if (!g_vpcm_verbose)
        return;
    fprintf(stderr, "[VPCM:trace] ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void vpcm_log_v8_result(const char *side, const v8_parms_t *result)
{
    char mods[128];

    mods[0] = '\0';
    if (result->jm_cm.modulations & V8_MOD_V22)
        strncat(mods, mods[0] ? "|V22" : "V22", sizeof(mods) - strlen(mods) - 1);
    if (result->jm_cm.modulations & V8_MOD_V34)
        strncat(mods, mods[0] ? "|V34" : "V34", sizeof(mods) - strlen(mods) - 1);
    if (result->jm_cm.modulations & V8_MOD_V90)
        strncat(mods, mods[0] ? "|V90" : "V90", sizeof(mods) - strlen(mods) - 1);
    if (result->jm_cm.modulations & V8_MOD_V92)
        strncat(mods, mods[0] ? "|V92" : "V92", sizeof(mods) - strlen(mods) - 1);
    if (mods[0] == '\0')
        strlcpy(mods, "none", sizeof(mods));

    vpcm_log("V.8 %s result: status=%s (%d) call=%s mods=%s pcm=0x%X pstn=0x%X",
             side,
             v8_status_to_str(result->status), result->status,
             v8_call_function_to_str(result->jm_cm.call_function),
             mods,
             result->jm_cm.pcm_modem_availability,
             result->jm_cm.pstn_access);
}

static uint32_t prng_next(uint32_t *state)
{
    uint32_t x;

    x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static bool run_v91_info_roundtrip_case(v91_state_t *tx,
                                        v91_state_t *rx,
                                        const char *label,
                                        const v91_info_frame_t *expected_info,
                                        bool require_ack)
{
    uint8_t info_buf[V91_INFO_SYMBOLS];
    v91_info_frame_t info_rx;
    v91_info_diag_t tx_diag;
    v91_info_diag_t rx_diag;
    char validate_reason[128];

    vpcm_log("Case: %s", label);
    if (!v91_info_frame_validate(expected_info, validate_reason, sizeof(validate_reason))) {
        fprintf(stderr, "V.91 INFO validation failed for %s: %s\n", label, validate_reason);
        return false;
    }
    if (!v91_info_build_diag(tx, expected_info, &tx_diag)) {
        fprintf(stderr, "V.91 INFO TX diag build failed for %s\n", label);
        return false;
    }
    if (v91_tx_info_codewords(tx, info_buf, (int) sizeof(info_buf), expected_info) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "V.91 INFO encode length mismatch for %s\n", label);
        return false;
    }
    if (!tx->last_tx_info_valid || !vpcm_info_frames_equal(expected_info, &tx->last_tx_info)) {
        fprintf(stderr, "V.91 TX tracking mismatch for %s\n", label);
        return false;
    }
    if (!v91_info_decode_diag(rx, info_buf, (int) sizeof(info_buf), &rx_diag)) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 INFO decode failed for %s\n", label);
        return false;
    }
    info_rx = rx_diag.frame;
    if (!rx->last_rx_info_valid || !vpcm_info_frames_equal(expected_info, &rx->last_rx_info)) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 RX tracking mismatch for %s\n", label);
        return false;
    }
    if (!vpcm_info_frames_equal(expected_info, &info_rx)) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 INFO round-trip mismatch for %s\n", label);
        return false;
    }
    if (require_ack && !info_rx.acknowledge_info_frame) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 INFO acknowledgement bit missing for %s\n", label);
        return false;
    }
    vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
    return true;
}

static bool vpcm_dil_desc_equal(const v91_dil_desc_t *a, const v91_dil_desc_t *b)
{
    return a->n == b->n
        && a->lsp == b->lsp
        && a->ltp == b->ltp
        && memcmp(a->sp, b->sp, sizeof(a->sp)) == 0
        && memcmp(a->tp, b->tp, sizeof(a->tp)) == 0
        && memcmp(a->h, b->h, sizeof(a->h)) == 0
        && memcmp(a->ref, b->ref, sizeof(a->ref)) == 0
        && memcmp(a->train_u, b->train_u, sizeof(a->train_u)) == 0;
}

static int vpcm_v91_gpc_scramble_bit(uint32_t *reg, int in_bit)
{
    int out_bit;

    out_bit = (in_bit ^ (int) (*reg >> 17) ^ (int) (*reg >> (23 - 1))) & 1;
    *reg = (*reg << 1) | (uint32_t) out_bit;
    return out_bit;
}

static int vpcm_v91_gpc_descramble_bit(uint32_t *reg, int in_bit)
{
    int out_bit;

    out_bit = (in_bit ^ (int) (*reg >> 17) ^ (int) (*reg >> (23 - 1))) & 1;
    *reg = (*reg << 1) | (uint32_t) in_bit;
    return out_bit;
}

static bool vpcm_v91_decode_scrambled_codewords_to_bits(const uint8_t *codewords,
                                                        int ncodewords,
                                                        uint8_t *bits_out)
{
    uint8_t scrambled_bits[VPCM_CP_MAX_BITS];
    uint32_t scramble_reg;
    int sign;
    int prev_sign;
    int i;

    if (ncodewords <= 0 || ncodewords > VPCM_CP_MAX_BITS)
        return false;
    prev_sign = 0;
    for (i = 0; i < ncodewords; i++) {
        sign = (codewords[i] & 0x80) ? 1 : 0;
        scrambled_bits[i] = (uint8_t) (sign ^ prev_sign);
        prev_sign = sign;
    }
    scramble_reg = 0;
    for (i = 0; i < ncodewords; i++)
        bits_out[i] = (uint8_t) vpcm_v91_gpc_descramble_bit(&scramble_reg, scrambled_bits[i] & 1U);
    return true;
}

static bool vpcm_expect_scrambled_ones_sequence(v91_law_t law,
                                                const uint8_t *actual,
                                                int nsymbols,
                                                uint32_t initial_scramble_reg,
                                                int initial_diff_sign,
                                                uint32_t *scramble_reg_out,
                                                int *diff_sign_out)
{
    uint32_t scramble_reg;
    int diff_sign;
    int i;

    scramble_reg = initial_scramble_reg;
    diff_sign = initial_diff_sign;
    for (i = 0; i < nsymbols; i++) {
        diff_sign ^= vpcm_v91_gpc_scramble_bit(&scramble_reg, 1);
        if (actual[i] != v91_ucode_to_codeword(law, 66, diff_sign != 0))
            return false;
    }
    if (scramble_reg_out)
        *scramble_reg_out = scramble_reg;
    if (diff_sign_out)
        *diff_sign_out = diff_sign;
    return true;
}

static bool test_v91_default_dil(v91_law_t law)
{
    v91_state_t tx;
    v91_dil_desc_t dil;
    uint8_t codewords[V91_DEFAULT_DIL_SYMBOLS];
    int count;
    int i;
    int first_seg_start;
    int second_seg_start;
    int last_seg_start;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_default_dil_init(&dil);
    vpcm_log("Test: V.91 default DIL over raw-G.711 (%s)", vpcm_law_to_str(law));

    if (dil.n != V91_DEFAULT_DIL_SEGMENTS
        || dil.lsp != 12
        || dil.ltp != 12) {
        fprintf(stderr, "V.91 default DIL descriptor header mismatch\n");
        return false;
    }
    for (i = 0; i < 8; i++) {
        if (dil.h[i] != 1 || dil.ref[i] != 0) {
            fprintf(stderr, "V.91 default DIL H/REF mismatch at uchord %d\n", i);
            return false;
        }
    }
    if (dil.train_u[0] != 124 || dil.train_u[1] != 0 || dil.train_u[2] != 123 || dil.train_u[124] != 62) {
        fprintf(stderr, "V.91 default DIL training-Ucode sequence mismatch\n");
        return false;
    }
    if (v91_dil_symbol_count(&dil) != V91_DEFAULT_DIL_SYMBOLS) {
        fprintf(stderr, "V.91 default DIL symbol count mismatch\n");
        return false;
    }

    count = v91_tx_dil_codewords(&tx, codewords, (int) sizeof(codewords), &dil);
    if (count != V91_DEFAULT_DIL_SYMBOLS) {
        fprintf(stderr, "V.91 default DIL encode length mismatch: %d\n", count);
        return false;
    }
    if (!tx.last_tx_dil_valid || !vpcm_dil_desc_equal(&dil, &tx.last_tx_dil)) {
        fprintf(stderr, "V.91 default DIL TX tracking mismatch\n");
        return false;
    }

    first_seg_start = 0;
    second_seg_start = V91_DEFAULT_DIL_SEGMENT_SYMBOLS;
    last_seg_start = V91_DEFAULT_DIL_SYMBOLS - V91_DEFAULT_DIL_SEGMENT_SYMBOLS;
    for (i = 0; i < 6; i++) {
        if (codewords[first_seg_start + i] != v91_ucode_to_codeword(law, 124, false)) {
            fprintf(stderr, "V.91 default DIL first segment negative half mismatch at %d\n", i);
            return false;
        }
        if (codewords[first_seg_start + 6 + i] != v91_ucode_to_codeword(law, 124, true)) {
            fprintf(stderr, "V.91 default DIL first segment positive half mismatch at %d\n", i);
            return false;
        }
        if (codewords[second_seg_start + i] != v91_ucode_to_codeword(law, 0, false)) {
            fprintf(stderr, "V.91 default DIL second segment negative half mismatch at %d\n", i);
            return false;
        }
        if (codewords[second_seg_start + 6 + i] != v91_ucode_to_codeword(law, 0, true)) {
            fprintf(stderr, "V.91 default DIL second segment positive half mismatch at %d\n", i);
            return false;
        }
        if (codewords[last_seg_start + i] != v91_ucode_to_codeword(law, 62, false)) {
            fprintf(stderr, "V.91 default DIL last segment negative half mismatch at %d\n", i);
            return false;
        }
        if (codewords[last_seg_start + 6 + i] != v91_ucode_to_codeword(law, 62, true)) {
            fprintf(stderr, "V.91 default DIL last segment positive half mismatch at %d\n", i);
            return false;
        }
    }

    vpcm_log("PASS: V.91 default DIL over raw-G.711 (%s), segments=%u symbols=%d",
             vpcm_law_to_str(law), dil.n, count);
    return true;
}

static bool test_v91_eu_and_frame_alignment(v91_law_t law)
{
    v91_state_t tx;
    v91_info_frame_t info;
    uint8_t info_buf[V91_INFO_SYMBOLS];
    uint8_t eu[V91_EU_SYMBOLS];
    uint8_t dil[V91_DEFAULT_DIL_SYMBOLS];
    uint8_t expected_eu_symbol;
    int i;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 Eu and frame alignment (%s)", vpcm_law_to_str(law));

    memset(&info, 0, sizeof(info));
    info.request_default_dil = true;
    info.power_measured_after_digital_impairments = true;
    info.tx_uses_alaw = (law == V91_LAW_ALAW);
    info.request_transparent_mode = true;
    info.cleardown_if_transparent_denied = true;

    if (v91_tx_info_codewords(&tx, info_buf, (int) sizeof(info_buf), &info) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "V.91 INFO encode failed before Eu\n");
        return false;
    }
    expected_eu_symbol = v91_ucode_to_codeword(law, 66, tx.diff_sign != 0);
    if (v91_tx_eu_codewords(&tx, eu, (int) sizeof(eu)) != V91_EU_SYMBOLS) {
        fprintf(stderr, "V.91 Eu encode length mismatch\n");
        return false;
    }
    for (i = 0; i < V91_EU_SYMBOLS; i++) {
        if (eu[i] != expected_eu_symbol) {
            fprintf(stderr, "V.91 Eu symbol mismatch at %d: %02X != %02X\n",
                    i, eu[i], expected_eu_symbol);
            return false;
        }
    }
    if (!tx.frame_aligned || tx.retrain_requested || tx.next_frame_interval != 0) {
        fprintf(stderr, "V.91 Eu did not establish frame alignment correctly\n");
        return false;
    }
    if (!tx.circuit_107_on) {
        fprintf(stderr, "V.91 Eu did not raise circuit 107\n");
        return false;
    }

    if (v91_tx_default_dil_codewords(&tx, dil, (int) sizeof(dil)) != V91_DEFAULT_DIL_SYMBOLS) {
        fprintf(stderr, "V.91 default DIL encode failed after Eu\n");
        return false;
    }
    if (!tx.frame_aligned || tx.next_frame_interval != 0) {
        fprintf(stderr, "V.91 frame alignment did not persist across DIL\n");
        return false;
    }

    v91_note_frame_sync_loss(&tx);
    if (tx.frame_aligned || !tx.retrain_requested || tx.next_frame_interval != -1) {
        fprintf(stderr, "V.91 frame sync loss did not request retrain\n");
        return false;
    }

    vpcm_log("PASS: V.91 Eu and frame alignment (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_em_and_startup_sequence(v91_law_t law)
{
    v91_state_t tx;
    v91_info_frame_t local_info;
    v91_info_frame_t peer_info;
    v91_dil_desc_t dil;
    uint8_t em[V91_EM_SYMBOLS];
    uint8_t seq[V91_EM_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    v91_align_signal_t align_signal;
    uint32_t expected_scramble_reg;
    int expected_sign;
    int i;
    int seq_len;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 Em and startup DIL sequence (%s)", vpcm_law_to_str(law));

    memset(&local_info, 0, sizeof(local_info));
    local_info.request_default_dil = false;
    local_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    local_info.power_measured_after_digital_impairments = true;
    memset(&peer_info, 0, sizeof(peer_info));
    peer_info.request_default_dil = false;
    peer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    peer_info.power_measured_after_digital_impairments = true;

    tx.last_tx_info = local_info;
    tx.last_tx_info_valid = true;
    tx.last_rx_info = peer_info;
    tx.last_rx_info_valid = true;
    tx.scramble_reg = 0x0055AA55U;
    tx.diff_sign = 1;

    expected_scramble_reg = tx.scramble_reg;
    expected_sign = tx.diff_sign;
    if (v91_tx_em_codewords(&tx, em, (int) sizeof(em)) != V91_EM_SYMBOLS) {
        fprintf(stderr, "V.91 Em encode length mismatch\n");
        return false;
    }
    for (i = 0; i < V91_EM_SYMBOLS; i++) {
        expected_sign ^= vpcm_v91_gpc_scramble_bit(&expected_scramble_reg, 0);
        if (em[i] != v91_ucode_to_codeword(law, 66, expected_sign != 0)) {
            fprintf(stderr, "V.91 Em symbol mismatch at %d\n", i);
            return false;
        }
    }
    if (tx.scramble_reg != expected_scramble_reg || tx.diff_sign != expected_sign) {
        fprintf(stderr, "V.91 Em did not preserve scrambler/differential state\n");
        return false;
    }
    if (!tx.frame_aligned || !tx.circuit_107_on || tx.next_frame_interval != 0) {
        fprintf(stderr, "V.91 Em did not establish frame alignment correctly\n");
        return false;
    }

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    tx.last_tx_info = local_info;
    tx.last_tx_info_valid = true;
    tx.last_rx_info = peer_info;
    tx.last_rx_info_valid = true;
    tx.scramble_reg = 0x00123456U;
    tx.diff_sign = 1;
    v91_default_dil_init(&dil);
    seq_len = v91_tx_startup_dil_sequence_codewords(&tx,
                                                    seq,
                                                    (int) sizeof(seq),
                                                    &dil,
                                                    &align_signal);
    if (seq_len != V91_EM_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS) {
        fprintf(stderr, "V.91 startup Em+DIL sequence length mismatch: %d\n", seq_len);
        return false;
    }
    if (align_signal != V91_ALIGN_EM) {
        fprintf(stderr, "V.91 startup helper chose Eu instead of Em\n");
        return false;
    }
    if (!tx.frame_aligned || !tx.circuit_107_on || tx.next_frame_interval != 0) {
        fprintf(stderr, "V.91 startup Em+DIL sequence did not keep alignment\n");
        return false;
    }

    vpcm_log("PASS: V.91 Em and startup DIL sequence (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_phil_and_scr_sequences(v91_law_t law)
{
    v91_state_t tx;
    uint8_t phil[V91_INFO_SYMBOLS];
    uint8_t scr[18];
    uint8_t em[V91_EM_SYMBOLS];
    uint32_t expected_scramble_reg;
    int expected_sign;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 PHIL/SCR primitives (%s)", vpcm_law_to_str(law));

    if (v91_tx_phil_codewords(&tx, phil, (int) sizeof(phil), V91_INFO_SYMBOLS, false) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "V.91 PHIL encode length mismatch\n");
        return false;
    }
    if (!vpcm_expect_scrambled_ones_sequence(law,
                                             phil,
                                             V91_INFO_SYMBOLS,
                                             0,
                                             0,
                                             &expected_scramble_reg,
                                             &expected_sign)) {
        fprintf(stderr, "V.91 PHIL sequence mismatch\n");
        return false;
    }
    if (tx.scramble_reg != expected_scramble_reg || tx.diff_sign != expected_sign) {
        fprintf(stderr, "V.91 PHIL coder state mismatch\n");
        return false;
    }
    if (tx.frame_aligned || tx.circuit_107_on) {
        fprintf(stderr, "V.91 PHIL should not establish frame alignment\n");
        return false;
    }

    if (v91_tx_em_codewords(&tx, em, (int) sizeof(em)) != V91_EM_SYMBOLS) {
        fprintf(stderr, "V.91 Em-after-PHIL encode length mismatch\n");
        return false;
    }
    if (!tx.frame_aligned || !tx.circuit_107_on || tx.next_frame_interval != 0) {
        fprintf(stderr, "V.91 Em after PHIL did not establish frame alignment\n");
        return false;
    }

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    if (v91_tx_scr_codewords(&tx, scr, (int) sizeof(scr), 18) != 18) {
        fprintf(stderr, "V.91 SCR encode length mismatch\n");
        return false;
    }
    if (!vpcm_expect_scrambled_ones_sequence(law,
                                             scr,
                                             18,
                                             0,
                                             0,
                                             &expected_scramble_reg,
                                             &expected_sign)) {
        fprintf(stderr, "V.91 SCR sequence mismatch\n");
        return false;
    }
    if (tx.scramble_reg != expected_scramble_reg || tx.diff_sign != expected_sign) {
        fprintf(stderr, "V.91 SCR coder state mismatch\n");
        return false;
    }
    if (v91_tx_scr_codewords(&tx, scr, (int) sizeof(scr), 10) != 0) {
        fprintf(stderr, "V.91 SCR accepted a non-multiple-of-6 length\n");
        return false;
    }

    vpcm_log("PASS: V.91 PHIL/SCR primitives (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_eu_startup_sequence(v91_law_t law)
{
    v91_state_t tx;
    v91_info_frame_t local_info;
    v91_info_frame_t peer_info;
    uint8_t seq[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    v91_align_signal_t align_signal;
    int seq_len;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 Eu startup DIL sequence (%s)", vpcm_law_to_str(law));

    memset(&local_info, 0, sizeof(local_info));
    local_info.request_default_dil = true;
    local_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    local_info.power_measured_after_digital_impairments = true;
    local_info.request_transparent_mode = true;
    local_info.cleardown_if_transparent_denied = true;
    peer_info = local_info;

    tx.last_tx_info = local_info;
    tx.last_tx_info_valid = true;
    tx.last_rx_info = peer_info;
    tx.last_rx_info_valid = true;

    seq_len = v91_tx_startup_dil_sequence_codewords(&tx,
                                                    seq,
                                                    (int) sizeof(seq),
                                                    NULL,
                                                    &align_signal);
    if (seq_len != V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS) {
        fprintf(stderr, "V.91 startup Eu+DIL sequence length mismatch: %d\n", seq_len);
        return false;
    }
    if (align_signal != V91_ALIGN_EU) {
        fprintf(stderr, "V.91 startup helper chose Em instead of Eu\n");
        return false;
    }
    if (!tx.frame_aligned || !tx.circuit_107_on || tx.next_frame_interval != 0) {
        fprintf(stderr, "V.91 startup Eu+DIL sequence did not keep alignment\n");
        return false;
    }

    vpcm_log("PASS: V.91 Eu startup DIL sequence (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_bilateral_info_tracking(v91_law_t law_a, v91_law_t law_b)
{
    v91_state_t side_a;
    v91_state_t side_b;
    v91_info_frame_t info_a;
    v91_info_frame_t info_b;

    v91_init(&side_a, law_a, V91_MODE_TRANSPARENT);
    v91_init(&side_b, law_b, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 bilateral INFO tracking (%s -> %s)",
             vpcm_law_to_str(law_a), vpcm_law_to_str(law_b));

    memset(&info_a, 0, sizeof(info_a));
    info_a.reserved_12_25 = 0x0155;
    info_a.request_default_dil = true;
    info_a.request_control_channel = false;
    info_a.acknowledge_info_frame = false;
    info_a.reserved_29_32 = 0x3;
    info_a.max_tx_power = 7;
    info_a.power_measured_after_digital_impairments = true;
    info_a.tx_uses_alaw = (law_a == V91_LAW_ALAW);
    info_a.request_transparent_mode = true;
    info_a.cleardown_if_transparent_denied = true;

    memset(&info_b, 0, sizeof(info_b));
    info_b.reserved_12_25 = 0x02aa;
    info_b.request_default_dil = false;
    info_b.request_control_channel = true;
    info_b.acknowledge_info_frame = true;
    info_b.reserved_29_32 = 0xc;
    info_b.max_tx_power = 21;
    info_b.power_measured_after_digital_impairments = false;
    info_b.tx_uses_alaw = (law_b == V91_LAW_ALAW);
    info_b.request_transparent_mode = false;
    info_b.cleardown_if_transparent_denied = false;

    if (!run_v91_info_roundtrip_case(&side_a, &side_b, "A -> B INFO", &info_a, false))
        return false;
    if (!run_v91_info_roundtrip_case(&side_b, &side_a, "B -> A INFO", &info_b, true))
        return false;

    if (!side_a.last_tx_info_valid
        || !side_a.last_rx_info_valid
        || !vpcm_info_frames_equal(&info_a, &side_a.last_tx_info)
        || !vpcm_info_frames_equal(&info_b, &side_a.last_rx_info)) {
        fprintf(stderr, "V.91 bilateral tracking mismatch on side A\n");
        return false;
    }
    if (!side_b.last_tx_info_valid
        || !side_b.last_rx_info_valid
        || !vpcm_info_frames_equal(&info_b, &side_b.last_tx_info)
        || !vpcm_info_frames_equal(&info_a, &side_b.last_rx_info)) {
        fprintf(stderr, "V.91 bilateral tracking mismatch on side B\n");
        return false;
    }

    vpcm_log("PASS: V.91 bilateral INFO tracking (%s -> %s)",
             vpcm_law_to_str(law_a), vpcm_law_to_str(law_b));
    return true;
}

static void vpcm_log_cp_compare_row(const char *field, const char *tx, const char *rx)
{
    vpcm_log("| %-10s | %-18s | %-18s |", field, tx, rx);
}

static void vpcm_log_cp_diag_compare(const vpcm_cp_diag_t *tx, const vpcm_cp_diag_t *rx)
{
    char tx_buf[64];
    char rx_buf[64];
    int i;

    vpcm_log("+------------+--------------------+--------------------+");
    vpcm_log("| Field      | TX                 | RX                 |");
    vpcm_log("+------------+--------------------+--------------------+");
    snprintf(tx_buf, sizeof(tx_buf), "%s", tx->frame_sync_ok ? "correct" : "incorrect");
    snprintf(rx_buf, sizeof(rx_buf), "%s", rx->frame_sync_ok ? "correct" : "incorrect");
    vpcm_log_cp_compare_row("FS", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%s", tx->start_bits_ok ? "correct" : "incorrect");
    snprintf(rx_buf, sizeof(rx_buf), "%s", rx->start_bits_ok ? "correct" : "incorrect");
    vpcm_log_cp_compare_row("Starts", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%s", tx->frame.transparent_mode_granted ? "granted" : "not granted");
    snprintf(rx_buf, sizeof(rx_buf), "%s", rx->frame.transparent_mode_granted ? "granted" : "not granted");
    vpcm_log_cp_compare_row("Mode", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%s", tx->frame.v90_compatibility ? "compatible" : "non-v90");
    snprintf(rx_buf, sizeof(rx_buf), "%s", rx->frame.v90_compatibility ? "compatible" : "non-v90");
    vpcm_log_cp_compare_row("Compat", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%u", tx->frame.drn);
    snprintf(rx_buf, sizeof(rx_buf), "%u", rx->frame.drn);
    vpcm_log_cp_compare_row("DRN", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%s", tx->frame.acknowledge ? "yes" : "no");
    snprintf(rx_buf, sizeof(rx_buf), "%s", rx->frame.acknowledge ? "yes" : "no");
    vpcm_log_cp_compare_row("ACK", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%u", tx->frame.constellation_count);
    snprintf(rx_buf, sizeof(rx_buf), "%u", rx->frame.constellation_count);
    vpcm_log_cp_compare_row("Consts", tx_buf, rx_buf);
    for (i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        snprintf(tx_buf, sizeof(tx_buf), "%u", tx->frame.dfi[i]);
        snprintf(rx_buf, sizeof(rx_buf), "%u", rx->frame.dfi[i]);
        if (i == 0)
            vpcm_log_cp_compare_row("DFI0", tx_buf, rx_buf);
        else if (i == 1)
            vpcm_log_cp_compare_row("DFI1", tx_buf, rx_buf);
        else if (i == 2)
            vpcm_log_cp_compare_row("DFI2", tx_buf, rx_buf);
        else if (i == 3)
            vpcm_log_cp_compare_row("DFI3", tx_buf, rx_buf);
        else if (i == 4)
            vpcm_log_cp_compare_row("DFI4", tx_buf, rx_buf);
        else
            vpcm_log_cp_compare_row("DFI5", tx_buf, rx_buf);
    }
    for (i = 0; i < tx->frame.constellation_count && i < VPCM_CP_MAX_CONSTELLATIONS; i++) {
        snprintf(tx_buf, sizeof(tx_buf), "%d set", vpcm_cp_mask_population(tx->frame.masks[i]));
        snprintf(rx_buf, sizeof(rx_buf), "%d set", vpcm_cp_mask_population(rx->frame.masks[i]));
        if (i == 0)
            vpcm_log_cp_compare_row("Mask0", tx_buf, rx_buf);
        else if (i == 1)
            vpcm_log_cp_compare_row("Mask1", tx_buf, rx_buf);
        else if (i == 2)
            vpcm_log_cp_compare_row("Mask2", tx_buf, rx_buf);
        else if (i == 3)
            vpcm_log_cp_compare_row("Mask3", tx_buf, rx_buf);
        else if (i == 4)
            vpcm_log_cp_compare_row("Mask4", tx_buf, rx_buf);
        else
            vpcm_log_cp_compare_row("Mask5", tx_buf, rx_buf);
    }
    snprintf(tx_buf, sizeof(tx_buf), "0x%04x", tx->crc_field);
    snprintf(rx_buf, sizeof(rx_buf), "0x%04x", rx->crc_field);
    vpcm_log_cp_compare_row("CRC", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "0x%04x", tx->crc_remainder);
    snprintf(rx_buf, sizeof(rx_buf), "0x%04x", rx->crc_remainder);
    vpcm_log_cp_compare_row("CRC rem", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%s", tx->valid ? "correct" : "incorrect");
    snprintf(rx_buf, sizeof(rx_buf), "%s", rx->valid ? "correct" : "incorrect");
    vpcm_log_cp_compare_row("CP", tx_buf, rx_buf);
    vpcm_log("+------------+--------------------+--------------------+");
}

static void vpcm_maybe_log_info_session_diag(v91_state_t *tx_state,
                                             const v91_info_frame_t *tx_info,
                                             const uint8_t *rx_codewords,
                                             int rx_len,
                                             const char *label)
{
    v91_info_diag_t tx_diag;
    v91_info_diag_t rx_diag;

    if (!g_vpcm_session_diag)
        return;
    if (!v91_info_build_diag(tx_state, tx_info, &tx_diag))
        return;
    if (!v91_info_decode_diag(tx_state, rx_codewords, rx_len, &rx_diag))
        return;
    vpcm_log("%s INFO", label);
    vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
}

static void vpcm_maybe_log_cp_session_diag(const vpcm_cp_frame_t *tx_cp,
                                           const uint8_t *bits,
                                           int nbits,
                                           const char *label)
{
    vpcm_cp_diag_t tx_diag;
    vpcm_cp_diag_t rx_diag;

    if (!g_vpcm_session_diag)
        return;
    if (!vpcm_cp_build_diag(tx_cp, &tx_diag))
        return;
    if (!vpcm_cp_decode_diag(bits, nbits, &rx_diag))
        return;
    vpcm_log("%s CP", label);
    vpcm_log_cp_diag_compare(&tx_diag, &rx_diag);
}

static void vpcm_log_session_sequence_header(const char *side1_label,
                                             const char *side2_label)
{
    if (!g_vpcm_session_diag)
        return;
    vpcm_log("Sequence: %s <-> %s", side1_label, side2_label);
    vpcm_log("+----------------------+----------------------+----------------------+----------------------+");
    vpcm_log("| %-20s | %-20s | %-20s | %-20s |",
             "Side 1 TX",
             "Side 1 RX",
             "Side 2 RX",
             "Side 2 TX");
    vpcm_log("+----------------------+----------------------+----------------------+----------------------+");
}

static void vpcm_log_session_sequence_row(const char *side1_tx,
                                          const char *side1_rx,
                                          const char *side2_rx,
                                          const char *side2_tx)
{
    if (!g_vpcm_session_diag)
        return;
    vpcm_log("| %-20.20s | %-20.20s | %-20.20s | %-20.20s |",
             side1_tx,
             side1_rx,
             side2_rx,
             side2_tx);
}

static void vpcm_log_session_sequence_footer(void)
{
    if (!g_vpcm_session_diag)
        return;
    vpcm_log("+----------------------+----------------------+----------------------+----------------------+");
}

static void vpcm_maybe_log_startup_frame_diag(const char *label,
                                              v91_state_t *tx_state,
                                              const v91_info_frame_t *tx_info,
                                              const uint8_t *rx_codewords,
                                              int rx_len)
{
    vpcm_maybe_log_info_session_diag(tx_state, tx_info, rx_codewords, rx_len, label);
}

static void vpcm_maybe_log_rate_request_diag(const char *label,
                                             const vpcm_cp_frame_t *tx_cp,
                                             const uint8_t *bits,
                                             int nbits)
{
    vpcm_maybe_log_cp_session_diag(tx_cp, bits, nbits, label);
}

static void vpcm_format_rate(char *buf, size_t len, uint8_t drn)
{
    double bps;

    bps = vpcm_cp_drn_to_bps(drn);
    if (drn == 0)
        snprintf(buf, len, "cleardown");
    else if (((int) (bps + 0.5)) == (int) bps)
        snprintf(buf, len, "%.0f bps", bps);
    else
        snprintf(buf, len, "%.2f bps", bps);
}

static void vpcm_v92_init_digital_dil_from_ja(v91_dil_desc_t *desc, bool echo_limited)
{
    int i;
    static const uint8_t echo_train_u[8] = {60, 61, 62, 63, 64, 65, 66, 67};

    v91_default_dil_init(desc);
    if (!echo_limited)
        return;

    /*
     * Harness approximation of a Ja-derived echo-limited profile:
     * keep the default family framing, but constrain the DIL so the
     * analogue side sees shorter/narrower training and requests a safer
     * downstream/upstream pair.
     */
    desc->n = 96;
    desc->lsp = 6;
    desc->ltp = 6;
    for (i = 0; i < 8; i++) {
        desc->ref[i] = 1;
        desc->h[i] = 1;
    }
    for (i = 0; i < desc->n; i++)
        desc->train_u[i] = echo_train_u[i % 8];
}

static void vpcm_init_robbed_bit_dil_profile(v91_dil_desc_t *desc)
{
    int i;

    v91_default_dil_init(desc);
    desc->ltp = 6;
    for (i = 0; i < desc->ltp; i++)
        desc->tp[i] = 1;
}

static void vpcm_v92_select_profile_from_dil(const v91_dil_analysis_t *analysis,
                                             uint8_t *downstream_drn,
                                             uint8_t *upstream_drn)
{
    if (analysis && analysis->recommended_downstream_drn != 0 && analysis->recommended_upstream_drn != 0) {
        *downstream_drn = analysis->recommended_downstream_drn;
        *upstream_drn = analysis->recommended_upstream_drn;
    } else {
        *downstream_drn = 19;
        *upstream_drn = 16;
    }
}

static void vpcm_log_dil_analysis(const char *label, const v91_dil_analysis_t *analysis)
{
    char down_rate_buf[64];
    char up_rate_buf[64];

    if (!analysis)
        return;

    vpcm_format_rate(down_rate_buf, sizeof(down_rate_buf), analysis->recommended_downstream_drn);
    vpcm_format_rate(up_rate_buf, sizeof(up_rate_buf), analysis->recommended_upstream_drn);
    vpcm_log("%s DIL analysis: N=%u LSP=%u LTP=%u unique=%u refs=%u score=%u requested downstream=%s upstream=%s%s",
             label,
             analysis->n,
             analysis->lsp,
             analysis->ltp,
             analysis->unique_train_u,
             analysis->non_default_refs,
             analysis->impairment_score,
             down_rate_buf,
             up_rate_buf,
             analysis->robbed_bit_limited ? ", robbed-bit-limited"
             : (analysis->echo_limited ? ", echo-limited" : ""));
}

static bool test_v91_cp_exchange(v91_law_t law)
{
    v91_state_t tx;
    v91_state_t rx;
    vpcm_cp_frame_t cp_tx;
    vpcm_cp_frame_t cp_rx;
    vpcm_cp_diag_t tx_diag;
    vpcm_cp_diag_t rx_diag;
    uint8_t codewords[VPCM_CP_MAX_BITS];
    uint8_t bits[VPCM_CP_MAX_BITS];
    int nbits;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_init(&rx, law, V91_MODE_TRANSPARENT);
    vpcm_cp_init(&cp_tx);
    cp_tx.transparent_mode_granted = true;
    cp_tx.v90_compatibility = true;
    cp_tx.drn = 9;
    cp_tx.acknowledge = true;
    cp_tx.constellation_count = 3;
    cp_tx.dfi[0] = 0;
    cp_tx.dfi[1] = 1;
    cp_tx.dfi[2] = 2;
    cp_tx.dfi[3] = 0;
    cp_tx.dfi[4] = 1;
    cp_tx.dfi[5] = 2;
    vpcm_cp_mask_set(cp_tx.masks[0], 0, true);
    vpcm_cp_mask_set(cp_tx.masks[0], 1, true);
    vpcm_cp_mask_set(cp_tx.masks[0], 15, true);
    vpcm_cp_mask_set(cp_tx.masks[1], 16, true);
    vpcm_cp_mask_set(cp_tx.masks[1], 31, true);
    vpcm_cp_mask_set(cp_tx.masks[1], 64, true);
    vpcm_cp_mask_set(cp_tx.masks[2], 32, true);
    vpcm_cp_mask_set(cp_tx.masks[2], 48, true);
    vpcm_cp_mask_set(cp_tx.masks[2], 127, true);

    vpcm_log("Test: V.91 CP exchange over raw-G.711 (%s)", vpcm_law_to_str(law));
    if (!vpcm_cp_build_diag(&cp_tx, &tx_diag)) {
        fprintf(stderr, "V.91 CP TX diag build failed\n");
        return false;
    }
    nbits = vpcm_cp_bit_length(&cp_tx);
    if (v91_tx_cp_codewords(&tx, codewords, (int) sizeof(codewords), &cp_tx, false) != nbits) {
        fprintf(stderr, "V.91 CP encode length mismatch\n");
        return false;
    }
    if (!tx.last_tx_cp_valid || !vpcm_cp_frames_equal(&cp_tx, &tx.last_tx_cp)) {
        fprintf(stderr, "V.91 CP TX tracking mismatch\n");
        return false;
    }
    if (!v91_rx_cp_codewords(&rx, codewords, nbits, &cp_rx, false)) {
        fprintf(stderr, "V.91 CP RX decode failed\n");
        return false;
    }
    if (!vpcm_v91_decode_scrambled_codewords_to_bits(codewords, nbits, bits)
        || !vpcm_cp_decode_diag(bits, nbits, &rx_diag)) {
        fprintf(stderr, "V.91 CP RX diag decode failed\n");
        return false;
    }
    vpcm_log_cp_diag_compare(&tx_diag, &rx_diag);
    if (!rx.last_rx_cp_valid || !vpcm_cp_frames_equal(&cp_tx, &rx.last_rx_cp) || !vpcm_cp_frames_equal(&cp_tx, &cp_rx)) {
        fprintf(stderr, "V.91 CP RX tracking mismatch\n");
        return false;
    }

    vpcm_log("PASS: V.91 CP exchange over raw-G.711 (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_startup_to_b1_multirate(v91_law_t law)
{
    static const uint8_t drn_cases[] = {1, 4, 9, 16, 24, 28};
    v91_state_t caller;
    v91_state_t answerer;
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    v91_info_frame_t rx_info;
    v91_dil_desc_t default_dil;
    uint8_t info_buf[V91_INFO_SYMBOLS];
    uint8_t startup_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t ez_buf[V91_EZ_SYMBOLS];
    uint8_t scr_buf[18];
    uint8_t cp_buf[VPCM_CP_MAX_BITS];
    uint8_t es_buf[V91_ES_SYMBOLS];
    uint8_t b1_buf[V91_B1_SYMBOLS];
    uint8_t first_b1[V91_B1_SYMBOLS];
    vpcm_cp_frame_t cp_offer;
    vpcm_cp_frame_t cp_ack;
    vpcm_cp_frame_t cp_rx;
    char rate_buf[64];
    bool first_b1_valid;
    bool saw_rate_sensitive_b1;
    int i;
    int startup_len;
    int cp_len;

    vpcm_log("Test: V.91 startup to Es/B1 multirate (%s)", vpcm_law_to_str(law));
    v91_default_dil_init(&default_dil);
    first_b1_valid = false;
    saw_rate_sensitive_b1 = false;

    for (i = 0; i < (int) (sizeof(drn_cases)/sizeof(drn_cases[0])); i++) {
        uint8_t drn;

        drn = drn_cases[i];
        v91_init(&caller, law, V91_MODE_TRANSPARENT);
        v91_init(&answerer, law, V91_MODE_TRANSPARENT);

        memset(&caller_info, 0, sizeof(caller_info));
        caller_info.request_default_dil = true;
        caller_info.tx_uses_alaw = (law == V91_LAW_ALAW);
        caller_info.power_measured_after_digital_impairments = true;
        memset(&answerer_info, 0, sizeof(answerer_info));
        answerer_info.request_default_dil = true;
        answerer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
        answerer_info.power_measured_after_digital_impairments = true;

        if (v91_tx_phase1_silence_codewords(&caller, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS
            || v91_tx_phase1_silence_codewords(&answerer, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS) {
            fprintf(stderr, "V.91 phase1 silence failed\n");
            return false;
        }
        if (v91_tx_ez_codewords(&caller, ez_buf, (int) sizeof(ez_buf)) != V91_EZ_SYMBOLS
            || v91_tx_ez_codewords(&answerer, ez_buf, (int) sizeof(ez_buf)) != V91_EZ_SYMBOLS) {
            fprintf(stderr, "V.91 Ez failed\n");
            return false;
        }

        if (v91_tx_info_codewords(&caller, info_buf, (int) sizeof(info_buf), &caller_info) != V91_INFO_SYMBOLS
            || !v91_rx_info_codewords(&answerer, info_buf, V91_INFO_SYMBOLS, &rx_info)) {
            fprintf(stderr, "V.91 INFO caller->answerer failed\n");
            return false;
        }
        if (v91_tx_info_codewords(&answerer, info_buf, (int) sizeof(info_buf), &answerer_info) != V91_INFO_SYMBOLS
            || !v91_rx_info_codewords(&caller, info_buf, V91_INFO_SYMBOLS, &rx_info)) {
            fprintf(stderr, "V.91 INFO answerer->caller failed\n");
            return false;
        }

        startup_len = v91_tx_startup_dil_sequence_codewords(&caller,
                                                            startup_buf,
                                                            (int) sizeof(startup_buf),
                                                            &default_dil,
                                                            NULL);
        if (startup_len <= 0) {
            fprintf(stderr, "V.91 caller startup DIL sequence failed\n");
            return false;
        }
        startup_len = v91_tx_startup_dil_sequence_codewords(&answerer,
                                                            startup_buf,
                                                            (int) sizeof(startup_buf),
                                                            &default_dil,
                                                            NULL);
        if (startup_len <= 0) {
            fprintf(stderr, "V.91 answerer startup DIL sequence failed\n");
            return false;
        }

        if (v91_tx_scr_codewords(&caller, scr_buf, (int) sizeof(scr_buf), 18) != 18
            || !v91_rx_scr_codewords(&answerer, scr_buf, 18, false)) {
            fprintf(stderr, "V.91 caller SCR failed\n");
            return false;
        }
        if (v91_tx_scr_codewords(&answerer, scr_buf, (int) sizeof(scr_buf), 18) != 18
            || !v91_rx_scr_codewords(&caller, scr_buf, 18, false)) {
            fprintf(stderr, "V.91 answerer SCR failed\n");
            return false;
        }

        vpcm_cp_init(&cp_offer);
        cp_offer.transparent_mode_granted = false;
        cp_offer.v90_compatibility = true;
        cp_offer.drn = drn;
        cp_offer.acknowledge = false;
        cp_offer.constellation_count = 2;
        cp_offer.dfi[0] = 0;
        cp_offer.dfi[1] = 1;
        cp_offer.dfi[2] = 0;
        cp_offer.dfi[3] = 1;
        cp_offer.dfi[4] = 0;
        cp_offer.dfi[5] = 1;
        vpcm_cp_mask_set(cp_offer.masks[0], 0, true);
        vpcm_cp_mask_set(cp_offer.masks[0], 1, true);
        vpcm_cp_mask_set(cp_offer.masks[0], 15, true);
        vpcm_cp_mask_set(cp_offer.masks[1], 16, true);
        vpcm_cp_mask_set(cp_offer.masks[1], 31, true);
        vpcm_cp_mask_set(cp_offer.masks[1], 63, true);

        cp_len = v91_tx_cp_codewords(&caller, cp_buf, (int) sizeof(cp_buf), &cp_offer, true);
        if (cp_len <= 0 || !v91_rx_cp_codewords(&answerer, cp_buf, cp_len, &cp_rx, true) || !vpcm_cp_frames_equal(&cp_offer, &cp_rx)) {
            fprintf(stderr, "V.91 CP offer failed for drn=%u\n", drn);
            return false;
        }

        cp_ack = cp_offer;
        cp_ack.acknowledge = true;
        cp_len = v91_tx_cp_codewords(&answerer, cp_buf, (int) sizeof(cp_buf), &cp_ack, true);
        if (cp_len <= 0 || !v91_rx_cp_codewords(&caller, cp_buf, cp_len, &cp_rx, true) || !vpcm_cp_frames_equal(&cp_ack, &cp_rx)) {
            fprintf(stderr, "V.91 CP acknowledge failed for drn=%u\n", drn);
            return false;
        }

        if (v91_tx_es_codewords(&caller, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
            || !v91_rx_es_codewords(&answerer, es_buf, V91_ES_SYMBOLS, true)) {
            fprintf(stderr, "V.91 Es caller->answerer failed for drn=%u\n", drn);
            return false;
        }
        if (v91_tx_b1_codewords(&caller, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS
            || !v91_rx_b1_codewords(&answerer, b1_buf, V91_B1_SYMBOLS, &cp_ack)) {
            fprintf(stderr, "V.91 B1 caller->answerer failed for drn=%u\n", drn);
            return false;
        }

        if (!first_b1_valid) {
            memcpy(first_b1, b1_buf, sizeof(first_b1));
            first_b1_valid = true;
        } else if (memcmp(first_b1, b1_buf, sizeof(first_b1)) != 0) {
            saw_rate_sensitive_b1 = true;
        }

        if (v91_tx_es_codewords(&answerer, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
            || !v91_rx_es_codewords(&caller, es_buf, V91_ES_SYMBOLS, true)) {
            fprintf(stderr, "V.91 Es answerer->caller failed for drn=%u\n", drn);
            return false;
        }
        if (v91_tx_b1_codewords(&answerer, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS
            || !v91_rx_b1_codewords(&caller, b1_buf, V91_B1_SYMBOLS, &cp_ack)) {
            fprintf(stderr, "V.91 B1 answerer->caller failed for drn=%u\n", drn);
            return false;
        }

        vpcm_format_rate(rate_buf, sizeof(rate_buf), drn);
        vpcm_log("PASS: startup -> CP/CP' -> Es -> B1 (%s, drn=%u, rate=%s)",
                 vpcm_law_to_str(law), drn, rate_buf);
    }

    if (!saw_rate_sensitive_b1) {
        fprintf(stderr, "V.91 multirate B1 patterns did not vary across DRN cases\n");
        return false;
    }

    vpcm_log("PASS: V.91 startup to Es/B1 multirate (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_mapped_data_multirate(v91_law_t law)
{
    static const struct {
        uint8_t drn;
        bool transparent;
        const char *mode_label;
    } cases[] = {
        {1,  false, "non-transparent"},
        {4,  false, "non-transparent"},
        {9,  false, "non-transparent"},
        {16, false, "non-transparent"},
        {24, false, "non-transparent"},
        {28, false, "non-transparent"},
        {28, true,  "transparent"},
    };
    enum { NOMINAL_10S_FRAMES = 13328 };
    v91_state_t caller_tx;
    v91_state_t answerer_rx;
    vpcm_cp_frame_t cp;
    char rate_buf[64];
    int i;

    vpcm_log("Test: V.91 mapped data multirate (%s)", vpcm_law_to_str(law));
    for (i = 0; i < (int) (sizeof(cases)/sizeof(cases[0])); i++) {
        uint8_t *data_in;
        uint8_t *data_out;
        uint8_t *pcm_tx;
        uint8_t drn;
        bool transparent;
        const char *mode_label;
        int total_bits;
        int total_bytes;
        int total_codewords;
        int produced;
        int consumed;
        int sample_data_len;
        int sample_pcm_len;

        drn = cases[i].drn;
        transparent = cases[i].transparent;
        mode_label = cases[i].mode_label;
        total_bits = NOMINAL_10S_FRAMES * ((int) drn + 20);
        total_bytes = total_bits / 8;
        total_codewords = NOMINAL_10S_FRAMES * VPCM_CP_FRAME_INTERVALS;

        data_in = (uint8_t *) malloc((size_t) total_bytes);
        data_out = (uint8_t *) malloc((size_t) total_bytes);
        pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
        if (!data_in || !data_out || !pcm_tx) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            fprintf(stderr, "allocation failure in mapped data test\n");
            return false;
        }

        v91_init(&caller_tx, law, V91_MODE_TRANSPARENT);
        v91_init(&answerer_rx, law, V91_MODE_TRANSPARENT);
        vpcm_cp_init(&cp);
        cp.transparent_mode_granted = transparent;
        cp.v90_compatibility = true;
        cp.drn = drn;
        cp.acknowledge = true;
        cp.constellation_count = 1;
        memset(cp.dfi, 0, sizeof(cp.dfi));
        vpcm_cp_enable_all_ucodes(cp.masks[0]);

        if (!v91_activate_data_mode(&caller_tx, &cp) || !v91_activate_data_mode(&answerer_rx, &cp)) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            fprintf(stderr, "failed to activate mapped data mode for drn=%u\n", drn);
            return false;
        }

        fill_pattern(data_in, total_bytes, 0x91000000U ^ ((uint32_t) law << 8) ^ (uint32_t) drn);
        produced = v91_tx_codewords(&caller_tx, pcm_tx, total_codewords, data_in, total_bytes);
        consumed = v91_rx_codewords(&answerer_rx, data_out, total_bytes, pcm_tx, produced);
        if (produced != total_codewords || consumed != total_bytes) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            fprintf(stderr,
                    "mapped data length mismatch for drn=%u: pcm=%d/%d data=%d/%d\n",
                    drn, produced, total_codewords, consumed, total_bytes);
            return false;
        }
        if (!expect_equal("V.91 mapped data", data_in, data_out, total_bytes)) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            return false;
        }

        sample_pcm_len = (produced < 6) ? produced : 6;
        if (cp.transparent_mode_granted) {
            sample_data_len = sample_pcm_len;
        } else {
            sample_data_len = (sample_pcm_len * ((int) drn + 20)) / 48;
        }
        if (sample_data_len < 1)
            sample_data_len = 1;
        if (sample_data_len > total_bytes)
            sample_data_len = total_bytes;
        vpcm_log_data_sample(data_in,
                             sample_data_len,
                             pcm_tx,
                             sample_pcm_len,
                             pcm_tx,
                             sample_pcm_len,
                             data_out,
                             (consumed < sample_data_len) ? consumed : sample_data_len);
        vpcm_format_rate(rate_buf, sizeof(rate_buf), drn);
        vpcm_log("PASS: mapped data (%s, drn=%u, mode=%s, rate=%s, bytes=%d, codewords=%d)",
                 vpcm_law_to_str(law), drn, mode_label, rate_buf, total_bytes, produced);

        free(data_in);
        free(data_out);
        free(pcm_tx);
    }

    vpcm_log("PASS: V.91 mapped data multirate (%s)", vpcm_law_to_str(law));
    return true;
}

static int first_mismatch_index(const uint8_t *expected, const uint8_t *actual, int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (expected[i] != actual[i])
            return i;
    }
    return -1;
}

static bool test_v91_robbed_bit_signalling(v91_law_t law)
{
    static const struct {
        bool transparent;
        const char *mode_label;
    } cases[] = {
        {false, "non-transparent"},
        {true,  "transparent"},
    };
    enum { NOMINAL_10S_FRAMES = 13328 };
    vpcm_cp_frame_t cp;
    int i;

    vpcm_log("Test: V.91 robbed-bit signalling degradation (%s)", vpcm_law_to_str(law));
    for (i = 0; i < (int) (sizeof(cases)/sizeof(cases[0])); i++) {
        v91_state_t caller_tx;
        v91_state_t answerer_rx;
        uint8_t *data_in;
        uint8_t *data_out;
        uint8_t *pcm_tx;
        uint8_t *pcm_rx;
        int total_bits;
        int total_bytes;
        int total_codewords;
        int produced;
        int consumed;
        int mismatch_at;
        int sample_data_len;
        double effective_bps;
        int j;

        total_bits = NOMINAL_10S_FRAMES * (28 + 20);
        total_bytes = total_bits / 8;
        total_codewords = NOMINAL_10S_FRAMES * VPCM_CP_FRAME_INTERVALS;
        effective_bps = 8000.0 * (8.0 - (1.0 / 6.0));

        data_in = (uint8_t *) malloc((size_t) total_bytes);
        data_out = (uint8_t *) malloc((size_t) total_bytes);
        pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
        pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
        if (!data_in || !data_out || !pcm_tx || !pcm_rx) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            free(pcm_rx);
            fprintf(stderr, "allocation failure in robbed-bit signalling test\n");
            return false;
        }

        v91_init(&caller_tx, law, V91_MODE_TRANSPARENT);
        v91_init(&answerer_rx, law, V91_MODE_TRANSPARENT);
        vpcm_cp_init(&cp);
        cp.transparent_mode_granted = cases[i].transparent;
        cp.v90_compatibility = true;
        cp.drn = 28;
        cp.acknowledge = true;
        cp.constellation_count = 1;
        memset(cp.dfi, 0, sizeof(cp.dfi));
        vpcm_cp_enable_all_ucodes(cp.masks[0]);

        if (!v91_activate_data_mode(&caller_tx, &cp) || !v91_activate_data_mode(&answerer_rx, &cp)) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            free(pcm_rx);
            fprintf(stderr, "failed to activate robbed-bit signalling test mode\n");
            return false;
        }

        fill_pattern(data_in, total_bytes, 0x91B00000U ^ ((uint32_t) law << 8) ^ (uint32_t) i);
        memset(data_out, 0, (size_t) total_bytes);

        produced = v91_tx_codewords(&caller_tx, pcm_tx, total_codewords, data_in, total_bytes);
        if (produced != total_codewords) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            free(pcm_rx);
            fprintf(stderr, "robbed-bit test failed to produce full PCM stream\n");
            return false;
        }

        memcpy(pcm_rx, pcm_tx, (size_t) produced);
        for (j = 5; j < produced; j += 6)
            pcm_rx[j] &= 0xFE;

        consumed = v91_rx_codewords(&answerer_rx, data_out, total_bytes, pcm_rx, produced);
        mismatch_at = (consumed < total_bytes) ? consumed : first_mismatch_index(data_in, data_out, total_bytes);
        if (mismatch_at < 0) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            free(pcm_rx);
            fprintf(stderr,
                    "robbed-bit signalling unexpectedly preserved drn=28 %s payload\n",
                    cases[i].mode_label);
            return false;
        }

        sample_data_len = 6;
        if (sample_data_len > total_bytes)
            sample_data_len = total_bytes;
        vpcm_log_data_sample(data_in,
                             sample_data_len,
                             pcm_tx,
                             6,
                             pcm_rx,
                             6,
                             data_out,
                             (consumed < sample_data_len) ? consumed : sample_data_len);
        vpcm_log("PASS: robbed-bit signalling degrades %s drn=28 as expected (%s, first mismatch at byte %d, effective ceiling %.2f bps)",
                 cases[i].mode_label,
                 vpcm_law_to_str(law),
                 mismatch_at,
                 effective_bps);

        free(data_in);
        free(data_out);
        free(pcm_tx);
        free(pcm_rx);
    }

    vpcm_log("PASS: V.91 robbed-bit signalling degradation (%s)", vpcm_law_to_str(law));
    return true;
}

static void vpcm_transport_pcm_family_codewords(bool robbed_bit,
                                                uint8_t *dst,
                                                const uint8_t *src,
                                                int len)
{
    if (robbed_bit)
        vpcm_transport_robbed_bit_codewords(dst, src, len);
    else {
        memcpy(dst, src, (size_t) len);
        vpcm_maybe_realtime_pace_samples(len);
    }
}

static bool __attribute__((unused)) run_vpcm_full_phase_session(v91_law_t law,
                                                                const char *family_label,
                                                                const char *path_label,
                                                                const vpcm_cp_frame_t *cp_offer_template,
                                                                bool robbed_bit,
                                                                uint32_t data_seed,
                                                                int data_seconds)
{
    enum { NOMINAL_10S_FRAMES = 13328 };
    v91_dil_desc_t default_dil;
    v91_state_t caller;
    v91_state_t answerer;
    v91_state_t caller_tx;
    v91_state_t caller_rx;
    v91_state_t answerer_tx;
    v91_state_t answerer_rx;
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    v91_info_frame_t rx_info;
    vpcm_cp_frame_t cp_offer;
    vpcm_cp_frame_t cp_ack;
    vpcm_cp_frame_t cp_rx;
    uint8_t transport_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t startup_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t scr_buf[18];
    uint8_t cp_buf[VPCM_CP_MAX_BITS];
    uint8_t es_buf[V91_ES_SYMBOLS];
    uint8_t b1_buf[V91_B1_SYMBOLS];
    uint8_t *caller_data_in = NULL;
    uint8_t *caller_data_out = NULL;
    uint8_t *answerer_data_in = NULL;
    uint8_t *answerer_data_out = NULL;
    uint8_t *caller_pcm_tx = NULL;
    uint8_t *caller_pcm_rx = NULL;
    uint8_t *answerer_pcm_tx = NULL;
    uint8_t *answerer_pcm_rx = NULL;
    char rate_buf[64];
    int startup_len;
    int caller_startup_len;
    int answerer_startup_len;
    int cp_len;
    int total_bits;
    int total_bytes;
    int total_codewords;
    int caller_produced;
    int answerer_produced;
    int caller_consumed;
    int answerer_consumed;
    int sample_data_len;
    int sample_pcm_len;
    int codewords_per_report;
    int codeword_offset;
    int data_frames;
    uint64_t c2a_bits_checked;
    uint64_t a2c_bits_checked;
    uint64_t c2a_bit_errors;
    uint64_t a2c_bit_errors;
    int c2a_mismatch_chunks;
    int a2c_mismatch_chunks;
    int c2a_chunks_checked;
    int a2c_chunks_checked;

    v91_default_dil_init(&default_dil);
    v91_init(&caller, law, V91_MODE_TRANSPARENT);
    v91_init(&answerer, law, V91_MODE_TRANSPARENT);
    cp_offer = *cp_offer_template;
    c2a_bits_checked = 0;
    a2c_bits_checked = 0;
    c2a_bit_errors = 0;
    a2c_bit_errors = 0;
    c2a_mismatch_chunks = 0;
    a2c_mismatch_chunks = 0;
    c2a_chunks_checked = 0;
    a2c_chunks_checked = 0;

    vpcm_log_e2e_phase("MODEL",
                       "V.91 startup/data path begins after successful V.8 capability signalling");

    if (g_vpcm_session_diag) {
        vpcm_log("Phase model: Phase 1 = V.8/V.8bis, Phase 2 ~= INFO0 -> A/B -> L1/L2 -> INFO1");
        vpcm_log("Phase model note: current harness still approximates Phase 2 transport with shared PCM startup primitives.");
    }

    memset(&caller_info, 0, sizeof(caller_info));
    caller_info.request_default_dil = true;
    caller_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    caller_info.power_measured_after_digital_impairments = true;
    caller_info.request_transparent_mode = cp_offer.transparent_mode_granted;
    caller_info.cleardown_if_transparent_denied = cp_offer.transparent_mode_granted;

    memset(&answerer_info, 0, sizeof(answerer_info));
    answerer_info.request_default_dil = true;
    answerer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    answerer_info.power_measured_after_digital_impairments = true;
    answerer_info.request_transparent_mode = cp_offer.transparent_mode_granted;
    answerer_info.cleardown_if_transparent_denied = cp_offer.transparent_mode_granted;

    if (v91_tx_phase1_silence_codewords(&caller, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS
        || v91_tx_phase1_silence_codewords(&answerer, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS) {
        fprintf(stderr, "%s phase1 silence failed\n", family_label);
        return false;
    }
    vpcm_log_e2e_phase("PHASE1", "Silence complete (%d symbols)", V91_PHASE1_SILENCE_SYMBOLS);
    if (v91_tx_ez_codewords(&caller, transport_buf, (int) sizeof(transport_buf)) != V91_EZ_SYMBOLS
        || v91_tx_ez_codewords(&answerer, transport_buf, (int) sizeof(transport_buf)) != V91_EZ_SYMBOLS) {
        fprintf(stderr, "%s Ez failed\n", family_label);
        return false;
    }
    vpcm_log_e2e_phase("PHASE1", "Ez exchanged (%d symbols)", V91_EZ_SYMBOLS);

    if (v91_tx_info_codewords(&caller, transport_buf, (int) sizeof(transport_buf), &caller_info) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "%s caller INFO tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, transport_buf, V91_INFO_SYMBOLS);
    if (!v91_rx_info_codewords(&answerer, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
        fprintf(stderr, "%s caller->answerer INFO rx failed\n", family_label);
        return false;
    }

    if (v91_tx_info_codewords(&answerer, transport_buf, (int) sizeof(transport_buf), &answerer_info) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "%s answerer INFO tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, transport_buf, V91_INFO_SYMBOLS);
    if (!v91_rx_info_codewords(&caller, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
        fprintf(stderr, "%s answerer->caller INFO rx failed\n", family_label);
        return false;
    }
    vpcm_log_e2e_phase("INFO",
                       "INFO/INFO' exchanged (caller alaw=%d transparent=%d, answerer alaw=%d transparent=%d)",
                       caller_info.tx_uses_alaw ? 1 : 0,
                       caller_info.request_transparent_mode ? 1 : 0,
                       answerer_info.tx_uses_alaw ? 1 : 0,
                       answerer_info.request_transparent_mode ? 1 : 0);

    startup_len = v91_tx_startup_dil_sequence_codewords(&caller,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &default_dil,
                                                        NULL);
    if (startup_len <= 0) {
        fprintf(stderr, "%s caller startup DIL sequence failed\n", family_label);
        return false;
    }
    caller_startup_len = startup_len;
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, startup_buf, startup_len);

    startup_len = v91_tx_startup_dil_sequence_codewords(&answerer,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &default_dil,
                                                        NULL);
    if (startup_len <= 0) {
        fprintf(stderr, "%s answerer startup DIL sequence failed\n", family_label);
        return false;
    }
    answerer_startup_len = startup_len;
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, startup_buf, startup_len);
    vpcm_log_e2e_phase("DIL", "Startup DIL sequence exchanged (caller=%d symbols, answerer=%d symbols)",
                       caller_startup_len, answerer_startup_len);

    if (v91_tx_scr_codewords(&caller, scr_buf, (int) sizeof(scr_buf), 18) != 18) {
        fprintf(stderr, "%s caller SCR tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, scr_buf, 18);
    if (!v91_rx_scr_codewords(&answerer, transport_buf, 18, false)) {
        fprintf(stderr, "%s caller SCR rx failed\n", family_label);
        return false;
    }

    if (v91_tx_scr_codewords(&answerer, scr_buf, (int) sizeof(scr_buf), 18) != 18) {
        fprintf(stderr, "%s answerer SCR tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, scr_buf, 18);
    if (!v91_rx_scr_codewords(&caller, transport_buf, 18, false)) {
        fprintf(stderr, "%s answerer SCR rx failed\n", family_label);
        return false;
    }
    vpcm_log_e2e_phase("SCR", "SCR/SCR' exchanged");

    cp_len = v91_tx_cp_codewords(&caller, cp_buf, (int) sizeof(cp_buf), &cp_offer, true);
    if (cp_len <= 0) {
        fprintf(stderr, "%s caller CP tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, cp_buf, cp_len);
    if (!v91_rx_cp_codewords(&answerer, transport_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&cp_offer, &cp_rx)) {
        fprintf(stderr, "%s caller CP rx failed\n", family_label);
        return false;
    }

    cp_ack = cp_offer;
    cp_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&answerer, cp_buf, (int) sizeof(cp_buf), &cp_ack, true);
    if (cp_len <= 0) {
        fprintf(stderr, "%s answerer CP tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, cp_buf, cp_len);
    if (!v91_rx_cp_codewords(&caller, transport_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&cp_ack, &cp_rx)) {
        fprintf(stderr, "%s answerer CP rx failed\n", family_label);
        return false;
    }
    vpcm_log_e2e_phase("CP",
                       "CP/CP' complete (transparent=%d drn=%u rate=%.0f bps)",
                       cp_ack.transparent_mode_granted ? 1 : 0,
                       cp_ack.drn,
                       vpcm_cp_drn_to_bps(cp_ack.drn));

    if (v91_tx_es_codewords(&caller, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS) {
        fprintf(stderr, "%s caller Es tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, es_buf, V91_ES_SYMBOLS);
    if (!v91_rx_es_codewords(&answerer, transport_buf, V91_ES_SYMBOLS, true)) {
        fprintf(stderr, "%s caller Es rx failed\n", family_label);
        return false;
    }

    if (v91_tx_b1_codewords(&caller, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS) {
        fprintf(stderr, "%s caller B1 tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, b1_buf, V91_B1_SYMBOLS);
    if (!v91_rx_b1_codewords(&answerer, transport_buf, V91_B1_SYMBOLS, &cp_ack)) {
        fprintf(stderr, "%s caller B1 rx failed\n", family_label);
        return false;
    }

    if (v91_tx_es_codewords(&answerer, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS) {
        fprintf(stderr, "%s answerer Es tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, es_buf, V91_ES_SYMBOLS);
    if (!v91_rx_es_codewords(&caller, transport_buf, V91_ES_SYMBOLS, true)) {
        fprintf(stderr, "%s answerer Es rx failed\n", family_label);
        return false;
    }

    if (v91_tx_b1_codewords(&answerer, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS) {
        fprintf(stderr, "%s answerer B1 tx failed\n", family_label);
        return false;
    }
    vpcm_transport_pcm_family_codewords(robbed_bit, transport_buf, b1_buf, V91_B1_SYMBOLS);
    if (!v91_rx_b1_codewords(&caller, transport_buf, V91_B1_SYMBOLS, &cp_ack)) {
        fprintf(stderr, "%s answerer B1 rx failed\n", family_label);
        return false;
    }
    vpcm_log_e2e_phase("B1", "Es/B1 exchanged");

    if (!v91_activate_data_mode(&caller, &cp_ack) || !v91_activate_data_mode(&answerer, &cp_ack)) {
        fprintf(stderr, "%s data-mode activation failed\n", family_label);
        return false;
    }
    caller_tx = caller;
    caller_rx = caller;
    answerer_tx = answerer;
    answerer_rx = answerer;
    vpcm_log_e2e_phase("DATA", "Data mode active");

    data_frames = vpcm_data_frames_from_seconds(data_seconds);
    if (data_frames <= 0)
        data_frames = NOMINAL_10S_FRAMES;

    total_bits = data_frames * ((int) cp_ack.drn + 20);
    total_bytes = total_bits / 8;
    total_codewords = data_frames * VPCM_CP_FRAME_INTERVALS;
    caller_data_in = (uint8_t *) malloc((size_t) total_bytes);
    caller_data_out = (uint8_t *) malloc((size_t) total_bytes);
    answerer_data_in = (uint8_t *) malloc((size_t) total_bytes);
    answerer_data_out = (uint8_t *) malloc((size_t) total_bytes);
    caller_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    caller_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    answerer_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    answerer_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    if (!caller_data_in || !caller_data_out || !answerer_data_in || !answerer_data_out
        || !caller_pcm_tx || !caller_pcm_rx || !answerer_pcm_tx || !answerer_pcm_rx) {
        free(caller_data_in);
        free(caller_data_out);
        free(answerer_data_in);
        free(answerer_data_out);
        free(caller_pcm_tx);
        free(caller_pcm_rx);
        free(answerer_pcm_tx);
        free(answerer_pcm_rx);
        fprintf(stderr, "allocation failure in %s session\n", family_label);
        return false;
    }
    vpcm_log_e2e_phase("DATA",
                       "Transmit plan: seconds=%d frames=%d bytes=%d codewords=%d",
                       data_seconds > 0 ? data_seconds : 10,
                       data_frames,
                       total_bytes,
                       total_codewords);

    fill_pattern(caller_data_in, total_bytes, data_seed);
    fill_pattern(answerer_data_in, total_bytes, data_seed ^ 0xA55AA55AU);
    memset(caller_data_out, 0, (size_t) total_bytes);
    memset(answerer_data_out, 0, (size_t) total_bytes);

    codewords_per_report = vpcm_data_frames_from_seconds(1) * VPCM_CP_FRAME_INTERVALS;
    if (codewords_per_report < VPCM_CP_FRAME_INTERVALS)
        codewords_per_report = VPCM_CP_FRAME_INTERVALS;

    for (codeword_offset = 0; codeword_offset < total_codewords; codeword_offset += codewords_per_report) {
        int chunk_codewords;
        int chunk_frames;
        int chunk_bytes;
        int byte_offset;
        int frame_index;
        bool caller_to_answerer_ok;
        bool answerer_to_caller_ok;

        chunk_codewords = total_codewords - codeword_offset;
        if (chunk_codewords > codewords_per_report)
            chunk_codewords = codewords_per_report;
        chunk_frames = chunk_codewords / VPCM_CP_FRAME_INTERVALS;
        chunk_bytes = (chunk_frames * ((int) cp_ack.drn + 20)) / 8;
        byte_offset = (codeword_offset / VPCM_CP_FRAME_INTERVALS) * ((int) cp_ack.drn + 20) / 8;
        frame_index = codeword_offset / VPCM_CP_FRAME_INTERVALS;

        caller_produced = v91_tx_codewords(&caller_tx,
                                           caller_pcm_tx + codeword_offset,
                                           chunk_codewords,
                                           caller_data_in + byte_offset,
                                           chunk_bytes);
        answerer_produced = v91_tx_codewords(&answerer_tx,
                                             answerer_pcm_tx + codeword_offset,
                                             chunk_codewords,
                                             answerer_data_in + byte_offset,
                                             chunk_bytes);
        if (caller_produced != chunk_codewords || answerer_produced != chunk_codewords) {
            free(caller_data_in);
            free(caller_data_out);
            free(answerer_data_in);
            free(answerer_data_out);
            free(caller_pcm_tx);
            free(caller_pcm_rx);
            free(answerer_pcm_tx);
            free(answerer_pcm_rx);
            fprintf(stderr, "%s tx length mismatch\n", family_label);
            return false;
        }

        vpcm_transport_pcm_family_codewords(robbed_bit,
                                            answerer_pcm_rx + codeword_offset,
                                            caller_pcm_tx + codeword_offset,
                                            chunk_codewords);
        vpcm_transport_pcm_family_codewords(robbed_bit,
                                            caller_pcm_rx + codeword_offset,
                                            answerer_pcm_tx + codeword_offset,
                                            chunk_codewords);

        answerer_consumed = v91_rx_codewords(&answerer_rx,
                                             answerer_data_out + byte_offset,
                                             chunk_bytes,
                                             answerer_pcm_rx + codeword_offset,
                                             chunk_codewords);
        caller_consumed = v91_rx_codewords(&caller_rx,
                                           caller_data_out + byte_offset,
                                           chunk_bytes,
                                           caller_pcm_rx + codeword_offset,
                                           chunk_codewords);
        if (answerer_consumed != chunk_bytes || caller_consumed != chunk_bytes) {
            free(caller_data_in);
            free(caller_data_out);
            free(answerer_data_in);
            free(answerer_data_out);
            free(caller_pcm_tx);
            free(caller_pcm_rx);
            free(answerer_pcm_tx);
            free(answerer_pcm_rx);
            fprintf(stderr, "%s rx length mismatch: caller=%d/%d answerer=%d/%d\n",
                    family_label,
                    caller_consumed,
                    chunk_bytes,
                    answerer_consumed,
                    chunk_bytes);
            return false;
        }

        caller_to_answerer_ok = (memcmp(caller_data_in + byte_offset,
                                        answerer_data_out + byte_offset,
                                        (size_t) chunk_bytes) == 0);
        answerer_to_caller_ok = (memcmp(answerer_data_in + byte_offset,
                                        caller_data_out + byte_offset,
                                        (size_t) chunk_bytes) == 0);
        c2a_chunks_checked++;
        a2c_chunks_checked++;
        c2a_bits_checked += ((uint64_t) chunk_bytes * 8ULL);
        a2c_bits_checked += ((uint64_t) chunk_bytes * 8ULL);
        c2a_bit_errors += vpcm_count_bit_errors(caller_data_in + byte_offset,
                                                answerer_data_out + byte_offset,
                                                chunk_bytes);
        a2c_bit_errors += vpcm_count_bit_errors(answerer_data_in + byte_offset,
                                                caller_data_out + byte_offset,
                                                chunk_bytes);
        if (!caller_to_answerer_ok)
            c2a_mismatch_chunks++;
        if (!answerer_to_caller_ok)
            a2c_mismatch_chunks++;

        if (g_vpcm_realtime) {
            int frame_cw = (chunk_codewords < VPCM_CP_FRAME_INTERVALS) ? chunk_codewords : VPCM_CP_FRAME_INTERVALS;
            int frame_data = (chunk_bytes < 5) ? chunk_bytes : 5;
            char caller_tx_frame[64];
            char answerer_tx_frame[64];
            char caller_tx_data_hex[64];
            char answerer_rx_data_hex[64];
            char answerer_tx_data_hex[64];
            char caller_rx_data_hex[64];
            double c2a_ber;
            double a2c_ber;

            vpcm_bytes_to_hex(caller_tx_frame, sizeof(caller_tx_frame), caller_pcm_tx + codeword_offset, frame_cw);
            vpcm_bytes_to_hex(answerer_tx_frame, sizeof(answerer_tx_frame), answerer_pcm_tx + codeword_offset, frame_cw);
            vpcm_bytes_to_hex(caller_tx_data_hex, sizeof(caller_tx_data_hex), caller_data_in + byte_offset, frame_data);
            vpcm_bytes_to_hex(answerer_rx_data_hex, sizeof(answerer_rx_data_hex), answerer_data_out + byte_offset, frame_data);
            vpcm_bytes_to_hex(answerer_tx_data_hex, sizeof(answerer_tx_data_hex), answerer_data_in + byte_offset, frame_data);
            vpcm_bytes_to_hex(caller_rx_data_hex, sizeof(caller_rx_data_hex), caller_data_out + byte_offset, frame_data);
            c2a_ber = (c2a_bits_checked == 0) ? 0.0 : ((double) c2a_bit_errors / (double) c2a_bits_checked);
            a2c_ber = (a2c_bits_checked == 0) ? 0.0 : ((double) a2c_bit_errors / (double) a2c_bits_checked);
            vpcm_log_e2e_phase("DATA",
                               "frame=%d codeword=%d caller_tx=[%s] answerer_tx=[%s]",
                               frame_index,
                               codeword_offset,
                               caller_tx_frame,
                               answerer_tx_frame);
            vpcm_log_e2e_phase("DATA",
                               "decode frame=%d caller->answerer tx=[%s] rx=[%s] %s, answerer->caller tx=[%s] rx=[%s] %s",
                               frame_index,
                               caller_tx_data_hex,
                               answerer_rx_data_hex,
                               caller_to_answerer_ok ? "OK" : "MISMATCH",
                               answerer_tx_data_hex,
                               caller_rx_data_hex,
                               answerer_to_caller_ok ? "OK" : "MISMATCH");
            vpcm_log_e2e_phase("DATA",
                               "stats frame=%d c2a chunks=%d mismatches=%d bits=%llu bit_errors=%llu ber=%.3e, a2c chunks=%d mismatches=%d bits=%llu bit_errors=%llu ber=%.3e",
                               frame_index,
                               c2a_chunks_checked,
                               c2a_mismatch_chunks,
                               (unsigned long long) c2a_bits_checked,
                               (unsigned long long) c2a_bit_errors,
                               c2a_ber,
                               a2c_chunks_checked,
                               a2c_mismatch_chunks,
                               (unsigned long long) a2c_bits_checked,
                               (unsigned long long) a2c_bit_errors,
                               a2c_ber);
        }
    }

    if (!expect_equal(family_label, caller_data_in, answerer_data_out, total_bytes)
        || !expect_equal(family_label, answerer_data_in, caller_data_out, total_bytes)) {
        free(caller_data_in);
        free(caller_data_out);
        free(answerer_data_in);
        free(answerer_data_out);
        free(caller_pcm_tx);
        free(caller_pcm_rx);
        free(answerer_pcm_tx);
        free(answerer_pcm_rx);
        return false;
    }

    sample_pcm_len = (total_codewords < 6) ? total_codewords : 6;
    if (cp_ack.transparent_mode_granted) {
        sample_data_len = sample_pcm_len;
    } else {
        sample_data_len = (sample_pcm_len * ((int) cp_ack.drn + 20)) / 48;
    }
    if (sample_data_len < 1)
        sample_data_len = 1;
    if (sample_data_len > total_bytes)
        sample_data_len = total_bytes;
    if (!g_vpcm_compact_e2e || g_vpcm_session_diag) {
        vpcm_log_data_sample_named("Caller Data TX",
                                   "Caller PCM TX",
                                   "Answerer PCM RX",
                                   "Answerer Data RX",
                                   caller_data_in,
                                   sample_data_len,
                                   caller_pcm_tx,
                                   sample_pcm_len,
                                   answerer_pcm_rx,
                                   sample_pcm_len,
                                   answerer_data_out,
                                   sample_data_len);
        vpcm_log_data_sample_named("Answerer Data TX",
                                   "Answerer PCM TX",
                                   "Caller PCM RX",
                                   "Caller Data RX",
                                   answerer_data_in,
                                   sample_data_len,
                                   answerer_pcm_tx,
                                   sample_pcm_len,
                                   caller_pcm_rx,
                                   sample_pcm_len,
                                   caller_data_out,
                                   sample_data_len);
    }
    vpcm_log("E2E data stats: c2a chunks=%d mismatches=%d bits=%llu bit_errors=%llu ber=%.3e, a2c chunks=%d mismatches=%d bits=%llu bit_errors=%llu ber=%.3e",
             c2a_chunks_checked,
             c2a_mismatch_chunks,
             (unsigned long long) c2a_bits_checked,
             (unsigned long long) c2a_bit_errors,
             (c2a_bits_checked == 0) ? 0.0 : ((double) c2a_bit_errors / (double) c2a_bits_checked),
             a2c_chunks_checked,
             a2c_mismatch_chunks,
             (unsigned long long) a2c_bits_checked,
             (unsigned long long) a2c_bit_errors,
             (a2c_bits_checked == 0) ? 0.0 : ((double) a2c_bit_errors / (double) a2c_bits_checked));
    vpcm_format_rate(rate_buf, sizeof(rate_buf), cp_ack.drn);
    vpcm_log("PASS: %s (%s, %s, drn=%u, rate=%s, data_seconds=%d, bytes=%d, codewords=%d)",
             family_label,
             vpcm_law_to_str(law),
             path_label,
             cp_ack.drn,
             rate_buf,
             data_seconds > 0 ? data_seconds : 10,
             total_bytes,
             total_codewords);

    free(caller_data_in);
    free(caller_data_out);
    free(answerer_data_in);
    free(answerer_data_out);
    free(caller_pcm_tx);
    free(caller_pcm_rx);
    free(answerer_pcm_tx);
    free(answerer_pcm_rx);
    return true;
}

/*
 * Temporary V.90/V.92 startup contract path.
 *
 * Phase 1 capability exchange is exercised via the real SpanDSP V.8 path.
 * After that, this helper only approximates the startup contract using the
 * existing shared PCM/V.91-style transport primitives so the vpcm harness
 * can keep exercising end-to-end negotiation/adaptation and payload flow.
 *
 * A real V.90/V.92 implementation should replace this with explicit
 * INFO0 -> A/B -> L1/L2 -> INFO1 timing and later native Phase 3/4 flow.
 */
static bool run_v90_v92_startup_contract_session(v91_law_t law,
                                                 const char *path_label,
                                                 bool echo_limited,
                                                 uint32_t seed_base)
{
    enum { NOMINAL_10S_FRAMES = 13328 };
    v91_dil_desc_t default_dil;
    v91_dil_desc_t digital_dil;
    v91_dil_analysis_t digital_dil_analysis;
    v91_state_t caller_startup;
    v91_state_t answerer_startup;
    v91_state_t caller_tx;
    v91_state_t caller_rx;
    v91_state_t answerer_tx;
    v91_state_t answerer_rx;
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    v91_info_frame_t rx_info;
    vpcm_cp_frame_t cp_down_offer;
    vpcm_cp_frame_t cp_down_ack;
    vpcm_cp_frame_t cp_up_offer;
    vpcm_cp_frame_t cp_up_ack;
    vpcm_cp_frame_t cp_rx;
    uint8_t transport_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t startup_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t scr_buf[18];
    uint8_t cp_buf[VPCM_CP_MAX_BITS];
    uint8_t es_buf[V91_ES_SYMBOLS];
    uint8_t b1_buf[V91_B1_SYMBOLS];
    uint8_t *down_data_in;
    uint8_t *down_data_out;
    uint8_t *up_data_in;
    uint8_t *up_data_out;
    uint8_t *down_pcm_tx;
    uint8_t *down_pcm_rx;
    uint8_t *up_pcm_tx;
    uint8_t *up_pcm_rx;
    char down_rate_buf[64];
    char up_rate_buf[64];
    uint8_t downstream_drn;
    uint8_t upstream_drn;
    int startup_len;
    int cp_len;
    int total_codewords;
    int down_total_bits;
    int down_total_bytes;
    int up_total_bits;
    int up_total_bytes;
    int down_produced;
    int down_consumed;
    int up_produced;
    int up_consumed;
    int sample_down_data_len;
    int sample_up_data_len;

    v91_default_dil_init(&default_dil);
    v91_init(&caller_startup, law, V91_MODE_TRANSPARENT);
    v91_init(&answerer_startup, law, V91_MODE_TRANSPARENT);
    v91_init(&caller_tx, law, V91_MODE_TRANSPARENT);
    v91_init(&caller_rx, law, V91_MODE_TRANSPARENT);
    v91_init(&answerer_tx, law, V91_MODE_TRANSPARENT);
    v91_init(&answerer_rx, law, V91_MODE_TRANSPARENT);
    v91_default_dil_init(&default_dil);
    vpcm_v92_init_digital_dil_from_ja(&digital_dil, echo_limited);
    vpcm_log_session_sequence_header("Digital", "Analogue");
    if (g_vpcm_session_diag) {
        vpcm_log("Phase model: Phase 1 = V.8/V.8bis (real SpanDSP path)");
        vpcm_log("Phase model: Phase 2 = INFO0 -> A/B -> L1/L2 -> INFO1 (not yet natively modelled here)");
        vpcm_log("Harness note: rows below after V.8 are a startup contract placeholder built from shared PCM helpers.");
        vpcm_log("Harness note: DIL/adaptation shown below is later-phase approximation, not part of Phase 2.");
    }

    memset(&caller_info, 0, sizeof(caller_info));
    caller_info.request_default_dil = true;
    caller_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    caller_info.power_measured_after_digital_impairments = true;
    memset(&answerer_info, 0, sizeof(answerer_info));
    answerer_info.request_default_dil = true;
    answerer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    answerer_info.power_measured_after_digital_impairments = true;

    if (v91_tx_phase1_silence_codewords(&caller_startup, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS
        || v91_tx_phase1_silence_codewords(&answerer_startup, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS) {
        fprintf(stderr, "V.90/V.92 startup contract path phase1 silence failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("Phase2 preamble TX", "waiting for INFO0", "waiting for INFO0", "Phase2 preamble TX");
    if (v91_tx_ez_codewords(&caller_startup, transport_buf, (int) sizeof(transport_buf)) != V91_EZ_SYMBOLS
        || v91_tx_ez_codewords(&answerer_startup, transport_buf, (int) sizeof(transport_buf)) != V91_EZ_SYMBOLS) {
        fprintf(stderr, "V.90/V.92 startup contract path Ez failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("INFO0-like TX", "waiting for INFO0", "INFO0-like RX", "INFO0-like TX");

    if (v91_tx_info_codewords(&caller_startup, transport_buf, (int) sizeof(transport_buf), &caller_info) != V91_INFO_SYMBOLS
        || !v91_rx_info_codewords(&answerer_startup, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
        fprintf(stderr, "V.90/V.92 startup contract path caller INFO failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("INFO0 ack TX", "waiting for A/B", "INFO0 ack RX", "waiting for A/B");
    vpcm_maybe_log_startup_frame_diag("Digital -> Analogue startup frame",
                                      &caller_startup,
                                      &caller_info,
                                      transport_buf,
                                      V91_INFO_SYMBOLS);
    if (v91_tx_info_codewords(&answerer_startup, transport_buf, (int) sizeof(transport_buf), &answerer_info) != V91_INFO_SYMBOLS
        || !v91_rx_info_codewords(&caller_startup, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
        fprintf(stderr, "V.90/V.92 startup contract path answerer INFO failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("A/B seen", "A/B seen", "waiting for L1/L2", "A/B TX");
    vpcm_maybe_log_startup_frame_diag("Analogue -> Digital startup frame",
                                      &answerer_startup,
                                      &answerer_info,
                                      transport_buf,
                                      V91_INFO_SYMBOLS);

    startup_len = v91_tx_startup_dil_sequence_codewords(&caller_startup,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &digital_dil,
                                                        NULL);
    if (startup_len <= 0) {
        fprintf(stderr, "V.90/V.92 startup contract path caller DIL failed\n");
        return false;
    }
    memcpy(transport_buf, startup_buf, (size_t) startup_len);
    if (!v91_note_received_dil(&answerer_startup, &digital_dil, &digital_dil_analysis)) {
        fprintf(stderr, "V.90/V.92 startup contract path answerer DIL analysis failed\n");
        return false;
    }
    if (g_vpcm_session_diag)
        vpcm_log("Later-phase adaptation block:");
    vpcm_log_session_sequence_row("DIL TX", "waiting for DIL", "DIL RX/analyse", "waiting for DIL");
    vpcm_v92_select_profile_from_dil(&digital_dil_analysis, &downstream_drn, &upstream_drn);
    vpcm_log_dil_analysis("Analogue side", &digital_dil_analysis);

    startup_len = v91_tx_startup_dil_sequence_codewords(&answerer_startup,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &default_dil,
                                                        NULL);
    if (startup_len <= 0) {
        fprintf(stderr, "V.90/V.92 startup contract path answerer DIL failed\n");
        return false;
    }
    memcpy(transport_buf, startup_buf, (size_t) startup_len);
    if (!v91_note_received_dil(&caller_startup, &default_dil, NULL)) {
        fprintf(stderr, "V.90/V.92 startup contract path caller DIL tracking failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("DIL RX", "DIL RX/track", "waiting for adapt", "DIL TX");

    if (v91_tx_scr_codewords(&caller_startup, scr_buf, (int) sizeof(scr_buf), 18) != 18
        || !v91_rx_scr_codewords(&answerer_startup, scr_buf, 18, false)) {
        fprintf(stderr, "V.90/V.92 startup contract path caller conditioning failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("adapt TX", "waiting for adapt", "adapt RX", "waiting for adapt");
    if (v91_tx_scr_codewords(&answerer_startup, scr_buf, (int) sizeof(scr_buf), 18) != 18
        || !v91_rx_scr_codewords(&caller_startup, scr_buf, 18, false)) {
        fprintf(stderr, "V.90/V.92 startup contract path answerer conditioning failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("adapt RX", "adapt RX", "waiting for req", "adapt TX");

    vpcm_cp_init(&cp_down_offer);
    cp_down_offer.transparent_mode_granted = false;
    cp_down_offer.v90_compatibility = true;
    cp_down_offer.drn = downstream_drn;
    cp_down_offer.acknowledge = false;
    cp_down_offer.constellation_count = 1;
    memset(cp_down_offer.dfi, 0, sizeof(cp_down_offer.dfi));
    vpcm_cp_enable_all_ucodes(cp_down_offer.masks[0]);

    cp_len = v91_tx_cp_codewords(&caller_startup, cp_buf, (int) sizeof(cp_buf), &cp_down_offer, true);
    if (cp_len <= 0 || !v91_rx_cp_codewords(&answerer_startup, cp_buf, cp_len, &cp_rx, true) || !vpcm_cp_frames_equal(&cp_down_offer, &cp_rx)) {
        fprintf(stderr, "V.90/V.92 startup contract path downstream rate request failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("INFO1 down TX", "waiting for INFO1 ack", "INFO1 down RX", "waiting for INFO1 ack");
    vpcm_maybe_log_rate_request_diag("Digital -> Analogue downstream request",
                                     &cp_down_offer,
                                     cp_buf,
                                     cp_len);
    cp_down_ack = cp_down_offer;
    cp_down_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&answerer_startup, cp_buf, (int) sizeof(cp_buf), &cp_down_ack, true);
    if (cp_len <= 0 || !v91_rx_cp_codewords(&caller_startup, cp_buf, cp_len, &cp_rx, true) || !vpcm_cp_frames_equal(&cp_down_ack, &cp_rx)) {
        fprintf(stderr, "V.90/V.92 startup contract path downstream acknowledge failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("INFO1 down RX", "INFO1 down RX", "waiting for train", "INFO1 down TX");
    if (v91_tx_es_codewords(&caller_startup, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
        || !v91_rx_es_codewords(&answerer_startup, es_buf, V91_ES_SYMBOLS, true)
        || v91_tx_b1_codewords(&caller_startup, b1_buf, (int) sizeof(b1_buf), &cp_down_ack) != V91_B1_SYMBOLS
        || !v91_rx_b1_codewords(&answerer_startup, b1_buf, V91_B1_SYMBOLS, &cp_down_ack)) {
        fprintf(stderr, "V.90/V.92 startup contract path downstream startup sequence failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("down train TX", "waiting for up INFO1", "down train RX", "waiting for up INFO1");

    vpcm_cp_init(&cp_up_offer);
    cp_up_offer.transparent_mode_granted = false;
    cp_up_offer.v90_compatibility = true;
    cp_up_offer.drn = upstream_drn;
    cp_up_offer.acknowledge = false;
    cp_up_offer.constellation_count = 1;
    memset(cp_up_offer.dfi, 0, sizeof(cp_up_offer.dfi));
    vpcm_cp_enable_all_ucodes(cp_up_offer.masks[0]);

    cp_len = v91_tx_cp_codewords(&answerer_startup, cp_buf, (int) sizeof(cp_buf), &cp_up_offer, true);
    if (cp_len <= 0 || !v91_rx_cp_codewords(&caller_startup, cp_buf, cp_len, &cp_rx, true) || !vpcm_cp_frames_equal(&cp_up_offer, &cp_rx)) {
        fprintf(stderr, "V.90/V.92 startup contract path upstream rate request failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("waiting for up ack", "INFO1 up RX", "waiting for up ack", "INFO1 up TX");
    vpcm_maybe_log_rate_request_diag("Analogue -> Digital upstream request",
                                     &cp_up_offer,
                                     cp_buf,
                                     cp_len);
    cp_up_ack = cp_up_offer;
    cp_up_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&caller_startup, cp_buf, (int) sizeof(cp_buf), &cp_up_ack, true);
    if (cp_len <= 0 || !v91_rx_cp_codewords(&answerer_startup, cp_buf, cp_len, &cp_rx, true) || !vpcm_cp_frames_equal(&cp_up_ack, &cp_rx)) {
        fprintf(stderr, "V.90/V.92 startup contract path upstream acknowledge failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("INFO1 up TX", "waiting for up train", "INFO1 up RX", "INFO1 up TX");
    if (v91_tx_es_codewords(&answerer_startup, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
        || !v91_rx_es_codewords(&caller_startup, es_buf, V91_ES_SYMBOLS, true)
        || v91_tx_b1_codewords(&answerer_startup, b1_buf, (int) sizeof(b1_buf), &cp_up_ack) != V91_B1_SYMBOLS
        || !v91_rx_b1_codewords(&caller_startup, b1_buf, V91_B1_SYMBOLS, &cp_up_ack)) {
        fprintf(stderr, "V.90/V.92 startup contract path upstream startup sequence failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("waiting for DATA", "up train RX", "waiting for DATA", "up train TX");

    if (!v91_activate_data_mode(&caller_tx, &cp_down_ack)
        || !v91_activate_data_mode(&answerer_rx, &cp_down_ack)
        || !v91_activate_data_mode(&answerer_tx, &cp_up_ack)
        || !v91_activate_data_mode(&caller_rx, &cp_up_ack)) {
        fprintf(stderr, "V.90/V.92 startup contract path data-mode activation failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("DATA down TX", "DATA up RX", "DATA down RX", "DATA up TX");

    total_codewords = NOMINAL_10S_FRAMES * VPCM_CP_FRAME_INTERVALS;
    down_total_bits = NOMINAL_10S_FRAMES * ((int) cp_down_ack.drn + 20);
    down_total_bytes = down_total_bits / 8;
    up_total_bits = NOMINAL_10S_FRAMES * ((int) cp_up_ack.drn + 20);
    up_total_bytes = up_total_bits / 8;
    down_data_in = (uint8_t *) malloc((size_t) down_total_bytes);
    down_data_out = (uint8_t *) malloc((size_t) down_total_bytes);
    up_data_in = (uint8_t *) malloc((size_t) up_total_bytes);
    up_data_out = (uint8_t *) malloc((size_t) up_total_bytes);
    down_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    down_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    up_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    up_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    if (!down_data_in || !down_data_out || !up_data_in || !up_data_out
        || !down_pcm_tx || !down_pcm_rx || !up_pcm_tx || !up_pcm_rx) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        fprintf(stderr, "allocation failure in V.90/V.92 startup contract session\n");
        return false;
    }

    fill_pattern(down_data_in, down_total_bytes, seed_base ^ 0x00D04E00U);
    fill_pattern(up_data_in, up_total_bytes, seed_base ^ 0x00A0B000U);
    memset(down_data_out, 0, (size_t) down_total_bytes);
    memset(up_data_out, 0, (size_t) up_total_bytes);

    down_produced = v91_tx_codewords(&caller_tx, down_pcm_tx, total_codewords, down_data_in, down_total_bytes);
    up_produced = v91_tx_codewords(&answerer_tx, up_pcm_tx, total_codewords, up_data_in, up_total_bytes);
    if (down_produced != total_codewords || up_produced != total_codewords) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        fprintf(stderr, "V.90/V.92 startup contract path tx length mismatch\n");
        return false;
    }

    memcpy(down_pcm_rx, down_pcm_tx, (size_t) total_codewords);
    memcpy(up_pcm_rx, up_pcm_tx, (size_t) total_codewords);
    down_consumed = v91_rx_codewords(&answerer_rx, down_data_out, down_total_bytes, down_pcm_rx, total_codewords);
    up_consumed = v91_rx_codewords(&caller_rx, up_data_out, up_total_bytes, up_pcm_rx, total_codewords);
    if (down_consumed != down_total_bytes || up_consumed != up_total_bytes) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        fprintf(stderr, "V.90/V.92 startup contract path rx length mismatch down=%d/%d up=%d/%d\n",
                down_consumed, down_total_bytes, up_consumed, up_total_bytes);
        return false;
    }
    if (!expect_equal("V.92 downstream payload", down_data_in, down_data_out, down_total_bytes)
        || !expect_equal("V.92 upstream payload", up_data_in, up_data_out, up_total_bytes)) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        return false;
    }

    sample_down_data_len = (6 * ((int) cp_down_ack.drn + 20)) / 48;
    sample_up_data_len = (6 * ((int) cp_up_ack.drn + 20)) / 48;
    if (sample_down_data_len < 1)
        sample_down_data_len = 1;
    if (sample_up_data_len < 1)
        sample_up_data_len = 1;
    if (sample_down_data_len > down_total_bytes)
        sample_down_data_len = down_total_bytes;
    if (sample_up_data_len > up_total_bytes)
        sample_up_data_len = up_total_bytes;

    vpcm_log("Digital -> Analogue sample");
    vpcm_log_data_sample(down_data_in,
                         sample_down_data_len,
                         down_pcm_tx,
                         6,
                         down_pcm_rx,
                         6,
                         down_data_out,
                         sample_down_data_len);
    vpcm_log("Analogue -> Digital sample");
    vpcm_log_data_sample(up_data_in,
                         sample_up_data_len,
                         up_pcm_tx,
                         6,
                         up_pcm_rx,
                         6,
                         up_data_out,
                         sample_up_data_len);

    vpcm_format_rate(down_rate_buf, sizeof(down_rate_buf), cp_down_ack.drn);
    vpcm_format_rate(up_rate_buf, sizeof(up_rate_buf), cp_up_ack.drn);
    vpcm_log("PASS: V.90/V.92 startup contract path (%s, %s, digital->analogue=%s, analogue->digital=%s%s)",
             vpcm_law_to_str(law),
             path_label,
             down_rate_buf,
             up_rate_buf,
             echo_limited ? ", echo-adapted" : "");
    vpcm_log_session_sequence_footer();

    free(down_data_in);
    free(down_data_out);
    free(up_data_in);
    free(up_data_out);
    free(down_pcm_tx);
    free(down_pcm_rx);
    free(up_pcm_tx);
    free(up_pcm_rx);
    return true;
}

static bool test_v91_startup_to_data_robbed_bit(v91_law_t law)
{
    static const struct {
        bool transparent;
        const char *mode_label;
    } cases[] = {
        {false, "non-transparent"},
        {true,  "transparent"},
    };
    enum { NOMINAL_10S_FRAMES = 13328 };
    v91_dil_desc_t default_dil;
    int i;

    vpcm_log("Test: V.91 startup -> data over robbed-bit signalling (%s)", vpcm_law_to_str(law));
    v91_default_dil_init(&default_dil);

    for (i = 0; i < (int) (sizeof(cases)/sizeof(cases[0])); i++) {
        v91_state_t caller;
        v91_state_t answerer;
        v91_info_frame_t caller_info;
        v91_info_frame_t answerer_info;
        v91_info_frame_t rx_info;
        vpcm_cp_frame_t cp_offer;
        vpcm_cp_frame_t cp_ack;
        vpcm_cp_frame_t cp_rx;
        uint8_t transport_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
        uint8_t startup_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
        uint8_t scr_buf[18];
        uint8_t cp_buf[VPCM_CP_MAX_BITS];
        uint8_t es_buf[V91_ES_SYMBOLS];
        uint8_t b1_buf[V91_B1_SYMBOLS];
        uint8_t *data_in;
        uint8_t *data_out;
        uint8_t *pcm_tx;
        uint8_t *pcm_rx;
        int startup_len;
        int cp_len;
        int total_bits;
        int total_bytes;
        int total_codewords;
        int produced;
        int consumed;
        int mismatch_at;
        int sample_len;

        v91_init(&caller, law, V91_MODE_TRANSPARENT);
        v91_init(&answerer, law, V91_MODE_TRANSPARENT);

        memset(&caller_info, 0, sizeof(caller_info));
        caller_info.request_default_dil = true;
        caller_info.tx_uses_alaw = (law == V91_LAW_ALAW);
        caller_info.power_measured_after_digital_impairments = true;
        caller_info.request_transparent_mode = cases[i].transparent;
        caller_info.cleardown_if_transparent_denied = cases[i].transparent;

        memset(&answerer_info, 0, sizeof(answerer_info));
        answerer_info.request_default_dil = true;
        answerer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
        answerer_info.power_measured_after_digital_impairments = true;
        answerer_info.request_transparent_mode = cases[i].transparent;
        answerer_info.cleardown_if_transparent_denied = cases[i].transparent;

        if (v91_tx_info_codewords(&caller, transport_buf, (int) sizeof(transport_buf), &caller_info) != V91_INFO_SYMBOLS) {
            fprintf(stderr, "robbed-bit startup caller INFO tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, transport_buf, V91_INFO_SYMBOLS);
        if (!v91_rx_info_codewords(&answerer, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
            fprintf(stderr, "robbed-bit startup caller->answerer INFO rx failed\n");
            return false;
        }

        if (v91_tx_info_codewords(&answerer, transport_buf, (int) sizeof(transport_buf), &answerer_info) != V91_INFO_SYMBOLS) {
            fprintf(stderr, "robbed-bit startup answerer INFO tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, transport_buf, V91_INFO_SYMBOLS);
        if (!v91_rx_info_codewords(&caller, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
            fprintf(stderr, "robbed-bit startup answerer->caller INFO rx failed\n");
            return false;
        }

        startup_len = v91_tx_startup_dil_sequence_codewords(&caller,
                                                            startup_buf,
                                                            (int) sizeof(startup_buf),
                                                            &default_dil,
                                                            NULL);
        if (startup_len <= 0) {
            fprintf(stderr, "robbed-bit startup caller DIL sequence failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, startup_buf, startup_len);

        startup_len = v91_tx_startup_dil_sequence_codewords(&answerer,
                                                            startup_buf,
                                                            (int) sizeof(startup_buf),
                                                            &default_dil,
                                                            NULL);
        if (startup_len <= 0) {
            fprintf(stderr, "robbed-bit startup answerer DIL sequence failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, startup_buf, startup_len);

        if (v91_tx_scr_codewords(&caller, scr_buf, (int) sizeof(scr_buf), 18) != 18) {
            fprintf(stderr, "robbed-bit startup caller SCR tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, scr_buf, 18);
        if (!v91_rx_scr_codewords(&answerer, transport_buf, 18, false)) {
            fprintf(stderr, "robbed-bit startup caller SCR rx failed\n");
            return false;
        }

        if (v91_tx_scr_codewords(&answerer, scr_buf, (int) sizeof(scr_buf), 18) != 18) {
            fprintf(stderr, "robbed-bit startup answerer SCR tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, scr_buf, 18);
        if (!v91_rx_scr_codewords(&caller, transport_buf, 18, false)) {
            fprintf(stderr, "robbed-bit startup answerer SCR rx failed\n");
            return false;
        }

        vpcm_cp_init(&cp_offer);
        cp_offer.transparent_mode_granted = cases[i].transparent;
        cp_offer.v90_compatibility = true;
        cp_offer.drn = 28;
        cp_offer.acknowledge = false;
        cp_offer.constellation_count = 1;
        memset(cp_offer.dfi, 0, sizeof(cp_offer.dfi));
        vpcm_cp_enable_all_ucodes(cp_offer.masks[0]);

        cp_len = v91_tx_cp_codewords(&caller, cp_buf, (int) sizeof(cp_buf), &cp_offer, true);
        if (cp_len <= 0) {
            fprintf(stderr, "robbed-bit startup caller CP tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, cp_buf, cp_len);
        if (!v91_rx_cp_codewords(&answerer, transport_buf, cp_len, &cp_rx, true)
            || !vpcm_cp_frames_equal(&cp_offer, &cp_rx)) {
            fprintf(stderr, "robbed-bit startup caller CP rx failed\n");
            return false;
        }

        cp_ack = cp_offer;
        cp_ack.acknowledge = true;
        cp_len = v91_tx_cp_codewords(&answerer, cp_buf, (int) sizeof(cp_buf), &cp_ack, true);
        if (cp_len <= 0) {
            fprintf(stderr, "robbed-bit startup answerer CP tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, cp_buf, cp_len);
        if (!v91_rx_cp_codewords(&caller, transport_buf, cp_len, &cp_rx, true)
            || !vpcm_cp_frames_equal(&cp_ack, &cp_rx)) {
            fprintf(stderr, "robbed-bit startup answerer CP rx failed\n");
            return false;
        }

        if (v91_tx_es_codewords(&caller, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS) {
            fprintf(stderr, "robbed-bit startup caller Es tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, es_buf, V91_ES_SYMBOLS);
        if (!v91_rx_es_codewords(&answerer, transport_buf, V91_ES_SYMBOLS, true)) {
            fprintf(stderr, "robbed-bit startup caller Es rx failed\n");
            return false;
        }

        if (v91_tx_b1_codewords(&caller, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS) {
            fprintf(stderr, "robbed-bit startup caller B1 tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, b1_buf, V91_B1_SYMBOLS);
        if (!v91_rx_b1_codewords(&answerer, transport_buf, V91_B1_SYMBOLS, &cp_ack)) {
            fprintf(stderr, "robbed-bit startup caller B1 rx failed\n");
            return false;
        }

        if (v91_tx_es_codewords(&answerer, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS) {
            fprintf(stderr, "robbed-bit startup answerer Es tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, es_buf, V91_ES_SYMBOLS);
        if (!v91_rx_es_codewords(&caller, transport_buf, V91_ES_SYMBOLS, true)) {
            fprintf(stderr, "robbed-bit startup answerer Es rx failed\n");
            return false;
        }

        if (v91_tx_b1_codewords(&answerer, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS) {
            fprintf(stderr, "robbed-bit startup answerer B1 tx failed\n");
            return false;
        }
        vpcm_transport_robbed_bit_codewords(transport_buf, b1_buf, V91_B1_SYMBOLS);
        if (!v91_rx_b1_codewords(&caller, transport_buf, V91_B1_SYMBOLS, &cp_ack)) {
            fprintf(stderr, "robbed-bit startup answerer B1 rx failed\n");
            return false;
        }

        if (!v91_activate_data_mode(&caller, &cp_ack) || !v91_activate_data_mode(&answerer, &cp_ack)) {
            fprintf(stderr, "robbed-bit startup data-mode activation failed\n");
            return false;
        }

        total_bits = NOMINAL_10S_FRAMES * (28 + 20);
        total_bytes = total_bits / 8;
        total_codewords = NOMINAL_10S_FRAMES * VPCM_CP_FRAME_INTERVALS;
        data_in = (uint8_t *) malloc((size_t) total_bytes);
        data_out = (uint8_t *) malloc((size_t) total_bytes);
        pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
        pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
        if (!data_in || !data_out || !pcm_tx || !pcm_rx) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            free(pcm_rx);
            fprintf(stderr, "allocation failure in robbed-bit startup->data test\n");
            return false;
        }

        fill_pattern(data_in, total_bytes, 0x91570000U ^ ((uint32_t) law << 8) ^ (uint32_t) i);
        memset(data_out, 0, (size_t) total_bytes);
        produced = v91_tx_codewords(&caller, pcm_tx, total_codewords, data_in, total_bytes);
        if (produced != total_codewords) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            free(pcm_rx);
            fprintf(stderr, "robbed-bit startup->data tx length mismatch\n");
            return false;
        }

        vpcm_transport_robbed_bit_codewords(pcm_rx, pcm_tx, produced);
        consumed = v91_rx_codewords(&answerer, data_out, total_bytes, pcm_rx, produced);
        mismatch_at = (consumed < total_bytes) ? consumed : first_mismatch_index(data_in, data_out, total_bytes);
        if (mismatch_at < 0) {
            free(data_in);
            free(data_out);
            free(pcm_tx);
            free(pcm_rx);
            fprintf(stderr,
                    "robbed-bit startup->data unexpectedly preserved drn=28 %s payload\n",
                    cases[i].mode_label);
            return false;
        }

        sample_len = 6;
        if (sample_len > total_bytes)
            sample_len = total_bytes;
        vpcm_log_data_sample(data_in,
                             sample_len,
                             pcm_tx,
                             6,
                             pcm_rx,
                             6,
                             data_out,
                             (consumed < sample_len) ? consumed : sample_len);
        vpcm_log("PASS: startup -> data over robbed-bit signalling degrades %s drn=28 as expected (%s, first mismatch at byte %d)",
                 cases[i].mode_label,
                 vpcm_law_to_str(law),
                 mismatch_at);

        free(data_in);
        free(data_out);
        free(pcm_tx);
        free(pcm_rx);
    }

    vpcm_log("PASS: V.91 startup -> data over robbed-bit signalling (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_startup_to_data_robbed_bit_safe_rate(v91_law_t law)
{
    enum { NOMINAL_10S_FRAMES = 13328 };
    v91_dil_desc_t robbed_bit_dil;
    v91_dil_analysis_t dil_analysis;
    v91_state_t caller;
    v91_state_t answerer;
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    v91_info_frame_t rx_info;
    vpcm_cp_frame_t cp_offer;
    vpcm_cp_frame_t cp_ack;
    vpcm_cp_frame_t cp_rx;
    uint8_t transport_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t startup_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t scr_buf[18];
    uint8_t cp_buf[VPCM_CP_MAX_BITS];
    uint8_t es_buf[V91_ES_SYMBOLS];
    uint8_t b1_buf[V91_B1_SYMBOLS];
    uint8_t *data_in;
    uint8_t *data_out;
    uint8_t *pcm_tx;
    uint8_t *pcm_rx;
    char rate_buf[64];
    uint8_t drn;
    int startup_len;
    int cp_len;
    int total_bits;
    int total_bytes;
    int total_codewords;
    int produced;
    int consumed;
    int sample_len;

    vpcm_log("Test: V.91 startup -> data over robbed-bit signalling (%s, safe rate)", vpcm_law_to_str(law));
    vpcm_init_robbed_bit_dil_profile(&robbed_bit_dil);
    v91_init(&caller, law, V91_MODE_TRANSPARENT);
    v91_init(&answerer, law, V91_MODE_TRANSPARENT);
    vpcm_log_session_sequence_header("Caller", "Answerer");

    memset(&caller_info, 0, sizeof(caller_info));
    caller_info.request_default_dil = true;
    caller_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    caller_info.power_measured_after_digital_impairments = true;

    memset(&answerer_info, 0, sizeof(answerer_info));
    answerer_info.request_default_dil = true;
    answerer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    answerer_info.power_measured_after_digital_impairments = true;

    if (v91_tx_info_codewords(&caller, transport_buf, (int) sizeof(transport_buf), &caller_info) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "robbed-bit safe-rate caller INFO tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, transport_buf, V91_INFO_SYMBOLS);
    if (!v91_rx_info_codewords(&answerer, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
        fprintf(stderr, "robbed-bit safe-rate caller->answerer INFO rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("INFO TX", "waiting for INFO'", "INFO RX", "waiting for INFO'");
    vpcm_maybe_log_info_session_diag(&caller, &caller_info, transport_buf, V91_INFO_SYMBOLS, "Caller -> Answerer");

    if (v91_tx_info_codewords(&answerer, transport_buf, (int) sizeof(transport_buf), &answerer_info) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "robbed-bit safe-rate answerer INFO tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, transport_buf, V91_INFO_SYMBOLS);
    if (!v91_rx_info_codewords(&caller, transport_buf, V91_INFO_SYMBOLS, &rx_info)) {
        fprintf(stderr, "robbed-bit safe-rate answerer->caller INFO rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("INFO' RX", "INFO' RX", "waiting for DIL", "INFO' TX");
    vpcm_maybe_log_info_session_diag(&answerer, &answerer_info, transport_buf, V91_INFO_SYMBOLS, "Answerer -> Caller");

    startup_len = v91_tx_startup_dil_sequence_codewords(&caller,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &robbed_bit_dil,
                                                        NULL);
    if (startup_len <= 0) {
        fprintf(stderr, "robbed-bit safe-rate caller DIL sequence failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, startup_buf, startup_len);
    if (!v91_note_received_dil(&answerer, &robbed_bit_dil, &dil_analysis)) {
        fprintf(stderr, "robbed-bit safe-rate answerer DIL analysis failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("DIL TX", "waiting for DIL", "DIL RX/analyse", "waiting for DIL");
    vpcm_log_dil_analysis("Analogue side", &dil_analysis);
    drn = dil_analysis.recommended_downstream_drn;
    if (!dil_analysis.robbed_bit_limited || drn != vpcm_cp_recommended_robbed_bit_drn()) {
        fprintf(stderr, "robbed-bit safe-rate DIL analysis selected drn=%u expected=%u\n",
                drn, vpcm_cp_recommended_robbed_bit_drn());
        return false;
    }

    startup_len = v91_tx_startup_dil_sequence_codewords(&answerer,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &robbed_bit_dil,
                                                        NULL);
    if (startup_len <= 0) {
        fprintf(stderr, "robbed-bit safe-rate answerer DIL sequence failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, startup_buf, startup_len);
    if (!v91_note_received_dil(&caller, &robbed_bit_dil, NULL)) {
        fprintf(stderr, "robbed-bit safe-rate caller DIL tracking failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("DIL RX", "DIL RX/track", "waiting for SCR", "DIL TX");

    if (v91_tx_scr_codewords(&caller, scr_buf, (int) sizeof(scr_buf), 18) != 18) {
        fprintf(stderr, "robbed-bit safe-rate caller SCR tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, scr_buf, 18);
    if (!v91_rx_scr_codewords(&answerer, transport_buf, 18, false)) {
        fprintf(stderr, "robbed-bit safe-rate caller SCR rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("SCR TX", "waiting for SCR", "SCR RX", "waiting for SCR");

    if (v91_tx_scr_codewords(&answerer, scr_buf, (int) sizeof(scr_buf), 18) != 18) {
        fprintf(stderr, "robbed-bit safe-rate answerer SCR tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, scr_buf, 18);
    if (!v91_rx_scr_codewords(&caller, transport_buf, 18, false)) {
        fprintf(stderr, "robbed-bit safe-rate answerer SCR rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("SCR RX", "SCR RX", "waiting for CP", "SCR TX");

    vpcm_cp_init_robbed_bit_safe_profile(&cp_offer, drn, false);

    cp_len = v91_tx_cp_codewords(&caller, cp_buf, (int) sizeof(cp_buf), &cp_offer, true);
    if (cp_len <= 0) {
        fprintf(stderr, "robbed-bit safe-rate caller CP tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, cp_buf, cp_len);
    if (!v91_rx_cp_codewords(&answerer, transport_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&cp_offer, &cp_rx)) {
        fprintf(stderr, "robbed-bit safe-rate caller CP rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("CP TX", "waiting for CP'", "CP RX", "waiting for CP'");
    vpcm_maybe_log_cp_session_diag(&cp_offer, transport_buf, cp_len, "Caller -> Answerer");

    cp_ack = cp_offer;
    cp_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&answerer, cp_buf, (int) sizeof(cp_buf), &cp_ack, true);
    if (cp_len <= 0) {
        fprintf(stderr, "robbed-bit safe-rate answerer CP tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, cp_buf, cp_len);
    if (!v91_rx_cp_codewords(&caller, transport_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&cp_ack, &cp_rx)) {
        fprintf(stderr, "robbed-bit safe-rate answerer CP rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("CP' RX", "CP' RX", "waiting for Es/B1", "CP' TX");

    if (v91_tx_es_codewords(&caller, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS) {
        fprintf(stderr, "robbed-bit safe-rate caller Es tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, es_buf, V91_ES_SYMBOLS);
    if (!v91_rx_es_codewords(&answerer, transport_buf, V91_ES_SYMBOLS, true)) {
        fprintf(stderr, "robbed-bit safe-rate caller Es rx failed\n");
        return false;
    }

    if (v91_tx_b1_codewords(&caller, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS) {
        fprintf(stderr, "robbed-bit safe-rate caller B1 tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, b1_buf, V91_B1_SYMBOLS);
    if (!v91_rx_b1_codewords(&answerer, transport_buf, V91_B1_SYMBOLS, &cp_ack)) {
        fprintf(stderr, "robbed-bit safe-rate caller B1 rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("Es/B1 TX", "waiting for Es/B1", "Es/B1 RX", "waiting for Es/B1");

    if (v91_tx_es_codewords(&answerer, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS) {
        fprintf(stderr, "robbed-bit safe-rate answerer Es tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, es_buf, V91_ES_SYMBOLS);
    if (!v91_rx_es_codewords(&caller, transport_buf, V91_ES_SYMBOLS, true)) {
        fprintf(stderr, "robbed-bit safe-rate answerer Es rx failed\n");
        return false;
    }

    if (v91_tx_b1_codewords(&answerer, b1_buf, (int) sizeof(b1_buf), &cp_ack) != V91_B1_SYMBOLS) {
        fprintf(stderr, "robbed-bit safe-rate answerer B1 tx failed\n");
        return false;
    }
    vpcm_transport_robbed_bit_codewords(transport_buf, b1_buf, V91_B1_SYMBOLS);
    if (!v91_rx_b1_codewords(&caller, transport_buf, V91_B1_SYMBOLS, &cp_ack)) {
        fprintf(stderr, "robbed-bit safe-rate answerer B1 rx failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("waiting for DATA", "Es/B1 RX", "waiting for DATA", "Es/B1 TX");

    if (!v91_activate_data_mode(&caller, &cp_ack) || !v91_activate_data_mode(&answerer, &cp_ack)) {
        fprintf(stderr, "robbed-bit safe-rate data-mode activation failed\n");
        return false;
    }
    vpcm_log_session_sequence_row("DATA TX", "waiting for DATA", "DATA RX", "waiting for DATA");

    total_bits = NOMINAL_10S_FRAMES * ((int) drn + 20);
    total_bytes = total_bits / 8;
    total_codewords = NOMINAL_10S_FRAMES * VPCM_CP_FRAME_INTERVALS;
    data_in = (uint8_t *) malloc((size_t) total_bytes);
    data_out = (uint8_t *) malloc((size_t) total_bytes);
    pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    if (!data_in || !data_out || !pcm_tx || !pcm_rx) {
        free(data_in);
        free(data_out);
        free(pcm_tx);
        free(pcm_rx);
        fprintf(stderr, "allocation failure in robbed-bit safe-rate test\n");
        return false;
    }

    fill_pattern(data_in, total_bytes, 0x91560000U ^ ((uint32_t) law << 8));
    memset(data_out, 0, (size_t) total_bytes);
    produced = v91_tx_codewords(&caller, pcm_tx, total_codewords, data_in, total_bytes);
    if (produced != total_codewords) {
        free(data_in);
        free(data_out);
        free(pcm_tx);
        free(pcm_rx);
        fprintf(stderr, "robbed-bit safe-rate tx length mismatch\n");
        return false;
    }

    vpcm_transport_robbed_bit_codewords(pcm_rx, pcm_tx, produced);
    consumed = v91_rx_codewords(&answerer, data_out, total_bytes, pcm_rx, produced);
    if (consumed != total_bytes) {
        free(data_in);
        free(data_out);
        free(pcm_tx);
        free(pcm_rx);
        fprintf(stderr, "robbed-bit safe-rate rx length mismatch: %d/%d\n", consumed, total_bytes);
        return false;
    }
    if (!expect_equal("V.91 robbed-bit safe-rate data", data_in, data_out, total_bytes)) {
        sample_len = 12;
        if (sample_len > total_bytes)
            sample_len = total_bytes;
        vpcm_log_data_sample(data_in,
                             sample_len,
                             pcm_tx,
                             12,
                             pcm_rx,
                             12,
                             data_out,
                             sample_len);
        free(data_in);
        free(data_out);
        free(pcm_tx);
        free(pcm_rx);
        return false;
    }

    sample_len = 6;
    if (sample_len > total_bytes)
        sample_len = total_bytes;
    vpcm_log_data_sample(data_in,
                         sample_len,
                         pcm_tx,
                         6,
                         pcm_rx,
                         6,
                         data_out,
                         sample_len);
    vpcm_format_rate(rate_buf, sizeof(rate_buf), drn);
    vpcm_log("PASS: startup -> data over robbed-bit signalling holds at safe rate (%s, drn=%u, rate=%s, ceiling=%.2f bps)",
             vpcm_law_to_str(law),
             drn,
             rate_buf,
             vpcm_cp_robbed_bit_ceiling_bps());
    vpcm_log_session_sequence_footer();

    free(data_in);
    free(data_out);
    free(pcm_tx);
    free(pcm_rx);
    return true;
}

static void fill_pattern(uint8_t *buf, int len, uint32_t seed)
{
    int i;

    for (i = 0; i < len; i++)
        buf[i] = (uint8_t) prng_next(&seed);
}

static void vpcm_bytes_to_hex(char *out, size_t out_len, const uint8_t *buf, int len)
{
    int i;
    size_t pos;

    if (out_len == 0)
        return;
    pos = 0;
    for (i = 0; i < len && pos + 3 < out_len; i++) {
        pos += (size_t) snprintf(out + pos, out_len - pos, "%02X", buf[i]);
        if (i + 1 < len && pos + 1 < out_len)
            out[pos++] = ' ';
    }
    out[pos] = '\0';
}

static uint64_t vpcm_count_bit_errors(const uint8_t *a, const uint8_t *b, int len)
{
    uint64_t total;
    int i;

    total = 0;
    for (i = 0; i < len; i++) {
        uint8_t x;

        x = (uint8_t) (a[i] ^ b[i]);
#if defined(__GNUC__) || defined(__clang__)
        total += (uint64_t) __builtin_popcount((unsigned int) x);
#else
        while (x != 0) {
            x &= (uint8_t) (x - 1U);
            total++;
        }
#endif
    }
    return total;
}

static void vpcm_log_data_sample_named(const char *h1,
                                       const char *h2,
                                       const char *h3,
                                       const char *h4,
                                       const uint8_t *data_tx,
                                       int data_tx_len,
                                       const uint8_t *pcm_tx,
                                       int pcm_tx_len,
                                       const uint8_t *remote_pcm_rx,
                                       int remote_pcm_rx_len,
                                       const uint8_t *remote_data_rx,
                                       int remote_data_rx_len)
{
    char data_tx_hex[128];
    char pcm_tx_hex[128];
    char remote_pcm_hex[128];
    char remote_data_hex[128];

    vpcm_bytes_to_hex(data_tx_hex, sizeof(data_tx_hex), data_tx, data_tx_len);
    vpcm_bytes_to_hex(pcm_tx_hex, sizeof(pcm_tx_hex), pcm_tx, pcm_tx_len);
    vpcm_bytes_to_hex(remote_pcm_hex, sizeof(remote_pcm_hex), remote_pcm_rx, remote_pcm_rx_len);
    vpcm_bytes_to_hex(remote_data_hex, sizeof(remote_data_hex), remote_data_rx, remote_data_rx_len);

    vpcm_log("+---------------------------+---------------------------+---------------------------+---------------------------+");
    vpcm_log("| %-25s | %-25s | %-25s | %-25s |", h1, h2, h3, h4);
    vpcm_log("+---------------------------+---------------------------+---------------------------+---------------------------+");
    vpcm_log("| %-25s | %-25s | %-25s | %-25s |",
             data_tx_hex, pcm_tx_hex, remote_pcm_hex, remote_data_hex);
    vpcm_log("+---------------------------+---------------------------+---------------------------+---------------------------+");
}

static void vpcm_log_data_sample(const uint8_t *data_tx,
                                 int data_tx_len,
                                 const uint8_t *pcm_tx,
                                 int pcm_tx_len,
                                 const uint8_t *remote_pcm_rx,
                                 int remote_pcm_rx_len,
                                 const uint8_t *remote_data_rx,
                                 int remote_data_rx_len)
{
    vpcm_log_data_sample_named("Data TX",
                               "PCM TX",
                               "Remote PCM RX",
                               "Remote Data RX",
                               data_tx,
                               data_tx_len,
                               pcm_tx,
                               pcm_tx_len,
                               remote_pcm_rx,
                               remote_pcm_rx_len,
                               remote_data_rx,
                               remote_data_rx_len);
}

static void vpcm_cp_enable_all_ucodes(uint8_t mask[VPCM_CP_MASK_BYTES])
{
    memset(mask, 0xFF, VPCM_CP_MASK_BYTES);
}

static bool expect_equal(const char *label,
                         const uint8_t *expected,
                         const uint8_t *actual,
                         int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (expected[i] != actual[i]) {
            fprintf(stderr,
                    "%s mismatch at byte %d: expected 0x%02X got 0x%02X\n",
                    label, i, expected[i], actual[i]);
            return false;
        }
    }
    return true;
}

static uint8_t canonical_codeword(v91_law_t law, uint8_t codeword)
{
    return v91_linear_to_codeword(law, v91_codeword_to_linear(law, codeword));
}

static void vpcm_transport_linear(vpcm_channel_t channel,
                                  int16_t *dst,
                                  const int16_t *src,
                                  int len)
{
    int i;

    if (channel.mode != VPCM_PATH_ANALOG_G711) {
        fprintf(stderr, "linear transport requested on non-analog channel\n");
        abort();
    }

    for (i = 0; i < len; i++) {
        uint8_t codeword = v91_linear_to_codeword(channel.law, src[i]);
        dst[i] = v91_codeword_to_linear(channel.law, codeword);
    }
    vpcm_maybe_realtime_pace_samples(len);
}

static void vpcm_transport_codewords(vpcm_channel_t channel,
                                     uint8_t *dst,
                                     const uint8_t *src,
                                     int len)
{
    if (channel.mode != VPCM_PATH_PCM_G711) {
        fprintf(stderr, "codeword transport requested on non-PCM channel\n");
        abort();
    }

    memcpy(dst, src, (size_t) len);
    vpcm_maybe_realtime_pace_samples(len);
}

static void vpcm_transport_robbed_bit_codewords(uint8_t *dst,
                                                const uint8_t *src,
                                                int len)
{
    int i;

    memcpy(dst, src, (size_t) len);
    for (i = 5; i < len; i += 6)
        dst[i] &= 0xFE;
    vpcm_maybe_realtime_pace_samples(len);
}

static void vpcm_v8_result_handler(void *user_data, v8_parms_t *result)
{
    vpcm_v8_result_t *capture;

    capture = (vpcm_v8_result_t *) user_data;
    capture->seen = true;
    capture->result = *result;
}

static void init_v8_parms(v8_parms_t *parms,
                          bool calling_party,
                          int modulations,
                          int pcm_availability)
{
    int answer_tone;

    memset(parms, 0, sizeof(*parms));
    answer_tone = MODEM_CONNECT_TONES_ANSAM_PR;
    if (g_vpcm_transport_backend == VPCM_TRANSPORT_PJ_SIP) {
        /* On packetized SIP paths ANSam AM is frequently mangled/stripped.
           Prefer ANS/PR for robust in-call V.8 startup detection. */
        answer_tone = MODEM_CONNECT_TONES_ANS_PR;
    }
    parms->modem_connect_tone = calling_party ? MODEM_CONNECT_TONES_NONE
                                              : answer_tone;
    parms->send_ci = calling_party;
    parms->v92 = -1;
    parms->jm_cm.call_function = V8_CALL_V_SERIES;
    parms->jm_cm.modulations = modulations;
    parms->jm_cm.protocols = V8_PROTOCOL_NONE;
    parms->jm_cm.pstn_access = (pcm_availability != 0) ? V8_PSTN_ACCESS_DCE_ON_DIGITAL : 0;
    parms->jm_cm.pcm_modem_availability = pcm_availability;
    parms->jm_cm.nsf = -1;
    parms->jm_cm.t66 = -1;
}

static bool vpcm_v8_result_has_v91(const v8_parms_t *result)
{
    return result
        && (result->jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V91) != 0
        && (result->jm_cm.pstn_access & V8_PSTN_ACCESS_DCE_ON_DIGITAL) != 0;
}

static bool run_v8_exchange(v91_law_t law,
                            const v8_parms_t *caller_parms,
                            const v8_parms_t *answer_parms,
                            vpcm_v8_result_t *caller_result,
                            vpcm_v8_result_t *answer_result)
{
    vpcm_channel_t analog_channel;
    v8_state_t *caller;
    v8_state_t *answer;
    int chunk;

    analog_channel.law = law;
    analog_channel.mode = VPCM_PATH_ANALOG_G711;
    memset(caller_result, 0, sizeof(*caller_result));
    memset(answer_result, 0, sizeof(*answer_result));
    vpcm_log("Starting V.8 exchange over %s using %s",
             vpcm_path_mode_to_str(analog_channel.mode),
             vpcm_law_to_str(law));

    caller = v8_init(NULL, true, (v8_parms_t *) caller_parms, vpcm_v8_result_handler, caller_result);
    answer = v8_init(NULL, false, (v8_parms_t *) answer_parms, vpcm_v8_result_handler, answer_result);
    if (caller == NULL || answer == NULL) {
        if (caller)
            v8_free(caller);
        if (answer)
            v8_free(answer);
        fprintf(stderr, "failed to initialize V.8 states\n");
        return false;
    }

    for (chunk = 0; chunk < VPCM_V8_MAX_CHUNKS; chunk++) {
        int16_t caller_tx[VPCM_CHUNK_SAMPLES];
        int16_t answer_tx[VPCM_CHUNK_SAMPLES];
        int16_t caller_rx[VPCM_CHUNK_SAMPLES];
        int16_t answer_rx[VPCM_CHUNK_SAMPLES];

        v8_tx(caller, caller_tx, VPCM_CHUNK_SAMPLES);
        v8_tx(answer, answer_tx, VPCM_CHUNK_SAMPLES);
        vpcm_transport_linear(analog_channel, answer_rx, caller_tx, VPCM_CHUNK_SAMPLES);
        vpcm_transport_linear(analog_channel, caller_rx, answer_tx, VPCM_CHUNK_SAMPLES);
        v8_rx(caller, caller_rx, VPCM_CHUNK_SAMPLES);
        v8_rx(answer, answer_rx, VPCM_CHUNK_SAMPLES);
        if (chunk == 0 || ((chunk + 1) % 50) == 0) {
            vpcm_trace("V.8 exchange progress: chunk=%d caller_seen=%d answer_seen=%d",
                       chunk + 1, caller_result->seen, answer_result->seen);
        }

        if (caller_result->seen
            && answer_result->seen
            && caller_result->result.status == V8_STATUS_V8_CALL
            && answer_result->result.status == V8_STATUS_V8_CALL)
            break;
    }

    v8_free(caller);
    v8_free(answer);

    if (!caller_result->seen || !answer_result->seen) {
        fprintf(stderr, "V.8 exchange produced no final result\n");
        return false;
    }
    vpcm_log_v8_result("caller", &caller_result->result);
    vpcm_log_v8_result("answerer", &answer_result->result);
    if (caller_result->result.status != V8_STATUS_V8_CALL
        || answer_result->result.status != V8_STATUS_V8_CALL) {
        fprintf(stderr, "V.8 exchange failed: caller=%d answer=%d\n",
                caller_result->result.status, answer_result->result.status);
        return false;
    }

    return true;
}

static bool test_v8_v90_startup_over_analog_g711(v91_law_t law)
{
    v8_parms_t caller_parms;
    v8_parms_t answer_parms;
    vpcm_v8_result_t caller_result;
    vpcm_v8_result_t answer_result;
    uint32_t expected_mods;
    int expected_pcm;

    expected_mods = V8_MOD_V22 | V8_MOD_V34 | V8_MOD_V90;
    expected_pcm = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;
    vpcm_log("Test: V.8 V.90 startup over analog-over-G.711 (%s)", vpcm_law_to_str(law));
    init_v8_parms(&caller_parms, true, expected_mods, expected_pcm);
    init_v8_parms(&answer_parms, false, expected_mods, expected_pcm);

    if (!run_v8_exchange(law, &caller_parms, &answer_parms, &caller_result, &answer_result))
        return false;

    if (caller_result.result.jm_cm.call_function != V8_CALL_V_SERIES
        || answer_result.result.jm_cm.call_function != V8_CALL_V_SERIES) {
        fprintf(stderr, "V.8 V.90 startup call function mismatch\n");
        return false;
    }
    if ((caller_result.result.jm_cm.modulations & expected_mods) != expected_mods
        || (answer_result.result.jm_cm.modulations & expected_mods) != expected_mods) {
        fprintf(stderr, "V.8 V.90 startup modulation mismatch caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.modulations,
                answer_result.result.jm_cm.modulations);
        return false;
    }
    if ((caller_result.result.jm_cm.pcm_modem_availability & expected_pcm) != expected_pcm
        || (answer_result.result.jm_cm.pcm_modem_availability & expected_pcm) != expected_pcm) {
        fprintf(stderr, "V.8 V.90 startup PCM availability mismatch caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.pcm_modem_availability,
                answer_result.result.jm_cm.pcm_modem_availability);
        return false;
    }

    vpcm_log("PASS: V.8 V.90 startup over analog-over-G.711 (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_spandsp_v90_info_startup_over_analog_g711(v91_law_t law)
{
    enum { VPCM_V34_MAX_CHUNKS = 2500, VPCM_V34_V8_HANDOFF_CHUNKS = 4 };
    vpcm_channel_t analog_channel;
    v34_state_t *caller;
    v34_state_t *answerer;
    vpcm_v90_info0a_t analog_info0a;
    vpcm_v90_info1a_t analog_info1a;
    uint8_t info0a_bits[(VPCM_V90_INFO0A_BITS + 7) / 8];
    uint8_t info1a_bits[(VPCM_V90_INFO1A_BITS + 7) / 8];
    char info0a_str[VPCM_V90_INFO0A_BITS + 1];
    char info1a_str[VPCM_V90_INFO1A_BITS + 1];
    bool caller_saw_info0;
    bool answerer_saw_info0;
    bool caller_saw_info1;
    bool answerer_saw_info1;
    bool answerer_saw_uinfo;
    bool phase3_seen;
    int last_caller_rx_stage;
    int last_answerer_rx_stage;
    int last_caller_tx_stage;
    int last_answerer_tx_stage;
    int chunk;
    bool ok;

    analog_channel.law = law;
    analog_channel.mode = VPCM_PATH_ANALOG_G711;
    vpcm_log("Test: SpanDSP V.90 INFO startup over analog-over-G.711 (%s)", vpcm_law_to_str(law));

    vpcm_v90_info0a_init(&analog_info0a);
    vpcm_v90_info1a_init(&analog_info1a);
    if (!vpcm_v90_build_info0a_bits(info0a_bits, (int) sizeof(info0a_bits), &analog_info0a)
        || !vpcm_v90_build_info1a_bits(info1a_bits, (int) sizeof(info1a_bits), &analog_info1a)) {
        fprintf(stderr, "failed to build analogue-side V.90 INFO contract frames\n");
        return false;
    }
    if (g_vpcm_session_diag) {
        vpcm_packed_bits_to_str(info0a_bits, VPCM_V90_INFO0A_BITS, info0a_str, sizeof(info0a_str));
        vpcm_packed_bits_to_str(info1a_bits, VPCM_V90_INFO1A_BITS, info1a_str, sizeof(info1a_str));
        vpcm_log("Analogue-side INFO0a contract bits: %s", info0a_str);
        vpcm_log("Analogue-side INFO1a contract bits: %s", info1a_str);
        vpcm_log("Analogue-side INFO1a contract fields: md=%u u_info=%u up_rate_code=%u down_rate_code=%u freq_offset=%d",
                 analog_info1a.md,
                 analog_info1a.u_info,
                 analog_info1a.upstream_symbol_rate_code,
                 analog_info1a.downstream_rate_code,
                 analog_info1a.freq_offset);
    }

    caller = v34_init(NULL, 3200, 21600, true, true,
                      vpcm_v34_dummy_get_bit, NULL,
                      vpcm_v34_dummy_put_bit, NULL);
    answerer = v34_init(NULL, 3200, 21600, false, true,
                        vpcm_v34_dummy_get_bit, NULL,
                        vpcm_v34_dummy_put_bit, NULL);
    if (caller == NULL || answerer == NULL)
    {
        if (caller)
            v34_free(caller);
        if (answerer)
            v34_free(answerer);
        fprintf(stderr, "failed to initialize SpanDSP V.34/V.90 states\n");
        return false;
    }

    /*
     * Enable V.90 mode on both sides:
     *   - Caller  = analog modem: sends INFO0a (49 bits), receives INFO0d (62 bits),
     *     sends INFO1a (70 bits with U_INFO), receives INFO1d (109 bits).
     *   - Answerer = digital modem: sends INFO0d (62 bits), receives INFO0a,
     *     sends INFO1d (109 bits), receives INFO1a.
     *
     * V.8 CJ silence must be supplied before Phase 2. Recreate that here.
     */
    for (chunk = 0; chunk < VPCM_V34_V8_HANDOFF_CHUNKS; chunk++)
    {
        int16_t caller_tx[VPCM_CHUNK_SAMPLES];
        int16_t answer_tx[VPCM_CHUNK_SAMPLES];
        int16_t caller_rx[VPCM_CHUNK_SAMPLES];
        int16_t answer_rx[VPCM_CHUNK_SAMPLES];

        if (v34_tx(caller, caller_tx, VPCM_CHUNK_SAMPLES) != VPCM_CHUNK_SAMPLES
            || v34_tx(answerer, answer_tx, VPCM_CHUNK_SAMPLES) != VPCM_CHUNK_SAMPLES)
        {
            fprintf(stderr, "SpanDSP V.90 handoff silence generation failed\n");
            v34_free(caller);
            v34_free(answerer);
            return false;
        }
        vpcm_transport_linear(analog_channel, answer_rx, caller_tx, VPCM_CHUNK_SAMPLES);
        vpcm_transport_linear(analog_channel, caller_rx, answer_tx, VPCM_CHUNK_SAMPLES);
        if (v34_rx(caller, caller_rx, VPCM_CHUNK_SAMPLES) != 0
            || v34_rx(answerer, answer_rx, VPCM_CHUNK_SAMPLES) != 0)
        {
            fprintf(stderr, "SpanDSP V.90 handoff silence RX left unprocessed samples\n");
            v34_free(caller);
            v34_free(answerer);
            return false;
        }
    }

    v34_set_v90_mode(caller, law == V91_LAW_ALAW ? 1 : 0);
    v34_set_v90_mode(answerer, law == V91_LAW_ALAW ? 1 : 0);
    if (g_vpcm_session_diag)
    {
        span_log_set_level(v34_get_logging_state(caller), SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL | SPAN_LOG_FLOW);
        span_log_set_level(v34_get_logging_state(answerer), SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL | SPAN_LOG_FLOW);
    }

    caller_saw_info0 = false;
    answerer_saw_info0 = false;
    caller_saw_info1 = false;
    answerer_saw_info1 = false;
    answerer_saw_uinfo = false;
    phase3_seen = false;
    last_caller_rx_stage = -1;
    last_answerer_rx_stage = -1;
    last_caller_tx_stage = -1;
    last_answerer_tx_stage = -1;
    ok = false;

    for (chunk = 0; chunk < VPCM_V34_MAX_CHUNKS; chunk++)
    {
        int16_t caller_tx[VPCM_CHUNK_SAMPLES];
        int16_t answer_tx[VPCM_CHUNK_SAMPLES];
        int16_t caller_rx[VPCM_CHUNK_SAMPLES];
        int16_t answer_rx[VPCM_CHUNK_SAMPLES];
        int caller_event;
        int answerer_event;
        int caller_rx_stage;
        int answerer_rx_stage;
        int caller_tx_stage;
        int answerer_tx_stage;

        if (v34_tx(caller, caller_tx, VPCM_CHUNK_SAMPLES) != VPCM_CHUNK_SAMPLES
            || v34_tx(answerer, answer_tx, VPCM_CHUNK_SAMPLES) != VPCM_CHUNK_SAMPLES)
        {
            fprintf(stderr, "SpanDSP V.90 INFO startup TX chunk generation failed\n");
            break;
        }

        vpcm_transport_linear(analog_channel, answer_rx, caller_tx, VPCM_CHUNK_SAMPLES);
        vpcm_transport_linear(analog_channel, caller_rx, answer_tx, VPCM_CHUNK_SAMPLES);

        if (v34_rx(caller, caller_rx, VPCM_CHUNK_SAMPLES) != 0
            || v34_rx(answerer, answer_rx, VPCM_CHUNK_SAMPLES) != 0)
        {
            fprintf(stderr, "SpanDSP V.90 INFO startup RX left unprocessed samples\n");
            break;
        }

        caller_event = v34_get_rx_event(caller);
        answerer_event = v34_get_rx_event(answerer);
        caller_rx_stage = v34_get_rx_stage(caller);
        answerer_rx_stage = v34_get_rx_stage(answerer);
        caller_tx_stage = v34_get_tx_stage(caller);
        answerer_tx_stage = v34_get_tx_stage(answerer);

        caller_saw_info0 |= (caller_event == V34_EVENT_INFO0_OK);
        answerer_saw_info0 |= (answerer_event == V34_EVENT_INFO0_OK);
        caller_saw_info1 |= (caller_event == V34_EVENT_INFO1_OK);
        answerer_saw_info1 |= (answerer_event == V34_EVENT_INFO1_OK);
        answerer_saw_uinfo |= (v34_get_v90_u_info(answerer) > 0);
        phase3_seen |= (caller_rx_stage >= V34_RX_STAGE_PHASE3_TRAINING)
                    || (answerer_rx_stage >= V34_RX_STAGE_PHASE3_TRAINING)
                    || v34_get_primary_channel_active(caller)
                    || v34_get_primary_channel_active(answerer);

        if (g_vpcm_session_diag
            && (caller_rx_stage != last_caller_rx_stage
                || answerer_rx_stage != last_answerer_rx_stage
                || caller_tx_stage != last_caller_tx_stage
                || answerer_tx_stage != last_answerer_tx_stage))
        {
            vpcm_log("SpanDSP V.90 INFO trace: caller tx=%s rx=%s ev=%d | answerer tx=%s rx=%s ev=%d",
                     vpcm_v34_tx_stage_to_str(caller_tx_stage),
                     vpcm_v34_rx_stage_to_str(caller_rx_stage),
                     caller_event,
                     vpcm_v34_tx_stage_to_str(answerer_tx_stage),
                     vpcm_v34_rx_stage_to_str(answerer_rx_stage),
                     answerer_event);
        }
        last_caller_rx_stage = caller_rx_stage;
        last_answerer_rx_stage = answerer_rx_stage;
        last_caller_tx_stage = caller_tx_stage;
        last_answerer_tx_stage = answerer_tx_stage;

        if (answerer_saw_info0
            && answerer_saw_info1
            && answerer_saw_uinfo
            && phase3_seen)
        {
            ok = true;
            break;
        }

        if (caller_event == V34_EVENT_TRAINING_FAILED
            || answerer_event == V34_EVENT_TRAINING_FAILED)
            break;
    }

    if (!ok)
    {
        fprintf(stderr,
                "SpanDSP V.90 INFO startup failed: caller_info0=%d answerer_info0=%d caller_info1=%d answerer_info1=%d u_info=%d phase3=%d caller_stage=%s/%s answerer_stage=%s/%s (note: caller is analogue-side contract, answerer is SpanDSP V.90 digital side)\n",
                caller_saw_info0 ? 1 : 0,
                answerer_saw_info0 ? 1 : 0,
                caller_saw_info1 ? 1 : 0,
                answerer_saw_info1 ? 1 : 0,
                answerer_saw_uinfo ? 1 : 0,
                phase3_seen ? 1 : 0,
                vpcm_v34_tx_stage_to_str(v34_get_tx_stage(caller)),
                vpcm_v34_rx_stage_to_str(v34_get_rx_stage(caller)),
                vpcm_v34_tx_stage_to_str(v34_get_tx_stage(answerer)),
                vpcm_v34_rx_stage_to_str(v34_get_rx_stage(answerer)));
    }

    v34_free(caller);
    v34_free(answerer);

    if (!ok)
        return false;

    vpcm_log("PASS: SpanDSP V.90 INFO startup over analog-over-G.711 (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v8_v91_advertisement_over_analog_g711(v91_law_t law)
{
    v8_parms_t caller_parms;
    v8_parms_t answer_parms;
    vpcm_v8_result_t caller_result;
    vpcm_v8_result_t answer_result;
    uint32_t expected_mods;
    int expected_pcm;

    expected_mods = V8_MOD_V22 | V8_MOD_V34;
    expected_pcm = V8_PSTN_PCM_MODEM_V91;
    vpcm_log("Test: V.8 V.91 advertisement over analog-over-G.711 (%s)", vpcm_law_to_str(law));
    init_v8_parms(&caller_parms, true, expected_mods, expected_pcm);
    init_v8_parms(&answer_parms, false, expected_mods, expected_pcm);

    if (!run_v8_exchange(law, &caller_parms, &answer_parms, &caller_result, &answer_result))
        return false;

    if ((caller_result.result.jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V91) == 0
        || (answer_result.result.jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V91) == 0) {
        fprintf(stderr, "V.8 V.91 advertisement missing caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.pcm_modem_availability,
                answer_result.result.jm_cm.pcm_modem_availability);
        return false;
    }

    vpcm_log("PASS: V.8 V.91 advertisement over analog-over-G.711 (%s)", vpcm_law_to_str(law));
    return true;
}

static bool __attribute__((unused)) test_v8_v92_qc_exchange_over_analog_g711(v91_law_t law)
{
    v8_parms_t caller_parms;
    v8_parms_t answer_parms;
    vpcm_v8_result_t caller_result;
    vpcm_v8_result_t answer_result;
    uint32_t expected_mods;
    int expected_pcm;
    const int caller_v92 = 0x45;
    const int answer_v92 = 0x46;

    /*
     * Current SpanDSP V.8 behavior in this path keeps the CM/JM modulation
     * set on the V.90 family advertisement while exchanging the separate
     * V.92 QC/QCA control byte.  So this test verifies the actual V.92-
     * specific path we have today: byte exchange over analog-over-G.711.
     */
    expected_mods = V8_MOD_V22 | V8_MOD_V34 | V8_MOD_V90;
    expected_pcm = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;
    vpcm_log("Test: V.8 V.92 QC/QCA exchange over analog-over-G.711 (%s)", vpcm_law_to_str(law));
    init_v8_parms(&caller_parms, true, expected_mods, expected_pcm);
    init_v8_parms(&answer_parms, false, expected_mods, expected_pcm);
    caller_parms.v92 = caller_v92;
    answer_parms.v92 = answer_v92;

    if (!run_v8_exchange(law, &caller_parms, &answer_parms, &caller_result, &answer_result))
        return false;

    if ((caller_result.result.jm_cm.modulations & expected_mods) != expected_mods
        || (answer_result.result.jm_cm.modulations & expected_mods) != expected_mods) {
        fprintf(stderr, "V.8 V.92 QC exchange base modulation mismatch caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.modulations,
                answer_result.result.jm_cm.modulations);
        return false;
    }
    if ((caller_result.result.jm_cm.pcm_modem_availability & expected_pcm) != expected_pcm
        || (answer_result.result.jm_cm.pcm_modem_availability & expected_pcm) != expected_pcm) {
        fprintf(stderr, "V.8 V.92 QC exchange PCM availability mismatch caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.pcm_modem_availability,
                answer_result.result.jm_cm.pcm_modem_availability);
        return false;
    }
    if (caller_result.result.v92 != answer_v92 || answer_result.result.v92 != caller_v92) {
        fprintf(stderr, "V.8 V.92 QC exchange control byte mismatch caller_rx=0x%X answer_rx=0x%X expected caller=0x%X answer=0x%X\n",
                caller_result.result.v92,
                answer_result.result.v92,
                answer_v92,
                caller_v92);
        return false;
    }

    vpcm_log("PASS: V.8 V.92 QC/QCA exchange over analog-over-G.711 (%s), caller_rx=0x%02X answer_rx=0x%02X",
             vpcm_law_to_str(law),
             caller_result.result.v92,
             answer_result.result.v92);
    return true;
}

static bool test_v92_dil_rate_adaptation(void)
{
    v91_dil_desc_t clean_dil;
    v91_dil_desc_t echo_dil;
    v91_dil_desc_t robbed_bit_dil;
    v91_dil_analysis_t clean_analysis;
    v91_dil_analysis_t echo_analysis;
    v91_dil_analysis_t robbed_bit_analysis;
    char clean_down_rate[64];
    char clean_up_rate[64];
    char echo_down_rate[64];
    char echo_up_rate[64];
    char robbed_rate[64];

    vpcm_log("Test: V.92 DIL-driven rate adaptation");

    vpcm_v92_init_digital_dil_from_ja(&clean_dil, false);
    vpcm_v92_init_digital_dil_from_ja(&echo_dil, true);
    vpcm_init_robbed_bit_dil_profile(&robbed_bit_dil);
    if (!v91_analyse_dil_descriptor(&clean_dil, &clean_analysis)
        || !v91_analyse_dil_descriptor(&echo_dil, &echo_analysis)
        || !v91_analyse_dil_descriptor(&robbed_bit_dil, &robbed_bit_analysis)) {
        fprintf(stderr, "V.92 DIL analysis failed\n");
        return false;
    }
    if (clean_analysis.recommended_downstream_drn <= echo_analysis.recommended_downstream_drn
        || clean_analysis.recommended_upstream_drn <= echo_analysis.recommended_upstream_drn) {
        fprintf(stderr, "V.92 DIL analysis did not reduce rate request for impaired line\n");
        return false;
    }
    if (!echo_analysis.echo_limited) {
        fprintf(stderr, "V.92 impaired DIL did not mark echo-limited\n");
        return false;
    }
    if (!robbed_bit_analysis.robbed_bit_limited
        || robbed_bit_analysis.recommended_downstream_drn != vpcm_cp_recommended_robbed_bit_drn()) {
        fprintf(stderr, "V.92 robbed-bit DIL profile did not select the safe ceiling\n");
        return false;
    }

    vpcm_format_rate(clean_down_rate, sizeof(clean_down_rate), clean_analysis.recommended_downstream_drn);
    vpcm_format_rate(clean_up_rate, sizeof(clean_up_rate), clean_analysis.recommended_upstream_drn);
    vpcm_format_rate(echo_down_rate, sizeof(echo_down_rate), echo_analysis.recommended_downstream_drn);
    vpcm_format_rate(echo_up_rate, sizeof(echo_up_rate), echo_analysis.recommended_upstream_drn);
    vpcm_format_rate(robbed_rate, sizeof(robbed_rate), robbed_bit_analysis.recommended_downstream_drn);
    vpcm_log("PASS: V.92 DIL-driven rate adaptation (clean=%s/%s, impaired=%s/%s, robbed-bit=%s)",
             clean_down_rate,
             clean_up_rate,
             echo_down_rate,
             echo_up_rate,
             robbed_rate);
    return true;
}

static bool test_v90_v92_startup_contract_path(v91_law_t law)
{
    v8_parms_t caller_parms;
    v8_parms_t answer_parms;
    vpcm_v8_result_t caller_result;
    vpcm_v8_result_t answer_result;

    vpcm_log("Test: V.90/V.92 startup contract path (%s)", vpcm_law_to_str(law));

    init_v8_parms(&caller_parms, true, V8_MOD_V22 | V8_MOD_V34 | V8_MOD_V90, V8_PSTN_PCM_MODEM_V90_V92_DIGITAL);
    init_v8_parms(&answer_parms, false, V8_MOD_V22 | V8_MOD_V34 | V8_MOD_V90, V8_PSTN_PCM_MODEM_V90_V92_DIGITAL);

    vpcm_log_session_sequence_header("Digital", "Analogue");
    vpcm_log_session_sequence_row("V.8 TX", "waiting for V.8", "waiting for V.8", "V.8 RX/TX");

    if (!run_v8_exchange(law, &caller_parms, &answer_parms, &caller_result, &answer_result))
        return false;
    if ((caller_result.result.jm_cm.modulations & V8_MOD_V90) == 0
        || (answer_result.result.jm_cm.modulations & V8_MOD_V90) == 0) {
        fprintf(stderr, "V.90/V.92 startup contract path V.8 modulation mismatch caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.modulations,
                answer_result.result.jm_cm.modulations);
        return false;
    }
    vpcm_log_session_sequence_row("V.8 RX/ok", "waiting for Ez", "waiting for Ez", "V.8 RX/ok");
    vpcm_log_session_sequence_footer();

    return run_v90_v92_startup_contract_session(law,
                                                "clean line",
                                                false,
                                                0x92000000U ^ ((uint32_t) law << 8))
        && run_v90_v92_startup_contract_session(law,
                                                "startup echo estimate",
                                                true,
                                                0x92100000U ^ ((uint32_t) law << 8));
}

static bool test_v91_codeword_loopback(v91_law_t law)
{
    v91_state_t tx;
    v91_state_t rx;
    uint8_t input[TEST_PAYLOAD_LEN];
    uint8_t g711[TEST_CHUNK_MAX];
    uint8_t output[TEST_PAYLOAD_LEN];
    vpcm_channel_t pcm_channel;
    int in_pos;
    int out_pos;
    int chunk_seed;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_init(&rx, law, V91_MODE_TRANSPARENT);
    pcm_channel.law = law;
    pcm_channel.mode = VPCM_PATH_PCM_G711;
    vpcm_log("Test: V.91 codeword loopback over %s (%s)",
             vpcm_path_mode_to_str(pcm_channel.mode), vpcm_law_to_str(law));
    fill_pattern(input, TEST_PAYLOAD_LEN, 0x12345678U ^ (uint32_t) law);

    in_pos = 0;
    out_pos = 0;
    chunk_seed = 17;
    while (in_pos < TEST_PAYLOAD_LEN) {
        int want;
        int produced;
        int consumed;

        want = 1 + (chunk_seed % TEST_CHUNK_MAX);
        if (want > (TEST_PAYLOAD_LEN - in_pos))
            want = TEST_PAYLOAD_LEN - in_pos;
        chunk_seed = (chunk_seed * 73 + 19) % 997;

        produced = v91_tx_codewords(&tx, g711, TEST_CHUNK_MAX, input + in_pos, want);
        vpcm_transport_codewords(pcm_channel, g711, g711, produced);
        consumed = v91_rx_codewords(&rx, output + out_pos, TEST_PAYLOAD_LEN - out_pos, g711, produced);
        if (chunk_seed == 17 || ((in_pos / TEST_CHUNK_MAX) % 16) == 0) {
            vpcm_trace("V.91 codeword loopback progress: in=%d out=%d chunk=%d",
                       in_pos, out_pos, want);
        }
        if (produced != want || consumed != want) {
            fprintf(stderr, "V.91 codeword loopback length mismatch: tx=%d rx=%d want=%d\n",
                    produced, consumed, want);
            return false;
        }

        in_pos += want;
        out_pos += consumed;
    }

    vpcm_log("PASS: V.91 codeword loopback over %s (%s), payload=%d bytes",
             vpcm_path_mode_to_str(pcm_channel.mode), vpcm_law_to_str(law), TEST_PAYLOAD_LEN);
    return expect_equal("V.91 codeword loopback", input, output, TEST_PAYLOAD_LEN);
}

static bool test_v91_startup_primitives(v91_law_t law)
{
    v91_state_t tx;
    v91_state_t rx;
    uint8_t silence[V91_PHASE1_SILENCE_SYMBOLS];
    uint8_t ez[V91_EZ_SYMBOLS];
    uint8_t expected_silence;
    uint8_t expected_ez;
    v91_info_frame_t info_tx;
    uint32_t random_state;
    int i;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_init(&rx, law, V91_MODE_TRANSPARENT);

    vpcm_log("Test: V.91 startup primitives over raw-G.711 (%s)", vpcm_law_to_str(law));

    if (v91_tx_phase1_silence_codewords(&tx, silence, (int) sizeof(silence)) != V91_PHASE1_SILENCE_SYMBOLS) {
        fprintf(stderr, "V.91 phase1 silence length mismatch\n");
        return false;
    }
    expected_silence = v91_ucode_to_codeword(law, 0, true);
    for (i = 0; i < V91_PHASE1_SILENCE_SYMBOLS; i++) {
        if (silence[i] != expected_silence) {
            fprintf(stderr, "V.91 phase1 silence mismatch at %d: %02X != %02X\n",
                    i, silence[i], expected_silence);
            return false;
        }
    }

    if (v91_tx_ez_codewords(&tx, ez, (int) sizeof(ez)) != V91_EZ_SYMBOLS) {
        fprintf(stderr, "V.91 Ez length mismatch\n");
        return false;
    }
    expected_ez = v91_ucode_to_codeword(law, 66, false);
    for (i = 0; i < V91_EZ_SYMBOLS; i++) {
        if (ez[i] != expected_ez) {
            fprintf(stderr, "V.91 Ez mismatch at %d: %02X != %02X\n",
                    i, ez[i], expected_ez);
            return false;
        }
    }

    memset(&info_tx, 0, sizeof(info_tx));
    info_tx.reserved_12_25 = 0x0000;
    info_tx.request_default_dil = true;
    info_tx.request_control_channel = false;
    info_tx.acknowledge_info_frame = false;
    info_tx.reserved_29_32 = 0x0;
    info_tx.max_tx_power = 0;
    info_tx.power_measured_after_digital_impairments = true;
    info_tx.tx_uses_alaw = (law == V91_LAW_ALAW);
    info_tx.request_transparent_mode = true;
    info_tx.cleardown_if_transparent_denied = true;

    if (!run_v91_info_roundtrip_case(&tx, &rx, "INFO", &info_tx, false))
        return false;

    info_tx.acknowledge_info_frame = true;
    if (!run_v91_info_roundtrip_case(&tx, &rx, "INFO'", &info_tx, true))
        return false;

    random_state = 0x91C0DE00U ^ (uint32_t) law;
    memset(&info_tx, 0, sizeof(info_tx));
    info_tx.reserved_12_25 = (uint16_t) (prng_next(&random_state) & 0x3FFFU);
    info_tx.request_default_dil = false;
    info_tx.request_control_channel = true;
    info_tx.acknowledge_info_frame = false;
    info_tx.reserved_29_32 = (uint8_t) (prng_next(&random_state) & 0x0FU);
    info_tx.max_tx_power = (uint8_t) (1 + (prng_next(&random_state) % 31U));
    info_tx.power_measured_after_digital_impairments = false;
    info_tx.tx_uses_alaw = (law == V91_LAW_ALAW);
    info_tx.request_transparent_mode = false;
    info_tx.cleardown_if_transparent_denied = false;
    if (!run_v91_info_roundtrip_case(&tx, &rx, "INFO variant (non-default/random power)", &info_tx, false))
        return false;

    vpcm_log("PASS: V.91 startup primitives over raw-G.711 (%s)", vpcm_law_to_str(law));
    return true;
}

static bool test_v91_linear_loopback(v91_law_t law)
{
    v91_state_t tx;
    v91_state_t rx;
    uint8_t input[TEST_PAYLOAD_LEN];
    uint8_t canonical[TEST_PAYLOAD_LEN];
    int16_t linear[TEST_CHUNK_MAX];
    uint8_t output[TEST_PAYLOAD_LEN];
    int in_pos;
    int out_pos;
    int chunk_seed;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_init(&rx, law, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 linear simulation loopback over analog-over-G.711 (%s)",
             vpcm_law_to_str(law));
    fill_pattern(input, TEST_PAYLOAD_LEN, 0xA5A55A5AU ^ (uint32_t) law);
    for (in_pos = 0; in_pos < TEST_PAYLOAD_LEN; in_pos++)
        canonical[in_pos] = canonical_codeword(law, input[in_pos]);

    in_pos = 0;
    out_pos = 0;
    chunk_seed = 29;
    while (in_pos < TEST_PAYLOAD_LEN) {
        int want;
        int produced;
        int consumed;

        want = 1 + (chunk_seed % TEST_CHUNK_MAX);
        if (want > (TEST_PAYLOAD_LEN - in_pos))
            want = TEST_PAYLOAD_LEN - in_pos;
        chunk_seed = (chunk_seed * 61 + 7) % 1009;

        produced = v91_tx_linear(&tx, linear, TEST_CHUNK_MAX, input + in_pos, want);
        consumed = v91_rx_linear(&rx, output + out_pos, TEST_PAYLOAD_LEN - out_pos, linear, produced);
        if (chunk_seed == 29 || ((in_pos / TEST_CHUNK_MAX) % 16) == 0) {
            vpcm_trace("V.91 linear loopback progress: in=%d out=%d chunk=%d",
                       in_pos, out_pos, want);
        }
        if (produced != want || consumed != want) {
            fprintf(stderr, "V.91 linear loopback length mismatch: tx=%d rx=%d want=%d\n",
                    produced, consumed, want);
            return false;
        }

        in_pos += want;
        out_pos += consumed;
    }

    vpcm_log("PASS: V.91 linear simulation loopback over analog-over-G.711 (%s), payload=%d bytes",
             vpcm_law_to_str(law), TEST_PAYLOAD_LEN);
    return expect_equal("V.91 linear loopback", canonical, output, TEST_PAYLOAD_LEN);
}

static bool test_v91_full_duplex(v91_law_t law)
{
    v91_state_t a_tx;
    v91_state_t a_rx;
    v91_state_t b_tx;
    v91_state_t b_rx;
    uint8_t a_input[TEST_PAYLOAD_LEN];
    uint8_t b_input[TEST_PAYLOAD_LEN];
    uint8_t a_output[TEST_PAYLOAD_LEN];
    uint8_t b_output[TEST_PAYLOAD_LEN];
    uint8_t a_to_b[160];
    uint8_t b_to_a[160];
    int a_in_pos;
    int b_in_pos;
    int a_out_pos;
    int b_out_pos;
    int stride;

    v91_init(&a_tx, law, V91_MODE_TRANSPARENT);
    v91_init(&a_rx, law, V91_MODE_TRANSPARENT);
    v91_init(&b_tx, law, V91_MODE_TRANSPARENT);
    v91_init(&b_rx, law, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 full duplex loopback over raw-G.711 (%s)", vpcm_law_to_str(law));

    fill_pattern(a_input, TEST_PAYLOAD_LEN, 0xCAFEBABEU ^ (uint32_t) law);
    fill_pattern(b_input, TEST_PAYLOAD_LEN, 0x0BADF00DU ^ (uint32_t) law);

    a_in_pos = 0;
    b_in_pos = 0;
    a_out_pos = 0;
    b_out_pos = 0;
    stride = 97;

    while (a_in_pos < TEST_PAYLOAD_LEN || b_in_pos < TEST_PAYLOAD_LEN) {
        int a_want;
        int b_want;
        int a_prod;
        int b_prod;
        int a_cons;
        int b_cons;

        a_want = (a_in_pos < TEST_PAYLOAD_LEN) ? stride : 0;
        b_want = (b_in_pos < TEST_PAYLOAD_LEN) ? (stride + 23) : 0;
        if (a_want > (int) sizeof(a_to_b))
            a_want = (int) sizeof(a_to_b);
        if (b_want > (int) sizeof(b_to_a))
            b_want = (int) sizeof(b_to_a);
        if (a_want > (TEST_PAYLOAD_LEN - a_in_pos))
            a_want = TEST_PAYLOAD_LEN - a_in_pos;
        if (b_want > (TEST_PAYLOAD_LEN - b_in_pos))
            b_want = TEST_PAYLOAD_LEN - b_in_pos;

        a_prod = v91_tx_codewords(&a_tx, a_to_b, 160, a_input + a_in_pos, a_want);
        b_prod = v91_tx_codewords(&b_tx, b_to_a, 160, b_input + b_in_pos, b_want);
        a_cons = v91_rx_codewords(&a_rx, a_output + a_out_pos, TEST_PAYLOAD_LEN - a_out_pos, b_to_a, b_prod);
        b_cons = v91_rx_codewords(&b_rx, b_output + b_out_pos, TEST_PAYLOAD_LEN - b_out_pos, a_to_b, a_prod);
        if (stride == 97 || ((a_in_pos / 512) != ((a_in_pos + a_want) / 512))) {
            vpcm_trace("V.91 full duplex progress: a_in=%d b_in=%d a_out=%d b_out=%d",
                       a_in_pos, b_in_pos, a_out_pos, b_out_pos);
        }

        if (a_cons != b_prod || b_cons != a_prod) {
            fprintf(stderr,
                    "V.91 full duplex length mismatch: A tx=%d B rx=%d, B tx=%d A rx=%d\n",
                    a_prod, b_cons, b_prod, a_cons);
            return false;
        }

        a_in_pos += a_want;
        b_in_pos += b_want;
        a_out_pos += a_cons;
        b_out_pos += b_cons;
        stride = (stride + 37) % 149 + 1;
    }

    vpcm_log("PASS: V.91 full duplex loopback over raw-G.711 (%s), payload=%d bytes each way",
             vpcm_law_to_str(law), TEST_PAYLOAD_LEN);
    return expect_equal("V.91 full duplex A<-B", b_input, a_output, TEST_PAYLOAD_LEN)
        && expect_equal("V.91 full duplex B<-A", a_input, b_output, TEST_PAYLOAD_LEN);
}

static bool run_vpcm_session_suite(void)
{
    return test_v92_dil_rate_adaptation()
        && test_v8_v90_startup_over_analog_g711(V91_LAW_ULAW)
        && test_v8_v90_startup_over_analog_g711(V91_LAW_ALAW)
        && (!g_vpcm_experimental_v90_info
            || (test_spandsp_v90_info_startup_over_analog_g711(V91_LAW_ULAW)
                && test_spandsp_v90_info_startup_over_analog_g711(V91_LAW_ALAW)))
        && test_v8_v91_advertisement_over_analog_g711(V91_LAW_ULAW)
        && test_v8_v91_advertisement_over_analog_g711(V91_LAW_ALAW)
        && test_v91_startup_to_data_robbed_bit(V91_LAW_ULAW)
        && test_v91_startup_to_data_robbed_bit(V91_LAW_ALAW)
        && test_v91_startup_to_data_robbed_bit_safe_rate(V91_LAW_ULAW)
        && test_v91_startup_to_data_robbed_bit_safe_rate(V91_LAW_ALAW)
        && test_v90_v92_startup_contract_path(V91_LAW_ULAW)
        && test_v90_v92_startup_contract_path(V91_LAW_ALAW);
}

static bool run_vpcm_primitive_suite(void)
{
    return test_vpcm_cp_robbed_bit_safe_profile()
        && test_v91_codeword_loopback(V91_LAW_ULAW)
        && test_v91_codeword_loopback(V91_LAW_ALAW)
        && test_v91_startup_primitives(V91_LAW_ULAW)
        && test_v91_startup_primitives(V91_LAW_ALAW)
        && test_v91_default_dil(V91_LAW_ULAW)
        && test_v91_default_dil(V91_LAW_ALAW)
        && test_v91_eu_and_frame_alignment(V91_LAW_ULAW)
        && test_v91_eu_and_frame_alignment(V91_LAW_ALAW)
        && test_v91_phil_and_scr_sequences(V91_LAW_ULAW)
        && test_v91_phil_and_scr_sequences(V91_LAW_ALAW)
        && test_v91_cp_exchange(V91_LAW_ULAW)
        && test_v91_cp_exchange(V91_LAW_ALAW)
        && test_v91_startup_to_b1_multirate(V91_LAW_ULAW)
        && test_v91_startup_to_b1_multirate(V91_LAW_ALAW)
        && test_v91_mapped_data_multirate(V91_LAW_ULAW)
        && test_v91_mapped_data_multirate(V91_LAW_ALAW)
        && test_v91_robbed_bit_signalling(V91_LAW_ULAW)
        && test_v91_robbed_bit_signalling(V91_LAW_ALAW)
        && test_v91_eu_startup_sequence(V91_LAW_ULAW)
        && test_v91_eu_startup_sequence(V91_LAW_ALAW)
        && test_v91_em_and_startup_sequence(V91_LAW_ULAW)
        && test_v91_em_and_startup_sequence(V91_LAW_ALAW)
        && test_v91_bilateral_info_tracking(V91_LAW_ALAW, V91_LAW_ULAW)
        && test_v91_linear_loopback(V91_LAW_ULAW)
        && test_v91_linear_loopback(V91_LAW_ALAW)
        && test_v91_full_duplex(V91_LAW_ULAW)
        && test_v91_full_duplex(V91_LAW_ALAW);
}

static int vpcm_decode_rx_codewords(v91_state_t *rx_state,
                                    const uint8_t *codewords,
                                    int codeword_len,
                                    uint8_t *rx_bytes,
                                    int rx_cap,
                                    int *rx_len,
                                    int *rx_codewords,
                                    int max_codewords_per_pull)
{
    uint8_t decoded[512];
    int cw_len;
    int decoded_cap;
    int consumed;

    if (!rx_state || !codewords || !rx_bytes || !rx_len || !rx_codewords)
        return 0;
    if (codeword_len <= 0 || *rx_len >= rx_cap)
        return 0;
    cw_len = codeword_len;
    if (max_codewords_per_pull > 0 && cw_len > max_codewords_per_pull)
        cw_len = max_codewords_per_pull;
    cw_len -= (cw_len % VPCM_CP_FRAME_INTERVALS);
    if (cw_len <= 0)
        return 0;
    decoded_cap = (int) sizeof(decoded);
    if (decoded_cap > (rx_cap - *rx_len))
        decoded_cap = rx_cap - *rx_len;
    if (decoded_cap <= 0)
        return 0;
    consumed = v91_rx_codewords(rx_state, decoded, decoded_cap, codewords, cw_len);
    if (consumed > 0) {
        memcpy(rx_bytes + *rx_len, decoded, (size_t) consumed);
        *rx_len += consumed;
    }
    *rx_codewords += cw_len;
    return consumed;
}

static void vpcm_g711_payload_to_linear(const uint8_t *payload,
                                        int payload_len,
                                        v91_law_t law,
                                        int16_t *linear)
{
    int i;

    if (!payload || !linear || payload_len <= 0)
        return;
    for (i = 0; i < payload_len; i++) {
        if (law == V91_LAW_ALAW)
            linear[i] = alaw_to_linear(payload[i]);
        else
            linear[i] = ulaw_to_linear(payload[i]);
    }
}

static void vpcm_linear_to_g711_payload(const int16_t *linear,
                                        int linear_len,
                                        v91_law_t law,
                                        uint8_t *payload)
{
    int i;

    if (!linear || !payload || linear_len <= 0)
        return;
    for (i = 0; i < linear_len; i++) {
        if (law == V91_LAW_ALAW)
            payload[i] = linear_to_alaw(linear[i]);
        else
            payload[i] = linear_to_ulaw(linear[i]);
    }
}

static void vpcm_pjsip_modem_reset(void)
{
    memset(&g_vpcm_pjsip_modem, 0, sizeof(g_vpcm_pjsip_modem));
}

static void vpcm_pjsip_modem_cleanup(void)
{
    if (g_vpcm_pjsip_modem.v8_state)
        v8_free(g_vpcm_pjsip_modem.v8_state);
    g_vpcm_pjsip_modem.v8_state = NULL;
    free(g_vpcm_pjsip_modem.tx_data);
    free(g_vpcm_pjsip_modem.expected_rx);
    free(g_vpcm_pjsip_modem.rx_data);
    free(g_vpcm_pjsip_modem.tx_codeword_stream);
    free(g_vpcm_pjsip_modem.startup_local_stream);
    free(g_vpcm_pjsip_modem.startup_remote_stream);
    free(g_vpcm_pjsip_modem.startup_remote_rx_stream);
    g_vpcm_pjsip_modem.tx_data = NULL;
    g_vpcm_pjsip_modem.expected_rx = NULL;
    g_vpcm_pjsip_modem.rx_data = NULL;
    g_vpcm_pjsip_modem.tx_codeword_stream = NULL;
    g_vpcm_pjsip_modem.startup_local_stream = NULL;
    g_vpcm_pjsip_modem.startup_remote_stream = NULL;
    g_vpcm_pjsip_modem.startup_remote_rx_stream = NULL;
    g_vpcm_pjsip_modem.initialized = false;
}

static bool vpcm_pjsip_build_startup_sequences(v91_law_t law)
{
    enum { VPCM_STARTUP_MAX_CODEWORDS = 8192 };
    v91_state_t caller;
    v91_state_t answerer;
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    v91_info_frame_t info_rx;
    uint8_t caller_stream[VPCM_STARTUP_MAX_CODEWORDS];
    uint8_t answerer_stream[VPCM_STARTUP_MAX_CODEWORDS];
    uint8_t buf[2048];
    int caller_len;
    int answerer_len;
    int len;
    int cp_offer_len;
    int cp_ack_len;
    int caller_startup_len;
    int answerer_startup_len;
    int i;
    int caller_stage_len[VPCM_STARTUP_STAGE_COUNT];
    int answerer_stage_len[VPCM_STARTUP_STAGE_COUNT];
    int *remote_stage_len;
    vpcm_cp_frame_t cp_offer;
    vpcm_cp_frame_t cp_rx;
    bool is_caller;

    v91_init(&caller, law, V91_MODE_TRANSPARENT);
    v91_init(&answerer, law, V91_MODE_TRANSPARENT);
    v91_default_dil_init(&g_vpcm_pjsip_modem.default_dil);

    memset(&caller_info, 0, sizeof(caller_info));
    caller_info.request_default_dil = true;
    caller_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    caller_info.power_measured_after_digital_impairments = true;
    memset(&answerer_info, 0, sizeof(answerer_info));
    answerer_info.request_default_dil = true;
    answerer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    answerer_info.power_measured_after_digital_impairments = true;

    vpcm_cp_init_robbed_bit_safe_profile(&cp_offer, vpcm_cp_recommended_robbed_bit_drn(), false);
    cp_offer.acknowledge = false;
    g_vpcm_pjsip_modem.cp_ack = cp_offer;
    g_vpcm_pjsip_modem.cp_ack.acknowledge = true;
    g_vpcm_pjsip_modem.expected_remote_cp = (g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER)
        ? g_vpcm_pjsip_modem.cp_ack
        : cp_offer;

    caller_len = 0;
    answerer_len = 0;
    for (i = 0; i < VPCM_STARTUP_STAGE_COUNT; i++)
        caller_stage_len[i] = answerer_stage_len[i] = 0;

    len = v91_tx_phase1_silence_codewords(&caller, buf, (int) sizeof(buf));
    if (len != V91_PHASE1_SILENCE_SYMBOLS)
        return false;
    memcpy(caller_stream + caller_len, buf, (size_t) len);
    caller_len += len;
    len = v91_tx_phase1_silence_codewords(&answerer, buf, (int) sizeof(buf));
    if (len != V91_PHASE1_SILENCE_SYMBOLS)
        return false;
    memcpy(answerer_stream + answerer_len, buf, (size_t) len);
    answerer_len += len;
    caller_stage_len[VPCM_STARTUP_STAGE_PHASE1] = V91_PHASE1_SILENCE_SYMBOLS;
    answerer_stage_len[VPCM_STARTUP_STAGE_PHASE1] = V91_PHASE1_SILENCE_SYMBOLS;

    len = v91_tx_ez_codewords(&caller, buf, (int) sizeof(buf));
    if (len != V91_EZ_SYMBOLS)
        return false;
    memcpy(caller_stream + caller_len, buf, (size_t) len);
    caller_len += len;
    len = v91_tx_ez_codewords(&answerer, buf, (int) sizeof(buf));
    if (len != V91_EZ_SYMBOLS)
        return false;
    memcpy(answerer_stream + answerer_len, buf, (size_t) len);
    answerer_len += len;
    caller_stage_len[VPCM_STARTUP_STAGE_EZ] = V91_EZ_SYMBOLS;
    answerer_stage_len[VPCM_STARTUP_STAGE_EZ] = V91_EZ_SYMBOLS;

    len = v91_tx_info_codewords(&caller, buf, (int) sizeof(buf), &caller_info);
    if (len != V91_INFO_SYMBOLS || !v91_rx_info_codewords(&answerer, buf, len, &info_rx))
        return false;
    memcpy(caller_stream + caller_len, buf, (size_t) len);
    caller_len += len;
    len = v91_tx_info_codewords(&answerer, buf, (int) sizeof(buf), &answerer_info);
    if (len != V91_INFO_SYMBOLS || !v91_rx_info_codewords(&caller, buf, len, &info_rx))
        return false;
    memcpy(answerer_stream + answerer_len, buf, (size_t) len);
    answerer_len += len;
    caller_stage_len[VPCM_STARTUP_STAGE_INFO] = V91_INFO_SYMBOLS;
    answerer_stage_len[VPCM_STARTUP_STAGE_INFO] = V91_INFO_SYMBOLS;

    len = v91_tx_startup_dil_sequence_codewords(&caller,
                                                buf,
                                                (int) sizeof(buf),
                                                &g_vpcm_pjsip_modem.default_dil,
                                                NULL);
    if (len <= 0)
        return false;
    caller_startup_len = len;
    memcpy(caller_stream + caller_len, buf, (size_t) len);
    caller_len += len;
    len = v91_tx_startup_dil_sequence_codewords(&answerer,
                                                buf,
                                                (int) sizeof(buf),
                                                &g_vpcm_pjsip_modem.default_dil,
                                                NULL);
    if (len <= 0)
        return false;
    answerer_startup_len = len;
    memcpy(answerer_stream + answerer_len, buf, (size_t) len);
    answerer_len += len;
    caller_stage_len[VPCM_STARTUP_STAGE_STARTUP_DIL] = caller_startup_len;
    answerer_stage_len[VPCM_STARTUP_STAGE_STARTUP_DIL] = answerer_startup_len;

    len = v91_tx_scr_codewords(&caller, buf, (int) sizeof(buf), 18);
    if (len != 18 || !v91_rx_scr_codewords(&answerer, buf, len, false))
        return false;
    memcpy(caller_stream + caller_len, buf, (size_t) len);
    caller_len += len;
    len = v91_tx_scr_codewords(&answerer, buf, (int) sizeof(buf), 18);
    if (len != 18 || !v91_rx_scr_codewords(&caller, buf, len, false))
        return false;
    memcpy(answerer_stream + answerer_len, buf, (size_t) len);
    answerer_len += len;
    caller_stage_len[VPCM_STARTUP_STAGE_SCR] = 18;
    answerer_stage_len[VPCM_STARTUP_STAGE_SCR] = 18;

    cp_offer_len = v91_tx_cp_codewords(&caller, buf, (int) sizeof(buf), &cp_offer, true);
    if (cp_offer_len <= 0 || !v91_rx_cp_codewords(&answerer, buf, cp_offer_len, &cp_rx, true))
        return false;
    memcpy(caller_stream + caller_len, buf, (size_t) cp_offer_len);
    caller_len += cp_offer_len;
    cp_ack_len = v91_tx_cp_codewords(&answerer, buf, (int) sizeof(buf), &g_vpcm_pjsip_modem.cp_ack, true);
    if (cp_ack_len <= 0 || !v91_rx_cp_codewords(&caller, buf, cp_ack_len, &cp_rx, true))
        return false;
    memcpy(answerer_stream + answerer_len, buf, (size_t) cp_ack_len);
    answerer_len += cp_ack_len;

    caller_stage_len[VPCM_STARTUP_STAGE_CP] = cp_offer_len;
    answerer_stage_len[VPCM_STARTUP_STAGE_CP] = cp_ack_len;

    len = v91_tx_es_codewords(&caller, buf, (int) sizeof(buf));
    if (len != V91_ES_SYMBOLS || !v91_rx_es_codewords(&answerer, buf, len, true))
        return false;
    memcpy(caller_stream + caller_len, buf, (size_t) len);
    caller_len += len;
    len = v91_tx_es_codewords(&answerer, buf, (int) sizeof(buf));
    if (len != V91_ES_SYMBOLS || !v91_rx_es_codewords(&caller, buf, len, true))
        return false;
    memcpy(answerer_stream + answerer_len, buf, (size_t) len);
    answerer_len += len;
    caller_stage_len[VPCM_STARTUP_STAGE_ES] = V91_ES_SYMBOLS;
    answerer_stage_len[VPCM_STARTUP_STAGE_ES] = V91_ES_SYMBOLS;

    len = v91_tx_b1_codewords(&caller, buf, (int) sizeof(buf), &g_vpcm_pjsip_modem.cp_ack);
    if (len != V91_B1_SYMBOLS || !v91_rx_b1_codewords(&answerer, buf, len, &g_vpcm_pjsip_modem.cp_ack))
        return false;
    memcpy(caller_stream + caller_len, buf, (size_t) len);
    caller_len += len;
    len = v91_tx_b1_codewords(&answerer, buf, (int) sizeof(buf), &g_vpcm_pjsip_modem.cp_ack);
    if (len != V91_B1_SYMBOLS || !v91_rx_b1_codewords(&caller, buf, len, &g_vpcm_pjsip_modem.cp_ack))
        return false;
    memcpy(answerer_stream + answerer_len, buf, (size_t) len);
    answerer_len += len;
    caller_stage_len[VPCM_STARTUP_STAGE_B1] = V91_B1_SYMBOLS;
    answerer_stage_len[VPCM_STARTUP_STAGE_B1] = V91_B1_SYMBOLS;

    is_caller = (g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER);
    remote_stage_len = is_caller ? answerer_stage_len : caller_stage_len;
    for (i = 0; i < VPCM_STARTUP_STAGE_COUNT; i++)
        g_vpcm_pjsip_modem.startup_stage_lengths[i] = remote_stage_len[i];
    g_vpcm_pjsip_modem.startup_stage_offsets[0] = 0;
    for (i = 1; i < VPCM_STARTUP_STAGE_COUNT; i++) {
        g_vpcm_pjsip_modem.startup_stage_offsets[i] =
            g_vpcm_pjsip_modem.startup_stage_offsets[i - 1]
            + g_vpcm_pjsip_modem.startup_stage_lengths[i - 1];
    }

    g_vpcm_pjsip_modem.startup_local_tx_len = is_caller ? caller_len : answerer_len;
    g_vpcm_pjsip_modem.startup_remote_total_len = is_caller ? answerer_len : caller_len;
    g_vpcm_pjsip_modem.startup_local_stream = (uint8_t *) malloc((size_t) g_vpcm_pjsip_modem.startup_local_tx_len);
    g_vpcm_pjsip_modem.startup_remote_stream = (uint8_t *) malloc((size_t) g_vpcm_pjsip_modem.startup_remote_total_len);
    g_vpcm_pjsip_modem.startup_remote_rx_stream = (uint8_t *) malloc((size_t) g_vpcm_pjsip_modem.startup_remote_total_len);
    if (!g_vpcm_pjsip_modem.startup_local_stream
        || !g_vpcm_pjsip_modem.startup_remote_stream
        || !g_vpcm_pjsip_modem.startup_remote_rx_stream)
        return false;
    if (is_caller) {
        memcpy(g_vpcm_pjsip_modem.startup_local_stream, caller_stream, (size_t) caller_len);
        memcpy(g_vpcm_pjsip_modem.startup_remote_stream, answerer_stream, (size_t) answerer_len);
    } else {
        memcpy(g_vpcm_pjsip_modem.startup_local_stream, answerer_stream, (size_t) answerer_len);
        memcpy(g_vpcm_pjsip_modem.startup_remote_stream, caller_stream, (size_t) caller_len);
    }
    g_vpcm_pjsip_modem.startup_local_tx_pos = 0;
    g_vpcm_pjsip_modem.startup_remote_rx_len = 0;
    g_vpcm_pjsip_modem.startup_rx_stage = 0;
    memset(g_vpcm_pjsip_modem.startup_remote_rx_stream, 0, (size_t) g_vpcm_pjsip_modem.startup_remote_total_len);
    g_vpcm_pjsip_modem.startup_timeout_packets = 20 * g_vpcm_pjsip_modem.packets_per_second;
    g_vpcm_pjsip_modem.startup_pkt_count = 0;
    return true;
}

static bool vpcm_pjsip_modem_prepare(v91_law_t law, int data_seconds, uint32_t seed,
                                     int packet_codewords_per_pull)
{
    int bytes_per_packet;
    int total_packets;
    int total_frames;
    int total_bits;
    int total_bytes;
    int total_codewords;
    uint32_t tx_seed;
    uint32_t rx_seed;
    logging_state_t *v8_log;

    if (data_seconds <= 0)
        data_seconds = 10;

    vpcm_pjsip_modem_reset();
    g_vpcm_pjsip_modem.law = law;
    g_vpcm_pjsip_modem.packet_codewords_per_pull = packet_codewords_per_pull;
    g_vpcm_pjsip_modem.packets_per_second = 8000 / packet_codewords_per_pull;
    if (g_vpcm_pjsip_modem.packets_per_second < 1)
        g_vpcm_pjsip_modem.packets_per_second = 1;
    g_vpcm_pjsip_modem.v8_negotiation_timeout_packets =
        15 * g_vpcm_pjsip_modem.packets_per_second;
    g_vpcm_pjsip_modem.post_v8_zero_preroll_packets = 0;
    g_vpcm_pjsip_modem.drain_packets = g_vpcm_pjsip_modem.packets_per_second;
    if (g_vpcm_pjsip_modem.drain_packets < 1)
        g_vpcm_pjsip_modem.drain_packets = 1;
    g_vpcm_pjsip_modem.last_v8_status = -1;

    vpcm_cp_init_robbed_bit_safe_profile(&g_vpcm_pjsip_modem.cp,
                                         vpcm_cp_recommended_robbed_bit_drn(),
                                         false);
    g_vpcm_pjsip_modem.v91_codewords_per_packet =
        (packet_codewords_per_pull / VPCM_CP_FRAME_INTERVALS) * VPCM_CP_FRAME_INTERVALS;
    if (g_vpcm_pjsip_modem.v91_codewords_per_packet <= 0)
        return false;

    bytes_per_packet = (g_vpcm_pjsip_modem.v91_codewords_per_packet
                        * ((int) g_vpcm_pjsip_modem.cp.drn + 20)) / 48;
    if (bytes_per_packet <= 0)
        return false;

    total_packets = data_seconds * g_vpcm_pjsip_modem.packets_per_second;
    if (total_packets < 1)
        total_packets = g_vpcm_pjsip_modem.packets_per_second;
    total_frames = total_packets * (g_vpcm_pjsip_modem.v91_codewords_per_packet / VPCM_CP_FRAME_INTERVALS);
    total_bits = total_frames * ((int) g_vpcm_pjsip_modem.cp.drn + 20);
    total_bytes = total_bits / 8;
    total_codewords = total_packets * g_vpcm_pjsip_modem.v91_codewords_per_packet;

    g_vpcm_pjsip_modem.bytes_per_packet = bytes_per_packet;
    g_vpcm_pjsip_modem.total_packets = total_packets;
    g_vpcm_pjsip_modem.total_bytes = total_bytes;
    g_vpcm_pjsip_modem.total_codewords = total_codewords;

    g_vpcm_pjsip_modem.tx_data = (uint8_t *) malloc((size_t) total_bytes);
    g_vpcm_pjsip_modem.expected_rx = (uint8_t *) malloc((size_t) total_bytes);
    g_vpcm_pjsip_modem.rx_data = (uint8_t *) malloc((size_t) total_bytes);
    g_vpcm_pjsip_modem.tx_codeword_stream = (uint8_t *) malloc((size_t) total_codewords);
    if (!g_vpcm_pjsip_modem.tx_data || !g_vpcm_pjsip_modem.expected_rx
        || !g_vpcm_pjsip_modem.rx_data || !g_vpcm_pjsip_modem.tx_codeword_stream) {
        vpcm_pjsip_modem_cleanup();
        return false;
    }
    memset(g_vpcm_pjsip_modem.rx_data, 0, (size_t) total_bytes);

    tx_seed = (g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER) ? seed : (seed ^ 0xA55AA55AU);
    rx_seed = (g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER) ? (seed ^ 0xA55AA55AU) : seed;
    g_vpcm_pjsip_modem.idle_codeword = v91_ucode_to_codeword(law, 0, true);

    fill_pattern(g_vpcm_pjsip_modem.tx_data, total_bytes, tx_seed);
    fill_pattern(g_vpcm_pjsip_modem.expected_rx, total_bytes, rx_seed);

    v91_init(&g_vpcm_pjsip_modem.tx_state, law, V91_MODE_TRANSPARENT);
    v91_init(&g_vpcm_pjsip_modem.rx_state, law, V91_MODE_TRANSPARENT);
    if (!vpcm_pjsip_build_startup_sequences(law)) {
        vpcm_pjsip_modem_cleanup();
        return false;
    }

    memset(&g_vpcm_pjsip_modem.v8_result, 0, sizeof(g_vpcm_pjsip_modem.v8_result));
    init_v8_parms(&g_vpcm_pjsip_modem.v8_parms,
                  g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER,
                  V8_MOD_V22 | V8_MOD_V34,
                  V8_PSTN_PCM_MODEM_V91);
    g_vpcm_pjsip_modem.v8_state = v8_init(NULL,
                                          g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER,
                                          &g_vpcm_pjsip_modem.v8_parms,
                                          vpcm_v8_result_handler,
                                          &g_vpcm_pjsip_modem.v8_result);
    if (!g_vpcm_pjsip_modem.v8_state) {
        vpcm_pjsip_modem_cleanup();
        return false;
    }
    if (g_vpcm_session_diag || g_vpcm_verbose) {
        v8_log = v8_get_logging_state(g_vpcm_pjsip_modem.v8_state);
        if (v8_log) {
            span_log_set_level(v8_log, SPAN_LOG_FLOW | SPAN_LOG_SHOW_TAG);
            span_log_set_tag(v8_log,
                             (g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER)
                                 ? "V8-CALLER"
                                 : "V8-ANSWERER");
        }
    }

    g_vpcm_pjsip_modem.initialized = true;
    return true;
}

static void vpcm_pjsip_modem_fill_rtp_tx_payload(uint8_t *payload, int payload_len)
{
    int produced = 0;
    int frame_index;
    int codeword_index;
    int cw_off;
    int data_pkt = 0;
    bool sending_data;
    bool sending_v8;
    int16_t tx_linear[320];
    uint8_t tx_codewords[320];

    if (!payload || payload_len <= 0)
        return;
    if (!g_vpcm_pjsip_modem_lock)
        return;
    pj_mutex_lock(g_vpcm_pjsip_modem_lock);

    if (!g_vpcm_pjsip_modem.initialized || g_vpcm_pjsip_modem.failed || g_vpcm_pjsip_modem.completed) {
        memset(payload, 0xFF, (size_t) payload_len);
        pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
        return;
    }

    if (payload_len > 320)
        payload_len = 320;
    if (g_vpcm_pjsip_modem.packet_codewords_per_pull != payload_len) {
        g_vpcm_pjsip_modem.failed = true;
        pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
        return;
    }

    sending_v8 = !g_vpcm_pjsip_modem.v8_complete;
    sending_data = (g_vpcm_pjsip_modem.data_mode_active
                    && g_vpcm_pjsip_modem.data_tx_pkt < g_vpcm_pjsip_modem.total_packets);
    if (sending_data)
        data_pkt = g_vpcm_pjsip_modem.data_tx_pkt;

    if (sending_v8) {
        produced = v8_tx(g_vpcm_pjsip_modem.v8_state, tx_linear, payload_len);
        if (produced < 0)
            produced = 0;
        if (produced > payload_len)
            produced = payload_len;
        if (produced > 0)
            vpcm_linear_to_g711_payload(tx_linear, produced, g_vpcm_pjsip_modem.law, payload);
        if (produced < payload_len)
            memset(payload + produced, g_vpcm_pjsip_modem.idle_codeword, (size_t) (payload_len - produced));
        g_vpcm_pjsip_modem.startup_tx_pkt++;
        g_vpcm_pjsip_modem.v8_pkt_count++;
    } else if (!g_vpcm_pjsip_modem.data_mode_active) {
        int remain;
        int copy_len;

        memset(payload, g_vpcm_pjsip_modem.idle_codeword, (size_t) payload_len);
        remain = g_vpcm_pjsip_modem.startup_local_tx_len - g_vpcm_pjsip_modem.startup_local_tx_pos;
        if (remain > 0) {
            copy_len = remain;
            if (copy_len > payload_len)
                copy_len = payload_len;
            memcpy(payload,
                   g_vpcm_pjsip_modem.startup_local_stream + g_vpcm_pjsip_modem.startup_local_tx_pos,
                   (size_t) copy_len);
            g_vpcm_pjsip_modem.startup_local_tx_pos += copy_len;
        }
        g_vpcm_pjsip_modem.startup_pkt_count++;
    } else if (sending_data) {
        memset(payload, g_vpcm_pjsip_modem.idle_codeword, (size_t) payload_len);
        cw_off = data_pkt * g_vpcm_pjsip_modem.v91_codewords_per_packet;
        memcpy(tx_codewords,
               g_vpcm_pjsip_modem.tx_codeword_stream + cw_off,
               (size_t) g_vpcm_pjsip_modem.v91_codewords_per_packet);
        memcpy(payload, tx_codewords, (size_t) g_vpcm_pjsip_modem.v91_codewords_per_packet);
        g_vpcm_pjsip_modem.data_tx_pkt++;
        g_vpcm_pjsip_modem.tx_len += g_vpcm_pjsip_modem.bytes_per_packet;
        frame_index = (data_pkt * g_vpcm_pjsip_modem.v91_codewords_per_packet) / VPCM_CP_FRAME_INTERVALS;
        codeword_index = data_pkt * g_vpcm_pjsip_modem.v91_codewords_per_packet;
        if (g_vpcm_realtime
            && (data_pkt % g_vpcm_pjsip_modem.packets_per_second) == 0) {
            char tx_hex[64];
            vpcm_bytes_to_hex(tx_hex, sizeof(tx_hex), tx_codewords, 6);
            if (g_vpcm_pjsip_modem.have_last_rx_pcm) {
                char rx_pcm_hex[64];
                vpcm_bytes_to_hex(rx_pcm_hex, sizeof(rx_pcm_hex), g_vpcm_pjsip_modem.last_rx_pcm_sample, 6);
                if (g_vpcm_pjsip_modem.have_last_rx_decoded) {
                    char rx_data_hex[64];
                    char exp_data_hex[64];
                    bool chunk_ok = (memcmp(g_vpcm_pjsip_modem.last_rx_decoded_sample,
                                            g_vpcm_pjsip_modem.last_expected_sample, 6) == 0);
                    vpcm_bytes_to_hex(rx_data_hex, sizeof(rx_data_hex),
                                      g_vpcm_pjsip_modem.last_rx_decoded_sample, 6);
                    vpcm_bytes_to_hex(exp_data_hex, sizeof(exp_data_hex),
                                      g_vpcm_pjsip_modem.last_expected_sample, 6);
                    vpcm_log_e2e_phase("DATA",
                                       "frame=%d codeword=%d tx=[%s] rx_pcm=[%s] rx_data=[%s] expected=[%s] %s decoded_bytes=%d decoded_codewords=%d",
                                       frame_index, codeword_index, tx_hex, rx_pcm_hex, rx_data_hex, exp_data_hex,
                                       chunk_ok ? "OK" : "MISMATCH",
                                       g_vpcm_pjsip_modem.rx_len,
                                       g_vpcm_pjsip_modem.rx_codewords);
                } else {
                    vpcm_log_e2e_phase("DATA",
                                       "frame=%d codeword=%d tx=[%s] rx_pcm=[%s] rx_data=[pending] decoded_bytes=%d decoded_codewords=%d",
                                       frame_index, codeword_index, tx_hex, rx_pcm_hex,
                                       g_vpcm_pjsip_modem.rx_len, g_vpcm_pjsip_modem.rx_codewords);
                }
            }
        }
    } else {
        memset(payload, g_vpcm_pjsip_modem.idle_codeword, (size_t) payload_len);
        if (g_vpcm_pjsip_modem.data_started)
            g_vpcm_pjsip_modem.drain_seen_packets++;
    }

    if (g_vpcm_pjsip_modem.data_mode_active
        && g_vpcm_pjsip_modem.data_tx_pkt >= g_vpcm_pjsip_modem.total_packets
        && g_vpcm_pjsip_modem.drain_seen_packets >= g_vpcm_pjsip_modem.drain_packets) {
        g_vpcm_pjsip_modem.completed = true;
    }

    pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
}

static void vpcm_pjsip_modem_handle_rtp_rx_payload(const uint8_t *payload, int payload_len)
{
    int n;
    int before;
    int added;
    int16_t rx_linear[320];

    if (!payload || payload_len <= 0)
        return;
    if (!g_vpcm_pjsip_modem_lock)
        return;
    pj_mutex_lock(g_vpcm_pjsip_modem_lock);

    if (!g_vpcm_pjsip_modem.initialized || g_vpcm_pjsip_modem.failed || g_vpcm_pjsip_modem.completed) {
        pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
        return;
    }

    n = (payload_len < 6) ? payload_len : 6;
    memcpy(g_vpcm_pjsip_modem.last_rx_pcm_sample, payload, (size_t) n);
    if (n < 6)
        memset(g_vpcm_pjsip_modem.last_rx_pcm_sample + n, 0, (size_t) (6 - n));
    g_vpcm_pjsip_modem.have_last_rx_pcm = true;

    if (!g_vpcm_pjsip_modem.v8_complete) {
        g_vpcm_pjsip_modem.v8_rx_packets++;
        g_vpcm_pjsip_modem.v8_rx_bytes += payload_len;
        if (payload_len > 320)
            payload_len = 320;
        vpcm_g711_payload_to_linear(payload, payload_len, g_vpcm_pjsip_modem.law, rx_linear);
        v8_rx(g_vpcm_pjsip_modem.v8_state, rx_linear, payload_len);
        pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
        return;
    }

    if (!g_vpcm_pjsip_modem.data_mode_active) {
        int room;
        int copy_len;

        room = g_vpcm_pjsip_modem.startup_remote_total_len - g_vpcm_pjsip_modem.startup_remote_rx_len;
        if (room > 0) {
            copy_len = payload_len;
            if (copy_len > room)
                copy_len = room;
            memcpy(g_vpcm_pjsip_modem.startup_remote_rx_stream + g_vpcm_pjsip_modem.startup_remote_rx_len,
                   payload,
                   (size_t) copy_len);
            g_vpcm_pjsip_modem.startup_remote_rx_len += copy_len;
        }
        while (!g_vpcm_pjsip_modem.failed
               && g_vpcm_pjsip_modem.startup_rx_stage < VPCM_STARTUP_STAGE_COUNT) {
            int stage = g_vpcm_pjsip_modem.startup_rx_stage;
            int off = g_vpcm_pjsip_modem.startup_stage_offsets[stage];
            int len = g_vpcm_pjsip_modem.startup_stage_lengths[stage];
            const uint8_t *stage_buf;

            if (g_vpcm_pjsip_modem.startup_remote_rx_len < (off + len))
                break;
            stage_buf = g_vpcm_pjsip_modem.startup_remote_rx_stream + off;
            if (stage == VPCM_STARTUP_STAGE_PHASE1 || stage == VPCM_STARTUP_STAGE_EZ
                || stage == VPCM_STARTUP_STAGE_STARTUP_DIL) {
                if (memcmp(stage_buf, g_vpcm_pjsip_modem.startup_remote_stream + off, (size_t) len) != 0) {
                    g_vpcm_pjsip_modem.failed = true;
                    break;
                }
                if (stage == VPCM_STARTUP_STAGE_STARTUP_DIL
                    && !v91_note_received_dil(&g_vpcm_pjsip_modem.rx_state,
                                              &g_vpcm_pjsip_modem.default_dil,
                                              NULL)) {
                    g_vpcm_pjsip_modem.failed = true;
                    break;
                }
            } else if (stage == VPCM_STARTUP_STAGE_INFO) {
                v91_info_frame_t info;
                if (!v91_rx_info_codewords(&g_vpcm_pjsip_modem.rx_state, stage_buf, len, &info)) {
                    g_vpcm_pjsip_modem.failed = true;
                    break;
                }
            } else if (stage == VPCM_STARTUP_STAGE_SCR) {
                if (!v91_rx_scr_codewords(&g_vpcm_pjsip_modem.rx_state, stage_buf, len, false)) {
                    g_vpcm_pjsip_modem.failed = true;
                    break;
                }
            } else if (stage == VPCM_STARTUP_STAGE_CP) {
                vpcm_cp_frame_t cp_rx;
                if (!v91_rx_cp_codewords(&g_vpcm_pjsip_modem.rx_state, stage_buf, len, &cp_rx, true)
                    || !vpcm_cp_frames_equal(&cp_rx, &g_vpcm_pjsip_modem.expected_remote_cp)) {
                    g_vpcm_pjsip_modem.failed = true;
                    break;
                }
            } else if (stage == VPCM_STARTUP_STAGE_ES) {
                if (!v91_rx_es_codewords(&g_vpcm_pjsip_modem.rx_state, stage_buf, len, true)) {
                    g_vpcm_pjsip_modem.failed = true;
                    break;
                }
            } else if (stage == VPCM_STARTUP_STAGE_B1) {
                if (!v91_rx_b1_codewords(&g_vpcm_pjsip_modem.rx_state, stage_buf, len, &g_vpcm_pjsip_modem.cp_ack)) {
                    g_vpcm_pjsip_modem.failed = true;
                    break;
                }
            }
            g_vpcm_pjsip_modem.startup_rx_stage++;
        }
        if (!g_vpcm_pjsip_modem.failed
            && g_vpcm_pjsip_modem.startup_rx_stage >= VPCM_STARTUP_STAGE_COUNT
            && g_vpcm_pjsip_modem.startup_local_tx_pos >= g_vpcm_pjsip_modem.startup_local_tx_len) {
            if (!v91_activate_data_mode(&g_vpcm_pjsip_modem.tx_state, &g_vpcm_pjsip_modem.cp_ack)
                || !v91_activate_data_mode(&g_vpcm_pjsip_modem.rx_state, &g_vpcm_pjsip_modem.cp_ack)
                || v91_tx_codewords(&g_vpcm_pjsip_modem.tx_state,
                                    g_vpcm_pjsip_modem.tx_codeword_stream,
                                    g_vpcm_pjsip_modem.total_codewords,
                                    g_vpcm_pjsip_modem.tx_data,
                                    g_vpcm_pjsip_modem.total_bytes) != g_vpcm_pjsip_modem.total_codewords) {
                g_vpcm_pjsip_modem.failed = true;
            } else {
                g_vpcm_pjsip_modem.data_mode_active = true;
                g_vpcm_pjsip_modem.data_started = true;
                vpcm_log_e2e_phase("V8", "V.91 startup complete from received symbols; entering mapped data mode");
                vpcm_log_e2e_phase("DATA",
                                   "mapped payload streaming active (codec=%s)",
                                   vpcm_expected_codec_name(g_vpcm_pjsip_modem.law));
            }
        }
        pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
        return;
    }

    before = g_vpcm_pjsip_modem.rx_len;
    added = vpcm_decode_rx_codewords(&g_vpcm_pjsip_modem.rx_state,
                                     payload,
                                     payload_len,
                                     g_vpcm_pjsip_modem.rx_data,
                                     g_vpcm_pjsip_modem.total_bytes,
                                     &g_vpcm_pjsip_modem.rx_len,
                                     &g_vpcm_pjsip_modem.rx_codewords,
                                     g_vpcm_pjsip_modem.v91_codewords_per_packet);
    if (added > 0 && g_vpcm_pjsip_modem.rx_len > before) {
        n = (added < 6) ? added : 6;
        memcpy(g_vpcm_pjsip_modem.last_rx_decoded_sample, g_vpcm_pjsip_modem.rx_data + before, (size_t) n);
        if (n < 6)
            memset(g_vpcm_pjsip_modem.last_rx_decoded_sample + n, 0, (size_t) (6 - n));
        memcpy(g_vpcm_pjsip_modem.last_expected_sample, g_vpcm_pjsip_modem.expected_rx + before, (size_t) n);
        if (n < 6)
            memset(g_vpcm_pjsip_modem.last_expected_sample + n, 0, (size_t) (6 - n));
        g_vpcm_pjsip_modem.have_last_rx_decoded = true;
        if (!g_vpcm_pjsip_modem.remote_data_logged) {
            g_vpcm_pjsip_modem.remote_data_logged = true;
            vpcm_log_e2e_phase("DATA", "Remote payload detected during drain; decode stream starting");
        }
    }

    pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
}

static bool vpcm_run_pjsip_payload_loop(v91_law_t law, int data_seconds, uint32_t seed)
{
    enum { DEFAULT_RTP_CODEWORDS_PER_PACKET = 160, V8_MAX_RESTARTS = 3 };
    const pjmedia_port_info *stream_info;
    int packet_codewords_per_pull;
    int status;
    int cmp_len;
    bool ok = false;
    if (!g_vpcm_pjsip_runtime.stream_port) {
        fprintf(stderr, "PJSIP stream port not available for passthrough media loop\n");
        return false;
    }
    packet_codewords_per_pull = DEFAULT_RTP_CODEWORDS_PER_PACKET;
    stream_info = &g_vpcm_pjsip_runtime.stream_port->info;
    if (stream_info
        && stream_info->fmt.type == PJMEDIA_TYPE_AUDIO
        && stream_info->fmt.detail_type == PJMEDIA_FORMAT_DETAIL_AUDIO) {
        unsigned spf = PJMEDIA_PIA_SPF(stream_info);
        if (spf > 0 && spf <= 320)
            packet_codewords_per_pull = (int) spf;
        vpcm_log("PJSIP stream format: srate=%u ptime=%ums bits=%u avg_bps=%u spf=%u",
                 PJMEDIA_PIA_SRATE(stream_info),
                 PJMEDIA_PIA_PTIME(stream_info),
                 PJMEDIA_PIA_BITS(stream_info),
                 PJMEDIA_PIA_AVG_BPS(stream_info),
                 spf);
    }
    if (packet_codewords_per_pull <= 0 || packet_codewords_per_pull > 320) {
        fprintf(stderr, "invalid PJSIP packet codewords=%d\n", packet_codewords_per_pull);
        return false;
    }

    if (!g_vpcm_pjsip_modem_lock) {
        if (!g_vpcm_pjsip_modem_pool)
            g_vpcm_pjsip_modem_pool = pjsua_pool_create("vpcm-modem", 1024, 1024);
        if (!g_vpcm_pjsip_modem_pool
            || pj_mutex_create_simple(g_vpcm_pjsip_modem_pool,
                                      "vpcm-modem-lock",
                                      &g_vpcm_pjsip_modem_lock) != PJ_SUCCESS) {
            fprintf(stderr, "failed to create PJSIP modem lock\n");
            return false;
        }
    }

    if (!vpcm_pjsip_modem_prepare(law, data_seconds, seed, packet_codewords_per_pull)) {
        fprintf(stderr, "failed to initialize raw RTP modem state\n");
        return false;
    }

    vpcm_log_e2e_phase("V8",
                       "In-call V.8 negotiation starting (law=%s, transcode=SpanDSP G.711<->linear, packet=%d samples)",
                       vpcm_law_to_str(law),
                       packet_codewords_per_pull);
    vpcm_log_e2e_phase("V8",
                       "local role=%s send_ci=%d answer_tone=%s",
                       vpcm_pjsip_role_to_str(g_vpcm_pjsip_runtime.role),
                       g_vpcm_pjsip_modem.v8_parms.send_ci ? 1 : 0,
                       modem_connect_tone_to_str(g_vpcm_pjsip_modem.v8_parms.modem_connect_tone));
    vpcm_log_e2e_phase("V8",
                       "Plan: after V.8, run receive-gated V.91 startup stages (Phase1/Ez/INFO/DIL/SCR/CP/Es/B1) and only enter data mode after remote stages validate");
    vpcm_log_e2e_phase("DATA",
                       "PJSIP transmit plan: role=%s seconds=%d packets=%d bytes=%d codewords=%d codec=%s (PJMEDIA passthrough stream)",
                       vpcm_pjsip_role_to_str(g_vpcm_pjsip_runtime.role),
                       data_seconds > 0 ? data_seconds : 10,
                       g_vpcm_pjsip_modem.total_packets,
                       g_vpcm_pjsip_modem.total_bytes,
                       g_vpcm_pjsip_modem.total_packets * g_vpcm_pjsip_modem.v91_codewords_per_packet,
                       vpcm_expected_codec_name(law));

    (void) seed;

    while (!g_vpcm_stop_requested && !g_vpcm_pjsip_runtime.call_disconnected) {
        pjsua_handle_events(10);
        pj_mutex_lock(g_vpcm_pjsip_modem_lock);
        if (!g_vpcm_pjsip_modem.v8_complete
            && g_vpcm_pjsip_modem.v8_result.seen
            && g_vpcm_pjsip_modem.v8_result.result.status != g_vpcm_pjsip_modem.last_v8_status) {
            status = g_vpcm_pjsip_modem.v8_result.result.status;
            g_vpcm_pjsip_modem.last_v8_status = status;
            vpcm_log_e2e_phase("V8",
                               "status=%s (%d) after %.2fs tx_packets=%d rx_packets=%d rx_bytes=%d",
                               v8_status_to_str(status),
                               status,
                               (double) g_vpcm_pjsip_modem.v8_pkt_count
                                   / (double) g_vpcm_pjsip_modem.packets_per_second,
                               g_vpcm_pjsip_modem.startup_tx_pkt,
                               g_vpcm_pjsip_modem.v8_rx_packets,
                               g_vpcm_pjsip_modem.v8_rx_bytes);
            if (status == V8_STATUS_V8_CALL) {
                vpcm_log_v8_result(vpcm_pjsip_role_to_str(g_vpcm_pjsip_runtime.role),
                                   &g_vpcm_pjsip_modem.v8_result.result);
                if (vpcm_v8_result_has_v91(&g_vpcm_pjsip_modem.v8_result.result)) {
                    g_vpcm_pjsip_modem.v8_complete = true;
                    vpcm_log_e2e_phase("V8",
                                       "V.8 result confirms V.91 PCM capability; beginning receive-gated V.91 startup exchange");
                } else {
                    fprintf(stderr,
                            "V.8 negotiated a call but not V.91 PCM mode: pcm=0x%X pstn=0x%X\n",
                            g_vpcm_pjsip_modem.v8_result.result.jm_cm.pcm_modem_availability,
                            g_vpcm_pjsip_modem.v8_result.result.jm_cm.pstn_access);
                    g_vpcm_pjsip_modem.failed = true;
                }
            } else if (status == V8_STATUS_FAILED
                       || status == V8_STATUS_NON_V8_CALL
                       || status == V8_STATUS_CALLING_TONE_RECEIVED
                       || status == V8_STATUS_FAX_CNG_TONE_RECEIVED) {
                if (g_vpcm_pjsip_modem.v8_restart_count < V8_MAX_RESTARTS) {
                    if (v8_restart(g_vpcm_pjsip_modem.v8_state,
                                   g_vpcm_pjsip_runtime.role == VPCM_PJSIP_ROLE_CALLER,
                                   &g_vpcm_pjsip_modem.v8_parms) != 0) {
                        g_vpcm_pjsip_modem.failed = true;
                    } else {
                        g_vpcm_pjsip_modem.v8_restart_count++;
                        memset(&g_vpcm_pjsip_modem.v8_result, 0, sizeof(g_vpcm_pjsip_modem.v8_result));
                        g_vpcm_pjsip_modem.last_v8_status = -1;
                        vpcm_log_e2e_phase("V8",
                                           "terminal status %s (%d) observed, restarting V.8 attempt %d/%d",
                                           v8_status_to_str(status),
                                           status,
                                           g_vpcm_pjsip_modem.v8_restart_count,
                                           V8_MAX_RESTARTS);
                    }
                } else {
                    fprintf(stderr,
                            "V.8 negotiation failed in-call after %d restarts: status=%s (%d)\n",
                            g_vpcm_pjsip_modem.v8_restart_count,
                            v8_status_to_str(status),
                            status);
                    g_vpcm_pjsip_modem.failed = true;
                }
            }
        }
        if (!g_vpcm_pjsip_modem.v8_complete
            && g_vpcm_pjsip_modem.v8_pkt_count >= g_vpcm_pjsip_modem.v8_negotiation_timeout_packets) {
            fprintf(stderr, "V.8 negotiation timed out in-call after %.1fs\n",
                    (double) g_vpcm_pjsip_modem.v8_pkt_count
                        / (double) g_vpcm_pjsip_modem.packets_per_second);
            g_vpcm_pjsip_modem.failed = true;
        }
        if (g_vpcm_pjsip_modem.v8_complete
            && !g_vpcm_pjsip_modem.data_mode_active
            && g_vpcm_pjsip_modem.startup_pkt_count >= g_vpcm_pjsip_modem.startup_timeout_packets) {
            fprintf(stderr, "V.91 startup timed out after %.1fs waiting for receive-gated stage completion\n",
                    (double) g_vpcm_pjsip_modem.startup_pkt_count
                        / (double) g_vpcm_pjsip_modem.packets_per_second);
            g_vpcm_pjsip_modem.failed = true;
        }
        if (g_vpcm_pjsip_modem.completed || g_vpcm_pjsip_modem.failed) {
            pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
            break;
        }
        pj_mutex_unlock(g_vpcm_pjsip_modem_lock);
        vpcm_maybe_realtime_pace_samples(packet_codewords_per_pull);
    }

    pj_mutex_lock(g_vpcm_pjsip_modem_lock);
    if (!g_vpcm_pjsip_modem.failed) {
        cmp_len = (g_vpcm_pjsip_modem.rx_len < g_vpcm_pjsip_modem.total_bytes)
            ? g_vpcm_pjsip_modem.rx_len
            : g_vpcm_pjsip_modem.total_bytes;
        if (cmp_len > 0) {
            g_vpcm_pjsip_modem.bit_errors = vpcm_count_bit_errors(g_vpcm_pjsip_modem.expected_rx,
                                                                  g_vpcm_pjsip_modem.rx_data,
                                                                  cmp_len);
            g_vpcm_pjsip_modem.bits_checked = (uint64_t) cmp_len * 8ULL;
        }
        ok = (g_vpcm_pjsip_modem.v8_complete
              && cmp_len == g_vpcm_pjsip_modem.total_bytes
              && g_vpcm_pjsip_modem.bit_errors == 0);
    }

    if (!g_vpcm_pjsip_modem.v8_complete) {
        vpcm_log_e2e_phase("V8",
                           "V.8 completion was not observed before call stop (tx_packets=%d rx_packets=%d rx_bytes=%d restarts=%d)",
                           g_vpcm_pjsip_modem.startup_tx_pkt,
                           g_vpcm_pjsip_modem.v8_rx_packets,
                           g_vpcm_pjsip_modem.v8_rx_bytes,
                           g_vpcm_pjsip_modem.v8_restart_count);
    }
    vpcm_log("PJSIP payload stats: role=%s tx_bytes=%d expected_rx_bytes=%d actual_rx_bytes=%d rx_codewords=%d bits=%llu bit_errors=%llu ber=%.3e v8_complete=%s",
             vpcm_pjsip_role_to_str(g_vpcm_pjsip_runtime.role),
             g_vpcm_pjsip_modem.tx_len,
             g_vpcm_pjsip_modem.total_bytes,
             g_vpcm_pjsip_modem.rx_len,
             g_vpcm_pjsip_modem.rx_codewords,
             (unsigned long long) g_vpcm_pjsip_modem.bits_checked,
             (unsigned long long) g_vpcm_pjsip_modem.bit_errors,
             (g_vpcm_pjsip_modem.bits_checked == 0) ? 0.0
                 : ((double) g_vpcm_pjsip_modem.bit_errors / (double) g_vpcm_pjsip_modem.bits_checked),
             g_vpcm_pjsip_modem.v8_complete ? "yes" : "no");
    pj_mutex_unlock(g_vpcm_pjsip_modem_lock);

    vpcm_pjsip_modem_cleanup();
    return ok;
}

static bool run_v91_single_e2e_call_pjsip(v91_law_t law, int data_seconds, uint32_t seed)
{
    const char *role_env;
    const char *dest_uri_env;
    const char *account_id_env;
    const char *reg_uri_env;
    const char *auth_user_env;
    const char *auth_pass_env;
    const char *auth_realm_env;
    const char *local_port_env;
    const char *rtp_port_env;
    const char *rtp_range_env;
    const char *caller_number_env;
    const char *answerer_number_env;
    const char *domain_env;
    const char *dummy_uri_env;
    const char *wait_seconds_env;
    pjsua_config ua_cfg;
    pjsua_logging_config log_cfg;
    pjsua_media_config media_cfg;
    pjsua_transport_config tp_cfg;
    pjsua_acc_config acc_cfg;
    pjsua_call_setting call_opt;
    pjsua_transport_id tid;
    pjsua_acc_id acc_id;
    pj_str_t dst_uri;
    pj_status_t st;
    double start_wait;
    double wait_limit;
    int local_port;
    int rtp_port;
    int rtp_range;
    int wait_seconds;
    bool ok;
    bool wait_confirm_logged;
    bool using_dummy_uri;
    vpcm_pjsip_role_t role;
    char caller_id_buf[256];
    char answerer_id_buf[256];
    char dest_uri_buf[256];

#if !PJMEDIA_HAS_PASSTHROUGH_CODECS
    fprintf(stderr, "PJMEDIA passthrough codecs are not enabled in pjproject build\n");
    return false;
#endif
#if !PJMEDIA_HAS_PASSTHROUGH_CODEC_PCMU || !PJMEDIA_HAS_PASSTHROUGH_CODEC_PCMA
    fprintf(stderr, "PJMEDIA passthrough G.711 codecs (PCMU/PCMA) are not fully enabled\n");
    return false;
#endif

    role_env = getenv("VPCM_PJSIP_ROLE");
    role = vpcm_pjsip_parse_role(role_env);
    account_id_env = getenv("VPCM_PJSIP_ACCOUNT_ID");
    reg_uri_env = getenv("VPCM_PJSIP_REG_URI");
    auth_user_env = getenv("VPCM_PJSIP_AUTH_USER");
    auth_pass_env = getenv("VPCM_PJSIP_AUTH_PASS");
    auth_realm_env = getenv("VPCM_PJSIP_AUTH_REALM");
    local_port_env = getenv("VPCM_PJSIP_LOCAL_PORT");
    rtp_port_env = getenv("VPCM_PJSIP_RTP_PORT");
    rtp_range_env = getenv("VPCM_PJSIP_RTP_RANGE");
    caller_number_env = getenv("VPCM_PJSIP_CALLER_NUMBER");
    answerer_number_env = getenv("VPCM_PJSIP_ANSWERER_NUMBER");
    domain_env = getenv("VPCM_PJSIP_DOMAIN");
    dummy_uri_env = getenv("VPCM_PJSIP_DUMMY_URI");
    dest_uri_env = getenv("VPCM_PJSIP_DEST_URI");
    wait_seconds_env = getenv("VPCM_PJSIP_WAIT_SECONDS");

    local_port = 0;
    if (local_port_env && local_port_env[0] != '\0')
        local_port = atoi(local_port_env);
    rtp_port = 0;
    if (rtp_port_env && rtp_port_env[0] != '\0')
        rtp_port = atoi(rtp_port_env);
    rtp_range = 0;
    if (rtp_range_env && rtp_range_env[0] != '\0')
        rtp_range = atoi(rtp_range_env);
    wait_seconds = 60;
    if (wait_seconds_env && wait_seconds_env[0] != '\0') {
        wait_seconds = atoi(wait_seconds_env);
        if (wait_seconds < 5)
            wait_seconds = 5;
    }

    memset(caller_id_buf, 0, sizeof(caller_id_buf));
    memset(answerer_id_buf, 0, sizeof(answerer_id_buf));
    memset(dest_uri_buf, 0, sizeof(dest_uri_buf));
    using_dummy_uri = false;

    if ((!account_id_env || account_id_env[0] == '\0')
        && domain_env && domain_env[0] != '\0') {
        if (role == VPCM_PJSIP_ROLE_CALLER
            && caller_number_env && caller_number_env[0] != '\0') {
            snprintf(caller_id_buf, sizeof(caller_id_buf), "sip:%s@%s", caller_number_env, domain_env);
            account_id_env = caller_id_buf;
        } else if (role == VPCM_PJSIP_ROLE_ANSWERER
                   && answerer_number_env && answerer_number_env[0] != '\0') {
            snprintf(answerer_id_buf, sizeof(answerer_id_buf), "sip:%s@%s", answerer_number_env, domain_env);
            account_id_env = answerer_id_buf;
        }
    }

    if (role == VPCM_PJSIP_ROLE_CALLER) {
        if (!dest_uri_env || dest_uri_env[0] == '\0') {
            if (answerer_number_env && answerer_number_env[0] != '\0'
                && domain_env && domain_env[0] != '\0') {
                snprintf(dest_uri_buf, sizeof(dest_uri_buf), "sip:%s@%s", answerer_number_env, domain_env);
                dest_uri_env = dest_uri_buf;
            }
        }
        if (!dest_uri_env || dest_uri_env[0] == '\0') {
            if (dummy_uri_env && dummy_uri_env[0] != '\0') {
                dest_uri_env = dummy_uri_env;
            } else {
                dest_uri_env = "sip:dummy@127.0.0.1";
            }
            using_dummy_uri = true;
            vpcm_log("PJSIP destination not configured; using dummy URI %s", dest_uri_env);
        }
    }

    memset(&g_vpcm_pjsip_runtime, 0, sizeof(g_vpcm_pjsip_runtime));
    g_vpcm_pjsip_runtime.call_id = PJSUA_INVALID_ID;
    g_vpcm_pjsip_runtime.call_conf_port = -1;
    g_vpcm_pjsip_runtime.role = role;
    g_vpcm_pjsip_runtime.expected_law = law;

    st = pjsua_create();
    if (st != PJ_SUCCESS) {
        fprintf(stderr, "pjsua_create failed: %d\n", st);
        return false;
    }

    pjsua_config_default(&ua_cfg);
    pjsua_logging_config_default(&log_cfg);
    pjsua_media_config_default(&media_cfg);
    ua_cfg.max_calls = 4;
    ua_cfg.cb.on_incoming_call = &vpcm_pjsip_on_incoming_call;
    ua_cfg.cb.on_call_state = &vpcm_pjsip_on_call_state;
    ua_cfg.cb.on_call_media_state = &vpcm_pjsip_on_call_media_state;
    ua_cfg.cb.on_stream_created2 = &vpcm_pjsip_on_stream_created2;
    log_cfg.console_level = (g_vpcm_session_diag || g_vpcm_verbose) ? 4 : 3;
    media_cfg.no_vad = PJ_TRUE;
    media_cfg.enable_ice = PJ_FALSE;
    media_cfg.clock_rate = 8000;
    media_cfg.audio_frame_ptime = 20;
    media_cfg.channel_count = 1;

    st = pjsua_init(&ua_cfg, &log_cfg, &media_cfg);
    if (st != PJ_SUCCESS) {
        fprintf(stderr, "pjsua_init failed: %d\n", st);
        pjsua_destroy();
        return false;
    }

    pjsua_transport_config_default(&tp_cfg);
    if (local_port > 0)
        tp_cfg.port = (pj_uint16_t) local_port;
    st = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &tp_cfg, &tid);
    if (st != PJ_SUCCESS) {
        fprintf(stderr, "pjsua_transport_create(UDP) failed: %d\n", st);
        pjsua_destroy();
        return false;
    }

    st = pjsua_start();
    if (st != PJ_SUCCESS) {
        fprintf(stderr, "pjsua_start failed: %d\n", st);
        pjsua_destroy();
        return false;
    }

    pjsua_set_null_snd_dev();

    st = vpcm_pjsip_register_g711_passthrough(law);
    if (st != PJ_SUCCESS) {
        fprintf(stderr, "unable to register G.711 passthrough codec set: %d\n", st);
        pjsua_destroy();
        return false;
    }

    st = vpcm_pjsip_set_g711_only_priorities(law);
    if (st != PJ_SUCCESS) {
        fprintf(stderr, "unable to enforce %s-only codec policy\n", vpcm_expected_codec_name(law));
        pjsua_destroy();
        return false;
    }

    if (account_id_env && account_id_env[0] != '\0') {
        pjsua_acc_config_default(&acc_cfg);
        if (rtp_port > 0)
            acc_cfg.rtp_cfg.port = (unsigned) rtp_port;
        if (rtp_range > 0)
            acc_cfg.rtp_cfg.port_range = (unsigned) rtp_range;
        acc_cfg.id = pj_str((char *) account_id_env);
        if (reg_uri_env && reg_uri_env[0] != '\0')
            acc_cfg.reg_uri = pj_str((char *) reg_uri_env);
        if (auth_user_env && auth_user_env[0] != '\0'
            && auth_pass_env && auth_pass_env[0] != '\0') {
            acc_cfg.cred_count = 1;
            acc_cfg.cred_info[0].scheme = pj_str((char *) "digest");
            acc_cfg.cred_info[0].realm = pj_str((char *) ((auth_realm_env && auth_realm_env[0] != '\0') ? auth_realm_env : "*"));
            acc_cfg.cred_info[0].username = pj_str((char *) auth_user_env);
            acc_cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
            acc_cfg.cred_info[0].data = pj_str((char *) auth_pass_env);
        }
        st = pjsua_acc_add(&acc_cfg, PJ_TRUE, &acc_id);
    } else {
        pjsua_acc_config_default(&acc_cfg);
        if (rtp_port > 0)
            acc_cfg.rtp_cfg.port = (unsigned) rtp_port;
        if (rtp_range > 0)
            acc_cfg.rtp_cfg.port_range = (unsigned) rtp_range;
        acc_cfg.transport_id = tid;
        acc_cfg.allow_contact_rewrite = PJ_FALSE;
        acc_cfg.allow_via_rewrite = PJ_FALSE;
        acc_cfg.publish_enabled = PJ_FALSE;
        acc_cfg.register_on_acc_add = PJ_FALSE;
        acc_cfg.id = pj_str((char *) "sip:local@localhost");
        st = pjsua_acc_add_local(tid, PJ_TRUE, &acc_id);
    }
    if (st != PJ_SUCCESS) {
        fprintf(stderr, "pjsua account add failed: %d\n", st);
        pjsua_destroy();
        return false;
    }

    if (role == VPCM_PJSIP_ROLE_CALLER) {
        pjsua_call_setting_default(&call_opt);
        call_opt.aud_cnt = 1;
        call_opt.vid_cnt = 0;
        dst_uri = pj_str((char *) dest_uri_env);
        st = pjsua_call_make_call(acc_id, &dst_uri, &call_opt, NULL, NULL, &g_vpcm_pjsip_runtime.call_id);
        if (st != PJ_SUCCESS) {
            fprintf(stderr, "pjsua_call_make_call failed: %d\n", st);
            pjsua_destroy();
            return false;
        }
        vpcm_log("PJSIP caller started: caller=%s dest=%s law=%s codec_policy=%s-only passthrough",
                 (account_id_env && account_id_env[0] != '\0') ? account_id_env : "<local>",
                 dest_uri_env,
                 vpcm_law_to_str(law),
                 vpcm_expected_codec_name(law));
        if (rtp_port > 0) {
            vpcm_log("PJSIP RTP bind preference: port=%d range=%d", rtp_port, (rtp_range > 0) ? rtp_range : 0);
        }
    } else {
        vpcm_log("PJSIP answerer ready: account=%s wait=%ds law=%s codec_policy=%s-only passthrough",
                 (account_id_env && account_id_env[0] != '\0') ? account_id_env : "<local>",
                 wait_seconds,
                 vpcm_law_to_str(law),
                 vpcm_expected_codec_name(law));
        if (rtp_port > 0) {
            vpcm_log("PJSIP RTP bind preference: port=%d range=%d", rtp_port, (rtp_range > 0) ? rtp_range : 0);
        }
    }

    start_wait = vpcm_monotonic_seconds();
    wait_limit = (role == VPCM_PJSIP_ROLE_CALLER)
        ? (using_dummy_uri ? 5.0 : 30.0)
        : (double) wait_seconds;
    ok = false;
    wait_confirm_logged = false;
    while (!g_vpcm_stop_requested) {
        pjsua_handle_events(100);
        if (g_vpcm_pjsip_runtime.call_disconnected)
            break;
        if (g_vpcm_pjsip_runtime.call_confirmed
            && g_vpcm_pjsip_runtime.media_ready
            && g_vpcm_pjsip_runtime.media_codec_ok
            && g_vpcm_pjsip_runtime.stream_ready
            && g_vpcm_pjsip_runtime.stream_port) {
            ok = true;
            break;
        }
        if (!wait_confirm_logged
            && g_vpcm_pjsip_runtime.media_ready
            && !g_vpcm_pjsip_runtime.call_confirmed) {
            vpcm_log("PJSIP media is up but call is not CONFIRMED yet; delaying modem/V.8 start");
            wait_confirm_logged = true;
        }
        if ((vpcm_monotonic_seconds() - start_wait) > wait_limit)
            break;
    }

    if (ok)
        ok = vpcm_run_pjsip_payload_loop(law, data_seconds, seed);

    if (g_vpcm_pjsip_runtime.call_id != PJSUA_INVALID_ID
        && !g_vpcm_pjsip_runtime.call_disconnected) {
        pjsua_call_hangup(g_vpcm_pjsip_runtime.call_id, 0, NULL, NULL);
        start_wait = vpcm_monotonic_seconds();
        while (!g_vpcm_pjsip_runtime.call_disconnected
               && (vpcm_monotonic_seconds() - start_wait) < 5.0) {
            pjsua_handle_events(100);
        }
    }

    if (g_vpcm_pjsip_modem_lock) {
        pj_mutex_destroy(g_vpcm_pjsip_modem_lock);
        g_vpcm_pjsip_modem_lock = NULL;
    }
    if (g_vpcm_pjsip_modem_pool) {
        pj_pool_release(g_vpcm_pjsip_modem_pool);
        g_vpcm_pjsip_modem_pool = NULL;
    }

    pjsua_destroy();

    if (!ok) {
        if (g_vpcm_pjsip_runtime.call_disconnected) {
            fprintf(stderr, "PJSIP call ended before payload completion (status=%d)\n",
                    g_vpcm_pjsip_runtime.disconnect_code);
        } else if (!g_vpcm_pjsip_runtime.call_confirmed) {
            fprintf(stderr, "PJSIP call never reached CONFIRMED state before timeout\n");
        } else if (!g_vpcm_pjsip_runtime.stream_ready || !g_vpcm_pjsip_runtime.stream_port) {
            fprintf(stderr, "PJSIP media stream was not ready for payload transport\n");
        } else if (!g_vpcm_pjsip_runtime.media_codec_checked) {
            fprintf(stderr, "PJSIP media was never established/checked\n");
        } else {
            fprintf(stderr, "PJSIP media codec policy check failed\n");
        }
    } else {
        vpcm_log("PASS: V.91 pj-sip payload E2E role=%s (%s only, duration=%d s)",
                 vpcm_pjsip_role_to_str(role),
                 vpcm_expected_codec_name(law),
                 data_seconds > 0 ? data_seconds : 0);
    }

    return ok;
}

static bool run_v91_single_e2e_call(v91_law_t law, uint32_t seed, int data_seconds)
{
    v8_parms_t caller_parms;
    v8_parms_t answer_parms;
    vpcm_v8_result_t caller_result;
    vpcm_v8_result_t answer_result;
    vpcm_cp_frame_t cp_offer;

    if (g_vpcm_transport_backend == VPCM_TRANSPORT_PJ_SIP) {
        return run_v91_single_e2e_call_pjsip(law, data_seconds, seed);
    }

    init_v8_parms(&caller_parms, true, V8_MOD_V22 | V8_MOD_V34, V8_PSTN_PCM_MODEM_V91);
    init_v8_parms(&answer_parms, false, V8_MOD_V22 | V8_MOD_V34, V8_PSTN_PCM_MODEM_V91);
    if (!run_v8_exchange(law, &caller_parms, &answer_parms, &caller_result, &answer_result))
        return false;

    vpcm_cp_init_robbed_bit_safe_profile(&cp_offer, vpcm_cp_recommended_robbed_bit_drn(), false);
    return run_vpcm_full_phase_session(law,
                                       "V.91 single-call E2E",
                                       "robbed-bit-safe profile",
                                       &cp_offer,
                                       true,
                                       seed,
                                       data_seconds);
}

static int run_v91_e2e_mode(v91_law_t law, int data_seconds)
{
    double start_wall;
    bool ok;
    uint32_t seed;

    seed = 0x09100000U ^ (uint32_t) law;
    if (data_seconds <= 0)
        data_seconds = 10;

    signal(SIGINT, vpcm_handle_sigint);
    signal(SIGTERM, vpcm_handle_sigint);
    g_vpcm_compact_e2e = true;
    start_wall = vpcm_monotonic_seconds();

    vpcm_log("V.91 E2E mode starting (law=%s, transport=%s, realtime=%s, data_seconds=%d)",
             vpcm_law_to_str(law),
             (g_vpcm_transport_backend == VPCM_TRANSPORT_LOOPBACK) ? "loopback" : "pj-sip",
             g_vpcm_realtime ? "on" : "off",
             data_seconds);

    if (g_vpcm_stop_requested) {
        vpcm_log("V.91 E2E finished: reason=cancelled before call start elapsed=%.1fs",
                 vpcm_monotonic_seconds() - start_wall);
        g_vpcm_compact_e2e = false;
        return 0;
    }

    vpcm_log("V.91 E2E call starting");
    ok = run_v91_single_e2e_call(law, seed, data_seconds);
    if (!ok) {
        vpcm_log("V.91 E2E finished: reason=call collapsed elapsed=%.1fs",
                 vpcm_monotonic_seconds() - start_wall);
        g_vpcm_compact_e2e = false;
        return 1;
    }

    if (g_vpcm_stop_requested) {
        vpcm_log("V.91 E2E finished: reason=cancelled elapsed=%.1fs",
                 vpcm_monotonic_seconds() - start_wall);
    } else {
        vpcm_log("V.91 E2E finished: reason=data duration reached elapsed=%.1fs",
                 vpcm_monotonic_seconds() - start_wall);
    }
    g_vpcm_compact_e2e = false;
    return 0;
}

int main(int argc, char **argv)
{
    const char *verbose_env;
    bool run_sessions;
    bool run_primitives;
    bool run_v91_e2e_call;
    int v91_e2e_seconds;
    v91_law_t v91_e2e_law;
    int i;

    verbose_env = getenv("VPCM_VERBOSE");
    g_vpcm_verbose = (verbose_env != NULL && verbose_env[0] != '\0' && strcmp(verbose_env, "0") != 0);
    run_sessions = true;
    run_primitives = false;
    run_v91_e2e_call = false;
    v91_e2e_seconds = 0;
    v91_e2e_law = V91_LAW_ULAW;
    g_vpcm_stop_requested = 0;

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--all-tests") == 0) {
            run_sessions = true;
            run_primitives = true;
        } else if (strcmp(argv[i], "--primitive-tests") == 0) {
            run_sessions = false;
            run_primitives = true;
        } else if (strcmp(argv[i], "--session-only") == 0) {
            run_sessions = true;
            run_primitives = false;
        } else if (strcmp(argv[i], "--session-diag") == 0) {
            g_vpcm_session_diag = true;
        } else if (strcmp(argv[i], "--experimental-v90-info") == 0) {
            g_vpcm_experimental_v90_info = true;
        } else if (strcmp(argv[i], "--realtime") == 0) {
            g_vpcm_realtime = true;
        } else if (strcmp(argv[i], "--transport") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "--transport requires one of: loopback, pj-sip\n");
                return 2;
            }
            i++;
            if (strcmp(argv[i], "loopback") == 0) {
                g_vpcm_transport_backend = VPCM_TRANSPORT_LOOPBACK;
            } else if (strcmp(argv[i], "pj-sip") == 0) {
                g_vpcm_transport_backend = VPCM_TRANSPORT_PJ_SIP;
            } else {
                fprintf(stderr, "Unknown transport backend: %s\n", argv[i]);
                return 2;
            }
        } else if (strcmp(argv[i], "--v91-e2e-call") == 0) {
            run_v91_e2e_call = true;
            run_sessions = false;
            run_primitives = false;
        } else if (strcmp(argv[i], "--v91-e2e-seconds") == 0) {
            char *end = NULL;
            long v;
            if (i + 1 >= argc) {
                fprintf(stderr, "--v91-e2e-seconds requires an integer argument\n");
                return 2;
            }
            i++;
            v = strtol(argv[i], &end, 10);
            if (end == NULL || *end != '\0' || v < 0 || v > 86400) {
                fprintf(stderr, "Invalid --v91-e2e-seconds value: %s (expected 0..86400)\n", argv[i]);
                return 2;
            }
            v91_e2e_seconds = (int) v;
        } else if (strcmp(argv[i], "--v91-e2e-law") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "--v91-e2e-law requires one of: ulaw, alaw\n");
                return 2;
            }
            i++;
            if (strcmp(argv[i], "ulaw") == 0) {
                v91_e2e_law = V91_LAW_ULAW;
            } else if (strcmp(argv[i], "alaw") == 0) {
                v91_e2e_law = V91_LAW_ALAW;
            } else {
                fprintf(stderr, "Unknown --v91-e2e-law value: %s\n", argv[i]);
                return 2;
            }
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [--session-only] [--primitive-tests] [--all-tests] [--session-diag] [--experimental-v90-info]\n", argv[0]);
            printf("  default           Run call-oriented end-to-end session tests only.\n");
            printf("  --primitive-tests Add focused primitive/component tests.\n");
            printf("  --all-tests       Run both session and primitive suites.\n");
            printf("  --session-diag    Emit INFO/CP diagnostic tables during session tests.\n");
            printf("  --experimental-v90-info  Run the real SpanDSP V.90 INFO startup harness.\n");
            printf("  --realtime        Pace G.711 transport at 8kHz wall clock instead of CPU speed.\n");
            printf("  --transport <loopback|pj-sip> Select test transport backend (default: loopback).\n");
            printf("                   pj-sip mode requires PJMEDIA passthrough and G.711 only.\n");
            printf("                   Env: VPCM_PJSIP_ROLE=<caller|answerer> (default caller),\n");
            printf("                        VPCM_PJSIP_DEST_URI (caller), VPCM_PJSIP_LOCAL_PORT,\n");
            printf("                        VPCM_PJSIP_ACCOUNT_ID, VPCM_PJSIP_REG_URI,\n");
            printf("                        VPCM_PJSIP_AUTH_USER, VPCM_PJSIP_AUTH_PASS, VPCM_PJSIP_AUTH_REALM.\n");
            printf("                        Optional dialplan inputs: VPCM_PJSIP_CALLER_NUMBER,\n");
            printf("                        VPCM_PJSIP_ANSWERER_NUMBER, VPCM_PJSIP_DOMAIN,\n");
            printf("                        VPCM_PJSIP_DUMMY_URI (caller default sip:dummy@127.0.0.1),\n");
            printf("                        VPCM_PJSIP_WAIT_SECONDS (answerer wait timeout, default 60).\n");
            printf("  --v91-e2e-call    Run V.91 single-call end-to-end mode (focus mode).\n");
            printf("  --v91-e2e-seconds <n> V.91 data transmit duration in seconds (default: 10).\n");
            printf("  --v91-e2e-law <ulaw|alaw> Set V.91 E2E law (default: ulaw).\n");
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 2;
        }
    }

    vpcm_log("PCM modem loopback harness starting (verbose=%s, session_diag=%s, sessions=%s, primitives=%s, experimental_v90_info=%s, realtime=%s, transport=%s, v91_e2e=%s)",
             g_vpcm_verbose ? "on" : "off",
             g_vpcm_session_diag ? "on" : "off",
             run_sessions ? "on" : "off",
             run_primitives ? "on" : "off",
             g_vpcm_experimental_v90_info ? "on" : "off",
             g_vpcm_realtime ? "on" : "off",
             (g_vpcm_transport_backend == VPCM_TRANSPORT_LOOPBACK) ? "loopback" : "pj-sip",
             run_v91_e2e_call ? "on" : "off");

    if (run_v91_e2e_call) {
        return run_v91_e2e_mode(v91_e2e_law, v91_e2e_seconds);
    }

    if (run_sessions && !run_vpcm_session_suite())
        return 1;
    if (run_primitives && !run_vpcm_primitive_suite())
        return 1;

    vpcm_log("All PCM modem loopback tests passed");
    puts("vpcm_loopback_test: OK");
    return 0;
}
