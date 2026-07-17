/*
 * sip_modem.c — PJSIP SIP user agent + G.711 media port
 *
 * Initialises pjsua as a minimal SIP UA:
 *   - Null audio device (audio handled entirely by this process)
 *   - G.711 a/µ-law (PCMA/U/8000) only; all other codecs disabled
 *   - Directly talks PCMA/U as transcoding will break V.90
 *   - Custom pjmedia_port that routes PCM samples to/from modem_engine
 *   - Detects ME_DIALING state and places the outgoing SIP call
 *   - Detects ME_HANGUP state and tears down the call
 *
 * Usage:
 *   ./sip_v90_modem [--sip-server <host>] [--username <user>]
 *                   [--password <pass>]  [--pty-link <path>]
 *                   [--local-port <port>]
 *
 * If --sip-server is omitted the UA starts in peer-to-peer mode and
 * waits for incoming calls on the specified local SIP port (default 5060).
 */

#include "modem_engine.h"
#include "data_interface.h"

#include <pjsua-lib/pjsua.h>
#include <pjmedia-codec/passthrough.h>
#include <pjmedia-audiodev/audiodev.h>
#include <pjmedia/frame.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

#define SAMPLE_RATE     8000    /* G.711 / V.90 sample rate */
#define SAMPLES_PER_FRAME 160   /* 20 ms frame at 8000 Hz */

/* ------------------------------------------------------------------ */
/* Global state                                                        */
/* ------------------------------------------------------------------ */

static pj_pool_t       *g_pool       = NULL;
static pjsua_acc_id     g_acc_id     = PJSUA_INVALID_ID;
static pjsua_call_id    g_call_id    = PJSUA_INVALID_ID;
static volatile int     g_running    = 1;
static pj_bool_t        g_media_connected = PJ_FALSE;
static pj_bool_t        g_passthrough_enabled = PJ_FALSE;

typedef struct modem_passthrough_port_s {
    pjmedia_port base;
    pj_pool_t *pool;
    pjmedia_port *downstream_port;
    unsigned payload_samples_per_frame;
    uint8_t tx_payload[320];
    uint8_t tx_ext_buf[2048];
    uint8_t rx_ext_buf[2048];
} modem_passthrough_port_t;

static int16_t g_tx_linear[SAMPLES_PER_FRAME * 2];
static me_state_t g_last_logged_me_state = ME_IDLE;
static int g_last_logged_media_connected = 0;

static void log_modem_diag_snapshot(const char *reason);

/* Ring state for incoming calls — emulates S0 register auto-answer */
#define RING_INTERVAL_MS    6000    /* 6 seconds between rings (realistic cadence) */
#define AUTO_ANSWER_RINGS   2       /* Answer after this many rings */
static pjsua_call_id   g_ringing_call = PJSUA_INVALID_ID;
static int             g_ring_count   = 0;
static pj_time_val     g_last_ring_time;

/* ------------------------------------------------------------------ */
/* Raw G.711 passthrough bridge between PJSIP and modem_engine        */
/* ------------------------------------------------------------------ */

static void clamp_linear_payload_samples(unsigned *count, unsigned max)
{
    if (*count == 0 || *count > max)
        *count = max;
}

static pj_status_t modem_passthrough_put_frame(pjmedia_port *this_port,
                                               pjmedia_frame *frame)
{
    modem_passthrough_port_t *port = (modem_passthrough_port_t *) this_port;
    pjmedia_frame_ext *tx_ext;
    pjmedia_frame audio_frame;
    unsigned count;

    PJ_UNUSED_ARG(frame);

    if (!port || !port->downstream_port)
        return PJ_EINVAL;

    count = port->payload_samples_per_frame;
    clamp_linear_payload_samples(&count, (unsigned) PJ_ARRAY_SIZE(g_tx_linear));

    if (!g_passthrough_enabled) {
        me_tx_audio(g_tx_linear, (int) count);
        audio_frame.type = PJMEDIA_FRAME_TYPE_AUDIO;
        audio_frame.buf = g_tx_linear;
        audio_frame.size = count * sizeof(int16_t);
        audio_frame.timestamp.u64 = 0;
        audio_frame.bit_info = 0;
        return pjmedia_port_put_frame(port->downstream_port, &audio_frame);
    }

    if (me_tx_g711(port->tx_payload, (int)count) != (int)count)
        return PJ_EBUG;

    memset(port->tx_ext_buf, 0, sizeof(port->tx_ext_buf));
    tx_ext = (pjmedia_frame_ext *) port->tx_ext_buf;
    tx_ext->base.type = PJMEDIA_FRAME_TYPE_EXTENDED;
    pjmedia_frame_ext_append_subframe(tx_ext,
                                      port->tx_payload,
                                      port->payload_samples_per_frame * 8U,
                                      (pj_uint16_t) port->payload_samples_per_frame);

    return pjmedia_port_put_frame(port->downstream_port, (pjmedia_frame *) tx_ext);
}

static pj_status_t modem_passthrough_get_frame(pjmedia_port *this_port,
                                               pjmedia_frame *frame)
{
    modem_passthrough_port_t *port = (modem_passthrough_port_t *) this_port;
    pj_status_t st;
    pjmedia_frame_ext *rx_ext;
    unsigned i;

    if (!port || !frame || !port->downstream_port)
        return PJ_EINVAL;

    if (!g_passthrough_enabled) {
        pjmedia_frame audio_in;
        unsigned byte_count;
        unsigned sample_count;

        pj_bzero(&audio_in, sizeof(audio_in));
        audio_in.type = PJMEDIA_FRAME_TYPE_AUDIO;
        audio_in.buf = g_tx_linear;
        audio_in.size = sizeof(g_tx_linear);

        st = pjmedia_port_get_frame(port->downstream_port, &audio_in);
        if (st == PJ_SUCCESS && audio_in.type == PJMEDIA_FRAME_TYPE_AUDIO
            && audio_in.buf && audio_in.size >= sizeof(int16_t)) {
            byte_count = (unsigned) audio_in.size;
            if (byte_count > sizeof(g_tx_linear))
                byte_count = sizeof(g_tx_linear);
            sample_count = byte_count / sizeof(int16_t);
            if (sample_count > 0)
                me_rx_audio((const int16_t *) audio_in.buf, (int) sample_count);
        }

        frame->type = PJMEDIA_FRAME_TYPE_NONE;
        frame->size = 0;
        return PJ_SUCCESS;
    }

    rx_ext = (pjmedia_frame_ext *) port->rx_ext_buf;
    pj_bzero(rx_ext, sizeof(pjmedia_frame_ext));

    st = pjmedia_port_get_frame(port->downstream_port, (pjmedia_frame *) rx_ext);
    if (st == PJ_SUCCESS && rx_ext->base.type == PJMEDIA_FRAME_TYPE_EXTENDED) {
        for (i = 0; i < rx_ext->subframe_cnt; i++) {
            pjmedia_frame_ext_subframe *sf = pjmedia_frame_ext_get_subframe(rx_ext, i);
            unsigned sf_bytes;
            if (!sf)
                continue;
            sf_bytes = ((unsigned) sf->bitlen + 7U) >> 3;
            if (sf_bytes == 0 || sf_bytes > PJ_ARRAY_SIZE(g_tx_linear))
                continue;
            me_rx_g711((const uint8_t *)sf->data, (int)sf_bytes);
        }
    } else if (st == PJ_SUCCESS && rx_ext->base.type == PJMEDIA_FRAME_TYPE_AUDIO
               && rx_ext->base.buf && rx_ext->base.size > 0) {
        unsigned sz = (unsigned) rx_ext->base.size;
        if (sz > PJ_ARRAY_SIZE(g_tx_linear))
            sz = PJ_ARRAY_SIZE(g_tx_linear);
        me_rx_g711((const uint8_t *)rx_ext->base.buf, (int)sz);
    }

    frame->type = PJMEDIA_FRAME_TYPE_NONE;
    frame->size = 0;
    return PJ_SUCCESS;
}

static pj_status_t modem_passthrough_on_destroy(pjmedia_port *this_port)
{
    modem_passthrough_port_t *port = (modem_passthrough_port_t *) this_port;
    if (port && port->pool)
        pj_pool_release(port->pool);
    return PJ_SUCCESS;
}

static pj_status_t modem_passthrough_port_create(const pjmedia_port *source_port,
                                                 unsigned payload_samples_per_frame,
                                                 pjmedia_port **p_port)
{
    pjmedia_endpt *endpt = pjsua_get_pjmedia_endpt();
    pj_pool_t *pool;
    modem_passthrough_port_t *port;

    if (!source_port || !p_port || !endpt)
        return PJ_EINVAL;
    if (payload_samples_per_frame == 0 || payload_samples_per_frame > 320)
        return PJ_EINVAL;

    pool = pjmedia_endpt_create_pool(endpt, "modem-pass-port", 1024, 1024);
    if (!pool)
        return PJ_ENOMEM;

    port = PJ_POOL_ZALLOC_T(pool, modem_passthrough_port_t);
    port->pool = pool;
    port->downstream_port = (pjmedia_port *) source_port;
    port->payload_samples_per_frame = payload_samples_per_frame;
    port->base.info = source_port->info;
    port->base.put_frame = &modem_passthrough_put_frame;
    port->base.get_frame = &modem_passthrough_get_frame;
    port->base.on_destroy = &modem_passthrough_on_destroy;
    *p_port = &port->base;
    return PJ_SUCCESS;
}

static void on_stream_created2(pjsua_call_id call_id,
                               pjsua_on_stream_created_param *param)
{
    pjmedia_port *pass_port = NULL;
    unsigned payload_samples_per_frame = SAMPLES_PER_FRAME;

    if (!param || call_id != g_call_id || param->stream_idx != 0)
        return;

    if (param->port
        && param->port->info.fmt.type == PJMEDIA_TYPE_AUDIO
        && param->port->info.fmt.detail_type == PJMEDIA_FORMAT_DETAIL_AUDIO
        && PJMEDIA_PIA_SPF(&param->port->info) > 0
        && PJMEDIA_PIA_SPF(&param->port->info) <= 320) {
        payload_samples_per_frame = PJMEDIA_PIA_SPF(&param->port->info);
    }

    if (param->port
        && modem_passthrough_port_create(param->port,
                                         payload_samples_per_frame,
                                         &pass_port) == PJ_SUCCESS) {
        param->port = pass_port;
        param->destroy_port = PJ_TRUE;
        PJ_LOG(3, ("sip_modem", "Installed G.711 passthrough stream wrapper (%u samples/frame)",
                   payload_samples_per_frame));
    }
}

/* ------------------------------------------------------------------ */
/* PJSUA callbacks                                                     */
/* ------------------------------------------------------------------ */

static void on_call_state(pjsua_call_id call_id, pjsip_event *e)
{
    (void)e;
    pjsua_call_info ci;
    pjsua_call_get_info(call_id, &ci);

    PJ_LOG(3, ("sip_modem", "Call %d state: %.*s",
               call_id, (int)ci.state_text.slen, ci.state_text.ptr));

    if (ci.state == PJSIP_INV_STATE_DISCONNECTED) {
        /* Cancel ringing if the caller hung up before we answered */
        if (call_id == g_ringing_call) {
            PJ_LOG(3, ("sip_modem", "Caller hung up during ringing"));
            g_ringing_call = PJSUA_INVALID_ID;
            g_ring_count   = 0;
        }
        if (call_id == g_call_id) {
            if (g_media_connected) {
                me_on_sip_disconnected();
                g_media_connected = PJ_FALSE;
            }
            g_call_id = PJSUA_INVALID_ID;
        }
    }
}

static void on_call_media_state(pjsua_call_id call_id)
{
    pjsua_call_info ci;
    pjsua_call_get_info(call_id, &ci);

    for (unsigned i = 0; i < ci.media_cnt; i++) {
        pjsua_stream_info si;

        if (ci.media[i].type == PJMEDIA_TYPE_AUDIO &&
            ci.media[i].status == PJSUA_CALL_MEDIA_ACTIVE) {
            pjsua_conf_port_id conf_port;

            g_call_id = call_id;
            if (pjsua_call_get_stream_info(call_id, i, &si) == PJ_SUCCESS &&
                si.type == PJMEDIA_TYPE_AUDIO) {
                pj_str_t pcma_name = pj_str("PCMA");
                if (pj_stricmp(&si.info.aud.fmt.encoding_name, &pcma_name) == 0) {
                    me_set_law(ME_LAW_ALAW);
                    PJ_LOG(3, ("sip_modem", "Codec: PCMA (A-law passthrough)"));
                } else {
                    me_set_law(ME_LAW_ULAW);
                    PJ_LOG(3, ("sip_modem", "Codec: PCMU (u-law passthrough)"));
                }
            }
            /* The null sound device supplies the media clock, but PJSUA does
             * not connect a call to it automatically.  Clock both directions
             * so the passthrough port's get/put callbacks actually run. */
            conf_port = pjsua_call_get_conf_port(call_id);
            if (conf_port != PJSUA_INVALID_ID) {
                pj_status_t tx_st = pjsua_conf_connect(0, conf_port);
                pj_status_t rx_st = pjsua_conf_connect(conf_port, 0);

                if (tx_st != PJ_SUCCESS || rx_st != PJ_SUCCESS)
                    PJ_LOG(2, ("sip_modem", "Media clock wiring failed: conf=%d tx=%d rx=%d",
                               conf_port, tx_st, rx_st));
                else
                    PJ_LOG(3, ("sip_modem", "Media clock wired: conf %d <-> 0", conf_port));
            }
            if (!g_media_connected) {
                me_on_sip_connected();
                g_media_connected = PJ_TRUE;
                g_last_logged_media_connected = 1;
                log_modem_diag_snapshot("media-connected");
            }
            break;
        }
    }
}

static void log_modem_diag_snapshot(const char *reason)
{
    me_diag_snapshot_t snapshot;

    me_get_diag_snapshot(&snapshot);
    PJ_LOG(
        3,
        ("sip_modem",
         "ME trace (%s): state=%s mod=%s law=%s role=%s media=%s phase_ms=%llu "
         "v34_rx=%d v34_tx=%d v90_rx=%d v90_tx=%d v90_event=%d phase3=%d s_events=%d dil=%d "
         "cp_bits=%llu cp_valid=%u cp_rejected=%u "
         "v92=%d trn2u=%d trn2u_symbols=%llu trn2u_ones=%u cpu_bits=%llu cpu_valid=%u cpu_rejected=%u "
         "g711_rx=%llu g711_tx=%llu raw_v90_tx=%llu linear_tx=%llu",
         reason,
         me_state_to_str(snapshot.state),
         me_modulation_to_str(snapshot.modulation),
         me_law_to_str(snapshot.law),
         snapshot.calling_party ? "caller" : "answerer",
         g_media_connected ? "up" : "down",
         (unsigned long long) snapshot.phase_elapsed_ms,
         snapshot.v34_rx_stage,
         snapshot.v34_tx_stage,
         snapshot.v90_bridge_rx_stage,
         snapshot.v90_bridge_tx_stage,
         snapshot.v90_bridge_rx_event,
         snapshot.v90_phase3_started,
         snapshot.v90_phase3_s_events,
         snapshot.v90_dil_valid,
         (unsigned long long)snapshot.v90_cp_input_bits,
         snapshot.v90_cp_valid_frames,
         snapshot.v90_cp_rejected_frames,
         snapshot.v92_active,
         snapshot.v92_trn2u_active,
         (unsigned long long)snapshot.v92_trn2u_symbols,
         snapshot.v92_trn2u_longest_ones,
         (unsigned long long)snapshot.v92_cp_input_bits,
         snapshot.v92_cp_valid_frames,
         snapshot.v92_cp_rejected_frames,
         (unsigned long long)snapshot.g711_rx_octets,
         (unsigned long long)snapshot.g711_tx_octets,
         (unsigned long long)snapshot.g711_raw_v90_tx_octets,
         (unsigned long long)snapshot.g711_linear_tx_octets));
}

static void on_incoming_call(pjsua_acc_id acc_id, pjsua_call_id call_id,
                              pjsip_rx_data *rdata)
{
    (void)acc_id; (void)rdata;
    pjsua_call_info ci;
    pjsua_call_get_info(call_id, &ci);

    PJ_LOG(3, ("sip_modem", "Incoming call from %.*s",
               (int)ci.remote_info.slen, ci.remote_info.ptr));

    /* Send 180 Ringing to the caller — don't answer yet */
    pjsua_call_answer(call_id, 180, NULL, NULL);

    /* Start the ring sequence */
    g_ringing_call = call_id;
    g_ring_count   = 1;
    pj_gettimeofday(&g_last_ring_time);

    /* First RING to the PTY */
    di_on_ring();
    PJ_LOG(3, ("sip_modem", "RING 1/%d", AUTO_ANSWER_RINGS));
}

/* ------------------------------------------------------------------ */
/* Codec setup — G.711 µ-law only                                      */
/* ------------------------------------------------------------------ */

static void restrict_to_g711(void)
{
    pj_str_t pcmu = pj_str("PCMU/8000");
    pj_str_t pcma = pj_str("PCMA/8000");
    const char *force_pcmu = getenv("SIP_FORCE_PCMU");

    /* Raise PCMU to highest priority */
    pjsua_codec_set_priority(&pcmu, PJMEDIA_CODEC_PRIO_HIGHEST);
    /* Allow PCMA as second choice (some ATAs only offer A-law) */
    pjsua_codec_set_priority(&pcma,
                             (force_pcmu && force_pcmu[0] && strcmp(force_pcmu, "0") != 0)
                                 ? PJMEDIA_CODEC_PRIO_DISABLED
                                 : PJMEDIA_CODEC_PRIO_NEXT_HIGHER);

    /* Disable all other codecs */
    const char *disable[] = {
        "speex/8000", "speex/16000", "speex/32000",
        "iLBC/8000",  "GSM/8000",    "G722/16000",
        "G7221/16000","G7221/32000", "opus/48000",
        NULL
    };
    for (int i = 0; disable[i]; i++) {
        pj_str_t id = pj_str((char *)disable[i]);
        pjsua_codec_set_priority(&id, PJMEDIA_CODEC_PRIO_DISABLED);
    }
}

static pj_status_t register_g711_passthrough(void)
{
#if PJMEDIA_HAS_PASSTHROUGH_CODECS
    pjmedia_codec_passthrough_setting setting;
    pjmedia_format fmts[2];
#endif

#if !PJMEDIA_HAS_PASSTHROUGH_CODECS
    g_passthrough_enabled = PJ_FALSE;
    return PJ_ENOTSUP;
#else
    memset(&setting, 0, sizeof(setting));
    memset(fmts, 0, sizeof(fmts));

    pjmedia_format_init_audio(&fmts[0], PJMEDIA_FORMAT_PCMU, 8000, 1, 8, 20000, 64000, 64000);
    pjmedia_format_init_audio(&fmts[1], PJMEDIA_FORMAT_PCMA, 8000, 1, 8, 20000, 64000, 64000);

    setting.fmt_cnt = 2;
    setting.fmts = fmts;
    setting.ilbc_mode = 20;

    pjmedia_codec_passthrough_deinit();
    if (pjmedia_codec_passthrough_init2(pjsua_get_pjmedia_endpt(), &setting) == PJ_SUCCESS) {
        g_passthrough_enabled = PJ_TRUE;
        return PJ_SUCCESS;
    }
    g_passthrough_enabled = PJ_FALSE;
    return PJ_ENOTSUP;
#endif
}

/* ------------------------------------------------------------------ */
/* Modem engine callbacks (called from data_interface)                 */
/* ------------------------------------------------------------------ */

static void on_dial(const char *number, void *user_data)
{
    (void)user_data;
    /* ATD received: store URI in modem engine, let the poll loop call pjsua */
    me_dial(number);
}

static void on_answer(void *user_data)
{
    (void)user_data;
    if (g_call_id != PJSUA_INVALID_ID)
        pjsua_call_answer(g_call_id, 200, NULL, NULL);
}

static void on_hangup(void *user_data)
{
    (void)user_data;
    me_hangup();
}

/* ------------------------------------------------------------------ */
/* Signal handler for clean shutdown                                   */
/* ------------------------------------------------------------------ */

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    const char *sip_server  = NULL;
    const char *username    = NULL;
    const char *password    = NULL;
    const char *pty_link    = "/tmp/modem0";
    int         local_port  = 5060;
    pj_bool_t   aud_subsys_inited = PJ_FALSE;

    /* Parse command-line arguments */
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--sip-server") && i+1 < argc)
            sip_server = argv[++i];
        else if (!strcmp(argv[i], "--username") && i+1 < argc)
            username = argv[++i];
        else if (!strcmp(argv[i], "--password") && i+1 < argc)
            password = argv[++i];
        else if (!strcmp(argv[i], "--pty-link") && i+1 < argc)
            pty_link = argv[++i];
        else if (!strcmp(argv[i], "--local-port") && i+1 < argc)
            local_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--verbose") || !strcmp(argv[i], "-v"))
            me_set_verbose(1);
        else if (!strcmp(argv[i], "--help")) {
            fprintf(stderr,
                "Usage: %s [--sip-server host] [--username u] [--password p]\n"
                "          [--pty-link path] [--local-port port] [--verbose]\n", argv[0]);
            return 0;
        }
    }

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── Initialise modem engine ──────────────────────────────────── */
    me_init();

    /* ── Initialise PJSUA ────────────────────────────────────────── */
    pj_status_t status = pjsua_create();
    if (status != PJ_SUCCESS) {
        PJ_LOG(1, ("sip_modem", "pjsua_create failed"));
        me_destroy();
        return 1;
    }

    status = pjmedia_aud_subsys_init(pjsua_get_pool_factory());
    if (status != PJ_SUCCESS) {
        PJ_LOG(2, ("sip_modem", "Audio subsystem init failed (continuing): %d", status));
    } else {
        aud_subsys_inited = PJ_TRUE;
    }

    /* UA config */
    pjsua_config ua_cfg;
    pjsua_config_default(&ua_cfg);
    ua_cfg.cb.on_call_state       = on_call_state;
    ua_cfg.cb.on_call_media_state = on_call_media_state;
    ua_cfg.cb.on_incoming_call    = on_incoming_call;
    ua_cfg.cb.on_stream_created2  = on_stream_created2;
    ua_cfg.max_calls              = 1;

    /* Logging */
    pjsua_logging_config log_cfg;
    pjsua_logging_config_default(&log_cfg);
    log_cfg.console_level = 3;
    {
        /* SIP_LOG_LEVEL=5 exposes SIP message traces and spandsp FLOW logs
           without a rebuild — used by the interop debugging workflow. */
        const char *lvl = getenv("SIP_LOG_LEVEL");
        if (lvl && *lvl)
            log_cfg.console_level = atoi(lvl);
    }

    /* Media config — disable all AEC, AGC, NR (modem signals must be clean) */
    pjsua_media_config media_cfg;
    pjsua_media_config_default(&media_cfg);
    media_cfg.clock_rate      = SAMPLE_RATE;
    media_cfg.channel_count   = 1;
    media_cfg.audio_frame_ptime = 20;   /* 20 ms frames */
    media_cfg.enable_ice      = PJ_FALSE;
    media_cfg.ec_tail_len     = 0;      /* No echo cancellation */
    media_cfg.no_vad          = PJ_TRUE; /* No voice activity detection */
    /* For modem pass-through, adaptive jitter buffering can destroy
       phase continuity needed by V.34 training. Keep playout fixed. */
    media_cfg.jb_init         = 0;
    media_cfg.jb_min_pre      = 0;
    media_cfg.jb_max_pre      = 0;
    media_cfg.jb_max          = 0;

    status = pjsua_init(&ua_cfg, &log_cfg, &media_cfg);
    if (status != PJ_SUCCESS) {
        PJ_LOG(1, ("sip_modem", "pjsua_init failed"));
        if (aud_subsys_inited)
            pjmedia_aud_subsys_shutdown();
        pjsua_destroy();
        me_destroy();
        return 1;
    }

    /* Memory pool */
    g_pool = pjsua_pool_create("sip_modem", 4000, 4000);

    /* Transport */
    pjsua_transport_config trans_cfg;
    pjsua_transport_config_default(&trans_cfg);
    trans_cfg.port = (unsigned)local_port;
    status = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &trans_cfg, NULL);
    if (status != PJ_SUCCESS) {
        PJ_LOG(1, ("sip_modem", "Transport create failed"));
        if (aud_subsys_inited)
            pjmedia_aud_subsys_shutdown();
        pjsua_destroy();
        me_destroy();
        return 1;
    }

    /* Use null sound device — audio is handled by our media port */
    pjsua_set_null_snd_dev();

    status = pjsua_start();
    if (status != PJ_SUCCESS) {
        PJ_LOG(1, ("sip_modem", "pjsua_start failed"));
        if (aud_subsys_inited)
            pjmedia_aud_subsys_shutdown();
        pjsua_destroy();
        me_destroy();
        return 1;
    }

    status = register_g711_passthrough();
    if (status != PJ_SUCCESS) {
        PJ_LOG(2, ("sip_modem",
                   "G.711 passthrough unavailable; falling back to linear PCM bridge for softmodem debugging"));
    } else {
        PJ_LOG(3, ("sip_modem", "G.711 passthrough enabled"));
    }

    /* Restrict to G.711 */
    restrict_to_g711();

    /* ── Initialise PTY/AT interface after PJSUA media init ─────── */
    di_set_callbacks(on_dial, on_answer, on_hangup, NULL);
    if (di_open(pty_link) < 0) {
        fprintf(stderr, "Failed to open PTY\n");
        if (aud_subsys_inited)
            pjmedia_aud_subsys_shutdown();
        pjsua_destroy();
        me_destroy();
        return 1;
    }

    /* ── Optional SIP account registration ──────────────────────── */
    if (sip_server && username) {
        pjsua_acc_config acc_cfg;
        pjsua_acc_config_default(&acc_cfg);

        char id_buf[256], reg_buf[256];
        snprintf(id_buf,  sizeof(id_buf),  "sip:%s@%s", username, sip_server);
        snprintf(reg_buf, sizeof(reg_buf), "sip:%s",     sip_server);

        acc_cfg.id            = pj_str(id_buf);
        acc_cfg.reg_uri       = pj_str(reg_buf);
        acc_cfg.cred_count    = 1;
        acc_cfg.cred_info[0].realm  = pj_str("*");
        acc_cfg.cred_info[0].scheme = pj_str("digest");
        acc_cfg.cred_info[0].username = pj_str((char *)username);
        acc_cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
        acc_cfg.cred_info[0].data      = pj_str((char *)password ? (char *)password : "");

        status = pjsua_acc_add(&acc_cfg, PJ_TRUE, &g_acc_id);
        if (status != PJ_SUCCESS)
            PJ_LOG(2, ("sip_modem", "Account registration failed (continuing)"));
    } else {
        /* Create a local-only account for peer-to-peer calls */
        pjsua_acc_config acc_cfg;
        pjsua_acc_config_default(&acc_cfg);
        char id_buf[64];
        snprintf(id_buf, sizeof(id_buf), "sip:modem@127.0.0.1:%d", local_port);
        acc_cfg.id = pj_str(id_buf);
        pjsua_acc_add(&acc_cfg, PJ_TRUE, &g_acc_id);
    }

    PJ_LOG(3, ("sip_modem", "SIP V.90 modem ready. PTY link: %s", pty_link));
    log_modem_diag_snapshot("startup");

    /* ── Main event loop ─────────────────────────────────────────── */
    while (g_running) {
        me_state_t state_now;
        uint8_t dte_buf[256];
        int dte_len;

        /* Poll for PJSIP events (10 ms tick) */
        pjsua_handle_events(10);

        /* Move online serial payload into the modem engine.  The PTY reader
         * owns AT/escape handling; only bytes exposed by di_read_data() are
         * connection payload. */
        while ((dte_len = di_read_data(dte_buf, (int) sizeof(dte_buf))) > 0) {
            int accepted = me_put_data(dte_buf, dte_len);

            if (accepted != dte_len) {
                PJ_LOG(2, ("sip_modem", "DTE TX ring overrun: accepted %d/%d bytes",
                           accepted, dte_len));
                break;
            }
        }

        state_now = me_get_state();
        if (state_now != g_last_logged_me_state) {
            g_last_logged_me_state = state_now;
            log_modem_diag_snapshot("state-change");
        }
        if ((g_media_connected ? 1 : 0) != g_last_logged_media_connected) {
            g_last_logged_media_connected = g_media_connected ? 1 : 0;
            log_modem_diag_snapshot(g_media_connected ? "media-up" : "media-down");
        }

        /* ── Ring timer: send RING and auto-answer after N rings ── */
        if (g_ringing_call != PJSUA_INVALID_ID) {
            pj_time_val now;
            pj_gettimeofday(&now);
            long elapsed_ms = (now.sec - g_last_ring_time.sec) * 1000
                            + (now.msec - g_last_ring_time.msec);

            if (elapsed_ms >= RING_INTERVAL_MS) {
                g_ring_count++;
                g_last_ring_time = now;
                di_on_ring();
                PJ_LOG(3, ("sip_modem", "RING %d/%d",
                           g_ring_count, AUTO_ANSWER_RINGS));

                if (g_ring_count >= AUTO_ANSWER_RINGS) {
                    /* Answer the call */
                    PJ_LOG(3, ("sip_modem", "Auto-answering after %d rings",
                               g_ring_count));
                    g_call_id      = g_ringing_call;
                    g_ringing_call = PJSUA_INVALID_ID;
                    g_ring_count   = 0;
                    /* Set g_call_id before pjsua_call_answer(): stream-created
                     * callbacks may run synchronously from the answer call. */
                    pjsua_call_answer(g_call_id, 200, NULL, NULL);
                }
            }
        }

        /* Answering a call can synchronously activate media and move the
         * modem engine from a previous HANGUP state into V8.  Do not act on
         * the snapshot taken before pjsua_call_answer(), or the newly
         * answered call is torn down immediately by the HANGUP check below. */
        state_now = me_get_state();

        /* Check if ATD has put us into DIALING state */
        if (state_now == ME_DIALING && g_call_id == PJSUA_INVALID_ID) {
            const char *uri = me_get_dial_uri();
            if (uri && uri[0]) {
                char dial_uri[512];
                const char *resolved_uri = uri;

                /* Hayes dial strings are normally numeric and SpanDSP's AT
                 * parser deliberately rejects SIP punctuation. Resolve a DTE
                 * number against --sip-server while still accepting a full
                 * URI from non-AT callers of me_dial(). */
                if (strncmp(uri, "sip:", 4) != 0 && sip_server && sip_server[0]) {
                    snprintf(dial_uri, sizeof(dial_uri), "sip:%s@%s", uri, sip_server);
                    resolved_uri = dial_uri;
                }
                pj_str_t dst = pj_str((char *)resolved_uri);
                pjsua_call_setting opt;
                pjsua_call_setting_default(&opt);
                status = pjsua_call_make_call(g_acc_id, &dst, &opt,
                                              NULL, NULL, &g_call_id);
                if (status != PJ_SUCCESS) {
                    PJ_LOG(2, ("sip_modem", "Call to %s failed", resolved_uri));
                    me_hangup();
                } else {
                    PJ_LOG(3, ("sip_modem", "Outgoing call to %s", resolved_uri));
                }
            }
        }

        /* Check if the modem engine requested a hang-up */
        if (state_now == ME_HANGUP && g_call_id != PJSUA_INVALID_ID) {
            pjsua_call_hangup(g_call_id, 0, NULL, NULL);
            /* on_call_state DISCONNECTED will clean up */
        }
    }

    /* ── Shutdown ─────────────────────────────────────────────────── */
    PJ_LOG(3, ("sip_modem", "Shutting down..."));

    if (g_call_id != PJSUA_INVALID_ID)
        pjsua_call_hangup(g_call_id, 0, NULL, NULL);

    pjsua_handle_events(200); /* flush pending events */
    pjsua_destroy();
    if (aud_subsys_inited)
        pjmedia_aud_subsys_shutdown();

    di_close();
    me_destroy();

    return 0;
}
