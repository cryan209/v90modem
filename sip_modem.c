/*
 * sip_modem.c — PJSIP SIP user agent + G.711 media port
 *
 * Initialises pjsua as a minimal SIP UA:
 *   - Null audio device (audio handled entirely by this process)
 *   - G.711 µ-law (PCMU/8000) only; all other codecs disabled
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
#include "clock_recovery.h"

#include <pjsua-lib/pjsua.h>

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
#define BITS_PER_SAMPLE  16

/* PJMEDIA port signature tag ('M','O','D','M') */
#define PORT_SIG PJMEDIA_FOURCC('M','O','D','M')

/* ------------------------------------------------------------------ */
/* Global state                                                        */
/* ------------------------------------------------------------------ */

static pj_pool_t       *g_pool       = NULL;
static pjsua_acc_id     g_acc_id     = PJSUA_INVALID_ID;
static pjsua_call_id    g_call_id    = PJSUA_INVALID_ID;
static pjsua_conf_port_id g_modem_slot = PJSUA_INVALID_ID;
static pjmedia_port     g_modem_port;
static int16_t          g_frame_buf[SAMPLES_PER_FRAME * 2]; /* TX + scratch */
static volatile int     g_running    = 1;

/* ------------------------------------------------------------------ */
/* Custom pjmedia_port — bridge between PJSIP and modem_engine        */
/* ------------------------------------------------------------------ */

/*
 * put_frame: called by the conference bridge with audio received from the
 * network (upstream modem signal). Forward to me_rx_audio.
 */
static pj_status_t modem_put_frame(pjmedia_port *port, pjmedia_frame *frame)
{
    (void)port;
    if (frame->type == PJMEDIA_FRAME_TYPE_AUDIO) {
        int16_t *samples = (int16_t *)frame->buf;
        int      count   = (int)(frame->size / sizeof(int16_t));
        me_rx_audio(samples, count);
    }
    return PJ_SUCCESS;
}

/*
 * get_frame: called by the conference bridge when it needs audio to send
 * to the network (downstream modem signal). Pull from me_tx_audio.
 */
static pj_status_t modem_get_frame(pjmedia_port *port, pjmedia_frame *frame)
{
    (void)port;
    int count = (int)(frame->size / sizeof(int16_t));
    me_tx_audio((int16_t *)frame->buf, count);
    frame->type      = PJMEDIA_FRAME_TYPE_AUDIO;
    frame->timestamp.u64 += count;
    return PJ_SUCCESS;
}

static pj_status_t modem_port_on_destroy(pjmedia_port *port)
{
    (void)port;
    return PJ_SUCCESS;
}

static void create_modem_port(pj_pool_t *pool)
{
    pj_str_t name = pj_str("modem");
    pjmedia_port_info_init(&g_modem_port.info, &name, PORT_SIG,
                           SAMPLE_RATE, 1, BITS_PER_SAMPLE, SAMPLES_PER_FRAME);
    g_modem_port.put_frame  = modem_put_frame;
    g_modem_port.get_frame  = modem_get_frame;
    g_modem_port.on_destroy = modem_port_on_destroy;
    (void)pool;
}

/* ------------------------------------------------------------------ */
/* Attach/detach the modem port to an active call's conference slot    */
/* ------------------------------------------------------------------ */

static void attach_modem_to_call(pjsua_call_id call_id)
{
    pjsua_call_info ci;
    if (pjsua_call_get_info(call_id, &ci) != PJ_SUCCESS) return;

    pjsua_conf_port_id call_slot = ci.conf_slot;
    if (call_slot == PJSUA_INVALID_ID) return;

    /* Add the modem port to the conference bridge */
    if (g_modem_slot == PJSUA_INVALID_ID) {
        if (pjsua_conf_add_port(g_pool, &g_modem_port, &g_modem_slot)
            != PJ_SUCCESS) {
            PJ_LOG(1, ("sip_modem", "conf_add_port failed"));
            return;
        }
    }

    /* Connect bidirectionally: network→modem, modem→network */
    pjsua_conf_connect(call_slot,     g_modem_slot);
    pjsua_conf_connect(g_modem_slot,  call_slot);

    PJ_LOG(3, ("sip_modem", "Modem port connected to call slot %d", call_slot));
    me_on_sip_connected();
}

static void detach_modem_from_call(void)
{
    if (g_modem_slot != PJSUA_INVALID_ID) {
        pjsua_conf_remove_port(g_modem_slot);
        g_modem_slot = PJSUA_INVALID_ID;
    }
    me_on_sip_disconnected();
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
        if (call_id == g_call_id) {
            detach_modem_from_call();
            g_call_id = PJSUA_INVALID_ID;
        }
    }
}

static void on_call_media_state(pjsua_call_id call_id)
{
    pjsua_call_info ci;
    pjsua_call_get_info(call_id, &ci);

    for (unsigned i = 0; i < ci.media_cnt; i++) {
        if (ci.media[i].type == PJMEDIA_TYPE_AUDIO &&
            ci.media[i].status == PJSUA_CALL_MEDIA_ACTIVE) {
            g_call_id = call_id;
            attach_modem_to_call(call_id);
            break;
        }
    }
}

static void on_incoming_call(pjsua_acc_id acc_id, pjsua_call_id call_id,
                              pjsip_rx_data *rdata)
{
    (void)acc_id; (void)rdata;
    pjsua_call_info ci;
    pjsua_call_get_info(call_id, &ci);

    PJ_LOG(3, ("sip_modem", "Incoming call from %.*s",
               (int)ci.remote_info.slen, ci.remote_info.ptr));

    /* Signal the data interface to send RING to the PTY */
    di_on_ring();

    /* Auto-answer after 1 ring (S0=1 equivalent) — in a full implementation
     * the AT interpreter's S0 register controls this. */
    pjsua_call_answer(call_id, 200, NULL, NULL);
    g_call_id = call_id;
}

/* ------------------------------------------------------------------ */
/* Codec setup — G.711 µ-law only                                      */
/* ------------------------------------------------------------------ */

static void restrict_to_g711(void)
{
    pj_str_t pcmu = pj_str("PCMU/8000");
    pj_str_t pcma = pj_str("PCMA/8000");

    /* Raise PCMU to highest priority */
    pjsua_codec_set_priority(&pcmu, PJMEDIA_CODEC_PRIO_HIGHEST);
    /* Allow PCMA as second choice (some ATAs only offer A-law) */
    pjsua_codec_set_priority(&pcma, PJMEDIA_CODEC_PRIO_NEXT_HIGHER);

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
        else if (!strcmp(argv[i], "--help")) {
            fprintf(stderr,
                "Usage: %s [--sip-server host] [--username u] [--password p]\n"
                "          [--pty-link path] [--local-port port]\n", argv[0]);
            return 0;
        }
    }

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* ── Initialise modem engine and PTY ─────────────────────────── */
    me_init();

    di_set_callbacks(on_dial, on_answer, on_hangup, NULL);
    if (di_open(pty_link) < 0) {
        fprintf(stderr, "Failed to open PTY\n");
        return 1;
    }

    /* ── Initialise PJSUA ────────────────────────────────────────── */
    pj_status_t status = pjsua_create();
    if (status != PJ_SUCCESS) {
        PJ_LOG(1, ("sip_modem", "pjsua_create failed"));
        return 1;
    }

    /* UA config */
    pjsua_config ua_cfg;
    pjsua_config_default(&ua_cfg);
    ua_cfg.cb.on_call_state       = on_call_state;
    ua_cfg.cb.on_call_media_state = on_call_media_state;
    ua_cfg.cb.on_incoming_call    = on_incoming_call;
    ua_cfg.max_calls              = 1;

    /* Logging */
    pjsua_logging_config log_cfg;
    pjsua_logging_config_default(&log_cfg);
    log_cfg.console_level = 3;

    /* Media config — disable all AEC, AGC, NR (modem signals must be clean) */
    pjsua_media_config media_cfg;
    pjsua_media_config_default(&media_cfg);
    media_cfg.snd_clock_rate  = SAMPLE_RATE;
    media_cfg.channel_count   = 1;
    media_cfg.audio_frame_ptime = 20;   /* 20 ms frames */
    media_cfg.ec_tail_len     = 0;      /* No echo cancellation */
    media_cfg.no_vad          = PJ_TRUE; /* No voice activity detection */

    status = pjsua_init(&ua_cfg, &log_cfg, &media_cfg);
    if (status != PJ_SUCCESS) {
        PJ_LOG(1, ("sip_modem", "pjsua_init failed"));
        pjsua_destroy();
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
        pjsua_destroy();
        return 1;
    }

    /* Use null sound device — audio is handled by our media port */
    pjsua_set_null_snd_dev();

    status = pjsua_start();
    if (status != PJ_SUCCESS) {
        PJ_LOG(1, ("sip_modem", "pjsua_start failed"));
        pjsua_destroy();
        return 1;
    }

    /* Restrict to G.711 */
    restrict_to_g711();

    /* Create the modem media port (must be after pjsua_start) */
    create_modem_port(g_pool);

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

    /* ── Main event loop ─────────────────────────────────────────── */
    while (g_running) {
        /* Poll for PJSIP events (10 ms tick) */
        pjsua_handle_events(10);

        /* Check if ATD has put us into DIALING state */
        if (me_get_state() == ME_DIALING && g_call_id == PJSUA_INVALID_ID) {
            const char *uri = me_get_dial_uri();
            if (uri && uri[0]) {
                pj_str_t dst = pj_str((char *)uri);
                pjsua_call_setting opt;
                pjsua_call_setting_default(&opt);
                status = pjsua_call_make_call(g_acc_id, &dst, &opt,
                                              NULL, NULL, &g_call_id);
                if (status != PJ_SUCCESS) {
                    PJ_LOG(2, ("sip_modem", "Call to %s failed", uri));
                    me_hangup();
                } else {
                    PJ_LOG(3, ("sip_modem", "Outgoing call to %s", uri));
                }
            }
        }

        /* Check if the modem engine requested a hang-up */
        if (me_get_state() == ME_HANGUP && g_call_id != PJSUA_INVALID_ID) {
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

    di_close();
    me_destroy();

    return 0;
}
