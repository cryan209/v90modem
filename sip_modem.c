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
 *                   [--local-port <port>] [--rtp-port <port>]
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

#include <pjmedia/rtp.h>
#include <pjmedia/transport.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <time.h>

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
/* Frames the receive path had nothing for and filled in rather than dropped.
   Reported with the RTP trace summary so loss concealment is visible next to
   the sequence gaps that caused it. */
static uint64_t g_rx_concealed_frames = 0;
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
    pj_bool_t fed = PJ_FALSE;

    if (!port || !frame || !port->downstream_port)
        return PJ_EINVAL;

    if (!g_passthrough_enabled) {
        pjmedia_frame audio_in;
        unsigned byte_count;
        unsigned sample_count;
        static int16_t adj_buf[PJ_ARRAY_SIZE(g_tx_linear) + 1];

        pj_bzero(&audio_in, sizeof(audio_in));
        audio_in.type = PJMEDIA_FRAME_TYPE_AUDIO;
        audio_in.buf = g_tx_linear;
        audio_in.size = sizeof(g_tx_linear);

        st = pjmedia_port_get_frame(port->downstream_port, &audio_in);
        if (st == PJ_SUCCESS && audio_in.type == PJMEDIA_FRAME_TYPE_AUDIO
            && audio_in.buf && audio_in.size >= sizeof(int16_t)) {
            /* Same gate as the G.711 paths below -- see
               me_rx_g711_slip_permitted() in modem_engine.h for why a
               spliced sample costs the whole connection. */
            int adj = me_rx_g711_slip_permitted() ? me_cr_get_adjustment() : 0;

            byte_count = (unsigned) audio_in.size;
            if (byte_count > sizeof(g_tx_linear))
                byte_count = sizeof(g_tx_linear);
            sample_count = byte_count / sizeof(int16_t);
            if (sample_count > 0) {
                const int16_t *samples = (const int16_t *) audio_in.buf;

                /* Clock recovery slips are off by default: the receivers
                   downstream track the remote oscillator with their own
                   fractional timing loops, and a spliced sample is a step
                   they cannot absorb.  Measured, one of them ends the call's
                   useful life -- me_rx_g711_slip_permitted(). */
                if (adj > 0 && sample_count < PJ_ARRAY_SIZE(adj_buf)) {
                    memcpy(adj_buf, samples, sample_count * sizeof(int16_t));
                    adj_buf[sample_count] = samples[sample_count - 1];
                    me_rx_audio(adj_buf, (int) sample_count + 1);
                } else if (adj < 0 && sample_count > 1) {
                    me_rx_audio(samples, (int) sample_count - 1);
                } else {
                    me_rx_audio(samples, (int) sample_count);
                }
            }
        }

        frame->type = PJMEDIA_FRAME_TYPE_NONE;
        frame->size = 0;
        return PJ_SUCCESS;
    }

    rx_ext = (pjmedia_frame_ext *) port->rx_ext_buf;
    pj_bzero(rx_ext, sizeof(pjmedia_frame_ext));

    st = pjmedia_port_get_frame(port->downstream_port, (pjmedia_frame *) rx_ext);
    if (st == PJ_SUCCESS && rx_ext->base.type == PJMEDIA_FRAME_TYPE_EXTENDED) {
        /* Off by default — see me_rx_g711_slip_permitted(). */
        int adj = me_rx_g711_slip_permitted() ? me_cr_get_adjustment() : 0;
        static uint8_t adj_g711_buf[PJ_ARRAY_SIZE(g_tx_linear) + 1];

        for (i = 0; i < rx_ext->subframe_cnt; i++) {
            pjmedia_frame_ext_subframe *sf = pjmedia_frame_ext_get_subframe(rx_ext, i);
            unsigned sf_bytes;
            if (!sf)
                continue;
            sf_bytes = ((unsigned) sf->bitlen + 7U) >> 3;
            if (sf_bytes == 0 || sf_bytes > PJ_ARRAY_SIZE(g_tx_linear))
                continue;
            fed = PJ_TRUE;

            /* Apply at most one slip per pulled frame, on the first
               subframe (there is normally exactly one). */
            if (i == 0 && adj > 0 && sf_bytes < PJ_ARRAY_SIZE(adj_g711_buf)) {
                memcpy(adj_g711_buf, sf->data, sf_bytes);
                adj_g711_buf[sf_bytes] = ((const uint8_t *) sf->data)[sf_bytes - 1];
                me_rx_g711(adj_g711_buf, (int) sf_bytes + 1);
            } else if (i == 0 && adj < 0 && sf_bytes > 1) {
                me_rx_g711((const uint8_t *)sf->data, (int) sf_bytes - 1);
            } else {
                me_rx_g711((const uint8_t *)sf->data, (int)sf_bytes);
            }
        }
    } else if (st == PJ_SUCCESS && rx_ext->base.type == PJMEDIA_FRAME_TYPE_AUDIO
               && rx_ext->base.buf && rx_ext->base.size > 0) {
        int adj = me_rx_g711_slip_permitted() ? me_cr_get_adjustment() : 0;
        static uint8_t adj_g711_buf[PJ_ARRAY_SIZE(g_tx_linear) + 1];
        unsigned sz = (unsigned) rx_ext->base.size;

        if (sz > PJ_ARRAY_SIZE(g_tx_linear))
            sz = PJ_ARRAY_SIZE(g_tx_linear);
        fed = PJ_TRUE;

        if (adj > 0 && sz < PJ_ARRAY_SIZE(adj_g711_buf) && sz > 0) {
            memcpy(adj_g711_buf, rx_ext->base.buf, sz);
            adj_g711_buf[sz] = ((const uint8_t *) rx_ext->base.buf)[sz - 1];
            me_rx_g711(adj_g711_buf, (int) sz + 1);
        } else if (adj < 0 && sz > 1) {
            me_rx_g711((const uint8_t *)rx_ext->base.buf, (int) sz - 1);
        } else {
            me_rx_g711((const uint8_t *)rx_ext->base.buf, (int)sz);
        }
    }

    /* Nothing arrived for this frame -- a lost packet, or a jitter-buffer
       underrun (the buffer is deliberately near-zero, see media_cfg.jb_*).
       Feed a frame of fill rather than nothing, so the codeword stream the
       receiver consumes keeps its length.
       
       Feeding nothing DELETES those samples from the stream, and a deletion
       is a permanent timing offset, not a gap: 160 samples is a whole number
       of symbols only at some rates (64 at 3200 baud, 60 at 3000, 48 at
       2400), and at 2743 or 3429 baud it is a fractional-symbol step of
       exactly the kind measured to end a connection -- see
       me_rx_g711_slip_permitted() in modem_engine.h.  Replayed over a clean
       recorded call, losing one packet the way we do today costs 1% of the
       call and concealing it costs nothing (99% vs 100% clean;
       tools/inject_sample_slips.py --lose/--conceal). */
    if (!fed) {
        static uint8_t fill_buf[PJ_ARRAY_SIZE(g_tx_linear)];
        unsigned n = port->payload_samples_per_frame;
        uint8_t fill = (me_get_law() == ME_LAW_ALAW) ? 0xD5 : 0xFF;

        if (n > 0 && n <= PJ_ARRAY_SIZE(fill_buf)) {
            memset(fill_buf, fill, n);
            me_rx_g711(fill_buf, (int) n);
            g_rx_concealed_frames++;
        }
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

/* ------------------------------------------------------------------ */
/* RTP timing tap — feeds clock_recovery.c's DPLL                     */
/*                                                                      */
/* modem_passthrough_get_frame() pulls audio through PJSIP's own       */
/* jitter buffer, which we've deliberately shrunk to near-zero         */
/* (see media_cfg.jb_* below) because adaptive jitter buffering        */
/* disrupts V.34/V.90 phase continuity. But two independent 8 kHz      */
/* clocks (ours and the far modem's) will always drift apart, and      */
/* nothing was compensating for that drift. This transport wraps the   */
/* real UDP transport purely to observe each inbound RTP packet's      */
/* timestamp against our wall clock, feeding modem_engine's DPLL       */
/* (me_cr_update). It never modifies the packet stream.                */
/* ------------------------------------------------------------------ */

typedef struct rtp_trace_state_s {
    FILE *file;
    pj_bool_t initialized;
    uint16_t last_seq;
    uint32_t last_ts;
    int64_t last_ns;
    uint64_t packets;
    uint64_t sequence_gaps;
    uint64_t timestamp_discontinuities;
} rtp_trace_state_t;

typedef struct rtp_tap_transport_s {
    pjmedia_transport base;
    pj_pool_t *pool;
    pjmedia_transport *slave_tp;

    void *stream_user_data;
    void (*stream_rtp_cb)(void *user_data, void *pkt, pj_ssize_t size);
    void (*stream_rtp_cb2)(pjmedia_tp_cb_param *param);
    void (*stream_rtcp_cb)(void *user_data, void *pkt, pj_ssize_t size);
    rtp_trace_state_t rx_trace;
    rtp_trace_state_t tx_trace;
} rtp_tap_transport_t;

static int64_t rtp_tap_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t) ts.tv_sec * 1000000000LL + (int64_t) ts.tv_nsec;
}

static void rtp_trace_open(rtp_trace_state_t *trace, const char *name)
{
    const char *dir = getenv("VPCM_G711_TAP_DIR");
    char path[1024];

    if (!trace || !dir || !*dir)
        return;
    if (snprintf(path, sizeof(path), "%s/%s", dir, name) >= (int)sizeof(path)) {
        PJ_LOG(2, ("sip_modem", "RTP trace path is too long; %s disabled", name));
        return;
    }
    trace->file = fopen(path, "w");
    if (!trace->file) {
        PJ_LOG(2, ("sip_modem", "Cannot open RTP trace %s: %s", path, strerror(errno)));
        return;
    }
    setvbuf(trace->file, NULL, _IOFBF, 64 * 1024);
    fprintf(trace->file,
            "monotonic_ns,seq,timestamp,marker,payload_type,packet_bytes,ssrc,seq_delta,ts_delta,wall_delta_ns\n");
}

static void rtp_trace_packet(rtp_trace_state_t *trace, const void *pkt,
                             pj_size_t size, int64_t now_ns)
{
    const pjmedia_rtp_hdr *hdr;
    uint16_t seq;
    uint32_t ts;
    int seq_delta = 0;
    int32_t ts_delta = 0;
    int64_t wall_delta = 0;

    if (!trace || size < sizeof(pjmedia_rtp_hdr))
        return;
    hdr = (const pjmedia_rtp_hdr *) pkt;
    seq = pj_ntohs(hdr->seq);
    ts = pj_ntohl(hdr->ts);
    if (trace->initialized) {
        seq_delta = (int)(uint16_t)(seq - trace->last_seq);
        ts_delta = (int32_t)(ts - trace->last_ts);
        wall_delta = now_ns - trace->last_ns;
        if (seq_delta != 1)
            trace->sequence_gaps++;
        /* G.711 is normally one 160-sample packet. A marker may legitimately
           restart a talkspurt/timestamp run, so only count an unexpected RTP
           timestamp step when sequence continuity says this is the next packet. */
        if (seq_delta == 1 && !hdr->m && ts_delta != 160)
            trace->timestamp_discontinuities++;
    }
    if (trace->file) {
        fprintf(trace->file, "%lld,%u,%u,%u,%u,%lu,%u,%d,%d,%lld\n",
                (long long)now_ns, (unsigned)seq, (unsigned)ts,
                (unsigned)hdr->m, (unsigned)hdr->pt, (unsigned long)size,
                (unsigned)pj_ntohl(hdr->ssrc), seq_delta, (int)ts_delta,
                (long long)wall_delta);
    }
    trace->initialized = PJ_TRUE;
    trace->last_seq = seq;
    trace->last_ts = ts;
    trace->last_ns = now_ns;
    trace->packets++;
}

static void rtp_tap_observe(rtp_tap_transport_t *tap, const void *pkt,
                            pj_ssize_t size)
{
    const pjmedia_rtp_hdr *hdr;
    int64_t now_ns;

    if (!tap || size < (pj_ssize_t) sizeof(pjmedia_rtp_hdr))
        return;
    hdr = (const pjmedia_rtp_hdr *) pkt;
    now_ns = rtp_tap_now_ns();
    rtp_trace_packet(&tap->rx_trace, pkt, (pj_size_t)size, now_ns);
    me_cr_update(pj_ntohl(hdr->ts), now_ns);
}

static void rtp_tap_rtp_cb2(pjmedia_tp_cb_param *param)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) param->user_data;

    rtp_tap_observe(tap, param->pkt, param->size);

    if (tap->stream_rtp_cb2) {
        pjmedia_tp_cb_param cbparam;
        pj_memcpy(&cbparam, param, sizeof(cbparam));
        cbparam.user_data = tap->stream_user_data;
        tap->stream_rtp_cb2(&cbparam);
    } else if (tap->stream_rtp_cb) {
        tap->stream_rtp_cb(tap->stream_user_data, param->pkt, param->size);
    }
}

static void rtp_tap_rtcp_cb(void *user_data, void *pkt, pj_ssize_t size)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) user_data;
    if (tap->stream_rtcp_cb)
        tap->stream_rtcp_cb(tap->stream_user_data, pkt, size);
}

static pj_status_t rtp_tap_get_info(pjmedia_transport *tp,
                                    pjmedia_transport_info *info)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_get_info(tap->slave_tp, info);
}

static pj_status_t rtp_tap_attach2(pjmedia_transport *tp,
                                   pjmedia_transport_attach_param *att_param)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    pj_status_t status;

    tap->stream_user_data = att_param->user_data;
    tap->stream_rtp_cb2 = att_param->rtp_cb2;
    tap->stream_rtp_cb = att_param->rtp_cb;
    tap->stream_rtcp_cb = att_param->rtcp_cb;

    att_param->rtp_cb2 = &rtp_tap_rtp_cb2;
    att_param->rtp_cb = NULL;
    att_param->rtcp_cb = &rtp_tap_rtcp_cb;
    att_param->user_data = tap;

    status = pjmedia_transport_attach2(tap->slave_tp, att_param);
    if (status != PJ_SUCCESS) {
        tap->stream_user_data = NULL;
        tap->stream_rtp_cb = NULL;
        tap->stream_rtp_cb2 = NULL;
        tap->stream_rtcp_cb = NULL;
    }
    return status;
}

static void rtp_tap_detach(pjmedia_transport *tp, void *strm)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    PJ_UNUSED_ARG(strm);
    if (tap->stream_user_data) {
        pjmedia_transport_detach(tap->slave_tp, tap);
        tap->stream_user_data = NULL;
        tap->stream_rtp_cb = NULL;
        tap->stream_rtp_cb2 = NULL;
        tap->stream_rtcp_cb = NULL;
    }
}

static pj_status_t rtp_tap_send_rtp(pjmedia_transport *tp, const void *pkt,
                                    pj_size_t size)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    rtp_trace_packet(&tap->tx_trace, pkt, size, rtp_tap_now_ns());
    return pjmedia_transport_send_rtp(tap->slave_tp, pkt, size);
}

static pj_status_t rtp_tap_send_rtcp(pjmedia_transport *tp, const void *pkt,
                                     pj_size_t size)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_send_rtcp(tap->slave_tp, pkt, size);
}

static pj_status_t rtp_tap_send_rtcp2(pjmedia_transport *tp,
                                      const pj_sockaddr_t *addr,
                                      unsigned addr_len, const void *pkt,
                                      pj_size_t size)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_send_rtcp2(tap->slave_tp, addr, addr_len, pkt, size);
}

static pj_status_t rtp_tap_media_create(pjmedia_transport *tp,
                                        pj_pool_t *sdp_pool, unsigned options,
                                        const pjmedia_sdp_session *rem_sdp,
                                        unsigned media_index)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_media_create(tap->slave_tp, sdp_pool, options,
                                          rem_sdp, media_index);
}

static pj_status_t rtp_tap_encode_sdp(pjmedia_transport *tp,
                                      pj_pool_t *sdp_pool,
                                      pjmedia_sdp_session *local_sdp,
                                      const pjmedia_sdp_session *rem_sdp,
                                      unsigned media_index)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_encode_sdp(tap->slave_tp, sdp_pool, local_sdp,
                                        rem_sdp, media_index);
}

static pj_status_t rtp_tap_media_start(pjmedia_transport *tp, pj_pool_t *pool,
                                       const pjmedia_sdp_session *local_sdp,
                                       const pjmedia_sdp_session *rem_sdp,
                                       unsigned media_index)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_media_start(tap->slave_tp, pool, local_sdp,
                                         rem_sdp, media_index);
}

static pj_status_t rtp_tap_media_stop(pjmedia_transport *tp)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_media_stop(tap->slave_tp);
}

static pj_status_t rtp_tap_simulate_lost(pjmedia_transport *tp,
                                         pjmedia_dir dir, unsigned pct_lost)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    return pjmedia_transport_simulate_lost(tap->slave_tp, dir, pct_lost);
}

static void rtp_trace_close(const char *direction, rtp_trace_state_t *trace)
{
    if (!trace)
        return;
    if (trace->file) {
        fclose(trace->file);
        trace->file = NULL;
    }
    PJ_LOG(3, ("sip_modem",
               "RTP %s trace: packets=%llu sequence_gaps=%llu "
               "timestamp_discontinuities=%llu concealed_frames=%llu",
               direction, (unsigned long long)trace->packets,
               (unsigned long long)trace->sequence_gaps,
               (unsigned long long)trace->timestamp_discontinuities,
               (unsigned long long)g_rx_concealed_frames));
}

static void rtp_tap_on_destroy(void *arg)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) arg;
    rtp_trace_close("RX", &tap->rx_trace);
    rtp_trace_close("TX", &tap->tx_trace);
    if (tap->pool)
        pj_pool_release(tap->pool);
}

static pj_status_t rtp_tap_destroy(pjmedia_transport *tp)
{
    rtp_tap_transport_t *tap = (rtp_tap_transport_t *) tp;
    /* Slave transport (base_tp from on_create_media_transport) is owned
       and destroyed by pjsua itself; we only release our own wrapper. Actual
       pool release happens in rtp_tap_on_destroy(), invoked by the group
       lock once every ref (including the one we added below) is gone. */
    if (tap->base.grp_lock)
        pj_grp_lock_dec_ref(tap->base.grp_lock);
    else
        rtp_tap_on_destroy(tap);
    return PJ_SUCCESS;
}

static pjmedia_transport_op rtp_tap_op = {
    &rtp_tap_get_info,
    NULL,
    &rtp_tap_detach,
    &rtp_tap_send_rtp,
    &rtp_tap_send_rtcp,
    &rtp_tap_send_rtcp2,
    &rtp_tap_media_create,
    &rtp_tap_encode_sdp,
    &rtp_tap_media_start,
    &rtp_tap_media_stop,
    &rtp_tap_simulate_lost,
    &rtp_tap_destroy,
    &rtp_tap_attach2,
};

static pjmedia_transport *on_create_media_transport(pjsua_call_id call_id,
                                                    unsigned media_idx,
                                                    pjmedia_transport *base_tp,
                                                    unsigned flags)
{
    pj_pool_t *pool;
    rtp_tap_transport_t *tap;

    PJ_UNUSED_ARG(flags);

    if (!base_tp)
        return base_tp;

    pool = pjmedia_endpt_create_pool(pjsua_get_pjmedia_endpt(),
                                     "rtp-tap", 512, 512);
    if (!pool)
        return base_tp;

    tap = PJ_POOL_ZALLOC_T(pool, rtp_tap_transport_t);
    tap->pool = pool;
    pj_ansi_strxcpy(tap->base.name, pool->obj_name, sizeof(tap->base.name));
    tap->base.type = base_tp->type;
    tap->base.op = &rtp_tap_op;
    tap->slave_tp = base_tp;
    rtp_trace_open(&tap->rx_trace, "rtp-rx.csv");
    rtp_trace_open(&tap->tx_trace, "rtp-tx.csv");

    if (base_tp->grp_lock) {
        tap->base.grp_lock = base_tp->grp_lock;
        pj_grp_lock_add_ref(base_tp->grp_lock);
        pj_grp_lock_add_handler(base_tp->grp_lock, pool, tap, &rtp_tap_on_destroy);
    }

    PJ_LOG(3, ("sip_modem", "RTP clock-recovery tap installed on call %d media %d",
               call_id, media_idx));
    return &tap->base;
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

/* Seconds of RTP silence on a connected call before we hang it up. */
#define RTP_IDLE_HANGUP_SEC 20
static uint64_t     g_last_rtp_rx_octets = 0;
static pj_time_val  g_last_rtp_rx_time = {0, 0};

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
    const char *force_pcma = getenv("SIP_FORCE_PCMA");
    pj_bool_t pcmu_only = (force_pcmu && force_pcmu[0] && strcmp(force_pcmu, "0") != 0);
    pj_bool_t pcma_only = (force_pcma && force_pcma[0] && strcmp(force_pcma, "0") != 0);

    /* The negotiated law must match the far-end D/A converter's law with no
       transcoding anywhere in the path; SIP_FORCE_PCMA pins A-law for ATAs
       whose FXS codec runs A-law (e.g. AudioCodes in A-law countries). */
    if (pcma_only) {
        pjsua_codec_set_priority(&pcma, PJMEDIA_CODEC_PRIO_HIGHEST);
        pjsua_codec_set_priority(&pcmu, PJMEDIA_CODEC_PRIO_DISABLED);
    } else {
        /* Raise PCMU to highest priority */
        pjsua_codec_set_priority(&pcmu, PJMEDIA_CODEC_PRIO_HIGHEST);
        /* Allow PCMA as second choice (some ATAs only offer A-law) */
        pjsua_codec_set_priority(&pcma,
                                 pcmu_only ? PJMEDIA_CODEC_PRIO_DISABLED
                                           : PJMEDIA_CODEC_PRIO_NEXT_HIGHER);
    }

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

/* On a multihomed host pjsua's default-IP guess can pick an interface
 * that does not route to the SIP server, so the SDP c= line advertises
 * an address the peer never sends to (and strict-RTP peers then drop
 * our media as well).  Resolve the outbound interface the kernel would
 * use to reach the server and bind SIP+RTP to it. */
static int detect_local_ip_for_host(const char *host, char *buf, size_t buflen)
{
    struct addrinfo hints, *res = NULL;
    int fd, rc = -1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(host, "5060", &hints, &res) != 0 || !res)
        return -1;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd >= 0) {
        if (connect(fd, res->ai_addr, (socklen_t)res->ai_addrlen) == 0) {
            struct sockaddr_in local;
            socklen_t len = sizeof(local);
            if (getsockname(fd, (struct sockaddr *)&local, &len) == 0 &&
                inet_ntop(AF_INET, &local.sin_addr, buf, (socklen_t)buflen))
                rc = 0;
        }
        close(fd);
    }
    freeaddrinfo(res);
    return rc;
}

int main(int argc, char *argv[])
{
    /* stdout is fully block-buffered (not line-buffered) whenever it's
       redirected to a file/pipe, as the interop test harness always does —
       PJ_LOG output then sits in a ~4KB buffer and never reaches the log
       file for short/quiet runs (e.g. a call that never connects), even
       though the process is running fine. Force line buffering so live
       debugging actually sees output as it happens. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    const char *sip_server  = NULL;
    const char *username    = NULL;
    const char *password    = NULL;
    const char *pty_link    = "/tmp/modem0";
    const char *bind_addr   = NULL;
    const char *modem_mode  = NULL;
    char        bind_addr_buf[64];
    int         local_port  = 5060;
    int         rtp_port    = 0;
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
        else if (!strcmp(argv[i], "--rtp-port") && i+1 < argc)
            rtp_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--bind-addr") && i+1 < argc)
            bind_addr = argv[++i];
        else if (!strcmp(argv[i], "--mode") && i+1 < argc)
            modem_mode = argv[++i];
        else if (!strcmp(argv[i], "--verbose") || !strcmp(argv[i], "-v"))
            me_set_verbose(1);
        else if (!strcmp(argv[i], "--help")) {
            fprintf(stderr,
                "Usage: %s [--sip-server host] [--username u] [--password p]\n"
                "          [--pty-link path] [--local-port port] [--rtp-port port]\n"
                "          [--bind-addr ip] [--mode v34|v90|v92] [--verbose]\n", argv[0]);
            return 0;
        }
    }

    if (modem_mode
        && strcmp(modem_mode, "v34") != 0
        && strcmp(modem_mode, "v90") != 0
        && strcmp(modem_mode, "v92") != 0) {
        fprintf(stderr, "Invalid --mode '%s' (expected v34, v90, or v92)\n",
                modem_mode);
        return 2;
    }
    if (modem_mode)
        setenv("ME_MODE", modem_mode, 1);

    if (!bind_addr && sip_server &&
        detect_local_ip_for_host(sip_server, bind_addr_buf,
                                 sizeof(bind_addr_buf)) == 0) {
        bind_addr = bind_addr_buf;
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
    ua_cfg.cb.on_create_media_transport = on_create_media_transport;
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
    if (bind_addr) {
        trans_cfg.bound_addr = pj_str((char *)bind_addr);
        PJ_LOG(3, ("sip_modem", "Binding SIP/RTP to local address %s",
                   bind_addr));
    }
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
        if (bind_addr)
            acc_cfg.rtp_cfg.bound_addr = pj_str((char *)bind_addr);
        acc_cfg.cred_count    = 1;
        acc_cfg.cred_info[0].realm  = pj_str("*");
        acc_cfg.cred_info[0].scheme = pj_str("digest");
        acc_cfg.cred_info[0].username = pj_str((char *)username);
        acc_cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
        acc_cfg.cred_info[0].data      = pj_str((char *)password ? (char *)password : "");
        if (rtp_port > 0) {
            acc_cfg.rtp_cfg.port = (unsigned)rtp_port;
            PJ_LOG(3, ("sip_modem", "RTP media base port: %d", rtp_port));
        }

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
        if (bind_addr)
            acc_cfg.rtp_cfg.bound_addr = pj_str((char *)bind_addr);
        if (rtp_port > 0) {
            acc_cfg.rtp_cfg.port = (unsigned)rtp_port;
            PJ_LOG(3, ("sip_modem", "RTP media base port: %d", rtp_port));
        }
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
            if (!g_media_connected)
                me_flush_g711_taps();   /* off the media clock; see header */
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

        /* Drop a call whose RTP has stopped.  Without this, a peer that
         * disappears without a BYE (the soak rig kills slmodemd outright)
         * leaves the single call slot occupied for the life of the process
         * and every later INVITE is refused with "too many calls". */
        if (g_call_id != PJSUA_INVALID_ID && g_media_connected) {
            me_diag_snapshot_t rtp_snap;
            pj_time_val rtp_now;

            me_get_diag_snapshot(&rtp_snap);
            pj_gettimeofday(&rtp_now);
            if (rtp_snap.g711_rx_octets != g_last_rtp_rx_octets) {
                g_last_rtp_rx_octets = rtp_snap.g711_rx_octets;
                g_last_rtp_rx_time = rtp_now;
            } else if (g_last_rtp_rx_time.sec != 0
                       && rtp_now.sec - g_last_rtp_rx_time.sec
                            >= RTP_IDLE_HANGUP_SEC) {
                PJ_LOG(2, ("sip_modem",
                           "No RTP for %d s; hanging up the stalled call",
                           RTP_IDLE_HANGUP_SEC));
                pjsua_call_hangup(g_call_id, 0, NULL, NULL);
                g_last_rtp_rx_time.sec = 0;
            }
        } else {
            g_last_rtp_rx_time.sec = 0;
            g_last_rtp_rx_octets = 0;
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
    /* The audio subsystem was initialized with PJSUA's pool factory.
     * Destroy its CoreAudio factories before pjsua_destroy() invalidates
     * that factory and the mutexes allocated from it.  The error paths above
     * use the same ordering. */
    if (aud_subsys_inited)
        pjmedia_aud_subsys_shutdown();
    pjsua_destroy();

    di_close();
    me_destroy();

    return 0;
}
