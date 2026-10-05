/*
 * v70.c -- ITU-T V.70 DSVD terminal: SCF, voice and data processing around
 * the V.75 control entity and the V.76 multiplexer.  See v70.h.
 */

#include "v70.h"

#include <stdlib.h>
#include <string.h>

#define CAUSE_UNSPECIFIED   1
#define CAUSE_UNSUPPORTED   2
#define CAUSE_COLLISION     3

enum { K_CTL = 0, K_VOICE, K_DATA };

struct v70_s {
    v70_config_t cfg;
    v70_io_t io;
    v76_t *mf;
    v75_t *ce;
    v70_state_t st;
    v70_stats_t stats;

    int ch[3];                  /* open channel numbers, -1 if none */
    bool pending[3];            /* our CE-ESTABLISH request is outstanding */
    int req_ch[3];              /* ... and the channel number it asked for */
    bool ctl_close_pending;     /* EndSessionCommand sent; close once it is acked */
    int next_ch;

    v75_tcs_t peer;
    bool have_peer, sent_tcs, tcs_acked;
    bool opened_media;

    uint64_t tx_bits;
    int ms_acc;
    int voice_ms;
    int hdr_seq;

    v70_tunnel_rx_t trx;
    bool ending;
};

void v70_config_default(v70_config_t *c, bool initiator)
{
    memset(c, 0, sizeof(*c));
    c->initiator = initiator;
    c->line_bit_rate = 28800;
    c->audio_blocking_factor = 1;
    c->voice_codec = V75_AUDIO_G729_ANNEX_A;
    c->voice_frame_octets = 10;         /* G.729 Annex A: 80 bits per 10 ms */
    c->voice_frame_ms = 10;
    c->voice_crc8 = true;
    c->data_mode = V70_DATA_ASYNC_ERM;
    c->data_n401 = 128;
    c->data_window = 15;
    c->data_recovery = V76_REC_REJ;
    c->t401_ms = 1000;
}

/* ---- helpers -------------------------------------------------------------- */

static void set_state(v70_t *t, v70_state_t s)
{
    if (t->st == s)
        return;
    t->st = s;
    if (t->io.state_ind)
        t->io.state_ind(t->io.ctx, s);
}

static bool cfg_audio_hdr(const v70_config_t *c)
{
    return c->audio_header || c->audio_blocking_factor > 1;
}

static bool audio_hdr_on(const v70_t *t)
{
    return cfg_audio_hdr(&t->cfg);
}

static int voice_sdu_max(const v70_t *t)
{
    return t->cfg.voice_frame_octets * t->cfg.audio_blocking_factor + (audio_hdr_on(t) ? 1 : 0);
}

static int kind_of(const v75_olc_t *olc)
{
    if (olc->fwd.media == V75_MEDIA_AUDIO)
        return K_VOICE;
    if (olc->fwd.data.app == V75_APP_DSVD_CONTROL)
        return K_CTL;
    return K_DATA;
}

static int alloc_channel(v70_t *t)
{
    return t->next_ch++;
}

static bool all_media_open(const v70_t *t)
{
    return t->ch[K_VOICE] >= 0 && t->ch[K_DATA] >= 0 &&
           (!t->cfg.oob_control || t->ch[K_CTL] >= 0);
}

static void maybe_active(v70_t *t)
{
    if (t->st == V70_ESTABLISHING && all_media_open(t))
        set_state(t, V70_ACTIVE);
}

static void fill_v76(v75_v76_params_t *m, const v70_config_t *c, int kind, int n401)
{
    memset(m, 0, sizeof(*m));
    m->n401 = n401;
    m->crc_len = (kind == K_VOICE && c->voice_crc8) ? V76_FCS_8 : V76_FCS_16;
    if (kind == K_VOICE) {
        m->mode = V76_UNERM;
        m->suspend_resume = c->suspend_resume ? V75_SR_WITHOUT_ADDRESS : V75_SR_NONE;
        m->audio_header = cfg_audio_hdr(c);
    } else if (kind == K_DATA && c->data_mode != V70_DATA_ASYNC_ERM) {
        m->mode = V76_UNERM;
    } else {
        m->mode = V76_ERM;
        m->window = c->data_window;
        m->recovery = c->data_recovery;
    }
}

static void build_olc(const v70_t *t, int kind, int channel, v75_olc_t *olc)
{
    v75_olc_dir_t *d[2];
    int i;

    memset(olc, 0, sizeof(*olc));
    olc->has_rev = true;
    d[0] = &olc->fwd;
    d[1] = &olc->rev;
    for (i = 0; i < 2; i++) {
        d[i]->channel = channel;
        d[i]->has_port = true;                  /* default 0: unspecified (Cor.1) */
        if (kind == K_VOICE) {
            d[i]->media = V75_MEDIA_AUDIO;
            d[i]->audio.cap = t->cfg.voice_codec;
            d[i]->audio.frames = t->cfg.audio_blocking_factor;
            fill_v76(&d[i]->mux, &t->cfg, K_VOICE, voice_sdu_max(t));
        } else if (kind == K_DATA) {
            d[i]->media = V75_MEDIA_DATA;
            d[i]->data.app = V75_APP_NONE;
            d[i]->data.max_bit_rate = 0;        /* V.70 Cor.1 */
            d[i]->data.protocol = t->cfg.data_mode == V70_DATA_ASYNC_ERM ? V75_DP_V42_LAPM :
                                  t->cfg.data_mode == V70_DATA_TUNNEL_SAR
                                  ? V75_DP_HDLC_TUNNELLING_W_SAR : V75_DP_HDLC_TUNNELLING;
            fill_v76(&d[i]->mux, &t->cfg, K_DATA, t->cfg.data_n401);
        } else {
            d[i]->media = V75_MEDIA_DATA;
            d[i]->data.app = V75_APP_DSVD_CONTROL;
            fill_v76(&d[i]->mux, &t->cfg, K_CTL, 128);
            d[i]->mux.mode = V76_ERM;
            d[i]->mux.window = 4;
        }
    }
}

static int open_kind(v70_t *t, int kind)
{
    v75_olc_t olc;
    int ch = alloc_channel(t);

    if (t->pending[kind] || t->ch[kind] >= 0)
        return -1;
    build_olc(t, kind, ch, &olc);
    if (v75_establish_req(t->ce, &olc) < 0)
        return -1;
    t->pending[kind] = true;
    t->req_ch[kind] = ch;
    return ch;
}

/* ---- capabilities (6.2.1) ------------------------------------------------- */

static void build_tcs(const v70_t *t, v75_tcs_t *c)
{
    memset(c, 0, sizeof(*c));
    c->has_mux = true;
    c->mux.crc8 = c->mux.crc16 = true;
    c->mux.rej = true;
    c->mux.srej = true;
    c->mux.sr_without_address = t->cfg.suspend_resume;
    c->mux.audio_header = true;
    c->mux.uih = false;
    c->mux.num_dlcs = 4;
    c->mux.n401 = t->cfg.data_n401 > voice_sdu_max(t) ? t->cfg.data_n401 : voice_sdu_max(t);
    c->mux.max_window = t->cfg.data_window;
    c->n_caps = 2;
    c->caps[0].number = 1;
    c->caps[0].is_audio = true;
    c->caps[0].audio.cap = t->cfg.voice_codec;
    c->caps[0].audio.frames = t->cfg.audio_blocking_factor;
    c->caps[1].number = 2;
    c->caps[1].data.protocol = t->cfg.data_mode == V70_DATA_ASYNC_ERM ? V75_DP_V42_LAPM :
                               t->cfg.data_mode == V70_DATA_TUNNEL_SAR
                               ? V75_DP_HDLC_TUNNELLING_W_SAR : V75_DP_HDLC_TUNNELLING;
    c->n_desc = 1;
    c->desc[0].number = 1;
    c->desc[0].n_sets = 2;
    c->desc[0].set[0].n_alts = 1;
    c->desc[0].set[0].alt[0] = 1;
    c->desc[0].set[1].n_alts = 1;
    c->desc[0].set[1].alt[0] = 2;
}

static bool peer_has_audio(const v75_tcs_t *p, v75_audio_cap_t cap)
{
    int i;

    for (i = 0; i < p->n_caps; i++)
        if (p->caps[i].is_audio && p->caps[i].audio.cap == cap)
            return true;
    return false;
}

static void send_our_caps(v70_t *t)
{
    v75_tcs_t c;

    if (t->sent_tcs)
        return;
    build_tcs(t, &c);
    t->sent_tcs = true;
    v75_setparm_req(t->ce, -1, &c);             /* -1: out of band (6.4.4.2) */
}

/* The initiator opens the media channels once capabilities are settled. */
static void open_media(v70_t *t)
{
    if (!t->cfg.initiator || t->opened_media)
        return;
    if (t->cfg.oob_control) {
        if (!(t->tcs_acked && t->have_peer))
            return;
        if (!peer_has_audio(&t->peer, t->cfg.voice_codec)) {
            /* The speech coder is mandatory (5.8): without a common one
             * there is no DSVD call. */
            v70_end(t);
            set_state(t, V70_FAILED);
            return;
        }
    }
    t->opened_media = true;
    open_kind(t, K_VOICE);
    open_kind(t, K_DATA);
}

/* ---- CE user callbacks ---------------------------------------------------- */

static void u_establish_ind(void *c, const v75_olc_t *olc)
{
    v70_t *t = c;
    int kind = kind_of(olc);
    int cause = 0;

    /* 6.2.2: a request that crosses ours with the same data type -- the
     * initiator refuses the responder's; the responder takes the initiator's. */
    if (t->pending[kind] && t->cfg.initiator)
        cause = CAUSE_COLLISION;
    else if (kind == K_VOICE) {
        if (olc->fwd.audio.cap != t->cfg.voice_codec || olc->fwd.mux.mode != V76_UNERM ||
            olc->fwd.mux.n401 < t->cfg.voice_frame_octets)
            cause = CAUSE_UNSUPPORTED;
    } else if (kind == K_DATA) {
        v75_dataproto_t want = t->cfg.data_mode == V70_DATA_ASYNC_ERM ? V75_DP_V42_LAPM :
                               t->cfg.data_mode == V70_DATA_TUNNEL_SAR
                               ? V75_DP_HDLC_TUNNELLING_W_SAR : V75_DP_HDLC_TUNNELLING;

        if (olc->fwd.data.protocol != want)
            cause = CAUSE_UNSUPPORTED;
    }
    if (t->st == V70_IDLE || t->ending)
        cause = CAUSE_UNSPECIFIED;
    if (cause) {
        v75_establish_refuse(t->ce, olc->fwd.channel, cause);
        return;
    }
    if (t->pending[kind]) {
        /* We are the responder and ours lost the collision: drop ours. */
        t->pending[kind] = false;
    }
    {
        v75_olc_ack_t ack;

        memset(&ack, 0, sizeof(ack));
        ack.reverse_channel = olc->fwd.channel;
        v75_establish_rsp(t->ce, olc->fwd.channel, &ack);
    }
    t->ch[kind] = olc->fwd.channel;
    if (t->st == V70_ESTABLISHING)
        maybe_active(t);
}

static void u_establish_conf(void *c, int channel, const v75_olc_ack_t *ack)
{
    v70_t *t = c;
    int k;

    (void)ack;
    for (k = 0; k < 3; k++) {
        if (t->pending[k] && t->req_ch[k] == channel) {
            t->pending[k] = false;
            t->ch[k] = channel;
            if (k == K_CTL && t->cfg.initiator)
                send_our_caps(t);
            break;
        }
    }
    maybe_active(t);
}

static bool nothing_left(const v70_t *t)
{
    int k;

    for (k = 0; k < 3; k++)
        if (t->ch[k] >= 0 || t->pending[k])
            return false;
    return true;
}

static void u_release_ind(void *c, int channel, v75_release_cause_t cause, int reason)
{
    v70_t *t = c;
    int k;
    bool refused_own = false;

    (void)reason;
    for (k = 0; k < 3; k++) {
        if (t->ch[k] == channel) {
            t->ch[k] = -1;
            break;
        }
        if (t->pending[k] && t->req_ch[k] == channel) {
            t->pending[k] = false;
            refused_own = cause == V75_REL_REFUSED;
            break;
        }
    }
    if (cause == V75_REL_LINK_LOST && t->st != V70_ENDED && t->st != V70_ENDING &&
        t->st != V70_FAILED) {
        set_state(t, V70_FAILED);
        return;
    }
    /* 6.2.2: a refusal of our own request is the initiator's failure; for the
     * responder it just means the peer's request won the collision. */
    if (refused_own && t->cfg.initiator && !t->ending) {
        v70_end(t);
        set_state(t, V70_FAILED);
        return;
    }
    if (nothing_left(t) && (t->ending || t->st == V70_ACTIVE))
        set_state(t, V70_ENDED);
}

static void u_setparm_ind(void *c, int channel, const v75_tcs_t *caps)
{
    v70_t *t = c;

    t->peer = *caps;
    t->have_peer = true;
    v75_setparm_rsp(t->ce, channel, true, 0);
    /* Cor.1 6.2.1: a CE-SETPARM indication is answered with our own request. */
    if (!t->sent_tcs && t->cfg.oob_control)
        send_our_caps(t);
    open_media(t);
}

static void u_setparm_conf(void *c, int channel, bool ack, int reason)
{
    v70_t *t = c;

    (void)channel;
    (void)reason;
    if (ack)
        t->tcs_acked = true;
    else if (t->st == V70_ESTABLISHING) {
        v70_end(t);
        set_state(t, V70_FAILED);
        return;
    }
    open_media(t);
}

static void u_session_end(void *c)
{
    v70_t *t = c;

    /* 6.3: assume all DLCs are closed; close ours locally and say so. */
    if (!t->ending) {
        t->ending = true;
        v70_end(t);
    }
}

static void voice_deliver(v70_t *t, const uint8_t *d, int len, const v75_audio_hdr_t *h)
{
    int fo = t->cfg.voice_frame_octets;

    if (h && h->lost > 0) {
        v75_audio_hdr_t lost = *h;

        t->stats.voice_lost += (uint64_t)h->lost;
        if (t->io.voice_put_frame)
            t->io.voice_put_frame(t->io.ctx, NULL, 0, &lost);
    }
    t->stats.voice_rx++;
    if (!t->io.voice_put_frame)
        return;
    if (t->cfg.audio_blocking_factor > 1 && fo > 0 && len == fo * t->cfg.audio_blocking_factor &&
        !(h && h->silence)) {
        int off;

        for (off = 0; off < len; off += fo)
            t->io.voice_put_frame(t->io.ctx, d + off, fo, h);
    } else {
        t->io.voice_put_frame(t->io.ctx, d, len, h);
    }
}

static void u_data_ind(void *c, int channel, const uint8_t *d, int len, const v75_audio_hdr_t *h)
{
    v70_t *t = c;

    if (channel == t->ch[K_VOICE]) {
        voice_deliver(t, d, len, h);
        return;
    }
    if (channel != t->ch[K_DATA])
        return;
    if (t->cfg.data_mode == V70_DATA_ASYNC_ERM) {
        int i;

        t->stats.data_octets_rx += (uint64_t)len;
        if (t->io.dte_push)
            for (i = 0; i < len; i++)
                t->io.dte_push(t->io.ctx, d[i]);
    } else {
        uint8_t out[2 * V76_MAX_N401 + 4];
        int n = v70_tunnel_encode(d, len, out, sizeof(out)), i;

        t->stats.frames_rx++;
        t->stats.data_octets_rx += (uint64_t)len;
        if (n > 0 && t->io.dte_push)
            for (i = 0; i < n; i++)
                t->io.dte_push(t->io.ctx, out[i]);
    }
}

static void u_frame_ind(void *c, int channel, const uint8_t *f, int len, bool idle)
{
    v70_t *t = c;

    (void)idle;
    if (channel != t->ch[K_DATA] || len <= 0)
        return;
    u_data_ind(c, channel, f, len, NULL);
}

static void u_break_ind(void *c, int channel, int option, int length)
{
    v70_t *t = c;

    if (channel == t->ch[K_DATA] && t->io.break_ind)
        t->io.break_ind(t->io.ctx, option, length);
}

static void u_fcs_error(void *c, int channel)
{
    v70_t *t = c;

    /* V.76 5.3 NOTE: tell the voice user a frame was lost to bit errors. */
    if (channel == t->ch[K_VOICE]) {
        v75_audio_hdr_t h;

        memset(&h, 0, sizeof(h));
        h.lost = 1;
        t->stats.voice_fcs_lost++;
        if (t->io.voice_put_frame)
            t->io.voice_put_frame(t->io.ctx, NULL, 0, &h);
    }
}

/* ---- lifecycle ------------------------------------------------------------ */

v70_t *v70_create(const v70_config_t *cfg, const v70_io_t *io)
{
    v70_t *t = calloc(1, sizeof(*t));
    v76_config_t mc;
    v75_user_t u;
    v76_su_t su;

    if (!t)
        return NULL;
    t->cfg = *cfg;
    if (t->cfg.audio_blocking_factor < 1)
        t->cfg.audio_blocking_factor = 1;
    if (t->cfg.voice_frame_octets < 1)
        t->cfg.voice_frame_octets = 10;
    if (t->cfg.voice_frame_ms < 1)
        t->cfg.voice_frame_ms = 10;
    if (t->cfg.data_n401 < 1)
        t->cfg.data_n401 = 128;
    if (t->cfg.data_window < 1)
        t->cfg.data_window = 15;
    if (io)
        t->io = *io;
    t->ch[0] = t->ch[1] = t->ch[2] = -1;
    t->next_ch = t->cfg.first_channel > 0 ? t->cfg.first_channel
                                          : (t->cfg.initiator ? 1 : 33);
    t->cfg.first_channel = t->next_ch;

    v76_config_default(&mc, t->cfg.initiator);
    mc.line_bit_rate = t->cfg.line_bit_rate;
    if (t->cfg.t401_ms > 0)
        mc.t401_ms = t->cfg.t401_ms;
    mc.suspend_resume = t->cfg.suspend_resume;
    mc.sr_with_address = false;                     /* one voice DLC: Annex C 18 */
    mc.fcs_support = V76_FCS_MASK_8 | V76_FCS_MASK_16;
    t->mf = v76_create(&mc, NULL);
    t->ce = v75_create(t->mf, NULL);
    if (!t->mf || !t->ce) {
        v70_destroy(t);
        return NULL;
    }
    memset(&u, 0, sizeof(u));
    u.ctx = t;
    u.establish_ind = u_establish_ind;
    u.establish_conf = u_establish_conf;
    u.release_ind = u_release_ind;
    u.setparm_ind = u_setparm_ind;
    u.setparm_conf = u_setparm_conf;
    u.session_end_ind = u_session_end;
    u.data_ind = u_data_ind;
    u.frame_ind = u_frame_ind;
    u.break_ind = u_break_ind;
    u.fcs_error_ind = u_fcs_error;
    v75_set_user(t->ce, &u);
    su = v75_mf_su(t->ce);
    v76_set_su(t->mf, &su);
    v70_tunnel_rx_init(&t->trx);
    return t;
}

void v70_destroy(v70_t *t)
{
    if (!t)
        return;
    v75_destroy(t->ce);
    v76_destroy(t->mf);
    free(t);
}

void v70_start(v70_t *t)
{
    if (t->st != V70_IDLE)
        return;
    set_state(t, V70_ESTABLISHING);
    if (t->cfg.initiator) {
        if (t->cfg.oob_control)
            open_kind(t, K_CTL);
        else
            open_media(t);
    }
}

int v70_open_voice(v70_t *t) { return open_kind(t, K_VOICE); }
int v70_open_data(v70_t *t) { return open_kind(t, K_DATA); }

int v70_end(v70_t *t)
{
    int k, n = 0;

    if (t->st == V70_IDLE || t->st == V70_ENDED)
        return -1;
    t->ending = true;
    if (t->st != V70_FAILED)
        set_state(t, V70_ENDING);
    for (k = K_DATA; k >= K_VOICE; k--) {
        if (t->ch[k] >= 0 && v75_release_req(t->ce, t->ch[k]) == 0)
            n++;
    }
    if (t->ch[K_CTL] >= 0) {
        /* 6.3: say it out of band first; closing the control DLC discards
         * anything still queued, so it is closed once that has been acked. */
        v75_end_session_req(t->ce);
        t->ctl_close_pending = true;
        n++;
    }
    if (n == 0 && nothing_left(t) && t->st == V70_ENDING)
        set_state(t, V70_ENDED);
    return 0;
}

int v70_break(v70_t *t, int option, int length_10ms)
{
    if (t->ch[K_DATA] < 0 || t->cfg.data_mode != V70_DATA_ASYNC_ERM)
        return -1;
    return v75_break_req(t->ce, t->ch[K_DATA], option, length_10ms);
}

v70_state_t v70_state(const v70_t *t) { return t->st; }
const v70_stats_t *v70_stats(const v70_t *t) { return &t->stats; }
const v75_tcs_t *v70_peer_capabilities(const v70_t *t) { return t->have_peer ? &t->peer : NULL; }
v76_t *v70_mf(v70_t *t) { return t->mf; }
v75_t *v70_ce(v70_t *t) { return t->ce; }
int v70_voice_channel(const v70_t *t) { return t->ch[K_VOICE]; }
int v70_data_channel(const v70_t *t) { return t->ch[K_DATA]; }
uint64_t v70_tx_bits(const v70_t *t) { return t->tx_bits; }

/* ---- the clock: one millisecond of transmit bits -------------------------- */

static void voice_tick(v70_t *t)
{
    int period = t->cfg.voice_frame_ms * t->cfg.audio_blocking_factor;
    uint8_t sdu[V76_MAX_N401];
    v75_audio_hdr_t h;
    int n = 0, i, got_any = 0;
    bool silence_all = true, sid = false;

    if (t->ch[K_VOICE] < 0 || !t->io.voice_get_frame)
        return;
    if (++t->voice_ms < period)
        return;
    t->voice_ms = 0;

    memset(&h, 0, sizeof(h));
    for (i = 0; i < t->cfg.audio_blocking_factor; i++) {
        bool sil = false, s = false;
        int got = t->io.voice_get_frame(t->io.ctx, sdu + n, (int)sizeof(sdu) - n, &sil, &s);

        if (got > 0) {
            n += got;
            got_any = 1;
        }
        if (!sil)
            silence_all = false;
        sid = sid || s;
    }
    if (!got_any && !audio_hdr_on(t))
        return;
    h.silence = silence_all;
    h.sid = sid;
    if (v75_data_req(t->ce, t->ch[K_VOICE], sdu, n, &h) == 0)
        t->stats.voice_tx++;
}

static void data_tick(v70_t *t)
{
    int ch = t->ch[K_DATA];
    int dlci, guard = 0;

    if (ch < 0 || !t->io.dte_pull)
        return;
    dlci = v75_dlci_of(t->ce, ch);
    if (dlci < 0)
        return;

    if (t->cfg.data_mode == V70_DATA_ASYNC_ERM) {
        while (v76_data_backlog(t->mf, dlci) < 2 && guard++ < 2) {
            uint8_t buf[V76_MAX_N401];
            int n = 0, b;

            while (n < t->cfg.data_n401 && (b = t->io.dte_pull(t->io.ctx)) >= 0)
                buf[n++] = (uint8_t)b;
            if (n == 0)
                break;
            if (v75_data_req(t->ce, ch, buf, n, NULL) == 0)
                t->stats.data_octets_tx += (uint64_t)n;
            if (n < t->cfg.data_n401)
                break;
        }
        return;
    }

    /* Frame mode: parse the DTE's octet stream, one HDLC frame per UI. */
    while (v76_unitdata_backlog(t->mf, dlci) < 4 && guard < 4096) {
        int b = t->io.dte_pull(t->io.ctx);
        int n;

        if (b < 0)
            break;
        guard++;
        n = v70_tunnel_rx_put(&t->trx, (uint8_t)b);
        if (n > 0) {
            if (t->cfg.data_mode == V70_DATA_TUNNEL_UNERM && n > t->cfg.data_n401)
                continue;                           /* does not fit one UI frame */
            if (v75_data_req(t->ce, ch, t->trx.buf, n, NULL) == 0) {
                t->stats.frames_tx++;
                t->stats.data_octets_tx += (uint64_t)n;
            }
        }
    }
}

static void tick_ms(v70_t *t)
{
    v75_advance_ms(t->ce, 1);
    if (t->ctl_close_pending && t->ch[K_CTL] >= 0) {
        int dlci = v75_dlci_of(t->ce, t->ch[K_CTL]);

        if (dlci < 0 || (v76_data_backlog(t->mf, dlci) == 0 &&
                         v76_unacked_frames(t->mf, dlci) == 0)) {
            t->ctl_close_pending = false;
            v75_release_req(t->ce, t->ch[K_CTL]);
        }
    }
    if (t->st == V70_ACTIVE) {
        voice_tick(t);
        data_tick(t);
    }
}

int v70_tx_get_bit(v70_t *t)
{
    int b = v76_tx_get_bit(t->mf);

    t->tx_bits++;
    t->ms_acc += 1000;
    if (t->ms_acc >= t->cfg.line_bit_rate) {
        t->ms_acc -= t->cfg.line_bit_rate;
        tick_ms(t);
    }
    return b;
}

void v70_rx_put_bit(v70_t *t, int bit)
{
    v76_rx_put_bit(t->mf, bit);
}

void v70_tx_fill_bytes(v70_t *t, uint8_t *out, int len)
{
    int i, b;

    for (i = 0; i < len; i++) {
        uint8_t v = 0;

        for (b = 0; b < 8; b++)
            v |= (uint8_t)(v70_tx_get_bit(t) << b);
        out[i] = v;
    }
}

void v70_rx_push_bytes(v70_t *t, const uint8_t *in, int len)
{
    int i, b;

    for (i = 0; i < len; i++)
        for (b = 0; b < 8; b++)
            v70_rx_put_bit(t, (in[i] >> b) & 1);
}

/* ---- Annex A: UNERM tunnelling -------------------------------------------- */

int v70_tunnel_encode(const uint8_t *frame, int len, uint8_t *out, int max)
{
    int n = 0, i;

    if (len < 0 || max < len + 2)
        return -1;
    out[n++] = V70_HDLC_FLAG;
    for (i = 0; i < len; i++) {
        uint8_t b = frame[i];

        if (b == V70_HDLC_FLAG || b == V70_HDLC_ESCAPE) {
            if (n + 3 > max)
                return -1;
            out[n++] = V70_HDLC_ESCAPE;
            out[n++] = (uint8_t)(b ^ 0x20);         /* complement bit 6 */
        } else {
            if (n + 2 > max)
                return -1;
            out[n++] = b;
        }
    }
    out[n++] = V70_HDLC_FLAG;
    return n;
}

void v70_tunnel_rx_init(v70_tunnel_rx_t *r)
{
    memset(r, 0, sizeof(*r));
}

int v70_tunnel_rx_put(v70_tunnel_rx_t *r, uint8_t o)
{
    if (o == V70_HDLC_FLAG) {
        int ret = 0;

        if (r->in_frame && (r->escape || r->overflow)) {
            ret = -1;                               /* control escape + flag: abort */
        } else if (r->in_frame && r->len > 0) {
            ret = r->len;
        }
        r->in_frame = true;
        r->len = 0;
        r->escape = false;
        r->overflow = false;
        if (ret > 0) {
            /* Keep the octets readable until the next call. */
            return ret;
        }
        return ret;
    }
    if (!r->in_frame)
        return 0;
    if (r->escape) {
        o ^= 0x20;
        r->escape = false;
    } else if (o == V70_HDLC_ESCAPE) {
        r->escape = true;
        return 0;
    }
    if (r->len < (int)sizeof(r->buf))
        r->buf[r->len++] = o;
    else
        r->overflow = true;
    return 0;
}
