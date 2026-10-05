/*
 * v75.c -- ITU-T V.75 DSVD control entity.  See v75.h for scope, and for why
 * the H.245 wire encoding is a seam rather than an implementation.
 */

#include "v75.h"

#include <stdlib.h>
#include <string.h>

#define BRK_T401_MS   1000
#define BRK_N400      10

/* BRK / BRKACK message types (V.76 Annex B, Table B.1): bit 8 is the break
 * sequence number X. */
#define BRK_TYPE      0x40
#define BRKACK_TYPE   0x60

/* ------------------------------------------------------------------------ *
 * FI wrapper, audio header, segmentation header
 * ------------------------------------------------------------------------ */

int v75_wrap(const uint8_t *msg, int len, uint8_t *out, int max)
{
    if (len < 0 || len + 1 > max)
        return -1;
    out[0] = V75_FI_USER_DATA;
    memcpy(out + 1, msg, (size_t)len);
    return len + 1;
}

int v75_unwrap(const uint8_t *in, int len, const uint8_t **msg)
{
    if (len < 1 || in[0] != V75_FI_USER_DATA)
        return -1;
    *msg = in + 1;
    return len - 1;
}

uint8_t v75_audio_header_encode(const v75_audio_hdr_t *h)
{
    return (uint8_t)((h->silence ? 0x01 : 0) | (h->sid ? 0x02 : 0) | ((h->seq & 0x1F) << 2));
}

void v75_audio_header_decode(uint8_t o, v75_audio_hdr_t *h)
{
    h->present = true;
    h->silence = (o & 0x01) != 0;
    h->sid = (o & 0x02) != 0;
    h->seq = (o >> 2) & 0x1F;
    h->lost = 0;
}

/* ------------------------------------------------------------------------ *
 * Control entity
 * ------------------------------------------------------------------------ */

enum { CH_IDLE = 0, CH_AWAIT_ACK, CH_AWAIT_USER, CH_OPEN, CH_CLOSING };

typedef struct {
    int st;
    int channel;
    int dlci;
    bool opener;
    v75_olc_t olc;
    v76_dlc_params_t mf;
    bool control;
    bool hdr_tx, hdr_rx;
    int tx_seq;
    int rx_next;
    bool rx_have;
    bool sar;
    uint8_t sar_buf[V76_MAX_N401 * 2];
    int sar_len;
    bool sar_open;
    bool sar_idle;
    int vsb, vrb;
    bool brk_pending;
    int brk_t, brk_n;
    uint8_t brk_msg[3];
    int brk_len;
    bool have_last_ack;
} chan_t;

struct v75_s {
    v76_t *mf;
    const v75_h245_codec_t *codec;
    v75_user_t user;
    chan_t ch[V75_MAX_CHANNELS];
    int last_tcs_seq;
};

static chan_t *by_channel(v75_t *ce, int channel)
{
    int i;

    for (i = 0; i < V75_MAX_CHANNELS; i++)
        if (ce->ch[i].st != CH_IDLE && ce->ch[i].channel == channel)
            return &ce->ch[i];
    return NULL;
}

static chan_t *by_dlci(v75_t *ce, int dlci)
{
    int i;

    for (i = 0; i < V75_MAX_CHANNELS; i++)
        if (ce->ch[i].st != CH_IDLE && ce->ch[i].dlci == dlci)
            return &ce->ch[i];
    return NULL;
}

static chan_t *ch_alloc(v75_t *ce)
{
    int i;

    for (i = 0; i < V75_MAX_CHANNELS; i++)
        if (ce->ch[i].st == CH_IDLE) {
            memset(&ce->ch[i], 0, sizeof(ce->ch[i]));
            return &ce->ch[i];
        }
    return NULL;
}

static int encode_wrapped(v75_t *ce, const v75_msg_t *m, uint8_t *out, int max)
{
    uint8_t tmp[V75_MAX_MSG];
    int n = ce->codec->encode(m, tmp, sizeof(tmp));

    if (n < 0)
        return -1;
    return v75_wrap(tmp, n, out, max);
}

static int decode_wrapped(v75_t *ce, const uint8_t *in, int len, v75_msg_t *m)
{
    const uint8_t *body;
    int n = v75_unwrap(in, len, &body);

    if (n < 0)
        return -1;
    return ce->codec->decode(body, n, m);
}

/* Translate OLC parameters into V.76 DLC parameters.  Forward is opener ->
 * acceptor, so which of forward and reverse is "ours" depends on the role. */
void v75_olc_to_mf(const v75_olc_t *olc, bool opener, v76_dlc_params_t *p)
{
    const v75_olc_dir_t *tx = opener ? &olc->fwd : (olc->has_rev ? &olc->rev : &olc->fwd);
    const v75_olc_dir_t *rx = opener ? (olc->has_rev ? &olc->rev : &olc->fwd) : &olc->fwd;
    const v75_v76_params_t *m = &olc->fwd.mux;

    v76_dlc_params_default(p);
    p->mode = m->mode;
    p->fcs_len = m->crc_len ? m->crc_len : V76_FCS_16;
    p->recovery = m->recovery;
    p->uih = m->uih;
    p->realtime = m->suspend_resume != V75_SR_NONE;
    p->n401_tx = tx->mux.n401 ? tx->mux.n401 : 128;
    p->n401_rx = rx->mux.n401 ? rx->mux.n401 : 128;
    p->n401_rt = p->n401_tx;
    p->k = tx->mux.window ? tx->mux.window : 15;
    p->k_rx = rx->mux.window ? rx->mux.window : 15;
}

static void user_release(v75_t *ce, int channel, v75_release_cause_t cause, int reason)
{
    if (ce->user.release_ind)
        ce->user.release_ind(ce->user.ctx, channel, cause, reason);
}

/* ---- MF callbacks -------------------------------------------------------- */

static void mf_establish_ind(void *c, int dlci, const uint8_t *ud, int len)
{
    v75_t *ce = c;
    v75_msg_t m;
    chan_t *ch;
    uint8_t out[64];
    v75_msg_t rej;
    int n;

    memset(&m, 0, sizeof(m));
    if (decode_wrapped(ce, ud, len, &m) != 0 || m.type != V75_MSG_OLC ||
        !(ch = ch_alloc(ce)) || by_channel(ce, m.u.olc.fwd.channel)) {
        /* 6.2: cannot accept -> OpenLogicalChannelReject in the DM. */
        memset(&rej, 0, sizeof(rej));
        rej.type = V75_MSG_OLC_REJECT;
        rej.u.olc_reject.forward_channel = (m.type == V75_MSG_OLC) ? m.u.olc.fwd.channel : 0;
        rej.u.olc_reject.cause = 1;
        n = encode_wrapped(ce, &rej, out, sizeof(out));
        v76_establish_reject(ce->mf, dlci, out, n > 0 ? n : 0);
        return;
    }
    ch->st = CH_AWAIT_USER;
    ch->channel = m.u.olc.fwd.channel;
    ch->dlci = dlci;
    ch->opener = false;
    ch->olc = m.u.olc;
    ch->control = m.u.olc.fwd.media == V75_MEDIA_DATA &&
                  m.u.olc.fwd.data.app == V75_APP_DSVD_CONTROL;
    v75_olc_to_mf(&ch->olc, false, &ch->mf);
    /* Receive side: the opener's forward channel. */
    ch->hdr_rx = ch->olc.fwd.mux.audio_header;
    ch->hdr_tx = ch->olc.has_rev && ch->olc.rev.mux.audio_header;
    ch->sar = ch->olc.fwd.media == V75_MEDIA_DATA &&
              (ch->olc.fwd.data.protocol == V75_DP_SEGMENTATION_REASSEMBLY ||
               ch->olc.fwd.data.protocol == V75_DP_HDLC_TUNNELLING_W_SAR);
    if (ce->user.establish_ind)
        ce->user.establish_ind(ce->user.ctx, &ch->olc);
    else
        v75_establish_refuse(ce, ch->channel, 1);
}

static void mf_establish_conf(void *c, int dlci, const uint8_t *ud, int len)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);
    v75_msg_t m;

    if (!ch || ch->st != CH_AWAIT_ACK)
        return;
    if (decode_wrapped(ce, ud, len, &m) != 0 || m.type != V75_MSG_OLC_ACK) {
        /* The peer accepted but sent no usable acknowledgement. */
        memset(&m, 0, sizeof(m));
        m.type = V75_MSG_OLC_ACK;
        m.u.olc_ack.forward_channel = ch->channel;
        m.u.olc_ack.reverse_channel = ch->channel;
    }
    ch->st = CH_OPEN;
    if (ce->user.establish_conf)
        ce->user.establish_conf(ce->user.ctx, ch->channel, &m.u.olc_ack);
}

static void mf_release_ind(void *c, int dlci, const uint8_t *ud, int len,
                           v76_release_reason_t why)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);
    v75_msg_t m;
    int channel;
    v75_release_cause_t cause;
    int reason = 0;

    if (!ch)
        return;
    channel = ch->channel;
    switch (why) {
    case V76_REL_DM_RECEIVED:
        if (ch->st == CH_AWAIT_ACK) {
            cause = V75_REL_REFUSED;
            if (decode_wrapped(ce, ud, len, &m) == 0 && m.type == V75_MSG_OLC_REJECT)
                reason = m.u.olc_reject.cause;
        } else {
            cause = V75_REL_LINK_LOST;
        }
        break;
    case V76_REL_DISC_RECEIVED: {
        uint8_t out[16];
        v75_msg_t ack;
        int n;

        cause = V75_REL_REMOTE_CLOSE;
        memset(&ack, 0, sizeof(ack));
        ack.type = V75_MSG_CLC_ACK;
        ack.u.clc_ack_channel = channel;
        n = encode_wrapped(ce, &ack, out, sizeof(out));
        if (n > 0)
            v76_release_response_data(ce->mf, out, n);     /* 6.3.4 */
        break;
    }
    case V76_REL_LOCAL_COMPLETE:
        cause = V75_REL_LOCAL_CLOSE_DONE;
        break;
    default:
        cause = V75_REL_LINK_LOST;
        reason = (int)why;
        break;
    }
    memset(ch, 0, sizeof(*ch));
    user_release(ce, channel, cause, reason);
}

static void deliver_media(v75_t *ce, chan_t *ch, const uint8_t *d, int len)
{
    v75_audio_hdr_t h;

    memset(&h, 0, sizeof(h));
    if (ch->hdr_rx) {
        if (len < 1)
            return;
        v75_audio_header_decode(d[0], &h);
        if (ch->rx_have)
            h.lost = (h.seq - ch->rx_next) & 0x1F;
        ch->rx_next = (h.seq + 1) & 0x1F;
        ch->rx_have = true;
        d++;
        len--;
    }
    if (ce->user.data_ind)
        ce->user.data_ind(ce->user.ctx, ch->channel, d, len, ch->hdr_rx ? &h : NULL);
}

static void sar_receive(v75_t *ce, chan_t *ch, const uint8_t *d, int len)
{
    uint8_t h;

    if (len < 1)
        return;
    h = d[0];
    d++;
    len--;
    /* 11.1.2 */
    if ((h & V75_H_BEGIN) && ch->sar_open) {
        ch->sar_len = 0;                    /* the previous message is deleted */
        ch->sar_open = false;
    }
    if (!(h & V75_H_BEGIN) && !ch->sar_open) {
        /* A segment with no message in progress is discarded. */
    } else {
        if (h & V75_H_BEGIN) {
            ch->sar_len = 0;
            ch->sar_open = true;
        }
        if (ch->sar_len + len <= (int)sizeof(ch->sar_buf)) {
            memcpy(ch->sar_buf + ch->sar_len, d, (size_t)len);
            ch->sar_len += len;
        } else {
            ch->sar_open = false;
            ch->sar_len = 0;
        }
        if ((h & V75_H_FINAL) && ch->sar_open) {
            ch->sar_open = false;
            if (ce->user.frame_ind && ch->sar_len > 0)
                ce->user.frame_ind(ce->user.ctx, ch->channel, ch->sar_buf, ch->sar_len,
                                   ch->sar_idle);
            ch->sar_len = 0;
        }
    }
    {
        bool idle = (h & V75_H_IDLE) != 0;

        if (idle != ch->sar_idle) {
            ch->sar_idle = idle;
            if (ce->user.frame_ind)
                ce->user.frame_ind(ce->user.ctx, ch->channel, NULL, 0, idle);
        }
    }
}

static void control_message(v75_t *ce, const uint8_t *d, int len)
{
    v75_msg_t m;

    if (decode_wrapped(ce, d, len, &m) != 0)
        return;
    switch (m.type) {
    case V75_MSG_TCS:
        ce->last_tcs_seq = m.u.tcs.sequence_number;
        if (ce->user.setparm_ind)
            ce->user.setparm_ind(ce->user.ctx, -1, &m.u.tcs);
        break;
    case V75_MSG_TCS_ACK:
        if (ce->user.setparm_conf)
            ce->user.setparm_conf(ce->user.ctx, -1, true, 0);
        break;
    case V75_MSG_TCS_REJECT:
        if (ce->user.setparm_conf)
            ce->user.setparm_conf(ce->user.ctx, -1, false, m.u.tcs_reject.cause);
        break;
    case V75_MSG_END_SESSION:
        if (ce->user.session_end_ind)
            ce->user.session_end_ind(ce->user.ctx);
        break;
    case V75_MSG_REQUEST_MODE:
        if (ce->user.request_mode_ind)
            ce->user.request_mode_ind(ce->user.ctx, &m.u.request_mode);
        break;
    default:
        break;
    }
}

static void mf_data_ind(void *c, int dlci, const uint8_t *d, int len)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);

    if (!ch || ch->st != CH_OPEN)
        return;
    if (ch->control)
        control_message(ce, d, len);
    else
        deliver_media(ce, ch, d, len);
}

/* ---- Break (clause 10, V.76 Annex B) ------------------------------------ */

static void brk_send(v75_t *ce, chan_t *ch)
{
    v76_unitdata_req(ce->mf, ch->dlci, ch->brk_msg, ch->brk_len);
    ch->brk_t = BRK_T401_MS;
}

static void mf_unitdata_ind(void *c, int dlci, const uint8_t *d, int len, bool is_resp,
                            bool pf, bool header_only)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);

    (void)header_only;
    if (!ch || ch->st != CH_OPEN || len < 1)
        return;

    if (ch->mf.mode == V76_ERM) {
        /* UI on an ERM channel carries break messages (V.75 6.5.5). */
        int type = d[0] & 0x7F;
        int x = (d[0] >> 7) & 1;

        if (type == BRK_TYPE && !is_resp) {
            if (x == ch->vrb) {
                ch->vrb ^= 1;
                if (ce->user.break_ind)
                    ce->user.break_ind(ce->user.ctx, ch->channel, len > 1 ? d[1] : 0,
                                       len > 2 ? d[2] : 0);
            }
            /* Acknowledge, or re-acknowledge a duplicate: N(RB) = V(RB). */
            {
                uint8_t ack = (uint8_t)(BRKACK_TYPE | (ch->vrb << 7));

                v76_unitdata_rsp(ce->mf, ch->dlci, &ack, 1, pf);
            }
            return;
        }
        if (type == BRKACK_TYPE && is_resp) {
            if (ch->brk_pending && x == (ch->vsb ^ 1)) {
                ch->vsb ^= 1;
                ch->brk_pending = false;
                ch->brk_t = 0;
                if (ce->user.break_conf)
                    ce->user.break_conf(ce->user.ctx, ch->channel);
            }
            return;
        }
        return;
    }
    if (ch->control)
        return;
    if (ch->sar)
        sar_receive(ce, ch, d, len);
    else
        deliver_media(ce, ch, d, len);
}

static void mf_setparm_ind(void *c, int dlci, const uint8_t *ud, int len)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);
    v75_msg_t m;

    if (!ch)
        return;
    if (decode_wrapped(ce, ud, len, &m) != 0 || m.type != V75_MSG_TCS) {
        v75_setparm_rsp(ce, ch->channel, false, 1);
        return;
    }
    ce->last_tcs_seq = m.u.tcs.sequence_number;
    if (ce->user.setparm_ind)
        ce->user.setparm_ind(ce->user.ctx, ch->channel, &m.u.tcs);
    else
        v75_setparm_rsp(ce, ch->channel, false, 1);
}

static void mf_setparm_conf(void *c, int dlci, const uint8_t *ud, int len)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);
    v75_msg_t m;

    if (!ch || decode_wrapped(ce, ud, len, &m) != 0)
        return;
    if (ce->user.setparm_conf) {
        if (m.type == V75_MSG_TCS_ACK)
            ce->user.setparm_conf(ce->user.ctx, ch->channel, true, 0);
        else if (m.type == V75_MSG_TCS_REJECT)
            ce->user.setparm_conf(ce->user.ctx, ch->channel, false, m.u.tcs_reject.cause);
    }
}

static void mf_setparm_fail(void *c, int dlci)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);

    if (ch && ce->user.setparm_conf)
        ce->user.setparm_conf(ce->user.ctx, ch->channel, false, -1);
}

static void mf_fcs_error(void *c, int dlci)
{
    v75_t *ce = c;
    chan_t *ch = by_dlci(ce, dlci);

    if (ch && ce->user.fcs_error_ind)
        ce->user.fcs_error_ind(ce->user.ctx, ch->channel);
}

v76_su_t v75_mf_su(v75_t *ce)
{
    v76_su_t su;

    memset(&su, 0, sizeof(su));
    su.ctx = ce;
    su.establish_ind = mf_establish_ind;
    su.establish_conf = mf_establish_conf;
    su.release_ind = mf_release_ind;
    su.data_ind = mf_data_ind;
    su.unitdata_ind = mf_unitdata_ind;
    su.setparm_ind = mf_setparm_ind;
    su.setparm_conf = mf_setparm_conf;
    su.setparm_fail = mf_setparm_fail;
    su.fcs_error = mf_fcs_error;
    return su;
}

/* ---- Primitives from the SCF -------------------------------------------- */

v75_t *v75_create(v76_t *mf, const v75_h245_codec_t *codec)
{
    v75_t *ce = calloc(1, sizeof(*ce));

    if (!ce)
        return NULL;
    ce->mf = mf;
    ce->codec = codec ? codec : &v75_h245_codec;
    return ce;
}

void v75_destroy(v75_t *ce)
{
    free(ce);
}

void v75_set_user(v75_t *ce, const v75_user_t *u)
{
    ce->user = *u;
}

int v75_establish_req(v75_t *ce, const v75_olc_t *olc)
{
    v75_msg_t m;
    uint8_t ud[V75_MAX_MSG];
    chan_t *ch;
    int n, dlci;

    if (by_channel(ce, olc->fwd.channel))
        return -1;
    ch = ch_alloc(ce);
    if (!ch)
        return -1;
    memset(&m, 0, sizeof(m));
    m.type = V75_MSG_OLC;
    m.u.olc = *olc;
    n = encode_wrapped(ce, &m, ud, sizeof(ud));
    if (n < 0)
        return -1;
    ch->olc = *olc;
    v75_olc_to_mf(&ch->olc, true, &ch->mf);
    dlci = v76_establish_req(ce->mf, &ch->mf, ud, n);
    if (dlci < 0)
        return -1;
    ch->st = CH_AWAIT_ACK;
    ch->channel = olc->fwd.channel;
    ch->dlci = dlci;
    ch->opener = true;
    ch->control = olc->fwd.media == V75_MEDIA_DATA && olc->fwd.data.app == V75_APP_DSVD_CONTROL;
    ch->hdr_tx = olc->fwd.mux.audio_header;
    ch->hdr_rx = olc->has_rev && olc->rev.mux.audio_header;
    ch->sar = olc->fwd.media == V75_MEDIA_DATA &&
              (olc->fwd.data.protocol == V75_DP_SEGMENTATION_REASSEMBLY ||
               olc->fwd.data.protocol == V75_DP_HDLC_TUNNELLING_W_SAR);
    return dlci;
}

int v75_establish_rsp(v75_t *ce, int channel, const v75_olc_ack_t *ack)
{
    chan_t *ch = by_channel(ce, channel);
    v75_msg_t m;
    uint8_t ud[64];
    int n;

    if (!ch || ch->st != CH_AWAIT_USER)
        return -1;
    memset(&m, 0, sizeof(m));
    m.type = V75_MSG_OLC_ACK;
    m.u.olc_ack.forward_channel = channel;
    m.u.olc_ack.reverse_channel = ack ? ack->reverse_channel : channel;
    m.u.olc_ack.has_port = ack ? ack->has_port : false;
    m.u.olc_ack.port = ack ? ack->port : 0;
    n = encode_wrapped(ce, &m, ud, sizeof(ud));
    if (n < 0)
        return -1;
    ch->st = CH_OPEN;
    return v76_establish_rsp(ce->mf, ch->dlci, &ch->mf, ud, n);
}

int v75_establish_refuse(v75_t *ce, int channel, int cause)
{
    chan_t *ch = by_channel(ce, channel);
    v75_msg_t m;
    uint8_t ud[64];
    int n, dlci, r;

    if (!ch || ch->st != CH_AWAIT_USER)
        return -1;
    memset(&m, 0, sizeof(m));
    m.type = V75_MSG_OLC_REJECT;
    m.u.olc_reject.forward_channel = channel;
    m.u.olc_reject.cause = cause;
    n = encode_wrapped(ce, &m, ud, sizeof(ud));
    dlci = ch->dlci;
    memset(ch, 0, sizeof(*ch));
    r = v76_establish_reject(ce->mf, dlci, ud, n > 0 ? n : 0);
    return r;
}

int v75_release_req(v75_t *ce, int channel)
{
    chan_t *ch = by_channel(ce, channel);
    v75_msg_t m;
    uint8_t ud[32];
    int n;

    if (!ch || (ch->st != CH_OPEN && ch->st != CH_AWAIT_ACK))
        return -1;
    memset(&m, 0, sizeof(m));
    m.type = V75_MSG_CLC;
    m.u.clc.forward_channel = channel;
    m.u.clc.source_lcse = false;
    n = encode_wrapped(ce, &m, ud, sizeof(ud));
    ch->st = CH_CLOSING;            /* 6.3.4: the local CE considers it closed */
    return v76_release_req(ce->mf, ch->dlci, ud, n > 0 ? n : 0);
}

int v75_control_channel(const v75_t *ce)
{
    int i;

    for (i = 0; i < V75_MAX_CHANNELS; i++)
        if (ce->ch[i].st == CH_OPEN && ce->ch[i].control)
            return ce->ch[i].channel;
    return -1;
}

static int send_control(v75_t *ce, const v75_msg_t *m)
{
    int ctl = v75_control_channel(ce);
    uint8_t ud[V75_MAX_MSG];
    int n;
    chan_t *ch;

    if (ctl < 0)
        return -1;
    ch = by_channel(ce, ctl);
    n = encode_wrapped(ce, m, ud, sizeof(ud));
    if (n < 0 || n > ch->mf.n401_tx)
        return -1;
    return v76_data_req(ce->mf, ch->dlci, ud, n) ? 0 : -1;
}

int v75_setparm_req(v75_t *ce, int channel, const v75_tcs_t *caps)
{
    v75_msg_t m;

    memset(&m, 0, sizeof(m));
    m.type = V75_MSG_TCS;
    m.u.tcs = *caps;
    m.u.tcs.sequence_number = 0;                    /* = 0 for DSVD (Table 5) */
    if (channel < 0) {
        /* Out-of-band: any number of simultaneous sets (6.2.1). */
        return send_control(ce, &m);
    }
    {
        chan_t *ch = by_channel(ce, channel);
        uint8_t ud[V75_MAX_MSG];
        int n, i;

        if (!ch || ch->st != CH_OPEN)
            return -1;
        /* In-band: a single AlternativeCapabilitySet (6.4.4). */
        for (i = 0; i < m.u.tcs.n_desc; i++)
            if (m.u.tcs.desc[i].n_sets > 1)
                return -1;
        n = encode_wrapped(ce, &m, ud, sizeof(ud));
        if (n < 0)
            return -1;
        return v76_setparm_req(ce->mf, ch->dlci, ud, n);
    }
}

int v75_setparm_rsp(v75_t *ce, int channel, bool ack, int reason)
{
    v75_msg_t m;

    memset(&m, 0, sizeof(m));
    if (ack) {
        m.type = V75_MSG_TCS_ACK;
        m.u.tcs_ack_sequence = 0;
    } else {
        m.type = V75_MSG_TCS_REJECT;
        m.u.tcs_reject.sequence_number = 0;
        m.u.tcs_reject.cause = reason;
    }
    if (channel < 0)
        return send_control(ce, &m);
    {
        chan_t *ch = by_channel(ce, channel);
        uint8_t ud[64];
        int n;

        if (!ch)
            return -1;
        n = encode_wrapped(ce, &m, ud, sizeof(ud));
        if (n < 0)
            return -1;
        return v76_setparm_rsp(ce->mf, ch->dlci, ud, n);
    }
}

int v75_end_session_ex(v75_t *ce, v75_end_kind_t kind)
{
    v75_msg_t m;

    memset(&m, 0, sizeof(m));
    m.type = V75_MSG_END_SESSION;
    m.u.end_session = kind;
    return send_control(ce, &m);
}

int v75_end_session_req(v75_t *ce)
{
    return v75_end_session_ex(ce, V75_END_DISCONNECT);
}

int v75_request_mode_req(v75_t *ce, const v75_request_mode_t *rm)
{
    v75_msg_t m;

    memset(&m, 0, sizeof(m));
    m.type = V75_MSG_REQUEST_MODE;
    m.u.request_mode = *rm;
    return send_control(ce, &m);
}

int v75_data_req(v75_t *ce, int channel, const uint8_t *data, int len,
                 const v75_audio_hdr_t *hdr)
{
    chan_t *ch = by_channel(ce, channel);
    uint8_t buf[V76_MAX_N401 + 4];

    if (!ch || ch->st != CH_OPEN || ch->control || len < 0)
        return -1;

    if (ch->mf.mode == V76_ERM) {
        if (len > ch->mf.n401_tx)
            return -1;
        return v76_data_req(ce->mf, ch->dlci, data, len) ? 0 : -1;
    }

    if (ch->sar) {
        /* Clause 11: begin/final segments, each with an H octet in front. */
        int max = ch->mf.n401_tx - 1;
        int pos = 0;

        if (max < 1)
            return -1;
        if (len == 0)
            return 0;
        while (pos < len) {
            int n = len - pos > max ? max : len - pos;
            uint8_t h = 0;

            if (pos == 0)
                h |= V75_H_BEGIN;
            if (pos + n == len)
                h |= V75_H_FINAL;
            if (ch->sar_idle)
                h |= 0;                 /* idle is reported with v75_sar_idle() */
            buf[0] = h;
            memcpy(buf + 1, data + pos, (size_t)n);
            if (!v76_unitdata_req(ce->mf, ch->dlci, buf, n + 1))
                return -1;
            pos += n;
        }
        return 0;
    }

    {
        int off = 0;

        if (ch->hdr_tx) {
            v75_audio_hdr_t h;

            memset(&h, 0, sizeof(h));
            if (hdr)
                h = *hdr;
            h.seq = ch->tx_seq;
            buf[0] = v75_audio_header_encode(&h);
            off = 1;
        }
        if (len + off > ch->mf.n401_tx)
            return -1;
        memcpy(buf + off, data, (size_t)len);
        if (!v76_unitdata_req(ce->mf, ch->dlci, buf, len + off))
            return -1;
        if (ch->hdr_tx)
            ch->tx_seq = (ch->tx_seq + 1) & 0x1F;   /* 6.5.4: per L-UNITDATA */
        return 0;
    }
}

int v75_sar_idle(v75_t *ce, int channel, bool idle)
{
    chan_t *ch = by_channel(ce, channel);
    uint8_t h;

    if (!ch || ch->st != CH_OPEN || !ch->sar)
        return -1;
    h = (uint8_t)(idle ? V75_H_IDLE : 0);
    return v76_unitdata_req(ce->mf, ch->dlci, &h, 1) ? 0 : -1;
}

int v75_break_req(v75_t *ce, int channel, int option, int length_10ms)
{
    chan_t *ch = by_channel(ce, channel);

    if (!ch || ch->st != CH_OPEN || ch->mf.mode != V76_ERM || ch->brk_pending)
        return -1;
    ch->brk_msg[0] = (uint8_t)(BRK_TYPE | (ch->vsb << 7));
    ch->brk_msg[1] = (uint8_t)option;
    ch->brk_msg[2] = (uint8_t)length_10ms;
    ch->brk_len = length_10ms ? 3 : 2;
    ch->brk_pending = true;
    ch->brk_n = 0;
    brk_send(ce, ch);
    return 0;
}

void v75_advance_ms(v75_t *ce, int ms)
{
    int i;

    while (ms-- > 0) {
        for (i = 0; i < V75_MAX_CHANNELS; i++) {
            chan_t *ch = &ce->ch[i];

            if (ch->st != CH_OPEN || !ch->brk_pending)
                continue;
            if (ch->brk_t > 0 && --ch->brk_t == 0) {
                if (++ch->brk_n >= BRK_N400) {
                    ch->brk_pending = false;
                    if (ce->user.break_fail)
                        ce->user.break_fail(ce->user.ctx, ch->channel);
                } else {
                    brk_send(ce, ch);           /* B.1.3.4 */
                }
            }
        }
    }
}

int v75_dlci_of(const v75_t *ce, int channel)
{
    int i;

    for (i = 0; i < V75_MAX_CHANNELS; i++)
        if (ce->ch[i].st != CH_IDLE && ce->ch[i].channel == channel)
            return ce->ch[i].dlci;
    return -1;
}

int v75_channel_of(const v75_t *ce, int dlci)
{
    int i;

    for (i = 0; i < V75_MAX_CHANNELS; i++)
        if (ce->ch[i].st != CH_IDLE && ce->ch[i].dlci == dlci)
            return ce->ch[i].channel;
    return -1;
}

bool v75_channel_open(const v75_t *ce, int channel)
{
    int i;

    for (i = 0; i < V75_MAX_CHANNELS; i++)
        if (ce->ch[i].st == CH_OPEN && ce->ch[i].channel == channel)
            return true;
    return false;
}
