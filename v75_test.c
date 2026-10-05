/*
 * v75_test.c -- V.75 control entity: message model round trip, the audio and
 * segmentation header layouts, and two control entities running over two real
 * V.76 multiplexers joined by a bit pipe.
 */

#include "v75.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures, checks;

#define CHECK(cond, ...) \
    do { \
        checks++; \
        if (!(cond)) { \
            printf("FAIL: "); printf(__VA_ARGS__); printf("  [%s:%d]\n", __FILE__, __LINE__); \
            failures++; \
        } \
    } while (0)

static uint32_t rng_state = 0x2468ACEu;
static uint32_t rnd(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

/* ---------------------------------------------------------------------- */

static void fill_dir(v75_olc_dir_t *d, int channel, bool audio)
{
    memset(d, 0, sizeof(*d));
    d->channel = channel;
    d->has_port = true;
    d->port = 0;
    if (audio) {
        d->media = V75_MEDIA_AUDIO;
        d->audio.cap = V75_AUDIO_G729_ANNEX_A;
        d->audio.frames = 1;
    } else {
        d->media = V75_MEDIA_DATA;
        d->data.app = V75_APP_T120;
        d->data.protocol = V75_DP_V42_LAPM;
        d->data.max_bit_rate = 0;
    }
    d->mux.crc_len = 2;
    d->mux.n401 = 128;
    d->mux.mode = audio ? V76_UNERM : V76_ERM;
    d->mux.window = 15;
    d->mux.recovery = V76_REC_REJ;
}

static void test_codec(void)
{
    v75_msg_t m, back;
    uint8_t a[V75_MAX_MSG], b[V75_MAX_MSG], w[V75_MAX_MSG];
    int na, nb, t;
    const uint8_t *body;

    for (t = 0; t < 10; t++) {
        memset(&m, 0, sizeof(m));
        m.type = (v75_msg_type_t)(t + 1);
        switch (m.type) {
        case V75_MSG_OLC:
            fill_dir(&m.u.olc.fwd, 5, true);
            m.u.olc.fwd.mux.audio_header = true;
            m.u.olc.fwd.mux.suspend_resume = V75_SR_WITH_ADDRESS;
            m.u.olc.fwd.mux.uih = true;
            m.u.olc.has_rev = true;
            fill_dir(&m.u.olc.rev, 5, true);
            m.u.olc.rev.audio.frames = 3;
            break;
        case V75_MSG_OLC_ACK:
            m.u.olc_ack.forward_channel = 5;
            m.u.olc_ack.reverse_channel = 9;
            m.u.olc_ack.has_port = true;
            m.u.olc_ack.port = 77;
            break;
        case V75_MSG_OLC_REJECT:
            m.u.olc_reject.forward_channel = 5;
            m.u.olc_reject.cause = 3;
            break;
        case V75_MSG_CLC:
            m.u.clc.forward_channel = 5;
            m.u.clc.source_lcse = true;
            break;
        case V75_MSG_CLC_ACK:
            m.u.clc_ack_channel = 5;
            break;
        case V75_MSG_TCS: {
            v75_tcs_t *c = &m.u.tcs;

            c->has_mux = true;
            c->mux.crc16 = c->mux.crc8 = true;
            c->mux.rej = c->mux.srej = true;
            c->mux.sr_with_address = true;
            c->mux.audio_header = true;
            c->mux.num_dlcs = 8;
            c->mux.n401 = 4095;
            c->mux.max_window = 127;
            c->n_caps = 3;
            c->caps[0].number = 1; c->caps[0].is_audio = true;
            c->caps[0].audio.cap = V75_AUDIO_G729_ANNEX_A; c->caps[0].audio.frames = 2;
            c->caps[1].number = 2; c->caps[1].is_audio = true;
            c->caps[1].audio.cap = V75_AUDIO_G723; c->caps[1].audio.silence_suppression = true;
            c->caps[2].number = 3;
            c->caps[2].data.app = V75_APP_T434; c->caps[2].data.protocol = V75_DP_V76_W_COMPRESSION;
            c->caps[2].data.compression = 3; c->caps[2].data.v42bis_codewords = 65536;
            c->caps[2].data.v42bis_string = 250;
            c->n_desc = 2;
            c->desc[0].number = 1;
            c->desc[0].n_sets = 2;
            c->desc[0].set[0].n_alts = 2; c->desc[0].set[0].alt[0] = 1; c->desc[0].set[0].alt[1] = 2;
            c->desc[0].set[1].n_alts = 1; c->desc[0].set[1].alt[0] = 3;
            c->desc[1].number = 2;
            c->desc[1].n_sets = 1;
            c->desc[1].set[0].n_alts = 1; c->desc[1].set[0].alt[0] = 1;
            break;
        }
        case V75_MSG_TCS_ACK:
            m.u.tcs_ack_sequence = 0;
            break;
        case V75_MSG_TCS_REJECT:
            m.u.tcs_reject.cause = 4;
            break;
        case V75_MSG_END_SESSION:
            break;
        case V75_MSG_REQUEST_MODE:
            m.u.request_mode.forward_channel = 6;
            m.u.request_mode.mux.crc_len = 4;
            m.u.request_mode.mux.n401 = 1000;
            break;
        }
        na = v75_native_codec.encode(&m, a, sizeof(a));
        CHECK(na > 0, "message type %d encodes", t + 1);
        CHECK(v75_native_codec.decode(a, na, &back) == 0 && back.type == m.type,
              "message type %d decodes", t + 1);
        nb = v75_native_codec.encode(&back, b, sizeof(b));
        CHECK(na == nb && memcmp(a, b, (size_t)na) == 0, "message type %d round trips exactly", t + 1);
        CHECK(v75_native_codec.decode(a, na - (na > 1 ? 1 : 0), &back) != 0 || na == 1,
              "truncated type %d is refused", t + 1);

        /* V.75 8.1: wrapped in an FI octet of 133. */
        nb = v75_wrap(a, na, w, sizeof(w));
        CHECK(nb == na + 1 && w[0] == 0x85 && w[0] == 133, "FI field is 133 D (0x85)");
        CHECK(v75_unwrap(w, nb, &body) == na && memcmp(body, a, (size_t)na) == 0, "unwrap");
    }
    {
        v75_msg_t big;

        memset(&m, 0, sizeof(m));
        m.type = V75_MSG_TCS;
        m.u.tcs.n_caps = 1;
        na = v75_native_codec.encode(&m, a, sizeof(a));
        memcpy(&big, &m, sizeof(m));
        a[0] = 99;
        CHECK(v75_native_codec.decode(a, na, &back) != 0, "unknown message type is refused");
    }
    CHECK(v75_unwrap((const uint8_t *)"\x84xyz", 4, &body) < 0, "wrong FI is refused");
}

static void test_headers(void)
{
    v75_audio_hdr_t h, d;
    uint8_t o;

    /* Table 7: bit 0 silence, bit 1 SID, bits 2-6 sequence (bit 2 = LSB),
     * bit 7 reserved. */
    memset(&h, 0, sizeof(h));
    h.silence = true;
    CHECK(v75_audio_header_encode(&h) == 0x01, "silence frame -> bit 0");
    h.silence = false; h.sid = true;
    CHECK(v75_audio_header_encode(&h) == 0x02, "SID -> bit 1");
    h.sid = false; h.seq = 1;
    CHECK(v75_audio_header_encode(&h) == 0x04, "sequence 1 -> bit 2 (LSB)");
    h.seq = 31;
    CHECK(v75_audio_header_encode(&h) == 0x7C, "sequence 31 -> bits 2-6");
    h.seq = 32;
    CHECK(v75_audio_header_encode(&h) == 0x00, "sequence wraps at 32");
    for (o = 0; o < 128; o++) {
        v75_audio_header_decode(o, &d);
        CHECK(v75_audio_header_encode(&d) == o, "audio header %02x round trips", o);
    }
    /* Table 10 / Figure 2: F is bit 1, B bit 2, I bit 7. */
    CHECK(V75_H_FINAL == 0x01 && V75_H_BEGIN == 0x02 && V75_H_IDLE == 0x40, "H octet bits");
    CHECK((V75_H_BEGIN | V75_H_FINAL) == 0x03, "single frame = B and F");
}

/* ---------------------------------------------------------------------- *
 * Two control entities over two multiplexers
 * ---------------------------------------------------------------------- */

typedef struct {
    int est_ind, est_conf, rel_ind, setparm_ind, setparm_conf, session_end, rm_ind;
    int last_rel_channel, last_rel_reason;
    v75_release_cause_t last_rel_cause;
    v75_olc_t last_olc;
    v75_olc_ack_t last_ack;
    v75_tcs_t last_tcs;
    bool last_setparm_ack;
    int setparm_channel;
    v75_request_mode_t last_rm;
    bool accept;                         /* auto-accept indicated channels */
    int refuse_cause;
    /* media */
    uint8_t data[64][300];
    int data_len[64];
    v75_audio_hdr_t hdr[64];
    int n_data;
    size_t ch_octets[16];
    int ch_frames[16];
    uint8_t data_stream[200000];
    size_t data_stream_len;
    uint8_t frames[16][600];
    int frame_len[16];
    int n_frames, idle_events;
    bool idle_state;
    int brk_ind, brk_conf, brk_fail, last_brk_opt, last_brk_len;
    v75_t *ce;
} user_t;

static void u_est_ind(void *c, const v75_olc_t *olc)
{
    user_t *u = c;

    u->est_ind++;
    u->last_olc = *olc;
    if (u->accept) {
        v75_olc_ack_t ack;

        memset(&ack, 0, sizeof(ack));
        ack.reverse_channel = olc->fwd.channel;
        v75_establish_rsp(u->ce, olc->fwd.channel, &ack);
    } else {
        v75_establish_refuse(u->ce, olc->fwd.channel, u->refuse_cause);
    }
}
static void u_est_conf(void *c, int ch, const v75_olc_ack_t *ack)
{
    user_t *u = c;

    (void)ch;
    u->est_conf++;
    u->last_ack = *ack;
}
static void u_rel(void *c, int ch, v75_release_cause_t cause, int reason)
{
    user_t *u = c;

    u->rel_ind++;
    u->last_rel_channel = ch;
    u->last_rel_cause = cause;
    u->last_rel_reason = reason;
}
static void u_setparm_ind(void *c, int ch, const v75_tcs_t *t)
{
    user_t *u = c;

    u->setparm_ind++;
    u->setparm_channel = ch;
    u->last_tcs = *t;
    v75_setparm_rsp(u->ce, ch, true, 0);
}
static void u_setparm_conf(void *c, int ch, bool ack, int reason)
{
    user_t *u = c;

    (void)reason;
    u->setparm_conf++;
    u->setparm_channel = ch;
    u->last_setparm_ack = ack;
}
static void u_session_end(void *c) { ((user_t *)c)->session_end++; }
static void u_rm(void *c, const v75_request_mode_t *rm)
{
    user_t *u = c;

    u->rm_ind++;
    u->last_rm = *rm;
}
static void u_data(void *c, int ch, const uint8_t *d, int len, const v75_audio_hdr_t *h)
{
    user_t *u = c;

    if (ch >= 0 && ch < 16) {
        u->ch_octets[ch] += (size_t)len;
        u->ch_frames[ch]++;
    }
    if (u->n_data < 64) {
        memcpy(u->data[u->n_data], d, (size_t)(len > 300 ? 300 : len));
        u->data_len[u->n_data] = len;
        if (h)
            u->hdr[u->n_data] = *h;
        else
            memset(&u->hdr[u->n_data], 0, sizeof(u->hdr[0]));
        u->n_data++;
    }
    if (u->data_stream_len + (size_t)len <= sizeof(u->data_stream)) {
        memcpy(u->data_stream + u->data_stream_len, d, (size_t)len);
        u->data_stream_len += (size_t)len;
    }
}
static void u_frame(void *c, int ch, const uint8_t *f, int len, bool idle)
{
    user_t *u = c;

    (void)ch;
    if (len == 0) {
        u->idle_events++;
        u->idle_state = idle;
        return;
    }
    if (u->n_frames < 16) {
        memcpy(u->frames[u->n_frames], f, (size_t)(len > 600 ? 600 : len));
        u->frame_len[u->n_frames++] = len;
    }
}
static void u_brk_ind(void *c, int ch, int opt, int len)
{
    user_t *u = c;

    (void)ch;
    u->brk_ind++;
    u->last_brk_opt = opt;
    u->last_brk_len = len;
}
static void u_brk_conf(void *c, int ch) { (void)ch; ((user_t *)c)->brk_conf++; }
static void u_brk_fail(void *c, int ch) { (void)ch; ((user_t *)c)->brk_fail++; }

typedef struct {
    v76_t *mf[2];
    v75_t *ce[2];
    user_t u[2];
    long clock;
    long drop_ba_from, drop_ba_to;       /* B->A silent (all ones) */
    long drop_ab_from, drop_ab_to;
    uint32_t ber_ab;
} rig_t;

static void rig_init(rig_t *r, bool sr)
{
    int i;

    memset(r, 0, sizeof(*r));
    for (i = 0; i < 2; i++) {
        v76_config_t cfg;
        v75_user_t uu;
        v76_su_t su;

        v76_config_default(&cfg, i == 0);
        cfg.line_bit_rate = 28800;
        cfg.t401_ms = 300;
        cfg.suspend_resume = sr;
        cfg.fcs_support = V76_FCS_MASK_8 | V76_FCS_MASK_16 | V76_FCS_MASK_32;
        r->mf[i] = v76_create(&cfg, NULL);
        r->ce[i] = v75_create(r->mf[i], NULL);
        r->u[i].ce = r->ce[i];
        r->u[i].accept = true;
        memset(&uu, 0, sizeof(uu));
        uu.ctx = &r->u[i];
        uu.establish_ind = u_est_ind;
        uu.establish_conf = u_est_conf;
        uu.release_ind = u_rel;
        uu.setparm_ind = u_setparm_ind;
        uu.setparm_conf = u_setparm_conf;
        uu.session_end_ind = u_session_end;
        uu.request_mode_ind = u_rm;
        uu.data_ind = u_data;
        uu.frame_ind = u_frame;
        uu.break_ind = u_brk_ind;
        uu.break_conf = u_brk_conf;
        uu.break_fail = u_brk_fail;
        v75_set_user(r->ce[i], &uu);
        su = v75_mf_su(r->ce[i]);
        v76_set_su(r->mf[i], &su);
    }
}

static void rig_free(rig_t *r)
{
    int i;

    for (i = 0; i < 2; i++) {
        v75_destroy(r->ce[i]);
        v76_destroy(r->mf[i]);
    }
}

static void rig_run(rig_t *r, long bits)
{
    long i;

    for (i = 0; i < bits; i++) {
        int a = v76_tx_get_bit(r->mf[0]);
        int b = v76_tx_get_bit(r->mf[1]);

        if (r->ber_ab && rnd() < r->ber_ab)
            a ^= 1;
        if (r->drop_ab_to && r->clock >= r->drop_ab_from && r->clock < r->drop_ab_to)
            a = 1;
        if (r->drop_ba_to && r->clock >= r->drop_ba_from && r->clock < r->drop_ba_to)
            b = 1;
        v76_rx_put_bit(r->mf[1], a);
        v76_rx_put_bit(r->mf[0], b);
        if (r->clock % 29 == 0) {           /* ~1 ms */
            v75_advance_ms(r->ce[0], 1);
            v75_advance_ms(r->ce[1], 1);
        }
        r->clock++;
    }
}

static void test_mapping(void)
{
    v75_olc_t olc;
    v76_dlc_params_t po, pa;

    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 1, false);
    olc.has_rev = true;
    fill_dir(&olc.rev, 1, false);
    olc.fwd.mux.n401 = 100;  olc.rev.mux.n401 = 200;
    olc.fwd.mux.window = 4;  olc.rev.mux.window = 9;
    olc.fwd.mux.crc_len = 4;
    olc.fwd.mux.recovery = V76_REC_SREJ;
    v75_olc_to_mf(&olc, true, &po);
    v75_olc_to_mf(&olc, false, &pa);
    CHECK(po.n401_tx == 100 && po.n401_rx == 200, "opener sends the forward N401");
    CHECK(pa.n401_tx == 200 && pa.n401_rx == 100, "acceptor sends the reverse N401");
    CHECK(po.k == 4 && po.k_rx == 9 && pa.k == 9 && pa.k_rx == 4, "windows follow the direction");
    CHECK(po.fcs_len == 4 && po.recovery == V76_REC_SREJ && po.mode == V76_ERM, "CRC, recovery, mode");
    olc.fwd.mux.suspend_resume = V75_SR_WITHOUT_ADDRESS;
    olc.fwd.mux.mode = V76_UNERM;
    v75_olc_to_mf(&olc, true, &po);
    CHECK(po.realtime && po.mode == V76_UNERM, "suspend/resume marks the channel real-time");
}

static void test_voice_channel(void)
{
    rig_t r;
    v75_olc_t olc;
    int dlci, i;

    rng_state = 77;
    rig_init(&r, false);
    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 3, true);
    olc.fwd.mux.crc_len = 1;                    /* 8-bit CRC "particularly useful for voice" */
    olc.fwd.mux.n401 = 12;
    olc.fwd.mux.audio_header = true;
    olc.has_rev = true;
    fill_dir(&olc.rev, 3, true);
    olc.rev.mux.crc_len = 1;
    olc.rev.mux.n401 = 12;
    olc.rev.mux.audio_header = true;
    dlci = v75_establish_req(r.ce[0], &olc);
    CHECK(dlci == 0, "voice channel on DLCI 0");
    rig_run(&r, 30000);
    CHECK(r.u[1].est_ind == 1 && r.u[0].est_conf == 1, "voice channel established");
    CHECK(r.u[1].last_olc.fwd.audio.cap == V75_AUDIO_G729_ANNEX_A &&
          r.u[1].last_olc.fwd.channel == 3 && r.u[1].last_olc.fwd.mux.audio_header,
          "the OpenLogicalChannel arrived intact in the SABME");
    CHECK(r.u[0].last_ack.forward_channel == 3, "OpenLogicalChannelAck arrived in the UA");
    CHECK(v75_channel_open(r.ce[0], 3) && v75_channel_open(r.ce[1], 3), "both CEs consider it open");
    CHECK(v75_dlci_of(r.ce[1], 3) == 0 && v75_channel_of(r.ce[0], 0) == 3, "channel <-> DLCI");

    /* Send voice frames, silence and SID flags, and skip two. */
    for (i = 0; i < 20; i++) {
        uint8_t v[10];
        v75_audio_hdr_t h;

        memset(v, i, sizeof(v));
        memset(&h, 0, sizeof(h));
        h.silence = (i % 5) == 4;
        h.sid = (i % 7) == 6;
        if (i == 8 || i == 9) {
            /* "lose" frames 8 and 9 locally: consume the sequence numbers by
             * sending them into a dead line. */
            r.drop_ab_from = r.clock;
            r.drop_ab_to = r.clock + 1500;
            CHECK(v75_data_req(r.ce[0], 3, v, 10, &h) == 0, "voice frame %d queued", i);
            rig_run(&r, 1200);
            continue;
        }
        CHECK(v75_data_req(r.ce[0], 3, v, 10, &h) == 0, "voice frame %d queued", i);
        rig_run(&r, 800);
    }
    rig_run(&r, 5000);
    CHECK(r.u[1].n_data >= 17, "voice frames delivered (%d)", r.u[1].n_data);
    {
        int k, lost_total = 0, ok = 1;

        for (k = 0; k < r.u[1].n_data; k++) {
            lost_total += r.u[1].hdr[k].lost;
            if (r.u[1].data_len[k] != 10)
                ok = 0;
            if (!r.u[1].hdr[k].present)
                ok = 0;
        }
        CHECK(ok, "voice frames are 10 octets with the audio header stripped");
        CHECK(lost_total >= 1 && lost_total <= 3,
              "the header's 5-bit sequence number revealed the missing frames (%d)", lost_total);
        CHECK(r.u[1].hdr[0].seq == 0, "first frame carries sequence 0");
        CHECK(r.u[1].hdr[4].silence == ((r.u[1].hdr[4].seq % 5) == 4), "silence flag preserved");
    }
    rig_free(&r);
}

static void test_data_channel_and_close(void)
{
    rig_t r;
    v75_olc_t olc;
    uint8_t d[100];
    int i;
    size_t total = 0;

    rig_init(&r, false);
    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 4, false);
    olc.has_rev = true;
    fill_dir(&olc.rev, 4, false);
    v75_establish_req(r.ce[0], &olc);
    rig_run(&r, 30000);
    CHECK(r.u[0].est_conf == 1, "data channel established");
    for (i = 0; i < 40; i++) {
        memset(d, i, sizeof(d));
        while (v75_data_req(r.ce[0], 4, d, 100, NULL) != 0)
            rig_run(&r, 500);
        total += 100;
    }
    rig_run(&r, 200000);
    CHECK(r.u[1].data_stream_len == total, "all %zu data octets delivered (%zu)", total,
          r.u[1].data_stream_len);
    CHECK(r.u[1].data_stream[0] == 0 && r.u[1].data_stream[total - 1] == 39, "in order");

    /* 6.3: close. */
    CHECK(v75_release_req(r.ce[0], 4) == 0, "CE-RELEASE request");
    rig_run(&r, 30000);
    CHECK(r.u[1].rel_ind == 1 && r.u[1].last_rel_cause == V75_REL_REMOTE_CLOSE &&
          r.u[1].last_rel_channel == 4, "peer sees the channel closed");
    CHECK(r.u[0].rel_ind == 1 && r.u[0].last_rel_cause == V75_REL_LOCAL_CLOSE_DONE,
          "closer sees its CloseLogicalChannel acknowledged");
    CHECK(!v75_channel_open(r.ce[0], 4) && !v75_channel_open(r.ce[1], 4), "closed both ends");
    rig_free(&r);
}

static void test_refusal(void)
{
    rig_t r;
    v75_olc_t olc;

    rig_init(&r, false);
    r.u[1].accept = false;
    r.u[1].refuse_cause = 7;
    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 9, false);
    olc.has_rev = true;
    fill_dir(&olc.rev, 9, false);
    v75_establish_req(r.ce[0], &olc);
    rig_run(&r, 30000);
    CHECK(r.u[0].est_conf == 0 && r.u[0].rel_ind == 1 &&
          r.u[0].last_rel_cause == V75_REL_REFUSED && r.u[0].last_rel_reason == 7,
          "OpenLogicalChannelReject cause reaches the opener (cause %d)", r.u[0].last_rel_reason);
    CHECK(!v75_channel_open(r.ce[0], 9) && !v75_channel_open(r.ce[1], 9), "no channel");
    /* The same channel number can be tried again with other parameters (6.2.4 NOTE). */
    r.u[1].accept = true;
    v75_establish_req(r.ce[0], &olc);
    rig_run(&r, 30000);
    CHECK(r.u[0].est_conf == 1, "a second attempt succeeds");
    rig_free(&r);
}

static void test_control_channel(void)
{
    rig_t r;
    v75_olc_t olc;
    v75_tcs_t tcs;
    v75_request_mode_t rm;

    rig_init(&r, false);
    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 1, false);
    olc.fwd.data.app = V75_APP_DSVD_CONTROL;
    olc.fwd.data.protocol = V75_DP_NONE;
    olc.has_rev = true;
    olc.rev = olc.fwd;
    v75_establish_req(r.ce[0], &olc);
    rig_run(&r, 30000);
    CHECK(r.u[0].est_conf == 1 && v75_control_channel(r.ce[0]) == 1 &&
          v75_control_channel(r.ce[1]) == 1, "out-of-band control channel is up");
    CHECK(r.u[1].last_olc.fwd.data.app == V75_APP_DSVD_CONTROL, "marked DSVDControl");

    memset(&tcs, 0, sizeof(tcs));
    tcs.has_mux = true;
    tcs.mux.crc16 = true; tcs.mux.rej = true; tcs.mux.num_dlcs = 4; tcs.mux.n401 = 128;
    tcs.mux.max_window = 15;
    tcs.n_caps = 2;
    tcs.caps[0].number = 1; tcs.caps[0].is_audio = true;
    tcs.caps[0].audio.cap = V75_AUDIO_G729_ANNEX_A; tcs.caps[0].audio.frames = 1;
    tcs.caps[1].number = 2; tcs.caps[1].data.app = V75_APP_T120;
    tcs.caps[1].data.protocol = V75_DP_V42_LAPM;
    tcs.n_desc = 1;
    tcs.desc[0].number = 1;
    tcs.desc[0].n_sets = 2;                       /* allowed only out of band */
    tcs.desc[0].set[0].n_alts = 1; tcs.desc[0].set[0].alt[0] = 1;
    tcs.desc[0].set[1].n_alts = 1; tcs.desc[0].set[1].alt[0] = 2;
    CHECK(v75_setparm_req(r.ce[0], -1, &tcs) == 0, "CE-SETPARM request (out of band)");
    rig_run(&r, 40000);
    CHECK(r.u[1].setparm_ind == 1 && r.u[1].setparm_channel == -1, "TerminalCapabilitySet delivered");
    CHECK(r.u[1].last_tcs.n_caps == 2 && r.u[1].last_tcs.desc[0].n_sets == 2 &&
          r.u[1].last_tcs.caps[0].audio.cap == V75_AUDIO_G729_ANNEX_A &&
          r.u[1].last_tcs.mux.n401 == 128 && r.u[1].last_tcs.sequence_number == 0,
          "capabilities arrived intact, sequence number 0");
    CHECK(r.u[0].setparm_conf == 1 && r.u[0].last_setparm_ack, "TerminalCapabilitySetAck came back");

    rm.forward_channel = 5;
    memset(&rm.mux, 0, sizeof(rm.mux));
    rm.mux.crc_len = 2; rm.mux.n401 = 64;
    CHECK(v75_request_mode_req(r.ce[0], &rm) == 0, "RequestMode");
    CHECK(v75_end_session_req(r.ce[0]) == 0, "EndSessionCommand");
    rig_run(&r, 40000);
    CHECK(r.u[1].rm_ind == 1 && r.u[1].last_rm.mux.n401 == 64, "RequestMode with V76ModeParameters");
    CHECK(r.u[1].session_end == 1, "EndSessionCommand reaches the SCF (6.3)");
    rig_free(&r);
}

static void test_inband_setparm(void)
{
    rig_t r;
    v75_olc_t olc;
    v75_tcs_t tcs;

    rig_init(&r, false);
    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 2, false);
    olc.has_rev = true;
    fill_dir(&olc.rev, 2, false);
    v75_establish_req(r.ce[0], &olc);
    rig_run(&r, 30000);
    memset(&tcs, 0, sizeof(tcs));
    tcs.n_caps = 1;
    tcs.caps[0].number = 1; tcs.caps[0].data.app = V75_APP_T434;
    tcs.n_desc = 1;
    tcs.desc[0].number = 1;
    tcs.desc[0].n_sets = 1;
    tcs.desc[0].set[0].n_alts = 1; tcs.desc[0].set[0].alt[0] = 1;
    CHECK(v75_setparm_req(r.ce[0], 2, &tcs) == 0, "in-band CE-SETPARM (XID)");
    rig_run(&r, 30000);
    CHECK(r.u[1].setparm_ind == 1 && r.u[1].setparm_channel == 2 &&
          r.u[1].last_tcs.caps[0].data.app == V75_APP_T434, "capabilities arrived in the XID");
    CHECK(r.u[0].setparm_conf == 1 && r.u[0].last_setparm_ack, "XID response carried the Ack");
    tcs.desc[0].n_sets = 2;
    tcs.desc[0].set[1] = tcs.desc[0].set[0];
    CHECK(v75_setparm_req(r.ce[0], 2, &tcs) < 0,
          "multiple simultaneous sets are refused in-band (6.4.4)");
    rig_free(&r);
}

static void test_break(void)
{
    rig_t r;
    v75_olc_t olc;

    rig_init(&r, false);
    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 2, false);
    olc.has_rev = true;
    fill_dir(&olc.rev, 2, false);
    v75_establish_req(r.ce[0], &olc);
    rig_run(&r, 30000);

    CHECK(v75_break_req(r.ce[0], 2, 0x80, 25) == 0, "BRK sent");
    CHECK(v75_break_req(r.ce[0], 2, 0, 0) < 0, "one break at a time (V.75 10.1)");
    rig_run(&r, 20000);
    CHECK(r.u[1].brk_ind == 1 && r.u[1].last_brk_opt == 0x80 && r.u[1].last_brk_len == 25,
          "break indicated with option and length");
    CHECK(r.u[0].brk_conf == 1, "BRKACK confirmed it");

    /* The BRKACK is lost: B->A is dead.  The BRK is retransmitted with the
     * same N(SB) and the receiver must indicate it only ONCE (B.1.3.2). */
    r.drop_ba_from = r.clock;
    r.drop_ba_to = r.clock + 40000;               /* beyond one T401 */
    CHECK(v75_break_req(r.ce[0], 2, 0x40, 0) == 0, "second break (next sequence number)");
    rig_run(&r, 80000);
    CHECK(r.u[1].brk_ind == 2, "exactly one more indication despite retransmission (%d)", r.u[1].brk_ind);
    CHECK(r.u[0].brk_conf == 2, "and it is eventually confirmed (%d)", r.u[0].brk_conf);
    CHECK(r.u[0].brk_fail == 0, "N400 not reached");
    rig_free(&r);
}

static void test_sar(void)
{
    rig_t r;
    v75_olc_t olc;
    uint8_t f[100];
    int i;

    rig_init(&r, false);
    memset(&olc, 0, sizeof(olc));
    fill_dir(&olc.fwd, 6, false);
    olc.fwd.data.protocol = V75_DP_SEGMENTATION_REASSEMBLY;
    olc.fwd.mux.mode = V76_UNERM;
    olc.fwd.mux.n401 = 21;                       /* 20 data octets + the H octet */
    olc.has_rev = true;
    olc.rev = olc.fwd;
    v75_establish_req(r.ce[0], &olc);
    rig_run(&r, 30000);
    CHECK(r.u[0].est_conf == 1, "SAR channel up");
    for (i = 0; i < 100; i++)
        f[i] = (uint8_t)i;
    CHECK(v75_data_req(r.ce[0], 6, f, 100, NULL) == 0, "a 100-octet frame is segmented");
    CHECK(v75_data_req(r.ce[0], 6, f, 15, NULL) == 0, "a short frame goes as one segment");
    rig_run(&r, 40000);
    CHECK(r.u[1].n_frames == 2, "two frames reassembled (%d)", r.u[1].n_frames);
    CHECK(r.u[1].frame_len[0] == 100 && memcmp(r.u[1].frames[0], f, 100) == 0,
          "the segmented frame comes back whole");
    CHECK(r.u[1].frame_len[1] == 15 && memcmp(r.u[1].frames[1], f, 15) == 0, "single-segment frame");
    {
        /* 5 segments of 20: count UI frames sent. */
        CHECK(v76_stats(r.mf[0])->tx_ui_frames == 6, "5 + 1 UI frames on the wire (%llu)",
              (unsigned long long)v76_stats(r.mf[0])->tx_ui_frames);
    }

    /* 11.1.2: a "begin" while a message is in progress deletes the old one;
     * a middle segment with nothing in progress is discarded. */
    {
        int dlci = v75_dlci_of(r.ce[0], 6);
        uint8_t s1[6] = { V75_H_BEGIN, 1, 2, 3, 4, 5 };
        uint8_t s2[6] = { V75_H_BEGIN | V75_H_FINAL, 9, 9, 9, 9, 9 };
        uint8_t s3[4] = { 0x00, 7, 7, 7 };               /* middle, no begin */
        uint8_t s4[4] = { V75_H_FINAL, 8, 8, 8 };        /* final, no begin */

        v76_unitdata_req(r.mf[0], dlci, s1, 6);          /* begin, never finished */
        v76_unitdata_req(r.mf[0], dlci, s2, 6);          /* new begin+final replaces it */
        v76_unitdata_req(r.mf[0], dlci, s3, 4);
        v76_unitdata_req(r.mf[0], dlci, s4, 4);
        rig_run(&r, 30000);
        CHECK(r.u[1].n_frames == 3, "only the complete frame was delivered (%d)", r.u[1].n_frames);
        CHECK(r.u[1].frame_len[2] == 5 && r.u[1].frames[2][0] == 9, "the begin that replaced the old one");
    }

    /* The I bit: HDLC idle at the user interface. */
    CHECK(v75_sar_idle(r.ce[0], 6, true) == 0, "idle condition reported");
    rig_run(&r, 20000);
    CHECK(r.u[1].idle_state && r.u[1].idle_events >= 1, "peer sees idle begin");
    CHECK(v75_sar_idle(r.ce[0], 6, false) == 0, "idle ends");
    rig_run(&r, 20000);
    CHECK(!r.u[1].idle_state, "peer sees idle end");
    rig_free(&r);
}

static void test_suspend_resume_channel(void)
{
    /* A real-time audio channel (S/R) next to an ERM data channel, opened
     * through the CE: the OLC's suspendResume choice reaches the MF. */
    rig_t r;
    v75_olc_t a, d;
    uint8_t buf[1000];
    int i;

    rig_init(&r, true);
    memset(&d, 0, sizeof(d));
    fill_dir(&d.fwd, 1, false);
    d.has_rev = true;
    fill_dir(&d.rev, 1, false);
    d.fwd.mux.n401 = d.rev.mux.n401 = 1000;
    v75_establish_req(r.ce[0], &d);
    rig_run(&r, 30000);
    memset(&a, 0, sizeof(a));
    fill_dir(&a.fwd, 2, true);
    a.fwd.mux.n401 = 12;
    a.fwd.mux.crc_len = 1;
    a.fwd.mux.suspend_resume = V75_SR_WITHOUT_ADDRESS;
    a.has_rev = true;
    a.rev = a.fwd;
    v75_establish_req(r.ce[0], &a);
    rig_run(&r, 30000);
    CHECK(r.u[0].est_conf == 2, "data and real-time audio channels up");
    for (i = 0; i < 8; i++) {
        memset(buf, i, sizeof(buf));
        v75_data_req(r.ce[0], 1, buf, 1000, NULL);
    }
    for (i = 0; i < 20; i++) {
        uint8_t v[10];

        memset(v, i, sizeof(v));
        v75_data_req(r.ce[0], 2, v, 10, NULL);
        rig_run(&r, 288);
    }
    rig_run(&r, 300000);
    CHECK(r.u[1].ch_octets[1] == 8000, "data channel intact next to S/R audio (%zu)",
          r.u[1].ch_octets[1]);
    CHECK(r.u[1].ch_frames[2] == 20 && r.u[1].ch_octets[2] == 200,
          "all 20 audio frames delivered (%d frames, %zu octets)", r.u[1].ch_frames[2],
          r.u[1].ch_octets[2]);
    CHECK(v76_stats(r.mf[0])->sr_suspends > 0, "the audio frames really interrupted data frames");
    rig_free(&r);
}

int main(void)
{
    test_codec();
    test_headers();
    test_mapping();
    test_voice_channel();
    test_data_channel_and_close();
    test_refusal();
    test_control_channel();
    test_inband_setparm();
    test_break();
    test_sar();
    test_suspend_resume_channel();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
