/*
 * v8bis_fsm.c -- V.8bis transaction state machine.  Clause 9 and Figures 14
 * and 15 of V.8bis (08/96).  See the header for the transaction table.
 *
 * Figure 14 (initiating station), by state:
 *   Initial       -/MR, -/CR, -/MS, -/CL, -/CLR       (initiate)
 *   Sent MR       MS/ACK -> MS mode, MS/NAK -> Initial, MRd/MS, MRd/CRd (auto),
 *                 CRd/CL, CRd/CLR
 *   Sent CR       CL/MS, CLR/CL-MS, CL/ACK(2) and CLR/ACK(2) loops, CRd/CL, CRd/CLR
 *   Sent MS       ACK -> MS mode, NAK -> Initial
 *   Sent CL       MS/ACK, MS/NAK, ACK(2)/CL loop
 *   Sent CLR      CL-MS/ACK, CL-MS/NAK, ACK(2)/CLR loop
 * Figure 15 (responding station):
 *   Initial       MR/MS, MRe/MRd (auto), MRe/CRd (auto), CR/CL, CR/CLR, CRe/CRd
 *                 (auto), CL/MS, CLR/CL, MS/ACK, MS/NAK, CL/ACK(2), CLR/ACK(2)
 *   Sent MS       ACK -> MS mode, NAK -> Initial
 *   Sent CR       CL/MS, CLR/CL-MS, CL/ACK(2), CLR/ACK(2)             (auto only)
 *   Sent CL or MR MS/ACK, MS/NAK, ACK(2)/CL loop, CRd/CL loop, CRd/CLR
 *   Sent CLR      CL-MS/ACK, CL-MS/NAK, ACK(2)/CLR loop
 */
#include "v8bis_fsm.h"

#include <string.h>

const char *v8bis_state_name(v8bis_state_t s)
{
    switch (s) {
    case V8BIS_S_INITIAL: return "Initial";
    case V8BIS_S_SENT_MR: return "Sent MR";
    case V8BIS_S_SENT_CR: return "Sent CR";
    case V8BIS_S_SENT_MS: return "Sent MS";
    case V8BIS_S_SENT_CL: return "Sent CL";
    case V8BIS_S_SENT_CLR: return "Sent CLR";
    case V8BIS_S_MS_MODE: return "MS mode";
    }
    return "?";
}

const char *v8bis_why_name(v8bis_why_t w)
{
    switch (w) {
    case V8BIS_WHY_NONE: return "none";
    case V8BIS_WHY_NAK_RECEIVED: return "NAK received";
    case V8BIS_WHY_NAK_SENT: return "NAK sent";
    case V8BIS_WHY_TIMEOUT: return "timeout";
    case V8BIS_WHY_INVALID_FRAME: return "invalid frame";
    case V8BIS_WHY_NO_COMMON_MODE: return "no common mode";
    }
    return "?";
}

/* ---- capabilities helpers --------------------------------------------- */

static void or_bytes(uint8_t *d, const uint8_t *s, unsigned n)
{
    for (unsigned i = 0; i < n; i++)
        d[i] |= s[i];
}

void v8bis_caps_merge(v8bis_msg_t *dst, const v8bis_msg_t *src)
{
    dst->id_npar1 |= src->id_npar1 & (V8BIS_ID_V8 | V8BIS_ID_SHORT_V8 | V8BIS_ID_NON_STANDARD);
    if (src->network_type) {
        dst->network_type = true;
        dst->network_npar2 |= src->network_npar2;
    }
    dst->s_npar1 |= src->s_npar1;
    dst->s_spar1 |= src->s_spar1;
    or_bytes(dst->data, src->data, 3);
    or_bytes(dst->svd, src->svd, 3);
    dst->h324_npar2 |= src->h324_npar2;
    dst->h324_spar2 |= src->h324_spar2;
    dst->h324_data |= src->h324_data;
    dst->v18 |= src->v18;
    dst->analogue_tel |= src->analogue_tel;
    dst->t101 |= src->t101;
    for (unsigned i = 0; i < src->ns_count && dst->ns_count < V8BIS_NS_BLOCKS_MAX; i++)
        dst->ns[dst->ns_count++] = src->ns[i];
}

static const uint8_t BLOCKS[] = {V8BIS_S_DATA, V8BIS_S_SVD, V8BIS_S_H324,
                                 V8BIS_S_V18, V8BIS_S_ANALOGUE_TEL, V8BIS_S_T101};

static void copy_block(v8bis_msg_t *d, const v8bis_msg_t *s, unsigned bit)
{
    d->s_spar1 |= bit;
    switch (bit) {
    case V8BIS_S_DATA: memcpy(d->data, s->data, 3); break;
    case V8BIS_S_SVD: memcpy(d->svd, s->svd, 3); break;
    case V8BIS_S_H324:
        d->h324_npar2 = s->h324_npar2;
        d->h324_spar2 = s->h324_spar2;
        d->h324_data = s->h324_data;
        break;
    case V8BIS_S_V18: d->v18 = s->v18; break;
    case V8BIS_S_ANALOGUE_TEL: d->analogue_tel = s->analogue_tel; break;
    case V8BIS_S_T101: d->t101 = s->t101; break;
    }
}

static int enc_len(const v8bis_msg_t *m)
{
    uint8_t tmp[V8BIS_MAX_INFO_OCTETS + 1];

    return v8bis_msg_encode(m, tmp, sizeof(tmp));
}

unsigned v8bis_caps_split(const v8bis_msg_t *caps, unsigned max_octets, v8bis_msg_t *segs,
                          unsigned max_segs)
{
    v8bis_msg_t cur;
    unsigned n = 0, blocks_in_cur = 0;

    if (!max_octets || max_octets > V8BIS_MAX_INFO_OCTETS)
        max_octets = V8BIS_MAX_INFO_OCTETS;
    v8bis_msg_init(&cur, V8BIS_MT_CL);
    cur.id_npar1 = caps->id_npar1 & (V8BIS_ID_V8 | V8BIS_ID_SHORT_V8);
    cur.network_type = caps->network_type;
    cur.network_npar2 = caps->network_npar2;
    cur.s_npar1 = caps->s_npar1 & ~V8BIS_S_NPAR1_NON_STANDARD;

    for (unsigned b = 0; b < sizeof(BLOCKS); b++) {
        v8bis_msg_t tmp;

        if (!(caps->s_spar1 & BLOCKS[b]))
            continue;
        tmp = cur;
        copy_block(&tmp, caps, BLOCKS[b]);
        if (blocks_in_cur == 0 || enc_len(&tmp) <= (int)max_octets) {
            cur = tmp;
            blocks_in_cur++;
            continue;
        }
        if (n >= max_segs)
            return 0;
        segs[n++] = cur;
        v8bis_msg_init(&cur, V8BIS_MT_CL);
        copy_block(&cur, caps, BLOCKS[b]);
        blocks_in_cur = 1;
    }
    for (unsigned i = 0; i < caps->ns_count; i++) {
        v8bis_msg_t tmp = cur;

        tmp.id_npar1 |= V8BIS_ID_NON_STANDARD;
        tmp.ns[tmp.ns_count++] = caps->ns[i];
        if (enc_len(&tmp) <= (int)max_octets || tmp.ns_count == 1) {
            if (cur.ns_count == 0 && enc_len(&tmp) > (int)max_octets && blocks_in_cur) {
                /* a lone NS block that does not fit beside the standard blocks gets its own CL */
                if (n >= max_segs)
                    return 0;
                segs[n++] = cur;
                v8bis_msg_init(&cur, V8BIS_MT_CL);
                cur.id_npar1 = V8BIS_ID_NON_STANDARD;
                cur.ns[cur.ns_count++] = caps->ns[i];
                blocks_in_cur = 0;
                continue;
            }
            cur = tmp;
            continue;
        }
        if (n >= max_segs)
            return 0;
        segs[n++] = cur;
        v8bis_msg_init(&cur, V8BIS_MT_CL);
        cur.id_npar1 = V8BIS_ID_NON_STANDARD;
        cur.ns[cur.ns_count++] = caps->ns[i];
        blocks_in_cur = 0;
    }
    if (n >= max_segs)
        return 0;
    segs[n++] = cur;
    for (unsigned i = 0; i + 1 < n; i++)
        segs[i].id_npar1 |= V8BIS_ID_MORE_INFO;
    return n;
}

/* Preference order for the data modulation an MS picks, fastest first. */
static const struct { unsigned octet; uint8_t bit; } MODS[] = {
    {1, V8BIS_DATA2_V34}, {1, V8BIS_DATA2_V32BIS}, {2, V8BIS_DATA3_V32},
    {2, V8BIS_DATA3_V22BIS}, {2, V8BIS_DATA3_V22}, {2, V8BIS_DATA3_V21}};

bool v8bis_default_select_ms(const v8bis_msg_t *ours, const v8bis_msg_t *peer, bool tx_ack1,
                             v8bis_msg_t *ms)
{
    v8bis_msg_t common = *ours;
    bool chosen = false;

    if (peer) {
        common.id_npar1 = ours->id_npar1 & peer->id_npar1;
        common.s_spar1 = ours->s_spar1 & peer->s_spar1;
        for (int i = 0; i < 3; i++) {
            common.data[i] = ours->data[i] & peer->data[i];
            common.svd[i] = ours->svd[i] & peer->svd[i];
        }
        common.analogue_tel = ours->analogue_tel & peer->analogue_tel;
    }
    v8bis_msg_init(ms, V8BIS_MT_MS);
    if (tx_ack1)
        ms->id_npar1 |= V8BIS_ID_TX_ACK1;

    if (common.s_spar1 & V8BIS_S_DATA) {
        for (unsigned i = 0; i < sizeof(MODS) / sizeof(MODS[0]); i++)
            if (common.data[MODS[i].octet] & MODS[i].bit) {
                ms->s_spar1 = V8BIS_S_DATA;
                ms->data[MODS[i].octet] = MODS[i].bit;
                chosen = true;
                /* Error control and compression ride along when both ends have them. */
                ms->data[0] |= common.data[0] & (V8BIS_DATA_V42 | V8BIS_DATA_V42BIS);
                /* Start-up: the short procedure is recommended for V.34 (9.9.2). */
                if ((common.id_npar1 & V8BIS_ID_SHORT_V8) && i == 0)
                    ms->id_npar1 |= V8BIS_ID_SHORT_V8;
                else if (common.id_npar1 & V8BIS_ID_V8)
                    ms->id_npar1 |= V8BIS_ID_V8;
                else if ((common.id_npar1 & V8BIS_ID_SHORT_V8))
                    ms->id_npar1 |= V8BIS_ID_SHORT_V8;
                break;
            }
    }
    if (!chosen && (common.s_spar1 & V8BIS_S_ANALOGUE_TEL) && (common.analogue_tel & V8BIS_TEL_VOICE)) {
        ms->s_spar1 = V8BIS_S_ANALOGUE_TEL;
        ms->analogue_tel = V8BIS_TEL_VOICE;
        chosen = true;
    }
    return chosen;
}

static bool subset_bytes(const uint8_t *a, const uint8_t *b, unsigned n)
{
    for (unsigned i = 0; i < n; i++)
        if (a[i] & ~b[i])
            return false;
    return true;
}

bool v8bis_ms_within_caps(const v8bis_msg_t *ours, const v8bis_msg_t *ms)
{
    if ((ms->id_npar1 & (V8BIS_ID_V8 | V8BIS_ID_SHORT_V8)) & ~ours->id_npar1)
        return false;
    if (ms->s_spar1 & ~ours->s_spar1)
        return false;
    return subset_bytes(ms->data, ours->data, 3) && subset_bytes(ms->svd, ours->svd, 3)
           && !(ms->h324_npar2 & ~ours->h324_npar2) && !(ms->h324_data & ~ours->h324_data)
           && !(ms->v18 & ~ours->v18) && !(ms->analogue_tel & ~ours->analogue_tel)
           && !(ms->t101 & ~ours->t101);
}

v8bis_startup_t v8bis_ms_startup(const v8bis_msg_t *ms)
{
    if (ms->s_spar1 == V8BIS_S_ANALOGUE_TEL)
        return V8BIS_STARTUP_TELEPHONY;
    if (ms->id_npar1 & V8BIS_ID_SHORT_V8)
        return V8BIS_STARTUP_SHORT_V8;
    if (ms->id_npar1 & V8BIS_ID_V8)
        return V8BIS_STARTUP_V8;
    return V8BIS_STARTUP_V25;
}

/* ---- machine ----------------------------------------------------------- */

void v8bis_fsm_cfg_default(v8bis_fsm_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    v8bis_msg_init(&cfg->caps, V8BIS_MT_CL);
    cfg->want_more_info = true;
    cfg->tx_ack1 = true;
    cfg->mode_available = true;
    cfg->mr_reply = V8BIS_MRR_MS;
    cfg->cr_reply = V8BIS_CRR_CL;
}

void v8bis_fsm_init(v8bis_fsm_t *f, const v8bis_fsm_cfg_t *cfg)
{
    unsigned max = cfg->max_info_octets;

    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    f->state = V8BIS_S_INITIAL;
    v8bis_msg_init(&f->peer_caps, V8BIS_MT_CL);
    f->n_segs = v8bis_caps_split(&f->cfg.caps, max, f->segs, V8BIS_MAX_CL_SEGMENTS);
    if (!f->n_segs) {                                   /* does not fit: send what there is */
        f->segs[0] = f->cfg.caps;
        f->segs[0].type = V8BIS_MT_CL;
        f->n_segs = 1;
    }
}

static void push(v8bis_fsm_t *f, const v8bis_action_t *a)
{
    if (f->qn == V8BIS_ACTION_QUEUE) {                  /* never expected: drop the oldest */
        f->qh = (f->qh + 1) % V8BIS_ACTION_QUEUE;
        f->qn--;
    }
    f->q[(f->qh + f->qn) % V8BIS_ACTION_QUEUE] = *a;
    f->q[(f->qh + f->qn) % V8BIS_ACTION_QUEUE].role = f->role;
    f->qn++;
}

bool v8bis_fsm_next_action(v8bis_fsm_t *f, v8bis_action_t *a)
{
    if (!f->qn)
        return false;
    *a = f->q[f->qh];
    f->qh = (f->qh + 1) % V8BIS_ACTION_QUEUE;
    f->qn--;
    return true;
}

static void enter(v8bis_fsm_t *f, v8bis_state_t s)
{
    f->state = s;
    f->elapsed_ms = 0;
}

static void act_signal(v8bis_fsm_t *f, v8bis_signal_t sig, bool responding_set)
{
    v8bis_action_t a;

    memset(&a, 0, sizeof(a));
    a.type = V8BIS_ACT_SIGNAL;
    a.sig = sig;
    a.responding_set = responding_set;
    push(f, &a);
}

static void act_msgs(v8bis_fsm_t *f, v8bis_es_t es, const v8bis_msg_t *m0, const v8bis_msg_t *m1)
{
    v8bis_action_t a;

    memset(&a, 0, sizeof(a));
    a.type = V8BIS_ACT_MESSAGES;
    a.es = es;
    a.es_gap = es != V8BIS_ES_NONE && f->cfg.echo_suppressor;
    a.msg[0] = *m0;
    a.n_msg = 1;
    if (m1) {
        a.msg[1] = *m1;
        a.n_msg = 2;
    }
    push(f, &a);
}

static void act_simple(v8bis_fsm_t *f, unsigned type)
{
    v8bis_msg_t m;

    v8bis_msg_init(&m, type);
    act_msgs(f, V8BIS_ES_NONE, &m, NULL);
}

static void reset_transaction(v8bis_fsm_t *f)
{
    f->role = V8BIS_ROLE_NONE;
    f->got_cl = false;
    f->peer_more_info = false;
    f->seg_next = 0;
    f->our_ms_valid = false;
    f->init_clr = false;
    f->sent_mr_signal = false;
    enter(f, V8BIS_S_INITIAL);
}

static void go_initial(v8bis_fsm_t *f, v8bis_why_t why, unsigned nak)
{
    v8bis_action_t a;

    memset(&a, 0, sizeof(a));
    a.type = V8BIS_ACT_INITIAL;
    a.why = why;
    a.nak = nak;
    push(f, &a);
    reset_transaction(f);
}

void v8bis_fsm_abandon(v8bis_fsm_t *f)
{
    if (f->state != V8BIS_S_MS_MODE)
        reset_transaction(f);
}

static void enter_mode(v8bis_fsm_t *f, bool we_sent_ms, const v8bis_msg_t *ms, bool ack_sent,
                       bool ack_expected)
{
    v8bis_action_t a;

    memset(&a, 0, sizeof(a));
    a.type = V8BIS_ACT_MS_MODE;
    a.mode.we_sent_ms = we_sent_ms;
    a.mode.ms = *ms;
    a.mode.startup = v8bis_ms_startup(ms);
    a.mode.answer_modem = !we_sent_ms;                  /* 9.9 */
    a.mode.ack_sent = ack_sent;
    a.mode.ack_expected = ack_expected;
    a.mode.start_signal_next = !we_sent_ms;
    push(f, &a);
    f->mode_done = true;
    enter(f, V8BIS_S_MS_MODE);
}

/* Choose the MS we send.  False when nothing common can be selected. */
static bool pick_ms(v8bis_fsm_t *f)
{
    const v8bis_msg_t *peer = f->peer_caps_known ? &f->peer_caps : NULL;
    v8bis_msg_t ms;
    bool ok;

    if (f->cfg.select_ms)
        ok = f->cfg.select_ms(f->cfg.user, &f->cfg.caps, peer, &ms);
    else if (!peer && f->cfg.have_preset_ms) {
        ms = f->cfg.preset_ms;
        ok = true;
    } else
        ok = v8bis_default_select_ms(&f->cfg.caps, peer, f->cfg.tx_ack1, &ms);
    if (!ok)
        return false;
    ms.type = V8BIS_MT_MS;
    if (f->cfg.tx_ack1)
        ms.id_npar1 |= V8BIS_ID_TX_ACK1;
    else
        ms.id_npar1 &= (uint8_t)~V8BIS_ID_TX_ACK1;
    f->our_ms = ms;
    f->our_ms_valid = true;
    f->our_ms_wants_ack = f->cfg.tx_ack1;
    return true;
}

/* Our next CL segment, as the given type. */
static v8bis_msg_t next_cl(v8bis_fsm_t *f, unsigned type)
{
    unsigned idx = f->seg_next < f->n_segs ? f->seg_next : f->n_segs - 1;
    v8bis_msg_t m = f->segs[idx];

    m.type = type;
    if (f->seg_next < f->n_segs)
        f->seg_next++;
    return m;
}

static void send_ms(v8bis_fsm_t *f, v8bis_es_t es)
{
    act_msgs(f, es, &f->our_ms, NULL);
}

static void send_cl_ms(v8bis_fsm_t *f)
{
    v8bis_msg_t cl = next_cl(f, V8BIS_MT_CL);

    act_msgs(f, V8BIS_ES_NONE, &cl, &f->our_ms);
}

static bool want_more(const v8bis_fsm_t *f, const v8bis_msg_t *m)
{
    return (m->id_npar1 & V8BIS_ID_MORE_INFO) && f->cfg.want_more_info;
}

static void learn_caps(v8bis_fsm_t *f, const v8bis_msg_t *m)
{
    v8bis_caps_merge(&f->peer_caps, m);
    f->peer_caps_known = true;
    f->peer_more_info = (m->id_npar1 & V8BIS_ID_MORE_INFO) != 0;
}

/* We received an MS: acknowledge it (9.5, 9.7).  `force_ack1` is the CL-MS
 * rule of clause 9.10: a response is required whatever the ACK(1) bit says. */
static void handle_ms(v8bis_fsm_t *f, const v8bis_msg_t *ms, bool force_ack1)
{
    v8bis_accept_t verdict;

    f->peer_ms = *ms;
    if (f->cfg.accept)
        verdict = f->cfg.accept(f->cfg.user, ms);
    else if (!f->cfg.mode_available)
        verdict = V8BIS_ACCEPT_NAK2;
    else
        verdict = v8bis_ms_within_caps(&f->cfg.caps, ms) ? V8BIS_ACCEPT_ACK : V8BIS_ACCEPT_NAK3;

    if (verdict == V8BIS_ACCEPT_ACK) {
        bool ack = (ms->id_npar1 & V8BIS_ID_TX_ACK1) || force_ack1;

        if (ack)
            act_simple(f, V8BIS_MT_ACK1);
        enter_mode(f, false, ms, ack, false);
    } else {
        unsigned n = verdict == V8BIS_ACCEPT_NAK2 ? 2 : 3;

        act_simple(f, n == 2 ? V8BIS_MT_NAK2 : V8BIS_MT_NAK3);
        go_initial(f, V8BIS_WHY_NAK_SENT, n);
    }
}

static void fail_no_mode(v8bis_fsm_t *f)
{
    go_initial(f, V8BIS_WHY_NO_COMMON_MODE, 0);
}

bool v8bis_fsm_initiate(v8bis_fsm_t *f, v8bis_init_t how)
{
    bool e = f->cfg.answering_station && f->cfg.auto_answer_call;
    v8bis_msg_t m;

    if (f->state != V8BIS_S_INITIAL)
        return false;
    f->role = V8BIS_ROLE_INITIATOR;
    f->seg_next = 0;
    switch (how) {
    case V8BIS_INIT_MR:
        act_signal(f, e ? V8BIS_SIG_MRE : V8BIS_SIG_MRD, false);
        enter(f, V8BIS_S_SENT_MR);
        break;
    case V8BIS_INIT_CR:
        act_signal(f, e ? V8BIS_SIG_CRE : V8BIS_SIG_CRD, false);
        enter(f, V8BIS_S_SENT_CR);
        break;
    case V8BIS_INIT_MS:
        if (!pick_ms(f)) {
            fail_no_mode(f);
            return true;
        }
        send_ms(f, V8BIS_ES_ESI);
        enter(f, V8BIS_S_SENT_MS);
        break;
    case V8BIS_INIT_CL:
        m = next_cl(f, V8BIS_MT_CL);
        act_msgs(f, V8BIS_ES_ESI, &m, NULL);
        enter(f, V8BIS_S_SENT_CL);
        break;
    case V8BIS_INIT_CLR:
        /* Table 7 transaction 6: CLR -> CL -> MS -> ACK/NAK.  The reply is a plain
         * CL that we answer with MS, which is the "Sent CR" state's CL/MS. */
        m = next_cl(f, V8BIS_MT_CLR);
        f->init_clr = true;
        act_msgs(f, V8BIS_ES_ESI, &m, NULL);
        enter(f, V8BIS_S_SENT_CR);
        break;
    default:
        f->role = V8BIS_ROLE_NONE;
        return false;
    }
    return true;
}

/* A CRd arrived mid transaction: answer it with our capabilities, CL or CLR. */
static void reply_crd(v8bis_fsm_t *f)
{
    v8bis_msg_t m;

    f->seg_next = 0;
    if (f->cfg.on_crd_send_clr) {
        m = next_cl(f, V8BIS_MT_CLR);
        act_msgs(f, V8BIS_ES_NONE, &m, NULL);
        enter(f, V8BIS_S_SENT_CLR);
    } else {
        m = next_cl(f, V8BIS_MT_CL);
        act_msgs(f, V8BIS_ES_NONE, &m, NULL);
        enter(f, V8BIS_S_SENT_CL);
    }
}

void v8bis_fsm_signal(v8bis_fsm_t *f, v8bis_signal_t sig, bool responding_set)
{
    bool auto_ok = f->cfg.auto_answer_call;
    v8bis_msg_t m;

    (void)responding_set;
    if (sig == V8BIS_SIG_ESI || sig == V8BIS_SIG_ESR)
        return;                                         /* the message that follows is the event */

    if (f->state == V8BIS_S_INITIAL) {
        v8bis_es_t es = auto_ok ? V8BIS_ES_ESR : V8BIS_ES_NONE;
        bool dotted_ok = auto_ok && (sig == V8BIS_SIG_MRE || sig == V8BIS_SIG_CRE);

        if (sig == V8BIS_SIG_MRE || sig == V8BIS_SIG_MRD) {
            v8bis_mr_reply_t r = f->cfg.mr_reply;

            if (r != V8BIS_MRR_MS && !dotted_ok)
                r = V8BIS_MRR_MS;
            f->role = V8BIS_ROLE_RESPONDER;
            f->seg_next = 0;
            switch (r) {
            case V8BIS_MRR_MS:
                if (!pick_ms(f)) {
                    fail_no_mode(f);
                    return;
                }
                send_ms(f, es);
                enter(f, V8BIS_S_SENT_MS);
                break;
            case V8BIS_MRR_MRD:
                act_signal(f, V8BIS_SIG_MRD, true);
                enter(f, V8BIS_S_SENT_CL);              /* "Sent CL or MR" */
                f->sent_mr_signal = true;
                break;
            case V8BIS_MRR_CRD:
                act_signal(f, V8BIS_SIG_CRD, true);
                enter(f, V8BIS_S_SENT_CR);
                break;
            }
        } else if (sig == V8BIS_SIG_CRE || sig == V8BIS_SIG_CRD) {
            v8bis_cr_reply_t r = f->cfg.cr_reply;

            if (r == V8BIS_CRR_CRD && !dotted_ok)
                r = V8BIS_CRR_CL;
            f->role = V8BIS_ROLE_RESPONDER;
            f->seg_next = 0;
            switch (r) {
            case V8BIS_CRR_CL:
                m = next_cl(f, V8BIS_MT_CL);
                act_msgs(f, es, &m, NULL);
                enter(f, V8BIS_S_SENT_CL);
                break;
            case V8BIS_CRR_CLR:
                m = next_cl(f, V8BIS_MT_CLR);
                act_msgs(f, es, &m, NULL);
                enter(f, V8BIS_S_SENT_CLR);
                break;
            case V8BIS_CRR_CRD:
                act_signal(f, V8BIS_SIG_CRD, true);
                enter(f, V8BIS_S_SENT_CR);
                break;
            }
        }
        return;
    }

    /* Mid transaction. */
    if (f->role == V8BIS_ROLE_INITIATOR) {
        if (f->state == V8BIS_S_SENT_MR && sig == V8BIS_SIG_MRD) {
            if (f->cfg.on_mrd_ask_caps && auto_ok) {            /* MRd/CRd: transactions 8 and 9 */
                act_signal(f, V8BIS_SIG_CRD, false);
                enter(f, V8BIS_S_SENT_CR);
            } else {                                            /* MRd/MS: transaction 7 */
                if (!pick_ms(f)) {
                    fail_no_mode(f);
                    return;
                }
                send_ms(f, V8BIS_ES_NONE);
                enter(f, V8BIS_S_SENT_MS);
            }
        } else if ((f->state == V8BIS_S_SENT_MR || f->state == V8BIS_S_SENT_CR)
                   && sig == V8BIS_SIG_CRD) {
            reply_crd(f);
        }
    } else if (f->role == V8BIS_ROLE_RESPONDER) {
        if (f->state == V8BIS_S_SENT_CL && sig == V8BIS_SIG_CRD && f->sent_mr_signal) {
            reply_crd(f);                               /* CRd/CL or CRd/CLR */
        }
    }
}

void v8bis_fsm_message(v8bis_fsm_t *f, const v8bis_msg_t *m)
{
    v8bis_msg_t out;

    if (!m->known_type || f->state == V8BIS_S_MS_MODE)
        return;
    /* NAK(1) means the peer could not read what we sent (9.8).  The diagrams
     * only draw NAK after an MS, but a station that has already returned to the
     * Initial State must not be left waiting for a reply that is not coming. */
    if (m->type == V8BIS_MT_NAK1 && f->state != V8BIS_S_INITIAL && f->state != V8BIS_S_SENT_MS) {
        go_initial(f, V8BIS_WHY_NAK_RECEIVED, 1);
        return;
    }

    switch (f->state) {
    case V8BIS_S_INITIAL:
        if (m->type == V8BIS_MT_MS) {
            f->role = V8BIS_ROLE_RESPONDER;
            handle_ms(f, m, false);
        } else if (m->type == V8BIS_MT_CL) {
            learn_caps(f, m);
            if (want_more(f, m)) {
                act_simple(f, V8BIS_MT_ACK2);           /* CL/ACK(2): stay in Initial */
                enter(f, V8BIS_S_INITIAL);
                break;
            }
            f->role = V8BIS_ROLE_RESPONDER;
            if (!pick_ms(f)) {
                fail_no_mode(f);
                break;
            }
            send_ms(f, V8BIS_ES_NONE);
            enter(f, V8BIS_S_SENT_MS);
        } else if (m->type == V8BIS_MT_CLR) {
            learn_caps(f, m);
            if (want_more(f, m)) {
                act_simple(f, V8BIS_MT_ACK2);
                enter(f, V8BIS_S_INITIAL);
                break;
            }
            f->role = V8BIS_ROLE_RESPONDER;
            f->seg_next = 0;
            out = next_cl(f, V8BIS_MT_CL);
            act_msgs(f, V8BIS_ES_NONE, &out, NULL);
            enter(f, V8BIS_S_SENT_CL);
        }
        break;

    case V8BIS_S_SENT_MR:
        if (m->type == V8BIS_MT_MS)
            handle_ms(f, m, false);                     /* MS/ACK, MS/NAK */
        break;

    case V8BIS_S_SENT_CR:
        if (m->type == V8BIS_MT_ACK2 && f->init_clr) {
            out = next_cl(f, V8BIS_MT_CLR);             /* ACK(2)/CLR: the rest of our list */
            act_msgs(f, V8BIS_ES_NONE, &out, NULL);
            enter(f, V8BIS_S_SENT_CR);
        } else if (m->type == V8BIS_MT_CL) {
            learn_caps(f, m);
            if (want_more(f, m)) {
                act_simple(f, V8BIS_MT_ACK2);
                enter(f, V8BIS_S_SENT_CR);
                break;
            }
            if (!pick_ms(f)) {
                fail_no_mode(f);
                break;
            }
            send_ms(f, V8BIS_ES_NONE);                  /* CL/MS */
            enter(f, V8BIS_S_SENT_MS);
        } else if (m->type == V8BIS_MT_CLR) {
            learn_caps(f, m);
            if (want_more(f, m)) {
                act_simple(f, V8BIS_MT_ACK2);
                enter(f, V8BIS_S_SENT_CR);
                break;
            }
            if (!pick_ms(f)) {
                fail_no_mode(f);
                break;
            }
            f->seg_next = 0;
            send_cl_ms(f);                              /* CLR/CL-MS */
            enter(f, V8BIS_S_SENT_MS);
        }
        break;

    case V8BIS_S_SENT_MS:
        if (m->type == V8BIS_MT_ACK1) {
            enter_mode(f, true, &f->our_ms, false, true);
        } else if (m->type == V8BIS_MT_ACK2) {
            /* CL-MS with more to say: send the next segment with the same MS */
            if (f->seg_next < f->n_segs && f->our_ms_valid) {
                send_cl_ms(f);
                enter(f, V8BIS_S_SENT_MS);
            }
        } else if (m->type == V8BIS_MT_NAK1 || m->type == V8BIS_MT_NAK2 || m->type == V8BIS_MT_NAK3) {
            go_initial(f, V8BIS_WHY_NAK_RECEIVED, m->type == V8BIS_MT_NAK1 ? 1 : m->type == V8BIS_MT_NAK2 ? 2 : 3);
        }
        break;

    case V8BIS_S_SENT_CL:
        if (m->type == V8BIS_MT_MS) {
            handle_ms(f, m, false);
        } else if (m->type == V8BIS_MT_ACK2) {
            out = next_cl(f, V8BIS_MT_CL);              /* ACK(2)/CL */
            act_msgs(f, V8BIS_ES_NONE, &out, NULL);
            enter(f, V8BIS_S_SENT_CL);
        }
        break;

    case V8BIS_S_SENT_CLR:
        if (m->type == V8BIS_MT_CL) {
            learn_caps(f, m);
            f->got_cl = true;
        } else if (m->type == V8BIS_MT_MS) {
            bool had_cl = f->got_cl, more = f->peer_more_info;

            f->got_cl = false;
            if (had_cl && more && f->cfg.want_more_info) {
                act_simple(f, V8BIS_MT_ACK2);           /* ask for the rest of the CL-MS */
                f->peer_more_info = false;
                enter(f, V8BIS_S_SENT_CLR);
            } else {
                handle_ms(f, m, had_cl && more);        /* 9.10: ACK(1) is required here */
            }
        } else if (m->type == V8BIS_MT_ACK2) {
            out = next_cl(f, V8BIS_MT_CLR);             /* ACK(2)/CLR */
            act_msgs(f, V8BIS_ES_NONE, &out, NULL);
            enter(f, V8BIS_S_SENT_CLR);
        }
        break;

    default:
        break;
    }
}

void v8bis_fsm_invalid_frame(v8bis_fsm_t *f)
{
    if (f->state == V8BIS_S_MS_MODE)
        return;
    act_simple(f, V8BIS_MT_NAK1);                       /* 9.8 */
    go_initial(f, V8BIS_WHY_INVALID_FRAME, 1);
}

void v8bis_fsm_startup_signal(v8bis_fsm_t *f)
{
    /* 9.7: the MS receiver omitted ACK(1) and began ANS/ANSam. */
    if (f->state == V8BIS_S_SENT_MS && f->our_ms_valid && !f->our_ms_wants_ack)
        enter_mode(f, true, &f->our_ms, false, false);
}

void v8bis_fsm_tick(v8bis_fsm_t *f, unsigned ms)
{
    if (f->state == V8BIS_S_INITIAL || f->state == V8BIS_S_MS_MODE)
        return;
    f->elapsed_ms += ms;
    if (f->elapsed_ms >= V8BIS_STATE_TIMEOUT_MS)
        go_initial(f, V8BIS_WHY_TIMEOUT, 0);
}
