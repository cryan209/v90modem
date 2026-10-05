/*
 * v76.c -- ITU-T V.76 (08/96) + Cor.1 (01/2005) multiplex function.
 * See v76.h for what is and is not covered.  Clause numbers in comments are
 * V.76's unless prefixed.
 *
 * Structure
 *   framer      tx_refill() / rx_put_bit(): flags, 0-bit insertion, abort,
 *               Annex A suspend/resume, FCS handling
 *   frame layer build_frame() / parse_frame()
 *   DLC layer   rx_dispatch() and the per-DLC procedures (7.x, 8.x)
 *   pump        pick_next(): which frame goes out at the next boundary
 *
 * Frames that must carry fresh sequence numbers (I frames, supervisory
 * frames) are built when they are picked for transmission, not when the
 * decision to send them is taken: an N(R) taken at decision time can be
 * stale by the time the line is free.
 */

#include "v76.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------ *
 * FCS (5.1.6)
 *
 * All three are the reflected CRCs of HDLC: the generator polynomial is run
 * with the register preset to all ones and the result complemented, and the
 * register's low-order bit is the highest power of x, which is why the
 * wire format of 5.2.1.2 Figure 5 ("high-order bit is bit 1 of the first
 * octet") is simply the register sent low byte first.
 * ------------------------------------------------------------------------ */

static uint32_t fcs_poly(int fcs_len)
{
    switch (fcs_len) {
    case V76_FCS_8:  return 0xE0u;          /* x^8 + x^2 + x + 1, reflected */
    case V76_FCS_32: return 0xEDB88320u;    /* x^32 + x^26 + ... + x + 1 */
    default:         return 0x8408u;        /* x^16 + x^12 + x^5 + 1 */
    }
}

static uint32_t fcs_init(int fcs_len)
{
    return fcs_len == V76_FCS_8 ? 0xFFu : fcs_len == V76_FCS_32 ? 0xFFFFFFFFu : 0xFFFFu;
}

static uint32_t fcs_mask(int fcs_len)
{
    return fcs_len == V76_FCS_8 ? 0xFFu : fcs_len == V76_FCS_32 ? 0xFFFFFFFFu : 0xFFFFu;
}

static uint32_t crc_run(int fcs_len, uint32_t crc, const uint8_t *d, int n)
{
    uint32_t poly = fcs_poly(fcs_len);
    int i, b;

    for (i = 0; i < n; i++) {
        crc ^= d[i];
        for (b = 0; b < 8; b++)
            crc = (crc & 1u) ? (crc >> 1) ^ poly : (crc >> 1);
    }
    return crc;
}

uint32_t v76_fcs(int fcs_len, const uint8_t *data, int len)
{
    return ~crc_run(fcs_len, fcs_init(fcs_len), data, len) & fcs_mask(fcs_len);
}

uint32_t v76_fcs_residue(int fcs_len, const uint8_t *d, int len)
{
    return crc_run(fcs_len, fcs_init(fcs_len), d, len);
}

/* 5.1.6.1-3 give the error-free remainder with x^7 / x^15 / x^31 as the
 * first listed bit; in the reflected register that is the bit-reversed value. */
uint32_t v76_fcs_expected_residue(int fcs_len)
{
    switch (fcs_len) {
    case V76_FCS_8:  return 0xCFu;          /* 1111 0011 (x7..x0) reflected */
    case V76_FCS_32: return 0xDEBB20E3u;    /* 1100 0111 0000 0100 1101 1101 0111 1011 */
    default:         return 0xF0B8u;        /* 0001 1101 0000 1111 */
    }
}

static void fcs_put(uint8_t *out, int fcs_len, uint32_t fcs)
{
    int i;

    for (i = 0; i < fcs_len; i++)
        out[i] = (uint8_t)(fcs >> (8 * i));
}

static uint32_t fcs_get(const uint8_t *in, int fcs_len)
{
    uint32_t v = 0;
    int i;

    for (i = 0; i < fcs_len; i++)
        v |= (uint32_t)in[i] << (8 * i);
    return v;
}

/* ------------------------------------------------------------------------ *
 * Control field codes (Table 4)
 * ------------------------------------------------------------------------ */

enum {
    FT_I, FT_RR, FT_RNR, FT_REJ, FT_SREJ,
    FT_SABME, FT_DM, FT_UI, FT_UIH, FT_DISC, FT_UA, FT_FRMR, FT_XID, FT_TEST,
    FT_BAD
};

#define U_SABME 0x6F
#define U_DM    0x0F
#define U_UI    0x03
#define U_UIH   0xEF
#define U_DISC  0x43
#define U_UA    0x63
#define U_FRMR  0x87
#define U_XID   0xAF
#define U_TEST  0xE3
#define U_PF    0x10

#define S_RR    0x01
#define S_RNR   0x05
#define S_REJ   0x09
#define S_SREJ  0x0D

void v76_dlc_params_default(v76_dlc_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->mode = V76_ERM;
    p->k = 15;                  /* Annex C feature 11 */
    p->k_rx = 0;
    p->n401_tx = 128;           /* Annex C feature 10 */
    p->n401_rx = 128;
    p->recovery = V76_REC_REJ;
    p->uih_protect = 4;         /* Annex C features 6, 7 */
    p->fcs_len = V76_FCS_16;
    p->addr_octets = 1;
}

void v76_config_default(v76_config_t *c, bool initiator)
{
    memset(c, 0, sizeof(*c));
    c->initiator = initiator;
    c->t401_ms = 1000;
    c->n400 = 10;
    c->addr_octets = 1;
    c->fcs_support = V76_FCS_MASK_16;
}

/* ------------------------------------------------------------------------ *
 * State
 * ------------------------------------------------------------------------ */

typedef struct sdu_s {
    struct sdu_s *next;
    int len;
    bool is_resp;
    bool pf;
    uint8_t d[1];               /* over-allocated */
} sdu_t;

typedef struct txreq_s {
    struct txreq_s *next;
    int kind;                   /* FT_* */
    int dlci;
    bool cmd;                   /* command or response */
    bool pf;
    int addr_octets;
    int fcs_len;
    uint8_t *ud;
    int ud_len;
    uint8_t frmr[5];            /* FRMR information field */
} txreq_t;

typedef struct {
    bool used;
    int dlci;
    v76_dlc_state_t st;
    v76_dlc_params_t p;
    int k_rx;
    bool opener;
    bool rx_pf;                 /* acceptor: P bit of the SABME to answer */
    int fcs_len, addr_octets;

    /* mode-setting procedure (SABME / DISC) */
    int t_proc, n_proc;
    uint8_t *proc_ud;
    int proc_ud_len;
    bool sent_disc;             /* what t_proc is retransmitting */
    bool ua_for_colliding;      /* 7.5.1 collision: UA sent, waiting for ours */

    /* XID procedure */
    int t_xid, n_xid;
    uint8_t *xid_ud;
    int xid_len;
    bool xid_pending;

    /* ERM */
    int vs, vs_hi, va, vr;
    sdu_t *slot[128];
    sdu_t *q_head, *q_tail;
    int q_len;
    bool peer_busy, own_busy, rej_exc, tmr_rec, poll_out, ack_pending;
    int t401, retrans, t403;
    bool srej_retx[128];
    sdu_t *held[128];
    bool srej_sent[128];

    /* UNERM / UI */
    sdu_t *ui_head, *ui_tail;
    int ui_len;
} dlc_t;

typedef struct {
    uint8_t buf[V76_FRAME_MAX + 16];
    int len, pos;
    bool active;
    bool rt_nocontrol;          /* suspend-format real-time frame */
    bool is_rt;                 /* a real-time DLC's frame: never suspended */
    bool max_rt;                /* info field of exactly N401RT octets */
} txframe_t;

enum { SR_NORMAL, SR_SUSPEND, SR_ABORT };

struct v76_s {
    v76_config_t cfg;
    v76_su_t su;
    v76_stats_t st;
    dlc_t dlc[V76_MAX_DLC];

    txreq_t *q_head, *q_tail;
    int rr;                     /* round-robin cursor */

    /* transmit bit generator */
    uint64_t bq;
    int bq_n;
    int tx_ones;
    txframe_t cur, nrt;
    bool tx_suspended;
    int clk_acc;

    /* receive bit assembler */
    uint8_t rbuf[2 * V76_FRAME_MAX + 64];
    int rbits;
    int rx_ones;
    bool rx_lead0;              /* the last bit pushed before the ones run was a 0 */
    bool rx_last_pushed_zero;
    bool rx_run_lead0;
    int rx_sr;
    uint8_t nrt_buf[2 * V76_FRAME_MAX + 64];
    int nrt_bits;
    bool nrt_valid;
    int rt_total_bits;          /* >0 once the RT frame's full length is known */
    int rt_tail;                /* bits received past a full-length RT frame */
    bool rt_tail_flaglike;
    bool rx_hunting;            /* basic mode after an abort: wait for a flag */
};

/* ------------------------------------------------------------------------ *
 * Small helpers
 * ------------------------------------------------------------------------ */

static void su_violation(v76_t *mf, int dlci, const char *what)
{
    if (mf->su.violation)
        mf->su.violation(mf->su.ctx, dlci, what);
}

static sdu_t *sdu_new(const uint8_t *d, int len)
{
    sdu_t *s = malloc(sizeof(*s) + (size_t)(len > 0 ? len : 0));

    if (!s)
        return NULL;
    s->next = NULL;
    s->len = len;
    s->is_resp = false;
    s->pf = false;
    if (len > 0)
        memcpy(s->d, d, (size_t)len);
    return s;
}

static dlc_t *dlc_find(v76_t *mf, int dlci)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++)
        if (mf->dlc[i].used && mf->dlc[i].dlci == dlci)
            return &mf->dlc[i];
    return NULL;
}

static dlc_t *dlc_alloc(v76_t *mf, int dlci)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++) {
        if (!mf->dlc[i].used) {
            memset(&mf->dlc[i], 0, sizeof(mf->dlc[i]));
            mf->dlc[i].used = true;
            mf->dlc[i].dlci = dlci;
            mf->dlc[i].t_proc = mf->dlc[i].t_xid = -1;
            mf->dlc[i].t401 = mf->dlc[i].t403 = -1;
            return &mf->dlc[i];
        }
    }
    return NULL;
}

static void sdu_list_free(sdu_t *s)
{
    while (s) {
        sdu_t *n = s->next;

        free(s);
        s = n;
    }
}

static void dlc_clear_erm(dlc_t *d)
{
    int i;

    for (i = 0; i < 128; i++) {
        free(d->slot[i]);
        d->slot[i] = NULL;
        free(d->held[i]);
        d->held[i] = NULL;
        d->srej_retx[i] = false;
        d->srej_sent[i] = false;
    }
    sdu_list_free(d->q_head);
    d->q_head = d->q_tail = NULL;
    d->q_len = 0;
    d->vs = d->vs_hi = d->va = d->vr = 0;
    d->peer_busy = d->own_busy = d->rej_exc = d->tmr_rec = false;
    d->poll_out = d->ack_pending = false;
    d->t401 = -1;
    d->retrans = 0;
}

static void dlc_release_slot(dlc_t *d)
{
    dlc_clear_erm(d);
    sdu_list_free(d->ui_head);
    d->ui_head = d->ui_tail = NULL;
    d->ui_len = 0;
    free(d->proc_ud);
    free(d->xid_ud);
    memset(d, 0, sizeof(*d));
}

static int addr_max_dlci(int addr_octets)
{
    return addr_octets >= 2 ? 8191 : 63;
}

/* 6.1.1 + Cor.1: initiator takes the lowest free DLCI, responder the highest. */
static int dlci_alloc(v76_t *mf, int addr_octets)
{
    int max = addr_max_dlci(addr_octets);
    int dlci;

    if (mf->cfg.initiator) {
        for (dlci = 0; dlci <= max; dlci++)
            if (!dlc_find(mf, dlci))
                return dlci;
    } else {
        for (dlci = max; dlci >= 0; dlci--)
            if (!dlc_find(mf, dlci))
                return dlci;
    }
    return -1;
}

/* ------------------------------------------------------------------------ *
 * Frame build / parse
 * ------------------------------------------------------------------------ */

static int put_addr(uint8_t *out, int addr_octets, int dlci, bool cr)
{
    if (addr_octets >= 2) {
        out[0] = (uint8_t)(((dlci >> 7) & 0x3F) << 2 | (cr ? 2 : 0));
        out[1] = (uint8_t)(((dlci & 0x7F) << 1) | 1);
        return 2;
    }
    out[0] = (uint8_t)(((dlci & 0x3F) << 2) | (cr ? 2 : 0) | 1);
    return 1;
}

int v76_build_frame(uint8_t *out, int addr_octets, int dlci, bool cr,
                    const uint8_t *ctl, int ctl_len,
                    const uint8_t *info, int info_len,
                    int fcs_len, int protect)
{
    int n = put_addr(out, addr_octets, dlci, cr);
    int cover;

    memcpy(out + n, ctl, (size_t)ctl_len);
    n += ctl_len;
    if (info_len > 0)
        memcpy(out + n, info, (size_t)info_len);
    n += info_len;
    cover = (protect > 0 && protect < n) ? protect : n;
    fcs_put(out + n, fcs_len, v76_fcs(fcs_len, out, cover));
    return n + fcs_len;
}

typedef struct {
    int type;
    int dlci;
    bool cmd;
    bool pf;
    int ns, nr;
    const uint8_t *info;
    int info_len;
    int fcs_len;
    int addr_octets;
    bool hdr_only;
} rxframe_t;

/* Does `fcs_len` check out on this candidate frame?  Returns the parsed
 * frame in *f on success. */
static bool parse_with_fcs(v76_t *mf, const uint8_t *b, int n, int fcs_len,
                           rxframe_t *f)
{
    int a = 1, c, hdr, body;
    uint8_t c1;
    int type;
    int cover;
    dlc_t *d;

    if (n < 3)
        return false;
    if (!(b[0] & 1)) {
        if (n < 4 || !(b[1] & 1))
            return false;               /* >2 octet address, 5.3 e) */
        a = 2;
    }
    c1 = b[a];
    if (!(c1 & 1))
        type = FT_I, c = 2;
    else if ((c1 & 3) == 1)
        type = FT_RR, c = 2;            /* refined below */
    else
        type = FT_BAD, c = 1;
    if (type == FT_RR) {
        switch (c1) {
        case S_RR:   type = FT_RR; break;
        case S_RNR:  type = FT_RNR; break;
        case S_REJ:  type = FT_REJ; break;
        case S_SREJ: type = FT_SREJ; break;
        default:     type = FT_BAD; break;
        }
    } else if (type == FT_BAD) {
        switch (c1 & ~U_PF) {
        case U_SABME: type = FT_SABME; break;
        case U_DM:    type = FT_DM; break;
        case U_UI:    type = FT_UI; break;
        case U_UIH:   type = FT_UIH; break;
        case U_DISC:  type = FT_DISC; break;
        case U_UA:    type = FT_UA; break;
        case U_FRMR:  type = FT_FRMR; break;
        case U_XID:   type = FT_XID; break;
        case U_TEST:  type = FT_TEST; break;
        default:      type = FT_BAD; break;
        }
    }
    hdr = a + c;
    if (n < hdr + fcs_len)
        return false;
    body = n - fcs_len;

    f->dlci = a == 2 ? (((b[0] >> 2) & 0x3F) << 7) | (b[1] >> 1) : (b[0] >> 2) & 0x3F;
    cover = body;
    if (type == FT_UIH) {
        d = dlc_find(mf, f->dlci);
        if (d && d->p.uih && d->p.uih_protect > 0 && d->p.uih_protect < body)
            cover = d->p.uih_protect;
    }
    if (v76_fcs(fcs_len, b, cover) != fcs_get(b + body, fcs_len))
        return false;

    f->type = type;
    f->hdr_only = cover < body;
    f->addr_octets = a;
    f->fcs_len = fcs_len;
    f->nr = f->ns = 0;
    f->pf = false;
    if (type == FT_I) {
        f->ns = c1 >> 1;
        f->nr = b[a + 1] >> 1;
        f->pf = b[a + 1] & 1;
    } else if (type == FT_RR || type == FT_RNR || type == FT_REJ || type == FT_SREJ) {
        f->nr = b[a + 1] >> 1;
        f->pf = b[a + 1] & 1;
    } else if (type != FT_BAD) {
        f->pf = (c1 & U_PF) != 0;
    }
    f->info = b + hdr;
    f->info_len = body - hdr;
    {
        bool cr = (b[0] >> 1) & 1;

        /* Table 2: a command from the initiator carries C/R = 1, from the
         * responder 0.  We are the opposite end of whoever sent this. */
        f->cmd = (cr == (mf->cfg.initiator ? false : true));
    }
    return true;
}

/* ------------------------------------------------------------------------ *
 * Transmit queue of control frames
 * ------------------------------------------------------------------------ */

static void txq_push(v76_t *mf, int kind, int dlci, bool cmd, bool pf,
                     int addr_octets, int fcs_len, const uint8_t *ud, int ud_len)
{
    txreq_t *r = calloc(1, sizeof(*r));

    if (!r)
        return;
    r->kind = kind;
    r->dlci = dlci;
    r->cmd = cmd;
    r->pf = pf;
    r->addr_octets = addr_octets;
    r->fcs_len = fcs_len;
    if (ud_len > 0 && ud) {
        r->ud = malloc((size_t)ud_len);
        if (r->ud) {
            memcpy(r->ud, ud, (size_t)ud_len);
            r->ud_len = ud_len;
        }
    }
    if (mf->q_tail)
        mf->q_tail->next = r;
    else
        mf->q_head = r;
    mf->q_tail = r;
}

static void txq_push_dlc(v76_t *mf, dlc_t *d, int kind, bool cmd, bool pf,
                         const uint8_t *ud, int ud_len)
{
    txq_push(mf, kind, d->dlci, cmd, pf, d->addr_octets, d->fcs_len, ud, ud_len);
}

/* Remove queued supervisory frames for a DLC that is going away. */
static void txq_drop_dlc_sup(v76_t *mf, int dlci)
{
    txreq_t **pp = &mf->q_head, *prev = NULL;

    while (*pp) {
        txreq_t *r = *pp;

        if (r->dlci == dlci &&
            (r->kind == FT_RR || r->kind == FT_RNR || r->kind == FT_REJ ||
             r->kind == FT_SREJ)) {
            *pp = r->next;
            if (mf->q_tail == r)
                mf->q_tail = prev;
            free(r->ud);
            free(r);
            continue;
        }
        prev = r;
        pp = &r->next;
    }
}

/* ------------------------------------------------------------------------ *
 * Releasing a DLC
 * ------------------------------------------------------------------------ */

static void dlc_terminate(v76_t *mf, dlc_t *d, v76_release_reason_t why,
                          const uint8_t *ud, int ud_len)
{
    int dlci = d->dlci;

    txq_drop_dlc_sup(mf, dlci);
    dlc_release_slot(d);
    if (mf->su.release_ind)
        mf->su.release_ind(mf->su.ctx, dlci, ud, ud_len, why);
}

/* ------------------------------------------------------------------------ *
 * ERM machinery (8.1)
 * ------------------------------------------------------------------------ */

static int seq_dist(int from, int to)
{
    return (to - from) & 127;
}

static bool nr_valid(const dlc_t *d, int nr)
{
    return seq_dist(d->va, nr) <= seq_dist(d->va, d->vs_hi);
}

static void t401_update(v76_t *mf, dlc_t *d, bool progress)
{
    bool need = d->va != d->vs_hi || d->poll_out || d->peer_busy;

    if (!need) {
        d->t401 = -1;
        if (d->st == V76_DLC_CONNECTED && mf->cfg.t403_ms > 0)
            d->t403 = mf->cfg.t403_ms;
        return;
    }
    d->t403 = -1;
    if (progress || d->t401 < 0)
        d->t401 = mf->cfg.t401_ms;
}

static void ack_to(dlc_t *d, int nr)
{
    /* After a go-back-N rewind V(S) can sit behind what is now acknowledged;
     * it may not lag V(A). */
    if (seq_dist(d->va, d->vs) < seq_dist(d->va, nr))
        d->vs = nr;
    while (d->va != nr) {
        free(d->slot[d->va]);
        d->slot[d->va] = NULL;
        d->srej_retx[d->va] = false;
        d->va = (d->va + 1) & 127;
    }
}

static void send_sup(v76_t *mf, dlc_t *d, bool cmd, bool pf, bool force_rr)
{
    int kind = d->own_busy ? FT_RNR : FT_RR;

    (void)force_rr;
    txq_push_dlc(mf, d, kind, cmd, pf, NULL, 0);
}

static void send_poll(v76_t *mf, dlc_t *d)
{
    send_sup(mf, d, true, true, false);
    d->poll_out = true;
    d->t401 = mf->cfg.t401_ms;
    d->t403 = -1;
}

static void erm_enter_recovery(v76_t *mf, dlc_t *d)
{
    d->tmr_rec = true;
    d->retrans = 0;
    send_poll(mf, d);
}

/* 8.1.3-8.1.6: everything a valid I or S frame's N(R) and P/F bit do. */
static void erm_rx_nr(v76_t *mf, dlc_t *d, const rxframe_t *f)
{
    int old_va = d->va;
    bool rec = d->tmr_rec;
    bool progress;

    /* A single-SREJ's N(R) names the frame wanted; it acknowledges nothing
     * (6.4.8.1). */
    if (f->type != FT_SREJ)
        ack_to(d, f->nr);
    progress = d->va != old_va;

    if (!f->cmd && f->pf && f->type != FT_I) {
        /* The answer to our poll. */
        if (d->tmr_rec) {
            d->tmr_rec = false;
            d->retrans = 0;
            if (nr_valid(d, f->nr))
                d->vs = f->nr;
        } else if (f->type == FT_RR || f->type == FT_RNR || f->type == FT_REJ) {
            su_violation(mf, d->dlci, "unsolicited F=1 supervisory response");
        }
        d->poll_out = false;
        progress = true;
    }

    switch (f->type) {
    case FT_RR:
        d->peer_busy = false;
        break;
    case FT_RNR:
        d->peer_busy = true;
        break;
    case FT_REJ:
        mf->st.rej_received++;
        d->peer_busy = false;
        /* 8.1.4 a)/b): go back to N(R).  In timer recovery only the F=1
         * response may (case c just takes the acknowledgement). */
        if (!rec || (!f->cmd && f->pf))
            d->vs = f->nr;
        progress = true;
        break;
    case FT_SREJ:
        mf->st.srej_received++;
        if (d->p.recovery == V76_REC_SREJ &&
            seq_dist(d->va, f->nr) < seq_dist(d->va, d->vs_hi))
            d->srej_retx[f->nr] = true;
        progress = true;
        break;
    default:
        break;
    }

    if (f->cmd && f->pf && f->type != FT_I)
        send_sup(mf, d, false, true, false);

    t401_update(mf, d, progress);
}

static void erm_deliver(v76_t *mf, dlc_t *d, const uint8_t *data, int len)
{
    if (mf->su.data_ind)
        mf->su.data_ind(mf->su.ctx, d->dlci, data, len);
    (void)d;
}

static void erm_rx_i(v76_t *mf, dlc_t *d, const rxframe_t *f)
{
    int k_rx = d->k_rx > 0 ? d->k_rx : d->p.k;

    mf->st.rx_i_frames++;
    if (!nr_valid(d, f->nr)) {
        uint8_t frmr[5] = {0};

        frmr[4] = 0x01;
        txq_push_dlc(mf, d, FT_FRMR, false, f->pf, frmr, 5);
        dlc_terminate(mf, d, V76_REL_N_R_ERROR, NULL, 0);
        return;
    }
    if (f->info_len > d->p.n401_rx) {
        uint8_t frmr[5] = {0};

        frmr[4] = 0x04;
        txq_push_dlc(mf, d, FT_FRMR, false, f->pf, frmr, 5);
        dlc_terminate(mf, d, V76_REL_FRAME_REJECT, NULL, 0);
        return;
    }
    erm_rx_nr(mf, d, f);
    if (d->st != V76_DLC_CONNECTED)
        return;

    if (d->own_busy) {
        if (f->pf)
            txq_push_dlc(mf, d, FT_RNR, false, true, NULL, 0);
        return;
    }

    if (f->ns == d->vr) {
        d->vr = (d->vr + 1) & 127;
        d->rej_exc = false;
        d->srej_sent[f->ns] = false;
        erm_deliver(mf, d, f->info, f->info_len);
        while (d->held[d->vr]) {
            sdu_t *h = d->held[d->vr];

            d->held[d->vr] = NULL;
            d->srej_sent[d->vr] = false;
            d->vr = (d->vr + 1) & 127;
            erm_deliver(mf, d, h->d, h->len);
            free(h);
        }
        if (f->pf)
            send_sup(mf, d, false, true, false);
        else
            d->ack_pending = true;
        return;
    }

    /* N(S) sequence error (8.1.9). */
    if (d->p.recovery == V76_REC_SREJ) {
        int off = seq_dist(d->vr, f->ns);
        int m;

        if (off >= k_rx || off == 0) {
            if (f->pf)
                send_sup(mf, d, false, true, false);
            return;
        }
        if (!d->held[f->ns]) {
            d->held[f->ns] = sdu_new(f->info, f->info_len);
        }
        for (m = d->vr; m != f->ns; m = (m + 1) & 127) {
            if (!d->held[m] && !d->srej_sent[m]) {
                txq_push_dlc(mf, d, FT_SREJ, false, false, NULL, 0);
                /* The SREJ's N(R) is the sequence number wanted: stash it in
                 * the request's ns slot via the ud byte. */
                mf->q_tail->frmr[0] = (uint8_t)m;
                d->srej_sent[m] = true;
                mf->st.srej_sent++;
            }
        }
        if (f->pf)
            send_sup(mf, d, false, true, false);
        return;
    }
    if (!d->rej_exc) {
        txq_push_dlc(mf, d, FT_REJ, false, f->pf, NULL, 0);
        d->rej_exc = true;
        mf->st.rej_sent++;
    } else if (f->pf) {
        send_sup(mf, d, false, true, false);
    }
}

static void erm_t401_expired(v76_t *mf, dlc_t *d)
{
    d->t401 = -1;
    mf->st.t401_expiries++;
    if (!d->tmr_rec) {
        d->tmr_rec = true;
        d->retrans = 0;
    } else {
        d->retrans++;
    }
    if (d->retrans < mf->cfg.n400)
        send_poll(mf, d);
    else
        dlc_terminate(mf, d, V76_REL_N400, NULL, 0);
}

/* ------------------------------------------------------------------------ *
 * Establishment, release, XID
 * ------------------------------------------------------------------------ */

static void dlc_connect(v76_t *mf, dlc_t *d)
{
    dlc_clear_erm(d);
    d->st = V76_DLC_CONNECTED;
    d->t_proc = -1;
    d->t403 = mf->cfg.t403_ms > 0 ? mf->cfg.t403_ms : -1;
    d->k_rx = d->p.k_rx;
}

static void send_sabme(v76_t *mf, dlc_t *d)
{
    txq_push_dlc(mf, d, FT_SABME, true, true, d->proc_ud, d->proc_ud_len);
    d->t_proc = mf->cfg.t401_ms;
}

static void send_disc(v76_t *mf, dlc_t *d)
{
    txq_push_dlc(mf, d, FT_DISC, true, true, d->proc_ud, d->proc_ud_len);
    d->t_proc = mf->cfg.t401_ms;
}

void v76_set_busy(v76_t *mf, int dlci, bool busy)
{
    dlc_t *d = dlc_find(mf, dlci);

    if (!d || d->st != V76_DLC_CONNECTED || d->own_busy == busy)
        return;
    d->own_busy = busy;
    if (busy) {
        txq_push_dlc(mf, d, FT_RNR, false, false, NULL, 0);
    } else {
        txq_push_dlc(mf, d, d->rej_exc ? FT_REJ : FT_RR, false, false, NULL, 0);
    }
}

int v76_establish_req(v76_t *mf, const v76_dlc_params_t *p, const uint8_t *ud, int len)
{
    int a = p && p->addr_octets ? p->addr_octets : mf->cfg.addr_octets;
    int dlci = dlci_alloc(mf, a);
    dlc_t *d;

    if (dlci < 0 || len < 0 || len > V76_MAX_N401)
        return -1;
    d = dlc_alloc(mf, dlci);
    if (!d)
        return -1;
    if (p)
        d->p = *p;
    else
        v76_dlc_params_default(&d->p);
    d->addr_octets = a;
    d->fcs_len = d->p.fcs_len ? d->p.fcs_len : V76_FCS_16;
    d->opener = true;
    d->st = V76_DLC_AWAIT_EST;
    d->n_proc = 0;
    if (len > 0) {
        d->proc_ud = malloc((size_t)len);
        if (!d->proc_ud) {
            d->used = false;
            return -1;
        }
        memcpy(d->proc_ud, ud, (size_t)len);
        d->proc_ud_len = len;
    }
    send_sabme(mf, d);
    return dlci;
}

int v76_establish_rsp(v76_t *mf, int dlci, const v76_dlc_params_t *p,
                      const uint8_t *ud, int len)
{
    dlc_t *d = dlc_find(mf, dlci);

    if (!d || d->st != V76_DLC_AWAIT_SU)
        return -1;
    if (p)
        d->p = *p;
    else
        v76_dlc_params_default(&d->p);
    d->p.fcs_len = d->fcs_len;
    d->p.addr_octets = d->addr_octets;
    dlc_connect(mf, d);
    txq_push_dlc(mf, d, FT_UA, false, d->rx_pf, ud, len);
    return 0;
}

int v76_establish_reject(v76_t *mf, int dlci, const uint8_t *ud, int len)
{
    dlc_t *d = dlc_find(mf, dlci);

    if (!d || d->st != V76_DLC_AWAIT_SU)
        return -1;
    txq_push_dlc(mf, d, FT_DM, false, d->rx_pf, ud, len);
    dlc_release_slot(d);
    return 0;
}

int v76_release_req(v76_t *mf, int dlci, const uint8_t *ud, int len)
{
    dlc_t *d = dlc_find(mf, dlci);

    if (!d || d->st == V76_DLC_DISCONNECTED || d->st == V76_DLC_AWAIT_SU)
        return -1;
    dlc_clear_erm(d);
    sdu_list_free(d->ui_head);
    d->ui_head = d->ui_tail = NULL;
    d->ui_len = 0;
    txq_drop_dlc_sup(mf, dlci);
    free(d->proc_ud);
    d->proc_ud = NULL;
    d->proc_ud_len = 0;
    if (len > 0 && ud) {
        d->proc_ud = malloc((size_t)len);
        if (d->proc_ud) {
            memcpy(d->proc_ud, ud, (size_t)len);
            d->proc_ud_len = len;
        }
    }
    d->st = V76_DLC_AWAIT_REL;
    d->n_proc = 0;
    d->t403 = -1;
    send_disc(mf, d);
    return 0;
}

bool v76_data_req(v76_t *mf, int dlci, const uint8_t *data, int len)
{
    dlc_t *d = dlc_find(mf, dlci);
    sdu_t *s;

    if (!d || d->st != V76_DLC_CONNECTED || d->p.mode != V76_ERM)
        return false;
    if (len < 0 || len > d->p.n401_tx || d->q_len >= 256)
        return false;
    s = sdu_new(data, len);
    if (!s)
        return false;
    if (d->q_tail)
        d->q_tail->next = s;
    else
        d->q_head = s;
    d->q_tail = s;
    d->q_len++;
    return true;
}

static bool ui_queue(v76_t *mf, int dlci, const uint8_t *data, int len,
                     bool is_resp, bool pf)
{
    dlc_t *d = dlc_find(mf, dlci);
    sdu_t *s;

    (void)mf;
    if (!d || d->st != V76_DLC_CONNECTED)
        return false;
    if (len < 0 || len > d->p.n401_tx || d->ui_len >= 256)
        return false;
    s = sdu_new(data, len);
    if (!s)
        return false;
    s->is_resp = is_resp;
    s->pf = pf;
    if (d->ui_tail)
        d->ui_tail->next = s;
    else
        d->ui_head = s;
    d->ui_tail = s;
    d->ui_len++;
    return true;
}

bool v76_unitdata_req(v76_t *mf, int dlci, const uint8_t *data, int len)
{
    return ui_queue(mf, dlci, data, len, false, false);
}

bool v76_unitdata_rsp(v76_t *mf, int dlci, const uint8_t *data, int len, bool pf)
{
    return ui_queue(mf, dlci, data, len, true, pf);
}

int v76_setparm_req(v76_t *mf, int dlci, const uint8_t *ud, int len)
{
    dlc_t *d = dlc_find(mf, dlci);

    if (!d || d->st != V76_DLC_CONNECTED || d->xid_pending || len < 0 ||
        len > d->p.n401_tx)
        return -1;
    free(d->xid_ud);
    d->xid_ud = NULL;
    d->xid_len = 0;
    if (len > 0) {
        d->xid_ud = malloc((size_t)len);
        if (!d->xid_ud)
            return -1;
        memcpy(d->xid_ud, ud, (size_t)len);
        d->xid_len = len;
    }
    d->xid_pending = true;
    d->n_xid = 0;
    d->t_xid = mf->cfg.t401_ms;
    txq_push_dlc(mf, d, FT_XID, true, false, ud, len);
    return 0;
}

int v76_setparm_rsp(v76_t *mf, int dlci, const uint8_t *ud, int len)
{
    dlc_t *d = dlc_find(mf, dlci);

    if (!d || d->st != V76_DLC_CONNECTED)
        return -1;
    txq_push_dlc(mf, d, FT_XID, false, false, ud, len);
    return 0;
}

int v76_test_req(v76_t *mf, int dlci, const uint8_t *data, int len)
{
    dlc_t *d = dlc_find(mf, dlci);

    if (!d || d->st != V76_DLC_CONNECTED)
        return -1;
    txq_push_dlc(mf, d, FT_TEST, true, false, data, len);
    return 0;
}

/* ------------------------------------------------------------------------ *
 * Receive dispatch
 * ------------------------------------------------------------------------ */

static void rx_sabme(v76_t *mf, const rxframe_t *f)
{
    dlc_t *d = dlc_find(mf, f->dlci);

    if (d) {
        if (d->st == V76_DLC_AWAIT_EST) {
            if (mf->cfg.initiator) {
                /* 6.1.1: the responder backs off; our attempt stands. */
                return;
            }
            /* We are the responder and collided: give way, then act as the
             * acceptor for the initiator's SABME. */
            dlc_terminate(mf, d, V76_REL_BACKOFF, NULL, 0);
            d = NULL;
        } else if (d->st == V76_DLC_AWAIT_SU) {
            return;                         /* repeat of an unanswered SABME */
        } else if (d->st == V76_DLC_CONNECTED) {
            /* The peer re-establishes, or our UA was lost: answer and reset
             * the link without bothering the SU again (6.4.3). */
            dlc_clear_erm(d);
            d->t403 = mf->cfg.t403_ms > 0 ? mf->cfg.t403_ms : -1;
            txq_push_dlc(mf, d, FT_UA, false, f->pf, NULL, 0);
            return;
        } else if (d->st == V76_DLC_AWAIT_REL) {
            /* 7.5.2: different mode-setting commands -> DM. */
            txq_push_dlc(mf, d, FT_DM, false, f->pf, NULL, 0);
            return;
        }
    }
    d = dlc_alloc(mf, f->dlci);
    if (!d) {
        txq_push(mf, FT_DM, f->dlci, false, f->pf, f->addr_octets, f->fcs_len, NULL, 0);
        return;
    }
    d->st = V76_DLC_AWAIT_SU;
    d->opener = false;
    d->rx_pf = f->pf;
    d->fcs_len = f->fcs_len;
    d->addr_octets = f->addr_octets;
    d->p.fcs_len = f->fcs_len;
    d->p.addr_octets = f->addr_octets;
    if (mf->su.establish_ind)
        mf->su.establish_ind(mf->su.ctx, f->dlci, f->info, f->info_len);
    else
        v76_establish_reject(mf, f->dlci, NULL, 0);
}

static void rx_u_connected(v76_t *mf, dlc_t *d, const rxframe_t *f)
{
    switch (f->type) {
    case FT_DISC:
        txq_push_dlc(mf, d, FT_UA, false, f->pf, NULL, 0);
        dlc_terminate(mf, d, V76_REL_DISC_RECEIVED, f->info, f->info_len);
        return;
    case FT_DM:
        if (f->pf && d->tmr_rec) {
            dlc_terminate(mf, d, V76_REL_DM_RECEIVED, f->info, f->info_len);
        } else if (!f->pf) {
            /* Unsolicited DM, F=0: terminate (Table 5). */
            dlc_terminate(mf, d, V76_REL_DM_RECEIVED, f->info, f->info_len);
        }
        return;
    case FT_FRMR:
        dlc_terminate(mf, d, V76_REL_FRMR_RECEIVED, NULL, 0);
        return;
    case FT_UA:
        su_violation(mf, d->dlci, "unsolicited UA");
        return;
    default:
        return;
    }
}

static void rx_ui(v76_t *mf, dlc_t *d, const rxframe_t *f)
{
    if (f->info_len > d->p.n401_rx)
        return;
    mf->st.rx_ui_frames++;
    if (mf->su.unitdata_ind)
        mf->su.unitdata_ind(mf->su.ctx, d->dlci, f->info, f->info_len,
                            !f->cmd, f->pf, f->hdr_only);
}

static void rx_dispatch(v76_t *mf, const rxframe_t *f)
{
    dlc_t *d = dlc_find(mf, f->dlci);

    mf->st.rx_frames++;

    if (f->type == FT_SABME) {
        if (f->cmd)
            rx_sabme(mf, f);
        return;
    }
    if (f->type == FT_XID) {
        if (f->cmd) {
            if (mf->su.setparm_ind)
                mf->su.setparm_ind(mf->su.ctx, f->dlci, f->info, f->info_len);
        } else if (d && d->xid_pending) {
            d->xid_pending = false;
            d->t_xid = -1;
            if (mf->su.setparm_conf)
                mf->su.setparm_conf(mf->su.ctx, f->dlci, f->info, f->info_len);
        }
        return;
    }
    if (f->type == FT_TEST) {
        if (f->cmd && mf->su.test_ind)
            mf->su.test_ind(mf->su.ctx, f->dlci, f->info, f->info_len);
        return;
    }

    if (!d) {
        mf->st.rx_unknown_dlci++;
        /* 7.4: a DISC to a disconnected DLC gets a DM; all else is dropped. */
        if (f->type == FT_DISC && f->cmd)
            txq_push(mf, FT_DM, f->dlci, false, f->pf, f->addr_octets, f->fcs_len, NULL, 0);
        return;
    }

    switch (d->st) {
    case V76_DLC_AWAIT_SU:
        return;                                 /* nothing until the SU answers */

    case V76_DLC_AWAIT_EST:
        if (f->type == FT_UA && !f->cmd && f->pf) {
            d->n_proc = 0;
            free(d->proc_ud);
            d->proc_ud = NULL;
            d->proc_ud_len = 0;
            dlc_connect(mf, d);
            if (mf->su.establish_conf)
                mf->su.establish_conf(mf->su.ctx, d->dlci, f->info, f->info_len);
            return;
        }
        if (f->type == FT_DM && !f->cmd) {
            if (f->pf) {
                free(d->proc_ud);
                d->proc_ud = NULL;
                dlc_terminate(mf, d, V76_REL_DM_RECEIVED, f->info, f->info_len);
            }
            return;
        }
        if (f->type == FT_DISC && f->cmd) {
            /* 7.5.2 different commands */
            txq_push_dlc(mf, d, FT_DM, false, f->pf, NULL, 0);
            return;
        }
        if (f->type == FT_I || f->type == FT_RR || f->type == FT_RNR ||
            f->type == FT_REJ || f->type == FT_SREJ) {
            /* 7.1.2.1: the UA was lost.  Proceed as if it had arrived. */
            free(d->proc_ud);
            d->proc_ud = NULL;
            d->proc_ud_len = 0;
            dlc_connect(mf, d);
            if (mf->su.establish_conf)
                mf->su.establish_conf(mf->su.ctx, d->dlci, NULL, 0);
            if (d->used)
                rx_dispatch(mf, f);
            return;
        }
        return;

    case V76_DLC_AWAIT_REL:
        if (f->type == FT_UA && !f->cmd && f->pf) {
            free(d->proc_ud);
            d->proc_ud = NULL;
            txq_drop_dlc_sup(mf, d->dlci);
            if (mf->su.release_ind)
                mf->su.release_ind(mf->su.ctx, d->dlci, f->info, f->info_len,
                                   V76_REL_LOCAL_COMPLETE);
            dlc_release_slot(d);
            return;
        }
        if (f->type == FT_DM && !f->cmd && f->pf) {
            free(d->proc_ud);
            d->proc_ud = NULL;
            if (mf->su.release_ind)
                mf->su.release_ind(mf->su.ctx, d->dlci, f->info, f->info_len,
                                   V76_REL_LOCAL_COMPLETE);
            dlc_release_slot(d);
            return;
        }
        if (f->type == FT_DISC && f->cmd) {
            /* 7.5.1: identical commands -> UA, then wait for ours. */
            txq_push_dlc(mf, d, FT_UA, false, f->pf, NULL, 0);
            return;
        }
        if (f->type == FT_SABME && f->cmd) {
            txq_push_dlc(mf, d, FT_DM, false, f->pf, NULL, 0);
            return;
        }
        return;

    case V76_DLC_CONNECTED:
        break;

    case V76_DLC_DISCONNECTED:
    default:
        return;
    }

    /* Connected. */
    switch (f->type) {
    case FT_I:
        if (d->p.mode == V76_ERM && f->cmd)
            erm_rx_i(mf, d, f);
        return;
    case FT_RR: case FT_RNR: case FT_REJ: case FT_SREJ:
        if (d->p.mode == V76_ERM) {
            if (f->type == FT_SREJ && d->p.recovery != V76_REC_SREJ) {
                /* 8.1.5.1: SREJ not agreed -> unrecognised control field. */
                uint8_t frmr[5] = {0};

                frmr[4] = 0x01;
                txq_push_dlc(mf, d, FT_FRMR, false, f->pf, frmr, 5);
                dlc_terminate(mf, d, V76_REL_FRAME_REJECT, NULL, 0);
                return;
            }
            if (!nr_valid(d, f->nr)) {
                uint8_t frmr[5] = {0};

                frmr[4] = 0x01;
                txq_push_dlc(mf, d, FT_FRMR, false, f->pf, frmr, 5);
                dlc_terminate(mf, d, V76_REL_N_R_ERROR, NULL, 0);
                return;
            }
            erm_rx_nr(mf, d, f);
        }
        return;
    case FT_UI:
    case FT_UIH:
        if ((f->type == FT_UIH) != d->p.uih)
            return;
        rx_ui(mf, d, f);
        return;
    default:
        rx_u_connected(mf, d, f);
        return;
    }
}

/* ------------------------------------------------------------------------ *
 * Receive framer
 * ------------------------------------------------------------------------ */

static int rbuf_cap_bits(void)
{
    return (int)(sizeof(((v76_t *)0)->rbuf) * 8);
}

static void rx_process_bytes(v76_t *mf, const uint8_t *b, int nbits)
{
    static const int try_order[3] = { V76_FCS_16, V76_FCS_8, V76_FCS_32 };
    rxframe_t f;
    int n, i;

    if (nbits == 0)
        return;
    if (nbits & 7) {
        mf->st.rx_invalid++;
        return;
    }
    n = nbits / 8;
    for (i = 0; i < 3; i++) {
        unsigned bit = try_order[i] == V76_FCS_8 ? V76_FCS_MASK_8 :
                       try_order[i] == V76_FCS_16 ? V76_FCS_MASK_16 : V76_FCS_MASK_32;

        if (!(mf->cfg.fcs_support & bit))
            continue;
        if (parse_with_fcs(mf, b, n, try_order[i], &f)) {
            /* The U-frame minimum is one control octet, S/I two (5.3 b). */
            if (f.type == FT_BAD) {
                mf->st.rx_invalid++;
                return;
            }
            if ((f.type == FT_I || f.type == FT_RR || f.type == FT_RNR ||
                 f.type == FT_REJ || f.type == FT_SREJ) &&
                n < f.addr_octets + 2 + f.fcs_len) {
                mf->st.rx_invalid++;
                return;
            }
            if ((f.type == FT_RR || f.type == FT_RNR || f.type == FT_REJ ||
                 f.type == FT_SREJ) && f.info_len != 0) {
                /* No information field on an S frame (the m-SREJ span list
                 * is not implemented). */
                mf->st.rx_invalid++;
                return;
            }
            rx_dispatch(mf, &f);
            return;
        }
    }
    mf->st.rx_fcs_errors++;
    mf->st.rx_invalid++;
    if (mf->su.fcs_error) {
        int dlci = -1;

        if (n >= 1)
            dlci = (b[0] & 1) ? (b[0] >> 2) & 0x3F :
                   (n >= 2 ? (((b[0] >> 2) & 0x3F) << 7) | (b[1] >> 1) : -1);
        mf->su.fcs_error(mf->su.ctx, dlci);
    }
}

/* A frame sent in suspend state: no control field, treated as UI/UIH. */
static void rx_process_rt(v76_t *mf, const uint8_t *b, int nbits)
{
    int n, a = 0, dlci = -1, i, fcs_len;
    dlc_t *d = NULL;
    bool ok = false;

    if (nbits & 7 || nbits < 16) {
        mf->st.rx_invalid++;
        mf->nrt_valid = false;
        return;
    }
    n = nbits / 8;
    if (mf->cfg.sr_with_address) {
        a = (b[0] & 1) ? 1 : 2;
        if (a == 2 && !(b[1] & 1)) {
            mf->st.rx_invalid++;
            mf->nrt_valid = false;
            return;
        }
        dlci = a == 2 ? (((b[0] >> 2) & 0x3F) << 7) | (b[1] >> 1) : (b[0] >> 2) & 0x3F;
        d = dlc_find(mf, dlci);
    } else {
        for (i = 0; i < V76_MAX_DLC; i++)
            if (mf->dlc[i].used && mf->dlc[i].st == V76_DLC_CONNECTED &&
                mf->dlc[i].p.realtime) {
                d = &mf->dlc[i];
                dlci = d->dlci;
                break;
            }
    }
    if (!d || !d->p.realtime || d->st != V76_DLC_CONNECTED) {
        mf->st.rx_invalid++;
        mf->nrt_valid = false;
        return;
    }
    fcs_len = d->fcs_len;
    if (n < a + 1 + fcs_len || n - a - fcs_len > d->p.n401_rx) {
        mf->st.rx_invalid++;
        mf->nrt_valid = false;
        return;
    }
    {
        int body = n - fcs_len, cover = body;

        if (d->p.uih && d->p.uih_protect > 0 && d->p.uih_protect < body)
            cover = d->p.uih_protect;
        ok = v76_fcs(fcs_len, b, cover) == fcs_get(b + body, fcs_len);
        if (ok) {
            mf->st.rx_frames++;
            mf->st.rx_ui_frames++;
            if (mf->su.unitdata_ind)
                mf->su.unitdata_ind(mf->su.ctx, dlci, b + a, body - a, false, false,
                                    cover < body);
        }
    }
    if (!ok) {
        mf->st.rx_fcs_errors++;
        mf->st.rx_invalid++;
        mf->nrt_valid = false;
        if (mf->su.fcs_error)
            mf->su.fcs_error(mf->su.ctx, dlci);
    }
}

static void rbuf_reset(v76_t *mf)
{
    mf->rbits = 0;
    mf->rt_total_bits = 0;
    mf->rt_tail = 0;
    mf->rt_tail_flaglike = false;
}

static void rbuf_push(v76_t *mf, int bit)
{
    if (mf->rbits >= rbuf_cap_bits()) {
        /* 8.4.2 NOTE: an unbounded frame is discarded. */
        rbuf_reset(mf);
        mf->rx_hunting = true;
        mf->st.rx_invalid++;
        return;
    }
    if ((mf->rbits & 7) == 0)
        mf->rbuf[mf->rbits >> 3] = 0;
    if (bit)
        mf->rbuf[mf->rbits >> 3] |= (uint8_t)(1u << (mf->rbits & 7));
    mf->rbits++;
}

/* In suspend state work out how long a real-time frame is allowed to be, as
 * soon as the address (if any) says which DLC it belongs to. */
static void rt_learn_length(v76_t *mf)
{
    dlc_t *d = NULL;
    int a = 0, i, dlci;

    if (mf->rt_total_bits || mf->rx_sr != SR_SUSPEND)
        return;
    if (mf->cfg.sr_with_address) {
        if (mf->rbits < 8)
            return;
        a = (mf->rbuf[0] & 1) ? 1 : 2;
        if (mf->rbits < 8 * a)
            return;
        dlci = a == 2 ? (((mf->rbuf[0] >> 2) & 0x3F) << 7) | (mf->rbuf[1] >> 1)
                      : (mf->rbuf[0] >> 2) & 0x3F;
        d = dlc_find(mf, dlci);
    } else {
        for (i = 0; i < V76_MAX_DLC; i++)
            if (mf->dlc[i].used && mf->dlc[i].st == V76_DLC_CONNECTED &&
                mf->dlc[i].p.realtime) {
                d = &mf->dlc[i];
                break;
            }
    }
    if (!d || !d->p.realtime)
        return;
    mf->rt_total_bits = 8 * (a + d->p.n401_rx + d->fcs_len);
}

static void nrt_save(v76_t *mf)
{
    memcpy(mf->nrt_buf, mf->rbuf, (size_t)((mf->rbits + 7) / 8));
    mf->nrt_bits = mf->rbits;
    mf->nrt_valid = (mf->rbits % 8) == 0;
}

static void nrt_restore(v76_t *mf, bool valid_ok)
{
    if (valid_ok && mf->nrt_valid) {
        memcpy(mf->rbuf, mf->nrt_buf, (size_t)((mf->nrt_bits + 7) / 8));
        mf->rbits = mf->nrt_bits;
    } else {
        mf->rbits = 0;
        mf->rx_hunting = false;
        /* An NRT frame with an S/R error in it is invalid; keep reading to
         * its closing flag so that frame boundary is not lost. */
        mf->nrt_valid = false;
    }
    mf->rt_total_bits = 0;
    mf->rt_tail = 0;
    mf->rt_tail_flaglike = false;
    mf->nrt_bits = 0;
}

static void rx_event_flag(v76_t *mf, int nones)
{
    /* Strip the flag's own bits (leading 0 and the ones) from the buffer. */
    int strip = nones + (mf->rx_run_lead0 ? 1 : 0);

    if (mf->rbits >= strip)
        mf->rbits -= strip;
    else
        mf->rbits = 0;

    if (nones == 6) {
        switch (mf->rx_sr) {
        case SR_NORMAL:
            if (mf->nrt_valid)
                rx_process_bytes(mf, mf->rbuf, mf->rbits);
            else if (mf->rbits)
                mf->st.rx_invalid++;    /* A.4.1: an S/R error inside it */
            break;
        case SR_SUSPEND:
            /* A.4.4 d): a normal flag in suspend state. */
            mf->st.sr_violations++;
            mf->rx_sr = SR_ABORT;
            break;
        case SR_ABORT:
            mf->rx_sr = SR_NORMAL;
            break;
        }
        rbuf_reset(mf);
        mf->rx_hunting = false;
        if (mf->rx_sr == SR_NORMAL)
            mf->nrt_valid = true;
        return;
    }

    if (nones == 7) {                               /* suspend flag */
        switch (mf->rx_sr) {
        case SR_NORMAL:
            if (mf->rbits == 0) {
                mf->st.sr_violations++;             /* A.4.2 e) */
                mf->rx_sr = SR_ABORT;
            } else {
                nrt_save(mf);
                mf->rx_sr = SR_SUSPEND;
                mf->st.sr_suspends++;
            }
            break;
        case SR_SUSPEND:
            if (mf->rbits == 0) {
                mf->st.sr_violations++;             /* A.4.2 a) */
                mf->rx_sr = SR_ABORT;
            } else {
                rx_process_rt(mf, mf->rbuf, mf->rbits);
            }
            break;
        case SR_ABORT:
            mf->rx_sr = SR_SUSPEND;
            mf->nrt_valid = false;
            mf->nrt_bits = 0;
            break;
        }
        rbuf_reset(mf);
        return;
    }

    /* nones == 8: resume flag */
    switch (mf->rx_sr) {
    case SR_SUSPEND:
        if (mf->rbits == 0) {
            mf->st.sr_violations++;                 /* A.4.2 b) */
            mf->rx_sr = SR_ABORT;
            rbuf_reset(mf);
            return;
        }
        rx_process_rt(mf, mf->rbuf, mf->rbits);
        mf->rx_sr = SR_NORMAL;
        mf->st.sr_resumes++;
        rbuf_reset(mf);
        nrt_restore(mf, true);
        break;
    case SR_NORMAL:
        mf->st.sr_violations++;                     /* A.4.2 d) */
        mf->rx_sr = SR_ABORT;
        rbuf_reset(mf);
        break;
    case SR_ABORT:
        rbuf_reset(mf);
        break;
    }
}

/* Called after every pushed bit in suspend state: has a max-length RT frame
 * run its course (Annex A.3 d), third indent)? */
static void rt_check_full(v76_t *mf)
{
    rt_learn_length(mf);
    if (!mf->rt_total_bits || mf->rbits <= mf->rt_total_bits)
        return;

    mf->rt_tail = mf->rbits - mf->rt_total_bits;
    if (mf->rt_tail_flaglike)
        return;
    if (mf->rt_tail == 8) {
        uint8_t t = 0;
        int i;

        for (i = 0; i < 8; i++) {
            int bi = mf->rt_total_bits + i;

            if (mf->rbuf[bi >> 3] & (1u << (bi & 7)))
                t |= (uint8_t)(1u << i);
        }
        if (t == 0xFE) {                /* 0 then seven ones: a flag begins */
            mf->rt_tail_flaglike = true;
            return;
        }
    } else if (mf->rt_tail < 8) {
        /* A prospective flag starts with 0 then ones: if the tail already
         * deviates there is no need to wait the full octet. */
        int i;
        bool could = true;

        for (i = 0; i < mf->rt_tail; i++) {
            int bi = mf->rt_total_bits + i;
            int bit = (mf->rbuf[bi >> 3] >> (bi & 7)) & 1;

            if (bit != (i == 0 ? 0 : 1)) {
                could = false;
                break;
            }
        }
        if (could)
            return;
    } else {
        return;
    }

    /* The RT frame was of maximum length and no flag followed: it is
     * complete, and the suspended NRT frame resumes with the tail bits. */
    {
        int tail_bits = mf->rt_tail;
        uint8_t tail[2] = {0, 0};
        int i;

        for (i = 0; i < tail_bits; i++) {
            int bi = mf->rt_total_bits + i;

            if (mf->rbuf[bi >> 3] & (1u << (bi & 7)))
                tail[i >> 3] |= (uint8_t)(1u << (i & 7));
        }
        rx_process_rt(mf, mf->rbuf, mf->rt_total_bits);
        mf->rx_sr = SR_NORMAL;
        mf->st.sr_resumes++;
        rbuf_reset(mf);
        nrt_restore(mf, true);
        for (i = 0; i < tail_bits; i++)
            rbuf_push(mf, (tail[i >> 3] >> (i & 7)) & 1);
    }
}

void v76_rx_put_bit(v76_t *mf, int bit)
{
    bool sr = mf->cfg.suspend_resume;
    int abort_at = sr ? 9 : 7;

    mf->st.rx_bits++;
    bit &= 1;

    if (bit) {
        if (mf->rx_ones == 0)
            mf->rx_run_lead0 = mf->rx_last_pushed_zero;
        mf->rx_ones++;
        if (mf->rx_ones >= abort_at) {
            if (mf->rx_ones == abort_at) {
                mf->st.rx_aborts++;
                rbuf_reset(mf);
                if (sr)
                    mf->rx_sr = SR_ABORT;
                mf->rx_hunting = true;
            }
            return;
        }
        if (!mf->rx_hunting) {
            rbuf_push(mf, 1);
            mf->rx_last_pushed_zero = false;
            if (mf->rx_sr == SR_SUSPEND)
                rt_check_full(mf);
        }
        return;
    }

    /* A zero. */
    {
        int ones = mf->rx_ones;

        mf->rx_ones = 0;
        if (ones == 5) {
            mf->rx_last_pushed_zero = false;    /* stuffing: not a data bit */
            return;
        }
        if (ones == 6 || (sr && (ones == 7 || ones == 8))) {
            if (mf->rx_hunting) {
                /* Resynchronise on the first flag after an abort. */
                mf->rx_hunting = false;
                rbuf_reset(mf);
                if (ones == 6 && mf->rx_sr == SR_ABORT)
                    mf->rx_sr = SR_NORMAL;
                mf->nrt_valid = true;
                mf->rx_last_pushed_zero = false;
                return;
            }
            rx_event_flag(mf, ones);
            mf->rx_last_pushed_zero = false;
            return;
        }
        if (ones >= 7) {
            /* Basic mode: seven or more ones is an abort, already handled. */
            mf->rx_last_pushed_zero = false;
            return;
        }
        if (!mf->rx_hunting) {
            rbuf_push(mf, 0);
            mf->rx_last_pushed_zero = true;
            if (mf->rx_sr == SR_SUSPEND)
                rt_check_full(mf);
        } else {
            mf->rx_last_pushed_zero = false;
        }
    }
}

void v76_rx_push_bytes(v76_t *mf, const uint8_t *in, int len)
{
    int i, b;

    for (i = 0; i < len; i++)
        for (b = 0; b < 8; b++)
            v76_rx_put_bit(mf, (in[i] >> b) & 1);
}

/* ------------------------------------------------------------------------ *
 * Transmit framer
 * ------------------------------------------------------------------------ */

static void bq_push(v76_t *mf, uint32_t bits, int n)
{
    mf->bq |= (uint64_t)bits << mf->bq_n;
    mf->bq_n += n;
}

static void tx_put_byte(v76_t *mf, uint8_t byte)
{
    int i;

    for (i = 0; i < 8; i++) {
        int b = (byte >> i) & 1;

        bq_push(mf, (uint32_t)b, 1);
        if (b) {
            if (++mf->tx_ones == 5) {
                bq_push(mf, 0, 1);              /* 5.1.2 zero insertion */
                mf->tx_ones = 0;
            }
        } else {
            mf->tx_ones = 0;
        }
    }
}

static void tx_flag(v76_t *mf)
{
    bq_push(mf, 0x7E, 8);                       /* 0 1111 11 0, bit 1 first */
    mf->tx_ones = 0;
}

static void tx_suspend_flag(v76_t *mf)
{
    bq_push(mf, 0xFE, 9);                       /* 0 1^7 0 */
    mf->tx_ones = 0;
}

static void tx_resume_flag(v76_t *mf)
{
    bq_push(mf, 0x1FE, 10);                     /* 0 1^8 0 */
    mf->tx_ones = 0;
}

/* Build a control/response frame from a queued request. */
static int build_req(v76_t *mf, const txreq_t *r, uint8_t *out)
{
    uint8_t ctl[2];
    int cl = 1;
    bool cr = r->cmd ? mf->cfg.initiator : !mf->cfg.initiator;
    dlc_t *d = dlc_find(mf, r->dlci);
    uint8_t info[V76_MAX_N401 + 8];
    int il = 0;

    if (r->ud && r->ud_len > 0) {
        il = r->ud_len;
        if (il > V76_MAX_N401)
            il = V76_MAX_N401;
        memcpy(info, r->ud, (size_t)il);
    }
    switch (r->kind) {
    case FT_SABME: ctl[0] = (uint8_t)(U_SABME | (r->pf ? U_PF : 0)); break;
    case FT_DM:    ctl[0] = (uint8_t)(U_DM | (r->pf ? U_PF : 0)); break;
    case FT_DISC:  ctl[0] = (uint8_t)(U_DISC | (r->pf ? U_PF : 0)); break;
    case FT_UA:    ctl[0] = (uint8_t)(U_UA | (r->pf ? U_PF : 0)); break;
    case FT_FRMR:
        ctl[0] = (uint8_t)(U_FRMR | (r->pf ? U_PF : 0));
        memcpy(info, r->frmr, 5);
        il = 5;
        break;
    case FT_XID:   ctl[0] = (uint8_t)(U_XID); break;
    case FT_TEST:  ctl[0] = (uint8_t)(U_TEST); break;
    case FT_RR: case FT_RNR: case FT_REJ: case FT_SREJ: {
        int nr = d ? d->vr : 0;

        cl = 2;
        ctl[0] = r->kind == FT_RR ? S_RR : r->kind == FT_RNR ? S_RNR :
                 r->kind == FT_REJ ? S_REJ : S_SREJ;
        if (r->kind == FT_SREJ)
            nr = r->frmr[0];
        ctl[1] = (uint8_t)((nr << 1) | (r->pf ? 1 : 0));
        break;
    }
    default:
        return 0;
    }
    return v76_build_frame(out, r->addr_octets, r->dlci, cr, ctl, cl, info, il,
                           r->fcs_len, 0);
}

static void load_frame(txframe_t *t, const uint8_t *b, int n)
{
    memcpy(t->buf, b, (size_t)n);
    t->len = n;
    t->pos = 0;
    t->active = true;
    t->rt_nocontrol = false;
    t->is_rt = false;
    t->max_rt = false;
}

static bool erm_build_i(v76_t *mf, dlc_t *d, txframe_t *t)
{
    uint8_t ctl[2];
    int ns;
    sdu_t *s;
    bool retx = false;

    if (d->st != V76_DLC_CONNECTED || d->p.mode != V76_ERM)
        return false;

    /* Selective retransmissions first (8.1.5.1): they do not move V(S). */
    if (d->p.recovery == V76_REC_SREJ) {
        int i, n;

        for (i = 0, n = d->va; n != d->vs_hi && i < 128; i++, n = (n + 1) & 127) {
            if (d->srej_retx[n] && d->slot[n]) {
                d->srej_retx[n] = false;
                ns = n;
                s = d->slot[n];
                goto build;
            }
        }
    }
    if (d->peer_busy || d->tmr_rec)
        return false;
    if (d->vs != d->vs_hi) {
        ns = d->vs;
        s = d->slot[ns];
        if (!s) {                       /* nothing stored: resynchronise */
            d->vs = d->vs_hi;
            return false;
        }
        d->vs = (d->vs + 1) & 127;
        retx = true;
    } else {
        if (!d->q_head || seq_dist(d->va, d->vs_hi) >= d->p.k)
            return false;
        s = d->q_head;
        d->q_head = s->next;
        if (!d->q_head)
            d->q_tail = NULL;
        d->q_len--;
        s->next = NULL;
        ns = d->vs_hi;
        d->slot[ns] = s;
        d->vs_hi = (d->vs_hi + 1) & 127;
        d->vs = d->vs_hi;
    }
build:
    ctl[0] = (uint8_t)(ns << 1);
    ctl[1] = (uint8_t)(d->vr << 1);
    {
        uint8_t fr[V76_FRAME_MAX + 16];
        int n = v76_build_frame(fr, d->addr_octets, d->dlci,
                                mf->cfg.initiator, ctl, 2, s->d, s->len, d->fcs_len, 0);

        load_frame(t, fr, n);
    }
    mf->st.tx_i_frames++;
    if (retx)
        mf->st.tx_retransmitted_i++;
    d->ack_pending = false;
    if (d->t401 < 0)
        d->t401 = mf->cfg.t401_ms;
    d->t403 = -1;
    return true;
}

static bool ui_build(v76_t *mf, dlc_t *d, txframe_t *t)
{
    uint8_t ctl[1];
    uint8_t fr[V76_FRAME_MAX + 16];
    sdu_t *s;
    bool cr;
    int n;

    if (d->st != V76_DLC_CONNECTED || !d->ui_head)
        return false;
    s = d->ui_head;
    d->ui_head = s->next;
    if (!d->ui_head)
        d->ui_tail = NULL;
    d->ui_len--;
    cr = s->is_resp ? !mf->cfg.initiator : mf->cfg.initiator;
    ctl[0] = (uint8_t)((d->p.uih ? U_UIH : U_UI) | (s->pf ? U_PF : 0));
    n = v76_build_frame(fr, d->addr_octets, d->dlci, cr, ctl, 1, s->d, s->len,
                        d->fcs_len, d->p.uih ? d->p.uih_protect : 0);
    load_frame(t, fr, n);
    t->is_rt = d->p.realtime;
    free(s);
    mf->st.tx_ui_frames++;
    return true;
}

/* Annex A: an RT frame in suspend format (no control field, A.3 note 3). */
static bool rt_build_suspend(v76_t *mf, txframe_t *t)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++) {
        dlc_t *d = &mf->dlc[(mf->rr + i) % V76_MAX_DLC];
        uint8_t fr[V76_FRAME_MAX + 16];
        sdu_t *s;
        int n = 0, cover, body;

        if (!d->used || d->st != V76_DLC_CONNECTED || !d->p.realtime || !d->ui_head)
            continue;
        s = d->ui_head;
        if (s->is_resp)
            continue;                   /* only plain UI can be an RT frame */
        d->ui_head = s->next;
        if (!d->ui_head)
            d->ui_tail = NULL;
        d->ui_len--;
        if (mf->cfg.sr_with_address)
            n = put_addr(fr, d->addr_octets, d->dlci, mf->cfg.initiator);
        memcpy(fr + n, s->d, (size_t)s->len);
        n += s->len;
        body = n;
        cover = (d->p.uih && d->p.uih_protect > 0 && d->p.uih_protect < body)
                ? d->p.uih_protect : body;
        fcs_put(fr + n, d->fcs_len, v76_fcs(d->fcs_len, fr, cover));
        n += d->fcs_len;
        load_frame(t, fr, n);
        t->rt_nocontrol = true;
        t->is_rt = true;
        t->max_rt = s->len >= d->p.n401_rt && d->p.n401_rt > 0;
        free(s);
        mf->st.tx_ui_frames++;
        mf->st.rt_frames_in_nrt++;
        return true;
    }
    return false;
}

static bool rt_ready(const v76_t *mf)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++) {
        const dlc_t *d = &mf->dlc[i];

        if (d->used && d->st == V76_DLC_CONNECTED && d->p.realtime && d->ui_head &&
            !d->ui_head->is_resp)
            return true;
    }
    return false;
}

static bool pick_next(v76_t *mf, txframe_t *t)
{
    int i;

    if (mf->q_head) {
        txreq_t *r = mf->q_head;
        uint8_t fr[V76_FRAME_MAX + 16];
        int n;

        mf->q_head = r->next;
        if (!mf->q_head)
            mf->q_tail = NULL;
        n = build_req(mf, r, fr);
        free(r->ud);
        free(r);
        if (n > 0) {
            load_frame(t, fr, n);
            mf->st.tx_frames++;
            return true;
        }
    }
    /* Real-time DLCs, then other unnumbered information, then ERM data. */
    for (i = 0; i < V76_MAX_DLC; i++) {
        dlc_t *d = &mf->dlc[(mf->rr + i) % V76_MAX_DLC];

        if (d->used && d->p.realtime && ui_build(mf, d, t)) {
            mf->rr = (mf->rr + i + 1) % V76_MAX_DLC;
            mf->st.tx_frames++;
            return true;
        }
    }
    for (i = 0; i < V76_MAX_DLC; i++) {
        dlc_t *d = &mf->dlc[(mf->rr + i) % V76_MAX_DLC];

        if (d->used && !d->p.realtime && ui_build(mf, d, t)) {
            mf->rr = (mf->rr + i + 1) % V76_MAX_DLC;
            mf->st.tx_frames++;
            return true;
        }
    }
    for (i = 0; i < V76_MAX_DLC; i++) {
        dlc_t *d = &mf->dlc[(mf->rr + i) % V76_MAX_DLC];

        if (!d->used)
            continue;
        if (erm_build_i(mf, d, t)) {
            mf->rr = (mf->rr + i + 1) % V76_MAX_DLC;
            mf->st.tx_frames++;
            return true;
        }
        if (d->ack_pending && d->st == V76_DLC_CONNECTED && d->p.mode == V76_ERM) {
            uint8_t fr[V76_FRAME_MAX + 16];
            txreq_t r;
            int n;

            memset(&r, 0, sizeof(r));
            r.kind = d->own_busy ? FT_RNR : FT_RR;
            r.dlci = d->dlci;
            r.cmd = false;
            r.addr_octets = d->addr_octets;
            r.fcs_len = d->fcs_len;
            d->ack_pending = false;
            n = build_req(mf, &r, fr);
            load_frame(t, fr, n);
            mf->st.tx_frames++;
            return true;
        }
    }
    return false;
}

static void tx_refill(v76_t *mf)
{
    txframe_t *c = &mf->cur;

    if (!c->active) {
        if (mf->tx_suspended && mf->nrt.active) {
            /* Cannot happen: a suspended frame is resumed when its RT frame
             * ends.  Defensive. */
            *c = mf->nrt;
            mf->nrt.active = false;
            mf->tx_suspended = false;
        } else if (!pick_next(mf, c)) {
            tx_flag(mf);                        /* interframe time fill, 5.5 */
            return;
        }
    }

    /* Annex A: a real-time frame is ready while a non-real-time one is
     * partly sent.  Suspend only on an octet boundary. */
    if (mf->cfg.suspend_resume && !mf->tx_suspended && !c->rt_nocontrol && !c->is_rt &&
        c->pos > 0 && c->pos < c->len && rt_ready(mf)) {
        txframe_t rt;

        memset(&rt, 0, sizeof(rt));
        if (rt_build_suspend(mf, &rt)) {
            mf->nrt = *c;
            *c = rt;
            mf->tx_suspended = true;
            mf->st.sr_suspends++;
            tx_suspend_flag(mf);
        }
    }

    if (c->pos < c->len) {
        tx_put_byte(mf, c->buf[c->pos++]);
        return;
    }

    /* The frame is complete. */
    if (mf->tx_suspended && c->rt_nocontrol) {
        txframe_t rt;

        memset(&rt, 0, sizeof(rt));
        if (rt_build_suspend(mf, &rt)) {
            tx_suspend_flag(mf);                /* another RT frame */
            mf->st.sr_suspends++;
            *c = rt;
            return;
        }
        if (mf->nrt.active) {
            if (!c->max_rt) {
                tx_resume_flag(mf);
                mf->st.sr_resumes++;
            } else {
                mf->st.sr_resumes++;            /* A.3 b) ii): no RF */
            }
            *c = mf->nrt;
            mf->nrt.active = false;
            mf->tx_suspended = false;
            return;
        }
        mf->tx_suspended = false;
    }
    c->active = false;
    tx_flag(mf);
}

void v76_advance_ms(v76_t *mf, int ms)
{
    int i;

    while (ms-- > 0) {
        for (i = 0; i < V76_MAX_DLC; i++) {
            dlc_t *d = &mf->dlc[i];

            if (!d->used)
                continue;
            if (d->t_proc >= 0 && --d->t_proc < 0) {
                if (d->st == V76_DLC_AWAIT_EST || d->st == V76_DLC_AWAIT_REL) {
                    if (d->n_proc + 1 >= mf->cfg.n400) {
                        bool was_est = d->st == V76_DLC_AWAIT_EST;
                        int dlci = d->dlci;

                        free(d->proc_ud);
                        d->proc_ud = NULL;
                        if (was_est) {
                            dlc_terminate(mf, d, V76_REL_N400, NULL, 0);
                        } else {
                            dlc_release_slot(d);
                            if (mf->su.release_ind)
                                mf->su.release_ind(mf->su.ctx, dlci, NULL, 0,
                                                   V76_REL_N400);
                        }
                        continue;
                    }
                    d->n_proc++;
                    if (d->st == V76_DLC_AWAIT_EST)
                        send_sabme(mf, d);
                    else
                        send_disc(mf, d);
                }
            }
            if (d->t_xid >= 0 && --d->t_xid < 0 && d->xid_pending) {
                if (d->n_xid + 1 >= mf->cfg.n400) {
                    d->xid_pending = false;
                    if (mf->su.setparm_fail)
                        mf->su.setparm_fail(mf->su.ctx, d->dlci);
                } else {
                    d->n_xid++;
                    d->t_xid = mf->cfg.t401_ms;
                    txq_push_dlc(mf, d, FT_XID, true, false, d->xid_ud, d->xid_len);
                }
            }
            if (d->st == V76_DLC_CONNECTED && d->p.mode == V76_ERM) {
                if (d->t401 >= 0 && --d->t401 < 0) {
                    erm_t401_expired(mf, d);
                    if (!d->used)
                        continue;
                }
                if (d->t403 >= 0 && --d->t403 < 0) {
                    d->t403 = -1;
                    erm_enter_recovery(mf, d);
                }
            }
        }
    }
}

int v76_tx_get_bit(v76_t *mf)
{
    int bit;

    if (mf->bq_n == 0)
        tx_refill(mf);
    bit = (int)(mf->bq & 1);
    mf->bq >>= 1;
    mf->bq_n--;
    mf->st.tx_bits++;
    if (mf->cfg.line_bit_rate > 0) {
        mf->clk_acc += 1000;
        if (mf->clk_acc >= mf->cfg.line_bit_rate) {
            mf->clk_acc -= mf->cfg.line_bit_rate;
            v76_advance_ms(mf, 1);
        }
    }
    return bit;
}

void v76_tx_fill_bytes(v76_t *mf, uint8_t *out, int len)
{
    int i, b;

    for (i = 0; i < len; i++) {
        uint8_t v = 0;

        for (b = 0; b < 8; b++)
            v |= (uint8_t)(v76_tx_get_bit(mf) << b);
        out[i] = v;
    }
}

/* ------------------------------------------------------------------------ *
 * Lifecycle and introspection
 * ------------------------------------------------------------------------ */

v76_t *v76_create(const v76_config_t *cfg, const v76_su_t *su)
{
    v76_t *mf = calloc(1, sizeof(*mf));

    if (!mf)
        return NULL;
    if (cfg)
        mf->cfg = *cfg;
    else
        v76_config_default(&mf->cfg, true);
    if (!mf->cfg.fcs_support)
        mf->cfg.fcs_support = V76_FCS_MASK_16;
    if (mf->cfg.t401_ms <= 0)
        mf->cfg.t401_ms = 1000;
    if (mf->cfg.n400 <= 0)
        mf->cfg.n400 = 10;
    if (mf->cfg.addr_octets < 1)
        mf->cfg.addr_octets = 1;
    if (su)
        mf->su = *su;
    mf->rx_hunting = true;      /* wait for the first flag */
    mf->nrt_valid = true;
    return mf;
}

void v76_destroy(v76_t *mf)
{
    int i;

    if (!mf)
        return;
    for (i = 0; i < V76_MAX_DLC; i++)
        if (mf->dlc[i].used)
            dlc_release_slot(&mf->dlc[i]);
    while (mf->q_head) {
        txreq_t *r = mf->q_head;

        mf->q_head = r->next;
        free(r->ud);
        free(r);
    }
    free(mf);
}

void v76_set_su(v76_t *mf, const v76_su_t *su)
{
    mf->su = *su;
}

const v76_stats_t *v76_stats(const v76_t *mf)
{
    return &mf->st;
}

v76_dlc_state_t v76_dlc_state(const v76_t *mf, int dlci)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++)
        if (mf->dlc[i].used && mf->dlc[i].dlci == dlci)
            return mf->dlc[i].st;
    return V76_DLC_DISCONNECTED;
}

bool v76_dlc_params(const v76_t *mf, int dlci, v76_dlc_params_t *out)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++)
        if (mf->dlc[i].used && mf->dlc[i].dlci == dlci) {
            *out = mf->dlc[i].p;
            return true;
        }
    return false;
}

int v76_data_backlog(const v76_t *mf, int dlci)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++)
        if (mf->dlc[i].used && mf->dlc[i].dlci == dlci)
            return mf->dlc[i].q_len;
    return 0;
}

int v76_unacked_frames(const v76_t *mf, int dlci)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++)
        if (mf->dlc[i].used && mf->dlc[i].dlci == dlci)
            return seq_dist(mf->dlc[i].va, mf->dlc[i].vs_hi);
    return 0;
}

int v76_unitdata_backlog(const v76_t *mf, int dlci)
{
    int i;

    for (i = 0; i < V76_MAX_DLC; i++)
        if (mf->dlc[i].used && mf->dlc[i].dlci == dlci)
            return mf->dlc[i].ui_len;
    return 0;
}

bool v76_tx_busy(const v76_t *mf)
{
    int i;

    if (mf->cur.active || mf->q_head || mf->bq_n > 8)
        return true;
    for (i = 0; i < V76_MAX_DLC; i++) {
        const dlc_t *d = &mf->dlc[i];

        if (d->used && (d->q_head || d->ui_head || d->ack_pending ||
                        d->va != d->vs_hi))
            return true;
    }
    return false;
}
