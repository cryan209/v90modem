/*
 * clear_channel.c — 64 kbit/s clear channel and V.120 over the DS0
 *
 * See clear_channel.h for what the two modes are and the conventions.
 */
#include "clear_channel.h"

#include <stdlib.h>
#include <string.h>

int cc_line_rate(const clear_channel_t *cc)
{
    if (cc->mode == CC_V110)
        return cc->v110_user_rate;
    return cc->r56 ? 56000 : 64000;
}

/* ---------------------------------------------------------------- */
/* Octet <-> bit stream                                              */
/* ---------------------------------------------------------------- */

static int next_bit(clear_channel_t *cc)
{
    int bit;

    if (cc->mode == CC_CLEAR) {
        bit = cc->get_bit ? cc->get_bit(cc->ctx) : 1;
        return bit < 0 ? 1 : (bit & 1);
    }
    bit = hdlc_tx_get_bit(cc->htx);
    return bit < 0 ? 1 : (bit & 1);
}

static void took_bit(clear_channel_t *cc, int bit)
{
    if (cc->mode == CC_CLEAR) {
        if (cc->put_bit)
            cc->put_bit(cc->ctx, bit);
        return;
    }
    hdlc_rx_put_bit(cc->hrx, bit);
}

/* ---------------------------------------------------------------- */
/* V.120 framing                                                     */
/* ---------------------------------------------------------------- */

void cc_v120_frame_header(const clear_channel_t *cc, uint8_t hdr[4])
{
    /* Figure 6: LLI0 (high six bits), C/R, EA0 = 0; LLI1 (low seven bits),
     * EA1 = 1.  6.2.2.3: C/R is "employed symmetrically for the two
     * directions" -- 0 for a command, 1 for a response (Table 4) -- and UI
     * is a command, so both ends send C/R 0.  (This used to follow LAPD's
     * role convention and send C/R 1 from the answerer: audit V120-1.) */
    hdr[0] = (uint8_t) (((cc->lli >> 7) & 0x3F) << 2);
    hdr[1] = (uint8_t) (((cc->lli & 0x7F) << 1) | 1);
    hdr[2] = 0x03;                                   /* UI, P = 0 */
    /* 3.1.1.5: B = F = 1 is "required" in the asynchronous mode; E = 1,
     * no control-state octet (3.2.3: its use is optional). */
    hdr[3] = CC_V120_H_E | CC_V120_H_B | CC_V120_H_F;
}

/* Queue a built frame (address .. FCS-less body). */
static void v120_queue(clear_channel_t *cc, const uint8_t *frame, int len)
{
    if (hdlc_tx_frame(cc->htx, frame, (size_t) len) == 0) {
        cc->tx_frame_queued = true;
        cc->tx_frames++;
    }
}

/* Pull up to one frame's worth of DTE characters behind a header of
 * 'hdr' octets already in frame[]; returns the count. */
static int v120_pull_data(clear_channel_t *cc, uint8_t *dst)
{
    int n = 0;

    while (n < CC_V120_MAX_DATA) {
        int b = cc->pull(cc->ctx);

        if (b < 0)
            break;
        dst[n++] = (uint8_t) b;
    }
    return n;
}

/* The next information field's header octet and characters, or -1 for
 * nothing to send.  V.120 3.1.1.2/7.2.2: a break goes in a frame with BR = 1
 * after every queued character, and a later frame with BR = 0 ends it; the
 * DTE's data waits for that end, since BR = 0 on a data frame is it. */
static int v120_build_info(clear_channel_t *cc, uint8_t *data, int *n)
{
    uint8_t h = CC_V120_H_E | CC_V120_H_B | CC_V120_H_F;

    *n = 0;
    if (cc->brk_active) {
        if (cc->tx_octets < cc->brk_end_at)
            return -1;
        cc->brk_active = false;               /* this frame is the end */
        *n = cc->pull ? v120_pull_data(cc, data) : 0;
        return h;
    }
    if (cc->pull)
        *n = v120_pull_data(cc, data);
    if (cc->brk_pending && *n < CC_V120_MAX_DATA) {
        cc->brk_pending = false;
        cc->brk_active = true;
        cc->brk_end_at = cc->tx_octets + (uint64_t) cc->brk_ms * 8u;
        return h | CC_V120_H_BR;
    }
    return *n ? h : -1;
}

/* ---- Q.922 multiple-frame acknowledged operation (V.120 4.2) ---- */

#define LF_MOD        128
#define LF_T200_OCTETS 12000u     /* 1.5 s (Q.922 default) */
#define LF_N200       3

/* Address octets for a command (C/R 0) or response (C/R 1), Table 4. */
static void lf_addr(const clear_channel_t *cc, bool response, uint8_t *a)
{
    a[0] = (uint8_t) ((((cc->lli >> 7) & 0x3F) << 2) | (response ? 0x02 : 0));
    a[1] = (uint8_t) (((cc->lli & 0x7F) << 1) | 1);
}

static void lf_send_u(clear_channel_t *cc, bool response, uint8_t ctl, bool pf)
{
    uint8_t f[3];

    lf_addr(cc, response, f);
    f[2] = (uint8_t) (ctl | (pf ? 0x10 : 0));
    v120_queue(cc, f, 3);
}

static void lf_send_s(clear_channel_t *cc, bool response, uint8_t ctl, bool pf)
{
    uint8_t f[4];

    lf_addr(cc, response, f);
    f[2] = ctl;
    f[3] = (uint8_t) ((cc->lf_vr << 1) | (pf ? 1 : 0));
    v120_queue(cc, f, 4);
}

static void lf_reset_vars(clear_channel_t *cc)
{
    cc->lf_vs = cc->lf_va = cc->lf_vnew = cc->lf_vr = 0;
    cc->lf_peer_busy = cc->lf_rej_sent = cc->lf_timer_rec = false;
    cc->lf_pend_ack = cc->lf_enquire = false;
    cc->lf_t200_on = false;
    cc->lf_retries = 0;
}

static void lf_t200_start(clear_channel_t *cc)
{
    cc->lf_t200_on = true;
    cc->lf_t200_at = cc->tx_octets + LF_T200_OCTETS;
}

/* Q.922 5.6.x: N(R) acknowledges everything before it, if it is in range. */
static bool lf_ack_upto(clear_channel_t *cc, int nr)
{
    int outstanding = (cc->lf_vnew - cc->lf_va) & (LF_MOD - 1);
    int acked = (nr - cc->lf_va) & (LF_MOD - 1);

    if (acked > outstanding)
        return false;                       /* invalid N(R) */
    if (acked) {
        cc->lf_va = nr;
        cc->lf_retries = 0;
        if (cc->lf_va == cc->lf_vnew)
            cc->lf_t200_on = false;
        else
            lf_t200_start(cc);
    }
    return true;
}

static void v120_deliver(clear_channel_t *cc, const uint8_t *info, int len);

/* One I-frame: a retransmission from the window, or new DTE data. */
static bool lf_send_i(clear_channel_t *cc)
{
    uint8_t f[4 + 1 + CC_V120_MAX_DATA];
    int ilen, ns;

    if (cc->lf_peer_busy || cc->lf_timer_rec)
        return false;
    if (cc->lf_vs != cc->lf_vnew) {
        ns = cc->lf_vs;
        ilen = cc->lf_win_len[ns & 15];
        memcpy(f + 4, cc->lf_win[ns & 15], (size_t) ilen);
        cc->lf_vs = (cc->lf_vs + 1) & (LF_MOD - 1);
    } else {
        int n;

        if (((cc->lf_vnew - cc->lf_va) & (LF_MOD - 1)) >= CC_LF_K)
            return false;
        int h = v120_build_info(cc, f + 5, &n);

        if (h < 0)
            return false;
        f[4] = (uint8_t) h;
        ilen = 1 + n;
        ns = cc->lf_vnew;
        memcpy(cc->lf_win[ns & 15], f + 4, (size_t) ilen);
        cc->lf_win_len[ns & 15] = ilen;
        cc->lf_vnew = cc->lf_vs = (cc->lf_vnew + 1) & (LF_MOD - 1);
        cc->tx_data_bytes += (uint64_t) n;
    }
    lf_addr(cc, false, f);
    f[2] = (uint8_t) (ns << 1);
    f[3] = (uint8_t) (cc->lf_vr << 1);
    cc->lf_pend_ack = false;               /* N(R) rides on this frame */
    if (!cc->lf_t200_on)
        lf_t200_start(cc);
    v120_queue(cc, f, 4 + ilen);
    return true;
}

static void lf_timers(clear_channel_t *cc)
{
    if (!cc->lf_t200_on || cc->tx_octets < cc->lf_t200_at)
        return;
    cc->lf_t200_on = false;
    if (cc->lf_state == CC_LF_SETUP) {
        if (++cc->lf_retries > LF_N200) {
            /* Nobody answers SABME: a UI-only peer drops it silently.
             * Carry the data in UI frames, as the audit's profile does. */
            cc->lf_state = CC_LF_UI;
            cc->lf_fallbacks++;
        } else {
            lf_send_u(cc, false, 0x6F, true);
            lf_t200_start(cc);
        }
    } else if (cc->lf_state == CC_LF_UP) {
        if (++cc->lf_retries > LF_N200) {
            /* Q.922 5.6.7: N200 exhausted, re-establish the link. */
            cc->lf_state = CC_LF_DOWN;
            cc->lf_resets++;
        } else {
            cc->lf_timer_rec = true;       /* enquire with RR, P = 1 */
            cc->lf_enquire = true;
            lf_t200_start(cc);
        }
    }
}

static void v120_ack_poll(clear_channel_t *cc)
{
    lf_timers(cc);
    if (cc->tx_frame_queued)
        return;
    if (cc->lf_pend_dm) {
        cc->lf_pend_dm = false;
        lf_send_u(cc, true, 0x0F, cc->lf_pend_f);
        return;
    }
    if (cc->lf_pend_ua) {
        cc->lf_pend_ua = false;
        lf_send_u(cc, true, 0x63, cc->lf_pend_f);
        return;
    }
    if (cc->lf_pend_xid) {                  /* 4.2.3: respond and stay in state */
        cc->lf_pend_xid = false;
        lf_send_u(cc, true, 0xAF, false);
        return;
    }
    if (cc->lf_state == CC_LF_DOWN) {
        lf_reset_vars(cc);
        cc->lf_state = CC_LF_SETUP;
        lf_send_u(cc, false, 0x6F, true);
        lf_t200_start(cc);
        return;
    }
    if (cc->lf_state != CC_LF_UP)
        return;
    if (cc->lf_pend_rr_f) {
        cc->lf_pend_rr_f = false;
        cc->lf_pend_ack = false;
        lf_send_s(cc, true, 0x01, true);
        return;
    }
    if (cc->lf_pend_rej) {
        cc->lf_pend_rej = false;
        cc->lf_pend_ack = false;
        lf_send_s(cc, true, 0x09, cc->lf_pend_rej_f);
        return;
    }
    if (cc->lf_enquire) {
        cc->lf_enquire = false;
        lf_send_s(cc, false, 0x01, true);
        return;
    }
    if (lf_send_i(cc))
        return;
    if (cc->lf_pend_ack) {
        cc->lf_pend_ack = false;
        lf_send_s(cc, true, 0x01, false);
    }
}

/* Receive: everything with an address for our LLI that is not a UI frame. */
static void lf_rx(clear_channel_t *cc, const uint8_t *pkt, int len)
{
    bool cmd = (pkt[0] & 0x02) == 0;
    uint8_t ctl = pkt[2];

    if ((ctl & 0x01) == 0) {                       /* I frame */
        int ns, nr;
        bool p;

        if (len < 5 || !cc->v120_ack || cc->lf_state != CC_LF_UP) {
            cc->rx_unsupported++;
            if (cc->lf_state != CC_LF_UP) {         /* Q.922: DM when no link */
                cc->lf_pend_dm = true;
                cc->lf_pend_f = (pkt[3] & 1) != 0;
            }
            return;
        }
        ns = ctl >> 1;
        nr = pkt[3] >> 1;
        p = (pkt[3] & 1) != 0;
        if (!lf_ack_upto(cc, nr)) {
            cc->lf_state = CC_LF_DOWN;            /* invalid N(R): re-establish */
            cc->lf_resets++;
            return;
        }
        if (ns == cc->lf_vr) {
            cc->lf_vr = (cc->lf_vr + 1) & (LF_MOD - 1);
            cc->lf_rej_sent = false;
                    v120_deliver(cc, pkt + 4, len - 4);
            cc->lf_pend_ack = true;
            if (p)
                cc->lf_pend_rr_f = true;
        } else {
            cc->lf_discarded++;
            if (!cc->lf_rej_sent || p) {           /* one REJ per gap, F = P */
                cc->lf_rej_sent = true;
                cc->lf_pend_rej = true;
                cc->lf_pend_rej_f = p;
            }
        }
        return;
    }
    if ((ctl & 0x03) == 0x01) {                    /* S frame: RR, RNR, REJ */
        int nr = pkt[3] >> 1;
        bool pf = (pkt[3] & 1) != 0;

        if (len < 4 || cc->lf_state != CC_LF_UP)
            return;
        if (!lf_ack_upto(cc, nr)) {
            cc->lf_state = CC_LF_DOWN;
            cc->lf_resets++;
            return;
        }
        switch (ctl & 0x0C) {
        case 0x00: cc->lf_peer_busy = false; break;   /* RR */
        case 0x04: cc->lf_peer_busy = true;  break;   /* RNR */
        case 0x08: cc->lf_peer_busy = false;          /* REJ: go back to N(R) */
                   cc->lf_vs = cc->lf_va; cc->lf_rewinds++; break;
        }
        if (cmd && pf)
            cc->lf_pend_rr_f = true;
        if (!cmd && pf && cc->lf_timer_rec) {      /* answer to our enquiry */
            cc->lf_timer_rec = false;
            if (cc->lf_va != cc->lf_vnew) {
                cc->lf_vs = cc->lf_va;             /* nothing heard: resend */
                cc->lf_rewinds++;
                lf_t200_start(cc);
            }
        }
        return;
    }
    switch (ctl & ~0x10) {                         /* U frames */
    case 0x6F:                                     /* SABME */
        if (!cc->v120_ack) {                       /* Q.922: refuse with DM */
            cc->lf_pend_dm = true;
            cc->lf_pend_f = (ctl & 0x10) != 0;
            cc->rx_unsupported++;
            return;
        }
        cc->lf_pend_ua = true;
        cc->lf_pend_f = (ctl & 0x10) != 0;
        if (cc->lf_state == CC_LF_UP)
            cc->lf_resets++;
        if (cc->lf_state != CC_LF_SETUP) {         /* collision: wait for UA */
            lf_reset_vars(cc);
            cc->lf_state = CC_LF_UP;
        }
        break;
    case 0x63:                                     /* UA */
        if (cc->lf_state == CC_LF_SETUP) {
            lf_reset_vars(cc);
            cc->lf_state = CC_LF_UP;
        }
        break;
    case 0x0F:                                     /* DM */
        if (cc->lf_state == CC_LF_SETUP) {
            cc->lf_state = CC_LF_UI;               /* refused: UI frames */
            cc->lf_t200_on = false;
            cc->lf_fallbacks++;
        }
        break;
    case 0x43:                                     /* DISC */
        cc->lf_pend_ua = true;
        cc->lf_pend_f = (ctl & 0x10) != 0;
        if (cc->lf_state == CC_LF_UP) {
            cc->lf_state = CC_LF_DOWN;
            cc->lf_resets++;
        }
        break;
    case 0xAF:                                     /* XID (4.2.2, 4.2.3) */
        if (cmd) {
            cc->lf_pend_xid = true;               /* always answered, any state */
        } else if (cc->vf_state == 1) {
            cc->vf_state = 2;                     /* link verified */
            cc->vf_ok++;
        }
        break;
    default:
        cc->rx_unsupported++;                      /* FRMR and the rest */
        break;
    }
}

/* Queue a frame of whatever the DTE has, if it has anything. */
static void v120_load_frame(clear_channel_t *cc)
{
    uint8_t frame[4 + CC_V120_MAX_DATA];
    int n;

    if (cc->tx_frame_queued)
        return;
    if (cc->v120_ack && cc->lf_state != CC_LF_UI) {
        v120_ack_poll(cc);
        return;
    }
    if (cc->lf_pend_dm) {                  /* refuse a SABME (Q.922 5.5) */
        cc->lf_pend_dm = false;
        lf_send_u(cc, true, 0x0F, cc->lf_pend_f);
        return;
    }
    /* 3.2.4.1: the peer's RR = 0 asserts flow control; no user data until
     * a control-state octet sets it back to 1. */
    if (cc->lf_pend_xid) {                 /* 4.2.2: shall respond to an XID command */
        cc->lf_pend_xid = false;
        lf_send_u(cc, true, 0xAF, false);
        return;
    }
    /* 4.2.2: UI-only link verification, when wanted: XID command, TM20, NM20;
     * no data until the response (or, retries spent, "may begin"). */
    if (cc->vf_enable && !cc->v120_ack && cc->vf_state != 2) {
        if (cc->vf_state == 0 || cc->tx_octets >= cc->vf_at) {
            if (cc->vf_state == 1 && ++cc->vf_retries > CC_V120_NM20) {
                cc->vf_state = 2;                 /* begin transmission anyway */
                cc->vf_gave_up++;
            } else {
                cc->vf_state = 1;
                cc->vf_at = cc->tx_octets + CC_V120_TM20_OCTETS;
                lf_send_u(cc, false, 0xAF, false);
                return;
            }
        } else {
            return;
        }
    }
    if (!cc->v120_peer_rr)
        return;
    {
        int h = v120_build_info(cc, frame + 4, &n);

        if (h < 0)
            return;
        cc_v120_frame_header(cc, frame);
        frame[3] = (uint8_t) h;
    }
    v120_queue(cc, frame, 4 + n);
    cc->tx_data_bytes += (uint64_t) n;
}

static void v120_underflow(void *user_data)
{
    clear_channel_t *cc = (clear_channel_t *) user_data;

    cc->tx_frame_queued = false;
    v120_load_frame(cc);
}

/* The information field: H, an optional control-state octet, the data. */
static void v120_deliver(clear_channel_t *cc, const uint8_t *info, int len)
{
    int pos = 1;
    uint8_t h = info[0];

    /* 3.1: the header is H plus at most ONE control-state octet.  With
     * H.E = 0 the CS must be present and carry E = 1 (3.1.2.1: "the receipt
     * of a control state octet with the E bit set to 0 shall be considered
     * an error").  Reject before any payload is delivered -- audit V120-2. */
    if (!(h & CC_V120_H_E)) {
        if (len < 2 || !(info[1] & CC_V120_CS_E)) {
            cc->rx_bad_frames++;
            return;
        }
        /* 3.2.3.3/3.2.4.1: RR(R) follows the received RR; RR = 0 holds
         * our user data back.  DR and SR map V.24 circuits this byte
         * interface does not have. */
        cc->v120_peer_rr = (info[1] & CC_V120_CS_RR) != 0;
        pos = 2;
    }
    cc->rx_frames++;
    if (!(h & CC_V120_H_BR) && cc->rx_in_break) {   /* BR = 0: the end (3.1.1.2) */
        cc->rx_in_break = false;
        if (cc->brk_cb)
            cc->brk_cb(cc->ctx, false);
    }
    for (; pos < len; pos++) {
        if (cc->push)
            cc->push(cc->ctx, info[pos]);
        cc->rx_data_bytes++;
    }
    /* 7.2.2 (5): a break follows the frame's characters. */
    if ((h & CC_V120_H_BR) && !cc->rx_in_break) {
        cc->rx_in_break = true;
        cc->rx_breaks++;
        if (cc->brk_cb)
            cc->brk_cb(cc->ctx, true);
    }
}

static void v120_frame(void *user_data, const uint8_t *pkt, int len, int ok)
{
    clear_channel_t *cc = (clear_channel_t *) user_data;
    int lli;

    if (len < 0)
        return;            /* a status report, not a frame */
    if (!ok || len < 3) {
        cc->rx_bad_frames++;
        return;
    }
    /* Figure 6: the circuit-mode address is two octets, EA 0 then 1. */
    if ((pkt[0] & 0x01) != 0 || (pkt[1] & 0x01) != 1) {
        cc->rx_bad_frames++;
        return;
    }
    /* 6.2.2.1: LLI0 (bits 8..3 of octet 1) then LLI1 (bits 8..2 of octet
     * 2).  Only our link's frames reach the DTE: LLI 0 is in-channel
     * signalling, 8191 layer management, and any other value is another
     * logical link (Table 3) -- audit V120-3.  C/R is not checked for UI:
     * UI is a command (C/R 0), but builds before 2026-10-06 sent C/R 1
     * from the answerer and a lenient receiver costs nothing here. */
    lli = (((pkt[0] >> 2) & 0x3F) << 7) | ((pkt[1] >> 1) & 0x7F);
    if (lli != cc->lli) {
        cc->rx_other_lli++;
        return;
    }
    /* UI (P/F either way) carries the V.120 header directly; everything
     * else is Q.922 acknowledged operation (4.2) or XID (4.2.2, optional
     * link verification, not implemented). */
    if ((pkt[2] & ~0x10) == 0x03) {
        if (len < 4) {
            cc->rx_bad_frames++;
            return;
        }
        v120_deliver(cc, pkt + 3, len - 3);
        return;
    }
    lf_rx(cc, pkt, len);
}

/* ---------------------------------------------------------------- */
/* V.110 (02/2000): RA0 + RA1 + RA2 for an asynchronous DTE          */
/* ---------------------------------------------------------------- */

/* Table 8, less 50 bit/s (5 data units; this profile is 8N1). */
const int cc_v110_rates[] = {
    75, 110, 150, 200, 300, 600, 1200, 2400, 3600, 4800, 7200, 9600,
    12000, 14400, 19200, 24000, 28800, 38400
};
const int cc_v110_n_rates = (int) (sizeof(cc_v110_rates) / sizeof(cc_v110_rates[0]));

/* Table 8's "RA0/RA1 rate" column. */
int cc_v110_ra0_rate(int user_rate)
{
    int i;

    for (i = 0; i < cc_v110_n_rates; i++)
        if (cc_v110_rates[i] == user_rate)
            break;
    if (i == cc_v110_n_rates)
        return 0;
    if (user_rate <= 600)   return 600;
    if (user_rate <= 1200)  return 1200;
    if (user_rate <= 2400)  return 2400;
    if (user_rate <= 4800)  return 4800;
    if (user_rate <= 9600)  return 9600;
    if (user_rate <= 19200) return 19200;
    return 38400;
}

const char *cc_v110_cause_name(cc_v110_cause_t c)
{
    switch (c) {
    case CC_V110_CAUSE_NONE:      return "none";
    case CC_V110_CAUSE_T1:        return "no S = X = ON within T1 (7.1.2.4)";
    case CC_V110_CAUSE_SYNC_LOST: return "frame synchronization not recovered in 3 s (7.1.5)";
    case CC_V110_CAUSE_REMOTE:    return "far end's disconnect request (7.1.4.2)";
    case CC_V110_CAUSE_LOCAL:     return "our disconnect request acknowledged (7.1.4.3)";
    case CC_V110_CAUSE_T2:        return "our disconnect request unanswered within T2 (7.1.4.1)";
    }
    return "?";
}

/* T1 (7.1.2.2, suggested 10 s) and 7.1.5 e)'s 3 s, in DS0 octets. */
#define V110_T1_OCTETS        80000u
#define V110_RESYNC_OCTETS    24000u
/* T2 (7.1.4.1, suggested 5 s): the far end has not answered our request. */
#define V110_T2_OCTETS        40000u
/* 5.1.3.2: loss only after three consecutive frames with a framing error. */
#define V110_LOSS_FRAMES      3
/* 5.1.3.1/7.1.2.4: a status change must persist this many frames. */
#define V110_PERSIST_FRAMES   2
/* 7.1.5 e): "at least three" frames of the disconnect request. */
#define V110_DISC_FRAMES      3
/* 5.3.5: more than 2M zeros cannot be characters, even with one stop
 * element deleted between two NULs (19 zeros); with M = 10 it is a break. */
#define V110_BREAK_ZEROS      20

static bool v110_106_on(const clear_channel_t *cc)
{
    /* 7.1.2.4 d): N bits after 109; 7.1.5 c) and 5.4.2: OFF while the far
     * end's X is OFF; 7.1.5: OFF while our own framing is lost. */
    return cc->v110_state == CC_V110_CONNECTED && cc->v110_synced
        && cc->v110_rem_x_on && cc->v110_n_count >= CC_V110_N_BITS;
}

/* RA0 transmit: the next element of the start/stop signal.  A character in
 * progress always completes; a new one starts only when allowed (5.4.2:
 * "after completion of the character in progress"). */
static int v110_async_tx_bit(clear_channel_t *cc, bool allow_new)
{
    int b;

    if (cc->v110_tx_bits > 0) {
        b = cc->v110_tx_shift & 1;
        cc->v110_tx_shift >>= 1;
        cc->v110_tx_bits--;
        return b;
    }
    if (cc->v110_tx_marks > 0) {
        cc->v110_tx_marks--;
        return 1;
    }
    if (allow_new && cc->v110_brk_bits > 0) {   /* 5.3.5: all zeros */
        if (--cc->v110_brk_bits == 0)
            cc->v110_tx_marks = 12;           /* idle mark before the next start */
        return 0;
    }
    if (allow_new && cc->brk_pending) {
        int bits = (int) ((int64_t) cc->brk_ms * cc->v110_user_rate / 1000);

        /* More than 2M zeros is what the far end reads as a break; M = 10. */
        cc->brk_pending = false;
        cc->v110_brk_bits = (bits < 24 ? 24 : bits) - 1;
        if (cc->v110_brk_bits == 0)
            cc->v110_tx_marks = 12;
        return 0;
    }
    if (!allow_new || !cc->pull || (b = cc->pull(cc->ctx)) < 0)
        return 1;                        /* idle: stop polarity */
    /* start (0), 8 data LSB first; stop element(s) from v110_tx_marks. */
    cc->v110_tx_shift = (uint16_t) ((unsigned) (b & 0xFF) << 1);
    cc->v110_tx_bits = 9;
    if (cc->v110_user_rate >= 600) {
        /* 5.3.3: "padded by the addition of stop elements to fit" the RA0
         * rate -- 7200 bit/s characters on a 9600 bit/s stream get 3 1/3
         * extra stop elements on average.  Equal rates: plain 8N1. */
        uint64_t scaled = cc->v110_tx_pace + 10u * (uint64_t) cc->v110_ra0_rate;
        int bits = (int) (scaled / (uint64_t) cc->v110_user_rate);

        cc->v110_tx_pace = scaled % (uint64_t) cc->v110_user_rate;
        cc->v110_tx_marks = bits - 9;
    } else {
        cc->v110_tx_marks = 1;
    }
    cc->tx_data_bytes++;
    return v110_async_tx_bit(cc, false);
}

/* One bit of the 2^n x 600 bit/s RA0 stream. */
static int v110_ra0_tx_bit(clear_channel_t *cc)
{
    bool allow = v110_106_on(cc);

    if (cc->v110_user_rate >= 600)
        return v110_async_tx_bit(cc, allow);
    /* Below 600 bit/s the 600 bit/s stream samples the start/stop signal
     * (the rates of 5.3.5's "asynchronous rate lower than the synchronous
     * rate"): each element lasts 600/r stream bits, 5 or 6 at 110 bit/s. */
    if (cc->v110_os_acc <= 0) {
        cc->v110_os_bit = v110_async_tx_bit(cc, allow);
        cc->v110_os_acc += 600;
    }
    cc->v110_os_acc -= cc->v110_user_rate;
    return cc->v110_os_bit;
}

/* RA1 transmit: build one 80-bit frame (Table 2 with Tables 6a-6e). */
static void v110_build_frame(clear_channel_t *cc)
{
    static const uint8_t status_octet[] = { 1, 2, 3, 4, 6, 7, 8, 9 };
    uint8_t *f = cc->v110_txf;
    bool down = cc->v110_state == CC_V110_DISCONNECTING
             || cc->v110_state == CC_V110_DOWN;
    int s_bit, x_bit, slot = 0;
    int nd = 48 / cc->v110_rep;
    uint8_t d[48];

    if (down) {
        /* 7.1.4.1: S OFF, X kept ON, D = 0; 7.1.5 e): all status OFF. */
        s_bit = 1;
        x_bit = cc->v110_cause == CC_V110_CAUSE_SYNC_LOST ? 1 : 0;
        cc->v110_disc_frames++;
    } else {
        s_bit = cc->v110_tx_s_on ? 0 : 1;
        if (cc->v110_state == CC_V110_CONNECTED && cc->room) {
            int fr = 0, sz = 0;

            cc->room(cc->ctx, &fr, &sz);
            if (sz > 0) {
                if (!cc->v110_rx_hold && fr * 4 < sz) {
                    cc->v110_rx_hold = true;
                    cc->v110_flow_holds++;
                } else if (cc->v110_rx_hold && fr * 2 > sz) {
                    cc->v110_rx_hold = false;
                }
            }
        } else {
            cc->v110_rx_hold = false;
        }
        x_bit = (cc->v110_tx_x_on && !cc->v110_rx_hold) ? 0 : 1;
    }
    for (int i = 0; i < nd; i++) {
        if (down)
            d[i] = 0;
        else if (cc->v110_state == CC_V110_CONNECTED) {
            /* 7.1.2.4 b)/e): data once 106 is ON, binary 1 before. */
            d[i] = (uint8_t) v110_ra0_tx_bit(cc);
            if (cc->v110_n_count < CC_V110_N_BITS)
                cc->v110_n_count++;
        } else
            d[i] = 1;                    /* 7.1.2.1 b) */
    }
    memset(f, 1, CC_V110_FRAME_BITS);
    memset(f, 0, 8);                     /* octet 0 */
    for (int k = 0; k < 8; k++) {
        int o = status_octet[k];

        for (int b = 1; b <= 6; b++, slot++)
            f[o * 8 + b] = d[slot / cc->v110_rep];
        /* bit 8: S1, X, S3, S4, S6, X, S8, S9 */
        f[o * 8 + 7] = (uint8_t) ((o == 2 || o == 7) ? x_bit : s_bit);
    }
    /* Octet 5: 1, E1..E7.  E4-E6 unused, so ONE (Table 5 Note 3); E7 = 1,
     * except 0 in every fourth frame at 600 bit/s (Note 2). */
    f[41] = (cc->v110_e123 >> 2) & 1;
    f[42] = (cc->v110_e123 >> 1) & 1;
    f[43] = cc->v110_e123 & 1;
    f[44] = f[45] = f[46] = 1;
    f[47] = (cc->v110_ra0_rate == 600 && (cc->v110_tx_frame_no & 3) == 3) ? 0 : 1;
    cc->v110_tx_frame_no++;
    cc->tx_frames++;
}

static int v110_tx_ir_bit(clear_channel_t *cc)
{
    if (cc->v110_txf_pos == 0)
        v110_build_frame(cc);
    {
        int b = cc->v110_txf[cc->v110_txf_pos];

        cc->v110_txf_pos = (cc->v110_txf_pos + 1) % CC_V110_FRAME_BITS;
        return b;
    }
}

/* ---- receive ---- */

static void v110_push(clear_channel_t *cc, uint8_t b)
{
    if (cc->push)
        cc->push(cc->ctx, b);
    cc->rx_data_bytes++;
}

static void v110_ra0_rx_reset(clear_channel_t *cc)
{
    cc->v110_rx_bits = -1;
    cc->v110_zero_run = 0;
    cc->v110_held_nuls = 0;
    cc->v110_rx_prev = 1;
}

/* RA0 receive at user rates of 600 and up: one stream bit per element,
 * V.14's technique (5.3.1, 5.3.4: a deleted stop element re-inserted). */
static void v110_ra0_rx_bit(clear_channel_t *cc, int bit)
{
    if (bit) {
        cc->v110_zero_run = 0;
        while (cc->v110_held_nuls > 0) {     /* they were characters */
            cc->v110_held_nuls--;
            v110_push(cc, 0);
        }
    } else if (++cc->v110_zero_run == V110_BREAK_ZEROS) {
        /* 5.3.5: a break.  The byte interface has no break to give the
         * DTE, so drop the NULs it was read as and count it. */
        cc->v110_held_nuls = 0;
        cc->rx_breaks++;
        cc->v110_rx_bits = -2;
        cc->rx_in_break = true;
        if (cc->brk_cb)
            cc->brk_cb(cc->ctx, true);
        return;
    }
    if (cc->v110_rx_bits == -2) {            /* break: wait for stop polarity */
        if (bit) {
            cc->v110_rx_bits = -1;
            if (cc->rx_in_break) {
                cc->rx_in_break = false;
                if (cc->brk_cb)
                    cc->brk_cb(cc->ctx, false);
            }
        }
        return;
    }
    if (cc->v110_rx_bits == -1) {
        if (!bit) {
            cc->v110_rx_bits = 0;
            cc->v110_rx_shift = 0;
        }
        return;
    }
    if (cc->v110_rx_bits < 8) {
        cc->v110_rx_shift |= (uint16_t) (bit << cc->v110_rx_bits);
        cc->v110_rx_bits++;
        return;
    }
    /* Stop element position. */
    if (bit) {
        v110_push(cc, (uint8_t) cc->v110_rx_shift);
        cc->v110_rx_bits = -1;
        return;
    }
    /* 0: a deleted stop element, so this is the next start (5.3.4) -- or a
     * break under way, which an all-zero character cannot yet rule out. */
    if (cc->v110_rx_shift == 0) {
        if (cc->v110_held_nuls < 2)
            cc->v110_held_nuls++;
    } else {
        v110_push(cc, (uint8_t) cc->v110_rx_shift);
    }
    cc->v110_rx_bits = 0;
    cc->v110_rx_shift = 0;
}

/* RA0 receive below 600 bit/s: a UART on the 600 bit/s samples.  Element
 * j is read at the first sample n after the start edge with
 * (2n + 1) r >= (2j + 1) 600, i.e. as near its middle as the grid allows. */
static void v110_ra0_rx_sample(clear_channel_t *cc, int bit)
{
    int r = cc->v110_user_rate;

    if (cc->rx_in_break && bit) {            /* line back to mark: break over */
        cc->rx_in_break = false;
        if (cc->brk_cb)
            cc->brk_cb(cc->ctx, false);
    }
    if (cc->v110_rx_bits < 0) {
        if (cc->v110_rx_prev && !bit) {
            cc->v110_rx_bits = 0;
            cc->v110_rx_n = 0;
            cc->v110_rx_shift = 0;
        }
        cc->v110_rx_prev = bit;
        if (cc->v110_rx_bits < 0)
            return;
    }
    cc->v110_rx_prev = bit;
    if ((2 * cc->v110_rx_n + 1) * r >= (2 * cc->v110_rx_bits + 1) * 600) {
        int j = cc->v110_rx_bits;

        if (j == 0 && bit) {
            cc->v110_rx_bits = -1;           /* a glitch, not a start */
            return;
        }
        if (j >= 1 && j <= 8)
            cc->v110_rx_shift |= (uint16_t) (bit << (j - 1));
        if (j == 9) {
            if (!bit && cc->v110_rx_shift == 0) {   /* all zeros, no stop: a break */
                if (!cc->rx_in_break) {
                    cc->rx_in_break = true;
                    cc->rx_breaks++;
                    if (cc->brk_cb)
                        cc->brk_cb(cc->ctx, true);
                }
                cc->v110_rx_bits = -1;
                return;
            }
            v110_push(cc, (uint8_t) cc->v110_rx_shift);
            cc->v110_rx_bits = -1;
            return;
        }
        cc->v110_rx_bits++;
    }
    cc->v110_rx_n++;
}

static bool v110_framing_ok(const uint8_t *f)
{
    for (int k = 0; k < 8; k++)
        if (f[k])
            return false;
    for (int o = 1; o <= 9; o++)
        if (!f[o * 8])
            return false;
    return true;
}

static void v110_lose_sync(clear_channel_t *cc)
{
    cc->v110_synced = false;
    cc->v110_verify = false;
    cc->v110_hist_n = 0;
    cc->v110_bad_run = 0;
    cc->v110_sync_losses++;
    if (cc->v110_state == CC_V110_DISCONNECTING) {
        /* 7.1.4.3: loss of framing acknowledges our request too. */
        cc->v110_state = CC_V110_DOWN;
        cc->v110_cause = CC_V110_CAUSE_LOCAL;
        return;
    }
    /* 7.1.5 a)/b): circuit 104 to binary 1 (stop delivering, and lose any
     * half-received character), our X OFF. */
    v110_ra0_rx_reset(cc);
    cc->v110_tx_x_on = false;
    cc->v110_lost_at = cc->rx_octets;
}

static void v110_frame(clear_channel_t *cc, const uint8_t *f)
{
    static const uint8_t s_pos[] = { 15, 31, 39, 55, 71, 79 };   /* S1 S3 S4 S6 S8 S9 */
    int nd = 48 / cc->v110_rep, s_zero = 0, slot = 0;
    uint8_t d[48];
    bool d_all_zero = true;

    if (!v110_framing_ok(f)) {
        cc->v110_frame_errors++;
        if (++cc->v110_bad_run >= V110_LOSS_FRAMES) {
            v110_lose_sync(cc);
            return;
        }
        /* A framing bit error alone does not mean misalignment (5.1.3.2);
         * the frame's contents are still the best there is. */
    } else {
        cc->v110_bad_run = 0;
    }
    cc->rx_frames++;
    for (int k = 0; k < 6; k++)
        s_zero += !f[s_pos[k]];
    /* 7.1: SA and SB "treated as a single sequence"; a majority decides. */
    if (s_zero >= 4)
        cc->v110_rem_s_on = true;
    else if (s_zero <= 2)
        cc->v110_rem_s_on = false;
    if (!f[23] && !f[63])
        cc->v110_rem_x_on = true;
    else if (f[23] && f[63])
        cc->v110_rem_x_on = false;
    {
        uint8_t e = (uint8_t) ((f[41] << 2) | (f[42] << 1) | f[43]);

        cc->v110_rx_e123 = e;
        if (e != cc->v110_e123)
            cc->v110_rate_mismatch++;
    }
    /* D bits, majority over the Table 6a/6b/6c repetitions (ties go to the
     * first copy). */
    {
        static const uint8_t data_octet[] = { 1, 2, 3, 4, 6, 7, 8, 9 };
        int ones[48] = { 0 }, first[48];

        for (int k = 0; k < 8; k++)
            for (int b = 1; b <= 6; b++, slot++) {
                int i = slot / cc->v110_rep;

                if (slot % cc->v110_rep == 0)
                    first[i] = f[data_octet[k] * 8 + b];
                ones[i] += f[data_octet[k] * 8 + b];
            }
        for (int i = 0; i < nd; i++) {
            int twice = 2 * ones[i];

            d[i] = (uint8_t) (twice > cc->v110_rep ? 1
                              : twice < cc->v110_rep ? 0 : first[i]);
            if (d[i])
                d_all_zero = false;
        }
    }

    switch (cc->v110_state) {
    case CC_V110_SYNCED:
        if (cc->v110_rem_s_on && cc->v110_rem_x_on) {
            if (++cc->v110_rem_on_run >= V110_PERSIST_FRAMES) {
                /* 7.1.2.4 a)/c): 107 and 109 ON, data bits to 104. */
                cc->v110_state = CC_V110_CONNECTED;
                cc->v110_n_count = 0;
                cc->v110_rem_disc_run = 0;
            }
        } else {
            cc->v110_rem_on_run = 0;
        }
        break;
    case CC_V110_CONNECTED:
        /* 7.1.4.2: S from ON to OFF with the data bits at 0. */
        if (!cc->v110_rem_s_on && d_all_zero) {
            if (++cc->v110_rem_disc_run >= V110_PERSIST_FRAMES) {
                cc->v110_state = CC_V110_DOWN;
                cc->v110_cause = CC_V110_CAUSE_REMOTE;
                cc->v110_held_nuls = 0;
                return;
            }
        } else {
            cc->v110_rem_disc_run = 0;
        }
        for (int i = 0; i < nd; i++) {
            if (cc->v110_user_rate >= 600)
                v110_ra0_rx_bit(cc, d[i]);
            else
                v110_ra0_rx_sample(cc, d[i]);
        }
        break;
    case CC_V110_DISCONNECTING:
        if (!cc->v110_rem_s_on) {        /* 7.1.4.3 */
            cc->v110_state = CC_V110_DOWN;
            cc->v110_cause = CC_V110_CAUSE_LOCAL;
        }
        break;
    default:
        break;
    }
}

static void v110_sync_found(clear_channel_t *cc)
{
    cc->v110_synced = true;
    cc->v110_bad_run = 0;
    if (cc->v110_state == CC_V110_SEARCH) {
        /* 7.1.2.3: S and X ON in the frames we send. */
        cc->v110_state = CC_V110_SYNCED;
        cc->v110_tx_s_on = cc->v110_tx_x_on = true;
        cc->v110_rem_on_run = 0;
    } else if (cc->v110_state == CC_V110_CONNECTED) {
        /* 7.1.5 f)/g): X back ON; 106 again after N bits. */
        cc->v110_tx_x_on = true;
        cc->v110_n_count = 0;
    }
    cc->v110_sync_seen = true;
}

/* One bit of the intermediate-rate stream (RA2 already undone). */
static void v110_rx_ir_bit(clear_channel_t *cc, int bit)
{
    if (cc->v110_synced) {
        cc->v110_hist[cc->v110_hist_n++] = (uint8_t) bit;
        if (cc->v110_hist_n == CC_V110_FRAME_BITS) {
            cc->v110_hist_n = 0;
            v110_frame(cc, cc->v110_hist);
        }
        return;
    }
    if (cc->v110_verify) {
        /* 5.1.3.1: "at least two 17-bit alignment patterns in consecutive
         * frames" -- the second frame must align as well. */
        cc->v110_hist[cc->v110_hist_n++] = (uint8_t) bit;
        if (cc->v110_hist_n < CC_V110_FRAME_BITS)
            return;
        cc->v110_hist_n = 0;
        cc->v110_verify = false;
        if (v110_framing_ok(cc->v110_hist)) {
            v110_sync_found(cc);
            v110_frame(cc, cc->v110_hist);
        }
        return;
    }
    /* Hunting: slide an 80-bit window until it is a frame.  Outside octet
     * 0 every octet begins with a 1, so eight zeros followed by the 1 of
     * octet 1 occur nowhere else in a frame. */
    if (cc->v110_hist_n < CC_V110_FRAME_BITS) {
        cc->v110_hist[cc->v110_hist_n++] = (uint8_t) bit;
    } else {
        memmove(cc->v110_hist, cc->v110_hist + 1, CC_V110_FRAME_BITS - 1);
        cc->v110_hist[CC_V110_FRAME_BITS - 1] = (uint8_t) bit;
    }
    if (cc->v110_hist_n == CC_V110_FRAME_BITS && v110_framing_ok(cc->v110_hist)) {
        cc->v110_verify = true;
        cc->v110_hist_n = 0;
    }
}

static void v110_timers(clear_channel_t *cc)
{
    switch (cc->v110_state) {
    case CC_V110_SEARCH:
    case CC_V110_SYNCED:
        /* 7.1.2.4: no 107 ON within T1 -> disconnect per 7.1.4. */
        if (cc->rx_octets - cc->v110_start_at >= V110_T1_OCTETS) {
            cc->v110_state = CC_V110_DOWN;
            cc->v110_cause = CC_V110_CAUSE_T1;
        }
        break;
    case CC_V110_CONNECTED:
        if (!cc->v110_synced
            && cc->rx_octets - cc->v110_lost_at >= V110_RESYNC_OCTETS) {
            cc->v110_state = CC_V110_DOWN;
            cc->v110_cause = CC_V110_CAUSE_SYNC_LOST;
        }
        break;
    case CC_V110_DISCONNECTING:
        /* 7.1.4.1: guard against the far end never responding.  On ISDN
         * T2 clears the call on the D channel; here, the SIP call. */
        if (cc->rx_octets - cc->v110_disc_at >= V110_T2_OCTETS) {
            cc->v110_state = CC_V110_DOWN;
            cc->v110_cause = CC_V110_CAUSE_T2;
        }
        break;
    default:
        break;
    }
}

void cc_v110_disconnect(clear_channel_t *cc)
{
    if (cc->mode != CC_V110 || cc->v110_state == CC_V110_DOWN
        || cc->v110_state == CC_V110_DISCONNECTING)
        return;
    cc->v110_state = CC_V110_DISCONNECTING;
    cc->v110_disc_frames = 0;
    cc->v110_disc_at = cc->rx_octets;
    /* 7.1.4.1 b): 106 OFF.  A character already on its way out is cut
     * short -- c) puts binary 0 in the data bits from the next frame. */
    cc->v110_tx_bits = cc->v110_tx_marks = 0;
}

bool cc_v110_finished(const clear_channel_t *cc)
{
    return cc->mode == CC_V110 && cc->v110_state == CC_V110_DOWN
        && cc->v110_disc_frames >= V110_DISC_FRAMES;
}

int cc_init_v110(clear_channel_t *cc, int user_rate,
                 cc_pull_byte_fn pull, cc_push_byte_fn push, void *ctx)
{
    int ra0 = cc_v110_ra0_rate(user_rate);

    memset(cc, 0, sizeof(*cc));
    if (ra0 == 0)
        return -1;
    cc->mode = CC_V110;
    cc->v110_user_rate = user_rate;
    cc->v110_ra0_rate = ra0;
    /* Table 1 / Table 8: the RA1 rate, and the first step's repetition. */
    cc->v110_ir_bits = ra0 <= 4800 ? 1 : ra0 == 9600 ? 2 : ra0 == 19200 ? 4 : 8;
    cc->v110_rep = ra0 == 600 ? 8 : ra0 == 1200 ? 4 : ra0 == 2400 ? 2 : 1;
    /* Table 5, E1 E2 E3. */
    cc->v110_e123 = ra0 == 600 ? 0x4 : ra0 == 1200 ? 0x2 : ra0 == 2400 ? 0x6 : 0x3;
    cc->pull = pull;
    cc->push = push;
    cc->ctx = ctx;
    /* 7.1.2.1: frames, D = 1, S = X = OFF. */
    cc->v110_state = CC_V110_SEARCH;
    v110_ra0_rx_reset(cc);
    return 0;
}

/* ---------------------------------------------------------------- */
/* Lifecycle and the octet interface                                 */
/* ---------------------------------------------------------------- */

int cc_init_clear(clear_channel_t *cc, bool r56,
                  cc_get_bit_fn get_bit, cc_put_bit_fn put_bit, void *ctx)
{
    memset(cc, 0, sizeof(*cc));
    cc->mode = CC_CLEAR;
    cc->r56 = r56;
    cc->get_bit = get_bit;
    cc->put_bit = put_bit;
    cc->ctx = ctx;
    return 0;
}

void cc_v120_set_verify(clear_channel_t *cc, bool on)
{
    cc->vf_enable = on;
    cc->vf_state = 0;
}

void cc_v120_set_ack(clear_channel_t *cc, bool ack)
{
    cc->v120_ack = ack;
    cc->lf_state = ack ? CC_LF_DOWN : CC_LF_UI;
}

int cc_init_v120(clear_channel_t *cc, bool r56, bool caller,
                 cc_pull_byte_fn pull, cc_push_byte_fn push, void *ctx)
{
    memset(cc, 0, sizeof(*cc));
    cc->mode = CC_V120;
    cc->r56 = r56;
    cc->caller = caller;
    cc->lli = CC_V120_DEFAULT_LLI;
    cc->v120_peer_rr = true;     /* 3.2.3.1: assume 1 until a CS arrives */
    cc->lf_state = CC_LF_UI;
    cc->pull = pull;
    cc->push = push;
    cc->ctx = ctx;
    cc->htx = hdlc_tx_init(NULL, false, 1, false, v120_underflow, cc);
    cc->hrx = hdlc_rx_init(NULL, false, true, 1, v120_frame, cc);
    if (!cc->htx || !cc->hrx) {
        cc_release(cc);
        return -1;
    }
    hdlc_tx_set_max_frame_len(cc->htx, 5 + CC_V120_MAX_DATA);
    /* SpanDSP's transmitter starts with no flag at all, so a frame queued
     * at once would begin at the first octet with nothing to sync a
     * receiver to.  Open with flags, as a line idles before data. */
    hdlc_tx_flags(cc->htx, 2);
    hdlc_rx_set_max_frame_len(cc->hrx, 4 + CC_V120_MAX_DATA + 16);
    return 0;
}

void cc_release(clear_channel_t *cc)
{
    if (cc->htx)
        hdlc_tx_free(cc->htx);
    if (cc->hrx)
        hdlc_rx_free(cc->hrx);
    cc->htx = NULL;
    cc->hrx = NULL;
}

void cc_tx(clear_channel_t *cc, uint8_t *octets, int n)
{
    int nbits = cc->r56 ? 7 : 8;

    if (cc->mode == CC_V110) {
        /* RA2 (I.460): the intermediate rate in the first 1, 2, 4 or 8 bits
         * of the octet -- bit 1, transmitted first, is the octet's MSB on
         * the DS0 -- and every unused bit set to 1. */
        for (int i = 0; i < n; i++) {
            uint8_t o = 0xFF;

            for (int b = 0; b < cc->v110_ir_bits; b++)
                if (!v110_tx_ir_bit(cc))
                    o = (uint8_t) (o & ~(0x80 >> b));
            octets[i] = o;
        }
        cc->tx_octets += (uint64_t) n;
        return;
    }

    if (cc->mode == CC_V120)
        v120_load_frame(cc);   /* also runs the acknowledged-mode timers */
    for (int i = 0; i < n; i++) {
        uint8_t o = 0;

        for (int b = 0; b < nbits; b++)
            o = (uint8_t) (o | (next_bit(cc) << (7 - b)));
        if (cc->r56)
            o |= 0x01;
        octets[i] = o;
    }
    cc->tx_octets += (uint64_t) n;
}

void cc_rx(clear_channel_t *cc, const uint8_t *octets, int n)
{
    int nbits = cc->r56 ? 7 : 8;

    if (cc->mode == CC_V110) {
        if (cc->v110_state == CC_V110_DOWN)
            return;
        for (int i = 0; i < n; i++) {
            for (int b = 0; b < cc->v110_ir_bits; b++)
                v110_rx_ir_bit(cc, (octets[i] >> (7 - b)) & 1);
            cc->rx_octets++;
        }
        v110_timers(cc);
        return;
    }

    for (int i = 0; i < n; i++)
        for (int b = 0; b < nbits; b++)
            took_bit(cc, (octets[i] >> (7 - b)) & 1);
    cc->rx_octets += (uint64_t) n;
}

void cc_v110_set_rx_room(clear_channel_t *cc, cc_room_fn room)
{
    cc->room = room;
}

void cc_set_break_cb(clear_channel_t *cc, cc_break_fn fn)
{
    cc->brk_cb = fn;
}

void cc_send_break(clear_channel_t *cc, int ms)
{
    if (cc->mode == CC_CLEAR)
        return;
    cc->brk_ms = ms < 1 ? 1 : ms;
    cc->brk_pending = true;
}
