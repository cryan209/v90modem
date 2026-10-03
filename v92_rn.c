/*
 * v92_rn.c — V.92 9.8 rate renegotiation, 9.9 fast parameter exchange and
 * 9.11 cleardown (Amd.1 item 6), both roles.  See v92_rn.h.
 */
#include "v92_rn.h"

#include <string.h>

#define R_SYMBOLS 384
#define RBAR_SYMBOLS 24
#define TRN_CHUNK 96                  /* a multiple of 6 and of 12 */
#define TRN2U_MIN 2400                /* 9.8.2.1.2: may stop after 2400T */
#define TRN2_MAX 16008
#define TRN2U_AFTER_E_MAX 8004        /* 9.8.2.1.5/6 */
#define MS100 800                     /* 100 ms in T */

/* Unit lengths on the line.  The procedure does not depend on them beyond
 * the 100 ms windows; they are what the codecs produce: SUVd is 52 bits at
 * one per T filled to a multiple of 6, SUVu 72 bits at two per T (4-point
 * TRN2u), CP lengths a representative constellation set. */
static int frame_symbols(const v92_rn_t *rn, v92_rn_tx_kind_t k)
{
    bool d = rn->role == V92_RN_DIGITAL;

    switch (k) {
    case V92_RN_TX_SUV: return d ? 54 : 36;
    case V92_RN_TX_CP:  return d ? 300 : 450;
    case V92_RN_TX_E:   return d ? 18 : 12;
    case V92_RN_TX_FB1: return 48;
    case V92_RN_TX_B1:  return 48;
    default:            return 0;
    }
}

const char *v92_rn_state_name(v92_rn_state_t s)
{
    static const char *n[] = {
        "IDLE", "WAIT_RBAR", "SEND_R", "TRN", "SUV1", "SUV_ACK", "SILENT",
        "TRN_AFTER_E", "FPE_WAIT_R", "CORE", "SEND_E", "WAIT_B1", "CLEARDOWN", "DONE"
    };
    return (unsigned)s < sizeof(n)/sizeof(n[0]) ? n[s] : "?";
}

static void act(v92_rn_t *rn, v92_rn_action_t a)
{
    if (rn->n_actions < (int)(sizeof(rn->actions)/sizeof(rn->actions[0])))
        rn->actions[rn->n_actions++] = a;
}

v92_rn_action_t v92_rn_take_action(v92_rn_t *rn)
{
    v92_rn_action_t a;

    if (rn->n_actions == 0)
        return V92_RN_ACT_NONE;
    a = rn->actions[0];
    memmove(rn->actions, rn->actions + 1, (size_t)(rn->n_actions - 1) * sizeof(rn->actions[0]));
    rn->n_actions--;
    return a;
}

void v92_rn_init(v92_rn_t *rn, v92_rn_role_t role, int rtd_symbols)
{
    memset(rn, 0, sizeof(*rn));
    rn->role = role;
    rn->rtd = rtd_symbols;
    rn->trn_symbols = role == V92_RN_DIGITAL ? 4002 : TRN2U_MIN;
    rn->silence_symbols = 4008;
    rn->trn_after_e_symbols = TRN2U_AFTER_E_MAX;
    rn->drn = 19;
    rn->state = V92_RN_IDLE;
}

/* Everything one procedure accumulates, cleared for the next. */
static void begin(v92_rn_t *rn, v92_rn_proc_t proc, bool initiator)
{
    long now = rn->now;
    v92_rn_role_t role = rn->role;
    int rtd = rn->rtd, trn = rn->trn_symbols, sil = rn->silence_symbols;
    int trn_e = rn->trn_after_e_symbols;
    uint8_t drn = rn->drn;
    bool rsil = rn->respond_silence;

    memset(rn, 0, sizeof(*rn));
    rn->respond_silence = rsil;
    rn->role = role;
    rn->rtd = rtd;
    rn->trn_symbols = trn;
    rn->silence_symbols = sil;
    rn->trn_after_e_symbols = trn_e;
    rn->drn = drn;
    rn->now = now;
    rn->proc = proc;
    rn->initiator = initiator;
    /* 9.6.x.2.1: B1 within 20 s plus 6 round trips, or retrain. */
    rn->deadline = now + 20L * 8000 + 6L * rtd;
}

static v92_r_t own_r(const v92_rn_t *rn)
{
    if (rn->role == V92_RN_DIGITAL)
        return rn->proc == V92_RN_RENEG ? V92_R_RD : V92_R_RF;
    return rn->proc == V92_RN_RENEG ? V92_R_RU : V92_R_RM;
}

bool v92_rn_initiate(v92_rn_t *rn, v92_rn_proc_t proc, bool silence, bool cleardown)
{
    if (rn->state != V92_RN_IDLE && rn->state != V92_RN_DONE)
        return false;
    /* 9.11 (Amd.1): "The initiating modem shall not request silence". 9.9
     * has no silent period at all (9.9.x.1.2: SUV with bit 32 clear). */
    if (silence && (cleardown || proc == V92_RN_FPE))
        return false;
    begin(rn, proc, true);
    rn->want_silence = silence;
    rn->cleardown = cleardown;
    rn->r_tx = own_r(rn);
    rn->state = V92_RN_SEND_R;
    act(rn, V92_RN_ACT_CLAMP);      /* 9.8.x.1.1 / 9.9.x.1.1: circuit 106 off */
    return true;
}

static void enter_core(v92_rn_t *rn, bool suv_first)
{
    rn->state = V92_RN_CORE;
    rn->suv_first_in_core = suv_first;
    rn->suv_sent = false;
    /* An SUV that arrived before the core began still calls for the CP. */
    if (rn->peer_suv && !rn->cp_sent)
        rn->cp_due = true;
}

static void unit(v92_rn_unit_t *u, v92_rn_tx_kind_t k, int symbols)
{
    memset(u, 0, sizeof(*u));
    u->kind = k;
    u->symbols = symbols;
}

static void suv(v92_rn_t *rn, v92_rn_unit_t *u, bool ack, bool silence)
{
    unit(u, V92_RN_TX_SUV, frame_symbols(rn, V92_RN_TX_SUV));
    u->ack = ack;
    u->silence = silence;
    rn->suv_sent = true;
}

void v92_rn_next(v92_rn_t *rn, v92_rn_unit_t *u)
{
    switch (rn->state) {
    case V92_RN_SEND_R:
        if (rn->r_stage == 0) {
            unit(u, V92_RN_TX_R, R_SYMBOLS);
            u->r = rn->r_tx;
            rn->r_stage = 1;
            return;
        }
        unit(u, V92_RN_TX_RBAR, RBAR_SYMBOLS);
        u->r = rn->r_tx;
        rn->r_stage = 0;
        if (rn->r_tx == V92_R_RT) {
            /* 9.8.1.1.4/5: Rt, bar, SUVd -- then 9.6.1.1.2. */
            enter_core(rn, true);
        } else if (rn->proc == V92_RN_RENEG) {
            rn->state = V92_RN_TRN;
            rn->trn_sent = 0;
        } else {
            /* 9.9.x.1.2 / 9.9.x.2.3 */
            act(rn, V92_RN_ACT_REINIT_SCRAMBLER);
            if (rn->initiator && !rn->peer_r_done)
                rn->state = V92_RN_FPE_WAIT_R;
            else
                enter_core(rn, true);
        }
        return;

    case V92_RN_TRN: {
        int limit = rn->trn_symbols > TRN2_MAX ? TRN2_MAX : rn->trn_symbols;
        /* 9.8.2.1.2: the analogue modem may stop TRN2u after 2400T once
         * SUVd has arrived. */
        bool early = rn->role == V92_RN_ANALOGUE && rn->peer_suv
                     && rn->trn_sent >= TRN2U_MIN;

        if (rn->trn_sent < limit && !early) {
            int n = limit - rn->trn_sent;

            unit(u, V92_RN_TX_TRN, n < TRN_CHUNK ? n : TRN_CHUNK);
            rn->trn_sent += u->symbols;
            return;
        }
        rn->state = V92_RN_SUV1;
        rn->suv_sent = false;
        v92_rn_next(rn, u);
        return;
    }

    case V92_RN_SUV1:
        /* Decision: 9.8.1.1.2 (digital, on receiving SUVu) and 9.8.2.1.3
         * (analogue, having sent SUVu and received SUVd). */
        /* Both roles have sent at least one SUV first: 9.8.1.1.2 has
         * TRN2d "followed by SUVd sequences", and the analogue modem's
         * 9.8.2.1.3 waits for one. */
        if (rn->peer_suv && rn->suv_sent) {
            if (!rn->want_silence && !rn->peer_silence) {
                enter_core(rn, false);
            } else {
                rn->silence_mine = rn->want_silence;
                rn->state = V92_RN_SUV_ACK;
            }
            v92_rn_next(rn, u);
            return;
        }
        suv(rn, u, false, rn->want_silence);
        return;

    case V92_RN_SUV_ACK:
        /* 9.8.1.1.3 / 9.8.2.1.4: SUV with bit 33 until the peer's bit 33 or
         * its E, then E. */
        if (rn->peer_suv_ack || rn->ack_rx) {
            unit(u, V92_RN_TX_E, frame_symbols(rn, V92_RN_TX_E));
            rn->peer_suv = false;
            rn->peer_suv_ack = false;
            rn->ack_rx = false;
            if (rn->role == V92_RN_DIGITAL) {
                rn->state = V92_RN_SILENT;
                rn->silent_sent = 0;
            } else {
                rn->state = V92_RN_TRN_AFTER_E;
                rn->trn_sent = 0;
            }
            return;
        }
        suv(rn, u, true, rn->want_silence);
        return;

    case V92_RN_SILENT:
        /* 9.8.1.1.4: the analogue modem asked -- wait for its SUVu with
         * bit 32 clear.  9.8.1.1.5: we asked -- Rt when our silent period
         * is over, or on another SUVu. */
        if (rn->rt_go || (!rn->peer_silence && rn->silent_sent >= rn->silence_symbols)) {
            rn->r_tx = V92_R_RT;
            rn->r_stage = 0;
            rn->state = V92_RN_SEND_R;
            v92_rn_next(rn, u);
            return;
        }
        unit(u, V92_RN_TX_SILENCE, TRN_CHUNK);
        rn->silent_sent += TRN_CHUNK;
        return;

    case V92_RN_TRN_AFTER_E: {
        /* 9.8.2.1.5: we asked for silence -- TRN2u up to 8004T.  9.8.2.1.6:
         * the digital modem asked -- until its Rt, or 8004T. */
        int limit = rn->trn_after_e_symbols > TRN2U_AFTER_E_MAX
                    ? TRN2U_AFTER_E_MAX : rn->trn_after_e_symbols;
        bool stop = rn->trn_sent >= limit || (rn->peer_silence && rn->rt_seen);

        if (!stop) {
            int n = limit - rn->trn_sent;

            unit(u, V92_RN_TX_TRN, n < TRN_CHUNK ? n : TRN_CHUNK);
            rn->trn_sent += u->symbols;
            return;
        }
        enter_core(rn, true);
        v92_rn_next(rn, u);
        return;
    }

    case V92_RN_FPE_WAIT_R:
        suv(rn, u, false, false);
        return;

    case V92_RN_CORE:
        /* 9.11: a cleardown ends in going on-hook, not in E and B1. */
        if (rn->ack_frame_sent && rn->ack_rx && !rn->cleardown && !rn->peer_cleardown
            && !rn->suv_first_in_core) {
            rn->state = V92_RN_SEND_E;
            v92_rn_next(rn, u);
            return;
        }
        if ((rn->cp_due || rn->cp_repeat) && !rn->suv_first_in_core) {
            unit(u, V92_RN_TX_CP, frame_symbols(rn, V92_RN_TX_CP));
            u->ack = rn->ack_tx;
            u->drn = rn->cleardown ? 0 : rn->drn;
            rn->cp_due = false;
            rn->cp_sent = true;
            rn->ack_frame_sent |= u->ack;
            rn->cp_end = rn->now + u->symbols;
            if (rn->ack_tx) {
                rn->cp_with_ack_sent = true;
                rn->cp_with_ack_end = rn->cp_end;
            }
            /* 9.11 (Amd.1): the initiator waits for an acknowledged CP or
             * 100 ms plus a round trip after its CP; the responder 100 ms
             * plus half a round trip after its acknowledged CP. */
            if (rn->cleardown && rn->disconnect_at == 0)
                rn->disconnect_at = rn->cp_end + MS100 + rn->rtd;
            if (rn->peer_cleardown && rn->ack_tx)
                rn->disconnect_at = rn->cp_end + MS100 + rn->rtd / 2;
            return;
        }
        suv(rn, u, rn->ack_tx, false);
        rn->ack_frame_sent |= u->ack;
        rn->suv_first_in_core = false;
        return;

    case V92_RN_SEND_E:
        if (!rn->e_sent) {
            unit(u, V92_RN_TX_E, frame_symbols(rn, V92_RN_TX_E));
            rn->e_sent = true;
            return;
        }
        /* 9.6.2.1.5: for a fast parameter exchange FB1u precedes B1u. */
        if (rn->role == V92_RN_ANALOGUE && rn->proc == V92_RN_FPE && !rn->fb1_sent) {
            unit(u, V92_RN_TX_FB1, frame_symbols(rn, V92_RN_TX_FB1));
            rn->fb1_sent = true;
            return;
        }
        unit(u, V92_RN_TX_B1, frame_symbols(rn, V92_RN_TX_B1));
        rn->b1_sent = true;
        rn->tx_data = true;
        act(rn, V92_RN_ACT_TX_DATA);
        rn->state = rn->rx_data ? V92_RN_DONE : V92_RN_WAIT_B1;
        return;

    default:
        unit(u, V92_RN_TX_DATA, 0);
        return;
    }
}

/* 9.6.x.1.3: no acknowledgement in anything received up to and including
 * the whole frame that ends after 100 ms plus a round trip from the end of
 * our CP -- repeat the CP. */
static void check_cp_repeat(v92_rn_t *rn)
{
    if (rn->state == V92_RN_CORE && rn->cp_sent && !rn->ack_rx && !rn->cp_repeat
        && rn->now >= rn->cp_end + MS100 + rn->rtd)
        rn->cp_repeat = true;
}

void v92_rn_rx(v92_rn_t *rn, const v92_rn_event_t *ev)
{
    switch (ev->kind) {
    case V92_RN_EV_R: {
        bool d = rn->role == V92_RN_DIGITAL;
        v92_rn_proc_t proc;

        /* Responding: 9.8.1.2.1 / 9.8.2.2.1 / 9.9.1.2.1 / 9.9.2.2.1.  An FPE
         * initiator that meets the peer's renegotiation follows it
         * (9.9.1.1.2, 9.9.2.1.2). */
        if (ev->r == (d ? V92_R_RU : V92_R_RD))
            proc = V92_RN_RENEG;
        else if (ev->r == (d ? V92_R_RM : V92_R_RF))
            proc = V92_RN_FPE;
        else
            break;
        if (rn->state == V92_RN_IDLE || rn->state == V92_RN_DONE
            || (proc == V92_RN_RENEG && rn->proc == V92_RN_FPE && rn->initiator
                && (rn->state == V92_RN_FPE_WAIT_R || rn->state == V92_RN_SEND_R))) {
            bool keep_cleardown = rn->cleardown;

            begin(rn, proc, false);
            rn->cleardown = keep_cleardown;
            rn->want_silence = proc == V92_RN_RENEG && rn->respond_silence;
            rn->state = V92_RN_WAIT_RBAR;
            rn->r_wait = ev->r;
            act(rn, V92_RN_ACT_CLAMP);
        }
        break;
    }

    case V92_RN_EV_RBAR:
        if (rn->state == V92_RN_WAIT_RBAR && ev->r == rn->r_wait) {
            rn->r_tx = own_r(rn);
            rn->r_stage = 0;
            rn->state = V92_RN_SEND_R;
        } else if (rn->proc == V92_RN_FPE && rn->initiator
                   && ev->r == (rn->role == V92_RN_DIGITAL ? V92_R_RM : V92_R_RF)) {
            /* 9.9.x.1.2: the peer's R pair -- now receive its SUV. */
            rn->peer_r_done = true;
            if (rn->state == V92_RN_FPE_WAIT_R)
                enter_core(rn, false);
        } else if (ev->r == V92_R_RT && rn->role == V92_RN_ANALOGUE) {
            rn->rt_seen = true;
        }
        break;

    case V92_RN_EV_SUV:
        check_cp_repeat(rn);
        /* After E, frames from the first exchange (bit 33 set) are still
         * in flight for a round trip; the ones that matter now are the
         * second phase's, which carry bit 33 clear. */
        if ((rn->state == V92_RN_SILENT || rn->state == V92_RN_TRN_AFTER_E) && ev->ack)
            break;
        if (rn->state == V92_RN_SILENT) {
            /* 9.8.1.1.4: with the analogue modem's silence, only an SUVu
             * with bit 32 clear ends it; with ours, any SUVu may. */
            if (!rn->peer_silence || !ev->silence) {
                rn->rt_go = true;
                rn->peer_suv = true;
            }
            break;
        }
        if (rn->state == V92_RN_SUV1 || rn->state == V92_RN_TRN)
            rn->peer_silence |= ev->silence;
        rn->peer_suv = true;
        if (ev->ack) {
            rn->peer_suv_ack = true;
            if (rn->state == V92_RN_CORE)
                rn->ack_rx = true;
        }
        if (rn->state == V92_RN_CORE && !rn->cp_sent)
            rn->cp_due = true;
        break;

    case V92_RN_EV_CP:
        check_cp_repeat(rn);
        if (rn->state != V92_RN_CORE && rn->state != V92_RN_FPE_WAIT_R)
            break;
        if (rn->state == V92_RN_FPE_WAIT_R)
            enter_core(rn, false);
        rn->ack_tx = true;          /* 9.6.x.1.2 */
        rn->peer_drn = ev->drn;
        if (ev->ack)
            rn->ack_rx = true;
        if (!rn->cp_sent)
            rn->cp_due = true;
        if (ev->drn == 0 && !rn->peer_cleardown) {
            /* 9.11: answer with an acknowledged CP, then go on-hook. */
            rn->peer_cleardown = true;
            if (!rn->cp_with_ack_sent)
                rn->cp_due = true;
        }
        /* The initiator of a cleardown goes on-hook on its acknowledgement. */
        if (rn->cleardown && ev->ack && rn->cp_sent)
            rn->disconnect_at = rn->now;
        break;

    case V92_RN_EV_E:
        /* Ed / E2u: an acknowledgement in its own right (9.6.x.1.4,
         * 9.8.x.1.3/4). */
        rn->ack_rx = true;
        break;

    case V92_RN_EV_FB1:
        break;

    case V92_RN_EV_B1:
        if (rn->state == V92_RN_SEND_E || rn->state == V92_RN_WAIT_B1
            || rn->state == V92_RN_CORE) {
            rn->rx_data = true;
            act(rn, V92_RN_ACT_RX_DATA);
            if (rn->tx_data)
                rn->state = V92_RN_DONE;
        }
        break;
    }
}

void v92_rn_tick(v92_rn_t *rn, int symbols)
{
    rn->now += symbols;
    if (rn->disconnect_at && rn->now >= rn->disconnect_at
        && rn->state != V92_RN_DONE) {
        rn->state = V92_RN_DONE;
        act(rn, V92_RN_ACT_DISCONNECT);
        return;
    }
    if (rn->state != V92_RN_IDLE && rn->state != V92_RN_DONE
        && rn->now >= rn->deadline) {
        rn->state = V92_RN_DONE;
        act(rn, V92_RN_ACT_RETRAIN);
    }
}
