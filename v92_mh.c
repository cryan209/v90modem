/*
 * v92_mh.c — V.92 modem-on-hold: MH codec (8.9.2, Table 32 as replaced by
 * Amd.1 item 7, Table 33), framer, and the 9.10 transaction controller
 * (9.10.1 as replaced by Cor.1 item 2, 9.10.2, Table 34).
 */
#include "v92_mh.h"

#include <spandsp.h>
#include <spandsp/crc.h>

#include <string.h>

/* 4:11, "01110010, where the left-most bit is first in time" -- the same
 * frame sync as the V.34/V.90 INFO sequences whose modulation 8.9.2 uses. */
static const uint8_t mh_sync[8] = {0, 1, 1, 1, 0, 0, 1, 0};

static void set_bits(uint8_t *b, int first, int n, uint32_t v)
{
    for (int i = 0; i < n; i++)
        b[first + i] = (uint8_t)((v >> i) & 1U);
}

static uint32_t get_bits(const uint8_t *b, int first, int n)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++)
        v |= (uint32_t)(b[first + i] & 1U) << i;
    return v;
}

/* V.34 10.1.2.3.2 over the information bits 12:19, preset to ones; the
 * remainder goes on the wire in the order SpanDSP's INFO encoders use
 * (v34tx.c: crc_bit_block() then bitstream_put() LSB first). */
static uint16_t mh_crc(const uint8_t *b)
{
    uint16_t crc = 0xFFFF;
    for (int i = 12; i <= 19; i++)
        crc = crc_itu16_bits(b[i] & 1U, 1, crc);
    return crc;
}

bool v92_mh_signal_valid(v92_mh_signal_t s)
{
    switch (s) {
    case V92_MH_REQ: case V92_MH_ACK: case V92_MH_NACK:
    case V92_MH_CLRD: case V92_MH_CDA: case V92_MH_FRR:
        return true;
    default:
        return false;
    }
}

const char *v92_mh_signal_name(v92_mh_signal_t s)
{
    switch (s) {
    case V92_MH_REQ:  return "MHreq";
    case V92_MH_ACK:  return "MHack";
    case V92_MH_NACK: return "MHnack";
    case V92_MH_CLRD: return "MHclrd";
    case V92_MH_CDA:  return "MHcda";
    case V92_MH_FRR:  return "MHfrr";
    default:          return "MH?";
    }
}

bool v92_mh_is_initiating(v92_mh_signal_t s)
{
    /* 9.10.1.1: REQ, CLRD and FRR initiate; NACK initiates "a second
     * transaction in response to MHreq". */
    return s == V92_MH_REQ || s == V92_MH_CLRD || s == V92_MH_FRR || s == V92_MH_NACK;
}

bool v92_mh_is_response_to(v92_mh_signal_t init, v92_mh_signal_t resp)
{
    switch (init) {                   /* Table 34 */
    case V92_MH_REQ:  return resp == V92_MH_ACK || resp == V92_MH_NACK;
    case V92_MH_NACK: return resp == V92_MH_CDA || resp == V92_MH_FRR;
    case V92_MH_CLRD: return resp == V92_MH_CDA;
    default:          return false;   /* MHfrr -> ANSam */
    }
}

int v92_mh_t1_seconds(uint8_t code)
{
    static const int t[16] = {
        -1, 10, 20, 30, 40, 60, 120, 180, 240, 360, 480, 720, 960, 0, -1, -1
    };
    return t[code & 0xF];
}

static bool info_defined(v92_mh_signal_t s, uint8_t info)
{
    switch (s) {
    case V92_MH_ACK:
        return v92_mh_t1_seconds(info) >= 0;
    case V92_MH_CLRD:
        return info == V92_MH_CLRD_INCOMING || info == V92_MH_CLRD_OUTGOING
            || info == V92_MH_CLRD_OTHER;
    case V92_MH_NACK:
        /* Amd.1 item 7 gives MHnack its own reasons.  The 11/2000 text had
         * it repeat the signal bits, which is 0111 -- the same as
         * V92_MH_NACK_LATER, so an old peer reads as "ask again later". */
        return info == V92_MH_NACK_NEVER || info == V92_MH_NACK_LATER;
    default:
        return info == (uint8_t)s;    /* repeat the signal indication bits */
    }
}

bool v92_mh_encode(const v92_mh_frame_t *f, uint8_t bits[V92_MH_BITS])
{
    uint8_t info;

    if (!v92_mh_signal_valid(f->signal))
        return false;
    info = (f->signal == V92_MH_REQ || f->signal == V92_MH_CDA || f->signal == V92_MH_FRR)
           ? (uint8_t)f->signal : (uint8_t)(f->info & 0xF);
    set_bits(bits, 0, 4, 0xF);
    memcpy(bits + 4, mh_sync, 8);
    set_bits(bits, 12, 4, (uint32_t)f->signal);
    set_bits(bits, 16, 4, info);
    set_bits(bits, 20, 16, mh_crc(bits));
    set_bits(bits, 36, 4, 0xF);
    return true;
}

bool v92_mh_decode(const uint8_t bits[V92_MH_BITS], v92_mh_decode_t *out)
{
    v92_mh_decode_t d;

    memset(&d, 0, sizeof(d));
    d.fill_ok = get_bits(bits, 0, 4) == 0xF && get_bits(bits, 36, 4) == 0xF;
    d.sync_ok = memcmp(bits + 4, mh_sync, 8) == 0;
    d.crc_field = (uint16_t)get_bits(bits, 20, 16);
    d.crc_expected = mh_crc(bits);
    d.crc_ok = d.crc_field == d.crc_expected;
    d.frame.signal = (v92_mh_signal_t)get_bits(bits, 12, 4);
    d.frame.info = (uint8_t)get_bits(bits, 16, 4);
    d.signal_known = v92_mh_signal_valid(d.frame.signal);
    d.info_defined = d.signal_known && info_defined(d.frame.signal, d.frame.info);
    if (out)
        *out = d;
    return d.fill_ok && d.sync_ok && d.crc_ok && d.signal_known;
}

/* ---- framer ---- */

void v92_mh_rx_init(v92_mh_rx_t *r)
{
    memset(r, 0, sizeof(*r));
}

bool v92_mh_rx_put_bit(v92_mh_rx_t *r, int bit, v92_mh_frame_t *out)
{
    uint8_t win[V92_MH_BITS];
    v92_mh_decode_t d;

    r->hist[r->pos] = (uint8_t)(bit & 1);
    r->pos = (r->pos + 1) % V92_MH_BITS;
    if (r->filled < V92_MH_BITS)
        r->filled++;
    if (r->filled < V92_MH_BITS)
        return false;
    for (int i = 0; i < V92_MH_BITS; i++)
        win[i] = r->hist[(r->pos + i) % V92_MH_BITS];
    /* Cheap gate first: fill + sync are 12 fixed bits. */
    if (get_bits(win, 0, 4) != 0xF || memcmp(win + 4, mh_sync, 8) != 0)
        return false;
    v92_mh_decode(win, &d);
    if (!d.crc_ok) {
        r->crc_rejects++;
        return false;
    }
    if (!d.fill_ok)
        return false;
    if (!d.signal_known) {
        r->unknown++;                 /* Table 32 note 1: ignore */
        return false;
    }
    r->frames++;
    if (out)
        *out = d.frame;
    return true;
}

/* ---- controller ---- */

const char *v92_mh_state_name(v92_mh_state_t s)
{
    static const char *n[] = {
        "IDLE", "INIT_SILENCE", "INIT_RT", "INIT_SEND", "INIT_AWAIT_ANSAM",
        "INIT_HOLD", "RESP_RT", "RESP_SEND", "RESP_ANSAM_GAP", "ON_HOLD", "DONE"
    };
    return (unsigned)s < sizeof(n)/sizeof(n[0]) ? n[s] : "?";
}

static void push_action(v92_mh_ctrl_t *c, v92_mh_action_t a)
{
    if (a != V92_MH_ACT_NONE && c->n_actions < (int)(sizeof(c->actions)/sizeof(c->actions[0])))
        c->actions[c->n_actions++] = a;
}

v92_mh_action_t v92_mh_ctrl_take_action(v92_mh_ctrl_t *c)
{
    v92_mh_action_t a;

    if (c->n_actions == 0)
        return V92_MH_ACT_NONE;
    a = c->actions[0];
    memmove(c->actions, c->actions + 1, (size_t)(c->n_actions - 1) * sizeof(c->actions[0]));
    c->n_actions--;
    return a;
}

void v92_mh_ctrl_init(v92_mh_ctrl_t *c, int round_trip_ms)
{
    memset(c, 0, sizeof(*c));
    c->round_trip_ms = round_trip_ms;
    c->grant = true;
    c->t1_code = 0x3;                 /* 30 s */
    c->nack_reason = V92_MH_NACK_LATER;
    c->after_nack_reconnect = true;
    c->skip_rt_if_peer_rt = true;
    c->state = V92_MH_ST_IDLE;
    c->tx = V92_MH_TX_DATA;
    v92_mh_rx_init(&c->rx);
}

static void enter(v92_mh_ctrl_t *c, v92_mh_state_t s, v92_mh_tx_t tx)
{
    c->state = s;
    c->tx = tx;
    c->ms = 0;
}

/* Start sending `f`.  A sequence already on the line is finished first
 * (9.10.1, "Each transmitted sequence shall be completed before
 * transmitting other signals"); a new MH frame is swapped in at the
 * boundary, which also keeps the stream back to back. */
static void send_mh(v92_mh_ctrl_t *c, v92_mh_state_t s, v92_mh_signal_t sig, uint8_t info)
{
    c->tx_frame.signal = sig;
    c->tx_frame.info = info;
    if (c->tx != V92_MH_TX_MH) {
        v92_mh_encode(&c->tx_frame, c->tx_bits);
        c->tx_bit = 0;
    }
    c->state = s;
    c->tx = V92_MH_TX_MH;
    c->ms = 0;
    c->deferred = false;
}

/* Leave MH for something else once the open sequence completes. */
static void after_sequence(v92_mh_ctrl_t *c, v92_mh_state_t s, v92_mh_tx_t tx, v92_mh_action_t a)
{
    if (c->tx == V92_MH_TX_MH && c->tx_bit != 0) {
        c->deferred = true;
        c->deferred_state = s;
        c->deferred_tx = tx;
        c->deferred_action = a;
        return;
    }
    enter(c, s, tx);
    push_action(c, a);
}

int v92_mh_ctrl_tx_bit(v92_mh_ctrl_t *c)
{
    int b;

    if (c->tx != V92_MH_TX_MH)
        return 1;
    if (c->tx_bit == 0)
        v92_mh_encode(&c->tx_frame, c->tx_bits);   /* pick up a swapped frame */
    b = c->tx_bits[c->tx_bit];
    if (++c->tx_bit >= V92_MH_BITS) {
        c->tx_bit = 0;
        if (c->tx_frame.signal == V92_MH_ACK && !c->first_mhack_sent) {
            c->first_mhack_sent = true;              /* T1 runs from here (9.10.2.1) */
            c->on_hold_ms = 0;
        }
        if (c->deferred) {
            c->deferred = false;
            enter(c, c->deferred_state, c->deferred_tx);
            push_action(c, c->deferred_action);
        }
    }
    return b;
}

bool v92_mh_ctrl_initiate(v92_mh_ctrl_t *c, v92_mh_signal_t s, uint8_t info)
{
    if (c->state != V92_MH_ST_IDLE)
        return false;
    if (s != V92_MH_REQ && s != V92_MH_CLRD && s != V92_MH_FRR)
        return false;
    if (s == V92_MH_REQ && c->no_outgoing_requests)
        return false;
    c->initiator = true;
    c->initiated = s;
    c->tx_frame.signal = s;
    c->tx_frame.info = info;
    c->total_ms = 0;
    c->peer_rt_during_silence = false;
    /* Cor.1 9.10.1: data -> 70 +/- 5 ms silence -> Tone RT -> MH. */
    enter(c, V92_MH_ST_INIT_SILENCE, V92_MH_TX_SILENCE);
    push_action(c, V92_MH_ACT_SUSPEND_LINK);
    return true;
}

/* The responder's RT must have been up 50 ms before an MH follows it
 * (9.10.1); otherwise there is no lower bound. */
static bool rt_long_enough(const v92_mh_ctrl_t *c)
{
    return c->tx != V92_MH_TX_RT || c->ms >= 50;
}

static void respond(v92_mh_ctrl_t *c, v92_mh_signal_t init)
{
    c->peer_initiated = init;
    c->since_init_seen_ms = 0;
    switch (init) {
    case V92_MH_REQ:
        c->last_response = c->grant ? V92_MH_ACK : V92_MH_NACK;
        send_mh(c, V92_MH_ST_RESP_SEND, c->last_response,
                c->grant ? c->t1_code : c->nack_reason);
        if (c->grant)
            push_action(c, V92_MH_ACT_ON_HOLD);    /* "shall enter an on-hold state" */
        break;
    case V92_MH_CLRD:
        c->last_response = V92_MH_CDA;
        send_mh(c, V92_MH_ST_RESP_SEND, V92_MH_CDA, 0);
        break;
    case V92_MH_FRR:
        /* 9.10.2.3: silence for up to 80 ms, then ANSam. */
        c->last_response = V92_MH_NONE;
        after_sequence(c, V92_MH_ST_RESP_ANSAM_GAP, V92_MH_TX_SILENCE, V92_MH_ACT_NONE);
        break;
    default:
        break;
    }
}

static void handle_frame(v92_mh_ctrl_t *c, const v92_mh_frame_t *f)
{
    switch (c->state) {
    case V92_MH_ST_IDLE:
    case V92_MH_ST_RESP_RT:
        if (f->signal != V92_MH_REQ && f->signal != V92_MH_CLRD && f->signal != V92_MH_FRR)
            return;
        if (c->state == V92_MH_ST_IDLE)
            push_action(c, V92_MH_ACT_SUSPEND_LINK);
        c->initiator = false;
        if (!rt_long_enough(c)) {
            c->peer_initiated = f->signal;          /* answered in tick() */
            return;
        }
        respond(c, f->signal);
        return;

    case V92_MH_ST_RESP_SEND:
        if (f->signal == c->peer_initiated) {
            c->since_init_seen_ms = 0;
            return;
        }
        if (c->last_response == V92_MH_NACK) {
            if (f->signal == V92_MH_CDA)            /* 9.10.2.1 */
                after_sequence(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE, V92_MH_ACT_DISCONNECT);
            else if (f->signal == V92_MH_FRR)
                after_sequence(c, V92_MH_ST_RESP_ANSAM_GAP, V92_MH_TX_SILENCE, V92_MH_ACT_NONE);
        }
        return;

    case V92_MH_ST_INIT_RT:
    case V92_MH_ST_INIT_SEND:
        if (!v92_mh_is_response_to(c->initiated, f->signal))
            return;
        c->since_response_ms = 0;
        if (c->initiated == V92_MH_CLRD) {          /* 9.10.2.2 */
            after_sequence(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE, V92_MH_ACT_DISCONNECT);
        } else if (f->signal == V92_MH_ACK) {
            /* 9.10.2.1: "may continue sending MHreq for a maximum of 30 s
             * or send Tone RT or silence".  Tone RT now, which is what
             * releases the responder into ANSam; it follows an MH so its
             * floor is 20 ms. */
            c->t1_ms = v92_mh_t1_seconds(f->info) > 0 ? v92_mh_t1_seconds(f->info) * 1000 : -1;
            after_sequence(c, V92_MH_ST_INIT_HOLD, V92_MH_TX_RT, V92_MH_ACT_ON_HOLD);
        } else if (f->signal == V92_MH_NACK) {
            if (f->info == V92_MH_NACK_NEVER)
                c->no_outgoing_requests = true;
            /* Table 34: MHnack is itself initiating; answer within 10 s. */
            c->peer_initiated = V92_MH_NACK;
            c->since_init_seen_ms = 0;
            if (c->after_nack_reconnect) {
                c->initiated = V92_MH_FRR;
                send_mh(c, V92_MH_ST_INIT_AWAIT_ANSAM, V92_MH_FRR, 0);
            } else {
                c->last_response = V92_MH_CDA;
                send_mh(c, V92_MH_ST_RESP_SEND, V92_MH_CDA, 0);
            }
        }
        return;

    default:
        return;
    }
}

void v92_mh_ctrl_rx_frame(v92_mh_ctrl_t *c, const v92_mh_frame_t *f)
{
    handle_frame(c, f);
}

void v92_mh_ctrl_rx_bit(v92_mh_ctrl_t *c, int bit)
{
    v92_mh_frame_t f;

    if (v92_mh_rx_put_bit(&c->rx, bit, &f))
        handle_frame(c, &f);
}

void v92_mh_ctrl_tick(v92_mh_ctrl_t *c, int ms, const v92_mh_line_t *l)
{
    static const v92_mh_line_t quiet = {0};
    int timeout = 2000 + c->round_trip_ms;   /* 9.10.1.1 */

    if (!l)
        l = &quiet;
    c->ms += ms;
    c->total_ms += ms;
    c->peer_rt_ms = l->rt ? c->peer_rt_ms + ms : 0;
    c->silence_ms = l->silence ? c->silence_ms + ms : 0;
    c->ansam_ms = l->ansam ? c->ansam_ms + ms : 0;
    c->since_init_seen_ms += ms;
    c->since_response_ms += ms;
    if (c->first_mhack_sent)
        c->on_hold_ms += ms;
    if (c->deferred)
        return;                         /* finishing a sequence; tx_bit() moves on */

    switch (c->state) {
    case V92_MH_ST_IDLE:
        if (l->rt) {
            /* Could be a retrain or an MH transaction: answer with our RT
             * and listen for both (9.10.1.1, Cor.1 9.7.1.2 NOTE). */
            c->initiator = false;
            c->total_ms = 0;
            enter(c, V92_MH_ST_RESP_RT, V92_MH_TX_RT);
            push_action(c, V92_MH_ACT_SUSPEND_LINK);
        }
        break;

    case V92_MH_ST_INIT_SILENCE:
        if (l->rt)
            c->peer_rt_during_silence = true;
        if (c->ms >= 70) {
            if (c->skip_rt_if_peer_rt && c->peer_rt_during_silence)
                send_mh(c, V92_MH_ST_INIT_SEND, c->initiated, c->tx_frame.info);
            else
                enter(c, V92_MH_ST_INIT_RT, V92_MH_TX_RT);
        }
        break;

    case V92_MH_ST_INIT_RT:
        /* At least 50 ms of RT (9.10.1), and 9.10.1.1 lets the initiating
         * sequence go only once the far end's RT is received. */
        if (c->ms >= 50 && (c->peer_rt_ms > 0 || c->peer_rt_during_silence)) {
            if (c->initiated == V92_MH_FRR)
                send_mh(c, V92_MH_ST_INIT_AWAIT_ANSAM, V92_MH_FRR, 0);
            else
                send_mh(c, V92_MH_ST_INIT_SEND, c->initiated, c->tx_frame.info);
        } else if (c->ms >= timeout) {
            enter(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE);
            push_action(c, V92_MH_ACT_RETRAIN);
        }
        break;

    case V92_MH_ST_INIT_SEND:
        if (c->ms >= timeout)
            after_sequence(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE, V92_MH_ACT_RETRAIN);
        break;

    case V92_MH_ST_INIT_AWAIT_ANSAM:
        /* 9.10.2.3: ANSam detected for 1 s, then Phase 1. */
        if (c->ansam_ms >= 1000)
            after_sequence(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE, V92_MH_ACT_PHASE1_CALL);
        else if (c->ansam_ms == 0 && c->ms >= timeout)
            after_sequence(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE, V92_MH_ACT_RETRAIN);
        break;

    case V92_MH_ST_INIT_HOLD:
        break;                          /* the engine decides when to come back */

    case V92_MH_ST_RESP_RT:
        if (l->reversal && c->peer_initiated == V92_MH_NONE) {
            enter(c, V92_MH_ST_DONE, V92_MH_TX_RT);
            push_action(c, V92_MH_ACT_RETRAIN);
        } else if (c->peer_initiated != V92_MH_NONE && rt_long_enough(c)) {
            respond(c, c->peer_initiated);
        } else if (c->ms >= timeout) {
            enter(c, V92_MH_ST_DONE, V92_MH_TX_RT);
            push_action(c, V92_MH_ACT_RETRAIN);
        }
        break;

    case V92_MH_ST_RESP_SEND:
        if (c->last_response == V92_MH_ACK) {
            /* 9.10.2.1: RT for 100 ms or silence for 2 s releases MHack;
             * ANSam follows within 80 ms. */
            if (c->peer_rt_ms >= 100 || c->silence_ms >= 2000)
                after_sequence(c, V92_MH_ST_ON_HOLD, V92_MH_TX_ANSAM, V92_MH_ACT_NONE);
        } else if (c->last_response == V92_MH_NACK) {
            if (c->since_init_seen_ms >= 10000 + c->round_trip_ms)
                after_sequence(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE, V92_MH_ACT_DISCONNECT);
        } else if (c->last_response == V92_MH_CDA) {
            /* 9.10.1.2 / 9.10.2.2 */
            if (l->ansam || c->silence_ms > 0 || c->peer_rt_ms > 0 || c->since_init_seen_ms >= 200)
                after_sequence(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE, V92_MH_ACT_DISCONNECT);
        }
        break;

    case V92_MH_ST_RESP_ANSAM_GAP:
        if (c->ms >= 40) {              /* inside 9.10.2.3's 80 ms */
            enter(c, V92_MH_ST_DONE, V92_MH_TX_ANSAM);
            push_action(c, V92_MH_ACT_PHASE1_ANSWER);
        }
        break;

    case V92_MH_ST_ON_HOLD:
        if (l->phase1) {
            if (l->qc_cleardown) {
                enter(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE);
                push_action(c, V92_MH_ACT_DISCONNECT);
            } else {
                c->null_cm = l->cm_null;
                enter(c, V92_MH_ST_DONE, V92_MH_TX_ANSAM);
                push_action(c, V92_MH_ACT_PHASE1_ANSWER);
            }
        } else if (c->t1_ms > 0 && c->on_hold_ms >= c->t1_ms) {
            enter(c, V92_MH_ST_DONE, V92_MH_TX_SILENCE);
            push_action(c, V92_MH_ACT_DISCONNECT);
        }
        break;

    default:
        break;
    }

    /* The responder learns T1 from its own MHack. */
    if (c->state == V92_MH_ST_RESP_SEND && c->last_response == V92_MH_ACK)
        c->t1_ms = v92_mh_t1_seconds(c->t1_code) > 0 ? v92_mh_t1_seconds(c->t1_code) * 1000 : -1;
}
