/*
 * v8bis_modem.c -- V.8bis at sample level.  See the header for the clauses.
 *
 * Transmit is a queue of segments (silence, a tone signal, a run of V.21 bits)
 * that the FSM's actions are turned into; receive runs the tone detector and
 * both V.21 demodulators all the time and hands the FSM whatever is allowed to
 * count (the channel rule in the header, an armed preamble for frames).
 */
#include "v8bis_modem.h"

#include <spandsp.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_SEGS 8
#define EVENT_QUEUE 32
#define MARK_PREAMBLE_BITS V8BIS_PREAMBLE_BITS      /* 30 bits = 100 ms at 300 bit/s */
/* SpanDSP's FSK modulator emits one bit period of the idle (mark) tone before it fetches the first
 * bit, so a run of bits that starts a modulation carries one mark bit more than it was given. */
#define MODULATOR_LEAD_BITS 1
#define PREAMBLE_ARM_BITS 12                         /* marks seen before a flag for a frame to count */
#define TONE_MUTE_AFTER 800                          /* 100 ms of rx tone deafness after our own tone */
#define SILENT_BAND_POWER 400.0                      /* amplitude 20 in one tone: well under any signal */
#define BITS_MAX (2 * V8BIS_MAX_FRAME_BITS + 2 * MARK_PREAMBLE_BITS)

typedef enum { SEG_SILENCE, SEG_TONE, SEG_BITS } seg_kind_t;

typedef struct {
    seg_kind_t kind;
    unsigned n_samples;                 /* SILENCE */
    v8bis_signal_t sig;                 /* TONE */
    bool set, seg1_only, watch;
    double level;
    bool high;                          /* BITS: V.21(H) */
    unsigned nbits;
    uint8_t bits[BITS_MAX];
} seg_t;

struct v8bis_modem_s;

typedef struct {
    struct v8bis_modem_s *m;
    int ch;                             /* 0 = V.21(L), 1 = V.21(H) */
    fsk_rx_state_t *rx;
    v8bis_frame_rx_t fr;
    unsigned ones;
    bool carrier, armed, tainted;
    bool dominant;                      /* this channel's band holds most of the energy right now */
} chan_t;

struct v8bis_modem_s {
    v8bis_modem_cfg_t cfg;
    v8bis_fsm_t fsm;
    uint64_t now;                       /* transmit samples: the clock */
    uint64_t rx_now;

    /* transmit */
    seg_t segs[MAX_SEGS];
    unsigned sh, sn;
    seg_t cur;
    bool cur_valid;
    unsigned cur_pos;
    v8bis_tone_tx_t tone;
    fsk_tx_state_t *ftx[2];
    unsigned bit_idx;
    bool bits_over;
    unsigned blk_off;                   /* position in the block being generated, for exact timestamps */
    uint64_t tone_mute_until;
    bool tone_active;
    unsigned idle_acc;

    /* receive */
    v8bis_tone_rx_t trx;
    chan_t ch[2];
    int16_t rxbuf[V8BIS_RX_BLOCK];
    unsigned rxfill;
    double band_peak;                   /* recent strongest block, for judging the quiet ones */
    uint64_t rx_pos;                    /* samples through the last processed block */
    bool pend_valid;                    /* a detected signal waits for its own end before the FSM sees it */
    v8bis_signal_t pend_sig;
    bool pend_set;
    uint64_t pend_end;
    bool last_rx_high;
    bool rx_active;                     /* anything heard since the transaction began */

    /* retransmission of MRe/CRe (10.2.2) */
    bool have_how;
    v8bis_init_t how;
    unsigned retries_left;
    bool first_signal;
    bool watching;
    bool watch_armed;                   /* the tone has finished */
    uint64_t watch_from;

    v8bis_modem_event_t ev[EVENT_QUEUE];
    unsigned evh, evn;
};

void v8bis_modem_cfg_default(v8bis_modem_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    v8bis_fsm_cfg_default(&cfg->fsm);
    cfg->level_dbm0 = -13.0;
    cfg->initial_silence_ms = 400;
    cfg->retransmit_ms = 3000;
    cfg->retries = 2;
    cfg->es_gap_ms = 1500;
    cfg->open_flags = 2;
    cfg->close_flags = 1;
}

static void post(v8bis_modem_t *m, const v8bis_modem_event_t *e)
{
    if (m->evn == EVENT_QUEUE) {
        m->evh = (m->evh + 1) % EVENT_QUEUE;
        m->evn--;
    }
    m->ev[(m->evh + m->evn) % EVENT_QUEUE] = *e;
    m->ev[(m->evh + m->evn) % EVENT_QUEUE].at_sample = m->now;
    m->evn++;
}

bool v8bis_modem_event(v8bis_modem_t *m, v8bis_modem_event_t *ev)
{
    if (!m->evn)
        return false;
    *ev = m->ev[m->evh];
    m->evh = (m->evh + 1) % EVENT_QUEUE;
    m->evn--;
    return true;
}

/* ---- transmit queue ---------------------------------------------------- */

static seg_t *seg_new(v8bis_modem_t *m)
{
    seg_t *s;

    if (m->sn == MAX_SEGS)
        return NULL;
    s = &m->segs[(m->sh + m->sn) % MAX_SEGS];
    memset(s, 0, offsetof(seg_t, bits));
    m->sn++;
    return s;
}

static unsigned ms_to_samples(unsigned ms)
{
    return ms * 8;
}

static void q_silence(v8bis_modem_t *m, unsigned ms)
{
    seg_t *s = seg_new(m);

    if (s) {
        s->kind = SEG_SILENCE;
        s->n_samples = ms_to_samples(ms);
    }
}

static void q_tone(v8bis_modem_t *m, v8bis_signal_t sig, bool set, bool seg1_only, bool watch)
{
    seg_t *s = seg_new(m);

    if (s) {
        s->kind = SEG_TONE;
        s->sig = sig;
        s->set = set;
        s->seg1_only = seg1_only;
        s->watch = watch;
        s->level = v8bis_signal_default_level_dbm0(sig, m->cfg.level_dbm0);
    }
}

static seg_t *q_bits(v8bis_modem_t *m, bool high)
{
    seg_t *s = seg_new(m);

    if (s) {
        s->kind = SEG_BITS;
        s->high = high;
    }
    return s;
}

static int tx_get_bit(void *user)
{
    v8bis_modem_t *m = user;

    if (m->bit_idx < m->cur.nbits)
        return m->cur.bits[m->bit_idx++];
    m->bits_over = true;            /* everything has gone out; this is the first bit past it */
    return 1;
}

static bool next_segment(v8bis_modem_t *m)
{
    if (!m->sn) {
        m->cur_valid = false;
        return false;
    }
    m->cur = m->segs[m->sh];
    m->sh = (m->sh + 1) % MAX_SEGS;
    m->sn--;
    m->cur_valid = true;
    m->cur_pos = 0;
    switch (m->cur.kind) {
    case SEG_TONE:
        v8bis_tone_tx_start(&m->tone, m->cur.sig, m->cur.set, m->cur.level, 0.0);
        if (m->cur.seg1_only)
            v8bis_tone_tx_seg1_only(&m->tone);
        m->tone_active = true;
        break;
    case SEG_BITS:
        fsk_tx_restart(m->ftx[m->cur.high], &preset_fsk_specs[m->cur.high ? FSK_V21CH2 : FSK_V21CH1]);
        fsk_tx_power(m->ftx[m->cur.high], (float)m->cfg.level_dbm0);
        m->bit_idx = 0;
        m->bits_over = false;
        break;
    default:
        break;
    }
    return true;
}

static void end_segment(v8bis_modem_t *m)
{
    if (m->cur.kind == SEG_TONE) {
        m->tone_active = false;
        if (m->cur.watch) {
            m->watch_armed = true;
            m->watch_from = m->now + m->blk_off;
        }
    }
    m->cur_valid = false;
}

bool v8bis_modem_tx_busy(const v8bis_modem_t *m)
{
    return m->cur_valid || m->sn > 0;
}

bool v8bis_modem_heard_peer(const v8bis_modem_t *m)
{
    return m->rx_active;
}

uint64_t v8bis_modem_now(const v8bis_modem_t *m)
{
    return m->now;
}

const v8bis_fsm_t *v8bis_modem_fsm(const v8bis_modem_t *m)
{
    return &m->fsm;
}

/* ---- actions -> segments ------------------------------------------------ */

static bool role_high(const v8bis_modem_t *m, v8bis_role_t role)
{
    if (role == V8BIS_ROLE_INITIATOR)
        return false;                       /* V.21(L) */
    if (role == V8BIS_ROLE_RESPONDER)
        return true;                        /* V.21(H) */
    return !m->last_rx_high;                /* answering a message with no role yet: the other channel */
}

static void queue_messages(v8bis_modem_t *m, const v8bis_action_t *a)
{
    bool high = role_high(m, a->role);
    seg_t *s;
    size_t nb = 0;

    if (a->es != V8BIS_ES_NONE) {
        q_tone(m, a->es == V8BIS_ES_ESI ? V8BIS_SIG_ESI : V8BIS_SIG_ESR, a->es == V8BIS_ES_ESR, true, false);
        if (a->es_gap) {
            /* Segment 2 as plain marking, then the silence, then the message with its own preamble. */
            s = q_bits(m, high);
            if (s) {
                memset(s->bits, 1, MARK_PREAMBLE_BITS - MODULATOR_LEAD_BITS);
                s->nbits = MARK_PREAMBLE_BITS - MODULATOR_LEAD_BITS;
            }
            q_silence(m, m->cfg.es_gap_ms);
        }
    }
    s = q_bits(m, high);
    if (!s)
        return;
    for (unsigned i = 0; i < a->n_msg; i++) {
        uint8_t info[V8BIS_MAX_INFO_OCTETS];
        int n = v8bis_msg_encode(&a->msg[i], info, sizeof(info));
        size_t b;

        if (n <= 0)
            continue;
        /* back to back (CL-MS): the second message's preamble follows the first's flag directly */
        b = v8bis_frame_encode(info, (size_t)n, MARK_PREAMBLE_BITS - (i == 0 ? MODULATOR_LEAD_BITS : 0),
                               m->cfg.open_flags,
                               m->cfg.close_flags, s->bits + nb, BITS_MAX - nb);
        nb += b;
    }
    s->nbits = (unsigned)nb;
}

static void handle_actions(v8bis_modem_t *m)
{
    v8bis_action_t a;

    while (v8bis_fsm_next_action(&m->fsm, &a)) {
        v8bis_modem_event_t e;

        memset(&e, 0, sizeof(e));
        switch (a.type) {
        case V8BIS_ACT_SIGNAL: {
            bool initial = m->have_how && m->watching && m->first_signal
                           && (a.sig == V8BIS_SIG_MRE || a.sig == V8BIS_SIG_CRE);
            bool watch = m->watching && m->fsm.role == V8BIS_ROLE_INITIATOR
                         && (m->fsm.state == V8BIS_S_SENT_MR || m->fsm.state == V8BIS_S_SENT_CR)
                         && (a.sig == V8BIS_SIG_MRE || a.sig == V8BIS_SIG_MRD
                             || a.sig == V8BIS_SIG_CRE || a.sig == V8BIS_SIG_CRD);

            if (initial)
                q_silence(m, m->cfg.initial_silence_ms);     /* 10.2.2: at least 400 ms of nothing first */
            m->first_signal = false;
            q_tone(m, a.sig, a.responding_set, false, watch);
            break;
        }
        case V8BIS_ACT_MESSAGES:
            queue_messages(m, &a);
            break;
        case V8BIS_ACT_MS_MODE:
            e.type = V8BIS_MEV_MODE;
            e.mode = a.mode;
            post(m, &e);
            m->watching = false;
            break;
        case V8BIS_ACT_INITIAL:
            e.type = (a.why == V8BIS_WHY_TIMEOUT && !m->rx_active) ? V8BIS_MEV_NO_PEER : V8BIS_MEV_INITIAL;
            e.why = a.why;
            e.nak = a.nak;
            post(m, &e);
            m->watching = false;
            break;
        }
    }
}

bool v8bis_modem_initiate(v8bis_modem_t *m, v8bis_init_t how)
{
    if (m->fsm.state != V8BIS_S_INITIAL)
        return false;
    m->have_how = true;
    m->how = how;
    m->retries_left = m->cfg.retries;
    m->first_signal = true;
    m->watching = how == V8BIS_INIT_MR || how == V8BIS_INIT_CR;
    m->watch_armed = false;
    m->rx_active = false;
    if (!v8bis_fsm_initiate(&m->fsm, how))
        return false;
    handle_actions(m);
    return true;
}

void v8bis_modem_startup_signal(v8bis_modem_t *m)
{
    v8bis_fsm_startup_signal(&m->fsm);
    handle_actions(m);
}

/* ---- transmit ------------------------------------------------------------ */

static void maybe_retransmit(v8bis_modem_t *m)
{
    v8bis_modem_event_t e;

    if (!m->watching || !m->watch_armed || v8bis_modem_tx_busy(m) || m->rx_active)
        return;
    if (m->fsm.role != V8BIS_ROLE_INITIATOR
        || (m->fsm.state != V8BIS_S_SENT_MR && m->fsm.state != V8BIS_S_SENT_CR))
        return;
    if (m->now - m->watch_from < ms_to_samples(m->cfg.retransmit_ms))
        return;
    v8bis_fsm_abandon(&m->fsm);
    m->watch_armed = false;
    if (m->retries_left) {
        m->retries_left--;
        v8bis_fsm_initiate(&m->fsm, m->how);                   /* the same signal again, no new silence */
        handle_actions(m);
        return;
    }
    memset(&e, 0, sizeof(e));
    e.type = V8BIS_MEV_NO_PEER;
    post(m, &e);
    m->watching = false;
}

void v8bis_modem_tx(v8bis_modem_t *m, int16_t *amp, int len)
{
    int i = 0;
    int idle = 0, audio = 0;

    while (i < len) {
        m->blk_off = (unsigned)i;
        if (!m->cur_valid && !next_segment(m)) {
            amp[i++] = 0;
            idle++;
            continue;
        }
        switch (m->cur.kind) {
        case SEG_SILENCE: {
            unsigned left = m->cur.n_samples - m->cur_pos;
            unsigned n = (unsigned)(len - i) < left ? (unsigned)(len - i) : left;

            memset(amp + i, 0, n * sizeof(int16_t));
            i += (int)n;
            m->cur_pos += n;
            m->blk_off = (unsigned)i;
            if (m->cur_pos >= m->cur.n_samples)
                end_segment(m);
            break;
        }
        case SEG_TONE: {
            int n = v8bis_tone_tx(&m->tone, amp + i, len - i);

            i += n;
            audio += n;
            m->blk_off = (unsigned)i;
            if (v8bis_tone_tx_done(&m->tone))
                end_segment(m);
            break;
        }
        case SEG_BITS: {
            int16_t s = 0;

            fsk_tx(m->ftx[m->cur.high], &s, 1);
            if (m->bits_over) {                 /* that sample belonged to the terminator bit */
                m->blk_off = (unsigned)i;
                end_segment(m);
                break;
            }
            amp[i++] = s;
            audio++;
            break;
        }
        }
    }
    m->now += (uint64_t)len;
    if (audio)                                  /* deaf to tones while we speak, and while the echo dies away */
        m->tone_mute_until = m->now + TONE_MUTE_AFTER;

    m->idle_acc += (unsigned)idle;
    while (m->idle_acc >= 160) {                /* the 9.8 clock runs while we are only listening */
        m->idle_acc -= 160;
        v8bis_fsm_tick(&m->fsm, 20);
        handle_actions(m);
    }
    maybe_retransmit(m);
}

/* ---- receive --------------------------------------------------------------- */

static bool listening_high(const v8bis_modem_t *m)
{
    return m->fsm.role == V8BIS_ROLE_INITIATOR;
}

static void frame_cb(void *user, const uint8_t *info, size_t len, v8bis_frame_status_t st)
{
    chan_t *c = user;
    v8bis_modem_t *m = c->m;
    v8bis_modem_event_t e;
    v8bis_msg_t d;

    if (c->ch != (listening_high(m) ? 1 : 0))
        return;
    if (!c->armed || c->tainted)
        return;                                 /* no preamble in front of it, or the other channel was stronger */
    /* A frame that fails 7.2.9 earns a NAK(1) only if it looks like an attempt at one: the preamble
     * armed it above, and 7.2.5's two opening flags are there.  Noise and stray tones produce the
     * odd flag-shaped run of bits; they must not make an idle station start transmitting. */
    if (st != V8BIS_FRAME_OK && c->fr.open_flags < 2) {
        c->armed = false;
        return;
    }
    c->armed = false;
    m->last_rx_high = c->ch == 1;
    m->rx_active = true;
    memset(&e, 0, sizeof(e));
    if (st == V8BIS_FRAME_OK && v8bis_msg_decode(info, len, &d) == 0) {
        e.type = V8BIS_MEV_RX_MESSAGE;
        e.msg_type = d.type;
        post(m, &e);
        v8bis_fsm_message(&m->fsm, &d);
    } else {
        e.type = V8BIS_MEV_RX_BAD_FRAME;
        post(m, &e);
        v8bis_fsm_invalid_frame(&m->fsm);
    }
    handle_actions(m);
}

static void put_bit(void *user, int bit)
{
    chan_t *c = user;

    if (bit < 0) {
        if (bit == SIG_STATUS_CARRIER_UP) {
            c->carrier = true;
            c->ones = 0;
            c->armed = false;
            c->tainted = false;
            v8bis_frame_rx_init(&c->fr, frame_cb, c);
        } else if (bit == SIG_STATUS_CARRIER_DOWN) {
            /* the carrier went in the middle of a frame: not properly bounded by flags (7.2.9 a) */
            if (c->armed && c->fr.in_frame && c->fr.nbits > 0)
                frame_cb(c, NULL, 0, V8BIS_FRAME_ABORT);
            c->carrier = false;
            c->armed = false;
        }
        return;
    }
    if (!c->carrier)
        return;
    if (!c->dominant)
        c->tainted = true;
    if (bit) {
        c->ones++;
    } else {
        if (c->ones >= PREAMBLE_ARM_BITS) {
            c->armed = true;                    /* this zero opens the first flag */
            c->tainted = !c->dominant;
        }
        c->ones = 0;
    }
    v8bis_frame_rx_bit(&c->fr, bit);
}

/* Goertzel power at f over a block, normalised by block energy x N/2 so a pure tone reads 1. */
static double band_power(const int16_t *x, int n, double f)
{
    double w = 2.0 * 3.14159265358979323846 * f / 8000.0, c = 2.0 * cos(w), s1 = 0.0, s2 = 0.0;

    for (int i = 0; i < n; i++) {
        double s0 = x[i] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return (s1 * s1 + s2 * s2 - c * s1 * s2) / ((double)n * n / 4.0);
}

/* A V.21 channel is only believed when its band holds clearly more energy than the other one.  A
 * hybrid returns our own message at a level that leaks through the other demodulator's filter and
 * decodes as a perfectly good frame; the stronger-band rule is what rejects it. */
static void update_dominance(v8bis_modem_t *m, const int16_t *x, int n)
{
    double lo = band_power(x, n, 980.0) + band_power(x, n, 1180.0);
    double hi = band_power(x, n, 1650.0) + band_power(x, n, 1850.0);

    /* A block of silence says nothing: the demodulator is a few bits behind the line, so the last
     * bits of a message are delivered while the block being judged is already quiet. */
    if (lo + hi < SILENT_BAND_POWER)
        return;
    /* The same goes for the block in which the signal dies: a few milliseconds of transient and
     * filter splatter, 20 dB under what was there a moment ago, can point at either band. */
    if (lo + hi < m->band_peak * 0.01) {
        m->band_peak *= 0.98;
        return;
    }
    m->band_peak = lo + hi > m->band_peak * 0.98 ? lo + hi : m->band_peak * 0.98;
    m->ch[0].dominant = lo > 4.0 * hi;
    m->ch[1].dominant = hi > 4.0 * lo;
}

static void deliver_signal(v8bis_modem_t *m, v8bis_signal_t sig, bool set)
{
    v8bis_modem_event_t e;

    m->rx_active = true;
    memset(&e, 0, sizeof(e));
    e.type = V8BIS_MEV_RX_SIGNAL;
    e.sig = sig;
    e.responding_set = set;
    post(m, &e);
    v8bis_fsm_signal(&m->fsm, sig, set);
    handle_actions(m);
}

static void rx_block(v8bis_modem_t *m, const int16_t *x)
{
    v8bis_tone_event_t te;

    v8bis_tone_rx(&m->trx, x, V8BIS_RX_BLOCK);
    m->rx_pos += V8BIS_RX_BLOCK;
    while (v8bis_tone_rx_event(&m->trx, &te)) {
        if (m->now < m->tone_mute_until)
            continue;                           /* our own signal coming back */
        /* The detector knows a signal at the second tone; the peer is still sending the rest of
         * it.  Answering over its tail would put our preamble on top of its segment 2, so the FSM
         * hears of it when the signal's 500 ms are over. */
        m->pend_valid = true;
        m->pend_sig = te.sig;
        m->pend_set = te.responding_set;
        m->pend_end = te.seg1_start + V8BIS_SEG1_SAMPLES + V8BIS_SEG2_SAMPLES;
    }
    if (m->pend_valid && m->rx_pos >= m->pend_end) {
        m->pend_valid = false;
        deliver_signal(m, m->pend_sig, m->pend_set);
    }
    update_dominance(m, x, V8BIS_RX_BLOCK);
    for (int c = 0; c < 2; c++)
        fsk_rx(m->ch[c].rx, x, V8BIS_RX_BLOCK);
}

void v8bis_modem_rx(v8bis_modem_t *m, const int16_t *amp, int len)
{
    for (int i = 0; i < len; i++) {
        m->rxbuf[m->rxfill++] = amp[i];
        if (m->rxfill == V8BIS_RX_BLOCK) {
            rx_block(m, m->rxbuf);
            m->rxfill = 0;
        }
    }
    m->rx_now += (uint64_t)len;
}

/* ---- lifecycle --------------------------------------------------------------- */

v8bis_modem_t *v8bis_modem_new(const v8bis_modem_cfg_t *cfg)
{
    v8bis_modem_t *m = calloc(1, sizeof(*m));

    if (!m)
        return NULL;
    m->cfg = *cfg;
    v8bis_fsm_init(&m->fsm, &m->cfg.fsm);
    v8bis_tone_rx_init(&m->trx);
    for (int c = 0; c < 2; c++) {
        m->ftx[c] = fsk_tx_init(NULL, &preset_fsk_specs[c ? FSK_V21CH2 : FSK_V21CH1], tx_get_bit, m);
        m->ch[c].m = m;
        m->ch[c].ch = c;
        m->ch[c].rx = fsk_rx_init(NULL, &preset_fsk_specs[c ? FSK_V21CH2 : FSK_V21CH1],
                                  FSK_FRAME_MODE_SYNC, put_bit, &m->ch[c]);
        v8bis_frame_rx_init(&m->ch[c].fr, frame_cb, &m->ch[c]);
        if (!m->ftx[c] || !m->ch[c].rx) {
            v8bis_modem_free(m);
            return NULL;
        }
    }
    return m;
}

void v8bis_modem_free(v8bis_modem_t *m)
{
    if (!m)
        return;
    for (int c = 0; c < 2; c++) {
        if (m->ftx[c])
            fsk_tx_free(m->ftx[c]);
        if (m->ch[c].rx)
            fsk_rx_free(m->ch[c].rx);
    }
    free(m);
}
