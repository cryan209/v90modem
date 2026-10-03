/*
 * v92_mh_test.c — V.92 modem-on-hold codec and 9.10 transactions.
 *
 * The CRC is graded by an independent MSB-register oracle (V.34
 * 10.1.2.3.2), not by the production helper.  Transactions run two
 * controllers against each other over a line with a one-way delay, MH bits
 * at 600 bit/s (8.2.3.1/V.90), and each side's detectors derived from what
 * the other side actually put on the line.  Every scenario is a figure or a
 * clause: Figures 20-24, 9.10.1.1's timeout and retrain discrimination,
 * and Cor.1's optional Tone RT skip.
 */
#include "v92_mh.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint16_t oracle_crc(const uint8_t *b)
{
    uint16_t r = 0xFFFF, wire = 0;
    for (int i = 12; i <= 19; i++) {
        int fb = (r >> 15) ^ b[i];
        r <<= 1;
        if (fb) r ^= 0x1021;
    }
    for (int i = 0; i < 16; i++) wire |= ((r >> i) & 1) << (15 - i);
    return wire;
}

static void test_codec(void)
{
    static const v92_mh_signal_t all[] = {
        V92_MH_REQ, V92_MH_ACK, V92_MH_NACK, V92_MH_CLRD, V92_MH_CDA, V92_MH_FRR
    };
    static const uint8_t sync[8] = {0,1,1,1,0,0,1,0};
    uint8_t b[V92_MH_BITS];
    v92_mh_decode_t d;

    for (size_t k = 0; k < sizeof(all)/sizeof(all[0]); k++) {
        for (int info = 0; info < 16; info++) {
            v92_mh_frame_t f = {all[k], (uint8_t)info};
            unsigned field = 0;
            assert(v92_mh_encode(&f, b));
            for (int i = 0; i < 4; i++) assert(b[i] == 1 && b[36+i] == 1);
            assert(memcmp(b + 4, sync, 8) == 0);
            for (int i = 0; i < 16; i++) field |= (unsigned)b[20+i] << i;
            assert(field == oracle_crc(b));
            assert(v92_mh_decode(b, &d));
            assert(d.frame.signal == all[k]);
            if (all[k] == V92_MH_REQ || all[k] == V92_MH_CDA || all[k] == V92_MH_FRR)
                assert(d.frame.info == (uint8_t)all[k] && d.info_defined);
            else
                assert(d.frame.info == info);
            /* Any single bit error in 12:35 is caught. */
            for (int i = 12; i < 36; i++) {
                b[i] ^= 1;
                assert(!v92_mh_decode(b, NULL));
                b[i] ^= 1;
            }
        }
    }
    /* Table 32 note 1: an undefined signal with a good CRC is ignored. */
    {
        v92_mh_frame_t f = {V92_MH_REQ, 0};
        uint16_t crc;
        v92_mh_encode(&f, b);
        b[12] = 0; b[13] = 0; b[14] = 0; b[15] = 0;   /* 0000 */
        crc = oracle_crc(b);
        for (int i = 0; i < 16; i++) b[20+i] = (crc >> i) & 1;
        assert(!v92_mh_decode(b, &d));
        assert(d.crc_ok && !d.signal_known);
    }
    /* Table 33. */
    assert(v92_mh_t1_seconds(0x0) == -1 && v92_mh_t1_seconds(0x1) == 10);
    assert(v92_mh_t1_seconds(0x5) == 60 && v92_mh_t1_seconds(0xC) == 960);
    assert(v92_mh_t1_seconds(0xD) == 0 && v92_mh_t1_seconds(0xF) == -1);
    /* Table 34. */
    assert(v92_mh_is_response_to(V92_MH_REQ, V92_MH_ACK));
    assert(v92_mh_is_response_to(V92_MH_NACK, V92_MH_FRR));
    assert(!v92_mh_is_response_to(V92_MH_CLRD, V92_MH_ACK));
    assert(!v92_mh_is_response_to(V92_MH_FRR, V92_MH_CDA));
    /* Amd.1: MHclrd/MHnack reserved info is flagged, not rejected. */
    {
        v92_mh_frame_t f = {V92_MH_CLRD, 0x3};
        v92_mh_encode(&f, b);
        assert(v92_mh_decode(b, &d) && !d.info_defined);
    }
    puts("PASS: MH codec (Tables 32-34, V.34 10.1.2.3.2 oracle, every 1-bit error)");
}

static void test_framer(void)
{
    v92_mh_rx_t r;
    v92_mh_frame_t f = {V92_MH_CLRD, V92_MH_CLRD_INCOMING}, got;
    uint8_t b[V92_MH_BITS];
    unsigned seed = 12345;
    int found = 0, first_at = -1, n = 0;

    v92_mh_rx_init(&r);
    for (int i = 0; i < 37; i++) {           /* junk, then back to back */
        seed = seed * 1103515245u + 12345u;
        v92_mh_rx_put_bit(&r, (seed >> 16) & 1, NULL);
        n++;
    }
    v92_mh_encode(&f, b);
    for (int rep = 0; rep < 5; rep++) {
        for (int i = 0; i < V92_MH_BITS; i++, n++) {
            if (v92_mh_rx_put_bit(&r, b[i], &got)) {
                assert(got.signal == V92_MH_CLRD && got.info == V92_MH_CLRD_INCOMING);
                if (first_at < 0) first_at = n;
                found++;
            }
        }
    }
    /* Back-to-back sequences share no fill, so each is found exactly once. */
    assert(found == 5 && first_at == 37 + V92_MH_BITS - 1);
    puts("PASS: MH framer (5 back-to-back sequences after junk, no false hits)");
}

/* ---- two modems on a line ---- */

#define DELAY_MS 60                         /* one way; round trip 120 */
#define HIST 4096

typedef struct {
    v92_mh_ctrl_t c;
    v92_mh_tx_t hist_tx[HIST];
    uint8_t hist_bit[HIST * 2];             /* bits, indexed by bit count */
    int bit_ms[HIST * 2];                   /* time each bit was sent */
    int nbits;
    double credit;
    int run_bits;                           /* bits in the current MH run */
    bool reversal_next;                     /* inject a Tone B reversal */
    bool dumb;                              /* never responds */
    v92_mh_action_t acts[16];
    int act_ms[16];
    int nact;
    int ansam_start, last_mh_end;
    bool phase1_to_peer;                    /* the peer sends CM (for on-hold exit) */
} side_t;

static int now;

static void side_init(side_t *s, int rtt)
{
    memset(s, 0, sizeof(*s));
    v92_mh_ctrl_init(&s->c, rtt);
    s->ansam_start = -1;
    s->last_mh_end = -1;
}

static void step(side_t *s, const side_t *o)
{
    v92_mh_detect_t l = {0};
    v92_mh_tx_t far = now >= DELAY_MS ? o->hist_tx[(now - DELAY_MS) % HIST] : V92_MH_TX_DATA;
    v92_mh_action_t a;

    l.rt = far == V92_MH_TX_RT;
    l.silence = far == V92_MH_TX_SILENCE;
    l.ansam = far == V92_MH_TX_ANSAM;
    l.reversal = s->reversal_next;
    l.phase1 = s->phase1_to_peer;
    s->reversal_next = false;
    if (!s->dumb)
        v92_mh_ctrl_tick(&s->c, 1, &l);
    while ((a = v92_mh_ctrl_take_action(&s->c)) != V92_MH_ACT_NONE) {
        assert(s->nact < 16);
        s->acts[s->nact] = a;
        s->act_ms[s->nact++] = now;
    }
}

static void emit(side_t *s)
{
    v92_mh_tx_t tx = s->c.tx;

    s->hist_tx[now % HIST] = tx;
    if (tx == V92_MH_TX_MH) {
        s->credit += V92_MH_BIT_RATE / 1000.0;
        while (s->credit >= 1.0) {
            s->credit -= 1.0;
            assert(s->nbits < HIST * 2);
            s->bit_ms[s->nbits] = now;
            s->hist_bit[s->nbits++] = (uint8_t)v92_mh_ctrl_tx_bit(&s->c);
            s->run_bits++;
        }
    } else {
        if (s->run_bits) {
            /* 9.10.1: every sequence is completed. */
            assert(s->run_bits % V92_MH_BITS == 0);
            s->last_mh_end = now;
        }
        s->run_bits = 0;
        s->credit = 0;
        if (tx == V92_MH_TX_ANSAM && s->ansam_start < 0)
            s->ansam_start = now;
    }
}

static void deliver(side_t *to, const side_t *from, int *next)
{
    while (*next < from->nbits && from->bit_ms[*next] + DELAY_MS <= now) {
        if (!to->dumb)
            v92_mh_ctrl_rx_bit(&to->c, from->hist_bit[*next]);
        (*next)++;
    }
}

static bool has(const side_t *s, v92_mh_action_t a, int *at)
{
    for (int i = 0; i < s->nact; i++)
        if (s->acts[i] == a) { if (at) *at = s->act_ms[i]; return true; }
    return false;
}

static void run(side_t *a, side_t *b, int ms)
{
    static int na, nb;
    if (now == 0) na = nb = 0;
    for (int end = now + ms; now < end; now++) {
        deliver(a, b, &nb);
        deliver(b, a, &na);
        step(a, b);
        step(b, a);
        emit(a);
        emit(b);
    }
}

static void start(side_t *a, side_t *b)
{
    now = 0;
    side_init(a, 2 * DELAY_MS);
    side_init(b, 2 * DELAY_MS);
    run(a, b, 100);                      /* data mode */
}

static void test_request_granted(void)
{
    side_t a, b;
    int t_hold_a, t_hold_b, t_disc;

    start(&a, &b);
    b.c.t1_code = 0x1;                   /* 10 s */
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_REQ, 0));
    run(&a, &b, 3000);
    assert(has(&a, V92_MH_ACT_SUSPEND_LINK, NULL) && has(&b, V92_MH_ACT_SUSPEND_LINK, NULL));
    assert(has(&a, V92_MH_ACT_ON_HOLD, &t_hold_a) && has(&b, V92_MH_ACT_ON_HOLD, &t_hold_b));
    assert(a.c.state == V92_MH_ST_INIT_HOLD && a.c.tx == V92_MH_TX_RT);
    assert(b.c.state == V92_MH_ST_ON_HOLD && b.c.tx == V92_MH_TX_ANSAM);
    /* Figure 20: ANSam within 80 ms of the last MHack. */
    assert(b.ansam_start >= b.last_mh_end && b.ansam_start - b.last_mh_end <= 80);
    /* T1 = 10 s from the end of the first MHack, then disconnect. */
    run(&a, &b, 12000);
    assert(has(&b, V92_MH_ACT_DISCONNECT, &t_disc));
    printf("PASS: Figure 20 MHreq/MHack (A on hold %d ms, B ANSam %d ms after MHack, T1 disconnect at %d ms)\n",
           t_hold_a, b.ansam_start - b.last_mh_end, t_disc);
}

static void test_on_hold_resume(void)
{
    side_t a, b;
    start(&a, &b);
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_REQ, 0));
    run(&a, &b, 3000);
    assert(b.c.state == V92_MH_ST_ON_HOLD);
    b.phase1_to_peer = true;             /* A returns with CM */
    run(&a, &b, 10);
    assert(has(&b, V92_MH_ACT_PHASE1_ANSWER, NULL) && !has(&b, V92_MH_ACT_DISCONNECT, NULL));
    puts("PASS: on hold, CM -> Phase 1 as answer modem (9.10.2.1)");
}

static void test_denied_cleardown(void)
{
    side_t a, b;
    start(&a, &b);
    b.c.grant = false;
    b.c.nack_reason = V92_MH_NACK_NEVER;
    a.c.after_nack_reconnect = false;
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_REQ, 0));
    run(&a, &b, 3000);
    assert(has(&a, V92_MH_ACT_DISCONNECT, NULL) && has(&b, V92_MH_ACT_DISCONNECT, NULL));
    assert(!has(&b, V92_MH_ACT_ON_HOLD, NULL));
    assert(a.c.no_outgoing_requests);
    puts("PASS: Figure 21 MHreq/MHnack/MHcda, both disconnect; MHnack 0101 remembered");
}

static void test_denied_reconnect(void)
{
    side_t a, b;
    int ta, tb;
    start(&a, &b);
    b.c.grant = false;
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_REQ, 0));
    run(&a, &b, 4000);
    assert(has(&b, V92_MH_ACT_PHASE1_ANSWER, &tb));
    assert(has(&a, V92_MH_ACT_PHASE1_CALL, &ta));
    assert(ta - tb >= 1000);              /* 1 s of ANSam (9.10.2.3) */
    assert(b.ansam_start - b.last_mh_end <= 80);
    printf("PASS: Figure 22 MHreq/MHnack/MHfrr/ANSam (B answer at %d, A call at %d ms)\n", tb, ta);
}

static void test_cleardown(void)
{
    side_t a, b;
    start(&a, &b);
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_CLRD, V92_MH_CLRD_OUTGOING));
    run(&a, &b, 3000);
    assert(has(&a, V92_MH_ACT_DISCONNECT, NULL) && has(&b, V92_MH_ACT_DISCONNECT, NULL));
    puts("PASS: Figure 23 MHclrd/MHcda, both disconnect");
}

static void test_fast_reconnect(void)
{
    side_t a, b;
    start(&a, &b);
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_FRR, 0));
    run(&a, &b, 3000);
    assert(has(&b, V92_MH_ACT_PHASE1_ANSWER, NULL) && has(&a, V92_MH_ACT_PHASE1_CALL, NULL));
    puts("PASS: Figure 24 MHfrr/ANSam");
}

static void test_timeout_retrain(void)
{
    side_t a, b;
    int t;
    start(&a, &b);
    b.dumb = true;                       /* sends its RT forever, never decodes */
    b.c.tx = V92_MH_TX_RT;
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_REQ, 0));
    run(&a, &b, 4000);
    assert(has(&a, V92_MH_ACT_RETRAIN, &t));
    assert(t >= 100 + 70 + 2000 + 2 * DELAY_MS);
    printf("PASS: 9.10.1.1 no response -> sequence completed, retrain at %d ms\n", t);
}

static void test_retrain_discrimination(void)
{
    side_t a, b;
    start(&a, &b);
    b.dumb = true;                       /* the "initiator" is really retraining */
    b.c.tx = V92_MH_TX_RT;
    run(&a, &b, 200);
    assert(a.c.state == V92_MH_ST_RESP_RT);
    a.reversal_next = true;
    run(&a, &b, 5);
    assert(has(&a, V92_MH_ACT_RETRAIN, NULL));
    puts("PASS: 9.10.1.1 responder takes a Tone B reversal as a retrain");
}

static void test_cor1_rt_skip(void)
{
    side_t a, b;
    start(&a, &b);
    /* B raises its RT 10 ms before A starts, so it arrives inside A's
     * 70 ms silence. */
    b.c.state = V92_MH_ST_RESP_RT;
    b.c.tx = V92_MH_TX_RT;
    run(&a, &b, 10);
    assert(v92_mh_ctrl_initiate(&a.c, V92_MH_REQ, 0));
    run(&a, &b, 70);
    assert(a.c.peer_rt_during_silence);
    run(&a, &b, 2);
    assert(a.c.tx == V92_MH_TX_MH);       /* no Tone RT of its own */
    puts("PASS: Cor.1 9.10.1 Tone RT skipped when the peer's RT arrived during the silence");
}

int main(void)
{
    test_codec();
    test_framer();
    test_request_granted();
    test_on_hold_resume();
    test_denied_cleardown();
    test_denied_reconnect();
    test_cleardown();
    test_fast_reconnect();
    test_timeout_retrain();
    test_retrain_discrimination();
    test_cor1_rt_skip();
    puts("v92_mh_test: all passed");
    return 0;
}
