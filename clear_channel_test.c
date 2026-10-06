/*
 * clear_channel_test.c -- 64/56 kbit/s clear channel and V.120 on the DS0.
 *
 * Three layers.  The octet packing on its own (first bit in bit 8; at 56k
 * seven bits and bit 1 forced to 1).  Two instances back to back, both
 * modes and both rates, data both ways, with the V.120 frames on the wire
 * read by an independent HDLC receiver so the address/control/header octets
 * are checked as sent rather than as our own receiver accepts them.  Then
 * the engine: AT+MS on the PTY, a call connect that must skip V.8 and report
 * CONNECT at the line rate, and the engine's own DS0 looped back on itself
 * so text typed at the PTY comes back to it.
 */

#include "clear_channel.h"
#include "data_stack.h"
#include "data_interface.h"
#include "modem_engine.h"

#include <spandsp.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static int failures = 0;

static void check(int ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        failures++;
}

/* ---------------------------------------------------------------- */
/* Byte sources and sinks                                            */
/* ---------------------------------------------------------------- */

typedef struct {
    const uint8_t *src;
    size_t src_len, src_pos;
    uint8_t dst[8192];
    size_t dst_len;
    data_stack_t ds;        /* CLEAR mode only */
} end_t;

static int end_pull(void *ctx)
{
    end_t *e = ctx;

    return e->src_pos < e->src_len ? e->src[e->src_pos++] : -1;
}

static void end_push(void *ctx, uint8_t b)
{
    end_t *e = ctx;

    if (e->dst_len < sizeof(e->dst))
        e->dst[e->dst_len++] = b;
}

static int end_get_bit(void *ctx)
{
    return ds_tx_get_bit(&((end_t *) ctx)->ds);
}

static void end_put_bit(void *ctx, int bit)
{
    ds_rx_put_bit(&((end_t *) ctx)->ds, bit);
}

/* ---------------------------------------------------------------- */
/* Octet packing                                                     */
/* ---------------------------------------------------------------- */

static const int pattern[] = { 1, 0, 1, 1, 0, 0, 1, 0,   0, 1, 1, 1, 0, 1, 0, 0 };
static size_t pattern_pos;

static int pattern_bit(void *ctx)
{
    (void) ctx;
    return pattern[pattern_pos++ % 16];
}

static void test_packing(void)
{
    clear_channel_t cc;
    uint8_t o[2];

    printf("octet packing:\n");
    pattern_pos = 0;
    cc_init_clear(&cc, false, pattern_bit, NULL, NULL);
    cc_tx(&cc, o, 2);
    check(o[0] == 0xB2 && o[1] == 0x74, "64k: first bit in bit 8 (0xB2 0x74)");
    check(cc_line_rate(&cc) == 64000, "64k line rate");

    pattern_pos = 0;
    cc_init_clear(&cc, true, pattern_bit, NULL, NULL);
    cc_tx(&cc, o, 2);
    /* 1011001 + 1, then 0011101 + 1 */
    check(o[0] == 0xB3 && o[1] == 0x3B, "56k: seven bits, bit 1 forced to 1 (0xB3 0x3B)");
    check(cc_line_rate(&cc) == 56000, "56k line rate");
}

/* ---------------------------------------------------------------- */
/* Back to back                                                      */
/* ---------------------------------------------------------------- */

static uint8_t text_a[3000], text_b[1200];

static void make_text(void)
{
    for (size_t i = 0; i < sizeof(text_a); i++)
        text_a[i] = (uint8_t) (i * 7 + 3);          /* every octet value */
    for (size_t i = 0; i < sizeof(text_b); i++)
        text_b[i] = (uint8_t) ("The quick brown fox. "[i % 21]);
}

/* Run both directions for `octets` DS0 octets; return whether 56k octets
 * all had bit 1 set. */
static bool run_pair(clear_channel_t *a, clear_channel_t *b, int octets, bool r56,
                     hdlc_rx_state_t *spy_ab, hdlc_rx_state_t *spy_ba)
{
    uint8_t ab[160], ba[160];
    bool lsb_ok = true;

    for (int done = 0; done < octets; done += 160) {
        cc_tx(a, ab, 160);
        cc_tx(b, ba, 160);
        if (r56)
            for (int i = 0; i < 160; i++)
                lsb_ok = lsb_ok && (ab[i] & 1) && (ba[i] & 1);
        if (spy_ab)
            for (int i = 0; i < 160; i++)
                for (int k = 0; k < (r56 ? 7 : 8); k++)
                    hdlc_rx_put_bit(spy_ab, (ab[i] >> (7 - k)) & 1);
        if (spy_ba)
            for (int i = 0; i < 160; i++)
                for (int k = 0; k < (r56 ? 7 : 8); k++)
                    hdlc_rx_put_bit(spy_ba, (ba[i] >> (7 - k)) & 1);
        cc_rx(b, ab, 160);
        cc_rx(a, ba, 160);
    }
    return lsb_ok;
}

static void test_clear_pair(bool r56)
{
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    char what[96];
    bool lsb;

    ea->src = text_a; ea->src_len = sizeof(text_a);
    eb->src = text_b; eb->src_len = sizeof(text_b);
    ds_init(&ea->ds, DS_FRAMING_V14, end_pull, ea, end_push, ea);
    ds_init(&eb->ds, DS_FRAMING_V14, end_pull, eb, end_push, eb);
    ds_set_v14_rates(&ea->ds, r56 ? 56000 : 64000, r56 ? 56000 : 64000);
    ds_set_v14_rates(&eb->ds, r56 ? 56000 : 64000, r56 ? 56000 : 64000);
    cc_init_clear(&a, r56, end_get_bit, end_put_bit, ea);
    cc_init_clear(&b, r56, end_get_bit, end_put_bit, eb);
    /* 3000 characters at 10 bits on 7 or 8 bits an octet, with room. */
    lsb = run_pair(&a, &b, 8000, r56, NULL, NULL);

    snprintf(what, sizeof(what), "CLEAR %s: %zu bytes A->B exact", r56 ? "56k" : "64k",
             sizeof(text_a));
    check(eb->dst_len == sizeof(text_a) && !memcmp(eb->dst, text_a, sizeof(text_a)), what);
    snprintf(what, sizeof(what), "CLEAR %s: %zu bytes B->A exact", r56 ? "56k" : "64k",
             sizeof(text_b));
    check(ea->dst_len == sizeof(text_b) && !memcmp(ea->dst, text_b, sizeof(text_b)), what);
    if (r56)
        check(lsb, "CLEAR 56k: every octet both ways has bit 1 set");
    ds_release(&ea->ds);
    ds_release(&eb->ds);
    free(ea);
    free(eb);
}

/* The independent receiver: what V.120 frames are actually on the wire. */
static int spy_frames, spy_bad, spy_hdr_ok, spy_max_len;
static uint8_t spy_want[4];

static void spy_frame(void *user_data, const uint8_t *pkt, int len, int ok)
{
    (void) user_data;
    if (len < 0)
        return;
    if (!ok) {
        spy_bad++;
        return;
    }
    spy_frames++;
    if (len >= 4 && !memcmp(pkt, spy_want, 4))
        spy_hdr_ok++;
    if (len > spy_max_len)
        spy_max_len = len;
}

/* ---- V.120 acknowledged mode (Q.922, V.120 4.2) ---- */

static int ack_ctl_seen[256];
static void ack_spy_frame(void *user, const uint8_t *pkt, int len, int ok)
{
    (void) user;
    if (len >= 3 && ok)
        ack_ctl_seen[pkt[2]]++;
}

/* Run an acknowledged pair; blocks of 160 octets A->B in [la0,la1) and
 * B->A in [lb0,lb1) are destroyed.  Returns when `ticks` have run. */
static void ack_run(clear_channel_t *a, clear_channel_t *b, int ticks,
                    int la0, int la1, int lb0, int lb1, hdlc_rx_state_t *spy)
{
    uint8_t ab[160], ba[160];

    for (int t = 0; t < ticks; t++) {
        cc_tx(a, ab, 160);
        cc_tx(b, ba, 160);
        if (spy)
            for (int i = 0; i < 160; i++)
                for (int k = 0; k < 8; k++)
                    hdlc_rx_put_bit(spy, (ab[i] >> (7 - k)) & 1);
        if (t >= la0 && t < la1)
            memset(ab, 0x55, sizeof(ab));
        if (t >= lb0 && t < lb1)
            memset(ba, 0x55, sizeof(ba));
        cc_rx(b, ab, 160);
        cc_rx(a, ba, 160);
    }
}

/* ---- break, both framings, both directions ---- */

typedef struct {
    const char *src; size_t src_len, src_pos;
    char ev[256]; size_t ev_len;               /* received characters and break markers */
} brk_end_t;

static int brk_pull(void *ctx)
{
    brk_end_t *e = ctx;

    return e->src_pos < e->src_len ? (uint8_t) e->src[e->src_pos++] : -1;
}
static void brk_push(void *ctx, uint8_t b)
{
    brk_end_t *e = ctx;

    if (e->ev_len < sizeof(e->ev) - 1)
        e->ev[e->ev_len++] = (char) b;
}
static void brk_mark(void *ctx, bool on)
{
    brk_end_t *e = ctx;

    if (e->ev_len < sizeof(e->ev) - 1)
        e->ev[e->ev_len++] = on ? '[' : ']';
}

static void brk_run(clear_channel_t *a, clear_channel_t *b, brk_end_t *ea, int ticks)
{
    uint8_t ab[160], ba[160];

    for (int t = 0; t < ticks; t++) {
        if (t == 30)
            cc_send_break(a, 200);
        if (t == 32) {              /* more text, queued while the break is on */
            ea->src = "DEF";
            ea->src_len = 3;
            ea->src_pos = 0;
        }
        cc_tx(a, ab, 160);
        cc_tx(b, ba, 160);
        cc_rx(b, ab, 160);
        cc_rx(a, ba, 160);
    }
}

static void test_break(void)
{
    static brk_end_t ea, eb;
    clear_channel_t a, b;
    int n;

    printf("break (V.120 3.1.1.2, V.110 5.3.5):\n");
    for (n = 0; n < 4; n++) {
        const char *what[] = { "V.120 UI", "V.120 acknowledged", "V.110 9600", "V.110 300" };

        memset(&ea, 0, sizeof(ea));
        memset(&eb, 0, sizeof(eb));
        ea.src = "ABC";
        ea.src_len = 3;
        if (n < 2) {
            cc_init_v120(&a, false, true, brk_pull, brk_push, &ea);
            cc_init_v120(&b, false, false, brk_pull, brk_push, &eb);
            if (n == 1) {
                cc_v120_set_ack(&a, true);
                cc_v120_set_ack(&b, true);
            }
        } else {
            int rate = n == 2 ? 9600 : 300;

            cc_init_v110(&a, rate, brk_pull, brk_push, &ea);
            cc_init_v110(&b, rate, brk_pull, brk_push, &eb);
        }
        cc_set_break_cb(&b, brk_mark);
        cc_set_break_cb(&a, brk_mark);
        brk_run(&a, &b, &ea, n >= 2 ? 400 : 120);
        eb.ev[eb.ev_len] = 0;
        {
            char msg[120];

            snprintf(msg, sizeof(msg), "%s: B hears \"ABC[ ]DEF\" in order (got \"%s\")", what[n], eb.ev);
            check(!strcmp(eb.ev, "ABC[]DEF"), msg);
        }
        if (n < 2)
            check(b.rx_breaks == 1 && a.rx_breaks == 0, "exactly one break counted, none at the sender");
        cc_release(&a);
        cc_release(&b);
    }
}

static void test_v120_ack(void)
{
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    hdlc_rx_state_t *spy;

    printf("V.120 acknowledged mode (Q.922):\n");
    ea->src = text_a; ea->src_len = sizeof(text_a);
    eb->src = text_b; eb->src_len = sizeof(text_b);
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_init_v120(&b, false, false, end_pull, end_push, eb);
    cc_v120_set_ack(&a, true);
    cc_v120_set_ack(&b, true);
    memset(ack_ctl_seen, 0, sizeof(ack_ctl_seen));
    spy = hdlc_rx_init(NULL, false, true, 1, ack_spy_frame, NULL);
    hdlc_rx_set_max_frame_len(spy, 400);
    ack_run(&a, &b, 100, 0, 0, 0, 0, spy);
    check(a.lf_state == CC_LF_UP && b.lf_state == CC_LF_UP, "SABME / UA: both ends established");
    check(ack_ctl_seen[0x7F] >= 1 && ack_ctl_seen[0x73] >= 1,
          "on the wire: SABME (P = 1) 0x7F and UA (F = 1) 0x73, from independent decoding");
    check(eb->dst_len == sizeof(text_a) && !memcmp(eb->dst, text_a, sizeof(text_a))
          && ea->dst_len == sizeof(text_b) && !memcmp(ea->dst, text_b, sizeof(text_b)),
          "data both ways byte-exact over I-frames");
    check(a.rx_bad_frames == 0 && b.rx_bad_frames == 0 && a.lf_resets == 0 && b.lf_resets == 0
          && a.lf_state == CC_LF_UP && a.lf_va == a.lf_vnew,
          "no bad frames, no resets, everything acknowledged");
    {
        int i_frames = 0;

        for (int c = 0; c < 256; c += 2)
            i_frames += ack_ctl_seen[c];
        check(i_frames >= 12 && ack_ctl_seen[0x03] == 0, "I-frames carried the data, no UI frames");
    }
    hdlc_rx_free(spy);
    cc_release(&a);
    cc_release(&b);

    /* Loss: 6 blocks (0.12 s) of A's frames destroyed mid-transfer, and a
     * stretch of B's acknowledgements too.  REJ / T200 recovery must deliver
     * every byte exactly once and in order. */
    memset(ea, 0, sizeof(*ea));
    memset(eb, 0, sizeof(*eb));
    ea->src = text_a; ea->src_len = sizeof(text_a);
    eb->src = text_b; eb->src_len = sizeof(text_b);
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_init_v120(&b, false, false, end_pull, end_push, eb);
    cc_v120_set_ack(&a, true);
    cc_v120_set_ack(&b, true);
    ack_run(&a, &b, 400, 5, 9, 10, 14, NULL);
    check(eb->dst_len == sizeof(text_a) && !memcmp(eb->dst, text_a, sizeof(text_a))
          && ea->dst_len == sizeof(text_b) && !memcmp(ea->dst, text_b, sizeof(text_b)),
          "after loss both ways: every byte exactly once, in order");
    check(b.lf_discarded + a.lf_discarded + a.lf_rewinds + b.lf_rewinds > 0
          && a.lf_state == CC_LF_UP && b.lf_state == CC_LF_UP,
          "recovery actually ran (REJ / enquiry / rewind) and the link stayed up");
    cc_release(&a);
    cc_release(&b);

    /* An acknowledged caller against a UI-only peer: DM refuses, UI follows. */
    memset(ea, 0, sizeof(*ea));
    memset(eb, 0, sizeof(*eb));
    ea->src = text_a; ea->src_len = 800;
    eb->src = text_b; eb->src_len = 500;
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_init_v120(&b, false, false, end_pull, end_push, eb);
    cc_v120_set_ack(&a, true);
    ack_run(&a, &b, 60, 0, 0, 0, 0, NULL);
    check(a.lf_state == CC_LF_UI && a.lf_fallbacks == 1
          && eb->dst_len == 800 && !memcmp(eb->dst, text_a, 800)
          && ea->dst_len == 500 && !memcmp(ea->dst, text_b, 500),
          "UI-only peer answers SABME with DM: caller falls back to UI, data exact");
    cc_release(&a);
    cc_release(&b);

    /* A peer that says nothing at all: N200 SABMEs, then UI. */
    memset(ea, 0, sizeof(*ea));
    ea->src = text_a; ea->src_len = 300;
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_v120_set_ack(&a, true);
    {
        uint8_t o[160], quiet[160];

        memset(quiet, 0xFF, sizeof(quiet));
        for (int t = 0; t < 40; t++) {            /* 0.8 s: still trying */
            cc_tx(&a, o, 160);
            cc_rx(&a, quiet, 160);
        }
        check(a.lf_state == CC_LF_SETUP && ea->src_pos == 0, "SABME unanswered: no data sent yet");
        for (int t = 0; t < 400; t++) {           /* 8 s more */
            cc_tx(&a, o, 160);
            cc_rx(&a, quiet, 160);
        }
        check(a.lf_state == CC_LF_UI && ea->src_pos == 300,
              "after N200 retries a silent peer gets UI frames");
    }
    cc_release(&a);
    free(ea);
    free(eb);
}

/* ---- V.120 4.2.2 UI-only link verification ---- */

static int vf_xid_cmd, vf_xid_rsp, vf_ui_before_rsp, vf_seen_rsp;
static void vf_spy_frame(void *user, const uint8_t *pkt, int len, int ok)
{
    (void) user;
    if (len < 3 || !ok)
        return;
    if ((pkt[2] & ~0x10) == 0xAF) {
        if (pkt[0] & 0x02) {
            vf_xid_rsp++;
            vf_seen_rsp = 1;
        } else {
            vf_xid_cmd++;
        }
    } else if ((pkt[2] & ~0x10) == 0x03 && !vf_seen_rsp) {
        vf_ui_before_rsp++;
    }
}

static void test_v120_verify(void)
{
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    hdlc_rx_state_t *spy;

    printf("V.120 4.2.2 link verification (UI only):\n");
    ea->src = text_a; ea->src_len = 600;
    eb->src = text_b; eb->src_len = 400;
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_init_v120(&b, false, false, end_pull, end_push, eb);
    cc_v120_set_verify(&a, true);
    cc_v120_set_verify(&b, true);
    vf_xid_cmd = vf_xid_rsp = vf_ui_before_rsp = vf_seen_rsp = 0;
    spy = hdlc_rx_init(NULL, false, true, 1, vf_spy_frame, NULL);
    hdlc_rx_set_max_frame_len(spy, 400);
    ack_run(&a, &b, 40, 0, 0, 0, 0, spy);
    check(a.vf_state == 2 && b.vf_state == 2 && a.vf_gave_up == 0 && b.vf_gave_up == 0,
          "both ends verified by the XID exchange");
    check(vf_xid_cmd >= 1 && vf_xid_rsp >= 1, "XID command (C/R 0) and XID response (C/R 1) on the wire");
    check(eb->dst_len == 600 && ea->dst_len == 400 && !memcmp(eb->dst, text_a, 600)
          && !memcmp(ea->dst, text_b, 400), "data exact both ways after verification");
    check(vf_ui_before_rsp == 0, "no UI data frame from A before it saw B's XID response");
    hdlc_rx_free(spy);
    cc_release(&a);
    cc_release(&b);

    /* The peer does not verify itself but still answers (4.2.2 "shall"). */
    memset(ea, 0, sizeof(*ea));
    memset(eb, 0, sizeof(*eb));
    ea->src = text_a; ea->src_len = 300;
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_init_v120(&b, false, false, end_pull, end_push, eb);
    cc_v120_set_verify(&a, true);
    ack_run(&a, &b, 20, 0, 0, 0, 0, NULL);
    check(a.vf_state == 2 && a.vf_gave_up == 0 && eb->dst_len == 300, "a non-verifying peer answers the XID");
    cc_release(&a);
    cc_release(&b);

    /* Nobody answers: NM20 retransmissions, then data begins (4.2.2). */
    memset(ea, 0, sizeof(*ea));
    ea->src = text_a; ea->src_len = 200;
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_v120_set_verify(&a, true);
    {
        uint8_t o[160], quiet[160];

        memset(quiet, 0xFF, sizeof(quiet));
        for (int t = 0; t < 300; t++) {           /* 6 s: still waiting */
            cc_tx(&a, o, 160);
            cc_rx(&a, quiet, 160);
        }
        check(a.vf_state == 1 && ea->src_pos == 0, "no response: data held back, XID repeated");
        for (int t = 0; t < 300; t++) {           /* to 12 s: 3 retries x 2.5 s spent */
            cc_tx(&a, o, 160);
            cc_rx(&a, quiet, 160);
        }
        check(a.vf_state == 2 && a.vf_gave_up == 1 && ea->src_pos == 200,
              "NM20 spent: data begins anyway");
    }
    cc_release(&a);

    /* 4.2.3: an XID command while an SABME is outstanding is answered and the
     * state is kept. */
    memset(ea, 0, sizeof(*ea));
    memset(eb, 0, sizeof(*eb));
    ea->src = text_a; ea->src_len = 200;
    eb->src = text_b; eb->src_len = 200;
    cc_init_v120(&a, false, true, end_pull, end_push, ea);
    cc_init_v120(&b, false, false, end_pull, end_push, eb);
    cc_v120_set_ack(&a, true);
    cc_v120_set_ack(&b, true);
    {
        uint8_t xid[3] = { 0x08, 0x01, 0xAF }, pk[64];
        int n = 0;
        hdlc_tx_state_t *tx = hdlc_tx_init(NULL, false, 1, false, NULL, NULL);
        uint8_t line[64];

        hdlc_tx_flags(tx, 2);
        hdlc_tx_frame(tx, xid, 3);
        for (int i = 0; i < 40; i++) {
            uint8_t o = 0;

            for (int k = 0; k < 8; k++) {
                int bit = hdlc_tx_get_bit(tx);

                o = (uint8_t) (o | ((bit < 0 ? 1 : bit) << (7 - k)));
            }
            line[n++ % 64] = o;
        }
        hdlc_tx_free(tx);
        (void) pk;
        cc_tx(&a, (uint8_t[160]) {0}, 160);     /* A sends its SABME */
        check(a.lf_state == CC_LF_SETUP, "A has an SABME outstanding");
        cc_rx(&a, line, 40);                    /* an XID command arrives meanwhile */
        ack_run(&a, &b, 200, 0, 0, 0, 0, NULL);   /* the first SABME was dropped: T200 resends */
        check(a.lf_state == CC_LF_UP && b.lf_state == CC_LF_UP && ea->src_pos == 200,
              "XID during SABME did not disturb establishment (4.2.3)");
    }
    cc_release(&a);
    cc_release(&b);
    free(ea);
    free(eb);
}

static void test_v120_pair(bool r56)
{
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    hdlc_rx_state_t *spy, *spy_ba;
    char what[160];
    bool lsb;

    ea->src = text_a; ea->src_len = sizeof(text_a);
    eb->src = text_b; eb->src_len = sizeof(text_b);
    check(cc_init_v120(&a, r56, true, end_pull, end_push, ea) == 0
          && cc_init_v120(&b, r56, false, end_pull, end_push, eb) == 0, "V.120 init");
    /* Caller, LLI 256, C/R 0: 0x08 0x01; UI 0x03; header E|B|F 0x83. */
    memcpy(spy_want, "\x08\x01\x03\x83", 4);
    spy_frames = spy_bad = spy_hdr_ok = spy_max_len = 0;
    spy = hdlc_rx_init(NULL, false, true, 1, spy_frame, NULL);
    hdlc_rx_set_max_frame_len(spy, 400);
    spy_ba = hdlc_rx_init(NULL, false, true, 1, spy_frame, NULL);
    hdlc_rx_set_max_frame_len(spy_ba, 400);
    lsb = run_pair(&a, &b, 8000, r56, spy, spy_ba);

    snprintf(what, sizeof(what), "V.120 %s: %zu bytes A->B exact", r56 ? "56k" : "64k",
             sizeof(text_a));
    check(eb->dst_len == sizeof(text_a) && !memcmp(eb->dst, text_a, sizeof(text_a)), what);
    snprintf(what, sizeof(what), "V.120 %s: %zu bytes B->A exact", r56 ? "56k" : "64k",
             sizeof(text_b));
    check(ea->dst_len == sizeof(text_b) && !memcmp(ea->dst, text_b, sizeof(text_b)), what);
    snprintf(what, sizeof(what),
             "V.120 %s on the wire, both ways: %d frames, all 08 01 03 83, none bad, longest %d (4 + 256; FCS stripped)",
             r56 ? "56k" : "64k", spy_frames, spy_max_len);
    check(spy_frames >= 16 && spy_hdr_ok == spy_frames && spy_bad == 0
          && spy_max_len == 4 + CC_V120_MAX_DATA, what);
    check(a.rx_bad_frames == 0 && b.rx_bad_frames == 0
          && a.rx_unsupported == 0 && b.rx_unsupported == 0, "V.120: no bad or unsupported frames");
    if (r56)
        check(lsb, "V.120 56k: every octet both ways has bit 1 set");
    {
        uint8_t h[4];

        /* 6.2.2.3, Table 4: C/R is symmetric, UI a command (audit V120-1). */
        cc_v120_frame_header(&b, h);
        check(!memcmp(h, "\x08\x01\x03\x83", 4), "V.120 answerer's UI is a command too: C/R 0 (0x08 0x01)");
    }
    hdlc_rx_free(spy);
    hdlc_rx_free(spy_ba);
    cc_release(&a);
    cc_release(&b);
    free(ea);
    free(eb);
}

/* Frames from elsewhere, built independently with SpanDSP's HDLC
 * transmitter: control-state octets good and bad, I-frames, other logical
 * links, a corrupt FCS. */
static int put_frame(hdlc_tx_state_t *tx, uint8_t *line, int n, int upto,
                     const char *frame, int len)
{
    hdlc_tx_frame(tx, (const uint8_t *) frame, (size_t) len);
    while (n < upto)
        line[n++] = (uint8_t) hdlc_tx_get_byte(tx);
    return n;
}

static void test_v120_rx_variants(void)
{
    end_t *e = calloc(1, sizeof(*e));
    clear_channel_t rx;
    hdlc_tx_state_t *tx = hdlc_tx_init(NULL, false, 2, false, NULL, NULL);
    uint8_t line[3000];
    int n = 0;

    printf("V.120 receive variants:\n");
    cc_init_v120(&rx, false, true, end_pull, end_push, e);
    hdlc_tx_flags(tx, 2);   /* SpanDSP opens with no flag otherwise */
    /* E = 0 and one CS octet (E DR SR RR = 1), then "AB"; C/R 1 from a
     * pre-fix build is accepted. */
    n = put_frame(tx, line, n, 200, "\x0A\x01\x03\x03\xF0" "AB", 7);
    /* An I-frame: belongs to the acknowledged mode, must be dropped. */
    n = put_frame(tx, line, n, 400, "\x08\x01\x00\x00\x83" "XX", 7);
    /* UI with P set and BR: "CD", break counted. */
    n = put_frame(tx, line, n, 600, "\x08\x01\x13\xC3" "CD", 6);
    /* 3.1.2.1: a CS octet with E = 0 is an error -- nothing delivered,
     * neither "XY" nor the CS octet itself. */
    n = put_frame(tx, line, n, 800, "\x08\x01\x03\x03\x70" "XY", 7);
    /* H.E = 0 and no CS at all. */
    n = put_frame(tx, line, n, 1000, "\x08\x01\x03\x03", 4);
    /* LLI 257 and LLI 0 (in-channel signalling): not ours (Table 3). */
    n = put_frame(tx, line, n, 1200, "\x08\x03\x03\x83" "ZZ", 6);
    n = put_frame(tx, line, n, 1400, "\x00\x01\x03\x83" "QQ", 6);
    /* A good frame, then corrupt one bit inside it. */
    {
        int start = n;

        n = put_frame(tx, line, n, 1600, "\x08\x01\x03\x83" "EF", 6);
        line[start + 4] ^= 0x10;
    }
    cc_rx(&rx, line, n);
    check(e->dst_len == 4 && !memcmp(e->dst, "ABCD", 4),
          "CS octet taken off, I-frame dropped, P bit accepted: \"ABCD\" only");
    check(rx.rx_unsupported == 1, "the I-frame counted as unsupported");
    check(rx.rx_breaks == 1, "BR counted");
    check(rx.rx_bad_frames == 3,
          "bad: CS with E = 0, H.E = 0 with no CS, corrupt FCS (audit V120-2)");
    check(rx.rx_other_lli == 2, "LLI 257 and LLI 0 kept off the DTE (audit V120-3)");
    hdlc_tx_free(tx);
    cc_release(&rx);
    free(e);
}

/* 3.2.4.1: a CS with RR = 0 stops our user data; RR = 1 resumes it. */
static int rr_frames, rr_data;

static void rr_spy(void *user_data, const uint8_t *pkt, int len, int ok)
{
    (void) user_data;
    (void) pkt;
    if (len < 0 || !ok)
        return;
    rr_frames++;
    if (len > 4)
        rr_data += len - 4;
}

static void test_v120_rr(void)
{
    end_t *e = calloc(1, sizeof(*e));
    clear_channel_t cc;
    hdlc_tx_state_t *peer = hdlc_tx_init(NULL, false, 2, false, NULL, NULL);
    hdlc_rx_state_t *spy = hdlc_rx_init(NULL, false, true, 1, rr_spy, NULL);
    uint8_t line[400], out[160];
    int n = 0, data_while_off;

    printf("V.120 RR flow control:\n");
    e->src = text_a;
    e->src_len = sizeof(text_a);
    cc_init_v120(&cc, false, true, end_pull, end_push, e);
    hdlc_rx_set_max_frame_len(spy, 400);
    hdlc_tx_flags(peer, 2);
    n = put_frame(peer, line, n, 200, "\x08\x01\x03\x03\xE0", 5);   /* RR = 0 */
    cc_rx(&cc, line, n);
    check(!cc.v120_peer_rr, "RR(R) = 0 after the peer's CS");
    rr_frames = rr_data = 0;
    for (int t = 0; t < 20; t++) {
        cc_tx(&cc, out, 160);
        for (int i = 0; i < 160; i++)
            for (int k = 0; k < 8; k++)
                hdlc_rx_put_bit(spy, (out[i] >> (7 - k)) & 1);
    }
    /* At most the frame already queued before the CS arrived goes out. */
    data_while_off = rr_data;
    check(data_while_off <= CC_V120_MAX_DATA && e->src_pos <= CC_V120_MAX_DATA,
          "flow controlled: no new user data queued");
    n = put_frame(peer, line, 0, 200, "\x08\x01\x03\x03\xF0", 5);   /* RR = 1 */
    cc_rx(&cc, line, n);
    for (int t = 0; t < 40; t++) {
        cc_tx(&cc, out, 160);
        for (int i = 0; i < 160; i++)
            for (int k = 0; k < 8; k++)
                hdlc_rx_put_bit(spy, (out[i] >> (7 - k)) & 1);
    }
    check(cc.v120_peer_rr && rr_data > data_while_off + 1000,
          "RR = 1: user data flows again");
    hdlc_tx_free(peer);
    hdlc_rx_free(spy);
    cc_release(&cc);
    free(e);
}

/* ---------------------------------------------------------------- */
/* V.110                                                             */
/* ---------------------------------------------------------------- */

/* An independent V.110 wire checker, written from Table 2/Tables 6/Table 5
 * and I.460 rather than from clear_channel.c: it is fed one side's DS0
 * octets and grades every 80-bit frame it finds. */
typedef struct {
    int ir_bits, rep, e123, ra0;
    uint8_t win[80];
    int n, aligned;
    int frames, bad_frames, unused_bits_bad, e_bad, e7_zero, rep_bad;
    int s_off_frames, s_on_frames, x_off_frames;
    int first_on;          /* frame index of first S = X = ON, or -1 */
} v110_spy_t;

static void v110_spy_init(v110_spy_t *w, int ra0)
{
    memset(w, 0, sizeof(*w));
    w->ra0 = ra0;
    w->ir_bits = ra0 <= 4800 ? 1 : ra0 == 9600 ? 2 : ra0 == 19200 ? 4 : 8;
    w->rep = ra0 == 600 ? 8 : ra0 == 1200 ? 4 : ra0 == 2400 ? 2 : 1;
    w->e123 = ra0 == 600 ? 4 : ra0 == 1200 ? 2 : ra0 == 2400 ? 6 : 3;
    w->first_on = -1;
}

static int spy_align(const uint8_t *f)
{
    for (int i = 0; i < 8; i++)
        if (f[i])
            return 0;
    for (int o = 1; o < 10; o++)
        if (!f[8 * o])
            return 0;
    return 1;
}

static void v110_spy_frame(v110_spy_t *w, const uint8_t *f)
{
    int s0 = 0, slot = 0;

    w->frames++;
    if (!spy_align(f)) {
        w->bad_frames++;
        return;
    }
    if (((f[41] << 2) | (f[42] << 1) | f[43]) != w->e123 || !f[44] || !f[45] || !f[46])
        w->e_bad++;
    if (!f[47])
        w->e7_zero++;
    {
        uint8_t sl[48];

        for (int o = 1; o < 10; o++) {
            if (o == 5)
                continue;
            for (int b = 1; b <= 6; b++)
                sl[slot++] = f[o * 8 + b];
        }
        for (int i = 0; i < 48; i++)
            if (i % w->rep && sl[i] != sl[i - 1])
                w->rep_bad++;
    }
    s0 = !f[15] + !f[31] + !f[39] + !f[55] + !f[71] + !f[79];
    if (s0 == 6 && !f[23] && !f[63]) {
        w->s_on_frames++;
        if (w->first_on < 0)
            w->first_on = w->frames;
    } else if (s0 == 0) {
        w->s_off_frames++;
        if (f[23] && f[63])
            w->x_off_frames++;
    }
}

static void v110_spy_octets(v110_spy_t *w, const uint8_t *o, int n)
{
    for (int i = 0; i < n; i++) {
        uint8_t unused = (uint8_t) (0xFF >> w->ir_bits);

        if ((o[i] & unused) != unused)
            w->unused_bits_bad++;
        for (int b = 0; b < w->ir_bits; b++) {
            int bit = (o[i] >> (7 - b)) & 1;

            if (!w->aligned) {
                if (w->n < 80)
                    w->win[w->n++] = (uint8_t) bit;
                else {
                    memmove(w->win, w->win + 1, 79);
                    w->win[79] = (uint8_t) bit;
                }
                if (w->n == 80 && spy_align(w->win)) {
                    w->aligned = 1;
                    v110_spy_frame(w, w->win);
                    w->n = 0;
                }
                continue;
            }
            w->win[w->n++] = (uint8_t) bit;
            if (w->n == 80) {
                v110_spy_frame(w, w->win);
                w->n = 0;
            }
        }
    }
}

static uint8_t v110_text[3000];

/* Run a pair for `octets`; B's transmitter joins `b_late` octets after A's
 * (A hears idle ones, 7.1.1.2, until then).  `gap` (if >0) replaces what
 * B hears with ones for that many octets starting at `gap_at`. */
static void v110_run(clear_channel_t *a, clear_channel_t *b, int octets,
                     int b_late, int gap_at, int gap, v110_spy_t *spy_a)
{
    uint8_t ab[160], ba[160];

    for (int done = 0; done < octets; done += 160) {
        cc_tx(a, ab, 160);
        if (done >= b_late)
            cc_tx(b, ba, 160);
        else
            memset(ba, 0xFF, sizeof(ba));
        if (spy_a)
            v110_spy_octets(spy_a, ab, 160);
        if (gap > 0 && done >= gap_at && done < gap_at + gap)
            memset(ab, 0xFF, sizeof(ab));
        if (done >= b_late)
            cc_rx(b, ab, 160);
        cc_rx(a, ba, 160);
    }
}

static void test_v110_pair(int rate)
{
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    v110_spy_t spy;
    char what[200];
    size_t na, nb;
    int octets;

    na = (size_t) (rate / 10 * 3 / 2);
    if (na < 12)
        na = 12;
    if (na > sizeof(v110_text))
        na = sizeof(v110_text);
    nb = na * 2 / 3;
    ea->src = v110_text; ea->src_len = na;
    eb->src = v110_text + 7; eb->src_len = nb;
    check(cc_init_v110(&a, rate, end_pull, end_push, ea) == 0
          && cc_init_v110(&b, rate, end_pull, end_push, eb) == 0, "V.110 init");
    v110_spy_init(&spy, cc_v110_ra0_rate(rate));
    /* Characters at 10 elements each, plus padding and a second of start. */
    octets = (int) (8000.0 * ((double) na * 10.0 / rate * 1.6 + 1.0));
    v110_run(&a, &b, octets, 1937, 0, 0, &spy);

    snprintf(what, sizeof(what), "V.110 %5d (RA0 %5d, IR %2d k): connected both ends, %zu + %zu bytes exact",
             rate, a.v110_ra0_rate, a.v110_ir_bits * 8, na, nb);
    check(a.v110_state == CC_V110_CONNECTED && b.v110_state == CC_V110_CONNECTED
          && eb->dst_len == na && !memcmp(eb->dst, v110_text, na)
          && ea->dst_len == nb && !memcmp(ea->dst, v110_text + 7, nb), what);
    snprintf(what, sizeof(what),
             "       wire: %d frames aligned, E1-E3 per Table 5, unused bits 1, repeats equal, "
             "S=X=OFF first (%d) then ON (from %d)",
             spy.frames, spy.x_off_frames, spy.first_on);
    check(spy.frames > 20 && spy.bad_frames == 0 && spy.e_bad == 0
          && spy.unused_bits_bad == 0 && spy.rep_bad == 0
          && spy.x_off_frames > 0 && spy.first_on > spy.x_off_frames
          && spy.s_on_frames + spy.x_off_frames == spy.frames
          && a.v110_frame_errors == 0 && b.v110_frame_errors == 0
          && a.v110_rate_mismatch == 0, what);
    if (cc_v110_ra0_rate(rate) == 600)
        check(spy.e7_zero == spy.frames / 4 || spy.e7_zero == spy.frames / 4 + 1,
              "       600: E7 = 0 in every fourth frame (Table 5 Note 2)");
    else if (rate == 38400)
        check(spy.e7_zero == 0, "       E7 = 1 above 600 bit/s");
    cc_release(&a);
    cc_release(&b);
    free(ea);
    free(eb);
}

static void test_v110_procedures(void)
{
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    uint8_t ab[160], ba[160];

    printf("V.110 clause 7 procedures:\n");

    /* 7.1.4: A's disconnect request; B sees S OFF and D = 0. */
    cc_init_v110(&a, 9600, end_pull, end_push, ea);
    cc_init_v110(&b, 9600, end_pull, end_push, eb);
    v110_run(&a, &b, 8000, 0, 0, 0, NULL);
    cc_v110_disconnect(&a);
    v110_run(&a, &b, 1600, 0, 0, 0, NULL);
    check(b.v110_state == CC_V110_DOWN && b.v110_cause == CC_V110_CAUSE_REMOTE
          && cc_v110_finished(&b), "7.1.4.2: far end's disconnect request seen");
    check(a.v110_state == CC_V110_DOWN && a.v110_cause == CC_V110_CAUSE_LOCAL
          && cc_v110_finished(&a), "7.1.4.3: our request acknowledged by S OFF");

    /* T1: nobody there. */
    cc_init_v110(&a, 38400, end_pull, end_push, ea);
    memset(ba, 0xFF, sizeof(ba));
    for (int t = 0; t < 480; t++) {          /* 9.6 s at 8000 octets/s */
        cc_tx(&a, ab, 160);
        cc_rx(&a, ba, 160);
    }
    check(a.v110_state == CC_V110_SEARCH, "T1: still searching at 9.6 s");
    for (int t = 0; t < 30; t++) {
        cc_tx(&a, ab, 160);
        cc_rx(&a, ba, 160);
    }
    check(a.v110_state == CC_V110_DOWN && a.v110_cause == CC_V110_CAUSE_T1
          && cc_v110_finished(&a), "T1 = 10 s: disconnect (7.1.2.4)");

    /* 7.1.5: B loses A's framing for 0.4 s and recovers. */
    memset(ea, 0, sizeof(*ea));
    memset(eb, 0, sizeof(*eb));
    ea->src = v110_text;
    ea->src_len = 400;
    cc_init_v110(&a, 9600, end_pull, end_push, ea);
    cc_init_v110(&b, 9600, end_pull, end_push, eb);
    v110_run(&a, &b, 8000, 0, 0, 0, NULL);
    check(eb->dst_len == 400 && !memcmp(eb->dst, v110_text, 400), "before: 400 bytes exact");
    ea->src = v110_text + 400;
    ea->src_len = 1000;
    ea->src_pos = 0;
    eb->dst_len = 0;
    {
        size_t pulled_at_gap_end;

        v110_run(&a, &b, 3200, 0, 0, 3200, NULL);
        pulled_at_gap_end = ea->src_pos;
        check(b.v110_sync_losses == 1 && !b.v110_synced && b.v110_state == CC_V110_CONNECTED,
              "three bad frames: B has lost sync, still connected (7.1.5 Note 2)");
        v110_run(&a, &b, 1600, 0, 0, 1600, NULL);
        check(ea->src_pos == pulled_at_gap_end,
              "B's X OFF holds A's data back (7.1.5 c): nothing pulled meanwhile");
    }
    v110_run(&a, &b, 16000, 0, 0, 0, NULL);
    check(b.v110_synced && a.v110_state == CC_V110_CONNECTED && b.v110_state == CC_V110_CONNECTED
          && ea->src_pos == 1000 && eb->dst_len >= 900
          && !memcmp(eb->dst + eb->dst_len - 500, v110_text + 900, 500),
          "resynchronized, X ON, data flows again (7.1.5 f/g)");

    /* 7.1.5 e): not recovered in 3 s. */
    v110_run(&a, &b, 8000 * 4, 0, 0, 8000 * 4, NULL);
    check(b.v110_state == CC_V110_DOWN && b.v110_cause == CC_V110_CAUSE_SYNC_LOST
          && cc_v110_finished(&b), "no framing for 3 s: B disconnects (7.1.5 e)");
    check(a.v110_state == CC_V110_DOWN && a.v110_cause == CC_V110_CAUSE_REMOTE,
          "and A reads B's all-OFF, D = 0 frames as a disconnect request");
    cc_release(&a);
    cc_release(&b);
    free(ea);
    free(eb);
}

/* A far end that keeps S = X = ON and D = 1 whatever it hears: Table 6e
 * frames at 9600 bit/s (IR 16 kbit/s, two bits an octet) built here. */
static int v110_enc_9600_on(uint8_t *line, int octets)
{
    uint8_t f[80];
    int n = 0;

    memset(f, 1, 80);
    memset(f, 0, 8);
    for (int o = 1; o < 10; o++)
        if (o != 5)
            f[o * 8 + 7] = 0;            /* S and X ON */
    f[41] = 0; f[42] = 1; f[43] = 1;     /* E1-E3, Table 5 */
    for (int i = 0; n < octets; i += 2)
        line[n++] = (uint8_t) (0x3F | (f[i % 80] << 7) | (f[(i + 1) % 80] << 6));
    return n;
}

static int g_room_free = 16000, g_room_size = 16000;
static void test_room(void *ctx, int *fr, int *sz)
{
    (void)ctx;
    *fr = g_room_free;
    *sz = g_room_size;
}

/* 5.4.2: our own receive buffer filling turns X OFF towards the far end. */
static void test_v110_flow(void)
{
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    size_t at;

    printf("V.110 5.4.2 flow control towards the far end:\n");
    ea->src = v110_text;
    ea->src_len = 1400;
    cc_init_v110(&a, 9600, end_pull, end_push, ea);
    cc_init_v110(&b, 9600, end_pull, end_push, eb);
    cc_v110_set_rx_room(&b, test_room);
    g_room_free = 16000;
    v110_run(&a, &b, 3200, 0, 0, 0, NULL);       /* connect, some data */
    check(a.v110_state == CC_V110_CONNECTED && b.v110_state == CC_V110_CONNECTED
          && !b.v110_rx_hold, "connected, buffer empty: X stays ON");
    g_room_free = 1000;                           /* under a quarter free */
    v110_run(&a, &b, 800, 0, 0, 0, NULL);        /* let in-flight drain */
    at = ea->src_pos;
    v110_run(&a, &b, 3200, 0, 0, 0, NULL);
    check(b.v110_rx_hold && b.v110_flow_holds == 1 && ea->src_pos == at
          && ea->src_pos < 1400, "buffer nearly full: B's X OFF stops A's data");
    g_room_free = 5000;                           /* over a quarter, under half */
    v110_run(&a, &b, 1600, 0, 0, 0, NULL);
    check(b.v110_rx_hold && ea->src_pos == at, "hysteresis: still held below half free");
    g_room_free = 12000;
    v110_run(&a, &b, 16000, 0, 0, 0, NULL);
    check(!b.v110_rx_hold && ea->src_pos == 1400 && eb->dst_len == 1400
          && !memcmp(eb->dst, v110_text, 1400),
          "drained: X back ON, the rest arrives, nothing lost or repeated");
    check(b.v110_sync_losses == 0 && a.v110_state == CC_V110_CONNECTED,
          "flow control is not a loss of framing");
    cc_release(&a);
    cc_release(&b);
    free(ea);
    free(eb);
}

/* ---- V.110 synchronous user data ---- */

/* Independent decode of the D bits of one wire frame, from Tables 6a-6f as
 * printed (not from clear_channel.c's maps).  Returns the number of D bits. */
static int sync_wire_d(const uint8_t *f, int rate, uint8_t *d)
{
    static const char *t6d[8] = { "123456", "789aFF", "bcFFde", "FFfghi",
                                  "jklmno", "pqrsFF", "tuFFvw", "FFxyzA" };
    static const char *t6f[8] = { "123456", "789aFF", "bcFFde", "FFfFFF",
                                  "ghijkl", "mnopFF", "qrFFst", "FFuFFF" };
    int slot = 0, n = 0, rows[8] = { 1, 2, 3, 4, 6, 7, 8, 9 };

    for (int k = 0; k < 8; k++)
        for (int b = 1; b <= 6; b++, slot++) {
            int bit = f[rows[k] * 8 + b];

            if (rate == 600 || rate == 1200 || rate == 2400) {
                int rep = rate == 600 ? 8 : rate == 1200 ? 4 : 2;

                if (slot % rep == 0)
                    d[n++] = (uint8_t) bit;
            } else if (rate == 7200 || rate == 14400 || rate == 28800) {
                if (t6d[k][b - 1] != 'F')
                    d[n++] = (uint8_t) bit;
            } else if (rate == 12000 || rate == 24000) {
                if (t6f[k][b - 1] != 'F')
                    d[n++] = (uint8_t) bit;
            } else {
                d[n++] = (uint8_t) bit;
            }
        }
    return n;
}

typedef struct {
    int rate, ir_bits, e123;
    uint8_t win[80];
    int n, aligned, frames, bad, e_bad, unused_bad;
    uint8_t stream[200000];
    size_t slen;
    bool on;                    /* S = X = ON seen */
} sync_spy_t;

static void sync_spy_octets(sync_spy_t *w, const uint8_t *o, int n)
{
    for (int i = 0; i < n; i++) {
        uint8_t unused = (uint8_t) (0xFF >> w->ir_bits);

        if ((o[i] & unused) != unused)
            w->unused_bad++;
        for (int b = 0; b < w->ir_bits; b++) {
            int bit = (o[i] >> (7 - b)) & 1;

            if (!w->aligned) {
                if (w->n < 80)
                    w->win[w->n++] = (uint8_t) bit;
                else {
                    memmove(w->win, w->win + 1, 79);
                    w->win[79] = (uint8_t) bit;
                }
                if (w->n == 80 && spy_align(w->win))
                    w->aligned = 1;
                else
                    continue;
            } else {
                w->win[w->n++] = (uint8_t) bit;
                if (w->n < 80)
                    continue;
            }
            w->n = 0;
            w->frames++;
            if (!spy_align(w->win)) {
                w->bad++;
                continue;
            }
            if (((w->win[41] << 2) | (w->win[42] << 1) | w->win[43]) != w->e123
                || !w->win[44] || !w->win[45] || !w->win[46])
                w->e_bad++;
            if (!w->win[15] && !w->win[23])
                w->on = true;
            if (w->on && w->slen + 48 < sizeof(w->stream))
                w->slen += (size_t) sync_wire_d(w->win, w->rate, w->stream + w->slen);
        }
    }
}

/* Find 1 0^8 1^8 in the wire's D stream; the octets after it, LSB first. */
static int sync_wire_octets(const sync_spy_t *w, uint8_t *out, int max)
{
    for (size_t i = 0; i + 17 < w->slen; i++) {
        int ok = w->stream[i] == 1;

        for (int k = 1; ok && k <= 8; k++)
            ok = w->stream[i + k] == 0;
        for (int k = 9; ok && k <= 16; k++)
            ok = w->stream[i + k] == 1;
        if (!ok)
            continue;
        {
            int n = 0;

            for (size_t p = i + 17; p + 8 <= w->slen && n < max; p += 8, n++) {
                uint8_t v = 0;

                for (int k = 0; k < 8; k++)
                    v = (uint8_t) (v | (w->stream[p + k] << k));
                out[n] = v;
            }
            return n;
        }
    }
    return -1;
}

/* Sync data is a continuous stream: the far end's idle (0xFF fill) reaches the
 * DTE too.  The text used here has no 0xFF, so dropping them recovers it. */
static size_t sync_strip_idle(const uint8_t *in, size_t n, uint8_t *out)
{
    size_t m = 0;

    for (size_t i = 0; i < n; i++)
        if (in[i] != 0xFF)
            out[m++] = in[i];
    return m;
}

static uint8_t sync_text[3200];

static void test_v110_sync_rate(int rate, int gap_test)
{
    static const struct { int rate, ir, e123; } tab[] = {
        { 600, 1, 4 }, { 1200, 1, 2 }, { 2400, 1, 6 }, { 4800, 1, 3 }, { 7200, 2, 5 },
        { 9600, 2, 3 }, { 12000, 4, 1 }, { 14400, 4, 5 }, { 19200, 4, 3 }, { 24000, 8, 1 },
        { 28800, 8, 5 }, { 38400, 8, 3 }
    };
    static sync_spy_t spy;
    end_t *ea = calloc(1, sizeof(*ea)), *eb = calloc(1, sizeof(*eb));
    clear_channel_t a, b;
    char what[200];
    size_t na, nb;
    int octets, ti = 0;
    static uint8_t wire[4096];

    while (tab[ti].rate != rate)
        ti++;
    na = (size_t) (gap_test ? rate / 4 : rate / 20);
    if (na < 24)
        na = 24;
    if (na > 3000)
        na = 3000;
    nb = na * 2 / 3;
    ea->src = sync_text; ea->src_len = na;
    eb->src = sync_text + 11; eb->src_len = nb;
    check(cc_init_v110(&a, rate, end_pull, end_push, ea) == 0
          && cc_init_v110(&b, rate, end_pull, end_push, eb) == 0
          && cc_v110_set_sync(&a) == 0 && cc_v110_set_sync(&b) == 0, "V.110 sync init");
    memset(&spy, 0, sizeof(spy));
    spy.rate = rate; spy.ir_bits = tab[ti].ir; spy.e123 = tab[ti].e123;
    octets = (int) (8000.0 * ((double) na * 8.0 / rate * 1.5 + (gap_test ? 3.5 : 1.5)));
    {
        uint8_t ab[160], ba[160];

        for (int done = 0; done < octets; done += 160) {
            cc_tx(&a, ab, 160);
            cc_tx(&b, ba, 160);
            sync_spy_octets(&spy, ab, 160);
            if (gap_test && done >= 3200 && done < 3200 + 3200)   /* 0.4 s of nothing */
                memset(ab, 0xFF, sizeof(ab));
            cc_rx(&b, ab, 160);
            cc_rx(&a, ba, 160);
        }
    }
    if (!gap_test) {
        int nw;
        static uint8_t sa[8192], sb[8192];
        size_t la = sync_strip_idle(eb->dst, eb->dst_len, sa), lb = sync_strip_idle(ea->dst, ea->dst_len, sb);

        snprintf(what, sizeof(what), "V.110 sync %5d (IR %2d k): %zu + %zu octets exact, connected",
                 rate, tab[ti].ir * 8, na, nb);
        check(a.v110_state == CC_V110_CONNECTED && b.v110_state == CC_V110_CONNECTED
              && la == na && !memcmp(sa, sync_text, na)
              && lb == nb && !memcmp(sb, sync_text + 11, nb), what);
        check(spy.frames > 20 && spy.bad == 0 && spy.e_bad == 0 && spy.unused_bad == 0
              && a.v110_frame_errors == 0 && a.v110_rate_mismatch == 0,
              "       wire: frames aligned, E1-E3 per Table 5, unused bits 1");
        nw = sync_wire_octets(&spy, wire, (int) na);
        check(nw == (int) na && !memcmp(wire, sync_text, na),
              "       wire: 1 0^8 1^8 opens the stream and the octets after it are the DTE's, LSB first");
    } else {
        snprintf(what, sizeof(what),
                 "V.110 sync %5d, 0.4 s loss of framing: tail of the stream still octet-aligned", rate);
        static uint8_t sa[8192];
        size_t la = sync_strip_idle(eb->dst, eb->dst_len, sa);

        check(b.v110_sync_losses >= 1 && b.sy_rx_gaps >= 1 && la >= 60 && la < na
              && !memcmp(sa + la - 60, sync_text + na - 60, 60), what);
    }
    cc_release(&a);
    cc_release(&b);
    free(ea);
    free(eb);
}

static void test_v110_sync(void)
{
    for (size_t i = 0; i < sizeof(sync_text); i++)
        sync_text[i] = (uint8_t) (0x20 + (i * 7 + i / 13) % 95);

    static const int rates[] = { 600, 1200, 2400, 4800, 7200, 9600, 12000, 14400, 19200,
                                 24000, 28800, 38400 };

    printf("V.110 synchronous (5.1, Tables 6a-6f):\n");
    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
        test_v110_sync_rate(rates[i], 0);
    {
        end_t *e = calloc(1, sizeof(*e));
        clear_channel_t c;

        check(cc_init_v110(&c, 110, end_pull, end_push, e) == 0 && cc_v110_set_sync(&c) == -1,
              "an asynchronous-only rate (110) is refused for synchronous");
        cc_release(&c);
        free(e);
    }
    for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
        if (rates[i] != 4800 && rates[i] != 9600 && rates[i] != 19200 && rates[i] != 38400
            && rates[i] != 2400)
            test_v110_sync_rate(rates[i], 1);
}

static void test_v110_t2(void)
{
    end_t *e = calloc(1, sizeof(*e));
    clear_channel_t a;
    uint8_t out[160], in[160];
    size_t pulled;

    printf("V.110 local disconnect, unanswered:\n");
    e->src = v110_text;
    e->src_len = sizeof(v110_text);
    cc_init_v110(&a, 9600, end_pull, end_push, e);
    for (int t = 0; t < 50; t++) {               /* 1 s */
        cc_tx(&a, out, 160);
        v110_enc_9600_on(in, 160);
        cc_rx(&a, in, 160);
    }
    check(a.v110_state == CC_V110_CONNECTED && e->src_pos > 0, "connected, data flowing");
    cc_v110_disconnect(&a);
    pulled = e->src_pos;
    for (int t = 0; t < 240; t++) {              /* 4.8 s */
        cc_tx(&a, out, 160);
        v110_enc_9600_on(in, 160);
        cc_rx(&a, in, 160);
    }
    check(a.v110_state == CC_V110_DISCONNECTING && e->src_pos == pulled,
          "far end keeps S ON: still disconnecting at 4.8 s, nothing more pulled (106 OFF)");
    for (int t = 0; t < 20; t++) {
        cc_tx(&a, out, 160);
        v110_enc_9600_on(in, 160);
        cc_rx(&a, in, 160);
    }
    check(a.v110_state == CC_V110_DOWN && a.v110_cause == CC_V110_CAUSE_T2
          && cc_v110_finished(&a), "T2 = 5 s: given up (7.1.4.1)");
    cc_release(&a);
    free(e);
}

/* RA0 receive, fed by an independent Table 6e encoder (9600 bit/s, IR
 * 16 kbit/s): a deleted stop element (5.3.4), a NUL, and a break (5.3.5). */
static uint8_t enc_bits[20000];
static int enc_n;

static void enc_char(int c, int stops)
{
    enc_bits[enc_n++] = 0;
    for (int i = 0; i < 8; i++)
        enc_bits[enc_n++] = (uint8_t) ((c >> i) & 1);
    while (stops-- > 0)
        enc_bits[enc_n++] = 1;
}

static void test_v110_ra0_rx(void)
{
    end_t *e = calloc(1, sizeof(*e));
    clear_channel_t rx;
    uint8_t line[8000];
    int nl = 0, d = 0;
    uint8_t ir[64000];
    int ni = 0;

    printf("V.110 RA0 receive (independent encoder):\n");
    enc_n = 0;
    for (int i = 0; i < 500; i++)
        enc_bits[enc_n++] = 1;
    enc_char('H', 0);              /* stop element deleted */
    enc_char('i', 1);
    enc_char(0, 1);                /* a real NUL */
    for (int i = 0; i < 23; i++)   /* 2M + 3 of start polarity */
        enc_bits[enc_n++] = 0;
    for (int i = 0; i < 25; i++)
        enc_bits[enc_n++] = 1;
    enc_char('!', 1);
    while (enc_n < 4800)
        enc_bits[enc_n++] = 1;
    /* Frames: S = X = ON throughout, Table 6e, E1-E3 = 011. */
    while (d < enc_n) {
        uint8_t f[80];

        memset(f, 1, 80);
        memset(f, 0, 8);
        for (int o = 1; o < 10; o++) {
            if (o == 5)
                continue;
            for (int b = 1; b <= 6; b++)
                f[o * 8 + b] = enc_bits[d++];
            f[o * 8 + 7] = 0;
        }
        f[41] = 0; f[42] = 1; f[43] = 1;
        memcpy(ir + ni, f, 80);
        ni += 80;
    }
    for (int i = 0; i + 1 < ni; i += 2)
        line[nl++] = (uint8_t) (0x3F | (ir[i] << 7) | (ir[i + 1] << 6));
    cc_init_v110(&rx, 9600, end_pull, end_push, e);
    cc_rx(&rx, line, nl);
    check(rx.v110_state == CC_V110_CONNECTED, "connected on the encoder's S = X = ON");
    check(e->dst_len == 4 && !memcmp(e->dst, "Hi\0!", 4),
          "\"Hi\", NUL, \"!\": deleted stop re-inserted, break not read as NULs");
    check(rx.rx_breaks == 1, "the break counted");
    cc_release(&rx);
    free(e);
}

/* ---------------------------------------------------------------- */
/* The engine, through the PTY                                       */
/* ---------------------------------------------------------------- */

static int dte_fd = -1;

static void drain(char *out, size_t max, int timeout_ms)
{
    size_t used = 0;

    out[0] = '\0';
    for (int waited = 0; waited < timeout_ms; waited += 10) {
        char buf[512];
        int n = (int) read(dte_fd, buf, sizeof(buf));

        if (n > 0 && used + (size_t) n < max) {
            memcpy(out + used, buf, (size_t) n);
            used += (size_t) n;
            out[used] = '\0';
        } else if (n <= 0) {
            usleep(10000);
        }
    }
}

static void expect(const char *cmd, const char *want)
{
    char line[128], resp[2048];
    int n = snprintf(line, sizeof(line), "%s\r", cmd);

    if (write(dte_fd, line, (size_t) n) != n)
        perror("write");
    drain(resp, sizeof(resp), 300);
    if (strstr(resp, want)) {
        printf("  ok   %-26s -> %s\n", cmd, want);
    } else {
        printf("  FAIL %-26s -> wanted \"%s\", got \"%s\"\n", cmd, want, resp);
        failures++;
    }
}

/* Connect, check CONNECT, type text, loop the engine's DS0 onto itself, and
 * require the text back on the PTY.  Returns whether every TX octet had
 * bit 1 set. */
static bool engine_loop(const char *connect, const char *what)
{
    static const char msg[] = "loop through the DS0 0123456789 abcdefghijklmnopqrstuvwxyz\r\n";
    char resp[4096];
    uint8_t ds0[160];
    bool lsb_set = true;
    char line[160];

    me_on_sip_connected();
    drain(resp, sizeof(resp), 300);
    snprintf(line, sizeof(line), "%s: %s, no V.8", what, connect);
    check(strstr(resp, connect) != NULL && me_get_state() == ME_DATA, line);

    if (write(dte_fd, msg, sizeof(msg) - 1) != (ssize_t) (sizeof(msg) - 1))
        perror("write");
    usleep(100000);
    resp[0] = '\0';
    {
        size_t used = 0;

        for (int tick = 0; tick < 100; tick++) {
            uint8_t dte[256];
            int n;

            while (me_put_space() > 0 && (n = di_read_data(dte, sizeof(dte))) > 0)
                me_put_data(dte, n);
            me_tx_g711(ds0, 160);
            for (int i = 0; i < 160; i++)
                lsb_set = lsb_set && (ds0[i] & 1);
            me_rx_g711(ds0, 160);
            while ((n = (int) read(dte_fd, resp + used, sizeof(resp) - 1 - used)) > 0)
                used += (size_t) n;
            resp[used] = '\0';
        }
    }
    snprintf(line, sizeof(line), "%s: PTY text looped through the engine's DS0", what);
    check(strstr(resp, msg) != NULL, line);
    me_on_sip_disconnected();
    drain(resp, sizeof(resp), 300);
    return lsb_set;
}

/* V.110 through the engine, its DS0 looped onto itself: the far end it
 * meets is its own transmitter, so clause 7.1's S/X exchange must complete
 * before CONNECT, which therefore arrives after the call, not with it. */
static void engine_loop_v110(const char *connect)
{
    static const char msg[] = "V.110 through the engine's DS0 0123456789 abcdefghijklmnopqrstuvwxyz\r\n";
    char resp[4096];
    uint8_t ds0[160];
    size_t used = 0;
    bool sent = false, early;
    char line[160];

    me_on_sip_connected();
    drain(resp, sizeof(resp), 200);
    early = strstr(resp, "CONNECT") != NULL;
    resp[0] = '\0';
    for (int tick = 0; tick < 150; tick++) {
        uint8_t dte[256];
        int n;

        if (!sent && strstr(resp, connect)) {
            if (write(dte_fd, msg, sizeof(msg) - 1) != (ssize_t) (sizeof(msg) - 1))
                perror("write");
            usleep(50000);
            sent = true;
        }
        while (me_put_space() > 0 && (n = di_read_data(dte, sizeof(dte))) > 0)
            me_put_data(dte, n);
        me_tx_g711(ds0, 160);
        me_rx_g711(ds0, 160);
        while ((n = (int) read(dte_fd, resp + used, sizeof(resp) - 1 - used)) > 0)
            used += (size_t) n;
        resp[used] = '\0';
    }
    snprintf(line, sizeof(line), "V.110: no CONNECT with the call, %s after S = X = ON", connect);
    check(!early && strstr(resp, connect) != NULL, line);
    check(strstr(resp, msg) != NULL, "V.110: PTY text looped through the engine's DS0");
    me_on_sip_disconnected();
    drain(resp, sizeof(resp), 300);
}

/* ATH on a V.110 call (me_hangup(), the DTE's hang-up callback): the SIP
 * call stays up while 7.1.4.1's request goes out, and ends once it is
 * acknowledged -- here by our own S OFF coming back round the loop. */
static void engine_v110_ath(void)
{
    char resp[2048];
    uint8_t ds0[160];
    int ticks = 0;
    bool held;

    me_on_sip_connected();
    for (int t = 0; t < 25; t++) {
        me_tx_g711(ds0, 160);
        me_rx_g711(ds0, 160);
    }
    drain(resp, sizeof(resp), 200);
    for (int t = 0; t < 50; t++) {      /* past a periodic link report */
        me_tx_g711(ds0, 160);
        me_rx_g711(ds0, 160);
    }
    check(strstr(resp, "CONNECT 9600") != NULL && me_get_state() == ME_DATA,
          "V.110 ATH: connected first, and a bounded 9600 call stays up "
          "(the link report used the previous call's rate)");
    me_hangup();
    held = me_get_state() == ME_DATA;
    while (me_get_state() == ME_DATA && ticks < 50) {
        me_tx_g711(ds0, 160);
        me_rx_g711(ds0, 160);
        ticks++;
    }
    check(held && me_get_state() == ME_HANGUP && ticks >= 1 && ticks < 10,
          "V.110 ATH: call held for the 7.1.4.1 request, then hung up on its acknowledgement");
    me_on_sip_disconnected();
    drain(resp, sizeof(resp), 300);

    /* The same through the PTY: escape, ATH, and the DTE is owed OK, not
     * NO CARRIER, once the request has been acknowledged. */
    me_on_sip_connected();
    for (int t = 0; t < 25; t++) {
        me_tx_g711(ds0, 160);
        me_rx_g711(ds0, 160);
    }
    drain(resp, sizeof(resp), 1100);            /* V.250 guard time */
    if (write(dte_fd, "+++", 3) != 3)
        perror("write");
    drain(resp, sizeof(resp), 1300);
    if (write(dte_fd, "ATH\r", 4) != 4)
        perror("write");
    usleep(200000);
    held = me_get_state() == ME_DATA;
    for (ticks = 0; me_get_state() == ME_DATA && ticks < 50; ticks++) {
        me_tx_g711(ds0, 160);
        me_rx_g711(ds0, 160);
    }
    if (me_get_state() == ME_HANGUP)
        me_on_sip_disconnected();               /* what sip_modem.c does */
    drain(resp, sizeof(resp), 300);
    check(held && ticks < 10 && strstr(resp, "OK") && !strstr(resp, "NO CARRIER"),
          "V.110 +++ ATH on the PTY: request sent, then OK");

    /* The far end's request (7.1.4.2), from an independent V.110 instance
     * on the other side of the DS0: the engine hangs up and the DTE gets
     * NO CARRIER. */
    {
        end_t *pe = calloc(1, sizeof(*pe));
        clear_channel_t peer;
        uint8_t back[160];

        cc_init_v110(&peer, 9600, end_pull, end_push, pe);
        me_on_sip_connected();
        for (int t = 0; t < 50; t++) {
            me_tx_g711(ds0, 160);
            cc_rx(&peer, ds0, 160);
            cc_tx(&peer, back, 160);
            me_rx_g711(back, 160);
        }
        drain(resp, sizeof(resp), 200);
        held = strstr(resp, "CONNECT 9600") != NULL && peer.v110_state == CC_V110_CONNECTED;
        cc_v110_disconnect(&peer);
        for (ticks = 0; me_get_state() == ME_DATA && ticks < 50; ticks++) {
            me_tx_g711(ds0, 160);
            cc_rx(&peer, ds0, 160);
            cc_tx(&peer, back, 160);
            me_rx_g711(back, 160);
        }
        check(held && me_get_state() == ME_HANGUP && ticks < 10
              && peer.v110_state == CC_V110_DOWN && peer.v110_cause == CC_V110_CAUSE_LOCAL,
              "V.110 far-end disconnect request: engine hangs up, peer sees it acknowledged");
        me_on_sip_disconnected();
        drain(resp, sizeof(resp), 300);
        check(strstr(resp, "NO CARRIER") != NULL, "V.110 far-end disconnect: NO CARRIER to the DTE");
        cc_release(&peer);
        free(pe);
    }

    /* A second request does not wait. */
    me_on_sip_connected();
    for (int t = 0; t < 25; t++) {
        me_tx_g711(ds0, 160);
        me_rx_g711(ds0, 160);
    }
    me_hangup();
    me_hangup();
    check(me_get_state() == ME_HANGUP, "V.110 ATH twice: immediate");
    me_on_sip_disconnected();
    drain(resp, sizeof(resp), 300);
}

/* sip_modem.c's DI hang-up callback: ATH reaches the engine through it. */
static void test_on_hangup(void *user_data)
{
    (void) user_data;
    me_hangup();
}

static int test_engine(void)
{
    const char *link = "/tmp/clear_channel_test_pty";
    struct termios tio;
    char desc[64];

    printf("engine, through the PTY:\n");
    unsetenv("ME_MODE");
    unsetenv("ME_K56FLEX");
    unsetenv("ME_V8_ADVERTISE_V91");
    unsetenv("ME_DATA_FRAMING");
    me_init();
    if (di_open(link) < 0)
        return -1;
    di_set_callbacks(NULL, NULL, test_on_hangup, NULL);
    if ((dte_fd = open(link, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
        return -1;
    if (tcgetattr(dte_fd, &tio) == 0) {
        cfmakeraw(&tio);
        tcsetattr(dte_fd, TCSANOW, &tio);
    }

    expect("ATE0", "OK");
    expect("AT+MS=CLEAR", "OK");
    me_modulation_offer_describe(desc, sizeof(desc));
    check(!strcmp(desc, "CLEAR 64k, no V.8"), desc);
    expect("AT+MS?", "+MS: CLEAR,1,0,0,0,0");
    engine_loop("CONNECT 64000", "CLEAR 64k");

    expect("AT+MS=CLEARMODE,0,0,56000", "OK");
    me_modulation_offer_describe(desc, sizeof(desc));
    check(!strcmp(desc, "CLEAR 56k, no V.8"), desc);
    expect("AT+MS?", "+MS: CLEAR,0,0,56000,0,56000");
    check(engine_loop("CONNECT 56000", "CLEAR 56k"), "CLEAR 56k: engine TX octets all have bit 1 set");

    expect("AT+MS=V120", "OK");
    me_modulation_offer_describe(desc, sizeof(desc));
    check(!strcmp(desc, "V120 64k, no V.8"), desc);
    engine_loop("CONNECT 64000", "V.120 64k");

    expect("AT+MS=V120,1,0,56000", "OK");
    expect("AT+MS?", "+MS: V120,1,0,56000,0,56000");
    check(engine_loop("CONNECT 56000", "V.120 56k"), "V.120 56k: engine TX octets all have bit 1 set");

    expect("AT+MS=V120,1,0,64001", "ERROR");

    expect("AT+MS=V110", "OK");
    me_modulation_offer_describe(desc, sizeof(desc));
    check(!strcmp(desc, "V110 38400 async, no V.8"), desc);
    expect("AT+MS=V110,0,0,14400", "OK");
    me_modulation_offer_describe(desc, sizeof(desc));
    check(!strcmp(desc, "V110 14400 async, no V.8"), desc);
    expect("AT+MS=V110,0,0,9600", "OK");
    expect("AT+MS?", "+MS: V110,0,0,9600,0,9600");
    engine_loop_v110("CONNECT 9600");
    engine_v110_ath();
    expect("AT+MS=V110,0,9601,9700", "ERROR");      /* no Table 8 rate */
    expect("AT+MS=V110,0,50,70", "ERROR");
    expect("ATZ", "OK");
    expect("ATE0", "OK");
    me_modulation_offer_describe(desc, sizeof(desc));
    check(!strcmp(desc, "V90|V34|V32|V22"), "ATZ back to the V.90 offer");

    close(dte_fd);
    di_close();
    return 0;
}

int main(void)
{
    make_text();
    test_packing();
    printf("back to back:\n");
    test_clear_pair(false);
    test_clear_pair(true);
    test_v120_pair(false);
    test_v120_pair(true);
    test_v120_rx_variants();
    test_v120_rr();
    for (size_t i = 0; i < sizeof(v110_text); i++)
        v110_text[i] = (uint8_t) (i * 13 + 5);     /* every octet value */
    printf("V.110 back to back:\n");
    for (int i = 0; i < cc_v110_n_rates; i++)
        test_v110_pair(cc_v110_rates[i]);
    {
        clear_channel_t z;

        check(cc_init_v110(&z, 50, NULL, NULL, NULL) < 0 && cc_v110_ra0_rate(56000) == 0,
              "V.110: 50 bit/s and 56000 refused");
    }
    test_v110_procedures();
    test_v110_ra0_rx();
    test_v110_t2();
    test_v110_flow();
    test_v110_sync();
    test_v120_ack();
    test_break();
    test_v120_verify();
    if (test_engine() < 0) {
        printf("  FAIL engine/PTY setup\n");
        failures++;
    }
    if (failures) {
        printf("clear_channel_test: %d failure(s)\n", failures);
        return 1;
    }
    printf("clear_channel_test: all passed\n");
    return 0;
}
