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
