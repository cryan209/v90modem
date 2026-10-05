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
                     hdlc_rx_state_t *spy_ab)
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
    lsb = run_pair(&a, &b, 8000, r56, NULL);

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
    hdlc_rx_state_t *spy;
    char what[120];
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
    lsb = run_pair(&a, &b, 8000, r56, spy);

    snprintf(what, sizeof(what), "V.120 %s: %zu bytes A->B exact", r56 ? "56k" : "64k",
             sizeof(text_a));
    check(eb->dst_len == sizeof(text_a) && !memcmp(eb->dst, text_a, sizeof(text_a)), what);
    snprintf(what, sizeof(what), "V.120 %s: %zu bytes B->A exact", r56 ? "56k" : "64k",
             sizeof(text_b));
    check(ea->dst_len == sizeof(text_b) && !memcmp(ea->dst, text_b, sizeof(text_b)), what);
    snprintf(what, sizeof(what),
             "V.120 %s on the wire: %d frames, all 08 01 03 83, none bad, longest %d (4 + 256; FCS checked and stripped)",
             r56 ? "56k" : "64k", spy_frames, spy_max_len);
    check(spy_frames >= 12 && spy_hdr_ok == spy_frames && spy_bad == 0
          && spy_max_len == 4 + CC_V120_MAX_DATA, what);
    check(a.rx_bad_frames == 0 && b.rx_bad_frames == 0
          && a.rx_unsupported == 0 && b.rx_unsupported == 0, "V.120: no bad or unsupported frames");
    if (r56)
        check(lsb, "V.120 56k: every octet both ways has bit 1 set");
    {
        uint8_t h[4];

        cc_v120_frame_header(&b, h);
        check(!memcmp(h, "\x0A\x01\x03\x83", 4), "V.120 answerer sends C/R 1 (0x0A 0x01)");
    }
    hdlc_rx_free(spy);
    cc_release(&a);
    cc_release(&b);
    free(ea);
    free(eb);
}

/* Frames from elsewhere: control-state octets, I-frames, a corrupt FCS. */
static void test_v120_rx_variants(void)
{
    end_t *e = calloc(1, sizeof(*e));
    clear_channel_t rx;
    hdlc_tx_state_t *tx = hdlc_tx_init(NULL, false, 2, false, NULL, NULL);
    uint8_t line[2000];
    int n = 0;

    printf("V.120 receive variants:\n");
    cc_init_v120(&rx, false, true, end_pull, end_push, e);
    hdlc_tx_flags(tx, 2);   /* SpanDSP opens with no flag otherwise */
    /* E = 0, one CS octet (bit 8 set ends it), then "AB". */
    hdlc_tx_frame(tx, (const uint8_t *) "\x0A\x01\x03\x03\x80" "AB", 7);
    while (n < 200)
        line[n++] = (uint8_t) hdlc_tx_get_byte(tx);
    /* An I-frame: belongs to the acknowledged mode, must be dropped. */
    hdlc_tx_frame(tx, (const uint8_t *) "\x0A\x01\x00\x00\x83" "XX", 7);
    while (n < 400)
        line[n++] = (uint8_t) hdlc_tx_get_byte(tx);
    /* UI with P set and BR: "CD", break counted. */
    hdlc_tx_frame(tx, (const uint8_t *) "\x0A\x01\x13\xC3" "CD", 6);
    while (n < 600)
        line[n++] = (uint8_t) hdlc_tx_get_byte(tx);
    /* A good frame, then corrupt one bit inside it. */
    hdlc_tx_frame(tx, (const uint8_t *) "\x0A\x01\x03\x83" "EF", 6);
    {
        int start = n;

        while (n < 800)
            line[n++] = (uint8_t) hdlc_tx_get_byte(tx);
        line[start + 4] ^= 0x10;
    }
    cc_rx(&rx, line, n);
    check(e->dst_len == 4 && !memcmp(e->dst, "ABCD", 4),
          "CS octet skipped, I-frame dropped, P bit accepted: \"ABCD\"");
    check(rx.rx_unsupported == 1, "the I-frame counted as unsupported");
    check(rx.rx_breaks == 1, "BR counted");
    check(rx.rx_bad_frames == 1, "the corrupted frame counted bad");
    hdlc_tx_free(tx);
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
    expect("ATZ", "OK");
    expect("ATE0", "OK");
    me_modulation_offer_describe(desc, sizeof(desc));
    check(!strcmp(desc, "V90|V34|V22"), "ATZ back to the V.90 offer");

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
