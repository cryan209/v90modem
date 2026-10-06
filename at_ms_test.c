/*
 * at_ms_test.c -- V.250 6.4.1 AT+MS modulation selection.
 *
 * Two layers.  at_ms.c's parser and responses on their own; then the whole
 * path a DTE uses: the real PTY, SpanDSP's AT interpreter (whose +MS was a
 * TODO that swallowed the command and answered OK), data_interface.c, and
 * the engine's offer -- checked as the V8_MOD_* bits the next call's CM/JM
 * will carry, which is what "AT+MS=V34 makes the next call offer V.34" means.
 */

#include "at_ms.h"
#include "data_interface.h"
#include "modem_engine.h"

#include <spandsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

static int dte_fd = -1;
static int failures = 0;

static void check(int ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        failures++;
}

/* ---------------------------------------------------------------- */
/* at_ms.c alone                                                     */
/* ---------------------------------------------------------------- */

static void parse_ok(const char *args, const char *carrier, int automode,
                     int min_tx, int max_tx, int min_rx, int max_rx)
{
    at_ms_settings_t s;
    char what[160];
    int ok;

    ok = at_ms_parse(args, &s) == AT_MS_SET
      && !strcmp(s.carrier, carrier) && s.automode == automode
      && s.min_tx_rate == min_tx && s.max_tx_rate == max_tx
      && s.min_rx_rate == min_rx && s.max_rx_rate == max_rx;
    snprintf(what, sizeof(what), "parse \"%s\"", args);
    check(ok, what);
}

static void parse_bad(const char *args)
{
    at_ms_settings_t s;
    char what[160];

    snprintf(what, sizeof(what), "reject \"%s\"", args);
    check(at_ms_parse(args, &s) == AT_MS_ERROR, what);
}

static void test_parser(void)
{
    char buf[200];
    at_ms_settings_t s;

    printf("at_ms.c parser:\n");
    check(at_ms_parse("?", &s) == AT_MS_READ, "\"?\" is a read");
    check(at_ms_parse("=?", &s) == AT_MS_TEST, "\"=?\" is a test");
    parse_ok("=V34", "V34", 1, 0, 0, 0, 0);
    parse_ok("=V34,0", "V34", 0, 0, 0, 0, 0);
    parse_ok("=\"V90\",1", "V90", 1, 0, 0, 0, 0);
    parse_ok("=v92", "V92", 1, 0, 0, 0, 0);
    parse_ok("=V34,1,2400,28800", "V34", 1, 2400, 28800, 2400, 28800);
    parse_ok("=V90,1,300,33600,300,56000", "V90", 1, 300, 33600, 300, 56000);
    parse_ok("=V34,,,14400", "V34", 1, 0, 14400, 0, 14400);
    parse_ok("=V22B", "V22B", 1, 0, 0, 0, 0);
    parse_ok("=V22BIS", "V22B", 1, 0, 0, 0, 0);
    parse_ok("=K56", "K56", 1, 0, 0, 0, 0);
    parse_ok("=56", "K56", 1, 0, 0, 0, 0);
    parse_ok("=56K,1", "K56", 1, 0, 0, 0, 0);
    parse_ok("=\"K56FLEX\",1,0,56000", "K56", 1, 0, 56000, 0, 56000);
    parse_ok("=V91,0,0,64000,0,64000", "V91", 0, 0, 64000, 0, 64000);
    parse_ok("=V32B", "V32B", 1, 0, 0, 0, 0);
    parse_ok("=V32,1,4800,9600", "V32", 1, 4800, 9600, 4800, 9600);
    parse_ok("=X2", "X2", 1, 0, 0, 0, 0);
    parse_bad("=K56,0");        /* no K56flex data mode to stand alone on */
    parse_bad("=56,0");
    parse_ok("=V32B,0", "V32B", 0, 0, 0, 0, 0);
    parse_bad("=V34,1,0,56000");/* above what V.34 carries */
    parse_bad("=V22,1,0,2400");
    parse_bad("=V90,1,0,64000");
    parse_ok("=K56,1,0,60000", "K56", 1, 0, 60000, 0, 60000);
    parse_bad("=K56,1,0,62000");
    parse_ok("=X2,1,0,64000", "X2", 1, 0, 64000, 0, 64000);
    parse_bad("=V91,1,0,64001");
    parse_bad("=V21");
    parse_ok("=V34+", "V34", 1, 0, 0, 0, 0);
    parse_ok("=V34B,0", "V34", 0, 0, 0, 0, 0);
    parse_ok("=HST,1,0,16800", "HST", 1, 0, 16800, 0, 16800);
    parse_ok("=TERBO", "V32TERBO", 1, 0, 0, 0, 0);
    parse_ok("=V.FC", "VFC", 1, 0, 0, 0, 0);
    parse_bad("=HST,0");
    parse_bad("=V32TERBO,1,0,21600");
    parse_bad("=B103");          /* recognised, no datapump */
    parse_bad("=B212A");
    parse_ok("=CLEAR", "CLEAR", 1, 0, 0, 0, 0);
    parse_ok("=CLEARMODE,0", "CLEAR", 0, 0, 0, 0, 0);
    parse_ok("=64K,1,0,56000", "CLEAR", 1, 0, 56000, 0, 56000);
    parse_bad("=V110");
    parse_ok("=V120", "V120", 1, 0, 0, 0, 0);
    parse_bad("=V120,1,0,64001");
    parse_bad("=X75");
    check(at_ms_carrier_available("V34") && !at_ms_carrier_available("B103")
          && at_ms_carrier_available("64K") && at_ms_carrier_available("V120")
          && !at_ms_carrier_available("V110") && at_ms_carrier_max_rate("X75") == 64000,
          "unavailable carriers known but unavailable");
    {
        at_ms_settings_t m;

        at_ms_parse("=CLEAR", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "clear"), "CLEAR -> clear");
        at_ms_parse("=CLEAR,1,0,56000", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "clear56"), "CLEAR max 56000 -> clear56");
        at_ms_parse("=V120,0,0,64000,0,56000", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "v120-56"), "V120 rx max 56000 -> v120-56");
        at_ms_parse("=V120,1,0,64000", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "v120"), "V120 max 64000 -> v120");
        at_ms_parse("=V22", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "v22-1200"), "V22 -> v22-1200");
        at_ms_parse("=V22B,1,0,1200", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "v22-1200"), "V22B max 1200 -> v22-1200");
        at_ms_parse("=V22B,1,0,2400", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "v22"), "V22B max 2400 -> v22");
        at_ms_parse("=V34,1,0,28800", &m);
        check(!strcmp(at_ms_settings_to_mode(&m), "v34"), "56k rule only for CLEAR/V120");
        check(!strcmp(at_ms_mode_to_carrier("clear56"), "CLEAR")
              && !strcmp(at_ms_mode_to_carrier("v120-56"), "V120"), "56k modes read back");
    }
    parse_bad("=V17");          /* a carrier this DCE cannot offer */
    parse_bad("=");
    parse_bad("=V34,2");        /* automode is 0 or 1 */
    parse_bad("=V34,1,9600,4800");
    parse_bad("=V34,1,0,99999");
    parse_bad("=V34,1,1,2,3,4,5");
    parse_bad("=V34X");
    parse_bad("V34");

    check(!strcmp(at_ms_carrier_to_mode("V22", false), "v22-1200")
          && !strcmp(at_ms_carrier_to_mode("V22", true), "v22-1200")
          && !strcmp(at_ms_mode_to_carrier("v22-1200"), "V22")
          && !strcmp(at_ms_carrier_to_mode("V22B", true), "v22")
          && !strcmp(at_ms_carrier_to_mode("V32B", true), "v32bis")
          && !strcmp(at_ms_carrier_to_mode("V32B", false), "v32bis")
          && !strcmp(at_ms_carrier_to_mode("HST", true), "v32bis")
          && at_ms_carrier_to_mode("HST", false) == NULL
          && !strcmp(at_ms_mode_to_carrier("v32bis"), "V32B")
          && !strcmp(at_ms_carrier_to_mode("56", true), "k56")
          && at_ms_carrier_to_mode("K56", false) == NULL
          && !strcmp(at_ms_carrier_to_mode("V91", false), "v91")
          && !strcmp(at_ms_mode_to_carrier("v22"), "V22B")
          && !strcmp(at_ms_mode_to_carrier("v92"), "V92")
          && !strcmp(at_ms_mode_to_carrier("k56"), "K56")
          && !strcmp(at_ms_mode_to_carrier("v91"), "V91")
          && at_ms_carrier_to_mode("V17", true) == NULL, "carrier <-> mode names");
    check(at_ms_carrier_max_rate("V32B") == 14400
          && at_ms_carrier_max_rate("V91") == 64000
          && at_ms_carrier_max_rate("K56") == 60000
          && at_ms_carrier_max_rate("X2") == 64000
          && at_ms_carrier_max_rate("V17") == 0, "carrier maximum rates");

    parse_ok("=V90,0,0,0,4800,33600", "V90", 0, 0, 0, 4800, 33600);
    at_ms_parse("=V90,0,0,0,4800,33600", &s);
    at_ms_format_read(&s, buf, sizeof(buf));
    check(!strcmp(buf, "+MS: V90,0,0,0,4800,33600"), buf);
    check(at_ms_parse("$", &s) == AT_MS_HELP, "\"$\" is help");
    {
        char help[4096];

        at_ms_parse("=V91,0", &s);
        at_ms_format_help(&s, help, sizeof(help));
        check(strstr(help, "K56      56,56K,K56FLEX   1         60000") != NULL
              && strstr(help, "V34      V34+,V34B,V34BIS 0,1       33600") != NULL
              && strstr(help, "V32B     V32BIS           0,1       14400") != NULL
              && strstr(help, "V91                       0,1       64000") != NULL
              && strstr(help, "V32TERBO TERBO,V32T       1         19200") != NULL
              && strstr(help, "Recognised, no datapump here") != NULL
              && strstr(help, "B103                      -           300  Bell 103") != NULL
              && strstr(help, "Recognised") > strstr(help, "V120 ")
              && strstr(help, "CLEAR    CLEARMODE,64K    0,1       64000  DS0 bits, V.14/LAPM") != NULL
              && strstr(help, "V110                      -         64000") != NULL
              && strstr(help, "Recognised") < strstr(help, "B103")
              && strstr(help, "Current: V91,0,0,0,0,0") != NULL
              && strlen(help) < sizeof(help) - 1, "+MS$ help rows and current setting");
    }
    at_ms_format_test(buf, sizeof(buf));
    check(!strcmp(buf, "+MS: (V22,V22B,V32,V32B,HST,V32TERBO,VFC,V34,K56,V90,V92,V91,X2,CLEAR,V120),(0,1),"
                       "(0-64000),(0-64000),(0-64000),(0-64000)"), buf);
}

/* ---------------------------------------------------------------- */
/* PTY -> SpanDSP -> data_interface -> engine                        */
/* ---------------------------------------------------------------- */

static void drain(char *out, size_t max, int timeout_ms)
{
    size_t used = 0;

    out[0] = '\0';
    for (int waited = 0; waited < timeout_ms; waited += 10) {
        char buf[512];
        int n = (int) read(dte_fd, buf, sizeof(buf));

        if (n > 0) {
            if (used + (size_t) n < max) {
                memcpy(out + used, buf, (size_t) n);
                used += (size_t) n;
                out[used] = '\0';
            }
        } else {
            usleep(10000);
        }
    }
}

static void expect(const char *cmd, const char *want)
{
    char line[128];
    char resp[8192];
    int n = snprintf(line, sizeof(line), "%s\r", cmd);

    if (write(dte_fd, line, (size_t) n) != n)
        perror("write");
    drain(resp, sizeof(resp), 300);
    if (strstr(resp, want)) {
        printf("  ok   %-24s -> %s\n", cmd, want);
    } else {
        printf("  FAIL %-24s -> wanted \"%s\", got \"%s\"\n", cmd, want, resp);
        failures++;
    }
}

static void expect_offer(int want, const char *what)
{
    int got = me_modulation_offer_bits();
    char buf[160];

    snprintf(buf, sizeof(buf), "next call offers %s (bits 0x%x, want 0x%x)",
             what, got, want);
    check(got == want, buf);
}

/* The whole next-call offer, V.91 and K56flex included. */
static void expect_describe(const char *want)
{
    char got[64];
    char buf[160];

    me_modulation_offer_describe(got, sizeof(got));
    snprintf(buf, sizeof(buf), "next call offers \"%s\" (want \"%s\")", got, want);
    check(!strcmp(got, want), buf);
}

static int test_engine(void)
{
    const char *link = "/tmp/at_ms_test_pty";

    printf("AT+MS through the PTY and the engine:\n");
    /* The power-on default must be the shipped one, whatever the caller's
     * environment says; ME_MODE is exercised separately below. */
    unsetenv("ME_MODE");
    unsetenv("ME_V92_ENABLE");
    unsetenv("ME_V90_ROLE");
    unsetenv("ME_K56FLEX");
    unsetenv("ME_V8_ADVERTISE_V91");
    me_init();
    if (di_open(link) < 0) {
        fprintf(stderr, "di_open failed\n");
        return -1;
    }
    if ((dte_fd = open(link, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
        perror("open pty slave");
        di_close();
        return -1;
    }
    /* Raw, like a real DTE: with the slave's line discipline echoing, ATZ
     * turning the modem's own echo back on makes the two feed each other. */
    {
        struct termios tio;

        if (tcgetattr(dte_fd, &tio) == 0) {
            cfmakeraw(&tio);
            tcsetattr(dte_fd, TCSANOW, &tio);
        }
    }

    expect("ATE0", "OK");
    expect_offer(V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "V.90|V.34|V.32|V.22 by default");
    expect("AT+MS?", "+MS: V90,1,0,0,0,0");
    expect("AT+MS=?", "+MS: (V22,V22B,V32,V32B,HST,V32TERBO,VFC,V34,K56,V90,V92,V91,X2,CLEAR,V120),(0,1)");
    expect_describe("V90|V34|V32|V22");
    expect("AT+MS$", "K56      56,56K,K56FLEX   1         60000  K56flex V.8bis, then V.90");
    expect("AT+MS$", "Current: V90,1,0,0,0,0");

    expect("AT+MS=V34", "OK");
    expect_offer(V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "V.34 (automode: V.22 fallback kept)");
    expect("AT+MS?", "+MS: V34,1,0,0,0,0");

    expect("AT+MS=V34,0", "OK");
    expect_offer(V8_MOD_V34, "V.34 alone");
    expect("AT+MS?", "+MS: V34,0,0,0,0,0");

    expect("AT+MS=V90,0", "OK");
    expect_offer(V8_MOD_V90 | V8_MOD_V34, "V.90 with the V.34 its upstream needs");

    expect("AT+MS=V92,1,0,33600,0,48000", "OK");
    expect_offer(V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "V.92's V.8 bits");
    expect("AT+MS?", "+MS: V92,1,0,33600,0,48000");

    expect("AT+MS=V22B,0", "OK");
    expect_offer(V8_MOD_V22, "V.22bis alone");
    expect("AT+MS?", "+MS: V22B,0,0,0,0,0");

    expect("AT+MS=V22", "OK");
    expect_offer(V8_MOD_V22, "V.22 alone (V.8's V.22/V.22bis bit)");
    expect("AT+MS?", "+MS: V22,1,0,0,0,0");
    expect_describe("V22");

    /* K56flex: V.8bis first, then the ordinary V.90 offer; never alone. */
    expect("AT+MS=K56", "OK");
    expect_describe("V90|V34|V32|V22|+K56");
    expect("AT+MS?", "+MS: K56,1,0,0,0,0");
    expect("AT+MS$", "Current: K56,1,0,0,0,0");
    expect("AT+MS=56", "OK");
    expect("AT+MS?", "+MS: K56,1,0,0,0,0");
    expect("AT+MS=K56,0", "ERROR");
    expect_describe("V90|V34|V32|V22|+K56");

    /* V.91: on top of V.90 with automode, with only V.34 beside it without. */
    expect("AT+MS=V91", "OK");
    expect_describe("V90|V34|V32|V22|+V91");
    expect("AT+MS=V91,0,0,64000,0,64000", "OK");
    expect_describe("V34|+V91");
    expect("AT+MS?", "+MS: V91,0,0,64000,0,64000");

    /* V.32bis: V.8's V.32 bit, with V.22 under automode and alone
     * without it. */
    expect("AT+MS=V32B,1,0,14400", "OK");
    expect_describe("V32|V22");
    expect("AT+MS?", "+MS: V32B,1,0,14400,0,14400");
    expect("AT+MS=V32B,0", "OK");
    expect_describe("V32");
    expect("AT+MS=V34,1,0,56000", "ERROR");
    expect("AT+MS=V34+", "OK");
    expect("AT+MS?", "+MS: V34,1,0,0,0,0");
    expect("AT+MS=HST", "OK");
    expect_describe("V32|V22");
    expect("AT+MS?", "+MS: HST,1,0,0,0,0");
    expect("AT+MS=B103", "ERROR");
    expect("AT+MS=X75", "ERROR");
    expect("AT+MS?", "+MS: HST,1,0,0,0,0");
    expect("AT+MS=V22B,0", "OK");

    /* A rejected command must leave the offer alone. */
    expect("AT+MS=V17", "ERROR");
    expect("AT+MS=V34,7", "ERROR");
    expect_offer(V8_MOD_V22, "unchanged after ERROR");

    /* +MS among other commands on one line (V.250 5.4.1's ';'). */
    expect("AT+MS=V34;+MS?", "+MS: V34,1,0,0,0,0");
    expect_offer(V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "V.34 from a compound line");

    /* ATZ and AT&F restore the power-on default. */
    expect("ATZ", "OK");
    expect("ATE0", "OK");
    expect_offer(V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "the default again after ATZ");
    expect("AT+MS=V34,0", "OK");
    expect("AT&F", "OK");
    expect("ATE0", "OK");
    expect("AT+MS?", "+MS: V90,1,0,0,0,0");

    /* The engine API directly: names, and ME_MODE's "auto". */
    check(me_set_modulation_offer("v34", false) == 0, "me_set_modulation_offer(v34)");
    expect_offer(V8_MOD_V34, "V.34 alone via the API");
    check(me_set_modulation_offer("v17", true) < 0, "unknown mode rejected");
    expect_offer(V8_MOD_V34, "unchanged after a rejected mode");
    check(me_set_modulation_offer("k56", false) < 0, "k56 without automode rejected");
    check(me_set_modulation_offer("auto", true) == 0, "auto accepted");
    expect_offer(V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "auto is the v90 default");

    /* The env overrides still win when set. */
    setenv("ME_V8_ADVERTISE_V91", "1", 1);
    setenv("ME_K56FLEX", "1", 1);
    expect_describe("V90|V34|V32|V22|+V91|+K56");
    setenv("ME_K56FLEX", "0", 1);
    check(me_set_modulation_offer("k56", true) == 0, "k56 accepted");
    expect_describe("V90|V34|V32|V22|+V91");
    unsetenv("ME_K56FLEX");
    unsetenv("ME_V8_ADVERTISE_V91");

    close(dte_fd);
    /* 6.4.1 rate bounds: a modulation that cannot connect inside them is not
     * offered, and bounds nothing can meet are ERROR (5.4.4.2). */
    expect("AT+MS=V32B,0,10000,11000", "ERROR");    /* no V.32bis rate in there */
    expect("AT+MS=V34,1,0,9600", "OK");
    expect("AT+MS?", "+MS: V34,1,0,9600,0,9600");
    expect_offer(V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "max 9600: V.34, V.32bis and V.22 still fit");
    expect("AT+MS=V34,1,4800,9600", "OK");
    expect_offer(V8_MOD_V34 | V8_MOD_V32, "min 4800: V.22bis (2400 at most) withdrawn");
    expect("AT+MS=V90,1,0,26000,0,0", "OK");
    expect_offer(V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22,
                 "transmit max 26000: below V.90's 28000 downstream floor, V.90 withdrawn");
    expect("AT+MS=V90,1,40000,0,0,0", "OK");
    expect_offer(V8_MOD_V90 | V8_MOD_V34,
                 "transmit min 40000: V.90 alone (its V.34 bit kept), no V.32bis/V.22");
    expect("AT+MS=V90,0,0,0,40000,0", "ERROR");      /* nothing receives 40000 */
    expect("AT+MS?", "+MS: V90,1,40000,0,0,0");      /* ...and the setting stands */
    expect("AT+MS=V90", "OK");
    expect_offer(V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V32 | V8_MOD_V22, "no bounds: the full offer again");

    di_close();
    return 0;
}

int main(void)
{
    test_parser();
    if (test_engine() < 0)
        failures++;
    if (failures) {
        printf("at_ms_test: %d failure(s)\n", failures);
        return 1;
    }
    printf("at_ms_test: all passed\n");
    return 0;
}
