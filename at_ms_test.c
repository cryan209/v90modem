/*
 * at_ms_test.c -- V.250 6.4.1 +MS modulation selection over the DTE PTY.
 *
 * SpanDSP parses +MS and data_interface.c hands it to whoever owns the
 * datapump.  This drives the real PTY against a stand-in owner, so it checks
 * the parsing (omitted subparameters keep their value, chained commands,
 * malformed input), the ATZ / AT&F reset, the owner's right to refuse, and
 * that a connection below the minimum rate is cleared rather than CONNECTed.
 * The engine's own handler (me_at_modulation) is exercised end to end by
 * sip_v90_modem itself; it is too large to link here.
 */

#include "data_interface.h"

#include <spandsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

static int dte_fd = -1;
static int failures = 0;
static volatile int hangups = 0;

static void on_dial(const char *uri, void *u) { (void)uri; (void)u; }
static void on_answer(void *u)                { (void)u; }
static void on_hangup(void *u)                { (void)u; hangups++; }

/* The stand-in owner: three carriers, V90 by default. */
static at_modulation_t cur;
static const at_modulation_t dflt = { "V90", 1, 0, 0, 0, 0, "" };

static int handler(void *u, int op, struct at_modulation_s *m)
{
    (void)u;
    switch (op) {
    case AT_MODULATION_LIST:
        snprintf(m->supported, sizeof(m->supported), "V90,V34,V22B");
        return 0;
    case AT_MODULATION_QUERY:
        *m = cur;
        return 0;
    case AT_MODULATION_SET:
        if (strcmp(m->carrier, "V90") && strcmp(m->carrier, "V34")
            && strcmp(m->carrier, "V22B"))
            return -1;
        if (m->max_tx_rate && m->min_tx_rate > m->max_tx_rate)
            return -1;
        cur = *m;
        return 0;
    case AT_MODULATION_RESET:
        cur = dflt;
        return 0;
    }
    return -1;
}

static void drain(char *out, size_t max, int timeout_ms)
{
    size_t used = 0;

    out[0] = '\0';
    for (int waited = 0; waited < timeout_ms; waited += 10) {
        char buf[512];
        int n = (int)read(dte_fd, buf, sizeof(buf));

        if (n > 0) {
            if (used + (size_t)n < max) {
                memcpy(out + used, buf, (size_t)n);
                used += (size_t)n;
                out[used] = '\0';
            }
        } else {
            usleep(10000);
        }
    }
}

static void check(const char *what, const char *resp, const char *want, int present)
{
    int found = strstr(resp, want) != NULL;

    if (found == present) {
        printf("  ok   %-28s %s \"%s\"\n", what, present ? "->" : "-/>", want);
    } else {
        printf("  FAIL %-28s %s \"%s\", got \"%s\"\n", what,
               present ? "wanted" : "did not want", want, resp);
        failures++;
    }
}

static void expect(const char *cmd, const char *want)
{
    char line[128];
    char resp[2048];
    int n = snprintf(line, sizeof(line), "%s\r", cmd);

    if (write(dte_fd, line, (size_t)n) != n)
        perror("write");
    drain(resp, sizeof(resp), 300);
    check(cmd, resp, want, 1);
}

int main(void)
{
    const char *link = "/tmp/at_ms_test_pty";
    char resp[2048];

    cur = dflt;
    if (di_open(link) < 0) {
        fprintf(stderr, "di_open failed\n");
        return 1;
    }
    di_set_callbacks(on_dial, on_answer, on_hangup, NULL);
    if ((dte_fd = open(link, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
        perror("open pty slave");
        di_close();
        return 1;
    }

    printf("no owner registered:\n");
    expect("ATE0",            "OK");
    expect("AT+MS?",          "ERROR");

    di_set_modulation_handler(handler, NULL);

    printf("reporting:\n");
    expect("AT+MS=?",         "+MS: (V90,V34,V22B),(0,1),");
    expect("AT+MS?",          "+MS: V90,1,0,0,0,0");

    printf("setting (V.250 6.4.1):\n");
    expect("AT+MS=V34",       "OK");
    expect("AT+MS?",          "+MS: V34,1,0,0,0,0");
    expect("AT+MS=V34,0,2400,14400", "OK");
    expect("AT+MS?",          "+MS: V34,0,2400,14400,0,0");
    /* An omitted subparameter keeps its previous value (V.250 5.3.2). */
    expect("AT+MS=,1",        "OK");
    expect("AT+MS?",          "+MS: V34,1,2400,14400,0,0");
    expect("AT+MS=V22B;+MS?", "+MS: V22B,1,2400,14400,0,0");
    expect("at+ms=v90",       "OK");
    expect("AT+MS?",          "+MS: V90,1,");

    printf("refusals leave the setting alone:\n");
    expect("AT+MS=V17",       "ERROR");
    expect("AT+MS=V34,2",     "ERROR");
    expect("AT+MS=V34,1,14400,2400", "ERROR");
    expect("AT+MS=V34,1,X",   "ERROR");
    expect("AT+MS?",          "+MS: V90,1,");

    printf("ATZ and AT&F restore the default:\n");
    expect("AT+MS=V34,0",     "OK");
    expect("ATZ",             "OK");
    expect("ATE0",            "OK");
    expect("AT+MS?",          "+MS: V90,1,0,0,0,0");
    expect("AT+MS=V22B",      "OK");
    expect("AT&F",            "OK");
    expect("ATE0",            "OK");
    expect("AT+MS?",          "+MS: V90,1,0,0,0,0");

    printf("minimum rate:\n");
    expect("AT+MS=V34,1,0,0,9600,0", "OK");
    di_on_connected(4800);
    drain(resp, sizeof(resp), 300);
    check("connect at 4800", resp, "CONNECT", 0);
    if (hangups != 1) {
        printf("  FAIL connect at 4800: %d hangup requests, wanted 1\n", hangups);
        failures++;
    } else {
        printf("  ok   connect at 4800: call cleared\n");
    }
    di_on_disconnected();
    drain(resp, sizeof(resp), 100);
    di_on_connected(14400);
    drain(resp, sizeof(resp), 300);
    check("connect at 14400", resp, "CONNECT 14400", 1);
    di_on_disconnected();

    close(dte_fd);
    di_close();
    if (failures) {
        printf("at_ms_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("at_ms_test: all passed\n");
    return 0;
}
