/*
 * console_test.c -- the two DTE console arrangements through the real PTYs,
 * SpanDSP's AT interpreter and data_interface.c, with a stand-in engine.
 *
 * Classic combined console (V.250 5.2.x, 6.3.x): one port carries commands and,
 * once CONNECT has been reported, the payload.  "+++" with guard times (TIES)
 * returns it to command state with the call up, ATO resumes the data, ATH ends
 * the call.
 *
 * Control + data console: a control port that is ALWAYS in command state --
 * commands work mid-call, CONNECT / NO CARRIER / RING are reported on it -- and
 * a data port that carries only payload, so a "+++" in the payload is payload
 * and there is no escape to guard.
 *
 * The engine is replaced by di_on_connected()/di_on_disconnected() calls and
 * di_read_data()/di_write_data() on the other side, which is exactly the
 * surface the real engine uses.
 */

#include "data_interface.h"

#include <spandsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

static int failures;
static int dial_calls;
static int answer_calls;
static int hangup_calls;
static char last_number[64];

static void check(int ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        failures++;
}

static void cb_dial(const char *num, void *u)
{
    (void) u;
    dial_calls++;
    snprintf(last_number, sizeof(last_number), "%s", num);
}

static void cb_answer(void *u)
{
    (void) u;
    answer_calls++;
}

static void cb_hangup(void *u)
{
    (void) u;
    hangup_calls++;
}

static int open_dte(const char *link)
{
    int fd = open(link, O_RDWR | O_NOCTTY | O_NONBLOCK);
    struct termios tio;

    if (fd < 0) {
        perror(link);
        return -1;
    }
    if (tcgetattr(fd, &tio) == 0) {
        cfmakeraw(&tio);
        tcsetattr(fd, TCSANOW, &tio);
    }
    return fd;
}

/* Everything the port sends within timeout_ms. */
static size_t collect(int fd, char *out, size_t max, int timeout_ms)
{
    size_t used = 0;

    out[0] = '\0';
    for (int waited = 0; waited < timeout_ms; waited += 10) {
        char buf[512];
        int n = (int) read(fd, buf, sizeof(buf));

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
    return used;
}

static void send_str(int fd, const char *s)
{
    if (write(fd, s, strlen(s)) != (ssize_t) strlen(s))
        perror("write");
}

/* A command and what came back.  The reply is matched as a substring. */
static void expect(int fd, const char *cmd, const char *want)
{
    char line[160];
    char resp[4096];
    char what[256];

    snprintf(line, sizeof(line), "%s\r", cmd);
    send_str(fd, line);
    collect(fd, resp, sizeof(resp), 250);
    snprintf(what, sizeof(what), "%s -> %s", cmd, want);
    if (!strstr(resp, want))
        printf("       got \"%s\"\n", resp);
    check(strstr(resp, want) != NULL, what);
}

/* Payload the engine would transmit: bytes the DTE wrote to the data path. */
static int engine_reads(char *out, int max, int timeout_ms)
{
    int used = 0;

    for (int waited = 0; waited < timeout_ms && used < max; waited += 10) {
        int n = di_read_data((uint8_t *) out + used, max - used);

        if (n > 0)
            used += n;
        else
            usleep(10000);
    }
    return used;
}

/* ---------------------------------------------------------------- */

static void test_classic(void)
{
    const char *link = "/tmp/console_test_classic";
    char buf[4096];
    char got[256];
    int dte;

    printf("classic combined console:\n");
    dial_calls = answer_calls = hangup_calls = 0;
    if (di_open(link) < 0) {
        failures++;
        return;
    }
    di_set_callbacks(cb_dial, cb_answer, cb_hangup, NULL);
    dte = open_dte(link);
    if (dte < 0) {
        failures++;
        di_close();
        return;
    }

    expect(dte, "ATE0", "OK");
    expect(dte, "ATO", "NO CARRIER");   /* 6.3.7: nothing to resume */
    expect(dte, "ATO1", "ERROR");
    expect(dte, "ATD5551234", "");
    usleep(100000);
    check(dial_calls == 1 && !strcmp(last_number, "5551234"), "ATD reaches the dial callback");
    send_str(dte, "should be dropped, no call\r");
    usleep(100000);
    check(engine_reads(got, sizeof(got), 100) == 0, "bytes sent before CONNECT are not payload");

    di_on_connected(33600);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "CONNECT 33600") != NULL, "CONNECT <rate> on the one port");

    send_str(dte, "hello\r\n");
    check(engine_reads(got, sizeof(got) - 1, 300) == 7 && !memcmp(got, "hello\r\n", 7),
          "payload goes to the engine verbatim (no AT parsing)");
    di_write_data((const uint8_t *) "from far end", 12);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "from far end") != NULL, "far-end payload reaches the DTE");

    /* TIES: guard, +++, guard.  The 1 s guards are what make a "+++" in a
     * payload stream harmless, so exercise both halves. */
    send_str(dte, "a+++b");
    check(engine_reads(got, sizeof(got) - 1, 300) == 5 && !memcmp(got, "a+++b", 5),
          "+++ inside a payload run is payload");
    sleep(2);
    send_str(dte, "+++");
    collect(dte, buf, sizeof(buf), 1500);
    check(strstr(buf, "OK") != NULL, "guarded +++ answers OK (online command state)");
    check(hangup_calls == 0, "the escape does not drop the call");
    expect(dte, "ATS0?", "000");
    send_str(dte, "ATO\r");
    collect(dte, buf, sizeof(buf), 300);
    send_str(dte, "back online");
    check(engine_reads(got, sizeof(got) - 1, 300) == 11 && !memcmp(got, "back online", 11),
          "ATO resumes the data connection");

    sleep(2);
    send_str(dte, "+++");
    collect(dte, buf, sizeof(buf), 1500);
    send_str(dte, "ATH\r");
    usleep(200000);
    check(hangup_calls == 1, "ATH from online command state hangs up");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 200);
    check(strstr(buf, "NO CARRIER") == NULL, "no NO CARRIER after the DTE's own ATH (it was answered OK)");
    expect(dte, "ATO", "NO CARRIER");   /* and the call really is gone */
    expect(dte, "AT", "OK");

    /* The far end (or the engine) ends a call the DTE did not: that is owed
     * NO CARRIER, and ATO afterwards has nothing to resume. */
    di_on_connected(33600);
    collect(dte, buf, sizeof(buf), 150);
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 200);
    check(strstr(buf, "NO CARRIER") != NULL, "NO CARRIER when the far end ends the call");
    expect(dte, "AT", "OK");

    close(dte);
    di_close();
}

static void test_split(void)
{
    const char *clink = "/tmp/console_test_ctl";
    const char *dlink = "/tmp/console_test_data";
    char buf[4096];
    char got[256];
    int ctl, dat;

    printf("control + data console:\n");
    dial_calls = answer_calls = hangup_calls = 0;
    if (di_open_split(clink, dlink) < 0) {
        failures++;
        return;
    }
    di_set_callbacks(cb_dial, cb_answer, cb_hangup, NULL);
    ctl = open_dte(clink);
    dat = open_dte(dlink);
    if (ctl < 0 || dat < 0) {
        failures++;
        di_close();
        return;
    }

    expect(ctl, "ATE0", "OK");
    expect(ctl, "ATD5559876", "");
    usleep(100000);
    check(dial_calls == 1 && !strcmp(last_number, "5559876"), "ATD on the control port dials");
    send_str(dat, "early payload");
    usleep(100000);
    check(engine_reads(got, sizeof(got), 100) == 0, "data port is inert until CONNECT");

    di_on_connected(28800);
    collect(ctl, buf, sizeof(buf), 150);
    check(strstr(buf, "CONNECT 28800") != NULL, "CONNECT is reported on the control port");
    collect(dat, buf, sizeof(buf), 100);
    check(strstr(buf, "CONNECT") == NULL, "...and not on the data port");

    send_str(dat, "payload+++payload");
    check(engine_reads(got, sizeof(got) - 1, 300) == 17 && !memcmp(got, "payload+++payload", 17),
          "payload (even a +++) passes untouched, no escape guard");
    di_write_data((const uint8_t *) "downstream", 10);
    collect(dat, buf, sizeof(buf), 150);
    check(strstr(buf, "downstream") != NULL, "far-end payload appears on the data port only");
    collect(ctl, buf, sizeof(buf), 100);
    check(strstr(buf, "downstream") == NULL, "...and not on the control port");

    /* Command state throughout the call. */
    expect(ctl, "ATS0?", "000");
    expect(ctl, "AT", "OK");
    send_str(dat, "still flowing");
    check(engine_reads(got, sizeof(got) - 1, 300) == 13, "payload flows while commands are issued");
    {
        char want[300];
        char dname[256];
        const char *slash;

        /* ATO names the data port's PTY and stays in command state. */
        send_str(ctl, "ATO\r");
        collect(ctl, buf, sizeof(buf), 300);
        slash = strstr(buf, "/dev/");
        snprintf(dname, sizeof(dname), "%s", slash ? slash : "");
        dname[strcspn(dname, "\r\n")] = '\0';
        snprintf(want, sizeof(want), "ATO names a /dev/ pty and answers OK (got \"%s\")", dname);
        check(slash && strstr(buf, "OK") && !strstr(buf, "CONNECT"), want);
        check(slash && !strcmp(dname, ttyname(dat)), "...and it is the data port's slave");
    }
    expect(ctl, "ATH", "");
    usleep(200000);
    check(hangup_calls == 1, "ATH on the control port hangs up while the data port is live");
    di_on_disconnected();
    collect(ctl, buf, sizeof(buf), 200);
    check(strstr(buf, "NO CARRIER") == NULL, "no NO CARRIER after the DTE's own ATH");
    send_str(dat, "after hangup");
    usleep(100000);
    check(engine_reads(got, sizeof(got), 100) == 0, "data port is inert again after the call");
    expect(ctl, "AT", "OK");

    /* The far end drops the call: NO CARRIER on the control port. */
    di_on_connected(28800);
    collect(ctl, buf, sizeof(buf), 150);
    di_on_disconnected();
    collect(ctl, buf, sizeof(buf), 200);
    check(strstr(buf, "NO CARRIER") != NULL, "NO CARRIER on the control port when the far end ends the call");
    send_str(dat, "after remote drop");
    usleep(100000);
    check(engine_reads(got, sizeof(got), 100) == 0, "data port inert after the far end drops too");

    /* Remote-initiated: RING on control, ATA answers. */
    di_on_ring();
    collect(ctl, buf, sizeof(buf), 200);
    check(strstr(buf, "RING") != NULL, "RING is reported on the control port");

    close(ctl);
    close(dat);
    di_close();
}

/* What the engine would report for the "call" in progress. */
static v250_connect_report_t fake_report;

static void fake_connect_info(int rate, v250_connect_report_t *r)
{
    (void) rate;
    *r = fake_report;
}

/* Everything the port says after cmd, in one string. */
static void exchange(int fd, const char *cmd, char *out, size_t max)
{
    char line[160];

    snprintf(line, sizeof(line), "%s\r", cmd);
    send_str(fd, line);
    collect(fd, out, max, 250);
}

static void test_v250_parameters(void)
{
    const char *link = "/tmp/console_test_v250";
    char buf[4096];
    int dte;

    printf("V.250 +MR/+ES/+ER/+DS/+DR through the interpreter:\n");
    if (di_open(link) < 0) {
        failures++;
        return;
    }
    di_set_callbacks(cb_dial, cb_answer, cb_hangup, NULL);
    dte = open_dte(link);
    if (dte < 0) {
        failures++;
        di_close();
        return;
    }
    expect(dte, "ATE0", "OK");
    expect(dte, "AT+MR?", "+MR: 0");
    expect(dte, "AT+ES?", "+ES: 3,0,2");
    expect(dte, "AT+DS?", "+DS: 3,0,1024,32");
    expect(dte, "AT+ES=?", "+ES: (1-3),(0,2-3),(1-2,4-5)");
    expect(dte, "AT+DS=?", "+DS: (0-3),(0,1),(512-65535),(6-250)");
    expect(dte, "AT+ES=3,2,5", "OK");
    expect(dte, "AT+ES?", "+ES: 3,2,5");
    expect(dte, "AT+ES=4", "ERROR");               /* alternative protocol */
    expect(dte, "AT+ES?", "+ES: 3,2,5");           /* untouched */
    expect(dte, "AT+DS=3,1,4096,64", "OK");
    expect(dte, "AT+DS?", "+DS: 3,1,4096,64");
    expect(dte, "AT+DS=3,0,100", "ERROR");
    expect(dte, "AT+DS?", "+DS: 3,1,4096,64");
    expect(dte, "AT+MR=1;+ER=1;+DR=1", "OK");      /* extended commands chained */
    expect(dte, "AT+MR?;+ER?;+DR?", "+DR: 1");
    expect(dte, "AT+MR=2", "ERROR");
    /* Settings are per-power-on/ATZ, like the rest of the profile. */
    expect(dte, "ATZ", "OK");
    expect(dte, "AT+ES?", "+ES: 3,0,2");
    expect(dte, "AT+DS?", "+DS: 3,0,1024,32");
    expect(dte, "AT+MR?", "+MR: 0");
    expect(dte, "AT+ER?", "+ER: 0");
    expect(dte, "AT+DR?", "+DR: 0");
    expect(dte, "AT+ES=1,0,1", "OK");
    expect(dte, "AT&F", "OK");
    expect(dte, "AT+ES?", "+ES: 3,0,2");
    {
        v250_ctl_t cfg;

        expect(dte, "AT+ES=2,3,4", "OK");
        di_get_v250_settings(&cfg);
        check(cfg.es_set && cfg.es[0] == 2 && cfg.es[1] == 3 && cfg.es[2] == 4,
              "the engine sees what the DTE set (di_get_v250_settings)");
    }

    /* Reports at CONNECT: modulation, error control, compression, in that order
     * and before CONNECT, and only as enabled. */
    expect(dte, "ATZ", "OK");
    di_set_connect_info_cb(fake_connect_info);
    fake_report = (v250_connect_report_t) { "V90", 52000, 31200, "LAPM", 1, true, true };
    expect(dte, "ATD1", "");
    di_on_connected(52000);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "CONNECT 52000") && !strstr(buf, "+MCR") && !strstr(buf, "+ER") && !strstr(buf, "+DR"),
          "reporting is off by default: only CONNECT");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);

    expect(dte, "AT+MR=1;+ER=1;+DR=1", "OK");
    di_on_connected(52000);
    collect(dte, buf, sizeof(buf), 150);
    {
        const char *mcr = strstr(buf, "+MCR: V90");
        const char *mrr = strstr(buf, "+MRR: 52000,31200");
        const char *er = strstr(buf, "+ER: LAPM");
        const char *dr = strstr(buf, "+DR: V42B");
        const char *con = strstr(buf, "CONNECT 52000");

        if (!(mcr && mrr && er && dr && con))
            printf("       got \"%s\"\n", buf);
        check(mcr && mrr && er && dr && con && mcr < mrr && mrr < er && er < dr && dr < con,
              "+MCR, +MRR, +ER, +DR then CONNECT, with both directional rates");
    }
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);

    /* A call that settled on no error control and no compression says so. */
    fake_report = (v250_connect_report_t) { "V32B", 14400, 0, "NONE", 0, false, false };
    di_on_connected(14400);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "+MCR: V32B") && strstr(buf, "+MRR: 14400\r") && strstr(buf, "+ER: NONE")
          && strstr(buf, "+DR: NONE"), "an unprotected call reports NONE/NONE");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);

    /* Result codes: ATQ1 silences the reports with the rest. */
    expect(dte, "ATQ1", "");
    di_on_connected(14400);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "+MCR") == NULL && strstr(buf, "+ER") == NULL,
          "ATQ1 suppresses the intermediate result codes");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    exchange(dte, "ATQ0", buf, sizeof(buf));

    di_set_connect_info_cb(NULL);
    close(dte);
    di_close();
}

int main(void)
{
    test_classic();
    test_split();
    test_v250_parameters();
    printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures != 0;
}
