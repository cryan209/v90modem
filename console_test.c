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

#include "profile_file.h"
#include "data_interface.h"
#include "at_help.h"
#include "line_monitor.h"

#include <math.h>

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
    usleep(150000);
    send_str(dte, "should be dropped, no call\r");
    collect(dte, buf, sizeof(buf), 200);
    check(engine_reads(got, sizeof(got), 100) == 0, "bytes sent before CONNECT are not payload");
    /* V.250 5.6.1: they abort the dial, which answers OK. */
    check(hangup_calls == 1 && strstr(buf, "OK"), "...they abort the dial in progress (OK)");
    hangup_calls = 0;
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "ATD5551234", "");

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
    fake_report = (v250_connect_report_t) { "V90", 52000, 31200, "LAPM", 1, true, true, 0, false };
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

    /* Courier &A: the call named in the CONNECT text; &M/&K/\\N reach +ES/+DS. */
    expect(dte, "AT+MR=0;+ER=0;+DR=0", "OK");
    expect(dte, "AT&A3", "OK");
    di_on_connected(52000);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "CONNECT 52000/ARQ/V90/LAPM/V42BIS\r") != NULL, "&A3: CONNECT 52000/ARQ/V90/LAPM/V42BIS");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "AT&A0&M5&K0", "OK");
    expect(dte, "AT+ES?", "+ES: 3,2,4");
    expect(dte, "AT+DS?", "+DS: 0,");
    expect(dte, "AT\\N3%C2", "OK");
    expect(dte, "AT+ES?", "+ES: 3,0,2");
    expect(dte, "AT+DS?", "+DS: 3,");
    expect(dte, "AT&M1", "ERROR");
    expect(dte, "AT\\N5", "ERROR");
    expect(dte, "AT&F2", "ERROR");
    expect(dte, "AT&F1", "OK");
    expect(dte, "AT&H1&R2&I0&B1", "OK");
    expect(dte, "ATI4", "&A0");
    expect(dte, "AT+MR=1;+ER=1;+DR=1", "OK");

    /* A call that settled on no error control and no compression says so. */
    fake_report = (v250_connect_report_t) { "V32B", 14400, 0, "NONE", 0, false, false, 0, false };
    di_on_connected(14400);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "+MCR: V32B") && strstr(buf, "+MRR: 14400\r") && strstr(buf, "+ER: NONE")
          && strstr(buf, "+DR: NONE"), "an unprotected call reports NONE/NONE");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);

    /* +ILRR: the rate the DTE set on its own port (+IPR=0), or the fixed +IPR. */
    {
        struct termios tio;

        tcgetattr(dte, &tio);
        cfsetispeed(&tio, B57600);
        cfsetospeed(&tio, B57600);
        tcsetattr(dte, TCSANOW, &tio);
    }
    expect(dte, "AT+ILRR=1", "OK");
    di_on_connected(14400);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "+DR: NONE\r\n+ILRR: 57600\r\n") && strstr(buf, "CONNECT 14400"),
          "+ILRR: the DTE's own port rate, after +DR and before CONNECT");
    if (!strstr(buf, "+ILRR: 57600"))
        printf("       got \"%s\"\n", buf);
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "AT+IPR=19200", "OK");
    di_on_connected(14400);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "+ILRR: 19200\r\n") != NULL, "+ILRR with a fixed +IPR reports that");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "AT+ILRR=0;+IPR=0", "OK");

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

/* A stand-in for the engine's +MS offer, so +MS?/+MS$ have something to read. */
static char fake_mode[16] = "v90";
static bool fake_automode = true;

static int fake_ms_set(const char *mode, bool automode)
{
    snprintf(fake_mode, sizeof(fake_mode), "%s", mode);
    fake_automode = automode;
    return 0;
}

static void fake_ms_get(char *mode, size_t len, bool *automode)
{
    snprintf(mode, len, "%s", fake_mode);
    *automode = fake_automode;
}

static void fake_ms_reset(void)
{
    snprintf(fake_mode, sizeof(fake_mode), "v90");
    fake_automode = true;
}

/* What the engine would say for ATI11. */
static bool fake_originate;

static void fake_link_detail(char *out, size_t len, bool *originate)
{
    *originate = fake_originate;
    snprintf(out, len, "Mode               v90 (offer V90)\r\nV.92               no");
}

/* The final result code is OK (a help page may mention ERROR in its text). */
static int final_ok(const char *buf)
{
    size_t n = strlen(buf);

    return n >= 6 && strcmp(buf + n - 6, "\r\nOK\r\n") == 0;
}

/* Courier-style "$" help and the ATI pages, through the real interpreter. */
static void test_help(void)
{
    static const char *const topics[] = { "", "D", "&", "+", "I", "S" };
    static const char *const cmds[] = { "AT$", "ATD$", "AT&$", "AT+$", "ATI$", "ATS$" };
    static const char *const titles[] = {
        "Command Quick Reference", "Dial Commands", "Ampersand Commands",
        "Extended Commands", "Identification and Diagnostics", "S-Registers"
    };
    const char *link = "/tmp/console_test_help";
    char buf[8192];
    char what[256];
    int dte;

    printf("$ help and ATI pages:\n");
    if (di_open(link) < 0) {
        failures++;
        return;
    }
    di_set_callbacks(cb_dial, cb_answer, cb_hangup, NULL);
    di_set_modulation_ops(fake_ms_set, fake_ms_get, fake_ms_reset);
    dte = open_dte(link);
    if (dte < 0) {
        failures++;
        di_close();
        return;
    }
    expect(dte, "ATE0", "OK");

    /* Each page arrives whole, every row of its table in it, then OK. */
    for (size_t i = 0; i < sizeof(topics) / sizeof(topics[0]); i++) {
        const at_help_entry_t *rows;
        size_t n;
        int all = 1;

        exchange(dte, cmds[i], buf, sizeof(buf));
        rows = at_help_table(topics[i], &n);
        for (size_t r = 0; rows && r < n; r++)
            if (rows[r].cmd[0] && !strstr(buf, rows[r].cmd)) {
                printf("       %s lacks \"%s\"\n", cmds[i], rows[r].cmd);
                all = 0;
            }
        snprintf(what, sizeof(what), "%s: \"%s\", every row, then OK", cmds[i], titles[i]);
        check(rows && all && strstr(buf, titles[i]) && final_ok(buf), what);
    }

    /* The help never names a command the interpreter does not take: every row
       with a probe answers OK to it. */
    for (size_t i = 0; i < sizeof(topics) / sizeof(topics[0]); i++) {
        const at_help_entry_t *rows;
        size_t n;

        rows = at_help_table(topics[i], &n);
        for (size_t r = 0; rows && r < n; r++) {
            char line[64];

            if (!rows[r].probe)
                continue;
            snprintf(line, sizeof(line), "AT%s", rows[r].probe);
            exchange(dte, line, buf, sizeof(buf));
            snprintf(what, sizeof(what), "%s$ lists %s: %s is accepted", topics[i], rows[r].cmd, line);
            if (!final_ok(buf))
                printf("       got \"%s\"\n", buf);
            check(final_ok(buf), what);
        }
    }

    /* S$ shows the live value, and help composes with other commands. */
    expect(dte, "ATS0=2S$", "S0   002");
    expect(dte, "ATS0=0", "OK");
    expect(dte, "ATS$", "S0   000");
    expect(dte, "ATE0$", "Command Quick Reference");
    expect(dte, "AT+MS$", "+MS");                   /* at_ms.c's page, untouched */
    /* V.250 5.6: an unrecognised command ends the line with ERROR -- the
       interpreter used to skip it and answer OK. */
    expect(dte, "ATS9?", "ERROR");
    expect(dte, "ATS9=3", "ERROR");
    expect(dte, "ATK", "ERROR");
    expect(dte, "AT+ZZZ", "ERROR");
    expect(dte, "AT&Q5", "ERROR");
    /* 5.6: commands before the bad one have run; the rest of the line is not. */
    expect(dte, "ATS0=1KS0=2", "ERROR");
    expect(dte, "ATS0?", "001");
    expect(dte, "ATS0=0", "OK");
    expect(dte, "ATQ0$", "Command Quick Reference"); /* Q0, then AT$ */
    expect(dte, "AT$Z", "OK");                      /* help then the next command */
    expect(dte, "ATE0", "OK");                      /* Z restored echo */

    /* ATY11: the line spectrum, from whatever audio the engine has fed. */
    expect(dte, "ATY11", "No line audio yet");
    expect(dte, "ATY", "ERROR");
    expect(dte, "ATY5", "ERROR");
    {
        int16_t x[160];

        for (int blk = 0; blk < 50; blk++) {
            for (int i = 0; i < 160; i++)          /* 5000 peak: -13.2 dBm0 at 1800 Hz */
                x[i] = (int16_t) lrint(5000.0 * sin(2.0 * M_PI * 1800.0 * (blk * 160 + i) / 8000.0));
            lm_feed(LM_RX, x, 160);
        }
    }
    exchange(dte, "ATY11", buf, sizeof(buf));
    check(strstr(buf, "Line Spectrum") && strstr(buf, "  1800 -13.2") && strstr(buf, "Total  Rx -13.2")
          && final_ok(buf), "ATY11 puts a -13.2 dBm0 1800 Hz tone in its band and the total");
    if (!strstr(buf, "  1800 -13.2"))
        printf("       got \"%s\"\n", buf);
    lm_reset();

    /* Hayes registers and commands that used to be missing. */
    expect(dte, "ATS2?", "043");
    expect(dte, "ATS12?", "050");
    expect(dte, "ATS1?", "000");
    di_on_ring();
    di_on_ring();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "ATS1?", "002");                    /* rings counted, S0=0: not answered */
    expect(dte, "ATH", "OK");
    expect(dte, "ATS1?", "000");
    expect(dte, "ATS7?", "060");
    send_str(dte, "A/");                            /* V.250 5.2.4: no terminator */
    collect(dte, buf, sizeof(buf), 250);
    check(strstr(buf, "060") && final_ok(buf), "A/ repeats the last command line at once");
    expect(dte, "AT&V", "Current Settings");
    expect(dte, "AT&V1", "ERROR");
    expect(dte, "AT+GCAP", "+GCAP: +FCLASS, +MS, +ES, +DS");

    /* ATI pages. */
    expect(dte, "ATI0", "v90modem");
    expect(dte, "ATI3", "v90modem ");
    expect(dte, "ATI4", "+ES: 3,0,2");
    expect(dte, "AT+ES=1,0,1", "OK");
    expect(dte, "ATI4", "+ES: 1,0,1");
    expect(dte, "ATI4", "E0 Q0 V1");
    expect(dte, "ATI4", "+MS: V90,1");
    expect(dte, "ATI7", "Modulations");
    expect(dte, "ATI6", "No call since power-on");
    expect(dte, "ATI11", "No call since power-on");
    expect(dte, "ATI5", "ERROR");

    di_set_connect_info_cb(fake_connect_info);
    di_set_link_detail_cb(fake_link_detail);
    fake_originate = true;
    fake_report = (v250_connect_report_t) { "V90", 52000, 31200, "LAPM", 1, true, true, 0, false };
    expect(dte, "ATS2=42S12=10", "OK");              /* escape on "***" after 0.2 s */
    expect(dte, "ATD1", "");
    di_on_connected(52000);
    collect(dte, buf, sizeof(buf), 150);
    send_str(dte, "hello");
    usleep(300000);
    send_str(dte, "+++");                           /* no longer the escape: payload */
    {
        char got[16];

        check(engine_reads(got, 8, 600) == 8 && !memcmp(got, "hello+++", 8),
              "with S2=42, \"+++\" is payload");
    }
    di_write_data((const uint8_t *) "worlds", 6);
    collect(dte, buf, sizeof(buf), 100);
    /* Live: the engine pushes a renegotiated rate and a retrain in progress. */
    {
        v250_connect_report_t now = { "V90", 48000, 26400, "LAPM", 1, true, true, 0, false };

        di_update_link(&now, "Mode               v90 (offer V90)\r\nData-mode retrains 1", "retraining");
    }
    usleep(300000);                                 /* S12 guard, then escape to ask */
    send_str(dte, "***");
    collect(dte, buf, sizeof(buf), 500);
    check(strstr(buf, "OK") != NULL, "\"***\" escapes with S2=42 and a 0.2 s S12 guard");
    exchange(dte, "ATI6", buf, sizeof(buf));
    check(strstr(buf, "Originate, retraining") && strstr(buf, "TX 48000  RX 26400"),
          "ATI6 mid-call shows the engine's latest push: retraining, renegotiated rates");
    if (!strstr(buf, "retraining"))
        printf("       got \"%s\"\n", buf);
    exchange(dte, "ATI11", buf, sizeof(buf));
    check(strstr(buf, "(live)") && strstr(buf, "Data-mode retrains 1"), "ATI11 mid-call is live");
    di_on_disconnected_cause("Remote (call cleared)", 1);
    collect(dte, buf, sizeof(buf), 150);
    exchange(dte, "ATI6", buf, sizeof(buf));
    check(strstr(buf, "Originate, ended") && strstr(buf, "Modulation         V90")
          && strstr(buf, "TX 48000  RX 26400") && strstr(buf, "V.42 LAPM")
          && strstr(buf, "V.42bis TX RX") && strstr(buf, "Octets to line     8\r")
          && strstr(buf, "Octets to DTE      6\r") && strstr(buf, "Remote (call cleared)"),
          "ATI6 after the call: direction, carrier, rates, protocols, octets, cause");
    if (!strstr(buf, "Remote (call cleared)"))
        printf("       got \"%s\"\n", buf);
    expect(dte, "ATI11", "(at end of call)");

    /* An answered call that the DTE ended itself. */
    fake_report = (v250_connect_report_t) { "V34", 28800, 0, "NONE", 0, false, false, 0, false };
    fake_originate = false;
    di_on_ring();
    collect(dte, buf, sizeof(buf), 100);
    di_on_connected(28800);
    collect(dte, buf, sizeof(buf), 150);
    exchange(dte, "ATI6", buf, sizeof(buf));        /* on the combined port this is data */
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    exchange(dte, "ATI6", buf, sizeof(buf));
    check(strstr(buf, "Answer, ended") && strstr(buf, "Rate               28800")
          && strstr(buf, "None (V.14)") && strstr(buf, "Compression        None"),
          "ATI6 for an answered, unprotected call");

    /* A call the modem gave up on before CONNECT replaces the older record. */
    di_on_disconnected_cause("Modem (protocol or training failure)", 1);
    collect(dte, buf, sizeof(buf), 150);
    exchange(dte, "ATI6", buf, sizeof(buf));
    check(strstr(buf, "Originate, failed before data mode")
          && strstr(buf, "Modem (protocol or training failure)") && !strstr(buf, "28800"),
          "ATI6 after a call that failed before data mode");
    expect(dte, "ATI11", "failed before data mode");

    di_set_connect_info_cb(NULL);
    di_set_link_detail_cb(NULL);
    close(dte);
    di_close();
}

/* Call progress: V.250 6.2.5-6.2.7, 6.3.1 (Table 8), 6.3.10 and 5.6.1. */
static void test_call_progress(void)
{
    const char *link = "/tmp/console_test_progress";
    char buf[4096];
    int dte;
    int h;

    printf("Call progress, X/V/Q, abort and S7:\n");
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
    expect(dte, "ATZ2", "ERROR");                   /* profiles are 0 and 1 */
    expect(dte, "ATI4", " X4 ");                    /* the default */

    /* CONNECT as X, V and Q say. */
    expect(dte, "ATD1", "");
    di_on_connected(31200);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "\r\nCONNECT 31200\r\n") != NULL, "X4: CONNECT 31200");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "ATX0", "OK");
    expect(dte, "ATD1", "");
    di_on_connected(31200);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "\r\nCONNECT\r\n") && !strstr(buf, "31200"), "X0: CONNECT without text");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "ATX4V0", "0\r");
    expect(dte, "ATD1", "");
    di_on_connected(31200);
    collect(dte, buf, sizeof(buf), 150);
    check(!strcmp(buf, "1\r"), "V0: numeric 1 for CONNECT");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "3\r") != NULL, "V0: numeric 3 for NO CARRIER");
    expect(dte, "ATV1Q1", "");
    expect(dte, "ATD1", "");
    di_on_connected(31200);
    collect(dte, buf, sizeof(buf), 150);
    check(buf[0] == '\0', "Q1: no CONNECT at all");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    expect(dte, "ATQ0", "OK");

    /* A dialled call that never answers: the result code follows X. */
    expect(dte, "ATD1", "");
    di_on_call_failed(486);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "BUSY") != NULL, "X4: SIP 486 is BUSY");
    expect(dte, "ATI6", "Busy (SIP 486)");
    expect(dte, "ATX0D1", "");
    di_on_call_failed(486);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "NO CARRIER") && !strstr(buf, "BUSY"), "X0: busy detection off, NO CARRIER");
    expect(dte, "ATX4D1", "");
    di_on_call_failed(503);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "NO DIALTONE") != NULL, "X4: SIP 503 is NO DIALTONE");
    expect(dte, "ATX3D1", "");
    di_on_call_failed(0);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "NO CARRIER") && !strstr(buf, "DIALTONE"), "X3: dial tone detection off, NO CARRIER");
    expect(dte, "ATX4D1", "");
    di_on_call_failed(480);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "NO CARRIER") != NULL, "SIP 480 without @ is NO CARRIER");
    expect(dte, "ATD@1", "");
    di_on_call_failed(480);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "NO ANSWER") != NULL, "SIP 480 with @ is NO ANSWER");

    /* Any character aborts a dial in progress (after 125 ms): OK, hung up. */
    h = hangup_calls;
    expect(dte, "ATD1", "");
    usleep(200000);
    send_str(dte, "x");
    collect(dte, buf, sizeof(buf), 250);
    check(strstr(buf, "OK") && hangup_calls == h + 1, "a character aborts the dial: OK, call ended");
    di_on_disconnected();                           /* the engine's teardown follows */
    collect(dte, buf, sizeof(buf), 150);
    check(!strstr(buf, "NO CARRIER"), "...and the teardown adds no NO CARRIER");
    expect(dte, "ATI6", "Aborted by the DTE");

    /* ATH ends a dial that has not connected. */
    h = hangup_calls;
    expect(dte, "ATD1", "");
    usleep(150000);
    expect(dte, "ATH", "OK");
    check(hangup_calls == h + 1, "ATH ends a dial still in progress");

    /* S7: no connection in time is NO CARRIER and a hang-up. */
    h = hangup_calls;
    expect(dte, "ATS7=1D1", "");
    collect(dte, buf, sizeof(buf), 1500);
    check(strstr(buf, "NO CARRIER") && hangup_calls == h + 1, "S7=1: NO CARRIER after a second");
    expect(dte, "ATI6", "Timeout (S7)");
    expect(dte, "ATS7=60", "OK");

    /* Caller ID (V.253 9.2.3.1) and answering on S0. */
    expect(dte, "AT+VCID=?", "+VCID: (0,1)");
    expect(dte, "AT+VCID?", "+VCID: 0");
    expect(dte, "AT+VCID=2", "ERROR");               /* no raw ICLID packet on SIP */
    di_set_caller_id("6004", "Apple Modem");
    di_on_ring();
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "RING") && !strstr(buf, "NMBR"), "+VCID=0: RING alone");
    expect(dte, "ATH", "OK");
    expect(dte, "AT+VCID=1", "OK");
    di_set_caller_id("6004", "Apple Modem");
    di_on_ring();
    collect(dte, buf, sizeof(buf), 150);
    {
        const char *ring = strstr(buf, "RING");
        const char *date = strstr(buf, "DATE = ");
        const char *nmbr = strstr(buf, "NMBR = 6004");
        const char *name = strstr(buf, "NAME = Apple Modem");

        check(ring && date && strstr(buf, "TIME = ") && nmbr && name && ring < date && date < nmbr,
              "+VCID=1: RING, then DATE, TIME, NMBR, NAME (spaces round '=')");
        if (!nmbr)
            printf("       got \"%s\"\n", buf);
    }
    di_on_ring();
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "RING") && !strstr(buf, "NMBR"), "...after the first ring only");
    expect(dte, "ATH", "OK");
    di_set_caller_id("P", NULL);
    di_on_ring();
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "NMBR = P") && !strstr(buf, "NAME"), "a private caller is NMBR = P");
    expect(dte, "ATH+VCID=0", "OK");

    {
        int a = answer_calls;

        di_set_auto_answer(2);
        expect(dte, "ATS0?", "002");
        expect(dte, "ATZ", "OK");
        expect(dte, "ATS0?", "002");                /* the factory value */
        di_set_caller_id("6004", NULL);
        di_on_ring();
        collect(dte, buf, sizeof(buf), 100);
        check(answer_calls == a, "S0=2: not answered on the first ring");
        di_on_ring();
        collect(dte, buf, sizeof(buf), 100);
        check(answer_calls == a + 1, "S0=2: answered on the second");
        usleep(150000);                             /* 5.6.1: 125 ms before an abort counts */
        expect(dte, "ATH", "OK");
        di_set_auto_answer(0);
        expect(dte, "ATS0?", "000");
        di_set_caller_id("6004", NULL);
        di_on_ring();
        di_on_ring();
        di_on_ring();
        collect(dte, buf, sizeof(buf), 150);
        check(answer_calls == a + 1, "S0=0: never answered by itself");
        expect(dte, "ATA", "");
        check(answer_calls == a + 2, "...ATA answers the ringing call");
        expect(dte, "ATH", "OK");
    }

    /* Stored numbers: V.250 6.3.15 +ASTO and D S=<n> (6.3.1.8), the Courier's
       &Zn and DSn on the same slots, and Hayes DL. */
    expect(dte, "AT+ASTO=?", "+ASTO: (0-9),(40)");
    expect(dte, "AT+ASTO=3,555-1234", "OK");
    expect(dte, "AT+ASTO?", "+ASTO: 3,5551234");    /* '-' is not storable */
    expect(dte, "AT+ASTO=3,\"64 9 555\"", "OK");
    expect(dte, "AT+ASTO?", "+ASTO: 3,649555");
    expect(dte, "AT+ASTO=10,1", "ERROR");
    expect(dte, "AT+ASTO=1,12345678901234567890123456789012345678901", "ERROR");
    expect(dte, "ATDS=3", "");
    check(!strcmp(last_number, "649555"), "D S=3 dials what +ASTO stored");
    usleep(150000);
    expect(dte, "ATH", "OK");
    expect(dte, "AT&Z1=6001", "OK");
    expect(dte, "AT&Z1?", "6001");
    expect(dte, "ATD9S1", "");                      /* dial chars may precede S */
    check(!strcmp(last_number, "96001"), "DSn dials &Zn's number after the preceding digits");
    usleep(150000);
    expect(dte, "ATH", "OK");
    expect(dte, "ATDL?", "96001");
    snprintf(last_number, sizeof(last_number), "-");
    expect(dte, "ATDL", "");
    check(!strcmp(last_number, "96001"), "DL redials the last number");
    usleep(150000);
    expect(dte, "ATH", "OK");
    expect(dte, "ATDS=5", "ERROR");                 /* nothing stored there */
    expect(dte, "ATDS=10", "ERROR");
    expect(dte, "ATD", "ERROR");                    /* no number: nothing a SIP call can do */
    expect(dte, "ATZ", "OK");
    expect(dte, "AT&Z1?", "6001");                  /* stored numbers survive ATZ */

    close(dte);
    di_close();
}

static void write_file(const char *path, const char *text)
{
    FILE *f = fopen(path, "w");

    if (f) {
        fputs(text, f);
        fclose(f);
    }
}

static size_t read_file(const char *path, char *buf, size_t max)
{
    FILE *f = fopen(path, "r");
    size_t n = 0;

    buf[0] = '\0';
    if (f) {
        n = fread(buf, 1, max - 1, f);
        buf[n] = '\0';
        fclose(f);
    }
    return n;
}

/* Open the console with a --profile file, as sip_modem.c does at start-up. */
static int profile_session(const char *link, const char *file)
{
    if (di_open(link) < 0)
        return -1;
    di_set_callbacks(cb_dial, cb_answer, cb_hangup, NULL);
    di_set_modulation_ops(fake_ms_set, fake_ms_get, fake_ms_reset);
    fake_ms_reset();
    di_load_profile(file);
    return open_dte(link);
}

static int fake_lim[4];
static int fake_lim_ok = 0;

static int fake_ms_limits(const char *mode, bool automode, int min_tx, int max_tx,
                          int min_rx, int max_rx)
{
    if (fake_lim_ok < 0)
        return -1;
    fake_ms_set(mode, automode);
    fake_lim[0] = min_tx;
    fake_lim[1] = max_tx;
    fake_lim[2] = min_rx;
    fake_lim[3] = max_rx;
    return 0;
}

/* V.250 6.4.1: +MS's rate subparameters reach the engine, the engine may
 * refuse them, and a call it reports as outside them is not a CONNECT. */
static void test_ms_limits(void)
{
    const char *link = "/tmp/console_test_mslim";
    char buf[1024];
    int dte;

    printf("+MS rate bounds:\n");
    if (di_open(link) < 0) {
        failures++;
        return;
    }
    di_set_callbacks(cb_dial, cb_answer, cb_hangup, NULL);
    di_set_modulation_ops(fake_ms_set, fake_ms_get, fake_ms_reset);
    di_set_modulation_limits_op(fake_ms_limits);
    dte = open_dte(link);
    expect(dte, "ATE0", "OK");
    expect(dte, "AT+MS=V34,1,4800,9600,2400,7200", "OK");
    check(fake_lim[0] == 4800 && fake_lim[1] == 9600 && fake_lim[2] == 2400 && fake_lim[3] == 7200
          && !strcmp(fake_mode, "v34"), "tx and rx bounds reach the engine");
    expect(dte, "AT+MS=V34,1,0,9600", "OK");
    check(fake_lim[2] == 0 && fake_lim[3] == 9600, "the four-value form bounds both directions");
    fake_lim_ok = -1;
    expect(dte, "AT+MS=V32B,0,10000,11000", "ERROR");
    expect(dte, "AT+MS?", "+MS: V34,1,0,9600,0,9600");      /* unchanged */
    fake_lim_ok = 0;

    di_set_connect_info_cb(fake_connect_info);
    fake_report = (v250_connect_report_t) { "V22B", 1200, 0, "NONE", 0, false, false, 0, true };
    expect(dte, "ATD1", "");
    di_on_connected(1200);
    collect(dte, buf, sizeof(buf), 150);
    check(!strstr(buf, "CONNECT"), "a call the engine refused is not reported as CONNECT");
    di_on_disconnected();
    collect(dte, buf, sizeof(buf), 150);
    if (!strstr(buf, "NO CARRIER")) printf("       got \"%s\"\n", buf);
    check(strstr(buf, "NO CARRIER") != NULL, "  ...it ends with NO CARRIER");
    di_set_connect_info_cb(NULL);
    di_set_modulation_limits_op(NULL);
    close(dte);
    di_close();
}

static int fake_pmhr_result = -1;

static int fake_pmhr(void)
{
    return fake_pmhr_result;
}

/* V.250 6.8.4 +PMHR through the console: ERROR unless the engine says MH is
 * armed on a call; the answer arrives later as an information line. */
static void test_pmhr(void)
{
    const char *link = "/tmp/console_test_pmhr";
    char buf[1024];
    int dte;

    printf("+PMHR and the V.92 parameters:\n");
    if (di_open(link) < 0) {
        failures++;
        return;
    }
    dte = open_dte(link);
    expect(dte, "ATE0", "OK");
    expect(dte, "AT+PMHR", "ERROR");                /* no engine at all */
    di_set_pmhr_cb(fake_pmhr);
    fake_pmhr_result = -1;
    expect(dte, "AT+PMHR", "ERROR");                /* not armed / idle */
    fake_pmhr_result = 0;
    expect(dte, "AT+PMHR", "OK");
    di_report_pmhr(5);
    collect(dte, buf, sizeof(buf), 150);
    check(strstr(buf, "\r\n+PMHR: 5\r\n") != NULL, "the far end's answer: +PMHR: 5");
    expect(dte, "AT+PMH=0;+PMHT=6;+PIG=0;+PCW=1", "OK");
    expect(dte, "AT+PMH?;+PMHT?", "+PMHT: 6");
    expect(dte, "ATI4", "+PCW: 1");
    expect(dte, "AT+PQC=0", "ERROR");
    expect(dte, "AT+PMHF", "ERROR");
    expect(dte, "AT&F", "OK");
    expect(dte, "AT+PMH?", "+PMH: 1");
    di_set_pmhr_cb(NULL);
    close(dte);
    di_close();
}

/* profile_file.c alone: both syntaxes, round trips, and what it refuses. */
static void test_profile_file(void)
{
    static pf_doc_t doc, back;
    static char text[16384];
    char err[200];
    char at[PF_LINE];

    printf("Profile file syntax (Cisco-style, JSON, legacy AT lines):\n");
    pf_doc_init(&doc);
    doc.power_on = 1;
    pf_add(&doc.global, 0, "number 3 \"9,55\\225\"");
    pf_add(&doc.profile[1], 0, "no echo");
    pf_add(&doc.profile[1], 0, "result-codes 4");
    pf_add(&doc.profile[1], 0, "dial pulse");
    pf_add(&doc.profile[1], 0, "s-register 0 2");
    pf_add(&doc.profile[1], 0, "+MS V34,1,300,0,300,0");
    pf_add(&doc.profile[1], 0, "at &K3");
    for (int json = 0; json < 2; json++) {
        int len = pf_render(&doc, json != 0, text, sizeof(text));
        bool same;

        check(len > 0 && pf_parse(text, &back, err, sizeof(err)) == 0,
              json ? "JSON renders and parses back" : "Cisco-style renders and parses back");
        same = back.power_on == 1 && back.global.n == 1 && back.profile[1].n == 6
               && !back.profile[0].present && !strcmp(back.global.line[0], doc.global.line[0]);
        for (int i = 0; same && i < back.profile[1].n; i++) {
            bool found = false;

            for (int k = 0; k < doc.profile[1].n; k++)
                found |= !strcmp(back.profile[1].line[i], doc.profile[1].line[k]);
            same = found;
        }
        check(same, json ? "  ...to the same settings (JSON)" : "  ...to the same settings (Cisco)");
    }
    check(strstr(text, "\"9,55\\\"5\"") != NULL, "JSON shows a dial string's quote as \\\", not \\22");
    check(pf_setting_to_at("no verbose", at, sizeof(at)) == 0 && !strcmp(at, "V0"), "no verbose -> V0");
    check(pf_setting_to_at("+ES 3,0,2", at, sizeof(at)) == 0 && !strcmp(at, "+ES=3,0,2"), "+ES 3,0,2 -> +ES=3,0,2");
    check(pf_setting_to_at("s-register 7 60", at, sizeof(at)) == 0 && !strcmp(at, "S7=60"), "s-register 7 60 -> S7=60");
    check(pf_parse("profile 0\n speaker on\n", &back, err, sizeof(err)) < 0
          && strstr(err, "line 2") && strstr(err, "speaker"), "an unknown setting is refused with its line");
    check(pf_parse("echo\n", &back, err, sizeof(err)) < 0, "a setting before any profile is refused");
    check(pf_parse("{ \"profiles\": { \"0\": { \"echo\": 1.5 } } }", &back, err, sizeof(err)) < 0,
          "JSON: a fractional number is refused");
    check(pf_parse("{ \"profiles\": { \"0\": { \"s-registers\": { \"0\": 3 }, \"at\": [\"&K3\"] } } }",
                   &back, err, sizeof(err)) == 0 && back.profile[0].n == 2
          && !strcmp(back.profile[0].line[0], "s-register 0 3") && !strcmp(back.profile[0].line[1], "at &K3"),
          "JSON: s-registers object and at list");
    check(pf_parse("# old\nATE0X2\nATS0=3\n", &back, err, sizeof(err)) == 0 && back.power_on == 0
          && back.profile[0].n == 2 && !strcmp(back.profile[0].line[1], "at S0=3"),
          "legacy AT-line file reads as profile 0");
    check(pf_path_is_json("x/p.JSON") && !pf_path_is_json("p.cfg"), "file syntax chosen by extension");
}

/* Stored profiles: &Wn, Zn, &Yn, &F, &V and --profile (di_load_profile). */
static void test_profile(void)
{
    const char *link = "/tmp/console_test_profile";
    const char *file = "/tmp/console_test_profile.cfg";
    const char *jfile = "/tmp/console_test_profile.json";
    char buf[16384];
    int dte;

    printf("Stored profiles (&Wn, Zn, &Yn, &F, &V, --profile):\n");
    unlink(file);
    unlink(jfile);
    if ((dte = profile_session(link, file)) < 0) {   /* no file yet: nothing stored */
        failures++;
        di_close();
        return;
    }
    expect(dte, "ATE0", "OK");
    expect(dte, "AT&V", "none: ATZ restores the factory settings");
    expect(dte, "ATS0=3X2+ES=1,0,1;+VCID=1;+EWIND=6;+MS=V34;+ASTO=2,555", "OK");
    expect(dte, "AT&A2", "OK");
    expect(dte, "AT&W", "OK");                      /* &W is &W0 */
    check(access(file, R_OK) == 0, "&W writes the --profile file");
    read_file(file, buf, sizeof(buf));
    check(strstr(buf, "profile 0\n no echo\n") && strstr(buf, " s-register 0 3\n")
          && strstr(buf, " +ES 1,0,1\n") && strstr(buf, "number 2 \"555\"\n")
          && strstr(buf, "power-on-profile 0\n") && strstr(buf, "\nend\n")
          && strstr(buf, " connect-suffix 2\n"),
          "  ...in the Cisco-style syntax");
    expect(dte, "ATS0=5X3+MS=V90", "OK");
    expect(dte, "AT&W1", "OK");
    expect(dte, "AT&W2", "ERROR");                  /* profiles are 0 and 1 */
    expect(dte, "ATS0=0X4+ES=3,0,2;+VCID=0;+EWIND=15;+ASTO=2,1", "OK");
    expect(dte, "ATZ", "OK");
    expect(dte, "ATS0?", "003");
    expect(dte, "ATI4", " X2 ");
    expect(dte, "ATI4", "&A2");
    expect(dte, "AT+ES?", "+ES: 1,0,1");
    expect(dte, "AT+VCID?", "+VCID: 1");
    expect(dte, "AT+EWIND?", "+EWIND: 6,0");
    expect(dte, "AT+MS?", "+MS: V34");
    expect(dte, "AT+ASTO?", "+ASTO: 2,1");          /* numbers are not in a profile */
    expect(dte, "ATZ1", "OK");
    expect(dte, "ATS0?", "005");
    expect(dte, "AT+MS?", "+MS: V90");
    expect(dte, "AT&F", "OK");                      /* factory, not a stored profile */
    expect(dte, "ATS0?", "000");
    expect(dte, "AT+ES?", "+ES: 3,0,2");
    expect(dte, "ATE0Z0", "OK");
    expect(dte, "ATS0?", "003");
    expect(dte, "AT&Y1", "OK");
    expect(dte, "AT&Y2", "ERROR");
    expect(dte, "AT&V", "power-on &Y1");
    expect(dte, "AT&V", " +MS V90");
    expect(dte, "AT&V", "number 2 \"1\"");          /* &Y wrote the numbers as they are now */
    close(dte);
    di_close();

    /* A restart: &Y1 is what comes back, before any ATZ. */
    if ((dte = profile_session(link, file)) < 0) {
        failures++;
        di_close();
        return;
    }
    expect(dte, "ATS0?", "005");                    /* E0 is stored too: no echo */
    expect(dte, "AT+MS?", "+MS: V90");
    expect(dte, "AT+ASTO?", "+ASTO: 2,1");
    expect(dte, "ATZ0", "OK");
    expect(dte, "AT+ES?;+EWIND?", "+EWIND: 6,0");
    close(dte);
    di_close();

    /* A hand-written JSON file: Q1 inside a profile does not hide errors,
     * a refused setting is skipped, and &W keeps the JSON syntax. */
    write_file(jfile,
               "{ \"power-on-profile\": 0,\n"
               "  \"numbers\": { \"4\": \"123\" },\n"
               "  \"profiles\": { \"0\": { \"quiet\": true, \"+ES\": \"9,9,9\",\n"
               "                       \"s-registers\": { \"0\": 7 }, \"echo\": false } } }\n");
    if ((dte = profile_session(link, jfile)) < 0) {
        failures++;
        di_close();
        return;
    }
    expect(dte, "ATQ0", "OK");
    expect(dte, "ATS0?", "007");
    expect(dte, "AT+ES?", "+ES: 3,0,2");            /* the refused setting left the default */
    expect(dte, "AT+ASTO?", "+ASTO: 4,123");
    expect(dte, "AT&W", "OK");
    read_file(jfile, buf, sizeof(buf));
    check(buf[0] == '{' && strstr(buf, "\"s-registers\": {") && strstr(buf, "\"quiet\": false"),
          "&W rewrites a .json file as JSON");
    close(dte);
    di_close();

    /* A file that does not parse is left alone: &W is ERROR. */
    write_file(file, "profile 0\n warp-drive engaged\n");
    if ((dte = profile_session(link, file)) < 0) {
        failures++;
        di_close();
        return;
    }
    expect(dte, "ATE0", "OK");
    expect(dte, "AT&W", "ERROR");
    read_file(file, buf, sizeof(buf));
    check(strstr(buf, "warp-drive") != NULL, "  ...and the file is untouched");
    close(dte);
    di_close();

    /* An &W that cannot be kept is ERROR. */
    if ((dte = profile_session(link, "/nonexistent-dir/profile.cfg")) < 0) {
        failures++;
        di_close();
        return;
    }
    expect(dte, "ATE0", "OK");
    expect(dte, "AT&W", "ERROR");
    expect(dte, "AT&V", "none: ATZ restores");
    close(dte);
    di_close();
    unlink(file);
    unlink(jfile);
}

int main(void)
{
    test_classic();
    test_split();
    test_v250_parameters();
    test_help();
    test_call_progress();
    test_ms_limits();
    test_pmhr();
    test_profile_file();
    test_profile();
    printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures != 0;
}
