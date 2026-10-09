/*
 * fax_class_test.c -- T.31 fax service class over the AT interface.
 *
 * The class 1 action commands (T.31 8.3: +FTS/+FRS/+FTM/+FRM/+FTH/+FRH) are
 * dispatched by SpanDSP's process_class1_cmd() to a class 1 handler.  With no
 * handler registered every one of them answers ERROR, however well the
 * command parses -- so a test that only checks AT+FCLASS=? proves nothing.
 * This drives the real PTY the DTE sees, takes the modem off hook, and
 * requires that a transmit command both reports CONNECT and puts a fax
 * carrier on the wire.
 */

#include "data_interface.h"
#include "test_tmp.h"
#include <spandsp.h>
#include <spandsp/t31.h>
#include <spandsp/v34.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <termios.h>
#include <pthread.h>

static int   dte_fd = -1;
static int   dial_seen = 0;
static int   failures = 0;

static void on_dial(const char *uri, void *u)   { (void)uri; (void)u; dial_seen = 1; }
static void on_answer(void *u)                  { (void)u; }
static void on_hangup(void *u)                  { (void)u; }

/* Read whatever the modem has sent, for up to timeout_ms. */
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

static void send_cmd(const char *cmd)
{
    char line[128];
    int n = snprintf(line, sizeof(line), "%s\r", cmd);
    if (write(dte_fd, line, (size_t)n) != n)
        perror("write");
}

static void expect(const char *cmd, const char *want, int timeout_ms)
{
    char resp[4096];

    send_cmd(cmd);
    drain(resp, sizeof(resp), timeout_ms);
    if (strstr(resp, want)) {
        printf("  ok   %-14s -> %s\n", cmd, want);
    } else {
        printf("  FAIL %-14s -> wanted \"%s\", got \"%s\"\n", cmd, want, resp);
        failures++;
    }
}

/*
 * The engine pumps a 20 ms frame every 20 ms in both directions, and T.31's
 * state machine advances on those samples rather than on wall time: +FTS
 * counts transmitted silence, +FRS listens, and +FTM/+FTH report CONNECT only
 * once the modulator has started.  Without this thread the commands are
 * accepted and then never complete, which is a different failure from being
 * rejected -- so run the audio the way a call does.
 */
static pthread_mutex_t pump_mtx = PTHREAD_MUTEX_INITIALIZER;
static double          pump_energy = 0.0;
static long            pump_samples = 0;
static volatile int    pump_running = 1;

static void *pump_thread(void *arg)
{
    (void)arg;
    while (pump_running) {
        int16_t tx[160];
        int16_t rx[160];
        double energy = 0.0;

        memset(tx, 0, sizeof(tx));
        di_fax_tx(tx, 160);
        for (int i = 0; i < 160; i++)
            energy += (double)tx[i] * tx[i];

        pthread_mutex_lock(&pump_mtx);
        pump_energy += energy;
        pump_samples += 160;
        pthread_mutex_unlock(&pump_mtx);

        /* Quiet line into the receiver: enough for +FRS, and it keeps the
         * receive side's sample clock running. */
        memset(rx, 0, sizeof(rx));
        di_fax_rx(rx, 160);

        usleep(20000);
    }
    return NULL;
}

/* RMS of what the fax transmitter has put on the wire since the last call. */
static double tx_rms(void)
{
    double energy;
    long samples;

    pthread_mutex_lock(&pump_mtx);
    pump_energy = 0.0;
    pump_samples = 0;
    pthread_mutex_unlock(&pump_mtx);

    usleep(500000);

    pthread_mutex_lock(&pump_mtx);
    energy = pump_energy;
    samples = pump_samples;
    pthread_mutex_unlock(&pump_mtx);

    return samples ? sqrt(energy / (double)samples) : 0.0;
}

typedef struct { uint8_t bytes[32768]; int len; } v34_capture_t;
static int v34_capture(void *user, const uint8_t *buf, size_t len)
{
    v34_capture_t *c = user;
    if (len <= sizeof(c->bytes) - c->len) { memcpy(c->bytes + c->len, buf, len); c->len += len; }
    return 0;
}
static int v34_control(t31_state_t *s, void *u, int op, const char *num)
{ (void)s; (void)u; (void)op; (void)num; return 0; }
static void v34_check(int good, const char *what)
{ printf("  %s %s\n", good ? "ok  " : "FAIL", what); if (!good) failures++; }
static void v34_send_frame(t31_state_t *s, const uint8_t *frame, int len)
{
    uint8_t data[600]; int n = 0;
    for (int i = 0; i < len; i++) {
        int c = frame[i];
        if (c == 0x10 || c == 0x11 || c == 0x13) { data[n++] = 0x10; data[n++] = c == 0x11 ? 0x51 : c == 0x13 ? 0x53 : c; }
        else data[n++] = c;
    }
    data[n++] = 0x10; data[n++] = 0x03;
    /* Fragment every escape pair across separate DTE writes. */
    for (int i = 0; i < n; i++) t31_at_rx(s, (const char *)&data[i], 1);
}
static int v34_frame_matches(v34_capture_t *c, const uint8_t *frame, int len, int ferr)
{
    uint8_t out[300]; int n = 0;
    for (int i = 0; i < c->len; i++) {
        int b = c->bytes[i];
        if (b == 0x10 && i + 1 < c->len) {
            b = c->bytes[++i];
            if (b == 3 || b == 7) {
                if (b == (ferr ? 7 : 3) && n == len + 2 && !memcmp(out, frame, len)
                    && (ferr || crc_itu16_check(out, n))) return 1;
                n = 0; continue;
            }
            if (b == 0x51) b = 0x11;
            else if (b == 0x53) b = 0x13;
            else if (b != 0x10) continue;
        }
        if (n < (int)sizeof(out)) out[n++] = b;
    }
    return 0;
}
static void test_v34_class1(void)
{
    v34_capture_t captures[2] = {{{0},0},{{0},0}};
    t31_state_t *a = t31_init(NULL, v34_capture, &captures[0], v34_control, NULL, NULL, NULL);
    t31_state_t *b = t31_init(NULL, v34_capture, &captures[1], v34_control, NULL, NULL, NULL);
    v34_check(a && b, "Class 1 Annex B terminals start");
    if (!a || !b) return;
    const char *limits = "AT+FCLASS=1.0\rAT+F34=12,5,1\r";
    t31_at_rx(a,limits,strlen(limits));
    v34_check(t31_v34hdx_start(a,9600,true) < 0,
              "the negotiated rate cannot violate the retained F34 minimum");
    const char *setup = "AT+FCLASS=1.0\rAT+F34=12,1,1\r";
    t31_at_rx(a, setup, strlen(setup)); t31_at_rx(b, setup, strlen(setup));
    v34_check(t31_v34hdx_start(a, 9600, true) == 0 && t31_v34hdx_start(b, 9600, false) == 0,
              "trained Class 1 transport attaches in both roles");
    v34_check(memmem(captures[0].bytes, captures[0].len, "+F34:4,1", 8) != NULL,
              "negotiated primary/control rates precede CONNECT");
    captures[0].len = captures[1].len = 0;
    uint8_t frames[2][7] = {{0xFF,0x13,0x84,0x10,0x11,0x13,0x51},{0xFF,0x13,0x80,0x13,0x10,0x11,0x53}};
    v34_send_frame(a, frames[0], 7); v34_send_frame(b, frames[1], 7);
    for (int i = 0; i < 2000; i++) { int x = t31_v34hdx_get_bit(a), y = t31_v34hdx_get_bit(b); t31_v34hdx_put_bit(a,y); t31_v34hdx_put_bit(b,x); }
    v34_check(v34_frame_matches(&captures[0], frames[1], 7, 0) && v34_frame_matches(&captures[1], frames[0], 7, 0),
              "duplex HDLC frames preserve shielded octets and received FCS");
    const char pri[] = {0x10,0x6B}; t31_at_rx(a, pri, 2);
    for (int tick = 0; tick < 20; tick++) {
        for (int bit = 0; bit < 24; bit++) {
            int x = t31_v34hdx_get_bit(a);
            t31_v34hdx_put_bit(b,x);
            if (t31_v34hdx_get_mode(b) != V34_HALF_DUPLEX_PRIMARY_CHANNEL) t31_v34hdx_put_bit(a,t31_v34hdx_get_bit(b));
        }
        t31_v34hdx_advance(a,160); t31_v34hdx_advance(b,160);
    }
    v34_check(t31_v34hdx_get_mode(a) == V34_HALF_DUPLEX_PRIMARY_CHANNEL && t31_v34hdx_get_mode(b) == V34_HALF_DUPLEX_PRIMARY_CHANNEL,
              "DLE pri exchanges forty marks and waits for recipient flags to stop");
    t31_v34hdx_set_channel(a,V34_HALF_DUPLEX_PRIMARY_CHANNEL); t31_v34hdx_set_channel(b,V34_HALF_DUPLEX_PRIMARY_CHANNEL);
    captures[1].len = 0;
    uint8_t image[260] = {0xFF,0x03,0x06,0}; for (int i = 4; i < 260; i++) image[i] = i;
    v34_send_frame(a,image,260);
    for (int i = 0; i < 4000; i++) t31_v34hdx_put_bit(b,t31_v34hdx_get_bit(a));
    v34_check(v34_frame_matches(&captures[1],image,260,0), "ECM image frame crosses the primary channel without alteration");
    captures[1].len = 0;
    const char pause[] = {0x10,0x13}, resume[] = {0x10,0x11};
    t31_at_rx(b,pause,2);
    hdlc_tx_state_t *tx = hdlc_tx_init(NULL,false,2,false,NULL,NULL);
    hdlc_tx_flags(tx,3); hdlc_tx_frame(tx,image,260); hdlc_tx_corrupt_frame(tx);
    for (int i = 0; i < 4000; i++) t31_v34hdx_put_bit(b,hdlc_tx_get_bit(tx));
    v34_check(captures[1].len == 0, "DC3 pauses frame delivery without discarding it");
    t31_at_rx(b,resume,2);
    v34_check(v34_frame_matches(&captures[1],image,260,1), "DC1 releases the retained bad-FCS frame with DLE ferr");
    captures[0].len = 0;
    const char eot[] = {0x10,0x04};
    t31_at_rx(a,eot,2);
    int marks = 0;
    for (int i = 0; i < 128; i++)
        if (t31_v34hdx_get_bit(a) == 1) marks++;
    v34_check(marks >= 40 && t31_v34hdx_get_mode(a) != V34_HALF_DUPLEX_SILENCE,
              "EOT sends termination marks before releasing the carrier");
    t31_v34hdx_advance(a,320);
    v34_check(t31_v34hdx_get_mode(a) == V34_HALF_DUPLEX_SILENCE
              && memmem(captures[0].bytes,captures[0].len,"OK",2),
              "EOT returns to off-hook command mode after peer flags cease");
    hdlc_tx_free(tx); t31_free(a); t31_free(b);
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "--v34")) { test_v34_class1(); return failures ? 1 : 0; }
    const char *link = test_tmp("fax_class_test_pty");
    char resp[4096];
    double rms;

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

    printf("T.31 capability reporting:\n");
    expect("ATE0",         "OK",        300);
    expect("AT+GCAP",      "+GCAP: +FCLASS", 300);
    expect("AT+FCLASS=?",  "0,1,1.0,2.0,2.1", 300);
    expect("AT+FCLASS=2.1", "OK", 300);
    expect("AT+FCLASS?", "2.1", 300);
    expect("AT+FCC?", "1,B,0,2,3,1,0,7,0", 300);
    expect("AT+FCC=?", "(0-B)", 300);
    expect("AT+FCLASS=0", "OK", 300);

    /*
     * Class 2.0 (T.32) is a different module behind the same PTY, so check it
     * is reached through it: a +F command that only fax_class2.c answers, a
     * non-fax command that must still reach the T.31 interpreter, and the way
     * back out to class 0.
     */
    printf("class 2.0 selection (T.32):\n");
    expect("AT+FCLASS=2.0", "OK",        300);
    expect("AT+FCLASS?",   "2.0",        300);
    expect("AT+FCC?",      "1,5,0,2,3,2,0,7,0", 300);
    expect("AT+FLI=\"4412345\"", "OK",   300);
    expect("AT+FLI?",      "4412345",    300);
    expect("ATE0",         "OK",         300);  /* passed on to T.31 */
    expect("AT+FCLASS=0",  "OK",         300);
    expect("AT+FCLASS?",   "0",          300);

    expect("AT+FCLASS=1",  "OK",        300);
    expect("AT+FCLASS?",   "1",         300);
    expect("AT+FTM=?",     "24,48,72",  300);
    expect("AT+FRH=?",     "3",         300);

    if (!di_fax_active()) {
        printf("  FAIL di_fax_active() false after AT+FCLASS=1\n");
        failures++;
    } else {
        printf("  ok   di_fax_active() after AT+FCLASS=1\n");
    }

    printf("class 1 action commands, on hook (T.31 8.3: must be ERROR):\n");
    expect("AT+FTM=96",    "ERROR",     300);

    /* Take the call off hook the way the engine does. */
    send_cmd("ATD5551234");
    drain(resp, sizeof(resp), 300);
    if (!dial_seen) {
        printf("  FAIL ATD did not reach the dial callback\n");
        failures++;
    }
    di_on_connected(0);
    drain(resp, sizeof(resp), 300);

    pthread_t pump_tid;
    if (pthread_create(&pump_tid, NULL, pump_thread, NULL) != 0) {
        perror("pthread_create");
        di_close();
        return 1;
    }

    printf("class 1 action commands, off hook:\n");
    expect("AT+FTS=8",     "OK",        1500);
    expect("AT+FRS=1",     "OK",        1500);

    /* +FTM starts a fax transmit carrier: CONNECT, then the DTE would send
     * image data terminated by DLE ETX.  Silence here would mean the command
     * was accepted and nothing was driving the modulator. */
    send_cmd("AT+FTM=96");
    drain(resp, sizeof(resp), 300);
    if (!strstr(resp, "CONNECT")) {
        printf("  FAIL AT+FTM=96 -> wanted CONNECT, got \"%s\"\n", resp);
        failures++;
    } else {
        printf("  ok   AT+FTM=96      -> CONNECT\n");
    }

    rms = tx_rms();
    if (rms > 1000.0) {
        printf("  ok   V.29 9600 carrier present (tx RMS %.0f)\n", rms);
    } else {
        printf("  FAIL +FTM=96 put no carrier on the wire (tx RMS %.0f)\n", rms);
        failures++;
    }

    /* DLE ETX ends the transmission (T.31 8.3.3). */
    {
        static const char end[] = { 0x10, 0x03 };
        if (write(dte_fd, end, sizeof(end)) != (ssize_t)sizeof(end))
            perror("write");
        drain(resp, sizeof(resp), 1500);
        if (!strstr(resp, "OK")) {
            printf("  FAIL DLE ETX -> wanted OK, got \"%s\"\n", resp);
            failures++;
        } else {
            printf("  ok   DLE ETX ends +FTM  -> OK\n");
        }
    }

    printf("HDLC (V.21) transmit:\n");
    send_cmd("AT+FTH=3");
    drain(resp, sizeof(resp), 1500);
    if (!strstr(resp, "CONNECT")) {
        printf("  FAIL AT+FTH=3 -> wanted CONNECT, got \"%s\"\n", resp);
        failures++;
    } else {
        printf("  ok   AT+FTH=3       -> CONNECT\n");
    }
    rms = tx_rms();
    if (rms > 1000.0) {
        printf("  ok   V.21 flags present (tx RMS %.0f)\n", rms);
    } else {
        printf("  FAIL +FTH=3 put no carrier on the wire (tx RMS %.0f)\n", rms);
        failures++;
    }

    pump_running = 0;
    pthread_join(pump_tid, NULL);

    close(dte_fd);
    di_close();

    test_v34_class1();
    printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
