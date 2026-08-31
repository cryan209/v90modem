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

int main(void)
{
    const char *link = "/tmp/fax_class_test_pty";
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
    expect("AT+GCAP",      "+GCAP:+FCLASS", 300);
    expect("AT+FCLASS=?",  "0,1,1.0",   300);
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

    printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
