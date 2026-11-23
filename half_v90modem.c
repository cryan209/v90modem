/*
 * half_v90modem.c -- the modem engine as a D-Modem interop peer.
 *
 * D-Modem (github.com/D-Modem/D-Modem) tests its modem against other modems
 * offline: each peer is a program that reads its line's audio from fd 3 and
 * writes its own to fd 4, 16-bit native-endian samples at the line rate, and
 * prints "created <name>: <pty>" with its DTE on that pty
 * (dsp/test/modem_half.hpp, interop_call.h).  Their netsim sits between two
 * such programs as the G.711 network, and hands an 8000 Hz end the decoded
 * codewords as they are (no codec filter, no resampling).  This program is
 * that end for our engine: the digital modem, as on a SIP call, with the
 * analogue peers (SmartLink, Agere, SM56, PCTel, HSF, Broadcom, ESS, Intel,
 * Courier, D-Modem's own) on 9600 Hz lines:
 *
 *   NET=9600:8000:ulaw interop_call <peer half> half_v90modem "ATX3"
 *
 * The samples are G.711 levels, so codeword <-> linear is exact both ways
 * (D-Modem's g711.h and SpanDSP use the same +/-32124 u-law and +/-32256
 * A-law scales), and the engine sees exactly the octets an RTP payload would
 * have carried -- CLAUDE.md's "never transcode G.711" holds.  The one thing
 * a linear sample cannot carry is u-law's sign of zero: our -0 (0x7f)
 * reaches netsim as +0 (0xff).  Both are 0 V at the far end's D/A.
 *
 * Lockstep: for every n samples read, n are written, generated after
 * reading them; the peer owns the clock, as with audio_sock_modem.  Between
 * calls the line carries silence.
 *
 * The call model is D-Modem's, not SIP's: the line is always there, and the
 * DTE's ATD or ATA starts the call (what a SIP connect does for
 * sip_v90_modem: me_on_sip_connected()).  The call ends when the engine
 * hangs up (ATH, NO CARRIER, a failed training); fd 3 at EOF ends the
 * program.
 *
 * Settings, from the environment:
 *   V90MODEM_LAW=ulaw|alaw   the network's law (default: NET's third field,
 *                            else u-law)
 *   V90MODEM_PTY=<path>      the DTE's symlink (default $TMPDIR/half_v90modem.<pid>)
 *   V90MODEM_VERBOSE=1       the engine's verbose log
 * and every ME_*, V34_*, V90_* knob as sip_v90_modem reads them.
 * interop_call's --debug=<n> is accepted and ignored.
 */
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <spandsp.h>

#include "modem_engine.h"
#include "data_interface.h"

#define LINE_IN  3
#define LINE_OUT 4
/* Largest block handed to the engine at once: 20 ms, one RTP packet's. */
#define CHUNK 160

static volatile sig_atomic_t g_answer = 0;
static volatile sig_atomic_t g_running = 1;
static bool g_alaw = false;

static void on_signal(int sig)
{
    (void) sig;
    g_running = 0;
}

/* ATD: the engine records the number and waits in ME_DIALING; the main loop
   puts the call through (the network is netsim's, not ours to dial). */
static void on_dial(const char *number, void *user_data)
{
    (void) user_data;
    me_dial(number);
}

/* ATA (or S0's rings): the main loop answers.  This runs on the PTY reader's
   thread, the engine's line on the main one. */
static void on_answer(void *user_data)
{
    (void) user_data;
    g_answer = 1;
}

static void on_hangup(void *user_data)
{
    (void) user_data;
    me_hangup();
}

/* What sip_modem.c's main loop does with the DTE: online payload the PTY
   reader has accepted goes into the engine, no more than it can take. */
static void pump_dte(void)
{
    for (;;) {
        uint8_t buf[256];
        int room = me_put_space();
        int got;

        if (room <= 0)
            break;
        got = di_read_data(buf, room < (int) sizeof(buf) ? room : (int) sizeof(buf));
        if (got <= 0)
            break;
        if (me_put_data(buf, got) != got)
            break;
    }
}

static int read_full(int fd, void *buf, size_t n)
{
    size_t got = 0;

    while (got < n) {
        ssize_t r = read(fd, (uint8_t *) buf + got, n - got);

        if (r < 0 && errno == EINTR)
            continue;
        if (r <= 0)
            return -1;
        got += (size_t) r;
    }
    return 0;
}

static int write_full(int fd, const void *buf, size_t n)
{
    size_t put = 0;

    while (put < n) {
        ssize_t r = write(fd, (const uint8_t *) buf + put, n - put);

        if (r < 0 && errno == EINTR)
            continue;
        if (r <= 0)
            return -1;
        put += (size_t) r;
    }
    return 0;
}

/* Up to CHUNK samples: whatever the peer has sent, at least one (blocking),
   and whole samples only.  The count, or -1 at EOF. */
static int read_block(int16_t *in)
{
    ssize_t r;

    do
        r = read(LINE_IN, in, CHUNK*sizeof(int16_t));
    while (r < 0 && errno == EINTR);
    if (r <= 0)
        return -1;
    if (r & 1) {
        /* Half a sample: wait for its other octet. */
        if (read_full(LINE_IN, (uint8_t *) in + r, 1) < 0)
            return -1;
        r++;
    }
    return (int) (r/2);
}

/* V90MODEM_LAW=ulaw|alaw: the line's law, else NET's third field (ulaw). */
static const char *law_setting(void)
{
    const char *law = getenv("V90MODEM_LAW");
    const char *net = getenv("NET");

    if (law && law[0])
        return law;
    /* NET=<rate A>:<rate B>:<ulaw|alaw> */
    if (net && strstr(net, "alaw"))
        return "alaw";
    return "ulaw";
}

int main(int argc, char *argv[])
{
    char pty_link[PATH_MAX];
    char pty_name[PATH_MAX];
    const char *e;
    bool in_call = false;
    unsigned long long samples = 0;

    for (int i = 1;  i < argc;  i++) {
        if (strncmp(argv[i], "--debug", 7) != 0) {
            fprintf(stderr, "usage: %s [--debug=<n>]  (line on fds 3 and 4)\n", argv[0]);
            return 2;
        }
    }

    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    g_alaw = strcmp(law_setting(), "alaw") == 0;
    /* V90MODEM_PTY=<path>: where the DTE's pty link goes (default under TMPDIR). */
    if ((e = getenv("V90MODEM_PTY")) && e[0]) {
        snprintf(pty_link, sizeof(pty_link), "%s", e);
    } else {
        const char *tmp = getenv("TMPDIR");

        snprintf(pty_link, sizeof(pty_link), "%s/half_v90modem.%d",
                 tmp && tmp[0] ? tmp : "/tmp", (int) getpid());
    }

    /* V90MODEM_VERBOSE=1: the engine's [ME] log on stderr. */
    me_set_verbose((e = getenv("V90MODEM_VERBOSE")) && atoi(e));
    me_init();
    di_set_callbacks(on_dial, on_answer, on_hangup, NULL);
    if (di_open(pty_link) < 0) {
        fprintf(stderr, "v90modem: cannot open the DTE PTY at %s\n", pty_link);
        return 1;
    }
    me_set_law(g_alaw ? ME_LAW_ALAW : ME_LAW_ULAW);
    if (!realpath(pty_link, pty_name))
        snprintf(pty_name, sizeof(pty_name), "%s", pty_link);

    /* interop_call waits for this line on our stdout. */
    printf("created v90modem: %s\n", pty_name);
    fflush(stdout);
    fprintf(stderr, "v90modem: line on fds %d/%d, %s, DTE %s (link %s)\n",
            LINE_IN, LINE_OUT, g_alaw ? "A-law" : "u-law", pty_name, pty_link);

    while (g_running) {
        int16_t in[CHUNK];
        int16_t out[CHUNK];
        uint8_t rx[CHUNK];
        uint8_t tx[CHUNK];
        int n;

        /* A call starts at the line's sample, from the DTE: its ATD, or
           its ATA.  Before the samples, so the first block is the call's. */
        if (!in_call && me_get_state() == ME_DIALING) {
            fprintf(stderr, "v90modem: ATD at sample %llu: calling\n", samples);
            me_on_sip_connected();
            in_call = true;
        } else if (!in_call && g_answer) {
            g_answer = 0;
            fprintf(stderr, "v90modem: ATA at sample %llu: answering\n", samples);
            me_on_sip_connected();
            in_call = true;
        }

        n = read_block(in);
        if (n < 0)
            break;
        samples += (unsigned long long) n;

        if (in_call) {
            for (int i = 0;  i < n;  i++)
                rx[i] = g_alaw ? linear_to_alaw(in[i]) : linear_to_ulaw(in[i]);
            me_rx_g711(rx, n);
            (void) me_tx_g711(tx, n);
            for (int i = 0;  i < n;  i++)
                out[i] = g_alaw ? alaw_to_linear(tx[i]) : ulaw_to_linear(tx[i]);
        } else {
            memset(out, 0, (size_t) n*sizeof(out[0]));
        }
        if (write_full(LINE_OUT, out, (size_t) n*sizeof(out[0])) < 0)
            break;
        pump_dte();

        if (in_call && me_get_state() == ME_HANGUP) {
            fprintf(stderr, "v90modem: call ended at sample %llu\n", samples);
            me_flush_g711_taps();
            me_flush_io_schedule();
            me_on_sip_disconnected();
            in_call = false;
            g_answer = 0;
        }
    }

    if (in_call) {
        me_flush_g711_taps();
        me_flush_io_schedule();
        me_on_sip_disconnected();
    }
    di_close();
    me_destroy();
    return 0;
}
