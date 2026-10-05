/*
 * Engine-level V.32bis loopback: two whole modem engines, a call modem and an
 * answer modem, in two processes (the engine is a process-wide singleton),
 * clocked in lockstep over a socketpair carrying G.711 in both directions.
 *
 * Unlike v32bis_duplex_test, which drives the SpanDSP datapump directly, this
 * runs everything a live call runs: V.8 (or V.25 and Annex A/V.32bis
 * automode), the engine's dispatch to the V.32bis datapump, the data stack
 * (V.14, or V.42 LAPM when V.8 negotiates it) and the DTE PTY -- and then
 * requires CONNECT at the expected rate on both PTYs and numbered lines typed
 * at each DTE to arrive intact and in order at the other.
 *
 *   v32bis_engine_pair_test <ulaw|alaw> <case> [expected_bps]
 *
 * case:
 *   v8        Both modems in ME_MODE=v32bis: V.8 offers V.32 and V.22 only,
 *             so JM selects V.32bis (V.8 8.2.3, 8.1.2).
 *   automode  The answer modem has no V.8 (ME_V8=0): it sends V.25 ANS,
 *             hears no AA, sends USB1 for Ta and then AC (V.32bis A.2.2).  The
 *             call modem is an ordinary V.8 modem, whose V.8 fails against a
 *             peer that never answers CM, and which takes the AC as V.8
 *             8.1.1's sigA (A.2.1.1).
 *   aa        Neither modem runs V.8.  The call modem answers 1 s of plain
 *             ANS with AA (A.2.1.3) and the answer modem takes AA during its
 *             answer tone straight to AC (A.2.2).
 *
 * Every byte a DTE receives after CONNECT must belong to an intact line: junk
 * ahead of the lines (bits demodulated before the modem trained) fails the
 * run even when all 150 lines follow it.  PAIR_TAP_DIR=<dir> writes each
 * side's transmit G.711 and everything its DTE received; PAIR_DUMP_DTE prints
 * the head and tail of the latter.
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "modem_engine.h"
#include "data_interface.h"

#define FRAME       160         /* 20 ms at 8 kHz */
#define MAX_SECONDS 40
#define LINES       150         /* numbered lines each DTE sends */
#define LINE_LEN    10          /* "C0000001\r\n" */

static void dial_cb(const char *uri, void *p) { (void) uri; (void) p; }
static void ctl_cb(void *p) { (void) p; }

static int read_full(int fd, uint8_t *buf, int n)
{
    int got = 0;

    while (got < n) {
        ssize_t r = read(fd, buf + got, (size_t) (n - got));

        if (r <= 0) {
            if (r < 0 && errno == EINTR)
                continue;
            return -1;
        }
        got += (int) r;
    }
    return got;
}

static int write_full(int fd, const uint8_t *buf, int n)
{
    int put = 0;

    while (put < n) {
        ssize_t r = write(fd, buf + put, (size_t) (n - put));

        if (r <= 0) {
            if (r < 0 && errno == EINTR)
                continue;
            return -1;
        }
        put += (int) r;
    }
    return put;
}

/* One side of the call.  Returns the process exit status. */
static int run_side(bool caller, bool alaw, int sock, const char *pty_link, int expected_bps)
{
    char rxtext[65536];
    int rxlen = 0;
    int connect_rate = -1;
    int connect_at = -1;
    int sent = 0;
    int next_expected = 1;
    int intact = 0;
    int bad = 0;
    int stray = 0;
    char peer = caller ? 'A' : 'C';
    char self = caller ? 'C' : 'A';
    int dte;
    struct termios tio;
    int tick;
    int done_ticks = -1;
    me_diag_snapshot_t snap;
    FILE *tap = NULL;

    me_set_verbose(getenv("PAIR_VERBOSE") != NULL);
    me_init();
    di_set_callbacks(dial_cb, ctl_cb, ctl_cb, NULL);
    if (di_open(pty_link) < 0) {
        fprintf(stderr, "%s: cannot open PTY\n", caller ? "call" : "answer");
        return 2;
    }
    dte = open(pty_link, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (dte < 0) {
        perror("open dte");
        return 2;
    }
    tcgetattr(dte, &tio);
    cfmakeraw(&tio);
    tcsetattr(dte, TCSANOW, &tio);

    if (getenv("PAIR_TAP_DIR")) {
        char path[512];

        snprintf(path, sizeof(path), "%s/%s-tx.g711", getenv("PAIR_TAP_DIR"),
                 caller ? "call" : "answer");
        tap = fopen(path, "wb");
    }
    me_set_law(alaw ? ME_LAW_ALAW : ME_LAW_ULAW);
    if (caller)
        me_dial("pair");
    me_on_sip_connected();

    for (tick = 0; tick < MAX_SECONDS*50; tick++) {
        uint8_t tx[FRAME];
        uint8_t rx[FRAME];
        char buf[4096];
        ssize_t n;

        (void) me_tx_g711(tx, FRAME);
        if (tap)
            fwrite(tx, 1, FRAME, tap);
        if (write_full(sock, tx, FRAME) < 0 || read_full(sock, rx, FRAME) < 0)
            break;
        me_rx_g711(rx, FRAME);

        /* What sip_modem.c's media loop does: online DTE payload the PTY
           reader has accepted goes into the engine, no more than it can
           take. */
        for (;;) {
            uint8_t dte_buf[256];
            int room = me_put_space();
            int got;

            if (room <= 0)
                break;
            got = di_read_data(dte_buf, room < (int) sizeof(dte_buf) ? room : (int) sizeof(dte_buf));
            if (got <= 0)
                break;
            if (me_put_data(dte_buf, got) != got)
                break;
        }

        /* Give the DTE reader thread a moment; the lockstep runs faster than
           real time and the PTY is asynchronous. */
        if (connect_rate > 0)
            usleep(200);

        while ((n = read(dte, buf, sizeof(buf))) > 0) {
            if (rxlen + n >= (ssize_t) sizeof(rxtext) - 1)
                n = (ssize_t) sizeof(rxtext) - 1 - rxlen;
            memcpy(rxtext + rxlen, buf, (size_t) n);
            rxlen += (int) n;
            rxtext[rxlen] = '\0';
        }
        if (connect_rate < 0) {
            char *c = strstr(rxtext, "CONNECT ");

            if (c && strstr(c, "\r\n")) {
                connect_rate = atoi(c + 8);
                connect_at = (int) (strstr(c, "\r\n") + 2 - rxtext);
                fprintf(stderr, "%s: CONNECT %d at %.2f s\n",
                        caller ? "call  " : "answer", connect_rate, tick*0.02);
            }
        } else if (sent < LINES && (tick & 1) == 0) {
            /* One line every 40 ms: 2000 bit/s of DTE traffic. */
            char line[LINE_LEN + 1];

            snprintf(line, sizeof(line), "%c%07d\r\n", self, sent + 1);
            if (write_full(dte, (const uint8_t *) line, LINE_LEN) == LINE_LEN)
                sent++;
        }
        if (connect_rate > 0) {
            /* Count the peer's lines that have arrived complete and in order.
               The DTE stream is binary -- a receiver delivering anything else
               would put NULs in it -- so this is bounded by rxlen and never
               by the string functions, which would stop at the first NUL and
               count nothing after it. */
            const char *p = rxtext + connect_at;
            const char *end = rxtext + rxlen;

            next_expected = 1;
            intact = 0;
            bad = 0;
            while ((p = memchr(p, peer, (size_t) (end - p))) != NULL) {
                int num;

                if (end - p < LINE_LEN)
                    break;
                if (sscanf(p + 1, "%7d", &num) == 1 && p[8] == '\r' && p[9] == '\n') {
                    if (num == next_expected) {
                        intact++;
                        next_expected++;
                    } else {
                        bad++;
                    }
                }
                p++;
            }
            /* Everything the DTE received after CONNECT has to be the peer's
               lines: nothing the modem demodulated before it trained may
               reach the DTE (V.32bis 6.1/6.2, V.22bis 6.3.1.1.2 e)). */
            stray = (rxlen - connect_at) - intact*LINE_LEN;
            if (sent == LINES && intact == LINES && done_ticks < 0)
                done_ticks = tick;
        }
        /* Keep clocking a while after both have finished, so the peer can
           finish too. */
        if (done_ticks >= 0 && tick - done_ticks > 150)
            break;
    }
    if (getenv("PAIR_TAP_DIR")) {
        char path[512];
        FILE *f;

        snprintf(path, sizeof(path), "%s/%s-dte.bin", getenv("PAIR_TAP_DIR"),
                 caller ? "call" : "answer");
        if ((f = fopen(path, "wb")) != NULL) {
            fwrite(rxtext, 1, (size_t) rxlen, f);
            fclose(f);
        }
    }
    if (getenv("PAIR_DUMP_DTE")) {
        fprintf(stderr, "%s DTE received %d bytes:\n", caller ? "call" : "answer", rxlen);
        fwrite(rxtext, 1, (size_t) (rxlen > 400 ? 400 : rxlen), stderr);
        if (rxlen > 400) {
            int from = (rxlen - 120 > 400) ? rxlen - 120 : 400;

            fprintf(stderr, "\n... last %d bytes:\n", rxlen - from);
            fwrite(rxtext + from, 1, (size_t) (rxlen - from), stderr);
        }
        fprintf(stderr, "\n");
    }
    me_get_diag_snapshot(&snap);
    fprintf(stderr, "%s: state=%s mod=%s CONNECT %d, sent %d lines, received %d of %d "
            "intact and in order (%d out of sequence, %d stray bytes)\n",
            caller ? "call  " : "answer", me_state_to_str(snap.state),
            me_modulation_to_str(snap.modulation), connect_rate, sent, intact, LINES, bad, stray);
    if (tap)
        fclose(tap);
    close(dte);
    di_close();
    shutdown(sock, SHUT_RDWR);
    close(sock);
    if (connect_rate != expected_bps || snap.modulation != ME_MOD_V32BIS
        || sent != LINES || intact != LINES || stray != 0)
        return 1;
    return 0;
}

/* PAIR_RETRAIN=call|answer has that side alone initiate a V.32bis clause 7
 * retrain PAIR_RETRAIN_MS (default 2000) ms into data mode, through the
 * engine's ME_V32BIS_RETRAIN_AFTER_MS hook; the other side must detect it
 * from the line.  The responder's circuit 104 is clamped only once the far
 * end's tone is detected (7.1/7.2), so the bits before that reach the data
 * stack: run it with ME_DATA_FRAMING=lapm, whose FCS discards them, and the
 * lines still have to arrive intact with nothing stray. */
static void set_retrain_side(bool caller)
{
    const char *side = getenv("PAIR_RETRAIN");
    const char *ms = getenv("PAIR_RETRAIN_MS");

    if (side == NULL || strcmp(side, caller ? "call" : "answer") != 0)
        return;
    setenv("ME_V32BIS_RETRAIN_AFTER_MS", ms ? ms : "2000", 1);
}

int main(int argc, char *argv[])
{
    bool alaw;
    const char *mode;
    int expected = 14400;
    int sv[2];
    pid_t child;
    int status;
    int rc;
    char call_link[128];
    char answer_link[128];

    if (argc < 3) {
        fprintf(stderr, "usage: %s <ulaw|alaw> <v8|automode|aa> [expected_bps]\n", argv[0]);
        return 2;
    }
    alaw = strcmp(argv[1], "alaw") == 0;
    mode = argv[2];
    if (argc > 3)
        expected = atoi(argv[3]);
    if (strcmp(mode, "v8") != 0 && strcmp(mode, "automode") != 0 && strcmp(mode, "aa") != 0) {
        fprintf(stderr, "unknown case '%s'\n", mode);
        return 2;
    }
    snprintf(call_link, sizeof(call_link), "/tmp/v32pair-%d-call", (int) getpid());
    snprintf(answer_link, sizeof(answer_link), "/tmp/v32pair-%d-answer", (int) getpid());
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0) {
        perror("socketpair");
        return 2;
    }
    signal(SIGPIPE, SIG_IGN);
    fflush(NULL);
    child = fork();
    if (child < 0) {
        perror("fork");
        return 2;
    }
    if (child == 0) {
        /* The answer modem. */
        close(sv[0]);
        if (strcmp(mode, "v8") == 0)
            setenv("ME_MODE", "v32bis", 1);
        else
            setenv("ME_V8", "0", 1);
        set_retrain_side(false);
        _exit(run_side(false, alaw, sv[1], answer_link, expected));
    }
    close(sv[1]);
    if (strcmp(mode, "v8") == 0)
        setenv("ME_MODE", "v32bis", 1);
    else if (strcmp(mode, "aa") == 0)
        setenv("ME_V8", "0", 1);
    set_retrain_side(true);
    rc = run_side(true, alaw, sv[0], call_link, expected);
    if (waitpid(child, &status, 0) < 0)
        return 2;
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
        rc = 1;
    printf("V.32bis engine pair %s %s: %s\n", argv[1], mode, rc == 0 ? "PASS" : "FAIL");
    return rc;
}
