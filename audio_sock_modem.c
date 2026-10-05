/*
 * audio_sock_modem.c -- the modem engine on a raw audio socket instead of SIP.
 *
 * sip_v90_modem carries the call's audio as G.711 RTP.  This carries the same
 * G.711 codewords over a stream socket, so the engine can be put on a line
 * that is not a SIP call: a software peer such as slmodemd (through
 * rig/slm_bridge/slm_bridge.c), a simulated channel, a capture replayer.
 * Nothing about the modem changes -- me_rx_g711()/me_tx_g711() see exactly
 * the bytes an RTP payload would have carried, so the "never transcode G.711"
 * rule holds as it does on SIP.
 *
 *   audio_sock_modem --listen <path> [--pty-link <path>] [--alaw] [--verbose]
 *
 * The socket is a Unix stream socket the modem listens on.  Each accepted
 * connection is one call:
 *
 *   - if the DTE has dialled (ATD..., the engine is in ME_DIALING), the
 *     connection completes that outgoing call and this modem is the call
 *     modem;
 *   - otherwise it is an incoming call: RING is sent to the DTE and the call
 *     is answered at once, as sip_v90_modem's auto-answer does.
 *
 * Wire format: G.711 codewords (u-law, or A-law with --alaw) at 8000/s, in
 * both directions.  The peer owns the clock: for every n codewords the modem
 * reads it writes exactly n back, generated after consuming them -- the same
 * contract slmodemd's own socket driver uses, and the one that lets a peer
 * run the call faster or slower than real time without either end slipping.
 * A peer that wants real time paces itself.
 *
 * The call ends when the peer closes the connection (the engine is told the
 * line dropped) or when the engine hangs up (ATH, NO CARRIER, a failed
 * training), which closes the connection.
 */
#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "modem_engine.h"
#include "data_interface.h"

/* Largest chunk handed to the engine at once: 20 ms, what one RTP packet
   carries on the SIP path. */
#define CHUNK 160

static volatile sig_atomic_t g_running = 1;

static void on_signal(int sig)
{
    (void) sig;
    g_running = 0;
}

static void on_dial(const char *number, void *user_data)
{
    (void) user_data;
    /* Completed when a peer connects (see the header). */
    me_dial(number);
}

static void on_answer(void *user_data)
{
    (void) user_data;
}

static void on_hangup(void *user_data)
{
    (void) user_data;
    me_hangup();
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s --listen <socket-path> [--pty-link <path>] [--alaw] [--verbose]\n",
            argv0);
}

static int write_full(int fd, const uint8_t *buf, int n)
{
    int put = 0;

    while (put < n) {
        ssize_t r = write(fd, buf + put, (size_t) (n - put));

        if (r < 0 && errno == EINTR)
            continue;
        if (r <= 0)
            return -1;
        put += (int) r;
    }
    return put;
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

/* Run one call on an accepted connection.  Returns when it ends. */
static void run_call(int conn)
{
    bool calling = (me_get_state() == ME_DIALING);
    struct pollfd pfd;

    fprintf(stderr, "[SOCK] connection accepted: %s call\n",
            calling ? "outgoing (completes ATD)" : "incoming (auto-answered)");
    if (!calling)
        di_on_ring();
    me_on_sip_connected();

    pfd.fd = conn;
    pfd.events = POLLIN;
    while (g_running) {
        uint8_t rx[CHUNK];
        uint8_t tx[CHUNK];
        ssize_t n;
        int r;

        pump_dte();
        if (me_get_state() == ME_HANGUP) {
            fprintf(stderr, "[SOCK] engine hung up; closing the connection\n");
            break;
        }
        /* The timeout only bounds how long a hang-up from the DTE side waits
           to be noticed while the peer is quiet. */
        r = poll(&pfd, 1, 50);
        if (r < 0 && errno == EINTR)
            continue;
        if (r < 0) {
            perror("poll");
            break;
        }
        if (r == 0)
            continue;
        n = read(conn, rx, sizeof(rx));
        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0) {
            fprintf(stderr, "[SOCK] peer closed the connection\n");
            break;
        }
        me_rx_g711(rx, (int) n);
        (void) me_tx_g711(tx, (int) n);
        if (write_full(conn, tx, (int) n) < 0) {
            fprintf(stderr, "[SOCK] peer write failed; ending the call\n");
            break;
        }
    }
    me_flush_g711_taps();
    me_flush_io_schedule();
    close(conn);
    me_on_sip_disconnected();
}

int main(int argc, char *argv[])
{
    const char *listen_path = NULL;
    const char *pty_link = "/tmp/modem0";
    me_law_t law = ME_LAW_ULAW;
    int verbose = 0;
    struct sockaddr_un addr;
    int lfd;
    int i;

    for (i = 1;  i < argc;  i++) {
        if (strcmp(argv[i], "--listen") == 0 && i + 1 < argc) {
            listen_path = argv[++i];
        } else if (strcmp(argv[i], "--pty-link") == 0 && i + 1 < argc) {
            pty_link = argv[++i];
        } else if (strcmp(argv[i], "--alaw") == 0) {
            law = ME_LAW_ALAW;
        } else if (strcmp(argv[i], "--verbose") == 0) {
            verbose = 1;
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (listen_path == NULL || strlen(listen_path) >= sizeof(addr.sun_path)) {
        usage(argv[0]);
        return 2;
    }

    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    me_set_verbose(verbose);
    me_init();
    di_set_callbacks(on_dial, on_answer, on_hangup, NULL);
    if (di_open(pty_link) < 0) {
        fprintf(stderr, "cannot open the DTE PTY at %s\n", pty_link);
        return 1;
    }
    me_set_law(law);

    lfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (lfd < 0) {
        perror("socket");
        return 1;
    }
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, listen_path);
    unlink(listen_path);
    if (bind(lfd, (struct sockaddr *) &addr, sizeof(addr)) < 0 || listen(lfd, 1) < 0) {
        perror(listen_path);
        return 1;
    }
    fprintf(stderr, "[SOCK] listening on %s (%s), DTE on %s\n",
            listen_path, law == ME_LAW_ALAW ? "A-law" : "u-law", pty_link);

    while (g_running) {
        struct pollfd pfd = { .fd = lfd, .events = POLLIN };
        int r = poll(&pfd, 1, 50);

        if (r < 0 && errno != EINTR) {
            perror("poll");
            break;
        }
        if (r <= 0)
            continue;
        int conn = accept(lfd, NULL, NULL);

        if (conn < 0) {
            if (errno != EINTR)
                perror("accept");
            continue;
        }
        run_call(conn);
    }
    close(lfd);
    unlink(listen_path);
    di_close();
    me_destroy();
    return 0;
}
