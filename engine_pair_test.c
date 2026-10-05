/*
 * engine_pair_test.c -- two whole modem engines, a calling one and an
 * answering one, run against each other over a G.711 bearer.
 *
 * Each end is a v90_engine_peer process: the complete engine (V.8, every
 * datapump it can select, the data stack, the DTE PTY), clocked by this
 * driver one 20 ms frame at a time, each frame generated only after the far
 * end's previous frame was consumed.  Nothing is replayed.  The test is
 * then graded from the outside, the way a user would see it: both PTYs must
 * report CONNECT, the expected modulation must be the one that connected,
 * and a block of DTE text written into each PTY must come out of the other
 * one intact.
 *
 *   engine_pair_test [--alaw] [--seconds N] [--expect MOD] [--expect-connect RATE]
 *                    [--both-env K=V] [--call-env K=V] [--answer-env K=V]
 *                    [--both-at CMD] [--call-at CMD] [--answer-at CMD]
 *
 * The AT commands are sent to that side's PTY before the call starts, each
 * required to answer OK -- the way a DTE configures a modem (AT+MS).
 *
 * MOD is the engine's modulation name (V32BIS, V22BIS, V34, ...).
 */
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define FRAME 160
#define MAX_ENV 16

typedef struct {
    const char *name;
    int call;
    pid_t pid;
    int to_fd;          /* driver -> peer stdin */
    int from_fd;        /* peer stdout -> driver */
    int pty_fd;
    char pty_path[128];
    char log_path[128];
    const char *env[MAX_ENV];
    int n_env;
    const char *at[MAX_ENV];
    int n_at;
    char dte[65536];    /* everything read from the DTE side */
    size_t dte_len;
    char payload[2048];
    size_t payload_len;
    int connect_frame;  /* frame index CONNECT was seen, -1 if not */
    int sent;
} side_t;

static int read_full(int fd, void *buf, size_t len)
{
    size_t got = 0;

    while (got < len) {
        ssize_t n = read(fd, (char *) buf + got, len - got);

        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return -1;
        got += (size_t) n;
    }
    return 0;
}

static int write_full(int fd, const void *buf, size_t len)
{
    size_t done = 0;

    while (done < len) {
        ssize_t n = write(fd, (const char *) buf + done, len - done);

        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return -1;
        done += (size_t) n;
    }
    return 0;
}

static int spawn(side_t *s, int alaw)
{
    int in[2], out[2];

    if (pipe(in) || pipe(out))
        return -1;
    s->pid = fork();
    if (s->pid < 0)
        return -1;
    if (s->pid == 0) {
        int log = open(s->log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        const char *args[6];
        int n = 0;

        dup2(in[0], STDIN_FILENO);
        dup2(out[1], STDOUT_FILENO);
        if (log >= 0)
            dup2(log, STDERR_FILENO);
        close(in[1]);
        close(out[0]);
        setenv("ME_MEDIA_CLOCK", "1", 1);
        for (int i = 0; i < s->n_env; i++)
            putenv((char *) s->env[i]);
        args[n++] = "./v90_engine_peer";
        args[n++] = s->pty_path;
        if (s->call)
            args[n++] = "--call";
        if (alaw)
            args[n++] = "--alaw";
        args[n] = NULL;
        execv(args[0], (char *const *) args);
        perror("execv v90_engine_peer");
        _exit(127);
    }
    close(in[0]);
    close(out[1]);
    s->to_fd = in[1];
    s->from_fd = out[0];
    /* The other peer is forked later and must not inherit these: a stray
       copy of this one's stdin write end would keep it from ever seeing
       EOF. */
    fcntl(s->to_fd, F_SETFD, FD_CLOEXEC);
    fcntl(s->from_fd, F_SETFD, FD_CLOEXEC);
    return 0;
}

static int open_pty(side_t *s)
{
    for (int i = 0; i < 200; i++) {
        s->pty_fd = open(s->pty_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (s->pty_fd >= 0)
            return 0;
        usleep(10000);
    }
    return -1;
}

static void poll_dte(side_t *s, int frame)
{
    for (;;) {
        char buf[1024];
        ssize_t n = read(s->pty_fd, buf, sizeof(buf));

        if (n <= 0)
            break;
        if (s->dte_len + (size_t) n < sizeof(s->dte) - 1) {
            memcpy(s->dte + s->dte_len, buf, (size_t) n);
            s->dte_len += (size_t) n;
            s->dte[s->dte_len] = '\0';
        }
    }
    if (s->connect_frame < 0 && strstr(s->dte, "CONNECT"))
        s->connect_frame = frame;
}

/* Last "closed loop:" summary line the peer printed. */
static void final_line(const side_t *s, char *out, size_t max)
{
    FILE *f = fopen(s->log_path, "r");
    char line[512];

    out[0] = '\0';
    if (!f)
        return;
    while (fgets(line, sizeof(line), f))
        if (strncmp(line, "closed loop:", 12) == 0)
            snprintf(out, max, "%s", line);
    fclose(f);
}

int main(int argc, char **argv)
{
    side_t side[2];
    int alaw = 0;
    double seconds = 40.0;
    const char *expect = NULL;
    const char *expect_connect = NULL;
    uint8_t tx[2][FRAME], rx[FRAME];
    int frames;
    int failed = 0;
    int done_frame = -1;

    memset(side, 0, sizeof(side));
    side[0].name = "call";
    side[0].call = 1;
    side[1].name = "answer";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--alaw")) {
            alaw = 1;
        } else if (!strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--expect") && i + 1 < argc) {
            expect = argv[++i];
        } else if (!strcmp(argv[i], "--expect-connect") && i + 1 < argc) {
            expect_connect = argv[++i];
        } else if ((!strcmp(argv[i], "--call-at") || !strcmp(argv[i], "--answer-at")
                    || !strcmp(argv[i], "--both-at")) && i + 1 < argc) {
            int both = argv[i][2] == 'b';
            int which = argv[i][2] == 'c' ? 0 : 1;
            const char *cmd = argv[++i];

            for (int k = 0; k < 2; k++) {
                if ((both || k == which) && side[k].n_at < MAX_ENV)
                    side[k].at[side[k].n_at++] = cmd;
            }
        } else if ((!strcmp(argv[i], "--both-env") || !strcmp(argv[i], "--call-env")
                    || !strcmp(argv[i], "--answer-env")) && i + 1 < argc) {
            int both = argv[i][2] == 'b';
            int which = argv[i][2] == 'c' ? 0 : 1;
            const char *kv = argv[++i];

            for (int k = 0; k < 2; k++) {
                if ((both || k == which) && side[k].n_env < MAX_ENV)
                    side[k].env[side[k].n_env++] = kv;
            }
        } else {
            fprintf(stderr, "usage: %s [--alaw] [--seconds N] [--expect MOD] [--expect-connect RATE] "
                    "[--both-env K=V] [--call-env K=V] [--answer-env K=V]\n"
                    "       [--both-at CMD] [--call-at CMD] [--answer-at CMD]\n", argv[0]);
            return 2;
        }
    }

    signal(SIGPIPE, SIG_IGN);
    for (int k = 0; k < 2; k++) {
        side_t *s = &side[k];

        snprintf(s->pty_path, sizeof(s->pty_path), "/tmp/engine_pair_%d_%s", (int) getpid(), s->name);
        snprintf(s->log_path, sizeof(s->log_path), "/tmp/engine_pair_%d_%s.log", (int) getpid(), s->name);
        s->payload_len = 0;
        for (int line = 0; line < 12; line++)
            s->payload_len += (size_t) snprintf(s->payload + s->payload_len,
                                                sizeof(s->payload) - s->payload_len,
                                                "%c%04d the quick brown fox jumps over the lazy dog\r\n",
                                                k ? 'A' : 'C', line);
        s->connect_frame = -1;
        if (spawn(s, alaw) != 0 || open_pty(s) != 0) {
            fprintf(stderr, "could not start the %s side\n", s->name);
            return 1;
        }
    }

    /* DTE configuration before the call: each command must answer OK. */
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < side[k].n_at; i++) {
            char line[128];
            int n = snprintf(line, sizeof(line), "%s\r", side[k].at[i]);

            side[k].dte_len = 0;
            side[k].dte[0] = '\0';
            write_full(side[k].pty_fd, line, (size_t) n);
            for (int w = 0; w < 100 && !strstr(side[k].dte, "OK") && !strstr(side[k].dte, "ERROR"); w++) {
                usleep(10000);
                poll_dte(&side[k], -1);
            }
            if (!strstr(side[k].dte, "OK")) {
                printf("  %-6s FAIL: %s did not answer OK\n", side[k].name, side[k].at[i]);
                return 1;
            }
        }
        side[k].dte_len = 0;
        side[k].dte[0] = '\0';
        side[k].connect_frame = -1;
    }

    memset(tx, alaw ? 0xD5 : 0xFF, sizeof(tx));
    frames = (int) (seconds * 8000.0 / FRAME);
    for (int f = 0; f < frames; f++) {
        uint8_t next[2][FRAME];

        for (int k = 0; k < 2; k++) {
            uint8_t header[2] = { FRAME & 0xFF, FRAME >> 8 };

            memcpy(rx, tx[1 - k], FRAME);
            if (write_full(side[k].to_fd, header, 2) || write_full(side[k].to_fd, rx, FRAME)
                || read_full(side[k].from_fd, next[k], FRAME)) {
                fprintf(stderr, "%s side stopped at frame %d\n", side[k].name, f);
                failed = 1;
                break;
            }
        }
        if (failed)
            break;
        memcpy(tx, next, sizeof(tx));

        for (int k = 0; k < 2; k++) {
            side_t *s = &side[k];

            poll_dte(s, f);
            /* Half a second after both ends report CONNECT, each DTE sends. */
            if (!s->sent && side[0].connect_frame >= 0 && side[1].connect_frame >= 0
                && f > side[0].connect_frame + 25 && f > side[1].connect_frame + 25) {
                write_full(s->pty_fd, s->payload, s->payload_len);
                s->sent = 1;
            }
        }
        if (side[0].sent && side[1].sent
            && strstr(side[0].dte, side[1].payload) && strstr(side[1].dte, side[0].payload)) {
            done_frame = f;
            break;
        }
        /* Each peer moves DTE bytes between its PTY and the engine on a
           reader thread.  Run the line at ~10x real time once data mode is
           up, so that thread is not starved by a frame loop that would
           otherwise do 40 s of audio in two. */
        if (side[0].connect_frame >= 0 && side[1].connect_frame >= 0)
            usleep(2000);
        else if ((f & 7) == 0)
            usleep(1000);
    }

    for (int k = 0; k < 2; k++) {
        close(side[k].to_fd);
        waitpid(side[k].pid, NULL, 0);
        poll_dte(&side[k], frames);
    }

    printf("engine pair (%s):", alaw ? "A-law" : "u-law");
    for (int i = 1; i < argc; i++)
        printf(" %s", argv[i]);
    printf("\n  %s\n", done_frame >= 0 ? "payload exchanged" : "payload NOT exchanged");
    for (int k = 0; k < 2; k++) {
        side_t *s = &side[k];
        char fin[512];
        const char *c = strstr(s->dte, "CONNECT");
        char conn[32] = "none";

        if (c)
            sscanf(c, "%31[^\r\n]", conn);
        final_line(s, fin, sizeof(fin));
        printf("  %-6s %s at %.2f s; %s", s->name, conn,
               s->connect_frame >= 0 ? s->connect_frame * FRAME / 8000.0 : -1.0,
               fin[0] ? fin : "no summary\n");
        if (s->connect_frame < 0)
            failed = 1;
        if (expect) {
            char want[64];

            snprintf(want, sizeof(want), "modulation=%s ", expect);
            if (!strstr(fin, want)) {
                printf("  %-6s FAIL: expected %s\n", s->name, want);
                failed = 1;
            }
        }
        if (expect_connect) {
            char want[64];

            snprintf(want, sizeof(want), "CONNECT %s\r", expect_connect);
            if (!strstr(s->dte, want)) {
                printf("  %-6s FAIL: expected CONNECT %s\n", s->name, expect_connect);
                failed = 1;
            }
        }
        if (!strstr(side[1 - k].dte, s->payload)) {
            printf("  %-6s FAIL: its DTE text did not arrive intact at the %s side\n",
                   s->name, side[1 - k].name);
            failed = 1;
        }
    }
    if (failed) {
        for (int k = 0; k < 2; k++) {
            printf("  %s DTE received %zu bytes:\n", side[k].name, side[k].dte_len);
            fwrite(side[k].dte, 1, side[k].dte_len > 600 ? 600 : side[k].dte_len, stdout);
            printf("\n");
        }
        printf("  logs: %s %s\n", side[0].log_path, side[1].log_path);
        return 1;
    }
    for (int k = 0; k < 2; k++)
        unlink(side[k].log_path);
    return 0;
}
