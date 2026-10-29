/*
 * line_supervisor.c -- run N modem lines as N worker processes.
 *
 * See line_supervisor.h and docs/multi_line.md.  Worker i is this binary
 * with the supervisor's arguments minus --lines/--recycle, plus:
 *
 *   --line-index i              answer stagger and log tag
 *   --local-port  base + i      SIP (base from --local-port, default 5060)
 *   --rtp-port    rtp + 100*i   RTP (rtp from --rtp-port, default 4000)
 *   --pty-link    link.i        or --control-link ctl.i --data-link data.i
 *   --profile     file.i        when a profile file was given
 *   --username/--password       the i-th of a comma-separated list
 *   --exit-after-call           with --recycle
 *
 * The parser takes the last value of a repeated option, so the overrides
 * are simply appended.  Diagnostics that name one file or directory per
 * process (ME_IO_SCHEDULE, ME_DUMP_DIR, ...) would have every worker write
 * the same one, so in a worker's environment "%L" becomes its line number,
 * and an ME_DUMP_DIR without "%L" gets a "line<i>" subdirectory.  Each worker's stdout and stderr come back through a
 * pipe and are copied to ours a line at a time, prefixed "[Li] ".  A worker
 * that exits is started again: at once after a recycled call, after a second
 * when it failed, after ten seconds when it keeps failing within seconds of
 * starting.
 */
#include "line_supervisor.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

typedef struct {
    pid_t pid;
    int fd;                 /* read end of the worker's output pipe */
    char buf[4096];         /* partial output line */
    size_t len;
    double started;         /* monotonic seconds */
    double restart_at;      /* 0: running or never to restart */
    int quick_failures;     /* consecutive exits within 5 s of starting */
    unsigned starts;
} worker_t;

static volatile sig_atomic_t g_stop;

static void on_stop(int sig)
{
    (void) sig;
    g_stop = 1;
}

static double now_s(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

int line_supervisor_parse(int argc, char **argv, bool *recycle)
{
    int lines = 0;

    *recycle = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--recycle")) {
            *recycle = true;
        } else if (!strcmp(argv[i], "--lines")) {
            char *end;
            long n;

            if (i + 1 >= argc)
                return -1;
            n = strtol(argv[++i], &end, 10);
            if (*end || n < 1 || n > LINE_SUPERVISOR_MAX_LINES)
                return -1;
            lines = (int) n;
        }
    }
    return lines;
}

/* The value of the last occurrence of opt, or NULL. */
static const char *opt_value(int argc, char **argv, const char *opt)
{
    const char *v = NULL;

    for (int i = 1; i + 1 < argc; i++)
        if (!strcmp(argv[i], opt))
            v = argv[i + 1];
    return v;
}

/* Validate the whole allocation before spawning any line. Otherwise an
 * overlapping/overflowed port silently leaves part of the pool restarting. */
static int valid_port_layout(int argc, char **argv, int lines)
{
    long sip = 5060, rtp = 4000;

    for (int i = 1; i < argc; i++) {
        bool is_sip = !strcmp(argv[i], "--local-port");
        bool is_rtp = !strcmp(argv[i], "--rtp-port");
        char *end;
        long value;

        if (!is_sip && !is_rtp)
            continue;
        if (++i == argc)
            return 0;
        errno = 0;
        value = strtol(argv[i], &end, 10);
        if (errno || end == argv[i] || *end || value < 0 || value > 65535)
            return 0;
        if (is_sip)
            sip = value;
        else
            rtp = value ? value : 4000;
    }
    if (sip < 1 || sip + lines - 1 > 65535 || rtp + 100L*(lines - 1) + 1 > 65535)
        return 0;
    for (int i = 0; i < lines; i++) {
        long port = rtp + 100L*i;

        if (port <= sip + lines - 1 && port + 1 >= sip)
            return 0;
    }
    return 1;
}

/* The k-th comma-separated field of list (the last one when there are
 * fewer), into out.  Returns the number of fields. */
static int list_field(const char *list, int k, char *out, size_t len)
{
    int n = 1;
    const char *p = list;
    const char *start = list;

    for (const char *c = list; *c; c++)
        if (*c == ',')
            n++;
    for (int f = 0; f < k && f < n - 1; f++) {
        p = strchr(p, ',') + 1;
        start = p;
    }
    {
        const char *end = strchr(start, ',');
        size_t m = end ? (size_t) (end - start) : strlen(start);

        if (m >= len)
            m = len - 1;
        memcpy(out, start, m);
        out[m] = '\0';
    }
    return n;
}

typedef struct {
    char *argv[256];
    int argc;
    char store[16][512];
    int nstore;
} worker_args_t;

static void wa_push(worker_args_t *w, char *s)
{
    if (w->argc < 255)
        w->argv[w->argc++] = s;
    w->argv[w->argc] = NULL;
}

static void wa_pushf(worker_args_t *w, const char *opt, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void wa_pushf(worker_args_t *w, const char *opt, const char *fmt, ...)
{
    va_list ap;
    char *slot;

    if (w->nstore >= 16)
        return;
    slot = w->store[w->nstore++];
    va_start(ap, fmt);
    vsnprintf(slot, sizeof(w->store[0]), fmt, ap);
    va_end(ap);
    wa_push(w, (char *) opt);
    wa_push(w, slot);
}

static void build_worker_args(int argc, char **argv, int line, bool recycle,
                              worker_args_t *w)
{
    const char *v;
    int local_port = 5060, rtp_port = 4000;
    char field[256];

    memset(w, 0, sizeof(*w));
    wa_push(w, argv[0]);
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--recycle"))
            continue;
        if (!strcmp(argv[i], "--lines")) {
            i++;
            continue;
        }
        wa_push(w, argv[i]);
    }
    if ((v = opt_value(argc, argv, "--local-port")))
        local_port = atoi(v);
    if ((v = opt_value(argc, argv, "--rtp-port")) && atoi(v) > 0)
        rtp_port = atoi(v);
    wa_pushf(w, "--line-index", "%d", line);
    wa_pushf(w, "--local-port", "%d", local_port + line);
    wa_pushf(w, "--rtp-port", "%d", rtp_port + 100 * line);
    if (opt_value(argc, argv, "--control-link")) {
        wa_pushf(w, "--control-link", "%s.%d", opt_value(argc, argv, "--control-link"), line);
        wa_pushf(w, "--data-link", "%s.%d", opt_value(argc, argv, "--data-link"), line);
    } else {
        v = opt_value(argc, argv, "--pty-link");
        wa_pushf(w, "--pty-link", "%s.%d", v ? v : "/tmp/modem0", line);
    }
    if ((v = opt_value(argc, argv, "--profile")))
        wa_pushf(w, "--profile", "%s.%d", v, line);
    if ((v = opt_value(argc, argv, "--username"))) {
        list_field(v, line, field, sizeof(field));
        wa_pushf(w, "--username", "%s", field);
    }
    if ((v = opt_value(argc, argv, "--password"))) {
        list_field(v, line, field, sizeof(field));
        wa_pushf(w, "--password", "%s", field);
    }
    if (recycle)
        wa_push(w, "--exit-after-call");
}

extern char **environ;

/* Only known path-valued diagnostics get automatic suffixes. A boolean
 * *_DUMP setting must not accidentally become a filename. Unknown paths
 * can still use the explicit %L convention. */
static int diagnostic_path_kind(const char *name)
{
    static const char *const files[] = {
        "ME_IO_SCHEDULE", "ME_G711_CAPTURE", "DS_RX_BIT_DUMP", "DS_TX_BIT_DUMP",
        "DS_TX_FRAME_DUMP", "ME_V90_UPSTREAM_SYM_DUMP", "ME_RX_BIT_DUMP",
        "V90_CP_ACCEPT_DUMP", "V90_CP_BIT_DUMP", "V90_RENEG_BIT_DUMP",
        "V90_JA_BIT_DUMP", "V90A_RX_CODEWORD_DUMP", "V90A_T2_DUMP", "V90A_SYM_DUMP"
    };

    if (!strcmp(name, "ME_DUMP_DIR") || !strcmp(name, "VPCM_G711_TAP_DIR"))
        return 1;
    for (size_t i = 0; i < sizeof(files)/sizeof(files[0]); i++)
        if (!strcmp(name, files[i]))
            return 2;
    return 0;
}

static int make_directory(const char *path)
{
    struct stat st;

    if (mkdir(path, 0777) == 0)
        return 0;
    if (errno == EEXIST && stat(path, &st) == 0 && S_ISDIR(st.st_mode))
        return 0;
    return -1;
}

static int make_directories(char *path)
{
    for (char *p = path + 1; *p; p++) {
        if (*p != '/')
            continue;
        *p = '\0';
        if (make_directory(path) != 0) {
            *p = '/';
            return -1;
        }
        *p = '/';
    }
    return make_directory(path);
}

/* In the child, before starting threads: isolate diagnostics without
 * changing the parent's environment. Preserve the ORIGINAL template flag:
 * testing for %L after expansion used to append an extra /line<i>. */
static int per_line_environment(int line)
{
    char name[256], number[32];
    size_t n = 0;
    char **copy;
    int result = -1;

    snprintf(number, sizeof(number), "%d", line);
    while (environ[n])
        n++;
    copy = calloc(n + 1, sizeof(char *));
    if (!copy)
        return -1;
    for (size_t i = 0; i < n; i++) {
        if (!(copy[i] = strdup(environ[i])))
            goto done;
    }
    for (size_t i = 0; i < n; i++) {
        const char *eq = strchr(copy[i], '=');
        const char *src;
        char *value;
        size_t o = 0, capacity;
        bool templated;
        int kind;

        if (!eq || (size_t)(eq - copy[i]) >= sizeof(name) || !eq[1])
            continue;
        memcpy(name, copy[i], (size_t)(eq - copy[i]));
        name[eq - copy[i]] = '\0';
        templated = strstr(eq + 1, "%L") != NULL;
        kind = diagnostic_path_kind(name);
        if (!templated && !kind)
            continue;
        capacity = strlen(eq + 1) + 32;
        for (src = eq + 1; (src = strstr(src, "%L")); src += 2)
            capacity += strlen(number);
        value = malloc(capacity);
        if (!value)
            goto done;
        for (src = eq + 1; *src; src++) {
            if (src[0] == '%' && src[1] == 'L') {
                memcpy(value + o, number, strlen(number));
                o += strlen(number);
                src++;
            } else {
                value[o++] = *src;
            }
        }
        value[o] = '\0';
        if (!templated && kind)
            snprintf(value + o, capacity - o, "%sline%d", kind == 1 ? "/" : ".", line);
        if ((kind == 1 && make_directories(value) != 0)
            || setenv(name, value, 1) != 0) {
            fprintf(stderr, "line supervisor: cannot isolate %s: %s\n", name, strerror(errno));
            free(value);
            goto done;
        }
        free(value);
    }
    if (!getenv("ME_DUMP_DIR") || !*getenv("ME_DUMP_DIR")) {
        const char *tmp = getenv("TMPDIR");
        char *path;

        if (!tmp || !*tmp)
            tmp = "/tmp";
        path = malloc(strlen(tmp) + 64);
        if (!path)
            goto done;
        snprintf(path, strlen(tmp) + 64, "%s/v90modem-line%d-XXXXXX", tmp, line);
        if (!mkdtemp(path) || setenv("ME_DUMP_DIR", path, 1) != 0) {
            perror("line supervisor: private dump directory");
            free(path);
            goto done;
        }
        free(path);
    }
    result = 0;
done:
    for (size_t i = 0; i < n; i++)
        free(copy[i]);
    free(copy);
    return result;
}

/* The supervisor's read ends must not leak into subsequently exec'd lines.
 * Set flags before fork; only dup2's stdout/stderr survive in the worker. */
static int make_worker_pipe(int fds[2])
{
    int flags;

    if (pipe(fds) != 0)
        return -1;
    flags = fcntl(fds[0], F_GETFL);
    if (flags < 0 || fcntl(fds[0], F_SETFL, flags | O_NONBLOCK) < 0
        || fcntl(fds[0], F_SETFD, FD_CLOEXEC) < 0
        || fcntl(fds[1], F_SETFD, FD_CLOEXEC) < 0) {
        int saved = errno;
        close(fds[0]);
        close(fds[1]);
        errno = saved;
        return -1;
    }
    return 0;
}

static int start_worker(worker_t *wk, int line, int argc, char **argv, bool recycle)
{
    int fds[2];
    pid_t pid;
    worker_args_t args;

    build_worker_args(argc, argv, line, recycle, &args);
    if (make_worker_pipe(fds) != 0)
        return -1;
    pid = fork();
    if (pid < 0) {
        close(fds[0]);
        close(fds[1]);
        return -1;
    }
    if (pid == 0) {
        dup2(fds[1], STDOUT_FILENO);
        dup2(fds[1], STDERR_FILENO);
        close(fds[0]);
        close(fds[1]);
        signal(SIGINT, SIG_DFL);
        signal(SIGTERM, SIG_DFL);
        if (per_line_environment(line) != 0)
            _exit(127);
        execv(argv[0], args.argv);
        execvp(argv[0], args.argv);
        perror("line supervisor: exec");
        _exit(127);
    }
    close(fds[1]);
    wk->pid = pid;
    wk->fd = fds[0];
    wk->len = 0;
    wk->started = now_s();
    wk->restart_at = 0;
    wk->starts++;
    printf("[supervisor] line %d: worker pid %d started (start %u)\n",
           line, (int) pid, wk->starts);
    return 0;
}

/* Copy whole lines of a worker's output to ours, prefixed. */
static void drain(worker_t *wk, int line, bool flush_partial)
{
    unsigned reads = 0;

    /* A continuously chatty worker must not starve other lines, restarts
       or signal handling. After exit, drain the finite remainder to EOF. */
    while (flush_partial || reads++ < 16) {
        ssize_t n = wk->fd >= 0
            ? read(wk->fd, wk->buf + wk->len, sizeof(wk->buf) - 1 - wk->len) : 0;

        if (n > 0)
            wk->len += (size_t) n;
        for (;;) {
            char *nl = memchr(wk->buf, '\n', wk->len);
            size_t take;

            if (!nl && wk->len < sizeof(wk->buf) - 1 && !(flush_partial && wk->len))
                break;
            take = nl ? (size_t) (nl - wk->buf) + 1 : wk->len;
            printf("[L%d] %.*s%s", line, (int) take, wk->buf,
                   nl ? "" : "\n");
            memmove(wk->buf, wk->buf + take, wk->len - take);
            wk->len -= take;
            if (!wk->len)
                break;
        }
        if (n <= 0)
            break;
    }
}

int line_supervisor_run(int argc, char **argv, int lines, bool recycle)
{
    worker_t wk[LINE_SUPERVISOR_MAX_LINES];
    struct sigaction sa;

    if (lines < 1 || lines > LINE_SUPERVISOR_MAX_LINES
        || !valid_port_layout(argc, argv, lines)) {
        fprintf(stderr, "%s: --lines requires disjoint SIP and RTP/RTCP ports within 1..65535; "
                        "choose --local-port/--rtp-port bases (e.g. --rtp-port 10000)\n", argv[0]);
        return 2;
    }
    if ((opt_value(argc, argv, "--control-link") != NULL)
        != (opt_value(argc, argv, "--data-link") != NULL)
        || (opt_value(argc, argv, "--control-link") && opt_value(argc, argv, "--pty-link"))) {
        fprintf(stderr, "%s: use --pty-link or both --control-link/--data-link with --lines\n", argv[0]);
        return 2;
    }
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--gui") || !strcmp(argv[i], "--gui-web")
            || !strcmp(argv[i], "--gui-loopback") || !strcmp(argv[i], "--line-index")
            || !strcmp(argv[i], "--exit-after-call")) {
            fprintf(stderr, "%s: %s cannot be used with --lines\n", argv[0], argv[i]);
            return 2;
        }
    setvbuf(stdout, NULL, _IOLBF, 0);
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_stop;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    memset(wk, 0, sizeof(wk));
    printf("[supervisor] %d line%s%s\n", lines, lines == 1 ? "" : "s",
           recycle ? ", each worker recycled after a call" : "");
    for (int i = 0; i < lines; i++) {
        wk[i].fd = -1;
        if (start_worker(&wk[i], i, argc, argv, recycle) != 0) {
            perror("line supervisor: start");
            wk[i].restart_at = now_s() + 1.0;
        }
    }

    while (!g_stop) {
        struct pollfd pfd[LINE_SUPERVISOR_MAX_LINES];
        int map[LINE_SUPERVISOR_MAX_LINES];
        int n = 0;
        double t;
        int status;
        pid_t pid;

        for (int i = 0; i < lines; i++)
            if (wk[i].fd >= 0) {
                pfd[n].fd = wk[i].fd;
                pfd[n].events = POLLIN;
                map[n++] = i;
            }
        poll(pfd, (nfds_t) n, 200);
        for (int k = 0; k < n; k++)
            if (pfd[k].revents)
                drain(&wk[map[k]], map[k], false);

        while ((pid = waitpid(-1, &status, WNOHANG)) > 0) {
            for (int i = 0; i < lines; i++) {
                double ran;

                if (wk[i].pid != pid)
                    continue;
                drain(&wk[i], i, true);
                close(wk[i].fd);
                wk[i].fd = -1;
                wk[i].pid = 0;
                ran = now_s() - wk[i].started;
                if (WIFEXITED(status) && WEXITSTATUS(status) == 0 && recycle) {
                    printf("[supervisor] line %d: call done after %.0f s; fresh worker\n", i, ran);
                    wk[i].quick_failures = 0;
                    wk[i].restart_at = now_s();
                } else {
                    wk[i].quick_failures = ran < 5.0 ? wk[i].quick_failures + 1 : 0;
                    if (WIFSIGNALED(status))
                        printf("[supervisor] line %d: worker killed by signal %d after %.0f s\n",
                               i, WTERMSIG(status), ran);
                    else
                        printf("[supervisor] line %d: worker exited %d after %.0f s\n",
                               i, WEXITSTATUS(status), ran);
                    wk[i].restart_at = now_s() + (wk[i].quick_failures >= 3 ? 10.0 : 1.0);
                }
            }
        }

        t = now_s();
        for (int i = 0; i < lines; i++)
            if (!wk[i].pid && wk[i].restart_at > 0 && t >= wk[i].restart_at
                && start_worker(&wk[i], i, argc, argv, recycle) != 0)
                wk[i].restart_at = t + 1.0;
    }

    printf("[supervisor] stopping %d line%s\n", lines, lines == 1 ? "" : "s");
    for (int i = 0; i < lines; i++)
        if (wk[i].pid)
            kill(wk[i].pid, SIGTERM);
    {
        double deadline = now_s() + 5.0;
        int left;

        do {
            pid_t pid;
            int status;

            left = 0;
            for (int i = 0; i < lines; i++) {
                if (!wk[i].pid)
                    continue;
                drain(&wk[i], i, false);
                pid = waitpid(wk[i].pid, &status, WNOHANG);
                if (pid == wk[i].pid) {
                    drain(&wk[i], i, true);
                    close(wk[i].fd);
                    wk[i].pid = 0;
                } else {
                    left++;
                }
            }
            if (left)
                usleep(50000);
        } while (left && now_s() < deadline);
        for (int i = 0; i < lines; i++)
            if (wk[i].pid) {
                printf("[supervisor] line %d: worker %d did not stop; killing it\n",
                       i, (int) wk[i].pid);
                kill(wk[i].pid, SIGKILL);
                waitpid(wk[i].pid, NULL, 0);
            }
    }
    return 0;
}
