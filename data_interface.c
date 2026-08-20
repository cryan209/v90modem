/*
 * data_interface.c — PTY virtual serial port(s) and AT command interpreter
 *
 * Mode A (classic): one PTY carries AT commands and data inline. In online
 * data mode a "+++" sequence with 1 s guard times (TIES) escapes back to
 * command mode; ATO resumes the data connection.
 *
 * Mode B (split): a control PTY that always speaks AT — commands work
 * mid-call and unsolicited result codes (RING, CONNECT, NO CARRIER) appear
 * here — plus a separate data PTY carrying only connection payload.
 *
 * SpanDSP's at_state_t handles the Hayes AT command set:
 *   ATZ, ATE, ATH, ATD, ATA, ATQ, ATV, ATS, ATO, AT+FCLASS, AT&F, etc.
 *
 * PTY limitation: pseudo-terminals carry no modem-control lines, so DCD/DTR
 * semantics are emulated only as result codes and hangup on close. Dial-in
 * software must treat the port as CLOCAL.
 */

#include "data_interface.h"
#include "modem_engine.h"

#include <spandsp.h>
#include <spandsp/private/logging.h>
#include <spandsp/private/at_interpreter.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <sys/ioctl.h>
#if defined(__APPLE__)
#include <util.h>   /* openpty() on macOS */
#elif defined(__linux__)
#include <pty.h>    /* openpty() on Linux */
#endif

/* ------------------------------------------------------------------ */
/* Ring buffer for upstream data (PTY → modem engine)                 */
/* ------------------------------------------------------------------ */

#define RING_SIZE 8192

typedef struct {
    uint8_t  buf[RING_SIZE];
    volatile int head;  /* write pointer */
    volatile int tail;  /* read pointer  */
    pthread_mutex_t mtx;
} ring_t;

static void ring_init(ring_t *r) {
    r->head = r->tail = 0;
    pthread_mutex_init(&r->mtx, NULL);
}

static int ring_write(ring_t *r, const uint8_t *data, int len) {
    pthread_mutex_lock(&r->mtx);
    int written = 0;
    for (int i = 0; i < len; i++) {
        int next = (r->head + 1) % RING_SIZE;
        if (next == r->tail) break; /* full */
        r->buf[r->head] = data[i];
        r->head = next;
        written++;
    }
    pthread_mutex_unlock(&r->mtx);
    return written;
}

static int ring_read(ring_t *r, uint8_t *buf, int max) {
    pthread_mutex_lock(&r->mtx);
    int n = 0;
    while (n < max && r->tail != r->head) {
        buf[n++] = r->buf[r->tail];
        r->tail = (r->tail + 1) % RING_SIZE;
    }
    pthread_mutex_unlock(&r->mtx);
    return n;
}

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    int  master_fd;
    char slave_name[256];
    char symlink_path[256];
} di_pty_t;

static di_pty_t     ctrl_pty  = { .master_fd = -1 };
static di_pty_t     data_pty  = { .master_fd = -1 };
static int          split_mode = 0;

static at_state_t  *at         = NULL;
static int          di_mode    = 0; /* Mode A: 0=command, 1=online data */
static volatile int connected  = 0; /* carrier is up */
static volatile int running    = 0;
static pthread_t    reader_tid;
static ring_t       upstream_ring;

/* TIES escape state (Mode A online data mode) */
#define ESCAPE_GUARD_MS 1000
static int      esc_count = 0;
static int64_t  last_data_byte_ms = 0;

/* Callbacks registered by the modem engine */
static di_dial_cb_t   dial_cb   = NULL;
static di_answer_cb_t answer_cb = NULL;
static di_hangup_cb_t hangup_cb = NULL;
static void          *cb_user_data = NULL;

static int64_t now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t) ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static int data_port_active(void)
{
    if (split_mode)
        return connected;
    return di_mode == 1;
}

static int data_master_fd(void)
{
    return split_mode ? data_pty.master_fd : ctrl_pty.master_fd;
}

/* ------------------------------------------------------------------ */
/* SpanDSP AT callbacks                                               */
/* ------------------------------------------------------------------ */

/* Called by SpanDSP to write response text back to the terminal */
static int at_tx_handler(void *user_data,
                         const uint8_t *buf, size_t len)
{
    (void)user_data;
    if (ctrl_pty.master_fd >= 0)
        write(ctrl_pty.master_fd, buf, len);
    return 0;
}

/*
 * Called by SpanDSP when an AT command requires modem action.
 * op  — one of the AT_MODEM_CONTROL_* enum values
 * num — dial string (for CALL), or NULL
 */
static int at_modem_control_handler(void *user_data,
                                    int op, const char *num)
{
    (void)user_data;

    switch (op) {
    case AT_MODEM_CONTROL_CALL:
        if (dial_cb && num && num[0])
            dial_cb(num, cb_user_data);
        break;

    case AT_MODEM_CONTROL_ANSWER:
        if (answer_cb)
            answer_cb(cb_user_data);
        break;

    case AT_MODEM_CONTROL_HANGUP:
    case AT_MODEM_CONTROL_ONHOOK:
        if (hangup_cb)
            hangup_cb(cb_user_data);
        break;

    /* Signal line controls — not connected to real hardware */
    case AT_MODEM_CONTROL_DTR:
    case AT_MODEM_CONTROL_RTS:
    case AT_MODEM_CONTROL_CTS:
    case AT_MODEM_CONTROL_CAR:
    case AT_MODEM_CONTROL_RNG:
    case AT_MODEM_CONTROL_DSR:
        break;

    default:
        break;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Mode A escape ("+++") and command/data byte handling               */
/* ------------------------------------------------------------------ */

static void flush_pending_escape_bytes(void)
{
    static const uint8_t pluses[3] = { '+', '+', '+' };

    if (esc_count > 0) {
        ring_write(&upstream_ring, pluses, esc_count);
        esc_count = 0;
    }
}

static void perform_escape(void)
{
    esc_count = 0;
    di_mode = 0;
    at_set_at_rx_mode(at, AT_MODE_OFFHOOK_COMMAND);
    at_put_response_code(at, AT_RESPONSE_CODE_OK);
    fprintf(stderr, "[DI] +++ escape: online command mode\n");
}

/* Mode A online-data bytes: watch for the TIES escape, forward the rest. */
static void handle_online_data_bytes(const uint8_t *buf, int n)
{
    int64_t now = now_ms();

    for (int i = 0; i < n; i++) {
        uint8_t byte = buf[i];

        if (byte == '+'
            && esc_count < 3
            && (esc_count > 0 || now - last_data_byte_ms >= ESCAPE_GUARD_MS)) {
            /* Withhold candidate escape characters until resolved. */
            esc_count++;
        } else {
            flush_pending_escape_bytes();
            ring_write(&upstream_ring, &byte, 1);
        }
        last_data_byte_ms = now;
    }
}

/* Mode A command-mode bytes: feed the AT interpreter, then honour ATO by
 * following SpanDSP's own mode switch back to online data. */
static void handle_command_bytes(const uint8_t *buf, int n)
{
    at_interpreter(at, (const char *)buf, n);
    if (!split_mode && connected && di_mode == 0
        && at->at_rx_mode == AT_MODE_CONNECTED) {
        di_mode = 1;
        esc_count = 0;
        last_data_byte_ms = now_ms();
        fprintf(stderr, "[DI] ATO: returning to online data mode\n");
    }
}

/* ------------------------------------------------------------------ */
/* PTY reader thread                                                   */
/* ------------------------------------------------------------------ */

static void *pty_reader_thread(void *arg)
{
    (void)arg;
    uint8_t buf[256];

    while (running) {
        fd_set fds;
        int maxfd = ctrl_pty.master_fd;

        FD_ZERO(&fds);
        FD_SET(ctrl_pty.master_fd, &fds);
        if (split_mode && data_pty.master_fd >= 0) {
            FD_SET(data_pty.master_fd, &fds);
            if (data_pty.master_fd > maxfd)
                maxfd = data_pty.master_fd;
        }
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 }; /* 50 ms */

        int r = select(maxfd + 1, &fds, NULL, NULL, &tv);

        /* Escape timer: three withheld '+' followed by a silent guard time */
        if (!split_mode && di_mode == 1 && esc_count == 3
            && now_ms() - last_data_byte_ms >= ESCAPE_GUARD_MS) {
            perform_escape();
        }

        if (r <= 0)
            continue;

        /* With no process holding the slave, a pty master selects readable
         * for ever and read() returns 0, or -1 with EIO on macOS.  Left
         * alone this thread then spins: measured at 99.9% of a core once
         * the first DTE detached, which starved the media thread badly
         * enough to matter on the wire -- transmit jitter went from 1.4 ms
         * to 10 ms and the far end stopped being able to acquire our Phase
         * 3 signal at all.  Back off instead; a DTE opening the slave again
         * makes the master readable with real data. */
        bool idle_eof = false;

        if (FD_ISSET(ctrl_pty.master_fd, &fds)) {
            int n = (int)read(ctrl_pty.master_fd, buf, sizeof(buf));

            if (n > 0) {
                if (!split_mode && di_mode == 1)
                    handle_online_data_bytes(buf, n);
                else
                    handle_command_bytes(buf, n);
            } else if (n == 0 || (n < 0 && errno == EIO)) {
                idle_eof = true;
            }
        }

        if (split_mode && data_pty.master_fd >= 0
            && FD_ISSET(data_pty.master_fd, &fds)) {
            int n = (int)read(data_pty.master_fd, buf, sizeof(buf));

            /* Payload only flows while the carrier is up. */
            if (n > 0 && connected)
                ring_write(&upstream_ring, buf, n);
            else if (n == 0 || (n < 0 && errno == EIO))
                idle_eof = true;
        }

        if (idle_eof)
            usleep(20000);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* PTY setup helpers                                                   */
/* ------------------------------------------------------------------ */

static int di_pty_open(di_pty_t *p, const char *link_path, const char *label)
{
    int slave_fd;

    if (openpty(&p->master_fd, &slave_fd, p->slave_name, NULL, NULL) < 0) {
        perror("openpty");
        return -1;
    }

    /* Close the slave — applications open it by name */
    close(slave_fd);

    /* Make the master non-blocking so the reader thread doesn't hang */
    fcntl(p->master_fd, F_SETFL, O_NONBLOCK);

    /* Create convenience symlink (ignore error if it already exists) */
    snprintf(p->symlink_path, sizeof(p->symlink_path), "%s", link_path);
    unlink(p->symlink_path);
    if (symlink(p->slave_name, p->symlink_path) < 0)
        fprintf(stderr, "di_open: symlink %s -> %s: %s\n",
                p->symlink_path, p->slave_name, strerror(errno));

    fprintf(stderr, "[DI] %s PTY slave: %s (symlink: %s)\n",
            label, p->slave_name, p->symlink_path);
    return 0;
}

static void di_pty_close(di_pty_t *p)
{
    if (p->master_fd >= 0) {
        close(p->master_fd);
        p->master_fd = -1;
    }
    if (p->symlink_path[0]) {
        unlink(p->symlink_path);
        p->symlink_path[0] = '\0';
    }
}

static int di_start(void)
{
    ring_init(&upstream_ring);
    at = at_init(NULL, at_tx_handler, NULL,
                 at_modem_control_handler, NULL);
    if (!at) {
        fprintf(stderr, "di_open: at_init failed\n");
        return -1;
    }
    at_set_at_rx_mode(at, AT_MODE_ONHOOK_COMMAND);

    running = 1;
    if (pthread_create(&reader_tid, NULL, pty_reader_thread, NULL) != 0) {
        perror("pthread_create");
        at_free(at);
        at = NULL;
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int di_open(const char *link_path)
{
    split_mode = 0;
    if (di_pty_open(&ctrl_pty, link_path, "modem") < 0)
        return -1;
    if (di_start() < 0) {
        di_pty_close(&ctrl_pty);
        return -1;
    }
    return 0;
}

int di_open_split(const char *control_link_path, const char *data_link_path)
{
    split_mode = 1;
    if (di_pty_open(&ctrl_pty, control_link_path, "control") < 0)
        return -1;
    if (di_pty_open(&data_pty, data_link_path, "data") < 0) {
        di_pty_close(&ctrl_pty);
        return -1;
    }
    if (di_start() < 0) {
        di_pty_close(&data_pty);
        di_pty_close(&ctrl_pty);
        return -1;
    }
    return 0;
}

void di_close(void)
{
    running = 0;
    pthread_join(reader_tid, NULL);

    if (at) { at_free(at); at = NULL; }
    di_pty_close(&data_pty);
    di_pty_close(&ctrl_pty);
    split_mode = 0;
    connected = 0;
    di_mode = 0;
    esc_count = 0;
}

void di_set_callbacks(di_dial_cb_t   dial,
                      di_answer_cb_t answer,
                      di_hangup_cb_t hangup,
                      void *user_data)
{
    dial_cb      = dial;
    answer_cb    = answer;
    hangup_cb    = hangup;
    cb_user_data = user_data;
}

void di_on_connected(int rate)
{
    char msg[64];

    connected = 1;
    esc_count = 0;
    last_data_byte_ms = now_ms();

    if (split_mode) {
        /* Control port stays in command mode; data port goes live. */
        at_set_at_rx_mode(at, AT_MODE_OFFHOOK_COMMAND);
    } else {
        di_mode = 1;
        at_set_at_rx_mode(at, AT_MODE_CONNECTED);
    }

    snprintf(msg, sizeof(msg), "\r\nCONNECT %d\r\n", rate);
    if (ctrl_pty.master_fd >= 0)
        write(ctrl_pty.master_fd, msg, strlen(msg));
}

void di_on_disconnected(void)
{
    connected = 0;
    di_mode = 0;
    esc_count = 0;
    at_set_at_rx_mode(at, AT_MODE_ONHOOK_COMMAND);
    at_call_event(at, AT_CALL_EVENT_HANGUP);
    at_put_response_code(at, AT_RESPONSE_CODE_NO_CARRIER);
}

void di_on_ring(void)
{
    at_call_event(at, AT_CALL_EVENT_ALERTING);
}

int di_read_data(uint8_t *buf, int max_len)
{
    if (!data_port_active())
        return 0;
    return ring_read(&upstream_ring, buf, max_len);
}

int di_write_data(const uint8_t *buf, int len)
{
    int fd = data_master_fd();

    if (fd < 0 || !data_port_active())
        return 0;
    return (int)write(fd, buf, (size_t)len);
}
