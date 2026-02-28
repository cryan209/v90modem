/*
 * data_interface.c — PTY virtual serial port and AT command interpreter
 *
 * Creates a master/slave PTY pair. Applications (terminal emulators, legacy
 * software) open the slave end as a serial modem device. This module reads
 * from the master end, feeds characters to SpanDSP's AT interpreter in
 * command mode, or passes bytes to/from the modem engine in data mode.
 *
 * SpanDSP's at_state_t handles the full Hayes AT command set:
 *   ATZ, ATE, ATH, ATD, ATA, ATQ, ATV, ATS, AT+FCLASS, AT&F, +++, etc.
 */

#include "data_interface.h"
#include "modem_engine.h"

#include <spandsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <util.h>   /* openpty() on macOS */

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

static int          master_fd  = -1;
static char         slave_name[256];
static char         symlink_path[256];
static at_state_t  *at         = NULL;
static int          di_mode    = 0; /* 0=command, 1=data */
static volatile int running    = 0;
static pthread_t    reader_tid;
static ring_t       upstream_ring;

/* Callbacks registered by the modem engine */
static di_dial_cb_t   dial_cb   = NULL;
static di_answer_cb_t answer_cb = NULL;
static di_hangup_cb_t hangup_cb = NULL;
static void          *cb_user_data = NULL;

/* ------------------------------------------------------------------ */
/* SpanDSP AT callbacks                                               */
/* ------------------------------------------------------------------ */

/* Called by SpanDSP to write response text back to the terminal */
static int at_tx_handler(at_state_t *s, void *user_data,
                         const uint8_t *buf, size_t len)
{
    (void)s; (void)user_data;
    if (master_fd >= 0)
        write(master_fd, buf, len);
    return 0;
}

/*
 * Called by SpanDSP when an AT command requires modem action.
 * op  — one of the AT_MODEM_CONTROL_* enum values
 * num — dial string (for CALL), or NULL
 */
static int at_modem_control_handler(at_state_t *s, void *user_data,
                                    int op, const char *num)
{
    (void)s; (void)user_data;

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
/* PTY reader thread                                                   */
/* ------------------------------------------------------------------ */

static void *pty_reader_thread(void *arg)
{
    (void)arg;
    uint8_t buf[256];

    while (running) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(master_fd, &fds);
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 }; /* 50 ms */

        int r = select(master_fd + 1, &fds, NULL, NULL, &tv);
        if (r <= 0)
            continue;

        int n = (int)read(master_fd, buf, sizeof(buf));
        if (n <= 0)
            break;

        if (di_mode == 0) {
            /* Command mode: feed characters to AT interpreter */
            at_interpreter(at, (const char *)buf, n);
        } else {
            /* Data mode: push bytes to modem engine upstream buffer */
            ring_write(&upstream_ring, buf, n);
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int di_open(const char *link_path)
{
    int slave_fd;

    /* Create the PTY pair */
    if (openpty(&master_fd, &slave_fd, slave_name, NULL, NULL) < 0) {
        perror("openpty");
        return -1;
    }

    /* Close the slave — applications open it by name */
    close(slave_fd);

    /* Make the master non-blocking so the reader thread doesn't hang */
    fcntl(master_fd, F_SETFL, O_NONBLOCK);

    /* Create convenience symlink (ignore error if it already exists) */
    snprintf(symlink_path, sizeof(symlink_path), "%s", link_path);
    unlink(symlink_path);
    if (symlink(slave_name, symlink_path) < 0)
        fprintf(stderr, "di_open: symlink %s -> %s: %s\n",
                symlink_path, slave_name, strerror(errno));

    fprintf(stderr, "[DI] PTY slave: %s (symlink: %s)\n",
            slave_name, symlink_path);

    /* Initialise SpanDSP AT interpreter */
    ring_init(&upstream_ring);
    at = at_init(NULL, at_tx_handler, NULL,
                 at_modem_control_handler, NULL);
    if (!at) {
        fprintf(stderr, "di_open: at_init failed\n");
        close(master_fd);
        master_fd = -1;
        return -1;
    }
    at_set_at_rx_mode(at, AT_MODE_ONHOOK_COMMAND);

    /* Start reader thread */
    running = 1;
    if (pthread_create(&reader_tid, NULL, pty_reader_thread, NULL) != 0) {
        perror("pthread_create");
        at_free(at);
        close(master_fd);
        return -1;
    }

    return 0;
}

void di_close(void)
{
    running = 0;
    pthread_join(reader_tid, NULL);

    if (at) { at_free(at); at = NULL; }
    if (master_fd >= 0) { close(master_fd); master_fd = -1; }
    unlink(symlink_path);
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
    /* Switch to data mode and send CONNECT result code */
    di_mode = 1;
    at_set_at_rx_mode(at, AT_MODE_CONNECTED);

    char msg[64];
    snprintf(msg, sizeof(msg), "\r\nCONNECT %d\r\n", rate);
    if (master_fd >= 0)
        write(master_fd, msg, strlen(msg));
}

void di_on_disconnected(void)
{
    /* Switch back to command mode */
    di_mode = 0;
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
    if (di_mode == 0) return 0;
    return ring_read(&upstream_ring, buf, max_len);
}

int di_write_data(const uint8_t *buf, int len)
{
    if (master_fd < 0 || di_mode == 0) return 0;
    return (int)write(master_fd, buf, (size_t)len);
}
