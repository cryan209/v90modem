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
#include "fax_class2.h"
#include "at_ms.h"
#include "v250_ctl.h"
#include "at_test.h"
#include "at_help.h"
#include "line_monitor.h"
#include "build_version.h"

#include <spandsp.h>
#include <spandsp/private/logging.h>
#include <spandsp/private/at_interpreter.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <sys/ioctl.h>
#include <poll.h>
#if defined(__APPLE__)
#include <util.h>   /* openpty() on macOS */
#elif defined(__linux__)
#include <pty.h>    /* openpty() on Linux */
#endif
#include <termios.h>

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

static int ring_space(ring_t *r) {
    pthread_mutex_lock(&r->mtx);
    int used = (r->head - r->tail + RING_SIZE) % RING_SIZE;
    pthread_mutex_unlock(&r->mtx);
    return RING_SIZE - 1 - used;
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

/* Peek/consume keeps local-loop output queued across a short PTY write. */
static int ring_peek(ring_t *r, uint8_t *buf, int max)
{
    pthread_mutex_lock(&r->mtx);
    int pos=r->tail,n=0;
    while(n<max && pos!=r->head) {buf[n++]=r->buf[pos];pos=(pos+1)%RING_SIZE;}
    pthread_mutex_unlock(&r->mtx);
    return n;
}
static void ring_consume(ring_t *r, int n)
{
    pthread_mutex_lock(&r->mtx);
    r->tail=(r->tail+n)%RING_SIZE;
    pthread_mutex_unlock(&r->mtx);
}
static void ring_clear(ring_t *r)
{
    pthread_mutex_lock(&r->mtx);
    r->head=r->tail=0;
    pthread_mutex_unlock(&r->mtx);
}

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    int  master_fd;
    int  slave_hold_fd;     /* kept open so the raw termios survives */
    char slave_name[256];
    char symlink_path[256];
} di_pty_t;

static di_pty_t     ctrl_pty  = { .master_fd = -1, .slave_hold_fd = -1 };
static di_pty_t     data_pty  = { .master_fd = -1, .slave_hold_fd = -1 };
static int          split_mode = 0;

/*
 * The AT interpreter is SpanDSP's T.31 fax modem (T.31 8.2/8.3), whose own
 * at_state_t carries the Hayes command set.  Running it here rather than a
 * bare at_init() is what makes the +F command set work: the class 1 action
 * commands (+FTM/+FRM/+FTH/+FRH/+FTS/+FRS) are dispatched by
 * process_class1_cmd() to a class 1 handler, and T.31 supplies one along with
 * the V.21/V.27ter/V.29/V.17 fax datapumps it drives.  With no such handler
 * every one of those commands answers ERROR.
 *
 * AT+FCLASS=0 leaves the data path exactly as before -- T.31 is then only an
 * AT parser and the engine owns the audio.  A non-zero FCLASS makes the fax
 * datapumps the audio path instead; see di_fax_active().
 */
static t31_state_t *t31        = NULL;
static at_state_t  *at         = NULL;
/* T.31 owns both the DTE command parser and the fax datapumps.  The PTY
 * reader issues +F commands while the RTP media callbacks call t31_rx/tx;
 * SpanDSP's state machine is not re-entrant across that boundary. */
static pthread_mutex_t t31_mtx = PTHREAD_MUTEX_INITIALIZER;
static int          di_mode    = 0; /* Mode A: 0=command, 1=online data */
static volatile int connected  = 0; /* carrier is up */
/* The DTE ended this call itself (ATH): its teardown is answered by OK, so the
 * engine's later report that the call is gone must not add a NO CARRIER. */
static volatile int local_hangup = 0;
static volatile int running    = 0;
static pthread_t    reader_tid;
static ring_t       upstream_ring;
static ring_t       local_loop_ring;
static at_test_t    diagnostics;
/* Leaf lock: t31_mtx or g_state_mtx may be above it. Never call the engine,
 * T.31 or a blocking PTY writer while holding it. Ring locks are below it. */
static pthread_mutex_t test_mtx=PTHREAD_MUTEX_INITIALIZER;
static int test_rate=9600;
static int64_t test_last_ms;
static uint64_t test_bit_fraction;


/* AT+MS: the engine owns the modulation offer, the rates live here. */
static di_ms_set_cb_t   ms_set_cb;
static di_ms_get_cb_t   ms_get_cb;
static di_ms_reset_cb_t ms_reset_cb;
/* What the last accepted AT+MS said, so +MS? can report the carrier name and
 * rates as given (V32B, not the V.22bis offer it maps to).  Reported only
 * while the engine still holds the mode it mapped to. */
static at_ms_settings_t ms_cur;
static bool             ms_cur_valid;

/* V.250 +MR/+ES/+ER/+DS/+DR.  Written by the AT interpreter (under t31_mtx),
 * read by the engine thread when it sets up a call, so it has a leaf lock of
 * its own: nothing is called while holding it. */
static v250_ctl_t        v250;
static pthread_mutex_t   v250_mtx = PTHREAD_MUTEX_INITIALIZER;
static di_connect_info_cb_t connect_info_cb;

/* ATI6/ATI11: the current or last call, as a DTE asks about it after NO
 * CARRIER.  Under test_mtx -- a leaf the AT path already takes, and the lock
 * di_read_data()/di_write_data() hold when they count the octets. */
typedef struct {
    bool valid;             /* a call has connected or failed since power-on */
    bool active;
    bool failed;            /* it ended before CONNECT */
    bool originate;
    bool fax;
    int64_t start_ms;
    int64_t end_ms;
    int rate;               /* the CONNECT rate */
    char carrier[16];
    char ec[8];
    int tx_rate;
    int rx_rate;
    int dc_scheme;
    bool dc_tx;
    bool dc_rx;
    uint64_t to_line;       /* DTE octets handed to the engine */
    uint64_t to_dte;        /* line octets delivered to the DTE */
    char cause[64];
    char state[16];         /* live: "data", "retraining" */
    int64_t updated_ms;     /* last engine push */
    char detail[1024];      /* the engine's ATI11 text */
} di_link_t;
static di_link_t link_now;
static bool link_next_originate;
static di_link_detail_cb_t link_detail_cb;
/* at_interpreter.c's private NO_RESULT_CODES (ATQ1). */
#define DI_NO_RESULT_CODES 3

/*
 * Class 2.0 (T.32) line assembly.  T.31 does its own line buffering, but in
 * class 2.0 the +F commands are ours, so a line has to be complete before it
 * can be offered to fax_class2.c -- and what is not a +F command goes on to
 * the T.31 interpreter, which owns the S registers, echo and result codes.
 */
#define FC2_LINE_MAX 512
static char fc2_line[FC2_LINE_MAX];
static int  fc2_line_len;

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

static void diagnostic_reset(bool power_on)
{
    pthread_mutex_lock(&test_mtx);
    if(power_on)at_test_reset(&diagnostics);
    else at_test_disconnect(&diagnostics);
    ring_clear(&local_loop_ring);
    test_bit_fraction=0;
    test_last_ms=now_ms();
    pthread_mutex_unlock(&test_mtx);
}

/* Common payload seam, after escape handling in Mode A. +TLDL is V.250
 * 6.7.2.13's DTE->DTE loop; it must never send these bytes to the peer. */
static void dte_payload(const uint8_t *buf,int n)
{
    pthread_mutex_lock(&test_mtx);
    if(connected) {
        if(diagnostics.local_loop) {
            if(!diagnostics.type)ring_write(&local_loop_ring,buf,n);
        } else ring_write(&upstream_ring,buf,n);
    }
    pthread_mutex_unlock(&test_mtx);
}

static bool payload_has_room(int n)
{
    pthread_mutex_lock(&test_mtx);
    bool room=diagnostics.local_loop
        ? diagnostics.type || ring_space(&local_loop_ring)>=n
        : ring_space(&upstream_ring)>=n;
    pthread_mutex_unlock(&test_mtx);
    return room;
}

static void diagnostic_poll(void)
{
    uint8_t buf[256];
    int64_t now=now_ms();
    pthread_mutex_lock(&test_mtx);
    int64_t elapsed=now-test_last_ms;
    test_last_ms=now;
    /* This is a software interface-loop clock. No PCM stream accounting is
     * changed. Don't invent hours of checked bits after machine suspension. */
    if(elapsed>1000)elapsed=1000;
    if(elapsed>0 && diagnostics.type) {
        test_bit_fraction+=(uint64_t)elapsed*(unsigned)test_rate;
        at_test_clock_local(&diagnostics,test_bit_fraction/1000);
        test_bit_fraction%=1000;
    }
    if(diagnostics.local_loop && !diagnostics.type && data_port_active()) {
        int n=ring_peek(&local_loop_ring,buf,sizeof(buf));
        if(n>0) {
            int written=(int)write(data_master_fd(),buf,(size_t)n);
            if(written>0)ring_consume(&local_loop_ring,written);
        }
    }
    pthread_mutex_unlock(&test_mtx);
}

/* ------------------------------------------------------------------ */
/* SpanDSP AT callbacks                                               */
/* ------------------------------------------------------------------ */

/* Command-state text to the DTE.  The master is non-blocking and a pty holds
 * about 1 KB, so a plain write() of a long response kept the first kilobyte
 * and dropped the rest: AT+MS$'s help (1.6 KB) arrived cut off mid-table and
 * at_ms_test failed on it.  Wait for the DTE to read, bounded so a DTE that
 * never reads cannot stall the reader thread for long. */
static void ctrl_write(const void *buf, size_t len)
{
    const uint8_t *p = buf;
    int waited_ms = 0;

    while (len > 0 && ctrl_pty.master_fd >= 0) {
        ssize_t n = write(ctrl_pty.master_fd, p, len);

        if (n > 0) {
            p += n;
            len -= (size_t) n;
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            return;
        if (waited_ms >= 500)
            return;
        {
            struct pollfd pfd = { .fd = ctrl_pty.master_fd, .events = POLLOUT };

            poll(&pfd, 1, 20);
        }
        waited_ms += 20;
    }
}

/* Called by SpanDSP to write response text back to the terminal */
static int at_tx_handler(void *user_data,
                         const uint8_t *buf, size_t len)
{
    (void)user_data;
    ctrl_write(buf, len);
    return 0;
}

/*
 * Called by SpanDSP when an AT command requires modem action.
 * op  — one of the AT_MODEM_CONTROL_* enum values
 * num — dial string (for CALL), or NULL
 */
/* V.250 6.4.1 +MS.  args is the text after "+MS", or NULL for ATZ/AT&F.
 * Runs inside the AT interpreter, so at_put_response() lands before the
 * final OK.  Returns <0 for ERROR. */
/* What +MS? reports: the last AT+MS as given while the engine still holds
 * the mode it mapped to, otherwise the engine's mode with no rate limits. */
static void plus_ms_current(at_ms_settings_t *ms)
{
    char mode[16];
    bool automode;
    const char *carrier;
    const char *want;

    ms_get_cb(mode, sizeof(mode), &automode);
    want = ms_cur_valid ? at_ms_settings_to_mode(&ms_cur) : NULL;
    if (want && !strcmp(want, mode) && (ms_cur.automode != 0) == automode) {
        *ms = ms_cur;
        return;
    }
    memset(ms, 0, sizeof(*ms));
    carrier = at_ms_mode_to_carrier(mode);
    snprintf(ms->carrier, sizeof(ms->carrier), "%s", carrier ? carrier : "V90");
    ms->automode = automode ? 1 : 0;
}

static int handle_plus_ms(const char *args)
{
    at_ms_settings_t ms;
    char buf[4096];
    const char *want;

    if (!args) {
        /* ATZ / AT&F: the whole V.250 profile this module owns goes back to its
         * defaults, whether or not an engine has registered +MS. */
        pthread_mutex_lock(&v250_mtx);
        v250_ctl_reset(&v250);
        pthread_mutex_unlock(&v250_mtx);
        if (ms_reset_cb)
            ms_reset_cb();
        ms_cur_valid = false;
        return 0;
    }
    if (!ms_set_cb || !ms_get_cb || !ms_reset_cb)
        return -1;
    switch (at_ms_parse(args, &ms)) {
    case AT_MS_SET:
        want = at_ms_settings_to_mode(&ms);
        if (!want || ms_set_cb(want, ms.automode != 0) < 0)
            return -1;
        ms_cur = ms;
        ms_cur_valid = true;
        return 0;
    case AT_MS_READ:
        plus_ms_current(&ms);
        at_ms_format_read(&ms, buf, sizeof(buf));
        at_put_response(at, buf);
        return 0;
    case AT_MS_TEST:
        at_ms_format_test(buf, sizeof(buf));
        at_put_response(at, buf);
        return 0;
    case AT_MS_HELP:
        plus_ms_current(&ms);
        at_ms_format_help(&ms, buf, sizeof(buf));
        at_put_response(at, buf);
        return 0;
    default:
        return -1;
    }
}

/* V.250 6.4.3, 6.5.1, 6.5.5, 6.6.1, 6.6.3: see v250_ctl.c. */
static int handle_v250_parameter(const char *text)
{
    char info[160];
    v250_ctl_result_t r;

    pthread_mutex_lock(&v250_mtx);
    r = v250_ctl_command(&v250, text, info, sizeof(info));
    pthread_mutex_unlock(&v250_mtx);
    if (r != V250_CTL_OK)
        return -1;
    if (info[0])
        at_put_response(at, info);
    return 0;
}

void di_get_v250_settings(v250_ctl_t *out)
{
    pthread_mutex_lock(&v250_mtx);
    *out = v250;
    pthread_mutex_unlock(&v250_mtx);
}

void di_set_connect_info_cb(di_connect_info_cb_t cb)
{
    connect_info_cb = cb;
}

void di_set_modulation_ops(di_ms_set_cb_t set, di_ms_get_cb_t get,
                           di_ms_reset_cb_t reset)
{
    ms_set_cb = set;
    ms_get_cb = get;
    ms_reset_cb = reset;
}

static int handle_diagnostic(const char *command)
{
    char response[256];
    bool online=connected && at && at->fclass_mode==0 && !fc2_active()
        && at->at_rx_mode==AT_MODE_OFFHOOK_COMMAND;
    if(!command)return -1;
    pthread_mutex_lock(&test_mtx);
    bool was_loop=diagnostics.local_loop;
    int result=at_test_command(&diagnostics,command,online,response,sizeof(response));
    if(result==0) {
        /* Entering/stopping a loop cannot leak test bytes into later calls.
         * Starting a BERT discards pending local echo, per 6.7.2.13. */
        bool bert_action=!strncmp(command,"+TTER=",6) && command[6]!='?';
        if(was_loop!=diagnostics.local_loop || bert_action) {
            ring_clear(&local_loop_ring);
            test_last_ms=now_ms();test_bit_fraction=0;
        }
    }
    pthread_mutex_unlock(&test_mtx);
    if(result==0 && response[0])at_put_response(at,response);
    return result;
}

/* ------------------------------------------------------------------ */
/* "$" help and ATI pages                                              */
/* ------------------------------------------------------------------ */

void di_update_link(const v250_connect_report_t *rep, const char *detail, const char *state)
{
    pthread_mutex_lock(&test_mtx);
    if (link_now.active && !link_now.fax) {
        if (rep) {
            if (rep->carrier)
                snprintf(link_now.carrier, sizeof(link_now.carrier), "%s", rep->carrier);
            snprintf(link_now.ec, sizeof(link_now.ec), "%s", rep->ec ? rep->ec : "NONE");
            if (rep->tx_rate > 0) {
                link_now.tx_rate = rep->tx_rate;
                link_now.rx_rate = rep->rx_rate;
            }
            link_now.dc_scheme = rep->dc_scheme;
            link_now.dc_tx = rep->dc_tx;
            link_now.dc_rx = rep->dc_rx;
        }
        if (detail)
            snprintf(link_now.detail, sizeof(link_now.detail), "%s", detail);
        if (state)
            snprintf(link_now.state, sizeof(link_now.state), "%s", state);
        link_now.updated_ms = now_ms();
    }
    pthread_mutex_unlock(&test_mtx);
}

/* Power-on: ATI6/ATI11 have no call to report until one connects. */
static void link_reset(void)
{
    pthread_mutex_lock(&test_mtx);
    memset(&link_now, 0, sizeof(link_now));
    link_next_originate = false;
    pthread_mutex_unlock(&test_mtx);
}

void di_set_link_detail_cb(di_link_detail_cb_t cb)
{
    link_detail_cb = cb;
}

/* At CONNECT, in the engine's context (so the detail callback may read the
 * engine's state).  The detail text is taken before test_mtx: the callback is
 * the engine's, and nothing of the engine's is called under this leaf. */
static void link_latch(int rate, const v250_connect_report_t *rep, bool fax)
{
    char detail[sizeof(link_now.detail)];
    bool originate = false;
    bool engine_knows = false;

    detail[0] = '\0';
    if (link_detail_cb && !fax) {
        link_detail_cb(detail, sizeof(detail), &originate);
        engine_knows = true;
    }
    pthread_mutex_lock(&test_mtx);
    link_now.valid = true;
    link_now.active = true;
    link_now.originate = engine_knows ? originate : link_next_originate;
    link_now.fax = fax;
    link_now.start_ms = now_ms();
    link_now.end_ms = 0;
    link_now.rate = rate;
    snprintf(link_now.carrier, sizeof(link_now.carrier), "%s",
             fax ? "FAX" : (rep->carrier ? rep->carrier : "-"));
    snprintf(link_now.ec, sizeof(link_now.ec), "%s", rep->ec ? rep->ec : "NONE");
    link_now.tx_rate = rep->tx_rate > 0 ? rep->tx_rate : rate;
    link_now.rx_rate = rep->rx_rate;
    link_now.dc_scheme = rep->dc_scheme;
    link_now.dc_tx = rep->dc_tx;
    link_now.dc_rx = rep->dc_rx;
    link_now.to_line = 0;
    link_now.to_dte = 0;
    link_now.cause[0] = '\0';
    snprintf(link_now.state, sizeof(link_now.state), "data");
    link_now.updated_ms = link_now.start_ms;
    snprintf(link_now.detail, sizeof(link_now.detail), "%s", detail);
    pthread_mutex_unlock(&test_mtx);
}

/* at_put_response() frames the text with its own line ends. */
static void put_page(char *text)
{
    size_t n = strlen(text);

    while (n > 0 && (text[n - 1] == '\r' || text[n - 1] == '\n'))
        text[--n] = '\0';
    at_put_response(at, text);
}

static int handle_help(const char *topic)
{
    char buf[4096];

    if (!topic || at_help_format(topic, at ? at->p.s_regs : NULL, buf, sizeof(buf)) < 0)
        return -1;
    put_page(buf);
    return 1;
}

typedef struct {
    char *p;
    size_t left;
} page_t;

static void page_put(page_t *pg, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (pg->left == 0)
        return;
    va_start(ap, fmt);
    n = vsnprintf(pg->p, pg->left, fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if ((size_t) n >= pg->left)
        n = (int) pg->left - 1;
    pg->p += n;
    pg->left -= (size_t) n;
}

static const char *console_desc(char *buf, size_t len)
{
    if (split_mode)
        snprintf(buf, len, "control %s, data %s",
                 ctrl_pty.symlink_path[0] ? ctrl_pty.symlink_path : ctrl_pty.slave_name,
                 data_pty.symlink_path[0] ? data_pty.symlink_path : data_pty.slave_name);
    else
        snprintf(buf, len, "combined %s",
                 ctrl_pty.symlink_path[0] ? ctrl_pty.symlink_path : ctrl_pty.slave_name);
    return buf;
}

/* ATI4: the settings the next call will use, in the commands that set them. */
static void info_settings(page_t *pg)
{
    static const char *const fclass[] = { "0", "1", "1.0", "2.0" };
    static const int sregs[] = { 0, 3, 4, 5, 6, 7, 8, 10 };
    static const char *const v250_reads[] = { "MR?", "ER?", "DR?", "ES?", "DS?" };
    char line[160];
    v250_ctl_t cfg;

    page_put(pg, "Current Settings...\r\n");
    page_put(pg, "E%d Q%d V%d X%d &C%d &D%d %s\r\n",
             at->p.echo ? 1 : 0, at->p.result_code_format == DI_NO_RESULT_CODES ? 1 : 0,
             at->p.verbose ? 1 : 0, at->result_code_mode, at->rlsd_behaviour,
             at->dtr_behaviour, at->p.pulse_dial ? "P" : "T");
    for (size_t i = 0; i < sizeof(sregs) / sizeof(sregs[0]); i++)
        page_put(pg, "S%02d=%03d%s", sregs[i], at->p.s_regs[sregs[i]],
                 i + 1 < sizeof(sregs) / sizeof(sregs[0]) ? " " : "\r\n");
    page_put(pg, "+FCLASS=%s\r\n",
             at->fclass_mode >= 0 && at->fclass_mode < 4 ? fclass[at->fclass_mode] : "?");
    if (ms_get_cb) {
        at_ms_settings_t ms;

        plus_ms_current(&ms);
        at_ms_format_read(&ms, line, sizeof(line));
        page_put(pg, "%s\r\n", line);
    }
    di_get_v250_settings(&cfg);
    for (size_t i = 0; i < sizeof(v250_reads) / sizeof(v250_reads[0]); i++) {
        v250_ctl_t copy = cfg;

        if (v250_ctl_command(&copy, v250_reads[i], line, sizeof(line)) == V250_CTL_OK)
            page_put(pg, "%s%s", line, i + 1 < sizeof(v250_reads) / sizeof(v250_reads[0]) ? "  " : "\r\n");
    }
    page_put(pg, "Console: %s\r\n", console_desc(line, sizeof(line)));
}

static void fmt_duration(char *buf, size_t len, int64_t ms)
{
    int64_t s = ms / 1000;

    snprintf(buf, len, "%02lld:%02lld:%02lld",
             (long long) (s / 3600), (long long) ((s / 60) % 60), (long long) (s % 60));
}

/* ATI6: the current or last call, after the Courier's link diagnostics. */
static void info_link(page_t *pg)
{
    di_link_t l;
    char dur[32];

    pthread_mutex_lock(&test_mtx);
    l = link_now;
    pthread_mutex_unlock(&test_mtx);
    page_put(pg, "Link Diagnostics...\r\n");
    if (!l.valid) {
        page_put(pg, "No call since power-on.\r\n");
        return;
    }
    fmt_duration(dur, sizeof(dur), (l.active ? now_ms() : l.end_ms) - l.start_ms);
    page_put(pg, "Call               %s, %s\r\n", l.originate ? "Originate" : "Answer",
             l.active ? (strcmp(l.state, "data") ? l.state : "in progress")
                      : l.failed ? "failed before data mode" : "ended");
    if (l.failed) {
        page_put(pg, "Disconnect reason  %s\r\n", l.cause);
        return;
    }
    page_put(pg, "Modulation         %s\r\n", l.carrier);
    if (l.rx_rate > 0 && l.rx_rate != l.tx_rate)
        page_put(pg, "Rate               TX %d  RX %d\r\n", l.tx_rate, l.rx_rate);
    else
        page_put(pg, "Rate               %d\r\n", l.tx_rate);
    page_put(pg, "CONNECT rate       %d\r\n", l.rate);
    if (!l.fax) {
        page_put(pg, "Error control      %s\r\n", strcmp(l.ec, "LAPM") ? "None (V.14)" : "V.42 LAPM");
        if (l.dc_scheme == 0 || (!l.dc_tx && !l.dc_rx))
            page_put(pg, "Compression        None\r\n");
        else
            page_put(pg, "Compression        %s%s%s\r\n", l.dc_scheme == 2 ? "V.44" : "V.42bis",
                     l.dc_tx ? " TX" : "", l.dc_rx ? " RX" : "");
        page_put(pg, "Octets to line     %llu\r\n", (unsigned long long) l.to_line);
        page_put(pg, "Octets to DTE      %llu\r\n", (unsigned long long) l.to_dte);
    }
    page_put(pg, "Duration           %s\r\n", dur);
    {
        float rx, tx;
        bool have_rx = lm_level_dbm0(LM_RX, &rx);
        bool have_tx = lm_level_dbm0(LM_TX, &tx);

        if (have_rx || have_tx) {
            page_put(pg, "%s", l.active ? "Line level now     " : "Line level at end  ");
            if (have_rx)
                page_put(pg, "RX %.1f dBm0  ", rx);
            if (have_tx)
                page_put(pg, "TX %.1f dBm0", tx);
            page_put(pg, "\r\n");
        }
    }
    page_put(pg, "Disconnect reason  %s\r\n", l.active ? "-" : l.cause);
}

/* ATI7: what this build can do. */
static void info_config(page_t *pg)
{
    char line[512];

    page_put(pg, "Product Configuration...\r\n");
    page_put(pg, "Product            v90modem %s\r\n", V90MODEM_VERSION);
    page_put(pg, "Line               SIP, G.711 PCMU/PCMA, byte-exact passthrough\r\n");
    at_ms_format_test(line, sizeof(line));
    page_put(pg, "Modulations        %s\r\n", line);
    page_put(pg, "Error control      V.42 LAPM, buffered (V.14)\r\n");
    page_put(pg, "Compression        V.42bis (+DS); V.44 by ME_DATA_COMPRESSION=v44\r\n");
    page_put(pg, "Fax                Class 1, 1.0 (T.31), 2.0 (T.32)\r\n");
    page_put(pg, "Diagnostics        +TLDL loop, +TTER/+TNUM error test, +TSELF\r\n");
    page_put(pg, "Console            %s\r\n", console_desc(line, sizeof(line)));
}

/* ATI11: the engine's own detail, latched at CONNECT. */
static void info_link_detail(page_t *pg)
{
    di_link_t l;

    pthread_mutex_lock(&test_mtx);
    l = link_now;
    pthread_mutex_unlock(&test_mtx);
    page_put(pg, "Extended Link Diagnostics (%s)...\r\n",
             !l.valid || l.failed ? "no call" : l.active ? "live" : "at end of call");
    if (!l.valid)
        page_put(pg, "No call since power-on.\r\n");
    else if (l.failed)
        page_put(pg, "The last call failed before data mode (see ATI6).\r\n");
    else if (l.fax)
        page_put(pg, "Fax call: see the +F session reports.\r\n");
    else if (!l.detail[0])
        page_put(pg, "The engine reported no detail for this call.\r\n");
    else
        page_put(pg, "%s\r\n", l.detail);
}

/* ATY<n>.  Only the Courier's frequency/level table so far. */
static int handle_diag_table(const char *num)
{
    char buf[4096];

    if (!num || atoi(num) != 11)
        return -1;
    if (lm_format_bands(buf, sizeof(buf)) < 0)
        return -1;
    put_page(buf);
    return 1;
}

static int handle_info(const char *num)
{
    char buf[4096];
    page_t pg = { buf, sizeof(buf) };

    if (!num || !at)
        return 0;
    buf[0] = '\0';
    switch (atoi(num)) {
    case 0:
        page_put(&pg, "v90modem SIP V.90/V.92 data and fax modem");
        break;
    case 3:
        page_put(&pg, "v90modem %s", V90MODEM_VERSION);
        break;
    case 4:
        info_settings(&pg);
        break;
    case 6:
        info_link(&pg);
        break;
    case 7:
        info_config(&pg);
        break;
    case 11:
        info_link_detail(&pg);
        break;
    default:
        return 0;           /* the interpreter's: ERROR */
    }
    put_page(buf);
    return 1;
}

static int at_modem_control_handler(t31_state_t *t31_state, void *user_data,
                                    int op, const char *num)
{
    (void)t31_state;
    (void)user_data;

    switch (op) {
    case AT_MODEM_CONTROL_CALL:
        pthread_mutex_lock(&test_mtx);
        link_next_originate = true;
        pthread_mutex_unlock(&test_mtx);
        if (dial_cb && num && num[0])
            dial_cb(num, cb_user_data);
        break;

    case AT_MODEM_CONTROL_ANSWER:
        pthread_mutex_lock(&test_mtx);
        link_next_originate = false;
        pthread_mutex_unlock(&test_mtx);
        if (answer_cb)
            answer_cb(cb_user_data);
        break;

    case AT_MODEM_CONTROL_HANGUP:
    case AT_MODEM_CONTROL_ONHOOK:
        /* A remote SIP teardown is reported to T.31 as HANGUP so it can
         * flush its datapumps.  Do not feed that notification back into the
         * engine as a new local ATH: di_on_disconnected() has already cleared
         * connected, and the engine has just returned to ME_IDLE. */
        if (hangup_cb && connected) {
            local_hangup = 1;
            hangup_cb(cb_user_data);
        }
        break;

    /* Signal line controls — not connected to real hardware */
    case AT_MODEM_CONTROL_DTR:
    case AT_MODEM_CONTROL_RTS:
    case AT_MODEM_CONTROL_CTS:
    case AT_MODEM_CONTROL_CAR:
    case AT_MODEM_CONTROL_RNG:
    case AT_MODEM_CONTROL_DSR:
        break;

    case AT_MODEM_CONTROL_MODULATION:
        if(!num)diagnostic_reset(true); /* ATZ/AT&F, V.250 6.7.2.17 */
        return handle_plus_ms(num);
    case AT_MODEM_CONTROL_DIAGNOSTIC:
        return handle_diagnostic(num);

    case AT_MODEM_CONTROL_PARAMETER:
        return handle_v250_parameter(num);

    case AT_MODEM_CONTROL_HELP:
        return handle_help(num);

    case AT_MODEM_CONTROL_INFO:
        return handle_info(num);

    case AT_MODEM_CONTROL_DIAG_TABLE:
        return handle_diag_table(num);

    case AT_MODEM_CONTROL_RESUME:
        /* V.250 6.3.7.  With a separate data port there is no online data
         * state on this port to return to: the payload path is the other PTY.
         * ATO therefore just names it (slave device path) and answers OK,
         * call up or not.  On the combined console it resumes the data
         * connection, or answers NO CARRIER if there is none.  A fax class
         * owns its own session and keeps the interpreter's behaviour. */
        if (split_mode) {
            at_put_response(at, data_pty.slave_name);
            return 1;
        }
        if (!connected)
            return -1;
        /* t31_mtx is already held on this path; di_fax_active() would take it. */
        return 0;

    default:
        break;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Class 2.0 (T.32) callbacks                                          */
/* ------------------------------------------------------------------ */

static void fc2_write(const uint8_t *buf, int len, void *user_data)
{
    (void)user_data;
    ctrl_write(buf, (size_t)len);
}

static void fc2_dial(const char *number, void *user_data)
{
    (void)user_data;
    if (dial_cb && number && number[0])
        dial_cb(number, cb_user_data);
}

static void fc2_answer(void *user_data)
{
    (void)user_data;
    if (answer_cb)
        answer_cb(cb_user_data);
}

static void fc2_hangup(void *user_data)
{
    (void)user_data;
    if (hangup_cb)
        hangup_cb(cb_user_data);
}

/* ------------------------------------------------------------------ */
/* Mode A escape ("+++") and command/data byte handling               */
/* ------------------------------------------------------------------ */

static void flush_pending_escape_bytes(void)
{
    static const uint8_t pluses[3] = { '+', '+', '+' };

    if (esc_count > 0) {
        dte_payload(pluses,esc_count);
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
            dte_payload(&byte,1);
        }
        last_data_byte_ms = now;
    }
}

/* Mode A command-mode bytes: feed the AT interpreter, then honour ATO by
 * following SpanDSP's own mode switch back to online data. */
/* AT+FCLASS=2.0 selects class 2.0 and AT+FCLASS=0 leaves it; fclass_mode is
 * T.31's, so this follows it rather than parsing +FCLASS twice. */
static void sync_fax_class(void)
{
    int want = (at && at->fclass_mode == 3);
    if(at && at->fclass_mode!=0)diagnostic_reset(false);

    if (want != fc2_active()) {
        fc2_select(want);
        if (!want)
            fc2_line_len = 0;
    }
}

/* One class 2.0 line: ours if fax_class2.c claims it, T.31's otherwise. */
static void fc2_dispatch_line(void)
{
    char line[FC2_LINE_MAX + 2];

    fc2_line[fc2_line_len] = '\0';
    if (fc2_line_len == 0)
        return;
    if (!fc2_at_line(fc2_line)) {
        int n = snprintf(line, sizeof(line), "%s\r", fc2_line);

        pthread_mutex_lock(&t31_mtx);
        t31_at_rx(t31, line, n);
        sync_fax_class();
        pthread_mutex_unlock(&t31_mtx);
    }
    fc2_line_len = 0;
}

static void handle_class2_bytes(const uint8_t *buf, int n)
{
    for (int i = 0; i < n; i++) {
        if (fc2_in_dte_data()) {
            /* T.32 3.2: DLE-stuffed image data until <DLE><ETX>.  Hand over
             * the whole of the rest of the block; fax_class2.c finds the end
             * and stops there. */
            fc2_dte_bytes(buf + i, n - i);
            return;
        }
        if (buf[i] == '\r' || buf[i] == '\n') {
            fc2_dispatch_line();
        } else if (fc2_line_len < FC2_LINE_MAX - 1) {
            if (at && at->p.echo && !fc2_echo_suppressed())
                ctrl_write(&buf[i], 1);
            fc2_line[fc2_line_len++] = (char) buf[i];
        }
    }
}

static void handle_command_bytes(const uint8_t *buf, int n)
{
    if (fc2_active()) {
        handle_class2_bytes(buf, n);
        return;
    }

    pthread_mutex_lock(&t31_mtx);
    t31_at_rx(t31, (const char *)buf, n);
    sync_fax_class();
    pthread_mutex_unlock(&t31_mtx);
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

        /* Flow control toward the DTE: while online, read payload only when
         * there is room for it.  Unread bytes stay in the pty, which fills
         * and blocks the DTE's writes -- the alternative, reading and then
         * truncating in ring_write(), lost whole runs of a bulk transfer
         * (artifacts/slm-v90-pay1: 521 gaps downstream at 54666 bit/s). */
        bool payload_room = payload_has_room(sizeof(buf));

        FD_ZERO(&fds);
        if (split_mode || di_mode != 1 || payload_room)
            FD_SET(ctrl_pty.master_fd, &fds);
        if (split_mode && data_pty.master_fd >= 0 && payload_room) {
            FD_SET(data_pty.master_fd, &fds);
            if (data_pty.master_fd > maxfd)
                maxfd = data_pty.master_fd;
        }
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 }; /* 50 ms */

        int r = select(maxfd + 1, &fds, NULL, NULL, &tv);

        diagnostic_poll();

        /* Class 2.0 reports raised by the media thread are emitted here, and
         * so is the page a +FDR has been waiting for. */
        if (fc2_active())
            fc2_poll();

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
                dte_payload(buf,n);
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

    /* A modem port does its own echo (ATE) and line handling, so the slave
     * starts raw.  Left in the default cooked mode with ECHO on, a DTE that
     * opens the port without configuring it (cat, a test, a script) has the
     * line discipline reflect every response straight back as input: once
     * ATE1 is on, the modem's echo of a command is echoed back to the modem,
     * which echoes it again.  A DTE that sets its own termios is unaffected.
     *
     * The slave stays open here for the life of the port.  BSD ptys (macOS)
     * reinitialise the slave's termios to TTYDEF_* on its first open, so a
     * raw setting made and then closed is gone by the time the DTE opens the
     * port: ICRNL turned every CR into LF and ECHO looped the modem's output
     * back in as DTE data (each engine_pair_test V.22bis/V.32bis row read
     * "CONNECT" five times in 64 KB of LFs and failed).  Linux keeps it, which
     * is why the rows passed there.  Holding a reference also stops the
     * master reading EOF/EIO whenever no DTE is attached. */
    {
        struct termios tio;

        if (tcgetattr(slave_fd, &tio) == 0) {
            cfmakeraw(&tio);
            tcsetattr(slave_fd, TCSANOW, &tio);
        }
    }
    fcntl(slave_fd, F_SETFD, FD_CLOEXEC);
    p->slave_hold_fd = slave_fd;

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
    if (p->slave_hold_fd >= 0) {
        close(p->slave_hold_fd);
        p->slave_hold_fd = -1;
    }
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
    pthread_mutex_lock(&v250_mtx);
    v250_ctl_reset(&v250);
    pthread_mutex_unlock(&v250_mtx);
    ring_init(&local_loop_ring);
    diagnostic_reset(true);
    t31 = t31_init(NULL, at_tx_handler, NULL,
                   at_modem_control_handler, NULL, NULL, NULL);
    if (!t31) {
        pthread_mutex_destroy(&upstream_ring.mtx);
        pthread_mutex_destroy(&local_loop_ring.mtx);
        fprintf(stderr, "di_open: t31_init failed\n");
        return -1;
    }
    at = t31_get_at_state(t31);
    /* Audio mode, and keep generating silence when the fax transmitter is
     * idle: the RTP path needs a sample for every timeslot either way. */
    t31_set_mode(t31, false);
    t31_set_transmit_on_idle(t31, true);
    at_set_at_rx_mode(at, AT_MODE_ONHOOK_COMMAND);

    fc2_init(fc2_write, fc2_dial, fc2_answer, fc2_hangup, NULL);

    running = 1;
    if (pthread_create(&reader_tid, NULL, pty_reader_thread, NULL) != 0) {
        perror("pthread_create");
        t31_free(t31);
        t31 = NULL;
        at = NULL;
        pthread_mutex_destroy(&upstream_ring.mtx);
        pthread_mutex_destroy(&local_loop_ring.mtx);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

int di_open(const char *link_path)
{
    link_reset();
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
    link_reset();
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

    diagnostic_reset(true);
    pthread_mutex_destroy(&upstream_ring.mtx);
    pthread_mutex_destroy(&local_loop_ring.mtx);
    fc2_release();
    if (t31) { t31_free(t31); t31 = NULL; at = NULL; }
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
    v250_connect_report_t rep;
    bool fax = fc2_active() || di_fax_active();

    /* What the call settled on, asked once: it feeds the +MCR/+ER/+DR reports
     * below and ATI6, which the DTE may read after the call (or with ATQ1). */
    memset(&rep, 0, sizeof(rep));
    rep.tx_rate = rate;
    rep.ec = "NONE";
    if (connect_info_cb && !fax)
        connect_info_cb(rate, &rep);
    link_latch(rate, &rep, fax);

    diagnostic_reset(false);
    pthread_mutex_lock(&test_mtx);
    test_rate=rate>0?rate:9600;
    pthread_mutex_unlock(&test_mtx);
    connected = 1;
    esc_count = 0;
    last_data_byte_ms = now_ms();

    /* A fax class call reports its own result: T.31 8.2 has the modem answer
     * OK and sit in off-hook command mode, waiting for the class 1 action
     * commands that carry T.30.  A CONNECT <rate> here would be a data-mode
     * result code the fax DTE is not expecting. */
    if (fc2_active()) {
        /* T.32: the DCE runs T.30 from here.  The DTE hears about the call
         * through the session reports (+FCS, +FPS, +FHS), not CONNECT. */
        fc2_on_connected();
        return;
    }

    if (di_fax_active()) {
        pthread_mutex_lock(&t31_mtx);
        t31_call_event(t31, AT_CALL_EVENT_CONNECTED);
        pthread_mutex_unlock(&t31_mtx);
        return;
    }

    if (split_mode) {
        /* Control port stays in command mode; data port goes live. */
        at_set_at_rx_mode(at, AT_MODE_OFFHOOK_COMMAND);
    } else {
        di_mode = 1;
        at_set_at_rx_mode(at, AT_MODE_CONNECTED);
    }

    /* 6.4.3, 6.5.5, 6.6.3: the modulation, error control and compression
     * reports go out at the point the DCE has settled them, in that order, and
     * before CONNECT.  They are result codes, so ATQ1 silences them. */
    if (connect_info_cb && at && at->p.result_code_format != DI_NO_RESULT_CODES) {
        v250_ctl_t cfg;
        char text[256];
        size_t n;

        di_get_v250_settings(&cfg);
        n = v250_ctl_format_report(&cfg, &rep, text, sizeof(text));
        if (n)
            ctrl_write(text, n);
    }

    snprintf(msg, sizeof(msg), "\r\nCONNECT %d\r\n", rate);
    if (ctrl_pty.master_fd >= 0)
        write(ctrl_pty.master_fd, msg, strlen(msg));
}

void di_on_disconnected(void)
{
    di_on_disconnected_cause(NULL, -1);
}

void di_on_disconnected_cause(const char *cause, int originate)
{
    int local = local_hangup;

    pthread_mutex_lock(&test_mtx);
    if (!link_now.active) {
        /* Ended before CONNECT: ATI6 reports the attempt, not an older call. */
        memset(&link_now, 0, sizeof(link_now));
        link_now.valid = true;
        link_now.failed = true;
        link_now.originate = originate >= 0 ? originate != 0 : link_next_originate;
        link_now.start_ms = now_ms();
        snprintf(link_now.carrier, sizeof(link_now.carrier), "-");
        snprintf(link_now.ec, sizeof(link_now.ec), "NONE");
    }
    link_now.active = false;
    link_now.end_ms = now_ms();
    snprintf(link_now.cause, sizeof(link_now.cause), "%s",
             local ? "Local (ATH)" : (cause ? cause : "Remote or line"));
    pthread_mutex_unlock(&test_mtx);
    local_hangup = 0;
    connected = 0;
    diagnostic_reset(false);
    di_mode = 0;
    esc_count = 0;
    at_set_at_rx_mode(at, AT_MODE_ONHOOK_COMMAND);
    if (fc2_active()) {
        fc2_on_disconnected();
        return;
    }

    if (di_fax_active()) {
        /* Lets T.31 stop its datapumps and flush any pending DLE ETX. */
        pthread_mutex_lock(&t31_mtx);
        t31_call_event(t31, AT_CALL_EVENT_HANGUP);
        at_put_response_code(at, AT_RESPONSE_CODE_NO_CARRIER);
        pthread_mutex_unlock(&t31_mtx);
        return;
    }
    at_call_event(at, AT_CALL_EVENT_HANGUP);
    if (!local)
        at_put_response_code(at, AT_RESPONSE_CODE_NO_CARRIER);
}

void di_on_ring(void)
{
    pthread_mutex_lock(&test_mtx);
    link_next_originate = false;
    pthread_mutex_unlock(&test_mtx);
    if (di_fax_active()) {
        pthread_mutex_lock(&t31_mtx);
        t31_call_event(t31, AT_CALL_EVENT_ALERTING);
        pthread_mutex_unlock(&t31_mtx);
    } else
        at_call_event(at, AT_CALL_EVENT_ALERTING);
}

int di_read_data(uint8_t *buf, int max_len)
{
    if (!data_port_active())
        return 0;
    pthread_mutex_lock(&test_mtx);
    int n=diagnostics.local_loop?0:ring_read(&upstream_ring,buf,max_len);
    if(n>0)link_now.to_line+=(uint64_t)n;
    pthread_mutex_unlock(&test_mtx);
    return n;
}

int di_write_data(const uint8_t *buf, int len)
{
    /* Serialize route selection with +TLDL activation. A non-blocking write
     * is bounded; checking the flag then unlocking would let remote bytes
     * arrive at the DTE after the loop-start command had already answered OK. */
    pthread_mutex_lock(&test_mtx);
    int n;
    int fd=data_master_fd();
    if(diagnostics.local_loop)n=len;
    else if(fd<0 || !data_port_active())n=0;
    else {
        n=(int)write(fd,buf,(size_t)len);
        if(n>0)link_now.to_dte+=(uint64_t)n;
    }
    pthread_mutex_unlock(&test_mtx);
    return n;
}

/* ------------------------------------------------------------------ */
/* Fax (T.31 class 1) audio path                                       */
/* ------------------------------------------------------------------ */

/*
 * True while the DTE has selected a fax service class (T.31 8.2: AT+FCLASS=1
 * or 1.0).  The engine consults this to hand the call's audio to the fax
 * datapumps instead of running the V.8/V.34/V.90 startup, which is what a
 * fax call needs -- T.30 does its own negotiation in the DTE above us.
 */
int di_fax_active(void)
{
    int active;

    pthread_mutex_lock(&t31_mtx);
    active = (at && at->fclass_mode != 0);
    pthread_mutex_unlock(&t31_mtx);
    return active;
}

int di_fax_rx(const int16_t *amp, int len)
{
    int n;

    if (!amp || len <= 0)
        return 0;
    /* Class 2.0 runs T.30 in the DCE, so its audio belongs to fax_class2.c;
     * class 1 leaves T.30 to the DTE and the audio to T.31's datapumps. */
    if (fc2_active())
        return fc2_rx(amp, len);
    if (!t31)
        return 0;
    /* t31_rx() does not modify the samples; the non-const prototype is
     * SpanDSP's, shared with paths that do. */
    pthread_mutex_lock(&t31_mtx);
    n = t31_rx(t31, (int16_t *) amp, len);
    pthread_mutex_unlock(&t31_mtx);
    return n;
}

int di_fax_tx(int16_t *amp, int len)
{
    int n;

    if (!amp || len <= 0)
        return 0;
    if (fc2_active())
        return fc2_tx(amp, len);
    if (!t31)
        return 0;
    pthread_mutex_lock(&t31_mtx);
    n = t31_tx(t31, amp, len);
    pthread_mutex_unlock(&t31_mtx);
    if (n < 0)
        n = 0;
    if (n < len)
        memset(amp + n, 0, sizeof(int16_t) * (size_t)(len - n));
    return len;
}

int di_fax_v34hdx_start_control(int primary_bit_rate)
{
    return fc2_active() ? fc2_v34hdx_start_control(primary_bit_rate) : -1;
}

int di_fax_v34hdx_get_bit(void)
{
    return fc2_active() ? fc2_v34hdx_get_bit() : SIG_STATUS_END_OF_DATA;
}

void di_fax_v34hdx_put_bit(int bit)
{
    if (fc2_active())
        fc2_v34hdx_put_bit(bit);
}

int di_fax_v34hdx_get_mode(void)
{
    return fc2_active() ? fc2_v34hdx_get_mode() : V34_HALF_DUPLEX_SILENCE;
}
