/*
 * fax_class2.c -- Service class 2.0 fax (ITU-T T.32) over the AT interface
 *
 * See fax_class2.h for the division of labour against class 1.  In short: the
 * DCE runs T.30 here, so this file is a T.32 command layer over SpanDSP's
 * fax_state_t.
 *
 * Image data crosses the DTE link as compressed T.4/T.6 data, and SpanDSP's
 * T.30 sends and receives TIFF files.  SpanDSP's own T.4 codec bridges the
 * two: t4_rx turns the DTE's compressed page into the TIFF T.30 transmits
 * (which also validates it and recovers the row count, which the DTE never
 * states), and t4_tx turns a received TIFF page back into the compressed
 * stream the DTE reads.  Nothing here touches libtiff directly.
 *
 * Documented deviation: T.32 8.3.3 has the DCE report +FCS from the completed
 * Phase B negotiation and then take the page from the DTE.  SpanDSP's T.30
 * decides what to do at the DIS it receives, and needs the document to exist
 * by then, so a +FDT here answers CONNECT and takes the page first.  The DTE
 * still gets its +FCS report, after the data rather than before it.  In the
 * ordinary sequence -- +FDT issued straight after ATD, while the call is
 * still being set up -- the spool is complete long before Phase B, and the
 * only difference the DTE can see is where the report lands.
 */

#include "fax_class2.h"

#include <spandsp.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>

#define DLE 0x10
#define ETX 0x03

/* ------------------------------------------------------------------ */
/* T.32 session parameters (8.5.1.1 +FCC, 8.5.1.2 +FIS, 8.5.1.3 +FCS)  */
/* ------------------------------------------------------------------ */

typedef struct {
    int vr;   /* vertical resolution bitmap */
    int br;   /* bit rate: 0=2400 ... 5=14400 */
    int wd;   /* page width */
    int ln;   /* page length */
    int df;   /* data format: 0=MH 1=MR 2=uncompressed 3=MMR */
    int ec;   /* error correction: 0=off, 1=ECM 64, 2=ECM 256 */
    int bf;   /* binary file transfer */
    int st;   /* scan time */
    int jp;   /* JPEG */
} fc2_params_t;

/* T.32 8.5.1.1: the DCE's own capabilities.  This is what SpanDSP's T.30 and
 * T.4 can actually do here -- fine resolution, up to 14400 (V.17), A4 through
 * unlimited length, MH/MR/MMR, and ECM. */
static const fc2_params_t fc2_caps = {
    .vr = 1, .br = 5, .wd = 0, .ln = 2, .df = 3, .ec = 2, .bf = 0, .st = 7, .jp = 0
};

static const fc2_params_t fc2_defaults = {
    .vr = 1, .br = 5, .wd = 0, .ln = 2, .df = 3, .ec = 2, .bf = 0, .st = 0, .jp = 0
};

typedef enum {
    FC2_IDLE = 0,
    FC2_DTE_TX_DATA,      /* +FDT: taking a page from the DTE */
    FC2_DTE_RX_DATA       /* +FDR: giving a page to the DTE */
} fc2_dte_mode_t;

/* ------------------------------------------------------------------ */
/* Module state                                                        */
/* ------------------------------------------------------------------ */

static pthread_mutex_t  fc2_mtx = PTHREAD_MUTEX_INITIALIZER;

static fc2_write_cb_t   write_cb;
static fc2_dial_cb_t    dial_cb;
static fc2_action_cb_t  answer_cb;
static fc2_action_cb_t  hangup_cb;
static void            *cb_user_data;

static int              selected;          /* AT+FCLASS=2.0 in force */
static fc2_params_t     p_cc;              /* +FCC, DCE capabilities offered */
static fc2_params_t     p_is;              /* +FIS, this session's offer */
static fc2_params_t     p_cs;              /* +FCS, negotiated (read only) */
static char             local_id[21];      /* +FLI */
static char             polling_id[21];    /* +FPI */
static int              p_cr;              /* +FCR, willing to receive */
static int              p_bo;              /* +FBO, phase C bit order */
static int              p_cq, p_ie, p_ct, p_ms, p_ea, p_ffc, p_nr, p_aa, p_ry;

static fax_state_t     *fax;
static int              call_up;
static int              calling_party;
static int              t30_started;

static fc2_dte_mode_t   dte_mode;
static int              dte_saw_dle;

/* +FDT spool: the DTE's compressed page, decoded into a TIFF for T.30. */
static t4_rx_state_t   *spool_rx;
static char             tx_tiff[128];
static int              tx_tiff_ready;     /* a page is spooled and unsent */
static int              tx_pages_sent;

/* +FDR: T.30's received pages land in this TIFF; t4_tx reads them back. */
static char             rx_tiff[128];
static int              rx_pages_done;     /* pages complete in rx_tiff */
static int              rx_pages_given;    /* pages handed to the DTE */
static int              fdr_pending;       /* DTE is waiting in +FDR */
static int              fdt_await_page;    /* +FDT is waiting for the page to go */

static int              session_done;
static int              hangup_code = -1;  /* +FHS, -1 = not reported yet */
static int              page_status = 1;   /* +FPS */

/* Reports raised under the lock (often from the media thread) and emitted by
 * fc2_poll() on the DTE thread. */
#define FC2_Q_SIZE 4096
static char             report_q[FC2_Q_SIZE];
static int              report_len;

/* ------------------------------------------------------------------ */
/* Output                                                              */
/* ------------------------------------------------------------------ */

static void emit_raw(const char *s, int len)
{
    if (write_cb)
        write_cb((const uint8_t *) s, len, cb_user_data);
}

/* Queue a line for fc2_poll().  Callers hold the lock. */
static void queue_line(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

static void queue_line(const char *fmt, ...)
{
    char line[512];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if (n > (int) sizeof(line) - 1)
        n = (int) sizeof(line) - 1;
    if (report_len + n < FC2_Q_SIZE) {
        memcpy(report_q + report_len, line, (size_t) n);
        report_len += n;
    }
}

/* Emit immediately.  Callers are on the DTE thread and hold the lock. */
static void put_line(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

static void put_line(const char *fmt, ...)
{
    char line[512];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if (n > (int) sizeof(line) - 1)
        n = (int) sizeof(line) - 1;
    emit_raw(line, n);
}

static void put_ok(void)     { put_line("\r\nOK\r\n"); }
static void put_error(void)  { put_line("\r\nERROR\r\n"); }

/* ------------------------------------------------------------------ */
/* T.32 <-> SpanDSP parameter mapping                                  */
/* ------------------------------------------------------------------ */

static int bps_to_br(int bps)
{
    switch (bps) {
    case 2400:  return 0;
    case 4800:  return 1;
    case 7200:  return 2;
    case 9600:  return 3;
    case 12000: return 4;
    default:    return 5;
    }
}

/* T.32 8.5.1.1 DF: the phase C data format. */
static int df_to_compression(int df)
{
    switch (df) {
    case 0:  return T4_COMPRESSION_T4_1D;
    case 1:  return T4_COMPRESSION_T4_2D;
    case 3:  return T4_COMPRESSION_T6;
    default: return 0;               /* 2 (uncompressed) is not supported */
    }
}

static int compression_to_df(int compression)
{
    switch (compression) {
    case T4_COMPRESSION_T4_1D: return 0;
    case T4_COMPRESSION_T4_2D: return 1;
    case T4_COMPRESSION_T6:    return 3;
    default:                   return 0;
    }
}

/* Every format at or below the selected one, since DF names a ceiling. */
static int df_to_supported_compressions(int df)
{
    int mask = T4_COMPRESSION_T4_1D;

    if (df >= 1)
        mask |= T4_COMPRESSION_T4_2D;
    if (df >= 3)
        mask |= T4_COMPRESSION_T6;
    return mask;
}

static int vr_to_resolutions(int vr)
{
    int res = T4_RESOLUTION_R8_STANDARD;

    if (vr & 1)
        res |= T4_RESOLUTION_R8_FINE | T4_RESOLUTION_200_200;
    return res | T4_RESOLUTION_200_100;
}

static int br_to_modems(int br)
{
    int modems = T30_SUPPORT_V27TER;

    if (br >= 2)
        modems |= T30_SUPPORT_V29;
    if (br >= 4)
        modems |= T30_SUPPORT_V17;
    return modems;
}

/*
 * T.32 8.5.2.7 +FHS.  T.30's completion codes are finer grained than T.32's
 * two-digit status, so this maps the ones with a clear T.32 counterpart and
 * falls back on the "unspecified phase B/C/D error" codes for the rest,
 * choosing the phase from where the failure was.  The fallback is deliberate:
 * inventing a precise code for an error T.32 does not enumerate would be
 * worse than reporting the phase honestly.
 */
static int t30_status_to_fhs(int status, int transmitting)
{
    switch (status) {
    case T30_ERR_OK:            return 0x00;
    case T30_ERR_CEDTONE:       return 0x11;
    case T30_ERR_T0_EXPIRED:    return 0x11;
    case T30_ERR_T1_EXPIRED:    return 0x11;
    case T30_ERR_INCOMPATIBLE:  return 0x21;
    case T30_ERR_RX_INCAPABLE:  return 0x21;
    case T30_ERR_TX_INCAPABLE:  return 0x21;
    case T30_ERR_NORESSUPPORT:  return 0x21;
    case T30_ERR_NOSIZESUPPORT: return 0x21;
    case T30_ERR_CANNOT_TRAIN:  return transmitting ? 0x22 : 0x43;
    case T30_ERR_TX_GOTDCN:     return 0x23;
    case T30_ERR_TX_NODIS:      return 0x22;
    case T30_ERR_TX_PHBDEAD:    return 0x20;
    case T30_ERR_TX_PHDDEAD:    return 0x70;
    case T30_ERR_TX_BADDCS:     return 0x22;
    case T30_ERR_TX_BADPG:      return 0x71;
    case T30_ERR_RX_NOCARRIER:  return 0x50;
    case T30_ERR_RX_NOEOL:      return 0x50;
    case T30_ERR_RX_NOFAX:      return 0x43;
    case T30_ERR_CALLDROPPED:   return 0x03;
    default:                    return transmitting ? 0x20 : 0x40;
    }
}

static void params_to_string(const fc2_params_t *p, char *out, size_t max)
{
    snprintf(out, max, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
             p->vr, p->br, p->wd, p->ln, p->df, p->ec, p->bf, p->st, p->jp);
}

/*
 * Parse a T.32 subparameter list.  Omitted fields keep their current value,
 * which is what "," means in a class 2.0 parameter list.  Returns 0 on a
 * malformed list.
 */
static int parse_params(const char *s, fc2_params_t *p)
{
    int *field[9] = { &p->vr, &p->br, &p->wd, &p->ln,
                      &p->df, &p->ec, &p->bf, &p->st, &p->jp };
    fc2_params_t work = *p;
    int *wfield[9] = { &work.vr, &work.br, &work.wd, &work.ln,
                       &work.df, &work.ec, &work.bf, &work.st, &work.jp };
    int idx = 0;

    (void) field;
    while (*s && idx < 9) {
        if (*s == ',') {
            s++;
            idx++;
            continue;
        }
        if (*s == '(' ) {
            /* A range list is a query response, not an assignment. */
            return 0;
        }
        if (!isdigit((unsigned char) *s))
            return 0;
        *wfield[idx] = (int) strtol(s, (char **) &s, 10);
        if (*s == ',') {
            s++;
            idx++;
        } else if (*s != '\0') {
            return 0;
        }
    }
    *p = work;
    return 1;
}

/* ------------------------------------------------------------------ */
/* T.30 session                                                        */
/* ------------------------------------------------------------------ */

static void update_negotiated_params(void)
{
    t30_stats_t t;

    if (!fax)
        return;
    t30_get_transfer_statistics(fax_get_t30_state(fax), &t);
    p_cs = p_is;
    p_cs.br = bps_to_br(t.bit_rate);
    p_cs.ec = t.error_correcting_mode ? p_is.ec : 0;
    p_cs.df = compression_to_df(t.compression);
    p_cs.vr = (t.y_resolution > 100) ? 1 : 0;
    if (t.width > 0 && t.width != 1728)
        p_cs.wd = (t.width >= 2432) ? 2 : 1;
}

/* T.30 Phase B: the negotiation is done, so +FCS and the remote ID are known
 * (T.32 8.5.1.3, and the +FTI/+FCI report of the remote's identification). */
static int phase_b_handler(void *user_data, int result)
{
    t30_stats_t t;
    const char *ident;
    char params[64];

    (void) user_data;
    (void) result;

    update_negotiated_params();
    params_to_string(&p_cs, params, sizeof(params));

    t30_get_transfer_statistics(fax_get_t30_state(fax), &t);
    ident = t30_get_rx_ident(fax_get_t30_state(fax));
    if (ident && ident[0]) {
        /* T.32 8.5.1.5.  We hear the remote's CSI when we called it and its
         * TSI when it called us. */
        queue_line("\r\n%s:\"%s\"\r\n", calling_party ? "+FCI" : "+FTI", ident);
    }
    queue_line("\r\n+FCS:%s\r\n", params);
    return T30_ERR_OK;
}

/* T.30 Phase D: a page has been transferred (T.32 8.5.2.2 +FPS). */
static int phase_d_handler(void *user_data, int result)
{
    t30_stats_t t;

    (void) user_data;

    t30_get_transfer_statistics(fax_get_t30_state(fax), &t);
    switch (result) {
    case T30_MPS:
    case T30_EOM:
    case T30_EOP:
        page_status = 1;                    /* page good */
        break;
    case T30_MCF:
        page_status = 1;
        break;
    case T30_RTN:
        page_status = 2;                    /* page bad, retrain requested */
        break;
    case T30_RTP:
        page_status = 3;                    /* page good, retrain requested */
        break;
    default:
        break;
    }
    rx_pages_done = t.pages_rx;
    if (t.pages_tx > tx_pages_sent) {
        tx_pages_sent = t.pages_tx;
        if (fdt_await_page) {
            /* T.32 8.3.3: the +FDT completes when the page has been sent, not
             * when the DTE finished handing it over. */
            fdt_await_page = 0;
            tx_tiff_ready = 0;
            queue_line("\r\n+FPS:%d\r\n", page_status);
            queue_line("\r\nOK\r\n");
        }
    }
    return T30_ERR_OK;
}

/* T.30 Phase E: the call is over (T.32 8.5.2.7 +FHS). */
static void phase_e_handler(void *user_data, int completion_code)
{
    (void) user_data;

    hangup_code = t30_status_to_fhs(completion_code, tx_tiff_ready || tx_pages_sent > 0);
    session_done = 1;
}

static void session_stop(void)
{
    if (fax) {
        fax_free(fax);
        fax = NULL;
    }
    t30_started = 0;
}

static void session_start(void)
{
    t30_state_t *t30;

    if (fax || !call_up)
        return;

    fax = fax_init(NULL, calling_party ? true : false);
    if (!fax)
        return;
    fax_set_transmit_on_idle(fax, true);
    t30 = fax_get_t30_state(fax);

    t30_set_phase_b_handler(t30, phase_b_handler, NULL);
    t30_set_phase_d_handler(t30, phase_d_handler, NULL);
    t30_set_phase_e_handler(t30, phase_e_handler, NULL);

    t30_set_tx_ident(t30, local_id[0] ? local_id : "");
    t30_set_supported_modems(t30, br_to_modems(p_is.br));
    t30_set_supported_compressions(t30, df_to_supported_compressions(p_is.df));
    t30_set_supported_bilevel_resolutions(t30, vr_to_resolutions(p_is.vr));
    t30_set_ecm_capability(t30, p_is.ec != 0);

    /* The page the DTE handed us in +FDT, if any. */
    if (tx_tiff_ready)
        t30_set_tx_file(t30, tx_tiff, -1, -1);

    /*
     * T.32 8.5.1.9 +FCR: whether we will accept a document.  T.30 advertises
     * "ready to receive" only when it has somewhere to put it, so the receive
     * spool is opened whenever the DTE has not said no.
     */
    if (p_cr) {
        /* Keep what arrives in the format it arrived in, so the stream handed
         * to the DTE in +FDR is the one the far end sent. */
        t30_set_supported_output_compressions(t30,
                                              T4_COMPRESSION_T4_1D
                                              | T4_COMPRESSION_T4_2D
                                              | T4_COMPRESSION_T6);
        t30_set_rx_file(t30, rx_tiff, -1);
    }

    session_done = 0;
    hangup_code = -1;
    t30_started = 1;
}

/* ------------------------------------------------------------------ */
/* T.32 8.5.3.4 +FBO -- phase C bit order                              */
/* ------------------------------------------------------------------ */

/*
 * The octets T.4 hands us are transmitted LSB first: t4_tx_get_bit() takes
 * bit 0 of each octet first, and T.31's class 1 path does the same with the
 * DTE's own octets and no reversal at all.  So the stream this module carries
 * by default is the one a class 1 DTE sees, which is +FBO's direct order --
 * +FBO=0 changes nothing, and the reversed setting flips each phase C octet
 * on its way across the DTE link.
 *
 * +FBO has two halves: the phase C image data, and the negotiation frame data
 * a DTE reads with +FNR.  0 is direct in both and 3 is reversed in both; of
 * the two mixed values this takes 1 as "phase C reversed", the reading that
 * puts the phase C bit in the low position.  Nothing here reports negotiation
 * frames to the DTE, so the other half has nothing to act on and 0/1 behave
 * as 2/3 do.
 */
static int phase_c_reversed(void)
{
    return (p_bo & 1) != 0;
}

static void reverse_bits(uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
        buf[i] = bit_reverse8(buf[i]);
}

/* ------------------------------------------------------------------ */
/* +FDT: taking a page from the DTE (T.32 8.3.3)                       */
/* ------------------------------------------------------------------ */

static void make_temp_name(char *out, size_t max, const char *tag)
{
    snprintf(out, max, "/tmp/v90modem-fax-%s-%d.tif", tag, (int) getpid());
}

static int spool_open(void)
{
    int compression = df_to_compression(p_is.df);

    if (compression == 0)
        return 0;

    make_temp_name(tx_tiff, sizeof(tx_tiff), "tx");
    unlink(tx_tiff);

    /*
     * The DTE's page is compressed data with no row count in it, so it is fed
     * through the T.4 decoder that T.30 would use on a received page.  That
     * writes the TIFF T.30 transmits, and rejects a page the DTE has garbled
     * rather than putting it on the line.
     */
    spool_rx = t4_rx_init(NULL, tx_tiff, T4_COMPRESSION_T4_1D
                                         | T4_COMPRESSION_T4_2D
                                         | T4_COMPRESSION_T6);
    if (!spool_rx)
        return 0;
    t4_rx_set_rx_encoding(spool_rx, compression);
    t4_rx_set_image_width(spool_rx, 1728);
    t4_rx_set_x_resolution(spool_rx, T4_X_RESOLUTION_R8);
    t4_rx_set_y_resolution(spool_rx, (p_is.vr & 1) ? T4_Y_RESOLUTION_FINE
                                                   : T4_Y_RESOLUTION_STANDARD);
    t4_rx_start_page(spool_rx);
    return 1;
}

static void spool_close(void)
{
    if (!spool_rx)
        return;
    t4_rx_end_page(spool_rx);
    t4_rx_free(spool_rx);
    spool_rx = NULL;
    tx_tiff_ready = 1;

    /* A session already under way takes the page straight away. */
    if (fax)
        t30_set_tx_file(fax_get_t30_state(fax), tx_tiff, -1, -1);
}

/* ------------------------------------------------------------------ */
/* +FDR: giving a page to the DTE (T.32 8.3.4)                         */
/* ------------------------------------------------------------------ */

/* DLE-stuff a block on its way to the DTE (T.32 3.2: a DLE in the data is
 * doubled, and <DLE><ETX> ends the stream). */
static void emit_stuffed(const uint8_t *buf, int len)
{
    uint8_t out[1024];
    int n = 0;

    for (int i = 0; i < len; i++) {
        if (n + 2 > (int) sizeof(out)) {
            emit_raw((const char *) out, n);
            n = 0;
        }
        if (buf[i] == DLE)
            out[n++] = DLE;
        out[n++] = buf[i];
    }
    if (n)
        emit_raw((const char *) out, n);
}

/*
 * Read one page back out of the received TIFF in the negotiated format and
 * hand it to the DTE.  t4_tx is the same encoder T.30 uses to transmit, so
 * the DTE gets a stream a fax machine would have sent.
 */
static int send_page_to_dte(int page)
{
    t4_tx_state_t *tx;
    uint8_t buf[512];
    int compression = df_to_compression(p_cs.df);
    int len;

    if (compression == 0)
        compression = T4_COMPRESSION_T4_1D;

    tx = t4_tx_init(NULL, rx_tiff, page, page);
    if (!tx)
        return 0;
    t4_tx_set_tx_image_format(tx, compression,
                              T4_SUPPORT_WIDTH_215MM,
                              T4_RESOLUTION_R8_STANDARD | T4_RESOLUTION_R8_FINE,
                              0);
    if (t4_tx_start_page(tx)) {
        t4_tx_free(tx);
        return 0;
    }
    while ((len = t4_tx_get(tx, buf, sizeof(buf))) > 0) {
        /* Reverse first: DLE stuffing is about the octets that appear on the
         * DTE link, so it has to see the octets the DTE will read. */
        if (phase_c_reversed())
            reverse_bits(buf, len);
        emit_stuffed(buf, len);
    }
    t4_tx_end_page(tx);
    t4_tx_free(tx);

    {
        static const char end[2] = { DLE, ETX };
        emit_raw(end, 2);
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* DTE data mode                                                       */
/* ------------------------------------------------------------------ */

int fc2_in_dte_data(void)
{
    int r;

    pthread_mutex_lock(&fc2_mtx);
    r = (dte_mode == FC2_DTE_TX_DATA);
    pthread_mutex_unlock(&fc2_mtx);
    return r;
}

void fc2_dte_bytes(const uint8_t *buf, int len)
{
    uint8_t clean[512];
    int n = 0;
    int finished = 0;

    pthread_mutex_lock(&fc2_mtx);
    if (dte_mode != FC2_DTE_TX_DATA) {
        pthread_mutex_unlock(&fc2_mtx);
        return;
    }

    for (int i = 0; i < len && !finished; i++) {
        uint8_t byte = buf[i];

        if (dte_saw_dle) {
            dte_saw_dle = 0;
            if (byte == ETX) {
                finished = 1;
                break;
            }
            if (byte == DLE) {
                clean[n++] = DLE;       /* stuffed DLE */
            } else {
                /* T.32 3.2: any other <DLE><chr> is a control sequence we do
                 * not implement; drop it rather than corrupt the image. */
            }
        } else if (byte == DLE) {
            dte_saw_dle = 1;
        } else {
            clean[n++] = byte;
        }

        if (n >= (int) sizeof(clean)) {
            if (spool_rx) {
                if (phase_c_reversed())
                    reverse_bits(clean, n);
                t4_rx_put(spool_rx, clean, (size_t) n);
            }
            n = 0;
        }
    }
    if (n && spool_rx) {
        if (phase_c_reversed())
            reverse_bits(clean, n);
        t4_rx_put(spool_rx, clean, (size_t) n);
    }

    if (finished) {
        spool_close();
        dte_mode = FC2_IDLE;
        dte_saw_dle = 0;
        if (call_up) {
            /* The page goes out on this call; the DTE hears about it when it
             * has (phase_d_handler). */
            fdt_await_page = 1;
            session_start();
        } else {
            /* +FDT ahead of the call, which is the ordinary sequence: the
             * page is spooled and will go out when the call connects. */
            put_line("\r\n+FPS:%d\r\n", page_status);
            put_ok();
        }
    }
    pthread_mutex_unlock(&fc2_mtx);
}

/* ------------------------------------------------------------------ */
/* AT command parsing (T.32 8.3 actions, 8.5 parameters)               */
/* ------------------------------------------------------------------ */

/* Matches "+FXX" at the head of t, case insensitively, and steps past it. */
static int match(const char **t, const char *name)
{
    size_t n = strlen(name);

    if (strncasecmp(*t, name, n) != 0)
        return 0;
    *t += n;
    return 1;
}

/*
 * One numeric parameter: "=n" sets it, "?" reports it, "=?" reports the
 * range.  Returns 1 when the command was well formed.
 */
static int num_param(const char *t, int *value, int min, int max,
                     const char *range)
{
    if (t[0] == '?' && t[1] == '\0') {
        put_line("\r\n%d\r\n", *value);
        put_ok();
        return 1;
    }
    if (t[0] == '=' && t[1] == '?' && t[2] == '\0') {
        put_line("\r\n%s\r\n", range);
        put_ok();
        return 1;
    }
    if (t[0] == '=') {
        char *end;
        long v = strtol(t + 1, &end, 10);

        if (end == t + 1 || *end != '\0' || v < min || v > max)
            return 0;
        *value = (int) v;
        put_ok();
        return 1;
    }
    return 0;
}

/* A quoted string parameter (+FLI, +FPI). */
static int str_param(const char *t, char *value, size_t max)
{
    if (t[0] == '?' && t[1] == '\0') {
        put_line("\r\n\"%s\"\r\n", value);
        put_ok();
        return 1;
    }
    if (t[0] == '=' && t[1] == '?' && t[2] == '\0') {
        put_line("\r\n(20),(32-127)\r\n");
        put_ok();
        return 1;
    }
    if (t[0] == '=') {
        const char *v = t + 1;
        size_t n;

        if (*v == '"')
            v++;
        n = strlen(v);
        if (n && v[n - 1] == '"')
            n--;
        if (n >= max)
            n = max - 1;
        memcpy(value, v, n);
        value[n] = '\0';
        put_ok();
        return 1;
    }
    return 0;
}

/* A subparameter list parameter (+FCC, +FIS, +FCS). */
static int list_param(const char *t, fc2_params_t *p, int read_only)
{
    char buf[64];

    if (t[0] == '?' && t[1] == '\0') {
        params_to_string(p, buf, sizeof(buf));
        put_line("\r\n%s\r\n", buf);
        put_ok();
        return 1;
    }
    if (t[0] == '=' && t[1] == '?' && t[2] == '\0') {
        /* T.32 8.5.1.1: the ranges the DCE supports, in subparameter order. */
        put_line("\r\n(0,1),(0-5),(0),(0-2),(0,1,3),(0-2),(0),(0-7),(0)\r\n");
        put_ok();
        return 1;
    }
    if (t[0] == '=' && !read_only) {
        fc2_params_t work = *p;

        if (!parse_params(t + 1, &work))
            return 0;
        *p = work;
        put_ok();
        return 1;
    }
    return 0;
}

static void params_reset(void)
{
    p_cc = fc2_caps;
    p_is = fc2_defaults;
    p_cs = fc2_defaults;
    p_cr = 1;
    p_bo = 0;
    p_cq = 0; p_ie = 0; p_ct = 30; p_ms = 0; p_ea = 0;
    p_ffc = 0; p_nr = 0; p_aa = 0; p_ry = 3;
    page_status = 1;
    local_id[0] = '\0';
    polling_id[0] = '\0';
}

/*
 * Handle one AT line.  Returns 1 if this module dealt with it.
 *
 * A class 2.0 DTE sends a line that is all fax: T.32 has no concatenation of
 * a +F command with anything else, so one line is one command here, and
 * anything that is not a +F command (or a dial/answer/hangup) goes back to
 * the T.31 interpreter, which owns the S registers, echo and result-code
 * settings.
 */
int fc2_at_line(const char *line)
{
    const char *t = line;
    int handled = 1;
    int ok;

    if (!selected)
        return 0;

    while (*t == ' ')
        t++;
    if (!(t[0] == 'A' || t[0] == 'a') || !(t[1] == 'T' || t[1] == 't'))
        return 0;
    t += 2;
    while (*t == ' ')
        t++;

    pthread_mutex_lock(&fc2_mtx);

    /* Call control.  T.32 leaves these to V.250, but they have to reach the
     * engine and start or stop the T.30 session. */
    if (t[0] == 'D' || t[0] == 'd') {
        char number[128];
        size_t n = strlen(t + 1);

        if (n >= sizeof(number))
            n = sizeof(number) - 1;
        memcpy(number, t + 1, n);
        number[n] = '\0';
        calling_party = 1;
        pthread_mutex_unlock(&fc2_mtx);
        if (dial_cb)
            dial_cb(number, cb_user_data);
        return 1;
    }
    if ((t[0] == 'A' || t[0] == 'a') && t[1] == '\0') {
        calling_party = 0;
        pthread_mutex_unlock(&fc2_mtx);
        if (answer_cb)
            answer_cb(cb_user_data);
        return 1;
    }
    if ((t[0] == 'H' || t[0] == 'h')
        && (t[1] == '\0' || (t[1] == '0' && t[2] == '\0'))) {
        session_stop();
        pthread_mutex_unlock(&fc2_mtx);
        if (hangup_cb)
            hangup_cb(cb_user_data);
        pthread_mutex_lock(&fc2_mtx);
        put_ok();
        pthread_mutex_unlock(&fc2_mtx);
        return 1;
    }

    if (t[0] != '+') {
        pthread_mutex_unlock(&fc2_mtx);
        return 0;
    }

    /* T.31 8.2 -- leaving class 2.0 is handled by the T.31 interpreter, which
     * owns fclass_mode; it calls fc2_select() for us. */
    if (strncasecmp(t, "+FCLASS", 7) == 0) {
        pthread_mutex_unlock(&fc2_mtx);
        return 0;
    }

    ok = 0;
    if (match(&t, "+FCC"))       ok = list_param(t, &p_cc, 0);
    else if (match(&t, "+FIS"))  ok = list_param(t, &p_is, 0);
    else if (match(&t, "+FCS"))  ok = list_param(t, &p_cs, 1);
    else if (match(&t, "+FLI"))  ok = str_param(t, local_id, sizeof(local_id));
    else if (match(&t, "+FPI"))  ok = str_param(t, polling_id, sizeof(polling_id));
    else if (match(&t, "+FCR"))  ok = num_param(t, &p_cr, 0, 1, "(0,1)");
    else if (match(&t, "+FBO"))  ok = num_param(t, &p_bo, 0, 3, "(0-3)");
    else if (match(&t, "+FCQ"))  ok = num_param(t, &p_cq, 0, 2, "(0-2),(0-2)");
    else if (match(&t, "+FIE"))  ok = num_param(t, &p_ie, 0, 1, "(0,1)");
    else if (match(&t, "+FCT"))  ok = num_param(t, &p_ct, 0, 255, "(0-255)");
    else if (match(&t, "+FMS"))  ok = num_param(t, &p_ms, 0, 5, "(0-5)");
    else if (match(&t, "+FEA"))  ok = num_param(t, &p_ea, 0, 1, "(0,1)");
    else if (match(&t, "+FFC"))  ok = num_param(t, &p_ffc, 0, 3, "(0-3)");
    else if (match(&t, "+FNR"))  ok = num_param(t, &p_nr, 0, 1, "(0,1),(0,1),(0,1),(0,1)");
    else if (match(&t, "+FAA"))  ok = num_param(t, &p_aa, 0, 1, "(0,1)");
    else if (match(&t, "+FRY"))  ok = num_param(t, &p_ry, 0, 255, "(0-255)");
    else if (match(&t, "+FPS")) {
        ok = num_param(t, &page_status, 1, 5, "(1-5)");
    } else if (match(&t, "+FHS")) {
        if (t[0] == '?' && t[1] == '\0') {
            put_line("\r\n%02X\r\n", hangup_code < 0 ? 0 : hangup_code);
            put_ok();
            ok = 1;
        }
    } else if (match(&t, "+FBS")) {
        /* T.32 8.5.3.2, read only: transmit buffer size, then receive. */
        if (t[0] == '?' && t[1] == '\0') {
            put_line("\r\n0,0\r\n");
            put_ok();
            ok = 1;
        }
    } else if (match(&t, "+FMI")) {
        put_line("\r\n\"v90modem\"\r\n"); put_ok(); ok = 1;
    } else if (match(&t, "+FMM")) {
        put_line("\r\n\"SIP V.90/V.92 modem\"\r\n"); put_ok(); ok = 1;
    } else if (match(&t, "+FMR")) {
        put_line("\r\n\"2.0\"\r\n"); put_ok(); ok = 1;
    } else if (match(&t, "+FIP")) {
        /* T.32 8.3.6 */
        params_reset();
        put_ok();
        ok = 1;
    } else if (match(&t, "+FKS")) {
        /* T.32 8.3.5: terminate the session in an orderly way. */
        session_stop();
        pthread_mutex_unlock(&fc2_mtx);
        if (hangup_cb)
            hangup_cb(cb_user_data);
        pthread_mutex_lock(&fc2_mtx);
        put_ok();
        ok = 1;
    } else if (match(&t, "+FDT")) {
        /* T.32 8.3.3.  An optional "=DF,VR,WD,LN" sets the format first. */
        if (t[0] == '=') {
            fc2_params_t work = p_is;
            int df = work.df, vr = work.vr, wd = work.wd, ln = work.ln;
            const char *s = t + 1;
            int *f[4] = { &df, &vr, &wd, &ln };
            int idx = 0;

            while (*s && idx < 4) {
                if (*s == ',') { s++; idx++; continue; }
                if (!isdigit((unsigned char) *s)) break;
                *f[idx] = (int) strtol(s, (char **) &s, 10);
                if (*s == ',') { s++; idx++; }
            }
            work.df = df; work.vr = vr; work.wd = wd; work.ln = ln;
            p_is = work;
        }
        if (df_to_compression(p_is.df) == 0) {
            ok = 0;                     /* DF=2, uncompressed, is not offered */
        } else if (spool_open()) {
            dte_mode = FC2_DTE_TX_DATA;
            dte_saw_dle = 0;
            put_line("\r\nCONNECT\r\n");
            ok = 1;
        }
    } else if (match(&t, "+FDR")) {
        /* T.32 8.3.4.  If a page is already in, it goes now; otherwise the
         * DTE waits here until one is (fc2_poll). */
        fdr_pending = 1;
        ok = 1;
    } else {
        handled = 0;
    }

    if (handled && !ok)
        put_error();

    pthread_mutex_unlock(&fc2_mtx);
    return handled;
}

/* ------------------------------------------------------------------ */
/* Public entry points                                                 */
/* ------------------------------------------------------------------ */

void fc2_init(fc2_write_cb_t write_fn,
              fc2_dial_cb_t dial_fn,
              fc2_action_cb_t answer_fn,
              fc2_action_cb_t hangup_fn,
              void *user_data)
{
    pthread_mutex_lock(&fc2_mtx);
    write_cb     = write_fn;
    dial_cb      = dial_fn;
    answer_cb    = answer_fn;
    hangup_cb    = hangup_fn;
    cb_user_data = user_data;
    params_reset();
    make_temp_name(rx_tiff, sizeof(rx_tiff), "rx");
    pthread_mutex_unlock(&fc2_mtx);
}

void fc2_release(void)
{
    pthread_mutex_lock(&fc2_mtx);
    session_stop();
    if (spool_rx) {
        t4_rx_free(spool_rx);
        spool_rx = NULL;
    }
    selected = 0;
    pthread_mutex_unlock(&fc2_mtx);
}

void fc2_select(int on)
{
    pthread_mutex_lock(&fc2_mtx);
    if (on && !selected) {
        params_reset();
        make_temp_name(rx_tiff, sizeof(rx_tiff), "rx");
        unlink(rx_tiff);
        rx_pages_done = 0;
        rx_pages_given = 0;
        tx_pages_sent = 0;
        tx_tiff_ready = 0;
        dte_mode = FC2_IDLE;
        report_len = 0;
    } else if (!on && selected) {
        session_stop();
    }
    selected = on ? 1 : 0;
    pthread_mutex_unlock(&fc2_mtx);
}

int fc2_active(void)
{
    int r;

    pthread_mutex_lock(&fc2_mtx);
    r = selected;
    pthread_mutex_unlock(&fc2_mtx);
    return r;
}

void fc2_on_connected(void)
{
    pthread_mutex_lock(&fc2_mtx);
    call_up = 1;
    session_start();
    pthread_mutex_unlock(&fc2_mtx);
}

void fc2_on_disconnected(void)
{
    pthread_mutex_lock(&fc2_mtx);
    call_up = 0;
    if (hangup_code < 0)
        hangup_code = 0x03;             /* T.32 8.5.2.7: call dropped */
    session_stop();
    pthread_mutex_unlock(&fc2_mtx);
}

int fc2_rx(const int16_t *amp, int len)
{
    int r = 0;

    pthread_mutex_lock(&fc2_mtx);
    if (fax)
        r = fax_rx(fax, (int16_t *) amp, len);
    pthread_mutex_unlock(&fc2_mtx);
    return r;
}

int fc2_tx(int16_t *amp, int len)
{
    int n = 0;

    pthread_mutex_lock(&fc2_mtx);
    if (fax)
        n = fax_tx(fax, amp, len);
    pthread_mutex_unlock(&fc2_mtx);
    if (n < 0)
        n = 0;
    if (n < len)
        memset(amp + n, 0, sizeof(int16_t) * (size_t) (len - n));
    return len;
}

/*
 * Deferred work, run on the DTE thread: emit whatever the media thread queued,
 * then finish a +FDR that was waiting for a page and report the end of the
 * session.  Doing the page encode here rather than in the phase D handler
 * keeps a page's worth of T.4 work off the media thread.
 */
void fc2_poll(void)
{
    char pending[FC2_Q_SIZE];
    char params[64];
    int pending_len;
    int have_page = 0;
    int page = 0;
    int finish_session = 0;
    int code = 0;

    pthread_mutex_lock(&fc2_mtx);
    if (!selected) {
        pthread_mutex_unlock(&fc2_mtx);
        return;
    }
    pending_len = report_len;
    if (pending_len) {
        memcpy(pending, report_q, (size_t) pending_len);
        report_len = 0;
    }

    if (fdr_pending && rx_pages_done > rx_pages_given) {
        have_page = 1;
        page = rx_pages_given;
        rx_pages_given++;
        fdr_pending = 0;
        params_to_string(&p_cs, params, sizeof(params));
    }

    /*
     * The end of the session is reported only once the DTE has taken every
     * page: T.32 8.5.2.7's +FHS ends the +FDR, and a page still waiting here
     * would be lost.
     */
    if (session_done && hangup_code >= 0 && !fdr_pending && !have_page
        && rx_pages_done == rx_pages_given) {
        finish_session = 1;
        code = hangup_code;
        session_done = 0;
    }
    pthread_mutex_unlock(&fc2_mtx);

    if (pending_len)
        emit_raw(pending, pending_len);

    if (have_page) {
        /* T.32 8.3.4: the session results, then CONNECT, then the page. */
        put_line("\r\n+FCS:%s\r\n", params);
        put_line("\r\nCONNECT\r\n");
        if (send_page_to_dte(page)) {
            put_line("\r\n+FPS:%d\r\n", page_status);
            put_ok();
        } else {
            put_error();
        }
    }

    if (finish_session) {
        put_line("\r\n+FHS:%02X\r\n", code);
        put_ok();
    }
}
