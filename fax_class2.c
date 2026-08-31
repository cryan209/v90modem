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
#include <spandsp/private/t30_dis_dtc_dcs_bits.h>

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

/* T.30's HDLC address and control fields, which SpanDSP keeps private. */
#define T30_ADDRESS_FIELD               0xFF
#define T30_CONTROL_NON_FINAL_FRAME     0x03

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
static int              p_lp;              /* +FLP, a document is pollable */
static int              p_sp;              /* +FSP, we want to poll */
static int              p_ap[3];           /* +FAP: sub-address, SEP, PWD reporting */
static char             sub_address[21];   /* +FSA */
static char             sel_poll_address[21]; /* +FPA */
static char             password[21];      /* +FPW */

/*
 * T.32 8.5.1.6 +FNS: the FIF of the non-standard frame this DCE sends.  Which
 * frame carries it is decided by T.30 rather than by the DTE -- NSF goes with
 * a DIS, NSS with a DCS, NSC with a DTC -- so the one string is offered for
 * all three and whichever frame gets sent takes it.
 */
#define FNS_MAX_OCTETS 90
static uint8_t          nsf_fif[FNS_MAX_OCTETS];
static int              nsf_fif_len;
static int              p_bo;              /* +FBO, phase C bit order */
static int              p_bu;              /* +FBU, HDLC frame reporting */
static int              p_cq, p_ie, p_ct, p_ms, p_ea, p_ffc, p_aa, p_ry;

/* T.32 8.5.1.11 +FNR: which negotiation messages get reported to the DTE. */
static struct {
    int rpr;    /* receive message reporting  -- the remote's DIS/DTC */
    int tpr;    /* transmit message reporting -- the DCS we send */
    int idr;    /* ID reporting               -- +FCI/+FTI */
    int nsr;    /* non-standard frame reporting -- +FNF */
} p_nr;

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
static int              remote_pollable;   /* the remote's DIS had bit 9 set */
static int              dtc_seen;          /* a DTC arrived: we are being polled */
static int              dtc_refused;       /* ... and +FLP was 0 */
static uint8_t          last_ident_fcf;    /* CSI, CIG or TSI */

static int              session_done;
static int              hangup_code = -1;  /* +FHS, -1 = not reported yet */
static int              page_status = 1;   /* +FPS */
static int              pending_fet;       /* +FET, the remote's post page message */
static int              have_fet;

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
 * T.32 8.5.2.7 and Table 20/T.32.  T.30's completion codes are finer grained
 * than T.32's status, and the table is organised by phase and direction --
 * 40-4F is transmit phase C and 90-9F is receive phase C, so a code is only
 * right alongside the direction it happened in.  Codes with no counterpart
 * fall back on the "unspecified" code for their phase and direction rather
 * than on an invented value.
 */
static int t30_status_to_fhs(int status, int transmitting)
{
    switch (status) {
    case T30_ERR_OK:            return 0x00;   /* normal end of connection */
    case T30_ERR_CALLDROPPED:   return 0x03;
    case T30_ERR_CEDTONE:       return 0x05;   /* answer without CED */
    case T30_ERR_T0_EXPIRED:    return 0x04;   /* ringback, no answer */
    case T30_ERR_T1_EXPIRED:    return 0x11;   /* no answer, T.30 T1 */

    /* 20-3F, transmit phase B */
    case T30_ERR_INCOMPATIBLE:  return 0x21;   /* remote cannot receive/send */
    case T30_ERR_RX_INCAPABLE:  return 0x21;
    case T30_ERR_TX_INCAPABLE:  return 0x21;
    case T30_ERR_NORESSUPPORT:  return 0x21;
    case T30_ERR_NOSIZESUPPORT: return 0x21;
    case T30_ERR_NOPOLL:        return 0x23;   /* COMREC invalid command */
    case T30_ERR_TX_INVALRSP:   return 0x28;   /* RSPREC invalid response */
    case T30_ERR_TX_NODIS:      return 0x26;   /* DIS/DTC 3 times, no DCS */
    case T30_ERR_TX_GOTDCN:     return 0x22;   /* COMREC error, tx phase B */
    case T30_ERR_TX_PHBDEAD:    return 0x20;
    case T30_ERR_CANNOT_TRAIN:  return transmitting ? 0x27 : 0x70;
    case T30_ERR_TX_BADDCS:     return 0x24;   /* RSPREC error */

    /* 40-4F, transmit phase C */
    case T30_ERR_TX_BADPG:      return 0x41;   /* unspecified image format */
    case T30_ERR_BADPAGE:       return 0x41;
    case T30_ERR_BADTIFF:       return 0x42;   /* image conversion error */
    case T30_ERR_BADTIFFHDR:    return 0x42;
    case T30_ERR_NOPAGE:        return 0x42;
    case T30_ERR_FILEERROR:     return 0x42;

    /* 50-6F, transmit phase D */
    case T30_ERR_TX_PHDDEAD:    return 0x50;
    case T30_ERR_TX_T5EXP:      return 0x50;

    /* 70-8F, receive phase B */
    case T30_ERR_RX_INVALCMD:   return 0x72;   /* COMREC error */
    case T30_ERR_RX_GOTDCS:     return 0x72;
    case T30_ERR_RX_NOFAX:      return 0x72;
    case T30_ERR_RX_T2EXPDCN:
    case T30_ERR_RX_T2EXPD:
    case T30_ERR_RX_T2EXPFAX:
    case T30_ERR_RX_T2EXPMPS:
    case T30_ERR_RX_T2EXPRR:
    case T30_ERR_RX_T2EXP:      return 0x73;   /* T.30 T2, page not received */

    /* 90-9F, receive phase C */
    case T30_ERR_RX_NOEOL:      return 0x91;   /* missing EOL after 5 s */
    case T30_ERR_RX_ECMPHD:     return 0x92;   /* bad CRC or frame, ECM */
    case T30_ERR_RX_NOCARRIER:  return 0x90;

    /* A0-BF, receive phase D */
    case T30_ERR_RX_DCNWHY:
    case T30_ERR_RX_DCNDATA:
    case T30_ERR_RX_DCNFAX:
    case T30_ERR_RX_DCNPHD:
    case T30_ERR_RX_DCNRRD:
    case T30_ERR_RX_DCNNORTN:   return 0xA2;   /* COMREC invalid response */

    default:                    return transmitting ? 0x20 : 0x70;
    }
}

/* T.32 Table 19: the remote's post page message, for the +FET: report. */
static int t30_ppm_to_fet(int result, int *known)
{
    *known = 1;
    switch (result) {
    case T30_MPS:     return 0;
    case T30_EOM:     return 1;
    case T30_EOP:     return 2;
    case T30_PRI_MPS: return 3;
    case T30_PRI_EOM: return 4;
    case T30_PRI_EOP: return 5;
    }
    *known = 0;
    return 0;
}

/* T.32 Table 23: the post page response, for +FPS. */
static int t30_ppr_to_fps(int result, int *known)
{
    *known = 1;
    switch (result) {
    case T30_MCF: return 1;    /* page good */
    case T30_RTN: return 2;    /* page bad, retrain requested */
    case T30_RTP: return 3;    /* page good, retrain requested */
    case T30_PIN: return 4;    /* page bad, interrupt requested */
    case T30_PIP: return 5;    /* page good, interrupt requested */
    }
    *known = 0;
    return 0;
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

static void update_negotiated_params(void);
static int  phase_bd_reversed(void);

/* T.30's control frames put the FIF at msg[3], so bit n of the FIF (numbered
 * from 1, as T.30 Table 2 numbers them) lives here.  This is SpanDSP's own
 * test_ctrl_bit(), which is private to t30.c. */
#define fif_bit(msg, len, bit) \
    (((3 + (((bit) - 1) / 8)) < (len)) \
     && ((msg)[3 + (((bit) - 1) / 8)] & (1 << (((bit) - 1) % 8))))

/*
 * Decode a received DIS or DTC into the T.32 subparameters +FIS reports: what
 * the remote says it can do.  Only the fields T.32 has a subparameter for are
 * read, and the ones this DCE cannot act on (BF, JP) are reported as zero
 * rather than guessed at.
 *
 * The modem-type bits are T.30 Table 2 bits 11-14.  V.17 is only valid
 * combined with V.29 and V.27ter, so the highest rate is read off the
 * highest bit present, which is how T.30 builds them.
 */
static void dis_to_params(const uint8_t *msg, int len, fc2_params_t *p)
{
    memset(p, 0, sizeof(*p));

    p->vr = fif_bit(msg, len, T30_DIS_BIT_200_200_CAPABLE) ? 1 : 0;

    if (fif_bit(msg, len, T30_DIS_BIT_MODEM_TYPE_4))
        p->br = 5;                                      /* V.17, 14400 */
    else if (fif_bit(msg, len, T30_DIS_BIT_MODEM_TYPE_1))
        p->br = 3;                                      /* V.29, 9600 */
    else if (fif_bit(msg, len, T30_DIS_BIT_MODEM_TYPE_2))
        p->br = 1;                                      /* V.27ter, 4800 */
    else
        p->br = 0;                                      /* V.27ter, 2400 */

    if (fif_bit(msg, len, T30_DIS_BIT_215MM_255MM_303MM_WIDTH_CAPABLE))
        p->wd = 2;
    else if (fif_bit(msg, len, T30_DIS_BIT_215MM_255MM_WIDTH_CAPABLE))
        p->wd = 1;

    if (fif_bit(msg, len, T30_DIS_BIT_UNLIMITED_LENGTH_CAPABLE))
        p->ln = 2;
    else if (fif_bit(msg, len, T30_DIS_BIT_A4_B4_LENGTH_CAPABLE))
        p->ln = 1;

    if (fif_bit(msg, len, T30_DIS_BIT_T6_CAPABLE))
        p->df = 3;
    else if (fif_bit(msg, len, T30_DIS_BIT_2D_CAPABLE))
        p->df = 1;

    if (fif_bit(msg, len, T30_DIS_BIT_ECM_CAPABLE)) {
        p->ec = fif_bit(msg, len, T30_DIS_BIT_64_OCTET_ECM_FRAMES_PREFERRED)
                ? 1 : 2;
    }

    /* T.30 Table 2 bits 21-24, in the order T.32's ST subparameter uses. */
    p->st = (fif_bit(msg, len, T30_DIS_BIT_MIN_SCAN_LINE_TIME_CAPABILITY_1) ? 1 : 0)
          | (fif_bit(msg, len, T30_DIS_BIT_MIN_SCAN_LINE_TIME_CAPABILITY_2) ? 2 : 0)
          | (fif_bit(msg, len, T30_DIS_BIT_MIN_SCAN_LINE_TIME_CAPABILITY_3) ? 4 : 0);
}

/*
 * T.32 8.5.1.11.  Every T.30 control frame passes through here, in both
 * directions, which is the only place the negotiation messages exist as
 * messages -- by Phase B they have become state.
 */
static void real_time_frame_handler(void *user_data, bool incoming,
                                    const uint8_t *msg, int len)
{
    char params[64];
    uint8_t fcf;

    (void) user_data;
    if (len < 3)
        return;
    fcf = msg[2];

    /*
     * T.32 8.5.1.10 and 8.6.  Every phase B and phase D frame, in both
     * directions, as hex.  8.6: "The DCE shall delete HDLC Flags and FCS
     * octets" -- which is already the frame SpanDSP hands over -- and "the
     * DCE shall report these frames before the corresponding responses are
     * generated", so this comes before everything else here.
     *
     * "This facility does not apply to ECM Phase C data frames": those reach
     * the same handler, and are the non-final frames carrying FCD or RCP.
     */
    if (p_bu
        && !(len > 2 && msg[0] == T30_ADDRESS_FIELD
             && msg[1] == T30_CONTROL_NON_FINAL_FRAME
             && (fcf == T4_FCD || fcf == T4_RCP))) {
        char hex[3 * 64 + 16];
        int n = 0;

        for (int i = 0; i < len && n < (int) sizeof(hex) - 4; i++) {
            uint8_t octet = phase_bd_reversed() ? bit_reverse8(msg[i]) : msg[i];

            n += snprintf(hex + n, sizeof(hex) - (size_t) n,
                          (i == 0) ? "%02X" : " %02X", octet);
        }
        hex[n] = '\0';
        queue_line("\r\n%s: %s\r\n", incoming ? "+FHR" : "+FHT", hex);
    }

    if (incoming && (fcf == T30_DIS || fcf == T30_DTC)) {
        if (p_nr.rpr) {
            fc2_params_t remote;

            dis_to_params(msg, len, &remote);
            params_to_string(&remote, params, sizeof(params));
            /* T.32 Table 22: a DIS is the remote's receiver parameters and a
             * DTC its transmitter's, which is the +FTC: report. */
            queue_line("\r\n%s:%s\r\n", (fcf == T30_DTC) ? "+FTC" : "+FIS",
                       params);
        }
        /*
         * T.32 8.4.2.2.  Table 2/T.30 bit 9 says the remote has a document to
         * poll.  +FSP=0 inhibits the report, and 8.5.1.8's note makes +FCR=0
         * act as +FSP=0 -- with no receive capability we could not poll it.
         * The report goes after +FIS: and before the final result code, which
         * is where the queue puts it.
         */
        if (fcf == T30_DIS && p_sp && p_cr
            && fif_bit(msg, len, T30_DIS_BIT_READY_TO_TRANSMIT_FAX_DOCUMENT)) {
            remote_pollable = 1;
            queue_line("\r\n+FPO\r\n");
        }
    }

    if (!incoming && (fcf & 0xFE) == (T30_DCS & 0xFE) && p_nr.tpr) {
        /* The rate is already chosen by the time the DCS goes out, so the
         * session parameters here are the ones this frame is setting up. */
        update_negotiated_params();
        params_to_string(&p_cs, params, sizeof(params));
        queue_line("\r\n+FCS:%s\r\n", params);
    }

    /*
     * CSI, CIG and TSI all decode into T.30's one identification field, so
     * which of T.32 8.4.2.3's three reports it is has to be taken from the
     * frame that carried it.
     */
    if (incoming) {
        /*
         * The low bit of an FCF is a real distinction in some frames and a
         * don't-care in others.  CSI/CIG are a pair and are told apart by it;
         * TSI's low bit is set from whether a DIS has been received, so it
         * arrives as 0x42 or 0x43 and has to be matched with it masked off --
         * which is what T.30 itself does.  Comparing TSI exactly missed every
         * one sent by a calling station.
         */
        if (fcf == T30_CSI)
            last_ident_fcf = T30_CSI;
        else if (fcf == T30_CIG)
            last_ident_fcf = T30_CIG;
        else if ((fcf & 0xFE) == (T30_TSI & 0xFE))
            last_ident_fcf = T30_TSI;
        /* T.32 8.5.1.7: with no document offered for polling, a DTC is a
         * command we cannot honour -- Table 20's "COMREC invalid command". */
        if (fcf == T30_DTC) {
            dtc_seen = 1;
            if (!p_lp)
                dtc_refused = 1;
        }
    }

    /*
     * T.32 8.4.2.4: one response per received non-standard frame, and a
     * different response for each of the three -- +FNF: for NSF, +FNC: for
     * NSC, +FNS: for NSS.  The FIF only ("beginning with the country code,
     * but not including the FCS"), in hex, separated by spaces.
     */
    if (incoming && p_nr.nsr
        && (fcf == T30_NSF || fcf == T30_NSC
            || (fcf & 0xFE) == (T30_NSS & 0xFE))) {
        /* NSF and NSC are a pair told apart by the low bit; NSS carries the
         * same don't-care bit as TSI and DCS do. */
        const char *report = (fcf == T30_NSF) ? "+FNF"
                           : (fcf == T30_NSC) ? "+FNC" : "+FNS";
        char hex[3 * FNS_MAX_OCTETS + 8];
        int n = 0;

        for (int i = 3; i < len && n < (int) sizeof(hex) - 4; i++)
            n += snprintf(hex + n, sizeof(hex) - (size_t) n,
                          (i == 3) ? "%02X" : " %02X", msg[i]);
        hex[n] = '\0';
        queue_line("\r\n%s:%s\r\n", report, hex);
    }
}

static void update_negotiated_params(void)
{  /* forward declared above for the real-time frame handler */
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
    if (ident && ident[0] && p_nr.idr) {
        /*
         * T.32 8.4.2.3: +FCI: for a CSI, +FTI: for a TSI, +FPI: for a CIG.
         * Reported here rather than from the frame itself because T.30 has
         * decoded the identification by now and the frame handler runs
         * before it does; the frame handler recorded which one it was.
         */
        const char *report = "+FCI";

        if (last_ident_fcf == T30_TSI)
            report = "+FTI";
        else if (last_ident_fcf == T30_CIG)
            report = "+FPI";
        queue_line("\r\n%s:\"%s\"\r\n", report, ident);
    }
    /*
     * T.32 Table 22 is explicit that tpr=0 suppresses the +FCS: report while
     * still loading the +FCS parameter, so there is no report here: a
     * transmit session gets it from the DCS we send (the frame handler) and a
     * receive session from the +FDR, per Tables 13 and 16.  Note 1 to Table
     * 22 spells out the consequence the DTE then lives with -- it must send
     * the T.30 mandated format, or enable +FFC conversion.
     */
    (void) params;
    return T30_ERR_OK;
}

/*
 * T.30 Phase D.  T.32 8.4.3 and 8.4.4.1: the receiving DCE reports the page
 * status with its line counts, then the remote's post page message; the
 * transmitting DCE learns from the remote's response whether the page was
 * accepted, which is what 8.3.3.4 makes the +FDT command's OK or ERROR.
 *
 * SpanDSP hands this one handler both sides of that: on a page we received
 * `result` is the remote's post page COMMAND (MPS/EOM/EOP), and on a page we
 * sent it is the remote's post page RESPONSE (MCF/RTN/RTP/PIN/PIP).
 */
static int phase_d_handler(void *user_data, int result)
{
    t30_stats_t t;
    int known;
    int v;

    (void) user_data;

    t30_get_transfer_statistics(fax_get_t30_state(fax), &t);

    v = t30_ppr_to_fps(result, &known);
    if (known) {
        /* A response to a page we sent. */
        page_status = v;
    }

    v = t30_ppm_to_fet(result, &known);
    if (known) {
        /* A command about a page we received.  T.32 8.5.2.3: with copy
         * quality checking off the page status is 1, which is what this DCE
         * reports -- T.30's own error correction is what decides. */
        page_status = 1;
        pending_fet = v;
        have_fet = 1;
    }

    if (t.pages_rx > rx_pages_done && remote_pollable) {
        /* T.32 8.5.1.8: the DCE resets +FSP after a polled document is
         * received. */
        p_sp = 0;
    }
    rx_pages_done = t.pages_rx;
    if (t.pages_tx > tx_pages_sent) {
        if (dtc_seen) {
            /* T.32 8.5.1.7: and resets +FLP after a polled document is
             * sent. */
            p_lp = 0;
        }
        tx_pages_sent = t.pages_tx;
        if (fdt_await_page) {
            /*
             * T.32 8.3.3.4: the +FDT completes when the page has been sent,
             * and it completes with ERROR if the remote rejected the page
             * (RTN or PIN) rather than OK.
             */
            fdt_await_page = 0;
            tx_tiff_ready = 0;
            queue_line("\r\n+FPS:%d\r\n", page_status);
            queue_line("\r\n%s\r\n",
                       (page_status == 2 || page_status == 4) ? "ERROR" : "OK");
        }
    }
    return T30_ERR_OK;
}

/* T.30 Phase E: the call is over (T.32 8.5.2.7 +FHS). */
static void phase_e_handler(void *user_data, int completion_code)
{
    (void) user_data;

    if (dtc_refused && completion_code != T30_ERR_OK) {
        /* T.32 8.5.1.7: a DTC received with no document offered for polling
         * ends the call with this status. */
        hangup_code = 0x23;
    } else {
        hangup_code = t30_status_to_fhs(completion_code,
                                        tx_tiff_ready || tx_pages_sent > 0);
    }
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
    /* T.32 8.5.1.11 +FNR reporting; the handler checks the flags itself, so
     * it is always registered and changing +FNR mid-call takes effect. */
    t30_set_real_time_frame_handler(t30, real_time_frame_handler, NULL);

    t30_set_tx_ident(t30, local_id[0] ? local_id : "");
    t30_set_supported_modems(t30, br_to_modems(p_is.br));
    t30_set_supported_compressions(t30, df_to_supported_compressions(p_is.df));
    t30_set_supported_bilevel_resolutions(t30, vr_to_resolutions(p_is.vr));
    t30_set_ecm_capability(t30, p_is.ec != 0);

    /* T.32 8.5.1.13 and 8.5.1.12: the addressing this session offers. */
    if (sub_address[0])
        t30_set_tx_sub_address(t30, sub_address);
    if (sel_poll_address[0])
        t30_set_tx_selective_polling_address(t30, sel_poll_address);
    if (password[0])
        t30_set_tx_password(t30, password);
    if (polling_id[0])
        t30_set_tx_polled_sub_address(t30, polling_id);

    /* T.32 8.5.1.6.  T.30 picks the frame; all three carry the same FIF. */
    if (nsf_fif_len > 0) {
        t30_set_tx_nsf(t30, nsf_fif, nsf_fif_len);
        t30_set_tx_nsc(t30, nsf_fif, nsf_fif_len);
        t30_set_tx_nss(t30, nsf_fif, nsf_fif_len);
    }

    /*
     * The page the DTE handed us in +FDT, if any.
     *
     * T.32 8.5.1.7: DIS bit 9 -- "I have a document to poll" -- is set from
     * having a document to send, so offering one at all is what +FLP gates.
     * Only the answering side's DIS carries that bit, so a calling DCE hands
     * the page over regardless: there it is an ordinary send, not an offer to
     * be polled.
     */
    if (tx_tiff_ready && (calling_party || p_lp))
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

/*
 * The other half of +FBO: T.30 phase B and phase D control messages, which
 * reach the DTE only in 8.6's +FHT:/+FHR: reports.  8.5.3.4's Table 27 puts
 * that in bit 1 of the value.  The direct order is the octets as T.30 has
 * them -- 8.6's own worked example is "+FHR: FF 13 80 00 4E 78 FE AD", an
 * address, a control field and a DIS whose FCF is 0x80, which is exactly the
 * frame SpanDSP hands over.
 */
static int phase_bd_reversed(void)
{
    return (p_bo & 2) != 0;
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

/*
 * T.32 8.5.1.11 +FNR=<rpr>,<tpr>,<idr>,<nsr>.  Four flags, not one value: the
 * single-parameter form this used to be parsed as answered ERROR to the
 * AT+FNR=1,1,1,1 that fax software actually sends.
 */
static int fnr_param(const char *t)
{
    int *field[4] = { &p_nr.rpr, &p_nr.tpr, &p_nr.idr, &p_nr.nsr };
    int work[4] = { p_nr.rpr, p_nr.tpr, p_nr.idr, p_nr.nsr };
    const char *s;
    int idx = 0;

    if (t[0] == '?' && t[1] == '\0') {
        put_line("\r\n%d,%d,%d,%d\r\n", p_nr.rpr, p_nr.tpr, p_nr.idr, p_nr.nsr);
        put_ok();
        return 1;
    }
    if (t[0] == '=' && t[1] == '?' && t[2] == '\0') {
        put_line("\r\n(0,1),(0,1),(0,1),(0,1)\r\n");
        put_ok();
        return 1;
    }
    if (t[0] != '=')
        return 0;

    for (s = t + 1; *s && idx < 4; ) {
        if (*s == ',') {
            s++;
            idx++;
            continue;
        }
        if (*s != '0' && *s != '1')
            return 0;
        work[idx] = *s - '0';
        s++;
        if (*s == ',') {
            s++;
            idx++;
        } else if (*s != '\0') {
            return 0;
        }
    }
    for (int i = 0; i < 4; i++)
        *field[i] = work[i];
    put_ok();
    return 1;
}

/*
 * T.32 8.5.1.6 +FNS="<hex octets>".  Up to 90 octets; spaces between them are
 * ignored; a repeated +FNS APPENDS to what is there already, and +FNS=""
 * resets it to the null string.  +FNS=? reports the number of octets the
 * parameter can hold.
 *
 * The octet values need no transformation: 8.5.1.6 says each octet is sent
 * LSB first, so "D8A2" is the bit pattern 0001101101000101 on the line, and
 * that is what HDLC does with the octets 0xD8, 0xA2 -- the same convention
 * 8.4.2.4 reports received frames in.
 */
static int fns_param(const char *t)
{
    uint8_t work[FNS_MAX_OCTETS];
    int n = 0;
    const char *v;
    int digits = 0;
    int value = 0;

    if (t[0] == '?' && t[1] == '\0') {
        char hex[3 * FNS_MAX_OCTETS + 8];
        int k = 0;

        for (int i = 0; i < nsf_fif_len && k < (int) sizeof(hex) - 4; i++)
            k += snprintf(hex + k, sizeof(hex) - (size_t) k, "%02X", nsf_fif[i]);
        hex[k] = '\0';
        put_line("\r\n\"%s\"\r\n", hex);
        put_ok();
        return 1;
    }
    if (t[0] == '=' && t[1] == '?' && t[2] == '\0') {
        put_line("\r\n(%d)\r\n", FNS_MAX_OCTETS);
        put_ok();
        return 1;
    }
    if (t[0] != '=')
        return 0;

    v = t + 1;
    if (*v == '"')
        v++;
    for (; *v && *v != '"'; v++) {
        int d;

        if (*v == ' ')
            continue;               /* 8.5.1.6: spaces are ignored */
        if (*v >= '0' && *v <= '9')
            d = *v - '0';
        else if (*v >= 'A' && *v <= 'F')
            d = *v - 'A' + 10;
        else if (*v >= 'a' && *v <= 'f')
            d = *v - 'a' + 10;
        else
            return 0;
        value = (value << 4) | d;
        if (++digits == 2) {
            if (n >= FNS_MAX_OCTETS)
                return 0;
            work[n++] = (uint8_t) value;
            digits = 0;
            value = 0;
        }
    }
    if (digits)
        return 0;                   /* a half octet is not an octet */

    if (n == 0) {
        /* +FNS="" resets the parameter. */
        nsf_fif_len = 0;
        put_ok();
        return 1;
    }
    if (nsf_fif_len + n > FNS_MAX_OCTETS)
        return 0;
    memcpy(nsf_fif + nsf_fif_len, work, (size_t) n);
    nsf_fif_len += n;
    put_ok();
    return 1;
}

/* T.32 8.5.1.12 +FAP=<sub>,<sep>,<pwd>: which addressing frames get reported. */
static int fap_param(const char *t)
{
    int work[3] = { p_ap[0], p_ap[1], p_ap[2] };
    const char *s;
    int idx = 0;

    if (t[0] == '?' && t[1] == '\0') {
        put_line("\r\n%d,%d,%d\r\n", p_ap[0], p_ap[1], p_ap[2]);
        put_ok();
        return 1;
    }
    if (t[0] == '=' && t[1] == '?' && t[2] == '\0') {
        put_line("\r\n(0,1),(0,1),(0,1)\r\n");
        put_ok();
        return 1;
    }
    if (t[0] != '=')
        return 0;

    for (s = t + 1; *s && idx < 3; ) {
        if (*s == ',') {
            s++;
            idx++;
            continue;
        }
        if (*s != '0' && *s != '1')
            return 0;
        work[idx] = *s - '0';
        s++;
        if (*s == ',') {
            s++;
            idx++;
        } else if (*s != '\0') {
            return 0;
        }
    }
    for (int i = 0; i < 3; i++)
        p_ap[i] = work[i];
    put_ok();
    return 1;
}

static void params_reset(void)
{
    p_cc = fc2_caps;
    p_is = fc2_defaults;
    p_cs = fc2_defaults;
    p_cr = 1;
    p_lp = 0;
    p_sp = 0;
    p_ap[0] = p_ap[1] = p_ap[2] = 0;
    sub_address[0] = sel_poll_address[0] = password[0] = '\0';
    nsf_fif_len = 0;
    p_bo = 0;
    p_bu = 0;
    p_cq = 0; p_ie = 0; p_ct = 30; p_ms = 0; p_ea = 0;
    p_ffc = 0; p_aa = 0; p_ry = 3;
    /* T.32 8.5.1.11 default: no negotiation reporting. */
    p_nr.rpr = p_nr.tpr = p_nr.idr = p_nr.nsr = 0;
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
    /* T.32 8.5.1.7/8.5.1.8: polling, offered and requested. */
    else if (match(&t, "+FLP"))  ok = num_param(t, &p_lp, 0, 1, "(0,1)");
    else if (match(&t, "+FSP"))  ok = num_param(t, &p_sp, 0, 1, "(0,1)");
    /* T.32 8.5.1.13: the addressing a polled or polling session carries. */
    else if (match(&t, "+FSA"))  ok = str_param(t, sub_address, sizeof(sub_address));
    else if (match(&t, "+FPA"))  ok = str_param(t, sel_poll_address, sizeof(sel_poll_address));
    else if (match(&t, "+FPW"))  ok = str_param(t, password, sizeof(password));
    else if (match(&t, "+FNS"))  ok = fns_param(t);
    else if (match(&t, "+FAP"))  ok = fap_param(t);
    else if (match(&t, "+FBO"))  ok = num_param(t, &p_bo, 0, 3, "(0-3)");
    else if (match(&t, "+FBU"))  ok = num_param(t, &p_bu, 0, 1, "(0,1)");
    else if (match(&t, "+FCQ"))  ok = num_param(t, &p_cq, 0, 2, "(0-2),(0-2)");
    else if (match(&t, "+FIE"))  ok = num_param(t, &p_ie, 0, 1, "(0,1)");
    else if (match(&t, "+FCT"))  ok = num_param(t, &p_ct, 0, 255, "(0-255)");
    else if (match(&t, "+FMS"))  ok = num_param(t, &p_ms, 0, 5, "(0-5)");
    else if (match(&t, "+FEA"))  ok = num_param(t, &p_ea, 0, 1, "(0,1)");
    else if (match(&t, "+FFC"))  ok = num_param(t, &p_ffc, 0, 3, "(0-3)");
    else if (match(&t, "+FNR"))  ok = fnr_param(t);
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
        /*
         * T.32 8.3.4: "The +FDR command shall result in an ERROR result code
         * if the DCE is on-hook or if the capability to receive is missing or
         * disabled (+FCR=0)."  Otherwise, if a page is already in it goes
         * now; if not the DTE waits here until one is (fc2_poll).
         */
        if (!call_up || !p_cr) {
            ok = 0;
        } else {
            fdr_pending = 1;
            ok = 1;
        }
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

/*
 * T.32 8.6: "the DTE should not attempt to change serial port rate or parity
 * with +FBU set, and DTE commands shall not be echoed."  The reports arrive
 * unsolicited, interleaved with whatever the DTE is typing, so an echo makes
 * the stream ambiguous.  ATE is left alone -- this suppresses the echo only
 * while +FBU is on.
 */
int fc2_echo_suppressed(void)
{
    int r;

    pthread_mutex_lock(&fc2_mtx);
    r = selected && p_bu;
    pthread_mutex_unlock(&fc2_mtx);
    return r;
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
    int page_rows = 0, page_bad_rows = 0, page_bad_run = 0;
    int report_fcs = 0;
    int report_fet = 0;
    int fet = 0;
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
        t30_stats_t t;

        have_page = 1;
        page = rx_pages_given;
        rx_pages_given++;
        fdr_pending = 0;
        params_to_string(&p_cs, params, sizeof(params));
        report_fcs = p_nr.tpr;
        report_fet = have_fet;
        fet = pending_fet;
        have_fet = 0;
        if (fax) {
            t30_get_transfer_statistics(fax_get_t30_state(fax), &t);
            page_rows = t.length;
            page_bad_rows = t.bad_rows;
            page_bad_run = t.longest_bad_row_run;
        }
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
        /* T.32 Table 16: the session parameters (if +FNR's tpr allows), then
         * CONNECT, then the page. */
        if (report_fcs)
            put_line("\r\n+FCS:%s\r\n", params);
        put_line("\r\nCONNECT\r\n");
        if (send_page_to_dte(page)) {
            /*
             * T.32 8.4.3: +FPS:<ppr>,<lc>,<blc>,<cblc>,<lbc> -- the page
             * status and the line counts.  The lost octet count is zero
             * because nothing here drops data on the way to the DTE; it is
             * buffered as a whole page.
             */
            put_line("\r\n+FPS:%d,%d,%d,%d,0\r\n",
                     page_status, page_rows, page_bad_rows, page_bad_run);
            /* T.32 8.4.4.1: what the remote says comes next. */
            if (report_fet)
                put_line("\r\n+FET:%d\r\n", fet);
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
