/*
 * fax_class2_test.c -- a service class 2.0 (T.32) session, both directions.
 *
 * The class 2.0 DCE is driven exactly as a DTE drives it -- AT lines in,
 * DLE-stuffed image data in and out -- and the far end is a plain SpanDSP
 * fax terminal, so nothing here is graded by the code under test.  The page
 * that arrives is compared against the page that was sent, raster by raster.
 *
 * A result-code check alone would not be worth much: the interesting failures
 * (a page that negotiates and transfers as garbage, a stream handed to the
 * DTE in a format it did not ask for) all report OK.
 */

#include "fax_class2.h"

#include <spandsp.h>
#include <tiffio.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define IMAGE_WIDTH  1728
#define IMAGE_ROWS   80

static int failures;

/* ------------------------------------------------------------------ */
/* Test images                                                         */
/* ------------------------------------------------------------------ */

/* One byte per 8 pixels, MSB first, as TIFF and T.4 both want. */
static int row_variant;     /* shifts the pattern, so pages are distinguishable */

static void make_row(uint8_t *row, int y)
{
    y += row_variant * 7;

    int stride = IMAGE_WIDTH / 8;

    memset(row, 0xFF, (size_t) stride);          /* white */
    for (int x = 0; x < IMAGE_WIDTH; x++) {
        /* A pattern that is different on every row, so a page delivered with
         * its rows shifted or duplicated does not compare equal. */
        int black = ((x / 16) + (y / 4)) % 5 == 0 || (x % 64) == (y % 64);

        if (black)
            row[x >> 3] &= (uint8_t) ~(0x80 >> (x & 7));
    }
}

static int write_test_tiff(const char *path)
{
    TIFF *tif;
    uint8_t row[IMAGE_WIDTH / 8];

    if ((tif = TIFFOpen(path, "w")) == NULL)
        return 0;
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, IMAGE_WIDTH);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, IMAGE_ROWS);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 1);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_CCITTFAX4);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISWHITE);
    TIFFSetField(tif, TIFFTAG_FILLORDER, FILLORDER_LSB2MSB);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, -1L);
    TIFFSetField(tif, TIFFTAG_XRESOLUTION, 204.0);
    TIFFSetField(tif, TIFFTAG_YRESOLUTION, 196.0);
    TIFFSetField(tif, TIFFTAG_RESOLUTIONUNIT, RESUNIT_INCH);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);

    for (int y = 0; y < IMAGE_ROWS; y++) {
        make_row(row, y);
        if (TIFFWriteScanline(tif, row, (uint32_t) y, 0) < 0) {
            TIFFClose(tif);
            return 0;
        }
    }
    TIFFClose(tif);
    return 1;
}

/*
 * The same, as a document of several pages -- each with its own row_variant,
 * so "page two arrived" means something.  This is what the far end transmits
 * in the multi-page receive test.
 */
/*
 * Each page of the far end's document is a different length as well as a
 * different pattern.  Both matter: 8.4.3's line counts are latched when the
 * page ends and held until the DTE collects it, and pages of equal length
 * would report the right number even if the latch were never updated.
 */
static int page_rows(int page)
{
    return IMAGE_ROWS - 8 * page;
}

static int write_test_tiff_pages(const char *path, int pages)
{
    TIFF *tif;
    uint8_t row[IMAGE_WIDTH / 8];
    int saved = row_variant;

    if ((tif = TIFFOpen(path, "w")) == NULL)
        return 0;
    for (int page = 0; page < pages; page++) {
        row_variant = page;
        TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, IMAGE_WIDTH);
        TIFFSetField(tif, TIFFTAG_IMAGELENGTH, page_rows(page));
        TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 1);
        TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
        TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_CCITTFAX4);
        TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISWHITE);
        TIFFSetField(tif, TIFFTAG_FILLORDER, FILLORDER_LSB2MSB);
        TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, -1L);
        TIFFSetField(tif, TIFFTAG_XRESOLUTION, 204.0);
        TIFFSetField(tif, TIFFTAG_YRESOLUTION, 196.0);
        TIFFSetField(tif, TIFFTAG_RESOLUTIONUNIT, RESUNIT_INCH);
        TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
        TIFFSetField(tif, TIFFTAG_SUBFILETYPE, FILETYPE_PAGE);
        TIFFSetField(tif, TIFFTAG_PAGENUMBER, (uint16_t) page, (uint16_t) pages);

        for (int y = 0; y < page_rows(page); y++) {
            make_row(row, y);
            if (TIFFWriteScanline(tif, row, (uint32_t) y, 0) < 0) {
                TIFFClose(tif);
                row_variant = saved;
                return 0;
            }
        }
        if (!TIFFWriteDirectory(tif)) {
            TIFFClose(tif);
            row_variant = saved;
            return 0;
        }
    }
    TIFFClose(tif);
    row_variant = saved;
    return 1;
}

/*
 * Read a page back and compare it against what make_row() would have put
 * there.  Returns the number of differing rows, or -1 if the page could not
 * be read at all.
 */
static int compare_page(const char *path, int page, int *rows_out)
{
    TIFF *tif;
    uint32_t w = 0, h = 0;
    uint8_t got[IMAGE_WIDTH / 8];
    uint8_t want[IMAGE_WIDTH / 8];
    int bad = 0;

    if ((tif = TIFFOpen(path, "r")) == NULL)
        return -1;
    for (int i = 0; i < page; i++) {
        if (!TIFFReadDirectory(tif)) {
            TIFFClose(tif);
            return -1;
        }
    }
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);
    if (w != IMAGE_WIDTH) {
        TIFFClose(tif);
        return -1;
    }
    *rows_out = (int) h;
    for (uint32_t y = 0; y < h && y < IMAGE_ROWS; y++) {
        if (TIFFReadScanline(tif, got, y, 0) < 0) {
            TIFFClose(tif);
            return -1;
        }
        make_row(want, (int) y);
        if (memcmp(got, want, sizeof(want)) != 0)
            bad++;
    }
    TIFFClose(tif);
    return bad;
}

/* ------------------------------------------------------------------ */
/* DCE plumbing                                                        */
/* ------------------------------------------------------------------ */

static char dte_buf[1 << 20];
static int  dte_len;
static int  dial_seen;
static int  hangup_seen;

static void dce_write(const uint8_t *buf, int len, void *user_data)
{
    (void) user_data;
    if (dte_len + len < (int) sizeof(dte_buf)) {
        memcpy(dte_buf + dte_len, buf, (size_t) len);
        dte_len += len;
    }
}

static void dce_dial(const char *number, void *u)  { (void) number; (void) u; dial_seen = 1; }
static void dce_answer(void *u)                    { (void) u; dial_seen = 1; }
static void dce_hangup(void *u)                    { (void) u; hangup_seen = 1; }

static void dte_reset(void)   { dte_len = 0; }

/* The captured stream carries page image data, which contains NUL bytes, so
 * this has to search the buffer rather than treat it as a C string -- strstr
 * stops at the first NUL and would miss every report after a +FDR. */
static const char *dte_find(const char *needle)
{
    int n = (int) strlen(needle);

    for (int i = 0; i + n <= dte_len; i++) {
        if (memcmp(dte_buf + i, needle, (size_t) n) == 0)
            return dte_buf + i;
    }
    return NULL;
}

static int dte_saw(const char *s) { return dte_find(s) != NULL; }

static void at(const char *line)
{
    if (!fc2_at_line(line))
        printf("  note: \"%s\" was not handled by the class 2.0 layer\n", line);
}

static void check(int cond, const char *what)
{
    if (cond) {
        printf("  ok   %s\n", what);
    } else {
        printf("  FAIL %s\n", what);
        failures++;
    }
}

/* ------------------------------------------------------------------ */
/* The far end: a plain SpanDSP fax terminal                           */
/* ------------------------------------------------------------------ */

static int peer_done;
static int peer_status = -1;

static void peer_phase_e(void *user_data, int completion_code)
{
    (void) user_data;
    peer_status = completion_code;
    peer_done = 1;
}

/* Capability knobs, so a +FNR report can be checked against a far end that was
 * deliberately configured differently from our own parameters. */
static int          peer_ecm = 1;
static int          peer_t6 = 1;
static const uint8_t *peer_nsf;
static int          peer_nsf_len;
static const uint8_t *peer_nss;
static int          peer_nss_len;
static int          peer_interrupt;   /* the far end asks for / grants one */
static int          peer_ecm_octets = 256; /* the far end's DIS/DTC bit 7 preference */
static int          peer_retransmit;  /* the far end resends a page it is told is bad */
static int          peer_reject_pages; /* how many received pages the far end answers with RTN */
static int          peer_pages_judged;

/*
 * The ECM frame size actually used, taken off the wire at the far end: the
 * DCS's T.30 Table 2 bit 28, and the payload length of the T4_FCD frames that
 * carry the page.  An FCD frame is address, control, FCF, a frame number and
 * then the octets, so 64-octet frames arrive as len - 4 == 64.
 */
static int          peer_saw_dcs_ecm64 = -1;
static int          peer_fcd_min;
static int          peer_fcd_max;
static int          peer_fcd_count;

/* The post page message and response the far end put on the wire. */
static int          peer_saw_ppm;
static int          peer_saw_ppr;
static int          peer_saw_dcn;

/*
 * What the far end saw arrive, captured as the frames arrive.  T.30 frees its
 * received-frame store in release_resources() immediately after the phase E
 * handler, so t30_get_rx_nss() after a session is always empty -- an
 * observation about SpanDSP's lifetimes, not about the frame.
 */
static uint8_t      peer_saw_nss[128];
static int          peer_saw_nss_len;

/*
 * The T.30 control frames the far end sent and received, in order, as a
 * single line.  A rejected page is a long sequence with several plausible
 * ways to go wrong, and "the far end holds the wrong pages" says nothing
 * about which; the trace is printed only when a check in that test failed.
 */
static char peer_trace[4096];
static int  peer_trace_len;
static int  peer_trace_on;

static void peer_frame_handler(void *user_data, bool incoming,
                               const uint8_t *msg, int len)
{
    (void) user_data;
    if (len < 3)
        return;
    if (peer_trace_on && msg[2] != T4_FCD && msg[2] != T4_RCP
        && peer_trace_len < (int) sizeof(peer_trace) - 40) {
        peer_trace_len += snprintf(peer_trace + peer_trace_len,
                                   sizeof(peer_trace) - (size_t) peer_trace_len,
                                   "%s%s ", incoming ? "<" : ">",
                                   t30_frametype(msg[2]));
    }
    /*
     * send_simple_frame() ORs the "a DIS was received" bit into the FCF, so a
     * post page message from a calling station is 0x2F where the constant is
     * 0x2E.  Mask it off, exactly as T.30's own dispatcher does -- comparing
     * these exactly captures nothing at all from one side of the call.
     */
    if ((msg[2] & 0xFE) == (T30_DCN & 0xFE))
        peer_saw_dcn = 1;
    /* The DCS's low bit is a don't-care, as everywhere else. */
    if ((msg[2] & 0xFE) == (T30_DCS & 0xFE) && len > 3 + 3) {
        peer_saw_dcs_ecm64 = (msg[3 + 3] & (1 << 3)) ? 1 : 0;   /* bit 28 */
    }
    if (msg[2] == T4_FCD && len >= 4) {
        int octets = len - 4;

        if (peer_fcd_count == 0 || octets < peer_fcd_min)
            peer_fcd_min = octets;
        if (octets > peer_fcd_max)
            peer_fcd_max = octets;
        peer_fcd_count++;
    }
    switch (msg[2] & 0xFE) {
    case (T30_MPS & 0xFE): case (T30_EOM & 0xFE): case (T30_EOP & 0xFE):
    case (T30_PRI_MPS & 0xFE): case (T30_PRI_EOM & 0xFE): case (T30_PRI_EOP & 0xFE):
        peer_saw_ppm = msg[2] & 0xFE;
        break;
    case (T30_MCF & 0xFE): case (T30_RTN & 0xFE): case (T30_RTP & 0xFE):
    case (T30_PIN & 0xFE): case (T30_PIP & 0xFE):
        peer_saw_ppr = msg[2] & 0xFE;
        break;
    }
    if (!incoming)
        return;
    /* NSS carries the same don't-care low bit as TSI and DCS do. */
    if ((msg[2] & 0xFE) == (T30_NSS & 0xFE)
        && len - 3 <= (int) sizeof(peer_saw_nss)) {
        memcpy(peer_saw_nss, msg + 3, (size_t) (len - 3));
        peer_saw_nss_len = len - 3;
    }
}

static fax_state_t *peer_start(int calling, const char *tx_file, const char *rx_file)
{
    fax_state_t *f = fax_init(NULL, calling ? true : false);
    t30_state_t *t30;

    if (!f)
        return NULL;
    fax_set_transmit_on_idle(f, true);
    t30 = fax_get_t30_state(f);
    t30_set_tx_ident(t30, "peer");
    t30_set_ecm_capability(t30, peer_ecm ? true : false);
    t30_set_ecm_frame_size(t30, peer_ecm_octets);
    /* T.30 5.3.6.3: a transmitter that gets RTN retrains and may send the
     * page again.  SpanDSP only does so when it is told it can; without this
     * it moves on to the next page instead, and the retransmission this test
     * is about never happens. */
    t30_set_retransmit_capable(t30, peer_retransmit ? true : false);
    /* The far end refuses the first peer_reject_pages pages it receives.  It
     * holds its own post page response and releases it from pump(), which is
     * the same mechanism the DCE under test uses for its DTE. */
    t30_set_post_page_response_hold(t30, peer_reject_pages > 0);
    peer_pages_judged = 0;
    if (peer_nsf)
        t30_set_tx_nsf(t30, peer_nsf, peer_nsf_len);
    if (peer_nss)
        t30_set_tx_nss(t30, peer_nss, peer_nss_len);
    t30_set_supported_compressions(t30, T4_COMPRESSION_T4_1D
                                        | T4_COMPRESSION_T4_2D
                                        | (peer_t6 ? T4_COMPRESSION_T6 : 0));
    t30_set_supported_output_compressions(t30, T4_COMPRESSION_T4_1D
                                               | T4_COMPRESSION_T4_2D
                                               | T4_COMPRESSION_T6);
    t30_set_phase_e_handler(t30, peer_phase_e, NULL);
    t30_set_real_time_frame_handler(t30, peer_frame_handler, NULL);
    peer_saw_nss_len = 0;
    peer_saw_ppm = 0;
    peer_saw_ppr = 0;
    peer_saw_dcn = 0;
    peer_saw_dcs_ecm64 = -1;
    peer_fcd_min = 0;
    peer_fcd_max = 0;
    peer_fcd_count = 0;
    t30_remote_interrupts_allowed(t30, true);
    if (peer_interrupt)
        t30_local_interrupt_request(t30, true);
    if (tx_file)
        t30_set_tx_file(t30, tx_file, -1, -1);
    if (rx_file)
        t30_set_rx_file(t30, rx_file, -1);
    peer_done = 0;
    peer_status = -1;
    return f;
}

/*
 * T.32 8.3.4.3: the DCE holds the post page response until the DTE releases it
 * with another +FDR.  A receiving session therefore needs one more +FDR than
 * it has pages, and the last one is what the +FHS: completes.
 */
static void release_post_page(fax_state_t *peer);

/* One 20 ms frame each way, then whatever the DCE has queued for the DTE. */
static void pump(fax_state_t *peer, int frames)
{
    for (int i = 0; i < frames; i++) {
        int16_t a[160];
        int16_t b[160];

        memset(a, 0, sizeof(a));
        fc2_tx(a, 160);
        memset(b, 0, sizeof(b));
        fax_tx(peer, b, 160);

        fax_rx(peer, a, 160);
        fc2_rx(b, 160);

        if (peer_reject_pages > 0) {
            int ppr = (peer_pages_judged < peer_reject_pages) ? T30_RTN : T30_MCF;

            if (t30_release_post_page_response(fax_get_t30_state(peer), ppr) == 0)
                peer_pages_judged++;
        }

        fc2_poll();
        if (peer_done)
            return;
    }
}

/*
 * Pump until the DCE has said something in particular, then stop.  A held post
 * page response has to be released promptly -- T.30's own timers do not wait
 * for a DTE, and the far end gives up after a few repeats of its post page
 * message -- so a receive test cannot simply pump for the whole call and then
 * release.
 */
static void pump_until(fax_state_t *peer, int frames, const char *needle)
{
    for (int i = 0; i < frames; i++) {
        pump(peer, 1);
        fc2_poll();
        if (dte_find(needle))
            return;
        if (peer_done)
            return;
    }
}

/*
 * pump() stops as soon as the far end has finished, which is not what a test
 * of our own shutdown wants: the DCE still needs audio to run its own phase E
 * out after the DCN has gone.
 */
static void pump_regardless(fax_state_t *peer, int frames)
{
    for (int i = 0; i < frames; i++)
        pump(peer, 1);
    for (int i = 0; i < 20; i++)
        fc2_poll();
}

static void release_post_page(fax_state_t *peer)
{
    at("AT+FDR");
    pump(peer, 30 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();
}

/* ------------------------------------------------------------------ */
/* Encoding a page for the DTE side of +FDT                            */
/* ------------------------------------------------------------------ */

/* Turn a TIFF into the compressed stream a class 2.0 DTE hands to +FDT, DLE
 * stuffed and terminated by <DLE><ETX> (T.32 3.2, 8.3.3). */
static uint8_t reverse8(uint8_t v)
{
    v = (uint8_t) (((v & 0xF0) >> 4) | ((v & 0x0F) << 4));
    v = (uint8_t) (((v & 0xCC) >> 2) | ((v & 0x33) << 2));
    v = (uint8_t) (((v & 0xAA) >> 1) | ((v & 0x55) << 1));
    return v;
}

static int encode_page_for_dte(const char *path, int compression, int reversed,
                               uint8_t *out, int max)
{
    t4_tx_state_t *tx;
    uint8_t buf[512];
    int n = 0;
    int len;

    if ((tx = t4_tx_init(NULL, path, -1, -1)) == NULL)
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
        for (int i = 0; i < len; i++) {
            uint8_t byte = reversed ? reverse8(buf[i]) : buf[i];

            if (n + 2 >= max)
                break;
            /* T.32 3.2's stuffing applies to the octets on the DTE link, so
             * it sees the octet after any +FBO reversal. */
            if (byte == 0x10)
                out[n++] = 0x10;
            out[n++] = byte;
        }
    }
    t4_tx_end_page(tx);
    t4_tx_free(tx);

    if (n + 2 < max) {
        out[n++] = 0x10;
        out[n++] = 0x03;
    }
    return n;
}

/* ------------------------------------------------------------------ */
/* The two sessions                                                    */
/* ------------------------------------------------------------------ */

/* The unstuffed phase C octets the DCE handed the DTE, per +FBO setting. */
static uint8_t cap_buf[2][1 << 18];
static int     cap_len[2];

static const char *SRC_TIFF  = "/tmp/fc2_test_src.tif";
static const char *PEER_RX   = "/tmp/fc2_test_peer_rx.tif";
static const char *PEER_TX   = "/tmp/fc2_test_peer_tx.tif";
static const char *DTE_RX    = "/tmp/fc2_test_dte_rx.tif";
static const char *MULTI_TIFF = "/tmp/fc2_test_multi.tif";
/* The far end's own multi-page document, kept separate from the one the DTE
 * feeds through +FDT so neither test can be fed the other's file. */
static const char *PEER_MULTI = "/tmp/fc2_test_peer_multi.tif";

static void test_parameters(void)
{
    printf("T.32 parameter commands:\n");
    dte_reset();
    at("AT+FCC?");
    check(dte_saw("1,5,0,2,3,2,0,7,0"), "AT+FCC? reports the DCE capabilities");

    dte_reset();
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    check(dte_saw("OK"), "AT+FIS= accepts a subparameter list");
    dte_reset();
    at("AT+FIS?");
    check(dte_saw("1,3,0,2,0,0,0,0,0"), "AT+FIS? reads it back");

    dte_reset();
    at("AT+FIS=,5");
    check(dte_saw("OK"), "AT+FIS= with an omitted field");
    dte_reset();
    at("AT+FIS?");
    check(dte_saw("1,5,0,2,0,0,0,0,0"), "an omitted field keeps its value");

    dte_reset();
    at("AT+FLI=\"12345678\"");
    at("AT+FLI?");
    check(dte_saw("\"12345678\""), "AT+FLI sets and reads the local ID");

    dte_reset();
    at("AT+FCLASS=?");
    check(!dte_saw("OK"), "AT+FCLASS is left to the T.31 interpreter");

    dte_reset();
    at("AT+FCR=1");
    check(dte_saw("OK"), "AT+FCR=1");
    dte_reset();
    at("AT+FCR=2");
    check(dte_saw("ERROR"), "AT+FCR=2 is out of range");

    /* T.32 8.3.4: +FDR is an ERROR on-hook, whatever +FCR says. */
    dte_reset();
    at("AT+FCR=1");
    at("AT+FDR");
    check(dte_saw("ERROR"), "AT+FDR on hook is refused");

    dte_reset();
    at("AT+FBS?");
    check(dte_saw("0,0"), "AT+FBS? is read only");
    dte_reset();
    at("AT+FBS=1");
    check(dte_saw("ERROR"), "AT+FBS= is refused");
}

/* Class 2.0 sends a page (T.32 8.3.3) to a plain fax terminal. */
static void test_transmit(int fbo, int ec)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    char cmd[40];
    int rows = 0;
    int bad;
    int n;

    printf("+FDT: class 2.0 sends a page (+FBO=%d, EC=%d)\n", fbo, ec);

    unlink(PEER_RX);
    fc2_select(0);
    fc2_select(1);

    dte_reset();
    at("AT+FLI=\"sender\"");
    /* T.32 Table 22: +FCS: is reported only with +FNR's tpr set. */
    at("AT+FNR=0,1,0,0");
    snprintf(cmd, sizeof(cmd), "AT+FBO=%d", fbo); at(cmd);
    /* MH, fine resolution.  EC=0 is the plainest thing a fax machine sends;
     * EC=2 is T.30 Annex A error correction with 256-octet frames, which is
     * what every modern peer negotiates. */
    snprintf(cmd, sizeof(cmd), "AT+FIS=1,3,0,2,0,%d,0,0,0", ec); at(cmd);
    at("ATD5551234");
    check(dial_seen, "ATD reached the engine");

    dte_reset();
    at("AT+FDT");
    check(dte_saw("CONNECT"), "AT+FDT answers CONNECT");

    n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, fbo & 1,
                            page, sizeof(page));
    check(n > 0, "the test page encodes to a class 2.0 data stream");

    dte_reset();
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : n - off;
        fc2_dte_bytes(page + off, chunk);
    }
    /* T.32 8.3.3.4: a +FDT completes with OK or ERROR; 8.4.3's +FPS: report
     * belongs to +FDR. */
    check(dte_saw("OK") && !dte_saw("+FPS:"), "the spooled page is acknowledged");

    peer = peer_start(0, NULL, PEER_RX);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    pump(peer, 60 * 50);                 /* up to 60 s of call */

    check(peer_done, "the far end reached T.30 phase E");
    check(peer_status == T30_ERR_OK, "the far end reports a good session");
    check(dte_saw("+FCS:"), "the DTE was told the negotiated session parameters");
    {
        /* T.32 8.5.1.3: the EC subparameter of what was actually negotiated.
         * The far end is ECM-capable in both cases, so this separates
         * reporting the session from echoing the offer back. */
        char want[40];

        snprintf(want, sizeof(want), "+FCS:1,3,0,2,0,%d,", ec);
        check(dte_saw(want), "+FCS: reports the negotiated error correction");
    }
    if (ec) {
        /*
         * T.30 A.3.1: the transmitting terminal chooses the frame size, and
         * Table 2 bit 28 of the DCS is where it says which.  Checked against
         * the octets the far end actually received, because a DCS that says
         * 64 and then sends 256-octet frames would pass a bit check and fail
         * against a real peer -- and a receiver that ignored bit 28 would
         * still reassemble the page here, since the frames carry their own
         * lengths.
         */
        int want64 = (ec == 1);

        check(peer_saw_dcs_ecm64 == want64,
              "the DCS says which ECM frame size is in use");
        check(peer_fcd_count > 1 && peer_fcd_max == (want64 ? 64 : 256),
              "the page goes out in frames of that size");
        if (peer_fcd_max != (want64 ? 64 : 256))
            printf("       (%d FCD frames, %d..%d octets, DCS bit 28 = %d)\n",
                   peer_fcd_count, peer_fcd_min, peer_fcd_max,
                   peer_saw_dcs_ecm64);
    }

    bad = compare_page(PEER_RX, 0, &rows);
    check(bad == 0 && rows == IMAGE_ROWS,
          "the page the far end received is the page that was sent");
    if (bad != 0 || rows != IMAGE_ROWS)
        printf("       (%d rows, %d differing)\n", rows, bad);

    fax_free(peer);
    fc2_on_disconnected();
}

/* Class 2.0 receives a page (T.32 8.3.4) from a plain fax terminal. */
static void test_receive(int fbo, int ec)
{
    fax_state_t *peer;
    t4_rx_state_t *decode;
    const char *body;
    int rows = 0;
    int bad;
    int saw_dle = 0;
    int start;

    char cmd2[40];

    printf("+FDR: class 2.0 receives a page (+FBO=%d, EC=%d)\n", fbo, ec);

    unlink(DTE_RX);
    /* test_bit_order_is_real() compares the two captures against each other,
     * so each session replaces its own rather than appending to it -- this
     * now runs more than once per +FBO setting. */
    cap_len[fbo & 1] = 0;
    fc2_select(0);
    fc2_select(1);

    dte_reset();
    at("AT+FLI=\"receiver\"");
    at("AT+FNR=0,1,0,0");
    { char cmd[32]; snprintf(cmd, sizeof(cmd), "AT+FBO=%d", fbo); at(cmd); }
    at("AT+FCR=1");
    snprintf(cmd2, sizeof(cmd2), "AT+FIS=1,3,0,2,0,%d,0,0,0", ec); at(cmd2);
    at("ATA");

    peer = peer_start(1, PEER_TX, NULL);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    at("AT+FDR");
    pump_until(peer, 60 * 50, "+FPS:");
    /* T.32 8.3.4.3: release the post page response so the session can end. */
    release_post_page(peer);

    check(peer_done, "the far end reached T.30 phase E");
    check(peer_status == T30_ERR_OK, "the far end reports a good session");

    check(dte_saw("+FCS:"), "+FDR reported the session parameters");
    check(dte_saw("CONNECT"), "+FDR answered CONNECT");
    /* T.32 8.4.3: +FPS:<ppr>,<lc>,<blc>,<cblc>,<lbc> -- status and counts. */
    {
        char want[32];

        snprintf(want, sizeof(want), "+FPS:1,%d,0,0,0", IMAGE_ROWS);
        check(dte_saw(want), "+FDR reported the page status with line counts");
        if (!dte_saw(want)) {
            const char *p = dte_find("+FPS:");

            if (p)
                printf("       (reported %.24s)\n", p);
        }
        /* T.32 8.5.1.3 EC: 0 off, 1 ECM with 64-octet frames, 2 with 256.
         * The far end is ECM-capable either way, so this separates reporting
         * what the DCS settled from echoing our own +FIS offer back.  Only
         * the EC field is asserted: the rest is the far end's page, whose
         * resolution is its own business. */
        const char *p = dte_find("+FCS:");
        int vr, br, wd, ln, df, got = -1;

        if (p)
            sscanf(p, "+FCS:%d,%d,%d,%d,%d,%d", &vr, &br, &wd, &ln, &df, &got);
        check(got == ec, "+FCS: reports the negotiated error correction");
        if (got != ec)
            printf("       (EC=%d, wanted %d)\n", got, ec);
    }
    if (ec) {
        /*
         * The other direction of T.30 A.3.1: here this DCE is the receiver,
         * so all it can do is ask, with Table 2 bit 7 of its DIS -- which is
         * what +FIS's EC=1 has to reach.  The far end honours it (note 42
         * allows it not to), so the frames on the wire are the proof that the
         * request left this end at all.
         */
        int want64 = (ec == 1);

        check(peer_saw_dcs_ecm64 == want64,
              "the far end's DCS says which ECM frame size is in use");
        check(peer_fcd_count > 1 && peer_fcd_max == (want64 ? 64 : 256),
              "the page arrives in frames of that size");
        if (peer_fcd_max != (want64 ? 64 : 256))
            printf("       (%d FCD frames, %d..%d octets, DCS bit 28 = %d)\n",
                   peer_fcd_count, peer_fcd_min, peer_fcd_max,
                   peer_saw_dcs_ecm64);
    }
    /* T.32 8.4.4.1 Table 19: 2 is EOP, no more pages or documents. */
    check(dte_saw("+FET:2"), "+FDR reported the remote's post page message");
    check(dte_saw("+FHS:"), "the end of the session was reported");

    /*
     * Decode the stream the DTE was given, back through the T.4 decoder, and
     * compare it against the page the far end sent.
     */
    body = dte_find("CONNECT\r\n");
    check(body != NULL, "the data stream follows CONNECT");
    if (!body) {
        fax_free(peer);
        return;
    }
    body += strlen("CONNECT\r\n");
    start = (int) (body - dte_buf);

    decode = t4_rx_init(NULL, DTE_RX, T4_COMPRESSION_T4_1D
                                      | T4_COMPRESSION_T4_2D
                                      | T4_COMPRESSION_T6);
    check(decode != NULL, "the decoder for the DTE stream starts");
    if (decode) {
        t4_rx_set_rx_encoding(decode, T4_COMPRESSION_T4_1D);
        t4_rx_set_image_width(decode, IMAGE_WIDTH);
        t4_rx_set_x_resolution(decode, T4_X_RESOLUTION_R8);
        t4_rx_set_y_resolution(decode, T4_Y_RESOLUTION_FINE);
        t4_rx_start_page(decode);
        for (int i = start; i < dte_len; i++) {
            uint8_t byte = (uint8_t) dte_buf[i];

            if (saw_dle) {
                saw_dle = 0;
                if (byte == 0x03)
                    break;              /* <DLE><ETX> ends the page */
                if (byte != 0x10)
                    continue;           /* a control sequence, not data */
            } else if (byte == 0x10) {
                saw_dle = 1;
                continue;
            }

            /* Record what came off the link, before undoing +FBO, so the two
             * settings can be compared against each other. */
            if (cap_len[fbo & 1] < (int) sizeof(cap_buf[0]))
                cap_buf[fbo & 1][cap_len[fbo & 1]++] = byte;

            if (fbo & 1)
                byte = reverse8(byte);
            t4_rx_put(decode, &byte, 1);
        }
        t4_rx_end_page(decode);
        t4_rx_free(decode);

        bad = compare_page(DTE_RX, 0, &rows);
        check(bad == 0 && rows == IMAGE_ROWS,
              "the page handed to the DTE is the page the far end sent");
        if (bad != 0 || rows != IMAGE_ROWS)
            printf("       (%d rows, %d differing)\n", rows, bad);
    }

    fax_free(peer);
    fc2_on_disconnected();
}

/* Count the occurrences of a report prefix in what the DTE was sent. */
static int dte_count(const char *needle)
{
    int n = (int) strlen(needle);
    int count = 0;

    for (int i = 0; i + n <= dte_len; i++) {
        if (memcmp(dte_buf + i, needle, (size_t) n) == 0)
            count++;
    }
    return count;
}

/*
 * T.32 8.5.1.11 +FNR.  Run a transmit-shaped session, which is the one where
 * the remote sends the DIS and we send the DCS, so all four reports have
 * something to report.
 */
static void run_fnr_session(const char *fnr)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int n;

    fc2_select(0);
    fc2_select(1);
    unlink(PEER_RX);

    at("AT+FLI=\"sender\"");
    at(fnr);
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATD5551234");
    at("AT+FDT");

    n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0, page, sizeof(page));
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : n - off;
        fc2_dte_bytes(page + off, chunk);
    }

    peer = peer_start(0, NULL, PEER_RX);
    if (!peer)
        return;
    dte_reset();
    fc2_on_connected();
    pump(peer, 60 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();
    fax_free(peer);
    fc2_on_disconnected();
}

static void test_fnr(void)
{
    static const uint8_t nsf[] = { 0xAD, 0x00, 0x01, 0x02 };

    printf("+FNR: negotiation message reporting (T.32 8.5.1.11)\n");

    dte_reset();
    at("AT+FNR=1,1,1,1");
    check(dte_saw("OK"), "AT+FNR=1,1,1,1 is accepted");
    dte_reset();
    at("AT+FNR?");
    check(dte_saw("1,1,1,1"), "AT+FNR? reads the four flags back");
    dte_reset();
    at("AT+FNR=0,1");
    at("AT+FNR?");
    check(dte_saw("0,1,1,1"), "an omitted flag keeps its value");
    dte_reset();
    at("AT+FNR=2,0,0,0");
    check(dte_saw("ERROR"), "a flag outside (0,1) is refused");
    dte_reset();
    at("AT+FNR=?");
    check(dte_saw("(0,1),(0,1),(0,1),(0,1)"), "AT+FNR=? reports the ranges");

    /* Reporting off: only the session result, which is not a +FNR report. */
    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = nsf;
    peer_nsf_len = (int) sizeof(nsf);
    run_fnr_session("AT+FNR=0,0,0,0");
    check(!dte_saw("+FIS:"), "rpr=0: the remote's capabilities are not reported");
    check(!dte_saw("+FNF:"), "nsr=0: non-standard frames are not reported");
    check(!dte_saw("+FCI:") && !dte_saw("+FTI:"),
          "idr=0: the remote's identification is not reported");
    check(dte_count("+FCS:") == 0,
          "tpr=0 suppresses the +FCS: report (Table 22/T.32)");

    /* Everything on, against a far end that supports T.6 and ECM. */
    run_fnr_session("AT+FNR=1,1,1,1");
    check(dte_saw("+FIS:"), "rpr=1: the remote's capabilities are reported");
    /* T.32 8.4.2.4: the FIF in hex, "separated by spaces". */
    check(dte_saw("+FNF:AD 00 01 02"),
          "nsr=1: the NSF frame is reported as spaced hex");
    check(dte_saw("+FCI:\"peer\""), "idr=1: the remote's identification is reported");
    check(dte_count("+FCS:") == 1,
          "tpr=1 reports the session result exactly once");
    {
        const char *fis = dte_find("+FIS:");
        int vr, br, wd, ln, df, ec;

        if (fis && sscanf(fis, "+FIS:%d,%d,%d,%d,%d,%d",
                          &vr, &br, &wd, &ln, &df, &ec) == 6) {
            check(df == 3, "+FIS reports the far end's T.6 capability");
            check(ec != 0, "+FIS reports the far end's ECM capability");
        } else {
            check(0, "+FIS: is a subparameter list");
        }
    }

    /*
     * The same report against a far end configured the other way.  Our own
     * +FIS defaults would give df=3 and ec=2, so this is what separates
     * decoding the DIS from echoing our own parameters back.
     */
    peer_ecm = 0;
    peer_t6 = 0;
    peer_nsf = NULL;
    run_fnr_session("AT+FNR=1,0,0,0");
    {
        const char *fis = dte_find("+FIS:");
        int vr, br, wd, ln, df, ec;

        if (fis && sscanf(fis, "+FIS:%d,%d,%d,%d,%d,%d",
                          &vr, &br, &wd, &ln, &df, &ec) == 6) {
            check(df < 3, "+FIS follows a far end without T.6");
            check(ec == 0, "+FIS follows a far end without ECM");
        } else {
            check(0, "+FIS: is a subparameter list");
        }
    }
    check(!dte_saw("+FNF:"), "no NSF is reported when the far end sends none");
    peer_ecm = 1;
    peer_t6 = 1;
}

/*
 * The round trips above would also pass if +FBO did nothing, because the test
 * reverses on both sides of a setting that was ignored.  This compares the two
 * streams the DCE actually put on the DTE link.
 */
static void test_bit_order_is_real(void)
{
    int mismatched = 0;

    printf("+FBO: the setting reaches the DTE link\n");
    check(cap_len[0] > 0 && cap_len[0] == cap_len[1],
          "both settings delivered a page of the same length");
    if (cap_len[0] == 0 || cap_len[0] != cap_len[1])
        return;

    for (int i = 0; i < cap_len[0]; i++) {
        if (cap_buf[1][i] != reverse8(cap_buf[0][i]))
            mismatched++;
    }
    check(mismatched == 0,
          "+FBO=1 delivers the bit reverse of what +FBO=0 delivers");
    if (mismatched)
        printf("       (%d of %d octets)\n", mismatched, cap_len[0]);

    /* And they are genuinely different octets, not a palindrome. */
    check(memcmp(cap_buf[0], cap_buf[1], (size_t) cap_len[0]) != 0,
          "the two streams differ");
}

/*
 * T.32 8.3.4.3, 8.4.3 and 8.5.2.2: the DCE holds the post page response until
 * the DTE releases it with the next +FDR, so the DTE can read the page status
 * and change it.  Two things have to be true and the first is easy to miss: it
 * must not go out BEFORE the release, and the DTE's value must be what goes
 * out.  A DCE that sent MCF straight away and ignored +FPS would still pass a
 * page comparison and still report OK.
 */
static void test_post_page_hold(void)
{
    fax_state_t *peer;

    printf("post page response held for the DTE (T.32 8.3.4.3, 8.4.3)\n");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;

    fc2_select(0);
    fc2_select(1);
    at("AT+FCR=1");
    at("AT+FNR=0,1,0,0");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATA");

    peer = peer_start(1, PEER_TX, NULL);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    at("AT+FDR");
    pump_until(peer, 60 * 50, "+FPS:");

    check(dte_saw("+FPS:1,"), "the page arrives and is reported good");

    /*
     * There is deliberately no "and nothing has gone out yet" check here.
     * One was written and measured: it reads 0 with the hold removed as well,
     * so it cannot fail and proves nothing.  The assertion below is what
     * carries this -- without the hold an MCF goes out at the end of the page
     * and the DTE's RTN never does, so it fails, which was confirmed by
     * removing the hold and watching it.
     */

    /* T.32 8.5.2.2: the DTE writes a different verdict.  2 is RTN, "page bad,
     * retrain requested" -- the everyday reason to hold the response. */
    at("AT+FPS=2");
    release_post_page(peer);

    check(peer_saw_ppr == (T30_RTN & 0xFE),
          "the DTE's +FPS=2 is what reaches the far end, as RTN");
    if (peer_saw_ppr != (T30_RTN & 0xFE))
        printf("       (far end saw post page response 0x%02X)\n", peer_saw_ppr);

    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * T.32 8.5.2.6 +FCT bounds the wait for the +FDR that releases a held post
 * page response: "when this timeout is reached, the DCE shall send the T.30
 * DCN response to the remote station and execute an implied orderly abort
 * command."  Without it a DTE that stops answering leaves the call hanging on
 * T.30's own timers with nothing reported to the DTE at all.
 */
static void test_held_response_timeout(void)
{
    fax_state_t *peer;

    printf("+FCT: the held response times out (T.32 8.5.2.6)\n");

    fc2_select(0);
    fc2_select(1);
    dte_reset();
    at("AT+FCT?");
    check(dte_saw("30"), "AT+FCT? defaults to 30 seconds (1Eh)");
    dte_reset();
    at("AT+FCT=2");
    check(dte_saw("OK"), "AT+FCT= sets it");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;

    at("AT+FCR=1");
    at("AT+FNR=0,1,0,0");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATA");

    peer = peer_start(1, PEER_TX, NULL);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    at("AT+FDR");
    pump_until(peer, 60 * 50, "+FPS:");
    check(dte_saw("+FPS:1,"), "the page arrives and is reported");

    /* And now the DTE says nothing at all.  Three seconds against a +FCT of
     * two. */
    dte_reset();
    pump_regardless(peer, 6 * 50);

    check(peer_saw_dcn, "the far end is told with a DCN");
    /* T.32 Table 20: 02 is "call aborted, from +FKS or <CAN>", which is what
     * 8.5.2.6's implied orderly abort is. */
    check(dte_saw("+FHS:02"), "the DTE is told with +FHS:02");
    check(dte_saw("OK"), "and the outstanding command completes");

    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * T.32 8.5.2.6 the other way: "For transmission (+FDT), when this timeout is
 * reached, the DCE shall properly terminate any Phase C data transfer in
 * progress, then execute an implied +FKS orderly abort command."  A DTE that
 * opens +FDT, takes its CONNECT and then stops feeding the page leaves the DCE
 * with nothing to send and nothing to wait for; without this the module sits
 * in data mode for the rest of the call, with the DTE's bytes -- including any
 * command it tries -- read as image data.
 */
static void test_transmit_timeout(int feed_part)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int n;

    printf("+FCT: a +FDT that %s times out (T.32 8.5.2.6)\n",
           feed_part ? "stalls part way" : "sends nothing at all");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;

    fc2_select(0);
    fc2_select(1);
    unlink(PEER_RX);

    at("AT+FCT=1");
    at("AT+FBU=1");
    at("AT+FNR=0,1,0,0");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATD5551234");

    peer = peer_start(0, NULL, PEER_RX);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();

    /* Open the page and hand over a third of it, then stop. */
    at("AT+FDT");
    check(dte_saw("CONNECT"), "AT+FDT answers CONNECT");
    if (feed_part) {
        n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0,
                                page, sizeof(page));
        fc2_dte_bytes(page, n / 3);
    }
    check(fc2_in_dte_data(), "the DCE is taking the page");

    dte_reset();
    pump_regardless(peer, 6 * 50);

    check(!fc2_in_dte_data(), "the stalled transfer is terminated");
    /*
     * The DCN is asserted on our own transmit path, through the +FBU frame
     * report, which is generated inside send_frame() -- so it says the frame
     * went out.  Not at the far end: a stalled +FDT means T.30 never had a
     * document, so the call collapses on its own account within the first
     * second and the DCN arrives after the far end has stopped listening.
     * That is the scenario, not the implementation.
     */
    check(dte_saw("+FHT: FF 13 FA"), "a DCN goes out (T30_DCN, 0xFA)");
    check(dte_saw("+FHS:02"), "the DTE is told with +FHS:02");

    /* And the DTE is back in command mode: a plain AT command is answered
     * rather than swallowed as image data. */
    dte_reset();
    at("AT+FCT?");
    check(dte_saw("1"), "and AT commands are understood again");

    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * The other half of the transmit timeout: a DTE that is feeding the page, just
 * slowly.  +FCT bounds the wait for the DTE, so every block it hands over has
 * to restart it -- otherwise a page that takes longer than +FCT to cross a
 * slow DTE link is aborted in the middle of a transfer that is going fine.
 */
static void test_transmit_timeout_not_tripped(void)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int n;

    printf("+FCT: a slow but live +FDT is not aborted (T.32 8.5.2.6)\n");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;

    fc2_select(0);
    fc2_select(1);
    unlink(PEER_RX);

    at("AT+FCT=1");
    at("AT+FNR=0,1,0,0");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATD5551234");

    peer = peer_start(0, NULL, PEER_RX);
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    at("AT+FDT");
    n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0, page, sizeof(page));

    /* Four blocks, 0.6 s of call between them: 2.4 s in total against a +FCT
     * of one second, with no gap longer than the timeout. */
    for (int i = 0; i < 4; i++) {
        int off = (n * i) / 4;
        int end = (n * (i + 1)) / 4;

        /* Keep the terminating <DLE><ETX> out of the last block, so the
         * transfer is still open when the checks run. */
        if (i == 3)
            end -= 2;
        fc2_dte_bytes(page + off, end - off);
        pump_regardless(peer, 30);
    }

    check(fc2_in_dte_data(), "the transfer is still open after 2.4 s");
    check(!dte_saw("+FHS:02"), "and nothing was aborted");

    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * T.32 8.3.3.3 and 8.3.3.7: a document of several pages is one +FDT per page,
 * each ended by a <DLE><ppm> that says whether another follows -- "," for MPS
 * and "." for EOP.  The far end must end up with both pages, in order and
 * distinct from each other; a DCE that sent the first page twice, or only the
 * last one, would still complete the session and still report OK.
 */
static int  send_page_via_fdt(int variant, int ppm)
{
    static uint8_t page[1 << 20];
    int n;

    /*
     * Its own file: SRC_TIFF is the shared one every other test compares
     * against, and writing a variant into it makes those tests compare a page
     * they never sent.
     */
    row_variant = variant;
    if (!write_test_tiff(MULTI_TIFF))
        return 0;
    at("AT+FDT");
    n = encode_page_for_dte(MULTI_TIFF, T4_COMPRESSION_T4_1D, 0, page, sizeof(page));
    if (n < 2)
        return 0;
    n -= 2;                     /* drop the encoder's <DLE><ETX> */
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : n - off;
        fc2_dte_bytes(page + off, chunk);
    }
    {
        uint8_t end[2] = { 0x10, (uint8_t) ppm };
        fc2_dte_bytes(end, 2);
    }
    row_variant = 0;
    return 1;
}

static void test_multipage_transmit(int connect_midway, int ec)
{
    fax_state_t *peer;
    char cmd[40];
    int rows = 0;
    int bad;

    printf("multi-page +FDT, call answered %s (EC=%d) (T.32 8.3.3.3, 8.3.3.7)\n",
           connect_midway ? "between the pages" : "after the document", ec);

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;

    fc2_select(0);
    fc2_select(1);
    unlink(PEER_RX);

    at("AT+FLI=\"sender\"");
    at("AT+FNR=0,1,0,0");
    snprintf(cmd, sizeof(cmd), "AT+FIS=1,3,0,2,0,%d,0,0,0", ec); at(cmd);
    at("ATD5551234");

    /* Page one, ended with MPS: another page of the same format follows. */
    dte_reset();
    check(send_page_via_fdt(0, 0x2C), "page one is handed over");
    check(dte_saw("OK"), "and acknowledged");

    peer = peer_start(0, NULL, PEER_RX);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    /*
     * The interesting order: the call is answered while the DTE still has a
     * page to hand over.  T.30 must not be started on half a document, or it
     * sends the first page and an EOP after it.
     */
    if (connect_midway) {
        dte_reset();
        fc2_on_connected();
        /*
         * Three seconds, not a token frame or two: T.30 started with no
         * document has that long to decide it has nothing to do and give up,
         * which is the thing the deferred start exists to prevent.  A short
         * gap passes either way and proves nothing.
         */
        pump(peer, 10 * 50);
    }

    /* Page two, ended with EOP: that is the document. */
    dte_reset();
    check(send_page_via_fdt(1, 0x2E), "page two is handed over");

    if (!connect_midway) {
        dte_reset();
        fc2_on_connected();
    }
    pump(peer, 90 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();

    check(peer_done && peer_status == T30_ERR_OK,
          "the far end reports a good session");

    /* Both pages, in order, each the one that was sent. */
    row_variant = 0;
    bad = compare_page(PEER_RX, 0, &rows);
    check(bad == 0 && rows == IMAGE_ROWS, "page one arrived intact");
    if (bad != 0 || rows != IMAGE_ROWS)
        printf("       (page 1: %d rows, %d differing)\n", rows, bad);

    row_variant = 1;
    bad = compare_page(PEER_RX, 1, &rows);
    check(bad == 0 && rows == IMAGE_ROWS, "page two arrived intact, and second");
    if (bad != 0 || rows != IMAGE_ROWS)
        printf("       (page 2: %d rows, %d differing)\n", rows, bad);
    row_variant = 0;

    /* And they really are different pages, so "both intact" means something. */
    row_variant = 1;
    bad = compare_page(PEER_RX, 0, &rows);
    check(bad > 0, "the two pages are not the same image");
    row_variant = 0;

    if (ec) {
        /* Both pages go out in error correction mode, at the frame size the
         * DCS names.  T.30 A.1 restarts the ECM block per page, so a size
         * that came from a stale partial page would show here. */
        check(peer_saw_dcs_ecm64 == (ec == 1),
              "the DCS says which ECM frame size is in use");
        check(peer_fcd_count > 2 && peer_fcd_max == ((ec == 1) ? 64 : 256),
              "both pages go out in frames of that size");
        if (peer_fcd_max != ((ec == 1) ? 64 : 256))
            printf("       (%d FCD frames, %d..%d octets, DCS bit 28 = %d)\n",
                   peer_fcd_count, peer_fcd_min, peer_fcd_max,
                   peer_saw_dcs_ecm64);
    }

    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * The other direction: a multi-page document arriving, one +FDR per page.
 * This is where the class 2.0 bookkeeping and T.30's ECM buffering meet.  A
 * receiving DCE hands over one page and then holds the post page response
 * (8.3.4.3) until the next +FDR releases it, so the pages are strictly
 * interleaved with the DTE -- and 8.4.3's line counts are latched per page
 * when the page ends, which is only safe because that hold stops T.30
 * starting the next one.  Two pages is what tells a latch that is reused
 * from a latch that is overwritten.
 *
 * 8.4.4.1 Table 19: <ppm> 1 is MPS, another page follows; 2 is EOP, that is
 * the document.  The last +FDR takes no page and ends the session (Table 17).
 */
static void test_multipage_receive(int ec)
{
    fax_state_t *peer;
    t4_rx_state_t *decode;
    char cmd[40];
    int rows = 0;
    int bad;

    printf("multi-page +FDR: two pages, one +FDR each (EC=%d)\n", ec);

    unlink(DTE_RX);
    if (!write_test_tiff_pages(PEER_MULTI, 2)) {
        check(0, "the far end's two-page document is written");
        return;
    }

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;

    fc2_select(0);
    fc2_select(1);

    dte_reset();
    at("AT+FLI=\"receiver\"");
    at("AT+FNR=0,1,0,0");
    at("AT+FBO=0");
    at("AT+FCR=1");
    snprintf(cmd, sizeof(cmd), "AT+FIS=1,3,0,2,0,%d,0,0,0", ec); at(cmd);
    at("ATA");

    peer = peer_start(1, PEER_MULTI, NULL);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    fc2_on_connected();

    for (int page = 0; page < 2; page++) {
        char want[48];
        const char *body;
        int start;
        int saw_dle = 0;

        dte_reset();
        at("AT+FDR");
        pump_until(peer, 60 * 50, "+FPS:");

        snprintf(want, sizeof(want), "+FPS:1,%d,0,0,0", page_rows(page));
        check(dte_saw(want), page ? "page two reports its own line counts"
                                  : "page one reports its line counts");
        if (!dte_saw(want)) {
            const char *p = dte_find("+FPS:");

            if (p)
                printf("       (reported %.24s)\n", p);
        }

        /* T.32 8.4.4.1 Table 19: <ppm> 0 is MPS, another page of the same
         * document; 2 is EOP, the end of it. */
        check(dte_saw(page ? "+FET:2" : "+FET:0"),
              page ? "the last page is followed by EOP"
                   : "the first page is followed by MPS");
        if (!dte_saw(page ? "+FET:2" : "+FET:0")) {
            const char *p = dte_find("+FET:");

            printf("       (page %d: %.8s)\n", page + 1,
                   p ? p : "no +FET at all");
        }

        /* And the page itself, decoded back off the DTE link. */
        body = dte_find("CONNECT\r\n");
        check(body != NULL, "the page follows CONNECT");
        if (!body)
            break;
        start = (int) (body - dte_buf) + 9;

        row_variant = page;
        decode = t4_rx_init(NULL, DTE_RX, T4_COMPRESSION_T4_2D);
        check(decode != NULL, "the decoder for the DTE stream starts");
        if (!decode)
            break;
        t4_rx_set_rx_encoding(decode, T4_COMPRESSION_T4_1D);
        t4_rx_set_image_width(decode, IMAGE_WIDTH);
        t4_rx_start_page(decode);
        for (int i = start; i < dte_len; i++) {
            uint8_t byte = (uint8_t) dte_buf[i];

            if (saw_dle) {
                saw_dle = 0;
                if (byte == 0x03)
                    break;
                if (byte != 0x10)
                    continue;
            } else if (byte == 0x10) {
                saw_dle = 1;
                continue;
            }
            t4_rx_put(decode, &byte, 1);
        }
        t4_rx_end_page(decode);
        t4_rx_release(decode);
        t4_rx_free(decode);

        bad = compare_page(DTE_RX, 0, &rows);
        check(bad == 0 && rows == page_rows(page),
              page ? "page two is the page the far end sent"
                   : "page one is the page the far end sent");
        if (bad != 0 || rows != page_rows(page))
            printf("       (page %d: %d rows, %d differing)\n",
                   page + 1, rows, bad);

        /* The pages must differ, or "each is the page that was sent" is
         * satisfied by any two identical ones. */
        if (page == 1) {
            row_variant = 0;
            bad = compare_page(DTE_RX, 0, &rows);
            check(bad > 0, "the two pages are not the same image");
        }
        row_variant = 0;
    }

    /* Table 17: the +FDR that releases the last response ends the session. */
    dte_reset();
    release_post_page(peer);
    check(dte_saw("+FHS:"), "the last +FDR ends the session");
    check(peer_done && peer_status == T30_ERR_OK,
          "the far end reports a good session");

    if (ec) {
        check(peer_saw_dcs_ecm64 == (ec == 1),
              "the far end's DCS says which ECM frame size is in use");
        check(peer_fcd_count > 2 && peer_fcd_max == ((ec == 1) ? 64 : 256),
              "both pages arrive in frames of that size");
        if (peer_fcd_max != ((ec == 1) ? 64 : 256))
            printf("       (%d FCD frames, %d..%d octets, DCS bit 28 = %d)\n",
                   peer_fcd_count, peer_fcd_min, peer_fcd_max,
                   peer_saw_dcs_ecm64);
    }

    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * T.32 Annex II.8, "receive one page with line errors and retransmission".
 * The DTE reads the page, does not like it, and writes +FPS=2 before the
 * +FDR that releases the response -- so RTN goes on the wire, the far end
 * retrains and sends the same page again, and that repeat has to reach the
 * DTE as a page of its own.  Mid-document, so what follows the repeat is the
 * rest of the document rather than the end of the call.
 *
 * The far end's document is two pages of different lengths, and page one is
 * the one rejected, so "the page came again" is checked by its content and
 * its line count rather than by a page simply arriving.
 */
static void test_page_rejected_rtn(int ec)
{
    fax_state_t *peer;
    t4_rx_state_t *decode;
    char cmd[40];
    char want[48];
    int expect_page;
    int rows = 0;
    int bad;

    printf("+FPS=2 mid-document%s (EC=%d) (T.32 II.8)\n",
           ec ? ": ECM has no RTN" : ": RTN and retransmit", ec);

    unlink(DTE_RX);
    if (!write_test_tiff_pages(PEER_MULTI, 2)) {
        check(0, "the far end's two-page document is written");
        return;
    }

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;
    peer_retransmit = 1;

    fc2_select(0);
    fc2_select(1);

    dte_reset();
    at("AT+FLI=\"receiver\"");
    at("AT+FNR=0,1,0,0");
    at("AT+FBO=0");
    at("AT+FCR=1");
    snprintf(cmd, sizeof(cmd), "AT+FIS=1,3,0,2,0,%d,0,0,0", ec); at(cmd);
    at("ATA");

    peer = peer_start(1, PEER_MULTI, NULL);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer) {
        peer_retransmit = 0;
        return;
    }

    fc2_on_connected();

    /* Page one, as sent the first time. */
    dte_reset();
    at("AT+FDR");
    pump_until(peer, 60 * 50, "+FPS:");
    snprintf(want, sizeof(want), "+FPS:1,%d,0,0,0", page_rows(0));
    check(dte_saw(want), "page one arrives and is reported");

    /*
     * T.32 8.5.2.2 and Table 23: the DTE overwrites the verdict.  2 is RTN,
     * "page bad, retrain requested" -- the everyday reason the post page
     * response is held for the DTE at all.  The +FDR that releases it is also
     * the one that collects whatever comes next.
     */
    dte_reset();
    at("AT+FPS=2");
    peer_saw_ppr = 0;
    at("AT+FDR");
    pump_until(peer, 90 * 50, "+FPS:");
    for (int i = 0; i < 20; i++)
        fc2_poll();

    if (ec) {
        /*
         * T.30 Table 5, note 2 to RTN: "RTN is not applicable to the optional
         * T.4 error correction mode."  ECM repairs a page frame by frame --
         * the receiver answers a partial page with MCF or PPR and there is no
         * signal for "bad page, retrain" -- so the DTE's +FPS=2 cannot be
         * honoured, MCF goes out and the document carries on to page two.
         * Pinned rather than left implicit: it looks exactly like the held
         * response being ignored, which is a real bug in the non-ECM case,
         * and the difference is worth stating.
         */
        check(peer_saw_ppr == (T30_MCF & 0xFE),
              "in ECM the page cannot be refused, so MCF goes out (T.30 note 2)");
        expect_page = 1;
    } else {
        check(peer_saw_ppr == (T30_RTN & 0xFE),
              "the DTE's +FPS=2 reaches the far end as RTN");
        /* T.32 II.8: RTN forces back to phase B, so the session parameters
         * are negotiated again and reported again. */
        check(dte_saw("+FCS:"), "the retrain is reported to the DTE");
        expect_page = 0;
    }
    if (peer_saw_ppr != (ec ? (T30_MCF & 0xFE) : (T30_RTN & 0xFE)))
        printf("       (far end saw post page response 0x%02X)\n", peer_saw_ppr);

    check(dte_saw("CONNECT"), "the next page follows CONNECT");
    snprintf(want, sizeof(want), "+FPS:1,%d,0,0,0", page_rows(expect_page));
    check(dte_saw(want), ec ? "page two follows, its own line count with it"
                            : "page one arrives again, with its own line count");
    if (!dte_saw(want)) {
        const char *p = dte_find("+FPS:");

        printf("       (reported %.24s)\n", p ? p : "nothing");
    }

    /* And it is the page it should be -- in the non-ECM arm, page one again
     * rather than page two arriving early. */
    {
        const char *body = dte_find("CONNECT\r\n");
        int saw_dle = 0;
        int start;

        check(body != NULL, "the page follows CONNECT");
        if (body) {
            start = (int) (body - dte_buf) + 9;
            row_variant = expect_page;
            decode = t4_rx_init(NULL, DTE_RX, T4_COMPRESSION_T4_2D);
            check(decode != NULL, "the decoder for the DTE stream starts");
            if (decode) {
                t4_rx_set_rx_encoding(decode, T4_COMPRESSION_T4_1D);
                t4_rx_set_image_width(decode, IMAGE_WIDTH);
                t4_rx_start_page(decode);
                for (int i = start; i < dte_len; i++) {
                    uint8_t byte = (uint8_t) dte_buf[i];

                    if (saw_dle) {
                        saw_dle = 0;
                        if (byte == 0x03)
                            break;
                        if (byte != 0x10)
                            continue;
                    } else if (byte == 0x10) {
                        saw_dle = 1;
                        continue;
                    }
                    t4_rx_put(decode, &byte, 1);
                }
                t4_rx_end_page(decode);
                t4_rx_release(decode);
                t4_rx_free(decode);
                bad = compare_page(DTE_RX, 0, &rows);
                check(bad == 0 && rows == page_rows(expect_page),
                      ec ? "and it is page two"
                         : "and it is page one, not the next page");
                if (bad != 0 || rows != page_rows(expect_page))
                    printf("       (%d rows, %d differing)\n", rows, bad);
            }
            row_variant = 0;
        }
    }

    if (!ec) {
        /* Accept it this time, and take the rest of the document. */
        dte_reset();
        at("AT+FPS=1");
        at("AT+FDR");
        pump_until(peer, 90 * 50, "+FPS:");
        for (int i = 0; i < 20; i++)
            fc2_poll();
        snprintf(want, sizeof(want), "+FPS:1,%d,0,0,0", page_rows(1));
        check(dte_saw(want), "page two follows the accepted page");
        if (!dte_saw(want)) {
            const char *p = dte_find("+FPS:");

            printf("       (reported %.24s)\n", p ? p : "nothing");
        }
    }
    check(dte_saw("+FET:2"), "the document ends with EOP");

    dte_reset();
    at("AT+FPS=1");
    release_post_page(peer);
    check(dte_saw("+FHS:"), "the last +FDR ends the session");
    check(peer_done && peer_status == T30_ERR_OK,
          "the far end reports a good session");

    row_variant = 0;
    peer_retransmit = 0;
    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * T.32 Annex II.7, "send one page with line errors and retransmission", the
 * transmit half of the same story: the far end answers a page with RTN, the
 * +FDT completes with ERROR rather than OK (8.3.3.4), +FPS reads back the 2
 * that says why (Table 23), and the DTE hands the page over again.
 */
static void test_transmit_page_rejected(void)
{
    fax_state_t *peer;
    int failures_before = failures;
    int rows = 0;
    int bad;

    printf("+FDT: a page the far end rejects with RTN (T.32 II.7)\n");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;
    peer_reject_pages = 1;

    fc2_select(0);
    fc2_select(1);
    unlink(PEER_RX);

    at("AT+FLI=\"sender\"");
    at("AT+FNR=0,1,0,0");
    /* No ECM: T.30 Table 5 note 2 puts RTN outside it, so a rejected page is
     * only a thing that can happen here. */
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATD5551234");

    /* Two pages: the first ends with MPS, so the rejection lands in the
     * middle of the document rather than at the end of the procedure. */
    dte_reset();
    check(send_page_via_fdt(0, 0x2C), "page one is handed over");
    check(dte_saw("OK"), "and acknowledged");
    check(send_page_via_fdt(1, 0x2E), "page two is handed over");

    peer = peer_start(0, NULL, PEER_RX);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer) {
        peer_reject_pages = 0;
        return;
    }

    dte_reset();
    fc2_on_connected();

    /* Up to the far end's verdict on page one, so +FPS can be read while it
     * still says what the far end thought -- T.32 II.7's [AT+FPS?] step. */
    for (int i = 0; i < 90 * 50 && peer_saw_ppr == 0; i++)
        pump(peer, 1);
    check(peer_saw_ppr == (T30_RTN & 0xFE), "the far end rejects page one with RTN");

    /*
     * T.32 II.7's "[AT+FPS?] -> 2" step.  The far end's frame handler sees
     * RTN as it goes out, so this side still has to receive and demodulate
     * it -- hence the wait rather than a fixed pump.  Table 23: 2 is "page
     * bad, retrain requested".  Nothing else covers that mapping in the
     * transmit direction, and page_status returns to 1 as soon as a page is
     * accepted, so it can only be read here.
     */
    {
        int saw_two = 0;

        for (int i = 0; i < 200 && !saw_two; i++) {
            dte_reset();
            at("AT+FPS?");
            for (int k = 0; k < 5; k++)
                fc2_poll();
            saw_two = dte_saw("\r\n2\r\n");
            if (!saw_two)
                pump(peer, 10);
        }
        check(saw_two, "+FPS? reads back the far end's verdict as 2");
    }

    dte_reset();
    pump(peer, 90 * 50);
    pump_regardless(peer, 5 * 50);

    /* T.32 II.7: "RTN forces back to phase B", so the session is negotiated
     * again -- and the DCE reports the new parameters, at the rate it stepped
     * down to. */
    /* T.32 II.7: "RTN forces back to Phase B".  The buffer was cleared after
     * the rejection, so any +FCS: here belongs to the renegotiation. */
    check(dte_count("+FCS:") >= 1, "the retrain is reported to the DTE");
    check(dte_saw("+FHS:00"), "and the session completes normally");
    check(peer_done && peer_status == T30_ERR_OK,
          "the far end reports a good session");

    /*
     * What the far end holds is the whole point: page one, then page one
     * again after the retrain, then page two.  It keeps the copy it refused,
     * which is its own business -- what matters here is that the rejected
     * page was sent a second time and that page two still followed it.  A
     * DCE that skipped the bad page would leave two pages and the wrong one
     * missing, and would still report a good session.
     */
    {
        TIFF *t = TIFFOpen(PEER_RX, "r");
        int pages = 0;

        if (t) {
            do
                pages++;
            while (TIFFReadDirectory(t));
            TIFFClose(t);
        }
        check(pages == 3, "the far end holds the rejected page, the repeat and page two");
        if (pages != 3)
            printf("       (%d pages)\n", pages);
    }
    row_variant = 0;
    bad = compare_page(PEER_RX, 1, &rows);
    check(bad == 0 && rows == IMAGE_ROWS, "the repeat is page one, sent again");
    if (bad != 0 || rows != IMAGE_ROWS)
        printf("       (repeat: %d rows, %d differing)\n", rows, bad);
    row_variant = 1;
    bad = compare_page(PEER_RX, 2, &rows);
    check(bad == 0 && rows == IMAGE_ROWS, "and page two followed it");
    if (bad != 0 || rows != IMAGE_ROWS)
        printf("       (page two: %d rows, %d differing)\n", rows, bad);
    row_variant = 0;

    if (failures != failures_before)
        printf("       (T.30 frames at the far end: %s)\n", peer_trace);

    peer_reject_pages = 0;
    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * A document the DTE never finished must not survive the call it belonged to.
 * Pages accumulate now, so an abandoned one would be appended to by the next
 * +FDT and transmitted on somebody else's call.  This stays inside one
 * +FCLASS=2.0 selection deliberately -- leaving and re-entering class 2.0
 * clears everything anyway, and would hide the leak.
 */
static void test_unfinished_document_discarded(void)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int rows = 0;
    int bad;
    int n;

    printf("an unfinished document does not outlive its call\n");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    peer_interrupt = 0;

    fc2_select(0);
    fc2_select(1);
    at("AT+FNR=0,1,0,0");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");

    /* Call one: hand over a page that says another follows, then hang up. */
    at("ATD5551234");
    check(send_page_via_fdt(1, 0x2C), "a page is handed over with MPS");
    fc2_on_disconnected();

    /* Call two, no +FCLASS in between: one page, complete. */
    unlink(PEER_RX);
    at("ATD5551234");
    at("AT+FDT");
    n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0, page, sizeof(page));
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : n - off;
        fc2_dte_bytes(page + off, chunk);
    }

    peer = peer_start(0, NULL, PEER_RX);
    if (!peer)
        return;
    dte_reset();
    fc2_on_connected();
    pump(peer, 90 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();

    check(peer_done && peer_status == T30_ERR_OK,
          "the second call completes");
    row_variant = 0;
    bad = compare_page(PEER_RX, 0, &rows);
    check(bad == 0 && rows == IMAGE_ROWS,
          "its first page is the page it was given");
    /* And there is no second page carried over from the abandoned document. */
    check(compare_page(PEER_RX, 1, &rows) < 0,
          "and the abandoned page did not come with it");

    fax_free(peer);
    fc2_on_disconnected();
}

/*
 * T.32 8.5.2.1 +FIE and 8.4.4.2 +FVO -- procedure interrupts.
 *
 * There are two of them and they go opposite ways.  A transmitting DTE asks
 * with 8.3.3.8's <DLE><pri>, which makes T.30's post page message a PRI-Q, and
 * the remote grants by answering PIN or PIP.  A receiving DTE asks by setting
 * +FPS to 4 or 5 before the post page +FDR, which makes T.30's post page
 * RESPONSE a PIN or PIP.  Both are checked on the wire, from the far end's own
 * frame handler, because both are a substitution inside a frame that gets sent
 * either way -- a session that ignored the request entirely still completes.
 */
static void run_interrupt_tx_session(const char *fie, int send_pri)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int n;

    fc2_select(0);
    fc2_select(1);
    unlink(PEER_RX);

    at("AT+FLI=\"sender\"");
    at(fie);
    at("AT+FNR=0,1,0,0");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATD5551234");
    at("AT+FDT");

    n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0, page, sizeof(page));
    /* Strip the <DLE><ETX> the encoder appended; this session ends the page
     * itself, with 8.3.3.7's <DLE><ppm> and optionally 8.3.3.8's <DLE><pri>
     * before it. */
    if (n >= 2)
        n -= 2;
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : n - off;
        fc2_dte_bytes(page + off, chunk);
    }
    if (send_pri) {
        static const uint8_t pri[2] = { 0x10, 0x21 };   /* <DLE><pri> */
        fc2_dte_bytes(pri, 2);
    }
    {
        static const uint8_t eop[2] = { 0x10, 0x2E };   /* <DLE><ppm>, EOP */
        fc2_dte_bytes(eop, 2);
    }

    peer = peer_start(0, NULL, PEER_RX);
    if (!peer)
        return;
    dte_reset();
    fc2_on_connected();
    pump(peer, 60 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();
    fax_free(peer);
    fc2_on_disconnected();
}

static void test_procedure_interrupt(void)
{
    int rows = 0;

    printf("+FIE/+FVO: procedure interrupts (T.32 8.5.2.1, 8.4.4.2)\n");

    fc2_select(0);
    fc2_select(1);
    dte_reset();
    at("AT+FIE=1");
    check(dte_saw("OK"), "AT+FIE=1 is accepted");
    dte_reset();
    at("AT+FIE?");
    check(dte_saw("1"), "AT+FIE? reads it back");
    dte_reset();
    at("AT+FIE=2");
    check(dte_saw("ERROR"), "AT+FIE=2 is out of range");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;

    /* 8.3.3.7's <DLE><ppm> terminates the page, with no interrupt asked for. */
    peer_interrupt = 0;
    run_interrupt_tx_session("AT+FIE=1", 0);
    check(peer_saw_ppm == (T30_EOP & 0xFE),
          "<DLE><ppm> ends the page with the plain post page message");
    check(compare_page(PEER_RX, 0, &rows) == 0 && rows == IMAGE_ROWS,
          "and the page still arrives intact");
    check(!dte_saw("+FVO"), "no +FVO when no interrupt was asked for");

    /*
     * 8.3.3.8: with <DLE><pri> the post page message becomes a PRI-Q.  The
     * far end here grants it, answering PIP, which is 8.4.4.2's +FVO.
     */
    peer_interrupt = 1;
    run_interrupt_tx_session("AT+FIE=1", 1);
    check(peer_saw_ppm == (T30_PRI_EOP & 0xFE),
          "<DLE><pri> makes the post page message a PRI-Q");
    check(peer_saw_ppr == (T30_PIP & 0xFE) || peer_saw_ppr == (T30_PIN & 0xFE),
          "the far end grants it with PIP or PIN");
    check(dte_saw("+FVO"), "the granted interrupt is reported as +FVO");
    dte_reset();
    at("AT+FPS?");
    check(dte_saw("5") || dte_saw("4"),
          "and the +FPS parameter carries the PIP/PIN page status");

    /* 8.5.2.1: with +FIE=0 the grant is not reported. */
    run_interrupt_tx_session("AT+FIE=0", 1);
    check(!dte_saw("+FVO"), "+FIE=0 does not report the interrupt");

    /*
     * 8.5.2.1 the other way: a PRI-Q from the far end.  With +FIE=1 the
     * +FET: report carries the PRI-Q code and +FPS is adjusted to 4 or 5;
     * with +FIE=0 the report carries the non-PRI equivalent.  The far end is
     * the transmitter here, so it is the one sending the PRI-Q.
     */
    for (int fie = 1; fie >= 0; fie--) {
        fax_state_t *peer;
        char cmd[16];

        fc2_select(0);
        fc2_select(1);
        snprintf(cmd, sizeof(cmd), "AT+FIE=%d", fie);
        at(cmd);
        at("AT+FCR=1");
        at("AT+FNR=0,1,0,0");
        at("AT+FIS=1,3,0,2,0,0,0,0,0");
        at("ATA");
        peer_interrupt = 1;
        peer = peer_start(1, PEER_TX, NULL);
        if (!peer)
            continue;
        dte_reset();
        fc2_on_connected();
        at("AT+FDR");
        pump_until(peer, 60 * 50, "+FPS:");
        release_post_page(peer);
        check(peer_saw_ppm == (T30_PRI_EOP & 0xFE),
              "the far end sends a PRI-Q post page message");
        if (fie) {
            /* Table 19: 5 is PRI-EOP. */
            check(dte_saw("+FET:5"), "+FIE=1 reports the PRI-Q in +FET:");
            check(dte_saw("+FPS:5,") || dte_saw("+FPS:4,"),
                  "+FIE=1 adjusts +FPS to 4 or 5");
        } else {
            /* Table 19: PRI-EOP reported as its non-PRI equivalent, EOP=2. */
            check(dte_saw("+FET:2"), "+FIE=0 reports the non-PRI equivalent");
            check(!dte_saw("+FET:5"), "and not the PRI-Q code");
        }
        fax_free(peer);
        fc2_on_disconnected();
    }
    /*
     * 8.3.4.8 the other way round: a RECEIVING DTE asks for an interrupt by
     * setting +FPS to 4 or 5, which makes T.30's post page RESPONSE a PIN or
     * PIP.  In the order 8.5.2.2 describes: take the page, read the status
     * the DCE reported, write a new one, and let the next +FDR release it.
     */
    {
        fax_state_t *peer;

        fc2_select(0);
        fc2_select(1);
        at("AT+FIE=1");
        at("AT+FCR=1");
        at("AT+FNR=0,1,0,0");
        at("AT+FIS=1,3,0,2,0,0,0,0,0");
        at("ATA");
        peer_interrupt = 0;
        peer = peer_start(1, PEER_TX, NULL);
        if (peer) {
            dte_reset();
            fc2_on_connected();
            at("AT+FDR");
            pump_until(peer, 60 * 50, "+FPS:");
            check(dte_saw("+FPS:1,"), "the page was reported as good");
            /* Now overwrite it: PIP is "page good, interrupt requested". */
            at("AT+FPS=5");
            release_post_page(peer);
            check(peer_saw_ppr == (T30_PIP & 0xFE),
                  "+FPS=5 before +FDR answers the page with PIP (8.3.4.8)");
            if (peer_saw_ppr != (T30_PIP & 0xFE))
                printf("       (far end saw post page response 0x%02X)\n",
                       peer_saw_ppr);
            fax_free(peer);
        }
        fc2_on_disconnected();
    }

    peer_interrupt = 0;
}

/*
 * T.32 8.5.1.6 +FNS.  The parameter's own rules first, then a session that
 * checks the octets actually reached the far end -- a +FNS that parsed
 * perfectly and put nothing on the line would pass every syntax check.
 */
static void test_fns(void)
{
    static const uint8_t nss[] = { 0xAD, 0x11, 0x22 };
    fax_state_t *peer;

    printf("+FNS: non-standard frame sending (T.32 8.5.1.6)\n");

    fc2_select(0);
    fc2_select(1);

    dte_reset();
    at("AT+FNS=?");
    check(dte_saw("(90)"), "AT+FNS=? reports the octet capacity");

    dte_reset();
    at("AT+FNS=\"AD0102\"");
    at("AT+FNS?");
    check(dte_saw("\"AD0102\""), "AT+FNS= sets the FIF");

    /* 8.5.1.6: "each use appends data to the data entered previously". */
    dte_reset();
    at("AT+FNS=\"0304\"");
    at("AT+FNS?");
    check(dte_saw("\"AD01020304\""), "a second AT+FNS appends");

    /* 8.5.1.6: "Spaces between octets shall be ignored by the DCE." */
    dte_reset();
    at("AT+FNS=\"\"");
    at("AT+FNS=\"AD 01 02\"");
    at("AT+FNS?");
    check(dte_saw("\"AD0102\""), "spaces between octets are ignored");

    dte_reset();
    at("AT+FNS=\"\"");
    at("AT+FNS?");
    check(dte_saw("\"\""), "AT+FNS=\"\" resets it to the null string");

    dte_reset();
    at("AT+FNS=\"ADXY\"");
    check(dte_saw("ERROR"), "a non-hex character is refused");
    dte_reset();
    at("AT+FNS=\"AD0\"");
    check(dte_saw("ERROR"), "a half octet is refused");

    /* Over the 90 octet limit. */
    {
        char cmd[256];
        int n = 0;

        at("AT+FNS=\"\"");
        n += snprintf(cmd + n, sizeof(cmd) - (size_t) n, "AT+FNS=\"");
        for (int i = 0; i < 91; i++)
            n += snprintf(cmd + n, sizeof(cmd) - (size_t) n, "AD");
        snprintf(cmd + n, sizeof(cmd) - (size_t) n, "\"");
        dte_reset();
        at(cmd);
        check(dte_saw("ERROR"), "more than 90 octets is refused");
    }

    /*
     * And it reaches the far end.  We call, so our non-standard frame travels
     * with the DCS -- which makes it an NSS (8.5.1.6: "NSF sent with DIS; NSS
     * sent with DCS; NSC sent with DTC").
     */
    at("AT+FNS=\"\"");
    at("AT+FNS=\"AD5566\"");
    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;
    peer_nss = NULL;
    {
        static uint8_t page[1 << 20];
        int n;

        at("AT+FIS=1,3,0,2,0,0,0,0,0");
        at("ATD5551234");
        at("AT+FDT");
        n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0,
                                page, sizeof(page));
        for (int off = 0; off < n; off += 512) {
            int chunk = (n - off > 512) ? 512 : n - off;
            fc2_dte_bytes(page + off, chunk);
        }
        unlink(PEER_RX);
        peer = peer_start(0, NULL, PEER_RX);
        dte_reset();
        fc2_on_connected();
        pump(peer, 60 * 50);
        for (int i = 0; i < 20; i++)
            fc2_poll();
    }
    if (peer) {
        static const uint8_t want[] = { 0xAD, 0x55, 0x66 };

        check(peer_saw_nss_len == (int) sizeof(want)
              && memcmp(peer_saw_nss, want, sizeof(want)) == 0,
              "the +FNS octets arrive at the far end in an NSS frame");
        if (peer_saw_nss_len != (int) sizeof(want))
            printf("       (far end received %d octets)\n", peer_saw_nss_len);
        fax_free(peer);
    }
    fc2_on_disconnected();

    /* The other direction: a received NSS is 8.4.2.4's +FNS: report. */
    fc2_select(0);
    fc2_select(1);
    at("AT+FCR=1");
    at("AT+FNR=0,0,1,1");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATA");
    peer_nss = nss;
    peer_nss_len = (int) sizeof(nss);
    peer = peer_start(1, PEER_TX, NULL);
    if (peer) {
        dte_reset();
        fc2_on_connected();
        at("AT+FDR");
        pump_until(peer, 60 * 50, "+FPS:");
        release_post_page(peer);
        check(dte_saw("+FNS:AD 11 22"),
              "a received NSS is reported as +FNS: (8.4.2.4)");
        check(!dte_saw("+FNF:AD 11 22"),
              "and not as +FNF:, which is the NSF report");
        /*
         * The far end is the transmitter here, so its identification arrives
         * in a TSI -- whose FCF carries the "a DIS was received" low bit, so
         * it is 0x43 rather than 0x42.  Matching it exactly reported every
         * such TSI as a CSI instead.
         */
        check(dte_saw("+FTI:\"peer\""),
              "a transmitting far end's TSI is reported as +FTI:");
        check(!dte_saw("+FCI:\"peer\""), "and not as +FCI:");
        fax_free(peer);
    }
    fc2_on_disconnected();
    peer_nss = NULL;
}

/*
 * T.32 8.5.1.10 and 8.6.  +FBU reports every phase B and phase D HDLC frame
 * in both directions.  This runs a transmit session and checks the reports
 * against what T.30 must have exchanged, then repeats it at +FBO=2 -- the
 * half of the bit order parameter that has nothing to act on until +FBU
 * exists.
 */
static void run_fbu_session(const char *fbu, const char *fbo, const char *fis)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int n;

    fc2_select(0);
    fc2_select(1);
    unlink(PEER_RX);

    at("AT+FLI=\"sender\"");
    at(fbu);
    at(fbo);
    at("AT+FNR=1,1,1,1");
    at(fis);
    at("ATD5551234");
    at("AT+FDT");
    n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0, page, sizeof(page));
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : n - off;
        fc2_dte_bytes(page + off, chunk);
    }

    peer = peer_start(0, NULL, PEER_RX);
    if (!peer)
        return;
    dte_reset();
    fc2_on_connected();
    pump(peer, 60 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();
    fax_free(peer);
    fc2_on_disconnected();
}

static void test_fbu(void)
{
    printf("+FBU: HDLC frame reporting (T.32 8.5.1.10, 8.6)\n");

    dte_reset();
    at("AT+FBU=1");
    check(dte_saw("OK"), "AT+FBU=1 is accepted");
    check(fc2_echo_suppressed(), "+FBU=1 suppresses command echo (8.6)");
    dte_reset();
    at("AT+FBU=0");
    check(!fc2_echo_suppressed(), "+FBU=0 restores it");
    dte_reset();
    at("AT+FBU=2");
    check(dte_saw("ERROR"), "AT+FBU=2 is out of range");

    peer_ecm = 1;
    peer_t6 = 1;
    peer_nsf = NULL;

    run_fbu_session("AT+FBU=0", "AT+FBO=0", "AT+FIS=1,3,0,2,0,0,0,0,0");
    check(!dte_saw("+FHR:") && !dte_saw("+FHT:"),
          "+FBU=0 reports no frames");

    run_fbu_session("AT+FBU=1", "AT+FBO=0", "AT+FIS=1,3,0,2,0,0,0,0,0");
    check(dte_saw("+FHR:"), "+FBU=1 reports received frames");
    check(dte_saw("+FHT:"), "+FBU=1 reports transmitted frames");

    /*
     * The frames themselves.  A transmit session must exchange a DIS from the
     * far end and a DCS from us, and 8.6 requires the address and control
     * fields to be kept and the FCS dropped -- so a received DIS reports as
     * "FF 13 80 ...", which is the form of the worked example in 8.6.
     */
    check(dte_saw("+FHR: FF 13 80 "), "a received DIS reports as 8.6 shows");
    check(dte_saw("+FHT: FF 13 83 "), "the DCS we send is reported");

    /* 8.6: the frame report precedes the response derived from it. */
    {
        const char *dis = dte_find("+FHR: FF 13 80 ");
        const char *fis = dte_find("+FIS:");

        check(dis && fis && dis < fis,
              "the frame report precedes the +FIS: it produced");
    }

    /*
     * 8.6: ECM phase C data frames are excluded.  That has to be checked on a
     * session that actually runs ECM -- the run above has none, so the
     * absence of an FCD report there would prove nothing.  T4_FCD is 0x06 in
     * a non-final frame (FF 03 06 ...), and an ECM page is hundreds of them.
     */
    run_fbu_session("AT+FBU=1", "AT+FBO=0", "AT+FIS=1,3,0,2,0,2,0,0,0");
    {
        const char *fcs = dte_find("+FCS:");
        int vr, br, wd, ln, df, ec = 0;

        check(fcs && sscanf(fcs, "+FCS:%d,%d,%d,%d,%d,%d",
                            &vr, &br, &wd, &ln, &df, &ec) == 6 && ec != 0,
              "the ECM control session negotiated ECM");
    }
    check(dte_saw("+FHT:") && dte_saw("+FHR:"),
          "control frames are still reported in an ECM session");
    check(!dte_saw("+FHT: FF 03 06 "),
          "ECM phase C data frames are not reported");

    /*
     * T.32 8.5.3.4: +FBO=2 selects reversed bit order for phase B/D data,
     * which is exactly these reports.  The DIS above reverses octet by octet:
     * FF -> FF, 13 -> C8, 80 -> 01.
     */
    run_fbu_session("AT+FBU=1", "AT+FBO=2", "AT+FIS=1,3,0,2,0,0,0,0,0");
    check(dte_saw("+FHR: FF C8 01 "),
          "+FBO=2 reverses the bit order of the frame reports");
    check(!dte_saw("+FHR: FF 13 80 "),
          "and the direct form is gone");
}

/*
 * T.32 8.5.1.7/8.5.1.8 polling.  Two whole sessions: one where the far end
 * offers a document and this DCE polls it (+FSP, +FPO, +FDR), and one where
 * this DCE offers a document and the far end polls it (+FLP, DTC).  A polled
 * transfer that reports +FPO and then delivers nothing would pass a
 * result-code check, so both compare the page.
 */
static void test_poll_remote(void)
{
    fax_state_t *peer;
    t4_rx_state_t *decode;
    const char *body;
    int rows = 0;
    int bad;
    int saw_dle = 0;
    int start;

    printf("+FSP: this DCE polls the far end (T.32 8.5.1.8)\n");

    unlink(DTE_RX);
    fc2_select(0);
    fc2_select(1);

    at("AT+FLI=\"poller\"");
    at("AT+FCR=1");
    at("AT+FNR=1,1,1,1");
    at("AT+FSP=1");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATD5551234");            /* the poller places the call */

    /* The far end answers with a document available, which sets DIS bit 9. */
    peer = peer_start(0, PEER_TX, NULL);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    at("AT+FDR");
    pump_until(peer, 60 * 50, "+FPS:");
    release_post_page(peer);

    check(peer_done && peer_status == T30_ERR_OK,
          "the far end reports a good polled session");
    check(dte_saw("+FPO"), "the remote's offer to be polled is reported");
    /*
     * We called, so the far end's identification arrives in a CSI and is
     * 8.4.2.3's +FCI:.  Written as an either/or this assertion passed while
     * every TSI was being misreported as a CSI, so it is exact now.
     */
    check(dte_saw("+FCI:\"peer\""), "the remote's CSI is reported as +FCI:");

    /* The page itself, checked before any query -- a query resets the
     * capture the page is in. */
    body = dte_find("CONNECT\r\n");
    check(body != NULL, "the polled page follows CONNECT");
    if (!body)
        goto done;
    start = (int) (body - dte_buf);
    decode = t4_rx_init(NULL, DTE_RX, T4_COMPRESSION_T4_1D
                                      | T4_COMPRESSION_T4_2D
                                      | T4_COMPRESSION_T6);
    if (decode) {
        t4_rx_set_rx_encoding(decode, T4_COMPRESSION_T4_1D);
        t4_rx_set_image_width(decode, IMAGE_WIDTH);
        t4_rx_set_x_resolution(decode, T4_X_RESOLUTION_R8);
        t4_rx_set_y_resolution(decode, T4_Y_RESOLUTION_FINE);
        t4_rx_start_page(decode);
        for (int i = start; i < dte_len; i++) {
            uint8_t byte = (uint8_t) dte_buf[i];

            if (saw_dle) {
                saw_dle = 0;
                if (byte == 0x03)
                    break;
                if (byte != 0x10)
                    continue;
            } else if (byte == 0x10) {
                saw_dle = 1;
                continue;
            }
            t4_rx_put(decode, &byte, 1);
        }
        t4_rx_end_page(decode);
        t4_rx_free(decode);
        bad = compare_page(DTE_RX, 0, &rows);
        check(bad == 0 && rows == IMAGE_ROWS,
              "the polled page is the page the far end offered");
        if (bad != 0 || rows != IMAGE_ROWS)
            printf("       (%d rows, %d differing)\n", rows, bad);
    }

    dte_reset();
    at("AT+FSP?");
    check(dte_saw("0"), "the DCE resets +FSP after a polled document arrives");

done:
    fax_free(peer);
    fc2_on_disconnected();
}

static void test_be_polled(int flp)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int rows = 0;
    int bad;
    int n;

    printf("+FLP=%d: the far end polls this DCE (T.32 8.5.1.7)\n", flp);

    unlink(PEER_RX);
    fc2_select(0);
    fc2_select(1);

    at("AT+FLI=\"polled\"");
    at("AT+FNR=0,1,0,0");
    { char cmd[32]; snprintf(cmd, sizeof(cmd), "AT+FLP=%d", flp); at(cmd); }
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATA");                   /* the polled station answers */

    dte_reset();
    at("AT+FDT");
    check(dte_saw("CONNECT"), "AT+FDT answers CONNECT");
    n = encode_page_for_dte(SRC_TIFF, T4_COMPRESSION_T4_1D, 0, page, sizeof(page));
    for (int off = 0; off < n; off += 512) {
        int chunk = (n - off > 512) ? 512 : n - off;
        fc2_dte_bytes(page + off, chunk);
    }

    /* The far end calls in with nothing to send and somewhere to put a page,
     * which is what makes it poll. */
    peer = peer_start(1, NULL, PEER_RX);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    pump(peer, 60 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();

    if (flp) {
        check(peer_done && peer_status == T30_ERR_OK,
              "the far end reports a good polled session");
        bad = compare_page(PEER_RX, 0, &rows);
        check(bad == 0 && rows == IMAGE_ROWS,
              "the far end polled the page that was offered");
        if (bad != 0 || rows != IMAGE_ROWS)
            printf("       (%d rows, %d differing)\n", rows, bad);
        dte_reset();
        at("AT+FLP?");
        check(dte_saw("0"), "the DCE resets +FLP after a polled document is sent");
    } else {
        /* T.32 8.5.1.7: with no document offered, the DTC is refused and the
         * call ends with that status rather than transferring the page. */
        /*
         * A conformant far end reads DIS bit 9, sees no document on offer and
         * gives up without ever sending a DTC -- which is the point of the
         * bit.  So this is what is observable here; the +FHS:23 that 8.5.1.7
         * specifies for a DTC that arrives anyway is implemented but needs a
         * peer that sends one unsolicited, which this one will not.
         */
        check(peer_status != T30_ERR_OK,
              "with +FLP=0 the far end does not get a document");
        check(compare_page(PEER_RX, 0, &rows) < 0,
              "with +FLP=0 no page reaches the far end at all");
        dte_reset();
        at("AT+FHS?");
        check(!dte_saw("00"), "the refused session reports a hangup cause");
    }

    fax_free(peer);
    fc2_on_disconnected();
}

int main(void)
{
    TIFFSetWarningHandler(NULL);
    TIFFSetErrorHandler(NULL);

    if (!write_test_tiff(SRC_TIFF)) {
        fprintf(stderr, "cannot write the test page\n");
        return 1;
    }
    if (!write_test_tiff(PEER_TX)) {
        fprintf(stderr, "cannot write the far end's test page\n");
        return 1;
    }

    fc2_init(dce_write, dce_dial, dce_answer, dce_hangup, NULL);
    fc2_select(1);

    test_parameters();
    test_transmit(0, 0);
    test_receive(0, 0);

    /*
     * The same two sessions in T.30 Annex A error correction mode.  ECM is
     * what a real peer negotiates, and it is not a variation on the plain
     * session from the DCE's point of view: T.30 buffers the page and
     * releases the T.4 receiver that counted its lines before the DTE
     * collects it, so 8.4.3's line counts have to be latched at the end of
     * the page.  Without that every page of every ECM fax is reported to the
     * DTE as "+FPS:1,0,0,0,0".
     */
    test_transmit(0, 2);
    test_receive(0, 2);

    /*
     * EC=1 is the same mode with T.30 A.3.2's other frame size, 64 octets.
     * The transmitting terminal chooses (A.3.1) and the receiving terminal
     * asks in bit 7 of its DIS, so each direction exercises a different half:
     * transmitting, this DCE puts 64 in the DCS and sends 64-octet frames;
     * receiving, its DIS asks and the far end honours it.
     */
    test_transmit(0, 1);
    test_receive(0, 1);

    /*
     * T.32 8.5.3.4.  The same two sessions with the phase C octets reversed on
     * the DTE link: the pages must still arrive intact, which they can only do
     * if the reversal is applied in both directions and in the right place
     * relative to the DLE stuffing.
     */
    test_transmit(1, 0);
    test_receive(1, 0);
    test_transmit(1, 2);
    test_receive(1, 2);
    test_bit_order_is_real();
    test_fnr();
    test_multipage_transmit(0, 0);
    test_multipage_transmit(1, 0);
    /* The same, in error correction mode -- and at both frame sizes, since
     * T.30 A.1 rebuilds the ECM block per page. */
    test_multipage_transmit(0, 2);
    test_multipage_transmit(1, 1);
    test_multipage_receive(0);
    test_multipage_receive(2);
    test_multipage_receive(1);
    test_page_rejected_rtn(0);
    test_page_rejected_rtn(2);
    test_transmit_page_rejected();
    test_unfinished_document_discarded();
    test_post_page_hold();
    test_held_response_timeout();
    test_transmit_timeout(1);
    test_transmit_timeout(0);
    test_transmit_timeout_not_tripped();
    test_procedure_interrupt();
    test_fns();
    test_fbu();
    test_poll_remote();
    test_be_polled(1);
    test_be_polled(0);

    fc2_release();

    printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
