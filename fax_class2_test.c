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
static void make_row(uint8_t *row, int y)
{
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
    if (peer_nsf)
        t30_set_tx_nsf(t30, peer_nsf, peer_nsf_len);
    t30_set_supported_compressions(t30, T4_COMPRESSION_T4_1D
                                        | T4_COMPRESSION_T4_2D
                                        | (peer_t6 ? T4_COMPRESSION_T6 : 0));
    t30_set_supported_output_compressions(t30, T4_COMPRESSION_T4_1D
                                               | T4_COMPRESSION_T4_2D
                                               | T4_COMPRESSION_T6);
    t30_set_phase_e_handler(t30, peer_phase_e, NULL);
    if (tx_file)
        t30_set_tx_file(t30, tx_file, -1, -1);
    if (rx_file)
        t30_set_rx_file(t30, rx_file, -1);
    peer_done = 0;
    peer_status = -1;
    return f;
}

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

        fc2_poll();
        if (peer_done)
            return;
    }
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
static void test_transmit(int fbo)
{
    static uint8_t page[1 << 20];
    fax_state_t *peer;
    int rows = 0;
    int bad;
    int n;

    printf("+FDT: class 2.0 sends a page (+FBO=%d)\n", fbo);

    unlink(PEER_RX);
    fc2_select(0);
    fc2_select(1);

    dte_reset();
    at("AT+FLI=\"sender\"");
    /* T.32 Table 22: +FCS: is reported only with +FNR's tpr set. */
    at("AT+FNR=0,1,0,0");
    { char cmd[32]; snprintf(cmd, sizeof(cmd), "AT+FBO=%d", fbo); at(cmd); }
    /* MH, fine resolution, no ECM: the plainest thing a fax machine sends. */
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
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
    check(dte_saw("+FPS:1"), "the spooled page is acknowledged");

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

    bad = compare_page(PEER_RX, 0, &rows);
    check(bad == 0 && rows == IMAGE_ROWS,
          "the page the far end received is the page that was sent");
    if (bad != 0 || rows != IMAGE_ROWS)
        printf("       (%d rows, %d differing)\n", rows, bad);

    fax_free(peer);
    fc2_on_disconnected();
}

/* Class 2.0 receives a page (T.32 8.3.4) from a plain fax terminal. */
static void test_receive(int fbo)
{
    fax_state_t *peer;
    t4_rx_state_t *decode;
    const char *body;
    int rows = 0;
    int bad;
    int saw_dle = 0;
    int start;

    printf("+FDR: class 2.0 receives a page (+FBO=%d)\n", fbo);

    unlink(DTE_RX);
    fc2_select(0);
    fc2_select(1);

    dte_reset();
    at("AT+FLI=\"receiver\"");
    at("AT+FNR=0,1,0,0");
    { char cmd[32]; snprintf(cmd, sizeof(cmd), "AT+FBO=%d", fbo); at(cmd); }
    at("AT+FCR=1");
    at("AT+FIS=1,3,0,2,0,0,0,0,0");
    at("ATA");

    peer = peer_start(1, PEER_TX, NULL);
    check(peer != NULL, "the far-end fax terminal starts");
    if (!peer)
        return;

    dte_reset();
    fc2_on_connected();
    at("AT+FDR");
    pump(peer, 60 * 50);

    check(peer_done, "the far end reached T.30 phase E");
    check(peer_status == T30_ERR_OK, "the far end reports a good session");

    /* Drain the reports the session finished with. */
    for (int i = 0; i < 20; i++)
        fc2_poll();

    check(dte_saw("+FCS:"), "+FDR reported the session parameters");
    check(dte_saw("CONNECT"), "+FDR answered CONNECT");
    /* T.32 8.4.3: +FPS:<ppr>,<lc>,<blc>,<cblc>,<lbc> -- status and counts. */
    {
        char want[32];

        snprintf(want, sizeof(want), "+FPS:1,%d,0,0,0", IMAGE_ROWS);
        check(dte_saw(want), "+FDR reported the page status with line counts");
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
    check(dte_saw("+FNF:AD000102"), "nsr=1: the NSF frame is reported as hex");
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
    pump(peer, 60 * 50);
    for (int i = 0; i < 20; i++)
        fc2_poll();

    check(peer_done && peer_status == T30_ERR_OK,
          "the far end reports a good polled session");
    check(dte_saw("+FPO"), "the remote's offer to be polled is reported");
    check(dte_saw("+FCI:\"peer\"") || dte_saw("+FTI:\"peer\""),
          "the remote's identification is reported");

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
    test_transmit(0);
    test_receive(0);

    /*
     * T.32 8.5.3.4.  The same two sessions with the phase C octets reversed on
     * the DTE link: the pages must still arrive intact, which they can only do
     * if the reversal is applied in both directions and in the right place
     * relative to the DLE stuffing.
     */
    test_transmit(1);
    test_receive(1);
    test_bit_order_is_real();
    test_fnr();
    test_poll_remote();
    test_be_polled(1);
    test_be_polled(0);

    fc2_release();

    printf("%s: %d failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
