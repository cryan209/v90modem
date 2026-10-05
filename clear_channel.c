/*
 * clear_channel.c — 64 kbit/s clear channel and V.120 over the DS0
 *
 * See clear_channel.h for what the two modes are and the conventions.
 */
#include "clear_channel.h"

#include <stdlib.h>
#include <string.h>

int cc_line_rate(const clear_channel_t *cc)
{
    return cc->r56 ? 56000 : 64000;
}

/* ---------------------------------------------------------------- */
/* Octet <-> bit stream                                              */
/* ---------------------------------------------------------------- */

static int next_bit(clear_channel_t *cc)
{
    int bit;

    if (cc->mode == CC_CLEAR) {
        bit = cc->get_bit ? cc->get_bit(cc->ctx) : 1;
        return bit < 0 ? 1 : (bit & 1);
    }
    bit = hdlc_tx_get_bit(cc->htx);
    return bit < 0 ? 1 : (bit & 1);
}

static void took_bit(clear_channel_t *cc, int bit)
{
    if (cc->mode == CC_CLEAR) {
        if (cc->put_bit)
            cc->put_bit(cc->ctx, bit);
        return;
    }
    hdlc_rx_put_bit(cc->hrx, bit);
}

/* ---------------------------------------------------------------- */
/* V.120 framing                                                     */
/* ---------------------------------------------------------------- */

void cc_v120_frame_header(const clear_channel_t *cc, uint8_t hdr[4])
{
    /* Two-octet address: LLI high six bits, C/R, EA=0; LLI low seven bits,
     * EA=1.  UI is a command, which the originator sends with C/R 0 and
     * the answerer with C/R 1, as in LAPD. */
    int cr = cc->caller ? 0 : 1;

    hdr[0] = (uint8_t) ((((cc->lli >> 7) & 0x3F) << 2) | (cr << 1));
    hdr[1] = (uint8_t) (((cc->lli & 0x7F) << 1) | 1);
    hdr[2] = 0x03;                                   /* UI, P = 0 */
    hdr[3] = CC_V120_H_E | CC_V120_H_B | CC_V120_H_F; /* complete, no CS */
}

/* Queue a frame of whatever the DTE has, if it has anything. */
static void v120_load_frame(clear_channel_t *cc)
{
    uint8_t frame[4 + CC_V120_MAX_DATA];
    int n = 0;

    if (cc->tx_frame_queued || !cc->pull)
        return;
    while (n < CC_V120_MAX_DATA) {
        int b = cc->pull(cc->ctx);

        if (b < 0)
            break;
        frame[4 + n++] = (uint8_t) b;
    }
    if (n == 0)
        return;
    cc_v120_frame_header(cc, frame);
    if (hdlc_tx_frame(cc->htx, frame, (size_t) (4 + n)) == 0) {
        cc->tx_frame_queued = true;
        cc->tx_frames++;
        cc->tx_data_bytes += (uint64_t) n;
    }
}

static void v120_underflow(void *user_data)
{
    clear_channel_t *cc = (clear_channel_t *) user_data;

    cc->tx_frame_queued = false;
    v120_load_frame(cc);
}

static void v120_frame(void *user_data, const uint8_t *pkt, int len, int ok)
{
    clear_channel_t *cc = (clear_channel_t *) user_data;
    int pos;

    if (len < 0)
        return;            /* a status report, not a frame */
    if (!ok || len < 4) {
        cc->rx_bad_frames++;
        return;
    }
    /* The address must be two octets: EA 0 then 1. */
    if ((pkt[0] & 0x01) != 0 || (pkt[1] & 0x01) != 1) {
        cc->rx_bad_frames++;
        return;
    }
    /* UI (P/F either way).  Anything else -- I-frames, RR, SABME -- belongs
     * to the multiple-frame acknowledged mode this does not implement. */
    if ((pkt[2] & ~0x10) != 0x03) {
        cc->rx_unsupported++;
        return;
    }
    cc->rx_frames++;
    pos = 3;
    /* Header octet; with E = 0, control-state octets follow, each with its
     * own extension bit, until one has bit 8 set. */
    {
        uint8_t h = pkt[pos++];

        if (h & CC_V120_H_BR)
            cc->rx_breaks++;
        if (!(h & CC_V120_H_E)) {
            while (pos < len && !(pkt[pos] & 0x80))
                pos++;
            if (pos < len)
                pos++;
        }
    }
    for (; pos < len; pos++) {
        if (cc->push)
            cc->push(cc->ctx, pkt[pos]);
        cc->rx_data_bytes++;
    }
}

/* ---------------------------------------------------------------- */
/* Lifecycle and the octet interface                                 */
/* ---------------------------------------------------------------- */

int cc_init_clear(clear_channel_t *cc, bool r56,
                  cc_get_bit_fn get_bit, cc_put_bit_fn put_bit, void *ctx)
{
    memset(cc, 0, sizeof(*cc));
    cc->mode = CC_CLEAR;
    cc->r56 = r56;
    cc->get_bit = get_bit;
    cc->put_bit = put_bit;
    cc->ctx = ctx;
    return 0;
}

int cc_init_v120(clear_channel_t *cc, bool r56, bool caller,
                 cc_pull_byte_fn pull, cc_push_byte_fn push, void *ctx)
{
    memset(cc, 0, sizeof(*cc));
    cc->mode = CC_V120;
    cc->r56 = r56;
    cc->caller = caller;
    cc->lli = CC_V120_DEFAULT_LLI;
    cc->pull = pull;
    cc->push = push;
    cc->ctx = ctx;
    cc->htx = hdlc_tx_init(NULL, false, 1, false, v120_underflow, cc);
    cc->hrx = hdlc_rx_init(NULL, false, true, 1, v120_frame, cc);
    if (!cc->htx || !cc->hrx) {
        cc_release(cc);
        return -1;
    }
    hdlc_tx_set_max_frame_len(cc->htx, 4 + CC_V120_MAX_DATA);
    /* SpanDSP's transmitter starts with no flag at all, so a frame queued
     * at once would begin at the first octet with nothing to sync a
     * receiver to.  Open with flags, as a line idles before data. */
    hdlc_tx_flags(cc->htx, 2);
    hdlc_rx_set_max_frame_len(cc->hrx, 4 + CC_V120_MAX_DATA + 16);
    return 0;
}

void cc_release(clear_channel_t *cc)
{
    if (cc->htx)
        hdlc_tx_free(cc->htx);
    if (cc->hrx)
        hdlc_rx_free(cc->hrx);
    cc->htx = NULL;
    cc->hrx = NULL;
}

void cc_tx(clear_channel_t *cc, uint8_t *octets, int n)
{
    int nbits = cc->r56 ? 7 : 8;

    if (cc->mode == CC_V120)
        v120_load_frame(cc);
    for (int i = 0; i < n; i++) {
        uint8_t o = 0;

        for (int b = 0; b < nbits; b++)
            o = (uint8_t) (o | (next_bit(cc) << (7 - b)));
        if (cc->r56)
            o |= 0x01;
        octets[i] = o;
    }
    cc->tx_octets += (uint64_t) n;
}

void cc_rx(clear_channel_t *cc, const uint8_t *octets, int n)
{
    int nbits = cc->r56 ? 7 : 8;

    for (int i = 0; i < n; i++)
        for (int b = 0; b < nbits; b++)
            took_bit(cc, (octets[i] >> (7 - b)) & 1);
    cc->rx_octets += (uint64_t) n;
}
