/*
 * v8bis_msg.c -- V.8bis message framing.  Clause 7.2 (flags, FCS, transparency
 * and invalid frames).
 */
#include "v8bis_msg.h"

#include <string.h>

uint16_t v8bis_fcs(const uint8_t *info, size_t len)
{
    uint16_t reg = 0xffff;

    for (size_t i = 0; i < len; i++)
        for (int b = 0; b < 8; b++) {
            unsigned in = (info[i] >> b) & 1u;                 /* bit 1 first */
            unsigned fb = ((reg >> 15) & 1u) ^ in;

            reg = (uint16_t)(reg << 1);
            if (fb)
                reg ^= 0x1021;                                 /* x^16 + x^12 + x^5 + 1 */
        }
    return (uint16_t)~reg;
}

typedef struct {
    uint8_t *bits;
    size_t n, cap;
    bool overflow;
    unsigned ones;
} bitw_t;

static void put_raw(bitw_t *w, unsigned bit)
{
    if (w->n >= w->cap) {
        w->overflow = true;
        return;
    }
    w->bits[w->n++] = (uint8_t)bit;
}

/* Transparency, 7.2.8: a zero after every five ones within the frame. */
static void put_stuffed(bitw_t *w, unsigned bit)
{
    put_raw(w, bit);
    if (bit) {
        if (++w->ones == 5) {
            put_raw(w, 0);
            w->ones = 0;
        }
    } else {
        w->ones = 0;
    }
}

static void put_flag(bitw_t *w)
{
    static const uint8_t F[8] = {0, 1, 1, 1, 1, 1, 1, 0};

    for (int i = 0; i < 8; i++)
        put_raw(w, F[i]);
}

size_t v8bis_frame_encode(const uint8_t *info, size_t len, unsigned preamble_bits,
                          unsigned open_flags, unsigned close_flags, uint8_t *bits, size_t cap)
{
    bitw_t w = {bits, 0, cap, false, 0};
    uint16_t fcs;

    if (!info || !bits || len == 0 || len > V8BIS_MAX_INFO_OCTETS || open_flags < 2
        || open_flags > 5 || close_flags < 1 || close_flags > 3)
        return 0;
    for (unsigned i = 0; i < preamble_bits; i++)
        put_raw(&w, 1);
    for (unsigned i = 0; i < open_flags; i++)
        put_flag(&w);
    for (size_t i = 0; i < len; i++)
        for (int b = 0; b < 8; b++)
            put_stuffed(&w, (info[i] >> b) & 1u);
    fcs = v8bis_fcs(info, len);
    for (int b = 15; b >= 0; b--)                              /* MSB first, Figure 3 */
        put_stuffed(&w, (fcs >> b) & 1u);
    for (unsigned i = 0; i < close_flags; i++)
        put_flag(&w);
    return w.overflow ? 0 : w.n;
}

const char *v8bis_frame_status_name(v8bis_frame_status_t st)
{
    switch (st) {
    case V8BIS_FRAME_OK:    return "ok";
    case V8BIS_FRAME_FCS:   return "fcs error";
    case V8BIS_FRAME_SHORT: return "short frame";
    case V8BIS_FRAME_ALIGN: return "not octet aligned";
    case V8BIS_FRAME_ABORT: return "abort";
    }
    return "?";
}

void v8bis_frame_rx_init(v8bis_frame_rx_t *s, v8bis_frame_cb_t cb, void *user)
{
    memset(s, 0, sizeof(*s));
    s->cb = cb;
    s->user = user;
}

static void report(v8bis_frame_rx_t *s, const uint8_t *info, size_t len, v8bis_frame_status_t st)
{
    if (s->cb)
        s->cb(s->user, info, len, st);
}

/* The closing flag arrived with `nbits` data bits seen, the last seven of which
 * (the zero and six ones of the flag, less a zero shared with a previous flag)
 * were not data. */
static void end_frame(v8bis_frame_rx_t *s)
{
    unsigned n = s->nbits >= 7 ? s->nbits - 7 : 0;

    s->nbits = 0;
    if (n == 0) {                       /* back-to-back flags */
        s->flags_seen++;
        return;
    }
    if (n % 8) {
        report(s, NULL, n / 8, V8BIS_FRAME_ALIGN);
    } else if (n / 8 < 3) {
        report(s, NULL, n / 8, V8BIS_FRAME_SHORT);
    } else {
        size_t octets = n / 8;
        /* The FCS is sent MSB first (Figure 3), so each of its two octets
         * arrives bit-reversed relative to the LSB-first order the buffer was
         * filled in. */
        uint16_t want = v8bis_fcs(s->buf, octets - 2);
        uint8_t hi = s->buf[octets - 2], lo = s->buf[octets - 1];
        unsigned f = 0;
        uint16_t got;

        for (int b = 0; b < 8; b++) {
            f |= (unsigned)((hi >> b) & 1u) << (15 - b);
            f |= (unsigned)((lo >> b) & 1u) << (7 - b);
        }
        got = (uint16_t)f;
        if (got == want)
            report(s, s->buf, octets - 2, V8BIS_FRAME_OK);
        else
            report(s, NULL, octets, V8BIS_FRAME_FCS);
    }
    s->flags_seen = 1;
}

void v8bis_frame_rx_bit(v8bis_frame_rx_t *s, int bit)
{
    if (bit) {
        s->ones++;
        if (s->ones >= 7) {             /* 7.2.8's complement: seven ones is not a frame */
            if (s->in_frame && s->nbits)
                report(s, NULL, 0, V8BIS_FRAME_ABORT);
            s->in_frame = false;
            s->nbits = 0;
            s->flags_seen = 0;
            s->ones = 7;
            return;
        }
        if (!s->in_frame)
            return;
        goto append;
    }
    /* a zero */
    if (s->ones == 6) {                 /* flag */
        if (s->in_frame)
            end_frame(s);
        else
            s->flags_seen = 1;
        s->in_frame = true;
        s->nbits = 0;
        s->ones = 0;
        return;
    }
    if (s->ones == 5) {                 /* a stuffed zero: discard */
        s->ones = 0;
        return;
    }
    s->ones = 0;
    if (!s->in_frame)
        return;
append:
    if (s->nbits >= 8u * sizeof(s->buf)) {
        report(s, NULL, 0, V8BIS_FRAME_ABORT);
        s->in_frame = false;
        s->nbits = 0;
        return;
    }
    if (bit)
        s->buf[s->nbits >> 3] |= (uint8_t)(1u << (s->nbits & 7));
    else
        s->buf[s->nbits >> 3] &= (uint8_t)~(1u << (s->nbits & 7));
    s->nbits++;
}
