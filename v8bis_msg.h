/*
 * v8bis_msg.h -- V.8bis message framing (clause 7.2).
 *
 * A message is HDLC over V.21: 100 ms of marking, 2-5 flags, the information
 * field and a 16-bit FCS with zero insertion, then 1-3 flags.  This module is
 * bit level only (the V.21 modem is the caller's); bits are one per byte, 0/1,
 * in transmission order.
 */
#ifndef V8BIS_MSG_H
#define V8BIS_MSG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define V8BIS_PREAMBLE_BITS 30        /* 100 ms of V.21 marking at 300 bit/s */
#define V8BIS_MAX_INFO_OCTETS 64      /* 8.6 */
#define V8BIS_MAX_FRAME_BITS 1200

/* 7.2.7.  Returns the FCS with x^15 in bit 15, to be sent MSB first: the
 * register presets to ones, takes the information bits in transmission order
 * (bit 1 of each octet first), and the result is complemented. */
uint16_t v8bis_fcs(const uint8_t *info, size_t len);

/* Build a message: `preamble_bits` of marking, `open_flags` (2-5) flags, the
 * stuffed information field and FCS, `close_flags` (1-3) flags.  Returns the
 * number of bits, or 0 on a bad argument or if `cap` is too small. */
size_t v8bis_frame_encode(const uint8_t *info, size_t len, unsigned preamble_bits,
                          unsigned open_flags, unsigned close_flags, uint8_t *bits, size_t cap);

typedef enum {
    V8BIS_FRAME_OK = 0,
    V8BIS_FRAME_FCS,        /* 7.2.9 d) */
    V8BIS_FRAME_SHORT,      /* 7.2.9 b): fewer than three octets between flags */
    V8BIS_FRAME_ALIGN,      /* 7.2.9 c): not an integral number of octets */
    V8BIS_FRAME_ABORT       /* seven ones in a frame, or longer than we hold */
} v8bis_frame_status_t;

const char *v8bis_frame_status_name(v8bis_frame_status_t st);

/* `info` excludes the FCS and is valid only for V8BIS_FRAME_OK; for the other
 * statuses it is NULL and len is the octet count seen (0 for ABORT). */
typedef void (*v8bis_frame_cb_t)(void *user, const uint8_t *info, size_t len,
                                 v8bis_frame_status_t status);

typedef struct {
    v8bis_frame_cb_t cb;
    void *user;
    bool in_frame;
    unsigned ones;
    unsigned nbits;
    uint8_t buf[V8BIS_MAX_INFO_OCTETS + 8];
    unsigned flags_seen;            /* consecutive flags before the current frame */
} v8bis_frame_rx_t;

void v8bis_frame_rx_init(v8bis_frame_rx_t *s, v8bis_frame_cb_t cb, void *user);
void v8bis_frame_rx_bit(v8bis_frame_rx_t *s, int bit);

#endif
