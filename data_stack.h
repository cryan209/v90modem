/*
 * data_stack.h — DTE-side data framing between the serial byte stream and
 * the modem datapump bit stream.
 *
 * The datapump (V.22bis/V.34/V.90) moves a synchronous bit stream. The DTE
 * moves async serial bytes. This module owns what the bits in between mean:
 *
 *   DS_FRAMING_V14  — V.14 async character framing (start + 8 data LSB-first
 *                     + stop, mark idle). This is what a real analog modem
 *                     produces in buffered async mode without error
 *                     correction, so it is the default.
 *   DS_FRAMING_RAW  — legacy raw byte shuttle (8 data bits, no framing).
 *                     Only interoperates with another instance of this
 *                     software; kept for loopback debugging.
 *
 * V.42 LAPM will slot in here as a third framing in a later phase.
 *
 * Threading: all calls are expected from the engine/RTP clock domain. The
 * DTE-side pull/push callbacks are the boundary to other threads and must
 * be thread-safe themselves (the engine wires them to mutex-guarded rings).
 */

#ifndef DATA_STACK_H
#define DATA_STACK_H

#include <stdint.h>

typedef enum {
    DS_FRAMING_RAW = 0,
    DS_FRAMING_V14 = 1
} ds_framing_t;

/* Pull one DTE byte to transmit; return -1 when none is pending. */
typedef int (*ds_pull_byte_fn)(void *ctx);
/* Push one received DTE byte. */
typedef void (*ds_push_byte_fn)(void *ctx, uint8_t byte);

/* ds_tx_get_bit() returns this when framing is RAW and no data is pending.
 * V.14 framing never returns it (idle is mark). */
#define DS_TX_NO_DATA (-1)

typedef struct {
    ds_framing_t framing;
    ds_pull_byte_fn pull;
    void *pull_ctx;
    ds_push_byte_fn push;
    void *push_ctx;

    /* TX shift register: bit 0 is the next bit on the line.  V.14 keeps
     * start+data here and emits the rate-adapted stop/idle marks separately. */
    uint16_t tx_shift;
    int tx_bits;
    int tx_mark_bits;

    /* V.14 rate adaptation.  Rates are bit/s.  The accumulator schedules
     * the number of synchronous line bits occupied by each 8N1 character. */
    int dte_bit_rate;
    int line_bit_rate;
    uint64_t tx_rate_accum;

    /* RX character assembly */
    int rx_hunting;      /* 1 = waiting for a start bit */
    uint8_t rx_shift;
    int rx_bits;

    /* Statistics */
    uint64_t tx_chars;
    uint64_t rx_chars;
    uint64_t tx_deleted_stop_bits;
    uint64_t tx_extra_mark_bits;
    uint64_t rx_deleted_stop_bits;
    uint64_t rx_invalid_bits;
} data_stack_t;

void ds_init(data_stack_t *s,
             ds_framing_t framing,
             ds_pull_byte_fn pull, void *pull_ctx,
             ds_push_byte_fn push, void *push_ctx);

/* Reset framing state (e.g. on retrain) without dropping callbacks. */
void ds_reset(data_stack_t *s);

/* Configure V.14 rate adaptation.  dte_bit_rate is the virtual asynchronous
 * DTE rate and line_bit_rate is the synchronous datapump bit rate.  Equal
 * rates produce ordinary 8N1 (10 line bits per character).  A modest DTE
 * overspeed deletes stop bits as permitted by V.14; an underspeed inserts
 * additional mark bits.  Non-positive values restore equal-rate 8N1. */
void ds_set_v14_rates(data_stack_t *s, int dte_bit_rate, int line_bit_rate);

/* Line side, bit-oriented (V.22bis / V.34 datapump callbacks).
 * Returns 0/1, or DS_TX_NO_DATA in RAW framing when idle. */
int ds_tx_get_bit(data_stack_t *s);
void ds_rx_put_bit(data_stack_t *s, int bit);

/* Line side, packed-byte oriented (V.90 codeword mapper input/output).
 * Bits are packed LSB-first, matching the datapump byte order. The TX fill
 * always produces len bytes: framed characters while DTE data is pending,
 * idle fill (mark for V.14, zeros for RAW) otherwise. */
void ds_tx_fill_bytes(data_stack_t *s, uint8_t *out, int len);
void ds_rx_push_bytes(data_stack_t *s, const uint8_t *in, int len);

#endif
