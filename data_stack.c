/*
 * data_stack.c — V.14 async framing / raw shuttle between DTE bytes and the
 * datapump bit stream. See data_stack.h for the layer contract.
 */

#include "data_stack.h"

#include <string.h>

void ds_init(data_stack_t *s,
             ds_framing_t framing,
             ds_pull_byte_fn pull, void *pull_ctx,
             ds_push_byte_fn push, void *push_ctx)
{
    memset(s, 0, sizeof(*s));
    s->framing = framing;
    s->pull = pull;
    s->pull_ctx = pull_ctx;
    s->push = push;
    s->push_ctx = push_ctx;
    s->rx_hunting = 1;
}

void ds_reset(data_stack_t *s)
{
    s->tx_shift = 0;
    s->tx_bits = 0;
    s->rx_hunting = 1;
    s->rx_shift = 0;
    s->rx_bits = 0;
}

/* Load the next DTE byte into the TX shift register. Returns 0 if no byte
 * was pending. */
static int ds_tx_load(data_stack_t *s)
{
    int byte;

    if (!s->pull)
        return 0;
    byte = s->pull(s->pull_ctx);
    if (byte < 0)
        return 0;

    if (s->framing == DS_FRAMING_V14) {
        /* start(0), data LSB-first, stop(1): 10 bits, bit 0 first */
        s->tx_shift = (uint16_t) (0x200u | ((unsigned) (byte & 0xFF) << 1));
        s->tx_bits = 10;
    } else {
        s->tx_shift = (uint16_t) (byte & 0xFF);
        s->tx_bits = 8;
    }
    s->tx_chars++;
    return 1;
}

int ds_tx_get_bit(data_stack_t *s)
{
    int bit;

    if (s->tx_bits == 0 && !ds_tx_load(s)) {
        /* Idle: async lines rest at mark; RAW keeps the legacy
         * "no data" signal so the caller can transmit silence. */
        return (s->framing == DS_FRAMING_V14) ? 1 : DS_TX_NO_DATA;
    }
    bit = s->tx_shift & 1;
    s->tx_shift >>= 1;
    s->tx_bits--;
    return bit;
}

void ds_rx_put_bit(data_stack_t *s, int bit)
{
    bit &= 1;

    if (s->framing == DS_FRAMING_RAW) {
        s->rx_shift |= (uint8_t) (bit << s->rx_bits);
        if (++s->rx_bits == 8) {
            if (s->push)
                s->push(s->push_ctx, s->rx_shift);
            s->rx_chars++;
            s->rx_shift = 0;
            s->rx_bits = 0;
        }
        return;
    }

    /* V.14 */
    if (s->rx_hunting) {
        if (bit == 0) {
            /* start bit */
            s->rx_hunting = 0;
            s->rx_shift = 0;
            s->rx_bits = 0;
        }
        return;
    }

    if (s->rx_bits < 8) {
        s->rx_shift |= (uint8_t) (bit << s->rx_bits);
        if (++s->rx_bits < 8)
            return;
        return;
    }

    /* Stop-bit position. V.14 transmitters may delete stop bits under
     * overspeed, so a 0 here is the start bit of the next character, not a
     * framing error. */
    if (s->push)
        s->push(s->push_ctx, s->rx_shift);
    s->rx_chars++;
    if (bit == 1) {
        s->rx_hunting = 1;
    } else {
        s->rx_deleted_stop_bits++;
        s->rx_hunting = 0;
        s->rx_shift = 0;
        s->rx_bits = 0;
    }
}

void ds_tx_fill_bytes(data_stack_t *s, uint8_t *out, int len)
{
    for (int i = 0; i < len; i++) {
        uint8_t byte = 0;

        for (int b = 0; b < 8; b++) {
            int bit = ds_tx_get_bit(s);

            if (bit == DS_TX_NO_DATA)
                bit = 0;
            byte |= (uint8_t) ((bit & 1) << b);
        }
        out[i] = byte;
    }
}

void ds_rx_push_bytes(data_stack_t *s, const uint8_t *in, int len)
{
    for (int i = 0; i < len; i++) {
        for (int b = 0; b < 8; b++)
            ds_rx_put_bit(s, (in[i] >> b) & 1);
    }
}
