/*
 * data_stack.c — V.14 async framing / raw shuttle between DTE bytes and the
 * datapump bit stream. See data_stack.h for the layer contract.
 */

#include "data_stack.h"

#include <spandsp.h>

#include <string.h>

static int ds_v42_get_frame(void *user_data, uint8_t *msg, int max_len)
{
    data_stack_t *s = (data_stack_t *)user_data;
    int len = 0;

    while (len < max_len && s->pull) {
        int byte = s->pull(s->pull_ctx);

        if (byte < 0)
            break;
        msg[len++] = (uint8_t)byte;
    }
    return len;
}

static void ds_v42_put_frame(void *user_data, const uint8_t *msg, int len)
{
    data_stack_t *s = (data_stack_t *)user_data;

    if (!msg || len <= 0 || !s->push)
        return;
    for (int i = 0; i < len; i++)
        s->push(s->push_ctx, msg[i]);
}

static void ds_v42_status(void *user_data, int status)
{
    data_stack_t *s = (data_stack_t *)user_data;
    ds_link_event_t event;
    bool report = true;

    switch (status) {
    case V42_STATUS_DETECTING:
        event = DS_LINK_DETECTING;
        break;
    case V42_STATUS_DETECTION_SUCCEEDED:
        event = DS_LINK_DETECTED;
        break;
    case V42_STATUS_XID_NEGOTIATED:
        event = DS_LINK_XID_NEGOTIATED;
        break;
    case V42_STATUS_DETECTION_UNSUPPORTED:
        s->link_ready = false;
        event = DS_LINK_UNSUPPORTED;
        break;
    case SIG_STATUS_LINK_CONNECTED:
        s->link_ready = true;
        event = DS_LINK_CONNECTED;
        break;
    case SIG_STATUS_LINK_DISCONNECTED:
        s->link_ready = false;
        event = DS_LINK_DISCONNECTED;
        break;
    case SIG_STATUS_LINK_ERROR:
        s->link_ready = false;
        event = DS_LINK_ERROR;
        break;
    default:
        report = false;
        break;
    }
    if (report && s->link_event)
        s->link_event(s->link_event_ctx, event);
}

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
    s->dte_bit_rate = 1;
    s->line_bit_rate = 1;
}

int ds_init_v42(data_stack_t *s,
                bool calling_party,
                bool detect,
                int line_bit_rate,
                ds_pull_byte_fn pull, void *pull_ctx,
                ds_push_byte_fn push, void *push_ctx,
                ds_link_event_fn link_event, void *link_event_ctx)
{
    ds_init(s, DS_FRAMING_V42, pull, pull_ctx, push, push_ctx);
    s->link_event = link_event;
    s->link_event_ctx = link_event_ctx;
    s->v42 = v42_init(NULL,
                      calling_party,
                      detect,
                      ds_v42_get_frame,
                      ds_v42_put_frame,
                      s);
    if (!s->v42)
        return -1;
    v42_set_status_callback(s->v42, ds_v42_status, s);
    if (v42_set_bit_rate(s->v42, line_bit_rate) != 0) {
        v42_free(s->v42);
        s->v42 = NULL;
        return -1;
    }
    s->line_bit_rate = line_bit_rate;
    v42_restart(s->v42);
    return 0;
}

void ds_release(data_stack_t *s)
{
    if (s && s->v42) {
        v42_free(s->v42);
        s->v42 = NULL;
    }
    if (s)
        s->link_ready = false;
}

bool ds_link_is_ready(const data_stack_t *s)
{
    return s && (s->framing != DS_FRAMING_V42 || s->link_ready);
}

void ds_stop_link(data_stack_t *s)
{
    if (s && s->v42)
        v42_stop(s->v42);
}

void ds_reset(data_stack_t *s)
{
    s->tx_shift = 0;
    s->tx_bits = 0;
    s->tx_mark_bits = 0;
    s->tx_rate_accum = 0;
    s->rx_hunting = 1;
    s->rx_shift = 0;
    s->rx_bits = 0;
    s->link_ready = false;
    if (s->v42)
        v42_restart(s->v42);
}

void ds_set_v14_rates(data_stack_t *s, int dte_bit_rate, int line_bit_rate)
{
    if (dte_bit_rate <= 0 || line_bit_rate <= 0) {
        dte_bit_rate = 1;
        line_bit_rate = 1;
    }
    s->dte_bit_rate = dte_bit_rate;
    s->line_bit_rate = line_bit_rate;
    s->tx_rate_accum = 0;
}

/* Number of synchronous line bits allocated to the next 8N1 character.
 * V.14 can reduce a character from 10 to 9 bits by deleting its stop bit.
 * Ratios requiring fewer than 9 bits cannot be represented; the DTE queue
 * must provide the remaining rate buffering/flow control. */
static int ds_v14_character_line_bits(data_stack_t *s)
{
    uint64_t scaled;
    uint64_t bits;

    scaled = s->tx_rate_accum + 10u*(uint64_t)s->line_bit_rate;
    bits = scaled/(uint64_t)s->dte_bit_rate;
    s->tx_rate_accum = scaled%(uint64_t)s->dte_bit_rate;
    if (bits < 9)
        bits = 9;
    if (bits > 1000000)
        bits = 1000000;
    return (int)bits;
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
        int line_bits = ds_v14_character_line_bits(s);

        /* start(0), data LSB-first: 9 fixed bits, bit 0 first.  The stop
         * mark and any rate-adaptation marks follow from tx_mark_bits. */
        s->tx_shift = (uint16_t) ((unsigned) (byte & 0xFF) << 1);
        s->tx_bits = 9;
        s->tx_mark_bits = line_bits - 9;
        if (s->tx_mark_bits == 0)
            s->tx_deleted_stop_bits++;
        else if (s->tx_mark_bits > 1)
            s->tx_extra_mark_bits += (uint64_t)(s->tx_mark_bits - 1);
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

    if (s->framing == DS_FRAMING_V42)
        return s->v42 ? v42_tx_bit(s->v42) : 1;
    if (s->framing == DS_FRAMING_V14 && s->tx_bits == 0
        && s->tx_mark_bits > 0) {
        s->tx_mark_bits--;
        return 1;
    }
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
    if (bit != 0 && bit != 1) {
        s->rx_invalid_bits++;
        s->rx_hunting = 1;
        s->rx_shift = 0;
        s->rx_bits = 0;
        return;
    }
    bit &= 1;

    if (s->framing == DS_FRAMING_V42) {
        if (s->v42)
            v42_rx_bit(s->v42, bit);
        return;
    }

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
