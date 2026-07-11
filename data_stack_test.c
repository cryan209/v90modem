/*
 * data_stack_test.c — unit tests for the V.14/raw DTE framing layer.
 */

#include "data_stack.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t tx_fifo[4096];
static int tx_fifo_len = 0;
static int tx_fifo_pos = 0;

static uint8_t rx_sink[4096];
static int rx_sink_len = 0;

static int pull_byte(void *ctx)
{
    (void) ctx;
    if (tx_fifo_pos >= tx_fifo_len)
        return -1;
    return tx_fifo[tx_fifo_pos++];
}

static void push_byte(void *ctx, uint8_t byte)
{
    (void) ctx;
    if (rx_sink_len < (int) sizeof(rx_sink))
        rx_sink[rx_sink_len++] = byte;
}

static void load_tx(const uint8_t *data, int len)
{
    memcpy(tx_fifo, data, (size_t) len);
    tx_fifo_len = len;
    tx_fifo_pos = 0;
    rx_sink_len = 0;
}

static int failures = 0;

#define CHECK(cond, name) \
    do { \
        if (cond) { \
            printf("PASS: %s\n", name); \
        } else { \
            printf("FAIL: %s\n", name); \
            failures++; \
        } \
    } while (0)

/* Round-trip: TX bits from one stack straight into RX of another. */
static void test_v14_roundtrip(void)
{
    data_stack_t tx, rx;
    const uint8_t payload[] = "Hello, V.14 world! \x00\xFF\x55\xAA binary too";

    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    load_tx(payload, (int) sizeof(payload));

    /* Push some idle mark first: RX must stay in hunt. */
    for (int i = 0; i < 64; i++)
        ds_rx_put_bit(&rx, 1);

    for (int i = 0; i < (int) sizeof(payload) * 10 + 64; i++)
        ds_rx_put_bit(&rx, ds_tx_get_bit(&tx));

    CHECK(rx_sink_len == (int) sizeof(payload)
          && memcmp(rx_sink, payload, sizeof(payload)) == 0,
          "V.14 round-trip is byte exact");
    CHECK(tx.tx_chars == sizeof(payload) && rx.rx_chars == sizeof(payload),
          "V.14 round-trip character counters");
}

static void test_v14_idle_is_mark(void)
{
    data_stack_t tx;

    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    load_tx((const uint8_t *) "", 0);
    int all_mark = 1;
    for (int i = 0; i < 100; i++)
        if (ds_tx_get_bit(&tx) != 1)
            all_mark = 0;
    CHECK(all_mark, "V.14 idle line rests at mark");
}

/* A transmitter that deletes stop bits (overspeed): data bits followed
 * immediately by the next start bit. The receiver must resynchronise and
 * still deliver every character. */
static void test_v14_deleted_stop_bits(void)
{
    data_stack_t rx;
    const uint8_t chars[] = { 0x41, 0x42, 0x43, 0x44 };

    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    rx_sink_len = 0;

    for (int i = 0; i < 16; i++)
        ds_rx_put_bit(&rx, 1);
    for (int c = 0; c < (int) sizeof(chars); c++) {
        ds_rx_put_bit(&rx, 0);                       /* start */
        for (int b = 0; b < 8; b++)
            ds_rx_put_bit(&rx, (chars[c] >> b) & 1); /* data */
        if (c == (int) sizeof(chars) - 1)
            ds_rx_put_bit(&rx, 1);                   /* final stop kept */
        /* other stop bits deleted: next start follows immediately */
    }

    CHECK(rx_sink_len == (int) sizeof(chars)
          && memcmp(rx_sink, chars, sizeof(chars)) == 0,
          "V.14 receiver tolerates deleted stop bits");
    CHECK(rx.rx_deleted_stop_bits == sizeof(chars) - 1,
          "V.14 deleted stop bits are counted");
}

static void test_v14_rate_adaptation(void)
{
    data_stack_t tx, rx;
    uint8_t payload[2400];

    for (int i = 0; i < (int)sizeof(payload); i++)
        payload[i] = (uint8_t)(0x30 + i);

    /* 2280 line bit/s for a 2400 bit/s DTE averages 9.5 line bits per
     * character: half the stop bits are deleted. */
    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    ds_set_v14_rates(&tx, 2400, 2280);
    load_tx(payload, (int)sizeof(payload));
    for (int i = 0; i < 22800; i++)
        ds_rx_put_bit(&rx, ds_tx_get_bit(&tx));

    CHECK(rx_sink_len == (int)sizeof(payload)
          && memcmp(rx_sink, payload, sizeof(payload)) == 0,
          "V.14 fractional overspeed remains byte exact");
    CHECK(tx.tx_deleted_stop_bits == 1200
          && rx.rx_deleted_stop_bits == 1200,
          "V.14 fractional overspeed deletes the scheduled stop bits");

    /* A DTE running at half the synchronous line rate needs ten additional
     * mark bits after each normal 8N1 character. */
    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    ds_set_v14_rates(&tx, 1200, 2400);
    load_tx(payload, 1);
    for (int i = 0; i < 20; i++)
        (void)ds_tx_get_bit(&tx);
    CHECK(tx.tx_extra_mark_bits == 10 && tx.tx_chars == 1,
          "V.14 underspeed inserts the scheduled idle marks");
}

static void test_v14_invalid_line_value_resets_receiver(void)
{
    data_stack_t rx;

    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    rx_sink_len = 0;
    ds_rx_put_bit(&rx, 0);
    ds_rx_put_bit(&rx, 1);
    ds_rx_put_bit(&rx, -1);

    CHECK(rx.rx_invalid_bits == 1 && rx.rx_hunting,
          "V.14 invalid line value is counted and resynchronises the receiver");
}

static void test_reset_clears_partial_character_and_rate_phase(void)
{
    data_stack_t tx, rx;
    const uint8_t payload[] = { 0x55, 0xAA };

    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    ds_set_v14_rates(&tx, 2400, 2280);
    load_tx(payload, (int)sizeof(payload));
    for (int i = 0; i < 4; i++)
        ds_rx_put_bit(&rx, ds_tx_get_bit(&tx));

    ds_reset(&tx);
    ds_reset(&rx);
    CHECK(tx.tx_bits == 0 && tx.tx_mark_bits == 0 && tx.tx_rate_accum == 0
          && rx.rx_hunting && rx.rx_bits == 0,
          "data stack reset clears partial framing and rate phase");
}

static void test_raw_roundtrip(void)
{
    data_stack_t tx, rx;
    const uint8_t payload[] = { 0x00, 0x01, 0x7E, 0x80, 0xFF, 0x55 };

    ds_init(&tx, DS_FRAMING_RAW, pull_byte, NULL, NULL, NULL);
    ds_init(&rx, DS_FRAMING_RAW, NULL, NULL, push_byte, NULL);
    load_tx(payload, (int) sizeof(payload));

    for (int i = 0; i < (int) sizeof(payload) * 8; i++) {
        int bit = ds_tx_get_bit(&tx);
        if (bit == DS_TX_NO_DATA)
            break;
        ds_rx_put_bit(&rx, bit);
    }
    CHECK(rx_sink_len == (int) sizeof(payload)
          && memcmp(rx_sink, payload, sizeof(payload)) == 0,
          "RAW round-trip is byte exact");

    CHECK(ds_tx_get_bit(&tx) == DS_TX_NO_DATA,
          "RAW idle reports no data");
}

static void test_packed_byte_helpers(void)
{
    data_stack_t tx, rx;
    const uint8_t payload[] = "packed byte path \x00\x7f\xff test";
    uint8_t line[512];
    int line_bytes = ((int) sizeof(payload) * 10 + 7) / 8 + 8;

    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    load_tx(payload, (int) sizeof(payload));

    ds_tx_fill_bytes(&tx, line, line_bytes);
    ds_rx_push_bytes(&rx, line, line_bytes);

    CHECK(rx_sink_len == (int) sizeof(payload)
          && memcmp(rx_sink, payload, sizeof(payload)) == 0,
          "V.14 packed-byte path is byte exact");

    /* Idle fill must be mark so the far-end receiver stays in hunt. */
    load_tx((const uint8_t *) "", 0);
    ds_tx_fill_bytes(&tx, line, 4);
    CHECK(line[0] == 0xFF && line[3] == 0xFF,
          "V.14 packed idle fill is mark");
}

/* V.90 consumes a non-byte-aligned number of bits per six-symbol frame.  This
 * mirrors its byte reservoir at d=29 and proves packed V.14 state remains
 * continuous across frame boundaries. */
static void test_v14_v90_style_reservoir(void)
{
    enum { PAYLOAD_LEN = 512, FRAME_BITS = 29, MAX_FRAMES = 2048 };
    data_stack_t tx, rx;
    uint8_t payload[PAYLOAD_LEN];
    uint64_t reservoir = 0;
    int reservoir_bits = 0;

    for (int i = 0; i < PAYLOAD_LEN; i++)
        payload[i] = (uint8_t)(i * 61 + 7);
    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    load_tx(payload, PAYLOAD_LEN);

    for (int frame = 0; frame < MAX_FRAMES && rx_sink_len < PAYLOAD_LEN; frame++) {
        uint8_t packed[8];
        uint64_t frame_bits;
        int missing = FRAME_BITS - reservoir_bits;
        int needed = missing > 0 ? (missing + 7) / 8 : 0;

        ds_tx_fill_bytes(&tx, packed, needed);
        for (int i = 0; i < needed; i++) {
            reservoir |= (uint64_t)packed[i] << reservoir_bits;
            reservoir_bits += 8;
        }
        frame_bits = reservoir & ((1ULL << FRAME_BITS) - 1ULL);
        reservoir >>= FRAME_BITS;
        reservoir_bits -= FRAME_BITS;
        for (int i = 0; i < FRAME_BITS; i++)
            ds_rx_put_bit(&rx, (int)((frame_bits >> i) & 1ULL));
    }

    CHECK(rx_sink_len == PAYLOAD_LEN
          && memcmp(rx_sink, payload, PAYLOAD_LEN) == 0,
          "V.14 remains byte exact through V.90-style 29-bit frames");
}

typedef struct {
    uint8_t tx[2048];
    int tx_len;
    int tx_pos;
    uint8_t rx[2048];
    int rx_len;
    bool connected;
    bool xid;
    bool failed;
} lapm_endpoint_t;

static int lapm_pull(void *ctx)
{
    lapm_endpoint_t *ep = (lapm_endpoint_t *)ctx;

    return (ep->tx_pos < ep->tx_len) ? ep->tx[ep->tx_pos++] : -1;
}

static void lapm_push(void *ctx, uint8_t byte)
{
    lapm_endpoint_t *ep = (lapm_endpoint_t *)ctx;

    if (ep->rx_len < (int)sizeof(ep->rx))
        ep->rx[ep->rx_len++] = byte;
}

static void lapm_event(void *ctx, ds_link_event_t event)
{
    lapm_endpoint_t *ep = (lapm_endpoint_t *)ctx;

    if (event == DS_LINK_CONNECTED)
        ep->connected = true;
    else if (event == DS_LINK_XID_NEGOTIATED)
        ep->xid = true;
    else if (event == DS_LINK_ERROR || event == DS_LINK_UNSUPPORTED)
        ep->failed = true;
}

static void test_lapm_data_stack_roundtrip_mode(bool detect, const char *label)
{
    data_stack_t caller;
    data_stack_t answerer;
    lapm_endpoint_t caller_ep;
    lapm_endpoint_t answerer_ep;
    bool caller_initialized;
    bool answerer_initialized;

    memset(&caller_ep, 0, sizeof(caller_ep));
    memset(&answerer_ep, 0, sizeof(answerer_ep));
    memset(&caller, 0, sizeof(caller));
    memset(&answerer, 0, sizeof(answerer));
    caller_ep.tx_len = 1024;
    answerer_ep.tx_len = 1024;
    for (int i = 0; i < 1024; i++) {
        caller_ep.tx[i] = (uint8_t)(i * 29 + 3);
        answerer_ep.tx[i] = (uint8_t)(i * 47 + 11);
    }

    caller_initialized = ds_init_v42(&caller, true, detect, 9600,
                                     lapm_pull, &caller_ep,
                                     lapm_push, &caller_ep,
                                     lapm_event, &caller_ep) == 0;
    answerer_initialized = ds_init_v42(&answerer, false, detect, 9600,
                                       lapm_pull, &answerer_ep,
                                       lapm_push, &answerer_ep,
                                       lapm_event, &answerer_ep) == 0;
    if (caller_initialized && answerer_initialized) {
        for (int tick = 0; tick < 9600 * 10; tick++) {
            ds_rx_put_bit(&answerer, ds_tx_get_bit(&caller));
            ds_rx_put_bit(&caller, ds_tx_get_bit(&answerer));
            if (caller_ep.rx_len == answerer_ep.tx_len
                && answerer_ep.rx_len == caller_ep.tx_len) {
                break;
            }
        }
    }

    CHECK(caller_initialized && answerer_initialized
          && caller_ep.connected && answerer_ep.connected
          && caller_ep.xid && answerer_ep.xid
          && ds_link_is_ready(&caller) && ds_link_is_ready(&answerer)
          && !caller_ep.failed && !answerer_ep.failed
          && caller_ep.rx_len == answerer_ep.tx_len
          && answerer_ep.rx_len == caller_ep.tx_len
          && memcmp(caller_ep.rx, answerer_ep.tx,
                    (size_t)answerer_ep.tx_len) == 0
          && memcmp(answerer_ep.rx, caller_ep.tx,
                    (size_t)caller_ep.tx_len) == 0,
          label);
    if (caller_initialized)
        ds_release(&caller);
    if (answerer_initialized)
        ds_release(&answerer);
}

/* Random payloads through the bit interface with idle gaps between bursts. */
static void test_v14_bursty_random(void)
{
    data_stack_t tx, rx;
    uint8_t payload[1024];
    uint8_t expected[1024];
    int total = 0;

    srand(1234);
    ds_init(&tx, DS_FRAMING_V14, pull_byte, NULL, NULL, NULL);
    ds_init(&rx, DS_FRAMING_V14, NULL, NULL, push_byte, NULL);
    rx_sink_len = 0;

    for (int burst = 0; burst < 8; burst++) {
        int n = 1 + rand() % 128;

        for (int i = 0; i < n; i++)
            payload[i] = (uint8_t) rand();
        memcpy(expected + total, payload, (size_t) n);
        total += n;

        memcpy(tx_fifo, payload, (size_t) n);
        tx_fifo_len = n;
        tx_fifo_pos = 0;

        for (int i = 0; i < n * 10; i++)
            ds_rx_put_bit(&rx, ds_tx_get_bit(&tx));
        /* idle gap */
        for (int i = 0; i < rand() % 40; i++)
            ds_rx_put_bit(&rx, ds_tx_get_bit(&tx));
    }

    CHECK(rx_sink_len == total && memcmp(rx_sink, expected, (size_t) total) == 0,
          "V.14 bursty random payload is byte exact");
}

int main(void)
{
    test_v14_roundtrip();
    test_v14_idle_is_mark();
    test_v14_deleted_stop_bits();
    test_v14_rate_adaptation();
    test_v14_invalid_line_value_resets_receiver();
    test_reset_clears_partial_character_and_rate_phase();
    test_raw_roundtrip();
    test_packed_byte_helpers();
    test_v14_v90_style_reservoir();
    test_lapm_data_stack_roundtrip_mode(true,
          "data stack LAPM detection negotiates and transfers byte-exact payloads");
    test_lapm_data_stack_roundtrip_mode(false,
          "data stack LAPM bypass negotiates and transfers byte-exact payloads");
    test_v14_bursty_random();

    if (failures) {
        printf("data_stack_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("data_stack_test: OK\n");
    return 0;
}
