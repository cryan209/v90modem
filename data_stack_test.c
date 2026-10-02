/*
 * data_stack_test.c — unit tests for the V.14/raw DTE framing layer.
 */

#include "data_stack.h"
#include <spandsp.h>

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

static void test_lapm_data_stack_case(bool detect, int offer, int peer_offer,
                                      bool repeated, bool corrupt, int dictionary, const char *label)
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
        caller_ep.tx[i] = (uint8_t)(repeated ? "ABCD"[i % 4] : (i * 29 + 3));
        answerer_ep.tx[i] = (uint8_t)(repeated ? "xyz0"[i % 4] : (i * 47 + 11));
    }

    caller_initialized = ds_init_v42_ex(&caller, true, detect, 9600, offer, dictionary > 2048 ? dictionary : 2048, 64,
                                     lapm_pull, &caller_ep,
                                     lapm_push, &caller_ep,
                                     lapm_event, &caller_ep) == 0;
    answerer_initialized = ds_init_v42_ex(&answerer, false, detect, 9600, peer_offer, dictionary, 32,
                                       lapm_pull, &answerer_ep,
                                       lapm_push, &answerer_ep,
                                       lapm_event, &answerer_ep) == 0;
    if (caller_initialized && answerer_initialized) {
        for (int tick = 0; tick < 9600 * 10; tick++) {
            int bit = ds_tx_get_bit(&caller);
            if (corrupt && caller.link_ready && tick % 1703 == 0)
                bit ^= 1;
            ds_rx_put_bit(&answerer, bit);
            ds_rx_put_bit(&caller, ds_tx_get_bit(&answerer));
            if (caller_ep.rx_len == answerer_ep.tx_len
                && answerer_ep.rx_len == caller_ep.tx_len) {
                break;
            }
        }
    }

    if (getenv("DS_TEST_DEBUG"))
        printf("debug %s: conn %d %d xid %d %d failed %d %d rx %d/%d %d/%d\n", label,
               caller_ep.connected, answerer_ep.connected, caller_ep.xid, answerer_ep.xid,
               caller_ep.failed, answerer_ep.failed, caller_ep.rx_len, answerer_ep.tx_len,
               answerer_ep.rx_len, caller_ep.tx_len);
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
    if (caller_initialized && answerer_initialized)
    {
        v42_negotiated_parameters_t cp, ap;
        CHECK(v42_get_negotiated_parameters(caller.v42, &cp) == 0
              && v42_get_negotiated_parameters(answerer.v42, &ap) == 0
              && cp.compression_p0 == (offer & peer_offer)
              && ap.compression_p0 == cp.compression_p0
              && cp.compression_p1 == dictionary && ap.compression_p1 == dictionary
              && cp.compression_p2 == 32 && ap.compression_p2 == 32,
              "V.42bis directions and smaller dictionary/string limits agree");
        if (repeated && (offer & peer_offer))
        {
            CHECK(!(offer & peer_offer & 1) || caller.v42_tx_wire_bytes < 512,
                  "initiator compression reduces application bytes on the wire");
            CHECK(!(offer & peer_offer & 2) || answerer.v42_tx_wire_bytes < 512,
                  "responder compression reduces application bytes on the wire");
        }
        /* A peer may re-establish without XID. Feed its fresh transparent
           compression stream through accepted I-frames, independently of our
           old peer endpoint (which still has the previous sequence state). */
        const uint8_t sabme[] = {1, 0x7f};
        lapm_receive(caller.v42, sabme, sizeof(sabme), 1);
        caller_ep.rx_len = 0;
        const uint8_t fresh[] = "fresh-session-after-SABME";
        uint8_t iframe[3 + sizeof(fresh) - 1] = {1, 0, 0};
        memcpy(iframe + 3, fresh, sizeof(fresh) - 1);
        lapm_receive(caller.v42, iframe, sizeof(iframe), 1);
        CHECK(caller_ep.rx_len == sizeof(fresh) - 1
              && memcmp(caller_ep.rx, fresh, sizeof(fresh) - 1) == 0,
              "V.42bis C-INIT on SABME without a new XID");
        /* New LAPM establishment must initialize a fresh codec/dictionary. */
        caller_ep.tx_pos = answerer_ep.tx_pos = 0;
        caller_ep.rx_len = answerer_ep.rx_len = 0;
        ds_reset(&caller);
        ds_reset(&answerer);
        for (int tick = 0; tick < 9600 * 10; tick++)
        {
            ds_rx_put_bit(&answerer, ds_tx_get_bit(&caller));
            ds_rx_put_bit(&caller, ds_tx_get_bit(&answerer));
            if (caller_ep.rx_len == 1024 && answerer_ep.rx_len == 1024)
                break;
        }
        CHECK(caller_ep.rx_len == 1024 && answerer_ep.rx_len == 1024
              && memcmp(caller_ep.rx, answerer_ep.tx, 1024) == 0
              && memcmp(answerer_ep.rx, caller_ep.tx, 1024) == 0,
              "V.42bis dictionaries restart with a new LAPM session");
    }
    if (caller_initialized)
        ds_release(&caller);
    if (answerer_initialized)
        ds_release(&answerer);
}

static int malformed_peer_frame(void *ctx, uint8_t *msg, int max_len)
{
    int n = 0;
    while (n < max_len)
    {
        int byte = lapm_pull(ctx);
        if (byte < 0)
            break;
        msg[n++] = (uint8_t)byte;
    }
    return n;
}

/* A CRC-valid LAPM I-frame can still contain invalid compression data. */
static void test_compression_error(void)
{
    data_stack_t caller;
    lapm_endpoint_t caller_ep = {0}, peer_ep = {0};
    /* ECM, then nine-bit code 291, absent from the fresh dictionary. */
    static const uint8_t invalid[] = {0, 0, 0x23, 0x01};
    memcpy(peer_ep.tx, invalid, sizeof(invalid));
    peer_ep.tx_len = sizeof(invalid);
    v42_state_t *peer = v42_init(NULL, false, false, malformed_peer_frame, NULL, &peer_ep);
    bool initialized = ds_init_v42(&caller, true, false, 9600,
                                   lapm_pull, &caller_ep, lapm_push, &caller_ep,
                                   lapm_event, &caller_ep) == 0;
    if (peer && initialized)
    {
        v42_set_compression(peer, 3, 512, 32);
        v42_restart(peer);
        for (int tick = 0; tick < 9600 * 5 && !caller_ep.failed; tick++)
        {
            v42_rx_bit(peer, ds_tx_get_bit(&caller));
            ds_rx_put_bit(&caller, v42_tx_bit(peer));
        }
    }
    CHECK(peer && initialized && caller_ep.failed && caller.compression_failed
          && !ds_link_is_ready(&caller) && caller_ep.rx_len == 0,
          "invalid compressed data reports link error without fabricated output");
    if (initialized && caller.v42bis)
    {
        const uint8_t sabme[] = {1, 0x7f};
        const uint8_t fresh[] = {1, 0, 0, 'O', 'K'};
        lapm_receive(caller.v42, sabme, sizeof(sabme), 1);
        lapm_receive(caller.v42, fresh, sizeof(fresh), 1);
        CHECK(!caller.compression_failed && ds_link_is_ready(&caller)
              && caller_ep.rx_len == 2 && !memcmp(caller_ep.rx, "OK", 2),
              "V.42bis C-ERROR recovers only through fresh link C-INIT");
    }
    if (initialized)
        ds_release(&caller);
    if (peer)
        v42_free(peer);
}

static void test_v44_stack(int caller_direction, int answerer_direction,
                            bool detect, bool corrupt, bool plain_peer, const char *label)
{
    data_stack_t caller = {0}, answerer = {0};
    lapm_endpoint_t ce = {0}, ae = {0};
    v42_v44_parameters_t cp = {0, caller_direction, 1024, 768, 64, 48, 3072, 2048};
    v42_v44_parameters_t ap = {0, answerer_direction, 512, 2048, 32, 64, 1024, 4096};
    ce.tx_len = ae.tx_len = 1024;
    for (int i = 0; i < 1024; i++) {
        ce.tx[i] = (uint8_t)"ABCDEFGH"[i % 8];
        ae.tx[i] = (uint8_t)"01234567"[i % 8];
    }
    bool ci = ds_init_v44(&caller, true, detect, 9600, &cp,
                           lapm_pull,&ce,lapm_push,&ce,lapm_event,&ce) == 0;
    bool ai = plain_peer
        ? ds_init_v42_ex(&answerer,false,detect,9600,0,512,6,
                          lapm_pull,&ae,lapm_push,&ae,lapm_event,&ae) == 0
        : ds_init_v44(&answerer,false,detect,9600,&ap,
                      lapm_pull,&ae,lapm_push,&ae,lapm_event,&ae) == 0;
    if (ci && ai) {
        for (int tick = 0; tick < 9600 * 10; tick++) {
            int bit = ds_tx_get_bit(&caller);
            if (corrupt && caller.link_ready && tick % 1703 == 0) bit ^= 1;
            ds_rx_put_bit(&answerer,bit);
            ds_rx_put_bit(&caller,ds_tx_get_bit(&answerer));
            if (ce.rx_len == 1024 && ae.rx_len == 1024) break;
        }
    }
    CHECK(ci && ai && !ce.failed && !ae.failed && ce.connected && ae.connected
          && ce.rx_len == 1024 && ae.rx_len == 1024
          && memcmp(ce.rx,ae.tx,1024) == 0 && memcmp(ae.rx,ce.tx,1024) == 0, label);
    if (ci && ai) {
        v42_negotiated_parameters_t cn = {0}, an = {0};
        bool got = v42_get_negotiated_parameters(caller.v42,&cn) == 0
                   && v42_get_negotiated_parameters(answerer.v42,&an) == 0;
        int cd = plain_peer ? 0 : caller_direction
            & (((answerer_direction & 1) << 1) | ((answerer_direction & 2) >> 1));
        int ad = ((cd & 1) << 1) | ((cd & 2) >> 1);
        CHECK(got && cn.compression_p0 == 0 && an.compression_p0 == 0
              && (plain_peer ? !cn.v44_valid && !an.v44_valid
                  : cn.v44_valid && an.v44_valid && cn.v44.directions == cd
                    && an.v44.directions == ad && cn.v44.tx_codewords == 1024
                    && cn.v44.rx_codewords == 512 && cn.v44.tx_max_string == 64
                    && cn.v44.rx_max_string == 32 && cn.v44.tx_history == 3072
                    && cn.v44.rx_history == 1024
                    && cn.v44.tx_codewords == an.v44.rx_codewords
                    && cn.v44.rx_codewords == an.v44.tx_codewords
                    && cn.v44.tx_history == an.v44.rx_history
                    && cn.v44.rx_history == an.v44.tx_history),
              "V.44 negotiates complementary directions and asymmetric limits");
        CHECK(!(cd & 1) || caller.v42_tx_wire_bytes < 512,
              "V.44 caller compression reduces wire bytes");
        CHECK(!(ad & 1) || answerer.v42_tx_wire_bytes < 512,
              "V.44 answerer compression reduces wire bytes");
        /* Idle gaps and physical rate changes preserve the existing dictionaries. */
        for (int tick = 0; tick < 4096; tick++) {
            ds_rx_put_bit(&answerer,ds_tx_get_bit(&caller));
            ds_rx_put_bit(&caller,ds_tx_get_bit(&answerer));
        }
        v42_set_bit_rate(caller.v42,19200);
        v42_set_bit_rate(answerer.v42,19200);
        ce.tx_pos = ae.tx_pos = ce.rx_len = ae.rx_len = 0;
        for (int tick = 0; tick < 19200 * 5; tick++) {
            ds_rx_put_bit(&answerer,ds_tx_get_bit(&caller));
            ds_rx_put_bit(&caller,ds_tx_get_bit(&answerer));
            if (ce.rx_len == 1024 && ae.rx_len == 1024) break;
        }
        CHECK(ce.rx_len == 1024 && ae.rx_len == 1024
              && memcmp(ce.rx,ae.tx,1024) == 0 && memcmp(ae.rx,ce.tx,1024) == 0,
              "V.44 preserves dictionary context across idle and line-rate changes");
        ce.tx_pos = ae.tx_pos = ce.rx_len = ae.rx_len = 0;
        ds_reset(&caller); ds_reset(&answerer);
        for (int tick = 0; tick < 19200 * 10; tick++) {
            ds_rx_put_bit(&answerer,ds_tx_get_bit(&caller));
            ds_rx_put_bit(&caller,ds_tx_get_bit(&answerer));
            if (ce.rx_len == 1024 && ae.rx_len == 1024) break;
        }
        CHECK(ce.rx_len == 1024 && ae.rx_len == 1024
              && memcmp(ce.rx,ae.tx,1024) == 0 && memcmp(ae.rx,ce.tx,1024) == 0,
              "V.44 C-INIT restarts dictionaries with a new LAPM session");
    }
    if (ci) ds_release(&caller);
    if (ai) ds_release(&answerer);
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

/* Numbered lines, as the live soak sends: compress well, like most DTE text. */
typedef struct {
    uint32_t tx_line;
    int tx_pos;
    char tx_buf[16];
    uint32_t rx_line;
    int rx_pos;
    char rx_buf[16];
    uint64_t rx_bytes;
    bool rx_bad;
    bool connected;
} lines_endpoint_t;

static int lines_pull(void *ctx)
{
    lines_endpoint_t *ep = (lines_endpoint_t *)ctx;

    if (ep->tx_pos == 0)
        snprintf(ep->tx_buf, sizeof(ep->tx_buf), "D%07u\r\n", (unsigned)ep->tx_line);
    {
        int c = (uint8_t)ep->tx_buf[ep->tx_pos++];

        if (ep->tx_pos == 10) {
            ep->tx_pos = 0;
            ep->tx_line++;
        }
        return c;
    }
}

static void lines_push(void *ctx, uint8_t byte)
{
    lines_endpoint_t *ep = (lines_endpoint_t *)ctx;

    if (ep->rx_pos == 0)
        snprintf(ep->rx_buf, sizeof(ep->rx_buf), "D%07u\r\n", (unsigned)ep->rx_line);
    if ((char)byte != ep->rx_buf[ep->rx_pos])
        ep->rx_bad = true;
    if (++ep->rx_pos == 10) {
        ep->rx_pos = 0;
        ep->rx_line++;
    }
    ep->rx_bytes++;
}

static void lines_event(void *ctx, ds_link_event_t event)
{
    lines_endpoint_t *ep = (lines_endpoint_t *)ctx;

    if (event == DS_LINK_CONNECTED)
        ep->connected = true;
}

/* V.42bis must fill I-frames.  LAPM's window is k FRAMES, and with a round
   trip longer than k frame times the window, not the line, sets the rate.
   A frame carrying the compressed form of only 128 DTE octets then caps the
   DTE rate at k*128 octets per round trip -- compression buys nothing --
   which is the ~20 kbit/s both ways a live 54666/31200 V.90 call delivered.
   Run both directions saturated through a 400 ms delay line each way. */
static void test_v42bis_fills_frames_on_long_round_trip(void)
{
    enum { RATE = 56000, ONE_WAY_BITS = RATE*400/1000, SECONDS = 20 };
    static uint8_t down_line[ONE_WAY_BITS], up_line[ONE_WAY_BITS];
    data_stack_t caller, answerer;
    lines_endpoint_t cep, aep;
    int pos = 0;
    uint64_t start_c = 0, start_a = 0;
    bool measuring = false;
    double rtt_s, window_bound, caller_rate, answerer_rate;

    memset(&caller, 0, sizeof(caller));
    memset(&answerer, 0, sizeof(answerer));
    memset(&cep, 0, sizeof(cep));
    memset(&aep, 0, sizeof(aep));
    memset(down_line, 1, sizeof(down_line));
    memset(up_line, 1, sizeof(up_line));
    if (ds_init_v42_ex(&caller, true, false, RATE, 3, 2048, 32,
                       lines_pull, &cep, lines_push, &cep, lines_event, &cep) != 0
        || ds_init_v42_ex(&answerer, false, false, RATE, 3, 2048, 32,
                          lines_pull, &aep, lines_push, &aep, lines_event, &aep) != 0) {
        CHECK(false, "V.42bis long round trip: init");
        return;
    }
    for (long tick = 0; tick < (long)RATE*(SECONDS + 5); tick++) {
        int cb = ds_tx_get_bit(&caller);
        int ab = ds_tx_get_bit(&answerer);

        ds_rx_put_bit(&answerer, up_line[pos]);
        ds_rx_put_bit(&caller, down_line[pos]);
        up_line[pos] = (uint8_t)cb;
        down_line[pos] = (uint8_t)ab;
        if (++pos == ONE_WAY_BITS)
            pos = 0;
        if (!measuring && cep.connected && aep.connected && tick >= RATE*3) {
            measuring = true;
            start_c = cep.rx_bytes;
            start_a = aep.rx_bytes;
            tick = 0;   /* measure exactly SECONDS from here */
            continue;
        }
        if (measuring && tick == (long)RATE*SECONDS)
            break;
    }
    caller_rate = (double)(cep.rx_bytes - start_c)/SECONDS;
    answerer_rate = (double)(aep.rx_bytes - start_a)/SECONDS;
    /* What 15 frames of 128 DTE octets per round trip would allow. */
    rtt_s = 0.8 + 131.0*8/RATE;
    window_bound = 15.0*128/rtt_s;
    printf("V.42bis 400 ms each way: %.0f and %.0f DTE octets/s "
           "(uncompressed-window bound %.0f, line %d)\n",
           caller_rate, answerer_rate, window_bound, RATE/8);
    CHECK(measuring && !cep.rx_bad && !aep.rx_bad
          && caller_rate > 2.0*window_bound && answerer_rate > 2.0*window_bound,
          "V.42bis fills I-frames: compression multiplies window-limited throughput");
    ds_release(&caller);
    ds_release(&answerer);
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
    test_lapm_data_stack_case(true, 3, 3, false, false, 512,
          "data stack LAPM detection negotiates and transfers byte-exact payloads");
    test_lapm_data_stack_case(false, 3, 3, false, false, 512,
          "data stack LAPM bypass negotiates and transfers byte-exact payloads");
    test_lapm_data_stack_case(false, 3, 3, true, true, 512,
          "compressed LAPM retries preserve dictionaries and exact payloads");
    test_lapm_data_stack_case(false, 1, 3, true, false, 512,
          "V.42bis initiator-only compression transfers both directions");
    test_lapm_data_stack_case(false, 2, 3, true, false, 512,
          "V.42bis responder-only compression transfers both directions");
    test_lapm_data_stack_case(false, 3, 0, true, false, 512,
          "V.42bis refusal falls back to plain LAPM in both directions");
    test_lapm_data_stack_case(false, 3, 3, true, false, 65535,
          "V.42bis full two-octet P1 negotiation and transfer");
    test_compression_error();
    test_v44_stack(3,3,true,false,false,"V.44 detection and duplex compressed transfer");
    test_v44_stack(3,3,false,true,false,"V.44 retransmission preserves dictionary synchronization");
    test_v44_stack(1,3,false,false,false,"V.44 caller-only compression");
    test_v44_stack(2,3,false,false,false,"V.44 answerer-only compression");
    test_v44_stack(3,0,false,false,false,"V.44 direction refusal uses plain LAPM");
    test_v44_stack(3,3,false,false,true,"V.44 unsupported peer falls back to plain LAPM");
    test_v14_bursty_random();
    test_v42bis_fills_frames_on_long_round_trip();

    if (failures) {
        printf("data_stack_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("data_stack_test: OK\n");
    return 0;
}
