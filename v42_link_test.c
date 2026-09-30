/*
 * v42_link_test.c — deterministic two-ended tests for the vendored LAPM core.
 *
 * This intentionally uses only the public SpanDSP V.42 API.  Application
 * integration must not depend on spandsp/private/v42.h state values.
 */

#include <spandsp.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PAYLOAD_MAX 4096

typedef struct {
    uint8_t tx[PAYLOAD_MAX];
    int tx_len;
    int tx_pos;
    uint8_t rx[PAYLOAD_MAX];
    int rx_len;
    bool detecting;
    bool detection_succeeded;
    bool detection_unsupported;
    bool xid_negotiated;
    bool connected;
    bool disconnected;
    bool link_error;
    uint64_t tick;
    uint64_t unsupported_at;
} endpoint_t;

static int failures;

#define CHECK(condition, label) \
    do { \
        if (condition) { \
            printf("PASS: %s\n", label); \
        } else { \
            printf("FAIL: %s\n", label); \
            failures++; \
        } \
    } while (0)

static int get_payload(void *user_data, uint8_t *msg, int max_len)
{
    endpoint_t *ep = (endpoint_t *)user_data;
    int available = ep->tx_len - ep->tx_pos;

    if (available > max_len)
        available = max_len;
    if (available <= 0)
        return 0;
    memcpy(msg, ep->tx + ep->tx_pos, (size_t)available);
    ep->tx_pos += available;
    return available;
}

static void put_payload(void *user_data, const uint8_t *msg, int len)
{
    endpoint_t *ep = (endpoint_t *)user_data;
    int room;

    if (len <= 0 || msg == NULL)
        return;
    room = PAYLOAD_MAX - ep->rx_len;
    if (len > room)
        len = room;
    memcpy(ep->rx + ep->rx_len, msg, (size_t)len);
    ep->rx_len += len;
}

static void status_changed(void *user_data, int status)
{
    endpoint_t *ep = (endpoint_t *)user_data;

    switch (status) {
    case V42_STATUS_DETECTING:
        ep->detecting = true;
        break;
    case V42_STATUS_DETECTION_SUCCEEDED:
        ep->detection_succeeded = true;
        break;
    case V42_STATUS_DETECTION_UNSUPPORTED:
        ep->detection_unsupported = true;
        ep->unsupported_at = ep->tick;
        break;
    case V42_STATUS_XID_NEGOTIATED:
        ep->xid_negotiated = true;
        break;
    case SIG_STATUS_LINK_CONNECTED:
        ep->connected = true;
        break;
    case SIG_STATUS_LINK_DISCONNECTED:
        ep->connected = false;
        ep->disconnected = true;
        break;
    case SIG_STATUS_LINK_ERROR:
        ep->link_error = true;
        break;
    default:
        break;
    }
}

static void init_payload(endpoint_t *ep, int len, uint32_t seed)
{
    memset(ep, 0, sizeof(*ep));
    ep->tx_len = len;
    for (int i = 0; i < len; i++) {
        seed = seed*1664525U + 1013904223U;
        ep->tx[i] = (uint8_t)(seed >> 24);
    }
}

static bool run_link_case(int bit_rate,
                          int payload_len,
                          int flip_period,
                          bool exercise_busy,
                          bool exercise_release)
{
    endpoint_t caller_ep;
    endpoint_t answerer_ep;
    v42_state_t *caller = NULL;
    v42_state_t *answerer = NULL;
    v42_negotiated_parameters_t caller_xid;
    v42_negotiated_parameters_t answerer_xid;
    uint64_t max_ticks = (uint64_t)bit_rate*20U;
    uint64_t busy_release_tick = 0;
    bool busy_started = false;
    bool busy_released = false;
    bool far_busy_seen = false;
    bool release_started = false;
    bool ok = false;

    init_payload(&caller_ep, payload_len, 0xC011CA11U);
    init_payload(&answerer_ep, payload_len, 0xA45E42E2U);
    caller = v42_init(NULL, true, true,
                      get_payload, put_payload, &caller_ep);
    answerer = v42_init(NULL, false, true,
                        get_payload, put_payload, &answerer_ep);
    if (!caller || !answerer)
        goto done;
    v42_set_status_callback(caller, status_changed, &caller_ep);
    v42_set_status_callback(answerer, status_changed, &answerer_ep);
    if (v42_set_bit_rate(caller, bit_rate) != 0
        || v42_set_bit_rate(answerer, bit_rate) != 0
        || v42_get_bit_rate(caller) != bit_rate
        || v42_get_bit_rate(answerer) != bit_rate) {
        goto done;
    }
    v42_restart(caller);
    v42_restart(answerer);

    for (uint64_t tick = 1; tick <= max_ticks; tick++) {
        int caller_bit;
        int answerer_bit;
        bool inject = flip_period > 0
                   && caller_ep.connected && answerer_ep.connected;

        caller_ep.tick = tick;
        answerer_ep.tick = tick;
        caller_bit = v42_tx_bit(caller);
        if (inject && tick%(uint64_t)flip_period == 0 && caller_bit >= 0)
            caller_bit ^= 1;
        v42_rx_bit(answerer, caller_bit);

        answerer_bit = v42_tx_bit(answerer);
        if (inject && tick%(uint64_t)(flip_period + 137) == 0
            && answerer_bit >= 0) {
            answerer_bit ^= 1;
        }
        v42_rx_bit(caller, answerer_bit);

        if (exercise_busy && caller_ep.connected && answerer_ep.connected
            && !busy_started && answerer_ep.rx_len >= 128) {
            v42_set_local_busy_status(answerer, true);
            busy_started = true;
            /* Hold through T401 so the peer polls and observes RNR. */
            busy_release_tick = tick + ((uint64_t)bit_rate*3U)/2U;
        }
        if (busy_started && v42_get_far_busy_status(caller))
            far_busy_seen = true;
        if (busy_started && !busy_released && tick >= busy_release_tick) {
            v42_set_local_busy_status(answerer, false);
            busy_released = true;
        }

        if (caller_ep.rx_len == payload_len
            && answerer_ep.rx_len == payload_len) {
            if (!exercise_release)
                break;
            if (!release_started) {
                v42_stop(caller);
                release_started = true;
            }
            if (caller_ep.disconnected && answerer_ep.disconnected)
                break;
        }
    }

    memset(&caller_xid, 0, sizeof(caller_xid));
    memset(&answerer_xid, 0, sizeof(answerer_xid));
    ok = caller_ep.detecting && answerer_ep.detecting
      && caller_ep.detection_succeeded && answerer_ep.detection_succeeded
      && caller_ep.xid_negotiated && answerer_ep.xid_negotiated
      && v42_get_negotiated_parameters(caller, &caller_xid) == 0
      && v42_get_negotiated_parameters(answerer, &answerer_xid) == 0
      && caller_xid.valid && answerer_xid.valid
      && caller_xid.tx_n401 == 128 && caller_xid.rx_n401 == 128
      && answerer_xid.tx_n401 == 128 && answerer_xid.rx_n401 == 128
      && caller_xid.tx_window_size_k == 15
      && answerer_xid.tx_window_size_k == 15
      && caller_xid.compression_p0 == 0
      && answerer_xid.compression_p0 == 0
      && answerer_xid.compression_p1 == 512
      && answerer_xid.compression_p2 == 6
      && (exercise_release
          ? (caller_ep.disconnected && answerer_ep.disconnected)
          : (caller_ep.connected && answerer_ep.connected))
      && !caller_ep.detection_unsupported && !answerer_ep.detection_unsupported
      && !caller_ep.link_error && !answerer_ep.link_error
      && caller_ep.rx_len == answerer_ep.tx_len
      && answerer_ep.rx_len == caller_ep.tx_len
      && memcmp(caller_ep.rx, answerer_ep.tx, (size_t)answerer_ep.tx_len) == 0
      && memcmp(answerer_ep.rx, caller_ep.tx, (size_t)caller_ep.tx_len) == 0;
    if (exercise_busy)
        ok = ok && busy_started && busy_released && far_busy_seen;
    if (!ok && exercise_busy) {
        fprintf(stderr,
                "busy case: started=%d released=%d far_seen=%d connected=%d/%d rx=%d/%d errors=%d/%d\n",
                busy_started, busy_released, far_busy_seen,
                caller_ep.connected, answerer_ep.connected,
                caller_ep.rx_len, answerer_ep.rx_len,
                caller_ep.link_error, answerer_ep.link_error);
    }

done:
    if (caller)
        v42_free(caller);
    if (answerer)
        v42_free(answerer);
    return ok;
}

static bool run_detection_timeout_case(int bit_rate)
{
    endpoint_t ep;
    v42_state_t *caller;
    uint64_t expected = (uint64_t)bit_rate*750U/1000U;
    bool ok;

    init_payload(&ep, 0, 0);
    caller = v42_init(NULL, true, true, get_payload, put_payload, &ep);
    if (!caller)
        return false;
    v42_set_status_callback(caller, status_changed, &ep);
    if (v42_set_bit_rate(caller, bit_rate) != 0) {
        v42_free(caller);
        return false;
    }
    v42_restart(caller);
    for (uint64_t tick = 1; tick <= expected + 1; tick++) {
        ep.tick = tick;
        (void)v42_tx_bit(caller);
        if (ep.detection_unsupported)
            break;
    }
    ok = ep.detecting && ep.detection_unsupported
      && ep.unsupported_at == expected;
    v42_free(caller);
    return ok;
}

static bool run_sustained_outage_case(int bit_rate)
{
    endpoint_t caller_ep;
    endpoint_t answerer_ep;
    v42_state_t *caller = NULL;
    v42_state_t *answerer = NULL;
    /* An idle link may wait T403 (10 s), exhaust T401 retries, enter DISC,
     * and exhaust release retries, so allow the complete bounded sequence. */
    uint64_t max_ticks = (uint64_t)bit_rate*35U;
    bool outage = false;
    bool ok = false;

    init_payload(&caller_ep, 2048, 0x0A7A6E11U);
    init_payload(&answerer_ep, 2048, 0x0A7A6E22U);
    caller = v42_init(NULL, true, true,
                      get_payload, put_payload, &caller_ep);
    answerer = v42_init(NULL, false, true,
                        get_payload, put_payload, &answerer_ep);
    if (!caller || !answerer)
        goto done;
    v42_set_status_callback(caller, status_changed, &caller_ep);
    v42_set_status_callback(answerer, status_changed, &answerer_ep);
    if (v42_set_bit_rate(caller, bit_rate) != 0
        || v42_set_bit_rate(answerer, bit_rate) != 0) {
        goto done;
    }
    v42_restart(caller);
    v42_restart(answerer);

    for (uint64_t tick = 1; tick <= max_ticks; tick++) {
        int caller_bit;
        int answerer_bit;

        caller_ep.tick = tick;
        answerer_ep.tick = tick;
        caller_bit = v42_tx_bit(caller);
        answerer_bit = v42_tx_bit(answerer);
        if (!outage) {
            v42_rx_bit(answerer, caller_bit);
            v42_rx_bit(caller, answerer_bit);
            if (caller_ep.connected && answerer_ep.connected
                && caller_ep.rx_len >= 128 && answerer_ep.rx_len >= 128) {
                outage = true;
            }
        }
        if (outage && caller_ep.disconnected && answerer_ep.disconnected)
            break;
    }

    ok = outage && caller_ep.disconnected && answerer_ep.disconnected
      && caller_ep.rx_len > 0 && answerer_ep.rx_len > 0
      && caller_ep.rx_len < caller_ep.tx_len
      && answerer_ep.rx_len < answerer_ep.tx_len
      && memcmp(caller_ep.rx, answerer_ep.tx, (size_t)caller_ep.rx_len) == 0
      && memcmp(answerer_ep.rx, caller_ep.tx, (size_t)answerer_ep.rx_len) == 0;
    if (!ok) {
        fprintf(stderr,
                "outage case: outage=%d disconnected=%d/%d connected=%d/%d rx=%d/%d errors=%d/%d\n",
                outage, caller_ep.disconnected, answerer_ep.disconnected,
                caller_ep.connected, answerer_ep.connected,
                caller_ep.rx_len, answerer_ep.rx_len,
                caller_ep.link_error, answerer_ep.link_error);
    }

done:
    if (caller)
        v42_free(caller);
    if (answerer)
        v42_free(answerer);
    return ok;
}

static bool run_peer_compression_request(void)
{
    /* CRC-valid RasFinder XID response requests P0=3. The byte-shuttle
       endpoint has no V.42bis codec and must negotiate P0=0 (5.1/Annex A). */
    static const uint8_t xid[] = {
        0x03, 0xaf, 0x82, 0x80, 0x00, 0x13, 0x03, 0x03, 0x8e, 0x89, 0x00, 0x05, 0x02, 0x04, 0x00, 0x06, 0x02, 0x04, 0x00, 0x07, 0x01, 0x0f, 0x08, 0x01, 0x0f, 0xf0, 0x00, 0x0f, 0x00, 0x03, 0x56, 0x34, 0x32, 0x01, 0x01, 0x03, 0x02, 0x02, 0x04, 0x00, 0x03, 0x01, 0x20
    };
    endpoint_t ep = {0};
    v42_negotiated_parameters_t parameters;
    v42_state_t *caller = v42_init(NULL, true, false,
                                  get_payload, put_payload, &ep);
    hdlc_tx_state_t *peer = hdlc_tx_init(NULL, false, 1, false, NULL, NULL);
    bool ok = false;

    if (!caller || !peer)
        goto done;
    v42_set_status_callback(caller, status_changed, &ep);
    v42_restart(caller);
    hdlc_tx_flags(peer, 16);
    hdlc_tx_frame(peer, xid, sizeof(xid));
    for (int i = 0; i < 2048; i++)
    {
        (void)v42_tx_bit(caller);
        v42_rx_bit(caller, hdlc_tx_get_bit(peer));
    }
    ok = ep.xid_negotiated
      && v42_get_negotiated_parameters(caller, &parameters) == 0
      && parameters.compression_p0 == 0;
done:
    if (peer)
        hdlc_tx_free(peer);
    if (caller)
        v42_free(caller);
    return ok;
}


/* Decode our transmitted bits independently of the LAPM XID parser. */
static void check_xid_frame(void *user_data, const uint8_t *frame, int len, int ok)
{
    bool *valid = user_data;
    bool options = false, refusal = false;
    if (!ok || len < 3 || frame[2] != 0x82)
        return;
    for (int i = 3; i < len; )
    {
        if (len - i < 3)
            return;
        int group = frame[i];
        int end = i + 3 + (frame[i + 1] << 8) + frame[i + 2];
        i += 3;
        if (end > len)
            return;
        while (i < end)
        {
            if (end - i < 2)
                return;
            int id = frame[i++];
            int n = frame[i++];
            if (n > end - i)
                return;
            if (group == 0x80 && id == 3)
                options = n == 4 && memcmp(frame + i, "\x8a\x89\x00\x00", 4) == 0;
            if (group == 0xf0 && id == 1)
                refusal = n == 1 && frame[i] == 0;
            i += n;
        }
    }
    *valid = options && refusal;
}

static bool run_outgoing_xid(void)
{
    endpoint_t ep = {0};
    bool valid = false;
    v42_state_t *caller = v42_init(NULL, true, false, get_payload, put_payload, &ep);
    hdlc_rx_state_t *wire = hdlc_rx_init(NULL, false, false, 1, check_xid_frame, &valid);
    if (caller && wire)
    {
        v42_restart(caller);
        for (int i = 0; i < 2048; i++)
            hdlc_rx_put_bit(wire, v42_tx_bit(caller));
    }
    if (wire)
        hdlc_rx_free(wire);
    if (caller)
        v42_free(caller);
    return valid;
}


typedef struct { uint8_t bytes[128]; int len; } xid_wire_t;
static void capture_v44_xid(void *ctx, const uint8_t *p, int n, int ok)
{
    xid_wire_t *wire = ctx;
    if (ok && n >= 3 && p[2] == 0x82 && n <= (int)sizeof(wire->bytes)) {
        memcpy(wire->bytes,p,(size_t)n);
        wire->len = n;
    }
}
static bool v44_xid_offer(void)
{
    static const uint8_t tail[] = {
        0xff,0x40,3,'V','4','4',0x41,1,0,0x42,1,1,
        0x43,2,4,0,0x44,2,3,0,0x45,1,64,0x46,1,48,
        0x47,2,12,0,0x48,2,8,0
    };
    v42_v44_parameters_t p = {0,1,1024,768,64,48,3072,2048};
    endpoint_t ep = {0}; xid_wire_t wire = {0};
    v42_state_t *caller = v42_init(NULL,true,false,get_payload,put_payload,&ep);
    hdlc_rx_state_t *rx = hdlc_rx_init(NULL,false,false,1,capture_v44_xid,&wire);
    bool ok = false;
    if (caller && rx && v42_set_v44(caller,&p) == 0) {
        v42_restart(caller);
        for (int i = 0; i < 2048; i++) hdlc_rx_put_bit(rx,v42_tx_bit(caller));
        /* Header=3, parameter group=23, V.44 user data=34. No private group. */
        ok = wire.len == 60 && wire.bytes[3] == 0x80 && wire.bytes[5] == 20
             && memcmp(wire.bytes+26,tail,sizeof(tail)) == 0;
    }
    if (rx) hdlc_rx_free(rx);
    if (caller) v42_free(caller);
    return ok;
}
static bool v44_peer_xid(int direction, bool malformed, bool defaults, bool support)
{
    /* Foreign CX93001 XID; C0=3 supports post-link negotiation. Our C0=0
       chooses XID parameters under V.44 Cor.1/2002, 7.4.1. */
    static const uint8_t captured[] = {
        3,0xaf,0x82,0x80,0,0x13,3,3,0x8a,0x89,0,5,2,4,0,6,2,4,0,7,1,15,8,1,15,
        0xff,0x40,3,'V','4','4',0x41,1,3,0x42,1,3,
        0x43,2,2,0,0x44,2,2,0,0x45,1,32,0x46,1,32,
        0x47,2,4,0,0x48,2,4,0
    };
    uint8_t frame[sizeof(captured)]; memcpy(frame,captured,sizeof(frame));
    int n = sizeof(frame);
    frame[36] = (uint8_t)direction;
    if (malformed) { frame[39] = 0; frame[40] = 1; } /* P1T=1 */
    if (defaults) n = 34; /* Through C0: default P0..P3 apply. */
    endpoint_t ep = {0};
    v42_v44_parameters_t local = {0,3,2048,2048,255,255,12288,12288};
    v42_negotiated_parameters_t agreed = {0};
    v42_state_t *caller = v42_init(NULL,true,false,get_payload,put_payload,&ep);
    hdlc_tx_state_t *tx = hdlc_tx_init(NULL,false,1,false,NULL,NULL);
    bool ok = false;
    if (caller && tx && (!support || v42_set_v44(caller,&local) == 0)) {
        v42_set_status_callback(caller,status_changed,&ep);
        v42_restart(caller);
        hdlc_tx_flags(tx,16); hdlc_tx_frame(tx,frame,n);
        for (int i = 0; i < 2048; i++) {
            (void)v42_tx_bit(caller);
            v42_rx_bit(caller,hdlc_tx_get_bit(tx));
        }
        int status = v42_get_negotiated_parameters(caller,&agreed);
        if (malformed) ok = !ep.xid_negotiated && status == -1;
        else if (!support) ok = ep.xid_negotiated && status == 0
                                && !agreed.v44_valid && agreed.compression_p0 == 0;
        else ok = ep.xid_negotiated && status == 0 && agreed.v44_valid
                  && agreed.compression_p0 == 0 && agreed.v44.capability == 0
                  && agreed.v44.directions == (defaults ? 3 : ((direction&1)<<1)|((direction&2)>>1))
                  && agreed.v44.tx_codewords == (defaults ? 1024 : 512)
                  && agreed.v44.rx_codewords == (defaults ? 1024 : 512)
                  && agreed.v44.tx_max_string == (defaults ? 255 : 32)
                  && agreed.v44.tx_history == (defaults ? 3072 : 1024);
    }
    if (tx) hdlc_tx_free(tx);
    if (caller) v42_free(caller);
    return ok;
}

int main(void)
{
    static const int rates[] = { 2400, 9600, 28800, 33600 };
    char label[128];

    for (size_t i = 0; i < sizeof(rates)/sizeof(rates[0]); i++) {
        snprintf(label, sizeof(label),
                 "V.42 detects, connects, and transfers at %d bit/s", rates[i]);
        CHECK(run_link_case(rates[i], 1024, 0, false, false), label);
        snprintf(label, sizeof(label),
                 "V.42 T400 is clocked at the configured %d bit/s", rates[i]);
        CHECK(run_detection_timeout_case(rates[i]), label);
    }
    CHECK(run_link_case(9600, 2048, 5000, false, false),
          "V.42 retransmits corrupted frames without payload corruption");
    CHECK(run_link_case(9600, 2048, 0, true, false),
          "V.42 RNR/RR backpressure pauses and resumes byte-exact transfer");
    CHECK(run_link_case(9600, 1024, 0, false, true),
          "V.42 DISC/UA release disconnects both endpoints cleanly");
    CHECK(run_sustained_outage_case(2400),
          "V.42 sustained outage reaches explicit retry-exhaustion disconnect");
    CHECK(run_outgoing_xid(),
          "V.42 transmits well-formed XID groups and explicit compression refusal");
    CHECK(run_peer_compression_request(),
          "V.42 declines a peer compression request without a V.42bis codec");
    CHECK(v44_xid_offer(), "V.44 Annex A outgoing XID user data has exact wire format");
    CHECK(v44_peer_xid(3,false,false,true), "V.44 accepts the captured CX93001 XID");
    CHECK(v44_peer_xid(1,false,false,true), "V.44 peer TX-only offer becomes local RX-only");
    CHECK(v44_peer_xid(3,true,false,true), "V.44 invalid dictionary limits reject the XID");
    CHECK(v44_peer_xid(3,false,true,true), "V.44 omitted parameters use corrected Table 10 defaults");
    CHECK(v44_peer_xid(3,false,false,false), "unsupported V.44 user data does not break plain LAPM negotiation");
    CHECK(v42_set_bit_rate(NULL, 9600) == -1,
          "V.42 rejects a missing timer context");

    if (failures) {
        printf("v42_link_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("v42_link_test: OK\n");
    return 0;
}
