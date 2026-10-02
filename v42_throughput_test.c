/*
 * v42_throughput_test.c -- LAPM throughput against round-trip delay.
 *
 * Two V.42 LAPM instances, both transmitting continuously, joined by a pair
 * of delay lines clocked at the 8 kHz media rate.  The bit rates default to a
 * V.90 call's (54666 down, 31200 up).  What this measures is the window: with
 * k I-frames of N401 octets outstanding at most, a direction cannot carry
 * more than k*N401 octets per acknowledgement round trip, however fast its
 * line is (V.42 8.4; k and N401 from XID, 9.2.3).  Everything in the round
 * trip counts -- the line, both modems' buffering, and our own receive jitter
 * buffer (sip_modem.c, ME_JB_MS).
 *
 *   v42_throughput_test                         the asserted cases (make test)
 *   v42_throughput_test <one-way-ms> [down up [flip-every-n-bits [outage-ms [down-source-bps]]]]
 *
 * Payload is pseudo-random, so V.42bis would gain nothing and is not used.
 * Uses only the public SpanDSP V.42 API.
 */

#include <spandsp.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SAMPLE_RATE  8000
#define MAX_DELAY_MS 2000
/* Bits per 8 kHz sample are at most 56000/8000 = 7; size generously. */
#define FIFO_BITS    (MAX_DELAY_MS*8*64 + 1024)

typedef struct {
    uint32_t seed;
    uint32_t check_seed;
    uint64_t tx_octets;
    uint64_t rx_octets;
    uint64_t rx_errors;
    bool connected;
    bool disconnected;
    double quota;       /* octets the source may still supply; < 0 = unlimited */
} endpoint_t;

typedef struct {
    uint8_t *bits;
    int head;
    int tail;
} fifo_t;

static int get_payload(void *user_data, uint8_t *msg, int max_len)
{
    endpoint_t *ep = (endpoint_t *)user_data;

    if (ep->quota >= 0) {
        if (ep->quota < max_len)
            max_len = (int)ep->quota;
        ep->quota -= max_len;
    }
    for (int i = 0; i < max_len; i++) {
        ep->seed = ep->seed*1664525U + 1013904223U;
        msg[i] = (uint8_t)(ep->seed >> 24);
    }
    ep->tx_octets += (uint64_t)max_len;
    return max_len;
}

static void put_payload(void *user_data, const uint8_t *msg, int len)
{
    endpoint_t *ep = (endpoint_t *)user_data;

    for (int i = 0; i < len; i++) {
        ep->check_seed = ep->check_seed*1664525U + 1013904223U;
        if (msg[i] != (uint8_t)(ep->check_seed >> 24))
            ep->rx_errors++;
    }
    ep->rx_octets += (uint64_t)len;
}

static void status_changed(void *user_data, int status)
{
    endpoint_t *ep = (endpoint_t *)user_data;

    if (status == SIG_STATUS_LINK_CONNECTED)
        ep->connected = true;
    else if (status == SIG_STATUS_LINK_DISCONNECTED) {
        ep->connected = false;
        ep->disconnected = true;
    }
}

static void fifo_put(fifo_t *f, int bit)
{
    f->bits[f->head] = (uint8_t)bit;
    if (++f->head >= FIFO_BITS)
        f->head = 0;
}

static int fifo_get(fifo_t *f)
{
    int bit = f->bits[f->tail];

    if (++f->tail >= FIFO_BITS)
        f->tail = 0;
    return bit;
}

static int fifo_used(const fifo_t *f)
{
    return (f->head - f->tail + FIFO_BITS) % FIFO_BITS;
}

typedef struct {
    double down_bps;   /* answerer (us, the digital modem) -> caller */
    double up_bps;     /* caller -> answerer */
    double down_kbps;  /* measured payload, answerer -> caller */
    double up_kbps;
    bool ok;
} result_t;

/* Each bit is held in its line for exactly one_way_ms: a bit transmitted at
   sample n is received at sample n + delay, in the order sent.  The delay is
   modelled as whole samples, which is all the 8 kHz media path can carry. */
static result_t run(int one_way_ms, int down_bps, int up_bps, int seconds, int flip_bits, int outage_ms,
                    int down_source_bps)
{
    result_t r = {down_bps, up_bps, 0, 0, false};
    endpoint_t ans = {0}, cal = {0};
    v42_state_t *a, *c;
    fifo_t down = {0}, up = {0};
    /* Bits written per sample, recorded so they can be released delay later. */
    int delay = one_way_ms*SAMPLE_RATE/1000;
    int *down_n = calloc((size_t)delay + 1, sizeof(int));
    int *up_n = calloc((size_t)delay + 1, sizeof(int));
    double down_acc = 0, up_acc = 0;
    int down_flip = 0;
    uint64_t start_down = 0, start_up = 0;
    int64_t measure_from = -1;
    int64_t total = (int64_t)(seconds + 5)*SAMPLE_RATE;

    down.bits = calloc(FIFO_BITS, 1);
    up.bits = calloc(FIFO_BITS, 1);
    ans.seed = ans.check_seed = 0x1234;
    cal.seed = cal.check_seed = 0x5678;
    /* The answerer receives what the caller sends, and vice versa. */
    ans.check_seed = 0x5678;
    cal.check_seed = 0x1234;
    ans.quota = (down_source_bps > 0)  ?  0  :  -1;
    cal.quota = -1;

    a = v42_init(NULL, false, false, get_payload, put_payload, &ans);
    c = v42_init(NULL, true, false, get_payload, put_payload, &cal);
    v42_set_status_callback(a, status_changed, &ans);
    v42_set_status_callback(c, status_changed, &cal);
    v42_set_bit_rate(a, down_bps);
    v42_set_bit_rate(c, up_bps);
    v42_restart(a);
    v42_restart(c);

    for (int64_t n = 0; n < total; n++) {
        int slot = (int)(n % (delay + 1));
        int k;

        /* Release what was sent delay samples ago (this slot's old count). */
        for (k = down_n[slot]; k > 0; k--)
            v42_rx_bit(c, fifo_get(&down));
        for (k = up_n[slot]; k > 0; k--)
            v42_rx_bit(a, fifo_get(&up));

        /* A source slower than the line keeps the window part-empty. */
        if (down_source_bps > 0)
            ans.quota += (double)down_source_bps/8.0/SAMPLE_RATE;
        down_acc += (double)down_bps/SAMPLE_RATE;
        up_acc += (double)up_bps/SAMPLE_RATE;
        down_n[slot] = (int)down_acc;
        up_n[slot] = (int)up_acc;
        down_acc -= down_n[slot];
        up_acc -= up_n[slot];
        for (k = 0; k < down_n[slot]; k++) {
            int bit = v42_tx_bit(a);

            /* Corrupt one bit in flip_bits: costs a frame, forces REJ and
               T401 recovery with a full window in flight. */
            if (flip_bits > 0 && ans.connected && cal.connected && ++down_flip >= flip_bits) {
                down_flip = 0;
                bit ^= 1;
            }
            fifo_put(&down, bit);
        }
        for (k = 0; k < up_n[slot]; k++) {
            int bit = v42_tx_bit(c);

            /* An outage of the return direction starting 2 s into the
               measurement: no acknowledgements for outage_ms, so T401
               expires on the answerer with a full window outstanding. */
            if (outage_ms > 0 && measure_from >= 0
                && n >= measure_from + 2*SAMPLE_RATE
                && n < measure_from + 2*SAMPLE_RATE + (int64_t)outage_ms*SAMPLE_RATE/1000)
                bit = 1;
            fifo_put(&up, bit);
        }
        if (fifo_used(&down) > FIFO_BITS - 64 || fifo_used(&up) > FIFO_BITS - 64)
            break;

        /* Measure over the last `seconds`, after a settling second. */
        if (measure_from < 0 && ans.connected && cal.connected && n >= SAMPLE_RATE)
            measure_from = n;
        if (measure_from >= 0 && n == measure_from + SAMPLE_RATE) {
            start_down = cal.rx_octets;
            start_up = ans.rx_octets;
        }
        if (measure_from >= 0 && n == measure_from + SAMPLE_RATE + (int64_t)seconds*SAMPLE_RATE - 1) {
            r.down_kbps = (double)(cal.rx_octets - start_down)*8/seconds/1000.0;
            r.up_kbps = (double)(ans.rx_octets - start_up)*8/seconds/1000.0;
            r.ok = ans.rx_errors == 0 && cal.rx_errors == 0
                   && !ans.disconnected && !cal.disconnected;
            break;
        }
    }
    v42_free(a);
    v42_free(c);
    free(down.bits);
    free(up.bits);
    free(down_n);
    free(up_n);
    return r;
}

/* What the window allows: k*N401 octets per acknowledgement round trip,
   which is both one-way delays plus the time to send one frame -- capped at
   the line rate.  Framing overhead is ignored, so it is a little high. */
static double window_bound_kbps(int one_way_ms, double bps)
{
    double window_octets = 15.0*128.0;
    double rtt_s = 2.0*one_way_ms/1000.0 + 131.0*8.0/bps;
    double bound = window_octets*8.0/rtt_s;

    return ((bound < bps)  ?  bound  :  bps)/1000.0;
}

static void print_row(int one_way_ms, const result_t *r)
{
    printf("one-way %4d ms: down %5.1f kbit/s of %5.1f (window bound %5.1f), "
           "up %5.1f kbit/s of %5.1f (window bound %5.1f)%s\n",
           one_way_ms,
           r->down_kbps, r->down_bps/1000.0, window_bound_kbps(one_way_ms, r->down_bps),
           r->up_kbps, r->up_bps/1000.0, window_bound_kbps(one_way_ms, r->up_bps),
           r->ok ? "" : "  PAYLOAD ERRORS");
}

int main(int argc, char **argv)
{
    int failures = 0;

    if (argc >= 2) {
        int ms = atoi(argv[1]);
        int down = (argc >= 3) ? atoi(argv[2]) : 54666;
        int up = (argc >= 4) ? atoi(argv[3]) : 31200;
        result_t r;

        if (ms < 0 || ms > MAX_DELAY_MS) {
            fprintf(stderr, "one-way delay must be 0..%d ms\n", MAX_DELAY_MS);
            return 2;
        }
        r = run(ms, down, up, 10, (argc >= 5) ? atoi(argv[4]) : 0, (argc >= 6) ? atoi(argv[5]) : 0,
                (argc >= 7) ? atoi(argv[6]) : 0);
        print_row(ms, &r);
        return r.ok ? 0 : 1;
    }

    /* Short round trip: each direction runs near its line rate (HDLC
       framing, FCS and bit stuffing take a few percent). */
    {
        result_t r = run(10, 54666, 31200, 10, 0, 0, 0);

        print_row(10, &r);
        if (!(r.ok && r.down_kbps > 0.90*54.666 && r.up_kbps > 0.90*31.2)) {
            printf("FAIL: short round trip should run each direction near its line rate\n");
            failures++;
        } else {
            printf("PASS: short round trip runs near line rate both ways\n");
        }
    }
    /* Long round trip: the window, not the line, sets the rate, and both
       directions converge on it -- what the live calls showed. */
    {
        result_t r = run(400, 54666, 31200, 10, 0, 0, 0);

        print_row(400, &r);
        if (!(r.ok
              && r.down_kbps < 0.80*54.666
              && r.down_kbps < 1.15*window_bound_kbps(400, 54666)
              && r.down_kbps > 0.70*window_bound_kbps(400, 54666))) {
            printf("FAIL: long round trip should be window-limited near k*N401/RTT\n");
            failures++;
        } else {
            printf("PASS: long round trip is window-limited at k*N401/RTT\n");
        }
    }
    /* Errors on a long round trip: every lost frame means REJ or T401
       recovery with a window in flight.  SpanDSP used to keep sending
       I-frames after its T401 poll; the F=1 response then rewound V(S)
       below them and their acknowledgement read as an invalid N(R), which
       tore the link down (a live V.90 call to slmodemd died this way). */
    {
        result_t r = run(400, 54666, 31200, 20, 150000, 0, 0);

        print_row(400, &r);
        if (!(r.ok && r.down_kbps > 0.5*window_bound_kbps(400, 54666))) {
            printf("FAIL: errored long round trip must recover without disconnecting\n");
            failures++;
        } else {
            printf("PASS: errored long round trip recovers and keeps the link\n");
        }
    }
    return failures ? 1 : 0;
}
