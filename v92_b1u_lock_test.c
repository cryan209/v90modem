/*
 * v92_b1u_lock_test.c -- the equalised-mode B1u receiver locks on eta mod Mi,
 * whichever member of each equivalence class the transmitter sends.
 *
 * V.92 6.4.2 defines E(Ki) and leaves the choice of member to the analogue
 * modem; our encoder minimises |x|, slmodemd demonstrably does not.  Each
 * case generates a lead-in, B1u (8.7.1: 48 frames of scrambled ones from
 * zeroed memories) and data frames of known random bits with one of the
 * three member policies, at an arbitrary sample offset and with additive
 * noise, and requires the receiver to lock on B1u and return the data bits
 * exactly.  A last case feeds data with no B1u and requires no lock.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v92_upstream_rx.h"

static int failures;

#define CHECK(cond, ...) do { if (!(cond)) { failures++; \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); \
    printf("\n"); } } while (0)

static uint32_t rng = 12345;
static uint32_t rnd(void) { rng = rng*1103515245u + 12345u; return rng >> 8; }
static double gauss(void)
{
    double u1 = ((rnd() & 0xFFFFFF) + 1.0)/16777217.0;
    double u2 = (rnd() & 0xFFFFFF)/16777216.0;
    return sqrt(-2.0*log(u1))*cos(2.0*M_PI*u2);
}

/* A Table 30 profile like the one the engine designs from TRN2u: G.711
 * levels kept apart, drn from the moduli. */
static void make_cpd(v92_cpd_frame_t *cpd, int drn)
{
    static const int levels[] = { 276, 812, 1372, 1884, 2620, 3132, 3644,
                                  4348, 4860, 5372, 5884, 6396, 6908, 7420,
                                  7932 };
    int n = (int)(sizeof(levels)/sizeof(levels[0]));

    memset(cpd, 0, sizeof(*cpd));
    cpd->modulus_present = true;
    cpd->constellations_present = true;
    cpd->selected_upstream_drn = (uint8_t)drn;
    cpd->gain_q0_16 = 0x8000;                    /* G = 0.125 */
    for (int i = 0; i < n; i++)
        cpd->points[0][i] = (uint16_t)(levels[i]*8);
    cpd->set_sizes[0] = (uint8_t)n;
    for (int i = 0; i < 12; i++)
        cpd->moduli[i] = (uint8_t)n;
}

typedef struct {
    const uint8_t *expected;
    int nexpected;
    int got;
    int wrong;
} sink_t;

static void put_byte(void *user, uint8_t byte)
{
    sink_t *s = user;

    for (int b = 0; b < 8; b++) {
        if (s->got < s->nexpected && ((byte >> b) & 1) != s->expected[s->got])
            s->wrong++;
        s->got++;
    }
}

static void run_case(int member, int lead, double noise, bool with_b1u,
                     int drn)
{
    enum { DATA_FRAMES = 400 };
    v92_cpd_frame_t cpd;
    v92_upstream_wave_tx_t tx;
    v92_upstream_rx_t *rx = calloc(1, sizeof(*rx));
    int k;
    int nsym;
    double *samples;
    uint8_t *bits;
    uint8_t ones[V92_UPSTREAM_MAX_FRAME_BITS];
    sink_t sink = { 0 };
    int pos = 0;
    int got_frames = 0;

    make_cpd(&cpd, drn);
    k = v92_upstream_bits_per_frame(cpd.selected_upstream_drn);
    nsym = lead + (V92_B1U_FRAMES + DATA_FRAMES)*12;
    samples = calloc((size_t)nsym, sizeof(double));
    bits = calloc((size_t)DATA_FRAMES*k, 1);
    memset(ones, 1, sizeof(ones));

    /* Lead-in: four-level noise about the TRN2u size, nothing B1u-like. */
    for (int i = 0; i < lead; i++)
        samples[pos++] = ((int)(rnd() % 4) - 1.5)*600.0;

    v92_upstream_wave_tx_init(&tx);
    tx.class_member = member;
    if (with_b1u) {
        for (int f = 0; f < V92_B1U_FRAMES; f++) {
            if (!v92_upstream_wave_encode_frame(&tx, &cpd, ones, k,
                                                &samples[pos]))
                CHECK(0, "B1u encode failed");
            pos += 12;
        }
    }
    for (int f = 0; f < DATA_FRAMES; f++) {
        for (int i = 0; i < k; i++)
            bits[f*k + i] = (uint8_t)(rnd() & 1);
        if (!v92_upstream_wave_encode_frame(&tx, &cpd, &bits[f*k], k,
                                            &samples[pos]))
            CHECK(0, "data encode failed");
        pos += 12;
    }
    for (int i = 0; i < pos; i++)
        samples[i] += noise*gauss();

    sink.expected = bits;
    sink.nexpected = DATA_FRAMES*k;
    if (!v92_upstream_b1_rx_init_equalized(rx, &cpd, put_byte, &sink)) {
        CHECK(0, "init failed");
    } else {
        /* Odd chunks, as the engine's callbacks deliver them. */
        for (int at = 0; at < pos; ) {
            int n = 1 + (int)(rnd() % 37);

            if (n > pos - at)
                n = pos - at;
            (void)v92_upstream_b1_rx_feed_values(rx, &samples[at], n);
            at += n;
        }
        got_frames = sink.got/k;
        if (with_b1u) {
            CHECK(rx->locked, "member %d lead %d noise %.0f drn %d: no lock",
                  member, lead, noise, drn);
            CHECK(sink.got >= (DATA_FRAMES - 1)*k && sink.wrong == 0,
                  "member %d lead %d noise %.0f drn %d: %d bits, %d wrong",
                  member, lead, noise, drn, sink.got, sink.wrong);
        } else {
            CHECK(!rx->locked, "locked on data with no B1u (%llu tried)",
                  (unsigned long long)rx->candidates_started);
        }
    }
    printf("member %d lead %4d noise %3.0f drn %2d b1u %d: locked %d, "
           "%d data frames, %d wrong bits, %llu alignments tried\n",
           member, lead, noise, drn, with_b1u ? 1 : 0, rx->locked ? 1 : 0,
           got_frames, sink.wrong,
           (unsigned long long)rx->candidates_started);
    free(samples);
    free(bits);
    free(rx);
}

int main(void)
{
    static const int members[] = { V92_UPSTREAM_MEMBER_MIN_X,
                                   V92_UPSTREAM_MEMBER_MOST_NEGATIVE,
                                   V92_UPSTREAM_MEMBER_MOST_POSITIVE };

    for (int m = 0; m < 3; m++) {
        run_case(members[m], 0, 0.0, true, 6);
        run_case(members[m], 1237, 0.0, true, 6);
        /* sigma 60: the TRN2u noise the design measured against slmodemd,
         * a sixth of the closest spacing (512). */
        run_case(members[m], 503, 60.0, true, 6);
        run_case(members[m], 77, 30.0, true, 1);
    }
    run_case(V92_UPSTREAM_MEMBER_MIN_X, 200, 30.0, false, 6);
    run_case(V92_UPSTREAM_MEMBER_MOST_NEGATIVE, 200, 30.0, false, 6);
    if (failures) {
        printf("v92_b1u_lock_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("v92_b1u_lock_test: all passed\n");
    return 0;
}
