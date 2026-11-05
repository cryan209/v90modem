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
#include <float.h>
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
static void make_cpd(v92_cpd_frame_t *cpd, int drn, bool k3_half)
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
    if (k3_half)
        n--;                     /* LC even, as v90_build_v92_cpd_frame() */
    for (int i = 0; i < n; i++)
        cpd->points[0][i] = (uint16_t)(levels[i]*8);
    cpd->set_sizes[0] = (uint8_t)n;
    for (int i = 0; i < 12; i++)
        cpd->moduli[i] = (uint8_t)((k3_half && i%4 == 3) ? n/2 : n);
    /* Back drn off until product(Mi) >= 2^K, as the CPd builder does. */
    {
        double bits = 0.0;

        for (int i = 0; i < 12; i++)
            bits += log2((double)cpd->moduli[i]);
        while (cpd->selected_upstream_drn > 1
               && 2*(cpd->selected_upstream_drn + 17) > (int)bits)
            cpd->selected_upstream_drn--;
    }
}

/* After lock the receiver delivers what is left of B1u (ones) and then the
 * data; the sink keeps everything and the case finds the data in it. */
typedef struct {
    const uint8_t *expected;
    int nexpected;
    uint8_t *bits;
    int nbits;
    int got;
    int wrong;
} sink_t;

static void put_byte(void *user, uint8_t byte)
{
    sink_t *s = user;

    for (int b = 0; b < 8; b++)
        if (s->nbits < s->nexpected + 48*72)
            s->bits[s->nbits++] = (byte >> b) & 1;
}

/* Data must follow nothing but ones; returns data bits found, sets wrong. */
static void score(sink_t *s)
{
    int start = -1;

    for (int off = 0; off + 64 <= s->nbits && start < 0; off++) {
        bool all_ones_before = true;

        if (memcmp(s->bits + off, s->expected, 64) != 0)
            continue;
        for (int i = 0; i < off; i++)
            all_ones_before &= s->bits[i] == 1;
        if (all_ones_before)
            start = off;
    }
    s->got = 0;
    s->wrong = 0;
    if (start < 0)
        return;
    for (int i = start; i < s->nbits && s->got < s->nexpected; i++, s->got++)
        s->wrong += s->bits[i] != s->expected[s->got];
}

static int skip_b1u_frames;   /* start the feed this far into B1u */

static void run_case(int member, int lead, double noise, bool with_b1u,
                     int drn, bool k3_half, bool expect_lock)
{
    /* Transmit real trailing frames to drain decoder latency and byte
     * packing; do not flush by inventing samples at the receiver. */
    enum { DATA_FRAMES = 400, TAIL_FRAMES = V92_UPSTREAM_TRELLIS_DELAY_FRAMES + 2 };
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

    make_cpd(&cpd, drn, k3_half);
    k = v92_upstream_bits_per_frame(cpd.selected_upstream_drn);
    nsym = lead + (V92_B1U_FRAMES + DATA_FRAMES + TAIL_FRAMES)*12;
    samples = calloc((size_t)nsym, sizeof(double));
    bits = calloc((size_t)(DATA_FRAMES + TAIL_FRAMES)*k, 1);
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
    for (int f = 0; f < DATA_FRAMES + TAIL_FRAMES; f++) {
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
    sink.bits = calloc((size_t)(DATA_FRAMES*k + 48*72), 1);
    if (!v92_upstream_b1_rx_init_equalized(rx, &cpd, put_byte, &sink)) {
        CHECK(0, "init failed");
    } else {
        /* Odd chunks, as the engine's callbacks deliver them.  A late start
         * drops the lead-in and the first B1u frames, as the engine joins
         * B1u only once the acknowledged SUVu' is decoded. */
        int from = skip_b1u_frames ? lead + skip_b1u_frames*12 : 0;

        for (int at = from; at < pos; ) {
            int n = 1 + (int)(rnd() % 37);

            if (n > pos - at)
                n = pos - at;
            (void)v92_upstream_b1_rx_feed_values(rx, &samples[at], n);
            at += n;
        }
        score(&sink);
        got_frames = sink.got/k;
        if (with_b1u && !expect_lock) {
            CHECK(!rx->locked, "member %d drn %d k3_half %d: locked on a "
                  "stream the emulated peer could not have sent decodably",
                  member, drn, k3_half);
        } else if (with_b1u) {
            CHECK(rx->locked, "member %d lead %d noise %.0f drn %d: no lock",
                  member, lead, noise, drn);
            CHECK(sink.got == DATA_FRAMES*k && sink.wrong == 0,
                  "member %d lead %d noise %.0f drn %d: %d bits, %d wrong",
                  member, lead, noise, drn, sink.got, sink.wrong);
        } else {
            CHECK(!rx->locked, "locked on data with no B1u (%llu tried)",
                  (unsigned long long)rx->candidates_started);
        }
    }
    printf("member %d lead %4d noise %3.0f drn %2d k3/2 %d b1u %d: locked %d, "
           "%d data frames, %d wrong bits, %llu alignments tried\n",
           member, lead, noise, (int)cpd.selected_upstream_drn, k3_half ? 1 : 0, with_b1u ? 1 : 0, rx->locked ? 1 : 0,
           got_frames, sink.wrong,
           (unsigned long long)rx->candidates_started);
    free(samples);
    free(bits);
    free(sink.bits);
    free(rx);
}

/* Continuous data-mode regression: all Table 30 rate indices, exact
 * delayed source/trellis state and bounded ring wrap, including non-byte K. */
static void test_stream_rates(void)
{
    int saw_differential = 0;
    for (int drn = 1; drn <= 19; drn++) {
        v92_cpd_frame_t c = {0};
        v92_upstream_wave_tx_t tx;
        v92_upstream_wave_rx_t initial;
        v92_upstream_trellis_t stream;
        uint8_t source[64][V92_UPSTREAM_MAX_FRAME_BITS], out[72];
        v92_upstream_tx_state_t saved[64];
        uint8_t conv[64];
        double samples[12];
        int k = v92_upstream_bits_per_frame((uint8_t)drn);

        c.modulus_present = c.constellations_present = true;
        c.selected_upstream_drn = (uint8_t)drn;
        c.gain_q0_16 = 32768;
        /* Minimal even LC for this K also exercises differential folding,
         * unlike an oversized alphabet with R permanently below P/2. */
        int lc;
        for (lc = 2; lc <= 96; lc += 2) {
            __uint128_t product = 1;
            for (int i = 0; i < 12; i++) product *= (unsigned)(i%4 == 3 ? lc/2 : lc);
            if (product >= ((__uint128_t)1 << k)) break;
        }
        c.set_sizes[0] = (uint8_t)lc;
        for (int i = 0; i < lc; i++) c.points[0][i] = (uint16_t)(256*(2*i + 1));
        for (int i = 0; i < 12; i++) c.moduli[i] = (uint8_t)(i%4 == 3 ? lc/2 : lc);
        v92_upstream_wave_tx_init(&tx);
        v92_upstream_wave_rx_init(&initial);
        CHECK(v92_upstream_trellis_init(&stream, &c, &initial), "stream init drn %d", drn);
        for (int f = 0; f < 64; f++) {
            int result;
            for (int i = 0; i < k; i++) source[f][i] = (uint8_t)(rnd() & 1);
            CHECK(v92_upstream_wave_encode_frame(&tx, &c, source[f], k, samples), "encode");
            saved[f] = tx.data;
            saw_differential |= tx.data.previous_differential_sign;
            conv[f] = tx.convolutional_state;
            result = v92_upstream_trellis_feed(&stream, samples, out, sizeof(out));
            if (f < V92_UPSTREAM_TRELLIS_DELAY_FRAMES) {
                CHECK(result == 0 && stream.data.descramble_reg == 0,
                      "drn %d advanced source state during latency fill", drn);
            } else {
                int old = f - V92_UPSTREAM_TRELLIS_DELAY_FRAMES;
                CHECK(result == 1 && memcmp(out, source[old], (size_t)k) == 0,
                      "drn %d delayed frame %d", drn, old);
                CHECK(stream.data.descramble_reg == saved[old].scramble_reg
                      && stream.data.previous_differential_sign == saved[old].previous_differential_sign
                      && stream.convolutional_state == conv[old], "delayed state drn %d", drn);
            }
        }
        {
            uint64_t before = stream.groups;
            samples[0] = NAN;
            CHECK(v92_upstream_trellis_feed(&stream, samples, out, sizeof(out)) == -2
                  && stream.groups == before, "NaN consumed stream state");
            samples[0] = DBL_MAX;
            CHECK(v92_upstream_trellis_feed(&stream, samples, out, sizeof(out)) == -2
                  && stream.groups == before, "overflow consumed stream state");
            CHECK(v92_upstream_trellis_init(&stream, &c, &initial)
                  && stream.groups == 0 && stream.data.descramble_reg == 0,
                  "stream per-call reset");
        }
    }
    CHECK(saw_differential, "test did not exercise differential folding");
}

static void dense_cpd(v92_cpd_frame_t *c)
{
    static const uint16_t points[] = {
        129,396,654,929,1188,1446,1705,1996,2254,2513,2771,3030,
        3288,3547,3805,4258,4516,4775,5033,5292,5550,5809,6067,6326,
        6584,6843,7102,7360,7619,7877,8265,8782,9299,9816,10333,10850
    };
    memset(c, 0, sizeof(*c));
    c->modulus_present = c->constellations_present = true;
    c->selected_upstream_drn = 11;
    c->gain_q0_16 = 90;
    c->set_sizes[0] = 36;
    memcpy(c->points[0], points, sizeof(points));
    for (int i = 0; i < 12; i++) c->moduli[i] = (uint8_t)(i%4 == 3 ? 18 : 36);
}

/* A geometrically valid trellis path outside §6.4.1's source alphabet must
 * be marked erased, not reported valid or silently removed from bit time. */
static void test_stream_erasure(void)
{
    v92_cpd_frame_t c;
    v92_upstream_wave_tx_t tx;
    v92_upstream_wave_rx_t initial;
    v92_upstream_trellis_t stream;
    uint8_t ki[12], ones[72], out[72];
    double samples[12];
    __uint128_t word = (__uint128_t)1 << 56;

    dense_cpd(&c);
    for (int i = 0; i < 12; i++) { ki[i] = (uint8_t)(word%c.moduli[i]); word /= c.moduli[i]; }
    memset(ones, 1, sizeof(ones));
    v92_upstream_wave_tx_init(&tx);
    v92_upstream_wave_rx_init(&initial);
    CHECK(v92_upstream_trellis_init(&stream, &c, &initial), "erasure init");
    for (int f = 0; f < 12; f++) {
        int result;
        CHECK(f == 0 ? v92_upstream_wave_encode_ki(&tx, &c, ki, samples)
                     : v92_upstream_wave_encode_frame(&tx, &c, ones, 56, samples), "erasure encode");
        result = v92_upstream_trellis_feed(&stream, samples, out, sizeof(out));
        if (f == V92_UPSTREAM_TRELLIS_DELAY_FRAMES)
            CHECK(result == -1 && memcmp(out, ones, 56) == 0, "invalid word not erased");
        else if (f > V92_UPSTREAM_TRELLIS_DELAY_FRAMES)
            CHECK(result == 1 && memcmp(out, ones, 56) == 0, "did not recover after erasure");
        else CHECK(result == 0, "erasure latency fill");
    }
}

static void test_erasure_bit_clock(void)
{
    static const int drns[] = {1,10,11};
    for (unsigned d = 0; d < sizeof(drns)/sizeof(drns[0]); d++) {
        v92_cpd_frame_t c;
        v92_upstream_wave_tx_t tx;
        v92_upstream_rx_t *rx = calloc(1, sizeof(*rx));
        uint8_t ki[12], ones[72], received[5000];
        double samples[12];
        int k = v92_upstream_bits_per_frame((uint8_t)drns[d]);
        __uint128_t word = (__uint128_t)1 << k;
        sink_t sink = {0};

        dense_cpd(&c); c.selected_upstream_drn = (uint8_t)drns[d];
        for (int i = 0; i < 12; i++) { ki[i] = (uint8_t)(word%c.moduli[i]); word /= c.moduli[i]; }
        memset(ones, 1, sizeof(ones));
        sink.nexpected = 13*k; sink.bits = received;
        v92_upstream_wave_tx_init(&tx);
        CHECK(v92_upstream_b1_rx_init_equalized(rx, &c, put_byte, &sink), "bit clock init");
        rx->locked = true;
        CHECK(v92_upstream_trellis_init(&rx->trellis, &c, &rx->wave_rx), "bit clock stream init");
        for (int f = 0; f < 13; f++) {
            CHECK(f == 0 ? v92_upstream_wave_encode_ki(&tx, &c, ki, samples)
                         : v92_upstream_wave_encode_frame(&tx, &c, ones, k, samples), "bit clock encode");
            (void)v92_upstream_b1_rx_feed_values(rx, samples, 12);
        }
        CHECK(rx->input_symbols == 13*12 && rx->output_bits == (uint64_t)(9*k)
              && rx->byte_bits == (9*k)%8 && sink.nbits == 8*((9*k)/8)
              && rx->rejected_frames == 1 && rx->decoded_frames == 8,
              "erasure deleted bits at drn %d", drns[d]);
        for (int i = 0; i < sink.nbits; i++) CHECK(sink.bits[i] == 1, "erasure/recovery bit");
        free(rx);
    }
}

/* Live 37333 soft samples, before trellis decoding; §8.7.1 supplies the
 * warm state from the known first 18 B1u frames. No payload oracle is used
 * by the decoder. Quantised Q20 fixture and packed source are indexed under
 * reference-data/, not expiring captures. Also exercise refit-ring offsets
 * and pending frames that straddle a replay/live input boundary. */
static void test_live_soft_fixture(void)
{
    enum { FRAMES = 5000, K = 56 };
    v92_cpd_frame_t full, c;
    v92_upstream_wave_tx_t tx;
    v92_upstream_wave_rx_t initial;
    double *soft = calloc(FRAMES*12, sizeof(*soft));
    uint8_t *expected = calloc(FRAMES*K, 1);
    FILE *f = fopen("reference-data/v92-upstream-trellis/37333-soft-q20.s32le", "rb");
    FILE *gold = fopen("reference-data/v92-upstream-trellis/37333-source-lsb.bin", "rb");
    uint8_t ones[72], out[72];
    double samples[12];
    static const int16_t p1[] = {2819,5710,821,1808,-868,1870,261,360};
    static const int chunks[] = {1,37,160,320};

    CHECK(f && gold && soft && expected, "fixture open/allocation");
    if (!f || !gold || !soft || !expected) {
        if (f) fclose(f);
        if (gold) fclose(gold);
        free(soft); free(expected);
        return;
    }
    for (int i = 0; i < FRAMES*12; i++) {
        uint8_t b[4] = {0}; uint32_t raw; int32_t value;
        CHECK(fread(b, 1, 4, f) == 4, "short soft fixture");
        raw = (uint32_t)b[0] | (uint32_t)b[1]<<8 | (uint32_t)b[2]<<16 | (uint32_t)b[3]<<24;
        memcpy(&value, &raw, sizeof(value));
        soft[i] = value/1048576.0;
    }
    for (int i = 0; i < FRAMES*K; i += 8) {
        int b = fgetc(gold); CHECK(b != EOF, "short source fixture");
        for (int j = 0; j < 8; j++) expected[i+j] = (uint8_t)((b >> j)&1);
    }
    CHECK(fgetc(f) == EOF && fgetc(gold) == EOF, "fixture has extra data");
    fclose(f); fclose(gold);
    dense_cpd(&full); full.coeffs_present = true; full.lp1 = 8; full.lz2 = 4;
    full.prefilter_ff[0] = 32767; memcpy(full.precoder_fb, p1, sizeof(p1));
    memset(ones, 1, sizeof(ones));
    v92_upstream_wave_tx_init(&tx); tx.class_member = V92_UPSTREAM_MEMBER_SLMODEMD;
    v92_upstream_wave_rx_init(&initial);
    for (int frame = 0; frame < 18; frame++) {
        CHECK(v92_upstream_wave_encode_frame(&tx, &full, ones, K, samples)
              && v92_upstream_wave_decode_frame(&initial, &full, samples, out, 72)
              && memcmp(out, ones, K) == 0, "known B1u warm state");
    }
    c = full; c.coeffs_present = false;
    for (unsigned chunk = 0; chunk < sizeof(chunks)/sizeof(chunks[0]); chunk++) {
        v92_upstream_rx_t *rx = calloc(1, sizeof(*rx));
        sink_t sink = {0}; uint64_t marked = 0;
        sink.nexpected = FRAMES*K; sink.bits = calloc(FRAMES*K + 48*72, 1);
        CHECK(v92_upstream_b1_rx_init_equalized(rx, &c, put_byte, &sink), "fixture init");
        rx->locked = true; rx->wave_rx = initial;
        CHECK(v92_upstream_trellis_init(&rx->trellis, &c, &initial), "fixture stream init");
        for (int at = 0; at < FRAMES*12;) {
            int n = chunks[chunk]; if (n > FRAMES*12-at) n = FRAMES*12-at;
            (void)v92_upstream_b1_rx_feed_values(rx, soft+at, n); at += n;
            if (!marked && at >= 37) {
                v92_upstream_b1_rx_refit_live(rx); marked = rx->input_symbols;
            }
            if (rx->decision_end_symbol) {
                for (int i = 0; i < 12; i++) {
                    int back = v92_upstream_b1_rx_decision_back(rx, 2, i);
                    if (rx->decision_end_symbol - 12 < marked)
                        CHECK(back == -1, "replayed/pending target used for a live row");
                    else CHECK(back >= 0 && rx->input_symbols+2-(uint64_t)back
                               == rx->decision_end_symbol-11+(uint64_t)i,
                               "wrong equaliser input window, chunk %d", n);
                }
            }
        }
        CHECK(rx->input_symbols == FRAMES*12
              && rx->output_bits == (FRAMES-V92_UPSTREAM_TRELLIS_DELAY_FRAMES)*K
              && sink.nbits == (int)rx->output_bits, "sample/bit latency accounting");
        CHECK(rx->rejected_frames == 0 && memcmp(sink.bits, expected, (size_t)sink.nbits) == 0,
              "live soft fixture: chunk %d, %llu rejected", chunks[chunk],
              (unsigned long long)rx->rejected_frames);
        {
            double invalid[12] = {NAN};
            uint64_t before = rx->output_bits;
            (void)v92_upstream_b1_rx_feed_values(rx, invalid, 12);
            CHECK(!rx->locked && !rx->trellis.initialized
                  && rx->output_bits == before && rx->decision_end_symbol == 0,
                  "invalid samples did not fail closed");
        }
        CHECK(v92_upstream_b1_rx_init_equalized(rx, &c, NULL, NULL)
              && !rx->locked && !rx->trellis.initialized && rx->refit_skip_symbols == 0
              && rx->output_bits == 0, "receiver per-call reset");
        free(sink.bits); free(rx);
    }
    free(soft); free(expected);
}

int main(void)
{
    static const int members[] = { V92_UPSTREAM_MEMBER_MIN_X,
                                   V92_UPSTREAM_MEMBER_MOST_NEGATIVE,
                                   V92_UPSTREAM_MEMBER_MOST_POSITIVE };

    for (int h = 0; h < 2; h++) {
        for (int m = 0; m < 3; m++) {
            run_case(members[m], 0, 0.0, true, 6, h, true);
            run_case(members[m], 1237, 0.0, true, 6, h, true);
            /* sigma 60: the TRN2u noise the design measured against
             * slmodemd, a sixth of the closest spacing (512). */
            run_case(members[m], 503, 60.0, true, 6, h, true);
            run_case(members[m], 77, 30.0, true, 1, h, true);
        }
    }
    /* slmodemd's precoder (V92_UPSTREAM_MEMBER_SLMODEMD): with Mi = LC at
     * k = 3 it sends 0 for half of those symbols and nothing can decode
     * it; with LC even and Mi = LC/2 there it sends valid points. */
    run_case(V92_UPSTREAM_MEMBER_SLMODEMD, 503, 30.0, true, 6, false, false);
    run_case(V92_UPSTREAM_MEMBER_SLMODEMD, 503, 30.0, true, 6, true, true);
    run_case(V92_UPSTREAM_MEMBER_SLMODEMD, 77, 60.0, true, 4, true, true);
    /* Joining B1u 15 and 31 frames late, as against slmodemd. */
    skip_b1u_frames = 15;
    run_case(V92_UPSTREAM_MEMBER_SLMODEMD, 503, 30.0, true, 6, true, true);
    run_case(V92_UPSTREAM_MEMBER_MIN_X, 0, 60.0, true, 6, false, true);
    skip_b1u_frames = 31;
    run_case(V92_UPSTREAM_MEMBER_SLMODEMD, 503, 30.0, true, 6, true, true);
    skip_b1u_frames = 0;
    run_case(V92_UPSTREAM_MEMBER_MIN_X, 200, 30.0, false, 6, false, true);
    run_case(V92_UPSTREAM_MEMBER_MOST_NEGATIVE, 200, 30.0, false, 6, true, true);
    test_stream_rates();
    test_stream_erasure();
    test_erasure_bit_clock();
    test_live_soft_fixture();
    if (failures) {
        printf("v92_b1u_lock_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("v92_b1u_lock_test: all passed\n");
    return 0;
}
