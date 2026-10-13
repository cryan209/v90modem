/*
 * v92_qc_test.c -- the digital modem's short Phase 1 (V.92 9.2.4), signal by
 * signal, against the decoders the project already validates on captures.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v90.h"
#include "v91.h"
#include "v92_anspcm_decode.h"
#include "v92_qc.h"
#include "v92_short_phase1_decode.h"

static int failures;

#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); failures++; } \
} while (0)

#define MAX_CW 40000

typedef struct {
    uint8_t cw[MAX_CW];
    int n;
} stream_t;

static void run_tx(v92_qc_t *q, stream_t *s, int total, int chunk)
{
    while (s->n < total && v92_qc_running(q)) {
        int want = total - s->n < chunk ? total - s->n : chunk;
        int got = v92_qc_tx(q, s->cw + s->n, want);

        s->n += got;
        if (got < want)
            break;
    }
}

/* ---- QCA1d demodulation ---- */
typedef struct { uint8_t bits[400]; int n; } bitbuf_t;

static void put_bit(void *user, int bit)
{
    bitbuf_t *b = user;

    if (bit < 0 || b->n >= (int) sizeof(b->bits))
        return;
    b->bits[b->n++] = (uint8_t) bit;
}

static void test_qca1d(v91_law_t law, int lm, bool lapm)
{
    v92_qc_t q;
    stream_t s = { .n = 0 };
    fsk_rx_state_t *rx;
    bitbuf_t bb = { .n = 0 };
    int16_t lin[160];
    bool found = false;

    v92_qc_init(&q);
    assert(v92_qc_start(&q, law, 66, lm, lapm));
    run_tx(&q, &s, 2200, 160);
    /* QCA1d is 70 bits at 300 bit/s: 1867 samples. */
    CHECK(v92_qc_state(&q) == V92_QC_GAP || s.n == 2200,
          "QCA1d should have ended by 2200 samples (state %s)", v92_qc_state_name(v92_qc_state(&q)));

    rx = fsk_rx_init(NULL, &preset_fsk_specs[FSK_V21CH2], FSK_FRAME_MODE_SYNC, put_bit, &bb);
    for (int off = 0; off < s.n; off += 160) {
        int n = s.n - off < 160 ? s.n - off : 160;

        for (int i = 0; i < n; i++)
            lin[i] = v91_codeword_to_linear(law, s.cw[off + i]);
        fsk_rx(rx, lin, n);
    }
    for (int start = 0; start + 70 <= bb.n && !found; start++) {
        v92_short_phase1_candidate_t c;

        if (v92_decode_short_phase1_sequence_strict(bb.bits + start, bb.n - start, true, &c)) {
            found = true;
            CHECK(c.digital_modem && c.qca, "decoded as %s", c.name);
            CHECK(c.aux_value == lm, "LM %d != %d", c.aux_value, lm);
            CHECK(c.lapm == lapm, "P bit");
            CHECK(c.strict_sequence_bits == 70, "70-bit QCA sequence");
        }
    }
    CHECK(found, "QCA1d is not a strict QCA1d sequence on the wire (law %d)", (int) law);
    fsk_rx_free(rx);
    v92_qc_free(&q);
    printf("%s: QCA1d law=%d LM=%d lapm=%d\n", found ? "PASS" : "FAIL", (int) law, lm, lapm);
}

/* ---- the whole sequence ---- */
static void tone_cw(v91_law_t law, uint8_t *cw, int n, double hz, double amp, double *ph)
{
    for (int i = 0; i < n; i++) {
        cw[i] = v91_linear_to_codeword(law, (int16_t) lrint(amp * sin(*ph)));
        *ph += 2.0 * M_PI * hz / 8000.0;
    }
}

static void test_sequence(v91_law_t law, int uqts, int level)
{
    v92_qc_t q;
    stream_t s = { .n = 0 };
    v92_qts_hit_t qts;
    v92_anspcm_table_hit_t pcm;
    int qts_at, silent = 0;

    v92_qc_init(&q);
    assert(v92_qc_start(&q, law, uqts, level, true));
    /* Run to well into ANSpcm. */
    run_tx(&q, &s, 1867 + 600 + 768 + 48 + 3000, 160);
    CHECK(v92_qc_state(&q) == V92_QC_ANSPCM, "state %s, want ANSpcm", v92_qc_state_name(v92_qc_state(&q)));
    qts_at = (int) q.qts_start;
    /* QCA1d is 1866.67 samples of bits behind one bit-time of lead-in carrier
     * (spandsp's FSK modulator fetches its first bit a bit period in). */
    CHECK(qts_at >= 1867 + 600 && qts_at <= 1867 + 600 + 30, "QTS starts at %d", qts_at);

    /* The 75 ms before QTS is exactly 600 codewords of +0. */
    for (int i = qts_at - 600; i < qts_at; i++)
        if (s.cw[i] == v91_ucode_to_codeword(law, 0, true))
            silent++;
    CHECK(silent == 600, "gap before QTS: %d of 600 are silence", silent);

    CHECK(v92_detect_qts_sequence(s.cw, s.n, law, uqts, qts_at - 20, s.n,
                                  (v92_codeword_to_ucode_fn) v91_codeword_to_ucode, &qts)
          && qts.qts_reps == 128 && qts.qts_bar_reps == 8,
          "QTS detect: seen=%d reps=%d bar=%d", qts.seen, qts.qts_reps, qts.qts_bar_reps);
    /* The first QTS codeword is +V, then +0. */
    CHECK(s.cw[qts_at] == v91_ucode_to_codeword(law, uqts, true)
          && s.cw[qts_at + 1] == v91_ucode_to_codeword(law, 0, true)
          && s.cw[qts_at + 3] == v91_ucode_to_codeword(law, uqts, false),
          "QTS symbol order");
    /* QTS\ is -QTS. */
    CHECK(s.cw[qts_at + 768] == v91_ucode_to_codeword(law, uqts, false)
          && s.cw[qts_at + 771] == v91_ucode_to_codeword(law, uqts, true),
          "QTS-bar polarity");

    memset(&pcm, 0, sizeof(pcm));
    CHECK(v92_anspcm_table_decode(s.cw, s.n, law, qts_at + 816, s.n, &pcm)
          && pcm.level == level && pcm.mismatches == 0 && !pcm.wrong_law,
          "ANSpcm: seen=%d level=%d mismatches=%d", pcm.seen, pcm.level, pcm.mismatches);
    v92_qc_free(&q);
    printf("PASS: sequence law=%d UQTS=%d level=%d\n", (int) law, uqts, level);
}

/* TONEq answers ANSpcm: silence for exactly 75 ms, then Phase 2. */
static void test_toneq(v91_law_t law)
{
    v92_qc_t q;
    stream_t s = { .n = 0 };
    uint8_t rx[160];
    double ph = 0;
    int n_after = 0;

    v92_qc_init(&q);
    assert(v92_qc_start(&q, law, 66, 1, false));
    run_tx(&q, &s, 1867 + 600 + 816 + 2000, 160);
    assert(v92_qc_state(&q) == V92_QC_ANSPCM);

    /* The far end hears ANSpcm and answers.  A short blip is not TONEq. */
    for (int b = 0; b < 3; b++) {              /* 30 ms of tone */
        tone_cw(law, rx, 80, 980.0, 1200.0, &ph);
        v92_qc_rx(&q, rx, 80);
    }
    memset(rx, v91_ucode_to_codeword(law, 0, true), sizeof(rx));
    v92_qc_rx(&q, rx, 160);
    CHECK(v92_qc_state(&q) == V92_QC_ANSPCM, "30 ms of 980 Hz must not be TONEq");

    /* 2100 Hz (our own ANSpcm in the echo) is not TONEq either. */
    for (int b = 0; b < 10; b++) {
        tone_cw(law, rx, 80, 2100.0, 1200.0, &ph);
        v92_qc_rx(&q, rx, 80);
    }
    CHECK(v92_qc_state(&q) == V92_QC_ANSPCM, "2100 Hz must not be TONEq");

    /* A tone too quiet to be a signal. */
    for (int b = 0; b < 12; b++) {
        tone_cw(law, rx, 80, 980.0, 30.0, &ph);
        v92_qc_rx(&q, rx, 80);
    }
    CHECK(v92_qc_state(&q) == V92_QC_ANSPCM, "a -50 dBm0 tone must not be TONEq");

    /* 100 ms of real TONEq. */
    for (int b = 0; b < 12; b++) {
        tone_cw(law, rx, 80, 980.0, 1200.0, &ph);
        v92_qc_rx(&q, rx, 80);
        if (v92_qc_state(&q) == V92_QC_SILENCE)
            break;
    }
    CHECK(v92_qc_state(&q) == V92_QC_SILENCE, "TONEq not detected");
    CHECK(q.toneq_seen, "toneq_seen");

    /* Silence runs 600 codewords and the call is told PHASE2 on the one it
     * stops at, mid-chunk. */
    {
        uint8_t out[160];
        int total = 0;
        int zeros = 0;

        while (v92_qc_running(&q)) {
            int got = v92_qc_tx(&q, out, 160);

            for (int i = 0; i < got; i++)
                if (out[i] == v91_ucode_to_codeword(law, 0, true))
                    zeros++;
            total += got;
            n_after++;
            if (got < 160)
                break;
        }
        CHECK(v92_qc_state(&q) == V92_QC_PHASE2, "state %s", v92_qc_state_name(v92_qc_state(&q)));
        CHECK(total == 600 && zeros == 600, "silence after TONEq: %d codewords (%d silent)", total, zeros);
        CHECK(v92_qc_tx(&q, out, 160) == 0, "no output once PHASE2");
    }
    (void) n_after;
    v92_qc_free(&q);
    printf("PASS: TONEq law=%d -> 75 ms silence -> Phase 2\n", (int) law);
}

/* No TONEq: ANSpcm runs to 2 s after QCA1d, then V.8. */
static void test_timeout(void)
{
    v92_qc_t q;
    uint8_t out[160];
    long total = 0;

    v92_qc_init(&q);
    assert(v92_qc_start(&q, V91_LAW_ULAW, 70, 2, false));
    while (v92_qc_running(&q)) {
        int got = v92_qc_tx(&q, out, 160);

        total += got;
        if (got < 160)
            break;
        if (total > 40000)
            break;
    }
    CHECK(v92_qc_state(&q) == V92_QC_V8, "state %s", v92_qc_state_name(v92_qc_state(&q)));
    /* 2 s counted from the end of QCA1d. */
    CHECK(q.since_qca == V92_QC_TONEQ_TIMEOUT, "since_qca %ld", q.since_qca);
    CHECK(total >= 1867 + 16000 && total <= 1867 + 16000 + 30, "ended at %ld", total);
    v92_qc_free(&q);
    printf("PASS: no TONEq within 2 s -> V.8 (after %ld codewords)\n", total);
}

/* Bad arguments leave the call on V.8. */
static void test_refusals(void)
{
    v92_qc_t q;

    v92_qc_init(&q);
    CHECK(!v92_qc_start(&q, V91_LAW_ULAW, -1, 1, false), "UQTS -1 accepted");
    CHECK(!v92_qc_start(&q, V91_LAW_ULAW, 66, 4, false), "level 4 accepted");
    CHECK(v92_qc_state(&q) == V92_QC_IDLE, "state after refusal");
    v92_qc_free(&q);
    printf("PASS: unusable arguments are refused\n");
}

/* 8.3.6: with QTS sent, Sd (whose first symbol is +W) must start in frame
 * interval 0 of the alignment QTS set; without QTS nothing constrains it. */
static void test_sd_alignment(void)
{
    for (int offset = -1; offset < 6; offset++) {
        v90_state_t *s = v90_init_data_pump(V90_LAW_ULAW);
        uint8_t cw;
        int n = 0, first_sd = -1, event_at = 12;

        assert(s);
        v90_set_sd_delay_ms(s, 0);
        v90_start_phase3(s, 40);
        while (n < 60 && first_sd < 0) {
            if (offset >= 0)
                v90_set_tx_frame_phase(s, (n + offset) % 6);
            if (n == event_at)
                (void) v90_handle_rx_event(s, V90_RX_EVENT_J);
            assert(v90_phase3_tx_codewords(s, &cw, 1) == 1);
            if (v90_get_tx_phase(s) == V90_TX_SD || cw != v91_ucode_to_codeword(V91_LAW_ULAW, 0, true)) {
                if (v90_get_tx_phase(s) == V90_TX_SD && cw != v91_ucode_to_codeword(V91_LAW_ULAW, 0, true))
                    first_sd = n;
            }
            n++;
        }
        CHECK(first_sd >= event_at, "offset %d: Sd never started (%d)", offset, first_sd);
        if (offset < 0)
            CHECK(first_sd <= event_at + 1, "unconstrained Sd should start at once (%d)", first_sd);
        else
            CHECK((first_sd + offset) % 6 == 0, "offset %d: Sd starts at codeword %d, frame interval %d",
                  offset, first_sd, (first_sd + offset) % 6);
        v90_free(s);
    }
    printf("PASS: Sd waits for the QTS frame boundary\n");
}

int main(void)
{
    test_sd_alignment();
    for (int law = 0; law < 2; law++) {
        test_qca1d((v91_law_t) law, 1, true);
        test_qca1d((v91_law_t) law, 3, false);
        test_sequence((v91_law_t) law, 66, 1);
        test_sequence((v91_law_t) law, 87, 0);
        test_sequence((v91_law_t) law, 61, 3);
        test_toneq((v91_law_t) law);
    }
    test_timeout();
    test_refusals();
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("v92_qc_test: all passed\n");
    return 0;
}
