/*
 * v92_p3_rx_line_test -- the V.92 Phase 3 upstream receiver against an
 * analogue modem on a real 2-wire loop.
 *
 * Every other V.92 receive test is fed a byte-exact DS0, so ISI, a fractional
 * sampling phase and a clock offset are invisible to them.  This one feeds
 * artifacts/v92-loop-upstream/live-rx.g711: the digital side's own receive
 * tap from a live call in the real topology (Apple USB modem as the V.92
 * analogue modem -> 2-wire loop -> VG224 -> SIP G.711 -> sip_v90_modem), cut
 * from artifacts/apple-v92-sip-r4.  The analogue side transmitted Ru, Ru-bar,
 * 2040T of TRN1u (V.92 8.5.7, scrambler zero-initialised) and then repeated
 * Ja for 1.5 s; the Ja carries the DIL descriptor it built,
 * "measurement-120x66" (N=120, LSP=12, LTP=11).
 *
 * Passing means 9.5.1.1.1-9.5.1.1.3 succeed: Ru, the Ru-to-Ru-bar transition,
 * TRN1u, and a CRC-valid Ja carrying that descriptor.  The fixture is required:
 * a missing file is a failure, not a skip.
 *
 * --expect-failure is for while docs/v92_p3_rx_line_plan.md is in progress:
 * it requires the part that works today (Ru and Ru-bar acquired at the
 * recorded positions, TRN1u entered) and the known failure (no Ja), so it is
 * green while the defect stands and loud both when acquisition regresses and
 * when the defect is fixed -- at which point this moves into `make test`
 * without the flag.
 *
 *   v92_p3_rx_line_test [--expect-failure] [fixture]
 */
#include "v92_p3_rx.h"
#include "v92_p3_eq.h"
#include "v92_analogue_audio.h"
#include "v92_line_channel.h"
#include "v90_dil_presets.h"

#include <spandsp.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIXTURE_DEFAULT   "artifacts/v92-loop-upstream/live-rx.g711"
#define FIXTURE_BYTES     20000
/* The engine armed the live receiver at G.711 sample 81440; the fixture
 * starts at 80000. */
#define ARM_SAMPLE        1440
/* Where today's receiver acquires Ru and Ru-bar on this fixture.  They are
 * the parts the loop does not break (a 1333 Hz line's signs survive ISI). */
#define RU1_EXPECTED      5008
#define UR1_EXPECTED      5272
#define POSITION_SLACK    16
/* TRN1u's first symbol (plan step 3), from tools/v92_trn1u_bound.py's 1-tap
 * fit: offset -31 from where today's receiver declared it (5323).  The
 * receiver's own reference correlation must land within START_SLACK. */
#define TRN1U_EXPECTED    5292
#define START_SLACK       2

static int first_entry[V92_P3_RX_FAILED + 1];

/* ------------------------------------------------------------------------
 * Plan step 4: the TRN1u equaliser (v92_p3_eq) run from the receiver's own
 * aligned TRN1u start.  Graded data-aided from symbol 256 (the end of the
 * block least-squares seed) to the end of the 2040T the analogue side sends.
 * ------------------------------------------------------------------------ */
#define EQ_AGREE_MIN      0.995
/* tools/v92_trn1u_bound.py --len 2040 --taps 41 at 5292: 7.6 dB with a
 * fixed filter and no timing tracking.  The plan asks for within 2 dB. */
#define EQ_FIXTURE_BOUND_DB 7.6

typedef struct {
    bool ran;
    double agree;      /* sign agreement, symbols 256..2039 */
    double snr_db;
    double ppm;
    int worst_x10;     /* lowest 256-symbol agreement seen after the seed */
    int main_tap;
} eq_result_t;

static double env_or(const char *name, double fallback)
{
    const char *v = getenv(name);

    return v && *v ? atof(v) : fallback;
}

static eq_result_t eq_run(const uint8_t *cw, int count, bool alaw, int start)
{
    v92_p3_eq_config_t cfg;
    v92_p3_eq_t eq;
    eq_result_t r = {0};

    v92_p3_eq_default_config(&cfg);
    /* Experiment knobs; the defaults are what is graded. */
    cfg.ntaps = (int)env_or("V92_P3_EQ_TAPS", cfg.ntaps);
    cfg.mu = env_or("V92_P3_EQ_MU", cfg.mu);
    cfg.timing_kp = env_or("V92_P3_EQ_KP", cfg.timing_kp);
    cfg.timing_ki = env_or("V92_P3_EQ_KI", cfg.timing_ki);
    cfg.timing = env_or("V92_P3_EQ_TIMING", 1) != 0;
    cfg.detector = (int)env_or("V92_P3_EQ_DET", cfg.detector);
    cfg.timing_avg = env_or("V92_P3_EQ_AVG", cfg.timing_avg);
    cfg.mu = env_or("V92_P3_EQ_MU", cfg.mu);
    if (start < 0 || !v92_p3_eq_init(&eq, &cfg))
        return r;
    eq.freq = env_or("V92_P3_EQ_FIXPPM", 0)*1e-6;
    /* Past TRN1u the equaliser runs decision-directed on Ja. */
    int run_to = (int)env_or("V92_P3_EQ_RUN_TO", cfg.trn_symbols);
    r.worst_x10 = 1000;
    for (int i = 0; i < count; i++) {
        v92_p3_eq_push(&eq, alaw ? alaw_to_linear(cw[i]) : ulaw_to_linear(cw[i]));
        if (i == start + 64 && !v92_p3_eq_start(&eq, start))
            return r;
        while (eq.started && eq.k < run_to && v92_p3_eq_step(&eq)) {
            if (getenv("V92_P3_EQ_TRACE") && eq.k % 128 == 0)
                fprintf(stderr, "EQTRACE k=%lld tau=%.4f ppm=%.1f det=%.5f agree=%.1f\n",
                        (long long)eq.k, eq.tau, eq.freq*1e6, eq.mm_avg,
                        v92_p3_eq_agree_x10(&eq)/10.0);
            if (eq.k > cfg.seed_symbols + V92_P3_EQ_AGREE_WINDOW
                && v92_p3_eq_agree_x10(&eq) < r.worst_x10)
                r.worst_x10 = v92_p3_eq_agree_x10(&eq);
        }
        if (eq.k >= run_to)
            break;
    }
    if (eq.k < cfg.trn_symbols)
        return r;
    r.ran = true;
    r.agree = v92_p3_eq_post_seed_agreement(&eq);
    r.snr_db = v92_p3_eq_snr_db(&eq);
    r.ppm = v92_p3_eq_ppm(&eq);
    r.main_tap = v92_p3_eq_main_tap(&eq);
    return r;
}

static void eq_print(const eq_result_t *r)
{
    if (!r->ran) {
        printf("eq did not run");
        return;
    }
    printf("eq %6.2f%% (worst 256: %5.1f%%) %5.1f dB %+6.0f ppm main tap %d",
           100.0*r->agree, r->worst_x10/10.0, r->snr_db, r->ppm, r->main_tap);
}

/* Plan step 5's control: the same fixture with the TRN1u start forced three
 * symbols late must be refused by the gate's start check.  The equaliser
 * alone would not refuse it -- a 31-tap fit absorbs the shift. */
static int fixture_wrong_start(const char *path)
{
    unsigned char cw[FIXTURE_BYTES];
    FILE *f = fopen(path, "rb");
    v92_p3_rx_t rx;
    v92_p3_rx_reject_t reason = V92_P3_RX_REJECT_NONE;
    int m0 = 0;
    int m1 = 0;

    if (!f || fread(cw, 1, sizeof(cw), f) != sizeof(cw)) {
        if (f)
            fclose(f);
        return 1;
    }
    fclose(f);
    v92_p3_rx_init(&rx);
    v92_p3_rx_start(&rx, ARM_SAMPLE);
    v92_p3_rx_set_md_length(&rx, 0);
    v92_p3_rx_set_law(&rx, 0);
    rx.test_start_offset = 3;
    for (int i = ARM_SAMPLE; i < FIXTURE_BYTES; i++) {
        (void)v92_p3_rx_feed(&rx, cw[i], i);
        if (rx.trn1u_align_done && rx.state != V92_P3_RX_TRN1U) {
            reason = v92_p3_rx_last_reject(&rx, NULL, &m0, &m1);
            break;
        }
    }
    printf("  start + 3: %s (m0=%d m1=%d), gate %s\n",
           v92_p3_rx_reject_name(reason), m0, m1,
           rx.eq_gate_done ? "PASSED -- WRONG" : "refused");
    if (rx.eq_gate_done || reason != V92_P3_RX_REJECT_TRN1U_START) {
        printf("FAIL: the gate accepted a TRN1u start three symbols out\n");
        return 1;
    }
    return 0;
}

static int fixture_row(const char *path, bool expect_failure)
{
    unsigned char *cw;
    long len;
    FILE *f;
    v92_p3_rx_t rx;
    int last_state = -1;
    v92_p3_rx_reject_t reason;
    int reject_sample = -1;
    int m0 = 0;
    int m1 = 0;
    bool acquired;
    bool decoded = false;
    int trn1u_start = -1;
    bool aligned;
    eq_result_t eqr;
    bool trained;
    int gate = -1;       /* first TRN1u: 1 passed the gate, 0 rejected */

    printf("fixture %s\n", path);
    f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "FAIL: fixture %s missing -- it is tracked with "
                        "git add -f; this test does not skip\n", path);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len != FIXTURE_BYTES) {
        fprintf(stderr, "FAIL: fixture %s is %ld bytes, expected %d\n",
                path, len, FIXTURE_BYTES);
        fclose(f);
        return 1;
    }
    cw = malloc((size_t)len);
    if (!cw || fread(cw, 1, (size_t)len, f) != (size_t)len) {
        fprintf(stderr, "FAIL: short read on %s\n", path);
        fclose(f);
        return 1;
    }
    fclose(f);

    for (int s = 0; s <= V92_P3_RX_FAILED; s++)
        first_entry[s] = -1;

    v92_p3_rx_init(&rx);
    v92_p3_rx_start(&rx, ARM_SAMPLE);
    v92_p3_rx_set_md_length(&rx, 0);   /* INFO1a reported MD=0 */
    v92_p3_rx_set_law(&rx, 0);         /* the call was PCMU */
    for (int i = ARM_SAMPLE; i < (int)len; i++) {
        int state;

        (void)v92_p3_rx_feed(&rx, cw[i], i);
        if (trn1u_start < 0 && rx.trn1u_align_done)
            trn1u_start = rx.trn1u_start;
        if (gate < 0 && rx.trn1u_align_done) {
            if (rx.eq_gate_done)
                gate = 1;
            else if (rx.state != V92_P3_RX_TRN1U)
                gate = 0;
        }
        state = (int)v92_p3_rx_get_state(&rx);
        if (state != last_state) {
            printf("  sample %6d (%6.3f s) %s\n", i, i/8000.0,
                   v92_p3_rx_state_name((v92_p3_rx_state_t)state));
            if (first_entry[state] < 0)
                first_entry[state] = i;
            last_state = state;
        }
        if (state == V92_P3_RX_DONE || state == V92_P3_RX_FAILED)
            break;
    }
    eqr = eq_run(cw, (int)len, false, trn1u_start);
    free(cw);

    reason = v92_p3_rx_last_reject(&rx, &reject_sample, &m0, &m1);
    printf("  final %s, %d rejects, last %s at %d (m0=%d m1=%d)\n",
           v92_p3_rx_state_name(v92_p3_rx_get_state(&rx)),
           rx.reject_count, v92_p3_rx_reject_name(reason),
           reject_sample, m0, m1);

    aligned = trn1u_start >= 0
           && abs(trn1u_start - TRN1U_EXPECTED) <= START_SLACK;
    printf("  TRN1u start %d by reference correlation (expected %d +/-%d)%s\n",
           trn1u_start, TRN1U_EXPECTED, START_SLACK, aligned ? "" : "  WRONG");
    if (!aligned) {
        printf("FAIL: TRN1u start not found where it is\n");
        return 1;
    }
    trained = eqr.ran && eqr.agree >= EQ_AGREE_MIN
           && eqr.snr_db >= EQ_FIXTURE_BOUND_DB - 2.0;
    printf("  TRN1u equaliser: ");
    eq_print(&eqr);
    printf(" (need >= %.1f%%, >= %.1f dB)%s\n", 100.0*EQ_AGREE_MIN,
           EQ_FIXTURE_BOUND_DB - 2.0, trained ? "" : "  FAIL");
    if (!trained) {
        printf("FAIL: the TRN1u equaliser does not train on the fixture\n");
        return 1;
    }
    printf("  TRN1u gate (plan step 5): %s\n", gate == 1 ? "passed" : "REJECTED");
    if (gate != 1) {
        printf("FAIL: the fixture's TRN1u does not pass the gate\n");
        return 1;
    }

    if (v92_p3_rx_ja_ok(&rx)) {
        const ja_dil_decode_t *ja = v92_p3_rx_get_ja(&rx);

        printf("  Ja at %d: N=%u LSP=%u LTP=%u\n", ja->start_sample,
               (unsigned)ja->desc.n, (unsigned)ja->desc.lsp,
               (unsigned)ja->desc.ltp);
        decoded = ja->desc.n == 120 && ja->desc.lsp == 12
               && ja->desc.ltp == 11;
        if (!decoded)
            printf("  Ja decoded but is not the descriptor the analogue "
                   "side sent (N=120 LSP=12 LTP=11)\n");
    }

    acquired = first_entry[V92_P3_RX_RU1] >= 0
            && abs(first_entry[V92_P3_RX_RU1] - RU1_EXPECTED) <= POSITION_SLACK
            && first_entry[V92_P3_RX_UR1] >= 0
            && abs(first_entry[V92_P3_RX_UR1] - UR1_EXPECTED) <= POSITION_SLACK
            && first_entry[V92_P3_RX_TRN1U] >= 0;

    if (!expect_failure) {
        if (decoded) {
            printf("PASS: V.92 Phase 3 upstream decoded off a real loop\n");
            return 0;
        }
        printf("FAIL: no CRC-valid Ja with the expected DIL descriptor\n");
        return 1;
    }
    if (decoded) {
        printf("FAIL (expected failure did not occur): Ja now decodes -- "
               "move this test into `make test` without --expect-failure\n");
        return 1;
    }
    if (!acquired) {
        printf("FAIL: regression -- Ru/Ru-bar no longer acquired at "
               "%d/%d (+/-%d), or TRN1u never entered\n",
               RU1_EXPECTED, UR1_EXPECTED, POSITION_SLACK);
        return 1;
    }
    printf("PASS (expected failure): Ru/Ru-bar acquired, TRN1u trained and "
           "through the gate, no Ja from raw signs -- "
           "docs/v92_p3_rx_line_plan.md step 6\n");
    return 0;
}

/* ------------------------------------------------------------------------
 * Synthetic rows: our own analogue Phase 3 transmitter through a modelled
 * loop (v92_line_channel) into the network ADC and the receiver.
 *
 * The analogue side's Ru, Ru-bar, TRN1u and Ja need nothing from the digital
 * side until Sd (9.5.2.1.1-9.5.2.1.3), so the analogue core can run alone with
 * a silent receive path, exactly as v92_startup_test drives it up to there.
 * ------------------------------------------------------------------------ */

#define SYN_SYMBOLS       16000   /* Ru..TRN1u is ~2.5k; Ja repeats to 12000T */
#define SYN_LU            6000.0
/* Where TRN1u's first symbol reaches the receiver, from
 * tools/v92_trn1u_bound.py's 1-tap fit on each dumped row (--dump-row): 976
 * with no loop, 979-981 through the r4 loop's delay. */
#define SYN_TRN1U_START   976
#define SYN_R4_DELAY      4
#define SYN_CAPTURE       4000    /* covers TRN1u (976..3016) and margin */

typedef struct {
    const char *name;
    bool alaw;
    bool isi;                     /* the r4 loop FIR */
    double phase;
    double ppm;
    double snr_db;                /* re L_U; 0 = no noise */
    bool antialias;               /* the A/D's own filter (no r4 taps) */
    bool expect_pass_today;       /* outcome before plan steps 3-6 */
} syn_row_t;

static const syn_row_t syn_rows[] = {
    /* The raw sign slicer survives noise on its own; with the r4 loop's ISI
     * its raw-sign Ja search fails, as on the real recording.  Rows
     * that move the sampling instant without the r4 taps (which hold the
     * real codec's filter) get the A/D's anti-alias filter (see
     * v92_line_channel.h) -- without it they modelled an A/D that folds
     * the analogue modem's images above 4 kHz back in, and "passed" only
     * because that fold is a fixed map while the sampling is synchronous.
     * With it a fractional phase alone defeats the raw slicer's Ja search
     * too.  The +200 ppm one decodes Ja from raw signs; it used to be
     * stopped only by the descrambled-ones TRN1u check (65%), which plan
     * step 5 replaced. */
    /* name                          law    isi    phase  ppm    snr  aa today */
    {"ideal u-law",                  false, false, 0.00,    0.0,  0.0, false, true},
    {"ideal A-law",                  true,  false, 0.00,    0.0,  0.0, false, true},
    {"phase 0.5 only, A/D filter",   false, false, 0.50,    0.0,  0.0, true,  false},
    {"+200 ppm only, A/D filter",    false, false, 0.00,  200.0,  0.0, true,  true},
    {"noise 25 dB only",             false, false, 0.00,    0.0, 25.0, false, true},
    {"r4 loop",                      false, true,  0.00,    0.0,  0.0, false, false},
    {"r4 loop, phase 0.25",          false, true,  0.25,    0.0,  0.0, false, false},
    {"r4 loop, phase 0.50",          false, true,  0.50,    0.0,  0.0, false, false},
    {"r4 loop, phase 0.75",          false, true,  0.75,    0.0,  0.0, false, false},
    {"r4 loop, +200 ppm",            false, true,  0.00,  200.0,  0.0, false, false},
    {"r4 loop, -200 ppm",            false, true,  0.00, -200.0,  0.0, false, false},
    {"r4 loop, noise 25 dB",         false, true,  0.00,    0.0, 25.0, false, false},
    {"r4 loop, A-law, ph .5, 163ppm",true,  true,  0.50,  163.0, 25.0, false, false},
};

static v92a_config_t syn_config(bool alaw)
{
    v92a_config_t cfg = {
        .law = alaw ? V90_LAW_ALAW : V90_LAW_ULAW, .u_info = 78,
        .lu = SYN_LU, .digital_max_tx_dbm0 = -13, .upstream_rate_mask = 1,
        .round_trip_symbols = 16,
    };
    if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil))
        abort();
    return cfg;
}

/* Our own analogue upstream at 16 kHz, in the core linear units the
 * network ADC quantises (int16 little-endian), for fitting the loop. */
static int dump_tx(const char *path)
{
    v92a_config_t cfg = syn_config(false);
    v92a_audio_t *fe = v92a_audio_init_rate(&cfg, V92_AUDIO_RATE);
    FILE *f = fopen(path, "wb");
    int last = -1;

    if (!fe || !f)
        return 1;
    for (int i = 0; i < 6000; i++) {
        int16_t up[V92_AUDIO_PER_SYMBOL];
        int16_t silence[V92_AUDIO_PER_SYMBOL] = {0};
        int stage = (int)v92a_stage(v92a_audio_core(fe));

        if (stage != last) {
            fprintf(stderr, "symbol %d: analogue stage %d\n", i, stage);
            last = stage;
        }
        v92a_audio_tx(fe, up, V92_AUDIO_PER_SYMBOL);
        v92a_audio_rx(fe, silence, V92_AUDIO_PER_SYMBOL);
        for (int k = 0; k < V92_AUDIO_PER_SYMBOL; k++) {
            int16_t v = (int16_t)(up[k]*V92_AUDIO_LINEAR_SCALE);
            fwrite(&v, sizeof(v), 1, f);
        }
    }
    fclose(f);
    v92a_audio_free(fe);
    return 0;
}

static bool syn_run(const syn_row_t *row, int *ja_sample, int *rejects,
                    v92_p3_rx_reject_t *last, int *trn1u_start, FILE *dump,
                    uint8_t *capture, int *captured)
{
    v92a_config_t cfg = syn_config(row->alaw);
    v92a_audio_t *fe = v92a_audio_init_rate(&cfg, V92_AUDIO_RATE);
    v92_line_channel_config_t cc = {
        .taps = row->isi ? v92_line_channel_r4_taps : NULL,
        .ntaps = row->isi ? v92_line_channel_r4_ntaps : 0,
        .phase = row->phase, .ppm = row->ppm,
        .noise_rms = row->snr_db > 0.0 ? SYN_LU/pow(10.0, row->snr_db/20.0) : 0.0,
        .seed = 12345,
        .antialias = row->antialias,
    };
    v92_line_channel_t ch;
    v92_p3_rx_t rx;
    int idx = 0;
    bool ok = false;

    if (!fe || !v92_line_channel_init(&ch, &cc))
        abort();
    v92_p3_rx_init(&rx);
    v92_p3_rx_start(&rx, 0);
    v92_p3_rx_set_md_length(&rx, 0);
    v92_p3_rx_set_law(&rx, row->alaw);
    *ja_sample = -1;
    *trn1u_start = -1;
    /* A dumped row runs to the end so the whole of Ja is recorded. */
    if (captured)
        *captured = 0;
    /* A captured row runs until the equaliser has all of TRN1u. */
    for (int i = 0; i < SYN_SYMBOLS && (!ok || dump || (capture && i < SYN_CAPTURE)); i++) {
        int16_t up[V92_AUDIO_PER_SYMBOL];
        int16_t silence[V92_AUDIO_PER_SYMBOL] = {0};
        double in[V92_AUDIO_PER_SYMBOL];
        double adc[4];
        int n;

        v92a_audio_tx(fe, up, V92_AUDIO_PER_SYMBOL);
        v92a_audio_rx(fe, silence, V92_AUDIO_PER_SYMBOL);
        for (int k = 0; k < V92_AUDIO_PER_SYMBOL; k++)
            in[k] = up[k]*V92_AUDIO_LINEAR_SCALE;
        n = v92_line_channel_put(&ch, in, V92_AUDIO_PER_SYMBOL, adc, 4);
        for (int k = 0; k < n; k++) {
            double v = floor(adc[k] + 0.5);
            int16_t s = (int16_t)(v > 32767.0 ? 32767.0
                                  : v < -32768.0 ? -32768.0 : v);
            uint8_t cw = row->alaw ? linear_to_alaw(s) : linear_to_ulaw(s);

            if (dump)
                fputc(cw, dump);
            if (capture && *captured < SYN_CAPTURE)
                capture[(*captured)++] = cw;
            v92_p3_rx_feed(&rx, cw, idx++);
            if (*trn1u_start < 0 && rx.trn1u_align_done)
                *trn1u_start = rx.trn1u_start;
        }
        if (v92_p3_rx_ja_ok(&rx)) {
            const ja_dil_decode_t *ja = v92_p3_rx_get_ja(&rx);

            ok = ja->desc.n == cfg.dil.n && ja->desc.lsp == cfg.dil.lsp
              && ja->desc.ltp == cfg.dil.ltp;
            *ja_sample = ja->start_sample;
            if (!dump && !(capture && i < SYN_CAPTURE))
                break;
        }
    }
    *rejects = rx.reject_count;
    *last = v92_p3_rx_last_reject(&rx, NULL, NULL, NULL);
    v92a_audio_free(fe);
    return ok;
}

static int synthetic_rows(bool expect_failure)
{
    int failures = 0;

    printf("synthetic loop rows (r4 FIR: %d taps at 16 kHz)\n",
           v92_line_channel_r4_ntaps);
    for (size_t r = 0; r < sizeof(syn_rows)/sizeof(syn_rows[0]); r++) {
        const syn_row_t *row = &syn_rows[r];
        const char *only = getenv("V92_P3_LINE_ROW");

        if (only && *only && atoi(only) != (int)r)
            continue;
        int ja_sample;
        int rejects;
        int start;
        int want_start = SYN_TRN1U_START + (row->isi ? SYN_R4_DELAY : 0)
                       + (row->antialias ? V92_LINE_CHANNEL_AA_DELAY_SYMBOLS : 0);
        v92_p3_rx_reject_t last;
        static uint8_t capture[SYN_CAPTURE];
        int captured;
        bool ok = syn_run(row, &ja_sample, &rejects, &last, &start, NULL,
                          capture, &captured);
        eq_result_t eqr = eq_run(capture, captured, row->alaw, start);
        bool trained = eqr.ran && eqr.agree >= EQ_AGREE_MIN;
        bool wanted = expect_failure ? row->expect_pass_today : true;
        bool aligned = start >= 0 && abs(start - want_start) <= START_SLACK;
        const char *verdict = !aligned ? "FAIL (TRN1u start)"
                            : !trained ? "FAIL (equaliser)"
                            : ok == wanted ? "ok"
                            : ok ? "UNEXPECTED PASS" : "FAIL";

        printf("  %-30s %s  TRN1u %4d/%4d  Ja %s (sample %d, %d rejects, "
               "last %s)  %s\n",
               row->name, row->alaw ? "A" : "u", start, want_start,
               ok ? "decoded" : "missing", ja_sample, rejects,
               v92_p3_rx_reject_name(last), verdict);
        printf("  %-30s    ", "");
        eq_print(&eqr);
        printf("\n");
        if (ok != wanted || !aligned || !trained)
            failures++;
    }
    if (failures && expect_failure)
        printf("  %d row(s) differ from today's expected outcome; if a row "
               "now passes, update expect_pass_today\n", failures);
    return failures ? 1 : 0;
}

int main(int argc, char **argv)
{
    bool expect_failure = false;
    const char *path = FIXTURE_DEFAULT;
    int rc;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--expect-failure") == 0)
            expect_failure = true;
        else if (strcmp(argv[i], "--dump-tx") == 0 && i + 1 < argc)
            return dump_tx(argv[++i]);
        else if (strcmp(argv[i], "--dump-row") == 0 && i + 2 < argc) {
            /* The G.711 stream one synthetic row feeds the receiver, for
             * offline analysis (tools/v92_trn1u_bound.py). */
            int r = atoi(argv[++i]);
            FILE *f = fopen(argv[++i], "wb");
            int ja_sample, rejects, start;
            v92_p3_rx_reject_t last;

            if (!f || r < 0
                || r >= (int)(sizeof(syn_rows)/sizeof(syn_rows[0])))
                return 2;
            printf("%s: Ja %s\n", syn_rows[r].name,
                   syn_run(&syn_rows[r], &ja_sample, &rejects, &last, &start, f,
                           NULL, NULL)
                   ? "decoded" : "missing");
            fclose(f);
            return 0;
        }
        else
            path = argv[i];
    }
    rc = fixture_row(path, expect_failure);
    rc |= fixture_wrong_start(path);
    rc |= synthetic_rows(expect_failure);
    return rc;
}
