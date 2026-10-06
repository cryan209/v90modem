/*
 * v92_p3_rx.c — V.92 Phase 3 upstream receiver (PCM domain)
 *
 * See v92_p3_rx.h for protocol overview and design notes.
 */

#include "v92_p3_rx.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <spandsp.h>

/* -------------------------------------------------------------------------
 * Internal constants
 * ------------------------------------------------------------------------- */

/* Minimum run before promoting a period-6 lock to Ru. */
#define P6_LOCK_MIN    320
/* Soft-run lock candidate threshold when noise breaks strict runs. */
#define P6_LOCK_MIN_SOFT 128
/* After this many symbols, allow brief mismatch streaks without reset. */
#define P6_SOFT_GLITCH_FLOOR 12
/* Maximum consecutive mismatches tolerated by soft-run tracker. */
#define P6_SOFT_GLITCH_MAX 4
/* LU-quality gates to reject sign-only false locks. */
#define P6_LU_MEAN_MIN   64.0
#define P6_LU_RANGE_MAX  16
#define P6_LU_STD_MAX     6.0

/* Acceptance window for Ru burst length. */
#define RU_ACCEPT_MIN  (V92_P3_RX_RU_T - 24)
#define RU_ACCEPT_MAX  (V92_P3_RX_RU_T + 72)
/* Hard cap to avoid latching forever on unrelated long period-6 regions. */
#define RU_MAX_T       (V92_P3_RX_RU_T * 8)
/* Folded-lock fallback: Ru/uR/Ru can appear as one long contiguous run. */
#define RU_FOLDED_EXPECT_T       (V92_P3_RX_RU_T + V92_P3_RX_UR_T + V92_P3_RX_RU_T)
#define RU_FOLDED_MIN_T          (RU_FOLDED_EXPECT_T - 96)
#define RU_FOLDED_MAX_T          (RU_FOLDED_EXPECT_T + 96)
#define RU_FOLDED_UR2_ANCHOR_TOL 96

/* Acceptance window for uR burst length. */
#define UR_ACCEPT_MIN  (V92_P3_RX_UR_T - 8)
#define UR_ACCEPT_MAX  (V92_P3_RX_UR_T * 2 + 8)
/* Relaxed bounds used only when lock entered via soft run tracking. */
#define UR_ACCEPT_MIN_SOFT 3
#define UR_ACCEPT_MAX_SOFT 64
/* Hard cap for uR; true uR should be short. */
#define UR_MAX_T       (V92_P3_RX_UR_T * 6)
/* Allow brief entry glitches before rejecting uR lock. */
#define UR_RELOCK_GRACE 6
/* Relaxed Ru bounds/lock for the second Ru in soft mode. */
#define RU_ACCEPT_MIN_SOFT 96
#define RU2_LOCK_MIN_SOFT  96
/* Descrambled ones over the first 256 TRN1u symbols: a diagnostic only. */
#define TRN1U_EARLY_CHECK_T 256
/* TRN1u start alignment (docs/v92_p3_rx_line_plan.md step 3).  The uR->TRN1u
 * transition is declared from the period-6 run trackers, which need a run of
 * off-pattern symbols before they let go, so the receiver enters TRN1u ~31
 * symbols after TRN1u actually began -- on a byte-exact channel as well as
 * off the loop.  TRN1u's signs are known from its first symbol (8.5.7), so
 * the start is found instead by correlating ALIGN_LEN received signs against
 * the reference at every offset within +/-ALIGN_SPAN of the nominal start.
 * Signs only: the receiver is not told the G.711 law, and the MSB is the
 * sign in both.  ALIGN_MIN is the normalised peak below which the nominal
 * start is kept: a reference against itself scores 1.0, its own sidelobes
 * over 256 symbols reach ~0.2, and the r4 loop's peaks measure 0.57-0.74. */
#define TRN1U_ALIGN_SPAN 64
#define TRN1U_ALIGN_LEN  256
#define TRN1U_ALIGN_MIN_X1000 300
/* The TRN1u gate (plan step 5).  It replaces the descrambled-ones check,
 * which roughly triples each sign error and so read 48% on a loop the
 * equaliser recovers at 99.9%; that figure is still computed and reported.
 *
 * Two parts, because the equaliser cannot police its own start: a 31-tap
 * least-squares fit absorbs a start a few symbols out by moving its main
 * tap, and agrees with the reference just as well.  So the start is judged
 * by the 1-tap correlation score AT THE START IN USE -- peaks measure
 * 0.57-1.0 on the fixture and every synthetic row, the neighbouring
 * offsets at most 0.41, and three symbols off at most 0.14 -- and training
 * by the equaliser's sign agreement over the 256 symbols after its seed,
 * which are out of sample. */
#define TRN1U_START_SCORE_MIN_X1000 500
#define TRN1U_AGREE_MIN_X10 950
/* With the equaliser switched off (v92_p3_rx_set_equaliser), the old
 * metric: descrambled ones over the first 256 symbols. */
#define TRN1U_NO_EQ_ONES_MIN_PCT 75
/* V.92 9.5.1.1.3: the digital modem conditions its receiver for Ja only
 * "after receiving the first 2040T of signal TRNlu", and Figure 10 gives
 * TRN1u as >2040T -- so 2040 is guaranteed by the peer, not a target to
 * relax.  This was 256, which started the Ja search inside the peer's own
 * TRN1u and could therefore only ever return a soft lock on training data
 * (observed live: trn1u->ja_search after 280 samples, reject=ja_soft_only).
 * Soft mode still relaxes the *lock* thresholds; it must not shorten this. */
#define TRN1U_MIN_SOFT_T V92_P3_RX_TRN1U_MIN_T
#define JA_LEAD_SOFT_T 24

/* -------------------------------------------------------------------------
 * Sign bit helpers
 * ------------------------------------------------------------------------- */

static inline int sign_bit(uint8_t cw)  { return (cw >> 7) & 1; }
static inline int amp7(uint8_t cw)      { return cw & 0x7F; }

static int p3rx_debug_enabled(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *v = getenv("V92_P3_RX_DEBUG");
        cached = (v && *v && *v != '0') ? 1 : 0;
    }
    return cached;
}

static void p3rx_set_reject(v92_p3_rx_t *rx,
                            v92_p3_rx_reject_t reason,
                            int sample_index,
                            int metric0,
                            int metric1)
{
    if (!rx || reason == V92_P3_RX_REJECT_NONE)
        return;
    rx->reject_count++;
    rx->last_reject = reason;
    rx->last_reject_sample = sample_index;
    rx->last_reject_metric0 = metric0;
    rx->last_reject_metric1 = metric1;
    if (p3rx_debug_enabled()) {
        fprintf(stderr,
                "[P3RX] sample=%d reject=%s m0=%d m1=%d total=%d\n",
                sample_index,
                v92_p3_rx_reject_name(reason),
                metric0,
                metric1,
                rx->reject_count);
    }
}

/*
 * p6_expected() — expected G.711 MSB at period-6 phase p.
 * Ru:  phase 0,1,2 → 1 (+L_U / positive), phase 3,4,5 → 0 (−L_U)
 * uR:  inverted.
 */
static inline int p6_exp(int phase, bool ru_pol)
{
    int ru = (phase < 3) ? 1 : 0;
    return ru_pol ? ru : (1 - ru);
}

/* -------------------------------------------------------------------------
 * Period-6 tracker — 12 hypotheses (6 phases x 2 polarities)
 * ------------------------------------------------------------------------- */

static inline bool p6_hyp_pol(int h)
{
    return (h / 6) == 0;
}

static inline int p6_hyp_phase0(int h)
{
    return h % 6;
}

static inline int p6_hyp_index(bool ru_pol, int phase0)
{
    return (ru_pol ? 0 : 6) + (phase0 % 6);
}

static void p6_update_hyp_runs(v92_p3_rx_t *rx, uint8_t cw, int sample_index)
{
    int msb = sign_bit(cw);
    int a7 = amp7(cw);
    for (int h = 0; h < 12; h++) {
        bool pol = p6_hyp_pol(h);
        int phase = (sample_index + p6_hyp_phase0(h)) % 6;
        int expected = p6_exp(phase, pol);

        if (msb == expected) {
            if (rx->p6_hyp_run[h] == 0) {
                rx->p6_hyp_sum[h] = 0;
                rx->p6_hyp_sumsq[h] = 0;
                rx->p6_hyp_min[h] = (uint8_t) a7;
                rx->p6_hyp_max[h] = (uint8_t) a7;
            }
            rx->p6_hyp_run[h]++;
            rx->p6_hyp_sum[h] += (uint32_t) a7;
            rx->p6_hyp_sumsq[h] += (uint32_t) (a7 * a7);
            if (a7 < rx->p6_hyp_min[h])
                rx->p6_hyp_min[h] = (uint8_t) a7;
            if (a7 > rx->p6_hyp_max[h])
                rx->p6_hyp_max[h] = (uint8_t) a7;
            rx->p6_hyp_soft_run[h]++;
            rx->p6_hyp_soft_bad[h] = 0;
        } else {
            rx->p6_hyp_run[h] = 0;
            rx->p6_hyp_sum[h] = 0;
            rx->p6_hyp_sumsq[h] = 0;
            rx->p6_hyp_min[h] = 0x7F;
            rx->p6_hyp_max[h] = 0;
            if (rx->p6_hyp_soft_run[h] >= P6_SOFT_GLITCH_FLOOR
                && rx->p6_hyp_soft_bad[h] < P6_SOFT_GLITCH_MAX) {
                rx->p6_hyp_soft_bad[h]++;
                rx->p6_hyp_soft_run[h]++;
            } else {
                rx->p6_hyp_soft_run[h] = 0;
                rx->p6_hyp_soft_bad[h] = 0;
            }
        }
    }
}

static bool p6_hyp_lu_ok(const v92_p3_rx_t *rx, int h)
{
    int run;
    double mean;
    int range;
    double var;
    double stddev;

    if (!rx || h < 0 || h >= 12)
        return false;
    run = rx->p6_hyp_run[h];
    if (run <= 0)
        return false;

    mean = (double) rx->p6_hyp_sum[h] / (double) run;
    range = (int) rx->p6_hyp_max[h] - (int) rx->p6_hyp_min[h];
    var = (double) rx->p6_hyp_sumsq[h] / (double) run - mean * mean;
    if (var < 0.0)
        var = 0.0;
    stddev = sqrt(var);

    return (mean >= P6_LU_MEAN_MIN
            && range <= P6_LU_RANGE_MAX
            && stddev <= P6_LU_STD_MAX);
}

static bool p6_hyp_lu_stats(const v92_p3_rx_t *rx,
                            int h,
                            double *mean_out,
                            int *range_out,
                            double *std_out,
                            bool *ok_out)
{
    int run;
    double mean;
    int range;
    double var;
    double stddev;
    bool ok;

    if (!rx || h < 0 || h >= 12)
        return false;
    run = rx->p6_hyp_run[h];
    if (run <= 0)
        return false;

    mean = (double) rx->p6_hyp_sum[h] / (double) run;
    range = (int) rx->p6_hyp_max[h] - (int) rx->p6_hyp_min[h];
    var = (double) rx->p6_hyp_sumsq[h] / (double) run - mean * mean;
    if (var < 0.0)
        var = 0.0;
    stddev = sqrt(var);
    ok = (mean >= P6_LU_MEAN_MIN
          && range <= P6_LU_RANGE_MAX
          && stddev <= P6_LU_STD_MAX);

    if (mean_out)
        *mean_out = mean;
    if (range_out)
        *range_out = range;
    if (std_out)
        *std_out = stddev;
    if (ok_out)
        *ok_out = ok;
    return true;
}

static int p6_best_hyp_run(const v92_p3_rx_t *rx, bool ru_pol, int min_run, bool require_lu)
{
    int best_h = -1;
    int best_r = min_run - 1;
    int base = ru_pol ? 0 : 6;

    for (int p = 0; p < 6; p++) {
        int h = base + p;
        int r = rx->p6_hyp_run[h];
        if (require_lu && !p6_hyp_lu_ok(rx, h))
            continue;
        if (r > best_r) {
            best_r = r;
            best_h = h;
        }
    }
    return best_h;
}

static int p6_best_hyp_soft_run(const v92_p3_rx_t *rx, bool ru_pol, int min_run)
{
    int best_h = -1;
    int best_r = min_run - 1;
    int base = ru_pol ? 0 : 6;

    for (int p = 0; p < 6; p++) {
        int h = base + p;
        int r = rx->p6_hyp_soft_run[h];
        if (r > best_r) {
            best_r = r;
            best_h = h;
        }
    }
    return best_h;
}

static void p6_reset(v92_p3_rx_t *rx)
{
    rx->p6_phase      = 0;
    rx->p6_locked     = false;
    rx->p6_run        = 0;
    rx->p6_err_window = 0;
    rx->p6_err_wpos   = 0;
    rx->p6_ru_polarity = true;
    memset(rx->p6_hyp_run, 0, sizeof(rx->p6_hyp_run));
    memset(rx->p6_hyp_soft_run, 0, sizeof(rx->p6_hyp_soft_run));
    memset(rx->p6_hyp_sum, 0, sizeof(rx->p6_hyp_sum));
    memset(rx->p6_hyp_sumsq, 0, sizeof(rx->p6_hyp_sumsq));
    memset(rx->p6_hyp_soft_bad, 0, sizeof(rx->p6_hyp_soft_bad));
    for (int h = 0; h < 12; h++) {
        rx->p6_hyp_min[h] = 0x7F;
        rx->p6_hyp_max[h] = 0;
    }
    rx->ru_hyp = -1;
    rx->ur_hyp = -1;
    rx->p6_soft_mode = false;
    rx->hunt_best_run = 0;
    rx->hunt_best_hyp = -1;
    rx->hunt_best_start = -1;
    rx->hunt_best_lu_ok = 0;
    rx->hunt_best_mean_x10 = 0;
    rx->hunt_best_range = 0;
    rx->hunt_best_std_x10 = 0;
}

/* -------------------------------------------------------------------------
 * Upstream GPA descrambler (1 + x^-5 + x^-23, delay taps 5/23 → reg>>4 ^
 * reg>>22)
 * -------------------------------------------------------------------------
 * V.92 §6.3 mandates GPA (eq 7-2/V.34) for everything the ANALOGUE modem
 * transmits.  This read reg>>17 (delay tap 18 = GPC) until 2026-07-23: the
 * "x^18" in GPA's positive-power form x^23 + x^18 + 1 had been misread as
 * a delay tap.  Tap ground truth is in-tree and live-validated: spandsp
 * v34tx.c assigns the answerer tx.scrambler_tap = 4, and v34rx.c's V.90
 * branch (info1a code 6) records that SmartLink's real upstream resolves
 * at 96-99% ones with tap 4 vs ~52% with tap 17 — the same peer and the
 * same analogue-modem-GPA mandate as here (V.90 §6.5 ≡ V.92 §6.3).  The
 * pre-fix TRN1u all-ones ceiling of ~53% (docs/v92_pcm_upstream_findings.md)
 * is that identical wrong-tap signature.
 * Self-synchronising: shift register is updated with the raw INPUT bit.
 */
static inline int gpa_descramble(uint32_t *reg, int in_bit)
{
    int out = (in_bit ^ (int)(*reg >> 22) ^ (int)(*reg >> 4)) & 1;
    *reg = (*reg << 1) | (uint32_t)in_bit;
    return out;
}

/* -------------------------------------------------------------------------
 * TRN1u single-sample processor
 *
 * V.92 8.5.7: GPA-scrambled ones directly select +/-LU. TRN1u is
 * NOT differential; 8.5.4 introduces differential encoding at Ja. Keep
 * the last sign for diagnostics but feed the absolute sign to GPA.
 * ------------------------------------------------------------------------- */
static int trn1u_process(v92_p3_rx_t *rx, uint8_t cw)
{
    int v92_bit = 1 - sign_bit(cw); /* 0 positive, 1 negative */
    if (rx->trn1u_inverted)
        v92_bit ^= 1;
    rx->diff_prev = v92_bit;
    rx->diff_valid = true;
    int out = gpa_descramble(&rx->gpa_reg, v92_bit);
    rx->trn1u_count++;
    if (out) rx->trn1u_ones++;
    if (rx->trn1u_count == TRN1U_EARLY_CHECK_T)
        rx->trn1u_ones_early = rx->trn1u_ones;
    return out;
}

/* The TRN1u sign reference, V.92 8.5.7: the GPA scrambler (6.3, delay taps 5
 * and 23) initialised to zero and fed binary ones, output 0 -> +L_U.  Element
 * k is +1 or -1 for TRN1u symbol k. */
static void trn1u_reference(int8_t *ref, int n)
{
    uint32_t reg = 0;

    for (int k = 0; k < n; k++) {
        int b = 1 ^ (int)((reg >> 4) & 1) ^ (int)((reg >> 22) & 1);
        reg = (reg << 1) | (uint32_t)b;
        ref[k] = b ? -1 : 1;
    }
}

/*
 * Locate TRN1u's first symbol in ja_buf by correlation against the known
 * reference (see TRN1U_ALIGN_*).  Moves trn1u_start, records the score and
 * the line polarity, and replays the descrambler from the new start so the
 * TRN1u counters describe TRN1u and not uR.  The aligned start is also
 * 9.5.1.1.10's modulo-12 frame origin for the second TRN1u.
 */
static void trn1u_align(v92_p3_rx_t *rx)
{
    int8_t ref[TRN1U_ALIGN_LEN];
    int nominal = rx->trn1u_nominal_start;
    int best_d = 0;
    int best_c = 0;
    int start;

    trn1u_reference(ref, TRN1U_ALIGN_LEN);
    for (int d = -TRN1U_ALIGN_SPAN; d <= TRN1U_ALIGN_SPAN; d++) {
        int off = nominal + d - rx->ja_buf_base;
        int c = 0;

        if (off < 0 || off + TRN1U_ALIGN_LEN > rx->ja_buf_fill)
            continue;
        for (int k = 0; k < TRN1U_ALIGN_LEN; k++)
            c += (sign_bit(rx->ja_buf[off + k]) ? 1 : -1)*ref[k];
        if (abs(c) > abs(best_c)) {
            best_c = c;
            best_d = d;
        }
    }
    rx->trn1u_align_done = true;
    rx->trn1u_align_score_x1000 = abs(best_c)*1000/TRN1U_ALIGN_LEN;
    if (rx->trn1u_align_score_x1000 < TRN1U_ALIGN_MIN_X1000) {
        rx->trn1u_align_offset = 0;
        rx->trn1u_inverted = false;
    } else {
        rx->trn1u_align_offset = best_d;
        rx->trn1u_inverted = best_c < 0;
    }
    rx->trn1u_start = nominal + rx->trn1u_align_offset + rx->test_start_offset;
    {
        int off = rx->trn1u_start - rx->ja_buf_base;
        int c = 0;

        if (off >= 0 && off + TRN1U_ALIGN_LEN <= rx->ja_buf_fill)
            for (int k = 0; k < TRN1U_ALIGN_LEN; k++)
                c += (sign_bit(rx->ja_buf[off + k]) ? 1 : -1)*ref[k];
        rx->trn1u_start_score_x1000 = abs(c)*1000/TRN1U_ALIGN_LEN;
    }

    /* Replay from the aligned start.  At the true start the zero register is
     * the transmitter's own initial state, so a clean TRN1u descrambles to
     * ones from its first symbol. */
    rx->gpa_reg = 0;
    rx->trn1u_count = 0;
    rx->trn1u_ones = 0;
    start = rx->trn1u_start - rx->ja_buf_base;
    for (int i = start; i < rx->ja_buf_fill; i++)
        (void) trn1u_process(rx, rx->ja_buf[i]);

    /* The equaliser(s), fed everything buffered so far.  Their sample index
     * is the buffer's, as it stands now. */
    rx->eq_base_sample = rx->ja_buf_base;
    for (int law = 0; law < 2; law++) {
        rx->eq_running[law] = false;
        if (rx->no_equaliser || (rx->law >= 0 && law != rx->law))
            continue;
        {
            v92_p3_eq_config_t cfg;

            v92_p3_eq_default_config(&cfg);
            if (rx->eq_ntaps > 0)
                cfg.ntaps = rx->eq_ntaps;
            if (!v92_p3_eq_init(&rx->eq[law], &cfg))
                continue;
        }
        for (int i = 0; i < rx->ja_buf_fill; i++)
            v92_p3_eq_push(&rx->eq[law], law ? alaw_to_linear(rx->ja_buf[i])
                                             : ulaw_to_linear(rx->ja_buf[i]));
        rx->dec_fill[law] = 0;
        rx->dec_base[law] = 0;
        if (v92_p3_eq_start(&rx->eq[law], start))
            rx->eq_running[law] = true;
    }

    if (p3rx_debug_enabled()) {
        fprintf(stderr,
                "[P3RX] trn1u aligned start=%d nominal=%d offset=%+d score=%d.%03d used=%d.%03d%s\n",
                rx->trn1u_start, nominal, rx->trn1u_align_offset,
                rx->trn1u_align_score_x1000/1000,
                rx->trn1u_align_score_x1000%1000,
                rx->trn1u_start_score_x1000/1000,
                rx->trn1u_start_score_x1000%1000,
                rx->trn1u_inverted ? " inverted" : "");
    }
}

/* Feed one codeword to the running equaliser(s) and step them. */
static void dec_push(v92_p3_rx_t *rx, int law, int bit)
{
    if (rx->dec_fill[law] == V92_P3_RX_JA_BUF) {
        memmove(rx->dec_buf[law], rx->dec_buf[law] + 144, V92_P3_RX_JA_BUF - 144);
        rx->dec_fill[law] -= 144;
        rx->dec_base[law] += 144;
    }
    rx->dec_buf[law][rx->dec_fill[law]++] = (uint8_t)bit;
}

/* Feed one codeword to the running equaliser(s), step them, and keep their
 * sign decisions -- data-aided through the guaranteed 2040T of TRN1u and
 * decision-directed after it, there being no reference for Ja. */
static void trn1u_eq_feed(v92_p3_rx_t *rx, uint8_t cw)
{
    for (int law = 0; law < 2; law++) {
        if (!rx->eq_running[law])
            continue;
        v92_p3_eq_push(&rx->eq[law], law ? alaw_to_linear(cw) : ulaw_to_linear(cw));
        while (v92_p3_eq_step(&rx->eq[law]))
            dec_push(rx, law, rx->eq[law].decision > 0);
    }
}

/* Plan step 5's gate.  Returns 0 while undecided, 1 passed, -1 rejected
 * (with the reject recorded through *reason, *m0). */
static int trn1u_gate(v92_p3_rx_t *rx, v92_p3_rx_reject_t *reason, int *m0)
{
    int best = -1;

    if (rx->eq_gate_done)
        return 1;
    if (rx->trn1u_start_score_x1000 < TRN1U_START_SCORE_MIN_X1000) {
        *reason = V92_P3_RX_REJECT_TRN1U_START;
        *m0 = rx->trn1u_start_score_x1000;
        return -1;
    }
    if (rx->no_equaliser) {
        /* The receiver before plan steps 4-6, for A/B: TRN1u judged by
         * raw-sign descrambled ones over its first 256 symbols. */
        int ones_pct = (rx->trn1u_ones_early*100 + TRN1U_EARLY_CHECK_T/2)
                     / TRN1U_EARLY_CHECK_T;

        if (rx->trn1u_count < TRN1U_EARLY_CHECK_T)
            return 0;
        if (ones_pct < TRN1U_NO_EQ_ONES_MIN_PCT) {
            *reason = V92_P3_RX_REJECT_TRN1U_ONES_LOW;
            *m0 = ones_pct;
            return -1;
        }
        rx->eq_gate_done = true;
        return 1;
    }
    for (int law = 0; law < 2; law++) {
        if (!rx->eq_running[law])
            continue;
        if (rx->eq[law].k < rx->eq[law].cfg.seed_symbols + V92_P3_EQ_AGREE_WINDOW)
            return 0;
        /* Unknown law: the better fit is the right expansion. */
        if (best < 0 || v92_p3_eq_snr_db(&rx->eq[law]) > v92_p3_eq_snr_db(&rx->eq[best]))
            best = law;
    }
    if (best < 0) {
        *reason = V92_P3_RX_REJECT_TRN1U_UNTRAINED;
        *m0 = 0;
        return -1;
    }
    rx->eq_agree_x10 = v92_p3_eq_agree_x10(&rx->eq[best]);
    if (p3rx_debug_enabled()) {
        fprintf(stderr,
                "[P3RX] trn1u gate law=%s agree=%d.%d%% snr=%.1fdB main_tap=%d "
                "start_score=%d ones256=%d%%\n",
                best ? "alaw" : "ulaw", rx->eq_agree_x10/10, rx->eq_agree_x10%10,
                v92_p3_eq_snr_db(&rx->eq[best]), v92_p3_eq_main_tap(&rx->eq[best]),
                rx->trn1u_start_score_x1000,
                (rx->trn1u_ones_early*100 + TRN1U_EARLY_CHECK_T/2)/TRN1U_EARLY_CHECK_T);
    }
    if (rx->eq_agree_x10 < TRN1U_AGREE_MIN_X10) {
        *reason = V92_P3_RX_REJECT_TRN1U_UNTRAINED;
        *m0 = rx->eq_agree_x10;
        return -1;
    }
    rx->eq_running[!best] = false;
    rx->eq_law = best;
    rx->eq_gate_done = true;
    return 1;
}






/* -------------------------------------------------------------------------
 * Ja codeword buffer
 * ------------------------------------------------------------------------- */
static void ja_buf_push(v92_p3_rx_t *rx, uint8_t cw, int sample_index)
{
    /* V.92 9.5.2.1.2 permits more than minimum-length TRN1u.
     * Retain 6000 symbols: enough for the largest Table 20 descriptor
     * (N=255, Lsp=Ltp=128), its GPA history and a search cadence. */
    if (rx->ja_buf_fill == V92_P3_RX_JA_BUF) {
        memmove(rx->ja_buf, rx->ja_buf + 144, V92_P3_RX_JA_BUF - 144);
        rx->ja_buf_fill -= 144;
        rx->ja_buf_base += 144;
    }
    if (rx->ja_buf_fill == 0)
        rx->ja_buf_base = sample_index;
    if (rx->ja_buf_fill < V92_P3_RX_JA_BUF)
        rx->ja_buf[rx->ja_buf_fill++] = cw;
}

static void prehist_push(v92_p3_rx_t *rx, uint8_t cw, int sample_index)
{
    int pos;

    if (!rx)
        return;
    pos = rx->prehist_head;
    rx->prehist_cw[pos] = cw;
    rx->prehist_sample[pos] = sample_index;
    rx->prehist_head = (pos + 1) % V92_P3_RX_PRE_HIST;
    if (rx->prehist_fill < V92_P3_RX_PRE_HIST)
        rx->prehist_fill++;
}

static int prehist_copy_tail(const v92_p3_rx_t *rx,
                             int max_count,
                             uint8_t *dst,
                             int *first_sample_out)
{
    int count;
    int start;

    if (!rx || !dst || max_count <= 0)
        return 0;
    count = rx->prehist_fill;
    if (count > max_count)
        count = max_count;
    if (count <= 0)
        return 0;

    start = rx->prehist_head - count;
    while (start < 0)
        start += V92_P3_RX_PRE_HIST;

    for (int i = 0; i < count; i++) {
        int pos = (start + i) % V92_P3_RX_PRE_HIST;
        dst[i] = rx->prehist_cw[pos];
    }
    if (first_sample_out) {
        int pos = start % V92_P3_RX_PRE_HIST;
        *first_sample_out = rx->prehist_sample[pos];
    }
    return count;
}

/*
 * Enter TRN1u accumulation, seeding the Ja buffer with codeword prehistory so
 * the GPA descrambler has sync.  Reached from uR2 in the MD-bearing flow, and
 * directly from uR1 when INFO1a signalled MD = 0 (V.92 9.5.1.1.1: "If the
 * duration of signal MD indicated by INFO1a is zero, the digital modem shall
 * proceed according to 9.5.1.1.2", i.e. straight to training on TRN1u after
 * the first Ru-to-Ru-bar transition -- there is no second Ru/uR pair to wait
 * for).  Callers set ur1_end/ur2_end as appropriate before calling.
 */
static void enter_trn1u(v92_p3_rx_t *rx, uint8_t codeword, int sample_index)
{
    int copied;
    int base_sample = sample_index;
    bool soft_path = rx->p6_soft_mode;

    rx->trn1u_start = sample_index;
    rx->trn1u_nominal_start = sample_index;
    rx->trn1u_align_done = false;
    rx->trn1u_align_offset = 0;
    rx->trn1u_align_score_x1000 = 0;
    rx->trn1u_inverted = false;
    rx->trn1u_ones_early = 0;
    rx->trn1u_start_score_x1000 = 0;
    rx->eq_running[0] = rx->eq_running[1] = false;
    rx->eq_gate_done = false;
    rx->eq_law = -1;
    rx->eq_agree_x10 = 0;
    rx->dec_fill[0] = rx->dec_fill[1] = 0;
    rx->dec_base[0] = rx->dec_base[1] = 0;
    rx->ja_from_eq = false;
    rx->follow_started = false;
    rx->trn1u2_state = 0;
    rx->trn1u2_start = -1;
    /* Enough history for the alignment to look TRN1U_ALIGN_SPAN symbols back,
     * plus the 24 the Ja search's differential/GPA decode reaches behind. */
    copied = prehist_copy_tail(rx, TRN1U_ALIGN_SPAN + 24, rx->ja_buf,
                               &base_sample);
    rx->ja_buf_base = (copied > 0) ? base_sample : rx->trn1u_start;
    rx->ja_buf_fill = copied;
    rx->ja_buf_lead = copied > 23 ? copied - 23 : 0;
    rx->gpa_reg      = 0;
    rx->diff_valid   = false;
    rx->trn1u_count  = 0;
    rx->trn1u_ones   = 0;
    p6_reset(rx);
    rx->p6_soft_mode = soft_path;
    rx->state = V92_P3_RX_TRN1U;
    /* Consume the transition sample as first TRN1u symbol. */
    if (sample_index >= rx->ja_buf_base)
        ja_buf_push(rx, codeword, sample_index);
    (void) trn1u_process(rx, codeword);
}

/* Leave uR1: wait out MD, or skip straight to TRN1u when INFO1a said MD = 0. */
static void ur1_advance(v92_p3_rx_t *rx, uint8_t codeword, int sample_index)
{
    if (rx->md_symbols == 0) {
        if (p3rx_debug_enabled()) {
            fprintf(stderr,
                    "[P3RX] sample=%d ur1->trn1u (MD=0, no second Ru/uR pair)\n",
                    sample_index);
        }
        enter_trn1u(rx, codeword, sample_index);
        return;
    }
    rx->state = V92_P3_RX_MD_WAIT;
}

/* -------------------------------------------------------------------------
 * Ja search — called once the buffer has enough data
 * ------------------------------------------------------------------------- */
/*
 * Search a stream of signs (1 = positive, one per symbol) for a CRC-valid
 * Table 20 descriptor.  6.3 / 8.5.4: Ja is differentially encoded and then
 * GPA-scrambled, so each plain bit is the XOR of six signs; differential
 * decoding makes a constant polarity inversion harmless.  Only exact Table
 * 20 frames count -- no soft candidates.  Returns the index of the frame's
 * first bit in signs[], or -1.
 */
static int ja_search_signs(v92_p3_rx_t *rx, const uint8_t *signs, int fill,
                           int search_start)
{
    uint8_t plain[V92_P3_RX_JA_BUF];
    int search_end = fill - 207;

    if (search_start < 24)
        search_start = 24;
    if (search_end <= search_start)
        return -1;
    for (int i = 24; i < fill; i++)
        plain[i] = (signs[i] ^ signs[i-1] ^ signs[i-5] ^ signs[i-6]
                    ^ signs[i-23] ^ signs[i-24]) & 1;
    for (int start = search_start; start <= search_end; start++) {
        int sync = 0;
        uint8_t packed[(V92_P3_RX_JA_BUF+7)/8] = {0};
        int count = fill - start;
        v92_ja_parse_meta_t meta;
        v90_dil_desc_t desc;

        while (sync < 17 && plain[start+sync]) sync++;
        if (sync != 17 || plain[start+17]) continue;
        for (int i = 0; i < count; i++)
            packed[i/8] |= plain[start+i] << (i%8);
        if (!v92_parse_ja_descriptor_strict(&desc, packed, count, &meta)
            || !meta.is_v92) continue;
        memset(&rx->ja_result, 0, sizeof(rx->ja_result));
        rx->ja_result.ok = true;
        rx->ja_result.parsed_v92 = true;
        rx->ja_result.calling_party = true;
        rx->ja_result.descriptor_bits = meta.bit_len;
        rx->ja_result.desc = desc;
        v90_analyse_dil_descriptor(&desc, &rx->ja_result.analysis);
        return start;
    }
    return -1;
}

static bool run_ja_search(v92_p3_rx_t *rx, bool force_hard_min)
{
    int trn_min_t;
    int found;
    uint8_t signs[V92_P3_RX_JA_BUF];
    int law = rx->eq_law;

    trn_min_t = (force_hard_min
                 ? V92_P3_RX_TRN1U_MIN_T
                 : (rx->p6_soft_mode ? TRN1U_MIN_SOFT_T : V92_P3_RX_TRN1U_MIN_T));

    /* 9.5.1.1.3 arms after 2040T; that is not a Ja onset deadline.
     * Search the retained stream, including descriptors incomplete at the
     * previous probe. The call owner must enforce 9.5.1.2.1's retrain timer.
     *
     * Raw codeword signs first: on a byte-exact DS0 they are the symbols,
     * and they are complete up to the newest codeword, where the equaliser
     * lags by its half length plus the interpolator's -- searching them
     * first keeps Ja's decode, and the instant Sd starts, exactly as they
     * were before the equaliser existed. */
    for (int i = 0; i < rx->ja_buf_fill; i++)
        signs[i] = (rx->ja_buf[i] >> 7) & 1;
    found = ja_search_signs(rx, signs, rx->ja_buf_fill,
                            rx->trn1u_start - rx->ja_buf_base + trn_min_t - 50);
    if (found >= 0) {
        rx->ja_from_eq = false;
    rx->follow_started = false;
    rx->trn1u2_state = 0;
    rx->trn1u2_start = -1;
        rx->ja_result.start_sample = rx->ja_buf_base + found;
        return true;
    }

    /* Then the equalised decisions (plan step 6): what a real loop needs.
     * Symbol k is DS0 sample trn1u_start + k plus the timing loop's offset. */
    if (law >= 0 && rx->eq_gate_done) {
        found = ja_search_signs(rx, rx->dec_buf[law], rx->dec_fill[law],
                                trn_min_t - 50 - rx->dec_base[law]);
        if (p3rx_debug_enabled())
            fprintf(stderr, "[P3RX] eq Ja search base=%d fill=%d from=%d found=%d\n",
                    rx->dec_base[law], rx->dec_fill[law],
                    trn_min_t - 50 - rx->dec_base[law], found);
        if (found >= 0) {
            rx->ja_from_eq = true;
            rx->ja_result.start_sample = rx->trn1u_start + rx->dec_base[law]
                                       + found + (int)lround(rx->eq[law].tau);
            return true;
        }
    }

    p3rx_set_reject(rx,
                    V92_P3_RX_REJECT_JA_SEARCH_FAIL,
                    rx->trn1u_start + rx->trn1u_count,
                    rx->ja_buf_fill,
                    law >= 0 ? rx->dec_fill[law] : 0);
    return false;
}

static void p6_rehunt_from_current(v92_p3_rx_t *rx,
                                   uint8_t cw,
                                   int sample_index,
                                   v92_p3_rx_reject_t reason,
                                   int metric0,
                                   int metric1)
{
    p3rx_set_reject(rx, reason, sample_index, metric0, metric1);
    p6_reset(rx);
    p6_update_hyp_runs(rx, cw, sample_index);
    rx->state = V92_P3_RX_RU1_HUNT;
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void v92_p3_rx_init(v92_p3_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->state       = V92_P3_RX_IDLE;
    rx->ru1_start   = -1;  rx->ru1_end   = -1;
    rx->ur1_start   = -1;  rx->ur1_end   = -1;
    rx->ru2_start   = -1;  rx->ru2_end   = -1;
    rx->ur2_start   = -1;  rx->ur2_end   = -1;
    rx->trn1u_start = -1;
    rx->arm_sample_min = 0;
    rx->law = -1;
    rx->eq_law = -1;
    rx->last_reject = V92_P3_RX_REJECT_NONE;
    rx->last_reject_sample = -1;
    rx->last_reject_metric0 = 0;
    rx->last_reject_metric1 = 0;
    p6_reset(rx);
}

void v92_p3_rx_set_equaliser(v92_p3_rx_t *rx, bool on)
{
    if (rx)
        rx->no_equaliser = !on;
}

void v92_p3_rx_set_equaliser_taps(v92_p3_rx_t *rx, int ntaps)
{
    if (rx)
        rx->eq_ntaps = (ntaps > 0 && ntaps <= V92_P3_EQ_MAX_TAPS && (ntaps & 1))
                     ? ntaps : 0;
}

void v92_p3_rx_set_law(v92_p3_rx_t *rx, int law)
{
    if (rx)
        rx->law = (law == 0 || law == 1) ? law : -1;
}

void v92_p3_rx_set_md_length(v92_p3_rx_t *rx, int md_symbols)
{
    if (rx)
        rx->md_symbols = (md_symbols > 0) ? md_symbols : 0;
}

void v92_p3_rx_start(v92_p3_rx_t *rx, int first_sample_index)
{
    v92_p3_rx_init(rx);
    rx->arm_sample_min = (first_sample_index >= 0) ? first_sample_index : 0;
    rx->state = V92_P3_RX_RU1_HUNT;
}

bool v92_p3_rx_feed(v92_p3_rx_t *rx, uint8_t codeword, int sample_index)
{
    v92_p3_rx_state_t prev = rx->state;

    rx->fed_until = sample_index;
    if (rx->state != V92_P3_RX_IDLE && sample_index < rx->arm_sample_min) {
        if (rx->last_reject != V92_P3_RX_REJECT_PRE_ARM) {
            p3rx_set_reject(rx,
                            V92_P3_RX_REJECT_PRE_ARM,
                            sample_index,
                            rx->arm_sample_min,
                            0);
        }
        prehist_push(rx, codeword, sample_index);
        return false;
    }

    if (rx->state == V92_P3_RX_RU1_HUNT
        || rx->state == V92_P3_RX_RU1
        || rx->state == V92_P3_RX_UR1
        || rx->state == V92_P3_RX_MD_WAIT
        || rx->state == V92_P3_RX_RU2
        || rx->state == V92_P3_RX_UR2) {
        p6_update_hyp_runs(rx, codeword, sample_index);
    }

    switch (rx->state) {

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_IDLE:
        break;

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU1_HUNT: {
        int h_ru = p6_best_hyp_run(rx, true, P6_LOCK_MIN, true);
        int h_ur = p6_best_hyp_run(rx, false, P6_LOCK_MIN, true);
        int best_h = -1;
        int best_any_h = -1;
        int best_any_run = 0;
        bool soft_fallback = false;

        if (h_ru < 0)
            h_ru = p6_best_hyp_soft_run(rx, true, P6_LOCK_MIN_SOFT);
        if (h_ur < 0)
            h_ur = p6_best_hyp_soft_run(rx, false, P6_LOCK_MIN_SOFT);
        for (int h = 0; h < 12; h++) {
            if (rx->p6_hyp_soft_run[h] > best_any_run) {
                best_any_run = rx->p6_hyp_soft_run[h];
                best_any_h = h;
            }
        }
        if (best_any_h >= 0 && best_any_run > rx->hunt_best_run) {
            double mean = 0.0;
            double stdv = 0.0;
            int range = 0;
            bool lu_ok = false;
            (void) p6_hyp_lu_stats(rx, best_any_h, &mean, &range, &stdv, &lu_ok);
            rx->hunt_best_run = best_any_run;
            rx->hunt_best_hyp = best_any_h;
            rx->hunt_best_start = sample_index - best_any_run + 1;
            rx->hunt_best_lu_ok = lu_ok ? 1 : 0;
            rx->hunt_best_mean_x10 = (int) lround(mean * 10.0);
            rx->hunt_best_range = range;
            rx->hunt_best_std_x10 = (int) lround(stdv * 10.0);
        }

        if (h_ru >= 0)
            best_h = h_ru;
        if (h_ur >= 0
            && (best_h < 0 || rx->p6_hyp_soft_run[h_ur] > rx->p6_hyp_soft_run[best_h])) {
            best_h = h_ur;
        }

        if (best_h >= 0) {
            int run_soft = rx->p6_hyp_soft_run[best_h];
            int run_hard = rx->p6_hyp_run[best_h];

            if (run_soft < P6_LOCK_MIN_SOFT)
                break;
            soft_fallback = (run_soft < P6_LOCK_MIN);
            rx->ru_hyp = best_h;
            rx->ur_hyp = p6_hyp_index(!p6_hyp_pol(best_h), p6_hyp_phase0(best_h));
            rx->p6_ru_polarity = p6_hyp_pol(best_h);
            rx->p6_run = run_soft;
            rx->p6_locked = true;
            rx->p6_soft_mode = soft_fallback || (run_hard < (P6_LOCK_MIN / 4));
            rx->ru1_start = sample_index - rx->p6_run + 1;
            if (soft_fallback && p3rx_debug_enabled()) {
                fprintf(stderr,
                        "[P3RX] sample=%d soft-lock hyp=%d run_soft=%d run_hard=%d soft_mode=%d\n",
                        sample_index, best_h, run_soft, run_hard, rx->p6_soft_mode ? 1 : 0);
            }
            rx->state = V92_P3_RX_RU1;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU1: {
        int run = (rx->ru_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ru_hyp] : 0;

        if (run > 0) {
            rx->p6_run = run;
            if (run > RU_MAX_T)
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_RU_MISMATCH,
                                       run, RU_MAX_T);
        } else {
            int run_effective = rx->p6_run;
            int ur_run = (rx->ur_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ur_hyp] : 0;
            bool ru1_len_ok_strict = (run_effective >= RU_ACCEPT_MIN
                                      && run_effective <= RU_ACCEPT_MAX);
            bool ru1_len_ok_soft = (run_effective >= P6_LOCK_MIN_SOFT
                                    && run_effective <= RU_ACCEPT_MAX);

            if (ru1_len_ok_soft
                && rx->ur_hyp >= 0
                && ur_run > 0) {
                if (!ru1_len_ok_strict && p3rx_debug_enabled()) {
                    fprintf(stderr,
                            "[P3RX] sample=%d ru1 truncated soft-accept run=%d ur_run=%d\n",
                            sample_index, run_effective, ur_run);
                }
                rx->ru1_end = sample_index - 1;
                rx->ur1_start = sample_index;
                rx->p6_run = ur_run;
                rx->state = V92_P3_RX_UR1;
            } else if (run_effective >= RU_FOLDED_MIN_T
                       && run_effective <= RU_FOLDED_MAX_T
                       && rx->ru1_start >= 0
                       && rx->ur_hyp >= 0
                       && ur_run > 0) {
                int inferred_ur2_start = rx->ru1_start + RU_FOLDED_EXPECT_T;
                int delta = sample_index - inferred_ur2_start;

                /*
                 * Some captures merge Ru1/uR1/Ru2 into one apparent 6-symbol
                 * run under a strong hypothesis. Keep strict lock as primary,
                 * but accept this bounded fallback and continue at UR2.
                 */
                if (abs(delta) <= RU_FOLDED_UR2_ANCHOR_TOL) {
                    rx->ru1_end = rx->ru1_start + V92_P3_RX_RU_T - 1;
                    rx->ur1_start = rx->ru1_end + 1;
                    rx->ur1_end = rx->ur1_start + V92_P3_RX_UR_T - 1;
                    rx->ru2_start = rx->ur1_end + 1;
                    rx->ru2_end = rx->ru2_start + V92_P3_RX_RU_T - 1;
                    rx->ur2_start = sample_index;
                    rx->p6_run = ur_run;
                    rx->state = V92_P3_RX_UR2;
                } else {
                    p6_rehunt_from_current(rx, codeword, sample_index,
                                           V92_P3_RX_REJECT_RU_MISMATCH,
                                           run_effective, delta);
                }
            } else {
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_RU_MISMATCH,
                                       run_effective, ur_run);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_UR1: {
        int run = (rx->ur_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ur_hyp] : 0;

        if (run > 0) {
            rx->p6_run = run;
            if (run > UR_MAX_T)
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_UR_MISMATCH,
                                       run, UR_MAX_T);
        } else {
            int run_effective = rx->p6_run;
            int elapsed = sample_index - rx->ur1_start;
            int ur_min = rx->p6_soft_mode ? UR_ACCEPT_MIN_SOFT : UR_ACCEPT_MIN;
            int ur_max = rx->p6_soft_mode ? UR_ACCEPT_MAX_SOFT : UR_ACCEPT_MAX;
            if (p3rx_debug_enabled()) {
                fprintf(stderr,
                        "[P3RX] sample=%d ur1 eval run=%d elapsed=%d soft_mode=%d ur_min=%d ur_max=%d\n",
                        sample_index,
                        run_effective,
                        elapsed,
                        rx->p6_soft_mode ? 1 : 0,
                        ur_min,
                        ur_max);
            }
            if (elapsed < UR_RELOCK_GRACE)
                break;
            if (run_effective >= ur_min && run_effective <= ur_max) {
                rx->ur1_end = sample_index - 1;
                rx->p6_run = 0;
                ur1_advance(rx, codeword, sample_index);
            } else if (rx->p6_soft_mode
                       && run_effective > 0
                       && elapsed <= UR_ACCEPT_MAX_SOFT) {
                if (p3rx_debug_enabled()) {
                    fprintf(stderr,
                            "[P3RX] sample=%d ur1 inferred soft-accept run=%d elapsed=%d\n",
                            sample_index, run_effective, elapsed);
                }
                rx->ur1_end = sample_index - 1;
                rx->p6_run = 0;
                ur1_advance(rx, codeword, sample_index);
            } else {
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_UR_MISMATCH,
                                       run_effective, elapsed);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_MD_WAIT: {
        int elapsed = sample_index - rx->ur1_end;
        /* V.92 9.5.1.1.1: wait for the duration signalled in INFO1a
         * before acquiring the second Ru. MD can resemble a periodic
         * training signal; it is not evidence of an early Ru. Table 18
         * allows MD longer than the old fixed one-second timeout. */
        if (elapsed <= rx->md_symbols)
            break;
        if (elapsed - rx->md_symbols > V92_P3_RX_MD_MAX_T) {
            p6_rehunt_from_current(rx, codeword, sample_index,
                                   V92_P3_RX_REJECT_MD_TIMEOUT,
                                   elapsed - rx->md_symbols, V92_P3_RX_MD_MAX_T);
            break;
        }

        if (rx->ru_hyp < 0) {
            p6_rehunt_from_current(rx, codeword, sample_index,
                                   V92_P3_RX_REJECT_RU_MISMATCH,
                                   -1, elapsed);
            break;
        }

        {
            int run = rx->p6_hyp_soft_run[rx->ru_hyp];
            int min_run = rx->p6_soft_mode ? RU2_LOCK_MIN_SOFT : P6_LOCK_MIN_SOFT;
            /* Do not backdate Ru2 into MD using a run accumulated there. */
            if (run > elapsed - rx->md_symbols)
                run = elapsed - rx->md_symbols;
            if (run >= min_run) {
                rx->p6_run = run;
                rx->ru2_start = sample_index - run + 1;
                rx->state = V92_P3_RX_RU2;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU2: {
        int run = (rx->ru_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ru_hyp] : 0;
        if (run > sample_index - rx->ru2_start + 1)
            run = sample_index - rx->ru2_start + 1;

        if (run > 0) {
            rx->p6_run = run;
            if (run > RU_MAX_T)
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_RU_MISMATCH,
                                       run, RU_MAX_T);
        } else {
            int run_effective = rx->p6_run;
            int ru_min = rx->p6_soft_mode ? RU_ACCEPT_MIN_SOFT : RU_ACCEPT_MIN;

            if (run_effective >= ru_min
                && run_effective <= RU_ACCEPT_MAX
                && rx->ur_hyp >= 0
                && rx->p6_hyp_soft_run[rx->ur_hyp] > 0) {
                rx->ru2_end = sample_index - 1;
                rx->ur2_start = sample_index;
                rx->p6_run = rx->p6_hyp_soft_run[rx->ur_hyp];
                rx->state = V92_P3_RX_UR2;
            } else {
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_RU_MISMATCH,
                                       run_effective,
                                       (rx->ur_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ur_hyp] : -1);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_UR2: {
        int run = (rx->ur_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ur_hyp] : 0;

        if (run > 0) {
            rx->p6_run = run;
            if (run > UR_MAX_T)
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_UR_MISMATCH,
                                       run, UR_MAX_T);
        } else {
            int run_effective = rx->p6_run;
            int elapsed = sample_index - rx->ur2_start;
            int ur_min = rx->p6_soft_mode ? UR_ACCEPT_MIN_SOFT : UR_ACCEPT_MIN;
            int ur_max = rx->p6_soft_mode ? UR_ACCEPT_MAX_SOFT : UR_ACCEPT_MAX;
            if (elapsed < UR_RELOCK_GRACE)
                break;
            if (run_effective >= ur_min && run_effective <= ur_max) {
                rx->ur2_end = sample_index - 1;
                enter_trn1u(rx, codeword, sample_index);
            } else {
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_UR_MISMATCH,
                                       run_effective, elapsed);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_TRN1U:
    {
        int trn_min_t = rx->p6_soft_mode ? TRN1U_MIN_SOFT_T : V92_P3_RX_TRN1U_MIN_T;
        int ja_lead_t = rx->p6_soft_mode ? JA_LEAD_SOFT_T : V92_P3_RX_JA_LEAD_T;
        /* Buffer every codeword from trn1u_start − 23 onwards. */
        if (sample_index >= rx->ja_buf_base)
            ja_buf_push(rx, codeword, sample_index);

        trn1u_process(rx, codeword);

        /* Find the true start once the whole correlation window has
         * arrived, before anything judges TRN1u. */
        if (!rx->trn1u_align_done
            && sample_index >= rx->trn1u_nominal_start + TRN1U_ALIGN_SPAN
                               + TRN1U_ALIGN_LEN - 1)
            trn1u_align(rx);
        else if (rx->trn1u_align_done)
            trn1u_eq_feed(rx, codeword);

        /*
         * The TRN1u gate (plan step 5): the start by its correlation score,
         * training by equalised sign agreement with the 8.5.7 reference.
         *
         * It replaces a check on GPA-descrambled ones, which a self-
         * synchronising descrambler makes roughly three times worse than
         * the sign error itself: the r4 loop read 48% there against a 75%
         * gate while the equaliser recovers 99.9% of its signs.  The ones
         * figure is still computed and goes out as m1 of a reject.
         *
         * There used to be an "equalised" fallback here as well: p3_demod,
         * a V.34 passband demodulator, run over 24 hypotheses on baseband
         * PCM.  It never ran -- one codeword short of history every call --
         * and woken up it "passed" the fixture's false lock at 73%.
         */
        if (rx->trn1u_align_done && !rx->eq_gate_done) {
            v92_p3_rx_reject_t reason = V92_P3_RX_REJECT_NONE;
            int m0 = 0;

            if (trn1u_gate(rx, &reason, &m0) < 0) {
                int ones_pct = (rx->trn1u_ones_early*100 + TRN1U_EARLY_CHECK_T/2)
                             / TRN1U_EARLY_CHECK_T;

                p6_rehunt_from_current(rx, codeword, sample_index,
                                       reason, m0, ones_pct);
                break;
            }
        }
        /* Ja may not be searched for until the gate has passed. */
        if (!rx->eq_gate_done)
            goto trn1u_wait;

        if (rx->ja_buf_fill >= V92_P3_RX_JA_BUF) {
            p3rx_set_reject(rx,
                            V92_P3_RX_REJECT_JA_BUFFER_FULL,
                            sample_index,
                            rx->ja_buf_fill,
                            V92_P3_RX_JA_BUF);
            rx->state = V92_P3_RX_FAILED;
            break;
        }

        /* Once ≥ TRN1U_MIN + JA_LEAD_T codewords buffered after
         * trn1u_start, we should have enough for the Ja search. */
        if (rx->trn1u_count >= trn_min_t + ja_lead_t)
            rx->state = V92_P3_RX_JA_SEARCH;
trn1u_wait:
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_JA_SEARCH:
    {
        int ready_hard = 24 + V92_P3_RX_TRN1U_MIN_T + V92_P3_RX_JA_LEAD_T + 207;
        int fill;

        ja_buf_push(rx, codeword, sample_index);
        trn1u_eq_feed(rx, codeword);
        /* Count from the 23-symbol seed the buffer used to start with, so the
         * deeper history the TRN1u alignment needs does not move the search
         * cadence -- and with it the instant Ja is declared and Sd starts. */
        fill = rx->ja_buf_fill - rx->ja_buf_lead;

        /* V.92 9.5.1.1.3 requires the received Table-20 descriptor.
         * A repaired CRC or a soft candidate is never a receive event.
         * Keep collecting until a whole descriptor is available, including
         * long descriptors; a single early failed probe is not a timeout. */
        if (fill >= ready_hard && (fill-ready_hard)%144 == 0) {
            if (run_ja_search(rx, true) && rx->ja_result.ok && rx->ja_result.parsed_v92) {
                rx->last_reject = V92_P3_RX_REJECT_NONE;
                rx->ja_found = true;
                rx->state = V92_P3_RX_DONE;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_DONE:
    case V92_P3_RX_FAILED:
        break;
    }

    if (rx->state != prev && p3rx_debug_enabled()) {
        int ru_run = (rx->ru_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ru_hyp] : -1;
        int ur_run = (rx->ur_hyp >= 0) ? rx->p6_hyp_soft_run[rx->ur_hyp] : -1;
        fprintf(stderr,
                "[P3RX] sample=%d %s->%s p6_run=%d ru_run=%d ur_run=%d ru_hyp=%d ur_hyp=%d soft=%d\n",
                sample_index,
                v92_p3_rx_state_name(prev),
                v92_p3_rx_state_name(rx->state),
                rx->p6_run,
                ru_run,
                ur_run,
                rx->ru_hyp,
                rx->ur_hyp,
                rx->p6_soft_mode ? 1 : 0);
    }
    prehist_push(rx, codeword, sample_index);
    return (rx->state != prev);
}

/* -------------------------------------------------------------------------
 * After Ja: the equaliser handed on (docs/v92_p3_rx_line_plan.md step 7)
 * ------------------------------------------------------------------------- */

/* The second TRN1u: located like the first (TRN1U_ALIGN_LEN raw signs
 * against the reference, step 3; start score at least step 5's
 * TRN1U_START_SCORE_MIN_X1000), within TRN1U2_SPAN of the expected start,
 * which comes from the Su detector's six-symbol blocks.  The bar the Su
 * receiver reports lasts 24T (8.5.6, 9.5.2.1.9) and it fires a block or two
 * into it. */
#define TRN1U2_SPAN        64
#define TRN1U2_AFTER_FINAL 18

static void trn1u2_search(v92_p3_rx_t *rx)
{
    v92_p3_eq_t *eq = &rx->eq[rx->eq_law];
    int8_t ref[TRN1U_ALIGN_LEN];
    int best_c = 0;
    int best_s = -1;

    if (rx->trn1u2_state != 1
        || rx->follow_sample < rx->trn1u2_expect + TRN1U2_SPAN + TRN1U_ALIGN_LEN)
        return;
    v92_p3_eq_reference(ref, TRN1U_ALIGN_LEN);
    for (int s0 = rx->trn1u2_expect - TRN1U2_SPAN;
         s0 <= rx->trn1u2_expect + TRN1U2_SPAN; s0++) {
        int c = 0;

        if (rx->follow_sample - s0 >= 1024 - TRN1U_ALIGN_LEN)
            continue;
        for (int k = 0; k < TRN1U_ALIGN_LEN; k++)
            c += (rx->follow_sign[(s0 + k) & 1023] ? 1 : -1)*ref[k];
        if (abs(c) > abs(best_c)) {
            best_c = c;
            best_s = s0;
        }
    }
    rx->trn1u2_score_x1000 = abs(best_c)*1000/TRN1U_ALIGN_LEN;
    rx->trn1u2_start = best_s;
    if (best_s < 0 || rx->trn1u2_score_x1000 < TRN1U_START_SCORE_MIN_X1000) {
        rx->trn1u2_state = -1;
    } else {
        /* Symbol k of the equaliser is DS0 sample start + k + tau.  Data-
         * aided for the guaranteed 2040T, decision-directed after it (TRN1u
         * runs on through the DIL, then CPt keeps its two levels). */
        int64_t k0 = best_s - (rx->eq_base_sample + eq->start)
                   - (int64_t)lround(eq->tau);

        v92_p3_eq_train_from(eq, k0, V92_P3_RX_TRN1U_MIN_T);
        rx->trn1u2_state = 3;
    }
    if (p3rx_debug_enabled()) {
        fprintf(stderr, "[P3RX] trn1u2 expect=%d start=%d score=%d tau=%.3f ppm=%.1f %s\n",
                rx->trn1u2_expect, rx->trn1u2_start, rx->trn1u2_score_x1000,
                eq->tau, eq->freq*1e6,
                rx->trn1u2_state == 3 ? "retraining" : "refused");
    }
}

/* Step 5's agreement gate, on the 256 symbols after retraining began. */
static void trn1u2_gate(v92_p3_rx_t *rx)
{
    v92_p3_eq_t *eq = &rx->eq[rx->eq_law];

    if (rx->trn1u2_state != 3)
        return;
    if (p3rx_debug_enabled() && eq->agree_fill == V92_P3_EQ_AGREE_WINDOW
        && (eq->k - eq->ref_from) % 256 == 0)
        fprintf(stderr, "[P3RX] trn1u2 k=%lld agree=%d tau=%.3f ppm=%.1f\n",
                (long long)(eq->k - eq->ref_from), v92_p3_eq_agree_x10(eq),
                eq->tau, eq->freq*1e6);
    /* Judged at the end of the guaranteed 2040T, over its last 256
     * symbols: the equaliser was held since Ja while the analogue modem's
     * upstream clock locked to the downstream it recovered from Sd (6.2),
     * and the existing raw-path lock comes no earlier either. */
    if (eq->k < eq->ref_until)
        return;
    rx->trn1u2_agree_x10 = v92_p3_eq_agree_x10(eq);
    rx->trn1u2_state = rx->trn1u2_agree_x10 >= TRN1U_AGREE_MIN_X10 ? 2 : -1;
    if (p3rx_debug_enabled())
        fprintf(stderr, "[P3RX] trn1u2 agree=%d.%d%% %s\n",
                rx->trn1u2_agree_x10/10, rx->trn1u2_agree_x10%10,
                rx->trn1u2_state == 2 ? "trained" : "refused");
}

int v92_p3_rx_follow(v92_p3_rx_t *rx, uint8_t codeword, int sample_index,
                     double *out, int max)
{
    int law;
    int produced = 0;
    v92_p3_eq_t *eq;

    if (!rx || !rx->ja_found || rx->eq_law < 0)
        return 0;
    law = rx->eq_law;
    eq = &rx->eq[law];
    /* The engine finds Ja part way through a block and then follows from
     * that block's first codeword, so without this the samples between the
     * two went into the equaliser twice (65 against slmodemd), shifting
     * every later symbol index -- and with it the reference the second
     * TRN1u is retrained against, which then diverged to chance. */
    if (sample_index <= rx->fed_until)
        return 0;
    if (!rx->follow_started) {
        v92_p3_eq_hold(eq, true);
        rx->follow_started = true;
    }
    rx->follow_sample = sample_index;
    rx->follow_sign[sample_index & 1023] = (codeword >> 7) & 1;
    v92_p3_eq_push(eq, law ? alaw_to_linear(codeword) : ulaw_to_linear(codeword));
    while (v92_p3_eq_step(eq)) {
        dec_push(rx, law, eq->decision > 0);
        if (out && produced < max)
            out[produced] = eq->y;
        produced++;
    }
    trn1u2_search(rx);
    trn1u2_gate(rx);
    return produced < max ? produced : max;
}

void v92_p3_rx_expect_trn1u2(v92_p3_rx_t *rx, int sample_index)
{
    if (!rx || rx->eq_law < 0 || rx->trn1u2_state != 0)
        return;
    rx->trn1u2_expect = sample_index + TRN1U2_AFTER_FINAL;
    rx->trn1u2_state = 1;
}

int v92_p3_rx_trn1u2_state(const v92_p3_rx_t *rx)
{
    return rx ? (rx->trn1u2_state == 2 ? 1 : rx->trn1u2_state == -1 ? -1 : 0) : 0;
}

void v92_p3_rx_feed_block(v92_p3_rx_t *rx,
                          const uint8_t *codewords,
                          int            count,
                          int            first_sample_index)
{
    for (int i = 0; i < count; i++)
        v92_p3_rx_feed(rx, codewords[i], first_sample_index + i);
}

v92_p3_rx_state_t v92_p3_rx_get_state(const v92_p3_rx_t *rx)
{
    return rx->state;
}

bool v92_p3_rx_ja_ok(const v92_p3_rx_t *rx)
{
    return rx->state == V92_P3_RX_DONE && rx->ja_found;
}

const ja_dil_decode_t *v92_p3_rx_get_ja(const v92_p3_rx_t *rx)
{
    return v92_p3_rx_ja_ok(rx) ? &rx->ja_result : NULL;
}

const char *v92_p3_rx_state_name(v92_p3_rx_state_t s)
{
    switch (s) {
    case V92_P3_RX_IDLE:       return "idle";
    case V92_P3_RX_RU1_HUNT:   return "ru1_hunt";
    case V92_P3_RX_RU1:        return "ru1";
    case V92_P3_RX_UR1:        return "ur1";
    case V92_P3_RX_MD_WAIT:    return "md_wait";
    case V92_P3_RX_RU2:        return "ru2";
    case V92_P3_RX_UR2:        return "ur2";
    case V92_P3_RX_TRN1U:      return "trn1u";
    case V92_P3_RX_JA_SEARCH:  return "ja_search";
    case V92_P3_RX_DONE:       return "done";
    case V92_P3_RX_FAILED:     return "failed";
    default:                   return "unknown";
    }
}

const char *v92_p3_rx_reject_name(v92_p3_rx_reject_t r)
{
    switch (r) {
    case V92_P3_RX_REJECT_NONE:          return "none";
    case V92_P3_RX_REJECT_PRE_ARM:       return "pre_arm";
    case V92_P3_RX_REJECT_RU_MISMATCH:   return "ru_mismatch";
    case V92_P3_RX_REJECT_UR_MISMATCH:   return "ur_mismatch";
    case V92_P3_RX_REJECT_MD_TIMEOUT:    return "md_timeout";
    case V92_P3_RX_REJECT_TRN1U_ONES_LOW:return "trn1u_ones_low";
    case V92_P3_RX_REJECT_TRN1U_START:   return "trn1u_start";
    case V92_P3_RX_REJECT_TRN1U_UNTRAINED:return "trn1u_untrained";
    case V92_P3_RX_REJECT_JA_BUFFER_FULL:return "ja_buffer_full";
    case V92_P3_RX_REJECT_JA_SEARCH_FAIL:return "ja_search_fail";
    case V92_P3_RX_REJECT_JA_SOFT_ONLY:  return "ja_soft_only";
    default:                             return "unknown";
    }
}

v92_p3_rx_reject_t v92_p3_rx_last_reject(const v92_p3_rx_t *rx,
                                         int *sample_out,
                                         int *metric0_out,
                                         int *metric1_out)
{
    if (sample_out)
        *sample_out = rx ? rx->last_reject_sample : -1;
    if (metric0_out)
        *metric0_out = rx ? rx->last_reject_metric0 : 0;
    if (metric1_out)
        *metric1_out = rx ? rx->last_reject_metric1 : 0;
    return rx ? rx->last_reject : V92_P3_RX_REJECT_NONE;
}
