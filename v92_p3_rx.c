/*
 * v92_p3_rx.c — V.92 Phase 3 upstream receiver (PCM domain)
 *
 * See v92_p3_rx.h for protocol overview and design notes.
 */

#include "v92_p3_rx.h"
#include "p3_demod.h"

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
/* TRN1u sanity: descrambled stream should be heavily biased to ones. */
#define TRN1U_ONES_MIN_PCT 75
#define TRN1U_ONES_MIN_PCT_SOFT 60
#define TRN1U_EARLY_CHECK_T 256
#define TRN1U_MIN_SOFT_T 256
#define JA_LEAD_SOFT_T 24
#define TRN1U_DEMOD_MAX_HYP 12
#define JA_DEMOD_MAX_BITS   4096

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
 * GPA descrambler (x^23 + x^18 + 1, tap = 17)
 * -------------------------------------------------------------------------
 * Self-synchronising: shift register is updated with the raw INPUT bit.
 */
static inline int gpa_descramble(uint32_t *reg, int in_bit)
{
    int out = (in_bit ^ (int)(*reg >> 22) ^ (int)(*reg >> 17)) & 1;
    *reg = (*reg << 1) | (uint32_t)in_bit;
    return out;
}

/* -------------------------------------------------------------------------
 * TRN1u single-sample processor
 *
 * V.92 §8.5.7: all-ones fed through GPA scrambler, differentially encoded,
 * mapped 0→+L_U, 1→−L_U.  Receiver inverts: +L_U(MSB=1)→0, −L_U(MSB=0)→1.
 * Then differential-decode, then GPA descramble → should yield all-ones.
 *
 * Returns the descrambled bit (0 or 1), or -1 on the first call
 * (diff_prev seeding only).
 * ------------------------------------------------------------------------- */
static int trn1u_process(v92_p3_rx_t *rx, uint8_t cw)
{
    /* V.92 sign convention: positive(MSB=1)=0, negative(MSB=0)=1 */
    int v92_bit = 1 - sign_bit(cw);

    if (!rx->diff_valid) {
        rx->diff_prev  = v92_bit;
        rx->diff_valid = true;
        return -1;
    }

    int diff = v92_bit ^ rx->diff_prev;
    rx->diff_prev = v92_bit;

    int out = gpa_descramble(&rx->gpa_reg, diff);
    rx->trn1u_count++;
    if (out) rx->trn1u_ones++;
    return out;
}

static bool trn1u_ones_ok_at_count(const v92_p3_rx_t *rx, int count)
{
    if (!rx || count <= 0 || rx->trn1u_count < count)
        return false;
    return (rx->trn1u_ones * 100 >= count * TRN1U_ONES_MIN_PCT);
}

static inline int gpa_descramble_t17_bit(uint32_t *reg, int in_bit)
{
    int out = (in_bit ^ (int) (*reg >> 22) ^ (int) (*reg >> 17)) & 1;
    *reg = ((*reg << 1) | (uint32_t) (in_bit & 1)) & 0x7FFFFFU;
    return out;
}

static int trn1u_ones_pct_from_result(const p3_result_t *r,
                                      int trn_start_sample,
                                      int payload_symbols)
{
    int start_sym = -1;
    int best_pct = -1;

    if (!r || !r->symbols || r->symbol_count <= 0 || payload_symbols <= 0)
        return -1;

    for (int i = 0; i < r->symbol_count; i++) {
        if (r->symbols[i].sample_index >= trn_start_sample) {
            start_sym = i;
            break;
        }
    }
    if (start_sym < 0)
        return -1;

    {
        int available = r->symbol_count - start_sym;
        int history = 23;
        if (available <= history)
            return -1;
        if (payload_symbols > available - history)
            payload_symbols = available - history;
    }

    /*
     * Evaluate both differential-bit mappings from dibit and both polarity
     * inversions. Pick the strongest one-rate.
     */
    for (int map = 0; map < 2; map++) {
        for (int inv = 0; inv < 2; inv++) {
            uint32_t reg = 0;
            int ones = 0;
            int total = payload_symbols;

            for (int i = 0; i < 23; i++) {
                int d = r->symbols[start_sym + i].dibit & 3;
                int raw = (map == 0) ? ((d >> 1) & 1) : (d & 1);
                raw ^= inv;
                (void) gpa_descramble_t17_bit(&reg, raw);
            }
            for (int i = 0; i < payload_symbols; i++) {
                int d = r->symbols[start_sym + 23 + i].dibit & 3;
                int raw = (map == 0) ? ((d >> 1) & 1) : (d & 1);
                raw ^= inv;
                ones += gpa_descramble_t17_bit(&reg, raw);
            }

            {
                int pct = (ones * 100 + total / 2) / total;
                if (pct > best_pct)
                    best_pct = pct;
            }
        }
    }

    return best_pct;
}

static int trn1u_demod_best_ones_pct(const v92_p3_rx_t *rx, int trn_symbols)
{
    int eval_total;
    int best_pct = -1;
    p3_hypothesis_t hyps[TRN1U_DEMOD_MAX_HYP];

    if (!rx || trn_symbols <= 64)
        return -1;
    if (rx->ja_buf_fill < 24 + trn_symbols)
        return -1;

    eval_total = 23 + trn_symbols;

    for (int law = 0; law < 2; law++) {
        int16_t *lin = (int16_t *) malloc((size_t) eval_total * sizeof(int16_t));
        int count;

        if (!lin)
            return best_pct;

        for (int i = 0; i < eval_total; i++) {
            uint8_t cw = rx->ja_buf[i];
            lin[i] = (int16_t) (law ? alaw_to_linear(cw) : ulaw_to_linear(cw));
        }

        count = p3_scan_all_hypotheses(lin,
                                       eval_total,
                                       rx->ja_buf_base,
                                       8000,
                                       hyps,
                                       TRN1U_DEMOD_MAX_HYP);
        if (p3rx_debug_enabled()) {
            fprintf(stderr,
                    "[P3RX] demod_ja_search law=%s eval_total=%d hyps=%d base=%d trn_start=%d\n",
                    law ? "alaw" : "ulaw",
                    eval_total,
                    count,
                    rx->ja_buf_base,
                    rx->trn1u_start);
        }
        for (int hi = 0; hi < count; hi++) {
            p3_result_t *r = p3_demod_run(lin,
                                          eval_total,
                                          rx->ja_buf_base,
                                          hyps[hi].baud_code,
                                          hyps[hi].carrier_sel,
                                          8000);
            int pct;
            if (!r)
                continue;
            pct = trn1u_ones_pct_from_result(r, rx->trn1u_start, trn_symbols);
            if (pct > best_pct)
                best_pct = pct;
            p3_result_free(r);
        }

        free(lin);
    }

    return best_pct;
}

static int unpacked_ones_pct(const uint8_t *bits, int count)
{
    int ones = 0;
    if (!bits || count <= 0)
        return 0;
    for (int i = 0; i < count; i++)
        ones += bits[i] ? 1 : 0;
    return (ones * 100 + count / 2) / count;
}

static int demod_build_gpa_bits(const p3_result_t *r,
                                int trn_start_sample,
                                int map,
                                int inv,
                                uint8_t *out_bits,
                                int out_cap,
                                int *first_sample_out)
{
    int start_sym = -1;
    int available;
    int out_n;
    uint32_t reg = 0;

    if (!r || !r->symbols || r->symbol_count <= 0 || !out_bits || out_cap <= 0)
        return 0;

    for (int i = 0; i < r->symbol_count; i++) {
        if (r->symbols[i].sample_index >= trn_start_sample) {
            start_sym = i;
            break;
        }
    }
    if (start_sym < 0)
        return 0;

    available = r->symbol_count - start_sym;
    if (available <= 23)
        return 0;

    out_n = available - 23;
    if (out_n > out_cap)
        out_n = out_cap;

    for (int i = 0; i < 23; i++) {
        int d = r->symbols[start_sym + i].dibit & 3;
        int raw = (map == 0) ? ((d >> 1) & 1) : (d & 1);
        raw ^= inv;
        (void) gpa_descramble_t17_bit(&reg, raw);
    }

    for (int i = 0; i < out_n; i++) {
        int d = r->symbols[start_sym + 23 + i].dibit & 3;
        int raw = (map == 0) ? ((d >> 1) & 1) : (d & 1);
        raw ^= inv;
        out_bits[i] = (uint8_t) gpa_descramble_t17_bit(&reg, raw);
    }

    if (first_sample_out)
        *first_sample_out = r->symbols[start_sym + 23].sample_index;
    return out_n;
}

static int unpacked_slice_pack(const uint8_t *src_bits,
                               int src_count,
                               int off,
                               uint8_t *packed_out,
                               int packed_cap_bytes)
{
    int nbits;
    int nbytes;

    if (!src_bits || !packed_out || packed_cap_bytes <= 0 || off < 0 || off >= src_count)
        return 0;
    nbits = src_count - off;
    nbytes = (nbits + 7) / 8;
    if (nbytes > packed_cap_bytes)
        nbytes = packed_cap_bytes;
    memset(packed_out, 0, (size_t) nbytes);

    for (int i = 0; i < nbytes * 8 && (off + i) < src_count; i++) {
        if (src_bits[off + i])
            packed_out[i / 8] |= (uint8_t) (1U << (i % 8));
    }
    return nbytes * 8;
}

static bool demod_ja_search(v92_p3_rx_t *rx, ja_dil_decode_t *out)
{
    int eval_total;
    p3_hypothesis_t hyps[TRN1U_DEMOD_MAX_HYP];
    bool found = false;
    ja_dil_decode_t best;
    int best_score = -1000000;
    int considered = 0;
    int strict_hits = 0;

    if (!rx || !out)
        return false;
    if (rx->ja_buf_fill < 24 + V92_P3_RX_TRN1U_MIN_T + 206)
        return false;

    memset(&best, 0, sizeof(best));
    eval_total = rx->ja_buf_fill;

    for (int law = 0; law < 2; law++) {
        int16_t *lin = (int16_t *) malloc((size_t) eval_total * sizeof(int16_t));
        int count;

        if (!lin)
            break;
        for (int i = 0; i < eval_total; i++) {
            uint8_t cw = rx->ja_buf[i];
            lin[i] = (int16_t) (law ? alaw_to_linear(cw) : ulaw_to_linear(cw));
        }

        count = p3_scan_all_hypotheses(lin,
                                       eval_total,
                                       rx->ja_buf_base,
                                       8000,
                                       hyps,
                                       TRN1U_DEMOD_MAX_HYP);

        for (int hi = 0; hi < count; hi++) {
            p3_result_t *r = p3_demod_run(lin,
                                          eval_total,
                                          rx->ja_buf_base,
                                          hyps[hi].baud_code,
                                          hyps[hi].carrier_sel,
                                          8000);
            if (!r)
                continue;

            for (int map = 0; map < 2; map++) {
                for (int inv = 0; inv < 2; inv++) {
                    uint8_t bits[JA_DEMOD_MAX_BITS];
                    uint8_t packed[512];
                    int bit_count;
                    int bit_base_sample = -1;
                    int trn_eval_bits;
                    int trn_pct;
                    int anchor_start;
                    int anchor_end;
                    int variant_hits = 0;
                    double sym_ratio;
                    int expected_trn_bits;
                    int lead_bits;

                    bit_count = demod_build_gpa_bits(r,
                                                     rx->trn1u_start,
                                                     map,
                                                     inv,
                                                     bits,
                                                     JA_DEMOD_MAX_BITS,
                                                     &bit_base_sample);
                    sym_ratio = (eval_total > 0)
                        ? ((double) r->symbol_count / (double) eval_total)
                        : 0.40;
                    if (sym_ratio < 0.20)
                        sym_ratio = 0.20;
                    if (sym_ratio > 0.60)
                        sym_ratio = 0.60;
                    expected_trn_bits = (int) lround((double) (V92_P3_RX_TRN1U_MIN_T - 23) * sym_ratio);
                    lead_bits = (int) lround((double) V92_P3_RX_JA_LEAD_T * sym_ratio);
                    if (expected_trn_bits > bit_count - (24 + 206))
                        expected_trn_bits = bit_count - (24 + 206);
                    if (expected_trn_bits < 128)
                        expected_trn_bits = 128;

                    if (p3rx_debug_enabled() && hi == 0 && map == 0 && inv == 0) {
                        int s0 = (r->symbol_count > 0) ? r->symbols[0].sample_index : -1;
                        int s1 = (r->symbol_count > 0) ? r->symbols[r->symbol_count - 1].sample_index : -1;
                        fprintf(stderr,
                                "[P3RX] demod_ja hyp0 sym_count=%d span=%d..%d bit_count=%d exp_trn=%d lead=%d base_bit_sample=%d\n",
                                r->symbol_count, s0, s1, bit_count,
                                expected_trn_bits, lead_bits, bit_base_sample);
                    }

                    if (bit_count < (128 + 24 + 206))
                        continue;
                    considered++;

                    trn_eval_bits = expected_trn_bits;
                    if (trn_eval_bits > bit_count)
                        trn_eval_bits = bit_count;
                    trn_pct = unpacked_ones_pct(bits, trn_eval_bits);
                    if (trn_pct < TRN1U_ONES_MIN_PCT)
                        continue;

                    anchor_start = expected_trn_bits - 64;
                    if (anchor_start < 0)
                        anchor_start = 0;
                    anchor_end = expected_trn_bits + lead_bits + 128;
                    if (anchor_end > bit_count - 24 - 206)
                        anchor_end = bit_count - 24 - 206;

                    for (int anchor = anchor_start; anchor <= anchor_end; anchor++) {
                        v90_dil_desc_t desc;
                        v90_dil_analysis_t analysis;
                        v92_ja_parse_meta_t meta;
                        int desc_off = anchor + 24;
                        int packed_bits;
                        int score = 0;
                        bool preamble_ok = true;

                        for (int i = 0; i < 24; i++) {
                            if (!bits[anchor + i]) {
                                preamble_ok = false;
                                break;
                            }
                        }
                        if (!preamble_ok)
                            continue;

                        packed_bits = unpacked_slice_pack(bits,
                                                          bit_count,
                                                          desc_off,
                                                          packed,
                                                          (int) sizeof(packed));
                        if (packed_bits < 206)
                            continue;

                        memset(&meta, 0, sizeof(meta));
                        if (!v92_parse_ja_descriptor_strict(&desc,
                                                            packed,
                                                            packed_bits,
                                                            &meta))
                            continue;
                        variant_hits++;
                        strict_hits++;
                        if (!v90_analyse_dil_descriptor(&desc, &analysis))
                            continue;

                        score = (meta.is_v92 ? 2000 : 1000)
                                + (int) analysis.unique_train_u * 100
                                + (int) analysis.used_uchords * 20
                                - (int) analysis.impairment_score * 5;
                        if (score > best_score) {
                            memset(&best, 0, sizeof(best));
                            best.ok = true;
                            best.soft_lock = false;
                            best.calling_party = true;
                            best.u_info = 0;
                            best.start_sample = bit_base_sample + desc_off;
                            best.invert_sign = inv ? true : false;
                            best.parsed_v92 = meta.is_v92;
                            best.descriptor_bits = meta.bit_len;
                            best.desc = desc;
                            best.analysis = analysis;
                            best_score = score;
                            found = true;
                        }
                    }

                    if (variant_hits == 0) {
                        int desc_start = expected_trn_bits - 64;
                        int desc_end = expected_trn_bits + lead_bits + 128;
                        if (desc_start < 0)
                            desc_start = 0;
                        if (desc_end > bit_count - 206)
                            desc_end = bit_count - 206;
                        for (int desc_off = desc_start; desc_off <= desc_end; desc_off++) {
                            v90_dil_desc_t desc;
                            v90_dil_analysis_t analysis;
                            v92_ja_parse_meta_t meta;
                            int packed_bits;
                            int score;

                            packed_bits = unpacked_slice_pack(bits,
                                                              bit_count,
                                                              desc_off,
                                                              packed,
                                                              (int) sizeof(packed));
                            if (packed_bits < 206)
                                continue;
                            memset(&meta, 0, sizeof(meta));
                            if (!v92_parse_ja_descriptor_strict(&desc,
                                                                packed,
                                                                packed_bits,
                                                                &meta))
                                continue;
                            variant_hits++;
                            strict_hits++;
                            if (!v90_analyse_dil_descriptor(&desc, &analysis))
                                continue;
                            score = (meta.is_v92 ? 2000 : 1000)
                                    + (int) analysis.unique_train_u * 100
                                    + (int) analysis.used_uchords * 20
                                    - (int) analysis.impairment_score * 5;
                            if (score > best_score) {
                                memset(&best, 0, sizeof(best));
                                best.ok = true;
                                best.soft_lock = false;
                                best.calling_party = true;
                                best.u_info = 0;
                                best.start_sample = bit_base_sample + desc_off;
                                best.invert_sign = inv ? true : false;
                                best.parsed_v92 = meta.is_v92;
                                best.descriptor_bits = meta.bit_len;
                                best.desc = desc;
                                best.analysis = analysis;
                                best_score = score;
                                found = true;
                            }
                        }
                    }
                }
            }

            p3_result_free(r);
        }
        free(lin);
    }

    if (p3rx_debug_enabled()) {
        fprintf(stderr,
                "[P3RX] demod_ja_search considered=%d strict_hits=%d found=%d best_score=%d\n",
                considered, strict_hits, found ? 1 : 0, best_score);
    }

    if (found)
        *out = best;
    return found;
}

static void ja_result_normalize_sample(const v92_p3_rx_t *rx, ja_dil_decode_t *res)
{
    if (!rx || !res || res->start_sample < 0)
        return;
    /*
     * v92_ja_dil_search may report a start index relative to ja_buf[0].
     * Promote such values to the absolute sample timeline.
     */
    if (res->start_sample < rx->ja_buf_fill)
        res->start_sample += rx->ja_buf_base;
}

/* -------------------------------------------------------------------------
 * Ja codeword buffer
 * ------------------------------------------------------------------------- */
static void ja_buf_push(v92_p3_rx_t *rx, uint8_t cw, int sample_index)
{
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

/* -------------------------------------------------------------------------
 * Ja search — called once the buffer has enough data
 * ------------------------------------------------------------------------- */
static bool run_ja_search(v92_p3_rx_t *rx)
{
    ja_dil_search_params_t params;
    int buf_trn_off;
    int trn_min_t;
    int ja_lead_t;

    memset(&params, 0, sizeof(params));
    trn_min_t = rx->p6_soft_mode ? TRN1U_MIN_SOFT_T : V92_P3_RX_TRN1U_MIN_T;
    ja_lead_t = rx->p6_soft_mode ? JA_LEAD_SOFT_T : V92_P3_RX_JA_LEAD_T;

    /* Offset of trn1u_start within ja_buf. */
    buf_trn_off = rx->trn1u_start - rx->ja_buf_base;
    if (buf_trn_off < 24)
        buf_trn_off = 24;   /* need at least 23 seed samples before search */

    /*
     * Ja is expected to appear right after TRN1u minimum.
     * Open a ±50 sample window around that point, plus JA lead slack.
     */
    params.search_start  = buf_trn_off + trn_min_t - 50;
    params.search_end    = buf_trn_off + trn_min_t + ja_lead_t;
    params.tx_ja_sample  = -1;
    params.u_info        = 0;
    params.calling_party = true;

    if (params.search_start < 24)
        params.search_start = 24;

    int max_end = rx->ja_buf_fill - 207;
    if (params.search_end > max_end)
        params.search_end = max_end;

    if (params.search_end <= params.search_start) {
        p3rx_set_reject(rx,
                        V92_P3_RX_REJECT_JA_SEARCH_FAIL,
                        rx->trn1u_start + rx->trn1u_count,
                        params.search_start,
                        params.search_end);
        return false;
    }

    memset(&rx->ja_result, 0, sizeof(rx->ja_result));
    bool found = v92_ja_dil_search(rx->ja_buf, rx->ja_buf_fill,
                                   &params, &rx->ja_result);
    if (found)
        ja_result_normalize_sample(rx, &rx->ja_result);
    if (found && rx->ja_result.ok)
        return true;

    /* Try to upgrade soft-lock to strict parse via symbol-domain demod path. */
    if (demod_ja_search(rx, &rx->ja_result)) {
        ja_result_normalize_sample(rx, &rx->ja_result);
        return true;
    }

    if (found && rx->ja_result.soft_lock) {
        p3rx_set_reject(rx,
                        V92_P3_RX_REJECT_JA_SOFT_ONLY,
                        rx->ja_result.start_sample,
                        rx->ja_result.soft_score,
                        rx->ja_result.descriptor_bits);
        return true;
    }

    p3rx_set_reject(rx,
                    V92_P3_RX_REJECT_JA_SEARCH_FAIL,
                    rx->trn1u_start + rx->trn1u_count,
                    rx->ja_buf_fill,
                    0);
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
    rx->last_reject = V92_P3_RX_REJECT_NONE;
    rx->last_reject_sample = -1;
    rx->last_reject_metric0 = 0;
    rx->last_reject_metric1 = 0;
    p6_reset(rx);
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
                rx->state = V92_P3_RX_MD_WAIT;
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
                rx->state = V92_P3_RX_MD_WAIT;
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
        if (elapsed > V92_P3_RX_MD_MAX_T) {
            p6_rehunt_from_current(rx, codeword, sample_index,
                                   V92_P3_RX_REJECT_MD_TIMEOUT,
                                   elapsed, V92_P3_RX_MD_MAX_T);
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
                int copied;
                int base_sample = sample_index;
                bool soft_path = rx->p6_soft_mode;
                rx->ur2_end = sample_index - 1;
                rx->trn1u_start = sample_index;
                /* Seed Ja buffer with true codeword prehistory for GPA sync. */
                copied = prehist_copy_tail(rx, 23, rx->ja_buf, &base_sample);
                rx->ja_buf_base = (copied > 0) ? base_sample : rx->trn1u_start;
                rx->ja_buf_fill = copied;
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
        int trn_ones_min_pct = rx->p6_soft_mode ? TRN1U_ONES_MIN_PCT_SOFT : TRN1U_ONES_MIN_PCT;
        int ja_lead_t = rx->p6_soft_mode ? JA_LEAD_SOFT_T : V92_P3_RX_JA_LEAD_T;
        /* Buffer every codeword from trn1u_start − 23 onwards. */
        if (sample_index >= rx->ja_buf_base)
            ja_buf_push(rx, codeword, sample_index);

        trn1u_process(rx, codeword);

        /*
         * TRN1u (V.92 §8.5.7) is an all-ones training source through the GPA
         * scrambler/differential path. If descrambled bits are not strongly
         * one-biased, this lock is likely wrong.
         */
        if (rx->trn1u_count == TRN1U_EARLY_CHECK_T
            && !trn1u_ones_ok_at_count(rx, TRN1U_EARLY_CHECK_T)) {
            int eq_pct = trn1u_demod_best_ones_pct(rx, TRN1U_EARLY_CHECK_T);
            if (p3rx_debug_enabled()) {
                int raw_pct = (rx->trn1u_count > 0)
                    ? ((rx->trn1u_ones * 100 + rx->trn1u_count / 2) / rx->trn1u_count)
                    : 0;
                fprintf(stderr,
                        "[P3RX] sample=%d trn1u early check raw=%d%% eq=%d%% min=%d%% soft=%d\n",
                        sample_index, raw_pct, eq_pct, trn_ones_min_pct, rx->p6_soft_mode ? 1 : 0);
            }
            if (eq_pct < trn_ones_min_pct) {
                int raw_pct = (rx->trn1u_count > 0)
                    ? ((rx->trn1u_ones * 100 + rx->trn1u_count / 2) / rx->trn1u_count)
                    : 0;
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_TRN1U_ONES_LOW,
                                       raw_pct, eq_pct);
                break;
            }
        }
        if (rx->trn1u_count == trn_min_t
            && !trn1u_ones_ok_at_count(rx, trn_min_t)) {
            int eq_pct = trn1u_demod_best_ones_pct(rx, trn_min_t);
            if (p3rx_debug_enabled()) {
                int raw_pct = (rx->trn1u_count > 0)
                    ? ((rx->trn1u_ones * 100 + rx->trn1u_count / 2) / rx->trn1u_count)
                    : 0;
                fprintf(stderr,
                        "[P3RX] sample=%d trn1u min check raw=%d%% eq=%d%% min=%d%% soft=%d\n",
                        sample_index, raw_pct, eq_pct, trn_ones_min_pct, rx->p6_soft_mode ? 1 : 0);
            }
            if (eq_pct < trn_ones_min_pct) {
                int raw_pct = (rx->trn1u_count > 0)
                    ? ((rx->trn1u_ones * 100 + rx->trn1u_count / 2) / rx->trn1u_count)
                    : 0;
                p6_rehunt_from_current(rx, codeword, sample_index,
                                       V92_P3_RX_REJECT_TRN1U_ONES_LOW,
                                       raw_pct, eq_pct);
                break;
            }
        }

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

        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_JA_SEARCH:
    {
        int trn_min_t = rx->p6_soft_mode ? TRN1U_MIN_SOFT_T : V92_P3_RX_TRN1U_MIN_T;
        int ja_lead_t = rx->p6_soft_mode ? JA_LEAD_SOFT_T : V92_P3_RX_JA_LEAD_T;
        int ready_min = 24 + trn_min_t + ja_lead_t + 207;
        ja_buf_push(rx, codeword, sample_index);

        if (rx->ja_buf_fill >= V92_P3_RX_JA_BUF) {
            p3rx_set_reject(rx,
                            V92_P3_RX_REJECT_JA_BUFFER_FULL,
                            sample_index,
                            rx->ja_buf_fill,
                            V92_P3_RX_JA_BUF);
            rx->state = V92_P3_RX_FAILED;
            break;
        }

        /* Run the search once we have enough buffered. */
        if (rx->ja_buf_fill >= ready_min) {
            if (run_ja_search(rx)) {
                rx->ja_found = true;
                rx->state    = V92_P3_RX_DONE;
            } else {
                p3rx_set_reject(rx,
                                V92_P3_RX_REJECT_JA_SEARCH_FAIL,
                                sample_index,
                                rx->ja_buf_fill,
                                0);
                rx->state = V92_P3_RX_FAILED;
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
