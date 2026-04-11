/*
 * v92_p3_rx.c — V.92 Phase 3 upstream receiver (PCM domain)
 *
 * See v92_p3_rx.h for protocol overview and design notes.
 */

#include "v92_p3_rx.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------
 * Internal constants
 * ------------------------------------------------------------------------- */

/* Minimum run before promoting a period-6 lock to Ru. */
#define P6_LOCK_MIN    320

/* Acceptance window for Ru burst length. */
#define RU_ACCEPT_MIN  (V92_P3_RX_RU_T - 24)
#define RU_ACCEPT_MAX  (V92_P3_RX_RU_T + 36)
/* Hard cap to avoid latching forever on unrelated long period-6 regions. */
#define RU_MAX_T       (V92_P3_RX_RU_T * 8)

/* Acceptance window for uR burst length. */
#define UR_ACCEPT_MIN  (V92_P3_RX_UR_T - 8)
#define UR_ACCEPT_MAX  (V92_P3_RX_UR_T * 2 + 8)
/* Hard cap for uR; true uR should be short. */
#define UR_MAX_T       (V92_P3_RX_UR_T * 6)

/* -------------------------------------------------------------------------
 * Sign bit helpers
 * ------------------------------------------------------------------------- */

static inline int sign_bit(uint8_t cw)  { return (cw >> 7) & 1; }

static int p3rx_debug_enabled(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *v = getenv("V92_P3_RX_DEBUG");
        cached = (v && *v && *v != '0') ? 1 : 0;
    }
    return cached;
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

static void p6_update_hyp_runs(v92_p3_rx_t *rx, int msb, int sample_index)
{
    for (int h = 0; h < 12; h++) {
        bool pol = p6_hyp_pol(h);
        int phase = (sample_index + p6_hyp_phase0(h)) % 6;
        int expected = p6_exp(phase, pol);

        if (msb == expected)
            rx->p6_hyp_run[h]++;
        else
            rx->p6_hyp_run[h] = 0;
    }
}

static int p6_best_hyp_run(const v92_p3_rx_t *rx, bool ru_pol, int min_run)
{
    int best_h = -1;
    int best_r = min_run - 1;
    int base = ru_pol ? 0 : 6;

    for (int p = 0; p < 6; p++) {
        int h = base + p;
        int r = rx->p6_hyp_run[h];
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
    rx->ru_hyp = -1;
    rx->ur_hyp = -1;
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

/* -------------------------------------------------------------------------
 * Ja search — called once the buffer has enough data
 * ------------------------------------------------------------------------- */
static bool run_ja_search(v92_p3_rx_t *rx)
{
    ja_dil_search_params_t params;
    int buf_trn_off;

    memset(&params, 0, sizeof(params));

    /* Offset of trn1u_start within ja_buf. */
    buf_trn_off = rx->trn1u_start - rx->ja_buf_base;
    if (buf_trn_off < 24)
        buf_trn_off = 24;   /* need at least 23 seed samples before search */

    /*
     * Ja is expected to appear right after 2040T of TRN1u.
     * Open a ±50 sample window around that point, plus JA_LEAD_T for slack.
     */
    params.search_start  = buf_trn_off + V92_P3_RX_TRN1U_MIN_T - 50;
    params.search_end    = buf_trn_off + V92_P3_RX_TRN1U_MIN_T
                           + V92_P3_RX_JA_LEAD_T;
    params.tx_ja_sample  = -1;
    params.u_info        = 0;
    params.calling_party = true;

    if (params.search_start < 24)
        params.search_start = 24;

    int max_end = rx->ja_buf_fill - 207;
    if (params.search_end > max_end)
        params.search_end = max_end;

    if (params.search_end <= params.search_start)
        return false;

    memset(&rx->ja_result, 0, sizeof(rx->ja_result));
    bool found = v92_ja_dil_search(rx->ja_buf, rx->ja_buf_fill,
                                   &params, &rx->ja_result);
    return found && (rx->ja_result.ok || rx->ja_result.soft_lock);
}

static void p6_rehunt_from_current(v92_p3_rx_t *rx, int msb, int sample_index)
{
    p6_reset(rx);
    p6_update_hyp_runs(rx, msb, sample_index);
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
    p6_reset(rx);
}

void v92_p3_rx_start(v92_p3_rx_t *rx, int first_sample_index)
{
    v92_p3_rx_init(rx);
    (void)first_sample_index;
    rx->state = V92_P3_RX_RU1_HUNT;
}

bool v92_p3_rx_feed(v92_p3_rx_t *rx, uint8_t codeword, int sample_index)
{
    v92_p3_rx_state_t prev = rx->state;
    int msb = sign_bit(codeword);

    if (rx->state == V92_P3_RX_RU1_HUNT
        || rx->state == V92_P3_RX_RU1
        || rx->state == V92_P3_RX_UR1
        || rx->state == V92_P3_RX_MD_WAIT
        || rx->state == V92_P3_RX_RU2
        || rx->state == V92_P3_RX_UR2) {
        p6_update_hyp_runs(rx, msb, sample_index);
    }

    switch (rx->state) {

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_IDLE:
        break;

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU1_HUNT: {
        int h_ru = p6_best_hyp_run(rx, true, P6_LOCK_MIN);
        int h_ur = p6_best_hyp_run(rx, false, P6_LOCK_MIN);
        int best_h = -1;

        if (h_ru >= 0)
            best_h = h_ru;
        if (h_ur >= 0
            && (best_h < 0 || rx->p6_hyp_run[h_ur] > rx->p6_hyp_run[best_h])) {
            best_h = h_ur;
        }

        if (best_h >= 0) {
            rx->ru_hyp = best_h;
            rx->ur_hyp = p6_hyp_index(!p6_hyp_pol(best_h), p6_hyp_phase0(best_h));
            rx->p6_ru_polarity = p6_hyp_pol(best_h);
            rx->p6_run = rx->p6_hyp_run[best_h];
            rx->p6_locked = true;
            rx->ru1_start = sample_index - rx->p6_run + 1;
            rx->state = V92_P3_RX_RU1;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU1: {
        int run = (rx->ru_hyp >= 0) ? rx->p6_hyp_run[rx->ru_hyp] : 0;

        if (run > 0) {
            rx->p6_run = run;
            if (run > RU_MAX_T)
                p6_rehunt_from_current(rx, msb, sample_index);
        } else {
            int run_effective = rx->p6_run;

            if (run_effective >= RU_ACCEPT_MIN
                && run_effective <= RU_ACCEPT_MAX
                && rx->ur_hyp >= 0
                && rx->p6_hyp_run[rx->ur_hyp] > 0) {
                rx->ru1_end = sample_index - 1;
                rx->ur1_start = sample_index;
                rx->p6_run = rx->p6_hyp_run[rx->ur_hyp];
                rx->state = V92_P3_RX_UR1;
            } else {
                p6_rehunt_from_current(rx, msb, sample_index);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_UR1: {
        int run = (rx->ur_hyp >= 0) ? rx->p6_hyp_run[rx->ur_hyp] : 0;

        if (run > 0) {
            rx->p6_run = run;
            if (run > UR_MAX_T)
                p6_rehunt_from_current(rx, msb, sample_index);
        } else {
            int run_effective = rx->p6_run;
            if (run_effective >= UR_ACCEPT_MIN && run_effective <= UR_ACCEPT_MAX) {
                rx->ur1_end = sample_index - 1;
                rx->p6_run = 0;
                rx->state = V92_P3_RX_MD_WAIT;
            } else {
                p6_rehunt_from_current(rx, msb, sample_index);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_MD_WAIT: {
        int elapsed = sample_index - rx->ur1_end;
        if (elapsed > V92_P3_RX_MD_MAX_T) {
            p6_rehunt_from_current(rx, msb, sample_index);
            break;
        }

        if (rx->ru_hyp < 0) {
            p6_rehunt_from_current(rx, msb, sample_index);
            break;
        }

        {
            int run = rx->p6_hyp_run[rx->ru_hyp];
            if (run >= P6_LOCK_MIN) {
                rx->p6_run = run;
                rx->ru2_start = sample_index - run + 1;
                rx->state = V92_P3_RX_RU2;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU2: {
        int run = (rx->ru_hyp >= 0) ? rx->p6_hyp_run[rx->ru_hyp] : 0;

        if (run > 0) {
            rx->p6_run = run;
            if (run > RU_MAX_T)
                p6_rehunt_from_current(rx, msb, sample_index);
        } else {
            int run_effective = rx->p6_run;

            if (run_effective >= RU_ACCEPT_MIN
                && run_effective <= RU_ACCEPT_MAX
                && rx->ur_hyp >= 0
                && rx->p6_hyp_run[rx->ur_hyp] > 0) {
                rx->ru2_end = sample_index - 1;
                rx->ur2_start = sample_index;
                rx->p6_run = rx->p6_hyp_run[rx->ur_hyp];
                rx->state = V92_P3_RX_UR2;
            } else {
                p6_rehunt_from_current(rx, msb, sample_index);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_UR2: {
        int run = (rx->ur_hyp >= 0) ? rx->p6_hyp_run[rx->ur_hyp] : 0;

        if (run > 0) {
            rx->p6_run = run;
            if (run > UR_MAX_T)
                p6_rehunt_from_current(rx, msb, sample_index);
        } else {
            int run_effective = rx->p6_run;
            if (run_effective >= UR_ACCEPT_MIN && run_effective <= UR_ACCEPT_MAX) {
                rx->ur2_end = sample_index - 1;
                rx->trn1u_start = sample_index;
                /* Set up Ja buffer to start 23 samples before TRN1u
                 * so v92_ja_dil_search has its seed window. */
                rx->ja_buf_base  = rx->trn1u_start - 23;
                rx->ja_buf_fill  = 0;
                rx->gpa_reg      = 0;
                rx->diff_valid   = false;
                rx->trn1u_count  = 0;
                rx->trn1u_ones   = 0;
                p6_reset(rx);
                rx->state = V92_P3_RX_TRN1U;
            } else {
                p6_rehunt_from_current(rx, msb, sample_index);
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_TRN1U:
        /* Buffer every codeword from trn1u_start − 23 onwards. */
        if (sample_index >= rx->ja_buf_base)
            ja_buf_push(rx, codeword, sample_index);

        trn1u_process(rx, codeword);

        if (rx->ja_buf_fill >= V92_P3_RX_JA_BUF) {
            rx->state = V92_P3_RX_FAILED;
            break;
        }

        /* Once ≥ TRN1U_MIN_T + JA_LEAD_T codewords buffered after
         * trn1u_start, we should have enough for the Ja search. */
        if (rx->trn1u_count >= V92_P3_RX_TRN1U_MIN_T + V92_P3_RX_JA_LEAD_T)
            rx->state = V92_P3_RX_JA_SEARCH;

        break;

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_JA_SEARCH:
        ja_buf_push(rx, codeword, sample_index);

        if (rx->ja_buf_fill >= V92_P3_RX_JA_BUF) {
            rx->state = V92_P3_RX_FAILED;
            break;
        }

        /* Run the search once we have enough buffered. */
        if (rx->ja_buf_fill >= 24 + V92_P3_RX_TRN1U_MIN_T
                                  + V92_P3_RX_JA_LEAD_T + 207) {
            if (run_ja_search(rx)) {
                rx->ja_found = true;
                rx->state    = V92_P3_RX_DONE;
            } else {
                rx->state = V92_P3_RX_FAILED;
            }
        }
        break;

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_DONE:
    case V92_P3_RX_FAILED:
        break;
    }

    if (rx->state != prev && p3rx_debug_enabled()) {
        int ru_run = (rx->ru_hyp >= 0) ? rx->p6_hyp_run[rx->ru_hyp] : -1;
        int ur_run = (rx->ur_hyp >= 0) ? rx->p6_hyp_run[rx->ur_hyp] : -1;
        fprintf(stderr,
                "[P3RX] sample=%d %s->%s p6_run=%d ru_run=%d ur_run=%d ru_hyp=%d ur_hyp=%d\n",
                sample_index,
                v92_p3_rx_state_name(prev),
                v92_p3_rx_state_name(rx->state),
                rx->p6_run,
                ru_run,
                ur_run,
                rx->ru_hyp,
                rx->ur_hyp);
    }
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
