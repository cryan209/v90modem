/*
 * v92_p3_rx.c — V.92 Phase 3 upstream receiver (PCM domain)
 *
 * See v92_p3_rx.h for protocol overview and design notes.
 */

#include "v92_p3_rx.h"

#include <string.h>
#include <stdio.h>

/* -------------------------------------------------------------------------
 * Internal constants
 * ------------------------------------------------------------------------- */

/* Tolerance: max errors per 12-symbol window during period-6 phases. */
#define P6_ERR_MAX      2

/* Minimum run to declare period-6 lock (2 complete cycles). */
#define P6_LOCK_MIN    12

/* Minimum symbols of Ru before we accept it and look for uR. */
#define RU_MIN_T       (V92_P3_RX_RU_T - 20)

/* Minimum symbols of uR before we accept it and advance. */
#define UR_MIN_T       (V92_P3_RX_UR_T - 4)

/* -------------------------------------------------------------------------
 * Sign bit helpers
 * ------------------------------------------------------------------------- */

static inline int sign_bit(uint8_t cw)  { return (cw >> 7) & 1; }

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
 * Period-6 tracker — embedded in the RX context via p6_* fields.
 * -------------------------------------------------------------------------
 *
 * We track the period-6 pattern with three parallel counters — one for each
 * possible starting alignment (we reset them all and pick the winner):
 *
 *   p6_phase       : current expected phase (0-5) within the Ru cycle
 *   p6_run         : consecutive on-pattern symbols (errors eat into this)
 *   p6_err_window  : rolling error count over last 12 symbols
 *   p6_err_wpos    : position counter for the 12-symbol window reset
 *   p6_ru_polarity : true = Ru {+,+,+,-,-,-}, false = uR {-,-,-,+,+,+}
 *   p6_locked      : true once p6_run >= P6_LOCK_MIN
 */

static void p6_reset(v92_p3_rx_t *rx)
{
    rx->p6_phase      = 0;
    rx->p6_locked     = false;
    rx->p6_run        = 0;
    rx->p6_err_window = 0;
    rx->p6_err_wpos   = 0;
    rx->p6_ru_polarity = true;
}

/*
 * p6_check() — test whether msb matches the current period-6 expectation,
 * update internal counters, and return:
 *
 *   > 0  pattern still live (on-pattern or tolerated error)
 *     0  pattern broken — caller should handle transition or re-hunt
 */
static int p6_check(v92_p3_rx_t *rx, int msb)
{
    int expected = p6_exp(rx->p6_phase, rx->p6_ru_polarity);
    int match    = (msb == expected);

    /* Advance phase regardless of match (keeps us frame-aligned). */
    rx->p6_phase = (rx->p6_phase + 1) % 6;

    /* Roll the 12-symbol error window. */
    rx->p6_err_wpos++;
    if (rx->p6_err_wpos >= 12) {
        rx->p6_err_wpos   = 0;
        rx->p6_err_window = 0;
    }

    if (!match)
        rx->p6_err_window++;

    if (rx->p6_err_window > P6_ERR_MAX) {
        p6_reset(rx);
        return 0;
    }

    rx->p6_run++;
    if (!rx->p6_locked && rx->p6_run >= P6_LOCK_MIN)
        rx->p6_locked = true;

    return 1;
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

    switch (rx->state) {

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_IDLE:
        break;

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU1_HUNT: {
        /*
         * Try to lock onto the period-6 pattern.
         * At each sample, check if msb matches the current (phase, polarity).
         * If not, also try the opposite polarity at the same phase-1 position
         * (since we just advanced phase in p6_check, check both polarities
         * for next cycle start).
         */
        if (!p6_check(rx, msb)) {
            /* p6_check reset rx — try matching from phase 0, both polarities */
            if (msb == p6_exp(0, true)) {
                rx->p6_ru_polarity = true;
            } else if (msb == p6_exp(0, false)) {
                rx->p6_ru_polarity = false;
            }
            /* phase is already 0 after p6_reset; advance it for this bit */
            rx->p6_phase = 1;
            rx->p6_run   = 1;
            break;
        }

        /* Check if the current match could also be the OTHER polarity
         * starting fresh — prefer Ru polarity over uR. */
        if (!rx->p6_locked && rx->p6_run == 1) {
            /* First matched bit: confirm polarity against MSB value. */
            int ru_phase0_exp = p6_exp(0, true);
            if (msb == ru_phase0_exp)
                rx->p6_ru_polarity = true;
        }

        if (rx->p6_locked) {
            rx->ru1_start = sample_index - rx->p6_run + 1;
            rx->state     = V92_P3_RX_RU1;
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU1: {
        /*
         * Accumulate Ru1 (expect V92_P3_RX_RU_T symbols of the current
         * period-6 polarity).  The run ends when:
         *   (a) the pattern breaks (transition to uR or noise), or
         *   (b) we've seen enough and the next 6-symbol half begins with
         *       the opposite polarity.
         */
        bool alive = (p6_check(rx, msb) != 0);

        if (!alive || rx->p6_run >= V92_P3_RX_RU_T + 12) {
            /* Pattern ended or we've well exceeded 384T. */
            if (rx->p6_run >= RU_MIN_T) {
                rx->ru1_end      = sample_index;
                rx->ur1_start    = sample_index;
                /* Flip polarity for the upcoming uR. */
                bool ur_pol      = !rx->p6_ru_polarity;
                p6_reset(rx);
                rx->p6_ru_polarity = ur_pol;
                rx->state          = V92_P3_RX_UR1;
            } else {
                p6_reset(rx);
                rx->state = V92_P3_RX_RU1_HUNT;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_UR1: {
        bool alive = (p6_check(rx, msb) != 0);

        if (!alive || rx->p6_run >= V92_P3_RX_UR_T + 12) {
            if (rx->p6_run >= UR_MIN_T) {
                rx->ur1_end = sample_index;
                p6_reset(rx);
                rx->state = V92_P3_RX_MD_WAIT;
            } else {
                /* uR too short — back to hunting */
                p6_reset(rx);
                rx->state = V92_P3_RX_RU1_HUNT;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_MD_WAIT: {
        /*
         * After uR1, wait for Ru2.  Allow up to V92_P3_RX_MD_MAX_T samples
         * for the optional MD phase.
         */
        int elapsed = sample_index - rx->ur1_end;
        if (elapsed > V92_P3_RX_MD_MAX_T) {
            rx->state = V92_P3_RX_FAILED;
            break;
        }

        /* Try both polarities for Ru2 onset. */
        bool try_pol = (msb == p6_exp(0, true)) ? true
                     : (msb == p6_exp(0, false)) ? false
                     : rx->p6_ru_polarity;

        if (msb == p6_exp(rx->p6_phase, rx->p6_ru_polarity)
            || msb == p6_exp(0, try_pol)) {

            if (msb != p6_exp(rx->p6_phase, rx->p6_ru_polarity)) {
                p6_reset(rx);
                rx->p6_ru_polarity = try_pol;
            }

            p6_check(rx, msb);

            if (rx->p6_locked) {
                rx->ru2_start = sample_index - rx->p6_run + 1;
                rx->state     = V92_P3_RX_RU2;
            }
        } else {
            p6_reset(rx);
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_RU2: {
        bool alive = (p6_check(rx, msb) != 0);

        if (!alive || rx->p6_run >= V92_P3_RX_RU_T + 12) {
            if (rx->p6_run >= RU_MIN_T) {
                rx->ru2_end      = sample_index;
                rx->ur2_start    = sample_index;
                bool ur_pol      = !rx->p6_ru_polarity;
                p6_reset(rx);
                rx->p6_ru_polarity = ur_pol;
                rx->state          = V92_P3_RX_UR2;
            } else {
                rx->state = V92_P3_RX_FAILED;
            }
        }
        break;
    }

    /* ------------------------------------------------------------------ */
    case V92_P3_RX_UR2: {
        bool alive = (p6_check(rx, msb) != 0);

        if (!alive || rx->p6_run >= V92_P3_RX_UR_T + 12) {
            if (rx->p6_run >= UR_MIN_T) {
                rx->ur2_end      = sample_index;
                rx->trn1u_start  = sample_index + 1;
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
                rx->state = V92_P3_RX_FAILED;
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
