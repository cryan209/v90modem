/*
 * v92_p3_rx.h — V.92 Phase 3 upstream receiver (PCM domain)
 *
 * Detects and decodes the V.92 analogue-modem Phase 3 training sequence
 * directly from raw G.711 PCM codewords (one codeword per 125 µs timeslot):
 *
 *   Ru  (384T)  {+L_U,+L_U,+L_U,−L_U,−L_U,−L_U} × 64
 *   uR   (24T)  {−L_U,−L_U,−L_U,+L_U,+L_U,+L_U} × 4
 *   [MD optional]
 *   Ru  (384T)  repeat
 *   uR   (24T)  repeat
 *   TRN1u (≥2040T)  GPA-scrambled ±L_U (V.92 §8.5.7)
 *   Ja          24 ones preamble + DIL descriptor (V.92 §8.5.4)
 *
 * All signals originate from the analogue modem (caller); we are the
 * digital side (answerer) receiving them in the G.711 RTP stream.
 *
 * Sign convention used throughout (V.92 §8.5.7):
 *   PCM MSB = 1 (positive, +L_U) → decoded bit 0
 *   PCM MSB = 0 (negative, −L_U) → decoded bit 1
 *
 * GPA polynomial: x^23 + x^18 + 1  (V.92 §6.3, V.34 eq. 7-2, tap = 17)
 */

#ifndef V92_P3_RX_H
#define V92_P3_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "v92_ja_decode.h"   /* ja_dil_decode_t */

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Timing constants (ITU-T V.92 §8.5)
 * All in units of T = one PCM symbol = one G.711 codeword = 125 µs.
 * ------------------------------------------------------------------------- */
#define V92_P3_RX_RU_T          384   /* Ru burst length */
#define V92_P3_RX_UR_T           24   /* uR burst length */
#define V92_P3_RX_TRN1U_MIN_T  2040   /* TRN1u minimum before Ja */
#define V92_P3_RX_MD_MAX_T     8000   /* max MD gap between uR1 and Ru2 (1 s) */

/* Extra codewords buffered before calling v92_ja_dil_search. */
#define V92_P3_RX_JA_LEAD_T     200   /* give Ja 200 T to appear after TRN1u */

/* Codeword buffer for the Ja search (seed + TRN1u + Ja descriptor). */
#define V92_P3_RX_JA_BUF       6144   /* ≥ 23 + TRN1u_MIN + DIL + slack */
/* Rolling prehistory so TRN1u/Ja decoding gets true pre-seed symbols. */
#define V92_P3_RX_PRE_HIST       64

/* -------------------------------------------------------------------------
 * Receiver state
 * ------------------------------------------------------------------------- */
typedef enum {
    V92_P3_RX_IDLE = 0,    /* not yet started */
    V92_P3_RX_RU1_HUNT,    /* hunting for first Ru period-6 onset */
    V92_P3_RX_RU1,         /* accumulating Ru1 (target 384T) */
    V92_P3_RX_UR1,         /* accumulating uR1 (target 24T) */
    V92_P3_RX_MD_WAIT,     /* waiting for Ru2 or TRN1u (optional MD gap) */
    V92_P3_RX_RU2,         /* accumulating Ru2 (target 384T) */
    V92_P3_RX_UR2,         /* accumulating uR2 (target 24T) */
    V92_P3_RX_TRN1U,       /* accumulating TRN1u (target ≥ 2040T) */
    V92_P3_RX_JA_SEARCH,   /* v92_ja_dil_search in progress */
    V92_P3_RX_DONE,        /* Ja decoded successfully */
    V92_P3_RX_FAILED,      /* could not decode */
} v92_p3_rx_state_t;

typedef enum {
    V92_P3_RX_REJECT_NONE = 0,
    V92_P3_RX_REJECT_PRE_ARM,
    V92_P3_RX_REJECT_RU_MISMATCH,
    V92_P3_RX_REJECT_UR_MISMATCH,
    V92_P3_RX_REJECT_MD_TIMEOUT,
    V92_P3_RX_REJECT_TRN1U_ONES_LOW,
    V92_P3_RX_REJECT_JA_BUFFER_FULL,
    V92_P3_RX_REJECT_JA_SEARCH_FAIL,
    V92_P3_RX_REJECT_JA_SOFT_ONLY,
} v92_p3_rx_reject_t;

/* -------------------------------------------------------------------------
 * Receiver context (caller allocates, typically on the heap)
 * ------------------------------------------------------------------------- */
typedef struct {
    v92_p3_rx_state_t state;

    /* ------- period-6 tracking ------- */

    /*
     * p6_phase:  index 0-5 within the 6-symbol cycle, aligned to Ru polarity.
     *            Expected sign at phase p is (p < 3) ? 1 : 0 for Ru polarity,
     *            inverted for uR polarity.
     * p6_locked: true once ≥ 12 consecutive on-pattern symbols observed.
     * p6_run:    symbols accumulated in current run.
     * p6_err_window: error count within the current 12-symbol window.
     * p6_err_wpos:   position counter for window reset.
     */
    int      p6_phase;
    bool     p6_locked;
    int      p6_run;
    int      p6_err_window;  /* errors in last 12-symbol window */
    int      p6_err_wpos;    /* position counter for window reset */
    bool     p6_ru_polarity; /* true = Ru {+,+,+,-,-,-}, false = uR */
    /* 12-hypothesis run tracker: 6 phases x 2 polarities. */
    int      p6_hyp_run[12];
    uint32_t p6_hyp_sum[12];
    uint32_t p6_hyp_sumsq[12];
    uint8_t  p6_hyp_min[12];
    uint8_t  p6_hyp_max[12];
    int      ru_hyp;         /* active Ru hypothesis index, or -1 */
    int      ur_hyp;         /* active uR hypothesis index, or -1 */

    /* ------- phase 3 sample anchors ------- */
    int      ru1_start;
    int      ru1_end;
    int      ur1_start;
    int      ur1_end;
    int      ru2_start;
    int      ru2_end;
    int      ur2_start;
    int      ur2_end;
    int      trn1u_start;
    int      arm_sample_min; /* ignore Phase-3 lock before this sample */

    /* ------- TRN1u accumulator ------- */
    int      trn1u_count;    /* symbols accumulated */
    int      trn1u_ones;     /* GPA-descrambled bits that were 1 */
    uint32_t gpa_reg;        /* GPA shift register (x^23+x^18+1) */
    int      diff_prev;      /* previous sign bit for differential decode */
    bool     diff_valid;     /* true once diff_prev is initialised */

    /* ------- rolling prehistory ------- */
    uint8_t  prehist_cw[V92_P3_RX_PRE_HIST];
    int      prehist_sample[V92_P3_RX_PRE_HIST];
    int      prehist_head;    /* next write index */
    int      prehist_fill;    /* number of valid entries */

    /* ------- Ja codeword buffer ------- */
    /*
     * Filled with rolling prehistory first (up to 23 symbols before TRN1u),
     * then with live TRN1u/Ja codewords so v92_ja_dil_search has a proper
     * GPA seed window before the search region.
     */
    uint8_t  ja_buf[V92_P3_RX_JA_BUF];
    int      ja_buf_base;    /* sample index corresponding to ja_buf[0] */
    int      ja_buf_fill;    /* number of valid codewords in ja_buf */

    /* ------- result ------- */
    bool           ja_found;
    ja_dil_decode_t ja_result;
    int              reject_count;
    v92_p3_rx_reject_t last_reject;
    int              last_reject_sample;
    int              last_reject_metric0;
    int              last_reject_metric1;
} v92_p3_rx_t;

/* -------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

/*
 * Initialise (or reset) a receiver context.  State → IDLE.
 */
void v92_p3_rx_init(v92_p3_rx_t *rx);

/*
 * Arm the receiver: state → RU1_HUNT.
 * Call this once you know the analogue modem has entered Phase 3
 * (e.g. after TONEq / INFO0 exchange).
 */
void v92_p3_rx_start(v92_p3_rx_t *rx, int first_sample_index);

/*
 * Feed one raw G.711 codeword (µ-law or A-law; sign bit is MSB in both).
 *
 * @sample_index  Monotonically increasing sample counter (= RTP timestamp
 *                or your own 8000 Hz counter).  Must match the index space
 *                used when you later call v92_ja_dil_search.
 *
 * Returns true when the state has advanced (useful for logging/debug).
 */
bool v92_p3_rx_feed(v92_p3_rx_t *rx, uint8_t codeword, int sample_index);

/*
 * Convenience: feed a block of codewords.
 */
void v92_p3_rx_feed_block(v92_p3_rx_t *rx,
                          const uint8_t *codewords,
                          int            count,
                          int            first_sample_index);

/*
 * Query current state.
 */
v92_p3_rx_state_t v92_p3_rx_get_state(const v92_p3_rx_t *rx);

/*
 * Returns true when Ja has been decoded successfully.
 * Valid only after state == V92_P3_RX_DONE.
 */
bool v92_p3_rx_ja_ok(const v92_p3_rx_t *rx);

/*
 * Pointer to the decoded Ja result.  Valid only when v92_p3_rx_ja_ok().
 */
const ja_dil_decode_t *v92_p3_rx_get_ja(const v92_p3_rx_t *rx);

/*
 * Human-readable state name (for logging).
 */
const char *v92_p3_rx_state_name(v92_p3_rx_state_t s);
const char *v92_p3_rx_reject_name(v92_p3_rx_reject_t r);
v92_p3_rx_reject_t v92_p3_rx_last_reject(const v92_p3_rx_t *rx,
                                         int *sample_out,
                                         int *metric0_out,
                                         int *metric1_out);

#ifdef __cplusplus
}
#endif

#endif /* V92_P3_RX_H */
