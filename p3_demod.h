/*
 * p3_demod.h — Lightweight Phase 3 demodulator for V.34/V.90/V.92
 *
 * Offline QAM demodulator for modem training signals:
 *   - Carrier recovery (PLL) and symbol timing (Gardner TED)
 *   - 2-point and 4-point differential PSK demodulation
 *   - Descrambler (x^23 + x^5 + 1)
 *   - Pattern detectors for S/S-bar, TRN, J/Ja, Ru/uR
 *
 * Designed for batch processing of recorded PCM at 8000 Hz.
 * Replaces SpanDSP V.34 RX dependency for Phase 3 analysis.
 */

#ifndef P3_DEMOD_H
#define P3_DEMOD_H

#include <stdbool.h>
#include <stdint.h>

/* Baud rate codes matching V.34 Table 1 */
#define P3_BAUD_2400    0
#define P3_BAUD_2743    1
#define P3_BAUD_2800    2
#define P3_BAUD_3000    3
#define P3_BAUD_3200    4
#define P3_BAUD_3429    5
#define P3_BAUD_COUNT   6

/* Carrier selection */
#define P3_CARRIER_LOW  0
#define P3_CARRIER_HIGH 1

/* Dibit values (differential phase quadrants) */
#define P3_DIBIT_0      0   /*   0 degrees */
#define P3_DIBIT_1      1   /*  90 degrees */
#define P3_DIBIT_2      2   /* 180 degrees */
#define P3_DIBIT_3      3   /* 270 degrees */

/* Pattern detection results */
typedef enum {
    P3_SIGNAL_UNKNOWN = 0,
    P3_SIGNAL_SILENCE,
    P3_SIGNAL_S,            /* S (known 6-symbol training pattern) */
    P3_SIGNAL_S_BAR,        /* S-bar (complement of S) */
    P3_SIGNAL_PP,           /* PP (probing period) — not yet demodulated */
    P3_SIGNAL_TRN,          /* TRN (scrambled ones) */
    P3_SIGNAL_J,            /* J frame (16-bit repeating pattern) */
    P3_SIGNAL_J_PRIME,      /* J' (Table 19, transmitted once) */
    P3_SIGNAL_RU,           /* V.92 Ru (analog, 2-point, +LU pattern) */
    P3_SIGNAL_UR,           /* V.92 uR (analog, 2-point, -LU pattern) */
} p3_signal_type_t;

/* Baud rate parameters (derived from V.34 Table 1) */
typedef struct {
    int   baud_rate;              /* Approximate baud rate */
    int   samples_num;            /* Samples-per-symbol numerator */
    int   samples_den;            /* Samples-per-symbol denominator */
    float carrier_low_hz;         /* Low carrier frequency */
    float carrier_high_hz;        /* High carrier frequency */
} p3_baud_params_t;

/* A single demodulated symbol */
typedef struct {
    int   sample_index;           /* Sample position in input */
    float re;                     /* Baseband I component */
    float im;                     /* Baseband Q component */
    float magnitude;              /* |symbol| */
    float phase;                  /* Phase angle (radians) */
    int   quadrant;               /* Absolute received quadrant (0-3) */
    int   dibit;                  /* Differential dibit (0-3) */
    int   bit0;                   /* First descrambled bit */
    int   bit1;                   /* Second descrambled bit */
} p3_symbol_t;

/* Demodulator state */
typedef struct {
    /* Configuration */
    float carrier_hz;
    float baud_rate;
    int   sample_rate;
    int   baud_code;
    int   carrier_sel;

    /* Carrier NCO */
    uint32_t nco_phase;
    uint32_t nco_phase_inc;

    /* Carrier PLL */
    float pll_alpha;
    float pll_beta;
    float pll_freq_err;

    /* Symbol timing */
    float samples_per_symbol;
    float baud_phase;             /* Fractional position within symbol */
    float ted_alpha;              /* Timing loop gain */

    /* Integrate-and-dump accumulator */
    float idum_re;
    float idum_im;
    int   idum_count;

    /* Simple matched-filter state (causal FIR over mixed baseband) */
    float mf_hist_re[5];
    float mf_hist_im[5];
    int   mf_hist_pos;
    float mf_prev_re;
    float mf_prev_im;
    bool  mf_prev_valid;

    /* SpanDSP V.34 RRC T/2 resampler */
    float rrc_hist[27];
    int   rrc_hist_pos;
    const float (*rrc_re)[27];
    const float (*rrc_im)[27];
    int   rrc_step;
    int   rrc_steps_per_baud;
    int   rrc_half_baud;

    /* Godard band-edge timing recovery */
    float ted_low[2];
    float ted_high[2];
    float ted_dc[2];
    float ted_phase;
    float ted_low_coeff[3];
    float ted_high_coeff[3];
    float ted_mixed_coeff;
    int   timing_correction;
    bool  use_rrc_frontend;
    bool  bypass_equalizer;
    float rrc_agc_gain;
    float rrc_input_power;
    bool  use_instant_rrc_agc;
    bool  rrc_signal_active;
    bool  use_dd_equalizer;
    bool  emit_half_baud;

    /* PP-directed equalizer training (SPRA159 §3.2.3 fast equalizer).
     * When pp_train_start_sample >= 0, the equalizer switches from blind
     * CMA to LMS with the known PP reference sequence during the PP
     * segment, then to DD-QPSK after PP ends.  This trains the equalizer
     * in one shot on the known CAZAC sequence instead of relying on blind
     * convergence — the difference between detecting J on a noisy Courier
     * and not. */
    int   pp_train_start_sample;   /* -1 = disabled */
    int   pp_train_phase;          /* PP phase offset (0..47) */
    int   pp_train_symbol_count;   /* symbols trained so far */

    /* 127-tap T/2 fractionally-spaced equalizer (SpanDSP dimensions). */
    float eq_buf_re[128];
    float eq_buf_im[128];
    float eq_coeff_re[127];
    float eq_coeff_im[127];
    int   eq_buf_pos;
    float eq_delta;
    int   cma_freeze_symbols;
    int   cma_freeze_after_sample;
    int   pll_freeze_after_sample;
    int   s_alternating_run;
    int   s_previous_dibit;

    /* Previous symbol for differential decode */
    float prev_re;
    float prev_im;
    bool  prev_valid;

    /* AGC */
    float agc_gain;
    float agc_target;

    /* Descrambler (x^23 + x^5 + 1) */
    uint32_t descrambler_sr;

    /* Statistics */
    int   total_symbols;
    float magnitude_sum;
    int   magnitude_count;
} p3_demod_t;

/* Detected signal segment */
typedef struct {
    p3_signal_type_t type;
    int   start_symbol;           /* First symbol index */
    int   length;                 /* Number of symbols */
    int   start_sample;           /* Sample offset of first symbol */
    int   end_sample;             /* Sample offset past last symbol */
    float avg_magnitude;          /* Average symbol magnitude in segment */
    float confidence;             /* 0.0 - 1.0 quality metric */

    /* For J frames */
    uint16_t j_trn16;            /* 16-bit repeating pattern */
    int   j_hypothesis;           /* Best-fit hypothesis (0-7) */
    int   j_table_bits;           /* 4, 16, or 0 if no Table 18 match */
    int   j_table_phase;          /* Best Table 18 phase offset (0-15) */
    int   j_table_transform;      /* 0=none,1=invert,2=swap,3=swap+invert */
    int   j_table_match_pct;      /* Best periodic Table 18 match percentage */
    int   j_periodic_match_pct;   /* Best learned 16-bit periodic match percentage */
    int   jprime_match_pct;       /* Best Table 19 one-block match percentage */

    /* For TRN */
    int   trn_errors;             /* Number of scrambler-recurrence errors */
    int   trn_recurrence_checks;  /* Scrambler-recurrence comparisons */
    int   trn_recurrence_match_pct; /* Scrambler-recurrence match percentage */
    int   trn_descrambled_errors; /* Descrambled zero bits (TRN expects ones) */
    int   trn_descrambled_bits;   /* Descrambled bits included in BER */
    int   trn_descrambled_ber_pct; /* Rounded descrambled bit-error percentage */

    /* For Ru/uR */
    bool  ru_positive_first;      /* true = +LU first (Ru), false = -LU (uR) */

    /* For PP */
    int   pp_phase;               /* Best 48-symbol phase offset (0..47) */
    int   pp_blocks;              /* Number of 48-symbol blocks matched */
} p3_segment_t;

/* Batch demodulation result */
typedef struct {
    p3_symbol_t *symbols;
    int          symbol_count;
    int          symbol_capacity;

    p3_segment_t *segments;
    int           segment_count;
    int           segment_capacity;

    /* Overall quality */
    float carrier_freq_estimate;
    float baud_rate_estimate;
    float snr_estimate_db;
    bool  locked;
} p3_result_t;

/* Spec-timed Phase 4 transition quality. The transmitting call modem sends
 * J -> one J' block -> TRN; the transmitting answer modem sends
 * S(128T) -> S-bar(16T) -> TRN. */
/* Live Ja acquisition policy.  A clean short Table-18 match is accepted
 * immediately; a distorted candidate must instead prove that it is a
 * sustained periodic J sequence rather than a chance TRN match. */
bool p3_is_adaptive_ja_candidate(const p3_segment_t *segment, int baud_rate);

typedef struct {
    bool  found;
    bool  source_calling_party;
    int   start_symbol;
    int   start_sample;
    int   trn_start_symbol;
    int   trn_start_sample;
    int   s_match_pct;
    int   sbar_match_pct;
    int   jprime_match_pct;
    int   trn_recurrence_match_pct;
    int   trn_descrambled_ber_pct;
} p3_phase4_timing_quality_t;

/* Multi-hypothesis scan result */
typedef struct {
    int   baud_code;
    int   carrier_sel;
    float carrier_hz;
    float baud_rate;
    int   symbol_count;
    int   segment_count;
    float score;                  /* Overall quality score */
    bool  has_s;
    bool  has_trn;
    bool  has_j;
    bool  has_ru;
} p3_hypothesis_t;

/*
 * Get baud rate parameters for a given code (0-5).
 * Returns false if code is out of range.
 */
bool p3_get_baud_params(int baud_code, p3_baud_params_t *out);

/*
 * Initialize demodulator for a known carrier and baud rate.
 */
void p3_demod_init(p3_demod_t *d, int baud_code, int carrier_sel, int sample_rate);

/*
 * Process a block of PCM samples. Appends demodulated symbols to result.
 * Returns number of new symbols produced.
 */
int p3_demod_process(p3_demod_t *d,
                     const int16_t *samples,
                     int sample_count,
                     int sample_offset,
                     p3_result_t *result);

/*
 * Reset demodulator state (keep configuration, clear PLL/timing/descrambler).
 */
void p3_demod_reset(p3_demod_t *d);

/*
 * Allocate a result structure. Must be freed with p3_result_free().
 */
p3_result_t *p3_result_alloc(int max_symbols, int max_segments);

/*
 * Free a result structure.
 */
void p3_result_free(p3_result_t *r);

/*
 * Segment the demodulated symbols into signal regions.
 * Populates result->segments. Returns number of segments found.
 */
int p3_segment_symbols(p3_result_t *result);

/* Search around an accepted Phase 3 J segment for the role-specific,
 * symbol-exact Phase 4 transition defined by V.34 clauses 10.1.3 and 11.4. */
bool p3_find_phase4_timing(const p3_result_t *result,
                           int j_start_symbol,
                           int j_length,
                           int j_transform,
                           bool source_calling_party,
                           p3_phase4_timing_quality_t *out);

/* Find the answer-modem S(128T) -> S-bar(16T) -> TRN transition without
 * relying on the legacy J/S/TRN segmenter. */
bool p3_find_answer_phase4_timing(const p3_result_t *result,
                                  p3_phase4_timing_quality_t *out);

/* Joint stereo timing: pair the call modem's J'->TRN boundary with the
 * answer modem's S->S-bar->TRN boundary in the standard 0..500 ms response
 * window.  This disambiguates Phase 4 from the identical S/S-bar waveform
 * used at the beginning of Phase 3. */
bool p3_find_stereo_phase4_timing(const p3_result_t *call_source,
                                  const p3_result_t *answer_source,
                                  int min_start_sample,
                                  p3_phase4_timing_quality_t *call_out,
                                  p3_phase4_timing_quality_t *answer_out,
                                  int *score_out);

/*
 * Run demodulation on a sample range with known parameters.
 * Convenience function: init + process + segment.
 */
p3_result_t *p3_demod_run(const int16_t *samples,
                          int sample_count,
                          int sample_offset,
                          int baud_code,
                          int carrier_sel,
                          int sample_rate);

/* Two-pass variant that trains the equalizer on the known V.34 PP sequence
 * before classifying J (SPRA159 §3.2.3 fast-equalizer principle).
 *
 * Pass 1: blind CMA, segment to find PP.
 * Pass 2: if PP was found, re-run with PP-directed LMS training enabled at
 * the PP start sample, then segment to find J with a converged equalizer.
 *
 * This is what real V.34 chipsets do — they don't rely on blind CMA for
 * initial training.  On noisy signals (e.g. USR Courier) blind CMA may not
 * converge in a short window, but PP-directed LMS converges in one shot
 * because it uses the known 48-symbol CAZAC reference sequence. */
p3_result_t *p3_demod_run_pp_trained(const int16_t *samples,
                                     int sample_count,
                                     int sample_offset,
                                     int baud_code,
                                     int carrier_sel,
                                     int sample_rate);

/* As above, but choose the initial timing-phase trial by the normative
 * answer-modem Phase-4 S/S-bar signature before falling back to the generic
 * segment score. */
p3_result_t *p3_demod_run_answer_phase4(const int16_t *samples,
                                        int sample_count,
                                        int sample_offset,
                                        int baud_code,
                                        int carrier_sel,
                                        int sample_rate);

/* Select the timing-phase trial by the call modem's J'->TRN evidence. */
p3_result_t *p3_demod_run_call_phase4(const int16_t *samples,
                                      int sample_count,
                                      int sample_offset,
                                      int baud_code,
                                      int carrier_sel,
                                      int sample_rate);

/* Role-specific Phase-4 trial selection with an explicit timing-phase count. */
p3_result_t *p3_demod_run_phase4_trials(const int16_t *samples,
                                        int sample_count,
                                        int sample_offset,
                                        int baud_code,
                                        int carrier_sel,
                                        int sample_rate,
                                        bool source_calling_party,
                                        int timing_trials);

/* Run the Phase-4-selected front end through data while freezing the CMA
 * equalizer at an absolute sample boundary.  This preserves the trained
 * channel inverse when the signal changes from constant-modulus TRN/MP to
 * the multi-ring data constellation. */
p3_result_t *p3_demod_run_phase4_data(const int16_t *samples,
                                      int sample_count,
                                      int sample_offset,
                                      int baud_code,
                                      int carrier_sel,
                                      int sample_rate,
                                      bool source_calling_party,
                                      int timing_trials,
                                      int cma_freeze_after_sample);

/* Run one exact member of the timing-phase sweep.  Protocol-aware callers
 * use this to retain every trial until a CRC-valid frame selects the winner,
 * instead of letting the training segment score discard the useful phase. */
p3_result_t *p3_demod_run_phase4_data_at_timing(
                                      const int16_t *samples,
                                      int sample_count,
                                      int sample_offset,
                                      int baud_code,
                                      int carrier_sel,
                                      int sample_rate,
                                      bool source_calling_party,
                                      int timing_index,
                                      int timing_count,
                                      int cma_freeze_after_sample);

/* Return the matched-filter complex stream at T/2 for supervised,
 * fractionally-spaced equalizer training. */
p3_result_t *p3_demod_run_half_baud_at_timing(
                                      const int16_t *samples,
                                      int sample_count,
                                      int sample_offset,
                                      int baud_code,
                                      int carrier_sel,
                                      int sample_rate,
                                      bool source_calling_party,
                                      int timing_index,
                                      int timing_count,
                                      int cma_freeze_after_sample);

/*
 * Try all baud/carrier combinations and return the best match.
 * Useful when INFO frames are not available.
 * hypotheses[] must have room for P3_BAUD_COUNT * 2 entries.
 * Returns the number of hypotheses tried.
 */
int p3_scan_all_hypotheses(const int16_t *samples,
                           int sample_count,
                           int sample_offset,
                           int sample_rate,
                           p3_hypothesis_t *hypotheses,
                           int max_hypotheses);

/*
 * Descramble a sequence of bits using x^23 + x^5 + 1.
 * sr is the shift register state (updated in place).
 * Returns number of bits that differ from all-ones (error count).
 */
int p3_descramble_bits(const uint8_t *scrambled, int count,
                       uint8_t *descrambled, uint32_t *sr);

/*
 * Return a human-readable name for a signal type.
 */
const char *p3_signal_type_name(p3_signal_type_t type);

#endif /* P3_DEMOD_H */
