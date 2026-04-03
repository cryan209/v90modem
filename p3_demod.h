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
    P3_SIGNAL_J_PRIME,      /* J' (12 scrambled zeros) */
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

    /* For TRN */
    int   trn_errors;             /* Number of descrambled bits != 1 */

    /* For Ru/uR */
    bool  ru_positive_first;      /* true = +LU first (Ru), false = -LU (uR) */
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
