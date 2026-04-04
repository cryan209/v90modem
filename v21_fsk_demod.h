/*
 * v21_fsk_demod.h — Standalone V.21 FSK demodulator (no spandsp)
 *
 * Implements ITU-T V.21 300 baud FSK demodulation for both channels:
 *   CH1 (calling): mark=980 Hz, space=1180 Hz
 *   CH2 (answering): mark=1650 Hz, space=1850 Hz
 *
 * Uses Goertzel correlation for tone detection and a brute-force
 * phase/rate sweep for robust offline symbol recovery.
 */

#ifndef V21_FSK_DEMOD_H
#define V21_FSK_DEMOD_H

#include <stdbool.h>
#include <stdint.h>

/* V.21 channel definitions */
#define V21_CH1  0   /* Calling modem: 980/1180 Hz */
#define V21_CH2  1   /* Answering modem: 1650/1850 Hz */

/* Bit callback: sample_offset is the sample index where the bit was detected */
typedef void (*v21_bit_cb_t)(void *ctx, int bit, int sample_offset);

/* ------------------------------------------------------------------ */
/* Block-mode demodulator (offline, brute-force phase sweep)           */
/* ------------------------------------------------------------------ */

/*
 * Demodulate a block of V.21 FSK samples and deliver bits via callback.
 * This is the primary interface for offline decode — it sweeps baud rates
 * and phase offsets to find the best symbol alignment.
 *
 * Returns total number of bits delivered.
 */
int v21_fsk_demod_block(const int16_t *samples,
                        int total_samples,
                        int sample_rate,
                        int channel,          /* V21_CH1 or V21_CH2 */
                        bool invert,          /* swap mark/space sense */
                        v21_bit_cb_t put_bit,
                        void *put_bit_ctx);

/*
 * Single-symbol Goertzel decision at a given sample offset.
 * Returns 1 for mark, 0 for space, -1 on error.
 * Optionally returns confidence (|mark_ratio - space_ratio|).
 */
int v21_fsk_symbol_decision(const int16_t *samples,
                            int total_samples,
                            int sample_rate,
                            int channel,
                            int offset,
                            int symbol_len,
                            double *confidence_out);

/* ------------------------------------------------------------------ */
/* Streaming demodulator (sample-at-a-time with PLL timing recovery)   */
/* ------------------------------------------------------------------ */

typedef struct {
    int channel;
    int sample_rate;
    double mark_hz;
    double space_hz;
    bool invert;

    /* Goertzel accumulators for mark and space */
    double mark_re, mark_im;
    double space_re, space_im;
    double mark_cos, mark_sin;
    double space_cos, space_sin;
    double mark_osc_re, mark_osc_im;
    double space_osc_re, space_osc_im;

    /* Symbol timing */
    double symbol_samples;        /* nominal samples per symbol (~26.67) */
    double phase_accum;           /* fractional sample counter */
    int window_count;             /* samples accumulated in current symbol */
    double window_energy;

    /* PLL */
    double prev_decision_val;     /* mark-space ratio for timing recovery */

    /* Output */
    v21_bit_cb_t put_bit;
    void *put_bit_ctx;
    int total_samples_fed;
} v21_fsk_stream_t;

void v21_fsk_stream_init(v21_fsk_stream_t *d,
                          int channel,
                          int sample_rate,
                          bool invert,
                          v21_bit_cb_t put_bit,
                          void *put_bit_ctx);

void v21_fsk_stream_rx(v21_fsk_stream_t *d,
                        const int16_t *samples,
                        int count);

void v21_fsk_stream_reset(v21_fsk_stream_t *d);

/* ------------------------------------------------------------------ */
/* Utility: compute Goertzel tone energy for int16 samples              */
/* ------------------------------------------------------------------ */

typedef struct {
    double re;
    double im;
    double energy;   /* re*re + im*im */
} goertzel_result_t;

goertzel_result_t goertzel_analyze(const int16_t *samples,
                                   int len,
                                   int sample_rate,
                                   double freq_hz);

#endif /* V21_FSK_DEMOD_H */
