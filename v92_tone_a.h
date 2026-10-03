/*
 * v92_tone_a.h — Tone A (2400 Hz) detection on a V.92 PCM upstream, for the
 * digital modem's retrain response (V.92 9.7.1.2 as replaced by Cor.1 item 1:
 * "After detecting Tone A for more than 50 ms ...").
 *
 * On a V.92 call the upstream is PCM (TRN1u, TRN2u, data), not a V.34 signal,
 * so the V.34 receiver's retrain watcher is not looking at it.  This measures
 * the fraction of each 10 ms block's energy in the 2400 Hz Goertzel bin: a
 * tone puts ~all of it there, a wideband PCM upstream a few percent.
 */
#ifndef V92_TONE_A_H
#define V92_TONE_A_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    double g1, g2, energy;
    int n;
    int run_ms;                       /* consecutive 10 ms blocks of Tone A */
    bool detected;
} v92_tone_a_t;

void v92_tone_a_init(v92_tone_a_t *t);
/* Linear samples at 8 kHz.  Returns true once, when Tone A has lasted more
 * than 50 ms. */
bool v92_tone_a_put(v92_tone_a_t *t, const int16_t *x, int n);

#endif
