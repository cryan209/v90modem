/*
 * v92_tone_a.c — see v92_tone_a.h.
 */
#include "v92_tone_a.h"

#include <math.h>
#include <string.h>

#define BLOCK 80                          /* 10 ms */
#define FRACTION 0.70
#define MIN_RMS 90.0                      /* about -45 dBm0 */

void v92_tone_a_init(v92_tone_a_t *t)
{
    memset(t, 0, sizeof(*t));
}

bool v92_tone_a_put(v92_tone_a_t *t, const int16_t *x, int n)
{
    /* 2cos(2*pi*2400/8000) */
    const double coeff = -0.61803398874989;
    bool fire = false;

    for (int i = 0; i < n; i++) {
        double v = x[i];
        double g0 = v + coeff * t->g1 - t->g2;

        t->g2 = t->g1;
        t->g1 = g0;
        t->energy += v * v;
        if (++t->n == BLOCK) {
            double p = t->g1 * t->g1 + t->g2 * t->g2 - coeff * t->g1 * t->g2;
            double frac = t->energy > 0 ? 2.0 * p / (BLOCK * t->energy) : 0;
            bool tone = sqrt(t->energy / BLOCK) > MIN_RMS && frac > FRACTION;

            t->run_ms = tone ? t->run_ms + 10 : 0;
            /* "more than 50 ms" */
            if (t->run_ms > 50 && !t->detected) {
                t->detected = true;
                fire = true;
            }
            t->g1 = t->g2 = t->energy = 0;
            t->n = 0;
        }
    }
    return fire;
}
