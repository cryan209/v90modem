/*
 * v90_analogue_pr4_test.c — v90_analogue_pr4.c on a synthetic 1 - D^2 line.
 *
 * T/2 samples carry s[k] - s[k-2] plus noise, as the Apple modem's loop does
 * once equalised (v90_analogue_pr4.h).  Train on known symbols, then detect a
 * multilevel stream whose candidate set changes with the frame slot, and check
 * every codeword comes out, in order and right.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "v90_analogue_pr4.h"

enum { N = 40000, TRAIN = 5000, START = 8000 };
static double sym[N];
static int    lev[N];          /* index into the slot's level table, signed */
static const double table[6][4] = {
    {0.05, 0.20, 0.45, 0.90}, {0.08, 0.25, 0.50, 1.00}, {0.05, 0.18, 0.40, 0.80},
    {0.10, 0.30, 0.60, 1.10}, {0.06, 0.22, 0.47, 0.95}, {0.07, 0.24, 0.52, 1.05},
};
static uint32_t rng = 7;
static double ur(void) { rng = rng*1664525u + 1013904223u; return (rng >> 8)/16777216.0; }
static double gn(void) { double u = ur() + 1e-12, v = ur(); return sqrt(-2.0*log(u))*cos(2.0*M_PI*v); }

static uint8_t code(int slot, int signed_idx)
{
    /* Any unique byte per (slot, level) will do for the test. */
    return (uint8_t) (slot*16 + (signed_idx < 0 ? 8 : 0) + abs(signed_idx) - 1);
}

static int set_fn(void *user, int64_t index, double *levels, uint8_t *cws, int max)
{
    int slot = (int) (index % 6), n = 0;

    (void) user;
    for (int i = 0; i < 4  &&  n + 2 <= max; i++) {
        levels[n] = table[slot][i];   cws[n++] = code(slot, i + 1);
        levels[n] = -table[slot][i];  cws[n++] = code(slot, -(i + 1));
    }
    return n;
}

int main(void)
{
    v90a_pr4_t *pr = v90a_pr4_init(16, 6, 24);
    long errors = 0, got = 0;
    int fail = 0;

    if (pr == NULL) { printf("FAIL init\n"); return 1; }
    for (int k = 0; k < N; k++) {
        int i = (int) (ur()*4.0), sgn = ur() < 0.5 ? -1 : 1;

        if (k < START) {                 /* training-like: two far-apart points */
            lev[k] = 0;
            sym[k] = sgn*1.0;
        } else {
            lev[k] = sgn*(i + 1);
            sym[k] = sgn*table[k % 6][i];
        }
    }
    v90a_pr4_train(pr, TRAIN);
    for (int k = 0; k < N; k++) {
        double z = sym[k] - (k >= 2 ? sym[k - 2] : 0.0);

        /* Two T/2 samples per symbol; the second is the symbol's own. */
        v90a_pr4_sample(pr, (int16_t) lrint(1000.0*(z + 0.004*gn())));
        v90a_pr4_sample(pr, (int16_t) lrint(1000.0*(z + 0.004*gn())));
        if (k == START  &&  !v90a_pr4_start(pr, START, set_fn, NULL)) {
            printf("FAIL start (trained %d)\n", v90a_pr4_trained(pr));
            return 1;
        }
        v90a_pr4_symbol(pr, sym[k]);
        {
            uint8_t out[64];
            int n = v90a_pr4_take(pr, out, 64);

            for (int j = 0; j < n; j++, got++) {
                int64_t idx = START + got;

                if (out[j] != code((int) (idx % 6), lev[idx]))
                    errors++;
            }
        }
    }
    printf("v90a_pr4: delay %d, training fit %.1f dB, %ld codewords out of %d, %ld wrong\n",
           v90a_pr4_delay(pr), v90a_pr4_train_snr_db(pr), got, N - START, errors);
    if (got < N - START - 2*24 - 16) { printf("FAIL too few codewords out\n"); fail = 1; }
    if (errors != 0) { printf("FAIL decision errors\n"); fail = 1; }
    v90a_pr4_free(pr);
    printf("%s\n", fail ? "FAIL" : "PASS");
    return fail;
}
