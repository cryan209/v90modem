/*
 * near_echo_test.c — near_echo.c against a known echo under double talk.
 *
 * The conditions are the Apple modem's (near_echo.h): a long echo at a bulk
 * delay the canceller is not told, an echo-only stretch (V.34 Phase 2's
 * probing), then a far-end signal 17 dB above the echo that never stops.  It
 * must find the delay from the quiet stretch, remove most of the echo during
 * the double talk, and not be pulled off by it.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "near_echo.h"

static uint32_t rng = 1;
static double urand(void) { rng = rng*1664525u + 1013904223u; return (rng >> 8)/16777216.0 - 0.5; }

enum { FS = 9600, N = 12*FS, DELAY = 1140, ELEN = 300, QUIET = 2*FS };
static int16_t rx_inline[N];

static int run(int threaded)
{
    static int16_t tx[N], rx[N], echo[N];
    double h[ELEN], lp = 0.0, far = 0.0;
    near_echo_t *ec = near_echo_init(700, 768, 4800);

    rng = 1;
    if (ec != NULL  &&  threaded)
        near_echo_set_threaded(ec, 1);
    double pe = 0.0, pr = 0.0;
    int fail = 0;

    if (ec == NULL) { printf("FAIL init\n"); return 1; }
    /* A ringing, band-limited echo response returning at about -27 dB, as
     * measured (rms 140 against a transmit of 3200). */
    for (int j = 0; j < ELEN; j++)
        h[j] = 0.0103*exp(-j/60.0)*sin(2.0*M_PI*j*0.21 + 0.4);
    for (int i = 0; i < N; i++) {
        lp = 0.6*lp + 0.4*urand();          /* coloured transmit, rms ~3000 */
        tx[i] = (int16_t) lrint(lp*30000.0);
    }
    for (int i = 0; i < N; i++) {
        double e = 0.0;
        for (int j = 0; j < ELEN; j++)
            if (i - DELAY - j >= 0)
                e += h[j]*tx[i - DELAY - j];
        far = 0.3*far + urand();            /* far end, ~17 dB above the echo */
        echo[i] = (int16_t) lrint(e);
        rx[i] = (int16_t) lrint(e + (i < QUIET ? far*4.0 : far*2400.0));
    }
    for (int i = 0; i < N; i += 512) {
        int n = (N - i < 512) ? N - i : 512;
        near_echo_tx(ec, tx + i, n);
        near_echo_rx(ec, rx + i, n);
    }
    {
        /* Over the last 4 s, rx holds far + (echo - estimate): rebuild the
         * far end from the same generator and what is left is residual echo. */
        double fr = 0.0;
        rng = 1;
        for (int i = 0; i < N; i++) urand();   /* consume tx draws */
        for (int i = 0; i < N; i++) {
            fr = 0.3*fr + urand();
            if (i >= N - 4*FS) {
                double far_i = (double) lrint(echo[i] + fr*2400.0) - echo[i];
                /* (only the double-talk stretch is scored) */
                double res = rx[i] - far_i;
                pe += (double) echo[i]*echo[i];
                pr += res*res;
            }
        }
    }
    {
        double erle = 10.0*log10(pe/(pr > 1e-9 ? pr : 1e-9));
        printf("near_echo (%s): delay %d in [700, 1468), %d fits on %ld blocks, echo rms %.0f, ERLE %.1f dB\n",
               threaded ? "worker" : "inline",
               DELAY, near_echo_solved(ec), near_echo_blocks_used(ec),
               sqrt(pe/(4*FS)), erle);
        if (!near_echo_active(ec)) {
            printf("FAIL never fitted\n");
            fail = 1;
        }
        if (erle < 20.0) {
            printf("FAIL ERLE %.1f dB < 20\n", erle);
            fail = 1;
        }
    }
    near_echo_free(ec);
    if (!threaded) {
        memcpy(rx_inline, rx, sizeof(rx));
    } else if (memcmp(rx_inline, rx, sizeof(rx)) != 0) {
        /* A live call and its --rx-replay must cancel identically. */
        printf("FAIL worker output differs from inline\n");
        fail = 1;
    }
    return fail;
}

int main(void)
{
    int fail = run(0);

    fail |= run(1);
    printf("%s\n", fail ? "FAIL" : "PASS");
    return fail;
}
