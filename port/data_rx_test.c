/* Drives port/data_rx.c with a ME_V90_UPSTREAM_SYM_DUMP capture and reports
 * the same quantities the engine logs, so the two can be compared.
 *
 * Reference: artifacts/reneg-eq/reneg-r1, whose engine run reports
 *   "decision error 0.1803 per symbol (mean symbol power 738.04) over 3200"
 * for the first window, and a sym-err series with median 0.168.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "data_rx.h"

int main(int argc, char *argv[])
{
    FILE *f;
    char line[256];
    data_rx_t rx;
    long n = 0;
    long window = (argc > 2) ? atol(argv[2]) : 3200;
    long w_n = 0;
    float ema_min = 1e9f, ema_max = 0.0f;
    float *emas = NULL;
    long emas_n = 0, emas_cap = 0;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <sym-dump> [window]\n", argv[0]);
        return 2;
    }
    if ((f = fopen(argv[1], "r")) == NULL) {
        perror(argv[1]);
        return 2;
    }

    /* reneg-r1: B1 reports conjugate=0 and a 0.0008 deg/symbol residual, which
       the engine's own carrier loop then tracks out.  Open loop here. */
    data_rx_init(&rx, 0.0f, 0, 0);

    while (fgets(line, sizeof(line), f)) {
        int idx;
        float re, im;

        if (sscanf(line, "%d %f %f", &idx, &re, &im) != 3)
            continue;
        data_rx_put(&rx, re, im, NULL, NULL);
        n++;
        if (++w_n == window)
            w_n = 0;
        if (n == 3200) {
            printf("first window : decision error %.4f  mean symbol power %.2f\n",
                   rx.err_sum/3200.0, rx.pow_sum/3200.0);
        }
        if (n > 512) {
            if (rx.err_ema < ema_min) ema_min = rx.err_ema;
            if (rx.err_ema > ema_max) ema_max = rx.err_ema;
            if (emas_n == emas_cap) {
                emas_cap = emas_cap ? emas_cap*2 : 4096;
                emas = realloc(emas, emas_cap*sizeof(float));
            }
            emas[emas_n++] = rx.err_ema;
        }
    }
    fclose(f);

    printf("symbols      : %ld\n", n);
    printf("whole capture: decision error %.4f  mean symbol power %.2f\n",
           rx.err_sum/(double) n, rx.pow_sum/(double) n);
    if (emas_n) {
        /* median of the 256-EMA, the engine's "sym err" */
        long i, j;
        for (i = 1; i < emas_n; i++) {   /* insertion sort is fine offline */
            float v = emas[i];
            for (j = i; j > 0 && emas[j-1] > v; j--) emas[j] = emas[j-1];
            emas[j] = v;
        }
        printf("sym err (256-EMA): min %.3f  median %.3f  max %.3f\n",
               emas[0], emas[emas_n/2], emas[emas_n-1]);
    }
    free(emas);
    return 0;
}
