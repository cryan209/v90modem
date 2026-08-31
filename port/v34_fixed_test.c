/* Accuracy of the fixed-point FSE kernel against the float one, on the REAL
 * taps and ring values a live call produced.
 *
 * Input is a V90_T3_SYMBOL_PROBE dump.  The probe prints the receiver's own
 * output y for the symbol, so this compares three things: float recomputed
 * here (must match the receiver, proving the harness), the fixed-point kernel,
 * and the receiver's y.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v34_fixed.h"

#define MAX_TAPS 64

int main(int argc, char *argv[])
{
    FILE *f;
    char line[512];
    float tre[MAX_TAPS], tim[MAX_TAPS], rre[MAX_TAPS], rim[MAX_TAPS];
    int n = 0;
    float yre = 0.0f, yim = 0.0f;
    int have_y = 0;

    if (argc < 2) { fprintf(stderr, "usage: %s <T3PROBE dump>\n", argv[0]); return 2; }
    if ((f = fopen(argv[1], "r")) == NULL) { perror(argv[1]); return 2; }

    while (fgets(line, sizeof(line), f)) {
        float a, b, c, d;
        int tap;

        if (!have_y && strstr(line, "y=(") && !strstr(line, "tap=")) {
            char *p = strstr(line, "y=(");
            if (p && sscanf(p, "y=(%f,%f)", &a, &b) == 2) { yre = a; yim = b; have_y = 1; }
            continue;
        }
        if (strstr(line, "tap=") && n < MAX_TAPS) {
            char *p = strstr(line, "fse=(");
            char *q = strstr(line, "rawfrac=(");
            if (p && q
                && sscanf(p, "fse=(%f,%f)", &a, &b) == 2
                && sscanf(q, "rawfrac=(%f,%f)", &c, &d) == 2) {
                if (sscanf(strstr(line, "tap="), "tap=%d", &tap) == 1 && tap == n) {
                    tre[n] = a; tim[n] = b; rre[n] = c; rim[n] = d; n++;
                }
            }
        }
    }
    fclose(f);
    if (n == 0) { fprintf(stderr, "no taps parsed\n"); return 2; }

    /* float, exactly as v90_t3_emit_ready() does it */
    double fre = 0.0, fim = 0.0;
    for (int k = 0; k < n; k++) {
        fre += (double) tre[k]*rre[k] - (double) tim[k]*rim[k];
        fim += (double) tre[k]*rim[k] + (double) tim[k]*rre[k];
    }

    /* fixed */
    v34_fx_complex_t taps[MAX_TAPS], ring[MAX_TAPS], out;
    for (int k = 0; k < n; k++) {
        taps[k].re = v34_fx_from_float(tre[k], V34_FX_TAP_SHIFT);
        taps[k].im = v34_fx_from_float(tim[k], V34_FX_TAP_SHIFT);
        ring[k].re = v34_fx_from_float(rre[k], V34_FX_RING_SHIFT);
        ring[k].im = v34_fx_from_float(rim[k], V34_FX_RING_SHIFT);
    }
    out = v34_fx_fse(taps, ring, n);
    double xre = v34_fx_to_float(out.re, V34_FX_RING_SHIFT);
    double xim = v34_fx_to_float(out.im, V34_FX_RING_SHIFT);

    printf("taps            %d\n", n);
    if (have_y)
        printf("receiver  y     (%.6f, %.6f)\n", yre, yim);
    printf("float     recomp (%.6f, %.6f)\n", fre, fim);
    printf("fixed     Q1.30  (%.6f, %.6f)\n", xre, xim);

    double err = hypot(xre - fre, xim - fim);
    double mag = hypot(fre, fim);
    printf("fixed vs float  abs %.3e   rel %.3e  (%.1f dB SNR)\n",
           err, mag > 0 ? err/mag : 0.0, mag > 0 ? 20.0*log10(mag/err) : 0.0);
    if (have_y) {
        double e2 = hypot(fre - yre, fim - yim);
        printf("float vs receiver abs %.3e  (harness check: should be ~0)\n", e2);
    }
    /* The decision is to an odd-integer lattice of spacing 2, so what matters
       is the error against a half-spacing of 1.0. */
    printf("error as a fraction of the decision half-distance (1.0): %.3e\n", err);
    return 0;
}
