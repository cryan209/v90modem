/* Does the fixed-point NLMS converge like the float one, or does it stall?
 *
 * A known channel, random 4-point symbols, both loops driven identically from
 * the same data.  Stall shows up as the fixed taps freezing while the float
 * ones keep improving -- so the figure to read is the residual at the END, and
 * how many taps moved at all in the last block.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "v34_fixed.h"

#define NTAP  21
#define NSYM  200000

static unsigned rng = 12345;
static int rnd4(void) { rng = rng*1103515245u + 12345u; return (rng >> 16) & 3; }

int main(void)
{
    /* a channel to invert: main tap plus a couple of echoes */
    float h_re[5] = {0.05f, 0.9f, -0.2f, 0.08f, -0.03f};
    float h_im[5] = {0.01f, 0.0f,  0.05f, -0.02f, 0.01f};
    float xr[NTAP], xi[NTAP];
    float fre[NTAP], fim[NTAP];            /* float taps  */
    v34_fx_complex_t taps[NTAP], ring[NTAP];
    v34_fx_acc_t acc[NTAP];
    double f_res = 0.0, x_res = 0.0;
    int f_n = 0, moved = 0;
    int i, k;

    for (k = 0; k < NTAP; k++) { fre[k] = fim[k] = 0.0f; xr[k] = xi[k] = 0.0f;
                                 taps[k].re = taps[k].im = 0; }
    fre[NTAP/2] = 1.0f;                     /* centre spike, both start here */
    taps[NTAP/2].re = v34_fx_from_float(1.0f, V34_FX_TAP_SHIFT);
    v34_fx_lms_init(acc, taps, NTAP);

    for (i = 0; i < NSYM; i++) {
        /* transmit a 4-point symbol, scaled to the ring's real-world range */
        int b = rnd4();
        float sre = (b & 1) ? 1.0f : -1.0f;
        float sim = (b & 2) ? 1.0f : -1.0f;
        float nre = 0.0f, nim = 0.0f;

        for (k = 4; k > 0; k--) { xr[k] = xr[k-1]; xi[k] = xi[k-1]; }
        xr[0] = sre*1000.0f; xi[0] = sim*1000.0f;
        for (k = 0; k < 5; k++) { nre += h_re[k]*xr[k] - h_im[k]*xi[k];
                                  nim += h_re[k]*xi[k] + h_im[k]*xr[k]; }
        for (k = NTAP-1; k > 0; k--) { xr[k] = xr[k-1]; xi[k] = xi[k-1]; }
        xr[0] = nre; xi[0] = nim;

        float yr = 0.0f, yi = 0.0f, energy = 1e-6f;
        for (k = 0; k < NTAP; k++) {
            yr += fre[k]*xr[k] - fim[k]*xi[k];
            yi += fre[k]*xi[k] + fim[k]*xr[k];
            energy += xr[k]*xr[k] + xi[k]*xi[k];
        }
        /* decision to the odd-integer lattice, as the receiver does */
        float tr = 2.0f*floorf(yr/2.0f) + 1.0f, ti = 2.0f*floorf(yi/2.0f) + 1.0f;
        float er = tr - yr, ei = ti - yi;
        float mu = 0.02f/energy;
        for (k = 0; k < NTAP; k++) {
            fre[k] += mu*(er*xr[k] + ei*xi[k]);
            fim[k] += mu*(ei*xr[k] - er*xi[k]);
        }

        for (k = 0; k < NTAP; k++) {
            ring[k].re = v34_fx_from_float(xr[k], V34_FX_RING_SHIFT);
            ring[k].im = v34_fx_from_float(xi[k], V34_FX_RING_SHIFT);
        }
        v34_fx_lms_taps(taps, acc, NTAP);
        v34_fx_complex_t z = v34_fx_fse(taps, ring, NTAP);
        float zr = v34_fx_to_float(z.re, V34_FX_RING_SHIFT);
        float zi = v34_fx_to_float(z.im, V34_FX_RING_SHIFT);
        float ztr = 2.0f*floorf(zr/2.0f) + 1.0f, zti = 2.0f*floorf(zi/2.0f) + 1.0f;
        int64_t e_fx_re = v34_fx_from_float(ztr - zr, V34_FX_RING_SHIFT);
        int64_t e_fx_im = v34_fx_from_float(zti - zi, V34_FX_RING_SHIFT);
        v34_fx_acc_t before = acc[0];
        v34_fx_lms_update(acc, ring, NTAP,
                          (int32_t) e_fx_re, (int32_t) e_fx_im,
                          v34_fx_from_float(0.02f, V34_FX_TAP_SHIFT),
                          (int64_t) energy);
        if (i >= NSYM - 1000 && acc[0].re != before.re) moved++;

        if (i >= NSYM - 20000) {
            f_res += (tr-yr)*(tr-yr) + (ti-yi)*(ti-yi);
            x_res += (ztr-zr)*(ztr-zr) + (zti-zi)*(zti-zi);
            f_n++;
        }
    }
    printf("  symbols                    %d\n", NSYM);
    printf("  float  residual (last 20k) %.4e per symbol\n", f_res/f_n);
    printf("  fixed  residual (last 20k) %.4e per symbol\n", x_res/f_n);
    printf("  ratio fixed/float          %.3f\n", (f_res > 0) ? x_res/f_res : 0.0);
    {   /* do the two converge to the SAME filter? */
        double d = 0.0, m = 0.0;
        for (k = 0; k < NTAP; k++) {
            double a = v34_fx_to_float(taps[k].re, V34_FX_TAP_SHIFT) - fre[k];
            double b = v34_fx_to_float(taps[k].im, V34_FX_TAP_SHIFT) - fim[k];
            d += a*a + b*b;
            m += (double) fre[k]*fre[k] + (double) fim[k]*fim[k];
        }
        printf("  tap-vector distance        %.4e  (%.1f dB below the float taps)\n",
               sqrt(d), (d > 0) ? 10.0*log10(m/d) : 0.0);
    }
    printf("  tap[0] updates in last 1000 symbols: %d  %s\n", moved,
           moved == 0 ? "<-- STALLED" : "(adapting)");
    return 0;
}
