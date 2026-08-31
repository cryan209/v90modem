/* NCO and RRC accuracy against the float front end.
 *
 * The mixer is the interesting one: the float path recomputes its phase from
 * the absolute output count in DOUBLE, so this checks the incremental uint32
 * accumulator against that over a full call's worth of samples -- 5.8M -- which
 * is where a naive float phase would have failed.
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "v34_fixed.h"

#define RRC 97
#define RATE   9600.0
#define FC     1828.571411

int main(void)
{
    v34_fx_nco_t nco;
    double worst_ph = 0.0, sum2 = 0.0;
    long n = 5800000, i;

    v34_fx_nco_init(&nco, -FC, RATE);
    for (i = 0; i < n; i++) {
        int32_t c, s;
        v34_fx_nco_step(&nco, &c, &s);
        if ((i % 97) == 0) {                       /* sample the error */
            double ang = -2.0*M_PI*FC*(double) i/RATE;
            double dc = c/1073741824.0 - cos(ang);
            double ds = s/1073741824.0 - sin(ang);
            double e = hypot(dc, ds);
            if (e > worst_ph) worst_ph = e;
            sum2 += e*e;
        }
    }
    printf("  NCO over %ld samples (a 600 s call):\n", n);
    printf("    worst |e^{j.phi} error|  %.3e\n", worst_ph);
    printf("    rms                      %.3e\n", sqrt(sum2/(n/97)));

    /* RRC: random band-limited input through both, compare */
    static float fc_[RRC], fh_re[RRC], fh_im[RRC];
    static int32_t xc[RRC];
    static v34_fx_complex_t xh[RRC];
    double num = 0.0, den = 0.0;
    unsigned r = 7;
    int pos = 0;

    for (i = 0; i < RRC; i++) {
        double t = (i - RRC/2)/4.0;
        double v = (fabs(t) < 1e-9) ? 1.0 : sin(M_PI*t)/(M_PI*t);
        v *= 0.54 - 0.46*cos(2.0*M_PI*i/(RRC - 1));
        fc_[i] = (float) (v*0.02);
        xc[i] = v34_fx_from_float(fc_[i], V34_FX_TAP_SHIFT);
    }
    for (long k = 0; k < 20000; k++) {
        r = r*1103515245u + 12345u;
        float sr = (float) ((int) ((r >> 16) & 8191) - 4096);
        r = r*1103515245u + 12345u;
        float si = (float) ((int) ((r >> 16) & 8191) - 4096);

        fh_re[pos] = sr; fh_im[pos] = si;
        xh[pos].re = v34_fx_from_float(sr, V34_FX_RING_SHIFT);
        xh[pos].im = v34_fx_from_float(si, V34_FX_RING_SHIFT);
        if (++pos >= RRC) pos = 0;

        double ar = 0.0, ai = 0.0;
        for (int t = 0; t < RRC; t++) {
            int p = pos - 1 - t; if (p < 0) p += RRC;
            ar += (double) fc_[t]*fh_re[p];
            ai += (double) fc_[t]*fh_im[p];
        }
        v34_fx_complex_t o = v34_fx_fir(xc, xh, pos, RRC);
        double xr = v34_fx_to_float(o.re, V34_FX_RING_SHIFT);
        double xi = v34_fx_to_float(o.im, V34_FX_RING_SHIFT);
        num += (xr-ar)*(xr-ar) + (xi-ai)*(xi-ai);
        den += ar*ar + ai*ai;
    }
    printf("  RRC 97 taps, 20000 samples:\n");
    printf("    error / signal           %.3e  (%.1f dB SNR)\n",
           sqrt(num/den), 10.0*log10(den/num));
    return 0;
}
