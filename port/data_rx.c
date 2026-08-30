/* V.90 upstream DATA: the feed-forward decode core, standalone.
 *
 * The second file of the ESP32 port.  v34rx.c's V34_RX_STAGE_DATA case is
 * 800 lines (11524-12323); this is the part of it that every symbol must
 * run.  What is left behind is measurement, not decoding:
 *
 *   - a 33-trial gain sweep, PER SYMBOL (V34_V90_T3_GAIN_TRIALS), answering
 *     "if some other scale put the symbols on the grid, the fault would be a
 *     scaling one".  At 3200 baud that is ~1 MFLOP/s -- about half the entire
 *     receiver's arithmetic budget -- spent on a question that is asked once
 *     during bring-up and never again.
 *   - the B1-era distance report, the receive-SNR/rate projection, the
 *     divergence guard (V34_V90_T3_DIVERGED_POWER) and the per-frame RMS and
 *     Q9.7 frame dumps.
 *
 * Two things in that block are NOT diagnostics and are kept: the 256-symbol
 * and 16-symbol error means.  They gate the timing loop, the DD-LMS and the
 * carrier loop, and dropping them silently un-gates all three -- which is the
 * failure the plain-V.34 data mode needed the health gate to prevent.
 *
 * V.34 9.x puts every constellation point on odd integers, so the decision is
 * available here without the shell decoder.
 */

#include <math.h>
#include "data_rx.h"

/* Two thirds is the mean-square distance to the grid for symbols bearing no
 * relation to the lattice (uniform over a cell of spacing 2).  A receiver
 * that is decoding reads about 0.10.  Anything near 0.667 is WHITE, not
 * merely noisy -- do not read it as "poor but working". */
#define DATA_RX_WHITE 0.6667f

void data_rx_init(data_rx_t *s, float derot_rate_rad, int rotation, int conjugate)
{
    s->derot = 0.0f;
    s->derot_rate = derot_rate_rad;
    s->rotation = rotation & 3;
    s->conjugate = conjugate ? 1 : 0;
    s->err_ema = 0.0f;
    s->err_fast = 0.0f;
    s->baseline = 0.0f;
    s->count = 0;
    s->err_sum = 0.0f;
    s->pow_sum = 0.0f;
}

/* One equalized symbol in; the sliced lattice point and this symbol's squared
 * distance out.  Returns the distance so the caller can gate on it. */
float data_rx_put(data_rx_t *s, float re, float im, float *out_re, float *out_im)
{
    float c, sn, dr, di, tr, ti, d_re, d_im, d;

    /* Carry the B1-measured residual carrier forward.  Without this the
       derotator is a constant and the offset it cannot see accumulates
       without limit. */
    s->derot += s->derot_rate;
    c = cosf(s->derot);
    sn = sinf(s->derot);
    dr = re*c + im*sn;
    di = im*c - re*sn;
    if (s->conjugate)
        di = -di;

    switch (s->rotation) {
    case 1:  tr = -di; ti = dr;  break;
    case 2:  tr = -dr; ti = -di; break;
    case 3:  tr = di;  ti = -dr; break;
    default: tr = dr;  ti = di;  break;
    }
    dr = tr;
    di = ti;

    /* Slice to the odd-integer lattice. */
    tr = 2.0f*floorf(dr/2.0f) + 1.0f;
    ti = 2.0f*floorf(di/2.0f) + 1.0f;
    d_re = dr - tr;
    d_im = di - ti;
    d = d_re*d_re + d_im*d_im;

    /* The two gates. 256 for the timing loop, 16 for the fast adaptive
       elements (V34_V90_T3_ERR_FAST_SHIFT). */
    s->err_ema  += (d - s->err_ema)/256.0f;
    s->err_fast += (d - s->err_fast)/16.0f;

    s->err_sum += d;
    s->pow_sum += dr*dr + di*di;
    s->count++;

    if (out_re) *out_re = tr;
    if (out_im) *out_im = ti;
    return d;
}

/* Is the receiver healthy enough for the adaptive elements to steer on its
 * decisions?  The absolute test is against WHITE; the relative one is against
 * what this call itself settled at, because a call decoding at 0.23 is
 * healthy and one that settled at 0.10 and drifted to 0.23 is not. */
int data_rx_healthy(const data_rx_t *s, float slack)
{
    if (s->err_fast >= DATA_RX_WHITE*0.5f)
        return 0;
    if (s->baseline > 0.0f && s->err_fast > s->baseline*slack)
        return 0;
    return 1;
}

void data_rx_set_baseline(data_rx_t *s)
{
    s->baseline = s->err_ema;
}
