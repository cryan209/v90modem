/* Out-of-line parts of the fixed-point kernels: the NCO's table, which is too
 * big to be an inline and is built once per restart. */
#include <math.h>
#include <stdint.h>

#include "v34_fixed.h"

void v34_fx_nco_init(v34_fx_nco_t *n, double freq, double rate)
{
    int i;

    /* Negative frequencies are the normal case here (the mixer runs at
       -2*pi*fc), and a uint32 phase wraps for free, so just take the
       increment modulo 2^32. */
    n->phase = 0;
    n->inc = (uint32_t) (int64_t) llround(freq/rate*4294967296.0);
    for (i = 0;  i <= V34_FX_SIN_SIZE;  i++)
        n->sine[i] = (int32_t) llround(sin(2.0*M_PI*i/(4.0*V34_FX_SIN_SIZE))*1073741824.0);
    /*endfor*/
}

/* Quarter-wave lookup with linear interpolation between entries. */
static int32_t v34_fx_sin_q30(const v34_fx_nco_t *n, uint32_t phase)
{
    uint32_t quad = phase >> 30;
    uint32_t idx = (phase >> (30 - V34_FX_SIN_BITS)) & (V34_FX_SIN_SIZE - 1);
    uint32_t frac = (phase << (V34_FX_SIN_BITS + 2)) >> 16;     /* Q16 */
    int32_t a;
    int32_t b;
    int32_t v;

    if (quad & 1)
    {
        a = n->sine[V34_FX_SIN_SIZE - idx];
        b = n->sine[V34_FX_SIN_SIZE - idx - 1];
    }
    else
    {
        a = n->sine[idx];
        b = n->sine[idx + 1];
    }
    /*endif*/
    v = a + (int32_t) (((int64_t) frac*(b - a)) >> 16);
    return (quad & 2) ? -v : v;
}

void v34_fx_nco_step(v34_fx_nco_t *n, int32_t *cos_q30, int32_t *sin_q30)
{
    *sin_q30 = v34_fx_sin_q30(n, n->phase);
    *cos_q30 = v34_fx_sin_q30(n, n->phase + 0x40000000u);
    n->phase += n->inc;
}
