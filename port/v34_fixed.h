/* Fixed-point kernels for the V.34 receiver, selectable against the float ones.
 *
 * WHY THIS EXISTS, AND WHEN NOT TO USE IT
 *
 * On an ESP32-S3 you do not want this.  The LX7 has a single-precision FPU and
 * the compiler uses it: built with -mdynconfig=xtensa_esp32s3.so the V.34
 * modem emits 1164 hardware FP instructions -- 57 mul.s and 37 madd.s in the
 * data stage alone -- and is 9% SMALLER than the soft-float build.  Trading
 * madd.s for integer MAC plus shift and saturate bookkeeping would cost
 * dynamic range and buy nothing.
 *
 * It exists for the FPU-less parts: ESP32-C3/C6/H2 are RV32IMC with no F
 * extension, so every float operation there is a libgcc call.
 *
 * FORMATS, DERIVED FROM MEASURED DATA rather than chosen
 *
 * Taken from V90_T3_SYMBOL_PROBE on artifacts/reneg-eq/reneg-r1, one real
 * data symbol, 21 complex taps:
 *
 *     |tap| max 0.005743, min 0.000069   -> 7 bits of range, but the small
 *                                           taps still need resolution
 *     |ring| max 3309 (G.711 can reach ~32635 after the Hilbert)
 *     |y|   ~27
 *
 * So Q1.15 is NOT enough for the taps: 0.000069 * 32768 = 2 counts, one bit of
 * precision on the smallest tap.  Taps are Q1.30 in int32 (0.000069 becomes
 * 74000 counts), the ring is Q17.14, and products accumulate in int64 before a
 * single shift back.  21 taps x 4 real multiplies = 84 int64 MACs per symbol,
 * 269k/s at 3200 baud -- comfortable on a 160 MHz C3.
 */
#ifndef PORT_V34_FIXED_H
#define PORT_V34_FIXED_H

#include <stdint.h>

#define V34_FX_TAP_SHIFT    30                      /* taps  Q1.30  */
#define V34_FX_RING_SHIFT   14                      /* ring  Q17.14 */
#define V34_FX_OUT_SHIFT    (V34_FX_TAP_SHIFT + V34_FX_RING_SHIFT - V34_FX_RING_SHIFT)

typedef struct { int32_t re, im; } v34_fx_complex_t;

static __inline__ int32_t v34_fx_from_float(float v, int shift)
{
    double s = (double) v * (double) (1u << shift);

    if (s >  2147483647.0) return  2147483647;
    if (s < -2147483648.0) return -2147483647 - 1;
    return (int32_t) (s < 0 ? s - 0.5 : s + 0.5);
}

static __inline__ float v34_fx_to_float(int32_t v, int shift)
{
    return (float) v / (float) (1u << shift);
}

/* One FSE output: sum(taps[k] * ring[k]) over n complex taps.
 * Result is Q17.14, matching the ring, so the caller's scale is unchanged. */
static __inline__ v34_fx_complex_t v34_fx_fse(const v34_fx_complex_t *taps,
                                              const v34_fx_complex_t *ring,
                                              int n)
{
    int64_t acc_re = 0;
    int64_t acc_im = 0;
    v34_fx_complex_t out;
    int k;

    for (k = 0;  k < n;  k++)
    {
        acc_re += (int64_t) taps[k].re*ring[k].re - (int64_t) taps[k].im*ring[k].im;
        acc_im += (int64_t) taps[k].re*ring[k].im + (int64_t) taps[k].im*ring[k].re;
    }
    out.re = (int32_t) (acc_re >> V34_FX_TAP_SHIFT);
    out.im = (int32_t) (acc_im >> V34_FX_TAP_SHIFT);
    return out;
}

#endif
