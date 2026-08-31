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


/* ---- Decision-directed NLMS -------------------------------------------
 *
 * tap += (mu/energy) * e * conj(x), the update in v90_t3_emit_ready().
 *
 * THE FAILURE MODE, AND IT IS NOT THE TEXTBOOK ONE.  With mu = 0.02
 * (v90_t3_dd_mu) and NLMS normalisation a correction is about
 * 0.02*|e|/(42*|x|); at a settled error of 0.01 and |x| ~ 3000 that is 1.6e-9,
 * under two LSBs of the Q1.30 the taps are stored in.  The expected result is
 * a stall -- corrections round to zero, the filter freezes.  Measured, that is
 * NOT what happens.  port/v34_fixed_lms_test.c sweeps the accumulator width
 * against a float NLMS on the same data, 200000 symbols:
 *
 *     accumulator   residual/symbol   taps vs float   tap[0] updates/1000
 *     Q1.30         6.66e-01          -17.5 dB        1000
 *     Q1.34         6.62e-01           +0.6 dB        1000
 *     Q1.38         3.20e-09          113.9 dB         646
 *     Q1.42         1.96e-09          114.0 dB         529
 *     Q1.46         2.97e-09          114.0 dB         676
 *
 * The narrow accumulators update every single symbol -- they are not stalled.
 * They are being driven by their own quantisation noise instead of the
 * gradient, and settle at 0.666, which is precisely the mean-square
 * distance-to-grid of symbols bearing no relation to the lattice.  A stall
 * would have been obvious; this looks like a working adaptive filter and
 * produces white output.
 *
 * The cliff is between Q1.34 and Q1.38.  Taps are therefore carried in Q1.46
 * inside int64 -- eight bits past the cliff -- and narrowed to Q1.30 only when
 * the filter is applied.  Above the cliff the fixed loop converges to the same
 * filter as float to 114 dB, and its residual is actually LOWER (3e-09 against
 * 6.9e-06), because a 46-bit accumulator carries the tiny corrections that
 * float32's 24-bit mantissa rounds away.
 */
typedef struct { int64_t re, im; } v34_fx_acc_t;

#ifndef V34_FX_ACC_SHIFT
#define V34_FX_ACC_SHIFT    46
/* ---- Least squares: the B1 supervised FSE fit ---------------------------
 *
 * v90_t3_solve() is Gauss-Jordan with partial pivoting over n=42 (21 complex
 * taps as real/imag), and it is the piece most likely to break in fixed point,
 * so it was measured before it was written.  Over 24 real solves from
 * artifacts/reneg-eq/reneg-r1:
 *
 *     input |a| max 4.2e8, and max/min across the matrix 1.2e3 .. 1.1e7
 *     pivots: first ~4.0e8, LAST ~2.9e5 (min seen 2.6e5)
 *     pivot max/min WITHIN a solve: median 1.5e3, worst 1.6e3
 *
 * THAT PIVOT RATIO IS NOT A CONDITION NUMBER, and reading it as one is the
 * mistake this comment exists to stop.  It says 10.7 bits.  The actual
 * condition number, computed as ||A||inf * ||A^-1||inf over the same three
 * systems, is 2.89e5, 2.89e5 and 2.80e5 -- 18.1 bits, understated by 7.5.
 *
 * MEASURED RESULT: THIS DOES NOT WORK, AND THE SOLVE SHOULD STAY IN DOUBLE.
 * Against the three real systems, float residual 1.3e-13:
 *
 *     shift   relative residual   |x_fixed - x_float| / |x_float|
 *     Q16     2.06e-01            2.9e+03
 *     Q20     9.18e-03            5.4e+01
 *     Q24     5.73e-04            3.1e+00
 *     Q26     1.34e+03            2.2e+03
 *     Q28     7.09e+03            1.3e+04
 *     Q31     1.08e+03            1.0e+03
 *
 * Above Q24 the `a << SHIFT` before the divide overflows int64: entries grow
 * by up to the pivot ratio during elimination, so 1600*2^31 << 31 is 7.4e21
 * against int64's 9.2e18.  Below it there are not enough fractional bits.  The
 * window is Q20..Q24 and the budget explains why there is no good point in it:
 * 63 bits, minus SHIFT burned by the pre-divide shift, minus 11 bits of
 * intra-solve growth, minus 18 bits lost to conditioning.  At Q24 that leaves
 * about 6 bits of solution accuracy.
 *
 * And the residual is the WRONG criterion here.  Q24's 5.7e-4 looks tolerable
 * until you notice |dx|/|x| = 3.1: the taps differ from the float solution by
 * 310% while still fitting B1, because an 18-bit-conditioned system has many x
 * that fit.  Whether such a tap set generalises is exactly what the receiver's
 * own out-of-sample check exists to catch, so a small in-sample residual here
 * means very little.
 *
 * The engineering answer is the hybrid: per-symbol work in fixed point, this
 * solve in double.  It runs once per acquisition -- 24 times in a 600 s call --
 * so even soft-float double costs ~46 ms per acquisition on an FPU-less part,
 * against 3200 symbols a second that now cost nothing.  Making that pay would
 * need 128-bit intermediates (not available on RV32) or a different algorithm
 * (Householder QR is far better conditioned than Gauss-Jordan here), and
 * neither is worth it for a once-per-acquisition operation.
 *
 * Kept, not deleted, so the next person reads the table instead of retrying it.
 */
#ifndef V34_FX_SOLVE_SHIFT
#define V34_FX_SOLVE_SHIFT  31
#endif

static __inline__ int v34_fx_solve(int64_t *a, int64_t *b, int64_t *x, int n)
{
    const int64_t one = (int64_t) 1 << V34_FX_SOLVE_SHIFT;
    int i, j, k;

    for (i = 0;  i < n;  i++)
    {
        int pivot = i;
        int64_t best = a[i*n + i] < 0 ? -a[i*n + i] : a[i*n + i];

        for (j = i + 1;  j < n;  j++)
        {
            int64_t v = a[j*n + i] < 0 ? -a[j*n + i] : a[j*n + i];

            if (v > best) { best = v; pivot = j; }
            /*endif*/
        }
        /* The float guard is 1e-12 against entries of order 1 after scaling;
           at Q31 the equivalent floor is a few counts.  Measured pivots never
           come near it, so this catches a singular system, not a tight one. */
        if (best < 16)
            return 0;
        /*endif*/
        if (pivot != i)
        {
            for (k = i;  k < n;  k++)
            {
                int64_t t = a[i*n + k]; a[i*n + k] = a[pivot*n + k]; a[pivot*n + k] = t;
            }
            /*endfor*/
            { int64_t t = b[i]; b[i] = b[pivot]; b[pivot] = t; }
        }
        /*endif*/
        {
            int64_t d = a[i*n + i];

            for (k = i;  k < n;  k++)
                a[i*n + k] = (a[i*n + k] << V34_FX_SOLVE_SHIFT)/d;
            /*endfor*/
            b[i] = (b[i] << V34_FX_SOLVE_SHIFT)/d;
        }
        for (j = 0;  j < n;  j++)
        {
            int64_t f;

            if (j == i)
                continue;
            /*endif*/
            f = a[j*n + i];
            if (f == 0)
                continue;
            /*endif*/
            for (k = i;  k < n;  k++)
                a[j*n + k] -= (f*a[i*n + k]) >> V34_FX_SOLVE_SHIFT;
            /*endfor*/
            b[j] -= (f*b[i]) >> V34_FX_SOLVE_SHIFT;
        }
        /*endfor*/
    }
    /*endfor*/
    for (i = 0;  i < n;  i++)
        x[i] = b[i];
    /*endfor*/
    (void) one;
    return 1;
}

#endif

static __inline__ void v34_fx_lms_init(v34_fx_acc_t *acc,
                                       const v34_fx_complex_t *taps, int n)
{
    int k;

    for (k = 0;  k < n;  k++)
    {
        acc[k].re = (int64_t) taps[k].re << (V34_FX_ACC_SHIFT - V34_FX_TAP_SHIFT);
        acc[k].im = (int64_t) taps[k].im << (V34_FX_ACC_SHIFT - V34_FX_TAP_SHIFT);
    }
}

/* Narrow the wide accumulator back to the taps the FSE uses. */
static __inline__ void v34_fx_lms_taps(v34_fx_complex_t *taps,
                                       const v34_fx_acc_t *acc, int n)
{
    int k;

    for (k = 0;  k < n;  k++)
    {
        taps[k].re = (int32_t) (acc[k].re >> (V34_FX_ACC_SHIFT - V34_FX_TAP_SHIFT));
        taps[k].im = (int32_t) (acc[k].im >> (V34_FX_ACC_SHIFT - V34_FX_TAP_SHIFT));
    }
}

/* One NLMS step.
 *
 * err is Q17.14 like the ring.  energy is sum(|x|^2) as a PLAIN int64 in
 * real units (the caller has it already), NOT a Q-format value -- mixing
 * formats through a divide is how the first version of this silently
 * truncated every correction to zero.
 *
 * Derivation, kept because the shift is not guessable:
 *   pr   = sum(err*ring)          -> value (e.x) scaled by 2^28
 *   want acc += mu * (e.x)/energy scaled by 2^ACC
 *   so   acc += (mu_q30/2^30) * (pr/2^28) / energy * 2^ACC
 *             = pr * mu_q30 / energy >> (30 + 28 - ACC)
 * With ACC = 46 that is a LEFT shift of 12, so do it before the divide to
 * keep the small corrections this exists to preserve.
 */
static __inline__ void v34_fx_lms_update(v34_fx_acc_t *acc,
                                         const v34_fx_complex_t *ring,
                                         int n,
                                         int32_t err_re, int32_t err_im,
                                         int32_t mu_q30,
                                         int64_t energy)
{
    int k;

    if (energy <= 0)
        return;
    /*endif*/
    for (k = 0;  k < n;  k++)
    {
        int64_t pr = ((int64_t) err_re*ring[k].re + (int64_t) err_im*ring[k].im) >> 14;
        int64_t pi = ((int64_t) err_im*ring[k].re - (int64_t) err_re*ring[k].im) >> 14;

        acc[k].re += (((pr*mu_q30) >> 2)/energy) << 2;
        acc[k].im += (((pi*mu_q30) >> 2)/energy) << 2;
    }
    /*endfor*/
}

#endif
