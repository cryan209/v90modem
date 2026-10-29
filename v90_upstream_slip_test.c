/*
 * V.90 8.5.2/9.4.2.5: local T/3 timing recovery must preserve the FSE's
 * carrier coordinates. Include the production receiver, as the generator
 * tests include v34tx.c, to exercise the real static scorer and actuator.
 *
 * This is a scoring/state-transition test, not a pulse-shaped bearer or
 * hardware test. Supply a ring with a known odd-integer constellation one
 * sample away, with the passband-delay phase and an independent carrier
 * phase. The other positions deliberately have a closed eye. Both input
 * orientations must find the same physical delay and land back on the grid.
 * Also check fractional-coordinate equivalence and the V.34 10.1.3.1 B1
 * acquisition correlation's invariance when template energy is cached.
 */
#include "spandsp-master/src/v34rx.c"
#include "port/v34_fixed.h"

static int check_case(int baud, int high, int conjugate, int slip, float phase)
{
    v34_rx_state_t *s = calloc(1, sizeof(*s));
    const int64_t first = 3999; /* multiple of three */
    float omega = 2.0f*(float)M_PI*carrier_frequency(baud, high)
                  /(3.0f*baud_rate_parameters[baud].baud_rate);
    float before, candidate, after;
    bool recovered;
    int failed;

    if (!s)
        return 1;
    s->baud_rate = baud;
    s->high_carrier = high;
    s->v90_t3_internal_rate = 3*baud_rate_parameters[baud].baud_rate;
    s->v90_t3_fse_conjugate = conjugate != 0;
    s->v90_t3_fse[V34_V90_T3_FSE_TAPS/2].re = 1.0f;
    s->v90_t3_carrier_enabled = true;
    s->v90_t3_carrier.phase = phase;
    /* Disable fractional timing so the only applicable moves are +/-1.
       This does not disable the carrier correction accompanying a move. */
    s->v90_t3_timing_enabled = false;
    s->v90_t3_next_symbol = first;
    s->v90_t3_raw_count = 4096;
    for (int n = 0; n < 4096; n++) {
        if ((n - slip) % 3 == 0) {
            uint32_t k = (uint32_t)((n - slip)/3);
            float re, im, zr, zi;

            k = k*1664525U + 1013904223U;
            re = 2.0f*(float)((k >> 16) & 7) - 7.0f;
            im = 2.0f*(float)((k >> 24) & 7) - 7.0f;
            zr = re*cosf(phase) - im*sinf(phase);
            zi = re*sinf(phase) + im*cosf(phase);
            if (conjugate)
                zi = -zi;
            /* A passband delay D yields b(n-D)*exp(-j*omega*D)
               after mixing by the absolute receive sample counter. */
            s->v90_t3_raw[n].re = zr*cosf(omega*slip) + zi*sinf(omega*slip);
            s->v90_t3_raw[n].im = zi*cosf(omega*slip) - zr*sinf(omega*slip);
        }
    }
    before = v90_t3_score_offset(s, 0.0f);
    candidate = v90_t3_score_offset(s, (float)slip);
    recovered = v90_t3_slip_resync(s);
    after = v90_t3_score_offset(s, 0.0f);
    failed = before < 1.0f || candidate > 1e-8f || !recovered
             || s->v90_t3_next_symbol != first + slip
             || s->v90_t3_slips_recovered != 1 || after > 1e-8f;
    printf("%s: %d baud %s carrier conjugate=%d slip=%+d phase=%+.2f "
           "score %.6f -> %.6f, candidate %.6f recovered=%d\n",
           failed ? "FAIL" : "PASS", baud_rate_parameters[baud].baud_rate,
           high ? "high" : "low", conjugate, slip, phase,
           before, after, candidate, recovered);
    free(s);
    return failed;
}

/* The scorer adds a trial offset to Gardner's residual fraction. That sum
   is not restricted to [0,1): normalize the integer carry rather than
   clamping everything above one to a whole-sample read. */
static int check_fractional_positions(void)
{
    const float fractions[] = {-2.25f, -1.25f, -0.75f, -0.25f,
                                0.0f, 0.25f, 0.75f, 1.0f, 1.25f, 2.25f};
    v34_rx_state_t *s = calloc(1, sizeof(*s));
    int failed = 0;

    if (!s)
        return 1;
    s->baud_rate = 4;
    s->v90_t3_internal_rate = 9600;
    s->v90_t3_raw_count = 4096;
    s->v90_t3_next_symbol = 3999;
    s->v90_t3_timing_enabled = true;
    s->v90_t3_fse[V34_V90_T3_FSE_TAPS/2].re = 1.0f;
    for (int n = 0; n < 4096; n++) {
        s->v90_t3_raw[n].re = 5.0f*cosf(0.73f*n) + 2.0f*sinf(0.31f*n);
        s->v90_t3_raw[n].im = 3.0f*sinf(0.83f*n) - 4.0f*cosf(0.27f*n);
    }
    for (unsigned k = 0; k < sizeof(fractions)/sizeof(fractions[0]); k++) {
        float frac = fractions[k];
        int whole = (int)floorf(frac);
        float residual = frac - whole;
        complexf_t got = v90_t3_raw_get_frac(s, 3000, frac);
        complexf_t expected = v90_t3_raw_get_frac(s, 3000 + whole, residual);
        float error = fabsf(got.re - expected.re) + fabsf(got.im - expected.im);
        int need = whole + (residual > 1e-4f ? 4 : 0);
        int ahead = v90_t3_frac_lookahead(frac);
        bool bad = error > 1e-5f || ahead < need;

        printf("%s: fractional read %+.2f: equivalent-position error %.6f, "
               "lookahead=%d required=%d\n", bad ? "FAIL" : "PASS", frac,
               error, ahead, need);
        failed += bad;
    }
    for (int conjugate = 0; conjugate < 2; conjugate++) {
        float a, b;
        bool bad;

        s->v90_t3_fse_conjugate = conjugate != 0;
        s->v90_t3_gardner.acc = 0.4f;
        a = v90_t3_score_offset(s, 5.0f/6.0f);
        /* Same physical position, represented with a one-sample carry. */
        s->v90_t3_next_symbol++;
        s->v90_t3_gardner.acc = -0.6f;
        b = v90_t3_score_offset(s, 5.0f/6.0f);
        s->v90_t3_next_symbol--;
        bad = fabsf(a - b) > 1e-4f;
        printf("%s: fractional candidate conjugate=%d: same-position scores "
               "%.6f / %.6f\n", bad ? "FAIL" : "PASS", conjugate, a, b);
        failed += bad;
    }
    /* The optional integer actuator uses Q16 linear interpolation rather
       than this path's sinc, but it must carry the same whole-sample part. */
    {
        v34_fx_complex_t ring[16];

        for (int n = 0; n < 16; n++) {
            ring[n].re = (n*n*137 + 31*n) % 2000;
            ring[n].im = (n*n*n*19 + 7*n) % 2000;
        }
        for (unsigned k = 0; k < sizeof(fractions)/sizeof(fractions[0]); k++) {
            float frac = fractions[k];
            int whole = (int)floorf(frac);
            int32_t q16 = (int32_t)(frac*65536.0f);
            int32_t residual = (int32_t)((frac - whole)*65536.0f);
            v34_fx_complex_t got = v34_fx_ring_get_frac(ring, 8, q16, 16, 16, 15);
            v34_fx_complex_t expected = v34_fx_ring_get_frac(ring, 8 + whole,
                                                             residual, 16, 16, 15);
            bool bad = got.re != expected.re || got.im != expected.im;

            printf("%s: Q16 fractional read %+.2f: (%d,%d) / (%d,%d)\n",
                   bad ? "FAIL" : "PASS", frac, got.re, got.im,
                   expected.re, expected.im);
            failed += bad;
        }
    }
    free(s);
    return failed;
}

/* Hoisting template energy must not change either spectral orientation's
   normalized correlation (V.34 10.1.3.1), including clipped ring windows. */
static int check_coarse_template_power(void)
{
    const int64_t positions[] = {-2, 0, 1, 37, 128, 999, 4080, 4096};
    v34_rx_state_t *s = calloc(1, sizeof(*s));
    int failed = 0;

    if (!s)
        return 1;
    s->v90_t3_raw_count = 4096;
    for (int mode = 0; mode < 4; mode++) {
        double power = 0.0;
        int mismatches = 0;

        s->v90_t3_b1_symbols = mode & 1 ? 128 : 32;
        for (int n = 0; n < 4096; n++) {
            s->v90_t3_matched[n].re = mode == 2 ? 0.0f :
                ((n*137 + 31) % 4093 - 2046)*0.03125f;
            s->v90_t3_matched[n].im = mode == 2 ? 0.0f :
                ((n*239 + 17) % 4091 - 2045)*0.007f;
        }
        for (int n = 0; n < s->v90_t3_b1_symbols; n++) {
            complexf_t y = {mode == 3 ? 0.0f : (n % 17 - 8)*0.13f,
                            mode == 3 ? 0.0f : (n % 13 - 6)*0.27f};
            s->v90_t3_b1[n] = y;
            power += y.re*y.re + y.im*y.im;
        }
        for (unsigned k = 0; k < sizeof(positions)/sizeof(positions[0]); k++) {
            float score[2];

            v90_t3_coarse_scores(s, positions[k], power, score);
            for (int conjugate = 0; conjugate < 2; conjugate++) {
                float reference = v90_t3_coarse_score(s, positions[k],
                                                       conjugate != 0);
                mismatches += score[conjugate] != reference;
            }
        }
        printf("%s: cached template energy mode=%d: %d/16 score mismatches\n",
               mismatches ? "FAIL" : "PASS", mode, mismatches);
        failed += mismatches;
    }
    free(s);
    return failed;
}

int main(void)
{
    const int bauds[] = {3, 4};
    const float phases[] = {-0.37f, 0.0f, 0.63f};
    int failed = 0;

    for (unsigned b = 0; b < sizeof(bauds)/sizeof(bauds[0]); b++)
        for (int high = 0; high < 2; high++)
            for (int conjugate = 0; conjugate < 2; conjugate++)
                for (int slip = -1; slip <= 1; slip += 2)
                    for (unsigned p = 0; p < sizeof(phases)/sizeof(phases[0]); p++)
                        failed += check_case(bauds[b], high, conjugate, slip, phases[p]);
    failed += check_fractional_positions();
    failed += check_coarse_template_power();
    return failed ? 1 : 0;
}
