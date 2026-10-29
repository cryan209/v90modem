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
 */
#include "spandsp-master/src/v34rx.c"

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
    return failed ? 1 : 0;
}
