#include "v92_su.h"
#include <spandsp.h>
#include <string.h>
#include <math.h>
/* The six-slot bin energy a Su must reach.  It was 900^2*9 (a 900-unit
 * mean amplitude), which the HSF's Su sits just under: its bin reads
 * 7.20-7.24M and its 24.5T Su-bar 6.56-7.68M against a floor of 7.29M,
 * so the 9.5.2.1.7 reversal was never seen, Jp was never sent and the HSF
 * retrained after 3.6 s of Su on every V.92 call (D-Modem rig,
 * 2026-11-26).  6 dB lower; the periodicity and polarity tests, not the
 * level, are what reject non-Su signals. */
#define V92_SU_MIN_ENERGY (450.0*450*9)

void v92_su_init(v92_su_t *s, bool alaw)
{
    memset(s, 0, sizeof(*s));
    s->lock = -1;
    s->alaw = alaw;
}
v92_su_event_t v92_su_put(v92_su_t *s, uint8_t cw)
{
    int sample = s->alaw ? alaw_to_linear(cw) : ulaw_to_linear(cw);
    int obs = sample > 900 ? 1 : sample < -900 ? -1 : 0;
    v92_su_event_t event = V92_SU_NONE;
    if (s->stage == V92_SU_FINAL) {
        /* 8.5.7/9.5.1.1.13: accept actual absolute-GPA TRN1u, never
         * a timer standing in for an upstream receiver. Require a clean
         * 2040T run after the self-synchronizing scrambler's 23 bits. */
        unsigned bit = obs < 0;
        unsigned one = (bit ^ (s->training_reg >> 4) ^ (s->training_reg >> 22)) & 1;
        s->training_reg = (s->training_reg << 1) | bit;
        s->training_samples++;
        if (s->training_samples > 23) {
            s->training_ones = obs && one ? s->training_ones + 1 : 0;
            if (s->training_ones >= 2040) {
                event = V92_SU_TRAINED;
                s->stage = event;
            }
        }
    } else if (s->stage == V92_SU_TRAINED) {
        /* The Phase-4 receiver owns the following CPt/TRN2u. */
    } else {
        /* §8.5.6's six-symbol Su has a 1333 1/3 Hz fundamental. A phone
         * line attenuates the Nyquist component, so its zero slots need not
         * survive. Measure the fundamental on an absolute six-slot clock:
         * a reversal is then a phase CHANGE, not the three-slot alias.
         * §9.5.2.1.7 permits a half-symbol shift only on returned Su. */
        static const double c[6] = {1, .5, -.5, -1, -.5, .5};
        static const double q[6] = {0, .8660254037844386, .8660254037844386,
                                    0, -.8660254037844386, -.8660254037844386};
        s->window[s->phase] = sample;
        if (s->filled < 6) s->filled++;
        /* 9.5.2.1.7 sends Su-bar for only 24.5T, about four blocks of this
         * grid, wherever it happens to fall on it; with line ISI no reversed
         * block was stable enough on most alignments (slmodemd, live: we
         * missed it, never sent Jp, and the peer retrained).  The same bin
         * evaluated every sample on the absolute clock does not depend on the
         * alignment: six consecutive reversed windows (one Su period) is
         * Su-bar. */
        s->slide_sum_i += c[s->phase]*sample - s->slide_i[s->phase];
        s->slide_sum_q += q[s->phase]*sample - s->slide_q[s->phase];
        s->slide_i[s->phase] = c[s->phase]*sample;
        s->slide_q[s->phase] = q[s->phase]*sample;
        if (s->slide_filled < 6) s->slide_filled++;
        if (s->stage == V92_SU_ACQUIRED && s->slide_filled == 6) {
            double e = s->slide_sum_i*s->slide_sum_i + s->slide_sum_q*s->slide_sum_q;
            double r = s->reference_i*s->reference_i + s->reference_q*s->reference_q;
            double d = s->slide_sum_i*s->reference_i + s->slide_sum_q*s->reference_q;
            s->bar_run = (e > V92_SU_MIN_ENERGY && d < -.85*sqrt(r*e)) ? s->bar_run+1 : 0;
            if (s->bar_run >= 6) {
                event = V92_SU_BAR;
                s->stage = event;
                s->lock = 0;
                s->stable = 0;
                s->bar_run = 0;
            }
        }
        if (s->filled == 6 && s->phase == 5 && !event) {
            double re = 0, im = 0;
            for (int j = 0; j < 6; j++) { re += c[j]*s->window[j]; im += q[j]*s->window[j]; }
            double energy = re*re+im*im;
            double previous = s->previous_i*s->previous_i+s->previous_q*s->previous_q;
            double change = (re-s->previous_i)*(re-s->previous_i)
                          + (im-s->previous_q)*(im-s->previous_q);
            bool periodic = energy > V92_SU_MIN_ENERGY && previous > 0 && change < .03*energy;
            s->stable = periodic ? s->stable+1 : 0;
            double reference = s->reference_i*s->reference_i+s->reference_q*s->reference_q;
            double dot = re*s->reference_i+im*s->reference_q;
            /* Sum the reference's own polarity only: the block straddling
             * the reversal fails the periodic test, and a reversed block
             * seen before the sliding detector fires fails the dot test. */
            if (s->stage == V92_SU_ACQUIRED && periodic
                && dot > .85*sqrt(reference*energy)) {
                s->phase_i += re;
                s->phase_q += im;
                s->phase_blocks++;
            }
            if (s->lock < 0 && s->stable >= 3) {
                event = V92_SU_ACQUIRED;
            } else if (s->stage == V92_SU_BAR && s->stable >= 1
                       && dot > .35*sqrt(reference*energy)) {
                event = V92_SU_RETURNED;
            } else if ((s->stage == V92_SU_ACQUIRED || s->stage == V92_SU_RETURNED)
                       && s->stable >= 1 && dot < -.85*sqrt(reference*energy)) {
                event = s->stage == V92_SU_ACQUIRED ? V92_SU_BAR : V92_SU_FINAL;
            }
            if (event) {
                s->stage = event;
                s->lock = 0;
                s->stable = 0;
                if (event == V92_SU_ACQUIRED || event == V92_SU_RETURNED) {
                    s->reference_i = re;
                    s->reference_q = im;
                }
            }
            s->previous_i = re;
            s->previous_q = im;
        }
    }
    s->phase = (s->phase+1)%6;
    return event;
}

bool v92_su_jp_phase(const v92_su_t *s, double *fraction, unsigned *blocks)
{
    /* Su = {a, 0, a, -a, 0, -a} has fundamental phase -pi/3 on its own
     * slots; received m symbols late the bin sum_n y(n) e^{+j 2 pi n/6}
     * on the absolute slot clock has argument 2 pi m/6 + pi/3, so
     * frac(m) = frac(6 arg/2 pi) whatever whole-symbol offset the slot
     * clock has.  After Su-bar's half symbol the delay is m + 1/2, and
     * extending Su by 1 - frac(m + 1/2) puts it on a symbol. */
    double m, f;

    if (blocks)
        *blocks = s ? s->phase_blocks : 0;
    if (!s || s->phase_blocks < 4 || (s->phase_i == 0.0 && s->phase_q == 0.0))
        return false;
    m = 6.0*atan2(s->phase_q, s->phase_i)/(2.0*M_PI) + 0.5;
    f = -m - floor(-m);
    if (f >= 1.0)
        f = 0.0;
    if (fraction)
        *fraction = f;
    return true;
}
