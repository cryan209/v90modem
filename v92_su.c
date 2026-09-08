#include "v92_su.h"
#include <spandsp.h>
#include <string.h>
#include <math.h>
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
        if (s->filled == 6 && s->phase == 5) {
            double re = 0, im = 0;
            for (int j = 0; j < 6; j++) { re += c[j]*s->window[j]; im += q[j]*s->window[j]; }
            double energy = re*re+im*im;
            double previous = s->previous_i*s->previous_i+s->previous_q*s->previous_q;
            double change = (re-s->previous_i)*(re-s->previous_i)
                          + (im-s->previous_q)*(im-s->previous_q);
            bool periodic = energy > 900.0*900*9 && previous > 0 && change < .03*energy;
            s->stable = periodic ? s->stable+1 : 0;
            double reference = s->reference_i*s->reference_i+s->reference_q*s->reference_q;
            double dot = re*s->reference_i+im*s->reference_q;
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
