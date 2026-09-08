#include "v92_su.h"
#include <spandsp.h>
#include <string.h>
void v92_su_init(v92_su_t *s, bool alaw)
{
    memset(s, 0, sizeof(*s));
    s->lock = -1;
    s->alaw = alaw;
}
v92_su_event_t v92_su_put(v92_su_t *s, uint8_t cw)
{
    static const int p[6] = {1,0,1,-1,0,-1};
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
    } else if (s->lock < 0 || s->stage == V92_SU_BAR) {
        for (int h = 0; h < 6; h++) {
            if (s->lock >= 0 && h != s->lock && h != (s->lock+1)%6 && h != (s->lock+5)%6)
                continue;
            s->run[h] = obs == p[(s->phase+h)%6] ? s->run[h]+1 : 0;
            if (s->run[h] >= 24) {
                event = s->lock < 0 ? V92_SU_ACQUIRED : V92_SU_RETURNED;
                s->stage = event;
                s->lock = h;
                s->transition_run = 0;
                memset(s->run, 0, sizeof(s->run));
                break;
            }
        }
    } else if (s->stage != V92_SU_FINAL) {
        s->transition_run = obs == -p[(s->phase+s->lock)%6] ? s->transition_run+1 : 0;
        if (s->transition_run >= 24) {
            event = s->stage == V92_SU_ACQUIRED ? V92_SU_BAR : V92_SU_FINAL;
            s->stage = event;
            s->transition_run = 0;
        }
    }
    s->phase = (s->phase+1)%6;
    return event;
}
