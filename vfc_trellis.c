/* V.FC streaming Viterbi on Courier's verified 2D branch geometry.
 * See vfc_trellis.h; does not share V.34's 4D state transitions. */
#include "vfc_trellis.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define UNREACHABLE (FLT_MAX/4)

bool vfc_trellis_init(vfc_trellis_t *s, unsigned code, unsigned points,
                      unsigned initial_quadrant)
{
    if (!s || (code != 8 && code != 32) || !points || points > 160 || initial_quadrant > 3)
        return false;
    memset(s, 0, sizeof(*s));
    s->points = points;
    s->quadrant = initial_quadrant;
    for (unsigned state = 0; state < 32; state++) {
        s->metric[state] = state ? UNREACHABLE : 0;
        for (unsigned q = 0; q < 4; q++) {
            vfc_mapper_t mapper = {code, state, 0};
            vfc_point_t point;
            vfc_mapper_put(&mapper, 0, q, &point);
            s->next[state][q] = mapper.trellis;
        }
    }
    for (unsigned parity = 0; parity < 2; parity++)
        for (unsigned q = 0; q < 4; q++)
            for (unsigned index = 0; index < points; index++) {
                vfc_mapper_t mapper = {code, parity, 0};
                vfc_mapper_put(&mapper, index, q, &s->constellation[4*parity+q][index]);
            }
    s->valid = true;
    return true;
}

static void oldest(vfc_trellis_t *s, vfc_decision_t *out)
{
    unsigned state = s->best;
    for (uint64_t pos = s->received; pos > s->decoded;) {
        pos--;
        const vfc_survivor_t *h = &s->history[pos % VFC_TRELLIS_DEPTH][state];
        if (pos == s->decoded) {
            out->index = h->index;
            out->dibit = (h->quadrant + 4 - s->quadrant) & 3;
            s->quadrant = h->quadrant;
            s->decoded++;
            return;
        }
        state = h->previous;
    }
}

int vfc_trellis_put(vfc_trellis_t *s, float i, float q, vfc_decision_t *out)
{
    if (!s || !s->valid || s->finished || !out || !isfinite(i) || !isfinite(q)
        || !isfinite(i*i + q*q)) return -1;
    float distance[8], metric[32];
    uint8_t index[8];
    vfc_survivor_t survivors[32] = {{0}};
    for (unsigned family = 0; family < 8; family++) {
        distance[family] = UNREACHABLE;
        index[family] = 0;
        for (unsigned point = 0; point < s->points; point++) {
            float di = i - s->constellation[family][point].i;
            float dq = q - s->constellation[family][point].q;
            float d = di*di + dq*dq;
            if (d < distance[family]) {
                distance[family] = d;
                index[family] = point;
            }
        }
    }
    for (unsigned state = 0; state < 32; state++) metric[state] = UNREACHABLE;
    for (unsigned state = 0; state < 32; state++) {
        if (s->metric[state] == UNREACHABLE) continue;
        for (unsigned quadrant = 0; quadrant < 4; quadrant++) {
            unsigned family = 4*(state & 1) + quadrant;
            unsigned next = s->next[state][quadrant];
            float d = s->metric[state] + distance[family];
            if (d < metric[next]) {
                metric[next] = d;
                survivors[next] = (vfc_survivor_t){state, quadrant, index[family]};
            }
        }
    }
    unsigned best = 0;
    for (unsigned state = 1; state < 32; state++)
        if (metric[state] < metric[best]) best = state;
    float minimum = metric[best];
    if (minimum == UNREACHABLE) return -1;
    for (unsigned state = 0; state < 32; state++)
        s->metric[state] = metric[state] == UNREACHABLE ? UNREACHABLE : metric[state] - minimum;
    memcpy(s->history[s->received % VFC_TRELLIS_DEPTH], survivors, sizeof(survivors));
    s->best = best;
    s->received++;
    if (s->received < VFC_TRELLIS_DEPTH) return 0;
    oldest(s, out);
    return 1;
}

unsigned vfc_trellis_finish(vfc_trellis_t *s, vfc_decision_t out[63])
{
    if (!s || !s->valid || s->finished || !out) return 0;
    unsigned count = 0;
    while (s->decoded < s->received) oldest(s, &out[count++]);
    s->finished = true;
    return count;
}
