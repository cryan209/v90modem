/* V.92 8.5.6 / 9.5.1.1.6-9: Su, Su-bar, Su, final Su-bar.
 * Digital receiver, one byte-exact G.711 DS0 codeword per input sample.
 * Fixed-phase polarity checks prevent the inverse/three-slot alias from
 * manufacturing transitions. The middle 24.5T interval permits one sample
 * of phase change when acquiring the returned Su. After the final bar,
 * verifies absolute-GPA TRN1u before reporting zero-DIL training readiness
 * (9.5.1.1.13). This is a symbol-clock receiver, without an analogue EQ. */
#ifndef V92_SU_H
#define V92_SU_H
#include <stdint.h>
#include <stdbool.h>
typedef enum { V92_SU_NONE, V92_SU_ACQUIRED, V92_SU_BAR, V92_SU_RETURNED, V92_SU_FINAL, V92_SU_TRAINED } v92_su_event_t;
typedef struct {
    unsigned run[6], phase, transition_run;
    uint32_t training_reg;
    unsigned training_samples, training_ones;
    int lock;
    v92_su_event_t stage;
    bool alaw;
    double window[6], previous_i, previous_q, reference_i, reference_q;
    unsigned filled, stable;
} v92_su_t;
void v92_su_init(v92_su_t *s, bool alaw);
v92_su_event_t v92_su_put(v92_su_t *s, uint8_t cw);
#endif
