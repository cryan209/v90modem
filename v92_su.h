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
    /* Sliding six-sample bin on the same absolute clock, for Su-bar. */
    double slide_i[6], slide_q[6], slide_sum_i, slide_sum_q;
    unsigned slide_filled, bar_run;
    /* 9.5.1.1.6: the fundamental summed over the Su blocks that match the
     * acquired reference, for the Jp phase (v92_su_jp_phase). */
    double phase_i, phase_q;
    unsigned phase_blocks;
} v92_su_t;
void v92_su_init(v92_su_t *s, bool alaw);
v92_su_event_t v92_su_put(v92_su_t *s, uint8_t cw);
/* 9.5.1.1.6-8 / Table 22 bits 18:33: the fraction of a symbol, [0, 1), by
 * which the analogue modem should extend Su so that the network A/D samples
 * its pulse on the peak, from the phase of the 144T Su's 1333 1/3 Hz
 * fundamental: the channel's delay modulo one symbol (whole symbols are
 * frame alignment, which TRN1u sets).  The 24.5T Su-bar (9.5.2.1.7) then
 * moves the analogue modem's timing T/2 later, and its TRN1u keeps that, so
 * this includes the half symbol: measured on the line taps of ESS and
 * SmartLink, the first Su's phase plus T/2 is the returned Su's to 0.03T and
 * the second TRN1u's to 0.03T.  (Measuring the returned Su itself would
 * mean holding Jd for it, and one held Jd repetition made SmartLink retrain.)
 * False until four blocks have been seen. */
bool v92_su_jp_phase(const v92_su_t *s, double *fraction, unsigned *blocks);
#endif
