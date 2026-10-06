/*
 * v92_p3_eq.h -- equaliser and symbol timing for the V.92 Phase 3 upstream
 * (TRN1u and what follows it) on the digital side.
 *
 * V.92's Phase 3 upstream is linear PCM at the full 8 kHz symbol rate, one
 * symbol per DS0 sample.  On a byte-exact DS0 every codeword arrives as
 * itself and the receiver can slice raw signs; behind a real 2-wire loop
 * and one codec the loop's ISI puts ~14% of TRN1u's raw signs wrong, and
 * the analogue modem's clock is not yet locked to the network's (V.92 6.2;
 * 163 ppm measured on the r4 recording).  See docs/v92_p3_rx_line_plan.md,
 * step 4.
 *
 * TRN1u (8.5.7) is known from its first symbol -- the GPA scrambler
 * zero-initialised and fed ones, 0 -> +L_U -- so the equaliser is trained
 * data-aided, never on its own decisions:
 *
 * Feed-forward taps on the received samples plus decision-feedback taps on
 * the past symbols.  The loop's ISI is what limits a linear equaliser here
 * (fixture, 600-symbol least-squares bound: 31 taps 11.4 dB, 31 + 8
 * feedback 17.6 dB), and TRN1u lets the feedback train on the true past
 * symbols before it ever has to trust its own.
 *
 *   1. a block least-squares solve over the first `seed_symbols` of TRN1u
 *      against the reference (the computation tools/v92_trn1u_bound.py
 *      does), so none of 2040T goes on LMS convergence;
 *   2. data-aided normalised LMS for the rest of the guaranteed TRN1u
 *      (`trn_symbols`, 9.5.1.1.3's 2040T);
 *   3. decision-directed after that, since nothing yet says how much
 *      longer TRN1u runs or where Ja begins.
 *
 * Timing: a windowed-sinc fractional interpolator in front of the
 * equaliser, at instants start + m + tau, with tau and its rate driven by a
 * second-order loop.  The detector is the data-aided gradient of the
 * squared error with respect to the instant, e*dy/dtau, with dy/dtau the
 * equaliser applied to the interpolated input's slope, normalised by the
 * running slope power so the loop gain does not depend on the channel.
 * Equaliser and timing then descend one cost instead of fighting.  Two
 * alternatives stay selectable because they were measured and lost
 * (docs/v92_p3_rx_line_plan.md step 4): Mueller and Muller on the equaliser
 * output, the pattern in v92_trn2u_demod_feed_adaptive(), reads exactly the
 * h(+/-T) the adapting equaliser drives to zero; and the tap-centroid servo
 * v92_analogue_audio.c uses moves only as fast as NLMS does.  The
 * interpolator only moves the observation instant; it never duplicates or
 * drops a DS0 sample.
 *
 * Input is linear 8 kHz samples; G.711 expansion is the caller's, since only
 * the caller knows the law.  Outputs are normalised so the reference is
 * +/-1.
 */
#ifndef V92_P3_EQ_H
#define V92_P3_EQ_H

#include <stdbool.h>
#include <stdint.h>

#define V92_P3_EQ_MAX_TAPS     63
#define V92_P3_EQ_MAX_FB       32     /* decision-feedback taps */
#define V92_P3_EQ_MAX_SEED     512
#define V92_P3_EQ_HISTORY      2048   /* input samples kept (power of two) */
#define V92_P3_EQ_INTERP_HALF  8      /* windowed-sinc half length */
#define V92_P3_EQ_AGREE_WINDOW 256

/* Timing detectors.  GRADIENT (default) is the normalised data-aided MSE
 * gradient.  CENTROID holds the taps' energy centroid where the seed put it;
 * MM is Mueller and Muller on the equaliser output.  Both lose -- see
 * docs/v92_p3_rx_line_plan.md step 4. */
#define V92_P3_EQ_DET_CENTROID 0
#define V92_P3_EQ_DET_MM       1
#define V92_P3_EQ_DET_GRADIENT 2

typedef struct {
    int ntaps;           /* feed-forward taps, odd; default 31 */
    int nfb;             /* decision-feedback taps; default 8, 0 = linear */
    int seed_symbols;    /* block LS length; default 256 */
    int trn_symbols;     /* data-aided length; default 2040 (9.5.1.1.3) */
    double mu;           /* NLMS step while data-aided */
    double mu_dd;        /* NLMS step once decision-directed */
    double timing_kp;    /* tau += kp*average M&M error, per symbol */
    double timing_ki;    /* frequency += ki*average M&M error, per symbol */
    double timing_avg;   /* M&M averaging coefficient (0.01 = ~100 symbols) */
    double max_ppm;      /* frequency bound; default 500 */
    bool timing;         /* run the timing loop at all */
    int detector;        /* V92_P3_EQ_DET_* */
} v92_p3_eq_config_t;

typedef struct {
    v92_p3_eq_config_t cfg;

    double x[V92_P3_EQ_HISTORY];   /* input, by absolute sample index */
    int64_t written;               /* absolute index of the next input */

    bool started;
    int64_t start;                 /* sample index of TRN1u symbol 0 */
    double window[2*V92_P3_EQ_INTERP_HALF];

    /* Interpolated, symbol-spaced stream z[m] at start - half + m + tau_m. */
    double zseed[V92_P3_EQ_MAX_SEED + V92_P3_EQ_MAX_TAPS];
    double z[V92_P3_EQ_MAX_TAPS];  /* most recent ntaps, z[0] oldest */
    double dzseed[V92_P3_EQ_MAX_SEED + V92_P3_EQ_MAX_TAPS];
    double dz[V92_P3_EQ_MAX_TAPS]; /* d z / d tau, alongside z */
    int64_t zcount;
    double tau;                    /* fractional timing, samples */
    double freq;                   /* samples per symbol; ppm*1e-6 */
    double mm_avg;                 /* averaged detector output */
    double centroid_ref;           /* centroid of the seeded taps */
    int main_tap;                  /* largest seeded tap */
    double dy2_avg;                /* mean (dy/dtau)^2, normalises GRADIENT */

    bool seeded;
    double taps[V92_P3_EQ_MAX_TAPS];
    double fb[V92_P3_EQ_MAX_FB];   /* on past symbols, newest first */
    double dhist[V92_P3_EQ_MAX_FB];/* past symbols, the reference while
                                      data-aided and decisions after, as
                                      +/-fb_scale */
    double fb_scale;               /* input RMS: keeps the feedback columns
                                      the size of the feed-forward ones */
    int64_t k;                     /* next symbol to produce */
    uint32_t gpa;                  /* reference generator */
    int64_t ref_from;              /* symbols [ref_from, ref_until) are */
    int64_t ref_until;             /* trained on the 8.5.7 reference */
    bool hold;                     /* no adaptation: taps and timing frozen,
                                      the learned frequency still applied */
    bool pam4;                     /* decide on V.92 Table 28's 4-point PAM
                                      (+/-1/sqrt5, +/-3/sqrt5) rather than
                                      +/-1: TRN2u, SUVu and CPu */

    double y_prev;
    double d_prev;
    bool have_prev;

    /* Per-symbol output of the last symbol produced. */
    double y;                      /* soft value, reference = +/-1 */
    int decision;                  /* +1 / -1 */
    int reference;                 /* +1 / -1 while data-aided, else 0 */

    /* Agreement with the reference over the last V92_P3_EQ_AGREE_WINDOW
     * data-aided symbols. */
    uint8_t agree_ring[V92_P3_EQ_AGREE_WINDOW];
    int agree_fill;
    int agree_pos;
    int agree_sum;

    /* Data-aided totals from symbol seed_symbols onward. */
    int64_t post_seed_symbols;
    int64_t post_seed_agree;
    double post_seed_err2;
} v92_p3_eq_t;

void v92_p3_eq_default_config(v92_p3_eq_config_t *cfg);
bool v92_p3_eq_init(v92_p3_eq_t *eq, const v92_p3_eq_config_t *cfg);

/* Append one linear 8 kHz sample.  Samples are numbered from 0 in the order
 * pushed; the caller pushes at least V92_P3_EQ_MAX_TAPS + INTERP_HALF before
 * the TRN1u start so the first symbols have their past. */
void v92_p3_eq_push(v92_p3_eq_t *eq, double sample);

/* TRN1u symbol 0 is input sample `start` (the index space of push). */
bool v92_p3_eq_start(v92_p3_eq_t *eq, int64_t start);

/* Produce the next symbol if its inputs have arrived.  Returns true and
 * fills eq->y / decision / reference when it did.  The first call that can
 * see seed_symbols of TRN1u solves the seed and produces symbol 0. */
bool v92_p3_eq_step(v92_p3_eq_t *eq);

/* Freeze (true) or release the taps and the timing loop.  Held, the
 * interpolator keeps advancing at the learned frequency, so a long interval
 * of something the equaliser must not train on (V.92 Phase 3's silences
 * and three-level Su) costs only the frequency error times its length. */
void v92_p3_eq_hold(v92_p3_eq_t *eq, bool hold);
/* Decision-directed on 4-point PAM from here on (Phase 4): decisions,
 * decision feedback, tap and timing adaptation all use the 4-level slicer.
 * Two-level decisions on a four-level signal corrupt every feedback term. */
void v92_p3_eq_set_pam4(v92_p3_eq_t *eq, bool pam4);

/* Train data-aided again, on a TRN1u whose first symbol is eq symbol k0
 * (9.5.1.1.10's second TRN1u is zero-initialised like the first), for n
 * symbols; releases the hold.  k0 may already be past -- the reference is
 * advanced to the symbol produced next.  Agreement statistics restart. */
void v92_p3_eq_train_from(v92_p3_eq_t *eq, int64_t k0, int n);

/* The 8.5.7 TRN1u reference, +1/-1, for n symbols from its first. */
void v92_p3_eq_reference(int8_t *ref, int n);

/* Percentage (x10) of sign agreement over the last 256 data-aided symbols. */
int v92_p3_eq_agree_x10(const v92_p3_eq_t *eq);
/* Equalised SNR in dB over data-aided symbols from seed_symbols onward. */
double v92_p3_eq_snr_db(const v92_p3_eq_t *eq);
/* Sign agreement fraction over data-aided symbols from seed_symbols on. */
double v92_p3_eq_post_seed_agreement(const v92_p3_eq_t *eq);
/* Index of the largest seeded tap, 0..ntaps-1; ntaps/2 is the centre.  A
 * least-squares fit absorbs a start that is a few symbols out by moving
 * this, so it is what says whether the reference is aligned. */
int v92_p3_eq_main_tap(const v92_p3_eq_t *eq);
/* Frequency estimate in ppm (positive: the input runs fast). */
double v92_p3_eq_ppm(const v92_p3_eq_t *eq);

#endif
