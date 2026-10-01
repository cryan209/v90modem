/*
 * v92_line_channel.h -- a test model of the analogue path between a V.92
 * analogue modem's transmitter and the network A/D.
 *
 * Every V.92 receive test used to be fed a byte-exact DS0, which hides the
 * three things a real 2-wire loop does to the upstream: intersymbol
 * interference, a fractional sampling phase at the network A/D, and a clock
 * offset between the analogue modem and the network (V.92 6.2 puts the
 * upstream symbol clock on the network's, which an analogue modem only
 * approximates before it has locked).  This models them, plus additive
 * noise, so the digital side's Phase 3 receiver can be tested against them
 * offline (docs/v92_p3_rx_line_plan.md, step 2).
 *
 * Input is the analogue modem's 16 kHz audio (V92_AUDIO_PER_SYMBOL samples per
 * symbol) in the calibrated core's linear units -- the value the harness's
 * network ADC quantises.  Output is the 8 kHz sequence that ADC samples, still
 * linear; G.711 quantisation stays with the caller, once.
 *
 * Test code only.  It is not in SRCS and nothing in a live call uses it.
 */
#ifndef V92_LINE_CHANNEL_H
#define V92_LINE_CHANNEL_H

#include <stdbool.h>
#include <stdint.h>

#define V92_LINE_CHANNEL_HISTORY 4096   /* 16 kHz samples kept */
#define V92_LINE_CHANNEL_INTERP_HALF 16 /* windowed-sinc half length, 16 kHz */

typedef struct {
    /* FIR at 16 kHz applied to the analogue audio; NULL or one tap of 1.0
     * is no ISI. */
    const double *taps;
    int ntaps;
    /* Network A/D sampling instant, in 8 kHz samples, 0 <= phase < 1.
     * 0 samples exactly where the ideal harness does. */
    double phase;
    /* Network A/D clock relative to the analogue modem's, in ppm.  Positive
     * means the A/D runs fast and takes more samples per modem symbol. */
    double ppm;
    /* Additive white Gaussian noise at the A/D input, linear units RMS.
     * Zero adds none. */
    double noise_rms;
    uint32_t seed;
    /* The network A/D's anti-alias filter (G.712 passband to 3.4 kHz):
     * a 129-tap windowed-sinc low-pass at 16 kHz, cut-off 3.7 kHz, delay
     * exactly 64 samples = 32 symbols.  The analogue modem's 16 kHz output
     * keeps ~18% of TRN1u's energy above 4 kHz (its reconstruction images),
     * which a synchronous A/D folds into a FIXED linear map an equaliser
     * absorbs, but which a fractional phase or a clock offset folds into
     * something no receiver can undo.  The r4 taps already contain the real
     * codec's filter; rows that move the sampling instant without them need
     * this, or they model an A/D that does not exist. */
    bool antialias;
} v92_line_channel_config_t;

#define V92_LINE_CHANNEL_AA_TAPS 129
#define V92_LINE_CHANNEL_AA_DELAY_SYMBOLS 32

typedef struct {
    v92_line_channel_config_t cfg;
    bool ideal;               /* bit-exact pass-through of the even samples */
    double fir_hist[256];
    int fir_pos;
    double aa_taps[V92_LINE_CHANNEL_AA_TAPS];
    double aa_hist[256];
    int aa_pos;
    double ring[V92_LINE_CHANNEL_HISTORY];
    uint64_t written;         /* filtered 16 kHz samples produced so far */
    double t;                 /* next A/D instant, 16 kHz sample units */
    double step;              /* 16 kHz samples per A/D sample */
    double window[2*V92_LINE_CHANNEL_INTERP_HALF];
    uint32_t rng;
    bool have_spare;
    double spare;
} v92_line_channel_t;

/* False if the configuration is out of range (more than 256 taps, phase
 * outside [0,1), |ppm| above 5000, negative noise). */
bool v92_line_channel_init(v92_line_channel_t *c,
                           const v92_line_channel_config_t *cfg);

/* Push n 16 kHz input samples; writes up to max 8 kHz A/D samples to out and
 * returns how many.  With an ideal configuration the output is exactly every
 * other input sample, starting with the first, as the harness's network ADC
 * has always taken them. */
int v92_line_channel_put(v92_line_channel_t *c, const double *in, int n,
                         double *out, int max);

/* The r4 loop channel (artifacts/v92-loop-upstream), fitted by
 * tools/v92_fit_line_channel.py; see v92_line_channel_r4.h. */
extern const double v92_line_channel_r4_taps[];
extern const int v92_line_channel_r4_ntaps;

#endif
