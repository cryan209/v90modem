/* V.92 §§3.1–3.2, 6.2: oversampled analogue audio, separate from the DS0.
 * 16 kHz signed 16-bit PCM; two audio samples per network symbol. One PCM
 * unit represents four of the calibrated core's linear units (12 dB of
 * reconstruction headroom). The adaptive clock/equalizer path is experimental;
 * complete analogue audio startup has not yet been validated.
 * No G.711 encoder or decoder belongs in this endpoint.
 */
#ifndef V92_ANALOGUE_AUDIO_H
#define V92_ANALOGUE_AUDIO_H
#include "v92_analogue_phase3.h"

#define V92_AUDIO_RATE 16000
#define V92_AUDIO_PER_SYMBOL 2
#define V92_AUDIO_LINEAR_SCALE 4
#define V92_AUDIO_INTERPOLATOR_TAPS 16

/* Finite windowed-sinc reconstruction, also usable by a test network DAC.
 * Delay: eight INPUT samples. Each push writes exactly factor output samples.
 * Factors 1 through 6 are supported; the 16 kHz network DAC uses 2.
 * This implementation filter is not an ITU-mandated pulse shape.
 */
typedef struct {
    double history[V92_AUDIO_INTERPOLATOR_TAPS];
    double coefficients[6][V92_AUDIO_INTERPOLATOR_TAPS];
    int factor;
    uint64_t clipped;
} v92_pcm_interpolator_t;
bool v92_pcm_interpolator_init(v92_pcm_interpolator_t *s, int factor);
int v92_pcm_interpolator_put(v92_pcm_interpolator_t *s, int16_t input,
                             int16_t *output);

typedef struct v92a_audio_s v92a_audio_t;
/* 16 kHz default. rx_phase is a legacy argument, accepted in 0..1;
 * acquisition currently determines the sampling phase itself.
 * The endpoint owns its protocol core. Do not drive that core's audio APIs
 * separately; the accessor is for status and payload configuration only. */
v92a_audio_t *v92a_audio_init(const v92a_config_t *cfg, unsigned rx_phase);
/* Explicit alternate audio rate for experiments (8000..48000 Hz). */
v92a_audio_t *v92a_audio_init_rate(const v92a_config_t *cfg, unsigned rate);
void v92a_audio_free(v92a_audio_t *s);
v92a_t *v92a_audio_core(v92a_audio_t *s);
int v92a_audio_tx(v92a_audio_t *s, int16_t *samples, int count);
void v92a_audio_rx(v92a_audio_t *s, const int16_t *samples, int count);
uint64_t v92a_audio_clipped(const v92a_audio_t *s);
bool v92a_audio_acquired(const v92a_audio_t *s);
double v92a_audio_clock_ppm(const v92a_audio_t *s);
double v92a_audio_eq_error(const v92a_audio_t *s);
#endif
