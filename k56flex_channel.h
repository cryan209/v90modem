/* Simulated analogue loop for exercising the linear-audio client: what lies between the
 * network D/A converter and a client modem's codec.  Test support, not a model of any real
 * line: gain (the pad), a fractional delay, a low-pass and a high-pass, DC, white noise and
 * a client sample clock that differs from the network's by a few ppm. */
#ifndef K56FLEX_CHANNEL_H
#define K56FLEX_CHANNEL_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    double gain;            /* linear, 1.0 = unity */
    double delay;           /* extra delay in samples (fractional part honoured) */
    double lowpass_hz;      /* two cascaded RC sections with this corner, 0 = off */
    double highpass_hz;     /* 1st-order, 0 = off */
    double dc;              /* added offset */
    double noise_rms;       /* white Gaussian, in output units */
    double clock_ppm;       /* + : the client's clock is fast (more samples per second) */
    unsigned seed;
} k56flex_channel_cfg_t;

typedef struct k56flex_channel k56flex_channel_t;

k56flex_channel_t *k56flex_channel_new(const k56flex_channel_cfg_t *cfg);
void k56flex_channel_free(k56flex_channel_t *ch);
/* Push one network-rate linear sample; returns how many client-rate samples were produced
 * (0 or 1, occasionally 0/2 over time with a clock offset), written to out[]. */
unsigned k56flex_channel_push(k56flex_channel_t *ch, double in, int16_t out[2]);

#endif
