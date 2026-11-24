/*
 * v92_prefilter.h -- V.92 6.4.2 prefilter design for the PCM upstream.
 *
 * Role: from TRN2u (8.5.6, which the analogue modem sends through neither
 * its precoder nor its prefilter), fit the channel from the analogue
 * modem's transmitter to the network A/D, then choose the prefilter z2
 * that the CPd (Table 30) gives the analogue modem so that the A/D input
 * is the point itself, one symbol per sample, at a fixed delay.
 *
 * Without it the A/D quantises the channel's intersymbol interference:
 * G.711's step noise is then on every received sample, and no receiver
 * equaliser can remove it afterwards.  With it the points are G.711
 * levels where the A/D samples them, and what is left is the line's own
 * noise and the interference the prefilter could not reach.
 *
 * Units: the channel maps a transmitted signal of unit mean square (the
 * power LU stands for, V.92 3.8 and Table 30) to received DS0 linear
 * values.  The prefilter is normalised to unit energy, so it leaves the
 * transmitted power where Table 30's G puts it.
 */
#ifndef V92_PREFILTER_H
#define V92_PREFILTER_H

#include <stdbool.h>

#define V92_PF_CH_PRE   24                     /* channel taps before the main */
#define V92_PF_CH_POST  40                     /* and after it */
#define V92_PF_CH       (V92_PF_CH_PRE + 1 + V92_PF_CH_POST)
#define V92_PF_MAX_W    128                    /* prefilter taps (Table 30) */
#define V92_PF_MAX_B    64                     /* precoder taps left free */

typedef struct {
    double h[V92_PF_CH];   /* r(n) = sum_j h[j] t(n + V92_PF_CH_PRE - j) */
    double fit_err2;       /* mean square of the fit's error, DS0^2 */
    double quant2;         /* the part G.711's step noise explains, DS0^2 */
    double noise2;         /* the rest: the line's noise, DS0^2 */
    double power;          /* sum h^2: the received power of unit transmit */
    int rows;
} v92_pf_channel_t;

typedef struct {
    int lw;                       /* prefilter taps */
    int lb;                       /* precoder taps after the main one */
    int delay;                    /* d: the main response tap, in symbols
                                     after the channel's first fitted tap */
    double w[V92_PF_MAX_W];       /* unit energy */
    double b[V92_PF_MAX_B];       /* (h*w)[d+k]/(h*w)[d], k = 1..lb */
    double gain;                  /* (h*w)[d], DS0 per unit transmit */
    double interference;          /* sum of the other (h*w)^2 / gain^2 */
    double snr_db;                /* gain^2 / (noise2 + interference gain^2) */
    double identity_snr_db;       /* the same figure for no prefilter */
} v92_pf_design_t;

/* Least squares: r[k], the received sample, against t[k + V92_PF_CH_PRE -
 * j], the transmitted TRN2u symbols (unit mean square), j = 0 ..
 * V92_PF_CH - 1.  t must hold n + V92_PF_CH - 1 symbols: t[0] is the
 * symbol V92_PF_CH_POST before the one r[0] is centred on.  levels, when
 * not NULL, lists the law's 128 positive linear levels in ascending order
 * for the quantisation estimate. */
bool v92_pf_fit_channel(v92_pf_channel_t *ch, const double *r, const double *t,
                        int n, const double *levels);

/* The prefilter of lw taps (and lb free precoder taps after the main one)
 * that maximises the main tap's SNR against noise2 and the interference
 * left, over every delay the taps allow. */
bool v92_pf_design(v92_pf_design_t *out, const v92_pf_channel_t *ch,
                   int lw, int lb);

#endif
