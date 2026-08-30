/*
 * v34rx_data.c - ITU-T V.34 receive, data mode (V34_RX_STAGE_DATA).
 *
 * Lifted verbatim out of process_primary_symbol() in v34rx.c, where it was
 * 807 lines of a 4,900-line function.  It took nothing from the enclosing
 * switch -- every one of that function's locals is unused here -- so the
 * split is a straight move, and the arithmetic is unchanged.
 *
 * What lives here:
 *   - B1 acquisition scoring and the residual carrier frequency taken from
 *     B1's two halves (10.1.3.1), which is what makes rates above 12000
 *     bit/s decode at all;
 *   - mapping-frame collection, the decision-aided derotator and the
 *     lattice slice;
 *   - decision-directed LMS on the data constellation, gated on the
 *     receiver still being healthy;
 *   - second-order decision-directed carrier tracking;
 *   - the distance-to-grid, receive-SNR and shell-index instrumentation.
 *
 * Two things in the instrumentation are NOT diagnostics and must not be
 * dropped with it: v90_t3_sym_err_ema and v90_t3_sym_err_fast gate the
 * timing loop, the LMS and the carrier loop, and the divergence branch
 * (V34_V90_T3_DIVERGED_POWER) stands CMA down and restores the last good
 * equalizer taps.
 */

#if defined(HAVE_CONFIG_H)
#include "config.h"
#endif

#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#if defined(HAVE_TGMATH_H)
#include <tgmath.h>
#endif
#if defined(HAVE_MATH_H)
#include <math.h>
#endif
#if defined(HAVE_STDBOOL_H)
#include <stdbool.h>
#else
#include "spandsp/stdbool.h"
#endif
#include "floating_fudge.h"

#include "spandsp/telephony.h"
#include "spandsp/alloc.h"
#include "spandsp/fast_convert.h"
#include "spandsp/logging.h"
#include "spandsp/bit_operations.h"
#include "spandsp/bitstream.h"
#include "spandsp/complex.h"
#include "spandsp/vector_float.h"
#include "spandsp/complex_vector_float.h"
#include "spandsp/vector_int.h"
#include "spandsp/complex_vector_int.h"
#include "spandsp/modem_echo.h"
#include "spandsp/async.h"
#include "spandsp/power_meter.h"
#include "spandsp/arctan2.h"
#include "spandsp/dds.h"
#include "spandsp/crc.h"
#include "spandsp/complex_filters.h"

#include "spandsp/v29rx.h"
#include "spandsp/v34.h"

#include "spandsp/private/bitstream.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/modem_echo.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/v34.h"

#include "v34_tables.h"
#include "v34rx_internal.h"

/* ME_V90_DATA_LEAN=1 drops the measurement this stage carries and keeps only
   what steers the receiver -- the shape an embedded build of this file needs.
   Gated here rather than deleted so the two can be run against each other on
   the same recording, which is the only way to know that a block that LOOKS
   like instrumentation is not quietly load-bearing.  Two in this file are:
   v90_t3_sym_err_ema / _fast gate the timing loop, the DD-LMS and the carrier
   loop, and the V34_V90_T3_DIVERGED_POWER branch stands CMA down and restores
   the last good taps.  Neither is gated. */
static int v34_rx_data_lean(void)
{
    static int cache = -1;

    if (cache < 0)
    {
        const char *v = getenv("ME_V90_DATA_LEAN");

        cache = (v && *v) ? (atoi(v) != 0) : 0;
    }
    /*endif*/
    return cache;
}
/*- End of function --------------------------------------------------------*/

void v34_rx_data_symbol(v34_rx_state_t *s, const complexf_t *sym)
{
        if (s->b1_acquisition_active)
        {
            int n = s->b1_observed_symbols++;

            if (n < s->v90_t3_b1_symbols)
                s->b1_observed[n] = *sym;
            if (s->b1_observed_symbols >= s->v90_t3_b1_symbols)
            {
                complexf_t corr = complex_setf(0.0f, 0.0f);
                complexf_t corr_conj = complex_setf(0.0f, 0.0f);
                float expected_power = 0.0f;
                float observed_power = 0.0f;
                float corr_mag2;
                float corr_conj_mag2;
                bool conjugate;
                float phase;
                float gain;

                for (int i = 0; i < s->v90_t3_b1_symbols; i++)
                {
                    complexf_t o = s->b1_observed[i];
                    complexf_t e = s->v90_t3_b1[i];

                    corr.re += o.re*e.re + o.im*e.im;
                    corr.im += o.im*e.re - o.re*e.im;
                    corr_conj.re += o.re*e.re - o.im*e.im;
                    corr_conj.im += o.im*e.re + o.re*e.im;
                    expected_power += e.re*e.re + e.im*e.im;
                    observed_power += o.re*o.re + o.im*o.im;
                }
                corr_mag2 = corr.re*corr.re + corr.im*corr.im;
                corr_conj_mag2 = corr_conj.re*corr_conj.re
                               + corr_conj.im*corr_conj.im;
                conjugate = corr_conj_mag2 > corr_mag2;
                if (conjugate)
                    corr = corr_conj;
                gain = sqrtf(corr.re*corr.re + corr.im*corr.im)
                     / expected_power;
                phase = atan2f(corr.im, corr.re);
                s->phase4_da_derot = (int32_t)
                    (phase*2147483648.0f/3.14159265358979f);
                /* B1 also gives the residual carrier FREQUENCY, which the
                   phase term cannot.  Training leaves about 0.7 Hz of it in
                   this receiver -- 0.105 degrees per symbol at 2400 baud,
                   measured against the transmitter's own symbols over a
                   bit-exact loopback -- and nothing in data mode removes it
                   except the decision-directed loop below.  That loop can
                   hold a dense constellation but cannot acquire one: at 9600
                   it pulls the offset out and the drift measures zero, while
                   at 14400 and above the first decisions are already wrong and
                   the phase runs away (12748 degrees over one call), which is
                   why every rate with uncoded Q bits produced white output.
                   Correlating the two halves of B1 separately gives the
                   advance per symbol directly: the sequence is known, so this
                   owes nothing to a decision. */
                {
                    /* Correlate the two halves of B1 separately and take the
                       angle between them: the advance over half the sequence.
                       Averaging coherently inside each half first is what makes
                       this work at 96 symbols -- the obvious one-lag
                       autocorrelation over all of them was tried and reads
                       0.91 degrees per symbol where the truth is 0.105, because
                       it never averages the noise down before taking an angle. */
                    complexf_t c0 = complex_setf(0.0f, 0.0f);
                    complexf_t c1 = complex_setf(0.0f, 0.0f);
                    int half = s->v90_t3_b1_symbols/2;

                    for (int i = 0; i < s->v90_t3_b1_symbols; i++)
                    {
                        complexf_t o = s->b1_observed[i];
                        complexf_t e = s->v90_t3_b1[i];
                        float pr = o.re*e.re + (conjugate ? -o.im*e.im : o.im*e.im);
                        float pi = (conjugate ? -o.im : o.im)*e.re - o.re*e.im;
                        complexf_t *acc = (i < half) ? &c0 : &c1;

                        acc->re += pr;
                        acc->im += pi;
                    }
                    /*endfor*/
                    if (half > 0
                        &&
                        (c0.re*c0.re + c0.im*c0.im) > 0.0f
                        &&
                        (c1.re*c1.re + c1.im*c1.im) > 0.0f)
                    {
                        float d_re = c1.re*c0.re + c1.im*c0.im;
                        float d_im = c1.im*c0.re - c1.re*c0.im;
                        float dphi = atan2f(d_im, d_re)/(float) half;

                        /* Half a turn per symbol is not a carrier offset, it is
                           a wrapped measurement; refuse anything that large. */
                        if (fabsf(dphi) < 0.2f)
                        {
                            s->phase4_da_derot_rate = (int32_t)
                                (dphi*2147483648.0f/3.14159265358979f);
                        }
                        else
                        {
                            s->phase4_da_derot_rate = 0;
                        }
                        /*endif*/
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - B1 residual carrier: %.4f deg/symbol "
                                 "(%.2f Hz at this rate)%s\n",
                                 dphi*180.0f/3.14159265f,
                                 dphi*180.0f/3.14159265f
                                   *baud_rate_parameters[s->baud_rate].baud_rate/360.0f,
                                 (fabsf(dphi) < 0.2f) ? "" : " - rejected as wrapped");
                    }
                    else
                    {
                        s->phase4_da_derot_rate = 0;
                    }
                    /*endif*/
                }
                s->data_decision_ema = 0.0f;
                s->data_decision_baseline = 0.0f;
                s->data_decision_count = 0;
                s->data_symbol_conjugate = conjugate;
                s->data_symbol_rotation = 0;
                s->data_symbol_scale = (gain > 0.0001f) ? 1.0f/gain : 1.0f;
                {
                    /* Tap energy is the equalizer's noise gain.  A converged
                       FSE that is merely matched sits near 1; one that is
                       inverting a band edge sits far above it and multiplies
                       whatever the bearer put in -- which is the difference
                       between the 37 dB mu-law actually delivers at this level
                       and the SNR the symbols show. */
                    float te = 0.0f;
                    int i;

                    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
                        te += s->eq_coeff[i].re*s->eq_coeff[i].re
                            + s->eq_coeff[i].im*s->eq_coeff[i].im;
                    /*endfor*/
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - DATA entry: equalizer tap energy %.4f\n", te);
                }
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - B1 acquired: symbols=%d phase=%.2f deg gain=%.4f "
                         "conjugate=%d normalized-correlation=%.3f\n",
                         s->v90_t3_b1_symbols, phase*180.0f/3.14159265f,
                         gain, conjugate,
                         sqrtf((conjugate ? corr_conj_mag2 : corr_mag2)
                               /(expected_power*observed_power)));
                s->mapping_frame_count = 0;
                for (int i = 0; i < s->v90_t3_b1_symbols; i++)
                {
                    complexf_t o = s->b1_observed[i];
                    float c = cosf(phase);
                    float sn = sinf(phase);
                    float re = (o.re*c + o.im*sn)*s->data_symbol_scale;
                    float im = (o.im*c - o.re*sn)*s->data_symbol_scale;

                    if (conjugate)
                        im = -im;
                    s->mapping_frame_buf[s->mapping_frame_count++] =
                        (int16_t)(re*128.0f);
                    s->mapping_frame_buf[s->mapping_frame_count++] =
                        (int16_t)(im*128.0f);
                    if (s->mapping_frame_count >= 16)
                    {
                        v34_put_mapping_frame(s, s->mapping_frame_buf);
                        s->mapping_frame_count = 0;
                    }
                }
                s->b1_acquisition_active = false;
            }
            s->duration++;
            s->last_sample = *sym;
            /* Was a `break` out of the enclosing switch. */
            return;
        }
        /* V.34 data mode: collect equalized symbols into mapping frames (8 x 2D symbols)
           and run the full decode pipeline.
           The CMA equalizer (frozen from training) normalizes to unit magnitude.
           Training DQPSK is constant modulus, so its frozen equalizer does not
           establish the scale or arbitrary phase of the negotiated DATA grid.
           The complete known B1 frame above supplies that calibration from this
           call's waveform before payload reaches v34_rx_quantize_n_ways(). */
        {
            float re;
            float im;
            float transformed_re;
            float transformed_im;

            /* Take out the decision-aided derotator acquired over the CP/MP
               stretch: this is what makes the symbols coherent.  CMA (the
               wander source) is frozen in DATA, so from here only genuine
               carrier drift remains, held by the DD tracker below. */
            {
                float c;
                float sn;

                /* Carry the B1-measured residual carrier forward.  Without
                   this the derotator is a constant and the offset it cannot
                   see accumulates without limit. */
                s->phase4_da_derot += (uint32_t) s->phase4_da_derot_rate;
                c = cosf((float) s->phase4_da_derot
                         *(float) (3.14159265358979/2147483648.0));
                sn = sinf((float) s->phase4_da_derot
                          *(float) (3.14159265358979/2147483648.0));
                float dr = sym->re*c + sym->im*sn;
                float di = sym->im*c - sym->re*sn;

                re = dr;
                im = s->data_symbol_conjugate ? -di : di;
            }
            switch (s->data_symbol_rotation & 3)
            {
            case 1:
                transformed_re = -im;
                transformed_im = re;
                break;
            case 2:
                transformed_re = -re;
                transformed_im = -im;
                break;
            case 3:
                transformed_re = im;
                transformed_im = -re;
                break;
            default:
                transformed_re = re;
                transformed_im = im;
                break;
            }
            /*endswitch*/
            s->mapping_frame_buf[s->mapping_frame_count++] =
                (int16_t)(transformed_re * 128.0f * s->data_symbol_scale);
            s->mapping_frame_buf[s->mapping_frame_count++] =
                (int16_t)(transformed_im * 128.0f * s->data_symbol_scale);

            /* Decision-directed LMS on the data constellation.
               Nothing adapted the equalizer in data mode at all: CMA stops in
               Phase 4 and the taps it leaves are wrong by about 5% on the
               symbols either side of the main one.  Measured against the
               transmitter's own symbols over a bit-exact loopback, a linear
               filter on the TRUE symbols removes 79% of the residual power,
               so it is ISI and not noise -- and 9.x's dense constellations are
               where that matters: the same taps that cost 9600 nothing put
               21600's symbols (RMS ~20 on a grid of spacing 2) a whole
               decision region away from where they belong.
               The decision is taken in the grid domain and mapped back
               through the derotator into the equalizer's own domain, which is
               where the taps live.  ME_V34_DATA_EQ=0 disables. */
            if (v34_rx_t2_data_path(s)  &&  v34_rx_data_mode_eq_enabled())
            {
                float g_re = transformed_re*s->data_symbol_scale;
                float g_im = transformed_im*s->data_symbol_scale;
                float t_re = 2.0f*floorf(g_re/2.0f) + 1.0f;
                float t_im = 2.0f*floorf(g_im/2.0f) + 1.0f;
                float d2 = (g_re - t_re)*(g_re - t_re)
                         + (g_im - t_im)*(g_im - t_im);
                bool healthy;

                /* How well the receiver is doing, and how well it was doing
                   once B1 had settled it.  The absolute value means nothing on
                   its own -- it depends entirely on how dense the constellation
                   is -- so the baseline is measured on this call rather than
                   assumed. */
                s->data_decision_ema += (d2 - s->data_decision_ema)/256.0f;
                if (s->data_decision_count < 4096)
                {
                    s->data_decision_count++;
                    s->data_decision_baseline = s->data_decision_ema;
                    healthy = true;
                }
                else
                {
                    healthy = (s->data_decision_ema
                               < 2.0f*s->data_decision_baseline + 0.02f);
                }
                /*endif*/
                /* Adapt only on decisions that are probably right, and only
                   while the receiver as a whole still is.  Without the second
                   test this loop is a ratchet: measured live at 3000 baud/9600,
                   a call sitting at 0.10 from the lattice took one disturbance
                   and went to 0.66 for the rest of the call, delivering 73 of
                   300 pattern lines, where the same call with data-mode
                   adaptation off held 0.10 throughout and delivered all 300.
                   Freezing on the way down leaves exactly that frozen-tap
                   behaviour as the worst case. */
                if (healthy
                    &&  d2 < v34_rx_data_mode_decision_gate()
                    &&  s->data_symbol_scale > 0.0f)
                {
                    complexf_t da_target;
                    float ct = cosf((float) s->phase4_da_derot
                                    *(float) (3.14159265358979/2147483648.0));
                    float st = sinf((float) s->phase4_da_derot
                                    *(float) (3.14159265358979/2147483648.0));
                    float ur = t_re/s->data_symbol_scale;
                    float ui = (s->data_symbol_conjugate ? -t_im : t_im)
                             / s->data_symbol_scale;

                    /* Undo the zero-delay derotator: the taps see the symbol
                       before it. */
                    da_target.re = ur*ct - ui*st;
                    da_target.im = ui*ct + ur*st;
                    {
                        float saved = s->eq_delta;

                        s->eq_delta *= v34_rx_data_mode_eq_step();
                        v34_rx_tune_equalizer(s, sym, &da_target);
                        s->eq_delta = saved;
                    }
                }
                /*endif*/
            }
            /*endif*/

            /* The equalizer's own frequency response, so the question "is
               this receiver enhancing noise at the band edges?" can be asked
               of the taps rather than guessed at.  V34_EQ_TAP_DUMP names a
               file; one line of 2*(PRE+1+POST) floats per 4096 symbols. */
            {
                static const char *tap_path = NULL;
                static int tap_count[2];
                int who_t = s->calling_party ? 1 : 0;

                if (tap_path == NULL)
                    tap_path = (getenv("V34_EQ_TAP_DUMP") && !v34_rx_data_lean())
                              ? getenv("V34_EQ_TAP_DUMP") : "";
                /*endif*/
                if (tap_path[0]  &&  ++tap_count[who_t] % 4096 == 0)
                {
                    FILE *tf = fopen(tap_path, "a");

                    if (tf)
                    {
                        int ti;

                        fprintf(tf, "%s %d", who_t ? "caller" : "answer", tap_count[who_t]);
                        for (ti = 0;  ti < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  ti++)
                            fprintf(tf, " %.6g %.6g", s->eq_coeff[ti].re, s->eq_coeff[ti].im);
                        /*endfor*/
                        fprintf(tf, "\n");
                        fclose(tf);
                    }
                    /*endif*/
                }
                /*endif*/
            }

            /* Plain V.34 runs none of the V.90 upstream lock machinery, so it
               had no read at all on its own data mode.  Distance to the grid
               says whether the waveform arrives decodable -- 9.x puts every
               constellation point on odd integers, so a receiver that is
               tracking reads a small residual and one producing white output
               reads about two thirds.  Read it against the shell-index check
               in v34_rx_pack_output_bitstream(): grid small + shell 0% is a working
               data mode, grid small + shell high is correct symbols grouped
               wrongly, and grid large is a fault before the mapper. */
            if (v34_rx_t2_data_path(s))
            {
                int who = s->calling_party ? 1 : 0;
                static double err_sum[2];
                static double sig_sum[2];
                static int err_count[2];
                float g_re = transformed_re*s->data_symbol_scale;
                float g_im = transformed_im*s->data_symbol_scale;
                float t_re = 2.0f*floorf(g_re/2.0f) + 1.0f;
                float t_im = 2.0f*floorf(g_im/2.0f) + 1.0f;

                if (!v34_rx_data_lean())
                {
                    err_sum[who] += (g_re - t_re)*(g_re - t_re)
                                  + (g_im - t_im)*(g_im - t_im);
                    sig_sum[who] += t_re*t_re + t_im*t_im;
                    err_count[who]++;
                }
                /*endif*/
                /* The same distance, kept on the state as a running estimate
                   so the engine can act on it rather than only read it in a
                   log line.  V.34 11.5 and 11.6 both exist for a receiver
                   that has stopped decoding, and neither can be reached
                   without first noticing that it has. */
                {
                    float d = (g_re - t_re)*(g_re - t_re)
                            + (g_im - t_im)*(g_im - t_im);

                    s->data_grid_err_ema += (d - s->data_grid_err_ema)/256.0f;
                    if (s->data_grid_symbols < V34_DATA_LOST_SETTLE_SYMBOLS)
                    {
                        /* Give the EMA and the receiver time to settle after
                           B1 before any of this counts. */
                        s->data_grid_symbols++;
                        s->data_lost_run = 0;
                    }
                    else if (s->data_grid_err_ema >= V34_DATA_LOST_ERR)
                    {
                        if (++s->data_lost_run == V34_DATA_LOST_SYMBOLS)
                        {
                            span_log(s->logging, SPAN_LOG_WARNING,
                                     "Rx - V.34 data mode has stopped "
                                     "decoding: %.3f from the grid for %d "
                                     "symbols (2/3 is white)\n",
                                     s->data_grid_err_ema, s->data_lost_run);
                        }
                        /*endif*/
                    }
                    else
                    {
                        s->data_lost_run = 0;
                    }
                    /*endif*/
                }
                if (err_count[who] >= 4096)
                {
                    double dist = err_sum[who]/err_count[who];
                    double power = sig_sum[who]/err_count[who];

                    /* The same numbers as a rate, which is the only form in
                       which they answer the question that matters.  The mean
                       squared distance is scale-free against the lattice, so
                       distance against the mean symbol power is the receive
                       SNR of the constellation actually in use, and 6 dB per
                       bit per symbol is the slope every QAM ladder has.

                       This is the only measurement in the call that sees what
                       the data mode sees.  The Phase-2 L1/L2 probe cannot:
                       measured against this rig it reported no detectable
                       noise (its empty bins read zero, the line adds none)
                       while the data mode that followed ran at 18 dB, because
                       the impairment is signal-proportional and a 21-tone
                       probe on a 150 Hz grid does not excite it.  The Phase-4
                       TRN segment cannot either -- see
                       v34_phase4_trn_measured_rate_n().

                       Live ladder against the SmartLink rig at 3000 baud, all
                       three calls kept under artifacts/v34-rate-*: 14400
                       decodes and carries payload at 0.32-0.39 from the
                       lattice, 16800 is white (0.63-0.68) from its first
                       window, 21600 loses the call.  The offset below is set
                       so that an 18 dB line accepts 14400 and refuses 16800;
                       treat it as calibrated on one channel, not derived. */
                    if (dist > 0.0  &&  power > 0.0)
                    {
                        double snr = 10.0*log10(power/dist);
                        double bits = (snr + V34_DATA_SNR_RATE_OFFSET_DB)
                                    / V34_DATA_SNR_RATE_SLOPE_DB;
                        double baud = 2400.0
                                    *baud_rate_parameters[s->baud_rate].a
                                    /baud_rate_parameters[s->baud_rate].c;
                        int carries = ((int) floor(bits*baud/2400.0))*2400;

                        if (carries < 2400)
                            carries = 2400;
                        /*endif*/
                        if (carries > 33600)
                            carries = 33600;
                        /*endif*/
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - DATA: distance to grid %.4f per symbol "
                                 "over %d symbols; receive SNR %.1f dB, this "
                                 "line will carry %d bit/s%s\n",
                                 dist, err_count[who], snr, carries,
                                 (dist > 0.55)
                                 ? " -- OUTPUT IS WHITE, the requested rate is"
                                   " above what this line carries"
                                 : "");
                    }
                    else
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - DATA: distance to grid %.4f per symbol over %d symbols\n",
                                 dist, err_count[who]);
                    }
                    /*endif*/
                    err_sum[who] = 0.0;
                    sig_sum[who] = 0.0;
                    err_count[who] = 0;
                }
                /*endif*/
            }
            /*endif*/

            /* How far the data-mode symbols actually land from the grid.
               V.34 9.x places every constellation point on odd integers, so
               a receiver that is decoding has a small residual here and one
               that is producing white output has a large one.  This is the
               only direct read on whether an upstream failure is in the
               waveform or after it. */
            if (s->v90_t3_acquired  &&  s->v90_t3_suppress_output
                &&  !v34_rx_data_lean())
            {
                /* Same measure over the B1 era, where the symbols are known
                   to match their template.  If B1 is on the lattice and the
                   data that follows is not, whatever changes does so exactly
                   at that boundary -- which is a statement about the signal,
                   not about our filter. */
                float g_re = transformed_re*s->data_symbol_scale;
                float g_im = transformed_im*s->data_symbol_scale;
                float t_re = 2.0f*floorf(g_re/2.0f) + 1.0f;
                float t_im = 2.0f*floorf(g_im/2.0f) + 1.0f;

                s->v90_t3_b1_err += (g_re - t_re)*(g_re - t_re)
                                  + (g_im - t_im)*(g_im - t_im);
                if (++s->v90_t3_b1_err_count == 128)
                {
                    span_log(s->logging, SPAN_LOG_WARNING,
                             "Rx - V.90 upstream B1-era distance to grid: "
                             "%.4f per symbol over %d symbols\n",
                             s->v90_t3_b1_err/s->v90_t3_b1_err_count,
                             s->v90_t3_b1_err_count);
                }
                /*endif*/
            }
            /*endif*/
            if (s->v90_t3_acquired  &&  !s->v90_t3_suppress_output)
            {
                float g_re = transformed_re*s->data_symbol_scale;
                float g_im = transformed_im*s->data_symbol_scale;
                float t_re = 2.0f*floorf(g_re/2.0f) + 1.0f;
                float t_im = 2.0f*floorf(g_im/2.0f) + 1.0f;
                float d_re = g_re - t_re;
                float d_im = g_im - t_im;

                s->v90_t3_decision_err += d_re*d_re + d_im*d_im;
                s->v90_t3_decision_pow += g_re*g_re + g_im*g_im;
                /* Running estimate of how far the symbols sit from the
                   constellation, for the timing loop's gate.  Two thirds is
                   the figure for symbols with no relation to the lattice at
                   all; a receiver that is decoding reads about 0.10. */
                s->v90_t3_sym_err_ema += ((d_re*d_re + d_im*d_im)
                                          - s->v90_t3_sym_err_ema)/256.0f;
                /* The same distance over a much shorter memory, for the
                   adaptive elements to gate on -- see
                   V34_V90_T3_ERR_FAST_SHIFT. */
                s->v90_t3_sym_err_fast += ((d_re*d_re + d_im*d_im)
                                           - s->v90_t3_sym_err_fast)
                                          /V34_V90_T3_ERR_FAST_SHIFT;
                /* A mean-square distance-to-grid of 2/3 is exactly what
                   symbols bearing no relation to the lattice give (uniform
                   over a cell of spacing 2).  Sweep a scale factor: if some
                   other gain puts the symbols on the grid, the fault is a
                   scaling one -- V.34's per-rate modulation factor being the
                   obvious candidate -- and if none does, the symbols are not
                   a scaled version of the constellation at all. */
                if (v34_rx_gain_sweep_enabled()  &&  !v34_rx_data_lean())
                {
                    for (int g = 0;  g < V34_V90_T3_GAIN_TRIALS;  g++)
                    {
                        float scale = 0.40f + 0.05f*g;
                        float sr = g_re*scale;
                        float si = g_im*scale;
                        float tr = 2.0f*floorf(sr/2.0f) + 1.0f;
                        float ti = 2.0f*floorf(si/2.0f) + 1.0f;

                        s->v90_t3_gain_err[g] += (sr - tr)*(sr - tr)
                                               + (si - ti)*(si - ti);
                    }
                    /*endfor*/
                }
                /*endif*/
                if (++s->v90_t3_decision_count >= 3200)
                {
                    int best_g = 0;
                    char gains[256];
                    int len = 0;

                    if (!v34_rx_gain_sweep_enabled()  ||  v34_rx_data_lean())
                        goto skip_gain_report;
                    /*endif*/
                    for (int g = 0;  g < V34_V90_T3_GAIN_TRIALS;  g++)
                    {
                        if (s->v90_t3_gain_err[g] < s->v90_t3_gain_err[best_g])
                            best_g = g;
                        /*endif*/
                        if (len < (int) sizeof(gains) - 16)
                        {
                            len += snprintf(gains + len, sizeof(gains) - len,
                                            "%s%.2f:%.3f",
                                            len ? " " : "", 0.40f + 0.05f*g,
                                            s->v90_t3_gain_err[g]
                                              /s->v90_t3_decision_count);
                        }
                        /*endif*/
                    }
                    span_log(s->logging, SPAN_LOG_WARNING,
                             "Rx - V.90 upstream gain sweep: best=%.2f at %.3f "
                             "[%s]\n",
                             0.40f + 0.05f*best_g,
                             s->v90_t3_gain_err[best_g]
                               /s->v90_t3_decision_count,
                             gains);
                    for (int g = 0;  g < V34_V90_T3_GAIN_TRIALS;  g++)
                        s->v90_t3_gain_err[g] = 0.0f;
                    /*endfor*/
skip_gain_report:
                    {
                        /* Say what the line will carry, not just how far the
                           symbols are from it.  Nothing before MP measures
                           this direction -- the rate goes out in MP and the
                           only honest reading appears once data mode is
                           running -- so the receiver has to report it even
                           though it is too late to act on within the call.
                           Same instrument and same calibration as the plain
                           V.34 path's "distance to grid"; see
                           docs/v34_data_mode_rates.md.

                           Measured on artifacts/goal-v90-073744Z and
                           goal-v90-r2: 35.4 and 36.2 dB, against a wire whose
                           least-squares bound is 36.3 and 36.9 -- so this is
                           the line, not the receiver -- while the call was
                           asking 31200, which at 3200 baud is 9.75
                           bits/symbol and wants far more. */
                        float err = s->v90_t3_decision_err
                                  /s->v90_t3_decision_count;
                        float pow = s->v90_t3_decision_pow
                                  /s->v90_t3_decision_count;
                        float snr = (err > 0.0f  &&  pow > 0.0f)
                                  ? 10.0f*log10f(pow/err)  :  0.0f;
                        int baud = baud_rate_parameters[s->baud_rate].baud_rate;
                        int carries = (int) ((snr + V34_DATA_SNR_RATE_OFFSET_DB)
                                             /V34_DATA_SNR_RATE_SLOPE_DB
                                             *baud/2400.0f)*2400;

                        if (carries < 2400)
                            carries = 2400;
                        /*endif*/
                        if (carries > 33600)
                            carries = 33600;
                        /*endif*/
                        /* Report only.  The accumulators behind it are NOT
                           gated: v90_t3_decision_err/_pow/_count also feed the
                           DATA-bits line's sym err column and the divergence
                           branch below, and their 3200-symbol reset cadence is
                           what defines that column's window. */
                        if (!v34_rx_data_lean())
                        span_log(s->logging, SPAN_LOG_WARNING,
                                 "Rx - V.90 upstream DATA: decision error "
                                 "%.4f per symbol (mean symbol power %.2f) "
                                 "over %d symbols; receive SNR %.1f dB, this "
                                 "line will carry %d bit/s against the %d "
                                 "asked for%s\n",
                                 err, pow, s->v90_t3_decision_count, snr,
                                 carries, (s->bit_rate/2 + 1)*2400,
                                 (pow > V34_V90_T3_DIVERGED_POWER)
                                 ? " -- RECEIVER DIVERGED, this reading means"
                                   " nothing"
                                 : (err > 0.55f)
                                 ? " -- OUTPUT IS WHITE"  :  "");
                    }
                    /* A diverged filter cannot be measured out of, because
                       the metric it would be measured with has stopped
                       working -- see V34_V90_T3_DIVERGED_POWER.  Put the last
                       filter that worked back and let the ordinary machinery
                       carry on from there. */
                    if (s->v90_t3_decision_count > 0
                        &&
                        s->v90_t3_decision_pow/s->v90_t3_decision_count
                            > V34_V90_T3_DIVERGED_POWER)
                    {
                        span_log(s->logging, SPAN_LOG_WARNING,
                                 "Rx - V.90 upstream receiver diverged "
                                 "(mean symbol power %.3g); resetting the "
                                 "equalizer\n",
                                 s->v90_t3_decision_pow
                                     /s->v90_t3_decision_count);
                        s->v90_t3_cma_active = false;
                        s->v90_t3_cma_run = 0;
                        if (s->v90_t3_fse_good_valid)
                        {
                            memcpy(s->v90_t3_fse, s->v90_t3_fse_good,
                                   sizeof(s->v90_t3_fse));
                        }
                        /*endif*/
                    }
                    /*endif*/
                    s->v90_t3_decision_err = 0.0f;
                    s->v90_t3_decision_pow = 0.0f;
                    s->v90_t3_decision_count = 0;
                }
                /*endif*/
            }
            /*endif*/

            /* Decision-directed carrier tracking against the DATA
               constellation.  Root cause (2026-07-23, offline 4th-power
               analysis of the winner capture): the V.90 answerer's Phase 3/4
               receive path is never phase-locked -- CP/MP decode succeeds
               because those sequences are DIFFERENTIALLY encoded (8.5.2/V.90)
               and the 24-hypothesis machinery absorbs the unknown rotation,
               while carrier tracking is deliberately frozen through PHASE4_MP
               (phase4_trn_should_freeze_tracking).  Data mode is the first
               thing in the call that needs true coherence, so it must pull in
               and hold its own lock.  The phase detector is the same
               Im(sym x conj(target)) form as the training loop, with the
               target taken from the nearest odd-integer constellation point
               and normalized so the error is sin(delta-phi) regardless of
               ring radius.  The 90-degree acquisition ambiguity is absorbed
               by the differential quadrant bits (V.34 data framing).
               ME_V34_DATA_CARRIER_TRACK=0 disables for A/B. */
            {
                static int dd_enabled = -1;

                if (dd_enabled < 0)
                {
                    const char *value = getenv("ME_V34_DATA_CARRIER_TRACK");

                    dd_enabled = (value == NULL || atoi(value) != 0);
                }
                if (dd_enabled)
                {
                    float g_re = transformed_re * s->data_symbol_scale;
                    float g_im = transformed_im * s->data_symbol_scale;
                    float t_re = 2.0f*floorf((g_re - 1.0f)/2.0f + 0.5f) + 1.0f;
                    float t_im = 2.0f*floorf((g_im - 1.0f)/2.0f + 0.5f) + 1.0f;
                    float sym_mag;
                    float tgt_mag;

                    if (t_re > 43.0f) t_re = 43.0f;
                    else if (t_re < -43.0f) t_re = -43.0f;
                    if (t_im > 43.0f) t_im = 43.0f;
                    else if (t_im < -43.0f) t_im = -43.0f;
                    sym_mag = sqrtf(g_re*g_re + g_im*g_im);
                    tgt_mag = sqrtf(t_re*t_re + t_im*t_im);
                    /* Only steer on a decision worth trusting.  A dense
                       constellation makes most decisions wrong until the loops
                       have converged, and a loop driven by those does not
                       merely fail to acquire -- the frequency integrator below
                       accumulates the noise and drives the phase away: measured
                       at 21600, ungated it produced 0.139 degrees per symbol of
                       drift and a 17434 degree excursion, worse than no loop at
                       all.  Two thirds is the distance of a symbol unrelated to
                       the lattice; half of that keeps the gradient honest. */
                    if (sym_mag > 0.5f && tgt_mag > 0.5f
                        &&
                        ((g_re - t_re)*(g_re - t_re)
                         + (g_im - t_im)*(g_im - t_im)) < v34_rx_data_mode_decision_gate())
                    {
                        /* Phase error in the transformed (grid) domain equals
                           the error in the equalizer domain: the transform is
                           a fixed rotation/conjugation/scale, and the
                           conjugate flips the error sign, which we undo.
                           Update the zero-delay derotator (positive error =
                           received leads target = derotator must remove
                           more), gently -- data decisions are less reliable
                           than the CP/MP decision-aided ones. */
                        float error = (g_im*t_re - g_re*t_im)
                                    / (sym_mag*tgt_mag);

                        if (s->data_symbol_conjugate)
                            error = -error;
                        s->phase4_da_derot +=
                            (int32_t) (error*(1.0f/32.0f)*2147483648.0f/3.14159265f);
                        /* Second-order: integrate the same error into the
                           per-symbol advance.  A phase-only loop settles at a
                           standing error proportional to the residual carrier
                           and cannot remove it, so whatever B1's estimate left
                           behind accumulates for the rest of the call -- 0.0089
                           degrees per symbol still walks a thousand degrees
                           over a minute, and the decision-directed equalizer
                           below then adapts onto decisions that the walk has
                           already made wrong.  Measured, that is what turned a
                           residual of -24 dB at the start of a 21600 call into
                           -12 dB by the end. */
                        s->phase4_da_derot_rate +=
                            (int32_t) (error*v34_rx_data_mode_freq_gain()
                                       *2147483648.0f/3.14159265f);
                    }
                }
            }
        }
        s->duration++;
        if (s->mapping_frame_count >= 16)
        {
            /* Per-frame RMS diagnostic.  MUST stay off by default on the live
               media path: at 3200 baud this is ~400 stderr lines/s, and disk
               I/O on the media clock is a proven call-killer (the buffered-tap
               lesson).  V34_DATA_FRAME_RMS_LOG=1 enables it for offline work. */
            {
                static int rms_log_enabled = -1;

                if (rms_log_enabled < 0)
                    rms_log_enabled = (getenv("V34_DATA_FRAME_RMS_LOG") != NULL)
                                    && !v34_rx_data_lean();
                if (rms_log_enabled)
                {
                    float rms_sum = 0;
                    int ii;
                    for (ii = 0; ii < 16; ii++)
                        rms_sum += (float)s->mapping_frame_buf[ii] * s->mapping_frame_buf[ii];
                    fprintf(stderr, "[DATA] baud=%d frame_rms=%.1f (%.3f)\n",
                            s->duration, sqrtf(rms_sum / 16.0f),
                            sqrtf(rms_sum / 16.0f) / 128.0f);
                }
            }
            /* Dump the EXACT Q9.7 values entering the mapping-frame decoder.
               Offline diagnosis only (raw int16 LE pairs, 16 per frame). */
            {
                static int dump_initialized[2] = {0, 0};
                static FILE *dump_fp[2] = {NULL, NULL};
                int dump_index = s->calling_party ? 1 : 0;

                if (!dump_initialized[dump_index])
                {
                    const char *path = v34_rx_data_lean()
                                     ? NULL : getenv("V34_DATA_FRAME_DUMP");

                    dump_initialized[dump_index] = 1;
                    if (path && *path)
                    {
                        char endpoint_path[1024];
                        snprintf(endpoint_path, sizeof(endpoint_path), "%s.%s",
                                 path, dump_index ? "caller" : "answer");
                        dump_fp[dump_index] = fopen(endpoint_path, "wb");
                    }
                }
                if (dump_fp[dump_index])
                {
                    fwrite(s->mapping_frame_buf, sizeof(int16_t), 16,
                           dump_fp[dump_index]);
                    fflush(dump_fp[dump_index]);
                }
            }
            v34_put_mapping_frame(s, s->mapping_frame_buf);
            s->mapping_frame_count = 0;
        }
        /*endif*/
        s->last_sample = *sym;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
