/* 
 * Copyright (C) 2021 Aon plc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */

#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>        /* getenv/strtod for DM_RS_HEADROOM */
#include <signal.h>
#include <errno.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

#include <pjsua-lib/pjsua.h>

#define SIGNATURE PJMEDIA_SIG_CLASS_PORT_AUD('D','M')

struct dmodem {
	pjmedia_port base;
	pj_timestamp timestamp;
	pj_sock_t sock;
};

static struct dmodem port;
static bool destroying = false;
static pj_pool_t *pool;

static void error_exit(const char *title, pj_status_t status) {
	pjsua_perror(__FILE__, title, status);
	if (!destroying) {
		destroying = true;
		pjsua_destroy();
		exit(1);
	}
}

static FILE *dm_tap_tx, *dm_tap_rx, *dm_tap_tx_9600;

/* Net(8000) -> DSP(9600) rational 6/5 resampler.
 *
 * V.90 puts the modem detector's whole burden on exact 8 kHz symbol timing
 * and on near-Nyquist structure: Sd = {+W,+0,+W,-W,-0,-W} has most of its
 * energy at 4 kHz (the 8 kHz Nyquist).  Two naive resamplers both fail the
 * SmartLink Sd detector:
 *   - zero-order hold keeps the 4 kHz line but its floor() phase injects up
 *     to ~104 us of jitter onto a 125 us symbol, so the recovered pattern
 *     comes out as +W,0,+W,0,-W,-W (corrupted);
 *   - pjmedia's polyphase keeps timing but its ~3.4 kHz voice cutoff nukes
 *     the 4 kHz line to -53 dB.
 * A windowed-sinc polyphase interpolator with the cutoff placed at 4 kHz
 * preserves both.  The long kernel matters for TRN2d: the old 12-tap kernel
 * left enough fractional-delay error to trip SmartLink's Phase-4 PDSNR gate.
 * Six output phases implement k*5 mod 6.  A one-frame pipeline delay supplies
 * the "future" samples the symmetric FIR needs without edge glitches. */
#define RS_TAPS   257
#define RS_HALF   128         /* taps span offsets [-128 .. +128] */
#define RS_PAST   RS_HALF
#define RS_FUTURE (RS_TAPS - RS_HALF - 1)
#define RS_PHASES 6

/* Headroom, folded into the kernel (see the clipping analysis below).
 *
 * Placing the cutoff at the exact input Nyquist makes this kernel very ringy:
 * its per-phase L1 norms are 1.000 2.432 3.447 3.814 3.447 2.432, so a
 * full-scale input can overshoot to 3.81x int16 full scale.  V.90 downstream
 * PCM is exactly the pathological input -- full-scale codewords whose sign
 * flips sample to sample -- so without attenuation the interpolated 9.6 kHz
 * signal saturates the int16 DSP interface.
 *
 * That clipping is not a cosmetic loss.  It is nonlinear, so it breaks the
 * analogue modem's pad-gain estimate: SmartLink's findPadGain() fits a single
 * gain g with received ~= g * level(Ucode) across the DIL sweep, and clipped
 * samples make every candidate g fit badly.  V90TRN2Design() then reports
 * "constelation design failed" and the modem retrains -- the long-standing
 * Phase 4 failure on this rig.  Measured offline with tools/v90_pad_gain.py
 * against a ./vpcm_encode truth capture (DIL at 7000-7560 ms):
 *
 *   transport            interp peak    clipped   fitted gain   per-Ucode err
 *   ideal DS0 (control)          --          --      1.000000     0.000% rms
 *   this kernel, no headroom  71900 (2.19x)    4757    0.952100   120.441% rms
 *   this kernel, 0.45         32355 (0.99x)       0    0.999989     0.689% rms
 *
 * Attenuating costs nothing: a fixed downstream gain is just a digital pad,
 * and recovering it is precisely what findPadGain() is for -- it comes back to
 * within 1e-5 above.
 *
 * The default is set from the L1 bound rather than from measurement.  Worst-
 * case output is L1 * full scale, so any headroom <= 1/3.8143 = 0.2622 cannot
 * clip for *any* input; 0.25 takes that with a little margin (0.25 * 3.8143 =
 * 0.954).  Measuring alone would have been a trap: 0.45 clipped nothing on
 * every capture available, but only because it was already using 98.7% of full
 * scale on the truth capture -- no margin at all, and a slightly different
 * signal would have clipped again.
 *
 * The extra attenuation is free in accuracy terms.  Per-Ucode residual over
 * the DIL sweep is essentially flat from 0.45 down to 0.25 (0.69% -> 0.33%
 * rms overall), and the bands a real constellation actually uses stay far
 * below anything that matters: at 0.25, Ucode 16-63 is 0.114% rms, 64-95 is
 * 0.011%, and 96-127 is 0.0022% against a 512-unit adjacent-Ucode step.  The
 * only band that degrades is Ucode 0-15 (int16 rounding on levels of ~2),
 * which no real V.90 constellation uses.
 *
 * DM_RS_HEADROOM overrides it for sweeps without rebuilding the image. */
#define RS_HEADROOM_DEFAULT 0.25
static float rs_ker[RS_PHASES][RS_TAPS];
static int   rs_ker_ready;
static double rs_headroom = RS_HEADROOM_DEFAULT;
static double rs_fc = 4000.0;
static unsigned long rs_clip_count, rs_out_count, rs_clip_logged;
static double rs_clip_worst;

/* ---------------------------------------------------------------------------
 * Loop model: DS0 staircase + local-loop lowpass (DM_RESAMPLER=zoh, default).
 *
 * The interpolator above is mathematically "ideal reconstruction" and is
 * physically wrong, in a way that breaks V.90 specifically.  With the cutoff at
 * the exact input Nyquist, phase 0's kernel collapses to a SINGLE TAP: output
 * samples on that phase are the input codeword passed through bit-exactly,
 * while the other five phases are 249-tap interpolations of a signal that
 * changes every sample.  Measured on a real downstream capture, distinguishable
 * levels per output phase come out:
 *
 *     phase      0     1     2     3     4     5
 *     levels   inf   5.6   3.2   2.0   1.6   1.5
 *
 * One phase resolves every codeword perfectly and the rest resolve almost
 * nothing.  That is exactly the signature of robbed-bit signalling -- per-phase
 * differences in how many levels survive -- so SmartLink's
 * V90AutoDigitalImpDetector reports "alternate rbs false detection on phase
 * 1/5", computes a nonsense trn1Sigma, and issues "drop to V34 requested".
 * The peer then leaves V.90 in JaTXMIT and never sends the S that terminates
 * our Jd.  No amount of patience on our side can fix that; the signal has to
 * stop looking impaired.
 *
 * The real path is a CO D/A emitting a 125 us staircase into 1-3 km of copper
 * into the far end's sound card.  Modelling that -- zero-order hold to 9600
 * then one lowpass -- makes every output sample derive from exactly one
 * codeword through the same filter, so per-phase uniformity is structural
 * rather than tuned.  Same capture, ZOH + 97-tap 4400 Hz:
 *
 *     phase      0     1     2     3     4     5
 *     levels  13.6  13.3  13.4  13.4  13.6  13.2      (ratio 1.02)
 *
 * Two knobs, because the tradeoff is empirical and has to be swept on the rig:
 *   DM_LOOP_FC    lowpass cutoff in Hz (default 4400).  Higher = less level
 *                 information destroyed; the cutoff is not doing spectral
 *                 shaping here, it is just how much of the codeword survives.
 *   DM_LOOP_TAPS  kernel length (default 97, odd, <= LOOP_TAPS_MAX).  Shorter
 *                 kernels ring less, so the L1 bound allows much more headroom
 *                 -- N=9 permits 0.90 against N=97's 0.44, i.e. +6 dB more
 *                 codeword level -- at some cost in per-phase uniformity
 *                 (ratio 1.58 vs 1.02).  Pure ZOH (no filter) is the limit and
 *                 is known to fail: its floor() edge quantisation puts up to
 *                 104 us of jitter on a 125 us symbol, which is what broke the
 *                 Sd detector when ZOH was tried alone.  Some smoothing is
 *                 required; how little is what the sweep answers.
 *
 * Headroom is again derived from the L1 bound (worst-case output = L1 * FS) so
 * it provably cannot clip, and it is much less punishing here than for the
 * ringy interpolator: L1 falls from 3.81 to ~2.26 (97 taps) or ~1.11 (9 taps),
 * so the codewords arrive 1.8x to 3.6x larger than under the old 0.25.  That
 * is the point -- level resolution is what the peer's detectors actually use.
 * DM_RESAMPLER=sinc restores the old interpolator for A/B.
 * DM_RESAMPLER=hybrid keeps that interpolator at the Phase-3-safe 0.25 gain
 * through Sd and S-bar-d, then reduces only its scalar gain to 0.22 over one
 * second of TRN1d/Jd.  The channel shape stays fixed while the lower final
 * gain keeps SmartLink's TRN2d error below its absolute drop threshold. */
#define LOOP_TAPS_MAX 129
static float  loop_ker[LOOP_TAPS_MAX];
static int    loop_ker_ready;
static int    loop_taps = 97;
static double loop_fc = 4400.0;
static double loop_headroom;
static int    use_zoh_loop = 1;
static int    use_hybrid_loop;
static int    hybrid_gain_active;
static int    hybrid_blend_started;
static int    hybrid_blend_progress;
static double hybrid_target_headroom = 0.22;

static void rs_build_kernel(void) {
	const double fs_in = 8000.0;
	double wc;
	const char *env;
	int p, t;
	if (rs_ker_ready) return;
	env = getenv("DM_RS_FC");
	if (env && *env) {
		char *end;
		double parsed = strtod(env, &end);
		if (end != env && *end == '\0' && parsed >= 3500.0 && parsed <= 4000.0)
			rs_fc = parsed;
		else
			PJ_LOG(2,(__FILE__, "DM_RS_FC='%s' ignored (want 3500..4000 Hz), "
			                    "using %.0f Hz", env, rs_fc));
	}
	wc = rs_fc/(fs_in/2.0);
	env = getenv("DM_RS_HEADROOM");
	if (env && *env) {
		char *end;
		double parsed = strtod(env, &end);
		/* Reject junk and values that cannot help: >1.0 amplifies into the
		 * clamp, <=0 would mute the DSP feed entirely. */
		if (end != env && *end == '\0' && parsed > 0.0 && parsed <= 1.0)
			rs_headroom = parsed;
		else
			PJ_LOG(2,(__FILE__, "DM_RS_HEADROOM='%s' ignored (want 0 < h <= 1), "
			                    "using %.3f", env, rs_headroom));
	}
	for (p = 0; p < RS_PHASES; p++) {
		double f = (double)p/(double)RS_PHASES;   /* fractional output pos */
		double sum = 0.0, l1 = 0.0;
		for (t = 0; t < RS_TAPS; t++) {
			double x = (double)(t - RS_HALF) - f;  /* input samples from output pt */
			double s, w, y = wc*x;
			if (fabs(y) < 1e-9) s = wc;
			else s = wc*sin(M_PI*y)/(M_PI*y);
			w = 0.5 - 0.5*cos(2.0*M_PI*t/(RS_TAPS-1));  /* Hann */
			rs_ker[p][t] = (float)(s*w);
			sum += rs_ker[p][t];
		}
		for (t = 0; t < RS_TAPS; t++) {
			/* Unity DC gain, then headroom. */
			rs_ker[p][t] = (float)(rs_ker[p][t]/sum*rs_headroom);
			l1 += fabs((double)rs_ker[p][t]);
		}
		PJ_LOG(4,(__FILE__, "resampler phase %d: L1=%.3f (worst-case %.2fx "
		                    "int16 FS)", p, l1, l1));
	}
	PJ_LOG(3,(__FILE__, "resampler cutoff %.0f Hz, headroom %.3f (%.2f dB) folded into kernel",
	          rs_fc, rs_headroom, 20.0*log10(rs_headroom)));
	rs_ker_ready = 1;
}

/* floor division, needed because output index k runs negative into history */
static int loop_fdiv(int a, int b) {
	int q = a/b;
	if ((a % b) != 0 && ((a < 0) != (b < 0))) q--;
	return q;
}

/* Sd is {+W,+0,+W,-W,-0,-W}; S-bar-d is its exact sign inverse for
 * eight repetitions.  Search the assembled FIR work buffer, which gives us
 * 128 past and future input samples around prev[].  A 96-symbol periodic Sd
 * history, the W/0/W magnitude signature, and the complete 48-symbol barred
 * sequence make this boundary unique without mistaking Ri or mapped data for
 * it.  Keep sinc through all of S-bar-d, then change at the first TRN1d
 * symbol so the peer trains its equalizer on the final channel model. */
#define HYBRID_SD_HISTORY_SYMBOLS  96
#define HYBRID_SD_BAR_SYMBOLS      48
#define HYBRID_BLEND_DELAY_SYMBOLS 12000
#define HYBRID_BLEND_SYMBOLS        8000
static int loop_find_trn1d_start(const pj_int16_t *work,
                                 int prev_start,
                                 int prev_count) {
	int transition, k;

	if (!work || prev_start < HYBRID_SD_HISTORY_SYMBOLS)
		return -1;
	/* Also consider transitions in the last 48 samples of hist[]: their
	 * TRN1d boundary can fall in the current prev[] frame. */
	for (transition = prev_start - HYBRID_SD_BAR_SYMBOLS;
	     transition < prev_start + prev_count;
	     transition++) {
		int first = work[transition - 6];

		if (abs(first) < 256
		    || work[transition - 5] != 0
		    || work[transition - 4] != first
		    || work[transition - 3] != -first
		    || work[transition - 2] != 0
		    || work[transition - 1] != -first)
			continue;
		for (k = -HYBRID_SD_HISTORY_SYMBOLS; k < 0; k++) {
			int phase = ((k % 6) + 6) % 6;

			if (work[transition + k] != work[transition - 6 + phase])
				break;
		}
		if (k != 0)
			continue;
		for (k = 0; k < HYBRID_SD_BAR_SYMBOLS; k++) {
			if (work[transition + k] != -work[transition - 6 + (k % 6)])
				break;
		}
		if (k == HYBRID_SD_BAR_SYMBOLS) {
			int switch_at = transition - prev_start
			              + HYBRID_SD_BAR_SYMBOLS;

			if (switch_at >= 0 && switch_at < prev_count)
				return switch_at;
		}
	}
	return -1;
}

static void loop_build_kernel(void) {
	const double fs = 9600.0;
	const char *env;
	double wc, sum = 0.0, l1 = 0.0;
	int t, H;

	if (loop_ker_ready) return;

	if ((env = getenv("DM_RESAMPLER")) && *env) {
		use_hybrid_loop = (strcmp(env, "hybrid") == 0);
		use_zoh_loop = !use_hybrid_loop && (strcmp(env, "sinc") != 0);
	}
	if ((env = getenv("DM_HYBRID_HEADROOM")) && *env) {
		char *end; double v = strtod(env, &end);
		if (end != env && *end == '\0' && v > 0.0 && v <= rs_headroom)
			hybrid_target_headroom = v;
		else
			PJ_LOG(2,(__FILE__, "DM_HYBRID_HEADROOM='%s' ignored "
			                    "(want 0 < h <= %.3f), using %.3f",
			                    env, rs_headroom, hybrid_target_headroom));
	}
	if ((env = getenv("DM_LOOP_FC")) && *env) {
		char *end; double v = strtod(env, &end);
		if (end != env && *end == '\0' && v > 300.0 && v <= fs/2.0) loop_fc = v;
		else PJ_LOG(2,(__FILE__,"DM_LOOP_FC='%s' ignored, using %.0f", env, loop_fc));
	}
	if ((env = getenv("DM_LOOP_TAPS")) && *env) {
		char *end; long v = strtol(env, &end, 10);
		if (end != env && *end == '\0' && v >= 3 && v <= LOOP_TAPS_MAX && (v & 1))
			loop_taps = (int)v;
		else PJ_LOG(2,(__FILE__,"DM_LOOP_TAPS='%s' ignored (odd, 3..%d), using %d",
		               env, LOOP_TAPS_MAX, loop_taps));
	}

	H = loop_taps/2;
	wc = loop_fc/(fs/2.0);
	for (t = 0; t < loop_taps; t++) {
		double x = (double)(t - H), y = wc*x, s, w;
		if (fabs(y) < 1e-9) s = wc;
		else s = wc*sin(M_PI*y)/(M_PI*y);
		/* Blackman: lower sidelobes than Hann for a given length, which keeps
		 * L1 (and therefore the headroom penalty) down. */
		w = 0.42 - 0.5*cos(2.0*M_PI*t/(loop_taps-1))
		         + 0.08*cos(4.0*M_PI*t/(loop_taps-1));
		loop_ker[t] = (float)(s*w);
		sum += loop_ker[t];
	}
	for (t = 0; t < loop_taps; t++) {
		loop_ker[t] = (float)(loop_ker[t]/sum);          /* unity DC */
		l1 += fabs((double)loop_ker[t]);
	}
	/* Provably cannot clip for any input, same argument as RS_HEADROOM_DEFAULT.
	 * DM_RS_HEADROOM still overrides, for sweeping below the safe bound. */
	loop_headroom = use_hybrid_loop ? rs_headroom : 1.0/l1;
	if ((env = getenv("DM_RS_HEADROOM")) && *env) {
		char *end; double v = strtod(env, &end);
		if (end != env && *end == '\0' && v > 0.0 && v <= 1.0) loop_headroom = v;
	}
	for (t = 0; t < loop_taps; t++)
		loop_ker[t] = (float)(loop_ker[t]*loop_headroom);

	PJ_LOG(3,(__FILE__,"loop model: ZOH staircase + %d-tap %.0f Hz lowpass, "
	                   "L1=%.3f headroom=%.3f (%.2f dB, %.2fx vs old 0.25)",
	                   loop_taps, loop_fc, l1, loop_headroom,
	                   20.0*log10(loop_headroom), loop_headroom/0.25));
	/* Both kernels are always built, so the two lines above say nothing about
	 * which one is actually in the signal path.  State it explicitly -- a
	 * control run is worthless if you cannot prove the control was applied. */
	PJ_LOG(3,(__FILE__,"ACTIVE downstream path: %s",
	          use_hybrid_loop
	              ? "sinc at 0.25 through early Jd, then delayed 8000-symbol gain ramp (DM_RESAMPLER=hybrid)"
	              : (use_zoh_loop
	                    ? "ZOH staircase + loop lowpass (DM_RESAMPLER=zoh)"
	                    : "windowed-sinc polyphase interpolator (DM_RESAMPLER=sinc)")));
	loop_ker_ready = 1;
}

static pj_status_t dmodem_put_frame(pjmedia_port *this_port, pjmedia_frame *frame) {
	struct dmodem *sm = (struct dmodem *)this_port;
	/* pipeline: hist[] = tail of frame N-2, prev[] = frame N-1 */
	static pj_int16_t hist[RS_PAST];
	static pj_int16_t prev[1024];
	static int prev_n = -1;      /* -1 until first frame seen */
	int len;

	if (frame->type == PJMEDIA_FRAME_TYPE_AUDIO) {
		const pj_int16_t *in = (const pj_int16_t *)frame->buf;
		int in_n = (int)(frame->size / 2);
		pj_int16_t out[1152];
		pj_int16_t work[RS_PAST + 1024 + RS_FUTURE];
		int out_n, k, i, hybrid_switch_at = -1;
		int hybrid_blend_base = 0, hybrid_blending = 0;

		rs_build_kernel();
		loop_build_kernel();
		if (in_n > 1024)
			return PJ_ETOOBIG;
		if (prev_n >= 0 && (prev_n < RS_PAST || in_n < RS_FUTURE))
			return PJ_EINVAL;

		if (prev_n < 0) {
			/* Prime the pipeline: emit one frame of silence (20 ms latency). */
			out_n = in_n * 6 / 5;
			memset(out, 0, (size_t)out_n * 2);
			memcpy(prev, in, (size_t)in_n * 2);
			memset(hist, 0, sizeof(hist));
			prev_n = in_n;
		} else {
			/* Render prev[] with hist before it and this frame's head after. */
			out_n = prev_n * 6 / 5;
			for (i = 0; i < RS_PAST; i++)
				work[i] = hist[i];
			for (i = 0; i < prev_n; i++)
				work[RS_PAST + i] = prev[i];
			for (i = 0; i < RS_FUTURE; i++)
				work[RS_PAST + prev_n + i] = in[i];
			if (use_hybrid_loop && !hybrid_gain_active && !hybrid_blend_started)
				hybrid_switch_at = loop_find_trn1d_start(work, RS_PAST, prev_n);
			if (hybrid_blend_started) {
				hybrid_blending = 1;
				hybrid_blend_base = hybrid_blend_progress;
			} else if (hybrid_switch_at >= 0) {
				hybrid_blending = 1;
				hybrid_blend_base = -hybrid_switch_at
				                  - HYBRID_BLEND_DELAY_SYMBOLS;
			}
			int loop_h = loop_taps/2;
			for (k = 0; k < out_n; k++) {
				int num = k * 5;
				int ic = num / 6;           /* integer input index in prev */
				int ph = num % 6;           /* output phase */
				const float *h = rs_ker[ph];
				int base = RS_PAST + ic - RS_HALF;   /* work index of tap 0 */
				float acc = 0.0f;
				int t;
				if (use_zoh_loop) {
					/* Staircase: output sample m takes the codeword of the DS0
					 * interval it falls in, floor(m*5/6).  Every output on
					 * every phase therefore comes from exactly one codeword
					 * through the same filter -- that is what makes the
					 * per-phase level resolution uniform. */
					for (t = 0; t < loop_taps; t++) {
						int m = k - loop_h + t;
						int si = RS_PAST + loop_fdiv(m*5, 6);
						acc += (float)work[si] * loop_ker[t];
					}
				} else {
					float gain = 1.0f;

					for (t = 0; t < RS_TAPS; t++)
						acc += (float)work[base + t] * h[t];
					if (use_hybrid_loop && hybrid_gain_active) {
						gain = (float)(hybrid_target_headroom/rs_headroom);
					} else if (use_hybrid_loop && hybrid_blending
					           && hybrid_blend_base + ic >= 0) {
						int progress = hybrid_blend_base + ic;
						float alpha = (progress >= HYBRID_BLEND_SYMBOLS)
						              ? 1.0f
						              : (float)progress/(float)HYBRID_BLEND_SYMBOLS;
						float target = (float)(hybrid_target_headroom/rs_headroom);

						gain = 1.0f + alpha*(target - 1.0f);
					}
					acc *= gain;
				}
				/* Clipping here is nonlinear and silently destroys the peer's
				 * pad-gain fit (see RS_HEADROOM_DEFAULT).  It cost a long
				 * investigation precisely because nothing ever reported it,
				 * so count it and say so. */
				if (acc > 32767.0f || acc < -32768.0f) {
					double over = fabs((double)acc)/32767.0;
					if (over > rs_clip_worst) rs_clip_worst = over;
					rs_clip_count++;
				}
				if (acc > 32767.0f) acc = 32767.0f;
				else if (acc < -32768.0f) acc = -32768.0f;
				out[k] = (pj_int16_t)(acc >= 0 ? acc + 0.5f : acc - 0.5f);
			}
			if (hybrid_switch_at >= 0) {
				PJ_LOG(3,(__FILE__, "hybrid downstream: complete S-bar-d detected; "
					                    "starting %d-symbol gain ramp %d symbols after TRN1d input offset %d",
					                    HYBRID_BLEND_SYMBOLS,
					                    HYBRID_BLEND_DELAY_SYMBOLS,
					                    hybrid_switch_at));
				hybrid_blend_started = 1;
				hybrid_blend_progress = prev_n - hybrid_switch_at
				                      - HYBRID_BLEND_DELAY_SYMBOLS;
			} else if (hybrid_blend_started && !hybrid_gain_active) {
				hybrid_blend_progress += prev_n;
			}
			if (hybrid_blend_started && !hybrid_gain_active
			    && hybrid_blend_progress >= HYBRID_BLEND_SYMBOLS) {
				hybrid_gain_active = 1;
				PJ_LOG(3,(__FILE__, "hybrid downstream: gain ramp complete; "
				                    "sinc headroom %.3f is fully active",
				                    hybrid_target_headroom));
			}
			rs_out_count += out_n;
			if (rs_clip_count && rs_out_count - rs_clip_logged >= 9600) {
				PJ_LOG(2,(__FILE__, "resampler CLIPPING %lu/%lu samples (%.3f%%), "
				          "worst %.2fx int16 FS -- this breaks the peer's "
				          "findPadGain(); lower DM_RS_HEADROOM (now %.3f)",
				          rs_clip_count, rs_out_count,
				          100.0*(double)rs_clip_count/(double)rs_out_count,
				          rs_clip_worst, rs_headroom));
				rs_clip_logged = rs_out_count;
			}
			/* advance pipeline */
			for (i = 0; i < RS_PAST; i++)
				hist[i] = prev[prev_n - RS_PAST + i];
			memcpy(prev, in, (size_t)in_n * 2);
			prev_n = in_n;
		}
		if (!dm_tap_rx) dm_tap_rx = fopen("/tmp/dm_to_dsp.raw","wb");
		if (dm_tap_rx) { fwrite(out,2,out_n,dm_tap_rx); fflush(dm_tap_rx); }
		if ((len=write(sm->sock, out, out_n*2)) != out_n*2) {
			error_exit("error writing frame",0);
		}
	}

	return PJ_SUCCESS;
}

/* DSP 9600 -> net 8000: the upstream is V.34-modulated, band-limited
 * well below 3.4 kHz, so simple linear interpolation is adequate. */
static pj_status_t dmodem_get_frame(pjmedia_port *this_port, pjmedia_frame *frame) {
	struct dmodem *sm = (struct dmodem *)this_port;
	pj_int16_t raw[1152];
	pj_int16_t *out;
	int out_n, in_n, k;
	frame->size = PJMEDIA_PIA_AVG_FSZ(&this_port->info); // MAX? what is

	int len;
	out_n = (int)(frame->size / 2);
	in_n = out_n * 6 / 5;
	if (in_n > (int)PJ_ARRAY_SIZE(raw))
		return PJ_ETOOBIG;
	if ((len=read(sm->sock, raw, in_n*2)) != in_n*2) {
		error_exit("error reading frame",0);
	}
	/* Pre-interpolation tap: the DSP's native 9600 Hz upstream output,
	 * before the 6/5 linear interpolation below.  dm_from_dsp.raw is
	 * written *after* that step, so it cannot distinguish "the resampler
	 * destroyed the signal" from "the DSP never produced one".  Compare
	 * this file against dm_from_dsp.raw to tell them apart. */
	if (!dm_tap_tx_9600) dm_tap_tx_9600 = fopen("/tmp/dm_from_dsp_9600.raw","wb");
	if (dm_tap_tx_9600) { fwrite(raw,2,(size_t)in_n,dm_tap_tx_9600); fflush(dm_tap_tx_9600); }
	out = (pj_int16_t *)frame->buf;
	for (k = 0; k < out_n; k++) {
		int num = k * 6;
		int i = num / 5;
		int frac = num % 5;          /* source position k*1.2 = i + frac/5 */
		int a = raw[i];
		int b = (i + 1 < in_n) ? raw[i + 1] : raw[i];
		out[k] = (pj_int16_t)((a * (5 - frac) + b * frac) / 5);
	}
	len = out_n * 2;

	if (!dm_tap_tx) dm_tap_tx = fopen("/tmp/dm_from_dsp.raw","wb");
	if (dm_tap_tx) { fwrite(frame->buf,1,(size_t)len,dm_tap_tx); fflush(dm_tap_tx); }
	frame->timestamp.u64 = sm->timestamp.u64;
	frame->type = PJMEDIA_FRAME_TYPE_AUDIO;
	sm->timestamp.u64 += PJMEDIA_PIA_SPF(&this_port->info);

	return PJ_SUCCESS;
}

static pj_status_t dmodem_on_destroy(pjmedia_port *this_port) {
	printf("destroy\n");
	exit(-1);
}

/* Callback called by the library when call's state has changed */
static void on_call_state(pjsua_call_id call_id, pjsip_event *e) {
	pjsua_call_info ci;

	PJ_UNUSED_ARG(e);

	pjsua_call_get_info(call_id, &ci);
	PJ_LOG(3,(__FILE__, "Call %d state=%.*s", call_id,
				(int)ci.state_text.slen,
				ci.state_text.ptr));

	if (ci.state == PJSIP_INV_STATE_DISCONNECTED) {
		close(port.sock);
		if (!destroying) {
			destroying = true;
			pjsua_destroy();
			exit(0);
		}
	}
}

/* Callback called by the library when call's media state has changed */
static void on_call_media_state(pjsua_call_id call_id) {
	pjsua_call_info ci;
	pjsua_conf_port_id port_id;
	static int done=0;

	pjsua_call_get_info(call_id, &ci);

//	printf("media_status %d media_cnt %d ci.conf_slot %d aud.conf_slot %d\n",ci.media_status,ci.media_cnt,ci.conf_slot,ci.media[0].stream.aud.conf_slot);
	if (ci.media_status == PJSUA_CALL_MEDIA_ACTIVE) {
		if (!done) {
			/* The port itself now converts 8000<->9600 (ZOH down,
			   linear up); it faces the bridge at 8000 Hz directly. */
			pjsua_conf_add_port(pool, &port.base, &port_id);
			pjsua_conf_connect(ci.conf_slot, port_id);
			pjsua_conf_connect(port_id, ci.conf_slot);
			done = 1;
		}
	} else {
		done = 0;
	}
}


int main(int argc, char *argv[]) {
	pjsua_acc_id acc_id;
	pj_status_t status;

	if (argc != 3) {
		return -1;
	}

	signal(SIGPIPE,SIG_IGN);

	char *dialstr = argv[1];

	char *sip_user = getenv("SIP_LOGIN");
	if (!sip_user) {
		return -1;
	}
	char *sip_domain = strchr(sip_user,'@');
	if (!sip_domain) {
		return -1;
	}
	*sip_domain++ = '\0';
	char *sip_pass = strchr(sip_user,':');
	if (!sip_pass) {
		return -1;
	}
	*sip_pass++ = '\0';

	status = pjsua_create();
	if (status != PJ_SUCCESS) error_exit("Error in pjsua_create()", status);

	/* Init pjsua */
	{
		pjsua_config cfg;
		pjsua_logging_config log_cfg;
		pjsua_media_config med_cfg;

		pjsua_config_default(&cfg);
		cfg.cb.on_call_media_state = &on_call_media_state;
		cfg.cb.on_call_state = &on_call_state;

		pjsua_logging_config_default(&log_cfg);
		log_cfg.console_level = 4;

		pjsua_media_config_default(&med_cfg);
		med_cfg.no_vad = true;
		med_cfg.ec_tail_len = 0;
		/* Modem waveforms cannot tolerate the adaptive jitter buffer
		 * periodically discarding samples to chase a voice-oriented latency
		 * target.  The SIP hop is on the local LAN, so use a small fixed
		 * three-packet playout buffer and never discard a received frame. */
		med_cfg.jb_init = 60;
		med_cfg.jb_min_pre = 60;
		med_cfg.jb_max_pre = 60;
		med_cfg.jb_max = 200;
		med_cfg.jb_discard_algo = PJMEDIA_JB_DISCARD_NONE;
		med_cfg.clock_rate = 8000;
		med_cfg.snd_clock_rate = 8000;
		med_cfg.audio_frame_ptime = 20;

		status = pjsua_init(&cfg, &log_cfg, &med_cfg);
		if (status != PJ_SUCCESS) error_exit("Error in pjsua_init()", status);
	}

	pjsua_set_ec(0,0); // maybe?
	pjsua_set_null_snd_dev();
	
	/* g711 only */
	pjsua_codec_info codecs[32];
	unsigned count = sizeof(codecs)/sizeof(*codecs);
	pjsua_enum_codecs(codecs,&count);
	for (int i=0; i<count; i++) {
		int pri = 0;
		if (pj_strcmp2(&codecs[i].codec_id,"PCMU/8000/1") == 0) {
			/* u-law preferred: the SmartLink V.90 datapump is
			   hardwired for u-law PCM tables (US-market blob). */
			pri = 2;
		} else if (pj_strcmp2(&codecs[i].codec_id,"PCMA/8000/1") == 0) {
			pri = 0;
		}
		pjsua_codec_set_priority(&codecs[i].codec_id, pri);
//		printf("codec: %s %d\n",pj_strbuf(&codecs[i].codec_id),pri);
	}

	/* Add UDP transport. */
	{
		pjsua_transport_config cfg;

		pjsua_transport_config_default(&cfg);
		cfg.port = 5060;
		status = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, NULL);
		if (status != PJ_SUCCESS) error_exit("Error creating transport", status);
	}

	pj_caching_pool cp;
	pj_caching_pool_init(&cp, NULL, 1024*1024);
	pool = pj_pool_create(&cp.factory, "pool1", 4000, 4000, NULL);

	pj_str_t name = pj_str("dmodem");
	
	memset(&port,0,sizeof(port));
	port.sock = atoi(argv[2]); // inherited from parent
	pjmedia_port_info_init(&port.base.info, &name, SIGNATURE, 8000, 1, 16, 160);
	port.base.put_frame = dmodem_put_frame;
	port.base.get_frame = dmodem_get_frame;
	port.base.on_destroy = dmodem_on_destroy;

	char buf[384];
	memset(buf,0,sizeof(buf));
	write(port.sock, buf, sizeof(buf));

	/* Initialization is done, now start pjsua */
	status = pjsua_start();
	if (status != PJ_SUCCESS) error_exit("Error starting pjsua", status);

	{
		pjsua_acc_config cfg;
		pjsua_acc_config_default(&cfg);
		snprintf(buf,sizeof(buf),"sip:%s@%s",sip_user,sip_domain);
		pj_strdup2(pool,&cfg.id,buf);
		snprintf(buf,sizeof(buf),"sip:%s",sip_domain);
		pj_strdup2(pool,&cfg.reg_uri,buf);
		cfg.register_on_acc_add = false;
		cfg.cred_count = 1;
		cfg.cred_info[0].realm = pj_str("*");
		cfg.cred_info[0].scheme = pj_str("digest");
		cfg.cred_info[0].username = pj_str(sip_user);
		cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
		cfg.cred_info[0].data = pj_str(sip_pass);

		status = pjsua_acc_add(&cfg, PJ_TRUE, &acc_id);
		if (status != PJ_SUCCESS) error_exit("Error adding account", status);
	}

	snprintf(buf,sizeof(buf),"sip:%s@%s",dialstr,sip_domain);
	printf("calling %s\n",buf);
	pj_str_t uri = pj_str(buf);
	
	pjsua_call_id callid;
	status = pjsua_call_make_call(acc_id, &uri, 0, NULL, NULL, &callid);
	if (status != PJ_SUCCESS) error_exit("Error making call", status);

	struct timespec ts = {100, 0};
	while(1) {
		nanosleep(&ts,NULL);
	}

	return 0;
}
