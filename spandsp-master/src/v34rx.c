/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v34rx.c - ITU V.34 modem, receive part
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2009 Steve Underwood
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*! \file */

/* THIS IS A WORK IN PROGRESS - NOT YET FUNCTIONAL! */

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

#include "v34rx_internal.h"

/* V34_RX_LEAN_BUILD: compile the diagnostics out, as v34rx_data.c does.  This
   file carries 213 V34_RX_LOG() calls and 13 *_DUMP environment gates; the log
   calls are the expensive ones, because every float argument is promoted to
   double by C's default argument promotion and the ESP32-S3's FPU is
   single-precision, so each promotion is a libgcc software call.

   ONLY the diagnostics go.  The ~40 ME_* behaviour knobs stay exactly as they
   are -- they select algorithms (timing detector, echo policy, eye selection,
   CMA bounds) and several are load-bearing defaults with measurements behind
   them.  A build that silently pinned those would be a different modem.

   Undefined by default; nothing about a normal build changes. */
#if defined(V34_RX_LEAN_BUILD)
#define V34_RX_LOG(...)             ((void) 0)
#define V34_DIAG_GETENV(name)       ((const char *) 0)
/* Every stdio use in this file is a *_DUMP block.  Their getenv is already
   NULL above, so they never run -- but the FILE* they test is a static the
   compiler will not always prove unreachable, and two of them sit inside
   process_primary_symbol() with (double) casts in their argument lists.
   Removing stdio outright folds them and takes the promotions with it. */
#define fopen(a, b)                 ((FILE *) 0)
#define fclose(a)                   ((void) 0)
#define fflush(a)                   ((void) 0)
#define fwrite(a, b, c, d)          ((size_t) 0)
#define fprintf(...)                ((void) 0)
#else
#define V34_RX_LOG(...)             span_log(__VA_ARGS__)
#define V34_DIAG_GETENV(name)       getenv(name)
#endif

#ifndef V34_TRACE_DIAGNOSTICS
#define V34_TRACE_DIAGNOSTICS v34_rx_trace_diagnostics()
/* Opt-in diagnostics.  Each caches its getenv() so the check costs nothing
   on the per-symbol paths that use it. */
/* How many times one rate renegotiation may re-acquire its CP conditioning
   after the line goes dead.  A window is about seven seconds; a line that
   drops out more often than this is not one a renegotiation can rescue. */
#define V34_V90_RENEG_CP_MAX_REACQUIRES 8

static int v90_reneg_cp_reacquire_enabled(void)
{
    return 0;      /* it did not recover the constellation; see the write-up */
}

/* RMS of the block of line samples the receiver was last handed, read by
   V90_RENEG_SYM_DUMP; set in primary_channel_rx(). */
static double v90_reneg_feed_rms = 0.0;

/* The per-symbol 33-trial gain sweep in the DATA case (see
   V34_V90_T3_GAIN_TRIALS) accumulates into v90_t3_gain_err, which is read
   only by the report beside it and zeroed with it -- it steers nothing.  It
   answers "if some other scale put the symbols on the grid, the fault would
   be a scaling one", which is a bring-up question, and it costs about 330
   float ops PER SYMBOL: at 3200 baud roughly 1 Mflop/s, which is around half
   the arithmetic budget of this whole receiver on an embedded target.
   ME_V90_DATA_LEAN=1 turns it off. */
/* PHASE4_TRN spends most of its length discovering the MP decode's
   domain/tap/bit-order and publishing them as phase4_trn_lock_*, which the MP
   case then uses as its lock hint.  All three are fixed by 8.5.2/10.1.3.3 and
   the ordering of training_constellation_4, so the discovery looks removable
   -- exactly the reasoning that streaming startup CP measured and found
   false for the MP search itself.  Withholding the hint and letting MP start
   from the pinned defaults is the same question one stage earlier, and it
   stops plain V.34 training (v34rx_phase4_trn.c). */
/* The PHASE3_WAIT_S counterpart of the TRN hint: this stage publishes
   phase3_j_lock_hyp, which the MP case falls back to when PHASE4_TRN has not
   produced a lock (see the hint_h line in the MP stage). */
static bool v34_rx_caller_hearing_own_phase3(v34_rx_state_t *s);
static bool v34_rx_is_v34_call_modem(const v34_rx_state_t *s);
static bool v34_rx_answerer_sending_own_phase3(v34_rx_state_t *s);

static bool v34_rx_caller_hearing_own_phase3_m(v34_rx_state_t *s, int what);

int v34_rx_j_hint_enabled(void)
{
    return 1;
}
/*- End of function --------------------------------------------------------*/

static int v34_diag_flag(const char *name, int *cache)
{
    if (*cache < 0)
        *cache = (getenv(name) != NULL);
    /*endif*/
    return *cache;
}

/* The frame-phase confirmation stage (see V34_V90_T3_CONFIRM_BITS).
   OFF by default, and the reason is a measurement against it.

   The theory is sound and the arithmetic behind V34_V90_T3_CONFIRM_BITS still
   holds, but the one live call in which this code has ever actually run says
   it does not work: relgate-r5 of artifacts/gate-ab-31200-*, the only call of
   twelve whose sweep ever reached its completion branch (784 steps, 7
   completions), locked five times on shell evidence and delivered ZERO
   payload across 111.8 s of open eye, while the other eleven locked on marks
   and carried 20000 lines apiece.  Whatever the shell bound is worth over 133
   frames, it is not enough on its own to choose among the candidates the
   coarse pass leaves -- or this implementation picks the first clean one
   where it should pick the best.

   Left in, off, with the evidence beside it: the diagnosis it came from is
   still the right one and the next attempt should start here rather than from
   scratch.  ME_V90_PHASE_CONFIRM=1 enables it. */
static int v90_t3_phase_confirm_enabled(void)
{
    static int cache = -1;

    if (cache < 0)
    {
        const char *v = getenv("ME_V90_PHASE_CONFIRM");

        cache = (v  &&  atoi(v) != 0)  ?  1  :  0;
    }
    /*endif*/
    return cache;
}

/* ME_V90_PHASE_NO_MARKS=1 denies the content-dependent locks (the marks on an
   idle line, the V.14 ratio on a busy one), leaving only the shell-index
   evidence.  It exists because the failure this machinery is for -- the peer
   already transmitting when data mode starts, so no phase ever reads high --
   does NOT occur in any recording we hold: every tap begins with the peer
   idle, so phase 0 locks on 100% marks after 0 steps and the sweep is never
   asked a hard question.  Setting this reproduces the live condition on a
   recording whose correct answer is known. */
/* ME_V90_SWEEP_EYE_ABS=1 restores the absolute eye test that used to gate the
   frame-phase sweep, for A/B work.  See the comment at its call site. */
static int v90_t3_sweep_abs_eye(void)
{
    static int cache = -1;

    if (cache < 0)
    {
        const char *v = getenv("ME_V90_SWEEP_EYE_ABS");

        cache = (v  &&  atoi(v) != 0)  ?  1  :  0;
    }
    /*endif*/
    return cache;
}

/* Require positive evidence that the current phase is wrong before sweeping.
   OFF by default, on a measurement against it.

   The reasoning was good: B1 pins the phase correctly on almost every call,
   the live busy-start failure ARRIVED on a good phase and was pushed into a
   sweep by noise on 15-frame windows, and a sweep that goes wrong is
   unrecoverable.  But held up to a controlled test it does not pay.  With the
   marks lock denied in BOTH arms -- which forces every call into the
   busy-start condition the search actually has to handle -- six calls a side
   at 28800: the arm that could not sweep made 0 steps and delivered 38362
   payload lines, and the arm that swept made 224 steps in every call and
   delivered 49201.  Sweeping 224 steps cost nothing there, and gating it cost
   about a fifth of the payload.  The 31200 repeat, where the original failure
   was seen, yielded no data-mode calls at all (the rig spent that session in
   the "no S after Jd" Phase 3 blocker), so this is decided on 28800 alone.

   ME_V90_SWEEP_NEEDS_EVIDENCE=1 enables it. */
static int v90_t3_sweep_needs_evidence(void)
{
    static int cache = -1;

    if (cache < 0)
    {
        const char *v = getenv("ME_V90_SWEEP_NEEDS_EVIDENCE");

        cache = (v  &&  atoi(v) != 0)  ?  1  :  0;
    }
    /*endif*/
    return cache;
}

/* ME_V90_PHASE_SWEEP=0 never sweeps the upstream frame phase (diagnostic):
   with ME_V90_PHASE_FORCE_OFFSET it holds the decoder at one candidate for
   the whole call, so each candidate can be graded on what it decodes. */
static int v90_t3_phase_sweep_enabled(void)
{
    static int cache = -1;

    if (cache < 0)
    {
        const char *v = getenv("ME_V90_PHASE_SWEEP");

        cache = (v  &&  atoi(v) == 0)  ?  0  :  1;
    }
    /*endif*/
    return cache;
}

static int v90_t3_phase_no_marks(void)
{
    static int cache = -1;

    if (cache < 0)
    {
        const char *v = getenv("ME_V90_PHASE_NO_MARKS");

        cache = (v  &&  atoi(v) != 0)  ?  1  :  0;
    }
    /*endif*/
    return cache;
}

#define V90_T3_PH_SET(s, i)  \
    ((s)->v90_t3_phase_shortlist[((i) >> 3) & 31] |= (uint8_t) (1u << ((i) & 7)))
#define V90_T3_PH_GET(s, i)  \
    (((s)->v90_t3_phase_shortlist[((i) >> 3) & 31] >> ((i) & 7)) & 1)

int v34_rx_trace_diagnostics(void)
{
    static int cache = -1;

    return v34_diag_flag("V34_TRACE", &cache);
}

/* Append one equalized training symbol to the file named by the environment
   variable, if it is set.
 *
 * A warning about what these dumps will and will not tell you.  4th-power
 * coherence over the Phase 4 TRN dump looks like a receiver-health metric and
 * is not one: measured at 3200 baud u-law it reads 0.39-0.47 both in a run
 * that completes with zero payload errors and in one that never trains at all.
 * It was read as one here, and the conclusion it supported -- that the
 * constellation collapses at the Phase 3 -> Phase 4 seam -- is wrong.  The
 * apparent collapse was comparing ~130 Phase 3 symbols, which this dump only
 * emits from inside a scoring block and so samples favourably, against 4800
 * Phase 4 symbols spanning stretches where the far end is not sending TRN.
 * Nor is the MP-stage decision error on the [EQ] line a substitute.  It does
 * separate those two particular runs (0.077 median passing against 0.583
 * failing), but its target angle is integrated from the *received* dibit --
 * phase4_da_expected_ang += data_bits << 30 -- so it measures how tightly the
 * symbols cluster on the 4-point grid and says nothing about whether the
 * dibits are right.  3429 baud reads 0.048 and 2800 reads 0.042 while neither
 * ever completes training.
 *
 * Check any metric taken from here against a known-passing *and* a
 * known-failing run before believing it.  Two in a row failed that test. */
float v34_rx_eq_tap_energy(const v34_rx_state_t *s, float *main_tap)
{
    int i;
    float e = 0.0f;

    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
        e += s->eq_coeff[i].re*s->eq_coeff[i].re + s->eq_coeff[i].im*s->eq_coeff[i].im;
    /*endfor*/
    *main_tap = sqrtf(s->eq_coeff[V34_EQUALIZER_PRE_LEN].re*s->eq_coeff[V34_EQUALIZER_PRE_LEN].re
                    + s->eq_coeff[V34_EQUALIZER_PRE_LEN].im*s->eq_coeff[V34_EQUALIZER_PRE_LEN].im);
    return e;
}

void v34_rx_dump_training_symbol(const char *env_name,
                                     const char **path_cache,
                                     int calling_party,
                                     int index,
                                     float re,
                                     float im,
                                     long rx_power,
                                     float tap_energy,
                                     float main_tap,
                                     int eq_put_step,
                                     int timing_corr)
{
    FILE *f;

    if (*path_cache == NULL)
        *path_cache = getenv(env_name)  ?  getenv(env_name)  :  "";
    /*endif*/
    if ((*path_cache)[0] == '\0')
        return;
    /*endif*/
    if ((f = fopen(*path_cache, "a")) == NULL)
        return;
    /*endif*/
    fprintf(f, "%s %d %.6f %.6f %ld %.6f %.6f %d %d\n",
            calling_party  ?  "caller"  :  "answer", index, re, im, rx_power,
            tap_energy, main_tap, eq_put_step, timing_corr);
    fclose(f);
}

#endif

/* Phase 3 RX audio capture for offline analysis */
static FILE *phase3_rx_dump_fp = NULL;
static int phase3_rx_dump_count = 0;
#define PHASE3_RX_DUMP_SAMPLES 24000  /* 3 seconds */

#include "v22bis_rx_1200_rrc.h"
#include "v22bis_rx_2400_rrc.h"

#include "v34_rx_2400_low_carrier_rrc.h"
#include "v34_rx_2400_high_carrier_rrc.h"
#include "v34_rx_2743_low_carrier_rrc.h"
#include "v34_rx_2743_high_carrier_rrc.h"
#include "v34_rx_2800_low_carrier_rrc.h"
#include "v34_rx_2800_high_carrier_rrc.h"
#include "v34_rx_3000_low_carrier_rrc.h"
#include "v34_rx_3000_high_carrier_rrc.h"
#include "v34_rx_3200_low_carrier_rrc.h"
#include "v34_rx_3200_high_carrier_rrc.h"
#include "v34_rx_3429_rrc.h"

#include "v34_local.h"
#include "v34_tables.h"
#include "v34_superconstellation_map.h"
#include "v34_convolutional_coders.h"
#include "v34_shell_map.h"
#include "v34_probe_signals.h"

/*! V.34 12.4 control channel PPh acquisition.  PPh is 32 symbols at 600 baud,
    so a decay giving the correlator a memory of roughly one PPh and a minimum
    of two 8-symbol periods before it may fire keeps a decision inside the
    signal that carries it. */
/*! The correlator's memory is matched to PPh's own length -- 1/(1 - 0.985) is
    about 64 T/2 steps, which is the 32 symbols of 10.2.4.5 -- so the whole
    signal contributes and the silence before it has decayed away.
    The decision is taken on the winning PHASE being stable, not on the score
    alone: the control channel has no equalizer and its band edge timing
    recovery gets nothing to converge on until PPh itself starts, so the
    normalised score tops out well short of 1 and how far short of it depends
    on where in the eye this receiver happened to start.  At 3200 baud it
    peaked at 0.742 against 0.805 at 3429 -- with the same, correct, phase
    winning every step in both.  A score gate alone therefore separates the
    symbol rates from each other rather than a real PPh from noise, which a
    stable argmax does not. */
#define PPH_ACQUIRE_MIN_BAUDS   16
#define PPH_ACQUIRE_DECAY       0.94f
#define PPH_ACQUIRE_SCORE_MIN   0.55f
#define PPH_ACQUIRE_HOLD_STEPS  8

/*! The half-duplex recipient leaves the primary channel for the control
    channel on 12.4.1.1's 70 +/- 5 ms of silence.  32 symbols is 10 ms at 3200
    baud and 13 ms at 2400, comfortably inside it, and far longer than any gap
    a live TRN symbol can produce. */
/*! The control channel gets its own AGC.  It used to run on whatever the
    PRIMARY channel's AGC happened to be left at, which on the source is the
    untouched reset default -- 12.3.1 has the source TRANSMITTING throughout
    Phase 3, so its receiver adapts to nothing at all -- and that put its
    control channel symbols at |z| ~ 4.4 against the recipient's 1.43.  The
    differential decode does not care about gain, but `cc_symbol_sync()`'s
    band edge timing loop is fed the scaled sample and its loop gain is
    therefore proportional to it: at 4.4 the timing jitters enough to put
    isolated symbols over the decision boundary.  Measured on the 12.4 control
    channel data of the half-duplex loopback, 3000 baud u-law, with the far
    end's generator as truth: 87 bit errors at the inherited 0.0017 and zero
    at 0.001, 0.00055 and 0.0003 alike.  PPh, ALT and MPh are all constant
    modulus, so the loop has good material to converge on before any data. */
#define CC_AGC_TARGET_MAG           1.4f
#define CC_AGC_ADAPT_SHIFT          6

#define HDX_TRN_END_SILENCE_BAUDS   32
#define HDX_TRN_END_SILENCE_MAG2    0.04f

#if !defined(M_PI)
/* C99 systems may not define M_PI */
#define M_PI 3.14159265358979323846264338327
#endif

#if defined(SPANDSP_USE_FIXED_POINT)
#define FP_FACTOR                       4096
#define FP_SHIFT_FACTOR                 12
#endif

#define FP_Q9_7_TO_F(x)                 ((float) x/128.0f)

#define CARRIER_NOMINAL_FREQ            1800.0f
#define EQUALIZER_DELTA                 0.21f
#define EQUALIZER_SLOW_ADAPT_RATIO      0.1f

#define V34_TRAINING_SEG_1              0
#define V34_TRAINING_SEG_4              0
#define V34_TRAINING_END                0
#define V34_TRAINING_SHUTDOWN_END       0
#define MP_LOCK_SCORE_MIN               15
#define MP_PREAMBLE_SCORE_MIN           18
#define MP_HINT_LOCK_SCORE_MIN          14
#define MP_PREAMBLE_WAIT_BITS           800
#define MP_PRELOCK_PREAMBLE_WAIT_BITS   160
#define PHASE4_MP_TIMEOUT_BAUDS         20000
/* V.90 §9.4.1 lets the digital modem wait 15 s plus round-trip delays for
   B1 after INFO1a.  Before CPt is accepted, silence is therefore not enough
   evidence that the analogue modem has retrained: it may still be between
   repeated CPt attempts.  Keep the legacy silence heuristic out of that
   acquisition window; the independent Tone A detector below still handles a
   genuine §9.5.2.1 retrain immediately. */
#define PHASE4_CP_ACQUISITION_WAIT_SECONDS 15
#define MP_TRN_PRELOCK_SCORE_MIN        70

/* Mapping from the Phase-4 TRN SNR measurement to a Table 16 rate index:
   rate_n = floor((snr_db - offset)/step), so 2400*n bit/s.  Calibrated in
   docs/v34_data_mode_rates.md. */
#define V34_TRN_SNR_RATE_OFFSET_DB      4.0
#define V34_TRN_SNR_RATE_STEP_DB        2.0
#define PHASE4_TRN_SNR_BLOCK            64

/* Data-mode receive SNR to the rate the line will carry, in bits per symbol:
   bits = (snr_db + offset)/slope.  6 dB per bit is the QAM ladder's slope;
   the offset is calibrated so that the one live channel whose behaviour is
   known -- 18 dB, artifacts/v34-rate-* -- accepts 14400 at 3000 baud and
   refuses 16800, which is what those three calls did. */
#define MP_EARLY_START_ERR_MAX          2
#define MP_EARLY_START_ERR_FRAME_LIMIT  85
#define MP1_START_ERR_ACCEPT_MAX        3
#define MP_BOUNDARY_BRUTEFORCE_MAX_CHANGES 4
#define MP_HINT_STRICT_REJECTS          2
#define MP_HINT_MAX_NOLOCKS             3
/* Minimum equalizer-output |y|^2 required before attempting a new MP
   preamble lock (squared magnitude, so no sqrt needed at the call site).
   Prevents locking onto near-zero-energy/silent samples: real in-lock MP
   symbols measured live against d-modem/slmodemd sit around |y|~0.03-2.9
   (median ~1.1), while true silence reads as an exact 0.0 (bit-identical,
   not just small). 0.01^2 sits two orders of magnitude below the weakest
   observed real symbol while still comfortably clearing float noise on
   true silence. NOTE: this and MP_LOCK_SETTLE_BAUDS below do not fix the
   live MP CRC failure against d-modem/slmodemd -- that failure's actual
   root cause turned out to be d-modem discarding its Phase 3/4 state and
   restarting the whole handshake (Phase 1/2 tones) to retry V.90 after
   its own constellation designer rejects our CPt (see "Phase 4 MP frame
   CRC failure" in rig/README.md), so the "signal" these gates wait for
   is d-modem's handshake-retrain tone, not a real MP0 frame. Kept anyway
   as correct, generally-useful defensive behavior (don't lock onto
   silence or a still-settling equalizer), not because it resolves this
   bug. */
#define MP_LOCK_MIN_SIGNAL_MAG2         0.0001f
/* Consecutive above-threshold bauds required before a preamble lock is
   attempted, once real signal has appeared. Live measurement showed the
   CMA equalizer is still actively re-adapting its gain well past 100
   bauds after the initial silence->signal transition (coefmag kept
   climbing through baud 5537-5600 with no clear plateau); 200 bauds
   still let a lock land mid-readaptation. 400 (~167ms at 2400 baud) is
   a conservative middle ground -- see the note on MP_LOCK_MIN_SIGNAL_MAG2
   above for why this does not actually fix the live CRC failure. */
#define MP_LOCK_SETTLE_BAUDS            400
#define PHASE3_PP_TRAIN_BAUDS           232
#define PHASE3_TRN_REFINE_BAUDS         256
#define PHASE3_PP_ACQUIRE_MIN_BAUDS     48
#define PHASE3_PP_ACQUIRE_HOLD_BAUDS    12
#define PHASE3_PP_ACQUIRE_SCORE_MIN     650
#define PHASE3_PP_ONSET_SCORE           400
#define PHASE3_PP_ONSET_NOMINAL         40
#define PHASE3_PP_ONSET_MAX_SKIP        160
#define PHASE3_PP_ONSET_MIN_RISE        16

static int v34_pp_onset_trim_enabled(void);
static float v34_p4_trn_coh_gate(void);
#define PHASE3_PP_ACQUIRE_DECAY         0.98f
#define V34_AGC_POWER_MIN               100000
#define V34_AGC_SCALING_MIN             0.00001f
#define V34_AGC_SCALING_MAX             0.01f
#define PHASE3_PP_MAG_SANITY_MAX        20.0f
#define V34_DEBUG_INFO_RX_DIAG          1
#define V34_DEBUG_MP_DIBIT_DIST         0
#define PHASE4_MP_NOLOCK_LOG_INTERVAL   800
#define PHASE4_MP_BAUD_LOG_INTERVAL     400
#define PHASE4_MP_DIBIT_LOG_INTERVAL    1600
#define PHASE4_MP_REJECT_DETAIL_LOG_INTERVAL 3200
/* Phase 4 CMA runs until the level estimate settles; see phase4_cma_converged(). */
#define PHASE4_CMA_SETTLE_BAUDS         128
#define PHASE4_CMA_SETTLE_TOL           0.05f
#define PHASE4_CMA_MAX_BAUDS            100000
/* Phase 4 TRN: hand the equalizer from blind CMA to decision-directed LMS
   after this many bauds of TRN. */
#define V34_P4_TRN_DD_START             256
/* 9.6 CP window: dead blocks before re-arming the one-shot CP acquisition
   (v90_reneg_cp_reacquire_enabled(), off). */
#define V90_RENEG_CP_REACQUIRE_BLOCKS   2
/* The Phase 3 S/J detector constants moved to v34rx_internal.h with the stage
   that uses them; their rationale travelled with them. */
#define PHASE3_PP_ACQUIRE_LOG_INTERVAL  256
#define PHASE3_PP_BAUD_LOG_INTERVAL     192

/* Test instrument: behave as if the receiver had entered PHASE3_WAIT_S this
   many symbols later than it did, so the Ja capture's differential chain
   starts somewhere else on the SAME recording.  The capture is anchored to
   the symbol decisions and to phase3_j_prev_z/prev_valid, which chain forward
   from the first captured symbol; the descrambler is self-synchronising and
   so is not an anchor.  Sweeping this is the controlled version of the
   comparison in docs/v90_phase3_s_and_rbs_false_positive.md 35a, which varied
   the recording and the decoder state together and could attribute nothing.
   Zero (the default) leaves the receiver exactly as it was. */

enum
{
    TRAINING_TX_STAGE_NORMAL_OPERATION_V34 = 0,
    TRAINING_TX_STAGE_NORMAL_OPERATION_CC = 1,
    TRAINING_TX_STAGE_PARKED
};

static const char *v34_rx_stage_to_str(int stage)
{
    switch (stage)
    {
    case V34_RX_STAGE_INFO0: return "INFO0";
    case V34_RX_STAGE_INFOH: return "INFOH";
    case V34_RX_STAGE_INFO1C: return "INFO1C";
    case V34_RX_STAGE_INFO1A: return "INFO1A";
    case V34_RX_STAGE_TONE_A: return "TONE_A";
    case V34_RX_STAGE_TONE_B: return "TONE_B";
    case V34_RX_STAGE_L1_L2: return "L1_L2";
    case V34_RX_STAGE_CC: return "CC";
    case V34_RX_STAGE_PRIMARY_CHANNEL: return "PRIMARY_CHANNEL";
    case V34_RX_STAGE_PHASE3_WAIT_S: return "PHASE3_WAIT_S";
    case V34_RX_STAGE_PHASE3_TRAINING: return "PHASE3_TRAINING";
    case V34_RX_STAGE_PHASE3_DONE: return "PHASE3_DONE";
    case V34_RX_STAGE_PHASE4_S: return "PHASE4_S";
    case V34_RX_STAGE_PHASE4_S_BAR: return "PHASE4_S_BAR";
    case V34_RX_STAGE_PHASE4_TRN: return "PHASE4_TRN";
    case V34_RX_STAGE_PHASE4_MP: return "PHASE4_MP";
    case V34_RX_STAGE_DATA: return "DATA";
    case V34_RX_STAGE_V90_CP: return "V90_CP";
    default: return "UNKNOWN";
    }
}

/* The CP stage's taps adapt at STARTUP, the way
   9.6's CP conditioning is allowed to while it finds the level.
   v34_force_v90_phase4_cp_rx() freezes them on the stated assumption that
   "9.4.2.2 assumes the channel is static through this seam", and against the
   RasFinder that assumption is measurably false: with the taps frozen the
   differential decision margin grows from 8.0 deg rms before the CPt pair to
   10.0 through CPt #1, 11.8 and then 22.2 across CPt #2 and 23.1 in SCR (26
   is white), with the mean at 0 and the magnitude pinned at 1.00 -- growing
   phase NOISE while every loop is static -- where an offline demodulation of
   the same tap with a freshly adapted CMA reads 5.75.  9.6's own
   adapt-vs-freeze A/B came out the other way over 28 windows
   (9.6 freezes once the level has settled), so this is measured here and
   not assumed by analogy.

   DEFAULT ON since 2026-09-30.  Scored on six wired-rig RasFinder calls, each
   replayed fed as whole 160-sample frames and as the two 80-sample halves a
   live pjmedia call delivers (v90_engine_replay --split), with the CP-stage
   eye criterion below: CPt accepted in 11 of 12 cells frozen, 12 of 12
   adapting.  The one frozen miss is a call whose Phase 3 eye chooser flipped
   on a half-silent window (sums 45.7 vs 43.5) and saved taps that leave
   BOTH T/2 phases white (26 deg) at the CP stage -- no symbol instant can
   rescue that, adapting on the constant-modulus CPt does.  That call missed
   all 11 of the peer's CRC-valid CPt frames live and was retrained out of
   Ri. */
static bool v90_startup_cp_adapt(void)
{
    return true;
}

/* Step size: the CP stage adapts at the ordinary CMA step.  Adaptation at that step
   is measurably ON but ineffective: over the RasFinder's 10432-symbol CP
   stage the main tap creeps 0.74367 -> 0.75576, 1.6%, while the channel
   steps at symbol ~4000 and the differential margin goes 5.4 -> 21 deg and
   stays there.  The same audio through a fresh 41-tap CMA, fully converged
   on that region, reads 5.75 deg -- so a solution exists that this loop does
   not reach at this rate.

   RE-CONVERGING WAS TRIED AND IT DOES NOT WORK.  Swept on that recording,
   as CPt #1 margin / post-step margin / post-step |z| sd/mean / CRC-valid
   frames:

     mu=1     5.90 / 22.29 / 0.323 / CPt#1 OK      <- default, and the best
     mu=5     6.20 / 26.27 / 0.638 / CPt#1 OK
     mu=20   26.07 / 25.84 / 0.642 / NONE
     mu=50   26.22 / 26.12 / 0.704 / NONE
     mu=100  26.35 / 25.90 / 1.750 / NONE
     mu=300  25.67 / 25.99 / 0.768 / NONE

   26 deg is white.  Above mu=1 CMA's phase-blind gradient walks the trained
   solution off -- the main tap goes 0.74367 -> 0.39985 at mu=20 -- and
   destroys the CPt #1 that the ordinary step decodes cleanly, which is the
   failure this file's other CMA comments already describe.  Throwing the
   Phase-3 solution away instead (equalizer_reset(), the renegotiation
   path's choice) is also worse, not better: 9.61 / 22.28 / 0.317, so CPt #1
   degrades from 5.90 and the post-step region does not move at all.

   So the post-step degradation is immune to EVERY equalizer treatment tried
   -- freeze, slow adapt, fast adapt, reset-and-adapt -- and is therefore not
   an equalizer problem, and the ordinary step (mu=1 above) stays. */

static bool v34_rx_stage_is_phase4_frame(int stage)
{
    return stage == V34_RX_STAGE_PHASE4_MP
        || stage == V34_RX_STAGE_V90_CP;
}

static bool v34_rx_stage_is_primary_training(int stage)
{
    return (stage >= V34_RX_STAGE_PHASE3_WAIT_S
            && stage <= V34_RX_STAGE_PHASE4_MP)
        || stage == V34_RX_STAGE_V90_CP;
}

/* Where a peer-initiated retrain has to be watched for.
 *
 * V.90 9.5.1.2 ("After detecting Tone A for more than 50 ms, the digital
 * modem shall ...") and V.34 11.5.1.2 carry no phase qualifier at all, and
 * 9.6 puts a rate renegotiation -- which a peer may abandon into a retrain --
 * "at any time during data mode".  The detector below was nevertheless gated
 * on the training stages alone, so a peer that gave up on our upstream and
 * retrained during DATA held Tone A into a receiver that was not listening
 * for it, and the call died instead of resynchronising. */
static bool v34_rx_stage_watches_retrain(int stage)
{
    return v34_rx_stage_is_primary_training(stage)
        || stage == V34_RX_STAGE_DATA;
}

static const char *v34_event_to_str(int event)
{
    switch (event)
    {
    case V34_EVENT_NONE: return "NONE";
    case V34_EVENT_TONE_SEEN: return "TONE_SEEN";
    case V34_EVENT_REVERSAL_1: return "REVERSAL_1";
    case V34_EVENT_REVERSAL_2: return "REVERSAL_2";
    case V34_EVENT_REVERSAL_3: return "REVERSAL_3";
    case V34_EVENT_INFO0_OK: return "INFO0_OK";
    case V34_EVENT_INFO0_BAD: return "INFO0_BAD";
    case V34_EVENT_INFO1_OK: return "INFO1_OK";
    case V34_EVENT_INFO1_BAD: return "INFO1_BAD";
    case V34_EVENT_INFOH_OK: return "INFOH_OK";
    case V34_EVENT_INFOH_BAD: return "INFOH_BAD";
    case V34_EVENT_L2_SEEN: return "L2_SEEN";
    case V34_EVENT_S: return "S";
    case V34_EVENT_J: return "J";
    case V34_EVENT_J_DASHED: return "J_DASHED";
    case V34_EVENT_PHASE4_TRN_READY: return "PHASE4_TRN_READY";
    case V34_EVENT_PPH: return "PPH";
    default: return "UNKNOWN";
    }
}

static const char *v34_demodulator_to_str(int mod)
{
    switch (mod)
    {
    case V34_MODULATION_V34: return "V34";
    case V34_MODULATION_CC: return "CC";
    case V34_MODULATION_TONES: return "TONES";
    case V34_MODULATION_L1_L2: return "L1_L2";
    case V34_MODULATION_SILENCE: return "SILENCE";
    default: return "UNKNOWN";
    }
}

static void v34_rx_log_state_change(v34_rx_state_t *s)
{
    if (s->last_logged_stage != s->stage
        || s->last_logged_demodulator != s->current_demodulator)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - stage=%s (%d) demod=%s (%d)\n",
                 v34_rx_stage_to_str(s->stage), s->stage,
                 v34_demodulator_to_str(s->current_demodulator), s->current_demodulator);
        s->last_logged_stage = s->stage;
        s->last_logged_demodulator = s->current_demodulator;
    }
    if (s->last_logged_event != s->received_event)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - event=%s (%d)\n",
                 v34_event_to_str(s->received_event), s->received_event);
        s->last_logged_event = s->received_event;
    }
}

enum
{
    V34_MP_DIAG_STATE_NONE = -1,
    V34_MP_DIAG_STATE_DET_SYNC = 0,
    V34_MP_DIAG_STATE_DET_INFO = 1,
    V34_MP_DIAG_STATE_COMPLETE = 2
};

static const char *v34_mp_diag_state_to_str(int state)
{
    switch (state)
    {
    case V34_MP_DIAG_STATE_DET_SYNC: return "DET_SYNC";
    case V34_MP_DIAG_STATE_DET_INFO: return "DET_INFO";
    case V34_MP_DIAG_STATE_COMPLETE: return "COMPLETE";
    default: return "NONE";
    }
}

static void v34_rx_log_mp_diag_state(v34_rx_state_t *s, int state, const char *reason)
{
    if (s->last_logged_mp_diag_state == state)
        return;
    /*endif*/
    if (reason && reason[0])
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4 MP microstate=%s (%s)\n",
                 v34_mp_diag_state_to_str(state), reason);
    }
    else
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4 MP microstate=%s\n",
                 v34_mp_diag_state_to_str(state));
    }
    /*endif*/
    s->last_logged_mp_diag_state = state;
}

static const v34_rx_shaper_t *v34_rx_shapers_re[6][2] =
{
    {&rx_pulseshaper_2400_low_carrier_re, &rx_pulseshaper_2400_high_carrier_re},
    {&rx_pulseshaper_2743_low_carrier_re, &rx_pulseshaper_2743_high_carrier_re},
    {&rx_pulseshaper_2800_low_carrier_re, &rx_pulseshaper_2800_high_carrier_re},
    {&rx_pulseshaper_3000_low_carrier_re, &rx_pulseshaper_3000_high_carrier_re},
    {&rx_pulseshaper_3200_low_carrier_re, &rx_pulseshaper_3200_high_carrier_re},
    {&rx_pulseshaper_3429_re, &rx_pulseshaper_3429_re}
};

static const v34_rx_shaper_t *v34_rx_shapers_im[6][2] =
{
    {&rx_pulseshaper_2400_low_carrier_im, &rx_pulseshaper_2400_high_carrier_im},
    {&rx_pulseshaper_2743_low_carrier_im, &rx_pulseshaper_2743_high_carrier_im},
    {&rx_pulseshaper_2800_low_carrier_im, &rx_pulseshaper_2800_high_carrier_im},
    {&rx_pulseshaper_3000_low_carrier_im, &rx_pulseshaper_3000_high_carrier_im},
    {&rx_pulseshaper_3200_low_carrier_im, &rx_pulseshaper_3200_high_carrier_im},
    {&rx_pulseshaper_3429_im, &rx_pulseshaper_3429_im}
};

#if defined(SPANDSP_USE_FIXED_POINT)
#define complex_sig_set(re,im) complex_seti16(re,im)
#define complex_sig_t complexi16_t
#else
#define complex_sig_set(re,im) complex_setf(re,im)
#define complex_sig_t complexf_t
#endif

#if defined(SPANDSP_USE_FIXED_POINT)
#define TRAINING_SCALE(x)       ((int16_t) (32767.0f*x + ((x >= 0.0)  ?  0.5  :  -0.5)))
#else
#define TRAINING_SCALE(x)       (x)
#endif

static const complex_sig_t zero = {TRAINING_SCALE(0.0f), TRAINING_SCALE(0.0f)};

static void process_cc_half_baud(v34_rx_state_t *s, const complexf_t *sample);
static void process_primary_half_baud(v34_rx_state_t *s, const complexf_t *sample);
static void process_primary_symbol(v34_rx_state_t *s, const complexf_t *sym);
static void l1_l2_analysis_init(v34_rx_state_t *s);
static void equalizer_reset(v34_rx_state_t *s);
static complexf_t equalizer_get(v34_rx_state_t *s);
static bool v90_upstream_t2_enabled(void);
bool v34_rx_t2_data_path(const v34_rx_state_t *s);
void v34_rx_tune_equalizer(v34_rx_state_t *s, const complexf_t *z, const complexf_t *target);
static void create_godard_coeffs(ted_t *coeffs, float carrier, float baud_rate, float alpha);
static int set_tx_trellis_mode(v34_state_t *s, int trellis_size);
static int set_rx_trellis_mode(v34_state_t *s, int trellis_size);
SPAN_DECLARE(void) v34_put_mapping_frame(v34_rx_state_t *s, int16_t bits[16]);

int v34_rx_descramble(v34_rx_state_t *s, int in_bit)
{
    int out_bit;

    /* One of the scrambler taps is a variable, so it can be adjusted for caller or answerer operation. */
    out_bit = (in_bit ^ (s->scramble_reg >> s->scrambler_tap) ^ (s->scramble_reg >> (23 - 1))) & 1;
    s->scramble_reg = (s->scramble_reg << 1) | in_bit;
    return out_bit;
}
/*- End of function --------------------------------------------------------*/

static void v34_reset_rx_data_frame_state(v34_rx_state_t *s, int super_frame);
static bool v90_t3_phase_evidence_ok(v34_rx_state_t *s);
static complexf_t v90_t3_raw_get_frac(const v34_rx_state_t *s,
                                      int64_t index,
                                      float frac);

int v34_rx_descramble_reg(uint32_t *reg, int scrambler_tap, int in_bit)
{
    int out_bit;

    out_bit = (in_bit ^ (*reg >> scrambler_tap) ^ (*reg >> (23 - 1))) & 1;
    *reg = (*reg << 1) | in_bit;
    return out_bit;
}
/*- End of function --------------------------------------------------------*/

/* Descramble one data bit, and on the V.90 upstream also run the other
   clause-7 polynomial in parallel.  An idle DTE sends marks, so the correct
   descrambler yields ones and the wrong one yields a white stream -- which
   is the difference between upstream payload and upstream noise, and is not
   otherwise visible from outside. */
static int v90_t3_probe_descramble(v34_rx_state_t *s, int in_bit)
{
    int out_bit = v34_rx_descramble(s, in_bit);

    if (s->v90_t3_acquired  &&  !s->v90_t3_suppress_output)
    {
        /* Raw descrambled bits, before any V.14 framing.  An idle DTE reads
           100% ones here even if the bits within a data frame are permuted,
           because all-ones is invariant under a permutation -- and so is the
           encode/decode loopback test, which uses the same order at both
           ends.  Only a foreign peer sending real characters can show it, so
           dump the stream and align it offline against the known pattern.
           ME_V90_UPSTREAM_BIT_DUMP=<path>. */
        if (s->v90_t3_bit_dump == NULL  &&  !s->v90_t3_bit_dump_tried)
        {
            const char *path = V34_DIAG_GETENV("ME_V90_UPSTREAM_BIT_DUMP");

            s->v90_t3_bit_dump_tried = true;
            if (path && *path)
                s->v90_t3_bit_dump = fopen(path, "wb");
            /*endif*/
        }
        /*endif*/
        if (s->v90_t3_bit_dump)
        {
            s->v90_t3_dump_byte = (uint8_t) ((s->v90_t3_dump_byte << 1)
                                             | (out_bit & 1));
            if (++s->v90_t3_dump_bits == 8)
            {
                fputc(s->v90_t3_dump_byte, s->v90_t3_bit_dump);
                s->v90_t3_dump_bits = 0;
            }
            /*endif*/
        }
        /*endif*/
        int alt_tap = (s->scrambler_tap == 4) ? 17 : 4;
        int alt_bit = v34_rx_descramble_reg(&s->v90_t3_alt_scramble, alt_tap, in_bit);

        if (s->v90_t3_first_bits < 96)
        {
            s->v90_t3_first_word = (s->v90_t3_first_word << 1) | out_bit;
            if ((++s->v90_t3_first_bits % 32) == 0)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 upstream first data bits %d-%d: %08X\n",
                         s->v90_t3_first_bits - 32, s->v90_t3_first_bits - 1,
                         (unsigned) s->v90_t3_first_word);
            }
            /*endif*/
        }
        /*endif*/
        s->v90_t3_ones += out_bit;
        s->v90_t3_alt_ones += alt_bit;
        {
            /* Start bit nine places back, stop bit here. */
            int start = (s->v90_t3_v14_hist >> 9) & 1;
            int ph = (int) (s->v90_t3_v14_bits % 10);

            if (s->v90_t3_v14_bits >= 9)
            {
                s->v90_t3_v14_count[ph]++;
                if (start == 0  &&  out_bit == 1)
                    s->v90_t3_v14_ok[ph]++;
                /*endif*/
            }
            /*endif*/
            s->v90_t3_v14_hist = ((s->v90_t3_v14_hist << 1) | out_bit) & 0x3FF;
            s->v90_t3_v14_bits++;
        }
        /* While searching, judge on a short window so the whole phase space
           fits inside the peer's idle period; once locked, report on a long
           one.

           Short means SHORT.  The discrimination on an idle line is 100% ones
           against 50%, so 800 bits separates the two by twenty standard
           deviations -- and 800 bits is what makes the search affordable:
           112 candidates in nine seconds rather than forty-five.  Forty-five
           was longer than the intervals this peer leaves between the slips
           that disturb the phase in the first place, so the sweep was losing
           a race it did not have to be in. */
        if (++s->v90_t3_bit_count
                >= (s->v90_t3_sf_locked
                    ? 4800
                    : (s->v90_t3_confirming ? V34_V90_T3_CONFIRM_BITS : 1200)))
        {
            int ones_pct = 100*s->v90_t3_ones/s->v90_t3_bit_count;
            int shell_pct = s->v90_t3_shell_frames
                          ? 100*s->v90_t3_shell_bad/s->v90_t3_shell_frames
                          : -1;
            int v14_pct = 0;
            int v14_ratio = 0;
            int sweep_score = 0;

            /* A second, content-independent lock metric.  The ones fraction
               only says anything while the peer's line is mostly idle: with
               its DTE sending, no phase reads high and the sweep cannot tell
               a good phase from a bad one -- measured, a whole batch of
               calls with the peer transmitting from the first second never
               locked at all.  V.14 characters are ten bits with a zero start
               and a one stop, so a correctly framed stream has one bit phase
               in ten where both hold.

               An absolute threshold cannot work: on a mostly-idle line the
               best phase reads 0-11% even when the decode is perfect,
               because idle marks contain no start bits.  The level is set by
               how busy the line is; the RATIO between the best phase and the
               rest is what says whether the framing is real. */
            {
                int score[10];
                int total = 0;
                int best_ph = 0;

                for (int ph = 0;  ph < 10;  ph++)
                {
                    int n = s->v90_t3_v14_count[ph];

                    score[ph] = n ? 100*s->v90_t3_v14_ok[ph]/n : 0;
                    total += score[ph];
                    if (score[ph] > score[best_ph])
                        best_ph = ph;
                    /*endif*/
                    s->v90_t3_v14_ok[ph] = 0;
                    s->v90_t3_v14_count[ph] = 0;
                }
                /*endfor*/
                v14_pct = score[best_ph];
                /* Judge the best phase against the others, not against an
                   absolute figure.  How high it can possibly read depends on
                   how busy the line is -- at a fifth occupancy a correct
                   phase reads about 20% and the wrong ones about 5%, because
                   idle marks have no start bit to find -- so the ratio is
                   the discriminator and the level is not. */
                v14_ratio = (total > score[best_ph])
                          ? (10*score[best_ph])/((total - score[best_ph])/9 + 1)
                          : 0;
            }

            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                     "Rx - V.90 upstream DATA bits: t=%.1fs tap=%d ones=%d%%, "
                     "tap=%d ones=%d%% (over %d bits, slips %d, freq %+.6f, "
                     "sym err %.3f, V.14 %d%% at %dx, shell bad %d%%)\n",
                     (double) s->v90_t3_data_symbols
                         /baud_rate_parameters[s->baud_rate].baud_rate,
                     s->scrambler_tap, ones_pct,
                     alt_tap,
                     100*s->v90_t3_alt_ones/s->v90_t3_bit_count,
                     s->v90_t3_bit_count, s->v90_t3_gardner.slips,
                     s->v90_t3_gardner.freq,
                     s->v90_t3_decision_count
                         ? s->v90_t3_decision_err/s->v90_t3_decision_count
                         : 0.0f,
                     v14_pct, v14_ratio/10,
                     (shell_pct >= 0) ? shell_pct : 0);
            /* Report the frame count beside the percentage.  Without it a
               window with no mapping frames at all is indistinguishable from
               a window in which every frame was well formed: both print 0%,
               and the release rule treats them as opposite evidence.  On a
               live call 1964 of 2829 windows printed "shell bad 0%" while
               their symbols were white, which reads as the strongest possible
               confirmation of a phase that was in fact wrong. */
            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                     "Rx - V.90 upstream shell: %d bad of %d frames%s\n",
                     s->v90_t3_shell_bad, s->v90_t3_shell_frames,
                     (shell_pct < 0) ? " (no evidence this window)" : "");
            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                     "Rx - V.90 upstream carrier: freq %+.5f rad/sym "
                     "(%d decision-directed, %d fourth-power updates)\n",
                     s->v90_t3_carrier.freq,
                     s->v90_t3_carrier.dd_updates,
                     s->v90_t3_carrier.nda_updates);
            /* An idle DTE sends marks, so a correctly framed stream reads
               near 100% ones.  A superframe phase off by k reads about 57%:
               one data frame in j lands on the right index and the other six
               decode to noise (1/7 x 100 + 6/7 x 50).  Measured, not assumed
               -- 55-56% held steady for a whole call while the symbols
               themselves were clean at 0.098.  j is only 7 wide, so search
               it against the marks rather than deriving it. */
            /* Locks are not forever.  Measured live, the upstream held 100%
               ones for about twenty-seven seconds and then fell back to 50%
               as the timing loop started slipping several times a second --
               and with the lock held, nothing looked for the phase again and
               the rest of the call was lost.  Release it and let the sweep
               run once more. */
            /* A lock is cheap to give up now that a sweep costs ten seconds
               rather than forty-five, and staying wrong is not cheap at all.
               The old cap of eight relocks meant a call that had used them up
               sat on a phase it had already measured as bad for the rest of
               its length. */
            /* Judge a standing lock on the shell indices where they are
               available, and only fall back on the marks where they are not.
               The marks cannot tell a correct decode of real traffic from
               noise -- see V34_V90_T3_SHELL_BAD_PCT -- and releasing on them
               was throwing away working locks the moment the peer's DTE said
               anything. */
            if (s->v90_t3_sf_locked
                &&
                v90_t3_phase_evidence_ok(s)
                &&
                ((shell_pct >= 0)
                     ? (shell_pct >= V34_V90_T3_SHELL_BAD_PCT)
                     : (ones_pct < 70  &&  v14_ratio < 25))
                &&
                s->v90_t3_relocks < 200)
            {
                s->v90_t3_sf_locked = false;
                s->v90_t3_sf_tries = 0;
                s->v90_t3_relocks++;
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 upstream lost frame phase (%d%% ones); "
                         "sweeping again (release %d)\n",
                         ones_pct, s->v90_t3_relocks);
            }
            /*endif*/
            /* Only sweep where the metric can tell one phase from another.
               The ones fraction only discriminates while the peer's line has
               idle in it; on a busy line every phase reads the same, so the
               sweep steps for ever and destroys a decode that may already be
               correct.  That is not speculation -- with the sweep free to
               run, the offline T/3 regression, whose phase is right by
               construction and whose traffic is pseudo-random, fails.  So
               require evidence that idle exists on this call before touching
               anything. */
            if (ones_pct >= 60)
                s->v90_t3_idle_seen = true;
            /*endif*/
            /* Score this window for the phase sweep.  Both terms say "the
               bits have structure a wrong phase would destroy": marks on an
               idle line, and V.14 start/stop framing on a busy one.  They are
               measured on the same traffic for every candidate, so they are
               comparable across candidates even where neither is large. */
            /* Rank candidates by shell consistency first: it separates the
               right grouping from the wrong one whatever the peer is sending,
               where both the other terms need the traffic to cooperate.  The
               weight makes one percent of bad frames outweigh any difference
               the marks can show. */
            sweep_score = ones_pct
                        + ((v14_pct >= 5) ? v14_ratio/10 : 0)
                        - ((shell_pct > 0) ? 4*shell_pct : 0);
            if (!s->v90_t3_sf_locked)
            {
                /* Lock on marks only when they are really marks.  A correct
                   phase on an idle line reads 99-100%; 75-87% is a phase that
                   is partly right, and accepting one stopped the sweep and
                   then spent a 20000-bit window -- two seconds -- proving it
                   wrong before releasing.  Measured on round1: 35 locks, most
                   of them in the 75-87% band, and the sweep restarting after
                   each. */
                if (!v90_t3_phase_no_marks()
                    &&
                    (ones_pct >= 90
                     ||
                     (v14_pct >= 5  &&  v14_ratio >= 30)))
                {
                    s->v90_t3_sf_locked = true;
                    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                             "Rx - V.90 upstream frame phase locked "
                             "(%d%% ones, V.14 %d%% at %dx the other phases, "
                             "after %d steps)\n",
                             ones_pct, v14_pct, v14_ratio/10,
                             s->v90_t3_sf_tries);
                }
                else if (s->v90_t3_confirming
                         &&
                         !s->v90_t3_phase_pending)
                {
                    /* A shortlisted candidate, measured over a long window
                       where 9.6.3.3 is decisive.  This is the only lock that
                       owes nothing to what the peer is sending: the marks
                       need an idle line and the V.14 ratio needs a busy one,
                       and a call whose DTE trickles satisfies neither. */
                    int span = s->parms.j*s->parms.p;
                    int next;

                    /* An idle line cannot answer this question, and the
                       measurement says so plainly: displaced five data frames
                       from the phase B1 pins, offsets 5, 6 and 7 all read
                       100% ones while the peer was idle, and only once its
                       DTE began sending did the wrong ones fall to 51% and
                       35%.  Marks are phase-AMBIGUOUS -- a wrong grouping
                       still decodes a continuous mark to a continuous mark --
                       so confirming against them would lock whichever
                       candidate happened to be tried first.  Wait for
                       traffic instead of answering from silence. */
                    if (ones_pct >= 90)
                    {
                        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                                 "Rx - V.90 upstream frame phase offset %d: "
                                 "line idle (%d%% ones), no evidence yet\n",
                                 s->v90_t3_phase_pos, ones_pct);
                    }
                    else if (shell_pct == 0
                        &&
                        s->v90_t3_shell_frames >= V34_V90_T3_CONFIRM_MIN_FRAMES)
                    {
                        s->v90_t3_sf_locked = true;
                        s->v90_t3_confirming = false;
                        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                                 "Rx - V.90 upstream frame phase locked on "
                                 "shell evidence (offset %d, 0 bad of %d "
                                 "frames, %d%% ones, V.14 %d%% at %dx)\n",
                                 s->v90_t3_phase_pos, s->v90_t3_shell_frames,
                                 ones_pct, v14_pct, v14_ratio/10);
                    }
                    else
                    {
                        /* Rejected.  Drop it from the shortlist so a later
                           episode does not pay for it again. */
                        s->v90_t3_phase_shortlist[(s->v90_t3_phase_pos >> 3) & 31]
                            &= (uint8_t) ~(1u << (s->v90_t3_phase_pos & 7));
                        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                                 "Rx - V.90 upstream frame phase offset %d "
                                 "rejected (%d bad of %d frames)\n",
                                 s->v90_t3_phase_pos, s->v90_t3_shell_bad,
                                 s->v90_t3_shell_frames);
                        next = -1;
                        for (int k = 1;  k <= span;  k++)
                        {
                            int cand = (s->v90_t3_phase_pos + k) % span;

                            if (V90_T3_PH_GET(s, cand))
                            {
                                next = cand;
                                break;
                            }
                            /*endif*/
                        }
                        /*endfor*/
                        if (next >= 0)
                        {
                            s->v90_t3_phase_delta = (next - s->v90_t3_phase_pos
                                                     + span) % span;
                            s->v90_t3_phase_pending = true;
                        }
                        else
                        {
                            /* Nothing survived.  Fall back to the coarse
                               winner rather than sitting wherever the
                               confirmation walk ended, and let the ordinary
                               sweep have another episode if it is allowed
                               one. */
                            int delta = ((s->v90_t3_sweep_best_pos
                                          - s->v90_t3_phase_pos) % span + span)
                                        % span;

                            s->v90_t3_confirming = false;
                            if (delta != 0)
                            {
                                s->v90_t3_phase_delta = delta;
                                s->v90_t3_phase_pending = true;
                            }
                            /*endif*/
                            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                                     "Rx - V.90 upstream frame phase "
                                     "confirmation found nothing; back to "
                                     "offset %d\n",
                                     s->v90_t3_sweep_best_pos);
                        }
                        /*endif*/
                    }
                    /*endif*/
                }
                else if (v90_t3_phase_sweep_enabled()
                         &&
                         v90_t3_phase_evidence_ok(s)
                         &&
                         /* Sweeping is the dangerous act, so it needs
                            positive cause.  B1 pins the phase correctly on
                            almost every call -- 18 of 19 idle-start calls
                            decode from step 0 -- and the live busy-start
                            call relgate-r5 ARRIVED on a good phase (1 bad of
                            45 frames, then 0/19, 1/15, 0/16) and was pushed
                            over the 3% threshold by noise on 15-frame
                            windows.  It swept 784 steps and never found its
                            way back: 111.8 s of open eye, no payload.  So
                            require evidence accumulated over
                            V34_V90_T3_WRONG_MIN_FRAMES before starting, and
                            once started let the enumeration finish. */
                         (s->v90_t3_sf_tries > 0
                          ||
                          !v90_t3_sweep_needs_evidence()
                          ||
                          (s->v90_t3_wrong_frames >= V34_V90_T3_WRONG_MIN_FRAMES
                           &&
                           100*s->v90_t3_wrong_bad/s->v90_t3_wrong_frames
                               >= V34_V90_T3_SHELL_BAD_PCT))
                         &&
                         /* Eye health is already judged, one condition
                            up, by v90_t3_phase_evidence_ok() -- and it is
                            judged RELATIVE to what this call's own eye
                            settled at.  This term used to re-test it against
                            the absolute V34_V90_T3_SWEEP_ERR, and that is
                            the wrong yardstick at the high rates: 31200
                            decodes happily at 0.16-0.23, comfortably inside
                            the 0.30 at which the constellation is still
                            open, and entirely outside 0.15.
                            Measured on the live call fixed-r5, which held an
                            open eye for 115.6 s and delivered nothing: 2% of
                            its 2976 windows were under 0.15 and the peer was
                            never idle, so the sweep was permitted in one
                            window in fifty.  It crawled to step 73 of 112
                            and stopped there -- the enumeration never
                            finished, so the best candidate was never
                            restored and nothing downstream of the sweep,
                            including the confirmation stage, could run at
                            all.  Keep the episode cap; drop the second,
                            absolute eye test. */
                         (s->v90_t3_idle_seen
                          ||
                          ((!v90_t3_sweep_abs_eye()
                            ||
                            s->v90_t3_sym_err_ema < V34_V90_T3_SWEEP_ERR)
                           &&
                           s->v90_t3_sweep_episodes
                               < V34_V90_T3_SWEEP_EPISODES))
                         &&
                         s->parms.j > 0
                         &&
                         s->parms.p > 0
                         &&
                         s->v90_t3_sf_tries < s->parms.j*s->parms.p
                         &&
                         !s->v90_t3_phase_pending)
                {
                    /* Try the next phase.  The decoder applies it at its own
                       frame boundary, so this costs nothing but the window
                       spent measuring.  j is 7, and a short window is used
                       until the phase locks, so the whole space fits inside
                       the peer's idle period several times over.

                       Enumerate from a counter rather than stepping from
                       whatever phase the decoder happens to be on: the
                       decoder advances between the measurement and the
                       boundary where the change lands, so "current + 1" is a
                       walk, not a sweep, and can revisit phases while
                       missing others. */
                    /* Two dimensions, not one.  The superframe index is
                       the obvious one, but the decoder also has a position
                       WITHIN the data frame, and B1's end only pins that if
                       B1 really ends where we think.  Sweeping the seven
                       superframe phases alone found nothing on a live call
                       where the symbols were clean and B1 matched at 98.6%,
                       so sweep the pair: p*j is 112 candidates, and at a
                       0.4 s window that is under a minute -- affordable
                       while the peer's DTE is idle or trickling. */
                    /* The window just measured belongs to whatever phase was
                       applied last time round, so score that one before
                       moving on.  Step 0 is the phase the receiver arrived
                       with, which is the one to beat. */
                    /* Shortlist rather than rank.  The score cannot separate
                       candidates that all read zero bad frames, and on a busy
                       line that is most of the ones worth considering -- so
                       record which ones the cheap test failed to reject, and
                       decide between them later where the test has enough
                       frames to mean something. */
                    if (s->v90_t3_sf_tries == 0)
                    {
                        memset(s->v90_t3_phase_shortlist, 0,
                               sizeof(s->v90_t3_phase_shortlist));
                        s->v90_t3_shortlist_n = 0;
                    }
                    /*endif*/
                    if (shell_pct == 0
                        &&
                        ones_pct < 90
                        &&
                        s->v90_t3_shell_frames >= V34_V90_T3_COARSE_MIN_FRAMES
                        &&
                        s->v90_t3_phase_pos < 256
                        &&
                        !V90_T3_PH_GET(s, s->v90_t3_phase_pos))
                    {
                        V90_T3_PH_SET(s, s->v90_t3_phase_pos);
                        s->v90_t3_shortlist_n++;
                    }
                    /*endif*/
                    if (s->v90_t3_sf_tries == 0)
                    {
                        s->v90_t3_sweep_base = sweep_score;
                        s->v90_t3_sweep_best = sweep_score;
                        s->v90_t3_sweep_best_pos = s->v90_t3_phase_pos;
                    }
                    else if (sweep_score > s->v90_t3_sweep_best)
                    {
                        s->v90_t3_sweep_best = sweep_score;
                        s->v90_t3_sweep_best_pos = s->v90_t3_phase_pos;
                    }
                    /*endif*/
                    /* One data frame further on.  The shift is relative, so
                       the position it reaches is known -- v90_t3_phase_pos --
                       and can be returned to, which an absolute pair of frame
                       labels cannot. */
                    s->v90_t3_phase_delta = 1;
                    s->v90_t3_phase_pending = true;
                    if (s->v90_t3_sf_tries == 0)
                    {
                        s->v90_t3_wrong_bad = 0;
                        s->v90_t3_wrong_frames = 0;
                    }
                    /*endif*/
                    s->v90_t3_sf_tries++;
                    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                             "Rx - V.90 upstream frame phase +1 frame "
                             "(step %d of %d, from offset %d, %d%% ones, "
                             "V.14 %d%% at %dx)\n",
                             s->v90_t3_sf_tries, s->parms.j*s->parms.p,
                             s->v90_t3_phase_pos, ones_pct, v14_pct,
                             v14_ratio/10);
                }
                else if (s->v90_t3_sf_tries > 0
                         &&
                         s->parms.j > 0
                         &&
                         s->parms.p > 0
                         &&
                         s->v90_t3_sf_tries >= s->parms.j*s->parms.p
                         &&
                         !s->v90_t3_phase_pending)
                {
                    /* Every candidate has been measured.  Go to the best one
                       -- which is the phase the sweep started from unless
                       something beat it by a clear margin.

                       Restoring the best rather than stopping wherever the
                       walk ended is what makes an ungated sweep safe.  The
                       sweep used to be allowed only once the marks proved an
                       idle line existed, because a free-running walk wrecks a
                       decode that is already right; but ones only rises above
                       60% when the phase is ALREADY correct, so a call that
                       started on the wrong phase could never run the sweep
                       that would fix it.  Measured live: 0.10 symbol error --
                       an open constellation -- for 230 of 310 seconds, 50%
                       ones throughout, and not one byte of payload.  With the
                       best candidate restored, a receiver that is already
                       right wins its own sweep and nothing moves. */
                    int span = s->parms.j*s->parms.p;
                    int delta = ((s->v90_t3_sweep_best_pos
                                  - s->v90_t3_phase_pos) % span + span) % span;

                    /* Every candidate has had its cheap look.  If more than
                       one survived it, the score cannot choose between them
                       and going to its argmax is picking noise -- confirm
                       them instead. */
                    if (v90_t3_phase_confirm_enabled()
                        &&
                        s->v90_t3_shortlist_n > 0)
                    {
                        int first = -1;

                        for (int k = 0;  k < span;  k++)
                        {
                            int cand = (s->v90_t3_phase_pos + k) % span;

                            if (V90_T3_PH_GET(s, cand))
                            {
                                first = cand;
                                break;
                            }
                            /*endif*/
                        }
                        /*endfor*/
                        if (first >= 0)
                        {
                            s->v90_t3_confirming = true;
                            s->v90_t3_phase_delta = (first - s->v90_t3_phase_pos
                                                     + span) % span;
                            s->v90_t3_phase_pending
                                = (s->v90_t3_phase_delta != 0);
                            s->v90_t3_sf_tries = 0;
                            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                                     "Rx - V.90 upstream phase sweep done: "
                                     "%d of %d candidates survived; "
                                     "confirming from offset %d over %d "
                                     "bits each\n",
                                     s->v90_t3_shortlist_n, span, first,
                                     V34_V90_T3_CONFIRM_BITS);
                            goto v90_t3_window_done;
                        }
                        /*endif*/
                    }
                    /*endif*/
                    if (delta != 0)
                    {
                        s->v90_t3_phase_delta = delta;
                        s->v90_t3_phase_pending = true;
                    }
                    /*endif*/
                    s->v90_t3_sweep_episodes++;
                    /* Leave the counter at its limit unless another sweep is
                       still allowed.  The walk this replaces stopped for good
                       once it had tried everything; letting it restart freely
                       costs more than it can win -- measured on a recorded
                       call whose phase was right from the start, repeated
                       sweeping took the clean fraction from 33% to 21%,
                       because each candidate resets the Viterbi state.  What
                       is worth keeping from this branch is not the repetition
                       but the restore: the old walk simply stopped wherever
                       it happened to end. */
                    if (s->v90_t3_sweep_episodes < V34_V90_T3_SWEEP_EPISODES)
                        s->v90_t3_sf_tries = 0;
                    /*endif*/
                    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                             "Rx - V.90 upstream phase sweep done: best "
                             "offset %d scored %d against %d where it "
                             "started; going back %d frames (episode %d)\n",
                             s->v90_t3_sweep_best_pos,
                             s->v90_t3_sweep_best, s->v90_t3_sweep_base,
                             delta, s->v90_t3_sweep_episodes);
                }
                /*endif*/
            }
            /*endif*/
v90_t3_window_done:
            /* Evidence about the phase the receiver is on NOW.  Only while no
               sweep is in progress: mid-sweep the window belongs to whichever
               candidate is being tried.  Idle windows are excluded, since a
               wrong grouping still decodes a mark to a mark. */
            if (!s->v90_t3_sf_locked  &&  s->v90_t3_sf_tries == 0
                &&
                ones_pct < 90)
            {
                s->v90_t3_wrong_bad += s->v90_t3_shell_bad;
                s->v90_t3_wrong_frames += s->v90_t3_shell_frames;
            }
            else if (s->v90_t3_sf_locked)
            {
                s->v90_t3_wrong_bad = 0;
                s->v90_t3_wrong_frames = 0;
            }
            /*endif*/
            s->v90_t3_ones = 0;
            s->v90_t3_alt_ones = 0;
            s->v90_t3_bit_count = 0;
            s->v90_t3_shell_frames = 0;
            s->v90_t3_shell_bad = 0;
        }
        /*endif*/
    }
    /*endif*/
    return out_bit;
}
/*- End of function --------------------------------------------------------*/

/* The distance at which this receiver's filter is worth snapshotting, and
   the distance at which it has lost the constellation, both as multiples of
   the error the receiver settled at after B1.  Until the baseline is
   established the old fixed value stands in, which is what the receiver used
   to use for the whole call. */
static float v90_t3_fse_keep_err(v34_rx_state_t *s)
{
    float v;

    if (s->v90_t3_err_base_n < V34_V90_T3_ERR_BASE_SYMBOLS)
        return V34_V90_T3_FSE_KEEP_ERR;
    /*endif*/
    v = V34_V90_T3_FSE_KEEP_MULT*s->v90_t3_err_base;
    if (v < V34_V90_T3_FSE_KEEP_MIN)
        v = V34_V90_T3_FSE_KEEP_MIN;
    /*endif*/
    if (v > V34_V90_T3_FSE_KEEP_MAX)
        v = V34_V90_T3_FSE_KEEP_MAX;
    /*endif*/
    return v;
}
/*- End of function --------------------------------------------------------*/

static float v90_t3_fse_lost_err(v34_rx_state_t *s)
{
    float v;

    if (s->v90_t3_err_base_n < V34_V90_T3_ERR_BASE_SYMBOLS)
        return V34_V90_T3_FSE_KEEP_ERR;
    /*endif*/
    v = V34_V90_T3_FSE_LOST_MULT*s->v90_t3_err_base;
    if (v < V34_V90_T3_FSE_LOST_MIN)
        v = V34_V90_T3_FSE_LOST_MIN;
    /*endif*/
    if (v > V34_V90_T3_FSE_LOST_MAX)
        v = V34_V90_T3_FSE_LOST_MAX;
    /*endif*/
    return v;
}
/*- End of function --------------------------------------------------------*/

/* Whether the decisions the DD-LMS is about to adapt on are close enough to
   this receiver's own settled error to be worth adapting on.  See the call
   site. */
static bool v90_t3_dd_gate_ok(v34_rx_state_t *s)
{
    const float mult = V34_V90_T3_DD_GATE_MULT;
    if (mult <= 0.0f
        ||
        s->v90_t3_err_base_n < V34_V90_T3_ERR_BASE_SYMBOLS)
    {
        return true;
    }
    /*endif*/
    return s->v90_t3_sym_err_fast <= mult*s->v90_t3_err_base;
}
/*- End of function --------------------------------------------------------*/

/* Whether what the frame-phase logic is about to read means anything.

   Both its inputs -- the shell-index check that releases a standing lock and
   the sweep score that ranks candidates -- describe how the bits are GROUPED,
   and neither can separate a wrong grouping from symbols the eye no longer
   resolves.  See V34_V90_T3_PHASE_TRUST_MULT. */
static bool v90_t3_phase_evidence_ok(v34_rx_state_t *s)
{
    /* x2's accepted reset-state B1 establishes the Table-12 input epoch
       (V.34 10.1.3.1/11.4.1.1.4). Real 4800-bit/s traffic has K=0, so
       neither the shell bound nor a falling idle-marks fraction provides
       evidence for moving it. Sweeping after an idle preamble corrupts a
       correctly decoded busy line. A new E/B1, not user content, owns the
       next epoch. */
    if (s->x2_mode)
        return false;
    if (s->v90_t3_err_base_n < V34_V90_T3_ERR_BASE_SYMBOLS)
        return s->v90_t3_sym_err_ema < V34_V90_T3_SWEEP_ERR;
    /*endif*/
    return s->v90_t3_sym_err_ema
               <= V34_V90_T3_PHASE_TRUST_MULT*s->v90_t3_err_base;
}
/*- End of function --------------------------------------------------------*/

/* Blind recovery for an eye that has already shut.

   Every adaptive element in this receiver -- the DD-LMS above, the timing
   loop, the decision-directed carrier loop, even the slip search -- is gated
   on the symbols being near the constellation, and each gate is right on its
   own terms: a decision taken on white symbols is noise, and adapting to it
   walks the filter off.  Together they are a trap.  Once the eye shuts there
   is no decision to adapt on, so nothing adapts, so the eye stays shut: on
   artifacts/goal-matrix-115515Z/rate28800-r1 the receiver ran 19 s at 0.10
   from the lattice, met a disturbance, and spent the remaining 95 s at 0.66
   -- the value for symbols bearing no relation to the lattice -- while the
   wire went on carrying the peer's signal.  Neither a rotation nor a gain
   recovers those symbols offline (swept +/-45 degrees and +/-15%: 0.65 stays
   0.62), so it is the filter, not the carrier, and only something that needs
   no decisions can put the filter back.

   That is CMA.  It has a phase ambiguity, which is already this receiver's
   normal condition -- the fourth-power carrier loop takes over exactly when
   the decision-directed one is gated off -- and it is stopped the moment the
   eye reopens, so the steady state is unchanged: the DD-LMS owns the taps
   whenever there are decisions worth owning them with. */

/* The fixed-point FSE runs from its OWN copy of the taps: v90_t3_fse_fx, and
   the wide accumulator behind it, seeded from the float array once at prime
   time and thereafter advanced only by the integer NLMS.  Every OTHER writer
   of the float taps -- the equalizer restore, the blind CMA loop, and a fresh
   B1 acquisition -- therefore has no effect at all in a fixed-point build
   unless it says so here.  Clearing the prime flag makes the next symbol
   re-seed the integer taps (and re-choose the ring's binary point from the
   level now present, which is the right thing after the level has moved).

   This is not a theoretical gap.  On rate19200-r1 the two datapaths track each
   other to 0.01 of the lattice for nine tenths of the call, both meet the same
   disturbance, and float then recovers to 0.10 while fixed stays at 0.76 for
   the remaining 150 s -- because every mechanism that exists to recover was
   writing taps the receiver had stopped reading. */
static void v90_t3_fse_taps_replaced(v34_rx_state_t *s)
{
#if defined(V34_FIXED_POINT)
    s->v90_t3_fx_primed = 0;
#else
    (void) s;
#endif
}
/*- End of function --------------------------------------------------------*/

static void v90_t3_blind_recover(v34_rx_state_t *s,
                                 const complexf_t *y,
                                 int pre,
                                 float frac,
                                 float energy)
{
    float p2;
    float e;
    float mu;

    /* On, but only worth it once the fourth-power carrier term is held:
           on its own it is worth nothing, because it reopens the eye into a
           frequency that estimator has meanwhile walked away.  With the hold
           in, 28800 goes from 23% of the call clean to 55% and its longest
           hold from 19.7 s to 37.1 s, and 19200 from 48% to 69%.

           Beware the first result this produced -- 17% becoming 82% -- which
           was the diverged-receiver artefact V34_V90_T3_DIVERGED_POWER now
       names, measured at a mean symbol power of 1.5e20. */
    /* Only ever against a dispersion constant this call measured for itself,
       and only once the receiver has an operating point to compare with. */
    if (s->v90_t3_cma_r2 <= 0.0f)
        return;
    /*endif*/
    /* And not before there is a filter to start from.  CMA is a gradient
       descent on a surface with local minima: started from the taps a
       collapse walked off it does not come back (measured -- 95 s of trying),
       and started from the last snapshot it reopens the eye in under a
       second.  Arming it earlier than the first snapshot is therefore not
       "sooner", it is "into the case that does not work": with r2 known at
       the handover rather than measured over 32000 settled symbols, the loop
       became able to arm in the first seconds of a call and 19200 fell from
       69% of the call clean to 48%. */
    if (!s->v90_t3_fse_good_valid)
        return;
    /*endif*/
    if (!s->v90_t3_cma_active)
    {
        /* Judge the start on the SLOW estimate.  The fast one oscillates
           across any threshold while the eye is shut -- measured on
           rate28800-r1 it crosses 0.55 every few tens of symbols -- so a run
           counted on it never reaches a length worth acting on, while the
           slow estimate sits steadily at 0.64.  The stop below stays on the
           fast one, where turning round quickly is what matters. */
        if (s->v90_t3_sym_err_ema >= V34_V90_T3_LOST_ERR)
        {
            if (++s->v90_t3_cma_run >= V34_V90_T3_CMA_START_RUN
                &&
                s->v90_t3_cma_episodes < V34_V90_T3_CMA_MAX_EPISODES)
            {
                s->v90_t3_cma_active = true;
                s->v90_t3_cma_episodes++;
                s->v90_t3_cma_run = 0;
                /* Start from the last filter that demonstrably worked, where
                   there is one.  CMA is a gradient descent on a surface with
                   local minima, and the taps it would otherwise start from
                   are the ones the collapse walked off -- measured on
                   rate28800-r1, started from those it never reopened the eye
                   in 95 s, and started from the snapshot it reopened it in
                   under a second.  The snapshot alone is not enough either:
                   restoring it is what the receiver already did, ten times,
                   and the eye shut again within a second each time.  It is
                   the pair -- a sane starting point AND a loop that needs no
                   decisions to leave it -- that recovers the call. */
                if (s->v90_t3_fse_good_valid)
                {
                    memcpy(s->v90_t3_fse, s->v90_t3_fse_good,
                           sizeof(s->v90_t3_fse));
                    v90_t3_fse_taps_replaced(s);
                }
                /*endif*/
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 upstream blind recovery started at %.3f "
                         "from the constellation (r2 %.1f, episode %d)\n",
                         s->v90_t3_sym_err_ema, s->v90_t3_cma_r2,
                         s->v90_t3_cma_episodes);
            }
            /*endif*/
        }
        else
        {
            s->v90_t3_cma_run = 0;
        }
        /*endif*/
        return;
    }
    /*endif*/
    /* Stop as soon as the decisions mean something again -- from here the
       DD-LMS is strictly better, and CMA left running would fight it. */
    if (s->v90_t3_sym_err_ema < V34_V90_T3_TIMING_TRACK_ERR)
    {
        s->v90_t3_cma_active = false;
        s->v90_t3_cma_run = 0;
        /* The snapshot describes the filter that failed.  Let the receiver
           earn a new one from the taps that are working now, rather than
           keeping a "last good" the collapse has already disproved. */
        s->v90_t3_fse_good_valid = false;
        s->v90_t3_fse_good_age = 0;
        s->v90_t3_fse_bad_run = 0;
        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                 "Rx - V.90 upstream blind recovery done: %.3f from the "
                 "constellation\n", s->v90_t3_sym_err_ema);
        return;
    }
    /*endif*/
    /* Bounded.  A blind loop that is not converging is stirring the filter,
       so give up on the episode and put the snapshot back rather than run
       for the rest of the call. */
    if (++s->v90_t3_cma_run > V34_V90_T3_CMA_MAX_SYMBOLS)
    {
        s->v90_t3_cma_active = false;
        s->v90_t3_cma_run = 0;
        if (s->v90_t3_fse_good_valid)
        {
            memcpy(s->v90_t3_fse, s->v90_t3_fse_good,
                   sizeof(s->v90_t3_fse));
            v90_t3_fse_taps_replaced(s);
        }
        /*endif*/
        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                 "Rx - V.90 upstream blind recovery gave up at %.3f from the "
                 "constellation\n", s->v90_t3_sym_err_ema);
        return;
    }
    /*endif*/
    p2 = y->re*y->re + y->im*y->im;
    /* Godard p=2, normalised so that the step is a fixed FRACTION of the
       taps whatever the level.  The error is made dimensionless against r2
       and the gradient divided by the input energy: |y| goes as sqrt(r2),
       |x| as sqrt(energy) and the taps themselves as sqrt(r2/energy), so
       what is left is mu times the relative error.  Written with the bare
       (|y|^2 - r2) instead, the step scales with the constellation and the
       loop diverges to NaN at 28800 while looking stable at 9600 -- which is
       exactly the class of rate-dependent fault this whole investigation is
       about. */
    e = (p2 - s->v90_t3_cma_r2)/(s->v90_t3_cma_r2 + 1e-6f);
    /* A blind loop cannot be allowed to run away: without decisions there is
       nothing to notice that it has. */
    if (!isfinite(e)  ||  !isfinite(p2)  ||  fabsf(e) > 64.0f)
        return;
    /*endif*/
    mu = V34_V90_T3_CMA_MU/energy;
    for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
    {
        complexf_t x = v90_t3_raw_get_frac(
            s, s->v90_t3_next_symbol - pre + tap, frac);
        float g_re;
        float g_im;

        if (s->v90_t3_fse_conjugate)
            x.im = -x.im;
        /*endif*/
        g_re = e*(y->re*x.re + y->im*x.im);
        g_im = e*(y->im*x.re - y->re*x.im);
        s->v90_t3_fse[tap].re -= mu*g_re;
        s->v90_t3_fse[tap].im -= mu*g_im;
    }
    /*endfor*/
    v90_t3_fse_taps_replaced(s);
}
/*- End of function --------------------------------------------------------*/

/*- End of function --------------------------------------------------------*/

/* Hypothesis index whose map_table row is dibit -> (-dibit) & 3, i.e.
   {0, 3, 2, 1}.  V.34 10.1.3.3 advances the transmitted point index by the
   dibit, and training_constellation_4 is ordered so that an increasing index
   rotates CLOCKWISE (225, 135, 45, 315 degrees).  The receiver measures the
   phase increment counter-clockwise, so the recovered differential dibit is
   always the negation of the transmitted one.  This is a property of the
   encoder and the table, not of the channel, so for a differential decode it
   is fixed rather than searched. */

int v34_rx_map_phase4_raw_bits(int dibit, int hypothesis)
{
    static const uint8_t map_table[MP_HYPOTHESIS_COUNT][4] =
    {
        /* Legacy hyp8 ordering first for Phase 3 lock-hint compatibility */
        {0, 1, 2, 3}, /*  0: identity */
        {1, 0, 3, 2}, /*  1: xor 1 */
        {2, 3, 0, 1}, /*  2: xor 2 */
        {3, 2, 1, 0}, /*  3: xor 3 */
        {0, 2, 1, 3}, /*  4: swap */
        {2, 0, 3, 1}, /*  5: swap + xor 1 */
        {1, 3, 0, 2}, /*  6: swap + xor 2 */
        {3, 1, 2, 0}, /*  7: swap + xor 3 */

        /* Additional invertible affine transforms on dibits */
        {0, 3, 2, 1}, /*  8 */
        {1, 2, 3, 0}, /*  9 */
        {2, 1, 0, 3}, /* 10 */
        {3, 0, 1, 2}, /* 11 */
        {0, 1, 3, 2}, /* 12 */
        {1, 0, 2, 3}, /* 13 */
        {2, 3, 1, 0}, /* 14 */
        {3, 2, 0, 1}, /* 15 */
        {0, 2, 3, 1}, /* 16 */
        {1, 3, 2, 0}, /* 17 */
        {2, 0, 1, 3}, /* 18 */
        {3, 1, 0, 2}, /* 19 */
        {0, 3, 1, 2}, /* 20 */
        {1, 2, 0, 3}, /* 21 */
        {2, 1, 3, 0}, /* 22 */
        {3, 0, 2, 1}  /* 23 */
    };

    int hidx;

    hidx = hypothesis % MP_HYPOTHESIS_COUNT;
    if (hidx < 0)
        hidx += MP_HYPOTHESIS_COUNT;
    /*endif*/
    return map_table[hidx][dibit & 0x3];
}
/*- End of function --------------------------------------------------------*/

static void bits32_to_str(uint32_t v, char out[33])
{
    int j;

    for (j = 31;  j >= 0;  j--)
        out[31 - j] = ((v >> j) & 1) ? '1' : '0';
    /*endfor*/
    out[32] = '\0';
}
/*- End of function --------------------------------------------------------*/

static void frame_bits_to_str(const uint8_t bits[], int start, int count, char *out)
{
    int i;

    for (i = 0;  i < count;  i++)
        out[i] = bits[start + i] ? '1' : '0';
    /*endfor*/
    out[count] = '\0';
}
/*- End of function --------------------------------------------------------*/

void v34_rx_bits16_to_str(uint16_t v, char out[17])
{
    int j;

    /* Left-most output bit is first-in-time.
       For our shift-left stream register, that is bit 15 (oldest). */
    for (j = 15;  j >= 0;  j--)
        out[15 - j] = ((v >> j) & 1) ? '1' : '0';
    /*endfor*/
    out[16] = '\0';
}
/*- End of function --------------------------------------------------------*/

uint16_t v34_rx_j_ordered16(uint16_t rx_recent16, int total_bits, int phase)
{
    uint16_t ordered;
    int t;

    ordered = 0;
    for (t = 0;  t < 16;  t++)
    {
        int seq_idx;
        int b;

        /* t runs from oldest -> newest bit in time. */
        seq_idx = (total_bits - 16 + t + phase) & 0xF;
        b = (rx_recent16 >> (15 - t)) & 1;
        ordered |= (uint16_t) (b << seq_idx);
    }
    /*endfor*/
    return ordered;
}
/*- End of function --------------------------------------------------------*/

static int mp_preamble_score(uint32_t bitstream)
{
    int k;
    int score;

    /* Score fixed MP preamble bits currently in bitstream[18:1]:
       17x'1' followed by start '0'. bitstream[0] is the type bit. */
    score = 0;
    for (k = 1;  k <= 18;  k++)
    {
        int got;
        int want;

        got = (bitstream >> k) & 1;
        want = (k == 1)  ?  0  :  1;
        if (got == want)
            score++;
        /*endif*/
    }
    /*endfor*/
    return score;
}
/*- End of function --------------------------------------------------------*/

static bool mp_preamble_has_start_zero(uint32_t bitstream)
{
    /* Preamble layout in bitstream[18:1]:
       17x'1' followed by start '0' at bit 1. */
    return ((bitstream >> 1) & 1) == 0;
}
/*- End of function --------------------------------------------------------*/

static bool mp_preamble_has_sync_ones(uint32_t bitstream)
{
    int k;
    int ones;

    /* Preamble layout in bitstream[18:1]:
       17x'1' in bits [18:2], start '0' in bit [1], type in bit [0]. */
    ones = 0;
    for (k = 2;  k <= 18;  k++)
    {
        ones += ((bitstream >> k) & 1);
    }
    /*endfor*/
    /* Allow up to two sync-bit errors in the preamble gate.
       CRC/fill checks still provide the final validity filter. */
    return (ones >= 15);
}
/*- End of function --------------------------------------------------------*/

static void mp_seed_frame_prefix(uint8_t bits[], uint32_t preamble_stream)
{
    int idx;

    /* frame_idx 0..16 are sync bits, 17 is start, 18 is type. */
    for (idx = 0;  idx <= 18;  idx++)
        bits[idx] = (preamble_stream >> (18 - idx)) & 1;
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

void v34_rx_mp_reset_hypothesis_search(v34_rx_state_t *s)
{
    s->mp_hypothesis = -1;
    s->mp_count = -1;
    /* Do NOT zero mp_hyp_scramble or mp_hyp_bitstream here — the GPC
       descrambler shift registers must track every input baud continuously.
       Zeroing them destroys synchronization and makes all subsequent
       descrambled data garbage even if preamble re-locks.  The registers
       are properly initialized at MP entry from the TRN scramble state
       and updated every baud in the hypothesis scan loop. */
}
/*- End of function --------------------------------------------------------*/

static void phase3_pp_reset(v34_rx_state_t *s)
{
    memset(s->phase3_pp_lag8, 0, sizeof(s->phase3_pp_lag8));
    s->phase3_pp_obs = 0;
    s->phase3_pp_match = 0;
    memset(s->phase3_pp_error, 0, sizeof(s->phase3_pp_error));
    memset(s->phase3_pp_corr, 0, sizeof(s->phase3_pp_corr));
    s->phase3_pp_corr_energy = 0.0f;
    s->phase3_pp_corr_weight = 0.0f;
    s->phase3_pp_rotation.re = 1.0f;
    s->phase3_pp_rotation.im = 0.0f;
    s->phase3_pp_phase = -1;
    s->phase3_pp_phase_score = -1;
    s->phase3_pp_acquire_hits = 0;
    s->phase3_pp_onset = -1;
    s->phase3_pp_skip = 0;
    s->phase3_pp_started = 0;
    s->v90_p3_eye_released = false;
}
/*- End of function --------------------------------------------------------*/

void v34_rx_phase4_trn_hyp_reset(v34_rx_state_t *s)
{
    memset(s->phase4_trn_scramble_tap, 0, sizeof(s->phase4_trn_scramble_tap));
    memset(s->phase4_trn_one_count_tap, 0, sizeof(s->phase4_trn_one_count_tap));
    memset(s->phase4_trn_scramble, 0, sizeof(s->phase4_trn_scramble));
    memset(s->phase4_trn_prev_z, 0, sizeof(s->phase4_trn_prev_z));
    memset(s->phase4_trn_prev_valid, 0, sizeof(s->phase4_trn_prev_valid));
    memset(s->phase4_trn_one_count, 0, sizeof(s->phase4_trn_one_count));
    s->phase4_trn_lock_hyp = -1;
    s->phase4_trn_lock_score = -1;
    s->phase4_trn_lock_tap = -1;
    s->phase4_trn_lock_order = -1;
    s->phase4_trn_lock_domain = -1;
    s->phase4_trn_current_hyp = -1;
    s->phase4_trn_current_score = -1;
    s->phase4_trn_current_tap = -1;
    s->phase4_trn_current_order = -1;
    s->phase4_trn_current_domain = -1;
    s->phase4_trn_recent_scramble = 0;
    s->phase4_trn_recent_window_bits = 0;
    s->phase4_trn_recent_window_ones = 0;
    s->phase4_trn_recent_window_fill = 0;
    s->phase4_trn_recent_score = -1;
    memset(s->phase4_trn_recent_symbol_ones, 0, sizeof(s->phase4_trn_recent_symbol_ones));
    s->phase4_trn_recent_active = 0;
}
/*- End of function --------------------------------------------------------*/

static int phase4_trn_should_freeze_tracking(const v34_rx_state_t *s)
{
    /* Also freeze carrier tracking during MP — the carrier phase was locked
       during TRN and CMA-based QPSK slicer decisions during MP are too noisy
       to drive carrier tracking without corrupting the locked phase. */
    if (v34_rx_stage_is_phase4_frame(s->stage))
        return 1;
    return (s->stage == V34_RX_STAGE_PHASE4_TRN
            && s->phase4_j_seen
            && s->phase4_trn_lock_hyp >= 0
            && s->phase4_trn_lock_score >= PHASE4_TRN_FREEZE_SCORE
            && s->phase4_trn_current_hyp == s->phase4_trn_lock_hyp
            && s->phase4_trn_current_tap == s->phase4_trn_lock_tap
            && s->phase4_trn_current_order == s->phase4_trn_lock_order
            && s->phase4_trn_current_domain == s->phase4_trn_lock_domain
            && s->phase4_trn_current_score >= PHASE4_TRN_FREEZE_SCORE);
}
/*- End of function --------------------------------------------------------*/

static void phase3_trn_hyp_reset(v34_rx_state_t *s)
{
    memset(s->phase3_trn_scramble, 0, sizeof(s->phase3_trn_scramble));
    memset(s->phase3_trn_one_count, 0, sizeof(s->phase3_trn_one_count));
    s->phase3_trn_bits = 0;
    s->phase3_trn_lock_hyp = -1;
    s->phase3_trn_lock_score = -1;
    s->phase3_trn_rescore_bits = 0;
    s->phase3_tracking_armed = false;
}
/*- End of function --------------------------------------------------------*/

static int mp_alternate_scrambler_tap(int tap)
{
    /* V.34 uses the two complementary scrambler taps (x^-5 and x^-18),
       represented here as zero-based indices 4 and 17. */
    return (tap == 17) ? 4 : 17;
}
/*- End of function --------------------------------------------------------*/

const char *v34_rx_phase4_trn_order_name(int order_idx)
{
    return (order_idx == 1) ? "b1,b0" : "b0,b1";
}
/*- End of function --------------------------------------------------------*/

const char *v34_rx_phase4_trn_domain_name(int domain_idx)
{
    return (domain_idx == 1) ? "abs" : "diff";
}
/*- End of function --------------------------------------------------------*/

static void phase4_unpack_ordered_bits(int raw_bits, int order_idx, int *first_bit, int *second_bit)
{
    if (order_idx == 1)
    {
        *first_bit = (raw_bits >> 1) & 1;
        *second_bit = raw_bits & 1;
    }
    else
    {
        *first_bit = raw_bits & 1;
        *second_bit = (raw_bits >> 1) & 1;
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_min4(const int hist[4])
{
    int i;
    int min_v;

    min_v = hist[0];
    for (i = 1;  i < 4;  i++)
    {
        if (hist[i] < min_v)
            min_v = hist[i];
        /*endif*/
    }
    /*endfor*/
    return min_v;
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_max4(const int hist[4])
{
    int i;
    int max_v;

    max_v = hist[0];
    for (i = 1;  i < 4;  i++)
    {
        if (hist[i] > max_v)
            max_v = hist[i];
        /*endif*/
    }
    /*endfor*/
    return max_v;
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_top2_sum(const int hist[4])
{
    int i;
    int top1;
    int top2;

    top1 = -1;
    top2 = -1;
    for (i = 0;  i < 4;  i++)
    {
        int v;

        v = hist[i];
        if (v > top1)
        {
            top2 = top1;
            top1 = v;
        }
        else if (v > top2)
        {
            top2 = v;
        }
        /*endif*/
    }
    /*endfor*/
    return top1 + top2;
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_bottom2_sum(const int hist[4])
{
    int i;
    int low1;
    int low2;

    low1 = 0x7FFFFFFF;
    low2 = 0x7FFFFFFF;
    for (i = 0;  i < 4;  i++)
    {
        int v;

        v = hist[i];
        if (v < low1)
        {
            low2 = low1;
            low1 = v;
        }
        else if (v < low2)
        {
            low2 = v;
        }
        /*endif*/
    }
    /*endfor*/
    return low1 + low2;
}
/*- End of function --------------------------------------------------------*/

static int mp_phase4_diff_hist_collapsed(const int diff_hist[4])
{
    int top2_sum;
    int bot2_sum;

    top2_sum = mp_hist_top2_sum(diff_hist);
    bot2_sum = mp_hist_bottom2_sum(diff_hist);
    return (top2_sum >= 360 && bot2_sum <= 8);
}
/*- End of function --------------------------------------------------------*/

static int mp_phase4_abs_hist_healthy(const int abs_hist[4])
{
    int min_v;
    int max_v;

    min_v = mp_hist_min4(abs_hist);
    max_v = mp_hist_max4(abs_hist);
    return (min_v >= 60 && max_v <= 140);
}
/*- End of function --------------------------------------------------------*/

static void mp_phase4_update_auto_domain(v34_rx_state_t *s, const int diff_hist[4], const int abs_hist[4])
{
    int diff_collapsed;
    int abs_healthy;

    /* Once a Phase 4 hypothesis has locked, its bit stream must remain in the
       same dibit domain for the complete CP/MP frame.  Switching from the
       absolute fallback back to differential dibits while the frame is being
       collected preserves the preamble but corrupts the DFI/DRN/CRC fields.
       Leave the selected domain latched until the hypothesis is explicitly
       reset after a rejected frame. */
    if (s->mp_hypothesis >= 0)
        return;

    diff_collapsed = mp_phase4_diff_hist_collapsed(diff_hist);
    abs_healthy = mp_phase4_abs_hist_healthy(abs_hist);
    if (diff_collapsed && abs_healthy)
    {
        s->mp_phase4_diff_collapse_streak++;
        s->mp_phase4_diff_recover_streak = 0;
    }
    else
    {
        s->mp_phase4_diff_collapse_streak = 0;
        if (!diff_collapsed)
            s->mp_phase4_diff_recover_streak++;
        else
            s->mp_phase4_diff_recover_streak = 0;
        /*endif*/
    }
    /*endif*/

    if (!s->mp_phase4_force_abs_active
        && s->mp_phase4_diff_collapse_streak >= 1)
    {
        s->mp_phase4_force_abs_active = 1;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: auto-domain fallback enabled (diff dibits collapsed); forcing abs decode\n");
    }
    else if (s->mp_phase4_force_abs_active
             && s->mp_phase4_diff_recover_streak >= 2)
    {
        s->mp_phase4_force_abs_active = 0;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: auto-domain fallback cleared (diff dibits recovered); restoring configured domain=%s\n",
                 v34_rx_phase4_trn_domain_name(s->mp_phase4_domain));
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static void phase4_j_detector_reset(v34_rx_state_t *s)
{
    s->phase4_j_bits = 0;
    memset(s->phase4_j_scramble_tap, 0, sizeof(s->phase4_j_scramble_tap));
    memset(s->phase4_j_stream_tap, 0, sizeof(s->phase4_j_stream_tap));
    memset(s->phase4_j_prev_z_tap, 0, sizeof(s->phase4_j_prev_z_tap));
    memset(s->phase4_j_prev_valid_tap, 0, sizeof(s->phase4_j_prev_valid_tap));
    memset(s->phase4_j_win_tap, 0, sizeof(s->phase4_j_win_tap));
}
/*- End of function --------------------------------------------------------*/

static void mp_phase4_apply_retry_mode(v34_rx_state_t *s, int retry_mode)
{
    int use_alt_domain;
    int use_alt_order;
    int use_alt_tap;
    int tap;
    int order;
    int domain;

    use_alt_domain = ((retry_mode & 0x4) != 0);
    use_alt_order = ((retry_mode & 0x1) != 0);
    use_alt_tap = ((retry_mode & 0x2) != 0);
    tap = s->mp_phase4_default_scrambler_tap;
    order = s->mp_phase4_default_bit_order;
    domain = s->mp_phase4_default_domain;
    if (use_alt_tap)
        tap = mp_alternate_scrambler_tap(tap);
    /*endif*/
    if (use_alt_order)
        order ^= 1;
    /*endif*/
    if (use_alt_domain)
        domain ^= 1;
    /*endif*/
    s->scrambler_tap = tap;
    s->mp_phase4_bit_order = order;
    s->mp_phase4_domain = domain;
    s->mp_phase4_alt_tap_active = use_alt_tap;
    s->mp_phase4_alt_order_active = use_alt_order;
    s->mp_phase4_alt_domain_active = use_alt_domain;
}
/*- End of function --------------------------------------------------------*/

void v34_rx_mp_vote_reset(v34_rx_state_t *s);

static void mp_v90_cp_reset_at_carrier_gap(v34_rx_state_t *s)
{
    /* V.90 §8.5.2 initializes the analogue modem's scrambler and
       differential encoder to zero before the first CPt.  The forced V.90
       receiver can be armed while the preceding Phase-3 signal is still
       present, so hypotheses accumulated before the carrier gap do not
       belong to CPt.  Reset only the CP framing/descrambling search at that
       unambiguous boundary; retain the trained equalizer, carrier recovery
       and sample timing. */
    s->mp_phase4_retry_mode = 0;
    mp_phase4_apply_retry_mode(s, 0);
    s->mp_phase4_reject_streak = 0;
    s->mp_phase4_nolock_count = 0;
    s->mp_phase4_force_abs_active = 0;
    s->mp_phase4_diff_collapse_streak = 0;
    s->mp_phase4_diff_recover_streak = 0;
    s->phase4_da_active = 0;
    s->phase4_da_seeded = 0;
    s->phase4_da_expected_ang = 0;
    s->phase4_da_derot = 0;
    s->mp_frame_pos = 0;
    s->mp_frame_target = 0;
    s->mp_early_rejects = 0;
    s->bitstream = 0;
    s->bit_count = 0;
    memset(s->mp_hyp_scramble, 0, sizeof(s->mp_hyp_scramble));
    memset(s->mp_hyp_bitstream, 0, sizeof(s->mp_hyp_bitstream));
    v34_rx_mp_reset_hypothesis_search(s);
    v34_rx_mp_vote_reset(s);
}
/*- End of function --------------------------------------------------------*/

static int mp_phase4_has_pinned_trn_lock(const v34_rx_state_t *s)
{
    return (s->phase4_trn_lock_hyp >= 0
            && s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT
            && s->phase4_trn_lock_score >= PHASE4_TRN_READY_MIN_SCORE);
}
/*- End of function --------------------------------------------------------*/

static void mp_phase4_rotate_retry_mode(v34_rx_state_t *s, const char *reason)
{
    if (mp_phase4_has_pinned_trn_lock(s))
    {
        if (s->mp_phase4_reject_streak < 3)
        {
            s->mp_phase4_nolock_count++;
            /* After 3 no-lock rotations (~1200 bauds), broaden search beyond
               TRN hint hypothesis — the TRN-locked hyp may not match MP. */
            if (s->mp_phase4_nolock_count >= 3)
            {
                s->mp_phase4_reject_streak = 3;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: %s; TRN hint hyp=%d failed %d times, broadening MP search\n",
                         reason, s->phase4_trn_lock_hyp, s->mp_phase4_nolock_count);
            }
            else
            {
                v34_rx_mp_reset_hypothesis_search(s);
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: %s; keeping TRN-locked MP settings (hyp=%d, dom=%s, tap=%d, ord=%s)\n",
                         reason, s->phase4_trn_lock_hyp,
                         v34_rx_phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                         v34_rx_phase4_trn_order_name(s->mp_phase4_bit_order));
                return;
            }
        }
        /*endif*/
    }
    /*endif*/
    s->mp_phase4_retry_mode = (s->mp_phase4_retry_mode + 1) & 0x7;
    mp_phase4_apply_retry_mode(s, s->mp_phase4_retry_mode);
    v34_rx_mp_reset_hypothesis_search(s);
    if (s->mp_phase4_retry_mode == 0)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: %s; restoring MP descrambler defaults (dom=%s, tap=%d, ord=%s)\n",
                 reason, v34_rx_phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 v34_rx_phase4_trn_order_name(s->mp_phase4_bit_order));
    }
    else
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: %s; switching MP descrambler retry mode=%d (dom=%s, tap=%d, ord=%s)\n",
                 reason, s->mp_phase4_retry_mode,
                 v34_rx_phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 v34_rx_phase4_trn_order_name(s->mp_phase4_bit_order));
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static void mp_unlock_after_reject(v34_rx_state_t *s, bool count_tap_reject)
{
    const int tap_switch_rejects = 3;

    if (s->duplex  &&  s->stage == V34_RX_STAGE_PHASE4_MP  &&  s->mp_seen >= 1
        &&  s->mp_last_frame_bits > 0)
    {
        /* A frame has already been accepted with this hypothesis and decode
           mode, so a later CRC failure is damage to that frame (a lost MP'),
           not evidence the lock is wrong.  Dropping it -- and rotating the
           decode mode after three such frames -- is what stopped the E that
           follows from ever being looked for (11.4.1.x.3 lets E replace the
           peer's MP').  Keep the lock and wait for the next frame boundary. */
        s->mp_early_rejects = 0;
        s->mp_frame_pos = 0;
        s->mp_frame_target = 0;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: keeping MP hypothesis=%d after a rejected frame (an earlier MP was accepted)\n",
                 s->mp_hypothesis);
        return;
    }
    /*endif*/
    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
             "Rx - Phase 4: unlock MP hypothesis=%d after rejected frame\n",
             s->mp_hypothesis);
    s->mp_early_rejects = 0;
    v34_rx_mp_reset_hypothesis_search(s);
    if (s->stage == V34_RX_STAGE_V90_CP && s->v90_cp_diff_hypothesis >= 0)
    {
        /* Every axis the retry mode rotates is already fixed for a V.90
           Phase 4 CP: the domain is differential (8.5.2 sends CP through J's
           modulation, which 10.1.3.3 differentially encodes), the scrambler
           is the analogue modem's GPA (tap 4, V.90 9), and the bit order is
           b0,b1.  Rotating off that configuration cannot find a better one --
           it can only spend the CP window decoding with settings known to be
           wrong.  Measured: after three rejects the rotation moved to
           ord=b1,b0 and then tap=17, and no further frame validated.
           A rejected frame here means line errors in the body, and the answer
           to that is to re-acquire the next repetition, not to change how it
           is decoded. */
        s->mp_frame_pos = 0;
        s->mp_frame_target = 0;
        s->mp_phase4_reject_streak = 0;
        return;
    }
    /*endif*/
    if (mp_phase4_has_pinned_trn_lock(s))
    {
        if (count_tap_reject)
            s->mp_phase4_reject_streak++;
        /*endif*/
        if (s->mp_phase4_reject_streak >= tap_switch_rejects)
        {
            s->mp_phase4_retry_mode = (s->mp_phase4_retry_mode + 1) & 0x7;
            mp_phase4_apply_retry_mode(s, s->mp_phase4_retry_mode);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: pinned TRN lock with %d rejects, switching MP retry mode=%d (dom=%s, tap=%d, ord=%s)\n",
                     s->mp_phase4_reject_streak, s->mp_phase4_retry_mode,
                     v34_rx_phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                     v34_rx_phase4_trn_order_name(s->mp_phase4_bit_order));
            s->mp_phase4_reject_streak = 0;
        }
        /*endif*/
        s->mp_frame_pos = 0;
        s->mp_frame_target = 0;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: keeping TRN-locked MP hypothesis/settings after reject (hyp=%d, streak=%d, dom=%s, tap=%d, ord=%s)\n",
                 s->phase4_trn_lock_hyp, s->mp_phase4_reject_streak,
                 v34_rx_phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 v34_rx_phase4_trn_order_name(s->mp_phase4_bit_order));
        return;
    }
    /*endif*/
    if (count_tap_reject
        && ++s->mp_phase4_reject_streak >= tap_switch_rejects)
    {
        s->mp_phase4_retry_mode = (s->mp_phase4_retry_mode + 1) & 0x7;
        mp_phase4_apply_retry_mode(s, s->mp_phase4_retry_mode);
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: after %d rejects, MP retry mode=%d (dom=%s, tap=%d, ord=%s)\n",
                 s->mp_phase4_reject_streak, s->mp_phase4_retry_mode,
                 v34_rx_phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 v34_rx_phase4_trn_order_name(s->mp_phase4_bit_order));
        s->mp_phase4_reject_streak = 0;
    }
    /*endif*/
    if (!count_tap_reject)
        s->mp_phase4_reject_streak = 0;
    /*endif*/
    s->mp_frame_pos = 0;
    s->mp_frame_target = 0;
}
/*- End of function --------------------------------------------------------*/

void v34_rx_mp_vote_reset(v34_rx_state_t *s)
{
    memset(s->mp0_vote_counts, 0, sizeof(s->mp0_vote_counts));
    memset(s->mp0_vote_frames_by_hyp, 0, sizeof(s->mp0_vote_frames_by_hyp));
    memset(s->mp0_vote_same_lock, 0, sizeof(s->mp0_vote_same_lock));
    s->mp0_vote_last_hyp = -1;
    s->mp0_vote_frames = 0;
    s->mp0_vote_hyp = -1;
    memset(s->mp1_vote_counts, 0, sizeof(s->mp1_vote_counts));
    s->mp1_vote_frames = 0;
    s->mp1_vote_hyp = -1;
}
/*- End of function --------------------------------------------------------*/

int v34_rx_phase3_j_pattern_bit(int pat_type, int bit_idx)
{
    /* LSB-first pattern bits, per V.34 Table 18/19 representation used by TX:
       J (4-point)  = 0x8990
       J (16-point) = 0x89B0
       J'           = 0x899F */
    static const uint16_t pats[3] =
    {
        0x8990,
        0x89B0,
        0x899F
    };

    return (pats[pat_type] >> (bit_idx & 15)) & 1;
}
/*- End of function --------------------------------------------------------*/

static uint16_t mp_crc_bits(const uint8_t bits[], int type)
{
    int i;
    int len;
    uint16_t crc;

    crc = 0xFFFF;
    len = (type == 1)  ?  170  :  68;
    for (i = 17;  i < len;  i += 17)
    {
        int j;

        /* Each 17-bit block is: start bit + 16 payload bits.
           CRC is over the 16 payload bits only (exclude start bit). */
        for (j = 0;  j < 16;  j++)
            crc = crc_itu16_bits(bits[i + 1 + j], 1, crc);
        /*endfor*/
    }
    /*endfor*/
    return crc;
}
/*- End of function --------------------------------------------------------*/

static bool mp_crc_ok(const uint8_t bits[], int type, uint16_t *rx_crc_out, uint16_t *residual_out)
{
    int i;
    int crc_start;
    uint16_t crc;
    uint16_t rx_crc;

    crc = mp_crc_bits(bits, type);
    crc_start = (type == 1)  ?  171  :  69;
    rx_crc = 0;
    for (i = 0;  i < 16;  i++)
    {
        rx_crc |= bits[crc_start + i] << i;
        crc = crc_itu16_bits(bits[crc_start + i], 1, crc);
    }
    /*endfor*/
    if (rx_crc_out)
        *rx_crc_out = rx_crc;
    /*endif*/
    if (residual_out)
        *residual_out = crc;
    /*endif*/
    return (crc == 0);
}
/*- End of function --------------------------------------------------------*/

static bool mp_fill_ok(const uint8_t bits[], int type)
{
    if (type == 1)
        return bits[187] == 0;
    return (bits[85] | bits[86] | bits[87]) == 0;
}
/*- End of function --------------------------------------------------------*/

static bool mp_start_bit_ok(int type, int bit_index, int bit_value)
{
    if (bit_index == 17 || bit_index == 34 || bit_index == 51 || bit_index == 68)
        return bit_value == 0;
    if (type == 1
        &&
        (bit_index == 85
         || bit_index == 102
         || bit_index == 119
         || bit_index == 136
         || bit_index == 153
         || bit_index == 170))
    {
        return bit_value == 0;
    }
    /*endif*/
    return true;
}
/*- End of function --------------------------------------------------------*/

static int mp_start_error_count(const uint8_t bits[], int type, int target)
{
    int i;
    int errs;

    errs = 0;
    for (i = 17;  i < target;  i++)
    {
        if (!mp_start_bit_ok(type, i, bits[i]))
            errs++;
        /*endif*/
    }
    /*endfor*/
    return errs;
}
/*- End of function --------------------------------------------------------*/

static bool mp_try_slip_recovery(uint8_t bits[188], int type, int target, int *slip_out)
{
    static const int slips[] = {-2, -1, 1, 2};
    uint8_t trial[188];
    int sidx;

    for (sidx = 0;  sidx < (int) (sizeof(slips)/sizeof(slips[0]));  sidx++)
    {
        int slip;
        int i;
        uint16_t rx_crc;
        uint16_t residual_crc;
        bool crc_ok;
        bool fill_ok;

        slip = slips[sidx];
        memcpy(trial, bits, sizeof(trial));
        /* Start at 17 (the first start bit, right after the 17-bit sync
           run), not 19: a lock that is off by one right at the sync/start17
           boundary -- confirmed live against d-modem/slmodemd, where the
           preamble scorer accepted a 17/18 match with bit 16 (last sync bit)
           actually holding what should be bit 17's value -- was structurally
           unreachable by every recovery path here, since all four started
           their earliest correctable boundary at 19, leaving positions
           17-18 (start bit, type bit) permanently excluded from correction.
           Bits 0-16 are a uniform run of 1s, so there is nothing to gain
           shifting from earlier than 17. */
        for (i = 17;  i < target;  i++)
        {
            int src;

            src = i + slip;
            trial[i] = (src >= 17 && src < target)  ?  bits[src]  :  0;
        }
        /*endfor*/
        crc_ok = mp_crc_ok(trial, type, &rx_crc, &residual_crc);
        fill_ok = mp_fill_ok(trial, type);
        if (crc_ok  &&  fill_ok)
        {
            memcpy(bits, trial, sizeof(trial));
            if (slip_out)
                *slip_out = slip;
            /*endif*/
            return true;
        }
        /*endif*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static bool mp_try_boundary_slip_recovery(uint8_t bits[188], int type, int target, int *boundary_out, int *slip_out)
{
    static const int starts_mp0[] = {34, 51, 68};
    static const int starts_mp1[] = {34, 51, 68, 85, 102, 119, 136, 153, 170};
    static const int slips[] = {-2, -1, 1, 2};
    uint8_t trial[188];
    const int *starts;
    int nstarts;
    int si;

    starts = (type == 1) ? starts_mp1 : starts_mp0;
    nstarts = (type == 1) ? (int) (sizeof(starts_mp1)/sizeof(starts_mp1[0]))
                          : (int) (sizeof(starts_mp0)/sizeof(starts_mp0[0]));

    for (si = 0;  si < nstarts;  si++)
    {
        int boundary;
        int k;

        boundary = starts[si];
        if (boundary < 19 || boundary >= target)
            continue;
        /*endif*/
        for (k = 0;  k < (int) (sizeof(slips)/sizeof(slips[0]));  k++)
        {
            int slip;
            int i;
            uint16_t rx_crc;
            uint16_t residual_crc;
            bool crc_ok;
            bool fill_ok;

            slip = slips[k];
            memcpy(trial, bits, sizeof(trial));
            for (i = boundary;  i < target;  i++)
            {
                int src;

                src = i + slip;
                trial[i] = (src >= boundary && src < target) ? bits[src] : 0;
            }
            /*endfor*/
            crc_ok = mp_crc_ok(trial, type, &rx_crc, &residual_crc);
            fill_ok = mp_fill_ok(trial, type);
            if (crc_ok  &&  fill_ok)
            {
                memcpy(bits, trial, sizeof(trial));
                if (boundary_out)
                    *boundary_out = boundary;
                /*endif*/
                if (slip_out)
                    *slip_out = slip;
                /*endif*/
                return true;
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static void mp_apply_boundary_slip(uint8_t bits[188], int boundary, int target, int slip)
{
    uint8_t src[188];
    int i;

    memcpy(src, bits, sizeof(src));
    for (i = boundary;  i < target;  i++)
    {
        int from;

        from = i + slip;
        bits[i] = (from >= boundary && from < target) ? src[from] : 0;
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static bool mp_try_boundary_double_slip_recovery(uint8_t bits[188], int type, int target, int *b1_out, int *s1_out, int *b2_out, int *s2_out)
{
    static const int starts_mp0[] = {34, 51, 68};
    static const int starts_mp1[] = {34, 51, 68, 85, 102, 119, 136, 153, 170};
    static const int slips[] = {-2, -1, 1, 2};
    const int *starts;
    int nstarts;
    int i;
    int j;
    int k1;
    int k2;

    starts = (type == 1) ? starts_mp1 : starts_mp0;
    nstarts = (type == 1) ? (int) (sizeof(starts_mp1)/sizeof(starts_mp1[0]))
                          : (int) (sizeof(starts_mp0)/sizeof(starts_mp0[0]));

    for (i = 0;  i < nstarts;  i++)
    {
        int b1;

        b1 = starts[i];
        if (b1 < 19 || b1 >= target)
            continue;
        /*endif*/
        for (k1 = 0;  k1 < (int) (sizeof(slips)/sizeof(slips[0]));  k1++)
        {
            uint8_t trial1[188];
            uint16_t rx_crc;
            uint16_t residual_crc;
            bool crc_ok;
            bool fill_ok;

            memcpy(trial1, bits, sizeof(trial1));
            mp_apply_boundary_slip(trial1, b1, target, slips[k1]);
            crc_ok = mp_crc_ok(trial1, type, &rx_crc, &residual_crc);
            fill_ok = mp_fill_ok(trial1, type);
            if (crc_ok  &&  fill_ok)
            {
                memcpy(bits, trial1, sizeof(trial1));
                if (b1_out) *b1_out = b1;
                if (s1_out) *s1_out = slips[k1];
                if (b2_out) *b2_out = -1;
                if (s2_out) *s2_out = 0;
                return true;
            }
            /*endif*/

            for (j = i + 1;  j < nstarts;  j++)
            {
                int b2;

                b2 = starts[j];
                if (b2 < 19 || b2 >= target)
                    continue;
                /*endif*/
                for (k2 = 0;  k2 < (int) (sizeof(slips)/sizeof(slips[0]));  k2++)
                {
                    uint8_t trial2[188];

                    memcpy(trial2, trial1, sizeof(trial2));
                    mp_apply_boundary_slip(trial2, b2, target, slips[k2]);
                    crc_ok = mp_crc_ok(trial2, type, &rx_crc, &residual_crc);
                    fill_ok = mp_fill_ok(trial2, type);
                    if (crc_ok  &&  fill_ok)
                    {
                        memcpy(bits, trial2, sizeof(trial2));
                        if (b1_out) *b1_out = b1;
                        if (s1_out) *s1_out = slips[k1];
                        if (b2_out) *b2_out = b2;
                        if (s2_out) *s2_out = slips[k2];
                        return true;
                    }
                    /*endif*/
                }
                /*endfor*/
            }
            /*endfor*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}

static bool mp_try_boundary_bruteforce_recovery(uint8_t bits[188], int type, int target, int *changes_out)
{
    static const int base_slips[] = {-2, -1, 0, 1, 2};
    int boundaries[16];
    int boundary_count;
    int b;
    int bs;
    int total_states;

    boundary_count = 0;
    boundaries[boundary_count++] = 34;
    boundaries[boundary_count++] = 51;
    boundaries[boundary_count++] = 68;
    if (type == 1)
    {
        boundaries[boundary_count++] = 85;
        boundaries[boundary_count++] = 102;
        boundaries[boundary_count++] = 119;
        boundaries[boundary_count++] = 136;
        boundaries[boundary_count++] = 153;
        boundaries[boundary_count++] = 170;
    }
    /*endif*/
    for (b = 0;  b < boundary_count;  b++)
    {
        if (boundaries[b] >= target)
            break;
        /*endif*/
    }
    /*endfor*/
    boundary_count = b;
    if (boundary_count <= 0)
        return false;
    /*endif*/

    total_states = 1;
    for (b = 0;  b < boundary_count;  b++)
        total_states *= 3;
    /*endfor*/

    for (bs = 0;  bs < (int) (sizeof(base_slips)/sizeof(base_slips[0]));  bs++)
    {
        uint8_t base_trial[188];
        int base_slip;
        int state;

        base_slip = base_slips[bs];
        memcpy(base_trial, bits, sizeof(base_trial));
        if (base_slip != 0)
        {
            int i;

            /* See mp_try_slip_recovery(): start at 17, not 19, so a slip
               right at the sync/start17 boundary is reachable here too. */
            for (i = 17;  i < target;  i++)
            {
                int src;

                src = i + base_slip;
                base_trial[i] = (src >= 17  &&  src < target)  ?  bits[src]  :  0;
            }
            /*endfor*/
        }

        for (state = 0;  state < total_states;  state++)
        {
            uint8_t trial[188];
            int code;
            int i;
            int change_count;
            bool crc_ok;
            bool fill_ok;
            uint16_t rx_crc;
            uint16_t residual_crc;

            memcpy(trial, base_trial, sizeof(trial));
            code = state;
            change_count = (base_slip != 0) ? 1 : 0;
            for (i = 0;  i < boundary_count;  i++)
            {
                int trit;
                int slip;

                trit = code % 3;
                code /= 3;
                slip = trit - 1;   /* 0->-1, 1->0, 2->+1 */
                if (slip == 0)
                    continue;
                /*endif*/
                change_count++;
                mp_apply_boundary_slip(trial, boundaries[i], target, slip);
            }
            /*endfor*/
            if (change_count <= 0  ||  change_count > MP_BOUNDARY_BRUTEFORCE_MAX_CHANGES)
                continue;
            /*endif*/
            crc_ok = mp_crc_ok(trial, type, &rx_crc, &residual_crc);
            fill_ok = mp_fill_ok(trial, type);
            if (crc_ok  &&  fill_ok)
            {
                memcpy(bits, trial, sizeof(trial));
                if (changes_out)
                    *changes_out = change_count;
                /*endif*/
                return true;
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static int mp_data_bit_index(int type, int frame_idx)
{
    /* Return 1-based data-bit index within MP body (excluding inserted start bits).
       For inserted start-bit locations return -1. frame_idx is the absolute MP bit
       index where 0..16 are sync '1's, 17=start, 18=type. */
    int idx;
    int data_count;

    if (frame_idx < 19)
        return -1;
    /*endif*/
    if (mp_start_bit_ok(type, frame_idx, 1))
    {
        /* Non-start-bit location. Count non-start bits from frame_idx 19..frame_idx. */
        data_count = 0;
        for (idx = 19;  idx <= frame_idx;  idx++)
        {
            if (mp_start_bit_ok(type, idx, 1))
                data_count++;
            /*endif*/
        }
        /*endfor*/
        return data_count;
    }
    /*endif*/
    return -1;
}
/*- End of function --------------------------------------------------------*/

static void mp_pack_for_parser(uint8_t out[25], const uint8_t bits[], int type)
{
    int i;
    int limit;
    uint8_t *t;
    bitstream_state_t bs;

    memset(out, 0, 25);
    bitstream_init(&bs, true);
    t = out;
    limit = (type == 1)  ?  188  :  88;
    for (i = 18;  i < limit;  i++)
        bitstream_put(&bs, &t, bits[i], 1);
    /*endfor*/
    bitstream_flush(&bs, &t);
}
/*- End of function --------------------------------------------------------*/

static void log_mp_frame_diag(v34_rx_state_t *s, const uint8_t bits[], int type, bool crc_ok, uint16_t rx_crc, uint16_t residual_crc, bool fill_ok)
{
    if (type == 0)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - MP0 diag: sync[0..16]=all1 start17=%d type18=%d reserved19=%d "
                 "start34=%d start51=%d start68=%d crc_rx=0x%04X crc_res=0x%04X "
                 "fill85..87=%d%d%d crc_ok=%d fill_ok=%d\n",
                 bits[17], bits[18], bits[19],
                 bits[34], bits[51], bits[68],
                 rx_crc, residual_crc,
                 bits[85], bits[86], bits[87],
                 crc_ok, fill_ok);
    }
    else
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - MP1 diag: sync[0..16]=all1 start17=%d type18=%d reserved19=%d "
                 "starts34/51/68/85/102/119/136/153/170=%d%d%d%d%d%d%d%d%d "
                 "crc_rx=0x%04X crc_res=0x%04X fill187=%d crc_ok=%d fill_ok=%d\n",
                 bits[17], bits[18], bits[19],
                 bits[34], bits[51], bits[68], bits[85], bits[102],
                 bits[119], bits[136], bits[153], bits[170],
                 rx_crc, residual_crc, bits[187],
                 crc_ok, fill_ok);
    }
    /*endif*/
}

/*- End of function --------------------------------------------------------*/

static void log_mp_lock_seed(v34_rx_state_t *s,
                             int hyp,
                             int type_bit,
                             int score,
                             int bit_pos,
                             uint32_t preamble_stream,
                             int pending_valid,
                             int pending_bit)
{
    char tail[33];
    char seed_bits[25];

    if (score < (MP_PREAMBLE_SCORE_MIN - 1))
        return;
    /*endif*/

    bits32_to_str(preamble_stream, tail);
    frame_bits_to_str(s->mp_frame_bits, 0, 24, seed_bits);
    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
             "Rx - Phase 4: MP lock seed hyp=%d type=%d score=%d/18 bit%d pending=%s%d "
             "frame_pos=%d target=%d preamble=0b%s seeded[0..23]=%s\n",
             hyp, type_bit, score, bit_pos,
             pending_valid ? "" : "none/",
             pending_valid ? pending_bit : 0,
             s->mp_frame_pos, s->mp_frame_target,
             tail, seed_bits);
}
/*- End of function --------------------------------------------------------*/

static int mp_highest_enabled_rate(int maximum, int mask)
{
    int rate;

    if (maximum > 14)
        maximum = 14;
    for (rate = maximum; rate >= 1; rate--)
    {
        if (mask & (1 << (rate - 1)))
            return rate;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* V.34 11.4: measure the receive channel over the Phase-4 TRN segment.

   TRN is a constant-modulus 4-point constellation (10.1.3.8) delivered
   through the same equalizer, timing loop and carrier loop the data mode
   will use, so the spread of its symbols is the receiver's own honest read
   on what constellation density the line can carry.  Nothing earlier in the
   call can substitute.  The L1/L2 probe projection in v34tx.c measures 21
   sparse tones against a single empty DFT bin: that ratio carries the DFT's
   coherent processing gain, it never sees a signal-proportional impairment
   the far transmitter only produces on a wideband signal, and measured
   against a live line it read 37.9 dB where the data mode that followed ran
   at 17.6 dB.

   The channel leaves an unknown rotation on the constellation, so recover it
   first: for points at 45 + 90k degrees, the fourth power of the unit-modulus
   symbol has a mean argument of 180 + 4*theta, which fixes theta modulo 90 --
   and modulo 90 is all that is needed, because the measurement is symmetric
   in the four points.  Rotate the tail onto (+/-a, +/-a) and the per-axis mean
   and variance give the ratio directly. */
int v34_phase4_trn_measured_rate_n(v34_state_t *st, float *snr_db)
{
    v34_rx_state_t *s = &st->rx;
    double sig_sum;
    double noise_sum;
    double snr;
    int n;
    int blocks;
    int b;
    int rate_n;

    if (snr_db)
        *snr_db = 0.0f;
    /*endif*/
    n = s->phase4_trn_snr_fill;
    if (n < PHASE4_TRN_SNR_BLOCK)
        return 0;
    /*endif*/
    /* The residual carrier this receiver cannot remove before data mode is
       about 0.1 degrees per symbol (docs/v34_data_mode_rates.md), which is
       54 degrees across the whole ring and would read as noise.  Estimate
       the rotation per block instead, where it is a few degrees. */
    blocks = n/PHASE4_TRN_SNR_BLOCK;
    sig_sum = 0.0;
    noise_sum = 0.0;
    for (b = 0;  b < blocks;  b++)
    {
        double s4_re;
        double s4_im;
        double alpha;
        double ca;
        double sa;
        double ax;
        double ay;
        double axx;
        double ayy;
        int base;
        int i;

        base = b*PHASE4_TRN_SNR_BLOCK;
        s4_re = 0.0;
        s4_im = 0.0;
        for (i = 0;  i < PHASE4_TRN_SNR_BLOCK;  i++)
        {
            double re = s->phase4_trn_snr_ring[base + i].re;
            double im = s->phase4_trn_snr_ring[base + i].im;
            double m2 = re*re + im*im;
            double u_re;
            double u_im;

            if (m2 <= 0.0)
                continue;
            /*endif*/
            /* z^4, normalised to unit modulus so a loud symbol does not
               outvote a quiet one when the rotation is what is wanted. */
            u_re = (re*re - im*im)/m2;
            u_im = (2.0*re*im)/m2;
            s4_re += u_re*u_re - u_im*u_im;
            s4_im += 2.0*u_re*u_im;
        }
        /*endfor*/
        if (s4_re == 0.0  &&  s4_im == 0.0)
            continue;
        /*endif*/
        /* atan2 gives 180 + 4*theta; the constellation sits at alpha + 90k,
           and rotating by 45 - alpha puts it on (+/-a, +/-a). */
        alpha = atan2(s4_im, s4_re)/4.0;
        ca = cos(0.78539816339744831 - alpha);
        sa = sin(0.78539816339744831 - alpha);
        ax = ay = axx = ayy = 0.0;
        for (i = 0;  i < PHASE4_TRN_SNR_BLOCK;  i++)
        {
            double re = s->phase4_trn_snr_ring[base + i].re;
            double im = s->phase4_trn_snr_ring[base + i].im;
            double x = re*ca - im*sa;
            double y = re*sa + im*ca;

            ax += fabs(x);
            ay += fabs(y);
            axx += x*x;
            ayy += y*y;
        }
        /*endfor*/
        ax /= PHASE4_TRN_SNR_BLOCK;
        ay /= PHASE4_TRN_SNR_BLOCK;
        axx /= PHASE4_TRN_SNR_BLOCK;
        ayy /= PHASE4_TRN_SNR_BLOCK;
        sig_sum += ax*ax + ay*ay;
        noise_sum += (axx - ax*ax) + (ayy - ay*ay);
    }
    /*endfor*/
    if (sig_sum <= 0.0  ||  noise_sum <= 0.0)
        return 0;
    /*endif*/
    snr = 10.0*log10(sig_sum/noise_sum);
    V34_RX_LOG(&st->logging, SPAN_LOG_FLOW,
             "Rx - Phase 4 TRN SNR: %.1f dB over %d symbols (%d blocks), after_j=%d\n",
             snr, blocks*PHASE4_TRN_SNR_BLOCK, blocks, s->phase4_trn_after_j);
    if (snr_db)
        *snr_db = (float) snr;
    /*endif*/
    /* Table 16 rate index, through the same calibration the data mode's own
       report uses (bits/symbol ~ (SNR + 13)/6, measured on the live channel,
       docs/v34_data_mode_rates.md).  The old offset/slope pair was fitted
       while this measurement read the receiver's own Phase-4 convergence
       (a flat 8-11 dB whatever the line); with decision-directed TRN it
       tracks injected noise (19.9/23.3/27.3/28.6 dB at 20/24/30/40 dB). */
    rate_n = (int) floor((snr + 13.0)/6.0
                         *baud_rate_parameters[s->baud_rate].baud_rate/2400.0);
    if (rate_n < 1)
        rate_n = 1;
    /*endif*/
    if (rate_n > 14)
        rate_n = 14;
    /*endif*/
    s->phase4_trn_snr_db = (float) snr;
    s->phase4_trn_snr_rate_n = rate_n;
    return rate_n;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_negotiate_mp_rates(const v34_mp_rate_offer_t *local,
                                         const v34_mp_rate_offer_t *remote,
                                         int *rate_a_to_c,
                                         int *rate_c_to_a)
{
    int common_mask;
    int a_to_c;
    int c_to_a;

    if (!local || !remote || !rate_a_to_c || !rate_c_to_a)
        return -1;
    /* V.34 11.4.1.1.4/.2.4: each direction is the highest rate enabled
       in both masks and no greater than either MP maximum. */
    common_mask = (local->signalling_rate_mask
                 & remote->signalling_rate_mask) & 0x3FFF;
    a_to_c = local->max_rate_a_to_c;
    if (a_to_c > remote->max_rate_a_to_c)
        a_to_c = remote->max_rate_a_to_c;
    c_to_a = local->max_rate_c_to_a;
    if (c_to_a > remote->max_rate_c_to_a)
        c_to_a = remote->max_rate_c_to_a;
    a_to_c = mp_highest_enabled_rate(a_to_c, common_mask);
    c_to_a = mp_highest_enabled_rate(c_to_a, common_mask);
    if (a_to_c <= 0 || c_to_a <= 0)
        return -1;

    /* Table 20 bit 50 permits asymmetric data rates only when both modems
       set it.  Otherwise both channels use the lower selected rate. */
    if (!(local->asymmetric_rates_allowed
          && remote->asymmetric_rates_allowed))
    {
        int symmetric = (a_to_c < c_to_a) ? a_to_c : c_to_a;
        a_to_c = symmetric;
        c_to_a = symmetric;
    }
    *rate_a_to_c = a_to_c;
    *rate_c_to_a = c_to_a;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static bool mp_negotiate_rates(v34_state_t *s, const mp_t *remote)
{
    v34_mp_rate_offer_t local_offer;
    v34_mp_rate_offer_t remote_offer;

    local_offer.max_rate_a_to_c = s->tx.mp.bit_rate_a_to_c;
    local_offer.max_rate_c_to_a = s->tx.mp.bit_rate_c_to_a;
    local_offer.signalling_rate_mask = s->tx.mp.signalling_rate_mask;
    local_offer.asymmetric_rates_allowed = s->tx.mp.asymmetric_rates_allowed;
    remote_offer.max_rate_a_to_c = remote->bit_rate_a_to_c;
    remote_offer.max_rate_c_to_a = remote->bit_rate_c_to_a;
    remote_offer.signalling_rate_mask = remote->signalling_rate_mask;
    remote_offer.asymmetric_rates_allowed = remote->asymmetric_rates_allowed;
    if (v34_negotiate_mp_rates(&local_offer, &remote_offer,
                               &s->tx.negotiated_rate_a_to_c,
                               &s->tx.negotiated_rate_c_to_a))
        return false;
    s->tx.negotiated_rates_valid = true;
    return true;
}
/*- End of function --------------------------------------------------------*/

static bool mp_apply_parameters(v34_state_t *s, const mp_t *remote)
{
    int rx_rate_n;

    /* 11.7.1.1: a cleardown is an MP requesting zero for BOTH directions,
       exchanged inside a rate renegotiation.  There is nothing to negotiate. */
    if (remote->bit_rate_a_to_c == 0  &&  remote->bit_rate_c_to_a == 0
        &&  s->tx.reneg_active)
    {
        s->rx.cleardown_requested = true;
        return true;
    }
    /*endif*/
    /* The initiator's own MP asks for zero, so the responder's ordinary MP has
       no rates to be intersected with; it only has to be received. */
    if (s->tx.cleardown  &&  s->tx.reneg_active)
        return true;
    /*endif*/

    /* V.34 10.1.3.9/Table 20: a modem's MP encoder fields select the
       remote-end transmitter.  Keep TX and RX choices directional. */
    if (remote->type == 1)
        memcpy(s->tx.precoder_coeffs, remote->precoder_coeffs,
               sizeof(s->tx.precoder_coeffs));
    /* MP0 leaves existing coefficients unaffected. */
    if (set_tx_trellis_mode(s, remote->trellis_size)
        || set_rx_trellis_mode(s, s->tx.mp.trellis_size))
        return false;
    s->tx.use_non_linear_encoder = remote->use_non_linear_encoder;
    s->rx.use_non_linear_encoder = s->tx.mp.use_non_linear_encoder;
    if (!mp_negotiate_rates(s, remote))
        return false;

    /* The call modem receives answer-to-call; a V.90 call that fell back to
       V.34 is the call modem with calling_party clear (V.90 9.2.1.1.8). */
    rx_rate_n = (s->rx.calling_party  ||  s->rx.v90_v34_fallback)
              ? s->tx.negotiated_rate_a_to_c
              : s->tx.negotiated_rate_c_to_a;
    s->rx.bit_rate = (rx_rate_n - 1)*2;
    s->bit_rate = rx_rate_n*2400;
    v34_set_working_parameters(&s->rx.parms, s->rx.baud_rate, s->rx.bit_rate,
                               s->tx.mp.expanded_shaping);
    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
             "Rx - Phase 4 negotiated: a2c=%d bps c2a=%d bps; "
             "RX rate=%d bps trellis=%d nonlinear=%d expanded=%d\n",
             s->tx.negotiated_rate_a_to_c*2400,
             s->tx.negotiated_rate_c_to_a*2400,
             rx_rate_n*2400, s->tx.mp.trellis_size,
             s->tx.mp.use_non_linear_encoder,
             s->tx.mp.expanded_shaping);
    return true;
}
/*- End of function --------------------------------------------------------*/

/* V.34 12.4.1.3 and 12.4.2.4 settle the primary channel data signalling rate,
   and they are the same sentence read from the two ends:

     "The source modem's transmit rate shall be the maximum rate enabled that
      is less than or equal to the data signalling rates specified in both
      modems' MPh sequences."
     "The recipient modem's receive rate shall be the maximum rate enabled
      that is less than or equal to the data signalling rates specified in
      both modems' MPh sequences."

   Half-duplex has one primary channel, so the two clauses necessarily produce
   the same number and each end configures its own half of it.  "Enabled" is
   Table 23 bits 35:49 -- a rate the far end did not offer is not enabled at
   the far end -- so the mask is the intersection.

   Unlike the duplex 11.4 negotiation there is no direction to keep straight
   and no acknowledge bit: MPh has neither, which is why 12.4.1.3 turns on
   having received "at least one MPh sequence" rather than on an MP'. */
static bool mph_apply_parameters(v34_state_t *s, const mph_t *remote)
{
    int mask;
    int ceiling;
    int rate_n;
    int baud;
    bool source;

    source = (s->tx.half_duplex_source == V34_HALF_DUPLEX_SOURCE);
    baud = source ? s->tx.baud_rate : s->rx.baud_rate;
    mask = s->tx.mph.signalling_rate_mask & remote->signalling_rate_mask;
    ceiling = (s->tx.mph.max_data_rate < remote->max_data_rate)
            ? s->tx.mph.max_data_rate
            : remote->max_data_rate;
    if (ceiling > 14)
        ceiling = 14;
    /*endif*/
    for (rate_n = ceiling;  rate_n >= 1;  rate_n--)
    {
        if (mask & (1 << (rate_n - 1)))
            break;
        /*endif*/
    }
    /*endfor*/
    if (rate_n < 1)
    {
        V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
                 "Rx - MPh 12.4 rate negotiation found nothing in common "
                 "(local max=%d mask=0x%04X, remote max=%d mask=0x%04X)\n",
                 s->tx.mph.max_data_rate, s->tx.mph.signalling_rate_mask & 0x7FFF,
                 remote->max_data_rate, remote->signalling_rate_mask & 0x7FFF);
        return false;
    }
    /*endif*/
    /* Table 23: bit 27 selects the REMOTE transmitter. Bit 50 permits
       different rates only with mutual consent; otherwise use the lower. */
    bool asym = s->tx.mph.asymmetric_rates_allowed && remote->asymmetric_rates_allowed;
    s->tx.hdx_control_tx_rate = remote->control_channel_2400 ? 2400 : 1200;
    s->rx.hdx_control_rx_rate = s->tx.mph.control_channel_2400 ? 2400 : 1200;
    if (!asym && s->tx.hdx_control_tx_rate != s->rx.hdx_control_rx_rate)
        s->tx.hdx_control_tx_rate = s->rx.hdx_control_rx_rate = 1200;
    s->tx.hdx_negotiated_rate_n = rate_n;
    s->bit_rate = rate_n*2400;
    /* The bit rate CODE is the index into Table 16's mapping for the symbol
       rate; 2*(N - 1) is the code for N*2400 bit/s.  Only the end that owns
       the primary channel direction has working parameters to set: the source
       transmits it and the recipient receives it. */
    if (source)
    {
        s->tx.bit_rate = (rate_n - 1)*2;
        v34_set_working_parameters(&s->tx.parms, s->tx.baud_rate, s->tx.bit_rate,
                                   remote->expanded_shaping);
        s->tx.use_non_linear_encoder = remote->use_non_linear_encoder;
    }
    else
    {
        s->rx.bit_rate = (rate_n - 1)*2;
        v34_set_working_parameters(&s->rx.parms, s->rx.baud_rate, s->rx.bit_rate,
                                   s->tx.mph.expanded_shaping);
    }
    /*endif*/
    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
             "Rx - MPh 12.4.%d: primary channel %s rate %d bit/s "
             "(local max=%d remote max=%d, common mask=0x%04X, symbol rate index %d)\n",
             source ? 1 : 2, source ? "transmit" : "receive", rate_n*2400,
             s->tx.mph.max_data_rate, remote->max_data_rate, mask & 0x7FFF,
             baud);
    return true;
}
/*- End of function --------------------------------------------------------*/

static bool mp_semantic_ok_phase4(v34_rx_state_t *s, const mp_t *mp, int type, const uint8_t bits[])
{
    int bit_idx;

    if (mp->type != type)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (type mismatch frame=%d parsed=%d)\n",
                 type, type, mp->type);
        return false;
    }
    /*endif*/
    if (bits[19] != 0)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d reserved19=%d (expected 0), tolerating\n",
                 type, bits[19]);
    }
    /*endif*/
    if (mp->bit_rate_a_to_c == 0  &&  mp->bit_rate_c_to_a == 0
        &&  ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)))->tx.reneg_active)
    {
        /* 11.7.1.1 cleardown request: legal, and handled by
           mp_apply_parameters() rather than by rate selection. */
    }
    else if (mp->bit_rate_a_to_c < 1  ||  mp->bit_rate_a_to_c > 14
        ||  mp->bit_rate_c_to_a < 1  ||  mp->bit_rate_c_to_a > 14)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (rate fields out of range a_to_c=%d c_to_a=%d)\n",
                 type, mp->bit_rate_a_to_c, mp->bit_rate_c_to_a);
        return false;
    }
    /*endif*/
    if (mp->trellis_size < V34_TRELLIS_16  ||  mp->trellis_size > V34_TRELLIS_64)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (invalid trellis code=%d)\n",
                 type, mp->trellis_size);
        return false;
    }
    /*endif*/
    if ((mp->signalling_rate_mask & 0x3FFF) == 0)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (empty signalling_rate_mask=0x%04X)\n",
                 type, mp->signalling_rate_mask & 0x7FFF);
        return false;
    }
    /*endif*/
    if (mp->signalling_rate_mask & 0x4000)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d reserved rate-mask bit set (0x%04X), tolerating for analog interop\n",
                 type, mp->signalling_rate_mask & 0x7FFF);
    }
    /*endif*/
    bit_idx = mp->bit_rate_a_to_c - 1;
    if (!(mp->signalling_rate_mask & (1 << bit_idx)))
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d a_to_c rate %d missing from mask 0x%04X, tolerating for analog interop\n",
                 type, mp->bit_rate_a_to_c, mp->signalling_rate_mask & 0x7FFF);
    }
    /*endif*/
    bit_idx = mp->bit_rate_c_to_a - 1;
    if (!(mp->signalling_rate_mask & (1 << bit_idx)))
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d c_to_a rate %d missing from mask 0x%04X, tolerating for analog interop\n",
                 type, mp->bit_rate_c_to_a, mp->signalling_rate_mask & 0x7FFF);
    }
    /*endif*/
    s->last_rx_mp = *mp;
    s->last_rx_mp_valid = true;
    return true;
}
/*- End of function --------------------------------------------------------*/

void v34_rx_pack_output_bitstream(v34_rx_state_t *s)
{
    uint8_t *t;
    const uint8_t *u;
    int i;
    int n;
    int bit;
    int bb;
    int kk;

    V34_RX_LOG(s->logging,
             SPAN_LOG_FLOW,
             "Rx - Packed %p %8X - %X %X %X %X - %2X %2X %2X %2X %2X %2X %2X %2X\n",
             s,
             s->r0,
             s->ibits[0],
             s->ibits[1],
             s->ibits[2],
             s->ibits[3],
             s->qbits[0],
             s->qbits[1],
             s->qbits[2],
             s->qbits[3],
             s->qbits[4],
             s->qbits[5],
             s->qbits[6],
             s->qbits[7]);
    bitstream_init(&s->bs, true);
    t = s->rxbuf;
    bb = s->parms.b;
    kk = s->parms.k;
    /* If there are S bits, we switch between high mapping frames and low mapping frames based
       on the SWP pattern. We derive SWP algorithmically. */
    s->s_bit_cnt += s->parms.r;
    if (s->s_bit_cnt >= s->parms.p)
    {
        /* This is a high mapping frame */
        s->s_bit_cnt -= s->parms.p;
    }
    else
    {
        /* 8.2/Table 8 and 9.3.2: K=0 still has low mapping frames. */
        bb--;
        if (kk > 0)
            kk--;
    }
    /*endif*/
    if (s->parms.k)
    {
        /* A shell index that does not fit in kk bits cannot have come from a
           correctly grouped frame: 9.6.3.3 builds r0 from the ring indices of
           eight 2D symbols that belong together, and the transmitter's own
           construction bounds it.  bitstream_put() truncates silently, so
           without this the evidence is thrown away. */
        s->v90_t3_shell_frames++;
        if (kk < 32  &&  s->r0 >= (int32_t) (1u << kk))
            s->v90_t3_shell_bad++;
        /*endif*/
        /* The same check is the only honest read on a plain V.34 data mode,
           where none of the V.90 upstream lock machinery runs: it owes nothing
           to the content, so it separates "the peer is sending nothing we
           recognise" from "we are grouping its symbols wrongly".  A correctly
           grouped stream reads 0%. */
        if (!s->v90_mode  &&  (s->v90_t3_shell_frames % 512) == 0)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - DATA: shell index over k=%d bits in %d of %d frames (%d%%)\n",
                     kk, s->v90_t3_shell_bad, s->v90_t3_shell_frames,
                     100*s->v90_t3_shell_bad/s->v90_t3_shell_frames);
        }
        /*endif*/
        /* k is always < 32, so we always put the entire k bits into a single word */
        bitstream_put(&s->bs, &t, s->r0, kk);
        /* We can rely on this calculation always producing a value for chunk with no
           fractional part? */
        for (i = 0;  i < 4;  i++)
        {
            /* Some I bits */
            bitstream_put(&s->bs, &t, s->ibits[i], 3);
            if (s->parms.q)
            {
                /* Some Q bits */
                bitstream_put(&s->bs, &t, s->qbits[2*i], s->parms.q);
                bitstream_put(&s->bs, &t, s->qbits[2*i + 1], s->parms.q);
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        /* If K is zero (i.e. b = 8, 9, 11, or 12), things need slightly special treatment */
        /* Pack 4 'i' fields */
        /* Need to treat 8, 9, 11, and 12 individually */
        n = bb - 8;
        for (i = 0;  i < n;  i++)
            bitstream_put(&s->bs, &t, s->ibits[i], 3);
        /*endfor*/
        for (  ;  i < 4;  i++)
            bitstream_put(&s->bs, &t, s->ibits[i], 2);
        /*endfor*/
    }
    /*endif*/
    bitstream_flush(&s->bs, &t);
#if 0
    printf("Block ");
    for (i = 0;  i < (s->b + 7)/8;  i++)
        printf("%02X ", s->rxbuf[i]);
    /*endfor*/
    printf("\n");
#endif

    /* Bit order of the data-frame unpack.  This is invisible to everything
       we can test ourselves: the transmitter packs with the same constant, so
       a loopback cancels it, and an idle DTE sends all ones, which is
       invariant under a reversal.  Only a foreign peer sending real
       characters can show it -- and against slmodemd the idle stream decodes
       at 100% ones while the payload is indistinguishable from noise, with no
       periodicity at any frame lag. */
    bitstream_init(&s->bs, true);
    u = s->rxbuf;
    /* The first of the I bits might be auxiliary data */
    i = 0;
    s->aux_bit_cnt += s->parms.w;
    if (s->aux_bit_cnt >= s->parms.p)
    {
        s->aux_bit_cnt -= s->parms.p;
        for (  ;  i < kk;  i++)
        {
            bit = bitstream_get(&s->bs, &u, 1);
            int out_bit = v90_t3_probe_descramble(s, bit);
            /* Advance the scrambler through B1, but clamp its delayed
               output until DATA (V.34 11.4.1.1.5). */
            if (!s->x2_mode || !s->v90_t3_suppress_output)
                s->put_bit(s->put_bit_user_data, out_bit);
        }
        /*endfor*/
        /* Auxiliary data bits are not scrambled (V.34/7) */
        bit = bitstream_get(&s->bs, &u, 1);
        if (s->put_aux_bit)
            s->put_aux_bit(s->put_bit_user_data, bit);
        /*endif*/
        i++;
    }
    for (  ;  i < bb;  i++)
    {
        bit = bitstream_get(&s->bs, &u, 1);
        int out_bit = v90_t3_probe_descramble(s, bit);
        if (!s->x2_mode || !s->v90_t3_suppress_output)
            s->put_bit(s->put_bit_user_data, out_bit);
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void shell_unmap(v34_rx_state_t *s)
{
    int n21;
    int n22;
    int n23;
    int n24;
    int n41;
    int n42;
    int32_t n8;
    int k;
    int w41;
    int w42;
    int w2;
    int w8;
    const uint32_t *g2;
    const uint32_t *g4;
    const uint32_t *z8;

    g2 = g2s[s->parms.m];
    g4 = g4s[s->parms.m];
    z8 = z8s[s->parms.m];

    /* TODO: This code comes directly from the equations in V.34. Can it be improved? */
    n21 = (s->mjk[6] < s->parms.m - s->mjk[7])  ?  s->mjk[6]  :  (s->parms.m - 1 - s->mjk[7]);
    n22 = (s->mjk[4] < s->parms.m - s->mjk[5])  ?  s->mjk[4]  :  (s->parms.m - 1 - s->mjk[5]);
    n23 = (s->mjk[2] < s->parms.m - s->mjk[3])  ?  s->mjk[2]  :  (s->parms.m - 1 - s->mjk[3]);
    n24 = (s->mjk[0] < s->parms.m - s->mjk[1])  ?  s->mjk[0]  :  (s->parms.m - 1 - s->mjk[1]);

    w2 = s->mjk[4] + s->mjk[5];
    w41 = w2 + s->mjk[6] + s->mjk[7];
    n41 = 0;
    for (k = 0;  k < w2;  k++)
        n41 += g2[k]*g2[w41 - k];
    /*endfor*/
    n41 += n21*g2[w2];
    n41 += n22;

    w2 = s->mjk[0] + s->mjk[1];
    w42 = w2 + s->mjk[2] + s->mjk[3];
    n42 = 0;
    for (k = 0;  k < w2;  k++)
        n42 += g2[k]*g2[w42 - k];
    /*endfor*/
    n42 += n23*g2[w2];
    n42 += n24;

    w8 = w41 + w42;
    n8 = 0;
    for (k = 0;  k < w42;  k++)
        n8 += g4[k]*g4[w8 - k];
    /*endfor*/
    n8 += n41*g4[w42];
    n8 += n42;

    s->r0 = z8[w8] + n8;
}
/*- End of function --------------------------------------------------------*/

static int get_inverse_constellation_point(complexi16_t *point)
{
    int x;
    int y;

    x = point->re + 1;
    x = (x + 43)/4;
    if (x < 0)
        x = 0;
    else if (x > 22)
        x = 22;
    /*endif*/
    y = point->im + 1;
    y = (y + 43)/4;
    if (y < 0)
        y = 0;
    else if (y > 22)
        y = 22;
    /*endif*/
    return v34_inverse_superconstellation[y][x];
}
/*- End of function --------------------------------------------------------*/

static complexi16_t rotate90_counterclockwise(complexi16_t *x, int quads)
{
    complexi16_t y;

    /* Rotate a point counter-clockwise by quads 90 degree steps */
    switch (quads & 3)
    {
    case 0:
        y.re = x->re;
        y.im = x->im;
        break;
    case 1:
        y.re = -x->im;
        y.im = x->re;
        break;
    case 2:
        y.re = -x->re;
        y.im = -x->im;
        break;
    case 3:
        y.re = x->im;
        y.im = -x->re;
        break;
    }
    /*endswitch*/
    return y;
}
/*- End of function --------------------------------------------------------*/

/* Determine the 3 bits subset label for a particular constellation point */
static int16_t get_binary_subset_label(complexi16_t *pos)
{
    int x;
    int xored;
    int16_t subset;

    /* See V.34/9.6.3.1 */
    xored = pos->re ^ pos->im;
    x = xored & 2;
    subset = ((xored & 4) ^ (x << 1)) | (pos->re & 2) | (x >> 1);
    //printf("XXX Pre subset %d,%d => %d\n", pos->re, pos->im, subset);
    return subset;
}
/*- End of function --------------------------------------------------------*/

static int16_t get_binary_subset_label_q9_7(const complexi16_t *pos)
{
    complexi16_t integer;

    integer.re = pos->re >> 7;
    integer.im = pos->im >> 7;
    return get_binary_subset_label(&integer);
}
/*- End of function --------------------------------------------------------*/

static complexi16_t quantize_rx(v34_rx_state_t *s, complexi16_t *x)
{
    complexi16_t y;

    /* Value is stored in Q9.7 format. */
    /* Output integer values. i.e. Q16.0 */
    y.re = abs(x->re);
    y.im = abs(x->im);
    if (s->parms.b >= 56)
    {
        /* 2w is 4 */
        /* We must mask out the 1st and 2nd bits, because we are rounding to the 3rd bit.
           All numbers coming out of this routine should be a multiple of 4. */
        y.re = (y.re + 0x0FF) >> 7;
        y.re &= ~0x03;
        y.im = (y.im + 0x0FF) >> 7;
        y.im &= ~0x03;
    }
    else
    {
        /* 2w is 2 */
        /* We must mask out the 1st bit, because we are rounding to the 2nd bit.
           All numbers coming out of this routine should be even. */
        y.re = (y.re + 0x07F) >> 7;
        y.re &= ~0x01;
        y.im = (y.im + 0x07F) >> 7;
        y.im &= ~0x01;
    }
    /*endif*/
    if (x->re < 0)
        y.re = -y.re;
    /*endif*/
    if (x->im < 0)
        y.im = -y.im;
    /*endif*/
    return y;
}
/*- End of function --------------------------------------------------------*/

static complexi16_t precoder_rx_filter(v34_rx_state_t *s)
{
    /* h's are stored in Q2.14
       x's are stored in Q9.7
       not sure about x's
       so product is stored in Q11.21 */
    int i;
    complexi32_t sum;
    complexi16_t p;

    sum.re = 0;
    sum.im = 0;
    for (i = 0;  i < 3;  i++)
    {
        sum.re += ((int32_t) s->x[i].re*s->h[i].re - (int32_t) s->x[i].im*s->h[i].im);
        sum.im += ((int32_t) s->x[i].re*s->h[i].im + (int32_t) s->x[i].im*s->h[i].re);
    }
    /*endfor*/
    /* Round Q11.21 number format to Q9.7 */
    p.re = (abs(sum.re) + 0x01FFFL) >> 14;
    if (sum.re < 0)
        p.re = -p.re;
    /*endif*/
    p.im = (abs(sum.im) + 0x01FFFL) >> 14;
    if (sum.im < 0)
        p.im = -p.im;
    /*endif*/
    for (i = 2;  i > 0;  i--)
        s->x[i] = s->x[i - 1];
    /*endfor*/
    return p;
}
/*- End of function --------------------------------------------------------*/

static complexi16_t prediction_error_filter(v34_rx_state_t *s)
{
    int i;
    complexi32_t sum;
    complexi16_t yt;

    sum.re = (int32_t) s->xt[0].re*16384;
    sum.im = (int32_t) s->xt[0].im*16384;
    for (i = 0;  i < 3;  i++)
    {
        sum.re += ((int32_t) s->xt[i + 1].re*s->h[i].re - (int32_t) s->xt[i + 1].im*s->h[i].im);
        sum.im += ((int32_t) s->xt[i + 1].im*s->h[i].re + (int32_t) s->xt[i + 1].re*s->h[i].im);
    }
    /*endfor*/
    for (i = 3;  i > 0;  i--)
        s->xt[i] = s->xt[i - 1];
    /*endfor*/
    /* Round Q11.21 number format to Q9.7 */
    yt.re = (abs(sum.re) + 0x01FFFL) >> 14;
    if (sum.re < 0)
        yt.re = -yt.re;
    /*endif*/
    yt.im = (abs(sum.im) + 0x01FFFL) >> 14;
    if (sum.im < 0)
        yt.im = -yt.im;
    /*endif*/
    return yt;
}
/*- End of function --------------------------------------------------------*/

void v34_rx_quantize_n_ways(complexi16_t xy[], complexi16_t *yt)
{
    int16_t q;

    /* Quantize the current x,y point to points in the 4 2D subsets */
    /* TODO: This suits the 16 way convolutional code. The 32 and 64 way codes need 8 way quantization here */

    /* We want to quantize to a -7, -3, 1, 5, 9 grid, but -8, -4, 0, 4, 8 is easier to deal with.
       We subtract 1, quantize to the nearest multiple of 4, and add the 1 back. */
    /* Note that this works in Q9.7 format. */

    /* Offset by one */
    xy[0].re = yt->re - FP_Q9_7(1);
    xy[0].im = yt->im - FP_Q9_7(1);
    /* Round to the nearest multiple of 4 towards zero */
    q = xy[0].re;
    xy[0].re = (abs(xy[0].re) + FP_Q9_7(2)) & ~(FP_Q9_7(4) - 1);
    if (q < 0)
        xy[0].re = -xy[0].re;
    /*endif*/
    q = xy[0].im;
    xy[0].im = (abs(xy[0].im) + FP_Q9_7(2)) & ~(FP_Q9_7(4) - 1);
    if (q < 0)
        xy[0].im = -xy[0].im;
    /*endif*/
    /* Restore the offset of one */
    xy[0].re += FP_Q9_7(1);
    xy[0].im += FP_Q9_7(1);

    /* Subset 0 done. Figure out the rest as offsets from subset 0 */
    xy[1].re = xy[0].re;
    if (yt->re < xy[0].re)
    {
        xy[2].re = xy[0].re - FP_Q9_7(2);
        xy[3].re = xy[0].re - FP_Q9_7(2);
    }
    else
    {
        xy[2].re = xy[0].re + FP_Q9_7(2);
        xy[3].re = xy[0].re + FP_Q9_7(2);
    }
    /*endif*/
    if (yt->im < xy[0].im)
    {
        xy[1].im = xy[0].im - FP_Q9_7(2);
        xy[2].im = xy[0].im - FP_Q9_7(2);
    }
    else
    {
        xy[1].im = xy[0].im + FP_Q9_7(2);
        xy[2].im = xy[0].im + FP_Q9_7(2);
    }
    /*endif*/
    xy[3].im = xy[0].im;
}
/*- End of function --------------------------------------------------------*/

static void viterbi_calculate_candidate_errors(int16_t error[4], complexi16_t xy[4], complexi16_t *yt)
{
    int i;
    complexi32_t diff;
    int32_t err;

    /* Calculate the errors between yt and the four 2D candidates. Errors are stored as 6:10 */
//printf("CIC");
    for (i = 0;  i < 4;  i++)
    {
        diff.re = (int32_t) xy[i].re - yt->re;
        diff.im = (int32_t) xy[i].im - yt->im;
        err = diff.re*diff.re + diff.im*diff.im;
        error[i] = err >> 4;
//printf(" %3d", error[i]);
    }
    /*endfor*/
//printf("\n");
}
/*- End of function --------------------------------------------------------*/

static void viterbi_calculate_branch_errors(viterbi_t *s, complexi16_t xy[2][4], int invert)
{
    static const uint8_t kk[8][4] =
    {
        {0, 0, 2, 2},
        {0, 1, 2, 3},
        {0, 2, 2, 0},
        {0, 3, 2, 1},
        {1, 1, 3, 3},
        {1, 2, 3, 0},
        {1, 3, 3, 1},
        {1, 0, 3, 2}
    };
    int br;
    int n;
    int inv;
    int error0;
    int error1;
    int smaller;
    int k0;
    int k1;

    inv = (invert)  ?  4  :  0;
    for (br = 0;  br < 8;  br++)
    {
        n = br ^ inv;
        error0 = s->error[0][kk[n][0]] + s->error[1][kk[n][1]];
        error1 = s->error[0][kk[n][2]] + s->error[1][kk[n][3]];
        if (error0 < error1)
        {
            smaller = error0;
            k0 = kk[n][0];
            k1 = kk[n][1];
        }
        else
        {
            smaller = error1;
            k0 = kk[n][2];
            k1 = kk[n][3];
        }
        /*endif*/
        s->branch_error[br] = smaller;
        s->vit[s->ptr].branch_error_x[br] = smaller;
        s->vit[s->ptr].bb[0][br] = xy[0][k0];
        s->vit[s->ptr].bb[1][br] = xy[1][k1];
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void viterbi_update_path_metrics(viterbi_t *s,
                                        complexi16_t xy[2][4],
                                        int invert)
{
    static const int8_t geometric_branch[4][4] =
    {
        {0, 1, 2, 3},
        {7, 4, 5, 6},
        {2, 3, 0, 1},
        {5, 6, 7, 4}
    };
    int16_t state;
    int16_t k0;
    int16_t k1;
    uint32_t curr_min_metric;
    uint32_t metric;
    int prev_ptr;

    if (V34_DIAG_GETENV("SPANDSP_V34_DIAG_VITERBI"))
    {
        static int diag_state = 0;
        static int diag_count = 0;
        int dk0 = -1;
        int dk1 = -1;

        for (k0 = 0; k0 < 4; k0++)
        {
            if (s->error[0][k0] == 0)
                dk0 = k0;
            if (s->error[1][k0] == 0)
                dk1 = k0;
        }
        if (dk0 >= 0 && dk1 >= 0)
        {
            int subset0 = get_binary_subset_label_q9_7(&xy[0][dk0]) & 7;
            int subset1 = get_binary_subset_label_q9_7(&xy[1][dk1]) & 7;
            int input = conv_encode_input[subset0][subset1];
            int expected = (((input & 3) << 1) | (diag_state & 1))
                         ^ (invert ? 1 : 0);

            fprintf(stderr,
                    "V34VIT n=%d state=%d y0=%d input=%x k=%d,%d geom=%d expected=%d invert=%d next=%d\n",
                    diag_count,
                    diag_state,
                    diag_state & 1,
                    input,
                    dk0,
                    dk1,
                    geometric_branch[dk0][dk1],
                    expected,
                    invert,
                    s->encode_table[diag_state][input]);
            diag_state = s->encode_table[diag_state][input];
            diag_count++;
        }
    }

    curr_min_metric = UINT32_MAX;
    /* Build the full 4D candidate set.  The old dormant decoder collapsed
       these to eight branches and then tried to invert a generated table;
       that loses the actual Table-13 subset pair and is not a valid trellis
       metric. */
    for (k0 = 0; k0 < 4; k0++)
    {
        for (k1 = 0; k1 < 4; k1++)
        {
            int branch = 4*k0 + k1;
            s->vit[s->ptr].bb[0][branch] = xy[0][k0];
            s->vit[s->ptr].bb[1][branch] = xy[1][k1];
            s->vit[s->ptr].branch_error_x[branch] =
                s->error[0][k0] + s->error[1][k1];
        }
        /*endfor*/
    }
    /*endfor*/
    prev_ptr = (s->ptr - 1) & 0xF;
    for (state = 0; state < s->state_count; state++)
    {
        s->vit[s->ptr].cumulative_path_metric[state] = UINT32_MAX;
        s->vit[s->ptr].previous_path_ptr[state] = 0;
        s->vit[s->ptr].pts[state] = 0;
    }
    /*endfor*/
    for (state = 0; state < s->state_count; state++)
    {
        for (k0 = 0; k0 < 4; k0++)
        {
            int subset0 = get_binary_subset_label_q9_7(&xy[0][k0]) & 7;

            for (k1 = 0; k1 < 4; k1++)
            {
                int subset1 = get_binary_subset_label_q9_7(&xy[1][k1]) & 7;
                int input = conv_encode_input[subset0][subset1];
                int next_state = s->encode_table[state][input];
                int branch = 4*k0 + k1;

                /* Table 11 steps 4-6 and 9.6.3: with h=0, C0=0 and
                   U0=Y0 XOR V0. The quarter-superconstellation has zero
                   quadrant label, so the parity of the two subset labels'
                   quadrant difference is U0. Nonzero precoders need C0
                   carried with each survivor and retain the existing path. */
                if (s->zero_precoder
                    && ((subset1 - subset0) & 1) != ((state & 1) ^ (invert != 0)))
                    continue;
                if (s->vit[prev_ptr].cumulative_path_metric[state] == UINT32_MAX)
                    continue;

                /* V.34 9.6.3: Table 13 supplies Y4321 and the authoritative
                   encoder table constrains its state transition. U0 cannot be
                   inferred from state&1 alone: 9.6.3.3 adds C0 and Table 12
                   adds V0. A constrained decoder must carry the precoder and
                   modulo history per survivor; rejecting candidates here
                   without that state corrupts even exact precoded symbols.
                   The zero-precoder case above needs no survivor C0 history. */
                metric = s->vit[prev_ptr].cumulative_path_metric[state]
                       + s->vit[s->ptr].branch_error_x[branch];
                if (metric < s->vit[s->ptr].cumulative_path_metric[next_state])
                {
                    s->vit[s->ptr].cumulative_path_metric[next_state] = metric;
                    s->vit[s->ptr].previous_path_ptr[next_state] = state;
                    s->vit[s->ptr].pts[next_state] = branch;
                }
                /*endif*/
            }
            /*endfor*/
        }
        /*endfor*/
    }
    /*endfor*/
    for (state = 0; state < s->state_count; state++)
    {
        metric = s->vit[s->ptr].cumulative_path_metric[state];
        if (metric < curr_min_metric)
        {
            curr_min_metric = metric;
            s->curr_min_state = state;
        }
        /*endif*/
    }
    /*endfor*/
//printf("GGG %p min metric %d, state %d\n", s, curr_min_metric, s->curr_min_state);
//printf("JJJ %p ", s);
    for (state = 0; state < s->state_count; state++)
    {
        if (s->vit[s->ptr].cumulative_path_metric[state] != UINT32_MAX)
            s->vit[s->ptr].cumulative_path_metric[state] -= curr_min_metric;
//printf("%4d ", s->cumulative_path_metric[s->ptr][i]);
    }
    /*endfor*/
//printf("\n");
}
/*- End of function --------------------------------------------------------*/

static void viterbi_trace_back(viterbi_t *s, complexi16_t y[2])
{
    int branch;
    int next_state;
    int last_baud;
    int i;

    next_state = s->curr_min_state;
    last_baud = (s->ptr - 15) & 0xF;
//printf("FFF %p %2d", s, next_state);
    for (i = s->ptr;  i != last_baud;  i = (i - 1) & 0xF)
    {
        next_state = s->vit[i].previous_path_ptr[next_state];
//printf(" %2d", next_state);
    }
    /*endfor*/
    for (i = 0;  i < 8;  i++)
    {
        if (s->vit[last_baud].branch_error_x[i] == 0)
        {
            branch = i;
            break;
        }
    }
    /*endfor*/
    branch = s->vit[last_baud].pts[next_state];
//printf(" (%d)\n", branch);

    y[0] = s->vit[last_baud].bb[0][branch];
    y[1] = s->vit[last_baud].bb[1][branch];
}
/*- End of function --------------------------------------------------------*/

static int viterbi_set_trellis(viterbi_t *s,
                               const uint8_t (*encode_table)[16],
                               int state_count)
{
    int state;
    int input;
    int previous;
    int found;

    if (!s || !encode_table
        || (state_count != 16 && state_count != 32 && state_count != 64))
        return -1;
    /* In Figures 10-12/V.34 each destination state has one predecessor for
       each of the four coded input pairs.  Build the inverse mapping from the
       authoritative encoder tables so the receiver supports every negotiated
       trellis without maintaining another set of generated tables. */
    for (state = 0;  state < state_count;  state++)
    {
        for (input = 0;  input < 4;  input++)
        {
            found = -1;
            for (previous = 0;  previous < state_count;  previous++)
            {
                if (encode_table[previous][input] == state)
                    found = previous;
                /*endif*/
            }
            /*endfor*/
            if (found < 0)
                return -1;
            s->previous_state[state][input] = (uint8_t) found;
            s->branch[state][input] = (uint8_t) ((input << 1) | (found & 1));
        }
        /*endfor*/
    }
    /*endfor*/
    s->state_count = state_count;
    s->zero_precoder = false;
    s->encode_table = encode_table;
    memset(s->vit, 0, sizeof(s->vit));
    s->ptr = 0;
    s->windup = 15;
    s->curr_min_state = 0;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int trellis_parameters(int trellis_size,
                              const uint8_t (**table)[16],
                              int *states)
{
    switch (trellis_size)
    {
    case V34_TRELLIS_16:
        *table = v34_conv16_encode_table;
        *states = 16;
        return 0;
    case V34_TRELLIS_32:
        *table = v34_conv32_encode_table;
        *states = 32;
        return 0;
    case V34_TRELLIS_64:
        *table = v34_conv64_encode_table;
        *states = 64;
        return 0;
    default:
        return -1;
    }
    /*endswitch*/
}
/*- End of function --------------------------------------------------------*/

static int set_tx_trellis_mode(v34_state_t *s, int trellis_size)
{
    const uint8_t (*table)[16];
    int states;

    if (trellis_parameters(trellis_size, &table, &states))
        return -1;
    (void) states;
    s->tx.conv_encode_table = table;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int set_rx_trellis_mode(v34_state_t *s, int trellis_size)
{
    const uint8_t (*table)[16];
    int states;

    if (trellis_parameters(trellis_size, &table, &states))
        return -1;
    return viterbi_set_trellis(&s->rx.viterbi, table, states);
}
/*- End of function --------------------------------------------------------*/

static int set_trellis_mode(v34_state_t *s, int trellis_size)
{
    /* Offline seed/restart callers configure a matched local pair.  Live MP
       processing uses the directional helpers: V.34 10.1.3.9 says the
       trellis field selects the remote-end transmitter, not both channels. */
    return set_tx_trellis_mode(s, trellis_size)
        || set_rx_trellis_mode(s, trellis_size);
}
/*- End of function --------------------------------------------------------*/

static __inline__ float exact_baud_rate(int symbol_rate_code)
{
    float a;
    float c;

    a = baud_rate_parameters[symbol_rate_code].a;
    c = baud_rate_parameters[symbol_rate_code].c;
    return 2400.0f*a/c;
}
/*- End of function --------------------------------------------------------*/

static __inline__ float carrier_frequency(int symbol_rate_code, int low_high)
{
    float d;
    float e;

    d = baud_rate_parameters[symbol_rate_code].low_high[low_high].d;
    e = baud_rate_parameters[symbol_rate_code].low_high[low_high].e;
    return exact_baud_rate(symbol_rate_code)*d/e;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_info0(v34_rx_state_t *s, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;
    uint8_t raw_26_27;

    memset(&s->far_capabilities, 0, sizeof(s->far_capabilities));
    s->info0d_extensions_valid = false;
    s->info0_raw_26_27 = 0;
    s->info0d_nominal_power_code = 0;
    s->info0d_max_power_code = 0;
    s->info0d_power_measured_at_codec_output = false;
    s->info0d_pcm_alaw = false;
    s->info0d_upstream_3429_support = false;
    s->info0d_reserved_41 = 0;
    bitstream_init(&bs, true);
    t = buf;
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2400] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_2400] = true;
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2743] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_2743] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2800] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_2800] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3429] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3429] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3000] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3000] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3200] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3200] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.rate_3429_allowed = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_power_reduction = bitstream_get(&bs, &t, 1);
    s->far_capabilities.max_baud_rate_difference = bitstream_get(&bs, &t, 3);
    s->far_capabilities.from_cme_modem = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_1664_point_constellation = bitstream_get(&bs, &t, 1);
    if (s->v90_mode && s->calling_party)
    {
        /* V.90 INFO0d (62 bits): bits 26-27 are reserved (not tx_clock_source),
           bit 28 is acknowledgement, then additional digital modem fields follow. */
        raw_26_27 = bitstream_get(&bs, &t, 2);     /* 26:27 reserved (V.90), V.92 flags */
        s->info0_raw_26_27 = raw_26_27;
        s->info0_acknowledgement = bitstream_get(&bs, &t, 1);  /* 28 */
        /* 29:32    Digital modem nominal TX power (-1 dBm0 steps, 0=-6 dBm0) */
        s->info0d_nominal_power_code = bitstream_get(&bs, &t, 4);
        /* 33:37    Maximum digital modem TX power (-0.5 dBm0 steps) */
        s->info0d_max_power_code = bitstream_get(&bs, &t, 5);
        /* 38       Power measurement location (1=codec output) */
        s->info0d_power_measured_at_codec_output = bitstream_get(&bs, &t, 1);
        /* 39       PCM coding: 0=µ-law, 1=A-law */
        s->far_capabilities.tx_clock_source = bitstream_get(&bs, &t, 1);  /* reuse field for pcm_law */
        s->info0d_pcm_alaw = (s->far_capabilities.tx_clock_source != 0);
        /* 40       V.90 upstream symbol rate 3429 support */
        s->info0d_upstream_3429_support = bitstream_get(&bs, &t, 1);
        /* 41       Reserved */
        s->info0d_reserved_41 = bitstream_get(&bs, &t, 1);
        s->info0d_extensions_valid = true;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx INFO0d (V.90): PCM law=%s, ack=%d\n",
                 s->far_capabilities.tx_clock_source ? "A-law" : "u-law",
                 s->info0_acknowledgement);
    }
    else
    {
        raw_26_27 = bitstream_get(&bs, &t, 2);
        s->info0_raw_26_27 = raw_26_27;
        s->far_capabilities.tx_clock_source = raw_26_27;
        s->info0_acknowledgement = bitstream_get(&bs, &t, 1);
        if (s->v90_mode)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx INFO0a V.92 flags: capability(bit26)=%d, short-phase2(bit27)=%d\n",
                     (raw_26_27 & 0x01U) != 0,
                     (raw_26_27 & 0x02U) != 0);
        }
    }
    /*endif*/

    log_info0(s->logging, false, &s->far_capabilities, s->info0_acknowledgement);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_info1c(v34_rx_state_t *s, info1c_t *info1c, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;
    int i;

    bitstream_init(&bs, true);
    t = buf;
    /* 12:14    Minimum power reduction to be implemented by the answer modem transmitter. An integer between 0 and 7
                gives the recommended power reduction in dB. These bits shall indicate 0 if INFO0a indicated that the answer
                modem transmitter cannot reduce its power. */
    info1c->power_reduction = bitstream_get(&bs, &t, 3);
    /* 15:17    Additional power reduction, below that indicated by bits 12-14, which can be tolerated by the call modem
                receiver. An integer between 0 and 7 gives the additional power reduction in dB. These bits shall indicate 0 if
                INFO0a indicated that the answer modem transmitter cannot reduce its power. */
    info1c->additional_power_reduction = bitstream_get(&bs, &t, 3);
    /* 18:24    Length of MD to be transmitted by the call modem during Phase 3. An integer between 0 and 127 gives the
                length of this sequence in 35 ms increments. */
    info1c->md = bitstream_get(&bs, &t, 7);
    /* 25       Set to 1 indicates that the high carrier frequency is to be used in transmitting from the answer modem to the call
                modem for a symbol rate of 2400. */
    /* 26:29    Pre-emphasis filter to be used in transmitting from the answer modem to the call modem for a symbol
                rate of 2400. These bits form an integer between 0 and 10 which represents the pre-emphasis filter index
                (see Tables 3 and 4). */
    /* 30:33    Projected maximum data rate for a symbol rate of 2400. These bits form an integer between 0 and 14 which
                gives the projected data rate as a multiple of 2400 bits/s. A 0 indicates the symbol rate cannot be used. */

    /* 34:42    Probing results pertaining to a final symbol rate selection of 2743 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. */

    /* 43:51    Probing results pertaining to a final symbol rate selection of 2800 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. */

    /* 52:60    Probing results pertaining to a final symbol rate selection of 3000 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. Information in this field shall be consistent with the answer modem
                capabilities indicated in INFO0a. */

    /* 61:69    Probing results pertaining to a final symbol rate selection of 3200 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. Information in this field shall be consistent with the answer modem
                capabilities indicated in INFO0a. */

    /* 70:78    Probing results pertaining to a final symbol rate selection of 3429 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. Information in this field shall be consistent with the answer modem
                capabilities indicated in INFO0a. */
    for (i = 0;  i <= 5;  i++)
    {
        info1c->rate_data[i].use_high_carrier = bitstream_get(&bs, &t, 1);
        info1c->rate_data[i].pre_emphasis = bitstream_get(&bs, &t, 4);
        info1c->rate_data[i].max_bit_rate = bitstream_get(&bs, &t, 4);
    }
    /*endfor*/
    /* 79:88    Frequency offset of the probing tones as measured by the call modem receiver. The frequency offset number
                shall be the difference between the nominal 1050 Hz line probing signal tone received and the 1050 Hz tone
                transmitted, f(received) and f(transmitted). A two's complement signed integer between -511 and 511 gives the
                measured offset in 0.02 Hz increments. Bit 88 is the sign bit of this integer. The frequency offset measurement
                shall be accurate to 0.25 Hz. Under conditions where this accuracy cannot be achieved, the integer shall be set
                to -512 indicating that this field is to be ignored. */
    info1c->freq_offset = bitstream_get(&bs, &t, 10);
    if ((info1c->freq_offset & 0x200))
        info1c->freq_offset = -(info1c->freq_offset ^ 0x3FF) - 1;
    /*endif*/

    log_info1c(s->logging, false, info1c);
    s->info1c_received = true;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_info1a(v34_rx_state_t *s, info1a_t *info1a, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;
    uint16_t raw_freq;
    info1a_t saved_info1a;

    bitstream_init(&bs, true);
    t = buf;

    if (s->v90_mode)
    {
        /* V.90 §8.2.3.2 Table 10: INFO1a from analog modem when V.90 is selected.
           Different field layout from V.34 INFO1a. */
        /* 12:17    Reserved for ITU (set to 0 by analog modem) */
        s->info1a_raw_12_17 = bitstream_get(&bs, &t, 6);
        info1a->power_reduction = 0;
        info1a->additional_power_reduction = 0;
        /* 18:24    Length of MD to be transmitted by the analog modem during Phase 3 */
        info1a->md = bitstream_get(&bs, &t, 7);
        /* 25:31    U_INFO: Ucode of the PCM codeword for the 2-point train.
                    Stored in use_high_carrier (1 bit) + preemphasis_filter (4 bits) + 2 extra bits.
                    We pack the 7-bit value into max_data_rate for the caller to retrieve. */
        info1a->max_data_rate = bitstream_get(&bs, &t, 7);  /* U_INFO */
        info1a->use_high_carrier = false;
        info1a->preemphasis_filter = 0;
        /* V.90 Table 10 reserves 32:33.  V.92 Table 19 retains bit 32 as
           reserved and uses bit 33 to select the high upstream carrier. */
        s->info1a_raw_32_33 = bitstream_get(&bs, &t, 2);
        /* 34:36    Symbol rate for analog→digital (upstream). 3=3000, 4=3200, 5=3429 */
        info1a->baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
        /* 37:39    Symbol rate of 8000 (the integer 6) — V.90 PCM downstream rate */
        info1a->baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
        /* 40:49    Frequency offset (same as V.34) */
        raw_freq = (uint16_t) bitstream_get(&bs, &t, 10);
        s->info1a_raw_40_49 = raw_freq;
        info1a->freq_offset = (int) raw_freq;
        if ((info1a->freq_offset & 0x200))
            info1a->freq_offset = -(info1a->freq_offset ^ 0x3FF) - 1;
        /*endif*/

        if (info1a->baud_rate_c_to_a >= 0  &&  info1a->baud_rate_c_to_a <= 5)
        {
            /* V.90 §8.2.3.2 Table 11 / §9.2.1.1.8: bits 37:39 in 0..5 means
               the analogue modem selected V.34.  The frame then carries the
               standard V.34 INFO1a fields (Table 11 is bit-identical to
               Table 16/V.34), NOT the Table 10 U_INFO layout parsed above.
               Re-parse from the top with the V.34 field layout.  Live ground
               truth (CX93001, 2026-07-23): raw 0f c0 34 89 c0 ba 66 decodes
               to power reduction 7+1 dB, low carrier, pre-emphasis 3,
               projected max 31200, 3200/3200 baud -- all coherent, where the
               Table 10 reading gave nonzero "reserved" bits. */
            bitstream_init(&bs, true);
            t = buf;
            info1a->power_reduction = bitstream_get(&bs, &t, 3);
            info1a->additional_power_reduction = bitstream_get(&bs, &t, 3);
            info1a->md = bitstream_get(&bs, &t, 7);
            info1a->use_high_carrier = bitstream_get(&bs, &t, 1);
            info1a->preemphasis_filter = bitstream_get(&bs, &t, 4);
            info1a->max_data_rate = bitstream_get(&bs, &t, 4);
            info1a->baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
            info1a->baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
            raw_freq = (uint16_t) bitstream_get(&bs, &t, 10);
            s->info1a_raw_40_49 = raw_freq;
            info1a->freq_offset = (int) raw_freq;
            if ((info1a->freq_offset & 0x200))
                info1a->freq_offset = -(info1a->freq_offset ^ 0x3FF) - 1;
            /*endif*/
            s->v90_v34_fallback = true;

            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx INFO1a (V.90 Table 11 - V.34 selected):\n");
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Power reduction = %d dB + %d dB additional\n",
                     info1a->power_reduction, info1a->additional_power_reduction);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Length of MD = %dms\n", info1a->md*35);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  High carrier (digital->analogue) = %d\n", info1a->use_high_carrier);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Pre-emphasis filter = %d\n", info1a->preemphasis_filter);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Projected max rate = %d (%d bps)\n",
                     info1a->max_data_rate, info1a->max_data_rate*2400);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Symbol rate analogue->digital = %d\n", info1a->baud_rate_a_to_c);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Symbol rate digital->analogue = %d\n", info1a->baud_rate_c_to_a);
            if (info1a->freq_offset == -512)
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Frequency offset not available\n");
            else
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Frequency offset = %fHz\n", info1a->freq_offset*0.02f);

            /* The analogue modem transmits Phase 3 at the a->c symbol rate.
               V.34 §10.1.2.3.4/Table 12: carrier and pre-emphasis are those
               this digital modem already indicated for that row in INFO1d. */
            if (info1a->baud_rate_a_to_c >= 0  &&  info1a->baud_rate_a_to_c <= 5)
            {
                s->baud_rate = info1a->baud_rate_a_to_c;
                s->high_carrier = s->local_info1c_high_carrier[s->baud_rate];
                s->v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->baud_rate, s->high_carrier));
                create_godard_coeffs(&s->pri_ted,
                                     carrier_frequency(s->baud_rate, s->high_carrier),
                                     baud_rate_parameters[s->baud_rate].baud_rate,
                                     0.99f);
            }
            /*endif*/
        }
        else
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx INFO1a (V.90 Table 10):\n");
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Length of MD = %dms\n", info1a->md*35);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  U_INFO = %d\n", info1a->max_data_rate);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Upstream symbol rate code = %d\n", info1a->baud_rate_a_to_c);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Downstream rate code = %d (8000 PCM)\n", info1a->baud_rate_c_to_a);
            if (info1a->freq_offset == -512)
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Frequency offset not available\n");
            else
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "  Frequency offset = %fHz\n", info1a->freq_offset*0.02f);

            /* V.90 §8.2.3.2 Tables 9/10: INFO1a selects the upstream row;
               bits 32:33 are reserved and do not repeat its carrier choice.
               Apply the row this digital modem retained when sending INFO1d. */
            if (info1a->baud_rate_a_to_c >= 0  &&  info1a->baud_rate_a_to_c <= 5)
            {
                s->baud_rate = info1a->baud_rate_a_to_c;
                s->high_carrier = s->local_info1c_high_carrier[s->baud_rate];
                s->v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->baud_rate, s->high_carrier));
                create_godard_coeffs(&s->pri_ted,
                                     carrier_frequency(s->baud_rate, s->high_carrier),
                                     baud_rate_parameters[s->baud_rate].baud_rate,
                                     0.99f);
            }
            /*endif*/
        }
        /*endif*/
    }
    else
    {
        s->info1a_raw_12_17 = 0;
        s->info1a_raw_32_33 = 0;
        s->info1a_raw_40_49 = 0;
        /* Standard V.34 INFO1a parsing */
        saved_info1a = *info1a;
        /* 12:14    Minimum power reduction */
        info1a->power_reduction = bitstream_get(&bs, &t, 3);
        /* 15:17    Additional power reduction */
        info1a->additional_power_reduction = bitstream_get(&bs, &t, 3);
        /* 18:24    Length of MD */
        info1a->md = bitstream_get(&bs, &t, 7);
        /* 25       High carrier frequency */
        info1a->use_high_carrier = bitstream_get(&bs, &t, 1);
        /* 26:29    Pre-emphasis filter index */
        info1a->preemphasis_filter = bitstream_get(&bs, &t, 4);
        /* 30:33    Projected maximum data rate */
        info1a->max_data_rate = bitstream_get(&bs, &t, 4);
        /* 34:36    Symbol rate answer→call */
        info1a->baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
        /* 37:39    Symbol rate call→answer */
        info1a->baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
        /* 40:49    Frequency offset */
        info1a->freq_offset = bitstream_get(&bs, &t, 10);
        if ((info1a->freq_offset & 0x200))
            info1a->freq_offset = -(info1a->freq_offset ^ 0x3FF) - 1;
        /*endif*/
        /* Table 16: symbol-rate indices 0..5, pre-emphasis 0..10, projected
           rate 0..14.  A frame naming anything else is not a negotiation:
           reject it whole, before it can retune the receiver or publish
           INFO1_OK, and keep the previous values. */
        if (info1a->baud_rate_a_to_c > V34_BAUD_RATE_3429
            || info1a->baud_rate_c_to_a > V34_BAUD_RATE_3429
            || info1a->preemphasis_filter > 10
            || info1a->max_data_rate > 14)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                       "Rx INFO1a rejected: undefined selection (a2c baud %d, c2a baud %d, "
                       "pre-emphasis %d, projected rate %d)\n",
                       info1a->baud_rate_a_to_c, info1a->baud_rate_c_to_a,
                       info1a->preemphasis_filter, info1a->max_data_rate);
            *info1a = saved_info1a;
            return -1;
        }
        /*endif*/
        /* V.34 10.1.2.3.5/Table 16 bits 34:36 select answer->call,
           which is this call modem receiver's direction.  Carrier and
           pre-emphasis came from the INFO1c row we transmitted earlier.
           Bits 37:39 select our transmitter and are applied by v34tx.c. */
        if (info1a->baud_rate_a_to_c >= V34_BAUD_RATE_2400
            && info1a->baud_rate_a_to_c <= V34_BAUD_RATE_3429)
        {
            s->baud_rate = info1a->baud_rate_a_to_c;
            s->high_carrier = s->local_info1c_high_carrier[s->baud_rate];
            s->v34_carrier_phase_rate =
                dds_phase_ratef(carrier_frequency(s->baud_rate, s->high_carrier));
            create_godard_coeffs(&s->pri_ted,
                                 carrier_frequency(s->baud_rate, s->high_carrier),
                                 baud_rate_parameters[s->baud_rate].baud_rate,
                                 0.99f);
        }

        log_info1a(s->logging, false, info1a);
    }
    /*endif*/
    /* One line per INFO1a.  Table 16 is the answer modem's verdict on OUR
       transmit direction (bits 12:17 power, 25 carrier, 26:29 pre-emphasis,
       30:33 projected rate, 37:39 symbol rate) and it was visible nowhere on
       a live call. */
    fprintf(stderr, "[V34] INFO1a: c2a (our TX) baud=%d high_carrier=%d pre_emphasis=%d "
            "projected_N=%d power_reduction=%d+%d; a2c baud=%d "
            "(baud 0..5 = 2400/2743/2800/3000/3200/3429, 6 = V.90 PCM)\n",
            s->info1a.baud_rate_c_to_a, s->info1a.use_high_carrier ? 1 : 0,
            s->info1a.preemphasis_filter, s->info1a.max_data_rate,
            s->info1a.power_reduction, s->info1a.additional_power_reduction,
            s->info1a.baud_rate_a_to_c);
    s->info1a_received = true;

#if defined(SPANDSP_USE_FIXED_POINT)
    s->pri_ted.symbol_sync_low[0] = s->pri_ted.symbol_sync_low[1] = 0;
    s->pri_ted.symbol_sync_high[0] = s->pri_ted.symbol_sync_high[1] = 0;
    s->pri_ted.symbol_sync_dc_filter[0] = s->pri_ted.symbol_sync_dc_filter[1] = 0;
    s->pri_ted.baud_phase = 0;
#else
    s->pri_ted.symbol_sync_low[0] = s->pri_ted.symbol_sync_low[1] = 0.0f;
    s->pri_ted.symbol_sync_high[0] = s->pri_ted.symbol_sync_high[1] = 0.0f;
    s->pri_ted.symbol_sync_dc_filter[0] = s->pri_ted.symbol_sync_dc_filter[1] = 0.0f;
    s->pri_ted.baud_phase = 0.0f;
#endif

    return 0;
}
/*- End of function --------------------------------------------------------*/

static void v90_enter_phase3_from_info1a(v34_rx_state_t *s)
{
    v34_state_t *owner;

    /* INFO1a can finish part-way through an RTP media frame.  Merely changing
       current_demodulator here lets the uninitialised primary-channel frontend
       consume the residue of that frame before the TX state machine gets its
       next callback and performs the normal Phase 3 reset.  That shifted the
       live PP acquisition by 24 bauds relative to replay and destroyed the TRN
       lock.  Enter Phase 3 synchronously so the first primary-channel sample is
       processed with the same clean frontend used by offline replay. */
    owner = (v34_state_t *) ((char *) s - offsetof(v34_state_t, rx));

    /* Table 10 bits 37:39 confirm which protocol the analogue modem actually
       committed to: the integer 6 means genuine V.90 PCM downstream; 0-5
       means it declined V.90 and is falling back to plain V.34 at that
       symbol rate (V.90 9.2.1.1.8). The two cases need different upstream
       scrambler polynomials. Forcing the V.90 tap onto a real V.34-fallback
       signal corrupts every descrambled bit from here on — live interop
       with a CX93001 that declined V.90 showed exactly this: an MP
       hypothesis would lock, then every subsequent frame failed CRC and got
       rejected, forever (2026-07-19). */
    if (s->info1a.baud_rate_c_to_a == 6)
    {
        /* V.90 §9 uses the analog-modem upstream scrambler 1 + x^-5 + x^-23.
           SpanDSP's ordinary V.34 answerer initialisation selects tap 17 for
           the far-end caller, which makes SmartLink TRN look random (~52%
           ones). The captured upstream resolves at 96-99% with the V.90 tap
           value 4, so select it before Phase 3 resets its hypothesis banks. */
        s->scrambler_tap = 4;
        s->mp_phase4_default_scrambler_tap = 4;
    }
    else
    {
        /* V.90 §9.2.1.1.8: the digital modem proceeds per 11.3.1.1/V.34
           "assuming the role of a call modem", which makes the analogue
           modem the V.34 answer modem.  The answer modem scrambles with
           GPA (1 + x^-5 + x^-23), so v34_rx_descramble its TRN/J with tap 4 and
           scramble our own TX with GPC (tap 17).  The earlier tap-17 RX
           choice here dated from before the role mapping was pinned down
           (2026-07-19, when the whole fallback Phase 3 was desynced). */
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90: INFO1a declined PCM (downstream code=%d, not 6); "
                 "V.34 fallback, we take the call-modem role (RX descrambler GPA/tap 4, TX GPC/tap 17)\n",
                 s->info1a.baud_rate_c_to_a);
        s->scrambler_tap = 4;
        s->mp_phase4_default_scrambler_tap = 4;
        owner->tx.scrambler_tap = 17;
        owner->tx.v90_v34_fallback = true;
    }
    /*endif*/
    v34_force_phase3_rx(owner);

    if (!phase3_rx_dump_fp)
    {
        phase3_rx_dump_fp = fopen("/tmp/v90_phase3_rx.raw", "wb");
        phase3_rx_dump_count = 0;
        if (phase3_rx_dump_fp)
            fprintf(stderr, "[V34 RX] Phase 3 RX audio dump started -> /tmp/v90_phase3_rx.raw\n");
        /*endif*/
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int process_rx_infoh(v34_rx_state_t *s, infoh_t *infoh, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;

    memset(infoh, 0, sizeof(*infoh));
    bitstream_init(&bs, true);
    t = buf;
    /* 12:14    Power reduction requested by the recipient modem receiver. An integer between 0 and 7
                gives the requested power reduction in dB. These bits shall indicate 0 if the source
                modem's INFO0 indicated that the source modem transmitter cannot reduce its power. */
    infoh->power_reduction = bitstream_get(&bs, &t, 3);
    /* 15:21    Length of TRN to be transmitted by the source modem during Phase 3. An integer between
                0 and 127 gives the length of this sequence in 35 ms increments. */
    infoh->length_of_trn = bitstream_get(&bs, &t, 7);
    /* 22       Set to 1 indicates the high carrier frequency is to be used in data mode transmission. This
                must be consistent with the capabilities indicated in the source modem's INFO0. */
    infoh->use_high_carrier = bitstream_get(&bs, &t, 1);
    /* 23:26    Pre-emphasis filter to be used in transmitting from the source modem to the recipient modem.
                These bits form an integer between 0 and 10 which represents the pre-emphasis filter index
                (see Tables 3 and 4). */
    infoh->preemphasis_filter = bitstream_get(&bs, &t, 4);
    /* 27:29    Symbol rate to be used for data transmission. An integer between 0 and 5 gives the symbol rate, where 0
                represents 2400 and a 5 represents 3429. */
    infoh->baud_rate = bitstream_get(&bs, &t, 3);
    /* 30       Set to 1 indicates TRN uses a 16-point constellation, 0 indicates TRN uses a 4-point constellation. */
    infoh->trn16 = bitstream_get(&bs, &t, 1);

    log_infoh(s->logging, false, infoh);
    /* V.34 Table 22 reserves symbol-rate indices 6/7 and pre-emphasis
       indices 11..15. A valid CRC does not make those selections usable. */
    return (infoh->baud_rate <= 5 && infoh->preemphasis_filter <= 10) ? 0 : -1;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_mp(v34_rx_state_t *s, mp_t *mp, uint8_t buf[])
{
    int i;
    const uint8_t *t;
    bitstream_state_t bs;

    bitstream_init(&bs, true);
    t = buf;
    /* 18       Type */
    mp->type = bitstream_get(&bs, &t, 1);
    /* 19       Reserved by the ITU */
    bitstream_get(&bs, &t, 1);
    /* 20:23    Maximum call modem to answer modem data signalling rate: Data rate = N * 2400
                where N is a four-bit integer between 1 and 14. */
    mp->bit_rate_c_to_a = bitstream_get(&bs, &t, 4);
    /* 24:27    Maximum answer modem to call modem data signalling rate: Data rate = N * 2400
                where N is a four-bit integer between 1 and 14. */
    mp->bit_rate_a_to_c = bitstream_get(&bs, &t, 4);
    /* 28       Auxiliary channel select bit. Set to 1 if modem is capable of supporting and
                enables auxiliary channel. Auxiliary channel is used only if both modems set
                this bit to 1. */
    mp->aux_channel_supported = bitstream_get(&bs, &t, 1);
    /* 29:30    Trellis encoder select bits:
                0 = 16 state; 1 = 32 state; 2 = 64 state; 3 = Reserved for ITU-T.
                Receiver requires remote-end transmitter to use selected trellis encoder. */
    mp->trellis_size = bitstream_get(&bs, &t, 2);
    /* 31       Non-linear encoder parameter select bit for the remote-end transmitter.
                0: Q = 0, 1: Q = 0.3125. */
    mp->use_non_linear_encoder = bitstream_get(&bs, &t, 1);
    /* 32       Constellation shaping select bit for the remote-end transmitter. 0: minimum,
                1: expanded (see Table 10). */
    mp->expanded_shaping = bitstream_get(&bs, &t, 1);
    /* 33       Acknowledge bit. 0 = modem has not received MP from far end. 1 = received MP from far end. */
    mp->mp_acknowledged = bitstream_get(&bs, &t, 1);
    /* 34       Start bit: 0. */
    bitstream_get(&bs, &t, 1);
    /* 35:49    Data signalling rate capability mask.
                Bit 35:2400; bit 36:4800; bit 37:7200;...; bit 46:28800; bit 47:31200; bit 48:33600;
                bit 49: Reserved for ITU-T. (This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem.) Bits set to 1 indicate data signalling rates supported
                and enabled in both transmitter and receiver of modem. */
    mp->signalling_rate_mask = bitstream_get(&bs, &t, 15);
    /* 50       Asymmetric data signalling rate enable. 1 indicates a modem capable of
                asymmetric data signalling rates. */
    mp->asymmetric_rates_allowed = bitstream_get(&bs, &t, 1);
    if (mp->type == 1)
    {
        /* 51       Start bit: 0. */
        /* 52:67    Precoding coefficient h(1) real. */
        /* 68       Start bit: 0. */
        /* 69:84    Precoding coefficient h(1) imaginary. */
        /* 85       Start bit: 0. */
        /* 86:101   Precoding coefficient h(2) real. */
        /* 102      Start bit: 0. */
        /* 103:118  Precoding coefficient h(2) imaginary. */
        /* 119      Start bit: 0. */
        /* 120:135  Precoding coefficient h(3) real. */
        /* 136      Start bit: 0. */
        /* 137:152  Precoding coefficient h(3) imaginary. */
        for (i = 0;  i < 3;  i++)
        {
            bitstream_get(&bs, &t, 1);
            mp->precoder_coeffs[i].re = bitstream_get(&bs, &t, 16);
            bitstream_get(&bs, &t, 1);
            mp->precoder_coeffs[i].im = bitstream_get(&bs, &t, 16);
        }
        /*endfor*/
    }
    else
    {
        /* The following are not included in an MP0 message */
        for (i = 0;  i < 3;  i++)
        {
            mp->precoder_coeffs[i].re = 0;
            mp->precoder_coeffs[i].im = 0;
        }
        /*endfor*/
    }
    /*endif*/
    /* We can ignore the remaining bits. They are not used. */

    log_mp(s->logging, false, mp);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_mph(v34_rx_state_t *s, mph_t *mph, uint8_t buf[])
{
    int i;
    const uint8_t *t;
    bitstream_state_t bs;

    bitstream_init(&bs, true);
    t = buf;
    /* 18       Type */
    mph->type = bitstream_get(&bs, &t, 1);
    /* 19       Reserved by the ITU */
    bitstream_get(&bs, &t, 1);
    /* 20:23    Maximum data signalling rate:
                Data rate = N * 2400 where N is a 4-bit integer between 1 and 14. */
    mph->max_data_rate = bitstream_get(&bs, &t, 4);
    /* 24:26    Reserved for ITU-T: These bits are set to 0 by the transmitting modem and are
                not interpreted by the receiving modem. */
    bitstream_get(&bs, &t, 3);
    /* 27       Control channel data signalling rate selected for remote transmitter.
                0 = 1200 bit/s, 1 = 2400 bit/s (see bit 50 below). */
    mph->control_channel_2400 = bitstream_get(&bs, &t, 1);
    /* 28       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    bitstream_get(&bs, &t, 1);
    /* 29:30    Trellis encoder select bits:
                0 = 16 state; 1 = 32 state; 2 = 64 state; 3 = Reserved for ITU-T.
                Receiver requires remote-end transmitter to use selected trellis encoder. */
    mph->trellis_size = bitstream_get(&bs, &t, 2);
    /* 31       Non-linear encoder parameter select bit for the remote-end transmitter.
                0: Q = 0, 1: Q = 0.3125. */
    mph->use_non_linear_encoder = bitstream_get(&bs, &t, 1);
    /* 32       Constellation shaping select bit for the remote-end transmitter.
                0: minimum, 1: expanded (see Table 10). */
    mph->expanded_shaping = bitstream_get(&bs, &t, 1);
    /* 33       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    /* 34       Start bit: 0. */
    bitstream_get(&bs, &t, 2);
    /* 35:49    Data signalling rate capability mask.
                Bit 35:2400; bit 36:4800; bit 37:7200;...; bit 46:28800; bit 47:31200; bit 48:33600;
                bit 49: Reserved for ITU-T. (This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem.) Bits set to 1 indicate data signalling rates supported
                and enabled in both transmitter and receiver of modem. */
    mph->signalling_rate_mask = bitstream_get(&bs, &t, 15);
    /* 50       Enables asymmetric control channel data rates:
                0 = Asymmetric mode not allowed; 1 = Asymmetric mode allowed.
                Asymmetric mode shall be used only when both modems set bit 50 to 1. If different data rates are selected
                in symmetric mode, both modems shall transmit at the lower rate. */
    mph->asymmetric_rates_allowed = bitstream_get(&bs, &t, 1);
    if (mph->type == 1)
    {
        /* 51       Start bit: 0. */
        /* 52:67    Precoding coefficient h(1) real. */
        /* 68       Start bit: 0. */
        /* 69:84    Precoding coefficient h(1) imaginary. */
        /* 85       Start bit: 0. */
        /* 86:101   Precoding coefficient h(2) real. */
        /* 102      Start bit: 0. */
        /* 103:118  Precoding coefficient h(2) imaginary. */
        /* 119      Start bit: 0. */
        /* 120:135  Precoding coefficient h(3) real. */
        /* 136      Start bit: 0. */
        /* 137:152  Precoding coefficient h(3) imaginary. */
        for (i = 0;  i < 3;  i++)
        {
            bitstream_get(&bs, &t, 1);
            mph->precoder_coeffs[i].re = bitstream_get(&bs, &t, 16);
            bitstream_get(&bs, &t, 1);
            mph->precoder_coeffs[i].im = bitstream_get(&bs, &t, 16);
        }
        /*endfor*/
    }
    else
    {
        for (i = 0;  i < 3;  i++)
        {
            mph->precoder_coeffs[i].re = 0;
            mph->precoder_coeffs[i].im = 0;
        }
        /*endfor*/
    }
    /*endif*/
    /* We can ignore the remaining bits. They are not used. */
    log_mph(s->logging, false, mph);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void info_unpack_bits(uint8_t bits[], int nbits, const uint8_t buf[])
{
    int i;

    for (i = 0;  i < nbits;  i++)
    {
        uint8_t octet;

        octet = bit_reverse8(buf[i >> 3]);
        bits[i] = (octet >> (7 - (i & 7))) & 1;
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void info_pack_bits(uint8_t buf[25], const uint8_t bits[], int nbits)
{
    int i;

    memset(buf, 0, 25);
    for (i = 0;  i < nbits;  i++)
    {
        if (bits[i])
            buf[i >> 3] |= (1 << (i & 7));
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static uint16_t info_crc_from_bits(const uint8_t bits[], int nbits)
{
    uint16_t crc;
    int i;

    crc = 0xFFFF;
    for (i = 0;  i < nbits;  i++)
        crc = crc_itu16_bits(bits[i], 1, crc);
    /*endfor*/
    return crc;
}
/*- End of function --------------------------------------------------------*/

static void info_bits_to_str(char out[], size_t out_len, const uint8_t bits[], int nbits)
{
    int i;
    size_t pos;

    if (out_len == 0)
        return;
    /*endif*/
    pos = 0;
    for (i = 0;  i < nbits  &&  pos + 1 < out_len;  i++)
        out[pos++] = bits[i] ? '1' : '0';
    /*endfor*/
    out[pos] = '\0';
}
/*- End of function --------------------------------------------------------*/

static void info_log_candidate_diag(v34_rx_state_t *s, const uint8_t in[25], int nbits, uint16_t crc)
{
    uint8_t bits[80];
    char prefix[33];
    char suffix[33];
    char full_bits[81];
    uint16_t shift_crc[5];
    int shift;
    int i;

    if (nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return;
    /*endif*/

    info_unpack_bits(bits, nbits, in);
    memset(shift_crc, 0, sizeof(shift_crc));
    for (shift = -2;  shift <= 2;  shift++)
    {
        uint8_t trial[80];

        if (shift == 0)
        {
            shift_crc[shift + 2] = crc;
            continue;
        }
        /*endif*/
        for (i = 0;  i < nbits;  i++)
        {
            int src;

            src = i + shift;
            trial[i] = (src >= 0  &&  src < nbits)  ?  bits[src]  :  0;
        }
        /*endfor*/
        shift_crc[shift + 2] = info_crc_from_bits(trial, nbits);
    }
    /*endfor*/

    frame_bits_to_str(bits, 0, (nbits < 32) ? nbits : 32, prefix);
    if (nbits > 32)
        frame_bits_to_str(bits, nbits - 32, 32, suffix);
    else
        suffix[0] = '\0';
    /*endif*/
    info_bits_to_str(full_bits, sizeof(full_bits), bits, nbits);

    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
             "Rx - info candidate diag: stage=%s bits=%d crc=0x%04x sig=%d pwr=%" PRId32 " peak=%" PRId32 " bit_count=%d duration=%d\n",
             v34_rx_stage_to_str(s->stage),
             nbits,
             crc,
             s->signal_present,
             s->last_info_rx_power,
             s->last_info_rx_power_peak,
             s->bit_count,
             s->duration);
    if (nbits <= 64)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - info candidate bits=%s\n",
                 full_bits);
    }
    else
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - info candidate prefix=%s suffix=%s\n",
                 prefix,
                 suffix);
    }
    /*endif*/
    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
             "Rx - info candidate shift CRCs: -2=0x%04x -1=0x%04x 0=0x%04x +1=0x%04x +2=0x%04x\n",
             shift_crc[0], shift_crc[1], shift_crc[2], shift_crc[3], shift_crc[4]);
}
/*- End of function --------------------------------------------------------*/

static bool try_info_boundary_recovery(uint8_t out[25], const uint8_t in[25], int nbits, int *shift_out, uint16_t *crc_out)
{
    uint8_t bits[80];
    uint8_t trial[80];
    int shift;
    int i;

    if (nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return false;
    /*endif*/
    info_unpack_bits(bits, nbits, in);
    for (shift = -2;  shift <= 2;  shift++)
    {
        uint16_t crc;

        if (shift == 0)
            continue;
        /*endif*/
        for (i = 0;  i < nbits;  i++)
        {
            int src;

            src = i + shift;
            trial[i] = (src >= 0  &&  src < nbits)  ?  bits[src]  :  0;
        }
        /*endfor*/
        crc = info_crc_from_bits(trial, nbits);
        if (crc == 0)
        {
            info_pack_bits(out, trial, nbits);
            if (shift_out)
                *shift_out = shift;
            /*endif*/
            if (crc_out)
                *crc_out = crc;
            /*endif*/
            return true;
        }
        /*endif*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static bool try_info_local_slip_recovery(uint8_t out[25], const uint8_t in[25], int nbits, int *pivot_out, int *shift_out)
{
    uint8_t bits[80];
    uint8_t trial[80];
    int pivot;
    int shift;
    int i;

    if (nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return false;
    /*endif*/
    info_unpack_bits(bits, nbits, in);
    for (pivot = 4;  pivot < nbits - 4;  pivot++)
    {
        for (shift = -1;  shift <= 1;  shift += 2)
        {
            uint16_t crc;

            for (i = 0;  i < nbits;  i++)
            {
                int src;

                if (i < pivot)
                {
                    src = i;
                }
                else
                {
                    src = i + shift;
                }
                /*endif*/
                trial[i] = (src >= 0  &&  src < nbits)  ?  bits[src]  :  0;
            }
            /*endfor*/
            crc = info_crc_from_bits(trial, nbits);
            if (crc == 0)
            {
                info_pack_bits(out, trial, nbits);
                if (pivot_out)
                    *pivot_out = pivot;
                /*endif*/
                if (shift_out)
                    *shift_out = shift;
                /*endif*/
                return true;
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static bool info_has_valid_prefix_crc(const uint8_t in[25], int total_bits, int prefix_bits, uint16_t *prefix_crc_out)
{
    uint8_t bits[80];
    uint16_t prefix_crc;

    if (total_bits > (int) (sizeof(bits)/sizeof(bits[0])) || prefix_bits > total_bits)
        return false;
    /*endif*/
    info_unpack_bits(bits, total_bits, in);
    prefix_crc = info_crc_from_bits(bits, prefix_bits);
    if (prefix_crc_out)
        *prefix_crc_out = prefix_crc;
    /*endif*/
    return prefix_crc == 0;
}
/*- End of function --------------------------------------------------------*/

static int put_info_bit_count = 0;

/* V34_RX_STAGE_TONE_A's bit-persistence counters (below) drive Tone A /
   reversal detection purely off bit polarity, gated only by the generic
   signal_present hysteresis — which is tuned to catch weak-but-real
   carriers and so also passes ordinary line noise. On a real analog line
   with an old/noisy modem, background noise sits a consistent order of
   magnitude below a genuine Tone A carrier (observed ~4-5M vs ~14-25M in
   this codebase's own power units on a noisy line), but is still loud
   enough to occasionally produce a run of same-polarity bit decisions by
   chance, falsely tripping "Tone A detected"/reversal events. This
   threshold requires real carrier-level power, not just "louder than the
   off/on hysteresis", before letting persistence2 accumulate at all. */
/* How far below the measured carrier reference still counts as a real
   carrier.  The original absolute gate sat at 13000000 against a peer whose
   real carrier measured ~185000000 -- a ratio of ~14 -- so 8 reproduces that
   intent while being slightly more permissive. */
#define TONE_A_CARRIER_DIVISOR          8       /* ~9 dB below the carrier reference */

/* Is info_rx() currently looking at a real carrier rather than line noise?
 *
 * This used to be an absolute power threshold (13000000), calibrated against
 * one hardware modem.  That cannot generalise: the received level depends on
 * the peer's transmit power, the FXS gateway, and every digital pad in the
 * SIP path, none of which we control.  Worse, it was calibrated to within
 * about 1 dB of the signal it had to pass, so it was never a threshold so
 * much as a coin toss.  Measured on the d-modem rig, peak received power over
 * a 0.5 s window was 16411569 on 2026-07-17 (clears 13000000 by 1.0 dB) and
 * 12664417 on 2026-07-20 (misses it by 0.1 dB) -- ordinary run-to-run
 * variation of about 2 dB, no protocol change at all.
 *
 * On the losing side of that coin toss the failure is total, not degraded:
 * persistence2 is reset on every sample, Tone A is never declared, the third
 * reversal never arrives, and the receiver never advances to
 * V34_RX_STAGE_INFO1A.  The peer sends INFO1a perfectly well; we are
 * structurally unable to go and listen for it, and every call dies at
 * "aborting after 6 INFO1a timeouts".
 *
 * Gate relative to a measured carrier reference instead, so the same relative
 * discrimination applies at any absolute level. */
static int tone_a_carrier_present(v34_rx_state_t *s)
{
    int32_t reference;

    /* V.90 digital modem hearing the analogue modem's Tone A.  On a call we
       originate, V.8 completes on our side (JM received, CJ and 75 ms of
       silence sent) while the peer is still sending JM, which it stops only
       on detecting CJ -- measured on the RasFinder, the receiver starts
       ~180 ms before JM ends.  JM's steady V.21 tone reads here as Tone A and
       its FSK shifts as phase reversals, so the reversal fallback fired
       before INFO0a had been sent, and the INFO0a search was off by the time
       it arrived (artifacts/rasfinder-info0a-200334Z).  The level test below
       cannot see that: JM is as loud as Tone A.  The spectrum can -- Tone A
       is a 2400 Hz line. */
    if (s->v90_mode
        &&  !s->calling_party
        &&  s->tone_a_bin_frac_valid
        &&  s->tone_a_bin_frac < 0.15f)
    {
        return false;
    }
    /*endif*/
    reference = s->info_rx_carrier_ref;
    if (reference <= 0)
    {
        /* Nothing measured yet (first window of the call).  Fail open -- the
           coarse signal_present hysteresis still applies, and refusing
           everything here is exactly the failure mode being fixed. */
        return true;
    }
    /*endif*/
    return s->last_info_rx_power >= reference/TONE_A_CARRIER_DIVISOR;
}
/*- End of function --------------------------------------------------------*/

static int info0_target_bits(v34_rx_state_t *s)
{
    /* Payload+CRC length of an INFO0 sequence, excluding the sync code, the
       fill bits and the postamble.  Matches the value v34_rx_restart() picks
       when the receiver is first conditioned for INFO0.

       INFO0 is 49 bits (Table 7, bits 0:48) in BOTH modes: V.34 10.2.2 says
       the Phase 2 signals for half-duplex "are identical to those specified in
       10.1.2, except that INFO1a and INFO1c are replaced by INFOh".  INFO0 is
       not replaced.  Returning INFOh's 51 bits here made the receiver read two
       bits too many, so the INFO0 CRC failed on a bit-exact loopback and the
       single-bit recovery below "fixed" it by flipping bit 33 -- the first bit
       past the true 33-bit payload -- on every call. */
    (void) s;
    return 49 - (4 + 8 + 4);
}
/*- End of function --------------------------------------------------------*/

static void put_info_bit(v34_rx_state_t *s, int bit, int time_offset)
{
    int info_search_enabled;

    /* Put info0, info1, tone A or tone B bits */
    s->bitstream = (s->bitstream << 1) | bit;
    if (++put_info_bit_count % 600 == 0)
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - info_rx bits=%d bitstream=0x%03x stage=%d\n",
                 put_info_bit_count, (int)(s->bitstream & 0xFFF), s->stage);
    /* Log only sync code matches and CRC results (verbose bit logging removed) */
    switch (s->stage)
    {
    case V34_RX_STAGE_TONE_A:
        /* Calling side */
        if (++s->persistence1 < 10)
            break;
        /*endif*/
        if (!tone_a_carrier_present(s))
        {
            /* Not a real carrier — just line noise clearing the coarser
               signal_present hysteresis. Don't let it accumulate toward a
               false Tone A / reversal declaration. */
            s->persistence2 = 0;
            break;
        }
        /*endif*/
        if (bit == 0)
        {
            s->v90_infomarksa_run = 0;
            if (++s->persistence2 == 20)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - Tone A detected\n");
                if (!s->duplex)
                    s->hdx_tone_a_present = true;
                /* Only set TONE_SEEN if we haven't already seen a reversal —
                   otherwise we'd overwrite REVERSAL_1 and the next reversal
                   would be misidentified as the first instead of the second. */
                if (s->received_event != V34_EVENT_REVERSAL_1
                    && s->received_event != V34_EVENT_REVERSAL_2
                    && s->received_event != V34_EVENT_L2_SEEN)
                {
                    s->received_event = V34_EVENT_TONE_SEEN;
                }
            }
            /*endif*/
            break;
        }
        /*endif*/
        if (!s->signal_present)
            s->persistence2 = 0;
        /*endif*/
        /* A *sustained* run of ones is INFOMARKSa (V.34 10.1.2.3.6: binary
           ones on the DPSK modulator, so a phase reversal every baud), as
           opposed to the isolated reversal handled below.  V.90 9.2.1.2.6
           needs the two told apart: after the INFO1a deadline, INFOMARKSa
           means re-send INFO1d and continue per 9.2.1.1.8, while Tone A means
           the peer is retraining and we answer per 9.5.1.2.
           The threshold matches the 20 bauds the Tone A detector above uses,
           and is well past the 1-2 ones an isolated reversal produces. */
        if (s->v90_infomarksa_run < 1000000)
            s->v90_infomarksa_run++;
        /*endif*/
        if (s->v90_infomarksa_run == 20)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - INFOMARKSa detected (%d consecutive ones)\n",
                     s->v90_infomarksa_run);
            if (s->received_event == V34_EVENT_NONE
                ||  s->received_event == V34_EVENT_TONE_SEEN)
            {
                s->received_event = V34_EVENT_INFOMARKSA_SEEN;
            }
            /*endif*/
        }
        /*endif*/
        /* We have a reversal, but we should only recognise it if it has been
           a little while since the last one.

           Progress is counted in phase2_reversal_count, which only this
           detector writes. It used to be inferred from received_event, but
           v34tx.c clears that field in 39 places as it consumes events, so a
           reversal sequence could silently reset between reversals: live
           captures show runs with 10 "reversal 1" and 2 "reversal 2" that
           never reach the third and therefore never advance to INFO1A, which
           is the "aborting after 6 INFO1a timeouts" failure. received_event is
           still published for consumers -- it is just no longer the memory. */
        if (s->persistence2 > 20)
        {
            /* L2 having been seen means Phase 2 is already past the first two
               reversals, whatever the counter says. */
            if (s->received_event == V34_EVENT_L2_SEEN  &&  s->phase2_reversal_count < 2)
                s->phase2_reversal_count = 2;
            /*endif*/
            if (s->phase2_reversal_count < 3)
                s->phase2_reversal_count++;
            /*endif*/
            s->tone_ab_hop_time = s->sample_time + time_offset;
            switch (s->phase2_reversal_count)
            {
            case 1:
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - reversal 1 in tone A\n");
                s->received_event = V34_EVENT_REVERSAL_1;
                if (!s->duplex && s->half_duplex_source == V34_HALF_DUPLEX_RECIPIENT)
                    l1_l2_analysis_init(s); /* 12.2.2.1.4: first A reversal */
                break;
            case 2:
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - reversal 2 in tone A\n");
                s->received_event = V34_EVENT_REVERSAL_2;
                if (s->duplex || s->half_duplex_source == V34_HALF_DUPLEX_RECIPIENT)
                    l1_l2_analysis_init(s);
                if (s->v90_mode  &&  s->calling_party)
                {
                    /* 9.2.1.1.7: INFO1d follows the digital modem's L2 with no
                       reversal, and L2 may be far shorter than this receiver's
                       fixed L1/L2 analysis window.  info_rx() keeps framing
                       during L1/L2, so frame at INFO1d's length from here. */
                    s->target_bits = 109 - (4 + 8 + 4);
                    s->bit_count = 0;
                }
                break;
            default:
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - reversal 3 in tone A\n");
                s->received_event = V34_EVENT_REVERSAL_3;
                if (s->v90_mode && s->calling_party)
                {
                    /* V.90 caller expects INFO1d (109 bits, same format as INFO1c)
                       from the digital answerer. */
                    s->target_bits = 109 - (4 + 8 + 4);
                    s->stage = V34_RX_STAGE_INFO1C;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 caller: expecting INFO1d (109 bits) from digital answerer\n");
                }
                else if (!s->duplex)
                {
                    /* V.34 10.2.2: in half-duplex operation INFO1a and INFO1c
                       are replaced by INFOh, which Table 22 defines as 51 bits.
                       V34_RX_STAGE_INFOH was never assigned anywhere in the
                       tree, so the receiver could not enter the stage that
                       decodes it and INFOh was transmitted and never received.
                       12.2.1.1.4 / 12.2.2.2.4: condition the receiver for
                       INFOh at this point. */
                    s->target_bits = 51 - (4 + 8 + 4);
                    s->stage = V34_RX_STAGE_INFOH;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - half-duplex: expecting INFOh (51 bits)\n");
                }
                else
                {
                    /* Standard V.34: next info message will be INFO1a */
                    s->target_bits = 70 - (4 + 8 + 4);
                    s->stage = V34_RX_STAGE_INFO1A;
                }
                /*endif*/
                break;
            }
            /*endswitch*/
            s->persistence1 = 0;
        }
        /*endif*/
        s->persistence2 = 0;
        break;
    case V34_RX_STAGE_TONE_B:
        /* Answering side */
        if (!s->signal_present  &&  s->tone_b_present)
        {
            /* Tone B has stopped.  11.2.1.1.3 has the call modem transmit
               silence from its Tone B reversal until it has received L1 and
               L2, so this falling edge says it is ready for the probe -- and
               is worth as much to the answer modem as the onset. */
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - Tone B ended\n");
            s->tone_b_present = false;
            s->tone_b_ended = true;
        }
        /*endif*/
        if (++s->persistence1 < 10)
            break;
        /*endif*/
        if (bit == 0)
        {
            if (++s->persistence2 == 20)
            {
                /* V.90 §9.5.2 resumes after INFO0, so the analogue modem must
                   publish sustained Tone B to FIRST_A.  Plain V.34 retains
                   its historical INFO0-driven shortcut. */
                if (s->v90_mode && s->calling_party && s->info0_received)
                    s->received_event = V34_EVENT_TONE_SEEN;
                /*endif*/
                /* 11.2.1.2.6's "when Tone B is detected" is a separate fact
                   from the reversal ordinal in received_event, so it gets its
                   own flag rather than an event: the answer modem needs both
                   at once, and writing TONE_SEEN here would make the next
                   reversal read as the first.  The carrier gate is the same
                   one Tone A uses -- 20 same-polarity bits of line noise is
                   otherwise enough to declare a tone that is not there. */
                if (s->signal_present  &&  tone_a_carrier_present(s)
                    &&
                    !s->tone_b_present)
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Tone B detected (power=%d ref=%d)\n",
                             s->last_info_rx_power,
                             s->info_rx_carrier_ref);
                    s->tone_b_present = true;
                    s->tone_b_ended = false;
                }
                /*endif*/
            }
            /*endif*/
            break;
        }
        /*endif*/
        if (!s->signal_present)
            s->persistence2 = 0;
        /*endif*/
        /* We have a reversal, but we should only recognise it if it has been
           a little while since the last one */
        if (s->persistence2 > 20)
        {
            switch (s->received_event)
            {
            case V34_EVENT_REVERSAL_2:
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - reversal 3 in tone B\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_3;
                break;
            case V34_EVENT_REVERSAL_1:
                /* The phase step here is only 11.2.1.1.6's second Tone B
                   reversal if the answer modem has itself sent its post-L2
                   Tone A and reversal (11.2.1.2.6): that reversal is what the
                   call modem is answering, 40 ms later, and 11.2.1.2.7 starts
                   the probe receiver off it.  Before then a step in Tone B is
                   the tone RESUMING after the call modem's silence (11.2.1.1.5
                   transmits Tone B again once it has received our L2) -- or, with
                   our own L2 echoing back, a phase jump in a tone that never
                   seemed to stop.  Taking it as the reversal opened the L1/L2
                   window up to 0.7 s ahead of a probe that the conformant call
                   modem sends only after our reversal, so the window closed on
                   Tone B and INFO1a was built from noise ("snr=-28.8 dB").
                   The old timer-driven call modem sent its probe at about the
                   same time as the resumption, which is the only reason this
                   was ever right. */
                if (!s->v90_mode  &&  s->duplex  &&  !s->calling_party)
                {
                    const v34_state_t *owner =
                        (const v34_state_t *) ((const char *) s - offsetof(v34_state_t, rx));

                    if (owner->tx.stage < V34_TX_STAGE_POST_L2_NOT_A)
                    {
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Tone B phase step before our post-L2 Tone A reversal "
                                 "(tx stage %d); the tone resuming, not 11.2.1.2.7's reversal\n",
                                 owner->tx.stage);
                        break;
                    }
                    /*endif*/
                }
                /*endif*/
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - reversal 2 in tone B\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_2;
                if (s->v90_mode)
                {
                    /* V.90 §8.2.3.2 Table 10: analog modem sends INFO1a (70 bits),
                       not INFO1c (109 bits) as in standard V.34 */
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 mode: expecting INFO1a (70 bits) from analog modem\n");
                    s->target_bits = 70 - (4 + 8 + 4);
                    s->stage = V34_RX_STAGE_INFO1A;
                }
                else
                {
                    /* The next info message will be INFO1c */
                    s->target_bits = 109 - (4 + 8 + 4);
                }
                l1_l2_analysis_init(s);
                break;
            default:
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - reversal 1 in tone B\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_1;
                break;
            }
            /*endswitch*/
            s->persistence1 = 0;
        }
        /*endif*/
        s->persistence2 = 0;
        break;
    }
    info_search_enabled = (s->stage == V34_RX_STAGE_INFO0
                           ||
                           s->stage == V34_RX_STAGE_INFOH
                           ||
                           s->stage == V34_RX_STAGE_INFO1A
                           ||
                           s->stage == V34_RX_STAGE_INFO1C
                           ||
                           /* V.90: analog modem may start sending INFO1a during L1/L2
                              analysis — keep searching for sync codes so we don't miss it */
                           (s->v90_mode
                            &&
                            s->stage == V34_RX_STAGE_L1_L2)
                           ||
                           /* V.90 answerer: once INFO1d has been sent, keep INFO1a
                              sync search active across all Tone A/B event states so
                              we can lock immediately after the 10 ms A-bar period. */
                           (s->v90_mode
                            && !s->calling_party
                            && s->v90_info1d_sent
                            && (s->stage == V34_RX_STAGE_TONE_A
                                || s->stage == V34_RX_STAGE_TONE_B
                                || s->stage == V34_RX_STAGE_L1_L2
                                || s->stage == V34_RX_STAGE_CC))
                           ||
                           /* V.90 analogue modem: while the INFO0d we hold is
                              unacknowledged, keep framing INFO0d in Tone A.
                              9.2.2.2.1 turns on seeing it *repeated*, and the
                              INFO0_OK the first one raised would otherwise
                              switch the search off until the tone arrives. */
                           (s->v90_mode
                            && s->calling_party
                            && s->info0_received
                            && !s->info0_acknowledgement
                            && s->stage == V34_RX_STAGE_TONE_A)
                           ||
                           ((s->stage == V34_RX_STAGE_TONE_A
                             ||
                             s->stage == V34_RX_STAGE_TONE_B)
                            &&
                            (s->received_event == V34_EVENT_NONE
                             ||
                             s->received_event == V34_EVENT_INFO0_BAD
                             ||
                             s->received_event == V34_EVENT_TONE_SEEN)));
    if (s->v90_mode
        && !s->calling_party
        && s->v90_info1d_sent
        && s->info0_received
        && info_search_enabled
        && s->target_bits != (70 - (4 + 8 + 4)))
    {
        /* Bias framing toward INFO1a in the post-INFO1d window, even when
           we are still traversing Tone A/B transitions. */
        s->target_bits = 70 - (4 + 8 + 4);
    }
    /* Search for INFO0, INFOh, INFO1a or INFO1c messages. */
    if (!info_search_enabled)
    {
        s->bit_count = 0;
    }
    else if (s->bit_count == 0)
    {
        /* Look for info message sync code.  10.1.2.3.1: four fill ones,
           then 01110010.  In V.90 only the LAST fill one is required:
           8.2.3.1 precedes every INFO sequence with a point at an arbitrary
           carrier phase, so the first fill bits are decided across the
           carrier onset, and on a real line they are not reliable -- the
           RasFinder's INFO0a reads `0011 01110010` to an independent
           block demodulator and `1101 01110010` to this one
           (artifacts/rasfinder-flow-192655Z), so the two-ones match below
           never fired and INFO0a was lost on every originated call.  The
           CRC, not the fill, is what rejects a false sync.  The digital
           modem still waiting for INFO0a takes the sync with no fill at all:
           when INFO0a follows a silent gap, "Signal up" resets the phase
           history and the last fill bit is the first decision after it
           (artifacts/rasfinder-info0a-200334Z reads `0 01110010`).  In that
           state the competing signal is Tone A, all zeros, which cannot
           produce 01110010. */
        if ((s->bitstream & 0x3FF) == 0x372
            ||  (s->v90_mode  &&  (s->bitstream & 0x1FF) == 0x172)
            ||  (s->v90_mode
                 &&  !s->calling_party
                 &&  !s->info0_received
                 &&  (s->stage == V34_RX_STAGE_INFO0  ||  s->stage == V34_RX_STAGE_TONE_A)
                 &&  (s->bitstream & 0xFF) == 0x72))
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - info sync code detected\n");
            s->crc = 0xFFFF;
            s->bit_count = 1;
        }
        /*endif*/
    }
    else
    {
        /* Every 8 bits save the resulting byte */
        if ((s->bit_count & 0x07) == 0)
            s->info_buf[(s->bit_count >> 3) - 1] = bit_reverse8(s->bitstream & 0xFF);
        /*endif*/
        s->crc = crc_itu16_bits(bit, 1, s->crc);
        /* V.34/11.2.2.1.1: while we sit in the INFO1c wait the call modem may
           instead be repeatedly sending INFO0c, which is shorter than INFO1c.
           Without this the repeats are read as malformed 93 bit frames (the
           first bytes match the genuine INFO0c, then we run off the end and
           the CRC always fails), the recovery is never noticed, and both
           sides wait for each other until the peer's train timeout. Test the
           CRC as we pass the INFO0 length so a repeat is recognised. */
        if (!s->v90_mode
            &&  !s->calling_party
            &&  s->stage == V34_RX_STAGE_INFO1C
            &&  s->bit_count == info0_target_bits(s)
            &&  s->crc == 0)
        {
            int tail = s->bit_count & 0x07;

            if (tail != 0)
                s->info_buf[(s->bit_count >> 3)] = bit_reverse8(s->bitstream & 0xFF);
            /*endif*/
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - repeated INFO0c during INFO1c wait (11.2.2.1.1 recovery)\n");
            process_rx_info0(s, s->info_buf);
            s->received_event = V34_EVENT_INFO0_OK;
            s->bit_count = 0;
            return;
        }
        /*endif*/
        if (s->bit_count++ == s->target_bits)
        {
            /* Flush any remaining bits in the last partial byte into info_buf.
               The byte-write at line 3248 only fires when bit_count is a multiple
               of 8, so the tail bits (target_bits % 8 != 0) are lost. Write the
               partial byte now so info_buf and CRC are consistent. */
            {
                int tail = s->target_bits & 0x07;
                if (tail != 0)
                    s->info_buf[(s->target_bits >> 3)] = bit_reverse8(s->bitstream & 0xFF);
            }
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - info CRC result 0x%x (target_bits=%d)\n", s->crc, s->target_bits);
            {
                int nbytes = (s->target_bits + 7) / 8;
                if (nbytes > 25) nbytes = 25;
                char hexbuf[80];
                int hoff = 0;
                for (int hh = 0; hh < nbytes; hh++)
                    hoff += snprintf(hexbuf + hoff, sizeof(hexbuf) - hoff, " %02x", s->info_buf[hh]);
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - info raw bytes:%s\n", hexbuf);
            }
            if (s->crc == 0)
            {
                int v90_info1a_search;

                v90_info1a_search = (s->v90_mode
                                     && !s->calling_party
                                     && s->target_bits == (70 - (4 + 8 + 4))
                                     && (s->stage == V34_RX_STAGE_TONE_A
                                         || s->stage == V34_RX_STAGE_TONE_B
                                         || s->stage == V34_RX_STAGE_L1_L2
                                         || s->stage == V34_RX_STAGE_CC
                                         || s->stage == V34_RX_STAGE_INFO1A));
                if (s->v90_mode
                    &&  s->calling_party
                    &&  s->stage == V34_RX_STAGE_L1_L2
                    &&  s->target_bits == (109 - (4 + 8 + 4)))
                {
                    /* 9.2.2.1.8: INFO1d, arriving before this receiver's L1/L2
                       analysis window has closed. */
                    process_rx_info1c(s, &s->info1c, s->info_buf);
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 caller: INFO1d received during L1/L2 analysis\n");
                    s->stage = V34_RX_STAGE_INFO1C;
                    s->phase2_reversal_count = 3;
                    s->received_event = V34_EVENT_INFO1_OK;
                }
                else switch (s->stage)
                {
                case V34_RX_STAGE_TONE_A:
                case V34_RX_STAGE_TONE_B:
                case V34_RX_STAGE_L1_L2:
                case V34_RX_STAGE_CC:
                case V34_RX_STAGE_INFO0:
                    if (v90_info1a_search)
                    {
                        process_rx_info1a(s, &s->info1a, s->info_buf);
                        if (s->v90_mode)
                        {
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - V.90: INFO1a received, switching to Phase 3 primary channel RX\n");
                            v90_enter_phase3_from_info1a(s);
                        }
                        /* v34_force_phase3_rx() has already consumed the
                           INFO1a transition on the TX side.  Do not leave a
                           stale INFO1_OK event blocking Phase 3 J detection. */
                        s->received_event = V34_EVENT_NONE;
                    }
                    else
                    {
                        int preserve_tone_a_recovery;
                        int first_info0;

                        preserve_tone_a_recovery = (s->v90_mode
                                                    && !s->calling_party
                                                    && s->stage == V34_RX_STAGE_TONE_A);
                        first_info0 = !s->info0_received;
                        process_rx_info0(s, s->info_buf);
                        if (preserve_tone_a_recovery)
                        {
                            /* V.90 answerer recovery: after receiving INFO0a and
                               conditioning RX for Tone A, keep looking for Tone A
                               instead of falling back to Tone B on every repeated
                               INFO0a.  A repeated INFO0a is not a new Phase 2
                               transaction: do not erase a Tone A reversal that
                               may have arrived in the same media block. */
                            s->stage = V34_RX_STAGE_TONE_A;
                            if (first_info0)
                                s->phase2_reversal_count = 0;
                            /*endif*/
                            s->persistence1 = 0;
                            s->persistence2 = 0;
                        }
                        else
                        {
                            s->stage = (s->calling_party)  ?   V34_RX_STAGE_TONE_A  :  V34_RX_STAGE_TONE_B;
                        }
                        /* Only set INFO0_OK on the first reception. Repeated
                           INFO0a decodes during V.90 FIRST_B_SILENCE would overwrite
                           REVERSAL_1 events from the tone handler, preventing the
                           TX from detecting the second Tone A reversal. Track them
                           via a sticky flag instead so the TX state machine can
                           choose whether to recover Phase 2. */
                        if (!s->info0_received)
                            s->received_event = V34_EVENT_INFO0_OK;
                        else if (s->v90_mode
                                 && !s->calling_party
                                 && (s->stage == V34_RX_STAGE_TONE_A
                                     || s->stage == V34_RX_STAGE_TONE_B
                                     || s->stage == V34_RX_STAGE_INFO0))
                            s->v90_repeated_info0a_pending = true;
                        else if (s->v90_mode
                                 && s->calling_party
                                 && !s->info0_acknowledgement
                                 && (s->stage == V34_RX_STAGE_TONE_A
                                     || s->stage == V34_RX_STAGE_INFO0))
                            s->v90_repeated_info0d_pending = true;
                        s->info0_received = true;
                    }
                    break;
                case V34_RX_STAGE_INFOH:
                    if (process_rx_infoh(s, &s->infoh, s->info_buf) != 0)
                        break;
                    /* V34_EVENT_INFOH_OK is what the half-duplex transmitter
                       waits on in V34_TX_STAGE_HDX_POST_L2_B (12.2.1.1.4).  It
                       was defined, named and waited on, but NEVER RAISED
                       anywhere in the tree, so the source modem could not leave
                       that stage and Phase 3 never started. */
                    s->received_event = V34_EVENT_INFOH_OK;
                    break;
                case V34_RX_STAGE_INFO1C:
                    process_rx_info1c(s, &s->info1c, s->info_buf);
                    s->received_event = V34_EVENT_INFO1_OK;
                    break;
                case V34_RX_STAGE_INFO1A:
                    if (process_rx_info1a(s, &s->info1a, s->info_buf) < 0)
                    {
                        /* Undefined selections: no INFO1_OK, no state change. */
                        s->received_event = V34_EVENT_NONE;
                        break;
                    }
                    /*endif*/
                    if (s->v90_mode)
                    {
                        /* V.90 §9.2.1.1.8: INFO1a received — now proceed to Phase 3.
                           Switch RX from CC demodulator to primary channel for
                           upstream V.34 reception. */
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - V.90: INFO1a received, switching to Phase 3 primary channel RX\n");
                        v90_enter_phase3_from_info1a(s);
                        s->received_event = V34_EVENT_NONE;
                    }
                    else
                    {
                        s->received_event = V34_EVENT_INFO1_OK;
                    }
                    break;
                }
                /*endswitch*/
            }
            else
            {
                int v90_info1a_search;
                uint8_t recovered_info[25];
                int recovery_shift;
                int recovery_pivot;
                uint16_t prefix_crc;

                v90_info1a_search = (s->v90_mode
                                     && !s->calling_party
                                     && s->target_bits == (70 - (4 + 8 + 4))
                                     && (s->stage == V34_RX_STAGE_TONE_A
                                         || s->stage == V34_RX_STAGE_TONE_B
                                         || s->stage == V34_RX_STAGE_L1_L2
                                         || s->stage == V34_RX_STAGE_CC
                                         || s->stage == V34_RX_STAGE_INFO1A));
                if (v90_info1a_search)
                    info_log_candidate_diag(s, s->info_buf, s->target_bits, s->crc);
                /*endif*/
                /* Try INFO1a boundary/slip recovery before the INFO0a prefix check.
                   Once INFO1d has been sent, the normal path is INFO1a, so
                   prioritise INFO1a decoding.  A peer in Phase 2 error recovery,
                   however, deliberately resumes INFO0a at this point to request an
                   acknowledged INFO0d.  Keep the valid-prefix fallback active after
                   INFO1d as well; otherwise the shorter INFO0a is padded with Tone A
                   bits to the INFO1a target length and reported forever as INFO1_BAD. */
                if (v90_info1a_search
                    && try_info_boundary_recovery(recovered_info, s->info_buf, s->target_bits, &recovery_shift, NULL))
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - INFO1a boundary recovery succeeded with %d-bit shift\n",
                             recovery_shift);
                    process_rx_info1a(s, &s->info1a, recovered_info);
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90: recovered INFO1a, switching to Phase 3 primary channel RX\n");
                    v90_enter_phase3_from_info1a(s);
                    s->received_event = V34_EVENT_NONE;
                    s->bit_count = 0;
                    return;
                }
                /*endif*/
                if (v90_info1a_search
                    && try_info_local_slip_recovery(recovered_info, s->info_buf, s->target_bits, &recovery_pivot, &recovery_shift))
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - INFO1a local-slip recovery succeeded at bit %d with suffix shift %d\n",
                             recovery_pivot,
                             recovery_shift);
                    process_rx_info1a(s, &s->info1a, recovered_info);
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90: recovered INFO1a via local-slip recovery, switching to Phase 3 primary channel RX\n");
                    v90_enter_phase3_from_info1a(s);
                    s->received_event = V34_EVENT_NONE;
                    s->bit_count = 0;
                    return;
                }
                /*endif*/
                if (v90_info1a_search
                    && info_has_valid_prefix_crc(s->info_buf, s->target_bits, 33, &prefix_crc))
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - INFO1a candidate contains a valid 33-bit INFO0a prefix (crc=0x%04x); treating it as repeated INFO0a, not INFO1a\n",
                             prefix_crc);
                    process_rx_info0(s, s->info_buf);
                    s->v90_repeated_info0a_pending = true;
                    s->received_event = V34_EVENT_INFO0_OK;
                    s->bit_count = 0;
                    return;
                }
                /*endif*/
                /* INFO0 single-bit-error recovery: if the CRC failed with only
                   1 bit wrong, try flipping each bit and recomputing.  This is
                   cheap (max 46 iterations for INFO0d) and dramatically improves
                   INFO0 decode success on recordings with minor DPSK bit errors. */
                if (s->stage == V34_RX_STAGE_INFO0
                    && !v90_info1a_search
                    && !s->info0_received)
                {
                    int nb = s->target_bits;
                    int nbytes_r = (nb + 7) / 8;
                    int recovered_info0 = 0;

                    for (int flip = 0;  flip < nb;  flip++)
                    {
                        uint16_t test_crc = 0xFFFF;
                        int byte_idx = flip >> 3;
                        int bit_idx = flip & 0x07;
                        uint8_t saved = s->info_buf[byte_idx];

                        s->info_buf[byte_idx] ^= (1 << bit_idx);
                        for (int bi = 0;  bi < nbytes_r;  bi++)
                        {
                            int bits_left = nb - bi * 8;
                            if (bits_left > 8)
                                bits_left = 8;
                            test_crc = crc_itu16_bits(s->info_buf[bi], bits_left, test_crc);
                        }
                        if (test_crc == 0)
                        {
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - INFO0 single-bit recovery: flipped bit %d, CRC now 0\n",
                                     flip);
                            process_rx_info0(s, s->info_buf);
                            if (!s->info0_received)
                                s->received_event = V34_EVENT_INFO0_OK;
                            s->info0_received = true;
                            /* Condition the receiver exactly as the ordinary
                               INFO0 path does.  This raised INFO0_OK and
                               returned without touching the stage, so a
                               recovered INFO0 left the receiver still hunting
                               INFO0 while the transmitter had already moved on:
                               V.34 11.2.1.1.2 / 12.2.1.1.2 require the receiver
                               to be conditioned for the far tone and its phase
                               reversal at this point, and it never was, so the
                               reversal was never reported. */
                            s->stage = (s->calling_party)
                                     ? V34_RX_STAGE_TONE_A
                                     : V34_RX_STAGE_TONE_B;
                            recovered_info0 = 1;
                            break;
                        }
                        s->info_buf[byte_idx] = saved;
                    }
                    if (recovered_info0)
                    {
                        s->bit_count = 0;
                        return;
                    }
                }
                /*endif*/
                switch (s->stage)
                {
                case V34_RX_STAGE_TONE_A:
                case V34_RX_STAGE_TONE_B:
                case V34_RX_STAGE_L1_L2:
                case V34_RX_STAGE_CC:
                case V34_RX_STAGE_INFO0:
                    s->received_event = v90_info1a_search ? V34_EVENT_INFO1_BAD : V34_EVENT_INFO0_BAD;
                case V34_RX_STAGE_INFOH:
                    break;
                case V34_RX_STAGE_INFO1C:
                case V34_RX_STAGE_INFO1A:
                    s->received_event = V34_EVENT_INFO1_BAD;
                    break;
                }
                /*endswitch*/
            }
            /*endif*/
            s->bit_count = 0;
        }
        /*endif*/
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

/* blip_duration advances 3 per sample, 40 per 600-baud bit.  Two genuine
   reversals are a whole bit apart, so a second one inside half a bit is the
   same reversal seen twice -- the phase is ill-defined as the envelope goes
   through zero, and a coarse quantiser (A-law's low-level steps are four
   times mu-law's) can make it jump by more than 90 degrees twice.  The old
   gate was 3 (two samples), which let that through as an extra one bit and
   slipped the rest of the frame: on our own V.90 Phase 2 over A-law the
   first INFO1d arrived with the right information and its CRC shifted by
   one bit, every repetition, costing ~1.5 s until the resend
   (v92_mh_retrain_test, 2026-10-03). */
#define V34_INFO_REVERSAL_REFRACTORY 20

static int info_rx(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;
    int step;
    complexf_t z;
    complexf_t zz;
    complexf_t sample;
    float ii;
    float qq;
    uint32_t angle;
    int32_t phase_delta;
    int32_t power;

    s->agc_scaling = 0.01f;
    step = 6;
    for (i = 0;  i < len;  i++)
    {
        /* Guard-tone / carrier ratio (V.34 10.1.2.1, 10.1.2.3).  Two Goertzel
           bins over 320 samples (40 ms, 25 Hz resolution -- the tones are
           600 Hz apart, so this is ample).  The ratio is the only reliable
           statement this receiver can make about whether the peer is holding
           Tone A or transmitting an INFO sequence: the spec fixes both levels,
           and the two states differ by ~7 dB. */
        {
            /* Goertzel coefficient is 2cos(2*pi*f/8000):
                 f=1800 -> 2cos(0.45*pi) =  0.31286893
                 f=2400 -> 2cos(0.60*pi) = -0.61803399
               and the sign is folded into the recurrences below. */
            float x = (float) amp[i];
            float g0;
            float c0;

            g0 = x + 0.31286893f*s->guard_g1 - s->guard_g2;
            s->guard_g2 = s->guard_g1;
            s->guard_g1 = g0;
            c0 = x - 0.61803399f*s->carrier_g1 - s->carrier_g2;
            s->carrier_g2 = s->carrier_g1;
            s->carrier_g1 = c0;
            s->guard_block_energy += x*x;
            /* The same 2400 Hz bin, non-coherently over 80-sample (10 ms)
               sub-blocks, so that a Tone A phase reversal part-way through
               the 40 ms block cannot cancel it. */
            c0 = x - 0.61803399f*s->tone_a_sub_g1 - s->tone_a_sub_g2;
            s->tone_a_sub_g2 = s->tone_a_sub_g1;
            s->tone_a_sub_g1 = c0;
            if ((s->guard_block_len + 1)%80 == 0)
            {
                s->tone_a_sub_pow += s->tone_a_sub_g1*s->tone_a_sub_g1
                                   + s->tone_a_sub_g2*s->tone_a_sub_g2
                                   + 0.61803399f*s->tone_a_sub_g1*s->tone_a_sub_g2;
                s->tone_a_sub_g1 = s->tone_a_sub_g2 = 0.0f;
            }
            /*endif*/
            if (++s->guard_block_len >= 320)
            {
                float gp = s->guard_g1*s->guard_g1 + s->guard_g2*s->guard_g2
                         - 0.31286893f*s->guard_g1*s->guard_g2;
                float cp = s->carrier_g1*s->carrier_g1 + s->carrier_g2*s->carrier_g2
                         + 0.61803399f*s->carrier_g1*s->carrier_g2;

                if (cp > 1.0f  &&  gp > 1.0f)
                {
                    s->guard_carrier_db = 10.0f*log10f(gp/cp);
                    s->guard_carrier_valid = 1;
                }
                else
                {
                    s->guard_carrier_valid = 0;
                }
                /*endif*/
                /* A pure tone of amplitude A gives each 80-sample Goertzel
                   a power of (40A)^2, four of them 6400A^2, against a block
                   energy of 160A^2: dividing by energy*40 normalises the bin
                   to its share of the block. */
                if (s->guard_block_energy > 320.0f*100.0f*100.0f)
                {
                    s->tone_a_bin_frac = s->tone_a_sub_pow/(s->guard_block_energy*40.0f);
                    s->tone_a_bin_frac_valid = 1;
                }
                else
                {
                    s->tone_a_bin_frac_valid = 0;
                }
                /*endif*/
                s->guard_block_energy = 0.0f;
                s->tone_a_sub_pow = 0.0f;
                s->guard_g1 = s->guard_g2 = 0.0f;
                s->carrier_g1 = s->carrier_g2 = 0.0f;
                s->guard_block_len = 0;
            }
            /*endif*/
        }
        power = power_meter_update(&s->power, amp[i]);
        s->last_info_rx_power = power;
        if (s->sample_time - s->last_info_rx_power_peak_reset >= 800)
        {
            s->last_info_rx_power_peak = power;
            s->last_info_rx_power_peak_reset = s->sample_time;
        }
        else if (power > s->last_info_rx_power_peak)
        {
            s->last_info_rx_power_peak = power;
        }
        /*endif*/
        /* Carrier-level reference feeding the Tone A gate: instant attack,
           slow decay (~4096-sample time constant, so roughly -17 dB/s with no
           signal).  Deliberately not windowed against s->sample_time -- the
           V.90 Phase 2 flow reconditions this receiver repeatedly, resetting
           that clock, so a 2 s window never completed and the reference stayed
           at 0.  A minimum tracker is wrong here too: Phase 2 contains genuine
           silent gaps, so a minimum collapses to ~0 and the gate degenerates
           to always-open. */
        if (power > s->info_rx_carrier_ref)
            s->info_rx_carrier_ref = power;
        else
            s->info_rx_carrier_ref -= s->info_rx_carrier_ref >> 12;
        /*endif*/
        if (s->v90_mode
            && !s->calling_party
            && !s->signal_present
            && (s->stage == V34_RX_STAGE_INFO0 || s->stage == V34_RX_STAGE_TONE_A || s->stage == V34_RX_STAGE_TONE_B)
            && (s->duration == 400 || s->duration == 800 || s->duration == 1600 || s->duration == 3200))
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - V.90 Phase 2 waiting for carrier: stage=%d power=%" PRId32 " on=%" PRId32 " off=%" PRId32 "\n",
                     s->stage, power, s->carrier_on_power, s->carrier_off_power);
        }
        if (s->signal_present)
        {
            if (power < s->carrier_off_power)
            {
V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Signal down\n");
                s->signal_present = false;
                s->persistence2 = 0;
            }
            /*endif*/
        }
        else
        {
            if (power > s->carrier_on_power)
            {
V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Signal up\n");
                s->signal_present = true;
                s->persistence2 = 0;
                /* Reset phase tracking so stale last_angles don't
                   cause spurious reversals when CC resumes */
                s->last_angles[0] = 0;
                s->last_angles[1] = 0;
                s->blip_duration = 0;
            }
            /*endif*/
        }
        /*endif*/
        s->rrc_filter[s->rrc_filter_step] = amp[i];
        if (++s->rrc_filter_step >= V34_RX_FILTER_STEPS)
            s->rrc_filter_step = 0;
        /*endif*/
        /* Standard V.34: caller RX at 2400 Hz, answerer RX at 1200 Hz.
           V.90 §8.2.3.1: carriers swapped — answerer RX at 2400 Hz, caller RX at 1200 Hz. */
        if (s->calling_party != s->v90_mode)
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        else
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        /*endif*/
        sample.re = ii*s->agc_scaling;
        sample.im = qq*s->agc_scaling;
        /* Shift to baseband - since this is done in full complex form, the result is clean. */
        z = dds_lookup_complexf(s->carrier_phase);
        zz.re = sample.re*z.re - sample.im*z.im;
        zz.im = -sample.re*z.im - sample.im*z.re;
        angle = arctan2(zz.im, zz.re);
        phase_delta = (int32_t) (angle - s->last_angles[1]);
        /* V34_DUMP_INFO_RX (diagnostic, 2026-07-19): dumps the info_rx
           demod chain -- pre-AGC matched-filter output, post-mix baseband,
           angle, and phase_delta -- to find why put_info_bit() never sees
           a reversal (bitstream stuck at 0x000) against a real remote
           modem despite signal_present staying true and the raw carrier
           measuring correct in an offline capture. */
        if (V34_DIAG_GETENV("V34_DUMP_INFO_RX")
            && s->stage == V34_RX_STAGE_INFO0 && (s->duration % 400) == 0)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - info_rx sample: t=%d ii=%.4f qq=%.4f agc=%.5f zz=(%.4f,%.4f) angdeg=%.2f pdeltadeg=%.2f blip=%d\n",
                     s->duration, (double) ii, (double) qq, (double) s->agc_scaling,
                     (double) zz.re, (double) zz.im,
                     (double) (angle * 360.0 / 4294967296.0),
                     (double) (phase_delta * 360.0 / 4294967296.0),
                     s->blip_duration);
        }
        if ((phase_delta > (int32_t) DDS_PHASE(90.0f)
             || phase_delta < -(int32_t) DDS_PHASE(90.0f))
            &&
            s->blip_duration > V34_INFO_REVERSAL_REFRACTORY)
        {
            /* Reversal event logging removed (was verbose) */
            put_info_bit(s, 1, i);
            s->duration = 0;
            s->blip_duration = 0;
        }
        else
        {
            if (s->blip_duration > 60)
            {
                /* We are getting rather late for a transition. This must be a zero bit. */
                put_info_bit(s, 0, i);
                /* Step on by one bit time. */
                s->blip_duration -= 40;
            }
            /*endif*/
        }
        /*endif*/
        s->last_angles[1] = s->last_angles[0];
        s->last_angles[0] = angle;
        s->duration++;
        s->blip_duration += 3;
        dds_advancef(&s->carrier_phase, s->cc_carrier_phase_rate);
    }
    /*endfor*/
    /* Periodic diagnostic: log every 8000 samples (~1s) */
    if (V34_DEBUG_INFO_RX_DIAG
        && s->duration % 8000 < (unsigned)len)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx info_rx diag: ref=%d gate=%d carrier=%d "
                 "stage=%d sig=%d pwr=%d bits=%d\n",
                 s->info_rx_carrier_ref,
                 s->info_rx_carrier_ref/TONE_A_CARRIER_DIVISOR,
                 tone_a_carrier_present(s),
                 s->stage, s->signal_present, power, s->bit_count);
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static __inline__ void cc_symbol_sync(v34_rx_state_t *s)
{
    int i;
#if defined(SPANDSP_USE_FIXED_POINT)
    int32_t v;
    int32_t p;
#else
    float v;
    float p;
    const float ted_error_clip = 50.0f;
    const float ted_fine_trigger = 100.0f;
    const float ted_coarse_trigger = 200.0f;
    const float ted_phase_clip = 500.0f;
#endif

    /* This routine adapts the position of the half baud samples entering the equalizer. */

    /* This symbol sync scheme is based on the technique first described by Dominique Godard in
        Passband Timing Recovery in an All-Digital Modem Receiver
        IEEE TRANSACTIONS ON COMMUNICATIONS, VOL. COM-26, NO. 5, MAY 1978 */

    /* This is slightly rearranged from figure 3b of the Godard paper, as this saves a couple of
       maths operations */
#if defined(SPANDSP_USE_FIXED_POINT)
    /* TODO: The scalings used here need more thorough evaluation, to see if overflows are possible. */
    /* Cross correlate */
    v = (((s->cc_ted.symbol_sync_low[1] >> 5)*(s->cc_ted.symbol_sync_high[0] >> 4)) >> 15)*s->cc_ted.low_band_edge_coeff[2]
      - (((s->cc_ted.symbol_sync_low[0] >> 5)*(s->cc_ted.symbol_sync_high[1] >> 4)) >> 15)*s->cc_ted.high_band_edge_coeff[2]
      + (((s->cc_ted.symbol_sync_low[1] >> 5)*(s->cc_ted.symbol_sync_high[1] >> 4)) >> 15)*s->cc_ted.mixed_edges_coeff_3;
    /* Filter away any DC component */
    p = v - s->cc_ted.symbol_sync_dc_filter[1];
    s->cc_ted.symbol_sync_dc_filter[1] = s->cc_ted.symbol_sync_dc_filter[0];
    s->cc_ted.symbol_sync_dc_filter[0] = v;
    /* Reduce data-directed jitter for the multilevel control constellation. */
    if (s->mp_seen >= 2 && s->hdx_control_rx_rate == 2400)
        p *= 0.01f;
    /* A little integration will now filter away much of the HF noise */
    s->cc_ted.baud_phase -= p;
    v = abs(s->cc_ted.baud_phase);
    if (v > 100*FP_FACTOR)
    {
        i = (v > 1000*FP_FACTOR)  ?  15  :  1;
        if (s->cc_ted.baud_phase < 0)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d %d\n", v, i, p, s->cc_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
    }
    /*endif*/
#else
    /* Cross correlate */
    v = s->cc_ted.symbol_sync_low[1]*s->cc_ted.symbol_sync_high[0]*s->cc_ted.low_band_edge_coeff[2]
      - s->cc_ted.symbol_sync_low[0]*s->cc_ted.symbol_sync_high[1]*s->cc_ted.high_band_edge_coeff[2]
      + s->cc_ted.symbol_sync_low[1]*s->cc_ted.symbol_sync_high[1]*s->cc_ted.mixed_edges_coeff_3;
    /* Filter away any DC component  */
    p = v - s->cc_ted.symbol_sync_dc_filter[1];
    if (!isfinite(p))
        p = 0.0f;
    else if (p > ted_error_clip)
        p = ted_error_clip;
    else if (p < -ted_error_clip)
        p = -ted_error_clip;
    s->cc_ted.symbol_sync_dc_filter[1] = s->cc_ted.symbol_sync_dc_filter[0];
    s->cc_ted.symbol_sync_dc_filter[0] = v;
    /* Reduce data-directed jitter for the multilevel control constellation. */
    if (s->mp_seen >= 2 && s->hdx_control_rx_rate == 2400)
        p *= 0.01f;
    /* A little integration will now filter away much of the HF noise */
    s->cc_ted.baud_phase -= p;
    if (!isfinite(s->cc_ted.baud_phase))
        s->cc_ted.baud_phase = 0.0f;
    else if (s->cc_ted.baud_phase > ted_phase_clip)
        s->cc_ted.baud_phase = ted_phase_clip;
    else if (s->cc_ted.baud_phase < -ted_phase_clip)
        s->cc_ted.baud_phase = -ted_phase_clip;
    v = fabsf(s->cc_ted.baud_phase);
    if (v > ted_fine_trigger)
    {
        i = (v > ted_coarse_trigger)  ?  2  :  1;
        if (s->cc_ted.baud_phase < 0.0f)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d\n", v, i, p, s->cc_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
        /* Anti-windup: consume phase error when applying a timing step. */
        s->cc_ted.baud_phase -= i*ted_fine_trigger;
    }
    /*endif*/
#endif
}
/*- End of function --------------------------------------------------------*/

/* ME_V34_DATA_TIMING_DIAG_FREEZE=1 holds the Godard symbol timing loop for
   the whole of data mode.  Diagnostic only: it is how the 2026-10-02 work
   showed the 32 dB data-mode floor at 31200 was not timing jitter. */
static int v34_data_timing_diag_freeze(void)
{
    static int cache = -1;

    if (cache < 0)
        cache = (getenv("ME_V34_DATA_TIMING_DIAG_FREEZE") != NULL);
    /*endif*/
    return cache;
}
/*- End of function --------------------------------------------------------*/

static __inline__ void pri_symbol_sync(v34_rx_state_t *s)
{
    int i;
#if defined(SPANDSP_USE_FIXED_POINT)
    int32_t v;
    int32_t p;
#else
    float v;
    float p;
    const float ted_error_clip = 50.0f;
    const float ted_fine_trigger = 100.0f;
    const float ted_coarse_trigger = 200.0f;
    const float ted_phase_clip = 500.0f;
#endif

    if (s->stage == V34_RX_STAGE_DATA
        && v34_rx_data_mm_timing_enabled(s))
        return;

    /* This routine adapts the position of the half baud samples entering the equalizer. */

    /* This symbol sync scheme is based on the technique first described by Dominique Godard in
        Passband Timing Recovery in an All-Digital Modem Receiver
        IEEE TRANSACTIONS ON COMMUNICATIONS, VOL. COM-26, NO. 5, MAY 1978 */

    /* This is slightly rearranged from figure 3b of the Godard paper, as this saves a couple of
       maths operations */
#if defined(SPANDSP_USE_FIXED_POINT)
    /* TODO: The scalings used here need more thorough evaluation, to see if overflows are possible. */
    /* Cross correlate */
    v = (((s->pri_ted.symbol_sync_low[1] >> 5)*(s->pri_ted.symbol_sync_high[0] >> 4)) >> 15)*s->pri_ted.low_band_edge_coeff[2]
      - (((s->pri_ted.symbol_sync_low[0] >> 5)*(s->pri_ted.symbol_sync_high[1] >> 4)) >> 15)*s->pri_ted.high_band_edge_coeff[2]
      + (((s->pri_ted.symbol_sync_low[1] >> 5)*(s->pri_ted.symbol_sync_high[1] >> 4)) >> 15)*s->pri_ted.mixed_edges_coeff_3;
    /* Filter away any DC component */
    p = v - s->pri_ted.symbol_sync_dc_filter[1];
    s->pri_ted.symbol_sync_dc_filter[1] = s->pri_ted.symbol_sync_dc_filter[0];
    s->pri_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->pri_ted.baud_phase -= p;
    v = abs(s->pri_ted.baud_phase);
    if (v > 100*FP_FACTOR)
    {
        i = (v > 1000*FP_FACTOR)  ?  15  :  1;
        if (s->pri_ted.baud_phase < 0)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d %d\n", v, i, p, s->pri_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
    }
    /*endif*/
#else
    /* Cross correlate */
    v = s->pri_ted.symbol_sync_low[1]*s->pri_ted.symbol_sync_high[0]*s->pri_ted.low_band_edge_coeff[2]
      - s->pri_ted.symbol_sync_low[0]*s->pri_ted.symbol_sync_high[1]*s->pri_ted.high_band_edge_coeff[2]
      + s->pri_ted.symbol_sync_low[1]*s->pri_ted.symbol_sync_high[1]*s->pri_ted.mixed_edges_coeff_3;
    /* Filter away any DC component  */
    p = v - s->pri_ted.symbol_sync_dc_filter[1];
    if (!isfinite(p))
        p = 0.0f;
    else if (p > ted_error_clip)
        p = ted_error_clip;
    else if (p < -ted_error_clip)
        p = -ted_error_clip;
    s->pri_ted.symbol_sync_dc_filter[1] = s->pri_ted.symbol_sync_dc_filter[0];
    s->pri_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->pri_ted.baud_phase -= p;
    if (!isfinite(s->pri_ted.baud_phase))
        s->pri_ted.baud_phase = 0.0f;
    else if (s->pri_ted.baud_phase > ted_phase_clip)
        s->pri_ted.baud_phase = ted_phase_clip;
    else if (s->pri_ted.baud_phase < -ted_phase_clip)
        s->pri_ted.baud_phase = -ted_phase_clip;
    v = fabsf(s->pri_ted.baud_phase);
    if (v > ted_fine_trigger)
    {
        i = (v > ted_coarse_trigger)  ?  2  :  1;
        if (s->pri_ted.baud_phase < 0.0f)
            i = -i;
        /*endif*/
        /* This loop is NOT frozen during Phase 4 MP the way carrier tracking
           is (phase4_trn_should_freeze_tracking()).  Freezing it there was
           tested live on 2026-07-19 because it fires one large correction at
           the TRN->MP boundary: the decoded MP bits came out byte-for-byte
           identical, so it is not why MP frames failed CRC.  See
           rig/README.md for the fuller elimination list. */
        //printf("v = %10.5f %5d - %f %f %d\n", v, i, p, s->pri_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
        /* Anti-windup: consume phase error when applying a timing step. */
        s->pri_ted.baud_phase -= i*ted_fine_trigger;
    }
    /*endif*/
#endif
    /* Periodic TED + carrier tracking diagnostic (every 256 bauds) */
    if (V34_TRACE_DIAGNOSTICS
        && (s->duration & 0xFF) == 0
        && s->stage >= V34_RX_STAGE_PHASE3_WAIT_S)
    {
        fprintf(stderr, "[V34 RX] %s baud=%d ted_phase=%.1f ted_corr=%d carrier=%.2fHz eq_step=%d stage=%s\n",
                s->calling_party ? "caller" : "answer",
                s->duration, (double)s->pri_ted.baud_phase, s->total_baud_timing_correction,
                dds_frequencyf(s->v34_carrier_phase_rate), s->eq_put_step,
                v34_rx_stage_to_str(s->stage));
    }
}
/*- End of function --------------------------------------------------------*/

static void create_godard_coeffs(ted_t *coeffs, float carrier, float baud_rate, float alpha)
{
    float low_edge;
    float high_edge;

    /* Create the coefficient set for an arbitrary Godard TED/symbol sync filter */
    low_edge = 2.0*M_PI*(carrier - baud_rate/2.0)/SAMPLE_RATE;
    high_edge = 2.0*M_PI*(carrier + baud_rate/2.0)/SAMPLE_RATE;

#if defined(SPANDSP_USE_FIXED_POINT)
    coeffs->low_band_edge_coeff[0] = ((int32_t)(FP_FACTOR*(2.0*alpha*cos(low_edge))));
    coeffs->high_band_edge_coeff[0] = ((int32_t)(FP_FACTOR*(2.0*alpha*cos(high_edge))));
    coeffs->low_band_edge_coeff[1] =
    coeffs->high_band_edge_coeff[1] = ((int32_t)(FP_FACTOR*(-alpha*alpha)));
    coeffs->low_band_edge_coeff[2] = ((int32_t)(FP_FACTOR*(-alpha*sin(low_edge))));
    coeffs->high_band_edge_coeff[2] = ((int32_t)(FP_FACTOR*(-alpha*sin(high_edge))));
    coeffs->mixed_edges_coeff_3 = ((int32_t)(FP_FACTOR*(-alpha*alpha*(sin(high_edge)*cos(low_edge) - sin(low_edge)*cos(high_edge)))));
#else
    coeffs->low_band_edge_coeff[0] = 2.0*alpha*cos(low_edge);
    coeffs->high_band_edge_coeff[0] = 2.0*alpha*cos(high_edge);
    coeffs->low_band_edge_coeff[1] =
    coeffs->high_band_edge_coeff[1] = -alpha*alpha;
    coeffs->low_band_edge_coeff[2] = -alpha*sin(low_edge);
    coeffs->high_band_edge_coeff[2] = -alpha*sin(high_edge);
    coeffs->mixed_edges_coeff_3 = -alpha*alpha*(sin(high_edge)*cos(low_edge) - sin(low_edge)*cos(high_edge));
#endif
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v34_rx_carrier_frequency(v34_state_t *s)
{
    return dds_frequency(s->rx.v34_carrier_phase_rate);
}
/*- End of function --------------------------------------------------------*/

static void report_status_change(v34_rx_state_t *s, int status)
{
    if (s->put_bit)
        s->put_bit(s->put_bit_user_data, status);
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

#if 0

SPAN_DECLARE(float) v34_rx_symbol_timing_correction(v34_state_t *s)
{
    return (float) s->rx.total_baud_timing_correction/((float) V34_RX_PULSESHAPER_COEFF_SETS*10.0f/3.0f);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v34_rx_signal_power(v34_state_t *s)
{
    return power_meter_current_dbm0(&s->rx.power);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_equalizer_state(v34_state_t *s, complexf_t **coeffs)
{
    *coeffs = s->rx.eq_coeff;
    return V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;
}
/*- End of function --------------------------------------------------------*/

static void report_status_change(v34_rx_state_t *s, int status)
{
    if (s->status_handler)
        s->status_handler(s->status_user_data, status);
    else if (s->put_bit)
        s->put_bit(s->put_bit_user_data, status);
    /*endif*/
}
/*- End of function --------------------------------------------------------*/
#endif  /* #if 0 - disabled API functions above */

static void equalizer_save(v34_rx_state_t *s)
{
    cvec_copyf(s->eq_coeff_save, s->eq_coeff, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    s->eq_coeff_save_baud_rate = s->baud_rate;
    s->eq_coeff_save_high_carrier = s->high_carrier ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

static void equalizer_restore(v34_rx_state_t *s)
{
    cvec_copyf(s->eq_coeff, s->eq_coeff_save, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    cvec_zerof(s->eq_buf, V34_EQUALIZER_MASK);

    s->eq_put_step = V34_RX_PULSESHAPER_COEFF_SETS*10/(3*2) - 1;
    s->eq_step = 0;
    s->eq_delta = EQUALIZER_SLOW_ADAPT_RATIO*EQUALIZER_DELTA/(V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
}
/*- End of function --------------------------------------------------------*/

static void equalizer_reset(v34_rx_state_t *s)
{
    /* Start with an equalizer based on everything being perfect */
    cvec_zerof(s->eq_coeff, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    s->eq_coeff[V34_EQUALIZER_PRE_LEN] = complex_sig_set(TRAINING_SCALE(1.0f), TRAINING_SCALE(0.0f));
    cvec_zerof(s->eq_buf, V34_EQUALIZER_MASK);

    s->eq_put_step = V34_RX_PULSESHAPER_COEFF_SETS*10/(3*2) - 1;
    s->eq_step = 0;
    s->eq_delta = EQUALIZER_DELTA/(V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    s->eq_target_mag = 0.0f;  /* Will be initialized from first equalizer output */
}
/*- End of function --------------------------------------------------------*/

static complexf_t equalizer_get(v34_rx_state_t *s)
{
    int i;
    int p;
    complexf_t z;
    complexf_t z1;

    /* Get the next equalized value. */
    z = zero;
    p = s->eq_step - 1;
    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
    {
        p = (p - 1) & V34_EQUALIZER_MASK;
        z1 = complex_mulf(&s->eq_coeff[i], &s->eq_buf[p]);
        z = complex_addf(&z, &z1);
    }
    /*endfor*/
    /* Guard against NaN/Inf from coefficient divergence — reset to
       center tap if the equalizer has blown up. */
    if (!isfinite(z.re) || !isfinite(z.im))
    {
        if (V34_DIAG_GETENV("ME_V34_DUMP_MP_DIBITS"))
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - EQUALIZER DIVERGED (NaN/Inf) at duration=%d stage=%d; resetting to identity coeffs\n",
                     s->duration, s->stage);
        }
        cvec_zerof(s->eq_coeff, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
        s->eq_coeff[V34_EQUALIZER_PRE_LEN] = complex_sig_set(TRAINING_SCALE(1.0f), TRAINING_SCALE(0.0f));
        z.re = 0.0f;
        z.im = 0.0f;
    }
    /*endif*/
    return z;
}
/*- End of function --------------------------------------------------------*/

/* Decision-directed equalizer adaptation in data mode is on. */
int v34_rx_data_mode_eq_enabled(void)
{
    return 1;
}
/*- End of function --------------------------------------------------------*/

/* Step-size scale for the data-mode DD-LMS, relative to the training step.
   Training's step is sized for a constant-modulus reference that is right every
   time; data decisions are neither, and on a real channel a step that is fine
   over a flat loopback walks the taps off: measured live at 3000 baud/9600, the
   full step took a call that had been sitting at 0.10 from the lattice out to
   0.66 within twelve thousand symbols and cost three quarters of the payload,
   while the same call with data-mode adaptation off held 0.10 to the end. */
float v34_rx_data_mode_eq_step(const v34_rx_state_t *s)
{
    return v34_rx_b1_batch_eq_enabled(s) ? 0.03f : 1.0f;
}
/*- End of function --------------------------------------------------------*/

void v34_rx_tune_equalizer(v34_rx_state_t *s, const complexf_t *z, const complexf_t *target)
{
    int i;
    int p;
    complexf_t ez;
    complexf_t z1;

    /* Find the x and y mismatch from the exact constellation position. */
    ez = complex_subf(target, z);
    if (!isfinite(z->re) || !isfinite(z->im)
        || !isfinite(target->re) || !isfinite(target->im)
        || !isfinite(ez.re) || !isfinite(ez.im))
    {
        return;
    }
    /*endif*/
    /* Keep the live modem path quiet by default.  This hook runs every 256
     * baud updates during V.90 CP; synchronous stderr writes can consume the
     * entire media-tick margin during a long loopback.  Opt in only when the
     * equalizer trace is explicitly requested. */
    if (V34_DIAG_GETENV("ME_V34_EQ_DIAG") && (s->duration & 0xFF) == 0)
    {
        float emag = sqrtf(ez.re*ez.re + ez.im*ez.im);
        float zmag = sqrtf(z->re*z->re + z->im*z->im);
        fprintf(stderr, "[EQ] %s baud=%d err=%.4f mag=%.4f target_mag=%.4f delta=%.6f stage=%s\n",
                s->calling_party ? "caller" : "answer",
                s->duration, emag, zmag, s->eq_target_mag, s->eq_delta,
                v34_rx_stage_to_str(s->stage));
    }
    ez.re *= s->eq_delta;
    ez.im *= s->eq_delta;

    p = s->eq_step - 1;
    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
    {
        p = (p - 1) & V34_EQUALIZER_MASK;
        z1 = complex_conjf(&s->eq_buf[p]);
        z1 = complex_mulf(&ez, &z1);
        s->eq_coeff[i] = complex_addf(&s->eq_coeff[i], &z1);
        /* Leak disabled — was causing coefficient decay faster than LMS convergence
           (0.9999^2400 = 0.787/sec), making equalizer actively harmful */
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

/* Has Phase 4 CMA done the job it is there for?
 *
 * Track the equalizer output magnitude and declare the level converged once a
 * settled estimate sits inside a tolerance of the unit circle CMA drives to.
 * Once it does, CMA stands down for the rest of Phase 4 and the 11.3 solution
 * is carried into MP unchanged.  Deliberately sticky: a momentary excursion
 * must not restart the blind gradient, which is the behaviour being removed. */

/* Phase 3 keeps adapting through the peer's TRN and Ja so that Ja -- which is
   data, and needs every symbol right -- is not decoded through a frozen,
   drifting front end.  What Ja needs from that is the *carrier* loop.  The
   blind CMA running beside it reshapes taps that PP training has already put
   where they belong: measured over the G.711 round trip the equalizer is the
   identity at the end of PP refinement (main tap 0.99 of 1.00 total energy)
   and has been walked to 0.83 by the time Phase 4 starts, with the off-centre
   energy quadrupled.  Stopping the blind tap gradient here was tried; CMA
   stays on. */
static int phase3_cma_disabled(void)
{
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int phase4_cma_converged(v34_rx_state_t *s, const complexf_t *z)
{
    float mag;

    if (s->stage != V34_RX_STAGE_PHASE4_TRN)
        return 0;
    /*endif*/
    if (s->phase4_cma_settled)
        return 1;
    /*endif*/
    mag = sqrtf(z->re*z->re + z->im*z->im);
    if (!isfinite(mag))
        return 0;
    /*endif*/
    if (s->phase4_cma_mag <= 0.0f)
        s->phase4_cma_mag = mag;
    else
        s->phase4_cma_mag += 0.02f*(mag - s->phase4_cma_mag);
    /*endif*/
    s->phase4_cma_bauds++;
    if ((s->phase4_cma_bauds >= PHASE4_CMA_SETTLE_BAUDS
         &&  fabsf(s->phase4_cma_mag - 1.0f) <= PHASE4_CMA_SETTLE_TOL)
        ||  s->phase4_cma_bauds >= PHASE4_CMA_MAX_BAUDS)
    {
        s->phase4_cma_settled = 1;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: CMA level converged after %d bauds (|z|=%.3f); "
                 "holding the Phase 3 tap solution\n",
                 s->phase4_cma_bauds, (double) s->phase4_cma_mag);
        return 1;
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* Whether 9.6's CP conditioning keeps adapting once the level has settled.
   See the call site for why the startup freeze does not carry over. */
static int v90_reneg_cp_adapt_through_burst(void)
{
    return 0;      /* frozen once the level has settled: see the history in docs */
}
/*- End of function --------------------------------------------------------*/

/* V.90 9.6.1.1.1's CP conditioning trains on Figure 8's SCR.  Same settle
   rule as phase4_cma_converged(), but that one is scoped to
   V34_RX_STAGE_PHASE4_TRN and a renegotiation has no TRN stage of its own:
   the receiver is put straight into V34_RX_STAGE_V90_CP, where CMA is frozen
   for the good reason that at STARTUP the taps arriving there were trained by
   Phase 3 moments earlier.  Measured on a replay of a live renegotiation
   (artifacts/reneg-eq/reneg-r1): with the freeze in force the equalizer
   output sat at |z| = 18-22 against the slicer's unit circle for the whole
   window, descrambled SCR read 68% ones instead of ~100%, and the Table-14
   framer took 381 false syncs and not one CRC-valid frame. */
static int v90_reneg_cma_converged(v34_rx_state_t *s, const complexf_t *z)
{
    float mag;

    if (!s->reneg_cp_train)
        return 1;
    /*endif*/
    mag = sqrtf(z->re*z->re + z->im*z->im);
    if (!isfinite(mag))
        return 0;
    /*endif*/
    if (s->reneg_cma_mag <= 0.0f)
        s->reneg_cma_mag = mag;
    else
        s->reneg_cma_mag += 0.02f*(mag - s->reneg_cma_mag);
    /*endif*/
    s->reneg_cma_bauds++;
    if (s->reneg_cma_bauds >= PHASE4_CMA_SETTLE_BAUDS
        &&
        fabsf(s->reneg_cma_mag - 1.0f) <= PHASE4_CMA_SETTLE_TOL)
    {
        s->reneg_cp_settled = 1;
        /* Whether to keep adapting through the peer's CP burst.
         *
         * Freezing here copies the startup rule, and the reason startup
         * freezes does not apply: there the taps arriving at the CP stage were
         * trained by Phase 3 moments earlier and blind CMA can only walk them
         * off, while here they were trained seconds ago on SCR and there is no
         * trained solution to protect.  Both signals in this window are
         * constant modulus -- Figure 8's SCR is scrambled ones and 8.5.2's CP
         * goes out through J's 4-point modulation -- so CMA is legitimate
         * across the whole of it, which is not true of the startup seam.
         * Measured, freezing wins (see v90_reneg_cp_adapt_through_burst()). */
        if (v90_reneg_cp_adapt_through_burst())
        {
            s->reneg_cma_bauds = 0;
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - V.90 9.6: CP-stage CMA level reached (|z|=%.3f); "
                     "keeping the taps adapting through the CP burst\n",
                     (double) s->reneg_cma_mag);
            return 0;
        }
        /*endif*/
        s->reneg_cp_train = 0;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90 9.6: CP-stage CMA converged on SCR after %d bauds "
                 "(|z|=%.3f); freezing the taps for CP\n",
                 s->reneg_cma_bauds, (double) s->reneg_cma_mag);
        return 1;
    }
    /*endif*/
    if (s->reneg_cma_bauds >= PHASE4_CMA_MAX_BAUDS)
    {
        s->reneg_cp_train = 0;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90 9.6: CP-stage CMA gave up after %d bauds "
                 "(|z|=%.3f)\n",
                 s->reneg_cma_bauds, (double) s->reneg_cma_mag);
        return 1;
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void tune_equalizer_cma(v34_rx_state_t *s, const complexf_t *z)
{
    int i;
    int p;
    complexf_t gz;
    complexf_t z1;
    float y_mag2;
    float R2;
    float error;
    float cma_delta;

    /* CMA (Constant Modulus Algorithm) — blind equalizer for constant-envelope
       signals like DQPSK.  Minimizes E[(R² - |y|²)²] without needing to know
       which symbol was transmitted.  Immune to the decision-directed convergence
       failure that occurs at 25% BER. */
    y_mag2 = z->re*z->re + z->im*z->im;
    /* Fixed unit radius for QPSK.  Measured and ruled out as the cause of the
       Phase 4 constellation collapse above 2400 baud: the Phase 3 solution
       delivers |z| ~ 1.47 in this receiver's scale, so CMA does pull the tap
       gain down 32% at the seam, but driving R2 from the PP-measured
       eq_target_mag instead moves the level (0.90 -> 1.11) and leaves the
       4th-power coherence where it was (0.42 -> 0.39).  The collapse is ISI,
       not gain. */
    R2 = 1.0f;
    error = R2 - y_mag2;

    cma_delta = s->eq_delta;

    /* Log CMA error periodically */
    if (V34_TRACE_DIAGNOSTICS && ((s->duration & 0xFF) == 0))
    {
        fprintf(stderr, "[CMA] %s baud=%d err=%.4f mag=%.4f R=1.0000 delta=%.6f stage=%s\n",
                s->calling_party ? "caller" : "answer",
                s->duration, error, sqrtf(y_mag2), s->eq_delta,
                v34_rx_stage_to_str(s->stage));
    }

    /* Normalized CMA gradient: error * y / |y|² — normalizing by |y|²
       prevents the positive feedback loop where large output → large gradient
       → even larger output.  This is equivalent to NCMA and makes convergence
       independent of the output magnitude.  Floor y_mag2 to avoid division
       by zero when the equalizer output is near zero. */
    if (error < -4.0f * R2)
        error = -4.0f * R2;
    else if (error > 4.0f * R2)
        error = 4.0f * R2;
    /*endif*/
    {
        float norm = (y_mag2 > 0.001f) ? y_mag2 : 0.001f;
        gz.re = 0.1f * cma_delta * error * z->re / norm;
        gz.im = 0.1f * cma_delta * error * z->im / norm;
    }

    p = s->eq_step - 1;
    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
    {
        p = (p - 1) & V34_EQUALIZER_MASK;
        z1 = complex_conjf(&s->eq_buf[p]);
        z1 = complex_mulf(&gz, &z1);
        s->eq_coeff[i].re += z1.re;
        s->eq_coeff[i].im += z1.im;
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

#if 0  /* Disabled functions - track_carrier reimplemented inline, others unused */
static void track_carrier(v34_rx_state_t *s, const complexf_t *z, const complexf_t *target)
{
    float error;

    /* For small errors the imaginary part of the difference between the actual and the target
       positions is proportional to the phase error, for any particular target. However, the
       different amplitudes of the various target positions scale things. */
    error = z->im*target->re - z->re*target->im;

    s->v34_carrier_phase_rate += (int32_t) (s->carrier_track_i*error);
    s->carrier_phase += (int32_t) (s->carrier_track_p*error);
    //V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - Im = %15.5f   f = %15.5f\n", error, dds_frequencyf(s->v34_carrier_phase_rate));
    //printf("XXX Im = %15.5f   f = %15.5f   %f %f %f %f (%f %f)\n", error, dds_frequencyf(s->v34_carrier_phase_rate), target->re, target->im, z->re, z->im, s->carrier_track_i, s->carrier_track_p);
}
/*- End of function --------------------------------------------------------*/

static __inline__ void put_bit(v34_rx_state_t *s, int bit)
{
    int out_bit;

    /* We need to strip the last part of the training - the test period of all 1s -
       before we let data go to the application. */
    if (s->training_stage == TRAINING_TX_STAGE_NORMAL_OPERATION_V34)
    {
        out_bit = v34_rx_descramble(s, bit);
        /* V.90 §8.5.1 defines B1 as V.34's final all-ones training frame.
           The T/3 receiver must consume it to advance the trellis, mapper and
           descrambler, but it is not user data and must never reach V.42's
           ODP detector.  The ordinary V.34 path uses training_stage for this
           gate; the independently acquired V.90 path needs its own boundary. */
        if (!s->v90_t3_suppress_output)
            s->put_bit(s->put_bit_user_data, out_bit);
    }
    else if (s->training_stage == TRAINING_STAGE_TEST_ONES)
    {
        /* The bits during the final stage of training should be all ones. However,
           buggy modems mean you cannot rely on this. Therefore we don't bother
           testing for ones, but just rely on a constellation mismatch measurement. */
        out_bit = v34_rx_descramble(s, bit);
        //V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - A 1 is really %d\n", out_bit);
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

#if defined(SPANDSP_USE_FIXED_POINT)
static __inline__ uint32_t dist_sq(const complexi_t *x, const complexi_t *y)
{
    return (x->re - y->re)*(x->re - y->re) + (x->im - y->im)*(x->im - y->im);
}
/*- End of function --------------------------------------------------------*/
#else
static __inline__ float dist_sq(const complexf_t *x, const complexf_t *y)
{
    return (x->re - y->re)*(x->re - y->re) + (x->im - y->im)*(x->im - y->im);
}
/*- End of function --------------------------------------------------------*/
#endif

#endif

static __inline__ complex_sig_t training_get(v34_tx_state_t *s)
{
    return zero;
}
/*- End of function --------------------------------------------------------*/

static __inline__ complex_sig_t connect_sequence_get(v34_tx_state_t *s)
{
    return zero;
}
/*- End of function --------------------------------------------------------*/

#if defined(SPANDSP_USE_FIXED_POINT)
#else
static void straight_line_fit(float *slope, float *intercept, const float x[], const float y[], int data_points)
{
    float sum_x;
    float sum_y;
    float sum_xy;
    float sum_x2;
    float slopex;
    int i;

    sum_x = 0.0f;
    sum_y = 0.0f;
    sum_xy = 0.0f;
    sum_x2 = 0.0f;
    for (i = 0;  i < data_points;  i++)
    {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i]*y[i];
        sum_x2 += x[i]*x[i];
    }
    /*endfor*/
    slopex = (sum_xy - sum_x*sum_y/data_points)/(sum_x2 - sum_x*sum_x/data_points);
    if (slope)
        *slope = slopex;
    /*endif*/
    if (intercept)
        *intercept = (sum_y - slopex*sum_x)/data_points;
    /*endif*/
}
/*- End of function --------------------------------------------------------*/
#endif

static void slow_dft(complexf_t data[], int len)
{
    int i;
    int bin;
    float arg;
    complexf_t buf[len];

    for (i = 0;  i < len;  i++)
    {
        buf[i].re = data[i].re;
        buf[i].im = data[i].im;
    }
    /*endfor*/

    for (bin = 0;  bin <= len/2;  bin++)
    {
        data[bin].re =
        data[bin].im = 0.0;
        for (i = 0;  i < len;  i++)
        {
            arg = bin*2.0f*3.1415926535f*i/(float) len;
            data[bin].re -= buf[i].re*sinf(arg);
            data[bin].im += buf[i].re*cosf(arg);
        }
        /*endfor*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static int perform_l1_l2_analysis(v34_rx_state_t *s)
{
    /* Phase adjustments to compensate for the tones which are sent phase inverted */
    static const float adjust[25] =
    {
        0.0f,           /**/
        3.14159265f,    /* 300 */
        0.0f,           /**/
        0.0f,           /**/
        0.0f,           /**/
        42.0f,          /* Tone not sent */
        0.0f,           /* 1050 nominal line probe frequency */
        42.0f,          /* Tone not sent */
        0.0f,           /**/
        0.0f,           /**/
        3.14159265f,    /* 1650 */
        42.0f,          /* Tone not sent */
        0.0f,           /**/
        0.0f,           /**/
        3.14159265f,    /* 2250 */
        42.0f,          /* Tone not sent */
        0.0f,           /**/
        3.14159265f,    /* 2700 */
        0.0f,           /**/
        3.14159265f,    /* 3000 */
        3.14159265f,    /* 3150 */
        3.14159265f,    /* 3300 */
        3.14159265f,    /* 3450 */
        0.0f,           /**/
        0.0f            /**/
    };
    int i;
    int j;
    float phase_1050;

    slow_dft(s->dft_buffer, LINE_PROBE_SAMPLES);
    phase_1050 = atan2f(s->dft_buffer[21].im, s->dft_buffer[21].re);
    /* Now resolve the analysis into gain and phase values for the bins which contain the tones */ 
    /* Base things around what happens at 1050Hz the first time through. */
    if (s->l1_l2_duration == 0)
        s->base_phase = phase_1050;
    /*endif*/
    for (i = 0;  i < 25;  i++)
    {
        j = 3*(i + 1);
        if (adjust[i] < 7.0f)
        {
            /* This tone should be present in the transmitted signal. */
            s->l1_l2_gains[i] = sqrtf(s->dft_buffer[j].re*s->dft_buffer[j].re
                                    + s->dft_buffer[j].im*s->dft_buffer[j].im);
            s->l1_l2_phases[i] = fmodf(atan2f(s->dft_buffer[j].im, s->dft_buffer[j].re) - s->base_phase + adjust[i],
                                       3.14159265f);
            /* V.34 10.1.2.3.4 and V.92 Table 17: INFO1 reports what the
               receiver measured from L1/L2.  Accumulate only L2 (the first
               eight 20-ms blocks are L1, 6 dB higher) so gain-dependent
               choices are not biased by the level transition. */
            if (s->l1_l2_duration >= 8)
                s->l1_l2_gain_sum[i] += s->l1_l2_gains[i];
            /*endif*/
        }
        else
        {
            float noise;

            /* The deliberately unoccupied probe bins provide an in-band
               noise estimate without requiring a second analysis window. */
            noise = sqrtf(s->dft_buffer[j].re*s->dft_buffer[j].re
                        + s->dft_buffer[j].im*s->dft_buffer[j].im);
            s->l1_l2_gains[i] = 0.0f;
            s->l1_l2_phases[i] = 0.0f;
            if (s->l1_l2_duration >= 8)
            {
                s->l1_l2_noise_sum += noise;
                s->l1_l2_noise_count++;
            }
            /*endif*/
        }
        /*endif*/
    }
    if (s->l1_l2_duration >= 8)
    {
        float step;

        s->l1_l2_gain_count++;
        if (s->l1_l2_have_prev_1050_phase)
        {
            step = phase_1050 - s->l1_l2_prev_1050_phase;
            while (step > 3.14159265f)
                step -= 2.0f*3.14159265f;
            while (step < -3.14159265f)
                step += 2.0f*3.14159265f;
            s->l1_l2_1050_phase_step_sum += step;
            s->l1_l2_1050_phase_step_count++;
        }
        s->l1_l2_prev_1050_phase = phase_1050;
        s->l1_l2_have_prev_1050_phase = 1;
    }
    /*endfor*/
    for (i = 0;  i < 25;  i++)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_DEBUG, "DFT %4d, %12.5f, %12.5f, %12.5f\n",
                 i,
                 (i + 1)*150.0f,
                 s->l1_l2_gains[i],
                 s->l1_l2_phases[i]);
    }
    /*endfor*/
    //straight_line_fit(&slope, &intercept, x, y, data_points);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void l1_l2_analysis_init(v34_rx_state_t *s)
{
    V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "Rx - Expect L1/L2\n");
    s->dft_ptr = 0;
    s->base_phase = 42.0;
    s->l1_l2_duration = 0;
    memset(s->l1_l2_gain_sum, 0, sizeof(s->l1_l2_gain_sum));
    s->l1_l2_gain_count = 0;
    s->l1_l2_noise_sum = 0.0f;
    s->l1_l2_noise_count = 0;
    s->l1_l2_prev_1050_phase = 0.0f;
    s->l1_l2_1050_phase_step_sum = 0.0f;
    s->l1_l2_1050_phase_step_count = 0;
    s->l1_l2_have_prev_1050_phase = 0;
    s->current_demodulator = V34_MODULATION_L1_L2;
    s->stage = V34_RX_STAGE_L1_L2;
}
/*- End of function --------------------------------------------------------*/

static int l1_l2_analysis(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;

    /* We need to work over whole cycles of the L1/L2 pattern, to avoid windowing and
       all its ills. One cycle takes 160/3 samples at 8000 samples/second, so we will
       process groups of 3 cycles, and run a Fourier transform every 160 samples (20ms).
       Since this is not a suitable length for an FFT we have to run a slow DFT. However,
       we don't do this for much of the time, so its not that big a deal. */
    for (i = 0;  i < len;  i++)
    {
        s->dft_buffer[s->dft_ptr].re = amp[i];
        s->dft_buffer[s->dft_ptr].im = 0.0f;
        if (++s->dft_ptr >= LINE_PROBE_SAMPLES)
        {
            /* We now have 160 samples, so process the 3 cycles we should have in the buffer. */
            perform_l1_l2_analysis(s);
            s->dft_ptr = 0;
            V34_RX_LOG(s->logging, SPAN_LOG_DEBUG, "L1/L2 analysis x %d\n", s->l1_l2_duration);
            if (++s->l1_l2_duration > 20)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW, "L1/L2 analysis done\n");
                s->phase2_l2_count++;
                s->current_demodulator = V34_MODULATION_TONES;
                if (s->calling_party  &&  s->v90_mode
                    &&  s->received_event == V34_EVENT_INFO1_OK)
                {
                    /* INFO1d already arrived inside the window; keep it. */
                }
                else
                {
                    s->received_event = V34_EVENT_L2_SEEN;
                }
                /*endif*/
                if (s->calling_party  &&  s->v90_mode
                    &&  s->stage == V34_RX_STAGE_INFO1C)
                {
                    /* Already receiving, or received, INFO1d. */
                }
                else if (s->calling_party  &&  s->v90_mode)
                {
                    /* V.90 9.2.2.1.8: having received L1 and L2 the analogue
                       modem "shall then transmit Tone A and condition its
                       receiver to receive INFO1d".  The digital modem sends
                       INFO1d straight after its L2 (9.2.1.1.7) with no
                       further Tone B reversal, so waiting for a third
                       reversal as the V.34 call modem does misses it: against
                       a Cisco MICA the receiver armed only when Tone B ended,
                       6.7 s after INFO1d had gone by. */
                    s->stage = V34_RX_STAGE_INFO1C;
                    /* Framing started at reversal 2; a frame already in
                       progress here is INFO1d's own, so keep it. */
                    if (s->target_bits != 109 - (4 + 8 + 4))
                    {
                        s->target_bits = 109 - (4 + 8 + 4);
                        s->bit_count = 0;
                    }
                    /*endif*/
                    s->phase2_reversal_count = 3;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 caller: L1/L2 received, expecting INFO1d (109 bits) (9.2.2.1.8)\n");
                }
                else if (s->calling_party)
                {
                    s->stage = V34_RX_STAGE_TONE_A;
                    /* L2 has been seen, so Phase 2 is demonstrably past the
                       first two reversals. Seed the durable counter with that
                       rather than zeroing it: received_event carries the same
                       fact today, but v34tx.c clears it as it consumes events,
                       which is exactly how this progress used to get lost. */
                    s->phase2_reversal_count = 2;
                }
                else if (s->v90_mode)
                {
                    s->stage = V34_RX_STAGE_TONE_A;
                    s->phase2_reversal_count = 2;   /* see above: L2 seen */
                    s->persistence1 = 0;
                    s->persistence2 = 0;
                }
                else
                    s->stage = V34_RX_STAGE_INFO1C;
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/
    /* Also run this signal through the info analysis, so we pick up A or B tones */
    info_rx(s, amp, len);

    return 0;
}
/*- End of function --------------------------------------------------------*/

static void process_cc_half_baud(v34_rx_state_t *s, const complexf_t *sample)
{
    int i;
    int data_bits;
    mp_t mp;
    mph_t mph;
    uint32_t ang1;
    uint32_t ang2;
    uint32_t ang3;
    int bits[4];
    v34_state_t *t;

    /* This routine processes every half a baud, as we put things into the equalizer
       at the T/2 rate. This routine adapts the position of the half baud samples,
       which the caller takes. */
#if 0
    /* Add a sample to the equalizer's circular buffer, but don't calculate anything at this time. */
    s->eq_buf[s->eq_step] = *sample;
    s->eq_step = (s->eq_step + 1) & V34_EQUALIZER_MASK;
#endif

    if (s->stage == V34_RX_STAGE_CC)
    {
        /* 10.2.4.1/12.8.2: AC alternates antipodal points. Require
           more than 100 ms on one eye phase, with energy at both symbols;
           silence and a single reversal cannot request a control retrain. */
        int half = s->baud_half;
        complexf_t prev = s->hdx_ac_previous[half];
        float dot = prev.re*sample->re + prev.im*sample->im;
        float cross = prev.re*sample->im - prev.im*sample->re;
        float power = sample->re*sample->re + sample->im*sample->im;
        float old_power = prev.re*prev.re + prev.im*prev.im;
        if (power > 0.01f && old_power > 0.01f && dot < 0.0f
            && cross*cross < 0.10f*power*old_power)
            s->hdx_ac_count[half]++;
        else
            s->hdx_ac_count[half] = 0;
        if (s->hdx_ac_count[half] > 60)
            s->hdx_ac_seen = true;
        s->hdx_ac_previous[half] = *sample;
    }
    if (s->stage == V34_RX_STAGE_CC && !s->pph_detected && s->hdx_cc_resync)
    {
        /* 10.2.3.3 and 12.6: correlate the complete 24T Sh + 8T Sh-bar
           at each eye phase. Requiring the barred suffix distinguishes
           the resynchronization signal from a continuous alternating tone.
           Normalized complex correlation permits unknown carrier phase. */
        int half = s->baud_half;
        int count = s->hdx_sh_count[half]++;
        int j;
        complexf_t corr = {0.0f, 0.0f};
        float energy = 0.0f, reference_energy = 0.0f;

        s->hdx_sh_history[half][count & 31] = *sample;
        if (count >= 31)
        {
            for (j = 0; j < 32; j++)
            {
                int point = j < 24 ? ((j & 1) ? 1 : 2) : ((j & 1) ? 3 : 0);
                /* Table 10.2.3.3 points, normalized; only direction
                   matters to the normalized correlation. */
                complexf_t ref = { point < 2 ? -1.0f : 1.0f,
                                   (point == 0 || point == 3) ? -1.0f : 1.0f };
                complexf_t z = s->hdx_sh_history[half][(count + 1 + j) & 31];
                corr.re += z.re*ref.re + z.im*ref.im;
                corr.im += z.im*ref.re - z.re*ref.im;
                energy += z.re*z.re + z.im*z.im;
                reference_energy += ref.re*ref.re + ref.im*ref.im;
            }
            if (energy > 0.01f && corr.re*corr.re + corr.im*corr.im >
                0.92f*energy*reference_energy)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                           "Rx - CC: Sh/Sh-bar detected (12.6), half=%d\n", half);
                s->hdx_sh_seen = true;
                s->pph_detected = true;
                s->baud_half = 0;
                s->mp_seen = 1; /* retained MPh parameters; now hunt E */
                s->mp_count = -1;
                s->bitstream = 0;
                s->bit_count = 0;
                memset(&s->last_sample, 0, sizeof(s->last_sample));
                return;
            }
        }
    }
    if (s->stage == V34_RX_STAGE_CC  &&  !s->pph_detected)
    {
        /* V.34 12.4.1.1/12.4.2.1: the modem conditions its receiver to detect
           signal PPh.  PPh (10.2.4.5) is four periods of a fixed 8-symbol
           sequence carrying neither the scrambler nor the differential
           encoder, so it is found by correlating the control channel symbols
           against the known pattern.  The control channel has no equalizer
           and starts on an arbitrary carrier phase, so the score is
           |correlation| normalised by the signal energy, exactly as the
           primary channel's PP acquisition at 11.3.1.2.4 does.

           This runs at the T/2 rate, ABOVE the baud_half gate, and keeps a
           correlator bank for each parity.  PPh is only 32 symbols and it is
           preceded by 12.4.1.1's 70 ms of silence, which leaves the band edge
           timing recovery nothing to converge on: whichever of the two T/2
           outputs is the eye centre is therefore decided by the correlation
           itself, and baud_half is aligned to the winner.  The same ambiguity
           on the primary channel is documented in docs/v34_symbol_rate_matrix
           terms -- a receiver that samples at the eye crossing looks exactly
           like one with no signal. */
        int phase;
        int best_phase;
        int best_half;
        float best_score;
        s->pph_hunt_bauds++;
        if (s->pph_bounded_acquisition)
        {
            /* 10.2.4.5: two complete eight-symbol periods, normalized with
               the energy of this same eye-phase window. Four further matching
               windows confirm the signal within its 32T duration. A bounded
               window excludes arbitrary old user data/AC during 12.8 recovery;
               the former exponential history could dilute the entire PPh. */
            best_phase = 0;
            best_half = s->baud_half;
            best_score = -1.0f;
            {
                int half = s->baud_half;
                complexf_t *history = s->pph_history[half];
                float energy = 0.0f;
                memmove(history, history + 1, 15*sizeof(*history));
                history[15] = *sample;
                if (s->pph_history_count[half] < 16)
                    s->pph_history_count[half]++;
                for (int k = 0; k < 16; k++)
                    energy += history[k].re*history[k].re + history[k].im*history[k].im;
                for (phase = 0; phase < 8; phase++)
                {
                    complexf_t corr = {0.0f, 0.0f};
                    float pattern_energy = 0.0f;
                    for (int k = 0; k < 16; k++)
                    {
                        complexf_t cand = pph_symbols[(k + phase)%8];
                        corr.re += history[k].re*cand.re + history[k].im*cand.im;
                        corr.im += history[k].im*cand.re - history[k].re*cand.im;
                        pattern_energy += cand.re*cand.re + cand.im*cand.im;
                    }
                    float denom = sqrtf(energy*pattern_energy);
                    float score = (denom > 0.0001f && s->pph_history_count[half] == 16)
                                ? sqrtf(corr.re*corr.re + corr.im*corr.im)/denom : 0.0f;
                    if (score > best_score)
                    {
                        best_score = score;
                        best_phase = phase;
                    }
                }
            }
        }
        else
        {
            float mag = sqrtf(sample->re*sample->re + sample->im*sample->im);
            s->pph_corr_energy = PPH_ACQUIRE_DECAY*s->pph_corr_energy + mag*mag;
            s->pph_corr_weight = PPH_ACQUIRE_DECAY*s->pph_corr_weight + 1.0f;
            best_phase = 0;
            best_half = 0;
            best_score = -1.0f;
            {
                int half = s->baud_half;
                for (phase = 0;  phase < 8;  phase++)
                {
                    complexf_t cand;
                    float corr_mag;
                    float denom;
                    float score;

                    /* pph_symbols[] is the whole 32 symbol signal, but it is the
                       8 symbol period repeated four times, so an 8-entry phase
                       search covers it. */
                    cand = pph_symbols[((s->pph_hunt_bauds >> 1) + phase)%8];
                    s->pph_corr[half][phase].re = PPH_ACQUIRE_DECAY*s->pph_corr[half][phase].re
                                                + sample->re*cand.re + sample->im*cand.im;
                    s->pph_corr[half][phase].im = PPH_ACQUIRE_DECAY*s->pph_corr[half][phase].im
                                                + sample->im*cand.re - sample->re*cand.im;
                    corr_mag = sqrtf(s->pph_corr[half][phase].re*s->pph_corr[half][phase].re
                                   + s->pph_corr[half][phase].im*s->pph_corr[half][phase].im);
                    denom = sqrtf(s->pph_corr_energy*s->pph_corr_weight);
                    score = (denom > 0.0001f) ? corr_mag/denom : 0.0f;
                    if (score > best_score)
                    {
                        best_score = score;
                        best_phase = phase;
                        best_half = half;
                    }
                    /*endif*/
                }
                /*endfor*/
            }
        }
        /* The hold is counted PER PARITY.  Steps alternate between the two
           banks, so comparing against the previous step's winner compares two
           different correlators and the count can never accumulate. */
        if (best_score >= (s->pph_bounded_acquisition ? 0.85f : PPH_ACQUIRE_SCORE_MIN)
            && best_phase == (s->pph_bounded_acquisition
                 ? ((s->pph_best_phase[best_half] + 1) % 8) : s->pph_best_phase[best_half]))
            s->pph_hold_steps[best_half]++;
        else
            s->pph_hold_steps[best_half] = 0;
        /*endif*/
        s->pph_best_phase[best_half] = best_phase;
        if (s->pph_hunt_bauds >= PPH_ACQUIRE_MIN_BAUDS
            &&
            s->pph_hold_steps[best_half] >= (s->pph_bounded_acquisition ? 4 : PPH_ACQUIRE_HOLD_STEPS))
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - CC: PPh detected after %d T/2 steps (phase=%d half=%d score=%.3f held=%d)\n",
                     s->pph_hunt_bauds, best_phase, best_half, best_score,
                     s->pph_hold_steps[best_half]);
            s->pph_detected = true;
            s->received_event = V34_EVENT_PPH;
            /* Take whole bauds from the T/2 output the correlation chose.
               The correlator only ever scores the bank for the parity of the
               step it is on, so a detection always lands ON the winning
               parity: the next step is the loser and must be skipped, the one
               after is the winner and must be processed.  The gate below is
               "toggle, then return if the result is 1", which makes 0 the
               right value here whichever parity won -- setting it to
               best_half instead aligned one modem correctly and the other
               onto the eye crossing, and the two behaved completely
               differently for that reason alone. */
            s->baud_half = 0;
            /* Everything from here is ALT, MPh and E, which the scanner below
               reads out of the descrambled differential bit stream.  Start it
               from a clean slate rather than from whatever PPh left behind. */
            s->bitstream = 0;
            s->mp_seen = 0;
            s->mp_bits_since_frame = 0;
            s->mp_last_frame_bits = 0;
            s->mp_count = -1;
            s->crc = 0xFFFF;
            s->bit_count = 0;
            memset(&s->last_sample, 0, sizeof(s->last_sample));
            return;
        }
        else if ((s->pph_hunt_bauds % 600) == 0)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - CC: hunting PPh, %d T/2 steps, best score %.3f at phase %d half %d\n",
                     s->pph_hunt_bauds, best_score, best_phase, best_half);
        }
        /*endif*/
        s->baud_half ^= 1;
        return;
    }
    /*endif*/

    /* On alternate insertions we have a whole baud and must process it. */
    if ((s->baud_half ^= 1))
    {
        s->cc_qam_mid = *sample;
        return;
    }
    /*endif*/
    if (s->hdx_control_rx_rate == 2400 ||
        (s->stage == V34_RX_STAGE_CC && s->mp_seen < 1 &&
         ((v34_state_t *)((char *)s - offsetof(v34_state_t, rx)))->hdx_control_requested_rate == 2400))
    {
        float power = s->last_sample.re*s->last_sample.re + s->last_sample.im*s->last_sample.im
                    + sample->re*sample->re + sample->im*sample->im;
        float error = ((s->last_sample.re-sample->re)*s->cc_qam_mid.re
                     + (s->last_sample.im-sample->im)*s->cc_qam_mid.im)/(power+0.001f);
        s->cc_ted.baud_phase += error*20.0f;
        int correction = (int)s->cc_ted.baud_phase;
        if (correction > 4) correction = 4;
        if (correction < -4) correction = -4;
        s->eq_put_step += correction;
        s->total_baud_timing_correction += correction;
        s->cc_ted.baud_phase -= correction;
    }
    else
        cc_symbol_sync(s);

    if (s->stage == V34_RX_STAGE_CC && !(s->mp_seen >= 2 && s->hdx_control_rx_rate == 2400))
    {
        float cc_mag = sqrtf(sample->re*sample->re + sample->im*sample->im);

        s->cc_level += (cc_mag - s->cc_level)/(1 << CC_AGC_ADAPT_SHIFT);
        if (s->cc_level > 0.01f)
        {
            float want = s->agc_scaling*CC_AGC_TARGET_MAG/s->cc_level;

            /* Bounded per baud, so one outlier cannot move the working point
               and the gain is constant across any single symbol. */
            if (want > s->agc_scaling*1.02f)
                want = s->agc_scaling*1.02f;
            else if (want < s->agc_scaling*0.98f)
                want = s->agc_scaling*0.98f;
            /*endif*/
            s->agc_scaling = want;
        }
        /*endif*/
    }
    /*endif*/

    if (s->mp_seen >= 2 && s->hdx_control_rx_rate == 2400)
    {
        /* 10.2.4/Figure 5. Keep Zn's reference separate from Q's phase;
           differencing two raw 16-QAM angles would mix Q bits into I bits. */
        static const complexf_t quarter[4] = {{-1,-1}, {3,-1}, {-1,3}, {3,3}};
        if (!s->cc_qam_started)
        {
            float mag = hypotf(s->last_sample.re, s->last_sample.im);
            s->cc_qam_gain = mag > 0.01f ? mag : CC_AGC_TARGET_MAG;
            s->cc_qam_reference.re = (-s->last_sample.re - s->last_sample.im)*0.707106781f/s->cc_qam_gain;
            s->cc_qam_reference.im = (s->last_sample.re - s->last_sample.im)*0.707106781f/s->cc_qam_gain;
            s->cc_qam_started = true;
        }
        complexf_t ref = s->cc_qam_reference;
        float re = (sample->re*ref.re + sample->im*ref.im)/s->cc_qam_gain;
        float im = (sample->im*ref.re - sample->re*ref.im)/s->cc_qam_gain;
        float best = 1.0e30f;
        int chosen = 0;
        for (int q = 0; q < 4; q++)
        {
            float x = quarter[q].re*0.316227766f;
            float y = quarter[q].im*0.316227766f;
            for (int z = 0; z < 4; z++)
            {
                float dist = (re-x)*(re-x) + (im-y)*(im-y);
                if (dist < best) { best = dist; chosen = (q << 2) | z; }
                float tmp = x; x = y; y = -tmp;
            }
        }
        if (getenv("V34_QAM_TRACE") && s->cc_stat_n++ < 30)
            fprintf(stderr, "QAM %p xy=%.3f,%.3f bits=%d dist=%.3f gain=%.3f level=%.3f\n", (void *)s,re,im,chosen,best,s->cc_qam_gain,s->cc_level);
        int q = chosen >> 2;
        complexf_t next = {sample->re*quarter[q].re + sample->im*quarter[q].im,
                          sample->im*quarter[q].re - sample->re*quarter[q].im};
        float norm = hypotf(next.re, next.im);
        /* Advance the differential reference by the decoded I dibit,
           then track residual carrier phase slowly. QAM's instantaneous
           angle is not a reliable carrier estimate. */
        for (int z = 0; z < (chosen & 3); z++)
        {
            float tmp = ref.re; ref.re = ref.im; ref.im = -tmp;
        }
        if (norm > 0.001f)
        {
            float x = ref.re*0.98f + (next.re/norm)*0.02f;
            float y = ref.im*0.98f + (next.im/norm)*0.02f;
            float mag = hypotf(x,y);
            s->cc_qam_reference.re = x/mag;
            s->cc_qam_reference.im = y/mag;
        }
        float gain = hypotf(sample->re, sample->im)/
                     (hypotf(quarter[q].re, quarter[q].im)*0.316227766f);
        s->cc_qam_gain += (gain-s->cc_qam_gain)/64.0f;
        for (int k = 0; k < 4; k++)
            s->put_bit(s->put_bit_user_data, v34_rx_descramble(s, (chosen >> k) & 1));
        s->last_sample = *sample;
        return;
    }

    /* Slice the phase difference, to get a pair of data bits */
    ang1 = arctan2(sample->im, sample->re);
    ang2 = arctan2(s->last_sample.im, s->last_sample.re);
    ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
    data_bits = (ang3 >> 30) & 0x3;
    /* 10.2.4 advances the point index by the dibit, and
       training_constellation_4[] is ordered so an increasing index rotates
       CLOCKWISE while this measures the difference counter-clockwise, so the
       recovered dibit is the NEGATION of the transmitted one.  The same fact
       is pinned in the V.90 9.4 CP decode and in the duplex 11.4 MP decode;
       it is a property of the encoder and the table, not of the channel, so
       there is nothing here to search for. */
    data_bits = (4 - data_bits) & 0x3;

    if (s->hdx_control_rx_rate == 2400)
    {
        /* Establish the QAM carrier/gain reference during the remaining
           1200-bit/s MPh/E training, before future QAM pulses affect E's
           final matched-filter sample. Point 0's phase is 225 degrees. */
        float mag = hypotf(sample->re, sample->im);
        if (mag > 0.01f)
        {
            complexf_t obs = {(-sample->re-sample->im)*0.707106781f/mag,
                             (sample->re-sample->im)*0.707106781f/mag};
            if (!s->cc_qam_started)
            {
                s->cc_qam_reference = obs;
                s->cc_qam_gain = mag;
                s->cc_qam_started = true;
            }
            else
            {
                complexf_t pred = s->cc_qam_reference;
                for (int z = 0; z < data_bits; z++)
                { float x = pred.re; pred.re = pred.im; pred.im = -x; }
                float x = pred.re*0.9f + obs.re*0.1f;
                float y = pred.im*0.9f + obs.im*0.1f;
                float norm = hypotf(x,y);
                s->cc_qam_reference.re = x/norm;
                s->cc_qam_reference.im = y/norm;
                s->cc_qam_gain += (mag-s->cc_qam_gain)*0.2f;
            }
        }
    }

    /* Descramble the data bits. */
    for (i = 0;  i < 2;  i++)
    {
        bits[i] = v34_rx_descramble(s, data_bits & 1);
        data_bits >>= 1;
    }
    /*endfor*/

    if (s->mp_seen >= 2  &&  V34_DIAG_GETENV("V34_CC_SYM_STATS"))
    {
        /* The control channel had no quality instrument at all.  |z| is the
           AGC's working point; the phase error is the distance of the
           differential angle from the CENTRE of its quadrant, so 0 degrees is
           a perfect eye and 45 is the decision boundary. */
        uint32_t resid = ang3 & 0x3FFFFFFFu;
        float deg = fabsf((float) resid*90.0f/1073741824.0f - 45.0f);

        s->cc_stat_sum += sqrtf(sample->re*sample->re + sample->im*sample->im);
        s->cc_stat_err += deg;
        if (deg > s->cc_stat_worst)
            s->cc_stat_worst = deg;
        /*endif*/
        if (++s->cc_stat_n >= 2000)
        {
            fprintf(stderr, "[CC] %s |z|=%.3f phase error from centre: mean %.2f deg worst %.2f deg\n",
                    s->calling_party ? "rx 2400Hz" : "rx 1200Hz",
                    (double) (s->cc_stat_sum/s->cc_stat_n),
                    (double) (s->cc_stat_err/s->cc_stat_n),
                    (double) s->cc_stat_worst);
            s->cc_stat_sum = 0.0f;
            s->cc_stat_err = 0.0f;
            s->cc_stat_worst = 0.0f;
            s->cc_stat_n = 0;
        }
        /*endif*/
    }
    /*endif*/
    /* Scan for MP/MPh and HDLC messages. */
    for (i = 0;  i < 2;  i++)
    {
        s->bitstream = (s->bitstream << 1) | bits[i];
        if (s->mp_seen >= 2)
        {
            /* Real control channel data */
            s->put_bit(s->put_bit_user_data, bits[i]);
            continue;
        }
        /*endif*/
        if (s->mp_seen == 1  &&  (s->bitstream & 0xFFFFF) == 0xFFFFF)
        {
            /* E is 20 consecutive ones, which signals the end of the MPh messages,
               and the start of actual user data */
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - CC: E signal detected, MP exchange complete\n");
            s->mp_seen = 2;
            if (s->hdx_control_rx_rate == 2400)
                s->cc_ted.baud_phase = 0;
            if (s->duplex)
            {
                report_status_change(s, SIG_STATUS_TRAINING_SUCCEEDED);
            }
            /*endif*/
        }
        else if ((s->bitstream & 0x7FFFE) == 0x7FFFC)
        {
            s->crc = 0xFFFF;
            s->bit_count = 0;
            s->mp_count = 17;
            /* Check the type bit, and set the expected length accordingly. */
            if (bits[i])
            {
                s->mp_len = 186 + 1;
                s->mp_and_fill_len = 186 + 1 + 1;
            }
            else
            {
                s->mp_len = 84 + 1;
                s->mp_and_fill_len = 84 + 3 + 1;
            }
            /*endif*/
        }
        /*endif*/
        if (s->mp_count >= 0)
        {
            s->mp_count++;
            /* Don't include the start bits in the CRC calculation. These occur every 16 bits of
               real data - i.e. every 17 bits, including the start bits themselves. */
            if (s->mp_count%17 != 0)
                s->crc = crc_itu16_bits(bits[i], 1, s->crc);
            /*endif*/
            s->bit_count++;
            if ((s->bit_count & 0x07) == 0)
                s->info_buf[(s->bit_count >> 3) - 1] = bit_reverse8(s->bitstream & 0xFF);
            /*endif*/
            if (s->mp_count >= s->mp_len)
            {
                if (s->mp_count == s->mp_len)
                {
                    /* This should be the end of the MPh message */
                    if (s->crc == 0)
                    {
                        if (s->duplex)
                        {
                            process_rx_mp(s, &mp, s->info_buf);
                            if (mp.mp_acknowledged)
                                s->mp_remote_ack_seen = 1;
                            /*endif*/
                            t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                            if (!mp_apply_parameters(t, &mp))
                                V34_RX_LOG(&t->logging, SPAN_LOG_FLOW,
                                         "Rx - CC MP directional encoder/rate negotiation failed\n");
                        }
                        else
                        {
                            process_rx_mph(s, &mph, s->info_buf);
                            t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                            /* Table 23 bit 27 selects this transmitter's
                               control rate (12.4.1.4/12.4.2.5). This modem
                               implements only 1200 bit/s; accepting 2400
                               would silently transmit the wrong bit stream.
                               Likewise, a CRC alone cannot validate a rate
                               offer with no common enabled primary rate. */
                            if (!mph_apply_parameters(t, &mph))
                            {
                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                           "Rx - CC: rejecting unsupported MPh parameters\n");
                                s->mp_count = -1;
                                continue;
                            }
                            if (mph.type == 1)
                            {
                                /* Set the precoder coefficients we are to use */
                                memcpy(&t->tx.precoder_coeffs, mph.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                            }
                            /*endif*/
                            if (set_trellis_mode(t, mph.trellis_size))
                                V34_RX_LOG(&t->logging, SPAN_LOG_FLOW, "Rx - Unexpected trellis size code %d\n", mph.trellis_size);
                            /*endif*/
                        }
                        /*endif*/
                        s->mp_seen = 1;
                        s->mp_accepted_baud = s->duration;
                    }
                    /*endif*/
                }
                /*endif*/
                /* Allow for the fill bits before ending the MP message */
                if (s->mp_count == s->mp_and_fill_len)
                    s->mp_count = -1;
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/

    s->last_sample = *sample;
}
/*- End of function --------------------------------------------------------*/

static int cc_rx(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;
    int step;
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t z;
    complexi16_t zz;
    complexi16_t sample;
#else
    complexf_t z;
    complexf_t zz;
    complexf_t sample;
#endif
    float ii;
    float qq;
    float v;

    step = 6;
    for (i = 0;  i < len;  i++)
    {
        s->rrc_filter[s->rrc_filter_step] = amp[i];
        if (++s->rrc_filter_step >= V34_RX_FILTER_STEPS)
            s->rrc_filter_step = 0;
        /*endif*/

        s->eq_put_step -= RX_PULSESHAPER_2400_COEFF_SETS;
        step = -s->eq_put_step;
        if (step > RX_PULSESHAPER_2400_COEFF_SETS - 1)
            step = RX_PULSESHAPER_2400_COEFF_SETS - 1;
        /*endif*/
        while (step < 0)
            step += RX_PULSESHAPER_2400_COEFF_SETS;
        /*endwhile*/
        /* Standard V.34: caller RX 2400, answerer RX 1200.
           V.90: swapped — caller RX 1200, answerer RX 2400. */
        if (s->calling_party != s->v90_mode)
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        else
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        /*endif*/
#if defined(SPANDSP_USE_FIXED_POINT)
        //sample.re = (ii*(int32_t) s->agc_scaling) >> 15;
        sample.re = ii*s->agc_scaling;
#else
        sample.re = ii*s->agc_scaling;
#endif
        /* Symbol timing synchronisation band edge filters */
        /* Low Nyquist band edge filter */
        v = s->cc_ted.symbol_sync_low[0]*s->cc_ted.low_band_edge_coeff[0] + s->cc_ted.symbol_sync_low[1]*s->cc_ted.low_band_edge_coeff[1] + sample.re;
        s->cc_ted.symbol_sync_low[1] = s->cc_ted.symbol_sync_low[0];
        s->cc_ted.symbol_sync_low[0] = v;
        /* High Nyquist band edge filter */
        v = s->cc_ted.symbol_sync_high[0]*s->cc_ted.high_band_edge_coeff[0] + s->cc_ted.symbol_sync_high[1]*s->cc_ted.high_band_edge_coeff[1] + sample.re;
        s->cc_ted.symbol_sync_high[1] = s->cc_ted.symbol_sync_high[0];
        s->cc_ted.symbol_sync_high[0] = v;

        /* Put things into the equalization buffer at T/2 rate. The symbol synchcronisation
           will fiddle the step to align this with the symbols. */
        if (s->eq_put_step <= 0)
        {
            /* CC channel AGC not needed — Phase 2 works with fixed scaling.
               Primary channel AGC is re-enabled in primary_channel_rx(). */
            s->eq_put_step += RX_PULSESHAPER_2400_COEFF_SETS*40/(3*2);
            /* Same carrier logic as above */
            if (s->calling_party != s->v90_mode)
            {
#if defined(SPANDSP_USE_FIXED_POINT)
                qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
                qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
            }
            else
            {
#if defined(SPANDSP_USE_FIXED_POINT)
                qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
                qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
            }
            /*endif*/
#if defined(SPANDSP_USE_FIXED_POINT)
            //sample.im = (qq*(int32_t) s->agc_scaling) >> 15;
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexi16(s->carrier_phase);
#else
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexf(s->carrier_phase);
#endif
            zz.re = sample.re*z.re - sample.im*z.im;
            zz.im = -sample.re*z.im - sample.im*z.re;
            process_cc_half_baud(s, &zz);

        }
        /*endif*/
        /* Use CC carrier phase rate, not V34 primary channel rate.
           For answerer RX: caller CC at 1200 Hz.
           For caller RX: answerer CC at 2400 Hz. */
#if defined(SPANDSP_USE_FIXED_POINT)
        dds_advance(&s->carrier_phase, s->cc_carrier_phase_rate);
#else
        dds_advancef(&s->carrier_phase, s->cc_carrier_phase_rate);
#endif
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void process_primary_symbol(v34_rx_state_t *s, const complexf_t *sym)
{
    float energy;
    uint32_t ang1;
    uint32_t ang2;
    uint32_t ang3;
    int data_bits;
    int phase3_abs_bits;
    int bits[4];
    int i;
    mp_t mp;
    v34_state_t *t;
    complexf_t eq_target;

    if (s->qam_report)
        s->qam_report(s->qam_user_data, sym, NULL, s->qam_sample_time);

    if (V34_TRACE_DIAGNOSTICS
        && (s->stage == V34_RX_STAGE_PHASE3_TRAINING || s->stage == V34_RX_STAGE_PHASE3_WAIT_S)
        && (s->duration == 0 || (s->duration % 256) == 0))
    {
        float sym_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
        fprintf(stderr,
                "[V34 RAW] primary_half_baud: stage=%d dur=%d sym=(%.4f,%.4f) mag=%.4f agc=%.5f\n",
                s->stage, s->duration, (double)sym->re, (double)sym->im,
                (double)sym_mag, (double)s->agc_scaling);
    }
    /*endif*/

    /* Opt-in dump of the equalized symbols this stage decides on.  Ja is
       carried on the 4-point training constellation, so the distance from
       these to the nearest constellation point is the direct measure of
       whether the symbol decisions -- the last of the four anchors in
       docs/v90_phase3_s_and_rbs_false_positive.md 35b -- are sound.  Nothing
       else in Phase 3 exposes them: the existing dumps are TRN and Phase 4.
       Two floats per symbol, opt-in via ME_V90_JA_SYM_DUMP. */
    if (s->stage == V34_RX_STAGE_PHASE3_WAIT_S)
    {
        static FILE *ja_sym_fp = NULL;
        static int ja_sym_tried = 0;

        if (!ja_sym_tried)
        {
            const char *path = V34_DIAG_GETENV("ME_V90_JA_SYM_DUMP");

            ja_sym_tried = 1;
            if (path  &&  path[0] != '\0')
                ja_sym_fp = fopen(path, "wb");
            /*endif*/
        }
        /*endif*/
        if (ja_sym_fp)
        {
            float rec[3];

            rec[0] = sym->re;
            rec[1] = sym->im;
            /* Sample clock at this symbol, so a gap in the SYMBOL stream can
               be told apart from a gap in the captured BITS: a hole in the
               bits with a continuous sample time means the capture dropped
               them, a jump here means the sample feed did. */
            rec[2] = (float) s->qam_sample_time;
            fwrite(rec, sizeof(float), 3, ja_sym_fp);
        }
        /*endif*/
    }
    /*endif*/

    /* Experiment (docs 35e): arm Phase 3 tracking without waiting for a TRN
       lock.  The adaptation that converges this stage's constellation is
       gated on phase3_tracking_armed, which is set only once the TRN
       hypothesis locks at >=70%.  In the V.90 digital role the peer enters
       JaTXMIT at the same instant it enters Phase 3 (measured: 772.931916 vs
       772.931933), so there is no TRN in this window at all and the lock has
       to be found on Ja itself -- which costs ~0.75 s of a peer budget of
       1.88 s before the equalizer starts converging. */

    /* Phase 3 S signal detection state machine */
    switch (s->stage)
    {
    case V34_RX_STAGE_PHASE3_WAIT_S:
        v34_rx_phase3_wait_s_symbol(s, sym);
        break;

    case V34_RX_STAGE_PHASE3_TRAINING:
        /* Phase 3 receiver conditioning:
           - acquire the PP start/phase against the known 48-symbol PP sequence
           - adapt equalizer directly against the aligned PP sequence
           - optionally refine for only the first 512T of TRN
           - then hold the equalizer steady until J/S handling takes over */
        {
        v34_state_t *t;

        t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
        if (s->hdx_await_trn_end)
        {
            /* 12.4.2.1, armed once the recipient has trained on PP and TRN.
               The move to the control channel is taken on the SILENCE that
               ends Phase 3 -- 12.4.1.1's 70 +/- 5 ms -- rather than on the
               INFOh TRN length, because the arithmetic gives the length of
               TRN and not the instant this receiver started counting it: the
               PP acquisition ahead of it takes a variable number of symbols,
               and at 2743 and 3000 baud that offset was enough to put the
               recipient on the control channel after the source's 32 symbols
               of PPh had already gone by, with nothing to lock to for the
               rest of the call.  The silence is unambiguous, needs no
               arithmetic, and is where the clause itself puts the boundary. */
            float hdx_mag2 = sym->re*sym->re + sym->im*sym->im;

            if (hdx_mag2 < HDX_TRN_END_SILENCE_MAG2)
            {
                if (++s->hdx_silence_bauds >= HDX_TRN_END_SILENCE_BAUDS)
                {
                    s->hdx_await_trn_end = false;
                    v34_condition_rx_for_pph(t, "12.4.2.1, on the silence ending the source's Phase 3");
                    return;
                }
                /*endif*/
            }
            else
            {
                s->hdx_silence_bauds = 0;
            }
            /*endif*/
        }
        /*endif*/
        /* Differential symbols must be measured in one consistent domain.
           last_sample is the previous equalizer output, so using the newest
           raw T/2 input here compared unrelated points and made TRN/Ja bits
           random even when the equalizer itself was usable. */
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        /* Absolute (direct-mapped) dibit, for V.34 10.1.3.8 TRN and the V.90
           9.4 CP hypotheses.  The +45 degree bias above belongs to the
           DIFFERENTIAL decode only: ang3 is a phase *difference*, which for
           4-point DPSK clusters on 0/90/180/270, so the bias moves the >>30
           boundaries off those cluster centres and onto the midpoints.
           ang1 is an ABSOLUTE angle, and the 4-point training constellation
           sits at 45/135/225/315 (training_constellation_4).  Applying the
           same bias here moved the boundaries ONTO the constellation points,
           so every training symbol landed on a decision boundary and sliced
           to a coin flip: measured against a known transmitter, TRN scored
           117/200 with the bias and 200/200 without it.  That kept TRN
           descrambling at ~54% ones, below the 70% lock gate, so Phase 3
           never locked and no V.90 Phase 4 CP frame ever demodulated.
           The absolute slicer's boundaries are the quadrant edges. */
        phase3_abs_bits = (int) (ang1 >> 30) & 0x3;
        s->duration++;
        if (!s->phase3_pp_started)
        {
            int phase;
            int best_phase;
            float best_score;
            float next_score;
            float mag;

            mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
            s->phase3_pp_corr_energy =
                PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr_energy + mag*mag;
            s->phase3_pp_corr_weight =
                PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr_weight + 1.0f;
            best_phase = 0;
            best_score = 0.0f;
            next_score = 0.0f;
            for (phase = 0;  phase < PP_PERIOD_SYMBOLS;  phase++)
            {
                complexf_t cand;
                float corr_mag;
                float denom;
                float score;

                cand = pp_symbols[(s->duration - 1 + phase)%PP_PERIOD_SYMBOLS];
                /* Correlate y with the conjugate of the known unit-magnitude
                   PP reference.  |correlation| is invariant to the arbitrary
                   carrier phase present when the primary demodulator starts. */
                s->phase3_pp_corr[phase].re =
                    PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr[phase].re
                    + sym->re*cand.re + sym->im*cand.im;
                s->phase3_pp_corr[phase].im =
                    PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr[phase].im
                    + sym->im*cand.re - sym->re*cand.im;
                corr_mag = sqrtf(s->phase3_pp_corr[phase].re*s->phase3_pp_corr[phase].re
                               + s->phase3_pp_corr[phase].im*s->phase3_pp_corr[phase].im);
                denom = sqrtf(s->phase3_pp_corr_energy*s->phase3_pp_corr_weight);
                score = (denom > 0.0001f) ? corr_mag/denom : 0.0f;
                s->phase3_pp_error[phase] = score;
                if (phase == 0 || score > best_score)
                {
                    next_score = best_score;
                    best_score = score;
                    best_phase = phase;
                }
                else if (score > next_score)
                {
                    next_score = score;
                }
                /*endif*/
            }
            /*endfor*/

            if (s->phase3_pp_phase == best_phase)
                s->phase3_pp_acquire_hits++;
            else
                s->phase3_pp_acquire_hits = 1;
            /*endif*/
            s->phase3_pp_phase = best_phase;
            s->phase3_pp_phase_score = (int) lrintf(1000.0f*best_score);
            if (s->phase3_pp_phase_score >= PHASE3_PP_ONSET_SCORE)
            {
                if (s->phase3_pp_onset < 0)
                    s->phase3_pp_onset = s->duration;
                /*endif*/
            }
            else if (s->phase3_pp_phase_score < PHASE3_PP_ONSET_SCORE/2)
            {
                s->phase3_pp_onset = -1;
            }
            /*endif*/

            if (s->duration <= 4 || (s->duration % PHASE3_PP_ACQUIRE_LOG_INTERVAL) == 0)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3 PP acquire baud %d: mag=%.3f data_bits=%d phase=%d score=%d hold=%d/%d\n",
                         s->duration, mag, data_bits,
                         s->phase3_pp_phase, s->phase3_pp_phase_score,
                         s->phase3_pp_acquire_hits, PHASE3_PP_ACQUIRE_HOLD_BAUDS);
            }
            /*endif*/

            if (s->duration >= PHASE3_PP_ACQUIRE_MIN_BAUDS
                && s->phase3_pp_phase_score >= PHASE3_PP_ACQUIRE_SCORE_MIN
                && s->phase3_pp_acquire_hits >= PHASE3_PP_ACQUIRE_HOLD_BAUDS)
            {
                int acquire_bauds;
                /* Do not arm PP conditioning while local Phase 3 TX is still in S.
                   In practice this avoids false PP locks on the S/!S interval.
                   However, in the external V.90 downstream path the local Phase 3
                   symbols are no longer driven by SpanDSP TX stage progression,
                   so this internal TX-stage guard can deadlock RX in
                   PHASE3_TRAINING forever. In V.90 mode, trust the Phase 3 RX
                   evidence and allow PP acquisition to proceed. */
                if (!s->v90_mode
                    && !t->tx.phase3_call_wait_j
                    && t->tx.stage < V34_TX_STAGE_FIRST_NOT_S)
                {
                    goto phase3_training_done;
                }
                /*endif*/
                /* Nor while a duplex answer modem is still sending its own
                   S-bar, PP and TRN: that PP can only be our own, coming
                   back through the line echo.  See
                   v34_rx_answerer_sending_own_phase3(). */
                if (v34_rx_answerer_sending_own_phase3(s))
                    goto phase3_training_done;
                /*endif*/

                acquire_bauds = s->duration;
                s->phase3_pp_started = 1;
                s->v90_p3_eye_released = false;
                {
                    float corr_mag;

                    corr_mag = sqrtf(s->phase3_pp_corr[s->phase3_pp_phase].re
                                   * s->phase3_pp_corr[s->phase3_pp_phase].re
                                   + s->phase3_pp_corr[s->phase3_pp_phase].im
                                   * s->phase3_pp_corr[s->phase3_pp_phase].im);
                    if (corr_mag > 0.0001f)
                    {
                        s->phase3_pp_rotation.re =
                            s->phase3_pp_corr[s->phase3_pp_phase].re/corr_mag;
                        s->phase3_pp_rotation.im =
                            s->phase3_pp_corr[s->phase3_pp_phase].im/corr_mag;
                    }
                    /*endif*/
                }
                /* Absorb the acquisition baud count into the phase offset so that
                   when duration resets to 0 the PP target index remains continuous:
                   during acquisition baud N, target = pp_symbols[(N-1+phase)%48].
                   After reset, conditioning baud 1 must use the same index as
                   acquisition baud (N+1), i.e. pp_symbols[(N+phase)%48].
                   With new_phase = (phase + acquire_bauds) % 48 and dur=1:
                   pp_symbols[(1-1+new_phase)%48] = pp_symbols[(acquire_bauds+phase)%48]. ✓ */
                /* PP is 288T (10.1.3.6) and the conditioning takes 232 of them, which
                   assumes PP start is declared a few dozen bauds after PP begins:
                   from correlation onset to detection measures 27-43 bauds on every
                   RasFinder call whose PP residual came out 0.2-0.4.  When the lock
                   is late -- an eye flip during acquisition resets it, and the score
                   then crosses its threshold only after the move -- the same 232
                   bauds run off the end of PP into TRN and train the equalizer
                   against the wrong reference: onset-to-detection 79-150 bauds on
                   every call with residual 0.50-1.06, and on
                   artifacts/rf-0929s-p10-3 no Ja hypothesis ever parsed on a line
                   that measures 31.6 dB offline.  So start the conditioning count
                   at the excess, keeping the PP index continuous; it then ends where
                   a timely detection would have.  V.90 digital receiver only: plain V.34
                   3200/21600 A-law, where every PP is timely, stops decoding its
                   payload with it. */
                s->phase3_pp_skip = 0;
                if (s->phase3_pp_onset >= 0
                    &&  v34_pp_onset_trim_enabled()
                    &&  ((s->v90_mode  &&  !s->calling_party
                          &&  !s->v90_v34_fallback  /* a V.34 call from Phase 3 on */)
                         ||  (!s->v90_mode  &&  v34_pp_onset_trim_enabled() >= 2)))
                {
                    int late = acquire_bauds - s->phase3_pp_onset - PHASE3_PP_ONSET_NOMINAL;

                    /* An onset stamped in the first few bauds of acquisition is
                       not a PP rising out of silence: the score was already
                       over threshold when this receiver started looking (a
                       correlation carried over from earlier signal), so
                       "onset N ago" is just how long it has been looking, and
                       trimming by it throws away PP that is there. */
                    if (s->v90_mode  ||  s->phase3_pp_onset >= PHASE3_PP_ONSET_MIN_RISE)
                    {
                        if (late > 0)
                            s->phase3_pp_skip = (late > PHASE3_PP_ONSET_MAX_SKIP)  ?  PHASE3_PP_ONSET_MAX_SKIP  :  late;
                        /*endif*/
                    }
                    /*endif*/
                }
                /*endif*/
                s->phase3_pp_phase = (s->phase3_pp_phase + acquire_bauds - s->phase3_pp_skip + 4*PP_PERIOD_SYMBOLS) % PP_PERIOD_SYMBOLS;
                /* vpcm_decode's VPCM_V90_PP_SWEEP sets this to grade each phase. */
                if (getenv("VPCM_V90_PP_PHASE"))
                {
                    int forced_phase = atoi(getenv("VPCM_V90_PP_PHASE"));

                    forced_phase %= PP_PERIOD_SYMBOLS;
                    if (forced_phase < 0)
                        forced_phase += PP_PERIOD_SYMBOLS;
                    s->phase3_pp_phase = forced_phase;
                }
                /*endif*/
                s->duration = s->phase3_pp_skip;
                memset(s->phase3_pp_lag8, 0, sizeof(s->phase3_pp_lag8));
                s->phase3_pp_obs = 0;
                s->phase3_pp_match = 0;
                /* Freeze AGC — the equalizer will now be conditioned with known
                   PP symbols at the current magnitude.  Freezing prevents AGC from
                   fighting the LMS equalizer adaptation. */
                s->agc_scaling_save = s->agc_scaling;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP start detected (phase=%d score=%d after %d bauds, locked %d, onset %d ago), starting supervised PP conditioning (agc frozen at %.6f)\n",
                         s->phase3_pp_phase, s->phase3_pp_phase_score, acquire_bauds, s->phase3_pp_acquire_hits, (s->phase3_pp_onset >= 0) ? acquire_bauds - s->phase3_pp_onset : -1, s->agc_scaling);
            }
            /*endif*/
        }
        else if (s->duration <= PHASE3_PP_TRAIN_BAUDS)
        {
            complexf_t pp_target;
            int idx;
            int prev;
            int pp_baud;

            pp_baud = s->duration;
            idx = (pp_baud - 1) & 7;
            prev = s->phase3_pp_lag8[idx];
            if (pp_baud > 8)
            {
                s->phase3_pp_obs++;
                if (data_bits == prev)
                    s->phase3_pp_match++;
                /*endif*/
            }
            /*endif*/
            s->phase3_pp_lag8[idx] = (uint8_t) data_bits;

            /* Scale PP target to match the actual equalizer output magnitude.
               The AGC + center-tap equalizer produces output at some magnitude
               that depends on signal power at AGC freeze time.  Track it with
               an EMA (stored in eq_target_mag) and scale the unit-magnitude PP
               reference to match, so LMS drives direction without fighting magnitude. */
            {
                float sym_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                float scale;

                if (!isfinite(sym_mag) || sym_mag <= 0.0f || sym_mag > PHASE3_PP_MAG_SANITY_MAX)
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3: PP conditioning symbol magnitude out of range (mag=%.3f), clamping target scale and resetting equalizer\n",
                             sym_mag);
                    sym_mag = 1.0f;
                    equalizer_reset(s);
                }
                /*endif*/
                if (pp_baud <= 1 + s->phase3_pp_skip)
                    s->eq_target_mag = sym_mag;
                else
                    s->eq_target_mag = 0.95f*s->eq_target_mag + 0.05f*sym_mag;
                /*endif*/
                scale = (s->eq_target_mag > 0.01f) ? s->eq_target_mag : 1.0f;
                pp_target = pp_symbols[(pp_baud - 1 + s->phase3_pp_phase)%PP_PERIOD_SYMBOLS];
                {
                    float re;

                    re = pp_target.re*s->phase3_pp_rotation.re
                       - pp_target.im*s->phase3_pp_rotation.im;
                    pp_target.im = pp_target.re*s->phase3_pp_rotation.im
                                 + pp_target.im*s->phase3_pp_rotation.re;
                    pp_target.re = re;
                }
                pp_target.re *= scale;
                pp_target.im *= scale;
                {
                    /* Residual between the equalized symbol and the known PP
                       reference.  PP is a fixed sequence, so this is a direct
                       measure of front-end health that does not depend on any
                       descrambler or hypothesis choice. */
                    float dre = sym->re - pp_target.re;
                    float dim = sym->im - pp_target.im;
                    float den = sqrtf(pp_target.re*pp_target.re
                                      + pp_target.im*pp_target.im);
                    if (den > 0.01f)
                    {
                        s->phase3_pp_resid_sum += sqrtf(dre*dre + dim*dim)/den;
                        s->phase3_pp_resid_count++;
                    }
                }
                v34_rx_tune_equalizer(s, sym, &pp_target);
                /* V.34 10.1.3.6 PP is not QPSK: equation 10-1 uses points at
                   multiples of 30 degrees.  The generic primary-training
                   carrier loop below snaps to the nearest four-point symbol,
                   so letting it run over PP makes it fight this supervised
                   target.  The 12.5.2 HDX primary resynchronization must
                   use this same PP detector: it has no later TRN interval
                   to repair a carrier/equalizer solution damaged by QPSK
                   decisions during PP.  Track against the known point here
                   and suppress the four-point loop for this interval. */
                if (s->v90_mode || s->hdx_primary_resync)
                {
                    float error = sym->im*pp_target.re - sym->re*pp_target.im;

                    s->v34_carrier_phase_rate += (int32_t) (s->carrier_track_i*error);
                    s->carrier_phase += (int32_t) (s->carrier_track_p*error);
                }
            }

            if (pp_baud == 1)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: conditioning on aligned PP sequence (%d bauds)\n",
                         PHASE3_PP_TRAIN_BAUDS);
            }
            /*endif*/
            if (pp_baud <= 4 || (pp_baud % PHASE3_PP_BAUD_LOG_INTERVAL) == 0)
            {
                float mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
                float pct = (s->phase3_pp_obs > 0)
                            ? (100.0f*s->phase3_pp_match/(float) s->phase3_pp_obs)
                            : 0.0f;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3 PP baud %d: mag=%.3f data_bits=%d lag8=%d/%d (%.1f%%) phase=%d score=%d\n",
                         pp_baud, mag, data_bits,
                         s->phase3_pp_match, s->phase3_pp_obs, pct,
                         s->phase3_pp_phase, s->phase3_pp_phase_score);
            }
            /*endif*/
            if (pp_baud == PHASE3_PP_TRAIN_BAUDS)
            {
                float pct = (s->phase3_pp_obs > 0)
                            ? (100.0f*s->phase3_pp_match/(float) s->phase3_pp_obs)
                            : 0.0f;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP mean residual %.3f over %d bauds\n",
                         (s->phase3_pp_resid_count > 0)
                             ? s->phase3_pp_resid_sum/s->phase3_pp_resid_count
                             : -1.0f,
                         s->phase3_pp_resid_count);
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP conditioning complete (lag8=%d/%d, %.1f%%, phase=%d, score=%d), refining with first %dT of TRN\n",
                         s->phase3_pp_match, s->phase3_pp_obs, pct,
                         s->phase3_pp_phase, s->phase3_pp_phase_score, PHASE3_TRN_REFINE_BAUDS);
                if (s->hdx_primary_resync)
                {
                    v34_state_t *t;

                    t = (v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx));
                    s->hdx_primary_resync = false;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - 12.5.2: PP complete; conditioning data decoder for B1\n");
                    v34_begin_rx_data(t);
                }
                /*endif*/
            }
            /*endif*/
        }
        else if (s->duration <= (PHASE3_PP_TRAIN_BAUDS + PHASE3_TRN_REFINE_BAUDS))
        {
            int h;
            int best_trn_h;
            int best_trn_score;
            int trn_refine_baud;

            best_trn_h = -1;
            best_trn_score = -1;
            trn_refine_baud = s->duration - PHASE3_PP_TRAIN_BAUDS;
            {
                static const char *p3_dump_path = NULL;
                float eq_main;
                float eq_e = v34_rx_eq_tap_energy(s, &eq_main);

                v34_rx_dump_training_symbol("V34_P3TRN_SYM_DUMP", &p3_dump_path,
                                         s->calling_party,
                                         s->phase3_trn_bits,
                                         sym->re, sym->im,
                                         (long) power_meter_current(&s->power),
                                         eq_e, eq_main,
                                         s->eq_put_step,
                                         s->total_baud_timing_correction);
            }
            for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
            {
                int raw_sym;
                uint32_t reg;
                int d0;
                int d1;

                /* V.34 10.1.3.8 TRN uses direct (absolute) mapping. */
                raw_sym = v34_rx_map_phase4_raw_bits(phase3_abs_bits, h);
                reg = s->phase3_trn_scramble[h];
                d0 = v34_rx_descramble_reg(&reg, s->scrambler_tap, raw_sym & 1);
                d1 = v34_rx_descramble_reg(&reg, s->scrambler_tap, (raw_sym >> 1) & 1);
                s->phase3_trn_scramble[h] = reg;
                s->phase3_trn_one_count[h] += (uint16_t) (d0 + d1);
                if (s->phase3_trn_one_count[h] > best_trn_score)
                {
                    best_trn_h = h;
                    best_trn_score = s->phase3_trn_one_count[h];
                }
                /*endif*/
            }
            /*endfor*/
            s->phase3_trn_bits += 2;
            {
                float trn_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                if (isfinite(trn_mag) && trn_mag > 0.0f) {
                    s->phase3_trn_mag_sum += trn_mag;
                    s->phase3_trn_mag_count++;
                    /* Distance to the nearest QPSK point, normalised. */
                    {
                        float a = fabsf(sym->re);
                        float b = fabsf(sym->im);
                        float r = trn_mag*0.7071068f;
                        float d = sqrtf((a - r)*(a - r) + (b - r)*(b - r));
                        s->phase3_trn_resid_sum += d/trn_mag;
                        s->phase3_trn_resid_count++;
                    }
                }
            }
            if (trn_refine_baud == 1)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP complete, using first %dT of TRN for equalizer refinement\n",
                         PHASE3_TRN_REFINE_BAUDS);
            }
            /*endif*/
            if (s->phase3_trn_bits >= 256  &&  best_trn_h >= 0)
            {
                int score_pct;

                score_pct = (100*best_trn_score + (s->phase3_trn_bits/2))/s->phase3_trn_bits;
                if (score_pct >= 70
                    &&
                    (s->phase3_trn_lock_hyp < 0  ||  score_pct > s->phase3_trn_lock_score))
                {
                    s->phase3_trn_lock_hyp = best_trn_h;
                    s->phase3_trn_lock_score = score_pct;
                    s->phase3_j_lock_hyp = v34_rx_j_hint_enabled() ? best_trn_h : -1;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 TRN refine: lock hint hyp=%d ones=%d/%d (%d%%)\n",
                             best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct);
                }
                else if ((s->phase3_trn_bits % 512) == 0)
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 TRN refine: best hyp=%d ones=%d/%d (%d%%) tap=%d\n",
                             best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct,
                             s->scrambler_tap);
                }
                /*endif*/
            }
            /*endif*/
            if (trn_refine_baud == PHASE3_TRN_REFINE_BAUDS)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: TRN mean distance to nearest 4-point %.3f over %d symbols\n",
                         (s->phase3_trn_resid_count > 0)
                             ? s->phase3_trn_resid_sum/s->phase3_trn_resid_count
                             : -1.0f,
                         s->phase3_trn_resid_count);
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: first %dT of TRN processed; equalizer frozen, waiting for J-handling stage\n",
                         PHASE3_TRN_REFINE_BAUDS);
                /* This transition wipes phase3_ja_capture_hyp[] below, which is
                   the ONLY input to the V.90 DIL descriptor parser.  If the
                   receiver re-enters PHASE3_TRAINING mid-Ja and comes back, the
                   descriptor bits captured so far are discarded and the parser
                   restarts from nothing -- so say how much is being thrown
                   away.  Live 2026-08-12: the peer's Ja carries exactly one
                   CRC-valid descriptor copy, so losing the capture across it
                   loses the whole call's DIL. */
                /* Report the longest hypothesis, not hyp 0.  All 24 fill
                   together today, but keying the report on one of them means a
                   fill path that ever skips hyp 0 reports "nothing discarded"
                   while discarding everything -- which is indistinguishable
                   from the capture never having started, and cost a session
                   telling those two apart.  The DIL descriptor sits ~14.3k bits
                   into this peer's Ja and needs ~16.3k to parse, so print the
                   count unconditionally: "0" here is itself the finding. */
                {
                    int cap_h;
                    int cap_max = 0;
                    int cap_max_h = -1;

                    for (cap_h = 0;  cap_h < MP_HYPOTHESIS_COUNT;  cap_h++)
                    {
                        if (s->phase3_ja_capture_hyp_len[cap_h] > cap_max)
                        {
                            cap_max = s->phase3_ja_capture_hyp_len[cap_h];
                            cap_max_h = cap_h;
                        }
                        /*endif*/
                    }
                    /*endfor*/
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3: DISCARDING %d captured Ja bits (longest hyp=%d) on re-entry to WAIT_S\n",
                             cap_max, cap_max_h);
                }
                if (!s->duplex)
                {
                    /* V.34 12.3.2.2 ends the recipient's Phase 3 at PP and
                       TRN -- there is no J in half-duplex -- and 12.4.2.1 has
                       it condition its receiver to detect signal PPh.  Falling
                       into PHASE3_WAIT_S ran the duplex J/Ja scanners over the
                       rest of the source's TRN and the recipient never
                       listened to the control channel at all.

                       The move must wait for the END of TRN, not for the
                       refinement window: a control channel demodulator pointed
                       at a second of primary channel TRN has its band edge
                       timing recovery locked to noise by the time PPh -- all
                       32 symbols of it -- arrives.  The length is the one this
                       modem asked for in INFOh (Table 22 bits 15:21, 35 ms
                       units), so it is arithmetic rather than detection. */
                    s->hdx_await_trn_end = true;
                    s->hdx_silence_bauds = 0;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3: half-duplex recipient trained; waiting for the end of TRN before the control channel (12.4.2.1)\n");
                    return;
                }
                /*endif*/
                s->stage = V34_RX_STAGE_PHASE3_WAIT_S;
                s->duration = 0;
                s->s_detect_count = 0;
                s->bit_count = 0;
                s->s_window = 0;
                s->phase3_s_alt_window = 0;
                s->phase3_s_alt_count = 0;
                s->phase3_s_stable_windows = 0;
                s->phase3_s_dom_windows = 0;
                s->phase3_s_dom_symbol = -1;
                s->phase3_s_fired_symbol = -1;
                s->phase3_s_detect_armed = false;
                memset(s->phase3_s_ring, 0, sizeof(s->phase3_s_ring));
                memset(s->phase3_s_mag_ring, 0, sizeof(s->phase3_s_mag_ring));
                memset(s->phase3_s_counts, 0, sizeof(s->phase3_s_counts));
                s->phase3_s_pos = 0;
                memset(s->phase3_j_scramble, 0, sizeof(s->phase3_j_scramble));
                memset(s->phase3_j_stream, 0, sizeof(s->phase3_j_stream));
                memset(s->phase3_j_prev_z, 0, sizeof(s->phase3_j_prev_z));
                memset(s->phase3_j_prev_valid, 0, sizeof(s->phase3_j_prev_valid));
                memset(s->phase3_j_win, 0, sizeof(s->phase3_j_win));
                s->phase3_j_bits = 0;
                s->phase3_j_lock_hyp = -1;
                s->phase3_j_trn16 = -1;
                s->phase3_j_candidate_hyp = -1;
                s->phase3_j_candidate_phase = -1;
                s->phase3_j_candidate_pat = -1;
                s->phase3_j_candidate_count = 0;
                s->phase3_j_candidate_last_bits = 0;
                memset(s->phase3_ja_scramble, 0, sizeof(s->phase3_ja_scramble));
                memset(s->phase3_ja_prev_z, 0, sizeof(s->phase3_ja_prev_z));
                memset(s->phase3_ja_prev_valid, 0, sizeof(s->phase3_ja_prev_valid));
                s->phase3_ja_bits = 0;
                s->phase3_ja_hyp = -1;
                memset(s->phase3_ja_capture, 0, sizeof(s->phase3_ja_capture));
                s->phase3_ja_capture_len = 0;
                memset(s->phase3_ja_capture_hyp, 0, sizeof(s->phase3_ja_capture_hyp));
                memset(s->phase3_ja_capture_hyp_len, 0, sizeof(s->phase3_ja_capture_hyp_len));
                memset(s->phase3_ja_capture_hyp_raw, 0, sizeof(s->phase3_ja_capture_hyp_raw));
                memset(s->phase3_ja_capture_hyp_raw_len, 0, sizeof(s->phase3_ja_capture_hyp_raw_len));
                phase3_trn_hyp_reset(s);
                s->phase3_trn_mag_sum = 0.0f;
                s->phase3_trn_mag_count = 0;
            }
            /*endif*/
        }
        else
        {
            int h;
            int raw_sym;
            int lock_h = (s->phase3_trn_lock_hyp >= 0
                          && s->phase3_trn_lock_hyp < MP_HYPOTHESIS_COUNT)
                         ? s->phase3_trn_lock_hyp
                         : s->phase3_j_lock_hyp;

            if (lock_h < 0 || lock_h >= MP_HYPOTHESIS_COUNT)
                goto phase3_training_done;

            h = lock_h;
            raw_sym = v34_rx_map_phase4_raw_bits(data_bits, h);
            if (s->phase3_ja_prev_valid[h])
            {
                int in_sym;
                uint32_t reg;
                int b0;
                int b1;

                in_sym = raw_sym;
                reg = s->phase3_ja_scramble[h];
                b0 = v34_rx_descramble_reg(&reg, s->scrambler_tap, in_sym & 1);
                b1 = v34_rx_descramble_reg(&reg, s->scrambler_tap, (in_sym >> 1) & 1);
                s->phase3_ja_scramble[h] = reg;
                if (s->put_aux_bit)
                {
                    s->put_aux_bit(s->put_aux_bit_user_data, b0);
                    s->put_aux_bit(s->put_aux_bit_user_data, b1);
                }
                /* Persist Ja bits for offline analyzers even when no aux callback is armed. */
                v34_ja_append(s->phase3_ja_capture, s->phase3_ja_capture_len++, b0);
                v34_ja_append(s->phase3_ja_capture, s->phase3_ja_capture_len++, b1);
                s->phase3_ja_bits += 2;
                s->phase3_ja_hyp = h;
                if (s->phase3_ja_bits == 2 || (s->phase3_ja_bits % 256) == 0)
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 Ja capture: emitted %d bits using hyp=%d\n",
                             s->phase3_ja_bits, h);
                }
                /*endif*/
            }
            /*endif*/
            s->phase3_ja_prev_z[h] = (uint8_t) raw_sym;
            s->phase3_ja_prev_valid[h] = 1;
        }
        phase3_training_done:
        ;
        }
        break;

    case V34_RX_STAGE_PHASE3_DONE:
        break;

    case V34_RX_STAGE_PHASE4_S:
        /* Phase 4: detect the far end's S signal.  10.1.3.7 alternates between
           point 0 and point 0 rotated counterclockwise by 90 degrees, so every
           baud of S is a +/-90 degree step -- data_bits 1 or 3 with the +45
           degree bias below, never 2.  This counted data_bits == 2, a 180
           degree step, which S never produces: the detector was firing on
           whatever noise happened to land in that quadrant, which is why it
           needed 128 bauds and only 20 of 32 to declare.  Count the steps S
           actually makes.  (The transmitter's own comment already said so --
           "the same absolute-point generator as Phase 3 S rather than the old
           180 degree differential sequence" -- but this half was never
           brought along.) */
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        s->duration++;

        if (s->hdx_primary_resync && !s->hdx_s_silence_seen)
        {
            float power = sym->re*sym->re + sym->im*sym->im;
            if (power < 0.01f)
                s->hdx_silence_bauds++;
            else
                s->hdx_silence_bauds = 0;
            /* 12.5.1 supplies 70 ms of silence. Sixteen low-energy
               symbols establish its presence after the CC filter tail;
               this is a detector qualification, not an added protocol delay. */
            if (s->hdx_silence_bauds >= 16
                && s->duration >= (baud_rate_parameters[s->baud_rate].baud_rate*65)/1000)
            {
                s->hdx_s_silence_seen = true;
                s->eye_n = s->eye_votes = 0;
                s->eye_on_sum = s->eye_off_sum = 0.0f;
                s->s_detect_count = 0;
                s->s_window = 0;
                s->phase4_s_last_step = -1;
            }
        }

        /* 12.6.3's four CC tail symbols can ring through the primary
           receive filter. The following 12.5.1 silence breaks that evidence:
           discard its S votes and junction history on a below-threshold
           symbol instead of carrying them into a phase change in silence. */
        if (s->hdx_primary_resync
            && (sym->re*sym->re + sym->im*sym->im < 0.01f
                || s->last_sample.re*s->last_sample.re
                   + s->last_sample.im*s->last_sample.im < 0.01f))
        {
            s->s_window = 0;
            s->s_detect_count = 0;
            s->phase4_s_last_step = -1;
        }

        /* Sliding window: shift in new bit, shift out old */
        {
            int idx = (s->duration - 1) & 31;  /* circular index 0-31 */
            int old_was_2 = (s->duration > 32) ? ((s->s_window >> idx) & 1) : 0;
            int new_is_2 = (data_bits == 1  ||  data_bits == 3) ? 1 : 0;

            /* 12.5.1 starts with 70 ms of silence. Numerical phase steps
               from an empty filter are not evidence of S; both symbols in
               the differential decision must contain a signal. */
            if (s->hdx_primary_resync
                && (!s->hdx_s_silence_seen
                    || sym->re*sym->re + sym->im*sym->im < 0.01f
                    || s->last_sample.re*s->last_sample.re
                       + s->last_sample.im*s->last_sample.im < 0.01f))
                new_is_2 = 0;
            /*endif*/
            if (new_is_2)
                s->s_window |= (1u << idx);
            else
                s->s_window &= ~(1u << idx);

            s->s_detect_count += new_is_2 - old_was_2;
        }

        if (s->duration <= 10 || (s->duration % 500) == 0)
        {
            float mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4 S baud %d: mag=%.3f data_bits=%d win=%d/32\n",
                     s->duration, mag, data_bits, s->s_detect_count);
        }
        /* Dump raw I/Q constellation for first 32 bauds to diagnose signal quality */
        #if V34_DEBUG_IQ_LOG
        if (s->duration <= 32)
        {
            uint32_t ang_abs = arctan2(sym->im, sym->re);
            float deg = (float)ang_abs / (4294967296.0f / 360.0f);
            float deg_diff = (float)ang3 / (4294967296.0f / 360.0f);
            fprintf(stderr, "[IQ] baud=%d re=%.4f im=%.4f ang=%.1f diff=%.1f data=%d\n",
                    s->duration, sym->re, sym->im, deg, deg_diff, data_bits);
        }
        #endif

        /* Pin the handoff to the S-to-S-bar junction rather than to whenever a
           window threshold happens to cross.  10.1.3.7 alternates S between two
           points 90 degrees apart and starts S-bar 180 degrees from where S
           ended, so the differential steps run ...+90,-90,+90 then *two* +90s
           across the junction before resuming.  That repeat is the only place
           the alternation breaks, it is where the spec puts the boundary, and
           it does not move with the law or the symbol rate -- unlike a
           threshold, which was landing at a different point in S for every row
           of the matrix. */
        if (s->phase4_s_bar_left < 0
            &&
            s->duration >= 64
            &&
            s->s_detect_count >= 24
            && (!s->hdx_primary_resync
                || (s->hdx_s_silence_seen && sym->re*sym->re + sym->im*sym->im >= 0.01f
                    && s->last_sample.re*s->last_sample.re
                       + s->last_sample.im*s->last_sample.im >= 0.01f))
            &&
            (data_bits == 1  ||  data_bits == 3)
            &&
            data_bits == s->phase4_s_last_step)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S-to-S-bar junction at baud %d (win=%d/32); "
                     "16T of S-bar to go\n",
                     s->duration, s->s_detect_count);
            s->phase4_s_bar_left = 16;
        }
        /*endif*/
        if (data_bits == 1  ||  data_bits == 3)
            s->phase4_s_last_step = data_bits;
        /*endif*/
        if (s->phase4_s_bar_left > 0  &&  --s->phase4_s_bar_left == 0)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S-bar complete at baud %d, starting TRN\n",
                     s->duration);
            s->duration = 128;
            s->s_detect_count = 32;
        }
        /*endif*/
        if ((!s->hdx_primary_resync && s->duration >= 128 && s->s_detect_count >= 24)
            ||
            s->phase4_s_bar_left == 0)
        {
            if (s->hdx_primary_resync)
            {
                /* V.34 12.5.2 has PP immediately after S/S-bar and B1
                   immediately after PP; there is no TRN/MP exchange. */
                s->stage = V34_RX_STAGE_PHASE3_TRAINING;
                s->duration = 0;
                phase3_pp_reset(s);
                s->phase3_pp_resid_sum = 0.0f;
                s->phase3_pp_resid_count = 0;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - 12.5.2: S/S-bar complete; acquiring PP before B1\n");
                break;
            }
            /*endif*/
            /* S signal confirmed.  Now transition to S-bar detection.
               We skip explicit S-bar detection since we can't reliably
               distinguish S-bar from S with this demodulator quality.
               Instead, wait a fixed time for the caller's S-bar(16T) + TRN(≥512T)
               to pass, then go straight to MP detection. */
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S signal confirmed (baud %d, win=%d/32), "
                     "waiting for S-bar + TRN\n",
                     s->duration, s->s_detect_count);
            s->stage = V34_RX_STAGE_PHASE4_TRN;
            s->duration = 0;
            s->scramble_reg = 0;
            phase4_j_detector_reset(s);
            v34_rx_phase4_trn_hyp_reset(s);
            if (v34_rx_is_v34_call_modem(s))
            {
                /* Caller-side Phase 4 does not wait for a far-end J':
                   after detecting the answerer's S/S-bar handoff, the far end
                   is already in TRN. Mark the J' gate as satisfied so Phase 4
                   TRN scoring can begin immediately instead of timing out while
                   waiting for a symbol pattern that the answerer never sends. */
                s->phase4_j_seen = 1;
                s->phase4_trn_after_j = 0;
                s->phase4_j_lock_hyp = s->phase3_j_lock_hyp;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: caller detected answerer S handoff; starting direct TRN scoring (phase3 hyp=%d)\n",
                         s->phase3_j_lock_hyp);
            }
            /*endif*/
        }
        else if (s->duration >= 2048)
        {
            /* If S wasn't confidently detected within a long guard interval,
               don't stall forever in Phase 4 S detection. Advance to TRN/MP
               search so the handshake can continue. */
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S detect timeout (%d bauds, win=%d/32), "
                     "forcing TRN/MP search\n",
                     s->duration, s->s_detect_count);
            s->stage = V34_RX_STAGE_PHASE4_TRN;
            s->duration = 0;
            s->scramble_reg = 0;
            phase4_j_detector_reset(s);
            v34_rx_phase4_trn_hyp_reset(s);
            if (v34_rx_is_v34_call_modem(s))
            {
                s->phase4_j_seen = 1;
                s->phase4_trn_after_j = 0;
                s->phase4_j_lock_hyp = s->phase3_j_lock_hyp;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: caller forcing direct TRN scoring after S timeout (phase3 hyp=%d)\n",
                         s->phase3_j_lock_hyp);
            }
            /*endif*/
        }
        break;

    case V34_RX_STAGE_PHASE4_S_BAR:
        /* Phase 4: S-bar is 16T. After S-bar, TRN begins. */
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        s->duration++;

        if (s->duration >= 16)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S-bar complete (%d bauds), starting TRN detection\n",
                     s->duration);
            s->stage = V34_RX_STAGE_PHASE4_TRN;
            s->duration = 0;
            s->scramble_reg = 0;
            phase4_j_detector_reset(s);
            v34_rx_phase4_trn_hyp_reset(s);
            if (v34_rx_is_v34_call_modem(s))
            {
                s->phase4_j_seen = 1;
                s->phase4_trn_after_j = 0;
                s->phase4_j_lock_hyp = s->phase3_j_lock_hyp;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: caller finished S-bar; starting direct TRN scoring (phase3 hyp=%d)\n",
                         s->phase3_j_lock_hyp);
            }
            /*endif*/
        }
        break;

    case V34_RX_STAGE_PHASE4_TRN:
        v34_rx_phase4_trn_symbol(s, sym);
        break;

    case V34_RX_STAGE_V90_CP:
    case V34_RX_STAGE_PHASE4_MP:
        /* These stages share only the V.34 primary-channel DQPSK front end
           and hypothesis acquisition. V34_RX_STAGE_V90_CP hands the recovered
           bitstream to the project-owned V.90 Table-14 framer; the ordinary
           MP stage retains V.34 frame lengths, acknowledgements, timeouts and
           the E transition. Keeping the protocol states distinct is important:
           V.90 9.4.1.1 requires CPt reception while the digital modem sends Ri,
           without the preceding V.34 J'/TRN-to-MP sequencing. */
        {
            int locked_this_symbol;
            int h;
            int expected_mp_type;
            int abs_bits;
            int mp_decode_domain;
            int v90_cp_rx;

        v90_cp_rx = (s->stage == V34_RX_STAGE_V90_CP);
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        abs_bits = (int) ((ang1 + DDS_PHASE(45.0f)) >> 30) & 0x3;
        mp_decode_domain = s->mp_phase4_force_abs_active ? 1 : s->mp_phase4_domain;

        /* V34_MP_RX_DUMP: one line per Phase 4 symbol -- the differential and
           absolute quadrant decisions and the symbol magnitude.  10.1.3.9 leaves
           nothing to search once the constellation is known from J, so when MP
           will not decode the question is which interpretation of these dibits is
           right, and that is settled offline against the frame CRC rather than by
           another hypothesis sweep.  Neither the 17-bit all-ones MP preamble nor
           a TRN of scrambled ones can discriminate the bit order, so the CRC is
           the only oracle there is. */
        {
            static int mp_dump_checked = 0;
            static FILE *mp_dump = NULL;

            if (!mp_dump_checked)
            {
                const char *path = V34_DIAG_GETENV("V34_MP_RX_DUMP");

                if (path  &&  *path)
                    mp_dump = fopen(path, "w");
                /*endif*/
                mp_dump_checked = 1;
            }
            /*endif*/
            if (mp_dump)
            {
                /* Fifth column is the ABSOLUTE angle in degrees.  The
                   4-point Phase-4 constellation sits on the 45-degree family,
                   so the distance from it -- min over k of |ang - (45+90k)|
                   -- is a per-symbol read on whether the receiver is holding
                   the constellation, which the magnitude alone cannot give
                   (a diverged equalizer can hold a steady modulus).
                   tools/v34_mp_offline.py indexes columns 1 and 2 only. */
                /* Columns 6-10 are the front end, not the decision: the
                   AGC scaling, the equalizer's level estimate, the carrier
                   loop's frequency, the input power meter and the main
                   equalizer tap's magnitude.  A startup CP frame that
                   decodes and is then followed by one that degrades
                   PROGRESSIVELY is a loop walking off, and the decision
                   columns alone cannot say which one -- the same read that
                   V90_RENEG_SYM_DUMP gives the 9.6 window, which does not
                   run here because it sits inside the streamed branch. */
                /* Keep the original ten columns stable.  The sample clock,
                   timing correction and half-symbol phase anchor a decision
                   to its own input, rather than inferring time from duration
                   (which is reset at each protocol stage). */
                fprintf(mp_dump, "%d %d %d %.4f %.2f %.6f %.5f %.4f %ld %.5f %d %d %d\n",
                        s->duration, data_bits, abs_bits,
                        sqrtf(sym->re*sym->re + sym->im*sym->im),
                        (double) (180.0f/3.14159265f)*atan2f(sym->im, sym->re),
                        (double) s->agc_scaling,
                        (double) s->eq_target_mag,
                        (double) dds_frequencyf(s->v34_carrier_phase_rate),
                        (long) power_meter_current(&s->power),
                        (double) sqrtf(
                            s->eq_coeff[V34_EQUALIZER_PRE_LEN].re
                                *s->eq_coeff[V34_EQUALIZER_PRE_LEN].re
                          + s->eq_coeff[V34_EQUALIZER_PRE_LEN].im
                                *s->eq_coeff[V34_EQUALIZER_PRE_LEN].im),
                        s->qam_sample_time,
                        s->total_baud_timing_correction,
                        s->baud_half);
                fflush(mp_dump);
            }
            /*endif*/
        }

        /* Decision-aided carrier acquisition through the CP/MP stretch.
           The receiver reaches Phase 4 with no coherent lock (the TRN/MP
           "locks" are bit-domain hypothesis latches; CP/MP decode survives
           on differential dibits alone).  Data mode needs true coherence,
           and this window is the last chance to acquire it: the analogue
           modem's differential encoder is initialized to zero at the first
           CPt (8.5.2/V.90), the 4-point constellation sits on the 45-degree
           family, and once the hypothesis machinery locks, the differential
           dibit decisions here are the same ones that go on to CRC-validate
           whole CP frames -- reliable known data.  Integrate those quadrant
           increments into an expected absolute angle (seeded by snapping the
           received angle onto the 45-degree family, leaving only the
           90-degree ambiguity that V.34's differential quadrant bits absorb
           in data mode) and drive the existing carrier loop with the angle
           error, so the constellation is pulled onto the true grid before
           E/B1 hands over to the DATA slicer. */
        {
            float da_sym_mag2 = sym->re*sym->re + sym->im*sym->im;

            /* V.90 9.6: the decision-aided derotator and its data-aided LMS
               are driven by the V.34 MP hypothesis machinery, and while the
               renegotiation's CP is being STREAMED that machinery decodes
               nothing -- the domain, dibit transform, scrambler tap and bit
               order are all fixed by 8.5.2/10.1.3.3, which is why
               v90_cp_stream exists.  What it still does is lock on Figure 8's
               SCR, whose descrambled 18/18 ones satisfy the preamble gate for
               the whole 2.5 s of it, get rejected by the Table-14 framer, and
               lock again -- and every one of those re-locks RE-SEEDS the
               derotator, which snaps the current symbol onto the 45-degree
               family and then drags the taps there through v34_rx_tune_equalizer().
               Measured on artifacts/reneg-ab-225015Z/reneg-r3, whose first CP
               window decodes 3 of 5 frames on a steady line: the unwrapped
               residual carrier phase is FLAT to 0.002 deg/symbol between
               events and steps by +25, +24, +27, +25, -20 and +23 degrees at
               six instants, each one a re-lock.  So the loop is not tracking
               the channel here, it is stepping the constellation around on
               its own decisions, so it is off while CP is streamed. */
            if (s->v90_cp_stream)
            {
                s->phase4_da_active = 0;
            }
            else if (s->mp_hypothesis < 0)
            {
                s->phase4_da_active = 0;
            }
            else if (da_sym_mag2 < 0.04f)
            {
                /* The analogue modem goes quiet for seconds between CPt and
                   the data-mode CP (9.4.2.2/V.90 makes SCR optional).  A
                   zero symbol has arctan2(0,0) = constant angle, so the
                   tracker "locks" perfectly on nothing and the stale seed
                   survives to the signal return.  Suspend; V34_DA_HOLD_AFTER_
                   SILENCE=1 additionally freezes the CPt-era solution through
                   the (16-point-suspect) second CP stretch instead of
                   re-acquiring on it -- the channel is static, so the CPt
                   solution should carry to DATA. */
                s->phase4_da_active = 0;
                if (s->phase4_da_seeded && getenv("V34_DA_HOLD_AFTER_SILENCE"))
                    s->phase4_da_seeded = 2;    /* 2 = hold: no more updates */
            }
            else if (s->phase4_da_seeded == 2)
            {
                /* Held: CPt-era derot + equalizer frozen through to DATA. */
            }
            else if (!s->phase4_da_active)
            {
                /* Seed the derotator so the corrected angle lands on the
                   45-degree constellation family (90-degree family ambiguity
                   is absorbed by the data differential quadrant bits), and
                   start the expected-angle integrator there. */
                uint32_t ang1c = ang1 - s->phase4_da_derot;
                uint32_t tmp = ang1c - DDS_PHASE(45.0f);
                uint32_t quadrant = (tmp + 0x20000000U) >> 30;
                uint32_t snapped = (quadrant << 30) + DDS_PHASE(45.0f);

                s->phase4_da_derot += (uint32_t) ((int32_t) (ang1c - snapped));
                s->phase4_da_expected_ang = snapped;
                s->phase4_da_active = 1;
                s->phase4_da_seeded = 1;
            }
            else
            {
                int32_t err_i;

                s->phase4_da_expected_ang += ((uint32_t) data_bits) << 30;
                err_i = (int32_t) ((ang1 - s->phase4_da_derot)
                                   - s->phase4_da_expected_ang);
                /* Zero-delay loop: quarter-deadbeat is stable and follows
                   the CMA random walk with a ~4-baud time constant. */
                s->phase4_da_derot += err_i/4;
                /* Data-aided LMS: with the transmitted symbol known, train
                   the equalizer to the true zero-ISI solution.  CMA only
                   restores the modulus; its residual ISI is invisible on
                   constant-envelope signals but is exactly what smears the
                   multi-amplitude data constellation into ringless mush.
                   Target = the known 4-point symbol at CMA's unit radius,
                   rotated back into the equalizer-output domain. */
                {
                    float tang = (float) (uint32_t) (s->phase4_da_expected_ang
                                                     + s->phase4_da_derot)
                               * (float) (3.14159265358979/2147483648.0);
                    complexf_t da_target;

                    da_target.re = cosf(tang);
                    da_target.im = sinf(tang);
                    v34_rx_tune_equalizer(s, sym, &da_target);
                }
                if (V34_DIAG_GETENV("V34_DA_TRACK_LOG") && (s->duration % 256) == 0)
                {
                    fprintf(stderr,
                            "[DA] baud=%d err=%.1fdeg derot=%.1fdeg timing_corr=%d baud_phase=%.1f\n",
                            s->duration,
                            (float) err_i*(360.0f/4294967296.0f),
                            (float) s->phase4_da_derot*(360.0f/4294967296.0f),
                            s->total_baud_timing_correction,
                            (float) s->pri_ted.baud_phase);
                }
            }
        }

        /* ME_V34_DUMP_MP_DIBITS (diagnostic, 2026-07-19): dumps the raw
           diff/abs dibit per baud for offline replay -- e.g. brute-forcing
           every (hypothesis, domain, tap, order) combination against the
           actual captured stream with a properly continuous self-
           synchronizing scrambler, independent of this file's own
           hypothesis-search state machine. Used to rule out a demapping/
           descrambling logic bug for a live MP CRC failure against
           d-modem/slmodemd: none of the 24*2*2*2=192 combinations, at any
           frame-start offset, produced a valid frame from the captured
           dibits, which points upstream at symbol demodulation (equalizer/
           carrier/timing feeding the angle computation below) rather than
           this file's bit-level decode logic. See rig/README.md. */
        if (V34_DIAG_GETENV("ME_V34_DUMP_MP_DIBITS") && s->mp_seen == 0 && s->duration < 6000)
        {
            /* coefmag/rawmag (diagnostic, 2026-07-19): distinguishes a
               zeroed-coefficient equalizer from a genuinely-silent input —
               coefmag is the summed tap magnitude of s->eq_coeff, rawmag is
               the magnitude of the most recently inserted raw eq_buf sample
               (same index tune_equalizer_cma() reads as its newest tap). */
            float coefmag = 0.0f;
            int ci;
            int rawp;
            for (ci = 0;  ci < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  ci++)
                coefmag += sqrtf(s->eq_coeff[ci].re*s->eq_coeff[ci].re + s->eq_coeff[ci].im*s->eq_coeff[ci].im);
            rawp = (s->eq_step - 1) & V34_EQUALIZER_MASK;
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - MP raw dibit dump: baud=%d diff=%d abs=%d re=%.4f im=%.4f mag=%.4f ang1deg=%.2f coefmag=%.4f rawmag=%.4f\n",
                     s->duration, data_bits, abs_bits,
                     (double) sym->re, (double) sym->im,
                     (double) sqrtf(sym->re*sym->re + sym->im*sym->im),
                     (double) (ang1 * 360.0f / 4294967296.0f),
                     (double) coefmag,
                     (double) sqrtf(s->eq_buf[rawp].re*s->eq_buf[rawp].re + s->eq_buf[rawp].im*s->eq_buf[rawp].im));
        }

        /* Dibit distribution diagnostic */
        {
            static int dibit_hist[4] = {0,0,0,0};
            static int abs_hist[4] = {0,0,0,0};
            static int diag_count = 0;
            static int diag_baud = 0;
            dibit_hist[data_bits]++;
            abs_hist[abs_bits]++;
            if (s->mp_seen == 0)
            {
                diag_count++;
                diag_baud++;
                if (diag_count == 400)
                {
                    if (V34_DEBUG_MP_DIBIT_DIST
                        && (diag_baud % PHASE4_MP_DIBIT_LOG_INTERVAL) == 0)
                    {
                        float mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                        float ang_deg = atan2f(sym->im, sym->re) * 180.0f / 3.14159265f;
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4 MP dibit dist (400 bauds): diff=[%d,%d,%d,%d] abs=[%d,%d,%d,%d] mag=%.3f ang=%.1f\n",
                                 dibit_hist[0], dibit_hist[1], dibit_hist[2], dibit_hist[3],
                                 abs_hist[0], abs_hist[1], abs_hist[2], abs_hist[3],
                                 mag, ang_deg);
                    }
                    mp_phase4_update_auto_domain(s, dibit_hist, abs_hist);
                    dibit_hist[0] = dibit_hist[1] = dibit_hist[2] = dibit_hist[3] = 0;
                    abs_hist[0] = abs_hist[1] = abs_hist[2] = abs_hist[3] = 0;
                    diag_count = 0;
                }
            }
        }

        /* Nothing in 10.1.3.9 or 11.4.1.2 makes MP0 come first: the type bit
           says whether the frame carries 11.4.1.2's precoder coefficients, and
           a modem that uses precoding sends MP1 from its first frame.  Locking
           only MP0 during acquisition was tuned against this tree's own
           transmitter, which sends Type 0, and it is exactly what blocked a
           foreign modem: replaying a live SmartLink call's Phase 4 symbols
           offline recovers 92 frame syncs 188 bits apart with 87 CRC-valid
           frames, every one of them Type 1, while the live receiver -- given
           the same dibits -- would not lock any of them.  The CRC is what
           rejects a false lock; the type bit is not ours to require. */
        expected_mp_type = -1;

            locked_this_symbol = 0;

            if (s->mp_hypothesis >= 0)
            {
                int in0;
                int in1;
                int raw_bits;

                raw_bits = v34_rx_map_phase4_raw_bits((mp_decode_domain == 1) ? abs_bits : data_bits,
                                               s->mp_hypothesis);
                phase4_unpack_ordered_bits(raw_bits, s->mp_phase4_bit_order, &in0, &in1);
                bits[0] = v34_rx_descramble(s, in0);
                bits[1] = v34_rx_descramble(s, in1);
            }
            else if ((sym->re*sym->re + sym->im*sym->im)
                         < MP_LOCK_MIN_SIGNAL_MAG2
                     || (v90_cp_rx
                         && power_meter_current(&s->power)
                              < s->carrier_off_power))
            {
                /* No real signal yet -- don't attempt a preamble lock
                   against near-zero-energy samples. Measured live: after
                   "far-end J' + TRN confirmed" our RX enters MP search
                   immediately, but d-modem/slmodemd doesn't actually start
                   transmitting real MP data until ~5500 bauds (~2.3s)
                   later, ramping from exactly 0.0 to full amplitude over
                   ~20 bauds. Without this gate, the hypothesis search
                   deterministically locks onto a frame straddling that
                   ramp -- structurally plausible (correct start bits) but
                   with corrupted payload bits from the still-settling
                   samples, failing CRC every time. See rig/README.md.

                   The equalizer can ring above MP_LOCK_MIN_SIGNAL_MAG2
                   through the real pre-CPt carrier gap. For the forced V.90
                   CP receiver, also use the primary-channel carrier-off
                   threshold; unlike an arbitrary constellation threshold,
                   that meter is already level-calibrated and hysteretic. */
                if (v90_cp_rx && s->mp_signal_settle_bauds > 0)
                {
                    mp_v90_cp_reset_at_carrier_gap(s);
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 Phase 4: CP carrier gap; reset Table-14 search to first-CPt initial state\n");
                }
                /*endif*/
                s->mp_signal_settle_bauds = 0;
                bits[0] = bits[1] = 0;
            }
            else if (!v90_cp_rx
                     && s->mp_signal_settle_bauds++ < MP_LOCK_SETTLE_BAUDS)
            {
                /* Signal is present but the CMA equalizer is still
                   re-adapting its gain from the silence-era near-zero
                   coefficients to the real signal's amplitude -- locking
                   here still produces structurally-plausible but corrupt
                   frames (start-bit/CRC failures). Wait for a sustained
                   settle window. See rig/README.md.

                   Do not apply that V.34 MP guard to the V.90 digital
                   answerer's project-owned CP receiver. The primary-channel
                   front end was already conditioned before
                   v34_force_v90_phase4_cp_rx(), and V.90 9.4.1.1 requires
                   the digital modem to condition its receiver to receive
                   CPt while sending Ri. A short analogue modem can finish
                   its first CPt before this 400-baud guard expires. The
                   external Table-14 receiver still requires exact framing,
                   CRC and semantic validity before accepting anything. */
                bits[0] = bits[1] = 0;
            }
            else
            {
                int chosen_hyp;
                int chosen_type_bit;
                int chosen_score;
                uint32_t chosen_reg;
                uint32_t chosen_bstream;
                int chosen_bit_pos;
                int chosen_pending_valid;
                int chosen_pending_bit;
                uint32_t chosen_preamble_stream;
                int hint_h;
                int hint_only;
                int pin_diff_hyp;
                int strict_mp0_lock;
                int hint_found;
                int hint_score;
                int hint_type_bit;
                uint32_t hint_reg;
                uint32_t hint_bstream;
                int hint_bit_pos;
                int hint_pending_valid;
                int hint_pending_bit;
                uint32_t hint_preamble_stream;

                if (v90_cp_rx)
                    s->mp_signal_settle_bauds++;
                /*endif*/
                chosen_hyp = -1;
                chosen_type_bit = 0;
                chosen_score = -1;
                chosen_reg = 0;
                chosen_bstream = 0;
                chosen_bit_pos = -1;
                chosen_pending_valid = 0;
                chosen_pending_bit = 0;
                chosen_preamble_stream = 0;
                hint_h = mp_phase4_has_pinned_trn_lock(s) ? s->phase4_trn_lock_hyp : s->phase3_j_lock_hyp;
                /* A differentially decoded V.90 CP has an exactly known dibit
                   transform (see MP_HYPOTHESIS_DIFF_INVERSE).  Pin it, and do
                   not let the reject/no-lock escapes wander off it: there is
                   nothing to search for, and wandering only trades the one
                   correct transform for false preamble hits. */
                /* Not just V.90 CP: plain V.34 MP is the same construction.
                   10.1.3.9 generates the 4-point MP "as described in 10.1.3.3",
                   which rotates the point CLOCKWISE by Zn*90 degrees, and the
                   receiver measures the increment counter-clockwise -- so the
                   recovered dibit is the negation of the transmitted one there
                   too, fixed by the encoder and the table rather than by the
                   channel.  Leaving it to the search cost the whole exchange:
                   measured against a SmartLink call modem, the search settled
                   on hypothesis 18 with the bit order swapped and decoded
                   garbage behind a perfect preamble, while replaying the same
                   symbols through the negation gives 98 frame syncs at exactly
                   188 bits apart, all Type 1, 97 of them CRC-valid. */
                pin_diff_hyp = (mp_decode_domain == 0);
                if (pin_diff_hyp)
                {
                    hint_h = (v90_cp_rx  &&  s->v90_cp_diff_hypothesis >= 0)
                           ?  s->v90_cp_diff_hypothesis
                           :  MP_HYPOTHESIS_DIFF_INVERSE;
                }
                /*endif*/
                strict_mp0_lock = (s->mp_seen == 0 && expected_mp_type == 0);
                /* Constrain early MP lock to the TRN/J hint until we have
                   accumulated a couple of failed frame attempts. Using absolute
                   phase4 duration is ineffective because MP starts late in phase4. */
                hint_only = pin_diff_hyp
                            || (hint_h >= 0
                             && s->mp_phase4_reject_streak < MP_HINT_STRICT_REJECTS
                             && s->mp_phase4_nolock_count < MP_HINT_MAX_NOLOCKS
                             && hint_h < MP_HYPOTHESIS_COUNT
                             && (mp_phase4_has_pinned_trn_lock(s)
                                 || s->phase3_j_lock_hyp >= 0));
                hint_found = 0;
                hint_score = -1;
                hint_type_bit = 0;
                hint_reg = 0;
                hint_bstream = 0;
                hint_bit_pos = -1;
                hint_pending_valid = 0;
                hint_pending_bit = 0;
                hint_preamble_stream = 0;
                for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                {
                    uint32_t reg;
                    uint32_t bstream;
                    uint32_t pre0;
                    int raw_bits;
                    int d0;
                    int d1;
                    int in0;
                    int in1;
                    int sc;
                    int sc0;

                    reg = s->mp_hyp_scramble[h];
                    bstream = s->mp_hyp_bitstream[h];
                    raw_bits = v34_rx_map_phase4_raw_bits((mp_decode_domain == 1) ? abs_bits : data_bits, h);
                    phase4_unpack_ordered_bits(raw_bits, s->mp_phase4_bit_order, &in0, &in1);
                    d0 = v34_rx_descramble_reg(&reg, s->scrambler_tap, in0);
                    bstream = (bstream << 1) | d0;
                    sc0 = mp_preamble_score(bstream);
                    pre0 = bstream;
                    d1 = v34_rx_descramble_reg(&reg, s->scrambler_tap, in1);
                    bstream = (bstream << 1) | d1;
                    if (sc0 >= ((hint_only && h == hint_h) ? MP_HINT_LOCK_SCORE_MIN : MP_LOCK_SCORE_MIN)
                        && (!hint_only || h == hint_h)
                        && (expected_mp_type < 0 || d0 == expected_mp_type)
                        && (d1 == 0
                            || v90_cp_rx)
                        && mp_preamble_has_start_zero(pre0)
                        && mp_preamble_has_sync_ones(pre0)
                        && sc0 > chosen_score)
                    {
                        /* Lock boundary at bit0: current bit is MP type bit.
                           The second bit of this dibit belongs to frame_idx=19. */
                        chosen_hyp = h;
                        chosen_type_bit = d0;
                        chosen_score = sc0;
                        chosen_reg = reg;
                        chosen_bstream = bstream;
                        chosen_bit_pos = 0;
                        chosen_pending_valid = 1;
                        chosen_pending_bit = d1;
                        chosen_preamble_stream = pre0;
                    }
                    /*endif*/
                    if (h == hint_h
                        && sc0 >= MP_HINT_LOCK_SCORE_MIN
                        && (expected_mp_type < 0 || d0 == expected_mp_type)
                        && (d1 == 0
                            || v90_cp_rx)
                        && mp_preamble_has_start_zero(pre0)
                        && mp_preamble_has_sync_ones(pre0)
                        && sc0 > hint_score)
                    {
                        hint_found = 1;
                        hint_score = sc0;
                        hint_type_bit = d0;
                        hint_reg = reg;
                        hint_bstream = bstream;
                        hint_bit_pos = 0;
                        hint_pending_valid = 1;
                        hint_pending_bit = d1;
                        hint_preamble_stream = pre0;
                    }
                    /*endif*/
                    sc = mp_preamble_score(bstream);
                    if ((!strict_mp0_lock || expected_mp_type != 0
                         || v90_cp_rx)
                        && sc >= ((hint_only && h == hint_h) ? MP_HINT_LOCK_SCORE_MIN : MP_LOCK_SCORE_MIN)
                        && (!hint_only || h == hint_h)
                        && (!strict_mp0_lock || sc >= MP_PREAMBLE_SCORE_MIN)
                        && d0 == 0
                        && (expected_mp_type < 0 || d1 == expected_mp_type)
                        && mp_preamble_has_start_zero(bstream)
                        && mp_preamble_has_sync_ones(bstream)
                        && sc > chosen_score)
                    {
                        chosen_hyp = h;
                        chosen_type_bit = d1;
                        chosen_score = sc;
                        chosen_reg = reg;
                        chosen_bstream = bstream;
                        chosen_bit_pos = 1;
                        chosen_pending_valid = 0;
                        chosen_pending_bit = 0;
                        chosen_preamble_stream = bstream;
                    }
                    /*endif*/
                    if ((!strict_mp0_lock || expected_mp_type != 0
                         || v90_cp_rx)
                        && h == hint_h
                        && sc >= MP_HINT_LOCK_SCORE_MIN
                        && (!strict_mp0_lock || sc >= MP_PREAMBLE_SCORE_MIN)
                        && d0 == 0
                        && (expected_mp_type < 0 || d1 == expected_mp_type)
                        && mp_preamble_has_start_zero(bstream)
                        && mp_preamble_has_sync_ones(bstream)
                        && sc > hint_score)
                    {
                        hint_found = 1;
                        hint_score = sc;
                        hint_type_bit = d1;
                        hint_reg = reg;
                        hint_bstream = bstream;
                        hint_bit_pos = 1;
                        hint_pending_valid = 0;
                        hint_pending_bit = 0;
                        hint_preamble_stream = bstream;
                    }
                    s->mp_hyp_scramble[h] = reg;
                    s->mp_hyp_bitstream[h] = bstream;
                }
                /*endfor*/
                if (hint_found
                    && hint_h >= 0
                    && hint_h < MP_HYPOTHESIS_COUNT
                    /* Do not let a weaker TRN/J hint override an equally or more
                       convincing live preamble candidate. The hint is only meant
                       to break ties when the observed MP preamble is just as good. */
                    && (chosen_hyp < 0 || hint_score >= chosen_score))
                {
                    chosen_hyp = hint_h;
                    chosen_type_bit = hint_type_bit;
                    chosen_score = hint_score;
                    chosen_reg = hint_reg;
                    chosen_bstream = hint_bstream;
                    chosen_bit_pos = hint_bit_pos;
                    chosen_pending_valid = hint_pending_valid;
                    chosen_pending_bit = hint_pending_bit;
                    chosen_preamble_stream = hint_preamble_stream;
                }
                /*endif*/
                /* Expose one live hypothesis stream for diagnostics while unlocked. */
                s->bitstream = s->mp_hyp_bitstream[0];

                if (chosen_hyp >= 0)
                {
                    /* V90_CP_LOCK_TRACE: one line per hypothesis lock.
                       The startup CP emit is gated on mp_hypothesis >= 0, and
                       a lock RE-SEEDS the descrambler, the bitstream and the
                       frame position -- so a second lock taken part way
                       through a CP frame rewrites the rest of it while
                       leaving everything already emitted intact.  That is the
                       one mechanism left that corrupts a TAIL under a front
                       end measured to be completely static (AGC, carrier,
                       equalizer tap and input power all constant across the
                       decay).  The lock criterion is the 17-one/start-zero MP
                       preamble allowing two errors, and a Table 14 CP body
                       carries long runs of ones in its constellation masks,
                       so a false preamble inside a frame is expected rather
                       than exotic.  `sym` is s->duration: the CP bit stream
                       runs at 2 bits per symbol from the FIRST lock, so a
                       re-lock at symbol N after that one lands at CP bit 2N,
                       which is what lines this up against V90_CP_BIT_DUMP. */
                    {
                        static int lock_trace_checked = 0;
                        static FILE *lock_trace = NULL;

                        if (!lock_trace_checked)
                        {
                            const char *path = V34_DIAG_GETENV("V90_CP_LOCK_TRACE");

                            if (path  &&  *path)
                                lock_trace = fopen(path, "w");
                            /*endif*/
                            lock_trace_checked = 1;
                        }
                        /*endif*/
                        if (lock_trace)
                        {
                            fprintf(lock_trace,
                                    "sym=%d relock=%d prev_hyp=%d hyp=%d score=%d type=%d "
                                    "frame_pos=%d cp_rx=%d nolock=%d\n",
                                    s->duration,
                                    (s->mp_hypothesis >= 0) ? 1 : 0,
                                    s->mp_hypothesis, chosen_hyp, chosen_score,
                                    chosen_type_bit, s->mp_frame_pos,
                                    v90_cp_rx ? 1 : 0,
                                    s->mp_phase4_nolock_count);
                            fflush(lock_trace);
                        }
                        /*endif*/
                    }
                    s->mp_phase4_nolock_count = 0;
                    s->mp_hypothesis = chosen_hyp;
                    s->scramble_reg = chosen_reg;
                    s->bitstream = chosen_bstream;
                    s->crc = 0xFFFF;
                    s->bit_count = 0;
                    s->mp_count = 0;
                    s->mp_len = 84 + 1;
                    s->mp_and_fill_len = 186 + 1 + 1;
                    s->mp_early_rejects = 0;
                    /* Lock criterion is the MP preamble itself, so start frame
                       collection immediately at this boundary. */
                    {
                        int type;

                        mp_seed_frame_prefix(s->mp_frame_bits, chosen_preamble_stream);
                        type = s->mp_frame_bits[18];
                        s->mp_frame_target = (type == 1)  ?  188  :  88;
                        s->mp_frame_pos = 19;
                        if (chosen_pending_valid)
                        {
                            s->mp_frame_bits[s->mp_frame_pos++] = chosen_pending_bit;
                            s->bit_count = 1;
                        }
                        else
                        {
                            s->bit_count = 0;
                        }
                        /*endif*/
                        if (v90_cp_rx)
                        {
                            int replay_len;

                            replay_len = s->v90_cp_stream ? 0 : s->mp_frame_pos;
                            for (int replay = 0; replay < replay_len; replay++)
                            {
                                int replay_bit;

                                /* The hypothesis lock permits up to two errors
                                   in the fixed 17-one/start-zero preamble.  The
                                   project CP framer quite properly requires an
                                   exact sync word, so repair only those fixed
                                   acquisition bits after the hypothesis has
                                   passed the preamble gate.  Type and all
                                   variable/CRC-protected bits remain observed
                                   data and are never corrected here. */
                                replay_bit = (replay < 17)
                                           ? 1
                                           : ((replay == 17)
                                              ? 0
                                              : (s->mp_frame_bits[replay] & 1));
                                s->put_phase4_bit(s->put_phase4_bit_user_data,
                                                  replay_bit);
                            }
                            /* The project-owned Table 14 framer owns CP length,
                               CRC, fill and semantics from this boundary on. */
                            s->mp_frame_pos = 0;
                            s->mp_frame_target = 0;
                        }
                        log_mp_lock_seed(s,
                                         chosen_hyp,
                                         chosen_type_bit,
                                         chosen_score,
                                         chosen_bit_pos,
                                         chosen_preamble_stream,
                                         chosen_pending_valid,
                                         chosen_pending_bit);
                    }
                    locked_this_symbol = 1;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: locked MP hypothesis=%d, type_bit=%d preamble_score=%d/18 at bit%d (baud %d), frame start armed%s\n",
                             chosen_hyp, chosen_type_bit, chosen_score, chosen_bit_pos,
                             s->duration + 1, chosen_pending_valid ? " (+1 pending bit)" : "");
                }
                /*endif*/
                bits[0] = bits[1] = 0;
            }
            /*endif*/

        s->duration++;
        if (v90_cp_rx)
        {
            if (s->v90_cp_stream)
            {
                /* V.90 9.6.1.1.1's CP, streamed rather than searched.
                 *
                 * The emit below is conditional on a V.34 MP hypothesis being
                 * LOCKED, which is a startup arrangement: there the search
                 * finds the preamble, replays the frame so far and then
                 * streams.  A renegotiation breaks it.  Figure 8/V.90 puts up
                 * to 2000 ms of SCR in front of CP, SCR descrambles to a solid
                 * run of ones, so the 17-one preamble gate reads 18/18 for the
                 * whole of it: measured on a replay of a live renegotiation
                 * (artifacts/reneg-eq/reneg-r1) the search locked and was
                 * rejected 91 times inside the window, the framer was handed
                 * only the fragments those locks covered, and not one frame
                 * ever synced -- 91 rejects, all structural, no CRC reject at
                 * all, which is what "the boundary was never even plausible"
                 * looks like.
                 *
                 * There is nothing to search for.  The domain is differential,
                 * the dibit transform is the fixed negation (see
                 * MP_HYPOTHESIS_DIFF_INVERSE), the scrambler is the analogue
                 * modem's GPA (tap 4) and the bit order is b0,b1 -- all four
                 * fixed by 8.5.2/10.1.3.3 and the constellation table, not by
                 * the channel.  So decode every symbol that way and let the
                 * Table-14 framer own sync, length, CRC and semantics, which
                 * it is written to do.  Running exactly this over the same
                 * recording's symbols offline recovers 47 CP frames, every gap
                 * exactly 700 bits and every consecutive pair bit-identical.
                 */
                int raw;
                int in0;
                int in1;
                int hyp = (s->v90_cp_diff_hypothesis >= 0)
                        ?  s->v90_cp_diff_hypothesis
                        :  MP_HYPOTHESIS_DIFF_INVERSE;

                /* V90_RENEG_SYM_DUMP: one line per symbol of the 9.6 CP
                   window -- the decision AND the two loops that could be
                   taking it away.  The window's bit stream shows the peer's
                   SCR and five 700-bit CP frames and then a run of exact
                   zeros, on a line whose RMS never moves, so what is needed
                   is not another look at the bits but a read of the front
                   end across that instant: the AGC scaling, the equalizer's
                   level estimate, the carrier loop's frequency, and the
                   input power meter, which says whether the collapse is on
                   this side of the demodulator at all.  A run of exact zeros
                   out of a self-synchronizing descrambler is a CONSTANT raw
                   dibit, so data_bits is here too. */
                {
                    static int reneg_sym_checked = 0;
                    static FILE *reneg_sym = NULL;

                    if (!reneg_sym_checked)
                    {
                        const char *path = V34_DIAG_GETENV("V90_RENEG_SYM_DUMP");

                        if (path  &&  *path)
                            reneg_sym = fopen(path, "w");
                        /*endif*/
                        reneg_sym_checked = 1;
                    }
                    /*endif*/
                    if (reneg_sym)
                    {
                        fprintf(reneg_sym,
                                "%d %d %.5f %.2f %.6f %.5f %.6f %ld %.1f %d %ld %.2f %.2f %.5f\n",
                                s->duration,
                                data_bits,
                                sqrtf(sym->re*sym->re + sym->im*sym->im),
                                (double) (180.0f/3.14159265f)
                                    *atan2f(sym->im, sym->re),
                                (double) s->agc_scaling,
                                (double) s->eq_target_mag,
                                (double) dds_frequencyf(s->v34_carrier_phase_rate),
                                (long) power_meter_current(&s->power),
                                v90_reneg_feed_rms,
                                s->total_baud_timing_correction,
                                (long) s->qam_sample_time,
                                (double) (uint32_t) s->carrier_phase
                                    *(360.0/4294967296.0),
                                (double) (180.0f/3.14159265f)
                                    *atan2f(s->eq_coeff[V34_EQUALIZER_PRE_LEN].im,
                                            s->eq_coeff[V34_EQUALIZER_PRE_LEN].re),
                                (double) sqrtf(
                                    s->eq_coeff[V34_EQUALIZER_PRE_LEN].re
                                        *s->eq_coeff[V34_EQUALIZER_PRE_LEN].re
                                  + s->eq_coeff[V34_EQUALIZER_PRE_LEN].im
                                        *s->eq_coeff[V34_EQUALIZER_PRE_LEN].im));
                    }
                    /*endif*/
                }
                raw = v34_rx_map_phase4_raw_bits(data_bits, hyp);
                phase4_unpack_ordered_bits(raw, s->mp_phase4_default_bit_order,
                                           &in0, &in1);
                s->put_phase4_bit(s->put_phase4_bit_user_data,
                                  v34_rx_descramble_reg(&s->v90_cp_stream_reg,
                                                 s->mp_phase4_default_scrambler_tap,
                                                 in0));
                s->put_phase4_bit(s->put_phase4_bit_user_data,
                                  v34_rx_descramble_reg(&s->v90_cp_stream_reg,
                                                 s->mp_phase4_default_scrambler_tap,
                                                 in1));
            }
            else if (s->mp_hypothesis >= 0 && !locked_this_symbol)
            {
                s->put_phase4_bit(s->put_phase4_bit_user_data, bits[0] & 1);
                s->put_phase4_bit(s->put_phase4_bit_user_data, bits[1] & 1);
            }
            /* CP is variable length and is validated by the V.90 layer. Do not
               apply V.34 MP lengths, semantic recovery or forced E timeouts. */
            break;
        }
        /* MP stage timeout: if we haven't successfully decoded an MP frame
           within ~2s (4800 bauds at 2400 baud rate), signal training failure
           so the call can retrain or drop.  The far-end won't wait forever. */
        if (s->mp_seen == 0
            && s->duration >= PHASE4_MP_TIMEOUT_BAUDS
            && s->received_event != V34_EVENT_TRAINING_FAILED)
        {
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: MP timeout (%d bauds without successful MP frame); signalling failure\n",
                     s->duration);
            s->received_event = V34_EVENT_TRAINING_FAILED;
        }
        /*endif*/
        /* 11.4.1.1.3/11.4.2.1.2 (caller: 2500 ms + 2 RTD after J'), 11.4.2.2.2
           (answerer: 2500 ms + 3 RTD from S-bar), 30 s when the peer's INFO0 has
           the CME bit: if E has not arrived the modem retrains.  The MP-only
           watchdog above stops looking once any MP is accepted, so a peer that
           goes quiet after its first MP was never given up on.  Measured from
           the first accepted MP, which is later than the clause's origins and so
           errs towards waiting; the round trip is an allowance
           (250 ms), not a measurement. */
        if (s->duplex
            && s->mp_seen == 1
            && s->mp_accepted_baud > 0
            && s->baud_rate >= 0  &&  s->baud_rate <= 5
            && s->received_event != V34_EVENT_TRAINING_FAILED)
        {
            const int rtd_ms = 250;
            int wait_ms;

            wait_ms = s->far_capabilities.from_cme_modem  ?  30000  :  2500 + 3*rtd_ms;
            if (s->duration - s->mp_accepted_baud
                >= (baud_rate_parameters[s->baud_rate].baud_rate*wait_ms + 500)/1000)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: no E within %d ms of the first MP (11.4); signalling failure\n",
                         wait_ms);
                s->received_event = V34_EVENT_TRAINING_FAILED;
            }
            /*endif*/
        }
        /*endif*/
        if (s->mp_hypothesis < 0
            && s->mp_seen == 0
            && (s->duration % 400) == 0)
        {
            /* A pinned TRN lock is the best initial MP hint, but it is not
               infallible. In A-law especially we can end up with the wrong
               MP domain/order/tap while never locking any MP hypothesis at
               all. Hold the pinned settings for a few no-lock windows first,
               then rotate retry modes so we do not sit in PHASE4_MP until
               timeout on a bad decode mode forever. */
            if (mp_phase4_has_pinned_trn_lock(s))
            {
                s->mp_phase4_nolock_count++;
                if (s->mp_phase4_nolock_count >= MP_HINT_MAX_NOLOCKS
                    && s->mp_phase4_reject_streak < MP_HINT_STRICT_REJECTS)
                {
                    s->mp_phase4_reject_streak = MP_HINT_STRICT_REJECTS;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: no MP lock at baud %d with pinned hint hyp=%d for %d windows; broadening MP search beyond hint-only\n",
                             s->duration, s->phase4_trn_lock_hyp, s->mp_phase4_nolock_count);
                }
                /*endif*/
                /* Log best preamble score for the hint hypothesis to diagnose MP lock failures */
                {
                    int best_sc = 0;
                    for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                    {
                        int sc_diag = mp_preamble_score(s->mp_hyp_bitstream[h]);
                        if (sc_diag > best_sc)
                            best_sc = sc_diag;
                    }
                    if ((s->duration % PHASE4_MP_NOLOCK_LOG_INTERVAL) == 0
                        || best_sc >= (MP_PREAMBLE_SCORE_MIN - 1))
                    {
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: no MP lock at baud %d; best_score=%d/%d hint_hyp=%d hint_bs=0x%08X (dom=%s%s, tap=%d, ord=%s)\n",
                                 s->duration, best_sc, MP_PREAMBLE_SCORE_MIN,
                                 s->phase4_trn_lock_hyp,
                                 (unsigned)s->mp_hyp_bitstream[s->phase4_trn_lock_hyp],
                                 v34_rx_phase4_trn_domain_name(s->mp_phase4_domain),
                                 s->mp_phase4_force_abs_active ? "/auto-abs" : "",
                                 s->scrambler_tap,
                                 v34_rx_phase4_trn_order_name(s->mp_phase4_bit_order));
                    }
                    /*endif*/
                }
                if (s->mp_phase4_nolock_count >= MP_HINT_MAX_NOLOCKS)
                {
                    mp_phase4_rotate_retry_mode(s, "no MP hypothesis lock");
                }
                else
                {
                    v34_rx_mp_reset_hypothesis_search(s);
                }
                /*endif*/
            }
            else
            {
                mp_phase4_rotate_retry_mode(s, "no MP hypothesis lock");
            }
        }
        /*endif*/
        /* Per-baud MP diagnostics are most useful once a hypothesis is locked.
           Suppress unlocked chatter (hyp=-1), which tends to dominate logs without
           adding actionable information. */
        if (s->mp_hypothesis >= 0
            && s->mp_seen == 0
            && (s->duration <= 6 || (s->duration % PHASE4_MP_BAUD_LOG_INTERVAL) == 0))
        {
            float mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4 MP baud %d: mag=%.3f data_bits=%d descr=%d,%d "
                     "bitstream=0x%08X hyp=%d\n",
                     s->duration, mag, data_bits, bits[0], bits[1],
                     (unsigned)s->bitstream, s->mp_hypothesis);
        }

        /* Scan for MP frames on the primary channel */
        if (s->mp_hypothesis >= 0  &&  !locked_this_symbol)
        {
            for (i = 0;  i < 2;  i++)
            {
                s->bitstream = (s->bitstream << 1) | bits[i];
                if (s->mp_bits_since_frame < 1000000)
                    s->mp_bits_since_frame++;
                /*endif*/
                if (s->mp_hypothesis >= 0  &&  s->mp_frame_pos == 0  &&  s->mp_seen == 0)
                {
                    int preamble_wait_limit;
                    const char *hold_env;
                    bool hold_mp_hypothesis;

                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "awaiting MP preamble");

                    hold_env = getenv("ME_V34_HOLD_MP_HYPOTHESIS");
                    hold_mp_hypothesis = hold_env && atoi(hold_env) != 0;

                    /* If we started from a TRN direct pre-lock, fail fast and
                       fall back to global hypothesis search rather than waiting
                       the full window on a possibly phase-offset hypothesis. */
                    preamble_wait_limit = (s->mp_phase4_reject_streak == 0)
                                          ? MP_PRELOCK_PREAMBLE_WAIT_BITS
                                          : MP_PREAMBLE_WAIT_BITS;
                    if (++s->mp_count > preamble_wait_limit)
                    {
                        if (hold_mp_hypothesis)
                        {
                            /* A strong preamble can be followed by a short
                               equalizer/timing excursion before the peer's
                               repeated MP arrives. Keep the same hypothesis
                               and restart only the local wait window. The
                               normal CRC and semantic checks still decide
                               whether this hypothesis is valid. */
                            s->mp_count = 0;
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: holding MP hypothesis=%d after preamble timeout; restarting local wait\n",
                                     s->mp_hypothesis);
                            break;
                        }
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: unlock MP hypothesis=%d (no preamble within %d bits)\n",
                                 s->mp_hypothesis, s->mp_count);
                        if (mp_phase4_has_pinned_trn_lock(s))
                        {
                            /* Keep TRN-locked descrambler settings, just reset
                               hypothesis search to re-scan for preamble */
                            v34_rx_mp_reset_hypothesis_search(s);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: preamble timeout; keeping TRN-locked settings, resetting preamble search\n");
                        }
                        else
                        {
                            mp_phase4_rotate_retry_mode(s, "unlock MP hypothesis (preamble timeout)");
                        }
                        break;
                    }
                    /*endif*/
                }
                /*endif*/
                if (s->mp_seen >= 2)
                {
                    /* Should not reach here — we transition to V34_RX_STAGE_DATA on E */
                    continue;
                }
                /*endif*/
                /* 11.4.1.1.3/11.4.1.2.3: after sending MP' a modem may complete on
                   E even when the peer's own MP' was damaged.  Without the
                   acknowledged frame there is no exact boundary to start the
                   20-one test from, and an MP body's ones can mimic E, so it is
                   accepted only when (a) our own complete MP' has gone out, and
                   (b) the 20 ones end on a bit offset that is a whole number of
                   MP frames, plus 20, after the last frame we did decode, with at
                   least one whole frame (the lost MP') in between. */
                bool e_without_ack = false;
                if (s->mp_seen == 1
                    && !s->mp_remote_ack_seen
                    && s->mp_last_frame_bits > 0
                    && (s->bitstream & 0xFFFFF) == 0xFFFFF
                    && s->mp_bits_since_frame >= s->mp_last_frame_bits + 20
                    && ((s->mp_bits_since_frame - 20) % s->mp_last_frame_bits) == 0)
                {
                    v34_state_t *owner = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));

                    e_without_ack = owner->tx.mp.mp_acknowledged
                                 && owner->tx.mp_prime_frames >= 1;
                }
                /*endif*/
                if (s->mp_seen == 1
                    && (s->mp_remote_ack_seen  ||  e_without_ack)
                    && (s->bitstream & 0xFFFFF) == 0xFFFFF)
                {
                    if (e_without_ack)
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: E after %d bits on the MP frame grid with no peer MP' decoded (11.4.1.x.3)\n",
                                 s->mp_bits_since_frame);
                    /* V.34 11.4.1.1.3/11.4.1.2.3 permits E only after MP' has
                       been received.  Requiring the acknowledged frame is also
                       essential for alignment: the tail of an ordinary MP can
                       contain enough ones that combining it with the first few
                       E bits falsely detects E up to 18 symbols early. */
                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_COMPLETE, "E detected");
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: E signal detected, MP exchange complete — transitioning to DATA mode\n");
                    /* Use the same reset-state B1 entry as the offline and
                       V.90 paths.  The old inline subset left every Viterbi
                       state at metric zero and started V0 at ordinary frame
                       zero, although 10.1.3.1 defines B1 as the final data
                       frame of a superframe with all encoder state reset. */
                    t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                    if (v34_begin_rx_data(t) != 0)
                    {
                        s->received_event = V34_EVENT_TRAINING_FAILED;
                        report_status_change(s, SIG_STATUS_TRAINING_FAILED);
                        break;
                    }
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - DATA mode: parms b=%d k=%d q=%d m=%d p=%d j=%d l=%d r=%d w=%d\n",
                             s->parms.b, s->parms.k, s->parms.q, s->parms.m,
                             s->parms.p, s->parms.j, s->parms.l, s->parms.r, s->parms.w);
                    if (s->duplex)
                        report_status_change(s, SIG_STATUS_TRAINING_SUCCEEDED);
                    /*endif*/
                    break;  /* Exit the bit loop; next baud will enter DATA stage */
                }
                /*endif*/
                /* Continue parsing repeated MP' frames while waiting for E.
                   Each CRC-valid MP' resets bitstream at its exact end, so its
                   17-one synchronization prefix cannot combine with preceding
                   fill bits and masquerade as the 20-one E sequence. */

                /* Detect 17x'1' + start '0' + type bit only when not already
                   collecting an MP frame. Otherwise we can retrigger on long runs
                   of ones and keep resetting frame alignment. */
                if (s->mp_frame_pos == 0)
                {
                    int preamble_score;

                    preamble_score = mp_preamble_score(s->bitstream);
                    if (preamble_score >= MP_PREAMBLE_SCORE_MIN
                        && mp_preamble_has_start_zero(s->bitstream)
                        && mp_preamble_has_sync_ones(s->bitstream))
                    {
                        int type;
                        char tail[33];

                        mp_seed_frame_prefix(s->mp_frame_bits, s->bitstream);
                        type = s->mp_frame_bits[18];
                        if (expected_mp_type >= 0  &&  type != expected_mp_type)
                            continue;
                        /*endif*/
                        /* Table 20 bit 19 is not interpreted by the receiver, and
                           mp_seed_frame_prefix() writes only positions 0..18, so
                           position 19 here is left over from an EARLIER frame:
                           testing it skipped valid preambles after any frame that
                           had it set.  It is collected with the body for the CRC. */
                        s->mp_frame_target = (type == 1)  ?  188  :  88;
                        s->mp_frame_pos = 19;
                        s->mp_count = 0;
                        s->bit_count = 0;
                        s->mp_early_rejects = 0;
                        v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP preamble found; collecting frame");

                        if (s->mp_seen == 0)
                        {
                            bits32_to_str(s->bitstream, tail);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP preamble detected (baud %d): "
                                     "score=%d/18 17x'1'+start(0)+type(%d), target=%d bits, "
                                     "frame body starts at frame_idx=19 (includes inserted start bits), tail=0b%s\n",
                                     s->duration, preamble_score, type, s->mp_frame_target, tail);
                        }
                        continue;
                    }
                    /*endif*/
                }
                /*endif*/

                if (s->mp_frame_pos > 0  &&  s->mp_frame_pos < s->mp_frame_target)
                {
                    bool crc_good;
                    bool fill_good;
                    bool starts_good;
                    int start_err_count;
                    int type;
                    uint16_t rx_crc;
                    uint16_t residual_crc;

                    s->mp_frame_bits[s->mp_frame_pos++] = bits[i];
                    s->bit_count = s->mp_frame_pos - 19;
                    {
                        int type_now;
                        int idx;

                        idx = s->mp_frame_pos - 1;
                        type_now = s->mp_frame_bits[18];
                        if (!mp_start_bit_ok(type_now, idx, bits[i]))
                        {
                            int data_idx;
                            int start_err_accept_max;

                            data_idx = mp_data_bit_index(type_now, idx);
                            s->mp_early_rejects++;
                            /* MP0 often suffers a transient inserted-start mismatch
                               from one-bit alignment wobble. Allow a small number
                               before dropping lock so CRC/slip recovery can run. */
                            start_err_accept_max = (type_now == 1)
                                                   ? MP1_START_ERR_ACCEPT_MAX
                                                   : MP_EARLY_START_ERR_MAX;
                            if (s->mp_early_rejects <= 3  ||  (s->mp_early_rejects % 8) == 0)
                            {
                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d start-bit mismatch at frame_idx=%d body_idx=%d data_idx=%d value=%d "
                                         "(expected 0), start_err_count=%d max=%d%s\n",
                                         type_now, idx, idx - 19 + 1, data_idx, bits[i],
                                         s->mp_early_rejects, start_err_accept_max,
                                         (s->mp_early_rejects > start_err_accept_max) ? " (rejecting lock immediately)" : " (continuing until CRC)");
                            }
                            /*endif*/
                            if (s->mp_early_rejects > start_err_accept_max)
                            {
                                mp_unlock_after_reject(s, true);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "start-bit mismatch limit exceeded; dropping lock");
                                break;
                            }
                            /*endif*/
                        }
                        /*endif*/
                    }
                    if (s->mp_hypothesis < 0)
                        break;
                    /*endif*/
                    {
                        int frame_idx;
                        int type_now;
                        bool is_inserted_start;
                        bool log_body_bit;

                        frame_idx = s->mp_frame_pos - 1;
                        type_now = s->mp_frame_bits[18];
                        if (frame_idx == 19  &&  bits[i] != 0)
                        {
                            int early_reject_max;

                            early_reject_max = (type_now == 1)
                                               ? MP1_START_ERR_ACCEPT_MAX
                                               : MP_EARLY_START_ERR_MAX;
                            s->mp_early_rejects++;
                            if (s->mp_early_rejects <= 3  ||  (s->mp_early_rejects % 8) == 0)
                            {
                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d reserved bit mismatch at frame_idx=19 value=%d "
                                         "(expected 0), early_reject_count=%d max=%d%s\n",
                                         type_now, bits[i], s->mp_early_rejects, early_reject_max,
                                         (s->mp_early_rejects > early_reject_max) ? " (rejecting lock immediately)" : " (continuing until CRC)");
                            }
                            /*endif*/
                            if (s->mp_early_rejects > early_reject_max)
                            {
                                mp_unlock_after_reject(s, true);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "reserved bit mismatch; dropping lock");
                                break;
                            }
                            /*endif*/
                        }
                        /*endif*/
                        is_inserted_start = (frame_idx == 34 || frame_idx == 51 || frame_idx == 68)
                                            || (type_now == 1
                                                && (frame_idx == 85
                                                    || frame_idx == 102
                                                    || frame_idx == 119
                                                    || frame_idx == 136
                                                    || frame_idx == 153
                                                    || frame_idx == 170));
                        /* Keep MP body logs lightweight:
                           - MP0: early window + periodic checkpoints + inserted starts
                           - MP1: slightly denser early window + periodic checkpoints + inserted starts */
                        log_body_bit = (s->mp_frame_target <= 88)
                                       ? (s->bit_count <= 4
                                          || (s->bit_count % 16) == 0
                                          || is_inserted_start)
                                       : (s->bit_count <= 16
                                       || (s->bit_count % 16) == 0
                                       || is_inserted_start);
                        if (log_body_bit  &&  s->mp_seen == 0)
                        {
                            int data_idx;

                            data_idx = mp_data_bit_index(type_now, frame_idx);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4 MP body_idx=%d frame_idx=%d data_idx=%d value=%d frame_pos=%d/%d\n",
                                     s->bit_count, frame_idx, data_idx, bits[i],
                                     s->mp_frame_pos, s->mp_frame_target);
                        }
                        /*endif*/
                    }

                    if (s->mp_frame_pos < s->mp_frame_target)
                        continue;
                    /*endif*/

                    type = s->mp_frame_bits[18];
                    if (!mp_start_bit_ok(type, 17, s->mp_frame_bits[17]))
                    {
                        s->mp_early_rejects++;
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: MP%d start-bit mismatch at frame_idx=17 body_idx=0 data_idx=-1 value=%d "
                                 "(expected 0), start_err_count=%d (continuing until CRC)\n",
                                 type, s->mp_frame_bits[17], s->mp_early_rejects);
                    }
                    /*endif*/
                    crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                    fill_good = mp_fill_ok(s->mp_frame_bits, type);
                    if (!(crc_good  &&  fill_good))
                    {
                        int recovered_slip;
                        int recovered_boundary;
                        int recovered_boundary_2;
                        int recovered_slip_2;
                        int recovered_changes;

                        recovered_slip = 0;
                        recovered_boundary = 0;
                        recovered_boundary_2 = 0;
                        recovered_slip_2 = 0;
                        recovered_changes = 0;
                        if (mp_try_slip_recovery(s->mp_frame_bits, type, s->mp_frame_target, &recovered_slip))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via bit-slip=%d before CRC/fill check\n",
                                     type, recovered_slip);
                        }
                        else if (mp_try_boundary_slip_recovery(s->mp_frame_bits, type, s->mp_frame_target, &recovered_boundary, &recovered_slip))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via boundary-slip at frame_idx=%d slip=%d before CRC/fill check\n",
                                     type, recovered_boundary, recovered_slip);
                        }
                        else if (mp_try_boundary_double_slip_recovery(s->mp_frame_bits,
                                                                       type,
                                                                       s->mp_frame_target,
                                                                       &recovered_boundary,
                                                                       &recovered_slip,
                                                                       &recovered_boundary_2,
                                                                       &recovered_slip_2))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via double-boundary-slip (%d,%d) and (%d,%d) before CRC/fill check\n",
                                     type,
                                     recovered_boundary,
                                     recovered_slip,
                                     recovered_boundary_2,
                                     recovered_slip_2);
                        }
                        else if (mp_try_boundary_bruteforce_recovery(s->mp_frame_bits,
                                                                      type,
                                                                      s->mp_frame_target,
                                                                      &recovered_changes))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via boundary-bruteforce (%d boundary adjustments) before CRC/fill check\n",
                                     type, recovered_changes);
                        }
                        /*endif*/
                    }
                    /*endif*/
                    start_err_count = mp_start_error_count(s->mp_frame_bits, type, s->mp_frame_target);
                    starts_good = (start_err_count == 0);
                    if (s->mp_seen == 0  ||  !crc_good  ||  !fill_good  ||  !starts_good)
                    {
                        bool log_reject_detail;

                        log_reject_detail = (s->duration <= PHASE4_MP_REJECT_DETAIL_LOG_INTERVAL
                                             || (s->duration % PHASE4_MP_REJECT_DETAIL_LOG_INTERVAL) == 0
                                             || !fill_good
                                             || !starts_good
                                             || (crc_good && fill_good));
                        if (log_reject_detail)
                        {
                            log_mp_frame_diag(s, s->mp_frame_bits, type, crc_good, rx_crc, residual_crc, fill_good);
                        }
                    }
                    {
                        bool frame_accepted;
                        bool first_mp_accept;
                        int start_err_accept_max;
                        bool starts_acceptable;

                        frame_accepted = false;
                        first_mp_accept = (s->mp_seen == 0);
                        start_err_accept_max = (type == 1)
                                               ? MP1_START_ERR_ACCEPT_MAX
                                               : MP_EARLY_START_ERR_MAX;
                        starts_acceptable = (start_err_count <= start_err_accept_max);
                        if (crc_good  &&  fill_good)
                        {
                            bool semantic_good;

                            semantic_good = true;
                            if (s->duplex)
                            {
                                mp_pack_for_parser(s->info_buf, s->mp_frame_bits, type);
                                if (first_mp_accept  ||  type == 1  ||  !s->mp_remote_ack_seen)
                                {
                                    process_rx_mp(s, &mp, s->info_buf);
                                    semantic_good = mp_semantic_ok_phase4(s, &mp, type, s->mp_frame_bits);
                                    if (semantic_good)
                                    {
                                        if (mp.mp_acknowledged)
                                        {
                                            s->mp_remote_ack_seen = 1;
                                            /* Start the 20-one E detector after
                                               the complete MP' boundary; no MP'
                                               payload bits may count toward E
                                               (10.1.3.2, 11.4.1.1.3). */
                                            s->bitstream = 0;
                                        }
                                        /*endif*/
                                        t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                                        if (!mp_apply_parameters(t, &mp))
                                        {
                                            V34_RX_LOG(&t->logging, SPAN_LOG_FLOW,
                                                     "Rx - MP directional encoder/rate negotiation failed "
                                                     "(local mask=0x%04X remote=0x%04X)\n",
                                                     t->tx.mp.signalling_rate_mask & 0x3FFF,
                                                     mp.signalling_rate_mask & 0x3FFF);
                                            semantic_good = false;
                                        }
                                    }
                                    /*endif*/
                                }
                                /*endif*/
                            }
                            /*endif*/
                            if (semantic_good)
                            {
                                if (!starts_good)
                                {
                                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP%d accepting frame with %d start-bit mismatches because CRC/fill are valid\n",
                                             type, start_err_count);
                                }
                                /*endif*/
                                if (first_mp_accept)
                                {
                                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP%d frame accepted (%d bits)\n",
                                             type, s->mp_frame_target);
                                }
                                s->mp_seen = 1;
                                /* Bit 33, acknowledge (Table 20/21): every
                                   CRC-valid far-end MP' counts, whether or not
                                   it was re-parsed above. */
                                if (s->mp_frame_bits[33])
                                    s->mp_remote_ack_count++;
                                /*endif*/
                                if (s->mp_accepted_baud == 0)
                                    s->mp_accepted_baud = s->duration;
                                s->mp_early_rejects = 0;
                                if (first_mp_accept)
                                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP frame accepted; awaiting E");
                                else if (s->mp_remote_ack_seen)
                                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP' (ack) accepted; awaiting E");
                                frame_accepted = true;
                            }
                            else
                            {
                                /* CRC/fill/starts passed but semantic parse failed.
                                   Prefer a local re-acquire on the same hypothesis
                                   before dropping back to global search. */
                                s->mp_early_rejects = 0;
                                s->mp_count = 0;
                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d rejected (semantic checks failed)\n",
                                         type);
                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: keeping MP hypothesis=%d after semantic reject; retrying local preamble reacquire\n",
                                         s->mp_hypothesis);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "semantic reject; reacquiring preamble");
                            }
                            /*endif*/
                        }
                        else
                        {
                            bool keep_hypothesis;

                            /* If all inserted start bits were consistent, this is often
                               a boundary/timing wobble rather than a bad phase hypothesis.
                               Keep the current hypothesis and re-acquire the next preamble
                               locally instead of immediately jumping to global search. */
                            keep_hypothesis = (s->mp_hypothesis >= 0
                                               && starts_acceptable);
                            V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d rejected (crc_ok=%d fill_ok=%d starts_ok=%d starts_acceptable=%d start_err_count=%d max=%d)\n",
                                     type, crc_good, fill_good, starts_good, starts_acceptable, start_err_count, start_err_accept_max);
                            /* Dump first 70 frame bits for diagnosis */
                            {
                                char dump[200];
                                int dlen = (s->mp_frame_target < 90) ? s->mp_frame_target : 90;
                                int d;
                                for (d = 0; d < dlen && d < (int)sizeof(dump) - 1; d++)
                                    dump[d] = '0' + (s->mp_frame_bits[d] & 1);
                                dump[d] = '\0';
                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP frame bits[0..%d]: %s\n",
                                         dlen - 1, dump);
                            }
                            /* Majority-vote accumulator for MP0 frames */
                            if (type == 0  &&  s->mp_frame_target == 88)
                            {
                                int vi;

                                /* V.34 11.4 transmits MP continuously, so a
                                   receiver that cannot decode one frame can
                                   still combine several.  Accumulate into the
                                   bin for the hypothesis this frame was read
                                   under, rather than resetting a single
                                   accumulator whenever the hypothesis changes:
                                   above 2400 baud the hypothesis churns
                                   between frames, so the single accumulator
                                   never survived to the three frames the vote
                                   needs. */
                                int vh = s->mp_hypothesis;

                                if (vh < 0  ||  vh >= MP_HYPOTHESIS_COUNT)
                                    vh = 0;
                                /*endif*/
                                s->mp0_vote_hyp = vh;
                                /* Accumulate: +1 for '1', -1 for '0' */
                                for (vi = 0;  vi < 88;  vi++)
                                    s->mp0_vote_counts[vh][vi] += (s->mp_frame_bits[vi] & 1) ? 1 : -1;
                                s->mp0_vote_frames_by_hyp[vh]++;
                                s->mp0_vote_frames = s->mp0_vote_frames_by_hyp[vh];
                                if (s->mp0_vote_last_hyp == vh)
                                    s->mp0_vote_same_lock[vh]++;
                                else
                                    s->mp0_vote_same_lock[vh] = 1;
                                /*endif*/
                                s->mp0_vote_last_hyp = vh;

                                if (s->mp0_vote_frames <= 2 || (s->mp0_vote_frames % 4) == 0)
                                {
                                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP0 vote accumulator: %d frames (hyp=%d)\n",
                                             s->mp0_vote_frames, s->mp0_vote_hyp);
                                }

                                /* Try majority-vote after every 3+ frames */
                                if (s->mp0_vote_frames >= 3)
                                {
                                    uint8_t voted_bits[88];
                                    uint16_t vote_rx_crc;
                                    uint16_t vote_res_crc;
                                    bool vote_crc_ok;
                                    bool vote_fill_ok;

                                    for (vi = 0;  vi < 88;  vi++)
                                        voted_bits[vi] = (s->mp0_vote_counts[vh][vi] > 0) ? 1 : 0;
                                    /* Force known structural bits */
                                    voted_bits[17] = 0;  /* start bit */
                                    voted_bits[18] = 0;  /* type = MP0 */
                                    voted_bits[19] = 0;  /* reserved */
                                    voted_bits[34] = 0;  /* start bit */
                                    voted_bits[51] = 0;  /* start bit */
                                    voted_bits[68] = 0;  /* start bit */

                                    vote_crc_ok = mp_crc_ok(voted_bits, 0, &vote_rx_crc, &vote_res_crc);
                                    vote_fill_ok = mp_fill_ok(voted_bits, 0);

                                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP0 majority-vote result: crc_ok=%d fill_ok=%d crc=0x%04X res=0x%04X (%d frames)\n",
                                             vote_crc_ok, vote_fill_ok, vote_rx_crc, vote_res_crc, s->mp0_vote_frames);

                                    if (!vote_crc_ok || !vote_fill_ok)
                                    {
                                        /* Three identical, perfectly framed
                                           but CRC-invalid MP repetitions are
                                           not timing wobble.  They identify a
                                           stable wrong dibit order/domain, and
                                           dropping the lock rather than
                                           re-locking it for the rest of the
                                           Phase 4 deadline is what keeps a
                                           foreign peer from wedging there.
                                           That inference needs the three
                                           frames to have come from one
                                           uninterrupted lock, which is what
                                           the old single accumulator
                                           guaranteed by resetting on every
                                           hypothesis change.  With a bin per
                                           hypothesis they may instead be three
                                           separate locks that happen to share
                                           a hypothesis, and a failed vote then
                                           says nothing about any of them.
                                           Keep the drop for the case it was
                                           written for, and only for that. */
                                        if (s->mp0_vote_same_lock[vh] >= 3)
                                        {
                                            keep_hypothesis = false;
                                            s->mp_phase4_reject_streak = 2;
                                        }
                                        /*endif*/
                                        s->mp0_vote_same_lock[vh] = 0;
                                        s->mp0_vote_frames = 0;
                                        s->mp0_vote_frames_by_hyp[vh] = 0;
                                        memset(s->mp0_vote_counts[vh], 0,
                                               sizeof(s->mp0_vote_counts[vh]));
                                    }
                                    else
                                    {
                                        int accepted_vote_frames;

                                        accepted_vote_frames = s->mp0_vote_frames;
                                        /* Replace frame bits with voted version and accept */
                                        memcpy(s->mp_frame_bits, voted_bits, 88);
                                        crc_good = true;
                                        fill_good = true;
                                        keep_hypothesis = true;
                                        s->mp0_vote_same_lock[vh] = 0;
                                        s->mp0_vote_frames = 0;
                                        s->mp0_vote_frames_by_hyp[vh] = 0;
                                        memset(s->mp0_vote_counts[vh], 0,
                                               sizeof(s->mp0_vote_counts[vh]));

                                        /* Process the accepted frame */
                                        if (s->duplex)
                                        {
                                            mp_pack_for_parser(s->info_buf, s->mp_frame_bits, 0);
                                            process_rx_mp(s, &mp, s->info_buf);
                                            if (mp_semantic_ok_phase4(s, &mp, 0, s->mp_frame_bits))
                                            {
                                                if (mp.mp_acknowledged)
                                                    s->mp_remote_ack_seen = 1;
                                                t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                                                if (mp_apply_parameters(t, &mp))
                                                {
                                                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                                             "Rx - Phase 4: MP0 ACCEPTED via majority vote (%d frames)\n",
                                                             accepted_vote_frames);
                                                    s->mp_seen = 1;
                                                    if (s->mp_accepted_baud == 0)
                                                        s->mp_accepted_baud = s->duration;
                                                    s->mp_early_rejects = 0;
                                                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP frame accepted via vote; awaiting E");
                                                    frame_accepted = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            /*endif*/
                            /* Majority-vote accumulator for MP1 frames */
                            else if (type == 1  &&  s->mp_frame_target == 188)
                            {
                                int vi;

                                if (s->mp_hypothesis != s->mp1_vote_hyp)
                                {
                                    memset(s->mp1_vote_counts, 0, sizeof(s->mp1_vote_counts));
                                    s->mp1_vote_frames = 0;
                                    s->mp1_vote_hyp = s->mp_hypothesis;
                                }

                                for (vi = 0;  vi < 188;  vi++)
                                    s->mp1_vote_counts[vi] += (s->mp_frame_bits[vi] & 1) ? 1 : -1;
                                s->mp1_vote_frames++;

                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP1 vote accumulator: %d frames (hyp=%d)\n",
                                         s->mp1_vote_frames, s->mp1_vote_hyp);

                                if (s->mp1_vote_frames >= 3)
                                {
                                    uint8_t voted_bits[188];
                                    uint16_t vote_rx_crc;
                                    uint16_t vote_res_crc;
                                    bool vote_crc_ok;
                                    bool vote_fill_ok;

                                    for (vi = 0;  vi < 188;  vi++)
                                        voted_bits[vi] = (s->mp1_vote_counts[vi] > 0) ? 1 : 0;
                                    /* Force structural bits */
                                    voted_bits[17] = 0;
                                    voted_bits[18] = 1;
                                    voted_bits[19] = 0;
                                    voted_bits[34] = 0;
                                    voted_bits[51] = 0;
                                    voted_bits[68] = 0;
                                    voted_bits[85] = 0;
                                    voted_bits[102] = 0;
                                    voted_bits[119] = 0;
                                    voted_bits[136] = 0;
                                    voted_bits[153] = 0;
                                    voted_bits[170] = 0;

                                    vote_crc_ok = mp_crc_ok(voted_bits, 1, &vote_rx_crc, &vote_res_crc);
                                    vote_fill_ok = mp_fill_ok(voted_bits, 1);

                                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP1 majority-vote result: crc_ok=%d fill_ok=%d crc=0x%04X res=0x%04X (%d frames)\n",
                                             vote_crc_ok, vote_fill_ok, vote_rx_crc, vote_res_crc, s->mp1_vote_frames);

                                    if (!vote_crc_ok || !vote_fill_ok)
                                    {
                                        keep_hypothesis = false;
                                        s->mp_phase4_reject_streak = 2;
                                        s->mp1_vote_frames = 0;
                                        memset(s->mp1_vote_counts, 0, sizeof(s->mp1_vote_counts));
                                    }
                                    else
                                    {
                                        memcpy(s->mp_frame_bits, voted_bits, 188);
                                        crc_good = true;
                                        fill_good = true;
                                        keep_hypothesis = true;
                                        s->mp1_vote_frames = 0;
                                        memset(s->mp1_vote_counts, 0, sizeof(s->mp1_vote_counts));

                                        if (s->duplex)
                                        {
                                            mp_pack_for_parser(s->info_buf, s->mp_frame_bits, 1);
                                            process_rx_mp(s, &mp, s->info_buf);
                                            if (mp_semantic_ok_phase4(s, &mp, 1, s->mp_frame_bits))
                                            {
                                                if (mp.mp_acknowledged)
                                                    s->mp_remote_ack_seen = 1;
                                                t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                                                if (mp_apply_parameters(t, &mp))
                                                {
                                                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                                             "Rx - Phase 4: MP1 ACCEPTED via majority vote\n");
                                                    s->mp_seen = 1;
                                                    if (s->mp_accepted_baud == 0)
                                                        s->mp_accepted_baud = s->duration;
                                                    s->mp_early_rejects = 0;
                                                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP1 accepted via vote; awaiting E");
                                                    frame_accepted = true;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            /*endif*/
                            if (!frame_accepted  &&  keep_hypothesis)
                            {
                                s->mp_early_rejects = 0;
                                s->mp_count = 0;
                                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: keeping MP hypothesis=%d after CRC-only reject; retrying local preamble reacquire\n",
                                         s->mp_hypothesis);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "CRC-only reject; reacquiring preamble");
                            }
                            else if (!frame_accepted)
                            {
                                /* Bad lock: drop hypothesis and resume global search. */
                                mp_unlock_after_reject(s, true);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "bad lock; resuming global search");
                            }
                            /*endif*/
                        }
                        /*endif*/
                        if (frame_accepted)
                        {
                            s->mp_phase4_reject_streak = 0;
                            /* The frame grid E must fall on if a later frame is
                               lost: bits since this frame's last bit, and its
                               length. */
                            s->mp_bits_since_frame = 0;
                            s->mp_last_frame_bits = s->mp_frame_target;
                        }
                        /*endif*/
                    }
                    if (s->mp_hypothesis >= 0)
                    {
                        s->mp_frame_pos = 0;
                        s->mp_frame_target = 0;
                    }
                    /*endif*/
                }
                /*endif*/
            }
            /*endfor*/
        }
        /*endif*/
        }
        break;

    case V34_RX_STAGE_DATA:
        /* When the T/3 receiver owns the V.90 upstream, only its symbols are
           data.  E is found in the CP bit stream, part way through a sample
           block, and v34_begin_rx_data() moves the stage there and then -- so
           the rest of that block's T/2 symbols used to land here first.
           Against slmodemd that was 19 symbols ahead of B1: every mapping
           frame after them grouped three symbols late (19 mod 8), a clean eye
           decoding to white bits, and no V.42 ODP ever seen
           (artifacts/slm-v90-lapm3).  No data-frame phase can fix that,
           which is why the frame-phase sweep never found one. */
        if (!v34_rx_t2_data_path(s)  &&  !s->v90_t3_in_emit)
            break;
        /*endif*/
        v34_rx_data_symbol(s, sym);
        break;

    default:
        break;
    }
    /*endswitch*/

    /* Decision-directed carrier tracking and equalizer training for DQPSK during
       Phase 3/4 training.  Snap to the nearest QPSK constellation point using a
       FIXED target magnitude (EMA of equalizer output) so the equalizer corrects
       both phase AND amplitude distortion (ISI).  Using the received magnitude as
       the target (as before) gives the equalizer zero amplitude-correction incentive
       and leaves ISI uncorrected. */
    if (v34_rx_stage_is_primary_training(s->stage))
    {
        float mag;

        mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
        if (mag > 0.001f  &&  isfinite(mag)  &&  mag < 100.0f)
        {
            float error;
            float target_mag;

            /* Fixed CMA target radius: QPSK constellation at unit radius.
               Adaptive EMA tracking was causing CMA divergence when it seeded
               from weak first samples (R=0.15), making the equalizer oscillate.
               A fixed target of 1.0 normalizes the equalizer output to a known
               level regardless of input power. */
            target_mag = 1.0f;

            /* Snap to nearest of the 4 QPSK points at the FIXED target radius.
               QPSK points at (±1,±1)/√2 scaled by target_mag. */
            float s2 = target_mag * 0.7071068f;  /* target_mag/√2 */
            eq_target.re = (sym->re >= 0.0f)  ?  s2  :  -s2;
            eq_target.im = (sym->im >= 0.0f)  ?  s2  :  -s2;

            /* Keep the receiver adapting through Phase 3 once TRN is locked.
             *
             * V34_RX_STAGE_PHASE3_WAIT_S spans the peer's whole Phase 3 -- its
             * TRN *and* its Ja -- and both adaptation loops used to be off for
             * the entire stage.  That is sample-and-hold: freeze the receiver,
             * accumulate bits, and batch-search them afterwards.  It is good
             * enough to spot a periodic TRN (measured live, TRN locks at 93%)
             * but not to decode Ja, which is data -- every symbol has to be
             * right, and nothing is correcting residual carrier/gain error
             * over the ~1.5 s of TRN that precedes it.  Live captures show
             * exactly that split: real structure through TRN, then ~50% ones
             * on all 24 hypotheses once Ja starts.
             *
             * Gated on phase3_trn_lock_hyp >= 0 so this only switches on once
             * TRN is locked, i.e. well past the S detector -- S is carried as
             * phase reversals and must not be tracked out. */
            if ((s->stage == V34_RX_STAGE_PHASE4_TRN
                 || v34_rx_stage_is_phase4_frame(s->stage)
                 || (s->stage == V34_RX_STAGE_PHASE3_WAIT_S
                     && s->phase3_tracking_armed
                     && !v34_rx_caller_hearing_own_phase3_m(s, 1)))
                &&
                !(s->stage == V34_RX_STAGE_PHASE3_WAIT_S && phase3_cma_disabled()))
            {
                v34_state_t *t_cma = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                /* V.34 11.4.1.1.2/11.4.1.2.2 conditions the receiver on
                   at least 512T of TRN and then receives MP.  MP is signalling,
                   not an equalizer-training sequence.  Continuing blind CMA
                   while searching for MP (or indefinitely through a delayed
                   TRN lock) can drive converged taps to infinity.  Bound Phase
                   4 refinement to its first 512T and freeze it for all framed
                   Phase-4 signalling. */
                /* Startup CP, taps allowed to adapt -- the same exception
                   reneg_cp_train carries, scoped to the startup seam. */
                bool startup_cp_adapt = (s->stage == V34_RX_STAGE_V90_CP)
                                     && !s->reneg_cp_train
                                     && v90_startup_cp_adapt();
                bool freeze_mp_cma = ((s->stage == V34_RX_STAGE_V90_CP)
                                      || v34_rx_stage_is_phase4_frame(s->stage)
                                      || (s->stage == V34_RX_STAGE_PHASE4_TRN
                                          && s->phase4_trn_after_j >= 512))
                                  && !s->reneg_cp_train
                                  && !startup_cp_adapt;
                /* Once the decision-aided Phase 4 tracker owns the taps
                   (data-aided LMS above), CMA must stand down or the two
                   fight: CMA's phase-blind gradient re-randomizes the phase
                   the DA loop just fixed. */
                /* ...except while 9.6's CP conditioning is still finding the
                   LEVEL.  The DA loop is decision-directed and its decisions
                   are meaningless at 27x the slicer's unit circle, which is
                   where a renegotiation's fresh equalizer starts: measured on
                   artifacts/reneg-eq/reneg-r1 it seeded on the third symbol
                   and stood CMA down for the whole window, leaving |z| at 27
                   and the descrambled SCR at 68% ones.  Level first, then
                   phase. */
                bool da_owns_eq = v34_rx_stage_is_phase4_frame(s->stage)
                               && s->phase4_da_seeded
                               && !s->reneg_cp_train
                               && !startup_cp_adapt;
                /* V.34 11.4: Phase 4 starts from the tap solution 11.3 already
                   trained on PP and TRN.  What that solution needs is a level
                   correction, not more shaping -- Phase 3 leaves |z| ~ 1.47 in
                   this receiver's units and the Phase 4 slicer expects the unit
                   circle.  Blind CMA supplies the level, but its phase-blind
                   per-tap gradient keeps walking the trained solution after the
                   level is right, and above 2400 baud it walks it off: the
                   Phase 4 TRN hypothesis search then reads a flat 50% ones for
                   the rest of the call.  Let it converge, then stop it. */
                bool p4_trn_trusted = true;

                if (s->stage == V34_RX_STAGE_PHASE4_TRN
                    &&  (!s->v90_mode  ||  s->v90_v34_fallback)
                    &&  !t_cma->tx.reneg_active   /* 11.6's TRN is short and re-seeded */
                    &&  v34_p4_trn_coh_gate() > 0.0f)
                {
                    /* Decision-directed training is only worth anything while the
                       decisions are right.  TRN's symbols are on the 4-point
                       grid, so the fourth power of an equalizer output that is
                       locked is a steady phase: track its coherence. */
                    float mag2 = sym->re*sym->re + sym->im*sym->im;

                    if (mag2 > 1e-6f)
                    {
                        float a = sym->re*sym->re - sym->im*sym->im;
                        float b = 2.0f*sym->re*sym->im;
                        float re4 = (a*a - b*b)/(mag2*mag2);
                        float im4 = (2.0f*a*b)/(mag2*mag2);

                        s->phase4_trn_coh_re += 0.02f*(re4 - s->phase4_trn_coh_re);
                        s->phase4_trn_coh_im += 0.02f*(im4 - s->phase4_trn_coh_im);
                    }
                    /*endif*/
                    p4_trn_trusted = sqrtf(s->phase4_trn_coh_re*s->phase4_trn_coh_re
                                         + s->phase4_trn_coh_im*s->phase4_trn_coh_im)
                                   >= v34_p4_trn_coh_gate();
                    if (!p4_trn_trusted)
                        freeze_mp_cma = false;
                    /*endif*/
                }
                /*endif*/
                if (s->stage == V34_RX_STAGE_PHASE4_TRN
                    &&  (!s->v90_mode  ||  s->v90_v34_fallback)
                    &&  s->phase4_trn_after_j >= V34_P4_TRN_DD_START
                    &&  p4_trn_trusted)
                {
                    /* V.34 11.4.1.1.2/11.4.1.2.2: TRN is sent "until the
                       receiver is trained adequately", and it is the
                       receiver's last known-constellation training before
                       MP and data.  Blind CMA stops after it has found the
                       level (above) and nothing else adapted here, so the
                       Phase 3 tap solution was carried unchanged across the
                       seam and the far end's echo-free Phase 3 channel no
                       longer matched a duplex Phase 4: against the RasFinder
                       its TRN sat 19 degrees from the grid with |z| spread
                       0.235, and B1 then matched at 0.997 (~23 dB).  Decision-
                       directed LMS on TRN's own 4-point decisions for the rest
                       of the segment takes the same recording to 1.1 degrees,
                       |z| spread 0.015 and B1 at 1.000 (32 dB)
                       (rf-tower-v34b-1).  V34_P4_TRN_DD_START is the TRN
                       symbol it starts at. */
                    v34_rx_tune_equalizer(s, sym, &eq_target);
                }
                else if (!t_cma->tx.tx_data_mode && !freeze_mp_cma && !da_owns_eq)
                {
                    if (s->reneg_cp_train)
                    {
                        if (!v90_reneg_cma_converged(s, sym))
                            tune_equalizer_cma(s, sym);
                        /*endif*/
                    }
                    else if (!p4_trn_trusted  ||  !phase4_cma_converged(s, sym))
                        tune_equalizer_cma(s, sym);
                    /*endif*/
                }
                /*endif*/
            }
            /*endif*/

            /* Re-enabled carrier tracking — test 4 showed MP detection worked
               better with carrier tracking on.  CMA equalization now provides
               more stable magnitude for eq_target, improving tracking quality. */
            if (!((s->v90_mode || s->hdx_primary_resync)
                  && s->stage == V34_RX_STAGE_PHASE3_TRAINING
                  && (!s->phase3_pp_started
                      || s->duration <= PHASE3_PP_TRAIN_BAUDS))
                && (s->stage != V34_RX_STAGE_PHASE3_WAIT_S
                 || (s->phase3_tracking_armed
                     && !v34_rx_caller_hearing_own_phase3_m(s, 2)))
                && !phase4_trn_should_freeze_tracking(s))
            {
                error = sym->im*eq_target.re - sym->re*eq_target.im;
                s->v34_carrier_phase_rate += (int32_t)(s->carrier_track_i*error);
                s->carrier_phase += (int32_t)(s->carrier_track_p*error);
            }
            }
        /*endif*/
    }
    /*endif*/

    s->last_sample = *sym;
}
/*- End of function --------------------------------------------------------*/

/* Which of the two T/2 outputs is the symbol instant.
 *
 * The band-edge timing detector recovers the symbol *rate* and pulls the
 * sampling phase within a T/2 interval, but it cannot resolve which of the two
 * T/2 outputs per symbol is the eye centre -- that ambiguity is settled purely
 * by how many T/2 intervals have been generated since the receiver started,
 * and a single +/-1 correction from the loop can insert or delete one and flip
 * it for the rest of the call.  Nothing here ever checked.
 *
 * Measured over the harness at 9600 bit/s, sweeping a deliberate offset across
 * a whole symbol: at 2400 baud the eye is at zero offset (normalised scatter
 * 0.384) and one T/2 away is the worst point in the sweep (0.590), while at
 * 3200 baud it is the other way round -- zero offset is the worst point of the
 * whole sweep (0.595) and one T/2 away is the best (0.293).  So the receiver
 * had been sampling 3200 baud Phase 4 at the eye crossing.  Which way a given
 * call lands is a coin flip on sample counts, which is why the failures looked
 * marginal and why every change reshuffled which rows passed.
 *
 * All the training signals V.34 uses here -- S, S-bar, PP, TRN and MP -- are
 * constant modulus, so the eye centre is simply where the equalized output is
 * largest.  It also minimises the spread of that output, and
 * mean(|z|)^2/mean(|z|^2) is in principle the sharper of the two statistics --
 * it is 1 for a constant modulus and falls away either side, where the mean
 * alone separates the phases at 3429 baud by as little as 1.05.  Measured over
 * 24 rows it is markedly worse (12 zero-error rows against 18), so the mean is
 * what is used.  Requiring consecutive windows to agree before flipping is
 * likewise worse (11 rows), because Phase 3 is short and a second 256-symbol
 * window puts the decision after the point where it can still help.  Evaluate both phases during training and take the bigger, with a
 * margin and a minimum observation so noise cannot flip it, and a cap on the
 * number of flips so it cannot oscillate. */
#define V34_EYE_OBSERVE_SYMBOLS         256
#define V34_EYE_MARGIN                  1.05f
#define V34_EYE_MAX_FLIPS               4
#define V34_EYE_MIN_MAG                 0.10f
#define V34_EYE_VOTES                   1

/* V.90 digital modem, CP stage: judge the T/2 eye by the DIFFERENTIAL angle
   of each phase rather than by its magnitude.  CPt is 4-point DPSK (V.90
   8.5.2 via V.34 10.1.3.9), so at the eye centre consecutive symbols differ
   by a multiple of 90 degrees; off it they do not.  The magnitude vote is
   biased there: the equalizer is frozen from Phase 3 and was trained at the
   CURRENT phase, so |z| favours staying put, and against the RasFinder the
   one CP-stage decision it took was an 8% call (264.9 vs 244.6, replay of
   rf-tower-shp-ld0-23988). */
static int v34_v90_cp_eye_angle_enabled(void)
{
    return 1;
}
/*- End of function --------------------------------------------------------*/

static float v34_eye_diff_angle_err2(complexf_t z, complexf_t *prev)
{
    float err;
    float a;

    if (prev->re == 0.0f  &&  prev->im == 0.0f)
    {
        *prev = z;
        return -1.0f;
    }
    /*endif*/
    a = atan2f(z.im*prev->re - z.re*prev->im, z.re*prev->re + z.im*prev->im);
    err = a - (float) (M_PI/2.0)*rintf(a/(float) (M_PI/2.0));
    *prev = z;
    return err*err;
}
/*- End of function --------------------------------------------------------*/

static int v34_eye_select_enabled(void)
{
    return 1;
}

static int v34_eye_pp_guard_enabled(void)
{
    return 1;
}

static int v34_pp_onset_trim_enabled(void)
{
    return 1;
}

/* ME_V34_P4_TRN_COH: Phase 4 TRN decision-directed training waits for the
   equalizer output's 4th-power coherence (0 = random phase, 1 = locked on the
   4-point grid) to reach this; below it blind CMA keeps running instead of the
   equalizer being frozen.  Default 0.35, 0 disables.

   Why: the DD-LMS starts at TRN symbol 256 whatever the decisions are worth.
   On the alignments where the receiver is not yet locked (3000/28800 at 20 dB
   echo, delays 0, 4, 17; coherence 0.02-0.08 for the whole 4800-symbol TRN, the
   main tap decaying 0.82 -> 0.42, Phase 4 TRN SNR 3.0 dB) it trained on wrong
   decisions and diverged; on the neighbouring alignment the same receiver is at
   0.97 within 512 symbols.  Over delays 0-27 at six rows this default is equal
   to or one run better than no gate everywhere (target row 8/10 -> 9/10, and
   24/30 -> 28/30 over delays 0-29 at 0.5), and 0.25 happened to lose 3429/21600
   A-law at zero delay, so the value is not a fine-tuned one.  Not the PP
   conditioning: a late PP lock at the same rows is real (docs), but trimming it
   for plain V.34 trades one failure for another (2743/26400 A-law loses a
   payload burst). */
static float v34_p4_trn_coh_gate(void)
{
    static float cache = -1.0f;

    if (cache < 0.0f)
    {
        const char *value = getenv("ME_V34_P4_TRN_COH");

        cache = value  ?  strtof(value, NULL)  :  0.35f;
    }
    /*endif*/
    return cache;
}

static bool v34_eye_pp_defer_enabled(void)
{
    return true;
}

/* The PP-deferred flip applies to the V.90 digital receiver only. */
static bool v34_eye_pp_defer_all(void)
{
    return false;
}

static void process_primary_half_baud(v34_rx_state_t *s, const complexf_t *sample)
{
    complexf_t eq_sample;
    bool eye_check;
    bool eye_hold;

    /* Ordinary V.34 uses the historical T/2 front end.  The V.90 DATA-only
       T/3 branch calls process_primary_symbol() directly after its supervised
       B1 equalizer, so both paths share the mapper and protocol state. */
    /* Retain both T/2 phases for supervised B1 conditioning and replay:
       12.5.2 in HDX and 11.4 in the dense full-duplex path. */
    if ((!s->duplex && s->half_duplex_state == V34_HALF_DUPLEX_PRIMARY_CHANNEL)
        || v34_rx_b1_batch_eq_enabled(s))
    {
        s->v90_t3_raw[s->v90_t3_raw_count & V34_V90_T3_RAW_MASK] = *sample;
        s->v90_t3_raw_count++;
    }
    s->eq_buf[s->eq_step] = *sample;
    s->eq_step = (s->eq_step + 1) & V34_EQUALIZER_MASK;
    /* Only while there is something on the line.  V.34 11.3.1.2.4 has the
       answer modem go silent for the whole of the call modem's Phase 3, and
       two near-zero sums differ by whatever noise decides -- which is a
       coin-flip chance of moving the symbol instant to the wrong phase just
       before the far end's Phase 4 S arrives. */
    eye_check = (v34_rx_stage_is_primary_training(s->stage)
                 &&  s->eye_flips < V34_EYE_MAX_FLIPS
                 &&  v34_eye_select_enabled()
                 &&  !v34_rx_caller_hearing_own_phase3_m(s, 4));
    /* V.90 digital modem: once PP conditioning has begun, leave the symbol
       instant alone until Ja has been accepted.  PP trains the equalizer at the
       instant it ran on, and a move afterwards lands on that equalizer --
       the harm the plain-V.34 note below already measured.  Against the
       RasFinder over a clean wired path (artifacts/rf-tower-scan1-1, -4:
       low carrier, PP locked late) the chooser moved the instant just after
       PP (off 351.8 vs on 332.4, a 6% call), the Ja that followed never
       demodulated, Sd went out on the 6 s fallback and the peer retrained;
       without the move both parse the Ja descriptor, and nine calls that
       already parsed (tower and Wi-Fi era, both carriers) still do.  S no
       longer depends on the eye either: v34_rx_watch_v90_jd_s() finds it on
       the line.  After Ja the chooser runs again: on the calls where the
       peer used the high carrier its flip lands during DIL and is what lets
       Phase 4 decode the CPt -- freezing to the end of Phase 3 loses it
       (rf-tower-scan1-6, rf-tower-nop3-1: 4 CPt -> 0), and so does disabling
       the chooser outright. */
    if (eye_check
        &&  s->v90_mode
        &&  !s->calling_party
        &&  !s->v90_v34_fallback  /* no Ja to release it in a V.34 fallback */
        &&  s->phase3_pp_started
        &&  !s->v90_p3_eye_released
        &&  (s->stage == V34_RX_STAGE_PHASE3_TRAINING
             ||  s->stage == V34_RX_STAGE_PHASE3_WAIT_S))
    {
        eye_check = false;
        s->eye_flip_pending = false;
    }
    /*endif*/
    /* Never move the symbol instant while 10.1.3.6's PP is being conditioned
       on.  That
                    stage is supervised against a known 232-baud sequence with
                    the AGC frozen, so moving the symbol instant part way
                    through invalidates every sample after the move and the
                    equalizer is trained on the wreckage.  Measured at 3429
                    baud, where 8000/3429 leaves the two T/2 phases only 1.17
                    samples apart and the decision is closest: a flip landing
                    inside the window takes the PP mean residual from 0.192 to
                    0.805, after which TRN never locks (55% ones against 80%),
                    the far end's J never decodes exactly (d4=3 where a healthy
                    call reads d4=0) and Phase 3 deadlocks.  2743 baud flips
                    just *before* PP starts and must keep doing so, which is
                    why this is scoped to the window rather than to a vote
                    count: requiring two agreeing windows instead costs
                    2743/9600 both laws. */
    eye_hold = (v34_eye_pp_guard_enabled()
                &&  s->phase3_pp_started
                &&  (v34_eye_pp_guard_enabled() > 1
                     ||  s->duration <= PHASE3_PP_TRAIN_BAUDS));
    /* Act on a flip the PP guard deferred, at the first half-baud after PP.
       The guard's own rationale is "measure during PP, act after it", but
       the decision used to be discarded, so the move waited for a fresh vote
       -- and on a peer whose Ja and post-Ja silence give no clear vote, the
       next one comes from the analogue modem's 9.3.2.7 S itself.  S is 128T,
       half a 256-symbol eye window, so whether that late flip lands in time
       to see S depended on where the window boundaries fell.  Measured on
       artifacts/rf-jafb1000-20260929-r2 (RasFinder, 3200 baud high
       carrier): replayed in 160-sample blocks the flip landed one window
       before S and S was detected; in the two 80-sample blocks per 20 ms a
       live pjmedia call delivers (ME_IO_SCHEDULE) it landed after, the
       detector read S at the eye crossing as 180-degree steps (dom=2 25/32
       against alt 30/32), and the call was lost -- which is what happened
       live. */
    if (s->eye_flip_pending  &&  !eye_hold  &&  eye_check)
    {
        s->eye_flip_pending = false;
        s->eye_flips++;
        s->eye_votes = 0;
        s->eye_on_sum = 0.0f;
        s->eye_off_sum = 0.0f;
        s->eye_n = 0;
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - T/2 eye: PP over, applying the flip measured during it (flip %d)\n",
                 s->eye_flips);
        s->baud_half ^= 1;
    }
    /*endif*/
    if ((s->baud_half ^= 1))
    {
        if (eye_check)
        {
            complexf_t off = equalizer_get(s);

            s->eye_off_sum += sqrtf(off.re*off.re + off.im*off.im);
            if (s->stage == V34_RX_STAGE_V90_CP  &&  v34_v90_cp_eye_angle_enabled())
            {
                float e = v34_eye_diff_angle_err2(off, &s->eye_prev_off);

                if (e >= 0.0f)
                    s->eye_off_aerr += e;
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/
        return;
    }
    /*endif*/
    if (!v34_rx_caller_hearing_own_phase3_m(s, 8)
        &&  !(s->stage == V34_RX_STAGE_DATA  &&  v34_data_timing_diag_freeze()))
        pri_symbol_sync(s);
    /*endif*/
    eq_sample = equalizer_get(s);
    s->p3_echo_mag_ema += 0.05f*(sqrtf(eq_sample.re*eq_sample.re + eq_sample.im*eq_sample.im)
                                 - s->p3_echo_mag_ema);
    if (s->p3_echo_mag_ema > 0.02f  &&  s->p3_echo_mag_ema < 0.5f)
        s->p3_echo_run++;
    else
        s->p3_echo_run = 0;
    /*endif*/
    if (eye_check)
    {
        s->eye_on_sum += sqrtf(eq_sample.re*eq_sample.re + eq_sample.im*eq_sample.im);
        if (s->stage == V34_RX_STAGE_V90_CP  &&  v34_v90_cp_eye_angle_enabled())
        {
            float e = v34_eye_diff_angle_err2(eq_sample, &s->eye_prev_on);

            if (e >= 0.0f)
                s->eye_on_aerr += e;
            /*endif*/
        }
        /*endif*/
        /* 12.5.1 gives only 128T of S before S-bar/PP. Reacquire the T/2
           eye within that signal, rather than after a 256-symbol window. */
        int eye_window = ((s->hdx_primary_resync && s->stage == V34_RX_STAGE_PHASE4_S)
                          || (!s->duplex && s->stage == V34_RX_STAGE_PHASE3_WAIT_S))
                         ? 64 : V34_EYE_OBSERVE_SYMBOLS;
        if (++s->eye_n >= eye_window)
        {
            bool cp_angle = (s->stage == V34_RX_STAGE_V90_CP
                             &&  v34_v90_cp_eye_angle_enabled());
            /* 26 degrees rms is a signal unrelated to the 90-degree grid; a
               phase already under 15 is on the eye and is left alone, and the
               other must be at least twice as good (in squared error) to win. */
            float on_rms_deg = sqrtf(s->eye_on_aerr/(float) s->eye_n)*(float) (180.0/M_PI);

            /* Only decide on signal.  V.34 11.3.1.2.4 has the answer modem
               silent for the whole of the call modem's Phase 3, and two
               near-zero sums differ by whatever noise decides -- measured at
               3429 baud, a flip was taken on sums of 15.5 against 13.0, a mean
               |z| of 0.06, which is silence.  Both phases are compared after
               the same AGC and equalizer, so their own magnitude is the test
               to use, not the line power meter: it is the quantity the
               decision is actually made on. */
            if (s->eye_on_sum + s->eye_off_sum
                    > 2.0f*V34_EYE_MIN_MAG*eye_window
                &&
                (cp_angle
                 ?  (on_rms_deg > 15.0f  &&  2.0f*s->eye_off_aerr < s->eye_on_aerr)
                 :  (s->eye_off_sum > V34_EYE_MARGIN*s->eye_on_sum)))
                s->eye_votes++;
            else
                s->eye_votes = 0;
            /*endif*/
            if (cp_angle)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - V.90 CP T/2 eye: differential angle rms on %.1f deg, off %.1f deg\n",
                         (double) on_rms_deg,
                         (double) (sqrtf(s->eye_off_aerr/(float) s->eye_n)*(float) (180.0/M_PI)));
            }
            /*endif*/
            /* Consecutive windows must agree.  At 3429 baud the two phases sit
               1.17 samples apart and a single window decided by ratios of 1.05
               to 1.19, so the receiver flipped four times in one call and
               finished wherever the cap left it. */
            if (s->eye_votes >= V34_EYE_VOTES  &&  eye_hold)
            {
                /* Measured, and not acted on until PP is over.  See above. */
                s->eye_votes = 0;
                /* Scoped to the V.90 digital modem's receiver, where the
                   failure was measured.  Plain V.34 is measured the other
                   way: applying the deferred flip after PP costs 2800/21600
                   u-law its whole payload (0 bits against 16421), because
                   the move lands on an equalizer PP has just trained at the
                   old instant. */
                s->eye_flip_pending = ((s->v90_mode  &&  !s->calling_party
                                         &&  !s->v90_v34_fallback)
                                        ||  v34_eye_pp_defer_all())
                                       &&  v34_eye_pp_defer_enabled();
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - T/2 eye favours the other phase (off %.1f vs on %.1f) "
                         "but PP is being conditioned on; not moving the symbol instant\n",
                         (double) s->eye_off_sum, (double) s->eye_on_sum);
            }
            else if (s->eye_votes >= V34_EYE_VOTES)
            {
                s->eye_votes = 0;
                s->eye_flips++;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - T/2 eye is on the other phase (off %.1f vs on %.1f over %d symbols); "
                         "moving the symbol instant (flip %d)\n",
                         (double) s->eye_off_sum, (double) s->eye_on_sum,
                         s->eye_n, s->eye_flips);
                s->baud_half ^= 1;
                s->eye_prev_on = complex_setf(0.0f, 0.0f);
                s->eye_prev_off = complex_setf(0.0f, 0.0f);
                if (s->stage == V34_RX_STAGE_PHASE4_TRN)
                {
                    /* The taps were trained, and CMA declared settled, with the
                       symbol instant where it was.  Half a symbol later they
                       are wrong, and a frozen wrong solution is what the
                       decision-directed LMS then fed on: caller 4th-power
                       coherence 0.02-0.08 for the whole of an 8 s Phase 4 TRN,
                       its main tap decaying 0.82 -> 0.42, against 0.97 within
                       512 symbols on the neighbouring alignment.  Let blind CMA
                       find the level again. */
                    s->phase4_cma_settled = 0;
                    s->phase4_cma_bauds = 0;
                    s->phase4_cma_mag = 0.0f;
                }
                /*endif*/
            }
            /*endif*/
            s->eye_on_sum = 0.0f;
            s->eye_off_sum = 0.0f;
            s->eye_on_aerr = 0.0f;
            s->eye_off_aerr = 0.0f;
            s->eye_n = 0;
        }
        /*endif*/
    }
    /*endif*/
    process_primary_symbol(s, &eq_sample);
}
/*- End of function --------------------------------------------------------*/

/* Solve the small real normal equation used by supervised B1 acquisition. */
static bool v90_t3_solve(double *a, double *b, double *x, int n)
{
    int i;
    int j;
    int k;

    for (i = 0;  i < n;  i++)
    {
        int pivot = i;
        double best = fabs(a[i*n + i]);

        for (j = i + 1;  j < n;  j++)
        {
            double v = fabs(a[j*n + i]);
            if (v > best)
            {
                best = v;
                pivot = j;
            }
        }
        if (!isfinite(best) || best < 1e-12)
            return false;
        if (pivot != i)
        {
            for (k = i;  k < n;  k++)
            {
                double t = a[i*n + k];
                a[i*n + k] = a[pivot*n + k];
                a[pivot*n + k] = t;
            }
            {
                double t = b[i];
                b[i] = b[pivot];
                b[pivot] = t;
            }
        }
        {
            double d = a[i*n + i];
            for (k = i;  k < n;  k++)
                a[i*n + k] /= d;
            b[i] /= d;
        }
        for (j = 0;  j < n;  j++)
        {
            double f;
            if (j == i)
                continue;
            f = a[j*n + i];
            if (f == 0.0)
                continue;
            for (k = i;  k < n;  k++)
                a[j*n + k] -= f*a[i*n + k];
            b[j] -= f*b[i];
        }
    }
    for (i = 0;  i < n;  i++)
        x[i] = b[i];
    return true;
}

/* V.34 11.4.1.1.5/11.4.1.2.5 conditions the receiver on the complete B1
   frame.  Scalar calibration alone leaves the dense 3429-baud modes exposed
   to the Phase-4 tap solution and acquisition phase.  The known 10.1.3.1
   symbols permit a supervised fit before any user decisions steer the taps.
   Keep the proven lower-rate and V.90 paths on their existing conditioning. */
bool v34_rx_b1_batch_eq_enabled(const v34_rx_state_t *s)
{
    /* bit_rate encodes 2*(N - 1): N >= 13 means 31200/33600 bit/s. */
    return s->duplex && !s->v90_mode
        && s->baud_rate == V34_BAUD_RATE_3429 && s->bit_rate >= 24;
}

bool v34_rx_data_mm_timing_enabled(const v34_rx_state_t *s)
{
    return v34_rx_b1_batch_eq_enabled(s);
}

/* 11.4/12.5.2: use B1's known data constellation to remove residual ISI left
   by the periodic PP reference. Fit a symbol-spaced correction to the
   existing FSE, using the same acquisition aperture as the T/3 receiver. The detected
   B1 offset also moves the raw anchor. The FSE group delay supplies the
   samples beyond B1 needed by the centred fit before payload is replayed. */
void v34_rx_condition_b1_equalizer(v34_rx_state_t *s, float gain, float phase)
{
    enum { T = V34_V90_T3_FSE_TAPS, N = 2*T,
           E = V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN };
    double matrix[N*N] = {0}, rhs[N] = {0}, solution[N];
    complexf_t coeff[E];
    int pre = T/2, length = s->v90_t3_b1_symbols;
    double trace = 0.0;
    double rate = s->phase4_da_derot_rate*(M_PI/2147483648.0);
    if ((s->duplex && !v34_rx_b1_batch_eq_enabled(s))
        || s->data_symbol_conjugate || length <= 2*T
        || s->v90_t3_e_anchor < 0
        || s->v90_t3_e_anchor < s->v90_t3_raw_count - V34_V90_T3_RAW_SIZE
        || s->v90_t3_e_anchor + 2*(length - pre - 1) + pre >= s->v90_t3_raw_count)
        return;
    for (int n = pre; n < length - pre; n++)
    {
        double fr[N], fi[N];
        double a = phase - rate*(length - 1 - n);
        complexf_t e = s->v90_t3_b1[n];
        double yr = gain*(e.re*cos(a) - e.im*sin(a));
        double yi = gain*(e.re*sin(a) + e.im*cos(a));
        for (int k = 0; k < T; k++)
        {
            complexf_t o = s->v90_t3_raw[(s->v90_t3_e_anchor + 2*n + pre - k)
                                         & V34_V90_T3_RAW_MASK];
            fr[2*k] = o.re;
            fr[2*k + 1] = -o.im;
            fi[2*k] = o.im;
            fi[2*k + 1] = o.re;
        }
        for (int r = 0; r < N; r++)
        {
            rhs[r] += fr[r]*yr + fi[r]*yi;
            for (int c = 0; c < N; c++)
                matrix[r*N + c] += fr[r]*fr[c] + fi[r]*fi[c];
        }
    }
    for (int k = 0; k < N; k++) trace += matrix[k*N + k];
    for (int k = 0; k < N; k++) matrix[k*N + k] += trace*1e-6;
    if (!v90_t3_solve(matrix, rhs, solution, N))
        return;
    for (int k = 0; k < N; k++)
        if (!isfinite(solution[k]))
            return;
    memset(coeff, 0, sizeof(coeff));
    for (int k = 0; k < T; k++)
        coeff[V34_EQUALIZER_PRE_LEN + k - pre] =
            complex_setf(solution[2*k], solution[2*k + 1]);
    memcpy(s->eq_coeff, coeff, sizeof(coeff));
}

complexf_t v34_rx_b1_equalized_symbol(const v34_rx_state_t *s, int symbol)
{
    complexf_t y = {0.0f, 0.0f};
    for (int j = 0; j < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN; j++)
    {
        int64_t idx = s->v90_t3_e_anchor + 2*symbol + V34_EQUALIZER_PRE_LEN - j;
        complexf_t o = s->v90_t3_raw[idx & V34_V90_T3_RAW_MASK];
        complexf_t v = complex_mulf(&s->eq_coeff[j], &o);
        y.re += v.re;
        y.im += v.im;
    }
    return y;
}

static complexf_t v90_t3_raw_get(const v34_rx_state_t *s, int64_t index)
{
    complexf_t z = {0.0f, 0.0f};

    if (index >= 0 && index < s->v90_t3_raw_count
        && index >= s->v90_t3_raw_count - V34_V90_T3_RAW_SIZE)
        z = s->v90_t3_raw[index & V34_V90_T3_RAW_MASK];
    return z;
}

/* The raw stream at a fractional sample position.  The timing loop corrects
   in whole samples and keeps the leftover fraction; applying that fraction
   here is what gives the loop a continuous actuator, and without it the
   residual timing error can never be smaller than half a sample -- a sixth
   of a symbol -- which is enough to keep the loop's integrator permanently
   wound up.  Linear interpolation is adequate: the signal is already
   band-limited by the receive RRC, and the fractionally spaced equalizer
   downstream adapts around whatever this leaves. */
static complexf_t v90_t3_raw_get_frac(const v34_rx_state_t *s,
                                      int64_t index,
                                      float frac)
{
    complexf_t a;
    complexf_t b;
    complexf_t z;

    if (frac < 0.0f)
    {
        index--;
        frac += 1.0f;
    }
    /*endif*/
    a = v90_t3_raw_get(s, index);
    if (frac <= 1e-4f)
        return a;
    /*endif*/
    b = v90_t3_raw_get(s, index + 1);
    z.re = a.re + frac*(b.re - a.re);
    z.im = a.im + frac*(b.im - a.im);
    return z;
}

static complexf_t v90_t3_matched_get(const v34_rx_state_t *s, int64_t index)
{
    complexf_t z = {0.0f, 0.0f};

    if (index >= 0 && index < s->v90_t3_raw_count
        && index >= s->v90_t3_raw_count - V34_V90_T3_RAW_SIZE)
        z = s->v90_t3_matched[index & V34_V90_T3_RAW_MASK];
    return z;
}

/* The matched-filtered stream at a fractional sample position.  The timing
   detector has to measure the instant the receiver is actually using,
   fraction included, or it reports an error the data path does not have. */
static complexf_t v90_t3_matched_get_frac(const v34_rx_state_t *s,
                                          int64_t index,
                                          float frac)
{
    complexf_t a;
    complexf_t b;
    complexf_t z;

    if (frac < 0.0f)
    {
        index--;
        frac += 1.0f;
    }
    /*endif*/
    a = v90_t3_matched_get(s, index);
    if (frac <= 1e-4f)
        return a;
    /*endif*/
    b = v90_t3_matched_get(s, index + 1);
    z.re = a.re + frac*(b.re - a.re);
    z.im = a.im + frac*(b.im - a.im);
    return z;
}

static void v90_t3_make_rrc(v34_rx_state_t *s)
{
    const double beta = 0.12;       /* V.34 §9: 12% excess bandwidth. */
    const int centre = V34_V90_T3_RRC_TAPS/2;
    double power = 0.0;
    int i;

    for (i = 0;  i < V34_V90_T3_RRC_TAPS;  i++)
    {
        double t = (i - centre)/3.0;
        double h;

        if (fabs(t) < 1e-10)
            h = 1.0 - beta + 4.0*beta/M_PI;
        else if (fabs(fabs(4.0*beta*t) - 1.0) < 1e-8)
            h = beta/sqrt(2.0)*((1.0 + 2.0/M_PI)*sin(M_PI/(4.0*beta))
                              + (1.0 - 2.0/M_PI)*cos(M_PI/(4.0*beta)));
        else
            h = (sin(M_PI*t*(1.0 - beta))
                 + 4.0*beta*t*cos(M_PI*t*(1.0 + beta)))
              / (M_PI*t*(1.0 - 16.0*beta*beta*t*t));
        s->v90_t3_rrc_coeff[i] = (float)h;
        power += h*h;
    }
    power = sqrt(power);
    for (i = 0;  i < V34_V90_T3_RRC_TAPS;  i++)
        s->v90_t3_rrc_coeff[i] /= (float)power;
#if defined(V34_FIXED_POINT)
    for (int tap = 0;  tap < V34_V90_T3_RRC_TAPS;  tap++)
        s->v90_t3_rrc_coeff_fx[tap] = v34_fx_from_float(s->v90_t3_rrc_coeff[tap], V34_FX_TAP_SHIFT);
    /*endfor*/
    memset(s->v90_t3_rrc_fx, 0, sizeof(s->v90_t3_rrc_fx));
    v34_fx_nco_init(&s->v90_t3_nco,
                    -(double) carrier_frequency(s->baud_rate, s->high_carrier),
                    (double) s->v90_t3_internal_rate);
#endif
}

static float v90_t3_coarse_score(v34_rx_state_t *s, int64_t first,
                                 bool conjugate)
{
    double cross_re = 0.0;
    double cross_im = 0.0;
    double input_power = 0.0;
    double target_power = 0.0;
    int n;

    for (n = 0;  n < s->v90_t3_b1_symbols;  n++)
    {
        complexf_t x = v90_t3_matched_get(s, first + 3*n);
        complexf_t y = s->v90_t3_b1[n];
        if (conjugate)
            x.im = -x.im;
        cross_re += y.re*x.re + y.im*x.im;
        cross_im += y.im*x.re - y.re*x.im;
        input_power += x.re*x.re + x.im*x.im;
        target_power += y.re*y.re + y.im*y.im;
    }
    if (input_power < 1e-12 || target_power < 1e-12)
        return 0.0f;
    return (float)((cross_re*cross_re + cross_im*cross_im)
                   /(input_power*target_power));
}

static float v90_t3_fit(v34_rx_state_t *s, int64_t first, bool conjugate,
                        complexf_t coeff[V34_V90_T3_FSE_TAPS])
{
    enum { N = 2*V34_V90_T3_FSE_TAPS };
    double matrix[N*N];
    double rhs[N];
    double solution[N];
    int train = s->v90_t3_b1_symbols;
    int pre = V34_V90_T3_FSE_TAPS/2;
    double trace = 0.0;
    double error = 0.0;
    double power = 0.0;
    int n;
    int row;
    int column;

    memset(matrix, 0, sizeof(matrix));
    memset(rhs, 0, sizeof(rhs));
    for (n = 0;  n < train;  n++)
    {
        double fr[N];
        double fi[N];
        for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
        {
            complexf_t x = v90_t3_raw_get(s, first + 3*n - pre + tap);
            if (conjugate)
                x.im = -x.im;
            fr[2*tap] = x.re;
            fr[2*tap + 1] = -x.im;
            fi[2*tap] = x.im;
            fi[2*tap + 1] = x.re;
        }
        for (row = 0;  row < N;  row++)
        {
            rhs[row] += fr[row]*s->v90_t3_b1[n].re
                      + fi[row]*s->v90_t3_b1[n].im;
            for (column = 0;  column < N;  column++)
                matrix[row*N + column] += fr[row]*fr[column]
                                        + fi[row]*fi[column];
        }
    }
    for (row = 0;  row < N;  row++)
        trace += matrix[row*N + row];
    for (row = 0;  row < N;  row++)
        matrix[row*N + row] += trace*1e-6;
    if (!v90_t3_solve(matrix, rhs, solution, N))
        return 0.0f;
    for (n = 0;  n < s->v90_t3_b1_symbols;  n++)
    {
        double yr = 0.0;
        double yi = 0.0;
        for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
        {
            complexf_t x = v90_t3_raw_get(s, first + 3*n - pre + tap);
            double ar = solution[2*tap];
            double ai = solution[2*tap + 1];
            if (conjugate)
                x.im = -x.im;
            yr += ar*x.re - ai*x.im;
            yi += ar*x.im + ai*x.re;
        }
        error += (yr - s->v90_t3_b1[n].re)*(yr - s->v90_t3_b1[n].re)
               + (yi - s->v90_t3_b1[n].im)*(yi - s->v90_t3_b1[n].im);
        power += s->v90_t3_b1[n].re*s->v90_t3_b1[n].re
               + s->v90_t3_b1[n].im*s->v90_t3_b1[n].im;
    }
    if (power <= 0.0 || error >= power)
        return 0.0f;
    for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
    {
        coeff[tap].re = (float)solution[2*tap];
        coeff[tap].im = (float)solution[2*tap + 1];
    }
    return (float)(1.0 - error/power);
}

/*! The carrier rotation that comes with reading the T/3 ring `offset`
    samples further on, in radians.

    The ring holds complex baseband, mixed down by an angle indexed on the
    ABSOLUTE sample count -- see v90_t3_put_sample().  A slip in the peer's
    stream is a delay of the passband signal, and a passband delay is a
    baseband delay TIMES a phase: with the received analytic signal
    a(n) = b(n)e^(jwn), a delay of D gives ring'(n) = b(n-D)e^(-jwD), so
    reading D samples further on recovers the right baseband sample rotated
    by -wD.  Correcting the timing without the phase leaves the constellation
    turned by 82 degrees at 3200 baud low carrier, which is why the slip
    search used to come back with every offset scoring the same 0.65: the
    right sampling instant was in the list and still did not fit the lattice.

    Only the residual matters, since V.34's differential mapping makes a
    90 degree rotation harmless -- and 82 is close enough to 90 that the
    fourth-power carrier estimator sometimes pulled a call back on its own,
    which is exactly why recovery looked like a lottery. */
static float v90_t3_offset_rotation(const v34_rx_state_t *s, float offset)
{
    return 2.0f*3.14159265f
         * carrier_frequency(s->baud_rate, s->high_carrier)
         * offset/s->v90_t3_internal_rate;
}
/*- End of function --------------------------------------------------------*/

/*! Score a candidate symbol-timing position: the mean square distance from
    the recent equalized symbols to the V.34 odd-integer lattice, recomputed
    with the current taps at an offset of `offset` samples, and derotated
    both by the carrier loop's standing phase -- which is what the slicer
    sees -- and by the phase the offset itself implies. */
static float v90_t3_score_offset(v34_rx_state_t *s, float offset)
{
    int pre = V34_V90_T3_FSE_TAPS/2;
    float rot = v90_t3_offset_rotation(s, offset);
    float rot_cs = cosf(rot);
    float rot_sn = sinf(rot);
    /* Score where the slicer would actually be looking: the emit path reads
       the ring through the timing loop's leftover fraction, so a scorer that
       reads whole samples is measuring a position the receiver never uses,
       and the true offset scores no better than its neighbours. */
    float frac = s->v90_t3_timing_enabled ? s->v90_t3_gardner.acc : 0.0f;
    float total = 0.0f;
    int counted = 0;

    for (int k = 1;  k <= V34_V90_T3_SLIP_WINDOW;  k++)
    {
        int64_t at = s->v90_t3_next_symbol - 3*k + (int) floorf(offset);
        float sub = frac + (offset - floorf(offset));
        complexf_t y = {0.0f, 0.0f};
        float t_re;
        float t_im;

        if (at - pre < 0
            ||
            at - pre < s->v90_t3_raw_count - V34_V90_T3_RAW_SIZE + 8)
        {
            break;
        }
        /*endif*/
        for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
        {
            complexf_t x = v90_t3_raw_get_frac(s, at - pre + tap, sub);
            complexf_t z;

            if (s->v90_t3_fse_conjugate)
                x.im = -x.im;
            /*endif*/
            z = complex_mulf(&s->v90_t3_fse[tap], &x);
            y = complex_addf(&y, &z);
        }
        /*endfor*/
        if (s->v90_t3_carrier_enabled)
        {
            float dr;
            float di;

            v34_carrier_derotate(&s->v90_t3_carrier, y.re, y.im, &dr, &di);
            y.re = dr;
            y.im = di;
        }
        /*endif*/
        {
            /* Undo the phase the offset itself brings with it. */
            float rr = y.re*rot_cs - y.im*rot_sn;
            float ri = y.re*rot_sn + y.im*rot_cs;

            y.re = rr;
            y.im = ri;
        }
        t_re = 2.0f*floorf(y.re/2.0f) + 1.0f;
        t_im = 2.0f*floorf(y.im/2.0f) + 1.0f;
        total += (t_re - y.re)*(t_re - y.re) + (t_im - y.im)*(t_im - y.im);
        counted++;
    }
    /*endfor*/
    if (counted < V34_V90_T3_SLIP_WINDOW/2)
        return 1e9f;
    /*endif*/
    return total/counted;
}
/*- End of function --------------------------------------------------------*/

/*! Recover from a whole-sample slip in the received stream.

    Measured offline against a recorded five-minute call (v90_upstream_replay
    on artifacts/dmodem-soak-0821-goalproper): the upstream decodes the peer
    perfectly -- distance to the lattice 0.002 -- for 42.1 s, and then loses
    it inside three symbols and never gets it back.  The wire is unchanged
    across that instant: same level, same 201-3439 Hz band, same carrier, no
    duplicated or missing RTP packet, and no coherent rotation afterwards.
    Deleting ONE 8 kHz sample at that instant doubles the clean stretch to
    81.7 s, which is what identifies it: the peer's stream gains a sample
    about every forty seconds, a clock offset of some three parts per
    million, absorbed as a whole-sample insertion.

    Gardner tracks fractional drift but cannot help here, because every
    adaptive element in this receiver -- the timing loop, the DD-LMS, the
    carrier loop -- is gated on the symbols already being good, and a slip
    closes the eye in one symbol.  The moment correction is needed, all of
    them freeze, which is exactly why the collapse was permanent and why
    restoring a known-good equalizer 337 times recovered nothing.

    So when the eye closes, search for it: re-run the existing taps over the
    recent past at each whole-sample offset and take the one that puts the
    symbols back on the lattice.  A slip is a step, so the recent past is
    already on the far side of it and scores it correctly. */
static bool v90_t3_slip_resync(v34_rx_state_t *s)
{
    float base;
    float best;
    float best_offset = 0.0f;
    char detail[256];
    int len = 0;

    base = v90_t3_score_offset(s, 0.0f);
    best = base;
    /* Thirds of a sample: at three samples per symbol a slip lands the
       sampling instant a whole sample out, but the fraction the timing loop
       was carrying when the eye closed is not necessarily the right one on
       the far side of the step.

       One symbol either side and no further -- see V34_V90_T3_SLIP_SPAN.
       The lattice score is periodic in three samples, so a wider search
       returns the same minimum again three samples away, and taking that
       copy shifts the whole symbol stream by one against the transmitter. */
    for (int step = -V34_V90_T3_SLIP_SPAN;
         step <= V34_V90_T3_SLIP_SPAN;
         step++)
    {
        float offset = step/6.0f;
        float q;

        if (step == 0)
            continue;
        /*endif*/
        /* With the fractional timing actuator disabled, only whole-sample
           candidates can actually be applied.  Scoring a sixth-sample point
           and then applying only its carrier rotation creates a correction
           that never existed in the sample stream. */
        if (!s->v90_t3_timing_enabled && step % 6 != 0)
            continue;
        /*endif*/
        q = v90_t3_score_offset(s, offset);
        if (len < (int) sizeof(detail) - 16)
        {
            len += snprintf(detail + len, sizeof(detail) - len,
                            "%s%+.2f:%.2f", len ? " " : "", offset, q);
        }
        /*endif*/
        if (q < best)
        {
            best = q;
            best_offset = offset;
        }
        /*endif*/
    }
    /*endfor*/
    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
             "Rx - V.90 upstream slip search: base %.3f [%s]\n", base, detail);
    /* ME_V90_SLIP_ROT_SWEEP: at the best offset, is there ANY static rotation
     * that opens the eye?  A slip is a passband delay and so carries a phase
     * -- 82 degrees per 8 kHz sample at 3200 baud low carrier -- which
     * v90_t3_offset_rotation() is supposed to remove.  If some other angle
     * scores far better than the one it computes, that term is wrong; if no
     * angle does, the collapse is not a re-alignable delay at all and the
     * search cannot be made to fix it.  Diagnostic only. */
    if (getenv("ME_V90_SLIP_ROT_SWEEP"))
    {
        static int sweep_n;

        if (best_offset != 0.0f  &&  (sweep_n++ % 64) == 0)
        {
            char rd[512];
            int rl = 0;
            float rbest = 1e9f;
            float rbest_a = 0.0f;
            float save = s->v90_t3_carrier.phase;

            for (int a = 0;  a < 16;  a++)
            {
                float ang = a*(2.0f*3.14159265f/16.0f);
                float q;

                s->v90_t3_carrier.phase = v34_carrier_wrap(save + ang);
                q = v90_t3_score_offset(s, best_offset);
                if (rl < (int) sizeof(rd) - 16)
                {
                    rl += snprintf(rd + rl, sizeof(rd) - rl, "%s%d:%.2f",
                                   rl ? " " : "", (int) (ang*180.0f/3.14159265f), q);
                }
                /*endif*/
                if (q < rbest)
                {
                    rbest = q;
                    rbest_a = ang*180.0f/3.14159265f;
                }
                /*endif*/
            }
            /*endfor*/
            s->v90_t3_carrier.phase = save;
            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                     "Rx - V.90 upstream slip rotation sweep at offset %+.2f: "
                     "as-computed %.3f, best %.3f at %+.0f deg [%s]\n",
                     best_offset, best, rbest, rbest_a, rd);
        }
        /*endif*/
    }
    /*endif*/
    /* Only move for an unambiguous win.  Adopting a marginally better
       position on noise would walk the receiver off a symbol at a time. */
    /* Adopt a clear win rather than only a perfect one.  A slip does not
       only move the sampling instant: the equalizer spends the symbols
       before the search is triggered adapting to nothing, so the best
       reachable position after a late slip scores around 0.42 -- better
       than the 0.65 of a closed eye by a wide margin, but nowhere near the
       0.002 of a good one.  Requiring 0.25 absolute rejected exactly those,
       and the receiver sat at 0.65 for the remaining five minutes. */
    if (best_offset == 0.0f
        ||
        best > V34_V90_T3_SLIP_ACCEPT_ERR
        ||
        best > 0.6f*base)
    {
        return false;
    }
    /*endif*/
    /* Take the phase with the timing.  Derotation is by exp(-j*phase), and
       the correction wanted is a multiply by exp(+j*rot). */
    s->v90_t3_carrier.phase = v34_carrier_wrap(
        s->v90_t3_carrier.phase - v90_t3_offset_rotation(s, best_offset));
    if (s->v90_t3_timing_enabled)
        s->v90_t3_next_symbol += v34_gardner_shift(&s->v90_t3_gardner,
                                                   best_offset);
    else
        s->v90_t3_next_symbol += (int) best_offset;
    /*endif*/
    s->v90_t3_sym_err_ema = best;
    s->v90_t3_sym_err_fast = best;
    s->v90_t3_slips_recovered++;
    /* Let the equalizer back in.  Every adaptive element here is gated on the
       symbols already being good, which is right in steady state and wrong
       immediately after a slip: the timing is correct again but the filter
       still carries whatever the closed eye taught it, and at an error just
       above the DD-LMS gate it can never pull itself back. */
    s->v90_t3_recover = V34_V90_T3_SLIP_RECOVER;
    /* The frame phase survives a whole-sample move -- the symbol stream is
       the same, sampled correctly again -- but the loops that froze on the
       way down have to be let go of. */
    s->v90_t3_gardner.freq = 0.0f;
    s->v90_t3_gardner.hold = 0;
    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
             "Rx - V.90 upstream slip of %+.2f sample(s) corrected "
             "(lattice distance %.3f -> %.3f, %d so far)\n",
             best_offset, base, best, s->v90_t3_slips_recovered);
    return true;
}
/*- End of function --------------------------------------------------------*/

/*! The error at which a slip is looked for.  Relative to the receiver's own
    settled operating point once it has one, because the fault a slip
    produces is a step away from THAT, not a crossing of any absolute.  Never
    above the absolute arm, so this can only make the search more alert. */
static float v90_t3_slip_trigger_err(v34_rx_state_t *s)
{
    float v;

    if (s->v90_t3_err_base_n < V34_V90_T3_ERR_BASE_SYMBOLS
        ||
        s->v90_t3_slip_mult <= 0.0f)
    {
        return V34_V90_T3_TIMING_TRACK_ERR;
    }
    /*endif*/
    v = s->v90_t3_slip_mult*s->v90_t3_err_base;
    if (v < V34_V90_T3_SLIP_MULT_MIN)
        v = V34_V90_T3_SLIP_MULT_MIN;
    /*endif*/
    if (v > V34_V90_T3_TIMING_TRACK_ERR)
        v = V34_V90_T3_TIMING_TRACK_ERR;
    /*endif*/
    return v;
}
/*- End of function --------------------------------------------------------*/

static void v90_t3_emit_ready(v34_rx_state_t *s)
{
    int pre = V34_V90_T3_FSE_TAPS/2;

    while (s->v90_t3_acquired
           && s->v90_t3_next_symbol - pre + V34_V90_T3_FSE_TAPS
                <= s->v90_t3_raw_count)
    {
        complexf_t y = {0.0f, 0.0f};
        /* The timing loop's leftover fraction of a sample. */
        float frac = s->v90_t3_timing_enabled ? s->v90_t3_gardner.acc : 0.0f;

#if defined(V34_FIXED_POINT)
        /* Integer FSE.  Taps live in the wide Q1.46 accumulator and are
           narrowed to Q1.30 here; port/v34_fixed.h has the measurements,
           including why a Q1.30 accumulator does not stall but instead tracks
           its own quantisation noise and settles at 0.666 from the lattice. */
        {
            v34_fx_complex_t xr[V34_V90_T3_FSE_TAPS];
            v34_fx_complex_t zf;
            int32_t frac_q16 = (int32_t) (frac*65536.0f);

            if (!s->v90_t3_fx_primed)
            {
                /* Block floating point: pick the ring's binary point from the
                   level actually present, instead of the Q17.14 that one
                   recording suggested.  Scan the live ring rather than trust a
                   reported power -- this is the number the format has to fit. */
                float peak = 0.0f;
                int scan;

                for (scan = 0;  scan < 4096;  scan++)
                {
                    complexf_t v = v90_t3_raw_get(s, s->v90_t3_raw_count - 1 - scan);
                    float m = fabsf(v.re) > fabsf(v.im) ? fabsf(v.re) : fabsf(v.im);

                    if (m > peak)
                        peak = m;
                    /*endif*/
                }
                /*endfor*/
                s->v90_t3_fx_rshift = v34_fx_choose_rshift(peak);
                span_log(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 fixed-point ring: peak %.1f -> Q%d\n",
                         peak, s->v90_t3_fx_rshift);

                /* The integer ring has already been filled, one sample at a
                   time, at the PREVIOUS binary point -- so changing the shift
                   and walking away leaves the FSE reading a ring holding two
                   different formats.  Re-convert what is already there from
                   the float ring it was derived from.

                   This was measured NEUTRAL when priming happened once, at
                   acquisition, and nothing had yet moved the taps.  It stops
                   being neutral now that a restore re-primes: restores fire
                   several times a second, so the mixed-format window is no
                   longer a one-off at the start of a call. */
                for (scan = 0;  scan < V34_V90_T3_RAW_SIZE;  scan++)
                {
                    int idx = (s->v90_t3_raw_count - 1 - scan) & V34_V90_T3_RAW_MASK;

                    s->v90_t3_raw_fx[idx].re =
                        v34_fx_from_float(s->v90_t3_raw[idx].re, s->v90_t3_fx_rshift);
                    s->v90_t3_raw_fx[idx].im =
                        v34_fx_from_float(s->v90_t3_raw[idx].im, s->v90_t3_fx_rshift);
                }
                /*endfor*/

                for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
                {
                    s->v90_t3_fse_fx[tap].re = v34_fx_from_float(s->v90_t3_fse[tap].re, V34_FX_TAP_SHIFT);
                    s->v90_t3_fse_fx[tap].im = v34_fx_from_float(s->v90_t3_fse[tap].im, V34_FX_TAP_SHIFT);
                }
                /*endfor*/
                v34_fx_lms_init(s->v90_t3_fse_acc, s->v90_t3_fse_fx, V34_V90_T3_FSE_TAPS);
                s->v90_t3_fx_primed = 1;
            }
            /*endif*/
            v34_fx_lms_taps(s->v90_t3_fse_fx, s->v90_t3_fse_acc, V34_V90_T3_FSE_TAPS);
            for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
            {
                xr[tap] = v34_fx_ring_get_frac(s->v90_t3_raw_fx,
                                               s->v90_t3_next_symbol - pre + tap,
                                               frac_q16,
                                               s->v90_t3_raw_count,
                                               V34_V90_T3_RAW_SIZE,
                                               V34_V90_T3_RAW_MASK);
                if (s->v90_t3_fse_conjugate)
                    xr[tap].im = -xr[tap].im;
                /*endif*/
            }
            /*endfor*/
            zf = v34_fx_fse(s->v90_t3_fse_fx, xr, V34_V90_T3_FSE_TAPS);
            y.re = v34_fx_to_float(zf.re, s->v90_t3_fx_rshift);
            y.im = v34_fx_to_float(zf.im, s->v90_t3_fx_rshift);
        }
#else
        for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
        {
            complexf_t x = v90_t3_raw_get_frac(
                s, s->v90_t3_next_symbol - pre + tap, frac);
            complexf_t z;
            if (s->v90_t3_fse_conjugate)
                x.im = -x.im;
            z = complex_mulf(&s->v90_t3_fse[tap], &x);
            y = complex_addf(&y, &z);
        }
#endif
        /* V90_T3_SYMBOL_PROBE=<data symbol index>: everything that went into
           ONE symbol, so the whole chain from the recorded tap to the
           equalizer output can be reproduced outside this receiver and checked
           stage by stage.  That is the only way to establish whether an
           offline model of this path -- tools/v34_channel_bound.py's among
           them -- is describing the receiver or something else. */
        {
            static int probe_sym = -2;

            if (probe_sym == -2)
            {
                const char *value = getenv("V90_T3_SYMBOL_PROBE");

                probe_sym = value ? atoi(value) : -1;
            }
            /*endif*/
            if (probe_sym >= 0  &&  s->v90_t3_data_symbols == probe_sym
                &&  !s->v90_t3_in_b1)
            {
                fprintf(stderr,
                        "[T3PROBE] symbol=%" PRId64 " next_symbol=%" PRId64
                        " pre=%d frac=%.9f conjugate=%d rate=%d fc=%.6f"
                        " input_count=%" PRId64 " next_output=%" PRId64
                        " raw_count=%" PRId64 " qam_sample_time=%" PRId64
                        " y=(%.9f,%.9f)\n",
                        s->v90_t3_data_symbols,
                        (int64_t) s->v90_t3_next_symbol, pre, (double) frac,
                        s->v90_t3_fse_conjugate ? 1 : 0,
                        s->v90_t3_internal_rate,
                        (double) carrier_frequency(s->baud_rate, s->high_carrier),
                        (int64_t) s->v90_t3_input_count,
                        (int64_t) s->v90_t3_next_output,
                        (int64_t) s->v90_t3_raw_count,
                        (int64_t) s->qam_sample_time,
                        (double) y.re, (double) y.im);
                for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
                {
                    complexf_t x = v90_t3_raw_get_frac(
                        s, s->v90_t3_next_symbol - pre + tap, frac);
                    complexf_t r0 = v90_t3_raw_get(
                        s, s->v90_t3_next_symbol - pre + tap);

                    fprintf(stderr,
                            "[T3PROBE] tap=%2d fse=(%.9f,%.9f)"
                            " raw=(%.9f,%.9f) rawfrac=(%.9f,%.9f) idx=%" PRId64 "\n",
                            tap,
                            (double) s->v90_t3_fse[tap].re,
                            (double) s->v90_t3_fse[tap].im,
                            (double) r0.re, (double) r0.im,
                            (double) x.re, (double) x.im,
                            (int64_t) (s->v90_t3_next_symbol - pre + tap));
                }
                /*endfor*/
            }
            /*endif*/
        }
        /* Carrier.  The equalizer output is derotated by this receiver's own
           loop before anything looks at it: decision-directed while the
           symbols are on the constellation, fourth-power while they are not,
           because the decisions a decision-directed loop needs are exactly
           what a spinning constellation does not give. */
        if (s->v90_t3_carrier_enabled)
        {
            float dr;
            float di;

            s->v90_t3_dump_phase = s->v90_t3_carrier.phase;
            v34_carrier_derotate(&s->v90_t3_carrier, y.re, y.im, &dr, &di);
            v34_carrier_update(&s->v90_t3_carrier, dr, di,
                               (s->v90_t3_sym_err_fast
                                    < V34_V90_T3_TIMING_TRACK_ERR) ? 1 : 0);
            y.re = dr;
            y.im = di;
        }
        /*endif*/
        /* 9.4.2.5/V.90 has the analogue modem "start a new superframe" after
           B1, and 10.1.3.1/V.34 has B1 carry the superframe inversions of the
           FINAL data frame -- which is why v34_begin_rx_data() parks the
           receiver at frame j-1.  That only lines up if B1 is exactly one
           data frame.  If the peer sends several, its superframe counter
           stays parked for all of them while ours advances, and every bit
           after that is wrong even though the symbols are perfect (measured:
           decision error 0.106, output still white).  So find B1's real end
           on the wire -- it stops looking like the template -- and hold the
           superframe state until then. */
        if (s->v90_t3_in_b1  &&  s->v90_t3_b1_symbols > 0)
        {
            int64_t idx = (s->v90_t3_next_symbol - s->v90_t3_b1_start)/3;
            int pos = (int) (idx % s->v90_t3_b1_symbols);
            float d_re = y.re - s->v90_t3_b1[pos].re;
            float d_im = y.im - s->v90_t3_b1[pos].im;

            s->v90_t3_b1_frame_err += d_re*d_re + d_im*d_im;
            if (pos == s->v90_t3_b1_symbols - 1)
            {
                float mean = s->v90_t3_b1_frame_err/s->v90_t3_b1_symbols;

                s->v90_t3_b1_frame_err = 0.0f;
                if (s->x2_mode)
                {
                    /* V.34 10.1.3.1 and 11.4.1.1.4: B1 is one data
                       frame at the final-superframe epoch; DATA starts a
                       new superframe. Original Courier mapper capture
                       confirms this boundary. Do not park the input epoch
                       back at j-1 after consuming that frame: doing so
                       invalidates Table 12's trellis parity in DATA. */
                    s->v90_t3_in_b1 = false;
                    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                               "Rx - x2 reset-state B1 complete; retaining DATA input epoch and trellis parity\n");
                }
                else if (mean < 1.0f)
                {
                    /* Still B1: keep the receiver parked on the final frame
                       of a superframe, exactly as the reset left it. */
                    if (s->parms.j > 0)
                    {
                        s->super_frame = s->parms.j - 1;
                        s->v0_pattern = (uint16_t)(2*(s->parms.j - 1));
                        s->input_4d = (s->parms.j - 1)*4*s->parms.p;
                    }
                    /*endif*/
                }
                else
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                             "Rx - V.90 upstream B1 ended after %d data frames "
                             "(frame error %.3f); publishing data\n",
                             (int) ((idx + 1)/s->v90_t3_b1_symbols), mean);
                    s->v90_t3_in_b1 = false;
                    /* ME_V90_PHASE_FORCE_OFFSET=n displaces the decoder n
                       data frames from the phase B1 pins.  It has to happen
                       HERE and not at acquisition: applied while B1 is still
                       running, the shift is absorbed by B1's own pinning and
                       only the bookkeeping counter moves -- which is exactly
                       what made the first version of this test useless, since
                       it reported "offset 5" while decoding perfectly.
                       Together with ME_V90_PHASE_NO_MARKS it reproduces the
                       live failure on a recording whose payload is known. */
                    {
                        const char *v = getenv("ME_V90_PHASE_FORCE_OFFSET");
                        int forced = (v != NULL)  ?  atoi(v)  :  0;

                        if (forced > 0)
                        {
                            s->v90_t3_phase_delta = forced;
                            s->v90_t3_phase_pending = true;
                        }
                        /*endif*/
                    }
                }
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/
        s->v90_t3_suppress_output = s->v90_t3_in_b1
            || (s->x2_mode && s->v90_t3_next_symbol < s->v90_t3_publish_symbol);
        if (!s->v90_t3_in_b1)
            s->v90_t3_data_symbols++;
        /*endif*/
        /* ME_V90_UPSTREAM_SYM_DUMP writes the equalized symbols this slicer
           actually sees, as text.  The distance-to-lattice figure in the
           logs says the symbols left the constellation but not what they
           left it FOR, and that difference -- a rotation, a scale, a
           different lattice -- is what names the cause. */
        if (!s->v90_t3_in_b1)
        {
            if (!s->v90_t3_sym_dump_tried)
            {
                const char *path = V34_DIAG_GETENV("ME_V90_UPSTREAM_SYM_DUMP");

                s->v90_t3_sym_dump_tried = true;
                if (path)
                    s->v90_t3_sym_dump = fopen(path, "w");
                /*endif*/
            }
            /*endif*/
            if (s->v90_t3_sym_dump)
            {
                /* Fourth column: the receiver's own INPUT-SAMPLE counter at
                   the moment this symbol was emitted, i.e. the position in the
                   recorded 8 kHz tap.  Without it the symbol index has to be
                   mapped to the tap by assuming a start instant and an exact
                   symbol rate, and that assumption was wrong -- this dump and
                   the DATA-bits window series put the same collapse at 91% and
                   67% of a call respectively.  Anything that fits a recorded
                   tap against these symbols (tools/v34_channel_bound.py) needs
                   the alignment to a few samples, so record it rather than
                   deriving it.  Fifth column is the T/3 symbol position, which
                   pins the sub-sample phase: the input sample is 5/6 of it. */
                fprintf(s->v90_t3_sym_dump,
                        "%" PRId64 " %.4f %.4f %" PRId64 " %" PRId64 " %.6f\n",
                        s->v90_t3_data_symbols, y.re, y.im,
                        (int64_t) s->qam_sample_time,
                        (int64_t) s->v90_t3_next_symbol,
                        (double) s->v90_t3_dump_phase);
            }
            /*endif*/
        }
        /*endif*/
        s->v90_t3_in_emit = true;
        process_primary_symbol(s, &y);
        s->v90_t3_in_emit = false;
        /* Decision-directed NLMS on the same taps.  The least-squares fit
           over B1's 128 symbols leaves about 1.4% residual energy -- roughly
           1.7 sigma of the decision half-distance -- which is several percent
           of raw symbol errors, and frozen taps only get worse as the channel
           and timing drift over a call lasting minutes.  V.34 puts every
           constellation point on odd integers, so the decision is available
           here without waiting for the shell decoder. */
        {
            float t_re = 2.0f*floorf(y.re/2.0f) + 1.0f;
            float t_im = 2.0f*floorf(y.im/2.0f) + 1.0f;
            float e_re = t_re - y.re;
            float e_im = t_im - y.im;
            float energy = 1e-6f;

            for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
            {
                complexf_t x = v90_t3_raw_get_frac(
                    s, s->v90_t3_next_symbol - pre + tap, frac);
                energy += x.re*x.re + x.im*x.im;
            }
            /* Adapt only on decisions worth adapting to.  Two gates:
               reject gross outliers symbol by symbol (the old 0.5 threshold
               was tighter than the error the equalizer actually leaves, so
               it never fired once, which is why an early "DD-LMS changes
               nothing" result meant nothing), and stop entirely when the
               constellation is not being hit at all.  Measured live: the
               symbols sat at 0.10 for thirteen seconds, jumped to 0.66 --
               the figure for symbols bearing no relation to the lattice --
               and stayed there for the remaining four minutes of the call.
               Adapting a filter towards decisions that are noise is how it
               gets walked off, and nothing then re-acquires it. */
            /* Gate on the FAST estimate.  This filter is the one thing the
               slip search needs intact to find the new sampling instant, and
               the slow average leaves it adapting onto garbage for the
               hundred symbols it takes to turn round. */
            /* And gate on the receiver's OWN operating point, not only on an
               absolute.  0.35 is three times the error a 28800 call settles
               at and eight times a 24000 one, so at the top of the rate
               ladder it lets the loop go on adapting through decisions that
               are already substantially wrong -- which is how a filter
               ratchets away from a working solution rather than falling off
               it.  The plain V.34 data mode reached the same conclusion from
               the other end (docs/v34_data_mode_rates.md: a smaller step is
               the wrong lever; adapt only while the error stays near the
               baseline that settled just after B1). */
            if (e_re*e_re + e_im*e_im < 8.0f
                &&
                v90_t3_dd_gate_ok(s)
                &&
                (s->v90_t3_sym_err_fast < V34_V90_T3_TIMING_TRACK_ERR
                 ||
                 (s->v90_t3_recover > 0
                  &&
                  s->v90_t3_sym_err_fast < V34_V90_T3_SLIP_ACCEPT_ERR)))
            {
#if defined(V34_FIXED_POINT)
                /* Integer NLMS into the wide accumulator.  energy is passed as
                   the plain real-units value the float path already computed;
                   mixing Q-formats through the divide is what silently
                   truncated every correction to zero in the first draft. */
                {
                    v34_fx_complex_t xr[V34_V90_T3_FSE_TAPS];
                    int32_t frac_q16 = (int32_t) (frac*65536.0f);

                    for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
                    {
                        xr[tap] = v34_fx_ring_get_frac(s->v90_t3_raw_fx,
                                                       s->v90_t3_next_symbol - pre + tap,
                                                       frac_q16,
                                                       s->v90_t3_raw_count,
                                                       V34_V90_T3_RAW_SIZE,
                                                       V34_V90_T3_RAW_MASK);
                        if (s->v90_t3_fse_conjugate)
                            xr[tap].im = -xr[tap].im;
                        /*endif*/
                    }
                    /*endfor*/
                    v34_fx_lms_update(s->v90_t3_fse_acc, xr, V34_V90_T3_FSE_TAPS,
                                      v34_fx_from_float(e_re, s->v90_t3_fx_rshift),
                                      v34_fx_from_float(e_im, s->v90_t3_fx_rshift),
                                      v34_fx_from_float(s->v90_t3_dd_mu, V34_FX_TAP_SHIFT),
                                      (int64_t) energy, s->v90_t3_fx_rshift);
                    v34_fx_lms_taps(s->v90_t3_fse_fx, s->v90_t3_fse_acc, V34_V90_T3_FSE_TAPS);
                    for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
                    {
                        s->v90_t3_fse[tap].re = v34_fx_to_float(s->v90_t3_fse_fx[tap].re, V34_FX_TAP_SHIFT);
                        s->v90_t3_fse[tap].im = v34_fx_to_float(s->v90_t3_fse_fx[tap].im, V34_FX_TAP_SHIFT);
                    }
                    /*endfor*/
                }
#else
                float mu = s->v90_t3_dd_mu/energy;

                for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
                {
                    complexf_t x = v90_t3_raw_get_frac(
                        s, s->v90_t3_next_symbol - pre + tap, frac);

                    if (s->v90_t3_fse_conjugate)
                        x.im = -x.im;
                    /* e * conj(x) */
                    s->v90_t3_fse[tap].re += mu*(e_re*x.re + e_im*x.im);
                    s->v90_t3_fse[tap].im += mu*(e_im*x.re - e_re*x.im);
                }
#endif
            }
            /*endif*/
            v90_t3_blind_recover(s, &y, pre, frac, energy);
        }
        /* Keep a copy of the filter from when it was demonstrably working,
           and put it back if the symbols collapse.  The supervised fit onto
           B1 happens once; after that only decision-directed adaptation
           carries the filter, and B1 is long gone by the time anything goes
           wrong, so without a snapshot there is nothing to return to -- a
           call that loses the constellation stays lost for its whole
           remaining length (measured: four minutes of it). */
        /* A slip closes the eye in a single symbol, so look for one as soon
           as the error says the constellation is gone -- long before the
           equalizer-restore path below, which is for a filter that has been
           walked off gradually and cannot fix a timing step at all. */
        if (s->v90_t3_sym_err_fast >= v90_t3_slip_trigger_err(s))
        {
            if (++s->v90_t3_slip_run >= V34_V90_T3_SLIP_RUN)
            {
                s->v90_t3_slip_run = 0;
                if (v90_t3_slip_resync(s))
                {
                    s->v90_t3_resync_misses = 0;
                }
                else if (s->v90_t3_err_base_n >= V34_V90_T3_ERR_BASE_SYMBOLS
                         && s->v90_t3_sym_err_fast
                                >= fmaxf(V34_V90_T3_RESYNC_ERR_MIN,
                                         V34_V90_T3_RESYNC_ERR_MULT
                                            *s->v90_t3_err_base))
                {
                    if (s->v90_t3_resync_misses
                            < V34_V90_T3_RESYNC_MISSES)
                    {
                        s->v90_t3_resync_misses++;
                    }
                    /* V.90 9.6 says the digital and analogue receivers shall
                       maintain data-frame synchronization during the rate
                       renegotiation.  V.34 11.6 says receiver
                       resynchronization uses training followed by MP, E and
                       B1, after which a new superframe begins.  Once the
                       bounded timing search has failed repeatedly, tell the
                       engine while the peer can still hear that exchange;
                       silently shifting mapper/frame state is not conformant
                       recovery and did not recover the controlled slip. */
                    if (s->v90_t3_resync_misses
                            >= V34_V90_T3_RESYNC_MISSES
                        && !s->v90_t3_resync_required)
                    {
                        s->v90_t3_resync_required = true;
                        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                                 "Rx - V.90 upstream abrupt discontinuity: "
                                 "%d local timing searches failed; requesting "
                                 "9.6/11.6 training-and-B1 resynchronization\n",
                                 s->v90_t3_resync_misses);
                    }
                    /*endif*/
                }
                /*endif*/
            }
            /*endif*/
        }
        else
        {
            s->v90_t3_slip_run = 0;
            s->v90_t3_resync_misses = 0;
        }
        /*endif*/
        if (s->v90_t3_recover > 0)
            s->v90_t3_recover--;
        /*endif*/
        /* Learn the receiver's own operating point before judging it against
           anything.  B1 hands over a converged filter, so the error settles
           within a second; average it over the next few thousand symbols and
           use that, rather than a constant, as the scale for "demonstrably
           good" and "lost".  See V34_V90_T3_FSE_KEEP_MULT for what the fixed
           threshold cost. */
        /* Has the upstream lost carrier?  V.90 9.6's rate renegotiation is
           the recovery, and the engine drives it; this only has to say when.
           Judge on the slow estimate, so a burst does not trigger a
           renegotiation that costs the best part of a second. */
        if (s->v90_t3_err_base_n >= V34_V90_T3_ERR_BASE_SYMBOLS
            &&
            s->v90_t3_sym_err_ema >= V34_V90_T3_LOST_ERR)
        {
            if (++s->v90_t3_lost_run == V34_V90_T3_LOST_SYMBOLS)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 upstream carrier lost: %.3f from the "
                         "constellation for %d symbols (settled at %.3f)\n",
                         s->v90_t3_sym_err_ema, s->v90_t3_lost_run,
                         s->v90_t3_err_base);
            }
            /*endif*/
        }
        else
        {
            s->v90_t3_lost_run = 0;
        }
        /*endif*/
        if (s->v90_t3_err_base_n < V34_V90_T3_ERR_BASE_SYMBOLS)
        {
            /* Average only over symbols that are actually settled.  The
               window is meant to answer "what does this receiver look like
               when it is working", and a call that loses the constellation
               inside it does not contribute to that answer: measured live on
               2026-08-24, a 28800 call that collapsed 0.9 s after B1 recorded
               a settled figure of 0.281 where its own first windows read
               0.106, and every threshold derived from it -- the snapshot, the
               restore, the phase-evidence gate -- was then scaled to a
               receiver that never existed.

               Skipping the lost symbols rather than taking the window's
               minimum: the minimum is immune to the same fault, but it
               estimates the floor rather than the operating point, and
               measured on the replays it costs 19200 twenty points of clean
               call (69% -> 48%).  If a call never accumulates a full window
               of settled symbols the baseline simply never establishes, and
               the fixed fallbacks stand -- which is the behaviour that
               predates any of this. */
            if (s->v90_t3_sym_err_ema < V34_V90_T3_LOST_ERR)
            {
                s->v90_t3_err_base += s->v90_t3_sym_err_ema;
                s->v90_t3_err_base_n++;
            }
            /*endif*/
            if (s->v90_t3_err_base_n == V34_V90_T3_ERR_BASE_SYMBOLS)
            {
                s->v90_t3_err_base /= V34_V90_T3_ERR_BASE_SYMBOLS;
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 upstream settled at %.3f from the "
                         "constellation; keeping the equalizer below %.3f, "
                         "restoring it above %.3f\n",
                         s->v90_t3_err_base,
                         v90_t3_fse_keep_err(s), v90_t3_fse_lost_err(s));
            }
            /*endif*/
        }
        /*endif*/
        if (s->v90_t3_sym_err_ema < v90_t3_fse_keep_err(s))
        {
            if (++s->v90_t3_fse_good_age >= 3200)
            {
                memcpy(s->v90_t3_fse_good, s->v90_t3_fse,
                       sizeof(s->v90_t3_fse_good));
                s->v90_t3_fse_good_valid = true;
                s->v90_t3_fse_good_age = 0;
            }
            /*endif*/
            s->v90_t3_fse_bad_run = 0;
        }
        else if (s->v90_t3_sym_err_ema < v90_t3_fse_lost_err(s))
        {
            /* Neither demonstrably good nor lost.  The old code had no such
               zone -- anything not good enough to snapshot counted towards a
               restore -- so a receiver sitting just above its snapshot
               threshold was being treated as one that had lost the
               constellation. */
            s->v90_t3_fse_bad_run = 0;
        }
        else if (s->v90_t3_fse_good_valid  &&  !s->v90_t3_cma_active)
        {
            /* Not while the blind loop has the taps.  The snapshot is from
               before the collapse and is exactly the filter that stopped
               working; putting it back every V34_V90_T3_FSE_BAD_RUN symbols
               undoes the recovery in progress, which is measurable: with the
               restore left free to run, CMA reopened the eye ten times on
               rate28800-r1 and lost it again within a second each time. */
            if (++s->v90_t3_fse_bad_run >= V34_V90_T3_FSE_BAD_RUN)
            {
                memcpy(s->v90_t3_fse, s->v90_t3_fse_good,
                       sizeof(s->v90_t3_fse));
                v90_t3_fse_taps_replaced(s);
                s->v90_t3_fse_bad_run = 0;
                s->v90_t3_fse_good_age = 0;
                /* The lock goes with it -- the bits cannot be trusted
                   while the eye is shut -- but NOT the sweep's progress.
                   Restoring the equalizer does not move the symbol stream,
                   so a candidate measured before the restore is still the
                   same candidate afterwards; and a restore fires as often as
                   every V34_V90_T3_FSE_BAD_RUN symbols, so resetting the
                   counter here starved the sweep completely.  Measured on
                   artifacts/dmodem-soak-0821-rounds/round1: 300-odd restores
                   in one call and the sweep never once passed step 1 of 112,
                   sitting on the same phase for five minutes. */
                s->v90_t3_sf_locked = false;
                /* Drop the frequency estimate the closed eye taught the
                   loop, but NOT its accumulator: acc is the fractional part
                   of the sampling position, and zeroing it moves that
                   position by up to a whole sample with nothing to
                   compensate -- manufacturing, on the recovery path, exactly
                   the fault the search below is there to undo.  Keeping the
                   frequency instead was measured slightly worse (30 clean
                   windows against 50, artifacts/goal-v90-073744Z). */
                s->v90_t3_gardner.freq = 0.0f;
                s->v90_t3_gardner.hold = 0;
                s->v90_t3_fse_restores++;
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 upstream symbols lost; restoring the "
                         "last good equalizer (restore %d)\n",
                         s->v90_t3_fse_restores);
                /* Search from the restored filter rather than declaring the
                   receiver well.  A restore on its own put a known-good
                   filter back at a sampling instant that is a whole sample
                   out, which is neither of the two states this receiver can
                   decode in; zeroing the error estimate then hid that for
                   the next few hundred symbols.  Together the two halves --
                   the filter from when it worked, the phase the wire is
                   actually on now -- are the state the call started in. */
                s->v90_t3_sym_err_ema = v90_t3_score_offset(s, 0.0f);
                s->v90_t3_sym_err_fast = s->v90_t3_sym_err_ema;
                v90_t3_slip_resync(s);
                s->v90_t3_recover = V34_V90_T3_SLIP_RECOVER;
            }
            /*endif*/
        }
        /*endif*/
        /* Timing.  This branch used to advance by exactly three samples for
           the life of a call, with nothing to correct it -- so a few ppm
           between the peer's symbol clock and our 8 kHz bearer, or a single
           sample inserted or dropped anywhere in the RTP path (a third of a
           symbol, instantly), accumulated without limit.  Measured against
           slmodemd: the idle stream decodes at 100% ones and then walks off
           into noise about fifteen seconds later, while the peer's DTE is
           still idle.

           Gardner's detector on the MATCHED-FILTERED stream closes it.  It
           wants the symbol instant and the point halfway back to the
           previous one; at three samples per symbol that midpoint is exactly
           the average of the two samples either side of it.  The loop hands
           back whole samples and keeps the fraction, which the reader above
           interpolates by -- see v34_gardner.h for why the fraction matters
           and v34_gardner_test for the S-curve, acquisition, tracking and
           quiescence it is held to. */
        if (s->v90_t3_timing_enabled)
        {
            /* Gardner wants the symbol instant and the point halfway back to
               the previous one, taken on the signal the DECISIONS are made
               on -- which here is the equalizer output, not the matched
               stream feeding it.  The supervised filter is fitted by least
               squares and its delay is whatever best matched B1 anywhere
               within its 21-tap span, so the matched stream's own best
               instant is not the one the slicer sees: measured on the clean
               loopback, a detector reading the matched stream reported an
               error of -0.40 where the true timing error is zero, and drove
               the loop into inserting symbols on a channel with no drift at
               all.  One extra filter evaluation, at the half-symbol point,
               keeps the detector and the decisions looking at the same
               signal. */
            complexf_t mid = {0.0f, 0.0f};

            for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
            {
                complexf_t x = v90_t3_raw_get_frac(
                    s, s->v90_t3_next_symbol - pre + tap - 2,
                    frac + 0.5f);
                complexf_t z;

                if (s->v90_t3_fse_conjugate)
                    x.im = -x.im;
                /*endif*/
                z = complex_mulf(&s->v90_t3_fse[tap], &x);
                mid = complex_addf(&mid, &z);
            }
            /* The sampling position is next_symbol + acc, and the loop
               takes a whole sample out of acc whenever it hands one back
               here, so a correction moves the position by the loop's small
               step and nothing else -- it is continuous, and the equalizer
               sees no jump.  (Shifting the taps to "compensate" for the index
               change, which an earlier version did, cancels the correction
               entirely: the loop then never sees its own effect and winds the
               integrator to the clamp.  Measured at 20 ppm, freq ran from
               -2e-5 straight past the -6e-5 it needed to -2e-3.) */
            /* Track only while the frame phase is locked.  Gardner's error
               is only meaningful when the decisions behind it are: once a
               decode goes wrong the detector reports whatever the garbage
               looks like, the integrator winds to its clamp (measured: freq
               pinned at -0.002 with 225 corrections requested in forty
               seconds), and a few slips in one direction shift the symbol
               clock a whole symbol against the transmitter -- which the
               frame-phase sweep cannot undo, because it searches mapping
               frames, not single symbols.  Holding the loop is also the only
               correct way to ignore it: dropping the returned correction
               while the loop had already wrapped its own accumulator moved
               the sampling position by a whole sample instead of leaving it
               where it was.

               The gate is symbol quality, not frame phase.  Gardner is
               non-data-aided -- it does not care whether the frames decode,
               only whether the symbols are on the lattice -- so gating it on
               the frame-phase lock froze the loop exactly when it was still
               useful: six consecutive calls then never locked at all, and
               sat at a symbol error of 0.2 where the calls that do lock read
               0.10. */
            /* The decision the two decision-aided detectors need is the
               same odd-integer slice the DD-LMS above makes, and V.34 puts
               every constellation point on that lattice, so it costs
               nothing to hand it over. */
            s->v90_t3_next_symbol +=
                v34_gardner_update(&s->v90_t3_gardner, y.re, y.im,
                                   mid.re, mid.im,
                                   2.0f*floorf(y.re/2.0f) + 1.0f,
                                   2.0f*floorf(y.im/2.0f) + 1.0f,
                                   s->v90_t3_sym_err_fast,
                                   (s->v90_t3_sym_err_fast
                                        < V34_V90_T3_TIMING_TRACK_ERR)
                                       ? 1 : 0);
            if (V34_DIAG_GETENV("ME_V90_UPSTREAM_TIMING_DEBUG"))
            {
                static int dbg;

                if ((dbg++ % 1600) == 0)
                {
                    fprintf(stderr,
                            "[T3TIMING] err=%+.4f freq=%+.6f acc=%+.3f "
                            "slips=%d\n",
                            s->v90_t3_gardner.last_error,
                            s->v90_t3_gardner.freq,
                            s->v90_t3_gardner.acc,
                            s->v90_t3_gardner.slips);
                }
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/
        s->v90_t3_next_symbol += 3;
    }
}

static bool v34_build_expected_b1_tap_trellis(v34_rx_state_t *rx,
                                              int scrambler_tap,
                                              int trellis_override,
                                              int initial_state);
static int v34_expected_b1_default_tap(v34_rx_state_t *rx);

/* Put the data-frame decoder back into its 10.1.3.1 reset state, at a chosen
   superframe index.  v34_begin_rx_data() does this once at the E handover
   with index j-1 (B1 stands in for the final frame of a superframe); the
   upstream phase search re-does it at a data-frame boundary to try another
   index.  Poking super_frame alone is not enough -- input_4d, the 4D symbol
   counter and the Viterbi metrics all have to agree with it, and a partial
   change explores nothing but corruption. */
static void v34_reset_rx_data_frame_state(v34_rx_state_t *s, int super_frame)
{
    int prior;

    s->step_2d = 0;
    s->data_frame = 0;
    s->mapping_frame_count = 0;
    s->s_bit_cnt = 0;
    s->aux_bit_cnt = 0;
    memset(s->xt, 0, sizeof(s->xt));
    memset(s->x, 0, sizeof(s->x));
    memset(s->ww, 0, sizeof(s->ww));
    s->viterbi.ptr = 0;
    s->viterbi.windup = 15;
    s->super_frame = super_frame;
    s->v0_pattern = (uint16_t) (2*super_frame);
    s->input_4d = super_frame*4*s->parms.p;
    prior = (s->viterbi.ptr - 1) & 0xF;
    for (int state = 0;  state < s->viterbi.state_count;  state++)
    {
        s->viterbi.vit[prior].cumulative_path_metric[state] =
            (state == 0)  ?  0U  :  0x3FFFFFFFU;
    }
    /*endfor*/
}

/* One acquisition pass over the currently loaded B1 template. */
static float v90_t3_acquire_pass(v34_rx_state_t *s,
                                 int64_t search_start,
                                 int64_t search_end,
                                 complexf_t best_coeff[V34_V90_T3_FSE_TAPS],
                                 int64_t *best_first_out,
                                 bool *best_conjugate_out,
                                 float *coarse_out)
{
    enum { KEEP = 8 };
    float score[KEEP] = {0};
    int64_t first[KEEP] = {0};
    bool conjugate[KEEP] = {0};
    float best_match = 0.0f;
    int pre = V34_V90_T3_FSE_TAPS/2;

    /* This runs synchronously in the media thread, so its cost is real time
       the audio path does not get.  A full-resolution scan of 30000 offsets
       x 2 conjugations x 128 symbols x 6 templates is tens of millions of
       complex MACs -- hundreds of milliseconds of stall, which the far end
       sees as a discontinuity and answers with a retrain.  Step a symbol at
       a time first and refine only around what that finds. */
    for (int64_t f = search_start;  f <= search_end;  f += 3)
    {
        for (int c = 0;  c < 2;  c++)
        {
            float q = v90_t3_coarse_score(s, f, c != 0);
            for (int k = 0;  k < KEEP;  k++)
            {
                if (q > score[k])
                {
                    for (int j = KEEP - 1;  j > k;  j--)
                    {
                        score[j] = score[j - 1];
                        first[j] = first[j - 1];
                        conjugate[j] = conjugate[j - 1];
                    }
                    score[k] = q;
                    first[k] = f - V34_V90_T3_RRC_TAPS/2;
                    conjugate[k] = c != 0;
                    break;
                }
            }
        }
    }
    /* Refine: the coarse pass only looked at every third sample. */
    for (int k = 0;  k < KEEP;  k++)
    {
        for (int d = -2;  d <= 2;  d++)
        {
            float q;

            if (d == 0)
                continue;
            /*endif*/
            if (first[k] + d < search_start - V34_V90_T3_RRC_TAPS/2)
                continue;
            /*endif*/
            q = v90_t3_coarse_score(s, first[k] + d + V34_V90_T3_RRC_TAPS/2,
                                    conjugate[k]);
            if (q > score[k])
            {
                score[k] = q;
                first[k] += d;
            }
            /*endif*/
        }
    }
    for (int k = 0;  k < KEEP;  k++)
    {
        complexf_t candidate[V34_V90_T3_FSE_TAPS];
        float match = v90_t3_fit(s, first[k], conjugate[k], candidate);
        if (match > best_match)
        {
            best_match = match;
            *best_first_out = first[k];
            *best_conjugate_out = conjugate[k];
            memcpy(best_coeff, candidate,
                   V34_V90_T3_FSE_TAPS*sizeof(complexf_t));
        }
    }
    *coarse_out = score[0];
    return best_match;
}
/*- End of function --------------------------------------------------------*/

/* A rejected acquisition window is not a failed call: E is detected on the CP
   bit stream and lags, so a first attempt usually means B1 is still ahead of
   the anchor.  Slide forward and search again when more wire has arrived.
   The backoff is not optional -- without it the search re-runs on identical
   samples every symbol, which is tens of millions of complex MACs per symbol
   in the media thread, and the far end hears the stall as a discontinuity.

   When the windows run out there is no upstream at all for the rest of the
   call, so say so rather than falling silent: V.90 9.5.1.1 is the only
   recovery with the reach to fix it, and a retrain ends in a fresh B1. */
static void v90_t3_acq_retry_or_abandon(v34_rx_state_t *s, const char *why)
{
    if (++s->v90_t3_acq_retries <= V34_V90_T3_ACQ_MAX_RETRIES)
    {
        s->v90_t3_acquisition_attempted = false;
        s->v90_t3_e_anchor += V34_V90_T3_ACQ_RETRY_GAP;
        s->v90_t3_acq_retry_at = s->v90_t3_raw_count + V34_V90_T3_ACQ_RETRY_GAP;
    }
    else
    {
        s->v90_t3_acq_abandoned = true;
        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                 "Rx - V.90 T/3 B1 giving up after %d windows (%s); the "
                 "upstream has no carrier and only a retrain can give it "
                 "one\n",
                 s->v90_t3_acq_retries, why);
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static void v90_t3_try_acquire(v34_rx_state_t *s)
{
    /* The E handover is only a coarse anchor: it is detected on the CP bit
       stream, whose alignment to this 9.6 kHz branch is worth far more than
       the 25 ms the old fixed 240-sample window allowed.  Measured against
       slmodemd every template scored ~5%, i.e. B1 was simply not inside the
       window.  Search as much history as the raw ring holds instead. */
    /* Reach back as far as the ring holds: the E detector's lag varies a
       lot between calls (measured from 0.3 s to well over 2.5 s), and a
       search that stops short of it simply never contains B1. */
    /* 3.4 s of history was not enough either: the E detector's lag varies a
       lot per call, and one measured at 79471 samples with B1 long rotated
       out of the ring by the time the search ran. */
    enum { SEARCH_BACK = 120000, SEARCH_FORWARD = 4800 };
    int64_t search_start;
    int64_t search_end;
    complexf_t best_coeff[V34_V90_T3_FSE_TAPS];
    float best_match = 0.0f;
    float best_coarse = 0.0f;
    int64_t best_first = 0;
    bool best_conjugate = false;
    int best_tap = 0;
    int best_trellis = -1;
    bool best_expanded = s->parms.expanded_shaping;
    /* 6.5/V.90 puts GPA on the analogue modem's upstream, but slmodemd
       measurably uses GPC, and a template built with the wrong polynomial
       does not correlate at all (about 5%, versus the 95% this needs).  The
       two are cheap to tell apart here -- build both and keep whichever
       actually fits the wire, rather than betting the whole upstream on a
       role assumption. */
    int candidate_tap[2];
    int candidate_count = 0;

    /* Search a span that straddles the anchor.  The E detector runs on the
       CP bit stream and lags the wire by a variable amount, so B1 can lie
       either side of it; only a window that reaches back before E can see
       the common case. */
    search_start = s->v90_t3_e_anchor - SEARCH_BACK;
    if (search_start < V34_V90_T3_FSE_TAPS/2 + V34_V90_T3_RRC_TAPS/2)
        search_start = V34_V90_T3_FSE_TAPS/2 + V34_V90_T3_RRC_TAPS/2;
    /*endif*/
    search_end = s->v90_t3_e_anchor + SEARCH_FORWARD;
    if (s->v90_t3_acquired || s->v90_t3_acquisition_attempted
        || s->v90_t3_raw_count < s->v90_t3_acq_retry_at
        || s->v90_t3_b1_symbols <= 0
        || s->v90_t3_e_anchor < 0
        || search_end <= search_start
        || s->v90_t3_raw_count < search_end
             + 3*s->v90_t3_b1_symbols + V34_V90_T3_FSE_TAPS)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90 T/3 B1 search held: acquired=%d attempted=%d "
                 "b1=%d anchor=%" PRId64 " start=%" PRId64 " end=%" PRId64
                 " raw=%" PRId64 "\n",
                 s->v90_t3_acquired ? 1 : 0,
                 s->v90_t3_acquisition_attempted ? 1 : 0,
                 s->v90_t3_b1_symbols, s->v90_t3_e_anchor,
                 search_start, search_end, s->v90_t3_raw_count);
        return;
    }
    /*endif*/
    /* Never search past what the ring still holds. */
    if (search_start
          < s->v90_t3_raw_count - V34_V90_T3_RAW_SIZE
              + 3*s->v90_t3_b1_symbols + V34_V90_T3_FSE_TAPS + 8)
    {
        search_start = s->v90_t3_raw_count - V34_V90_T3_RAW_SIZE
                     + 3*s->v90_t3_b1_symbols + V34_V90_T3_FSE_TAPS + 8;
    }
    /*endif*/
    s->v90_t3_acquisition_attempted = true;

    candidate_tap[candidate_count++] = v34_expected_b1_default_tap(s);
    if (s->v90_mode)
    {
        candidate_tap[candidate_count] =
            (candidate_tap[0] == 4) ? 17 : 4;
        candidate_count++;
    }
    /*endif*/

    /* The upstream trellis is the other thing this template has to guess:
       nothing the peer sends states it, and a wrong convolutional code makes
       B1 as uncorrelated as a wrong scrambler does. */
    for (int t = 0;  t < candidate_count  &&  best_match < 0.95f;  t++)
    {
        int trellis_count = s->v90_mode ? 3 : 1;

        /* Courier B1A1 -> A71B retains the MP expanded-shaping bit;
           unlike V.90 section 8.5.1, x2 does not force minimum shaping.
           Search both choices until the x2 MP field is decoded. */
        for (int sh = 0; sh < (s->x2_mode ? 2 : 1) && best_match < 0.95f; sh++)
        {
            if (s->x2_mode)
                v34_set_working_parameters(&s->parms, s->baud_rate, s->bit_rate, sh != 0);
            for (int tr = 0;  tr < trellis_count  &&  best_match < 0.95f;  tr++)
            {
                complexf_t coeff[V34_V90_T3_FSE_TAPS];
                int64_t first = 0;
                bool conjugate = false;
                float coarse = 0.0f;
                float match;
                int trellis = s->v90_mode ? tr : -1;

                if (!v34_build_expected_b1_tap_trellis(s, candidate_tap[t],
                                                       trellis, 0))
                    continue;
                /*endif*/
                match = v90_t3_acquire_pass(s, search_start, search_end, coeff,
                                            &first, &conjugate, &coarse);
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 T/3 B1 template tap=%d trellis=%d expanded=%d: "
                         "coarse=%.1f%% fit=%.1f%% conjugate=%d\n",
                         candidate_tap[t], trellis, s->parms.expanded_shaping ? 1 : 0,
                         100.0f*coarse, 100.0f*match, conjugate ? 1 : 0);
                if (match > best_match)
                {
                    best_match = match;
                    best_coarse = coarse;
                    best_first = first;
                    best_conjugate = conjugate;
                    best_tap = candidate_tap[t];
                    best_trellis = trellis;
                    best_expanded = s->parms.expanded_shaping;
                    memcpy(best_coeff, coeff, sizeof(best_coeff));
                }
                /*endif*/
            }
        }
    }
    /* Leave the winning template loaded; the data decoder reuses its state. */
    if (best_tap)
    {
        if (s->x2_mode)
            v34_set_working_parameters(&s->parms, s->baud_rate, s->bit_rate, best_expanded);
        (void)v34_build_expected_b1_tap_trellis(s, best_tap, best_trellis, 0);
        s->v90_far_tap_measured = best_tap;
        if (best_trellis >= 0)
            s->v90_t3_trellis_size = best_trellis;
        /*endif*/
    }
    /*endif*/

    /* Dense high-rate B1 can produce a superficially plausible least-squares
       solution which does not generalise into DATA.  Require a near-exact
       complete-frame fit; the clean 21.6 kbit/s regression is 99.9%, while
       the known-bad 28.8 kbit/s solution is only about 93%. */
    if (best_match < 0.95f)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                 "Rx - V.90 T/3 B1 acquisition failed "
                 "(first=%lld coarse=%.1f%% fit=%.1f%% tap=%d trellis=%d)\n",
                 (long long)best_first, 100.0f*best_coarse,
                 100.0f*best_match, best_tap, best_trellis);
        /* This used to return without arming a retry, leaving
           v90_t3_acquisition_attempted set -- so the ONE window that happened
           to be in front of the anchor decided the whole call, and a call
           that missed it carried no upstream for its entire life with nothing
           logged after this line.  Measured on three of the recorded rate
           matrix calls: one "acquisition failed" and then 115 s of silence.
           It is the same situation the out-of-sample rejection below handles
           by sliding the window forward, so handle it the same way. */
        v90_t3_acq_retry_or_abandon(s, "in-sample fit");
        return;
    }
    /* Validate the winner OUT OF SAMPLE before committing to it.  The fit is
       21 complex taps -- 42 free parameters -- least-squares onto 128
       symbols, which is only six real equations per parameter, so a quiet or
       otherwise unremarkable stretch of line can be fitted to 98.8% and mean
       nothing.  Measured live: a call acquired at 0.32 s with fit=98.8%, B1
       "ended" after one frame with a frame error of 9.1, and the resulting
       filter put the data symbols at a mean power of 721 against the
       template's 6.6 -- ten times the amplitude, on no lattice at all, for
       the whole call.  An in-sample score cannot see that; the symbols
       immediately after B1 can.  They are ordinary data, so they must land
       near the lattice and carry about the template's power if this filter
       is the right one. */
    {
        int64_t after = best_first + 3*s->v90_t3_b1_symbols;
        int pre = V34_V90_T3_FSE_TAPS/2;
        double dist = 0.0;
        double power = 0.0;
        int counted = 0;

        for (int k = 0;  k < V34_V90_T3_VALIDATE_SYMBOLS;  k++)
        {
            int64_t at = after + 3*k;
            complexf_t y = {0.0f, 0.0f};
            float t_re;
            float t_im;

            if (at - pre + V34_V90_T3_FSE_TAPS > s->v90_t3_raw_count)
                break;
            /*endif*/
            for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
            {
                complexf_t x = v90_t3_raw_get(s, at - pre + tap);
                complexf_t z;

                if (best_conjugate)
                    x.im = -x.im;
                /*endif*/
                z = complex_mulf(&best_coeff[tap], &x);
                y = complex_addf(&y, &z);
            }
            /*endfor*/
            t_re = 2.0f*floorf(y.re/2.0f) + 1.0f;
            t_im = 2.0f*floorf(y.im/2.0f) + 1.0f;
            dist += (t_re - y.re)*(t_re - y.re) + (t_im - y.im)*(t_im - y.im);
            power += y.re*y.re + y.im*y.im;
            counted++;
        }
        /*endfor*/
        if (counted >= V34_V90_T3_VALIDATE_SYMBOLS/2)
        {
            double mean_dist = dist/counted;
            double mean_power = power/counted;
            double template_power = 0.0;

            /* Judge the power against the TEMPLATE's, not an absolute.  B1 is
               drawn from the same constellation as the data that follows, so
               the two match when the filter is right -- whereas an absolute
               figure is only ever calibrated for one bit rate, and the offline
               T/3 regression runs at 21600, where the constellation is far
               bigger than the 9600 this was first measured at.  Set to a
               constant, it rejected a perfect acquisition. */
            for (int n = 0;  n < s->v90_t3_b1_symbols;  n++)
            {
                template_power += s->v90_t3_b1[n].re*s->v90_t3_b1[n].re
                                + s->v90_t3_b1[n].im*s->v90_t3_b1[n].im;
            }
            /*endfor*/
            if (s->v90_t3_b1_symbols > 0)
                template_power /= s->v90_t3_b1_symbols;
            else
                template_power = mean_power;
            /*endif*/

            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                     "Rx - V.90 T/3 B1 out-of-sample check: lattice distance "
                     "%.3f, symbol power %.1f against the template's %.1f, "
                     "over %d symbols\n",
                     mean_dist, mean_power, template_power, counted);
            /* Hold out for a GOOD acquisition -- distance as a fraction of
               the symbol power carrying it, see V34_V90_T3_ACQ_GOOD_SNR --
               and settle for a merely passable one only when the retries are
               spent.  Live, the first window after E often does not contain
               B1 at all yet, and fits 98% to whatever is there; half a second
               later it does. */
            double rel = (mean_power > 1e-6)
                       ? mean_dist/mean_power
                       : 1.0e9;
            bool power_ok = (mean_power
                                 <= V34_V90_T3_VALIDATE_POWER_RATIO
                                        *template_power);
            bool good = power_ok  &&  rel <= V34_V90_T3_ACQ_GOOD_SNR;
            bool passable = power_ok  &&  mean_dist <= V34_V90_T3_VALIDATE_ERR;
            bool retries_left = (s->v90_t3_acq_retries
                                     < V34_V90_T3_ACQ_MAX_RETRIES);

            /* Remember the best window seen, so that running out of retries
               settles for the best rather than for whichever came last. */
            if (passable
                &&
                (!s->v90_t3_acq_best_valid
                 ||
                 rel < s->v90_t3_acq_best_rel))
            {
                memcpy(s->v90_t3_acq_best_coeff, best_coeff,
                       sizeof(s->v90_t3_acq_best_coeff));
                s->v90_t3_acq_best_rel = (float) rel;
                s->v90_t3_acq_best_dist = (float) mean_dist;
                s->v90_t3_acq_best_match = best_match;
                s->v90_t3_acq_best_conjugate = best_conjugate;
                s->v90_t3_acq_best_first = best_first;
                s->v90_t3_acq_best_valid = true;
            }
            /*endif*/
            if (!good  &&  !retries_left  &&  s->v90_t3_acq_best_valid)
            {
                /* Out of retries: take the best window this call offered. */
                memcpy(best_coeff, s->v90_t3_acq_best_coeff,
                       sizeof(best_coeff));
                best_match = s->v90_t3_acq_best_match;
                best_conjugate = s->v90_t3_acq_best_conjugate;
                best_first = s->v90_t3_acq_best_first;
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 T/3 B1 settling for the best of %d "
                         "windows: distance %.3f (%.5f of its power), "
                         "in-sample fit %.1f%%\n",
                         s->v90_t3_acq_retries + 1,
                         s->v90_t3_acq_best_dist, s->v90_t3_acq_best_rel,
                         100.0f*best_match);
                good = true;
            }
            /*endif*/
            if (!good)
            {
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 T/3 B1 rejected: an in-sample fit of "
                         "%.1f%% that does not generalise (distance %.3f, "
                         "%.5f of its power against a limit of %.5f)\n",
                         100.0f*best_match, mean_dist, rel,
                         (double) V34_V90_T3_ACQ_GOOD_SNR);
                /* Not a failure of the call -- just of this window.  Search
                   again when more of the wire has arrived, sliding the
                   window forward: E is detected on the CP bit stream and
                   lags, so a rejected first attempt usually means B1 is
                   still ahead of the anchor rather than behind it.  The
                   backoff is not optional -- without it the search re-runs
                   on identical samples every symbol, which is tens of
                   millions of complex MACs per symbol in the media thread,
                   and the far end hears the stall as a discontinuity. */
                v90_t3_acq_retry_or_abandon(s, "out-of-sample distance");
                return;
            }
            /*endif*/
        }
        /*endif*/
    }
    if (s->x2_mode && best_trellis >= 0)
    {
        const uint8_t (*table)[16];
        int states;

        /* The accepted B1 selects the remote transmitter's convolutional
           code (Courier B1C2). Use it for DATA as well as the template. */
        if (trellis_parameters(best_trellis, &table, &states)
            || viterbi_set_trellis(&s->viterbi, table, states))
            return;
        /* V.34 9.6.3, Table 11: this x2 short-record profile has h=0,
           hence C0=0 and U0=Y0 XOR V0. Acquisition replays the reset-state
           B1 from its first symbol, so its input epoch is authoritative.
           Retain this constraint to correct isolated channel errors. */
        s->viterbi.zero_precoder = true;
        for (int i = 0; i < 3; i++)
            if (s->h[i].re != 0 || s->h[i].im != 0)
                s->viterbi.zero_precoder = false;
    }
    memcpy(s->v90_t3_fse, best_coeff, sizeof(best_coeff));
    v90_t3_fse_taps_replaced(s);
    s->v90_t3_training_match = best_match;
    s->v90_t3_fse_conjugate = best_conjugate;
    s->v90_t3_next_symbol = best_first;
    /* process_primary_symbol() has a 15-step Viterbi wind-up, so publishing
       is held off past the end of B1; ODP is repeated, so a bounded post-B1
       guard loses no V.42 information.
       The guard must be a whole number of DATA FRAMES, not just of mapping
       frames.  A data frame is 8*p symbols -- 128 at 3200 baud -- and the
       shell decoder's position within it is part of its state, so the old
       flat 32-symbol flush left the decoder a quarter of a data frame out of
       phase with the transmitter.  Measured live: symbols landing on the grid
       at ~30 dB, an exact-symbol decode that is bit-perfect offline, and a
       published bit stream that was nevertheless pure white. */
    {
        int frame_symbols = 8*s->parms.p;
        /* x2 replays the specified single reset-state B1. The traceback
           delays output by 15 4D pairs; suppress its trailing B1 bits, not
           a complete extra DATA frame (V.34 10.1.3.1/11.4.1.1.5). */
        int flush = s->x2_mode ? 2*15 : (frame_symbols > 0) ? frame_symbols : 32;

        s->v90_t3_publish_symbol =
            best_first + 3*(s->v90_t3_b1_symbols + flush);
    }
    s->v90_t3_suppress_output = true;
    s->v90_t3_acquired = true;
    s->v90_t3_b1_start = best_first;
    s->v90_t3_b1_frame_err = 0.0f;
    s->v90_t3_in_b1 = true;
    s->v90_t3_data_symbols = 0;
    s->v90_t3_phase_delta = 0;
    s->v90_t3_phase_pending = false;
    s->v90_t3_phase_pos = 0;
    s->v90_t3_sf_locked = false;
    s->v90_t3_idle_seen = false;
    s->v90_t3_relocks = 0;
    s->v90_t3_sym_err_ema = 0.0f;
    s->v90_t3_err_base = 0.0f;
    s->v90_t3_err_base_n = 0;
    s->v90_t3_lost_run = 0;
    s->v90_t3_resync_misses = 0;
    s->v90_t3_resync_required = false;
    s->v90_t3_lost_reported = 0;
    s->v90_t3_sym_err_fast = 0.0f;
    s->v90_t3_shell_frames = 0;
    s->v90_t3_shell_bad = 0;
    s->v90_t3_v14_hist = 0;
    s->v90_t3_v14_bits = 0;
    memset(s->v90_t3_v14_ok, 0, sizeof(s->v90_t3_v14_ok));
    memset(s->v90_t3_v14_count, 0, sizeof(s->v90_t3_v14_count));
    s->v90_t3_fse_good_valid = false;
    s->v90_t3_fse_good_age = 0;
    s->v90_t3_fse_bad_run = 0;
    s->v90_t3_fse_restores = 0;
    s->v90_t3_sf_tries = 0;
    /* Per call, not per process -- this server runs many calls in one. */
    s->v90_t3_cma_p2 = 0.0;
    s->v90_t3_cma_p4 = 0.0;
    s->v90_t3_cma_n = 0;
    /* NOT r2: this reset runs inside the acquisition, AFTER the B1 template
       build that measures it from the constellation, so zeroing it here threw
       the figure away and the blind loop never armed at all. */
    s->v90_t3_cma_active = false;
    s->v90_t3_cma_run = 0;
    s->v90_t3_cma_episodes = 0;
    /* Decision-directed adaptation step (gated by v90_t3_dd_gate_ok()). */
    s->v90_t3_dd_mu = 0.02f;
    {
        s->v90_t3_carrier_enabled = true;
        v34_carrier_init(&s->v90_t3_carrier);
        {
            /* Held by default.  A fourth-power line is a real thing on a
               4-point training constellation and a much weaker one on the
               768-point shaped constellation this receiver runs at 28800, so
               on a dense QAM the term integrates mostly noise into the
               frequency -- and it runs exactly when the eye is shut, which is
               the moment the receiver can least afford its frequency steered
               by a guess.  Measured on the rate matrix replays: holding it
               takes 19200 from 18% of the call clean to 48% and its longest
               hold from 18.7 s to 36.2 s, and 28800 from 17% to 23%, with
               nothing anywhere getting worse.  It is also what makes the
               blind recovery below worth anything at all. */
            s->v90_t3_carrier.nda_freq_hold = 1;
        }
    }
    {
        /* Without timing recovery this receiver decodes correctly for about
           fifteen seconds and then walks off, which is what the whole
           upstream investigation kept running into.  The gains are the ones
           v34_gardner_test holds to an S-curve, a static offset, a 50 ppm
           ramp and a perfect clock. */
        s->v90_t3_timing_enabled = true;
        {
            s->v90_t3_slip_mult = V34_V90_T3_SLIP_MULT;

            v34_gardner_init(&s->v90_t3_gardner,
                             V34_GARDNER_DEFAULT_MU,
                             V34_GARDNER_DEFAULT_BETA);
            /* Which timing error detector.  Non-data-aided Gardner is the
               one this loop was built on, and it is the wrong one once a
               dense constellation is carrying data: see v34_gardner.h and
               docs/v90_upstream_data_path.md for the matrix that says so. */
            s->v90_t3_gardner.detector = V34_GARDNER_DET_MM;
        }
    }
    /* The supervised filter already maps onto the exact Q9.7 template grid. */
    s->data_symbol_scale = 1.0f;
    s->data_symbol_rotation = 0;
    s->data_symbol_conjugate = false;
    s->phase4_da_derot = 0;
    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
             "Rx - V.90 upstream frame parms: b=%d p=%d w=%d j=%d k=%d "
             "bit_rate=%d b1_symbols=%d publish=+%d symbols\n",
             s->parms.b, s->parms.p, s->parms.w, s->parms.j, s->parms.k,
             s->bit_rate, s->v90_t3_b1_symbols, s->v90_t3_b1_symbols + 32);
    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
             "Rx - V.90 upstream B1 acquired at %d Hz T/3 "
             "(baud=%d carrier=%s sample=%lld, symbols=%d, fit=%.1f%%, "
             "conjugate=%d, tap=%d, trellis=%d)\n",
             s->v90_t3_internal_rate,
             baud_rate_parameters[s->baud_rate].baud_rate,
             s->high_carrier ? "high" : "low",
             (long long)best_first, s->v90_t3_b1_symbols,
             100.0f*best_match, best_conjugate ? 1 : 0, best_tap,
             best_trellis);
    /* Print the template and the symbols the emitter actually produces from
       the same samples.  Acquisition says these agree to 98.6% of energy;
       the DATA-path probe says the emitted ones are off the lattice.  Both
       cannot be true, and this is where the two measurements meet. */
    {
        char line[512];
        int len = 0;
        int pre = V34_V90_T3_FSE_TAPS/2;

        for (int n = 0;  n < 6  &&  len < (int) sizeof(line) - 32;  n++)
        {
            complexf_t y = {0.0f, 0.0f};

            for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
            {
                complexf_t x = v90_t3_raw_get(s, best_first + 3*n - pre + tap);
                complexf_t z;

                if (s->v90_t3_fse_conjugate)
                    x.im = -x.im;
                /*endif*/
                z = complex_mulf(&s->v90_t3_fse[tap], &x);
                y = complex_addf(&y, &z);
            }
            len += snprintf(line + len, sizeof(line) - len,
                            "%s(%.1f,%.1f)->(%.1f,%.1f)", n ? " " : "",
                            s->v90_t3_b1[n].re, s->v90_t3_b1[n].im,
                            y.re, y.im);
        }
        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                 "Rx - V.90 upstream B1 template->emitted: %s\n", line);
    }
    /* Seed the carrier loop's FREQUENCY from B1, exactly as the ordinary V.34
       data path does (see "Rx - B1 residual carrier" above).  The loop here
       only ever had decision-directed and fourth-power updates, and neither
       can ACQUIRE a frequency once the first decisions are already wrong --
       it holds a lock it cannot take.  In plain V.34 that is precisely what
       made every rate above 12000 bit/s decode to white, and this upstream is
       31200 bit/s at 3200 baud, 9.75 bits/symbol, far the other side of that
       line.  10.1.3.1's B1 is a known sequence, so the estimate owes nothing
       to a decision: correlate its two halves separately and the angle
       between them is the advance per symbol.  Averaging coherently inside
       each half before taking an angle is what makes it work over so few
       symbols -- the obvious one-lag autocorrelation reads an order of
       magnitude high because it never averages the noise down first. */
    {
        {
            complexf_t c0 = {0.0f, 0.0f};
            complexf_t c1 = {0.0f, 0.0f};
            int half = s->v90_t3_b1_symbols/2;
            int pre = V34_V90_T3_FSE_TAPS/2;

            for (int n = 0;  n < s->v90_t3_b1_symbols;  n++)
            {
                complexf_t y = {0.0f, 0.0f};
                complexf_t *acc = (n < half) ? &c0 : &c1;
                complexf_t e = s->v90_t3_b1[n];

                for (int tap = 0;  tap < V34_V90_T3_FSE_TAPS;  tap++)
                {
                    complexf_t x = v90_t3_raw_get(s, best_first + 3*n - pre + tap);
                    complexf_t z;

                    if (s->v90_t3_fse_conjugate)
                        x.im = -x.im;
                    /*endif*/
                    z = complex_mulf(&s->v90_t3_fse[tap], &x);
                    y = complex_addf(&y, &z);
                }
                /*endfor*/
                /* y * conj(e) */
                acc->re += y.re*e.re + y.im*e.im;
                acc->im += y.im*e.re - y.re*e.im;
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
                int ok = (fabsf(dphi) < V34_CARRIER_FREQ_LIMIT);

                /* Anything approaching half a turn per symbol is a wrapped
                   measurement rather than a carrier offset; refuse it. */
                if (ok)
                    s->v90_t3_carrier.freq = dphi;
                /*endif*/
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 upstream B1 residual carrier: %.4f "
                         "deg/symbol (%.2f Hz at this rate)%s\n",
                         dphi*180.0f/3.14159265f,
                         dphi*180.0f/3.14159265f
                           *baud_rate_parameters[s->baud_rate].baud_rate/360.0f,
                         ok ? "" : " - rejected as wrapped");
            }
            /*endif*/
        }
        /*endif*/
    }
    v90_t3_emit_ready(s);
}

static void v90_t3_put_sample(v34_rx_state_t *s, complexf_t value)
{
    double angle = -2.0*M_PI
                 * carrier_frequency(s->baud_rate, s->high_carrier)
                 * (double)s->v90_t3_output_count/s->v90_t3_internal_rate;
    complexf_t mixed;
    complexf_t filtered = {0.0f, 0.0f};

#if defined(V34_FIXED_POINT)
    /* Incremental phase.  The float lines below recompute the angle from the
       running output count in DOUBLE, which reaches ~5.8M on a 600 s call -- a
       float mantissa cannot hold that, which is why those lines are double and
       why this is the one genuine per-sample soft-float cost on an FPU-less
       part.  Measured against them over a whole call's samples: worst phasor
       error 8.4e-04, rms 4.9e-04. */
    {
        int32_t nc;
        int32_t ns;

        v34_fx_nco_step(&s->v90_t3_nco, &nc, &ns);
        mixed.re = value.re*(nc/1073741824.0f) - value.im*(ns/1073741824.0f);
        mixed.im = value.re*(ns/1073741824.0f) + value.im*(nc/1073741824.0f);
    }
    (void) angle;
#else
    mixed.re = value.re*(float)cos(angle) - value.im*(float)sin(angle);
    mixed.im = value.re*(float)sin(angle) + value.im*(float)cos(angle);
#endif
    s->v90_t3_rrc[s->v90_t3_rrc_pos] = mixed;
    if (++s->v90_t3_rrc_pos >= V34_V90_T3_RRC_TAPS)
        s->v90_t3_rrc_pos = 0;
#if defined(V34_FIXED_POINT)
    /* 97 complex taps at 9600 Hz is 931k MAC/s -- the largest float load in
       this receiver, an order above the FSE's 67k.  Measured against the float
       filter on 20000 samples: 3.8e-07 relative, 128.3 dB. */
    {
        int wr = (s->v90_t3_rrc_pos == 0) ? V34_V90_T3_RRC_TAPS - 1 : s->v90_t3_rrc_pos - 1;
        v34_fx_complex_t fo;

        int rs2 = s->v90_t3_fx_rshift ? s->v90_t3_fx_rshift : V34_FX_RING_SHIFT;

        s->v90_t3_rrc_fx[wr].re = v34_fx_from_float(mixed.re, rs2);
        s->v90_t3_rrc_fx[wr].im = v34_fx_from_float(mixed.im, rs2);
        fo = v34_fx_fir(s->v90_t3_rrc_coeff_fx, s->v90_t3_rrc_fx,
                        s->v90_t3_rrc_pos, V34_V90_T3_RRC_TAPS);
        filtered.re = v34_fx_to_float(fo.re, rs2);
        filtered.im = v34_fx_to_float(fo.im, rs2);
    }
#else
    for (int tap = 0;  tap < V34_V90_T3_RRC_TAPS;  tap++)
    {
        int pos = s->v90_t3_rrc_pos - 1 - tap;
        if (pos < 0)
            pos += V34_V90_T3_RRC_TAPS;
        filtered.re += s->v90_t3_rrc_coeff[tap]*s->v90_t3_rrc[pos].re;
        filtered.im += s->v90_t3_rrc_coeff[tap]*s->v90_t3_rrc[pos].im;
    }
#endif
    /* The RRC establishes the timing eye; the supervised fractionally spaced
       solve uses the pre-RRC T/3 samples, as the reference receiver does, so
       the adaptive filter owns the complete channel rather than inverting a
       separately truncated matched filter. */
    s->v90_t3_raw[s->v90_t3_raw_count & V34_V90_T3_RAW_MASK] = mixed;
#if defined(V34_FIXED_POINT)
    /* One conversion per INPUT sample.  The FSE reads the ring 21 times per
       symbol, so converting here rather than per tap is what makes a
       fixed-point FSE cheaper than the float one on an FPU-less part. */
    {
        v34_fx_complex_t *q = &s->v90_t3_raw_fx[s->v90_t3_raw_count & V34_V90_T3_RAW_MASK];

        int rs = s->v90_t3_fx_rshift ? s->v90_t3_fx_rshift : V34_FX_RING_SHIFT;

        q->re = v34_fx_from_float(mixed.re, rs);
        q->im = v34_fx_from_float(mixed.im, rs);
    }
#endif
    s->v90_t3_matched[s->v90_t3_raw_count & V34_V90_T3_RAW_MASK] = filtered;
    s->v90_t3_raw_count++;
    s->v90_t3_output_count++;
    if (!s->v90_t3_acquired)
    {
        if (s->v90_t3_e_anchor >= 0)
            v90_t3_try_acquire(s);
        /*endif*/
    }
    else
    {
        v90_t3_emit_ready(s);
    }
    /*endif*/
}

static int v90_t3_primary_rx(v34_rx_state_t *s, const int16_t amp[], int len)
{
    const int half = V34_V90_T3_RESAMPLE_TAPS/2;

    for (int i = 0;  i < len;  i++)
    {
        complexf_t analytic = {0.0f, 0.0f};
        int64_t n;

        if (!s->v90_t3_capture_only)
        {
            /* In capture-only mode the ordinary receiver is still running on
               these same samples and owns this accounting. */
            s->qam_sample_time++;
            (void) power_meter_update(&s->power, amp[i]);
        }
        /*endif*/
        s->v90_t3_hilbert[s->v90_t3_hilbert_pos] = amp[i];
        if (++s->v90_t3_hilbert_pos >= V34_V90_T3_HILBERT_TAPS)
            s->v90_t3_hilbert_pos = 0;
        /* Streaming analytic signal.  Form it before resampling, matching the
           proven reference receiver and preventing the negative-frequency
           image from sitting only 256 Hz beyond the RRC transition band. */
        {
            int centre = V34_V90_T3_HILBERT_TAPS/2;
            int real_pos = s->v90_t3_hilbert_pos - 1 - centre;
            if (real_pos < 0)
                real_pos += V34_V90_T3_HILBERT_TAPS;
            analytic.re = s->v90_t3_hilbert[real_pos];
            for (int tap = 0;  tap < V34_V90_T3_HILBERT_TAPS;  tap++)
            {
                int m = tap - centre;
                int pos;
                double window;
                if (m == 0 || (m & 1) == 0)
                    continue;
                pos = s->v90_t3_hilbert_pos - 1 - tap;
                if (pos < 0)
                    pos += V34_V90_T3_HILBERT_TAPS;
                window = 0.42 - 0.5*cos(2.0*M_PI*tap
                                        /(V34_V90_T3_HILBERT_TAPS - 1))
                         + 0.08*cos(4.0*M_PI*tap
                                    /(V34_V90_T3_HILBERT_TAPS - 1));
                analytic.im += (float)(window*2.0/(M_PI*m))
                             * s->v90_t3_hilbert[pos];
            }
        }
        n = s->v90_t3_input_count++;
        {
            int ring = (int)(n % (V34_V90_T3_RESAMPLE_TAPS + 8));
            s->v90_t3_input[ring] = analytic;
        }
        /* Exact rational interpolation from the 8 kHz bearer to three
           samples/symbol: 9 kHz (9/8) at 3000 baud and 9.6 kHz (6/5) at
           3200.  Publication is delayed by half the window; external input
           accounting remains exactly the caller's 8 kHz sample count. */
        while (8000*s->v90_t3_next_output
               <= (int64_t)s->v90_t3_internal_rate*(n - half))
        {
            double t = 8000.0*s->v90_t3_next_output
                     / s->v90_t3_internal_rate;
            int centre = (int)floor(t);
            complexf_t y = {0.0f, 0.0f};
            double norm = 0.0;
            for (int k = centre - half;  k <= centre + half;  k++)
            {
                double d;
                double sinc;
                double window;
                double h;
                int pos;
                if (k < 0 || k > n)
                    continue;
                d = t - k;
                sinc = fabs(d) < 1e-12 ? 1.0 : sin(M_PI*d)/(M_PI*d);
                window = 0.42 + 0.5*cos(M_PI*d/(half + 1.0))
                         + 0.08*cos(2.0*M_PI*d/(half + 1.0));
                h = sinc*window;
                pos = k % (V34_V90_T3_RESAMPLE_TAPS + 8);
                y.re += (float)h*s->v90_t3_input[pos].re;
                y.im += (float)h*s->v90_t3_input[pos].im;
                norm += h;
            }
            if (fabs(norm) > 1e-12)
            {
                y.re /= (float)norm;
                y.im /= (float)norm;
            }
            v90_t3_put_sample(s, y);
            s->v90_t3_next_output++;
        }
    }
    return 0;
}

/* V.90 9.5.1.2 / V.34 11.5.1.2 retrain watch.  Split out of
 * primary_channel_rx() so it can run before the data-mode branch
 * returns: with the T/3 upstream receiver active the whole of the
 * rest of that function is skipped in DATA, which is precisely where
 * a peer whose own receiver has failed starts holding Tone A. */
/* Is the responder for V.90 9.6.2 / V.34 11.6 enabled?
 *
 * Covers both stacks -- V.90 9.6.2 and plain V.34 11.6 send the same S.
 *
 * Default ON for plain V.34 and OFF for V.90 9.6 (see the function).  It was
 * off for both because it had never been exercised against a peer that starts
 * one.  Before the RasFinder, nothing in the recorded corpus renegotiated -- every
 * capture is of a call that either held data mode or died -- so the only
 * measurement available is the negative one: over the twelve recorded
 * rate-matrix calls, more than twenty minutes of live data-mode audio, this
 * detector fires zero times.  That bounds the false-positive rate and says
 * nothing at all about the true-positive rate.  A false detection would take
 * down a working call, so it stays behind the knob until a peer proves it.
 */
static int v34_reneg_respond_enabled(const v34_rx_state_t *s)
{
    static int cached = -2;

    if (cached == -2)
    {
        const char *e = getenv("ME_V90_RENEG_RESPOND");

        cached = (e  &&  *e)  ?  (atoi(e) != 0)  :  -1;
    }
    /*endif*/
    if (cached >= 0)
        return cached;
    /*endif*/
    /* Unset: on for plain V.34 (including a V.90 call that fell back to it),
       off for V.90's 9.6.  The plain V.34 half has now met a peer that starts
       one: the RasFinder opens an 11.6 renegotiation ~20 s into every data
       mode, and answered (with v34_answer_rate_renegotiation()'s conditioning
       and the B1 fallback in v34rx_data.c) a 300 s call held throughout where
       every call before died at the first one -- 10 of 11 completed, the
       other recovering through 11.5 (artifacts/rf-bbs-6).  V.90 9.6 has not. */
    return (!s->v90_mode  ||  s->v90_v34_fallback)  ?  1  :  0;
}
/*- End of function --------------------------------------------------------*/

/* V.90 9.6.1.2 / V.34 11.6.1.2: the peer has opened a rate renegotiation and
   we have to answer it.
 *
 * The cheap resynchronisation is the one worth having -- V.34 11.6 says
 * outright that the procedure "can also be used to resynchronize the receiver
 * without going through a complete retrain" -- but it is also the one that
 * has to be detected on a receiver whose eye may be shut, which is exactly
 * the state it exists to fix.  So do not look at symbols.
 *
 * 10.1.3.7's S alternates between point 0 of the quarter-superconstellation
 * and the same point rotated COUNTERCLOCKWISE BY 90 DEGREES.  Ninety, not a
 * hundred and eighty: a,ja,a,ja,... is a(1+j)/2 plus a(1-j)/2*(-1)^k, so the
 * energy sits in equal parts on a line at the carrier and a line at
 * fc +/- baud/2 -- three bins to watch, not two.  Measured on a real S from
 * this tree's own transmitter, watching only fc +/- baud/2 caught 0.44 of the
 * block energy and never crossed a 0.60 gate, which is exactly the half the
 * decomposition predicts.
 *
 * Three narrow bins holding most of a block still separates S from a data
 * mode primary channel, whose power is spread over the band by construction,
 * and it needs neither the equalizer, the carrier loop nor the timing loop to
 * be working.
 *
 * V.34 5.1 puts the carrier at fc = S*d/e, with d/e from Table 1 -- which is
 * the low_high[] pair in baud_rate_parameters, and is why 3200 baud low reads
 * 1828.6 Hz rather than 2400*4/7.
 *
 * Block length is 10 ms because the window is short: S is 128T and S-bar 16T,
 * which is 45 ms at 3200 baud and 42 ms at 3429, so three blocks of 10 ms fit
 * inside it with margin while 20 ms blocks would not reliably. */
static void v34_rx_watch_peer_reneg_s(v34_rx_state_t *s,
                                      const int16_t amp[],
                                      int len)
{
    /* 80 samples = 10 ms. */
    static const int block = 80;
    const baud_rate_parameters_t *p;
    float baud;
    float fc;
    float freq[3];
    float coeff[3];
    int i;
    int k;

    /* Responding (§9.6.1.2) is a data-mode watch by definition.  Detecting the
       answer to a renegotiation WE opened is not: by then the receiver has
       been conditioned for CP and its stage is V34_RX_STAGE_V90_CP, so gating
       on DATA silently disabled exactly the case §9.6.1.1.1 requires.  The
       watch flag is only ever armed across our own Rd, so it carries its own
       scope. */
    if ((s->stage != V34_RX_STAGE_DATA  &&  !s->reneg_s_watch)
        ||
        (!v34_reneg_respond_enabled(s)  &&  !s->reneg_s_watch)
        ||
        s->baud_rate < 0  ||  s->baud_rate >= 6)
    {
        memset(s->reneg_s_g1, 0, sizeof(s->reneg_s_g1));
        memset(s->reneg_s_g2, 0, sizeof(s->reneg_s_g2));
        s->reneg_s_energy = 0.0f;
        s->reneg_s_samples = 0;
        s->reneg_s_blocks = 0;
        s->reneg_s_reported = false;
        return;
    }
    /*endif*/
    p = &baud_rate_parameters[s->baud_rate];
    baud = 2400.0f*(float) p->a/(float) p->c;
    fc = baud*(float) p->low_high[s->high_carrier ? 1 : 0].d
             /(float) p->low_high[s->high_carrier ? 1 : 0].e;
    freq[0] = fc - baud/2.0f;
    freq[1] = fc;
    freq[2] = fc + baud/2.0f;
    for (k = 0;  k < 3;  k++)
        coeff[k] = 2.0f*cosf(2.0f*3.14159265358979f*freq[k]/8000.0f);
    /*endfor*/

    for (i = 0;  i < len;  i++)
    {
        float x = (float) amp[i];

        for (k = 0;  k < 3;  k++)
        {
            float g0 = x + coeff[k]*s->reneg_s_g1[k] - s->reneg_s_g2[k];

            s->reneg_s_g2[k] = s->reneg_s_g1[k];
            s->reneg_s_g1[k] = g0;
        }
        /*endfor*/
        s->reneg_s_energy += x*x;
        if (++s->reneg_s_samples >= block)
        {
            /* For a full-block sine the Goertzel power is energy*N/2, so a
               signal that is entirely these lines approaches 1.0 here. */
            float denom = s->reneg_s_energy*(float) block*0.5f;
            float sum = 0.0f;
            bool tonal = false;

            for (k = 0;  k < 3;  k++)
            {
                sum += s->reneg_s_g1[k]*s->reneg_s_g1[k]
                     + s->reneg_s_g2[k]*s->reneg_s_g2[k]
                     - coeff[k]*s->reneg_s_g1[k]*s->reneg_s_g2[k];
            }
            /*endfor*/
            /* Same energy floor as the retrain tone watch: mean square over
               100^2, so line noise and silence never qualify. */
            if (s->reneg_s_energy > 10000.0f*(float) block
                &&
                denom > 0.0f
                &&
                sum > 0.60f*denom)
            {
                tonal = true;
            }
            /*endif*/
            if (tonal)
                s->reneg_s_blocks++;
            else
                s->reneg_s_blocks = 0;
            /*endif*/
            {
                /* Opt-in, and it caches its getenv: what the bins actually
                   read, for calibrating the ratio against a real S rather
                   than an assumed one.  This is what found the 90-vs-180
                   error above. */
                static int debug = -1;

                if (debug < 0)
                    debug = (V34_DIAG_GETENV("V34_RENEG_S_DEBUG") != NULL);
                /*endif*/
                if (debug)
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - 11.6 S watch: caller=%d hi=%d "
                             "%.0f/%.0f/%.0f Hz sum/denom=%.3f energy=%.3g "
                             "run=%d\n",
                             s->calling_party, s->high_carrier ? 1 : 0,
                             (double) freq[0], (double) freq[1],
                             (double) freq[2],
                             (denom > 0.0f) ? (double) (sum/denom) : -1.0,
                             (double) s->reneg_s_energy,
                             s->reneg_s_blocks);
                }
                /*endif*/
            }
            memset(s->reneg_s_g1, 0, sizeof(s->reneg_s_g1));
            memset(s->reneg_s_g2, 0, sizeof(s->reneg_s_g2));
            s->reneg_s_energy = 0.0f;
            s->reneg_s_samples = 0;
            if (s->reneg_s_blocks >= 3  &&  !s->reneg_s_reported)
            {
                s->reneg_s_reported = true;
                s->received_event = V34_EVENT_PEER_RENEG_S;
                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - S detected in DATA (%d ms at %.0f/%.0f/%.0f Hz); "
                         "the peer has opened a rate renegotiation, answering "
                         "per 9.6.1.2/11.6.1.2\n",
                         s->reneg_s_blocks*block/8,
                         (double) freq[0], (double) freq[1], (double) freq[2]);
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

/* Phase 3 far-end S, measured on the line samples rather than on equalized
 * symbols.
 *
 * The constellation-domain detector in v34rx_phase3.c needs an open eye, and
 * an eye needs the sampling instant to be near the centre of the far end's
 * symbol.  Over a digital bearer that is free: both ends run off the same
 * 8 kHz grid.  Over a real analogue line it is not, and measured against this
 * project's own digital modem through a Conexant HSF DAA, sweeping the receive
 * sampling phase over one 16 kHz sample makes the difference between reaching
 * Phase 4 and sitting in PHASE3_WAIT_S for the whole call -- about a tenth of
 * the phases work.  In the same recording, at a failing phase, the three-bin
 * measurement below reads 0.933 with a 50 ms run, which is 10.1.3.7's S
 * (128T + 16T = 60 ms at 2400 baud) seen plainly.  So S is detectable at any
 * phase; only this detector was.
 *
 * Detecting S is enough to unblock the rest, because PP conditioning that
 * follows carries the T/2 eye-phase chooser, which is what fixes the instant.
 *
 * Not enabled: it publishes V34_EVENT_S without any of the J/TRN evidence
 * the alternation path carries, and it did not rescue the HSF call it was
 * written for (PP conditioning after S stays constellation-domain). */
static int v34_phase3_s_spectral_enabled(void)
{
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* Are we the V.34 CALL modem?  In plain V.34 that is the calling party.  On
 * a V.90 call it is true exactly when the analogue modem's INFO1a selected
 * V.34 (V.90 9.2.1.1.8: the digital modem then proceeds per 11.3.1.1/V.34
 * "assuming the role of a call modem"), whichever side placed the call.  The
 * plain-V.34 call-modem fixes used to test !v90_mode && calling_party, which
 * left that fallback without them: against the RasFinder it missed the
 * peer's Phase 4 S at 34.4 s on every attempt (rf-tower-fb-2). */
static bool v34_rx_is_v34_call_modem(const v34_rx_state_t *s)
{
    if (s->v90_mode)
        return s->v90_v34_fallback;
    /*endif*/
    return s->calling_party;
}
/*- End of function --------------------------------------------------------*/

/* Plain V.34 call modem, our J on the air: V.34 11.3.1.1.7 has it "send
 * sequence J and condition its receiver to detect signal S", and the S that
 * comes is the answer modem's Phase 4 S of 11.4.1.2.1 -- 128T, then S-bar and
 * TRN.  Against the RasFinder (MT5634SMI) from a wired host both plain V.34
 * calls of 2026-09-30 (rf-tower-v34-1-12840, -2-7702) lost the call here: the
 * peer's S arrives ~0.65 s after our J, decodes cleanly offline on the taps
 * trained on its own Phase 3 (alternating dibits 1,3 for exactly 128T), and
 * the constellation-domain detector missed it both times, because the T/2 eye
 * chooser moved the symbol instant in the middle of those 40 ms -- its
 * 256-symbol window straddled our own echo and the S onset.  The peer then
 * gave up after ~0.35 s of TRN and retrained, and the rotation-form detector
 * read that Tone A as S.  The three-bin measurement needs no eye, and on both
 * taps fires after exactly three 10 ms blocks of the real S (ratio 0.98-1.00)
 * and nowhere else.  Arming it only once our J is on the air keeps our own
 * S from being read through the echo when both directions share a carrier. */
bool v34_rx_caller_awaiting_phase4_s(v34_rx_state_t *s)
{
    const v34_state_t *owner;

    if (!v34_rx_is_v34_call_modem(s)
        ||  !s->duplex
        ||  s->stage != V34_RX_STAGE_PHASE3_WAIT_S)
        return false;
    /*endif*/
    owner = (const v34_state_t *) ((const char *) s - offsetof(v34_state_t, rx));
    return owner->tx.stage == V34_TX_STAGE_J;
}
/*- End of function --------------------------------------------------------*/

/* The answer modem's half of the same thing.  Its J goes out at the end of
 * its own Phase 3, it "conditions its receiver to detect S", and the S that
 * comes is the call modem's 11.3.1.1 Phase 3 S -- the same 128T, met by the
 * same constellation-domain detector and the same 256-symbol eye chooser.
 * Measured in v34_duplex_test at 3200/21600 against a -30 dB hybrid echo:
 * the chooser's window straddled the S onset, moved the symbol instant 80 ms
 * after it began, the alternation count never completed, and the answer
 * modem went on sending J into the call modem's whole Phase 3 -- which also
 * wrecked the call modem's echo canceller window.  At -20 dB the window
 * boundary happened to fall before S and the call trained.  The three-bin
 * watch fires on it either way.  Armed only while our own J is on the air,
 * as for the call modem, so our own S cannot be read back through the echo. */
static bool v34_rx_answerer_awaiting_phase3_s(v34_rx_state_t *s)
{
    const v34_state_t *owner;

    if (s->v90_mode
        ||  s->calling_party
        ||  !s->duplex
        ||  s->stage != V34_RX_STAGE_PHASE3_WAIT_S)
        return false;
    /*endif*/
    owner = (const v34_state_t *) ((const char *) s - offsetof(v34_state_t, rx));
    return owner->tx.stage == V34_TX_STAGE_J;
}
/*- End of function --------------------------------------------------------*/

/* Plain V.34 call modem sending its own Phase 3 (S, S-bar, PP, TRN, J).  V.34
 * 11.3.1.2.4 has the answer modem transmit silence from our S-to-S-bar
 * transition until its Phase 4 S, so everything this receiver hears in that
 * window is our own transmission coming back through the hybrid -- about
 * 1.1 s of it against the RasFinder.  The receiver used to keep adapting
 * through it: once the peer's Phase 3 TRN locked (100% on
 * rf-tower-v34p4s-1), blind CMA, the carrier loop, the band-edge timing loop
 * and the T/2 eye chooser all ran on that echo, a different transmitter with
 * a different carrier and symbol phase, and the peer's Phase 4 TRN that
 * followed read 55% ones -- chance -- on a signal that decodes to 8 degrees
 * from the grid with the Phase 3 taps left alone, and its MP never decoded.
 * Hold blind CMA there; 11.4.1.1.1 conditions the receiver on the peer's
 * Phase 4 TRN, which is where adapting resumes.  (The 55% TRN score is not
 * what changes -- the MP does: decoded and acknowledged instead of never.) */
/* Which loops v34_rx_caller_hearing_own_phase3() holds.
   Bits 1 blind CMA, 2 carrier tracking, 4 T/2 eye chooser, 8 band-edge timing;
   1, blind CMA only.  CMA is the one that matters: replayed on
   rf-tower-v34p4s-1 it alone takes the peer's MP from never decoding to
   decoding and acknowledged, and the call on to B1.  Holding the eye chooser
   as well (4) costs rf-v34-q3 -- a 09-28 call that reached data mode -- its
   whole Phase 4, and holding everything costs rf-tower-v34p4s-1 its B1, so
   those stay free.  Swept over d2, q1, q3 and both 2026-09-30 wired calls. */
static int v34_p3_echo_freeze_mask(void)
{
    return 1;
}

/* The plain V.34 answer modem's own Phase 3.  It goes out first, while the
   call modem waits in silence for its J (measured in v34_duplex_test: the
   call modem's transmit is exact zero from the end of INFO1a to the answer
   modem's J), so whatever the answer modem's receiver hears from its own S
   to the end of its own TRN is its own echo -- there is nothing else on the
   line.  On a clean bearer that is silence and the receiver idles; over a
   hybrid it is a perfectly formed PP, and the receiver acquired it: measured
   at 2400/9600 against a -30 dB echo, PP "detected" 0.3 s into the answer
   modem's own transmission, the AGC frozen at the echo's level, and when the
   call modem's real PP arrived 30 dB louder the conditioning clipped
   (|z| 25.6), reset the equalizer on every symbol and the call never trained.
   J is NOT included: the call modem's S may legitimately arrive while the
   answer modem is still sending J.  Only the PP acquisition is withheld; the
   other loops are left running, because freezing them too measurably cost
   2400/21600 its S detection on a clean bearer (15/16 delays -> 7/16). */
static bool v34_rx_answerer_sending_own_phase3(v34_rx_state_t *s)
{
    const v34_state_t *owner;

    if (s->v90_mode  ||  s->calling_party  ||  !s->duplex)
        return false;
    /*endif*/
    if (s->stage != V34_RX_STAGE_PHASE3_TRAINING)
        return false;
    /*endif*/
    owner = (const v34_state_t *) ((const char *) s - offsetof(v34_state_t, rx));
    return owner->tx.stage >= V34_TX_STAGE_FIRST_S
        &&  owner->tx.stage <= V34_TX_STAGE_TRN;
}

static bool v34_rx_caller_hearing_own_phase3_m(v34_rx_state_t *s, int what)
{
    return (v34_p3_echo_freeze_mask() & what)  &&  v34_rx_caller_hearing_own_phase3(s);
}

static bool v34_rx_caller_hearing_own_phase3(v34_rx_state_t *s)
{
    const v34_state_t *owner;

    if (!v34_p3_echo_freeze_mask()
        ||  !v34_rx_is_v34_call_modem(s)
        ||  !s->duplex
        ||  s->stage != V34_RX_STAGE_PHASE3_WAIT_S)
        return false;
    /*endif*/
    owner = (const v34_state_t *) ((const char *) s - offsetof(v34_state_t, rx));
    /* And only while what we hear is our echo: weak but present, and held
       there, not passing through on the way to silence.  The stage window
       alone also covers the answer modem's own Phase 4 S, S-bar and TRN when
       they arrive before we have detected S, and on a digital loopback the
       answer modem's silence is exact zero, where nothing adapts anyway --
       freezing through the fade of its J into that silence instead of after
       it moved the taps enough to cost 3429/21600 u-law its training, the
       row this matrix already calls a coin flip.  The peer at the
       equalizer's output is |z| ~ 1; our echo from the RasFinder is 0.03-0.14
       and steady. */
    return owner->tx.stage >= V34_TX_STAGE_FIRST_NOT_S
        &&  owner->tx.stage <= V34_TX_STAGE_J
        &&  s->p3_echo_run >= 96;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_b1_snr_db(v34_state_t *s, float *snr_db)
{
    float c2;

    if (!s  ||  !s->rx.b1_corr_valid)
        return -1;
    /*endif*/
    c2 = s->rx.b1_corr*s->rx.b1_corr;
    if (c2 >= 0.99999f)
        c2 = 0.99999f;
    /*endif*/
    if (snr_db)
        *snr_db = 10.0f*log10f(c2/(1.0f - c2));
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(bool) v34_rx_hearing_own_echo(v34_state_t *s)
{
    const v34_rx_state_t *rx;

    if (!s)
        return false;
    /*endif*/
    rx = &s->rx;
    return !rx->v90_mode
        &&  rx->calling_party
        &&  rx->duplex
        &&  rx->stage == V34_RX_STAGE_PHASE3_WAIT_S
        &&  s->tx.stage >= V34_TX_STAGE_FIRST_NOT_S
        &&  s->tx.stage <= V34_TX_STAGE_J
        &&  rx->p3_echo_run >= 96;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(bool) v34_rx_line_ec_window(v34_state_t *s)
{
    const v34_rx_state_t *rx;

    if (!s)
        return false;
    /*endif*/
    rx = &s->rx;
    /* A V.90 call that fell back to V.34 is a V.34 call from Phase 3 on, with
       this modem the call modem (V.90 9.2.1.1.8): its S-bar..TRN goes out
       while the analogue modem is silent (11.3.1.2.4) exactly as here. */
    if ((rx->v90_mode  &&  !rx->v90_v34_fallback)  ||  !rx->duplex)
        return false;
    /*endif*/
    /* From our S-bar to the end of our TRN.  J is excluded: the far end
       starts its next signal only once it has received J, so everything
       before J is guaranteed silent at the far end, while during J the far
       end's answer may already be arriving.  The call modem additionally
       must not yet have the answer modem's Phase 4 S. */
    if (s->tx.stage < V34_TX_STAGE_FIRST_NOT_S  ||  s->tx.stage > V34_TX_STAGE_TRN)
        return false;
    /*endif*/
    if (v34_rx_is_v34_call_modem(rx)  &&  rx->stage != V34_RX_STAGE_PHASE3_WAIT_S)
        return false;
    /*endif*/
    return true;
}
/*- End of function --------------------------------------------------------*/

static void v34_rx_watch_phase3_s(v34_rx_state_t *s,
                                  const int16_t amp[],
                                  int len)
{
    static const int block = 80;        /* 10 ms */
    const baud_rate_parameters_t *p;
    float baud;
    float fc;
    float freq[4];
    float coeff[4];
    int i;
    int k;

    if (!(v34_phase3_s_spectral_enabled()
          ||  v34_rx_caller_awaiting_phase4_s(s)
          ||  v34_rx_answerer_awaiting_phase3_s(s))
        ||
        s->stage != V34_RX_STAGE_PHASE3_WAIT_S
        ||
        s->phase3_s_present
        ||
        s->baud_rate < 0  ||  s->baud_rate >= 6)
    {
        memset(s->p3s_g1, 0, sizeof(s->p3s_g1));
        memset(s->p3s_g2, 0, sizeof(s->p3s_g2));
        s->p3s_energy = 0.0f;
        s->p3s_samples = 0;
        s->p3s_blocks = 0;
        {
            static int dbg = -1;

            if (dbg < 0)
                dbg = (V34_DIAG_GETENV("V34_PHASE3_S_DEBUG") != NULL);
            /*endif*/
            if (dbg  &&  s->stage == V34_RX_STAGE_PHASE3_WAIT_S)
            {
                const v34_state_t *o = (const v34_state_t *) ((const char *) s - offsetof(v34_state_t, rx));

                V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                         "Rx - Phase 3 S watch idle: enabled=%d present=%d baud_rate=%d "
                         "v90=%d fallback=%d calling=%d duplex=%d tx_stage=%d\n",
                         v34_phase3_s_spectral_enabled(), s->phase3_s_present,
                         s->baud_rate, s->v90_mode, s->v90_v34_fallback,
                         s->calling_party, s->duplex, o->tx.stage);
            }
            /*endif*/
        }
        return;
    }
    /*endif*/
    p = &baud_rate_parameters[s->baud_rate];
    baud = 2400.0f*(float) p->a/(float) p->c;
    fc = baud*(float) p->low_high[s->high_carrier ? 1 : 0].d
             /(float) p->low_high[s->high_carrier ? 1 : 0].e;
    freq[0] = fc - baud/2.0f;
    freq[1] = fc;
    freq[2] = fc + baud/2.0f;
    /* A fourth line at 2400 Hz, which is never one of S's three, to reject
       Tone A.  The answer modem's retrain tone is what follows a missed
       Phase 4 S, and this peer sends it with an 1800 Hz guard tone at about
       the same level -- which at 3000 baud low carrier (fc = 1800 Hz) is
       exactly S's middle line, putting half the block in the sum. */
    freq[3] = 2400.0f;
    for (k = 0;  k < 4;  k++)
        coeff[k] = 2.0f*cosf(2.0f*3.14159265358979f*freq[k]/8000.0f);
    /*endfor*/

    for (i = 0;  i < len;  i++)
    {
        float x = (float) amp[i];

        for (k = 0;  k < 4;  k++)
        {
            float g0 = x + coeff[k]*s->p3s_g1[k] - s->p3s_g2[k];

            s->p3s_g2[k] = s->p3s_g1[k];
            s->p3s_g1[k] = g0;
        }
        /*endfor*/
        s->p3s_energy += x*x;
        if (++s->p3s_samples >= block)
        {
            float denom = s->p3s_energy*(float) block*0.5f;
            float sum = 0.0f;
            float tone_a;

            for (k = 0;  k < 3;  k++)
            {
                sum += s->p3s_g1[k]*s->p3s_g1[k]
                     + s->p3s_g2[k]*s->p3s_g2[k]
                     - coeff[k]*s->p3s_g1[k]*s->p3s_g2[k];
            }
            /*endfor*/
            tone_a = s->p3s_g1[3]*s->p3s_g1[3]
                   + s->p3s_g2[3]*s->p3s_g2[3]
                   - coeff[3]*s->p3s_g1[3]*s->p3s_g2[3];
            if (s->p3s_energy > 10000.0f*(float) block
                &&
                denom > 0.0f
                &&
                sum > 0.60f*denom
                &&
                tone_a < 0.20f*denom)
            {
                s->p3s_blocks++;
            }
            else
            {
                s->p3s_blocks = 0;
            }
            /*endif*/
            {
                static int debug = -1;

                if (debug < 0)
                    debug = (V34_DIAG_GETENV("V34_PHASE3_S_DEBUG") != NULL);
                /*endif*/
                if (debug)
                {
                    V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                             "Rx - Phase 3 S watch: %.0f/%.0f/%.0f Hz "
                             "sum/denom=%.3f energy=%.3g run=%d\n",
                             (double) freq[0], (double) freq[1],
                             (double) freq[2],
                             (denom > 0.0f) ? (double) (sum/denom) : -1.0,
                             (double) s->p3s_energy, s->p3s_blocks);
                }
                /*endif*/
            }
            memset(s->p3s_g1, 0, sizeof(s->p3s_g1));
            memset(s->p3s_g2, 0, sizeof(s->p3s_g2));
            s->p3s_energy = 0.0f;
            s->p3s_samples = 0;
            if (s->p3s_blocks >= 3)
            {
                s->p3s_blocks = 0;
                s->phase3_s_present = true;
                s->phase3_s_event_count++;
                s->phase3_s_fired_symbol = -1;
                s->received_event = V34_EVENT_S;
                span_log(s->logging, SPAN_LOG_WARNING,
                         "Rx - Phase 3: far-end S detected in the sample domain "
                         "(count=%d, %.0f/%.0f/%.0f Hz)\n",
                         s->phase3_s_event_count,
                         (double) freq[0], (double) freq[1], (double) freq[2]);
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

/* V.90 digital modem: the analogue modem's 9.3.2.7 S, found while we send Jd.

   9.3.1.4/9.3.1.5 have the digital modem transmit Jd "and condition its
   receiver to detect signal S", and 9.3.2.7 has the analogue modem send S
   (and hold it until J'd) up to 5000 ms after it went silent.  Against the
   RasFinder, from a clean wired path (artifacts/rf-tower-scan1-*), that S
   starts ~1.9 s into our Jd -- in time -- but on the HIGH carrier (1920 Hz at
   3200 baud: 0.98 of the band energy on the 320/1920/3520 Hz lines, held for
   seconds) while the Phase 3 upstream before it was on the LOW carrier, which
   is where v34_rx_watch_phase3_carrier() left the receiver.  91 Hz off, the
   constellation-domain S detector missed it on three calls of four; Jd then
   expired (9.3.1.5 + our rtd allowance, ~3.1 s), we went back to waiting for
   Ja, and the peer retrained.  On the fourth it happened to fire.

   So look for S on the line, on both carriers, like the carrier watch does:
   S puts its power on fc and fc +/- baud/2, 20 ms blocks resolve the two
   carriers' line sets, and two consecutive blocks holding >= 0.6 of the
   energy on one set is S.  Armed only while we transmit Jd, when the peer's
   Ja is over (it ends on our Sd->S-bar-d, 9.3.2.4) and 9.3.2.7 allows nothing
   but silence or S -- so Ja, whose J pattern also puts energy on these
   lines, cannot be mistaken for it. */
static void v34_rx_watch_v90_jd_s(v34_rx_state_t *s,
                                  const int16_t amp[],
                                  int len)
{
    enum { BLK = 160 };
    float fc[2];
    float half;
    float coeff[6];
    int i;
    int k;

    if (!s->v90jd_s_armed
        ||  s->stage != V34_RX_STAGE_PHASE3_WAIT_S
        ||  (s->phase3_s_present  &&  !s->v90jd_dil_mode)
        ||  s->baud_rate < 0  ||  s->baud_rate >= 6)
    {
        memset(s->v90jd_g1, 0, sizeof(s->v90jd_g1));
        memset(s->v90jd_g2, 0, sizeof(s->v90jd_g2));
        s->v90jd_energy = 0.0f;
        s->v90jd_samples = 0;
        s->v90jd_blocks = 0;
        return;
    }
    /*endif*/
    half = 0.5f*baud_rate_parameters[s->baud_rate].baud_rate;
    fc[0] = carrier_frequency(s->baud_rate, 0);
    fc[1] = carrier_frequency(s->baud_rate, 1);
    for (k = 0;  k < 6;  k++)
        coeff[k] = 2.0f*cosf(2.0f*3.14159265f*(fc[k/3] + (float) (k%3 - 1)*half)/SAMPLE_RATE);
    /*endfor*/
    for (i = 0;  i < len;  i++)
    {
        float x = (float) amp[i];

        for (k = 0;  k < 6;  k++)
        {
            float g0 = x + coeff[k]*s->v90jd_g1[k] - s->v90jd_g2[k];

            s->v90jd_g2[k] = s->v90jd_g1[k];
            s->v90jd_g1[k] = g0;
        }
        /*endfor*/
        s->v90jd_energy += x*x;
        if (++s->v90jd_samples < BLK)
            continue;
        /*endif*/
        {
            float line[2] = {0.0f, 0.0f};
            float denom = s->v90jd_energy*(float) BLK*0.5f;
            bool busy = (s->v90jd_energy > 10000.0f*(float) BLK);
            int on = -1;

            for (k = 0;  k < 6;  k++)
            {
                line[k/3] += s->v90jd_g1[k]*s->v90jd_g1[k]
                           + s->v90jd_g2[k]*s->v90jd_g2[k]
                           - coeff[k]*s->v90jd_g1[k]*s->v90jd_g2[k];
            }
            /*endfor*/
            memset(s->v90jd_g1, 0, sizeof(s->v90jd_g1));
            memset(s->v90jd_g2, 0, sizeof(s->v90jd_g2));
            s->v90jd_energy = 0.0f;
            s->v90jd_samples = 0;
            if (busy  &&  denom > 0.0f)
            {
                if (fabsf(fc[0] - fc[1]) < 1.0f)
                {
                    /* 3429 baud has ONE carrier (1959.2 Hz), so both line sets are the same three
                       lines and "this set high, the other low" can never hold.  An Intel/Conexant
                       V.92 modem chose 3429 in INFO1a and its S, 0.4 s of 0.5-0.6 of the energy on
                       those lines, went unseen for the whole of Jd (artifacts/serial-flex-a2). */
                    if (line[0] > 0.45f*denom)
                        on = 0;
                    /*endif*/
                }
                else if (line[0] > 0.6f*denom  &&  line[1] < 0.2f*denom)
                    on = 0;
                else if (line[1] > 0.6f*denom  &&  line[0] < 0.2f*denom)
                    on = 1;
                /*endif*/
            }
            /*endif*/
            if (s->v90jd_holdoff > 0)
            {
                s->v90jd_holdoff--;
                on = -1;
            }
            /*endif*/
            s->v90jd_blocks = (on >= 0)  ?  s->v90jd_blocks + 1  :  0;
            if (s->v90jd_blocks >= 2)
            {
                s->v90jd_blocks = 0;
                if (s->v90jd_dil_mode)
                    s->v90jd_holdoff = 10;      /* 200 ms between events */
                else
                    s->v90jd_s_armed = false;
                /*endif*/
                s->phase3_s_present = true;
                s->phase3_s_event_count++;
                s->phase3_s_fired_symbol = -1;
                s->received_event = V34_EVENT_S;
                s->v90jd_line_event = true;
                span_log(s->logging, SPAN_LOG_WARNING,
                         "Rx - V.90 Phase 3: analogue S during Jd found on the line, %s carrier "
                         "(%.0f Hz; lines %.2f/%.2f; receiver on %s)\n",
                         on ? "high" : "low",
                         (double) fc[on],
                         (double) (line[0]/denom),
                         (double) (line[1]/denom),
                         s->high_carrier ? "high" : "low");
                return;
            }
            /*endif*/
        }
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

/* V.90 digital modem: take the analogue modem's Phase 3 carrier from its S.

   INFO1d carries a carrier choice for each symbol rate (V.34 10.1.2.3.4,
   V.90 8.2.3.2) and the Phase 3 receiver is tuned to the one we sent.  The
   RasFinder does not always use it: over seven originated calls with the
   same INFO1d row for 3200 baud (low) and the same INFO1a back, its S came up
   on the low carrier three times and on the high carrier four, and each time
   it chose high our receiver never acquired PP, missed Ja, and the peer
   retrained after 9.3.2.4's 1500 ms (artifacts/rasfinder-p4-201857Z,
   rasfinder-i1d-202451Z).  S (10.1.3.7) announces the carrier unambiguously:
   its power sits on three lines, fc and fc +/- baud/2, and over 20 ms blocks
   those three hold 0.9-1.0 of the energy on the carrier in use against ~0.01
   on the other.  So watch for S before PP and retune if it is on the other
   carrier.  One decision per Phase 3 entry. */
static void v34_rx_watch_phase3_carrier(v34_rx_state_t *s,
                                        const int16_t amp[],
                                        int len)
{
    enum { BLK = 160 };
    float fc[2];
    float half;
    float coeff[6];
    int i;
    int k;

    if (!s->v90_mode
        ||  s->calling_party
        ||  s->stage != V34_RX_STAGE_PHASE3_TRAINING
        ||  s->phase3_pp_started)
    {
        if (s->stage != V34_RX_STAGE_PHASE3_TRAINING)
            s->p3car_decided = false;
        /*endif*/
        memset(s->p3car_g1, 0, sizeof(s->p3car_g1));
        memset(s->p3car_g2, 0, sizeof(s->p3car_g2));
        s->p3car_energy = 0.0f;
        s->p3car_samples = 0;
        return;
    }
    /*endif*/
    if (s->p3car_decided)
        return;
    /*endif*/
    half = 0.5f*baud_rate_parameters[s->baud_rate].baud_rate;
    fc[0] = carrier_frequency(s->baud_rate, 0);
    fc[1] = carrier_frequency(s->baud_rate, 1);
    for (k = 0;  k < 6;  k++)
        coeff[k] = 2.0f*cosf(2.0f*3.14159265f*(fc[k/3] + (float) (k%3 - 1)*half)/SAMPLE_RATE);
    /*endfor*/
    for (i = 0;  i < len;  i++)
    {
        float x = (float) amp[i];

        for (k = 0;  k < 6;  k++)
        {
            float g0 = x + coeff[k]*s->p3car_g1[k] - s->p3car_g2[k];

            s->p3car_g2[k] = s->p3car_g1[k];
            s->p3car_g1[k] = g0;
        }
        /*endfor*/
        s->p3car_energy += x*x;
        if (++s->p3car_samples < BLK)
            continue;
        /*endif*/
        {
            float line[2] = {0.0f, 0.0f};
            float denom = s->p3car_energy*(float) BLK*0.5f;
            bool busy = (s->p3car_energy > 10000.0f*(float) BLK);

            for (k = 0;  k < 6;  k++)
            {
                line[k/3] += s->p3car_g1[k]*s->p3car_g1[k]
                           + s->p3car_g2[k]*s->p3car_g2[k]
                           - coeff[k]*s->p3car_g1[k]*s->p3car_g2[k];
            }
            /*endfor*/
            memset(s->p3car_g1, 0, sizeof(s->p3car_g1));
            memset(s->p3car_g2, 0, sizeof(s->p3car_g2));
            s->p3car_energy = 0.0f;
            s->p3car_samples = 0;
            if (busy)
            {
                int on = -1;

                if (line[0] > 0.6f*denom  &&  line[1] < 0.2f*denom)
                    on = 0;
                else if (line[1] > 0.6f*denom  &&  line[0] < 0.2f*denom)
                    on = 1;
                /*endif*/
                if (on >= 0)
                {
                    s->p3car_decided = true;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 Phase 3: analogue S is on the %s carrier (%.0f Hz; lines %.2f/%.2f)%s\n",
                             on ? "high" : "low",
                             fc[on],
                             line[0]/denom,
                             line[1]/denom,
                             (on != (s->high_carrier ? 1 : 0))
                             ? ", not the one INFO1d asked for; retuning" : "");
                    if (on != (s->high_carrier ? 1 : 0))
                    {
                        s->high_carrier = (on != 0);
                        s->v34_carrier_phase_rate = dds_phase_ratef(fc[on]);
                        create_godard_coeffs(&s->pri_ted,
                                             fc[on],
                                             baud_rate_parameters[s->baud_rate].baud_rate,
                                             0.99f);
                    }
                    /*endif*/
                    return;
                }
                /*endif*/
            }
            /*endif*/
        }
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void v34_rx_watch_peer_retrain(v34_rx_state_t *s,
                                      const int16_t amp[],
                                      int len)
{
    int i;

    /* The spec-mandated form of the same check: V.90 9.3.1/9.4.1 and 9.5.1.2,
     * and V.34 11.5.1.2, all require a modem that detects the peer's retrain
     * tone to respond to a retrain rather than carry on.  A peer initiating a
     * retrain (V.90 9.5.2.1, V.34 11.5.x) sends 70 +/- 5 ms of silence and
     * then holds its tone until it hears ours -- the SmartLink peer gives up
     * and drops the call about 3.1 s in.  The silence detector above can miss
     * the gap when transport filtering rings into it (the interop rig's
     * 257-tap polyphase resampler shaves the observed 80 ms gap below the
     * 60 ms threshold), so detect the tone itself: a Goertzel bin at
     * 2400 Hz against total block energy.  A modulated primary channel
     * (CPt/SCR/CP on the 1800 Hz carrier) spreads its power across the
     * band, and the periodic Phase 3 line spectra (S at 600/3000 Hz, J/Ja
     * harmonics) never put most of it in one bin, so require a dominant,
     * sustained single-bin ratio.  The same reasoning is what makes it safe
     * to run in DATA: the upstream there is a full V.34 primary channel,
     * whose power is spread over the whole band by construction. */
    /* NOT in half-duplex Phase 3.  V.34 12.2.1.1.4 has the source hold Tone B
       until it has received INFOh, and 12.3.1.1 gives it a further 70 ms of
       silence before S -- so the source's tone is NECESSARILY still on the
       line at the moment 12.3.2.1 has this end fall silent and start looking
       for S.  It is the ordinary tail of Phase 2, not a retrain request, and
       9.5.1.2/11.5.1.2 are about a peer that interrupts an established link.
       Measured against a Canon TR7560 sending a V.34 fax: INFOh goes out at
       tx_t=0.912 s, the recipient goes silent, and 308 ms later the watcher
       calls the still-present Tone B a peer retrain and tears the call down
       before Phase 3 has begun.  A loopback cannot show this -- there both
       ends are the same code with no propagation or processing delay, so the
       source's tone stops exactly when the recipient stops listening. */
    /* Clause 12 links have their own 12.7 recovery-tone detector below.
     * The clause 11 watcher also mistakes the primary-channel transition
     * for a retrain in PHASE4_S when driven by the production engine. */
    if (!s->duplex)
    {
        s->phase34_tone_a_blocks = 0;
        return;
    }
    /*endif*/
    if (v34_rx_stage_watches_retrain(s->stage))
    {
        /* Which tone the peer holds follows its ROLE, not the call
           direction.  V.34 11.2.1.1/11.2.1.2 give Tone B to the call modem
           and Tone A to the answer modem; V.90 8.2.3.1 keeps both timetables
           and hands them to the other end of the call.  That is exactly the
           predicate the control-channel receive carrier is already chosen on
           a few thousand lines up -- so the tone to listen for is simply the
           frequency this receiver is tuned to. */
        const bool listen_tone_a = (s->calling_party != s->v90_mode);
        /* 2*cos(2*pi*f/8000).  160 samples = 20 ms per block, and both 2400
           and 1200 Hz land exactly on a bin (48 and 24). */
        const float tone_a_coeff = listen_tone_a ? -0.6180339887f
                                                 :  1.1755705046f;
        static const int tone_a_block = 160;

        for (i = 0;  i < len;  i++)
        {
            float x = (float) amp[i];
            float g0 = x + tone_a_coeff*s->phase34_tone_a_g1 - s->phase34_tone_a_g2;

            s->phase34_tone_a_g2 = s->phase34_tone_a_g1;
            s->phase34_tone_a_g1 = g0;
            s->phase34_tone_a_energy += x*x;
            /* 2*cos(2*pi*1800/8000); 1800 Hz is bin 36 of the 160-sample
               block. */
            g0 = x + 0.3128689301f*s->phase34_guard_g1 - s->phase34_guard_g2;
            s->phase34_guard_g2 = s->phase34_guard_g1;
            s->phase34_guard_g1 = g0;
            if (++s->phase34_tone_a_samples >= tone_a_block)
            {
                float g1 = s->phase34_tone_a_g1;
                float g2 = s->phase34_tone_a_g2;
                float tone_power = g1*g1 + g2*g2 - tone_a_coeff*g1*g2;
                /* For a full-block sine, tone_power ~= energy*N/2, so this
                   ratio approaches 1.0 for a pure tone. */
                float denom = s->phase34_tone_a_energy*(float) tone_a_block*0.5f;
                bool tonal = false;

                /* Energy floor: mean square > 100^2 keeps line noise and the
                   Jd-wait silence from ever qualifying. */
                /* V.90 8.2.3.1 has the analogue modem send Tone A with an
                   1800 Hz guard tone, nominally 6 dB down but measured on the
                   RasFinder at about the same level as the tone, so the
                   2400 Hz bin alone holds only ~45% of the block and a
                   retrain went unanswered for 16 s
                   (artifacts/rasfinder-info0a2-201220Z).  Count the guard
                   bin with it, and still demand a real 2400 Hz line: a
                   primary-channel S or data signal on its ~1829 Hz carrier
                   leaks into the 1800 Hz bin but has nothing at 2400. */
                float guard_power = 0.0f;

                /* Plain V.34 too: V.34 11.2.1.2 defines no guard tone, but
                   the RasFinder's MT5634SMI sends one with its Tone A retrain
                   in plain V.34 as well (centroid 2057 Hz, RMS bandwidth
                   302 Hz on rf-tower-v34-1-12840), so the 2400 Hz bin alone
                   never reached 70% and the retrain went unanswered. */
                if (listen_tone_a)
                {
                    float h1 = s->phase34_guard_g1;
                    float h2 = s->phase34_guard_g2;

                    guard_power = h1*h1 + h2*h2 - 0.3128689301f*h1*h2;
                }
                /*endif*/
                if (s->phase34_tone_a_energy > 10000.0f*(float) tone_a_block
                    &&  denom > 0.0f
                    &&  (tone_power > 0.70f*denom
                         ||  (guard_power > 0.0f
                              &&  tone_power > 0.25f*denom
                              &&  tone_power + guard_power > 0.70f*denom)))
                {
                    tonal = true;
                }
                /*endif*/
                if (tonal)
                    s->phase34_tone_a_blocks++;
                else
                    s->phase34_tone_a_blocks = 0;
                /*endif*/
                {
                    static int tdebug = -1;

                    if (tdebug < 0)
                        tdebug = (V34_DIAG_GETENV("V34_RETRAIN_TONE_DEBUG") != NULL);
                    /*endif*/
                    if (tdebug)
                    {
                        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                                 "Rx - retrain tone watch: stage=%s dur=%d "
                                 "meansq=%.0f ratio=%.3f tonal=%d run=%d\n",
                                 v34_rx_stage_to_str(s->stage),
                                 s->duration,
                                 (double) (s->phase34_tone_a_energy/(float) tone_a_block),
                                 (denom > 0.0f) ? (double) (tone_power/denom) : -1.0,
                                 tonal ? 1 : 0,
                                 s->phase34_tone_a_blocks);
                    }
                    /*endif*/
                }
                s->phase34_tone_a_g1 = 0.0f;
                s->phase34_tone_a_g2 = 0.0f;
                s->phase34_guard_g1 = 0.0f;
                s->phase34_guard_g2 = 0.0f;
                s->phase34_tone_a_energy = 0.0f;
                s->phase34_tone_a_samples = 0;
                /* 4 blocks = 80 ms, satisfying the "more than 50 ms" of
                   9.5.1.2 with margin against a chance tonal block. */
                if (s->phase34_tone_a_blocks >= 4  &&  !s->phase34_tone_a_reported)
                {
                    s->phase34_tone_a_reported = true;
                    s->received_event = V34_EVENT_PEER_RETRAIN;
                    V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                             "Rx - Tone %c detected in stage %s (%d ms); peer initiated "
                             "a retrain, reporting peer retrain per V.90 9.5.1.2 / "
                             "V.34 11.5.1.2\n",
                             listen_tone_a ? 'A' : 'B',
                             v34_rx_stage_to_str(s->stage),
                             s->phase34_tone_a_blocks*tone_a_block/8);
                }
                /*endif*/
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        s->phase34_tone_a_g1 = 0.0f;
        s->phase34_tone_a_g2 = 0.0f;
        s->phase34_guard_g1 = 0.0f;
        s->phase34_guard_g2 = 0.0f;
        s->phase34_tone_a_energy = 0.0f;
        s->phase34_tone_a_samples = 0;
        s->phase34_tone_a_blocks = 0;
        s->phase34_tone_a_reported = false;
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int primary_channel_rx(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;
    int step;
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t z;
    complexi16_t zz;
    complexi16_t sample;
#else
    complexf_t z;
    complexf_t zz;
    complexf_t sample;
#endif
    float ii;
    float qq;
    float v;
    int32_t power;
    /* Samples per T/2 interval at 8 kHz, in units of 1/192 of a sample (192
       being V34_RX_PULSESHAPER_COEFF_SETS, the polyphase resolution), as an
       exact rational num/den.  V.34 5.1 puts symbol rate S = 2400*a/c, so
       samples per symbol is 10c/3a and per T/2 interval 5c/3a:

         2400 (1/1)  5/3    2743 (8/7)  35/24   2800 (7/6)  10/7
         3000 (5/4)  4/3    3200 (4/3)  5/4     3429 (10/7) 7/6

       Five of the six are a whole number of coefficient sets.  2800 is not
       (192*10/7 = 274.29), and this table previously wrote 189 in place of
       192 for that row alone to force an integer.  That is a 189/192 =
       1.56% symbol-rate error, and per the project's own rule a sample-rate
       error of a few ppm already destroys the constellation: 2800 baud never
       demodulated.  Carry the remainder in shaper_t2_acc instead, so the
       long-run rate is exact and the residual jitter is under 1/192 of a
       sample. */
    static const int t2_num[6] = {192*5, 192*35, 192*10, 192*4, 192*5, 192*7};
    static const int t2_den[6] = {    3,     24,      7,     3,     4,     6};

    /* 9.5.1.2 has no phase qualifier, so watch for the peer's Tone A
     * in every stage this receiver runs in, DATA included. */
    v34_rx_watch_peer_retrain(s, amp, len);
    v34_rx_watch_phase3_carrier(s, amp, len);
    /* 9.6 puts a rate renegotiation "at any time during data mode", and the
     * peer opens one with S.  Same reason for being here rather than below
     * the T/3 branch: that branch returns. */
    v34_rx_watch_peer_reneg_s(s, amp, len);
    /* 10.1.3.7's S, found on the line rather than on the constellation, for a
     * bearer whose symbol clock is not phase-locked to our sample grid. */
    v34_rx_watch_phase3_s(s, amp, len);
    v34_rx_watch_v90_jd_s(s, amp, len);

    /* V90_RENEG_SYM_DUMP's last column: the RMS of the block of line samples
       this call was handed, kept here so the dump can say whether a collapse
       inside the receiver was preceded by one at its own input.  A recording
       is written before any of this runs, so comparing the dump against the
       file only proves the file; comparing it against THIS proves the feed. */
    {
        double sum = 0.0;
        int k;

        for (k = 0;  k < len;  k++)
            sum += (double) amp[k]*amp[k];
        /*endfor*/
        v90_reneg_feed_rms = (len > 0) ? sqrt(sum/len) : 0.0;
    }

    /* V.90 9.6: re-acquire the CP conditioning after a dead stretch of line.
     *
     * Measured on artifacts/reneg-ab-225015Z/reneg-r1, which is the failing
     * renegotiation this work reproduces offline.  Its RTP trace carries
     * exactly one loss in 32868 packets -- three packets, 480 samples -- and
     * it lands on the frame where 9.6.1.2.3's terminating CP-prime is due.
     * Concealed as digital silence it costs 192 symbols, and the receiver
     * never comes back: the absolute constellation reads 0.9 degrees from the
     * 4-point family just before the gap and 14 degrees just after, settling
     * at 22.5 -- the mean for symbols distributed uniformly in angle -- for
     * the remaining 14000 symbols of the window.  Nothing in the front end
     * explains that: the AGC scaling never moves (its adaptation is inhibited
     * on the silence by its own power gate), the carrier loop's frequency
     * never moves (its error term is zero on zero input), the Godard timing
     * loop's total correction never moves, and freezing the taps
     * changes neither the angle nor the frame
     * count.  What is missing is a way BACK: the CP conditioning's
     * acquisition -- a fresh equalizer trained by Figure 8's SCR -- is
     * one-shot, so once it has converged there is nothing left that can
     * re-acquire, while the peer is still repeating CP and still supplying
     * material to acquire on.
     *
     * So treat a block of digital silence the way the seam itself is treated:
     * on the first live block after one, re-arm exactly what
     * v34_v90_force_reneg_cp_rx() arms.  Bounded per renegotiation, because a
     * line that is dead repeatedly is not one this can rescue.
     * It is OFF, on the measurement in docs/retrain_and_resync.md. */
    if (s->stage == V34_RX_STAGE_V90_CP  &&  len > 0)
    {
        if (v90_reneg_feed_rms == 0.0)
        {
            s->reneg_cp_silent_blocks++;
        }
        else if (s->reneg_cp_silent_blocks > 0)
        {
            int blocks = s->reneg_cp_silent_blocks;

            s->reneg_cp_silent_blocks = 0;
            /* Only after the acquisition has already SUCCEEDED, and only for
               a stretch long enough to matter.  Re-arming while it is still
               converging destroys the very convergence it is meant to
               restore: fired on every silent block, this window never
               converged at all (23.2 degrees from the 4-point family
               throughout, 0 valid frames, against 1.5 degrees and 4 frames
               with it left alone).  The test is reneg_cp_settled and NOT
               reneg_cp_train, which is not cleared on every path. */
            if (v90_reneg_cp_reacquire_enabled()
                &&
                s->reneg_cp_settled
                &&
                blocks >= V90_RENEG_CP_REACQUIRE_BLOCKS
                &&
                s->reneg_cp_reacquires < V34_V90_RENEG_CP_MAX_REACQUIRES)
            {
                s->reneg_cp_reacquires++;
                equalizer_reset(s);
                s->reneg_cp_train = 1;
                s->reneg_cma_mag = 0.0f;
                s->reneg_cma_bauds = 0;
                equalizer_save(s);
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - V.90 9.6: %d block(s) of dead line in the CP "
                         "window; re-acquiring on the peer's repeated CP "
                         "(re-acquisition %d)\n",
                         blocks, s->reneg_cp_reacquires);
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endif*/

    /* This branch is internal DSP only. v34_rx() still consumes exactly len
       8 kHz line samples; no sample is inserted into or removed from RTP. */
    if (s->v90_t3_active)
    {
        if (s->stage == V34_RX_STAGE_DATA)
        {
            s->v90_t3_capture_only = false;
            return v90_t3_primary_rx(s, amp, len);
        }
        /*endif*/
        /* Armed but not yet handed over: keep filling the ring so the B1
           search can reach back across the E boundary, and let the ordinary
           receiver carry on demodulating CP from the same samples. */
        s->v90_t3_capture_only = true;
        (void) v90_t3_primary_rx(s, amp, len);
        s->v90_t3_capture_only = false;
    }
    /*endif*/

    /* Use the negotiated baud rate and carrier assignment.
       baud_rate is set from INFO1a (process_rx_info1a) or v34_rx_restart.
       high_carrier is set from v34_rx_restart based on calling_party flag. */
    if (s->baud_rate < 0 || s->baud_rate > 5)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - ERROR: baud_rate=%d out of range (expected 0-5), forcing to 4\n",
                 s->baud_rate);
        s->baud_rate = 4;  /* V34_BAUD_RATE_3200 */
    }
    s->shaper_re = v34_rx_shapers_re[s->baud_rate][s->high_carrier];
    s->shaper_im = v34_rx_shapers_im[s->baud_rate][s->high_carrier];
    if (s->shaper_t2_num != t2_num[s->baud_rate]
        ||  s->shaper_t2_den != t2_den[s->baud_rate])
    {
        s->shaper_t2_num = t2_num[s->baud_rate];
        s->shaper_t2_den = t2_den[s->baud_rate];
        s->shaper_t2_acc = 0;
    }
    /*endif*/
    s->shaper_sets = (2*s->shaper_t2_num + s->shaper_t2_den/2)/s->shaper_t2_den;
    /* Periodic diagnostic: log primary channel RX config on first entry and every 8000 samples */
    /* Gated at PHASE3_TRAINING this printed nothing for a receiver sitting in
       PHASE3_WAIT_S, which is exactly where a wrong symbol rate or carrier
       stops S being found. */
    if (s->stage >= V34_RX_STAGE_PHASE3_WAIT_S && (s->sample_time % 8000) < (unsigned)len)
    {
        V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                 "Rx - primary_channel_rx: baud_rate=%d high_carrier=%d carrier=%.1fHz agc=%.5f power=%ld shaper_sets=%d\n",
                 s->baud_rate, s->high_carrier, dds_frequencyf(s->v34_carrier_phase_rate),
                 (double)s->agc_scaling, (long)power_meter_current(&s->power), s->shaper_sets);
    }
    /* Follow the far end when it abandons Phase 3/4 and restarts.
     *
     * A V.90 analogue modem that gives up (constellation design failure, an
     * unanswered DIL request, a Phase 3 timeout) goes SILENCERETRAIN ->
     * TONE_AB -> Phase 1.  Measured on the d-modem rig, that silence is about
     * 80 ms.  Without noticing it we keep transmitting Phase 3/4 at a peer
     * that is no longer listening, and our own receiver stays parked in a
     * Phase 3/4 stage decoding the peer's Phase 1/2 tones as though they were
     * Phase 4 frames -- which is what the long MP-CRC investigation was
     * actually chasing (see rig/README.md).  Report it so the application can
     * follow the peer back to Phase 2 and get another attempt, rather than
     * hanging until the call dies.
     *
     * Deliberately keyed on sustained near-silence rather than on tone
     * detection: the tone detectors live in info_rx(), which is not the
     * demodulator running during Phase 3/4, and silence is the unambiguous
     * first half of every retrain the peer can start. */
    /* Do not run this detector in PHASE3_TRAINING.  V.90 §9.3.2.2 requires
       the analogue modem to begin Phase 3 with 70 +/- 5 ms of silence, so the
       old 60 ms threshold deterministically classified the legitimate startup
       gap as SILENCERETRAIN.  Once the digital transmitter reaches Jd the
       dedicated expect-silence/energy detector below owns abandonment; this
       detector remains useful in the later Phase 4 stages.

       Likewise, do not arm it during the initial CPt acquisition window in
       V90_CP.  The project-owned V.90 transmitter hands the receiver
       directly from DIL to CPt acquisition, and the analogue modem may send
       CPt, SCR, or silence while it searches for a usable upstream path.
       V.90 gives this exchange a 15 s plus round-trip-delay budget.  A short
       silence inside that interval must not stop our unbarred Ri and restart
       Phase 2.  The independent Tone A detector below remains armed for an
       explicit peer retrain request. */
    int phase4_cp_wait_bauds = 0;
    if (s->baud_rate >= 0 && s->baud_rate < 6)
        phase4_cp_wait_bauds = PHASE4_CP_ACQUISITION_WAIT_SECONDS
                               * baud_rate_parameters[s->baud_rate].baud_rate;

    /* 12.6.3's primary transition includes 70 ms of silence. Clause 12
     * recovery is handled by hdx_recovery_tone(), not this clause 11/9
     * silence heuristic (which otherwise restarts the production engine). */
    if (s->duplex && v34_rx_stage_is_primary_training(s->stage)
        && s->stage > V34_RX_STAGE_PHASE3_TRAINING
        && !(s->stage == V34_RX_STAGE_V90_CP
             && s->duration < phase4_cp_wait_bauds))
    {
        int32_t retrain_floor = (s->info_rx_carrier_ref > 0)
                                ? s->info_rx_carrier_ref/64
                                : 0;

        for (i = 0;  i < len;  i++)
        {
            int32_t mag = (int32_t) amp[i]*amp[i];

            if (retrain_floor > 0  &&  mag < retrain_floor)
            {
                s->phase34_silence_samples++;
            }
            else
            {
                s->phase34_silence_samples = 0;
                /* Energy is back.  Re-arm so a later retrain is reported too. */
                s->phase34_retrain_reported = false;
            }
            /*endif*/
            /* 480 samples = 60 ms, comfortably inside the ~80 ms the peer
               spends in SILENCERETRAIN but far longer than any gap inside a
               live Phase 3/4 signal. */
            if (s->phase34_silence_samples >= 480  &&  !s->phase34_retrain_reported)
            {
                s->phase34_retrain_reported = true;
                s->received_event = V34_EVENT_PEER_RETRAIN;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - far end abandoned Phase 3/4 (%d ms silence in stage %s); "
                         "reporting peer retrain\n",
                         s->phase34_silence_samples/8, v34_rx_stage_to_str(s->stage));
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        s->phase34_silence_samples = 0;
        s->phase34_retrain_reported = false;
    }
    /*endif*/
    /* The mirror image of the check above, for the Jd wait.
     *
     * PHASE3_WAIT_S sits *below* PHASE3_TRAINING in the stage enum, so it is
     * deliberately outside the silence check -- and rightly so: §9.3.2.4 makes
     * the analogue modem silent for this whole window, so silence here is the
     * normal case and carries no information.  What does carry information is
     * energy: until §9.3.2.7 starts S the peer must not be transmitting at all,
     * so sustained energy that never resolves into S means it has abandoned
     * V.90.  Verified on the d-modem rig: its V90AutoDigitalImpDetector raises
     * "drop to V34 requested", goes JaTXMIT -> SILENCERETRAIN, and comes back
     * with Phase 1 tones while we still had ~2 s of Jd budget left -- which we
     * spent transmitting Phase 3 over its fresh Phase 1, corrupting exactly the
     * bulk-delay estimate its next attempt depends on. */
    if (s->stage == V34_RX_STAGE_PHASE3_WAIT_S
        &&
        s->phase3_expect_silence
        &&
        !s->phase3_s_present)
    {
        int32_t energy_floor = (s->info_rx_carrier_ref > 0)
                               ? s->info_rx_carrier_ref/64
                               : 0;

        for (i = 0;  i < len;  i++)
        {
            int32_t mag = (int32_t) amp[i]*amp[i];

            if (energy_floor > 0  &&  mag >= energy_floor)
                s->phase3_energy_samples++;
            else
                s->phase3_energy_samples = 0;
            /*endif*/
            /* 1600 samples = 200 ms.  Long enough that a burst of line noise
               or the leading edge of a real S cannot trip it (S is confirmed
               well inside that, and phase3_s_present gates this off), short
               enough to stop us long before the Jd budget runs out. */
            if (s->phase3_energy_samples >= 1600  &&  !s->phase3_energy_retrain_reported)
            {
                s->phase3_energy_retrain_reported = true;
                s->received_event = V34_EVENT_PEER_RETRAIN;
                V34_RX_LOG(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: %d ms of energy during the Jd wait with no S; "
                         "far end has left V.90, reporting peer retrain\n",
                         s->phase3_energy_samples/8);
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        s->phase3_energy_samples = 0;
        s->phase3_energy_retrain_reported = false;
    }
    /*endif*/
    /* Dump raw Phase 3 RX audio for offline analysis */
    if (phase3_rx_dump_fp && phase3_rx_dump_count < PHASE3_RX_DUMP_SAMPLES)
    {
        int to_write = len;
        if (phase3_rx_dump_count + to_write > PHASE3_RX_DUMP_SAMPLES)
            to_write = PHASE3_RX_DUMP_SAMPLES - phase3_rx_dump_count;
        fwrite(amp, sizeof(int16_t), to_write, phase3_rx_dump_fp);
        phase3_rx_dump_count += to_write;
        if (phase3_rx_dump_count >= PHASE3_RX_DUMP_SAMPLES)
        {
            fclose(phase3_rx_dump_fp);
            phase3_rx_dump_fp = NULL;
            fprintf(stderr, "[V34 RX] Phase 3 RX audio dump complete (%d samples)\n", phase3_rx_dump_count);
        }
    }
    for (i = 0;  i < len;  i++)
    {
        s->qam_sample_time++;
        s->rrc_filter[s->rrc_filter_step] = amp[i];
        if (++s->rrc_filter_step >= V34_RX_FILTER_STEPS)
            s->rrc_filter_step = 0;
        /*endif*/

        power = power_meter_update(&s->power, amp[i]);
        s->eq_put_step -= V34_RX_PULSESHAPER_COEFF_SETS;
        step = -s->eq_put_step;
        if (step > V34_RX_PULSESHAPER_COEFF_SETS - 1)
            step = V34_RX_PULSESHAPER_COEFF_SETS - 1;
        /*endif*/
        while (step < 0)
            step += V34_RX_PULSESHAPER_COEFF_SETS;
        /*endwhile*/
#if defined(SPANDSP_USE_FIXED_POINT)
        ii = vec_circular_dot_prodi16(s->rrc_filter, (*s->shaper_re)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
        ii = vec_circular_dot_prodf(s->rrc_filter, (*s->shaper_re)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
#if defined(SPANDSP_USE_FIXED_POINT)
        //sample.re = (ii*(int32_t) s->agc_scaling) >> 15;
        sample.re = ii*s->agc_scaling;
#else
        sample.re = ii*s->agc_scaling;
#endif
        /* Symbol timing synchronisation band edge filters */
        /* Low Nyquist band edge filter */
        v = s->pri_ted.symbol_sync_low[0]*s->pri_ted.low_band_edge_coeff[0] + s->pri_ted.symbol_sync_low[1]*s->pri_ted.low_band_edge_coeff[1] + sample.re;
        s->pri_ted.symbol_sync_low[1] = s->pri_ted.symbol_sync_low[0];
        s->pri_ted.symbol_sync_low[0] = v;
        /* High Nyquist band edge filter */
        v = s->pri_ted.symbol_sync_high[0]*s->pri_ted.high_band_edge_coeff[0] + s->pri_ted.symbol_sync_high[1]*s->pri_ted.high_band_edge_coeff[1] + sample.re;
        s->pri_ted.symbol_sync_high[1] = s->pri_ted.symbol_sync_high[0];
        s->pri_ted.symbol_sync_high[0] = v;

        /* Put things into the equalization buffer at T/2 rate. The symbol synchcronisation
           will fiddle the step to align this with the symbols. */
        if (s->eq_put_step <= 0)
        {
            /* AGC: adapt scaling until locked down.
               This normalizes equalizer input from the received power, so the
               constant-modulus PP symbols and CMA operate at the expected
               magnitude whatever level the peer transmitted at. */
            if (s->agc_scaling_save == 0.0f  &&  power > 10)
            {
                if (power > V34_AGC_POWER_MIN)
                {
                    float new_scaling;

                    new_scaling = 2.17f / sqrtf((float) power);
                    if (!isfinite(new_scaling) || new_scaling < V34_AGC_SCALING_MIN)
                        new_scaling = V34_AGC_SCALING_MIN;
                    else if (new_scaling > V34_AGC_SCALING_MAX)
                        new_scaling = V34_AGC_SCALING_MAX;
                    /*endif*/
                    s->agc_scaling = new_scaling;
                }
                /*endif*/
            }
            /*endif*/
            /* Exact rational T/2 advance; see t2_num/t2_den above. */
            s->shaper_t2_acc += s->shaper_t2_num;
            s->eq_put_step += s->shaper_t2_acc/s->shaper_t2_den;
            s->shaper_t2_acc %= s->shaper_t2_den;
#if defined(SPANDSP_USE_FIXED_POINT)
            qq = vec_circular_dot_prodi16(s->rrc_filter, (*s->shaper_im)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            qq = vec_circular_dot_prodf(s->rrc_filter, (*s->shaper_im)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
#if defined(SPANDSP_USE_FIXED_POINT)
            //sample.im = (qq*(int32_t) s->agc_scaling) >> 15;
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexi16(s->carrier_phase);
#else
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexf(s->carrier_phase);
#endif
            zz.re = sample.re*z.re - sample.im*z.im;
            zz.im = -sample.re*z.im - sample.im*z.re;
            process_primary_half_baud(s, &zz);
        }
        /*endif*/
#if defined(SPANDSP_USE_FIXED_POINT)
        dds_advance(&s->carrier_phase, s->v34_carrier_phase_rate);
#else
        dds_advancef(&s->carrier_phase, s->v34_carrier_phase_rate);
#endif
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* Figure 10/9.6.3.2: eliminate the unknown 16-state encoder contents.
   Y0[n] XOR Y0[n-3] XOR Y0[n-4] equals
   Y2[n-3] XOR Y2[n-2] XOR Y1[n-1]. With C0=0, substituting U0
   leaves the same three-term parity check of Table 12's V0 sequence.
   Use a complete superframe and accept only a unique zero-syndrome phase;
   noise or ambiguous evidence leaves the existing decoder in force. */
static int v34_v0_at(const v34_rx_state_t *s, int pair)
{
    int span = 4*s->parms.p*s->parms.j;
    pair = ((pair % span) + span) % span;
    return pair % (2*s->parms.p) == 0
         ? (0x5FEE >> (pair/(2*s->parms.p))) & 1 : 0;
}

static void v34_rx_acquire_v0(v34_rx_state_t *s)
{
    int nearest[2] = {0, 0};
    int span = 4*s->parms.p*s->parms.j;
    int n = s->v0_pairs;
    int label[2];
    int syndrome;
    int winner = -1;
    int winners = 0;

    if (!s->v0_acquiring || span <= 0 || span > 512)
        return;
    for (int i = 0; i < 2; i++)
    {
        for (int k = 1; k < 4; k++)
            if (s->viterbi.error[i][k] < s->viterbi.error[i][nearest[i]])
                nearest[i] = k;
        label[i] = get_binary_subset_label_q9_7(&s->xy[i][nearest[i]]) & 7;
    }
    s->v0_u[n%5] = (label[1] - label[0]) & 1;
    s->v0_input[n%5] = conv_encode_input[label[0]][label[1]];
    if (n >= 4)
    {
        syndrome = s->v0_u[n%5] ^ s->v0_u[(n-3)%5] ^ s->v0_u[(n-4)%5]
                 ^ ((s->v0_input[(n-3)%5] >> 1) & 1)
                 ^ ((s->v0_input[(n-2)%5] >> 1) & 1)
                 ^ (s->v0_input[(n-1)%5] & 1);
        for (int delta = 0; delta < span; delta++)
        {
            int expected = v34_v0_at(s, s->input_4d + delta)
                         ^ v34_v0_at(s, s->input_4d + delta - 3)
                         ^ v34_v0_at(s, s->input_4d + delta - 4);
            s->v0_score[delta] += syndrome != expected;
            if (s->v0_score[delta] == 0)
            {
                winner = delta;
                winners++;
            }
        }
    }
    if (++s->v0_pairs >= span && winners == 1)
    {
        s->input_4d = (s->input_4d + winner) % span;
        s->viterbi.zero_precoder = true;
        s->v0_acquiring = false;
        V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                   "Rx - V0 phase acquired: advance %d 4D pairs, %d observed pairs, "
                   "unique zero syndrome (9.6.3/Table 12)\n", winner, s->v0_pairs);
    }
    else if (s->v0_pairs >= 2*span)
    {
        s->v0_pairs = 0;
        memset(s->v0_score, 0, sizeof(s->v0_score));
    }
}

/* Keep this global until the modem is VERY well tested */
SPAN_DECLARE(void) v34_put_mapping_frame(v34_rx_state_t *s, int16_t bits[16])
{
    int i;
    int j;
    int constel;
    int invert;
    complexi16_t c;
    complexi16_t p;
    complexi16_t u;
    complexi16_t v;
    complexi16_t y[2];

    /* Put the four 4D symbols (eight 2D symbols) of a mapping frame */
//#define BYPASS_VITERBI
    for (i = 0;  i < 8;  i++)
    {
        s->xt[0].re = bits[2*i];
        s->xt[0].im = bits[2*i + 1];
//printf("AMZ %p [%6d, %6d] [%8.3f, %8.3f]\n", s, s->xt[0].re, s->xt[0].im, FP_Q9_7_TO_F(s->xt[0].re), FP_Q9_7_TO_F(s->xt[0].im));
        s->yt = prediction_error_filter(s);
        v34_rx_quantize_n_ways(s->xy[i & 1], &s->yt);
//printf("CCC %p [%8.3f, %8.3f] [%8.3f, %8.3f] [%8.3f, %8.3f] [%8.3f, %8.3f]\n",
//       s,
//       FP_Q9_7_TO_F(s->xy[i & 1][0].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][0].im),
//       FP_Q9_7_TO_F(s->xy[i & 1][1].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][1].im),
//       FP_Q9_7_TO_F(s->xy[i & 1][2].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][2].im),
//       FP_Q9_7_TO_F(s->xy[i & 1][3].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][3].im));
        viterbi_calculate_candidate_errors(s->viterbi.error[i & 1], s->xy[i & 1], &s->yt);
#if defined(BYPASS_VITERBI)
        y[i & 1].re = s->xt[0].re;
        y[i & 1].im = s->xt[0].im;
//printf("CCD %p [%8.3f, %8.3f]\n", s, FP_Q9_7_TO_F(y[i & 1].re), FP_Q9_7_TO_F(y[i & 1].im));
#endif
        if ((i & 1))
        {
            v34_rx_acquire_v0(s);
            /* Deal with super-frame sync inversion at the time the 4D pair
               enters the Viterbi decoder.  step_2d is the delayed output
               position and remains zero during windup, so using it here
               repeatedly consumed V0 bits at the start of B1. */
            if (s->parms.p > 0 && s->parms.j > 0
                && s->input_4d % (2*s->parms.p) == 0) {
                int pattern = (s->input_4d / (2*s->parms.p))
                            % (2*s->parms.j);

                invert = (0x5FEE >> pattern) & 1;
            }
            else
                invert = false;
            /*endif*/
            viterbi_update_path_metrics(&s->viterbi, s->xy, invert);
            if (++s->input_4d >= 4*s->parms.p*s->parms.j)
                s->input_4d = 0;
//printf("EEE %p %4d %4d %4d %4d %4d %4d %4d %4d (%d)\n",
//       s,
//       s->viterbi.branch_error[0],
//       s->viterbi.branch_error[1],
//       s->viterbi.branch_error[2],
//       s->viterbi.branch_error[3],
//       s->viterbi.branch_error[4],
//       s->viterbi.branch_error[5],
//       s->viterbi.branch_error[6],
//       s->viterbi.branch_error[7],
//       s->viterbi.windup);
#if defined(BYPASS_VITERBI)
            {
#else
            if (s->viterbi.windup)
            {
                /* Wait for the Viterbi buffer to fill with symbols. */
                s->viterbi.windup--;
            }
            else
            {
                viterbi_trace_back(&s->viterbi, y);
                /* The public QAM report callback normally observes the
                   equalizer input.  A non-NULL target reports the delayed
                   lattice points selected by the Viterbi traceback as well.
                   Offline receivers can use these decisions for a bounded
                   decision-directed refinement without duplicating the
                   trellis implementation. */
                if (s->qam_report)
                {
                    complexf_t decided;

                    for (j = 0;  j < 2;  j++)
                    {
                        decided.re = FP_Q9_7_TO_F(y[j].re);
                        decided.im = FP_Q9_7_TO_F(y[j].im);
                        s->qam_report(s->qam_user_data,
                                      NULL,
                                      &decided,
                                      s->qam_sample_time);
                    }
                    /*endfor*/
                }
                /*endif*/
#endif
                /* We now have two points in y to be decoded. They are in Q9.7 format. */
//printf("AAA %p [%8.3f, %8.3f] [%8.3f, %8.3f]\n",
//       s,
//       FP_Q9_7_TO_F(y[0].re),
//       FP_Q9_7_TO_F(y[0].im),
//       FP_Q9_7_TO_F(y[1].re),
//       FP_Q9_7_TO_F(y[1].im));
                for (j = 0;  j < 2;  j++)
                {
                    p = precoder_rx_filter(s);

                    c = quantize_rx(s, &p);
                    s->x[0].re = y[j].re - p.re;
                    s->x[0].im = y[j].im - p.im;
                    u.re = (y[j].re >> 7) - c.re;
                    u.im = (y[j].im >> 7) - c.im;

                    s->ww[j + 1] = get_binary_subset_label(&u);
                    v = rotate90_counterclockwise(&u, s->ww[j + 1]);
                    constel = get_inverse_constellation_point(&v);
//printf("AMQ %p %d [%d, %d] [%d, %d] %d\n", s, constel, v.re, v.im, u.re, u.im, s->ww[j + 1]);
//printf("AMQ %p [%6d, %6d] (%d) [%6d, %6d] [%8.3f, %8.3f]\n", s, v.re, v.im, s->ww[j + 1], u.re, u.im, FP_Q9_7_TO_F(y[j].re), FP_Q9_7_TO_F(y[j].im));
                    s->qbits[s->step_2d + j] = constel & s->parms.q_mask;
                    s->mjk[s->step_2d + j] = constel >> s->parms.q;
                }
                /*endfor*/
                /* Compute the I bits */
                s->ibits[s->step_2d >> 1] = (((s->ww[1] - s->ww[0]) & 3) << 1)
                                          | (((s->ww[2] - s->ww[1]) >> 1) & 1);
                s->ww[0] = s->ww[1];
                s->step_2d += 2;
                if (s->step_2d == 8)
                {
                    shell_unmap(s);
                    v34_rx_pack_output_bitstream(s);
                    if (++s->data_frame >= s->parms.p)
                    {
                        s->data_frame = 0;
                        if (++s->super_frame >= s->parms.j)
                        {
                            s->super_frame = 0;
                            s->v0_pattern = 0;
                        }
                        /*endif*/
                        /* The V.90 upstream's superframe phase search steps
                           here, and only here: this is the decoder's own
                           frame boundary, so moving the phase costs nothing
                           else.  An earlier version reset the whole frame
                           state from the emitter instead, which also zeroed
                           the mapping-frame position mid-frame -- so each
                           step shifted alignment by part of a frame and the
                           search wandered (5, 2, 5, 2, 6) instead of
                           enumerating the seven phases. */
                        if (s->v90_t3_phase_pending)
                        {
                            /* Shift the labelling by a whole number of data
                               frames, relative to whatever the counters hold
                               right now.  Reduce modulo j*p at the point of
                               use: everything the decoder derives from the
                               pair -- the V0 inversion pattern and the 4D
                               symbol counter -- indexes state sized for one
                               superframe of p data frames, so an
                               out-of-range pair is not a wrong answer but a
                               wild pointer. */
                            int span = s->parms.j*s->parms.p;

                            if (span > 0)
                            {
                                int total = s->super_frame*s->parms.p
                                          + s->data_frame
                                          + s->v90_t3_phase_delta;

                                total = ((total % span) + span) % span;
                                s->super_frame = total/s->parms.p;
                                s->data_frame = total % s->parms.p;
                                s->v0_pattern = (uint16_t) (2*s->super_frame);
                                s->input_4d = s->super_frame*4*s->parms.p
                                            + s->data_frame*4;
                                s->v90_t3_phase_pos =
                                    (((s->v90_t3_phase_pos
                                       + s->v90_t3_phase_delta) % span)
                                     + span) % span;
                            }
                            /*endif*/
                            V34_RX_LOG(s->logging, SPAN_LOG_WARNING,
                                     "Rx - V.90 upstream frame phase shift %+d applied "
                                     "(now offset %d, super_frame %d data_frame %d)\n",
                                     s->v90_t3_phase_delta, s->v90_t3_phase_pos,
                                     s->super_frame, s->data_frame);
                            s->v90_t3_phase_delta = 0;
                            s->v90_t3_phase_pending = false;
                            /* Start the measurement where the candidate
                               starts.  The change is scheduled at the end of
                               a window and lands here, up to a superframe
                               later -- 40 ms at 3200 baud, half a search
                               window -- so without this every candidate is
                               scored partly on its predecessor, and on a
                               short window that is most of the evidence. */
                            s->v90_t3_ones = 0;
                            s->v90_t3_alt_ones = 0;
                            s->v90_t3_bit_count = 0;
                            for (int ph = 0;  ph < 10;  ph++)
                            {
                                s->v90_t3_v14_ok[ph] = 0;
                                s->v90_t3_v14_count[ph] = 0;
                            }
                            /*endfor*/
                        }
                        /*endif*/
                    }
                    /*endif*/
//printf("ZAQ data frame %d, super frame %d\n", s->data_frame, s->super_frame);
                    s->step_2d = 0;
                }
                /*endif*/
            }
            /*endif*/
            s->viterbi.ptr = (s->viterbi.ptr + 1) & 0xF;
        }
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_rx_fillin(v34_state_t *s, int len)
{
    int i;

    /* We want to sustain the current state (i.e carrier on<->carrier off), and
       try to sustain the carrier phase. We should probably push the filters, as well */
    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW, "Rx - Fill-in %d samples\n", len);
    for (i = 0;  i < len;  i++)
    {
#if defined(SPANDSP_USE_FIXED_POINT)
        dds_advance(&s->rx.carrier_phase, s->rx.v34_carrier_phase_rate);
#else
        dds_advancef(&s->rx.carrier_phase, s->rx.v34_carrier_phase_rate);
#endif
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

void v34_condition_rx_for_pph(v34_state_t *s, const char *why)
{
    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
             "Rx - CC: conditioned to detect PPh (%s)\n", why);
    s->rx.stage = V34_RX_STAGE_CC;
    s->rx.current_demodulator = V34_MODULATION_CC;
    /* A new control interval owns fresh 12.4/12.6 deadlines, even when
       the previous page ended in the same watchdog state. */
    s->tx.hdx_watch_state = -1;
    s->tx.hdx_watch_samples = 0;
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.pph_detected = false;
    s->rx.hdx_cc_resync = false;
    s->rx.hdx_sh_seen = false;
    s->rx.hdx_ac_seen = false;
    memset(s->rx.hdx_ac_previous, 0, sizeof(s->rx.hdx_ac_previous));
    memset(s->rx.hdx_ac_count, 0, sizeof(s->rx.hdx_ac_count));
    memset(s->rx.hdx_sh_count, 0, sizeof(s->rx.hdx_sh_count));
    memset(s->rx.hdx_sh_history, 0, sizeof(s->rx.hdx_sh_history));
    s->rx.pph_hunt_bauds = 0;
    s->rx.pph_bounded_acquisition = false;
    s->rx.pph_corr_energy = 0.0f;
    s->rx.pph_corr_weight = 0.0f;
    memset(s->rx.pph_corr, 0, sizeof(s->rx.pph_corr));
    memset(s->rx.pph_history, 0, sizeof(s->rx.pph_history));
    memset(s->rx.pph_history_count, 0, sizeof(s->rx.pph_history_count));
    s->rx.cc_level = 0.0f;
    s->rx.pph_best_phase[0] = s->rx.pph_best_phase[1] = -1;
    s->rx.pph_hold_steps[0] = s->rx.pph_hold_steps[1] = 0;
    s->rx.hdx_await_trn_end = false;
    s->rx.hdx_silence_bauds = 0;
    s->rx.cc_qam_started = false;
    s->rx.mp_seen = 0;
    s->rx.mp_bits_since_frame = 0;
    s->rx.mp_last_frame_bits = 0;
    s->rx.mp_count = -1;
    s->rx.mp_remote_ack_seen = 0;
    s->rx.mp_remote_ack_count = 0;
    s->rx.bitstream = 0;
    s->rx.bit_count = 0;
    s->rx.crc = 0xFFFF;
    s->rx.duration = 0;
    s->rx.baud_half = 0;
    memset(&s->rx.last_sample, 0, sizeof(s->rx.last_sample));
    /* The control channel demodulator runs its own band edge timing recovery
       over the 600 baud RRC, so give it a clean filter history rather than
       whatever the primary channel left in it. */
    s->rx.rrc_filter_step = 0;
    memset(s->rx.rrc_filter, 0, sizeof(s->rx.rrc_filter));
    s->rx.eq_put_step = 0;
    memset(s->rx.cc_ted.symbol_sync_low, 0, sizeof(s->rx.cc_ted.symbol_sync_low));
    memset(s->rx.cc_ted.symbol_sync_high, 0, sizeof(s->rx.cc_ted.symbol_sync_high));
    memset(s->rx.cc_ted.symbol_sync_dc_filter, 0, sizeof(s->rx.cc_ted.symbol_sync_dc_filter));
    s->rx.cc_ted.baud_phase = 0;
}
/*- End of function --------------------------------------------------------*/

/* 12.7.1.2/12.7.2.2: a sustained foreign Tone A/B requests a primary
   retrain, including while the source's primary receiver is silent.
   Use raw DS0 samples so the primary equalizer cannot disguise the tone. */
static int hdx_recovery_tone(v34_state_t *s, const int16_t amp[], int len)
{
    float coefficient = s->calling_party ? -0.61803399f : 1.17557050f;
    int i;
    int action = s->half_duplex_state == V34_HALF_DUPLEX_PRIMARY_CHANNEL ? 1 : 0;
    /* An initiating recipient can still hear old primary data until the
       source detects its tone and responds. Qualify the returned tone on
       raw samples before allowing the first Phase 2 reversal. */
    if (!action && s->tx.retrain_omit_info0
        && (s->tx.stage == V34_TX_STAGE_HDX_INITIAL_A
            || s->tx.stage == V34_TX_STAGE_HDX_FIRST_A))
        action = 3;
    if (!action && s->half_duplex_source == V34_HALF_DUPLEX_SOURCE
        && s->tx.hdx_initial_control_startup
        && s->rx.stage == V34_RX_STAGE_CC && !s->rx.pph_detected
        && !s->tx.hdx_cc_resync && !s->tx.hdx_retrain_initiator
        && !s->tx.hdx_retrain_responding
        && (s->tx.stage == V34_TX_STAGE_HDX_CC_SILENCE
            || s->tx.stage == V34_TX_STAGE_HDX_PPH
            || s->tx.stage == V34_TX_STAGE_HDX_SECOND_ALT))
        action = 2; /* 12.4.3.1, before initial peer PPh only */
    if (s->duplex || !action)
    {
        s->rx.hdx_retrain_samples = s->rx.hdx_retrain_tone_samples = 0;
        s->rx.hdx_retrain_g1 = s->rx.hdx_retrain_g2 = s->rx.hdx_retrain_energy = 0.0f;
        return 0;
    }
    for (i = 0; i < len; i++)
    {
        float x = amp[i];
        float v = x + coefficient*s->rx.hdx_retrain_g1 - s->rx.hdx_retrain_g2;
        s->rx.hdx_retrain_g2 = s->rx.hdx_retrain_g1;
        s->rx.hdx_retrain_g1 = v;
        s->rx.hdx_retrain_energy += x*x;
        if (++s->rx.hdx_retrain_samples == 80)
        {
            float power = s->rx.hdx_retrain_g1*s->rx.hdx_retrain_g1
                        + s->rx.hdx_retrain_g2*s->rx.hdx_retrain_g2
                        - coefficient*s->rx.hdx_retrain_g1*s->rx.hdx_retrain_g2;
            if (s->rx.hdx_retrain_energy > 80.0f*100.0f*100.0f
                && power > 0.85f*40.0f*s->rx.hdx_retrain_energy)
                s->rx.hdx_retrain_tone_samples += 80;
            else
                s->rx.hdx_retrain_tone_samples = 0;
            s->rx.hdx_retrain_samples = 0;
            s->rx.hdx_retrain_g1 = s->rx.hdx_retrain_g2 = 0.0f;
            s->rx.hdx_retrain_energy = 0.0f;
            if (s->rx.hdx_retrain_tone_samples > milliseconds_to_samples(50))
                return action;
        }
    }
    return 0;
}

SPAN_DECLARE(int) v34_rx(v34_state_t *s, const int16_t amp[], int len)
{
    int leny;
    int lenx;
    int hdx_retrain = hdx_recovery_tone(s, amp, len);

    v34_rx_log_state_change(&s->rx);
    leny = 0;
    lenx = -1;
    do
    {
        switch (s->rx.current_demodulator)
        {
        case V34_MODULATION_SILENCE:
            /* 12.5.1: the source receives no bearer data while the recipient
               is silent. Consume samples without regressing sample_time. */
            lenx = len - leny;
            break;
        case V34_MODULATION_V34:
            lenx = primary_channel_rx(&s->rx, &amp[leny], len - leny);
            break;
        case V34_MODULATION_CC:
            lenx = cc_rx(&s->rx, &amp[leny], len - leny);
            break;
        case V34_MODULATION_L1_L2:
            lenx = l1_l2_analysis(&s->rx, &amp[leny], len - leny);
            break;
        case V34_MODULATION_TONES:
            lenx = info_rx(&s->rx, &amp[leny], len - leny);
            break;
        }
        /*endswitch*/
        leny += lenx;
        /* Add step by step, so each segment is seen up to date */
        s->rx.sample_time += lenx;
    }
    while (lenx > 0  &&  leny < len);

    if (s->rx.received_event == V34_EVENT_TRAINING_FAILED
        && !s->rx.training_failed_reported)
    {
        V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
                 "Rx - signalling training failure to host\n");
        report_status_change(&s->rx, SIG_STATUS_TRAINING_FAILED);
        s->rx.training_failed_reported = true;
    }
    /*endif*/
    /* If there is any residue, this should be the end of operation of the modem,
       so we don't really need to add that residue to the sample time. */
    if (hdx_retrain == 1)
        v34_start_retrain(s);
    else if (hdx_retrain == 2)
        v34_hdx_startup_tone_recovery(s);
    else if (hdx_retrain == 3)
        s->rx.hdx_retrain_peer_tone = true;
    return leny;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_rx_set_signal_cutoff(v34_state_t *s, float cutoff)
{
    /* The 0.4 factor allows for the gain of the DC blocker */
    s->rx.carrier_on_power = (int32_t) (power_meter_level_dbm0(cutoff + 2.5f)*0.4f);
    s->rx.carrier_off_power = (int32_t) (power_meter_level_dbm0(cutoff - 2.5f)*0.4f);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_put_bit(v34_state_t *s, span_put_bit_func_t put_bit, void *user_data)
{
    s->rx.put_bit = put_bit;
    s->rx.put_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_put_aux_bit(v34_state_t *s, span_put_bit_func_t put_bit, void *user_data)
{
    s->rx.put_aux_bit = put_bit;
    s->rx.put_aux_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_put_phase4_bit(v34_state_t *s,
                                          span_put_bit_func_t put_bit,
                                          void *user_data)
{
    s->rx.put_phase4_bit = put_bit;
    s->rx.put_phase4_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

/* V.90 §9.6.1.1.1: arm/disarm the watch for the analogue modem's S answer to
   our own Rd.  Figure 8/V.90 puts S (128T), S-bar (16T) and an optional SCR
   of up to 2000 ms between our R-bar-d and the peer's CP, and the clause says
   to "condition its receiver to detect S, S-bar, and CP".

   The receiver stays in V34_RX_STAGE_DATA while this is armed, deliberately:
   the S detector above is spectral (three narrow bins of §10.1.3.7) and needs
   neither equalizer, carrier loop nor timing loop, whereas the constellation
   detector in V34_RX_STAGE_PHASE4_S declared S on ordinary data-mode symbols
   140 ms after being armed -- 170 ms before the peer's real S was on the wire
   -- and the CP search then ran straight through the transition it exists to
   find. */
SPAN_DECLARE(void) v34_v90_watch_reneg_s(v34_state_t *s, int on)
{
    if (!s)
        return;
    /*endif*/
    s->rx.reneg_s_watch = (on != 0);
    s->rx.reneg_s_reported = false;
    s->rx.reneg_s_blocks = 0;
    s->rx.reneg_s_samples = 0;
    s->rx.reneg_s_energy = 0.0f;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_force_v90_phase4_cp_rx(v34_state_t *s)
{
    if (!s || !s->rx.v90_mode || s->rx.calling_party || !s->rx.put_phase4_bit)
        return;

    /* A V.90 analogue modem begins repeated CPt immediately after DIL.  The
       ordinary V.34 Phase 4 receiver waits for J'/TRN before scanning for MP;
       applying that gate here consumes several complete CPt repetitions.  The
       external downstream implementation has already established the phase
       boundary, so retain the freshly conditioned primary-channel frontend
       and begin the CP preamble/hypothesis search immediately. */
    s->primary_channel_active = true;
    s->rx.current_demodulator = V34_MODULATION_V34;
    s->rx.stage = V34_RX_STAGE_V90_CP;
    s->rx.duration = 0;
    s->rx.received_event = V34_EVENT_NONE;
    /* The T/2 eye chooser's per-call flip cap is usually spent by the end of
       Phase 3 (PP deferral, the post-Ja release, DIL), which left the CP
       stage no way to correct a wrong phase.  Against the RasFinder a live
       call spent all four in Phase 3 -- one on a half-silent window, sums
       45.7 vs 43.5 -- entered V90_CP on the wrong phase, read its 11
       CRC-valid CPt frames as white (26 degrees) and was retrained out of
       Ri (rf-tower-shp-codec-5136; reproduced with v90_engine_replay
       --split, which feeds RX as the two 80-sample calls per tick pjmedia
       makes live).  CPt is constant-modulus DPSK sent until we answer it,
       so give the stage two flips of its own and a fresh window. */
    if (V34_EYE_MAX_FLIPS - s->rx.eye_flips < 2)
        s->rx.eye_flips = V34_EYE_MAX_FLIPS - 2;
    /*endif*/
    s->rx.eye_votes = 0;
    s->rx.eye_on_sum = 0.0f;
    s->rx.eye_off_sum = 0.0f;
    s->rx.eye_on_aerr = 0.0f;
    s->rx.eye_off_aerr = 0.0f;
    s->rx.eye_n = 0;
    s->rx.eye_prev_on = complex_setf(0.0f, 0.0f);
    s->rx.eye_prev_off = complex_setf(0.0f, 0.0f);
    s->rx.bitstream = 0;
    s->rx.bit_count = 0;
    s->rx.mp_seen = 0;
    s->rx.mp_bits_since_frame = 0;
    s->rx.mp_last_frame_bits = 0;
    s->rx.mp_remote_ack_seen = 0;
    s->rx.mp_remote_ack_count = 0;
    s->rx.mp_count = -1;
    s->rx.mp_frame_pos = 0;
    s->rx.mp_frame_target = 0;
    s->rx.mp_early_rejects = 0;
    s->rx.phase3_j_lock_hyp = -1;
    s->rx.phase4_trn_lock_hyp = -1;
    s->rx.phase4_trn_lock_score = -1;

    /* V.90 upstream uses the analogue-modem scrambler polynomial (tap 4 in
       SpanDSP's zero-based representation).  Retry rotation still explores
       alternate order/domain/tap choices after rejected CP hypotheses. */
    /* Startup conditioning: the taps arriving here were trained by Phase 3
       moments ago and must not be walked.  Only the renegotiation path below
       turns training back on. */
    s->rx.reneg_cp_train = 0;
    /* Streaming startup CP belonged HERE, in the startup conditioning.
       It was read in v34_begin_rx_data() instead -- the DATA receiver's
       start, which runs long after the CP stage -- so it never reached this
       path at all: the startup CP bits came out byte-identical with the knob
       on and off, and V90_RENEG_SYM_DUMP, which sits inside the streamed
       branch, produced no file.  The "MEASURED, AND THE ANSWER IS THAT IT IS
       NEEDED" note beside that read is therefore about what the flag does to
       the DATA stage and to 11.6, not about startup CP, which had never been
       measured either way.  Default stays 0 so this is a plumbing fix and not
       a behaviour change. */
    s->rx.v90_cp_stream = 0;
    s->rx.v90_cp_stream_reg = 0;
    s->rx.scrambler_tap = 4;
    s->rx.mp_phase4_default_scrambler_tap = 4;
    s->rx.mp_phase4_default_bit_order = 0;
    /* CP arrives differentially encoded, so pin the fixed differential
       inverse instead of inheriting the Phase 3 TRN/J hint: that hint was
       measured on TRN, which V.34 10.1.3.8 maps DIRECTLY, so it describes the
       absolute-domain rotation and is meaningless for a differential decode.
       Feeding it here confined the search to the wrong transform while the
       2-error preamble gate produced false locks faster than the true one. */
    s->rx.v90_cp_diff_hypothesis = MP_HYPOTHESIS_DIFF_INVERSE;
    s->rx.mp_phase4_default_domain = 0;
    s->rx.mp_phase4_reject_streak = 0;
    s->rx.mp_phase4_nolock_count = 0;
    s->rx.mp_phase4_alt_tap_active = 0;
    s->rx.mp_phase4_alt_order_active = 0;
    s->rx.mp_phase4_alt_domain_active = 0;
    s->rx.mp_phase4_retry_mode = 0;
    s->rx.mp_phase4_bit_order = 0;
    s->rx.mp_phase4_domain = 0;
    s->rx.mp_phase4_force_abs_active = 0;
    s->rx.mp_phase4_diff_collapse_streak = 0;
    s->rx.mp_phase4_diff_recover_streak = 0;
    /* Preserve the Phase-3 channel solution before CP's decision-aided
       acquisition starts.  CP is differentially decodable and may need the
       DA loop to establish absolute phase, but §9.4.2.2/V.90 assumes the
       channel is static through this seam: adapting the CMA equalizer to a
       hypothesis that has not yet passed CP CRC can destroy the multi-level
       data slicer's only valid equalizer.  Resetting it instead, as the
       renegotiation path does, was measured worse (see the step-size note
       after v90_startup_cp_adapt()). */
    equalizer_save(&s->rx);
    s->rx.phase4_da_active = 0;
    s->rx.phase4_da_seeded = 0;
    s->rx.phase4_da_derot = 0;
    v34_rx_mp_reset_hypothesis_search(&s->rx);
    v34_rx_mp_vote_reset(&s->rx);

    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
             "Rx - V.90 Phase 4: immediate CPt acquisition armed (tap=4, domain=diff, order=b0,b1)\n");
}
/*- End of function --------------------------------------------------------*/

/* V.90 §9.6.1.1.1 CP conditioning, for a RATE RENEGOTIATION rather than
   startup.

   The startup path above deliberately preserves the Phase-3 channel solution,
   because §9.4.2.2 assumes the channel is static across that seam and Phase 3
   has only just finished training it.  A renegotiation has no such seam: the
   equalizer it would preserve was last trained before data mode, tens of
   seconds earlier, and the ordinary receive chain has been idle throughout
   because the T/3 upstream receiver owns data mode.  Measured live against
   the SmartLink rig, the [EQ] trace during a renegotiation's CP stage reads
   mag=26.8 against target_mag=1.29 -- the preserved solution is ~20x out, and
   the peer's CP, which is on the wire and 700 bits long, decoded not once.

   Figure 8/V.90 provides for exactly this: between S-bar and CP the analogue
   modem sends an optional SCR for up to 2000 ms, and SCR is scrambled ones --
   constant modulus, i.e. training material.  So start from a clean equalizer
   and let the SCR train it, rather than carrying a stale one into CP. */
SPAN_DECLARE(void) v34_v90_force_reneg_cp_rx(v34_state_t *s)
{
    if (!s)
        return;
    /*endif*/
    /* Hand the receiver back to the ordinary V.34 primary channel.
       §9.6.1.1.1's S, S-bar, SCR and CP are the Phase-4 4-point signals the
       T/2 chain demodulates; the T/3 branch is the DATA-mode upstream
       receiver, fitted to the negotiated data constellation by a supervised
       least-squares solve over B1.  It does not stop at the seam on its own:
       v90_t3_put_sample() calls v90_t3_emit_ready() whenever it has acquired,
       and that calls process_primary_symbol() directly.  So every symbol the
       CP stage saw during a renegotiation came from the data receiver --
       instrumented on artifacts/reneg-eq/reneg-r1, all 42496 of them -- with
       |z| pinned at 26.8 against the 4-point slicer's unit circle whatever
       the T/2 equalizer did, which is why resetting that equalizer and
       unfreezing its CMA changed nothing at all.  The upstream re-acquires
       from the renegotiation's own B1 afterwards, which is what
       v34_v90_prepare_upstream_data() is for and what the engine already
       arms by clearing its upstream-armed flag here. */
    s->rx.v90_t3_active = false;
    s->rx.v90_t3_acquired = false;
    s->rx.v90_t3_capture_only = false;
    s->rx.v90_t3_prepared = false;

    v34_force_v90_phase4_cp_rx(s);
    equalizer_reset(&s->rx);
    /* A fresh equalizer is only half of it: V34_RX_STAGE_V90_CP freezes blind
       adaptation, so without this nothing trains what was just reset. */
    s->rx.reneg_cp_train = 1;
    s->rx.reneg_cma_mag = 0.0f;
    s->rx.reneg_cma_bauds = 0;
    s->rx.reneg_cp_silent_blocks = 0;
    s->rx.reneg_cp_reacquires = 0;
    s->rx.reneg_cp_settled = 0;
    s->rx.v90_cp_stream = 1;
    s->rx.v90_cp_stream_reg = 0;
    /* Save the FRESH taps: the periodic equalizer restore would otherwise put
       the stale ones back a few hundred milliseconds later. */
    equalizer_save(&s->rx);
    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
             "Rx - V.90 9.6: CP conditioning with a fresh equalizer; SCR trains it\n");
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_reject_v90_phase4_hypothesis(v34_state_t *s)
{
    const char *hold_env;

    if (!s || !s->rx.v90_mode || s->rx.calling_party
        || s->rx.stage != V34_RX_STAGE_V90_CP)
        return;
    hold_env = getenv("ME_V34_HOLD_MP_HYPOTHESIS");
    if (hold_env && atoi(hold_env) != 0 && s->rx.mp_hypothesis >= 0)
    {
        /* The V.90 CP framer is the owner of frame validity, so a failed CP
           CRC does not by itself prove that SpanDSP's symbol hypothesis is
           wrong. Preserve that hypothesis and let the next repeated CP
           preamble try again with the same coherent bit stream. */
        s->rx.mp_early_rejects = 0;
        s->rx.mp_count = 0;
        s->rx.mp_frame_pos = 0;
        s->rx.mp_frame_target = 0;
        V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90 Phase 4: strict CP reject; holding MP hypothesis=%d for repeated CP\n",
                 s->rx.mp_hypothesis);
        return;
    }
    /* A strict Table 14 CRC rejection invalidates more than the current phase
       seed.  Advance the existing domain/tap/bit-order retry state as well;
       merely clearing mp_hypothesis retries the same decode mode forever. */
    mp_unlock_after_reject(&s->rx, true);
    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
             "Rx - V.90 Phase 4: strict CP framer rejected hypothesis; resuming preamble search\n");
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_rx_event(v34_state_t *s)
{
    return s->rx.received_event;
}
/*- End of function --------------------------------------------------------*/

/* The role-independent form.  V.34 11.5 has a retrain response of its own,
   and PEER_RETRAIN is application-owned: left set it would suppress the
   ordinary handshake events for the rest of the call. */
/* Clear a reported peer rate renegotiation, and re-arm the detector so a
   later one is reported too. */
SPAN_DECLARE(void) v34_clear_peer_reneg_s_event(v34_state_t *s)
{
    if (!s)
        return;
    /*endif*/
    if (s->rx.received_event == V34_EVENT_PEER_RENEG_S)
        s->rx.received_event = V34_EVENT_NONE;
    /*endif*/
    s->rx.reneg_s_blocks = 0;
    s->rx.reneg_s_reported = false;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_clear_peer_retrain_event(v34_state_t *s)
{
    if (s  &&  s->rx.received_event == V34_EVENT_PEER_RETRAIN)
        s->rx.received_event = V34_EVENT_NONE;
    /*endif*/
    if (s)
    {
        s->rx.phase34_tone_a_blocks = 0;
        s->rx.phase34_tone_a_reported = false;
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_clear_peer_retrain_event(v34_state_t *s)
{
    if (s && s->rx.received_event == V34_EVENT_PEER_RETRAIN)
        s->rx.received_event = V34_EVENT_NONE;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_phase3_s_event_count(v34_state_t *s)
{
    return s ? s->rx.phase3_s_event_count : 0;
}

SPAN_DECLARE(void) v34_v90_set_phase3_expect_silence(v34_state_t *s, int expect)
{
    if (!s)
        return;
    if (s->rx.phase3_expect_silence == (bool) expect)
        return;
    /*endif*/
    s->rx.phase3_expect_silence = (bool) expect;
    s->rx.phase3_energy_samples = 0;
    s->rx.phase3_energy_retrain_reported = false;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(bool) v34_phase3_s_alternation_confirmed(v34_state_t *s)
{
    return s && s->rx.phase3_s_present
        && s->rx.phase3_s_alt_count >= PHASE3_S_ALTERNATING_MIN
        && s->rx.phase3_s_stable_windows >= PHASE3_S_STABLE_WINDOWS;
}

SPAN_DECLARE(bool) v34_v90_take_line_s_event(v34_state_t *s)
{
    bool r;

    if (!s)
        return false;
    /*endif*/
    r = s->rx.v90jd_line_event;
    s->rx.v90jd_line_event = false;
    return r;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_arm_jd_s_watch(v34_state_t *s, int on)
{
    if (!s)
        return;
    /*endif*/
    if (on  &&  !s->rx.v90jd_s_armed)
    {
        memset(s->rx.v90jd_g1, 0, sizeof(s->rx.v90jd_g1));
        memset(s->rx.v90jd_g2, 0, sizeof(s->rx.v90jd_g2));
        s->rx.v90jd_energy = 0.0f;
        s->rx.v90jd_samples = 0;
        s->rx.v90jd_blocks = 0;
    }
    /*endif*/
    s->rx.v90jd_s_armed = (on != 0);
    s->rx.v90jd_dil_mode = (on == 2);
    s->rx.v90jd_holdoff = 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_arm_phase3_s_detector(v34_state_t *s)
{
    if (!s)
        return;
    s->rx.v90_p3_eye_released = true;

    /* Ja has already been delivered to the external V.90 digital-side state
       machine.  Clear the shared V.34 event before v34_tx() can interpret it
       as its own far-end J and call phase4_wait_init().  Preserve the decoded
       Ja constellation choice, but start the S/S-bar detector with a clean
       32-baud window so the later analogue S transition is unambiguous. */
    /* Pin the TRN constellation from the TRN lock rather than trusting
     * phase3_j_trn16 as left by the canonical J matcher.
     *
     * That matcher sets trn16 from whichever 16-bit pattern it last matched,
     * and it gets it wrong: measured live against a peer whose own log says
     * TRNSEG4 (4-point), the S detector was being armed "16-point", and the
     * J look-ahead then pinned 4-point a moment later -- the two disagreeing
     * about the same signal. Arming the S detector for the wrong constellation
     * is why the analogue S was never seen, which left Jd to expire on its
     * interop timeout and dragged the rest of Phase 3 onto timers.
     *
     * phase3_trn_lock_score is scored through a 4-point mapping, so a high
     * score is positive evidence the far end really is 4-point -- far better
     * evidence than the pattern matcher's guess. */
    /* Any latched TRN lock is 4-point evidence: the latch only ever accepts a
     * hypothesis scoring >=70% through a 4-point mapping, so there is no need
     * for a second, higher threshold here (an earlier 80% cut simply missed
     * legitimate 75% locks). */
    if (s->rx.phase3_trn_lock_hyp >= 0)
    {
        if (s->rx.phase3_j_trn16)
        {
            V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
                     "Rx - V.90: TRN lock (hyp=%d %d%%, 4-point mapping) overrides "
                     "canonical-J trn16=1; arming S detector as 4-point\n",
                     s->rx.phase3_trn_lock_hyp, s->rx.phase3_trn_lock_score);
        }
        /*endif*/
        s->rx.phase3_j_trn16 = 0;
    }
    /*endif*/
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.stage = V34_RX_STAGE_PHASE3_WAIT_S;
    s->rx.duration = 0;
    s->rx.bit_count = 0;
    s->rx.s_detect_count = 0;
    s->rx.s_window = 0;
    s->rx.phase3_s_present = false;
    s->rx.phase3_s_alt_window = 0;
    s->rx.phase3_s_alt_count = 0;
    s->rx.phase3_s_stable_windows = 0;
    s->rx.phase3_s_dom_windows = 0;
    s->rx.phase3_s_dom_symbol = -1;
    s->rx.phase3_s_fired_symbol = -1;
    memset(s->rx.phase3_s_counts, 0, sizeof(s->rx.phase3_s_counts));
    memset(s->rx.phase3_s_ring, 0, sizeof(s->rx.phase3_s_ring));
    memset(s->rx.phase3_s_mag_ring, 0, sizeof(s->rx.phase3_s_mag_ring));
    /* Arm unconditionally: this call *is* the statement that Ja has been
       consumed and the analogue S is the next thing to expect.  It used to arm
       only as a side effect of phase3_j_trn16 >= 0, so a call that reached Ja
       by the energy-gap heuristic or by v90_note_ja_confirmed_by_descriptor()
       -- neither of which runs the canonical J matcher -- left the detector off
       whenever there was also no TRN lock.  The constellation hint below is
       informational; the S detector does not use it. */
    s->rx.phase3_s_detect_armed = true;

    V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
             "Rx - V.90: analogue Ja consumed; armed clean Phase 3 S detector (trn=%s)\n",
             (s->rx.phase3_j_trn16 < 0)
                ? "unknown"
                : (s->rx.phase3_j_trn16 ? "16-point" : "4-point"));
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_copy_phase3_ja_bits(v34_state_t *s,
                                               int hypothesis,
                                               uint8_t bits[],
                                               int max_bits)
{
    int len;
    int total;
    int i;

    if (!s || !bits || max_bits <= 0
        || hypothesis < 0 || hypothesis >= MP_HYPOTHESIS_COUNT)
        return 0;
    total = s->rx.phase3_ja_capture_hyp_len[hypothesis];
    len = v34_ja_window_bits(total);
    if (len > max_bits)
        len = max_bits;
    /* Packed ring; callers still get one byte per bit, oldest first. */
    for (i = 0;  i < len;  i++)
        bits[i] = (uint8_t) v34_ja_window_get(s->rx.phase3_ja_capture_hyp[hypothesis], total, i);
    return len;
}
/*- End of function --------------------------------------------------------*/

/* Total Ja bits EVER captured for this hypothesis, not the ring's retained
   window.  The engine's re-search throttle needs a monotonic quantity: keying
   it on the copy-out length instead stops the search dead once the ring fills,
   because that length stops growing while bits keep arriving. */
SPAN_DECLARE(int) v34_v90_phase3_ja_total_bits(v34_state_t *s, int hypothesis)
{
    if (!s || hypothesis < 0 || hypothesis >= MP_HYPOTHESIS_COUNT)
        return 0;
    return s->rx.phase3_ja_capture_hyp_len[hypothesis];
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_copy_phase3_ja_raw_bits(v34_state_t *s,
                                                   int hypothesis,
                                                   uint8_t bits[],
                                                   int max_bits)
{
    int len;
    int total;
    int i;

    if (!s || !bits || max_bits <= 0
        || hypothesis < 0 || hypothesis >= MP_HYPOTHESIS_COUNT)
        return 0;
    total = s->rx.phase3_ja_capture_hyp_raw_len[hypothesis];
    len = v34_ja_window_bits(total);
    if (len > max_bits)
        len = max_bits;
    /* Packed ring; callers still get one byte per bit, oldest first. */
    for (i = 0;  i < len;  i++)
        bits[i] = (uint8_t) v34_ja_window_get(s->rx.phase3_ja_capture_hyp_raw[hypothesis], total, i);
    return len;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_seed_rx_mp(v34_state_t *s,
                                 int bit_rate_n,
                                 int trellis_size,
                                 int use_non_linear_encoder,
                                 int expanded_shaping,
                                 const int16_t precoder_coeffs[6])
{
    int i;

    if (!s || bit_rate_n < 1 || bit_rate_n > 14
        || set_trellis_mode(s, trellis_size))
        return -1;
    s->rx.bit_rate = (bit_rate_n - 1)*2;
    v34_set_working_parameters(&s->rx.parms,
                               s->rx.baud_rate,
                               s->rx.bit_rate,
                               expanded_shaping != 0);
    s->rx.use_non_linear_encoder = (use_non_linear_encoder != 0);
    for (i = 0;  i < 3;  i++)
    {
        s->rx.h[i].re = precoder_coeffs ? precoder_coeffs[2*i] : 0;
        s->rx.h[i].im = precoder_coeffs ? precoder_coeffs[2*i + 1] : 0;
    }
    /*endfor*/
    /* The external detector has supplied the receive parameters which our MP
       requested of the far-end transmitter.  Mark MP established so the
       existing Phase 4 path can detect E and enter DATA without guessing a
       frame boundary. */
    s->rx.mp_seen = 1;
    s->rx.mp_accepted_baud = s->rx.duration;
    if (s->rx.v90_mode && !s->calling_party)
    {
        /* In V.90 the externally decoded CPt replaces the ordinary V.34
           J'/TRN/MP receive handoff.  Do not leave the native receiver parked
           in PHASE4_TRN after the project layer has supplied the accepted MP
           parameters; enter MP/E search while preserving the trained primary
           equalizer and carrier/timing state. */
        s->rx.stage = V34_RX_STAGE_V90_CP;
        s->rx.duration = 0;
        s->rx.mp_accepted_baud = 0;
        s->rx.bitstream = 0;
        s->rx.bit_count = 0;
        s->rx.mp_count = -1;
        s->rx.mp_frame_pos = 0;
        s->rx.mp_frame_target = 0;
        s->rx.mp_early_rejects = 0;
        s->rx.mp_hypothesis = -1;
        s->rx.mp_phase4_default_scrambler_tap = 4;
        s->rx.scrambler_tap = 4;
        v34_rx_mp_reset_hypothesis_search(&s->rx);
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_set_rx_data_transform(v34_state_t *s,
                                            float scale,
                                            int rotation,
                                            int conjugate)
{
    if (!s || !isfinite(scale) || scale <= 0.0f
        || rotation < 0 || rotation > 3)
        return -1;
    s->rx.data_symbol_scale = scale;
    s->rx.data_symbol_rotation = rotation;
    s->rx.data_symbol_conjugate = (conjugate != 0);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static bool v90_t3_start(v34_rx_state_t *rx);

SPAN_DECLARE(int) v34_v90_prepare_upstream_data(v34_state_t *s,
                                                int baud_rate,
                                                int high_carrier,
                                                int bit_rate,
                                                int trellis_size)
{
    int i;
    int bit_rate_n;
    int max_bit_rate_n;

    if (!s || (baud_rate != V34_BAUD_RATE_3000
               && baud_rate != V34_BAUD_RATE_3200)
        || (high_carrier != 0 && high_carrier != 1)
        || bit_rate < 2400 || bit_rate > 33600 || (bit_rate % 2400) != 0)
        return -1;
    bit_rate_n = bit_rate/2400;
    max_bit_rate_n = (baud_rate_parameters[baud_rate].max_bit_rate_code >> 1) + 1;
    if (bit_rate_n > max_bit_rate_n || set_trellis_mode(s, trellis_size))
        return -1;
    /* V.90 §6.2 requires the digital modem to receive both 3000 and 3200
       symbols/s.  The dedicated V90_CP stage acquires CPt/CP at 2400 baud,
       so INFO1a's selected rate must be supplied explicitly at the CP' seam.
       Recompute carrier/shaper/mapper state without moving the CP stage;
       v34_begin_rx_data() performs the later E-to-B1 handoff. */
    s->rx.baud_rate = baud_rate;
    s->rx.high_carrier = high_carrier != 0;
    s->rx.v34_carrier_phase_rate =
        dds_phase_ratef(carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));
    s->rx.shaper_re = v34_rx_shapers_re[s->rx.baud_rate][s->rx.high_carrier];
    s->rx.shaper_im = v34_rx_shapers_im[s->rx.baud_rate][s->rx.high_carrier];
    create_godard_coeffs(&s->rx.pri_ted,
                         carrier_frequency(s->rx.baud_rate, s->rx.high_carrier),
                         baud_rate_parameters[s->rx.baud_rate].baud_rate,
                         0.99f);
    s->rx.bit_rate = (bit_rate_n - 1)*2;
    v34_set_working_parameters(&s->rx.parms,
                               s->rx.baud_rate,
                               s->rx.bit_rate,
                               false);
    /* CP hypothesis acquisition may move the decision-aided phase tracker,
       but not the Phase-3 equalizer describing the unchanged analogue
       channel.  Restore only coefficients: clearing eq_buf here would lose
       timing/history at the E->B1 sample boundary.

       The taps are only a channel solution on the grid they were adapted on.
       This function has just retuned the carrier, the shaper and the Godard
       coefficients to the SELECTED upstream rate, so if the save was taken at
       another baud rate or carrier assignment the restore puts an equalizer
       from one grid in front of a demodulator running on another.  Log the
       pair on every seam.  Skipping the restore was the probe for why a live
       28800 upstream loses the constellation 0.9 s
       after B1 while a replay of its own recording -- same samples, same
       parameters, ring length ruled out by a 14 s / 0.4 s control -- holds it
       for the whole call, and the equalizer live inherits here and the replay
       does not is the state that differs. */
    {
        V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90 upstream data prepare: equalizer saved at baud %d "
                 "carrier %s, preparing baud %d carrier %s\n",
                 s->rx.eq_coeff_save_baud_rate,
                 s->rx.eq_coeff_save_high_carrier ? "high" : "low",
                 baud_rate,
                 high_carrier ? "high" : "low");
        {
            cvec_copyf(s->rx.eq_coeff, s->rx.eq_coeff_save,
                       V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
        }
        /*endif*/
    }
    s->rx.v90_t3_prepared = true;
    s->rx.v90_t3_trellis_size = trellis_size;
    /* This context's externally visible current rate is the selected V.90
       upstream, not the initial 3200-baud capability ceiling. */
    s->bit_rate = bit_rate;
    /* V.90 §8.5.1 upstream B1/data uses the analogue-modem GPA tap -- unless
       Phase 4 measured the far end using the other one, which slmodemd does. */
    s->rx.scrambler_tap = s->rx.v90_far_tap_measured ? s->rx.v90_far_tap_measured : 4;
    s->rx.use_non_linear_encoder = false;
    for (i = 0;  i < 3;  i++)
    {
        s->rx.h[i].re = 0;
        s->rx.h[i].im = 0;
    }
    /*endfor*/
    /* Begin filling the T/3 ring now, while the peer is still sending CP.
       B1 is only 90 ms long and follows E immediately on the wire, whereas
       the E we anchor on is detected off the CP bit stream and arrives late;
       starting the capture at the handover meant B1 had already gone by. */
    if (v90_upstream_t2_enabled())
    {
        /* Hand the upstream to the ordinary V.34 data receiver instead. */
        s->rx.v90_t3_prepared = false;
        V34_RX_LOG(&s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90 upstream: T/3 branch disabled, using the ordinary "
                 "T/2 data receiver (ME_V90_UPSTREAM_T2)\n");
        return 0;
    }
    /*endif*/
    if (!s->rx.v90_t3_active  &&  !v90_t3_start(&s->rx))
    {
        s->rx.v90_t3_prepared = false;
        return -1;
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_upstream_rx_acquired(v34_state_t *s)
{
    return s && s->rx.v90_t3_acquired;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_upstream_rearm(v34_state_t *s)
{
    if (!s)
        return;
    /*endif*/
    /* B1 acquisition solves the T/3 equalizer from B1 itself (supervised
       least squares), so a restart needs no training sequence -- the same
       state the call started from at the startup E. */
    s->rx.v90_t3_active = false;
    s->rx.v90_t3_acquired = false;
    s->rx.v90_t3_capture_only = false;
    s->rx.v90_t3_prepared = false;
    s->rx.v90_t3_lost_run = 0;
    s->rx.v90_t3_err_base_n = 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v34_v90_upstream_rx_error(v34_state_t *s, float *settled)
{
    if (settled)
        *settled = (s && s->rx.v90_t3_err_base_n >= V34_V90_T3_ERR_BASE_SYMBOLS)
                   ? s->rx.v90_t3_err_base : 0.0f;
    /*endif*/
    return s ? s->rx.v90_t3_sym_err_fast : 0.0f;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_upstream_carrier_lost(v34_state_t *s)
{
    if (!s)
        return 0;
    /*endif*/
    /* An upstream that never acquired at all is the same condition as one
       that has stopped decoding, and worse: the call has carried nothing
       since B1.  Both are what 9.5.1.1 is for. */
    if (s->rx.v90_t3_acq_abandoned)
        return 1;
    /*endif*/
    if (!s->rx.v90_t3_acquired)
        return 0;
    /*endif*/
    return (s->rx.v90_t3_lost_run >= V34_V90_T3_LOST_SYMBOLS) ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

/* Called when a rate renegotiation has been started for this loss, so the
   same one does not start another.  The run restarts from zero, so a
   renegotiation that does not fix anything raises the condition again after
   another V34_V90_T3_LOST_SYMBOLS rather than immediately. */
SPAN_DECLARE(void) v34_v90_upstream_clear_carrier_lost(v34_state_t *s)
{
    if (!s)
        return;
    /*endif*/
    s->rx.v90_t3_lost_run = 0;
    s->rx.v90_t3_acq_abandoned = false;
}
/*- End of function --------------------------------------------------------*/

/* Which arithmetic this library was compiled with.  Reported at startup so a
   live A/B of the two datapaths can confirm from the log which arm ran -- and
   so a libspandsp built one way against an application built the other cannot
   pass unnoticed, which is the failure mode a -D flag in CFLAGS invites. */
SPAN_DECLARE(int) v34_datapath_is_fixed_point(void)
{
#if defined(V34_FIXED_POINT)
    return 1;
#else
    return 0;
#endif
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_upstream_resync_required(v34_state_t *s)
{
    return s && s->rx.v90_t3_acquired && s->rx.v90_t3_resync_required;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_upstream_clear_resync_required(v34_state_t *s)
{
    if (!s)
        return;
    /*endif*/
    s->rx.v90_t3_resync_required = false;
    s->rx.v90_t3_resync_misses = 0;
}
/*- End of function --------------------------------------------------------*/

/* V.34 11.5/11.6: a receiver that has stopped decoding is what both the
   retrain and the rate renegotiation exist for.  Plain V.34 had no read on
   its own data mode at all, so neither could ever be reached. */
SPAN_DECLARE(int) v34_data_carrier_lost(v34_state_t *s)
{
    if (!s)
        return 0;
    /*endif*/
    return (s->rx.data_lost_run >= V34_DATA_LOST_SYMBOLS) ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

/* Called when a recovery has been started for this loss, so the same one does
   not start another.  The run restarts from zero and the settle window is
   re-armed, so a recovery that does not fix anything raises the condition
   again only after the receiver has had time to converge. */
SPAN_DECLARE(void) v34_clear_data_carrier_lost(v34_state_t *s)
{
    if (!s)
        return;
    /*endif*/
    s->rx.data_lost_run = 0;
    s->rx.data_grid_symbols = 0;
    s->rx.data_grid_err_ema = 0.0f;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_upstream_sample_counts(v34_state_t *s,
                                                  int64_t *input_8k,
                                                  int64_t *output_t3)
{
    if (input_8k)
        *input_8k = s ? s->rx.v90_t3_input_count : 0;
    if (output_t3)
        *output_t3 = s ? s->rx.v90_t3_output_count : 0;
}
/*- End of function --------------------------------------------------------*/

static int v90_t3_b1_get_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void v90_t3_b1_put_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

/* Set while v34_rx_b1_search() tries the trellis states: only the winner's
   template is kept, so measuring the CMA dispersion constant over
   V34_V90_T3_CMA_MEASURE_SYMBOLS (32000) symbols past B1 for every candidate
   is wasted -- 16 x ~5 ms in the media thread, which held our transmit up
   for 80-130 ms at every V.34 data-mode entry (rtp-tx.csv,
   rf-tower-v34-2) and put a hole in the signal the far end receives. */
static bool b1_template_skip_cma = false;

static bool v34_build_expected_b1_tap_trellis(v34_rx_state_t *rx,
                                              int scrambler_tap,
                                              int trellis_override,
                                              int initial_state)
{
    v34_state_t *tx;
    int16_t frame[16];
    int16_t precoder[6];
    int rate = (rx->bit_rate/2 + 1)*2400;
    /* The V.90 upstream (T/3 receiver) takes its B1 parameters from V.90.  A
       V.90 call whose INFO1a selected V.34 is a plain V.34 call from Phase 3
       on (9.2.1.1.8): its B1 uses what the two MPs negotiated.  Treating it
       as V.90 here built the template with minimum shaping, no nonlinear
       encoder and the V.90 trellis while the peer transmitted the expanded
       shaping our MP asked for, so above 4800 bit/s -- where the two shapings
       differ -- B1 never correlated (0.25-0.29) and every data mode was white
       (rf-tower-fb-9). */
    bool v90_upstream = rx->v90_mode  &&  !rx->v90_v34_fallback;
    int trellis = (trellis_override >= 0) ? trellis_override
                : v90_upstream ? rx->v90_t3_trellis_size
                               : (rx->viterbi.state_count == 64
                                  ? V34_TRELLIS_64
                                  : (rx->viterbi.state_count == 32
                                     ? V34_TRELLIS_32 : V34_TRELLIS_16));

    for (int i = 0; i < 3; i++)
    {
        precoder[2*i] = rx->h[i].re;
        precoder[2*i + 1] = rx->h[i].im;
    }
    tx = v34_init(NULL,
                  baud_rate_parameters[rx->baud_rate].baud_rate,
                  rate,
                  /* the far end's role: the answer modem when we are the
                     V.34 call modem of a fallback */
                  rx->v90_v34_fallback  ?  false  :  !rx->calling_party,
                  true,
                  v90_t3_b1_get_bit, NULL,
                  v90_t3_b1_put_bit, NULL);
    if (!tx)
        return false;
    /* The received B1 uses the far-end role's clause-7 scrambler.  6.5/V.90
       says the analogue modem uses GPA, but slmodemd measurably scrambles its
       upstream with GPC (Phase-4 TRN descrambles to 100% ones at tap 17 and
       ~54% at tap 4), and a B1 template built with the wrong polynomial does
       not correlate with the wire at all -- coarse score 5%, so acquisition
       failed and no upstream data ever reached the DTE.  Phase 4 has already
       established which tap the far end really uses; reuse it. */
    tx->tx.scrambler_tap = scrambler_tap;
    tx->tx.baud_rate = rx->baud_rate;
    if (v34_seed_tx_data(tx, rate/2400, trellis,
                         v90_upstream ? 0 : rx->use_non_linear_encoder,
                         (v90_upstream && !rx->x2_mode) ? 0 : rx->parms.expanded_shaping,
                         v90_upstream ? NULL : precoder) != 0)
    {
        v34_free(tx);
        return false;
    }
    /* 10.1.3.1 has the trellis encoder zeroed before B1; see
       v34_rx_b1_search() for the peer that does not. */
    tx->tx.state = initial_state;
    if (tx->tx.parms.j > 0)
    {
        tx->tx.super_frame = tx->tx.parms.j - 1;
        tx->tx.v0_pattern = (uint16_t)(2*(tx->tx.parms.j - 1));
    }
    rx->v90_t3_b1_symbols = 0;
    for (int mapping = 0;  mapping < tx->tx.parms.p;  mapping++)
    {
        if (v34_get_mapping_frame_state(tx, frame) != 16)
            break;
        for (int n = 0;  n < 8;  n++)
        {
            int dst = rx->v90_t3_b1_symbols++;
            if (dst >= V34_V90_T3_B1_MAX_SYMBOLS)
            {
                v34_free(tx);
                return false;
            }
            rx->v90_t3_b1[dst].re = frame[2*n]/128.0f;
            rx->v90_t3_b1[dst].im = frame[2*n + 1]/128.0f;
        }
    }
    /* Godard's dispersion constant, from the constellation rather than from
       the call.  It is E[|x|^4]/E[|x|^2] of the points the far end is
       actually transmitting, and this local transmitter is generating exactly
       those -- so it is known here, at the handover, and does not have to be
       waited for.  Measuring it from the received symbols instead needs
       thousands of SETTLED ones, which is precisely what a call that
       collapses early never provides: live on 2026-08-24 a 28800 call lost
       the constellation 0.9 s after B1, so the blind loop that exists to
       recover it could never arm.  Keep generating past B1 for the sample --
       the symbols are still drawn from the same mapper -- and throw them
       away. */
    if (!b1_template_skip_cma)
    {
        double p2_sum = 0.0;
        double p4_sum = 0.0;
        int n_sym = 0;

        for (int n = 0;  n < rx->v90_t3_b1_symbols;  n++)
        {
            double p2 = rx->v90_t3_b1[n].re*rx->v90_t3_b1[n].re
                      + rx->v90_t3_b1[n].im*rx->v90_t3_b1[n].im;

            p2_sum += p2;
            p4_sum += p2*p2;
            n_sym++;
        }
        /*endfor*/
        while (n_sym < V34_V90_T3_CMA_MEASURE_SYMBOLS)
        {
            if (v34_get_mapping_frame_state(tx, frame) != 16)
                break;
            /*endif*/
            for (int n = 0;  n < 8;  n++)
            {
                double re = frame[2*n]/128.0;
                double im = frame[2*n + 1]/128.0;
                double p2 = re*re + im*im;

                p2_sum += p2;
                p4_sum += p2*p2;
                n_sym++;
            }
            /*endfor*/
        }
        /*endwhile*/
        if (p2_sum > 0.0)
        {
            rx->v90_t3_cma_r2 = (float) (p4_sum/p2_sum);
            rx->v90_t3_cma_n = n_sym;
            rx->v90_t3_cma_p2 = p2_sum;
            rx->v90_t3_cma_p4 = p4_sum;
        }
        /*endif*/
    }
    v34_free(tx);
    return rx->v90_t3_b1_symbols > 0;
}
/*- End of function --------------------------------------------------------*/

static bool v34_build_expected_b1_tap(v34_rx_state_t *rx, int scrambler_tap)
{
    return v34_build_expected_b1_tap_trellis(rx, scrambler_tap, -1, 0);
}
/*- End of function --------------------------------------------------------*/

static int v34_expected_b1_default_tap(v34_rx_state_t *rx)
{
    if (!rx->v90_mode)
        return rx->calling_party ? 4 : 17;
    /*endif*/
    /* 6.5/V.90 says the analogue modem scrambles with GPA (tap 4).  Phase 4
       overrides that when it measured the far end using GPC. */
    return rx->v90_far_tap_measured ? rx->v90_far_tap_measured : 4;
}
/*- End of function --------------------------------------------------------*/

static bool v34_build_expected_b1(v34_rx_state_t *rx)
{
    return v34_build_expected_b1_tap(rx, v34_expected_b1_default_tap(rx));
}

/* Normalized correlation of the loaded B1 template against the observed
   symbols starting at offset k, either conjugation. */
float v34_rx_b1_score_at(const v34_rx_state_t *s, int k)
{
    complexf_t c1 = complex_setf(0.0f, 0.0f);
    complexf_t c2 = complex_setf(0.0f, 0.0f);
    float ep = 0.0f;
    float op = 0.0f;

    for (int i = 0;  i < s->v90_t3_b1_symbols;  i++)
    {
        complexf_t o = s->b1_observed[k + i];
        complexf_t e = s->v90_t3_b1[i];

        c1.re += o.re*e.re + o.im*e.im;
        c1.im += o.im*e.re - o.re*e.im;
        c2.re += o.re*e.re - o.im*e.im;
        c2.im += o.im*e.re + o.re*e.im;
        ep += e.re*e.re + e.im*e.im;
        op += o.re*o.re + o.im*o.im;
    }
    /*endfor*/
    if (ep <= 0.0f  ||  op <= 0.0f)
        return 0.0f;
    /*endif*/
    return sqrtf(fmaxf(c1.re*c1.re + c1.im*c1.im, c2.re*c2.re + c2.im*c2.im)/(ep*op));
}
/*- End of function --------------------------------------------------------*/

/* Find B1 among the buffered symbols: where it starts, and which state the
 * far end's trellis encoder started it in.
 *
 * 10.1.3.1 initializes the trellis encoder to zero before B1, and a template
 * built that way is what the receiver has always used.  The RasFinder's
 * MT5634SMI does not: built from each of the 16 states in turn, one state
 * correlates at 0.991-0.993 against the received B1 and every other at
 * 0.71-0.78, and the state differs from call to call (1 on rf-v34-d2 and
 * rf-tower-v34frz-1, 13 on rf-tower-v34cma-1) -- so the peer carries its
 * encoder state into B1.  Every symbol whose 4D subset depends on it came out
 * a quarter turn off in the second half of the 4D pair, which is what read as
 * a white data mode on every plain V.34 call to that peer.  The Viterbi
 * decoder is started in the same state, since its path metrics otherwise
 * assume zero.  Leaves the winning template loaded; returns the state. */
int v34_rx_b1_search(v34_rx_state_t *s, int search, int *offset_out,
                     float *score_out, float *zero_score_out)
{
    int states = s->viterbi.state_count;
    int best_state = 0;
    int best_offset = 0;
    float best = -1.0f;
    int prior;

    if (states <= 0  ||  states > 64)
        states = 1;
    /*endif*/
    b1_template_skip_cma = true;
    for (int st = 0;  st < states;  st++)
    {
        if (!v34_build_expected_b1_tap_trellis(s, v34_expected_b1_default_tap(s), -1, st))
            break;
        /*endif*/
        for (int k = 0;  k <= search;  k++)
        {
            float score = v34_rx_b1_score_at(s, k);

            if (st == 0  &&  k == 0  &&  zero_score_out)
                *zero_score_out = score;
            /*endif*/
            if (score > best)
            {
                best = score;
                best_state = st;
                best_offset = k;
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    b1_template_skip_cma = false;
    (void) v34_build_expected_b1_tap_trellis(s, v34_expected_b1_default_tap(s), -1, best_state);
    /* 10.1.3.1 requires a zero-state B1 at the final data-frame epoch.
       A nonzero-state template match recovers the constellation, but does
       not establish that epoch. A captured peer needs its V0 input phase
       one mapping frame ahead of this reset. Until phase acquisition proves
       V0, do not reject otherwise valid symbols using an assumed inversion
       schedule. Keep the constraint for the specified reset-state B1. */
    if (best_state != 0 && s->viterbi.zero_precoder)
    {
        s->viterbi.zero_precoder = false;
        s->v0_acquiring = s->viterbi.state_count == 16 && !s->v90_mode;
        s->v0_pairs = 0;
        memset(s->v0_score, 0, sizeof(s->v0_score));
    }
    prior = (s->viterbi.ptr - 1) & 0xF;
    for (int state = 0;  state < s->viterbi.state_count;  state++)
    {
        s->viterbi.vit[prior].cumulative_path_metric[state] =
            (state == best_state)  ?  0U  :  0x3FFFFFFFU;
    }
    /*endfor*/
    if (offset_out)
        *offset_out = best_offset;
    /*endif*/
    if (score_out)
        *score_out = best;
    /*endif*/
    return best_state;
}
/*- End of function --------------------------------------------------------*/

/* V.90's upstream is an ordinary V.34 signal, so in principle the ordinary
   V.34 data receiver -- RRC front end, 127-tap T/2 equalizer, DD-LMS and the
   B1-calibrated derotator -- should decode it.  It has never been asked to:
   v34_v90_prepare_upstream_data() always hands the upstream to the dedicated
   T/3 receiver instead.  That choice was made when the ordinary path "phase
   locked nowhere" live, which is also what plain V.34 looked like before the
   engine's receive-path notch was found (docs/v34_data_mode_rates.md).
   ME_V90_UPSTREAM_T2=1 takes the T/3 branch out so the two can be compared on
   the same call. */
static bool v90_upstream_t2_enabled(void)
{
    static int cached = -1;

    if (cached < 0)
    {
        const char *e = getenv("ME_V90_UPSTREAM_T2");

        cached = (e  &&  (e[0] == '1'  ||  e[0] == 'y'  ||  e[0] == 'Y'))  ?  1  :  0;
    }
    /*endif*/
    return cached != 0;
}

/* True where the ordinary T/2 data receiver owns this stream: every plain
   V.34 call, and a V.90 upstream that the T/3 branch did not take. */
bool v34_rx_t2_data_path(const v34_rx_state_t *s)
{
    return !s->v90_mode  ||  !s->v90_t3_prepared;
}

static bool v90_t3_start(v34_rx_state_t *rx)
{
    rx->v90_t3_active = false;
    rx->v90_t3_internal_rate =
        3*baud_rate_parameters[rx->baud_rate].baud_rate;
    if (rx->v90_t3_internal_rate != 9000
        && rx->v90_t3_internal_rate != 9600)
        return false;
    rx->v90_t3_acquisition_attempted = false;
    rx->v90_t3_sweep_base = 0;
    rx->v90_t3_sweep_best = 0;
    rx->v90_t3_sweep_best_pos = 0;
    rx->v90_t3_sweep_episodes = 0;
    rx->v90_t3_phase_delta = 0;
    rx->v90_t3_phase_pending = false;
    rx->v90_t3_phase_pos = 0;
    rx->v90_t3_acq_retries = 0;
    rx->v90_t3_acq_abandoned = false;
    rx->v90_t3_acq_best_valid = false;
    rx->v90_t3_acq_best_rel = 0.0f;
    rx->v90_t3_acq_best_dist = 0.0f;
    rx->v90_t3_acq_best_match = 0.0f;
    rx->v90_t3_acq_best_conjugate = 0;
    rx->v90_t3_acq_best_first = 0;
    rx->v90_t3_acq_retry_at = 0;
    rx->v90_t3_e_anchor = -1;
    rx->v90_t3_in_b1 = false;
    rx->v90_t3_b1_frame_err = 0.0f;
    rx->v90_t3_capture_only = false;
    rx->v90_t3_acquired = false;
    rx->v90_t3_input_count = 0;
    rx->v90_t3_next_output = 0;
    rx->v90_t3_output_count = 0;
    rx->v90_t3_hilbert_pos = 0;
    rx->v90_t3_rrc_pos = 0;
    rx->v90_t3_raw_count = 0;
    rx->v90_t3_next_symbol = 0;
    rx->v90_t3_publish_symbol = 0;
    rx->v90_t3_suppress_output = true;
    rx->v90_t3_training_match = 0.0f;
    rx->v90_t3_fse_conjugate = false;
    memset(rx->v90_t3_hilbert, 0, sizeof(rx->v90_t3_hilbert));
    memset(rx->v90_t3_input, 0, sizeof(rx->v90_t3_input));
    memset(rx->v90_t3_rrc, 0, sizeof(rx->v90_t3_rrc));
    memset(rx->v90_t3_raw, 0, sizeof(rx->v90_t3_raw));
    memset(rx->v90_t3_matched, 0, sizeof(rx->v90_t3_matched));
    memset(rx->v90_t3_fse, 0, sizeof(rx->v90_t3_fse));
    v90_t3_make_rrc(rx);
    if (!v34_build_expected_b1(rx))
        return false;
    rx->v90_t3_active = true;
    return true;
}

SPAN_DECLARE(int) v34_begin_rx_data(v34_state_t *s)
{
    int prior;

    if (!s)
        return -1;
    /* 9.6's CP-stage training does not outlive the CP stage. */
    s->rx.reneg_cp_train = 0;
    /* Startup CP uses the 24-hypothesis search; 9.6 streams.  The pinning
       argument does not distinguish them -- domain, dibit transform,
       scrambler tap and bit order are fixed by 8.5.2/10.1.3.3 and the
       constellation table in BOTH -- so it is worth knowing whether the
       search is needed here.

       MEASURED, AND THE ANSWER IS THAT IT IS NEEDED.  On the startup CP
       receive itself streaming is indistinguishable -- vpcm_loopback_test
       reports sync=10 valid=5 rejected=4 (crc=1 structure=3) accepted=1/1/0
       rx_data=1 either way, differing only in delivering 6242 bits against
       6240, since it emits every symbol rather than starting at a lock.  But
       `make test` fails with it on: the 11.6 renegotiation row 2400/9600/ulaw
       comes back with 24 post-renegotiation bit errors on the answerer
       against 0, reproducibly, and the caller's resync restarts go 5 -> 12.
       So the search is doing something at startup that the Table 14 framer
       does not do for itself, and the streamed path is NOT a drop-in here the
       way it is for 9.6.

       The switch that streamed it is gone; the measurement stays here because
       the question will come up again the next time someone reads the 24-hypothesis machinery
       and concludes it is dead weight.  It is not. */
    s->rx.v90_cp_stream = 0;
    s->rx.v90_cp_stream_reg = 0;
    s->rx.step_2d = 0;
    s->rx.data_frame = 0;
    s->rx.mapping_frame_count = 0;
    s->rx.s_bit_cnt = 0;
    s->rx.aux_bit_cnt = 0;
    memset(s->rx.xt, 0, sizeof(s->rx.xt));
    memset(s->rx.x, 0, sizeof(s->rx.x));
    memset(s->rx.ww, 0, sizeof(s->rx.ww));
    /* V.34 10.1.3.1 initializes the B1 data scrambler independently of
       the preceding MP/E descrambler stream. */
    s->rx.scramble_reg = 0;
    s->rx.viterbi.ptr = 0;
    s->rx.viterbi.windup = 15;
    s->rx.v0_acquiring = false;
    /* The V.90 T/3 path independently re-pins B1/frame phase and does not
       yet preserve the input inversion epoch required by this constraint.
       Keep its existing decoder until that acquisition path is corrected. */
    s->rx.viterbi.zero_precoder = !s->rx.v90_mode;
    for (int i = 0; i < 3; i++)
        if (s->rx.h[i].re != 0 || s->rx.h[i].im != 0)
            s->rx.viterbi.zero_precoder = false;
    /* 10.1.3.1/V.34: B1 is a reset-state data frame that carries the
       superframe synchronization inversions of the FINAL data frame in a
       superframe, and the convolutional encoder is reset to state zero.
       Initializing to an ordinary frame zero leaves the V0 inversion
       pattern out of phase for the whole connection. */
    if (s->rx.parms.j > 0)
    {
        s->rx.super_frame = s->rx.parms.j - 1;
        s->rx.v0_pattern = (uint16_t)(2*(s->rx.parms.j - 1));
        s->rx.input_4d = (s->rx.parms.j - 1)*4*s->rx.parms.p;
    }
    else
    {
        s->rx.super_frame = 0;
        s->rx.v0_pattern = 0;
        s->rx.input_4d = 0;
    }
    /*endif*/
    prior = (s->rx.viterbi.ptr - 1) & 0xF;
    for (int state = 0;  state < s->rx.viterbi.state_count;  state++)
    {
        s->rx.viterbi.vit[prior].cumulative_path_metric[state] =
            (state == 0)  ?  0U  :  0x3FFFFFFFU;
    }
    /*endfor*/
    s->rx.received_event = V34_EVENT_E;
    s->rx.mp_seen = 2;
    s->rx.b1_acquisition_active = false;
    s->rx.b1_observed_symbols = 0;
    s->rx.b1_corr_valid = false;
    if (s->rx.v90_t3_prepared)
    {
        if (s->rx.v90_t3_active)
        {
            /* Already capturing since the upstream was armed -- keep the
               history and just mark where E landed. */
            s->rx.v90_t3_e_anchor = s->rx.v90_t3_raw_count;
        }
        else
        {
            if (!v90_t3_start(&s->rx))
                return -1;
            /*endif*/
            s->rx.v90_t3_e_anchor = s->rx.v90_t3_raw_count;
        }
        /*endif*/
    }
    /*endif*/
    if (v34_rx_t2_data_path(&s->rx) && v34_build_expected_b1(&s->rx))
        s->rx.b1_acquisition_active = true;
    /* 11.4.1.1.5/11.4.1.2.5 conditions on the complete known B1 before
       unclamping user data.  Buffer B1 to measure gain, phase and conjugation
       from waveform evidence, then replay it through this reset-state decoder. */
    s->rx.duration = 0;
    s->rx.stage = V34_RX_STAGE_DATA;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_put_mapping_frame_state(v34_state_t *s,
                                               int16_t bits[16])
{
    if (s && bits)
        v34_put_mapping_frame(&s->rx, bits);
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

void v34_rx_set_primary_channel(v34_state_t *s, int baud_rate, int high_carrier)
{
    if (!s || baud_rate < V34_BAUD_RATE_2400 || baud_rate > V34_BAUD_RATE_3429)
        return;
    /* V.34 10.1.2.3.5 and Table 16 select the two directions
       independently.  Apply the direction this receiver will demodulate;
       the configured startup profile is only a capability ceiling. */
    s->rx.baud_rate = baud_rate;
    s->rx.high_carrier = high_carrier != 0;
    s->rx.v34_carrier_phase_rate =
        dds_phase_ratef(carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));
    v34_set_working_parameters(&s->rx.parms, s->rx.baud_rate, s->rx.bit_rate, true);
    create_godard_coeffs(&s->rx.pri_ted,
                         carrier_frequency(s->rx.baud_rate, s->rx.high_carrier),
                         baud_rate_parameters[s->rx.baud_rate].baud_rate,
                         0.99f);
}
/*- End of function --------------------------------------------------------*/

int v34_rx_restart(v34_state_t *s, int baud_rate, int bit_rate, int high_carrier)
{
    int i;

    s->rx.baud_rate = baud_rate;
    s->rx.bit_rate = bit_rate;
    s->rx.high_carrier = high_carrier;
    s->rx.training_failed_reported = false;
    /* 9.6's CP-stage training does not outlive the CP stage. */
    s->rx.reneg_cp_train = 0;
    s->rx.v90_cp_stream = 0;
    s->rx.v90_t3_prepared = false;
    s->rx.v90_t3_active = false;
    /* Phase 3/4 peer-retrain detectors: a restart returns to Phase 2, where
       primary_channel_rx() -- and therefore these detectors' out-of-stage
       reset branches -- never runs, so one-shot flags latched during the
       previous attempt would otherwise mute the detectors for the whole next
       attempt.  Observed live 2026-07-22: the peer's second retrain (1.9 s of
       Tone A in armed stages) went unreported because phase34_tone_a_reported
       survived the first retrain's restart. */
    s->rx.phase34_silence_samples = 0;
    s->rx.phase34_retrain_reported = false;
    s->rx.phase34_tone_a_g1 = 0.0f;
    s->rx.phase34_tone_a_g2 = 0.0f;
    s->rx.phase34_guard_g1 = 0.0f;
    s->rx.phase34_guard_g2 = 0.0f;
    s->rx.phase34_tone_a_energy = 0.0f;
    s->rx.phase34_tone_a_samples = 0;
    s->rx.phase34_tone_a_blocks = 0;
    s->rx.phase34_tone_a_reported = false;
    s->rx.phase3_energy_samples = 0;
    s->rx.phase3_energy_retrain_reported = false;

    s->rx.v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));
    /* Phase 2 INFO exchange: answerer RX at 1200 Hz (tone B), caller RX at 2400 Hz (tone A).
       V.90 §8.2.3.1: in V.90 mode carriers are swapped — digital modem (answerer)
       RX listens at 2400 Hz (analog modem's CC carrier).
       This gets updated to Phase 4 CC frequencies in mp_or_mph_baud_init(). */
    /* V.90 §8.2.3.1: digital modem (answerer) listens at 2400 Hz (analog modem's CC),
       analog modem (caller) listens at 1200 Hz (digital modem's CC).
       Standard V.34: caller RX at 2400 Hz, answerer RX at 1200 Hz. */
    if (s->rx.v90_mode && !s->calling_party)
        s->rx.cc_carrier_phase_rate = dds_phase_ratef(2400.0f);
    else if (s->rx.v90_mode && s->calling_party)
        s->rx.cc_carrier_phase_rate = dds_phase_ratef(1200.0f);
    else
        s->rx.cc_carrier_phase_rate = dds_phase_ratef((s->calling_party)  ?  2400.0f  :  1200.0f);
    v34_set_working_parameters(&s->rx.parms, s->rx.baud_rate, s->rx.bit_rate, true);
    s->rx.parms.max_bit_rate_code = bit_rate;

    s->rx.high_sample = 0;
    s->rx.low_samples = 0;
    s->rx.carrier_drop_pending = false;

    power_meter_init(&s->rx.power, 4);

    s->rx.carrier_phase = 0;
    s->rx.agc_scaling_save = 0.0f;
    s->rx.agc_scaling = 0.0017f/V34_RX_PULSESHAPER_GAIN;
    equalizer_reset(&s->rx);
    s->rx.carrier_track_i = 5000.0f;
    s->rx.carrier_track_p = 40000.0f;

    /* Create a default symbol sync filter */
    create_godard_coeffs(&s->rx.pri_ted,
                         carrier_frequency(s->rx.baud_rate, s->rx.high_carrier),
                         baud_rate_parameters[s->rx.baud_rate].baud_rate,
                         0.99f);
    create_godard_coeffs(&s->rx.cc_ted,
                         (s->rx.v90_mode && !s->calling_party)  ?  2400.0f  :  ((s->calling_party)  ?  2400.0f  :  1200.0f),
                         600,
                         0.99f);
    /* Initialise the working data for symbol timing synchronisation */
#if defined(SPANDSP_USE_FIXED_POINT)
    for (i = 0;  i < 2;  i++)
    {
        s->rx.pri_ted.symbol_sync_low[i] = 0;
        s->rx.pri_ted.symbol_sync_high[i] = 0;
        s->rx.pri_ted.symbol_sync_dc_filter[i] = 0;
    }
    /*endfor*/
    s->rx.pri_ted.baud_phase = 0;
    for (i = 0;  i < 2;  i++)
    {
        s->rx.cc_ted.symbol_sync_low[i] = 0;
        s->rx.cc_ted.symbol_sync_high[i] = 0;
        s->rx.cc_ted.symbol_sync_dc_filter[i] = 0;
    }
    /*endfor*/
    s->rx.cc_ted.baud_phase = 0;
#else
    for (i = 0;  i < 2;  i++)
    {
        s->rx.pri_ted.symbol_sync_low[i] = 0.0f;
        s->rx.pri_ted.symbol_sync_high[i] = 0.0f;
        s->rx.pri_ted.symbol_sync_dc_filter[i] = 0.0f;
    }
    /*endfor*/
    s->rx.pri_ted.baud_phase = 0.0f;
    for (i = 0;  i < 2;  i++)
    {
        s->rx.cc_ted.symbol_sync_low[i] = 0.0f;
        s->rx.cc_ted.symbol_sync_high[i] = 0.0f;
        s->rx.cc_ted.symbol_sync_dc_filter[i] = 0.0f;
    }
    /*endfor*/
    s->rx.cc_ted.baud_phase = 0.0f;
#endif
    s->rx.baud_half = 0;

    s->rx.bitstream = 0;
    s->rx.bit_count = 0;
    s->rx.duration = 0;
    s->rx.blip_duration = 0;
    s->rx.last_angles[0] = 0;
    s->rx.last_angles[1] = 0;
    s->rx.total_baud_timing_correction = 0;
    s->rx.phase3_s_guard_samples = 4000;
    s->rx.phase3_s_hits = 0;
    s->rx.phase3_s_event_count = 0;
    s->rx.phase3_s_present = false;
    s->rx.phase3_s_alt_window = 0;
    s->rx.phase3_s_alt_count = 0;
    s->rx.phase3_s_stable_windows = 0;
    s->rx.phase3_s_detect_armed = false;
    memset(s->rx.phase3_s_ring, 0, sizeof(s->rx.phase3_s_ring));
    memset(s->rx.phase3_s_mag_ring, 0, sizeof(s->rx.phase3_s_mag_ring));
    memset(s->rx.phase3_s_counts, 0, sizeof(s->rx.phase3_s_counts));
    s->rx.phase3_s_pos = 0;
    phase3_pp_reset(&s->rx);
    memset(s->rx.phase3_j_scramble, 0, sizeof(s->rx.phase3_j_scramble));
    memset(s->rx.phase3_j_stream, 0, sizeof(s->rx.phase3_j_stream));
    memset(s->rx.phase3_j_prev_z, 0, sizeof(s->rx.phase3_j_prev_z));
    memset(s->rx.phase3_j_prev_valid, 0, sizeof(s->rx.phase3_j_prev_valid));
    memset(s->rx.phase3_j_win, 0, sizeof(s->rx.phase3_j_win));
    s->rx.phase3_j_bits = 0;
    s->rx.phase3_j_lock_hyp = -1;
    s->rx.phase3_j_trn16 = -1;
    s->rx.phase3_j_candidate_hyp = -1;
    s->rx.phase3_j_candidate_phase = -1;
    s->rx.phase3_j_candidate_pat = -1;
    s->rx.phase3_j_candidate_count = 0;
    s->rx.phase3_j_candidate_last_bits = 0;
    memset(s->rx.phase3_ja_scramble, 0, sizeof(s->rx.phase3_ja_scramble));
    memset(s->rx.phase3_ja_prev_z, 0, sizeof(s->rx.phase3_ja_prev_z));
    memset(s->rx.phase3_ja_prev_valid, 0, sizeof(s->rx.phase3_ja_prev_valid));
    s->rx.phase3_ja_bits = 0;
    s->rx.phase3_ja_hyp = -1;
    memset(s->rx.phase3_ja_capture, 0, sizeof(s->rx.phase3_ja_capture));
    s->rx.phase3_ja_capture_len = 0;
    memset(s->rx.phase3_ja_capture_hyp, 0, sizeof(s->rx.phase3_ja_capture_hyp));
    memset(s->rx.phase3_ja_capture_hyp_len, 0, sizeof(s->rx.phase3_ja_capture_hyp_len));
    memset(s->rx.phase3_ja_capture_hyp_raw, 0, sizeof(s->rx.phase3_ja_capture_hyp_raw));
    memset(s->rx.phase3_ja_capture_hyp_raw_len, 0, sizeof(s->rx.phase3_ja_capture_hyp_raw_len));
    phase3_trn_hyp_reset(&s->rx);
    s->rx.phase2_reversal_count = 0;
    s->rx.phase2_l2_count = 0;
    s->rx.phase3_trn_mag_sum = 0.0f;
    s->rx.phase3_trn_mag_count = 0;
    s->rx.phase4_j_seen = 0;
    s->rx.phase4_j_lock_hyp = -1;
    s->rx.phase4_trn_after_j = 0;
    phase4_j_detector_reset(&s->rx);
    v34_rx_phase4_trn_hyp_reset(&s->rx);

    s->rx.info0_received = false;
    s->rx.info1a_received = false;
    s->rx.info1c_received = false;
    s->rx.info1a_raw_12_17 = 0;
    s->rx.info1a_raw_32_33 = 0;
    s->rx.info1a_raw_40_49 = 0;
    s->rx.info0_raw_26_27 = 0;
    s->rx.info0d_nominal_power_code = 0;
    s->rx.info0d_max_power_code = 0;
    s->rx.info0d_power_measured_at_codec_output = false;
    s->rx.info0d_pcm_alaw = false;
    s->rx.info0d_upstream_3429_support = false;
    s->rx.info0d_reserved_41 = 0;
    s->rx.info0d_extensions_valid = false;
    s->rx.v90_repeated_info0a_pending = false;
    s->rx.v90_info1d_sent = false;
    /* Not pinned until the V.90 Phase 4 CP receiver arms it.  Hypothesis 0 is
       a legal index, so this must not be left at the zero-initialised value. */
    s->rx.v90_cp_diff_hypothesis = -1;
    s->rx.stage = V34_RX_STAGE_INFO0;
    /* The stage above says INFO0, so the length must be INFO0's.  INFOh is only
       ever the next message once the stage has moved to V34_RX_STAGE_INFOH,
       which is where its 51 bits are set; setting them here contradicted the
       stage on the line above and truncated nothing but broke the CRC. */
    s->rx.target_bits = 49 - (4 + 8 + 4);

    s->rx.mp_count = -1;
    s->rx.mp_len = 0;
    s->rx.mp_seen = -1;
    s->rx.mp_remote_ack_seen = 0;
    s->rx.mp_remote_ack_count = 0;
    s->rx.mp_frame_pos = 0;
    s->rx.mp_frame_target = 0;
    s->rx.mp_early_rejects = 0;
    s->rx.mp_phase4_default_scrambler_tap = s->rx.scrambler_tap;
    s->rx.mp_phase4_default_bit_order = 0;
    s->rx.mp_phase4_default_domain = 0;
    s->rx.mp_phase4_reject_streak = 0;
    s->rx.mp_phase4_nolock_count = 0;
    s->rx.mp_phase4_alt_tap_active = 0;
    s->rx.mp_phase4_alt_order_active = 0;
    s->rx.mp_phase4_alt_domain_active = 0;
    s->rx.last_rx_mp_valid = false;
    memset(&s->rx.last_rx_mp, 0, sizeof(s->rx.last_rx_mp));
    s->rx.mp_phase4_retry_mode = 0;
    s->rx.mp_phase4_bit_order = 0;
    s->rx.mp_phase4_domain = 0;
    s->rx.mp_phase4_force_abs_active = 0;
    s->rx.mp_phase4_diff_collapse_streak = 0;
    s->rx.mp_phase4_diff_recover_streak = 0;
    s->rx.phase4_da_active = 0;
    s->rx.phase4_da_seeded = 0;
    s->rx.phase4_da_derot = 0;
    v34_rx_mp_reset_hypothesis_search(&s->rx);
    v34_rx_mp_vote_reset(&s->rx);
    s->rx.last_logged_mp_diag_state = V34_MP_DIAG_STATE_NONE;
    s->rx.last_logged_stage = -1;
    s->rx.last_logged_event = -1;
    s->rx.last_logged_demodulator = -1;

    s->rx.viterbi.ptr = 0;
    s->rx.viterbi.windup = 15;

    s->rx.eq_put_step = RX_PULSESHAPER_2400_COEFF_SETS*40/(3*2) - 1;
    s->rx.eq_step = 0;
    s->rx.scramble_reg = 0;

    s->rx.current_demodulator = V34_MODULATION_TONES;
    (void) set_trellis_mode(s, V34_TRELLIS_16);

    s->rx.v0_pattern = 0;
    s->rx.super_frame = 0;
    s->rx.data_frame = 0;
    s->rx.s_bit_cnt = 0;
    s->rx.aux_bit_cnt = 0;
    s->rx.mapping_frame_count = 0;
    /* The primary equalizer is normalized to unit training radius and the
       data mapper's Q9.7 contract is applied explicitly in DATA.  A legacy
       capture-specific factor of 70 drove ordinary V.34 symbols to roughly
       +/-150 constellation units (and int16 clipping) instead of the Table
       10/11 odd-integer grid.  Start at unity; B1 calibration can refine it. */
    s->rx.data_symbol_scale = 1.0f;
    s->rx.data_symbol_rotation = 0;
    s->rx.data_symbol_conjugate = false;

    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_qam_report_handler(v34_state_t *s, qam_report_handler_t handler, void *user_data)
{
    s->rx.qam_report = handler;
    s->rx.qam_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/

/* x2 Draft 0.33 3.4 and QF marker parser 97b5..97dd: retune the
 * upstream receiver from the directional marker instead of INFO1a.
 * Only the V.34 receive front end is borrowed; x2 owns all transmitted PCM.
 */
SPAN_DECLARE(int) v34_x2_prepare_upstream(v34_state_t *s, int baud_index, int high_carrier)
{
    if (!s || baud_index != V34_BAUD_RATE_3200 || high_carrier != 1)
        return -1;
    /* Receive the peer's S/PP/TRN/J without local V.34 S/PP echo gating.
     * The primary front end already supports the asymmetric upstream role. */
    s->rx.v90_mode = true;
    s->rx.x2_mode = true;
    s->tx.v90_mode = true;
    v34_rx_restart(s, baud_index, (31200/2400 - 1)*2, high_carrier);
    s->primary_channel_active = true;
    s->rx.scrambler_tap = 4; /* GPA, tap 5, as executed on Courier originator */
    s->rx.current_demodulator = V34_MODULATION_V34;
    s->rx.stage = V34_RX_STAGE_PHASE3_TRAINING;
    s->rx.phase3_s_guard_samples = 0;
    return 0;
}
