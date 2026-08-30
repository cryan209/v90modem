/*
 * v34rx_phase3.c - ITU-T V.34 receive, Phase 3 (V34_RX_STAGE_PHASE3_WAIT_S).
 *
 * Lifted out of process_primary_symbol() in v34rx.c.  Like the DATA stage it
 * took nothing from the enclosing switch -- it writes ang1/ang2/ang3,
 * data_bits and phase3_abs_bits itself, declares its own loop variables (the
 * `int t` here deliberately shadows the outer v34_state_t *t), and contains no
 * `break` that was leaving the switch -- so the move is verbatim.
 *
 * What lives here: the three §10.1.3.7 S detectors (the alternation, the
 * sustained ±90°/symbol dominant-dibit path that this peer's second S actually
 * produces, and the independent reversal detector), the Ja/J capture and Table
 * 18 scoring, the TRN ones-lock hint, and the false-positive guards -- A-law
 * digital silence decoding to ±8 rather than exact zero, and the 256-window
 * dominant-run rejection.
 *
 * The J lock hint this stage publishes (phase3_j_lock_hyp) is the MP stage's
 * FALLBACK, used only when PHASE4_TRN has not produced one.  Measured with
 * ME_V34_J_HINT=0 it is never decisive in any coverage available here; see
 * docs/v34_flow_mapping.md, which also records why that is not the same as
 * saying it is dead.
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

#define V34_TRACE_DIAGNOSTICS v34_rx_trace_diagnostics()

static int v90_ja_capture_skip_symbols(void)
{
    static int initialized = 0;
    static int skip = 0;

    if (!initialized)
    {
        const char *value;

        value = getenv("ME_V90_JA_CAPTURE_SKIP_SYMBOLS");
        if (value  &&  value[0] != '\0')
        {
            char *end = NULL;
            long parsed = strtol(value, &end, 10);

            if (end != value  &&  end  &&  *end == '\0'
                &&  parsed > 0  &&  parsed <= 100000)
            {
                skip = (int) parsed;
            }
            /*endif*/
        }
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return skip;
}
/*- End of function --------------------------------------------------------*/

static int v90_phase3_j_lookahead_bits(void)
{
    static int initialized = 0;
    static int lookahead_bits = 0;

    if (!initialized)
    {
        const char *value;
        char *end;
        long parsed;

        value = getenv("ME_V90_J_LOOKAHEAD_BITS");
        if (value  &&  value[0] != '\0')
        {
            end = NULL;
            parsed = strtol(value, &end, 10);
            if (end != value  &&  end  &&  *end == '\0'
                && parsed >= 256  &&  parsed <= 20000)
            {
                lookahead_bits = (int) parsed;
            }
            /*endif*/
        }
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return lookahead_bits;
}

static void phase3_ja_capture_append(v34_rx_state_t *s, int bit0, int bit1)
{
    if (!s)
        return;
    /*endif*/
    if (s->phase3_ja_capture_len + 2 > (int) sizeof(s->phase3_ja_capture))
        return;
    /*endif*/
    s->phase3_ja_capture[s->phase3_ja_capture_len++] = (uint8_t) (bit0 & 1);
    s->phase3_ja_capture[s->phase3_ja_capture_len++] = (uint8_t) (bit1 & 1);
    s->phase3_ja_bits += 2;
    if (s->phase3_ja_bits == 2 || (s->phase3_ja_bits % 256) == 0)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 3 Ja capture: emitted %d bits using hyp=%d\n",
                 s->phase3_ja_bits, s->phase3_ja_hyp);
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

void v34_rx_phase3_wait_s_symbol(v34_rx_state_t *s, const complexf_t *sym)
{
    uint32_t ang1;
    uint32_t ang2;
    uint32_t ang3;
    int data_bits;
    int phase3_abs_bits;

        /* Phase 3 S detection using demodulated differential phase.
           Use a dominant-symbol detector over a 32-baud window.
           In real channels, phase/mapping ambiguity can move S away from a
           fixed symbol index, so don't hardcode data_bits==2. */
        {
            float mag_now;
            float mag_prev;
            float dot;
            int idx;
            int dominant_symbol;
            int dominant_count;
            int previous_symbol;
            int prior2_symbol;
            int old_alt;
            int new_alt;
            int old_rev;
            int new_rev;

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

            if (V34_TRACE_DIAGNOSTICS
                && (s->duration == 1 || (s->duration % 256) == 0))
            {
                fprintf(stderr,
                        "[V34 RAW] WAIT_S case: s=%p stage=%d demod=%d duration=%d event=%d j_bits=%d trn_bits=%d hint=%d/%d%%\n",
                        (void *) s, s->stage, s->current_demodulator, s->duration,
                        s->received_event, s->phase3_j_bits, s->phase3_trn_bits,
                        s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
            }
            /*endif*/

            if (s->duration == 1 || (s->duration % 256) == 0)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3 WAIT_S heartbeat: dur=%d demod=%d event=%d j_bits=%d trn_bits=%d trn_hint=%d/%d%%\n",
                         s->duration, s->current_demodulator, s->received_event,
                         s->phase3_j_bits, s->phase3_trn_bits,
                         s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
            }
            /*endif*/

            /* Explicit Phase 3 J/J' detector:
               - apply all dibit mapping hypotheses
               - map the phase-difference slicer output I_n
               - descramble
               - correlate against J/J' 16-bit templates

               Both roles need the far-end J decode:
               - answerer uses it to leave Phase 3 J and enter Phase 4 wait
               - caller uses it to learn the far-end TRN/J mode before it can
                 legally terminate its own J with J' once S is later seen */
            {
                int h;
                int best_score;
                int best_h;
                int best_p;
                int cap_bit0[MP_HYPOTHESIS_COUNT];
                int cap_bit1[MP_HYPOTHESIS_COUNT];
                uint8_t cap_valid[MP_HYPOTHESIS_COUNT];

                best_score = 0;
                best_h = -1;
                best_p = 0;
                memset(cap_bit0, 0, sizeof(cap_bit0));
                memset(cap_bit1, 0, sizeof(cap_bit1));
                memset(cap_valid, 0, sizeof(cap_valid));
                if ((int) (s->phase3_j_bits >> 1) < v90_ja_capture_skip_symbols())
                {
                    /* Not yet "in" the stage: capture nothing and hold the
                       differential chain unstarted, so the first captured
                       symbol is the one at the perturbed offset. */
                    for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                    {
                        s->phase3_j_prev_valid[h] = 0;
                        s->phase3_j_scramble[h] = 0;
                    }
                    /*endfor*/
                }
                else
                for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                {
                    int raw_sym;

                    raw_sym = v34_rx_map_phase4_raw_bits(data_bits, h);
                    if (s->phase3_j_prev_valid[h])
                    {
                        int in_sym;
                        uint32_t reg;
                        int dbit[2];
                        int b;

                        in_sym = raw_sym;
                        reg = s->phase3_j_scramble[h];
                        dbit[0] = v34_rx_descramble_reg(&reg, s->scrambler_tap, in_sym & 1);
                        dbit[1] = v34_rx_descramble_reg(&reg, s->scrambler_tap, (in_sym >> 1) & 1);
                        s->phase3_j_scramble[h] = reg;
                        cap_bit0[h] = dbit[0];
                        cap_bit1[h] = dbit[1];
                        cap_valid[h] = 1;
                        if (s->phase3_ja_capture_hyp_len[h] + 2 <= (int) sizeof(s->phase3_ja_capture_hyp[h]))
                        {
                            /* One sample-clock stamp per APPENDED bit pair, for
                               hypothesis 8 only.  The symbol dump above is taken
                               before the switch and so records every symbol; if
                               this stream has a gap the symbol stream does not,
                               the bits were lost between demodulation and the
                               capture rather than on the wire. */
                            if (h == 8)
                            {
                                static FILE *bt_fp = NULL;
                                static int bt_tried = 0;

                                if (!bt_tried)
                                {
                                    const char *p = getenv("ME_V90_JA_BITTIME_DUMP");

                                    bt_tried = 1;
                                    if (p  &&  p[0] != '\0')
                                        bt_fp = fopen(p, "wb");
                                    /*endif*/
                                }
                                /*endif*/
                                if (bt_fp)
                                {
                                    float v = (float) s->qam_sample_time;

                                    fwrite(&v, sizeof(float), 1, bt_fp);
                                }
                                /*endif*/
                            }
                            /*endif*/
                            s->phase3_ja_capture_hyp[h][s->phase3_ja_capture_hyp_len[h]++] = (uint8_t) (dbit[0] & 1);
                            s->phase3_ja_capture_hyp[h][s->phase3_ja_capture_hyp_len[h]++] = (uint8_t) (dbit[1] & 1);
                            /* Trajectory of the parser's actual input.  The ME
                               side samples this every 200 calls, which is too
                               coarse to tell "grew to 11k and was wiped" from
                               "never exceeded 11k" -- and those imply opposite
                               fixes.  ~14.3k is where this peer's descriptor
                               starts, so log either side of it. */
                            if (h == 0  &&  (s->phase3_ja_capture_hyp_len[h] % 2048) == 0)
                            {
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 3 Ja parser input: hyp0 len=%d bits (stage=%d, need ~16340 to parse)\n",
                                         s->phase3_ja_capture_hyp_len[h], s->stage);
                            }
                            /*endif*/
                        }
                        /*endif*/
                        if (s->phase3_ja_capture_hyp_raw_len[h] + 2 <= (int) sizeof(s->phase3_ja_capture_hyp_raw[h]))
                        {
                            s->phase3_ja_capture_hyp_raw[h][s->phase3_ja_capture_hyp_raw_len[h]++] = (uint8_t) (in_sym & 1);
                            s->phase3_ja_capture_hyp_raw[h][s->phase3_ja_capture_hyp_raw_len[h]++] = (uint8_t) ((in_sym >> 1) & 1);
                        }
                        /*endif*/
                        s->phase3_j_stream[h] = ((s->phase3_j_stream[h] << 1) | (uint32_t) dbit[0]) & 0xFFFFFFFFU;
                        s->phase3_j_stream[h] = ((s->phase3_j_stream[h] << 1) | (uint32_t) dbit[1]) & 0xFFFFFFFFU;

                        for (b = 0;  b < 2;  b++)
                        {
                            int t;
                            int bit_pos;

                            bit_pos = s->phase3_j_bits + b;
                            for (t = 0;  t < 3;  t++)
                            {
                                int p;

                                for (p = 0;  p < 16;  p++)
                                {
                                    uint32_t w;
                                    int match;
                                    int score;

                                    match = (dbit[b] == v34_rx_phase3_j_pattern_bit(t, bit_pos + p)) ? 1 : 0;
                                    w = s->phase3_j_win[h][t][p];
                                    w = (w << 1) | (uint32_t) match;
                                    s->phase3_j_win[h][t][p] = w;
                                    score = __builtin_popcount(w);
                                    if (score > best_score)
                                    {
                                        best_score = score;
                                        best_h = h;
                                        best_p = p;
                                    }
                                    /*endif*/
                                }
                                /*endfor*/
                            }
                            /*endfor*/
                        }
                        /*endfor*/
                    }
                    /*endif*/
                    s->phase3_j_prev_z[h] = (uint8_t) raw_sym;
                    s->phase3_j_prev_valid[h] = 1;
                }
                /*endfor*/
                s->phase3_j_bits += 2;
                if (!s->calling_party
                    && s->v90_mode
                    && s->phase3_j_trn16 < 0
                    && v90_phase3_j_lookahead_bits() > 0
                    && s->phase3_j_bits >= v90_phase3_j_lookahead_bits()
                    && (s->received_event == V34_EVENT_NONE
                        || s->received_event == V34_EVENT_S))
                {
                    /* The SmartLink test rig has roughly one media frame more
                       receive latency than its downstream-Sd wait permits.  An
                       explicitly configured recovered-bit threshold starts the
                       digital side early enough to compensate.  SmartLink uses
                       the 4-point TRN path in this rig; pin it so the later S
                       transition remains detectable after this synthetic J. */
                    s->phase3_j_trn16 = 0;
                    s->phase3_s_detect_armed = true;
                    s->received_event = V34_EVENT_J;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3: configured V.90 J look-ahead fired at bits=%d (ME_V90_J_LOOKAHEAD_BITS=%d, trn=4-point)\n",
                             s->phase3_j_bits, v90_phase3_j_lookahead_bits());
                }
                /*endif*/
                {
                    int capture_h;

                    capture_h = -1;
                    if (s->phase3_j_lock_hyp >= 0
                        && s->phase3_j_lock_hyp < MP_HYPOTHESIS_COUNT
                        && cap_valid[s->phase3_j_lock_hyp])
                    {
                        capture_h = s->phase3_j_lock_hyp;
                    }
                    else if (s->phase3_ja_hyp >= 0
                             && s->phase3_ja_hyp < MP_HYPOTHESIS_COUNT
                             && cap_valid[s->phase3_ja_hyp])
                    {
                        capture_h = s->phase3_ja_hyp;
                    }
                    else if (s->phase3_trn_lock_hyp >= 0
                             && s->phase3_trn_lock_hyp < MP_HYPOTHESIS_COUNT
                             && cap_valid[s->phase3_trn_lock_hyp])
                    {
                        capture_h = s->phase3_trn_lock_hyp;
                    }
                    else if (best_h >= 0
                             && best_h < MP_HYPOTHESIS_COUNT
                             && cap_valid[best_h])
                    {
                        capture_h = best_h;
                    }
                    /*endif*/
                    if (capture_h >= 0)
                    {
                        s->phase3_ja_hyp = capture_h;
                        phase3_ja_capture_append(s, cap_bit0[capture_h], cap_bit1[capture_h]);
                    }
                    /*endif*/
                }
                if (!s->phase3_s_detect_armed
                    && (s->phase3_j_bits <= 8
                        || (s->phase3_j_bits % 64) == 0))
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 J progress: bits=%d best_score=%d/32 hyp=%d trn_hint=%d/%d%%\n",
                             s->phase3_j_bits, best_score, best_h,
                             s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
                }
                /*endif*/
                if (!s->phase3_s_detect_armed
                    && s->phase3_j_bits >= 16
                    &&
                    (s->phase3_j_bits == 16
                        ||
                        (s->phase3_j_bits % PHASE3_J_PROGRESS_LOG_INTERVAL) == 0))
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 J detector: bits=%d best_score=%d/32 hyp=%d\n",
                             s->phase3_j_bits, best_score, best_h);
                }
                /*endif*/
                if (!s->phase3_s_detect_armed
                    && s->phase3_j_bits >= 32
                    &&
                    best_score >= 24
                    &&
                    (s->received_event == V34_EVENT_NONE || s->received_event == V34_EVENT_S))
                {
                    int d4;
                    int d16;
                    int djd;
                    int dmin;
                    int pat;
                    uint16_t rx_recent16;
                    uint16_t rx_ordered16;
                    char rx_recent_bits[17];
                    char rx_ordered_bits[17];
                    const char *j_validity;
                    int canonical_ok;
                    int emit_diag;

                    rx_recent16 = (uint16_t) (s->phase3_j_stream[best_h] & 0xFFFFU);
                    rx_ordered16 = v34_rx_j_ordered16(rx_recent16, s->phase3_j_bits, best_p);
                    v34_rx_bits16_to_str(rx_recent16, rx_recent_bits);
                    v34_rx_bits16_to_str(rx_ordered16, rx_ordered_bits);
                    d4 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x8990U));
                    d16 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x89B0U));
                    djd = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x899FU));
                    dmin = d4;
                    pat = 0;
                    if (d16 < dmin)
                    {
                        dmin = d16;
                        pat = 1;
                    }
                    /*endif*/
                    if (djd < dmin)
                    {
                        dmin = djd;
                        pat = 2;
                    }
                    /*endif*/
                    if (pat == 1  &&  (d16 + 1) >= d4)
                    {
                        /* Prefer 4-point when J(4)/J(16) are nearly tied.
                           A weak 1-bit advantage for 16-point is not stable
                           enough and can mis-classify TRN mode. */
                        pat = 0;
                        dmin = d4;
                    }
                    /*endif*/
                    canonical_ok = (dmin <= 3);
                    if (pat == 1)
                        j_validity = canonical_ok ? "valid J(16-point)" : "near/non-canonical";
                    else if (pat == 0)
                        j_validity = canonical_ok ? "valid J(4-point)" : "near/non-canonical";
                    else
                        j_validity = canonical_ok ? "valid J'" : "near/non-canonical";
                    emit_diag = (!s->phase3_s_detect_armed)
                               && (canonical_ok || ((s->phase3_j_bits % 16) == 0));
                    if (emit_diag)
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 3 J bits: recent16=%s ordered16=%s phase=%d (%s, d4=%d d16=%d dj'=%d)\n",
                                 rx_recent_bits, rx_ordered_bits, best_p, j_validity, d4, d16, djd);
                    }
                    /*endif*/
                    if (canonical_ok)
                    {
                        if (pat == 2)
                        {
                            /* Phase 3 transition should be driven by J (Table 18).
                               Treat J' hits here as diagnostics only to avoid
                               premature Phase 4 transitions with unknown TRN size. */
                            if (!s->phase3_s_detect_armed
                                && (s->phase3_j_bits % 32) == 0)
                            {
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 3: canonical J' candidate ignored (hyp=%d phase=%d score=%d/32 bits=%d)\n",
                                         best_h, best_p, best_score, s->phase3_j_bits);
                            }
                            /*endif*/
                        }
                        else
                        {
                            /* Require a sustained canonical sequence near the
                               complete Ja signature on the normal path.  Test
                               rigs that need latency compensation use the
                               explicit ME_V90_J_LOOKAHEAD_BITS path above.
                               The sustained path additionally demands
                               near-perfect windows (dmin <= 1, score >= 28):
                               scrambled TRN payload transiently matches the J
                               pattern within 2-3 bit errors across the many
                               hypothesis/phase alignments searched, and eight
                               such weak hits in a row happen in practice -
                               live captures show a false "confirmed J" firing
                               mid-TRN (score 25/32, d4=2..3) which launched Sd
                               ~900 ms before the analogue modem was listening. */
                            if (s->phase3_j_bits < 6000
                                ||  dmin > 1
                                ||  best_score < 28)
                            {
                                s->phase3_j_candidate_count = 0;
                            }
                            else if (s->phase3_j_candidate_hyp == best_h
                                && s->phase3_j_candidate_phase == best_p
                                && s->phase3_j_candidate_pat == pat
                                && s->phase3_j_bits > s->phase3_j_candidate_last_bits
                                && s->phase3_j_bits - s->phase3_j_candidate_last_bits <= 4)
                            {
                                s->phase3_j_candidate_count++;
                                s->phase3_j_candidate_last_bits = s->phase3_j_bits;
                            }
                            else if (s->phase3_j_candidate_hyp != best_h
                                     || s->phase3_j_candidate_phase != best_p
                                     || s->phase3_j_candidate_pat != pat
                                     || s->phase3_j_bits - s->phase3_j_candidate_last_bits > 4)
                            {
                                s->phase3_j_candidate_hyp = best_h;
                                s->phase3_j_candidate_phase = best_p;
                                s->phase3_j_candidate_pat = pat;
                                s->phase3_j_candidate_count = 1;
                                s->phase3_j_candidate_last_bits = s->phase3_j_bits;
                            }
                            if (s->phase3_j_candidate_count >= 8)
                            {
                                int v90_ja_already_consumed;

                                /* The V.90 digital answerer consumes Ja before
                                   the analogue modem later transmits S.  The
                                   canonical detector continues observing the
                                   buffered Ja symbols after the application has
                                   cleared received_event; do not publish that
                                   same Ja a second time and let the TX state
                                   machine mistake it for a Phase 4 transition. */
                                v90_ja_already_consumed = (!s->calling_party
                                                          && s->v90_mode
                                                          && s->phase3_j_trn16 >= 0);
                                s->phase3_j_lock_hyp = v34_rx_j_hint_enabled() ? best_h : -1;
                                s->phase3_j_trn16 = pat;
                                s->phase3_s_detect_armed = true;
                                if (!s->calling_party && !v90_ja_already_consumed)
                                {
                                    s->received_event = V34_EVENT_J;
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 3: confirmed repeating J/Ja (hyp=%d phase=%d hits=%d bits=%d, trn=%s)\n",
                                             best_h, best_p, s->phase3_j_candidate_count,
                                             s->phase3_j_bits, pat ? "16-point" : "4-point");
                                }
                                else if (s->calling_party)
                                {
                                    /* V.34 11.3.1.1.3: receipt of the answer
                                       modem's J is what permits the call modem
                                       to start S.  The detector used to log
                                       this transition without publishing it,
                                       leaving the transmitter silent forever. */
                                    s->received_event = V34_EVENT_J;
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 3: confirmed far-end J for caller (hyp=%d phase=%d hits=%d bits=%d, trn=%s)\n",
                                             best_h, best_p, s->phase3_j_candidate_count,
                                             s->phase3_j_bits, pat ? "16-point" : "4-point");
                                }
                            }
                            else
                            {
                                if (!s->phase3_s_detect_armed)
                                {
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 3: canonical J/Ja candidate %d/8 (hyp=%d phase=%d score=%d/32 bits=%d)\n",
                                             s->phase3_j_candidate_count, best_h, best_p,
                                             best_score, s->phase3_j_bits);
                                }
                            }
                        }
                    }
                    else
                    {
                        if (emit_diag && !s->phase3_s_detect_armed)
                        {
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3: J candidate rejected (non-canonical 16-bit pattern)\n");
                        }
                        /*endif*/
                    }
                }
                /*endif*/

                /* Phase 3 TRN ones-based lock hint (4-point path):
                   score hypotheses by descrambled ones ratio. TRN descrambles
                   to all-ones, so the correct hypothesis stands out.

                   This used to be gated on phase3_j_trn16 < 0, which made the
                   whole TRN lock hostage to a shared field: the canonical J
                   matcher writes its matched pattern there, and v34tx.c clears
                   it to -1 in three more places. The moment anything wrote it,
                   TRN scoring stopped dead. That is why the lock was a lottery
                   -- some calls reached 93%, others produced no lock line at
                   all -- and everything downstream (S-detector constellation,
                   Ja capture anchoring, the rescore) inherited that.

                   The latch below is already self-protecting: it only accepts
                   a hypothesis at >=70% and only ever improves on its best, so
                   letting this run on is harmless. Once TRN gives way to Ja the
                   ones ratio falls to ~50% and simply stops improving the
                   latch, rather than corrupting it. */
                if (s->duration >= 256)
                {
                    int h;
                    int best_trn_h;
                    int best_trn_score;

                    best_trn_h = -1;
                    best_trn_score = -1;
                    for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                    {
                        int raw_sym;
                        uint32_t reg;
                        int d0;
                        int d1;

                        /* TRN is mapped directly onto the constellation; only
                           J/Ja passes through the differential encoder. */
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
                    /* Re-score once the Phase 3 tracking loops have settled.
                     *
                     * phase3_trn_one_count[] is cumulative over the whole TRN
                     * and the lock is a monotonic "best ever" latch, so a score
                     * measured across the tracking transient averages two
                     * different regimes.  Enabling tracking moved this peer's
                     * lock to the correct hypothesis but dropped its reported
                     * confidence 93% -> 72%, purely because the correct
                     * hypothesis carries the pre-convergence stretch where it
                     * was still wrong.  Clear the counters and the latch once,
                     * a little after tracking engages, so the lock is retaken
                     * on post-convergence data only. */
                    if (v34_rx_phase3_tracking_enabled()
                        &&
                        s->phase3_tracking_armed
                        &&
                        s->phase3_trn_rescore_bits == 0
                        &&
                        s->phase3_trn_bits >= 512)
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 3 TRN: rescoring after tracking settled "
                                 "(was hyp=%d %d%% over %d bits)\n",
                                 s->phase3_trn_lock_hyp, s->phase3_trn_lock_score,
                                 s->phase3_trn_bits);
                        memset(s->phase3_trn_one_count, 0, sizeof(s->phase3_trn_one_count));
                        s->phase3_trn_rescore_bits = s->phase3_trn_bits;
                        s->phase3_trn_bits = 0;
                        s->phase3_trn_lock_hyp = -1;
                        s->phase3_trn_lock_score = -1;
                    }
                    /*endif*/
                    {
                        float trn_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                        if (isfinite(trn_mag) && trn_mag > 0.0f) {
                            s->phase3_trn_mag_sum += trn_mag;
                            s->phase3_trn_mag_count++;
                        }
                    }
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
                            s->phase3_tracking_armed = true;
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3 TRN: lock hint hyp=%d ones=%d/%d (%d%%)\n",
                                     best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct);
                        }
                        else if ((s->phase3_trn_bits % 512) == 0)
                        {
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3 TRN: best hyp=%d ones=%d/%d (%d%%)\n",
                                     best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct);
                        }
                        /*endif*/
                    }
                    /*endif*/
                }
                /*endif*/
            }

            idx = (s->duration - 1) & 31;
            int old_sym = s->phase3_s_ring[idx] & 3;
            previous_symbol = (s->duration > 1) ? s->phase3_s_ring[(idx + 31) & 31] & 3 : -1;
            prior2_symbol = (s->duration > 2) ? s->phase3_s_ring[(idx + 30) & 31] & 3 : -1;

            if (s->duration > 32)
                s->phase3_s_counts[old_sym]--;
            /*endif*/
            s->phase3_s_ring[idx] = (uint8_t) (data_bits & 3);
            s->phase3_s_mag_ring[idx] = sqrtf(sym->re*sym->re + sym->im*sym->im);
            s->phase3_s_counts[data_bits & 3]++;
            dominant_symbol = 0;
            dominant_count = s->phase3_s_counts[0];
            if (s->phase3_s_counts[1] > dominant_count)
            {
                dominant_count = s->phase3_s_counts[1];
                dominant_symbol = 1;
            }
            if (s->phase3_s_counts[2] > dominant_count)
            {
                dominant_count = s->phase3_s_counts[2];
                dominant_symbol = 2;
            }
            if (s->phase3_s_counts[3] > dominant_count)
            {
                dominant_count = s->phase3_s_counts[3];
                dominant_symbol = 3;
            }

            /* V.34 section 10.1.3.7 defines S as alternating between point 0
               and that point rotated by 90 degrees.  Some peers render that as
               an A,B,A,B alternation (caught below by phase3_s_alt_count);
               others (SmartLink d-modem, confirmed live) render S/S-bar as a
               steady +/-90 degrees/symbol rotation, i.e. one dominant
               differential dibit held for the whole 128T signal.  A *bare*
               dominant-symbol detector would false-fire on the short dominant
               runs inside scrambled Ja, but those never exceed ~10 bauds,
               whereas S holds >=72; a *sustained* dominant ±90 dibit
               (phase3_s_dom_windows below) is therefore an unambiguous S. */
            old_alt = (s->duration > 32) ? ((s->phase3_s_alt_window >> idx) & 1) : 0;
            new_alt = (prior2_symbol >= 0
                       && data_bits != previous_symbol
                       && data_bits == prior2_symbol) ? 1 : 0;
            if (new_alt)
                s->phase3_s_alt_window |= (1u << idx);
            else
                s->phase3_s_alt_window &= ~(1u << idx);
            s->phase3_s_alt_count += new_alt - old_alt;
            s->s_detect_count = s->phase3_s_alt_count;

            if (s->duration >= 32  &&  s->phase3_s_alt_count >= PHASE3_S_ALTERNATING_MIN)
                s->phase3_s_stable_windows++;
            else
                s->phase3_s_stable_windows = 0;
            /*endif*/

            /* Sustained-rotation tracking: count how long one +/-90 dibit
               (dibit 1 or 3) dominates the 32-baud window.  A change of the
               dominant dibit (the S-to-S-bar rotation flip) or loss of
               dominance restarts the run.

               NOTE (2026-07-24): this rotation detector false-fires on any
               sustained single-frequency tone, because a pure tone differen-
               tially demodulates to a constant dibit.  Live against a USR
               Courier it fired on our own ~1325 Hz DIL echo (RX RMS ~132) and
               on the peer's 2400 Hz Tone A retrain (RMS ~3976) -- neither is a
               real far-end S.  Raw input power does NOT cleanly separate them
               (echo-tone S ~61k..275k overlaps real-signal power), so an energy
               gate is not a sufficient fix; the far-end-S log below now carries
               power/carrier_ref to support a better discriminator. */
            if (s->duration >= 32
                &&  dominant_count >= PHASE3_S_DOMINANT_MIN
                &&  (dominant_symbol == 1  ||  dominant_symbol == 3))
            {
                if (dominant_symbol == s->phase3_s_dom_symbol)
                    s->phase3_s_dom_windows++;
                else
                {
                    s->phase3_s_dom_symbol = dominant_symbol;
                    s->phase3_s_dom_windows = 1;
                }
                /*endif*/
            }
            else
            {
                s->phase3_s_dom_windows = 0;
                s->phase3_s_dom_symbol = -1;
            }
            /*endif*/

            /* Independent reversal detector:
               count bauds where current symbol is close to 180° from previous. */
            mag_now = sqrtf(sym->re * sym->re + sym->im * sym->im);
            mag_prev = sqrtf(s->last_sample.re * s->last_sample.re
                             + s->last_sample.im * s->last_sample.im);
            dot = sym->re*s->last_sample.re + sym->im*s->last_sample.im;
            old_rev = (s->duration > 32) ? ((s->s_window >> idx) & 1) : 0;
            new_rev = (mag_now > 0.2f  &&  mag_prev > 0.2f  &&  dot < -0.15f*mag_now*mag_prev) ? 1 : 0;
            if (new_rev)
                s->s_window |= (1u << idx);
            else
                s->s_window &= ~(1u << idx);
            s->bit_count += new_rev - old_rev;  /* reuse bit_count as reversal window count */

        if (s->duration <= 4 || (s->duration % PHASE3_S_BAUD_LOG_INTERVAL) == 0)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3 S baud %d: mag=%.3f data_bits=%d alt=%d/32 stable=%d dom=%d:%d/32 rev=%d/32 counts=%d,%d,%d,%d\n",
                     s->duration, mag_now, data_bits,
                     s->phase3_s_alt_count, s->phase3_s_stable_windows,
                     dominant_symbol, dominant_count, s->bit_count,
                     s->phase3_s_counts[0], s->phase3_s_counts[1],
                     s->phase3_s_counts[2], s->phase3_s_counts[3]);
        }

        if (s->phase3_s_detect_armed
            && !s->phase3_s_present
            && s->duration >= 64
            /* A-law digital silence decodes to +/-8 rather than exact zero.
               Its near-zero equalizer output has a constant differential
               dibit and otherwise satisfies the sustained-rotation test.
               V.34 10.1.3.7 S is a real primary-channel signal, so require
               non-trivial symbol energy before publishing S. */
            && mag_now > 0.2f
            && mag_prev > 0.2f
            && ((s->phase3_s_alt_count >= PHASE3_S_ALTERNATING_MIN
                 && s->phase3_s_stable_windows >= PHASE3_S_STABLE_WINDOWS)
                || (s->phase3_s_dom_windows >= PHASE3_S_DOMINANT_STABLE
                    &&  s->phase3_s_dom_windows <= PHASE3_S_DOMINANT_RUN_MAX)))
        {
            bool by_rotation = (s->phase3_s_dom_windows >= PHASE3_S_DOMINANT_STABLE
                                &&  s->phase3_s_dom_windows <= PHASE3_S_DOMINANT_RUN_MAX);

            s->phase3_s_present = true;
            s->phase3_s_event_count++;
            s->phase3_s_fired_symbol = by_rotation ? s->phase3_s_dom_symbol : dominant_symbol;
            s->received_event = V34_EVENT_S;
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3: far-end S detected after J decode (count=%d via=%s role=%s alt=%d/32 stable=%d dom=%d:%d/32 domwin=%d rev=%d/32 bits=%d trn=%s power=%ld carrier_ref=%ld)\n",
                     s->phase3_s_event_count,
                     by_rotation ? "rotation" : "alternation",
                     s->calling_party ? "caller"
                                      : (s->v90_mode ? "V.90 digital answerer" : "answerer"),
                     s->phase3_s_alt_count, s->phase3_s_stable_windows,
                     dominant_symbol, dominant_count, s->phase3_s_dom_windows,
                     s->bit_count, s->phase3_j_bits,
                     s->phase3_j_trn16 ? "16-point" : "4-point",
                     (long) power_meter_current(&s->power),
                     (long) s->info_rx_carrier_ref);
        }

        /* Rearm only after the current S has genuinely ended, so V.90 can
           count the later DIL-termination S / the S-to-S-bar transition as a
           fresh event without double-counting one long S signal:
             - alternation form: the A,B pattern faded and no rotation took
               over;
             - rotation form: the dominant +/-90 dibit flipped (S -> S-bar) to
               the opposite rotation. */
        if (s->phase3_s_present && s->duration >= 96
            && ((s->phase3_s_alt_count <= 12 && s->phase3_s_dom_windows == 0)
                || (s->phase3_s_fired_symbol >= 0
                    && s->phase3_s_dom_symbol >= 0
                    && s->phase3_s_dom_symbol != s->phase3_s_fired_symbol)))
        {
            s->phase3_s_present = false;
            s->phase3_s_stable_windows = 0;
            s->phase3_s_fired_symbol = -1;
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3: S detector rearmed after transition %d\n",
                     s->phase3_s_event_count);
        }

        /* Keep the decoded Ja/J state while renewing only the S observation
           window.  S is defined as alternating points separated by 90 degrees,
           so its differential dibit is dominant; scrambled Ja has no such
           dominant dibit even when it happens to contain many 180-degree
           reversals. */
        if (s->duration >= 6000)
        {
            /* Don't force a false S event; keep searching for a real pattern. */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3: S detect timeout (%d bauds, alt=%d/32 rev=%d/32, j_bits=%d trn_bits=%d trn_hint=%d/%d%%), continuing search\n",
                     s->duration, s->phase3_s_alt_count, s->bit_count,
                     s->phase3_j_bits, s->phase3_trn_bits,
                     s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
            s->duration = 0;
            s->s_detect_count = 0;
            s->bit_count = 0;
            s->s_window = 0;
            s->phase3_s_alt_window = 0;
            s->phase3_s_alt_count = 0;
            s->phase3_s_stable_windows = 0;
            s->phase3_s_dom_windows = 0;
            s->phase3_s_dom_symbol = -1;
            memset(s->phase3_s_ring, 0, sizeof(s->phase3_s_ring));
            memset(s->phase3_s_mag_ring, 0, sizeof(s->phase3_s_mag_ring));
            memset(s->phase3_s_counts, 0, sizeof(s->phase3_s_counts));
            s->phase3_s_pos = 0;
        }
        }
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
