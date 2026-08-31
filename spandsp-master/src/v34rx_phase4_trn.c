/*
 * v34rx_phase4_trn.c - ITU-T V.34 receive, Phase 4 TRN (V34_RX_STAGE_PHASE4_TRN).
 *
 * Lifted out of process_primary_symbol() in v34rx.c.  Self-contained in the
 * same way the DATA and PHASE3 stages were: it writes ang1/ang2/ang3 and
 * data_bits itself, uses one loop counter, and had a single `break` -- the
 * one terminating the case.
 *
 * What lives here: §11.4's TRN conditioning, the bounded Phase 4 CMA, the J'
 * detector, and the scoring that produces phase4_trn_lock_* -- the hint the MP
 * stage locks against.
 *
 * That hint is NOT optional.  Measured with ME_V34_TRN_HINT=0, withholding it
 * stops plain V.34 training altogether: trained=0/0, zero bits, 60 s timeout,
 * at 3200/21600 as well as 2400/9600.  The V.90 native startup CP receive is
 * byte-identical either way, because that path does not use the V.34 MP hint,
 * so the stage-local metric signs off on a change that breaks the modem --
 * see docs/v34_flow_mapping.md.
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

/* v34rx.c guards a block of its own helpers with `#ifndef
   V34_TRACE_DIAGNOSTICS` and includes the header above it, so the macro is
   defined per translation unit rather than in the header. */
#define V34_TRACE_DIAGNOSTICS v34_rx_trace_diagnostics()

static int v34_trn_hint_enabled(void)
{
    static int cache = -1;

    if (cache < 0)
    {
        const char *v = getenv("ME_V34_TRN_HINT");

        cache = (v && *v) ? (atoi(v) != 0) : 1;
    }
    /*endif*/
    return cache;
}
/*- End of function --------------------------------------------------------*/

static void phase4_trn_recent_reset(v34_rx_state_t *s)
{
    s->phase4_trn_recent_scramble = 0;
    s->phase4_trn_recent_window_bits = 0;
    s->phase4_trn_recent_window_ones = 0;
    s->phase4_trn_recent_window_fill = 0;
    s->phase4_trn_recent_score = -1;
    memset(s->phase4_trn_recent_symbol_ones, 0, sizeof(s->phase4_trn_recent_symbol_ones));
    s->phase4_trn_recent_active = 0;
}
/*- End of function --------------------------------------------------------*/

static void phase4_trn_recent_seed(v34_rx_state_t *s)
{
    if (s->phase4_trn_lock_hyp < 0
        || s->phase4_trn_lock_domain < 0
        || s->phase4_trn_lock_tap < 0
        || s->phase4_trn_lock_order < 0)
    {
        phase4_trn_recent_reset(s);
        return;
    }
    /*endif*/
    s->phase4_trn_recent_scramble =
        s->phase4_trn_scramble_tap[s->phase4_trn_lock_domain][s->phase4_trn_lock_tap][s->phase4_trn_lock_order][s->phase4_trn_lock_hyp];
    s->phase4_trn_recent_window_bits = 0;
    s->phase4_trn_recent_window_ones = 0;
    s->phase4_trn_recent_window_fill = 0;
    s->phase4_trn_recent_score = -1;
    memset(s->phase4_trn_recent_symbol_ones, 0, sizeof(s->phase4_trn_recent_symbol_ones));
    s->phase4_trn_recent_active = 1;
}
/*- End of function --------------------------------------------------------*/

static void phase4_trn_recent_update(v34_rx_state_t *s, int raw_sym)
{
    int d0;
    int d1;
    int ones;
    int pos;
    int tap;

    if (!s->phase4_trn_recent_active)
        return;
    /*endif*/
    if (s->phase4_trn_lock_hyp < 0)
        return;
    /*endif*/
    if (s->phase4_trn_after_j < PHASE4_TRN_SCORE_START_BAUD)
        return;
    /*endif*/

    tap = (s->phase4_trn_lock_tap == 0) ? 17 : 4;
    if (s->phase4_trn_lock_order == 0)
    {
        d0 = v34_rx_descramble_reg(&s->phase4_trn_recent_scramble, tap, raw_sym & 1);
        d1 = v34_rx_descramble_reg(&s->phase4_trn_recent_scramble, tap, (raw_sym >> 1) & 1);
    }
    else
    {
        d1 = v34_rx_descramble_reg(&s->phase4_trn_recent_scramble, tap, (raw_sym >> 1) & 1);
        d0 = v34_rx_descramble_reg(&s->phase4_trn_recent_scramble, tap, raw_sym & 1);
    }
    /*endif*/
    ones = d0 + d1;
    pos = (s->phase4_trn_after_j - PHASE4_TRN_SCORE_START_BAUD) & (PHASE4_TRN_RECENT_WINDOW_BAUDS - 1);

    if (s->phase4_trn_recent_window_fill >= PHASE4_TRN_RECENT_WINDOW_BAUDS)
        s->phase4_trn_recent_window_ones -= s->phase4_trn_recent_symbol_ones[pos];
    else
        s->phase4_trn_recent_window_fill++;
    /*endif*/
    s->phase4_trn_recent_symbol_ones[pos] = (uint8_t) ones;
    s->phase4_trn_recent_window_ones += (uint16_t) ones;
    s->phase4_trn_recent_window_bits = 2*s->phase4_trn_recent_window_fill;
    if (s->phase4_trn_recent_window_bits > 0)
    {
        s->phase4_trn_recent_score =
            (100*s->phase4_trn_recent_window_ones + (s->phase4_trn_recent_window_bits/2))/s->phase4_trn_recent_window_bits;
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int phase4_trn_tap_value(int tap_idx)
{
    static const int taps[2] = {17, 4};

    if (tap_idx < 0 || tap_idx > 1)
        return taps[0];
    /*endif*/
    return taps[tap_idx];
}
/*- End of function --------------------------------------------------------*/

void v34_rx_phase4_trn_symbol(v34_rx_state_t *s, const complexf_t *sym)
{
    uint32_t ang1;
    uint32_t ang2;
    uint32_t ang3;
    int data_bits;
    int i;

        /* Phase 4: gate MP entry on explicit far-end J' followed by >=512T TRN. */
        {
        int abs_bits;

        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        abs_bits = (int) ((ang1 + DDS_PHASE(45.0f)) >> 30) & 0x3;
        s->duration++;

        /* I/Q constellation diagnostic: log first 64 bauds of S+Sbar and first 64 bauds of TRN scoring */
        #if V34_DEBUG_IQ_LOG
        if (s->phase4_j_seen
            && (s->phase4_trn_after_j < 64
                || (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD
                    && s->phase4_trn_after_j < PHASE4_TRN_SCORE_START_BAUD + 64)))
        {
            float deg_abs = (float)ang1 / (4294967296.0f / 360.0f);
            float deg_diff = (float)ang3 / (4294967296.0f / 360.0f);
            float mag_now = sqrtf(sym->re*sym->re + sym->im*sym->im);
            float deg_prev = (float)ang2 / (4294967296.0f / 360.0f);
            fprintf(stderr, "[IQ] baud=%d re=%.4f im=%.4f mag=%.3f abs=%.1f prev=%.1f diff=%.1f data=%d abs_bits=%d\n",
                    s->phase4_trn_after_j, sym->re, sym->im, mag_now, deg_abs, deg_prev, deg_diff, data_bits, abs_bits);
        }
        #endif

        /* Descramble to let the descrambler self-sync (needs ~23 bits) */
        {
            int raw_bits = data_bits;
            for (i = 0;  i < 2;  i++)
            {
                v34_rx_descramble(s, raw_bits & 1);
                raw_bits >>= 1;
            }
        }

        if (!s->phase4_j_seen)
        {
            int domain_idx;
            int tap_idx;
            int order_idx;
            int h;
            int best_h;
            int best_p;
            int best_score;
            int best_domain;
            int best_tap;
            int best_order;

            best_h = -1;
            best_p = 0;
            best_score = -1;
            best_domain = 0;
            best_tap = 0;
            best_order = 0;
            for (domain_idx = 0;  domain_idx < 2;  domain_idx++)
            {
                for (tap_idx = 0;  tap_idx < 2;  tap_idx++)
                {
                    int tap;

                    tap = phase4_trn_tap_value(tap_idx);
                    for (order_idx = 0;  order_idx < 2;  order_idx++)
                    {
                        for (h = 0;  h < 8;  h++)
                        {
                            int raw_sym;
                            int in_sym;
                            uint32_t reg;
                            int dbit[2];
                            int b;

                            raw_sym = v34_rx_map_phase4_raw_bits(domain_idx ? abs_bits : data_bits, h);
                            if (s->phase4_j_prev_valid_tap[domain_idx][tap_idx][order_idx][h])
                            {
                                in_sym = (raw_sym - s->phase4_j_prev_z_tap[domain_idx][tap_idx][order_idx][h]) & 0x3;
                                reg = s->phase4_j_scramble_tap[domain_idx][tap_idx][order_idx][h];
                                if (order_idx == 0)
                                {
                                    dbit[0] = v34_rx_descramble_reg(&reg, tap, in_sym & 1);
                                    dbit[1] = v34_rx_descramble_reg(&reg, tap, (in_sym >> 1) & 1);
                                }
                                else
                                {
                                    dbit[0] = v34_rx_descramble_reg(&reg, tap, (in_sym >> 1) & 1);
                                    dbit[1] = v34_rx_descramble_reg(&reg, tap, in_sym & 1);
                                }
                                /*endif*/
                                s->phase4_j_scramble_tap[domain_idx][tap_idx][order_idx][h] = reg;
                                s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] =
                                    ((s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] << 1) | (uint32_t) dbit[0]) & 0xFFFFFFFFU;
                                s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] =
                                    ((s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] << 1) | (uint32_t) dbit[1]) & 0xFFFFFFFFU;

                                for (b = 0;  b < 2;  b++)
                                {
                                    int bit_pos;
                                    int p;

                                    bit_pos = s->phase4_j_bits + b;
                                    for (p = 0;  p < 16;  p++)
                                    {
                                        int match;
                                        uint32_t w;
                                        int score;

                                        match = (dbit[b] == v34_rx_phase3_j_pattern_bit(2, bit_pos + p)) ? 1 : 0;
                                        w = s->phase4_j_win_tap[domain_idx][tap_idx][order_idx][h][p];
                                        w = ((w << 1) | match) & 0xFFFFFFFFU;
                                        s->phase4_j_win_tap[domain_idx][tap_idx][order_idx][h][p] = w;
                                        score = __builtin_popcount(w);
                                        if (score > best_score)
                                        {
                                            best_score = score;
                                            best_h = h;
                                            best_p = p;
                                            best_domain = domain_idx;
                                            best_tap = tap_idx;
                                            best_order = order_idx;
                                        }
                                        /*endif*/
                                    }
                                    /*endfor*/
                                }
                                /*endfor*/
                            }
                            /*endif*/
                            s->phase4_j_prev_z_tap[domain_idx][tap_idx][order_idx][h] = (uint8_t) raw_sym;
                            s->phase4_j_prev_valid_tap[domain_idx][tap_idx][order_idx][h] = 1;
                        }
                        /*endfor*/
                    }
                    /*endfor*/
                }
                /*endfor*/
            }
            /*endfor*/

            s->phase4_j_bits += 2;
            if (s->phase4_j_bits >= 16
                &&
                (s->phase4_j_bits == 16
                    ||
                    (s->phase4_j_bits % PHASE4_J_PROGRESS_LOG_INTERVAL) == 0))
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4 J' detector: bits=%d best_score=%d/32 hyp=%d dom=%s tap=%d ord=%s\n",
                         s->phase4_j_bits, best_score, best_h,
                         v34_rx_phase4_trn_domain_name(best_domain),
                         phase4_trn_tap_value(best_tap),
                         v34_rx_phase4_trn_order_name(best_order));
            }
            /*endif*/

            if (s->phase4_j_bits >= 32
                && best_h >= 0
                && best_score >= 24)
            {
                int d4;
                int d16;
                int djd;
                int dmin;
                uint16_t rx_recent16;
                uint16_t rx_ordered16;
                char rx_recent_bits[17];
                char rx_ordered_bits[17];
                const char *j_validity;
                int canonical_ok;
                int emit_diag;

                rx_recent16 = (uint16_t) (s->phase4_j_stream_tap[best_domain][best_tap][best_order][best_h] & 0xFFFFU);
                rx_ordered16 = v34_rx_j_ordered16(rx_recent16, s->phase4_j_bits, best_p);
                v34_rx_bits16_to_str(rx_recent16, rx_recent_bits);
                v34_rx_bits16_to_str(rx_ordered16, rx_ordered_bits);
                d4 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x8990U));
                d16 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x89B0U));
                djd = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x899FU));
                dmin = djd;
                if (d4 < dmin)
                    dmin = d4;
                if (d16 < dmin)
                    dmin = d16;
                canonical_ok = (djd <= 3);
                j_validity = canonical_ok ? "valid J'" : ((dmin <= 3) ? "valid non-J'" : "near/non-canonical");
                emit_diag = canonical_ok || ((s->phase4_j_bits % 16) == 0);
                if (emit_diag)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4 J bits: recent16=%s ordered16=%s phase=%d (%s, d4=%d d16=%d dj'=%d)\n",
                             rx_recent_bits, rx_ordered_bits, best_p, j_validity, d4, d16, djd);
                }
                /*endif*/
                if (canonical_ok)
                {
                    s->phase4_j_seen = 1;
                    s->phase4_trn_after_j = 0;
                    v34_rx_phase4_trn_hyp_reset(s);
                    s->phase4_j_lock_hyp = best_h;
                    /* Use role-based tap, not TRN auto-detected tap (see §7) */
                    s->scrambler_tap = s->calling_party ? 4 : 17;
                    s->received_event = V34_EVENT_J_DASHED;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: explicit J' detected (hyp=%d phase=%d score=%d/32 bits=%d dom=%s tap=%d ord=%s)\n",
                             best_h, best_p, best_score, s->phase4_j_bits,
                             v34_rx_phase4_trn_domain_name(best_domain),
                             phase4_trn_tap_value(best_tap),
                             v34_rx_phase4_trn_order_name(best_order));
                }
                /*endif*/
            }
            /*endif*/
        }
        else
        {
            int h;
            int best_h;
            int best_score;
            int best_tap;
            int best_order;
            int best_domain;
            int lock_raw_sym;

            s->phase4_trn_after_j++;
            /* V.34 11.4: keep the tail of the Phase-4 TRN segment.  TRN is
               constant modulus on the 4-point constellation, so once the
               equalizer has converged the spread of these symbols is a
               direct measurement of the receive channel's wideband SNR --
               through the same filter chain the data mode will use, and on
               a wideband signal rather than the L1/L2 probe's 21 tones.
               v34_phase4_trn_measure_snr() reads the ring at MP time. */
            if (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD)
            {
                s->phase4_trn_snr_ring[s->phase4_trn_snr_pos].re = sym->re;
                s->phase4_trn_snr_ring[s->phase4_trn_snr_pos].im = sym->im;
                if (++s->phase4_trn_snr_pos >= 512)
                    s->phase4_trn_snr_pos = 0;
                /*endif*/
                if (s->phase4_trn_snr_fill < 512)
                    s->phase4_trn_snr_fill++;
                /*endif*/
            }
            /*endif*/
            {
                static const char *p4_dump_path = NULL;
                static const char *p4_bits_path = NULL;
                float eq_main;
                float eq_e = v34_rx_eq_tap_energy(s, &eq_main);

                v34_rx_dump_training_symbol("V34_P4TRN_SYM_DUMP", &p4_dump_path,
                                         s->calling_party,
                                         s->phase4_trn_after_j,
                                         sym->re, sym->im,
                                         (long) power_meter_current(&s->power),
                                         eq_e, eq_main,
                                         s->eq_put_step,
                                         s->total_baud_timing_correction);
                if (p4_bits_path == NULL)
                    p4_bits_path = getenv("V34_P4TRN_RX_DUMP")
                                 ?  getenv("V34_P4TRN_RX_DUMP")  :  "";
                /*endif*/
                if (p4_bits_path[0])
                {
                    FILE *f = fopen(p4_bits_path, "a");

                    if (f)
                    {
                        fprintf(f, "%s %d %d %d\n",
                                s->calling_party ? "caller" : "answer",
                                s->phase4_trn_after_j, data_bits, abs_bits);
                        fclose(f);
                    }
                    /*endif*/
                }
                /*endif*/
            }
            best_h = -1;
            best_score = -1;
            best_tap = -1;
            best_order = -1;
            best_domain = -1;
            for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
            {
                /* Phase 4 after J' starts with S (128T) + S-bar (16T).
                   Do not pollute TRN ones-scoring with those non-TRN symbols. */
                if (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD
                    && s->phase4_trn_prev_valid[h])
                {
                    int domain_idx;
                    int tap_idx;

                    /* TRN is not differentially encoded; score descrambled dibits directly.
                       Evaluate both domains (diff/abs), both complementary taps,
                       and both bit serialization orders in parallel. */
                    for (domain_idx = 0;  domain_idx < 2;  domain_idx++)
                    {
                        int raw_sym;

                        raw_sym = v34_rx_map_phase4_raw_bits(domain_idx ? abs_bits : data_bits, h);
                        for (tap_idx = 0;  tap_idx < 2;  tap_idx++)
                        {
                            int order_idx;

                            for (order_idx = 0;  order_idx < 2;  order_idx++)
                            {
                                uint32_t reg;
                                int d0;
                                int d1;
                                int tap;

                                tap = phase4_trn_tap_value(tap_idx);
                                reg = s->phase4_trn_scramble_tap[domain_idx][tap_idx][order_idx][h];
                                if (order_idx == 0)
                                {
                                    d0 = v34_rx_descramble_reg(&reg, tap, raw_sym & 1);
                                    d1 = v34_rx_descramble_reg(&reg, tap, (raw_sym >> 1) & 1);
                                }
                                else
                                {
                                    d1 = v34_rx_descramble_reg(&reg, tap, (raw_sym >> 1) & 1);
                                    d0 = v34_rx_descramble_reg(&reg, tap, raw_sym & 1);
                                }
                                /*endif*/
                                s->phase4_trn_scramble_tap[domain_idx][tap_idx][order_idx][h] = reg;
                                s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h] += (uint16_t) (d0 + d1);
                                if (s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h] > best_score)
                                {
                                    best_h = h;
                                    best_score = s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h];
                                    best_tap = tap_idx;
                                    best_order = order_idx;
                                    best_domain = domain_idx;
                                }
                                /*endif*/
                            }
                            /*endfor*/
                        }
                        /*endif*/
                    }
                    /*endfor*/
                }
                /*endif*/
                s->phase4_trn_prev_valid[h] = 1;
            }
            /*endfor*/
            if (s->phase4_trn_lock_hyp >= 0
                && s->phase4_trn_lock_domain >= 0
                && s->phase4_trn_lock_tap >= 0
                && s->phase4_trn_lock_order >= 0)
            {
                lock_raw_sym = v34_rx_map_phase4_raw_bits(s->phase4_trn_lock_domain ? abs_bits : data_bits,
                                                   s->phase4_trn_lock_hyp);
                phase4_trn_recent_update(s, lock_raw_sym);
            }
            /*endif*/
            if (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD)
            {
                int bits_observed;
                int score_pct;
                int scored_symbols;

                scored_symbols = s->phase4_trn_after_j - PHASE4_TRN_SCORE_START_BAUD + 1;
                bits_observed = 2*scored_symbols;
                if (bits_observed > 0  &&  best_h >= 0)
                {
                    int old_lock_hyp;
                    int old_lock_score;
                    int old_lock_tap;
                    int old_lock_order;
                    int old_lock_domain;
                    int lock_changed;
                    int lock_identity_changed;

                    score_pct = (100*best_score + (bits_observed/2))/bits_observed;
                    s->phase4_trn_current_hyp = best_h;
                    s->phase4_trn_current_score = score_pct;
                    s->phase4_trn_current_tap = best_tap;
                    s->phase4_trn_current_order = best_order;
                    s->phase4_trn_current_domain = best_domain;
                    old_lock_hyp = s->phase4_trn_lock_hyp;
                    old_lock_score = s->phase4_trn_lock_score;
                    old_lock_tap = s->phase4_trn_lock_tap;
                    old_lock_order = s->phase4_trn_lock_order;
                    old_lock_domain = s->phase4_trn_lock_domain;
                    lock_changed = 0;
                    lock_identity_changed = 0;

                    /* Keep the strongest sustained TRN candidate (with minimum
                       evidence), so later noisy segments do not overwrite it. */
                    if (bits_observed >= PHASE4_TRN_LOCK_MIN_BITS
                        && (s->phase4_trn_lock_hyp < 0
                            || score_pct > s->phase4_trn_lock_score))
                    {
                        s->phase4_trn_lock_hyp = v34_trn_hint_enabled() ? best_h : -1;
                        s->phase4_trn_lock_score = v34_trn_hint_enabled() ? score_pct : -1;
                        s->phase4_trn_lock_tap = best_tap;
                        s->phase4_trn_lock_order = best_order;
                        s->phase4_trn_lock_domain = best_domain;
                    }
                    /*endif*/
                    lock_changed = (s->phase4_trn_lock_hyp != old_lock_hyp
                                    || s->phase4_trn_lock_score != old_lock_score);
                    lock_identity_changed = (s->phase4_trn_lock_hyp != old_lock_hyp
                                             || s->phase4_trn_lock_tap != old_lock_tap
                                             || s->phase4_trn_lock_order != old_lock_order
                                             || s->phase4_trn_lock_domain != old_lock_domain);
                    if (lock_identity_changed)
                        phase4_trn_recent_seed(s);
                    /*endif*/
                    if (s->phase4_trn_recent_active
                        && s->phase4_trn_recent_window_bits > 0
                        && s->phase4_trn_lock_hyp >= 0)
                    {
                        s->phase4_trn_current_hyp = s->phase4_trn_lock_hyp;
                        s->phase4_trn_current_score = s->phase4_trn_recent_score;
                        s->phase4_trn_current_tap = s->phase4_trn_lock_tap;
                        s->phase4_trn_current_order = s->phase4_trn_lock_order;
                        s->phase4_trn_current_domain = s->phase4_trn_lock_domain;

                        /* Cumulative TRN score can be "poisoned" by early noisy
                           symbols and never recover above readiness threshold.
                           Promote a stable full-window recent score for the same
                           locked candidate to avoid stalling in TRN forever. */
                        if (s->phase4_trn_recent_window_fill >= PHASE4_TRN_RECENT_WINDOW_BAUDS
                            && s->phase4_trn_recent_score > s->phase4_trn_lock_score)
                        {
                            s->phase4_trn_lock_score = s->phase4_trn_recent_score;
                            lock_changed = 1;
                        }
                        /*endif*/
                    }
                    /*endif*/
                    if (lock_changed && s->phase4_trn_lock_score >= 70)
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4 TRN: lock hint hyp=%d dom=%s tap=%d ord=%s ones=%d/%d (%d%%, recent=%d%%)\n",
                                 s->phase4_trn_lock_hyp,
                                 v34_rx_phase4_trn_domain_name(s->phase4_trn_lock_domain),
                                 phase4_trn_tap_value(s->phase4_trn_lock_tap),
                                 v34_rx_phase4_trn_order_name(s->phase4_trn_lock_order),
                                 best_score, bits_observed, s->phase4_trn_lock_score,
                                 s->phase4_trn_recent_score);
                    }
                    else if ((s->phase4_trn_after_j % 256) == 0)
                    {
                        /* Find best score for each domain separately */
                        int best_diff_score = 0;
                        int best_abs_score = 0;
                        int dh, dt, do2;
                        for (dt = 0; dt < 2; dt++)
                            for (do2 = 0; do2 < 2; do2++)
                                for (dh = 0; dh < MP_HYPOTHESIS_COUNT; dh++)
                                {
                                    if (s->phase4_trn_one_count_tap[0][dt][do2][dh] > best_diff_score)
                                        best_diff_score = s->phase4_trn_one_count_tap[0][dt][do2][dh];
                                    if (s->phase4_trn_one_count_tap[1][dt][do2][dh] > best_abs_score)
                                        best_abs_score = s->phase4_trn_one_count_tap[1][dt][do2][dh];
                                }
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4 TRN: best hyp=%d dom=%s tap=%d ord=%s ones=%d/%d (%d%%, recent=%d%%) [diff_best=%d%% abs_best=%d%%]\n",
                                 best_h, v34_rx_phase4_trn_domain_name(best_domain),
                                 phase4_trn_tap_value(best_tap), v34_rx_phase4_trn_order_name(best_order),
                                 best_score, bits_observed, score_pct, s->phase4_trn_recent_score,
                                 (100*best_diff_score + (bits_observed/2))/bits_observed,
                                 (100*best_abs_score + (bits_observed/2))/bits_observed);
                    }
                    /*endif*/
                }
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/

        /* Transition to MP scan after explicit J' and sufficient TRN training.
           Per V.34 §11.4, TRN is at least 512T and up to 2000ms+RTD.
           We train on TRN for ~1.5-2s to lock the equalizer and scrambler
           before scanning for MP. */
        if (s->phase4_j_seen
            && s->phase4_trn_after_j >= PHASE4_TRN_READY_MIN_BAUD
            && s->phase4_trn_lock_score >= PHASE4_TRN_READY_MIN_SCORE)
        {
            int h;
            int domain_idx;
            int tap_idx;
            int order_idx;
            int trn_bits_observed;
            int trn_best_h;
            int trn_best_ones;
            int trn_best_score_pct;
            int trn_best_domain;
            int trn_best_tap;
            int trn_best_order;

            if (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD)
                trn_bits_observed = 2*(s->phase4_trn_after_j - PHASE4_TRN_SCORE_START_BAUD + 1);
            else
                trn_bits_observed = 0;
            /*endif*/
            trn_best_h = -1;
            trn_best_ones = -1;
            trn_best_score_pct = 0;
            trn_best_domain = -1;
            trn_best_tap = -1;
            trn_best_order = -1;
            if (trn_bits_observed > 0)
            {
                for (domain_idx = 0;  domain_idx < 2;  domain_idx++)
                {
                    for (tap_idx = 0;  tap_idx < 2;  tap_idx++)
                    {
                        for (order_idx = 0;  order_idx < 2;  order_idx++)
                        {
                            for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                            {
                                if (s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h] > trn_best_ones)
                                {
                                    trn_best_ones = s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h];
                                    trn_best_h = h;
                                    trn_best_domain = domain_idx;
                                    trn_best_tap = tap_idx;
                                    trn_best_order = order_idx;
                                }
                                /*endif*/
                            }
                            /*endif*/
                        }
                        /*endif*/
                    }
                    /*endif*/
                }
                /*endfor*/
                if (trn_best_h >= 0)
                    trn_best_score_pct = (100*trn_best_ones + (trn_bits_observed/2))/trn_bits_observed;
                /*endif*/
            }
            /*endif*/
            s->received_event = V34_EVENT_PHASE4_TRN_READY;
            if (s->phase4_trn_lock_hyp >= 0
                && s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT
                && s->phase4_trn_lock_domain >= 0 && s->phase4_trn_lock_domain < 2
                && s->phase4_trn_lock_tap >= 0 && s->phase4_trn_lock_tap < 2
                && s->phase4_trn_lock_order >= 0 && s->phase4_trn_lock_order < 2)
            {
                memcpy(s->phase4_trn_scramble,
                       s->phase4_trn_scramble_tap[s->phase4_trn_lock_domain][s->phase4_trn_lock_tap][s->phase4_trn_lock_order],
                       sizeof(s->phase4_trn_scramble));
                memcpy(s->phase4_trn_one_count,
                       s->phase4_trn_one_count_tap[s->phase4_trn_lock_domain][s->phase4_trn_lock_tap][s->phase4_trn_lock_order],
                       sizeof(s->phase4_trn_one_count));
                s->scrambler_tap = phase4_trn_tap_value(s->phase4_trn_lock_tap);
                s->phase3_j_lock_hyp = s->phase4_trn_lock_hyp;
            }
            else if (trn_best_h >= 0  &&  trn_best_h < MP_HYPOTHESIS_COUNT)
            {
                /* Use final TRN best hypothesis at MP handoff. */
                s->phase4_trn_lock_hyp = v34_trn_hint_enabled() ? trn_best_h : -1;
                s->phase4_trn_lock_score = v34_trn_hint_enabled() ? trn_best_score_pct : -1;
                s->phase4_trn_lock_domain = trn_best_domain;
                s->phase4_trn_lock_tap = trn_best_tap;
                s->phase4_trn_lock_order = trn_best_order;
                s->phase3_j_lock_hyp = trn_best_h;
                if (trn_best_domain >= 0 && trn_best_domain < 2
                    && trn_best_tap >= 0 && trn_best_tap < 2
                    && trn_best_order >= 0 && trn_best_order < 2)
                {
                    memcpy(s->phase4_trn_scramble,
                           s->phase4_trn_scramble_tap[trn_best_domain][trn_best_tap][trn_best_order],
                           sizeof(s->phase4_trn_scramble));
                    memcpy(s->phase4_trn_one_count,
                           s->phase4_trn_one_count_tap[trn_best_domain][trn_best_tap][trn_best_order],
                           sizeof(s->phase4_trn_one_count));
                    s->scrambler_tap = phase4_trn_tap_value(trn_best_tap);
                }
                /*endif*/
            }
            /*endif*/
            /* V.34 §7: call modem uses GPC (tap=17), answer modem uses GPA (tap=4).
               We are receiving from the far end, so use the far end's scrambler:
               - If we are the answerer, far end is caller → tap=17 (GPC)
               - If we are the caller, far end is answerer → tap=4 (GPA)
               V.90 reverses this assignment for its asymmetric upstream/downstream.
               TRN (all-ones) cannot distinguish between the two polynomials because
               a self-synchronizing descrambler produces all-ones output for either tap
               once it converges.  Force the correct tap here. */
            {
                int correct_tap;

                correct_tap = s->v90_mode
                            ? (s->calling_party ? 17 : 4)
                            : (s->calling_party ? 4 : 17);
                /* The premise above -- that TRN cannot tell GPA from GPC -- is
                   wrong.  TRN is scrambled ones, so descrambling it with the
                   wrong polynomial yields a pseudo-random stream (~50% ones),
                   not ones; only the transmitter's own polynomial converges to
                   100%.  Measured against slmodemd's upstream, tap=17 scores
                   99% and tap=4 scores ~54%, i.e. that peer transmits its
                   upstream with GPC where 6.5/V.90 asks for GPA.  When the
                   measurement is that decisive, the wire wins over the role
                   rule -- forcing tap=4 there turned a 100% TRN lock into an
                   MP search that never locked. */
                if (s->scrambler_tap != correct_tap
                    &&
                    s->phase4_trn_lock_score >= 90)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: role rule wants tap=%d but TRN measured tap=%d at %d%% ones; keeping the measured tap\n",
                             correct_tap, s->scrambler_tap, s->phase4_trn_lock_score);
                    correct_tap = s->scrambler_tap;
                    s->v90_far_tap_measured = s->scrambler_tap;
                }
                /*endif*/
                if (s->scrambler_tap != correct_tap)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: TRN selected tap=%d but role requires tap=%d (%s receives from %s); correcting\n",
                             s->scrambler_tap, correct_tap,
                             s->calling_party ? "caller" : "answerer",
                             s->calling_party ? "answerer/GPA" : "caller/GPC");
                    s->scrambler_tap = correct_tap;
                }
                /*endif*/
            }
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: far-end J' + TRN confirmed (J'->TRN=%d bauds, best_ones=%d%%), scanning for MP (decoder=hyp24-v2, gated=900)\n",
                     s->phase4_trn_after_j, s->phase4_trn_lock_score);
            s->stage = V34_RX_STAGE_PHASE4_MP;
            s->duration = 0;
            s->bitstream = 0;
            s->bit_count = 0;
            s->mp_seen = 0;
            /* Do NOT clear received_event here — TX needs to see PHASE4_TRN_READY
               to transition from TRN to MP transmission.  The MP timeout at line 5014
               will overwrite with TRAINING_FAILED if MP decoding fails. */
            s->eq_target_mag = 0.0f;  /* Reset so CMA re-seeds with minimum clamp (1.0) */
            s->mp_remote_ack_seen = 0;
            s->mp_signal_settle_bauds = 0;
            s->mp_count = -1;
            s->mp_frame_pos = 0;
            s->mp_frame_target = 0;
            s->mp_early_rejects = 0;
            s->mp_phase4_default_scrambler_tap = s->scrambler_tap;
            s->mp_phase4_default_bit_order = s->mp_phase4_bit_order;
            /* V.34 10.1.3.8 maps TRN directly, so its winning slicer
               domain is normally absolute phase.  MP is generated as in
               10.1.3.3 and is differentially encoded; inheriting TRN's
               absolute domain turns a perfect 100% TRN lock into random MP
               bits.  Preserve the mapping/tap/order evidence, but always
               enter MP in the differential domain. */
            s->mp_phase4_default_domain = 0;
            s->mp_phase4_reject_streak = 0;
            s->mp_phase4_nolock_count = 0;
            s->mp_phase4_alt_tap_active = 0;
            s->mp_phase4_alt_order_active = 0;
            s->mp_phase4_alt_domain_active = 0;
            s->mp_phase4_retry_mode = 0;
            s->mp_phase4_domain = s->mp_phase4_default_domain;
            s->mp_phase4_bit_order = (s->phase4_trn_lock_order == 1) ? 1 : 0;
            s->mp_phase4_default_bit_order = s->mp_phase4_bit_order;
            s->mp_phase4_force_abs_active = 0;
            s->mp_phase4_diff_collapse_streak = 0;
            s->mp_phase4_diff_recover_streak = 0;
            v34_rx_mp_vote_reset(s);
            if (s->phase4_trn_lock_hyp >= 0  &&  s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: TRN final best available (hyp=%d, dom=%s, tap=%d, ord=%s, ones=%d%%), starting MP hypothesis search\n",
                         s->phase4_trn_lock_hyp,
                         v34_rx_phase4_trn_domain_name(s->phase4_trn_lock_domain),
                         phase4_trn_tap_value(s->phase4_trn_lock_tap),
                         v34_rx_phase4_trn_order_name(s->phase4_trn_lock_order),
                         s->phase4_trn_lock_score);
            }
            else if (s->phase3_j_lock_hyp >= 0  &&  s->phase3_j_lock_hyp < MP_HYPOTHESIS_COUNT)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: Phase 3 J lock hint available (hyp=%d, trn=%s), starting MP hypothesis search\n",
                         s->phase3_j_lock_hyp,
                         (s->phase3_j_trn16 < 0) ? "unknown" : (s->phase3_j_trn16 ? "16-point" : "4-point"));
            }
            /*endif*/
            v34_rx_mp_reset_hypothesis_search(s);
            memcpy(s->mp_hyp_scramble, s->phase4_trn_scramble, sizeof(s->mp_hyp_scramble));
            if (s->phase4_trn_lock_hyp >= 0
                && s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT)
            {
                /* A strong TRN ones-lock is useful as a hypothesis/search hint,
                   but it is not sufficient to assert that the current symbols
                   already belong to MP. In practice this pre-lock path tends to
                   consume late-TRN symbols as if they were MP, producing fake
                   all-ones preambles and poisoning frame alignment immediately.
                   Keep the TRN-selected domain/tap/order as the initial search
                   mode, but require an actually observed MP preamble before
                   locking a hypothesis. */
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: TRN hint available but direct MP pre-lock disabled "
                         "(hyp=%d, dom=%s, tap=%d, ord=%s, ones=%d%%); waiting for observed MP preamble\n",
                         s->phase4_trn_lock_hyp,
                         v34_rx_phase4_trn_domain_name(s->phase4_trn_lock_domain),
                         phase4_trn_tap_value(s->phase4_trn_lock_tap),
                         v34_rx_phase4_trn_order_name(s->phase4_trn_lock_order),
                         s->phase4_trn_lock_score);
            }
            /*endif*/
        }
        /*endif*/
        else if (s->phase4_j_seen
                 && s->phase4_trn_after_j >= 512
                 && (s->phase4_trn_after_j % 256) == 0)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: waiting for TRN ones-lock before MP (after_j=%d, lock=%d%% current=%d%%, min=%d); staying in TRN\n",
                     s->phase4_trn_after_j, s->phase4_trn_lock_score, s->phase4_trn_current_score, PHASE4_TRN_READY_MIN_SCORE);
        }
        /*endif*/
        /* Timeout: if TRN lock never achieved within max allowed time, signal failure (once) */
        if (s->phase4_j_seen
            && s->phase4_trn_after_j >= PHASE4_TRN_READY_MAX_BAUD
            && s->phase4_trn_lock_score < PHASE4_TRN_READY_MIN_SCORE
            && s->received_event != V34_EVENT_TRAINING_FAILED)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: TRN timeout (%d bauds after J', lock=%d%% < %d%%); signalling failure\n",
                     s->phase4_trn_after_j, s->phase4_trn_lock_score, PHASE4_TRN_READY_MIN_SCORE);
            s->received_event = V34_EVENT_TRAINING_FAILED;
        }
        /*endif*/
        else if (s->duration >= 5200  &&  (s->duration % 512) == 0)
        {
            /* Do not force MP without explicit J' + TRN confirmation. */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: still waiting for far-end J'/TRN confirmation (%d bauds)\n",
                     s->duration);
        }
        }
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
