#include "v92_analogue_phase4.h"
#include "v90_analogue_linear.h"
#include "v92_trn2u.h"
#include "v92_upstream_data.h"
#include "v91.h"
#include "v90_dil_measure.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct v92a4_s {
    v92a4_stage_t stage;
    const char *failure;
    v90_analogue_phase4_t *rx;
    v90a_linear_t *linear;
    v92_trn2u_tx_t tx;
    v92_upstream_wave_tx_t data;
    v92_cp_frame_t cpu;
    v92_cpd_frame_t cpd;
    uint32_t rate_mask;
    bool suvd, cpd_seen, remote_ack, ack_sent, cpu_sent, downstream, cpu_retry;
    /* 9.6.2.1.3: once the unacknowledged initial CPu has timed out the
     * transmitter stays in repeated-CPu mode until E; ed_seen is 9.6.2.1.4's
     * alternative to an acknowledged CPd/SUVd. */
    bool cpu_repeat, ed_seen;
    unsigned symbols, stage_symbols, cpu_end, round_trip_symbols;
    bool peer_cleardown, cpd_ack_seen;
    unsigned cleardown_at;            /* 0 = not scheduled */
    int alignment, window_len;
    uint8_t window[V92_CPD_MAX_BITS];
    uint8_t bits[V92_CP_RX_MAX_BITS];
    int bit_pos, bit_count;
    double frame[12];
    double lu;                        /* 3.8: the data-mode transmit level */
    /* TRN2d decision noise (G.711 linear), for the CPu design. */
    double trn2d_err2;
    unsigned trn2d_errs;
    /* Downstream noise as the codec leaves it: codeword flips off the
     * mapping grid, and how many decisions could have shown one in each
     * direction (see v90_dil_codec_flip_sigma()).  Kept across a 9.8 renegotiation, so
     * the data mode before Rd counts towards the new CPu. */
    double flip_up[128], flip_down[128];
    unsigned flips;
    bool cpu_designed;
    vpcm_cp_frame_t cp;               /* the data-mode CP, as CPu offers it */
    v91_law_t law;
    int (*get_bit)(void *);
    void *data_user;
    /* V.92 9.8.2.2: answering the digital modem's rate renegotiation. */
    bool rn_due;                      /* R-bar-d seen: Ru at the next frame boundary */
    bool rn;                          /* from Ru to the new B1u */
    int renegotiations;
};

static void fail(v92a4_t *s, const char *why)
{
    s->failure = why;
    s->stage = V92A4_FAILED;
}

/* Messages begin and end on downstream mapping-frame boundaries (8.8).
 * Keep candidate starts until a CRC-valid frame arrives; a corrupt frame
 * cannot hide a later valid one by supplying a fictitious length. */
static bool control(void *user, const uint8_t *bits, int n)
{
    v92a4_t *s = user;
    if (s->window_len + n > (int)sizeof(s->window)) {
        memmove(s->window, s->window+n, s->window_len-n);
        s->window_len -= n;
    }
    memcpy(s->window+s->window_len, bits, n);
    s->window_len += n;
    for (int start = 0; start + 52 <= s->window_len; start += s->alignment) {
        const uint8_t *b = s->window+start;
        int have = s->window_len-start;
        bool sync = !b[17];
        for (int i = 0; i < 17; i++) sync &= b[i] == 1;
        if (!sync) continue;
        if (b[18]) {
            v92_suvd_frame_t f;
            v92_suvd_diag_t d;
            if (have != v92_suvd_bit_length(s->alignment)
                || !v92_suvd_decode_bits(b, have, &f, &d)) continue;
            if (f.silent_period_requested) {
                fail(s, s->rn ? "SUVd silent-period request (9.8.1.1.3) is unsupported"
                              : "SUVd silent-period request is unsupported");
                return false;
            }
            s->suvd = true;
            s->remote_ack |= f.acknowledge;
        } else {
            v92_cpd_frame_t f;
            v92_cpd_diag_t d;
            if (!v92_cpd_decode(b, have, &f, &d)) continue;
            if (f.acknowledge)
                s->cpd_ack_seen = true;
            if (!f.selected_upstream_drn) {
                /* 9.11: the digital modem is clearing down. */
                s->peer_cleardown = true;
                if (!s->cpd_seen && s->cpu_sent)
                    s->cpu_retry = true;      /* our CPu so far lacked the ack */
                s->cpd = f;
                s->cpd_seen = true;
                s->remote_ack |= f.acknowledge;
                s->window_len = 0;
                return false;
            }
            if (f.selected_upstream_drn > 19
                || !(s->rate_mask & (1u << (f.selected_upstream_drn-1)))
                || !v92_upstream_wave_profile_validate(&f)) {
                fail(s, "CPd selects an unsupported upstream profile");
                return false;
            }
            s->cpd = f;
            s->cpd_seen = true;
            s->remote_ack |= f.acknowledge;
        }
        if (!s->remote_ack && s->cpu_sent
            && s->symbols-s->cpu_end >= 800ULL + s->round_trip_symbols)
            s->cpu_retry = s->cpu_repeat = true;
        s->window_len = 0;
        /* Arm the Ed detector on any complete CPd once its parameters are
         * known: an acknowledged CPd/SUVd may be lost on the way back while
         * the peer, having accepted ours, sends Ed (9.6.2.1.4). */
        return s->cpd_seen;
    }
    return false;
}

v92a4_t *v92a4_init(const v90_analogue_phase4_config_t *cfg,
                    int points, double lu, uint32_t rate_mask, unsigned round_trip_symbols)
{
    if (!cfg || (points != 4 && points != 8) || !rate_mask
        || !isfinite(lu) || lu <= 0 || lu > 12000) return NULL;
    v92a4_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->rx = v90_analogue_phase4_init(cfg);
    s->linear = v90a_linear_init(cfg->law);
    if (!s->rx || !s->linear) { v92a4_free(s); return NULL; }
    s->alignment = cfg->cpt.drn+8;
    s->lu = lu;
    s->cp = cfg->cp;
    s->law = (v91_law_t)cfg->law;
    s->rate_mask = rate_mask;
    s->round_trip_symbols = round_trip_symbols;
    v90_analogue_phase4_set_control_receiver(s->rx, control, s);
    uint8_t cw = v91_ucode_to_codeword((v91_law_t)cfg->law, cfg->u_info, true);
    v90a_linear_set_reference(s->linear, cfg->u_info,
                              v91_codeword_to_linear((v91_law_t)cfg->law, cw));
    v92_trn2u_tx_init(&s->tx, points, lu, cfg->law == V90_LAW_ALAW);
    s->cpu.type = V92_CP_TYPE_CPU;
    s->cpu.codec_alaw = cfg->law == V90_LAW_ALAW;
    s->cpu.drn = cfg->cp.drn;
    s->cpu.trn1d_gain_q3_13 = cfg->cp.trn1d_gain_q3_13;
    s->cpu.constellation_count = cfg->cp.constellation_count;
    s->cpu.shaping_redundancy = cfg->cp.shaping_redundancy;
    s->cpu.shaping_lookahead = cfg->cp.shaping_lookahead;
    s->cpu.shaping_a1_q1_6 = cfg->cp.shaping_a1_q1_6;
    s->cpu.shaping_a2_q1_6 = cfg->cp.shaping_a2_q1_6;
    s->cpu.shaping_b1_q1_6 = cfg->cp.shaping_b1_q1_6;
    s->cpu.shaping_b2_q1_6 = cfg->cp.shaping_b2_q1_6;
    memcpy(s->cpu.dfi, cfg->cp.dfi, sizeof(s->cpu.dfi));
    memcpy(s->cpu.masks, cfg->cp.masks, sizeof(s->cpu.masks));
    return s;
}
void v92a4_free(v92a4_t *s)
{
    if (!s) return;
    v90_analogue_phase4_free(s->rx);
    v90a_linear_free(s->linear);
    free(s);
}
void v92a4_start(v92a4_t *s, int sign)
{
    if (!s || s->stage != V92A4_WAIT) return;
    v92_trn2u_tx_start(&s->tx, sign);
    s->stage = V92A4_TRN;
}
/*
 * V.92 9.6.2: CPu follows TRN2d, so design it from TRN2d.  The CP that came
 * out of Phase 3 was planned from the DIL, which holds each level for a
 * whole segment: the noise that a receiver adds on RANDOM symbols (its
 * equaliser's data-dependent misadjustment) never shows there.  TRN2d is
 * random mapped symbols, so its decision noise is the data-mode figure.
 * Measured in the reconstructed-audio pair: DIL planned A-law levels 16
 * apart at 3 sigma, and B1d then sliced 136 as 152.  Thin each constellation
 * to 3 sigma to each decision boundary (build_cp()'s second rung; the DIL
 * plan already took the first), then take drn down until 5.4.3's
 * 2^K <= prod(Mi) holds.  Behind a codec the noise comes from v90_dil_codec_flip_sigma()
 * and the boundary from v90_dil_codec_pair_margin() (v90_analogue_phase4_thin_cp()).  Thinning keeps
 * CP a subset of the Phase 3 CP, so its power and CPt rules still hold.
 */
#define V92A4_CPU_SIGMAS     3.0
#define V92A4_CPU_MIN_ERRS   500
/* Decisions that could show a flip before the flip sigma is believed. */
#define V92A4_FLIP_MIN_EXPOSURE 2000.0

/*
 * The noise that reaches us through a G.711 codec.  When the analogue loop's
 * noise is requantised before we see it (an ATA, or engine_pair_test's
 * --loop-noise-db), every received sample IS a codec level: a decision on a
 * grid point reads e = 0 exactly, and the noise shows only as a whole
 * codeword moved to its neighbour.  TRN2d cannot show that.  §8.6.5 maps it
 * through CPt, which sits on loud points whose codec step is many times the
 * noise; measured at -50 dB of loop noise, 20,202 TRN2d symbols gave zero
 * flips and e = 0.0, while B1d's d9 came back as db.  What does show it is
 * every decision on a quiet point -- the data mode before a 9.8 Rd most of
 * all.
 *
 * A decision on Ucode u can show a flip upwards only if u+1 is off the grid
 * it was sliced on (a flip onto a grid point is decided as that point), and
 * then with probability Q(h/sigma), h the half-step to u+1.  Counting those
 * opportunities per Ucode and solving sum count x Q(h/sigma) = flips for
 * sigma is the moment estimate of the noise before the codec.  No flips
 * means nothing to thin for, and gives 0.
 */
static void design_cpu(v92a4_t *s)
{
    vpcm_cp_frame_t cp = s->cp;
    double sigma = 0.0, codec_sigma;
    bool codec;
    int removed;

    s->cpu_designed = true;
    if (s->trn2d_errs >= V92A4_CPU_MIN_ERRS)
        sigma = sqrt(s->trn2d_err2/s->trn2d_errs);
    codec_sigma = v90_dil_codec_flip_sigma((v90_law_t)s->law, s->flip_up, s->flip_down,
                                           (double)s->flips, V92A4_FLIP_MIN_EXPOSURE);
    /* The codec figure governs only where it is the larger: on a linear
     * input the decision error already holds all of the noise. */
    codec = codec_sigma > sigma;
    if (codec)
        sigma = codec_sigma;
    fprintf(stderr, "[V92A] 9.6.2 CPu design: TRN2d sigma %.2f over %u symbols, "
            "codec-flip sigma %.2f (%u flips)%s\n",
            s->trn2d_errs ? sqrt(s->trn2d_err2/s->trn2d_errs) : 0.0, s->trn2d_errs,
            codec_sigma, s->flips, codec ? "; codec thresholds" : "");
    if (sigma <= 0.0)
        return;
    removed = v90_analogue_phase4_thin_cp(&cp, (v90_law_t)s->law, sigma,
                                          V92A4_CPU_SIGMAS, codec);
    /* Never offer less than the line can carry at all: keep the Phase 3 CP
     * if thinning leaves nothing usable. */
    if (removed < 0 || !v90_analogue_phase4_set_cp(s->rx, &cp))
        return;
    fprintf(stderr, "[V92A] 9.6.2 CPu: %d constellation points removed at %.1f sigma, "
            "drn %u -> %u\n", removed, V92A4_CPU_SIGMAS,
            (unsigned)s->cp.drn, (unsigned)cp.drn);
    s->cp = cp;
    s->cpu.drn = cp.drn;
    memcpy(s->cpu.masks, cp.masks, sizeof(s->cpu.masks));
}

static bool message(v92a4_t *s, bool cpu)
{
    s->bit_pos = 0;
    s->stage_symbols = 0;
    s->stage = cpu ? V92A4_CP : V92A4_SUV;
    if (cpu) {
        if (!s->cpu_designed && !s->cpu_sent && s->cpu.drn != 0)
            design_cpu(s);
        s->cpu_retry = false;
        s->cpu.acknowledge = s->cpd_seen;
        return v92_cp_encode(&s->cpu, s->tx.constellation_points,
                             s->bits, sizeof(s->bits), &s->bit_count);
    }
    v92_suvu_frame_t suv = {.acknowledge = s->cpd_seen, .prefilter_level_q2_2 = 16};
    return v92_suvu_encode(&suv, s->tx.constellation_points,
                            s->bits, sizeof(s->bits), &s->bit_count);
}
static int16_t sample(v92a4_t *s)
{
    int16_t out = 0;
    if (s->stage == V92A4_CLEARDOWN) return 0;
    if (s->cleardown_at && s->symbols >= s->cleardown_at) {
        s->stage = V92A4_CLEARDOWN;
        return 0;
    }
    int bps = v92_trn2u_bits_per_symbol(s->tx.constellation_points);
    if (s->stage == V92A4_DATA && s->rn_due && s->stage_symbols%12 == 0) {
        /* 9.8.2.2.2: Ru begins on the boundary of a data frame. */
        s->rn_due = false;
        s->rn = true;
        s->stage = V92A4_RU;
        s->stage_symbols = 0;
        fprintf(stderr, "[V92A] 9.8.2.2.2: Ru (384T) and R-bar-u (24T)\n");
    }
    if (s->stage == V92A4_RU) {
        /* V.92 8.5.5: Ru repeats {+LU,+LU,+LU,-LU,-LU,-LU}; R-bar-u is its
         * negation, for 24T. */
        unsigned i = s->stage_symbols;
        bool positive = (i%6) < 3;

        if (i >= 384)
            positive = !positive;
        out = (int16_t)lround(positive ? s->lu : -s->lu);
        if (++s->stage_symbols == 408) {
            /* 9.8.2.2.3 / 8.7.6 (Amd.1 item 3): the first renegotiation
             * TRN2u resets the scrambler and differential encoder. */
            v92_trn2u_tx_start_context(&s->tx, V92_TRN2U_RENEG_FIRST, 0);
            s->stage = V92A4_TRN;
            s->stage_symbols = 0;
        }
        s->symbols++;
        return out;
    }
    /* 9.8.2.2.3: TRN2u may end after 2400T once SUVd is in (9.6.2.1.1's
     * startup TRN2u runs to 12000T without one). */
    if (s->stage == V92A4_TRN && s->stage_symbols%12 == 0
        && (s->rn ? (s->stage_symbols >= 2400 && s->suvd)
                  : (s->stage_symbols >= 12000 || s->suvd))) {
        if (!message(s, false)) fail(s, "cannot encode SUVu");
    }
    if (s->stage == V92A4_SUV || s->stage == V92A4_CP) {
        if (s->bit_pos == s->bit_count) {
            /* 9.6.2.1.4: both acknowledgements, after a complete message. */
            s->ack_sent |= s->bits[33] != 0;
            if (s->stage == V92A4_CP) {
                s->cpu_sent = true;
                s->cpu_end = s->symbols;
                /* 9.11 (Amd.1) waits; see v92a4_cleardown(). */
                if (s->cpu.drn == 0 && !s->cleardown_at)
                    s->cleardown_at = s->symbols + 800u + s->round_trip_symbols;
                if (s->peer_cleardown && s->bits[33])
                    s->cleardown_at = s->symbols + 800u + s->round_trip_symbols / 2;
            }
            if ((s->cpu.drn == 0 && s->cpu_sent && s->cpd_ack_seen)
                || (s->cleardown_at && s->symbols >= s->cleardown_at)) {
                s->stage = V92A4_CLEARDOWN;
                return 0;
            }
            if (s->ack_sent && (s->remote_ack || s->ed_seen) && s->cpd_seen
                && s->cpu.drn != 0 && !s->peer_cleardown) {
                s->stage = V92A4_E;
                s->stage_symbols = 0;
            } else if (!message(s, s->suvd && (!s->cpu_sent
                         || s->cpu_retry || s->cpu_repeat))) {
                fail(s, "cannot encode Phase-4 control message");
            }
        }
    }
    if (s->stage == V92A4_TRN) {
        v92_trn2u_tx_ones_linear(&s->tx, &out, 1);
    } else if (s->stage == V92A4_SUV || s->stage == V92A4_CP) {
        v92_trn2u_tx_bits_linear(&s->tx, s->bits+s->bit_pos, bps, &out, 1);
        s->bit_pos += bps;
    } else if (s->stage == V92A4_E) {
        if (s->stage_symbols == 12u + s->cpd.extend_e2u) {
            v92_upstream_wave_tx_init(&s->data); /* 8.7.1: reset all memories */
            /* Test hook: send B1u and data like a peer that departs from
             * 6.4.2 (V92_UPSTREAM_QUIRK_* bits). */
            {
                /* Test hook: transmit like a peer departing from 6.4.2 (ESS). */
                const char *q = getenv("V92_TEST_UPSTREAM_QUIRKS");

                s->data.quirks = (q  &&  *q)  ?  atoi(q)  :  0;
                if (s->data.quirks)
                    fprintf(stderr, "[V92A] test: upstream quirks %d (V92_TEST_UPSTREAM_QUIRKS)\n",
                            s->data.quirks);
            }
            s->stage = V92A4_B1;
            s->stage_symbols = 0;
            return sample(s);
        }
        uint8_t zeros[3] = {0};
        v92_trn2u_tx_bits_linear(&s->tx, zeros, bps, &out, 1);
    } else if (s->stage == V92A4_B1 || s->stage == V92A4_DATA) {
        if (s->stage == V92A4_B1 && s->stage_symbols == 48*12) {
            s->stage = V92A4_DATA;
            if (s->rn) {
                s->rn = false;
                fprintf(stderr, "[V92A] 9.8 rate renegotiation %d: B1u sent, data at "
                        "upstream drn %u\n", s->renegotiations,
                        (unsigned)s->cpd.selected_upstream_drn);
            }
        }
        int pos = s->stage_symbols%12;
        if (!pos) {
            uint8_t ones[V92_UPSTREAM_MAX_FRAME_BITS];
            memset(ones, 1, sizeof(ones));
            if (s->stage == V92A4_DATA && s->get_bit) {
                int count = v92_upstream_bits_per_frame(s->cpd.selected_upstream_drn);
                for (int i = 0; i < count; i++) ones[i] = s->get_bit(s->data_user) & 1;
            }
            if (!v92_upstream_wave_encode_frame(&s->data, &s->cpd, ones,
                     v92_upstream_bits_per_frame(s->cpd.selected_upstream_drn), s->frame)) {
                fail(s, "cannot encode B1u"); return 0;
            }
        }
        /* Table 30: G x v at mean square 1 is the desired power, which
         * 3.8 makes LU's, so the line carries LU x G x v. */
        double v = s->lu*s->frame[pos];
        if (!isfinite(v) || v < -32768 || v > 32767) {
            fail(s, "upstream waveform exceeds linear PCM range"); return 0;
        }
        out = (int16_t)lround(v);
    }
    s->stage_symbols++;
    s->symbols++;
    return out;
}
int v92a4_tx(v92a4_t *s, int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return 0;
    for (int i = 0; i < count; i++) samples[i] = sample(s);
    return count;
}
/* One decision on the grid `set` towards the flip sigma: the decided Ucode,
 * the Ucode the full ladder puts the sample on, and which neighbours were
 * off the grid (only a flip onto one of those can be seen). */
static void count_flip(v92a4_t *s, int16_t sample, const uint8_t *set, int n)
{
    bool on[128] = {false};
    double mag = fabs((double)sample), dec = fabs(v90a_linear_last_decision(s->linear));
    double best = INFINITY;
    int ud = -1, ur = 0;

    for (int i = 0; i < n; i++) {
        double d = fabs(v90a_linear_ucode_level(s->linear, set[i]) - dec);

        on[set[i] & 0x7F] = true;
        if (d < best) { best = d; ud = set[i] & 0x7F; }
    }
    if (ud < 0)
        return;
    best = INFINITY;
    for (int u = 0; u < 128; u++) {
        double d = fabs(v90a_linear_ucode_level(s->linear, u) - mag);

        if (d < best) { best = d; ur = u; }
    }
    if (ud < 127 && !on[ud+1]) s->flip_up[ud] += 1.0;
    if (ud > 0 && !on[ud-1]) s->flip_down[ud] += 1.0;
    if (ur != ud)
        s->flips++;
}

void v92a4_rx(v92a4_t *s, const int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return;
    for (int i = 0; i < count; i++) {
        uint8_t cw, ucodes[128];
        int n = v90_analogue_phase4_slicer_ucodes(s->rx, ucodes, sizeof(ucodes));
        v90a_linear_set_constellation(s->linear, ucodes, n);
        if (v90a_linear_put(s->linear, samples+i, 1, &cw, 1) != 1) continue;
        if (v90_analogue_phase4_stage(s->rx) == V90A4_RX_TRN2D) {
            double e = samples[i] - v90a_linear_last_decision(s->linear);

            s->trn2d_err2 += e*e;
            s->trn2d_errs++;
        }
        if (n > 0 && v90a_linear_locked(s->linear))
            count_flip(s, samples[i], ucodes, n);
        unsigned e = v90_analogue_phase4_put(s->rx, &cw, 1);
        if (e) fprintf(stderr, "P4RX event=%x\n", e);
        if (e & V90A4_RX_EVENT_ED)
            s->ed_seen = true;
        if ((e & V90A4_RX_EVENT_RD_BAR) && s->stage == V92A4_DATA && !s->rn) {
            /* 9.8.2.2.1-.2: Rd, then its transition.  Everything the 9.6
             * exchange latched starts again; the receiver has already moved
             * on to the renegotiation TRN2d. */
            s->rn_due = true;
            s->renegotiations++;
            s->suvd = s->cpd_seen = s->remote_ack = s->ack_sent = false;
            s->cpu_sent = s->cpu_retry = s->cpu_repeat = s->ed_seen = false;
            s->cpd_ack_seen = s->cpu_designed = false;
            s->trn2d_err2 = 0.0;
            s->trn2d_errs = 0;
            s->window_len = 0;
            s->downstream = false;
            fprintf(stderr, "[V92A] 9.8.2.2.1: Rd-to-R-bar-d transition (renegotiation %d)\n",
                    s->renegotiations);
        }
        if (e & V90A4_RX_EVENT_DATA) {
            if (v90_analogue_phase4_b1d_bit_errors(s->rx)) fail(s, "B1d validation failed");
            else s->downstream = true;
        }
    }
}
v92a4_stage_t v92a4_stage(const v92a4_t *s) { return s ? s->stage : V92A4_FAILED; }
int v92a4_renegotiations(const v92a4_t *s) { return s ? s->renegotiations : 0; }
bool v92a4_renegotiating(const v92a4_t *s) { return s && (s->rn || s->rn_due); }
bool v92a4_downstream_ready(const v92a4_t *s) { return s && s->downstream; }
const v92_cpd_frame_t *v92a4_cpd(const v92a4_t *s) { return s && s->cpd_seen ? &s->cpd : NULL; }
bool v92a4_cleardown(const v92a4_t *s) { return s && s->stage == V92A4_CLEARDOWN; }
const char *v92a4_failure(const v92a4_t *s) { return s ? s->failure : "invalid configuration"; }
double v92a4_rx_decision(const v92a4_t *s) { return s ? v90a_linear_last_decision(s->linear) : 0; }
double v92a4_rx_tolerance(const v92a4_t *s) { return s ? v90a_linear_last_tolerance(s->linear) : 0; }

void v92a4_set_data_source(v92a4_t *s, int (*get_bit)(void *), void *user)
{
    if (!s) return;
    s->get_bit = get_bit;
    s->data_user = user;
}
int v92a4_get_data_bits(v92a4_t *s, uint8_t *bits, int capacity)
{
    return s ? v90_analogue_phase4_get_data_bits(s->rx, bits, capacity) : 0;
}
int v92a4_downstream_rate(const v92a4_t *s)
{
    return s ? (int)vpcm_cp_drn_to_bps(s->cpu.drn) : 0;
}

double v92a4_trn2d_sigma(const v92a4_t *s)
{
    return s && s->trn2d_errs ? sqrt(s->trn2d_err2/s->trn2d_errs) : 0.0;
}
