/*
 * v90_analogue_phase3.c — the analogue modem's Phase 3, both directions.
 *
 * See the header.  The interesting part of this file is apply_events(), which
 * is the whole of §9.3.2's dependency on the far end.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v90_analogue_phase3.h"

static const int baud_rates[6] = {2400, 2743, 2800, 3000, 3200, 3429};
#define RR_ED_DEADLINE_BASE_SAMPLES     (5000*8)
#define PHASE4_B1D_DEADLINE_BASE_SAMPLES (15000*8)

struct v90_analogue_phase3_s {
    v90_analogue_tx_t *tx;
    double             cp_margin;   /* noise margin the CP was built at */
    v90_analogue_rx_t *rx;
    v34_state_t       *v34;
    bool               owns_v34;

    /*
     * §9.4.  Phase 4's receiver cannot exist until Phase 3 has produced a
     * measurement -- everything after R̄i is mapped with the CPt that
     * measurement implies -- so it is created at the handover rather than up
     * front, and the codeword stream is routed to it from that point.
     */
    v90_law_t              law;
    int                    u_info;
    int                    shaping_redundancy;
    int                    shaping_lookahead;
    double                 digital_max_tx_dbm0;
    v90_analogue_phase4_t *p4;
    vpcm_cp_frame_t        cpt;
    vpcm_cp_frame_t        cp;
    bool                   phase4_started;
    bool                   phase4_failed;
    bool                   startup_data_reached;
    bool                   upstream_data_started;
    int                    upstream_rate_n;
    int                    upstream_max_n;
    int                    round_trip_delay_samples;
    bool                   zero_length_dil;
    uint64_t               rx_samples;
    uint64_t               phase4_deadline;
    uint64_t               rr_deadline;
    bool                   rr_active;

    /* §9.5.2.2 Tone B (1200 Hz) detector on the byte-exact downstream. */
    double                 tone_b_re;
    double                 tone_b_im;
    double                 tone_b_energy;
    uint64_t               tone_b_sample;
    int                    tone_b_block_samples;
    int                    tone_b_good_blocks;
    bool                   tone_b_reported;
};

v90_analogue_phase3_t *v90_analogue_phase3_init(const v90_analogue_phase3_config_t *cfg)
{
    v90_analogue_phase3_t *s;
    v90_analogue_tx_config_t txc;
    v90_analogue_rx_config_t rxc;

    if (cfg == NULL  ||  cfg->baud_rate_code < 0  ||  cfg->baud_rate_code > 5)
        return NULL;
    if (cfg->shaping_redundancy < 0  ||  cfg->shaping_redundancy > 3)
        return NULL;
    /* §8.5.2: ld describes look-ahead *during spectral shaping*, so Sr = 0
     * leaves it nothing to describe. */
    if (cfg->shaping_lookahead < 0  ||  cfg->shaping_lookahead > 3
        ||
        (cfg->shaping_redundancy == 0  &&  cfg->shaping_lookahead != 0)) {
        return NULL;
    }
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;

    memset(&txc, 0, sizeof(txc));
    txc.baud_rate_code = cfg->baud_rate_code;
    txc.md_units = cfg->md_units;
    txc.dil = cfg->dil;
    txc.scr_during_dil = cfg->scr_during_dil;

    memset(&rxc, 0, sizeof(rxc));
    rxc.law = cfg->law;
    rxc.u_info = cfg->u_info;
    rxc.dil = cfg->dil;
    rxc.dil_coverage = cfg->dil_coverage;
    rxc.zero_slot_fraction = cfg->zero_slot_fraction;
    rxc.w_slot_tolerance = cfg->w_slot_tolerance;

    s->law = cfg->law;
    s->u_info = cfg->u_info;
    s->shaping_redundancy = cfg->shaping_redundancy;
    s->shaping_lookahead = cfg->shaping_lookahead;
    s->digital_max_tx_dbm0 = cfg->digital_max_tx_dbm0;
    s->upstream_max_n = cfg->upstream_max_n;
    s->round_trip_delay_samples = cfg->round_trip_delay_samples > 0
                                ? cfg->round_trip_delay_samples : 0;
    s->zero_length_dil = (cfg->dil.n == 0);
    /* §9.4.2 measures 15 s + five RTDs from INFO1a.  This object is created
     * at the immediately following Phase-3 seam. */
    s->phase4_deadline = PHASE4_B1D_DEADLINE_BASE_SAMPLES
                       + 5U*(uint64_t)s->round_trip_delay_samples;

    s->tx = v90_analogue_tx_init(&txc);
    s->rx = v90_analogue_rx_init(&rxc);
    if (s->tx == NULL  ||  s->rx == NULL) {
        v90_analogue_phase3_free(s);
        return NULL;
    }

    /*
     * The V.34 modem here is a modulator and nothing else: its transmit state
     * machine and its receiver are both out of the picture, because the
     * analogue role's Phase 3 sequencing comes from the PCM downstream.  The
     * bit rate only has to pair validly with the symbol rate.
     */
    if (cfg->v34) {
        s->v34 = cfg->v34;
        s->owns_v34 = false;
    } else {
        s->v34 = v34_init(NULL, baud_rates[cfg->baud_rate_code], 28800, true, true,
                          NULL, NULL, NULL, NULL);
        s->owns_v34 = true;
    }
    if (s->v34 == NULL
        ||
        v34_tx_start_external_symbols(s->v34, cfg->baud_rate_code,
                                      cfg->high_carrier,
                                      v90_analogue_tx_get_symbol, s->tx) != 0) {
        v90_analogue_phase3_free(s);
        return NULL;
    }
    return s;
}

void v90_analogue_phase3_free(v90_analogue_phase3_t *s)
{
    if (s == NULL)
        return;
    if (s->v34  &&  s->owns_v34)
        v34_free(s->v34);
    v90_analogue_tx_free(s->tx);
    v90_analogue_rx_free(s->rx);
    v90_analogue_phase4_free(s->p4);
    free(s);
}

/*
 * §9.3.2's four conditional moments, and nothing else.
 *
 * Each transmitter entry point is a no-op unless the transmitter is in the
 * stage that signal is supposed to end, so an event that arrives early, twice,
 * or out of order cannot skip a stage — which matters because a real
 * downstream repeats Jd until it sees our S, so V90A_RX_EVENT_JD arrives once
 * per repetition.
 */
static void apply_events(v90_analogue_phase3_t *s, unsigned events)
{
    if (events & V90A_RX_EVENT_SD_BAR)
        v90_analogue_tx_sd_bar_seen(s->tx);        /* §9.3.2.4: end Ja */
    if (events & V90A_RX_EVENT_JD)
        v90_analogue_tx_jd_seen(s->tx);            /* §9.3.2.7: start S */
    if (events & V90A_RX_EVENT_JD_PRIME)
        v90_analogue_tx_jd_prime_seen(s->tx);      /* §9.3.2.8: S̄ for 16T */
    if (events & V90A_RX_EVENT_DIL_ENOUGH)
        v90_analogue_tx_dil_enough(s->tx);         /* §9.3.2.10: S, then S̄ */
}

/*
 * §9.4.2's conditional moments.  Same rule as apply_events(): each entry point
 * is a no-op outside the stage it ends, so the digital modem repeating MP
 * until it sees CP cannot push the transmitter past CP'.
 */
static void apply_phase4_events(v90_analogue_phase3_t *s, unsigned events)
{
    if (events & V90A4_RX_EVENT_RD_BAR) {
        /* V.90 §9.6.2.2.2: replace V.34 data at the preserved symbol seam
         * with S/S̄/CP.  The next B1 handoff will seed the newly received MP. */
        if (v90_analogue_tx_rate_renegotiate(s->tx)
            && v34_v90_resume_external_symbols(s->v34,
                                                v90_analogue_tx_get_symbol,
                                                s->tx) == 0) {
            s->upstream_data_started = false;
            s->upstream_rate_n = 0;
            s->rr_active = true;
            s->rr_deadline = 0; /* armed at the S-to-S-bar transition */
        } else {
            s->phase4_failed = true;
        }
    }
    if (events & V90A4_RX_EVENT_RT_BAR)
        v90_analogue_tx_rt_transition_seen(s->tx); /* §9.6.2.1.8 */
    if (events & V90A4_RX_EVENT_R_BAR)
        v90_analogue_tx_r_transition_seen(s->tx);  /* §9.4.2.2: end CPt */
    if (events & V90A4_RX_EVENT_MP)
        v90_analogue_tx_mp_seen(s->tx);            /* §9.4.2.3: CP' */
    /* §9.4.2.4 accepts either MP' or Ed as the cue for E. */
    if (events & (V90A4_RX_EVENT_MP_PRIME | V90A4_RX_EVENT_ED))
        v90_analogue_tx_mp_prime_seen(s->tx);
}

/*
 * Hand over to §9.4 once §9.3.2.10 is done and a measurement exists.
 *
 * Both conditions matter and neither implies the other: the transmitter can
 * reach V90A_TX_PHASE4 on a zero-length DIL with nothing measured, and a
 * measurement can exist while §9.3.2.10's closing S̄ is still going out.
 */
static void start_phase4(v90_analogue_phase3_t *s)
{
    v90_analogue_phase4_config_t p4c;
    const v90_dil_measurement_t *m;
    int ld;

    if (s->p4 != NULL  ||  s->phase4_failed)
        return;
    if (v90_analogue_tx_stage(s->tx) != V90A_TX_PHASE4)
        return;
    m = v90_analogue_rx_measurement(s->rx);
    if (m == NULL && !s->zero_length_dil)
        return;
    /*
     * §8.5.2: ld "shall be consistent with the capabilities of the digital
     * modem indicated in Jd", whose bits 49:50 are its maximum.  Jd has been
     * decoded by now -- §9.3.2.7 is upstream of here -- so this is the first
     * point where the requirement can actually be met.
     */
    ld = s->shaping_lookahead;
    if (s->shaping_redundancy > 0) {
        const uint8_t *jd = v90_analogue_rx_jd_bits(s->rx);

        if (jd != NULL) {
            int max_ld = (jd[49] & 1) | ((jd[50] & 1) << 1);

            if (ld > max_ld)
                ld = max_ld;
            /*endif*/
        }
        /*endif*/
    }
    /*endif*/
    if (m != NULL) {
        if (!v90_analogue_phase4_build_cp(m, s->law, s->digital_max_tx_dbm0,
                                          s->shaping_redundancy, ld,
                                          &s->cpt, &s->cp, &s->cp_margin)) {
            s->phase4_failed = true;
            return;
        }
    } else if (!v90_analogue_phase4_build_zero_dil_cp(
                   s->law, s->shaping_redundancy, ld, &s->cpt, &s->cp)) {
        s->phase4_failed = true;
        return;
    }
    memset(&p4c, 0, sizeof(p4c));
    p4c.law = s->law;
    p4c.u_info = s->u_info;
    p4c.cpt = s->cpt;
    p4c.cp = s->cp;
    if ((s->p4 = v90_analogue_phase4_init(&p4c)) == NULL
        ||
        !v90_analogue_tx_start_phase4(s->tx, &s->cpt, &s->cp, false)) {
        v90_analogue_phase4_free(s->p4);
        s->p4 = NULL;
        s->phase4_failed = true;
        return;
    }
    s->phase4_started = true;
}

static bool detect_tone_b(v90_analogue_phase3_t *s,
                          const uint8_t *codewords, int count)
{
    const double omega = 2.0*3.14159265358979323846*1200.0/8000.0;

    for (int i = 0; i < count; i++) {
        double sample = (s->law == V90_LAW_ALAW)
                      ? alaw_to_linear(codewords[i])
                      : ulaw_to_linear(codewords[i]);
        double phase = omega*(double)(s->tone_b_sample++ % 20U);

        s->tone_b_re += sample*cos(phase);
        s->tone_b_im -= sample*sin(phase);
        s->tone_b_energy += sample*sample;
        if (++s->tone_b_block_samples == 80) {
            double ratio = s->tone_b_energy > 0.0
                         ? 2.0*(s->tone_b_re*s->tone_b_re
                               + s->tone_b_im*s->tone_b_im)
                           /(80.0*s->tone_b_energy)
                         : 0.0;

            if (s->tone_b_energy/80.0 > 10000.0 && ratio > 0.65)
                s->tone_b_good_blocks++;
            else
                s->tone_b_good_blocks = 0;
            s->tone_b_re = 0.0;
            s->tone_b_im = 0.0;
            s->tone_b_energy = 0.0;
            s->tone_b_block_samples = 0;
            /* §9.5.2.2 says more than 50 ms. Six 10 ms blocks is strictly
             * greater, and phase reversals do not disturb this energy test. */
            if (s->tone_b_good_blocks >= 6 && !s->tone_b_reported) {
                s->tone_b_reported = true;
                return true;
            }
        }
    }
    return false;
}

unsigned v90_analogue_phase3_rx(v90_analogue_phase3_t *s,
                                const uint8_t *codewords,
                                int count)
{
    unsigned events;
    bool tone_b;

    if (s == NULL)
        return 0;
    s->rx_samples += (uint64_t) count;
    tone_b = detect_tone_b(s, codewords, count);
    if (s->p4 != NULL) {
        /* Phase 3's receiver is finished with this stream; §8.6's signals are
         * not §8.4's and feeding both would only produce noise in one. */
        events = v90_analogue_phase4_put(s->p4, codewords, count);
        apply_phase4_events(s, events);
        if ((events & V90A4_RX_EVENT_DATA) != 0) {
            s->startup_data_reached = true;
            s->rr_active = false;
        }
        if (tone_b)
            events |= V90A_EVENT_TONE_B_RETRAIN;
        return events;
    }
    /*endif*/
    events = v90_analogue_rx_put(s->rx, codewords, count);
    apply_events(s, events);
    start_phase4(s);
    if (tone_b)
        events |= V90A_EVENT_TONE_B_RETRAIN;
    return events;
}

int v90_analogue_phase3_tx(v90_analogue_phase3_t *s, int16_t *amp, int max_len)
{
    v90_analogue_tx_stage_t before;
    int len;

    if (s == NULL  ||  amp == NULL  ||  max_len <= 0)
        return 0;
    before = v90_analogue_tx_stage(s->tx);
    /* V.90 §9.4.2.4-.5: after E, select the highest upstream rate enabled by
     * both CP and the digital modem's MP, then hand the existing modulator to
     * V.34's reset-state B1/data mapper.  Keeping this at the next v34_tx()
     * boundary lets the final E symbol finish in the external source first. */
    if (!s->upstream_data_started
        && v90_analogue_tx_stage(s->tx) == V90A_TX_B1_PENDING
        && s->p4 != NULL) {
        const v90_analogue_mp_t *mp = v90_analogue_phase4_mp(s->p4);
        int n = 0;

        if (mp != NULL) {
            int highest = mp->max_drn;

            if (s->upstream_max_n >= 2 && highest > s->upstream_max_n)
                highest = s->upstream_max_n;
            for (int candidate = highest; candidate >= 2; candidate--) {
                unsigned bit = 1U << (candidate - 2);

                if ((mp->rate_mask & bit)  &&  (s->cp.upstream_rate_mask & bit)) {
                    n = candidate;
                    break;
                }
            }
            if (n > 0
                && v34_v90_begin_tx_data(s->v34, n, mp->trellis,
                                         mp->nonlinear,
                                         mp->expanded_shaping,
                                         &mp->precoder[0][0]) == 0) {
                s->upstream_rate_n = n;
                s->upstream_data_started = true;
            } else {
                s->phase4_failed = true;
            }
        }
    }
    len = v34_tx(s->v34, amp, max_len);
    if (s->rr_active && s->rr_deadline == 0
        && before == V90A_TX_RR_S
        && v90_analogue_tx_stage(s->tx) != V90A_TX_RR_S) {
        /* §9.6.2: Ed is due within 5000 ms plus two RTDs after S-to-S-bar. */
        s->rr_deadline = s->rx_samples + RR_ED_DEADLINE_BASE_SAMPLES
                       + 2U*(uint64_t)s->round_trip_delay_samples;
    }
    return len;
}

v90_analogue_tx_stage_t v90_analogue_phase3_tx_stage(const v90_analogue_phase3_t *s)
{
    return s ? v90_analogue_tx_stage(s->tx) : V90A_TX_PHASE4;
}

v90_analogue_rx_stage_t v90_analogue_phase3_rx_stage(const v90_analogue_phase3_t *s)
{
    return s ? v90_analogue_rx_stage(s->rx) : V90A_RX_HUNT_SD;
}

const v90_analogue_tx_t *v90_analogue_phase3_tx_state(const v90_analogue_phase3_t *s)
{
    return s ? s->tx : NULL;
}

const v90_analogue_rx_t *v90_analogue_phase3_rx_state(const v90_analogue_phase3_t *s)
{
    return s ? s->rx : NULL;
}

bool v90_analogue_phase3_complete(const v90_analogue_phase3_t *s)
{
    /* Once CPt is armed the transmitter immediately leaves the boundary
     * stage; Phase 3 remains complete throughout Phase 4. */
    return s && (s->phase4_started
                 || v90_analogue_tx_stage(s->tx) == V90A_TX_PHASE4);
}

bool v90_analogue_phase3_retrain_due(const v90_analogue_phase3_t *s)
{
    return s  &&  v90_analogue_tx_deadline_passed(s->tx);
}

const v90_dil_measurement_t *v90_analogue_phase3_measurement(const v90_analogue_phase3_t *s)
{
    return s ? v90_analogue_rx_measurement(s->rx) : NULL;
}

const v90_analogue_phase4_t *v90_analogue_phase3_phase4_state(const v90_analogue_phase3_t *s)
{
    return s ? s->p4 : NULL;
}

bool v90_analogue_phase3_phase4_failed(const v90_analogue_phase3_t *s)
{
    return s  &&  s->phase4_failed;
}

bool v90_analogue_phase3_phase4_retrain_due(const v90_analogue_phase3_t *s)
{
    if (s == NULL)
        return false;
    if (s->phase4_failed)
        return true;
    return s->phase4_started
        && !s->startup_data_reached
        && !v90_analogue_phase3_data_ready(s)
        && s->rx_samples >= s->phase4_deadline;
}

const vpcm_cp_frame_t *v90_analogue_phase3_cpt(const v90_analogue_phase3_t *s)
{
    return (s  &&  s->phase4_started) ? &s->cpt : NULL;
}

const vpcm_cp_frame_t *v90_analogue_phase3_cp(const v90_analogue_phase3_t *s)
{
    return (s  &&  s->phase4_started) ? &s->cp : NULL;
}

bool v90_analogue_phase3_data_ready(const v90_analogue_phase3_t *s)
{
    return s && s->p4
        && v90_analogue_phase4_stage(s->p4) == V90A4_RX_DATA;
}

int v90_analogue_phase3_get_data_bits(v90_analogue_phase3_t *s,
                                      uint8_t *bits, int max_bits)
{
    return (s && s->p4)
        ? v90_analogue_phase4_get_data_bits(s->p4, bits, max_bits) : 0;
}

int v90_analogue_phase3_upstream_rate(const v90_analogue_phase3_t *s)
{
    return s ? s->upstream_rate_n*2400 : 0;
}

bool v90_analogue_phase3_start_rate_renegotiation(v90_analogue_phase3_t *s,
                                                   bool silence_request)
{
    if (s == NULL || s->p4 == NULL || !s->upstream_data_started
        || s->rr_active
        || !v90_analogue_phase4_start_rate_renegotiation(s->p4,
                                                          silence_request)
        || !v90_analogue_tx_start_rate_renegotiation(s->tx, silence_request)
        || v34_v90_resume_external_symbols(s->v34,
                                            v90_analogue_tx_get_symbol,
                                            s->tx) != 0) {
        return false;
    }
    s->upstream_data_started = false;
    s->upstream_rate_n = 0;
    s->rr_active = true;
    s->rr_deadline = 0; /* §9.6.2 starts the timer at S-to-S-bar. */
    return true;
}

bool v90_analogue_phase3_rate_renegotiating(const v90_analogue_phase3_t *s)
{
    return s && s->rr_active;
}

bool v90_analogue_phase3_start_cleardown(v90_analogue_phase3_t *s)
{
    if (s == NULL || s->p4 == NULL || !s->upstream_data_started || s->rr_active
        || !v90_analogue_phase4_start_rate_renegotiation(s->p4, false)
        || !v90_analogue_tx_start_cleardown(s->tx)
        || v34_v90_resume_external_symbols(s->v34,
                                            v90_analogue_tx_get_symbol,
                                            s->tx) != 0) {
        return false;
    }
    s->upstream_data_started = false;
    s->upstream_rate_n = 0;
    s->rr_active = true;
    s->rr_deadline = 0;
    return true;
}

bool v90_analogue_phase3_cleardown_complete(const v90_analogue_phase3_t *s)
{
    return s && v90_analogue_tx_stage(s->tx) == V90A_TX_CLEARDOWN_DONE;
}

bool v90_analogue_phase3_rate_retrain_due(const v90_analogue_phase3_t *s)
{
    return s && s->rr_active && s->rr_deadline != 0
        && s->rx_samples >= s->rr_deadline;
}
