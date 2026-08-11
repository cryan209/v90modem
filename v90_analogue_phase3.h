/*
 * v90_analogue_phase3.h — the analogue modem's Phase 3, both directions.
 *
 * v90_analogue_tx.c emits the §9.3.2 signals and v90_analogue_rx.c reads the
 * §8.4 ones; this is the couple of dozen lines that make them one modem.  The
 * whole of §9.3.2's conditional structure lives in that join: Ja ends on the
 * Sd-to-S̄d transition, S starts on Jd, S̄ on J'd, and §9.3.2.10's S/S̄ pair on
 * having enough DIL.  Nothing else crosses between them.
 *
 * It also owns the V.34 modulator the transmitter feeds, so a caller deals in
 * G.711 codewords one way and linear samples the other, at 8 kHz both ways.
 */
#ifndef V90_ANALOGUE_PHASE3_H
#define V90_ANALOGUE_PHASE3_H

#include <stdbool.h>
#include <stdint.h>

#include <spandsp.h>

#include "v90_analogue_phase4.h"
#include "v90_analogue_rx.h"
#include "v90_analogue_tx.h"

typedef struct v90_analogue_phase3_s v90_analogue_phase3_t;

typedef struct {
    v90_law_t law;
    int       baud_rate_code;   /* INFO1a bits 34:36 — upstream symbol rate */
    bool      high_carrier;     /* as INFO1d directed (§8.2.3.2, Table 9) */
    int       u_info;           /* INFO1a bits 25:31 — this side chose it */
    /* Phase 2 RTDEa in 8000 Hz samples, used by §9.4.2 and §9.6.2 deadlines. */
    int       round_trip_delay_samples;
    int       md_units;         /* INFO1a bits 18:24, 35 ms units; 0 skips MD */
    v90_dil_desc_t dil;         /* the descriptor to carry in Ja; n == 0 = none */
    bool      scr_during_dil;   /* §9.3.2.9: SCR rather than silence */
    double    dil_coverage;     /* §9.3.2.10 stopping rule; 0 = default */
    /* INFO0d bits 33:37, in dBm0, for §8.5.2/Table 15.  Zero means the
     * capability was unavailable (the offline-test/unbounded case). */
    double    digital_max_tx_dbm0;
    /*
     * §5.4.5's Sr, carried into Phase 4's CPt and CP: 0 disables spectral
     * shaping, 1 to 3 spend that many of the six sign bits on it.  This side
     * chooses it, and it moves the rate both ways (§5.4.1), so it is settled
     * here rather than in the CP builder.  `shaping_lookahead` is Table 14's
     * ld and must not exceed the digital modem's Jd bits 49:50; it is ignored
     * — and required to be zero — when Sr is zero.
     */
    int       shaping_redundancy;
    int       shaping_lookahead;
    /* Diagnostic ceiling for the V.34 upstream N (N*2400 bit/s).  Zero trusts
     * MP; 2..14 permits probing peers which immediately request §9.6. */
    int       upstream_max_n;
    /*
     * A V.34 context to borrow as the modulator.  Phase 2 has already
     * configured one — power, symbol rate, carrier — and its receiver is what
     * read INFO0d/INFO1d, so the live engine passes it here rather than
     * standing a second modem up beside it.  Its transmit state machine is
     * left behind either way; only the modulator is used.  NULL creates one,
     * which is what the offline tests want.
     */
    v34_state_t *v34;
} v90_analogue_phase3_config_t;

v90_analogue_phase3_t *v90_analogue_phase3_init(const v90_analogue_phase3_config_t *cfg);
void v90_analogue_phase3_free(v90_analogue_phase3_t *s);

/*
 * Consume received G.711 codewords, and act on what they mean.  Returns the
 * receive events raised (V90A_RX_EVENT_*), after they have been applied to the
 * transmitter.
 */
/* Event outside the Phase 3/4 receiver bit range: §9.3.2/§9.4.2/§9.6.2 all
 * require the analogue modem to answer a sustained digital-modem Tone B. */
#define V90A_EVENT_TONE_B_RETRAIN  (1u << 16)

unsigned v90_analogue_phase3_rx(v90_analogue_phase3_t *s,
                                const uint8_t *codewords,
                                int count);

/* Produce the upstream, as linear samples.  Returns samples written. */
int v90_analogue_phase3_tx(v90_analogue_phase3_t *s, int16_t *amp, int max_len);

v90_analogue_tx_stage_t v90_analogue_phase3_tx_stage(const v90_analogue_phase3_t *s);
v90_analogue_rx_stage_t v90_analogue_phase3_rx_stage(const v90_analogue_phase3_t *s);
const v90_analogue_tx_t *v90_analogue_phase3_tx_state(const v90_analogue_phase3_t *s);
const v90_analogue_rx_t *v90_analogue_phase3_rx_state(const v90_analogue_phase3_t *s);

/* True once §9.3.2.10 has been answered and Phase 4 is next. */
bool v90_analogue_phase3_complete(const v90_analogue_phase3_t *s);

/*
 * A §9.3.2 deadline has passed with nothing to show for it — §9.3.2.4's 1500 ms
 * for the Sd-to-S̄d transition, or §9.3.2.7's 4500 ms for Jd.  Both require a
 * retrain (§9.5.2.1), which is the caller's business, not this module's.
 */
bool v90_analogue_phase3_retrain_due(const v90_analogue_phase3_t *s);

/* The DIL measurement, once §9.3.2.10 has fired.  NULL before that. */
const v90_dil_measurement_t *v90_analogue_phase3_measurement(const v90_analogue_phase3_t *s);

/*
 * §9.4.  The handover happens inside this module, at the one moment both its
 * conditions hold — the transmitter has finished §9.3.2.10 and a measurement
 * exists — because that is when a CPt can be built, and nothing after R̄i can
 * be read without one.  From then on the codeword stream goes to the Phase 4
 * receiver instead of the Phase 3 one.
 */
const v90_analogue_phase4_t *v90_analogue_phase3_phase4_state(const v90_analogue_phase3_t *s);
/* The measurement produced no constellation §8.5.2 would let us offer. */
bool v90_analogue_phase3_phase4_failed(const v90_analogue_phase3_t *s);
/* §9.4.2: no B1d within 15 s plus five round trips, or no usable CP. */
bool v90_analogue_phase3_phase4_retrain_due(const v90_analogue_phase3_t *s);
/* What is being transmitted, once Phase 4 has started.  NULL before that. */
const vpcm_cp_frame_t *v90_analogue_phase3_cpt(const v90_analogue_phase3_t *s);
const vpcm_cp_frame_t *v90_analogue_phase3_cp(const v90_analogue_phase3_t *s);
/* §8.6.1 has completed and downstream CP data is being decoded. */
bool v90_analogue_phase3_data_ready(const v90_analogue_phase3_t *s);
int v90_analogue_phase3_get_data_bits(v90_analogue_phase3_t *s,
                                      uint8_t *bits, int max_bits);
int v90_analogue_phase3_upstream_rate(const v90_analogue_phase3_t *s);

/* V.90 §9.6.2.1: begin an analogue-modem-initiated transaction.  With
 * silence_request, perform the CPs/Ed/SCR/CP and Rt/R̄t second pass. */
bool v90_analogue_phase3_start_rate_renegotiation(v90_analogue_phase3_t *s,
                                                   bool silence_request);
bool v90_analogue_phase3_rate_renegotiating(const v90_analogue_phase3_t *s);
/* §9.7: initiate CP drn=0 through the §9.6 transaction. */
bool v90_analogue_phase3_start_cleardown(v90_analogue_phase3_t *s);
bool v90_analogue_phase3_cleardown_complete(const v90_analogue_phase3_t *s);
/* §9.6.2: Ed must arrive within 5000 ms plus two round trips. */
bool v90_analogue_phase3_rate_retrain_due(const v90_analogue_phase3_t *s);

#endif
