/*
 * v90_analogue_tx.h — the analogue modem's Phase 3 transmitter (§9.3.2).
 *
 * The other half of this software.  v90.c transmits the *digital* side's
 * Phase 3 — Sd, S̄d, TRN1d, Jd, J'd, DIL — as PCM codewords.  This module
 * transmits the analogue side's answer to it: S, S̄, PP, TRN, Ja and SCR,
 * as V.34 symbols (§8.3), with Ja carrying the DIL descriptor that tells the
 * digital modem what to probe the line with (§8.3.1, Table 12).
 *
 * It emits *symbols*, one per baud, and nothing else.  Pulse shaping, carrier
 * and gain stay in SpanDSP's V.34 modulator, which this feeds through
 * v34_tx_start_external_symbols(); the sequencing stays here because what
 * drives it — the Sd-to-S̄d transition, Jd, J'd, the end of DIL — arrives in
 * the PCM downstream, which SpanDSP's V.34 receiver never sees.  So the state
 * machine advances on two things: symbol counts for the durations §9.3.2
 * fixes, and explicit events for everything it makes conditional.
 *
 * The scrambler is GPA (1 + x^-5 + x^-23, equation 7-2/V.34) for Ja, TRN and
 * SCR — §8.3.  Not GPC: that is the digital modem's, and the two coexisting
 * is a documented trap in this tree (v90.c:439).
 */
#ifndef V90_ANALOGUE_TX_H
#define V90_ANALOGUE_TX_H

#include <stdbool.h>
#include <stdint.h>

#include "v90.h"
#include "vpcm_cp.h"

typedef enum {
    /* §9.3.2.1 — 70 ± 5 ms of silence after INFO1a. */
    V90A_TX_INITIAL_SILENCE = 0,
    V90A_TX_S,                  /* S for 128T */
    V90A_TX_S_BAR,              /* S̄ for 16T */
    V90A_TX_MD,                 /* §9.3.2.1 — only when INFO1a's MD is non-zero */
    V90A_TX_S2,                 /* S for 128T, after MD */
    V90A_TX_S_BAR2,             /* S̄ for 16T, after MD */
    V90A_TX_PP,                 /* §9.3.2.2 — 288T */
    V90A_TX_TRN,                /* §9.3.2.3 — 4-point, at least 512T */
    V90A_TX_JA,                 /* §9.3.2.4 — DIL descriptor, repeated */
    V90A_TX_JA_SILENCE,         /* §9.3.2.4 — silent from the Sd-to-S̄d transition */
    V90A_TX_S_AFTER_JD,         /* §9.3.2.7 — S while waiting for J'd */
    V90A_TX_S_BAR_AFTER_JD,     /* §9.3.2.8 — S̄ for 16T */
    V90A_TX_DIL_RX,             /* §9.3.2.9 — silence or SCR while DIL arrives */
    V90A_TX_S_DIL_ENOUGH,       /* §9.3.2.10 — S for 128T: "enough of the DIL" */
    V90A_TX_S_BAR_DIL_ENOUGH,   /* §9.3.2.10 — S̄ for 16T */
    /*
     * §9.4.2.  Phase 4's transmit side is the same modulation as Ja -- §8.5.2
     * sends CP "according to 10.1.3.9/V.34", which is J's -- so it belongs
     * here rather than in a second symbol pump beside this one.  What changes
     * is only which bits are fed to it, and that Phase 4 has four conditional
     * moments instead of Phase 3's four.
     */
    V90A_TX_PHASE4,             /* §9.3.2.10 done, Phase 4 not yet armed */
    V90A_TX_CPT,                /* §9.4.2.1 — CPt, until the Ri→R̄i transition */
    V90A_TX_SCR4,               /* §9.4.2.2 — optional SCR, at most 4000 ms */
    V90A_TX_CP,                 /* §9.4.2.3 — CP, until MP arrives */
    V90A_TX_CP_PRIME,           /* §9.4.2.3 — CP with the acknowledge bit set */
    V90A_TX_E,                  /* §9.4.2.4 — §8.5.3's 20 ones */
    V90A_TX_B1_PENDING,         /* §9.4.2.5 — hand over to V.34 B1/data mapper */
    V90A_TX_RR_S,               /* §9.6.2.2.2 — response S for 128T */
    V90A_TX_RR_S_BAR,           /* §9.6.2.1.2/.2.3 — S̄ for 16T */
    V90A_TX_RR_CPS,             /* §9.6.2.1.3 — CP with silence request */
    V90A_TX_RR_CPS_PRIME,       /* §9.6.2.1.5 — acknowledged CPs */
    V90A_TX_RR_EC_SCR,          /* §9.6.2.1.6 — echo-canceller SCR */
    V90A_TX_RR_CP,              /* §9.6.2.1.7 — CP with bit 30 clear */
} v90_analogue_tx_stage_t;

typedef struct v90_analogue_tx_s v90_analogue_tx_t;

typedef struct {
    /* Symbol rate code, 0 = 2400 … 5 = 3429.  This is INFO1a bits 34:36, the
     * upstream rate the analogue modem selected. */
    int  baud_rate_code;
    /* INFO1a bits 18:24 — MD length in 35 ms units.  Zero skips MD, which is
     * what every implementation in this tree's captures does. */
    int  md_units;
    /* Descriptor to carry in Ja.  n == 0 requests a zero-length DIL, and
     * §9.3.2.8 then goes straight to Phase 4. */
    v90_dil_desc_t dil;
    /* §9.3.2.9: transmit SCR rather than silence while DIL arrives.  Both are
     * permitted; SCR keeps line energy up, which the NOTE under Table 12
     * points at for holding echo control devices down. */
    bool scr_during_dil;
} v90_analogue_tx_config_t;

/* Create a transmitter.  Returns NULL if the descriptor cannot be packed into
 * a Ja sequence, or if the baud rate code is out of range. */
v90_analogue_tx_t *v90_analogue_tx_init(const v90_analogue_tx_config_t *cfg);
void v90_analogue_tx_free(v90_analogue_tx_t *s);

/* Symbol source.  Signature matches v34_tx_external_symbol_func_t, so this
 * goes straight to v34_tx_start_external_symbols() with the context as the
 * user data.  Symbols are in constellation steps: the 4-point training
 * constellation is (±0.7071068, ±0.7071068), PP is unit magnitude. */
void v90_analogue_tx_get_symbol(void *user_data, float *re, float *im);

/*
 * Events from the PCM receive side.  Each is the condition §9.3.2 names, and
 * each is idempotent: a repeat in a stage that has already consumed it is
 * ignored rather than skipping a stage.
 */
void v90_analogue_tx_sd_bar_seen(v90_analogue_tx_t *s);   /* §9.3.2.4 */
void v90_analogue_tx_jd_seen(v90_analogue_tx_t *s);       /* §9.3.2.7 */
void v90_analogue_tx_jd_prime_seen(v90_analogue_tx_t *s); /* §9.3.2.8 */
void v90_analogue_tx_dil_enough(v90_analogue_tx_t *s);    /* §9.3.2.10 */

/*
 * Arm Phase 4 (§9.4.2.1) with the two CP sequences the measurement produced:
 * CPt names the constellation the digital modem trains on, CP the one it uses
 * in data mode.  CP' is CP with Table 14 bit 33 set, so it is derived here
 * rather than passed in — §8.5.2 requires every CP in a group to carry
 * identical parameters, and building it from the same frame is the only way to
 * be sure of that.
 *
 * Takes effect at V90A_TX_PHASE4, which §9.3.2.10 has already reached by the
 * time a measurement exists.  Returns false if either sequence will not pack.
 */
bool v90_analogue_tx_start_phase4(v90_analogue_tx_t *s,
                                  const vpcm_cp_frame_t *cpt,
                                  const vpcm_cp_frame_t *cp,
                                  bool scr_after_r);

/*
 * Phase 4's conditional moments, from the PCM receive side (§9.4.2).  Like the
 * Phase 3 events above, each is idempotent.
 *
 * "Complete the current sequence" in §9.4.2.2/.3/.4 is taken literally: the
 * event is latched and the stage changes at the next sequence boundary, so a
 * peer never sees a CP truncated mid-frame.
 */
void v90_analogue_tx_r_transition_seen(v90_analogue_tx_t *s); /* §9.4.2.2 */
void v90_analogue_tx_mp_seen(v90_analogue_tx_t *s);           /* §9.4.2.3 */
void v90_analogue_tx_mp_prime_seen(v90_analogue_tx_t *s);     /* §9.4.2.4 */
/* §9.6.2.1/.2: initiate locally, or respond to the digital modem's Rd→R̄d. */
bool v90_analogue_tx_start_rate_renegotiation(v90_analogue_tx_t *s,
                                               bool silence_request);
bool v90_analogue_tx_rate_renegotiate(v90_analogue_tx_t *s);
void v90_analogue_tx_rt_transition_seen(v90_analogue_tx_t *s);

v90_analogue_tx_stage_t v90_analogue_tx_stage(const v90_analogue_tx_t *s);
const char *v90_analogue_tx_stage_name(v90_analogue_tx_stage_t stage);
/* Symbols emitted in the current stage, and since the transmitter started. */
int v90_analogue_tx_stage_symbols(const v90_analogue_tx_t *s);
uint64_t v90_analogue_tx_total_symbols(const v90_analogue_tx_t *s);
/* Length of the packed Ja descriptor, in bits. */
int v90_analogue_tx_ja_bits(const v90_analogue_tx_t *s);

/*
 * Whether §9.3.2's deadline for the stage we are in has passed, given the
 * symbols emitted so far.  §9.3.2.4 allows 1500 ms from the start of Ja for
 * the Sd-to-S̄d transition and §9.3.2.7 allows 4500 ms from the end of Ja for
 * Jd; both then require a retrain (§9.5.2.1) rather than waiting longer.
 */
bool v90_analogue_tx_deadline_passed(const v90_analogue_tx_t *s);

#endif
