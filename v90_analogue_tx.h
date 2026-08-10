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
    V90A_TX_PHASE4,             /* §9.4 — this module is done */
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
