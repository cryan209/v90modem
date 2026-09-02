/*
 * v90_analogue_rx.h — the analogue modem's Phase 3 receiver (§9.3.2), live.
 *
 * The analogue modem's Phase 3 is driven entirely by what arrives from the
 * digital modem: §9.3.2.4 ends Ja on the Sd-to-S̄d transition, §9.3.2.7 starts
 * S after Jd, §9.3.2.8 sends S̄ after J'd, and §9.3.2.10 ends DIL when enough
 * of it has been received.  Those four moments are the whole interface between
 * this module and v90_analogue_tx.c.
 *
 * All of it arrives as G.711 codewords, not as a modulated signal — the
 * downstream *is* the DS0 stream — so this is a codeword state machine, and
 * it consumes the codewords exactly as they come off the wire.  There is no
 * V.34 receiver anywhere in the analogue role.
 *
 * The signals, and what distinguishes them (§8.4):
 *
 *   Sd      §8.4.4 — {+W, +0, +W, −W, −0, −W} repeated, W = Ucode(16+U_INFO).
 *   S̄d      §8.4.4 — the same pattern with every sign reversed.  Note this is
 *                    Sd rotated by exactly three slots, so it is *not*
 *                    distinguishable from Sd by content: only by the phase
 *                    discontinuity against the Sd alignment already acquired.
 *   TRN1d   §8.4.5 — scrambled ones on the *U_INFO* codeword.  The magnitude
 *                    change from W to U_INFO is what marks the boundary.
 *   Jd      §8.4.2 — a 72-bit Table 13 frame on the same U_INFO codeword,
 *                    scrambled (GPC) and differentially encoded onto the sign.
 *                    TRN1d descrambles to ones and Jd opens with 17 sync ones,
 *                    so the frame is found by its bit-17 start bit: the first
 *                    zero after §9.3.2.5's 2040T.
 *   J'd     §8.4.3 — 12 scrambled zeros, terminating Jd.
 *   DIL     §8.4.1 — the sequence this side asked for in Ja, which is why it
 *                    is measured (v90_dil_measure.c) and never "recovered".
 *
 * The scrambler is GPC (1 + x^-18 + x^-23) — the digital modem's, §5.3.  The
 * analogue modem's own GPA belongs to v90_analogue_tx.c and the two must not
 * be confused (v90.c:439).
 */
#ifndef V90_ANALOGUE_RX_H
#define V90_ANALOGUE_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "v90.h"
#include "v90_dil_measure.h"

typedef enum {
    V90A_RX_HUNT_SD = 0,    /* looking for Sd's 6-codeword pattern */
    V90A_RX_SD,             /* Sd acquired, tracking it */
    V90A_RX_SD_BAR,         /* §9.3.2.4 transition seen; S̄d running */
    V90A_RX_TRN1D,          /* §8.4.5 scrambled ones at U_INFO */
    V90A_RX_JD,             /* §8.4.2 Table 13 frames */
    V90A_RX_DIL,            /* §8.4.1, being measured */
    V90A_RX_DONE,           /* enough DIL; Phase 4 next */
} v90_analogue_rx_stage_t;

/* Events, as a bitmask returned by v90_analogue_rx_put(). */
#define V90A_RX_EVENT_SD          (1u << 0)  /* Sd acquired */
#define V90A_RX_EVENT_SD_BAR      (1u << 1)  /* §9.3.2.4 Sd-to-S̄d transition */
#define V90A_RX_EVENT_TRN1D       (1u << 2)  /* §9.3.2.5 TRN1d started */
#define V90A_RX_EVENT_JD          (1u << 3)  /* §9.3.2.6 a valid Jd frame */
#define V90A_RX_EVENT_JD_PRIME    (1u << 4)  /* §9.3.2.8 J'd */
#define V90A_RX_EVENT_DIL_ENOUGH  (1u << 5)  /* §9.3.2.10 enough of the DIL */

typedef struct v90_analogue_rx_s v90_analogue_rx_t;

typedef struct {
    v90_law_t law;
    /* U_INFO as this side put it in INFO1a — the analogue modem chose it, so
     * it is known, and the receiver never has to search for it. */
    int u_info;
    /* The descriptor sent in Ja.  n == 0 means no DIL was requested, and
     * §9.3.2.8 then goes straight to Phase 4 with no DIL stage at all. */
    v90_dil_desc_t dil;
    /*
     * How much of the DIL cycle to take before declaring "enough" (§9.3.2.10).
     * A full pass is not needed: every DIL-segment measures one training Ucode
     * independently, so half a cycle measures half the ladder, and the
     * measurement path is built for a partial pass.  0 uses the default.
     */
    double dil_coverage;
} v90_analogue_rx_config_t;

v90_analogue_rx_t *v90_analogue_rx_init(const v90_analogue_rx_config_t *cfg);
void v90_analogue_rx_free(v90_analogue_rx_t *s);

/*
 * Consume received codewords.  Returns the OR of every event raised by this
 * call; a single call can raise several if the buffer spans a boundary.
 */
unsigned v90_analogue_rx_put(v90_analogue_rx_t *s,
                             const uint8_t *codewords,
                             int count);

/*
 * Enter the DIL stage directly, as if J'd had just ended.
 *
 * The module normally finds that boundary itself (§8.4.3), and the live path
 * lets it.  This is for exercising §9.3.2.9/§9.3.2.10 on their own, and for a
 * receiver that joined the stream after J'd had already gone past.
 */
void v90_analogue_rx_begin_dil(v90_analogue_rx_t *s);

v90_analogue_rx_stage_t v90_analogue_rx_stage(const v90_analogue_rx_t *s);
const char *v90_analogue_rx_stage_name(v90_analogue_rx_stage_t stage);

/* Where each signal was found, as a codeword index into the stream fed in. */
int64_t v90_analogue_rx_sd_start(const v90_analogue_rx_t *s);
int64_t v90_analogue_rx_sd_bar_start(const v90_analogue_rx_t *s);
int64_t v90_analogue_rx_trn1d_start(const v90_analogue_rx_t *s);
int64_t v90_analogue_rx_jd_start(const v90_analogue_rx_t *s);
int64_t v90_analogue_rx_dil_start(const v90_analogue_rx_t *s);

int v90_analogue_rx_sd_reps(const v90_analogue_rx_t *s);
int v90_analogue_rx_sd_bar_reps(const v90_analogue_rx_t *s);
int v90_analogue_rx_trn1d_symbols(const v90_analogue_rx_t *s);
int v90_analogue_rx_jd_symbols(const v90_analogue_rx_t *s);
int v90_analogue_rx_jd_frames(const v90_analogue_rx_t *s);
int v90_analogue_rx_dil_symbols(const v90_analogue_rx_t *s);

/* §8.4.4's W, as learned off the wire (see sd_hunt_slot()).  Over an analogue
 * bearer this is a Ucode on the slicer\'s scaled ladder, not the far end\'s. */
int v90_analogue_rx_w(const v90_analogue_rx_t *s);

/* Table 13 bit 47: the constellation the digital modem will train with. */
bool v90_analogue_rx_jd_trn16(const v90_analogue_rx_t *s);
/* The 72 bits of the last Jd frame that passed structure and CRC. */
const uint8_t *v90_analogue_rx_jd_bits(const v90_analogue_rx_t *s);

/*
 * The DIL measurement, valid once V90A_RX_EVENT_DIL_ENOUGH has been raised.
 * This is what Phase 4's CP is built from (docs/v90_constellation_selection.md).
 */
const v90_dil_measurement_t *v90_analogue_rx_measurement(const v90_analogue_rx_t *s);

#endif
