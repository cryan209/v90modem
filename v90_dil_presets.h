/*
 * v90_dil_presets.h — DIL descriptors to send, and a check that they are worth
 * sending.
 *
 * The analogue modem authors the DIL: §8.4.1 has the descriptor travel to the
 * digital modem in Ja, and §9.3.2.9 has the analogue modem receive the
 * sequence it requested.  So anything taking the analogue role has to *have*
 * a descriptor, and which one it picks decides what it can learn — the
 * measurement in v90_dil_measure.c can only report on Ucodes the DIL actually
 * probed, in data-frame intervals it actually reached.
 *
 * v90_dil_desc_validate() is the important part of this header.  A descriptor
 * can satisfy every field constraint in Table 12 and still measure nothing in
 * an interval, and that failure is silent at the far end: the DIL transmits,
 * the levels arrive, and one sixth of the constellation is simply unknown.
 * See docs/v90_constellation_selection.md.
 */
#ifndef V90_DIL_PRESETS_H
#define V90_DIL_PRESETS_H

#include "v90.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    /*
     * The clean-line Ja profile this tree has used since before the
     * measurement path existed: 125 segments of 12T, REFc = (chord << 4) | 1,
     * training Ucodes sweeping 8 offsets across all 8 chords.
     */
    V90_DIL_PRESET_DEFAULT_JA = 0,

    /* Descriptors lifted from SLModem 2.9.11's setDilDescriptor profiles. */
    V90_DIL_PRESET_SMARTLINK_ADI,
    V90_DIL_PRESET_SMARTLINK_ADI_QC,

    /*
     * Built for measurement rather than inherited: 120 segments of 66T
     * sweeping the whole Ucode ladder, REFc = 0, and LTP 11 so that training
     * symbols reach every data-frame interval (see the trap above).
     */
    V90_DIL_PRESET_MEASUREMENT,

    /*
     * Modelled on what the Eicon card transmitted to a Courier that connected
     * (artifacts/eicon-digital-downstream/): 66T segments, REFc = 0, and a
     * descending ladder interleaved with low-Ucode probes.  Not a decode of
     * the Courier's actual descriptor -- that request travelled upstream in
     * Ja and is not in the capture, and docs/eicon_downstream_comparison.md
     * Finding 5 is why we cannot recover it from the downstream either.
     * Shape observed, values reconstructed.
     */
    V90_DIL_PRESET_COURIER_STYLE,

    V90_DIL_PRESET_COUNT
} v90_dil_preset_t;

bool v90_dil_preset_load(v90_dil_preset_t which, v90_dil_desc_t *out);
const char *v90_dil_preset_name(v90_dil_preset_t which);

/*
 * Build a descriptor around an arbitrary ladder of training Ucodes -- for
 * probing a specific part of the range, or for feeding the measurement path
 * something it was not designed for.  `hc` sets every chord's segment length
 * to (hc + 1) * 6; REFc is 0; SP/TP default to patterns whose period is
 * coprime with 6.  Validate the result before trusting it.
 */
bool v90_dil_desc_from_ucodes(const uint8_t *ucodes, int n, uint8_t hc,
                              v90_dil_desc_t *out);

typedef struct {
    int     cycle_symbols;
    double  cycle_ms;
    int     segments;
    int     distinct_ucodes;
    int     lowest_ucode;
    int     highest_ucode;
    uint8_t intervals_probed;   /* bit i set: interval i sees >= 2 distinct levels */
    int     min_interval_levels;/* fewest distinct levels any interval receives */
    int     chords_covered;     /* Uchords with at least one training Ucode */
    bool    ok;                 /* all six intervals probed and >= 2 Ucodes */
} v90_dil_desc_check_t;

/*
 * Report what a descriptor will actually be able to teach us.
 *
 * DIL starts on a data-frame boundary -- §8.4.4 puts Sd's first symbol in
 * interval 0 and everything between (TRN1d, Jd, J'd) is a multiple of 6 -- so
 * a symbol's interval is its position within the DIL, mod 6.
 *
 * An interval counts as probed when at least two *distinct levels* reach it.
 * Reference symbols count towards that: a profile whose REFc differs per
 * chord delivers a spread through its TP=0 positions alone.
 */
bool v90_dil_desc_validate(const v90_dil_desc_t *desc,
                           v90_dil_desc_check_t *out);

#endif
