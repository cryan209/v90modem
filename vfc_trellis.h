/* V.FC 2D trellis receiver for equalized, inverse-precoded lattice points.
 * Uses Courier c749..c788's branch geometry; not V.34's 4D code. Timing,
 * equalization, precoding and waveform acquisition are outside this layer.
 */
#ifndef VFC_TRELLIS_H
#define VFC_TRELLIS_H
#include "vfc_mapper.h"

#define VFC_TRELLIS_DEPTH 64
typedef struct { uint8_t previous, quadrant, index; } vfc_survivor_t;
typedef struct { uint8_t index, dibit; } vfc_decision_t;
typedef struct {
    bool valid, finished;
    unsigned points;
    uint8_t quadrant, best, next[32][4];
    vfc_point_t constellation[8][160];
    float metric[32];
    vfc_survivor_t history[VFC_TRELLIS_DEPTH][32];
    uint64_t received, decoded;
} vfc_trellis_t;

/* Known zero trellis state, as in native c6e9. Training supplies the previous
 * differential quadrant; do not assume it is zero at the data transition.
 * points is the negotiated number of uncoded point indices (<=160). */
bool vfc_trellis_init(vfc_trellis_t *s, unsigned code, unsigned points,
                      unsigned initial_quadrant);
/* -1 invalid input, 0 collecting traceback, 1 decoded decision. Coordinates
 * are in the integer lattice units of vfc_mapper_put, not fixed-point Q9.7.
 * Traceback delays delivery by 63 symbols; it never inserts bearer samples. */
int vfc_trellis_put(vfc_trellis_t *s, float i, float q, vfc_decision_t *out);
/* Drain at most 63 buffered decisions from the existing survivor path, without
 * inventing input symbols. Ends the stream; init is required for another call. */
unsigned vfc_trellis_finish(vfc_trellis_t *s, vfc_decision_t out[63]);
#endif
