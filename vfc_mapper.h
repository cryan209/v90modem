/* V.FC isolated symbol mapper. Courier 403 overlay 7, c749..c788.
 *
 * This core accepts the point index produced by the (not yet implemented)
 * V.FC frame/shell mapper and the two scrambled differential bits. It does
 * not implement training, framing, precoding, or a callable modem mode.
 * V.FC has no ITU Recommendation: firmware evidence and regeneration commands
 * are recorded in docs/vfc_implementation.md. Do not substitute V.34 9.6.3's
 * constellation ordering or its 4D convolutional encoder here.
 */
#ifndef VFC_MAPPER_H
#define VFC_MAPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t code;           /* 8 or 32 states; firmware encodes both in 5 bits */
    uint8_t trellis;
    uint8_t quadrant;
} vfc_mapper_t;

typedef struct {
    int16_t i, q;           /* integer lattice units, before precoder/scaling */
} vfc_point_t;

/* Resets all per-call state. Invalid arguments leave the state unchanged. */
bool vfc_mapper_init(vfc_mapper_t *s, unsigned code);

/* index 0..159; dibit 0..3. Failure changes neither state nor output. */
bool vfc_mapper_put(vfc_mapper_t *s, unsigned index, unsigned dibit,
                    vfc_point_t *point);

#endif
