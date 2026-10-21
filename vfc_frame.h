/* V.FC eight-symbol frame geometry/schedule and shell packing, Courier 403
 * c1bf..c20d, c81b..c98c. This layer does not process samples or train a modem.
 * Native firmware addresses, not ITU V.34 clauses, identify the rules.
 */
#ifndef VFC_FRAME_H
#define VFC_FRAME_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    unsigned high_bits, uncoded_bits, max_ring, shell_extra, period;
    uint16_t schedule_mask, schedule;
} vfc_frame_plan_t;

/* baud index: 2400, 2743, 2800, 3000, 3200, 3429. Rates 9600..28800,
 * steps of 2400. extra_ring is negotiated 0878/0879 bit 14. Rejects geometry
 * outside the 1280-point table; does not qualify a rate as negotiable on a
 * particular channel. Failure leaves the destination unchanged. */
bool vfc_frame_plan(vfc_frame_plan_t *p, unsigned baud_index, unsigned bit_rate,
                    bool extra_ring);
unsigned vfc_frame_next_bits(vfc_frame_plan_t *p);

typedef struct {
    uint8_t direct[8];       /* q uncoded bits above the low differential dibit */
    uint8_t ring[8];
    unsigned bits, next_cursor;
} vfc_symbol_frame_t;

/* c88a..c98c: map a frame from the 128-bit, LSB-first scrambled-bit ring.
 * cursor wraps modulo 128 as native 03af does. The plan advances only on
 * success. Samples and waveform clocks are outside this bit-layer API. */
bool vfc_frame_map(vfc_frame_plan_t *p, const uint8_t input[16], unsigned cursor,
                   vfc_symbol_frame_t *out);
/* Inverse enumeration/packing. Changes only the frame's bits in output[16];
 * all output and plan state are preserved on an invalid frame. */
bool vfc_frame_unmap(vfc_frame_plan_t *p, const vfc_symbol_frame_t *frame,
                     unsigned cursor, uint8_t output[16]);
#endif
