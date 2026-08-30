/* V.90 §9.6.1.1.1 CP receive: the streamed decode, standalone.
 *
 * This is the first file of the ESP32 port.  It is the whole of what
 * v34rx.c's V34_RX_STAGE_V90_CP case does when ME_V90_RENEG_CP_STREAM is on
 * -- 11 lines of decode -- lifted out of a 1600-line case whose remaining
 * 1589 lines are the searched path and its diagnostics.
 *
 * There is nothing to search for.  Per §8.5.2/§10.1.3.3 and the ordering of
 * training_constellation_4, all four degrees of freedom are fixed by spec
 * rather than by the channel:
 *
 *   domain     differential      (§8.5.2 sends CP through J's modulation)
 *   transform  the fixed negation (map_phase4_raw_bits hypothesis 8)
 *   scrambler  the analogue modem's GPA, tap 4
 *   bit order  b0,b1
 *
 * So the 24-entry hypothesis table collapses to one row, and the Table 14
 * framer in v90_cp_rx.c owns sync, length, CRC and semantics -- which it is
 * already written to do, with no dependency on the V.34 receiver.
 */

#include "cp_stream.h"

/* map_phase4_raw_bits() row 8, MP_HYPOTHESIS_DIFF_INVERSE, and nothing else. */
static const uint8_t cp_dibit_negate[4] = {0, 3, 2, 1};

#define CP_SCRAMBLER_TAP 4

void cp_stream_init(cp_stream_t *s, v90_cp_rx_t *framer)
{
    s->reg = 0;
    s->framer = framer;
}

/* One Phase 4 symbol's differential dibit in, two descrambled bits out. */
void cp_stream_put_dibit(cp_stream_t *s, int dibit)
{
    int raw = cp_dibit_negate[dibit & 3];
    int in[2];
    int i;

    /* Bit order b0,b1 (phase4_unpack_ordered_bits, order_idx 0). */
    in[0] = raw & 1;
    in[1] = (raw >> 1) & 1;

    for (i = 0; i < 2; i++) {
        /* descramble_reg(): GPA, x^-4 and x^-23. */
        int out = (in[i] ^ (s->reg >> CP_SCRAMBLER_TAP) ^ (s->reg >> 22)) & 1;
        s->reg = (s->reg << 1) | (uint32_t) in[i];
        v90_cp_rx_put_bit(s->framer, out);
    }
}
