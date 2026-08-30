#ifndef PORT_CP_STREAM_H
#define PORT_CP_STREAM_H

#include <stdint.h>
#include "v90_cp_rx.h"

typedef struct {
    uint32_t reg;            /* self-synchronizing descrambler state */
    v90_cp_rx_t *framer;     /* Table 14 framer; owns sync/length/CRC */
} cp_stream_t;

void cp_stream_init(cp_stream_t *s, v90_cp_rx_t *framer);
void cp_stream_put_dibit(cp_stream_t *s, int dibit);

#endif
