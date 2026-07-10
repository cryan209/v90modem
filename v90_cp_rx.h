/* Strict ITU-T V.90 Table 14 CP/CP' bitstream receiver. */
#ifndef V90_CP_RX_H
#define V90_CP_RX_H

#include "vpcm_cp.h"

#include <stdbool.h>
#include <stdint.h>

typedef void (*v90_cp_rx_frame_handler_t)(void *user_data,
                                          const vpcm_cp_diag_t *diag);

typedef struct {
    uint8_t bits[VPCM_CP_MAX_BITS];
    int bit_count;
    int target_bits;
    int sync_ones;
    int constellation_points;
    bool expected_alaw;
    bool collecting;
    uint64_t input_bits;
    uint32_t valid_frames;
    uint32_t rejected_frames;
    v90_cp_rx_frame_handler_t frame_handler;
    void *frame_user_data;
} v90_cp_rx_t;

void v90_cp_rx_init(v90_cp_rx_t *rx,
                    int constellation_points,
                    bool expected_alaw,
                    v90_cp_rx_frame_handler_t frame_handler,
                    void *user_data);
void v90_cp_rx_reset(v90_cp_rx_t *rx);
bool v90_cp_rx_put_bit(v90_cp_rx_t *rx, int bit);

#endif
