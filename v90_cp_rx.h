/* Strict ITU-T V.90 Table 14 CP/CP' bitstream receiver. */
#ifndef V90_CP_RX_H
#define V90_CP_RX_H

#include "vpcm_cp.h"

#include <stdbool.h>
#include <stdint.h>

typedef void (*v90_cp_rx_frame_handler_t)(void *user_data,
                                          const vpcm_cp_diag_t *diag);

/* Soft-combine history depth.  CPt/CP repeat; keeping the last few
 * CRC-failed copies lets a per-bit majority vote recover a CRC-valid
 * consensus even when no single copy is clean.  CRC remains the final
 * gate on both the raw copy and the voted consensus. */
#define V90_CP_RX_VOTE_FRAMES 8

typedef struct {
    uint8_t bits[VPCM_CP_MAX_BITS];
    int target_bits;
} v90_cp_rx_vote_t;

typedef struct {
    uint8_t bits[VPCM_CP_MAX_BITS];
    int bit_count;
    int target_bits;
    int sync_ones;
    int constellation_points;
    bool expected_alaw;
    bool collecting;
    uint64_t input_bits;
    uint32_t sync_candidates;
    uint32_t valid_frames;
    uint32_t rejected_frames;
    uint32_t crc_rejected_frames;
    uint32_t structure_rejected_frames;
    uint32_t semantic_rejected_frames;
    uint32_t voted_frames_accepted;
    v90_cp_rx_frame_handler_t frame_handler;
    void *frame_user_data;
    /* Soft-combine accumulator: the last V90_CP_RX_VOTE_FRAMES rejected
     * copies that shared the same target_bits.  Persists across the
     * per-frame v90_cp_rx_reset() so repeated CP copies accumulate; cleared
     * explicitly by v90_cp_rx_clear_votes() on retrain / phase transition
     * and by v90_cp_rx_init(). */
    v90_cp_rx_vote_t vote[V90_CP_RX_VOTE_FRAMES];
    int vote_count;
    int vote_target_bits;
} v90_cp_rx_t;

void v90_cp_rx_init(v90_cp_rx_t *rx,
                    int constellation_points,
                    bool expected_alaw,
                    v90_cp_rx_frame_handler_t frame_handler,
                    void *user_data);
void v90_cp_rx_reset(v90_cp_rx_t *rx);
/* Clear the soft-combine vote history.  Call on retrain / phase transition;
 * v90_cp_rx_reset() deliberately keeps votes so repeated CP copies
 * accumulate across frame boundaries. */
void v90_cp_rx_clear_votes(v90_cp_rx_t *rx);
bool v90_cp_rx_put_bit(v90_cp_rx_t *rx, int bit);

#endif
