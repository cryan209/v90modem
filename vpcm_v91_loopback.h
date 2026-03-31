#ifndef VPCM_V91_LOOPBACK_H
#define VPCM_V91_LOOPBACK_H

#include "vpcm_call_pair.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t *caller_data_in;
    uint8_t *caller_data_out;
    uint8_t *answerer_data_in;
    uint8_t *answerer_data_out;
    uint8_t *caller_pcm_tx;
    uint8_t *caller_pcm_rx;
    uint8_t *answerer_pcm_tx;
    uint8_t *answerer_pcm_rx;
    int data_frames;
    int total_bits;
    int total_bytes;
    int total_codewords;
    int codewords_per_report;
    int sample_data_len;
    int sample_pcm_len;
    vpcm_call_pair_v91_data_report_t data_report;
} vpcm_v91_loopback_run_t;

bool vpcm_v91_loopback_run_data(vpcm_v91_loopback_run_t *run,
                                vpcm_call_pair_t *pair,
                                vpcm_v91_session_t *session,
                                v91_state_t *caller_tx,
                                v91_state_t *caller_rx,
                                v91_state_t *answerer_tx,
                                v91_state_t *answerer_rx,
                                const vpcm_cp_frame_t *cp_ack,
                                uint32_t data_seed,
                                int data_seconds,
                                bool robbed_bit,
                                vpcm_call_pair_v91_chunk_logger_fn chunk_logger,
                                void *chunk_logger_user_data);
bool vpcm_v91_loopback_verify(const char *label,
                              const vpcm_v91_loopback_run_t *run);
void vpcm_v91_loopback_cleanup(vpcm_v91_loopback_run_t *run);

#endif
