#ifndef VPCM_CALL_PAIR_H
#define VPCM_CALL_PAIR_H

#include "vpcm_call.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    vpcm_call_t *caller;
    vpcm_call_t *answerer;
    const char *label;
} vpcm_call_pair_t;

typedef struct {
    int frame_index;
    int codeword_offset;
    int chunk_codewords;
    int chunk_bytes;
    int byte_offset;
    const uint8_t *caller_data_in;
    const uint8_t *answerer_data_in;
    const uint8_t *caller_data_out;
    const uint8_t *answerer_data_out;
    const uint8_t *caller_pcm_tx;
    const uint8_t *caller_pcm_rx;
    const uint8_t *answerer_pcm_tx;
    const uint8_t *answerer_pcm_rx;
    bool caller_to_answerer_ok;
    bool answerer_to_caller_ok;
    uint64_t c2a_bits_checked;
    uint64_t a2c_bits_checked;
    uint64_t c2a_bit_errors;
    uint64_t a2c_bit_errors;
    int c2a_mismatch_chunks;
    int a2c_mismatch_chunks;
    int c2a_chunks_checked;
    int a2c_chunks_checked;
} vpcm_call_pair_v91_chunk_t;

typedef struct {
    uint64_t c2a_bits_checked;
    uint64_t a2c_bits_checked;
    uint64_t c2a_bit_errors;
    uint64_t a2c_bit_errors;
    int c2a_mismatch_chunks;
    int a2c_mismatch_chunks;
    int c2a_chunks_checked;
    int a2c_chunks_checked;
} vpcm_call_pair_v91_data_report_t;

typedef void (*vpcm_call_pair_v91_chunk_logger_fn)(const vpcm_call_pair_v91_chunk_t *chunk,
                                                   void *user_data);

bool vpcm_call_pair_init(vpcm_call_pair_t *pair,
                         vpcm_call_t *caller,
                         vpcm_call_t *answerer,
                         const char *label);
bool vpcm_call_pair_attach_taps(vpcm_call_pair_t *pair, const char *dir);
void vpcm_call_pair_detach_taps(vpcm_call_pair_t *pair);
bool vpcm_call_pair_record_g711_exchange(vpcm_call_pair_t *pair,
                                         const uint8_t *caller_tx_codewords,
                                         const uint8_t *answerer_tx_codewords,
                                         size_t codeword_len);
bool vpcm_call_pair_drive_to_run(vpcm_call_pair_t *pair,
                                 vpcm_call_run_mode_t run_mode);
bool vpcm_call_pair_drive_to_done(vpcm_call_pair_t *pair);
void vpcm_call_pair_set_v91_state(vpcm_call_pair_t *pair,
                                  vpcm_v91_modem_state_t state);
bool vpcm_call_pair_run_v91_data(vpcm_call_pair_t *pair,
                                 vpcm_v91_session_t *session,
                                 v91_state_t *caller_tx,
                                 v91_state_t *caller_rx,
                                 v91_state_t *answerer_tx,
                                 v91_state_t *answerer_rx,
                                 const vpcm_cp_frame_t *cp_ack,
                                 uint8_t *caller_data_in,
                                 uint8_t *caller_data_out,
                                 uint8_t *answerer_data_in,
                                 uint8_t *answerer_data_out,
                                 uint8_t *caller_pcm_tx,
                                 uint8_t *caller_pcm_rx,
                                 uint8_t *answerer_pcm_tx,
                                 uint8_t *answerer_pcm_rx,
                                 int total_codewords,
                                 int codewords_per_report,
                                 bool robbed_bit,
                                 vpcm_call_pair_v91_chunk_logger_fn chunk_logger,
                                 void *chunk_logger_user_data,
                                 vpcm_call_pair_v91_data_report_t *report);

#endif
