#ifndef VPCM_CALL_H
#define VPCM_CALL_H

#include "vpcm_g711_stream.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    v91_law_t law;
    unsigned sample_rate;
    unsigned frame_samples;
    const char *caller_id;
    const char *callee_id;
    const char *label;
} vpcm_call_params_t;

typedef struct {
    vpcm_call_params_t params;
    vpcm_g711_stream_t tx_stream;
    vpcm_g711_stream_t rx_stream;
    uint64_t ticks;
    double elapsed_seconds;
} vpcm_call_t;

bool vpcm_call_params_normalize(vpcm_call_params_t *params);
bool vpcm_call_init(vpcm_call_t *call,
                    const vpcm_call_params_t *params,
                    uint8_t *tx_storage,
                    size_t tx_storage_len,
                    uint8_t *rx_storage,
                    size_t rx_storage_len);
void vpcm_call_reset(vpcm_call_t *call);
bool vpcm_call_advance_tick(vpcm_call_t *call);

size_t vpcm_call_frame_bytes(const vpcm_call_t *call);
double vpcm_call_tick_seconds(const vpcm_call_t *call);

#endif
