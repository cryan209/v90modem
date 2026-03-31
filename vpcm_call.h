#ifndef VPCM_CALL_H
#define VPCM_CALL_H

#include "vpcm_g711_stream.h"
#include "vpcm_v91_session.h"

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

typedef enum {
    VPCM_CALL_IDLE = 0,
    VPCM_CALL_WAIT_DIALTONE,
    VPCM_CALL_DIAL,
    VPCM_CALL_WAIT_RINGING,
    VPCM_CALL_ANSWER,
    VPCM_CALL_RUN,
    VPCM_CALL_HANGUP,
    VPCM_CALL_DONE
} vpcm_call_state_t;

typedef enum {
    VPCM_CALL_RUN_NONE = 0,
    VPCM_CALL_RUN_V90_MODEM,
    VPCM_CALL_RUN_V91_MODEM
} vpcm_call_run_mode_t;

typedef struct {
    vpcm_call_params_t params;
    vpcm_g711_stream_t tx_stream;
    vpcm_g711_stream_t rx_stream;
    vpcm_call_state_t state;
    vpcm_call_run_mode_t run_mode;
    vpcm_v91_session_t v91_session;
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
bool vpcm_call_step(vpcm_call_t *call);
bool vpcm_call_step_to_next_state(vpcm_call_t *call);
bool vpcm_call_step_until(vpcm_call_t *call, vpcm_call_state_t target_state);
bool vpcm_call_step_to_run_mode(vpcm_call_t *call, vpcm_call_run_mode_t run_mode);
bool vpcm_call_attach_stream_taps(vpcm_call_t *call,
                                  const char *tx_path,
                                  const char *rx_path);
void vpcm_call_detach_stream_taps(vpcm_call_t *call);

void vpcm_call_set_state(vpcm_call_t *call, vpcm_call_state_t state);
void vpcm_call_set_run_mode(vpcm_call_t *call, vpcm_call_run_mode_t run_mode);
void vpcm_call_set_v91_state(vpcm_call_t *call, vpcm_v91_modem_state_t state);
const char *vpcm_call_state_to_str(vpcm_call_state_t state);
const char *vpcm_call_run_mode_to_str(vpcm_call_run_mode_t run_mode);

size_t vpcm_call_frame_bytes(const vpcm_call_t *call);
double vpcm_call_tick_seconds(const vpcm_call_t *call);

#endif
