#include "vpcm_call.h"

#include <string.h>

static bool vpcm_call_state_allows_run_mode(vpcm_call_state_t state)
{
    return state == VPCM_CALL_RUN;
}

const char *vpcm_call_state_to_str(vpcm_call_state_t state)
{
    switch (state) {
    case VPCM_CALL_IDLE: return "IDLE";
    case VPCM_CALL_WAIT_DIALTONE: return "WAIT_DIALTONE";
    case VPCM_CALL_DIAL: return "DIAL";
    case VPCM_CALL_WAIT_RINGING: return "WAIT_RINGING";
    case VPCM_CALL_ANSWER: return "ANSWER";
    case VPCM_CALL_RUN: return "RUN";
    case VPCM_CALL_HANGUP: return "HANGUP";
    case VPCM_CALL_DONE: return "DONE";
    default: return "UNKNOWN";
    }
}

const char *vpcm_call_run_mode_to_str(vpcm_call_run_mode_t run_mode)
{
    switch (run_mode) {
    case VPCM_CALL_RUN_NONE: return "NONE";
    case VPCM_CALL_RUN_V90_MODEM: return "V90_MODEM";
    case VPCM_CALL_RUN_V91_MODEM: return "V91_MODEM";
    default: return "UNKNOWN";
    }
}

bool vpcm_call_params_normalize(vpcm_call_params_t *params)
{
    if (!params)
        return false;
    if (params->sample_rate == 0)
        params->sample_rate = VPCM_G711_STREAM_DEFAULT_SAMPLE_RATE;
    if (params->frame_samples == 0)
        params->frame_samples = VPCM_G711_STREAM_DEFAULT_FRAME_SAMPLES;
    return params->sample_rate > 0 && params->frame_samples > 0;
}

bool vpcm_call_init(vpcm_call_t *call,
                    const vpcm_call_params_t *params,
                    uint8_t *tx_storage,
                    size_t tx_storage_len,
                    uint8_t *rx_storage,
                    size_t rx_storage_len)
{
    vpcm_call_params_t normalized;
    vpcm_g711_stream_params_t tx_params;
    vpcm_g711_stream_params_t rx_params;

    if (!call || !params || !tx_storage || !rx_storage)
        return false;
    normalized = *params;
    if (!vpcm_call_params_normalize(&normalized))
        return false;

    memset(call, 0, sizeof(*call));
    call->params = normalized;
    call->state = VPCM_CALL_IDLE;
    call->run_mode = VPCM_CALL_RUN_NONE;

    memset(&tx_params, 0, sizeof(tx_params));
    tx_params.law = normalized.law;
    tx_params.sample_rate = normalized.sample_rate;
    tx_params.frame_samples = normalized.frame_samples;
    tx_params.label = "tx";

    memset(&rx_params, 0, sizeof(rx_params));
    rx_params.law = normalized.law;
    rx_params.sample_rate = normalized.sample_rate;
    rx_params.frame_samples = normalized.frame_samples;
    rx_params.label = "rx";

    if (!vpcm_g711_stream_init(&call->tx_stream, &tx_params, tx_storage, tx_storage_len))
        return false;
    if (!vpcm_g711_stream_init(&call->rx_stream, &rx_params, rx_storage, rx_storage_len))
        return false;
    return true;
}

void vpcm_call_reset(vpcm_call_t *call)
{
    if (!call)
        return;
    vpcm_g711_stream_reset(&call->tx_stream);
    vpcm_g711_stream_reset(&call->rx_stream);
    call->state = VPCM_CALL_IDLE;
    call->run_mode = VPCM_CALL_RUN_NONE;
    call->ticks = 0;
    call->elapsed_seconds = 0.0;
}

bool vpcm_call_advance_tick(vpcm_call_t *call)
{
    if (!call)
        return false;
    call->ticks++;
    call->elapsed_seconds += vpcm_call_tick_seconds(call);
    return true;
}

void vpcm_call_set_state(vpcm_call_t *call, vpcm_call_state_t state)
{
    if (!call)
        return;
    call->state = state;
    if (!vpcm_call_state_allows_run_mode(state))
        call->run_mode = VPCM_CALL_RUN_NONE;
}

void vpcm_call_set_run_mode(vpcm_call_t *call, vpcm_call_run_mode_t run_mode)
{
    if (!call)
        return;
    if (call->state != VPCM_CALL_RUN && run_mode != VPCM_CALL_RUN_NONE)
        call->state = VPCM_CALL_RUN;
    call->run_mode = run_mode;
}

bool vpcm_call_step(vpcm_call_t *call)
{
    if (!call)
        return false;

    switch (call->state) {
    case VPCM_CALL_IDLE:
    case VPCM_CALL_WAIT_DIALTONE:
    case VPCM_CALL_DIAL:
    case VPCM_CALL_WAIT_RINGING:
    case VPCM_CALL_ANSWER:
    case VPCM_CALL_RUN:
    case VPCM_CALL_HANGUP:
        return vpcm_call_advance_tick(call);
    case VPCM_CALL_DONE:
        return true;
    default:
        return false;
    }
}

size_t vpcm_call_frame_bytes(const vpcm_call_t *call)
{
    if (!call)
        return 0;
    return vpcm_g711_stream_frame_bytes(&call->tx_stream);
}

double vpcm_call_tick_seconds(const vpcm_call_t *call)
{
    if (!call)
        return 0.0;
    return vpcm_g711_stream_frame_seconds(&call->tx_stream);
}
