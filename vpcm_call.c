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
    vpcm_v91_session_init(&call->v91_session, normalized.law);

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
    vpcm_v91_session_reset(&call->v91_session);
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
    if (!vpcm_call_state_allows_run_mode(state)) {
        call->run_mode = VPCM_CALL_RUN_NONE;
        vpcm_v91_session_reset(&call->v91_session);
    }
}

void vpcm_call_set_run_mode(vpcm_call_t *call, vpcm_call_run_mode_t run_mode)
{
    if (!call)
        return;
    if (call->state != VPCM_CALL_RUN && run_mode != VPCM_CALL_RUN_NONE)
        call->state = VPCM_CALL_RUN;
    call->run_mode = run_mode;
    if (run_mode != VPCM_CALL_RUN_V91_MODEM)
        vpcm_v91_session_reset(&call->v91_session);
}

void vpcm_call_set_v91_state(vpcm_call_t *call, vpcm_v91_modem_state_t state)
{
    if (!call)
        return;
    if (call->run_mode != VPCM_CALL_RUN_V91_MODEM)
        call->run_mode = VPCM_CALL_RUN_V91_MODEM;
    if (call->state != VPCM_CALL_RUN)
        call->state = VPCM_CALL_RUN;
    switch (state) {
    case VPCM_V91_MODEM_PHASE1:
        vpcm_v91_session_enter_phase1(&call->v91_session);
        break;
    case VPCM_V91_MODEM_INFO:
        vpcm_v91_session_enter_info(&call->v91_session);
        break;
    case VPCM_V91_MODEM_DIL:
        vpcm_v91_session_enter_dil(&call->v91_session);
        break;
    case VPCM_V91_MODEM_SCR:
        vpcm_v91_session_enter_scr(&call->v91_session);
        break;
    case VPCM_V91_MODEM_CP:
        vpcm_v91_session_enter_cp(&call->v91_session);
        break;
    case VPCM_V91_MODEM_B1:
        vpcm_v91_session_enter_b1(&call->v91_session);
        break;
    case VPCM_V91_MODEM_DATA:
        vpcm_v91_session_enter_data(&call->v91_session);
        break;
    case VPCM_V91_MODEM_CLEARDOWN:
        vpcm_v91_session_enter_cleardown(&call->v91_session);
        break;
    default:
        vpcm_v91_session_set_state(&call->v91_session, state);
        break;
    }
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

bool vpcm_call_step_to_next_state(vpcm_call_t *call)
{
    if (!call)
        return false;

    switch (call->state) {
    case VPCM_CALL_IDLE:
        call->state = VPCM_CALL_WAIT_DIALTONE;
        call->run_mode = VPCM_CALL_RUN_NONE;
        vpcm_v91_session_reset(&call->v91_session);
        break;
    case VPCM_CALL_WAIT_DIALTONE:
        call->state = VPCM_CALL_DIAL;
        break;
    case VPCM_CALL_DIAL:
        call->state = VPCM_CALL_WAIT_RINGING;
        break;
    case VPCM_CALL_WAIT_RINGING:
        call->state = VPCM_CALL_ANSWER;
        break;
    case VPCM_CALL_ANSWER:
        call->state = VPCM_CALL_RUN;
        break;
    case VPCM_CALL_RUN:
        call->state = VPCM_CALL_HANGUP;
        call->run_mode = VPCM_CALL_RUN_NONE;
        vpcm_v91_session_set_state(&call->v91_session, VPCM_V91_MODEM_CLEARDOWN);
        break;
    case VPCM_CALL_HANGUP:
        call->state = VPCM_CALL_DONE;
        call->run_mode = VPCM_CALL_RUN_NONE;
        vpcm_v91_session_reset(&call->v91_session);
        break;
    case VPCM_CALL_DONE:
        break;
    default:
        return false;
    }

    return vpcm_call_step(call);
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
