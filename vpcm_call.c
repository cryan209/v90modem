#include "vpcm_call.h"

#include <string.h>

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
