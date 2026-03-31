#include "vpcm_g711_stream.h"

#include <string.h>

bool vpcm_g711_stream_params_normalize(vpcm_g711_stream_params_t *params)
{
    if (!params)
        return false;
    if (params->sample_rate == 0)
        params->sample_rate = VPCM_G711_STREAM_DEFAULT_SAMPLE_RATE;
    if (params->frame_samples == 0)
        params->frame_samples = VPCM_G711_STREAM_DEFAULT_FRAME_SAMPLES;
    return params->sample_rate > 0 && params->frame_samples > 0;
}

bool vpcm_g711_stream_init(vpcm_g711_stream_t *stream,
                           const vpcm_g711_stream_params_t *params,
                           uint8_t *storage,
                           size_t storage_len)
{
    vpcm_g711_stream_params_t normalized;

    if (!stream || !params || !storage || storage_len == 0)
        return false;
    normalized = *params;
    if (!vpcm_g711_stream_params_normalize(&normalized))
        return false;

    memset(stream, 0, sizeof(*stream));
    stream->params = normalized;
    stream->buf = storage;
    stream->capacity = storage_len;
    return true;
}

void vpcm_g711_stream_reset(vpcm_g711_stream_t *stream)
{
    if (!stream)
        return;
    stream->len = 0;
    stream->total_octets_in = 0;
    stream->total_octets_out = 0;
    stream->frame_counter = 0;
}

bool vpcm_g711_stream_attach_tap(vpcm_g711_stream_t *stream, const char *path)
{
    if (!stream || !path || !*path)
        return false;
    vpcm_g711_stream_detach_tap(stream);
    stream->tap_file = fopen(path, "wb");
    return stream->tap_file != NULL;
}

void vpcm_g711_stream_detach_tap(vpcm_g711_stream_t *stream)
{
    if (!stream)
        return;
    if (stream->tap_file) {
        fclose(stream->tap_file);
        stream->tap_file = NULL;
    }
}

size_t vpcm_g711_stream_frame_bytes(const vpcm_g711_stream_t *stream)
{
    if (!stream)
        return 0;
    return (size_t) stream->params.frame_samples;
}

double vpcm_g711_stream_frame_seconds(const vpcm_g711_stream_t *stream)
{
    if (!stream || stream->params.sample_rate == 0)
        return 0.0;
    return (double) stream->params.frame_samples / (double) stream->params.sample_rate;
}

size_t vpcm_g711_stream_writable_octets(const vpcm_g711_stream_t *stream)
{
    if (!stream || stream->len >= stream->capacity)
        return 0;
    return stream->capacity - stream->len;
}

size_t vpcm_g711_stream_readable_octets(const vpcm_g711_stream_t *stream)
{
    if (!stream)
        return 0;
    return stream->len;
}

size_t vpcm_g711_stream_write(vpcm_g711_stream_t *stream,
                              const uint8_t *src,
                              size_t len)
{
    size_t writable;

    if (!stream || !src || len == 0)
        return 0;
    writable = vpcm_g711_stream_writable_octets(stream);
    if (len > writable)
        len = writable;
    if (len == 0)
        return 0;
    memcpy(stream->buf + stream->len, src, len);
    stream->len += len;
    stream->total_octets_in += len;
    if (stream->tap_file)
        (void) fwrite(src, 1, len, stream->tap_file);
    return len;
}

size_t vpcm_g711_stream_read(vpcm_g711_stream_t *stream,
                             uint8_t *dst,
                             size_t len)
{
    if (!stream || !dst || len == 0)
        return 0;
    if (len > stream->len)
        len = stream->len;
    if (len == 0)
        return 0;
    memcpy(dst, stream->buf, len);
    stream->len -= len;
    if (stream->len > 0)
        memmove(stream->buf, stream->buf + len, stream->len);
    stream->total_octets_out += len;
    return len;
}

bool vpcm_g711_stream_push_frame(vpcm_g711_stream_t *stream,
                                 const uint8_t *frame,
                                 size_t frame_len)
{
    size_t want;

    if (!stream || !frame)
        return false;
    want = vpcm_g711_stream_frame_bytes(stream);
    if (want == 0 || frame_len != want)
        return false;
    if (vpcm_g711_stream_write(stream, frame, frame_len) != frame_len)
        return false;
    stream->frame_counter++;
    return true;
}

bool vpcm_g711_stream_pop_frame(vpcm_g711_stream_t *stream,
                                uint8_t *frame_out,
                                size_t frame_len)
{
    size_t want;

    if (!stream || !frame_out)
        return false;
    want = vpcm_g711_stream_frame_bytes(stream);
    if (want == 0 || frame_len != want)
        return false;
    if (stream->len < frame_len)
        return false;
    return vpcm_g711_stream_read(stream, frame_out, frame_len) == frame_len;
}
