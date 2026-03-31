#ifndef VPCM_G711_STREAM_H
#define VPCM_G711_STREAM_H

#include "v91.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>

#define VPCM_G711_STREAM_DEFAULT_SAMPLE_RATE 8000
#define VPCM_G711_STREAM_DEFAULT_FRAME_SAMPLES 64

typedef struct {
    v91_law_t law;
    unsigned sample_rate;
    unsigned frame_samples;
    const char *label;
} vpcm_g711_stream_params_t;

typedef struct {
    vpcm_g711_stream_params_t params;
    uint8_t *buf;
    size_t capacity;
    size_t len;
    FILE *tap_file;
    uint64_t total_octets_in;
    uint64_t total_octets_out;
    uint64_t frame_counter;
} vpcm_g711_stream_t;

bool vpcm_g711_stream_params_normalize(vpcm_g711_stream_params_t *params);
bool vpcm_g711_stream_init(vpcm_g711_stream_t *stream,
                           const vpcm_g711_stream_params_t *params,
                           uint8_t *storage,
                           size_t storage_len);
void vpcm_g711_stream_reset(vpcm_g711_stream_t *stream);
bool vpcm_g711_stream_attach_tap(vpcm_g711_stream_t *stream, const char *path);
void vpcm_g711_stream_detach_tap(vpcm_g711_stream_t *stream);

size_t vpcm_g711_stream_frame_bytes(const vpcm_g711_stream_t *stream);
double vpcm_g711_stream_frame_seconds(const vpcm_g711_stream_t *stream);

size_t vpcm_g711_stream_writable_octets(const vpcm_g711_stream_t *stream);
size_t vpcm_g711_stream_readable_octets(const vpcm_g711_stream_t *stream);

size_t vpcm_g711_stream_write(vpcm_g711_stream_t *stream,
                              const uint8_t *src,
                              size_t len);
size_t vpcm_g711_stream_read(vpcm_g711_stream_t *stream,
                             uint8_t *dst,
                             size_t len);

bool vpcm_g711_stream_push_frame(vpcm_g711_stream_t *stream,
                                 const uint8_t *frame,
                                 size_t frame_len);
bool vpcm_g711_stream_pop_frame(vpcm_g711_stream_t *stream,
                                uint8_t *frame_out,
                                size_t frame_len);

#endif
