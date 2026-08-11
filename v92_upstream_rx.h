/*
 * v92_upstream_rx.h — B1u acquisition and PCM-upstream frame delivery
 *
 * ITU-T V.92 §8.7.1 and §9.6.1.1.6.  B1u's 48 known data frames establish
 * frame interval zero and the received linear gain before data decoding.
 */

#ifndef V92_UPSTREAM_RX_H
#define V92_UPSTREAM_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "v92_upstream_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define V92_B1U_FRAMES 48
#define V92_B1U_SYMBOLS (V92_B1U_FRAMES*V92_UPSTREAM_INTERVALS)
#define V92_UPSTREAM_EQ_TAPS 7

typedef void (*v92_upstream_byte_handler_t)(void *user_data, uint8_t byte);

typedef struct {
    v92_cpd_frame_t cpd;
    v92_upstream_wave_rx_t wave_rx;
    v92_upstream_wave_rx_t b1_final_rx;
    v92_upstream_wave_tx_t decision_tx;
    v92_upstream_wave_tx_t b1_final_tx;
    double reference[V92_B1U_SYMBOLS];
    double acquisition[V92_B1U_SYMBOLS];
    int acquisition_pos;
    int acquisition_count;
    double frame[V92_UPSTREAM_INTERVALS];
    double frame_inputs[V92_UPSTREAM_INTERVALS][V92_UPSTREAM_EQ_TAPS];
    int frame_pos;
    double gain;
    double offset;
    double correlation;
    double equalizer[V92_UPSTREAM_EQ_TAPS];
    double equalizer_offset;
    double equalizer_history[V92_UPSTREAM_EQ_TAPS];
    int equalizer_delay;
    int equalizer_discard;
    bool equalizer_trained;
    bool locked;
    uint8_t byte_accumulator;
    int byte_bits;
    uint64_t input_symbols;
    uint64_t output_bits;
    uint64_t output_bytes;
    uint64_t rejected_frames;
    uint64_t equalizer_updates;
    v92_upstream_byte_handler_t handler;
    void *user_data;
} v92_upstream_rx_t;

bool v92_upstream_b1_rx_init(v92_upstream_rx_t *rx,
                          const v92_cpd_frame_t *cpd,
                          v92_upstream_byte_handler_t handler,
                          void *user_data);

/* Feed signed-linear samples decoded from the G.711 bearer.  Returns the
 * number of complete payload bytes delivered during this call. */
int v92_upstream_b1_rx_feed(v92_upstream_rx_t *rx,
                         const int16_t *samples,
                         int count);

#ifdef __cplusplus
}
#endif

#endif /* V92_UPSTREAM_RX_H */
