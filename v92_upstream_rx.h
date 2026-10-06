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

/* Candidate B1u alignments followed at once in the equalised mode. */
#define V92_B1U_CANDIDATES 8
/* Frames of B1u an equalised-mode candidate must decode before it is
 * believed.  Not all 48: the receiver can only start once the analogue
 * modem's acknowledged SUVu' has been decoded and the equaliser switched to
 * the data levels, and against slmodemd that is ~15 frames into B1u
 * (slm-r10-v92-k3).  B1u decodes exactly from mid-stream with zeroed
 * memories (no precoder/prefilter memory; the scrambler self-synchronises),
 * and 16 frames of >= 34 bits of ones cannot happen by chance. */
#define V92_B1U_LOCK_FRAMES 16
/* Bits at the start of a candidate's first frame not judged: the GPA
 * descrambler's memory, unknown when B1u is joined part way through. */
#define V92_B1U_DESCRAMBLER_BITS 23

typedef struct {
    v92_upstream_wave_rx_t state;  /* memories zero at B1u's first symbol */
    double frame[V92_UPSTREAM_INTERVALS];
    int frame_pos;
    int frames;                    /* B1u frames decoded so far */
    int zero_bits;                 /* decoded bits that were not one */
} v92_b1u_candidate_t;

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

    /* Equalised mode (v92_upstream_b1_rx_init_equalized()). */
    bool equalized_input;
    double recent[V92_UPSTREAM_INTERVALS];  /* last 12 inputs, oldest first */
    int recent_fill;
    v92_b1u_candidate_t candidates[V92_B1U_CANDIDATES];
    int ncandidates;
    uint64_t candidates_started;            /* frames tried as B1u frame 0 */
    uint64_t candidates_passed;             /* ... that decoded to all ones */
    int best_frames;                        /* most B1u frames one candidate
                                               survived */
    int best_first_ones;                    /* most ones any frame-0 trial
                                               decoded (of K) */
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

/*
 * Equalised mode: the caller's equaliser has already removed the channel and
 * hands over samples in the units of v92_upstream_wave_decode_frame(), i.e.
 * G x v for the CPd's own G and points.  Lock does not compare the input
 * with a reference waveform: 6.4.2 lets the transmitter pick ANY member of
 * the equivalence class E(Ki), so a waveform built with our encoder's choice
 * matches only a peer that chooses identically.  Instead every alignment is
 * decoded -- nearest point, then Ki from eta mod Mi through the same trellis
 * decoder data mode uses, from the zero memories 8.7.1 puts at B1u's first
 * symbol -- and B1u is the alignment whose frames descramble to ones,
 * V92_B1U_LOCK_FRAMES of them, which need not start at B1u's first frame.
 * After lock every frame is decoded and delivered (what is left of B1u
 * reaches the DTE as idle ones); there is no internal equaliser or
 * decision-directed update.
 */
bool v92_upstream_b1_rx_init_equalized(v92_upstream_rx_t *rx,
                                       const v92_cpd_frame_t *cpd,
                                       v92_upstream_byte_handler_t handler,
                                       void *user_data);

/* Equalised-mode input; returns payload bytes delivered. */
int v92_upstream_b1_rx_feed_values(v92_upstream_rx_t *rx,
                                   const double *values,
                                   int count);

#ifdef __cplusplus
}
#endif

#endif /* V92_UPSTREAM_RX_H */
