#ifndef X2_SESSION_H
#define X2_SESSION_H
#include "x2.h"
#include "x2_mp_rx.h"
#include "x2_sym.h"
/* Draft 0.33 clauses 3..6, 12..13 and 21. Asymmetric PCMU digital
 * answerer, or symmetric digital startup in either call role/law. */
typedef enum {
    X2_INFO0, X2_TONE_A, X2_PROBE, X2_MARKER_WAIT, X2_UPSTREAM_WAIT,
    X2_ZERO, X2_PATTERN_A, X2_TRAIN_B, X2_J, X2_J_ACK,
    X2_TRAIN_C, X2_TRAIN_D, X2_TRAIN_E, X2_RECORD_WAIT,
    X2_RECORD_ALIGN, X2_RECORD_TX,
    X2_FINAL_TRAINING, X2_DATA_STARTUP, X2_PAYLOAD, X2_SYMMETRIC, X2_FAILED
} x2_session_stage_t;
typedef struct {
    unsigned clock, sync, count;
    double re, im, previous_re, previous_im;
    uint8_t bits[33];
} x2_info_hypothesis_t;
typedef struct {
    x2_session_stage_t stage;
    unsigned alaw, symmetric, answering;
    x2_sym_link_t sym;
    uint64_t tx_samples, rx_samples;
    unsigned stage_samples, info_position, info_clock, sign;
    uint8_t info_bits[49], marker;
    unsigned peer_info_valid, marker_valid, upstream_ready, s_bar_seen;
    unsigned rejected_frames, accepted_frames;
    uint16_t peer_capabilities;
    /* V.34 11.2.1.2: receive Tone B, reverse A, receive B reversal,
       then reverse A 40 ms later and end it after 10 ms. */
    int16_t probe[160];
    int16_t b_window[40];
    unsigned b_position, b_samples, b_stable, b_present;
    unsigned a_reversals, b_reversed, b_crossing_valid;
    double b_reference_re, b_reference_im;
    uint64_t b_crossing_sample, second_a_tx_sample;
    x2_info_hypothesis_t info_rx[40];
    int16_t s_window[40];
    unsigned s_position, s_samples, s_stable, s_missing;
    double s_reference_re[3], s_reference_im[3];
    x2_scrambler_t training_scrambler;
    x2_pcm_tx_t training_mapper;
    x2_mp_rx_t mp_rx;
    x2_mp_t peer_mp;
    x2_pcm_config_t data_config;
    unsigned mp_valid, selected_index, upstream_rate_n;
    uint16_t upstream_rate_mask;
    unsigned record_position;
    uint8_t record_bits[96];
    x2_get_bit_func_t payload_source;
    void *payload_context;
} x2_session_t;
int x2_info_encode(uint32_t body, unsigned body_bits, uint8_t *bits);
int x2_info_decode(const uint8_t *bits, unsigned body_bits, uint32_t *body);
int x2_marker_parse(unsigned marker, unsigned server_answering, unsigned *index, unsigned *high_carrier);
int x2_session_init(x2_session_t *session);
int x2_session_init_symmetric(x2_session_t *session, unsigned alaw, unsigned answering);
void x2_session_rx(x2_session_t *session, const int16_t *samples, size_t count);
void x2_session_set_payload_source(x2_session_t *session, x2_get_bit_func_t get_bit, void *context);
void x2_session_receive_mp(x2_session_t *session, const x2_mp_t *mp);
void x2_session_upstream_j(x2_session_t *session);
void x2_session_upstream_s_bar(x2_session_t *session);
size_t x2_session_tx(x2_session_t *session, uint8_t *octets, size_t count);
const char *x2_session_stage_name(x2_session_stage_t stage);
#endif
