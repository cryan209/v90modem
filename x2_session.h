#ifndef X2_SESSION_H
#define X2_SESSION_H
#include "x2.h"
#include "x2_mp_rx.h"
#include "x2_sym.h"
#include "x2_client.h"
/* Draft 0.33 clauses 3..6, 12..13 and 21. Asymmetric PCMU digital
 * answerer, the asymmetric analogue client that calls one (Phase 2 and the
 * marker here, the rest in x2_client.c), or symmetric digital startup in
 * either call role/law. */
typedef enum {
    X2_INFO0, X2_TONE_A, X2_PROBE, X2_MARKER_WAIT, X2_UPSTREAM_WAIT,
    X2_ZERO, X2_PATTERN_A, X2_TRAIN_B, X2_J, X2_J_ACK,
    X2_TRAIN_C, X2_TRAIN_D, X2_TRAIN_E, X2_RECORD_WAIT,
    X2_RECORD_ALIGN, X2_RECORD_TX,
    X2_FINAL_TRAINING, X2_DATA_STARTUP, X2_PAYLOAD, X2_SYMMETRIC,
    /* Client: wait out the server's probe, send the marker, then x2_client. */
    X2_CLIENT_PROBE, X2_CLIENT_MARKER, X2_CLIENT,
    X2_FAILED
} x2_session_stage_t;
/* Why the session reached X2_FAILED, where the cause is known. */
typedef enum {
    X2_FAILURE_NONE,
    X2_FAILURE_STAGE_TIMEOUT,   /* a startup stage ran 10 s */
    X2_FAILURE_PEER_RETRAIN,    /* the peer's retrain in data, not answered */
    X2_FAILURE_RENEG_TIMEOUT,   /* no E within X2_RENEG_DEADLINE of its S */
    X2_FAILURE_CLIENT           /* client: no usable record, or no data in time */
} x2_session_failure_t;
/* Samples after the Courier's S without its E. A completed change takes
 * 0.25 s to E; an unanswered one falls back to Tone A 1.57 s after S. */
#define X2_RENEG_DEADLINE 24000
typedef struct {
    unsigned clock, sync, count;
    double re, im, previous_re, previous_im;
    uint8_t bits[33];
} x2_info_hypothesis_t;
typedef struct {
    x2_session_stage_t stage;
    unsigned alaw, symmetric, answering, client;
    uint32_t info_body;         /* our 17-bit INFO0 body (ITU bits 12..28) */
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
    uint16_t downstream_rate_mask;
    unsigned record_position;
    uint8_t record_bits[96];
    /* In-data fall forward (V.90 9.6.2.1 shape, x2 server response):
       the Courier sends S for 128T then S-bar/SCR/MP/E; we answer with the
       RECORD_ALIGN sign pattern and the rate record in the current data
       banks, then restart DATA_STARTUP in the bank its new MP selects. */
    unsigned reneg, renegotiations, reneg_ack, reneg_record_started, record_ack_built;
    /* rx_samples at the end of the Courier's S for this rate change, and
       windows of sustained Tone A in data (docs/x2_implementation.md,
       "Bounding an unanswered rate change", 19 Oct). */
    uint64_t reneg_s_sample;
    /* rx_samples where the S now being tracked began (the first window of
       its coherent run), and that of the S that opened this rate change:
       upstream decoded after it is S, not the peer's data. */
    uint64_t s_onset, reneg_s_onset;
    /* Client marker carrier in Hz (0: 1200, the caller's); the retrains
       answered (x2_session_client_retrain) and the silence still to send
       before its Tone A. */
    unsigned marker_carrier, retrains, rt_silence;
    /* Server: the most retrains a call answers or starts
       (x2_session_server_retrain); 0, the default, hangs up on Tone A. */
    unsigned max_retrains;
    /* Server retrain, the Tone B side (9.2.1.1.3-.4): set until the probe;
       the analogue modem's second Tone A reversal heard, and when our probe
       starts (14 ms after it, as Ie030002's does: artifacts/x2-rt-answer3). */
    unsigned server_rt, a_second_seen;
    uint64_t probe_tx_sample;
    unsigned tone_a_windows;
    x2_session_failure_t failure;
    x2_session_stage_t failed_stage;
    x2_pcm_config_t reneg_config;
    x2_get_bit_func_t payload_source;
    void *payload_context;
    /* Client: the server's 160 ms probe (V.34 10.1.2.4) before our marker. */
    unsigned probe_loud, probe_quiet, probe_ended;
    double probe_energy;
    uint8_t marker_bits[41];
    x2_client_t cl;
} x2_session_t;
int x2_info_encode(uint32_t body, unsigned body_bits, uint8_t *bits);
int x2_info_decode(const uint8_t *bits, unsigned body_bits, uint32_t *body);
int x2_marker_parse(unsigned marker, unsigned server_answering, unsigned *index, unsigned *high_carrier);
int x2_session_init(x2_session_t *session);
int x2_session_init_symmetric(x2_session_t *session, unsigned alaw, unsigned answering);
/* Asymmetric analogue client (PCMU, calling): INFO0 21ff, marker 4d.
 * pcm_mask: downstream indices we accept; upstream_mask: V.34 N mask. */
int x2_session_init_client(x2_session_t *session, uint16_t pcm_mask,
                           uint16_t upstream_mask, unsigned md);
void x2_session_rx(x2_session_t *session, const int16_t *samples, size_t count);
void x2_session_set_payload_source(x2_session_t *session, x2_get_bit_func_t get_bit, void *context);
void x2_session_receive_mp(x2_session_t *session, const x2_mp_t *mp);
void x2_session_upstream_j(x2_session_t *session);
void x2_session_upstream_s_bar(x2_session_t *session);
/* Non-zero while an in-data rate change is running (engine re-arms B1). */
unsigned x2_session_renegotiating(const x2_session_t *session);
void x2_session_fail(x2_session_t *session, x2_session_failure_t why);
size_t x2_session_tx(x2_session_t *session, uint8_t *octets, size_t count);
const char *x2_session_stage_name(x2_session_stage_t stage);
const char *x2_session_failure_name(x2_session_failure_t failure);
/* Client in data: the server has retrained (V.90 9.5.1.1, 70 ms of silence
 * then Tone B; x2_client.c tone_b). Answer as Ie030002 runs it (V.90
 * 9.5.2.2, traced 28 Oct, docs/x2_implementation.md): 70 ms of silence,
 * Tone A, the 9.2.2.1.3-.5 reversals against its Tone B, silence through
 * its probe, then the marker with the indices the other way round (69) on
 * our Tone A carrier, 2400 Hz, and the client's startup from X2_CLIENT
 * again. Returns -1 unless the session is a client in X2_CLIENT. */
int x2_session_client_retrain(x2_session_t *s);
/* Server in data: retrain (V.90 9.5.1.1), or answer the analogue modem's
 * Tone A (9.5.1.2, which x2_session_rx does itself while retrains <
 * max_retrains). Both send 70 ms of silence then Tone B and go on at
 * 9.2.1.1.3, the roles of x2_session_client_retrain the other way round:
 * our Tone B on 1200 Hz against its Tone A, its first reversal answered
 * 40 ms later, 10 ms more, our probe, then its marker 69 heard on 2400 Hz
 * and Phase 3 from X2_UPSTREAM_WAIT as at startup. Returns -1 unless the
 * session is a server in data (PAYLOAD, or a rate change's RECORD_ALIGN or
 * RECORD_TX). */
int x2_session_server_retrain(x2_session_t *s);
#endif
