/* x2 Draft 0.33: the analogue client after its marker (clauses 3.4, 3.5,
 * 6, 18, 20, 21), as the Courier 403 runs it against a digital server.
 *
 * Upstream is ordinary V.34 at the announced 3200 baud / high carrier
 * (Draft 0.33 3.4; V.34 10.1.3), sequenced here and modulated by SpanDSP
 * through v34_tx_start_external_symbols():
 *   60 ms silence, S 128T, S-bar 16T, PP 288T, 4-point TRN (GPA scrambled
 *   ones), 16-point J (0x89b0, V.34 Table 18) on a timer, silence;
 *   on the server's J word: S 128T, S-bar 16T, 16-point TRN, the four-word
 *   MP (clause 21) repeated, MP with the acknowledge bit once the server's
 *   record has arrived, E (20 ones), then V.34 B1/data.
 *
 * Downstream is the server's PCMU script (clause 3.5), decoded in the
 * codeword domain -- the bearer delivers the server's octets, so there is no
 * equalizer: B (GPC scrambled ones on b1/31) ending in the 12-bit J word,
 * C, D, the E training mapper, the three-word record in the training
 * banks, one frame of ones, then the data mapper in the record's banks
 * (clauses 10, 11, 18). The record's frame phase is found by decoding all
 * six phases; our own server's 54-octet alignment word also sets it. */
#ifndef X2_CLIENT_H
#define X2_CLIENT_H
#include <stdbool.h>
#include "x2.h"

typedef enum {
    X2C_UP_SILENCE, X2C_UP_S, X2C_UP_S_BAR, X2C_UP_PP, X2C_UP_TRN, X2C_UP_J,
    X2C_UP_WAIT_J, X2C_UP_S2, X2C_UP_S_BAR2, X2C_UP_TRN16, X2C_UP_MP,
    X2C_UP_E, X2C_UP_DATA
} x2_client_up_t;
typedef enum {
    X2C_DN_WAIT_B, X2C_DN_B, X2C_DN_TRAIN, X2C_DN_RECORD, X2C_DN_DATA,
    X2C_DN_FAILED
} x2_client_dn_t;
typedef void (*x2_client_put_bit_t)(void *context, unsigned bit);

/* One frame-phase decoder: six-octet frames through the training or data
 * banks, GPC descrambling and the record framer (clause 20 framing: 17
 * ones, 0, words with 0 separators, LSB-first CRC 8408 from FFFF). */
typedef struct {
    uint8_t frame[6];
    unsigned skip, frame_pos, errors;
    x2_pcm_state_t parity;
    x2_scrambler_t descrambler;
    uint64_t bits, record_start;
    uint8_t rbits[67];
    unsigned rones, rpos, record_repeats, record_valid;
    uint16_t record[3];
} x2_client_decoder_t;

typedef struct {
    /* Our MP (clause 21): W1 = N1<<2 | N2<<6, W2 = V.34 upstream mask,
       W3 = 0, W4 = MD<<8. */
    x2_mp_t mp;
    uint16_t pcm_mask;          /* local downstream index mask, bit 0 = index 1 */
    unsigned md;
    /* Upstream symbol source. */
    x2_client_up_t up;
    unsigned up_symbols;
    uint64_t up_total;
    uint32_t scramble;          /* GPA, V.34 equation 7-2 */
    unsigned diff, s_index;
    uint8_t mp_bits[X2_MP_BITS];
    unsigned mp_pos, ack_frames, ack_pending, j_pos;
    /* Downstream receiver. */
    x2_client_dn_t dn;
    uint64_t dn_samples;
    unsigned run, b_bits;
    uint64_t history;           /* descrambled B bits, newest at bit 0 */
    x2_scrambler_t descrambler;
    uint8_t ring[54];
    unsigned ring_pos, ring_count, record_phase;
    x2_pcm_config_t training, data;
    /* During E training, all six frame phases; then the one carrying the
       record. */
    x2_client_decoder_t phase[6], main;
    unsigned decode_errors;
    uint16_t record[3];
    /* Results. */
    unsigned j_seen, record_valid, final_seen, e_sent;
    unsigned selected_index, upstream_n;
    uint64_t data_bits;
    x2_client_put_bit_t put_bit;
    void *put_context;
} x2_client_t;

/* pcm_mask: the downstream indices we accept (N1 is its highest);
 * upstream_mask: V.34 N mask for W2 (bit 0 = 2400). Returns -1 if empty. */
int x2_client_init(x2_client_t *c, uint16_t pcm_mask, uint16_t upstream_mask, unsigned md);
void x2_client_set_put_bit(x2_client_t *c, x2_client_put_bit_t put_bit, void *context);
/* v34_tx_external_symbol_func_t: one symbol per baud, constellation steps. */
void x2_client_get_symbol(void *user_data, float *re, float *im);
/* done_fn for v34_v90_arm_tx_data(): E has gone out. */
bool x2_client_data_due(void *user_data);
void x2_client_rx_codewords(x2_client_t *c, const uint8_t *codewords, size_t count);
/* Downstream bit rate of the selected record, 0 before it. */
unsigned x2_client_downstream_rate(const x2_client_t *c);
const char *x2_client_up_name(x2_client_up_t stage);
const char *x2_client_dn_name(x2_client_dn_t stage);
#endif
