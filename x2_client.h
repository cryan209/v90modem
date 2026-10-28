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
 * six phases; our own server's 54-octet alignment word also sets it.
 *
 * In data the client can change the rate (x2_client_start_rate_change()),
 * as the Courier falls forward (docs/x2_implementation.md, "In-data fall
 * forward"; V.90 9.6.2.1's shape): S 128T and S-bar 16T at the data seam,
 * then the 4-point MP with the new N1, repeated, two acknowledged MPs once
 * the server's record carries its acknowledgement, E, and V.34 B1/data at
 * the record's upstream N. Downstream the server answers with its 54-octet
 * alignment word on the current banks, the record on a fresh mapper, one
 * frame of ones after our E, then data in the new banks; the alignment
 * word's nine frames are held back and never reach the DTE. */
#ifndef X2_CLIENT_H
#define X2_CLIENT_H
#include <stdbool.h>
#include "x2.h"

typedef enum {
    X2C_UP_SILENCE, X2C_UP_S, X2C_UP_S_BAR, X2C_UP_PP, X2C_UP_TRN, X2C_UP_J,
    X2C_UP_WAIT_J, X2C_UP_S2, X2C_UP_S_BAR2, X2C_UP_TRN16, X2C_UP_MP,
    X2C_UP_E, X2C_UP_DATA,
    X2C_UP_RR_S, X2C_UP_RR_S_BAR, X2C_UP_RR_MP, X2C_UP_RR_E
} x2_client_up_t;
typedef enum {
    X2C_DN_WAIT_B, X2C_DN_B, X2C_DN_TRAIN, X2C_DN_RECORD, X2C_DN_DATA,
    X2C_DN_FAILED, X2C_DN_RR_RECORD
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
    /* In-data rate change: active, its count, downstream codewords since
       our S, the decoders on the record's two candidate allocations
       (index 1 at the current MD, and the current banks), which one
       carries it, the new data banks, and the frames held back while the
       alignment word may be arriving. */
    unsigned rr, rate_changes, rr_decoder, drop_frames;
    uint64_t rr_samples, data_sample;
    x2_pcm_config_t rr_config[2], rr_next;
    x2_client_decoder_t rr_dec[2];
    uint8_t hold_bits[9][64];
    unsigned hold_count[9], hold_first, hold_frames;
    /* V.90 9.5.2.1 / V.34 11.5 retrain from the server: 2400 Hz Tone A
       with the 1800 Hz guard in place of data, 40-sample blocks. */
    int16_t tone_window[40];
    unsigned tone_pos, tone_a_blocks, tone_a;
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
/* Open an in-data rate change asking for PCM index `index` (its N1; the
 * server's record ceiling still applies). Only in data both ways, with no
 * change already open. The caller hands the modulator back with
 * v34_v90_resume_external_symbols() and re-arms V.34 data once
 * record_valid is set again. Returns -1 if it cannot start. */
int x2_client_start_rate_change(x2_client_t *c, unsigned index);
/* Whether a rate change could open now: data both ways, none open, and
 * the server's 4080 DATA_STARTUP samples over -- it listens for our S only
 * in PAYLOAD (x2_session.c). */
bool x2_client_can_change_rate(const x2_client_t *c);
/* Downstream bit rate of the selected record, 0 before it. */
unsigned x2_client_downstream_rate(const x2_client_t *c);
const char *x2_client_up_name(x2_client_up_t stage);
const char *x2_client_dn_name(x2_client_dn_t stage);
#endif
