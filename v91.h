/*
 * v91.h - V.91 PCM modem helpers
 *
 * This module provides the G.711 bearer/data-plane seam for V.91.
 * It currently covers:
 *   - transparent-mode codeword transport
 *   - Phase 1/V.91 transition silence
 *   - Ez
 *   - INFO/INFO'
 *   - Eu/Em, J, PHIL, SCR, CP, Es and B1 generation/reception
 *
 *   - frame-loss recovery, retrain and cleardown state transitions
 */

#ifndef V91_H
#define V91_H

#include "vpcm_cp.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    V91_LAW_ULAW = 0,
    V91_LAW_ALAW = 1
} v91_law_t;

typedef enum {
    V91_MODE_TRANSPARENT = 0
} v91_mode_t;

#define V91_PHASE1_SILENCE_SYMBOLS 600
#define V91_EU_SYMBOLS 12
#define V91_EM_SYMBOLS 12
#define V91_ES_SYMBOLS 12
#define V91_B1_SYMBOLS 12
#define V91_EZ_SYMBOLS 24
#define V91_INFO_SYMBOLS 62
#define V91_DEFAULT_DIL_SEGMENTS 125
#define V91_DEFAULT_DIL_SEGMENT_SYMBOLS 12
#define V91_DEFAULT_DIL_SYMBOLS (V91_DEFAULT_DIL_SEGMENTS * V91_DEFAULT_DIL_SEGMENT_SYMBOLS)
#define V91_J_MAX_BITS 4096
#define V91_ROBBED_BIT_DETECT_MIN_FLIPS 4

typedef struct {
    uint16_t reserved_12_25; /* Raw 14-bit reserved field */
    bool request_default_dil;
    bool request_control_channel;
    bool acknowledge_info_frame;
    uint8_t reserved_29_32; /* Raw 4-bit reserved field */
    uint8_t max_tx_power; /* Raw 5-bit field, 0..31 */
    bool power_measured_after_digital_impairments;
    bool tx_uses_alaw;
    bool request_transparent_mode;
    bool cleardown_if_transparent_denied;
} v91_info_frame_t;

typedef struct {
    v91_info_frame_t frame;
    uint8_t bits[V91_INFO_SYMBOLS];
    uint8_t codewords[V91_INFO_SYMBOLS];
    uint16_t crc_field;
    uint16_t crc_remainder;
    bool fill_ok;
    bool sync_ok;
    bool valid;
} v91_info_diag_t;

typedef struct {
    uint8_t n;
    uint8_t lsp;
    uint8_t ltp;
    uint8_t sp[128];
    uint8_t tp[128];
    uint8_t h[8];
    uint8_t ref[8];
    uint8_t train_u[255];
} v91_dil_desc_t;

typedef struct {
    uint8_t n;
    uint8_t lsp;
    uint8_t ltp;
    uint8_t unique_train_u;
    uint8_t repeated_uchords;
    uint8_t non_default_refs;
    uint8_t non_default_h;
    uint8_t impairment_score;
    bool default_like;
    bool robbed_bit_limited;
    bool echo_limited;
    uint8_t recommended_downstream_drn;
    uint8_t recommended_upstream_drn;
} v91_dil_analysis_t;

typedef struct {
    v91_law_t  law;
    v91_mode_t mode;
    bool last_tx_info_valid;
    bool last_rx_info_valid;
    v91_info_frame_t last_tx_info;
    v91_info_frame_t last_rx_info;
    bool last_tx_dil_valid;
    bool last_rx_dil_valid;
    bool last_rx_dil_analysis_valid;
    v91_dil_desc_t last_tx_dil;
    v91_dil_desc_t last_rx_dil;
    v91_dil_analysis_t last_rx_dil_analysis;
    bool last_tx_cp_valid;
    bool last_rx_cp_valid;
    vpcm_cp_frame_t last_tx_cp;
    vpcm_cp_frame_t last_rx_cp;
    uint32_t scramble_reg;
    uint32_t rx_scramble_reg;
    int diff_sign;
    int rx_prev_sign;
    bool frame_aligned;
    bool rx_robbed_bit_detected;
    uint8_t rx_robbed_slot_mask;
    bool retrain_requested;
    bool retrain_timer_active;
    bool rx_data_clamped;
    bool cleardown_requested;
    bool connection_terminated;
    bool circuit_106_on;
    bool circuit_107_on;
    bool circuit_109_on;
    int next_frame_interval;
    bool data_mode_active;
    vpcm_cp_frame_t active_cp;
    int active_k;
    int active_s;
    int active_primary_bits_per_frame;
    bool active_transparent_mode;
    uint64_t tx_bit_accum;
    int tx_bit_count;
    uint64_t rx_bit_accum;
    int rx_bit_count;
} v91_state_t;

typedef enum {
    V91_ALIGN_NONE = 0,
    V91_ALIGN_EU = 1,
    V91_ALIGN_EM = 2
} v91_align_signal_t;

void v91_init(v91_state_t *s, v91_law_t law, v91_mode_t mode);

uint8_t v91_idle_codeword(v91_law_t law);
int16_t v91_codeword_to_linear(v91_law_t law, uint8_t codeword);
uint8_t v91_linear_to_codeword(v91_law_t law, int16_t sample);
uint8_t v91_ucode_to_codeword(v91_law_t law, int ucode, bool positive);
int v91_codeword_to_ucode(v91_law_t law, uint8_t codeword);

int v91_tx_phase1_silence_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
int v91_tx_eu_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
bool v91_rx_eu_codewords(v91_state_t *s, const uint8_t *g711_in, int g711_len);
int v91_tx_em_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
bool v91_rx_em_codewords(v91_state_t *s, const uint8_t *g711_in, int g711_len);
int v91_tx_ez_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
bool v91_rx_ez_codewords(v91_state_t *s, const uint8_t *g711_in, int g711_len);
int v91_tx_phil_codewords(v91_state_t *s,
                          uint8_t *g711_out,
                          int g711_max,
                          int nsymbols,
                          bool continue_from_current);
bool v91_rx_phil_codewords(v91_state_t *s,
                           const uint8_t *g711_in,
                           int g711_len,
                           bool continue_from_current);
int v91_j_descriptor_bit_len(const v91_dil_desc_t *desc);
int v91_tx_j_codewords(v91_state_t *s,
                       uint8_t *g711_out,
                       int g711_max,
                       const v91_dil_desc_t *desc);
bool v91_rx_j_codewords(v91_state_t *s,
                        const uint8_t *g711_in,
                        int g711_len,
                        v91_dil_desc_t *desc_out);
int v91_tx_scr_codewords(v91_state_t *s,
                         uint8_t *g711_out,
                         int g711_max,
                         int nsymbols);
bool v91_rx_scr_codewords(v91_state_t *s,
                          const uint8_t *g711_in,
                          int g711_len,
                          bool continue_from_current);
int v91_tx_es_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
bool v91_rx_es_codewords(v91_state_t *s,
                         const uint8_t *g711_in,
                         int g711_len,
                         bool continue_from_cp);
int v91_tx_cp_codewords(v91_state_t *s,
                        uint8_t *g711_out,
                        int g711_max,
                        const vpcm_cp_frame_t *cp,
                        bool continue_from_scr);
bool v91_rx_cp_codewords(v91_state_t *s,
                         const uint8_t *g711_in,
                         int g711_len,
                         vpcm_cp_frame_t *cp_out,
                         bool continue_from_scr);
int v91_tx_b1_codewords(v91_state_t *s,
                        uint8_t *g711_out,
                        int g711_max,
                        const vpcm_cp_frame_t *cp);
bool v91_rx_b1_codewords(v91_state_t *s,
                         const uint8_t *g711_in,
                         int g711_len,
                         const vpcm_cp_frame_t *cp);
int v91_tx_info_codewords(v91_state_t *s,
                          uint8_t *g711_out,
                          int g711_max,
                          const v91_info_frame_t *info);
bool v91_rx_info_codewords(v91_state_t *s,
                           const uint8_t *g711_in,
                           int g711_len,
                           v91_info_frame_t *info_out);
bool v91_info_frame_validate(const v91_info_frame_t *info, char *reason, size_t reason_len);
bool v91_info_build_diag(v91_state_t *s, const v91_info_frame_t *info, v91_info_diag_t *diag);
bool v91_info_decode_diag(v91_state_t *s,
                          const uint8_t *g711_in,
                          int g711_len,
                          v91_info_diag_t *diag);
void v91_default_dil_init(v91_dil_desc_t *desc);
int v91_dil_symbol_count(const v91_dil_desc_t *desc);
int v91_tx_dil_codewords(v91_state_t *s,
                         uint8_t *g711_out,
                         int g711_max,
                         const v91_dil_desc_t *desc);
int v91_tx_default_dil_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);

/*
 * Receive DIL codewords and compare them against the sequence the peer's
 * descriptor says it transmitted. Codewords that differ only in the LSB are
 * tolerated and counted per frame-interval slot: a slot accumulating
 * V91_ROBBED_BIT_DETECT_MIN_FLIPS or more LSB flips marks the path as
 * robbed-bit (rx_robbed_bit_detected / rx_robbed_slot_mask). Any other
 * corruption fails the reception. On success the descriptor is recorded and
 * analysed as with v91_note_received_dil().
 */
bool v91_rx_dil_codewords(v91_state_t *s,
                          const uint8_t *g711_in,
                          int g711_len,
                          const v91_dil_desc_t *desc);

/*
 * Streaming access to the differential-sign GPC descrambler used by the
 * SCR/CP/Es startup signals, for receivers that must locate the CP start
 * inside a variable-length SCR run (SCR descrambles to continuous ones;
 * the first zero bit is CP bit 17 of the frame-sync pattern).
 */
void v91_rx_diff_reset(v91_state_t *s);
int v91_rx_diff_scrambled_bit(v91_state_t *s, uint8_t codeword);
int v91_tx_startup_dil_sequence_codewords(v91_state_t *s,
                                          uint8_t *g711_out,
                                          int g711_max,
                                          const v91_dil_desc_t *peer_dil,
                                          v91_align_signal_t *align_out);
bool v91_analyse_dil_descriptor(const v91_dil_desc_t *desc, v91_dil_analysis_t *analysis_out);
bool v91_note_received_dil(v91_state_t *s,
                           const v91_dil_desc_t *desc,
                           v91_dil_analysis_t *analysis_out);
void v91_note_frame_sync_loss(v91_state_t *s);
bool v91_note_frame_sync_reacquired(v91_state_t *s);
void v91_request_retrain(v91_state_t *s);
void v91_note_retrain_complete(v91_state_t *s);
void v91_request_cleardown(v91_state_t *s);
void v91_note_recovery_timeout(v91_state_t *s);
bool v91_activate_data_mode(v91_state_t *s, const vpcm_cp_frame_t *cp);
void v91_deactivate_data_mode(v91_state_t *s);

/*
 * True when the CP frame's constellation masks can carry the K data bits
 * per frame implied by the given data-rate number.
 */
bool v91_cp_supports_drn(const vpcm_cp_frame_t *cp, uint8_t drn);

/*
 * Rate adaptation: select the data-rate number for a CP offer.
 * The template drn is the configured ceiling; it is capped by the
 * received-DIL analysis recommendation and, on robbed-bit paths, by the
 * robbed-bit safe ceiling, then stepped down until the template's
 * constellation masks can carry the implied K bits per frame.
 * A path counts as robbed-bit when the robbed_bit hint is set OR when
 * DIL reception detected LSB robbing (rx_robbed_bit_detected) — detection
 * is authoritative, so an undetected-by-config robbed trunk still gets
 * capped. Transparent-mode templates are returned unchanged only on paths
 * with no robbed-bit evidence; a transparent 64 kbps bearer cannot survive
 * LSB robbing, so detection forces normal mapped-rate selection instead.
 * Returns 0 when no rate is usable (cleardown).
 */
uint8_t v91_select_drn(const v91_state_t *s,
                       const vpcm_cp_frame_t *cp_template,
                       bool robbed_bit);

/*
 * Encode application octets to G.711 codewords.
 * In transparent mode this is a 1:1 mapping.
 * Returns the number of codewords produced.
 */
int v91_tx_codewords(v91_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len);

/*
 * Decode G.711 codewords to application octets.
 * In transparent mode this is a 1:1 mapping.
 * Returns the number of octets produced.
 */
int v91_rx_codewords(v91_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len);

/*
 * Encode application octets to linear PCM samples.
 * This is useful for local simulation, but it is not sufficient for a
 * bit-transparent V.91 bearer on its own because a later G.711 encoder may
 * canonicalise some codewords. Real V.91 carriage should stay at the
 * codeword layer end-to-end.
 */
int v91_tx_linear(v91_state_t *s,
                  int16_t *amp_out,
                  int amp_max,
                  const uint8_t *data_in,
                  int data_len);

/*
 * Decode linear PCM samples back to application octets by quantising them
 * to the selected G.711 law and then decoding those codewords.
 * This is a simulation helper, not a replacement for codeword-level
 * transport.
 */
int v91_rx_linear(v91_state_t *s,
                  uint8_t *data_out,
                  int data_max,
                  const int16_t *amp_in,
                  int amp_len);

void v91_tx_idle(v91_state_t *s, int16_t *amp_out, int amp_len);

#endif /* V91_H */
