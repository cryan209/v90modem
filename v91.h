/*
 * v91.h - V.91 PCM modem helpers
 *
 * This module provides the G.711 bearer/data-plane seam for V.91.
 * It currently covers:
 *   - transparent-mode codeword transport
 *   - Phase 1/V.91 transition silence
 *   - Ez
 *   - INFO/INFO'
 *   - Eu/Em, PHIL, SCR, CP and frame-aligned default DIL generation
 *
 * That gives us a concrete startup/data boundary for the loopback harness
 * while later V.91 signals (J, Es, B1) are
 * still to be implemented.
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
#define V91_EZ_SYMBOLS 24
#define V91_INFO_SYMBOLS 62
#define V91_DEFAULT_DIL_SEGMENTS 125
#define V91_DEFAULT_DIL_SEGMENT_SYMBOLS 12
#define V91_DEFAULT_DIL_SYMBOLS (V91_DEFAULT_DIL_SEGMENTS * V91_DEFAULT_DIL_SEGMENT_SYMBOLS)

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
    v91_law_t  law;
    v91_mode_t mode;
    bool last_tx_info_valid;
    bool last_rx_info_valid;
    v91_info_frame_t last_tx_info;
    v91_info_frame_t last_rx_info;
    bool last_tx_dil_valid;
    v91_dil_desc_t last_tx_dil;
    bool last_tx_cp_valid;
    bool last_rx_cp_valid;
    vpcm_cp_frame_t last_tx_cp;
    vpcm_cp_frame_t last_rx_cp;
    uint32_t scramble_reg;
    uint32_t rx_scramble_reg;
    int diff_sign;
    int rx_prev_sign;
    bool frame_aligned;
    bool retrain_requested;
    bool circuit_107_on;
    int next_frame_interval;
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

int v91_tx_phase1_silence_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
int v91_tx_eu_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
int v91_tx_em_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
int v91_tx_ez_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
int v91_tx_phil_codewords(v91_state_t *s,
                          uint8_t *g711_out,
                          int g711_max,
                          int nsymbols,
                          bool continue_from_current);
int v91_tx_scr_codewords(v91_state_t *s,
                         uint8_t *g711_out,
                         int g711_max,
                         int nsymbols);
bool v91_rx_scr_codewords(v91_state_t *s,
                          const uint8_t *g711_in,
                          int g711_len,
                          bool continue_from_current);
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
int v91_tx_startup_dil_sequence_codewords(v91_state_t *s,
                                          uint8_t *g711_out,
                                          int g711_max,
                                          const v91_dil_desc_t *peer_dil,
                                          v91_align_signal_t *align_out);
void v91_note_frame_sync_loss(v91_state_t *s);

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
