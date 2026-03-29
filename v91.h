/*
 * v91.h - V.91 PCM modem helpers
 *
 * This module provides the G.711 bearer/data-plane seam for V.91.
 * It currently covers:
 *   - transparent-mode codeword transport
 *   - Phase 1/V.91 transition silence
 *   - Ez
 *   - INFO/INFO'
 *
 * That gives us a concrete startup/data boundary for the loopback harness
 * while later V.91 signals (J, PHIL, Eu, Em, DIL, SCR, CP, Es, B1) are
 * still to be implemented.
 */

#ifndef V91_H
#define V91_H

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

typedef struct {
    v91_law_t  law;
    v91_mode_t mode;
} v91_state_t;

#define V91_PHASE1_SILENCE_SYMBOLS 600
#define V91_EZ_SYMBOLS 24
#define V91_INFO_SYMBOLS 62

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

void v91_init(v91_state_t *s, v91_law_t law, v91_mode_t mode);

uint8_t v91_idle_codeword(v91_law_t law);
int16_t v91_codeword_to_linear(v91_law_t law, uint8_t codeword);
uint8_t v91_linear_to_codeword(v91_law_t law, int16_t sample);
uint8_t v91_ucode_to_codeword(v91_law_t law, int ucode, bool positive);

int v91_tx_phase1_silence_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
int v91_tx_ez_codewords(v91_state_t *s, uint8_t *g711_out, int g711_max);
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
