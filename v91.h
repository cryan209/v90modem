/*
 * v91.h - V.91 PCM modem helpers
 *
 * This module provides the G.711 bearer/data-plane seam for V.91.
 * The first implementation targets V.91 transparent mode, where data
 * octets are carried directly as G.711 PCM codewords. This gives us a
 * concrete codec boundary and a local loopback target while Phase 1/2
 * V.8/V.91 startup remains to be implemented.
 */

#ifndef V91_H
#define V91_H

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

void v91_init(v91_state_t *s, v91_law_t law, v91_mode_t mode);

uint8_t v91_idle_codeword(v91_law_t law);
int16_t v91_codeword_to_linear(v91_law_t law, uint8_t codeword);
uint8_t v91_linear_to_codeword(v91_law_t law, int16_t sample);

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
