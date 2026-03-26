/*
 * v90.h — V.90 digital modem module
 *
 * Implements the digital (server) side of ITU-T V.90:
 *   - Downstream: PCM codeword injection into G.711 stream (up to 56 kbps)
 *   - Upstream:   V.34 demodulation (handled by SpanDSP V.34 RX)
 *   - Training:   V.34 Phases 2-4 with V.90-specific INFO0d/INFO0a frames
 *
 * This module wraps SpanDSP's V.34 state machine, enabling V.90 mode
 * for Phase 2 INFO exchange and managing the PCM downstream encoder.
 */

#ifndef V90_H
#define V90_H

#include <spandsp.h>
#include <stdint.h>
#include <stdbool.h>

/* V.90 downstream encoder state */
typedef struct v90_state_s v90_state_t;

/* PCM law selection */
typedef enum {
    V90_LAW_ULAW = 0,
    V90_LAW_ALAW = 1
} v90_law_t;

/*
 * Initialise a V.90 digital modem context.
 * Creates an underlying V.34 modem with V.90 INFO0d enabled.
 *
 * Parameters:
 *   baud_rate      - V.34 upstream baud rate (3200 recommended for V.90)
 *   bit_rate       - V.34 upstream bit rate (max for baud rate)
 *   calling_party  - true if we initiated the call
 *   law            - G.711 PCM law in use on the SIP path
 *   get_bit        - callback for V.34 upstream TX data bits
 *   get_bit_user_data - opaque pointer for get_bit
 *   put_bit        - callback for V.34 upstream RX data bits
 *   put_bit_user_data - opaque pointer for put_bit
 *
 * Returns: pointer to V.90 context, or NULL on failure.
 */
v90_state_t *v90_init(int baud_rate,
                      int bit_rate,
                      bool calling_party,
                      v90_law_t law,
                      span_get_bit_func_t get_bit,
                      void *get_bit_user_data,
                      span_put_bit_func_t put_bit,
                      void *put_bit_user_data);

/* Free a V.90 context and its underlying V.34 state. */
void v90_free(v90_state_t *s);

/*
 * Get the underlying V.34 state for passing to v34_tx()/v34_rx()
 * during training.  After training completes, TX switches to PCM.
 */
v34_state_t *v90_get_v34(v90_state_t *s);

/*
 * Encode downstream data into PCM codewords for the G.711 RTP stream.
 * Call this instead of v34_tx() once training completes.
 *
 * Reads from data_in (user data bytes) and writes linear PCM samples
 * to amp[].  Each 6-symbol frame consumes 6 data bytes and produces
 * 6 PCM samples.
 *
 * Parameters:
 *   s        - V.90 context
 *   amp      - output buffer for linear PCM samples
 *   len      - number of samples to generate
 *   data_in  - input data bytes (at least len bytes available)
 *   data_len - number of bytes available in data_in
 *
 * Returns: number of data bytes consumed.
 */
int v90_tx_data(v90_state_t *s, int16_t amp[], int len,
                const uint8_t *data_in, int data_len);

/*
 * Generate idle (silence) PCM samples when no data is available.
 */
void v90_tx_idle(v90_state_t *s, int16_t amp[], int len);

/* Get the logging context for diagnostics. */
logging_state_t *v90_get_logging_state(v90_state_t *s);

#endif /* V90_H */
