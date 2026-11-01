/* MNP class 5 stream compression: adaptive frequency encoding (AFE) and
 * run-length encoding. This proprietary extension is NOT V.42 Annex A.
 * Wire behavior is checked against the independent Conexant HSF codec;
 * see docs/mnp.md for the reference, vectors and negotiation details.
 * Feed only accepted, in-sequence LT information bytes (V.42 A.7.3.2).
 * Flush terminates an octet-aligned segment, NOT the adaptive dictionary.
 */
#ifndef MNP5_H
#define MNP5_H
#include <stddef.h>
#include <stdint.h>

typedef struct mnp5_s mnp5_t;
typedef void (*mnp5_output_fn)(void *user, const uint8_t *data, int len);
/* Each instance is one direction. Caller supplies independent TX/RX states. */
mnp5_t *mnp5_init(mnp5_output_fn output, void *user);
void mnp5_free(mnp5_t *s);
void mnp5_restart(mnp5_t *s);
int mnp5_encode(mnp5_t *s, const uint8_t *data, size_t len);
int mnp5_flush(mnp5_t *s);
/* Invalid run counts latch failure until restart. There is no payload CRC
 * after decompression: link integrity alone cannot detect dictionary drift. */
int mnp5_decode(mnp5_t *s, const uint8_t *data, size_t len);
#endif
