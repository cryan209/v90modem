/* x2 Draft 0.33, six-symbol PCM payload and Courier four-word MP.
 * Independent implementation of recovered behavior; no vendor source.
 */
#ifndef X2_H
#define X2_H
#include <stdint.h>
#include <stddef.h>
/* Draft 0.33 §12.2, INFO0 body bits 0..16 (ITU bits 12..28).
 * Bearer role is independent of SIP calling/answering role. */
typedef enum { X2_ROLE_NONE, X2_ROLE_HOST, X2_ROLE_CLIENT, X2_ROLE_SYMMETRIC } x2_role_t;
x2_role_t x2_info_role_select(uint32_t local_body, uint32_t peer_body);
#define X2_POSITIONS 6
#define X2_BANK_MAX 128
#define X2_MP_BITS 104

typedef struct {
    uint8_t amplitude_bits; /* B: 19..37 in recovered allocation tables */
    uint8_t independent_signs; /* MD: 0..6 */
    uint8_t format_xor; /* recovered output format: 0 or 0x2a */
    uint16_t sizes[X2_POSITIONS];
    /* Emission order. Low octet = reference code; bits 8..14 = level rank. */
    uint16_t banks[X2_POSITIONS][X2_BANK_MAX];
    int16_t levels[X2_BANK_MAX];
} x2_pcm_config_t;

typedef struct { uint8_t parity; } x2_pcm_state_t;
typedef struct { uint32_t history; uint8_t tap; } x2_scrambler_t;
typedef struct { uint16_t words[4]; uint8_t tail; } x2_mp_t;

/* Returns 0 on success, -1 for invalid configuration/input. On failure,
 * output and parity are unchanged. Configuration must remain valid while used.
 */
int x2_pcm_validate(const x2_pcm_config_t *config);
unsigned x2_pcm_frame_bits(const x2_pcm_config_t *config);
/* Input is already scrambled, LSB-first: MD signs followed by B amplitudes.
 * delayed_monitor is the signed recovered 04d2 value at this frame boundary.
 * The audio dispatcher, not this frame mapper, maintains that monitor.
 */
int x2_pcm_encode(const x2_pcm_config_t *config, x2_pcm_state_t *state,
                  uint64_t bits, int16_t delayed_monitor, uint8_t octets[6]);
/* Exact-codeword inverse; requires negotiated banks and six-slot alignment.
 * This is not an analogue equalizer. Output bits remain scrambled.
 */
int x2_pcm_decode(const x2_pcm_config_t *config, x2_pcm_state_t *state,
                  const uint8_t octets[6], uint64_t *bits);
/* tap=18 (GPC) or 5 (GPA). Zero history is the explicit call reset. */
int x2_scrambler_init(x2_scrambler_t *state, unsigned tap, uint32_t history);
unsigned x2_scramble_bit(x2_scrambler_t *state, unsigned bit);
unsigned x2_descramble_bit(x2_scrambler_t *state, unsigned bit);
/* Steady payload source, compatible with an int (*get_bit)(void *) callback.
 * Returns 0/1, or negative to pause. Partial frames survive a pause.
 * Config is copied at init; RTP writes may end anywhere within six samples.
 * This implements the recovered monitor [0,1000,0,0] branch only.
 */
typedef int (*x2_get_bit_func_t)(void *user_data);
typedef struct {
    x2_pcm_config_t config;
    x2_pcm_state_t mapper;
    x2_scrambler_t scrambler;
    x2_get_bit_func_t get_bit;
    void *user_data;
    uint64_t pending;
    unsigned pending_count, output_position;
    uint8_t output[6];
    int16_t disparity_history[6];
} x2_pcm_tx_t;
int x2_pcm_tx_init(x2_pcm_tx_t *tx, const x2_pcm_config_t *config,
                   unsigned scrambler_tap, x2_get_bit_func_t get_bit, void *user_data);
size_t x2_pcm_tx_g711(x2_pcm_tx_t *tx, uint8_t *octets, size_t count);
/* Unpacked 0/1 bits, before line scrambling/modulation. Tail is preserved;
 * its semantics are intentionally not inferred from the CRC body.
 */
uint16_t x2_mp_crc(const uint16_t words[4]);
int x2_mp_encode(const x2_mp_t *mp, uint8_t bits[X2_MP_BITS]);
int x2_mp_decode(const uint8_t bits[X2_MP_BITS], x2_mp_t *mp);
/* Supervisor display table, not a throughput measurement. */
unsigned x2_nominal_rate(unsigned index);
#endif
