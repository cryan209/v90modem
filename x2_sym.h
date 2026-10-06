#ifndef X2_SYM_H
#define X2_SYM_H
#include "x2.h"
/* x2 Draft 0.33 §§12–13: digital symmetric component, raw DS0 octets.
 * This is the startup source/acquisition and data transform, not a complete
 * INFO0/capability/confirmation dialogue. Host/client asymmetry is separate. */
typedef enum { X2_SYM_RUN, X2_SYM_ZEROS, X2_SYM_RAMP, X2_SYM_ACQUIRED } x2_sym_rx_stage_t;
typedef struct {
    unsigned answering, tx_enabled, tx_position;
    x2_sym_rx_stage_t rx_stage;
    unsigned run, zeros, ramp_position, phase, error_map, restarts;
} x2_sym_startup_t;
int x2_sym_startup_init(x2_sym_startup_t *s, unsigned answering);
/* Stops at the end of the source; the caller must supply the capability frame.
 * Originator emits FF while listening, then starts after acquiring peer ramp. */
size_t x2_sym_startup_tx(x2_sym_startup_t *s, uint8_t *octets, size_t count);
/* Returns number consumed, ending exactly after the last ramp octet. */
size_t x2_sym_startup_rx(x2_sym_startup_t *s, const uint8_t *octets, size_t count);
/* Draft 0.33 §12.2: two starts, five body octets, four CRC nibbles.
 * Reserved fields are retained; decoding commits only after CRC validation. */
#define X2_SYM_CAP_OCTETS 11
typedef struct { uint8_t words[5]; } x2_sym_cap_t;
int x2_sym_cap_build(x2_sym_cap_t *cap, unsigned error_map, unsigned mask, unsigned scramble);
int x2_sym_cap_encode(const x2_sym_cap_t *cap, uint8_t octets[X2_SYM_CAP_OCTETS]);
int x2_sym_cap_decode(const uint8_t octets[X2_SYM_CAP_OCTETS], x2_sym_cap_t *cap);
/* Returns 64000/56000, or zero for no supported common digital width.
 * Negotiated scrambling requires both peers' flag; outputs are transactional. */
unsigned x2_sym_cap_merge(const x2_sym_cap_t *local, const x2_sym_cap_t *peer,
                          unsigned *error_map, unsigned *mask, unsigned *scramble);
typedef struct { x2_scrambler_t tx, rx; unsigned width, scramble, reverse; } x2_sym_data_t;
int x2_sym_data_init(x2_sym_data_t *s, unsigned width, unsigned scramble, unsigned reverse);
uint8_t x2_sym_data_tx(x2_sym_data_t *s, uint8_t source);
uint8_t x2_sym_data_rx(x2_sym_data_t *s, uint8_t octet);
typedef void (*x2_put_bit_func_t)(void *, int);
typedef struct {
    x2_sym_startup_t startup;
    x2_sym_cap_t local, peer;
    x2_sym_data_t data;
    uint8_t frame[12], receive[11];
    unsigned frame_position, receive_count, rate, errors, mask, scramble;
    unsigned cap_valid, confirm_tx, confirm_rx, rx_data, tx_data, failed;
    x2_get_bit_func_t get_bit; x2_put_bit_func_t put_bit; void *context;
} x2_sym_link_t;
int x2_sym_link_init(x2_sym_link_t *s, unsigned answering,
                     x2_get_bit_func_t get_bit, x2_put_bit_func_t put_bit, void *context);
void x2_sym_link_rx(x2_sym_link_t *s, const uint8_t *octets, size_t count);
void x2_sym_link_tx(x2_sym_link_t *s, uint8_t *octets, size_t count);
#endif
