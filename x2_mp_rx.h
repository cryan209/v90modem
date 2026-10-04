#ifndef X2_MP_RX_H
#define X2_MP_RX_H
#include "x2.h"
typedef void (*x2_mp_received_t)(void *context, const x2_mp_t *mp);
typedef struct {
    x2_scrambler_t descrambler;
    uint8_t frame[104];
    unsigned ones, position, previous_rotation, repeats;
    x2_mp_t previous_mp;
} x2_mp_rx_hypothesis_t;
typedef struct {
    double taps[127], ring_re[127], ring_im[127];
    unsigned position;
    uint64_t samples;
    double previous_re, previous_im, power[10], fourth_re[10], fourth_im[10];
    double next_symbol[10];
    x2_mp_rx_hypothesis_t hypotheses[10][2];
    x2_mp_received_t received;
    void *context;
    unsigned valid_frames, rejected_frames;
    /* Courier AF2E/AF38 and Ie030002 AACB: twenty decoded ones after
     * qualified MP, on the same timing/phase hypothesis. */
    unsigned e_detected, e_timing, e_phase;
    uint64_t e_sample;
} x2_mp_rx_t;
void x2_mp_rx_init(x2_mp_rx_t *rx, x2_mp_received_t received, void *context);
void x2_mp_rx_audio(x2_mp_rx_t *rx, const int16_t *samples, size_t count);
/* Already descrambled bits: the same strict streaming framer used by the
 * audio receiver. Notifications require repeated matching protected words. */
void x2_mp_rx_bit(x2_mp_rx_t *rx, unsigned timing, unsigned phase, unsigned bit);
/* Ie030002 CFF4/D24A, Draft 0.33 clauses 15/16/21/22. Supported short-record
 * mode zero, zero position controls, low-level PCMU banks. No variable XMF
 * bitmap record is guessed from a short MP. Returns selected local index. */
int x2_short_record_config(const x2_mp_t *mp, unsigned local_mask,
                           x2_pcm_config_t *config);
#endif
