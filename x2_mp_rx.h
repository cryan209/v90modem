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
/* Timing hypotheses 0..9 sample the low-pass output directly, a quarter
 * sample apart; X2_MP_RX_EQ onward are the equalized ones, a fifth of a
 * symbol apart. */
#define X2_MP_RX_EQS 5
#define X2_MP_RX_EQ 10
#define X2_MP_RX_HYPOTHESES (X2_MP_RX_EQ+X2_MP_RX_EQS)
#define X2_MP_RX_EQ_TAPS 32
typedef struct {
    double taps[127], ring_re[127], ring_im[127];
    unsigned position;
    uint64_t samples;
    double previous_re, previous_im, power[X2_MP_RX_HYPOTHESES];
    double fourth_re[X2_MP_RX_HYPOTHESES], fourth_im[X2_MP_RX_HYPOTHESES];
    double next_symbol[10];
    /* Carrier phase per timing hypothesis, unwrapped modulo 90 degrees. */
    double phase[X2_MP_RX_HYPOTHESES];
    unsigned phase_valid[X2_MP_RX_HYPOTHESES];
    x2_mp_rx_hypothesis_t hypotheses[X2_MP_RX_HYPOTHESES][2];
    /* Blind fractionally spaced equalizers (taps one 8 kHz sample = T/2.5
     * apart) on the 16 kHz baseband, for a real line: the hypotheses above
     * assume a flat channel and the far end's baud clock. Constant-modulus
     * adaptation, then radius-directed on the 16-point TRN/MP orbits, only
     * while the upstream carrier is present. Blind adaptation from a centre
     * spike stalls when the spike sits near an eye crossing, and where the
     * eye opens against the baud-rate line depends on the line's band-edge
     * delay (half a symbol from the envelope peak for the emulated Courier,
     * a quarter on a Cisco FXS port), so five equalizers run a fifth of a
     * symbol apart and CRC picks. */
    double half_taps[127];
    double eq_re[X2_MP_RX_EQS][X2_MP_RX_EQ_TAPS], eq_im[X2_MP_RX_EQS][X2_MP_RX_EQ_TAPS];
    double z_re[128], z_im[128];
    uint64_t z_count;
    /* Symbol strobe on the 16 kHz baseband (in samples, fractional) and its
     * correction to the nominal five samples a symbol: a second-order loop
     * on the baud-rate line of |z|^2, so the peer's own clock (a Courier on
     * a Cisco FXS port ran 80 ppm slow of the network, 14 November 2026)
     * does not walk the eye across the equalizer. It coasts on the last
     * rate through the silence between the Courier's two bursts. */
    double eq_pos, eq_rate, line_re, line_im;
    double eq_fast, eq_peak;
    unsigned eq_started, eq_burst_symbols, eq_gap, eq_valid_frames;
    /* Set while only the equalizer trains (before the server's TRAIN_C):
     * no MP or E is framed from the Courier's first upstream burst. */
    unsigned train_only;
    uint64_t warm_until;
    x2_mp_received_t received;
    void *context;
    unsigned valid_frames, rejected_frames;
    /* Courier AF2E/AF38 and Ie030002 AACB: twenty decoded ones after
     * qualified MP, on the same timing/phase hypothesis. */
    unsigned e_detected, e_timing, e_phase;
    /* In-data rate change (Courier, V.90 9.6 Jd bit 48 = 0): MP, MP' and E
     * on the 4-point constellation, two differential bits per symbol. */
    unsigned four_point;
    uint64_t e_sample;
} x2_mp_rx_t;
void x2_mp_rx_init(x2_mp_rx_t *rx, x2_mp_received_t received, void *context);
void x2_mp_rx_audio(x2_mp_rx_t *rx, const int16_t *samples, size_t count);
/* A fresh MP/E framer on the same line: every hypothesis, E and the
 * 4-point mode are reset; the front end and the equalizer keep running. */
void x2_mp_rx_restart(x2_mp_rx_t *rx);
/* Already descrambled bits: the same strict streaming framer used by the
 * audio receiver. Notifications require repeated matching protected words. */
void x2_mp_rx_bit(x2_mp_rx_t *rx, unsigned timing, unsigned phase, unsigned bit);
/* Ie030002 CFF4/D24A, Draft 0.33 clauses 15/16/21/22. Supported short-record
 * mode zero, zero position controls, low-level PCMU banks. No variable XMF
 * bitmap record is guessed from a short MP. Returns selected local index. */
int x2_short_record_config(const x2_mp_t *mp, unsigned local_mask,
                           x2_pcm_config_t *config);
/* One short-record allocation (index 1..15) at a given MD (0..6). */
int x2_short_profile(unsigned index, unsigned md, x2_pcm_config_t *config);
/* Draft 0.33 clauses 15/16/22: a frame carries B+MD source bits per six
 * samples, so short-record index 1..15 runs at (B+MD)*8000/6 bit/s at its
 * allocation's MD (5 in every captured Courier MP): index 1 = 32000,
 * index 14 = 54666. Not the supervisor display table (x2_nominal_rate).
 * Returns 0 outside 1..15. */
unsigned x2_short_record_rate(unsigned index);
/* The local PCM index mask (bit 0 = index 1) whose rates lie within
 * [min_bps, max_bps]; a bound <= 0 is open. */
uint16_t x2_short_record_mask(int min_bps, int max_bps);
/* Ie030002 D229/D238, CB3D: the server's E training banks (B=19, MD=5,
 * the nine-entry A5..E5 alphabet in every position), which also carry its
 * startup record. */
void x2_training_config(x2_pcm_config_t *config);
#endif
