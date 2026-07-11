/* V.92 Phase 4 messages and procedure analysis. */

#ifndef V92_PHASE4_DECODE_H
#define V92_PHASE4_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define V92_SUVD_BITS 54
#define V92_CPD_BASE_BITS 72

typedef struct {
    bool silent_period_requested;
    bool acknowledge;
} v92_suvd_frame_t;

typedef struct {
    bool valid;
    bool binary_bits_ok;
    bool frame_sync_ok;
    bool identifier_ok;
    bool reserved_ok;
    bool start_bits_ok;
    bool fill_bits_ok;
    bool crc_ok;
    uint16_t crc_field;
    uint16_t crc_expected;
} v92_suvd_diag_t;

/* Table 30 mandatory CPd part with all optional parts absent. */
typedef struct {
    uint8_t selected_upstream_drn;
    uint8_t trellis_select;
    bool extend_e2u;
    bool acknowledge;
    uint16_t gain_q0_16;
} v92_cpd_base_frame_t;

typedef struct {
    bool valid;
    bool binary_bits_ok;
    bool frame_sync_ok;
    bool identifier_ok;
    bool optional_parts_absent;
    bool reserved_ok;
    bool start_bits_ok;
    bool parameters_ok;
    bool fill_bits_ok;
    bool crc_ok;
    uint16_t crc_field;
    uint16_t crc_expected;
} v92_cpd_base_diag_t;

typedef struct {
    bool phase4_seen;
    bool training_failed;
    int phase4_sample;
    bool suvd_seen;
    bool suvd_valid;
    bool suvd_acknowledge_seen;
    bool cpd_seen;
    bool cpd_valid;
    bool ed_seen;
    bool b1d_seen;
    bool data_seen;
} v92_phase4_observation_t;

typedef struct {
    bool valid;
    bool started;
    bool complete;
    int phase4_sample;
    bool suvd_seen;
    bool suvd_valid;
    bool suvd_acknowledge_seen;
    bool cpd_seen;
    bool cpd_valid;
    bool ed_seen;
    bool b1d_seen;
    bool data_seen;
    const char *status;
} v92_phase4_result_t;

/* Table 31/V.92. Bits are stored one per byte, in transmission order. */
bool v92_suvd_encode(const v92_suvd_frame_t *frame,
                     uint8_t *bits,
                     int bits_max);
bool v92_suvd_decode(const uint8_t *bits,
                     int bit_count,
                     v92_suvd_frame_t *frame,
                     v92_suvd_diag_t *diag);

bool v92_cpd_base_encode(const v92_cpd_base_frame_t *frame,
                         uint8_t *bits,
                         int bits_max);
bool v92_cpd_base_decode(const uint8_t *bits,
                         int bit_count,
                         v92_cpd_base_frame_t *frame,
                         v92_cpd_base_diag_t *diag);

/* ---- Full Table 30 CPd with optional parts ----
 *
 * The three optional parts (modulus encoder parameters, prefilter and
 * precoder coefficients, constellation sets) are flagged by bits 19:21 and
 * removed entirely when absent.  alpha = 17 x (LZ1 + LP1 + LZ2 + LP2),
 * beta = 17 x (LC1 + ... + LC6).  Frames end with a start bit, 16 CRC bits,
 * and at least one fill bit, extended to `alignment` bits — the number of
 * bits carried per 6-symbol data frame of the transmitting modulation
 * (d = drn + 8 for TRN2d-mapped training, 6 for the sign-modulated
 * compatibility path). */

#define V92_CPD_MAX_TAPS 64
#define V92_CPD_MAX_SETS 6
#define V92_CPD_MAX_POINTS 128
#define V92_CPD_MAX_BITS 18432

/* SUVd mapped over TRN2d: align(52, d) with d <= 30. */
#define V92_SUVD_MAX_BITS 128

typedef struct {
    bool modulus_present;           /* bit 19 */
    bool coeffs_present;            /* bit 20 */
    bool constellations_present;    /* bit 21 */
    uint8_t selected_upstream_drn;  /* bits 22:26, 0..19 */
    uint8_t trellis_select;         /* bits 27:28 */
    bool extend_e2u;                /* bit 29 */
    bool acknowledge;               /* bit 33 — CPd' when set */
    uint16_t gain_q0_16;            /* bits 35:50, 4 x G, unsigned Q0.16 */
    /* Modulus encoder parameters (12 upstream data frame intervals). */
    uint8_t moduli[12];
    /* Precoder and prefilter coefficients. */
    uint16_t lz1;                   /* precoder feed-forward taps */
    uint16_t lp1;                   /* precoder feedback taps */
    uint16_t lz2;                   /* prefilter feed-forward taps */
    uint16_t lp2;                   /* prefilter feedback taps */
    int16_t precoder_ff[V92_CPD_MAX_TAPS];   /* z1, signed Q0.15 */
    int16_t precoder_fb[V92_CPD_MAX_TAPS];   /* p1, signed Q1.14 */
    int16_t prefilter_ff[V92_CPD_MAX_TAPS];  /* z2, signed Q0.15 */
    int16_t prefilter_fb[V92_CPD_MAX_TAPS];  /* p2, signed Q1.14 */
    /* Constellation sets: index per data frame interval pair (i, i + 6),
     * positive-point counts, and linear point values smallest first. */
    uint8_t dfi[6];
    uint8_t set_sizes[V92_CPD_MAX_SETS];
    uint16_t points[V92_CPD_MAX_SETS][V92_CPD_MAX_POINTS];
} v92_cpd_frame_t;

typedef struct {
    v92_cpd_frame_t frame;
    int nbits;
    uint16_t crc_field;
    uint16_t crc_expected;
    bool binary_bits_ok;
    bool frame_sync_ok;
    bool identifier_ok;
    bool start_bits_ok;
    bool reserved_ok;
    bool parameters_ok;
    bool fill_bits_ok;
    bool crc_ok;
    bool valid;
} v92_cpd_diag_t;

int v92_cpd_bit_length(const v92_cpd_frame_t *frame, int alignment);
bool v92_cpd_encode(const v92_cpd_frame_t *frame,
                    int alignment,
                    uint8_t *bits,
                    int bits_max,
                    int *nbits_out);
bool v92_cpd_decode(const uint8_t *bits,
                    int bit_count,
                    v92_cpd_frame_t *frame,
                    v92_cpd_diag_t *diag);

/* SUVd filled to an arbitrary per-frame alignment (Table 31 fill rule). */
int v92_suvd_bit_length(int alignment);
bool v92_suvd_encode_aligned(const v92_suvd_frame_t *frame,
                             int alignment,
                             uint8_t *bits,
                             int bits_max,
                             int *nbits_out);
/* Variable-length SUVd decode (52..V92_SUVD_MAX_BITS bits). */
bool v92_suvd_decode_bits(const uint8_t *bits,
                          int bit_count,
                          v92_suvd_frame_t *frame,
                          v92_suvd_diag_t *diag);

bool v92_phase4_analyze(const v92_phase4_observation_t *obs,
                        v92_phase4_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* V92_PHASE4_DECODE_H */
