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

bool v92_phase4_analyze(const v92_phase4_observation_t *obs,
                        v92_phase4_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* V92_PHASE4_DECODE_H */
