/* V.92 9.6.2: analogue final training. Calibrated 8 kHz linear PCM.
 * The network codec is outside this endpoint. Supports the implemented
 * 16-state upstream encoder; unsupported CPd profiles fail explicitly. */
#ifndef V92_ANALOGUE_PHASE4_H
#define V92_ANALOGUE_PHASE4_H
#include "v90_analogue_phase4.h"
#include "v92_phase4_decode.h"
typedef struct v92a4_s v92a4_t;
typedef enum { V92A4_WAIT, V92A4_TRN, V92A4_SUV, V92A4_CP,
               V92A4_E, V92A4_B1, V92A4_DATA, V92A4_FAILED } v92a4_stage_t;
v92a4_t *v92a4_init(const v90_analogue_phase4_config_t *cfg,
                     int points, double lu, uint32_t rate_mask, unsigned round_trip_symbols);
void v92a4_free(v92a4_t *s);
void v92a4_start(v92a4_t *s, int preceding_e1u_sign);
int v92a4_tx(v92a4_t *s, int16_t *samples, int count);
void v92a4_rx(v92a4_t *s, const int16_t *samples, int count);
v92a4_stage_t v92a4_stage(const v92a4_t *s);
bool v92a4_downstream_ready(const v92a4_t *s);
const v92_cpd_frame_t *v92a4_cpd(const v92a4_t *s);
/* Payload source is read only after all 48 B1u frames. NULL sends marks. */
void v92a4_set_data_source(v92a4_t *s, int (*get_bit)(void *), void *user);
int v92a4_get_data_bits(v92a4_t *s, uint8_t *bits, int capacity);
int v92a4_downstream_rate(const v92a4_t *s);
const char *v92a4_failure(const v92a4_t *s);
double v92a4_rx_decision(const v92a4_t *s);
double v92a4_rx_tolerance(const v92a4_t *s);
#endif
