/* V.92 8.5/8.6 and 9.5.2: analogue Phase-3 controller.
 * RX is calibrated, symbol-clock 8 kHz linear PCM from the network D/A.
 * TX is 16 kHz linear PCM so 9.5.2.1.7's 24.5T interval is representable.
 * This initial front end is an ideal sample/hold, not a bandlimited analogue
 * line model. Nonzero MD and nonzero Jp fractional corrections are rejected
 * explicitly until their waveform generators are available. No G.711
 * encoding occurs in the transmitter; the network codec belongs outside.
 */
#ifndef V92_ANALOGUE_PHASE3_H
#define V92_ANALOGUE_PHASE3_H
#include "v90.h"
#include "vpcm_cp.h"
#include "v92_analogue_phase4.h"

typedef enum {
    V92A_SILENCE, V92A_RU, V92A_RU_BAR, V92A_TRN1U, V92A_JA,
    V92A_WAIT_JD, V92A_SU, V92A_SU_BAR, V92A_WAIT_JP,
    V92A_SU_FINAL, V92A_DIL,
    V92A_CPT, V92A_E1U, V92A_PHASE4, V92A_FAILED
} v92a_stage_t;

typedef struct v92a_s v92a_t;
typedef struct {
    v90_law_t law;
    int u_info;
    int md_units;
    unsigned round_trip_symbols; /* measured delay at 8000 Hz; zero for local bearer */
    double lu;
    double digital_max_tx_dbm0;
    v90_dil_desc_t dil;
    uint32_t upstream_rate_mask; /* Table 20, 19 bits for 24000..48000 */
} v92a_config_t;

v92a_t *v92a_init(const v92a_config_t *cfg);
void v92a_free(v92a_t *s);
int v92a_tx(v92a_t *s, int16_t *samples, int count); /* 16000 Hz */
void v92a_rx(v92a_t *s, const int16_t *samples, int count); /* 8000 Hz */
v92a_stage_t v92a_stage(const v92a_t *s);
const char *v92a_failure(const v92a_t *s);
const vpcm_cp_frame_t *v92a_cpt(const v92a_t *s);
v92a4_t *v92a_phase4(v92a_t *s);
int v92a_final_e1u_sign(const v92a_t *s);

#endif
