#ifndef VPCM_V90_SESSION_H
#define VPCM_V90_SESSION_H

#include "v90.h"
#include "v91.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    VPCM_V90_MODEM_IDLE = 0,
    VPCM_V90_MODEM_PHASE1,
    VPCM_V90_MODEM_INFO,
    VPCM_V90_MODEM_DIL,
    VPCM_V90_MODEM_SCR,
    VPCM_V90_MODEM_DOWNSTREAM_CP,
    VPCM_V90_MODEM_DOWNSTREAM_B1,
    VPCM_V90_MODEM_UPSTREAM_CP,
    VPCM_V90_MODEM_UPSTREAM_B1,
    VPCM_V90_MODEM_DATA,
    VPCM_V90_MODEM_CLEARDOWN
} vpcm_v90_modem_state_t;

typedef struct {
    v91_law_t law;
    vpcm_v90_modem_state_t state;
    bool data_mode_active;
} vpcm_v90_session_t;

typedef struct {
    v91_law_t law;
    bool echo_limited;
    uint32_t seed_base;
    int data_seconds;
} vpcm_v90_startup_contract_params_t;

typedef struct {
    bool (*record_simplex_g711)(void *user_data,
                                v91_law_t law,
                                bool digital_to_analogue,
                                const uint8_t *tx_codewords,
                                int codeword_len);
    bool (*record_duplex_g711)(void *user_data,
                               const uint8_t *digital_tx_codewords,
                               const uint8_t *analogue_tx_codewords,
                               int codeword_len);
    void *user_data;
} vpcm_v90_startup_contract_io_t;

#define VPCM_V90_SAMPLE_PCM_LEN 6
#define VPCM_V90_SAMPLE_DATA_MAX 16

typedef struct {
    bool phase2_contract_valid;
    bool phase2_completed;
    bool phase2_phase3_seen;
    int phase2_u_info;
    bool phase2_received_info0a_valid;
    bool phase2_received_info1a_valid;
    v90_info0a_t phase2_received_info0a;
    v90_info1a_t phase2_received_info1a;
    v90_info0a_t analogue_info0a;
    v90_info1a_t analogue_info1a;
    uint8_t analogue_info0a_bits[(V90_INFO0A_BITS + 7) / 8];
    uint8_t analogue_info1a_bits[(V90_INFO1A_BITS + 7) / 8];
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    bool digital_dil_analysis_valid;
    v91_dil_analysis_t digital_dil_analysis;
    vpcm_cp_frame_t cp_down_offer;
    vpcm_cp_frame_t cp_down_ack;
    vpcm_cp_frame_t cp_up_offer;
    vpcm_cp_frame_t cp_up_ack;
    int data_seconds;
    int total_codewords;
    int down_total_bytes;
    int up_total_bytes;
    int sample_down_data_len;
    int sample_up_data_len;
    uint8_t down_data_in_sample[VPCM_V90_SAMPLE_DATA_MAX];
    uint8_t down_pcm_tx_sample[VPCM_V90_SAMPLE_PCM_LEN];
    uint8_t down_pcm_rx_sample[VPCM_V90_SAMPLE_PCM_LEN];
    uint8_t down_data_out_sample[VPCM_V90_SAMPLE_DATA_MAX];
    uint8_t up_data_in_sample[VPCM_V90_SAMPLE_DATA_MAX];
    uint8_t up_pcm_tx_sample[VPCM_V90_SAMPLE_PCM_LEN];
    uint8_t up_pcm_rx_sample[VPCM_V90_SAMPLE_PCM_LEN];
    uint8_t up_data_out_sample[VPCM_V90_SAMPLE_DATA_MAX];
} vpcm_v90_startup_contract_report_t;

void vpcm_v90_session_init(vpcm_v90_session_t *session, v91_law_t law);
void vpcm_v90_session_reset(vpcm_v90_session_t *session);
void vpcm_v90_session_set_state(vpcm_v90_session_t *session, vpcm_v90_modem_state_t state);
void vpcm_v92_init_digital_dil_from_ja(v91_dil_desc_t *desc, bool echo_limited);
void vpcm_v92_select_profile_from_dil(const v91_dil_analysis_t *analysis,
                                      uint8_t *downstream_drn,
                                      uint8_t *upstream_drn);
bool vpcm_v90_session_run_startup_contract(vpcm_v90_session_t *session,
                                           const vpcm_v90_startup_contract_params_t *params,
                                           const vpcm_v90_startup_contract_io_t *io,
                                           vpcm_v90_startup_contract_report_t *report);
const char *vpcm_v90_modem_state_to_str(vpcm_v90_modem_state_t state);

#endif
