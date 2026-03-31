#ifndef VPCM_V91_SESSION_H
#define VPCM_V91_SESSION_H

#include "v91.h"

#include <stdbool.h>

typedef enum {
    VPCM_V91_MODEM_IDLE = 0,
    VPCM_V91_MODEM_V8,
    VPCM_V91_MODEM_PHASE1,
    VPCM_V91_MODEM_INFO,
    VPCM_V91_MODEM_DIL,
    VPCM_V91_MODEM_SCR,
    VPCM_V91_MODEM_CP,
    VPCM_V91_MODEM_B1,
    VPCM_V91_MODEM_DATA,
    VPCM_V91_MODEM_CLEARDOWN
} vpcm_v91_modem_state_t;

typedef struct {
    v91_law_t law;
    vpcm_v91_modem_state_t state;
    bool data_mode_active;
} vpcm_v91_session_t;

typedef struct {
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    vpcm_cp_frame_t cp_offer;
    vpcm_cp_frame_t cp_ack;
} vpcm_v91_startup_cfg_t;

typedef struct {
    v91_info_frame_t caller_info;
    v91_info_frame_t answerer_info;
    vpcm_cp_frame_t cp_ack;
    int caller_startup_len;
    int answerer_startup_len;
} vpcm_v91_startup_report_t;

void vpcm_v91_session_init(vpcm_v91_session_t *session, v91_law_t law);
void vpcm_v91_session_reset(vpcm_v91_session_t *session);
void vpcm_v91_session_set_state(vpcm_v91_session_t *session, vpcm_v91_modem_state_t state);
void vpcm_v91_startup_cfg_init(vpcm_v91_startup_cfg_t *cfg,
                               v91_law_t law,
                               const vpcm_cp_frame_t *cp_offer_template);
void vpcm_v91_session_enter_phase1(vpcm_v91_session_t *session);
void vpcm_v91_session_enter_info(vpcm_v91_session_t *session);
void vpcm_v91_session_enter_dil(vpcm_v91_session_t *session);
void vpcm_v91_session_enter_scr(vpcm_v91_session_t *session);
void vpcm_v91_session_enter_cp(vpcm_v91_session_t *session);
void vpcm_v91_session_enter_b1(vpcm_v91_session_t *session);
void vpcm_v91_session_enter_data(vpcm_v91_session_t *session);
void vpcm_v91_session_enter_cleardown(vpcm_v91_session_t *session);
bool vpcm_v91_session_run_phase1(vpcm_v91_session_t *session,
                                 v91_state_t *caller,
                                 v91_state_t *answerer,
                                 uint8_t *startup_buf,
                                 int startup_cap,
                                 uint8_t *transport_buf,
                                 int transport_cap);
bool vpcm_v91_session_run_info(vpcm_v91_session_t *session,
                               v91_state_t *caller,
                               v91_state_t *answerer,
                               const vpcm_v91_startup_cfg_t *cfg,
                               v91_info_frame_t *rx_info,
                               uint8_t *transport_buf,
                               int transport_cap,
                               bool robbed_bit);
bool vpcm_v91_session_run_dil(vpcm_v91_session_t *session,
                              v91_state_t *caller,
                              v91_state_t *answerer,
                              const v91_dil_desc_t *default_dil,
                              uint8_t *startup_buf,
                              int startup_cap,
                              uint8_t *transport_buf,
                              int transport_cap,
                              int *caller_startup_len,
                              int *answerer_startup_len,
                              bool robbed_bit);
bool vpcm_v91_session_run_scr(vpcm_v91_session_t *session,
                              v91_state_t *caller,
                              v91_state_t *answerer,
                              uint8_t *scr_buf,
                              int scr_cap,
                              uint8_t *transport_buf,
                              int transport_cap,
                              bool robbed_bit);
bool vpcm_v91_session_run_cp(vpcm_v91_session_t *session,
                             v91_state_t *caller,
                             v91_state_t *answerer,
                             const vpcm_v91_startup_cfg_t *cfg,
                             vpcm_cp_frame_t *cp_rx,
                             uint8_t *cp_buf,
                             int cp_cap,
                             uint8_t *transport_buf,
                             int transport_cap,
                             bool robbed_bit);
bool vpcm_v91_session_run_b1(vpcm_v91_session_t *session,
                             v91_state_t *caller,
                             v91_state_t *answerer,
                             const vpcm_cp_frame_t *cp_ack,
                             uint8_t *es_buf,
                             int es_cap,
                             uint8_t *b1_buf,
                             int b1_cap,
                             uint8_t *transport_buf,
                             int transport_cap,
                             bool robbed_bit);
bool vpcm_v91_session_run_startup(vpcm_v91_session_t *session,
                                  v91_state_t *caller,
                                  v91_state_t *answerer,
                                  const v91_dil_desc_t *default_dil,
                                  const vpcm_cp_frame_t *cp_offer_template,
                                  bool robbed_bit,
                                  uint8_t *startup_buf,
                                  int startup_cap,
                                  uint8_t *transport_buf,
                                  int transport_cap,
                                  uint8_t *scr_buf,
                                  int scr_cap,
                                  uint8_t *cp_buf,
                                  int cp_cap,
                                  uint8_t *es_buf,
                                  int es_cap,
                                  uint8_t *b1_buf,
                                  int b1_cap,
                                  vpcm_v91_startup_report_t *report,
                                  v91_state_t *caller_tx,
                                  v91_state_t *caller_rx,
                                  v91_state_t *answerer_tx,
                                  v91_state_t *answerer_rx);
bool vpcm_v91_session_activate_data(vpcm_v91_session_t *session,
                                    v91_state_t *caller,
                                    v91_state_t *answerer,
                                    const vpcm_cp_frame_t *cp_ack,
                                    v91_state_t *caller_tx,
                                    v91_state_t *caller_rx,
                                    v91_state_t *answerer_tx,
                                    v91_state_t *answerer_rx);
bool vpcm_v91_session_describe_chunk(const vpcm_cp_frame_t *cp_ack,
                                     int total_codewords,
                                     int codewords_per_report,
                                     int codeword_offset,
                                     int *chunk_codewords,
                                     int *chunk_bytes,
                                     int *byte_offset,
                                     int *frame_index);
bool vpcm_v91_session_codec_duplex_chunk(vpcm_v91_session_t *session,
                                         v91_state_t *caller_tx,
                                         v91_state_t *caller_rx,
                                         v91_state_t *answerer_tx,
                                         v91_state_t *answerer_rx,
                                         const uint8_t *caller_data_in,
                                         const uint8_t *answerer_data_in,
                                         uint8_t *caller_data_out,
                                         uint8_t *answerer_data_out,
                                         uint8_t *caller_pcm_tx,
                                         uint8_t *caller_pcm_rx,
                                         uint8_t *answerer_pcm_tx,
                                         uint8_t *answerer_pcm_rx,
                                         int codeword_offset,
                                         int chunk_codewords,
                                         int byte_offset,
                                         int chunk_bytes,
                                         int *caller_produced,
                                         int *answerer_produced,
                                         int *caller_consumed,
                                         int *answerer_consumed);
const char *vpcm_v91_modem_state_to_str(vpcm_v91_modem_state_t state);

#endif
