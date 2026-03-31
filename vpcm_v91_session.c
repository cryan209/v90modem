#include "vpcm_v91_session.h"

#include <string.h>

static void vpcm_v91_transport_codewords(bool robbed_bit,
                                         uint8_t *dst,
                                         const uint8_t *src,
                                         int len)
{
    int i;

    if (!dst || !src || len <= 0)
        return;
    if (!robbed_bit) {
        memcpy(dst, src, (size_t) len);
        return;
    }
    memcpy(dst, src, (size_t) len);
    for (i = 5; i < len; i += 6)
        dst[i] &= 0xFE;
}

static bool vpcm_v91_cp_frames_equal(const vpcm_cp_frame_t *a, const vpcm_cp_frame_t *b)
{
    if (!a || !b)
        return false;
    return a->transparent_mode_granted == b->transparent_mode_granted
        && a->v90_compatibility == b->v90_compatibility
        && a->drn == b->drn
        && a->acknowledge == b->acknowledge
        && a->constellation_count == b->constellation_count
        && memcmp(a->dfi, b->dfi, sizeof(a->dfi)) == 0
        && memcmp(a->masks, b->masks, sizeof(a->masks)) == 0;
}

void vpcm_v91_session_init(vpcm_v91_session_t *session, v91_law_t law)
{
    if (!session)
        return;
    memset(session, 0, sizeof(*session));
    session->law = law;
    session->state = VPCM_V91_MODEM_IDLE;
}

void vpcm_v91_session_reset(vpcm_v91_session_t *session)
{
    if (!session)
        return;
    session->state = VPCM_V91_MODEM_IDLE;
    session->data_mode_active = false;
}

void vpcm_v91_session_set_state(vpcm_v91_session_t *session, vpcm_v91_modem_state_t state)
{
    if (!session)
        return;
    session->state = state;
    session->data_mode_active = (state == VPCM_V91_MODEM_DATA);
}

void vpcm_v91_startup_cfg_init(vpcm_v91_startup_cfg_t *cfg,
                               v91_law_t law,
                               const vpcm_cp_frame_t *cp_offer_template)
{
    if (!cfg || !cp_offer_template)
        return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->caller_info.request_default_dil = true;
    cfg->caller_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    cfg->caller_info.power_measured_after_digital_impairments = true;
    cfg->caller_info.request_transparent_mode = cp_offer_template->transparent_mode_granted;
    cfg->caller_info.cleardown_if_transparent_denied = cp_offer_template->transparent_mode_granted;

    cfg->answerer_info.request_default_dil = true;
    cfg->answerer_info.tx_uses_alaw = (law == V91_LAW_ALAW);
    cfg->answerer_info.power_measured_after_digital_impairments = true;
    cfg->answerer_info.request_transparent_mode = cp_offer_template->transparent_mode_granted;
    cfg->answerer_info.cleardown_if_transparent_denied = cp_offer_template->transparent_mode_granted;

    cfg->cp_offer = *cp_offer_template;
    cfg->cp_ack = *cp_offer_template;
    cfg->cp_ack.acknowledge = true;
}

void vpcm_v91_session_enter_phase1(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_PHASE1);
}

void vpcm_v91_session_enter_info(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_INFO);
}

void vpcm_v91_session_enter_dil(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_DIL);
}

void vpcm_v91_session_enter_scr(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_SCR);
}

void vpcm_v91_session_enter_cp(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_CP);
}

void vpcm_v91_session_enter_b1(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_B1);
}

void vpcm_v91_session_enter_data(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_DATA);
}

void vpcm_v91_session_enter_cleardown(vpcm_v91_session_t *session)
{
    vpcm_v91_session_set_state(session, VPCM_V91_MODEM_CLEARDOWN);
}

bool vpcm_v91_session_run_phase1(vpcm_v91_session_t *session,
                                 v91_state_t *caller,
                                 v91_state_t *answerer,
                                 uint8_t *startup_buf,
                                 int startup_cap,
                                 uint8_t *transport_buf,
                                 int transport_cap)
{
    if (!session || !caller || !answerer || !startup_buf || !transport_buf)
        return false;
    vpcm_v91_session_enter_phase1(session);
    if (v91_tx_phase1_silence_codewords(caller, startup_buf, startup_cap) != V91_PHASE1_SILENCE_SYMBOLS
        || v91_tx_phase1_silence_codewords(answerer, startup_buf, startup_cap) != V91_PHASE1_SILENCE_SYMBOLS)
        return false;
    if (v91_tx_ez_codewords(caller, transport_buf, transport_cap) != V91_EZ_SYMBOLS
        || v91_tx_ez_codewords(answerer, transport_buf, transport_cap) != V91_EZ_SYMBOLS)
        return false;
    return true;
}

bool vpcm_v91_session_run_info(vpcm_v91_session_t *session,
                               v91_state_t *caller,
                               v91_state_t *answerer,
                               const vpcm_v91_startup_cfg_t *cfg,
                               v91_info_frame_t *rx_info,
                               uint8_t *transport_buf,
                               int transport_cap,
                               bool robbed_bit)
{
    if (!session || !caller || !answerer || !cfg || !rx_info || !transport_buf)
        return false;
    vpcm_v91_session_enter_info(session);
    if (v91_tx_info_codewords(caller, transport_buf, transport_cap, &cfg->caller_info) != V91_INFO_SYMBOLS)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, transport_buf, V91_INFO_SYMBOLS);
    if (!v91_rx_info_codewords(answerer, transport_buf, V91_INFO_SYMBOLS, rx_info))
        return false;

    if (v91_tx_info_codewords(answerer, transport_buf, transport_cap, &cfg->answerer_info) != V91_INFO_SYMBOLS)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, transport_buf, V91_INFO_SYMBOLS);
    if (!v91_rx_info_codewords(caller, transport_buf, V91_INFO_SYMBOLS, rx_info))
        return false;
    return true;
}

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
                              bool robbed_bit)
{
    int len;

    if (!session || !caller || !answerer || !default_dil || !startup_buf || !transport_buf
        || !caller_startup_len || !answerer_startup_len)
        return false;
    vpcm_v91_session_enter_dil(session);
    len = v91_tx_startup_dil_sequence_codewords(caller, startup_buf, startup_cap, default_dil, NULL);
    if (len <= 0)
        return false;
    *caller_startup_len = len;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, startup_buf, len);

    len = v91_tx_startup_dil_sequence_codewords(answerer, startup_buf, startup_cap, default_dil, NULL);
    if (len <= 0)
        return false;
    *answerer_startup_len = len;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, startup_buf, len);
    (void) transport_cap;
    return true;
}

bool vpcm_v91_session_run_scr(vpcm_v91_session_t *session,
                              v91_state_t *caller,
                              v91_state_t *answerer,
                              uint8_t *scr_buf,
                              int scr_cap,
                              uint8_t *transport_buf,
                              int transport_cap,
                              bool robbed_bit)
{
    if (!session || !caller || !answerer || !scr_buf || !transport_buf)
        return false;
    vpcm_v91_session_enter_scr(session);
    if (v91_tx_scr_codewords(caller, scr_buf, scr_cap, 18) != 18)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, scr_buf, 18);
    if (!v91_rx_scr_codewords(answerer, transport_buf, 18, false))
        return false;

    if (v91_tx_scr_codewords(answerer, scr_buf, scr_cap, 18) != 18)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, scr_buf, 18);
    if (!v91_rx_scr_codewords(caller, transport_buf, 18, false))
        return false;
    (void) transport_cap;
    return true;
}

bool vpcm_v91_session_run_cp(vpcm_v91_session_t *session,
                             v91_state_t *caller,
                             v91_state_t *answerer,
                             const vpcm_v91_startup_cfg_t *cfg,
                             vpcm_cp_frame_t *cp_rx,
                             uint8_t *cp_buf,
                             int cp_cap,
                             uint8_t *transport_buf,
                             int transport_cap,
                             bool robbed_bit)
{
    int cp_len;

    if (!session || !caller || !answerer || !cfg || !cp_rx || !cp_buf || !transport_buf)
        return false;
    vpcm_v91_session_enter_cp(session);
    cp_len = v91_tx_cp_codewords(caller, cp_buf, cp_cap, &cfg->cp_offer, true);
    if (cp_len <= 0)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, cp_buf, cp_len);
    if (!v91_rx_cp_codewords(answerer, transport_buf, cp_len, cp_rx, true)
        || !vpcm_v91_cp_frames_equal(&cfg->cp_offer, cp_rx))
        return false;

    cp_len = v91_tx_cp_codewords(answerer, cp_buf, cp_cap, &cfg->cp_ack, true);
    if (cp_len <= 0)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, cp_buf, cp_len);
    if (!v91_rx_cp_codewords(caller, transport_buf, cp_len, cp_rx, true)
        || !vpcm_v91_cp_frames_equal(&cfg->cp_ack, cp_rx))
        return false;
    (void) transport_cap;
    return true;
}

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
                             bool robbed_bit)
{
    if (!session || !caller || !answerer || !cp_ack || !es_buf || !b1_buf || !transport_buf)
        return false;
    vpcm_v91_session_enter_b1(session);
    if (v91_tx_es_codewords(caller, es_buf, es_cap) != V91_ES_SYMBOLS)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, es_buf, V91_ES_SYMBOLS);
    if (!v91_rx_es_codewords(answerer, transport_buf, V91_ES_SYMBOLS, true))
        return false;

    if (v91_tx_b1_codewords(caller, b1_buf, b1_cap, cp_ack) != V91_B1_SYMBOLS)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, b1_buf, V91_B1_SYMBOLS);
    if (!v91_rx_b1_codewords(answerer, transport_buf, V91_B1_SYMBOLS, cp_ack))
        return false;

    if (v91_tx_es_codewords(answerer, es_buf, es_cap) != V91_ES_SYMBOLS)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, es_buf, V91_ES_SYMBOLS);
    if (!v91_rx_es_codewords(caller, transport_buf, V91_ES_SYMBOLS, true))
        return false;

    if (v91_tx_b1_codewords(answerer, b1_buf, b1_cap, cp_ack) != V91_B1_SYMBOLS)
        return false;
    vpcm_v91_transport_codewords(robbed_bit, transport_buf, b1_buf, V91_B1_SYMBOLS);
    if (!v91_rx_b1_codewords(caller, transport_buf, V91_B1_SYMBOLS, cp_ack))
        return false;
    (void) transport_cap;
    return true;
}

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
                                  v91_state_t *answerer_rx)
{
    vpcm_v91_startup_cfg_t startup_cfg;
    v91_info_frame_t rx_info;
    vpcm_cp_frame_t cp_rx;
    vpcm_v91_startup_report_t local_report;

    if (!session || !caller || !answerer || !default_dil || !cp_offer_template
        || !startup_buf || !transport_buf || !scr_buf || !cp_buf || !es_buf || !b1_buf
        || !caller_tx || !caller_rx || !answerer_tx || !answerer_rx) {
        return false;
    }

    memset(&local_report, 0, sizeof(local_report));
    vpcm_v91_startup_cfg_init(&startup_cfg, session->law, cp_offer_template);
    local_report.caller_info = startup_cfg.caller_info;
    local_report.answerer_info = startup_cfg.answerer_info;

    if (!vpcm_v91_session_run_phase1(session,
                                     caller,
                                     answerer,
                                     startup_buf,
                                     startup_cap,
                                     transport_buf,
                                     transport_cap)) {
        return false;
    }
    if (!vpcm_v91_session_run_info(session,
                                   caller,
                                   answerer,
                                   &startup_cfg,
                                   &rx_info,
                                   transport_buf,
                                   transport_cap,
                                   robbed_bit)) {
        return false;
    }
    if (!vpcm_v91_session_run_dil(session,
                                  caller,
                                  answerer,
                                  default_dil,
                                  startup_buf,
                                  startup_cap,
                                  transport_buf,
                                  transport_cap,
                                  &local_report.caller_startup_len,
                                  &local_report.answerer_startup_len,
                                  robbed_bit)) {
        return false;
    }
    if (!vpcm_v91_session_run_scr(session,
                                  caller,
                                  answerer,
                                  scr_buf,
                                  scr_cap,
                                  transport_buf,
                                  transport_cap,
                                  robbed_bit)) {
        return false;
    }
    if (!vpcm_v91_session_run_cp(session,
                                 caller,
                                 answerer,
                                 &startup_cfg,
                                 &cp_rx,
                                 cp_buf,
                                 cp_cap,
                                 transport_buf,
                                 transport_cap,
                                 robbed_bit)) {
        return false;
    }
    local_report.cp_ack = startup_cfg.cp_ack;
    if (!vpcm_v91_session_run_b1(session,
                                 caller,
                                 answerer,
                                 &local_report.cp_ack,
                                 es_buf,
                                 es_cap,
                                 b1_buf,
                                 b1_cap,
                                 transport_buf,
                                 transport_cap,
                                 robbed_bit)) {
        return false;
    }
    if (!vpcm_v91_session_activate_data(session,
                                        caller,
                                        answerer,
                                        &local_report.cp_ack,
                                        caller_tx,
                                        caller_rx,
                                        answerer_tx,
                                        answerer_rx)) {
        return false;
    }
    if (report)
        *report = local_report;
    return true;
}

bool vpcm_v91_session_activate_data(vpcm_v91_session_t *session,
                                    v91_state_t *caller,
                                    v91_state_t *answerer,
                                    const vpcm_cp_frame_t *cp_ack,
                                    v91_state_t *caller_tx,
                                    v91_state_t *caller_rx,
                                    v91_state_t *answerer_tx,
                                    v91_state_t *answerer_rx)
{
    if (!session || !caller || !answerer || !cp_ack
        || !caller_tx || !caller_rx || !answerer_tx || !answerer_rx)
        return false;
    if (!v91_activate_data_mode(caller, cp_ack) || !v91_activate_data_mode(answerer, cp_ack))
        return false;
    *caller_tx = *caller;
    *caller_rx = *caller;
    *answerer_tx = *answerer;
    *answerer_rx = *answerer;
    vpcm_v91_session_enter_data(session);
    return true;
}

bool vpcm_v91_session_describe_chunk(const vpcm_cp_frame_t *cp_ack,
                                     int total_codewords,
                                     int codewords_per_report,
                                     int codeword_offset,
                                     int *chunk_codewords,
                                     int *chunk_bytes,
                                     int *byte_offset,
                                     int *frame_index)
{
    int chunk_frames;

    if (!cp_ack || !chunk_codewords || !chunk_bytes || !byte_offset || !frame_index)
        return false;
    *chunk_codewords = total_codewords - codeword_offset;
    if (*chunk_codewords > codewords_per_report)
        *chunk_codewords = codewords_per_report;
    if (*chunk_codewords <= 0)
        return false;
    chunk_frames = *chunk_codewords / VPCM_CP_FRAME_INTERVALS;
    *chunk_bytes = (chunk_frames * ((int) cp_ack->drn + 20)) / 8;
    *byte_offset = (codeword_offset / VPCM_CP_FRAME_INTERVALS) * ((int) cp_ack->drn + 20) / 8;
    *frame_index = codeword_offset / VPCM_CP_FRAME_INTERVALS;
    return true;
}

bool vpcm_v91_session_encode_duplex_chunk(vpcm_v91_session_t *session,
                                          v91_state_t *caller_tx,
                                          v91_state_t *answerer_tx,
                                          const uint8_t *caller_data_in,
                                          const uint8_t *answerer_data_in,
                                          uint8_t *caller_pcm_tx,
                                          uint8_t *answerer_pcm_tx,
                                          int codeword_offset,
                                          int chunk_codewords,
                                          int byte_offset,
                                          int chunk_bytes,
                                          int *caller_produced,
                                          int *answerer_produced)
{
    if (!session || !caller_tx || !answerer_tx
        || !caller_data_in || !answerer_data_in
        || !caller_pcm_tx || !answerer_pcm_tx
        || !caller_produced || !answerer_produced)
        return false;
    if (!session->data_mode_active)
        return false;

    *caller_produced = v91_tx_codewords(caller_tx,
                                        caller_pcm_tx + codeword_offset,
                                        chunk_codewords,
                                        caller_data_in + byte_offset,
                                        chunk_bytes);
    *answerer_produced = v91_tx_codewords(answerer_tx,
                                          answerer_pcm_tx + codeword_offset,
                                          chunk_codewords,
                                          answerer_data_in + byte_offset,
                                          chunk_bytes);
    return *caller_produced == chunk_codewords && *answerer_produced == chunk_codewords;
}

bool vpcm_v91_session_decode_duplex_chunk(vpcm_v91_session_t *session,
                                          v91_state_t *caller_rx,
                                          v91_state_t *answerer_rx,
                                          uint8_t *caller_data_out,
                                          uint8_t *answerer_data_out,
                                          const uint8_t *caller_pcm_rx,
                                          const uint8_t *answerer_pcm_rx,
                                          int codeword_offset,
                                          int chunk_codewords,
                                          int byte_offset,
                                          int chunk_bytes,
                                          int *caller_consumed,
                                          int *answerer_consumed)
{
    if (!session || !caller_rx || !answerer_rx
        || !caller_data_out || !answerer_data_out
        || !caller_pcm_rx || !answerer_pcm_rx
        || !caller_consumed || !answerer_consumed)
        return false;
    if (!session->data_mode_active)
        return false;

    *answerer_consumed = v91_rx_codewords(answerer_rx,
                                          answerer_data_out + byte_offset,
                                          chunk_bytes,
                                          answerer_pcm_rx + codeword_offset,
                                          chunk_codewords);
    *caller_consumed = v91_rx_codewords(caller_rx,
                                        caller_data_out + byte_offset,
                                        chunk_bytes,
                                        caller_pcm_rx + codeword_offset,
                                        chunk_codewords);
    return *answerer_consumed == chunk_bytes && *caller_consumed == chunk_bytes;
}

const char *vpcm_v91_modem_state_to_str(vpcm_v91_modem_state_t state)
{
    switch (state) {
    case VPCM_V91_MODEM_IDLE: return "IDLE";
    case VPCM_V91_MODEM_V8: return "V8";
    case VPCM_V91_MODEM_PHASE1: return "PHASE1";
    case VPCM_V91_MODEM_INFO: return "INFO";
    case VPCM_V91_MODEM_DIL: return "DIL";
    case VPCM_V91_MODEM_SCR: return "SCR";
    case VPCM_V91_MODEM_CP: return "CP";
    case VPCM_V91_MODEM_B1: return "B1";
    case VPCM_V91_MODEM_DATA: return "DATA";
    case VPCM_V91_MODEM_CLEARDOWN: return "CLEARDOWN";
    default: return "UNKNOWN";
    }
}
