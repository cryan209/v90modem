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
