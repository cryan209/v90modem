#include "vpcm_v90_session.h"
#include "v90.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

enum { VPCM_V90_NOMINAL_10S_FRAMES = 13328 };

static bool vpcm_v90_record_simplex(const vpcm_v90_startup_contract_io_t *io,
                                    v91_law_t law,
                                    bool digital_to_analogue,
                                    const uint8_t *tx_codewords,
                                    int codeword_len)
{
    if (!io || !io->record_simplex_g711)
        return true;
    return io->record_simplex_g711(io->user_data,
                                   law,
                                   digital_to_analogue,
                                   tx_codewords,
                                   codeword_len);
}

static bool vpcm_v90_record_duplex(const vpcm_v90_startup_contract_io_t *io,
                                   const uint8_t *digital_tx_codewords,
                                   const uint8_t *analogue_tx_codewords,
                                   int codeword_len)
{
    if (!io || !io->record_duplex_g711)
        return true;
    return io->record_duplex_g711(io->user_data,
                                  digital_tx_codewords,
                                  analogue_tx_codewords,
                                  codeword_len);
}

static void vpcm_v90_fill_pattern(uint8_t *buf, int len, uint32_t seed)
{
    uint32_t x;
    int i;

    if (!buf || len <= 0)
        return;
    x = seed ? seed : 0x12345678U;
    for (i = 0; i < len; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        buf[i] = (uint8_t) (x & 0xFFU);
    }
}

static bool vpcm_v90_expect_equal(const char *label,
                                  const uint8_t *expected,
                                  const uint8_t *actual,
                                  int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (expected[i] != actual[i]) {
            fprintf(stderr,
                    "%s mismatch at byte %d: expected 0x%02X got 0x%02X\n",
                    label, i, expected[i], actual[i]);
            return false;
        }
    }
    return true;
}

static int vpcm_v90_frames_from_seconds(int seconds)
{
    long long frames;

    if (seconds <= 0)
        return 0;
    frames = ((long long) seconds * 8000LL) / (long long) VPCM_CP_FRAME_INTERVALS;
    if (frames < 1)
        frames = 1;
    if (frames > 2000000000LL)
        frames = 2000000000LL;
    return (int) frames;
}

static int vpcm_v90_align_frames_for_duplex_bytes(int frames,
                                                  int downstream_bits_per_frame,
                                                  int upstream_bits_per_frame)
{
    int aligned;

    if (frames < 1)
        return 1;
    aligned = frames;
    while (((aligned * downstream_bits_per_frame) % 8) != 0
           || ((aligned * upstream_bits_per_frame) % 8) != 0) {
        aligned++;
    }
    return aligned;
}

static void vpcm_v90_copy_sample(uint8_t *dst, int dst_len, const uint8_t *src, int src_len)
{
    if (!dst || dst_len <= 0)
        return;
    memset(dst, 0, (size_t) dst_len);
    if (!src || src_len <= 0)
        return;
    if (src_len > dst_len)
        src_len = dst_len;
    memcpy(dst, src, (size_t) src_len);
}

static v90_law_t vpcm_v90_data_law(v91_law_t law)
{
    return (law == V91_LAW_ALAW) ? V90_LAW_ALAW : V90_LAW_ULAW;
}

void vpcm_v90_session_init(vpcm_v90_session_t *session, v91_law_t law)
{
    if (!session)
        return;
    memset(session, 0, sizeof(*session));
    session->law = law;
    session->state = VPCM_V90_MODEM_IDLE;
}

void vpcm_v90_session_reset(vpcm_v90_session_t *session)
{
    if (!session)
        return;
    session->state = VPCM_V90_MODEM_IDLE;
    session->data_mode_active = false;
}

void vpcm_v90_session_set_state(vpcm_v90_session_t *session, vpcm_v90_modem_state_t state)
{
    if (!session)
        return;
    session->state = state;
    session->data_mode_active = (state == VPCM_V90_MODEM_DATA);
}

void vpcm_v92_init_digital_dil_from_ja(v91_dil_desc_t *desc, bool echo_limited)
{
    int i;
    static const uint8_t echo_train_u[8] = {60, 61, 62, 63, 64, 65, 66, 67};

    if (!desc)
        return;
    v91_default_dil_init(desc);
    if (!echo_limited)
        return;

    desc->n = 96;
    desc->lsp = 6;
    desc->ltp = 6;
    for (i = 0; i < 8; i++) {
        desc->ref[i] = 1;
        desc->h[i] = 1;
    }
    for (i = 0; i < desc->n; i++)
        desc->train_u[i] = echo_train_u[i % 8];
}

void vpcm_v92_select_profile_from_dil(const v91_dil_analysis_t *analysis,
                                      uint8_t *downstream_drn,
                                      uint8_t *upstream_drn)
{
    if (!downstream_drn || !upstream_drn)
        return;
    if (analysis && analysis->recommended_downstream_drn != 0 && analysis->recommended_upstream_drn != 0) {
        *downstream_drn = analysis->recommended_downstream_drn;
        *upstream_drn = analysis->recommended_upstream_drn;
    } else {
        *downstream_drn = 19;
        *upstream_drn = 16;
    }
}

bool vpcm_v90_session_run_startup_contract(vpcm_v90_session_t *session,
                                           const vpcm_v90_startup_contract_params_t *params,
                                           const vpcm_v90_startup_contract_io_t *io,
                                           vpcm_v90_startup_contract_report_t *report)
{
    v91_dil_desc_t default_dil;
    v91_dil_desc_t digital_dil;
    v91_dil_analysis_t digital_dil_analysis;
    v91_state_t caller_startup;
    v91_state_t answerer_startup;
    v91_state_t caller_rx;
    v91_state_t answerer_tx;
    v91_info_frame_t rx_info;
    vpcm_cp_frame_t cp_rx;
    vpcm_v90_startup_contract_report_t local_report;
    uint8_t transport_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t startup_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t scr_buf[18];
    uint8_t cp_buf[VPCM_CP_MAX_BITS];
    uint8_t es_buf[V91_ES_SYMBOLS];
    uint8_t b1_buf[V91_B1_SYMBOLS];
    uint8_t *down_data_in;
    uint8_t *down_data_out;
    uint8_t *up_data_in;
    uint8_t *up_data_out;
    uint8_t *down_pcm_tx;
    uint8_t *down_pcm_rx;
    uint8_t *up_pcm_tx;
    uint8_t *up_pcm_rx;
    v90_state_t *downstream_tx;
    v90_state_t *downstream_rx;
    int data_frames;
    int total_codewords;
    int up_total_bits;
    int down_total_bytes;
    int up_total_bytes;
    int startup_len;
    int cp_len;
    int down_produced;
    int up_produced;
    int down_consumed;
    int up_consumed;
    int data_seconds;
    uint8_t downstream_drn;
    uint8_t upstream_drn;

    if (!session || !params)
        return false;

    memset(&local_report, 0, sizeof(local_report));
    memset(&digital_dil_analysis, 0, sizeof(digital_dil_analysis));
    downstream_tx = NULL;
    downstream_rx = NULL;
    data_seconds = params->data_seconds;
    if (data_seconds <= 0)
        data_seconds = 10;

    vpcm_v90_session_init(session, params->law);
    v90_info0a_init(&local_report.analogue_info0a);
    v90_info1a_init(&local_report.analogue_info1a);
    local_report.phase2_contract_valid =
        v90_build_info0a_bits(local_report.analogue_info0a_bits,
                              (int) sizeof(local_report.analogue_info0a_bits),
                              &local_report.analogue_info0a)
        && v90_build_info1a_bits(local_report.analogue_info1a_bits,
                                 (int) sizeof(local_report.analogue_info1a_bits),
                                 &local_report.analogue_info1a);
    v91_default_dil_init(&default_dil);
    vpcm_v92_init_digital_dil_from_ja(&digital_dil, params->echo_limited);
    v91_init(&caller_startup, params->law, V91_MODE_TRANSPARENT);
    v91_init(&answerer_startup, params->law, V91_MODE_TRANSPARENT);
    v91_init(&caller_rx, params->law, V91_MODE_TRANSPARENT);
    v91_init(&answerer_tx, params->law, V91_MODE_TRANSPARENT);

    memset(&local_report.caller_info, 0, sizeof(local_report.caller_info));
    local_report.caller_info.request_default_dil = true;
    local_report.caller_info.tx_uses_alaw = (params->law == V91_LAW_ALAW);
    local_report.caller_info.power_measured_after_digital_impairments = true;

    memset(&local_report.answerer_info, 0, sizeof(local_report.answerer_info));
    local_report.answerer_info.request_default_dil = true;
    local_report.answerer_info.tx_uses_alaw = (params->law == V91_LAW_ALAW);
    local_report.answerer_info.power_measured_after_digital_impairments = true;

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE1);
    if (v91_tx_phase1_silence_codewords(&caller_startup, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS
        || v91_tx_phase1_silence_codewords(&answerer_startup, transport_buf, (int) sizeof(transport_buf)) != V91_PHASE1_SILENCE_SYMBOLS
        || !vpcm_v90_record_duplex(io, startup_buf, transport_buf, V91_PHASE1_SILENCE_SYMBOLS)
        || v91_tx_ez_codewords(&caller_startup, startup_buf, (int) sizeof(startup_buf)) != V91_EZ_SYMBOLS
        || v91_tx_ez_codewords(&answerer_startup, transport_buf, (int) sizeof(transport_buf)) != V91_EZ_SYMBOLS
        || !vpcm_v90_record_duplex(io, startup_buf, transport_buf, V91_EZ_SYMBOLS)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_INFO);
    if (v91_tx_info_codewords(&caller_startup, transport_buf, (int) sizeof(transport_buf), &local_report.caller_info) != V91_INFO_SYMBOLS
        || !v91_rx_info_codewords(&answerer_startup, transport_buf, V91_INFO_SYMBOLS, &rx_info)
        || !vpcm_v90_record_simplex(io, params->law, true, transport_buf, V91_INFO_SYMBOLS)
        || v91_tx_info_codewords(&answerer_startup, transport_buf, (int) sizeof(transport_buf), &local_report.answerer_info) != V91_INFO_SYMBOLS
        || !v91_rx_info_codewords(&caller_startup, transport_buf, V91_INFO_SYMBOLS, &rx_info)
        || !vpcm_v90_record_simplex(io, params->law, false, transport_buf, V91_INFO_SYMBOLS)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_DIL);
    startup_len = v91_tx_startup_dil_sequence_codewords(&caller_startup,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &digital_dil,
                                                        NULL);
    if (startup_len <= 0
        || !v91_note_received_dil(&answerer_startup, &digital_dil, &digital_dil_analysis)
        || !vpcm_v90_record_simplex(io, params->law, true, startup_buf, startup_len)) {
        return false;
    }
    local_report.digital_dil_analysis = digital_dil_analysis;
    local_report.digital_dil_analysis_valid = true;
    vpcm_v92_select_profile_from_dil(&digital_dil_analysis, &downstream_drn, &upstream_drn);

    startup_len = v91_tx_startup_dil_sequence_codewords(&answerer_startup,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &default_dil,
                                                        NULL);
    if (startup_len <= 0
        || !v91_note_received_dil(&caller_startup, &default_dil, NULL)
        || !vpcm_v90_record_simplex(io, params->law, false, startup_buf, startup_len)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_SCR);
    if (v91_tx_scr_codewords(&caller_startup, scr_buf, (int) sizeof(scr_buf), 18) != 18
        || !v91_rx_scr_codewords(&answerer_startup, scr_buf, 18, false)
        || !vpcm_v90_record_simplex(io, params->law, true, scr_buf, 18)
        || v91_tx_scr_codewords(&answerer_startup, scr_buf, (int) sizeof(scr_buf), 18) != 18
        || !v91_rx_scr_codewords(&caller_startup, scr_buf, 18, false)
        || !vpcm_v90_record_simplex(io, params->law, false, scr_buf, 18)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_DOWNSTREAM_CP);
    vpcm_cp_init(&local_report.cp_down_offer);
    local_report.cp_down_offer.transparent_mode_granted = false;
    local_report.cp_down_offer.v90_compatibility = true;
    local_report.cp_down_offer.drn = downstream_drn;
    local_report.cp_down_offer.constellation_count = 1;
    memset(local_report.cp_down_offer.dfi, 0, sizeof(local_report.cp_down_offer.dfi));
    vpcm_cp_enable_all_ucodes(local_report.cp_down_offer.masks[0]);
    cp_len = v91_tx_cp_codewords(&caller_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_down_offer,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&answerer_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_down_offer, &cp_rx)
        || !vpcm_v90_record_simplex(io, params->law, true, cp_buf, cp_len)) {
        return false;
    }

    local_report.cp_down_ack = local_report.cp_down_offer;
    local_report.cp_down_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&answerer_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_down_ack,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&caller_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_down_ack, &cp_rx)
        || !vpcm_v90_record_simplex(io, params->law, false, cp_buf, cp_len)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_DOWNSTREAM_B1);
    if (v91_tx_es_codewords(&caller_startup, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
        || !v91_rx_es_codewords(&answerer_startup, es_buf, V91_ES_SYMBOLS, true)
        || !vpcm_v90_record_simplex(io, params->law, true, es_buf, V91_ES_SYMBOLS)
        || v91_tx_b1_codewords(&caller_startup, b1_buf, (int) sizeof(b1_buf), &local_report.cp_down_ack) != V91_B1_SYMBOLS
        || !v91_rx_b1_codewords(&answerer_startup, b1_buf, V91_B1_SYMBOLS, &local_report.cp_down_ack)
        || !vpcm_v90_record_simplex(io, params->law, true, b1_buf, V91_B1_SYMBOLS)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_UPSTREAM_CP);
    vpcm_cp_init(&local_report.cp_up_offer);
    local_report.cp_up_offer.transparent_mode_granted = false;
    local_report.cp_up_offer.v90_compatibility = true;
    local_report.cp_up_offer.drn = upstream_drn;
    local_report.cp_up_offer.constellation_count = 1;
    memset(local_report.cp_up_offer.dfi, 0, sizeof(local_report.cp_up_offer.dfi));
    vpcm_cp_enable_all_ucodes(local_report.cp_up_offer.masks[0]);
    cp_len = v91_tx_cp_codewords(&answerer_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_up_offer,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&caller_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_up_offer, &cp_rx)
        || !vpcm_v90_record_simplex(io, params->law, false, cp_buf, cp_len)) {
        return false;
    }

    local_report.cp_up_ack = local_report.cp_up_offer;
    local_report.cp_up_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&caller_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_up_ack,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&answerer_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_up_ack, &cp_rx)
        || !vpcm_v90_record_simplex(io, params->law, true, cp_buf, cp_len)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_UPSTREAM_B1);
    if (v91_tx_es_codewords(&answerer_startup, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
        || !v91_rx_es_codewords(&caller_startup, es_buf, V91_ES_SYMBOLS, true)
        || !vpcm_v90_record_simplex(io, params->law, false, es_buf, V91_ES_SYMBOLS)
        || v91_tx_b1_codewords(&answerer_startup, b1_buf, (int) sizeof(b1_buf), &local_report.cp_up_ack) != V91_B1_SYMBOLS
        || !v91_rx_b1_codewords(&caller_startup, b1_buf, V91_B1_SYMBOLS, &local_report.cp_up_ack)
        || !vpcm_v90_record_simplex(io, params->law, false, b1_buf, V91_B1_SYMBOLS)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_DATA);
    downstream_tx = v90_init_data_pump(vpcm_v90_data_law(params->law));
    downstream_rx = v90_init_data_pump(vpcm_v90_data_law(params->law));
    if (!downstream_tx
        || !downstream_rx
        || !v91_activate_data_mode(&answerer_tx, &local_report.cp_up_ack)
        || !v91_activate_data_mode(&caller_rx, &local_report.cp_up_ack)) {
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        return false;
    }

    data_frames = vpcm_v90_frames_from_seconds(data_seconds);
    if (data_frames < 1)
        data_frames = VPCM_V90_NOMINAL_10S_FRAMES;
    data_frames = vpcm_v90_align_frames_for_duplex_bytes(data_frames,
                                                         8 * VPCM_CP_FRAME_INTERVALS,
                                                         (int) local_report.cp_up_ack.drn + 20);
    total_codewords = data_frames * VPCM_CP_FRAME_INTERVALS;
    up_total_bits = data_frames * ((int) local_report.cp_up_ack.drn + 20);
    down_total_bytes = total_codewords;
    up_total_bytes = up_total_bits / 8;

    down_data_in = (uint8_t *) malloc((size_t) down_total_bytes);
    down_data_out = (uint8_t *) malloc((size_t) down_total_bytes);
    up_data_in = (uint8_t *) malloc((size_t) up_total_bytes);
    up_data_out = (uint8_t *) malloc((size_t) up_total_bytes);
    down_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    down_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    up_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    up_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    if (!down_data_in || !down_data_out || !up_data_in || !up_data_out
        || !down_pcm_tx || !down_pcm_rx || !up_pcm_tx || !up_pcm_rx) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        return false;
    }

    vpcm_v90_fill_pattern(down_data_in, down_total_bytes, params->seed_base ^ 0x00D04E00U);
    vpcm_v90_fill_pattern(up_data_in, up_total_bytes, params->seed_base ^ 0x00A0B000U);
    memset(down_data_out, 0, (size_t) down_total_bytes);
    memset(up_data_out, 0, (size_t) up_total_bytes);

    down_produced = v90_tx_codewords(downstream_tx, down_pcm_tx, total_codewords, down_data_in, down_total_bytes);
    up_produced = v91_tx_codewords(&answerer_tx, up_pcm_tx, total_codewords, up_data_in, up_total_bytes);
    if (down_produced != total_codewords
        || up_produced != total_codewords
        || !vpcm_v90_record_duplex(io, down_pcm_tx, up_pcm_tx, total_codewords)) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        return false;
    }

    memcpy(down_pcm_rx, down_pcm_tx, (size_t) total_codewords);
    memcpy(up_pcm_rx, up_pcm_tx, (size_t) total_codewords);
    down_consumed = v90_rx_codewords(downstream_rx, down_data_out, down_total_bytes, down_pcm_rx, total_codewords);
    up_consumed = v91_rx_codewords(&caller_rx, up_data_out, up_total_bytes, up_pcm_rx, total_codewords);
    if (down_consumed != down_total_bytes
        || up_consumed != up_total_bytes
        || !vpcm_v90_expect_equal("V.90 downstream payload", down_data_in, down_data_out, down_total_bytes)
        || !vpcm_v90_expect_equal("V.92 upstream payload", up_data_in, up_data_out, up_total_bytes)) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        return false;
    }

    local_report.data_seconds = data_seconds;
    local_report.total_codewords = total_codewords;
    local_report.down_total_bytes = down_total_bytes;
    local_report.up_total_bytes = up_total_bytes;
    local_report.sample_down_data_len = VPCM_V90_SAMPLE_PCM_LEN;
    local_report.sample_up_data_len = (VPCM_V90_SAMPLE_PCM_LEN * ((int) local_report.cp_up_ack.drn + 20)) / 48;
    if (local_report.sample_down_data_len < 1)
        local_report.sample_down_data_len = 1;
    if (local_report.sample_up_data_len < 1)
        local_report.sample_up_data_len = 1;
    if (local_report.sample_down_data_len > VPCM_V90_SAMPLE_DATA_MAX)
        local_report.sample_down_data_len = VPCM_V90_SAMPLE_DATA_MAX;
    if (local_report.sample_up_data_len > VPCM_V90_SAMPLE_DATA_MAX)
        local_report.sample_up_data_len = VPCM_V90_SAMPLE_DATA_MAX;
    if (local_report.sample_down_data_len > down_total_bytes)
        local_report.sample_down_data_len = down_total_bytes;
    if (local_report.sample_up_data_len > up_total_bytes)
        local_report.sample_up_data_len = up_total_bytes;

    vpcm_v90_copy_sample(local_report.down_data_in_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         down_data_in,
                         local_report.sample_down_data_len);
    vpcm_v90_copy_sample(local_report.down_pcm_tx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         down_pcm_tx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.down_pcm_rx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         down_pcm_rx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.down_data_out_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         down_data_out,
                         local_report.sample_down_data_len);
    vpcm_v90_copy_sample(local_report.up_data_in_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         up_data_in,
                         local_report.sample_up_data_len);
    vpcm_v90_copy_sample(local_report.up_pcm_tx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         up_pcm_tx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.up_pcm_rx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         up_pcm_rx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.up_data_out_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         up_data_out,
                         local_report.sample_up_data_len);

    free(down_data_in);
    free(down_data_out);
    free(up_data_in);
    free(up_data_out);
    free(down_pcm_tx);
    free(down_pcm_rx);
    free(up_pcm_tx);
    free(up_pcm_rx);
    v90_free(downstream_tx);
    v90_free(downstream_rx);

    if (report)
        *report = local_report;
    return true;
}

const char *vpcm_v90_modem_state_to_str(vpcm_v90_modem_state_t state)
{
    switch (state) {
    case VPCM_V90_MODEM_IDLE: return "IDLE";
    case VPCM_V90_MODEM_PHASE1: return "PHASE1";
    case VPCM_V90_MODEM_INFO: return "INFO";
    case VPCM_V90_MODEM_DIL: return "DIL";
    case VPCM_V90_MODEM_SCR: return "SCR";
    case VPCM_V90_MODEM_DOWNSTREAM_CP: return "DOWNSTREAM_CP";
    case VPCM_V90_MODEM_DOWNSTREAM_B1: return "DOWNSTREAM_B1";
    case VPCM_V90_MODEM_UPSTREAM_CP: return "UPSTREAM_CP";
    case VPCM_V90_MODEM_UPSTREAM_B1: return "UPSTREAM_B1";
    case VPCM_V90_MODEM_DATA: return "DATA";
    case VPCM_V90_MODEM_CLEARDOWN: return "CLEARDOWN";
    default: return "UNKNOWN";
    }
}
