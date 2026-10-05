#include "legacy_pcm_decode.h"
#include "x2_session.h"
#include "k56flex_client.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    legacy_pcm_event_fn emit;
    void *user;
    x2_mp_rx_t *rx;
    x2_mp_t last;
    int have_last;
} mp_events_t;

static void mp_received(void *user, const x2_mp_t *mp)
{
    mp_events_t *e = user;
    char detail[512];
    /* Each audio timing hypothesis can report the same repeat. Retain a
     * changed record (including ACK/tail), not twenty duplicate events. */
    if (e->have_last && !memcmp(e->last.words, mp->words, sizeof(mp->words))
        && e->last.tail == mp->tail) return;
    e->last = *mp;
    e->have_last = 1;
    snprintf(detail, sizeof(detail),
             "source=x2_mp_rx timestamp=detection crc=valid repeated_protected_words=1 words=%04x/%04x/%04x/%04x ack=%u tail=%u carrier_hz=1920 baud=3200 payload_verified=0",
             mp->words[0], mp->words[1], mp->words[2], mp->words[3],
             (mp->words[0] >> 15) & 1, mp->tail);
    e->emit(e->user, (int)e->rx->samples, "x2", "MP qualified", detail);
}

/* Draft 0.33 clause 3: the digital-side 17-bit INFO0 is differential
 * 600 baud on 2400 Hz. Keep the full 17-bit body (including ACK), and
 * require two identical CRC-valid bodies before labelling this channel.
 * This is receive-only; no session tone schedule is inferred. */
typedef struct {
    unsigned clock, sync, count, repeats, last_body;
    double re, im, previous_re, previous_im;
    uint8_t bits[33];
} digital_info_hyp_t;

static void digital_info_sample(digital_info_hyp_t h[40], int16_t sample,
                                 size_t n, unsigned *reported, int *have_reported,
                                 legacy_pcm_event_fn event, void *user)
{
    const double pi = 3.14159265358979323846;
    double phase = 2 * pi * 2400 * (double)(n % 10) / 8000;
    for (unsigned i = 0; i < 40; i++) {
        digital_info_hyp_t *p = &h[i];
        p->re += sample * cos(phase); p->im -= sample * sin(phase);
        p->clock += 600;
        if (p->clock < 8000) continue;
        p->clock -= 8000;
        double power = p->re*p->re + p->im*p->im;
        double previous_power = p->previous_re*p->previous_re + p->previous_im*p->previous_im;
        unsigned bit = p->re*p->previous_re + p->im*p->previous_im < 0;
        if (power > 10000 && previous_power > 10000) {
            if (!p->count) {
                p->sync = ((p->sync << 1) | bit) & 255;
                if (p->sync == 0x72) p->count = 1;
            } else {
                p->bits[p->count - 1] = (uint8_t)bit;
                if (++p->count == 34) {
                    uint32_t body;
                    p->count = 0;
                    if (!x2_info_decode(p->bits, 17, &body)
                        && (body & 0x1840) == 0x1840) {
                        p->repeats = p->repeats && p->last_body == body ? p->repeats + 1 : 1;
                        p->last_body = body;
                        if (p->repeats >= 2 && (!*have_reported || *reported != body)) {
                            char detail[256];
                            snprintf(detail, sizeof(detail),
                                     "source=x2_info_decode timestamp=detection crc=valid repeated_body=1 body=%05x ack=%u carrier_hz=2400 role=digital_peer protocol_unconfirmed=1",
                                     (unsigned)body, (unsigned)(body >> 16));
                            event(user, (int)n, "Diagnostic", "x2-compatible digital INFO0", detail);
                            *reported = body; *have_reported = 1;
                        }
                    } else p->repeats = 0;
                }
            }
        } else { p->count = p->sync = 0; }
        p->previous_re = p->re; p->previous_im = p->im;
        p->re = p->im = 0;
    }
}

int legacy_pcm_decode_x2(const int16_t *samples, size_t count,
                         legacy_pcm_event_fn event, void *user)
{
    x2_session_t *info;
    x2_mp_rx_t *mp;
    mp_events_t events;
    int info_seen = 0, marker_seen = 0, digital_reported = 0;
    unsigned digital_body = 0;
    int info_sample = 0;
    char info_detail[512] = "";
    digital_info_hyp_t digital[40] = {0};
    for (unsigned i = 0; i < 40; i++) digital[i].clock = i * 200;
    char detail[512];
    if (!samples || !event || count > INT_MAX) return -1;
    info = malloc(sizeof(*info));
    mp = malloc(sizeof(*mp));
    if (!info || !mp) { free(info); free(mp); return -1; }
    if (x2_session_init(info)) { free(info); free(mp); return -1; }
    memset(&events, 0, sizeof(events));
    events.emit = event; events.user = user; events.rx = mp;
    x2_mp_rx_init(mp, mp_received, &events);
    /* Reuse Draft 0.33's recovered INFO/marker and Courier MP receivers.
     * A recorded signal cannot respond to TX; do not run session_tx() to
     * invent a dialogue or use transmitter-stage changes as wire evidence.
     * The MP receiver independently qualifies two agreeing CRC-valid frames,
     * so a capture beginning after INFO0 is still useful. */
    for (size_t n = 0; n < count; n++) {
        digital_info_sample(digital, samples[n], n, &digital_body, &digital_reported, event, user);
        x2_session_rx(info, samples + n, 1);
        if (!info_seen && info->peer_info_valid) {
            info_seen = 1;
            snprintf(detail, sizeof(detail),
                     "source=x2_session_rx timestamp=detection crc=valid body=%04x carrier_hz=1200 role=analogue_peer single_frame=1",
                     info->peer_capabilities);
            /* V.34 (10/1996) 10.1.2.3.3/Table 14's INFO0 shares this 17-bit
             * framing. Gough Lui plain-V.34 captures decode the same 21ff
             * body: CRC confirms the frame, not x2 selection. Draft 0.33
             * clause 3/4's supported directional marker qualifies the path. */
            info_sample = (int)n;
            snprintf(info_detail, sizeof(info_detail), "%s", detail);
            event(user, (int)n, "Diagnostic", "x2-compatible peer INFO0", detail);
        }
        if (!marker_seen && info->marker_valid) {
            unsigned index, high;
            marker_seen = 1;
            if (info_seen) {
                snprintf(detail, sizeof(detail), "%s qualified_by=directional_marker", info_detail);
                event(user, info_sample, "x2", "Peer INFO0 decoded", detail);
            }
            x2_marker_parse(info->marker, 1, &index, &high);
            snprintf(detail, sizeof(detail),
                     "source=x2_session_rx timestamp=detection crc=valid marker=%02x profile_index=%u high_carrier=%u role=analogue_peer",
                     info->marker, index, high);
            event(user, (int)n, "x2", "Directional marker decoded", detail);
        }
        x2_mp_rx_audio(mp, samples + n, 1);
    }
    if (mp->e_detected) {
        snprintf(detail, sizeof(detail),
                 "source=x2_mp_rx timestamp=detection qualified_mp=1 consecutive_ones=20 timing=%u phase=%u payload_verified=0",
                 mp->e_timing, mp->e_phase);
        event(user, (int)mp->e_sample, "x2", "Upstream E detected", detail);
    }
    free(info); free(mp);
    return 0;
}

int legacy_pcm_decode_flex(const int16_t *samples, size_t count, int alaw,
                           legacy_pcm_event_fn event, void *user)
{
    k56flex_client_cfg_t cfg;
    k56flex_client_t *client;
    int locked = 0, param = 0;
    size_t processed = 0;
    k56flex_train_phase_t previous = K56T_SIL0;
    char detail[512];
    if (!samples || !event || count > INT_MAX) return -1;
    memset(&cfg, 0, sizeof(cfg));
    cfg.law = alaw ? K56FLEX_LAW_A : K56FLEX_LAW_MU;
    /* No recovered upstream report waveform: these are receiver-model
     * assumptions, NOT peer-negotiated fields. Do not export user bits. */
    cfg.report_word = 0x8880;
    cfg.gate_b_pairs = 2;
    client = k56flex_client_new(&cfg);
    if (!client) return -1;
    /* Draft 0.23 4.10..4.12; receiver acquisition and observed transitions
     * are described in docs/k56flex_implementation.md. The client has no
     * upstream callbacks here: received audio alone must release its gates. */
    for (size_t n = 0; n < count && !client->failed; n++) {
        processed = n + 1;
        k56flex_client_rx_linear(client, samples + n, 1);
        if (!locked && client->fe_started) {
            locked = 1;
            snprintf(detail, sizeof(detail),
                     "source=k56flex_rxfe timestamp=detection p1_anchor_sample=%u law=%s gain=%.6g receiver_model=1 report_assumed=8880 report_ext_assumed=00",
                     k56flex_rxfe_locked_at(client->fe), alaw ? "alaw" : "ulaw",
                     k56flex_rxfe_gain(client->fe));
            event(user, (int)n, "K56flex", "P1 probe acquired", detail);
        }
        if (locked && previous != k56flex_client_phase(client)) {
            previous = k56flex_client_phase(client);
            snprintf(detail, sizeof(detail),
                     "source=k56flex_client timestamp=detection model_phase=%s checked_blocks=%u mismatched_blocks=%u snr_db=%.2f receiver_model=1",
                     k56flex_train_phase_name(previous), client->blocks_checked,
                     client->blocks_mismatched, k56flex_rxfe_snr_db(client->fe));
            event(user, (int)n, "Diagnostic", "K56flex receiver model stage", detail);
        }
        if (!param && client->param_ok) {
            param = 1;
            snprintf(detail, sizeof(detail),
                     "source=k56flex_client timestamp=detection checksum=valid record_rate_bps=%d mode=%u extra=%u control=%04x u=%u v=%u final=%u payload_verified=0 report_assumed=8880 report_ext_assumed=00",
                     client->rate_bps, client->param.mode, client->param.extra,
                     client->param.control, client->param.u, client->param.v,
                     client->param.final);
            event(user, (int)n, "K56flex", "Parameter record decoded", detail);
        }
    }
    if (locked) {
        snprintf(detail, sizeof(detail),
                 "source=k56flex_client receiver_model=1 failed=%d param_decoded=%d checked_blocks=%u mismatched_blocks=%u snr_db=%.2f clock_ppm=%.2f payload_verified=0",
                 client->failed, param, client->blocks_checked, client->blocks_mismatched,
                 k56flex_rxfe_snr_db(client->fe), k56flex_rxfe_ppm(client->fe));
        event(user, processed ? (int)processed - 1 : 0, "Diagnostic", "K56flex receiver outcome", detail);
    }
    k56flex_client_free(client);
    return 0;
}
