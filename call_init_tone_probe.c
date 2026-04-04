#include "call_init_tone_probe.h"

#include <spandsp.h>
#include <spandsp/modem_connect_tones.h>

#include <string.h>

static void call_init_note_first_sample(int *dst, int sample)
{
    if (!dst || sample < 0)
        return;
    if (*dst < 0 || sample < *dst)
        *dst = sample;
}

bool call_init_collect_tones(const int16_t *samples,
                             int total_samples,
                             int max_sample,
                             call_init_tone_probe_t *out)
{
    modem_connect_tones_rx_state_t *ans_rx;
    modem_connect_tones_rx_state_t *ct_rx;
    modem_connect_tones_rx_state_t *cng_rx;
    int limit;
    int offset = 0;

    if (!samples || total_samples <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    out->ans_sample = -1;
    out->ans_tone = MODEM_CONNECT_TONES_NONE;
    out->ct_sample = -1;
    out->cng_sample = -1;

    ans_rx = modem_connect_tones_rx_init(NULL, MODEM_CONNECT_TONES_ANS_PR, NULL, NULL);
    if (!ans_rx)
        return false;
    ct_rx = modem_connect_tones_rx_init(NULL, MODEM_CONNECT_TONES_CALLING_TONE, NULL, NULL);
    if (!ct_rx) {
        modem_connect_tones_rx_free(ans_rx);
        return false;
    }
    cng_rx = modem_connect_tones_rx_init(NULL, MODEM_CONNECT_TONES_FAX_CNG, NULL, NULL);
    if (!cng_rx) {
        modem_connect_tones_rx_free(ct_rx);
        modem_connect_tones_rx_free(ans_rx);
        return false;
    }

    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;

    while (offset < limit) {
        int chunk = limit - offset;
        int tone;

        if (chunk > 160)
            chunk = 160;

        modem_connect_tones_rx(ans_rx, samples + offset, chunk);
        tone = modem_connect_tones_rx_get(ans_rx);
        if (tone == MODEM_CONNECT_TONES_ANSAM
            || tone == MODEM_CONNECT_TONES_ANSAM_PR
            || tone == MODEM_CONNECT_TONES_ANS
            || tone == MODEM_CONNECT_TONES_ANS_PR) {
            call_init_note_first_sample(&out->ans_sample, offset + chunk);
            if (out->ans_tone == MODEM_CONNECT_TONES_NONE)
                out->ans_tone = tone;
        }

        modem_connect_tones_rx(ct_rx, samples + offset, chunk);
        if (modem_connect_tones_rx_get(ct_rx) == MODEM_CONNECT_TONES_CALLING_TONE)
            call_init_note_first_sample(&out->ct_sample, offset + chunk);

        modem_connect_tones_rx(cng_rx, samples + offset, chunk);
        if (modem_connect_tones_rx_get(cng_rx) == MODEM_CONNECT_TONES_FAX_CNG)
            call_init_note_first_sample(&out->cng_sample, offset + chunk);

        offset += chunk;
    }

    modem_connect_tones_rx_free(cng_rx);
    modem_connect_tones_rx_free(ct_rx);
    modem_connect_tones_rx_free(ans_rx);

    out->ok = (out->ans_sample >= 0 || out->ct_sample >= 0 || out->cng_sample >= 0);
    return out->ok;
}
