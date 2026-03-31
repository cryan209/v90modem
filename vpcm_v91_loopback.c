#include "vpcm_v91_loopback.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { VPCM_V91_LOOPBACK_NOMINAL_10S_FRAMES = 13328 };

static int vpcm_v91_loopback_frames_from_seconds(int seconds)
{
    long long frames;

    if (seconds <= 0)
        return 0;
    frames = ((long long) seconds * 8000LL) / (long long) VPCM_CP_FRAME_INTERVALS;
    if (frames < 1)
        frames = 1;
    frames -= (frames % 4LL);
    if (frames < 4)
        frames = 4;
    if (frames > 2000000000LL)
        frames = 2000000000LL;
    return (int) frames;
}

static void vpcm_v91_loopback_fill_pattern(uint8_t *buf, int len, uint32_t seed)
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

static bool vpcm_v91_loopback_expect_equal(const char *label,
                                           const uint8_t *expected,
                                           const uint8_t *actual,
                                           int len)
{
    int i;

    if (!expected || !actual || len < 0)
        return false;
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

void vpcm_v91_loopback_cleanup(vpcm_v91_loopback_run_t *run)
{
    if (!run)
        return;
    free(run->caller_data_in);
    free(run->caller_data_out);
    free(run->answerer_data_in);
    free(run->answerer_data_out);
    free(run->caller_pcm_tx);
    free(run->caller_pcm_rx);
    free(run->answerer_pcm_tx);
    free(run->answerer_pcm_rx);
    memset(run, 0, sizeof(*run));
}

bool vpcm_v91_loopback_run_data(vpcm_v91_loopback_run_t *run,
                                vpcm_call_pair_t *pair,
                                vpcm_v91_session_t *session,
                                v91_state_t *caller_tx,
                                v91_state_t *caller_rx,
                                v91_state_t *answerer_tx,
                                v91_state_t *answerer_rx,
                                const vpcm_cp_frame_t *cp_ack,
                                uint32_t data_seed,
                                int data_seconds,
                                bool robbed_bit,
                                vpcm_call_pair_v91_chunk_logger_fn chunk_logger,
                                void *chunk_logger_user_data)
{
    if (!run || !pair || !session || !caller_tx || !caller_rx || !answerer_tx || !answerer_rx || !cp_ack)
        return false;

    memset(run, 0, sizeof(*run));
    run->data_frames = vpcm_v91_loopback_frames_from_seconds(data_seconds);
    if (run->data_frames <= 0)
        run->data_frames = VPCM_V91_LOOPBACK_NOMINAL_10S_FRAMES;

    run->total_bits = run->data_frames * ((int) cp_ack->drn + 20);
    run->total_bytes = run->total_bits / 8;
    run->total_codewords = run->data_frames * VPCM_CP_FRAME_INTERVALS;
    run->caller_data_in = (uint8_t *) malloc((size_t) run->total_bytes);
    run->caller_data_out = (uint8_t *) malloc((size_t) run->total_bytes);
    run->answerer_data_in = (uint8_t *) malloc((size_t) run->total_bytes);
    run->answerer_data_out = (uint8_t *) malloc((size_t) run->total_bytes);
    run->caller_pcm_tx = (uint8_t *) malloc((size_t) run->total_codewords);
    run->caller_pcm_rx = (uint8_t *) malloc((size_t) run->total_codewords);
    run->answerer_pcm_tx = (uint8_t *) malloc((size_t) run->total_codewords);
    run->answerer_pcm_rx = (uint8_t *) malloc((size_t) run->total_codewords);
    if (!run->caller_data_in || !run->caller_data_out || !run->answerer_data_in || !run->answerer_data_out
        || !run->caller_pcm_tx || !run->caller_pcm_rx || !run->answerer_pcm_tx || !run->answerer_pcm_rx) {
        vpcm_v91_loopback_cleanup(run);
        return false;
    }

    vpcm_v91_loopback_fill_pattern(run->caller_data_in, run->total_bytes, data_seed);
    vpcm_v91_loopback_fill_pattern(run->answerer_data_in, run->total_bytes, data_seed ^ 0xA55AA55AU);
    memset(run->caller_data_out, 0, (size_t) run->total_bytes);
    memset(run->answerer_data_out, 0, (size_t) run->total_bytes);

    run->codewords_per_report = vpcm_v91_loopback_frames_from_seconds(1) * VPCM_CP_FRAME_INTERVALS;
    if (run->codewords_per_report < VPCM_CP_FRAME_INTERVALS)
        run->codewords_per_report = VPCM_CP_FRAME_INTERVALS;

    if (!vpcm_call_pair_run_v91_data(pair,
                                     session,
                                     caller_tx,
                                     caller_rx,
                                     answerer_tx,
                                     answerer_rx,
                                     cp_ack,
                                     run->caller_data_in,
                                     run->caller_data_out,
                                     run->answerer_data_in,
                                     run->answerer_data_out,
                                     run->caller_pcm_tx,
                                     run->caller_pcm_rx,
                                     run->answerer_pcm_tx,
                                     run->answerer_pcm_rx,
                                     run->total_codewords,
                                     run->codewords_per_report,
                                     robbed_bit,
                                     chunk_logger,
                                     chunk_logger_user_data,
                                     &run->data_report)) {
        vpcm_v91_loopback_cleanup(run);
        return false;
    }

    run->sample_pcm_len = (run->total_codewords < 6) ? run->total_codewords : 6;
    if (cp_ack->transparent_mode_granted) {
        run->sample_data_len = run->sample_pcm_len;
    } else {
        run->sample_data_len = (run->sample_pcm_len * ((int) cp_ack->drn + 20)) / 48;
    }
    if (run->sample_data_len < 1)
        run->sample_data_len = 1;
    if (run->sample_data_len > run->total_bytes)
        run->sample_data_len = run->total_bytes;
    return true;
}

bool vpcm_v91_loopback_verify(const char *label,
                              const vpcm_v91_loopback_run_t *run)
{
    char reverse_label[128];
    const char *base;

    if (!run)
        return false;
    base = (label && *label) ? label : "V.91 loopback";
    if (!vpcm_v91_loopback_expect_equal(base,
                                        run->caller_data_in,
                                        run->answerer_data_out,
                                        run->total_bytes)) {
        return false;
    }
    snprintf(reverse_label, sizeof(reverse_label), "%s reverse", base);
    return vpcm_v91_loopback_expect_equal(reverse_label,
                                          run->answerer_data_in,
                                          run->caller_data_out,
                                          run->total_bytes);
}
