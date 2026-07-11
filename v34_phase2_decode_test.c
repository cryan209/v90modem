#include "v34_phase2_decode.h"

#include <stdio.h>
#include <string.h>

enum {
    TEST_SAMPLE_RATE = 8000,
    TEST_TOTAL_SAMPLES = 2 * TEST_SAMPLE_RATE,
    TEST_INFO1_MIN_SAMPLES = 12000
};

typedef struct {
    int calls;
    int longest_pass;
} mock_decode_ctx_t;

static bool mock_phase2_pass(void *user_data,
                             const int16_t *samples,
                             int total_samples,
                             v91_law_t law,
                             bool calling_party,
                             float info_db_cutoff,
                             bool allow_info_rate_infer,
                             decode_v34_result_t *result)
{
    mock_decode_ctx_t *ctx = (mock_decode_ctx_t *) user_data;

    (void) samples;
    (void) law;
    (void) calling_party;
    (void) info_db_cutoff;
    (void) allow_info_rate_infer;
    if (!ctx || !result)
        return false;

    ctx->calls++;
    if (total_samples > ctx->longest_pass)
        ctx->longest_pass = total_samples;

    memset(result, 0, sizeof(*result));
    result->info0_seen = true;
    result->info0_sample = 1000;
    result->info1_sample = -1;
    if (total_samples >= TEST_INFO1_MIN_SAMPLES) {
        result->info1_seen = true;
        result->info1_sample = 10000;
    }
    return true;
}

static int test_partial_info_window_is_extended(void)
{
    int16_t samples[TEST_TOTAL_SAMPLES];
    v34_phase2_engine_t engine;
    mock_decode_ctx_t ctx;
    decode_v34_result_t answerer;
    decode_v34_result_t caller;
    bool have_answerer;
    bool have_caller;

    memset(samples, 0, sizeof(samples));
    /* The normal energy window around this short burst is too short for the
       mock INFO1, so only continuation of the INFO0 pass can complete it. */
    for (int i = 0; i < 3200; i++)
        samples[i] = (i & 1) ? 6000 : -6000;

    memset(&ctx, 0, sizeof(ctx));
    v34_phase2_engine_init(&engine, mock_phase2_pass, &ctx);
    v34_phase2_decode_pair(&engine,
                           samples,
                           TEST_TOTAL_SAMPLES,
                           V91_LAW_ULAW,
                           false,
                           &answerer,
                           &have_answerer,
                           &caller,
                           &have_caller);

    if (!have_answerer || !answerer.info0_seen || !answerer.info1_seen) {
        fprintf(stderr, "FAIL: answerer did not retain the extended INFO0/INFO1 decode\n");
        return 1;
    }
    if (!have_caller || !caller.info0_seen || !caller.info1_seen) {
        fprintf(stderr, "FAIL: caller did not retain the extended INFO0/INFO1 decode\n");
        return 1;
    }
    if (ctx.longest_pass < TEST_INFO1_MIN_SAMPLES) {
        fprintf(stderr, "FAIL: partial INFO0 candidate was not extended\n");
        return 1;
    }
    if (answerer.phase2_selected_window_end_sample <= TEST_SAMPLE_RATE) {
        fprintf(stderr, "FAIL: selected window metadata did not include continuation\n");
        return 1;
    }

    printf("PASS: partial Phase 2 INFO0 window continues through INFO1\n");
    return 0;
}

int main(void)
{
    if (test_partial_info_window_is_extended() != 0)
        return 1;
    printf("v34_phase2_decode_test: OK\n");
    return 0;
}
