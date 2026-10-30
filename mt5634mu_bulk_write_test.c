/* USB 2.0 8.6 / 9.4.5: keep the data toggle and retry a single packet,
 * not a multi-packet prefix whose accepted length WritePipeTO cannot report. */
#include "tools/mt5634mu_bulk_write.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(x) do { if (!(x)) { ++failures; fprintf(stderr, "FAIL %d: %s\n", __LINE__, #x); } } while (0)

struct writer {
    uint8_t *out;
    size_t used, capacity, max_seen;
    unsigned calls, retries, retry_left;
    const uint8_t *pending;
    size_t pending_len;
    int hard_error, cancel_on_retry;
    volatile sig_atomic_t quit;
};

static int packet(void *ctx, const uint8_t *data, size_t n)
{
    struct writer *w = ctx;
    ++w->calls;
    if (n > w->max_seen) w->max_seen = n;
    /* A 64-byte request could accept 48 then time out, exactly the live
     * corruption. Such requests are forbidden before reaching the USB API. */
    CHECK(n > 0 && n <= 16);
    if (n > 16 || w->hard_error) return MT5634MU_WRITE_ERROR;
    if (w->pending) {
        CHECK(data == w->pending && n == w->pending_len);
    } else if ((w->used % 4096) == 0) {
        w->retry_left = 3;
        w->pending = data;
        w->pending_len = n;
    }
    if (w->retry_left) {
        --w->retry_left;
        ++w->retries;
        if (w->cancel_on_retry) w->quit = 1;
        return MT5634MU_WRITE_RETRY;
    }
    CHECK(w->used + n <= w->capacity);
    if (w->used + n > w->capacity) return MT5634MU_WRITE_ERROR;
    memcpy(w->out + w->used, data, n);
    w->used += n;
    w->pending = NULL;
    return MT5634MU_WRITE_OK;
}

/* Model USB 2.0 8.6's lost-ACK case: the first packet was accepted,
 * but its same-toggle retry is acknowledged without delivering it again. */
static int lost_ack(void *ctx, const uint8_t *data, size_t n)
{
    struct writer *w = ctx;
    ++w->calls;
    if (w->calls == 1) {
        memcpy(w->out, data, n);
        w->used = n;
        w->pending = data;
        w->pending_len = n;
        return MT5634MU_WRITE_RETRY;
    }
    CHECK(w->calls == 2 && data == w->pending && n == w->pending_len);
    return MT5634MU_WRITE_OK;
}

static void transfer(unsigned endpoint_size, size_t count)
{
    uint8_t *input = malloc(count ? count : 1);
    uint8_t *output = malloc(count ? count : 1);
    struct writer w = {0};
    unsigned rng = 0x5634;
    CHECK(input && output);
    if (!input || !output) exit(2);
    for (size_t i = 0; i < count; ++i) {
        rng = rng*1664525u + 1013904223u;
        input[i] = (uint8_t)(rng >> 24);
    }
    w.out = output;
    w.capacity = count;
    CHECK(mt5634mu_write_packets(input, count, endpoint_size, packet, &w, &w.quit) == 0);
    CHECK(w.used == count);
    CHECK(!memcmp(input, output, count));
    CHECK(w.max_seen <= (endpoint_size < 16 ? endpoint_size : 16));
    if (count) CHECK(w.retries >= 3);
    else CHECK(w.calls == 0);
    free(input);
    free(output);
}

int main(void)
{
    /* 256 KiB plus a non-packet-aligned tail; repeated backpressure must
     * neither duplicate an accepted prefix nor advance on a retry. */
    transfer(16, 262144);
    transfer(64, 262147); /* even an oversized descriptor cannot lift the cap */
    transfer(8, 777);
    transfer(16, 3);     /* guarded +++ and other short AT writes */
    transfer(16, 0);
    {
        uint8_t input[16] = {0}, output[16];
        struct writer w = {.out=output, .capacity=sizeof(output), .cancel_on_retry=1};
        CHECK(mt5634mu_write_packets(input, sizeof(input), 16, packet, &w, &w.quit) == -1);
        CHECK(w.calls == 1 && w.used == 0 && w.quit);
        memset(&w, 0, sizeof(w));
        w.hard_error = 1;
        CHECK(mt5634mu_write_packets(input, sizeof(input), 16, packet, &w, &w.quit) == -1);
        CHECK(w.calls == 1 && w.used == 0);
        CHECK(mt5634mu_write_packets(input, sizeof(input), 0, packet, &w, NULL) == -1);
        CHECK(mt5634mu_write_packets(NULL, 1, 16, packet, &w, NULL) == -1);
        CHECK(mt5634mu_write_packets(input, 1, 16, NULL, &w, NULL) == -1);
        CHECK(w.calls == 1);
        memset(&w, 0, sizeof(w));
        w.out = output;
        CHECK(mt5634mu_write_packets(input, sizeof(input), 16, lost_ack, &w, NULL) == 0);
        CHECK(w.calls == 2 && w.used == sizeof(input));
        CHECK(!memcmp(input, output, sizeof(input)));
    }
    printf("mt5634mu_bulk_write_test: %s\n", failures ? "FAILED" : "all passed");
    return failures != 0;
}
