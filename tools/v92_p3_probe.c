/*
 * v92_p3_probe - replay a recorded digital-side G.711 receive tap through the
 * V.92 strict Phase-3 receiver (v92_p3_rx.c) and print every state change.
 *
 * The engine's own copy of this receiver consumes the same bytes live, but it
 * recovers from every failure by rehunting rather than by failing, so a live
 * call that acquires Ru, enters TRN1u and then rejects it is indistinguishable
 * in the server log from a call on which no upstream arrived at all.  Feeding
 * VPCM_G711_TAP_DIR's live-rx.g711 back through the same code reproduces the
 * live outcome exactly (verified on artifacts/apple-v92-sip-r4, which fails
 * offline at the same sample and with the same reject as it did live) and
 * makes a hypothesis a one-command experiment instead of a call.
 *
 *   v92_p3_probe <live-rx.g711> <arm-sample> [end-sample]
 *
 * The arm sample is the one the engine printed as "V.92 Phase 3 raw receiver
 * armed at G.711 sample N".  V92_P3_RX_DEBUG=1 adds the receiver's own
 * per-decision trace.
 */
#include "v92_p3_rx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    const char *path;
    long file_len;
    int arm;
    int end;
    int last_state = -1;
    unsigned char *cw;
    FILE *f;
    v92_p3_rx_t rx;
    v92_p3_rx_reject_t reason;
    int reject_sample = -1;
    int metric0 = 0;
    int metric1 = 0;

    if (argc < 3) {
        fprintf(stderr,
                "usage: %s <live-rx.g711> <arm-sample> [end-sample]\n",
                argv[0]);
        return 2;
    }
    path = argv[1];
    arm = atoi(argv[2]);
    f = fopen(path, "rb");
    if (!f) {
        perror(path);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    file_len = ftell(f);
    fseek(f, 0, SEEK_SET);
    cw = malloc((size_t)file_len);
    if (!cw || fread(cw, 1, (size_t)file_len, f) != (size_t)file_len) {
        fprintf(stderr, "%s: short read\n", path);
        fclose(f);
        return 1;
    }
    fclose(f);
    end = (argc > 3) ? atoi(argv[3]) : (int)file_len;
    if (end > (int)file_len)
        end = (int)file_len;
    if (arm < 0 || arm >= end) {
        fprintf(stderr, "arm sample %d outside 0..%d\n", arm, end);
        return 2;
    }

    v92_p3_rx_init(&rx);
    v92_p3_rx_start(&rx, arm);
    for (int i = arm; i < end; i++) {
        int state;

        (void)v92_p3_rx_feed(&rx, cw[i], i);
        state = (int)v92_p3_rx_get_state(&rx);
        if (state != last_state) {
            printf("sample %7d (%8.3fs) state=%s\n",
                   i, i / 8000.0,
                   v92_p3_rx_state_name((v92_p3_rx_state_t)state));
            last_state = state;
        }
        if (state == V92_P3_RX_DONE || state == V92_P3_RX_FAILED)
            break;
    }

    reason = v92_p3_rx_last_reject(&rx, &reject_sample, &metric0, &metric1);
    printf("final state=%s rejects=%d last=%s sample=%d m0=%d m1=%d ja_ok=%d\n",
           v92_p3_rx_state_name(v92_p3_rx_get_state(&rx)),
           rx.reject_count,
           v92_p3_rx_reject_name(reason),
           reject_sample, metric0, metric1,
           v92_p3_rx_ja_ok(&rx) ? 1 : 0);
    printf("best period-6 run seen: %d symbols (hyp %d, start %d, "
           "L_U ok=%d mean=%.1f range=%d std=%.1f)\n",
           rx.hunt_best_run, rx.hunt_best_hyp, rx.hunt_best_start,
           rx.hunt_best_lu_ok,
           rx.hunt_best_mean_x10 / 10.0,
           rx.hunt_best_range,
           rx.hunt_best_std_x10 / 10.0);
    if (v92_p3_rx_ja_ok(&rx)) {
        const ja_dil_decode_t *ja = v92_p3_rx_get_ja(&rx);

        if (ja)
            printf("Ja: ok=%d parsed_v92=%d start_sample=%d bits=%d "
                   "N=%u LSP=%u LTP=%u\n",
                   ja->ok, ja->parsed_v92, ja->start_sample,
                   ja->descriptor_bits,
                   (unsigned)ja->desc.n, (unsigned)ja->desc.lsp,
                   (unsigned)ja->desc.ltp);
    }
    free(cw);
    return 0;
}
