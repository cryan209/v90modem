#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "../v92_p3_rx.h"

static void usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s <input.g711> [--start <sample_index>] [--arm <sample_index>] [--max <samples>]\n",
            argv0);
}

int main(int argc, char **argv)
{
    const char *path = NULL;
    int start_sample = 0;
    int arm_sample = -1;
    int max_samples = -1;
    FILE *fp = NULL;
    v92_p3_rx_t rx;
    int c;
    int i = 0;
    v92_p3_rx_state_t last_state;
    int last_reject_count = 0;

    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }
    path = argv[1];

    for (int a = 2; a < argc; a++) {
        if (strcmp(argv[a], "--start") == 0 && (a + 1) < argc) {
            start_sample = atoi(argv[++a]);
            continue;
        }
        if (strcmp(argv[a], "--arm") == 0 && (a + 1) < argc) {
            arm_sample = atoi(argv[++a]);
            continue;
        }
        if (strcmp(argv[a], "--max") == 0 && (a + 1) < argc) {
            max_samples = atoi(argv[++a]);
            continue;
        }
        usage(argv[0]);
        return 2;
    }
    if (arm_sample < 0)
        arm_sample = start_sample;

    fp = fopen(path, "rb");
    if (!fp) {
        perror(path);
        return 1;
    }

    if (start_sample > 0) {
        if (fseek(fp, (long) start_sample, SEEK_SET) != 0) {
            /* Fallback for non-seekable streams: discard bytes manually. */
            int dropped = 0;
            while (dropped < start_sample && (c = fgetc(fp)) != EOF)
                dropped++;
            if (dropped < start_sample) {
                fprintf(stderr, "short input while skipping to --start=%d\n", start_sample);
                fclose(fp);
                return 1;
            }
        }
    }

    v92_p3_rx_init(&rx);
    v92_p3_rx_start(&rx, arm_sample);
    last_state = v92_p3_rx_get_state(&rx);
    printf("start_sample=%d arm_sample=%d state=%s\n",
           start_sample,
           arm_sample,
           v92_p3_rx_state_name(last_state));

    while ((c = fgetc(fp)) != EOF) {
        int sample_index = start_sample + i;
        bool changed = v92_p3_rx_feed(&rx, (uint8_t) c, sample_index);

        if (rx.reject_count != last_reject_count) {
            int rej_sample = -1;
            int m0 = 0;
            int m1 = 0;
            v92_p3_rx_reject_t rej = v92_p3_rx_last_reject(&rx, &rej_sample, &m0, &m1);
            printf("sample=%d reject=%s at=%d m0=%d m1=%d total=%d\n",
                   sample_index,
                   v92_p3_rx_reject_name(rej),
                   rej_sample,
                   m0,
                   m1,
                   rx.reject_count);
            last_reject_count = rx.reject_count;
        }

        if (changed) {
            v92_p3_rx_state_t s = v92_p3_rx_get_state(&rx);
            printf("sample=%d state=%s\n", sample_index, v92_p3_rx_state_name(s));
            last_state = s;
            if (s == V92_P3_RX_DONE || s == V92_P3_RX_FAILED)
                break;
        }

        i++;
        if (max_samples > 0 && i >= max_samples)
            break;
    }

    fclose(fp);

    {
        v92_p3_rx_state_t s = v92_p3_rx_get_state(&rx);
        printf("final_state=%s processed=%d\n", v92_p3_rx_state_name(s), i);
        if (v92_p3_rx_ja_ok(&rx)) {
            const ja_dil_decode_t *ja = v92_p3_rx_get_ja(&rx);
            printf("ja_ok=1 soft_lock=%d n=%u lsp=%u ltp=%u bit_offset=%d score=%d\n",
                   ja->soft_lock ? 1 : 0,
                   (unsigned) ja->desc.n,
                   (unsigned) ja->desc.lsp,
                   (unsigned) ja->desc.ltp,
                   ja->start_sample,
                   ja->soft_score);
            printf("ja_bits=%d parsed_v92=%d\n",
                   ja->descriptor_bits,
                   ja->parsed_v92 ? 1 : 0);
        } else {
            printf("ja_ok=0\n");
        }
        {
            int rej_sample = -1;
            int m0 = 0;
            int m1 = 0;
            v92_p3_rx_reject_t rej = v92_p3_rx_last_reject(&rx, &rej_sample, &m0, &m1);
            printf("reject_count=%d last_reject=%s sample=%d m0=%d m1=%d\n",
                   rx.reject_count,
                   v92_p3_rx_reject_name(rej),
                   rej_sample,
                   m0,
                   m1);
        }
    }

    return 0;
}
