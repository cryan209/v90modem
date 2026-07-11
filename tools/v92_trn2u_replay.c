/* Replay a raw G.711 capture through the native V.92 TRN2u/CPu receiver. */

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../v92_cp_rx.h"
#include "../v92_trn2u.h"

typedef struct {
    uint64_t sample;
    uint32_t cpt;
    uint32_t cpu;
    uint32_t cpus;
    uint32_t suvu;
} replay_report_t;

static void usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s <input.g711> [--law ulaw|alaw] [--points 4|8] "
            "[--lu amplitude] [--start sample] [--max samples]\n",
            argv0);
}

static bool parse_u64(const char *text, uint64_t *out)
{
    char *end = NULL;
    unsigned long long value;

    errno = 0;
    value = strtoull(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0')
        return false;
    *out = (uint64_t)value;
    return true;
}

static void frame_handler(void *user_data,
                          v92_p4u_kind_t kind,
                          const v92_cp_diag_t *cp,
                          const v92_cpus_diag_t *cpus,
                          const v92_suvu_diag_t *suvu)
{
    replay_report_t *report = (replay_report_t *)user_data;
    double ms = (double)report->sample / 8.0;

    if ((kind == V92_P4U_KIND_CPT || kind == V92_P4U_KIND_CPU) && cp) {
        const char *name = kind == V92_P4U_KIND_CPT ? "CPt" : "CPu";

        if (kind == V92_P4U_KIND_CPT)
            report->cpt++;
        else
            report->cpu++;
        printf("sample=%" PRIu64 " time_ms=%.3f frame=%s bits=%d drn=%u "
               "ack=%d law=%s Sr=%u ld=%u constellations=%u\n",
               report->sample, ms, name, cp->nbits,
               (unsigned)cp->frame.drn,
               cp->frame.acknowledge ? 1 : 0,
               cp->frame.codec_alaw ? "alaw" : "ulaw",
               (unsigned)cp->frame.shaping_redundancy,
               (unsigned)cp->frame.shaping_lookahead,
               (unsigned)cp->frame.constellation_count);
    } else if (kind == V92_P4U_KIND_CPUS && cpus) {
        report->cpus++;
        printf("sample=%" PRIu64 " time_ms=%.3f frame=CPus drn=%u ack=%d\n",
               report->sample, ms, (unsigned)cpus->frame.drn,
               cpus->frame.acknowledge ? 1 : 0);
    } else if (kind == V92_P4U_KIND_SUVU && suvu) {
        report->suvu++;
        printf("sample=%" PRIu64 " time_ms=%.3f frame=SUVu wait_cpu=%d "
               "prefilter_q2_2=%u silent=%d ack=%d\n",
               report->sample, ms,
               suvu->frame.wait_for_cpu ? 1 : 0,
               (unsigned)suvu->frame.prefilter_level_q2_2,
               suvu->frame.silent_period_requested ? 1 : 0,
               suvu->frame.acknowledge ? 1 : 0);
    }
}

int main(int argc, char **argv)
{
    const char *path;
    bool alaw = false;
    int points = 4;
    double lu = 8000.0;
    uint64_t start = 0;
    uint64_t max_samples = 0;
    uint64_t fed = 0;
    uint8_t buffer[4096];
    FILE *fp;
    v92_cp_rx_t rx;
    v92_trn2u_demod_t demod;
    replay_report_t report = {0};

    if (argc < 2) {
        usage(argv[0]);
        return 2;
    }
    path = argv[1];
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--law") == 0 && i + 1 < argc) {
            const char *law = argv[++i];
            if (strcmp(law, "alaw") == 0)
                alaw = true;
            else if (strcmp(law, "ulaw") != 0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--points") == 0 && i + 1 < argc) {
            points = atoi(argv[++i]);
            if (points != 4 && points != 8) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--lu") == 0 && i + 1 < argc) {
            char *end = NULL;
            lu = strtod(argv[++i], &end);
            if (!end || *end != '\0' || lu <= 0.0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--start") == 0 && i + 1 < argc) {
            if (!parse_u64(argv[++i], &start)) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--max") == 0 && i + 1 < argc) {
            if (!parse_u64(argv[++i], &max_samples)) {
                usage(argv[0]);
                return 2;
            }
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    fp = fopen(path, "rb");
    if (!fp) {
        perror(path);
        return 1;
    }
    if (start > 0 && fseek(fp, (long)start, SEEK_SET) != 0) {
        perror("fseek");
        fclose(fp);
        return 1;
    }

    report.sample = start;
    v92_cp_rx_init(&rx, points, alaw, frame_handler, &report);
    v92_trn2u_demod_init(&demod, points, lu, alaw, &rx);
    while (!max_samples || fed < max_samples) {
        size_t want = sizeof(buffer);
        size_t got;

        if (max_samples && max_samples - fed < want)
            want = (size_t)(max_samples - fed);
        got = fread(buffer, 1, want, fp);
        if (got == 0)
            break;
        for (size_t i = 0; i < got; i++) {
            report.sample = start + fed + i;
            (void)v92_trn2u_demod_feed(&demod, &buffer[i], 1);
        }
        fed += got;
    }
    fclose(fp);

    printf("summary file=%s law=%s points=%d lu=%.3f start=%" PRIu64
           " samples=%" PRIu64 " symbols=%u input_bits=%u valid=%u rejected=%u "
           "CPt=%u CPu=%u CPus=%u SUVu=%u\n",
           path, alaw ? "alaw" : "ulaw", points, lu, start, fed,
           demod.symbols, rx.input_bits, rx.valid_frames, rx.rejected_frames,
           report.cpt, report.cpu, report.cpus, report.suvu);
    return 0;
}
