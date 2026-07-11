/* Replay a raw G.711 capture through the native V.92 TRN2u/CPu receiver. */

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

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
            "[--lu amplitude] [--start sample] [--max samples] "
            "[--analog-wav --channel L|R --phase 0..1 --clock-ppm ppm "
            "--gain scale --timing-loop --timing-mu gain "
            "--equalizer --eq-mu gain]\n",
            argv0);
}

typedef struct {
    int16_t *samples;
    uint64_t frames;
    int channels;
    int sample_rate;
} wav_pcm_t;

static uint16_t get_le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool load_pcm16_wav(const char *path, wav_pcm_t *wav)
{
    uint8_t header[12];
    FILE *fp = fopen(path, "rb");
    bool have_fmt = false;
    uint32_t data_bytes = 0;

    memset(wav, 0, sizeof(*wav));
    if (!fp)
        return false;
    if (fread(header, 1, sizeof(header), fp) != sizeof(header)
        || memcmp(header, "RIFF", 4) != 0
        || memcmp(header + 8, "WAVE", 4) != 0)
        goto fail;
    for (;;) {
        uint8_t chunk[8];
        uint32_t size;

        if (fread(chunk, 1, sizeof(chunk), fp) != sizeof(chunk))
            break;
        size = get_le32(chunk + 4);
        if (memcmp(chunk, "fmt ", 4) == 0) {
            uint8_t fmt[40];
            size_t take = size < sizeof(fmt) ? size : sizeof(fmt);

            if (take < 16 || fread(fmt, 1, take, fp) != take)
                goto fail;
            if (size > take && fseek(fp, (long)(size - take), SEEK_CUR) != 0)
                goto fail;
            if (get_le16(fmt) != 1 || get_le16(fmt + 14) != 16)
                goto fail;
            wav->channels = (int)get_le16(fmt + 2);
            wav->sample_rate = (int)get_le32(fmt + 4);
            have_fmt = true;
        } else if (memcmp(chunk, "data", 4) == 0) {
            if (!have_fmt || wav->channels < 1)
                goto fail;
            data_bytes = size;
            wav->samples = malloc((size_t)data_bytes);
            if (!wav->samples || fread(wav->samples, 1, data_bytes, fp) != data_bytes)
                goto fail;
            wav->frames = data_bytes / (2U * (uint32_t)wav->channels);
            fclose(fp);
            return wav->sample_rate == 8000 && wav->frames > 1;
        } else if (fseek(fp, (long)size, SEEK_CUR) != 0) {
            goto fail;
        }
        if ((size & 1U) && fseek(fp, 1, SEEK_CUR) != 0)
            goto fail;
    }
fail:
    free(wav->samples);
    memset(wav, 0, sizeof(*wav));
    fclose(fp);
    return false;
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
    double timing_phase = 0.0;
    double clock_ppm = 0.0;
    double gain = 1.0;
    bool analog_wav = false;
    bool equalizer = false;
    double eq_mu = 0.01;
    bool timing_loop = false;
    double timing_mu = 0.002;
    int wav_channel = 0;
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
        } else if (strcmp(argv[i], "--analog-wav") == 0) {
            analog_wav = true;
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            const char *channel = argv[++i];
            if (strcmp(channel, "L") == 0 || strcmp(channel, "l") == 0)
                wav_channel = 0;
            else if (strcmp(channel, "R") == 0 || strcmp(channel, "r") == 0)
                wav_channel = 1;
            else {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--phase") == 0 && i + 1 < argc) {
            char *end = NULL;
            timing_phase = strtod(argv[++i], &end);
            if (!end || *end != '\0' || timing_phase < 0.0 || timing_phase >= 1.0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--clock-ppm") == 0 && i + 1 < argc) {
            char *end = NULL;
            clock_ppm = strtod(argv[++i], &end);
            if (!end || *end != '\0' || clock_ppm < -10000.0 || clock_ppm > 10000.0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--gain") == 0 && i + 1 < argc) {
            char *end = NULL;
            gain = strtod(argv[++i], &end);
            if (!end || *end != '\0' || gain <= 0.0 || gain > 100.0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--equalizer") == 0) {
            equalizer = true;
        } else if (strcmp(argv[i], "--eq-mu") == 0 && i + 1 < argc) {
            char *end = NULL;
            eq_mu = strtod(argv[++i], &end);
            if (!end || *end != '\0' || eq_mu <= 0.0 || eq_mu > 1.0) {
                usage(argv[0]);
                return 2;
            }
        } else if (strcmp(argv[i], "--timing-loop") == 0) {
            timing_loop = true;
        } else if (strcmp(argv[i], "--timing-mu") == 0 && i + 1 < argc) {
            char *end = NULL;
            timing_mu = strtod(argv[++i], &end);
            if (!end || *end != '\0' || timing_mu <= 0.0 || timing_mu > 0.25) {
                usage(argv[0]);
                return 2;
            }
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (analog_wav) {
        wav_pcm_t wav;
        double position;
        double step = 1.0 + clock_ppm * 1.0e-6;
        double eq_taps[5] = {1.0, 0.0, 0.0, 0.0, 0.0};
        double eq_hist[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
        double timing_prev_value = 0.0;
        double timing_prev_decision = 0.0;
        bool timing_prev_valid = false;

        if (!load_pcm16_wav(path, &wav)) {
            fprintf(stderr, "%s: expected 8 kHz, 16-bit PCM WAV\n", path);
            return 1;
        }
        if (wav_channel >= wav.channels || start >= wav.frames) {
            fprintf(stderr, "%s: channel/start outside WAV data\n", path);
            free(wav.samples);
            return 1;
        }
        report.sample = start;
        v92_cp_rx_init(&rx, points, alaw, frame_handler, &report);
        v92_trn2u_demod_init(&demod, points, lu, alaw, &rx);
        position = (double)start + timing_phase;
        while (position + 1.0 < (double)wav.frames
               && (!max_samples || fed < max_samples)) {
            uint64_t base = (uint64_t)position;
            double frac = position - (double)base;
            double a = wav.samples[base * (uint64_t)wav.channels + (uint64_t)wav_channel];
            double b = wav.samples[(base + 1) * (uint64_t)wav.channels + (uint64_t)wav_channel];
            double value = gain * (a + frac * (b - a));
            int sample;
            uint8_t codeword;
            double timing_adjust = 0.0;

            if (equalizer) {
                double y = 0.0;
                double energy = 1.0;
                double scale = sqrt(points == 4 ? 5.0 : 21.0);
                int labels = points / 2;
                int best_label = 0;
                double best_error = 1.0e30;
                double target;
                double error;

                memmove(eq_hist + 1, eq_hist, 4 * sizeof(eq_hist[0]));
                eq_hist[0] = value;
                for (int k = 0; k < 5; k++) {
                    y += eq_taps[k] * eq_hist[k];
                    energy += eq_hist[k] * eq_hist[k];
                }
                for (int label = 0; label < labels; label++) {
                    double level = ((double)(2 * label + 1) / scale) * lu;
                    double distance = fabs(fabs(y) - level);

                    if (distance < best_error) {
                        best_error = distance;
                        best_label = label;
                    }
                }
                target = ((double)(2 * best_label + 1) / scale) * lu;
                if (y < 0.0)
                    target = -target;
                error = target - y;
                for (int k = 0; k < 5; k++)
                    eq_taps[k] += eq_mu * error * eq_hist[k] / energy;
                value = y;
            }
            if (timing_loop) {
                double scale = sqrt(points == 4 ? 5.0 : 21.0);
                int labels = points / 2;
                int best_label = 0;
                double best_error = 1.0e30;
                double decision;

                for (int label = 0; label < labels; label++) {
                    double level = ((double)(2 * label + 1) / scale) * lu;
                    double distance = fabs(fabs(value) - level);

                    if (distance < best_error) {
                        best_error = distance;
                        best_label = label;
                    }
                }
                decision = ((double)(2 * best_label + 1) / scale) * lu;
                if (value < 0.0)
                    decision = -decision;
                if (timing_prev_valid) {
                    double error = (timing_prev_decision * value
                                    - decision * timing_prev_value)
                                   / (lu * lu + 1.0);

                    if (error > 1.0)
                        error = 1.0;
                    if (error < -1.0)
                        error = -1.0;
                    timing_adjust = timing_mu * error;
                }
                timing_prev_value = value;
                timing_prev_decision = decision;
                timing_prev_valid = true;
            }
            if (value > 32767.0)
                value = 32767.0;
            if (value < -32768.0)
                value = -32768.0;
            sample = (int)(value >= 0.0 ? value + 0.5 : value - 0.5);
            codeword = alaw ? linear_to_alaw(sample) : linear_to_ulaw(sample);
            report.sample = base;
            (void)v92_trn2u_demod_feed(&demod, &codeword, 1);
            position += step + timing_adjust;
            fed++;
        }
        free(wav.samples);
        printf("summary file=%s input=analog-wav channel=%c law=%s points=%d "
               "lu=%.3f phase=%.6f clock_ppm=%.3f gain=%.3f timing=%d "
               "timing_mu=%.5f eq=%d eq_mu=%.5f "
               "start=%" PRIu64
               " samples=%" PRIu64 " symbols=%u input_bits=%u valid=%u rejected=%u "
               "longest_ones=%u CPt=%u CPu=%u CPus=%u SUVu=%u\n",
               path, wav_channel ? 'R' : 'L', alaw ? "alaw" : "ulaw",
               points, lu, timing_phase, clock_ppm, gain,
               timing_loop ? 1 : 0, timing_mu, equalizer ? 1 : 0,
               eq_mu, start, fed,
               demod.symbols, rx.input_bits, rx.valid_frames, rx.rejected_frames,
               demod.longest_descrambled_one_run,
               report.cpt, report.cpu, report.cpus, report.suvu);
        return 0;
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
           "longest_ones=%u CPt=%u CPu=%u CPus=%u SUVu=%u\n",
           path, alaw ? "alaw" : "ulaw", points, lu, start, fed,
           demod.symbols, rx.input_bits, rx.valid_frames, rx.rejected_frames,
           demod.longest_descrambled_one_run,
           report.cpt, report.cpu, report.cpus, report.suvu);
    return 0;
}
