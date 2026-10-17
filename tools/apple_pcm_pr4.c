/* Offline, known-reference PR4 limit measurement for apple_pcm_matrix.py.
 * Equalise to 1-D^2 rather than invert the telephone path's DC/Nyquist
 * nulls; see v90_analogue_pr4.h and V.90 5.4.3 for the candidate sets.
 * Oracle training on the first 5000 known symbols is intentionally stronger
 * than live startup. Only later, unseen symbols count towards error rates. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v90_analogue_pr4.h"

static int16_t levels[256];
static double candidate_levels[256];
static uint8_t candidate_codes[256];
static int candidate_count;

static int candidates(void *user, int64_t index, double *v, uint8_t *cw, int max)
{
    (void)user; (void)index;
    int n = candidate_count < max ? candidate_count : max;
    memcpy(v, candidate_levels, n*sizeof *v);
    memcpy(cw, candidate_codes, n*sizeof *cw);
    return n;
}

static void *read_file(const char *path, long *bytes)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END)) { fclose(f); return NULL; }
    *bytes = ftell(f);
    rewind(f);
    if (*bytes <= 0) { fclose(f); return NULL; }
    void *p = malloc((size_t)*bytes);
    if (!p || fread(p, 1, (size_t)*bytes, f) != (size_t)*bytes) {
        free(p); fclose(f); return NULL;
    }
    fclose(f);
    return p;
}

int main(int argc, char **argv)
{
    if (argc != 4 && argc != 5) { fprintf(stderr, "usage: %s rx-16k.s16 reference.codes levels-256.s16 [traceback]\n", argv[0]); return 2; }
    int depth = 24;
    if (argc == 5) {
        char *end;
        long value = strtol(argv[4], &end, 10);
        if (!*argv[4] || *end || value < 1 || value > 256) return 2;
        depth = (int)value;
    }
    long nb = 0, nc = 0, nl = 0;
    int16_t *rx = read_file(argv[1], &nb), *lv = read_file(argv[3], &nl);
    uint8_t *ref = read_file(argv[2], &nc);
    if (!rx || !ref || !lv || nl != sizeof levels || nb % 2 || nc < 10000 || nb/4 < nc) {
        fprintf(stderr, "invalid or short input\n"); free(rx); free(ref); free(lv); return 2;
    }
    for (int i = 0; i < 256; i++) levels[i] = lv[i];
    int used[256] = {0};
    for (long i = 0; i < nc; i++) used[ref[i]] = 1;
    for (int c = 0; c < 256; c++) {
        if (!used[c]) continue;
        int duplicate = 0;
        for (int k = 0; k < c; k++) if (used[k] && levels[k] == levels[c]) duplicate = 1;
        if (duplicate) continue;
        candidate_levels[candidate_count] = levels[c]/3000.0;
        candidate_codes[candidate_count++] = (uint8_t)c;
    }
    free(lv);
    v90a_pr4_t *pr = v90a_pr4_init(128, 24, depth);
    if (!pr || !v90a_pr4_train(pr, 5000)) return 2;
    long got = 0, errors = 0, counts[256] = {0}, wrong[256] = {0};
    enum { START = 8000 };
    for (long i = 0; i < nc; i++) {
        v90a_pr4_sample(pr, rx[2*i]);
        v90a_pr4_sample(pr, rx[2*i+1]);
        if (i == START && !v90a_pr4_start(pr, START, candidates, NULL)) return 2;
        v90a_pr4_symbol(pr, levels[ref[i]]/3000.0);
        uint8_t out[64];
        int n = v90a_pr4_take(pr, out, sizeof out);
        for (int j = 0; j < n; j++, got++) {
            uint8_t truth = ref[START+got];
            counts[truth]++;
            if (levels[out[j]] != levels[truth]) { wrong[truth]++; errors++; }
        }
    }
    printf("{\"training_fit_db\":%.6f,\"decision_delay\":%d,\"traceback\":%d,\"candidate_levels\":%d,\"symbols\":%ld,\"errors\":%ld,\"levels\":[",
        v90a_pr4_train_snr_db(pr), v90a_pr4_delay(pr), depth, candidate_count, got, errors);
    for (int c = 0; c < 256; c++)
        printf("%s{\"code\":%d,\"tx_level\":%d,\"count\":%ld,\"errors\":%ld}",
            c ? "," : "", c, levels[c], counts[c], wrong[c]);
    printf("]}\n");
    v90a_pr4_free(pr); free(rx); free(ref);
    return got ? 0 : 1;
}
