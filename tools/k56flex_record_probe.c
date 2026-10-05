/* Offline probe references and joint block decisions for the record-recovery
 * harness. K56flex Draft 0.23, 4.10-4.12: PT_A/PT_B occupy 688+8 pairs;
 * PARAM_A/B preserve sequence/DC state. No live transmit or receive path. */
#include "k56flex_probe.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    k56flex_probe_t p;
    int16_t block[6];
    if (argc == 5 && !strcmp(argv[1], "reference")) {
        int stage = atoi(argv[2]), law = atoi(argv[3]), count = atoi(argv[4]);
        if (count < 0 || count > 1000000 || k56flex_probe_init(&p, stage, law, 0)) return 2;
        k56flex_probe_seed(&p, 0xffff);
        for (int i = 0; i < count; i += 6) {
            k56flex_probe_block(&p, block);
            int n = count - i < 6 ? count - i : 6;
            if (fwrite(block, sizeof(*block), n, stdout) != (size_t)n) return 3;
        }
        return 0;
    }
    if (argc != 6 || strcmp(argv[1], "decode")) return 2;
    int law = atoi(argv[3]), pacing = atoi(argv[4]);
    if ((pacing != 0 && pacing != 1) || k56flex_probe_init(&p, K56FLEX_PROBE_3, law, 0)) return 2;
    FILE *in = fopen(argv[2], "rb"), *out = fopen(argv[5], "wb");
    if (!in || !out) { if (in) fclose(in); if (out) fclose(out); return 3; }
    uint32_t history = 0;
    unsigned symbols = 0, bits = 0;
    float y[6];
    while (symbols < 1000000 && fread(y, sizeof(*y), 6, in) == 6) {
        if (symbols == (688u + 8u)*12u) {
            k56flex_probe_t old = p;
            k56flex_probe_init(&p, pacing ? K56FLEX_PROBE_PARAM_B : K56FLEX_PROBE_PARAM_A, law, 0);
            p.seq = old.seq; p.dc = old.dc;
        }
        double best = HUGE_VAL;
        uint32_t source = 0;
        for (uint32_t v = 0; v < (1u << p.cfg[3]); v++) {
            k56flex_probe_t candidate = p;
            k56flex_probe_block_src(&candidate, v, block);
            double error = 0;
            for (int i = 0; i < 6; i++) { double d = y[i] - block[i]; error += d*d; }
            if (error < best) { best = error; source = v; }
        }
        k56flex_probe_block_src(&p, source, block);
        for (unsigned i = 0; i < p.cfg[3]; i++) {
            unsigned b = (source >> i) & 1;
            uint8_t x = b ^ ((history >> 4) & 1) ^ ((history >> 22) & 1);
            history = (history << 1) | b;
            if (fwrite(&x, 1, 1, out) != 1) { fclose(in); fclose(out); return 3; }
            bits++;
        }
        symbols += 6;
    }
    int failed = ferror(in) || ferror(out);
    fclose(in); if (fclose(out)) failed = 1;
    printf("symbols=%u candidate_bits=%u payload_verified=0\n", symbols, bits);
    return failed ? 3 : 0;
}
