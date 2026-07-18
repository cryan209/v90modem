#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Offline oracle for SmartLink's V.90 spectral shaper.  Input records contain
 * six positive linear PCM magnitudes followed by five scrambled sign bits
 * (Sr=1).  Output records contain SmartLink's six signed linear values. */

struct shaper_input {
    int16_t magnitude[6];
    uint8_t sign_bits[5];
} __attribute__((packed));

extern void smartlink_shaper_ctor(void *state)
    __asm__("_ZN17V90SpectralShaperC1Ev");
extern void smartlink_shaper_reset(void *state,
                                  unsigned lookahead,
                                  unsigned redundancy,
                                  float a1,
                                  float a2,
                                  float b1,
                                  float b2)
    __asm__("_ZN17V90SpectralShaper5resetEjjffff");
extern void smartlink_shaper_process(void *state,
                                    int16_t *magnitudes,
                                    uint8_t *sign_bits,
                                    int16_t *output)
    __asm__("_ZN17V90SpectralShaper7processEPsPhS0_");

int main(int argc, char **argv)
{
    unsigned char state[512] __attribute__((aligned(16)));
    struct shaper_input input;
    int16_t output[6];
    FILE *in;
    FILE *out;
    unsigned frame = 0;

    if (argc != 3) {
        fprintf(stderr, "usage: %s input.bin output.s16\n", argv[0]);
        return 2;
    }
    in = fopen(argv[1], "rb");
    out = fopen(argv[2], "wb");
    if (!in || !out) {
        perror(!in ? argv[1] : argv[2]);
        return 2;
    }
    memset(state, 0, sizeof(state));
    smartlink_shaper_ctor(state);
    smartlink_shaper_reset(state, 1, 1, 1.0f, 0.0f, 0.0f, 0.0f);
    while (fread(&input, sizeof(input), 1, in) == 1) {
        memset(output, 0, sizeof(output));
        smartlink_shaper_process(state,
                                 input.magnitude,
                                 input.sign_bits,
                                 output);
        if (frame++ > 0)
            (void)fwrite(output, sizeof(output[0]), 6, out);
    }
    fclose(in);
    fclose(out);
    return 0;
}
