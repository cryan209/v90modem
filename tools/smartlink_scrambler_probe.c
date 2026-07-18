#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Offline oracle for SmartLink's byte-at-a-time GPC scrambler.  SmartLink's
 * Phase-4 modulator constructs Scrambler<unsigned char, unsigned char> with
 * taps 18 and 23 and a 99-byte history.  Emit one byte per input bit so the
 * exact sequence and bit convention can be compared without guessing its
 * private object layout. */

extern void smartlink_scrambler_ctor(void *state,
                                     unsigned tap1,
                                     unsigned tap2,
                                     unsigned history)
    __asm__("_ZN9ScramblerIhhEC1Ejjj");
extern void smartlink_scrambler_reset(void *state, uint8_t value)
    __asm__("_ZN9ScramblerIhhE5resetEh");
extern uint8_t smartlink_scrambler_process(void *state, uint8_t input)
    __asm__("_ZN9ScramblerIhhE7processEh");

int main(int argc, char **argv)
{
    unsigned char state[128] __attribute__((aligned(16)));
    unsigned count;
    unsigned input;
    FILE *out;

    if (argc != 4) {
        fprintf(stderr, "usage: %s output.u8 count input-bit\n", argv[0]);
        return 2;
    }
    count = (unsigned)strtoul(argv[2], NULL, 0);
    input = (unsigned)strtoul(argv[3], NULL, 0);
    if (input > 1) {
        fprintf(stderr, "input-bit must be 0 or 1\n");
        return 2;
    }
    out = fopen(argv[1], "wb");
    if (!out) {
        perror(argv[1]);
        return 2;
    }

    memset(state, 0, sizeof(state));
    smartlink_scrambler_ctor(state, 18, 23, 99);
    smartlink_scrambler_reset(state, 0);
    for (unsigned i = 0; i < count; i++) {
        uint8_t bit = smartlink_scrambler_process(state, (uint8_t)input);
        if (fwrite(&bit, sizeof(bit), 1, out) != 1) {
            perror(argv[1]);
            fclose(out);
            return 2;
        }
    }
    fclose(out);
    return 0;
}
