/*
 * SpanDSP V.32bis infrastructure smoke test.
 *
 * This deliberately does not claim modem interoperability.  It pins the
 * native API that the clause-level Python reference will be used to complete:
 * all five rates must initialise/restart honestly, invalid rates must fail,
 * and the inherited V.17 pulse-shaping core must produce audio.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <spandsp.h>

static int get_bit(void *user_data)
{
    uint32_t *state = (uint32_t *) user_data;

    *state = *state * 1664525U + 1013904223U;
    return (int) (*state >> 31);
}

static void put_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

static int test_startup_logic(void)
{
    static const int rates[] = {4800, 7200, 9600, 12000, 14400};
    static const uint8_t caller_r_states[8] = {0, 0, 0, 0, 1, 0, 3, 3};
    static const uint8_t caller_e_states[8] = {3, 0, 2, 0, 3, 2, 1, 0};
    static const uint8_t answer_r_states[8] = {0, 2, 1, 0, 1, 1, 1, 1};
    static const uint8_t answer_e_states[8] = {3, 2, 1, 1, 0, 3, 0, 1};
    uint8_t conditioning[256 + 16 + 1280];
    uint8_t states[8];
    uint16_t r_word;
    uint16_t e_word;
    uint16_t decoded_word;
    uint32_t reg;
    int diff;
    int decoded;
    size_t i;

    if (v32bis_build_rate_signal(0x1660, &r_word) != 0  ||  r_word != 0x17F0
        ||  v32bis_decode_rate_signal(r_word, &decoded) != 0  ||  decoded != 0x1660
        ||  v32bis_decode_rate_signal((uint16_t) (r_word | 1), &decoded) == 0)
    {
        fprintf(stderr, "V.32bis Table 5 rate-signal codec failed\n");
        return -1;
    }
    for (i = 0;  i < sizeof(rates)/sizeof(rates[0]);  i++)
    {
        if (v32bis_build_e_signal(rates[i], &e_word) != 0
            || v32bis_decode_e_signal(e_word, &decoded) != 0
            || decoded != rates[i])
        {
            fprintf(stderr, "V.32bis Table 6 E codec failed at %d bit/s\n", rates[i]);
            return -1;
        }
    }
    if (v32bis_build_e_signal(14400, &e_word) != 0  ||  e_word != 0x999F
        || v32bis_decode_e_signal((uint16_t) (e_word | V32BIS_RATE_12000), &decoded) == 0
        || v32bis_select_common_rate(V32BIS_RATE_14400 | V32BIS_RATE_9600,
                                     V32BIS_RATE_12000 | V32BIS_RATE_9600) != 9600
        || v32bis_select_common_rate(V32BIS_RATE_14400, V32BIS_RATE_12000) != 0)
    {
        fprintf(stderr, "V.32bis startup rate selection failed\n");
        return -1;
    }

    if (v32bis_build_conditioning(true, 1280, conditioning, &reg, &diff) != 1552
        || conditioning[0] != V32BIS_STARTUP_A
        || conditioning[1] != V32BIS_STARTUP_B
        || conditioning[256] != V32BIS_STARTUP_C
        || conditioning[257] != V32BIS_STARTUP_D
        || reg != 0x15C6B0  ||  diff != V32BIS_STARTUP_A)
    {
        fprintf(stderr, "V.32bis caller conditioning sequence failed\n");
        return -1;
    }
    if (v32bis_encode_startup_word(true, r_word, &reg, &diff, states) != 8
        || memcmp(states, caller_r_states, sizeof(states)) != 0
        || reg != 0x3055C9  ||  diff != 3)
    {
        fprintf(stderr, "V.32bis caller R word encoding failed\n");
        return -1;
    }
    reg = 0x15C6B0;
    diff = V32BIS_STARTUP_A;
    decoded_word = 0;
    if (v32bis_decode_startup_word(true, states, &reg, &diff, &decoded_word) != 8
        || decoded_word != r_word  ||  reg != 0x3055C9  ||  diff != 3)
    {
        fprintf(stderr, "V.32bis caller R word decoding failed\n");
        return -1;
    }
    reg = 0x15C6B0;
    diff = V32BIS_STARTUP_A;
    if (v32bis_encode_startup_word(true, e_word, &reg, &diff, states) != 8
        || memcmp(states, caller_e_states, sizeof(states)) != 0
        || reg != 0x30A3B8  ||  diff != 0)
    {
        fprintf(stderr, "V.32bis caller E word encoding failed\n");
        return -1;
    }

    if (v32bis_build_conditioning(false, 1280, conditioning, &reg, &diff) != 1552
        || reg != 0x3BBD8E  ||  diff != V32BIS_STARTUP_B)
    {
        fprintf(stderr, "V.32bis answerer conditioning sequence failed\n");
        return -1;
    }
    if (v32bis_encode_startup_word(false, r_word, &reg, &diff, states) != 8
        || memcmp(states, answer_r_states, sizeof(states)) != 0
        || reg != 0x0E08D5  ||  diff != 1)
    {
        fprintf(stderr, "V.32bis answerer R word encoding failed\n");
        return -1;
    }
    reg = 0x3BBD8E;
    diff = V32BIS_STARTUP_B;
    if (v32bis_decode_startup_word(false, states, &reg, &diff, &decoded_word) != 8
        || decoded_word != r_word  ||  reg != 0x0E08D5  ||  diff != 1)
    {
        fprintf(stderr, "V.32bis answerer R word decoding failed\n");
        return -1;
    }
    reg = 0x3BBD8E;
    diff = V32BIS_STARTUP_B;
    if (v32bis_encode_startup_word(false, e_word, &reg, &diff, states) != 8
        || memcmp(states, answer_e_states, sizeof(states)) != 0
        || reg != 0x0EF92B  ||  diff != 1)
    {
        fprintf(stderr, "V.32bis answerer E word encoding failed\n");
        return -1;
    }
    return 0;
}

int main(void)
{
    static const int rates[] = {4800, 7200, 9600, 12000, 14400};
    int16_t audio[160];
    uint32_t bits = 1;
    v32bis_state_t *modem;
    size_t i;

    if (test_startup_logic() != 0)
        return 1;

    if (v32bis_init(NULL, 12345, true, get_bit, &bits, put_bit, NULL) != NULL)
    {
        fprintf(stderr, "V.32bis accepted an invalid initial rate\n");
        return 1;
    }

    modem = v32bis_init(NULL, rates[0], true, get_bit, &bits, put_bit, NULL);
    if (modem == NULL)
    {
        fprintf(stderr, "V.32bis initialisation failed\n");
        return 1;
    }

    if (v32bis_set_supported_bit_rates(modem, 0) == 0
        || v32bis_set_supported_bit_rates(modem, 1) == 0)
    {
        fprintf(stderr, "V.32bis accepted an invalid supported-rate mask\n");
        v32bis_free(modem);
        return 1;
    }
    if (v32bis_set_supported_bit_rates(modem,
                                       V32BIS_RATE_4800
                                     | V32BIS_RATE_7200
                                     | V32BIS_RATE_9600
                                     | V32BIS_RATE_12000
                                     | V32BIS_RATE_14400) != 0)
    {
        fprintf(stderr, "V.32bis rejected the complete supported-rate mask\n");
        v32bis_free(modem);
        return 1;
    }

    for (i = 0; i < sizeof(rates)/sizeof(rates[0]); i++)
    {
        if (v32bis_restart(modem, rates[i]) != 0
            || v32bis_current_bit_rate(modem) != rates[i])
        {
            fprintf(stderr, "V.32bis restart failed at %d bit/s\n", rates[i]);
            v32bis_free(modem);
            return 1;
        }
        if (v32bis_tx(modem, audio, (int) (sizeof(audio)/sizeof(audio[0]))) <= 0)
        {
            fprintf(stderr, "V.32bis produced no audio at %d bit/s\n", rates[i]);
            v32bis_free(modem);
            return 1;
        }
    }

    if (v32bis_restart(modem, 12345) == 0
        || v32bis_current_bit_rate(modem) != rates[sizeof(rates)/sizeof(rates[0]) - 1])
    {
        fprintf(stderr, "V.32bis invalid restart corrupted the current rate\n");
        v32bis_free(modem);
        return 1;
    }

    v32bis_rx_set_signal_cutoff(modem, -43.0f);
    v32bis_release(modem);
    v32bis_free(modem);
    puts("SpanDSP V.32bis infrastructure smoke test passed");
    return 0;
}
