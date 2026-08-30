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

int main(void)
{
    static const int rates[] = {4800, 7200, 9600, 12000, 14400};
    int16_t audio[160];
    uint32_t bits = 1;
    v32bis_state_t *modem;
    size_t i;

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
