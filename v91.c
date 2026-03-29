/*
 * v91.c - V.91 PCM modem helpers
 */

#include "v91.h"

#include <spandsp.h>

#include <string.h>

void v91_init(v91_state_t *s, v91_law_t law, v91_mode_t mode)
{
    s->law = law;
    s->mode = mode;
}

uint8_t v91_idle_codeword(v91_law_t law)
{
    return (law == V91_LAW_ALAW) ? (uint8_t) 0xD5 : (uint8_t) 0xFF;
}

int16_t v91_codeword_to_linear(v91_law_t law, uint8_t codeword)
{
    if (law == V91_LAW_ALAW)
        return alaw_to_linear(codeword);
    return ulaw_to_linear(codeword);
}

uint8_t v91_linear_to_codeword(v91_law_t law, int16_t sample)
{
    if (law == V91_LAW_ALAW)
        return linear_to_alaw(sample);
    return linear_to_ulaw(sample);
}

int v91_tx_codewords(v91_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len)
{
    (void) s;

    if (g711_max <= 0 || data_len <= 0)
        return 0;

    if (data_len > g711_max)
        data_len = g711_max;

    memcpy(g711_out, data_in, (size_t) data_len);
    return data_len;
}

int v91_rx_codewords(v91_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len)
{
    (void) s;

    if (data_max <= 0 || g711_len <= 0)
        return 0;

    if (g711_len > data_max)
        g711_len = data_max;

    memcpy(data_out, g711_in, (size_t) g711_len);
    return g711_len;
}

int v91_tx_linear(v91_state_t *s,
                  int16_t *amp_out,
                  int amp_max,
                  const uint8_t *data_in,
                  int data_len)
{
    int i;
    int count;

    (void) s;

    if (amp_max <= 0 || data_len <= 0)
        return 0;

    count = (data_len < amp_max) ? data_len : amp_max;
    for (i = 0; i < count; i++)
        amp_out[i] = v91_codeword_to_linear(s->law, data_in[i]);
    return count;
}

int v91_rx_linear(v91_state_t *s,
                  uint8_t *data_out,
                  int data_max,
                  const int16_t *amp_in,
                  int amp_len)
{
    int i;
    int count;

    if (data_max <= 0 || amp_len <= 0)
        return 0;

    count = (amp_len < data_max) ? amp_len : data_max;
    for (i = 0; i < count; i++)
        data_out[i] = v91_linear_to_codeword(s->law, amp_in[i]);
    return count;
}

void v91_tx_idle(v91_state_t *s, int16_t *amp_out, int amp_len)
{
    int16_t sample;
    int i;

    sample = v91_codeword_to_linear(s->law, v91_idle_codeword(s->law));
    for (i = 0; i < amp_len; i++)
        amp_out[i] = sample;
}
