#include <stdbool.h>
#include <stdint.h>

#include <spandsp.h>

/* Temporary stubs for environments where the bundled SpanDSP build does not
 * include V.34 objects. This keeps the project linkable while non-V.34 paths
 * remain available. */

SPAN_DECLARE(v34_state_t *) v34_init(v34_state_t *s,
                                     int baud_rate,
                                     int bit_rate,
                                     bool calling_party,
                                     bool duplex,
                                     span_get_bit_func_t get_bit,
                                     void *get_bit_user_data,
                                     span_put_bit_func_t put_bit,
                                     void *put_bit_user_data)
{
    (void)s;
    (void)baud_rate;
    (void)bit_rate;
    (void)calling_party;
    (void)duplex;
    (void)get_bit;
    (void)get_bit_user_data;
    (void)put_bit;
    (void)put_bit_user_data;
    return NULL;
}

SPAN_DECLARE(int) v34_free(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v34_rx(v34_state_t *s, const int16_t amp[], int len)
{
    (void)s;
    (void)amp;
    return len;
}

SPAN_DECLARE(int) v34_tx(v34_state_t *s, int16_t amp[], int len)
{
    int i;
    (void)s;
    for (i = 0; i < len; i++) {
        amp[i] = 0;
    }
    return len;
}

SPAN_DECLARE(void) v34_tx_power(v34_state_t *s, float power)
{
    (void)s;
    (void)power;
}

SPAN_DECLARE(int) v34_get_current_bit_rate(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(logging_state_t *) v34_get_logging_state(v34_state_t *s)
{
    (void)s;
    return NULL;
}

SPAN_DECLARE(void) v34_set_put_aux_bit(v34_state_t *s, span_put_bit_func_t put_bit, void *user_data)
{
    (void)s;
    (void)put_bit;
    (void)user_data;
}

SPAN_DECLARE(int) v34_get_rx_stage(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v34_get_tx_stage(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v34_get_v90_u_info(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(bool) v34_get_primary_channel_active(v34_state_t *s)
{
    (void)s;
    return false;
}

SPAN_DECLARE(int) v34_get_rx_event(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(void) v34_force_phase4(v34_state_t *s)
{
    (void)s;
}

SPAN_DECLARE(void) v34_set_v90_mode(v34_state_t *s, int pcm_law)
{
    (void)s;
    (void)pcm_law;
}
