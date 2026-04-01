#include <stdbool.h>
#include <stdint.h>

#include <spandsp.h>

/* Temporary stubs for environments where the bundled SpanDSP build does not
 * include V.34 objects or CRC functions. This keeps the project linkable
 * on Linux where the bundled libspandsp.a is a cross-compiled Mach-O archive. */

/* ITU-T CRC-16 (polynomial x^16+x^12+x^5+1, init 0xFFFF) — local implementation
 * that matches spandsp's crc_itu16_bits() and crc_itu16_calc() signatures. */
SPAN_DECLARE(uint16_t) crc_itu16_bits(uint8_t buf, int len, uint16_t crc)
{
    int i;
    for (i = 0; i < len; i++) {
        uint16_t b = (buf >> i) & 1;
        if ((crc ^ b) & 1)
            crc = (crc >> 1) ^ 0x8408;  /* reflected polynomial */
        else
            crc >>= 1;
    }
    return crc;
}

SPAN_DECLARE(uint16_t) crc_itu16_calc(const uint8_t *buf, int len, uint16_t crc)
{
    int i;
    for (i = 0; i < len; i++) {
        int j;
        uint8_t byte = buf[i];
        for (j = 0; j < 8; j++) {
            if ((crc ^ byte) & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
            byte >>= 1;
        }
    }
    return crc;
}

SPAN_DECLARE(bool) crc_itu16_check(const uint8_t *buf, int len)
{
    (void)buf;
    (void)len;
    return false;
}

SPAN_DECLARE(int) crc_itu16_append(uint8_t *buf, int len)
{
    (void)buf;
    return len;
}

/* v8 stubs — v8 is used in the loopback test but not in the V.34 path */
SPAN_DECLARE(int) v8_tx(v8_state_t *s, int16_t *amp, int max_len)
{
    int i;
    (void)s;
    for (i = 0; i < max_len; i++)
        amp[i] = 0;
    return max_len;
}

SPAN_DECLARE(int) v8_rx(v8_state_t *s, const int16_t *amp, int len)
{
    (void)s;
    (void)amp;
    return len;
}

SPAN_DECLARE(v8_state_t *) v8_init(v8_state_t *s,
                                   bool calling_party,
                                   v8_parms_t *parms,
                                   v8_result_handler_t result_handler,
                                   void *user_data)
{
    (void)s;
    (void)calling_party;
    (void)parms;
    (void)result_handler;
    (void)user_data;
    return NULL;
}

SPAN_DECLARE(int) v8_free(v8_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v8_restart(v8_state_t *s,
                             bool calling_party,
                             v8_parms_t *parms)
{
    (void)s;
    (void)calling_party;
    (void)parms;
    return 0;
}

SPAN_DECLARE(logging_state_t *) v8_get_logging_state(v8_state_t *s)
{
    (void)s;
    return NULL;
}

SPAN_DECLARE(const char *) v8_status_to_str(int status)
{
    (void)status;
    return "stub";
}

SPAN_DECLARE(const char *) v8_call_function_to_str(int call_function)
{
    (void)call_function;
    return "stub";
}

/* span_log stubs */
SPAN_DECLARE(int) span_log_set_level(logging_state_t *s, int level)
{
    (void)s;
    (void)level;
    return 0;
}

SPAN_DECLARE(int) span_log_set_tag(logging_state_t *s, const char *tag)
{
    (void)s;
    (void)tag;
    return 0;
}

/* modem_connect_tones stub */
SPAN_DECLARE(const char *) modem_connect_tone_to_str(int tone)
{
    (void)tone;
    return "stub";
}

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

SPAN_DECLARE(int) v34_get_phase3_j_bits(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v34_get_phase3_j_trn16(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v34_get_phase3_trn_lock_score(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v34_get_tx_data_mode(v34_state_t *s)
{
    (void)s;
    return 0;
}

SPAN_DECLARE(int) v34_get_v90_received_info0a(v34_state_t *s, v34_v90_info0a_t *info)
{
    (void)s;
    (void)info;
    return 0;
}

SPAN_DECLARE(int) v34_get_v90_received_info1a(v34_state_t *s, v34_v90_info1a_t *info)
{
    (void)s;
    (void)info;
    return 0;
}
