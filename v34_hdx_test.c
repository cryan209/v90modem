/*
 * v34_hdx_test.c - ITU-T V.34 clause 12 half-duplex startup harness.
 *
 * Drives a source/recipient V.34 half-duplex pair at each other over a G.711
 * bearer and reports exactly how far clause 12 gets: Phase 2 (12.2, INFOh in
 * place of INFO1a/INFO1c per 10.2.2), Phase 3 (12.3, S/S-bar/PP/TRN) and
 * control channel start-up (12.4, PPh/ALT/MPh/E).
 *
 * This is an instrument, not an assertion of conformance. It prints the stage
 * each side reached so a change can be scored against a previous run.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "spandsp.h"

#define BLOCK_SAMPLES 160

typedef struct
{
    const char *name;
    uint32_t lfsr;
    int bits_out;
    int bits_in;
} endpoint_t;

static int get_bit(void *user_data)
{
    endpoint_t *e = (endpoint_t *) user_data;
    int bit = e->lfsr & 1;

    e->lfsr = (e->lfsr >> 1) ^ ((uint32_t) -(int32_t) bit & 0x80200003U);
    e->bits_out++;
    return bit;
}

static void put_bit(void *user_data, int bit)
{
    endpoint_t *e = (endpoint_t *) user_data;

    if (bit >= 0)
        e->bits_in++;
}

static void g711_round_trip(int16_t out[], const int16_t in[], int len, int alaw)
{
    int i;

    for (i = 0;  i < len;  i++)
    {
        out[i] = alaw ? alaw_to_linear(linear_to_alaw(in[i]))
                      : ulaw_to_linear(linear_to_ulaw(in[i]));
    }
}

int main(int argc, char *argv[])
{
    int baud = (argc > 1) ? atoi(argv[1]) : 3200;
    int bps = (argc > 2) ? atoi(argv[2]) : 9600;
    int alaw = (argc > 3  &&  strcmp(argv[3], "alaw") == 0);
    double seconds = (argc > 4) ? atof(argv[4]) : 20.0;
    endpoint_t call_e = {"call", 0x13579BDFU, 0, 0};
    endpoint_t answ_e = {"answer", 0x2468ACE1U, 0, 0};
    v34_state_t *call_modem;
    v34_state_t *answ_modem;
    int16_t call_tx[BLOCK_SAMPLES];
    int16_t answ_tx[BLOCK_SAMPLES];
    int16_t call_rx[BLOCK_SAMPLES];
    int16_t answ_rx[BLOCK_SAMPLES];
    int blocks = (int) (seconds*8000.0/BLOCK_SAMPLES);
    int block;
    int best_call_tx = -1;
    int best_answ_tx = -1;
    int best_call_rx = -1;
    int best_answ_rx = -1;

    /* duplex = false selects the clause 12 half-duplex modem. V.34 3.11/3.14:
       the source modem transmits primary channel data, the recipient receives
       it; the control channel is bidirectional throughout. */
    call_modem = v34_init(NULL, baud, bps, true, false,
                          get_bit, &call_e, put_bit, &call_e);
    answ_modem = v34_init(NULL, baud, bps, false, false,
                          get_bit, &answ_e, put_bit, &answ_e);
    if (call_modem == NULL  ||  answ_modem == NULL)
    {
        fprintf(stderr, "v34_hdx_test: v34_init failed\n");
        return 1;
    }
    v34_tx_power(call_modem, -12.0f);
    v34_tx_power(answ_modem, -12.0f);
    /* 12.2.1: call modem as source modem. */
    v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_SOURCE);
    v34_half_duplex_change_mode(answ_modem, V34_HALF_DUPLEX_RECIPIENT);
    if (getenv("V34_HDX_LOG"))
    {
        span_log_set_level(v34_get_logging_state(call_modem),
                           SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
        span_log_set_level(v34_get_logging_state(answ_modem),
                           SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
        /* Both modems log to the same stderr. Without distinct tags the two
           are indistinguishable, and a diagnostic read off the wrong one sends
           the investigation after a fault that is not there. */
        span_log_set_tag(v34_get_logging_state(call_modem), "SRC");
        span_log_set_tag(v34_get_logging_state(answ_modem), "RCP");
    }

    for (block = 0;  block < blocks;  block++)
    {
        memset(call_tx, 0, sizeof(call_tx));
        memset(answ_tx, 0, sizeof(answ_tx));
        v34_tx(call_modem, call_tx, BLOCK_SAMPLES);
        v34_tx(answ_modem, answ_tx, BLOCK_SAMPLES);
        g711_round_trip(answ_rx, call_tx, BLOCK_SAMPLES, alaw);
        g711_round_trip(call_rx, answ_tx, BLOCK_SAMPLES, alaw);
        v34_rx(answ_modem, answ_rx, BLOCK_SAMPLES);
        v34_rx(call_modem, call_rx, BLOCK_SAMPLES);

        if (getenv("V34_HDX_RMS"))
        {
            double e = 0.0;
            double f = 0.0;
            int k;

            for (k = 0;  k < BLOCK_SAMPLES;  k++)
            {
                e += (double) call_tx[k]*call_tx[k];
                f += (double) answ_tx[k]*answ_tx[k];
            }
            printf("  %7.3fs  src tx rms %8.1f  rcp tx rms %8.1f  (src stage %d)\n",
                   block*BLOCK_SAMPLES/8000.0,
                   sqrt(e/BLOCK_SAMPLES), sqrt(f/BLOCK_SAMPLES),
                   v34_get_tx_stage(call_modem));
        }
        /*endif*/
        if (getenv("V34_HDX_STAGES"))
        {
            static int p_ct = -1, p_at = -1, p_cr = -1, p_ar = -1;
            int ct = v34_get_tx_stage(call_modem);
            int at = v34_get_tx_stage(answ_modem);
            int cr = v34_get_rx_stage(call_modem);
            int ar = v34_get_rx_stage(answ_modem);

            if (ct != p_ct  ||  at != p_at  ||  cr != p_cr  ||  ar != p_ar)
            {
                printf("  %7.3fs  src tx=%-2d rx=%-2d | rcp tx=%-2d rx=%-2d\n",
                       block*BLOCK_SAMPLES/8000.0, ct, cr, at, ar);
                p_ct = ct;  p_at = at;  p_cr = cr;  p_ar = ar;
            }
            /*endif*/
        }
        /*endif*/
        if (v34_get_tx_stage(call_modem) > best_call_tx)
            best_call_tx = v34_get_tx_stage(call_modem);
        if (v34_get_tx_stage(answ_modem) > best_answ_tx)
            best_answ_tx = v34_get_tx_stage(answ_modem);
        if (v34_get_rx_stage(call_modem) > best_call_rx)
            best_call_rx = v34_get_rx_stage(call_modem);
        if (v34_get_rx_stage(answ_modem) > best_answ_rx)
            best_answ_rx = v34_get_rx_stage(answ_modem);
    }

    printf("V.34 half-duplex (clause 12), %d baud %d bps %s, %.1f s\n",
           baud, bps, alaw ? "A-law" : "u-law", seconds);
    printf("  call   (source)    tx stage %2d (max %2d)  rx stage %2d (max %2d)\n",
           v34_get_tx_stage(call_modem), best_call_tx,
           v34_get_rx_stage(call_modem), best_call_rx);
    printf("  answer (recipient) tx stage %2d (max %2d)  rx stage %2d (max %2d)\n",
           v34_get_tx_stage(answ_modem), best_answ_tx,
           v34_get_rx_stage(answ_modem), best_answ_rx);
    printf("  payload bits: call out %d in %d, answer out %d in %d\n",
           call_e.bits_out, call_e.bits_in, answ_e.bits_out, answ_e.bits_in);
    v34_free(call_modem);
    v34_free(answ_modem);
    return 0;
}
