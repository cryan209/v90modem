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

#define RX_CAPTURE_BITS 65536

typedef struct
{
    const char *name;
    uint32_t lfsr;
    int bits_out;
    int bits_in;
    /*! The bits this endpoint received, so the run can be graded against the
        far end's generator rather than merely counted.  A bit count says a
        modulator ran; it says nothing about whether the control channel
        carries what was put into it, and the point of 12.4 is the data. */
    uint8_t rx_bits[RX_CAPTURE_BITS];
    int rx_len;
} endpoint_t;

static uint32_t lfsr_next(uint32_t *lfsr)
{
    uint32_t bit = *lfsr & 1;

    *lfsr = (*lfsr >> 1) ^ ((uint32_t) -(int32_t) bit & 0x80200003U);
    return bit;
}

/*! Grade a received bit stream against the far end's generator.  The receiver
    starts part way into the sequence -- the control channel begins carrying
    data at E, and the far end has been pulling bits since its own E -- so the
    alignment is searched, requiring a long exact run before it is believed.
    Returns the number of bit errors after the alignment, or -1 if no
    alignment was found. */
static int grade_rx(const endpoint_t *e, uint32_t far_seed, int *graded, int *offset)
{
    uint32_t ref;
    int off;
    int i;
    int errors;

    *graded = 0;
    *offset = -1;
    if (e->rx_len < 256)
        return -1;
    /*endif*/
    for (off = 0;  off + 256 <= e->rx_len;  off++)
    {
        uint32_t probe = far_seed;
        int match = 1;

        /* The reference is regenerated from the seed each time rather than
           advanced, because the LFSR is the far end's and this side has no
           access to its state. */
        for (i = 0;  i < off;  i++)
            lfsr_next(&probe);
        /*endfor*/
        ref = probe;
        for (i = 0;  i < 128;  i++)
        {
            if ((int) lfsr_next(&ref) != e->rx_bits[i])
            {
                match = 0;
                break;
            }
            /*endif*/
        }
        /*endfor*/
        if (!match)
            continue;
        /*endif*/
        ref = probe;
        errors = 0;
        for (i = 0;  i < e->rx_len;  i++)
        {
            if ((int) lfsr_next(&ref) != e->rx_bits[i])
            {
                if (errors < 8  &&  getenv("V34_HDX_ERRPOS"))
                    fprintf(stderr, "%s: bit error at rx index %d of %d\n", e->name, i, e->rx_len);
                /*endif*/
                errors++;
            }
            /*endif*/
        }
        /*endfor*/
        *graded = e->rx_len;
        *offset = off;
        return errors;
    }
    /*endfor*/
    return -1;
}

static int get_bit(void *user_data)
{
    endpoint_t *e = (endpoint_t *) user_data;

    e->bits_out++;
    return (int) lfsr_next(&e->lfsr);
}

static void put_bit(void *user_data, int bit)
{
    endpoint_t *e = (endpoint_t *) user_data;

    if (bit < 0)
        return;
    /*endif*/
    e->bits_in++;
    if (e->rx_len < RX_CAPTURE_BITS)
        e->rx_bits[e->rx_len++] = (uint8_t) (bit & 1);
    /*endif*/
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
    static endpoint_t call_e;
    static endpoint_t answ_e;
    static const uint32_t call_seed = 0x13579BDFU;
    static const uint32_t answ_seed = 0x2468ACE1U;
    int call_errors;
    int answ_errors;
    int call_graded;
    int answ_graded;
    int call_offset;
    int answ_offset;
    v34_state_t *call_modem;
    v34_state_t *answ_modem;
    int failed = 0;
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

    call_e.name = "call";
    call_e.lfsr = call_seed;
    answ_e.name = "answer";
    answ_e.lfsr = answ_seed;

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
    call_errors = grade_rx(&call_e, answ_seed, &call_graded, &call_offset);
    answ_errors = grade_rx(&answ_e, call_seed, &answ_graded, &answ_offset);
    if (call_errors < 0)
        printf("  control channel data call<-answer: NO ALIGNMENT in %d bits\n", call_e.rx_len);
    else
        printf("  control channel data call<-answer: %d errors in %d bits (offset %d)\n",
               call_errors, call_graded, call_offset);
    /*endif*/
    if (answ_errors < 0)
        printf("  control channel data answer<-call: NO ALIGNMENT in %d bits\n", answ_e.rx_len);
    else
        printf("  control channel data answer<-call: %d errors in %d bits (offset %d)\n",
               answ_errors, answ_graded, answ_offset);
    /*endif*/
    failed = (call_errors != 0  ||  answ_errors != 0);
    v34_free(call_modem);
    v34_free(answ_modem);
    return failed;
}
