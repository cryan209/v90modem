/*
 * v34_hdx_test.c - ITU-T V.34 clause 12 half-duplex startup harness.
 *
 * Drives a source/recipient V.34 half-duplex pair at each other over a G.711
 * bearer and reports exactly how far clause 12 gets: Phase 2 (12.2, INFOh in
 * place of INFO1a/INFO1c per 10.2.2), Phase 3 (12.3, S/S-bar/PP/TRN) and
 * control channel start-up (12.4, PPh/ALT/MPh/E).
 *
 * Grades independent payload generators on the bidirectional control channel.
 * V34_HDX_PRIMARY=1 then requests 12.5 S/S-bar/PP/B1 and grades the source's
 * primary payload, requiring no skipped payload bits and a silent recipient.
 * V34_HDX_RX_DUMP=<path> saves recovered primary bits (one byte per bit).
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
static int grade_rx_skip(const endpoint_t *e, uint32_t far_seed,
                         int max_skip, int *graded, int *offset, int *skipped)
{
    uint8_t reference[RX_CAPTURE_BITS + 4096];
    uint32_t state = far_seed;
    int skip, off, i, errors;

    *graded = 0;
    *offset = -1;
    *skipped = 0;
    if (e->rx_len < 256)
        return -1;
    for (i = 0; i < (int) sizeof(reference); i++)
        reference[i] = (uint8_t) lfsr_next(&state);
    for (skip = 0; skip <= max_skip && skip + 256 <= e->rx_len; skip++)
    {
        for (off = 0; off < 4096; off++)
        {
            if (memcmp(reference + off, e->rx_bits + skip, 128))
                continue;
            errors = 0;
            for (i = skip; i < e->rx_len; i++)
                errors += reference[off + i - skip] != e->rx_bits[i];
            *graded = e->rx_len - skip;
            *offset = off;
            *skipped = skip;
            return errors;
        }
    }
    return -1;
}

static int grade_rx(const endpoint_t *e, uint32_t far_seed, int *graded, int *offset)
{
    int skipped;
    return grade_rx_skip(e, far_seed, 0, graded, offset, &skipped);
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
    /* An optional second bit rate lets the two ends be configured
       differently, which is the only way to see V.34 12.4.1.3/12.4.2.4 do
       anything: with both ends the same, a negotiation that ignored the far
       end entirely would give the same answer. */
    int answ_bps = (argc > 5) ? atoi(argv[5]) : bps;
    int expect_bps = (bps < answ_bps) ? bps : answ_bps;
    int call_rate;
    int answ_rate;
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
    int primary = getenv("V34_HDX_PRIMARY") != NULL;
    int primary_started = 0;
    int control_ok = 0;
    int restart = getenv("V34_HDX_RESTART") != NULL;
    int restarted = 0;
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
    answ_modem = v34_init(NULL, baud, answ_bps, false, false,
                          get_bit, &answ_e, put_bit, &answ_e);
    if (call_modem == NULL  ||  answ_modem == NULL)
    {
        fprintf(stderr, "v34_hdx_test: v34_init failed\n");
        return 1;
    }
    v34_tx_power(call_modem, -12.0f);
    v34_tx_power(answ_modem, -12.0f);
    /* Unknown modes and a primary request before MPh/E must be refused. */
    if (v34_half_duplex_change_mode(NULL, V34_HALF_DUPLEX_PRIMARY_CHANNEL) != -1
        || v34_half_duplex_change_mode(call_modem, -1) != -1
        || v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL) != -1)
    {
        fprintf(stderr, "invalid/early HDX mode request accepted\n");
        v34_free(call_modem);
        v34_free(answ_modem);
        return 1;
    }
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
        if (primary && restart && primary_started && !restarted
            && answ_e.rx_len >= 10000)
        {
            int skip;
            if (grade_rx_skip(&answ_e, call_seed, 2048, &answ_graded,
                              &answ_offset, &skip) != 0 || answ_offset != 0
                || answ_graded < 8000
                || v34_restart(call_modem, baud, bps, false)
                || v34_restart(answ_modem, baud, answ_bps, false))
            {
                fprintf(stderr, "primary payload/restart failed\n");
                failed = 1;
                break;
            }
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            primary_started = control_ok = 0;
            restarted = 1;
            printf("  restarted after verified primary payload at %.3fs\n",
                   block*BLOCK_SAMPLES/8000.0);
        }
        if (primary && !primary_started
            && v34_get_hdx_control_channel_ready(call_modem)
            && v34_get_hdx_control_channel_ready(answ_modem)
            && call_e.rx_len >= 512 && answ_e.rx_len >= 512)
        {
            control_ok = grade_rx(&call_e, answ_seed, &call_graded, &call_offset) == 0
                      && grade_rx(&answ_e, call_seed, &answ_graded, &answ_offset) == 0;
            if (!control_ok
                || v34_half_duplex_change_mode(answ_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL)
                || v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL))
            {
                fprintf(stderr, "primary transition failed\n");
                failed = 1;
                break;
            }
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            primary_started = 1;
            printf("  primary channel requested at %.3fs after verified control data\n",
                   block*BLOCK_SAMPLES/8000.0);
        }
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
    call_rate = v34_get_hdx_negotiated_bit_rate(call_modem);
    answ_rate = v34_get_hdx_negotiated_bit_rate(answ_modem);
    printf("  MPh rate (12.4.1.3/12.4.2.4): source tx %d bit/s, recipient rx %d bit/s (expected %d)\n",
           call_rate, answ_rate, expect_bps);
    printf("  payload bits: call out %d in %d, answer out %d in %d\n",
           call_e.bits_out, call_e.bits_in, answ_e.bits_out, answ_e.bits_in);
    if (!primary)
    {
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
    }
    if (primary)
    {
        const char *dump = getenv("V34_HDX_RX_DUMP");
        if (dump)
        {
            FILE *fp = fopen(dump, "wb");
            if (!fp || fwrite(answ_e.rx_bits, 1, answ_e.rx_len, fp) != (size_t) answ_e.rx_len)
                failed = 1;
            if (fp)
                fclose(fp);
        }
        /* B1 is not payload. Find a 128-bit exact prefix of the independent
           source generator after the reset-state B1 interval, then grade all
           remaining bits. Require source offset zero: acquisition may skip
           B1, but it must not hide a corrupt payload prefix. */
        int skip;
        answ_errors = grade_rx_skip(&answ_e, call_seed, 2048,
                                    &answ_graded, &answ_offset, &skip);
        printf("  primary recipient: %d errors in %d bits (B1 skipped %d, source offset %d)\n",
               answ_errors, answ_graded, skip, answ_offset);
        failed |= !primary_started || !control_ok || (restart && !restarted)
               || answ_errors != 0
               || answ_graded < 8000 || answ_offset != 0
               || answ_e.bits_out != 0 || call_e.bits_in != 0;
    }
    else
    {
        failed |= call_errors != 0 || answ_errors != 0;
    }
    failed |= call_rate != expect_bps || answ_rate != expect_bps;
    v34_free(call_modem);
    v34_free(answ_modem);
    return failed;
}
