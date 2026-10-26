/* Splice our K56flex client upstream into a recorded one, for replay into
 * MicaEmu's mica_trace (the real MICA firmware as the server).
 *
 *   k56flex_client_splice IN.ulaw OUT.ulaw AT_S HIGH PR PE TRN16 SECONDS
 *
 * OUT is IN up to AT_S seconds (that call's V.8bis, Phase 2 and INFO1a,
 * which set MICA's state), then SECONDS of k56flex_client_tx from S onward
 * through SpanDSP's V.34 modulator at V.34 Table 1 symbol rate BAUD (env,
 * default 4 = 3200), carrier HIGH, INFO1a power
 * reduction PR and pre-emphasis PE (V.34 11.2.1.1.8), mu-law.  The client's
 * configuration comes from the environment (unset = k56flex_client_tx's
 * defaults): MD, TRN, RECORDS, REPORT (hex), JREC, JBARE, JNONE, GAP, MPREP,
 * ACKREP, MASK (hex), DOWN, ASYM, RBARE, MPFILL, BAUD.  Stage start times go to stderr.
 *
 * The prefix's INFO1c governs what MICA expects: native HSF's announces an
 * MD of ~700 ms (V.34 11.3.1), so a splice into an HSF call needs MD=2200.
 * Replays say nothing about hardware; see docs/k56flex_implementation.md. */
#include "k56flex_client_tx.h"

#include <spandsp.h>
#include <stdio.h>
#include <stdlib.h>

static int get_bit(void *user) { (void) user; return 1; }
static void put_bit(void *user, int bit) { (void) user; (void) bit; }

static void source(void *user, float *re, float *im)
{
    k56flex_client_tx_symbol((k56flex_client_tx_t *) user, re, im);
}

static unsigned env_u(const char *name, unsigned dflt, int base)
{
    const char *v = getenv(name);
    return (v && *v) ? (unsigned) strtoul(v, NULL, base) : dflt;
}

int main(int argc, char **argv)
{
    k56flex_client_tx_cfg_t cfg;
    k56flex_client_tx_t tx;
    v34_state_t *v34;
    FILE *in, *out;
    double at, seconds;
    int high, pr, pe, c, baud, last = -1;
    long n, prefix;
    int16_t amp[160];

    if (argc != 9) {
        fprintf(stderr, "usage: %s IN.ulaw OUT.ulaw AT_S HIGH PR PE TRN16 SECONDS\n", argv[0]);
        return 2;
    }
    in = fopen(argv[1], "rb");
    out = fopen(argv[2], "wb");
    if (!in  ||  !out) {
        perror("open");
        return 2;
    }
    at = atof(argv[3]);
    high = atoi(argv[4]);
    pr = atoi(argv[5]);
    pe = atoi(argv[6]);
    seconds = atof(argv[8]);
    baud = (int) env_u("BAUD", 4, 10);

    k56flex_client_tx_defaults(&cfg);
    cfg.trn16 = (unsigned) atoi(argv[7]);
    cfg.md_symbols = env_u("MD", cfg.md_symbols, 10);
    cfg.trn_symbols = env_u("TRN", cfg.trn_symbols, 10);
    cfg.report_records = env_u("RECORDS", cfg.report_records, 10);
    cfg.report = (uint16_t) env_u("REPORT", cfg.report, 16);
    cfg.jprime_records = env_u("JREC", cfg.jprime_records, 10);
    cfg.jprime_bare = env_u("JBARE", 0, 10);
    cfg.report_bare = env_u("RBARE", 0, 10);
    cfg.mp_extra_fill = env_u("MPFILL", cfg.mp_extra_fill, 10);
    cfg.jprime_none = env_u("JNONE", 0, 10);
    cfg.gap_symbols = env_u("GAP", cfg.gap_symbols, 10);
    cfg.mp_repeats = env_u("MPREP", cfg.mp_repeats, 10);
    cfg.mp_ack_repeats = env_u("ACKREP", cfg.mp_ack_repeats, 10);
    cfg.mp_mask = env_u("MASK", cfg.mp_mask, 16);
    cfg.mp_down_code = env_u("DOWN", cfg.mp_down_code, 10);
    cfg.mp_asymmetric = env_u("ASYM", cfg.mp_asymmetric, 10);
    if (k56flex_client_tx_init(&tx, &cfg) != 0) {
        fprintf(stderr, "invalid client configuration\n");
        return 2;
    }

    prefix = (long) (at*8000);
    for (n = 0;  n < prefix  &&  (c = fgetc(in)) != EOF;  n++)
        fputc(c, out);

    v34 = v34_init(NULL, 3200, 31200, true, true, get_bit, NULL, put_bit, NULL);
    v34_tx_phase3_from_info1a(v34, baud, high, pr, pe);
    v34_tx_start_external_symbols(v34, baud, high, source, &tx);
    k56flex_client_tx_start(&tx);
    for (n = 0;  n < (long) (seconds*8000);  n += 160) {
        v34_tx(v34, amp, 160);
        if ((int) tx.stage != last) {
            fprintf(stderr, "%.3f %s\n", at + n/8000.0, k56flex_client_tx_stage_name(tx.stage));
            last = (int) tx.stage;
        }
        for (int i = 0;  i < 160;  i++)
            fputc(linear_to_ulaw(amp[i]), out);
    }
    v34_free(v34);
    fclose(in);
    fclose(out);
    return 0;
}
