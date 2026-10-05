/*
 * v250_ctl_test.c -- V.250 +MR/+ES/+ER/+DS/+DR parameter handling on its own.
 * Clause references are to V.250 (07/2003).
 */

#include "v250_ctl.h"

#include <stdio.h>
#include <string.h>

static int failures;

static void check(int ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        failures++;
}

/* A command: result and information text. */
static void cmd(v250_ctl_t *c, const char *text, v250_ctl_result_t want, const char *want_info)
{
    char info[200];
    char what[300];
    v250_ctl_result_t r = v250_ctl_command(c, text, info, sizeof(info));

    snprintf(what, sizeof(what), "+%s -> %s%s%s", text,
             r == V250_CTL_OK ? "OK" : r == V250_CTL_ERROR ? "ERROR" : "unknown",
             want_info ? ", " : "", want_info ? want_info : "");
    if (r != want || (want_info && strcmp(info, want_info)))
        printf("       got result %d info \"%s\"\n", r, info);
    check(r == want && (!want_info || !strcmp(info, want_info)), what);
}

static void test_defaults_and_reads(void)
{
    v250_ctl_t c;

    printf("defaults and reads:\n");
    v250_ctl_reset(&c);
    cmd(&c, "MR?", V250_CTL_OK, "+MR: 0");     /* 6.4.3 recommended default */
    cmd(&c, "ER?", V250_CTL_OK, "+ER: 0");
    cmd(&c, "DR?", V250_CTL_OK, "+DR: 0");
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,0,2");  /* 6.5.1 recommended defaults */
    cmd(&c, "DS?", V250_CTL_OK, "+DS: 3,0,1024,32");
    cmd(&c, "MR=?", V250_CTL_OK, "+MR: (0,1)");
    cmd(&c, "ER=?", V250_CTL_OK, "+ER: (0,1)");
    cmd(&c, "DR=?", V250_CTL_OK, "+DR: (0,1)");
    /* Test responses list only what is supported (5.4.4.4): no direct mode, no
     * alternative protocol; the V.42bis codec's own limits. */
    cmd(&c, "ES=?", V250_CTL_OK, "+ES: (1-3),(0,2-3),(1-2,4-5)");
    cmd(&c, "DS=?", V250_CTL_OK, "+DS: (0-3),(0,1),(512-65535),(6-250)");
    cmd(&c, "es?", V250_CTL_OK, "+ES: 3,0,2");
    cmd(&c, "ES44?", V250_CTL_UNKNOWN, NULL);   /* a longer name is not +ES */
    cmd(&c, "DS44=1", V250_CTL_UNKNOWN, NULL);
    cmd(&c, "EWINDX=1", V250_CTL_UNKNOWN, NULL);
}

/* 6.5.2, 6.5.4, 6.5.6-6.5.8: only what this DCE honours is accepted. */
static void test_error_control_parameters(void)
{
    v250_ctl_t c;
    int tk, rk, tn, rn;

    printf("+EB, +EFCS, +ETBM, +EWIND, +EFRAM:\n");
    v250_ctl_reset(&c);
    cmd(&c, "EB?", V250_CTL_OK, "+EB: 0,0,0");
    cmd(&c, "EB=?", V250_CTL_OK, "+EB: (0),(0),(0)");      /* a pty carries no break */
    cmd(&c, "EB=1", V250_CTL_ERROR, NULL);
    cmd(&c, "EB=0,0,0", V250_CTL_OK, NULL);
    cmd(&c, "EFCS?", V250_CTL_OK, "+EFCS: 0");
    cmd(&c, "EFCS=?", V250_CTL_OK, "+EFCS: (0)");          /* XID never offers 32-bit FCS */
    cmd(&c, "EFCS=1", V250_CTL_ERROR, NULL);
    cmd(&c, "ETBM?", V250_CTL_OK, "+ETBM: 0,1,20");
    cmd(&c, "ETBM=?", V250_CTL_OK, "+ETBM: (0),(1-2),(0-30)");
    cmd(&c, "ETBM=0,2,30", V250_CTL_OK, NULL);
    cmd(&c, "ETBM?", V250_CTL_OK, "+ETBM: 0,2,30");
    cmd(&c, "ETBM=1", V250_CTL_ERROR, NULL);               /* TD delivery not done */
    cmd(&c, "ETBM=,,31", V250_CTL_ERROR, NULL);
    cmd(&c, "ETBM?", V250_CTL_OK, "+ETBM: 0,2,30");        /* untouched by the ERRORs */
    cmd(&c, "EWIND?", V250_CTL_OK, "+EWIND: 15,0");
    cmd(&c, "EWIND=?", V250_CTL_OK, "+EWIND: (1-15),(0-15)");
    cmd(&c, "EWIND=4,7", V250_CTL_OK, NULL);
    cmd(&c, "EWIND?", V250_CTL_OK, "+EWIND: 4,7");
    cmd(&c, "EWIND=5", V250_CTL_OK, NULL);                 /* value2 omitted: as value1 */
    cmd(&c, "EWIND?", V250_CTL_OK, "+EWIND: 5,0");
    cmd(&c, "EWIND=0", V250_CTL_ERROR, NULL);
    cmd(&c, "EWIND=16", V250_CTL_ERROR, NULL);
    cmd(&c, "EWIND=,3", V250_CTL_ERROR, NULL);             /* value1 is not optional */
    cmd(&c, "EFRAM?", V250_CTL_OK, "+EFRAM: 128,0");
    cmd(&c, "EFRAM=?", V250_CTL_OK, "+EFRAM: (1-128),(0-128)");
    cmd(&c, "EFRAM=64,32", V250_CTL_OK, NULL);
    cmd(&c, "EFRAM=129", V250_CTL_ERROR, NULL);
    v250_ctl_link_params(&c, &tk, &rk, &tn, &rn);
    check(tk == 5 && rk == 5 && tn == 64 && rn == 32, "LAP.M parameters: value2 0 means value1");
    v250_ctl_reset(&c);
    v250_ctl_link_params(&c, &tk, &rk, &tn, &rn);
    check(tk == 15 && rk == 15 && tn == 128 && rn == 128, "reset restores the V.42 defaults");
}

static void test_set_and_store(void)
{
    v250_ctl_t c;

    printf("set stores, read reports (5.4.4.2, 5.4.4.3):\n");
    v250_ctl_reset(&c);
    cmd(&c, "MR=1", V250_CTL_OK, "");
    cmd(&c, "MR?", V250_CTL_OK, "+MR: 1");
    cmd(&c, "ER=1", V250_CTL_OK, "");
    cmd(&c, "ER?", V250_CTL_OK, "+ER: 1");
    cmd(&c, "DR=1", V250_CTL_OK, "");
    cmd(&c, "DR?", V250_CTL_OK, "+DR: 1");
    cmd(&c, "ES=2,3,5", V250_CTL_OK, "");
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 2,3,5");
    cmd(&c, "DS=1,1,4096,64", V250_CTL_OK, "");
    cmd(&c, "DS?", V250_CTL_OK, "+DS: 1,1,4096,64");
    check(c.es_set && c.ds_set, "the DTE's own settings are recorded as such");

    /* Omitted subparameters keep their value (5.4.2.3). */
    cmd(&c, "ES=3", V250_CTL_OK, "");
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,3,5");
    cmd(&c, "ES=,0", V250_CTL_OK, "");
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,0,5");
    cmd(&c, "ES=,,4", V250_CTL_OK, "");
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,0,4");
    cmd(&c, "DS=,,,100", V250_CTL_OK, "");
    cmd(&c, "DS?", V250_CTL_OK, "+DS: 1,1,4096,100");
    cmd(&c, "ES=", V250_CTL_OK, "");
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,0,4");
    cmd(&c, "MR=0", V250_CTL_OK, "");
    cmd(&c, "MR?", V250_CTL_OK, "+MR: 0");
}

static void test_rejection(void)
{
    v250_ctl_t c;

    printf("rejected commands leave every value unchanged (5.4.4.2):\n");
    v250_ctl_reset(&c);
    cmd(&c, "ES=3,2,5", V250_CTL_OK, "");
    /* One bad field poisons the whole command: the good first field must not
     * be stored. */
    cmd(&c, "ES=1,4,5", V250_CTL_ERROR, NULL);   /* 4: alternative only */
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,2,5");
    cmd(&c, "ES=0", V250_CTL_ERROR, NULL);       /* direct mode: not offered */
    cmd(&c, "ES=4", V250_CTL_ERROR, NULL);       /* alternative protocol */
    cmd(&c, "ES=3,1", V250_CTL_ERROR, NULL);     /* change DTE rate to line rate */
    cmd(&c, "ES=3,0,0", V250_CTL_ERROR, NULL);   /* ans: direct */
    cmd(&c, "ES=3,0,3", V250_CTL_ERROR, NULL);
    cmd(&c, "ES=3,0,6", V250_CTL_ERROR, NULL);   /* ans: alternative only */
    cmd(&c, "ES=3,0,7", V250_CTL_ERROR, NULL);
    cmd(&c, "ES=1,2,3,4", V250_CTL_ERROR, NULL);/* too many values */
    cmd(&c, "ES=a", V250_CTL_ERROR, NULL);       /* wrong type */
    cmd(&c, "ES=-1", V250_CTL_ERROR, NULL);
    cmd(&c, "ES=1 ", V250_CTL_ERROR, NULL);
    cmd(&c, "ES=1.0", V250_CTL_ERROR, NULL);
    cmd(&c, "ES=99999999999", V250_CTL_ERROR, NULL);
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,2,5");

    cmd(&c, "DS=2,1,2048,50", V250_CTL_OK, "");
    cmd(&c, "DS=4", V250_CTL_ERROR, NULL);
    cmd(&c, "DS=3,2", V250_CTL_ERROR, NULL);     /* table 27 has 0 and 1 */
    cmd(&c, "DS=3,0,511", V250_CTL_ERROR, NULL);
    cmd(&c, "DS=3,0,65536", V250_CTL_ERROR, NULL);
    cmd(&c, "DS=3,0,1024,5", V250_CTL_ERROR, NULL);
    cmd(&c, "DS=3,0,1024,251", V250_CTL_ERROR, NULL);
    cmd(&c, "DS=3,0,1024,32,1", V250_CTL_ERROR, NULL);
    cmd(&c, "DS=1,0,512,6", V250_CTL_OK, "");    /* the bounds themselves */
    cmd(&c, "DS=1,0,65535,250", V250_CTL_OK, "");
    cmd(&c, "DS?", V250_CTL_OK, "+DS: 1,0,65535,250");
    cmd(&c, "DS=3,1,9999,9", V250_CTL_OK, "");
    cmd(&c, "DS=2,0,1,1", V250_CTL_ERROR, NULL);
    cmd(&c, "DS?", V250_CTL_OK, "+DS: 3,1,9999,9");

    cmd(&c, "MR=2", V250_CTL_ERROR, NULL);
    cmd(&c, "ER=2", V250_CTL_ERROR, NULL);
    cmd(&c, "DR=2", V250_CTL_ERROR, NULL);
    cmd(&c, "MR=1,1", V250_CTL_ERROR, NULL);
    cmd(&c, "MR=x", V250_CTL_ERROR, NULL);
    cmd(&c, "MR?x", V250_CTL_ERROR, NULL);
    cmd(&c, "MR", V250_CTL_ERROR, NULL);
    cmd(&c, "MR?", V250_CTL_OK, "+MR: 0");

    /* ATZ / AT&F: everything back, including the "DTE set it" marks. */
    v250_ctl_reset(&c);
    check(!c.es_set && !c.ds_set, "reset clears the DTE-set marks");
    cmd(&c, "ES?", V250_CTL_OK, "+ES: 3,0,2");
}

static void test_ec_policy(void)
{
    v250_ctl_t c;
    v250_ec_policy_t p;

    printf("+ES as a policy for the call:\n");
    v250_ctl_reset(&c);
    v250_ctl_ec_policy(&c, true, &p);
    check(p.attempt && p.detect && !p.required, "originator default: V.42 with detection, optional");
    v250_ctl_ec_policy(&c, false, &p);
    check(p.attempt && p.detect && !p.required, "answerer default: V.42, optional");

    cmd(&c, "ES=1", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, true, &p);
    check(!p.attempt, "orig_rqst 1: buffered mode only, no V.42");
    cmd(&c, "ES=2", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, true, &p);
    check(p.attempt && !p.detect, "orig_rqst 2: V.42 without the detection phase");
    cmd(&c, "ES=3,2", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, true, &p);
    check(p.attempt && p.detect && p.required, "orig_fbk 2: error control required");
    cmd(&c, "ES=3,3", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, true, &p);
    check(p.required, "orig_fbk 3: LAPM required");
    cmd(&c, "ES=3,0", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, true, &p);
    check(!p.required, "orig_fbk 0: optional");

    cmd(&c, "ES=,,1", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, false, &p);
    check(!p.attempt, "ans_fbk 1: error control disabled");
    cmd(&c, "ES=,,4", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, false, &p);
    check(p.attempt && p.required, "ans_fbk 4: required");
    cmd(&c, "ES=,,5", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, false, &p);
    check(p.attempt && p.required, "ans_fbk 5: required, LAPM only");
    cmd(&c, "ES=,,2", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, false, &p);
    check(!p.required, "ans_fbk 2: optional");
    /* The originator's fields do not govern an answered call and vice versa. */
    cmd(&c, "ES=1,3,2", V250_CTL_OK, "");
    v250_ctl_ec_policy(&c, false, &p);
    check(p.attempt && !p.required, "answered call ignores orig_rqst/orig_fbk");
}

static void test_compression_policy(void)
{
    v250_ctl_t c;
    v250_compression_t k;

    printf("+DS as a policy for the call:\n");
    v250_ctl_reset(&c);
    v250_ctl_compression(&c, true, &k);
    check(k.enabled && k.p0 == 3 && k.p1 == 1024 && k.p2 == 32 && !k.required,
          "default: both directions, 1024 codewords, 32-octet strings");
    cmd(&c, "DS=0", V250_CTL_OK, "");
    v250_ctl_compression(&c, true, &k);
    check(!k.enabled && k.p0 == 0, "direction 0: no compression offered");
    /* Direction is from the DCE's own point of view; Annex A P0 bit 0 is
     * initiator -> responder, so the same request is a different P0 per role. */
    cmd(&c, "DS=1,1,2048,40", V250_CTL_OK, "");
    v250_ctl_compression(&c, true, &k);
    check(k.p0 == 1 && k.p1 == 2048 && k.p2 == 40 && k.required, "originator, transmit only: P0=1");
    v250_ctl_compression(&c, false, &k);
    check(k.p0 == 2, "answerer, transmit only: P0=2 (responder to initiator)");
    cmd(&c, "DS=2", V250_CTL_OK, "");
    v250_ctl_compression(&c, true, &k);
    check(k.p0 == 2, "originator, receive only: P0=2");
    v250_ctl_compression(&c, false, &k);
    check(k.p0 == 1, "answerer, receive only: P0=1");
    cmd(&c, "DS=3", V250_CTL_OK, "");
    v250_ctl_compression(&c, false, &k);
    check(k.p0 == 3, "both directions is P0=3 either way");

    cmd(&c, "DS=3", V250_CTL_OK, "");
    check(v250_ctl_compression_satisfied(&c, true, false)
          && v250_ctl_compression_satisfied(&c, false, true)
          && !v250_ctl_compression_satisfied(&c, false, false),
          "direction 3 (accept any direction): either way round satisfies it, none does not");
    cmd(&c, "DS=1", V250_CTL_OK, "");
    check(v250_ctl_compression_satisfied(&c, true, false)
          && !v250_ctl_compression_satisfied(&c, false, true), "direction 1 needs transmit");
    cmd(&c, "DS=2", V250_CTL_OK, "");
    check(!v250_ctl_compression_satisfied(&c, true, false)
          && v250_ctl_compression_satisfied(&c, false, true), "direction 2 needs receive");
}

static void report(const v250_ctl_t *c, const v250_connect_report_t *r, const char *want, const char *what)
{
    char out[300];

    v250_ctl_format_report(c, r, out, sizeof(out));
    if (strcmp(out, want))
        printf("       got \"%s\"\n", out);
    check(!strcmp(out, want), what);
}

static void test_reports(void)
{
    v250_ctl_t c;
    v250_connect_report_t r = { "V34", 28800, 0, "LAPM", 1, true, true, 0 };

    printf("connect reports (6.4.3, 6.5.5, 6.6.3):\n");
    v250_ctl_reset(&c);
    report(&c, &r, "", "all reporting off by default: nothing is sent");
    c.mr = 1;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n", "+MR=1: carrier then rate");
    r.rx_rate = 33600;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800,33600\r\n", "a distinct receive rate is reported as the second value");
    r.rx_rate = 28800;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n", "an equal receive rate is not repeated");
    r.tx_rate = 0;
    r.rx_rate = 0;
    report(&c, &r, "+MCR: V34\r\n+MRR: 0\r\n", "a failed negotiation reports rate 0");
    r.tx_rate = 28800;
    c.er = 1;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n+ER: LAPM\r\n", "+ER follows the modulation report");
    r.ec = "NONE";
    r.dc_scheme = 0;
    c.dr = 1;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n+ER: NONE\r\n+DR: NONE\r\n", "+DR follows +ER");
    r.ec = "LAPM";
    r.dc_scheme = 1;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n+ER: LAPM\r\n+DR: V42B\r\n", "V.42bis both directions");
    r.dc_rx = false;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n+ER: LAPM\r\n+DR: V42B TD\r\n", "transmit direction only");
    r.dc_tx = false;
    r.dc_rx = true;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n+ER: LAPM\r\n+DR: V42B RD\r\n", "receive direction only");
    r.dc_scheme = 2;
    r.dc_tx = r.dc_rx = true;
    report(&c, &r, "+MCR: V34\r\n+MRR: 28800\r\n+ER: LAPM\r\n+DR: V44\r\n", "V.44");
    c.mr = 0;
    report(&c, &r, "+ER: LAPM\r\n+DR: V44\r\n", "each report is independent of the others");
}

/* 6.2.10-6.2.13, 6.4.2, 6.4.8. */
static void test_interface_parameters(void)
{
    v250_ctl_t c;
    v250_connect_report_t r = { "V34", 33600, 0, "LAPM", 1, true, true, 57600 };
    char out[256];

    printf("+IPR, +ICF, +IFC, +ILRR, +MSC, +MA:\n");
    v250_ctl_reset(&c);
    cmd(&c, "IPR?", V250_CTL_OK, "+IPR: 0");               /* recommended: autodetect */
    cmd(&c, "IPR=?", V250_CTL_OK, "+IPR: (0,300,1200,2400,4800,9600,19200,38400,57600,115200,230400),()");
    cmd(&c, "IPR=9600", V250_CTL_OK, NULL);
    cmd(&c, "IPR?", V250_CTL_OK, "+IPR: 9600");
    cmd(&c, "IPR=9601", V250_CTL_ERROR, NULL);
    cmd(&c, "IPR?", V250_CTL_OK, "+IPR: 9600");
    cmd(&c, "ICF?", V250_CTL_OK, "+ICF: 3,3");
    cmd(&c, "ICF=?", V250_CTL_OK, "+ICF: (0,3),(0-3)");
    cmd(&c, "ICF=5", V250_CTL_ERROR, NULL);                /* 7 bits: a pty is 8-bit */
    cmd(&c, "ICF=0,1", V250_CTL_OK, NULL);
    cmd(&c, "IFC?", V250_CTL_OK, "+IFC: 2,2");
    cmd(&c, "IFC=?", V250_CTL_OK, "+IFC: (0,2),(0,2)");
    cmd(&c, "IFC=1,1", V250_CTL_ERROR, NULL);              /* no XON/XOFF */
    cmd(&c, "IFC=0,0", V250_CTL_OK, NULL);
    cmd(&c, "ILRR?", V250_CTL_OK, "+ILRR: 0");
    cmd(&c, "ILRR=1", V250_CTL_OK, NULL);
    cmd(&c, "MSC?", V250_CTL_OK, "+MSC: 1");               /* 6.4.8 recommended default */
    cmd(&c, "MSC=0", V250_CTL_OK, NULL);
    cmd(&c, "MSC=2", V250_CTL_ERROR, NULL);
    cmd(&c, "MA?", V250_CTL_ERROR, NULL);                  /* optional, not implemented */
    cmd(&c, "MA=V34", V250_CTL_ERROR, NULL);
    v250_ctl_format_report(&c, &r, out, sizeof(out));
    check(!strcmp(out, "+ILRR: 57600\r\n"), "+ILRR alone reports the DTE rate");
    c.mr = c.er = c.dr = 1;
    v250_ctl_format_report(&c, &r, out, sizeof(out));
    check(strstr(out, "+DR: V42B\r\n+ILRR: 57600\r\n") != NULL, "+ILRR follows +DR (6.2.13)");
}

int main(void)
{
    test_interface_parameters();
    test_error_control_parameters();
    test_defaults_and_reads();
    test_set_and_store();
    test_rejection();
    test_ec_policy();
    test_compression_policy();
    test_reports();
    printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures != 0;
}
