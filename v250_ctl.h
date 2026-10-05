/*
 * v250_ctl.h -- V.250 parameters that configure the NEXT call's data protocol
 * stack, and the reports of what the last call settled on.
 *
 *   +MR   6.4.3  modulation reporting      (+MCR / +MRR)
 *   +ES   6.5.1  error control selection
 *   +ER   6.5.5  error control reporting   (+ER)
 *   +DS   6.6.1  V.42bis data compression
 *   +DR   6.6.3  data compression reporting (+DR)
 *
 * The interpreter hands each command here as text ("ES=3,0,2", "DS?", "ER=?").
 * V.250 5.4.4.2 governs the set command: every value is validated before any is
 * stored, so a rejected command leaves the previous setting untouched, and a
 * value outside what this DCE supports is an ERROR -- not an OK that discards
 * it.  The test form reports only values that are actually supported, which is
 * what makes "AT+ES=?" something a DTE can rely on.
 *
 * Omitted subparameters of a compound value keep their previous value
 * (5.4.2.3 / 5.4.4.2 allow either that or a fixed default; the definition of
 * these parameters gives only a recommended default per field, so retaining is
 * the least surprising).
 *
 * What is supported, and why the rest is not:
 *   +ES <orig_rqst> 1 buffered only, 2 V.42 without detection, 3 V.42 with
 *                     detection.  0 (direct mode) needs a DTE rate locked to the
 *                     line rate, which a PTY has no way to offer; 4 (alternative
 *                     protocol, MNP) is not implemented.
 *       <orig_fbk>  0 optional + buffered, 2 required (LAPM is the only protocol
 *                     there is, so "either" = "LAPM only"), 3 required LAPM.
 *                     1 (change the DTE rate, direct mode) and 4 (alternative
 *                     only) are not supported.
 *       <ans_fbk>   1 error control disabled, 2 optional + buffered, 4/5 required.
 *                     0 and 3 (direct) and 6 (alternative only) are not supported.
 *   +DS <direction> 0..3 (none / transmit only / receive only / both),
 *       <negotiation> 0 or 1, <max_dict> 512..65535, <max_string> 6..250 -- the
 *       limits of the V.42bis codec in use.
 */

#ifndef V250_CTL_H
#define V250_CTL_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    int mr;                 /* +MR: report +MCR/+MRR before CONNECT */
    int er;                 /* +ER: report +ER before CONNECT */
    int dr;                 /* +DR: report +DR before CONNECT */
    int es[3];              /* +ES: orig_rqst, orig_fbk, ans_fbk */
    int ds[4];              /* +DS: direction, negotiation, max_dict, max_string */
    bool es_set;            /* the DTE has issued +ES since the last reset */
    bool ds_set;            /* the DTE has issued +DS since the last reset */
} v250_ctl_t;

typedef enum {
    V250_CTL_OK = 0,        /* handled; the info text (maybe empty) precedes OK */
    V250_CTL_ERROR = -1,    /* handled and rejected */
    V250_CTL_UNKNOWN = 1    /* not one of these commands */
} v250_ctl_result_t;

/* Power-on / ATZ / AT&F: the V.250 recommended defaults (+ES 3,0,2; +MR/+ER/+DR
 * 0) and +DS 3,0,1024,32 -- the 1024 codewords and 32-octet strings this stack
 * has always offered, which 6.6.1 leaves to the manufacturer. */
void v250_ctl_reset(v250_ctl_t *c);

/* text is the command without the leading '+'.  Info text (no CR/LF, no OK) is
 * written to info for reads and tests. */
v250_ctl_result_t v250_ctl_command(v250_ctl_t *c, const char *text,
                                   char *info, size_t info_len);

/* What +ES asks of the error control layer for a call in this role. */
typedef struct {
    bool attempt;           /* run V.42 at all (false: buffered mode only) */
    bool detect;            /* V.42 detection phase (false: straight to SABME) */
    bool required;          /* no error control => disconnect, else fall back */
} v250_ec_policy_t;
void v250_ctl_ec_policy(const v250_ctl_t *c, bool calling_party,
                        v250_ec_policy_t *out);

/* What +DS asks of V.42bis.  p0 is the Annex A value, relative to the
 * initiator (bit 0: initiator to responder), as v42_set_compression() takes it. */
typedef struct {
    bool enabled;           /* offer compression at all */
    int p0;
    int p1;
    int p2;
    bool required;          /* +DS <negotiation>=1 */
    int direction;          /* +DS <direction>, from this DCE's own point of view */
} v250_compression_t;
void v250_ctl_compression(const v250_ctl_t *c, bool calling_party,
                          v250_compression_t *out);

/* Whether a negotiated outcome satisfies +DS <negotiation>=1: compression must
 * be in use in the direction(s) asked for (3 = both, "accept any direction",
 * is satisfied by compression in either).  tx/rx are this DCE's transmit and
 * receive compression as negotiated. */
bool v250_ctl_compression_satisfied(const v250_ctl_t *c, bool tx, bool rx);

/* Intermediate result code text, one line each, no CR/LF. */
typedef struct {
    const char *carrier;    /* +MCR, Table 13 */
    int tx_rate;            /* +MRR; 0 = negotiation failed */
    int rx_rate;            /* 0 = same as tx_rate, not reported separately */
    const char *ec;         /* +ER: "NONE", "LAPM" or "ALT" */
    int dc_scheme;          /* +DR: 0 none, 1 V.42bis, 2 V.44 */
    bool dc_tx;
    bool dc_rx;
} v250_connect_report_t;

/* Writes the lines the settings call for, in the order 6.4.3 / 6.5.5 / 6.6.3
 * fix: +MCR, +MRR, +ER, +DR.  Returns the number written (each ends in "\r\n"
 * so the caller can send the buffer verbatim). */
size_t v250_ctl_format_report(const v250_ctl_t *c, const v250_connect_report_t *r,
                              char *out, size_t out_len);

#endif /* V250_CTL_H */
