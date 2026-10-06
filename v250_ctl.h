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
    /* 6.5.2-6.5.8.  Only the values this DCE can honour are accepted; see the
     * test responses in v250_ctl.c. */
    int eb[3];              /* +EB: break selection, timed, default length */
    int efcs;               /* +EFCS: 16-bit FCS only */
    int etbm[3];            /* +ETBM: pending TD, pending RD, timer */
    int ewind[2];           /* +EWIND: transmit, receive (0 = as transmit) */
    int efram[2];           /* +EFRAM: transmit, receive (0 = as transmit) */
    /* 6.2.10-6.2.13 and 6.4.8.  The DTE port is a pty: it has whatever rate the
     * DTE gives it, 8-bit characters, and back-pressure for flow control. */
    int ipr;                /* +IPR: fixed DTE rate, 0 = what the DTE sets */
    int icf[2];             /* +ICF: format (0 auto, 3 8N1), parity */
    int ifc[2];             /* +IFC: DCE by DTE, DTE by DCE (0 none, 2 circuit) */
    int ilrr;               /* +ILRR: report the DTE rate before CONNECT */
    int msc;                /* +MSC: V.34 seamless rate change (11.6) */
    /* 6.6.2 +DS44: direction, negotiation, capability, codewords tx/rx,
     * string tx/rx, history tx/rx. */
    int ds44[9];
    /* Not V.250: the USRobotics Courier's &A, how much of the call the
     * CONNECT text names (0 nothing, 1 /ARQ, 2 + modulation, 3 + protocol).
     * Factory 0 (the Courier ships &A1) so CONNECT stays what V.250 says. */
    int arq;
    /* 6.8 V.92 controls.  Only what this DCE honours is accepted; see the
     * range table in v250_ctl.c and v250_ctl_reset() for the defaults that
     * differ from 6.8's. */
    int pcw;                /* +PCW: a second call arriving mid-call */
    int pmh;                /* +PMH: 0 modem-on-hold enabled, 1 disabled */
    int pmht;               /* +PMHT: 0 deny, 1-13 grant with V.92 Table 33's T1 */
    int pig;                /* +PIG: 0 PCM upstream enabled, 1 disabled */
    int pqc;                /* +PQC: short Phase 1/2 (3: both disabled) */
    int pss;                /* +PSS: 0 DCEs decide, 2 force the full startup */
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

/* LAP.M window (k) and N401 per direction from +EWIND/+EFRAM, a value2 of 0
 * meaning "as value1" (6.5.7, 6.5.8). */
void v250_ctl_link_params(const v250_ctl_t *c, int *tx_k, int *rx_k,
                          int *tx_n401, int *rx_n401);

/* text is the command without the leading '+'.  Info text (no CR/LF, no OK) is
 * written to info for reads and tests. */
v250_ctl_result_t v250_ctl_command(v250_ctl_t *c, const char *text,
                                   char *info, size_t info_len);

/* The Courier/Rockwell spellings of these parameters, as the interpreter
 * forwards them: "&M5", "\N3", "%C2" and so on (a letter and one number).
 *   &M0 / \N0       +ES=1,0,1   buffered, no error control
 *   &M4 / \N3       +ES=3,0,2   V.42, falling back to buffered (factory)
 *   &M5 / \N2       +ES=3,2,4   error control required
 *   \N4             +ES=3,3,5   LAPM required
 *   &K0 / %C0        +DS=0, +DS44=0          no compression
 *   &K1-3 / %C1-2    +DS=3                   V.42bis both ways
 *   &H0 / &H1        +IFC=,0 / +IFC=,2       DTE flow-controlled by the DCE
 *   &R1 / &R2        +IFC=0 / +IFC=2         DCE flow-controlled by the DTE
 *   &I0              software flow control off (it always is)
 *   &B0-2            serial rate fixed/variable: a pty has no rate (no effect)
 *   &A0-3            the CONNECT suffix (arq above)
 * What this DCE cannot do is ERROR, not an OK that changes nothing: &M1-3
 * (synchronous, obsolete), \N1 (direct mode), \N5 and %C3 (MNP), &H2/&H3,
 * &I1-5 (XON/XOFF), &R0 (CTS delayed after RTS: a pty has neither).
 * V250_CTL_UNKNOWN for anything else. */
v250_ctl_result_t v250_ctl_alias(v250_ctl_t *c, const char *text);

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

/* +DS44 as a V.44 offer: directions from this DCE's (and its DTE's) point of
 * view, bit 0 transmit, bit 1 receive -- V.44 Annex A's sense, unlike +DS's
 * P0.  enabled is false for <direction> 0; required for <negotiation> 1. */
typedef struct {
    bool enabled;
    bool required;
    int directions;
    int tx_codewords, rx_codewords;
    int tx_max_string, rx_max_string;
    int tx_history, rx_history;
} v250_v44_t;
void v250_ctl_v44(const v250_ctl_t *c, v250_v44_t *out);
bool v250_ctl_v44_satisfied(const v250_ctl_t *c, int scheme, bool tx, bool rx);

/* Intermediate result code text, one line each, no CR/LF. */
typedef struct {
    const char *carrier;    /* +MCR, Table 13 */
    int tx_rate;            /* +MRR; 0 = negotiation failed */
    int rx_rate;            /* 0 = same as tx_rate, not reported separately */
    const char *ec;         /* +ER: "NONE", "LAPM" or "ALT" */
    int dc_scheme;          /* +DR: 0 none, 1 V.42bis, 2 V.44 */
    bool dc_tx;
    bool dc_rx;
    int dte_rate;           /* +ILRR: the DTE-DCE rate; 0 = not reported */
    bool refused;           /* settled outside the +MS bounds: no CONNECT */
} v250_connect_report_t;

/* Writes the lines the settings call for, in the order 6.4.3 / 6.5.5 / 6.6.3
 * fix: +MCR, +MRR, +ER, +DR.  Returns the number written (each ends in "\r\n"
 * so the caller can send the buffer verbatim). */
size_t v250_ctl_format_report(const v250_ctl_t *c, const v250_connect_report_t *r,
                              char *out, size_t out_len);

/* The Courier &A suffix for CONNECT ("/ARQ/V34/LAPM/V42BIS"), empty under
 * &A0.  /ARQ and the protocol names appear only when error control is in
 * use, as on the Courier. */
void v250_ctl_connect_suffix(const v250_ctl_t *c, const v250_connect_report_t *r,
                             char *out, size_t out_len);

#endif /* V250_CTL_H */
