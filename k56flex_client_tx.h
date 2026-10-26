/* K56flex client (analogue-side) upstream transmitter, Draft 0.23 clause 11.
 *
 * A symbol source: one unit-RMS four-point (or PP) symbol per baud, for a
 * V.34 modulator to shape, pre-emphasise, scale and put on the carrier that
 * INFO1a selected (v34_tx_set_symbol_source()).  It sends what our server's
 * receiver (k56flex_upstream_pcm.c) and native HSF's transmitter agree on:
 *
 *   S, S-bar   V.34 10.1.3.7: S alternates point 0 and point 0 rotated 90
 *              degrees, S-bar is S rotated 180 degrees.
 *   PP         V.34 10.1.3.6: the 48-symbol sequence, repeated.
 *   TRN        V.34 10.1.3.8, four-point, not differential: scrambled ones,
 *              GPC 1 + x^-18 + x^-23 from a zero state.  Native HSF sends
 *              3910 symbols; our receiver needs only the first 512.
 *   report     the 24-bit record (extension octet, then the 16-bit report,
 *              LSB first), repeated, scrambled and differentially encoded
 *              with both histories continuing from TRN, as the receiver
 *              decodes them (resident PM 0294/02B0 labels).
 *   J'         one more record with 899F in the report word's place.
 *   TRN2       a second TRN from a fresh scrambler (original HSF 120342c0
 *              "resets for TRN"): native HSF sends 9614 symbols, and a server
 *              trains on it before it looks for MP.  Our own server never
 *              checked; the 7910 waits in its gate C without it.
 *   MP, MP'    V.34 10.1.3.9, Tables 20/21, with K56's rate fields: bits
 *              20:23 upstream rate/2400, bits 24:27 downstream rate/2000-15.
 *
 * Until k56flex_client_tx_start() it returns zero symbols (silence).  After
 * MP' it sends idle ones; B1 and data are not implemented here.
 */
#ifndef K56FLEX_CLIENT_TX_H
#define K56FLEX_CLIENT_TX_H

#include <stdint.h>

typedef struct {
    unsigned s_symbols;        /* 128 */
    unsigned sbar_symbols;     /* 16 */
    unsigned pp_symbols;       /* 288 */
    unsigned trn_symbols;      /* 3910, as native HSF */
    unsigned report_records;   /* 21, as native HSF */
    uint8_t ext;               /* report extension octet */
    uint16_t report;           /* must satisfy k56flex_report_header_ok() */
    unsigned gap_symbols;      /* second TRN, between J' and the first MP */
    unsigned mp_repeats;       /* identical MP frames; the receiver needs two */
    unsigned mp_ack_repeats;   /* MP' frames */
    unsigned mp_up_code;       /* upstream rate / 2400 */
    unsigned mp_down_code;     /* downstream rate / 2000 - 15 */
    unsigned mp_mask;          /* 15-bit rate mask, bits 35:49 */
    unsigned mp_asymmetric;    /* bit 50, set by native HSF */
    unsigned trn2_hold;        /* 1: after J', idle until k56flex_client_tx_release_trn2() */
} k56flex_client_tx_cfg_t;

typedef enum {
    K56CT_IDLE_SILENT = 0,     /* not started */
    K56CT_S,
    K56CT_SBAR,
    K56CT_PP,
    K56CT_TRN,
    K56CT_REPORT,
    K56CT_JPRIME,
    K56CT_GAP,                 /* the second TRN */
    K56CT_MP,
    K56CT_MP_ACK,
    K56CT_TAIL,                /* idle ones after MP' */
    K56CT_HOLD                 /* idle after J', waiting to start TRN2 (trn2_hold) */
} k56flex_client_tx_stage_t;

typedef struct {
    k56flex_client_tx_cfg_t cfg;
    k56flex_client_tx_stage_t stage;
    unsigned n;                /* symbols sent in this stage */
    unsigned trn_scr;          /* TRN scrambler history */
    uint32_t scr;              /* data scrambler history (transmitted bits) */
    unsigned prev;             /* differential: previous transmitted label */
    uint8_t bits[188];         /* frame being sent */
    unsigned nbits, bitpos, frames;
    int trn2_released;
    uint64_t symbols;          /* since start */
} k56flex_client_tx_t;

/* Fill cfg with native HSF's profile (report 00/89b0, MP 9600 up / 52000 down). */
void k56flex_client_tx_defaults(k56flex_client_tx_cfg_t *cfg);
int k56flex_client_tx_init(k56flex_client_tx_t *t, const k56flex_client_tx_cfg_t *cfg);
void k56flex_client_tx_start(k56flex_client_tx_t *t);
/* With cfg.trn2_hold: start the second TRN (now, or straight after J' if J'
 * is still to come).  Experiment: the server may want TRN2 against its own PT
 * stages rather than straight after J'. */
void k56flex_client_tx_release_trn2(k56flex_client_tx_t *t);
/* Next symbol, unit RMS.  Returns the stage it belongs to. */
k56flex_client_tx_stage_t k56flex_client_tx_symbol(k56flex_client_tx_t *t, float *re, float *im);
const char *k56flex_client_tx_stage_name(k56flex_client_tx_stage_t st);
/* The 88-bit type 0 MP (ack 0 or 1) as k56flex_client_tx sends it. */
void k56flex_client_tx_mp_bits(const k56flex_client_tx_cfg_t *cfg, int ack, uint8_t bits[88]);

#endif
