/* K56flex client (analogue-side) receiver for our own server code.
 *
 * Ideal-channel, decision-level model: it takes the server's raw G.711 octets,
 * follows the training timeline, inverts each probe block back to source bits,
 * descrambles, decodes the parameter record and then reads data frames.  It is
 * not a demodulator: there is no gain control, equalizer or timing recovery
 * (the spec leaves those open) and it assumes it was listening from the start of
 * the server's stream.
 *
 * The upstream path (the signals that set the server's DM 8F31 bits 10..14 and
 * deliver the report) is not recovered, so it is a side channel here: the client
 * calls ctl->status() and ctl->report(), and the loopback test wires those
 * straight to the server.  Delivery must be immediate, because the client
 * mirrors the server's phase machine and both must see an event at the same
 * pair boundary.
 */
#ifndef K56FLEX_CLIENT_H
#define K56FLEX_CLIENT_H

#include "k56flex_rxfe.h"
#include "k56flex_train.h"

typedef struct {
    void (*status)(void *user, unsigned bits);          /* upstream: DM 8F31 bits */
    void (*report)(void *user, unsigned report_field);  /* upstream: accepted report (8FEF) */
    void (*data)(void *user, const uint8_t *bits, unsigned n);  /* descrambled data bits */
    void *user;
} k56flex_client_ctl_t;

typedef struct {
    k56flex_law_t law;
    uint8_t report_ext;          /* extension byte: RBS mask (bits 0-5), table selects (6, 7) */
    uint16_t report_word;        /* must satisfy k56flex_report_header_ok; bit 4 = pacing */
    unsigned gate_b_pairs;       /* silence pairs observed before signalling bit 10 */
    k56flex_client_ctl_t ctl;
} k56flex_client_cfg_t;

typedef struct {
    k56flex_client_cfg_t cfg;
    k56flex_train_t shadow;      /* phase machine only; its samples are used for verification */
    k56flex_probe_t cp;          /* client's own sequence/DC state from PARAM_1 on */
    int cp_valid;
    uint8_t pend[8];
    unsigned pend_n;
    uint8_t *y, *x;              /* received and descrambled source bits since PT_A */
    unsigned nbits, rec_end, param_from;
    int param_from_set;
    unsigned report_field;
    int report_sent, gate_a_sent, gate_b_sent, gate_c_sent, resp_sent, done_sent;
    int param_ok;
    k56flex_param_t param;       /* decoded record fields */
    int rate_bps;
    k56flex_pcm_rx_t rx;
    int rx_ready;
    unsigned prime_frames, prime_ones_bad, data_frames, data_bits_out;
    unsigned blocks_checked, blocks_mismatched, ambiguous_blocks;
    int failed;
    k56flex_rxfe_t *fe;          /* linear-audio front end, created by k56flex_client_rx_linear */
    int fe_started;
    int detect;                  /* linear path: phase changes are observed, not mirrored */
    float qy[16];                /* equalized symbols waiting for a block or pair */
    unsigned qn, sym_idx;
    uint64_t qfirst;             /* absolute front-end index of qy[0] */
    int seek, in_frames;
    int first_bad;               /* phase*100000 + pairs at the first mirror mismatch */
} k56flex_client_t;

k56flex_client_t *k56flex_client_new(const k56flex_client_cfg_t *cfg);
void k56flex_client_free(k56flex_client_t *c);
/* Feed received octets (any chunking). */
void k56flex_client_rx(k56flex_client_t *c, const uint8_t *octets, size_t n);
/* The same client over linear audio: 8 kHz samples as a client codec would deliver them.
 * The front end acquires timing from the P1 probe, so the client no longer has to be
 * listening from the first sample. */
void k56flex_client_rx_linear(k56flex_client_t *c, const int16_t *samples, size_t n);
k56flex_train_phase_t k56flex_client_phase(const k56flex_client_t *c);

#endif
