/* K56flex client (analogue-side) receiver for our own server code.
 *
 * Ideal-channel, decision-level model: it takes the server's raw G.711 octets,
 * follows the training timeline, inverts each probe block back to source bits,
 * descrambles, decodes the parameter record and then reads data frames.  It is
 * not a demodulator: there is no gain control, equalizer or timing recovery
 * (the spec leaves those open) and it assumes it was listening from the start of
 * the server's stream.
 *
 * This receiver has no upstream signalling transmitter. Peer control must
 * cross the audio/G.711 boundary (Draft 0.23 clauses 4.10-4.12 and 11);
 * it cannot release a server gate or deliver a report through host callbacks.
 * The raw-octet path keeps a local timeline model for decoder diagnostics;
 * that model is not evidence of a completed duplex handshake.
 */
#ifndef K56FLEX_CLIENT_H
#define K56FLEX_CLIENT_H

#include "k56flex_rxfe.h"
#include "k56flex_train.h"

#define K56FLEX_BLIND_RATES 15     /* 32000..60000 in 2000 steps */

typedef struct {
    void (*data)(void *user, const uint8_t *bits, unsigned n);  /* descrambled data bits */
    void *user;
} k56flex_client_ctl_t;

typedef struct {
    k56flex_law_t law;
    uint8_t report_ext;          /* extension byte: RBS mask (bits 0-5), table selects (6, 7) */
    uint16_t report_word;        /* must satisfy k56flex_report_header_ok; bit 4 = pacing */
    unsigned gate_b_pairs;       /* silence pairs observed before signalling bit 10 */
    k56flex_client_ctl_t ctl;
    int codec_identity;          /* linear path from G.711 octets: see k56flex_rxfe_set_codec_identity */
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
    int report_modeled, gate_a_modeled, gate_b_modeled, gate_c_modeled, response_modeled, final_param_seen;
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
    int input_silent;            /* linear path: the last received sample was codec silence */
    int input_static;            /* linear path: the last received sample repeated the one before */
    int16_t last_input;
    float prev_y;                /* last equalized symbol handed to the training path */
    unsigned magnitude_blocks;   /* blocks inverted on the flipped samples' magnitudes (7910) */
    unsigned abs_blocks;         /* blocks inverted on every sample's magnitude (7910 PT_C on) */
    float pt_b_level;            /* the PT_B magnitude, to find where PT_C starts */
    uint8_t prime_oct[6][8];     /* linear path: priming frames held until the rate is known */
    unsigned prime_buffered, rate_candidates;
    int rate_selected;
    /* Blind priming (the 7910): its stage boundaries are not on our pair
       grid and its record's rate field is not the mapper rate, so priming is
       found by its content, six frames that descramble to ones. */
    int blind;
    int16_t blind_lv[K56FLEX_BLIND_RATES][256];     /* each rate's signed levels */
    unsigned blind_nl[K56FLEX_BLIND_RATES];
    unsigned blind_run[K56FLEX_BLIND_RATES];        /* consecutive symbols near that table */
    uint8_t blind_ring[K56FLEX_BLIND_RATES][48];    /* the last 48 symbols sliced to it */
    k56flex_pcm_rx_t blind_rx[K56FLEX_BLIND_RATES]; /* each rate's receiver, initialised once */
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
