/* K56flex downstream payload core, V.8bis framing, parameter record and report.
 *
 * Independent implementation of behaviour recovered from MICA Portware 2.7.3.0
 * (K56flex Technical Specification Draft 0.23, clauses 4.4-4.6, 4.12, 7.1-7.12).
 * No vendor source; the shipped level payloads in k56flex_tables.h are fixed data.
 *
 * This is the digital-side (server) transmit core plus its exact inverse.  It
 * is not wired into a live call: Phase 1/2 line signalling, probing, the
 * receive front end and network RBS phase are not established by the spec.
 */
#ifndef K56FLEX_H
#define K56FLEX_H

#include <stddef.h>
#include <stdint.h>

/* Value is DM EC6E bit 0 and the directory offset (+52) of the level tables.
 * The shipped tables selected by 0 hold only mu-law codeword magnitudes and
 * those selected by 1 only A-law magnitudes (checked by k56flex_test); Draft
 * 0.23 labels them the other way round. */
typedef enum { K56FLEX_LAW_MU = 0, K56FLEX_LAW_A = 1 } k56flex_law_t;

#define K56FLEX_RATE_MIN_BPS 32000
#define K56FLEX_RATE_MAX_BPS 56000
#define K56FLEX_FRAME_SAMPLES 8
#define K56FLEX_MAX_FRAME_BITS 64

/* ---- G.711 codeword <-> signed level word ------------------------------ */
/* Level words are exact codeword magnitudes in the chosen law.  Returns the
 * octet, or -1 if `level` is not a codeword value of that law.  Level 0 (the
 * silence stage) is 0xff in mu-law and the idle code 0xd5 in A-law. */
int k56flex_g711_from_level(k56flex_law_t law, int level);
int k56flex_level_from_g711(k56flex_law_t law, uint8_t octet);

/* ---- downstream mapper ------------------------------------------------- */
typedef struct {
    k56flex_law_t law;
    int rate_bps;          /* 32000..56000 in 2000 steps (58/60k only unadjusted) */
    unsigned report_field; /* DM 8FEF as built by k56flex_report_field(); 0 = no report */
} k56flex_pcm_config_t;

typedef int (*k56flex_source_fn)(void *user, uint16_t *word);

typedef struct {
    const void *table;               /* k56flex_table_t, set by init */
    uint8_t law;
    uint8_t n;                       /* alphabet size - 1 */
    uint8_t width;                   /* extra index bits per sample */
    uint8_t total, amp_bits, alloc_sel;
    uint8_t phase;                   /* amplitude budget phase, toggles when alloc_sel & 4 */
    uint8_t mask;                    /* six-position RBS mask, rotates per sample */
    uint8_t selector;                /* effective impairment selector */
    uint8_t mask_field;
    uint8_t rate_index;
    unsigned frame;                  /* frame counter, selects the 1/2/4 reduction cycle */
    uint32_t scrambler;              /* y[n-1] in bit 0 */
    int32_t dc;                      /* signed running sum of emitted levels */
    uint64_t c[3][130];              /* C2, C4, C8 bounded-composition counts */
    uint64_t cum[130];               /* exclusive cumulative C8 */
    uint8_t q[192];                  /* scrambled bits waiting to be consumed */
    unsigned q_head, q_count;
    k56flex_source_fn source;
    void *source_user;
    int16_t out[K56FLEX_FRAME_SAMPLES];
    unsigned out_pos;                /* samples of `out` already emitted */
} k56flex_pcm_tx_t;

/* Returns 0 or -1 for an unsupported law/rate/report combination. */
int k56flex_pcm_tx_init(k56flex_pcm_tx_t *tx, const k56flex_pcm_config_t *cfg,
                        k56flex_source_fn source, void *user);
/* One eight-sample (1 ms) frame of signed level words.  Returns 1, 0 if the
 * source paused (no state consumed beyond buffered bits), -1 on error. */
int k56flex_pcm_tx_frame(k56flex_pcm_tx_t *tx, int16_t out[K56FLEX_FRAME_SAMPLES]);
/* Raw G.711 octets at 8000/s.  May return fewer than n if the source pauses. */
size_t k56flex_pcm_tx_g711(k56flex_pcm_tx_t *tx, uint8_t *out, size_t n);
/* The signed level words this configuration can emit (both signs of the shipped table),
 * for a receiver's slicer.  Returns the count written to out (at most `max`). */
unsigned k56flex_pcm_levels(const k56flex_pcm_tx_t *tx, int16_t *out, unsigned max);
/* Bits consumed per eight-sample frame, 32..56 at the documented rates. */
unsigned k56flex_pcm_frame_bits(const k56flex_pcm_config_t *cfg);

typedef struct {
    k56flex_pcm_tx_t m;              /* shares geometry with the transmitter */
    uint32_t descrambler;            /* received y history */
} k56flex_pcm_rx_t;

int k56flex_pcm_rx_init(k56flex_pcm_rx_t *rx, const k56flex_pcm_config_t *cfg);
/* Exact inverse of one frame: eight G.711 octets -> descrambled source bits
 * (LSB-first order, one per byte).  Returns the bit count, or -1 for a
 * non-codeword, an inconsistent derived sign, or a rank outside the frame's
 * power-of-two subset. */
int k56flex_pcm_rx_frame(k56flex_pcm_rx_t *rx, const uint8_t octets[K56FLEX_FRAME_SAMPLES],
                         uint8_t bits[K56FLEX_MAX_FRAME_BITS]);

/* ---- accepted report (clause 7.9, 7.12) -------------------------------- */
/* 1521: DM 8FEF = ext | popcount(ext & 3F) << 9 | report bit 4 << 8. */
unsigned k56flex_report_field(uint8_t ext, uint16_t report);
/* First predicate of collector 1D5A: (report & 888F) == 8880. */
int k56flex_report_header_ok(uint16_t report);
/* 24-bit record: eight extension bits then sixteen report bits, LSB first. */
uint32_t k56flex_report_record(uint8_t ext, uint16_t report);
/* Scramble a record stream: y[n] = x[n] ^ y[n-5] ^ y[n-23], zero history. */
void k56flex_report_scramble(const uint8_t *in, uint8_t *out, size_t nbits);

typedef struct {
    unsigned tap;            /* 5 (responding role) or 18 */
    uint32_t hist;           /* received scrambled bits */
    uint8_t window[96];      /* last 96 descrambled bits, ring */
    unsigned pos, count;
    int accepted;
    uint8_t ext;
    uint16_t report;
} k56flex_report_rx_t;
void k56flex_report_rx_init(k56flex_report_rx_t *rx, unsigned tap);
/* Push one received (scrambled) bit; returns 1 once three identical 24-bit
 * records with a valid header have been seen at some alignment. */
int k56flex_report_rx_bit(k56flex_report_rx_t *rx, int bit);

/* ---- V.8bis capability frame (clauses 4.4-4.6) ------------------------- */
typedef enum {
    K56FLEX_V8BIS_MS = 0, K56FLEX_V8BIS_CL, K56FLEX_V8BIS_ACK1, K56FLEX_V8BIS_NAK1
} k56flex_v8bis_msg_t;

/* Payload octets (no flags/FCS).  `v90_capable` is DM EEAC bit 9 (octet 12 =
 * 0x83), `mu_law` is DM EC6E bit 0 (octet 15 gains 0x20).  Returns length. */
size_t k56flex_v8bis_payload(k56flex_v8bis_msg_t type, int v90_capable, int mu_law,
                             uint8_t out[16]);
uint16_t k56flex_v8bis_fcs(const uint8_t *data, size_t len);
/* 3 x 7E, payload, FCS low then high, 2 x 7E.  Returns octet count (len+7). */
size_t k56flex_v8bis_frame(const uint8_t *payload, size_t len, uint8_t *out);
/* LSB-first bit sequence with the MICA stuffing rule; literal 7E bypasses
 * insertion.  Returns the number of bits written (<= 8*len + len). */
size_t k56flex_v8bis_stuff(const uint8_t *octets, size_t len, uint8_t *bits);

/* ---- parameter record (clause 4.12) ------------------------------------ */
typedef struct {
    unsigned mode;          /* DM 8C76: 0 short record, 1 long record */
    unsigned extra;         /* DM 8C77, placed at bit 2 unless suppress_extra */
    unsigned rate;          /* DM 8C78 = rate index - 18 */
    int suppress_extra;     /* DM 8F4F bit 4 */
    unsigned u, v;          /* DM 8FD0 (bit 14), 8FD1 (bit 13) */
    unsigned final;         /* call argument, bit 15 of word 1 */
    unsigned control;       /* DM 8FBA */
    unsigned bit;           /* DM 8FBC, bit 15 of word 2 */
    int special;            /* DM 8FAA bit 0: word 2 low field forced to 0FFF */
} k56flex_param_t;

/* Raw words (3 for mode 0, 9 for mode 1); returns the count. */
size_t k56flex_param_raw(const k56flex_param_t *p, uint16_t raw[9]);
/* Reflected 8408, init FFFF, bit 0 first, no final XOR (residue 0000). */
uint16_t k56flex_param_crc(const uint16_t *words, size_t n);
/* Source words DA56 queues: FFFF prefix, then marker/separator framing,
 * zero padded to 6 (mode 0) or 12 (mode 1) words.  Returns that count. */
size_t k56flex_param_source(const k56flex_param_t *p, uint16_t out[12]);
/* Inverse: validates framing and checksum, returns the raw word count or -1. */
int k56flex_param_parse(unsigned mode, const uint16_t *source, uint16_t raw[9]);

#endif
