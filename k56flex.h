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

/* Repeated 16-bit response collector, Draft 0.23 receive boundary;
 * original resident 1D0E/1D27/1D9E. These are already-sliced dibits,
 * not PCM samples. Tap selection is supplied by the receiver mode. */
typedef struct {
    uint32_t hist;
    uint16_t window, word;
    unsigned tap;
    int remaining, accepted;
} k56flex_response_rx_t;
/* Returns -1 for an unsupported descrambler tap (only 5 and 18 exist). */
int k56flex_response_rx_init(k56flex_response_rx_t *rx, unsigned tap);
int k56flex_response_rx_dibit(k56flex_response_rx_t *rx, unsigned dibit);

/* Bank-8E E4A9 feedback mode only: original BC84/BC88/BC90 tables,
 * nearest-point 5903 and differential 5961, Draft 0.23 clause 11.
 * Inputs are equalized firmware coordinates, not linear PCM samples. */
unsigned k56flex_feedback_slice(int16_t real, int16_t imag);
unsigned k56flex_feedback_dibit(unsigned *previous_raw, unsigned raw);

/* Original 5A54 complex rotation, supplied signed Q15 phasor and bias.
 * Matches the verified OVM-clear arithmetic: rounding and modulo stores.
 * Coefficient generation/carrier recovery are separate stages. */
void k56flex_feedback_rotate(int16_t real, int16_t imag,
                             int16_t u, int16_t v, int16_t bias,
                             int16_t *out_real, int16_t *out_imag);

/* Original 6910 forward stage for the recovered 48-tap/two-row descriptor.
 * Input ring is in logical (not DSP bit-reversed) order. No samples consumed,
 * and no coefficient adaptation performed here. See Draft 0.23 clause 7.16. */
void k56flex_feedback_fir(const int16_t input[256], unsigned phase,
                          const int16_t coefficients[192], int16_t output[4]);

typedef struct {
    unsigned tap, history_index, remaining;
} k56flex_feedback_adapt_t;
/* 6910/6B54 sweep with descriptor m70=2 and m73=m74=0, clause 7.16.
 * History is caller-owned logical workspace. Return -1 without changes for
 * invalid state or the unresolved zero-spacing refresh (DSP repeat FFFF). */
int k56flex_feedback_adapt(k56flex_feedback_adapt_t *state,
                           int16_t coefficients[192], int16_t history[256],
                           unsigned spacing, unsigned wrap,
                           const int16_t input[256], unsigned input_phase,
                           const int16_t errors[128], unsigned error_phase);

typedef struct {
    unsigned source_cursor, output_cursor, output_available;
    int slip; /* -1, 0, +1, cleared after a successful block */
} k56flex_feedback_resample_t;
/* Original 51CE/4580 bounded ring/gain path, Draft 0.23 clause 7.17.
 * Raw ring words are the supplied firmware lane ring, not an asserted PCM ABI.
 * Return -1 without mutation for unsupported state or filter overflow. */
int k56flex_feedback_resample(k56flex_feedback_resample_t *state,
                              const int16_t raw[128], unsigned available,
                              unsigned table_phase, unsigned shift,
                              int16_t gain, int16_t bias, int16_t output[256]);

/* Original 45B8/460E/535B timing control, DP-11B register image.
 * Returns -1 without mutation when arithmetic leaves the verified nonsaturating
 * domain. Caller owns initialization and phase/slip bookkeeping (542D). */
int k56flex_feedback_timing(uint16_t state[128], const int16_t ring[256],
                            unsigned phase, unsigned tick);

/* Original 542D phase/correction gating, same DP-11B image as timing.
 * Keeps pending coarse status and clears correction/scratch on every path. */
void k56flex_feedback_phase(uint16_t state[128]);

/* 533F reset with BD01 (startup) or BD05 (symbol-loop) constants. */
void k56flex_feedback_timing_init(uint16_t state[128], int symbol_loop);
/* 6C50's four-SUBC block accumulator, DP119 image. Nonzero return gives
 * number of blocks, sets 8CAB and preserves the original four-bit quotient. */
unsigned k56flex_feedback_block_count(uint16_t state[128]);

/* Original 4AAA..4ABD input subtraction after predictor processing.
 * bypass corresponds to DM8F4F bit4. Inputs/outputs are signed lane words. */
void k56flex_feedback_residual(int16_t raw[2], const int16_t predicted[2], int bypass);

/* 4A91..4AF0 predictor mix, residual and conjugate error rotation;
 * optional diagnostic path 8CD8 is excluded. Supplied pairs and phasor. */
void k56flex_feedback_predictor(int16_t lane[2], const int16_t source[2],
                                const int16_t phasor[2], int bypass,
                                int16_t prediction[2], int16_t error[2]);

/* DAB7 forward predictor: logical 8192-word source ring, three complex
 * 48-tap rows, effective division by 2^18. No adaptation or input consumption. */
void k56flex_feedback_predictor_fir(const int16_t input[8192], unsigned phase,
                                    const int16_t coefficients[288], int16_t output[6]);

/* DAB7 6CF8/6ADD adaptation profile: update all 48 taps in three rows.
 * History is a retained PM window with refresh at index 256 and old prefix.
 * Supports tap 0 or 48, profile spacing 24 / wrap 95 / remaining 0 or 1. */
int k56flex_feedback_predictor_adapt(k56flex_feedback_adapt_t *state,
                                      int16_t coefficients[288], int16_t history[512],
                                      const int16_t input[8192], unsigned input_phase,
                                      const int16_t errors[128], unsigned error_phase);

/* Original 6D4E correlation-to-angle conversion with PM0320 polynomials.
 * Returns the signed high-word phase-error term consumed by 6D93. */
uint32_t k56flex_feedback_predictor_angle(const uint16_t correlation[4]);

/* Mode-15 4B59..4B86 correlation accumulator, D930..D933 high/low pairs.
 * Three supplied complex input/reference pairs; no angle conversion/reset. */
void k56flex_feedback_predictor_correlate(uint16_t correlation[4],
                                         const int16_t input[6], const int16_t reference[6]);

/* 6D93..6DA8 loop filter. DP119 words 33/34 measured phase error, 36/37 gains,
 * 38/39 integrator; returns the increment consumed by 4BB1. */
uint32_t k56flex_feedback_predictor_increment(uint16_t state[128]);

/* 4BB1..4BCD predictor phase advance and phasor lookup. DP119 state,
 * supplied post-loop increment and original 512-word cosine table. */
void k56flex_feedback_predictor_phase(uint16_t state[128], uint32_t increment,
                                      const int16_t cosine[512]);

/* Complete 4AF3 mode-15 controller, including enable/bypass, correlation,
 * timer, angle conversion, reset, loop filter and phasor production.
 * Supplied pairs/cosine table; caller must select firmware mode EE9B=15. */
void k56flex_feedback_predictor_control15(uint16_t state[128], uint16_t correlation[4],
                                          const int16_t input[6], const int16_t reference[6],
                                          const int16_t cosine[512]);

/* Complete 4AF3 controller for EE9B modes other than 15, PMST.TRM=1.
 * Supplied three complex pairs in logical order, including AR5 references. */
void k56flex_feedback_predictor_control(uint16_t state[128],
                                        const int16_t input[6], const int16_t reference[6],
                                        const int16_t cosine[512]);

/* Join the three-pair 6C50 processing body: optional active-profile adaptation,
 * forward FIR, predictor/residual/error and controller. Caller supplies source
 * and raw blocks, schedules cursors and applies firmware adaptation gates.
 * Does not run the capture callback or diagnostic output path. */
int k56flex_feedback_predictor_block(uint16_t state[128],
                                      k56flex_feedback_adapt_t *adaptation,
                                      int16_t coefficients[288], int16_t history[512],
                                      const int16_t source[8192], unsigned source_phase,
                                      int16_t raw[6], int16_t errors[128], unsigned error_phase,
                                      unsigned mode, int bypass, const int16_t cosine[512],
                                      uint16_t correlation[4], int16_t prediction[6]);

typedef struct {
    unsigned source, raw, error, output; /* logical word positions */
} k56flex_feedback_predictor_cursors_t;

/* 6C50 cadence and consecutive three-pair blocks, profile 44=2/45=3.
 * state[9] is available pairs, state[42] the retained remainder (0..2).
 * Source production and the optional capture callback are caller-owned; cursors
 * replace DSP addresses. Returns blocks processed, or -1 for unsupported state. */
int k56flex_feedback_predictor_run(uint16_t state[128],
                                    k56flex_feedback_predictor_cursors_t *cursors,
                                    k56flex_feedback_adapt_t *adaptation,
                                    int16_t coefficients[288], int16_t history[512],
                                    const int16_t source[8192], int16_t raw[128],
                                    int16_t errors[128], unsigned mode, int bypass,
                                    const int16_t cosine[512], uint16_t correlation[4]);

/* Original 4A6C source pair mix into DAB7's logical ring (clause 7.18).
 * Supplied symbols/carrier coefficients; updates only the source ring cursor. */
void k56flex_feedback_source_pair(int16_t ring[8192], unsigned *cursor,
                                  const int16_t pair[2], const int16_t phasor[2]);

/* Original 97A2/97A8 quadrant coding followed by 97C3's startup mapper,
 * DP118 and SPM=0. Word 7 is the nibble, 0C prior quadrant, 3E amplitude;
 * outputs 0F/10. Does not select amplitudes or dispatch transmit modes. */
/* Original 8F6F/8F74 word-history producer, DP118. */
void k56flex_feedback_startup_bits(uint16_t state[128], uint16_t input,
                                   int scramble);
/* Original BE68 alternate profile, including scratch word 12. */
void k56flex_feedback_extended_bits(uint16_t state[128], uint16_t input);
/* Bounded original 90FF extraction: width 1..15, remaining 0..15.
 * Producers 0=8F6F, 1=8F74, 2=BE68. Returns next-word consumption or -1. */
int k56flex_feedback_startup_take(uint16_t state[128], uint16_t input,
                                 unsigned producer);
/* Overlay 88 43D1 rotation; supplied PM phasor, bounded phase range. */
int k56flex_feedback_startup_rotate(uint16_t state[128], int16_t pair[2],
                                   const int16_t phasor[2]);
/* Original 9367 at SPM=0; normalized 64-word BR0 output ring. */
int k56flex_feedback_startup_output(const uint16_t state[128],
                                   const int16_t pair[2], int16_t ring[64],
                                   unsigned *cursor);
/* Original 8274..827E sample count, bounded SPM=0 operands; -1 invalid. */
int k56flex_feedback_startup_sample_count(uint16_t state[128], unsigned symbols);
/* Original 829E..82A3 at SPM=0/OVM=0; caller-selected PM coefficients. */
int k56flex_feedback_startup_fir(const int16_t history[64], unsigned cursor,
                                const int16_t *coefficients, unsigned taps,
                                int16_t *sample);
/* Original 828B..8299 phase/bank selection; normalized history cursor. */
int k56flex_feedback_startup_phase(uint16_t state[128], unsigned *cursor,
                                  uint16_t *bank_address);
/* Bounded 8270 datapath, phase-major caller-supplied coefficient banks.
 * Positive count or -1; normalized cursors, external output counter omitted. */
int k56flex_feedback_startup_samples(uint16_t state[128], unsigned symbols,
                                    const int16_t history[64], unsigned *history_cursor,
                                    const int16_t *banks, unsigned bank_words,
                                    int16_t output[128], unsigned *output_cursor);
void k56flex_feedback_startup_symbol(uint16_t state[128], int differential);

/* 9320 preset installation and 4A6C phase-cycle writer, DP118, SPM=1.
 * Original PM650C's eleven presets and PM6490 carrier pairs. Emit returns
 * 1 when the phase cycle resets, 0 otherwise, -1 for unsupported
 * phase/cursor. Symbols remain caller-supplied; no transmit-mode selection. */
int k56flex_feedback_source_init(uint16_t state[128], unsigned profile);
int k56flex_feedback_source_emit(uint16_t state[128], int16_t ring[8192],
                                  unsigned *cursor, const int16_t pair[2]);
/* Same writer with explicit C5x SPM 0/1/2 (product shifts 0/1/4).
 * SPM=0 is required when joining to the verified startup symbol mapper. */
int k56flex_feedback_source_emit_scaled(uint16_t state[128], int16_t ring[8192],
                                         unsigned *cursor, const int16_t pair[2], unsigned spm);

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
