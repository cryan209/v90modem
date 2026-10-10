/* MICA C53 resident QAM engine, lifted instruction-for-instruction from the
 * original firmware (MICA Portware 2.7.3.0, flex.prog) and checked by the
 * tools/mica_qam_*_oracle.py fixtures against MicaEmu's isolated core.
 *
 * Receive: 1D0E/1D27/1D9E repeated-word collector, 5903/5961 slicing, 5A54
 * rotation, 6910 FIR and adaptation, 51CE/4580 resampler, 45B8/460E/535B
 * timing, the DAB7 predictor (4A8F/4AF3/6C50/6D4E). Transmit: 90FF, 8F6F/8F74/
 * BE68 producers, 97A2/97A8/97C3 mapper, overlay 88 43D1, 9367, 8270, 4A6C.
 *
 * These routines are shared by MICA's QAM modes. They were first lifted as
 * "K56flex" because Draft 0.23 (MicaEmu's reverse-engineered K56flex spec,
 * whose clause numbers the comments cite) recorded them; overlay 8E, which
 * drives most of the paths verified here, is MICA's V.32/V.32bis datapump
 * (V.32bis 1991: 2400 baud on 1800 Hz, rate signals R = 8990 and E sync 888F
 * packed LSB-first, GPA/GPC 5/23, 18/23). See docs/mica_qam_firmware.md.
 * Not linked into the modem. */
#ifndef MICA_QAM_H
#define MICA_QAM_H

#include <stddef.h>
#include <stdint.h>

/* Repeated 16-bit response collector, Draft 0.23 receive boundary;
 * original resident 1D0E/1D27/1D9E. These are already-sliced dibits,
 * not PCM samples. Tap selection is supplied by the receiver mode. */
typedef struct {
    uint32_t hist;
    uint16_t window, word;
    unsigned tap;
    int remaining, accepted;
} mica_qam_word_rx_t;
/* Returns -1 for an unsupported descrambler tap (only 5 and 18 exist). */
int mica_qam_word_rx_init(mica_qam_word_rx_t *rx, unsigned tap);
int mica_qam_word_rx_dibit(mica_qam_word_rx_t *rx, unsigned dibit);

/* Bank-8E E4A9 feedback mode only: original BC84/BC88/BC90 tables,
 * nearest-point 5903 and differential 5961, Draft 0.23 clause 11.
 * Inputs are equalized firmware coordinates, not linear PCM samples. */
unsigned mica_qam_slice(int16_t real, int16_t imag);
unsigned mica_qam_dibit(unsigned *previous_raw, unsigned raw);

/* Original 5A54 complex rotation, supplied signed Q15 phasor and bias.
 * Matches the verified OVM-clear arithmetic: rounding and modulo stores.
 * Coefficient generation/carrier recovery are separate stages. */
void mica_qam_rotate(int16_t real, int16_t imag,
                             int16_t u, int16_t v, int16_t bias,
                             int16_t *out_real, int16_t *out_imag);

/* Original 6910 forward stage for the recovered 48-tap/two-row descriptor.
 * Input ring is in logical (not DSP bit-reversed) order. No samples consumed,
 * and no coefficient adaptation performed here. See Draft 0.23 clause 7.16. */
void mica_qam_fir(const int16_t input[256], unsigned phase,
                          const int16_t coefficients[192], int16_t output[4]);

typedef struct {
    unsigned tap, history_index, remaining;
} mica_qam_adapt_t;
/* 6910/6B54 sweep with descriptor m70=2 and m73=m74=0, clause 7.16.
 * History is caller-owned logical workspace. Return -1 without changes for
 * invalid state or the unresolved zero-spacing refresh (DSP repeat FFFF). */
int mica_qam_adapt(mica_qam_adapt_t *state,
                           int16_t coefficients[192], int16_t history[256],
                           unsigned spacing, unsigned wrap,
                           const int16_t input[256], unsigned input_phase,
                           const int16_t errors[128], unsigned error_phase);

typedef struct {
    unsigned source_cursor, output_cursor, output_available;
    int slip; /* -1, 0, +1, cleared after a successful block */
} mica_qam_resample_t;
/* Original 51CE/4580 bounded ring/gain path, Draft 0.23 clause 7.17.
 * Raw ring words are the supplied firmware lane ring, not an asserted PCM ABI.
 * Return -1 without mutation for unsupported state or filter overflow. */
int mica_qam_resample(mica_qam_resample_t *state,
                              const int16_t raw[128], unsigned available,
                              unsigned table_phase, unsigned shift,
                              int16_t gain, int16_t bias, int16_t output[256]);

/* Original 45B8/460E/535B timing control, DP-11B register image.
 * Returns -1 without mutation when arithmetic leaves the verified nonsaturating
 * domain. Caller owns initialization and phase/slip bookkeeping (542D). */
int mica_qam_timing(uint16_t state[128], const int16_t ring[256],
                            unsigned phase, unsigned tick);

/* Original 542D phase/correction gating, same DP-11B image as timing.
 * Keeps pending coarse status and clears correction/scratch on every path. */
void mica_qam_phase(uint16_t state[128]);

/* 533F reset with BD01 (startup) or BD05 (symbol-loop) constants. */
void mica_qam_timing_init(uint16_t state[128], int symbol_loop);
/* 6C50's four-SUBC block accumulator, DP119 image. Nonzero return gives
 * number of blocks, sets 8CAB and preserves the original four-bit quotient. */
unsigned mica_qam_block_count(uint16_t state[128]);

/* Original 4AAA..4ABD input subtraction after predictor processing.
 * bypass corresponds to DM8F4F bit4. Inputs/outputs are signed lane words. */
void mica_qam_residual(int16_t raw[2], const int16_t predicted[2], int bypass);

/* 4A91..4AF0 predictor mix, residual and conjugate error rotation;
 * optional diagnostic path 8CD8 is excluded. Supplied pairs and phasor. */
void mica_qam_predictor(int16_t lane[2], const int16_t source[2],
                                const int16_t phasor[2], int bypass,
                                int16_t prediction[2], int16_t error[2]);

/* DAB7 forward predictor: logical 8192-word source ring, three complex
 * 48-tap rows, effective division by 2^18. No adaptation or input consumption. */
void mica_qam_predictor_fir(const int16_t input[8192], unsigned phase,
                                    const int16_t coefficients[288], int16_t output[6]);

/* DAB7 6CF8/6ADD adaptation profile: update all 48 taps in three rows.
 * History is a retained PM window with refresh at index 256 and old prefix.
 * Supports tap 0 or 48, profile spacing 24 / wrap 95 / remaining 0 or 1. */
int mica_qam_predictor_adapt(mica_qam_adapt_t *state,
                                      int16_t coefficients[288], int16_t history[512],
                                      const int16_t input[8192], unsigned input_phase,
                                      const int16_t errors[128], unsigned error_phase);

/* Original 6D4E correlation-to-angle conversion with PM0320 polynomials.
 * Returns the signed high-word phase-error term consumed by 6D93. */
uint32_t mica_qam_predictor_angle(const uint16_t correlation[4]);

/* Mode-15 4B59..4B86 correlation accumulator, D930..D933 high/low pairs.
 * Three supplied complex input/reference pairs; no angle conversion/reset. */
void mica_qam_predictor_correlate(uint16_t correlation[4],
                                         const int16_t input[6], const int16_t reference[6]);

/* 6D93..6DA8 loop filter. DP119 words 33/34 measured phase error, 36/37 gains,
 * 38/39 integrator; returns the increment consumed by 4BB1. */
uint32_t mica_qam_predictor_increment(uint16_t state[128]);

/* 4BB1..4BCD predictor phase advance and phasor lookup. DP119 state,
 * supplied post-loop increment and original 512-word cosine table. */
void mica_qam_predictor_phase(uint16_t state[128], uint32_t increment,
                                      const int16_t cosine[512]);

/* Complete 4AF3 mode-15 controller, including enable/bypass, correlation,
 * timer, angle conversion, reset, loop filter and phasor production.
 * Supplied pairs/cosine table; caller must select firmware mode EE9B=15. */
void mica_qam_predictor_control15(uint16_t state[128], uint16_t correlation[4],
                                          const int16_t input[6], const int16_t reference[6],
                                          const int16_t cosine[512]);

/* Complete 4AF3 controller for EE9B modes other than 15, PMST.TRM=1.
 * Supplied three complex pairs in logical order, including AR5 references. */
void mica_qam_predictor_control(uint16_t state[128],
                                        const int16_t input[6], const int16_t reference[6],
                                        const int16_t cosine[512]);

/* Join the three-pair 6C50 processing body: optional active-profile adaptation,
 * forward FIR, predictor/residual/error and controller. Caller supplies source
 * and raw blocks, schedules cursors and applies firmware adaptation gates.
 * Does not run the capture callback or diagnostic output path. */
int mica_qam_predictor_block(uint16_t state[128],
                                      mica_qam_adapt_t *adaptation,
                                      int16_t coefficients[288], int16_t history[512],
                                      const int16_t source[8192], unsigned source_phase,
                                      int16_t raw[6], int16_t errors[128], unsigned error_phase,
                                      unsigned mode, int bypass, const int16_t cosine[512],
                                      uint16_t correlation[4], int16_t prediction[6]);

typedef struct {
    unsigned source, raw, error, output; /* logical word positions */
} mica_qam_predictor_cursors_t;

/* 6C50 cadence and consecutive three-pair blocks, profile 44=2/45=3.
 * state[9] is available pairs, state[42] the retained remainder (0..2).
 * Source production and the optional capture callback are caller-owned; cursors
 * replace DSP addresses. Returns blocks processed, or -1 for unsupported state. */
int mica_qam_predictor_run(uint16_t state[128],
                                    mica_qam_predictor_cursors_t *cursors,
                                    mica_qam_adapt_t *adaptation,
                                    int16_t coefficients[288], int16_t history[512],
                                    const int16_t source[8192], int16_t raw[128],
                                    int16_t errors[128], unsigned mode, int bypass,
                                    const int16_t cosine[512], uint16_t correlation[4]);

/* Original 4A6C source pair mix into DAB7's logical ring (clause 7.18).
 * Supplied symbols/carrier coefficients; updates only the source ring cursor. */
void mica_qam_source_pair(int16_t ring[8192], unsigned *cursor,
                                  const int16_t pair[2], const int16_t phasor[2]);

/* Original 97A2/97A8 quadrant coding followed by 97C3's startup mapper,
 * DP118 and SPM=0. Word 7 is the nibble, 0C prior quadrant, 3E amplitude;
 * outputs 0F/10. Does not select amplitudes or dispatch transmit modes. */
/* Original 8F6F/8F74 word-history producer, DP118. */
void mica_qam_startup_bits(uint16_t state[128], uint16_t input,
                                   int scramble);
/* Original BE68 alternate profile, including scratch word 12. */
void mica_qam_extended_bits(uint16_t state[128], uint16_t input);
/* Bounded original 90FF extraction: width 1..15, remaining 0..15.
 * Producers 0=8F6F, 1=8F74, 2=BE68. Returns next-word consumption or -1. */
int mica_qam_startup_take(uint16_t state[128], uint16_t input,
                                 unsigned producer);
/* Overlay 88 43D1 rotation; supplied PM phasor, bounded phase range. */
int mica_qam_startup_rotate(uint16_t state[128], int16_t pair[2],
                                   const int16_t phasor[2]);
/* Original 9367 at SPM=0; normalized 64-word BR0 output ring. */
int mica_qam_startup_output(const uint16_t state[128],
                                   const int16_t pair[2], int16_t ring[64],
                                   unsigned *cursor);
/* Original 8274..827E sample count, bounded SPM=0 operands; -1 invalid. */
int mica_qam_startup_sample_count(uint16_t state[128], unsigned symbols);
/* Original 829E..82A3 at SPM=0/OVM=0; caller-selected PM coefficients. */
int mica_qam_startup_fir(const int16_t history[64], unsigned cursor,
                                const int16_t *coefficients, unsigned taps,
                                int16_t *sample);
/* Original 828B..8299 phase/bank selection; normalized history cursor. */
int mica_qam_startup_phase(uint16_t state[128], unsigned *cursor,
                                  uint16_t *bank_address);
/* Bounded 8270 datapath, phase-major caller-supplied coefficient banks.
 * Positive count or -1; normalized cursors, external output counter omitted. */
int mica_qam_startup_samples(uint16_t state[128], unsigned symbols,
                                    const int16_t history[64], unsigned *history_cursor,
                                    const int16_t *banks, unsigned bank_words,
                                    int16_t output[128], unsigned *output_cursor);
void mica_qam_startup_symbol(uint16_t state[128], int differential);
/* Overlay 8E (V.32bis) transmitter as that module configures it (
 * D698/D6D5/D886): 8F23 profile 0 (2400 baud, 10 samples per 3 symbols),
 * 94BA carrier select 1 (1800 Hz), 82B4 history clear, 83A3 overlay 0x11
 * banks. Per symbol: 90FF, 97A2|97A8, 97C3, 43D1, 9367, 8270 (one symbol).
 * Width, producer, mapping, amplitude (3E) and gain (11) are caller inputs:
 * the module's settings for them are not yet traced. 4A6C is not run. */
typedef struct {
    uint16_t state[128];        /* DP118 words; 13/2D/2E hold PM addresses */
    int16_t history[64];        /* 9367 ring, logical order */
    int16_t output[128];        /* 8270 ring, logical order */
    unsigned history_write, history_read, output_write;
    unsigned producer, differential;
    uint32_t samples;           /* the external counter at DM 8EEA */
} mica_v32bis_tx_t;
int mica_v32bis_tx_init(mica_v32bis_tx_t *tx, unsigned width,
                            unsigned producer, unsigned differential,
                            uint16_t amplitude, uint16_t gain);
/* One symbol: 3 or 4 samples into pcm[], or -1 (state unchanged). */
int mica_v32bis_tx_symbol(mica_v32bis_tx_t *tx, uint16_t word,
                              int16_t pcm[4], int *consumed);

/* 9320 preset installation and 4A6C phase-cycle writer, DP118, SPM=1.
 * Original PM650C's eleven presets and PM6490 carrier pairs. Emit returns
 * 1 when the phase cycle resets, 0 otherwise, -1 for unsupported
 * phase/cursor. Symbols remain caller-supplied; no transmit-mode selection. */
int mica_qam_source_init(uint16_t state[128], unsigned profile);
int mica_qam_source_emit(uint16_t state[128], int16_t ring[8192],
                                  unsigned *cursor, const int16_t pair[2]);
/* Same writer with explicit C5x SPM 0/1/2 (product shifts 0/1/4).
 * SPM=0 is required when joining to the verified startup symbol mapper. */
int mica_qam_source_emit_scaled(uint16_t state[128], int16_t ring[8192],
                                         unsigned *cursor, const int16_t pair[2], unsigned spm);

#endif
