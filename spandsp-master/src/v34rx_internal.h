/* Internal interface between v34rx.c and the receive stages split out of it.
 *
 * v34rx.c grew to 17,000 lines with a single 4,900-line function at its
 * centre.  Stages are being lifted out of that switch one at a time; this
 * header carries the few file-scope helpers a lifted stage still needs, so
 * the split is a real translation unit rather than a textual include.
 *
 * Not a public spandsp interface: nothing here is SPAN_DECLARE and nothing
 * outside src/ should include it.
 */
#ifndef V34RX_INTERNAL_H
#define V34RX_INTERNAL_H

#include "spandsp/private/v34.h"

/* Data-mode receive-SNR -> bit-rate projection, calibrated on the d-modem
   rig (docs/v34_data_mode_rates.md).  bits = (snr_db + OFFSET)/SLOPE. */
#define V34_DATA_SNR_RATE_OFFSET_DB     13.0
#define V34_DATA_SNR_RATE_SLOPE_DB      6.0

/* Defined in v34rx.c; not in the public spandsp/v34.h. */
SPAN_DECLARE(void) v34_put_mapping_frame(v34_rx_state_t *s, int16_t bits[16]);

/* Shared by the lifted stages. */
bool  v34_rx_t2_data_path(const v34_rx_state_t *s);
void  v34_rx_tune_equalizer(v34_rx_state_t *s, const complexf_t *z, const complexf_t *target);
void  v34_rx_quantize_n_ways(complexi16_t xy[], complexi16_t *yt);
void  v34_rx_pack_output_bitstream(v34_rx_state_t *s);

/* Data-mode tuning knobs, each caching its own getenv. */
int   v34_rx_data_mode_eq_enabled(void);
float v34_rx_data_mode_eq_step(void);
float v34_rx_data_mode_freq_gain(void);
float v34_rx_data_mode_decision_gate(void);
int   v34_rx_gain_sweep_enabled(void);

/* Shared constants and the diagnostics gate.  MP_HYPOTHESIS_COUNT is the size
   of map_phase4_raw_bits()'s transform table; V34_TRACE_DIAGNOSTICS caches its
   getenv so the per-symbol paths pay nothing for it. */
#define MP_HYPOTHESIS_COUNT             24
#define MP_HYPOTHESIS_DIFF_INVERSE      8
/* NOTE: do NOT #define V34_TRACE_DIAGNOSTICS here.  v34rx.c wraps a large
   block of its own helpers in `#ifndef V34_TRACE_DIAGNOSTICS`, and this header
   is included ABOVE it -- defining the macro here compiles that entire block
   out.  Declare the function; let each .c define the macro for itself. */
int v34_rx_trace_diagnostics(void);

/* Phase 3 S/J detector constants, shared with v34rx_phase3.c.

   Sustained-rotation S detection: a +/-90 degrees/symbol rotation shows as one
   dominant differential dibit filling the 32-baud window.  DOMINANT_MIN sits
   well above scrambled Ja (~11/32) and below a real S (~30/32);
   DOMINANT_STABLE (bauds held) sits above Ja's longest run (~10) and below the
   128T (~128 baud) S signal, so it is Ja-safe with margin.

   DOMINANT_RUN_MAX is how long is too long.  The sustained-rotation detector
   false-fires on any steady single-frequency input, because a constant
   differentially demodulates to a constant dibit -- A-law digital silence,
   which decodes to +/-8 rather than to zero, is exactly such an input.  Raw
   power does not separate the cases, but *duration* does, and for free:
   10.1.3.7 makes S 128T, so a dominant run an order of magnitude longer than
   that cannot be S however strong it is.  Observed on the 2743 A-law duplex
   row, a false detection carried a run of 1326 windows where a real one
   carries at most about 128. */
#define PHASE3_J_PROGRESS_LOG_INTERVAL  32
#define PHASE3_S_BAUD_LOG_INTERVAL      1000
#define PHASE3_S_ALTERNATING_MIN        24
#define PHASE3_S_STABLE_WINDOWS         32
#define PHASE3_S_DOMINANT_MIN           24
#define PHASE3_S_DOMINANT_STABLE        48
#define PHASE3_S_DOMINANT_RUN_MAX       256

/* Shared with v34rx_phase3.c (V34_RX_STAGE_PHASE3_WAIT_S). */
int      v34_rx_descramble_reg(uint32_t *reg, int scrambler_tap, int in_bit);
int      v34_rx_map_phase4_raw_bits(int dibit, int hypothesis);
void     v34_rx_bits16_to_str(uint16_t v, char out[17]);
int      v34_rx_phase3_tracking_enabled(void);
int      v34_rx_phase3_j_pattern_bit(int pat_type, int bit_idx);
uint16_t v34_rx_j_ordered16(uint16_t rx_recent16, int total_bits, int phase);
int      v34_rx_j_hint_enabled(void);

/* V34_RX_STAGE_PHASE3_WAIT_S, lifted to v34rx_phase3.c.  Like the DATA stage
   it takes nothing from the enclosing switch. */
void     v34_rx_phase3_wait_s_symbol(v34_rx_state_t *s, const complexf_t *sym);

/* V34_RX_STAGE_DATA, lifted to v34rx_data.c.  Takes nothing from the
   enclosing switch: the stage was already self-contained in (s, sym). */
void  v34_rx_data_symbol(v34_rx_state_t *s, const complexf_t *sym);

#endif
