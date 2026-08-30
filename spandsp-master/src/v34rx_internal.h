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

/* V34_RX_STAGE_DATA, lifted to v34rx_data.c.  Takes nothing from the
   enclosing switch: the stage was already self-contained in (s, sym). */
void  v34_rx_data_symbol(v34_rx_state_t *s, const complexf_t *sym);

#endif
