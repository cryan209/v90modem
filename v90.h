/*
 * v90.h — V.90 digital modem module
 *
 * Implements the digital (server) side of ITU-T V.90:
 *   - Downstream: PCM codeword injection into G.711 stream (up to 56 kbps)
 *   - Upstream:   V.34 demodulation (handled by SpanDSP V.34 RX)
 *   - Phase 2:    V.34-based INFO exchange with V.90-specific INFO0d/INFO1d
 *   - Phase 3:    Digital modem TX as raw PCM codewords (DIL, Jd, Sd, TRN1d)
 *   - Phase 4:    V.90-specific MP/CP exchange
 */

#ifndef V90_H
#define V90_H

#include <spandsp.h>
#include <stdint.h>
#include <stdbool.h>

#include "vpcm_cp.h"
#include "v92_phase4_decode.h"

/* V.90 downstream encoder state */
typedef struct v90_state_s v90_state_t;

/*
 * INFO0d bits 33:37, the maximum digital modem transmit power, in -0.5 dBm0
 * steps (code 0 = -0.5 dBm0, code 31 = -16 dBm0).  8.5.2/Table 15 makes this
 * the ceiling the analogue modem must design its constellations against, so
 * the number the transmitter announces and the number the Phase 4 mapper
 * enforces have to be the same one.
 *
 * KEEP IN SYNC with the literal in prepare_info0d()'s "33:37" bitstream_put
 * in spandsp-master/src/v34tx.c -- that file is not on this header's include
 * path, so the compiler cannot check it for you.
 */
#define V90_INFO0D_MAX_POWER_CODE 25    /* -13 dBm0 */

#define V90_DIL_MAX_PAT_BITS 128
#define V90_DIL_MAX_SEGMENTS 255

typedef struct {
    uint8_t n;
    uint8_t lsp;
    uint8_t ltp;
    uint8_t sp[128];
    uint8_t tp[128];
    uint8_t h[8];
    uint8_t ref[8];
    uint8_t train_u[255];
} v90_dil_desc_t;

typedef struct {
    uint8_t n;
    uint8_t lsp;
    uint8_t ltp;
    uint8_t unique_train_u;
    uint8_t used_uchords;
    uint8_t non_default_refs;
    uint8_t non_default_h;
    uint8_t impairment_score;
    bool looks_default_125x12;
    bool robbed_bit_limited;
    bool echo_limited;
    uint8_t recommended_downstream_drn;
    uint8_t recommended_upstream_drn;
} v90_dil_analysis_t;

#define V90_INFO_FILL_AND_SYNC_BITS  0x4EF
#define V90_INFO0A_BITS              49
#define V90_INFO1A_BITS              70

typedef struct {
    bool support_2743;
    bool support_2800;
    bool support_3429;
    bool support_3000_low;
    bool support_3000_high;
    bool support_3200_low;
    bool support_3200_high;
    bool rate_3429_allowed;
    bool support_power_reduction;
    uint8_t max_baud_rate_difference;
    bool from_cme_modem;
    bool support_1664_point_constellation;
    uint8_t tx_clock_source;
    bool acknowledge_info0d;
} v90_info0a_t;

typedef struct {
    uint8_t md;
    uint8_t u_info;
    uint8_t upstream_symbol_rate_code;
    uint8_t downstream_rate_code;
    int16_t freq_offset;
} v90_info1a_t;

typedef struct {
    v90_info0a_t frame;
    uint8_t bits[V90_INFO0A_BITS];
    uint16_t crc_field;
    uint16_t crc_remainder;
    bool fill_and_sync_ok;
    bool valid;
} v90_info0a_diag_t;

typedef struct {
    v90_info1a_t frame;
    uint8_t bits[V90_INFO1A_BITS];
    uint16_t crc_field;
    uint16_t crc_remainder;
    bool fill_and_sync_ok;
    bool valid;
} v90_info1a_diag_t;

/* PCM law selection */
typedef enum {
    V90_LAW_ULAW = 0,
    V90_LAW_ALAW = 1
} v90_law_t;

/* Strict receiver events used to drive live V.90 startup. Diagnostic guesses
 * and local transmitter stages must not be translated into these values. */
typedef enum {
    V90_RX_EVENT_NONE = 0,
    V90_RX_EVENT_INFO1A_VALID,
    V90_RX_EVENT_INFO1A_INVALID,
    V90_RX_EVENT_S,
    V90_RX_EVENT_TRN_LOCK,
    V90_RX_EVENT_J,
    V90_RX_EVENT_J_PRIME,
    V90_RX_EVENT_SU,
    V90_RX_EVENT_SU_BAR,
    V90_RX_EVENT_SU_FINAL,
    V90_RX_EVENT_CP_VALID,
    V90_RX_EVENT_CP_INVALID,
    V90_RX_EVENT_E,
    V90_RX_EVENT_B1,
    V90_RX_EVENT_FAILURE,
    V90_RX_EVENT_RETRAIN,
    V90_RX_EVENT_TIMEOUT
} v90_rx_event_t;

/* V.90 Phase 3/4 TX sub-states for the digital modem.
 * V.90 order (§9.3.1/§9.4.1):
 *   Sd → S̄d → TRN1d → Jd → J'd → DIL → Ri → TRN2d → MP/MP' → Ed → B1d → Data
 * V.92 Phase 3 replaces J'd with Jp → Jp' and sends DIL or SCR while
 * receiving the 2-point CPt stream:
 *   Sd → S̄d → TRN1d → Jd → Jp → Jp' → DIL/SCR → Ri → TRN2d …
 * V.92 Phase 4 extension (§9.6.1/V.92):
 *   … → TRN2d → SUVd → CPd → SUVd' → Ed → B1d → Data */
typedef enum {
    V90_TX_PHASE2,        /* SpanDSP V.34 handles INFO exchange */
    V90_TX_WAIT_JA,       /* Phase 3: silent until the analogue modem's Ja */
    V90_TX_SD_DELAY,      /* Phase 3: silent, calibrated pre-Sd delay (§9.3.1.3 "may wait up to 500ms") */
    V90_TX_SD,            /* Sending Sd — repeated {+W,+0,+W,-W,-0,-W} */
    V90_TX_SD_BAR,        /* Sending S̄d — 8 reps of {-W,-0,-W,+W,+0,+W} */
    V90_TX_TRN1D,         /* Sending TRN1d — scrambled ones on U_INFO */
    V90_TX_JD,            /* Sending Jd (Table 13) — capabilities frame */
    V90_TX_JD_PRIME,      /* Sending J'd — 12 zeros to terminate Jd */
    V90_TX_JP,            /* V.92 Jp (Table 22) */
    V90_TX_JP_PRIME,      /* V.92 Jp' — 12 scrambled zeros */
    V90_TX_DIL,           /* Sending DIL descriptor symbols */
    V90_TX_SCR,           /* V.92 SCR while waiting for CPt */
    V90_TX_RI,            /* Phase 4: Ri — retrain init (idle codewords, §9.4.1.1) */
    V90_TX_RI_ACK,        /* Phase 4: CPt acknowledgement, barred R-i */
    V90_TX_TRN2D,         /* Phase 4: negotiated mapped TRN2d after R-i */
    V90_TX_MP,            /* V.90 Phase 4: MP/MP' modulation-parameter frames */
    V90_TX_SUVD,          /* V.92 Phase 4: SUVd (no ack) — short update values digital */
    V90_TX_CP,            /* V.92 Phase 4: CPd compatibility frame */
    V90_TX_SUVD_ACK,      /* V.92 Phase 4: SUVd' (ack bit set) — acknowledges CPu */
    V90_TX_ED,            /* Phase 4: Ed — 2 data frames of scrambled zeros */
    V90_TX_B1D,           /* Phase 4: B1d — 48 mapped data frames */
    V90_TX_DATA,          /* Data mode — modulus encoder */
} v90_tx_phase_t;

/*
 * Initialise a V.90 digital modem context.
 * Creates an underlying V.34 modem with V.90 INFO0d enabled.
 */
/*
 * Create a V.90 context wrapping an existing V.34 state.
 * Use this when V.34 was already initialized externally (e.g., by modem_engine).
 */
v90_state_t *v90_init_with_v34(v34_state_t *v34, v90_law_t law);

/*
 * Create a V.90 context for data/codeword processing without allocating a
 * backing V.34 instance. Useful for loopback/session tests that only need the
 * PCM mapping path.
 */
v90_state_t *v90_init_data_pump(v90_law_t law);

v90_state_t *v90_init(int baud_rate,
                      int bit_rate,
                      bool calling_party,
                      v90_law_t law,
                      span_get_bit_func_t get_bit,
                      void *get_bit_user_data,
                      span_put_bit_func_t put_bit,
                      void *put_bit_user_data);

/* Free a V.90 context and its underlying V.34 state. */
void v90_free(v90_state_t *s);

/*
 * Get the underlying V.34 state for passing to v34_tx()/v34_rx()
 * during training.  After training completes, TX switches to PCM.
 */
v34_state_t *v90_get_v34(v90_state_t *s);

/* Get current Phase 3/4 TX sub-state. */
v90_tx_phase_t v90_get_tx_phase(v90_state_t *s);

/*
 * Check if V.90 Phase 3 TX should take over from V.34 TX.
 * Returns true once Phase 2 INFO exchange is complete and the
 * digital modem should begin sending PCM codewords.
 */
bool v90_phase3_active(v90_state_t *s);

/*
 * Start V.90 Phase 3 TX. Call this when the V.34 Phase 2 INFO
 * exchange completes (V34_TX_STAGE_FIRST_S detected).
 * u_info is the U_INFO Ucode from the analog modem's INFO1a.
 */
void v90_start_phase3(v90_state_t *s, int u_info);

/* Select the V.92 §9.5 Phase-3 state path before v90_start_phase3().
 * This is separate from v90_enable_v92_mode(): callers should only enable it
 * after mutual INFO0 capability confirmation. */
void v90_enable_v92_phase3(v90_state_t *s);

/*
 * Configure the DIL descriptor requested by the far-end analogue modem.
 * A descriptor with n == 0 disables DIL transmission.
 */
void v90_set_dil_descriptor(v90_state_t *s, const v90_dil_desc_t *desc);

/* Initialise default analogue-side Phase 2 INFO contracts. */
void v90_info0a_init(v90_info0a_t *info);
void v90_info1a_init(v90_info1a_t *info);

/* Pack analogue-side INFO frames bit-0 first for V.34/V.90 Phase 2 testing. */
bool v90_info0a_validate(const v90_info0a_t *info);
bool v90_info1a_validate(const v90_info1a_t *info);
bool v90_build_info0a_bits(uint8_t *buf, int buf_len, const v90_info0a_t *info);
bool v90_build_info1a_bits(uint8_t *buf, int buf_len, const v90_info1a_t *info);
bool v90_parse_info0a_bits(v90_info0a_t *out, const uint8_t *bits, int bit_len);
bool v90_parse_info1a_bits(v90_info1a_t *out, const uint8_t *bits, int bit_len);
bool v90_info0a_build_diag(const v90_info0a_t *info, v90_info0a_diag_t *diag);
bool v90_info1a_build_diag(const v90_info1a_t *info, v90_info1a_diag_t *diag);
bool v90_info0a_decode_diag(const uint8_t *bits, int bit_len, v90_info0a_diag_t *diag);
bool v90_info1a_decode_diag(const uint8_t *bits, int bit_len, v90_info1a_diag_t *diag);

/*
 * Parse a packed Ja/DIL descriptor bitstream (Table 12/V.90, bit 0 first)
 * into a DIL descriptor. Returns true on success.
 */
bool v90_parse_dil_descriptor(v90_dil_desc_t *out, const uint8_t *bits, int bit_len);

/* Load the SmartLink SL8200/SLModem ADI-QC Ja descriptor captured from the
 * peer DSP.  This is an explicit interoperability fallback for live testing;
 * normal operation should decode the descriptor received in Ja. */
bool v90_dil_load_smartlink_adi_qc(v90_dil_desc_t *out);
bool v90_dil_load_smartlink_adi(v90_dil_desc_t *out);

/*
 * G.711 codeword <-> (Ucode, sign) helpers (Table 1/V.90 mapping).
 * sign: 1 = positive, 0 = negative.
 */
uint8_t v90_codeword_compose(v90_law_t law, int ucode, int sign);
void v90_codeword_decompose(v90_law_t law, uint8_t codeword, int *ucode_out, int *sign_out);

/*
 * Stateless §8.4.1 DIL generation from a descriptor.
 * v90_dil_cycle_len() returns the symbols in one full N-segment cycle
 * (0 when the descriptor disables DIL). v90_dil_generate_codewords()
 * fills out[] with len codewords, repeating the cycle as needed, and
 * returns the count written.
 */
int v90_dil_cycle_len(const v90_dil_desc_t *desc);
int v90_dil_generate_codewords(v90_law_t law,
                               const v90_dil_desc_t *desc,
                               uint8_t *out,
                               int len);
int v90_dil_descriptor_bit_len(const v90_dil_desc_t *desc);
bool v90_build_dil_descriptor_bits(uint8_t *buf,
                                   int buf_len,
                                   int *bit_len_out,
                                   const v90_dil_desc_t *desc);
bool v90_analyse_dil_descriptor(const v90_dil_desc_t *desc, v90_dil_analysis_t *analysis_out);

/*
 * Notify the V.90 Phase 3 state machine that the far-end analogue modem's
 * S signal has been detected and the current Jd repetition should be the last.
 */
void v90_notify_s_detected(v90_state_t *s);

/* Apply a strict receiver event to the transmit startup machine. Returns true
 * only when the event is valid for, and accepted by, the current TX phase. */
bool v90_handle_rx_event(v90_state_t *s, v90_rx_event_t event);

const char *v90_rx_event_name(v90_rx_event_t event);

/*
 * Check whether the V.90 state machine has completed startup and may enter
 * data mode. Returns true after B1d completes and the state enters V90_TX_DATA.
 */
bool v90_training_complete(v90_state_t *s);

/*
 * V.90 §9.6 rate renegotiation, digital-modem side.
 *
 * §9.6 allows it "at any time during data mode" and lets the data signalling
 * rate and the spectral shaping parameters change as a result -- so it is the
 * speed shift.  It is also the re-acquisition after a loss of carrier: the
 * analogue modem answers with S, S-bar, SCR, CP and then a fresh B1, and B1 is
 * what the upstream receiver acquires against.
 *
 * v90_request_rate_renegotiation() only arms the request.  §9.6 requires the
 * digital modem's transmitter to start one "only on the boundary of a data
 * frame", so the transmit path calls v90_rate_renegotiation_start() there.
 */
bool v90_request_rate_renegotiation(v90_state_t *s);
bool v90_rate_renegotiation_pending(const v90_state_t *s);
bool v90_rate_renegotiation_active(const v90_state_t *s);
bool v90_rate_renegotiation_start(v90_state_t *s);
bool v90_rate_renegotiation_timed_out(const v90_state_t *s);
int  v90_rate_renegotiation_count(const v90_state_t *s);

/*
 * Apply a received CPt, data-mode CP, or CP' frame to Phase 4.  CPt configures
 * TRN2d/MP/Ed; CP configures B1d/data; a repeated data-mode CP may change only
 * its acknowledge bit. V.92 mode retains the CPd compatibility transmitter.
 * The caller's cp pointer need not remain valid after return.
 */
bool v90_set_phase4_cp(v90_state_t *s, const vpcm_cp_frame_t *cp);

/*
 * Repair the exact malformed dummy CPt emitted by the private SmartLink
 * interoperability peer after its TRN2 designer selects the built-in
 * 8-point fallback constellation.  The helper is deliberately fingerprinted
 * and does not relax normal Table-14 validation.  Callers should additionally
 * gate its use as an explicit interoperability workaround.
 */
bool v90_repair_smartlink_dummy_cpt(vpcm_cp_frame_t *cp);

/*
 * Override the pre-Sd delay (§9.3.1.3) for this training attempt, in
 * milliseconds; a negative value restores the ME_V90_SD_DELAY_MS env default.
 * A §9.5-retrained attempt needs a longer delay than the initial one: the
 * pre-converged Phase 2 makes our Ja detection outrun the peer's WaitForSd
 * arming, and an early Sd leaves its equalizer training unanchored.
 */
void v90_set_sd_delay_ms(v90_state_t *s, int ms);

/* Copy the currently prepared Type-0 MP/MP' bitstream for diagnostics/tests.
 * Returns its bit length, or 0 if the V.90 Phase 4 mapper is not configured. */
int v90_copy_phase4_mp_bits(const v90_state_t *s, uint8_t *bits, int max_bits);

/* Negotiated V.90 data-mode mapper diagnostics and one-frame encoder. */
int v90_data_bits_per_frame(const v90_state_t *s);
int v90_data_input_bytes_needed(const v90_state_t *s);
int v90_tx_data_frame_codewords(v90_state_t *s,
                                uint8_t codewords[6],
                                const uint8_t *data,
                                int data_len,
                                int *data_consumed,
                                bool fill_with_ones);

/*
 * Signal that the analogue modem's most recently supplied CPt, CP, or CP'
 * frame passed strict receive validation. CPt starts post-CP Ri/TRN2d, CP
 * causes MP' acknowledgement, and CP' permits Ed at an MP' frame boundary.
 */
void v90_notify_cp_ready(v90_state_t *s);

/* Cap the MP Type-0 upstream rate offer (bits 24:27 max drn and the 36:48
 * capability mask) at the rate the V.34 upstream receiver actually trained
 * at.  0 clears the cap (echo the peer's CPt mask verbatim). */
void v90_set_upstream_rate_limit(v90_state_t *s, int bps);

/*
 * Enable V.92 Phase 4 mode on this digital modem context.
 * When enabled, the Phase 4 sequence becomes:
 *   TRN2d → SUVd → CPd → SUVd' (ack) → Ed → B1d → Data
 * Must be called before v90_notify_cp_ready().
 */
void v90_enable_v92_mode(v90_state_t *s);

/*
 * Enable native V.92 upstream Phase 4 gating (§9.6.1.1/V.92).  Requires
 * v90_enable_v92_mode().  When enabled, SUVd repeats until a real SUVu or
 * CPu is received, a single CPd is sent per received SUVu/CPu, the SUVd
 * acknowledge bit is set only after a valid CPu, and Ed starts only once an
 * acknowledged sequence has been sent and CPu'/SUVu' (or E2u) received.
 */
void v90_enable_v92_native_cpu_rx(v90_state_t *s);

/* Apply a received V.92 SUVu/SUVu' frame (Table 27) to Phase 4. */
bool v90_set_v92_suvu(v90_state_t *s, bool acknowledge);

/*
 * Apply a received V.92 CPu/CPu' frame (Table 23, converted to the shared
 * V.PCM CP representation with v92_cp_frame_to_vpcm) to Phase 4.  The first
 * CPu configures the negotiated data-mode mapper used for B1d and data mode;
 * repeated frames may change only the acknowledge bit.
 */
bool v90_set_v92_cpu(v90_state_t *s, const vpcm_cp_frame_t *cpu);

/* Copy the CPu-derived data-mode CP for native CPd construction. */
bool v90_get_v92_cpu(const v90_state_t *s, vpcm_cp_frame_t *out);

/*
 * Set the Table 30 CPd upstream profile (analogue-to-digital rate 0..19,
 * trellis select 0..2, prefilter gain 4G in unsigned Q0.16).  Native V.92
 * mode transmits these in the mapped CPd until real upstream receiver
 * measurements can refine them.
 */
bool v90_set_v92_cpd_profile(v90_state_t *s,
                             uint8_t upstream_drn,
                             uint8_t trellis_select,
                             uint16_t gain_q0_16);

/*
 * Fill the native Table 30 CPd frame this context would transmit: profile
 * rate/trellis/gain, modulus parameters, and the robbed-bit-safe upstream
 * constellation set; the acknowledge bit reflects CPu receipt.
 */
bool v90_build_v92_cpd_frame(const v90_state_t *s, v92_cpd_frame_t *out);

/*
 * Demap one Sr=0 six-symbol mapped data frame (TRN2d/SUVd/CPd/Ed or B1d)
 * back into its descrambled input bits.  bits_per_frame is the mapper's d;
 * descramble_reg/prev_sign carry receiver state across frames (init 0).
 * Returns bits_per_frame, or 0 when the codewords escape the CP mapping.
 */
int v90_demap_mapped_frame(v90_law_t law,
                           const vpcm_cp_frame_t *cp,
                           int bits_per_frame,
                           uint32_t *descramble_reg,
                           int *prev_sign,
                           const uint8_t codewords[6],
                           uint8_t bits_out[]);

typedef struct {
    uint8_t prev_odd;
    uint8_t prev_t[6];
    uint8_t trellis_state;
} v90_shaped_rx_state_t;

/* Demap one Sr=1/2/3 shaped frame.  The shaping state and descrambler are
 * reset to zero at the first TRN2d frame and carried across subsequent MP,
 * Ed, B1d, and data frames. */
int v90_demap_shaped_frame(v90_law_t law,
                           const vpcm_cp_frame_t *cp,
                           int bits_per_frame,
                           uint32_t *descramble_reg,
                           v90_shaped_rx_state_t *shaper,
                           const uint8_t codewords[6],
                           uint8_t bits_out[]);

int v90_demap_shaped_sign_frame(const vpcm_cp_frame_t *cp,
                                v90_shaped_rx_state_t *shaper,
                                const uint8_t signs[6],
                                uint8_t scrambled_sign_bits[5]);

int v90_track_known_shaped_sign_frame(
                                const vpcm_cp_frame_t *cp,
                                v90_shaped_rx_state_t *shaper,
                                const uint8_t scrambled_sign_bits[6],
                                const uint8_t observed_signs[6]);

int v90_generate_trn2d_codewords(v90_law_t law,
                                 const vpcm_cp_frame_t *cp,
                                 const v90_shaped_rx_state_t *initial_state,
                                 int frames,
                                 uint8_t codewords_out[],
                                 int codewords_max);

/* Map an explicit Phase-4 plaintext bitstream from reset scrambler/shaper
 * state.  The input contains frames*(cp->drn+8) bits and is useful for
 * protocol-constrained offline MP waveform recovery. */
int v90_generate_phase4_codewords(v90_law_t law,
                                  const vpcm_cp_frame_t *cp,
                                  const v90_shaped_rx_state_t *initial_state,
                                  const uint8_t plain_bits[],
                                  int frames,
                                  uint8_t codewords_out[],
                                  int codewords_max);

/* Reset the active negotiated or compatibility data mapper state. */
void v90_reset_data_mode(v90_state_t *s);

/*
 * Generate V.90 Phase 3 TX samples (PCM codewords as linear samples).
 * Call this instead of v34_tx() for the downstream direction once
 * v90_phase3_active() returns true.
 *
 * Returns the number of samples written (always == len).
 */
int v90_phase3_tx(v90_state_t *s, int16_t amp[], int len);

/*
 * Generate the same Phase 3/4 transmit sequence directly as G.711 octets.
 * This is the authoritative path for an octet-transparent RTP bearer.
 */
int v90_phase3_tx_codewords(v90_state_t *s, uint8_t codewords[], int len);

/*
 * Legacy byte-per-symbol compatibility mapper retained for older standalone
 * loopback/session tests. Live negotiated V.90 uses
 * v90_tx_data_frame_codewords().
 */
int v90_tx_codewords(v90_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len);

/*
 * Decode the legacy v90_tx_codewords() compatibility format.
 */
int v90_rx_codewords(v90_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len);

/*
 * Legacy linear wrapper around the compatibility mapper.
 */
int v90_tx_data(v90_state_t *s, int16_t amp[], int len,
                const uint8_t *data_in, int data_len);

/*
 * Generate idle (silence) PCM samples when no data is available.
 */
void v90_tx_idle(v90_state_t *s, int16_t amp[], int len);

/* Get the idle G.711 codeword for the selected PCM law. */
uint8_t v90_idle_codeword(v90_law_t law);

/* Get the logging context for diagnostics. */
logging_state_t *v90_get_logging_state(v90_state_t *s);

#endif /* V90_H */
