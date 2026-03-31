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

/* V.90 downstream encoder state */
typedef struct v90_state_s v90_state_t;

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

/* PCM law selection */
typedef enum {
    V90_LAW_ULAW = 0,
    V90_LAW_ALAW = 1
} v90_law_t;

/* V.90 Phase 3/4 TX sub-states for the digital modem.
 * Order matches V.90 §9.3.1: Sd → S̄d → TRN1d → Jd → J'd → Phase 4 */
typedef enum {
    V90_TX_PHASE2,        /* SpanDSP V.34 handles INFO exchange */
    V90_TX_SD,            /* Sending Sd — 64 reps of {+W,+0,+W,-W,-0,-W} */
    V90_TX_SD_BAR,        /* Sending S̄d — 8 reps of {-W,-0,-W,+W,+0,+W} */
    V90_TX_TRN1D,         /* Sending TRN1d — scrambled ones on U_INFO */
    V90_TX_JD,            /* Sending Jd (Table 13) — capabilities frame */
    V90_TX_JD_PRIME,      /* Sending J'd — 12 zeros to terminate Jd */
    V90_TX_DIL,           /* Sending DIL (placeholder branch point) */
    V90_TX_PHASE4,        /* Phase 4 (B1d, TRN2d, MP, CP exchange) */
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
 * Returns true once external Phase 3 has finished and TX should be driven by
 * the wrapped SpanDSP V.34 engine for native Phase 4/MP/E/data processing.
 */
bool v90_using_internal_v34_tx(v90_state_t *s);

/*
 * Start V.90 Phase 3 TX. Call this when the V.34 Phase 2 INFO
 * exchange completes (V34_TX_STAGE_FIRST_S detected).
 * u_info is the U_INFO Ucode from the analog modem's INFO1a.
 */
void v90_start_phase3(v90_state_t *s, int u_info);

/*
 * Configure the DIL descriptor requested by the far-end analogue modem.
 * A descriptor with n == 0 disables DIL transmission.
 */
void v90_set_dil_descriptor(v90_state_t *s, const v90_dil_desc_t *desc);

/* Initialise default analogue-side Phase 2 INFO contracts. */
void v90_info0a_init(v90_info0a_t *info);
void v90_info1a_init(v90_info1a_t *info);

/* Pack analogue-side INFO frames bit-0 first for V.34/V.90 Phase 2 testing. */
bool v90_build_info0a_bits(uint8_t *buf, int buf_len, const v90_info0a_t *info);
bool v90_build_info1a_bits(uint8_t *buf, int buf_len, const v90_info1a_t *info);

/*
 * Parse a packed Ja/DIL descriptor bitstream (Table 12/V.90, bit 0 first)
 * into a DIL descriptor. Returns true on success.
 */
bool v90_parse_dil_descriptor(v90_dil_desc_t *out, const uint8_t *bits, int bit_len);

/*
 * Notify the V.90 Phase 3 state machine that the far-end analogue modem's
 * S signal has been detected and the current Jd repetition should be the last.
 */
void v90_notify_s_detected(v90_state_t *s);

/*
 * Check whether the V.90 state machine has completed startup and may enter
 * data mode. Phase 4 is still incomplete, so this is currently only true
 * once a future implementation marks it complete.
 */
bool v90_training_complete(v90_state_t *s);

/*
 * Generate V.90 Phase 3 TX samples (PCM codewords as linear samples).
 * Call this instead of v34_tx() for the downstream direction once
 * v90_phase3_active() returns true.
 *
 * Returns the number of samples written (always == len).
 */
int v90_phase3_tx(v90_state_t *s, int16_t amp[], int len);

/*
 * Encode downstream payload bytes directly into G.711 codewords.
 * The current implementation uses the same simplified byte-per-symbol mapping
 * as v90_tx_data(), but exposes it at the codeword level for loopback tests
 * and session plumbing.
 */
int v90_tx_codewords(v90_state_t *s,
                     uint8_t *g711_out,
                     int g711_max,
                     const uint8_t *data_in,
                     int data_len);

/*
 * Decode downstream G.711 codewords produced by v90_tx_codewords() back into
 * payload bytes.
 */
int v90_rx_codewords(v90_state_t *s,
                     uint8_t *data_out,
                     int data_max,
                     const uint8_t *g711_in,
                     int g711_len);

/*
 * Encode downstream data into PCM codewords for the G.711 RTP stream.
 * Call this instead of v34_tx() once training completes.
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
