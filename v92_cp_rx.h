/*
 * v92_cp_rx.h — Native V.92 upstream Phase 4 control-frame receiver
 *
 * Strict codecs and a bit-level receiver for the control frames the analogue
 * modem sends to the digital modem during V.92 Phase 4, rate renegotiation,
 * and fast parameter exchange:
 *
 *   CPt / CPu / CPu´  Table 23/V.92 (shared layout; type field selects)
 *   CPus              Table 24/V.92 (short CPu)
 *   SUVu / SUVu´      Table 27/V.92
 *
 * All frames share the 17-ones frame sync and start-bit prefix; bit 18
 * distinguishes SUVu (1) from the CP family (0), and CP bits 19:20 select
 * CPt (0), CPu (1), or CPus (2).  Bits arrive already descrambled and
 * differentially decoded from the upstream training modulation. CPt in
 * Phase 3 uses 2-point TRN1u (12-bit fill); Phase-4 CPu/SUVu use 4- or
 * 8-point TRN2u (24- or 36-bit fill).
 *
 * CPu supplies the real downstream rate (drn, d = drn + 20), TRN1d gain,
 * spectral shaping parameters, and data-mode constellations the digital
 * modem needs to build a native Table 30 CPd and run the negotiated mapper;
 * the acknowledge bit in CPu´/SUVu´ drives the §9.6.1.1 Ed transition.
 */

#ifndef V92_CP_RX_H
#define V92_CP_RX_H

#include <stdbool.h>
#include <stdint.h>

#include "vpcm_cp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define V92_CP_MAX_CONSTELLATIONS 6
#define V92_CP_MASK_BYTES VPCM_CP_MASK_BYTES

/* Largest Table 23 frame: 6 constellations with differing codec sets:
 * gamma = 136*5, delta = 2*gamma + 136, raw = 290 + delta = 1786 bits,
 * filled to a 12-symbol multiple (1800 bits at 2 or 3 bits per symbol). */
#define V92_CP_RX_MAX_BITS 1800

/* Fixed-size frames (CPus/SUVu): 52 raw bits filled to 72. */
#define V92_CPUS_MAX_BITS 72
#define V92_SUVU_MAX_BITS 72

typedef enum {
    V92_CP_TYPE_CPT = 0,
    V92_CP_TYPE_CPU = 1,
    V92_CP_TYPE_CPUS = 2
} v92_cp_type_t;

/* Table 23 CPt/CPu frame. */
typedef struct {
    uint8_t type;                   /* bits 19:20 — V92_CP_TYPE_CPT or _CPU */
    uint8_t drn;                    /* bits 21:25 — 0..22, 0 = cleardown */
    uint8_t shaping_redundancy;     /* Sr, bits 31:32 */
    bool acknowledge;               /* bit 33 — CPu´ when set */
    bool codec_alaw;                /* bit 35 — 0 = µ-law, 1 = A-law */
    uint8_t shaping_lookahead;      /* ld, bits 49:50 */
    uint16_t trn1d_gain_q3_13;      /* bits 52:67, unsigned Q3.13 */
    uint8_t shaping_a1_q1_6;        /* bits 69:76, signed Q1.6 */
    uint8_t shaping_a2_q1_6;        /* bits 77:84 */
    uint8_t shaping_b1_q1_6;        /* bits 86:93 */
    uint8_t shaping_b2_q1_6;        /* bits 94:101 */
    uint8_t dfi[VPCM_CP_FRAME_INTERVALS];  /* constellation index per interval */
    uint8_t constellation_count;    /* max dfi + 1 */
    bool codec_constellations_differ;      /* bit 128 */
    uint8_t masks[V92_CP_MAX_CONSTELLATIONS][V92_CP_MASK_BYTES];
    /* Valid only when codec_constellations_differ. */
    uint8_t codec_masks[V92_CP_MAX_CONSTELLATIONS][V92_CP_MASK_BYTES];
} v92_cp_frame_t;

typedef struct {
    v92_cp_frame_t frame;
    uint8_t bits[V92_CP_RX_MAX_BITS];
    int nbits;
    uint16_t crc_field;
    uint16_t crc_expected;
    bool binary_bits_ok;
    bool frame_sync_ok;
    bool identifier_ok;             /* bit 18 == 0, type <= 1 */
    bool start_bits_ok;
    bool reserved_ok;               /* bits 26:30, 36:48, 129:135 */
    bool parameters_ok;             /* drn <= 22, dfi <= 5 */
    bool fill_bits_ok;
    bool crc_ok;
    bool valid;
} v92_cp_diag_t;

/* Table 24 CPus frame. */
typedef struct {
    uint8_t drn;                    /* bits 21:25 */
    bool acknowledge;               /* bit 33 */
} v92_cpus_frame_t;

typedef struct {
    v92_cpus_frame_t frame;
    uint16_t crc_field;
    uint16_t crc_expected;
    bool binary_bits_ok;
    bool frame_sync_ok;
    bool identifier_ok;             /* bit 18 == 0, type == 2 */
    bool start_bits_ok;
    bool reserved_ok;               /* bits 26:32 */
    bool parameters_ok;             /* drn <= 22 */
    bool fill_bits_ok;
    bool crc_ok;
    bool valid;
} v92_cpus_diag_t;

/* Table 27 SUVu frame. */
typedef struct {
    bool wait_for_cpu;              /* bit 26 — hold CPd until CPu request */
    uint8_t prefilter_level_q2_2;   /* bits 27:31, signed Q2.2; 16 = unmeasured */
    bool silent_period_requested;   /* bit 32 */
    bool acknowledge;               /* bit 33 — SUVu´ when set */
} v92_suvu_frame_t;

typedef struct {
    v92_suvu_frame_t frame;
    uint16_t crc_field;
    uint16_t crc_expected;
    bool binary_bits_ok;
    bool frame_sync_ok;
    bool identifier_ok;             /* bit 18 == 1 */
    bool start_bits_ok;
    bool reserved_ok;               /* bits 19:25 */
    bool fill_bits_ok;
    bool crc_ok;
    bool valid;
} v92_suvu_diag_t;

/* ---- Frame codecs ----
 * constellation_points selects the upstream modulation (2-point TRN1u for
 * Phase-3 CPt, or 4/8-point TRN2u for Phase 4) and its 12-symbol fill
 * alignment. */

int v92_cp_bit_length(const v92_cp_frame_t *cp, int constellation_points);
bool v92_cp_encode(const v92_cp_frame_t *cp,
                   int constellation_points,
                   uint8_t *bits_out,
                   int bits_max,
                   int *nbits_out);
bool v92_cp_decode_diag(const uint8_t *bits, int nbits, v92_cp_diag_t *diag);

int v92_cpus_bit_length(int constellation_points);
bool v92_cpus_encode(const v92_cpus_frame_t *frame,
                     int constellation_points,
                     uint8_t *bits_out,
                     int bits_max,
                     int *nbits_out);
bool v92_cpus_decode_diag(const uint8_t *bits, int nbits, v92_cpus_diag_t *diag);

int v92_suvu_bit_length(int constellation_points);
bool v92_suvu_encode(const v92_suvu_frame_t *frame,
                     int constellation_points,
                     uint8_t *bits_out,
                     int bits_max,
                     int *nbits_out);
bool v92_suvu_decode_diag(const uint8_t *bits, int nbits, v92_suvu_diag_t *diag);

/*
 * Convert a Table 23 frame into the shared V.PCM CP representation used by
 * the V.90/V.92 mapper configuration paths.  CPu maps to a data-mode CP
 * (d = drn + 20); CPt maps to training parameters (d = drn + 8).  The V.92
 * codec-constellation extension stays in the v92_cp_frame_t; the mapper
 * always uses the transmit constellations.
 */
bool v92_cp_frame_to_vpcm(const v92_cp_frame_t *in, vpcm_cp_frame_t *out);

/* ---- Bit-level receiver ---- */

typedef enum {
    V92_P4U_KIND_CPT = 0,
    V92_P4U_KIND_CPU,
    V92_P4U_KIND_CPUS,
    V92_P4U_KIND_SUVU
} v92_p4u_kind_t;

/* Exactly one of cp/cpus/suvu is non-NULL, matching kind. */
typedef void (*v92_cp_rx_handler_t)(void *user_data,
                                    v92_p4u_kind_t kind,
                                    const v92_cp_diag_t *cp,
                                    const v92_cpus_diag_t *cpus,
                                    const v92_suvu_diag_t *suvu);

typedef struct {
    int constellation_points;       /* 2 (TRN1u), 4 or 8 (TRN2u) */
    bool expected_alaw;             /* CPt/CPu bit 35 must match */
    v92_cp_rx_handler_t handler;
    void *user_data;

    uint8_t bits[V92_CP_RX_MAX_BITS];
    int bit_count;
    int target_bits;
    int sync_ones;
    bool collecting;

    uint32_t input_bits;
    uint32_t valid_frames;
    uint32_t rejected_frames;
} v92_cp_rx_t;

void v92_cp_rx_init(v92_cp_rx_t *rx,
                    int constellation_points,
                    bool expected_alaw,
                    v92_cp_rx_handler_t handler,
                    void *user_data);
void v92_cp_rx_reset(v92_cp_rx_t *rx);

/* Feed one descrambled, differentially decoded upstream bit.  Returns true
 * when this bit completed a frame that passed strict validation. */
bool v92_cp_rx_put_bit(v92_cp_rx_t *rx, int bit);

#ifdef __cplusplus
}
#endif

#endif /* V92_CP_RX_H */
