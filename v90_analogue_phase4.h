/*
 * v90_analogue_phase4.h — the analogue modem's Phase 4 receiver (§9.4.2).
 *
 * Phase 3's receiver read signals that carry one bit on the sign of one
 * codeword.  Phase 4's do not: from TRN2d onwards the digital modem is using
 * the §5.4 modulus encoder over the constellation *this side* named in CPt, so
 * reading it means running that mapping backwards.  v90_demap_shaped_frame()
 * already does exactly that — it is the same function the offline decoders
 * use — so this module's job is to find the frame grid, keep the shaping and
 * descrambler state across it, and turn the resulting bit stream into the
 * three things §9.4.2 is conditional on.
 *
 * The signals (§8.6):
 *
 *   Ri      §8.6.4 — six codewords at Ucode(U_INFO) with the sign pattern
 *                    + + + − − −, repeated.  R̄i is the same six with the
 *                    pattern reversed, sent for exactly 24T (four repetitions)
 *                    before TRN2d.  Neither is differentially encoded, and the
 *                    NOTE requires the receiver to detect them "regardless of
 *                    their polarity" — so what is detected is the *transition*,
 *                    not either pattern's absolute sign.
 *   TRN2d   §8.6.5 — scrambled ones through the CPt constellation, at least
 *                    2040T, an integer multiple of 6 symbols.  The scrambler
 *                    and shaping state are zero at its first frame, which is
 *                    what makes the whole of Phase 4 decodable from here.
 *   MP      §8.6.3 — a Table 16 frame in the same mapping, opening with 17
 *                    sync ones.  MP′ is MP with bit 33 set.
 *   Ed      §8.6.2 — two data frames of scrambled zeroes, ending MP.
 *
 * Ri's Ucode is taken off the wire, not computed from U_INFO, for the reason
 * given at length in v90_analogue_rx.c: this peer does not honour U_INFO for
 * Sd or TRN1d, and §8.6.4 gives no reason to expect it to here.
 */
#ifndef V90_ANALOGUE_PHASE4_H
#define V90_ANALOGUE_PHASE4_H

#include <stdbool.h>
#include <stdint.h>

#include "v90.h"
#include "v90_dil_measure.h"
#include "vpcm_cp.h"

typedef enum {
    V90A4_RX_HUNT_R = 0,    /* looking for §8.6.4's six-codeword sign pattern */
    V90A4_RX_R,             /* Ri acquired, tracking it */
    V90A4_RX_R_BAR,         /* §9.4.2.2 transition seen; R̄i running */
    V90A4_RX_TRN2D,         /* §8.6.5, demapping to ones */
    V90A4_RX_MP,            /* §8.6.3 Table 16 frames */
    V90A4_RX_B1D,           /* §8.6.1: 48 CP-mapped all-ones frames */
    V90A4_RX_DATA,          /* §9.4.1.5: negotiated downstream data */
    V90A4_RX_RR_RD,         /* §9.6.2.2.1: tracking Rd */
    V90A4_RX_RR_R_BAR,      /* §9.6.2.2.2: 24T R̄d */
    V90A4_RX_RR_RT,         /* §9.6.2.1.7: tracking Rt after CPs */
    V90A4_RX_RR_RT_BAR,     /* §9.6.2.1.8: 24T R̄t */
} v90_analogue_phase4_rx_stage_t;

#define V90A4_RX_EVENT_R          (1u << 0)  /* Ri acquired */
#define V90A4_RX_EVENT_R_BAR      (1u << 1)  /* §9.4.2.2 Ri-to-R̄i transition */
#define V90A4_RX_EVENT_TRN2D      (1u << 2)  /* TRN2d demapping to ones */
#define V90A4_RX_EVENT_MP         (1u << 3)  /* §9.4.2.3 a valid MP frame */
#define V90A4_RX_EVENT_MP_PRIME   (1u << 4)  /* §9.4.2.4 MP with bit 33 set */
#define V90A4_RX_EVENT_ED         (1u << 5)  /* §8.6.2 Ed */
#define V90A4_RX_EVENT_B1D        (1u << 6)  /* first valid §8.6.1 B1d frame */
#define V90A4_RX_EVENT_DATA       (1u << 7)  /* all 48 B1d frames received */
#define V90A4_RX_EVENT_RD         (1u << 8)  /* §9.6.2.2.1 Rd acquired */
#define V90A4_RX_EVENT_RD_BAR     (1u << 9)  /* §9.6.2.2.2 Rd→R̄d */
#define V90A4_RX_EVENT_RT         (1u << 10) /* §9.6.2.1.7 Rt acquired */
#define V90A4_RX_EVENT_RT_BAR     (1u << 11) /* §9.6.2.1.8 Rt→R̄t */

/* Table 16, the fields §9.4.2.4 acts on. */
typedef struct {
    bool     type1;             /* bit 18: precoder coefficients present */
    uint8_t  max_drn;           /* bits 24:27 — analogue-to-digital rate cap */
    uint8_t  trellis;           /* bits 29:30 */
    bool     nonlinear;         /* bit 31 */
    bool     expanded_shaping;  /* bit 32 */
    bool     acknowledge;       /* bit 33 — this is an MP′ */
    uint16_t rate_mask;         /* bits 36:48 — 4800 … 33600 */
    /* Table 16 Type 1, signed two's-complement precoder coefficients.  These
     * seed the analogue modem's V.34 upstream transmitter before B1. */
    int16_t  precoder[3][2];    /* [h(1)..h(3)][real, imaginary] */
} v90_analogue_mp_t;

typedef struct v90_analogue_phase4_s v90_analogue_phase4_t;

typedef struct {
    v90_law_t law;
    int       u_info;           /* what INFO1a announced; a hint, not a rule */
    /*
     * The CPt this side is transmitting.  §8.6.5 and §8.6.3 both map TRN2d and
     * MP with it, so without it nothing after R̄i can be read at all — which is
     * why Phase 4 cannot start until the measurement has produced one.
     */
    vpcm_cp_frame_t cpt;
    /* §8.6.1 and data mode switch to CP and reset all mapper state after Ed. */
    vpcm_cp_frame_t cp;
} v90_analogue_phase4_config_t;

v90_analogue_phase4_t *v90_analogue_phase4_init(const v90_analogue_phase4_config_t *cfg);
void v90_analogue_phase4_free(v90_analogue_phase4_t *s);

/* Consume received G.711 codewords.  Returns the events raised. */
unsigned v90_analogue_phase4_put(v90_analogue_phase4_t *s,
                                 const uint8_t *codewords,
                                 int count);

v90_analogue_phase4_rx_stage_t v90_analogue_phase4_stage(const v90_analogue_phase4_t *s);
const char *v90_analogue_phase4_stage_name(v90_analogue_phase4_rx_stage_t stage);

int v90_analogue_phase4_r_symbols(const v90_analogue_phase4_t *s);
int v90_analogue_phase4_trn2d_symbols(const v90_analogue_phase4_t *s);
int v90_analogue_phase4_mp_frames(const v90_analogue_phase4_t *s);
/* The most recent MP that passed Table 16's structure and CRC, or NULL. */
const v90_analogue_mp_t *v90_analogue_phase4_mp(const v90_analogue_phase4_t *s);
/* Frames whose codewords were not in the CPt constellation.  A digital modem
 * that ignored CPt produces nothing else, which is a different failure from
 * one that has not started transmitting. */
int v90_analogue_phase4_demap_failures(const v90_analogue_phase4_t *s);
/* How much of TRN2d demapped to §8.6.5's ones. */
int v90_analogue_phase4_trn2d_ones(const v90_analogue_phase4_t *s);
/* §8.6.1 progress and decoded data bits.  Bits are returned unpacked, oldest
 * first; B1d's known ones are validated but are never delivered as data. */
int v90_analogue_phase4_b1d_frames(const v90_analogue_phase4_t *s);
int v90_analogue_phase4_b1d_bit_errors(const v90_analogue_phase4_t *s);
int v90_analogue_phase4_rate_renegotiations(const v90_analogue_phase4_t *s);
/* Arm §9.6.2.1's optional CPs/echo-reconditioning cycle before local S. */
bool v90_analogue_phase4_start_rate_renegotiation(v90_analogue_phase4_t *s,
                                                   bool silence_request);
int v90_analogue_phase4_get_data_bits(v90_analogue_phase4_t *s,
                                      uint8_t *bits, int max_bits);

/*
 * §5.4.1's K for a CP frame — the bits that enter the modulus encoder, which
 * is the number both mappers have to agree on and the only one that can be
 * checked against the frame itself.  Table 14 bit 19 says which rate encoding
 * the frame uses (D = drn + 8 for a CPt, drn + 20 for a CP) and Sr says how
 * many of the six sign bits are shaping redundancy, so K = D - (6 - Sr).
 *
 * Returns -1 when the frame is not one a digital modem could act on: §5.4.3's
 * 2^K <= prod(Mi) fails, or a CPt's K is outside Table 17's 6 to 24.
 */
int v90_analogue_phase4_cp_k(const vpcm_cp_frame_t *cp);

/*
 * Build the CPt and CP frames a measured DIL implies (§8.5.2).
 *
 * CPt names the constellation the digital modem trains on and CP the one it
 * uses in data mode, and §8.5.2 caps CP's average power at 3 dB above CPt's.
 * Table 14 encodes their rates differently — (drn+20) for CP against (drn+8)
 * for CPt — so the same line rate is a different drn in each, and conflating
 * them puts the digital modem's mapper on a different number of bits per frame
 * than its receiver is decoding.
 *
 * `max_tx_dbm0` is INFO0d bits 33:37 in dBm0 and applies §8.5.2/Table 15
 * to CP's data-mode constellation.  Pass 0 only when that capability was not
 * received (the offline-test/unbounded case).
 *
 * `shaping_redundancy` is §5.4.5's Sr, which this side chooses: 0 disables
 * spectral shaping, 1 to 3 enable it and spend that many sign bits on it.  It
 * has to be decided here rather than stamped on afterwards, because it moves
 * both halves of the rate — S = 6 - Sr sign bits carry data, and K grows by Sr.
 * `shaping_lookahead` is Table 14's ld, which §8.5.2 requires to be consistent
 * with the digital modem's maximum in Jd bits 49:50; it must be 0 when Sr is,
 * there being no shaping to look ahead for.
 */
bool v90_analogue_phase4_build_cp(const v90_dil_measurement_t *m,
                                  v90_law_t law,
                                  double max_tx_dbm0,
                                  int shaping_redundancy,
                                  int shaping_lookahead,
                                  vpcm_cp_frame_t *cpt_out,
                                  vpcm_cp_frame_t *cp_out);

#endif
