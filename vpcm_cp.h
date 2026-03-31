/*
 * vpcm_cp.h - Shared CP helpers for V.PCM-family modems
 */

#ifndef VPCM_CP_H
#define VPCM_CP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VPCM_CP_MAX_CONSTELLATIONS 6
#define VPCM_CP_FRAME_INTERVALS 6
#define VPCM_CP_MASK_BITS 128
#define VPCM_CP_MASK_BYTES (VPCM_CP_MASK_BITS / 8)
#define VPCM_CP_MAX_BITS 972

typedef struct {
    bool transparent_mode_granted;
    bool v90_compatibility;
    uint8_t drn;
    bool acknowledge;
    uint8_t constellation_count;
    uint8_t dfi[VPCM_CP_FRAME_INTERVALS];
    uint8_t masks[VPCM_CP_MAX_CONSTELLATIONS][VPCM_CP_MASK_BYTES];
} vpcm_cp_frame_t;

typedef struct {
    vpcm_cp_frame_t frame;
    uint8_t bits[VPCM_CP_MAX_BITS];
    int nbits;
    uint16_t crc_field;
    uint16_t crc_remainder;
    bool frame_sync_ok;
    bool start_bits_ok;
    bool v90_compat_ok;
    bool fill_ok;
    bool valid;
} vpcm_cp_diag_t;

void vpcm_cp_init(vpcm_cp_frame_t *cp);
bool vpcm_cp_validate(const vpcm_cp_frame_t *cp, char *reason, size_t reason_len);
int vpcm_cp_bit_length(const vpcm_cp_frame_t *cp);
bool vpcm_cp_encode_bits(const vpcm_cp_frame_t *cp, uint8_t *bits_out, int *nbits_out);
bool vpcm_cp_decode_bits(const uint8_t *bits, int nbits, vpcm_cp_frame_t *cp_out);
bool vpcm_cp_build_diag(const vpcm_cp_frame_t *cp, vpcm_cp_diag_t *diag);
bool vpcm_cp_decode_diag(const uint8_t *bits, int nbits, vpcm_cp_diag_t *diag);
bool vpcm_cp_frames_equal(const vpcm_cp_frame_t *a, const vpcm_cp_frame_t *b);
double vpcm_cp_drn_to_bps(uint8_t drn);
int vpcm_cp_drn_to_k(uint8_t drn);
double vpcm_cp_robbed_bit_ceiling_bps(void);
uint8_t vpcm_cp_recommended_robbed_bit_drn(void);
void vpcm_cp_enable_all_ucodes(uint8_t mask[VPCM_CP_MASK_BYTES]);
void vpcm_cp_enable_odd_ucodes(uint8_t mask[VPCM_CP_MASK_BYTES]);
void vpcm_cp_init_robbed_bit_safe_profile(vpcm_cp_frame_t *cp,
                                          uint8_t drn,
                                          bool transparent_mode_granted);
int vpcm_cp_select_ucode(const vpcm_cp_frame_t *cp, int frame_interval, bool prefer_high);

void vpcm_cp_mask_set(uint8_t mask[VPCM_CP_MASK_BYTES], int ucode, bool enabled);
bool vpcm_cp_mask_get(const uint8_t mask[VPCM_CP_MASK_BYTES], int ucode);
int vpcm_cp_mask_population(const uint8_t mask[VPCM_CP_MASK_BYTES]);

#endif /* VPCM_CP_H */
