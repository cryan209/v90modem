#include "v90_cp_rx.h"

#include <string.h>

static int v90_cp_candidate_length(int constellation_count, int points)
{
    vpcm_cp_frame_t cp;

    vpcm_cp_init(&cp);
    cp.constellation_count = (uint8_t)constellation_count;
    return vpcm_cp_modulated_bit_length(&cp, points);
}

static bool v90_cp_diag_is_strict(const v90_cp_rx_t *rx,
                                  const vpcm_cp_diag_t *diag)
{
    if (!rx || !diag || !diag->valid)
        return false;
    /* Initial Phase 4 uses CPt (Table 14 bit 19 = 1), without a silence
     * request, and the advertised digital-side law must match bit 35. */
    return diag->frame.v90_compatibility
        && diag->frame.drn <= 22
        && diag->bits[18] == 0
        && diag->bits[30] == 0
        && ((diag->bits[35] != 0) == rx->expected_alaw);
}

void v90_cp_rx_init(v90_cp_rx_t *rx,
                    int constellation_points,
                    bool expected_alaw,
                    v90_cp_rx_frame_handler_t frame_handler,
                    void *user_data)
{
    if (!rx)
        return;
    memset(rx, 0, sizeof(*rx));
    rx->constellation_points = constellation_points;
    rx->expected_alaw = expected_alaw;
    rx->frame_handler = frame_handler;
    rx->frame_user_data = user_data;
}

void v90_cp_rx_reset(v90_cp_rx_t *rx)
{
    if (!rx)
        return;
    rx->bit_count = 0;
    rx->target_bits = 0;
    rx->sync_ones = 0;
    rx->collecting = false;
}

bool v90_cp_rx_put_bit(v90_cp_rx_t *rx, int bit)
{
    bool accepted = false;

    if (!rx || (rx->constellation_points != 4 && rx->constellation_points != 16))
        return false;
    bit = bit ? 1 : 0;
    rx->input_bits++;

    if (!rx->collecting) {
        if (bit) {
            if (rx->sync_ones < 17)
                rx->sync_ones++;
            return false;
        }
        if (rx->sync_ones < 17) {
            rx->sync_ones = 0;
            return false;
        }
        memset(rx->bits, 0, sizeof(rx->bits));
        for (int i = 0; i < 17; i++)
            rx->bits[i] = 1;
        rx->bits[17] = 0;
        rx->bit_count = 18;
        rx->sync_ones = 0;
        rx->collecting = true;
        return false;
    }

    if (rx->bit_count >= VPCM_CP_MAX_BITS) {
        rx->rejected_frames++;
        v90_cp_rx_reset(rx);
        return false;
    }
    rx->bits[rx->bit_count++] = (uint8_t)bit;

    if (rx->bit_count == 129) {
        static const int dfi_pos[VPCM_CP_FRAME_INTERVALS] = {103, 107, 111, 115, 120, 124};
        int max_idx = 0;

        for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
            int value = 0;
            for (int b = 0; b < 4; b++)
                value |= (rx->bits[dfi_pos[i] + b] & 1) << b;
            if (value > max_idx)
                max_idx = value;
        }
        if (max_idx >= VPCM_CP_MAX_CONSTELLATIONS) {
            rx->rejected_frames++;
            v90_cp_rx_reset(rx);
            return false;
        }
        rx->target_bits = v90_cp_candidate_length(max_idx + 1,
                                                   rx->constellation_points);
    }

    if (rx->target_bits > 0 && rx->bit_count == rx->target_bits) {
        vpcm_cp_diag_t diag;
        if (vpcm_cp_decode_diag(rx->bits, rx->target_bits, &diag)
            && v90_cp_diag_is_strict(rx, &diag)) {
            rx->valid_frames++;
            if (rx->frame_handler)
                rx->frame_handler(rx->frame_user_data, &diag);
            accepted = true;
        } else {
            rx->rejected_frames++;
        }
        v90_cp_rx_reset(rx);
    }
    return accepted;
}
