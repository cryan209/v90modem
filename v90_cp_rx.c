#include "v90_cp_rx.h"

#include <string.h>

static int v90_cp_candidate_length(int constellation_count,
                                   int points,
                                   bool codec_constellations_differ)
{
    vpcm_cp_frame_t cp;

    vpcm_cp_init(&cp);
    cp.constellation_count = (uint8_t)constellation_count;
    cp.codec_constellations_differ = codec_constellations_differ;
    return vpcm_cp_modulated_bit_length(&cp, points);
}

static bool v90_cp_diag_is_strict(const v90_cp_rx_t *rx,
                                  const vpcm_cp_diag_t *diag)
{
    if (!rx || !diag || !diag->valid)
        return false;
    /* Accept both CPt (bit 19 = 0) and data-mode CP (bit 19 = 1). The V.90
     * state machine applies the phase-specific distinction. */
    return diag->frame.drn >= 1
        && diag->frame.drn <= 22
        && diag->bits[18] == 0
        && diag->bits[30] == 0
        && diag->frame.upstream_rate_mask != 0
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

void v90_cp_rx_clear_votes(v90_cp_rx_t *rx)
{
    if (!rx)
        return;
    rx->vote_count = 0;
    rx->vote_target_bits = 0;
}

/* Per-bit majority vote across the accumulated rejected copies.  Returns
 * true and fills out_diag if the voted consensus passes the same strict
 * validation as a raw copy.  CRC is the final gate: the vote only suggests
 * which bits to try, it never overrides a CRC failure. */
static bool v90_cp_rx_try_vote(v90_cp_rx_t *rx, vpcm_cp_diag_t *out_diag)
{
    uint8_t voted[VPCM_CP_MAX_BITS];
    int target;
    int n;
    vpcm_cp_diag_t diag;
    bool decoded;

    if (!rx || !out_diag)
        return false;
    target = rx->vote_target_bits;
    n = rx->vote_count;
    /* Need at least two copies for a vote to add any information beyond a
     * single raw copy (which was already CRC-tested and failed). */
    if (target < VPCM_CP_MAX_BITS && target <= 0)
        return false;
    if (n < 2)
        return false;
    if (target > VPCM_CP_MAX_BITS)
        return false;

    for (int b = 0; b < target; b++) {
        int ones = 0;
        for (int c = 0; c < n; c++)
            ones += (rx->vote[c].bits[b] & 1);
        voted[b] = (ones * 2 >= n) ? 1 : 0;
    }
    memset(&diag, 0, sizeof(diag));
    decoded = vpcm_cp_decode_diag(voted, target, &diag);
    if (decoded && v90_cp_diag_is_strict(rx, &diag)) {
        *out_diag = diag;
        return true;
    }
    return false;
}

/* Store a rejected copy for soft-combining.  Copies with a different
 * target_bits (frame length) than the running accumulator flush it: the
 * peer's CP parameters changed, so older votes are no longer comparable. */
static void v90_cp_rx_store_vote(v90_cp_rx_t *rx)
{
    int slot;

    if (!rx || rx->target_bits <= 0
        || rx->target_bits > VPCM_CP_MAX_BITS)
        return;
    if (rx->vote_target_bits != rx->target_bits) {
        rx->vote_count = 0;
        rx->vote_target_bits = rx->target_bits;
    }
    slot = rx->vote_count;
    if (slot >= V90_CP_RX_VOTE_FRAMES) {
        /* Ring: shift out the oldest. */
        for (int i = 1; i < V90_CP_RX_VOTE_FRAMES; i++)
            rx->vote[i - 1] = rx->vote[i];
        slot = V90_CP_RX_VOTE_FRAMES - 1;
    }
    memset(&rx->vote[slot], 0, sizeof(rx->vote[slot]));
    rx->vote[slot].target_bits = rx->target_bits;
    for (int b = 0; b < rx->target_bits; b++)
        rx->vote[slot].bits[b] = rx->bits[b];
    rx->vote_count = slot + 1;
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
        rx->sync_candidates++;
        return false;
    }

    if (rx->bit_count >= VPCM_CP_MAX_BITS) {
        rx->rejected_frames++;
        rx->structure_rejected_frames++;
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
            rx->structure_rejected_frames++;
            v90_cp_rx_reset(rx);
            return false;
        }
        rx->target_bits = v90_cp_candidate_length(
            max_idx + 1,
            rx->constellation_points,
            rx->bits[128] != 0);
    }

    if (rx->target_bits > 0 && rx->bit_count == rx->target_bits) {
        vpcm_cp_diag_t diag;
        bool decoded;

        memset(&diag, 0, sizeof(diag));
        decoded = vpcm_cp_decode_diag(rx->bits, rx->target_bits, &diag);
        if (decoded && v90_cp_diag_is_strict(rx, &diag)) {
            rx->valid_frames++;
            if (rx->frame_handler)
                rx->frame_handler(rx->frame_user_data, &diag);
            accepted = true;
            v90_cp_rx_clear_votes(rx);
        } else {
            rx->rejected_frames++;
            if (diag.frame_sync_ok
                && diag.start_bits_ok
                && diag.reserved_bits_ok
                && diag.v90_compat_ok
                && diag.fill_ok
                && diag.crc_remainder != 0) {
                rx->crc_rejected_frames++;
            } else if (diag.frame_sync_ok
                       && diag.start_bits_ok
                       && diag.reserved_bits_ok
                       && diag.v90_compat_ok
                       && diag.fill_ok
                       && diag.crc_remainder == 0) {
                /* The wire frame is intact, but a negotiated field is not
                 * usable by this V.90 endpoint (rate, law, mask, or a V.90
                 * reserved semantic).  vpcm_cp_decode_diag() deliberately
                 * returns false for some of these generic validation errors,
                 * so classify from its detailed observations, not its final
                 * Boolean alone. */
                rx->semantic_rejected_frames++;
            } else {
                rx->structure_rejected_frames++;
            }
            /* Soft-combine: if the frame boundary is aligned (sync + start
             * bits valid) but the CRC failed, the copy has correctable bit
             * errors.  Accumulate it and try a per-bit majority vote across
             * recent copies.  CPt/CP repeat, so a few corrupted copies that
             * agree on most bits can recover a CRC-valid consensus.  CRC is
             * still the final gate on the voted bits.  Frames with broken
             * sync/start/reserved/fill are not stored: their boundary may be
             * misaligned and majority-voting misaligned frames is noise. */
            if (diag.frame_sync_ok
                && diag.start_bits_ok
                && diag.crc_remainder != 0) {
                vpcm_cp_diag_t voted;

                v90_cp_rx_store_vote(rx);
                if (v90_cp_rx_try_vote(rx, &voted)) {
                    rx->voted_frames_accepted++;
                    if (rx->frame_handler)
                        rx->frame_handler(rx->frame_user_data, &voted);
                    accepted = true;
                    v90_cp_rx_clear_votes(rx);
                }
            }
        }
        v90_cp_rx_reset(rx);
    }
    return accepted;
}
