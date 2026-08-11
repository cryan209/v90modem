/*
 * v90_analogue_phase4.c — the analogue modem's Phase 4 receiver (§9.4.2).
 *
 * See v90_analogue_phase4.h for the signals.  The shape of this file is set by
 * one fact: everything after R̄i is mapped through §5.4 with the CPt this side
 * chose, so from that point the receiver is not looking at signs any more, it
 * is running the modulus mapper backwards.  v90_demap_shaped_frame() is that
 * inverse, and it needs three things kept exactly right across the whole of
 * Phase 4 -- the six-symbol frame grid, the §5.3 descrambler, and the §5.4.5
 * shaping state.  §8.6.5 zeroes the last two at TRN2d's first frame, and
 * §8.6.4's R̄i is 24T, a whole number of frames, which fixes the first.
 *
 * So the only thing that has to be *found* here is R, and after that the whole
 * phase is counted rather than searched.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v90_analogue_phase4.h"

/* §8.6.4: R and R̄ are six-symbol sequences; R̄ is exactly four repetitions. */
#define R_PERIOD            6
#define R_BAR_SYMBOLS       24
/* Symbols of history the R hunt matches against — two whole periods.  §9.4.1.1
 * sends Ri for at least 192T, so there is no shortage of them. */
#define R_HISTORY           12
/*
 * Consecutive symbols at the opposite polarity that call §9.4.2.2's transition.
 *
 * Two whole periods, not half of one.  §8.6.4's R̄ reverses *every* sign, so a
 * genuine reversal disagrees on every symbol; a slipped or repeated octet
 * shifts the alignment instead, which disagrees on some slots and agrees on
 * others.  Measured on the card's Ri -- 80 000 runs of exactly three signs with
 * 24 anomalies in thirty seconds -- a three-symbol test fires on those
 * anomalies and reports a transition that never happened.
 */
#define R_REVERSAL_SLOTS    12

/* §9.4.1.2 sends TRN2d for at least 2040T; MP is not looked for before then
 * (§9.4.1.3 allows up to 2000 ms). */
#define TRN2D_MIN_SYMBOLS   2040
/* Plain bits kept while hunting MP's sync run.  Two Type-1 MP frames' worth. */
#define MP_BIT_WINDOW       512
/* §8.6.3 Table 16: Type 0 runs to bit 85 plus fill, Type 1 to bit 187. */
#define MP_TYPE0_BITS       86
#define MP_TYPE1_BITS       188
#define MP_SYNC_BITS        17
/* §8.6.2 and §8.6.1 have exact whole-data-frame lengths. */
#define ED_FRAMES           2
#define B1D_FRAMES          48
#define DATA_BIT_QUEUE      16384

struct v90_analogue_phase4_s {
    v90_analogue_phase4_config_t cfg;
    v90_analogue_phase4_rx_stage_t stage;

    int64_t  index;

    /* R acquisition (§8.6.4): two whole periods of history to match against. */
    uint8_t  hist_sign[R_HISTORY];
    int      hist_ucode[R_HISTORY];
    int      hist_len;
    int      r_ucode;
    int      r_phase;               /* index%6 holding slot 0 */
    int      r_polarity;            /* sign seen in slot 0 when R was acquired */
    int      r_symbols;
    int      r_reversed_slots;
    int      r_bar_symbols;

    /* §5.4 demapping, from TRN2d onwards. */
    int      bits_per_frame;
    uint8_t  frame[6];
    int      frame_fill;
    uint32_t descramble_reg;
    /* §5.4.5: with Sr = 0 the signs are §5.4.5.1's differential chain and
     * there is no shaper; with Sr = 1 to 3 they come out of the shaper and
     * `prev_sign` is unused.  One of the two is live, decided by the CPt. */
    int      prev_sign;
    v90_shaped_rx_state_t shaper;
    int      trn2d_symbols;
    int      trn2d_ones;
    bool     trn2d_broke;
    int      demap_failures;

    /* Plain bits, as a sliding window for the MP hunt. */
    uint8_t  bits[MP_BIT_WINDOW];
    int      bit_len;
    int      ones_run;          /* §8.6.3's sync run, seen from the far side */
    int      mp_candidate;      /* frame start implied by a start bit, or -1 */
    bool     mp_seen;
    int      ed_zero_frames;

    /* §8.6.1/data mode uses CP, with state reset independently of CPt. */
    int      b1d_frames;
    int      b1d_bit_errors;
    uint8_t  data_bits[DATA_BIT_QUEUE];
    int      data_rd;
    int      data_wr;
    int      data_count;
    uint64_t data_dropped;

    /* V.90 §9.6.2.2: data-mode Rd/R̄d acquisition. */
    int      rd_candidate_frames;
    int      rd_polarity;
    int      rd_symbols;
    int      rd_bar_symbols;
    int      rate_renegotiations;
    bool     renegotiation_training;
    bool     silence_cycle;

    int      mp_frames;
    v90_analogue_mp_t mp;
    bool     mp_valid;
};

static int codeword_sign(const v90_analogue_phase4_t *s, uint8_t c, int *ucode)
{
    int sign;

    v90_codeword_decompose(s->cfg.law, c, ucode, &sign);
    return sign;
}

v90_analogue_phase4_t *v90_analogue_phase4_init(const v90_analogue_phase4_config_t *cfg)
{
    v90_analogue_phase4_t *s;

    if (cfg == NULL)
        return NULL;
    /*
     * Table 14: CPt's rate field is (drn + 8) bits per six-symbol frame, where
     * CP's is (drn + 20).  Using the wrong one puts this demapper on a
     * different frame length than the digital modem's mapper.  Sr comes from
     * the same frame and decides how the six sign bits carry it (§5.4.5).
     */
    if (cfg->cpt.shaping_redundancy > 3  ||  cfg->cp.shaping_redundancy > 3)
        return NULL;
    if (v90_analogue_phase4_cp_k(&cfg->cpt) < 0
        || v90_analogue_phase4_cp_k(&cfg->cp) < 0
        || !cfg->cp.v90_compatibility)
        return NULL;
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->cfg = *cfg;
    s->bits_per_frame = cfg->cpt.drn + 8;
    s->stage = V90A4_RX_HUNT_R;
    s->r_phase = -1;
    s->mp_candidate = -1;
    return s;
}

void v90_analogue_phase4_free(v90_analogue_phase4_t *s)
{
    free(s);
}

/* §10.1.2.3.2/V.34 over Table 16's three information blocks, which is the same
 * convention v90.c builds MP with. */
static bool mp_crc_ok(const uint8_t *bits)
{
    uint16_t crc;
    int crc_start;
    int last_data_start;
    int i;

    /* V.90 §8.6.3 Table 16: Type 0 has three 16-bit information blocks
     * following start bits 17/34/51 and its CRC at 69.  Type 1 adds six
     * coefficient/reserved blocks, through start bit 153, and puts its CRC
     * after start bit 170 at 171:186.  The old decoder always checked 69:84,
     * which can only accept Type 0 and leaves a real Type-1 peer apparently
     * transmitting TRN2d forever. */
    if (bits[18]) {
        crc_start = 171;
        last_data_start = 153;
    } else {
        crc_start = 69;
        last_data_start = 51;
    }
    crc = 0xFFFF;
    for (int start = 17; start <= last_data_start; start += 17) {
        for (int bit = start + 1; bit <= start + 16; bit++)
            crc = crc_itu16_bits(bits[bit] & 1U, 1, crc);
    }
    for (i = 0; i < 16; i++) {
        if (bits[crc_start + i] != ((crc >> i) & 1))
            return false;
    }
    return true;
}

/* Table 16's fixed fields.  A frame that fails these is not an MP however
 * plausible the CRC looks. */
static bool mp_structure_ok(const uint8_t *bits, int avail)
{
    int i;

    if (avail < MP_TYPE0_BITS)
        return false;
    for (i = 0; i < MP_SYNC_BITS; i++) {
        if (bits[i] != 1)
            return false;
    }
    /* Start bits 17, 34, 51, 68 and the reserved fields the digital modem
     * sets to zero (19:23, 28, 35, 50). */
    if (bits[17] || bits[34] || bits[51] || bits[68])
        return false;
    for (i = 19; i <= 23; i++) {
        if (bits[i])
            return false;
    }
    if (bits[28] || bits[35] || bits[50])
        return false;
    if (bits[18] == 0) {
        /* Type 0: 52:67 reserved, bit 85 fill. */
        for (i = 52; i <= 67; i++) {
            if (bits[i])
                return false;
        }
        if (bits[85])
            return false;
    } else {
        if (avail < MP_TYPE1_BITS)
            return false;
        /* Type 1: one start bit between every coefficient word, reserved
         * 154:169, start 170, CRC 171:186 and fill bit 187. */
        if (bits[68] || bits[85] || bits[102] || bits[119]
            || bits[136] || bits[153] || bits[170] || bits[187]) {
            return false;
        }
        for (i = 154; i <= 169; i++) {
            if (bits[i])
                return false;
        }
    }
    return mp_crc_ok(bits);
}

static uint32_t get_bits(const uint8_t *bits, int first, int count)
{
    uint32_t v = 0;

    for (int i = 0; i < count; i++)
        v |= (uint32_t) (bits[first + i] & 1U) << i;
    return v;
}

static void mp_decode(const uint8_t *bits, v90_analogue_mp_t *out)
{
    memset(out, 0, sizeof(*out));
    out->type1            = bits[18] != 0;
    out->max_drn          = (uint8_t) get_bits(bits, 24, 4);
    out->trellis          = (uint8_t) get_bits(bits, 29, 2);
    out->nonlinear        = bits[31] != 0;
    out->expanded_shaping = bits[32] != 0;
    out->acknowledge      = bits[33] != 0;
    out->rate_mask        = (uint16_t) get_bits(bits, 36, 13);
    if (out->type1) {
        out->precoder[0][0] = (int16_t) get_bits(bits, 52, 16);
        out->precoder[0][1] = (int16_t) get_bits(bits, 69, 16);
        out->precoder[1][0] = (int16_t) get_bits(bits, 86, 16);
        out->precoder[1][1] = (int16_t) get_bits(bits, 103, 16);
        out->precoder[2][0] = (int16_t) get_bits(bits, 120, 16);
        out->precoder[2][1] = (int16_t) get_bits(bits, 137, 16);
    }
}

/* Append one demapped plain bit, and answer whether it completed something. */
static unsigned push_bit(v90_analogue_phase4_t *s, int bit)
{
    unsigned events = 0;

    if (s->bit_len >= MP_BIT_WINDOW) {
        /* A Type-1 candidate that never completed.  Drop it rather than grow
         * the window: MP repeats, so the next frame costs nothing. */
        s->mp_candidate = -1;
        s->bit_len = 0;
    }
    /*endif*/
    s->bits[s->bit_len++] = (uint8_t) (bit & 1);

    /*
     * Find MP by Table 16's start bit, not by its sync run.
     *
     * The sync run cannot be found directly: §8.6.5's TRN2d is scrambled ones
     * and §8.6.3's MP opens with seventeen more of them, so there is no edge
     * between the two -- exactly the same problem §8.4.2's Jd sets against
     * TRN1d, and with the same answer.  Bit 17 is a zero and everything before
     * it is a one, so the first zero after seventeen or more ones is bit 17,
     * and the frame started seventeen bits earlier.
     */
    if (s->mp_candidate < 0) {
        if (bit == 0  &&  s->ones_run >= MP_SYNC_BITS) {
            s->mp_candidate = s->bit_len - 1 - MP_SYNC_BITS;
            if (s->mp_candidate < 0)
                s->mp_candidate = 0;
            /*endif*/
        }
        /*endif*/
    }
    /*endif*/
    s->ones_run = bit ? (s->ones_run + 1) : 0;

    if (s->mp_candidate >= 0) {
        const uint8_t *frame = s->bits + s->mp_candidate;
        int have = s->bit_len - s->mp_candidate;
        int need = MP_TYPE0_BITS;

        /* Bit 18 says which length to wait for (§8.6.3, Table 16). */
        if (have > 18  &&  frame[18])
            need = MP_TYPE1_BITS;
        /*endif*/
        if (have >= need) {
            if (mp_structure_ok(frame, have)) {
                mp_decode(frame, &s->mp);
                s->mp_valid = true;
                s->mp_frames++;
                s->mp_seen = true;
                if (s->stage == V90A4_RX_TRN2D)
                    s->stage = V90A4_RX_MP;
                /*endif*/
                events |= V90A4_RX_EVENT_MP;
                if (s->mp.acknowledge)
                    events |= V90A4_RX_EVENT_MP_PRIME;
                /*endif*/
            }
            /*endif*/
            /* Resolved either way: the next frame is found from its own start
             * bit, so nothing before this point is needed again. */
            s->mp_candidate = -1;
            s->bit_len = 0;
        }
    } else if (s->bit_len >= MP_BIT_WINDOW - 1) {
        /* No candidate and the window is full: keep only enough to carry the
         * ones run that a start bit would terminate. */
        memmove(s->bits, s->bits + s->bit_len - MP_SYNC_BITS, MP_SYNC_BITS);
        s->bit_len = MP_SYNC_BITS;
    }
    /*endif*/
    return events;
}

static void start_training_mapper(v90_analogue_phase4_t *s, bool renegotiation)
{
    /* §8.6: during rate renegotiation TRN2d/MP/Ed use the preceding data-mode
     * constellation and shaping parameters, but retain K from CPt. */
    s->renegotiation_training = renegotiation;
    s->bits_per_frame = s->cfg.cpt.drn + 8;
    s->descramble_reg = 0;
    s->prev_sign = 0;
    memset(&s->shaper, 0, sizeof(s->shaper));
    s->frame_fill = 0;
    s->bit_len = 0;
    s->ones_run = 0;
    s->mp_candidate = -1;
    s->ed_zero_frames = 0;
    s->trn2d_symbols = 0;
    s->trn2d_ones = 0;
    s->trn2d_broke = false;
    s->mp_frames = 0;
    s->mp_seen = false;
    s->mp_valid = false;
    s->stage = V90A4_RX_TRN2D;
}

static void start_data_mapper(v90_analogue_phase4_t *s)
{
    /* V.90 §8.6.1: CP replaces CPt and the scrambler, differential encoder
     * and spectral-shaping memory are all initialized to zero before B1d. */
    s->bits_per_frame = s->cfg.cp.drn + 20;
    s->descramble_reg = 0;
    s->prev_sign = 0;
    memset(&s->shaper, 0, sizeof(s->shaper));
    s->frame_fill = 0;
    s->b1d_frames = 0;
    s->b1d_bit_errors = 0;
    s->stage = V90A4_RX_B1D;
}

static void queue_data_bit(v90_analogue_phase4_t *s, uint8_t bit)
{
    if (s->data_count >= DATA_BIT_QUEUE) {
        s->data_dropped++;
        return;
    }
    s->data_bits[s->data_wr] = bit & 1U;
    s->data_wr = (s->data_wr + 1)%DATA_BIT_QUEUE;
    s->data_count++;
}

/* §8.6.4 Rd/Rt are R at the highest-power point of each data/training
 * constellation interval. Return slot-0 polarity for +++---/---+++, or -1. */
static int r_frame_polarity(const v90_analogue_phase4_t *s,
                            const vpcm_cp_frame_t *constellations)
{
    int first_sign = -1;

    for (int i = 0; i < 6; i++) {
        int constellation = constellations->dfi[i];
        int highest = -1;
        int ucode;
        int sign;

        if (constellation < 0
            || constellation >= constellations->constellation_count)
            return -1;
        for (int u = VPCM_CP_MASK_BITS - 1; u >= 0; u--) {
            if (vpcm_cp_mask_get(constellations->masks[constellation], u)) {
                highest = u;
                break;
            }
        }
        sign = codeword_sign(s, s->frame[i], &ucode);
        if (ucode != highest)
            return -1;
        if (i == 0)
            first_sign = sign;
        if (sign != ((i < 3) ? first_sign : !first_sign))
            return -1;
    }
    return first_sign;
}

/* Demap one complete six-codeword frame (§5.4) and feed its bits forward. */
static unsigned demap_frame(v90_analogue_phase4_t *s)
{
    const vpcm_cp_frame_t *mapping;
    uint8_t out[64];
    unsigned events = 0;
    bool data_mode;
    int n;

    /* §9.6: Rd/Rt begin on a data-frame boundary and barred R lasts 24T. */
    if (s->stage == V90A4_RX_DATA) {
        int polarity = r_frame_polarity(s, &s->cfg.cp);

        if (polarity >= 0) {
            if (s->rd_candidate_frames == 0 || polarity == s->rd_polarity) {
                s->rd_polarity = polarity;
                s->rd_candidate_frames++;
            } else {
                s->rd_polarity = polarity;
                s->rd_candidate_frames = 1;
            }
            if (s->rd_candidate_frames >= 2) {
                s->rd_symbols = 12;
                s->rd_bar_symbols = 0;
                s->stage = V90A4_RX_RR_RD;
                events |= V90A4_RX_EVENT_RD;
            }
            return events;
        }
        s->rd_candidate_frames = 0;
    } else if (s->stage == V90A4_RX_RR_RD
               || s->stage == V90A4_RX_RR_RT) {
        bool rt = (s->stage == V90A4_RX_RR_RT);
        const vpcm_cp_frame_t *set = rt ? &s->cfg.cpt : &s->cfg.cp;
        int polarity = r_frame_polarity(s, set);

        if (rt && s->rd_symbols == 0) {
            if (polarity < 0) {
                s->rd_candidate_frames = 0;
                return events;
            }
            if (s->rd_candidate_frames == 0 || polarity == s->rd_polarity) {
                s->rd_polarity = polarity;
                s->rd_candidate_frames++;
            } else {
                s->rd_polarity = polarity;
                s->rd_candidate_frames = 1;
            }
            if (s->rd_candidate_frames >= 2) {
                s->rd_symbols = 12;
                s->rd_bar_symbols = 0;
                events |= V90A4_RX_EVENT_RT;
            }
            return events;
        }
        if (polarity == s->rd_polarity) {
            s->rd_symbols += 6;
        } else if (polarity >= 0) {
            s->rd_bar_symbols += 6;
            if (s->rd_bar_symbols >= 12) {
                if (s->stage == V90A4_RX_RR_RT) {
                    s->stage = V90A4_RX_RR_RT_BAR;
                    events |= V90A4_RX_EVENT_RT_BAR;
                } else {
                    s->stage = V90A4_RX_RR_R_BAR;
                    events |= V90A4_RX_EVENT_RD_BAR;
                }
            }
        }
        return events;
    } else if (s->stage == V90A4_RX_RR_R_BAR
               || s->stage == V90A4_RX_RR_RT_BAR) {
        bool rt = (s->stage == V90A4_RX_RR_RT_BAR);
        const vpcm_cp_frame_t *set = rt ? &s->cfg.cpt : &s->cfg.cp;
        int polarity = r_frame_polarity(s, set);

        if (polarity >= 0 && polarity != s->rd_polarity)
            s->rd_bar_symbols += 6;
        if (s->rd_bar_symbols >= R_BAR_SYMBOLS) {
            s->rd_candidate_frames = 0;
            if (rt)
                s->silence_cycle = false;
            else
                s->rate_renegotiations++;
            start_training_mapper(s, true);
            events |= V90A4_RX_EVENT_TRN2D;
        }
        return events;
    }

    data_mode = (s->stage == V90A4_RX_B1D || s->stage == V90A4_RX_DATA);
    mapping = data_mode ? &s->cfg.cp
                        : s->renegotiation_training ? &s->cfg.cp : &s->cfg.cpt;
    /* §5.4.5: Sr = 0 disables spectral shaping, and the signs are then
     * §5.4.5.1's differential chain rather than the shaper's output. */
    if (mapping->shaping_redundancy == 0) {
        n = v90_demap_mapped_frame(s->cfg.law, mapping, s->bits_per_frame,
                                   &s->descramble_reg, &s->prev_sign,
                                   s->frame, out);
    } else {
        n = v90_demap_shaped_frame(s->cfg.law, mapping, s->bits_per_frame,
                                   &s->descramble_reg, &s->shaper, s->frame,
                                   out);
    }
    /*endif*/
    if (n <= 0) {
        /*
         * A frame that will not demap is a frame whose codewords are not in
         * the constellation this side asked for.  Count it: a digital modem
         * that ignored CPt produces nothing but these, and that is a different
         * problem from one that is simply not transmitting yet.
         */
        s->demap_failures++;
        return 0;
    }
    /*endif*/
    if (s->stage == V90A4_RX_B1D) {
        for (int i = 0; i < n; i++) {
            if (!out[i])
                s->b1d_bit_errors++;
        }
        if (s->b1d_frames++ == 0)
            events |= V90A4_RX_EVENT_B1D;
        if (s->b1d_frames >= B1D_FRAMES) {
            s->stage = V90A4_RX_DATA;
            events |= V90A4_RX_EVENT_DATA;
        }
        return events;
    }
    if (s->stage == V90A4_RX_DATA) {
        for (int i = 0; i < n; i++)
            queue_data_bit(s, out[i]);
        return events;
    }

    for (int i = 0; i < n; i++) {
        /* §8.6.5 is scrambled ones from the first frame, so what this counts
         * is the run before the first zero -- which is MP's start bit, and
         * therefore also where TRN2d ended. */
        if (s->stage == V90A4_RX_TRN2D  &&  !s->trn2d_broke) {
            if (out[i])
                s->trn2d_ones++;
            else
                s->trn2d_broke = true;
        }
        /*endif*/
        events |= push_bit(s, out[i]);
    }
    /* Ed starts on a mapping-frame boundary.  Detect its two complete zero
     * frames rather than a bit run: MP's mandatory fill zeroes can precede it
     * and made the old 48-bit heuristic switch constellations mid-frame. */
    if (s->stage == V90A4_RX_MP && s->mp_seen) {
        bool all_zero = true;

        for (int i = 0; i < n; i++) {
            if (out[i]) {
                all_zero = false;
                break;
            }
        }
        s->ed_zero_frames = all_zero ? s->ed_zero_frames + 1 : 0;
        if (s->ed_zero_frames >= ED_FRAMES) {
            events |= V90A4_RX_EVENT_ED;
            if (s->silence_cycle) {
                /* §9.6.2.1.6-.8: digital silence while we recondition, then
                 * Rt/R̄t under the retained training constellations. */
                s->frame_fill = 0;
                s->rd_candidate_frames = 0;
                s->rd_symbols = 0;
                s->rd_bar_symbols = 0;
                s->stage = V90A4_RX_RR_RT;
            } else {
                start_data_mapper(s);
            }
        }
    }
    return events;
}

/*
 * Is the sign history a run of §8.6.4's + + + − − − at alignment `phase`?
 * `phase` is the offset within the history at which slot 0 sits.
 */
static bool r_pattern_at(const v90_analogue_phase4_t *s, int phase, int *polarity)
{
    int first;

    first = s->hist_sign[phase];
    for (int i = 0; i < R_HISTORY; i++) {
        int slot = (i - phase + R_HISTORY)%R_PERIOD;
        int want = (slot < 3) ? first : !first;

        if (s->hist_sign[i] != want)
            return false;
    }
    *polarity = first;
    return true;
}

static unsigned put_one(v90_analogue_phase4_t *s, uint8_t c)
{
    unsigned events = 0;
    int ucode;
    int sign;

    sign = codeword_sign(s, c, &ucode);

    switch (s->stage) {
    case V90A4_RX_HUNT_R:
        /*
         * Slide a window of R_HISTORY codewords along and ask whether it is
         * two whole periods of §8.6.4's pattern at one non-zero level.  Two
         * periods at a fixed magnitude is not something TRN, Ja or silence
         * produce, and §9.4.1.1 sends at least 192T of it.
         */
        memmove(s->hist_sign, s->hist_sign + 1, R_HISTORY - 1);
        memmove(s->hist_ucode, s->hist_ucode + 1,
                (R_HISTORY - 1)*sizeof(s->hist_ucode[0]));
        s->hist_sign[R_HISTORY - 1] = (uint8_t) sign;
        s->hist_ucode[R_HISTORY - 1] = ucode;
        if (s->hist_len < R_HISTORY)
            s->hist_len++;
        /*endif*/
        if (s->hist_len == R_HISTORY  &&  s->hist_ucode[0] != 0) {
            bool level = true;

            for (int i = 1; i < R_HISTORY; i++) {
                if (s->hist_ucode[i] != s->hist_ucode[0]) {
                    level = false;
                    break;
                }
            }
            for (int phase = 0; level  &&  phase < R_PERIOD; phase++) {
                int polarity;

                if (!r_pattern_at(s, phase, &polarity))
                    continue;
                /*endif*/
                s->r_ucode = s->hist_ucode[0];
                s->r_polarity = polarity;
                /* The history's last symbol is at slot (R_HISTORY-1-phase). */
                s->r_phase = (int) ((s->index - (R_HISTORY - 1 - phase))%R_PERIOD);
                if (s->r_phase < 0)
                    s->r_phase += R_PERIOD;
                s->r_symbols = R_HISTORY;
                s->stage = V90A4_RX_R;
                events |= V90A4_RX_EVENT_R;
                break;
            }
        }
        /*endif*/
        break;

    case V90A4_RX_R: {
        int slot = (int) ((s->index - s->r_phase)%R_PERIOD);
        int want;

        if (slot < 0)
            slot += R_PERIOD;
        want = (slot < 3) ? s->r_polarity : !s->r_polarity;
        s->r_symbols++;
        if (ucode != s->r_ucode) {
            /* R is over without a reversal this side could see.  §9.4.2.2 has
             * nothing to act on, so start again rather than pretend. */
            s->stage = V90A4_RX_HUNT_R;
            s->hist_len = 0;
            break;
        }
        /*endif*/
        if (sign == want) {
            /*
             * The pattern broke and then came back: that is a slipped octet
             * moving the alignment, not a reversal.  Re-acquire rather than
             * carry a wrong alignment, which would disagree on some slots for
             * the rest of the call and hide the reversal when it does come.
             */
            if (s->r_reversed_slots > 0)
                s->stage = V90A4_RX_HUNT_R, s->hist_len = 0;
            /*endif*/
            s->r_reversed_slots = 0;
            break;
        }
        /*endif*/
        /* §8.6.4: R̄ is R with every sign reversed, so a sustained run at the
         * opposite polarity is §9.4.2.2's transition -- the same shape as
         * §9.3.2.4's Sd-to-S̄d, and detected the same way. */
        if (++s->r_reversed_slots >= R_REVERSAL_SLOTS) {
            s->stage = V90A4_RX_R_BAR;
            s->r_bar_symbols = s->r_reversed_slots;
            events |= V90A4_RX_EVENT_R_BAR;
        }
        /*endif*/
        break;
    }

    case V90A4_RX_R_BAR:
        /*
         * §8.6.4 fixes R̄ at four repetitions and §9.4.1.2 sends exactly 24T of
         * it, so TRN2d's first symbol is counted rather than searched for --
         * and 24 is a whole number of frames, which is what puts the §5.4
         * frame grid in step with R's alignment.  A level change before then
         * means the count was wrong somewhere, so take that too.
         */
        s->r_bar_symbols++;
        if (s->r_bar_symbols >= R_BAR_SYMBOLS  ||  ucode != s->r_ucode) {
            /* §8.6.5: scrambler, differential encoder and shaping state are
             * all zero at TRN2d's first frame, which is what makes everything
             * after it decodable. */
            start_training_mapper(s, false);
            events |= V90A4_RX_EVENT_TRN2D;
            if (ucode != s->r_ucode) {
                /* This codeword is already TRN2d's first. */
                s->frame[s->frame_fill++] = c;
                s->trn2d_symbols++;
            }
            /*endif*/
        }
        /*endif*/
        break;

    case V90A4_RX_TRN2D:
    case V90A4_RX_MP:
    case V90A4_RX_B1D:
    case V90A4_RX_DATA:
        if (s->stage == V90A4_RX_TRN2D)
            s->trn2d_symbols++;
        /*endif*/
        s->frame[s->frame_fill++] = c;
        if (s->frame_fill == 6) {
            s->frame_fill = 0;
            events |= demap_frame(s);
        }
        /*endif*/
        break;

    case V90A4_RX_RR_RD:
    case V90A4_RX_RR_R_BAR:
    case V90A4_RX_RR_RT:
    case V90A4_RX_RR_RT_BAR:
        s->frame[s->frame_fill++] = c;
        if (s->frame_fill == 6) {
            s->frame_fill = 0;
            /* Handled below by the raw Rd tracker; no modulus demapping is
             * valid while R uses one fixed level per interval. */
            events |= demap_frame(s);
        }
        break;

    }
    s->index++;
    return events;
}

unsigned v90_analogue_phase4_put(v90_analogue_phase4_t *s,
                                 const uint8_t *codewords,
                                 int count)
{
    unsigned events = 0;

    if (s == NULL  ||  codewords == NULL)
        return 0;
    for (int i = 0; i < count; i++)
        events |= put_one(s, codewords[i]);
    return events;
}

v90_analogue_phase4_rx_stage_t v90_analogue_phase4_stage(const v90_analogue_phase4_t *s)
{
    return s ? s->stage : V90A4_RX_HUNT_R;
}

const char *v90_analogue_phase4_stage_name(v90_analogue_phase4_rx_stage_t stage)
{
    switch (stage) {
    case V90A4_RX_HUNT_R: return "hunting Ri";
    case V90A4_RX_R:      return "Ri";
    case V90A4_RX_R_BAR:  return "R-bar_i";
    case V90A4_RX_TRN2D:  return "TRN2d";
    case V90A4_RX_MP:     return "MP/Ed";
    case V90A4_RX_B1D:    return "B1d";
    case V90A4_RX_DATA:   return "data";
    case V90A4_RX_RR_RD:  return "rate renegotiation Rd";
    case V90A4_RX_RR_R_BAR: return "rate renegotiation R-bar-d";
    case V90A4_RX_RR_RT:  return "rate renegotiation Rt";
    case V90A4_RX_RR_RT_BAR: return "rate renegotiation R-bar-t";
    }
    return "?";
}

int v90_analogue_phase4_r_symbols(const v90_analogue_phase4_t *s)
{
    return s ? s->r_symbols : 0;
}

int v90_analogue_phase4_trn2d_symbols(const v90_analogue_phase4_t *s)
{
    return s ? s->trn2d_symbols : 0;
}

int v90_analogue_phase4_mp_frames(const v90_analogue_phase4_t *s)
{
    return s ? s->mp_frames : 0;
}

const v90_analogue_mp_t *v90_analogue_phase4_mp(const v90_analogue_phase4_t *s)
{
    return (s  &&  s->mp_valid) ? &s->mp : NULL;
}

int v90_analogue_phase4_demap_failures(const v90_analogue_phase4_t *s)
{
    return s ? s->demap_failures : 0;
}

int v90_analogue_phase4_trn2d_ones(const v90_analogue_phase4_t *s)
{
    return s ? s->trn2d_ones : 0;
}

int v90_analogue_phase4_b1d_frames(const v90_analogue_phase4_t *s)
{
    return s ? s->b1d_frames : 0;
}

int v90_analogue_phase4_b1d_bit_errors(const v90_analogue_phase4_t *s)
{
    return s ? s->b1d_bit_errors : 0;
}

int v90_analogue_phase4_rate_renegotiations(const v90_analogue_phase4_t *s)
{
    return s ? s->rate_renegotiations : 0;
}

bool v90_analogue_phase4_start_rate_renegotiation(v90_analogue_phase4_t *s,
                                                   bool silence_request)
{
    if (s == NULL || s->stage != V90A4_RX_DATA)
        return false;
    s->silence_cycle = silence_request;
    s->rd_candidate_frames = 0;
    return true;
}

int v90_analogue_phase4_get_data_bits(v90_analogue_phase4_t *s,
                                      uint8_t *bits, int max_bits)
{
    int n = 0;

    if (s == NULL || bits == NULL || max_bits <= 0)
        return 0;
    while (n < max_bits && s->data_count > 0) {
        bits[n++] = s->data_bits[s->data_rd];
        s->data_rd = (s->data_rd + 1)%DATA_BIT_QUEUE;
        s->data_count--;
    }
    return n;
}

int v90_analogue_phase4_cp_k(const vpcm_cp_frame_t *cp)
{
    uint64_t product = 1;
    int d;
    int k;

    if (cp == NULL  ||  cp->drn == 0  ||  cp->shaping_redundancy > 3)
        return -1;
    /* Table 14 bit 19 picks the rate encoding, and §5.4.1 takes the sign bits
     * back out of it. */
    d = cp->drn + (cp->v90_compatibility ? 20 : 8);
    k = d - (6 - cp->shaping_redundancy);
    if (k < 1  ||  k > 56)
        return -1;
    /*
     * Table 17: Phase 4's K runs from 6 to 24, where data mode's (Table 2)
     * runs to 39.  A CPt above 24 is the one number a digital modem cannot
     * work around -- §8.6.5 maps TRN2d with it, so it has nothing to train on
     * and §9.4.1.2 never answers.
     */
    if (!cp->v90_compatibility  &&  (k < 6  ||  k > 24))
        return -1;
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        int c = cp->dfi[i];
        int mi;

        if (c < 0  ||  c >= cp->constellation_count)
            return -1;
        if ((mi = vpcm_cp_mask_population(cp->masks[c])) <= 0)
            return -1;
        product *= (uint64_t) mi;
    }
    /* §5.4.3: 2^K <= prod(Mi), or the modulus encoder has more to place than
     * the constellations hold. */
    if (k < 64  &&  product < (1ULL << k))
        return -1;
    return k;
}

bool v90_analogue_phase4_build_cp(const v90_dil_measurement_t *m,
                                  v90_law_t law,
                                  double max_tx_dbm0,
                                  int shaping_redundancy,
                                  int shaping_lookahead,
                                  vpcm_cp_frame_t *cpt_out,
                                  vpcm_cp_frame_t *cp_out)
{
    v90_dil_rate_plan_t plan;
    v90_dil_rate_plan_t training_plan;
    vpcm_cp_frame_t cp;
    vpcm_cp_frame_t cpt;
    int s;
    int k;

    if (m == NULL  ||  cpt_out == NULL  ||  cp_out == NULL)
        return false;
    if (shaping_redundancy < 0  ||  shaping_redundancy > 3)
        return false;
    /* §8.5.2: ld is the look-ahead requested *during spectral shaping*, so
     * with §5.4.5's shaping disabled there is nothing for it to describe. */
    if (shaping_lookahead < 0  ||  shaping_lookahead > 3
        ||
        (shaping_redundancy == 0  &&  shaping_lookahead != 0)) {
        return false;
    }
    /* The same call the engine already makes to report the rate: measured
     * noise from the bottom of the ladder, §8.5.2's Table 15 cap from the top
     * (docs/v90_constellation_selection.md).  Sr goes in here rather than onto
     * the frame afterwards because it decides the rate the plan comes back
     * with. */
    /* V.90 §8.5.2/Table 15: CP's average power must fit the maximum the
     * digital modem announced in INFO0d bits 33:37.  Passing zero here used
     * to disable the cap on every live call, producing an all-65-point CP at
     * 56 kbit/s even when the peer had explicitly offered less power. */
    if (!v90_dil_measure_plan_rate_sr(m, 0, 3.0, max_tx_dbm0, law,
                                      shaping_redundancy, &plan))
        return false;
    /* CPt is the Phase-4 training constellation, not the data-mode power
     * offer.  Keep the impairment/noise plan the peer already accepted in
     * run 81 and apply Table 15 only to CP.  Reusing the power-thinned CP set
     * for CPt changed the training constellation itself and the Eicon fell
     * from 0x00b2 back to Phase 3 before ever sending Ri (run 86).  Since the
     * bounded CP is a low-level subset of this set, §8.5.2's other condition
     * (CP no more than 3 dB above CPt) remains satisfied in the safe direction. */
    if (!v90_dil_measure_plan_rate_sr(m, 0, 3.0, 0.0, law,
                                      shaping_redundancy, &training_plan))
        return false;
    if (plan.intervals_unprobed != 0  ||  training_plan.intervals_unprobed != 0)
        return false;

    vpcm_cp_init(&cp);
    cp.v90_compatibility = true;        /* Table 14 bit 19: 1 = CP */
    cp.drn = plan.drn;
    cp.codec_alaw = (law == V90_LAW_ALAW);
    cp.shaping_redundancy = (uint8_t) shaping_redundancy;
    cp.shaping_lookahead = (uint8_t) shaping_lookahead;
    /*
     * Table 14 bits 52:67 -- the RMS of TRN1d at the digital modem's output
     * over its RMS at the codec's D/A, unsigned Q3.13.  The measurement's
     * gain_db is the reciprocal of that ratio in dB, which is where a digital
     * pad in the path shows up, so this is the field that tells the digital
     * modem about it.
     */
    {
        double ratio = pow(10.0, -m->gain_db/20.0);
        double q = ratio*8192.0;

        if (!(q >= 0.0))
            q = 8192.0;
        if (q > 65535.0)
            q = 65535.0;
        cp.trn1d_gain_q3_13 = (uint16_t) (q + 0.5);
    }
    /* Bits 36:48 -- what this side's transmitter supports upstream.  INFO1a
     * offered 3200 baud to 31200 bit/s, so offer the same ladder and no more:
     * §9.4.2.4 picks from the intersection of this and MP's. */
    cp.upstream_rate_mask = 0x0FFF;     /* 4800 … 31200 */
    cp.constellation_count = VPCM_CP_FRAME_INTERVALS;
    cp.codec_constellations_differ = false;
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        cp.dfi[i] = (uint8_t) i;
        memcpy(cp.masks[i], plan.mask[i], VPCM_CP_MASK_BYTES);
    }

    /*
     * CPt.  §8.5.2 caps CP's average power at 3 dB above CPt's.  CP uses the
     * Table-15-limited data plan above; CPt keeps the impairment/noise-only
     * training plan.  The former drops high levels from the latter, so CP is
     * on the safe side of the 3 dB relation without changing the Phase-4
     * constellation the peer trains against.  The quantity to carry across
     * for CPt's rate is K, not the
     * field -- Table 14 encodes D differently in the two ((drn + 20) against
     * (drn + 8)) while §5.4.3 and §8.6.5 are both about K.
     *
     * Table 17 then caps Phase 4's K at 24 where data mode's runs to 39, so a
     * fast line's CPt trains at a lower rate than it will carry.  That costs
     * nothing: fewer bits per frame over the same constellation is always
     * encodable, and TRN2d and MP do not need the line's full rate.
     */
    s = 6 - shaping_redundancy;
    k = training_plan.k;
    if (k > 24)
        k = 24;
    cpt = cp;
    cpt.v90_compatibility = false;      /* Table 14 bit 19: 0 = CPt */
    cpt.constellation_count = VPCM_CP_FRAME_INTERVALS;
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++) {
        cpt.dfi[i] = (uint8_t) i;
        memcpy(cpt.masks[i], training_plan.mask[i], VPCM_CP_MASK_BYTES);
    }
    if (k + s - 8 < 1  ||  k + s - 8 > 22)
        return false;
    cpt.drn = (uint8_t) (k + s - 8);

    if (!vpcm_cp_validate(&cp, NULL, 0)  ||  !vpcm_cp_validate(&cpt, NULL, 0))
        return false;
    /*
     * The check the digital modem is about to make.  Getting here with a frame
     * that fails it is how a Phase 4 stalls with the line looking perfect:
     * §9.4.1.2 sends R̄i only after *receiving* a CPt, and a CPt it cannot
     * build a mapper from is not one it has received.
     */
    if (v90_analogue_phase4_cp_k(&cpt) < 0  ||  v90_analogue_phase4_cp_k(&cp) < 0)
        return false;
    *cp_out = cp;
    *cpt_out = cpt;
    return true;
}
