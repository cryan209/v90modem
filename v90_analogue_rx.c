/*
 * v90_analogue_rx.c — the analogue modem's Phase 3 receiver (§9.3.2), live.
 *
 * See v90_analogue_rx.h for the signals and what separates them.  This file is
 * the state machine that walks them in order and raises the four events
 * v90_analogue_tx.c waits on.
 */

#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v90_analogue_rx.h"

#define JD_BITS             72
#define JD_SYNC_BITS        17
#define JD_PRIME_BITS       12
/* §9.3.2.5: the analogue modem trains its equaliser on the first 2040T of
 * TRN1d, so Jd is not looked for before then. */
#define TRN1D_MIN_SYMBOLS   2040
/* §9.3.2.10 allows 5000 ms from the S in §9.3.2.8 before DIL must end. */
#define DIL_DEADLINE_SYMBOLS 40000
#define DIL_MAX_SYMBOLS     80000
#define DIL_DEFAULT_COVERAGE 0.5
/* Re-measuring on every codeword would be pointless work; a DIL-segment is
 * at least 6 symbols and the answer only changes when segments complete. */
#define DIL_MEASURE_INTERVAL 600
/* Bits of scrambler output needed to bring a descrambler into step (§5.3). */
#define SCRAMBLER_HISTORY   32
/* TRN1d can legitimately run to §9.3.1.5's budget; the Eicon card spends
 * 30005T on it.  Cap the sign buffer well above that and no further. */
#define SIGN_MAX            (1 << 17)
/* How far around the point TRN1d stopped reading as ones to look for the Jd
 * frame.  The mapping change is what breaks it, so the boundary is within a
 * symbol or two; the window is generous rather than tight. */
#define JD_SEARCH_BACK      64
#define JD_SEARCH_FORWARD   8

struct v90_analogue_rx_s {
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_stage_t  stage;

    int      w_ucode;               /* §8.4.4: Ucode(16 + U_INFO) */
    uint8_t  sd_pat[6];             /* {+W, +0, +W, −W, −0, −W} */

    int64_t  index;                 /* codewords consumed */

    /* Sd acquisition: one run length per alignment hypothesis. */
    int      sd_run[6];
    int      sd_phase;              /* index%6 that holds sd_pat[0] */
    int      sd_shift_run;          /* consecutive codewords matching S̄d */

    int64_t  sd_start;
    int64_t  sd_bar_start;
    int64_t  trn1d_start;
    int64_t  jd_start;
    int64_t  dil_start;

    int      sd_reps;
    int      sd_bar_reps;
    int      trn1d_symbols;
    int      jd_symbols;
    int      jd_frames;

    /*
     * Sign-domain decode.  TRN1d and Jd do *not* share a mapping, and this is
     * the trap in §8.4: §8.4.5 puts the scrambler output straight on the sign
     * for TRN1d, while §8.4.2 passes Jd through the differential encoder as
     * well.  Decoding TRN1d differentially yields a clean stream of *zeros* --
     * self-consistent, entirely wrong, and it truncates TRN1d at whatever
     * symbol the search happens to start looking (verified against the Eicon
     * fixtures, which run 30005T).
     *
     * Since the two mappings differ, the raw signs are kept and both are run
     * over them: the TRN1d decoder to see how long TRN1d holds, and a
     * differential decode over a window of candidate starts to find the Jd
     * frame.  A sign buffer costs a byte per symbol and makes that possible.
     */
    uint8_t *signs;                 /* raw sign bits since TRN1d began */
    int      sign_len;
    int      sign_cap;
    int      trn1d_break;           /* first symbol whose plain bit is not 1 */
    int      jd_bit_pos;            /* cursor into signs[] for Jd decoding */
    uint32_t descramble_reg;        /* fed the scrambler output, whatever carries it */
    int      prev_sign;
    uint8_t  jd_bits[JD_BITS];
    int      jd_bit_count;
    bool     jd_valid;
    bool     jd_trn16;
    int      jd_prime_zeros;
    bool     in_jd_frame;

    uint8_t *dil_rx;
    int      dil_len;
    int      dil_cap;
    int      dil_cycle_len;
    int      dil_next_measure;
    v90_dil_measurement_t measurement;
    bool     measurement_valid;
};

/*
 * Match one slot of the Sd / S̄d pattern (§8.4.4).
 *
 * §8.4.4 writes the sequence as {+W, +0, +W, −W, −0, −W}, but Ucode 0 has two
 * G.711 codewords for what is one level at the far D/A, and a working digital
 * modem does not honour the distinction: the Eicon card transmits +0 in *every*
 * zero slot of both Sd and S̄d, on two captured calls the peer Courier answered
 * with CONNECT.  Requiring a polarity on a zero-level symbol rejects real Sd,
 * so match the level on those slots and the whole codeword everywhere else.
 * Same rule the offline scanner reached (vpcm_decode.c, v90_sd_slot_match).
 */
static bool slot_match(const v90_analogue_rx_t *s, uint8_t got, uint8_t want)
{
    int got_ucode;
    int want_ucode;

    v90_codeword_decompose(s->cfg.law, want, &want_ucode, NULL);
    if (want_ucode != 0)
        return got == want;
    v90_codeword_decompose(s->cfg.law, got, &got_ucode, NULL);
    return got_ucode == 0;
}

static int codeword_ucode(const v90_analogue_rx_t *s, uint8_t c, int *sign_out)
{
    int ucode;
    int sign;

    v90_codeword_decompose(s->cfg.law, c, &ucode, &sign);
    if (sign_out)
        *sign_out = sign;
    return ucode;
}

v90_analogue_rx_t *v90_analogue_rx_init(const v90_analogue_rx_config_t *cfg)
{
    v90_analogue_rx_t *s;
    int w;

    if (cfg == NULL  ||  cfg->u_info < 0  ||  cfg->u_info > 127)
        return NULL;
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->cfg = *cfg;
    if (s->cfg.dil_coverage <= 0.0)
        s->cfg.dil_coverage = DIL_DEFAULT_COVERAGE;

    /* §8.4.4: Sd is transmitted at W = Ucode(16 + U_INFO). */
    w = cfg->u_info + 16;
    if (w > 127)
        w = 127;
    s->w_ucode = w;
    s->sd_pat[0] = v90_codeword_compose(cfg->law, w, 1);
    s->sd_pat[1] = v90_codeword_compose(cfg->law, 0, 1);
    s->sd_pat[2] = s->sd_pat[0];
    s->sd_pat[3] = v90_codeword_compose(cfg->law, w, 0);
    s->sd_pat[4] = v90_codeword_compose(cfg->law, 0, 0);
    s->sd_pat[5] = s->sd_pat[3];

    s->sd_phase = -1;
    s->sd_start = -1;
    s->sd_bar_start = -1;
    s->trn1d_start = -1;
    s->jd_start = -1;
    s->dil_start = -1;
    s->stage = V90A_RX_HUNT_SD;

    s->dil_cycle_len = v90_dil_cycle_len(&s->cfg.dil);
    if (s->dil_cycle_len > 0) {
        s->dil_cap = s->dil_cycle_len*2;
        if (s->dil_cap > DIL_MAX_SYMBOLS)
            s->dil_cap = DIL_MAX_SYMBOLS;
        if ((s->dil_rx = malloc((size_t) s->dil_cap)) == NULL) {
            free(s);
            return NULL;
        }
    }
    return s;
}

void v90_analogue_rx_free(v90_analogue_rx_t *s)
{
    if (s) {
        free(s->dil_rx);
        free(s);
    }
}

static int jd_frame_errors(const uint8_t bits[JD_BITS]);

/* GPC descrambling of one scrambler-output bit (§5.3).  Self-synchronising:
 * the register is the transmitted output stream, so 23 bits of any history
 * bring it into step. */
static int descramble(uint32_t *reg, int scrambled)
{
    int plain;

    plain = (scrambled ^ (int) (*reg >> 22) ^ (int) (*reg >> 17)) & 1;
    *reg = ((*reg << 1) | (uint32_t) scrambled) & 0x7FFFFFu;
    return plain;
}

/* Append one received sign to the buffer TRN1d/Jd/J'd are decoded from. */
static bool push_sign(v90_analogue_rx_t *s, uint8_t codeword)
{
    int sign;

    (void) codeword_ucode(s, codeword, &sign);
    if (s->sign_len >= s->sign_cap) {
        int cap = s->sign_cap ? s->sign_cap*2 : 4096;
        uint8_t *grown;

        if (cap > SIGN_MAX)
            return false;
        if ((grown = realloc(s->signs, (size_t) cap)) == NULL)
            return false;
        s->signs = grown;
        s->sign_cap = cap;
    }
    /*endif*/
    s->signs[s->sign_len++] = (uint8_t) sign;
    return true;
}

/*
 * Try to read a Jd frame (§8.4.2) starting at signs[from]: differential
 * decode, then GPC, with the register seeded from the raw signs before it --
 * which are TRN1d's, where the scrambler output *is* the sign (§8.4.5).
 * Returns the frame's error count, or -1 if there is not enough buffered.
 */
static int try_jd_frame(v90_analogue_rx_t *s, int from,
                        uint8_t bits[JD_BITS], uint32_t *reg_out, int *prev_out)
{
    uint32_t reg;
    int prev;
    int i;

    if (from < SCRAMBLER_HISTORY + 1  ||  from + JD_BITS > s->sign_len)
        return -1;
    reg = 0;
    for (i = from - SCRAMBLER_HISTORY; i < from; i++)
        (void) descramble(&reg, s->signs[i]);
    prev = s->signs[from - 1];
    for (i = 0; i < JD_BITS; i++) {
        int sign = s->signs[from + i];
        int scrambled = sign ^ prev;

        prev = sign;
        bits[i] = (uint8_t) descramble(&reg, scrambled);
    }
    if (reg_out)
        *reg_out = reg;
    if (prev_out)
        *prev_out = prev;
    return jd_frame_errors(bits);
}

/*
 * Table 13 structure plus the §10.1.2.3.2/V.34 CRC, which covers the data
 * fields and the CRC field together and must leave a zero remainder.  Same
 * rule the offline decoder applies; a frame that fails it is not a frame.
 */
static int jd_frame_errors(const uint8_t bits[JD_BITS])
{
    uint16_t crc;
    int errors;
    int i;

    crc = 0xFFFF;
    errors = 0;
    for (i = 0; i < 17; i++)
        errors += bits[i] == 0;
    errors += bits[17] != 0;
    errors += bits[34] != 0;
    for (i = 41; i <= 46; i++)
        errors += bits[i] != 0;
    errors += bits[51] != 0;
    for (i = 68; i <= 71; i++)
        errors += bits[i] != 0;
    for (i = 18; i <= 33; i++)
        crc = crc_itu16_bits(bits[i] & 1U, 1, crc);
    for (i = 35; i <= 50; i++)
        crc = crc_itu16_bits(bits[i] & 1U, 1, crc);
    for (i = 52; i <= 67; i++)
        crc = crc_itu16_bits(bits[i] & 1U, 1, crc);
    errors += __builtin_popcount((unsigned) crc);
    return errors;
}

/* §9.3.2.10: has enough DIL arrived to stop asking for more? */
static bool dil_enough(v90_analogue_rx_t *s)
{
    v90_dil_measurement_t m;

    if (s->dil_cycle_len <= 0)
        return true;
    if (s->dil_len >= DIL_DEADLINE_SYMBOLS)
        return true;
    if (s->dil_len < (int) (s->dil_cycle_len*s->cfg.dil_coverage))
        return false;
    /* The DIL starts where J'd ended, so its offset is known exactly and
     * nothing has to be searched for (contrast v90_dil_measure_align(), which
     * exists for captures where it is not). */
    if (!v90_dil_measure(s->dil_rx, s->dil_len, s->cfg.law, &s->cfg.dil, 0, &m))
        return false;
    s->measurement = m;
    s->measurement_valid = true;
    /* Every training Ucode the descriptor asked about has now been seen at
     * least once, which is what §9.3.2.9 sent it for. */
    return m.ucodes_measured > 0  &&  m.coverage >= s->cfg.dil_coverage;
}

static unsigned put_one(v90_analogue_rx_t *s, uint8_t c)
{
    unsigned events;
    int ucode;
    int sign;
    int h;

    events = 0;
    switch (s->stage) {
    case V90A_RX_HUNT_SD:
        /* Six alignment hypotheses run in parallel; whichever holds for four
         * complete repetitions (§8.4.4 sends 64) is the real one. */
        for (h = 0; h < 6; h++) {
            int slot = (int) ((s->index - h)%6);

            if (slot < 0)
                slot += 6;
            if (slot_match(s, c, s->sd_pat[slot]))
                s->sd_run[h]++;
            else
                s->sd_run[h] = 0;
            if (s->sd_run[h] >= 24) {
                s->sd_phase = h;
                s->sd_start = s->index - s->sd_run[h] + 1;
                s->sd_reps = s->sd_run[h]/6;
                s->stage = V90A_RX_SD;
                s->sd_shift_run = 0;
                events |= V90A_RX_EVENT_SD;
                break;
            }
        }
        break;

    case V90A_RX_SD: {
        int slot = (int) ((s->index - s->sd_phase)%6);

        if (slot < 0)
            slot += 6;
        /*
         * §8.4.4's S̄d is Sd with every sign reversed, which is the same six
         * codewords rotated by three slots, so the transition shows up only as
         * a discontinuity against the alignment Sd already established.
         *
         * And only in the W slots.  Slots 1 and 4 carry Ucode 0, whose two
         * codewords are one level (see slot_match), so they match both
         * patterns and say nothing about which one is running.  Counting them
         * as evidence for Sd is what makes the transition invisible: the zero
         * slots keep resetting the count of flipped W slots, and S̄d reads as
         * eight more repetitions of Sd.
         */
        if (slot == 1  ||  slot == 4) {
            if (slot_match(s, c, s->sd_pat[slot]))
                break;
        } else if (slot_match(s, c, s->sd_pat[slot])) {
            s->sd_shift_run = 0;
            s->sd_reps = (int) ((s->index - s->sd_start + 1)/6);
            break;
        } else if (slot_match(s, c, s->sd_pat[(slot + 3)%6])) {
            if (s->sd_shift_run++ == 0)
                s->sd_bar_start = s->index - slot;
            /* Three flipped W slots is most of a repetition and cannot be a
             * coincidence of one symbol. */
            if (s->sd_shift_run >= 3) {
                s->sd_reps = (int) ((s->sd_bar_start - s->sd_start)/6);
                s->sd_bar_reps = 1;
                s->stage = V90A_RX_SD_BAR;
                events |= V90A_RX_EVENT_SD_BAR;
            }
            break;
        }
        /*endif*/
        s->sd_shift_run = 0;
        /* Neither pattern: Sd is over without an S̄d we could see.  Start
         * again rather than pretending Phase 3 advanced. */
        s->stage = V90A_RX_HUNT_SD;
        memset(s->sd_run, 0, sizeof(s->sd_run));
        break;
    }

    case V90A_RX_SD_BAR: {
        int slot = (int) ((s->index - s->sd_phase)%6);

        if (slot < 0)
            slot += 6;
        if (slot_match(s, c, s->sd_pat[(slot + 3)%6])) {
            s->sd_bar_reps = (int) ((s->index - s->sd_bar_start + 1)/6);
            break;
        }
        /* §8.4.5: TRN1d moves to the U_INFO codeword, so the magnitude change
         * is the boundary — no ambiguity with the S̄d pattern above it. */
        ucode = codeword_ucode(s, c, &sign);
        if (ucode == s->cfg.u_info) {
            s->trn1d_start = s->index;
            s->stage = V90A_RX_TRN1D;
            s->descramble_reg = 0;
            s->sign_len = 0;
            s->trn1d_break = -1;
            s->trn1d_symbols = 0;
            events |= V90A_RX_EVENT_TRN1D;
            events |= put_one(s, c);
            return events;
        }
        break;
    }

    case V90A_RX_TRN1D: {
        int plain;

        if (!push_sign(s, c)) {
            /* Nothing sane left to do with a stream this long: TRN1d cannot
             * legitimately outrun §9.3.1.5's budget by this much. */
            s->stage = V90A_RX_HUNT_SD;
            memset(s->sd_run, 0, sizeof(s->sd_run));
            break;
        }
        /*endif*/
        /* §8.4.5: the scrambler output is the sign, with no differential
         * encoder, so scrambled ones descramble to ones directly. */
        plain = descramble(&s->descramble_reg, s->signs[s->sign_len - 1]);
        s->trn1d_symbols++;
        /*
         * §9.3.2.5/§9.3.2.6: the first 2040T trains the equaliser and only
         * then is Jd looked for.  Jd switches the mapping to differential
         * (§8.4.2), so this decoder stops reading ones within a symbol or two
         * of the boundary -- close to it, but not exactly on it.  Take the
         * break as a hint and let the frame decide.
         */
        if (!plain
            && s->trn1d_symbols > TRN1D_MIN_SYMBOLS
            && s->trn1d_break < 0) {
            s->trn1d_break = s->sign_len - 1;
            s->stage = V90A_RX_JD;
            s->jd_bit_pos = -1;
        }
        /*endif*/
        break;
    }

    case V90A_RX_JD: {
        if (!push_sign(s, c))
            break;
        /*endif*/
        if (s->jd_bit_pos < 0) {
            int lo;
            int from;

            /*
             * Find where Jd actually starts, by decoding a frame at each
             * candidate and letting Table 13's structure and CRC answer.  A
             * window is enough: the transmitter switched to differential
             * encoding at the frame boundary, and this side noticed within a
             * couple of symbols.
             */
            lo = s->trn1d_break - JD_SEARCH_BACK;
            if (lo < SCRAMBLER_HISTORY + 1)
                lo = SCRAMBLER_HISTORY + 1;
            if (s->sign_len < s->trn1d_break + JD_SEARCH_FORWARD + JD_BITS)
                break;
            /*endif*/
            for (from = lo; from <= s->trn1d_break + JD_SEARCH_FORWARD; from++) {
                uint8_t bits[JD_BITS];
                uint32_t reg;
                int prev;

                if (try_jd_frame(s, from, bits, &reg, &prev) != 0)
                    continue;
                /*endif*/
                memcpy(s->jd_bits, bits, sizeof(bits));
                s->descramble_reg = reg;
                s->prev_sign = prev;
                s->jd_bit_pos = from + JD_BITS;
                s->jd_start = s->trn1d_start + from;
                s->trn1d_symbols = from;
                s->jd_symbols = JD_BITS;
                s->jd_frames = 1;
                s->jd_valid = true;
                s->jd_trn16 = bits[47] != 0;
                s->jd_bit_count = 0;
                s->in_jd_frame = false;
                s->jd_prime_zeros = 0;
                events |= V90A_RX_EVENT_JD;
                break;
            }
            if (s->jd_bit_pos < 0) {
                /* No frame in the window.  The break was not Jd: keep reading
                 * TRN1d rather than inventing a boundary. */
                s->trn1d_break = -1;
                s->stage = V90A_RX_TRN1D;
            }
            /*endif*/
            break;
        }
        /*endif*/
        /* Decode forward from the located frame, one bit per symbol. */
        while (s->jd_bit_pos < s->sign_len) {
            int sign = s->signs[s->jd_bit_pos];
            int scrambled = sign ^ s->prev_sign;
            int plain;

            s->prev_sign = sign;
            s->jd_bit_pos++;
            s->jd_symbols++;
            plain = descramble(&s->descramble_reg, scrambled);

            if (s->in_jd_frame) {
                s->jd_bits[s->jd_bit_count++] = (uint8_t) plain;
                if (s->jd_bit_count >= JD_BITS) {
                    if (jd_frame_errors(s->jd_bits) == 0) {
                        s->jd_frames++;
                        s->jd_valid = true;
                        s->jd_trn16 = s->jd_bits[47] != 0;
                        events |= V90A_RX_EVENT_JD;
                    }
                    /*endif*/
                    s->jd_bit_count = 0;
                    s->in_jd_frame = false;
                    s->jd_prime_zeros = 0;
                }
                /*endif*/
                continue;
            }
            /*endif*/
            /*
             * At a frame boundary the next bit says which signal follows: Jd
             * repeats and opens with sync ones (§8.4.2), J'd is 12 zeros
             * (§8.4.3).  One bit separates them.
             */
            if (plain) {
                s->jd_bits[0] = 1;
                s->jd_bit_count = 1;
                s->in_jd_frame = true;
                s->jd_prime_zeros = 0;
                continue;
            }
            /*endif*/
            if (++s->jd_prime_zeros >= JD_PRIME_BITS) {
                events |= V90A_RX_EVENT_JD_PRIME;
                if (s->dil_cycle_len > 0) {
                    s->dil_start = s->index + 1;
                    s->dil_next_measure = DIL_MEASURE_INTERVAL;
                    s->stage = V90A_RX_DIL;
                } else {
                    /* §9.3.2.8: a zero-length DIL was requested, so Phase 4 is
                     * next and there is nothing left to receive here. */
                    s->stage = V90A_RX_DONE;
                    events |= V90A_RX_EVENT_DIL_ENOUGH;
                }
                /*endif*/
                break;
            }
            /*endif*/
        }
        break;
    }

    case V90A_RX_DIL:
        if (s->dil_len < s->dil_cap)
            s->dil_rx[s->dil_len++] = c;
        if (s->dil_len >= s->dil_next_measure) {
            s->dil_next_measure = s->dil_len + DIL_MEASURE_INTERVAL;
            if (dil_enough(s)) {
                s->stage = V90A_RX_DONE;
                events |= V90A_RX_EVENT_DIL_ENOUGH;
            }
        }
        break;

    case V90A_RX_DONE:
        break;
    }
    s->index++;
    return events;
}

unsigned v90_analogue_rx_put(v90_analogue_rx_t *s,
                             const uint8_t *codewords,
                             int count)
{
    unsigned events;
    int i;

    events = 0;
    if (s == NULL  ||  codewords == NULL)
        return 0;
    for (i = 0; i < count; i++)
        events |= put_one(s, codewords[i]);
    return events;
}

void v90_analogue_rx_begin_dil(v90_analogue_rx_t *s)
{
    if (s == NULL  ||  s->dil_cycle_len <= 0)
        return;
    s->dil_start = s->index;
    s->dil_len = 0;
    s->dil_next_measure = DIL_MEASURE_INTERVAL;
    s->measurement_valid = false;
    s->stage = V90A_RX_DIL;
}

v90_analogue_rx_stage_t v90_analogue_rx_stage(const v90_analogue_rx_t *s)
{
    return s ? s->stage : V90A_RX_HUNT_SD;
}

const char *v90_analogue_rx_stage_name(v90_analogue_rx_stage_t stage)
{
    switch (stage) {
    case V90A_RX_HUNT_SD: return "hunting Sd";
    case V90A_RX_SD:      return "Sd";
    case V90A_RX_SD_BAR:  return "S-bar_d";
    case V90A_RX_TRN1D:   return "TRN1d";
    case V90A_RX_JD:      return "Jd";
    case V90A_RX_DIL:     return "DIL";
    case V90A_RX_DONE:    return "done";
    }
    return "?";
}

int64_t v90_analogue_rx_sd_start(const v90_analogue_rx_t *s)     { return s ? s->sd_start : -1; }
int64_t v90_analogue_rx_sd_bar_start(const v90_analogue_rx_t *s) { return s ? s->sd_bar_start : -1; }
int64_t v90_analogue_rx_trn1d_start(const v90_analogue_rx_t *s)  { return s ? s->trn1d_start : -1; }
int64_t v90_analogue_rx_jd_start(const v90_analogue_rx_t *s)     { return s ? s->jd_start : -1; }
int64_t v90_analogue_rx_dil_start(const v90_analogue_rx_t *s)    { return s ? s->dil_start : -1; }

int v90_analogue_rx_sd_reps(const v90_analogue_rx_t *s)      { return s ? s->sd_reps : 0; }
int v90_analogue_rx_sd_bar_reps(const v90_analogue_rx_t *s)  { return s ? s->sd_bar_reps : 0; }
int v90_analogue_rx_trn1d_symbols(const v90_analogue_rx_t *s){ return s ? s->trn1d_symbols : 0; }
int v90_analogue_rx_jd_symbols(const v90_analogue_rx_t *s)   { return s ? s->jd_symbols : 0; }
int v90_analogue_rx_jd_frames(const v90_analogue_rx_t *s)    { return s ? s->jd_frames : 0; }
int v90_analogue_rx_dil_symbols(const v90_analogue_rx_t *s)  { return s ? s->dil_len : 0; }

bool v90_analogue_rx_jd_trn16(const v90_analogue_rx_t *s)
{
    return s ? s->jd_trn16 : false;
}

const uint8_t *v90_analogue_rx_jd_bits(const v90_analogue_rx_t *s)
{
    return (s  &&  s->jd_valid) ? s->jd_bits : NULL;
}

const v90_dil_measurement_t *v90_analogue_rx_measurement(const v90_analogue_rx_t *s)
{
    return (s  &&  s->measurement_valid) ? &s->measurement : NULL;
}
