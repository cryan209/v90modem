/*
 * v90_analogue_rx.c — the analogue modem's Phase 3 receiver (§9.3.2), live.
 *
 * See v90_analogue_rx.h for the signals and what separates them.  This file is
 * the state machine that walks them in order and raises the four events
 * v90_analogue_tx.c waits on.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

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
/* How much of TRN1d has to read as §8.4.5's ones before the level the
 * S̄d→TRN1d boundary was matched at is believed. */
#define TRN1D_CONFIRM_SYMBOLS 256
/* TRN1d can legitimately run to §9.3.1.5's budget; the Eicon card spends
 * 30005T on it.  Cap the sign buffer well above that and no further. */
#define SIGN_MAX            (1 << 17)
/* How far around the point TRN1d stopped reading as ones to look for the Jd
 * frame.  The mapping change is what breaks it, so the boundary is within a
 * symbol or two; the window is generous rather than tight. */
#define JD_SEARCH_BACK      64
#define JD_SEARCH_FORWARD   8
/* Consecutive codewords off the Jd/J'd level that say DIL has started.  Jd and
 * J'd never leave it, so this cannot fire early; §8.4.1's first segment can sit
 * on it, so a handful of symbols rather than one. */
#define JD_EXIT_SYMBOLS     8

struct v90_analogue_rx_s {
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_stage_t  stage;

    int      w_ucode;               /* §8.4.4: Ucode(16 + U_INFO) */
    int      trn1d_ucode;           /* §8.4.5: Ucode(U_INFO) = w_ucode - 16 */
    uint8_t  sd_pat[6];             /* {+W, +0, +W, −W, −0, −W} */

    int64_t  index;                 /* codewords consumed */

    /* Sd acquisition: one run length per alignment hypothesis. */
    int      sd_run[6];
    int      sd_w[6];               /* W learned from the wire, 0 until seen */
    /*
     * The same W as a running mean LEVEL, for the level-tolerant comparison.
     * sd_w[] is latched from the first non-zero codeword a hypothesis sees,
     * which on an analogue line is one noisy sample: measured on
     * artifacts/hsf-v90/call-062754Z it latched Ucode 96 (level 4158) where
     * the true median W level is 3326 -- 25% high, which puts 5.3% of the W
     * slots outside a 35% tolerance and kills a 24-codeword run more often
     * than not.  A run of 4, 28 and then 6 repetitions is exactly that.
     */
    double   sd_w_lvl[6];
    int      sd_miss_run;           /* consecutive unusable slots inside Sd */
    /*
     * CMA is blind to sign -- driving |y| to a modulus leaves y -> -y equally
     * valid -- so a blindly equalised stream arrives at an arbitrary polarity
     * that has to be resolved once.  §8.4.5's plaintext is all ones, so an
     * inverted stream descrambles to all ZEROS, which is what resolves it.
     */
    int      sign_flip;
    int      sign_flip_tried;
    int      jd_best_errors;        /* best §8.4.2 frame seen while probing */
    void   (*jd_probe_report)(void *user, int errors, int at);
    void    *jd_probe_user;
    void   (*trn1d_report)(void *user, int ones, int of);
    void    *trn1d_report_user;
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
    int      trn1d_scan;            /* cursor into signs[] for TRN1d decoding */
    int      trn1d_ones;            /* how much of it read as §8.4.5's ones */
    int      jd_bit_pos;            /* cursor into signs[] for Jd decoding */
    uint32_t descramble_reg;        /* fed the scrambler output, whatever carries it */
    int      prev_sign;
    uint8_t  jd_bits[JD_BITS];
    int      jd_bit_count;
    bool     jd_valid;
    bool     jd_trn16;
    int      jd_prime_zeros;
    bool     in_jd_frame;
    int      jd_exit_run;           /* consecutive codewords off the Jd level */
    uint8_t  jd_exit_hold[JD_EXIT_SYMBOLS];

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
/* §8.4.4's slots compared in LEVEL rather than by codeword; see
 * zero_slot_fraction in the config.  Returns false when no tolerance is
 * configured, so the caller keeps its exact comparison. */
static bool level_tolerant(const v90_analogue_rx_t *s)
{
    return s->cfg.zero_slot_fraction > 0.0;
}

static double ucode_level(const v90_analogue_rx_t *s, int ucode)
{
    uint8_t c = v90_codeword_compose(s->cfg.law, ucode, 1);
    int lin = (s->cfg.law == V90_LAW_ALAW) ? alaw_to_linear(c)
                                           : ulaw_to_linear(c);
    return fabs((double) lin);
}

/* The Ucode whose level is closest to a measured one -- the inverse of
 * ucode_level(), by search, so no ladder arithmetic is duplicated. */
static int nearest_ucode(const v90_analogue_rx_t *s, double level)
{
    int best = 0, u;
    double bestd = -1.0;

    for (u = 0; u < 128; u++) {
        double d = fabs(ucode_level(s, u) - level);

        if (bestd < 0.0  ||  d < bestd) {
            bestd = d;
            best = u;
        }
    }
    return best;
}

static bool level_is_zero_slot(const v90_analogue_rx_t *s, int ucode, int w)
{
    if (w <= 0)
        return ucode == 0;
    return ucode_level(s, ucode)
           <= s->cfg.zero_slot_fraction*ucode_level(s, w);
}

static bool level_is_w_slot(const v90_analogue_rx_t *s, int ucode, int w)
{
    double lw, lu, tol;

    if (w <= 0)
        return false;
    lw = ucode_level(s, w);
    lu = ucode_level(s, ucode);
    tol = (s->cfg.w_slot_tolerance > 0.0) ? s->cfg.w_slot_tolerance : 0.35;
    return fabs(lu - lw) <= tol*lw;
}

static bool slot_match(const v90_analogue_rx_t *s, uint8_t got, uint8_t want)
{
    int got_ucode;
    int want_ucode;
    int got_sign;
    int want_sign;

    v90_codeword_decompose(s->cfg.law, want, &want_ucode, &want_sign);
    v90_codeword_decompose(s->cfg.law, got, &got_ucode, &got_sign);
    if (level_tolerant(s)) {
        if (want_ucode == 0)
            return level_is_zero_slot(s, got_ucode, s->w_ucode);
        return got_sign == want_sign
               &&  level_is_w_slot(s, got_ucode, want_ucode);
    }
    if (want_ucode != 0)
        return got == want;
    return got_ucode == 0;
}

static int codeword_ucode(const v90_analogue_rx_t *s, uint8_t c, int *sign_out);

/*
 * One slot of the Sd hunt, matched *structurally* rather than against a
 * precomputed pattern, learning W as it goes.
 *
 * §8.4.4 puts Sd at W = Ucode(16 + U_INFO), and U_INFO is the analogue
 * modem's own choice, announced in INFO1a bits 25:31 (§8.2.3.2 Table 10).
 * The hunt used to build the six codewords from that announced value and
 * accept nothing else, which is only correct against a digital modem that
 * honours it.  An Eicon Diva Server does not: told U_INFO = 78 it transmits
 * Sd at W = 64, and told 48 it transmits at W = 35 (measured live, run 57,
 * both calls also carrying a textbook 64-rep Sd, 8-rep S̄d and 30005T TRN1d).
 * Both in-tree fixtures in artifacts/eicon-digital-downstream/ -- calls a
 * Courier answered with CONNECT -- likewise run at W = 64.  A receiver pinned
 * to the value it requested is blind to all of them, and loopback cannot show
 * it: our own transmitter uses the same variable.
 *
 * So take W off the wire.  The structure is what identifies Sd: four W slots
 * at one common non-zero level with signs + + − −, and two zero slots.  The
 * offline scanner reached the same conclusion from the other direction, by
 * scanning every U_INFO (vpcm_decode.c).
 */
static bool sd_hunt_slot(v90_analogue_rx_t *s, int h, int slot, uint8_t c)
{
    int ucode;
    int sign;

    ucode = codeword_ucode(s, c, &sign);
    /* Slots 1 and 4 are Ucode 0, which is one level at the far D/A whatever
     * the sign bit says -- the same rule slot_match() applies. */
    if (slot == 1  ||  slot == 4) {
        if (level_tolerant(s))
            return level_is_zero_slot(s, ucode, s->sd_w[h]);
        return ucode == 0;
    }
    if (s->sd_w[h] == 0) {
        if (ucode == 0)
            return false;
        s->sd_w[h] = ucode;
        s->sd_w_lvl[h] = ucode_level(s, ucode);
    }
    if (level_tolerant(s)) {
        double lu = ucode_level(s, ucode);
        double ref = (s->sd_w_lvl[h] > 0.0) ? s->sd_w_lvl[h]
                                            : ucode_level(s, s->sd_w[h]);
        double tol = (s->cfg.w_slot_tolerance > 0.0)
                     ? s->cfg.w_slot_tolerance : 0.35;

        if (fabs(lu - ref) > tol*ref)
            return false;
        /* Walk the reference onto the population rather than trusting the
         * sample that opened the run. */
        s->sd_w_lvl[h] = ref + 0.25*(lu - ref);
        s->sd_w[h] = nearest_ucode(s, s->sd_w_lvl[h]);
    } else if (ucode != s->sd_w[h]) {
        return false;
    }
    return sign == ((slot == 0  ||  slot == 2)  ?  1  :  0);
}

/* Rebuild the Sd codeword pattern once acquisition has settled on a W.  Every
 * stage after acquisition works from sd_pat, so this is the only place the
 * learned value has to be installed. */
static void sd_set_w(v90_analogue_rx_t *s, int w)
{
    s->w_ucode = w;
    /*
     * §8.4.5 puts TRN1d at Ucode(U_INFO) while §8.4.4 puts Sd at
     * Ucode(16 + U_INFO), so on a peer that honours U_INFO the two indices are
     * 16 apart.  This is only a starting guess: a peer that ignores U_INFO for
     * Sd (see sd_hunt_slot) ignores it here too, and what it holds instead is
     * the *level* ratio those two indices imply, not the index gap.
     *
     * Measured on the Eicon card: Sd at Ucode 64 (µ-law magnitude 495) is
     * followed by TRN1d at Ucode 48 (231), and Sd at Ucode 35 (123) by TRN1d
     * at Ucode 22 (57).  Both are a factor of 2.15 -- about 6.6 dB -- but only
     * the first is a gap of 16.  §8.4.5's arithmetic coincides with the level
     * ratio exactly when Sd lands on a segment boundary (mantissa 0), which is
     * what Ucode 64 is, and what both in-tree fixtures happen to use.  The gap
     * of 16 read as a rule for four months on the strength of that coincidence.
     *
     * So the real value is taken off the wire in V90A_RX_SD_BAR; this is what
     * the hunt starts from.
     */
    s->trn1d_ucode = (w >= 16)  ?  (w - 16)  :  0;
    s->sd_pat[0] = v90_codeword_compose(s->cfg.law, w, 1);
    s->sd_pat[1] = v90_codeword_compose(s->cfg.law, 0, 1);
    s->sd_pat[2] = s->sd_pat[0];
    s->sd_pat[3] = v90_codeword_compose(s->cfg.law, w, 0);
    s->sd_pat[4] = v90_codeword_compose(s->cfg.law, 0, 0);
    s->sd_pat[5] = s->sd_pat[3];
}

static int codeword_ucode(const v90_analogue_rx_t *s, uint8_t c, int *sign_out)
{
    int ucode;
    int sign;

    v90_codeword_decompose(s->cfg.law, c, &ucode, &sign);
    if (sign_out)
        *sign_out = s->sign_flip ? !sign : sign;
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

    /* §8.4.4: Sd is transmitted at W = Ucode(16 + U_INFO).  That is what we
     * requested, so seed the pattern with it; acquisition replaces it with
     * whatever the digital modem actually sends (see sd_hunt_slot()). */
    w = cfg->u_info + 16;
    if (w > 127)
        w = 127;
    sd_set_w(s, w);

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
 * Put the TRN1d decoder back on signs[from] with a descrambler in step there.
 *
 * §5.3's GPC is self-synchronising, so seeding is a matter of running the
 * preceding SCRAMBLER_HISTORY signs through it and throwing the output away.
 * Called when a Jd search fails and the scan has to be resumed behind where
 * the search read to.
 */
static void trn1d_resume(v90_analogue_rx_t *s, int from)
{
    int i;

    if (from < 0)
        from = 0;
    if (from > s->sign_len)
        from = s->sign_len;
    s->descramble_reg = 0;
    for (i = (from > SCRAMBLER_HISTORY) ? (from - SCRAMBLER_HISTORY) : 0; i < from; i++)
        (void) descramble(&s->descramble_reg, s->signs[i]);
    s->trn1d_scan = from;
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
    if (m.ucodes_measured <= 0  ||  m.coverage < s->cfg.dil_coverage)
        return false;
    /*
     * ...and "enough" has to mean enough FOR THE THING IT FEEDS.
     *
     * §9.3.2.10 lets the analogue modem stop once it has enough, and a
     * coverage fraction is a reasonable proxy -- every DIL segment measures
     * one training Ucode independently, so half a pass really does measure
     * half the ladder.  But the consumer is v90_analogue_phase4_build_cp(),
     * which rejects any plan with intervals_unprobed set BEFORE it looks at
     * the noise margin, and an interval is unprobed when fewer than two of its
     * levels are distinguishable.  Half the ladder does not guarantee that for
     * all six.
     *
     * Measured live: three calls stopped at 53% coverage with 65 of 121 Ucodes
     * and then built no constellation at ANY margin -- "ignoring noise it
     * would be still empty" -- while the digital modem sat in Phase 4 Ri
     * waiting for the CPt.  So ask the builder's own question instead: keep
     * measuring until a plan exists with the noise ignored.  The
     * DIL_DEADLINE_SYMBOLS backstop above still bounds it, and a line that
     * cannot produce one by then is not one a longer DIL will rescue.
     */
    if (s->cfg.dil_require_plan) {
        v90_dil_rate_plan_t plan;

        if (!v90_dil_measure_plan_rate(&m, 0, 0.0, 0.0, s->cfg.law, &plan))
            return false;
    }
    return true;
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
            if (sd_hunt_slot(s, h, slot, c)) {
                s->sd_run[h]++;
            } else {
                s->sd_run[h] = 0;
                s->sd_w[h] = 0;
                s->sd_w_lvl[h] = 0.0;
            }
            /*endif*/
            if (s->sd_run[h] >= 24) {
                s->sd_phase = h;
                s->sd_start = s->index - s->sd_run[h] + 1;
                s->sd_reps = s->sd_run[h]/6;
                /* Install the W four complete repetitions agreed on.  It is
                 * usually 16 + the U_INFO we announced, but §8.4.4 is only
                 * binding on a peer that honours the request -- see
                 * sd_hunt_slot(). */
                sd_set_w(s, s->sd_w[h]);
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
            /*
             * A zero slot says nothing about WHICH pattern is running -- it is
             * the same in Sd and S̄d, which is why the matching branch above
             * simply breaks.  So a zero slot that does not match says nothing
             * either, and must not be read as "Sd is over".
             *
             * It is exactly what the §9.3.2.4 transition looks like on a real
             * line: the reversal lands inside a repetition, and that straddle
             * repetition's zero slots carry the discontinuity.  Measured on
             * the equalised stream of artifacts/hsf-v90/call-062754Z, Sd runs
             * 33 clean repetitions, repetition 33 reads zero/W = 1.120, and
             * repetitions 34 on are cleanly reversed -- so the receiver was
             * resetting to the hunt on the very repetition carrying the
             * transition it was waiting for, every time.
             *
             * Only where the slots are compared by level: on the digital
             * bearer a zero slot that is not Ucode 0 is a real break.
             */
            if (level_tolerant(s))
                break;
        } else if (slot_match(s, c, s->sd_pat[slot])) {
            s->sd_shift_run = 0;
            s->sd_miss_run = 0;
            s->sd_reps = (int) ((s->index - s->sd_start + 1)/6);
            break;
        } else if (slot_match(s, c, s->sd_pat[(slot + 3)%6])) {
            s->sd_miss_run = 0;
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
        /*
         * Neither pattern.  On the digital bearer that means Sd is over
         * without an S̄d we could see, and starting again is right.
         *
         * On an analogue line it is also what the §9.3.2.4 transition itself
         * looks like: the reversal is a discontinuity and the equaliser rings
         * through it, so the straddle repetition is unusable.  Measured on
         * artifacts/hsf-v90/call-062754Z, repetition 33 carries W slots at
         * 0.41, 0.96 and 0.49 of the running W while 32 clean repetitions
         * precede it and repetition 34 is cleanly reversed -- so one bad
         * repetition stood between the receiver and the transition, every
         * time.  Ride out up to one repetition of unusable slots without
         * losing the alignment or the flipped-slot evidence; a signal that has
         * really stopped being Sd exhausts that within a repetition anyway.
         */
        if (level_tolerant(s)  &&  ++s->sd_miss_run <= 6)
            break;
        s->sd_shift_run = 0;
        s->sd_miss_run = 0;
        s->stage = V90A_RX_HUNT_SD;
        memset(s->sd_run, 0, sizeof(s->sd_run));
        memset(s->sd_w, 0, sizeof(s->sd_w));
        memset(s->sd_w_lvl, 0, sizeof(s->sd_w_lvl));
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
        /*
         * §8.4.5: TRN1d moves to a lower codeword, so the magnitude change is
         * the boundary — and S̄d itself only ever carries two magnitudes, W and
         * zero, so *any* third one is that boundary.  Match it that way rather
         * than against the level §8.4.5 predicts: the prediction is only good
         * on a peer that honoured U_INFO, and this one does not (sd_set_w).
         * Taking the level off the wire is the same rule the W hunt applies
         * one stage earlier, and for the same reason.
         *
         * The reading is confirmed rather than assumed: trn1d_check counts how
         * much of the first TRN1D_CONFIRM_SYMBOLS descrambles to the ones
         * §8.4.5 says are there, and a level that does not is abandoned.
         */
        ucode = codeword_ucode(s, c, &sign);
        /*
         * "A third magnitude" has to be judged the same way the slots are.
         * With an exact comparison every symbol whose Ucode wanders off W by
         * one reads as a third magnitude, so on an analogue line this fires
         * almost immediately and carries a wrong TRN1d level with it.
         */
        if (level_tolerant(s)
            ?  (!level_is_zero_slot(s, ucode, s->w_ucode)
                &&  !level_is_w_slot(s, ucode, s->w_ucode))
            :  (ucode != 0  &&  ucode != s->w_ucode)) {
            s->trn1d_ucode = ucode;
            s->trn1d_start = s->index;
            s->stage = V90A_RX_TRN1D;
            s->descramble_reg = 0;
            s->sign_len = 0;
            s->trn1d_scan = 0;
            s->trn1d_break = -1;
            s->trn1d_symbols = 0;
            s->trn1d_ones = 0;
            events |= V90A_RX_EVENT_TRN1D;
            events |= put_one(s, c);
            return events;
        }
        break;
    }

    case V90A_RX_TRN1D: {
        int confirm_at;
        int confirm_of;
        if (!push_sign(s, c)) {
            /* Nothing sane left to do with a stream this long: TRN1d cannot
             * legitimately outrun §9.3.1.5's budget by this much. */
            s->stage = V90A_RX_HUNT_SD;
            memset(s->sd_run, 0, sizeof(s->sd_run));
            memset(s->sd_w, 0, sizeof(s->sd_w));
            break;
        }
        /*endif*/
        /*
         * LET THE CRC FIND THE BOUNDARY.
         *
         * The §8.4.5 -> §8.4.2 transition below is taken on a single
         * descrambled zero, which is right where the codewords are exact --
         * §8.4.5's plaintext is all ones, so the first zero IS Jd.  Where the
         * levels are sliced there is a residual sign-error rate (§8.4.5's own
         * confirmation reads 95-100%, not 100%), so the first zero after the
         * training interval is almost always a stray error tens of thousands
         * of symbols early; the frame search then hunts a window that cannot
         * contain Jd, fails back to TRN1d, and the next error repeats it.  A
         * windowed zero-RATE was tried instead and is not enough either: error
         * bursts reach 25% locally where the mean is 3.5%, so 8-in-32 fired at
         * 2593T against the peer's 20004T.
         *
         * So do not guess where TRN1d ends.  Table 13's structure and CRC are
         * ground truth -- 17 sync ones, nine fixed zeros and a 16-bit CRC, ~42
         * bits of constraint, so a false frame is a 2^-42 event -- and TRN1d
         * cannot counterfeit one, because its signs carry the scrambler output
         * directly while try_jd_frame() applies §8.4.2's differential decode.
         * Probe the newest complete frame on every symbol and leave only when
         * one VALIDATES.  That owes nothing to a threshold, and it is what the
         * code already says one stage on: "take the break as a hint and let
         * the frame decide".
         */
        if (level_tolerant(s)  &&  s->trn1d_symbols > TRN1D_MIN_SYMBOLS
            &&  s->sign_len >= JD_BITS) {
            uint8_t bits[JD_BITS];
            uint32_t reg;
            int prev;
            int from = s->sign_len - JD_BITS;

            int errs = try_jd_frame(s, from, bits, &reg, &prev);

            /*
             * How CLOSE the best candidate comes, not merely whether one
             * validated.  "Jd 0 frames" is the same report for a boundary in
             * the wrong place and for a stream too noisy to carry a 72-bit
             * frame, and those want opposite fixes.  jd_frame_errors() counts
             * violated structure bits plus CRC bits, so a handful means
             * marginal and tens means nowhere near.
             */
            if (errs >= 0  &&  (s->jd_best_errors < 0  ||  errs < s->jd_best_errors)) {
                s->jd_best_errors = errs;
                if (s->jd_probe_report)
                    s->jd_probe_report(s->jd_probe_user, errs,
                                       s->trn1d_symbols);
            }
            if (errs == 0) {
                memcpy(s->jd_bits, bits, sizeof(bits));
                s->descramble_reg = reg;
                s->prev_sign = prev;
                s->jd_bit_pos = from + JD_BITS;
                s->jd_bit_count = JD_BITS;
                s->trn1d_break = from;
                s->stage = V90A_RX_JD;
                s->jd_frames++;
                events |= V90A_RX_EVENT_JD;
                break;
            }
        }
        /*
         * Scan whatever is buffered and not yet scanned.  Normally that is the
         * one sign just pushed, but after a Jd search that came to nothing it
         * is the whole window the search had to buffer, replayed from a
         * re-seeded descrambler — see the failure path in V90A_RX_JD.
         */
        while (s->trn1d_scan < s->sign_len) {
            /* §8.4.5: the scrambler output is the sign, with no differential
             * encoder, so scrambled ones descramble to ones directly. */
            int plain = descramble(&s->descramble_reg, s->signs[s->trn1d_scan]);

            s->trn1d_scan++;
            s->trn1d_symbols = s->trn1d_scan;
            /* The descrambler needs SCRAMBLER_HISTORY bits to come into step,
             * so only count ones once it is in step. */
            if (plain  &&  s->trn1d_scan > SCRAMBLER_HISTORY)
                s->trn1d_ones++;
            /*endif*/
            /*
             * A level that is not TRN1d does not descramble to ones.  Checking
             * that is what lets the S̄d→TRN1d boundary be matched on "some
             * third magnitude" rather than on a predicted one.
             */
            /*
             * WHEN to confirm.  On the digital bearer the codewords are exact
             * from the first symbol, so 256 is early and cheap.  Where levels
             * are sliced the equaliser is still training -- §9.3.2.5 gives it
             * the first 2040T for exactly that -- so judging §8.4.5 at symbol
             * 256 measures an unconverged filter rather than the level.
             * Confirm at the end of that interval, over its last
             * CONFIRM_SYMBOLS, restarting the count so the unconverged part is
             * not what is counted.
             */
            confirm_at = level_tolerant(s) ? TRN1D_MIN_SYMBOLS
                                           : TRN1D_CONFIRM_SYMBOLS;
            confirm_of = level_tolerant(s)
                         ? TRN1D_CONFIRM_SYMBOLS
                         : TRN1D_CONFIRM_SYMBOLS - SCRAMBLER_HISTORY;
            if (level_tolerant(s)
                &&  s->trn1d_scan == confirm_at - TRN1D_CONFIRM_SYMBOLS)
                s->trn1d_ones = 0;
            if (s->trn1d_scan == confirm_at) {
                if (s->trn1d_report)
                    s->trn1d_report(s->trn1d_report_user, s->trn1d_ones,
                                    confirm_of);
                /*
                 * A stream that descrambles to all ZEROS is §8.4.5's all-ones
                 * plaintext read at the opposite polarity: inverting every
                 * sign inverts the recurrence's output, since
                 * ~a ^ ~b ^ ~c = (a ^ b ^ c) ^ 1.  That is not an error, it is
                 * CMA's sign ambiguity -- |y| is what it drives, so y and -y
                 * are equally good solutions and the polarity of a blindly
                 * equalised stream is arbitrary until something resolves it.
                 * §8.4.5 is what resolves it, so resolve it here rather than
                 * rejecting a level that is in fact correct: flip, re-read the
                 * signs buffered since TRN1d began, and let the check run
                 * again on the corrected stream.
                 */
                if (level_tolerant(s)  &&  !s->sign_flip_tried
                    &&  s->trn1d_ones*10 <= confirm_of) {
                    int i;

                    s->sign_flip_tried = 1;
                    s->sign_flip = !s->sign_flip;
                    for (i = 0; i < s->sign_len; i++)
                        s->signs[i] = (uint8_t) !s->signs[i];
                    s->descramble_reg = 0;
                    s->trn1d_scan = 0;
                    s->trn1d_ones = 0;
                    s->trn1d_symbols = 0;
                    break;
                }
                if (s->trn1d_ones*10 < confirm_of*9) {
                    s->stage = V90A_RX_HUNT_SD;
                    memset(s->sd_run, 0, sizeof(s->sd_run));
                    memset(s->sd_w, 0, sizeof(s->sd_w));
                    break;
                }
            }
            /*endif*/
            /*
             * §9.3.2.5/§9.3.2.6: the first 2040T trains the equaliser and only
             * then is Jd looked for.  Jd switches the mapping to differential
             * (§8.4.2), so this decoder stops reading ones within a symbol or
             * two of the boundary -- close to it, but not exactly on it.  Take
             * the break as a hint and let the frame decide.
             */
            /*
             * The single-bit break is for the digital bearer only.  Where the
             * levels are sliced the CRC probe above is the exit, and leaving
             * this enabled beside it just burns the stage on stray errors:
             * that is the TRN1d/Jd oscillation, thousands of transitions with
             * no frame ever validating.  Staying in TRN1d until Table 13
             * validates is also the honest report -- the receiver has not
             * found Jd, and saying so beats claiming a boundary it cannot
             * back up.
             */
            if (!level_tolerant(s)
                &&  !plain  &&  s->trn1d_symbols > TRN1D_MIN_SYMBOLS) {
                s->trn1d_break = s->trn1d_scan - 1;
                s->stage = V90A_RX_JD;
                s->jd_bit_pos = -1;
                break;
            }
            /*endif*/
        }
        break;
    }

    case V90A_RX_JD: {
        /*
         * §8.4.2's Jd and §8.4.3's J'd are both carried on the sign of the one
         * TRN1d codeword, so the bit walk below is what separates them -- but
         * the DIL that follows is not: §8.4.1 sweeps the ladder the descriptor
         * asked for, and leaves that codeword behind on its first symbol.
         *
         * That level change is a second, independent witness of the same
         * moment §9.3.2.8 turns on, and it survives what the bit walk does
         * not.  The walk is positional: it counts 72 bits per frame from where
         * the frame was located, so one inserted or lost octet moves every
         * boundary after it.  Inside Jd that costs a frame and re-locks on the
         * next §8.4.2 sync run, which is why 27 of the card's 38 frames still
         * decoded; but landing on the last frame it eats J'd's twelve zeros
         * and DIL is never entered at all.  Measured live: the card sent a
         * textbook DIL for 23 s and this receiver sat in Jd through all of it.
         */
        ucode = codeword_ucode(s, c, &sign);
        if (ucode != s->trn1d_ucode) {
            if (s->jd_exit_run < (int) sizeof(s->jd_exit_hold))
                s->jd_exit_hold[s->jd_exit_run] = c;
            /*endif*/
            if (++s->jd_exit_run >= JD_EXIT_SYMBOLS) {
                events |= V90A_RX_EVENT_JD_PRIME;
                if (s->dil_cycle_len > 0) {
                    int i;

                    /* The first symbol off the Jd codeword is DIL's first, so
                     * the run that proved it is DIL and belongs in the buffer. */
                    s->dil_start = s->index - s->jd_exit_run + 1;
                    s->dil_len = 0;
                    for (i = 0; i < s->jd_exit_run  &&  s->dil_len < s->dil_cap; i++)
                        s->dil_rx[s->dil_len++] = s->jd_exit_hold[i];
                    s->dil_next_measure = DIL_MEASURE_INTERVAL;
                    s->stage = V90A_RX_DIL;
                } else {
                    s->stage = V90A_RX_DONE;
                    events |= V90A_RX_EVENT_DIL_ENOUGH;
                }
                /*endif*/
                s->index++;
                return events;
            }
            /*endif*/
        } else {
            s->jd_exit_run = 0;
        }
        /*endif*/
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
                /*
                 * No frame in the window.  The break was not Jd but a
                 * corrupted symbol, so go back to reading TRN1d — from just
                 * past the break, with the descrambler re-seeded out of the
                 * buffer.
                 *
                 * Re-seeding is the whole of it.  Leaving the register where
                 * the search left it strands it SCRAMBLER_HISTORY bits behind
                 * the stream, which guarantees another false break a couple of
                 * dozen symbols later, which fails the same way — and the
                 * receiver oscillates between TRN1d and Jd for the rest of the
                 * call, never seeing the boundary when it does arrive.  One
                 * slipped octet is enough to start it.
                 */
                trn1d_resume(s, s->trn1d_break + 1);
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

int v90_analogue_rx_w(const v90_analogue_rx_t *s)
{
    return s ? s->w_ucode : 0;
}

int v90_analogue_rx_trn1d_ucode(const v90_analogue_rx_t *s)
{
    return s ? s->trn1d_ucode : 0;
}

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

/* §8.4.5's confirmation reading, for a caller that wants to see it whether or
 * not the level is accepted.  The check itself is unchanged. */
void v90_analogue_rx_set_trn1d_report(v90_analogue_rx_t *s,
                                      void (*fn)(void *, int, int), void *user)
{
    if (s == NULL)
        return;
    s->trn1d_report = fn;
    s->trn1d_report_user = user;
}

/* Report each new best §8.4.2 frame candidate found while probing TRN1d. */
void v90_analogue_rx_set_jd_probe_report(v90_analogue_rx_t *s,
                                         void (*fn)(void *, int, int),
                                         void *user)
{
    if (s == NULL)
        return;
    s->jd_best_errors = -1;
    s->jd_probe_report = fn;
    s->jd_probe_user = user;
}
