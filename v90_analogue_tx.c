/*
 * v90_analogue_tx.c — the analogue modem's Phase 3 transmitter (§9.3.2).
 *
 * See v90_analogue_tx.h for what this is and why the sequencing lives here
 * rather than in SpanDSP's V.34 transmit state machine.
 *
 * Signal definitions are §8.3, which mostly defers to V.34:
 *   S    §8.3.4 -> §10.1.3.7/V.34 — alternation between two points 90° apart,
 *                  phase-reversed for S̄.
 *   PP   §8.3.3 -> §10.1.3.6/V.34 — six periods of a 48-symbol sequence.
 *   TRN  §8.3.6 -> §10.1.3.8/V.34, 4-point: scrambled ones, two bits per
 *                  symbol, direct mapping (no differential encoder).
 *   Ja   §8.3.1 -> §10.1.3.3/V.34 — the J modulation (scrambled, two bits per
 *                  symbol, differentially encoded) carrying repetitions of the
 *                  Table 12 DIL descriptor instead of J's 16-bit pattern.
 *   SCR  §8.3.5 — binary ones through the same mapping, with neither the
 *                  scrambler nor the differential encoder reinitialised.
 *   MD   §8.3.2 -> §10.1.3.5/V.34 — manufacturer-defined, optional.
 *
 * The scrambler is GPA (equation 7-2/V.34), tap x^-5, per §8.3.
 */

#include <stdlib.h>
#include <string.h>

#include "v90_analogue_tx.h"

/* §10.1.3.3/V.34 quarter-superconstellation point 0 rotated by n*90°, in
 * constellation steps.  Same values and same order as the V.34 transmitter's
 * 4-point training constellation, so a symbol index means the same thing on
 * both sides of this tree. */
static const float training_constellation_4[4][2] =
{
    {-0.7071068f, -0.7071068f},   /* 225 degrees */
    {-0.7071068f,  0.7071068f},   /* 135 degrees */
    { 0.7071068f,  0.7071068f},   /*  45 degrees */
    { 0.7071068f, -0.7071068f}    /* 315 degrees */
};

/*
 * PP, §10.1.3.6/V.34 equation 10-1: six periods of this 48-symbol sequence.
 * Data, not hand-editable code — these are the same values SpanDSP generates
 * from the equation in make_v34_probe_signals.c.
 */
#define PP_PERIOD_SYMBOLS   48
#define PP_TOTAL_SYMBOLS    (6*PP_PERIOD_SYMBOLS)

static const float pp_symbols[PP_PERIOD_SYMBOLS][2] =
{
    { 1.0000000f,  0.0000000f}, { 1.0000000f,  0.0000000f},
    { 1.0000000f,  0.0000000f}, { 1.0000000f,  0.0000000f},
    {-0.5000000f,  0.8660254f}, {-0.8660254f,  0.5000000f},
    {-1.0000000f,  0.0000000f}, {-0.8660254f, -0.5000000f},
    { 1.0000000f,  0.0000000f}, { 0.5000000f,  0.8660254f},
    {-0.5000000f,  0.8660254f}, {-1.0000000f,  0.0000000f},
    { 1.0000000f,  0.0000000f}, { 0.0000000f,  1.0000000f},
    {-1.0000000f,  0.0000000f}, {-0.0000000f, -1.0000000f},
    {-0.5000000f,  0.8660254f}, {-0.5000000f, -0.8660254f},
    { 1.0000000f, -0.0000000f}, {-0.5000000f,  0.8660254f},
    { 1.0000000f,  0.0000000f}, {-0.8660254f,  0.5000000f},
    { 0.5000000f, -0.8660254f}, { 0.0000000f,  1.0000000f},
    { 1.0000000f,  0.0000000f}, {-1.0000000f,  0.0000000f},
    { 1.0000000f, -0.0000000f}, {-1.0000000f,  0.0000000f},
    {-0.5000000f,  0.8660254f}, { 0.8660254f, -0.5000000f},
    {-1.0000000f,  0.0000000f}, { 0.8660254f,  0.5000000f},
    { 1.0000000f,  0.0000000f}, {-0.5000000f, -0.8660254f},
    {-0.5000000f,  0.8660254f}, { 1.0000000f, -0.0000000f},
    { 1.0000000f,  0.0000000f}, {-0.0000000f, -1.0000000f},
    {-1.0000000f,  0.0000000f}, { 0.0000000f,  1.0000000f},
    {-0.5000000f,  0.8660254f}, { 0.5000000f,  0.8660254f},
    { 1.0000000f, -0.0000000f}, { 0.5000000f, -0.8660254f},
    { 1.0000000f,  0.0000000f}, { 0.8660254f, -0.5000000f},
    { 0.5000000f, -0.8660254f}, {-0.0000000f, -1.0000000f}
};

static const int baud_rates[6] = {2400, 2743, 2800, 3000, 3200, 3429};

/* §9.3.2.1 and §9.3.2.10 durations, in symbols. */
#define S_SYMBOLS           128
#define S_BAR_SYMBOLS       16
#define INITIAL_SILENCE_MS  70      /* §9.3.2.1: 70 ± 5 ms */

/*
 * §9.3.2.3 requires "at least 512T" of TRN and caps MD-to-end-of-TRN at one
 * round trip plus 4000 ms.  The digital modem is training an equaliser on it,
 * so send what the V.34 transmitter in this tree sends for the same job —
 * 2048T, ~640 ms at 3200 baud — rather than the bare minimum.
 */
#define TRN_SYMBOLS         2048

/* §9.3.2.4: 1500 ms from the start of Ja for the Sd-to-S̄d transition.
 * §9.3.2.7: 4500 ms from the end of Ja for Jd.  Both expire into a retrain. */
#define JA_SD_BAR_DEADLINE_MS   1500
#define JD_DEADLINE_MS          4500

/* v90_build_dil_descriptor_bits() packs bit 0 of Table 12 into the LSB of
 * byte 0, and bit 0 is first in time. */
#define JA_MAX_BYTES            1024

/* §9.4.2.2: SCR after the R̄i transition, for no more than 4000 ms. */
#define SCR_PHASE4_MAX_MS       4000
/* §8.5.3 -> §10.1.3.2/V.34: E is 20 bits, and this mapping is 2 bits/symbol. */
#define E_SYMBOLS               10

struct v90_analogue_tx_s {
    v90_analogue_tx_config_t cfg;
    int      baud_rate;             /* symbols per second */

    v90_analogue_tx_stage_t stage;
    int      stage_symbols;         /* symbols emitted in this stage */
    uint64_t total_symbols;

    /* Index into training_constellation_4 of the next S (or S̄) symbol.
     * §10.1.3.7/V.34: S alternates point 0 with point 0 rotated
     * counterclockwise by 90°, and S̄ is the same pair rotated by 180°. */
    int      s_index;

    /* GPA scrambler, equation 7-2/V.34: out = in ^ x^-5 ^ x^-23. */
    uint32_t scramble_reg;
    /* Differential encoder state, Z_{n-1} of §10.1.3.3/V.34. */
    int      diff;

    uint8_t  ja_bits[JA_MAX_BYTES];
    int      ja_bit_len;
    int      ja_bit_pos;

    bool     dil_zero_length;

    /*
     * §9.4.2.  One bit per byte here, unlike Ja's packed descriptor, because
     * that is what vpcm_cp_encode_bits() produces and repacking it would buy
     * nothing: CP is sent once per few hundred symbols, not per sample.
     */
    uint8_t  cpt_bits[VPCM_CP_MAX_BITS];
    int      cpt_len;
    uint8_t  cp_bits[VPCM_CP_MAX_BITS];
    int      cp_len;
    uint8_t  cp_prime_bits[VPCM_CP_MAX_BITS];
    int      cp_prime_len;
    int      cp_bit_pos;            /* cursor into whichever is being sent */
    bool     scr_after_r;
    bool     phase4_armed;
    /* §9.4.2.2/.3/.4 all say "complete the current sequence" before changing
     * signal, so an event latches here and is acted on at the next wrap. */
    bool     pending_r_transition;
    bool     pending_mp;
    bool     pending_mp_prime;
};

static int ms_to_symbols(const v90_analogue_tx_t *s, int ms)
{
    return (s->baud_rate*ms + 500)/1000;
}

static int scramble_bit(v90_analogue_tx_t *s, int in_bit)
{
    int out_bit;

    /* GPA, equation 7-2/V.34.  The x^-5 tap is the analogue modem's; the
     * digital modem's GPC uses x^-18 instead, and conflating the two is the
     * classic silent breakage in this tree (v90.c:439). */
    out_bit = (in_bit ^ (s->scramble_reg >> 4) ^ (s->scramble_reg >> 22)) & 1;
    s->scramble_reg = (s->scramble_reg << 1) | out_bit;
    return out_bit;
}

/* Two scrambled bits, I1n first in time, differentially encoded (§10.1.3.3). */
static void diff_encoded_symbol(v90_analogue_tx_t *s, int b0, int b1,
                                float *re, float *im)
{
    int in;

    in = scramble_bit(s, b0);
    in |= scramble_bit(s, b1) << 1;
    s->diff = (s->diff + in) & 3;
    *re = training_constellation_4[s->diff][0];
    *im = training_constellation_4[s->diff][1];
}

/* The next Ja bit, wrapping to repeat the descriptor (§8.3.1). */
static int ja_bit(v90_analogue_tx_t *s)
{
    int pos;

    if (s->ja_bit_len <= 0)
        return 0;
    pos = s->ja_bit_pos;
    if (++s->ja_bit_pos >= s->ja_bit_len)
        s->ja_bit_pos = 0;
    return (s->ja_bits[pos >> 3] >> (pos & 7)) & 1;
}

static void enter_stage(v90_analogue_tx_t *s, v90_analogue_tx_stage_t stage)
{
    s->stage = stage;
    s->stage_symbols = 0;
}

/* The bit stream the Phase 4 stage is currently repeating, and its length. */
static const uint8_t *cp_stream(const v90_analogue_tx_t *s, int *len)
{
    switch (s->stage) {
    case V90A_TX_CPT:       *len = s->cpt_len;      return s->cpt_bits;
    case V90A_TX_CP:        *len = s->cp_len;       return s->cp_bits;
    case V90A_TX_CP_PRIME:  *len = s->cp_prime_len; return s->cp_prime_bits;
    default:                *len = 0;               return NULL;
    }
}

/*
 * One Phase 4 symbol from the CP stream, honouring §9.4.2's "complete the
 * current sequence".  Returns the stage to move to at this symbol, or the
 * current stage to stay.
 */
static v90_analogue_tx_stage_t cp_symbol(v90_analogue_tx_t *s,
                                         float *re, float *im)
{
    const uint8_t *bits;
    int len;
    int b0;
    int b1;

    bits = cp_stream(s, &len);
    if (bits == NULL  ||  len <= 0)
        return s->stage;
    b0 = bits[s->cp_bit_pos++] & 1;
    if (s->cp_bit_pos >= len)
        s->cp_bit_pos = 0;
    b1 = bits[s->cp_bit_pos++] & 1;
    if (s->cp_bit_pos >= len)
        s->cp_bit_pos = 0;
    diff_encoded_symbol(s, b0, b1, re, im);
    if (s->cp_bit_pos != 0)
        return s->stage;
    /*endif*/
    /* A sequence boundary: this is where a latched event may be acted on. */
    switch (s->stage) {
    case V90A_TX_CPT:
        /* §9.4.2.2: after the R̄i transition, complete the CPt and then send
         * SCR for at most 4000 ms, or go straight on -- both are permitted. */
        if (s->pending_r_transition) {
            s->pending_r_transition = false;
            return s->scr_after_r ? V90A_TX_SCR4 : V90A_TX_CP;
        }
        break;
    case V90A_TX_CP:
        /* §9.4.2.3: MP received, so acknowledge from the next sequence on. */
        if (s->pending_mp) {
            s->pending_mp = false;
            return V90A_TX_CP_PRIME;
        }
        break;
    case V90A_TX_CP_PRIME:
        /* §9.4.2.4: a CP' has now been sent and MP'/Ed has arrived. */
        if (s->pending_mp_prime) {
            s->pending_mp_prime = false;
            return V90A_TX_E;
        }
        break;
    default:
        break;
    }
    return s->stage;
}

v90_analogue_tx_t *v90_analogue_tx_init(const v90_analogue_tx_config_t *cfg)
{
    v90_analogue_tx_t *s;
    int bit_len;

    if (cfg == NULL  ||  cfg->baud_rate_code < 0  ||  cfg->baud_rate_code > 5)
        return NULL;
    if (cfg->md_units < 0  ||  cfg->md_units > 127)
        return NULL;

    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->cfg = *cfg;
    s->baud_rate = baud_rates[cfg->baud_rate_code];
    s->dil_zero_length = (cfg->dil.n == 0);

    bit_len = 0;
    if (!s->dil_zero_length) {
        if (!v90_build_dil_descriptor_bits(s->ja_bits, (int) sizeof(s->ja_bits),
                                           &bit_len, &cfg->dil)
            ||
            bit_len <= 0) {
            free(s);
            return NULL;
        }
    }
    s->ja_bit_len = bit_len;

    /* §10.1.3.8/V.34: the scrambler is initialised to zero before TRN, which
     * is the first scrambled signal this transmitter sends. */
    s->scramble_reg = 0;
    s->diff = 0;
    s->s_index = 0;
    enter_stage(s, V90A_TX_INITIAL_SILENCE);
    return s;
}

void v90_analogue_tx_free(v90_analogue_tx_t *s)
{
    free(s);
}

/*
 * One S (or S̄) symbol.
 *
 * §10.1.3.7/V.34: S alternates point 0 of the constellation with point 0
 * rotated counterclockwise by 90°, so it steps between indices 0 and 3 (225°
 * and 315°).  XOR 3 is that ±90° step, and it works for the S̄ pair too
 * (indices 2 and 1, 45° and 135°).
 *
 * S ends on point 0 rotated by 90°, and S̄ begins on point 0 rotated by 180°,
 * which is why the pair is reversed *after* the last symbol of the preceding
 * signal rather than before it.
 */
static void s_symbol(v90_analogue_tx_t *s, float *re, float *im)
{
    *re = training_constellation_4[s->s_index][0];
    *im = training_constellation_4[s->s_index][1];
    s->s_index ^= 3;
}

/* Rotate the S pair by 180° at an S-to-S̄ or S̄-to-S boundary. */
static void s_reverse(v90_analogue_tx_t *s)
{
    s->s_index ^= 2;
}

void v90_analogue_tx_get_symbol(void *user_data, float *re, float *im)
{
    v90_analogue_tx_t *s;
    int b0;
    int b1;

    s = (v90_analogue_tx_t *) user_data;
    *re = 0.0f;
    *im = 0.0f;
    if (s == NULL)
        return;

    s->stage_symbols++;
    s->total_symbols++;

    switch (s->stage) {
    case V90A_TX_INITIAL_SILENCE:
        /* §9.3.2.1: silence for 70 ± 5 ms after INFO1a. */
        if (s->stage_symbols >= ms_to_symbols(s, INITIAL_SILENCE_MS))
            enter_stage(s, V90A_TX_S);
        return;

    case V90A_TX_S:
    case V90A_TX_S2:
    case V90A_TX_S_DIL_ENOUGH:
        /* §9.3.2.1 / §9.3.2.10: S for 128T. */
        s_symbol(s, re, im);
        if (s->stage_symbols == S_SYMBOLS) {
            v90_analogue_tx_stage_t next;

            next = (s->stage == V90A_TX_S)  ? V90A_TX_S_BAR
                 : (s->stage == V90A_TX_S2) ? V90A_TX_S_BAR2
                                            : V90A_TX_S_BAR_DIL_ENOUGH;
            s_reverse(s);
            enter_stage(s, next);
        }
        return;

    case V90A_TX_S_BAR:
        /* §9.3.2.1: S̄ for 16T, then MD if INFO1a asked for one. */
        s_symbol(s, re, im);
        if (s->stage_symbols == S_BAR_SYMBOLS) {
            s_reverse(s);
            enter_stage(s, s->cfg.md_units > 0 ? V90A_TX_MD : V90A_TX_PP);
        }
        return;

    case V90A_TX_S_BAR2:
        /* §9.3.2.2: after the post-MD S̄, PP. */
        s_symbol(s, re, im);
        if (s->stage_symbols == S_BAR_SYMBOLS) {
            s_reverse(s);
            enter_stage(s, V90A_TX_PP);
        }
        return;

    case V90A_TX_MD:
        /* §8.3.2: MD is manufacturer-defined and optional.  Nothing here has
         * an echo canceller that TRN cannot train, so occupy the duration
         * INFO1a announced with silence rather than invent a waveform: the
         * digital modem is waiting out a stated interval, not decoding it. */
        if (s->stage_symbols >= ms_to_symbols(s, s->cfg.md_units*35))
            enter_stage(s, V90A_TX_S2);
        return;

    case V90A_TX_PP:
        /* §9.3.2.2 -> §10.1.3.6/V.34: 288 symbols, six periods of 48. */
        *re = pp_symbols[(s->stage_symbols - 1)%PP_PERIOD_SYMBOLS][0];
        *im = pp_symbols[(s->stage_symbols - 1)%PP_PERIOD_SYMBOLS][1];
        if (s->stage_symbols == PP_TOTAL_SYMBOLS)
            enter_stage(s, V90A_TX_TRN);
        return;

    case V90A_TX_TRN:
        /* §8.3.6 -> §10.1.3.8/V.34, 4-point: scrambled ones, direct mapping.
         * The differential encoder Ja will use is initialised from the final
         * TRN symbol (§10.1.3.3/V.34), which is what s->diff holds here. */
        {
            int in;

            in = scramble_bit(s, 1);
            in |= scramble_bit(s, 1) << 1;
            s->diff = in;
            *re = training_constellation_4[in][0];
            *im = training_constellation_4[in][1];
        }
        if (s->stage_symbols >= TRN_SYMBOLS)
            enter_stage(s, V90A_TX_JA);
        return;

    case V90A_TX_JA:
        /* §8.3.1: repetitions of the DIL descriptor, J's modulation.  The
         * sequence ends on the Sd-to-S̄d transition (§9.3.2.4) — mid
         * descriptor if that is where it falls, which §8.3.1 allows. */
        b0 = ja_bit(s);
        b1 = ja_bit(s);
        diff_encoded_symbol(s, b0, b1, re, im);
        return;

    case V90A_TX_JA_SILENCE:
        /* §9.3.2.4: silent from the Sd-to-S̄d transition until Jd arrives. */
        return;

    case V90A_TX_S_AFTER_JD:
        /* §9.3.2.7: S until J'd is detected — no fixed length. */
        s_symbol(s, re, im);
        return;

    case V90A_TX_S_BAR_AFTER_JD:
        /* §9.3.2.8: S̄ for 16T, then either Phase 4 (zero-length DIL) or the
         * DIL we asked for. */
        s_symbol(s, re, im);
        if (s->stage_symbols == S_BAR_SYMBOLS) {
            s_reverse(s);
            enter_stage(s, s->dil_zero_length ? V90A_TX_PHASE4 : V90A_TX_DIL_RX);
        }
        return;

    case V90A_TX_DIL_RX:
        /* §9.3.2.9: silence or SCR, at our discretion, while DIL arrives. */
        if (s->cfg.scr_during_dil) {
            /* §8.3.5: binary ones, and explicitly *without* reinitialising
             * the scrambler or the differential encoder. */
            diff_encoded_symbol(s, 1, 1, re, im);
        }
        return;

    case V90A_TX_S_BAR_DIL_ENOUGH:
        /* §9.3.2.10: the S̄ that closes "enough of the DIL". */
        s_symbol(s, re, im);
        if (s->stage_symbols == S_BAR_SYMBOLS) {
            s_reverse(s);
            enter_stage(s, V90A_TX_PHASE4);
        }
        return;

    case V90A_TX_PHASE4:
        /*
         * §9.3.2.10 is done.  §9.4.2.1 starts as soon as a CPt exists, which
         * is a decision the measurement makes, not this module -- so wait
         * silently for v90_analogue_tx_start_phase4() rather than guess.
         */
        if (s->phase4_armed) {
            /* §8.5.2: the scrambler and differential encoder are initialised
             * to zero before the first CPt. */
            s->scramble_reg = 0;
            s->diff = 0;
            s->cp_bit_pos = 0;
            enter_stage(s, V90A_TX_CPT);
        }
        /*endif*/
        return;

    case V90A_TX_CPT:
    case V90A_TX_CP:
    case V90A_TX_CP_PRIME: {
        v90_analogue_tx_stage_t next = cp_symbol(s, re, im);

        if (next != s->stage)
            enter_stage(s, next);
        /*endif*/
        return;
    }

    case V90A_TX_SCR4:
        /* §9.4.2.2: SCR for no more than 4000 ms.  §8.3.5 again — ones
         * through the same mapping, nothing reinitialised. */
        diff_encoded_symbol(s, 1, 1, re, im);
        if (s->stage_symbols >= ms_to_symbols(s, SCR_PHASE4_MAX_MS))
            enter_stage(s, V90A_TX_CP);
        /*endif*/
        return;

    case V90A_TX_E:
        /* §8.5.3 -> §10.1.3.2/V.34: twenty binary ones, through J's mapping.
         * Two bits per symbol, so ten symbols. */
        diff_encoded_symbol(s, 1, 1, re, im);
        if (s->stage_symbols >= E_SYMBOLS)
            enter_stage(s, V90A_TX_B1_PENDING);
        /*endif*/
        return;

    case V90A_TX_B1_PENDING:
        /* §9.4.2.5: v90_analogue_phase3_tx() sees this boundary on the next
         * output call and hands the modulator to SpanDSP's reset-state V.34
         * B1/data mapper.  Return silence only for the remainder of the block
         * in which E completed; starting B1 mid-block would lose its boundary. */
        return;

    case V90A_TX_RR_S:
        /* V.90 §9.6.2.2.2: response to the digital modem's Rd→R̄d. */
        s_symbol(s, re, im);
        if (s->stage_symbols >= S_SYMBOLS) {
            s_reverse(s);
            enter_stage(s, V90A_TX_RR_S_BAR);
        }
        return;

    case V90A_TX_RR_S_BAR:
        /* §9.6.2.2.3-.4: S̄ for 16T, omit optional SCR, then CP. */
        s_symbol(s, re, im);
        if (s->stage_symbols >= S_BAR_SYMBOLS) {
            s_reverse(s);
            s->cp_bit_pos = 0;
            s->pending_mp = false;
            s->pending_mp_prime = false;
            enter_stage(s, V90A_TX_CP);
        }
        return;
    }
}

void v90_analogue_tx_sd_bar_seen(v90_analogue_tx_t *s)
{
    if (s  &&  s->stage == V90A_TX_JA)
        enter_stage(s, V90A_TX_JA_SILENCE);
}

void v90_analogue_tx_jd_seen(v90_analogue_tx_t *s)
{
    if (s  &&  s->stage == V90A_TX_JA_SILENCE)
        enter_stage(s, V90A_TX_S_AFTER_JD);
}

void v90_analogue_tx_jd_prime_seen(v90_analogue_tx_t *s)
{
    if (s  &&  s->stage == V90A_TX_S_AFTER_JD) {
        s_reverse(s);
        enter_stage(s, V90A_TX_S_BAR_AFTER_JD);
    }
}

void v90_analogue_tx_dil_enough(v90_analogue_tx_t *s)
{
    if (s  &&  s->stage == V90A_TX_DIL_RX)
        enter_stage(s, V90A_TX_S_DIL_ENOUGH);
}

bool v90_analogue_tx_start_phase4(v90_analogue_tx_t *s,
                                  const vpcm_cp_frame_t *cpt,
                                  const vpcm_cp_frame_t *cp,
                                  bool scr_after_r)
{
    vpcm_cp_frame_t prime;

    if (s == NULL  ||  cpt == NULL  ||  cp == NULL)
        return false;
    if (!vpcm_cp_encode_bits(cpt, s->cpt_bits, &s->cpt_len))
        return false;
    if (!vpcm_cp_encode_bits(cp, s->cp_bits, &s->cp_len))
        return false;
    /* §8.5.2: CP' is CP with the acknowledge bit set, and every CP in a group
     * carries identical parameters -- so derive it rather than accept one. */
    prime = *cp;
    prime.acknowledge = true;
    if (!vpcm_cp_encode_bits(&prime, s->cp_prime_bits, &s->cp_prime_len))
        return false;
    /* Two bits go out per symbol, so an odd length would put the sequence
     * boundary mid-symbol and "complete the current sequence" would have no
     * meaning.  Table 14's fill bits make it even; check rather than assume. */
    if ((s->cpt_len & 1)  ||  (s->cp_len & 1)  ||  (s->cp_prime_len & 1))
        return false;
    s->scr_after_r = scr_after_r;
    s->phase4_armed = true;
    return true;
}

void v90_analogue_tx_r_transition_seen(v90_analogue_tx_t *s)
{
    if (s  &&  s->stage == V90A_TX_CPT)
        s->pending_r_transition = true;
}

void v90_analogue_tx_mp_seen(v90_analogue_tx_t *s)
{
    if (s == NULL)
        return;
    /* §9.4.2.3 acknowledges MP from CP; arriving during SCR only means the
     * digital modem is ahead of us, so cut SCR short rather than lose it. */
    if (s->stage == V90A_TX_SCR4)
        enter_stage(s, V90A_TX_CP);
    if (s->stage == V90A_TX_CP)
        s->pending_mp = true;
}

void v90_analogue_tx_mp_prime_seen(v90_analogue_tx_t *s)
{
    if (s  &&  s->stage == V90A_TX_CP_PRIME)
        s->pending_mp_prime = true;
}

bool v90_analogue_tx_rate_renegotiate(v90_analogue_tx_t *s)
{
    if (s == NULL  ||  s->stage != V90A_TX_B1_PENDING  ||  !s->phase4_armed)
        return false;
    /* §9.6.2.2.2 starts a fresh S/S̄ response, but CP's GPA scrambler and
     * differential state are deliberately retained: §8.5.2 initializes them
     * only before the first startup CPt, not on rate renegotiation. */
    s->s_index = 0;
    s->pending_mp = false;
    s->pending_mp_prime = false;
    enter_stage(s, V90A_TX_RR_S);
    return true;
}

v90_analogue_tx_stage_t v90_analogue_tx_stage(const v90_analogue_tx_t *s)
{
    return s ? s->stage : V90A_TX_PHASE4;
}

const char *v90_analogue_tx_stage_name(v90_analogue_tx_stage_t stage)
{
    switch (stage) {
    case V90A_TX_INITIAL_SILENCE:   return "silence";
    case V90A_TX_S:                 return "S";
    case V90A_TX_S_BAR:             return "S-bar";
    case V90A_TX_MD:                return "MD";
    case V90A_TX_S2:                return "S (post-MD)";
    case V90A_TX_S_BAR2:            return "S-bar (post-MD)";
    case V90A_TX_PP:                return "PP";
    case V90A_TX_TRN:               return "TRN";
    case V90A_TX_JA:                return "Ja";
    case V90A_TX_JA_SILENCE:        return "silence (post-Ja)";
    case V90A_TX_S_AFTER_JD:        return "S (awaiting J'd)";
    case V90A_TX_S_BAR_AFTER_JD:    return "S-bar (post-J'd)";
    case V90A_TX_DIL_RX:            return "DIL receive";
    case V90A_TX_S_DIL_ENOUGH:      return "S (DIL enough)";
    case V90A_TX_S_BAR_DIL_ENOUGH:  return "S-bar (DIL enough)";
    case V90A_TX_PHASE4:            return "Phase 4 (unarmed)";
    case V90A_TX_CPT:               return "CPt";
    case V90A_TX_SCR4:              return "SCR (post-R-bar)";
    case V90A_TX_CP:                return "CP";
    case V90A_TX_CP_PRIME:          return "CP'";
    case V90A_TX_E:                 return "E";
    case V90A_TX_B1_PENDING:        return "B1/data handover";
    case V90A_TX_RR_S:              return "rate renegotiation S";
    case V90A_TX_RR_S_BAR:          return "rate renegotiation S-bar";
    }
    return "?";
}

int v90_analogue_tx_stage_symbols(const v90_analogue_tx_t *s)
{
    return s ? s->stage_symbols : 0;
}

uint64_t v90_analogue_tx_total_symbols(const v90_analogue_tx_t *s)
{
    return s ? s->total_symbols : 0;
}

int v90_analogue_tx_ja_bits(const v90_analogue_tx_t *s)
{
    return s ? s->ja_bit_len : 0;
}

bool v90_analogue_tx_deadline_passed(const v90_analogue_tx_t *s)
{
    if (s == NULL)
        return false;
    switch (s->stage) {
    case V90A_TX_JA:
        return s->stage_symbols > ms_to_symbols(s, JA_SD_BAR_DEADLINE_MS);
    case V90A_TX_JA_SILENCE:
        return s->stage_symbols > ms_to_symbols(s, JD_DEADLINE_MS);
    default:
        return false;
    }
}
