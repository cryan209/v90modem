/*
 * v90_dil_presets.c — DIL descriptors to send, and a check that they are worth
 * sending.  See v90_dil_presets.h.
 */

#include "v90_dil_presets.h"

#include <string.h>

/* SP/TP defaults whose periods are coprime with 6, so training symbols reach
 * every data-frame interval whatever the segment length is. */
#define DIL_DEFAULT_SP_BITS 0x0A6DU
#define DIL_DEFAULT_SP_LEN  12
#define DIL_DEFAULT_TP_BITS 0x6B7U
#define DIL_DEFAULT_TP_LEN  11

static void dil_set_patterns(v90_dil_desc_t *d)
{
    int i;

    d->lsp = DIL_DEFAULT_SP_LEN;
    d->ltp = DIL_DEFAULT_TP_LEN;
    for (i = 0; i < DIL_DEFAULT_SP_LEN; i++)
        d->sp[i] = (uint8_t) ((DIL_DEFAULT_SP_BITS >> i) & 1U);
    for (i = 0; i < DIL_DEFAULT_TP_LEN; i++)
        d->tp[i] = (uint8_t) ((DIL_DEFAULT_TP_BITS >> i) & 1U);
}

static void dil_load_default_ja(v90_dil_desc_t *d)
{
    /* Kept bit-identical to the profile this tree has always used, including
     * its LTP of 12 -- it is what a peer is likely to have seen from us, and
     * changing it here would silently change interop behaviour.  It probes
     * every interval because its segments are 12T, so LTP 12 covers a whole
     * segment and every position in it. */
    static const uint8_t offsets[8] = { 2, 4, 6, 8, 10, 12, 14, 15 };
    int i;

    memset(d, 0, sizeof(*d));
    d->n = 125;
    d->lsp = 12;
    d->ltp = 12;
    for (i = 0; i < 12; i++) {
        d->sp[i] = (uint8_t) ((0x0A6DU >> i) & 1U);
        d->tp[i] = (uint8_t) ((0x0DB7U >> i) & 1U);
    }
    for (i = 0; i < 8; i++) {
        d->h[i] = 1;
        d->ref[i] = (uint8_t) ((i << 4) | 1);
    }
    for (i = 0; i < d->n; i++) {
        int uchord = i % 8;
        int variant = (i / 8) % 8;

        d->train_u[i] = (uint8_t) ((uchord << 4) | offsets[variant]);
    }
}

static void dil_load_measurement(v90_dil_desc_t *d)
{
    int i;

    memset(d, 0, sizeof(*d));
    d->n = 120;
    dil_set_patterns(d);
    for (i = 0; i < 8; i++) {
        d->h[i] = 10;           /* 66T segments */
        d->ref[i] = 0;
    }
    /* Sweep the whole ladder.  Low Ucodes matter as much as high ones: they
     * are the cheap constellation points §8.5.2 leaves alone, and without them
     * a power cap has nothing to fall back on. */
    for (i = 0; i < d->n; i++)
        d->train_u[i] = (uint8_t) (2 + (i * 125) / (d->n - 1));
}

static void dil_load_courier_style(v90_dil_desc_t *d)
{
    int i;

    memset(d, 0, sizeof(*d));
    d->n = 60;
    dil_set_patterns(d);
    for (i = 0; i < 8; i++) {
        d->h[i] = 10;           /* 66T segments, as measured on the card */
        d->ref[i] = 0;
    }
    /*
     * The observed shape: a descending high ladder with a low-Ucode probe
     * interleaved, which measures the top of the range and the noise floor
     * alternately.  The card's downstream ran 84, 83, 82, 81, 80, 79, 78, 1,
     * 77, 1, 76, 1, 75, 1, 2 ... before our segmentation lost it.
     */
    for (i = 0; i < d->n; i++) {
        if ((i & 1) == 0)
            d->train_u[i] = (uint8_t) (84 - i / 2);
        else
            d->train_u[i] = (uint8_t) (1 + ((i / 2) & 1));
    }
}

bool v90_dil_preset_load(v90_dil_preset_t which, v90_dil_desc_t *out)
{
    if (!out)
        return false;
    switch (which) {
    case V90_DIL_PRESET_DEFAULT_JA:
        dil_load_default_ja(out);
        return true;
    case V90_DIL_PRESET_SMARTLINK_ADI:
        return v90_dil_load_smartlink_adi(out);
    case V90_DIL_PRESET_SMARTLINK_ADI_QC:
        return v90_dil_load_smartlink_adi_qc(out);
    case V90_DIL_PRESET_MEASUREMENT:
        dil_load_measurement(out);
        return true;
    case V90_DIL_PRESET_COURIER_STYLE:
        dil_load_courier_style(out);
        return true;
    default:
        return false;
    }
}

const char *v90_dil_preset_name(v90_dil_preset_t which)
{
    switch (which) {
    case V90_DIL_PRESET_DEFAULT_JA:      return "default-ja-125x12";
    case V90_DIL_PRESET_SMARTLINK_ADI:   return "smartlink-adi";
    case V90_DIL_PRESET_SMARTLINK_ADI_QC:return "smartlink-adi-qc";
    case V90_DIL_PRESET_MEASUREMENT:     return "measurement-120x66";
    case V90_DIL_PRESET_COURIER_STYLE:   return "courier-style-60x66";
    default:                             return "?";
    }
}

bool v90_dil_desc_from_ucodes(const uint8_t *ucodes, int n, uint8_t hc,
                              v90_dil_desc_t *out)
{
    int i;

    if (!ucodes || !out || n < 1 || n > V90_DIL_MAX_SEGMENTS)
        return false;
    memset(out, 0, sizeof(*out));
    out->n = (uint8_t) n;
    dil_set_patterns(out);
    for (i = 0; i < 8; i++) {
        out->h[i] = hc;
        out->ref[i] = 0;
    }
    for (i = 0; i < n; i++) {
        if ((ucodes[i] & 0x7F) == 0)
            return false;       /* a segment whose training symbol is REFc
                                 * probes nothing */
        out->train_u[i] = (uint8_t) (ucodes[i] & 0x7F);
    }
    return true;
}

bool v90_dil_desc_validate(const v90_dil_desc_t *desc,
                           v90_dil_desc_check_t *out)
{
    bool seen_ucode[128];
    bool seen_chord[8];
    static bool interval_ucode[6][128];
    int interval_distinct[6];
    int pos = 0;
    int k;
    int i;

    if (!desc || !out || desc->n < 1)
        return false;

    memset(out, 0, sizeof(*out));
    memset(seen_ucode, 0, sizeof(seen_ucode));
    memset(seen_chord, 0, sizeof(seen_chord));
    memset(interval_ucode, 0, sizeof(interval_ucode));
    memset(interval_distinct, 0, sizeof(interval_distinct));
    out->lowest_ucode = 127;
    out->highest_ucode = 0;
    out->segments = desc->n;

    for (k = 0; k < desc->n; k++) {
        int train = desc->train_u[k] & 0x7F;
        int chord = (train >> 4) & 7;
        int ref = desc->ref[chord] & 0x7F;
        int seg_len = ((int) desc->h[chord] + 1) * 6;
        int ltp = desc->ltp ? desc->ltp : 1;

        if (!seen_ucode[train]) {
            seen_ucode[train] = true;
            out->distinct_ucodes++;
        }
        if (!seen_chord[chord]) {
            seen_chord[chord] = true;
            out->chords_covered++;
        }
        if (train < out->lowest_ucode)
            out->lowest_ucode = train;
        if (train > out->highest_ucode)
            out->highest_ucode = train;

        for (i = 0; i < seg_len; i++) {
            /*
             * What an interval learns is how many *distinct levels* reach it,
             * and reference symbols count: a profile whose REFc differs per
             * chord delivers a spread of levels through its TP=0 positions
             * alone.  The clean-line Ja profile is exactly that -- REFc is
             * (chord << 4) | 1 -- and an earlier version of this check, which
             * only counted training symbols, wrongly called its interval 3
             * blind.
             */
            int u = desc->tp[i % ltp] ? train : ref;
            int interval = (pos + i) % 6;

            if (!interval_ucode[interval][u & 0x7F]) {
                interval_ucode[interval][u & 0x7F] = true;
                interval_distinct[interval]++;
            }
        }
        pos += seg_len;
    }

    /* An interval with one level throughout carries no constellation: nothing
     * in it can be told apart from anything else. */
    out->min_interval_levels = 128;
    for (i = 0; i < 6; i++) {
        if (interval_distinct[i] >= 2)
            out->intervals_probed |= (uint8_t) (1u << i);
        if (interval_distinct[i] < out->min_interval_levels)
            out->min_interval_levels = interval_distinct[i];
    }

    out->cycle_symbols = pos;
    out->cycle_ms = pos / 8.0;
    out->ok = (out->intervals_probed == 0x3F) && (out->distinct_ucodes >= 2);
    return true;
}
