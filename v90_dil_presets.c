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

static void dil_load_rasfinder(v90_dil_desc_t *d)
{
    /* V.90 Table 12 descriptor decoded CRC-valid from the RasFinder's Ja in
     * artifacts/rf-maxpow-c1/live-rx.g711.  V90_DIL_DESC_LOG on an offline
     * replay provides the complete fields below.  This is peer-authored
     * recovery data for an explicit interop profile, not a locally invented
     * training sequence. */
    static const uint8_t h[8] = { 59, 59, 39, 39, 39, 9, 9, 9 };
    static const uint8_t train_u[192] = {
        96,96,88,88,88,88,104,104,102,102,94,94,94,94,110,110,
        100,100,92,92,92,92,108,108,98,98,90,90,90,90,106,106,
        63,71,55,87,87,87,87,79,62,70,54,86,86,86,86,78,
        61,69,53,85,85,85,85,77,60,68,52,84,84,84,84,76,
        59,67,51,83,83,83,83,75,58,66,50,82,82,82,82,74,
        57,65,49,81,81,81,81,73,56,64,48,80,80,80,80,72,
        31,15,112,112,47,103,103,39,95,95,95,95,18,2,23,111,
        111,7,30,14,113,113,29,13,114,114,20,4,45,101,101,37,
        93,93,93,93,42,34,21,109,109,5,28,12,115,115,27,11,
        116,116,43,99,99,35,16,46,91,91,91,91,22,6,19,107,
        107,3,26,10,117,117,44,36,25,9,118,118,41,97,97,33,
        89,89,89,89,40,32,17,105,105,1,24,8,119,119,38,0
    };
    static const char sp[] =
        "000000000000100000000000000000010000000000000000001000000000"
        "111000111000111000111000111000000111000111000111000111000111";
    static const char tp[] =
        "000000000000100000000100000000010000000010000000001000000001"
        "111111111111111111111111111111111111111111111111111111111111";
    int i;

    memset(d, 0, sizeof(*d));
    d->n = 192;
    d->lsp = 120;
    d->ltp = 120;
    memcpy(d->h, h, sizeof(h));
    memcpy(d->train_u, train_u, sizeof(train_u));
    for (i = 0; i < 120; i++) {
        d->sp[i] = (uint8_t)(sp[i] - '0');
        d->tp[i] = (uint8_t)(tp[i] - '0');
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
    case V90_DIL_PRESET_RASFINDER:
        dil_load_rasfinder(out);
        return true;
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
    case V90_DIL_PRESET_RASFINDER:       return "rasfinder";
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
