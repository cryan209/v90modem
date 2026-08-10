/*
 * v90_dil_rx.c — Offline V.90 DIL waveform decoder (§8.4.1).
 *
 * Pipeline:
 *   1. Decompose G.711 codewords into (Ucode, sign) symbols.
 *   2. Find the longest exactly-periodic run whose period is a multiple
 *      of 6 (every DIL-segment is (Hc+1)*6 symbols, so the full cycle is
 *      always a multiple of 6). The minimal such period is taken as the
 *      cycle; internally repetitive descriptors therefore decode to a
 *      shorter, waveform-equivalent canonical form.
 *   3. Segment one cycle: boundaries are detected where a third distinct
 *      Ucode value appears (each segment carries at most two Ucodes:
 *      its training symbol and its chord's reference symbol), then
 *      snapped to a common mod-6 residue. Reference-symbol runs shared
 *      by adjacent segments give each boundary its snapping slack.
 *   4. Derive the descriptor: training Ucode per segment (the larger of
 *      the two values — REFc in real requests is a low Ucode), Hc from
 *      segment lengths per chord, REFc from the smaller value, and
 *      minimal-period SP/TP patterns solved across all segments.
 *   5. Verify by re-expanding the descriptor with
 *      v90_dil_generate_codewords() and comparing byte-for-byte.
 *
 * Known limits (verification catches all of them, so a decode never
 * silently returns a wrong descriptor):
 *   - Adjacent segments with identical training Ucodes merge unless
 *     another segment of the same chord pins the true length.
 *   - A segment whose TP is all zeros decodes as all-training with the
 *     reference value as its training symbol (waveform-equivalent).
 */

#include "v90_dil_rx.h"

#include <stdlib.h>
#include <string.h>

#define DIL_MIN_RUN_SYMBOLS 240   /* ~30 ms: shortest run worth decoding */
#define DIL_SCAN_STEP       512
#define DIL_SCAN_MAX_RUNS   16
#define DIL_MAX_SEG_LEN     1536  /* (Hc max 255 + 1) * 6 */

typedef struct {
    uint8_t u;
    uint8_t s;
} dil_sym_t;

typedef struct {
    int start;      /* template position */
    int len;
    uint8_t val;    /* ucode */
} dil_run_t;

typedef struct {
    int start;      /* template position of the (snapped) segment start */
    int len;
    int train;      /* training ucode (assumed max value in segment) */
    int ref;        /* reference ucode, -1 when never transmitted */
} dil_seg_t;

static bool dil_sym_eq(dil_sym_t a, dil_sym_t b)
{
    return a.u == b.u && a.s == b.s;
}

static void dil_decompose(const uint8_t *codewords, int len, v90_law_t law,
                          dil_sym_t *out)
{
    int i;
    int u;
    int s;

    for (i = 0; i < len; i++) {
        v90_codeword_decompose(law, codewords[i], &u, &s);
        out[i] = (dil_sym_t){ (uint8_t)u, (uint8_t)s };
    }
}

/* A usable cycle template has some signal in it: at least two distinct
 * symbols and a training-grade ucode. Rejects idle/silence/dither
 * pseudo-periods (real DILs train with Ucodes well above the noise
 * floor; Sd already uses Ucode >= 16). */
#define DIL_MIN_TRAIN_UCODE 8

/*
 * §8.4.4 Sd is {+W, +0, +W, -W, -0, -W} repeated 64 times, which is exactly
 * periodic at 6 symbols and carries two magnitudes, so it satisfies every
 * structural test for a DIL cycle and gets decoded as a 6-symbol,
 * single-segment DIL.  On artifacts/eicon-digital-downstream/call1 that made
 * the 384-symbol Sd at 8366.9 ms the reported "codeword-exact DIL run".
 *
 * Sd's shape is fixed by the Recommendation, so it can be excluded exactly:
 * the zero slots sit at positions 1 and 4, the other four slots all carry the
 * same magnitude W, and the signs run +++--- (or the S-bar-d inverse).  No DIL
 * cycle has that form -- a DIL-segment's reference symbol is REFc, which is
 * not constrained to alternate with a single training Ucode this way.
 */
static bool dil_is_sd_template(const dil_sym_t *t, int c)
{
    int i;

    /* Sd is periodic at 6, so it also fits any multiple of 6 as a "cycle"
     * (12 symbols is two Sd frames).  Test the 6-frame and then require the
     * whole template to be that frame repeated, so the exclusion does not
     * depend on which multiple the periodicity search happened to pick. */
    if (c < 6 || c % 6 != 0)
        return false;
    if (t[0].u == 0 || t[1].u != 0 || t[4].u != 0)
        return false;
    if (t[2].u != t[0].u || t[3].u != t[0].u || t[5].u != t[0].u)
        return false;
    /* Signs are checked on the W slots only: the polarity of a zero-level
     * symbol carries nothing, and a real digital modem does not follow
     * §8.4.4's "-0" there (see v90_sd_slot_match in vpcm_decode.c). */
    if (t[2].s != t[0].s)
        return false;
    if (t[3].s == t[0].s || t[5].s != t[3].s)
        return false;
    for (i = 6; i < c; i++) {
        if (t[i].u != t[i % 6].u)
            return false;
        if (t[i].u != 0 && t[i].s != t[i % 6].s)
            return false;
    }
    return true;
}

static bool dil_template_plausible(const dil_sym_t *t, int c)
{
    int i;
    bool distinct = false;
    bool trainable = false;

    if (dil_is_sd_template(t, c))
        return false;

    for (i = 0; i < c; i++) {
        if (t[i].u >= DIL_MIN_TRAIN_UCODE)
            trainable = true;
        if (i > 0 && !dil_sym_eq(t[i], t[0]))
            distinct = true;
        if (distinct && trainable)
            return true;
    }
    return false;
}

/*
 * Find the exact-periodic run through/after `at` with the longest extent,
 * preferring the smallest period among equals. Returns period 0 if none.
 */
static void dil_best_run_at(const dil_sym_t *x, int len, int at, int max_period,
                            int *rs_out, int *re_out, int *c_out)
{
    int c;
    int best_c = 0;
    int best_rs = 0;
    int best_re = 0;

    /* Constant/near-idle windows are "periodic" at every candidate lag,
     * which would make the search quadratic. Require a plausible local
     * window before trying periods at all. */
    {
        int w = len - at;

        if (w > DIL_MIN_RUN_SYMBOLS)
            w = DIL_MIN_RUN_SYMBOLS;
        if (!dil_template_plausible(x + at, w)) {
            *rs_out = 0;
            *re_out = 0;
            *c_out = 0;
            return;
        }
    }

    for (c = 6; c <= max_period; c += 6) {
        int i;
        int rs;
        int re;

        if (at + 2 * c > len)
            break;
        i = at;
        while (i + c < len && dil_sym_eq(x[i], x[i + c]))
            i++;
        re = i + c;
        rs = at;
        while (rs > 0 && dil_sym_eq(x[rs - 1], x[rs - 1 + c]))
            rs--;
        if (re - rs < 2 * c || re - rs < DIL_MIN_RUN_SYMBOLS)
            continue;
        if (!dil_template_plausible(x + rs, c))
            continue;
        if (re - rs > best_re - best_rs) {
            best_c = c;
            best_rs = rs;
            best_re = re;
        }
    }
    *rs_out = best_rs;
    *re_out = best_re;
    *c_out = best_c;
}

/* Build the circular list of equal-ucode runs over template t[0..c). */
static int dil_build_runs(const dil_sym_t *t, int c, dil_run_t *runs)
{
    int i;
    int n = 0;

    for (i = 0; i < c; i++) {
        if (n > 0 && runs[n - 1].val == t[i].u) {
            runs[n - 1].len++;
        } else {
            runs[n].start = i;
            runs[n].len = 1;
            runs[n].val = t[i].u;
            n++;
        }
    }
    /* merge wrap-around run */
    if (n > 1 && runs[n - 1].val == runs[0].val) {
        runs[0].start = runs[n - 1].start - c;  /* negative start, len spans wrap */
        runs[0].len += runs[n - 1].len;
        n--;
    }
    return n;
}

/*
 * Solve a minimal-period on/off pattern across all segments.
 * bit(k, i) returns 0/1 for symbol i of segment k (or -1 to skip).
 * Returns the period (1..128) and fills pat[], or 0 when none fits.
 */
typedef int (*dil_bit_fn)(void *ctx, int seg, int pos);

static int dil_solve_pattern(const dil_seg_t *segs, int nseg,
                             dil_bit_fn bit, void *ctx, uint8_t *pat)
{
    int period;

    for (period = 1; period <= V90_DIL_MAX_PAT_BITS; period++) {
        uint8_t tmp[V90_DIL_MAX_PAT_BITS];
        bool ok = true;
        int k;

        memset(tmp, 0xFF, sizeof(tmp));
        for (k = 0; k < nseg && ok; k++) {
            int i;

            for (i = 0; i < segs[k].len; i++) {
                int b = bit(ctx, k, i);
                int idx = i % period;

                if (b < 0)
                    continue;
                if (tmp[idx] == 0xFF) {
                    tmp[idx] = (uint8_t)b;
                } else if (tmp[idx] != (uint8_t)b) {
                    ok = false;
                    break;
                }
            }
        }
        if (ok) {
            int i;

            for (i = 0; i < period; i++)
                pat[i] = (tmp[i] == 0xFF) ? 0 : tmp[i];
            return period;
        }
    }
    return 0;
}

typedef struct {
    const dil_sym_t *t;
    int c;
    const dil_seg_t *segs;
} dil_pat_ctx_t;

static int dil_sign_bit(void *vctx, int seg, int pos)
{
    const dil_pat_ctx_t *ctx = vctx;
    int p = ctx->segs[seg].start + pos;

    return ctx->t[((p % ctx->c) + ctx->c) % ctx->c].s;
}

static int dil_train_bit(void *vctx, int seg, int pos)
{
    const dil_pat_ctx_t *ctx = vctx;
    int p = ctx->segs[seg].start + pos;
    int u = ctx->t[((p % ctx->c) + ctx->c) % ctx->c].u;

    if (u == ctx->segs[seg].train)
        return 1;
    if (ctx->segs[seg].ref >= 0 && u == ctx->segs[seg].ref)
        return 0;
    return -1;
}

/* Fill train/ref for a segment from the template; false if > 2 values. */
static bool dil_seg_classify(const dil_sym_t *t, int c, dil_seg_t *seg)
{
    int i;
    int a = -1;
    int b = -1;

    for (i = 0; i < seg->len; i++) {
        int u = t[(((seg->start + i) % c) + c) % c].u;

        if (a < 0 || u == a)
            a = u;
        else if (b < 0 || u == b)
            b = u;
        else
            return false;
    }
    if (b >= 0 && b > a) {
        int swap = a;

        a = b;
        b = swap;
    }
    seg->train = a;
    seg->ref = b;
    return true;
}

/*
 * Derive a descriptor from a snapped segmentation and verify it against
 * the template. segs[] must be ordered by ascending start position.
 */
static bool dil_derive_and_verify(const dil_sym_t *t, int c, v90_law_t law,
                                  dil_seg_t *segs, int nseg,
                                  v90_dil_desc_t *desc_out)
{
    v90_dil_desc_t desc;
    dil_pat_ctx_t ctx;
    int chord_len[8];
    int chord_ref[8];
    int k;
    int lsp;
    int ltp;
    uint8_t *gen = NULL;
    bool ok;

    if (nseg < 1 || nseg > V90_DIL_MAX_SEGMENTS)
        return false;

    memset(&desc, 0, sizeof(desc));
    for (k = 0; k < 8; k++) {
        chord_len[k] = -1;
        chord_ref[k] = -1;
        desc.h[k] = 1;      /* defaults for unused chords */
        desc.ref[k] = 0;
    }

    for (k = 0; k < nseg; k++) {
        int chord;

        if (segs[k].len < 6 || segs[k].len % 6 != 0
            || segs[k].len > DIL_MAX_SEG_LEN)
            return false;
        if (!dil_seg_classify(t, c, &segs[k]))
            return false;
        chord = (segs[k].train >> 4) & 7;
        if (chord_len[chord] < 0)
            chord_len[chord] = segs[k].len;
        else if (chord_len[chord] != segs[k].len)
            return false;
        if (segs[k].ref >= 0) {
            if (chord_ref[chord] < 0)
                chord_ref[chord] = segs[k].ref;
            else if (chord_ref[chord] != segs[k].ref)
                return false;
        }
        desc.train_u[k] = (uint8_t)segs[k].train;
    }
    desc.n = (uint8_t)nseg;
    for (k = 0; k < 8; k++) {
        if (chord_len[k] >= 0)
            desc.h[k] = (uint8_t)(chord_len[k] / 6 - 1);
        if (chord_ref[k] >= 0)
            desc.ref[k] = (uint8_t)chord_ref[k];
    }

    ctx.t = t;
    ctx.c = c;
    ctx.segs = segs;
    lsp = dil_solve_pattern(segs, nseg, dil_sign_bit, &ctx, desc.sp);
    if (lsp == 0)
        return false;
    ltp = dil_solve_pattern(segs, nseg, dil_train_bit, &ctx, desc.tp);
    if (ltp == 0)
        return false;

    /* A pattern is only observable out to the longest segment. Expanding
     * the minimal period to that length (when it fits in 128 bits) gives
     * the most complete waveform-exact form and lets requested patterns
     * like the ubiquitous 12-bit defaults come back verbatim. Longer
     * segments wrap the pattern in-segment, so the minimal period is
     * kept there (any other length would change the expansion). */
    {
        int longest = 0;
        int i;

        for (k = 0; k < nseg; k++) {
            if (segs[k].len > longest)
                longest = segs[k].len;
        }
        if (longest <= V90_DIL_MAX_PAT_BITS && lsp <= longest && ltp <= longest) {
            for (i = lsp; i < longest; i++)
                desc.sp[i] = desc.sp[i % lsp];
            for (i = ltp; i < longest; i++)
                desc.tp[i] = desc.tp[i % ltp];
            lsp = longest;
            ltp = longest;
        }
    }
    desc.lsp = (uint8_t)lsp;
    desc.ltp = (uint8_t)ltp;

    /* Verify: the re-expanded cycle must reproduce the template starting
     * at the first segment boundary. */
    gen = malloc((size_t)c);
    if (!gen)
        return false;
    if (v90_dil_generate_codewords(law, &desc, gen, c) != c) {
        free(gen);
        return false;
    }
    ok = true;
    for (k = 0; k < c; k++) {
        dil_sym_t obs = t[(((segs[0].start + k) % c) + c) % c];

        if (gen[k] != v90_codeword_compose(law, obs.u, obs.s)) {
            ok = false;
            break;
        }
    }
    free(gen);
    if (!ok)
        return false;

    *desc_out = desc;
    return true;
}

/*
 * Split any segment whose chord has mixed lengths into copies of the
 * chord's shortest observed length (handles adjacent same-Ucode segments
 * that merged during boundary detection). Returns the new count, or -1.
 */
static int dil_split_merged(dil_seg_t *segs, int nseg, int max_seg)
{
    int min_len[8];
    int k;
    int out_n = 0;
    dil_seg_t *tmp;

    for (k = 0; k < 8; k++)
        min_len[k] = 0;
    for (k = 0; k < nseg; k++) {
        int chord = (segs[k].train >> 4) & 7;

        if (min_len[chord] == 0 || segs[k].len < min_len[chord])
            min_len[chord] = segs[k].len;
    }
    tmp = malloc(sizeof(*tmp) * (size_t)max_seg);
    if (!tmp)
        return -1;
    for (k = 0; k < nseg; k++) {
        int chord = (segs[k].train >> 4) & 7;
        int L = min_len[chord];
        int parts;
        int p;

        if (L <= 0 || segs[k].len % L != 0) {
            free(tmp);
            return -1;
        }
        parts = segs[k].len / L;
        if (out_n + parts > max_seg) {
            free(tmp);
            return -1;
        }
        for (p = 0; p < parts; p++) {
            tmp[out_n] = segs[k];
            tmp[out_n].start = segs[k].start + p * L;
            tmp[out_n].len = L;
            out_n++;
        }
    }
    memcpy(segs, tmp, sizeof(*tmp) * (size_t)out_n);
    free(tmp);
    return out_n;
}

/*
 * Boundary-based segmentation: walk the circular run list, opening a new
 * segment whenever a third distinct ucode appears, then snap boundaries
 * to a common mod-6 residue using shared-run slack. Tries every feasible
 * residue until one derivation verifies.
 */
static bool dil_decode_boundaries(const dil_sym_t *t, int c, v90_law_t law,
                                  const dil_run_t *runs, int nruns,
                                  v90_dil_desc_t *desc_out,
                                  int *first_seg_pos_out)
{
    int *bruns = NULL;      /* run index starting each raw segment */
    int *slack = NULL;
    dil_seg_t *segs = NULL;
    int nseg = 0;
    int max_seg = c / 6 + 2;
    int first_b = -1;
    int j;
    int r;
    bool found = false;

    /* Find the first boundary: scan runs circularly with a 2-value set. */
    {
        int va = runs[0].val;
        int vb = -1;

        for (j = 1; j < nruns * 2; j++) {
            int v = runs[j % nruns].val;

            if (v == va || v == vb)
                continue;
            if (vb < 0 && j < nruns) {
                vb = v;
                continue;
            }
            first_b = j % nruns;
            break;
        }
    }
    if (first_b < 0)
        return false;   /* <= 2 values in the whole cycle: uniform path */

    bruns = malloc(sizeof(int) * (size_t)(nruns + 1));
    slack = malloc(sizeof(int) * (size_t)(nruns + 1));
    segs = malloc(sizeof(*segs) * (size_t)max_seg);
    if (!bruns || !slack || !segs)
        goto out;

    /* Collect all boundaries around the circle starting from first_b. */
    {
        int va = runs[first_b].val;
        int vb = -1;

        bruns[nseg++] = first_b;
        for (j = 1; j < nruns; j++) {
            int idx = (first_b + j) % nruns;
            int v = runs[idx].val;

            if (v == va || v == vb)
                continue;
            if (vb < 0) {
                vb = v;
                continue;
            }
            bruns[nseg++] = idx;
            va = v;
            vb = -1;
        }
    }

    /* Slack: a boundary may move left across the immediately preceding
     * run when that run's value is a plausible member of both segments. */
    for (j = 0; j < nseg; j++) {
        int prev = (bruns[j] + nruns - 1) % nruns;

        slack[j] = runs[prev].len;
    }

    for (r = 0; r < 6 && !found; r++) {
        int k;
        bool feasible = true;
        int n2 = nseg;

        for (k = 0; k < nseg; k++) {
            int p = runs[bruns[k]].start;
            int d = ((p - r) % 6 + 6) % 6;

            if (d > slack[k]) {
                feasible = false;
                break;
            }
            segs[k].start = p - d;
            segs[k].train = -1;
            segs[k].ref = -1;
        }
        if (!feasible)
            continue;
        /* Boundary positions ascend around the circle with exactly one
         * wrap point, so a non-positive difference marks the wrap. */
        {
            int total = 0;

            for (k = 0; k < nseg; k++) {
                int next = segs[(k + 1) % nseg].start;

                segs[k].len = next - segs[k].start;
                if (segs[k].len <= 0)
                    segs[k].len += c;
                if (segs[k].len < 6) {
                    feasible = false;
                    break;
                }
                total += segs[k].len;
            }
            if (total != c)
                feasible = false;
        }
        if (!feasible)
            continue;
        for (k = 0; k < nseg; k++) {
            if (!dil_seg_classify(t, c, &segs[k])) {
                feasible = false;
                break;
            }
        }
        if (!feasible)
            continue;
        n2 = dil_split_merged(segs, nseg, max_seg);
        if (n2 < 0)
            continue;
        /* Order by template position, first segment = smallest start. */
        {
            int best = 0;

            for (k = 1; k < n2; k++) {
                if (((segs[k].start % c) + c) % c
                    < ((segs[best].start % c) + c) % c)
                    best = k;
            }
            if (best != 0) {
                dil_seg_t *rot = malloc(sizeof(*rot) * (size_t)n2);

                if (!rot)
                    goto out;
                for (k = 0; k < n2; k++)
                    rot[k] = segs[(best + k) % n2];
                memcpy(segs, rot, sizeof(*rot) * (size_t)n2);
                free(rot);
            }
        }
        if (dil_derive_and_verify(t, c, law, segs, n2, desc_out)) {
            *first_seg_pos_out = ((segs[0].start % c) + c) % c;
            found = true;
        }
    }

out:
    free(bruns);
    free(slack);
    free(segs);
    return found;
}

/*
 * Uniform path for cycles with at most two distinct ucodes: every
 * segment shares one training value, so try divisor lengths and phase
 * offsets until a derivation verifies.
 */
static bool dil_decode_uniform(const dil_sym_t *t, int c, v90_law_t law,
                               v90_dil_desc_t *desc_out,
                               int *first_seg_pos_out)
{
    int L;
    dil_seg_t *segs = malloc(sizeof(*segs) * (size_t)(c / 6 + 1));
    bool found = false;

    if (!segs)
        return false;
    for (L = 6; L <= c && L <= DIL_MAX_SEG_LEN && !found; L += 6) {
        int phi;

        if (c % L != 0)
            continue;
        if (c / L > V90_DIL_MAX_SEGMENTS)
            continue;
        for (phi = 0; phi < L && !found; phi += 6) {
            int n = c / L;
            int k;

            for (k = 0; k < n; k++) {
                segs[k].start = phi + k * L;
                segs[k].len = L;
                segs[k].train = -1;
                segs[k].ref = -1;
            }
            if (dil_derive_and_verify(t, c, law, segs, n, desc_out)) {
                *first_seg_pos_out = phi % c;
                found = true;
            }
        }
    }
    free(segs);
    return found;
}

static bool dil_decode_run(const dil_sym_t *x, int rs, int re, int c,
                           v90_law_t law, v90_dil_rx_result_t *out)
{
    const dil_sym_t *t = x + rs;
    dil_run_t *runs;
    int nruns;
    v90_dil_desc_t desc;
    int first_seg_pos = 0;
    bool ok;

    runs = malloc(sizeof(*runs) * (size_t)(c + 1));
    if (!runs)
        return false;
    nruns = dil_build_runs(t, c, runs);

    ok = dil_decode_boundaries(t, c, law, runs, nruns, &desc, &first_seg_pos);
    if (!ok)
        ok = dil_decode_uniform(t, c, law, &desc, &first_seg_pos);
    free(runs);
    if (!ok)
        return false;

    /*
     * Reject a constant-magnitude run.
     *
     * DIL probes the channel by transmitting a training Ucode against a
     * reference (§8.4.7), so every DIL cycle carries at least two distinct
     * magnitudes -- including a single-segment one, where the training and
     * reference Ucodes still differ.  A run with one magnitude throughout is
     * therefore not DIL, whatever descriptor can be fitted to its signs.
     *
     * §8.6.4 Ri is exactly such a run: U_INFO at a period-6 sign pattern.  On
     * artifacts/eicon-digital-downstream/call1 the 4548-symbol Ri run at
     * 14261 ms was decoded as "N=1 LSP=6 LTP=6, training Ucodes: 1 unique"
     * and reported as the DIL run, 2 s after the real DIL had started.
     *
     * The test is on the signal rather than the descriptor: N==1 is legal DIL
     * (see the single-segment roundtrip in vpcm_loopback_test), so rejecting
     * on descriptor shape would throw out real decodes with the false one.
     *
     * Failing here lets v90_dil_rx_scan() move on to the next candidate, and
     * leaves a genuine no-decode visible instead of masked by a false one.
     */
    {
        int i;
        bool varies = false;

        for (i = rs + 1; i < re; i++) {
            if (x[i].u != x[rs].u) {
                varies = true;
                break;
            }
        }
        if (!varies)
            return false;
    }

    memset(out, 0, sizeof(*out));
    out->desc = desc;
    out->run_start = rs;
    out->run_len = re - rs;
    out->first_segment_at = rs + first_seg_pos;
    out->cycle_len = c;
    out->cycles_seen = (re - rs) / c;
    out->mismatches = 0;    /* run extension is exact-match based */
    out->exact = true;
    return true;
}

bool v90_dil_rx_decode(const uint8_t *codewords, int len, v90_law_t law,
                       v90_dil_rx_result_t *out)
{
    dil_sym_t *x;
    int rs;
    int re;
    int c;
    int max_period;
    bool ok = false;

    if (!codewords || len < DIL_MIN_RUN_SYMBOLS || !out)
        return false;
    x = malloc(sizeof(*x) * (size_t)len);
    if (!x)
        return false;
    dil_decompose(codewords, len, law, x);

    max_period = len / 2;
    if (max_period > V90_DIL_RX_MAX_CYCLE)
        max_period = V90_DIL_RX_MAX_CYCLE;
    dil_best_run_at(x, len, 0, max_period, &rs, &re, &c);
    if (c > 0)
        ok = dil_decode_run(x, rs, re, c, law, out);
    free(x);
    if (!ok)
        return v90_dil_rx_scan(codewords, len, law, out);
    return ok;
}

bool v90_dil_rx_scan(const uint8_t *codewords, int len, v90_law_t law,
                     v90_dil_rx_result_t *out)
{
    typedef struct {
        int rs;
        int re;
        int c;
    } dil_cand_t;
    dil_sym_t *x;
    dil_cand_t cand[DIL_SCAN_MAX_RUNS];
    int ncand = 0;
    int max_period;
    int at;
    int i;
    bool ok = false;

    if (!codewords || len < DIL_MIN_RUN_SYMBOLS || !out)
        return false;
    x = malloc(sizeof(*x) * (size_t)len);
    if (!x)
        return false;
    dil_decompose(codewords, len, law, x);

    max_period = len / 2;
    if (max_period > V90_DIL_RX_MAX_CYCLE)
        max_period = V90_DIL_RX_MAX_CYCLE;

    for (at = 0; at + DIL_MIN_RUN_SYMBOLS <= len; ) {
        int rs;
        int re;
        int c;

        dil_best_run_at(x, len, at, max_period, &rs, &re, &c);
        if (c == 0) {
            at += DIL_SCAN_STEP;
            continue;
        }
        if (ncand < DIL_SCAN_MAX_RUNS) {
            cand[ncand].rs = rs;
            cand[ncand].re = re;
            cand[ncand].c = c;
            ncand++;
        }
        at = re + DIL_SCAN_STEP;
    }

    /* Longest run first. */
    for (i = 1; i < ncand; i++) {
        int j = i;

        while (j > 0 && cand[j].re - cand[j].rs > cand[j - 1].re - cand[j - 1].rs) {
            dil_cand_t swap = cand[j];

            cand[j] = cand[j - 1];
            cand[j - 1] = swap;
            j--;
        }
    }
    for (i = 0; i < ncand && !ok; i++)
        ok = dil_decode_run(x, cand[i].rs, cand[i].re, cand[i].c, law, out);
    free(x);
    return ok;
}
