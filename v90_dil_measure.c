/*
 * v90_dil_measure.c — measure the channel from a DIL we requested (§9.3.2.9).
 *
 * See v90_dil_measure.h for why this is a different problem from v90_dil_rx.c.
 */

#include "v90_dil_measure.h"

#include "v91.h"
#include "vpcm_cp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Enough of the DIL to lock onto without paying for the whole cycle. */
#define DIL_ALIGN_PROBE_SYMBOLS 396

static int dil_abs_level(v90_law_t law, uint8_t codeword)
{
    v91_law_t l = (law == V90_LAW_ALAW) ? V91_LAW_ALAW : V91_LAW_ULAW;
    int v = v91_codeword_to_linear(l, codeword);

    return v < 0 ? -v : v;
}

/*
 * Normalised correlation of |level| between the expected DIL and the received
 * stream at `at`.  Magnitudes rather than signed levels: a network that
 * inverts polarity is still a channel we can measure, and the sign pattern is
 * not what identifies the alignment.
 */
static double dil_align_score(const int *want, int n,
                              const uint8_t *rx, int rx_len, int at,
                              v90_law_t law)
{
    double sxy = 0.0;
    double sxx = 0.0;
    double syy = 0.0;
    double mx = 0.0;
    double my = 0.0;
    int i;

    if (at < 0 || at + n > rx_len)
        return -1.0;
    for (i = 0; i < n; i++) {
        mx += want[i];
        my += dil_abs_level(law, rx[at + i]);
    }
    mx /= n;
    my /= n;
    for (i = 0; i < n; i++) {
        double dx = want[i] - mx;
        double dy = dil_abs_level(law, rx[at + i]) - my;

        sxy += dx * dy;
        sxx += dx * dx;
        syy += dy * dy;
    }
    if (sxx <= 0.0 || syy <= 0.0)
        return -1.0;
    return sxy / sqrt(sxx * syy);
}

bool v90_dil_measure_align(const uint8_t *rx, int rx_len, v90_law_t law,
                           const v90_dil_desc_t *desc,
                           int from, int span,
                           int *offset_out, double *score_out)
{
    int cycle;
    int probe;
    uint8_t *gen;
    int *want;
    int hi;
    int at;
    int best_at = -1;
    double best = -1.0;
    int i;

    if (!rx || !desc || rx_len <= 0)
        return false;
    cycle = v90_dil_cycle_len(desc);
    if (cycle <= 0)
        return false;

    probe = cycle < DIL_ALIGN_PROBE_SYMBOLS ? cycle : DIL_ALIGN_PROBE_SYMBOLS;
    if (probe > rx_len)
        probe = rx_len;
    if (probe < 12)
        return false;

    gen = malloc((size_t) probe);
    want = malloc(sizeof(*want) * (size_t) probe);
    if (!gen || !want) {
        free(gen);
        free(want);
        return false;
    }
    if (v90_dil_generate_codewords(law, desc, gen, probe) != probe) {
        free(gen);
        free(want);
        return false;
    }
    for (i = 0; i < probe; i++)
        want[i] = dil_abs_level(law, gen[i]);

    if (from < 0)
        from = 0;
    /* `span` bounds where the DIL may *start*, not how much is examined: the
     * probe reads past it by design. */
    hi = (span > 0) ? from + span : rx_len;
    if (hi > rx_len - probe)
        hi = rx_len - probe;
    /*
     * Ties go to the earliest offset, and they are not rare.
     *
     * G.711 is self-similar across chords — the levels of Ucodes u and u+16
     * differ by exactly a factor of two — and a DIL that steps the ladder one
     * Ucode per segment therefore looks *identical*, up to a scale factor, 16
     * segments later.  Normalised correlation cannot see a scale factor, which
     * is the whole reason it is used here (a pad is a scale factor).  So a
     * blind search over a long capture is genuinely ambiguous by one chord,
     * and no scoring function on levels alone can break it.
     *
     * That is not a problem where this runs: §9.3.2.8-9 has the analogue modem
     * send S and then receive the DIL it just asked for, so it knows within a
     * few symbols when DIL starts and passes a narrow `from`/`span`.  The
     * earliest-tie rule keeps a wide search honest rather than correct.
     */
    for (at = from; at <= hi; at++) {
        double s = dil_align_score(want, probe, rx, rx_len, at, law);

        if (s > best + 1e-9) {
            best = s;
            best_at = at;
        }
    }

    free(gen);
    free(want);
    if (best_at < 0)
        return false;
    if (offset_out)
        *offset_out = best_at;
    if (score_out)
        *score_out = best;
    return true;
}

bool v90_dil_measure(const uint8_t *rx, int rx_len, v90_law_t law,
                     const v90_dil_desc_t *desc, int offset,
                     v90_dil_measurement_t *out)
{
    int cycle;
    int avail;
    uint8_t *gen;
    /* Per Ucode, how often each received Ucode turned up, so the dominant one
     * and the spread both fall out without a second pass. */
    int (*tally)[V90_DIL_UCODES];
    long *rx_sum;
    int *rx_n;
    int (*slot_tally)[6][V90_DIL_UCODES];
    long (*slot_sum)[6];
    int (*slot_cnt)[6];
    int i;
    int u;
    double gain_acc = 0.0;
    int gain_n = 0;
    int slot_bad[6];
    int slot_n[6];

    if (!rx || !desc || !out || rx_len <= 0 || offset < 0 || offset >= rx_len)
        return false;
    cycle = v90_dil_cycle_len(desc);
    if (cycle <= 0)
        return false;

    avail = rx_len - offset;
    if (avail > cycle)
        avail = cycle;       /* one pass is all the descriptor describes */
    if (avail < 12)
        return false;

    gen = malloc((size_t) avail);
    tally = calloc((size_t) V90_DIL_UCODES, sizeof(*tally));
    rx_sum = calloc((size_t) V90_DIL_UCODES, sizeof(*rx_sum));
    rx_n = calloc((size_t) V90_DIL_UCODES, sizeof(*rx_n));
    slot_tally = calloc((size_t) V90_DIL_UCODES, sizeof(*slot_tally));
    slot_sum = calloc((size_t) V90_DIL_UCODES, sizeof(*slot_sum));
    slot_cnt = calloc((size_t) V90_DIL_UCODES, sizeof(*slot_cnt));
    if (!gen || !tally || !rx_sum || !rx_n
        || !slot_tally || !slot_sum || !slot_cnt) {
        goto fail;
    }
    if (v90_dil_generate_codewords(law, desc, gen, avail) != avail)
        goto fail;

    memset(out, 0, sizeof(*out));
    memset(slot_bad, 0, sizeof(slot_bad));
    memset(slot_n, 0, sizeof(slot_n));

    for (i = 0; i < avail; i++) {
        int tx_u;
        int tx_s;
        int rx_u;
        int rx_s;

        v90_codeword_decompose(law, gen[i], &tx_u, &tx_s);
        v90_codeword_decompose(law, rx[offset + i], &rx_u, &rx_s);
        if (tx_u < 0 || tx_u >= V90_DIL_UCODES)
            continue;
        if (rx_u < 0 || rx_u >= V90_DIL_UCODES)
            continue;
        tally[tx_u][rx_u]++;
        rx_sum[tx_u] += dil_abs_level(law, rx[offset + i]);
        rx_n[tx_u]++;
        out->u[tx_u].tx_count++;
        out->u[tx_u].tx_level = dil_abs_level(law, gen[i]);
        {
            int slot = (i + offset) % 6;

            slot_tally[tx_u][slot][rx_u]++;
            slot_sum[tx_u][slot] += dil_abs_level(law, rx[offset + i]);
            slot_cnt[tx_u][slot]++;
        }
    }

    for (u = 0; u < V90_DIL_UCODES; u++) {
        int best_u = -1;
        int best_n = 0;
        int distinct = 0;
        int r;

        if (out->u[u].tx_count == 0)
            continue;
        for (r = 0; r < V90_DIL_UCODES; r++) {
            if (tally[u][r] == 0)
                continue;
            distinct++;
            if (tally[u][r] > best_n) {
                best_n = tally[u][r];
                best_u = r;
            }
        }
        out->u[u].rx_ucode = best_u;
        out->u[u].rx_agree = best_n;
        out->u[u].rx_distinct = distinct;
        out->u[u].rx_level = rx_n[u] ? (int) (rx_sum[u] / rx_n[u]) : 0;
        out->ucodes_measured++;

        for (r = 0; r < 6; r++) {
            int sbest_u = -1;
            int sbest_n = 0;
            int q;

            for (q = 0; q < V90_DIL_UCODES; q++) {
                if (slot_tally[u][r][q] > sbest_n) {
                    sbest_n = slot_tally[u][r][q];
                    sbest_u = q;
                }
            }
            out->u[u].rx_ucode_slot[r] = sbest_u;
            out->u[u].rx_level_slot[r] = slot_cnt[u][r]
                ? (int) (slot_sum[u][r] / slot_cnt[u][r]) : 0;
        }

        /* A pad shows up as a level ratio; Ucode 0 carries no level to
         * measure a ratio against. */
        if (out->u[u].tx_level > 0 && out->u[u].rx_level > 0) {
            gain_acc += (double) out->u[u].rx_level / (double) out->u[u].tx_level;
            gain_n++;
        }
    }

    /*
     * Robbed-bit signalling steals a bit from one DS0 frame in six, so the
     * damage is not spread evenly: it lands in a fixed phase of the six-symbol
     * frame.  Attribute each disagreement to its phase and the affected slots
     * separate themselves from a uniform impairment such as a pad.
     */
    for (i = 0; i < avail; i++) {
        int tx_u;
        int tx_s;
        int rx_u;
        int rx_s;
        int slot = (i + offset) % 6;

        v90_codeword_decompose(law, gen[i], &tx_u, &tx_s);
        v90_codeword_decompose(law, rx[offset + i], &rx_u, &rx_s);
        if (tx_u < 0 || tx_u >= V90_DIL_UCODES)
            continue;
        slot_n[slot]++;
        if (out->u[tx_u].rx_ucode >= 0 && rx_u != out->u[tx_u].rx_ucode) {
            slot_bad[slot]++;
            out->u[tx_u].slot_disagree |= (uint8_t) (1u << slot);
        }
    }
    for (i = 0; i < 6; i++) {
        if (slot_n[i] > 0 && slot_bad[i] * 4 > slot_n[i])
            out->rbs_slot_mask |= (uint8_t) (1u << i);
    }

    /*
     * Distinguishability, which is the whole point of the exercise: a Ucode is
     * worth offering only if what arrives for it is not what arrives for the
     * Ucode below it.  A pad that compresses the ladder collapses neighbours
     * onto the same received code, and those are exactly the ones to drop.
     */
    {
        int prev = -1;

        for (u = 0; u < V90_DIL_UCODES; u++) {
            if (out->u[u].tx_count == 0)
                continue;
            if (prev < 0 || out->u[u].rx_ucode != out->u[prev].rx_ucode) {
                out->usable[u] = true;
                out->usable_count++;
                prev = u;
            }
        }
    }

    out->align_offset = offset;
    out->symbols_used = avail;
    out->cycle_len = cycle;
    out->coverage = (double) avail / (double) cycle;
    out->gain_db = gain_n ? 20.0 * log10(gain_acc / gain_n) : 0.0;

    free(gen);
    free(tally);
    free(rx_sum);
    free(rx_n);
    free(slot_tally);
    free(slot_sum);
    free(slot_cnt);
    return true;

fail:
    free(gen);
    free(tally);
    free(rx_sum);
    free(rx_n);
    free(slot_tally);
    free(slot_sum);
    free(slot_cnt);
    return false;
}

int v90_dil_measure_usable_ucodes(const v90_dil_measurement_t *m,
                                  uint8_t *out, int max_out)
{
    int n = 0;
    int u;

    if (!m || !out || max_out <= 0)
        return 0;
    for (u = 0; u < V90_DIL_UCODES && n < max_out; u++) {
        if (m->usable[u])
            out[n++] = (uint8_t) u;
    }
    return n;
}

/*
 * Table 15/V.90, indexed in 0.5 dBm0 steps from -0.5 down to -16.  The
 * Recommendation tabulates an amplitude; the limit is its square.
 */
static const int dil_power_limit_amplitude[32] = {
    15124, 14276, 13480, 12724, 12012, 11340, 10708, 10108,
     9544,  9008,  8504,  8028,  7580,  7156,  6756,  6380,
     6020,  5684,  5368,  5068,  4784,  4516,  4264,  4024,
     3800,  3588,  3388,  3196,  3020,  2852,  2692,  2540,
};

double v90_dil_power_limit(double max_tx_dbm0)
{
    int idx;
    double amp;

    if (max_tx_dbm0 > -0.5 || max_tx_dbm0 < -16.0)
        return 0.0;
    idx = (int) ((-max_tx_dbm0) / 0.5 + 0.5) - 1;
    if (idx < 0 || idx >= 32)
        return 0.0;
    amp = dil_power_limit_amplitude[idx];
    return amp * amp;
}

double v90_dil_constellation_power(const uint8_t mask[6][VPCM_CP_MASK_BYTES],
                                   const int mi[6], int k, v90_law_t law)
{
    double total = 0.0;
    double two_k;
    double a_i = 1.0;
    double r_i;
    int i;

    if (!mask || !mi || k <= 0 || k > 60)
        return 0.0;
    two_k = ldexp(1.0, k);
    r_i = two_k - 1.0;      /* §8.5.2: the weights are taken at R0 = 2^K - 1 */

    for (i = 0; i < 6; i++) {
        double levels[V90_DIL_UCODES];
        int n = 0;
        int u;
        double r_next;
        double k_i;
        int j;

        if (mi[i] < 1)
            return 0.0;
        for (u = 0; u < V90_DIL_UCODES && n < mi[i]; u++) {
            if (!vpcm_cp_mask_get(mask[i], u))
                continue;
            levels[n++] = (double) dil_abs_level(
                law, v90_codeword_compose(law, u, 1));
        }
        if (n != mi[i])
            return 0.0;

        /* §5.4.3 modulus encoder, run on R0 = 2^K - 1. */
        k_i = fmod(r_i, (double) mi[i]);
        r_next = floor(r_i / (double) mi[i]);

        for (j = 0; j < mi[i]; j++) {
            double p = levels[j] * levels[j];
            double n_ij;

            if (j < (int) k_i)
                n_ij = a_i * (r_next + 1.0);
            else if (j == (int) k_i)
                n_ij = two_k - a_i * (r_i - r_next);
            else
                n_ij = a_i * r_next;
            total += p * n_ij;
        }

        a_i *= (double) mi[i];
        r_i = r_next;
    }
    return total / (6.0 * two_k);
}

static int dil_drn_for_bits(double bits)
{
    int d;

    for (d = 22; d >= 0; d--) {
        int k = vpcm_cp_drn_to_k((uint8_t) d);

        if (k > 0 && (double) k <= bits)
            return d;
    }
    return -1;
}

bool v90_dil_measure_plan_rate(const v90_dil_measurement_t *m,
                               int level_margin,
                               double max_tx_dbm0,
                               v90_law_t law,
                               v90_dil_rate_plan_t *out)
{
    double bits = 0.0;
    int widest = 0;
    int i;

    if (!m || !out || level_margin < 0)
        return false;

    memset(out, 0, sizeof(*out));

    for (i = 0; i < 6; i++) {
        int prev_level = -1;
        int probed = 0;
        int u;

        /*
         * Walk the ladder upward and keep a Ucode only when what arrived for
         * it in *this* interval is far enough from what arrived for the last
         * one kept.  Neighbours that a pad has compressed onto the same
         * received code, or that robbed-bit signalling has merged by taking
         * the LSB away, drop out here -- which is the impairment restated as
         * constellation points.
         */
        for (u = 0; u < V90_DIL_UCODES; u++) {
            int level;

            if (m->u[u].tx_count == 0)
                continue;
            if (m->u[u].rx_ucode_slot[i] < 0)
                continue;
            level = m->u[u].rx_level_slot[i];
            /* A margin of 0 still means "must have moved": two Ucodes that
             * arrive at the same level are one constellation point, not two. */
            if (prev_level >= 0
                && level - prev_level < (level_margin > 1 ? level_margin : 1))
                continue;
            if (u != 0)
                probed++;
            vpcm_cp_mask_set(out->mask[i], u, true);
            out->mi[i]++;
            prev_level = level;
        }
        if (probed == 0)
            out->intervals_unprobed |= (uint8_t) (1u << i);
        if (out->mi[i] < 1)
            return false;
        if (out->mi[i] > widest)
            widest = out->mi[i];
        bits += log2((double) out->mi[i]);
    }

    out->bits_available = bits;

    /*
     * §5.4.3: the modulus encoder has to fit K bits into the product of the
     * Mi, so the largest drn the line supports is the largest whose K does
     * not exceed sum(log2(Mi)).  §5.4.1 then fixes the rate: D = drn + 20
     * bits per 6-symbol frame at 8000 symbols/s.
     */
    {
        int d = dil_drn_for_bits(bits);

        if (d < 0)
            return false;
        out->drn = (uint8_t) d;
    }

    /*
     * §8.5.2: the set also has to fit Table 15's average-power limit, and
     * that is a second, independent ceiling — a line can be clean enough to
     * carry more points than the transmitter is allowed to send.
     *
     * Drop from the top of the ladder when it does not fit.  G.711 levels
     * grow geometrically, so the largest point dominates the sum and removing
     * it buys the most power for the least rate; taking it from whichever
     * interval currently has the most points keeps the six balanced, which is
     * what maximises prod(Mi) for a given total number of points.
     */
    out->power_limit = v90_dil_power_limit(max_tx_dbm0);
    if (out->power_limit > 0.0) {
        for (;;) {
            int k = vpcm_cp_drn_to_k(out->drn);
            int fattest = 0;
            int u;
            int highest = -1;

            out->avg_power = v90_dil_constellation_power(
                (const uint8_t (*)[VPCM_CP_MASK_BYTES]) out->mask,
                out->mi, k, law);
            if (out->avg_power <= out->power_limit)
                break;

            for (i = 1; i < 6; i++) {
                if (out->mi[i] > out->mi[fattest])
                    fattest = i;
            }
            if (out->mi[fattest] <= 2)
                return false;       /* cannot meet the limit at any rate */
            for (u = V90_DIL_UCODES - 1; u >= 0; u--) {
                if (vpcm_cp_mask_get(out->mask[fattest], u)) {
                    highest = u;
                    break;
                }
            }
            if (highest < 0)
                return false;
            vpcm_cp_mask_set(out->mask[fattest], highest, false);
            out->mi[fattest]--;
            out->points_dropped++;
            out->power_limited = true;

            bits = 0.0;
            for (i = 0; i < 6; i++)
                bits += log2((double) out->mi[i]);
            out->bits_available = bits;
            {
                int d = dil_drn_for_bits(bits);

                if (d < 0)
                    return false;
                out->drn = (uint8_t) d;
            }
        }
    }

    out->robbed_bit_limited = false;
    widest = 0;
    for (i = 0; i < 6; i++) {
        if (out->mi[i] > widest)
            widest = out->mi[i];
    }
    for (i = 0; i < 6; i++) {
        if (out->mi[i] < widest)
            out->robbed_bit_limited = true;
    }

    out->bps = vpcm_cp_drn_to_bps(out->drn);
    return true;
}
