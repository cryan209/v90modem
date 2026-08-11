/*
 * v92_upstream_data.c — V.92 PCM-upstream data-frame core
 *
 * ITU-T V.92 (11/2000), §6.3 and §6.4.  The bit-domain modulus core is
 * followed by the CPd-selected constellation, precoder/prefilter and
 * 16-state V.34 convolutional encoder.  The receive side provides both an
 * exact slicer and an initial two-candidate Viterbi path.
 */

#include "v92_upstream_data.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* V.92 §6.4.3-.4 incorporates V.34 Table 13 and Figure 10 verbatim. */
static const uint8_t v92_conv_input[8][8] = {
    {0,0,1,1,8,8,9,9}, {3,2,2,3,11,10,10,11},
    {5,5,4,4,13,13,12,12}, {6,7,7,6,14,15,15,14},
    {8,8,9,9,0,0,1,1}, {11,10,10,11,3,2,2,3},
    {13,13,12,12,5,5,4,4}, {14,15,15,14,6,7,7,6}
};

/* Figure 10/V.34, 16-state rate-2/3 convolutional encoder. */
static const uint8_t v92_conv16[16][16] = {
    {0x00,0x01,0x06,0x07,0x00,0x01,0x06,0x07,0x00,0x01,0x06,0x07,0x00,0x01,0x06,0x07},
    {0x0C,0x0D,0x0A,0x0B,0x0C,0x0D,0x0A,0x0B,0x0C,0x0D,0x0A,0x0B,0x0C,0x0D,0x0A,0x0B},
    {0x01,0x00,0x07,0x06,0x01,0x00,0x07,0x06,0x01,0x00,0x07,0x06,0x01,0x00,0x07,0x06},
    {0x0D,0x0C,0x0B,0x0A,0x0D,0x0C,0x0B,0x0A,0x0D,0x0C,0x0B,0x0A,0x0D,0x0C,0x0B,0x0A},
    {0x02,0x03,0x04,0x05,0x02,0x03,0x04,0x05,0x02,0x03,0x04,0x05,0x02,0x03,0x04,0x05},
    {0x0E,0x0F,0x08,0x09,0x0E,0x0F,0x08,0x09,0x0E,0x0F,0x08,0x09,0x0E,0x0F,0x08,0x09},
    {0x03,0x02,0x05,0x04,0x03,0x02,0x05,0x04,0x03,0x02,0x05,0x04,0x03,0x02,0x05,0x04},
    {0x0F,0x0E,0x09,0x08,0x0F,0x0E,0x09,0x08,0x0F,0x0E,0x09,0x08,0x0F,0x0E,0x09,0x08},
    {0x04,0x05,0x02,0x03,0x04,0x05,0x02,0x03,0x04,0x05,0x02,0x03,0x04,0x05,0x02,0x03},
    {0x08,0x09,0x0E,0x0F,0x08,0x09,0x0E,0x0F,0x08,0x09,0x0E,0x0F,0x08,0x09,0x0E,0x0F},
    {0x05,0x04,0x03,0x02,0x05,0x04,0x03,0x02,0x05,0x04,0x03,0x02,0x05,0x04,0x03,0x02},
    {0x09,0x08,0x0F,0x0E,0x09,0x08,0x0F,0x0E,0x09,0x08,0x0F,0x0E,0x09,0x08,0x0F,0x0E},
    {0x06,0x07,0x00,0x01,0x06,0x07,0x00,0x01,0x06,0x07,0x00,0x01,0x06,0x07,0x00,0x01},
    {0x0A,0x0B,0x0C,0x0D,0x0A,0x0B,0x0C,0x0D,0x0A,0x0B,0x0C,0x0D,0x0A,0x0B,0x0C,0x0D},
    {0x07,0x06,0x01,0x00,0x07,0x06,0x01,0x00,0x07,0x06,0x01,0x00,0x07,0x06,0x01,0x00},
    {0x0B,0x0A,0x0D,0x0C,0x0B,0x0A,0x0D,0x0C,0x0B,0x0A,0x0D,0x0C,0x0B,0x0A,0x0D,0x0C}
};

#if !defined(__SIZEOF_INT128__)
#error "V.92 upstream modulus coding requires an unsigned integer wider than 72 bits"
#endif

typedef __uint128_t v92_uint128_t;

static bool profile_product(const v92_upstream_profile_t *profile,
                            v92_uint128_t *product_out,
                            int *k_out)
{
    v92_uint128_t product = 1;
    const v92_uint128_t maximum = ~(v92_uint128_t)0;
    int k;

    if (!profile || !product_out)
        return false;
    k = v92_upstream_bits_per_frame(profile->drn);
    if (k <= 0)
        return false;
    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
        unsigned int modulus = profile->moduli[i];

        if (modulus < 2 || product > maximum/modulus)
            return false;
        product *= modulus;
    }
    if (product < ((v92_uint128_t)1 << k))
        return false;
    *product_out = product;
    if (k_out)
        *k_out = k;
    return true;
}

/* GPA = 1 + x^-5 + x^-23, V.92 §6.3 and equation 7-2/V.34. */
static int gpa_scramble_bit(uint32_t *reg, int input)
{
    int output = (input & 1) ^ (int)((*reg >> 4) & 1U)
                              ^ (int)((*reg >> 22) & 1U);

    *reg = ((*reg << 1) | (uint32_t)output) & 0x7FFFFFU;
    return output;
}

static int gpa_descramble_bit(uint32_t *reg, int input)
{
    int output = (input & 1) ^ (int)((*reg >> 4) & 1U)
                              ^ (int)((*reg >> 22) & 1U);

    *reg = ((*reg << 1) | (uint32_t)(input & 1)) & 0x7FFFFFU;
    return output;
}

int v92_upstream_bits_per_frame(uint8_t drn)
{
    if (drn < 1 || drn > 19)
        return 0;
    return 2*((int)drn + 17);
}

bool v92_upstream_profile_validate(const v92_upstream_profile_t *profile)
{
    v92_uint128_t product;

    return profile_product(profile, &product, NULL);
}

void v92_upstream_tx_init(v92_upstream_tx_state_t *state)
{
    if (state)
        memset(state, 0, sizeof(*state));
}

void v92_upstream_rx_init(v92_upstream_rx_state_t *state)
{
    if (state)
        memset(state, 0, sizeof(*state));
}

bool v92_upstream_encode_frame(v92_upstream_tx_state_t *state,
                               const v92_upstream_profile_t *profile,
                               const uint8_t *bits,
                               int bit_count,
                               uint8_t ki_out[V92_UPSTREAM_INTERVALS])
{
    v92_uint128_t product;
    v92_uint128_t r = 0;
    v92_uint128_t r0;
    uint32_t scramble_reg;
    int previous_sign;
    int sign;
    int differential_sign;
    int k;

    if (!state || !bits || !ki_out
        || !profile_product(profile, &product, &k)
        || bit_count != k)
        return false;

    /* §6.4.1 step 1: b0 is the least-significant bit of R.  Work on a
     * temporary register so malformed caller input cannot consume state. */
    scramble_reg = state->scramble_reg;
    for (int i = 0; i < k; i++) {
        if (bits[i] > 1)
            return false;
        if (gpa_scramble_bit(&scramble_reg, bits[i]))
            r |= (v92_uint128_t)1 << i;
    }

    /* §6.4.1 steps 2-4.  R0 uses d(f-1), before the state update. */
    previous_sign = state->previous_differential_sign & 1;
    sign = r > (product - 1)/2;
    differential_sign = sign ^ previous_sign;
    r0 = previous_sign ? product - 1 - r : r;

    /* §6.4.1 step 5: mixed-radix expansion, K0 first. */
    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
        unsigned int modulus = profile->moduli[i];

        ki_out[i] = (uint8_t)(r0 % modulus);
        r0 /= modulus;
    }
    if (r0 != 0)
        return false;
    state->scramble_reg = scramble_reg;
    state->previous_differential_sign = differential_sign;
    return true;
}

bool v92_upstream_decode_frame(v92_upstream_rx_state_t *state,
                               const v92_upstream_profile_t *profile,
                               const uint8_t ki[V92_UPSTREAM_INTERVALS],
                               uint8_t *bits_out,
                               int bits_max)
{
    v92_uint128_t product;
    v92_uint128_t place = 1;
    v92_uint128_t r0 = 0;
    v92_uint128_t r;
    int previous_sign;
    int sign;
    int differential_sign;
    int k;

    if (!state || !ki || !bits_out
        || !profile_product(profile, &product, &k)
        || bits_max < k)
        return false;

    /* Invert §6.4.1 step 5. */
    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
        unsigned int modulus = profile->moduli[i];

        if (ki[i] >= modulus)
            return false;
        r0 += (v92_uint128_t)ki[i]*place;
        place *= modulus;
    }
    if (r0 >= product)
        return false;

    /* Invert steps 2-4 and reject mixed-radix words outside the K-bit
     * source alphabet when product(Mi) is larger than 2^K. */
    previous_sign = state->previous_differential_sign & 1;
    r = previous_sign ? product - 1 - r0 : r0;
    if (r >= ((v92_uint128_t)1 << k))
        return false;
    sign = r > (product - 1)/2;
    differential_sign = sign ^ previous_sign;

    for (int i = 0; i < k; i++) {
        int scrambled = (int)((r >> i) & 1);

        bits_out[i] = (uint8_t)gpa_descramble_bit(&state->descramble_reg,
                                                   scrambled);
    }
    state->previous_differential_sign = differential_sign;
    return true;
}

static bool wave_data_profile(const v92_cpd_frame_t *cpd,
                              v92_upstream_profile_t *profile)
{
    if (!cpd || !profile)
        return false;
    memset(profile, 0, sizeof(*profile));
    profile->drn = cpd->selected_upstream_drn;
    memcpy(profile->moduli, cpd->moduli, sizeof(profile->moduli));
    return v92_upstream_profile_validate(profile);
}

static double q0_15(int16_t value)
{
    return (double)value/32768.0;
}

static double q1_14(int16_t value)
{
    return (double)value/16384.0;
}

static double cpd_gain(const v92_cpd_frame_t *cpd)
{
    /* Table 30 bits 35:50 carry 4G in unsigned Q0.16. */
    return (double)cpd->gain_q0_16/(4.0*65536.0);
}

bool v92_upstream_wave_profile_validate(const v92_cpd_frame_t *cpd)
{
    v92_upstream_profile_t profile;

    if (!cpd || !cpd->modulus_present || !cpd->constellations_present
        || cpd->trellis_select != 0 || cpd_gain(cpd) <= 0.0
        || !wave_data_profile(cpd, &profile))
        return false;
    if (cpd->coeffs_present
        && (cpd->lz1 > V92_CPD_MAX_TAPS
            || cpd->lp1 > V92_CPD_MAX_TAPS
            || cpd->lz2 < 1 || cpd->lz2 > V92_CPD_MAX_TAPS
            || cpd->lp2 > V92_CPD_MAX_TAPS
            || fabs(q0_15(cpd->prefilter_ff[0])) < 1.0e-9))
        return false;
    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
        int set = cpd->dfi[i%6];
        int points;

        if (set < 0 || set >= V92_CPD_MAX_SETS)
            return false;
        points = cpd->set_sizes[set];
        /* LC >= Mi is sufficient for every §6.4.2 equivalence class to
         * have representatives, including k=3's 2Mi class. */
        if (points < profile.moduli[i] || points > V92_CPD_MAX_POINTS)
            return false;
        for (int p = 0; p < points; p++) {
            if (cpd->points[set][p] == 0
                || (p > 0 && cpd->points[set][p]
                            <= cpd->points[set][p - 1]))
                return false;
        }
    }
    return true;
}

void v92_upstream_wave_tx_init(v92_upstream_wave_tx_t *state)
{
    if (state)
        memset(state, 0, sizeof(*state));
}

void v92_upstream_wave_rx_init(v92_upstream_wave_rx_t *state)
{
    if (state)
        memset(state, 0, sizeof(*state));
}

static int positive_mod(int value, int modulus)
{
    int result = value%modulus;

    return result < 0 ? result + modulus : result;
}

static double constellation_value(const v92_cpd_frame_t *cpd,
                                  int set,
                                  int eta)
{
    return eta >= 0 ? (double)cpd->points[set][eta]
                    : -(double)cpd->points[set][-eta - 1];
}

static int subset_label(int eta0, int eta1)
{
    int re = 2*eta0 + 1;
    int im = 2*eta1 + 1;
    int xored = re ^ im;
    int x = xored & 2;

    /* V.34 §9.6.3.1, Figure 9. */
    return ((xored & 4) ^ (x << 1)) | (re & 2) | (x >> 1);
}

static void push_history(double history[V92_CPD_MAX_TAPS], double value)
{
    memmove(&history[1], &history[0],
            (V92_CPD_MAX_TAPS - 1)*sizeof(history[0]));
    history[0] = value;
}

static double precoder_fixed(const v92_cpd_frame_t *cpd,
                             const double u_history[V92_CPD_MAX_TAPS],
                             const double x_history[V92_CPD_MAX_TAPS])
{
    double fixed = 0.0;

    if (!cpd->coeffs_present)
        return 0.0;
    for (int k = 0; k < cpd->lz1; k++)
        fixed += u_history[k]*q0_15(cpd->precoder_ff[k]);
    for (int k = 0; k < cpd->lp1; k++)
        fixed += x_history[k]*q1_14(cpd->precoder_fb[k]);
    return fixed;
}

static double prefilter_output(const v92_cpd_frame_t *cpd,
                               double x,
                               const double x_history[V92_CPD_MAX_TAPS],
                               const double v_history[V92_CPD_MAX_TAPS])
{
    double v;

    if (!cpd->coeffs_present)
        return x;
    v = x*q0_15(cpd->prefilter_ff[0]);
    for (int k = 1; k < cpd->lz2; k++)
        v += x_history[k - 1]*q0_15(cpd->prefilter_ff[k]);
    for (int k = 0; k < cpd->lp2; k++)
        v += v_history[k]*q1_14(cpd->prefilter_fb[k]);
    return v;
}

static bool select_equivalence_point(const v92_cpd_frame_t *cpd,
                                     int interval,
                                     int ki,
                                     int y0,
                                     const int eta_frame[4],
                                     const double u_history[V92_CPD_MAX_TAPS],
                                     const double x_history[V92_CPD_MAX_TAPS],
                                     int *eta_out,
                                     double *u_out,
                                     double *x_out)
{
    int set = cpd->dfi[interval%6];
    int lc = cpd->set_sizes[set];
    int modulus = cpd->moduli[interval];
    int k = interval%4;
    int parity = 0;
    double fixed = precoder_fixed(cpd, u_history, x_history);
    double best_cost = HUGE_VAL;
    int best_eta = 0;
    double best_u = 0.0;
    double best_x = 0.0;
    bool found = false;

    if (k == 3)
        parity = positive_mod(eta_frame[0] + eta_frame[1]
                            + eta_frame[2] + y0, 2);
    for (int eta = -lc; eta < lc; eta++) {
        bool member;
        double u;
        double x;
        double cost;

        if (k < 3) {
            member = positive_mod(eta, modulus) == ki;
        } else {
            member = positive_mod(eta - parity, 2) == 0
                  && positive_mod((eta - parity)/2, modulus) == ki;
        }
        if (!member)
            continue;
        u = constellation_value(cpd, set, eta);
        x = u + fixed;
        cost = fabs(x);
        if (!found || cost < best_cost
            || (cost == best_cost && abs(eta) < abs(best_eta))) {
            found = true;
            best_cost = cost;
            best_eta = eta;
            best_u = u;
            best_x = x;
        }
    }
    if (!found)
        return false;
    *eta_out = best_eta;
    *u_out = best_u;
    *x_out = best_x;
    return true;
}

bool v92_upstream_wave_encode_frame(v92_upstream_wave_tx_t *state,
                                    const v92_cpd_frame_t *cpd,
                                    const uint8_t *bits,
                                    int bit_count,
                                    double samples[V92_UPSTREAM_INTERVALS])
{
    v92_upstream_wave_tx_t trial;
    v92_upstream_profile_t profile;
    uint8_t ki[V92_UPSTREAM_INTERVALS];
    int eta_frame[4] = {0};
    double gain;

    if (!state || !bits || !samples
        || !v92_upstream_wave_profile_validate(cpd)
        || !wave_data_profile(cpd, &profile))
        return false;
    trial = *state;
    if (!v92_upstream_encode_frame(&trial.data, &profile, bits, bit_count, ki))
        return false;
    gain = cpd_gain(cpd);
    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
        double u;
        double x;
        double v;
        int eta;
        int k = i%4;

        if (!select_equivalence_point(cpd, i, ki[i],
                                      trial.convolutional_state & 1,
                                      eta_frame,
                                      trial.u_history, trial.x_history,
                                      &eta, &u, &x))
            return false;
        eta_frame[k] = eta;
        v = prefilter_output(cpd, x, trial.x_history, trial.v_history);
        samples[i] = gain*v;
        push_history(trial.u_history, u);
        push_history(trial.x_history, x);
        push_history(trial.v_history, v);

        if (k == 3) {
            int s0 = subset_label(eta_frame[0], eta_frame[1]);
            int s1 = subset_label(eta_frame[2], eta_frame[3]);
            int input = v92_conv_input[s0][s1];

            /* V.92 §6.4.4 replaces V.34's 2T delays with 4T delays: one
             * Figure-10 transition follows each four upstream symbols. */
            trial.convolutional_state =
                v92_conv16[trial.convolutional_state][input];
        }
    }
    *state = trial;
    return true;
}

static bool recover_symbol(const v92_cpd_frame_t *cpd,
                           int interval,
                           double sample,
                           v92_upstream_wave_rx_t *state,
                           int *eta_out,
                           double *u_out,
                           double *x_out,
                           double *v_out)
{
    int set = cpd->dfi[interval%6];
    int lc = cpd->set_sizes[set];
    double gain = cpd_gain(cpd);
    double v = sample/gain;
    double x;
    double fixed;
    double u;
    double best_error = HUGE_VAL;
    int best_eta = 0;

    if (!cpd->coeffs_present) {
        x = v;
    } else {
        double residual = v;
        double z0 = q0_15(cpd->prefilter_ff[0]);

        for (int k = 1; k < cpd->lz2; k++)
            residual -= state->x_history[k - 1]
                      * q0_15(cpd->prefilter_ff[k]);
        for (int k = 0; k < cpd->lp2; k++)
            residual -= state->v_history[k]
                      * q1_14(cpd->prefilter_fb[k]);
        x = residual/z0;
    }
    fixed = precoder_fixed(cpd, state->u_history, state->x_history);
    u = x - fixed;
    for (int eta = -lc; eta < lc; eta++) {
        double candidate = constellation_value(cpd, set, eta);
        double error = fabs(u - candidate);

        if (error < best_error) {
            best_error = error;
            best_eta = eta;
        }
    }
    *eta_out = best_eta;
    *u_out = constellation_value(cpd, set, best_eta);
    *x_out = x;
    *v_out = v;
    if (best_error > 0.25)
        state->slicing_errors++;
    return true;
}

bool v92_upstream_wave_decode_frame(v92_upstream_wave_rx_t *state,
                                    const v92_cpd_frame_t *cpd,
                                    const double samples[V92_UPSTREAM_INTERVALS],
                                    uint8_t *bits_out,
                                    int bits_max)
{
    v92_upstream_wave_rx_t trial;
    v92_upstream_profile_t profile;
    uint8_t ki[V92_UPSTREAM_INTERVALS];
    int eta_frame[4] = {0};

    if (!state || !samples || !bits_out
        || !v92_upstream_wave_profile_validate(cpd)
        || !wave_data_profile(cpd, &profile))
        return false;
    trial = *state;
    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
        double u;
        double x;
        double v;
        int eta;
        int modulus = profile.moduli[i];
        int k = i%4;

        if (!recover_symbol(cpd, i, samples[i], &trial,
                            &eta, &u, &x, &v))
            return false;
        eta_frame[k] = eta;
        if (k < 3) {
            ki[i] = (uint8_t)positive_mod(eta, modulus);
        } else {
            int parity = positive_mod(eta_frame[0] + eta_frame[1]
                                    + eta_frame[2]
                                    + (trial.convolutional_state & 1), 2);

            if (positive_mod(eta - parity, 2) != 0)
                return false;
            ki[i] = (uint8_t)positive_mod((eta - parity)/2, modulus);
        }
        push_history(trial.u_history, u);
        push_history(trial.x_history, x);
        push_history(trial.v_history, v);
        trial.symbols++;

        if (k == 3) {
            int s0 = subset_label(eta_frame[0], eta_frame[1]);
            int s1 = subset_label(eta_frame[2], eta_frame[3]);
            int input = v92_conv_input[s0][s1];

            trial.convolutional_state =
                v92_conv16[trial.convolutional_state][input];
        }
    }
    if (!v92_upstream_decode_frame(&trial.data, &profile, ki,
                                   bits_out, bits_max))
        return false;
    *state = trial;
    return true;
}

typedef struct {
    bool valid;
    double metric;
    int eta[V92_UPSTREAM_INTERVALS];
} v92_viterbi_path_t;

static void two_nearest_points(const v92_cpd_frame_t *cpd,
                               int interval,
                               double observation,
                               int eta_out[2],
                               double error_out[2])
{
    int set = cpd->dfi[interval%6];
    int lc = cpd->set_sizes[set];

    error_out[0] = error_out[1] = HUGE_VAL;
    eta_out[0] = eta_out[1] = 0;
    for (int eta = -lc; eta < lc; eta++) {
        double point = constellation_value(cpd, set, eta);
        double error = observation - point;

        error *= error;
        if (error < error_out[0]) {
            error_out[1] = error_out[0];
            eta_out[1] = eta_out[0];
            error_out[0] = error;
            eta_out[0] = eta;
        } else if (error < error_out[1]) {
            error_out[1] = error;
            eta_out[1] = eta;
        }
    }
}

bool v92_upstream_wave_decode_viterbi_frame(
    v92_upstream_wave_rx_t *state,
    const v92_cpd_frame_t *cpd,
    const double samples[V92_UPSTREAM_INTERVALS],
    uint8_t *bits_out,
    int bits_max)
{
    v92_upstream_profile_t profile;
    v92_viterbi_path_t paths[16];
    v92_viterbi_path_t next[16];
    int candidates[V92_UPSTREAM_INTERVALS][2];
    double errors[V92_UPSTREAM_INTERVALS][2];
    double gain;
    int initial_state;
    int best_state = -1;
    double best_metric = HUGE_VAL;
    uint8_t best_bits[V92_UPSTREAM_MAX_FRAME_BITS];
    v92_upstream_rx_state_t best_data;

    if (!state || !samples || !bits_out || cpd->coeffs_present
        || !v92_upstream_wave_profile_validate(cpd)
        || !wave_data_profile(cpd, &profile)
        || bits_max < v92_upstream_bits_per_frame(profile.drn))
        return false;
    gain = cpd_gain(cpd);
    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++)
        two_nearest_points(cpd, i, samples[i]/gain,
                           candidates[i], errors[i]);

    memset(paths, 0, sizeof(paths));
    initial_state = state->convolutional_state;
    paths[initial_state].valid = true;
    for (int group = 0; group < 3; group++) {
        int base = 4*group;

        memset(next, 0, sizeof(next));
        for (int previous = 0; previous < 16; previous++) {
            if (!paths[previous].valid)
                continue;
            for (int choices = 0; choices < 16; choices++) {
                int eta[4];
                int parity;
                int s0;
                int s1;
                int input;
                int destination;
                double metric = paths[previous].metric;

                for (int k = 0; k < 4; k++) {
                    int choice = (choices >> k) & 1;

                    eta[k] = candidates[base + k][choice];
                    metric += errors[base + k][choice];
                }
                parity = positive_mod(eta[0] + eta[1] + eta[2]
                                    + (previous & 1), 2);
                /* §6.4.2's k=3 equivalence class carries Y0. */
                if (positive_mod(eta[3] - parity, 2) != 0)
                    continue;
                s0 = subset_label(eta[0], eta[1]);
                s1 = subset_label(eta[2], eta[3]);
                input = v92_conv_input[s0][s1];
                destination = v92_conv16[previous][input];
                if (!next[destination].valid
                    || metric < next[destination].metric) {
                    next[destination] = paths[previous];
                    next[destination].valid = true;
                    next[destination].metric = metric;
                    memcpy(&next[destination].eta[base], eta, sizeof(eta));
                }
            }
        }
        memcpy(paths, next, sizeof(paths));
    }

    /* A modulus product may exceed 2^K, so a geometrically valid survivor
     * can still be outside the source alphabet.  Select the cheapest path
     * that also passes the exact §6.4.1 inverse. */
    for (int destination = 0; destination < 16; destination++) {
        v92_upstream_rx_state_t data_trial;
        uint8_t ki[V92_UPSTREAM_INTERVALS];
        uint8_t recovered[V92_UPSTREAM_MAX_FRAME_BITS];

        if (!paths[destination].valid || paths[destination].metric >= best_metric)
            continue;
        for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
            int eta = paths[destination].eta[i];
            int modulus = profile.moduli[i];
            int k = i%4;

            if (k < 3) {
                ki[i] = (uint8_t)positive_mod(eta, modulus);
            } else {
                int base = i - 3;
                int y0_state = initial_state;

                /* Replay the two preceding trellis transitions to obtain
                 * the state whose Y0 belongs to this four-symbol group. */
                for (int g = 0; g < i/4; g++) {
                    int p = 4*g;
                    int a = subset_label(paths[destination].eta[p],
                                         paths[destination].eta[p + 1]);
                    int b = subset_label(paths[destination].eta[p + 2],
                                         paths[destination].eta[p + 3]);
                    y0_state = v92_conv16[y0_state][v92_conv_input[a][b]];
                }
                {
                    int parity = positive_mod(paths[destination].eta[base]
                                            + paths[destination].eta[base + 1]
                                            + paths[destination].eta[base + 2]
                                            + (y0_state & 1), 2);
                    ki[i] = (uint8_t)positive_mod((eta - parity)/2,
                                                  modulus);
                }
            }
        }
        data_trial = state->data;
        if (!v92_upstream_decode_frame(&data_trial, &profile, ki,
                                       recovered, (int)sizeof(recovered)))
            continue;
        best_state = destination;
        best_metric = paths[destination].metric;
        best_data = data_trial;
        memcpy(best_bits, recovered,
               (size_t)v92_upstream_bits_per_frame(profile.drn));
    }
    if (best_state < 0)
        return false;

    for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
        int set = cpd->dfi[i%6];
        int eta = paths[best_state].eta[i];
        double u = constellation_value(cpd, set, eta);

        if (eta != candidates[i][0])
            state->slicing_errors++;
        push_history(state->u_history, u);
        push_history(state->x_history, u);
        push_history(state->v_history, u);
        state->symbols++;
    }
    state->data = best_data;
    state->convolutional_state = (uint8_t)best_state;
    memcpy(bits_out, best_bits,
           (size_t)v92_upstream_bits_per_frame(profile.drn));
    return true;
}
