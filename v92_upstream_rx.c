/*
 * v92_upstream_rx.c — V.92 B1u acquisition and upstream byte delivery
 *
 * References: ITU-T V.92 (11/2000), §8.7.1 and §9.6.1.1.6.
 */

#include "v92_upstream_rx.h"

#include <math.h>
#include <string.h>

static void acquisition_linearize(const v92_upstream_rx_t *rx, double *out)
{
    for (int i = 0; i < V92_B1U_SYMBOLS; i++)
        out[i] = rx->acquisition[(rx->acquisition_pos + i)%V92_B1U_SYMBOLS];
}

static bool fit_reference(const double *reference,
                          const double *observed,
                          int count,
                          double *gain_out,
                          double *offset_out,
                          double *correlation_out)
{
    double mean_x = 0.0;
    double mean_y = 0.0;
    double covariance = 0.0;
    double variance_x = 0.0;
    double variance_y = 0.0;

    for (int i = 0; i < count; i++) {
        mean_x += reference[i];
        mean_y += observed[i];
    }
    mean_x /= count;
    mean_y /= count;
    for (int i = 0; i < count; i++) {
        double x = reference[i] - mean_x;
        double y = observed[i] - mean_y;

        covariance += x*y;
        variance_x += x*x;
        variance_y += y*y;
    }
    if (variance_x <= 0.0 || variance_y <= 0.0 || covariance <= 0.0)
        return false;
    *gain_out = covariance/variance_x;
    *offset_out = mean_y - *gain_out*mean_x;
    *correlation_out = covariance/sqrt(variance_x*variance_y);
    return *gain_out > 0.0;
}

bool v92_upstream_b1_rx_init(v92_upstream_rx_t *rx,
                          const v92_cpd_frame_t *cpd,
                          v92_upstream_byte_handler_t handler,
                          void *user_data)
{
    v92_upstream_wave_tx_t tx;
    v92_upstream_wave_rx_t reference_rx;
    uint8_t ones[V92_UPSTREAM_MAX_FRAME_BITS];
    uint8_t recovered[V92_UPSTREAM_MAX_FRAME_BITS];
    int k;

    if (!rx || !cpd || !v92_upstream_wave_profile_validate(cpd))
        return false;
    memset(rx, 0, sizeof(*rx));
    rx->cpd = *cpd;
    rx->handler = handler;
    rx->user_data = user_data;
    k = v92_upstream_bits_per_frame(cpd->selected_upstream_drn);
    memset(ones, 1, sizeof(ones));
    v92_upstream_wave_tx_init(&tx);
    v92_upstream_wave_rx_init(&reference_rx);
    for (int frame = 0; frame < V92_B1U_FRAMES; frame++) {
        if (!v92_upstream_wave_encode_frame(
                &tx, cpd, ones, k,
                &rx->reference[frame*V92_UPSTREAM_INTERVALS])
            || !v92_upstream_wave_decode_frame(
                &reference_rx, cpd,
                &rx->reference[frame*V92_UPSTREAM_INTERVALS],
                recovered, (int)sizeof(recovered))) {
            memset(rx, 0, sizeof(*rx));
            return false;
        }
    }
    rx->b1_final_rx = reference_rx;
    rx->b1_final_tx = tx;
    rx->decision_tx = tx;
    v92_upstream_wave_rx_init(&rx->wave_rx);
    return true;
}

static bool solve_linear(double a[V92_UPSTREAM_EQ_TAPS + 1]
                                  [V92_UPSTREAM_EQ_TAPS + 1],
                         double b[V92_UPSTREAM_EQ_TAPS + 1],
                         double x[V92_UPSTREAM_EQ_TAPS + 1])
{
    const int n = V92_UPSTREAM_EQ_TAPS + 1;

    for (int col = 0; col < n; col++) {
        int pivot = col;

        for (int row = col + 1; row < n; row++) {
            if (fabs(a[row][col]) > fabs(a[pivot][col]))
                pivot = row;
        }
        if (fabs(a[pivot][col]) < 1.0e-12)
            return false;
        if (pivot != col) {
            for (int j = col; j < n; j++) {
                double t = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = t;
            }
            {
                double t = b[col];
                b[col] = b[pivot];
                b[pivot] = t;
            }
        }
        for (int row = col + 1; row < n; row++) {
            double factor = a[row][col]/a[col][col];

            for (int j = col; j < n; j++)
                a[row][j] -= factor*a[col][j];
            b[row] -= factor*b[col];
        }
    }
    for (int row = n - 1; row >= 0; row--) {
        double sum = b[row];

        for (int j = row + 1; j < n; j++)
            sum -= a[row][j]*x[j];
        x[row] = sum/a[row][row];
    }
    return true;
}

static bool train_equalizer(v92_upstream_rx_t *rx,
                            const double observed[V92_B1U_SYMBOLS],
                            double *correlation_out)
{
    const int dimension = V92_UPSTREAM_EQ_TAPS + 1;
    const int delay = V92_UPSTREAM_EQ_TAPS/2;
    double normal[V92_UPSTREAM_EQ_TAPS + 1]
                 [V92_UPSTREAM_EQ_TAPS + 1] = {{0}};
    double rhs[V92_UPSTREAM_EQ_TAPS + 1] = {0};
    double solution[V92_UPSTREAM_EQ_TAPS + 1] = {0};
    double desired[V92_B1U_SYMBOLS];
    double equalized[V92_B1U_SYMBOLS];
    int count = 0;
    double scale;
    double offset;
    double correlation;

    for (int n = V92_UPSTREAM_EQ_TAPS - 1; n < V92_B1U_SYMBOLS; n++) {
        int target = n - delay;
        double vector[V92_UPSTREAM_EQ_TAPS + 1];

        if (target < 0 || target >= V92_B1U_SYMBOLS)
            continue;
        for (int tap = 0; tap < V92_UPSTREAM_EQ_TAPS; tap++)
            vector[tap] = observed[n - tap];
        vector[V92_UPSTREAM_EQ_TAPS] = 1.0;
        for (int row = 0; row < dimension; row++) {
            rhs[row] += vector[row]*rx->reference[target];
            for (int col = 0; col < dimension; col++)
                normal[row][col] += vector[row]*vector[col];
        }
    }
    /* Small diagonal loading keeps quantized, nearly collinear B1u windows
     * invertible without materially biasing the seven-tap solution. */
    for (int i = 0; i < V92_UPSTREAM_EQ_TAPS; i++)
        normal[i][i] += 1.0e-8*normal[i][i] + 1.0e-9;
    if (!solve_linear(normal, rhs, solution))
        return false;
    for (int n = V92_UPSTREAM_EQ_TAPS - 1; n < V92_B1U_SYMBOLS; n++) {
        int target = n - delay;
        double value = solution[V92_UPSTREAM_EQ_TAPS];

        for (int tap = 0; tap < V92_UPSTREAM_EQ_TAPS; tap++)
            value += solution[tap]*observed[n - tap];
        desired[count] = rx->reference[target];
        equalized[count++] = value;
    }
    if (!fit_reference(desired, equalized, count,
                       &scale, &offset, &correlation)
        || correlation < 0.995 || fabs(scale) < 1.0e-9)
        return false;
    for (int tap = 0; tap < V92_UPSTREAM_EQ_TAPS; tap++)
        rx->equalizer[tap] = solution[tap]/scale;
    rx->equalizer_offset =
        (solution[V92_UPSTREAM_EQ_TAPS] - offset)/scale;
    rx->equalizer_delay = delay;
    rx->equalizer_discard = delay;
    rx->equalizer_trained = true;
    *correlation_out = correlation;
    return true;
}

static bool try_b1u_lock(v92_upstream_rx_t *rx)
{
    double observed[V92_B1U_SYMBOLS];
    double normalized[V92_UPSTREAM_INTERVALS];
    v92_upstream_wave_rx_t trial;
    uint8_t bits[V92_UPSTREAM_MAX_FRAME_BITS];
    double gain;
    double offset;
    double correlation;
    int k = v92_upstream_bits_per_frame(rx->cpd.selected_upstream_drn);

    acquisition_linearize(rx, observed);
    if (!fit_reference(rx->reference, observed, V92_B1U_SYMBOLS,
                       &gain, &offset, &correlation)
        || correlation < 0.20)
        return false;

    if (correlation >= 0.999) {
        /* Correlation alone is not enough: decode all 48 §8.7.1 frames and
         * require the known source ones before committing state. */
        v92_upstream_wave_rx_init(&trial);
        for (int frame = 0; frame < V92_B1U_FRAMES; frame++) {
            for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++)
                normalized[i] =
                    (observed[frame*V92_UPSTREAM_INTERVALS + i] - offset)/gain;
            if (!v92_upstream_wave_decode_frame(&trial, &rx->cpd, normalized,
                                                bits, (int)sizeof(bits)))
                return false;
            for (int i = 0; i < k; i++) {
                if (bits[i] != 1)
                    return false;
            }
        }
        memset(rx->equalizer, 0, sizeof(rx->equalizer));
        rx->equalizer[0] = 1.0/gain;
        rx->equalizer_offset = -offset/gain;
        rx->equalizer_delay = 0;
        rx->equalizer_discard = 0;
        rx->equalizer_trained = true;
        rx->wave_rx = trial;
        rx->decision_tx = rx->b1_final_tx;
    } else {
        /* A fractional sampling phase and the analogue channel appear as a
         * symbol-spaced FIR at the DS0 output.  Train its seven-tap inverse
         * on B1u and use the known final B1u state as §8.7.1 permits. */
        if (!train_equalizer(rx, observed, &correlation))
            return false;
        rx->wave_rx = rx->b1_final_rx;
        rx->decision_tx = rx->b1_final_tx;
    }
    for (int tap = 0; tap < V92_UPSTREAM_EQ_TAPS; tap++)
        rx->equalizer_history[tap] =
            observed[V92_B1U_SYMBOLS - 1 - tap];
    rx->gain = gain;
    rx->offset = offset;
    rx->correlation = correlation;
    rx->locked = true;
    rx->frame_pos = 0;
    return true;
}

static int deliver_frame(v92_upstream_rx_t *rx)
{
    uint8_t bits[V92_UPSTREAM_MAX_FRAME_BITS];
    double target[V92_UPSTREAM_INTERVALS];
    int before = (int)rx->output_bytes;
    int k = v92_upstream_bits_per_frame(rx->cpd.selected_upstream_drn);
    bool ok;

    if (rx->cpd.coeffs_present) {
        ok = v92_upstream_wave_decode_frame(&rx->wave_rx, &rx->cpd,
                                            rx->frame, bits,
                                            (int)sizeof(bits));
    } else {
        ok = v92_upstream_wave_decode_viterbi_frame(
            &rx->wave_rx, &rx->cpd, rx->frame, bits, (int)sizeof(bits));
    }
    if (!ok) {
        rx->rejected_frames++;
        return 0;
    }

    /* Decision-directed normalized LMS.  Re-encoding the recovered source
     * frame supplies the §6.4 target levels while preserving all modulus,
     * convolutional and filter memories from the known end of B1u. */
    if (v92_upstream_wave_encode_frame(&rx->decision_tx, &rx->cpd,
                                       bits, k, target)) {
        const double mu = 0.05;

        for (int i = 0; i < V92_UPSTREAM_INTERVALS; i++) {
            double error = target[i] - rx->frame[i];
            double norm = 1.0;

            for (int tap = 0; tap < V92_UPSTREAM_EQ_TAPS; tap++)
                norm += rx->frame_inputs[i][tap]
                      * rx->frame_inputs[i][tap];
            for (int tap = 0; tap < V92_UPSTREAM_EQ_TAPS; tap++)
                rx->equalizer[tap] += mu*error
                    *rx->frame_inputs[i][tap]/norm;
            rx->equalizer_offset += mu*error/norm;
            rx->equalizer_updates++;
        }
    }
    for (int i = 0; i < k; i++) {
        rx->byte_accumulator |= (uint8_t)(bits[i] << rx->byte_bits);
        rx->byte_bits++;
        rx->output_bits++;
        if (rx->byte_bits == 8) {
            if (rx->handler)
                rx->handler(rx->user_data, rx->byte_accumulator);
            rx->byte_accumulator = 0;
            rx->byte_bits = 0;
            rx->output_bytes++;
        }
    }
    return (int)rx->output_bytes - before;
}

int v92_upstream_b1_rx_feed(v92_upstream_rx_t *rx,
                         const int16_t *samples,
                         int count)
{
    int delivered = 0;

    if (!rx || !samples || count < 0)
        return 0;
    for (int n = 0; n < count; n++) {
        rx->input_symbols++;
        if (!rx->locked) {
            rx->acquisition[rx->acquisition_pos] = samples[n];
            rx->acquisition_pos =
                (rx->acquisition_pos + 1)%V92_B1U_SYMBOLS;
            if (rx->acquisition_count < V92_B1U_SYMBOLS)
                rx->acquisition_count++;
            if (rx->acquisition_count == V92_B1U_SYMBOLS
                && try_b1u_lock(rx))
                continue;
            continue;
        }
        {
            double equalized = rx->equalizer_offset;

            memmove(&rx->equalizer_history[1],
                    &rx->equalizer_history[0],
                    (V92_UPSTREAM_EQ_TAPS - 1)
                    *sizeof(rx->equalizer_history[0]));
            rx->equalizer_history[0] = samples[n];
            for (int tap = 0; tap < V92_UPSTREAM_EQ_TAPS; tap++)
                equalized += rx->equalizer[tap]
                           * rx->equalizer_history[tap];
            if (rx->equalizer_discard > 0) {
                rx->equalizer_discard--;
                continue;
            }
            memcpy(rx->frame_inputs[rx->frame_pos],
                   rx->equalizer_history,
                   sizeof(rx->equalizer_history));
            rx->frame[rx->frame_pos++] = equalized;
        }
        if (rx->frame_pos == V92_UPSTREAM_INTERVALS) {
            delivered += deliver_frame(rx);
            rx->frame_pos = 0;
        }
    }
    return delivered;
}
