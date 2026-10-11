/*
 * bell212a_test.c -- SpanDSP's V.22bis datapump in Bell 212A mode.
 *
 * V22BIS_BELL212A_COMPATIBILITY_MODE (spandsp/v22bis.h) makes the answering
 * modem send the Bell 2225 Hz answer tone where V.22 sends unscrambled ones,
 * until its receiver hears the caller's scrambled ones.  This checks, on the
 * datapump alone:
 *
 *   1. the answerer's line signal before the caller speaks is 2225 Hz and
 *      nothing else, and plain V.22's unscrambled ones are not (so the
 *      option is what made the tone);
 *   2. a 1200 bit/s caller (V.22, or a 212A originate modem, which are the
 *      same datapump) takes the tone as unscrambled ones and both ends train
 *      and pass error-free data both ways;
 *   3. a 2400 bit/s V.22bis caller does the same and settles at 1200, as
 *      V.22bis 6.3.1.1.1 has it do against a V.22 answerer;
 *   4. the tone is gone once data flows.
 *
 * Each direction carries PRBS9 (x^9 + x^5 + 1), checked self-synchronously.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <spandsp.h>

#define FRAME   160
#define RATE    8000

typedef struct side side_t;
struct side {
    v22bis_state_t *m;
    side_t *peer;
    const char *name;
    int trained_at;         /* sample index of TRAINING_SUCCEEDED, -1 if not */
    int now;
    int delay;              /* one-way bearer delay in samples */
    uint16_t tx_reg;        /* PRBS9 generator */
    uint16_t rx_hist;       /* last nine bits received */
    long rx_bits, rx_errors;
};

static int failures;

static void check(int ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        failures++;
}

static int get_bit(void *user)
{
    side_t *s = user;
    int bit = ((s->tx_reg >> 8) ^ (s->tx_reg >> 4)) & 1;

    s->tx_reg = (uint16_t) (((s->tx_reg << 1) | bit) & 0x1FF);
    return bit;
}

static void put_bit(void *user, int bit)
{
    side_t *s = user;

    if (bit < 0) {
        if (bit == SIG_STATUS_TRAINING_SUCCEEDED && s->trained_at < 0)
            s->trained_at = s->now;
        return;
    }
    /* Grade from 200 ms after BOTH ends have trained: until the far end has,
       it is still sending the handshake's scrambled ones, and the
       descrambler and the PRBS both need to have filled before the
       prediction means anything. */
    int from = s->trained_at > s->peer->trained_at ? s->trained_at : s->peer->trained_at;

    if (s->trained_at >= 0 && s->peer->trained_at >= 0 && s->now >= from + s->delay + RATE/5
        && ++s->rx_bits > 20) {
        int want = ((s->rx_hist >> 8) ^ (s->rx_hist >> 4)) & 1;

        if (bit != want)
            s->rx_errors++;
    }
    s->rx_hist = (uint16_t) (((s->rx_hist << 1) | bit) & 0x1FF);
}

/* Fraction of the block's power at f: 2|X(f)|^2 / (N sum x^2). */
static double tone_fraction(const int16_t *x, int n, double f)
{
    double c = 2.0*cos(2.0*M_PI*f/RATE), s1 = 0.0, s2 = 0.0, e = 0.0;

    for (int i = 0; i < n; i++) {
        double v = x[i] + c*s1 - s2;

        s2 = s1;
        s1 = v;
        e += (double) x[i]*x[i];
    }
    return e > 0.0 ? 2.0*(s1*s1 + s2*s2 - c*s1*s2)/(n*e) : 0.0;
}

static double rms(const int16_t *x, int n)
{
    double e = 0.0;

    for (int i = 0; i < n; i++)
        e += (double) x[i]*x[i];
    return sqrt(e/n);
}

/* The answerer alone, nobody calling: 75 ms of silence, then its opening
   signal, 100-1000 ms graded in 100 ms (10 Hz) blocks. */
static double answer_alone(int options, double *level)
{
    static int16_t tx[RATE];
    int16_t rx[FRAME] = {0};
    side_t a = { .name = "answer", .trained_at = -1, .tx_reg = 0x1FF };

    a.peer = &a;
    double worst = 1.0;

    a.m = v22bis_init(NULL, 1200, options, false, get_bit, &a, put_bit, &a);
    for (int t = 0; t < RATE; t += FRAME) {
        v22bis_tx(a.m, tx + t, FRAME);
        v22bis_rx(a.m, rx, FRAME);
    }
    for (int t = 800; t < RATE; t += 800) {
        double fr = tone_fraction(tx + t, 800, 2225.0);

        if (fr < worst)
            worst = fr;
    }
    *level = rms(tx + 800, RATE - 800);
    v22bis_free(a.m);
    return worst;
}

static void pair(int caller_rate, int bell, int delay_ms, const char *label)
{
    static int16_t ans_tx[RATE*12], call_tx[RATE*12];
    const int d = delay_ms*RATE/1000;   /* one-way bearer delay, a whole number of frames */
    int16_t silence[FRAME] = {0};
    side_t c = { .name = "call", .trained_at = -1, .tx_reg = 0x1FF };
    side_t a = { .name = "answer", .trained_at = -1, .tx_reg = 0x0F3 };
    const int total = RATE*12;
    char what[160];

    printf("%s\n", label);
    if (getenv("BELL212A_TRACE"))
        printf("  (SpanDSP flow log follows on stderr)\n");
    c.peer = &a;
    a.peer = &c;
    c.delay = a.delay = d;
    c.m = v22bis_init(NULL, caller_rate, V22BIS_GUARD_TONE_NONE | (bell == 2 ? V22BIS_BELL212A_COMPATIBILITY_MODE : 0),
                      true, get_bit, &c, put_bit, &c);
    a.m = v22bis_init(NULL, 1200, V22BIS_GUARD_TONE_NONE | (bell ? V22BIS_BELL212A_COMPATIBILITY_MODE : 0),
                      false, get_bit, &a, put_bit, &a);
    if (getenv("BELL212A_TRACE")) {
        span_log_set_level(v22bis_get_logging_state(c.m), SPAN_LOG_FLOW | SPAN_LOG_SHOW_TAG | SPAN_LOG_SHOW_SAMPLE_TIME);
        span_log_set_tag(v22bis_get_logging_state(c.m), "call");
        span_log_set_level(v22bis_get_logging_state(a.m), SPAN_LOG_FLOW | SPAN_LOG_SHOW_TAG | SPAN_LOG_SHOW_SAMPLE_TIME);
        span_log_set_tag(v22bis_get_logging_state(a.m), "answer");
    }
    /* A Bell 212A caller is started by its application once it has heard
       the answer tone for 150 ms (the engine's B212_TONE_HEARD_MS); until then
       it is silent and deaf.  The answerer's tone starts after 75 ms. */
    int call_start = bell == 2 ? 75*RATE/1000 + d + 150*RATE/1000 : 0;

    int clear = 0;

    call_start = (call_start + FRAME - 1)/FRAME*FRAME;
    for (int t = 0; t < total; t += FRAME) {
        c.now = a.now = t;
        if (t < call_start) {
            memset(call_tx + t, 0, sizeof(int16_t)*FRAME);
            v22bis_tx(a.m, ans_tx + t, FRAME);
            v22bis_rx(a.m, t >= d ? call_tx + t - d : silence, FRAME);
            continue;
        }
        v22bis_tx(c.m, call_tx + t, FRAME);
        v22bis_tx(a.m, ans_tx + t, FRAME);
        v22bis_rx(a.m, t >= d ? call_tx + t - d : silence, FRAME);
        v22bis_rx(c.m, t >= d ? ans_tx + t - d : silence, FRAME);
        /* The engine's tone detector: two 20 ms frames without the tone. */
        if (bell == 2 && t >= d) {
            clear = tone_fraction(ans_tx + t - d, FRAME, 2225.0) < 0.5 ? clear + 1 : 0;
            if (clear == 2)
                v22bis_bell212a_answer_tone_ended(c.m);
        }
    }
    snprintf(what, sizeof(what), "both ends trained (call %.2f s, answer %.2f s)",
             c.trained_at/(double) RATE, a.trained_at/(double) RATE);
    check(c.trained_at >= 0 && a.trained_at >= 0, what);
    printf("  .... caller carrier %.1f Hz, answerer carrier %.1f Hz\n",
           v22bis_rx_carrier_frequency(c.m), v22bis_rx_carrier_frequency(a.m));
    snprintf(what, sizeof(what), "settled at 1200 (call %d, answer %d)",
             v22bis_get_current_bit_rate(c.m), v22bis_get_current_bit_rate(a.m));
    check(v22bis_get_current_bit_rate(c.m) == 1200 && v22bis_get_current_bit_rate(a.m) == 1200, what);
    snprintf(what, sizeof(what), "answer -> call: %ld bits, %ld errors", c.rx_bits, c.rx_errors);
    check(c.rx_bits > 1200*5 && c.rx_errors == 0, what);
    snprintf(what, sizeof(what), "call -> answer: %ld bits, %ld errors", a.rx_bits, a.rx_errors);
    check(a.rx_bits > 1200*5 && a.rx_errors == 0, what);
    if (bell) {
        double head = tone_fraction(ans_tx + 800, 800, 2225.0);
        double tail = tone_fraction(ans_tx + total - 800, 800, 2225.0);

        snprintf(what, sizeof(what), "answerer opens with 2225 Hz (%.3f of power) and has left it "
                 "in data mode (%.3f)", head, tail);
        check(head > 0.95 && tail < 0.2, what);
    }
    v22bis_free(c.m);
    v22bis_free(a.m);
}

int main(void)
{
    double lvl212, lvl22;

    if (getenv("BELL212A_TRACE"))
        setvbuf(stdout, NULL, _IONBF, 0);
    double f212 = answer_alone(V22BIS_GUARD_TONE_NONE | V22BIS_BELL212A_COMPATIBILITY_MODE, &lvl212);
    double f22 = answer_alone(V22BIS_GUARD_TONE_NONE, &lvl22);
    char what[160];

    printf("answerer alone\n");
    snprintf(what, sizeof(what), "Bell 212A: every 100 ms block is 2225 Hz (worst %.4f of power)", f212);
    check(f212 > 0.99, what);
    snprintf(what, sizeof(what), "plain V.22 unscrambled ones are not (worst %.4f)", f22);
    check(f22 < 0.5, what);
    /* At the configured -14 dBm0: SpanDSP's 0 dBm0 sine is 3.14 dB below
       full scale.  (Its 1200 bit/s USB1 measures about 3 dB hotter than the
       configured power; that calibration is SpanDSP's and is left alone.) */
    {
        double want = 32767.0/sqrt(2.0)*pow(10.0, (-14.0 - DBM0_MAX_SINE_POWER)/20.0);

        snprintf(what, sizeof(what), "tone at the configured -14 dBm0 (RMS %.0f, want %.0f; USB1 %.0f)",
                 lvl212, want, lvl22);
        check(fabs(20.0*log10(lvl212/want)) < 0.5, what);
    }

    /* bell: 0 plain V.22 both ends, 1 a 212A answerer and a plain V.22 or
       V.22bis caller, 2 Bell 212A both ends. */
    {
        static const int delays[] = {0, 100, 200, 400};

        for (size_t k = 0; k < sizeof(delays)/sizeof(delays[0]); k++) {
            char label[128];

            snprintf(label, sizeof(label), "baseline: plain V.22 both ends, %d ms each way", delays[k]);
            pair(1200, 0, delays[k], label);
            snprintf(label, sizeof(label), "Bell 212A both ends, %d ms each way", delays[k]);
            pair(1200, 2, delays[k], label);
        }
    }
    pair(1200, 1, 0, "Bell 212A answerer, plain V.22 caller, no delay");
    pair(2400, 1, 0, "Bell 212A answerer, V.22bis 2400 caller, no delay");

    printf(failures ? "bell212a_test: %d failure(s)\n" : "bell212a_test: all passed\n", failures);
    return failures != 0;
}
