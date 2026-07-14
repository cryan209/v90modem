/*
 * Unit tests for the V.92 clause 9.2 procedure evaluator in
 * phase12_decode.c (p12_eval_v92_clause92_procedure and
 * p12_reconcile_v92_proc are static, so the file is included directly;
 * window_energy/tone_energy_ratio normally come from vpcm_decode.c).
 *
 * The tests fabricate Phase 1 timelines and call_init observations and
 * assert the evaluator's terminal outcome, retry anchoring, and Phase 2
 * handoff.  The short-phase2 (successful quick connect) outcome has no
 * line-capture truth data, so this is its only regression coverage.
 */

#include "phase12_decode.c"

#include <assert.h>

double window_energy(const int16_t *samples, int len)
{
    double e = 0.0;

    for (int i = 0; i < len; i++)
        e += (double) samples[i] * samples[i];
    return e;
}

double tone_energy_ratio(const int16_t *samples, int len,
                         int sample_rate, double freq_hz, double energy)
{
    (void) samples;
    (void) len;
    (void) sample_rate;
    (void) freq_hz;
    (void) energy;
    return 0.0;
}

bool call_log_append(call_log_t *log,
                     int sample_offset,
                     int duration_samples,
                     const char *protocol,
                     const char *summary,
                     const char *detail)
{
    (void) log;
    (void) sample_offset;
    (void) duration_samples;
    (void) protocol;
    (void) summary;
    (void) detail;
    return true;
}

#define SR 8000

static int ms_to_samples(int ms)
{
    return (SR * ms) / 1000;
}

static void add_event(phase12_result_t *r, const char *label, int ms, int dur_ms)
{
    p12_phase1_event_t *ev;

    assert(r->phase1_event_count < P12_MAX_PHASE1_EVENTS);
    ev = &r->phase1_events[r->phase1_event_count++];
    memset(ev, 0, sizeof(*ev));
    ev->seen = true;
    ev->sample_offset = ms_to_samples(ms);
    ev->duration_samples = ms_to_samples(dur_ms);
    snprintf(ev->label, sizeof(ev->label), "%s", label);
}

static void add_ansam(phase12_result_t *r, int ms, int dur_ms)
{
    r->answer_tone.detected = true;
    r->answer_tone.type = P12_TONE_ANSAM_PR;
    r->answer_tone.start_sample = ms_to_samples(ms);
    r->answer_tone.duration_samples = ms_to_samples(dur_ms);
}

/* Figure 3 base: ANSam from the digital answerer, then QC1a + CM from the
 * analogue caller, QCA1d answer, and the digital-side QTS/ANSpcm chain. */
static void setup_figure3_chain(phase12_result_t *r)
{
    phase12_result_init(r);
    add_ansam(r, 4000, 2000);
    add_event(r, "QC1a", 5400, 0);
    add_event(r, "CM", 5620, 100);
    add_event(r, "QCA1d", 6000, 0);
    r->call_init.v92_qts_seen = true;
    r->call_init.v92_qts_sample = ms_to_samples(6300);
    r->call_init.v92_qts_reps = 128;
    r->call_init.v92_qts_bar_reps = 8;
    r->call_init.v92_anspcm_seen = true;
    r->call_init.v92_anspcm_sample = ms_to_samples(6400);
    r->call_init.v92_anspcm_duration_symbols = ms_to_samples(1000);
    r->call_init.v92_anspcm_level = 15;
}

static const p12_v92_proc_step_t *find_step(const p12_v92_proc_result_t *proc,
                                            const char *clause,
                                            const char *signal)
{
    for (int i = 0; i < proc->step_count; i++) {
        const p12_v92_proc_step_t *s = &proc->steps[i];

        if (strcmp(s->clause, clause) == 0 && strcmp(s->signal, signal) == 0)
            return s;
    }
    return NULL;
}

/* Successful quick connect: TONEq answers ANSpcm, Phase 2 handoff follows
 * 75 ms after TONEq end (9.2.1.3/9.2.4.1). */
static void test_short_phase2_success(void)
{
    phase12_result_t r;
    int toneq_end_ms = 6600 + 100;

    setup_figure3_chain(&r);
    r.call_init.v92_toneq_seen = true;
    r.call_init.v92_toneq_sample = ms_to_samples(6600);
    r.call_init.v92_toneq_duration_samples = ms_to_samples(100);

    p12_eval_v92_clause92_procedure(&r, SR);
    assert(r.v92_proc.evaluated);
    assert(r.v92_proc.family == 1);
    assert(r.v92_proc.figure == P12_V92_PROC_FIGURE_3);
    assert(r.v92_proc.outcome == P12_V92_PROC_OUTCOME_SHORT_PHASE2);
    assert(r.v92_proc.missing_count == 0);
    assert(r.v92_proc.late_count == 0);
    assert(r.v92_proc.phase2_handoff_sample
           == ms_to_samples(toneq_end_ms) + (SR * P12_V92_SHORT_P1_TO_PHASE2_MS) / 1000);

    p12_reconcile_v92_proc(&r, SR);
    assert(r.call_init.v92_phase2_handoff_known);
    assert(r.call_init.v92_phase2_handoff_sample == r.v92_proc.phase2_handoff_sample);

    printf("PASS: short-phase2 success (figure 3, TONEq answered)\n");
}

/* TONEq missing: the V.8 retry search must anchor at the observed ANSpcm
 * end, not the fixed qca_end + 1500 ms guard, so a retry CM that starts
 * shortly after ANSpcm ends is found (the USR-Message-V92QC regression). */
static void test_v8_retry_anchored_at_anspcm_end(void)
{
    phase12_result_t r;
    const p12_v92_proc_step_t *retry;

    setup_figure3_chain(&r);
    /* ANSpcm ends at 7400 ms; qca_end + 1500 would be 7734 ms.  The retry
     * CM at 7500 ms is only found with the ANSpcm anchor. */
    add_event(&r, "CM", 7500, 130);
    add_event(&r, "CM", 9000, 130);

    p12_eval_v92_clause92_procedure(&r, SR);
    assert(r.v92_proc.outcome == P12_V92_PROC_OUTCOME_V8_FALLBACK);
    assert(r.v92_proc.phase2_handoff_sample == -1);
    retry = find_step(&r.v92_proc, "9.2.4.3", "CM");
    assert(retry != NULL);
    assert(retry->status == P12_V92_PROC_STEP_OBSERVED);
    assert(retry->sample_offset == ms_to_samples(7500));

    printf("PASS: v8-fallback retry anchored at ANSpcm end\n");
}

/* A CM while ANSpcm is still transmitting must NOT be taken as the retry. */
static void test_cm_during_anspcm_not_retry(void)
{
    phase12_result_t r;
    const p12_v92_proc_step_t *retry;

    setup_figure3_chain(&r);
    add_event(&r, "CM", 7000, 130);      /* inside ANSpcm (6400..7400 ms) */
    add_event(&r, "CM", 7600, 130);

    p12_eval_v92_clause92_procedure(&r, SR);
    assert(r.v92_proc.outcome == P12_V92_PROC_OUTCOME_V8_FALLBACK);
    retry = find_step(&r.v92_proc, "9.2.4.3", "CM");
    assert(retry != NULL);
    assert(retry->sample_offset == ms_to_samples(7600));

    printf("PASS: CM during ANSpcm not taken as retry\n");
}

/* Analog-side channel of a stereo split: no local ANSpcm, but the stereo
 * partner hint carries the digital side's ANSpcm end, so the retry search
 * anchors at the same place as the digital channel. */
static void test_partner_anspcm_end_anchor(void)
{
    phase12_result_t r;
    const p12_v92_proc_step_t *retry;

    phase12_result_init(&r);
    add_ansam(&r, 4000, 2000);
    add_event(&r, "QC1a", 5400, 0);
    add_event(&r, "CM", 5620, 100);
    /* qca_end from partner hint = 6000 + 234 ms; + 1500 guard = 7734 ms */
    add_event(&r, "CM", 7500, 130);
    r.stereo_short_p1_hint_valid = true;
    r.stereo_short_p1_expected_form = P12_SHORT_P1_FORM_ANALOG;
    r.stereo_short_p1_partner_family = 1;
    r.stereo_short_p1_partner_qca = true;
    r.stereo_short_p1_partner_sample = ms_to_samples(6000);
    r.stereo_short_p1_partner_anspcm_end = ms_to_samples(7400);

    p12_eval_v92_clause92_procedure(&r, SR);
    assert(r.v92_proc.outcome == P12_V92_PROC_OUTCOME_V8_FALLBACK);
    retry = find_step(&r.v92_proc, "9.2.4.3", "CM");
    assert(retry != NULL);
    assert(retry->sample_offset == ms_to_samples(7500));

    printf("PASS: partner-hint ANSpcm end anchors analog-channel retry\n");
}

/* Without the partner ANSpcm hint the fixed guard applies and the same
 * 7500 ms CM is out of reach: the story never resolves. */
static void test_incomplete_without_retry(void)
{
    phase12_result_t r;

    phase12_result_init(&r);
    add_ansam(&r, 4000, 2000);
    add_event(&r, "QC1a", 5400, 0);
    add_event(&r, "CM", 5620, 100);
    add_event(&r, "CM", 7500, 130);
    r.stereo_short_p1_hint_valid = true;
    r.stereo_short_p1_expected_form = P12_SHORT_P1_FORM_ANALOG;
    r.stereo_short_p1_partner_family = 1;
    r.stereo_short_p1_partner_qca = true;
    r.stereo_short_p1_partner_sample = ms_to_samples(6000);

    p12_eval_v92_clause92_procedure(&r, SR);
    assert(r.v92_proc.outcome == P12_V92_PROC_OUTCOME_INCOMPLETE);

    printf("PASS: incomplete when no retry reachable\n");
}

/* Figures 7/8: both modems analogue; TONEq answers the second ANSam and
 * the call proceeds to V.34 Phase 2. */
static void test_both_analog_v34_phase2(void)
{
    phase12_result_t r;

    phase12_result_init(&r);
    add_ansam(&r, 4000, 2000);
    add_event(&r, "QC1a", 5400, 0);
    add_event(&r, "CM", 5620, 100);
    add_event(&r, "QCA1a", 6000, 0);
    r.call_init.v92_toneq_seen = true;
    r.call_init.v92_toneq_sample = ms_to_samples(6600);
    r.call_init.v92_toneq_duration_samples = ms_to_samples(100);

    p12_eval_v92_clause92_procedure(&r, SR);
    assert(r.v92_proc.figure == P12_V92_PROC_FIGURE_7);
    assert(r.v92_proc.outcome == P12_V92_PROC_OUTCOME_V34_PHASE2);

    printf("PASS: both-analog TONEq path resolves to v34-phase2\n");
}

/* A fallback outcome must clear stale chain/handoff summary flags. */
static void test_reconcile_clears_stale_flags(void)
{
    phase12_result_t r;

    setup_figure3_chain(&r);
    add_event(&r, "CM", 7500, 130);
    r.call_init.v92_digital_chain_valid = true;
    r.call_init.v92_phase2_handoff_known = true;
    r.call_init.v92_phase2_handoff_sample = ms_to_samples(7000);

    p12_eval_v92_clause92_procedure(&r, SR);
    assert(r.v92_proc.outcome == P12_V92_PROC_OUTCOME_V8_FALLBACK);
    p12_reconcile_v92_proc(&r, SR);
    assert(!r.call_init.v92_digital_chain_valid);
    assert(!r.call_init.v92_phase2_handoff_known);
    assert(r.call_init.v92_phase2_handoff_sample == -1);

    printf("PASS: reconcile clears stale chain flags on fallback\n");
}

/* ------------------------------------------------------------------ */
/* Waveform QTS/QTS\ detector (v92_short_phase1_decode.c)              */
/* ------------------------------------------------------------------ */

/*
 * Synthesize silence + QTS (128 reps) [+ QTS\ (8 reps)] + a 2099.7 Hz
 * ANSpcm-like tone, passed through a short FIR to mimic the channel
 * smearing that defeats sample-exact boundary tests on line captures.
 */
static int synth_qts_signal(int16_t *buf, int cap, bool with_bar, int *qts_start)
{
    static const double fir[4] = { 0.7, 0.25, -0.1, 0.05 };
    enum { LEAD = 400, V = 2000, ANS_SAMPLES = 1200 };
    double clean[LEAD + 136 * 6 + ANS_SAMPLES];
    const int pattern[6] = { V, 0, V, -V, 0, -V };
    int total = 0;

    memset(clean, 0, sizeof(clean));
    total = LEAD;
    for (int rep = 0; rep < 128; rep++)
        for (int k = 0; k < 6; k++)
            clean[total++] = pattern[k];
    if (with_bar)
        for (int rep = 0; rep < 8; rep++)
            for (int k = 0; k < 6; k++)
                clean[total++] = -pattern[k];
    for (int i = 0; i < ANS_SAMPLES; i++)
        clean[total++] = 2500.0 * sin(2.0 * M_PI * 2099.7 * i / SR);

    assert(total <= cap && total <= (int) (sizeof(clean) / sizeof(clean[0])));
    for (int i = 0; i < total; i++) {
        double acc = 0.0;

        for (int t = 0; t < 4; t++)
            if (i - t >= 0)
                acc += fir[t] * clean[i - t];
        buf[i] = (int16_t) lround(acc);
    }
    *qts_start = LEAD;
    return total;
}

static void test_qts_waveform_split(void)
{
    int16_t buf[4096];
    v92_qts_hit_t hit;
    int qts_start;
    int total = synth_qts_signal(buf, 4096, true, &qts_start);

    assert(v92_detect_qts_waveform(buf, total, 0, total, &hit));
    assert(hit.seen);
    /* the probe locks a few samples early: leading silence is trivially
     * antisymmetric, so the run start precedes the physical QTS onset */
    assert(hit.start_sample >= qts_start - 36 && hit.start_sample <= qts_start + 6);
    /* nominal 128 + 8, plus up to a few leading silence/smear periods on
     * the QTS side; the boundary block may migrate one period */
    assert(hit.qts_reps >= 126 && hit.qts_reps <= 134);
    assert(hit.qts_bar_reps >= 7 && hit.qts_bar_reps <= 9);

    printf("PASS: QTS waveform split (128 QTS + 8 QTS\\ through FIR channel)\n");
}

static void test_qts_waveform_no_bar(void)
{
    int16_t buf[4096];
    v92_qts_hit_t hit;
    int qts_start;
    int total = synth_qts_signal(buf, 4096, false, &qts_start);

    assert(v92_detect_qts_waveform(buf, total, 0, total, &hit));
    assert(hit.seen);
    assert(hit.qts_reps >= 126 && hit.qts_reps <= 134);
    assert(hit.qts_bar_reps == 0);

    printf("PASS: QTS waveform without QTS\\ reports qts_bar=0\n");
}

int main(void)
{
    test_qts_waveform_split();
    test_qts_waveform_no_bar();
    test_short_phase2_success();
    test_v8_retry_anchored_at_anspcm_end();
    test_cm_during_anspcm_not_retry();
    test_partner_anspcm_end_anchor();
    test_incomplete_without_retry();
    test_both_analog_v34_phase2();
    test_reconcile_clears_stale_flags();
    printf("All V.92 clause 9.2 evaluator tests passed\n");
    return 0;
}
