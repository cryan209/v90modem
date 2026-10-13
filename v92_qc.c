/*
 * v92_qc.c -- V.92 Quick Connect, short Phase 1, digital modem (9.2.4)
 *
 * See v92_qc.h.  Clause references are to ITU-T V.92 (11/2000).
 */
#include "v92_qc.h"

#include <math.h>
#include <string.h>

#include <spandsp.h>

#include "v92_anspcm_decode.h"
#include "v92_short_phase1_decode.h"

#define TONEQ_HZ        980.0     /* 8.2.5 */
/* Block RMS below which a block is not a tone (about -38 dBm0).  TONEq is
 * sent at a normal signalling level, and this keeps line noise and the
 * residue of our own signal from being judged by their spectrum alone. */
#define TONEQ_MIN_RMS   100.0
/* Share of the block's energy the 980 Hz bin must hold.  A pure tone sits
 * near 1; the V.21 mark of a CM still on the line is also 980 Hz, but it is
 * 33 ms ones at most and the run required below is 50 ms. */
#define TONEQ_PURITY    0.70

const char *v92_qc_state_name(v92_qc_state_t s)
{
    switch (s) {
    case V92_QC_IDLE:    return "idle";
    case V92_QC_QCA1D:   return "QCA1d";
    case V92_QC_GAP:     return "gap";
    case V92_QC_QTS:     return "QTS";
    case V92_QC_QTS_BAR: return "QTS-bar";
    case V92_QC_ANSPCM:  return "ANSpcm";
    case V92_QC_SILENCE: return "silence";
    case V92_QC_PHASE2:  return "phase2";
    case V92_QC_V8:      return "v8";
    case V92_QC_ANSAM:   return "ansam";
    case V92_QC_WATCH:   return "watch";
    }
    return "?";
}

void v92_qc_init(v92_qc_t *q)
{
    memset(q, 0, sizeof(*q));
    q->state = V92_QC_IDLE;
    q->qts_start = -1;
}

void v92_qc_free(v92_qc_t *q)
{
    if (q->fsk) {
        fsk_tx_free((fsk_tx_state_t *) q->fsk);
        q->fsk = NULL;
    }
}

static int qc_get_bit(void *user_data)
{
    v92_qc_t *q = user_data;

    if (q->qca_bit >= q->qca_len)
        return SIG_STATUS_END_OF_DATA;
    return q->qca_bits[q->qca_bit++];
}

static void toneq_reset(v92_qc_t *q)
{
    q->blk_n = 0;
    q->blk_e = 0.0;
    q->blk_s1 = q->blk_s2 = 0.0;
    q->toneq_run = 0;
}

bool v92_qc_start(v92_qc_t *q, v91_law_t law, int uqts_ucode,
                  int anspcm_level, bool lapm)
{
    v92_qc_free(q);
    v92_qc_init(q);
    if (uqts_ucode < 1 || uqts_ucode > 127 || anspcm_level < 0 || anspcm_level > 3)
        return false;
    q->law = law;
    q->uqts_ucode = uqts_ucode;
    q->anspcm_level = anspcm_level;
    q->lapm = lapm;
    q->qca_len = v92_build_short_phase1_sequence(true, true, lapm, anspcm_level,
                                                 q->qca_bits, (int) sizeof(q->qca_bits));
    if (q->qca_len != 70)
        return false;
    q->qca_bit = 0;
    q->fsk = fsk_tx_init(NULL, &preset_fsk_specs[FSK_V21CH2], qc_get_bit, q);
    if (!q->fsk)
        return false;
    toneq_reset(q);
    q->state = V92_QC_QCA1D;
    return true;
}

bool v92_qc_start_analogue_answer(v92_qc_t *q, v91_law_t law, bool lapm)
{
    v92_qc_free(q);
    v92_qc_init(q);
    q->law = law;
    q->lapm = lapm;
    q->ana_answer = true;
    /* QCA1a, Table 4: analogue, QCA; W0XYZ is the UQTS of a digital caller's QTS
       (an analogue caller ignores it): 0101, UQTS 70. */
    q->qca_len = v92_build_short_phase1_sequence(false, true, lapm, 5,
                                                 q->qca_bits, (int) sizeof(q->qca_bits));
    if (q->qca_len != 70)
        return false;
    q->fsk = fsk_tx_init(NULL, &preset_fsk_specs[FSK_V21CH2], qc_get_bit, q);
    if (!q->fsk)
        return false;
    toneq_reset(q);
    q->state = V92_QC_QCA1D;
    return true;
}

bool v92_qc_watch_start(v92_qc_t *q, v91_law_t law)
{
    v92_qc_free(q);
    v92_qc_init(q);
    q->law = law;
    toneq_reset(q);
    q->state = V92_QC_WATCH;
    return true;
}

bool v92_qc_start_after_qca(v92_qc_t *q, v91_law_t law, int uqts_ucode,
                            int anspcm_level, bool lapm, unsigned silent)
{
    if (!v92_qc_start(q, law, uqts_ucode, anspcm_level, lapm))
        return false;
    /* No QCA1d of our own to send. */
    if (q->fsk) {
        fsk_tx_free((fsk_tx_state_t *) q->fsk);
        q->fsk = NULL;
    }
    if (silent > V92_QC_SILENCE_SYMBOLS)
        silent = V92_QC_SILENCE_SYMBOLS;
    q->state = V92_QC_GAP;
    q->pos = silent;
    q->since_qca = silent;
    return true;
}

static uint8_t qc_silence(const v92_qc_t *q)
{
    return v91_ucode_to_codeword(q->law, 0, true);
}

/* One QTS (or QTS\) codeword: {+V +0 +V -V -0 -V}, or the same negated. */
static uint8_t qc_qts_symbol(const v92_qc_t *q, long pos, bool bar)
{
    static const bool zero[6] = { false, true, false, false, true, false };
    static const bool positive[6] = { true, true, true, false, false, false };
    int i = (int) (pos % 6);
    bool pos_sign = bar ? !positive[i] : positive[i];

    return v91_ucode_to_codeword(q->law, zero[i] ? 0 : q->uqts_ucode, pos_sign);
}

int v92_qc_tx(v92_qc_t *q, uint8_t *cw, int n)
{
    int done = 0;

    while (done < n) {
        switch (q->state) {
        case V92_QC_IDLE:
        case V92_QC_PHASE2:
        case V92_QC_V8:
        case V92_QC_ANSAM:
        case V92_QC_WATCH:
            return done;

        case V92_QC_QCA1D: {
            int16_t amp[160];
            int want = n - done;
            int got;

            if (want > (int) (sizeof(amp) / sizeof(amp[0])))
                want = (int) (sizeof(amp) / sizeof(amp[0]));
            got = fsk_tx((fsk_tx_state_t *) q->fsk, amp, want);
            for (int i = 0; i < got; i++)
                cw[done + i] = v91_linear_to_codeword(q->law, amp[i]);
            done += got;
            q->total += got;
            if (got < want) {
                /* The last bit has gone out; QCA1d is over. */
                if (q->ana_answer) {
                    q->state = V92_QC_ANSAM;
                    return done;
                }
                q->state = V92_QC_GAP;
                q->pos = 0;
                q->since_qca = 0;
            }
            break;
        }

        case V92_QC_GAP:
        case V92_QC_SILENCE: {
            long left = V92_QC_SILENCE_SYMBOLS - q->pos;
            int run = (left < n - done) ? (int) left : n - done;

            memset(cw + done, qc_silence(q), (size_t) run);
            done += run;
            q->pos += run;
            q->total += run;
            if (q->state == V92_QC_GAP)
                q->since_qca += run;
            if (q->pos >= V92_QC_SILENCE_SYMBOLS) {
                if (q->state == V92_QC_GAP) {
                    q->state = V92_QC_QTS;
                    q->pos = 0;
                    q->qts_start = q->total;
                } else {
                    q->state = V92_QC_PHASE2;
                    return done;
                }
            }
            break;
        }

        case V92_QC_QTS:
        case V92_QC_QTS_BAR: {
            bool bar = (q->state == V92_QC_QTS_BAR);
            long len = bar ? V92_QC_QTS_BAR_SYMBOLS : V92_QC_QTS_SYMBOLS;

            while (done < n && q->pos < len) {
                cw[done++] = qc_qts_symbol(q, q->pos, bar);
                q->pos++;
                q->total++;
                q->since_qca++;
            }
            if (q->pos >= len) {
                q->pos = 0;
                q->state = bar ? V92_QC_ANSPCM : V92_QC_QTS_BAR;
            }
            break;
        }

        case V92_QC_ANSPCM:
            while (done < n) {
                cw[done++] = v92_anspcm_codeword(q->law, q->anspcm_level, q->anspcm_pos++);
                q->total++;
                q->since_qca++;
                if (q->since_qca >= V92_QC_TONEQ_TIMEOUT) {
                    /* 9.2.4.3: no TONEq within 2 s of QCA1d. */
                    q->state = V92_QC_V8;
                    return done;
                }
            }
            break;
        }
    }
    return done;
}

static void toneq_block_done(v92_qc_t *q)
{
    double c = 2.0 * cos(2.0 * M_PI * TONEQ_HZ / 8000.0);
    /* |X|^2 of the Goertzel recursion; for a pure tone of amplitude A over N
     * samples it is (A N / 2)^2, so the tone's energy is 2|X|^2 / N. */
    double mag2 = q->blk_s1 * q->blk_s1 + q->blk_s2 * q->blk_s2
                - c * q->blk_s1 * q->blk_s2;
    double tone_e = 2.0 * mag2 / V92_QC_TONEQ_BLOCK;
    double rms = sqrt(q->blk_e / V92_QC_TONEQ_BLOCK);

    if (rms >= TONEQ_MIN_RMS && q->blk_e > 0.0 && tone_e / q->blk_e >= TONEQ_PURITY) {
        if (q->toneq_run == 0)
            q->toneq_first = q->rx_total - V92_QC_TONEQ_BLOCK;
        q->toneq_run++;
    } else {
        q->toneq_run = 0;
    }
    q->blk_n = 0;
    q->blk_e = 0.0;
    q->blk_s1 = q->blk_s2 = 0.0;
}

void v92_qc_rx(v92_qc_t *q, const uint8_t *cw, int n)
{
    double c = 2.0 * cos(2.0 * M_PI * TONEQ_HZ / 8000.0);

    if (q->state == V92_QC_IDLE)
        return;
    for (int i = 0; i < n; i++) {
        double x = (double) v91_codeword_to_linear(q->law, cw[i]);
        double s0 = x + c * q->blk_s1 - q->blk_s2;

        q->blk_s2 = q->blk_s1;
        q->blk_s1 = s0;
        q->blk_e += x * x;
        q->rx_total++;
        if (++q->blk_n == V92_QC_TONEQ_BLOCK)
            toneq_block_done(q);
        if (q->toneq_run >= V92_QC_TONEQ_BLOCKS
            && (q->state == V92_QC_ANSPCM || q->state == V92_QC_WATCH)) {
            /* 9.2.4.3: TONEq detected -- stop ANSpcm, 75 +/- 5 ms of silence. */
            q->toneq_seen = true;
            q->state = V92_QC_SILENCE;
            q->pos = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Analogue caller                                                     */
/* ------------------------------------------------------------------ */

#define ANSPCM_HZ      2100.0
#define TONEQ_TX_PEAK  2500.0     /* about -13 dBm0 */

const char *v92_qca_state_name(v92_qca_state_t s)
{
    switch (s) {
    case V92_QCA_IDLE:     return "idle";
    case V92_QCA_WAIT_PCM: return "wait-ANSpcm";
    case V92_QCA_TONEQ:    return "TONEq";
    case V92_QCA_SILENCE:  return "silence";
    case V92_QCA_PHASE2:   return "phase2";
    case V92_QCA_V8:       return "v8";
    }
    return "?";
}

void v92_qca_init(v92_qca_t *q)
{
    memset(q, 0, sizeof(*q));
    q->state = V92_QCA_IDLE;
}

bool v92_qca_start(v92_qca_t *q, v91_law_t law)
{
    return v92_qca_start_ansam(q, law, V92_QCA_PCM_BLOCKS);
}

bool v92_qca_start_ansam(v92_qca_t *q, v91_law_t law, int blocks)
{
    return v92_qca_start_limit(q, law, blocks, V92_QCA_WAIT_LIMIT);
}

bool v92_qca_start_limit(v92_qca_t *q, v91_law_t law, int blocks, long limit)
{
    v92_qca_init(q);
    q->wait_limit = limit;
    q->pcm_blocks_needed = blocks;
    q->law = law;
    q->state = V92_QCA_WAIT_PCM;
    return true;
}

int v92_qca_tx(v92_qca_t *q, uint8_t *cw, int n)
{
    int done = 0;
    uint8_t sil = v91_ucode_to_codeword(q->law, 0, true);

    while (done < n) {
        switch (q->state) {
        case V92_QCA_IDLE:
        case V92_QCA_PHASE2:
        case V92_QCA_V8:
            return done;
        case V92_QCA_WAIT_PCM:
            cw[done++] = sil;
            q->waited++;
            if (q->waited >= q->wait_limit) {
                q->state = V92_QCA_V8;
                return done;
            }
            break;
        case V92_QCA_TONEQ:
            cw[done++] = v91_linear_to_codeword(q->law,
                             (int16_t) lrint(TONEQ_TX_PEAK * sin(q->tone_phase)));
            q->tone_phase += 2.0 * M_PI * TONEQ_HZ / 8000.0;
            q->tonq_symbols++;
            break;
        case V92_QCA_SILENCE:
            cw[done++] = sil;
            if (++q->pos >= V92_QC_SILENCE_SYMBOLS) {
                q->state = V92_QCA_PHASE2;
                return done;
            }
            break;
        }
    }
    return done;
}

/* One 10 ms block: is it ANSpcm (a pure 2100 Hz tone at signalling level)? */
static void qca_block_done(v92_qca_t *q)
{
    double c = 2.0 * cos(2.0 * M_PI * ANSPCM_HZ / 8000.0);
    double mag2 = q->blk_s1 * q->blk_s1 + q->blk_s2 * q->blk_s2 - c * q->blk_s1 * q->blk_s2;
    double tone_e = 2.0 * mag2 / V92_QC_TONEQ_BLOCK;
    double rms = sqrt(q->blk_e / V92_QC_TONEQ_BLOCK);
    bool pcm = rms >= TONEQ_MIN_RMS && q->blk_e > 0.0 && tone_e / q->blk_e >= TONEQ_PURITY;

    if (pcm) {
        q->pcm_run++;
        q->pcm_miss = 0;
    } else if (++q->pcm_miss >= V92_QCA_PCM_MISS) {
        /* ANSpcm's phase reversals (every 3612 symbols) smear a block or two;
         * three in a row is the signal gone. */
        if (q->state == V92_QCA_TONEQ) {
            /* 9.2.1.3: ANSpcm no longer detected -- end TONEq, 75 +/- 5 ms. */
            q->state = V92_QCA_SILENCE;
            q->pos = 0;
        }
        q->pcm_run = 0;
    }
    if (q->state == V92_QCA_WAIT_PCM && q->pcm_run >= q->pcm_blocks_needed) {
        /* 9.2.1.3: ANSpcm detected for 1 s -- TONEq. */
        q->state = V92_QCA_TONEQ;
        q->tone_phase = 0.0;
    }
    q->blk_n = 0;
    q->blk_e = 0.0;
    q->blk_s1 = q->blk_s2 = 0.0;
}

void v92_qca_rx(v92_qca_t *q, const uint8_t *cw, int n)
{
    double c = 2.0 * cos(2.0 * M_PI * ANSPCM_HZ / 8000.0);

    if (q->state == V92_QCA_IDLE)
        return;
    for (int i = 0; i < n; i++) {
        double x = (double) v91_codeword_to_linear(q->law, cw[i]);
        double s0 = x + c * q->blk_s1 - q->blk_s2;

        q->blk_s2 = q->blk_s1;
        q->blk_s1 = s0;
        q->blk_e += x * x;
        if (++q->blk_n == V92_QC_TONEQ_BLOCK)
            qca_block_done(q);
    }
}
