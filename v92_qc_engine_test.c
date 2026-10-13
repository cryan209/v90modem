/*
 * v92_qc_engine_test.c -- the whole answering engine against a scripted
 * ANALOGUE modem doing V.92 Quick Connect (9.2.1.1), over a byte-exact G.711
 * bearer.
 *
 * The engine is a v90_engine_peer child, clocked one 20 ms frame at a time
 * and configured over its PTY like any DTE would.  This program is the
 * analogue modem: it waits for ANSam for 1 s, sends QC1a, goes silent when
 * the engine stops ANSam, and measures what comes back against the clause:
 *
 *   QCA1d (a strict Table 5 sequence on V.21(H)), 75 ms of silence, QTS
 *   with the UQTS the QC1a asked for, QTS\, ANSpcm codeword-exact at the
 *   announced level; then, once it has heard ANSpcm for 1 s, TONEq -- after
 *   which ANSpcm must stop promptly and 75 ms of silence must precede
 *   Phase 2 (9.2.4.3).
 *
 * Cases: the happy path in both laws; no TONEq (ANSpcm for 2 s, then
 * ANSam and V.8, 9.2.4.3); and +PQC=3, where the same QC1a must be ignored.
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include <spandsp.h>

#include "test_tmp.h"
#include "v91.h"
#include "v92_anspcm_decode.h"
#include "v92_short_phase1_decode.h"

#define FRAME 160
#define BLOCK 80
#define MAX_SAMPLES (8000 * 24)

static int failures;

#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("    FAIL: "); printf(__VA_ARGS__); printf("\n"); failures++; } \
} while (0)

typedef struct {
    pid_t pid;
    int to_fd, from_fd, pty_fd;
    char pty_path[256], log_path[256];
} engine_t;

static int read_full(int fd, void *buf, size_t len)
{
    size_t got = 0;

    while (got < len) {
        ssize_t n = read(fd, (char *) buf + got, len - got);

        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return -1;
        got += (size_t) n;
    }
    return 0;
}

static int write_full(int fd, const void *buf, size_t len)
{
    size_t done = 0;

    while (done < len) {
        ssize_t n = write(fd, (const char *) buf + done, len - done);

        if (n < 0 && errno == EINTR)
            continue;
        if (n <= 0)
            return -1;
        done += (size_t) n;
    }
    return 0;
}

static int spawn_engine(engine_t *e, int alaw)
{
    int in[2], out[2];

    if (pipe(in) || pipe(out))
        return -1;
    e->pid = fork();
    if (e->pid < 0)
        return -1;
    if (e->pid == 0) {
        int log = open(e->log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        const char *args[5];
        int n = 0;

        dup2(in[0], STDIN_FILENO);
        dup2(out[1], STDOUT_FILENO);
        if (log >= 0)
            dup2(log, STDERR_FILENO);
        close(in[1]);
        close(out[0]);
        setenv("ME_MEDIA_CLOCK", "1", 1);
        setenv("ME_MODE", "v92", 1);
        args[n++] = "./v90_engine_peer";
        args[n++] = e->pty_path;
        if (alaw)
            args[n++] = "--alaw";
        args[n] = NULL;
        execv(args[0], (char *const *) args);
        perror("execv v90_engine_peer");
        _exit(127);
    }
    close(in[0]);
    close(out[1]);
    e->to_fd = in[1];
    e->from_fd = out[0];
    for (int i = 0; i < 200; i++) {
        e->pty_fd = open(e->pty_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (e->pty_fd >= 0)
            return 0;
        usleep(10000);
    }
    return -1;
}

static int dte_cmd(engine_t *e, const char *cmd)
{
    char line[128], reply[512];
    size_t len = 0;

    snprintf(line, sizeof(line), "%s\r", cmd);
    write_full(e->pty_fd, line, strlen(line));
    reply[0] = '\0';
    for (int w = 0; w < 200 && !strstr(reply, "OK") && !strstr(reply, "ERROR"); w++) {
        ssize_t n;

        usleep(10000);
        n = read(e->pty_fd, reply + len, sizeof(reply) - len - 1);
        if (n > 0) {
            len += (size_t) n;
            reply[len] = '\0';
        }
    }
    return strstr(reply, "OK") != NULL;
}

static char *slurp(const char *path)
{
    FILE *f = fopen(path, "r");
    char *buf;
    long n;

    if (!f)
        return strdup("");
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = malloc((size_t) n + 1);
    n = (long) fread(buf, 1, (size_t) n, f);
    buf[n] = '\0';
    fclose(f);
    return buf;
}

/* ---- the scripted analogue modem ---- */

typedef struct { const uint8_t *bits; int n, i; } bitsrc_t;

static int bit_source(void *user)
{
    bitsrc_t *b = user;

    /* QC1a, then the line idles at mark: the CM that follows in a real
     * modem is not needed to exercise the response. */
    return b->i < b->n ? b->bits[b->i++] : 1;
}

typedef struct { uint8_t bits[800]; int n; } bitbuf_t;

static void put_bit(void *user, int bit)
{
    bitbuf_t *b = user;

    if (bit >= 0 && b->n < (int) sizeof(b->bits))
        b->bits[b->n++] = (uint8_t) bit;
}

/* Goertzel power fraction of one block at `hz`. */
static double purity(const int16_t *x, int n, double hz, double *rms)
{
    double c = 2.0 * cos(2.0 * M_PI * hz / 8000.0), s1 = 0, s2 = 0, e = 0;

    for (int i = 0; i < n; i++) {
        double s0 = x[i] + c * s1 - s2;

        s2 = s1;
        s1 = s0;
        e += (double) x[i] * x[i];
    }
    *rms = sqrt(e / n);
    if (e <= 0)
        return 0;
    return 2.0 * (s1 * s1 + s2 * s2 - c * s1 * s2) / (n * e);
}

typedef struct {
    int alaw;
    int pqc;
    int send_toneq;
    int seconds;
} qcase_t;

static void run_case(const char *name, const qcase_t *k)
{
    static uint8_t eng[MAX_SAMPLES];   /* what the engine transmitted */
    static uint8_t ana[MAX_SAMPLES];   /* what we transmitted */
    engine_t e = { 0 };
    v91_law_t law = k->alaw ? V91_LAW_ALAW : V91_LAW_ULAW;
    const uint8_t sil = v91_ucode_to_codeword(law, 0, true);
    uint8_t qc_bits[64];
    int qc_len = v92_build_short_phase1_sequence(false, false, true, 5 /* WXYZ=0101: UQTS 70 */,
                                                 qc_bits, sizeof(qc_bits));
    bitsrc_t src = { qc_bits, qc_len, 0 };
    fsk_tx_state_t *fsk = NULL;
    int total = 0, ansam_blocks = 0, ansam_t0 = -1;
    int qc_start = -1, qc_stop = -1, ansam_gone_blocks = 0;
    int saw_gap = 0, pcm_blocks = 0, pcm_miss = 0;
    int toneq_start = -1, toneq_end = -1;
    double ph = 0;
    char *log;
    char cmd[32];

    printf("  case: %s\n", name);
    snprintf(e.pty_path, sizeof(e.pty_path), "%s", test_tmp("qc_engine_pty"));
    snprintf(e.log_path, sizeof(e.log_path), "%s", test_tmp("qc_engine.log"));
    if (spawn_engine(&e, k->alaw) != 0) {
        printf("    FAIL: could not start the engine\n");
        failures++;
        return;
    }
    snprintf(cmd, sizeof(cmd), "AT+PQC=%d", k->pqc);
    CHECK(dte_cmd(&e, cmd), "%s did not answer OK", cmd);

    memset(ana, sil, sizeof(ana));
    for (; total + FRAME <= k->seconds * 8000 && total + FRAME <= MAX_SAMPLES; total += FRAME) {
        uint8_t header[2] = { FRAME & 0xFF, FRAME >> 8 };
        uint8_t *out = ana + total;
        int16_t lin[FRAME];

        /* This frame of ours: QC1a once ANSam has run for a second, silence
         * once the engine has stopped ANSam, TONEq once ANSpcm has been
         * heard for a second. */
        if (qc_start < 0 && ansam_blocks >= 100) {
            qc_start = total;
            fsk = fsk_tx_init(NULL, &preset_fsk_specs[FSK_V21CH1], bit_source, &src);
        }
        if (qc_start >= 0 && qc_stop < 0) {
            fsk_tx(fsk, lin, FRAME);
            for (int i = 0; i < FRAME; i++)
                out[i] = v91_linear_to_codeword(law, lin[i]);
        } else if (toneq_start >= 0 && toneq_end < 0) {
            for (int i = 0; i < FRAME; i++) {
                out[i] = v91_linear_to_codeword(law, (int16_t) lrint(3000.0 * sin(ph)));
                ph += 2.0 * M_PI * 980.0 / 8000.0;
            }
        }
        if (write_full(e.to_fd, header, 2) || write_full(e.to_fd, out, FRAME)
            || read_full(e.from_fd, eng + total, FRAME)) {
            printf("    FAIL: the engine stopped at sample %d\n", total);
            failures++;
            break;
        }

        /* What the engine just sent, in 10 ms blocks. */
        for (int b = 0; b < FRAME / BLOCK; b++) {
            int16_t x[BLOCK];
            double rms, p2100;
            int at = total + b * BLOCK;

            for (int i = 0; i < BLOCK; i++)
                x[i] = v91_codeword_to_linear(law, eng[at + i]);
            p2100 = purity(x, BLOCK, 2100.0, &rms);
            if (getenv("QC_TEST_DEBUG") && (at % 800) == 0)
                printf("      t=%6d rms=%7.1f p2100=%.2f qc_start=%d qc_stop=%d gap=%d pcm=%d\n",
                       at, rms, p2100, qc_start, qc_stop, saw_gap, pcm_blocks);
            if (rms >= 100.0 && p2100 >= 0.7) {
                if (qc_start < 0 && ansam_blocks == 0)
                    ansam_t0 = at;
                ansam_blocks++;
                ansam_gone_blocks = 0;
                if (saw_gap) {
                    pcm_blocks++;
                    pcm_miss = 0;
                }
            } else {
                if (qc_start >= 0 && qc_stop < 0 && ++ansam_gone_blocks >= 3) {
                    qc_stop = total + FRAME;       /* 9.2.1.1: stop CM, go silent */
                }
                if (qc_start < 0)
                    ansam_blocks = 0;
                if (qc_stop >= 0 && rms < 10.0)
                    saw_gap = 1;
                /* ANSpcm's phase reversals (every 3612 symbols) smear a block
                 * or two; a detector rides through them. */
                if (++pcm_miss >= 3)
                    pcm_blocks = 0;
                if (toneq_start >= 0 && toneq_end < 0 && at > toneq_start + 800)
                    toneq_end = at;
            }
        }
        if (k->send_toneq && qc_stop >= 0 && toneq_start < 0 && pcm_blocks >= 100)
            toneq_start = total + FRAME;
        usleep(1000);
    }
    close(e.to_fd);
    close(e.from_fd);
    waitpid(e.pid, NULL, 0);
    if (fsk)
        fsk_tx_free(fsk);
    log = slurp(e.log_path);
    (void) ansam_t0;

    if (k->pqc == 3) {
        CHECK(!strstr(log, "9.2.4.1"), "+PQC=3 must ignore the QC1a");
        CHECK(qc_start >= 0, "script never reached QC1a (ANSam not heard)");
        printf("    QC1a ignored under +PQC=3: %s\n", strstr(log, "9.2.4.1") ? "no" : "yes");
        free(log);
        return;
    }
    CHECK(qc_start >= 0, "script never reached QC1a (ANSam not heard)");
    CHECK(strstr(log, "9.2.4.1: QC1a (WXYZ=5, UQTS=70, P=1)") != NULL,
          "engine did not take the QC1a");
    if (qc_start < 0 || !strstr(log, "9.2.4.1")) {
        free(log);
        return;
    }

    /* QCA1d: a strict Table 5 sequence on V.21(H), P and LM as offered. */
    {
        fsk_rx_state_t *rx;
        bitbuf_t bb = { .n = 0 };
        int found = 0;

        rx = fsk_rx_init(NULL, &preset_fsk_specs[FSK_V21CH2], FSK_FRAME_MODE_SYNC, put_bit, &bb);
        for (int off = qc_start; off + 160 <= total && off < qc_start + 12000; off += 160) {
            int16_t lin[160];

            for (int i = 0; i < 160; i++)
                lin[i] = v91_codeword_to_linear(law, eng[off + i]);
            fsk_rx(rx, lin, 160);
        }
        for (int st = 0; st + 70 <= bb.n && !found; st++) {
            v92_short_phase1_candidate_t c;

            if (v92_decode_short_phase1_sequence_strict(bb.bits + st, bb.n - st, true, &c)) {
                found = 1;
                CHECK(c.digital_modem && c.qca && c.aux_value == 1 && c.lapm,
                      "QCA1d fields: %s LM=%d P=%d", c.name, c.aux_value, c.lapm);
            }
        }
        CHECK(found, "no strict QCA1d on the wire");
        fsk_rx_free(rx);
    }

    /* The silence, QTS and ANSpcm that follow, codeword-exact. */
    {
        int gap = -1, run = 0, qts_at, pcm_at, pcm_len = 0, after = 0;
        v92_qts_hit_t qts;

        /* The first silent codeword after the engine took the line. */
        for (int i = qc_start + 1600; i < total; i++) {
            int16_t l = v91_codeword_to_linear(law, eng[i]);

            if (abs(l) <= 8 && eng[i] == sil) {
                gap = i;
                break;
            }
        }
        CHECK(gap > 0, "no silence after QCA1d");
        if (gap > 0) {
            while (gap + run < total && eng[gap + run] == sil)
                run++;
            CHECK(run == 600, "silence between QCA1d and QTS: %d codewords, want 600 (75 ms)", run);
            qts_at = gap + run;
            memset(&qts, 0, sizeof(qts));
            CHECK(v92_detect_qts_sequence(eng, total, law, 70, qts_at - 12, qts_at + 900,
                                          (v92_codeword_to_ucode_fn) v91_codeword_to_ucode, &qts)
                  && qts.start_sample == qts_at && qts.qts_reps == 128 && qts.qts_bar_reps == 8,
                  "QTS: seen=%d at %d (want %d) reps=%d bar=%d", qts.seen, qts.start_sample,
                  qts_at, qts.qts_reps, qts.qts_bar_reps);
            pcm_at = qts_at + 816;
            while (pcm_at + pcm_len < total
                   && eng[pcm_at + pcm_len] == v92_anspcm_codeword(law, 1, pcm_len))
                pcm_len++;
            printf("    QCA1d, %d-codeword gap, QTS at %d, ANSpcm %d codewords (-12 dBm0)\n",
                   run, qts_at, pcm_len);
            if (k->send_toneq) {
                int lag = pcm_at + pcm_len - toneq_start;

                CHECK(toneq_start > 0, "script never reached TONEq");
                CHECK(lag >= 0 && lag <= 1200, "ANSpcm stopped %d codewords after TONEq began "
                      "(want within 150 ms)", lag);
                while (pcm_at + pcm_len + after < total
                       && eng[pcm_at + pcm_len + after] == sil)
                    after++;
                printf("    TONEq at %d: ANSpcm stopped %d later, then %d silent codewords\n",
                       toneq_start, lag, after);
                CHECK(after >= 600 && after <= 600 + 40, "silence before Phase 2: %d codewords, "
                      "want 600 (75 +/- 5 ms)", after);
                CHECK(strstr(log, "short Phase 1 complete") != NULL, "no completion line");
                CHECK(strstr(log, "enter TRAINING") != NULL || strstr(log, "V.90") != NULL,
                      "Phase 2 never started");
            } else {
                int expect = 16000 - 1416;

                CHECK(pcm_len >= expect - 4 && pcm_len <= expect + 4,
                      "ANSpcm ran %d codewords, want about %d (2 s after QCA1d)", pcm_len, expect);
                CHECK(strstr(log, "no TONEq within 2 s") != NULL, "no timeout line");
                CHECK(strstr(log, "short Phase 1 complete") == NULL, "must not complete");
                /* Back on V.8: ANSam is on the line again. */
                {
                    int blocks = 0;

                    for (int i = pcm_at + pcm_len + 1600; i + BLOCK <= total; i += BLOCK) {
                        int16_t x[BLOCK];
                        double rms;

                        for (int q = 0; q < BLOCK; q++)
                            x[q] = v91_codeword_to_linear(law, eng[i + q]);
                        if (purity(x, BLOCK, 2100.0, &rms) >= 0.7 && rms >= 100.0)
                            blocks++;
                    }
                    CHECK(blocks >= 50, "ANSam did not resume after the timeout (%d blocks)", blocks);
                }
            }
        }
    }
    free(log);
}

int main(int argc, char **argv)
{
    qcase_t happy_u = { 0, 1, 1, 12 };
    qcase_t happy_a = { 1, 1, 1, 12 };
    qcase_t no_tone = { 0, 1, 0, 12 };
    qcase_t disabled = { 0, 3, 0, 6 };

    signal(SIGPIPE, SIG_IGN);
    /* An argument selects one case (0-3), for reading its engine log. */
    int only = argc > 1 ? atoi(argv[1]) : -1;

    if (only < 0 || only == 0)
        run_case("QC1a answered, TONEq sent (mu-law)", &happy_u);
    if (only < 0 || only == 1)
        run_case("QC1a answered, TONEq sent (A-law)", &happy_a);
    if (only < 0 || only == 2)
        run_case("QC1a answered, no TONEq", &no_tone);
    if (only < 0 || only == 3)
        run_case("+PQC=3 ignores QC1a", &disabled);
    if (failures) {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("v92_qc_engine_test: all passed\n");
    return 0;
}
