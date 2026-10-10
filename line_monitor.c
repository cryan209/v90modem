/*
 * line_monitor.c -- recent line audio and its spectrum.  See line_monitor.h.
 */

#include "line_monitor.h"

#include <spandsp.h>

#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RING     8000            /* one second at 8 kHz */
#define MIN_FED  800             /* 0.1 s before anything is reported */
#define NFFT     256             /* 31.25 Hz bins */
#define HOP      (NFFT / 2)

typedef struct {
    int16_t buf[RING];
    int wr;                      /* next write position */
    int fed;                     /* samples held, up to RING */
    uint64_t total;
} lm_ring_t;

static lm_ring_t rings[2];
static bool gui_enabled;
static uint8_t wire[2][256], pcm[2][256];
static uint64_t wire_count[2], pcm_count[2];
typedef struct { uint8_t byte; uint64_t count; } gui_run_t;
static gui_run_t runs[2][256];
static uint64_t run_count[2];
static unsigned partial[2], partial_bits[2];
static float iq[2][256][2];
static float eye[128][5];
static uint64_t eye_count;
static uint64_t iq_count[2];
static char events[16][192];
static uint64_t event_count, gui_epoch;

static pthread_mutex_t lm_mtx = PTHREAD_MUTEX_INITIALIZER;

void lm_reset(void)
{
    pthread_mutex_lock(&lm_mtx);
    memset(rings, 0, sizeof(rings));
    memset(wire_count, 0, sizeof(wire_count));
    memset(run_count, 0, sizeof(run_count));
    memset(pcm_count, 0, sizeof(pcm_count));
    memset(partial, 0, sizeof(partial));
    memset(partial_bits, 0, sizeof(partial_bits));
    memset(iq_count, 0, sizeof(iq_count));
    event_count = 0;
    eye_count = 0;
    gui_epoch++;
    pthread_mutex_unlock(&lm_mtx);
}

static void put_locked(lm_ring_t *r, int16_t v)
{
    r->total++;
    r->buf[r->wr] = v;
    if (++r->wr == RING)
        r->wr = 0;
    if (r->fed < RING)
        r->fed++;
}

void lm_feed(int dir, const int16_t *amp, int len)
{
    if (!amp || len <= 0 || (dir != LM_RX && dir != LM_TX))
        return;
    pthread_mutex_lock(&lm_mtx);
    for (int i = 0; i < len; i++)
        put_locked(&rings[dir], amp[i]);
    pthread_mutex_unlock(&lm_mtx);
}

void lm_feed_g711(int dir, const uint8_t *codewords, int len, bool alaw)
{
    if (!codewords || len <= 0 || (dir != LM_RX && dir != LM_TX))
        return;
    pthread_mutex_lock(&lm_mtx);
    for (int i = 0; i < len; i++) {
        put_locked(&rings[dir], alaw ? alaw_to_linear(codewords[i]) : ulaw_to_linear(codewords[i]));
        if (gui_enabled) pcm[dir][pcm_count[dir]++ % 256] = codewords[i];
    }
    pthread_mutex_unlock(&lm_mtx);
}

/* The held samples, oldest first.  Returns the count. */
static int snapshot(int dir, int16_t *out)
{
    lm_ring_t *r = &rings[dir];
    int n;
    int start;

    pthread_mutex_lock(&lm_mtx);
    n = r->fed;
    start = (r->wr - n + RING) % RING;
    for (int i = 0; i < n; i++)
        out[i] = r->buf[(start + i) % RING];
    pthread_mutex_unlock(&lm_mtx);
    return n;
}

/* Mean square to dBm0 on SpanDSP's scale: a full-scale sine, mean square
   32768^2/2, is DBM0_MAX_SINE_POWER (+3.14 dBm0). */
static float ms_to_dbm0(double ms)
{
    if (ms <= 0.0)
        return -99.0f;
    return (float) (10.0 * log10(ms / (32768.0 * 32768.0 / 2.0))) + DBM0_MAX_SINE_POWER;
}

bool lm_level_dbm0(int dir, float *dbm0)
{
    static int16_t x[RING];
    double sum = 0.0;
    int n;

    if (dir != LM_RX && dir != LM_TX)
        return false;
    n = snapshot(dir, x);
    if (n < MIN_FED)
        return false;
    for (int i = 0; i < n; i++)
        sum += (double) x[i] * x[i];
    *dbm0 = ms_to_dbm0(sum / n);
    return true;
}

/* Band powers for one direction.  For a Hann-windowed block, Parseval gives
   sum_k |X_k|^2 = N sum_n (w_n x_n)^2, so the mean square a set of one-sided
   bins carries is sum_k c_k |X_k|^2 / (N * sum_n w_n^2), with c_k = 2 except at
   DC and Nyquist.  Each band takes the bins whose centre falls in
   [centre - 75, centre + 75) Hz. */
static int bands_one(int dir, float *out)
{
    static int16_t x[RING];
    static double cosv[NFFT];
    static double sinv[NFFT];
    static double win[NFFT];
    static double wsum2;
    static bool init;
    double acc[NFFT / 2 + 1];
    int n;
    int blocks = 0;

    if (!init) {
        wsum2 = 0.0;
        for (int i = 0; i < NFFT; i++) {
            cosv[i] = cos(2.0 * M_PI * i / NFFT);
            sinv[i] = sin(2.0 * M_PI * i / NFFT);
            win[i] = 0.5 - 0.5 * cos(2.0 * M_PI * i / NFFT);
            wsum2 += win[i] * win[i];
        }
        init = true;
    }
    n = snapshot(dir, x);
    memset(acc, 0, sizeof(acc));
    for (int s = 0; s + NFFT <= n; s += HOP) {
        double y[NFFT];

        for (int i = 0; i < NFFT; i++)
            y[i] = win[i] * x[s + i];
        for (int k = 0; k <= NFFT / 2; k++) {
            double re = 0.0, im = 0.0;
            int idx = 0;

            for (int i = 0; i < NFFT; i++) {
                re += y[i] * cosv[idx];
                im -= y[i] * sinv[idx];
                idx += k;
                if (idx >= NFFT)
                    idx -= NFFT;
            }
            acc[k] += re * re + im * im;
        }
        blocks++;
    }
    for (int b = 0; b < LM_BANDS; b++) {
        double lo = (b + 1) * LM_BAND_HZ - LM_BAND_HZ / 2.0;
        double hi = lo + LM_BAND_HZ;
        double ms = 0.0;

        if (blocks == 0) {
            out[b] = -99.0f;
            continue;
        }
        for (int k = 0; k <= NFFT / 2; k++) {
            double f = k * 8000.0 / NFFT;

            if (f >= lo && f < hi)
                ms += (k == 0 || k == NFFT / 2 ? 1.0 : 2.0) * acc[k];
        }
        out[b] = ms_to_dbm0(ms / blocks / (NFFT * wsum2));
    }
    return blocks ? n : 0;
}

float lm_bands_dbm0(float *rx, float *tx)
{
    float tmp[LM_BANDS];
    int n_rx;
    int n_tx;

    n_rx = bands_one(LM_RX, rx ? rx : tmp);
    n_tx = bands_one(LM_TX, tx ? tx : tmp);
    return (float) (n_rx > n_tx ? n_rx : n_tx) / 8000.0f;
}

typedef struct {
    char *p;
    size_t left;
    size_t used;
} sink_t;

static void put(sink_t *s, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (s->left == 0)
        return;
    va_start(ap, fmt);
    n = vsnprintf(s->p, s->left, fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if ((size_t) n >= s->left)
        n = (int) s->left - 1;
    s->p += n;
    s->left -= (size_t) n;
    s->used += (size_t) n;
}

static void level(sink_t *s, float dbm0)
{
    if (dbm0 <= -98.0f)
        put(s, "   ---");
    else
        put(s, " %5.1f", dbm0);
}

int lm_format_bands(char *out, size_t len)
{
    float rx[LM_BANDS];
    float tx[LM_BANDS];
    float secs;
    float tot;
    sink_t s = { out, len, 0 };

    if (!out || len == 0)
        return -1;
    out[0] = '\0';
    secs = lm_bands_dbm0(rx, tx);
    put(&s, "Line Spectrum (last %.1f s, dBm0 per %d Hz band)\r\n", secs, LM_BAND_HZ);
    if (secs <= 0.0f) {
        put(&s, "No line audio yet.\r\n");
        return (int) s.used;
    }
    put(&s, "  Freq    Rx    Tx  Rx: -60     -40     -20     0\r\n");
    for (int b = 0; b < LM_BANDS; b++) {
        int bars = rx[b] <= -60.0f ? 0 : (int) ((rx[b] + 60.0f) / 2.5f + 0.5f);

        if (bars > 24)
            bars = 24;
        put(&s, "  %4d", (b + 1) * LM_BAND_HZ);
        level(&s, rx[b]);
        level(&s, tx[b]);
        put(&s, "      ");
        for (int i = 0; i < bars; i++)
            put(&s, "#");
        put(&s, "\r\n");
    }
    put(&s, "Total ");
    put(&s, " Rx");
    level(&s, lm_level_dbm0(LM_RX, &tot) ? tot : -99.0f);
    put(&s, "  Tx");
    level(&s, lm_level_dbm0(LM_TX, &tot) ? tot : -99.0f);
    put(&s, " dBm0\r\n");
    return (int) s.used;
}

/* All GUI storage is bounded. No I/O or analysis runs on the media clock. */
void lm_gui_enable(void) { gui_enabled = true; }
void lm_wire_bit(int dir, int bit)
{
    if (!gui_enabled || (bit != 0 && bit != 1) || dir < 0 || dir > 1) return;
    pthread_mutex_lock(&lm_mtx);
    partial[dir] |= (unsigned)bit << partial_bits[dir];
    if (++partial_bits[dir] == 8) {
        wire[dir][wire_count[dir]++ % 256] = partial[dir];
        gui_run_t *last = run_count[dir] ? &runs[dir][(run_count[dir]-1)%256] : NULL;
        if (last && last->byte == partial[dir]) last->count++;
        else { gui_run_t *r = &runs[dir][run_count[dir]++%256]; r->byte = partial[dir]; r->count = 1; }
        partial[dir] = partial_bits[dir] = 0;
    }
    pthread_mutex_unlock(&lm_mtx);
}
void lm_qam(float re, float im, bool decision)
{
    if (!gui_enabled || !isfinite(re) || !isfinite(im)) return;
    int d = decision ? 1 : 0;
    pthread_mutex_lock(&lm_mtx);
    unsigned i = iq_count[d]++ % 256;
    iq[d][i][0] = re; iq[d][i][1] = im;
    pthread_mutex_unlock(&lm_mtx);
}
void lm_eye(float in_re, float in_im, float eq_re, float eq_im, int phase)
{
    if (!gui_enabled || !isfinite(in_re) || !isfinite(in_im) || !isfinite(eq_re) || !isfinite(eq_im) || (phase != 0 && phase != 1)) return;
    pthread_mutex_lock(&lm_mtx);
    float *p = eye[eye_count++ % 128];
    p[0] = in_re; p[1] = in_im; p[2] = eq_re; p[3] = eq_im; p[4] = phase;
    pthread_mutex_unlock(&lm_mtx);
}
void lm_event(const char *text)
{
    if (!gui_enabled) return;
    pthread_mutex_lock(&lm_mtx);
    snprintf(events[event_count++ % 16], 192, "%s", text);
    pthread_mutex_unlock(&lm_mtx);
}
int lm_gui_json(char *out, size_t size)
{
    size_t at = 0;
#define ADD(...) do { if (at < size) { int added = snprintf(out+at, size-at, __VA_ARGS__); if (added > 0) at += (size_t)added; } } while (0)
    pthread_mutex_lock(&lm_mtx);
    ADD("\"epoch\":%llu,\"audio\":[", (unsigned long long)gui_epoch);
    for (int d = 0; d < 2; d++) {
        int n = rings[d].fed < 512 ? rings[d].fed : 512;
        ADD("%s[", d ? "," : "");
        for (int i = 0; i < n; i++) ADD("%s%d", i ? "," : "", rings[d].buf[(rings[d].wr-n+i+RING)%RING]);
        ADD("]");
    }
    ADD("],\"wire\":[");
    for (int d = 0; d < 2; d++) {
        uint64_t n = wire_count[d] < 256 ? wire_count[d] : 256;
        ADD("%s{\"count\":%llu,\"hex\":\"", d ? "," : "", (unsigned long long)wire_count[d]);
        for (uint64_t i = wire_count[d]-n; i < wire_count[d]; i++) ADD("%02x", wire[d][i%256]);
        ADD("\",\"runs\":[");
        uint64_t nr = run_count[d] < 256 ? run_count[d] : 256;
        for (uint64_t i = run_count[d]-nr; i < run_count[d]; i++) {
            gui_run_t *r = &runs[d][i%256];
            ADD("%s[%u,%llu]", i > run_count[d]-nr ? "," : "", r->byte, (unsigned long long)r->count);
        }
        ADD("]}");
    }
    ADD("],\"pcm\":[");
    for (int d = 0; d < 2; d++) {
        uint64_t n = pcm_count[d] < 256 ? pcm_count[d] : 256;
        ADD("%s{\"count\":%llu,\"hex\":\"", d ? "," : "", (unsigned long long)pcm_count[d]);
        for (uint64_t i = pcm_count[d]-n; i < pcm_count[d]; i++) ADD("%02x", pcm[d][i%256]);
        ADD("\"}");
    }
    ADD("],\"listen\":[");
    for (int d = 0; d < 2; d++) {
        int n = rings[d].fed < 3200 ? rings[d].fed : 3200;
        ADD("%s{\"count\":%llu,\"hex\":\"", d ? "," : "", (unsigned long long)rings[d].total);
        for (int i = 0; i < n; i++) {
            uint16_t v = (uint16_t)rings[d].buf[(rings[d].wr-n+i+RING)%RING];
            ADD("%02x%02x", v & 255, v >> 8);
        }
        ADD("\"}");
    }
    ADD("],\"iq\":[");
    for (int d = 0; d < 2; d++) {
        uint64_t n = iq_count[d] < 256 ? iq_count[d] : 256;
        ADD("%s[", d ? "," : "");
        for (uint64_t i = iq_count[d]-n; i < iq_count[d]; i++)
            ADD("%s[%.5g,%.5g]", i > iq_count[d]-n ? "," : "", iq[d][i%256][0], iq[d][i%256][1]);
        ADD("]");
    }
    ADD("],\"eye_count\":%llu,\"eye\":[", (unsigned long long)eye_count);
    uint64_t ne = eye_count < 128 ? eye_count : 128;
    for (uint64_t i = eye_count-ne; i < eye_count; i++) {
        float *p = eye[i%128];
        ADD("%s[%.5g,%.5g,%.5g,%.5g,%d]", i > eye_count-ne ? "," : "", p[0], p[1], p[2], p[3], (int)p[4]);
    }
    ADD("],\"event_count\":%llu,\"events\":[", (unsigned long long)event_count);
    uint64_t n = event_count < 16 ? event_count : 16;
    for (uint64_t i = event_count-n; i < event_count; i++) {
        ADD("%s\"", i > event_count-n ? "," : "");
        for (const unsigned char *c = (unsigned char *)events[i%16]; *c; c++) {
            if (*c == '"' || *c == '\\') ADD("\\%c", *c);
            else if (*c >= 32 && *c < 127) ADD("%c", *c);
            else ADD(" ");
        }
        ADD("\"");
    }
    ADD("]");
    pthread_mutex_unlock(&lm_mtx);
#undef ADD
    return at < size ? (int)at : -1;
}
