/* apple_usb_modem_coupler -- the Apple USB Modem (A1082, USB 05ac:1401) as
 * v90modem's ANALOGUE side over a real 2-wire line.
 *
 * The role hsf_v90_coupler.c / hsf_fxo_probe.c play for the Conexant 0572:1300
 * (docs/hsf_analogue_v90_coupler.md).  This part seizes the line, dials an
 * extension with DTMF, waits for the far end's answer tone and then runs the
 * modem engine over the result, with the DTE on a PTY.
 *
 * Line control is USB; audio is CoreAudio.  Both are established against the
 * device and written up in docs/apple_usb_modem_sm56.md:
 *
 *   register 5 bit 0   off-hook (loop closure).  Verified by the dial tone it
 *                      draws: 350 + 440 Hz at equal level, -15.9 dBFS.
 *   register 5 bit 3   receive path / on-hook monitor.  Not the hook.
 *   register 0x1d      analogue line sense.  0x00 means NO PAIR CONNECTED,
 *                      which is the one reading worth interpreting.
 *   80 <idx> 00        read a register (then GET_ENCAPSULATED_RESPONSE)
 *   00 <idx> <val>     write one
 *
 * THE RATE IS 9600, NOT 8000, and the arithmetic is the reason.  The device's
 * rate list is 8000 plus every V.34 symbol rate times three, because the SM56's
 * host datapump ran its receiver on a T/3 grid -- the same grid this tree's own
 * T/3 upstream receiver uses.  The engine wants two grids: 8000 for
 * me_rx_audio() and 16000 (T/2) for me_rx_v90a_16k(), and of the seven offered
 * rates only two reach both by a small exact ratio:
 *
 *     8000  -> 8000 = 1/1    -> 16000 = 2/1
 *     9600  -> 8000 = 5/6    -> 16000 = 5/3
 *
 * and the difference between them is the whole argument.  8000 -> 16000 is
 * UPSAMPLING: it invents the T/2 samples by interpolation instead of measuring
 * them, from a stream that is already critically sampled -- V.34 at 3429 baud
 * occupies up to 3673 Hz, so 8000 leaves 326 Hz of Nyquist margin.  9600 leaves
 * 1126 Hz, and 9600 -> 16000 carries genuine information to 4800 Hz, which
 * covers the whole DS0 band.  So sampling at 8000 is a dead end for the V.90
 * analogue role, which can only recover its downstream from T/2 samples, while
 * 9600 reaches both grids and is additionally T/3 at 3200 baud exactly.
 *
 * Both resamplers are therefore exact rational polyphase, not fractional
 * delays, and that is also what keeps the HSF path's defect out of here: that
 * coupler decimates 16 kHz by two and so must CHOOSE which of two sample sets
 * to keep, and swept as a fractional delay only 1/10, 2/10 and 9/10 of phases
 * reached Phase 4 on three recorded calls, with 0.0 -- the obvious value --
 * failing on all three.  A 5/6 polyphase discards nothing and has no free
 * parameter; the constant group delay it adds is not a choice.
 *
 * --rate still accepts 8000, which skips the receive resampler entirely and is
 * one filter fewer if all you want is V.34; it cannot feed the T/2 path.
 * --selftest measures the resamplers, since they are load-bearing.
 *
 * The DC offset is removed unconditionally.  This device sits at about +650
 * counts, and on the HSF part a standing 908-count offset made SpanDSP's
 * ANS/ANSam detector reject a 5000-count tone outright -- the same recording
 * with the offset removed is recognised in 1.4 s and as-is is never recognised
 * in 70 s.  Every level and slicer decision downstream has the same exposure.
 * One pole at 40 Hz: -0.08 dB at 300 Hz, so it costs the signal band nothing.
 *
 * Answer detection is a 2100 Hz Goertzel, not a post-dial timer.  A timer ran
 * V.8 into ringback on the HSF path and its 10 s timeout expired at the moment
 * the far end answered; ringback is 400/440/480 Hz and the answering modem's
 * ANS/ANSam is 2100 Hz, so one Goertzel separates them.  Measured on this
 * device dialling 8416: 2100 Hz at amplitude 6595 with the peak bin exactly
 * 2100 Hz, 6.6 s after the digits went out.
 *
 * NOT VERIFIED ON A LINE.  Everything above about the device is measured;
 * this program driving a call through the engine is not, because the pair was
 * disconnected when it was written.  --rx-replay runs the whole analogue side
 * offline against a recorded receive tap, which is how the HSF path killed four
 * hypotheses in an afternoon instead of a call each, and is the way to exercise
 * this without a line.
 *
 * Usage:
 *   apple_usb_modem_coupler --dial 8416 [--pty-link /tmp/applemodem]
 *   apple_usb_modem_coupler --rx-replay tap.s16 [--pty-link ...]
 *   apple_usb_modem_coupler --hook on|off      (line control only)
 *
 * Build: make apple_usb_modem_coupler
 */

#import <AVFoundation/AVFoundation.h>
#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <libusb.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "modem_engine.h"
#include "data_interface.h"

#define VID 0x05ac
#define PID 0x1401
#define IF_ACM 0
#define IF_CMD 1
#define REG_HOOK       0x05
#define REG_HOOK_BIT   0x01
#define REG_MONITOR_BIT 0x08
#define REG_SENSE      0x1d

/* ------------------------------------------------------------------ */
/* Line control over USB                                             */
/* ------------------------------------------------------------------ */

static libusb_device_handle *usb;

static int reg_read(unsigned char idx)
{
    unsigned char cmd[3] = { 0x80, idx, 0x00 }, in[8], notify[16];
    int xfer = 0, rc;

    if (libusb_control_transfer(usb, 0x21, 0x00, 0, IF_CMD, cmd, 3, 1000) != 3)
        return -1;
    /* RESPONSE_AVAILABLE if interface 0 is ours; not required. */
    libusb_interrupt_transfer(usb, 0x81, notify, sizeof notify, &xfer, 300);
    for (int try = 0; try < 4; try++) {
        rc = libusb_control_transfer(usb, 0xA1, 0x01, 0, IF_CMD, in, sizeof in, 1000);
        if (rc > 0) return in[0];
        if (rc < 0) return -1;
        usleep(20000);
    }
    return -1;
}

static int reg_write(unsigned char idx, unsigned char val)
{
    unsigned char cmd[3] = { 0x00, idx, val };
    return libusb_control_transfer(usb, 0x21, 0x00, 0, IF_CMD, cmd, 3, 1000) == 3 ? 0 : -1;
}

static int usb_open_line(void)
{
    int cfg = -1;

    if (libusb_init(NULL) < 0) { fprintf(stderr, "libusb_init failed\n"); return -1; }
    usb = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!usb) { fprintf(stderr, "no %04x:%04x on the bus\n", VID, PID); return -1; }
    libusb_get_configuration(usb, &cfg);
    /* An unconfigured device stalls EVERY request, including standard ones, and
     * ioreg shows it with no interface children -- it reads as dead hardware.
     * Setting the configuration is also what lets usbaudiod attach, so it must
     * happen before CoreAudio is asked for the device. */
    if (cfg != 1 && libusb_set_configuration(usb, 1) < 0) {
        fprintf(stderr, "could not set configuration 1\n");
        return -1;
    }
    libusb_claim_interface(usb, IF_ACM);
    if (libusb_claim_interface(usb, IF_CMD) < 0) {
        fprintf(stderr, "could not claim the command interface\n");
        return -1;
    }
    return 0;
}

static int line_hook(int off_hook)
{
    int v = reg_read(REG_HOOK), sense;

    if (v < 0) { fprintf(stderr, "cannot read register 5\n"); return -1; }
    v = off_hook ? (v | REG_HOOK_BIT) : (v & ~REG_HOOK_BIT);
    /* The monitor bit is left set either way: on-hook it is what makes a
     * capture possible at all, and off-hook it costs nothing. */
    v |= REG_MONITOR_BIT;
    if (reg_write(REG_HOOK, (unsigned char)v) < 0) {
        fprintf(stderr, "hook write rejected\n"); return -1;
    }
    usleep(500000);
    sense = reg_read(REG_SENSE);
    fprintf(stderr, "[APPLE] %s: register 5 = 0x%02x, line sense 0x%02x%s\n",
            off_hook ? "off-hook" : "on-hook", reg_read(REG_HOOK) & 0xff,
            sense & 0xff, sense == 0 ? "   <- NO PAIR CONNECTED" : "");
    return sense == 0 ? -1 : 0;
}

/* ------------------------------------------------------------------ */
/* Transmit script: DTMF, then hand the stream to the engine          */
/* ------------------------------------------------------------------ */

#define TX_MAX_SEG 128
struct tx_seg { double f1, f2; long n; };
static struct tx_seg tx_script[TX_MAX_SEG];
static int    tx_nseg, tx_seg;
static long   tx_pos;
static double tx_ph1, tx_ph2;
static double tx_amp = 0.15;   /* per tone; a pair lands near -16.5 dBFS */
static double g_rate = 9600.0;   /* see the header: not 8000 */
static double g_tx_gain = 1.0;
static volatile int tx_dtmf_done;

static int dtmf_pair(char c, double *lo, double *hi)
{
    static const char *rows[4] = { "123A", "456B", "789C", "*0#D" };
    static const double lf[4] = { 697, 770, 852, 941 };
    static const double hf[4] = { 1209, 1336, 1477, 1633 };
    for (int r = 0; r < 4; r++)
        for (int k = 0; k < 4; k++)
            if (rows[r][k] == c) { *lo = lf[r]; *hi = hf[k]; return 0; }
    return -1;
}

static void tx_add(double f1, double f2, double ms)
{
    if (tx_nseg >= TX_MAX_SEG) return;
    tx_script[tx_nseg].f1 = f1;
    tx_script[tx_nseg].f2 = f2;
    tx_script[tx_nseg].n = (long)(g_rate * ms / 1000.0);
    tx_nseg++;
}

static int build_dial_script(const char *digits, double on_ms, double off_ms)
{
    double lo, hi;

    tx_add(0, 0, 300.0);            /* let the exchange settle after seizure */
    for (const char *p = digits; *p; p++) {
        char c = *p;
        if (c == ',' || c == ' ') { tx_add(0, 0, 500.0); continue; }
        if (c >= 'a' && c <= 'd') c -= 32;
        if (dtmf_pair(c, &lo, &hi) < 0) {
            fprintf(stderr, "not a DTMF digit: '%c'\n", *p); return -1;
        }
        tx_add(lo, hi, on_ms);
        tx_add(0, 0, off_ms);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Exact rational resampling                                          */
/* ------------------------------------------------------------------ */

/* Polyphase L/M.  The prototype is a windowed sinc at the L-times-upsampled
 * rate, cut at 1/(2*max(L,M)) so it serves as both the interpolation and the
 * anti-alias filter, and the polyphase decomposition means only every Lth tap
 * is touched per output -- there is no free phase to pick, which is the point
 * (see the header on the HSF coupler's fractional delay). */
/* 48 taps per phase.  16 was measured too short and it was NOT a subtle
 * failure: --selftest read 0.82 gain at 3600 Hz on the 5/6 path and 16.2 dB
 * SNDR on the 6/5 transmit path, where the 8000 -> 9600 image at 4400 Hz sits
 * only 400 Hz into the stopband.  Both would have presented inside the modem as
 * a level or noise problem, miles from the cause. */
#define RS_TAPS 48
#define RS_L_MAX 6
#define RS_HIST (RS_TAPS + 2)

struct resamp {
    int L, M;
    double h[RS_TAPS * RS_L_MAX + RS_L_MAX];   /* h[j*L + p] */
    double hist[RS_HIST];
    int hn;                             /* input samples consumed */
    long k;                             /* output index */
};

static void rs_init(struct resamp *r, int L, int M)
{
    int n = RS_TAPS * L;
    double fc = 0.5 / (L > M ? L : M);   /* normalised to the L*fs_in rate */
    double sum = 0;

    memset(r, 0, sizeof *r);
    r->L = L;
    r->M = M;
    (void)sum;
    for (int i = 0; i < n; i++) {
        double t = i - (n - 1) / 2.0;
        double x = 2.0 * M_PI * fc * t;
        double sinc = (fabs(x) < 1e-9) ? 1.0 : sin(x) / x;
        /* Hamming, so the stopband is about -53 dB -- ample against the 1126 Hz
         * of Nyquist margin 9600 leaves at the worst V.34 symbol rate. */
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * i / (n - 1));
        r->h[i] = sinc * w;
    }
    /* Normalise EACH PHASE to unit DC gain.  One output touches one phase, so
     * normalising the whole prototype would leave every output a factor of L
     * out -- and a gain error here presents as a level problem in the modem,
     * miles from its cause. */
    for (int p = 0; p < L; p++) {
        double ps = 0;
        for (int j = 0; j < RS_TAPS; j++) ps += r->h[j * L + p];
        if (fabs(ps) < 1e-12) ps = 1.0;
        for (int j = 0; j < RS_TAPS; j++) r->h[j * L + p] /= ps;
    }
}

/* Feed one input sample, emit however many outputs fall due (0, 1 or more). */
static int rs_put(struct resamp *r, double x, double *out, int max_out)
{
    int got = 0;

    for (int i = RS_HIST - 1; i > 0; i--) r->hist[i] = r->hist[i - 1];
    r->hist[0] = x;
    r->hn++;
    /* Output k sits at input position k*M/L; emit while that is the sample just
     * pushed.  One input can owe several outputs when L > M. */
    for (;;) {
        long m = (r->k * r->M) / r->L;
        int  p = (int)((r->k * r->M) % r->L);
        double acc = 0;

        if (m > r->hn - 1) break;
        if (got >= max_out) break;
        /* hist[0] is input n = hn-1, so input (m - j) is hist[hn-1-m+j]. */
        for (int j = 0; j < RS_TAPS; j++) {
            int hi = (int)(r->hn - 1 - m) + j;
            if (hi < 0 || hi >= RS_HIST) continue;
            acc += r->h[j * r->L + p] * r->hist[hi];
        }
        out[got++] = acc;
        r->k++;
    }
    return got;
}

static int16_t clip16(double v)
{
    if (v >  32767.0) return  32767;
    if (v < -32768.0) return -32768;
    return (int16_t)lrint(v);
}

/* ------------------------------------------------------------------ */
/* Engine plumbing                                                    */
/* ------------------------------------------------------------------ */

static AudioUnit au;
static AudioBufferList *in_abl;
static int  render_errors;
static const char *g_pty_link;
static const char *g_dial;
static const char *g_replay_path;
static volatile int g_answered, g_engine_running, g_stop;
static unsigned g_ans_good, g_ans_bad;

/* DC blocker, one pole at 40 Hz.  See the header: this is not optional. */
static double dc_px, dc_py;

static void dc_block(int16_t *s, long n)
{
    const double a = 1.0 - 2.0 * M_PI * 40.0 / 8000.0;   /* 0.9686 at 8 kHz */
    for (long i = 0; i < n; i++) {
        double x = s[i];
        double y = x - dc_px + a * dc_py;
        dc_px = x;
        dc_py = y;
        if (y >  32767.0) y =  32767.0;
        if (y < -32768.0) y = -32768.0;
        s[i] = (int16_t)lrint(y);
    }
}

/* 2100 Hz answer detection over 20 ms blocks, as a ratio so it is not a level
 * test.  ANSam's phase reversals do not disturb this: the block is 20 ms and
 * the reversals are 450 ms apart, so at most one block in 22 straddles one. */
#define ANS_RATE   8000.0          /* it runs on the RESAMPLED stream, not the
                                    * device's -- using g_rate here had it
                                    * looking for 2100*8000/9600 = 1750 Hz */
#define ANS_BLOCK  160u            /* 20 ms at 8 kHz */
#define ANS_NEEDED  10u            /* 200 ms of 2100 Hz */
static int16_t ans_buf[ANS_BLOCK];
static unsigned ans_fill;

static void answer_block(const int16_t *s)
{
    double w = 2.0 * M_PI * 2100.0 / ANS_RATE;
    double coeff = 2.0 * cos(w), q1 = 0, q2 = 0, energy = 0;

    for (unsigned i = 0; i < ANS_BLOCK; i++) {
        double x = s[i];
        double q0 = coeff * q1 - q2 + x;
        q2 = q1; q1 = q0;
        energy += x * x;
    }
    double mag2 = q1 * q1 + q2 * q2 - coeff * q1 * q2;
    if (energy < 1.0) energy = 1.0;
    double frac = mag2 / (energy * ANS_BLOCK / 2.0);

    if (energy / ANS_BLOCK > 40000.0 && frac > 0.5) {
        g_ans_bad = 0;
        if (++g_ans_good >= ANS_NEEDED && !g_answered) {
            g_answered = 1;
            fprintf(stderr, "[APPLE] 2100 Hz answer tone detected\n");
        }
    } else if (++g_ans_bad >= 3) {
        g_ans_good = 0;
    }
}

/* The engine is driven from the RECEIVE clock in 80-sample blocks, which is the
 * shape live pjmedia delivers (two 80-sample calls per 20 ms tick) and what
 * v90_engine_replay --split reproduces.  Its transmit goes into a ring that the
 * output callback drains, so one clock owns the DS0 grid. */
#define RING_N 8192
static int16_t ring[RING_N];
static volatile unsigned ring_w, ring_r;

/* Receive chain: device rate -> 8000 for me_rx_audio, and (from 9600 only)
 * -> 16000 for me_rx_v90a_16k.  The T/2 stream is offered FIRST, as the engine
 * header requires: it is the stream the V.90 analogue Phase 3 receiver reads,
 * and it must see the samples before the 8 kHz decimation. */
static struct resamp rs_8k, rs_16k, rs_tx;
static int16_t fb8[1024];
static long    fb8_n;

static void engine_block_8k(const int16_t *s, long n)
{
    long off = 0;

    while (off < n) {
        long k = n - off;
        if (k > 80) k = 80;                 /* the shape live pjmedia delivers */
        if (!g_answered) {
            for (long i = 0; i < k; i++) {
                ans_buf[ans_fill++] = s[off + i];
                if (ans_fill == ANS_BLOCK) { answer_block(ans_buf); ans_fill = 0; }
            }
        }
        if (g_answered && !g_engine_running) {
            g_engine_running = 1;
            me_on_sip_connected();
            fprintf(stderr, "[APPLE] starting analogue engine (device %.0f Hz)\n", g_rate);
        }
        if (g_engine_running) {
            int16_t out[80];
            double up[8];
            me_rx_audio(s + off, (int)k);
            me_tx_audio(out, (int)k);
            /* 8000 -> device rate on the way out, so the engine keeps its own
             * grid on both sides and the resamplers are the only place that
             * knows about 9600. */
            for (long i = 0; i < k; i++) {
                int got = (g_rate == 8000.0)
                          ? (up[0] = out[i], 1)
                          : rs_put(&rs_tx, out[i], up, 8);
                for (int j = 0; j < got; j++) {
                    unsigned nw = (ring_w + 1) % RING_N;
                    if (nw == ring_r) break;      /* consumer behind; drop */
                    ring[ring_w] = clip16(up[j] * g_tx_gain);
                    ring_w = nw;
                }
            }
        }
        off += k;
    }
}

static void engine_feed(int16_t *s, long n)
{
    dc_block(s, n);

    if (g_rate == 8000.0) {                      /* no receive resampler at all */
        engine_block_8k(s, n);
        return;
    }
    for (long i = 0; i < n; i++) {
        double o[8];
        int got;

        /* T/2 first, per me_rx_v90a_16k()'s contract. */
        got = rs_put(&rs_16k, s[i], o, 8);
        if (got > 0 && g_engine_running) {
            int16_t w[8];
            for (int j = 0; j < got; j++) w[j] = clip16(o[j]);
            me_rx_v90a_16k(w, got);
        }
        got = rs_put(&rs_8k, s[i], o, 8);
        for (int j = 0; j < got; j++) {
            fb8[fb8_n++] = clip16(o[j]);
            if (fb8_n == 80) { engine_block_8k(fb8, 80); fb8_n = 0; }
        }
    }
}

static OSStatus input_cb(void *ref, AudioUnitRenderActionFlags *flags,
                         const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
                         AudioBufferList *unused)
{
    OSStatus rc;

    (void)ref; (void)unused;
    in_abl->mNumberBuffers = 1;
    in_abl->mBuffers[0].mNumberChannels = 1;
    in_abl->mBuffers[0].mDataByteSize = nframes * 2;
    rc = AudioUnitRender(au, flags, ts, bus, nframes, in_abl);
    if (rc != noErr) {
        if (render_errors++ < 3)
            fprintf(stderr, "  AudioUnitRender: %d\n", (int)rc);
        return rc;
    }
    engine_feed(in_abl->mBuffers[0].mData, nframes);
    return noErr;
}

static OSStatus output_cb(void *ref, AudioUnitRenderActionFlags *flags,
                          const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
                          AudioBufferList *io)
{
    int16_t *out = io->mBuffers[0].mData;

    (void)ref; (void)flags; (void)ts; (void)bus;
    for (UInt32 i = 0; i < nframes; i++) {
        double v = 0;

        /* DTMF first: while the script has samples left it owns the line, and
         * the engine's output is not yet meaningful anyway. */
        while (tx_seg < tx_nseg && tx_pos >= tx_script[tx_seg].n) {
            tx_seg++; tx_pos = 0; tx_ph1 = tx_ph2 = 0;
        }
        if (tx_seg < tx_nseg) {
            struct tx_seg *g = &tx_script[tx_seg];
            if (g->f1 > 0) { v += tx_amp * sin(tx_ph1); tx_ph1 += 2 * M_PI * g->f1 / g_rate; }
            if (g->f2 > 0) { v += tx_amp * sin(tx_ph2); tx_ph2 += 2 * M_PI * g->f2 / g_rate; }
            tx_pos++;
            if (v >  0.999) v =  0.999;
            if (v < -0.999) v = -0.999;
            out[i] = (int16_t)lrint(v * 32767.0);
            continue;
        }
        tx_dtmf_done = 1;
        if (ring_r != ring_w) { out[i] = ring[ring_r]; ring_r = (ring_r + 1) % RING_N; }
        else                    out[i] = 0;
    }
    return noErr;
}

/* ------------------------------------------------------------------ */
/* CoreAudio device                                                   */
/* ------------------------------------------------------------------ */

static AudioObjectID find_device(void)
{
    AudioObjectPropertyAddress a = { kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
    UInt32 z = 0;
    AudioObjectID found = kAudioObjectUnknown, *d;
    int n;

    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &a, 0, NULL, &z) != noErr)
        return found;
    n = z / sizeof(AudioObjectID);
    d = malloc(z);
    AudioObjectGetPropertyData(kAudioObjectSystemObject, &a, 0, NULL, &z, d);
    for (int i = 0; i < n; i++) {
        CFStringRef s = NULL;
        UInt32 zz = sizeof s;
        AudioObjectPropertyAddress na = { kAudioObjectPropertyName,
            kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
        if (AudioObjectGetPropertyData(d[i], &na, 0, NULL, &zz, &s) != noErr || !s)
            continue;
        char b[256] = { 0 };
        CFStringGetCString(s, b, sizeof b, kCFStringEncodingUTF8);
        CFRelease(s);
        if (strstr(b, "Modem")) { found = d[i]; break; }
    }
    free(d);
    return found;
}

static int audio_start(void)
{
    AudioObjectID dev = find_device();
    AudioObjectPropertyAddress sra = { kAudioDevicePropertyNominalSampleRate,
        kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
    Float64 want = g_rate, got = 0;
    UInt32 z, one = 1, slice = 4096;
    AudioComponentDescription cd = { kAudioUnitType_Output, kAudioUnitSubType_HALOutput,
                                     kAudioUnitManufacturer_Apple, 0, 0 };
    AudioComponent comp;
    AudioStreamBasicDescription f;
    AURenderCallbackStruct icb = { input_cb, NULL }, ocb = { output_cb, NULL };
    OSStatus rc;

    if (dev == kAudioObjectUnknown) {
        fprintf(stderr, "modem audio device not found -- was the configuration set?\n");
        return -1;
    }
    AudioObjectSetPropertyData(dev, &sra, 0, NULL, sizeof want, &want);
    for (int i = 0; i < 50; i++) {                /* the change is asynchronous */
        z = sizeof got;
        AudioObjectGetPropertyData(dev, &sra, 0, NULL, &z, &got);
        if (fabs(got - want) < 1.0) break;
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, true);
    }
    if (fabs(got - want) >= 1.0) {
        fprintf(stderr, "nominal rate did not take (asked %.0f, is %.0f)\n", want, got);
        return -1;
    }
    comp = AudioComponentFindNext(NULL, &cd);
    if (!comp || AudioComponentInstanceNew(comp, &au) != noErr) {
        fprintf(stderr, "could not create HAL audio unit\n"); return -1;
    }
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input,  1, &one, sizeof one);
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0, &one, sizeof one);
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &dev, sizeof dev);

    memset(&f, 0, sizeof f);
    f.mSampleRate = got;
    f.mFormatID = kAudioFormatLinearPCM;
    f.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    f.mBitsPerChannel = 16;
    f.mChannelsPerFrame = 1;
    f.mFramesPerPacket = 1;
    f.mBytesPerFrame = 2;
    f.mBytesPerPacket = 2;
    /* Capture format on the OUTPUT scope of element 1, transmit format on the
     * INPUT scope of element 0 -- opposite scopes on different elements. */
    if (AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Output, 1, &f, sizeof f) != noErr ||
        AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Input, 0, &f, sizeof f) != noErr) {
        fprintf(stderr, "could not set 16-bit mono @ %.0f both ways\n", got); return -1;
    }
    AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice,
                         kAudioUnitScope_Global, 0, &slice, sizeof slice);
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_SetInputCallback,
                         kAudioUnitScope_Global, 0, &icb, sizeof icb);
    AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback,
                         kAudioUnitScope_Input, 0, &ocb, sizeof ocb);

    in_abl = calloc(1, sizeof(AudioBufferList) + sizeof(AudioBuffer));
    in_abl->mBuffers[0].mData = calloc(slice * 2 + 64, 1);

    if ((rc = AudioUnitInitialize(au)) != noErr) {
        fprintf(stderr, "AudioUnitInitialize: %d\n", (int)rc); return -1;
    }
    if ((rc = AudioOutputUnitStart(au)) != noErr) {
        fprintf(stderr, "AudioOutputUnitStart: %d\n", (int)rc); return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Offline replay -- the way to work on this with no line             */
/* ------------------------------------------------------------------ */

static int replay(const char *path)
{
    FILE *fp = fopen(path, "rb");
    int16_t buf[80];
    long total = 0;

    if (!fp) { perror(path); return 1; }
    /* Goes through engine_feed() exactly as the live path does -- the DC
     * blocker, both resamplers, the answer detector and the 80-sample blocking.
     * An earlier version duplicated a shortened version of that here and fed
     * the file straight to me_rx_audio(): at a device rate of 9600 that handed
     * 9600 Hz samples to an 8000 Hz entry point, and it "passed" only because
     * the answer detector was mis-tuned by the same ratio.  Two wrongs.  The tap
     * must be at --rate, since that is what the device produced. */
    for (;;) {
        size_t n = fread(buf, 2, 80, fp);
        if (n == 0) break;
        engine_feed(buf, (long)n);
        ring_r = ring_w;                 /* no line to transmit into */
        total += (long)n;
    }
    fclose(fp);
    fprintf(stderr, "[APPLE] replay: %ld samples at %.0f Hz (%.2f s), engine %s\n",
            total, g_rate, total / g_rate, g_engine_running ? "ran" : "NEVER STARTED");
    return g_engine_running ? 0 : 1;
}

/* ------------------------------------------------------------------ */

/* 9600 -> 8000 is 5/6, 9600 -> 16000 is 5/3, 8000 -> 9600 is 6/5.  All exact. */
static void rs_setup(void)
{
    if (g_rate == 8000.0) return;
    if (g_rate != 9600.0) {
        fprintf(stderr, "[APPLE] %.0f Hz has no small exact ratio to 8000 or "
                        "16000; use 9600 (or 8000 for V.34 only)\n", g_rate);
        exit(2);
    }
    rs_init(&rs_8k,  5, 6);
    rs_init(&rs_16k, 5, 3);
    rs_init(&rs_tx,  6, 5);
}

/* The resamplers are load-bearing, so they are measurable without a device:
 * a tone at the device rate through each path, reporting the recovered
 * frequency, the level and the residual after the ideal tone is subtracted. */
static int selftest(void)
{
    struct { const char *name; int L, M; double out_rate; } c[] = {
        { "9600 -> 8000  (me_rx_audio)",     5, 6, 8000.0  },
        { "9600 -> 16000 (me_rx_v90a_16k)",  5, 3, 16000.0 },
        { "8000 -> 9600  (transmit)",        6, 5, 9600.0  },
    };
    const double in_rate[3] = { 9600.0, 9600.0, 8000.0 };
    int bad = 0;

    for (int t = 0; t < 3; t++) {
        /* 3673 Hz is V.34's worst case: 3429 baud at fc 1959 reaches it, and it
         * is 327 Hz from the 8 kHz grid's Nyquist, so it is the frequency that
         * decides whether this bearer can carry the top symbol rate. */
        static const double freqs[] = { 300, 1000, 2000, 3000, 3520, 3673 };
        for (unsigned fi = 0; fi < sizeof freqs / sizeof freqs[0]; fi++) {
            double f = freqs[fi];
            struct resamp r;
            double *y = malloc(sizeof(double) * 200000);
            long ny = 0;
            long nin = (long)(in_rate[t] * 0.5);

            rs_init(&r, c[t].L, c[t].M);
            for (long i = 0; i < nin; i++) {
                double o[8];
                int got = rs_put(&r, 10000.0 * sin(2 * M_PI * f * i / in_rate[t]), o, 8);
                for (int j = 0; j < got; j++) y[ny++] = o[j];
            }
            /* Skip the filter's transient, then fit amplitude and phase at f and
             * report what is left: a resampler wrong in gain, in phase slope or
             * in aliasing all show up in the residual. */
            long s0 = 4 * RS_TAPS * c[t].M / c[t].L + 64, n = ny - s0 - 64;
            double sc = 0, ss = 0, e = 0, pw = 0;
            for (long i = 0; i < n; i++) {
                double ph = 2 * M_PI * f * i / c[t].out_rate;
                sc += y[s0 + i] * cos(ph);
                ss += y[s0 + i] * sin(ph);
                pw += y[s0 + i] * y[s0 + i];
            }
            sc *= 2.0 / n; ss *= 2.0 / n;
            double amp = sqrt(sc * sc + ss * ss);
            for (long i = 0; i < n; i++) {
                double ph = 2 * M_PI * f * i / c[t].out_rate;
                double fit = sc * cos(ph) + ss * sin(ph);
                e += (y[s0 + i] - fit) * (y[s0 + i] - fit);
            }
            double sndr = 10 * log10((pw / n) / fmax(1e-12, e / n));
            int bad_row = fabs(amp / 10000.0 - 1.0) > 0.03 || sndr < 40.0;
            printf("  %-34s %4.0f Hz: gain %6.4f  SNDR %5.1f dB%s\n",
                   fi == 0 ? c[t].name : "", f, amp / 10000.0, sndr,
                   bad_row ? "   <- BAD" : "");
            bad += bad_row;
            free(y);
        }
    }
    printf("%s\n", bad ? "SELFTEST FAILED" : "selftest ok");
    return bad ? 1 : 0;
}

static void on_signal(int sig) { (void)sig; g_stop = 1; }

static void usage(const char *a0)
{
    fprintf(stderr,
        "usage: %s --dial NUMBER [--pty-link PATH] [--rate HZ] [--hold SECS]\n"
        "       %s --rx-replay TAP.s16 [--pty-link PATH]\n"
        "       %s --hook on|off\n"
        "       %s --selftest\n"
        "\n"
        "  --dial       seize the line, DTMF the number, wait for 2100 Hz, run\n"
        "               the engine.  Goes back on-hook on exit or SIGINT\n"
        "  --rx-replay  run the analogue side offline against a recorded tap;\n"
        "               needs no line and no device\n"
        "  --hook       line control only, then exit\n"
        "  --rate       9600 (default) reaches both engine grids exactly, 5/6 to\n"
        "               8000 and 5/3 to 16000; 8000 skips the receive resampler\n"
        "               but cannot feed the T/2 path at all.  See the header\n"
        "  --selftest   measure the resamplers; needs no device and no line\n"
        "\n"
        "APPLE_MODEM_TX_AMP per-tone DTMF amplitude (default 0.15)\n"
        "APPLE_MODEM_DTMF_MS on/off times in ms (default 100/120)\n"
        "APPLE_MODEM_TX_GAIN engine transmit gain into the DAA (default 1.0)\n",
        a0, a0, a0, a0);
}

int main(int argc, char **argv)
{
    const char *hook_arg = NULL;
    double hold = 60.0, on_ms = 100.0, off_ms = 120.0;
    int rc = 1;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dial") && i + 1 < argc)            g_dial = argv[++i];
        else if (!strcmp(argv[i], "--pty-link") && i + 1 < argc)   g_pty_link = argv[++i];
        else if (!strcmp(argv[i], "--rx-replay") && i + 1 < argc)   g_replay_path = argv[++i];
        else if (!strcmp(argv[i], "--hook") && i + 1 < argc)        hook_arg = argv[++i];
        else if (!strcmp(argv[i], "--rate") && i + 1 < argc)        g_rate = atof(argv[++i]);
        else if (!strcmp(argv[i], "--hold") && i + 1 < argc)        hold = atof(argv[++i]);
        else if (!strcmp(argv[i], "--selftest"))                    return selftest();
        else { usage(argv[0]); return 2; }
    }
    if (getenv("APPLE_MODEM_TX_AMP"))  tx_amp = atof(getenv("APPLE_MODEM_TX_AMP"));
    if (getenv("APPLE_MODEM_TX_GAIN")) g_tx_gain = atof(getenv("APPLE_MODEM_TX_GAIN"));
    if (getenv("APPLE_MODEM_DTMF_MS")) {
        const char *m = getenv("APPLE_MODEM_DTMF_MS"), *sl = strchr(m, '/');
        on_ms = atof(m);
        if (sl) off_ms = atof(sl + 1);
    }

    if (hook_arg) {
        if (usb_open_line() < 0) return 1;
        rc = line_hook(!strcmp(hook_arg, "on")) < 0 ? 1 : 0;
        libusb_close(usb);
        libusb_exit(NULL);
        return rc;
    }

    if (g_replay_path) {
        rs_setup();
        me_set_verbose(1);
        me_init();
        me_set_law(ME_LAW_ULAW);
        if (g_pty_link && di_open(g_pty_link) < 0) {
            fprintf(stderr, "failed to open PTY %s\n", g_pty_link);
            return 1;
        }
        me_dial("replay");
        rc = replay(g_replay_path);
        me_destroy();
        return rc;
    }

    if (!g_dial) { usage(argv[0]); return 2; }
    rs_setup();
    if (g_rate == 8000.0)
        fprintf(stderr, "[APPLE] 8000 Hz: no receive resampler, and NO T/2 path "
                        "-- the V.90 analogue role cannot run\n");

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (usb_open_line() < 0) return 1;
    if (build_dial_script(g_dial, on_ms, off_ms) < 0) return 2;

    me_set_verbose(1);
    me_init();
    me_set_law(ME_LAW_ULAW);
    if (g_pty_link && di_open(g_pty_link) < 0) {
        fprintf(stderr, "failed to open PTY %s\n", g_pty_link);
        me_destroy();
        return 1;
    }
    me_dial(g_dial);

    if (line_hook(1) < 0) {                  /* off-hook; refuses with no pair */
        me_destroy();
        libusb_close(usb);
        libusb_exit(NULL);
        return 1;
    }
    if (audio_start() < 0) {
        line_hook(0);
        me_destroy();
        libusb_close(usb);
        libusb_exit(NULL);
        return 1;
    }
    fprintf(stderr, "[APPLE] dialling %s, %d DTMF segments at %.0f/%.0f ms\n",
            g_dial, tx_nseg, on_ms, off_ms);

    for (double t = 0; t < hold && !g_stop; t += 0.25)
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.25, false);

    AudioOutputUnitStop(au);
    AudioUnitUninitialize(au);
    if (g_engine_running) me_on_sip_disconnected();
    me_destroy();
    fprintf(stderr, "[APPLE] render errors %d, engine %s, DTMF %s\n",
            render_errors, g_engine_running ? "ran" : "never started",
            tx_dtmf_done ? "completed" : "CUT SHORT");
    line_hook(0);                            /* always release the line */
    libusb_close(usb);
    libusb_exit(NULL);
    return g_engine_running ? 0 : 1;
}
