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
 * WHY THIS IS A BETTER BEARER THAN THE HSF PART, and it is the reason to prefer
 * it: the device's isochronous rate list contains 8000 Hz exactly, so the
 * engine is fed at its own DS0 rate with NO RESAMPLING AND NO SAMPLING-PHASE
 * CHOICE.  The HSF coupler streams 16 kHz and must decide where the 8 kHz grid
 * falls, and that decision is what cost that path a session -- swept as a pure
 * fractional delay, only 1/10, 2/10 and 9/10 of phases reached Phase 4 on three
 * recorded calls, and 0.0 (the obvious value) failed on all three.  There is no
 * equivalent knob here because there is no equivalent choice.
 *
 * THE LIMITATION THAT COMES WITH IT, stated because it bounds what this bearer
 * can ever do: me_rx_v90a_16k() wants two samples per DS0 interval, and this
 * device's rate ceiling is 10286 Hz, so the V.90 analogue Phase 3 downstream --
 * which can only be recovered from T/2 samples -- CANNOT be fed at 8000.  It is
 * reachable: 9600 Hz is in the rate list and 9600 * 5/3 = 16000 exactly, so an
 * exact rational resampler would supply it.  That is not implemented here, and
 * until it is, expect V.34 and below to work and the V.90 analogue role's
 * Phase 3 not to.  --rate says which rate to run at so the experiment is
 * available; only 8000 feeds the engine today.
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
static double g_rate = 8000.0;
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
#define ANS_BLOCK  160u            /* 20 ms at 8 kHz */
#define ANS_NEEDED  10u            /* 200 ms of 2100 Hz */
static int16_t ans_buf[ANS_BLOCK];
static unsigned ans_fill;

static void answer_block(const int16_t *s)
{
    double w = 2.0 * M_PI * 2100.0 / g_rate;
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

static void engine_feed(int16_t *s, long n)
{
    long off = 0;

    dc_block(s, n);
    while (off < n) {
        long k = n - off;
        if (k > 80) k = 80;
        if (!g_answered) {
            for (long i = 0; i < k; i++) {
                ans_buf[ans_fill++] = s[off + i];
                if (ans_fill == ANS_BLOCK) { answer_block(ans_buf); ans_fill = 0; }
            }
        }
        if (g_answered && !g_engine_running) {
            g_engine_running = 1;
            me_on_sip_connected();
            fprintf(stderr, "[APPLE] starting analogue engine at %.0f Hz\n", g_rate);
        }
        if (g_engine_running) {
            int16_t out[80];
            me_rx_audio(s + off, (int)k);
            me_tx_audio(out, (int)k);
            for (long i = 0; i < k; i++) {
                unsigned nw = (ring_w + 1) % RING_N;
                if (nw == ring_r) break;          /* consumer behind; drop */
                double v = out[i] * g_tx_gain;
                if (v >  32767.0) v =  32767.0;
                if (v < -32768.0) v = -32768.0;
                ring[ring_w] = (int16_t)lrint(v);
                ring_w = nw;
            }
        }
        off += k;
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
    int16_t buf[80], out[80];
    long total = 0;

    if (!fp) { perror(path); return 1; }
    /* A recorded tap is the line as it arrived, so the answer detector runs on
     * it exactly as live -- a replay that skips straight to the engine cannot
     * reproduce a call that failed in answer detection. */
    for (;;) {
        size_t n = fread(buf, 2, 80, fp);
        if (n == 0) break;
        dc_block(buf, (long)n);
        if (!g_answered) {
            for (size_t i = 0; i < n; i++) {
                ans_buf[ans_fill++] = buf[i];
                if (ans_fill == ANS_BLOCK) { answer_block(ans_buf); ans_fill = 0; }
            }
        }
        if (g_answered && !g_engine_running) {
            g_engine_running = 1;
            me_on_sip_connected();
            fprintf(stderr, "[APPLE] replay: engine started at sample %ld (%.2f s)\n",
                    total, total / g_rate);
        }
        if (g_engine_running) {
            me_rx_audio(buf, (int)n);
            me_tx_audio(out, (int)n);
        }
        total += (long)n;
    }
    fclose(fp);
    fprintf(stderr, "[APPLE] replay: %ld samples (%.2f s), engine %s\n",
            total, total / g_rate, g_engine_running ? "ran" : "NEVER STARTED");
    return g_engine_running ? 0 : 1;
}

/* ------------------------------------------------------------------ */

static void on_signal(int sig) { (void)sig; g_stop = 1; }

static void usage(const char *a0)
{
    fprintf(stderr,
        "usage: %s --dial NUMBER [--pty-link PATH] [--rate HZ] [--hold SECS]\n"
        "       %s --rx-replay TAP.s16 [--pty-link PATH]\n"
        "       %s --hook on|off\n"
        "\n"
        "  --dial       seize the line, DTMF the number, wait for 2100 Hz, run\n"
        "               the engine.  Goes back on-hook on exit or SIGINT\n"
        "  --rx-replay  run the analogue side offline against a recorded tap;\n"
        "               needs no line and no device\n"
        "  --hook       line control only, then exit\n"
        "  --rate       8000 (default) is the only rate that feeds the engine;\n"
        "               see this file's header on 9600 and the missing T/2 path\n"
        "\n"
        "APPLE_MODEM_TX_AMP per-tone DTMF amplitude (default 0.15)\n"
        "APPLE_MODEM_DTMF_MS on/off times in ms (default 100/120)\n"
        "APPLE_MODEM_TX_GAIN engine transmit gain into the DAA (default 1.0)\n",
        a0, a0, a0);
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
    if (g_rate != 8000.0)
        fprintf(stderr, "[APPLE] WARNING: only 8000 Hz feeds the engine; "
                        "%.0f will run the audio and not the modem\n", g_rate);

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
