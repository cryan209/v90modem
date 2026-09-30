/* apple_usb_modem_audio -- codec side of the Apple USB Modem (A1082),
 * USB 05ac:1401, through CoreAudio.
 *
 * Once the device's configuration is set (tools/apple_usb_modem_probe.c
 * --configure), macOS's usbaudiod claims the two AudioStreaming interfaces and
 * the modem becomes an ordinary 1-in/1-out CoreAudio device.  So the codec
 * needs no vendor bring-up at all, unlike the Conexant part in
 * docs/hsf_usb_daa.md -- there is no script to run first.
 *
 * The rate list is the point:
 *
 *     7200 8000 8229 8400 9000 9600 10286 Hz
 *
 * 8 kHz plus every V.34 symbol rate times three (2400/2743/2800/3000/3200/3429
 * x 3), i.e. the host's own T/3 grid per symbol rate.  9600 is what this tree's
 * T/3 upstream receiver already runs at for 3200 baud.
 *
 * TRAPS:
 *
 *  - Denied microphone access delivers digital SILENCE, which looks exactly
 *    like a perfect noise floor.  This tool prints the TCC authorization
 *    status and refuses to report statistics for a capture that holds only one
 *    distinct sample value.  Granting permission mid-run leaves that run with
 *    no frames; re-run.
 *  - Setting the nominal rate is asynchronous: AudioObjectSetPropertyData
 *    returns success and the device is still on the old rate for a few tens of
 *    ms.  Poll until it takes.
 *  - Initialise min/max from the first sample.  Starting max at 0 on
 *    all-negative data reports max = 0 and invents a zero sample.
 *
 * THE ANALOGUE PATH MUST BE TURNED ON FIRST, and setting the configuration is
 * not enough.  With register 5 at 0 the input is a CONSTANT at negative full
 * scale, varying with sample rate (-30069 at 7200, -31703 at 8000, -32768 at
 * 9600 and above) -- a fill pattern would not vary with rate, so a real ADC and
 * filter chain are running with their input at a rail.  Two bits matter:
 *
 *     ./apple_usb_modem_probe --monitor on    (reg 5 bit 3: on-hook monitor)
 *     ./apple_usb_modem_probe --hook on       (reg 5 bit 0: OFF-HOOK)
 *
 * Measured at 9600 Hz against a VG224 FXS port: monitor only gives -38 dBFS of
 * which 90% is mains hum below 300 Hz and no dial tone; off-hook gives
 * -15.9 dBFS with 350.0 + 440.0 Hz at equal level carrying 64% of the power
 * (North American dial tone) and the hum 53 dB down, because loop current drops
 * the line impedance.
 *
 * So a railed capture means register 5 is 0, not that the line is missing --
 * and with no line at all, bit 0 rails it too, because there is nothing to draw
 * current from.  Read the probe's line sense (register 0x1d: 0x00 = no pair,
 * ~217 = on-hook, ~250 = off-hook) before reading anything into a capture.
 *
 * -63.8 dBFS, the monitor's reading with no line, is the codec's own floor.  It
 * is not a measurement of a bearer.
 *
 * Build: make apple_usb_modem_audio
 */

#import <AVFoundation/AVFoundation.h>
#import <AudioToolbox/AudioToolbox.h>
#import <CoreAudio/CoreAudio.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static AudioUnit au;
static int16_t *acc;
static long acc_n, acc_cap;
static AudioBufferList *abl;
static int render_errors;

/* ---- transmit ----------------------------------------------------------
 * The output stream is the other half of the same CoreAudio device, so a
 * transmit path is a render callback on element 0 of the same HAL unit that
 * element 1 captures with.  Running both at once is the point: what proves a
 * digit reached the line is the far end's reaction to it, and that arrives on
 * the receive side while we are still transmitting.
 *
 * The script is a flat list of (f1, f2, samples); either frequency may be 0 for
 * silence, and phase is carried per segment so a tone starts at zero. */
#define TX_MAX_SEG 256
struct tx_seg { double f1, f2; long n; };
static struct tx_seg tx_script[TX_MAX_SEG];
static int tx_nseg, tx_seg;
static long tx_pos;              /* samples emitted inside the current segment */
static double tx_ph1, tx_ph2;    /* radians, reset at each segment boundary */
static double tx_amp = 0.15;     /* per tone, so a pair is about -16.5 dBFS */
static double tx_rate = 9600.0;
static long tx_done;             /* total samples emitted */
static int tx_underruns;

static OSStatus output_cb(void *ref, AudioUnitRenderActionFlags *flags,
                          const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
                          AudioBufferList *io)
{
    int16_t *out = io->mBuffers[0].mData;

    (void)ref; (void)flags; (void)ts; (void)bus;
    for (UInt32 i = 0; i < nframes; i++) {
        double v = 0;
        while (tx_seg < tx_nseg && tx_pos >= tx_script[tx_seg].n) {
            tx_seg++; tx_pos = 0; tx_ph1 = tx_ph2 = 0;
        }
        if (tx_seg < tx_nseg) {
            struct tx_seg *g = &tx_script[tx_seg];
            if (g->f1 > 0) { v += tx_amp * sin(tx_ph1); tx_ph1 += 2 * M_PI * g->f1 / tx_rate; }
            if (g->f2 > 0) { v += tx_amp * sin(tx_ph2); tx_ph2 += 2 * M_PI * g->f2 / tx_rate; }
            tx_pos++;
            tx_done++;
        } else {
            tx_underruns++;   /* script exhausted; emit silence, not garbage */
        }
        if (v >  0.999) v =  0.999;
        if (v < -0.999) v = -0.999;
        out[i] = (int16_t)lrint(v * 32767.0);
    }
    return noErr;
}

/* Q.23 DTMF.  Low group selects the row, high group the column. */
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
    tx_script[tx_nseg].n = (long)(tx_rate * ms / 1000.0);
    tx_nseg++;
}

static OSStatus input_cb(void *ref, AudioUnitRenderActionFlags *flags,
                         const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
                         AudioBufferList *unused)
{
    OSStatus rc;
    int16_t *s;

    (void)ref;
    (void)unused;
    abl->mNumberBuffers = 1;
    abl->mBuffers[0].mNumberChannels = 1;
    abl->mBuffers[0].mDataByteSize = nframes * 2;
    rc = AudioUnitRender(au, flags, ts, bus, nframes, abl);
    if (rc != noErr) {
        if (render_errors++ < 3)
            fprintf(stderr, "  AudioUnitRender: %d (nframes=%u)\n", (int)rc, (unsigned)nframes);
        return rc;
    }
    s = abl->mBuffers[0].mData;
    for (UInt32 i = 0; i < nframes && acc_n < acc_cap; i++) acc[acc_n++] = s[i];
    return noErr;
}

static char *copy_name(AudioObjectID d, AudioObjectPropertySelector sel)
{
    CFStringRef s = NULL;
    UInt32 z = sizeof s;
    AudioObjectPropertyAddress a = { sel, kAudioObjectPropertyScopeGlobal,
                                     kAudioObjectPropertyElementMain };
    if (AudioObjectGetPropertyData(d, &a, 0, NULL, &z, &s) != noErr || !s) return NULL;
    CFIndex n = CFStringGetMaximumSizeForEncoding(CFStringGetLength(s), kCFStringEncodingUTF8) + 1;
    char *b = malloc(n);
    CFStringGetCString(s, b, n, kCFStringEncodingUTF8);
    CFRelease(s);
    return b;
}

static AudioObjectID find_device(int verbose)
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
        char *name = copy_name(d[i], kAudioObjectPropertyName);
        if (!name || !strstr(name, "Modem")) { free(name); continue; }
        found = d[i];
        if (verbose) {
            char *uid = copy_name(d[i], kAudioDevicePropertyDeviceUID);
            Float64 sr = 0;
            printf("device id=%u name=\"%s\"\n  uid=%s\n", d[i], name, uid ? uid : "?");
            free(uid);
            a = (AudioObjectPropertyAddress){ kAudioDevicePropertyNominalSampleRate,
                kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
            z = sizeof sr;
            if (AudioObjectGetPropertyData(d[i], &a, 0, NULL, &z, &sr) == noErr)
                printf("  current nominal rate: %.0f Hz\n", sr);
            a.mSelector = kAudioDevicePropertyAvailableNominalSampleRates;
            if (AudioObjectGetPropertyDataSize(d[i], &a, 0, NULL, &z) == noErr) {
                int m = z / sizeof(AudioValueRange);
                AudioValueRange *r = malloc(z);
                AudioObjectGetPropertyData(d[i], &a, 0, NULL, &z, r);
                printf("  available rates (%d):", m);
                for (int k = 0; k < m; k++)
                    if (r[k].mMinimum == r[k].mMaximum) printf(" %.0f", r[k].mMinimum);
                    else printf(" %.0f-%.0f", r[k].mMinimum, r[k].mMaximum);
                printf("\n");
                free(r);
            }
            for (int scope = 0; scope < 2; scope++) {
                a = (AudioObjectPropertyAddress){ kAudioDevicePropertyStreamConfiguration,
                    scope ? kAudioObjectPropertyScopeOutput : kAudioObjectPropertyScopeInput,
                    kAudioObjectPropertyElementMain };
                if (AudioObjectGetPropertyDataSize(d[i], &a, 0, NULL, &z) != noErr) continue;
                AudioBufferList *bl = malloc(z);
                AudioObjectGetPropertyData(d[i], &a, 0, NULL, &z, bl);
                UInt32 ch = 0;
                for (UInt32 k = 0; k < bl->mNumberBuffers; k++)
                    ch += bl->mBuffers[k].mNumberChannels;
                printf("  %s channels: %u\n", scope ? "output" : "input", ch);
                free(bl);
            }
        }
        free(name);
        break;
    }
    free(d);
    return found;
}

static int check_permission(void)
{
    AVAuthorizationStatus st = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeAudio];
    static const char *names[] = { "notDetermined", "restricted", "DENIED", "authorized" };
    printf("microphone TCC status: %s\n",
           (st >= 0 && st <= AVAuthorizationStatusAuthorized) ? names[st] : "?");
    if (st == AVAuthorizationStatusNotDetermined) {
        __block BOOL done = NO, ok = NO;
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeAudio
                                 completionHandler:^(BOOL granted){ ok = granted; done = YES; }];
        for (int i = 0; i < 300 && !done; i++)
            CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, true);
        printf("permission request -> %s\n", ok ? "granted" : "not granted");
        printf("note: a run that granted permission usually captures nothing; re-run\n");
    }
    return st == AVAuthorizationStatusDenied ? 1 : 0;
}

static int capture(double rate, double secs, const char *outpath)
{
    AudioObjectID dev;
    AudioObjectPropertyAddress sra = { kAudioDevicePropertyNominalSampleRate,
        kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
    Float64 want = rate, got = 0;
    UInt32 z, one = 1, zero = 0, slice = 4096;
    AudioComponentDescription cd = { kAudioUnitType_Output, kAudioUnitSubType_HALOutput,
                                     kAudioUnitManufacturer_Apple, 0, 0 };
    AudioComponent comp;
    AudioStreamBasicDescription f;
    AURenderCallbackStruct cb = { input_cb, NULL };
    OSStatus rc;

    check_permission();
    dev = find_device(0);
    if (dev == kAudioObjectUnknown) { fprintf(stderr, "modem audio device not found\n"); return 1; }

    rc = AudioObjectSetPropertyData(dev, &sra, 0, NULL, sizeof want, &want);
    /* Asynchronous: poll until it takes. */
    for (int i = 0; i < 50; i++) {
        z = sizeof got;
        AudioObjectGetPropertyData(dev, &sra, 0, NULL, &z, &got);
        if (fabs(got - want) < 1.0) break;
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, true);
    }
    printf("nominal rate: requested %.0f, now %.0f%s\n", rate, got,
           fabs(got - want) < 1.0 ? "" : "   <- did NOT take");
    if (rc != noErr && fabs(got - want) >= 1.0) return 1;

    comp = AudioComponentFindNext(NULL, &cd);
    if (!comp || AudioComponentInstanceNew(comp, &au) != noErr) {
        fprintf(stderr, "could not create HAL audio unit\n"); return 1;
    }
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input,  1, &one,  sizeof one);
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0, &zero, sizeof zero);
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
    if (AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Output, 1, &f, sizeof f) != noErr) {
        fprintf(stderr, "could not set 16-bit mono @ %.0f\n", got); return 1;
    }
    AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice,
                         kAudioUnitScope_Global, 0, &slice, sizeof slice);
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_SetInputCallback,
                         kAudioUnitScope_Global, 0, &cb, sizeof cb);

    acc_cap = (long)(got * secs * 1.5) + 8192;
    acc = calloc(acc_cap, 2);
    abl = calloc(1, sizeof(AudioBufferList) + sizeof(AudioBuffer));
    abl->mBuffers[0].mData = calloc(slice * 2 + 64, 1);

    if ((rc = AudioUnitInitialize(au)) != noErr) {
        fprintf(stderr, "AudioUnitInitialize: %d\n", (int)rc); return 1;
    }
    if ((rc = AudioOutputUnitStart(au)) != noErr) {
        fprintf(stderr, "AudioOutputUnitStart: %d\n", (int)rc); return 1;
    }
    printf("capturing %.2f s ...\n", secs);
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, secs, false);
    AudioOutputUnitStop(au);
    AudioUnitUninitialize(au);

    printf("frames: %ld (%.2f s at %.0f Hz), render errors: %d\n",
           acc_n, acc_n / got, got, render_errors);
    if (acc_n == 0) { printf("NO SAMPLES -- nothing to measure\n"); return 1; }

    /* Statistics.  min/max start from the first sample, not from zero. */
    double sum = 0, sq = 0;
    long nonzero = 0, distinct = 1;
    int16_t mn = acc[0], mx = acc[0], first = acc[0];
    for (long i = 0; i < acc_n; i++) {
        int16_t s = acc[i];
        sum += s; sq += (double)s * s;
        if (s) nonzero++;
        if (s < mn) mn = s;
        if (s > mx) mx = s;
        if (s != first && distinct == 1) distinct = 2;
    }
    double mean = sum / acc_n;
    double rms  = sqrt(sq / acc_n);
    double ac   = sqrt(fmax(0.0, sq / acc_n - mean * mean));

    printf("nonzero samples: %ld of %ld (%.1f%%)\n", nonzero, acc_n, 100.0 * nonzero / acc_n);
    printf("min/max: %d / %d\n", mn, mx);
    if (distinct == 1) {
        printf("*** ONE DISTINCT VALUE (%d) -- this is NOT a noise-floor measurement.\n", first);
        printf("*** Either the analogue path is unpowered/railed, microphone access is\n");
        printf("*** denied, or the stream is not running.  See this file's header.\n");
    } else {
        printf("DC offset (mean): %.2f counts (%.2f%% of full scale)\n", mean, 100.0 * mean / 32768.0);
        printf("RMS incl. DC: %.2f = %.1f dBFS\n", rms, 20 * log10(fmax(1e-9, rms) / 32768.0));
        printf("RMS excl. DC: %.2f = %.1f dBFS   <- noise floor\n",
               ac, 20 * log10(fmax(1e-9, ac) / 32768.0));
        printf("effective bits of noise: %.1f\n", log2(fmax(1.0, ac)));
    }
    if (outpath) {
        FILE *fp = fopen(outpath, "wb");
        if (fp) { fwrite(acc, 2, acc_n, fp); fclose(fp);
                  printf("raw signed 16-bit LE written: %s\n", outpath); }
        else perror(outpath);
    }
    return 0;
}

/* Shared setup: both directions on one HAL unit at one rate. */
static int run_duplex(double rate, double secs, const char *outpath, int transmit)
{
    AudioObjectID dev;
    AudioObjectPropertyAddress sra = { kAudioDevicePropertyNominalSampleRate,
        kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
    Float64 want = rate, got = 0;
    UInt32 z, one = 1, zero = 0, slice = 4096;
    AudioComponentDescription cd = { kAudioUnitType_Output, kAudioUnitSubType_HALOutput,
                                     kAudioUnitManufacturer_Apple, 0, 0 };
    AudioComponent comp;
    AudioStreamBasicDescription f;
    AURenderCallbackStruct icb = { input_cb, NULL }, ocb = { output_cb, NULL };
    OSStatus rc;

    check_permission();
    dev = find_device(0);
    if (dev == kAudioObjectUnknown) { fprintf(stderr, "modem audio device not found\n"); return 1; }

    AudioObjectSetPropertyData(dev, &sra, 0, NULL, sizeof want, &want);
    for (int i = 0; i < 50; i++) {
        z = sizeof got;
        AudioObjectGetPropertyData(dev, &sra, 0, NULL, &z, &got);
        if (fabs(got - want) < 1.0) break;
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.1, true);
    }
    if (fabs(got - want) >= 1.0) {
        fprintf(stderr, "nominal rate did not take (asked %.0f, is %.0f)\n", want, got);
        return 1;
    }
    tx_rate = got;

    comp = AudioComponentFindNext(NULL, &cd);
    if (!comp || AudioComponentInstanceNew(comp, &au) != noErr) {
        fprintf(stderr, "could not create HAL audio unit\n"); return 1;
    }
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input,  1, &one, sizeof one);
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0,
                         transmit ? &one : &zero, sizeof one);
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
    /* Capture format is what element 1 GIVES us; transmit format is what we
     * GIVE element 0 -- opposite scopes on different elements, and mixing them
     * up yields a -10868 that reads like an unsupported rate. */
    if (AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Output, 1, &f, sizeof f) != noErr) {
        fprintf(stderr, "could not set capture format 16-bit mono @ %.0f\n", got); return 1;
    }
    if (transmit &&
        AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Input, 0, &f, sizeof f) != noErr) {
        fprintf(stderr, "could not set transmit format 16-bit mono @ %.0f\n", got); return 1;
    }
    AudioUnitSetProperty(au, kAudioUnitProperty_MaximumFramesPerSlice,
                         kAudioUnitScope_Global, 0, &slice, sizeof slice);
    AudioUnitSetProperty(au, kAudioOutputUnitProperty_SetInputCallback,
                         kAudioUnitScope_Global, 0, &icb, sizeof icb);
    if (transmit)
        AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback,
                             kAudioUnitScope_Input, 0, &ocb, sizeof ocb);

    acc_cap = (long)(got * secs * 1.5) + 8192;
    acc = calloc(acc_cap, 2);
    abl = calloc(1, sizeof(AudioBufferList) + sizeof(AudioBuffer));
    abl->mBuffers[0].mData = calloc(slice * 2 + 64, 1);

    if ((rc = AudioUnitInitialize(au)) != noErr) {
        fprintf(stderr, "AudioUnitInitialize: %d\n", (int)rc); return 1;
    }
    if ((rc = AudioOutputUnitStart(au)) != noErr) {
        fprintf(stderr, "AudioOutputUnitStart: %d\n", (int)rc); return 1;
    }
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, secs, false);
    AudioOutputUnitStop(au);
    AudioUnitUninitialize(au);

    printf("captured %ld frames (%.2f s at %.0f Hz), render errors %d\n",
           acc_n, acc_n / got, got, render_errors);
    if (transmit) {
        long want_n = 0;
        for (int i = 0; i < tx_nseg; i++) want_n += tx_script[i].n;
        printf("transmitted %ld of %ld scripted samples (%.2f s)%s\n",
               tx_done, want_n, tx_done / got,
               tx_done < want_n ? "   <- CUT SHORT, raise the duration" : "");
    }
    if (outpath && acc_n) {
        FILE *fp = fopen(outpath, "wb");
        if (fp) { fwrite(acc, 2, acc_n, fp); fclose(fp);
                  printf("raw signed 16-bit LE written: %s\n", outpath); }
        else perror(outpath);
    }
    return acc_n ? 0 : 1;
}

/* dial <rate> <digits> [out.s16] -- DTMF out, line audio in, at once. */
static int dial(double rate, const char *digits, const char *outpath,
                double on_ms, double off_ms)
{
    double lo, hi, total = 300.0;    /* 300 ms of leading silence */

    tx_rate = rate;
    tx_add(0, 0, 300.0);
    for (const char *p = digits; *p; p++) {
        char c = *p;
        if (c == ',' || c == ' ') { tx_add(0, 0, 500.0); total += 500.0; continue; }
        if (c >= 'a' && c <= 'd') c -= 32;
        if (dtmf_pair(c, &lo, &hi) < 0) {
            fprintf(stderr, "not a DTMF digit: '%c'\n", *p); return 1;
        }
        tx_add(lo, hi, on_ms);
        tx_add(0, 0, off_ms);
        total += on_ms + off_ms;
    }
    printf("dialling \"%s\": %d segments, %.0f ms of DTMF at %.0f Hz, "
           "%.0f/%.0f ms on/off, amplitude %.3f per tone\n",
           digits, tx_nseg, total, rate, on_ms, off_ms, tx_amp);
    /* Keep listening well past the last digit: the far end's answer (dial tone
     * stopping, ringback, a modem's ANSam) is what says the digits landed. */
    return run_duplex(rate, total / 1000.0 + 6.0, outpath, 1);
}

/* tone <rate> <hz> <secs> -- a single tone out, capturing the whole time.  With
 * the line on-hook this still shows up in the receive stream through the
 * hybrid, so it tests the transmit path without involving the exchange. */
static int tone(double rate, double hz, double secs, const char *outpath)
{
    tx_rate = rate;
    tx_add(0, 0, 200.0);
    tx_add(hz, 0, secs * 1000.0);
    printf("transmitting %.1f Hz for %.2f s at amplitude %.3f\n", hz, secs, tx_amp);
    return run_duplex(rate, secs + 1.0, outpath, 1);
}

int main(int argc, char **argv)
{
    if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
        fprintf(stderr,
            "usage: %s list\n"
            "       %s capture <rate> <seconds> [out.s16]\n"
            "       %s tone    <rate> <hz> <seconds> [out.s16]\n"
            "       %s dial    <rate> <digits> [out.s16]\n"
            "\n"
            "rate must be one the device offers: 7200 8000 8229 8400 9000 9600 10286\n"
            "Run tools/apple_usb_modem_probe --configure first if the device is new\n"
            "to this boot, or CoreAudio will not see it at all; go off-hook with\n"
            "--hook on before dialling, and check its line-sense report.\n"
            "\n"
            "digits: 0-9 A-D * #, a comma or space for a 500 ms pause.\n"
            "APPLE_MODEM_TX_AMP sets the per-tone amplitude (default 0.15),\n"
            "APPLE_MODEM_DTMF_MS the on/off times (default 100/100).\n",
            argv[0], argv[0], argv[0], argv[0]);
        return 0;
    }
    {   /* transmit knobs, read once */
        const char *a = getenv("APPLE_MODEM_TX_AMP");
        if (a) tx_amp = atof(a);
        if (tx_amp <= 0 || tx_amp > 0.5) {
            fprintf(stderr, "APPLE_MODEM_TX_AMP out of range (0, 0.5]\n");
            return 1;
        }
    }
    if (argc > 1 && !strcmp(argv[1], "tone")) {
        if (argc < 5) { fprintf(stderr, "tone <rate> <hz> <seconds> [out.s16]\n"); return 1; }
        return tone(atof(argv[2]), atof(argv[3]), atof(argv[4]),
                    (argc > 5) ? argv[5] : NULL);
    }
    if (argc > 1 && !strcmp(argv[1], "dial")) {
        double on = 100.0, off = 100.0;
        const char *m = getenv("APPLE_MODEM_DTMF_MS");
        if (m) { on = atof(m); const char *sl = strchr(m, '/'); if (sl) off = atof(sl + 1); }
        if (argc < 4) { fprintf(stderr, "dial <rate> <digits> [out.s16]\n"); return 1; }
        return dial(atof(argv[2]), argv[3], (argc > 4) ? argv[4] : NULL, on, off);
    }
    if (argc > 1 && !strcmp(argv[1], "capture")) {
        double rate = (argc > 2) ? atof(argv[2]) : 9600.0;
        double secs = (argc > 3) ? atof(argv[3]) : 3.0;
        return capture(rate, secs, (argc > 4) ? argv[4] : NULL);
    }
    if (find_device(1) == kAudioObjectUnknown) {
        fprintf(stderr, "modem audio device not found -- is it configured?\n"
                        "run: ./apple_usb_modem_probe --configure\n");
        return 1;
    }
    return 0;
}
