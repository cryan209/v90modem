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
 * As of 2026-09-30 the input is a CONSTANT at negative full scale, and the
 * constant varies with sample rate (-30069 at 7200, -31703 at 8000, -32768 at
 * 9600 and above).  A fill pattern would not vary with rate; a DC-saturated
 * front end read through a per-rate decimation chain does.  So the analogue
 * path is unpowered and no noise floor is measurable until line control works.
 *
 * Build: make apple_usb_modem_audio
 */

#import <AVFoundation/AVFoundation.h>
#import <AudioToolbox/AudioToolbox.h>
#import <CoreAudio/CoreAudio.h>
#include <stdio.h>
#include <math.h>

static AudioUnit au;
static int16_t *acc;
static long acc_n, acc_cap;
static AudioBufferList *abl;
static int render_errors;

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

int main(int argc, char **argv)
{
    if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
        fprintf(stderr,
            "usage: %s list\n"
            "       %s capture <rate> <seconds> [out.s16]\n"
            "\n"
            "rate must be one the device offers: 7200 8000 8229 8400 9000 9600 10286\n"
            "Run tools/apple_usb_modem_probe --configure first if the device is new\n"
            "to this boot, or CoreAudio will not see it at all.\n", argv[0], argv[0]);
        return 0;
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
