/* apple_usb_modem_coupler -- the Apple USB Modem (A1082, USB 05ac:1401) as
 * v90modem's ANALOGUE side over a real 2-wire line.
 *
 * The role hsf_v90_coupler.c / hsf_fxo_probe.c play for the Conexant 0572:1300
 * (docs/hsf_analogue_v90_coupler.md).  This part seizes the line, dials an
 * extension with DTMF or hook pulses, waits for the far end's answer tone
 * and then runs the modem engine over the result, with the DTE on a PTY.
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
 * THE DEFAULT RATE IS 9600, and the arithmetic is the reason. The device's
 * rate list is 8000 plus every V.34 symbol rate times three, because the SM56's
 * host datapump ran its receiver on a T/3 grid -- the same grid this tree's own
 * T/3 upstream receiver uses.  The engine wants two grids: 8000 for
 * me_rx_audio() and 16000 (T/2) for me_rx_v90a_16k(). Three native rates now
 * reach both by small rational ratios:
 *
 *     8000  -> 8000 = 1/1    -> 16000 = 2/1
 *     9600  -> 8000 = 5/6    -> 16000 = 5/3
 *     9000  -> 8000 = 8/9    -> 16000 = 16/9
 *
 * and the difference between them is the whole argument.  8000 -> 16000 is
 * UPSAMPLING: it invents the T/2 samples by interpolation instead of measuring
 * them, from a stream that is already critically sampled -- V.34 at 3429 baud
 * occupies up to 3673 Hz, so 8000 leaves 326 Hz of Nyquist margin.  9600 leaves
 * 1126 Hz, and 9600 -> 16000 carries genuine information to 4800 Hz, which
 * covers the whole DS0 band. Native 9000 still measures through 4500 Hz and
 * permits a comparison without 9600's fractional USB samples per millisecond.
 * Native 8000 is an opt-in waveform experiment (APPLE_MODEM_8K_PCM), measured
 * with less margin; 9600 remains the default and is T/3 at 3200 baud exactly.
 *
 * Both resamplers are therefore exact rational polyphase, not fractional
 * delays, and that is also what keeps the HSF path's defect out of here: that
 * coupler decimates 16 kHz by two and so must CHOOSE which of two sample sets
 * to keep, and swept as a fractional delay only 1/10, 2/10 and 9/10 of phases
 * reached Phase 4 on three recorded calls, with 0.0 -- the obvious value --
 * failing on all three.  A 5/6 polyphase discards nothing and has no free
 * parameter; the constant group delay it adds is not a choice.
 *
 * Exact, that is, against the CODEC's clock -- which is not the far end's.
 * Measured -165 ppm against a Grandstream ATA (-163 against the VG224), where
 * V.34 §5.2 allows the symbol rate +/-0.01%: our Phase 3 upstream arrived 165
 * ppm slow and the digital side decoded Ja at ~18% bit errors, never parsing
 * it.  So the coupler loop-times (V.34 INFO0 bits 26:27, "synchronized to
 * receive timing"): it measures the far end's ANSam on the raw device stream
 * and scales all three ratios by the codec's offset, still with no free phase
 * and no sample dropped or inserted.  APPLE_MODEM_LOOP_TIMING=0 turns it off,
 * APPLE_MODEM_CLOCK_PPM forces a value. Automatic ANSam correction is for
 * originating calls only: an answerer hears its own answer-tone echo, not a
 * remote clock reference (V.90 5.2, 9.2.1.1.2).
 * The transmit ring pre-rolls a cushion
 * so the moving block/cycle alignment that loop timing brings never underruns.
 *
 * --rate still accepts 8000, which skips the receive resampler entirely and is
 * one filter fewer if all you want is V.34; it cannot feed the T/2 path, and
 * it is not loop-timed.
 * --selftest measures the resamplers, at nominal and at +/-200 ppm, since they
 * are load-bearing.  APPLE_REPLAY_CHUNK=512 makes --rx-replay drain the
 * transmit ring in live-sized cycles, which is what reproduces ring underruns.
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
 * Live modem progress and on-hook CID validation are recorded in
 * docs/apple_usb_modem_sm56.md; offline tests alone do not establish interop.  --rx-replay runs the whole analogue side
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
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>

#include "modem_engine.h"
#include "data_interface.h"
#include "line_monitor.h"
#include <spandsp.h>
#include "apple_modem_line.h"
#include "apple_modem_loopback.h"
#include "apple_modem_dial.h"
#include "test_tmp.h"

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
static int g_gui_control, gui_watch;
static void gui_ring_notice(void);
static int gui_sense = -1, gui_hook = -1;
static apple_modem_line_t line_status = {-1,-1,-1,-1,-1};
static int line_overload_notice, line_guard_tripped;
static int line_protection_register = -1;
static uint64_t line_checked_ms;
static uint64_t apple_now_ms(void);
static unsigned long ring_notifications;
static void line_event(const char *kind, NSDictionary *details);
static void line_observe(void);
static FILE *line_event_file;
static const char *line_event_path;
static NSMutableArray *line_events;
static uint64_t line_event_start, line_event_seq;

/* All event formatting and disk I/O run on the control thread. */
static void line_event(const char *kind, NSDictionary *details)
{
    @autoreleasepool {
        uint64_t now = apple_now_ms();
        if (!line_event_start) line_event_start = now;
        struct timespec wall; clock_gettime(CLOCK_REALTIME,&wall);
        struct tm utc; gmtime_r(&wall.tv_sec,&utc); char stamp[48], date[32];
        strftime(date,sizeof date,"%Y-%m-%dT%H:%M:%S",&utc);
        snprintf(stamp,sizeof stamp,"%s.%03ldZ",date,wall.tv_nsec/1000000);
        NSDictionary *event = @{@"seq":@(++line_event_seq),@"utc":@(stamp),
            @"elapsed_ms":@(now-line_event_start),@"monotonic_ms":@(now),
            @"process_id":@(getpid()),@"event":@(kind),@"details":details ?: @{}};
        if (!line_events) line_events = [[NSMutableArray alloc] init];
        [line_events addObject:event];
        if (line_events.count > 128) [line_events removeObjectAtIndex:0];
        NSData *json = [NSJSONSerialization dataWithJSONObject:event options:0 error:NULL];
        fprintf(stderr,"[APPLE +%llums] %s %.*s\n",(unsigned long long)(now-line_event_start),kind,
            (int)json.length,(const char *)json.bytes);
        if (line_event_file) {
            if (ftell(line_event_file) >= 2*1024*1024) {
                fclose(line_event_file); line_event_file = NULL;
                char *backup = malloc(strlen(line_event_path)+3);
                int rotated = 0;
                if (backup) { sprintf(backup,"%s.1",line_event_path); rotated = rename(line_event_path,backup) == 0; free(backup); }
                line_event_file = fopen(line_event_path,rotated ? "w" : "a");
                if (!rotated) fprintf(stderr,"[APPLE] event log rotation failed; preserving current log\n");
            }
            if (line_event_file) { fwrite(json.bytes,1,json.length,line_event_file); fputc('\n',line_event_file); fflush(line_event_file); }
        }
    }
}
static void line_events_close(void)
{
    if (line_event_file) { fclose(line_event_file); line_event_file = NULL; }
}
static NSArray *line_events_json(void)
{
    if (!line_events) return @[];
    NSUInteger n = line_events.count;
    return [line_events subarrayWithRange:NSMakeRange(n > 16 ? n-16 : 0,n > 16 ? 16 : n)];
}

static void line_notification(const unsigned char *b, int n)
{
    /* USmSerial.sys 6.12.08 @0x181xx: a1/29 with wValue=9 calls
     * the on-hook routine and reports 0x8002 (LINE OVERLOAD DETECTED). */
    if (n >= 4 && b[0] == 0xa1 && b[1] == 0x29 && b[2] == 9 && b[3] == 0)
        { line_overload_notice = 1; line_event("overload_notification",@{}); }
    if (g_gui_control && gui_watch && n >= 2 && b[0] == 0xa1 && b[1] == 0x09)
        gui_ring_notice();
}

static int reg_read(unsigned char idx)
{
    unsigned char cmd[3] = { 0x80, idx, 0x00 }, in[8], notify[16];
    int xfer = 0, rc;

    if (libusb_control_transfer(usb, 0x21, 0x00, 0, IF_CMD, cmd, 3, 1000) != 3)
        return -1;
    /* RESPONSE_AVAILABLE if interface 0 is ours; not required. */
    libusb_interrupt_transfer(usb, 0x81, notify, sizeof notify, &xfer, 300);
    line_notification(notify,xfer);
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
    if (usb) return 0;

    if (libusb_init(NULL) < 0) { fprintf(stderr, "libusb_init failed\n"); return -1; }
    {   /* Two of these on one host is the ordinary case -- one dials, the
         * other answers -- so the device is selectable by USB address.  The
         * CoreAudio UID is a SEPARATE selection (APPLE_MODEM_AUDIO_UID) and
         * the two lists cannot be zipped; see docs/apple_usb_modem_sm56.md. */
        const char *want = getenv("APPLE_MODEM_ADDR");
        libusb_device **list;
        ssize_t n = libusb_get_device_list(NULL, &list);
        libusb_device *pick = NULL;
        for (ssize_t i = 0; i < n && n > 0; i++) {
            struct libusb_device_descriptor dd;
            char addr[32];
            if (libusb_get_device_descriptor(list[i], &dd) != 0 ||
                dd.idVendor != VID || dd.idProduct != PID) continue;
            snprintf(addr, sizeof addr, "%u:%u", libusb_get_bus_number(list[i]),
                     libusb_get_device_address(list[i]));
            if (want) { if (!strcmp(want, addr)) pick = list[i]; }
            else if (!pick) pick = list[i];
        }
        if (pick && libusb_open(pick, &usb) < 0) usb = NULL;
        if (n > 0) libusb_free_device_list(list, 1);
    }
    if (!usb) {
        fprintf(stderr, "no %04x:%04x on the bus%s%s\n", VID, PID,
                getenv("APPLE_MODEM_ADDR") ? " at APPLE_MODEM_ADDR=" : "",
                getenv("APPLE_MODEM_ADDR") ? getenv("APPLE_MODEM_ADDR") : "");
        libusb_exit(NULL); return -1;
    }
    libusb_get_configuration(usb, &cfg);
    /* An unconfigured device stalls EVERY request, including standard ones, and
     * ioreg shows it with no interface children -- it reads as dead hardware.
     * Setting the configuration is also what lets usbaudiod attach, so it must
     * happen before CoreAudio is asked for the device. */
    if (cfg != 1 && libusb_set_configuration(usb, 1) < 0) {
        fprintf(stderr, "could not set configuration 1\n");
        libusb_close(usb); usb = NULL; libusb_exit(NULL); return -1;
    }
    libusb_claim_interface(usb, IF_ACM);
    if (libusb_claim_interface(usb, IF_CMD) < 0) {
        fprintf(stderr, "could not claim the command interface\n");
        libusb_close(usb); usb = NULL; libusb_exit(NULL); return -1;
    }
    line_event("usb_open",@{@"selection":@(getenv("APPLE_MODEM_ADDR") ?: "automatic")});
    return 0;
}

static int line_hook(int off_hook)
{
    int v = reg_read(REG_HOOK), sense;
    int protection = off_hook ? reg_read(17) : -1;

    if (v < 0 || (off_hook && protection < 0)) { fprintf(stderr, "cannot read hook/protection register\n"); return -1; }
    /* Si3056 §5.22: OPE must be cleared before an off-hook transition and
     * re-enabled after 25 ms. Leaving it set trips on normal make transients. */
    if (off_hook && (protection&8) && reg_write(17,(unsigned char)(protection&~8)) < 0) return -1;
    if (off_hook) {
        int actual = reg_read(17);
        line_event("seizure_protection",@{@"before":@(protection),@"disabled_readback":@(actual),@"aux_before":@(reg_read(19)),@"current_before":@(reg_read(12)),@"termination":@(reg_read(26)),@"counter":@(reg_read(31))});
        if (actual < 0 || (actual&8)) { fprintf(stderr,"cannot verify overload detector disarmed before seizure\n"); reg_write(17,(unsigned char)protection); return -1; }
    }
    line_event("hook_request",@{@"off_hook":@(off_hook),@"register_before":@(v)});
    v = off_hook ? (v | REG_HOOK_BIT) : (v & ~REG_HOOK_BIT);
    /* The monitor bit is left set either way: on-hook it is what makes a
     * capture possible at all, and off-hook it costs nothing. */
    v |= REG_MONITOR_BIT;
    if (reg_write(REG_HOOK, (unsigned char)v) < 0) {
        if (off_hook) reg_write(17,(unsigned char)protection);
        fprintf(stderr, "hook write rejected\n"); return -1;
    }
    if (off_hook && (protection&8)) {
        /* Do not use USB write completion alone to time the DAA transition.
         * Read OH back before starting the Si3056 §5.22 settling interval.
         * Live alternated tests: without this barrier 3/3 OPD trips; with
         * it 3/3 normal seizures, with OPE restored and unchanged termination. */
        int actual = reg_read(REG_HOOK);
        if (actual < 0 || !(actual&REG_HOOK_BIT)) {
            reg_write(REG_HOOK,(unsigned char)(v&~REG_HOOK_BIT));
            reg_write(17,(unsigned char)protection);
            return -1;
        }
        usleep(25000);
        if (reg_write(17,(unsigned char)protection) < 0) {
            reg_write(REG_HOOK,(unsigned char)(v&~1)); return -1;
        }
    }
    /* Match the original driver's overload decision while the line settles,
     * rather than leaving the loop closed through an unchecked 500 ms sleep. */
    for (int i = 0; i < 10; i++) {
        usleep(50000);
        if (off_hook) {
            int status = reg_read(0x0c);
            apple_modem_line_t check = {v,status,-1,-1,reg_read(0x13)};
            if (apple_line_overcurrent(&check) || line_overload_notice) {
                line_guard_tripped = 1;
                line_event("overload_trip",@{@"status_register":@(status),@"aux_register":@(check.auxiliary)});
                reg_write(REG_HOOK,(unsigned char)(v & ~REG_HOOK_BIT));
                fprintf(stderr,"[APPLE] LINE OVERLOAD DETECTED during seizure; releasing line\n");
                return -1;
            }
        }
    }
    sense = reg_read(REG_SENSE);
    gui_sense = sense; gui_hook = reg_read(REG_HOOK);
    fprintf(stderr, "[APPLE] %s: register 5 = 0x%02x, line sense 0x%02x%s\n",
            off_hook ? "off-hook" : "on-hook", gui_hook & 0xff,
            sense & 0xff, sense == 0 ? "   <- NO PAIR CONNECTED" : "");
    line_event("hook_readback",@{@"hook_register":@(gui_hook),@"sense_raw":@(sense)});
    return sense <= 0 ? -1 : 0;
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
static int g_pcm8k;             /* opt-in native-8k waveform experiment */
static double g_tx_gain = 1.0;
static volatile int tx_dtmf_done;
static _Atomic int dial_control_active, dial_control_tone;
static struct apple_dial_plan physical_plan;

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
        if (c == 'P' || c == 'T' || c == 'p' || c == 't' || c == '-') continue;
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
#define RS_HIST (RS_TAPS + 2)

/* Loop timing (V.34 §5.2, INFO0 bits 26:27 "synchronized to receive timing").
 * The codec's clock is free-running and is NOT the network's: measured at
 * -165 ppm against a Grandstream ATA and -163 against the VG224, where V.34
 * allows the transmitted symbol rate +/-0.01%.  So the ratios are L/M only
 * nominally: each resampler can be scaled by the codec's measured offset, and
 * the engine's 8 kHz grid then runs on the far end's clock in both directions.
 * Scaling every ratio by the same factor keeps the transmit ring balanced --
 * one device sample in still owes one device sample out -- so no sample is
 * ever dropped or inserted; only where the filter samples between inputs moves.
 *
 * 240 phases because it is divisible by 3, 5 and 6: the original 9600 ratios
 * land exactly on phases at zero clock offset. Native-9000 TX and non-zero
 * clock correction use linear interpolation between adjacent phases; at 240
 * phases that error is far below the -53 dB stopband. */
#define RS_NPH 240
#define RS_FRAC_BITS 24
#define RS_UNIT ((uint64_t)RS_NPH << RS_FRAC_BITS)   /* position units per input */

struct resamp {
    double h[RS_TAPS * RS_NPH + 1];     /* h[j*RS_NPH + p], p = phase */
    double hist[RS_HIST];
    long hn;                            /* input samples consumed */
    uint64_t pos;                       /* next output's input position, RS_UNIT per sample */
    uint64_t step;                      /* input samples per output, same units */
    uint64_t step_nominal;              /* M/L exactly */
};

static void rs_init(struct resamp *r, int L, int M)
{
    int n = RS_TAPS * RS_NPH;
    /* Cut at the lower of the two Nyquists, as before: 0.5/max(L,M) at the L*fs
     * rate is 0.5*min(1, L/M) cycles per input sample. */
    double fc = 0.5 * (L < M ? (double)L / M : 1.0) / RS_NPH;

    memset(r, 0, sizeof *r);
    /* Retain fractional phase units when L does not divide RS_NPH (9/8
     * native-9000 TX). Dividing before the shift changes the sample rate. */
    r->step_nominal = ((uint64_t)M * RS_UNIT + L/2) / L;
    r->step = r->step_nominal;
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
     * normalising the whole prototype would leave every output a factor of the
     * phase count out -- and a gain error here presents as a level problem in
     * the modem, miles from its cause. */
    for (int p = 0; p < RS_NPH; p++) {
        double ps = 0;
        for (int j = 0; j < RS_TAPS; j++) ps += r->h[j * RS_NPH + p];
        if (fabs(ps) < 1e-12) ps = 1.0;
        for (int j = 0; j < RS_TAPS; j++) r->h[j * RS_NPH + p] /= ps;
    }
    r->h[n] = 0;   /* phase RS_NPH of the last tap: past the window, zero */
}

/* Scale input-samples-per-output by `factor`.  Takes effect at the next output,
 * from wherever the position already is, so the timeline stays continuous. */
static void rs_set_factor(struct resamp *r, double factor)
{
    r->step = (uint64_t)llround((double)r->step_nominal * factor);
}

/* Feed one input sample, emit however many outputs fall due (0, 1 or more). */
static int rs_put(struct resamp *r, double x, double *out, int max_out)
{
    int got = 0;

    for (int i = RS_HIST - 1; i > 0; i--) r->hist[i] = r->hist[i - 1];
    r->hist[0] = x;
    r->hn++;
    /* Emit while the next output's position is at or before the sample just
     * pushed.  One input can owe several outputs when upsampling. */
    for (;;) {
        long m = (long)(r->pos / RS_UNIT);
        uint64_t rem = r->pos % RS_UNIT;
        int p = (int)(rem >> RS_FRAC_BITS);
        double fr = (double)(rem & (((uint64_t)1 << RS_FRAC_BITS) - 1))
                    / (double)((uint64_t)1 << RS_FRAC_BITS);
        double acc = 0;

        if (m > r->hn - 1) break;
        if (got >= max_out) break;
        /* hist[0] is input n = hn-1, so input (m - j) is hist[hn-1-m+j].
         * Phase p+1 == RS_NPH is phase 0 one tap on, which the h[] layout
         * gives for free; it drops only tap 0 of that phase, a window tail. */
        for (int j = 0; j < RS_TAPS; j++) {
            int hi = (int)(r->hn - 1 - m) + j;
            const double *hp = &r->h[j * RS_NPH + p];
            if (hi < 0 || hi >= RS_HIST) continue;
            acc += (fr == 0.0 ? hp[0] : hp[0] + fr * (hp[1] - hp[0])) * r->hist[hi];
        }
        out[got++] = acc;
        r->pos += r->step;
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
static _Atomic int render_errors;
/* Transmit-ring underruns while the engine runs: each one plays a zero the
 * engine never produced, i.e. delays everything after it by one device sample
 * -- a 90 degree step at 2400 Hz on the far end's tap.  Counted always, not
 * only for the GUI, because that is exactly what a resampler ratio change can
 * provoke; the first few are dated in output samples since off-hook. */
static _Atomic unsigned long tx_underrun_samples, tx_underrun_runs;
static unsigned long tx_out_count, tx_underrun_at[8];
static int tx_in_underrun;
/* Device sample accounting.  Input and output share one device clock, so over
 * a call these must agree to within a buffer; if they do not, no ring cushion
 * can hold and the difference is what the far end hears as slips.  The ring's
 * fill is sampled once per output callback into 0.5 s bins (minimum kept). */
static unsigned long dev_in_frames, dev_out_frames;
#define FILL_BINS 160
static int fill_min[FILL_BINS];
static int fill_bins_init;
static const char *g_pty_link;
static const char *g_dial;
static const char *g_replay_path;
static volatile int g_answered, g_engine_running, g_stop;
static int g_answer_mode;
static unsigned answer_rings = 1;
static unsigned g_ans_good, g_ans_bad;

/* GUI call requests are posted by the V.250 PTY thread and consumed by the
 * control loop. USB/CoreAudio operations never run on that reader thread.
 * This backend is V.90 §5's analogue role; training telemetry describes
 * the engine's existing V.34 §10.1 / V.90 §9 transitions. */
static int g_di_open;
static void close_dte(void)
{
    if (g_di_open) { di_close(); g_di_open = 0; }
}
static const char *g_control_link, *g_data_link;
static pthread_mutex_t gui_request_lock = PTHREAD_MUTEX_INITIALIZER;
static int gui_request;
static char gui_number[128];
static int gui_socket = -1;
static struct sockaddr_in gui_destination;
static int gui_control_fd = -1, gui_monitor_only, gui_audio_active;
static int gui_rings;
static uint64_t gui_last_ring;
static void gui_commands(void);
static void gui_ring_poll(void);
static void gui_ring_notice(void);
static NSString *gui_raw_json(void);
static void gui_raw_feed(int dir, const int16_t *samples, unsigned n);
static void gui_audio_stop(void);

static const char *gui_status = "Idle · line on-hook";

/* On-hook CID is independent of the modem engine. V.253 §9.2.3.1.3 and
 * Table 13 require checksum-valid formatted reports; SpanDSP validates the
 * Bell 202 / V.23 checksum before this callback. V.23 §2 defines the latter
 * carrier pair. Both demodulators run on a private 8 kHz copy, never the DS0
 * or the engine's resamplers. Callbacks only post a bounded mailbox; PTY and
 * JSON output stay on the control thread. */
struct apple_cid_info {
    char number[65], name[129], date[5], time[5], standard[16];
};
static adsi_rx_state_t *cid_rx[2];
static struct resamp cid_resamp;
static pthread_mutex_t cid_lock = PTHREAD_MUTEX_INITIALIZER;
static struct apple_cid_info cid_pending, cid_current;
static int cid_has_pending;
static unsigned cid_messages, cid_malformed;
static uint64_t cid_received_ms;
static int gui_manual_monitor;

static uint64_t apple_now_ms(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint64_t)ts.tv_sec*1000+ts.tv_nsec/1000000;
}
static void cid_text(char *out, size_t cap, const uint8_t *in, unsigned n)
{
    if (n >= cap) n = (unsigned)cap-1;
    for (unsigned i = 0; i < n; i++) out[i] = in[i] >= 32 && in[i] < 127 ? in[i] : '?';
    out[n] = 0;
}
static void cid_message(void *user, const uint8_t *msg, int len)
{
    struct apple_cid_info info = {0};
    int which = (int)(intptr_t)user;
    if (len < 2 || msg[1] != len-2 || (msg[0] != 0x04 && msg[0] != 0x80)) goto malformed;
    if (msg[0] == 0x04) {
        if (len < 11) goto malformed;
        cid_text(info.date,sizeof info.date,msg+2,4);
        cid_text(info.time,sizeof info.time,msg+6,4);
        cid_text(info.number,sizeof info.number,msg+10,(unsigned)len-10);
    } else {
        for (int pos = 2; pos < len;) {
            if (pos+2 > len) goto malformed;
            unsigned tag = msg[pos++], n = msg[pos++];
            if (n > (unsigned)(len-pos)) goto malformed;
            switch (tag) {
            case 1:
                if (n != 8) goto malformed;
                cid_text(info.date,sizeof info.date,msg+pos,4);
                cid_text(info.time,sizeof info.time,msg+pos+4,4); break;
            case 2: cid_text(info.number,sizeof info.number,msg+pos,n); break;
            case 4:
                if (n != 1 || (msg[pos] != 'P' && msg[pos] != 'O')) goto malformed;
                cid_text(info.number,sizeof info.number,msg+pos,n); break;
            case 7: cid_text(info.name,sizeof info.name,msg+pos,n); break;
            case 8:
                if (n != 1 || (msg[pos] != 'P' && msg[pos] != 'O')) goto malformed;
                cid_text(info.name,sizeof info.name,msg+pos,n); break;
            }
            pos += n;
        }
    }
    if (!info.number[0] && !info.name[0]) return; /* e.g. message waiting */
    snprintf(info.standard,sizeof info.standard,"%s",which ? "V.23" : "Bell 202");
    pthread_mutex_lock(&cid_lock);
    cid_pending = info; cid_has_pending = 1;
    pthread_mutex_unlock(&cid_lock);
    return;
malformed:
    pthread_mutex_lock(&cid_lock); cid_malformed++; pthread_mutex_unlock(&cid_lock);
}
static void cid_stop(void)
{
    for (int i = 0; i < 2; i++) { if (cid_rx[i]) adsi_rx_free(cid_rx[i]); cid_rx[i] = NULL; }
}
static int cid_start(void)
{
    if (g_rate != 8000 && g_rate != 9600) return -1;
    cid_stop();
    pthread_mutex_lock(&cid_lock); cid_has_pending = 0; pthread_mutex_unlock(&cid_lock);
    rs_init(&cid_resamp,5,6);
    cid_rx[0] = adsi_rx_init(NULL,ADSI_STANDARD_CLASS,cid_message,(void *)(intptr_t)0);
    cid_rx[1] = adsi_rx_init(NULL,ADSI_STANDARD_CLIP,cid_message,(void *)(intptr_t)1);
    if (!cid_rx[0] || !cid_rx[1]) { cid_stop(); return -1; }
    return 0;
}
static void cid_feed(const int16_t *samples, unsigned n)
{
    int16_t block[160]; unsigned used = 0;
    if (!cid_rx[0] || !cid_rx[1]) return;
    for (unsigned i = 0; i < n; i++) {
        double out[8];
        int got = g_rate == 8000 ? (out[0] = samples[i], 1) : rs_put(&cid_resamp,samples[i],out,8);
        for (int j = 0; j < got; j++) {
            block[used++] = clip16(out[j]);
            if (used == 160) { for (int k = 0; k < 2; k++) adsi_rx(cid_rx[k],block,used); used = 0; }
        }
    }
    if (used) for (int k = 0; k < 2; k++) adsi_rx(cid_rx[k],block,used);
}
static void cid_deliver(void)
{
    struct apple_cid_info info; int ready;
    pthread_mutex_lock(&cid_lock);
    ready = cid_has_pending; info = cid_pending; cid_has_pending = 0;
    pthread_mutex_unlock(&cid_lock);
    if (!ready) return;
    /* Ignore repeats of the same valid message within a ring sequence. */
    uint64_t now = apple_now_ms();
    if (cid_received_ms && now-cid_received_ms < 8000 &&
        !strcmp(info.number,cid_current.number) && !strcmp(info.name,cid_current.name) &&
        !strcmp(info.date,cid_current.date) && !strcmp(info.time,cid_current.time)) return;
    cid_current = info; cid_messages++; cid_received_ms = now;
    di_receive_caller_id(info.number,info.name,info.date,info.time);
    line_event("caller_id",@{@"number":@(info.number),@"name":@(info.name),@"date":@(info.date),@"time":@(info.time)});
    fprintf(stderr,"[APPLE] CID %s: DATE=%s TIME=%s NMBR=%s NAME=%s\n",
        info.standard,info.date,info.time,info.number,info.name);
}

/* USB polling runs on the control thread, outside CoreAudio callbacks. */
static int line_poll(int force)
{
    uint64_t now = apple_now_ms();
    if (!usb || (!force && now-line_checked_ms < 1000)) return line_guard_tripped ? -1 : 0;
    line_status.hook = reg_read(REG_HOOK);
    line_status.status = reg_read(0x0c);
    line_status.sense = reg_read(REG_SENSE);
    line_status.auxiliary = reg_read(0x13);
    line_protection_register = reg_read(0x11);
    if (force || line_status.revision < 0) line_status.revision = reg_read(0x0b);
    gui_hook = line_status.hook; gui_sense = line_status.sense;
    line_checked_ms = apple_now_ms();
    line_observe();
    if (apple_line_overcurrent(&line_status) || line_overload_notice) {
        line_guard_tripped = 1;
        line_event("overload_trip",@{@"status_register":@(line_status.status),@"aux_register":@(line_status.auxiliary)});
        /* Use the known hook bit, without resetting country/gain registers.
         * If readback failed, zero is the safe on-hook/monitor-off value. */
        int hook = line_status.hook < 0 ? 0 : line_status.hook & ~REG_HOOK_BIT;
        int released = reg_write(REG_HOOK,(unsigned char)hook) == 0;
        gui_hook = reg_read(REG_HOOK);
        gui_status = "Line overload · call stopped";
        fprintf(stderr,"[APPLE] LINE OVERLOAD DETECTED: status=0x%02x notification=%d; on-hook %s\n",
            line_status.status & 255,line_overload_notice,
            released && gui_hook >= 0 && !(gui_hook & 1) ? "confirmed" : "NOT CONFIRMED");
        g_stop = 1;
        return -1;
    }
    return 0;
}
static NSDictionary *line_json(void)
{
    int valid = apple_line_valid(&line_status);
    int offhook = line_status.hook >= 0 && (line_status.hook & 1);
    int current = line_status.status < 0 ? -1 : line_status.status & 31;
    return @{@"checked_ms":@(line_checked_ms),@"valid":@(valid),
        @"status_register":@(line_status.status),@"revision_register":@(line_status.revision),
        @"aux_register":@(line_status.auxiliary),@"protection_register":@(line_protection_register),
        @"hardware_protection_enabled":line_protection_register >= 0 ? @((line_protection_register & 8) != 0) : (id)[NSNull null],@"sense_raw":@(line_status.sense),
        @"voltage_estimate_v":valid && line_status.sense >= 0 ? @(apple_line_signed_sense(line_status.sense)) : (id)[NSNull null],
        @"voltage_calibrated":@NO,
        @"current_code":valid && offhook ? @(current) : (id)[NSNull null],
        @"current_estimate_ma":valid && offhook && current < 31 ? @(current*3.3) : (id)[NSNull null],
        @"overcurrent":@(apple_line_overcurrent(&line_status) || line_overload_notice || line_guard_tripped),
        @"receive_overload":valid && line_status.auxiliary >= 0 ? @((line_status.auxiliary & 4) != 0) : (id)[NSNull null],
        @"dropout":valid && offhook && line_status.auxiliary >= 0 ? @((line_status.auxiliary & 2) != 0) : (id)[NSNull null]};
}

static void line_observe(void)
{
    static apple_modem_line_t last = {-1,-1,-1,-1,-1};
    static int seen, last_protection = -1;
    if (!seen) line_event("line_baseline",line_json());
    else {
        int valid = apple_line_valid(&line_status), before = apple_line_valid(&last);
        if (valid != before) line_event(valid ? "link_restored" : "link_lost",line_json());
        if (line_status.hook != last.hook &&
            (line_status.hook < 0 || last.hook < 0 || ((line_status.hook ^ last.hook) & 9)))
            line_event("hook_changed",line_json());
        if (valid && before) {
            if (line_status.sense >= 0 && last.sense >= 0 && line_status.sense != last.sense) {
                int v = apple_line_signed_sense(line_status.sense), old = apple_line_signed_sense(last.sense);
                line_event(v && old && ((v < 0) != (old < 0)) ? "polarity_reversed" : "voltage_changed",line_json());
            }
            if ((line_status.hook & 1) && ((last.status ^ line_status.status) & 31)) line_event("current_changed",line_json());
            if (line_status.auxiliary >= 0 && last.auxiliary >= 0 &&
                ((last.auxiliary ^ line_status.auxiliary) & ((line_status.hook & 1) ? 7 : 4)))
                line_event("line_flags_changed",line_json());
        }
        if (line_protection_register != last_protection) line_event("protection_changed",line_json());
    }
    last = line_status; last_protection = line_protection_register; seen = 1;
}
static void engine_events(const me_diag_snapshot_t *d)
{
    static int seen, ready;
    static me_state_t state;
    static char rx[128], tx[128];
    if (!seen || state != d->state || ready != d->data_ready || strcmp(rx,d->rx_signal) || strcmp(tx,d->tx_signal)) {
        line_event("engine_phase",@{@"state":@(d->state),@"data_ready":@(d->data_ready),
            @"rx_signal":@(d->rx_signal),@"tx_signal":@(d->tx_signal),@"phase_elapsed_ms":@(d->phase_elapsed_ms)});
        state = d->state; ready = d->data_ready;
        snprintf(rx,sizeof rx,"%s",d->rx_signal); snprintf(tx,sizeof tx,"%s",d->tx_signal); seen = 1;
    }
    static unsigned long errors, gaps;
    if (errors != (unsigned long)render_errors || gaps != tx_underrun_samples) {
        errors = render_errors; gaps = tx_underrun_samples;
        line_event("audio_transport",@{@"render_errors":@(errors),@"engine_tx_gap_samples":@(gaps),@"engine_tx_gap_runs":@(tx_underrun_runs)});
    }
}
static NSDictionary *loopback_last;
static const char *loopback_pattern = "prbs";
static double loopback_tone_level = 1.0;
static int loopback_rx_gain = -1;
static NSDictionary *cid_json(void)
{
    pthread_mutex_lock(&cid_lock); unsigned bad = cid_malformed; pthread_mutex_unlock(&cid_lock);
    return @{@"listening":@(gui_monitor_only && cid_rx[0] != NULL),@"messages":@(cid_messages),
        @"malformed":@(bad),@"number":@(cid_current.number),@"name":@(cid_current.name),
        @"date":@(cid_current.date),@"time":@(cid_current.time),@"decoder":@(cid_current.standard)};
}


static void gui_dial(const char *number, void *user)
{
    (void)user;
    pthread_mutex_lock(&gui_request_lock);
    if (!gui_request) {
        int n = snprintf(gui_number, sizeof gui_number, "%s", number);
        gui_request = n < 0 || (size_t)n >= sizeof gui_number ? 3 : 1;
    }
    pthread_mutex_unlock(&gui_request_lock);
}
static int gui_dial_plan(const char *plan, void *user)
{
    struct apple_dial_plan checked;
    if (apple_dial_build(&checked,plan,70,70) < 0 ||
        (!checked.pulses && checked.count+1 > TX_MAX_SEG)) return -1;
    gui_dial(plan,user);
    return 0;
}
static void gui_answer(void *user)
{
    (void)user;
    pthread_mutex_lock(&gui_request_lock);
    if (!gui_request) gui_request = 2;
    pthread_mutex_unlock(&gui_request_lock);
}
static void gui_hangup(void *user) { (void)user; g_stop = 1; }

static void gui_monitor_start(void)
{
    const char *port = getenv("ME_GUI_PORT");
    int n = port ? atoi(port) : 0;
    if (n < 1 || n > 65535) return;
    gui_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (gui_socket < 0) return;
    int bytes = 65536;
    setsockopt(gui_socket, SOL_SOCKET, SO_SNDBUF, &bytes, sizeof bytes);
    fcntl(gui_socket, F_SETFL, O_NONBLOCK);
    gui_destination.sin_family = AF_INET;
    gui_destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    gui_destination.sin_port = htons(n);
}
static void gui_publish(void)
{
    cid_deliver();
    @autoreleasepool {
    static uint64_t last;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now = (uint64_t)ts.tv_sec*1000 + ts.tv_nsec/1000000;
    if (now-last < 100) return;
    last = now;
    me_diag_snapshot_t d;
    me_get_diag_snapshot(&d);
    engine_events(&d);
    if (gui_socket < 0) return;
    char buf[65500];
    int n = snprintf(buf, sizeof buf,
        "{\"state\":%d,\"modulation\":%d,\"v92\":%d,\"law\":%d,"
        "\"rx_signal\":\"%s\",\"tx_signal\":\"%s\",\"rx_baud\":%d,\"tx_baud\":%d,"
        "\"rx_pcm\":%d,\"tx_pcm\":%d,\"rx_carrier\":%.3f,\"tx_carrier\":%.3f,"
        "\"data_ready\":%d,\"elapsed\":%llu,\"hardware_status\":\"%s\",",
        d.state,d.modulation,d.v92_active,d.law,d.rx_signal,d.tx_signal,
        d.rx_baud,d.tx_baud,d.rx_pcm,d.tx_pcm,d.rx_carrier,d.tx_carrier,
        d.data_ready,(unsigned long long)d.phase_elapsed_ms,gui_status);
    if (n < 0 || (size_t)n >= sizeof buf-2) return;
    int extra = lm_gui_json(buf+n,sizeof buf-(size_t)n-2);
    if (extra < 0) return;
    n += extra;
    pthread_mutex_lock(&gui_request_lock);
    int pending = gui_request;
    pthread_mutex_unlock(&gui_request_lock);
    int added = snprintf(buf+n,sizeof buf-(size_t)n,
        ",\"line_health\":{\"usb_open\":%s,\"sense\":%d,\"hook_register\":%d,\"rings\":%d,\"ring_notifications\":%lu,\"watching\":%s,\"monitoring\":%s,\"call_pending\":%s}}",
        usb ? "true" : "false",gui_sense,gui_hook,gui_rings,ring_notifications,
        gui_watch ? "true" : "false",gui_monitor_only ? "true" : "false",
        pending ? "true" : "false");
    if (added < 0 || (size_t)added >= sizeof buf-(size_t)n) return;
    n += added;
    sendto(gui_socket,buf,n,0,(struct sockaddr *)&gui_destination,sizeof gui_destination);
    NSDictionary *audio = [NSJSONSerialization JSONObjectWithData:[gui_raw_json() dataUsingEncoding:NSUTF8StringEncoding] options:0 error:NULL];
    NSData *packet = [NSJSONSerialization dataWithJSONObject:@{@"kind":@"apple_device",@"hardware_audio":audio,@"caller_id":cid_json(),@"line_diagnostics":line_json(),@"line_events":line_events_json(),@"loopback_test":loopback_last ?: @{}} options:0 error:NULL];
    sendto(gui_socket,packet.bytes,packet.length,0,(struct sockaddr *)&gui_destination,sizeof gui_destination);
    }
}

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

/*
 * Near-end echo cancellation (near_echo.h), in the device domain.
 *
 * In the analogue V.90 role this modem transmits CPt/CP for the whole of
 * Phase 4 (§9.4.2) while it receives the digital modem's Ri/TRN2d/MP, and its
 * own transmit returns through the hybrid ~118 ms later (the USB round trip;
 * 1133/1142/1165 device samples on three calls) at -16.7 dB against the
 * downstream.  The same receiver fits TRN1d at 59.7 dB while we are silent
 * and Phase 4's MP at 20 dB while we transmit (docs/apple_usb_modem_sm56.md).  Here both directions are
 * device samples on one clock, in the order they crossed the codec -- the
 * order the taps record -- so the echo path is LTI and the fit is exact.
 *
 * On by default in the PCM analogue role only (ME_V90_ROLE=analogue), where
 * full-duplex PCM training needs it; APPLE_MODEM_ECHO_CANCEL=1/0 forces it
 * either way.  In --rx-replay the reference must be the live call's transmit
 * tap (APPLE_MODEM_EC_REF), since the replay's own output never crossed a
 * device and has none of the delay.
 */
#include "../near_echo.h"
static near_echo_t *g_ec;
static FILE *g_ec_ref;
static int g_ec_reported_fits;
static long g_ec_last_report, g_ec_rx_n;

static void ec_setup(void)
{
    const char *force = getenv("APPLE_MODEM_ECHO_CANCEL");
    const char *role = getenv("ME_V90_ROLE");
    int on = (role && !strcmp(role, "analogue"));

    if (force && *force) on = strcmp(force, "0") != 0;
    if (!on || g_ec || (g_rate != 9600.0 && g_rate != 9000.0 && !(g_rate == 8000.0 && g_pcm8k))) return;
    /* Delays 900..1667 samples (94-174 ms): the measured onsets are 1121-1165
     * and the response needs ~384 taps to reach the line's floor.  A live
     * call solves on a worker (a 768-tap solve is ~75 M operations, far too
     * long for this callback); a replay solves inline so it stays
     * deterministic. */
    /* Keep the same physical delay coverage on native 8k. Direct-pair
     * captures measured self-echo peaks at 143/149 ms there, versus
     * 120/124 ms at 9600 Hz; both lie inside this existing window. */
    int lag = (int)lrint(900*g_rate/9600.0);
    int taps = (int)lrint(768*g_rate/9600.0);
    g_ec = near_echo_init(lag, taps, (int)lrint(4800*g_rate/9600.0));
    if (g_ec && !g_replay_path) near_echo_set_threaded(g_ec, 1);
    fprintf(stderr, "[APPLE] near-end echo canceller: %s (delays %.0f-%.0f ms, %d taps, echo-only blocks)\n",
            g_ec ? "on" : "FAILED to allocate", lag*1000/g_rate, (lag+taps)*1000/g_rate, taps);
}

static void ec_rx(int16_t *s, long n)
{
    if (!g_ec) return;
    near_echo_rx(g_ec, s, (int) n);
    g_ec_rx_n += n;
    /* APPLE_MODEM_EC_DUMP=<path>: the receive stream after cancellation, at
     * the device rate -- what is left of our own echo is measured here. */
    {
        static FILE *d; static int tried;
        if (!tried) { const char *p = getenv("APPLE_MODEM_EC_DUMP"); tried = 1; if (p && *p) d = fopen(p, "wb"); }
        if (d) fwrite(s, 2, (size_t) n, d);
    }
    if (near_echo_active(g_ec)
        && (g_ec_reported_fits == 0 || g_ec_rx_n - g_ec_last_report >= 10 * 9600)) {
        g_ec_reported_fits = near_echo_solved(g_ec);
        g_ec_last_report = g_ec_rx_n;
        fprintf(stderr, "[APPLE] echo canceller: "
                "echo rms %.0f of rx %.0f, fit %d on %ld echo-only blocks\n",
                near_echo_rms_echo(g_ec), near_echo_rms_in(g_ec),
                near_echo_solved(g_ec), near_echo_blocks_used(g_ec));
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
#define TX_CUSHION 192            /* two engine blocks, 20 ms at 9600 */
static volatile int tx_preroll;
static int16_t ring[RING_N];
static volatile unsigned ring_w, ring_r;
/* Count every lost TX sample even in headless runs. V.90 5.2 fixes the
 * downstream symbol clock; a full ring invalidates a continuity comparison. */
static unsigned long tx_overrun_samples;

/* Receive chain: device rate -> 8000 for me_rx_audio, and
 * -> 16000 for me_rx_v90a_16k.  The T/2 stream is offered FIRST, as the engine
 * header requires: it is the stream the V.90 analogue Phase 3 receiver reads,
 * and it must see the samples before the 8 kHz decimation. */
static struct resamp rs_8k, rs_16k, rs_tx;
static int16_t fb8[1024];
static long    fb8_n;

#define GUI_RAW_N 1600
static pthread_mutex_t gui_raw_lock = PTHREAD_MUTEX_INITIALIZER;
static int16_t gui_raw[2][GUI_RAW_N];
static uint64_t gui_raw_count[2], gui_clipped[2], gui_empty_tx, gui_overruns;

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
            /* The transmit ring needs a margin, and it has to be laid down
             * on the OUTPUT side.  The engine makes 96-sample blocks and the
             * device takes 512-frame cycles, so the ring's fill at each
             * output callback swings by up to a block with where block edges
             * fall against cycle edges.  At exact 5/6 and 6/5 that alignment
             * never moves and the startup underruns (the engine starts part
             * way through a cycle) leave the ring at EXACTLY zero margin --
             * harmless only because nothing ever moves.  Loop timing moves it
             * one engine sample every 0.63 s at 165 ppm, and each time a block
             * edge crosses a cycle edge the fill dips below empty: a zero the
             * engine never made, a one-sample slip in the upstream (live: 32
             * runs, 286 samples in 30 s; reproduced exactly by a 512-frame
             * replay).  Neither zeros pre-filled into the ring nor zeros
             * owed to the output side survive: the engine's first, partial
             * cycle underruns AFTER them and resets the margin to zero.  So
             * the output side pre-rolls: it plays silence, without reading
             * the ring, until the ring holds a whole callback plus
             * TX_CUSHION, and only then starts draining.  That is a constant
             * delay ahead of the engine's first sample, and the ring keeps
             * TX_CUSHION of margin over its steady-state low point for the
             * rest of the call. */
            tx_preroll = 1;
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
                    if (nw == ring_r) {
                        tx_overrun_samples += got - j;
                        if (g_gui_control) { pthread_mutex_lock(&gui_raw_lock); gui_overruns += got - j; pthread_mutex_unlock(&gui_raw_lock); }
                        break;
                    }
                    ring[ring_w] = clip16(up[j] * g_tx_gain);
                    ring_w = nw;
                }
            }
        }
        off += k;
    }
}

/* Receive-timing measurement for loop timing (see the resampler header).
 *
 * The reference is the far end's answer tone: generated on its own clock, on
 * the line for seconds before anything of ours matters, and ending exactly
 * when V.8 moves on.  ANSam reverses phase every 450 ms (V.25), which defeats
 * a plain phase fit -- measured: 0.88 rad residual and -364 ppm -- so the fit
 * runs on the SQUARED baseband, where a 180 degree reversal is invisible.  On
 * four recorded calls that read +165.0..+165.3 ppm at 0.005 rad residual, and
 * a server tap corrected anywhere in -100..-230 ppm parses the Ja descriptor
 * that 0 ppm cannot.  Raw device samples are measured, before any resampler,
 * so the correction never measures itself. */
#define LT_F0        2100.0
#define LT_BLOCK     192             /* 20 ms at 9600 */
#define LT_MAX       100             /* 2 s of tone is plenty */
#define LT_MIN       40              /* 0.8 s, or no verdict */
#define LT_MAX_PPM   1000.0
#define LT_MAX_RESID 0.2             /* rad, on the squared phase */

static int    lt_enabled = 1, lt_done;
static double lt_ph[LT_MAX], lt_w[LT_MAX], lt_t[LT_MAX];
static int    lt_n, lt_fill, lt_misses;
static long   lt_block_index;
static double lt_re, lt_im, lt_energy, lt_osc;

static void loop_timing_apply(double ppm, const char *why)
{
    /* The codec runs at g_rate*(1+d) in the far end's time.  Receive paths
     * consume (1+d) more device samples per output; transmit consumes 1/(1+d)
     * more engine samples per device sample. */
    double d = ppm * 1e-6;

    rs_set_factor(&rs_8k, 1.0 + d);
    rs_set_factor(&rs_16k, 1.0 + d);
    rs_set_factor(&rs_tx, 1.0 / (1.0 + d));
    lt_done = 1;
    fprintf(stderr, "[APPLE] loop timing: codec clock %+.1f ppm (%s); all three "
                    "resamplers corrected\n", ppm, why);
}

static void loop_timing_fit(void)
{
    double sw = 0, st = 0, sp = 0, stt = 0, stp = 0, slope, icpt, res = 0, ppm;
    char why[160];

    lt_done = 1;
    if (lt_n < LT_MIN) {
        fprintf(stderr, "[APPLE] loop timing: only %d blocks of answer tone (need %d); "
                        "codec left uncorrected\n", lt_n, LT_MIN);
        return;
    }
    for (int i = 0; i < lt_n; i++) {
        sw += lt_w[i]; st += lt_w[i] * lt_t[i]; sp += lt_w[i] * lt_ph[i];
        stt += lt_w[i] * lt_t[i] * lt_t[i]; stp += lt_w[i] * lt_t[i] * lt_ph[i];
    }
    slope = (sw * stp - st * sp) / (sw * stt - st * st);
    icpt = (sp - slope * st) / sw;
    for (int i = 0; i < lt_n; i++) {
        double e = lt_ph[i] - (slope * lt_t[i] + icpt);
        res += lt_w[i] * e * e;
    }
    res = sqrt(res / sw);
    /* Squared baseband: slope is 2*2*pi*(f_meas - F0).  The tone was made at F0
     * on the far clock and reads f_meas on ours, so (1+d) = F0 / f_meas. */
    double f_meas = LT_F0 + slope / (4.0 * M_PI);
    ppm = (LT_F0 / f_meas - 1.0) * 1e6;
    snprintf(why, sizeof why, "answer tone %.4f Hz over %.2f s, residual %.3f rad",
             f_meas, lt_n * LT_BLOCK / g_rate, res);
    if (res > LT_MAX_RESID || fabs(ppm) > LT_MAX_PPM) {
        fprintf(stderr, "[APPLE] loop timing: %s -> %+.1f ppm REJECTED; codec left "
                        "uncorrected\n", why, ppm);
        return;
    }
    loop_timing_apply(ppm, why);
}

static void loop_timing_feed(const int16_t *s, long n)
{
    const double w0 = 2.0 * M_PI * LT_F0 / g_rate;

    if (!lt_enabled || lt_done || !g_answered || g_rate == 8000.0)
        return;
    for (long i = 0; i < n; i++) {
        double x = s[i];
        lt_re += x * cos(lt_osc);
        lt_im -= x * sin(lt_osc);
        lt_energy += x * x;
        lt_osc += w0;
        if (lt_osc > 2.0 * M_PI) lt_osc -= 2.0 * M_PI;
        if (++lt_fill < LT_BLOCK)
            continue;
        /* One 20 ms block.  It counts as tone on the answer detector's terms --
         * most of its power in the 2100 Hz bin -- and the tone ending (V.8
         * moving on) is what closes the measurement. */
        double mag2 = lt_re * lt_re + lt_im * lt_im;
        double frac = 2.0 * mag2 / (LT_BLOCK * fmax(lt_energy, 1.0));
        if (frac > 0.3) {
            double sr = lt_re * lt_re - lt_im * lt_im, si = 2.0 * lt_re * lt_im;
            double ph = atan2(si, sr);
            if (lt_n > 0) {
                double prev = lt_ph[lt_n - 1];
                while (ph - prev >  M_PI) ph -= 2.0 * M_PI;
                while (ph - prev < -M_PI) ph += 2.0 * M_PI;
            }
            lt_ph[lt_n] = ph;
            lt_w[lt_n] = sqrt(sr * sr + si * si);
            lt_t[lt_n] = (lt_block_index + 0.5) * LT_BLOCK / g_rate;
            lt_n++;
            lt_misses = 0;
        } else if (lt_n > 0 && ++lt_misses >= 5) {
            loop_timing_fit();
            return;
        }
        lt_block_index++;
        lt_fill = 0; lt_re = lt_im = lt_energy = 0;
        if (lt_n >= LT_MAX) {
            loop_timing_fit();
            return;
        }
    }
}

static void engine_feed(int16_t *s, long n)
{
    dc_block(s, n);
    ec_rx(s, n);
    loop_timing_feed(s, n);

    if (g_rate == 8000.0 && !g_pcm8k) {           /* no waveform experiment */
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
        got = g_rate == 8000.0 ? (o[0] = s[i], 1) : rs_put(&rs_8k, s[i], o, 8);
        for (int j = 0; j < got; j++) {
            fb8[fb8_n++] = clip16(o[j]);
            if (fb8_n == 80) { engine_block_8k(fb8, 80); fb8_n = 0; }
        }
    }
}

/* Device-level taps.  --rx-replay could consume a recording and nothing could
 * MAKE one, which left the question that matters here unobservable: the
 * engine's own v34_rx.raw is written only while the V.34 receiver is fed, so on
 * a V.90/V.92 call it has a HOLE exactly over Phase 3 -- its stage marks jump
 * from Phase 2 straight to the post-retrain Phase 2 -- and it is post-filter
 * besides.  These two are the raw device streams at --rate, so rx-tap.s16 is
 * exactly what --rx-replay expects (before dc_block, which engine_feed applies
 * itself; recording inside engine_feed would make a replay re-record its own
 * input, so the receive tap lives in the live-only callback).  The transmit tap
 * is what actually went on the line, which is the only way to tell our own
 * echo from the far end -- a distinction this project has got wrong often
 * enough to be worth the two fopen()s. */
static FILE *rx_tap, *tx_tap;

static void taps_open(void)
{
    const char *r = getenv("APPLE_MODEM_RX_TAP"), *t = getenv("APPLE_MODEM_TX_TAP");

    if (r && *r && !(rx_tap = fopen(r, "wb")))
        fprintf(stderr, "[APPLE] cannot write RX tap %s\n", r);
    if (t && *t && !(tx_tap = fopen(t, "wb")))
        fprintf(stderr, "[APPLE] cannot write TX tap %s\n", t);
    if (rx_tap || tx_tap)
        fprintf(stderr, "[APPLE] device taps: s16le mono at %.0f Hz%s%s\n", g_rate,
                rx_tap ? " rx" : "", tx_tap ? " tx" : "");
}

static void taps_close(void)
{
    if (rx_tap) { fclose(rx_tap); rx_tap = NULL; }
    if (tx_tap) { fclose(tx_tap); tx_tap = NULL; }
}

/* Passive device tap: at most 1600 samples per direction. No playback queue,
 * filtering or sample insertion exists here. Formatting stays on the control
 * thread. V.90 §5: these are analogue codec samples, never DS0 codewords. */
static void gui_raw_feed(int dir, const int16_t *samples, unsigned n)
{
    if (!g_gui_control) return;
    pthread_mutex_lock(&gui_raw_lock);
    for (unsigned i = 0; i < n; i++) {
        gui_raw[dir][gui_raw_count[dir] % GUI_RAW_N] = samples[i];
        gui_raw_count[dir]++;
        if (samples[i] >= 32760 || samples[i] <= -32760) gui_clipped[dir]++;
    }
    pthread_mutex_unlock(&gui_raw_lock);
}
static NSString *gui_raw_json(void)
{
    int16_t copy[2][GUI_RAW_N];
    uint64_t counts[2], clipped[2], empty, overruns;
    pthread_mutex_lock(&gui_raw_lock);
    for (int dir = 0; dir < 2; dir++) {
        counts[dir] = gui_raw_count[dir]; clipped[dir] = gui_clipped[dir];
        unsigned n = (unsigned)(counts[dir] < GUI_RAW_N ? counts[dir] : GUI_RAW_N);
        for (unsigned i = 0; i < n; i++) copy[dir][i] = gui_raw[dir][(counts[dir]-n+i)%GUI_RAW_N];
    }
    empty = gui_empty_tx; overruns = gui_overruns;
    pthread_mutex_unlock(&gui_raw_lock);
    NSMutableArray *audio = [NSMutableArray array], *listen = [NSMutableArray array], *metrics = [NSMutableArray array];
    for (int dir = 0; dir < 2; dir++) {
        unsigned n = (unsigned)(counts[dir] < GUI_RAW_N ? counts[dir] : GUI_RAW_N);
        NSMutableArray *samples = [NSMutableArray array];
        NSMutableString *hex = [NSMutableString string];
        double sum = 0, square = 0;
        int low = n ? copy[dir][0] : 0, high = low;
        unsigned zeros = 0, rails = 0;
        for (unsigned i = 0; i < n; i++) {
            uint16_t v = (uint16_t)copy[dir][i];
            [hex appendFormat:@"%02x%02x",v&255,v>>8];
            sum += copy[dir][i]; square += (double)copy[dir][i]*copy[dir][i];
            if (copy[dir][i] < low) low = copy[dir][i];
            if (copy[dir][i] > high) high = copy[dir][i];
            zeros += copy[dir][i] == 0;
            rails += abs(copy[dir][i]) >= 32760;
            if (i+512 >= n) [samples addObject:@(copy[dir][i])];
        }
        double mean = n ? sum/n : 0;
        [audio addObject:samples];
        [listen addObject:@{@"count":@(counts[dir]),@"hex":hex}];
        double rms = n ? sqrt(fmax(0,square/n-mean*mean)) : 0;
        /* A constant rail is the documented unpowered/open-loop ADC symptom.
         * These are observations, not a diagnosis of the telephone bearer.
         * dBFS uses peak full scale, AC RMS excludes DC (V.90 §5 analogue side). */
        NSString *status = !n ? @"no samples" : low == high ?
            (rails ? @"constant rail" : zeros == n ? @"digital silence" : @"constant input") :
            rails ? @"near rail" : @"varying signal";
        [metrics addObject:@{@"dc":@(mean),@"rms":@(rms),@"clipped":@(clipped[dir]),
            @"samples":@(counts[dir]),@"window_samples":@(n),@"min":@(low),@"max":@(high),
            @"zero_samples":@(zeros),@"rail_samples":@(rails),@"status":status,
            @"ac_dbfs":rms > 0 ? @(20*log10(rms/32768.0)) : (id)[NSNull null]}];
    }
    NSDictionary *value = @{@"rate":@(g_rate),@"audio":audio,@"listen":listen,@"metrics":metrics,
        @"render_errors":@(render_errors),@"tx_empty_samples":@(empty),@"tx_ring_overruns":@(overruns),
        @"engine_tx_underrun_samples":@(tx_underrun_samples),@"engine_tx_underrun_runs":@(tx_underrun_runs)};
    NSData *json = [NSJSONSerialization dataWithJSONObject:value options:0 error:NULL];
    return [[[NSString alloc] initWithData:json encoding:NSUTF8StringEncoding] autorelease];
}

static int loopback_audio_active;
static int16_t *loopback_samples[2];
static _Atomic unsigned loopback_count[2], loopback_tx_available;
static unsigned loopback_capacity;
static int loopback_peer_fd = -1;
static _Atomic unsigned loopback_stream_gaps;

static OSStatus input_cb_impl(void *ref, AudioUnitRenderActionFlags *flags,
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
    dev_in_frames += nframes;
    if (rx_tap)
        fwrite(in_abl->mBuffers[0].mData, 2, nframes, rx_tap);
    gui_raw_feed(0,in_abl->mBuffers[0].mData,nframes);
    if (loopback_audio_active) {
        unsigned n = nframes;
        if (n > loopback_capacity-loopback_count[0]) n = loopback_capacity-loopback_count[0];
        memcpy(loopback_samples[0]+loopback_count[0],in_abl->mBuffers[0].mData,n*sizeof(int16_t));
        loopback_count[0] += n;
    }
    else if (atomic_load(&dial_control_active)) { /* capture only while dialling */ }
    else if (gui_monitor_only) cid_feed(in_abl->mBuffers[0].mData,nframes);
    else engine_feed(in_abl->mBuffers[0].mData,nframes);
    return noErr;
}

static OSStatus output_cb_impl(void *ref, AudioUnitRenderActionFlags *flags,
                          const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
                          AudioBufferList *io)
{
    int16_t *out = io->mBuffers[0].mData;

    (void)ref; (void)flags; (void)ts; (void)bus;
    if (!fill_bins_init) { for (int b = 0; b < FILL_BINS; b++) fill_min[b] = -1; fill_bins_init = 1; }
    if (g_engine_running) {
        int fill = (int)((ring_w + RING_N - ring_r) % RING_N);
        long bin = (long)(dev_out_frames / (g_rate * 0.5));
        if (bin < FILL_BINS && (fill_min[bin] < 0 || fill < fill_min[bin])) fill_min[bin] = fill;
    }
    dev_out_frames += nframes;
    if (loopback_audio_active) {
        unsigned n = nframes;
        unsigned available = loopback_peer_fd >= 0 ? atomic_load(&loopback_tx_available) : loopback_capacity;
        if (n > available-loopback_count[1]) n = available-loopback_count[1];
        if (loopback_peer_fd >= 0 && n < nframes) { loopback_stream_gaps += nframes-n; g_stop = 1; }
        memcpy(out,loopback_samples[1]+loopback_count[1],n*sizeof(int16_t));
        memset(out+n,0,(nframes-n)*sizeof(int16_t));
        loopback_count[1] += n;
        gui_raw_feed(1,out,nframes);
        if (tx_tap) fwrite(out,2,nframes,tx_tap);
        return noErr;
    }
    if (atomic_load(&dial_control_active)) {
        static int previous;
        static double phase1, phase2;
        int digit = atomic_load(&dial_control_tone);
        double lo = 0, hi = 0;
        if (digit != previous) { phase1 = phase2 = 0; previous = digit; }
        if (digit) dtmf_pair((char)digit,&lo,&hi);
        for (UInt32 i = 0; i < nframes; i++) {
            double v = digit ? tx_amp*(sin(phase1)+sin(phase2)) : 0;
            out[i] = (int16_t)lrint(fmax(-0.999,fmin(0.999,v))*32767.0);
            phase1 += 2*M_PI*lo/g_rate; phase2 += 2*M_PI*hi/g_rate;
        }
        gui_raw_feed(1,out,nframes);
        if (tx_tap) fwrite(out,2,nframes,tx_tap);
        return noErr;
    }
    if (tx_preroll && g_engine_running && !gui_monitor_only && tx_seg >= tx_nseg) {
        unsigned fill = (ring_w + RING_N - ring_r) % RING_N;
        if (fill < nframes + TX_CUSHION) {
            memset(out, 0, nframes * sizeof *out);
            tx_out_count += nframes;
            gui_raw_feed(1, out, nframes);
            if (tx_tap) fwrite(out, 2, nframes, tx_tap);
            if (g_ec && !g_ec_ref) near_echo_tx(g_ec, out, (int) nframes);
            return noErr;
        }
        tx_preroll = 0;
    }
    unsigned empty_samples = 0;
    for (UInt32 i = 0; i < nframes; i++) {
        double v = 0;
        if (gui_monitor_only) { out[i] = 0; continue; }

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
        tx_out_count++;
        if (ring_r != ring_w) { out[i] = ring[ring_r]; ring_r = (ring_r + 1) % RING_N; tx_in_underrun = 0; }
        else {
            out[i] = 0;
            if (g_engine_running) {
                empty_samples++;
                tx_underrun_samples++;
                if (!tx_in_underrun) {
                    if (tx_underrun_runs < 8) tx_underrun_at[tx_underrun_runs] = tx_out_count;
                    tx_underrun_runs++;
                }
                tx_in_underrun = 1;
            }
        }
    }
    if (g_gui_control) { pthread_mutex_lock(&gui_raw_lock); gui_empty_tx += empty_samples; pthread_mutex_unlock(&gui_raw_lock); }
    gui_raw_feed(1,out,nframes);
    if (tx_tap)
        fwrite(out, 2, nframes, tx_tap);
    if (g_ec && !g_ec_ref)
        near_echo_tx(g_ec, out, (int) nframes);
    return noErr;
}

/* ------------------------------------------------------------------ */
/* CoreAudio device                                                   */
/* ------------------------------------------------------------------ */

/* Observe callback continuity without inserting/deleting received samples.
 * Successful AudioUnitRender calls alone do not establish clock continuity
 * (V.90 §5.2's network symbol clock, V.34 §5.2's symbol-rate tolerance). */
typedef struct {
    uint64_t calls, gaps, slow, max_ns;
    double next_sample, gap_frames;
    int have_sample;
} audio_callback_stats_t;
static audio_callback_stats_t input_stats, output_stats;

static uint64_t callback_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec*1000000000ULL + t.tv_nsec;
}

static void callback_record(audio_callback_stats_t *s, const AudioTimeStamp *ts,
                            UInt32 frames, uint64_t elapsed)
{
    s->calls++;
    if (ts && (ts->mFlags & kAudioTimeStampSampleTimeValid)) {
        if (s->have_sample && fabs(ts->mSampleTime-s->next_sample) > 0.5) {
            s->gaps++;
            s->gap_frames += ts->mSampleTime-s->next_sample;
        }
        s->next_sample = ts->mSampleTime + frames;
        s->have_sample = 1;
    } else s->have_sample = 0;  /* no continuity claim across unknown timestamps */
    if (elapsed > s->max_ns) s->max_ns = elapsed;
    if (elapsed > frames*1000000000.0/g_rate) s->slow++;
}

static OSStatus input_cb(void *ref, AudioUnitRenderActionFlags *flags,
                         const AudioTimeStamp *ts, UInt32 bus, UInt32 frames,
                         AudioBufferList *io)
{
    uint64_t start = callback_now_ns();
    OSStatus rc = input_cb_impl(ref, flags, ts, bus, frames, io);
    callback_record(&input_stats, ts, frames, callback_now_ns()-start);
    return rc;
}

static OSStatus output_cb(void *ref, AudioUnitRenderActionFlags *flags,
                          const AudioTimeStamp *ts, UInt32 bus, UInt32 frames,
                          AudioBufferList *io)
{
    uint64_t start = callback_now_ns();
    OSStatus rc = output_cb_impl(ref, flags, ts, bus, frames, io);
    callback_record(&output_stats, ts, frames, callback_now_ns()-start);
    return rc;
}

static void callback_report(void)
{
    const audio_callback_stats_t *s[] = {&input_stats, &output_stats};
    for (int i = 0; i < 2; i++)
        fprintf(stderr, "[APPLE] %s callbacks: %llu, timestamp gaps %llu (%+.0f frames), "
                "over budget %llu, longest %.3f ms\n", i ? "output" : "input",
                (unsigned long long)s[i]->calls, (unsigned long long)s[i]->gaps,
                s[i]->gap_frames, (unsigned long long)s[i]->slow, s[i]->max_ns/1e6);
}

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
        if (strstr(b, "Modem")) {
            const char *want = getenv("APPLE_MODEM_AUDIO_UID");
            if (want && *want) {              /* substring of the CoreAudio UID */
                CFStringRef u = NULL;
                char ub[256] = { 0 };
                UInt32 uz = sizeof u;
                AudioObjectPropertyAddress ua = { kAudioDevicePropertyDeviceUID,
                    kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
                if (AudioObjectGetPropertyData(d[i], &ua, 0, NULL, &uz, &u) == noErr && u) {
                    CFStringGetCString(u, ub, sizeof ub, kCFStringEncodingUTF8);
                    CFRelease(u);
                }
                if (!strstr(ub, want)) continue;
            }
            found = d[i];
            break;
        }
    }
    free(d);
    return found;
}

static int audio_start(void)
{
    AudioObjectID dev;

    taps_open();
    dev = find_device();
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
    fprintf(stderr, "[APPLE] native HAL nominal rate: requested %.0f, read back %.0f Hz\n", want, got);
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
    memset(&input_stats, 0, sizeof input_stats);
    memset(&output_stats, 0, sizeof output_stats);
    if ((rc = AudioOutputUnitStart(au)) != noErr) {
        fprintf(stderr, "AudioOutputUnitStart: %d\n", (int)rc); return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
static void gui_audio_stop(void)
{
    if (gui_audio_active && gui_monitor_only) line_event("monitor_stopped",@{});
    if (au) {
        AudioOutputUnitStop(au); AudioUnitUninitialize(au); AudioComponentInstanceDispose(au);
        au = NULL;
    }
    if (in_abl) { free(in_abl->mBuffers[0].mData); free(in_abl); in_abl = NULL; }
    gui_audio_active = 0;
    cid_stop();
    taps_close();
}

/* Watching begins before the first ring, including pre-ring CID systems.
 * Keep RX alive between bursts and transmit only silence while on-hook. */
static int gui_onhook_start(void)
{
    int v = reg_read(REG_HOOK);
    if (v < 0 || (v & REG_HOOK_BIT) || reg_write(REG_HOOK,(unsigned char)(v|REG_MONITOR_BIT)) < 0) {
        gui_status = "On-hook monitor unavailable"; return -1;
    }
    gui_hook = v|REG_MONITOR_BIT; gui_sense = reg_read(REG_SENSE);
    if (!gui_audio_active) {
        gui_monitor_only = 1;
        if (cid_start() < 0 || audio_start() < 0) {
            gui_audio_stop(); cid_stop(); gui_monitor_only = 0;
            reg_write(REG_HOOK,(unsigned char)v); gui_hook = v;
            gui_status = "CoreAudio monitor failed"; return -1;
        }
        gui_audio_active = 1;
    }
    line_event("monitor_started",@{});
    gui_status = "On-hook audio · listening for caller ID";
    return 0;
}
static void gui_onhook_stop(void)
{
    gui_audio_stop(); gui_monitor_only = 0;
    int v = usb ? reg_read(REG_HOOK) : -1;
    if (v >= 0 && !(v & REG_HOOK_BIT) && reg_write(REG_HOOK,(unsigned char)(v & ~REG_MONITOR_BIT)) == 0)
        gui_hook = v & ~REG_MONITOR_BIT;
    gui_status = "Idle · line on-hook";
}


static void loopback_save(NSDictionary *value)
{
    [loopback_last release]; loopback_last = [value retain];
    line_event("loopback_result",value);
}
/* A bounded raw s16le stream for the independent V.34 duplex harness.
 * Prefix silence is an explicit channel delay. Never splice/repeat samples
 * to repair a gap: count it and fail the transport instead. */
static int loopback_stream_run(int powered)
{
    size_t received = 1536*sizeof(int16_t), sent = 0;
    int flags = fcntl(loopback_peer_fd,F_GETFL,0);
    if (flags < 0 || fcntl(loopback_peer_fd,F_SETFL,flags|O_NONBLOCK) < 0) return -1;
    uint64_t deadline = apple_now_ms()+65000;
    while (!g_stop && apple_now_ms() < deadline) {
        size_t room = loopback_capacity*sizeof(int16_t)-received;
        if (!room) return -1;
        /* Read into a temporary byte buffer so complete sample publication is
         * independent of socket fragmentation and quarter-scale TX happens
         * before the callback can observe those samples. */
        unsigned char bytes[4096];
        unsigned want = room < sizeof bytes ? (unsigned)room : (unsigned)sizeof bytes;
        ssize_t n = recv(loopback_peer_fd,bytes,want,0);
        if (!n) return received%2 ? -1 : 0;
        if (n > 0) {
            memcpy((char *)loopback_samples[1]+received,bytes,(size_t)n);
            unsigned old = atomic_load(&loopback_tx_available);
            received += (size_t)n;
            unsigned complete = (unsigned)(received/2);
            if (powered) for (unsigned i = old; i < complete; i++) loopback_samples[1][i] /= 4;
            atomic_store(&loopback_tx_available,complete);
        } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return -1;
        size_t captured = atomic_load(&loopback_count[0])*sizeof(int16_t);
        if (captured > sent) {
            ssize_t written = send(loopback_peer_fd,(char *)loopback_samples[0]+sent,captured-sent,0);
            if (written > 0) sent += (size_t)written;
            else if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return -1;
        }
        if (powered && line_poll(0) < 0) return -1;
        CFRunLoopRunInMode(kCFRunLoopDefaultMode,0.001,false);
    }
    return -1;
}
/* Si3056 §5.30: loopbacks are mutually exclusive. Digital/startup stay
 * on-hook; CLI isolation/analog modes seize and calibrate the line-side.
 * Release the line and restore every touched register on every exit. */
static int hardware_loopback(const char *mode)
{
    @autoreleasepool {
        int powered = !strcmp(mode,"analog") || !strcmp(mode,"isolation");
        int hook = reg_read(REG_HOOK);
        if (hook < 0 || (hook & 1) || g_engine_running || (g_gui_control &&
            ((getenv("APPLE_MODEM_RX_TAP") && *getenv("APPLE_MODEM_RX_TAP")) ||
             (getenv("APPLE_MODEM_TX_TAP") && *getenv("APPLE_MODEM_TX_TAP"))))) {
            loopback_save(@{@"mode":@(mode),@"passed":@NO,@"status":@"refused",
                @"reason":@"Requires an idle on-hook modem; stop raw recording first"});
            return 1;
        }
        /* Si3056 §5.30 standalone probes bypass engine resamplers. */
        if (g_rate != 8000 && g_rate != 9600 && g_rate != 10286) {
            loopback_save(@{@"mode":@(mode),@"passed":@NO,@"status":@"refused",@"reason":@"Use 8000, 9600 or 10286 Hz"}); return 1;
        }
        int watch = gui_watch, monitored = gui_audio_active;
        gui_watch = 0;
        if (gui_audio_active) gui_audio_stop();
        gui_monitor_only = 0;
        const unsigned char regs[] = {1,2,6,10,15};
        int saved[5], target[5], changed = 0, restored = 1, ready = 1;
        for (int i = 0; i < 5; i++) { saved[i] = reg_read(regs[i]); target[i] = saved[i]; if (saved[i] < 0) ready = 0; }
        const char *reason = "No correlated return";
        struct apple_loop_result result = {0};
        if (ready) {
            /* Si3056 register 15: ARX in 3 dB steps; restore per probe. */
            if (loopback_rx_gain >= 0) target[4] = (target[4]&~7) | loopback_rx_gain/3;
            target[0] &= ~2; target[1] |= 2;
            target[2] = !strcmp(mode,"startup") ? target[2] | 16 : target[2] & ~16;
            target[3] = !strcmp(mode,"digital") ? target[3] | 1 : target[3] & ~1;
            line_event("loopback_started",@{@"mode":@(mode),@"hook_register":@(hook)});
            for (int i = 0; i < 5; i++) {
                changed = i+1;
                int written = reg_write(regs[i],(unsigned char)target[i]);
                int actual = reg_read(regs[i]);
                if (written < 0 || actual != target[i]) {
                    ready = 0; reason = "Test register did not read back";
                    line_event("loopback_register_rejected",@{@"register":@(regs[i]),@"requested":@(target[i]),@"readback":@(actual)});
                    break;
                }
            }
            if (ready && reg_write(REG_HOOK,(unsigned char)(hook|8)) < 0) { ready = 0; reason = "Cannot enable on-hook receive path"; }
            /* Si3056 §5.30: line-side loopbacks need off-hook plus >402.75 ms
             * calibration before setting IDL or clearing HBE. line_hook waits
             * 500 ms and checks the hardware overload guard throughout. */
            if (ready && powered) {
                if (line_hook(1) < 0) { ready = 0; reason = "Cannot power line-side test"; }
                else {
                    int index = !strcmp(mode,"isolation") ? 0 : 1;
                    target[index] = index == 0 ? target[index]|2 : target[index]&~2;
                    if (reg_write(regs[index],(unsigned char)target[index]) < 0 || reg_read(regs[index]) != target[index]) {
                        ready = 0; reason = "Test register did not read back";
                    }
                }
            }
        } else reason = "Cannot read test registers";
        if (ready) {
            loopback_capacity = (unsigned)(g_rate*(loopback_peer_fd >= 0 ? 70 : 4));
            loopback_samples[0] = calloc(loopback_capacity,sizeof(int16_t));
            loopback_samples[1] = calloc(loopback_capacity,sizeof(int16_t));
            if (!loopback_samples[0] || !loopback_samples[1]) { ready = 0; reason = "Cannot allocate test buffers"; }
        }
        if (ready) {
            loopback_count[0] = loopback_count[1] = 0;
            if (loopback_peer_fd < 0) apple_loop_pattern(loopback_samples[1],loopback_capacity,(unsigned)g_rate);
            if (!strcmp(loopback_pattern,"tone")) {
                for (unsigned i = 0; i < loopback_capacity; i++)
                    loopback_samples[1][i] = i >= g_rate && i < g_rate*2.5
                        ? (int16_t)lrint(32767*loopback_tone_level*sin(2*M_PI*1000*i/g_rate)) : 0;
            } else if (strcmp(loopback_pattern,"prbs")) {
                uint32_t state = 0x6d56a108u;
                for (unsigned i = 0; i < loopback_capacity; i++) {
                    state ^= state<<13; state ^= state>>17; state ^= state<<5;
                    int16_t value = !strcmp(loopback_pattern,"ulaw") ? ulaw_to_linear((uint8_t)state) : alaw_to_linear((uint8_t)state);
                    loopback_samples[1][i] = i >= g_rate && i < g_rate*2.5 ? value : 0;
                }
            }
            /* §5.30: analog HBE-off needs 12 dB TX headroom. Also leave
             * this margin in IDL to distinguish filter loss from saturation. */
            if (powered) for (unsigned i = 0; i < loopback_capacity; i++) loopback_samples[1][i] /= 4;
            if (loopback_peer_fd >= 0) { atomic_store(&loopback_tx_available,1536); atomic_store(&loopback_stream_gaps,0); }
            loopback_audio_active = 1;
            gui_status = "On-hook hardware loopback test";
            if (audio_start() < 0) { ready = 0; reason = "CoreAudio test could not start"; }
            else if (loopback_peer_fd >= 0) {
                if (loopback_stream_run(powered) < 0) { ready = 0; reason = "Sample stream interrupted, timed out or transport failed"; }
            } else {
                uint64_t end = apple_now_ms()+3000;
                while (!g_stop && apple_now_ms() < end) {
                    if (powered && line_poll(0) < 0) break;
                    if (g_gui_control) gui_publish();
                    CFRunLoopRunInMode(kCFRunLoopDefaultMode,0.01,false);
                }
            }
            gui_audio_stop(); /* joins the callbacks before inspecting buffers */
            loopback_audio_active = 0;
            if (ready && !g_stop && loopback_peer_fd < 0) result = apple_loop_analyze(loopback_samples[1],loopback_count[1],
                loopback_samples[0],loopback_count[0],(unsigned)g_rate);
            if (g_stop) reason = "Test interrupted";
        }
        free(loopback_samples[0]); free(loopback_samples[1]);
        loopback_samples[0] = loopback_samples[1] = NULL;
        if (powered && reg_write(REG_HOOK,(unsigned char)(hook&~1)) < 0) restored = 0;
        /* Restore even the register whose write/readback failed. */
        for (int i = changed-1; i >= 0; i--) {
            if (reg_write(regs[i],(unsigned char)saved[i]) < 0 || reg_read(regs[i]) != saved[i]) restored = 0;
        }
        if (reg_write(REG_HOOK,(unsigned char)hook) < 0 || (reg_read(REG_HOOK)&9) != (hook&9)) restored = 0;
        line_poll(1);
        int stream_pass = loopback_peer_fd >= 0 && ready && !g_stop && restored && !loopback_stream_gaps;
        int passed = stream_pass || (ready && !g_stop && restored && result.found &&
            (!strcmp(mode,"digital") ? result.mismatches == 0 :
                powered ? fabs(result.correlation) > 0.5 :
                result.correlation > 0.8 && result.gain > 0.5 && result.gain < 1.2));
        if (stream_pass) reason = "Sample stream completed; modem result is reported by peer";
        if (!restored) reason = "Register restoration failed";
        else if (result.found) reason = powered ? "Line-side return measured; sample errors are not a PCM BER" : passed ? "Known pattern returned" : "Return differs from the test pattern";
        loopback_save(@{@"rate_hz":@(g_rate),@"mode":@(mode),@"passed":@(passed),@"status":passed ? @"passed" : !strcmp(reason,"Test register did not read back") ? @"unsupported" : @"failed",
            @"stream":@(loopback_peer_fd >= 0),@"stream_tx_samples":@(loopback_count[1]),@"stream_rx_samples":@(loopback_count[0]),@"stream_gap_samples":@(loopback_stream_gaps),
            @"tone_level":@(loopback_tone_level),@"rx_gain_db":@(loopback_rx_gain),@"pattern":@(loopback_pattern),@"line_side":@(powered),@"tx_scale":@(powered ? 0.25 : 1.0),
            @"reason":@(reason),@"restored":@(restored),@"frames_compared":@(result.frames),
            @"lag_samples":@(result.lag),@"lag_ms":@(result.lag*1000.0/g_rate),
            @"correlation":@(result.correlation),@"gain":@(result.gain),@"dc":@(result.dc),
            @"rmse":@(result.rmse),@"sample_errors":@(result.mismatches),@"bit_errors":@(result.bit_errors)});
        gui_status = passed ? "Hardware loopback passed" : "Hardware loopback failed · see result";
        if (!restored) g_stop = 1;
        if (!g_stop && monitored && gui_onhook_start() < 0) {
            loopback_save(@{@"mode":@(mode),@"passed":@NO,@"status":@"failed",@"reason":@"Test completed but on-hook monitoring could not resume",@"restored":@(restored)});
            return 1;
        }
        gui_watch = !g_stop && (monitored ? gui_audio_active : 1) ? watch : 0;
        return passed ? 0 : 1;
    }
}

/* Si3056 §5.17: disable resistor recalibration during pulse makes. Preserve
 * country settings and restore calibration/protection on every exit. §5.22
 * re-arms OPE 25 ms after each make. Absolute deadlines keep
 * USB and log latency out of the nominal 10 pps cadence. Never run USB from
 * the audio callback. Abort rather than silently stretch a late pulse. */
static int pulse_setting(unsigned char reg, int value, int mask)
{
    int written = reg_write(reg,(unsigned char)value), actual = reg_read(reg);
    int accepted = written == 0 && actual >= 0 && (actual&mask) == (value&mask);
    line_event("pulse_register",@{@"register":@(reg),@"requested":@(value),@"readback":@(actual),@"mask":@(mask),@"accepted":@(accepted)});
    return accepted;
}
static int physical_dial_run(void)
{
    int calibration = reg_read(25), adc = reg_read(17), counter = reg_read(31);
    int hook = reg_read(REG_HOOK), ok = 1;
    if (calibration < 0 || adc < 0 || counter < 0 || hook < 0 || !(hook & 1)) return -1;
    if (!pulse_setting(25,calibration|32,0x6f)) ok = 0;
    /* §5.11/5.21: keep the existing ADC calibration during short makes and
     * use the 8 ms off-hook counter. Ignore read-only/sticky event bits when
     * verifying writable configuration; OPE is re-armed per §5.22. */
    if (!pulse_setting(17,adc|32,0xec) || !pulse_setting(31,counter|96,0xef)) ok = 0;
    line_event("pulse_dial_started",@{@"digits":@(physical_plan.digits),@"pulses":@(physical_plan.pulses),@"pps":@10,@"break_ms":@60,@"make_ms":@40});
    uint64_t deadline = apple_now_ms()+300;
    while (!g_stop && apple_now_ms() < deadline) usleep(1000);
    unsigned worst_late = 0;
    for (unsigned i = 0; ok && !g_stop && i < physical_plan.count; i++) {
        struct apple_dial_step step = physical_plan.step[i];
        uint64_t now = apple_now_ms();
        unsigned late = now > deadline ? (unsigned)(now-deadline) : 0;
        if (late > worst_late) worst_late = late;
        if (late > 10) { ok = 0; break; }
        atomic_store(&dial_control_tone,step.kind == APPLE_DIAL_TONE ? step.digit : 0);
        if (step.kind == APPLE_DIAL_BREAK || step.kind == APPLE_DIAL_MAKE) {
            int v = step.kind == APPLE_DIAL_BREAK ? hook & ~1 : hook | 1;
            if (step.kind == APPLE_DIAL_BREAK && reg_write(17,(unsigned char)((adc|32)&~8)) < 0) { ok = 0; break; }
            if (reg_write(REG_HOOK,(unsigned char)v) < 0) { ok = 0; break; }
            line_event("pulse_edge",@{@"off_hook":@(!!(v&1)),@"digit":@((int)step.digit-'0'),@"step":@(i)});
        }
        deadline += step.ms;
        if (step.kind == APPLE_DIAL_MAKE && (adc&8)) {
            /* §5.22: arm OPE after the make transient, not before it. */
            uint64_t armed = apple_now_ms()+25;
            while (!g_stop && apple_now_ms() < armed) usleep(1000);
            if (reg_write(17,(unsigned char)(adc|32)) < 0) ok = 0;
        }
        /* Readbacks/overload checks belong in the interdigit gap, not the
         * 40 ms make. Hardware overcurrent protection remains enabled. */
        if (step.kind == APPLE_DIAL_WAIT && step.ms == 600) {
            if (line_poll(1) < 0 || (line_status.hook&1) != 1) ok = 0;
        }
        while (ok && !g_stop && apple_now_ms() < deadline) usleep(1000);
    }
    atomic_store(&dial_control_tone,0);
    if (!ok || g_stop) reg_write(REG_HOOK,(unsigned char)(hook&~1));
    int restored = 1;
    if (!pulse_setting(31,counter,0xef)) restored = 0;
    if (!pulse_setting(17,adc,0xec)) restored = 0;
    if (!pulse_setting(25,calibration,0x6f)) restored = 0;
    if (!restored) { reg_write(REG_HOOK,(unsigned char)(hook&~1)); ok = 0; }
    line_event("pulse_dial_result",@{@"completed":@(ok && !g_stop),@"calibration_restored":@(restored),@"max_late_ms":@(worst_late)});
    return ok && !g_stop ? 0 : -1;
}

/* Commands arrive over launcher-owned stdin. Only the control loop executes
 * USB/CoreAudio operations; malformed commands have no device side effects. */
static void gui_commands(void)
{
    static char pending[128]; static size_t used;
    if (gui_control_fd < 0) return;
    char c;
    while (read(gui_control_fd,&c,1) == 1) {
        if (c != '\n') { if (used+1 < sizeof pending) pending[used++] = c; continue; }
        pending[used] = 0; used = 0;
        if (strcmp(pending,"probe") && strcmp(pending,"monitor1") && strcmp(pending,"monitor0") && strcmp(pending,"watch1") && strcmp(pending,"watch0") && strcmp(pending,"loopback")) continue;
        if (!strcmp(pending,"monitor0")) { gui_manual_monitor = 0; if (!gui_watch) gui_onhook_stop(); continue; }
        if (!strcmp(pending,"watch0")) { gui_watch = 0; line_event("ring_watch",@{@"enabled":@NO}); if (!gui_manual_monitor) gui_onhook_stop(); continue; }
        if (usb_open_line() < 0) { gui_status = "No selected USB modem"; continue; }
        if (!strcmp(pending,"loopback")) {
            hardware_loopback("digital");
        } else if (!strcmp(pending,"probe")) {
            line_poll(1);
            if (!line_guard_tripped) gui_status = "Idle · line checked";
        } else if (!strcmp(pending,"watch1")) {
            if (gui_onhook_start() < 0) continue;
            gui_watch = 1; line_event("ring_watch",@{@"enabled":@YES}); gui_rings = 0; gui_last_ring = 0;
            memset(&cid_current,0,sizeof cid_current); cid_received_ms = 0;
            di_clear_caller_id();
        } else if (!strcmp(pending,"monitor1")) {
            if (gui_onhook_start() == 0) gui_manual_monitor = 1;
        }
    }
}
static int gui_ring_burst(uint64_t now, uint64_t *last)
{
    int fresh = !*last || now-*last > 1500;
    *last = now;
    return fresh;
}
static int gui_recent_ring(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    uint64_t now = (uint64_t)ts.tv_sec*1000+ts.tv_nsec/1000000;
    return gui_last_ring && now-gui_last_ring < 8000;
}
static void gui_ring_notice(void)
{
    ring_notifications++;
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC,&ts);
    uint64_t now = (uint64_t)ts.tv_sec*1000+ts.tv_nsec/1000000;
    // One ring burst, rather than one USB notification. Notifications within
    // 1.5 s extend the burst; this is host observation, not a DAA voltage code.
    cid_deliver();
    if (gui_last_ring && now-gui_last_ring > 8000) {
        gui_rings = 0;
        if (!cid_received_ms || now-cid_received_ms > 8000) {
            memset(&cid_current,0,sizeof cid_current); cid_received_ms = 0;
            di_clear_caller_id();
        }
    }
    if (gui_ring_burst(now,&gui_last_ring)) { gui_rings++; line_event("ring",@{@"burst":@(gui_rings),@"notifications":@(ring_notifications)}); di_on_ring(); }
}
static void gui_ring_poll(void)
{
    if (!gui_watch || !usb) return;
    unsigned char b[16]; int n = 0;
    if (libusb_interrupt_transfer(usb,0x81,b,sizeof b,&n,1) == 0) line_notification(b,n);
}

/* Offline replay -- the way to work on this with no line             */
/* ------------------------------------------------------------------ */

static int replay(const char *path)
{
    FILE *fp = fopen(path, "rb");
    int16_t buf[4096];
    /* Live CoreAudio delivers 512-frame cycles; replay's default 80 hides
     * transmit-ring effects that only a whole cycle's demand exposes. */
    const char *chunk_env = getenv("APPLE_REPLAY_CHUNK");
    size_t chunk = chunk_env ? (size_t)atoi(chunk_env) : 80;
    if (chunk < 1 || chunk > 4096) chunk = 80;
    long total = 0;

    if (!fp) { perror(path); return 1; }
    {
        const char *ref = getenv("APPLE_MODEM_EC_REF");
        if (ref && *ref && g_ec && !(g_ec_ref = fopen(ref, "rb")))
            perror(ref);
        if (g_ec && !g_ec_ref)
            fprintf(stderr, "[APPLE] echo canceller has no reference in replay "
                            "(set APPLE_MODEM_EC_REF to the call's tx-tap.s16); it will not fit\n");
    }
    /* Goes through engine_feed() exactly as the live path does -- the DC
     * blocker, both resamplers, the answer detector and the 80-sample blocking.
     * An earlier version duplicated a shortened version of that here and fed
     * the file straight to me_rx_audio(): at a device rate of 9600 that handed
     * 9600 Hz samples to an 8000 Hz entry point, and it "passed" only because
     * the answer detector was mis-tuned by the same ratio.  Two wrongs.  The tap
     * must be at --rate, since that is what the device produced. */
    for (; !g_stop;) {
        size_t n = fread(buf, 2, chunk, fp);
        if (n == 0) break;
        gui_raw_feed(0,buf,(unsigned)n);
        if (g_ec && g_ec_ref) {
            int16_t ref[4096];
            size_t m = fread(ref, 2, n, g_ec_ref);
            for (size_t k = m; k < n; k++) ref[k] = 0;
            near_echo_tx(g_ec, ref, (int) n);
        }
        engine_feed(buf, (long)n);
        if (g_gui_control) { gui_status = "Offline replay · no USB line"; gui_publish(); CFRunLoopRunInMode(kCFRunLoopDefaultMode,(double)n/g_rate,false); }
        /* No line to transmit into, but drain the ring as a device whose input
         * and output share one clock would: n samples out per n in.  That
         * makes transmit-ring underruns -- each a one-sample slip in what the
         * far end receives -- reproducible offline. */
        if (tx_preroll && g_engine_running) {
            if ((ring_w + RING_N - ring_r) % RING_N < n + TX_CUSHION) {
                tx_out_count += n;
                total += (long)n;
                continue;
            }
            tx_preroll = 0;
        }
        for (size_t i = 0; i < n; i++) {
            tx_out_count++;
            if (ring_r != ring_w) { ring_r = (ring_r + 1) % RING_N; tx_in_underrun = 0; }
            else if (g_engine_running) {
                tx_underrun_samples++;
                if (!tx_in_underrun) {
                    if (tx_underrun_runs < 8) tx_underrun_at[tx_underrun_runs] = tx_out_count;
                    tx_underrun_runs++;
                }
                tx_in_underrun = 1;
            }
        }
        total += (long)n;
    }
    fclose(fp);
    fprintf(stderr, "[APPLE] replay: %ld samples at %.0f Hz (%.2f s), engine %s\n",
            total, g_rate, total / g_rate, g_engine_running ? "ran" : "NEVER STARTED");
    fprintf(stderr, "[APPLE] replay transmit ring underruns: %lu runs, %lu samples",
            (unsigned long)tx_underrun_runs, (unsigned long)tx_underrun_samples);
    for (unsigned long i = 0; i < tx_underrun_runs && i < 8; i++)
        fprintf(stderr, "%s%.3f s", i ? ", " : " (first at ", tx_underrun_at[i] / g_rate);
    fprintf(stderr, "%s\n", tx_underrun_runs ? ")" : "");
    fprintf(stderr, "[APPLE] transmit ring overruns: %lu samples\n", tx_overrun_samples);
    return line_guard_tripped ? 1 : g_engine_running ? 0 : 1;
}

/* ------------------------------------------------------------------ */

/* Physical codec conversion; the negotiated upstream T/3 rate remains
 * independent of the device rate (V.90 5.2, V.34 5.2). Native 9000 does
 * not select 3000 baud: a 3200-baud upstream still uses 9600 internally. */
static void rs_setup(void)
{
    if (g_rate == 8000.0) {
        const char *pcm = getenv("APPLE_MODEM_8K_PCM");
        g_pcm8k = pcm && !strcmp(pcm, "1");
        /* This interpolates a native ANALOGUE waveform for the existing
         * T/2 detector, not a G.711 bearer. It creates no bandwidth above
         * 4 kHz. The 2026-10-17 direct-pair PR4 probe decodes a 16-level
         * alphabet at native 8 kHz without the 9600-Hz TX interpolator. */
        if (g_pcm8k) {
            rs_init(&rs_16k, 2, 1);
            fprintf(stderr, "[APPLE] native 8000 Hz PCM experiment: 2/1 T/2 waveform interpolation enabled\n");
            ec_setup();
        }
        return;
    }
    if (g_rate != 9600.0 && g_rate != 9000.0) {
        fprintf(stderr, "[APPLE] %.0f Hz has no small exact ratio to 8000 or "
                        "16000; use 9600, 9000 (or 8000 for V.34 only)\n", g_rate);
        exit(2);
    }
    /* Physical codec conversion only; the SIP G.711 bearer is untouched.
     * 9000 Hz is also native: 8/9 and 16/9 in, 9/8 out. */
    rs_init(&rs_8k,  g_rate == 9000.0 ? 8 : 5, g_rate == 9000.0 ? 9 : 6);
    rs_init(&rs_16k, g_rate == 9000.0 ? 16 : 5, g_rate == 9000.0 ? 9 : 3);
    rs_init(&rs_tx,  g_rate == 9000.0 ? 9 : 6, g_rate == 9000.0 ? 8 : 5);
    ec_setup();

    const char *lt = getenv("APPLE_MODEM_LOOP_TIMING");
    const char *forced = getenv("APPLE_MODEM_CLOCK_PPM");
    /* ANSam is a remote reference only when we originate. An answerer
     * receives its own 2100 Hz echo, which measures the local clock against
     * itself and cannot establish peer/network clock lock. V.90 5.2 derives
     * downstream timing from the digital network; V.90 9.2.1.1.2 instead
     * has this digital answerer receive the peer's Phase 2 Tone A.
     * Keep forced ppm available for measured physical-clock experiments. */
    lt_enabled = !g_answer_mode && !(lt && strcmp(lt, "0") == 0);
    lt_done = 0; lt_n = 0; lt_fill = 0; lt_misses = 0; lt_block_index = 0;
    lt_re = lt_im = lt_energy = lt_osc = 0;
    if (forced && *forced) {
        loop_timing_apply(atof(forced), "APPLE_MODEM_CLOCK_PPM");
    } else if (!lt_enabled) {
        lt_done = 1;
        fprintf(stderr, "[APPLE] loop timing: off (%s); "
                        "transmitting on the codec's free-running clock\n",
                g_answer_mode ? "answerer has no remote ANSam reference"
                              : "APPLE_MODEM_LOOP_TIMING=0");
    }
}

/* The resamplers are load-bearing, so they are measurable without a device:
 * a tone at the device rate through each path, reporting the recovered
 * frequency, the level and the residual after the ideal tone is subtracted. */
static int selftest(void)
{
    audio_callback_stats_t continuity = {0};
    AudioTimeStamp stamp = {.mSampleTime=100, .mFlags=kAudioTimeStampSampleTimeValid};
    callback_record(&continuity, &stamp, 512, 0);
    stamp.mSampleTime += 512;
    callback_record(&continuity, &stamp, 512, 0);
    stamp.mSampleTime += 640;
    callback_record(&continuity, &stamp, 512, 1000000000ULL);
    if (continuity.calls != 3 || continuity.gaps != 1 || continuity.gap_frames != 128
        || continuity.slow != 1 || continuity.max_ns != 1000000000ULL) return 1;
    stamp.mFlags = 0;
    callback_record(&continuity, &stamp, 512, 0);
    stamp.mFlags = kAudioTimeStampSampleTimeValid;
    stamp.mSampleTime += 1024;
    callback_record(&continuity, &stamp, 512, 0);
    if (continuity.gaps != 1) return 1;
    struct { const char *name; int L, M; double out_rate; } c[] = {
        { "9600 -> 8000  (me_rx_audio)",     5, 6, 8000.0  },
        { "9600 -> 16000 (me_rx_v90a_16k)",  5, 3, 16000.0 },
        { "8000 -> 9600  (transmit)",        6, 5, 9600.0  },
        { "8000 -> 16000 (PCM experiment)",  2, 1, 16000.0 },
        { "9000 -> 8000  (me_rx_audio)",     8, 9, 8000.0  },
        { "9000 -> 16000 (me_rx_v90a_16k)", 16, 9, 16000.0 },
        { "8000 -> 9000  (transmit)",        9, 8, 9000.0  },
    };
    const double in_rate[] = { 9600.0, 9600.0, 8000.0, 8000.0, 9000.0, 9000.0, 8000.0 };
    int bad = 0;

    /* Loop timing scales every ratio by the codec's offset, so the off-nominal
     * rows matter as much as the nominal ones: a tone must come out at exactly
     * f times the factor, at full gain, with nothing left over.  +/-200 ppm is
     * past the -165 ppm this hardware measures. */
    static const double ppms[] = { 0.0, +200.0, -200.0 };
    for (unsigned pi = 0; pi < sizeof ppms / sizeof ppms[0]; pi++)
    for (unsigned t = 0; t < sizeof c / sizeof c[0]; t++) {
        /* 3673 Hz is V.34's worst case: 3429 baud at fc 1959 reaches it, and it
         * is 327 Hz from the 8 kHz grid's Nyquist, so it is the frequency that
         * decides whether this bearer can carry the top symbol rate. */
        static const double freqs[] = { 300, 1000, 2000, 3000, 3520, 3673 };
        double d = ppms[pi] * 1e-6;
        /* Receive paths take (1+d) input per output, transmit 1/(1+d). */
        double factor = (t == 2 || t == 6) ? 1.0 / (1.0 + d) : 1.0 + d;
        for (unsigned fi = 0; fi < sizeof freqs / sizeof freqs[0]; fi++) {
            double f = freqs[fi], fo = f * factor;
            static struct resamp r;
            double *y = malloc(sizeof(double) * 200000);
            long ny = 0;
            /* Off nominal the position sweeps through every phase and wraps a
             * whole input sample once per 1/(ppm * phases-per-output) outputs --
             * 0.63 s at 165 ppm on the receive path.  Half a second never sees a
             * wrap, so those rows run long enough to cross several. */
            long nin = (long)(in_rate[t] * (ppms[pi] != 0.0 ? 3.0 : 0.5));

            if (ppms[pi] != 0.0 && f != 1000 && f != 3673)
                continue;
            rs_init(&r, c[t].L, c[t].M);
            rs_set_factor(&r, factor);
            for (long i = 0; i < nin; i++) {
                double o[8];
                int got = rs_put(&r, 10000.0 * sin(2 * M_PI * f * i / in_rate[t]), o, 8);
                for (int j = 0; j < got; j++) y[ny++] = o[j];
            }
            double expected_n = nin*c[t].out_rate/in_rate[t]/factor;
            if (fabs(ny-expected_n) > 2.0) {
                printf("FAIL sample count: %s got %ld expected %.3f\n",
                       c[t].name, ny, expected_n);
                bad++;
            }
            /* Skip the filter's transient, then fit amplitude and phase at the
             * expected frequency and report what is left: a resampler wrong in
             * gain, in phase slope, in ratio or in aliasing all show up in the
             * residual. */
            long s0 = 4 * RS_TAPS * c[t].M / c[t].L + 64, n = ny - s0 - 64;
            double sc = 0, ss = 0, e = 0, pw = 0;
            for (long i = 0; i < n; i++) {
                double ph = 2 * M_PI * fo * i / c[t].out_rate;
                sc += y[s0 + i] * cos(ph);
                ss += y[s0 + i] * sin(ph);
                pw += y[s0 + i] * y[s0 + i];
            }
            sc *= 2.0 / n; ss *= 2.0 / n;
            double amp = sqrt(sc * sc + ss * ss);
            for (long i = 0; i < n; i++) {
                double ph = 2 * M_PI * fo * i / c[t].out_rate;
                double fit = sc * cos(ph) + ss * sin(ph);
                e += (y[s0 + i] - fit) * (y[s0 + i] - fit);
            }
            double sndr = 10 * log10((pw / n) / fmax(1e-12, e / n));
            int bad_row = fabs(amp / 10000.0 - 1.0) > 0.03 || sndr < 40.0;
            printf("  %-34s %+5.0f ppm %4.0f Hz: gain %6.4f  SNDR %5.1f dB%s\n",
                   fi == 0 || (ppms[pi] != 0.0 && f == 1000) ? c[t].name : "",
                   ppms[pi], f, amp / 10000.0, sndr, bad_row ? "   <- BAD" : "");
            bad += bad_row;
            free(y);
        }
    }
    printf("%s\n", bad ? "SELFTEST FAILED" : "selftest ok");
    return bad ? 1 : 0;
}

/* Wait for an incoming call.  The ring is NOT in the audio path at all -- an
 * on-hook capture is just the codec's noise floor -- but interface 0's
 * interrupt endpoint delivers CDC RING_DETECT (0xa1 0x09) throughout it, and
 * nothing whatever when no call is arriving, so this is the only way to know.
 * Returns 0 once ringing, -1 on timeout.  Answering itself is just off-hook;
 * no DTMF is involved, which is why a line too poor to dial can still take a
 * call. */
static int wait_for_ring(double secs)
{
    unsigned char b[16];
    int n, rings = 0;
    uint64_t last_ring = 0, deadline = apple_now_ms()+(uint64_t)(secs*1000);

    libusb_claim_interface(usb, 0);
    fprintf(stderr, "[APPLE] waiting up to %.0f s for a ring\n", secs);
    while (apple_now_ms() < deadline && !g_stop) {
        if (line_poll(0) < 0) break;
        gui_publish();
        if (libusb_interrupt_transfer(usb, 0x81, b, sizeof b, &n, 500) != 0)
            continue;
        line_notification(b,n);
        if (line_overload_notice) { line_poll(1); break; }
        if (n >= 2 && b[0] == 0xa1 && b[1] == 0x09) {
            if (gui_ring_burst(apple_now_ms(),&last_ring)) {
                rings++;
                line_event("ring",@{@"burst":@(rings),@"notifications":@(ring_notifications)});
                if (!g_gui_control) di_on_ring();
                fprintf(stderr,"[APPLE] RING %d/%u\n",rings,answer_rings);
            }
            if ((unsigned)rings >= answer_rings) return 0;
        }
    }
    fprintf(stderr, "[APPLE] no ring within %.0f s\n", secs);
    return -1;
}

static void on_signal(int sig) { (void)sig; g_stop = 1; }

static void usage(const char *a0)
{
    fprintf(stderr,
        "usage: %s --dial NUMBER [--pty-link PATH] [--rate HZ] [--hold SECS]\n"
        "       %s --answer [--ring-wait SECS] [--pty-link PATH] [--hold SECS]\n"
        "       %s --rx-replay TAP.s16 [--pty-link PATH]\n"
        "       %s --hook on|off\n"
        "       %s --selftest\n"
        "\n"
        "  --line-events FILE  append timestamped line/engine events as JSONL\n"
        "  --loopback-test digital|startup|isolation|analog  hardware loopback\n"
        "               isolation/analog seize the line briefly, then restore it\n"
        "  --loopback-pattern tone|prbs|ulaw|alaw  G.711 requires --rate 8000\n"
        "  --loopback-tone-level 0..1  1 kHz probe level before line-side /4\n"
        "  --loopback-rx-gain 0|3|6|9|12  temporary ARX gain; restored afterwards\n"
        "  --loopback-peer-fd N  stream s16le to a test peer socket (8000 Hz)\n"
        "  --devices    read-only USB/CoreAudio device inventory as JSON\n"
        "  --diagnostic-selftest  check passive raw taps without hardware\n"
        "  --gui-replay PATH  offline raw RX replay with GUI split consoles\n"
        "  --gui-control wait on split AT/data PTYs for ATD or ATA; one call per\n"
        "               process, with passive ME_GUI_PORT telemetry\n"
        "  --control-link PATH --data-link PATH  split consoles for GUI mode\n"
        "  --dial       seize the line, dial (P=pulse/T=tone), wait for 2100 Hz, run\n"
        "               the engine.  Goes back on-hook on exit or SIGINT\n"
        "  --answer-rings N  answer after N bursts (1 default; use 2 for inter-ring CID)\n"
        "  --answer     wait for CDC RING_DETECT on the interrupt endpoint, go\n"
        "               off-hook and run the engine as the answering modem.\n"
        "               Answering needs no DTMF, so it works on a line too poor\n"
        "               to dial out on\n"
        "  --answer-now answer immediately without ringing on a powered direct pair\n"
        "  --rx-replay  run the analogue side offline against a recorded tap;\n"
        "               needs no line and no device\n"
        "  --hook       line control only, then exit\n"
        "  --rate       9600 (default) or native 9000; 9600 uses 5/6 to\n"
        "               8000 and 5/3 to 16000; 8000 skips the receive resampler\n"
        "               but cannot feed the T/2 path at all.  See the header\n"
        "  --cid-replay FILE  decode raw on-hook s16le CID at --rate, no USB\n"
        "  --selftest   measure the resamplers; needs no device and no line\n"
        "\n"
        "APPLE_MODEM_TX_AMP per-tone DTMF amplitude (default 0.15)\n"
        "APPLE_MODEM_DTMF_MS on/off times in ms (default 100/120)\n"
        "APPLE_MODEM_TX_GAIN engine transmit gain into the DAA (default 1.0)\n"
        "APPLE_MODEM_8K_PCM=1 opt in to native-8k T/2 waveform interpolation\n"
        "APPLE_MODEM_RX_TAP    record the raw device receive stream at --rate;\n"
        "                      this is exactly what --rx-replay consumes\n"
        "APPLE_MODEM_TX_TAP    record what went on the line, for telling our own\n"
        "                      echo from the far end\n"
        "APPLE_MODEM_ADDR       USB bus:addr, when two modems are present\n"
        "APPLE_MODEM_AUDIO_UID  substring of that modem's CoreAudio UID\n",
        a0, a0, a0, a0, a0);
}

static int gui_diagnostic_selftest(void)
{
    @autoreleasepool {
        g_gui_control = 1;
        int16_t signal[4096], before[4096];
        const int16_t pattern[] = {1000,-1000,32767,-32768};
        for (unsigned i = 0; i < 4096; i++) signal[i] = pattern[i%4];
        memcpy(before,signal,sizeof signal);
        gui_raw_feed(0,signal,4096);
        if (memcmp(signal,before,sizeof signal)) return 1;
        NSString *json = gui_raw_json();
        NSDictionary *v = [NSJSONSerialization JSONObjectWithData:[json dataUsingEncoding:NSUTF8StringEncoding] options:0 error:NULL];
        NSArray *audio = v[@"audio"], *listen = v[@"listen"], *metrics = v[@"metrics"];
        double expected = sqrt((1000000.0+1000000.0+32767.0*32767.0+32768.0*32768.0)/4-0.0625);
        if ([audio[0] count] != 512 || [audio[1] count] != 0 ||
            [listen[0][@"count"] unsignedLongLongValue] != 4096 || [listen[0][@"hex"] length] != 6400 ||
            ![listen[0][@"hex"] hasPrefix:@"e80318fcff7f0080"] ||
            [metrics[0][@"clipped"] unsignedLongLongValue] != 2048 ||
            fabs([metrics[0][@"dc"] doubleValue]+0.25) > 1e-9 ||
            fabs([metrics[0][@"rms"] doubleValue]-expected) > 1e-9 ||
            fabs([metrics[0][@"ac_dbfs"] doubleValue]-20*log10(expected/32768.0)) > 1e-9 ||
            ![metrics[0][@"status"] isEqual:@"near rail"] ||
            ![metrics[1][@"status"] isEqual:@"no samples"] ||
            [metrics[0][@"rail_samples"] intValue] != GUI_RAW_N/2 ||
            [json length] > 25000) return 1;
        /* Cross the circular-buffer boundary with each failure signature. */
        const int16_t constants[] = {-32768, 0, 657};
        NSArray *statuses = @[@"constant rail", @"digital silence", @"constant input"];
        for (unsigned k = 0; k < 3; k++) {
            for (unsigned i = 0; i < 4096; i++) signal[i] = constants[k];
            gui_raw_feed(0,signal,4096);
            NSDictionary *snapshot = [NSJSONSerialization JSONObjectWithData:[gui_raw_json() dataUsingEncoding:NSUTF8StringEncoding] options:0 error:NULL];
            NSDictionary *m = snapshot[@"metrics"][0];
            if (![m[@"status"] isEqual:statuses[k]] || [m[@"min"] intValue] != constants[k] ||
                [m[@"max"] intValue] != constants[k] || [m[@"rms"] doubleValue] != 0 ||
                m[@"ac_dbfs"] != [NSNull null] || [m[@"window_samples"] intValue] != GUI_RAW_N ||
                [m[@"zero_samples"] intValue] != (k == 1 ? GUI_RAW_N : 0) ||
                [m[@"rail_samples"] intValue] != (k == 0 ? GUI_RAW_N : 0)) return 1;
        }
        apple_modem_line_t status = {1,0x5f,0xff,0x14,0};
        if (!apple_line_overcurrent(&status) || apple_line_signed_sense(0xd8) != -40 ||
            apple_line_signed_sense(0x32) != 50) return 1;
        status.status = -1; if (apple_line_overcurrent(&status) || apple_line_valid(&status)) return 1;
        status.status = 0x1f; if (apple_line_overcurrent(&status)) return 1;
        status.status = 0x5f; status.hook = 0; if (apple_line_overcurrent(&status)) return 1;
        status.hook = 1; status.status = 0x4a; if (apple_line_overcurrent(&status)) return 1;
        status.auxiliary = 1; if (!apple_line_overcurrent(&status)) return 1;
        status.auxiliary = 4; if (apple_line_overcurrent(&status)) return 1;
        status.auxiliary = -1; if (apple_line_overcurrent(&status)) return 1;
        const unsigned char overload[] = {0xa1,0x29,9,0};
        line_notification(overload,2); if (line_overload_notice) return 1;
        line_notification(overload,4); if (!line_overload_notice) return 1;
        line_overload_notice = 0;
        uint64_t last = 0;
        if (!gui_ring_burst(100,&last) || gui_ring_burst(1000,&last) ||
            gui_ring_burst(2000,&last) || !gui_ring_burst(4000,&last)) return 1;

        /* Independent transport faults must not pass the PRBS comparison. */
        struct apple_dial_plan dial;
        if (apple_dial_build(&dial,"P10T#P2",70,70) || dial.pulses != 13 || dial.digits != 4 ||
            dial.step[0].kind != APPLE_DIAL_BREAK || dial.step[0].ms != 60 ||
            dial.step[1].kind != APPLE_DIAL_MAKE || dial.step[1].ms != 40 ||
            apple_dial_build(&dial,"P*",70,70) == 0 || apple_dial_build(&dial,"P",70,70) == 0 ||
            apple_dial_build(&dial,"T12!",70,70) == 0) return 1;
        const unsigned test_rate = 8000, test_n = 24000, delay = 123;
        int16_t *reference = calloc(test_n,sizeof(int16_t));
        int16_t *received = calloc(test_n+delay,sizeof(int16_t));
        if (!reference || !received) { free(reference); free(received); return 1; }
        apple_loop_pattern(reference,test_n,test_rate);
        memcpy(received+delay,reference,test_n*sizeof(int16_t));
        struct apple_loop_result result = apple_loop_analyze(reference,test_n,received,test_n+delay,test_rate);
        if (!result.found || result.lag != (int)delay || result.mismatches || result.bit_errors || result.frames < 8000) return 1;
        received[test_rate*3/2+delay] ^= 1;
        result = apple_loop_analyze(reference,test_n,received,test_n+delay,test_rate);
        if (!result.found || result.mismatches != 1 || result.bit_errors != 1) return 1;
        memset(received,0,(test_n+delay)*sizeof(int16_t));
        result = apple_loop_analyze(reference,test_n,received,test_n+delay,test_rate);
        if (result.found) return 1;
        result = apple_loop_analyze(reference,test_n,received,100,test_rate);
        if (result.found) return 1;
        for (unsigned i = 0; i < test_n; i++) received[i+delay] = (int16_t)lrint(reference[i]*0.9);
        result = apple_loop_analyze(reference,test_n,received,test_n+delay,test_rate);
        if (!result.found || result.correlation < 0.999 || fabs(result.gain-0.9) > 0.001 || !result.mismatches) return 1;
        free(reference); free(received);
        line_status = (apple_modem_line_t){0,0x40,50,0x14,2}; line_protection_register = 12;
        line_observe(); uint64_t seq = line_event_seq;
        line_observe(); if (line_event_seq != seq) return 1;
        line_status.sense = 0; line_observe();
        if (![line_events.lastObject[@"event"] isEqual:@"voltage_changed"]) return 1;
        line_status.status = -1; line_observe();
        if (![line_events.lastObject[@"event"] isEqual:@"link_lost"]) return 1;
        line_status.status = 0x40; line_status.sense = 206; line_observe();
        line_status.sense = 50; line_observe();
        if (![line_events.lastObject[@"event"] isEqual:@"polarity_reversed"]) return 1;
        for (unsigned i = 0; i < 140; i++) line_event("test_event",@{@"index":@(i)});
        if (line_events.count != 128 || [line_events.lastObject[@"seq"] unsignedLongLongValue]-[line_events.firstObject[@"seq"] unsignedLongLongValue] != 127 || [line_events_json() count] != 16) return 1;
        line_event_path = test_tmp("apple-line-events.jsonl");
        line_event_file = fopen(line_event_path,"w"); if (!line_event_file) return 1;
        for (unsigned i = 0; i < 2*1024*1024; i++) fputc(' ',line_event_file);
        fflush(line_event_file);
        line_event("rotation_test",@{}); line_events_close();
        NSData *event_data = [NSData dataWithContentsOfFile:@(line_event_path)];
        NSDictionary *saved_event = [NSJSONSerialization JSONObjectWithData:event_data options:0 error:NULL];
        if (![saved_event[@"event"] isEqual:@"rotation_test"] || !saved_event[@"utc"] || !saved_event[@"monotonic_ms"] || !saved_event[@"process_id"]) return 1;
        puts("Apple loopback analyzer: exact transport, single-bit corruption, silence and filtered return checks passed; event transitions, bounded history and rotation passed");
        puts("Apple passive raw tap: bounded history, immutable input, endian order, DC/RMS/clipping and ring-burst checks passed");
        return 0;
    }
}


/* Offline CID decoding uses the same on-hook path, with no USB operations. */
static int cid_replay_file(const char *path)
{
    @autoreleasepool {
        if (g_rate != 8000 && g_rate != 9600) { fprintf(stderr,"CID rate must be 8000 or 9600\n"); return 2; }
        FILE *fp = fopen(path,"rb");
        if (!fp) { perror(path); return 1; }
        if (cid_start() < 0) { fclose(fp); return 1; }
        int16_t samples[511]; size_t n;
        while ((n = fread(samples,sizeof *samples,511,fp)) > 0) { cid_feed(samples,(unsigned)n); cid_deliver(); }
        int failed = ferror(fp); fclose(fp); cid_stop();
        NSData *data = [NSJSONSerialization dataWithJSONObject:cid_json() options:0 error:NULL];
        fwrite(data.bytes,1,data.length,stdout); putchar('\n');
        return failed ? 1 : 0;
    }
}

static int gui_devices(void)
{
    @autoreleasepool {
    NSMutableArray *usbDevices = [NSMutableArray array], *audioDevices = [NSMutableArray array];
    if (libusb_init(NULL) == 0) {
        libusb_device **list; ssize_t n = libusb_get_device_list(NULL,&list);
        for (ssize_t i = 0; i < n; i++) {
            struct libusb_device_descriptor d;
            if (libusb_get_device_descriptor(list[i],&d) == 0 && d.idVendor == VID && d.idProduct == PID) {
                NSString *address = [NSString stringWithFormat:@"%u:%u",libusb_get_bus_number(list[i]),libusb_get_device_address(list[i])];
                [usbDevices addObject:@{@"id":address,@"label":[@"Apple A1082 · " stringByAppendingString:address]}];
            }
        }
        if (n >= 0) libusb_free_device_list(list,1);
        libusb_exit(NULL);
    }
    AudioObjectPropertyAddress a = {kAudioHardwarePropertyDevices,kAudioObjectPropertyScopeGlobal,kAudioObjectPropertyElementMain};
    UInt32 bytes = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject,&a,0,NULL,&bytes) == noErr && bytes) {
        AudioObjectID *ids = malloc(bytes);
        if (AudioObjectGetPropertyData(kAudioObjectSystemObject,&a,0,NULL,&bytes,ids) == noErr) {
            for (unsigned i = 0; i < bytes/sizeof(*ids); i++) {
                CFStringRef name = NULL, uid = NULL; UInt32 len = sizeof(name);
                AudioObjectPropertyAddress p = {kAudioObjectPropertyName,kAudioObjectPropertyScopeGlobal,kAudioObjectPropertyElementMain};
                if (AudioObjectGetPropertyData(ids[i],&p,0,NULL,&len,&name) != noErr || !name) continue;
                p.mSelector = kAudioDevicePropertyDeviceUID; len = sizeof(uid);
                AudioObjectGetPropertyData(ids[i],&p,0,NULL,&len,&uid);
                NSString *label = (__bridge NSString *)name;
                if (uid && [label rangeOfString:@"Modem" options:NSCaseInsensitiveSearch].location != NSNotFound)
                    [audioDevices addObject:@{@"id":(__bridge NSString *)uid,@"label":label}];
                CFRelease(name); if (uid) CFRelease(uid);
            }
        }
        free(ids);
    }
    NSData *json = [NSJSONSerialization dataWithJSONObject:@{@"usb":usbDevices,@"audio":audioDevices} options:0 error:NULL];
    fwrite(json.bytes,1,json.length,stdout); putchar('\n'); return 0;
    }
}

int main(int argc, char **argv)
{
    const char *hook_arg = NULL, *cid_replay_path = NULL, *loopback_mode = NULL;
    double hold = 60.0, on_ms = 100.0, off_ms = 120.0, ring_wait = 60.0;
    int rc = 1;
    int answer_now = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--diagnostic-selftest")) return gui_diagnostic_selftest();
        else if (!strcmp(argv[i], "--cid-replay") && i+1 < argc) cid_replay_path = argv[++i];
        else if (!strcmp(argv[i], "--loopback-test") && i+1 < argc) loopback_mode = argv[++i];
        else if (!strcmp(argv[i], "--line-events") && i+1 < argc) line_event_path = argv[++i];
        else if (!strcmp(argv[i], "--devices")) return gui_devices();
        else if (!strcmp(argv[i], "--gui-control"))                     g_gui_control = 1;
        else if (!strcmp(argv[i], "--gui-replay") && i+1 < argc) g_replay_path = argv[++i];
        else if (!strcmp(argv[i], "--control-link") && i+1 < argc) g_control_link = argv[++i];
        else if (!strcmp(argv[i], "--data-link") && i+1 < argc)    g_data_link = argv[++i];
        else if (!strcmp(argv[i], "--answer"))                          g_answer_mode = 1;
        else if (!strcmp(argv[i], "--answer-now")) { g_answer_mode = 1; answer_now = 1; }
        else if (!strcmp(argv[i], "--answer-rings") && i+1 < argc) {
            char *end; long count = strtol(argv[++i],&end,10);
            if (*end || count < 1 || count > 9) { fprintf(stderr,"answer rings must be 1..9\n"); return 2; }
            answer_rings = (unsigned)count;
        }
        else if (!strcmp(argv[i], "--ring-wait") && i + 1 < argc)   ring_wait = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dial") && i + 1 < argc)        g_dial = argv[++i];
        else if (!strcmp(argv[i], "--pty-link") && i + 1 < argc)   g_pty_link = argv[++i];
        else if (!strcmp(argv[i], "--rx-replay") && i + 1 < argc)   g_replay_path = argv[++i];
        else if (!strcmp(argv[i], "--hook") && i + 1 < argc)        hook_arg = argv[++i];
        else if (!strcmp(argv[i], "--loopback-peer-fd") && i+1 < argc) {
            char *end; const char *value = argv[++i]; long fd = strtol(value,&end,10);
            if (!*value || *end || fd < 3 || fd > INT_MAX) { fprintf(stderr,"Invalid loopback peer descriptor\n"); return 2; }
            loopback_peer_fd = (int)fd;
        }
        else if (!strcmp(argv[i], "--loopback-tone-level") && i+1 < argc) loopback_tone_level = atof(argv[++i]);
        else if (!strcmp(argv[i], "--loopback-rx-gain") && i+1 < argc) loopback_rx_gain = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--loopback-pattern") && i + 1 < argc) loopback_pattern = argv[++i];
        else if (!strcmp(argv[i], "--rate") && i + 1 < argc)        g_rate = atof(argv[++i]);
        else if (!strcmp(argv[i], "--hold") && i + 1 < argc)        hold = atof(argv[++i]);
        else if (!strcmp(argv[i], "--selftest"))                    return selftest();
        else { usage(argv[0]); return 2; }
    }
    if (!line_event_path) line_event_path = getenv("APPLE_MODEM_LINE_EVENTS");
    if (line_event_path && *line_event_path) {
        line_event_file = fopen(line_event_path,"a");
        if (!line_event_file) { perror(line_event_path); return 1; }
        atexit(line_events_close);
    }
    if (cid_replay_path) return cid_replay_file(cid_replay_path);
    if (loopback_peer_fd >= 0 && !loopback_mode) { fprintf(stderr,"Peer descriptor requires --loopback-test\n"); return 2; }
    if (loopback_mode) {
        signal(SIGINT,on_signal); signal(SIGTERM,on_signal);
        if (strcmp(loopback_mode,"digital") && strcmp(loopback_mode,"startup") && strcmp(loopback_mode,"isolation") && strcmp(loopback_mode,"analog")) { fprintf(stderr,"loopback must be digital, startup, isolation or analog\n"); return 2; }
        if (strcmp(loopback_pattern,"tone") && strcmp(loopback_pattern,"prbs") && strcmp(loopback_pattern,"ulaw") && strcmp(loopback_pattern,"alaw")) { fprintf(stderr,"loopback pattern must be tone, prbs, ulaw or alaw\n"); return 2; }
        if (strcmp(loopback_pattern,"tone") && strcmp(loopback_pattern,"prbs") && g_rate != 8000) { fprintf(stderr,"G.711 symbol testing requires --rate 8000\n"); return 2; }
        if (g_gui_control || g_dial || g_answer_mode || g_replay_path || hook_arg) { fprintf(stderr,"loopback excludes calls and replay\n"); return 2; }
        if (loopback_peer_fd >= 0) {
            int type; socklen_t size = sizeof type;
            if (g_rate != 8000 || loopback_peer_fd < 3 || getsockopt(loopback_peer_fd,SOL_SOCKET,SO_TYPE,&type,&size) < 0 || type != SOCK_STREAM) return 2;
#ifdef SO_NOSIGPIPE
            int one = 1; setsockopt(loopback_peer_fd,SOL_SOCKET,SO_NOSIGPIPE,&one,sizeof one);
#endif
        }
        if (!isfinite(loopback_tone_level) || loopback_tone_level <= 0 || loopback_tone_level > 1 || loopback_rx_gain < -1 || loopback_rx_gain > 12 || (loopback_rx_gain >= 0 && loopback_rx_gain%3)) return 2;
        if (loopback_peer_fd >= 0 && (loopback_rx_gain >= 0 || !strcmp(loopback_pattern,"tone"))) return 2;
        if (usb_open_line() < 0) return 1;
        int result = hardware_loopback(loopback_mode);
        @autoreleasepool {
            NSData *json = [NSJSONSerialization dataWithJSONObject:loopback_last ?: @{} options:0 error:NULL];
            fwrite(json.bytes,1,json.length,stdout); putchar('\n');
        }
        if (loopback_peer_fd >= 0) close(loopback_peer_fd);
        libusb_close(usb); libusb_exit(NULL); return result;
    }
    if (getenv("APPLE_MODEM_TX_AMP"))  tx_amp = atof(getenv("APPLE_MODEM_TX_AMP"));
    if (getenv("APPLE_MODEM_TX_GAIN")) g_tx_gain = atof(getenv("APPLE_MODEM_TX_GAIN"));
    if (getenv("APPLE_MODEM_DTMF_MS")) {
        const char *m = getenv("APPLE_MODEM_DTMF_MS"), *sl = strchr(m, '/');
        on_ms = atof(m);
        if (sl) off_ms = atof(sl + 1);
    }

    if ((g_control_link == NULL) != (g_data_link == NULL) ||
        (g_pty_link && g_control_link) ||
        (g_gui_control && (!g_control_link || g_dial || g_answer_mode || hook_arg))) {
        fprintf(stderr,"GUI control requires split PTYs and excludes immediate call/replay/hook options\n");
        return 2;
    }
    if (g_gui_control) {
        signal(SIGINT,on_signal); signal(SIGTERM,on_signal);
        setenv("ME_V90_ROLE","analogue",0);
        me_set_verbose(1); me_init(); me_set_law(ME_LAW_ULAW);
        di_set_callbacks(gui_dial,gui_answer,gui_hangup,NULL);
        di_set_dial_plan_callback(gui_dial_plan);
        if (di_open_split(g_control_link,g_data_link) < 0) { me_destroy(); return 1; }
        g_di_open = 1;
        di_set_auto_answer(0);
        const char *profile = getenv("APPLE_GUI_PROFILE");
        if (profile) di_load_profile(profile);
        gui_control_fd = STDIN_FILENO;
        fcntl(gui_control_fd,F_SETFL,fcntl(gui_control_fd,F_GETFL)|O_NONBLOCK);
        gui_monitor_start();
        if (g_replay_path) {
            rs_setup(); me_dial("replay");
            rc = replay(g_replay_path);
            close_dte(); me_destroy(); return rc;
        }
        fprintf(stderr,"[APPLE] GUI ready; line remains on-hook until ATD or ATA\n");
        while (!g_stop) {
            gui_commands(); gui_ring_poll();
            if (usb) line_poll(0);
            gui_publish();
            pthread_mutex_lock(&gui_request_lock);
            int request = gui_request;
            pthread_mutex_unlock(&gui_request_lock);
            if (request) {
                if (request == 3) {
                    fprintf(stderr,"[APPLE] dial string exceeds GUI limit; no line seized\n");
                    close_dte(); me_destroy(); return 1;
                }
                if (request == 1) g_dial = gui_number;
                else g_answer_mode = 1;
                break;
            }
            CFRunLoopRunInMode(kCFRunLoopDefaultMode,0.01,false);
        }
        gui_watch = 0;
        if (g_stop) {
            if (usb && gui_monitor_only) gui_onhook_stop(); else gui_audio_stop();
            gui_monitor_only = 0;
            if (usb) { libusb_close(usb); libusb_exit(NULL); }
            close_dte(); me_destroy(); return line_guard_tripped ? 1 : 0;
        }
        gui_status = g_answer_mode ? "Waiting for ring · line on-hook" : "Opening USB line";
        gui_publish();
    }

    if (hook_arg) {
        if (usb_open_line() < 0) return 1;
        rc = line_hook(!strcmp(hook_arg, "on")) < 0 ? 1 : 0;
        libusb_close(usb);
        libusb_exit(NULL);
        return rc;
    }

    if (g_replay_path) {
        setenv("ME_V90_ROLE", "analogue", 0);
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

    if (!g_dial && !g_answer_mode) { usage(argv[0]); return 2; }
    if (g_dial && g_answer_mode) {
        fprintf(stderr, "--dial and --answer are exclusive\n");
        return 2;
    }
    /* This binary IS the analogue modem -- it is wired to a 2-wire line through
     * a DAA, and V.90 puts the analogue modem on the calling side, the side that
     * dials.  Without this the engine takes the DIGITAL-role branch and both
     * ends of the call believe they are the digital modem.  setenv(..., 0) so an
     * explicit ME_V90_ROLE still wins -- which is what a replay of a recording
     * made from the digital side needs. */
    setenv("ME_V90_ROLE", "analogue", 0);
    rs_setup();
    if (g_rate == 8000.0 && !g_pcm8k)
        fprintf(stderr, "[APPLE] 8000 Hz: no receive resampler, and NO T/2 path "
                        "-- the V.90 analogue role cannot run\n");

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    if (usb_open_line() < 0) return 1;
    if (g_dial) {
        if (apple_dial_build(&physical_plan,g_dial,(unsigned)on_ms,(unsigned)off_ms) < 0) {
            fprintf(stderr,"Invalid physical dial plan; line not seized\n"); return 2;
        }
        if (!physical_plan.pulses && (physical_plan.count+1 > TX_MAX_SEG ||
            build_dial_script(g_dial,on_ms,off_ms) < 0)) return 2;
        atomic_store(&dial_control_active,physical_plan.pulses != 0);
    }

    if (!g_gui_control) {
        me_set_verbose(1); me_init(); me_set_law(ME_LAW_ULAW);
        di_set_callbacks(gui_dial,gui_answer,gui_hangup,NULL);
        di_set_dial_plan_callback(gui_dial_plan);
        if ((g_control_link ? di_open_split(g_control_link,g_data_link) :
             g_pty_link ? di_open(g_pty_link) : 0) < 0) {
            fprintf(stderr,"failed to open PTY\n"); me_destroy(); return 1;
        }
        g_di_open = g_control_link != NULL || g_pty_link != NULL;
        gui_monitor_start();
    }
    /* V.250 6.3.5: manual answering need not wait for a ring indication.
     * A powered direct pair has no FXS device to generate that indication. */
    if (g_answer_mode && !answer_now && !(g_gui_control && gui_recent_ring())) {
        if (gui_onhook_start() < 0 || wait_for_ring(ring_wait) < 0) {
            gui_onhook_stop(); close_dte(); me_destroy(); libusb_close(usb); libusb_exit(NULL); return 1;
        }
        cid_deliver();
    }
    if (g_answer_mode) {
        me_answer();
        /* There is no answer tone to wait for -- WE are the modem that sends
         * one -- so the engine starts with the first block of audio. */
        g_answered = 1;
    } else {
        me_dial(g_dial);
    }

    if (g_stop) { gui_audio_stop(); close_dte(); me_destroy(); libusb_close(usb); libusb_exit(NULL); return 0; }
    if (line_hook(1) < 0) {                  /* off-hook; refuses with no pair */
        line_hook(0); gui_audio_stop();
        close_dte();
        me_destroy();
        libusb_close(usb);
        libusb_exit(NULL);
        return 1;
    }
    if (g_stop) { line_hook(0); gui_audio_stop(); close_dte(); me_destroy(); libusb_close(usb); libusb_exit(NULL); return 0; }
    if (line_poll(1) < 0) { gui_audio_stop(); close_dte(); me_destroy(); libusb_close(usb); libusb_exit(NULL); return 1; }
    if (gui_monitor_only) line_event("monitor_stopped",@{@"reason":@"Call active"});
    gui_monitor_only = 0;
    if (!gui_audio_active && audio_start() < 0) {
        line_hook(0);
        close_dte();
        me_destroy();
        libusb_close(usb);
        libusb_exit(NULL);
        return 1;
    }
    gui_audio_active = 1;
    if (physical_plan.pulses) {
        if (physical_dial_run() < 0) {
            line_hook(0); gui_audio_stop(); close_dte(); me_destroy();
            libusb_close(usb); libusb_exit(NULL); return 1;
        }
        atomic_store(&dial_control_active,0);
        tx_dtmf_done = 1;
    }
    if (g_answer_mode)
        fprintf(stderr, "[APPLE] answered; engine runs from the first block\n");
    else if (physical_plan.pulses)
        fprintf(stderr,"[APPLE] pulse dial sequence completed; waiting for answer\n");
    else
        fprintf(stderr, "[APPLE] dialling %s, %d DTMF segments at %.0f/%.0f ms\n",
                g_dial, tx_nseg, on_ms, off_ms);

    gui_status = "Line active · analogue engine";
    /* The run loop is also where the DTE's bytes enter the engine: nothing in
       the audio callbacks does it, and without me_pump_dte() the modem
       connects and transmits idle marks for the whole call (the HSF coupler
       did exactly that on every call it placed). */
    uint64_t call_start_ms = apple_now_ms();
    while (!g_stop && apple_now_ms()-call_start_ms < hold*1000) {
        if (line_poll(0) < 0) break;
        (void)me_pump_dte();
        gui_publish();
        if (g_gui_control && g_engine_running &&
            (me_get_state() == ME_HANGUP || me_get_state() == ME_IDLE)) break;
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.01, false);
    }

    AudioOutputUnitStop(au);
    AudioUnitUninitialize(au);
    if (g_engine_running) me_on_sip_disconnected();
    close_dte();
    me_destroy();
    taps_close();
    fprintf(stderr, "[APPLE] render errors %d, engine %s, dial sequence %s\n",
            render_errors, g_engine_running ? "ran" : "never started",
            tx_dtmf_done ? "completed" : "CUT SHORT");
    callback_report();
    fprintf(stderr, "[APPLE] transmit ring underruns while the engine ran: %lu runs, "
                    "%lu samples", (unsigned long)tx_underrun_runs,
            (unsigned long)tx_underrun_samples);
    for (unsigned long i = 0; i < tx_underrun_runs && i < 8; i++)
        fprintf(stderr, "%s%.3f s", i ? ", " : " (first at ", tx_underrun_at[i] / g_rate);
    fprintf(stderr, "%s\n", tx_underrun_runs ? ")" : "");
    fprintf(stderr, "[APPLE] transmit ring overruns: %lu samples\n", tx_overrun_samples);
    fprintf(stderr, "[APPLE] device frames: in %lu, out %lu (out-in %+ld); ring fill minimum per 0.5 s:",
            dev_in_frames, dev_out_frames, (long)dev_out_frames - (long)dev_in_frames);
    for (int b = 0; b < FILL_BINS; b++) if (fill_min[b] >= 0) fprintf(stderr, " %d", fill_min[b]);
    fprintf(stderr, "\n");
    line_hook(0);                            /* always release the line */
    libusb_close(usb);
    libusb_exit(NULL);
    return line_guard_tripped ? 1 : g_engine_running ? 0 : 1;
}
