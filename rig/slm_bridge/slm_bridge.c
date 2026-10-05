/*
 * slm_bridge.c -- put slmodemd on a line to audio_sock_modem, with no SIP.
 *
 * slmodemd (the SmartLink soft modem, as packaged in AonCyberLabs' D-Modem)
 * runs its "-e" program for every call, passing the dial string (empty after
 * ATA) and the number of a socket carrying the DSP's audio: signed 16-bit
 * native-endian linear at 9600 samples/s.  slmodemd primes it with 192 zero
 * samples and then writes exactly as many samples as it reads, so the -e
 * program owns the clock.  D-Modem's own -e program puts that on a SIP call;
 * this one puts it on audio_sock_modem's G.711 socket instead:
 *
 *   slmodemd -e .../slm_bridge
 *   SLM_BRIDGE_SOCKET=<path>   audio_sock_modem's --listen socket (required)
 *   SLM_BRIDGE_ALAW=1          A-law on the G.711 side (default u-law)
 *   SLM_BRIDGE_REALTIME=0      run as fast as the two modems go (default:
 *                              paced at 20 ms per frame, as a line would be)
 *   SLM_BRIDGE_TAP_DIR=<dir>   write slm-tx.s16 / slm-rx.s16 (9600 Hz) and
 *                              line-tx.g711 / line-rx.g711 (8000 Hz) there
 *   SLM_BRIDGE_RX_GAIN_DB=<dB> gain on the line audio into slmodemd (default 0;
 *                              a loss on a real line)
 *   SLM_BRIDGE_ECHO_DB=<dB>    return slmodemd's own transmit into its receive
 *                              this far down, one frame late: the near end
 *                              hybrid a 2-wire line has and this bridge
 *                              otherwise does not (default: none)
 *
 * Each 20 ms: 192 samples from slmodemd -> 160 G.711 codewords to the modem,
 * 160 back -> 192 samples to slmodemd.  The 9600/8000 conversion is the one
 * thing here that touches the signal, so it is a proper windowed-sinc
 * polyphase in both directions rather than the linear interpolation the rig's
 * d-modem.c uses upstream: linear interpolation of a signal reaching 3.4 kHz
 * at 9600 Hz leaves images that the far modem receives as noise.  G.711 is
 * coded with SpanDSP's own tables (the modem engine's), so the line carries
 * the codewords a G.711 network would.
 */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include <spandsp.h>

#define SLM_FRAME   192     /* 20 ms at 9600 */
#define LINE_FRAME  160     /* 20 ms at 8000 */
#define HALF        32      /* taps either side of the output instant */

/* A streaming rational resampler, out_rate/in_rate = l/m. */
typedef struct {
    int l;
    int m;
    float ker[6][2*HALF + 1];   /* one kernel per output phase (l <= 6) */
    float hist[2*HALF + 1 + SLM_FRAME];
    int64_t in_count;           /* input samples consumed */
    int64_t out_count;          /* output samples produced */
} resampler_t;

static void rs_init(resampler_t *r, int l, int m, double cutoff_hz, double in_rate)
{
    double wc = cutoff_hz/(in_rate/2.0);
    int p;
    int t;

    memset(r, 0, sizeof(*r));
    r->l = l;
    r->m = m;
    for (p = 0;  p < l;  p++) {
        double frac = (double) ((p*m)%l)/(double) l;
        double sum = 0.0;

        for (t = 0;  t <= 2*HALF;  t++) {
            double x = (double) (t - HALF) - frac;
            double y = wc*x;
            double s = (fabs(y) < 1e-9) ? wc : wc*sin(M_PI*y)/(M_PI*y);
            /* Blackman window over the span */
            double u = (x + HALF + 1.0)/(2.0*HALF + 2.0);
            double w = 0.42 - 0.5*cos(2.0*M_PI*u) + 0.08*cos(4.0*M_PI*u);

            r->ker[p][t] = (float) (s*w);
            sum += s*w;
        }
        for (t = 0;  t <= 2*HALF;  t++)
            r->ker[p][t] = (float) (r->ker[p][t]/sum);
    }
}

/* Feed n input samples, produce exactly n*l/m outputs (n is a multiple of m).
   Output k sits at input position k*m/l - HALF, so every tap it needs has
   arrived: a fixed delay of HALF input samples, and exact frame counts. */
static int rs_run(resampler_t *r, const float *in, int n, float *out)
{
    int hlen = 2*HALF;
    float work[2*HALF + SLM_FRAME];
    int produced = 0;
    int i;

    memcpy(work, r->hist, sizeof(float)*(size_t) hlen);
    memcpy(work + hlen, in, sizeof(float)*(size_t) n);
    for (;;) {
        int64_t num = r->out_count*r->m;
        /* Integer part of the output instant, HALF input samples late. */
        int64_t base = num/r->l - HALF;
        int phase = (int) (r->out_count%r->l);
        /* Input index of tap 0, relative to work[0] (which is input sample
           in_count - hlen, zeros before the first frame). */
        int64_t first = base - HALF - (r->in_count - hlen);
        float acc = 0.0f;

        if (first + 2*HALF >= hlen + n)
            break;
        for (i = 0;  i <= 2*HALF;  i++)
            acc += work[first + i]*r->ker[phase][i];
        out[produced++] = acc;
        r->out_count++;
    }
    memcpy(r->hist, work + n, sizeof(float)*(size_t) hlen);
    r->in_count += n;
    return produced;
}

static int read_full(int fd, void *buf, int n)
{
    int got = 0;

    while (got < n) {
        ssize_t r = read(fd, (uint8_t *) buf + got, (size_t) (n - got));

        if (r < 0 && errno == EINTR)
            continue;
        if (r <= 0)
            return -1;
        got += (int) r;
    }
    return got;
}

static int write_full(int fd, const void *buf, int n)
{
    int put = 0;

    while (put < n) {
        ssize_t r = write(fd, (const uint8_t *) buf + put, (size_t) (n - put));

        if (r < 0 && errno == EINTR)
            continue;
        if (r <= 0)
            return -1;
        put += (int) r;
    }
    return put;
}

static FILE *open_tap(const char *dir, const char *name)
{
    char path[512];

    if (dir == NULL || *dir == '\0')
        return NULL;
    snprintf(path, sizeof(path), "%s/%s", dir, name);
    return fopen(path, "wb");
}

static int16_t clamp16(float v)
{
    if (v > 32767.0f)
        return 32767;
    if (v < -32768.0f)
        return -32768;
    return (int16_t) lrintf(v);
}

int main(int argc, char *argv[])
{
    const char *path = getenv("SLM_BRIDGE_SOCKET");
    const char *rt = getenv("SLM_BRIDGE_REALTIME");
    const char *alaw_env = getenv("SLM_BRIDGE_ALAW");
    const char *tapdir = getenv("SLM_BRIDGE_TAP_DIR");
    bool alaw = (alaw_env != NULL && strcmp(alaw_env, "0") != 0);
    bool realtime = (rt == NULL || strcmp(rt, "0") != 0);
    const char *gain_env = getenv("SLM_BRIDGE_RX_GAIN_DB");
    float rx_gain = (gain_env != NULL) ? powf(10.0f, (float) atof(gain_env)/20.0f) : 1.0f;
    const char *echo_env = getenv("SLM_BRIDGE_ECHO_DB");
    float echo_gain = (echo_env != NULL) ? powf(10.0f, (float) atof(echo_env)/20.0f) : 0.0f;
    int16_t echo[SLM_FRAME];
    resampler_t down;
    resampler_t up;
    struct sockaddr_un addr;
    struct timespec next;
    FILE *tap_slm_tx;
    FILE *tap_slm_rx;
    FILE *tap_line_tx;
    FILE *tap_line_rx;
    long frames = 0;
    int slm;
    int line;

    if (argc < 3 || path == NULL || strlen(path) >= sizeof(addr.sun_path)) {
        fprintf(stderr, "slm_bridge: run by slmodemd -e, with SLM_BRIDGE_SOCKET set\n");
        return 2;
    }
    slm = atoi(argv[2]);
    fprintf(stderr, "slm_bridge: %s call (dial string '%s') -> %s, %s%s\n",
            argv[1][0] ? "outgoing" : "incoming", argv[1], path,
            alaw ? "A-law" : "u-law", realtime ? ", real time" : "");

    line = socket(AF_UNIX, SOCK_STREAM, 0);
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, path);
    if (line < 0 || connect(line, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        perror(path);
        return 1;
    }

    /* 9600 -> 8000: keep the line inside its own Nyquist with some room.
       8000 -> 9600: the input is already band limited by G.711's 8 kHz. */
    rs_init(&down, 5, 6, 3700.0, 9600.0);
    rs_init(&up, 6, 5, 3900.0, 8000.0);
    tap_slm_tx = open_tap(tapdir, "slm-tx.s16");
    tap_slm_rx = open_tap(tapdir, "slm-rx.s16");
    tap_line_tx = open_tap(tapdir, "line-tx.g711");
    tap_line_rx = open_tap(tapdir, "line-rx.g711");

    clock_gettime(CLOCK_MONOTONIC, &next);
    for (;;) {
        int16_t s16[SLM_FRAME];
        float f[SLM_FRAME];
        float g[SLM_FRAME];
        uint8_t cw[LINE_FRAME];
        int n;
        int i;

        /* slmodemd's transmit: its 192-sample prime first, then whatever it
           wrote in answer to the frame we gave it last. */
        if (read_full(slm, s16, (int) sizeof(s16)) < 0)
            break;
        if (frames == 0)
            memset(echo, 0, sizeof(echo));
        if (tap_slm_tx)
            fwrite(s16, sizeof(int16_t), SLM_FRAME, tap_slm_tx);
        for (i = 0;  i < SLM_FRAME;  i++)
            f[i] = (float) s16[i];
        n = rs_run(&down, f, SLM_FRAME, g);
        /* This frame's transmit becomes next frame's echo. */
        {
            int16_t tmp[SLM_FRAME];

            memcpy(tmp, s16, sizeof(tmp));
            for (i = 0;  i < SLM_FRAME;  i++)
                s16[i] = echo[i];
            memcpy(echo, tmp, sizeof(echo));
        }
        if (n != LINE_FRAME) {
            fprintf(stderr, "slm_bridge: resampler produced %d, not %d\n", n, LINE_FRAME);
            return 1;
        }
        for (i = 0;  i < LINE_FRAME;  i++)
            cw[i] = alaw ? linear_to_alaw(clamp16(g[i])) : linear_to_ulaw(clamp16(g[i]));
        if (tap_line_tx)
            fwrite(cw, 1, LINE_FRAME, tap_line_tx);
        if (write_full(line, cw, LINE_FRAME) < 0 || read_full(line, cw, LINE_FRAME) < 0)
            break;
        if (tap_line_rx)
            fwrite(cw, 1, LINE_FRAME, tap_line_rx);
        for (i = 0;  i < LINE_FRAME;  i++)
            f[i] = (float) (alaw ? alaw_to_linear(cw[i]) : ulaw_to_linear(cw[i]));
        /* rs_run's buffer is sized for a 9600 Hz frame, and 160 fits. */
        n = rs_run(&up, f, LINE_FRAME, g);
        if (n != SLM_FRAME) {
            fprintf(stderr, "slm_bridge: resampler produced %d, not %d\n", n, SLM_FRAME);
            return 1;
        }
        for (i = 0;  i < SLM_FRAME;  i++)
            s16[i] = clamp16(g[i]*rx_gain + echo_gain*(float) s16[i]);
        if (tap_slm_rx)
            fwrite(s16, sizeof(int16_t), SLM_FRAME, tap_slm_rx);
        if (write_full(slm, s16, (int) sizeof(s16)) < 0)
            break;
        frames++;
        if (realtime) {
            next.tv_nsec += 20000000L;
            if (next.tv_nsec >= 1000000000L) {
                next.tv_nsec -= 1000000000L;
                next.tv_sec++;
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        }
    }
    fprintf(stderr, "slm_bridge: call ended after %.2f s\n", frames*0.02);
    if (tap_slm_tx) fclose(tap_slm_tx);
    if (tap_slm_rx) fclose(tap_slm_rx);
    if (tap_line_tx) fclose(tap_line_tx);
    if (tap_line_rx) fclose(tap_line_rx);
    close(line);
    close(slm);
    return 0;
}
