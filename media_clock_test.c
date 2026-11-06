/*
 * media_clock_test.c -- the loop-timed media clock locks to the far end.
 *
 * Simulates a far end whose 8 kHz clock runs at a given offset from ours,
 * sending 20 ms RTP packets that the network delays by 0-15 ms (with an
 * occasional 60 ms spike), and our tick loop driven by media_clock's period.
 * Checks that the tick rate converges on the far end's rate and that the
 * receive fill (what a gateway's playout buffer would see drift) stays put.
 */
#include "media_clock.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned rng = 12345u;
static double urand(void)
{
    rng = rng * 1103515245u + 12345u;
    return (double) ((rng >> 8) & 0xffffff) / 16777216.0;
}

static int run(double far_ppm, uint32_t ts0, double seconds, double *out_ppm,
               double *out_fill_swing)
{
    media_clock_t mc;
    double far_period = 20e6 / (1.0 + far_ppm * 1e-6);
    double next_send = 0.0, next_tick = 5e6;
    double t_end = seconds * 1e9;
    uint32_t ts = ts0;
    /* fill = far samples sent - our samples consumed, at whole-tick instants */
    long long sent = 0, ticks = 0;
    double fill_min = 1e9, fill_max = -1e9;
    double win_t0 = -1.0;
    long long win_ticks0 = 0;
    /* packets in flight: arrival time + ts, small ring */
    double arr_t[64];
    uint32_t arr_ts[64];
    int head = 0, count = 0;

    media_clock_init(&mc, 8000, 160);
    while (next_tick < t_end) {
        /* far end sends */
        while (next_send <= next_tick) {
            double delay = 15e6 * urand();
            if (urand() < 0.002)
                delay += 60e6;
            if (count < 64) {
                int slot = (head + count) % 64;
                arr_t[slot] = next_send + delay;
                arr_ts[slot] = ts;
                count++;
            }
            ts += 160;
            sent++;
            next_send += far_period;
        }
        /* deliver arrivals before this tick, in arrival order */
        for (;;) {
            int best = -1, i;
            for (i = 0; i < count; i++) {
                int slot = (head + i) % 64;
                if (arr_t[slot] <= next_tick && (best < 0 || arr_t[slot] < arr_t[best]))
                    best = slot;
            }
            if (best < 0)
                break;
            media_clock_rx(&mc, arr_ts[best], (long long) arr_t[best]);
            arr_t[best] = 1e30;
            while (count && arr_t[head] > 1e29) {
                head = (head + 1) % 64;
                count--;
            }
        }
        media_clock_tick(&mc, (long long) next_tick);
        ticks++;
        if (next_tick > 0.6 * t_end) {
            if (win_t0 < 0) {
                win_t0 = next_tick;
                win_ticks0 = ticks;
            }
            double fill = (double) (sent - ticks) * 160.0;
            /* correct for where in its period the far end is */
            fill += 160.0 * (next_tick - (next_send - far_period)) / far_period;
            if (fill < fill_min) fill_min = fill;
            if (fill > fill_max) fill_max = fill;
        }
        next_tick += media_clock_period_ns(&mc);
    }
    /* Average tick rate over the window, as ppm against nominal 50 Hz. */
    *out_ppm = ((double) (ticks - win_ticks0 + 1) * 20e6 / (next_tick - win_t0) - 1.0) * 1e6;
    *out_fill_swing = fill_max - fill_min;
    return 0;
}

int main(void)
{
    const double cases[] = { 62.3, 0.0, -150.0, 300.0 };
    int failures = 0;
    unsigned i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        double ppm, swing;
        /* start near a 32-bit wrap for one case */
        run(cases[i], i == 1 ? 0xfff00000u : 1000u, 600.0, &ppm, &swing);
        /* Over the last 240 s the far end sends at far_ppm; we must match it
           to ~3 ppm (0.024 samples/s, 6 samples over 240 s) and the fill
           must not drift: an unlocked 62 ppm drifts 120 samples there. */
        int ok = fabs(ppm - cases[i]) < 3.0 && swing < 40.0;
        printf("far %+7.1f ppm: ticked at %+7.2f ppm, fill swing %.1f samples over the last 240 s  %s\n",
               cases[i], ppm, swing, ok ? "ok" : "FAIL");
        failures += !ok;
    }

    /* A reset forgets the far end. */
    {
        media_clock_t mc;
        media_clock_init(&mc, 8000, 160);
        mc.ppm = 50.0;
        mc.integ = 50.0;
        media_clock_reset(&mc);
        if (media_clock_ppm(&mc) != 0.0 || fabs(media_clock_period_ns(&mc) - 20e6) > 1e-6) {
            printf("reset did not restore the nominal period  FAIL\n");
            failures++;
        }
    }
    printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
