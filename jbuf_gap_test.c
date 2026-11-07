/*
 * jbuf_gap_test.c -- the RX jitter buffer must keep sample accounting exact
 * across lost RTP packets.
 *
 * The G.711 passthrough feeds the jitter buffer's output to the modem
 * receivers as their DS0, one 20 ms frame per tick, and plays a frame of fill
 * for every tick the buffer has no audio for (sip_modem.c).  A lost packet
 * must therefore come out as exactly one frame of fill IN ITS OWN SLOT.
 * Anything else moves every later sample by 160 -- 64 symbols at 3200 baud --
 * and a modem receiver cannot absorb that (the 9.6/11.6 "abrupt
 * discontinuity" path, V.90 9.6 / V.34 11.6).
 *
 * Stock pjmedia reset the buffer origin when a packet arrived into an empty
 * buffer, which drops the lost slots: with the rig's 40 ms buffer (ME_JB_MS=40)
 * that was the common case, and each isolated loss deleted a frame until the
 * next underrun inserted one back.  The fix is in
 * pjproject/pjmedia/src/pjmedia/jbuf.c (jb_framelist_put_at).
 *
 * Simulated: 20 ms arrivals with up to 12 ms of jitter, the consumer ticking
 * at every phase, buffer configured as sip_modem.c configures it, and several
 * loss patterns.  Every frame the consumer plays must carry the sequence
 * number of its own slot.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pjlib.h>
#include <pjmedia/jbuf.h>

#define FRAME_MS    20
#define TICK_PER_MS 10                  /* timeline in 100 us units */
#define N_FRAMES    600

static int steps_for(pj_pool_t *pool, unsigned jb_ms, double ppm,
                     const int *lost, int n_lost, int jitter_ms, int phase,
                     int seed)
{
    static long arrival[N_FRAMES];
    static int played[2*N_FRAMES];
    pj_str_t name = pj_str("jbtest");
    pjmedia_jbuf *jb;
    unsigned prefetch = jb_ms/FRAME_MS;
    const long period = FRAME_MS*TICK_PER_MS;
    int n_played = 0;
    int next_arrival = 0;
    int steps = 0;
    int prev = -99999;

    /* As sip_modem.c sets it up through pjsua: jb_init = jb_min_pre =
       jb_max_pre = ME_JB_MS, jb_max = ME_JB_MS + 800, no discard. */
    if (pjmedia_jbuf_create(pool, &name, 160, FRAME_MS,
                            (jb_ms + 800)/FRAME_MS, &jb) != PJ_SUCCESS)
        return -1;
    pjmedia_jbuf_set_adaptive(jb, prefetch, prefetch, prefetch);
    pjmedia_jbuf_set_discard(jb, PJMEDIA_JB_DISCARD_NONE);

    srand((unsigned) (seed*7919 + phase*31 + jitter_ms));
    for (int k = 0;  k < N_FRAMES;  k++)
        arrival[k] = (long) (period*k*(1.0 + ppm*1e-6))
                   + (jitter_ms ? rand() % (jitter_ms*TICK_PER_MS + 1) : 0);

    for (long t = 1;  t < arrival[N_FRAMES - 1];  t++) {
        /* Jitter is under one period, so arrivals are nearly in order. */
        for (int k = next_arrival > 2 ? next_arrival - 2 : 0;
             k < N_FRAMES && k < next_arrival + 3;  k++) {
            if (arrival[k] != t)
                continue;
            int is_lost = 0;
            for (int i = 0;  i < n_lost;  i++)
                if (lost[i] == k)
                    is_lost = 1;
            if (!is_lost) {
                unsigned char frame[160];
                memset(frame, 0, sizeof(frame));
                memcpy(frame, &k, sizeof(k));
                pjmedia_jbuf_put_frame3(jb, frame, sizeof(frame), 0,
                                        1000 + k, 0, NULL);
            }
            if (k >= next_arrival)
                next_arrival = k + 1;
        }
        if (t % period == phase*TICK_PER_MS) {
            unsigned char frame[160];
            pj_size_t size = sizeof(frame);
            char type;
            pj_uint32_t bit_info;
            pj_uint32_t ts;
            int seq = -1;

            pjmedia_jbuf_get_frame3(jb, frame, &size, &type, &bit_info, &ts,
                                    NULL);
            if (type == PJMEDIA_JB_NORMAL_FRAME)
                memcpy(&seq, frame, sizeof(seq));
            played[n_played++] = seq;
        }
    }
    pjmedia_jbuf_destroy(jb);

    /* The offset between a frame's playout slot and its sequence number is
       fixed once the first frame plays; every change is a timing step. */
    for (int i = 0;  i < n_played;  i++) {
        if (played[i] < 0)
            continue;
        if (prev != -99999 && i - played[i] != prev)
            steps++;
        prev = i - played[i];
    }
    return steps;
}

int main(void)
{
    static const struct {
        const char *name;
        int lost[4];
        int n;
    } cases[] = {
        {"no loss",               {0},                  0},
        {"one packet",            {300},                1},
        {"two in a row",          {300, 301},           2},
        {"three in a row",        {300, 301, 302},      3},
        {"two, 3 apart",          {300, 303},           2},
        {"two, 37 apart",         {300, 337},           2},
        {"four spread",           {150, 300, 420, 540}, 4},
    };
    static const unsigned jb_ms[] = {40, 200};
    pj_caching_pool cp;
    pj_pool_t *pool;
    int failures = 0;

    pj_log_set_level(0);
    if (pj_init() != PJ_SUCCESS)
        return 2;
    pj_caching_pool_init(&cp, NULL, 0);
    pool = pj_pool_create(&cp.factory, "jbuf_gap_test", 4000, 4000, NULL);

    for (size_t j = 0;  j < sizeof(jb_ms)/sizeof(jb_ms[0]);  j++) {
        for (size_t c = 0;  c < sizeof(cases)/sizeof(cases[0]);  c++) {
            int runs = 0;
            int stepped = 0;

            for (int jitter = 0;  jitter <= 12;  jitter += 4) {
                for (int phase = 0;  phase < FRAME_MS;  phase++) {
                    for (int seed = 0;  seed < 3;  seed++) {
                        int steps = steps_for(pool, jb_ms[j], 0.0,
                                              cases[c].lost, cases[c].n,
                                              jitter, phase, seed);
                        runs++;
                        if (steps != 0) {
                            stepped++;
                            if (stepped <= 3)
                                printf("  %u ms, %s: %d step(s) at jitter "
                                       "%d ms, phase %d, seed %d\n",
                                       jb_ms[j], cases[c].name, steps,
                                       jitter, phase, seed);
                        }
                    }
                }
            }
            printf("%s: %u ms buffer, %s: %d of %d runs stepped\n",
                   stepped ? "FAIL" : "ok", jb_ms[j], cases[c].name,
                   stepped, runs);
            if (stepped)
                failures++;
        }
    }

    pj_pool_release(pool);
    pj_caching_pool_destroy(&cp);
    pj_shutdown();
    if (failures) {
        printf("jbuf_gap_test: %d case(s) FAILED\n", failures);
        return 1;
    }
    printf("jbuf_gap_test: all cases passed\n");
    return 0;
}
