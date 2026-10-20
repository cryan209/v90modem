/*
 * v42_stress_test.c — two data stacks (V.42 LAPM, optionally V.42bis or V.44)
 * across an impaired bit channel, both directions saturated, every received
 * octet graded against the sender's stream.
 *
 * The other V.42 tests each exercise one recovery path with a clean or
 * lightly flipped line.  A live line is worse: independent errors, error
 * bursts from a datapump losing its eye, outages that deliver noise rather
 * than marks, and long round trips.  The failures that matter here are
 *   - corruption: an octet reaches the DTE that the far DTE did not send
 *     (V.42 promises this cannot happen short of an FCS miss);
 *   - a stall: the link stays up but stops carrying data;
 *   - a teardown the line did not force.
 *
 * Usage: v42_stress_test                   run the regression matrix
 *        v42_stress_test <scenario> <seed> run one row, verbosely
 */

#include "data_stack.h"
#include "v44.h"

#include <spandsp.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const char *name;
    int rate;               /* line bit rate, both directions */
    int delay_ms;           /* one way */
    int compression;        /* 0 none, 1 V.42bis, 2 V.44 */
    double ber;             /* independent bit errors */
    int burst_every_ms;     /* mean spacing of error bursts, 0 none */
    int burst_bits;         /* random bits per burst */
    int outage_every_ms;    /* period of outages, 0 none */
    int outage_ms;
    int outage_fill;        /* 0 zeros, 1 ones, 2 random */
    int seconds;
    int min_kbps;           /* expected floor for each direction, DTE octets */
    double max_stall_s;     /* longest delivery gap allowed; 0 = 5 s */
    int rx_queue;           /* DTE receive queue, octets; 0 = unbounded */
    int drain_bps;          /* DTE drain rate from that queue, octets/s */
} scenario_t;

static const scenario_t scenarios[] = {
    {"clean-9600", 9600, 10, 0, 0, 0, 0, 0, 0, 0, 30, 6, 0, 0, 0},
    {"clean-v42bis-28800", 28800, 40, 1, 0, 0, 0, 0, 0, 0, 30, 20, 0, 0, 0},
    {"clean-v44-33600", 33600, 40, 2, 0, 0, 0, 0, 0, 0, 30, 20, 0, 0, 0},
    {"ber1e-5-14400", 14400, 20, 1, 1e-5, 0, 0, 0, 0, 0, 60, 7, 0, 0, 0},
    {"ber1e-4-14400", 14400, 20, 1, 1e-4, 0, 0, 0, 0, 0, 60, 2, 0, 0, 0},
    {"ber1e-4-v44-14400", 14400, 20, 2, 1e-4, 0, 0, 0, 0, 0, 60, 2, 0, 0, 0},
    {"bursts-31200", 31200, 100, 1, 0, 500, 64, 0, 0, 0, 60, 10, 0, 0, 0},
    {"bursts-v44-31200", 31200, 100, 2, 0, 500, 64, 0, 0, 0, 60, 10, 0, 0, 0},
    {"noise-outage-26400", 26400, 60, 1, 0, 0, 0, 5000, 400, 2, 120, 8, 0, 0, 0},
    {"zero-outage-26400", 26400, 60, 1, 0, 0, 0, 5000, 400, 0, 120, 8, 0, 0, 0},
    {"long-outage-21600", 21600, 60, 2, 0, 0, 0, 20000, 4000, 2, 120, 4, 0, 0, 0},
    {"noise-outage-plain", 26400, 60, 0, 0, 0, 0, 5000, 400, 2, 120, 8, 0, 0, 0},
    {"long-rtt-v42bis", 48000, 300, 1, 1e-6, 0, 0, 0, 0, 0, 60, 10, 0, 0, 0},
    /* A lost retransmission waits on T401 (2.5 s at 2400), and here one
       frame in five is hit, so a couple of those back to back is fair. */
    {"slow-2400-bursts", 2400, 20, 1, 0, 2000, 32, 0, 0, 0, 120, 1, 12.0, 0, 0},
    /* A DTE that reads slower than the link delivers (a terminal, a full
       PPP stack): V.42bis text expands ~4x, so the far end is throttled to
       the drain by RNR, and nothing already acknowledged may be lost. */
    {"slow-dte-v42bis", 31200, 60, 1, 0, 0, 0, 0, 0, 0, 60, 7, 0, 16384, 1000},
    {"slow-dte-v44", 31200, 60, 2, 0, 0, 0, 0, 0, 0, 60, 7, 0, 16384, 1000},
    {"slow-dte-plain-ber", 14400, 20, 0, 1e-5, 0, 0, 0, 0, 0, 60, 7, 0, 8192, 1000},
    {"stalled-dte-v42bis", 31200, 60, 1, 0, 0, 0, 0, 0, 0, 60, 0, 30.0, 16384, 1},
};

/* xorshift64*: deterministic per seed, independent per user. */
static uint64_t rng_next(uint64_t *s)
{
    uint64_t x = *s;

    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *s = x;
    return x * 0x2545F4914F6CDD1DULL;
}

static double rng_unit(uint64_t *s)
{
    return (double)(rng_next(s) >> 11) * (1.0/9007199254740992.0);
}

/* The DTE stream: numbered text lines (compressible) with occasional binary
   runs (not), generated octet by octet so the receiver can regenerate it. */
typedef struct {
    uint64_t rng;
    char line[96];
    int line_len;
    int line_pos;
    int binary_left;
    uint64_t line_no;
} source_t;

static void source_init(source_t *src, uint64_t seed)
{
    memset(src, 0, sizeof(*src));
    src->rng = seed | 1;
}

static uint8_t source_next(source_t *src)
{
    if (src->binary_left > 0)
    {
        src->binary_left--;
        return (uint8_t)rng_next(&src->rng);
    }
    if (src->line_pos >= src->line_len)
    {
        if ((rng_next(&src->rng) % 50) == 0)
        {
            src->binary_left = 64 + (int)(rng_next(&src->rng) % 512);
            src->line_len = src->line_pos = 0;
            return (uint8_t)rng_next(&src->rng);
        }
        src->line_len = snprintf(src->line, sizeof(src->line),
                                 "line %08llu the quick brown fox jumps over the lazy dog\r\n",
                                 (unsigned long long)src->line_no++);
        src->line_pos = 0;
    }
    return (uint8_t)src->line[src->line_pos++];
}

typedef struct {
    source_t tx;            /* what this end's DTE sends */
    source_t expect;        /* regenerates what the far DTE sent */
    uint64_t sent;
    uint64_t received;
    uint64_t bad;
    uint64_t first_bad_at;
    uint64_t last_rx_tick;
    uint64_t max_gap;       /* longest spell with nothing delivered, ticks */
    uint64_t max_gap_end;
    int tx_budget;          /* DTE octets the pull may hand over now */
    uint8_t *q;             /* bounded DTE receive queue */
    int q_size, q_head, q_len;
    uint64_t overruns;      /* octets the queue had no room for */
    int drain_acc;
    int connects;
    int disconnects;
    int errors;
    bool ready_once;
} end_t;

static uint64_t now_tick;

static int end_pull(void *ctx)
{
    end_t *e = ctx;

    if (e->tx_budget <= 0)
        return -1;
    e->tx_budget--;
    e->sent++;
    return source_next(&e->tx);
}

static void end_grade(end_t *e, uint8_t byte)
{
    uint8_t want = source_next(&e->expect);

    if (byte != want)
    {
        if (e->bad == 0)
            e->first_bad_at = e->received;
        e->bad++;
    }
    if (e->received  &&  now_tick - e->last_rx_tick > e->max_gap)
    {
        e->max_gap = now_tick - e->last_rx_tick;
        e->max_gap_end = now_tick;
    }
    e->received++;
    e->last_rx_tick = now_tick;
}

static void end_push(void *ctx, uint8_t byte)
{
    end_t *e = ctx;

    if (!e->q)
    {
        end_grade(e, byte);
        return;
    }
    if (e->q_len >= e->q_size)
    {
        /* Lost: the grader will see the gap as corruption too. */
        e->overruns++;
        return;
    }
    e->q[(e->q_head + e->q_len++) % e->q_size] = byte;
}

static int end_room(void *ctx)
{
    end_t *e = ctx;

    return e->q_size - e->q_len;
}

static void end_drain(end_t *e, int n)
{
    while (n-- > 0  &&  e->q_len > 0)
    {
        end_grade(e, e->q[e->q_head]);
        e->q_head = (e->q_head + 1) % e->q_size;
        e->q_len--;
    }
}

static void end_event(void *ctx, ds_link_event_t event)
{
    end_t *e = ctx;

    switch (event)
    {
    case DS_LINK_CONNECTED:
        e->connects++;
        e->ready_once = true;
        break;
    case DS_LINK_DISCONNECTED:
        e->disconnects++;
        break;
    case DS_LINK_ERROR:
    case DS_LINK_UNSUPPORTED:
        e->errors++;
        break;
    default:
        break;
    }
}

/* One direction of the line: impairments, then a fixed delay. */
typedef struct {
    uint8_t *ring;
    int len;
    int pos;
    uint64_t rng;
    uint64_t next_burst;
    int burst_left;
    uint64_t flips;
} channel_t;

static void channel_init(channel_t *c, const scenario_t *sc, uint64_t seed)
{
    c->len = sc->rate*sc->delay_ms/1000;
    if (c->len < 1)
        c->len = 1;
    c->ring = malloc((size_t)c->len);
    memset(c->ring, 1, (size_t)c->len);
    c->pos = 0;
    c->rng = seed | 1;
    c->next_burst = 0;
    c->burst_left = 0;
    c->flips = 0;
}

static int channel_pass(channel_t *c, const scenario_t *sc, uint64_t tick, int bit)
{
    int out;

    if (sc->outage_every_ms)
    {
        uint64_t period = (uint64_t)sc->rate*sc->outage_every_ms/1000;
        uint64_t span = (uint64_t)sc->rate*sc->outage_ms/1000;

        /* The first outage after the link has had time to establish. */
        if (tick > period  &&  tick % period < span)
        {
            if (sc->outage_fill == 2)
                bit = (int)(rng_next(&c->rng) & 1);
            else
                bit = sc->outage_fill;
        }
    }
    if (sc->burst_every_ms)
    {
        if (c->next_burst == 0)
            c->next_burst = tick + 1 + rng_next(&c->rng) % (2ULL*sc->rate*sc->burst_every_ms/1000);
        if (tick == c->next_burst)
        {
            c->burst_left = sc->burst_bits;
            c->next_burst = 0;
        }
        if (c->burst_left > 0)
        {
            c->burst_left--;
            bit = (int)(rng_next(&c->rng) & 1);
        }
    }
    if (sc->ber > 0  &&  rng_unit(&c->rng) < sc->ber)
    {
        bit ^= 1;
        c->flips++;
    }
    out = c->ring[c->pos];
    c->ring[c->pos] = (uint8_t)bit;
    if (++c->pos >= c->len)
        c->pos = 0;
    return out;
}

static int init_end(data_stack_t *ds, end_t *e, const scenario_t *sc, bool caller)
{
    if (sc->compression == 2)
        return ds_init_v44(ds, caller, true, sc->rate, NULL,
                           end_pull, e, end_push, e, end_event, e);
    return ds_init_v42_ex(ds, caller, true, sc->rate, sc->compression ? 3 : 0, 1024, 32,
                          end_pull, e, end_push, e, end_event, e);
}

typedef struct {
    bool pass;
    double kbps[2];
    uint64_t bad[2];
    int disconnects;
    int errors;
    double stall_s;
    uint64_t overruns;
} result_t;

static result_t run(const scenario_t *sc, uint64_t seed, bool verbose)
{
    data_stack_t ds[2];
    end_t ep[2];
    channel_t ch[2];
    result_t r;
    uint64_t total = (uint64_t)sc->rate*sc->seconds;
    /* DTE paced above the line rate so the link is always saturated. */
    int dte_per_tick_q16 = (int)(65536.0*sc->rate/8.0*4.0/sc->rate);
    int acc[2] = {0, 0};

    memset(&r, 0, sizeof(r));
    memset(ep, 0, sizeof(ep));
    for (int i = 0; i < 2; i++)
    {
        source_init(&ep[i].tx, seed*2 + (uint64_t)i + 1);
        source_init(&ep[i].expect, seed*2 + (uint64_t)(1 - i) + 1);
        channel_init(&ch[i], sc, seed*7919 + (uint64_t)i*104729 + 3);
    }
    if (init_end(&ds[0], &ep[0], sc, true) != 0  ||  init_end(&ds[1], &ep[1], sc, false) != 0)
    {
        printf("init failed\n");
        return r;
    }
    for (int i = 0; i < 2; i++)
    {
        if (sc->rx_queue)
        {
            ep[i].q_size = sc->rx_queue;
            ep[i].q = malloc((size_t)sc->rx_queue);
            /* As the engine does: room for one worst-case frame's output. */
            if (!getenv("V42_STRESS_NO_FLOW"))
                ds_set_rx_room(&ds[i], end_room, &ep[i], sc->rx_queue/2, sc->rx_queue*3/4);
        }
    }
    for (now_tick = 0;  now_tick < total;  now_tick++)
    {
        for (int i = 0; i < 2; i++)
        {
            acc[i] += dte_per_tick_q16;
            /* Bounded DTE queue: the pull refuses beyond a few octets, as the
               engine's ring does when the PTY is idle. */
            if (ep[i].tx_budget < 256)
                ep[i].tx_budget += acc[i] >> 16;
            acc[i] &= 0xFFFF;
        }
        for (int i = 0; i < 2; i++)
        {
            if (ep[i].q)
            {
                ep[i].drain_acc += sc->drain_bps;
                if (ep[i].drain_acc >= sc->rate)
                {
                    end_drain(&ep[i], ep[i].drain_acc/sc->rate);
                    ep[i].drain_acc %= sc->rate;
                }
            }
        }
        int b0 = ds_tx_get_bit(&ds[0]);
        int b1 = ds_tx_get_bit(&ds[1]);
        ds_rx_put_bit(&ds[1], channel_pass(&ch[0], sc, now_tick, b0));
        ds_rx_put_bit(&ds[0], channel_pass(&ch[1], sc, now_tick, b1));
    }
    for (int i = 0; i < 2; i++)
    {
        uint64_t gap = total - ep[i].last_rx_tick;
        double stall;

        if (ep[i].max_gap > gap)
            gap = ep[i].max_gap;
        stall = (double)gap/sc->rate;

        r.kbps[i] = 8.0*ep[i].received/sc->seconds/1000.0;
        r.bad[i] = ep[i].bad;
        r.disconnects += ep[i].disconnects;
        r.errors += ep[i].errors;
        if (stall > r.stall_s)
            r.stall_s = stall;
    }
    /* Pass: nothing corrupted, the link carried data to the end (a stall can
       last at most one outage plus recovery), and each direction met its
       floor.  Teardowns are failures: none of these lines is bad enough to
       justify one under N400 = 20. */
    double allowed_stall = ((sc->max_stall_s > 0) ? sc->max_stall_s : 5.0)
                           + (sc->outage_every_ms ? sc->outage_ms/1000.0 : 0.0);
    r.overruns = ep[0].overruns + ep[1].overruns;
    r.pass = r.bad[0] == 0  &&  r.bad[1] == 0  &&  r.overruns == 0
             &&  r.stall_s <= allowed_stall
             &&  r.disconnects == 0  &&  r.errors == 0
             &&  r.kbps[0] >= sc->min_kbps  &&  r.kbps[1] >= sc->min_kbps
             &&  ds_link_is_ready(&ds[0])  &&  ds_link_is_ready(&ds[1]);
    if (verbose)
    {
        for (int i = 0; i < 2; i++)
        {
            printf("  %s: rx %llu octets (%.1f kbit/s DTE), bad %llu (first at %llu), connects %d, disc %d, err %d, ready %d, wire tx %llu rx %llu\n",
                   i ? "answerer" : "caller  ",
                   (unsigned long long)ep[i].received, r.kbps[i],
                   (unsigned long long)ep[i].bad, (unsigned long long)ep[i].first_bad_at,
                   ep[i].connects, ep[i].disconnects, ep[i].errors,
                   ds_link_is_ready(&ds[i]),
                   (unsigned long long)ds[i].v42_tx_wire_bytes, (unsigned long long)ds[i].v42_rx_wire_bytes);
        }
        for (int i = 0; i < 2; i++)
            printf("  %s longest gap %.2f s ending at %.2f s\n", i ? "answerer" : "caller",
                   (double)ep[i].max_gap/sc->rate, (double)ep[i].max_gap_end/sc->rate);
        printf("  DTE queue overruns %llu/%llu, busy spells %llu/%llu\n",
               (unsigned long long)ep[0].overruns, (unsigned long long)ep[1].overruns,
               (unsigned long long)ds[0].rx_busy_spells, (unsigned long long)ds[1].rx_busy_spells);
        printf("  longest stall %.2f s, line flips %llu/%llu\n", r.stall_s,
               (unsigned long long)ch[0].flips, (unsigned long long)ch[1].flips);
    }
    for (int i = 0; i < 2; i++)
    {
        ds_release(&ds[i]);
        free(ch[i].ring);
        free(ep[i].q);
    }
    return r;
}

/* V42_STRESS_NO_FLOW=1 runs the slow-DTE rows without ds_set_rx_room(): the
   negative control (hundreds of thousands of octets lost). */
int main(int argc, char **argv)
{
    int n = (int)(sizeof(scenarios)/sizeof(scenarios[0]));
    int failures = 0;
    int seeds = 3;

    if (argc >= 3)
    {
        for (int i = 0; i < n; i++)
        {
            if (strcmp(argv[1], scenarios[i].name) == 0)
            {
                result_t r = run(&scenarios[i], strtoull(argv[2], NULL, 0), true);

                printf("%s: %s\n", r.pass ? "PASS" : "FAIL", scenarios[i].name);
                return r.pass ? 0 : 1;
            }
        }
        fprintf(stderr, "unknown scenario %s\n", argv[1]);
        return 2;
    }
    if (argc == 2)
        seeds = atoi(argv[1]);
    for (int i = 0; i < n; i++)
    {
        for (int seed = 1; seed <= seeds; seed++)
        {
            result_t r = run(&scenarios[i], (uint64_t)seed, false);

            printf("%s: %-20s seed %d  %6.1f / %6.1f kbit/s  bad %llu/%llu  overrun %llu  disc %d err %d  stall %.1f s\n",
                   r.pass ? "PASS" : "FAIL", scenarios[i].name, seed,
                   r.kbps[0], r.kbps[1],
                   (unsigned long long)r.bad[0], (unsigned long long)r.bad[1],
                   (unsigned long long)r.overruns,
                   r.disconnects, r.errors, r.stall_s);
            if (!r.pass)
                failures++;
        }
    }
    printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
