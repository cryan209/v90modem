/* K56flex training/probe sample generator (Draft 0.23 clause 4.10).
 *
 * Independent lift of module 8C's D873 block generator: source fetch (90FF),
 * sequence selector D890, level lookup D83C and sign/DC adjuster D84F, driven
 * by the configuration words D811 loads per stage.  Each call yields one
 * six-sample block (a "pair" is two blocks).  Output samples are firmware
 * level words, converted to G.711 with the same law rules as the data mapper.
 */
#ifndef K56FLEX_PROBE_H
#define K56FLEX_PROBE_H

#include "k56flex.h"

#include <stddef.h>
#include <stdint.h>

typedef enum {
    K56FLEX_PROBE_SILENCE = 0,   /* D9BE */
    K56FLEX_PROBE_ID,            /* D960, raw words (callback 8F6F) */
    K56FLEX_PROBE_1,             /* D971 */
    K56FLEX_PROBE_2,             /* D97E, sequence state 8F88/8F89 = 0002/0800 */
    K56FLEX_PROBE_3,             /* D98B */
    K56FLEX_PROBE_PARAM_A,       /* D99C */
    K56FLEX_PROBE_PARAM_B        /* D9AD, selected when report bit 8 */
} k56flex_probe_stage_t;

#define K56FLEX_PROBE_MAX_SAMPLES 6

typedef struct {
    const void *table;
    uint16_t cfg[12];
    const int16_t *levels;
    uint32_t seq;                /* DM 8F88:8F89 */
    int32_t dc;                  /* DM 8F9E:8F9D */
    int scrambled;               /* 0 = raw callback, 1 = BE68 */
    uint32_t scr_hist;
    /* source: a bit FIFO of (scrambled) words; when it runs dry the most recent
     * raw word is replayed (90FF's underflow path, matched against the firmware) */
    uint8_t fifo[512];           /* raw source bits, ring */
    unsigned fifo_head, fifo_bits;
    uint16_t last_word;
    uint8_t law;
} k56flex_probe_t;

/* flags is the DM 8FEF pad-table group (0, 0x40 or 0x80).  Returns 0 or -1. */
int k56flex_probe_init(k56flex_probe_t *p, k56flex_probe_stage_t stage, k56flex_law_t law,
                       unsigned flags);
/* Replace the source with one seed word (90BD: also resets the scrambler and
 * discards queued bits). */
void k56flex_probe_seed(k56flex_probe_t *p, uint16_t word);
/* Append a source word (90D0). */
void k56flex_probe_append(k56flex_probe_t *p, uint16_t word);
/* Next block of level words; returns the sample count (6). */
unsigned k56flex_probe_block(k56flex_probe_t *p, int16_t out[K56FLEX_PROBE_MAX_SAMPLES]);

/* One block from explicit source bits (the cfg[3] bits fetch would have returned);
 * updates sequence and DC state only. */
unsigned k56flex_probe_block_src(k56flex_probe_t *p, uint32_t src, int16_t out[K56FLEX_PROBE_MAX_SAMPLES]);
/* Brute-force inverse of one block under the current sequence/DC state: returns
 * how many source values reproduce `samples` (1 = unambiguous), *src the first. */
unsigned k56flex_probe_invert(const k56flex_probe_t *p, const int16_t samples[K56FLEX_PROBE_MAX_SAMPLES],
                              uint32_t *src);

#endif
