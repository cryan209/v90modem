/* K56flex server transmit sequencer: silence, identification, three probes,
 * parameter training, parameter record, priming frames and the hand-off to the
 * data mapper (Draft 0.23 clauses 4.10-4.12).
 *
 * The fixed stretches (pair counts, stage order, source words, state resets)
 * come from the DB11/DB2E/D9CB control flow and are checked stage by stage
 * against the original sample generator (k56flex_train_vectors.h).  Stretches
 * that the firmware leaves to the peer are gates driven by DM 8F31 status bits;
 * nothing on the wire tells us how a real client signals them, so they are
 * supplied by the caller (k56flex_train_status) and are Open in the spec.
 */
#ifndef K56FLEX_TRAIN_H
#define K56FLEX_TRAIN_H

#include "k56flex.h"
#include "k56flex_probe.h"

typedef enum {
    K56T_SIL0 = 0,    /* 47 pairs of D9BE silence */
    K56T_ID_A,        /* DBB5: E4E4, 22 pairs */
    K56T_ID_B,        /* then 1B1B, 2 pairs */
    K56T_P1,          /* DB37: 1364 pairs */
    K56T_P2,          /* DB68: 704 pairs */
    K56T_P3,          /* DB7F: 1366 pairs */
    K56T_GATE_A,      /* DBA8: P3 + training word until 8F31 bit 11 */
    K56T_GATE_B,      /* silence until bit 10 */
    K56T_ID2_A,       /* D9CB: identification again */
    K56T_ID2_B,
    K56T_PT_A,        /* 688 pairs of D98B */
    K56T_PT_B,        /* FFFF FFFF 0000, 8 pairs */
    K56T_PT_C,        /* D99C/D9AD, 340 pairs */
    K56T_GATE_C,      /* four blocks per poll until bit 12 or 13 */
    K56T_PARAM_1,     /* parameter record (argument 0) until bit 13 or 14 */
    K56T_PARAM_2,     /* record rebuilt with argument 1 until bit 14 */
    K56T_TAIL,        /* DA75: FFFF and one (or two) pairs */
    K56T_PRIME,       /* DC3B: six priming frames */
    K56T_DATA,
    K56T_FAILED       /* gate timed out (only if a timeout was configured) */
} k56flex_train_phase_t;

#define K56FLEX_STATUS_PROBE_PEER   (1u << 11)   /* DM 8F31 bit 11 */
#define K56FLEX_STATUS_SILENCE_END  (1u << 10)
#define K56FLEX_STATUS_TRAIN_READY  ((1u << 12) | (1u << 13))
#define K56FLEX_STATUS_PARAM_RESP   ((1u << 13) | (1u << 14))
#define K56FLEX_STATUS_PARAM_DONE   (1u << 14)

typedef struct {
    k56flex_law_t law;
    int rate_bps;                /* negotiated downstream rate */
    unsigned report_field;       /* DM 8FEF: 0 until the client's report is accepted */
    k56flex_param_t param;       /* record fields; param.rate is filled from rate_bps */
    uint16_t training_word;      /* B31D's word (unknown); appended at gate A */
    unsigned gate_timeout_pairs; /* 0 = wait forever */
} k56flex_train_cfg_t;

typedef struct {
    k56flex_train_cfg_t cfg;
    k56flex_train_phase_t phase;
    k56flex_probe_t probe;
    unsigned pairs_left, gate_pairs, prime_left;
    unsigned blocks_in_pair;     /* block index within the current pair */
    unsigned status;
    unsigned pairs_total;
    int param_second;
    k56flex_pcm_tx_t data;
    int data_ready;
    k56flex_source_fn data_source;
    void *data_user;
    int16_t buf[8];
    unsigned buf_len, buf_pos;
    unsigned prime_word_idx;
} k56flex_train_t;

int k56flex_train_init(k56flex_train_t *t, const k56flex_train_cfg_t *cfg);
/* OR new DM 8F31-style status bits in (the sequencer clears the ones the firmware clears). */
void k56flex_train_status(k56flex_train_t *t, unsigned bits);
/* The client's accepted report (k56flex_report_field), needed before PRIME. */
void k56flex_train_set_report(k56flex_train_t *t, unsigned report_field);
void k56flex_train_set_data_source(k56flex_train_t *t, k56flex_source_fn fn, void *user);
/* Produce n level words; may return fewer only in K56T_DATA when the source pauses. */
size_t k56flex_train_samples(k56flex_train_t *t, int16_t *out, size_t n);
size_t k56flex_train_g711(k56flex_train_t *t, uint8_t *out, size_t n);
const char *k56flex_train_phase_name(k56flex_train_phase_t p);
/* Pairs completed in the current phase boundaries are visible through this. */
k56flex_train_phase_t k56flex_train_phase(const k56flex_train_t *t);
/* Jump the phase machine (a receiver mirroring the server from the signal it observes). */
void k56flex_train_force_phase(k56flex_train_t *t, k56flex_train_phase_t ph);

#endif
