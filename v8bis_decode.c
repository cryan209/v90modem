/*
 * v8bis_decode.c — V.8bis signal and message decoder
 *
 * Implements ITU-T V.8bis (08/96) tone signal detection and
 * HDLC-framed V.21 FSK message decoding for the vpcm_decode tool.
 *
 * V.8bis signals consist of two segments:
 *   Segment 1 (400 ms): dual-tone pair to break through echo suppressors
 *     Initiating: 1375 + 2002 Hz
 *     Responding: 1529 + 2225 Hz
 *   Segment 2 (100 ms): single tone identifying the signal type
 *     MRe=650, CRe=400, ESi=980, MRd=1150, CRd=1900, ESr=1650 Hz
 *
 * V.8bis messages use V.21 modulation (300 bps FSK) with HDLC framing.
 *   CH1 (1080/1180 Hz) for initiating station
 *   CH2 (1750/1850 Hz) for responding station
 * Frame structure: Flag(s) | Information | FCS(16) | Flag(s)
 * Information field: [msg_type:4 | revision:4] [parameters...]
 */

#include "v8bis_decode.h"

#include <spandsp.h>
#include <spandsp/private/power_meter.h>
#include <spandsp/private/fsk.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* Signal definition table (ITU-T V.8bis §7.1, Tables 1 & 2)         */
/* ------------------------------------------------------------------ */

const v8bis_signal_def_t g_v8bis_signal_defs[V8BIS_NUM_SIGNALS] = {
    /* Initiating signals (Segment 1: 1375 + 2002 Hz) */
    { "MRe",   "initiating", 1375, 2002,  650 },  /* Mode Request establishing (auto-answer) */
    { "CRe",   "initiating", 1375, 2002,  400 },  /* Capabilities Request establishing (auto-answer) */
    { "ESi",   "initiating", 1375, 2002,  980 },  /* Escape Signal initiating */
    { "MRd_i", "initiating", 1375, 2002, 1150 },  /* Mode Request during call (initiating side) */
    { "CRd_i", "initiating", 1375, 2002, 1900 },  /* Capabilities Request during call (initiating side) */
    /* Responding signals (Segment 1: 1529 + 2225 Hz) */
    { "MRd",   "responding", 1529, 2225, 1150 },  /* Mode Request during call (responding) */
    { "CRd",   "responding", 1529, 2225, 1900 },  /* Capabilities Request during call (responding) */
    { "ESr",   "responding", 1529, 2225, 1650 }   /* Escape Signal responding */
};

/* ------------------------------------------------------------------ */
/* Tone signal scanning                                               */
/* ------------------------------------------------------------------ */

bool v8bis_scan_signals(const int16_t *samples,
                        int total_samples,
                        int max_sample,
                        v8bis_scan_result_t *out)
{
    int limit;

    if (!samples || total_samples <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;
    if (limit < V8BIS_SIGNAL_SAMPLES)
        return false;

    for (int offset = 0; offset + V8BIS_SIGNAL_SAMPLES <= limit; offset += V8BIS_SCAN_STEP_SAMPLES) {
        const int16_t *seg1 = samples + offset;
        const int16_t *seg2 = seg1 + V8BIS_SEGMENT1_SAMPLES;
        double seg1_energy = window_energy(seg1, V8BIS_SEGMENT1_SAMPLES);
        double seg2_energy = window_energy(seg2, V8BIS_SEGMENT2_SAMPLES);

        if (seg1_energy <= 1.0 || seg2_energy <= 1.0)
            continue;

        for (int i = 0; i < V8BIS_NUM_SIGNALS; i++) {
            const v8bis_signal_def_t *def = &g_v8bis_signal_defs[i];
            double seg1_a_ratio = tone_energy_ratio(seg1, V8BIS_SEGMENT1_SAMPLES, 8000, def->seg1_a_hz, seg1_energy);
            double seg1_b_ratio = tone_energy_ratio(seg1, V8BIS_SEGMENT1_SAMPLES, 8000, def->seg1_b_hz, seg1_energy);
            double seg2_ratio = tone_energy_ratio(seg2, V8BIS_SEGMENT2_SAMPLES, 8000, def->seg2_hz, seg2_energy);
            double dual_ratio = seg1_a_ratio + seg1_b_ratio;
            double balance = 0.0;
            double score;

            if (seg1_a_ratio > 0.0 && seg1_b_ratio > 0.0) {
                double hi = (seg1_a_ratio > seg1_b_ratio) ? seg1_a_ratio : seg1_b_ratio;
                double lo = (seg1_a_ratio > seg1_b_ratio) ? seg1_b_ratio : seg1_a_ratio;
                balance = lo / hi;
            }

            if (dual_ratio < 0.22 || seg1_a_ratio < 0.07 || seg1_b_ratio < 0.07 || balance < 0.25 || seg2_ratio < 0.15)
                continue;

            score = dual_ratio * 1000.0 + seg2_ratio * 1200.0 + balance * 200.0;
            if (!out->hits[i].seen || score > out->hits[i].score) {
                out->hits[i].seen = true;
                out->hits[i].sample_offset = offset;
                out->hits[i].duration_samples = V8BIS_SIGNAL_SAMPLES;
                out->hits[i].score = score;
                out->hits[i].seg1_a_ratio = seg1_a_ratio;
                out->hits[i].seg1_b_ratio = seg1_b_ratio;
                out->hits[i].seg1_dual_ratio = dual_ratio;
                out->hits[i].seg1_balance = balance;
                out->hits[i].seg2_ratio = seg2_ratio;
            }
        }
    }

    for (int i = 0; i < V8BIS_NUM_SIGNALS; i++) {
        if (out->hits[i].seen)
            return true;
    }
    return false;
}

bool v8bis_scan_weak_candidate(const int16_t *samples,
                               int total_samples,
                               int max_sample,
                               v8bis_weak_candidate_t *out)
{
    int limit;
    v8bis_weak_candidate_t best = {0};

    if (!samples || total_samples <= 0 || !out)
        return false;

    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;
    if (limit < V8BIS_SIGNAL_SAMPLES)
        return false;

    for (int offset = 0; offset + V8BIS_SIGNAL_SAMPLES <= limit; offset += V8BIS_SCAN_STEP_SAMPLES) {
        const int16_t *seg1 = samples + offset;
        const int16_t *seg2 = seg1 + V8BIS_SEGMENT1_SAMPLES;
        double seg1_energy = window_energy(seg1, V8BIS_SEGMENT1_SAMPLES);
        double seg2_energy = window_energy(seg2, V8BIS_SEGMENT2_SAMPLES);

        if (seg1_energy <= 1.0 || seg2_energy <= 1.0)
            continue;

        for (int i = 0; i < V8BIS_NUM_SIGNALS; i++) {
            const v8bis_signal_def_t *def = &g_v8bis_signal_defs[i];
            double seg1_a_ratio = tone_energy_ratio(seg1, V8BIS_SEGMENT1_SAMPLES, 8000, def->seg1_a_hz, seg1_energy);
            double seg1_b_ratio = tone_energy_ratio(seg1, V8BIS_SEGMENT1_SAMPLES, 8000, def->seg1_b_hz, seg1_energy);
            double seg2_ratio = tone_energy_ratio(seg2, V8BIS_SEGMENT2_SAMPLES, 8000, def->seg2_hz, seg2_energy);
            double dual_ratio = seg1_a_ratio + seg1_b_ratio;
            double balance = 0.0;
            double score;

            if (seg1_a_ratio > 0.0 && seg1_b_ratio > 0.0) {
                double hi = (seg1_a_ratio > seg1_b_ratio) ? seg1_a_ratio : seg1_b_ratio;
                double lo = (seg1_a_ratio > seg1_b_ratio) ? seg1_b_ratio : seg1_a_ratio;
                balance = lo / hi;
            }

            /* Weaker thresholds than v8bis_scan_signals */
            if (dual_ratio < 0.14 || seg1_a_ratio < 0.04 || seg1_b_ratio < 0.04 || balance < 0.15 || seg2_ratio < 0.08)
                continue;

            score = dual_ratio * 1000.0 + seg2_ratio * 1200.0 + balance * 200.0;
            if (!best.seen || score > best.score) {
                best.seen = true;
                best.signal_index = i;
                best.sample_offset = offset;
                best.duration_samples = V8BIS_SIGNAL_SAMPLES;
                best.score = score;
                best.seg1_a_ratio = seg1_a_ratio;
                best.seg1_b_ratio = seg1_b_ratio;
                best.seg1_dual_ratio = dual_ratio;
                best.seg1_balance = balance;
                best.seg2_ratio = seg2_ratio;
            }
        }
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}

/* ------------------------------------------------------------------ */
/* V.8bis HDLC message decoder (V.21 FSK at 300 bps)                 */
/* ------------------------------------------------------------------ */

/* Message types per V.8bis Table 3 (§8.3.1) */
const char *v8bis_msg_type_str(int type)
{
    switch (type) {
    case 0x1: return "MS";      /* Mode Select */
    case 0x2: return "CL";      /* Capabilities List */
    case 0x3: return "CLR";     /* Capabilities List Request */
    case 0x4: return "ACK(1)";  /* Acknowledge / terminate transaction */
    case 0x5: return "ACK(2)";  /* Acknowledge / request more info */
    case 0x8: return "NAK(1)";  /* Cannot interpret */
    case 0x9: return "NAK(2)";  /* Temporarily unable */
    case 0xA: return "NAK(3)";  /* Does not support requested mode */
    case 0xB: return "NAK(4)";  /* Cannot interpret, request retransmit */
    default:  return "unknown";
    }
}

/* V.8bis SPar(1) mode capabilities (§8.4, Table 6-2) — public API kept for compatibility */
const char *v8bis_spar1_mode_str(int spar1_bits)
{
    if (spar1_bits & 0x01) return "Data";
    if (spar1_bits & 0x02) return "Simultaneous voice and data";
    if (spar1_bits & 0x04) return "H.324 multimedia";
    if (spar1_bits & 0x08) return "V.18 text telephone";
    if (spar1_bits & 0x10) return "Reserved";
    if (spar1_bits & 0x20) return "Analogue telephony";
    if (spar1_bits & 0x40) return "T.101 videotex";
    return "unknown";
}

/* ------------------------------------------------------------------ */
/* Full V.8bis information field decoder (§8.1–§8.5)                  */
/* ------------------------------------------------------------------ */

/*
 * Read one level-1 parameter block delimited by bit 8 (§8.2.3).
 * Collects bits 1–7 from successive octets until bit 8 = 1.
 * Returns bytes consumed; sets *bits (up to 28) and *noctets consumed.
 */
static int v8bis_l1_block(const uint8_t *p, int len,
                           uint32_t *bits, int *noctets)
{
    int i = 0, shift = 0;
    *bits = 0;
    *noctets = 0;
    while (i < len) {
        uint8_t b = p[i++];
        if (shift < 28)
            *bits |= (uint32_t)(b & 0x7F) << shift;
        shift += 7;
        if (b & 0x80) break;  /* bit 8 = 1: last octet in block */
    }
    *noctets = i;
    return i;
}

/*
 * Read one NPar(2) sub-block within a Par(2) group (§8.2.3).
 * Bits 1–6 carry data; bit 7 terminates the NPar(2) sub-block;
 * bit 8 terminates the entire Par(2) group.
 * Returns bytes consumed; *bits gets data bits (up to 30);
 * *end_par2 is set when bit 8 is seen.
 */
static int v8bis_npar2_sub(const uint8_t *p, int len,
                            uint32_t *bits, bool *end_par2)
{
    int i = 0, shift = 0;
    *bits = 0;
    *end_par2 = false;
    while (i < len) {
        uint8_t b = p[i++];
        if (shift < 30)
            *bits |= (uint32_t)(b & 0x3F) << shift;
        shift += 6;
        if (b & 0x80) { *end_par2 = true; break; }  /* last Par(2) octet */
        if (b & 0x40) break;                          /* last NPar(2) octet */
    }
    return i;
}

/*
 * Skip remaining octets of a Par(2) group after NPar(2) is read
 * but end_par2 was not set (i.e. SPar(2)/NPar(3) follow).
 * Reads until bit 8 = 1.
 */
static int v8bis_skip_par2_tail(const uint8_t *p, int len)
{
    for (int i = 0; i < len; i++)
        if (p[i] & 0x80)
            return i + 1;
    return len;
}

/*
 * Append the names of all set bits in 'bits' to buf.
 * names[] maps 0-based bit positions → strings (NULL = unnamed/skip).
 * Returns updated pos. Writes "(none)" if no named bits are set.
 */
static int v8bis_append_bits(char *buf, int sz, int pos,
                              uint32_t bits, int nbits,
                              const char * const *names, int nnames)
{
    bool found = false;
    for (int i = 0; i < nbits && i < nnames; i++) {
        if (((bits >> i) & 1) && names[i]) {
            pos += snprintf(buf + pos, sz - pos, "%s%s", found ? "+" : "", names[i]);
            found = true;
        }
    }
    if (!found)
        pos += snprintf(buf + pos, sz - pos, "none");
    return pos;
}

/*
 * Decode the full V.8bis information field into buf.
 *
 * info[] holds all bytes AFTER the [revision:4|type:4] header octet.
 * Layout (V.8bis §8.1–§8.5):
 *
 *   I-field:
 *     NPar(1)  — Table 5-1: V.8, ShortV8, addl-info, ACK1, NS-field
 *     SPar(1)  — Table 5-2: Network type
 *     Par(2)_n — one per set SPar(1) bit; NPar(2) from Table 5-3
 *
 *   S-field:
 *     NPar(1)  — Table 6-1: NS capabilities
 *     SPar(1)  — Table 6-2: Data, SVD, H.324, V.18, Rsvd, Analogue, T.101
 *     Par(2)_n — one per set SPar(1) bit; NPar(2) from Tables 6-3 to 6-8
 *
 *   NS field (if I NPar(1) bit 7 set):
 *     One or more NS blocks: [len][T.35 country code][provider code len][provider code][data]
 *
 * Returns number of characters written (excluding NUL).
 */
static int v8bis_decode_info_field(const uint8_t *info, int info_len,
                                    char *buf, int buf_sz)
{
    /* ── Name tables (0-based bit indexing within each block) ───────── */

    /* I-field NPar(1) — Table 5-1 */
    static const char * const inpar1_names[] =
        { "V8", "ShortV8", "addl", "ack1", NULL, NULL, "NS" };

    /* I-field SPar(1) — Table 5-2 (bit 1 only; rest reserved) */
    /* I-field Par(2) for network type — Table 5-3 */
    static const char * const net_npar2_names[] =
        { "Cellular", "ISDN", NULL, NULL, NULL, "NS-net" };

    /* S-field NPar(1) — Table 6-1 (bit 7 = non-standard capabilities) */
    /* S-field SPar(1) — Table 6-2 */
    static const char * const sspar1_names[] =
        { "Data", "SVD", "H.324", "V.18", NULL, "Analogue", "T.101" };

    /* S-field Par(2) NPar(2) tables, keyed by SPar(1) bit position (0-based) */
    /* Table 6-3: Data NPar(2) — 3 octets × 6 data bits = 18 bits */
    static const char * const data_npar2[] = {
        "transparent", "V.42", "V.42bis", "V.14", "T.120", "NS",
        "T.84",        "T.434","V.80",    NULL,   "V.34",  "V.32bis",
        "V.32",        "V.22bis","V.22",  "V.21", NULL,    NULL
    };
    /* Table 6-4: SVD NPar(2) — 3 octets */
    static const char * const svd_npar2[] = {
        "V.70", "V.61", NULL, "V.34", "V.32bis", "NS",
        "transparent", "V.42", "V.42bis", "V.14", "T.120", "V.80",
        "T.84", "T.434", NULL, NULL, NULL, NULL
    };
    /* Table 6-5: H.324 NPar(2) — 1 octet */
    static const char * const h324_npar2[] =
        { "Video", "Audio", "Encrypt", NULL, NULL, "NS" };
    /* Table 6-6: V.18 NPar(2) — 1 octet */
    static const char * const v18_npar2[] =
        { "V.21", "V.61", NULL, NULL, NULL, "NS" };
    /* Table 6-7: Analogue telephony NPar(2) — 1 octet */
    static const char * const at_npar2[] =
        { "Voice", "AudioRec", "VoiceBridge", NULL, NULL, "NS" };
    /* Table 6-8: T.101 videotex NPar(2) — 1 octet */
    static const char * const t101_npar2[] =
        { "Duplex", "V.29short", "V.27ter", NULL, NULL, "NS" };

    /* Dispatch table: pointer and max named bits per S SPar(1) bit position */
    static const char * const * const mode_tables[] = {
        data_npar2, svd_npar2, h324_npar2, v18_npar2, NULL, at_npar2, t101_npar2
    };
    static const int mode_nbits[] = { 18, 18, 6, 6, 0, 6, 6 };
    static const char * const mode_labels[] = {
        "Data", "SVD", "H.324", "V.18", NULL, "Analogue", "T.101"
    };

    int p = 0;   /* current read position in info[] */
    int pos = 0; /* current write position in buf */
    uint32_t bits;
    bool end_par2;
    int noctets;
    int n;

    /* ── I-field NPar(1) (Table 5-1) ───────────────────────────────── */
    uint32_t inpar1 = 0;
    n = v8bis_l1_block(info + p, info_len - p, &inpar1, &noctets);
    p += n;
    pos += snprintf(buf + pos, buf_sz - pos, "I:[");
    pos  = v8bis_append_bits(buf, buf_sz, pos, inpar1, 7, inpar1_names, 7);
    pos += snprintf(buf + pos, buf_sz - pos, "]");

    /* ── I-field SPar(1) (Table 5-2) + Par(2) blocks ───────────────── */
    uint32_t ispar1 = 0;
    if (p < info_len) {
        n = v8bis_l1_block(info + p, info_len - p, &ispar1, &noctets);
        p += n;
    }
    for (int mode = 0; mode < 7 && p < info_len; mode++) {
        if (!((ispar1 >> mode) & 1)) continue;
        n = v8bis_npar2_sub(info + p, info_len - p, &bits, &end_par2);
        p += n;
        if (!end_par2 && p < info_len)
            p += v8bis_skip_par2_tail(info + p, info_len - p);
        if (mode == 0 && bits) {
            /* Network type NPar(2), Table 5-3 */
            pos += snprintf(buf + pos, buf_sz - pos, " net=[");
            pos  = v8bis_append_bits(buf, buf_sz, pos, bits, 6, net_npar2_names, 6);
            pos += snprintf(buf + pos, buf_sz - pos, "]");
        }
    }

    /* ── S-field NPar(1) (Table 6-1) ───────────────────────────────── */
    uint32_t snpar1 = 0;
    if (p < info_len) {
        n = v8bis_l1_block(info + p, info_len - p, &snpar1, &noctets);
        p += n;
    }

    /* ── S-field SPar(1) (Table 6-2) ───────────────────────────────── */
    uint32_t sspar1 = 0;
    if (p < info_len) {
        n = v8bis_l1_block(info + p, info_len - p, &sspar1, &noctets);
        p += n;
    }
    pos += snprintf(buf + pos, buf_sz - pos, " S:[");
    pos  = v8bis_append_bits(buf, buf_sz, pos, sspar1, 7, sspar1_names, 7);
    pos += snprintf(buf + pos, buf_sz - pos, "]");
    if (snpar1 & 0x40)
        pos += snprintf(buf + pos, buf_sz - pos, " S-NS");

    /* ── S-field Par(2) groups — one per set SPar(1) bit ───────────── */
    for (int mode = 0; mode < 7 && p < info_len; mode++) {
        if (!((sspar1 >> mode) & 1)) continue;
        n = v8bis_npar2_sub(info + p, info_len - p, &bits, &end_par2);
        p += n;
        if (!end_par2 && p < info_len)
            p += v8bis_skip_par2_tail(info + p, info_len - p);
        if (!mode_tables[mode] || !mode_labels[mode] || bits == 0) continue;
        pos += snprintf(buf + pos, buf_sz - pos, " %s:[", mode_labels[mode]);
        pos  = v8bis_append_bits(buf, buf_sz, pos, bits,
                                  mode_nbits[mode],
                                  mode_tables[mode], mode_nbits[mode]);
        pos += snprintf(buf + pos, buf_sz - pos, "]");
    }

    /* ── NS field (§8.5) ─────────────────────────────────────────────
     * Present when I NPar(1) bit 7 (0x40) is set.
     * Each NS block: [total-len][T.35-cc K octets][pcode-len][pcode L octets][data M octets] */
    if ((inpar1 & 0x40) && p < info_len) {
        bool ns_first = true;
        pos += snprintf(buf + pos, buf_sz - pos, " NS:[");
        while (p < info_len) {
            int blk_remaining = info[p++];          /* total length of rest of block */
            int blk_end = p + blk_remaining;
            if (blk_end > info_len) blk_end = info_len;
            if (!ns_first) pos += snprintf(buf + pos, buf_sz - pos, "|");
            ns_first = false;
            /* T.35 country code: 0xFF = escape (2-octet code), else 1 octet */
            if (p < blk_end) {
                uint8_t cc1 = info[p++];
                if (cc1 == 0xFF && p < blk_end) {
                    uint8_t cc2 = info[p++];
                    pos += snprintf(buf + pos, buf_sz - pos, "T35=0xFF%02X", cc2);
                } else {
                    pos += snprintf(buf + pos, buf_sz - pos, "T35=0x%02X", cc1);
                }
            }
            /* Provider code: 1-byte length + L bytes */
            if (p < blk_end) {
                int pc_len = info[p++];
                if (pc_len > 0 && p + pc_len <= blk_end) {
                    pos += snprintf(buf + pos, buf_sz - pos, " pcode=");
                    for (int i = 0; i < pc_len && pos < buf_sz - 3; i++)
                        pos += snprintf(buf + pos, buf_sz - pos, "%02X", info[p++]);
                }
            }
            /* Non-standard data remainder */
            int ns_data = blk_end - p;
            if (ns_data > 0) {
                pos += snprintf(buf + pos, buf_sz - pos, " nsdata=%d", ns_data);
                p = blk_end;
            }
        }
        pos += snprintf(buf + pos, buf_sz - pos, "]");
    }

    /* Raw hex of full info[] for cross-checking */
    if (info_len > 0) {
        int hlen = (info_len < 16) ? info_len : 16;
        pos += snprintf(buf + pos, buf_sz - pos, " raw=[");
        for (int i = 0; i < hlen && pos < buf_sz - 5; i++)
            pos += snprintf(buf + pos, buf_sz - pos, "%s%02X", i == 0 ? "" : "-", info[i]);
        if (info_len > 16)
            pos += snprintf(buf + pos, buf_sz - pos, "...");
        pos += snprintf(buf + pos, buf_sz - pos, "]");
    }

    return pos;
}

/* HDLC receiver state for one V.21 channel */
typedef enum {
    V8BIS_PARTIAL_SHORT_FRAME = 0,
    V8BIS_PARTIAL_NON_BYTE_ALIGNED,
    V8BIS_PARTIAL_BAD_CRC,
    V8BIS_PARTIAL_ABORT_SEQUENCE,
    V8BIS_PARTIAL_SHORT_PREAMBLE,   /* < 24 marking bits before first flag (Defect 4) */
    V8BIS_PARTIAL_SINGLE_FLAG       /* < 2 opening flags before info field (Defect 5) */
} v8bis_partial_reason_t;

typedef struct {
    int sample_offset;
    int channel;
    int frame_byte_count;
    int byte_bits;
    bool crc_ok;
    v8bis_partial_reason_t reason;
    uint8_t first_octet;
    uint8_t frame_buf[16];
} v8bis_partial_frame_t;

typedef struct {
    /* HDLC decoding state */
    int ones_run;
    bool flag_seen;
    bool in_frame;
    uint8_t frame_buf[80];
    int frame_byte_count;
    int byte_bits;
    uint8_t byte_val;
    int frame_start_bit;
    int bit_count;
    /* Preamble and flag validation (Defects 4 & 5) */
    int preamble_ones;          /* consecutive marking bits before first opening flag */
    int frame_preamble_ones;    /* preamble_ones snapshot when the first flag fired */
    int opening_flag_count;     /* consecutive flags before any data bytes */
    /* Sample position tracking */
    int current_chunk_sample;
    int carrier_on_sample;
    /* Results */
    v8bis_decoded_msg_t msgs[V8BIS_MSG_MAX];
    int msg_count;
    v8bis_partial_frame_t partials[V8BIS_MSG_MAX];
    int partial_count;
    int channel;
} v8bis_hdlc_rx_t;

typedef struct {
    bool ok;
    const char *name;           /* QC2a, QCA2a, QC2d, QCA2d */
    bool digital_modem;
    bool qca;
    bool lapm;
    int revision;               /* V.8bis revision nibble */
    int wxyz;                   /* analogue forms */
    int uqts_ucode;             /* analogue forms */
    int lm;                     /* digital forms */
    uint8_t id_octet;           /* bits 8..15 */
    bool reserved_bits_set;     /* non-zero reserved bits (log only, not fatal) */
    bool channel_mismatch;      /* arrived on unexpected V.21 channel */
    int expected_channel;       /* 0=CH1(QCA2x), 1=CH2(QC2x) */
} v92_qc2_id_t;

static int v92_uqts_from_wxyz(int wxyz)
{
    static const int table[16] = {
        61, 62, 63, 66,
        67, 70, 71, 74,
        75, 78, 79, 82,
        83, 86, 87, 87
    };

    if (wxyz < 0 || wxyz > 15)
        return -1;
    return table[wxyz];
}

static const char *v92_anspcm_level_to_str(int level)
{
    switch (level & 0x03) {
    case 0: return "-9.5 dBm0";
    case 1: return "-12 dBm0";
    case 2: return "-15 dBm0";
    case 3: return "-18 dBm0";
    default: return "unknown";
    }
}

static bool v92_decode_qc2_id(const v8bis_decoded_msg_t *msg,
                              v92_qc2_id_t *out)
{
    uint8_t b;
    bool digital_modem;
    bool qca;
    bool lapm;

    if (!msg || !out)
        return false;
    /* V.92 QC2/QCA2 identification field uses V.8bis message type 1011b */
    if (msg->msg_type != 0xB || msg->info_len < 1)
        return false;

    memset(out, 0, sizeof(*out));
    b = msg->info[0];
    out->id_octet = b;
    out->revision = msg->revision & 0x0F;

    qca = ((b >> 6) & 0x01U) != 0;          /* bit 14 */
    digital_modem = ((b >> 7) & 0x01U) != 0;/* bit 15 */
    lapm = ((b >> 5) & 0x01U) != 0;         /* bit 13 */

    out->qca = qca;
    out->digital_modem = digital_modem;
    out->lapm = lapm;

    if (digital_modem) {
        int lm = (int) (b & 0x03U);         /* bits 8..9 */

        /* bits 10..12 should be 000 per V.92 Tables 12 and 14; log if set
         * but do not reject — real hardware sometimes sets reserved bits */
        out->reserved_bits_set = ((b & 0x1CU) != 0);
        out->lm = lm;
        /* QCA2x on CH1 (initiating), QC2x on CH2 (responding) */
        out->name = qca ? "QCA2d" : "QC2d";
        out->expected_channel = qca ? 0 : 1;
        out->channel_mismatch = (msg->channel != out->expected_channel);
        out->ok = true;
        return true;
    }

    out->wxyz = (int) (b & 0x0FU);          /* bits 8..11 */
    /* bit 12 should be 0 per V.92 Tables 3 and 5; log if set but do not
     * reject — V.8bis compatibility says receivers ignore unknown bits */
    out->reserved_bits_set = ((b & 0x10U) != 0);
    out->uqts_ucode = v92_uqts_from_wxyz(out->wxyz);
    /* QCA2x on CH1 (initiating), QC2x on CH2 (responding) */
    out->name = qca ? "QCA2a" : "QC2a";
    out->expected_channel = qca ? 0 : 1;
    out->channel_mismatch = (msg->channel != out->expected_channel);
    out->ok = true;
    return true;
}

static bool v92_decode_qc2_id_partial(const v8bis_partial_frame_t *partial,
                                      v92_qc2_id_t *out)
{
    v8bis_decoded_msg_t msg;

    if (!partial || !out)
        return false;
    if (partial->frame_byte_count < 2)
        return false;

    memset(&msg, 0, sizeof(msg));
    msg.msg_type = partial->frame_buf[0] & 0x0F;
    msg.revision = (partial->frame_buf[0] >> 4) & 0x0F;
    msg.info_len = partial->frame_byte_count - 1;
    if (msg.info_len > (int) sizeof(msg.info))
        msg.info_len = (int) sizeof(msg.info);
    if (msg.info_len > 0)
        memcpy(msg.info, partial->frame_buf + 1, (size_t) msg.info_len);
    msg.sample_offset = partial->sample_offset;
    msg.channel = partial->channel;
    return v92_decode_qc2_id(&msg, out);
}

static int v8bis_partial_first_octet_valid_bits(const v8bis_partial_frame_t *partial)
{
    if (!partial)
        return 0;
    if (partial->frame_byte_count > 0)
        return 8;
    if (partial->byte_bits > 0)
        return partial->byte_bits;
    return 0;
}

static const char *v8bis_partial_type_str(const v8bis_partial_frame_t *partial)
{
    if (!partial)
        return "unknown";
    if (v8bis_partial_first_octet_valid_bits(partial) < 4)
        return "unknown";
    return v8bis_msg_type_str(partial->first_octet & 0x0F);
}

/* CRC-16/CCITT (x^16 + x^12 + x^5 + 1) per ISO/IEC 3309 / V.8bis §7.2.7 */
static uint16_t v8bis_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    }
    return crc;
}

static const char *v8bis_partial_reason_str(v8bis_partial_reason_t reason)
{
    switch (reason) {
    case V8BIS_PARTIAL_SHORT_FRAME:
        return "short_frame";
    case V8BIS_PARTIAL_NON_BYTE_ALIGNED:
        return "non_byte_aligned";
    case V8BIS_PARTIAL_BAD_CRC:
        return "bad_crc";
    case V8BIS_PARTIAL_ABORT_SEQUENCE:
        return "abort_sequence";
    case V8BIS_PARTIAL_SHORT_PREAMBLE:
        return "short_preamble";
    case V8BIS_PARTIAL_SINGLE_FLAG:
        return "single_flag";
    default:
        return "unknown";
    }
}

static void v8bis_hdlc_record_partial(v8bis_hdlc_rx_t *rx,
                                      bool crc_ok,
                                      v8bis_partial_reason_t reason)
{
    v8bis_partial_frame_t *partial;
    int copy_len;

    if (!rx || rx->partial_count >= V8BIS_MSG_MAX)
        return;
    if (rx->frame_byte_count <= 0 && rx->byte_bits <= 0)
        return;

    partial = &rx->partials[rx->partial_count++];
    memset(partial, 0, sizeof(*partial));
    partial->channel = rx->channel;
    partial->frame_byte_count = rx->frame_byte_count;
    partial->byte_bits = rx->byte_bits;
    partial->crc_ok = crc_ok;
    partial->reason = reason;
    partial->sample_offset = rx->carrier_on_sample
        + (int)((double)rx->frame_start_bit / 300.0 * 8000.0);
    partial->first_octet = (rx->frame_byte_count > 0) ? rx->frame_buf[0] : rx->byte_val;
    copy_len = rx->frame_byte_count;
    if (copy_len > (int) sizeof(partial->frame_buf))
        copy_len = (int) sizeof(partial->frame_buf);
    if (copy_len > 0)
        memcpy(partial->frame_buf, rx->frame_buf, (size_t) copy_len);
}

static void v8bis_hdlc_commit_frame(v8bis_hdlc_rx_t *rx)
{
    v8bis_decoded_msg_t *msg;
    uint16_t crc;

    /* Need at least: 1 I-field byte + 2 FCS bytes = 3.
     * The closing HDLC flag (0x7E) always deposits exactly 7 bits into
     * byte_val before the flag fires (the prefix 0,1,1,1,1,1,1).
     * So byte_bits must be exactly 7 for a byte-aligned frame. */
    if (rx->frame_byte_count < 3 || rx->byte_bits != 7) {
        v8bis_hdlc_record_partial(rx,
                                  false,
                                  rx->frame_byte_count < 3 ? V8BIS_PARTIAL_SHORT_FRAME
                                                           : V8BIS_PARTIAL_NON_BYTE_ALIGNED);
        return;
    }
    if (rx->msg_count >= V8BIS_MSG_MAX)
        return;

    /* CRC check: good frame residual is 0xF0B8 */
    crc = v8bis_crc16(rx->frame_buf, rx->frame_byte_count);
    if (crc != 0xF0B8) {
        v8bis_hdlc_record_partial(rx, false, V8BIS_PARTIAL_BAD_CRC);
        return;
    }

    msg = &rx->msgs[rx->msg_count++];
    msg->msg_type = rx->frame_buf[0] & 0x0F;
    msg->revision = (rx->frame_buf[0] >> 4) & 0x0F;
    /* info = I-field bytes after type/revision byte, before 2 FCS bytes */
    msg->info_len = rx->frame_byte_count - 3;
    if (msg->info_len > 0 && msg->info_len <= (int)sizeof(msg->info))
        memcpy(msg->info, rx->frame_buf + 1, msg->info_len);
    else
        msg->info_len = 0;
    /* Sample offset: bits-from-carrier-on / 300bps * 8000Hz + carrier-on sample */
    msg->sample_offset = rx->carrier_on_sample
        + (int)((double)rx->frame_start_bit / 300.0 * 8000.0);
    msg->channel = rx->channel;
}

static void v8bis_hdlc_put_bit(void *user_data, int bit)
{
    v8bis_hdlc_rx_t *rx = (v8bis_hdlc_rx_t *) user_data;

    if (bit < 0) {
        /* SIG_STATUS_CARRIER_UP = -2: record sample position */
        if (bit == -2)
            rx->carrier_on_sample = rx->current_chunk_sample + 80; /* midpoint of 160-sample chunk */
        return;
    }

    rx->bit_count++;

    if (bit == 1) {
        rx->ones_run++;
        if (!rx->in_frame)
            rx->preamble_ones++;  /* count marking bits before the first flag */
        if (rx->ones_run >= 7) {
            /* Abort sequence (7+ consecutive ones) */
            if (rx->in_frame)
                v8bis_hdlc_record_partial(rx, false, V8BIS_PARTIAL_ABORT_SEQUENCE);
            rx->ones_run = 7;
            rx->in_frame = false;
            rx->frame_byte_count = 0;
            rx->byte_bits = 0;
            rx->byte_val = 0;
            return;
        }
        /* Fall through: real data bit */
    } else {
        /* bit == 0 */
        if (!rx->in_frame)
            rx->preamble_ones = 0;  /* a 0 bit outside a frame resets preamble run */
        if (rx->ones_run == 5) {
            /* Stuffed zero — discard per §7.2.8 */
            rx->ones_run = 0;
            return;
        }
        if (rx->ones_run == 6) {
            /* Flag byte (0x7E): 6 ones + 0 */
            if (!rx->in_frame) {
                /* First opening flag: snapshot preamble, enter frame context */
                rx->frame_preamble_ones = rx->preamble_ones;
                rx->in_frame = true;
                rx->flag_seen = true;
                rx->frame_byte_count = 0;
                rx->byte_bits = 0;
                rx->byte_val = 0;
                rx->opening_flag_count = 1;
            } else if (rx->frame_byte_count == 0 && rx->byte_bits == 0) {
                /* Additional opening flag (no data bytes received yet) */
                rx->opening_flag_count++;
            } else {
                /* Closing flag: data was present, commit the frame */
                v8bis_hdlc_commit_frame(rx);
                rx->in_frame = true;
                rx->flag_seen = true;
                rx->frame_byte_count = 0;
                rx->byte_bits = 0;
                rx->byte_val = 0;
                rx->opening_flag_count = 1;
                rx->frame_preamble_ones = 30; /* subsequent frames share same carrier burst */
            }
            rx->ones_run = 0;
            rx->frame_start_bit = rx->bit_count;
            return;
        }
        rx->ones_run = 0;
        /* Fall through: real data bit */
    }

    if (!rx->in_frame)
        return;

    /* Accumulate byte (LSB first per §7.2.2) */
    rx->byte_val |= (uint8_t)(bit << rx->byte_bits);
    if (++rx->byte_bits == 8) {
        if (rx->frame_byte_count < (int)sizeof(rx->frame_buf))
            rx->frame_buf[rx->frame_byte_count++] = rx->byte_val;
        rx->byte_val = 0;
        rx->byte_bits = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Event collection functions                                         */
/* ------------------------------------------------------------------ */

void v8bis_collect_msg_events(call_log_t *log,
                              const int16_t *samples,
                              int total_samples,
                              int max_sample)
{
    fsk_rx_state_t *fsk_ch1;
    fsk_rx_state_t *fsk_ch2;
    v8bis_hdlc_rx_t rx_ch1;
    v8bis_hdlc_rx_t rx_ch2;
    /* Flat struct arrays — copies are safe across per-region rx_ch1/rx_ch2 resets */
    v8bis_decoded_msg_t all_msgs[V8BIS_MSG_MAX * 2];
    int all_count;
    v8bis_partial_frame_t all_partials[V8BIS_MSG_MAX * 2];
    int all_partial_count;
    int limit;
    char summary[160];
    char detail[1024];
    int last_partial_sample[16];
    uint8_t last_partial_type[16];
    int partial_emitted_count = 0;

    if (!log || !samples || total_samples <= 0)
        return;

    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;

    /* Energy-based region finding: scan for active signal before running FSK.
     * V.8bis HDLC messages ride on a V.21 FSK carrier that produces clear
     * energy.  Silence and digital noise between signals can be skipped.
     * Use per-chunk energy to build active regions; run a fresh FSK pair for
     * each region so skipped gaps don't corrupt the filter state.
     *
     * Threshold: ~-50 dBm in 16-bit PCM (mean-squared ≈ 500). */
    enum {
        V8BIS_ENERGY_CHUNK   = 160,     /* 20 ms at 8000 Hz */
        V8BIS_ENERGY_GAP_MAX = 20,      /* tolerate up to 400ms of low-energy gap */
        V8BIS_ENERGY_PAD     = 320,     /* 40ms padding before each region start */
    };
    static const double V8BIS_ENERGY_THRESHOLD = 500.0;

    typedef struct { int start; int end; } v8bis_region_t;
    v8bis_region_t regions[32];
    int n_regions = 0;
    {
        int region_start = -1;
        int gap_chunks = 0;

        for (int s = 0; s + V8BIS_ENERGY_CHUNK <= limit; s += V8BIS_ENERGY_CHUNK) {
            double e = window_energy(samples + s, V8BIS_ENERGY_CHUNK);
            bool active = (e >= V8BIS_ENERGY_THRESHOLD);

            if (active) {
                if (region_start < 0) {
                    int padded = s - V8BIS_ENERGY_PAD;
                    region_start = (padded > 0) ? padded : 0;
                }
                gap_chunks = 0;
            } else if (region_start >= 0) {
                if (++gap_chunks > V8BIS_ENERGY_GAP_MAX) {
                    if (n_regions < 32) {
                        regions[n_regions].start = region_start;
                        regions[n_regions].end   = s;
                        n_regions++;
                    }
                    region_start = -1;
                    gap_chunks   = 0;
                }
            }
        }
        /* Close any open region at the end */
        if (region_start >= 0 && n_regions < 32) {
            regions[n_regions].start = region_start;
            regions[n_regions].end   = limit;
            n_regions++;
        }
    }
    /* If no energy above threshold, fall back to scanning the whole limit
     * (handles all-silent captures or captures where threshold is too high). */
    if (n_regions == 0) {
        regions[0].start = 0;
        regions[0].end   = limit;
        n_regions = 1;
    }

    /* Run a fresh pair of FSK decoders over each active region.  Starting
     * fresh is safe because every V.8bis frame begins with ≥100ms of marking
     * preamble that re-settles the demodulator before any flags arrive. */
    all_count = 0;
    all_partial_count = 0;
    for (int ri = 0; ri < n_regions; ri++) {
        int rstart = regions[ri].start;
        int rend   = regions[ri].end;

        memset(&rx_ch1, 0, sizeof(rx_ch1));
        rx_ch1.channel = 0;  /* initiating / V.21 CH1 */
        memset(&rx_ch2, 0, sizeof(rx_ch2));
        rx_ch2.channel = 1;  /* responding / V.21 CH2 */

        /* V.21 channel 1 (initiating): Fmark=1080, Fspace=1180 Hz
         * V.21 channel 2 (responding): Fmark=1750, Fspace=1850 Hz */
        fsk_ch1 = fsk_rx_init(NULL, &preset_fsk_specs[FSK_V21CH1],
                               FSK_FRAME_MODE_ASYNC, v8bis_hdlc_put_bit, &rx_ch1);
        fsk_ch2 = fsk_rx_init(NULL, &preset_fsk_specs[FSK_V21CH2],
                               FSK_FRAME_MODE_ASYNC, v8bis_hdlc_put_bit, &rx_ch2);
        if (!fsk_ch1 || !fsk_ch2) {
            if (fsk_ch1) fsk_rx_free(fsk_ch1);
            if (fsk_ch2) fsk_rx_free(fsk_ch2);
            continue;
        }
        /* Lower detection threshold — V.8bis messages can be weak */
        fsk_rx_set_signal_cutoff(fsk_ch1, -45.0f);
        fsk_rx_set_signal_cutoff(fsk_ch2, -45.0f);

        /* Feed this region in 160-sample chunks; current_chunk_sample is the
         * absolute sample offset so carrier_on_sample positions are correct. */
        for (int offset = rstart; offset < rend; ) {
            int len = rend - offset;
            if (len > 160) len = 160;
            rx_ch1.current_chunk_sample = offset;
            rx_ch2.current_chunk_sample = offset;
            fsk_rx(fsk_ch1, samples + offset, len);
            fsk_rx(fsk_ch2, samples + offset, len);
            offset += len;
        }
        fsk_rx_free(fsk_ch1);
        fsk_rx_free(fsk_ch2);

        /* Accumulate messages and partials into combined flat arrays (struct copy) */
        for (int i = 0; i < rx_ch1.msg_count && all_count < V8BIS_MSG_MAX * 2; i++)
            all_msgs[all_count++] = rx_ch1.msgs[i];
        for (int i = 0; i < rx_ch2.msg_count && all_count < V8BIS_MSG_MAX * 2; i++)
            all_msgs[all_count++] = rx_ch2.msgs[i];
        for (int i = 0; i < rx_ch1.partial_count && all_partial_count < V8BIS_MSG_MAX * 2; i++)
            all_partials[all_partial_count++] = rx_ch1.partials[i];
        for (int i = 0; i < rx_ch2.partial_count && all_partial_count < V8BIS_MSG_MAX * 2; i++)
            all_partials[all_partial_count++] = rx_ch2.partials[i];
    }

    /* Sort all collected messages by sample offset */
    for (int i = 0; i < all_count - 1; i++) {
        for (int j = i + 1; j < all_count; j++) {
            if (all_msgs[j].sample_offset < all_msgs[i].sample_offset) {
                v8bis_decoded_msg_t tmp = all_msgs[i];
                all_msgs[i] = all_msgs[j];
                all_msgs[j] = tmp;
            }
        }
    }

    /* Dedup: skip a message if an identical (type, channel) pair was already
     * emitted within 400ms.  Crosstalk causes both stereo channels to decode
     * the same FSK frame, so same-type same-channel within 400ms is a dup.
     * QC2 and QCA2 both use msg_type 0xB but arrive on different V.21 channels
     * (QC2x on CH2, QCA2x on CH1) — they must NOT be deduped against each
     * other, so the dedup key includes the channel. */
    int last_emitted_sample[16];
    uint8_t last_emitted_key[16];   /* msg_type | (channel << 4) */
    int emitted_count = 0;
    memset(last_emitted_sample, -1, sizeof(last_emitted_sample));
    memset(last_partial_sample, -1, sizeof(last_partial_sample));

    for (int i = 0; i < all_count; i++) {
        const v8bis_decoded_msg_t *msg = &all_msgs[i];
        const char *type_str = v8bis_msg_type_str(msg->msg_type);
        const char *ch_str = (msg->channel == 0) ? "initiating/CH1" : "responding/CH2";
        uint8_t dedup_key = msg->msg_type | (uint8_t)(msg->channel << 4);
        v92_qc2_id_t v92_qc2;

        /* Skip frames with unknown/undefined type */
        if (msg->msg_type == 0 || strcmp(type_str, "unknown") == 0)
            continue;

        /* Dedup: skip if same (type, channel) key seen within ~400ms (3200 samples) */
        bool is_dup = false;
        for (int k = 0; k < emitted_count; k++) {
            if (last_emitted_key[k] == dedup_key
                && abs(msg->sample_offset - last_emitted_sample[k]) < 3200) {
                is_dup = true;
                break;
            }
        }
        if (is_dup)
            continue;
        if (emitted_count < 16) {
            last_emitted_key[emitted_count] = dedup_key;
            last_emitted_sample[emitted_count] = msg->sample_offset;
            emitted_count++;
        }

        snprintf(summary, sizeof(summary), "%s", type_str);

        {
            /* Build detail string: header + decoded I/S/NS fields (MS/CL/CLR)
             * or raw hex (ACK/NAK and any other type with payload). */
            int dpos = 0;
            dpos += snprintf(detail + dpos, sizeof(detail) - dpos,
                             "fsk_ch=%s rev=%d", ch_str, msg->revision);
            if (msg->info_len > 0 &&
                (msg->msg_type == 0x1 || msg->msg_type == 0x2 || msg->msg_type == 0x3)) {
                dpos += snprintf(detail + dpos, sizeof(detail) - dpos, " ");
                dpos += v8bis_decode_info_field(msg->info, msg->info_len,
                                                 detail + dpos,
                                                 (int)sizeof(detail) - dpos);
            } else if (msg->info_len > 0) {
                int hlen = (msg->info_len < 16) ? msg->info_len : 16;
                dpos += snprintf(detail + dpos, sizeof(detail) - dpos, " raw=[");
                for (int j = 0; j < hlen; j++)
                    dpos += snprintf(detail + dpos, sizeof(detail) - dpos,
                                     "%s%02X", j == 0 ? "" : "-", msg->info[j]);
                dpos += snprintf(detail + dpos, sizeof(detail) - dpos, "]");
            }
        }

        call_log_append(log, msg->sample_offset, 0, "V.8bis", summary, detail);

        /* Also emit explicit V.92 QC2/QCA2 identification when present. */
        if (v92_decode_qc2_id(msg, &v92_qc2)) {
            snprintf(summary, sizeof(summary), "%s", v92_qc2.name);
            if (v92_qc2.digital_modem) {
                snprintf(detail, sizeof(detail),
                         "source=V.8bis id_field rev=%d fsk_ch=%s lapm=%s lm=%d anspcm_level=%s id_octet=%02X%s%s",
                         v92_qc2.revision,
                         ch_str,
                         v92_qc2.lapm ? "yes" : "no",
                         v92_qc2.lm,
                         v92_anspcm_level_to_str(v92_qc2.lm),
                         v92_qc2.id_octet,
                         v92_qc2.reserved_bits_set ? " reserved_bits=set" : "",
                         v92_qc2.channel_mismatch  ? " channel_mismatch=yes" : "");
            } else {
                snprintf(detail, sizeof(detail),
                         "source=V.8bis id_field rev=%d fsk_ch=%s lapm=%s uqts_index=0x%X uqts_ucode=%d id_octet=%02X%s%s",
                         v92_qc2.revision,
                         ch_str,
                         v92_qc2.lapm ? "yes" : "no",
                         v92_qc2.wxyz,
                         v92_qc2.uqts_ucode,
                         v92_qc2.id_octet,
                         v92_qc2.reserved_bits_set ? " reserved_bits=set" : "",
                         v92_qc2.channel_mismatch  ? " channel_mismatch=yes" : "");
            }
            call_log_append(log, msg->sample_offset, 0, "V.92", summary, detail);
        }
    }

    {
        for (int i = 0; i < all_partial_count; i++) {
            const v8bis_partial_frame_t *partial = &all_partials[i];
            const char *ch_str = partial->channel == 0 ? "CH1/initiating" : "CH2/responding";
            const char *type_str = v8bis_partial_type_str(partial);
            char hex[80];
            char rev_buf[8];
            int hex_len = partial->frame_byte_count;
            int pos = 0;
            bool is_dup = false;
            v92_qc2_id_t v92_qc2;
            int first_octet_bits = v8bis_partial_first_octet_valid_bits(partial);

            if (hex_len > 6)
                hex_len = 6;
            if (first_octet_bits >= 8)
                snprintf(rev_buf, sizeof(rev_buf), "%d", (partial->first_octet >> 4) & 0x0F);
            else
                snprintf(rev_buf, sizeof(rev_buf), "?");
            hex[0] = '\0';
            for (int j = 0; j < hex_len; j++)
                pos += snprintf(hex + pos, sizeof(hex) - (size_t) pos, "%s%02X", j == 0 ? "" : " ", partial->frame_buf[j]);

            {
                uint8_t partial_key = (partial->first_octet & 0x0F)
                                    | (uint8_t)(partial->channel << 4);
                for (int k = 0; k < partial_emitted_count; k++) {
                    if (last_partial_type[k] == partial_key
                        && abs(partial->sample_offset - last_partial_sample[k]) < 3200) {
                        is_dup = true;
                        break;
                    }
                }
                if (!is_dup && partial_emitted_count < 16) {
                    last_partial_type[partial_emitted_count] = partial_key;
                    last_partial_sample[partial_emitted_count] = partial->sample_offset;
                    partial_emitted_count++;
                }
            }

            snprintf(summary, sizeof(summary), "Partial %s frame", type_str);
            snprintf(detail, sizeof(detail),
                     "fsk_ch=%s first_octet_bits=%d rev=%s bytes=%d trailing_bits=%d crc=%s reason=%s raw=%s",
                     ch_str,
                     first_octet_bits,
                     rev_buf,
                     partial->frame_byte_count,
                     partial->byte_bits,
                     partial->crc_ok ? "ok" : "failed",
                     v8bis_partial_reason_str(partial->reason),
                     hex[0] != '\0' ? hex : "n/a");
            call_log_append(log, partial->sample_offset, 0, "V.8bis?", summary, detail);

            if (v92_decode_qc2_id_partial(partial, &v92_qc2)) {
                snprintf(summary, sizeof(summary), "Partial %s", v92_qc2.name);
                if (v92_qc2.digital_modem) {
                    snprintf(detail, sizeof(detail),
                             "source=V.8bis partial_id_field rev=%d fsk_ch=%s lapm=%s lm=%d anspcm_level=%s id_octet=%02X crc=%s reason=%s",
                             v92_qc2.revision,
                             ch_str,
                             v92_qc2.lapm ? "yes" : "no",
                             v92_qc2.lm,
                             v92_anspcm_level_to_str(v92_qc2.lm),
                             v92_qc2.id_octet,
                             partial->crc_ok ? "ok" : "failed",
                             v8bis_partial_reason_str(partial->reason));
                } else {
                    snprintf(detail, sizeof(detail),
                             "source=V.8bis partial_id_field rev=%d fsk_ch=%s lapm=%s uqts_index=0x%X uqts_ucode=%d id_octet=%02X crc=%s reason=%s",
                             v92_qc2.revision,
                             ch_str,
                             v92_qc2.lapm ? "yes" : "no",
                             v92_qc2.wxyz,
                             v92_qc2.uqts_ucode,
                             v92_qc2.id_octet,
                             partial->crc_ok ? "ok" : "failed",
                             v8bis_partial_reason_str(partial->reason));
                }
                call_log_append(log, partial->sample_offset, 0, "V.92?", summary, detail);
            }
        }
    }
}

void v8bis_collect_signal_events(call_log_t *log,
                                 const int16_t *samples,
                                 int total_samples,
                                 int max_sample)
{
    v8bis_scan_result_t v8bis;
    v8bis_weak_candidate_t weak_v8bis;
    char summary[160];
    char detail[320];

    if (!log || !samples || total_samples <= 0)
        return;
    if (!v8bis_scan_signals(samples, total_samples, max_sample, &v8bis)) {
        if (!v8bis_scan_weak_candidate(samples, total_samples, max_sample, &weak_v8bis))
            return;
        snprintf(summary, sizeof(summary), "Weak %s-like energy", g_v8bis_signal_defs[weak_v8bis.signal_index].name);
        snprintf(detail, sizeof(detail),
                 "role=%s seg1=%d+%dHz seg2=%dHz seg1_ms=400 seg2_ms=100 seg1_a=%.1f%% seg1_b=%.1f%% dual=%.1f%% balance=%.1f%% seg2_strength=%.1f%% score=%.0f weak=yes",
                 g_v8bis_signal_defs[weak_v8bis.signal_index].role,
                 g_v8bis_signal_defs[weak_v8bis.signal_index].seg1_a_hz,
                 g_v8bis_signal_defs[weak_v8bis.signal_index].seg1_b_hz,
                 g_v8bis_signal_defs[weak_v8bis.signal_index].seg2_hz,
                 weak_v8bis.seg1_a_ratio * 100.0,
                 weak_v8bis.seg1_b_ratio * 100.0,
                 weak_v8bis.seg1_dual_ratio * 100.0,
                 weak_v8bis.seg1_balance * 100.0,
                 weak_v8bis.seg2_ratio * 100.0,
                 weak_v8bis.score);
        call_log_append(log,
                        weak_v8bis.sample_offset,
                        weak_v8bis.duration_samples,
                        "V.8bis?",
                        summary,
                        detail);
        return;
    }

    static const char *v8bis_descriptions[] = {
        "Mode Request establishing (auto-answer)",
        "Capabilities Request establishing (auto-answer)",
        "Escape Signal initiating",
        "Mode Request detected (initiating/calling-side)",
        "Capabilities Request detected (initiating/calling-side)",
        "Mode Request detected (responding)",
        "Capabilities Request detected (responding)",
        "Escape Signal responding"
    };

    for (int i = 0; i < V8BIS_NUM_SIGNALS; i++) {
        if (!v8bis.hits[i].seen)
            continue;
        snprintf(summary, sizeof(summary), "%s — %s",
                 g_v8bis_signal_defs[i].name,
                 i < (int)(sizeof(v8bis_descriptions)/sizeof(v8bis_descriptions[0]))
                     ? v8bis_descriptions[i] : "");
        snprintf(detail, sizeof(detail),
                 "role=%s seg1=%d+%dHz seg2=%dHz seg1_ms=400 seg2_ms=100 seg1_a=%.1f%% seg1_b=%.1f%% dual=%.1f%% balance=%.1f%% seg2_strength=%.1f%% score=%.0f",
                 g_v8bis_signal_defs[i].role,
                 g_v8bis_signal_defs[i].seg1_a_hz,
                 g_v8bis_signal_defs[i].seg1_b_hz,
                 g_v8bis_signal_defs[i].seg2_hz,
                 v8bis.hits[i].seg1_a_ratio * 100.0,
                 v8bis.hits[i].seg1_b_ratio * 100.0,
                 v8bis.hits[i].seg1_dual_ratio * 100.0,
                 v8bis.hits[i].seg1_balance * 100.0,
                 v8bis.hits[i].seg2_ratio * 100.0,
                 v8bis.hits[i].score);
        call_log_append(log,
                        v8bis.hits[i].sample_offset,
                        v8bis.hits[i].duration_samples,
                        "V.8bis",
                        summary,
                        detail);
    }
}

/* ------------------------------------------------------------------ */
/* Deduplication                                                      */
/* ------------------------------------------------------------------ */

void v8bis_dedup_msgs(call_log_t *log)
{
    size_t out = 0;

    if (!log || log->count == 0)
        return;

    /* For each V.8bis FSK event (not tone events which have duration>0),
     * check if a prior event with the same summary is within 8000 samples. */
    for (size_t i = 0; i < log->count; i++) {
        const call_log_event_t *event = &log->events[i];
        bool is_dup = false;

        if (strcmp(event->protocol, "V.8bis") == 0 && event->duration_samples == 0) {
            for (size_t j = 0; j < out; j++) {
                const call_log_event_t *prev = &log->events[j];
                if (strcmp(prev->protocol, "V.8bis") == 0
                    && prev->duration_samples == 0
                    && strcmp(prev->summary, event->summary) == 0
                    && abs(event->sample_offset - prev->sample_offset) < 8000) {
                    is_dup = true;
                    break;
                }
            }
        }
        if (!is_dup) {
            if (out != i)
                log->events[out] = log->events[i];
            out++;
        }
    }
    log->count = out;
}
