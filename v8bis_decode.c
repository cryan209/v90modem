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

/* V.8bis SPar(1) mode capabilities (§8.4, Table 6-2) */
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

/* HDLC receiver state for one V.21 channel */
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
    /* Sample position tracking */
    int current_chunk_sample;
    int carrier_on_sample;
    /* Results */
    v8bis_decoded_msg_t msgs[V8BIS_MSG_MAX];
    int msg_count;
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

        /* bits 10..12 reserved=000 for QC2d/QCA2d */
        if ((b & 0x1CU) != 0)
            return false;
        out->lm = lm;
        out->name = qca ? "QCA2d" : "QC2d";
        out->ok = true;
        return true;
    }

    out->wxyz = (int) (b & 0x0FU);          /* bits 8..11 */
    /* bit 12 reserved=0 for QC2a/QCA2a */
    if ((b & 0x10U) != 0)
        return false;
    out->uqts_ucode = v92_uqts_from_wxyz(out->wxyz);
    out->name = qca ? "QCA2a" : "QC2a";
    out->ok = true;
    return true;
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

static void v8bis_hdlc_commit_frame(v8bis_hdlc_rx_t *rx)
{
    v8bis_decoded_msg_t *msg;

    /* Need at least: 1 I-field byte + 2 FCS bytes = 3.
     * The closing HDLC flag (0x7E) always deposits exactly 7 bits into
     * byte_val before the flag fires (the prefix 0,1,1,1,1,1,1).
     * So byte_bits must be exactly 7 for a byte-aligned frame. */
    if (rx->frame_byte_count < 3 || rx->byte_bits != 7)
        return;
    if (rx->msg_count >= V8BIS_MSG_MAX)
        return;

    /* CRC check: good frame residual is 0xF0B8 */
    if (v8bis_crc16(rx->frame_buf, rx->frame_byte_count) != 0xF0B8)
        return;

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
            rx->carrier_on_sample = rx->current_chunk_sample;
        return;
    }

    rx->bit_count++;

    if (bit == 1) {
        rx->ones_run++;
        if (rx->ones_run >= 7) {
            /* Abort sequence (7+ consecutive ones) */
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
        if (rx->ones_run == 5) {
            /* Stuffed zero — discard per §7.2.8 */
            rx->ones_run = 0;
            return;
        }
        if (rx->ones_run == 6) {
            /* Flag byte (0x7E): 6 ones + 0 */
            if (rx->in_frame)
                v8bis_hdlc_commit_frame(rx);
            rx->in_frame = true;
            rx->flag_seen = true;
            rx->frame_byte_count = 0;
            rx->byte_bits = 0;
            rx->byte_val = 0;
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
    v8bis_decoded_msg_t *all_msgs[V8BIS_MSG_MAX * 2];
    int all_count;
    int limit;
    char summary[160];
    char detail[320];

    if (!log || !samples || total_samples <= 0)
        return;

    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;

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
        return;
    }
    /* Lower detection threshold — V.8bis messages can be weak, and CRe/MRe are
       specified at -12 to -15dB below normal per V.8bis §7.1.4 */
    fsk_rx_set_signal_cutoff(fsk_ch1, -45.0f);
    fsk_rx_set_signal_cutoff(fsk_ch2, -45.0f);

    /* Feed samples in 160-sample chunks; update current_chunk_sample before
     * each call so SIG_STATUS_CARRIER_UP records an accurate sample position */
    for (int offset = 0; offset < limit; ) {
        int len = limit - offset;
        if (len > 160) len = 160;
        rx_ch1.current_chunk_sample = offset;
        rx_ch2.current_chunk_sample = offset;
        fsk_rx(fsk_ch1, samples + offset, len);
        fsk_rx(fsk_ch2, samples + offset, len);
        offset += len;
    }

    fsk_rx_free(fsk_ch1);
    fsk_rx_free(fsk_ch2);

    /* Collect and sort all valid messages by sample offset */
    all_count = 0;
    for (int i = 0; i < rx_ch1.msg_count; i++)
        all_msgs[all_count++] = &rx_ch1.msgs[i];
    for (int i = 0; i < rx_ch2.msg_count; i++)
        all_msgs[all_count++] = &rx_ch2.msgs[i];
    for (int i = 0; i < all_count - 1; i++) {
        for (int j = i + 1; j < all_count; j++) {
            if (all_msgs[j]->sample_offset < all_msgs[i]->sample_offset) {
                v8bis_decoded_msg_t *tmp = all_msgs[i];
                all_msgs[i] = all_msgs[j];
                all_msgs[j] = tmp;
            }
        }
    }

    /* Dedup: skip a message if an identical type was already emitted within 400ms
     * (crosstalk causes both stereo channels to decode the same FSK frame) */
    int last_emitted_sample[16];
    uint8_t last_emitted_type[16];
    int emitted_count = 0;
    memset(last_emitted_sample, -1, sizeof(last_emitted_sample));

    for (int i = 0; i < all_count; i++) {
        const v8bis_decoded_msg_t *msg = all_msgs[i];
        const char *type_str = v8bis_msg_type_str(msg->msg_type);
        const char *ch_str = (msg->channel == 0) ? "initiating/CH1" : "responding/CH2";
        v92_qc2_id_t v92_qc2;

        /* Skip frames with unknown/undefined type */
        if (msg->msg_type == 0 || strcmp(type_str, "unknown") == 0)
            continue;

        /* Dedup: skip if same type decoded within ~400ms (3200 samples) */
        bool is_dup = false;
        for (int k = 0; k < emitted_count; k++) {
            if (last_emitted_type[k] == msg->msg_type
                && abs(msg->sample_offset - last_emitted_sample[k]) < 3200) {
                is_dup = true;
                break;
            }
        }
        if (is_dup)
            continue;
        if (emitted_count < 16) {
            last_emitted_type[emitted_count] = msg->msg_type;
            last_emitted_sample[emitted_count] = msg->sample_offset;
            emitted_count++;
        }

        snprintf(summary, sizeof(summary), "%s", type_str);

        if (msg->info_len > 0) {
            /* Show capability byte summary for MS/CL/CLR */
            char hex[100];
            int hlen = (msg->info_len < 12) ? msg->info_len : 12;
            int pos = 0;
            for (int j = 0; j < hlen; j++)
                pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", msg->info[j]);
            /* SPar(1) is at info[0] if present — decode mode per §8.4 */
            if (msg->info_len >= 1 && (msg->msg_type == 0x1 || msg->msg_type == 0x2 || msg->msg_type == 0x3)) {
                int spar1 = msg->info[0] & 0x7F;
                snprintf(detail, sizeof(detail),
                         "fsk_ch=%s rev=%d mode=%s info=[%s]",
                         ch_str, msg->revision,
                         v8bis_spar1_mode_str(spar1), hex);
            } else {
                snprintf(detail, sizeof(detail),
                         "fsk_ch=%s rev=%d info=[%s]", ch_str, msg->revision, hex);
            }
        } else {
            snprintf(detail, sizeof(detail), "fsk_ch=%s rev=%d", ch_str, msg->revision);
        }

        call_log_append(log, msg->sample_offset, 0, "V.8bis", summary, detail);

        /* Also emit explicit V.92 QC2/QCA2 identification when present. */
        if (v92_decode_qc2_id(msg, &v92_qc2)) {
            snprintf(summary, sizeof(summary), "%s", v92_qc2.name);
            if (v92_qc2.digital_modem) {
                snprintf(detail, sizeof(detail),
                         "source=V.8bis id_field rev=%d fsk_ch=%s lapm=%s anspcm_level=%s id_octet=%02X",
                         v92_qc2.revision,
                         ch_str,
                         v92_qc2.lapm ? "yes" : "no",
                         v92_anspcm_level_to_str(v92_qc2.lm),
                         v92_qc2.id_octet);
            } else {
                snprintf(detail, sizeof(detail),
                         "source=V.8bis id_field rev=%d fsk_ch=%s lapm=%s uqts_index=0x%X uqts_ucode=%d id_octet=%02X",
                         v92_qc2.revision,
                         ch_str,
                         v92_qc2.lapm ? "yes" : "no",
                         v92_qc2.wxyz,
                         v92_qc2.uqts_ucode,
                         v92_qc2.id_octet);
            }
            call_log_append(log, msg->sample_offset, 0, "V.92", summary, detail);
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
