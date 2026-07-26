/*
 * vpcm_encode.c — Synthetic V.90 downstream truth-capture generator
 *
 * Drives the v90.c digital-modem transmitter through a complete strict
 * training and data-mode flow, injecting the receive events a cooperating
 * analogue modem would cause, and writes the resulting DS0 codeword
 * stream as a WAV (stereo s16le 8 kHz, downstream on the left channel)
 * suitable for feeding to vpcm_decode.
 *
 * Because the stream comes from the engine's own encoder, every phase
 * boundary and negotiated parameter is known ground truth; the tool
 * prints them so decoder output can be diffed against them.
 *
 * Usage:
 *   vpcm_encode [--law ulaw|alaw] [--v92-compat] [--data-frames N]
 *               [--out capture.wav]
 */

#include "v90.h"
#include "v91.h"
#include "vpcm_cp.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#define ENC_MAX_CODEWORDS   (8000 * 120)   /* 2 minutes is plenty */
#define ENC_PHASE_SPIN_CAP  200000

typedef struct {
    uint8_t *codewords;
    int count;
    v91_law_t law;
} enc_stream_t;

static void enc_mark(const enc_stream_t *st, const char *label, const char *detail)
{
    printf("  [%7.1f ms] %-24s %s\n",
           (double) st->count * 1000.0 / 8000.0, label, detail ? detail : "");
}

static bool enc_append(enc_stream_t *st, uint8_t codeword)
{
    if (st->count >= ENC_MAX_CODEWORDS)
        return false;
    st->codewords[st->count++] = codeword;
    return true;
}

static bool enc_pump(enc_stream_t *st, v90_state_t *tx, int symbols)
{
    for (int i = 0; i < symbols; i++) {
        uint8_t codeword;

        if (v90_phase3_tx_codewords(tx, &codeword, 1) != 1)
            return false;
        if (!enc_append(st, codeword))
            return false;
    }
    return true;
}

static bool enc_pump_until_phase(enc_stream_t *st, v90_state_t *tx, v90_tx_phase_t target)
{
    int spin = 0;

    while (v90_get_tx_phase(tx) != target && spin++ < ENC_PHASE_SPIN_CAP) {
        if (!enc_pump(st, tx, 1))
            return false;
    }
    return v90_get_tx_phase(tx) == target;
}

/* Clean-line DIL profile (mirrors the loopback test's default Ja profile). */
static void enc_init_dil_profile(v90_dil_desc_t *desc)
{
    static const uint8_t clean_training_offsets[8] = {2, 4, 6, 8, 10, 12, 14, 15};
    static const uint16_t clean_sp_bits = 0x0A6DU;
    static const uint16_t clean_tp_bits = 0x0DB7U;

    memset(desc, 0, sizeof(*desc));
    desc->n = 125;
    desc->lsp = 12;
    desc->ltp = 12;
    for (int i = 0; i < 12; i++) {
        desc->sp[i] = (uint8_t) ((clean_sp_bits >> i) & 1U);
        desc->tp[i] = (uint8_t) ((clean_tp_bits >> i) & 1U);
    }
    for (int i = 0; i < 8; i++) {
        desc->h[i] = 1;
        desc->ref[i] = (uint8_t) ((i << 4) | 1);
    }
    for (int i = 0; i < desc->n; i++) {
        int uchord = i % 8;
        int variant = (i / 8) % 8;

        desc->train_u[i] = (uint8_t) ((uchord << 4) | clean_training_offsets[variant]);
    }
}

static void enc_init_cpt(vpcm_cp_frame_t *cp, v91_law_t law)
{
    vpcm_cp_init(cp);
    cp->v90_compatibility = false;      /* Table 14 bit 19: CPt */
    cp->drn = 9;                        /* 38666 bit/s */
    cp->codec_alaw = (law == V91_LAW_ALAW);
    cp->upstream_rate_mask = 0x1FFF;
    cp->constellation_count = 2;
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++)
        cp->dfi[i] = (uint8_t) (i & 1);
    vpcm_cp_enable_odd_ucodes(cp->masks[0]);
    vpcm_cp_enable_all_ucodes(cp->masks[1]);
}

/*
 * Raw G.711 output is the true DS0 capture: V.90 signalling depends on
 * signed zeros (+0 = 0xFF, -0 = 0x7F in µ-law), which collapse to the same
 * 16-bit sample in a linear WAV and destroy the sign stream.  Prefer .g711
 * for decoder-facing truth captures; use .wav only for listening.
 */
static bool enc_write_g711(const char *path, const enc_stream_t *st)
{
    FILE *f = fopen(path, "wb");

    if (!f)
        return false;
    if (fwrite(st->codewords, 1, (size_t) st->count, f) != (size_t) st->count) {
        fclose(f);
        return false;
    }
    fclose(f);
    return true;
}

static bool enc_write_wav(const char *path, const enc_stream_t *st)
{
    FILE *f = fopen(path, "wb");
    uint32_t data_bytes = (uint32_t) st->count * 4;  /* stereo s16le */
    uint8_t header[44];

    if (!f)
        return false;

    memcpy(header, "RIFF", 4);
    uint32_t riff_len = 36 + data_bytes;
    header[4] = (uint8_t) riff_len;
    header[5] = (uint8_t) (riff_len >> 8);
    header[6] = (uint8_t) (riff_len >> 16);
    header[7] = (uint8_t) (riff_len >> 24);
    memcpy(header + 8, "WAVEfmt ", 8);
    header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
    header[20] = 1; header[21] = 0;                   /* PCM */
    header[22] = 2; header[23] = 0;                   /* stereo */
    header[24] = 0x40; header[25] = 0x1F;             /* 8000 Hz */
    header[26] = 0; header[27] = 0;
    uint32_t byte_rate = 8000 * 4;
    header[28] = (uint8_t) byte_rate;
    header[29] = (uint8_t) (byte_rate >> 8);
    header[30] = (uint8_t) (byte_rate >> 16);
    header[31] = (uint8_t) (byte_rate >> 24);
    header[32] = 4; header[33] = 0;                   /* block align */
    header[34] = 16; header[35] = 0;                  /* bits/sample */
    memcpy(header + 36, "data", 4);
    header[40] = (uint8_t) data_bytes;
    header[41] = (uint8_t) (data_bytes >> 8);
    header[42] = (uint8_t) (data_bytes >> 16);
    header[43] = (uint8_t) (data_bytes >> 24);
    if (fwrite(header, 1, sizeof(header), f) != sizeof(header)) {
        fclose(f);
        return false;
    }

    for (int i = 0; i < st->count; i++) {
        int16_t left = v91_codeword_to_linear(st->law, st->codewords[i]);
        int16_t frame[2] = { left, 0 };

        if (fwrite(frame, sizeof(int16_t), 2, f) != 2) {
            fclose(f);
            return false;
        }
    }
    fclose(f);
    return true;
}

int main(int argc, char **argv)
{
    v91_law_t law = V91_LAW_ULAW;
    v90_law_t v90_law;
    const char *out_path = "vpcm_synth.wav";
    bool v92_compat = false;
    int data_frames = 1000;              /* 6 codewords per frame */
    enc_stream_t st;
    v90_state_t *tx = NULL;
    v90_dil_desc_t dil;
    vpcm_cp_frame_t cpt;
    vpcm_cp_frame_t data_cp;
    vpcm_cp_frame_t cp_prime;
    int ret = 1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--law") == 0 && i + 1 < argc) {
            i++;
            if (strcasecmp(argv[i], "alaw") == 0)
                law = V91_LAW_ALAW;
            else if (strcasecmp(argv[i], "ulaw") == 0)
                law = V91_LAW_ULAW;
            else {
                fprintf(stderr, "unknown law '%s'\n", argv[i]);
                return 1;
            }
        } else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (strcmp(argv[i], "--v92-compat") == 0) {
            v92_compat = true;
        } else if (strcmp(argv[i], "--data-frames") == 0 && i + 1 < argc) {
            data_frames = atoi(argv[++i]);
        } else {
            fprintf(stderr,
                    "usage: %s [--law ulaw|alaw] [--v92-compat] [--data-frames N] [--out file.wav]\n",
                    argv[0]);
            return 1;
        }
    }

    v90_law = (law == V91_LAW_ALAW) ? V90_LAW_ALAW : V90_LAW_ULAW;
    st.codewords = malloc(ENC_MAX_CODEWORDS);
    st.count = 0;
    st.law = law;
    if (!st.codewords) {
        fprintf(stderr, "allocation failed\n");
        return 1;
    }

    tx = v90_init_data_pump(v90_law);
    if (!tx) {
        fprintf(stderr, "v90_init_data_pump failed\n");
        goto done;
    }
    if (v92_compat)
        v90_enable_v92_mode(tx);

    enc_init_dil_profile(&dil);
    enc_init_cpt(&cpt, law);
    data_cp = cpt;
    data_cp.v90_compatibility = true;   /* data-mode CP */
    cp_prime = data_cp;
    cp_prime.acknowledge = true;

    printf("vpcm_encode: %s%s drn=%u constellations=%u u_info=78 out=%s\n",
           law == V91_LAW_ALAW ? "alaw" : "ulaw",
           v92_compat ? " v92-compat" : "",
           cpt.drn, cpt.constellation_count, out_path);

    /* Leading idle (500 ms) */
    for (int i = 0; i < 4000; i++)
        enc_append(&st, v91_idle_codeword(law));

    /* §8.3.2: U_INFO shall be greater than 66 */
    enc_mark(&st, "Phase 3 start", "Sd/S-bar/TRN1d/Jd, u_info=78");
    v90_start_phase3(tx, 78);
    v90_set_dil_descriptor(tx, &dil);

    if (!enc_pump_until_phase(&st, tx, V90_TX_JD)) {
        fprintf(stderr, "never reached Jd (phase=%d)\n", (int) v90_get_tx_phase(tx));
        goto done;
    }
    enc_mark(&st, "Jd", "");

    /* A realistic stretch of Jd repetitions before the analogue modem's S */
    if (!enc_pump(&st, tx, 20 * 72))
        goto done;
    if (!v90_handle_rx_event(tx, V90_RX_EVENT_S)) {
        fprintf(stderr, "S event rejected during Jd\n");
        goto done;
    }
    enc_mark(&st, "S received", "J'd/DIL follow");

    if (!enc_pump_until_phase(&st, tx, V90_TX_DIL)) {
        fprintf(stderr, "never reached DIL (phase=%d)\n", (int) v90_get_tx_phase(tx));
        goto done;
    }
    enc_mark(&st, "DIL", "125 segments x 12T");

    /* Let a few full DIL cycles play out, then the analogue modem's second
     * S terminates DIL at a segment boundary. */
    if (!enc_pump(&st, tx, 3 * 125 * 12))
        goto done;
    if (!v90_handle_rx_event(tx, V90_RX_EVENT_S)) {
        fprintf(stderr, "second S rejected during DIL\n");
        goto done;
    }
    enc_mark(&st, "S received (2nd)", "DIL terminates");

    if (!enc_pump_until_phase(&st, tx, V90_TX_RI)) {
        fprintf(stderr, "never reached Ri hold (phase=%d)\n", (int) v90_get_tx_phase(tx));
        goto done;
    }
    enc_mark(&st, "Ri (awaiting CPt)", "");

    /* Let the Ri hold breathe before the analogue modem's CPt arrives */
    if (!enc_pump(&st, tx, 480))
        goto done;
    if (v92_compat) {
        /* Compatibility path: cp_ready alone advances to sign-modulated
         * SUVd, legacy CP payload, SUVd', Ed. */
        if (!v90_set_phase4_cp(tx, &cpt)
            || !v90_handle_rx_event(tx, V90_RX_EVENT_CP_VALID)) {
            fprintf(stderr, "CPt rejected (v92-compat)\n");
            goto done;
        }
        enc_mark(&st, "CPt accepted", "SUVd/CPd compatibility flow");
        if (!enc_pump_until_phase(&st, tx, V90_TX_ED)) {
            fprintf(stderr, "never reached Ed (phase=%d)\n", (int) v90_get_tx_phase(tx));
            goto done;
        }
        enc_mark(&st, "Ed", "");
    } else {
        if (!v90_set_phase4_cp(tx, &cpt)
            || !v90_handle_rx_event(tx, V90_RX_EVENT_CP_VALID)) {
            fprintf(stderr, "CPt rejected\n");
            goto done;
        }
        enc_mark(&st, "CPt accepted", "24T Ri + 2040T TRN2d follow");

        if (!enc_pump_until_phase(&st, tx, V90_TX_MP)) {
            fprintf(stderr, "never reached MP (phase=%d)\n", (int) v90_get_tx_phase(tx));
            goto done;
        }
        enc_mark(&st, "MP", "ack=0, D=17 modulus-mapped");

        /* Repeat MP for a while, as if the analogue modem is still training */
        if (!enc_pump(&st, tx, 10 * 36))
            goto done;

        if (!v90_set_phase4_cp(tx, &data_cp)
            || !v90_handle_rx_event(tx, V90_RX_EVENT_CP_VALID)) {
            fprintf(stderr, "data-mode CP rejected\n");
            goto done;
        }
        enc_mark(&st, "data CP accepted", "MP' follows");
        if (!enc_pump(&st, tx, 5 * 36))
            goto done;

        if (!v90_set_phase4_cp(tx, &cp_prime)
            || !v90_handle_rx_event(tx, V90_RX_EVENT_CP_VALID)) {
            /* A-law/other paths may prefer E to terminate MP' */
            if (!v90_handle_rx_event(tx, V90_RX_EVENT_E)) {
                fprintf(stderr, "CP'/E rejected during MP'\n");
                goto done;
            }
        }
        enc_mark(&st, "CP'/E", "Ed at next MP' frame boundary");
        if (!enc_pump_until_phase(&st, tx, V90_TX_ED)) {
            fprintf(stderr, "never reached Ed (phase=%d)\n", (int) v90_get_tx_phase(tx));
            goto done;
        }
        enc_mark(&st, "Ed", "two mapping frames of scrambled zeros");
    }

    if (!enc_pump_until_phase(&st, tx, V90_TX_B1D)) {
        fprintf(stderr, "never reached B1d (phase=%d)\n", (int) v90_get_tx_phase(tx));
        goto done;
    }
    enc_mark(&st, "B1d", "");

    if (!enc_pump_until_phase(&st, tx, V90_TX_DATA)) {
        fprintf(stderr, "never reached data mode (phase=%d)\n", (int) v90_get_tx_phase(tx));
        goto done;
    }
    enc_mark(&st, "Data mode", "");

    {
        uint8_t payload[64];
        uint8_t frame_codewords[6];
        int payload_pos = 0;

        for (int i = 0; i < (int) sizeof(payload); i++)
            payload[i] = (uint8_t) (0xA5 ^ i);

        if (v92_compat) {
            /* Compatibility path negotiates no Table 14 mapper; the engine
             * uses the simplified one-byte-per-codeword data mode. */
            for (int frame = 0; frame < data_frames; frame++) {
                uint8_t cw[6];
                int n = 6;

                if (payload_pos + n > (int) sizeof(payload))
                    payload_pos = 0;
                if (v90_tx_codewords(tx, cw, n, payload + payload_pos, n) != n) {
                    fprintf(stderr, "simplified data mode stalled at frame %d\n", frame);
                    goto done;
                }
                payload_pos += n;
                for (int i = 0; i < n; i++) {
                    if (!enc_append(&st, cw[i]))
                        goto done;
                }
            }
        } else {
            for (int frame = 0; frame < data_frames; frame++) {
                int consumed = 0;

                if (payload_pos >= (int) sizeof(payload))
                    payload_pos = 0;
                if (v90_tx_data_frame_codewords(tx,
                                                frame_codewords,
                                                payload + payload_pos,
                                                (int) sizeof(payload) - payload_pos,
                                                &consumed,
                                                true) != 6) {
                    fprintf(stderr, "data mapper stalled at frame %d\n", frame);
                    goto done;
                }
                payload_pos += consumed;
                for (int i = 0; i < 6; i++) {
                    if (!enc_append(&st, frame_codewords[i]))
                        goto done;
                }
            }
        }
    }
    enc_mark(&st, "Data end", "");

    /* Trailing idle (250 ms) */
    for (int i = 0; i < 2000; i++)
        enc_append(&st, v91_idle_codeword(law));

    {
        const char *ext = strrchr(out_path, '.');
        bool raw = ext && (strcasecmp(ext, ".g711") == 0
                           || strcasecmp(ext, ".raw") == 0
                           || strcasecmp(ext, ".pcm") == 0);

        if (!(raw ? enc_write_g711(out_path, &st)
                  : enc_write_wav(out_path, &st))) {
            fprintf(stderr, "failed to write %s\n", out_path);
            goto done;
        }
        printf("wrote %s (%s): %d codewords (%.1f s)\n",
               out_path, raw ? "raw G.711" : "wav; signed zeros lost, prefer .g711",
               st.count, (double) st.count / 8000.0);
    }
    ret = 0;

done:
    if (tx)
        v90_free(tx);
    free(st.codewords);
    return ret;
}
