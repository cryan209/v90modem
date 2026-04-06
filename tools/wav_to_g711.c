/*
 * wav_to_g711.c — Split a stereo 8kHz 16-bit PCM WAV into L/R channels
 *                 and encode each to raw G.711 A-law or μ-law using SpanDSP.
 *
 * Usage: wav_to_g711 <input.wav> <out_L.g711> <out_R.g711> <a|u>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "spandsp/telephony.h"
#include "spandsp/bit_operations.h"
#include "spandsp/g711.h"

/* Minimal WAV header parser — enough for PCM files */
static int read_wav_header(FILE *fp, int *channels, int *sample_rate, int *bits)
{
    uint8_t hdr[44];
    if (fread(hdr, 1, 44, fp) != 44) return -1;
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr+8, "WAVE", 4) != 0) return -1;
    if (memcmp(hdr+12, "fmt ", 4) != 0) return -1;
    uint16_t audio_format = hdr[20] | (hdr[21] << 8);
    if (audio_format != 1) return -1; /* must be PCM */
    *channels    = hdr[22] | (hdr[23] << 8);
    *sample_rate = hdr[24] | (hdr[25] << 8) | (hdr[26] << 16) | (hdr[27] << 24);
    *bits        = hdr[34] | (hdr[35] << 8);

    /* Scan for "data" chunk (may not be exactly at offset 36) */
    uint8_t tag[4];
    uint32_t chunk_size;
    /* We've consumed 44 bytes; data chunk starts at offset 36 in standard PCM */
    /* Re-seek and skip format chunk properly */
    fseek(fp, 12, SEEK_SET);
    while (1) {
        if (fread(tag, 1, 4, fp) != 4) return -1;
        if (fread(&chunk_size, 4, 1, fp) != 1) return -1;
        if (memcmp(tag, "data", 4) == 0) break;
        fseek(fp, chunk_size, SEEK_CUR);
    }
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc != 5) {
        fprintf(stderr, "Usage: %s <input.wav> <out_L.g711> <out_R.g711> <a|u>\n", argv[0]);
        return 1;
    }

    const char *in_path  = argv[1];
    const char *out_l    = argv[2];
    const char *out_r    = argv[3];
    int use_alaw = (argv[4][0] == 'a' || argv[4][0] == 'A');

    FILE *fin = fopen(in_path, "rb");
    if (!fin) { perror(in_path); return 1; }

    int channels, sample_rate, bits;
    if (read_wav_header(fin, &channels, &sample_rate, &bits) != 0) {
        fprintf(stderr, "Not a supported PCM WAV: %s\n", in_path);
        return 1;
    }
    if (channels < 2) {
        fprintf(stderr, "Expected stereo WAV, got %d channel(s)\n", channels);
        return 1;
    }
    if (bits != 16) {
        fprintf(stderr, "Expected 16-bit PCM, got %d bits\n", bits);
        return 1;
    }

    FILE *fl = fopen(out_l, "wb");
    FILE *fr = fopen(out_r, "wb");
    if (!fl || !fr) { perror("output"); return 1; }

    int16_t frame[2];
    while (fread(frame, sizeof(int16_t), 2, fin) == 2) {
        uint8_t l_byte, r_byte;
        if (use_alaw) {
            l_byte = linear_to_alaw((int)frame[0]);
            r_byte = linear_to_alaw((int)frame[1]);
        } else {
            l_byte = linear_to_ulaw((int)frame[0]);
            r_byte = linear_to_ulaw((int)frame[1]);
        }
        fwrite(&l_byte, 1, 1, fl);
        fwrite(&r_byte, 1, 1, fr);
    }

    fclose(fin);
    fclose(fl);
    fclose(fr);
    return 0;
}
