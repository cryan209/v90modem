/* Offline probe for the strict live V.90 CP fallback. */
#include "v90_cp_live.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    FILE *fp;
    long bytes;
    int16_t *samples;
    int count;
    int hint;
    int compatibility;
    int baud_code;
    vpcm_cp_diag_t diag;
    v90_cp_live_meta_t meta;

    if (argc != 4 && argc != 5) {
        fprintf(stderr,
                "usage: %s input.s16le phase4_hint_sample compatibility [baud-code]\n",
                argv[0]);
        return 2;
    }
    hint = atoi(argv[2]);
    compatibility = atoi(argv[3]);
    baud_code = argc == 5 ? atoi(argv[4]) : 4;
    fp = fopen(argv[1], "rb");
    if (!fp || fseek(fp, 0, SEEK_END) != 0
        || (bytes = ftell(fp)) < 0 || fseek(fp, 0, SEEK_SET) != 0) {
        perror(argv[1]);
        if (fp)
            fclose(fp);
        return 2;
    }
    count = (int)(bytes / (long)sizeof(*samples));
    samples = malloc((size_t)count * sizeof(*samples));
    if (!samples || fread(samples, sizeof(*samples), (size_t)count, fp)
                      != (size_t)count) {
        fprintf(stderr, "unable to read %s\n", argv[1]);
        free(samples);
        fclose(fp);
        return 2;
    }
    fclose(fp);
    if (!v90_cp_live_recover(samples,
                             count,
                             hint,
                             baud_code,
                             compatibility,
                             false,
                             &diag,
                             &meta)) {
        fprintf(stderr, "no strict frame\n");
        free(samples);
        return 1;
    }
    printf("strict %s%s bits=%d frame=%d last=%d carrier=%s timing=%d "
           "step=%d pll=%.3f conjugate=%d drn=%u mask=0x%04x "
           "constellations=%u alaw=%d codec-diff=%d crc=%u map=%d order=%d vote=%d/%d%%\n",
           diag.frame.v90_compatibility ? "CP" : "CPt",
           diag.frame.acknowledge ? "'" : "",
           diag.nbits,
           meta.frame_sample,
           meta.last_sample,
           meta.carrier_sel ? "high" : "low",
           meta.timing_index,
           meta.carrier_step,
           meta.pll_gain,
           meta.conjugate ? 1 : 0,
           (unsigned)diag.frame.drn,
           diag.frame.upstream_rate_mask,
           (unsigned)diag.frame.constellation_count,
           diag.frame.codec_alaw ? 1 : 0,
           diag.frame.codec_constellations_differ ? 1 : 0,
           (unsigned)diag.crc_remainder,
           meta.map_index,
           meta.bit_order,
           meta.voted_frames,
           meta.agreement_pct);
    printf("CP fields: sr=%u ld=%u gain=0x%04x filter=%02x,%02x,%02x,%02x "
           "dfi=%u,%u,%u,%u,%u,%u codec-diff=%d\n",
           (unsigned)diag.frame.shaping_redundancy,
           (unsigned)diag.frame.shaping_lookahead,
           (unsigned)diag.frame.trn1d_gain_q3_13,
           (unsigned)diag.frame.shaping_a1_q1_6,
           (unsigned)diag.frame.shaping_a2_q1_6,
           (unsigned)diag.frame.shaping_b1_q1_6,
           (unsigned)diag.frame.shaping_b2_q1_6,
           (unsigned)diag.frame.dfi[0],
           (unsigned)diag.frame.dfi[1],
           (unsigned)diag.frame.dfi[2],
           (unsigned)diag.frame.dfi[3],
           (unsigned)diag.frame.dfi[4],
           (unsigned)diag.frame.dfi[5],
           diag.frame.codec_constellations_differ ? 1 : 0);
    for (int constellation = 0;
         constellation < diag.frame.constellation_count;
         constellation++) {
        printf("mask[%d]=", constellation);
        for (int byte = 15; byte >= 0; byte--)
            printf("%02x", diag.frame.masks[constellation][byte]);
        if (diag.frame.codec_constellations_differ) {
            printf(" codec=");
            for (int byte = 15; byte >= 0; byte--)
                printf("%02x", diag.frame.codec_masks[constellation][byte]);
        }
        putchar('\n');
    }
    free(samples);
    return 0;
}
