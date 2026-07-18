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
    vpcm_cp_diag_t diag;
    v90_cp_live_meta_t meta;

    if (argc != 4) {
        fprintf(stderr,
                "usage: %s input.s16le phase4_hint_sample compatibility\n",
                argv[0]);
        return 2;
    }
    hint = atoi(argv[2]);
    compatibility = atoi(argv[3]);
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
           "constellations=%u codec-diff=%d crc=%u map=%d order=%d vote=%d/%d%%\n",
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
           diag.frame.codec_constellations_differ ? 1 : 0,
           (unsigned)diag.crc_remainder,
           meta.map_index,
           meta.bit_order,
           meta.voted_frames,
           meta.agreement_pct);
    free(samples);
    return 0;
}
