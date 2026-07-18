#include "vpcm_cp.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    FILE *fp;
    uint8_t bits[VPCM_CP_MAX_BITS];
    int nbits = 0;
    int ch;
    vpcm_cp_diag_t diag;

    if (argc != 2) {
        fprintf(stderr, "usage: %s <ascii-bits-file>\n", argv[0]);
        return 2;
    }
    fp = fopen(argv[1], "rb");
    if (!fp) {
        perror(argv[1]);
        return 2;
    }
    while ((ch = fgetc(fp)) != EOF) {
        if (ch != '0' && ch != '1')
            continue;
        if (nbits >= VPCM_CP_MAX_BITS) {
            fprintf(stderr, "too many bits (maximum %d)\n", VPCM_CP_MAX_BITS);
            fclose(fp);
            return 2;
        }
        bits[nbits++] = (uint8_t) (ch - '0');
    }
    fclose(fp);

    (void) vpcm_cp_decode_diag(bits, nbits, &diag);
    printf("bits=%d valid=%d sync=%d start=%d reserved=%d compat=%d fill=%d "
           "crc_field=%04x crc_remainder=%04x\n",
           nbits, diag.valid, diag.frame_sync_ok, diag.start_bits_ok,
           diag.reserved_bits_ok, diag.v90_compat_ok, diag.fill_ok,
           diag.crc_field, diag.crc_remainder);
    printf("v90=%d transparent=%d drn=%u rate=%.0f ack=%d alaw=%d "
           "redundancy=%u lookahead=%u upstream_mask=%04x "
           "constellations=%u codec_diff=%d\n",
           diag.frame.v90_compatibility,
           diag.frame.transparent_mode_granted,
           diag.frame.drn, vpcm_cp_drn_to_bps(diag.frame.drn),
           diag.frame.acknowledge, diag.frame.codec_alaw,
           diag.frame.shaping_redundancy, diag.frame.shaping_lookahead,
           diag.frame.upstream_rate_mask,
           diag.frame.constellation_count,
           diag.frame.codec_constellations_differ);
    printf("dfi=%u,%u,%u,%u,%u,%u mask_population=",
           diag.frame.dfi[0], diag.frame.dfi[1], diag.frame.dfi[2],
           diag.frame.dfi[3], diag.frame.dfi[4], diag.frame.dfi[5]);
    for (int i = 0; i < diag.frame.constellation_count; ++i)
        printf("%s%d", i ? "," : "",
               vpcm_cp_mask_population(diag.frame.masks[i]));
    putchar('\n');

    return diag.valid ? 0 : 1;
}
