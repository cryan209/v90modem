#include "../p3_demod.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    FILE *fp;
    long bytes;
    int16_t *samples;
    p3_hypothesis_t h[P3_BAUD_COUNT*2];
    int n;

    if (argc != 2)
        return 2;
    fp = fopen(argv[1], "rb");
    if (!fp)
        return 2;
    fseek(fp, 0, SEEK_END);
    bytes = ftell(fp);
    rewind(fp);
    samples = malloc((size_t) bytes);
    if (!samples)
        return 2;
    fread(samples, 1, (size_t) bytes, fp);
    fclose(fp);
    n = p3_scan_all_hypotheses(samples, (int) (bytes/2), 0, 8000,
                               h, P3_BAUD_COUNT*2);
    for (int i = 0; i < n; i++)
        printf("%d baud_code=%d carrier=%d hz=%.1f baud=%.1f score=%.1f S=%d TRN=%d J=%d\n",
               i, h[i].baud_code, h[i].carrier_sel, h[i].carrier_hz,
               h[i].baud_rate, h[i].score, h[i].has_s, h[i].has_trn, h[i].has_j);
    free(samples);
    return 0;
}
