/* Feeds a V90_RENEG_SYM_DUMP capture through the standalone streamed CP
 * decoder and reports the frames the Table 14 framer accepts.
 *
 * Reference: artifacts/reneg-eq/reneg-r1, whose in-engine §9.6 CP window
 * reports  valid=6 cp_ack=1  with the streamed path on, and
 *          valid=0 cp_ack=0  with it off (91 syncs, all structural rejects).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cp_stream.h"
#include "vpcm_cp.h"

typedef struct {
    int valid;
    int ack;
    int last_bits;
    int gaps[64];
    int ngaps;
    long last_pos;
} tally_t;

static long g_bitpos;

static void on_frame(void *user_data, const vpcm_cp_diag_t *diag)
{
    tally_t *t = user_data;

    if (!diag->frame_sync_ok)
        return;
    t->valid++;
    t->last_bits = diag->nbits;
    if (t->last_pos >= 0 && t->ngaps < 64)
        t->gaps[t->ngaps++] = (int) (g_bitpos - t->last_pos);
    t->last_pos = g_bitpos;
    /* CP' -- the terminating frame that releases Ed (§9.6.1.2.3). */
    if (diag->frame.acknowledge)
        t->ack = 1;
    printf("  frame %d: %d bits, ack=%d, crc_field=0x%04x\n",
           t->valid, diag->nbits, diag->frame.acknowledge, diag->crc_field);
}

int main(int argc, char *argv[])
{
    FILE *f;
    char line[512];
    v90_cp_rx_t framer;
    cp_stream_t stream;
    tally_t t;
    long syms = 0;
    int limit = (argc > 2) ? atoi(argv[2]) : 0;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <sym-dump> [max-symbols]\n", argv[0]);
        return 2;
    }
    if ((f = fopen(argv[1], "r")) == NULL) {
        perror(argv[1]);
        return 2;
    }

    memset(&t, 0, sizeof(t));
    t.last_pos = -1;
    v90_cp_rx_init(&framer, 4, false /* mu-law */, on_frame, &t);
    cp_stream_init(&stream, &framer);

    while (fgets(line, sizeof(line), f)) {
        int duration, dibit;

        if (sscanf(line, "%d %d", &duration, &dibit) != 2)
            continue;
        cp_stream_put_dibit(&stream, dibit);
        syms++;
        g_bitpos += 2;
        if (limit && syms >= limit)
            break;
    }
    fclose(f);

    printf("symbols=%ld bits=%ld sync=%u valid=%u rejected=%u "
           "(crc=%u structure=%u semantic=%u) cp_ack=%d\n",
           syms, syms * 2,
           framer.sync_candidates, framer.valid_frames,
           framer.rejected_frames, framer.crc_rejected_frames,
           framer.structure_rejected_frames, framer.semantic_rejected_frames,
           t.ack);
    if (t.ngaps) {
        int i;
        printf("frame gaps (bits):");
        for (i = 0; i < t.ngaps; i++)
            printf(" %d", t.gaps[i]);
        printf("\n");
    }
    return 0;
}
