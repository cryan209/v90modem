#include <spandsp.h>
#include <spandsp/dds.h>
#include <spandsp/private/bitstream.h>
#include <spandsp/private/logging.h>
#include <spandsp/private/power_meter.h>
#include <spandsp/private/v34.h>
#include "../v90.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int get_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void put_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

static void try_descriptor(const uint8_t *bits, int n, int hypothesis)
{
    uint8_t packed[(65536 + 7)/8];
    int start;

    for (start = 0; start + 206 <= n; start++)
    {
        int i;
        v90_dil_desc_t desc;

        for (i = 0; i < 17 && bits[start + i]; i++)
            ;
        if (i != 17 || bits[start + 17])
            continue;
        memset(packed, 0, sizeof(packed));
        for (i = start; i < n; i++)
        {
            if (bits[i])
                packed[(i - start)/8] |= (uint8_t) (1U << ((i - start)%8));
        }
        if (v90_parse_dil_descriptor(&desc, packed, n - start))
        {
            fprintf(stderr,
                    "DESCRIPTOR hyp=%d start=%d N=%u LSP=%u LTP=%u\n",
                    hypothesis, start, desc.n, desc.lsp, desc.ltp);
            return;
        }
    }
}

int main(int argc, char *argv[])
{
    FILE *fp;
    v34_state_t *s;
    int16_t amp[160];
    size_t len;
    uint8_t *bits;
    int h;

    if (argc != 2)
    {
        fprintf(stderr, "usage: %s phase3-s16le-8000.raw\n", argv[0]);
        return 2;
    }
    fp = fopen(argv[1], "rb");
    if (!fp)
    {
        perror(argv[1]);
        return 2;
    }
    s = v34_init(NULL, 2400, 21600, false, true,
                 get_bit, NULL, put_bit, NULL);
    if (!s)
        return 2;
    v34_set_v90_mode(s, 0);
    s->rx.baud_rate = 0;
    s->tx.baud_rate = 0;
    s->rx.high_carrier = true;
    s->tx.high_carrier = true;
    s->rx.v34_carrier_phase_rate = dds_phase_ratef(1800.0f);
    s->rx.scrambler_tap = 4;
    span_log_set_level(v34_get_logging_state(s),
                       SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_FLOW);
    v34_force_phase3_rx(s);
    while ((len = fread(amp, sizeof(*amp), 160, fp)) > 0)
        v34_rx(s, amp, (int) len);
    fclose(fp);
    fprintf(stderr,
            "REPLAY stage=%d duration=%d pp_started=%d phase=%d score=%d hits=%d "
            "trn_bits=%d trn_hyp=%d trn_score=%d j_bits=%d ja_bits=%d\n",
            s->rx.stage, s->rx.duration, s->rx.phase3_pp_started,
            s->rx.phase3_pp_phase, s->rx.phase3_pp_phase_score,
            s->rx.phase3_pp_acquire_hits, s->rx.phase3_trn_bits,
            s->rx.phase3_trn_lock_hyp, s->rx.phase3_trn_lock_score,
            s->rx.phase3_j_bits, s->rx.phase3_ja_bits);
    bits = malloc(65536);
    if (bits)
    {
        for (h = 0; h < 24; h++)
        {
            char path[80];
            int n;
            FILE *out;

            n = v34_v90_copy_phase3_ja_bits(s, h, bits, 65536);
            try_descriptor(bits, n, h);
            snprintf(path, sizeof(path), "/tmp/replay-hyp%d.bits", h);
            out = fopen(path, "wb");
            if (out)
            {
                fwrite(bits, 1, (size_t) n, out);
                fclose(out);
            }
            n = v34_v90_copy_phase3_ja_raw_bits(s, h, bits, 65536);
            snprintf(path, sizeof(path), "/tmp/replay-hyp%d.rawbits", h);
            out = fopen(path, "wb");
            if (out)
            {
                fwrite(bits, 1, (size_t) n, out);
                fclose(out);
            }
        }
        free(bits);
    }
    v34_free(s);
    return 0;
}
