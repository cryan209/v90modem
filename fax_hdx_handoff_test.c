/* T.30 F.3.2.2: repeated DCS must finish CFR before the primary handoff.
 * Fixture: generated checkerboard from MT9234ZBA-USB, wired-11, 28 Oct 2026.
 * Receiver-only protocol regression; does not qualify a physical modem. */
#include <stdio.h>
#include <stdbool.h>
#include <spandsp.h>
#include "test_tmp.h"

int main(void)
{
    FILE *in = fopen("reference-data/mt9234zba/repeated-dcs.bits", "rb");
    if (!in) { perror("repeated-dcs.bits"); return 1; }
    fax_state_t *fax = fax_init(NULL, false);
    if (!fax) return 1;
    t30_state_t *t30 = fax_get_t30_state(fax);
    t30_set_rx_file(t30, test_tmp("hdx-handoff.tif"), -1);
    t30_set_supported_modems(t30, T30_SUPPORT_V34HDX | T30_SUPPORT_V17
                                   | T30_SUPPORT_V29 | T30_SUPPORT_V27TER);
    t30_set_ecm_capability(t30, true);
    if (fax_v34hdx_start_control(fax, 33600)) return 1;
    int mode = V34_HALF_DUPLEX_CONTROL_CHANNEL;
    int octet = 0;
    bool entered = false, returned = false;
    double clock = 0;
    for (int n = 0; n < 86000; n++)
    {
        if (!(n & 7)) octet = fgetc(in);
        if (octet == EOF) break;
        fax_v34hdx_put_bit(fax, (octet >> (n & 7)) & 1);
        /* A primary recipient is silent: no control TX callbacks can finish
         * a stale CFR after this boundary. Clocking get_bit here hides it. */
        if (mode == V34_HALF_DUPLEX_CONTROL_CHANNEL)
            fax_v34hdx_get_bit(fax);
        clock += 8000.0/(mode == V34_HALF_DUPLEX_PRIMARY_CHANNEL ? 33600 : 1200);
        int samples = (int)clock;
        clock -= samples;
        fax_v34hdx_advance(fax, samples);
        int next = fax_v34hdx_get_mode(fax);
        if (next != mode)
        {
            fax_v34hdx_set_channel(fax, next);
            if (next == V34_HALF_DUPLEX_PRIMARY_CHANNEL) entered = true;
            if (entered && mode == V34_HALF_DUPLEX_PRIMARY_CHANNEL
                        && next == V34_HALF_DUPLEX_CONTROL_CHANNEL)
            {
                returned = true;
                break;
            }
            mode = next;
        }
    }
    fclose(in);
    fax_free(fax);
    if (!entered || !returned)
    {
        fprintf(stderr, "FAIL: repeated DCS stranded the primary recipient in CFR\n");
        return 1;
    }
    puts("PASS: repeated DCS, primary FCD reception, RCP return to control");
    return 0;
}
