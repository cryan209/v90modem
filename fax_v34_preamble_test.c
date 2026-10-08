/* T.30 F.3.2.3 and T.4 A.3.1 require 200 ms of primary-channel flags.
 * Exercise the production transport branch and inspect its emitted bits. */
#include "spandsp-master/src/fax.c"
#include <assert.h>

int main(void)
{
    const int rates[] = {2400, 9600, 14400, 33600};
    const uint8_t frame[] = {0xff, 0x03, 0x06, 0x00};
    for (unsigned r = 0; r < sizeof(rates)/sizeof(rates[0]); r++)
    {
        fax_state_t *s = calloc(1, sizeof(*s));
        assert(s);
        s->v34hdx_external = true;
        s->v34hdx_primary_bit_rate = rates[r];
        hdlc_tx_init(&s->modems.hdlc_tx, false, 2, false, NULL, NULL);
        fax_set_tx_type(s, T30_MODEM_V34HDX, rates[r], 0, true);
        /* Model the completed Annex F marks/silence handoff. Queue a real
           frame so an empty transmitter cannot supply endless idle flags. */
        s->v34hdx_transition = 0;
        s->v34hdx_channel = s->v34hdx_requested_mode = V34_HALF_DUPLEX_PRIMARY_CHANNEL;
        assert(hdlc_tx_frame(&s->modems.hdlc_tx, frame, sizeof(frame)) == 0);
        for (int i = 0; i < rates[r]/5; i++)
            assert(fax_v34hdx_get_bit(s) == ((0x7e >> (i%8)) & 1));
        /* The queued address follows immediately after the preamble. */
        for (int i = 0; i < 5; i++)
            assert(fax_v34hdx_get_bit(s) == 1);
        free(s);
    }
    puts("PASS: primary ECM emits 200 ms of flags before the queued frame");
    return 0;
}
