/* V.34 10.1.3.3: the J a duplex modem RECEIVES selects the constellation of
 * its own Phase 4 TRN, MP, MP' and E (10.1.3.2, 10.1.3.8, 10.1.3.9).  Includes
 * the production v34tx.c and calls the real static generators.  Generator
 * level only: our receiver has no 16-point Phase 4 path, so there is no
 * waveform loopback for it.
 */
#include "spandsp-master/src/v34tx.c"

#include <assert.h>

static bool on_16point_table(complex_sig_t p)
{
    for (int i = 0; i < 16; i++)
        if (p.re == training_constellation_16[i].re
            && p.im == training_constellation_16[i].im)
            return true;
    return false;
}

static int mp_bits_per_symbol(v34_state_t *s, int j, bool reneg)
{
    int before;

    s->rx.phase3_j_trn16 = j;
    s->tx.reneg_active = reneg;
    memset(&s->tx.mp, 0, sizeof(s->tx.mp));
    s->tx.mp.type = 0;
    s->tx.mp.bit_rate_a_to_c = 9600;
    s->tx.mp.bit_rate_c_to_a = 9600;
    s->tx.mp.signalling_rate_mask = 0x1FFF;
    s->tx.txbits = mp_sequence_tx(&s->tx, &s->tx.mp);
    s->tx.txptr = 0;
    before = s->tx.txptr;
    (void) get_mp_or_mph_baud(s);
    return s->tx.txptr - before;
}

int main(void)
{
    v34_state_t *s = v34_init(NULL, 2400, 9600, true, true, NULL, NULL, NULL, NULL);
    int symbols;

    assert(s);
    assert(mp_bits_per_symbol(s, 0, false) == 2);      /* J asked for 4-point */
    assert(mp_bits_per_symbol(s, -1, false) == 2);     /* J never decoded */
    assert(mp_bits_per_symbol(s, 1, false) == 4);      /* J asked for 16-point */
    assert(mp_bits_per_symbol(s, 1, true) == 2);       /* 11.6 is four-point */

    /* E: 20 bits = ten 4-point symbols or five 16-point ones, all on the
     * 16-point table when 16-point. */
    for (int j = 0; j <= 1; j++)
    {
        s->rx.phase3_j_trn16 = j;
        s->tx.reneg_active = false;
        s->tx.scramble_reg = 0;
        s->tx.diff = 0;
        e_baud_init(s);
        symbols = 0;
        while (s->tx.current_getbaud == get_e_baud  &&  symbols < 40)
        {
            complex_sig_t p = get_e_baud(s);

            symbols++;
            assert(j ? on_16point_table(p) : true);
        }
        assert(symbols == (j ? 5 : 10));
    }
    v34_free(s);
    puts("PASS: V.34 Phase 4 MP/E constellation follows the received J");
    return 0;
}
