/* V.90 Table 7, 9.5.2: PCM law/power survive an INFO0-less retrain,
 * but cannot leak into a new call or alter a digital G.711 bearer. */
#include <assert.h>
#include <stdio.h>
#include "vpcm_analogue_peer.h"

int main(void)
{
    vpcm_analogue_peer_t p;
    vpcm_analogue_peer_reset(&p);
    assert(!p.valid);
    for (int alaw = 0; alaw < 2; alaw++)
        assert(vpcm_analogue_peer_alaw(&p, true, alaw) == (bool)alaw);
    for (int law = 0; law < 2; law++) {
        vpcm_analogue_peer_note(&p, true, law, 17, true);
        assert(p.valid && p.codec_output);
        assert(vpcm_analogue_peer_max_power(&p) == -9.0);
        for (int bearer = 0; bearer < 2; bearer++) {
            assert(vpcm_analogue_peer_alaw(&p, true, bearer) == (bool)law);
            assert(vpcm_analogue_peer_alaw(&p, false, bearer) == (bool)bearer);
        }
        /* v34_restart clears its INFO0 fields; 9.5.2 does not resend them. */
        vpcm_analogue_peer_note(&p, false, !law, 0, false);
        assert(p.alaw == (bool)law && p.codec_output);
        assert(vpcm_analogue_peer_max_power(&p) == -9.0);
    }
    for (unsigned code = 0; code < 32; code++) {
        vpcm_analogue_peer_note(&p, true, false, code, false);
        assert(vpcm_analogue_peer_max_power(&p) == -0.5*(code + 1));
    }
    vpcm_analogue_peer_reset(&p);
    assert(!p.valid && !p.alaw && !p.codec_output && p.max_power_code == 0);
    assert(vpcm_analogue_peer_max_power(&p) == 0.0);
    assert(!vpcm_analogue_peer_alaw(&p, true, false));
    puts("PASS: physical PCM law selection, unchanged digital bearer, retained INFO0d power/law and new-call reset");
    return 0;
}
