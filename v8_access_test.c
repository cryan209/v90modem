/* V.8 6.3/6.5, Tables 5/7: PCM role and network access are independent.
 * Exercise the production CM/JM serializer, including the zero-valued PSTN
 * category that is mandatory whenever PCM availability is present. */
#include "spandsp-master/src/v8.c"
#include <stdio.h>

int main(void)
{
    int failures = 0;
    for (int caller = 0; caller <= 1; caller++)
    for (int pcm = 0; pcm <= 3; pcm++)
    for (int digital = 0; digital <= 1; digital++)
    {
        v8_parms_t p = {0};
        p.modem_connect_tone = MODEM_CONNECT_TONES_ANSAM_PR;
        p.v92 = -1;
        p.jm_cm.call_function = V8_CALL_V_SERIES;
        p.jm_cm.modulations = V8_MOD_V90 | V8_MOD_V34;
        p.jm_cm.protocols = V8_PROTOCOL_LAPM_V42;
        p.jm_cm.pcm_modem_availability = pcm;
        p.jm_cm.pstn_access = digital ? V8_PSTN_ACCESS_DCE_ON_DIGITAL : 0;
        p.jm_cm.nsf = p.jm_cm.t66 = -1;
        v8_state_t *s = v8_init(NULL, caller, &p, NULL, NULL);
        if (!s) return 2;
        s->result.jm_cm = p.jm_cm;
        s->modulation_bytes = 3;
        queue_flush(s->tx_queue);
        send_cm_jm(s);
        uint8_t bits[256];
        int n = queue_read(s->tx_queue, bits, sizeof(bits));
        int access_count = 0, pcm_count = 0, ok = (n >= 20 && n % 10 == 0);
        for (int i = 10; i + 9 < n; i += 10)
        {
            unsigned value = 0;
            ok &= bits[i] == 0 && bits[i + 9] == 1;
            for (int k = 0; k < 8; k++) value |= (bits[i + 1 + k] & 1) << k;
            if ((value & 0x1f) == V8_PSTN_ACCESS_TAG)
            {
                access_count++;
                ok &= value == (unsigned)((p.jm_cm.pstn_access << 5) | V8_PSTN_ACCESS_TAG);
            }
            if ((value & 0x1f) == V8_PCM_MODEM_AVAILABILITY_TAG)
            {
                pcm_count++;
                ok &= value == (unsigned)((pcm << 5) | V8_PCM_MODEM_AVAILABILITY_TAG);
            }
        }
        ok &= access_count == ((pcm || digital) ? 1 : 0);
        ok &= pcm_count == (pcm ? 1 : 0);
        printf("%s %s PCM=%d access=%s\n", ok ? "PASS" : "FAIL",
               caller ? "CM" : "JM", pcm, digital ? "digital" : "analogue");
        failures += !ok;
        v8_free(s);
    }
    return failures != 0;
}
