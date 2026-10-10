/*
 * line_monitor_test.c -- the ATY11 band levels against tones of known level.
 *
 * A sine of peak A has mean square A^2/2, so on SpanDSP's scale (full-scale
 * sine = +3.14 dBm0) a tone at L dBm0 has A = 32768 * 10^((L - 3.14)/20).
 * Put one at a band centre and its band must read L, and bands well clear of
 * it must read far below; put it through G.711 and the level must survive.
 */
#include "line_monitor.h"

#include <spandsp.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;

static void check(int ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        failures++;
}

static void tone(int dir, double hz, double dbm0, int n, int g711, bool alaw)
{
    double a = 32768.0 * pow(10.0, (dbm0 - DBM0_MAX_SINE_POWER) / 20.0);
    int16_t x[160];
    uint8_t c[160];

    for (int s = 0; s < n; s += 160) {
        for (int i = 0; i < 160; i++)
            x[i] = (int16_t) lrint(a * sin(2.0 * M_PI * hz * (s + i) / 8000.0));
        if (g711) {
            for (int i = 0; i < 160; i++)
                c[i] = alaw ? linear_to_alaw(x[i]) : linear_to_ulaw(x[i]);
            lm_feed_g711(dir, c, 160, alaw);
        } else {
            lm_feed(dir, x, 160);
        }
    }
}

int main(void)
{
    float rx[LM_BANDS], tx[LM_BANDS], lvl;
    char what[160];
    char page[4096];

    printf("line_monitor:\n");
    check(!lm_level_dbm0(LM_RX, &lvl), "nothing fed: no level");
    lm_format_bands(page, sizeof(page));
    check(strstr(page, "No line audio") != NULL, "nothing fed: the page says so");

    /* Linear, both directions, two tones at band centres. */
    tone(LM_RX, 1050.0, -10.0, 8000, 0, false);
    tone(LM_TX, 1800.0, -20.0, 8000, 0, false);
    lm_bands_dbm0(rx, tx);
    snprintf(what, sizeof(what), "RX -10 dBm0 at 1050 Hz reads %.2f in its band", rx[6]);
    check(fabsf(rx[6] + 10.0f) < 0.3f, what);
    snprintf(what, sizeof(what), "TX -20 dBm0 at 1800 Hz reads %.2f in its band", tx[11]);
    check(fabsf(tx[11] + 20.0f) < 0.3f, what);
    snprintf(what, sizeof(what), "bands two away from the tone are >50 dB down (%.1f, %.1f)",
             rx[4], rx[8]);
    check(rx[4] < -60.0f && rx[8] < -60.0f, what);
    check(lm_level_dbm0(LM_RX, &lvl) && fabsf(lvl + 10.0f) < 0.1f, "RX total level -10 dBm0");
    check(lm_level_dbm0(LM_TX, &lvl) && fabsf(lvl + 20.0f) < 0.1f, "TX total level -20 dBm0");

    /* Through G.711 in each law: quantisation leaves the level within 0.2 dB. */
    for (int law = 0; law < 2; law++) {
        lm_reset();
        tone(LM_RX, 2400.0, -13.0, 8000, 1, law);
        lm_bands_dbm0(rx, NULL);
        snprintf(what, sizeof(what), "%s: -13 dBm0 at 2400 Hz reads %.2f", law ? "A-law" : "u-law",
                 rx[15]);
        check(fabsf(rx[15] + 13.0f) < 0.2f, what);
    }

    /* The ring keeps only the last second: a new tone replaces the old. */
    lm_reset();
    tone(LM_RX, 600.0, -10.0, 8000, 0, false);
    tone(LM_RX, 3000.0, -10.0, 8000, 0, false);
    lm_bands_dbm0(rx, NULL);
    check(rx[3] < -60.0f && fabsf(rx[19] + 10.0f) < 0.3f, "only the last second is measured");

    lm_format_bands(page, sizeof(page));
    check(strstr(page, "  3000 -10.0") && strstr(page, "  3900") && strstr(page, "Total"),
          "the page lists every band, 150..3900 Hz, and the totals");
    /* The GUI sees exact codewords and bit packing without touching DSP. */
    lm_gui_enable();
    lm_reset();
    uint8_t codes[] = {0x00, 0x7f, 0x80, 0xff};
    lm_feed_g711(LM_RX, codes, 4, false);
    for (int i = 0; i < 8; i++) lm_wire_bit(LM_TX, (0xa5 >> i) & 1);
    lm_wire_bit(LM_TX, -1);
    lm_qam(1.25f, -2.5f, false);
    lm_qam(2.0f, -3.0f, true);
    lm_event("TRN \"quoted\" \\ E");
    char json[60000];
    check(lm_gui_json(json, sizeof(json)) > 0, "GUI snapshot fits bounded buffer");
    check(strstr(json, "\"hex\":\"007f80ff\"") != NULL, "GUI preserves exact G.711 octets");
    check(strstr(json, "\"hex\":\"a5\"") != NULL, "GUI packs line bits LSB first, ignores status codes");
    check(strstr(json, "[1.25,-2.5]") && strstr(json, "[2,-3]"), "GUI distinguishes measured QAM from decisions");
    check(strstr(json, "TRN \\\"quoted\\\" \\\\ E") != NULL, "GUI escapes event JSON");
    check(lm_gui_json(page, 8) == -1, "GUI rejects truncated snapshots");
    for (int i = 0; i < 257*8; i++) lm_wire_bit(LM_RX, 1);
    lm_gui_json(json, sizeof(json));
    check(strstr(json, "\"count\":257") != NULL, "GUI wire ring wraps while retaining total count");
    check(strstr(json, "[255,257]") != NULL, "GUI keeps complete repeated byte run beyond raw ring");
    lm_wire_bit(LM_TX, 1); /* an incomplete octet must not leak to next call */
    lm_reset();
    for (int i = 0; i < 8; i++) lm_wire_bit(LM_TX, 0);
    lm_gui_json(json, sizeof(json));
    check(strstr(json, "\"hex\":\"00\"") && !strstr(json, "quoted"), "GUI resets partial bits and events per call");
    int16_t monitor_audio[1700];
    for (int i = 0; i < 1700; i++) monitor_audio[i] = (i & 1) ? -32768 : 32767;
    for (int d = 0; d < 2; d++) {
        lm_feed(d, monitor_audio, 1700);
        for (int i = 0; i < 300; i++) for (int b = 0; b < 8; b++) lm_wire_bit(d, (i >> b) & 1);
    }
    for (int i = 0; i < 256; i++) {
        lm_qam(12345.67f, -23456.78f, false);
        lm_qam(-23456.78f, 12345.67f, true);
    }
    check(lm_gui_json(json, sizeof(json)) > 0, "GUI full audio, runs and IQ fit one bounded UDP snapshot");
    check(strstr(json, "\"listen\":[{\"count\":1700,\"hex\":\"ff7f0080") != NULL,
          "GUI listening has sample count and exact little-endian diagnostic samples");
    printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures != 0;
}
