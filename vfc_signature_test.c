/* Golden TX/RX cases execute original Courier a6e4/a6f0 and aa18, independently
 * of vfc_signature.c. These test bits, not acquisition or sample processing. */
#include "vfc_signature.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    FILE *f = fopen("reference-data/vfc/signature-tx.csv", "r");
    assert(f);
    char line[256], expected[33];
    unsigned code, count = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        assert(sscanf(line, "%u,%32[0-3]", &code, expected) == 2);
        assert(strlen(expected) == 32);
        uint8_t dibits[32];
        assert(vfc_signature_encode(code, dibits));
        for (unsigned k = 0; k < 32; k++) assert(dibits[k] == expected[k] - '0');
        count++;
    }
    assert(!ferror(f));
    fclose(f);
    assert(count == 16);

    f = fopen("reference-data/vfc/signature-rx.csv", "r");
    assert(f);
    count = 0;
    unsigned passes = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        unsigned header, body, mode, flags, accepted, delay;
        assert(sscanf(line, "%x,%x,%x,%x,%u,%u", &header, &body, &mode, &flags,
                      &accepted, &delay) == 6);
        uint64_t packet = (uint64_t)header | ((uint64_t)body << 32);
        vfc_signature_rx_t rx;
        vfc_signature_t result = {255, 9999};
        vfc_signature_rx_init(&rx);
        unsigned events = 0;
        for (unsigned k = 0; k < 32; k++) {
            bool event = vfc_signature_rx_put(&rx, (packet >> (2*k)) & 3,
                                              (mode & 1) && (flags & 0x1000), &result);
            if (event) assert(k == 31);
            events += event;
        }
        assert(events == accepted);
        if (accepted) {
            assert(result.code == (body & 15) && result.response_count == delay);
            assert(rx.matched);
            assert(!vfc_signature_rx_put(&rx, 3, false, &result));
            passes++;
        } else {
            assert(result.code == 255 && result.response_count == 9999);
            assert(!rx.matched);
        }
        vfc_signature_rx_init(&rx);
        assert(!rx.window && !rx.dibits && !rx.matched);
        assert(!vfc_signature_rx_put(&rx, 4, false, &result));
        assert(!rx.window && !rx.dibits);
        count++;
    }
    assert(!ferror(f));
    fclose(f);
    assert(count == 1280 && passes == 192);
    uint8_t bytes[32];
    memset(bytes, 0xa5, sizeof(bytes));
    assert(!vfc_signature_encode(16, bytes));
    for (unsigned k = 0; k < sizeof(bytes); k++) assert(bytes[k] == 0xa5);
    puts("PASS: V.FC signature matches 16 native TX packets and 1280 native RX cases");
    return 0;
}
