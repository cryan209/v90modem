#include <stdint.h>
#include <stdio.h>

extern void DILdescriptorPacker_original(void *descriptor,
                                         int16_t *bits,
                                         int16_t *bit_count);

void DILdescriptorPacker(void *descriptor, int16_t *bits, int16_t *bit_count)
{
    const uint8_t *d = (const uint8_t *)descriptor;
    static unsigned calls;

    DILdescriptorPacker_original(descriptor, bits, bit_count);
    calls++;
    if (calls <= 8) {
        int count = bit_count ? *bit_count : 0;
        int shown = count < 256 ? count : 256;

        fprintf(stderr,
                "DILDBG pack call=%u N=%u LSP=%u LTP=%u bits=%d stream=",
                calls, d[0], d[1], d[2], count);
        for (int i = 0; i < shown; i++)
            fputc(bits[i] ? '1' : '0', stderr);
        if (shown < count)
            fputs("...", stderr);
        fputc('\n', stderr);
    }
}

extern int get_v90_ja_bits_original(void *self, int16_t *dibit);

int get_v90_ja_bits_debug(void *self, int16_t *dibit)
    __asm__("_ZN12VPcmFloModem12getV90JaBitsEPs");

int get_v90_ja_bits_debug(void *self, int16_t *dibit)
{
    const uint8_t *raw = (const uint8_t *)self;
    const int16_t *ja_bits = (const int16_t *)(raw + 0x21e);
    int count = *(const uint16_t *)(raw + 0x1736);
    static unsigned calls;
    int result;

    if (calls == 0) {
        int shown = count < 1024 ? count : 1024;

        fprintf(stderr, "DILDBG Ja source bits=%d stream=", count);
        for (int i = 0; i < shown; i++)
            fputc(ja_bits[i] ? '1' : '0', stderr);
        if (shown < count)
            fputs("...", stderr);
        fputc('\n', stderr);
    }
    result = get_v90_ja_bits_original(self, dibit);
    if (calls < 16)
        fprintf(stderr, "DILDBG Ja dibit[%u]=%d result=%d\n",
                calls, dibit ? *dibit : -1, result);
    calls++;
    return result;
}
