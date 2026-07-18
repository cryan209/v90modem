#include <stdio.h>
#include <string.h>

extern void parameters_ctor(void *, void *)
    __asm__("_ZN13V90ParametersC1EP19_tagModemParameters");
extern void parameters_set_default(void *)
    __asm__("_ZN13V90Parameters12setToDefaultEv");
extern void parameters_init_session(void *)
    __asm__("_ZN13V90Parameters11initSessionEv");

extern void jd_ctor(void *, void *)
    __asm__("_ZN5V90JdC1EP13V90Parameters");
extern void jd_set_rates(void *, int)
    __asm__("_ZN5V90Jd12setRatesMaskEi");
extern void jd_set_constellation(void *, unsigned char, unsigned char)
    __asm__("_ZN5V90Jd14setConstelSizeEhh");
extern void jd_set_lookahead(void *, unsigned char)
    __asm__("_ZN5V90Jd15setMaxLookaheadEh");
extern void jd_reset_crc(void *)
    __asm__("_ZN5V90Jd8resetCrcEv");
extern void jd_pack(void *)
    __asm__("_ZN5V90Jd8packDataEv");
extern unsigned char *jd_get_bits(void *)
    __asm__("_ZN5V90Jd12getBitVectorEv");

extern void phase3_ctor(void *, void *, unsigned int)
    __asm__("_ZN18V90Phase3ModulatorC1EP13V90Parametersj");
extern void phase3_reset(void *, int, unsigned char, int, unsigned int,
                         void *, void *, const void *, unsigned int)
    __asm__("_ZN18V90Phase3Modulator5resetE7PcmTypeh20Phase3ModulatorStatejP5V90JdP5V92JdPK19tagV90DILdescriptorj");
extern int phase3_generate_trn1d(void *)
    __asm__("_ZN18V90Phase3Modulator13generateTRN1dEv");
extern int phase3_generate_jd(void *)
    __asm__("_ZN18V90Phase3Modulator10generateJdEv");
extern int phase3_generate_jd_not(void *)
    __asm__("_ZN18V90Phase3Modulator13generateJdNotEv");

int main(void)
{
    unsigned char modem_parameters_storage[65536] __attribute__((aligned(16)));
    unsigned char parameters_storage[65536] __attribute__((aligned(16)));
    unsigned char jd_storage[65536] __attribute__((aligned(16)));
    unsigned char phase3_storage[65536] __attribute__((aligned(16)));
    unsigned char *bits;
    int i;

    memset(modem_parameters_storage, 0, sizeof(modem_parameters_storage));
    memset(parameters_storage, 0, sizeof(parameters_storage));
    memset(jd_storage, 0, sizeof(jd_storage));
    memset(phase3_storage, 0, sizeof(phase3_storage));

    parameters_ctor(parameters_storage, modem_parameters_storage);
    parameters_set_default(parameters_storage);
    parameters_init_session(parameters_storage);

    jd_ctor(jd_storage, parameters_storage);
    jd_set_rates(jd_storage, 0x7fffff);
    jd_set_constellation(jd_storage, 0, 0);
    jd_set_lookahead(jd_storage, 1);
    jd_reset_crc(jd_storage);
    jd_pack(jd_storage);

    bits = jd_get_bits(jd_storage);
    for (i = 0; i < 72; ++i)
        putchar(bits[i] ? '1' : '0');
    putchar('\n');

    phase3_ctor(phase3_storage, parameters_storage, 0);
    phase3_reset(phase3_storage, 0, 78, 0, 0,
                 jd_storage, NULL, NULL, 0);

    for (i = 0; i < 2040; ++i)
        (void) phase3_generate_trn1d(phase3_storage);

    for (i = 0; i < 72; ++i) {
        *(int *)(phase3_storage + 0x18) = i + 1;
        putchar(phase3_generate_jd(phase3_storage) > 0 ? '1' : '0');
    }
    putchar('\n');

    for (i = 0; i < 12; ++i)
        putchar(phase3_generate_jd_not(phase3_storage) > 0 ? '1' : '0');
    putchar('\n');
    return 0;
}
