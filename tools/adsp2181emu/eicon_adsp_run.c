// license:GPL-2.0-or-later
#include "adsp2181_core.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint64_t rx_reads[2];
static uint64_t tx_writes[2];
static int16_t last_tx[2];
static FILE *rx_files[2];
static FILE *tx_files[2];
static int loopback_sport[2];
static int trace_sport;

static int32_t sport_rx(adsp2181_t *cpu, int port)
{
    (void)cpu;
    if ((unsigned)port < 2) {
        unsigned char sample[2];
        int16_t value = loopback_sport[port] ? last_tx[port] : 0;
        rx_reads[port]++;
        if (rx_files[port] && fread(sample, 1, sizeof(sample), rx_files[port]) == sizeof(sample))
            value = (int16_t)(sample[0] | ((unsigned)sample[1] << 8));
        if (trace_sport && rx_reads[port] <= (uint64_t)trace_sport)
            fprintf(stderr, "[ADSP] sport%d rx[%" PRIu64 "]=%d\n", port,
                    rx_reads[port] - 1, value);
        return value;
    }
    return 0;
}

static void sport_tx(adsp2181_t *cpu, int port, int32_t value)
{
    (void)cpu;
    if ((unsigned)port < 2) {
        unsigned char sample[2];
        tx_writes[port]++;
        last_tx[port] = (int16_t)value;
        sample[0] = (unsigned char)value;
        sample[1] = (unsigned char)((unsigned)value >> 8);
        if (tx_files[port])
            (void)fwrite(sample, 1, sizeof(sample), tx_files[port]);
        if (trace_sport && tx_writes[port] <= (uint64_t)trace_sport)
            fprintf(stderr, "[ADSP] sport%d tx[%" PRIu64 "]=%d\n", port,
                    tx_writes[port] - 1, (int16_t)value);
    }
}

static void timer_changed(adsp2181_t *cpu, int enabled)
{
    (void)cpu;
    fprintf(stderr, "[ADSP] timer %s\n", enabled ? "enabled" : "disabled");
}

static int load_pm(const char *path, uint32_t *pm)
{
    FILE *fp = fopen(path, "rb");
    unsigned char word[3];
    if (!fp) {
        fprintf(stderr, "%s: %s\n", path, strerror(errno));
        return -1;
    }
    for (unsigned address = 0; address < 0x10000; address++) {
        if (fread(word, 1, sizeof(word), fp) != sizeof(word)) {
            fprintf(stderr, "%s: expected 65536 packed 24-bit words\n", path);
            fclose(fp);
            return -1;
        }
        if (address < 0x4000)
            pm[address] = word[0] | ((uint32_t)word[1] << 8) | ((uint32_t)word[2] << 16);
    }
    if (fgetc(fp) != EOF) {
        fprintf(stderr, "%s: trailing data\n", path);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

static int load_word_map(const char *path, uint32_t *pm, uint16_t *dm)
{
    FILE *fp = fopen(path, "r");
    unsigned address, value;
    int fields;
    if (!fp) {
        fprintf(stderr, "%s: %s\n", path, strerror(errno));
        return -1;
    }
    while ((fields = fscanf(fp, "%x %x", &address, &value)) == 2) {
        if (address >= 0x10000 || (pm && value > 0xffffff) || (dm && value > 0xffff)) {
            fprintf(stderr, "%s: word out of range\n", path);
            fclose(fp);
            return -1;
        }
        if (pm) pm[address] = value;
        else dm[address] = (uint16_t)value;
    }
    if (fields != EOF) {
        fprintf(stderr, "%s: malformed word map\n", path);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

static int load_dm(const char *path, uint16_t *dm)
{
    FILE *fp = fopen(path, "rb");
    unsigned char word[2];
    if (!fp) {
        fprintf(stderr, "%s: %s\n", path, strerror(errno));
        return -1;
    }
    for (unsigned address = 0; address < 0x10000; address++) {
        if (fread(word, 1, sizeof(word), fp) != sizeof(word)) {
            fprintf(stderr, "%s: expected 65536 little-endian 16-bit words\n", path);
            fclose(fp);
            return -1;
        }
        if (address < 0x4000)
            dm[address] = word[0] | ((uint16_t)word[1] << 8);
    }
    if (fgetc(fp) != EOF) {
        fprintf(stderr, "%s: trailing data\n", path);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

int main(int argc, char **argv)
{
    adsp2181_t *cpu;
    long cycles = 100000;
    char *end;
    int ran;
    int trace = 0;

    if (getenv("ADSP_TRACE"))
        trace = atoi(getenv("ADSP_TRACE"));
    if (getenv("ADSP_TRACE_SPORT"))
        trace_sport = atoi(getenv("ADSP_TRACE_SPORT"));
    loopback_sport[0] = getenv("ADSP_LOOPBACK_SPORT0") != NULL;
    loopback_sport[1] = getenv("ADSP_LOOPBACK_SPORT1") != NULL;
    for (int port = 0; port < 2; port++) {
        char name[16];
        const char *path;
        snprintf(name, sizeof(name), "ADSP_RX%d", port);
        if ((path = getenv(name)) != NULL && !(rx_files[port] = fopen(path, "rb"))) {
            fprintf(stderr, "%s: %s\n", path, strerror(errno));
            return 1;
        }
        snprintf(name, sizeof(name), "ADSP_TX%d", port);
        if ((path = getenv(name)) != NULL && !(tx_files[port] = fopen(path, "wb"))) {
            fprintf(stderr, "%s: %s\n", path, strerror(errno));
            return 1;
        }
    }
    if (argc < 3 || argc > 4) {
        fprintf(stderr, "Usage: %s <pm.bin> <dm.bin> [cycles]\n", argv[0]);
        return 2;
    }
    if (argc == 4) {
        cycles = strtol(argv[3], &end, 0);
        if (*end || cycles <= 0 || cycles > 1000000000L) {
            fprintf(stderr, "invalid cycle count: %s\n", argv[3]);
            return 2;
        }
    }

    cpu = adsp2181_create();
    if (!cpu) {
        fprintf(stderr, "cannot allocate ADSP-2181 core\n");
        return 1;
    }
    if (load_pm(argv[1], adsp2181_pm(cpu)) || load_dm(argv[2], adsp2181_dm(cpu))) {
        adsp2181_destroy(cpu);
        return 1;
    }
    adsp2181_set_callbacks(cpu, sport_rx, sport_tx, timer_changed);
    adsp2181_reset(cpu);
    if (getenv("ADSP_START_PC")) {
        unsigned long pc = strtoul(getenv("ADSP_START_PC"), &end, 0);
        if (*end || pc > 0x3fff) {
            fprintf(stderr, "invalid ADSP_START_PC: %s\n", getenv("ADSP_START_PC"));
            adsp2181_destroy(cpu);
            return 2;
        }
        adsp2181_set_pc(cpu, (uint16_t)pc);
    }
    fprintf(stderr, "[ADSP] reset pc=%04x op=%06x cycles=%ld\n",
            adsp2181_pc(cpu), adsp2181_pm(cpu)[adsp2181_pc(cpu)] & 0xffffff, cycles);
    if (getenv("ADSP_HOST_WORDS")) {
        long words = strtol(getenv("ADSP_HOST_WORDS"), &end, 0);
        if (*end || words < 0 || words > 1000000) {
            fprintf(stderr, "invalid ADSP_HOST_WORDS: %s\n", getenv("ADSP_HOST_WORDS"));
            adsp2181_destroy(cpu);
            return 2;
        }
        (void)adsp2181_run(cpu, 1000);
        if (getenv("ADSP_STAGE_PM_WORDS") &&
            load_word_map(getenv("ADSP_STAGE_PM_WORDS"), adsp2181_pm(cpu), NULL)) {
            adsp2181_destroy(cpu);
            return 1;
        }
        if (getenv("ADSP_STAGE_DM_WORDS") &&
            load_word_map(getenv("ADSP_STAGE_DM_WORDS"), NULL, adsp2181_dm(cpu))) {
            adsp2181_destroy(cpu);
            return 1;
        }
        if (getenv("ADSP_STAGE_ENTRY")) {
            unsigned long entry = strtoul(getenv("ADSP_STAGE_ENTRY"), &end, 0);
            if (*end || entry > 0x3fff) {
                fprintf(stderr, "invalid ADSP_STAGE_ENTRY: %s\n", getenv("ADSP_STAGE_ENTRY"));
                adsp2181_destroy(cpu);
                return 2;
            }
            adsp2181_call(cpu, (uint16_t)entry, 0x02a8);
            (void)adsp2181_run(cpu, 1000000);
            fprintf(stderr, "[ADSP] stage-init pc=%04x idle=%d imask=%03x icntl=%02x\n",
                    adsp2181_pc(cpu), adsp2181_idle(cpu), adsp2181_imask(cpu),
                    adsp2181_icntl(cpu));
        }
        if (getenv("ADSP_STAGE_ENTRY2")) {
            unsigned long entry = strtoul(getenv("ADSP_STAGE_ENTRY2"), &end, 0);
            if (*end || entry > 0x3fff) {
                fprintf(stderr, "invalid ADSP_STAGE_ENTRY2: %s\n", getenv("ADSP_STAGE_ENTRY2"));
                adsp2181_destroy(cpu);
                return 2;
            }
            adsp2181_call(cpu, (uint16_t)entry, 0x02a8);
            (void)adsp2181_run(cpu, 1000000);
            fprintf(stderr, "[ADSP] stage-init2 pc=%04x idle=%d imask=%03x icntl=%02x\n",
                    adsp2181_pc(cpu), adsp2181_idle(cpu), adsp2181_imask(cpu),
                    adsp2181_icntl(cpu));
        }
        fprintf(stderr, "[ADSP] host-start pc=%04x idle=%d imask=%03x icntl=%02x words=%ld staged=%s\n",
                adsp2181_pc(cpu), adsp2181_idle(cpu), adsp2181_imask(cpu),
                adsp2181_icntl(cpu), words,
                getenv("ADSP_STAGE_PM_WORDS") ? "yes" : "no");
        for (long word = 0; word < words; word++) {
            adsp2181_set_irq(cpu, ADSP2181_SPORT0_RX, 1);
            adsp2181_set_irq(cpu, ADSP2181_SPORT0_RX, 0);
            (void)adsp2181_run(cpu, 10000);
            if (trace_sport)
                fprintf(stderr, "[ADSP] host[%ld] pc=%04x idle=%d imask=%03x icntl=%02x\n",
                        word, adsp2181_pc(cpu), adsp2181_idle(cpu),
                        adsp2181_imask(cpu), adsp2181_icntl(cpu));
        }
    } else if (getenv("ADSP_WAKE_IRQ")) {
        long irq = strtol(getenv("ADSP_WAKE_IRQ"), &end, 0);
        if (*end || irq < 0 || irq >= ADSP2181_IRQ_COUNT) {
            fprintf(stderr, "invalid ADSP_WAKE_IRQ: %s\n", getenv("ADSP_WAKE_IRQ"));
            adsp2181_destroy(cpu);
            return 2;
        }
        (void)adsp2181_run(cpu, 1000);
        fprintf(stderr, "[ADSP] pre-wake pc=%04x idle=%d imask=%03x icntl=%02x irq=%ld\n",
                adsp2181_pc(cpu), adsp2181_idle(cpu), adsp2181_imask(cpu),
                adsp2181_icntl(cpu), irq);
        adsp2181_set_irq(cpu, (int)irq, 1);
        adsp2181_set_irq(cpu, (int)irq, 0);
    }
    if (trace > 0) {
        int limit = trace < cycles ? trace : (int)cycles;
        for (ran = 0; ran < limit; ran++) {
            uint16_t pc = adsp2181_pc(cpu);
            fprintf(stderr, "[ADSP] trace cycle=%d pc=%04x op=%06x\n",
                    ran, pc, adsp2181_pm(cpu)[pc] & 0xffffff);
            (void)adsp2181_run(cpu, 1);
        }
        if (ran < cycles)
            ran += adsp2181_run(cpu, (int)cycles - ran);
    } else {
        ran = adsp2181_run(cpu, (int)cycles);
    }
    fprintf(stderr,
            "[ADSP] stopped ran=%d pc=%04x op=%06x idle=%d "
            "sport-rx=%" PRIu64 "/%" PRIu64 " sport-tx=%" PRIu64 "/%" PRIu64
            " last-tx=%d/%d\n",
            ran, adsp2181_pc(cpu), adsp2181_pm(cpu)[adsp2181_pc(cpu)] & 0xffffff,
            adsp2181_idle(cpu), rx_reads[0], rx_reads[1], tx_writes[0], tx_writes[1],
            last_tx[0], last_tx[1]);
    if (getenv("ADSP_DUMP_DM")) {
        FILE *dump = fopen(getenv("ADSP_DUMP_DM"), "wb");
        if (dump) {
            for (unsigned address = 0; address < 0x4000; address++) {
                uint16_t value = adsp2181_dm(cpu)[address];
                unsigned char bytes[2] = {(unsigned char)value, (unsigned char)(value >> 8)};
                (void)fwrite(bytes, 1, sizeof(bytes), dump);
            }
            fclose(dump);
        }
    }
    adsp2181_destroy(cpu);
    for (int port = 0; port < 2; port++) {
        if (rx_files[port]) fclose(rx_files[port]);
        if (tx_files[port]) fclose(tx_files[port]);
    }
    return 0;
}
