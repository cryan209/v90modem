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

#define HOST_SCRIPT_MAX 4096
struct host_poke { long index; uint16_t addr; uint16_t value; };
static struct host_poke host_script[HOST_SCRIPT_MAX];
static int host_script_len;

static int load_host_script(const char *path)
{
    FILE *fp = fopen(path, "r");
    long index;
    unsigned addr, value;
    if (!fp) {
        fprintf(stderr, "%s: %s\n", path, strerror(errno));
        return -1;
    }
    while (host_script_len < HOST_SCRIPT_MAX &&
           fscanf(fp, "%ld %x %x", &index, &addr, &value) == 3) {
        if (addr > 0x7fff || value > 0xffff) {
            fprintf(stderr, "%s: host poke out of range\n", path);
            fclose(fp);
            return -1;
        }
        host_script[host_script_len].index = index;
        host_script[host_script_len].addr = (uint16_t)addr;
        host_script[host_script_len].value = (uint16_t)value;
        host_script_len++;
    }
    fclose(fp);
    fprintf(stderr, "[ADSP] host-script: %d pokes loaded\n", host_script_len);
    return 0;
}

#define HOST_POLL_MAX 64
static uint16_t host_poll_addr[HOST_POLL_MAX];
static uint16_t host_poll_last[HOST_POLL_MAX];
static int host_poll_len;

static int parse_addr_list(const char *text, uint16_t *out, int max)
{
    int count = 0;
    char *copy = strdup(text), *tok, *save = NULL;
    if (!copy) return -1;
    for (tok = strtok_r(copy, ",", &save); tok && count < max;
         tok = strtok_r(NULL, ",", &save)) {
        char *end;
        unsigned long addr = strtoul(tok, &end, 0);
        if (*end || addr > 0x3fff) { free(copy); return -1; }
        out[count++] = (uint16_t)addr;
    }
    free(copy);
    return count;
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
    {
        const char *vars[2] = { getenv("ADSP_WATCH_DM"), getenv("ADSP_WATCH_PM") };
        for (int which = 0; which < 2; which++) {
            uint16_t addrs[256];
            int n;
            if (!vars[which]) continue;
            n = parse_addr_list(vars[which], addrs, 256);
            if (n < 0) {
                fprintf(stderr, "invalid ADSP_WATCH_%s list\n", which ? "PM" : "DM");
                adsp2181_destroy(cpu);
                return 2;
            }
            for (int i = 0; i < n; i++)
                which ? adsp2181_watch_pm(cpu, addrs[i], 1)
                      : adsp2181_watch_dm(cpu, addrs[i], 1);
        }
    }
    if (getenv("ADSP_HOST_SCRIPT") &&
        load_host_script(getenv("ADSP_HOST_SCRIPT"))) {
        adsp2181_destroy(cpu);
        return 1;
    }
    if (getenv("ADSP_HOST_POLL")) {
        host_poll_len = parse_addr_list(getenv("ADSP_HOST_POLL"),
                                        host_poll_addr, HOST_POLL_MAX);
        if (host_poll_len < 0) {
            fprintf(stderr, "invalid ADSP_HOST_POLL list\n");
            adsp2181_destroy(cpu);
            return 2;
        }
        for (int i = 0; i < host_poll_len; i++)
            host_poll_last[i] = 0xeeee;
    }
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
        long entry2_at = -1;
        if (getenv("ADSP_STAGE_ENTRY2_AT"))
            entry2_at = strtol(getenv("ADSP_STAGE_ENTRY2_AT"), NULL, 0);
        if (getenv("ADSP_STAGE_ENTRY2") && entry2_at < 0) {
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
        if (getenv("ADSP_STAGE_ENTRY2"))
            entry2_at = entry2_at; /* keep */
        /* MIPS dsp_assign writes occur after the task initializer has built its
         * mailbox/database structures, so keep this separate from staged DM. */
        if (getenv("ADSP_POST_DM_WORDS") &&
            load_word_map(getenv("ADSP_POST_DM_WORDS"), NULL, adsp2181_dm(cpu))) {
            adsp2181_destroy(cpu);
            return 1;
        }
        long host_irq = -1;
        if (getenv("ADSP_HOST_IRQ")) {
            host_irq = strtol(getenv("ADSP_HOST_IRQ"), &end, 0);
            if (*end || host_irq < 0 || host_irq >= ADSP2181_IRQ_COUNT) {
                fprintf(stderr, "invalid ADSP_HOST_IRQ: %s\n", getenv("ADSP_HOST_IRQ"));
                adsp2181_destroy(cpu);
                return 2;
            }
        }
        fprintf(stderr, "[ADSP] host-start pc=%04x idle=%d imask=%03x icntl=%02x words=%ld staged=%s\n",
                adsp2181_pc(cpu), adsp2181_idle(cpu), adsp2181_imask(cpu),
                adsp2181_icntl(cpu), words,
                getenv("ADSP_STAGE_PM_WORDS") ? "yes" : "no");
        long force_imask = 0;
        if (getenv("ADSP_FORCE_IMASK"))
            force_imask = strtol(getenv("ADSP_FORCE_IMASK"), NULL, 0);
        if (getenv("ADSP_WATCH_ALL"))
            for (int a = 0; a < 0x4000; a++) {
                adsp2181_watch_dm(cpu, (uint16_t)a, 1);
                adsp2181_watch_pm(cpu, (uint16_t)a, 1);
            }
        long trace_at = getenv("ADSP_TRACE_AT_WORD")
            ? strtol(getenv("ADSP_TRACE_AT_WORD"), NULL, 0) : 0;
        if (getenv("ADSP_TRACE_HOST") && trace_at == 0)
            adsp2181_trace_budget(cpu, strtol(getenv("ADSP_TRACE_HOST"), NULL, 0));
        for (long word = 0; word < words; word++) {
            if (getenv("ADSP_TRACE_HOST") && word == trace_at && trace_at > 0)
                adsp2181_trace_budget(cpu, strtol(getenv("ADSP_TRACE_HOST"), NULL, 0));
            if (entry2_at == word && getenv("ADSP_STAGE_ENTRY2")) {
                unsigned long entry = strtoul(getenv("ADSP_STAGE_ENTRY2"), NULL, 16);
                adsp2181_call(cpu, (uint16_t)entry, 0x02a8);
                (void)adsp2181_run(cpu, 1000000);
                fprintf(stderr, "[ADSP] stage-init2(at %ld) pc=%04x idle=%d imask=%03x icntl=%02x\n",
                        word, adsp2181_pc(cpu), adsp2181_idle(cpu),
                        adsp2181_imask(cpu), adsp2181_icntl(cpu));
                entry2_at = -2; /* once */
            }
            for (int i = 0; i < host_script_len; i++)
                if (host_script[i].index == word) {
                    adsp2181_host_write(cpu, host_script[i].addr,
                                        host_script[i].value);
                    if (trace_sport)
                        fprintf(stderr, "[ADSP] host-poke[%ld] %04x=%04x\n",
                                word, host_script[i].addr, host_script[i].value);
                }
            adsp2181_set_irq(cpu, ADSP2181_SPORT0_RX, 1);
            adsp2181_set_irq(cpu, ADSP2181_SPORT0_RX, 0);
            if (force_imask)
                adsp2181_set_imask(cpu, (uint16_t)(force_imask | adsp2181_imask(cpu)));
            if (host_irq >= 0) {
                /* level-sensitive inputs must stay asserted into the run */
                adsp2181_set_irq(cpu, (int)host_irq, 1);
                (void)adsp2181_run(cpu, 5000);
                /* keep asserted: the first run is spent in the masked
                 * SPORT0 ISR, the second run is where it can vector */
                (void)adsp2181_run(cpu, 5000);
                adsp2181_set_irq(cpu, (int)host_irq, 0);
            } else {
                (void)adsp2181_run(cpu, 10000);
            }
            for (int i = 0; i < host_poll_len; i++) {
                uint16_t now = adsp2181_host_read(cpu, host_poll_addr[i]);
                if (now != host_poll_last[i]) {
                    fprintf(stderr, "[ADSP] host-poll[%ld] %04x: %04x -> %04x\n",
                            word, host_poll_addr[i], host_poll_last[i], now);
                    host_poll_last[i] = now;
                }
            }
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
