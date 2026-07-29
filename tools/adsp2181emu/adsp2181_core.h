// license:BSD-3-Clause
// Based on MAME's ADSP-21xx core, copyright Aaron Giles.
#ifndef EICON_ADSP2181_CORE_H
#define EICON_ADSP2181_CORE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct adsp2181 adsp2181_t;
typedef int32_t (*adsp2181_rx_cb)(adsp2181_t *, int port);
typedef void (*adsp2181_tx_cb)(adsp2181_t *, int port, int32_t value);
typedef void (*adsp2181_timer_cb)(adsp2181_t *, int enabled);

enum {
    ADSP2181_IRQ0 = 0,
    ADSP2181_SPORT1_RX = 0,
    ADSP2181_IRQ1 = 1,
    ADSP2181_SPORT1_TX = 1,
    ADSP2181_IRQ2 = 2,
    ADSP2181_SPORT0_RX = 3,
    ADSP2181_SPORT0_TX = 4,
    ADSP2181_TIMER = 5,
    ADSP2181_IRQE = 6,
    ADSP2181_IRQL1 = 7,
    ADSP2181_IRQL2 = 8,
    ADSP2181_IRQ_COUNT = 9
};

adsp2181_t *adsp2181_create(void);
void adsp2181_destroy(adsp2181_t *cpu);
void adsp2181_reset(adsp2181_t *cpu);
int adsp2181_run(adsp2181_t *cpu, int cycles);
uint16_t adsp2181_sport0_tdm_frame(adsp2181_t *cpu, int active_slot,
                                   int dispatch_slot, uint16_t active_word,
                                   uint16_t idle_word, int cycles_per_slot);
/* Selected-channel SPORT frame followed, when the ISR yielded, by the modem
 * task's no-host continuation. Combining these avoids three FFI crossings per
 * 8 kHz sample without changing either execution budget or sample count. */
uint16_t adsp2181_modem_sample(adsp2181_t *cpu, uint16_t active_word,
                               uint16_t idle_word, int cycles_per_pass,
                               uint16_t continuation, uint16_t return_pc);
/* Run the resident firmware G.711 encoder for a block in one host call. This
 * models the hardware SPORT compander without changing sample accounting. */
int adsp2181_g711_encode_block(adsp2181_t *cpu, const int16_t *samples,
                               uint8_t *codes, size_t count,
                               uint16_t entry, uint16_t return_pc,
                               int cycles_per_sample);
/* IDMA boot hold (BMODE=1, MMAP=0): the core executes nothing until an IDMA
 * write commits program memory location 0, then starts at PM 0. */
void adsp2181_set_idma_boot_hold(adsp2181_t *cpu, int on);
int adsp2181_idma_boot_held(const adsp2181_t *cpu);
void adsp2181_set_callbacks(adsp2181_t *cpu, adsp2181_rx_cb rx,
                            adsp2181_tx_cb tx, adsp2181_timer_cb timer);
uint32_t *adsp2181_pm(adsp2181_t *cpu);
uint16_t *adsp2181_dm(adsp2181_t *cpu);
uint16_t *adsp2181_io(adsp2181_t *cpu);
uint32_t *adsp2181_pm_overlay(adsp2181_t *cpu, int overlay);
uint16_t *adsp2181_dm_overlay(adsp2181_t *cpu, int overlay);
void adsp2181_idma_addr_write(adsp2181_t *cpu, uint16_t address);
uint16_t adsp2181_idma_addr_read(const adsp2181_t *cpu);
void adsp2181_idma_data_write(adsp2181_t *cpu, uint16_t value);
uint16_t adsp2181_idma_data_read(adsp2181_t *cpu);
void adsp2181_host_write(adsp2181_t *cpu, uint16_t addr, uint16_t value);
uint16_t adsp2181_host_read(adsp2181_t *cpu, uint16_t addr);
void adsp2181_watch_dm(adsp2181_t *cpu, uint16_t addr, int on);
void adsp2181_watch_pm(adsp2181_t *cpu, uint16_t addr, int on);
void adsp2181_watch_exec(adsp2181_t *cpu, uint16_t addr, int on);
void adsp2181_watch_irqs(adsp2181_t *cpu, int on);
int adsp2181_sport0_tx_written(const adsp2181_t *cpu);
uint16_t adsp2181_pmovlay(const adsp2181_t *cpu);
uint16_t adsp2181_dmovlay(const adsp2181_t *cpu);
uint32_t adsp2181_read_pm(adsp2181_t *cpu, uint16_t addr);
uint64_t adsp2181_cycles(const adsp2181_t *cpu);
/* Execution coverage used to reduce firmware opcode audits to instructions
 * actually reached by a replay. Counts are keyed by resident PM address. */
void adsp2181_coverage_clear(adsp2181_t *cpu);
uint64_t adsp2181_coverage_count(const adsp2181_t *cpu, uint16_t pc);
void adsp2181_trace_budget(adsp2181_t *cpu, int64_t n);
uint16_t adsp2181_pc(const adsp2181_t *cpu);
void adsp2181_set_pc(adsp2181_t *cpu, uint16_t pc);
void adsp2181_call(adsp2181_t *cpu, uint16_t entry, uint16_t return_pc);
void adsp2181_set_irq(adsp2181_t *cpu, int irq, int asserted);
uint16_t adsp2181_imask(const adsp2181_t *cpu);
void adsp2181_set_imask(adsp2181_t *cpu, uint16_t imask);
void adsp2181_set_flagin(adsp2181_t *cpu, int asserted);
int adsp2181_flagin(const adsp2181_t *cpu);
uint16_t adsp2181_icntl(const adsp2181_t *cpu);
int adsp2181_idle(const adsp2181_t *cpu);
void adsp2181_set_ar(adsp2181_t *cpu, uint16_t value);
uint16_t adsp2181_sr0(const adsp2181_t *cpu);
uint16_t adsp2181_sr1(const adsp2181_t *cpu);

#ifdef __cplusplus
}
#endif
#endif
