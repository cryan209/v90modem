// license:BSD-3-Clause
#include "adsp2181_core.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    adsp2181_t *cpu = adsp2181_create();
    assert(cpu);

    /* IDMA bit 0x4000 is the ADSP-2181 destination type: clear selects
     * 24-bit PM (two data accesses per word), set selects 16-bit DM (one).
     * Matches the te_dmlt.pm host helpers 0x80082950 / 0x80082920, which
     * use the two-access form for addresses below 0x4000. */
    adsp2181_idma_addr_write(cpu, 0x0123);
    adsp2181_idma_data_write(cpu, 0xabcd);
    adsp2181_idma_data_write(cpu, 0x00ef);
    assert(adsp2181_pm(cpu)[0x0123] == 0xabcdef);
    assert(adsp2181_idma_addr_read(cpu) == 0x0124);

    adsp2181_idma_addr_write(cpu, 0x4005);
    adsp2181_idma_data_write(cpu, 0x1234);
    assert(adsp2181_dm(cpu)[5] == 0x1234);
    assert(adsp2181_idma_addr_read(cpu) == 0x4006);
    adsp2181_idma_addr_write(cpu, 0x4005);
    assert(adsp2181_idma_data_read(cpu) == 0x1234);

    /* A DM block transfer keeps auto-incrementing from one address latch,
     * which is what the bulk helper 0x80082a38 relies on. */
    adsp2181_idma_addr_write(cpu, 0x4100);
    adsp2181_idma_data_write(cpu, 0x1111);
    adsp2181_idma_data_write(cpu, 0x2222);
    adsp2181_idma_data_write(cpu, 0x3333);
    assert(adsp2181_dm(cpu)[0x100] == 0x1111);
    assert(adsp2181_dm(cpu)[0x101] == 0x2222);
    assert(adsp2181_dm(cpu)[0x102] == 0x3333);

    /* A dangling PM read half must not write anything back on the next
     * address latch (only a pending write half commits). */
    adsp2181_idma_addr_write(cpu, 0x0123);
    (void)adsp2181_idma_data_read(cpu);
    adsp2181_idma_addr_write(cpu, 0x4000);
    assert(adsp2181_pm(cpu)[0x0123] == 0xabcdef);

    /* Whole-word host helper: PM words land as value<<8, DM words verbatim. */
    adsp2181_host_write(cpu, 0x3331, 0x9a0d);
    assert(adsp2181_pm(cpu)[0x3331] == 0x9a0d00);
    adsp2181_host_write(cpu, 0x7310, 0x0042);
    assert(adsp2181_dm(cpu)[0x3310] == 0x0042);
    assert(adsp2181_host_read(cpu, 0x3331) == 0x9a0d);
    assert(adsp2181_host_read(cpu, 0x7310) == 0x0042);

    assert(adsp2181_pm_overlay(cpu, 1));
    assert(adsp2181_pm_overlay(cpu, 2));
    assert(adsp2181_dm_overlay(cpu, 1));
    assert(adsp2181_dm_overlay(cpu, 2));
    assert(!adsp2181_pm_overlay(cpu, 0));

    adsp2181_destroy(cpu);
    puts("adsp2181_core_test: PASS");
    return 0;
}
