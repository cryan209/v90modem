// license:BSD-3-Clause
#include "adsp2181_core.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    adsp2181_t *cpu = adsp2181_create();
    assert(cpu);

    /* IDMA bit 0x4000 set selects 24-bit PM (two data writes per word),
     * clear selects 16-bit DM -- matching the te_dmlt.pm host helper. */
    adsp2181_idma_addr_write(cpu, 0x4123);
    adsp2181_idma_data_write(cpu, 0xabcd);
    adsp2181_idma_data_write(cpu, 0x00ef);
    assert(adsp2181_pm(cpu)[0x0123] == 0xabcdef);
    assert(adsp2181_idma_addr_read(cpu) == 0x4124);

    adsp2181_idma_addr_write(cpu, 0x0005);
    adsp2181_idma_data_write(cpu, 0x1234);
    assert(adsp2181_dm(cpu)[5] == 0x1234);
    adsp2181_idma_addr_write(cpu, 0x0005);
    assert(adsp2181_idma_data_read(cpu) == 0x1234);

    /* Single-word host helper: PM words land as value<<8. */
    adsp2181_host_write(cpu, 0x7331, 0x9a0d);
    assert(adsp2181_pm(cpu)[0x3331] == 0x9a0d00);
    adsp2181_host_write(cpu, 0x3310, 0x0042);
    assert(adsp2181_dm(cpu)[0x3310] == 0x0042);
    assert(adsp2181_host_read(cpu, 0x7331) == 0x9a0d);
    assert(adsp2181_host_read(cpu, 0x3310) == 0x0042);

    assert(adsp2181_pm_overlay(cpu, 1));
    assert(adsp2181_pm_overlay(cpu, 2));
    assert(adsp2181_dm_overlay(cpu, 1));
    assert(adsp2181_dm_overlay(cpu, 2));
    assert(!adsp2181_pm_overlay(cpu, 0));

    adsp2181_destroy(cpu);
    puts("adsp2181_core_test: PASS");
    return 0;
}
