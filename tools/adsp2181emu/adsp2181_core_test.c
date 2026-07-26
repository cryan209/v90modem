// license:BSD-3-Clause
#include "adsp2181_core.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    adsp2181_t *cpu = adsp2181_create();
    assert(cpu);

    adsp2181_idma_addr_write(cpu, 0x0123);
    adsp2181_idma_data_write(cpu, 0xabcd);
    adsp2181_idma_data_write(cpu, 0x00ef);
    assert(adsp2181_pm(cpu)[0x0123] == 0xabcdef);
    assert(adsp2181_idma_addr_read(cpu) == 0x0124);

    adsp2181_idma_addr_write(cpu, 0x4005);
    adsp2181_idma_data_write(cpu, 0x1234);
    assert(adsp2181_dm(cpu)[5] == 0x1234);
    adsp2181_idma_addr_write(cpu, 0x4005);
    assert(adsp2181_idma_data_read(cpu) == 0x1234);

    assert(adsp2181_pm_overlay(cpu, 1));
    assert(adsp2181_pm_overlay(cpu, 2));
    assert(adsp2181_dm_overlay(cpu, 1));
    assert(adsp2181_dm_overlay(cpu, 2));
    assert(!adsp2181_pm_overlay(cpu, 0));

    adsp2181_destroy(cpu);
    puts("adsp2181_core_test: PASS");
    return 0;
}
