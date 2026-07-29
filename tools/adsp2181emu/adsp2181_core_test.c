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

    /* ADSP-218x Instruction Set Reference §4-117 permits RXn/TXn on both
     * sides of a register-to-register MOVE.  The Eicon kernel relies on
     * RX1=SR0 and SR0=TX1 in its SPORT ISR. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x41234f; /* SR1 = 0x1234 */
    adsp2181_pm(cpu)[1] = 0x0d0caf; /* RX1 = SR1 */
    adsp2181_pm(cpu)[2] = 0x40000f; /* SR1 = 0 */
    adsp2181_pm(cpu)[3] = 0x0d03fa; /* SR1 = RX1 */
    adsp2181_pm(cpu)[4] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 16);
    assert(adsp2181_sr1(cpu) == 0x1234);

    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x45678f; /* SR1 = 0x5678 */
    adsp2181_pm(cpu)[1] = 0x0d0cbf; /* TX1 = SR1 */
    adsp2181_pm(cpu)[2] = 0x40000f; /* SR1 = 0 */
    adsp2181_pm(cpu)[3] = 0x0d03fb; /* SR1 = TX1 */
    adsp2181_pm(cpu)[4] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 16);
    assert(adsp2181_sr1(cpu) == 0x5678);

    /* Parallel sources are read before destinations are committed.  INFO's
     * detector uses this exact instruction at PM 0x25fc: the shift replaces
     * SR while AR must receive the preceding SR1 accumulator. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x41234f; /* SR1 = 0x1234 */
    adsp2181_pm(cpu)[1] = 0x1013af; /* SR = LSHIFT MR0 (LO), AR = SR1 */
    adsp2181_pm(cpu)[2] = 0x90100a; /* DM(0x0100) = AR */
    adsp2181_pm(cpu)[3] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 16);
    assert(adsp2181_dm(cpu)[0x0100] == 0x1234);

    /* INFO PM 0x33ae..0x33bc uses SUBTRACT followed by ABS and IF LE as an
     * equality test for its detector-count state transitions. */
    adsp2181_reset(cpu);
    adsp2181_dm(cpu)[0x06e6] = 3;
    adsp2181_pm(cpu)[0] = 0x1c010f; /* CALL 0x0010 */
    adsp2181_pm(cpu)[1] = 0x180043; /* IF LE JUMP 0x0004 */
    adsp2181_pm(cpu)[2] = 0x22180f; /* AR = 0 */
    adsp2181_pm(cpu)[3] = 0x18005f; /* JUMP 0x0005 */
    adsp2181_pm(cpu)[4] = 0x22380f; /* AR = 1 */
    adsp2181_pm(cpu)[5] = 0x90100a; /* DM(0x0100) = AR */
    adsp2181_pm(cpu)[6] = 0x028000; /* IDLE */
    adsp2181_pm(cpu)[0x10] = 0x400034; /* AY0 = 3 */
    adsp2181_pm(cpu)[0x11] = 0x806e60; /* AX0 = DM(0x06e6) */
    adsp2181_pm(cpu)[0x12] = 0x23200f; /* AR = AY0 - AX0 */
    adsp2181_pm(cpu)[0x13] = 0x23e20f; /* AR = ABS AR */
    adsp2181_pm(cpu)[0x14] = 0x0a000f; /* RTS */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] == 1);

    adsp2181_destroy(cpu);
    puts("adsp2181_core_test: PASS");
    return 0;
}
