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

    /* The same sequence must be false on either side of the target.  ABS
     * takes AZ and AN from its result; leaving the subtract's AN in ASTAT
     * made this an `count >= 3` test, so every INFO detector-count
     * transition fired early and the 0x0c37 profile was never reached. */
    for (int count = 0; count <= 8; count++) {
        if (count == 3) continue;
        adsp2181_dm(cpu)[0x06e6] = count;
        adsp2181_dm(cpu)[0x0100] = 0xffff;
        adsp2181_set_pc(cpu, 0);
        adsp2181_run(cpu, 32);
        assert(adsp2181_dm(cpu)[0x0100] == 0);
    }

    /* ADSP-2100 Family User's Manual §2.3.2.6 rounds the complete
     * accumulator, not just the multiply product. Construct an exact
     * midpoint from MR0=0x4000 plus 0x2000*1 in fractional mode. Unbiased
     * rounding keeps the even MR1 at zero. The old MAME-derived code tested
     * the product low word (0x4000), incorrectly rounding MR1 up to one. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x40000c; /* MR1 = 0 */
    adsp2181_pm(cpu)[1] = 0x44000b; /* MR0 = 0x4000 */
    adsp2181_pm(cpu)[2] = 0x420002; /* MX0 = 0x2000 */
    adsp2181_pm(cpu)[3] = 0x400016; /* MY0 = 1 */
    adsp2181_pm(cpu)[4] = 0x20400f; /* MR = MR + MX0*MY0 (RND) */
    adsp2181_pm(cpu)[5] = 0x90101c; /* DM(0x0101) = MR1 */
    adsp2181_pm(cpu)[6] = 0x90100b; /* DM(0x0100) = MR0 */
    adsp2181_pm(cpu)[7] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0101] == 0);
    assert(adsp2181_dm(cpu)[0x0100] == 0);

    /* The multiply instruction's status table specifies MV for either MR or
     * MF destinations. (-32768 * -32768) in fractional mode overflows the
     * signed result; the old mac_op_mf path never updated MV at all. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x480002; /* MX0 = 0x8000 */
    adsp2181_pm(cpu)[1] = 0x480006; /* MY0 = 0x8000 */
    adsp2181_pm(cpu)[2] = 0x24800f; /* MF = MX0*MY0 (SS) */
    adsp2181_pm(cpu)[3] = 0x0d03a0; /* AR = ASTAT */
    adsp2181_pm(cpu)[4] = 0x90100a; /* DM(0x0100) = AR */
    adsp2181_pm(cpu)[5] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] & 0x0040); /* MV */

    /* I registers remain 14-bit when L=0 disables circular buffering. A
     * linear negative modification must wrap 0 -> 0x3fff, rather than leave
     * the unsigned host intermediate at 0xffffffff. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x340000; /* I0 = 0 */
    adsp2181_pm(cpu)[1] = 0x37fff4; /* M0 = -1 */
    adsp2181_pm(cpu)[2] = 0x340008; /* L0 = 0 */
    adsp2181_pm(cpu)[3] = 0x090000; /* MODIFY(I0,M0) */
    adsp2181_pm(cpu)[4] = 0x0d01a0; /* AR = I0 */
    adsp2181_pm(cpu)[5] = 0x90100a; /* DM(0x0100) = AR */
    adsp2181_pm(cpu)[6] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] == 0x3fff);

    /* Manual §4.2.3 circular formula at base zero: (0 - 1) mod 3 = 2.
     * The previous unsigned comparison instead produced 0x3ffc. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x340000; /* I0 = 0 (base 0) */
    adsp2181_pm(cpu)[1] = 0x37fff4; /* M0 = -1 */
    adsp2181_pm(cpu)[2] = 0x340038; /* L0 = 3 */
    adsp2181_pm(cpu)[3] = 0x090000; /* MODIFY(I0,M0) */
    adsp2181_pm(cpu)[4] = 0x0d01a0; /* AR = I0 */
    adsp2181_pm(cpu)[5] = 0x90100a; /* DM(0x0100) = AR */
    adsp2181_pm(cpu)[6] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] == 2);

    /* INFO uses EXP(HI) followed by NORM(HI/LO) for double-precision
     * normalization. AR=1 derives SE=-14 and normalizes into SR bit 30. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x40001a; /* AR = 1 */
    adsp2181_pm(cpu)[1] = 0x0e620f; /* SE = EXP AR (HI) */
    adsp2181_pm(cpu)[2] = 0x0e420f; /* SR = NORM AR (HI) */
    adsp2181_pm(cpu)[3] = 0x90100f; /* DM(0x0100) = SR1 */
    adsp2181_pm(cpu)[4] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] == 0x4000);

    /* HIX with AV derives SE=+1. NORM(HI) then downshifts and fills bit 31
     * from AC, as specified in the shifter chapter. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x48000a; /* AR = 0x8000 */
    adsp2181_pm(cpu)[1] = 0x3c00c0; /* ASTAT = AV | AC */
    adsp2181_pm(cpu)[2] = 0x0e6a0f; /* SE = EXP AR (HIX) */
    adsp2181_pm(cpu)[3] = 0x0e420f; /* SR = NORM AR (HI) */
    adsp2181_pm(cpu)[4] = 0x90100f; /* DM(0x0100) = SR1 */
    adsp2181_pm(cpu)[5] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] == 0xc000);

    /* The first CNTR load after reset does not push an invalid old count.
     * Five active counts therefore use the current CNTR plus all four count
     * stack entries without setting COUNT_OVER. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x3c0015; /* CNTR = 1 */
    adsp2181_pm(cpu)[1] = 0x3c0025; /* CNTR = 2 */
    adsp2181_pm(cpu)[2] = 0x3c0035; /* CNTR = 3 */
    adsp2181_pm(cpu)[3] = 0x3c0045; /* CNTR = 4 */
    adsp2181_pm(cpu)[4] = 0x3c0055; /* CNTR = 5 */
    adsp2181_pm(cpu)[5] = 0x0d03a2; /* AR = SSTAT */
    adsp2181_pm(cpu)[6] = 0x90100a; /* DM(0x0100) = AR */
    adsp2181_pm(cpu)[7] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(!(adsp2181_dm(cpu)[0x0100] & 0x0008)); /* !COUNT_OVER */
    assert(!(adsp2181_dm(cpu)[0x0100] & 0x0004)); /* !COUNT_EMPTY */

    /* MR is 40 bits even though the host union gives MR2 a 16-bit slot.
     * Overflow maximum-positive MR by two; architectural MR2 is the signed
     * 8-bit value 0x80 and must read back sign-extended as 0xff80. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x4ffffc; /* MR1 = 0xffff */
    adsp2181_pm(cpu)[1] = 0x4007fd; /* MR2 = 0x7f */
    adsp2181_pm(cpu)[2] = 0x4ffffb; /* MR0 = 0xffff */
    adsp2181_pm(cpu)[3] = 0x400012; /* MX0 = 1 */
    adsp2181_pm(cpu)[4] = 0x400016; /* MY0 = 1 */
    adsp2181_pm(cpu)[5] = 0x21000f; /* MR = MR + MX0*MY0 (SS), +2 */
    adsp2181_pm(cpu)[6] = 0x90100d; /* DM(0x0100) = MR2 */
    adsp2181_pm(cpu)[7] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] == 0xff80);

    /* ADSP-218x DIS/ENA INTS use reserved stack-control bits 6 and 5.
     * DIS masks servicing globally without changing IMASK; ENA must then
     * take the still-pending IRQ2 and vector to PM 0x0004. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x040040; /* DIS INTS */
    adsp2181_pm(cpu)[1] = 0x040060; /* ENA INTS */
    adsp2181_set_imask(cpu, 0x0200); /* IRQ2 */
    adsp2181_run(cpu, 1);
    assert(adsp2181_pc(cpu) == 1);
    adsp2181_set_irq(cpu, ADSP2181_IRQ2, 1);
    assert(adsp2181_pc(cpu) == 1); /* pending, globally disabled */
    assert(adsp2181_imask(cpu) == 0x0200);
    adsp2181_run(cpu, 1);
    assert(adsp2181_pc(cpu) == 4); /* pending IRQ taken by ENA */
    adsp2181_set_irq(cpu, ADSP2181_IRQ2, 0);

    /* All hardware loops reached by INFO terminate on NOT CE with an
     * ordinary compute/move final instruction. CNTR=N must execute exactly
     * N passes while preserving the zero-overhead loop PC sequencing. */
    adsp2181_reset(cpu);
    adsp2181_pm(cpu)[0] = 0x400014; /* AY0 = 1 */
    adsp2181_pm(cpu)[1] = 0x40000a; /* AR = 0 */
    adsp2181_pm(cpu)[2] = 0x3c0035; /* CNTR = 3 */
    adsp2181_pm(cpu)[3] = 0x14004e; /* DO PM(4) UNTIL NOT CE */
    adsp2181_pm(cpu)[4] = 0x22620f; /* AR = AR + AY0 */
    adsp2181_pm(cpu)[5] = 0x90100a; /* DM(0x0100) = AR */
    adsp2181_pm(cpu)[6] = 0x028000; /* IDLE */
    adsp2181_run(cpu, 32);
    assert(adsp2181_dm(cpu)[0x0100] == 3);

    adsp2181_destroy(cpu);
    puts("adsp2181_core_test: PASS");
    return 0;
}
