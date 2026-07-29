// license:BSD-3-Clause
// Standalone adaptation of MAME's ADSP-21xx core, copyright Aaron Giles.
#include "adsp2181_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#define INLINE static inline
#define LSB_FIRST 1
#define TRACK_HOTSPOTS 0
#define CLEAR_LINE 0
#define ASSERT_LINE 1
#define CHIP_TYPE_ADSP2101 1
#define CHIP_TYPE_ADSP2181 5
#define ADSP2101_IRQ0 0
#define ADSP2101_IRQ1 1
#define ADSP2101_IRQ2 2
#define ADSP2101_SPORT0_RX 3
#define ADSP2101_SPORT0_TX 4
#define PC_STACK_DEPTH 16
#define CNTR_STACK_DEPTH 4
#define STAT_STACK_DEPTH 4
#define LOOP_STACK_DEPTH 4

typedef uint8_t UINT8;
typedef int8_t INT8;
typedef uint16_t UINT16;
typedef int16_t INT16;
typedef uint32_t UINT32;
typedef int32_t INT32;
typedef uint64_t UINT64;
typedef int64_t INT64;

#define logerror(...) fprintf(stderr, __VA_ARGS__)

/* 16-bit registers that can be loaded signed or unsigned */
typedef union
{
	UINT16	u;
	INT16	s;
} ADSPREG16;


/* the SHIFT result register is 32 bits */
typedef union
{
#ifdef LSB_FIRST
	struct { ADSPREG16 sr0, sr1; } srx;
#else
	struct { ADSPREG16 sr1, sr0; } srx;
#endif
	UINT32 sr;
} SHIFTRESULT;


/* the MAC result register is 40 bits */
typedef union
{
#ifdef LSB_FIRST
	struct { ADSPREG16 mr0, mr1, mr2, mrzero; } mrx;
	struct { UINT32 mr0, mr1; } mry;
#else
	struct { ADSPREG16 mrzero, mr2, mr1, mr0; } mrx;
	struct { UINT32 mr1, mr0; } mry;
#endif
	UINT64 mr;
} MACRESULT;

/* there are two banks of "core" registers */
typedef struct ADSPCORE
{
	/* ALU registers */
	ADSPREG16	ax0, ax1;
	ADSPREG16	ay0, ay1;
	ADSPREG16	ar;
	ADSPREG16	af;

	/* MAC registers */
	ADSPREG16	mx0, mx1;
	ADSPREG16	my0, my1;
	MACRESULT	mr;
	ADSPREG16	mf;

	/* SHIFT registers */
	ADSPREG16	si;
	ADSPREG16	se;
	ADSPREG16	sb;
	SHIFTRESULT	sr;

	/* dummy registers */
	ADSPREG16	zero;
} ADSPCORE;


/* ADSP-2100 Registers */
struct adsp2181
{
	/* Core registers, 2 banks */
	ADSPCORE	core;
	ADSPCORE	alt;

	/* Memory addressing registers */
	UINT32		i[8];
	INT32		m[8];
	UINT32		l[8];
	UINT32		lmask[8];
	UINT32		base[8];
	UINT8		px;

	/* other CPU registers */
	UINT32		pc;
	UINT32		ppc;
	UINT32		loop;
	UINT32		loop_condition;
	UINT32		cntr;

	/* status registers */
	UINT32		astat;
	UINT32		sstat;
	UINT32		mstat;
	UINT32		mstat_prev;
	UINT32		astat_clear;
	UINT32		idle;

	/* stacks */
	UINT32		loop_stack[LOOP_STACK_DEPTH];
	UINT32		cntr_stack[CNTR_STACK_DEPTH];
	UINT32		pc_stack[PC_STACK_DEPTH];
	UINT16		stat_stack[STAT_STACK_DEPTH][3];
	INT32		pc_sp;
	INT32		cntr_sp;
	INT32		stat_sp;
	INT32		loop_sp;

	/* external I/O */
	UINT8		flagout;
	UINT8		flagin;
	UINT8		fl0;
	UINT8		fl1;
	UINT8		fl2;
	UINT16		idma_addr;
	UINT16		idma_cache;
	UINT8		idma_offs;
	UINT8		idma_pending_write;
	UINT8		idma_boot_hold;
	UINT8		idma_boot_mode;
	UINT16		sport_rx[2];
	UINT16		sport_tx[2];
	/* Whether the firmware wrote each transmit latch since the flag was last
	 * cleared.  sport_tx alone cannot answer that: it holds the last value
	 * written whenever it was written, so a page that publishes its transmit
	 * sample through a DM pointer instead leaves a stale word behind. */
	UINT8		sport_tx_written[2];

	/* interrupt handling */
	UINT16		imask;
	UINT8		icntl;
	UINT16		ifc;
	UINT16		pmovlay;
	UINT16		dmovlay;
    UINT8   	irq_state[9];
    UINT8   	irq_latch[9];
    void *irq_callback;

	/* other internal states */
    int			icount;
	int			chip_type;
	int			mstat_mask;
	int			imask_mask;

	/* register maps */
	void *		alu_xregs[8];
	void *		alu_yregs[4];
	void *		mac_xregs[8];
	void *		mac_yregs[4];
	void *		shift_xregs[8];

    /* other callbacks */
	adsp2181_rx_cb sport_rx_callback;
	adsp2181_tx_cb sport_tx_callback;
	adsp2181_timer_cb timer_fired;

	/* memory spaces */
    UINT32 program[0x4000];
    UINT16 data[0x4000];
    UINT32 program_overlay[2][0x2000];
    UINT16 data_overlay[2][0x2000];
    UINT16 io[0x800];

	/* reverse-engineering instrumentation */
    UINT8 watch_dm[0x4000];
    UINT8 watch_pm[0x4000];
    UINT8 watch_exec[0x4000];
    UINT8 watch_irqs;
    UINT64 cycles;
    INT64 trace_budget;

};
typedef struct adsp2181 adsp2100_state;



/***************************************************************************
    PRIVATE GLOBAL VARIABLES
***************************************************************************/

static UINT16 *reverse_table = 0;
static UINT16 *mask_table = 0;
static UINT8 *condition_table = 0;

#if TRACK_HOTSPOTS
static UINT32 pcbucket[0x4000];
#endif

static UINT16 *reverse_table;
static UINT16 *mask_table;
static UINT8 *condition_table;
static void check_irqs(adsp2100_state *adsp);
INLINE UINT16 RWORD_DATA(adsp2100_state *a, UINT32 x)
{
    UINT16 v;
    x &= 0x3fff;
    if (x < 0x2000 && a->dmovlay >= 1 && a->dmovlay <= 2)
        v = a->data_overlay[a->dmovlay - 1][x];
    else
        v = a->data[x];
    if (a->watch_dm[x])
        logerror("[WATCH] dm r %04x=%04x pc=%04x ov=%u cyc=%llu\n", x, v,
                 (unsigned)(a->pc & 0x3fff), (unsigned)a->dmovlay,
                 (unsigned long long)a->cycles);
    return v;
}
INLINE void WWORD_DATA(adsp2100_state *a, UINT32 x, UINT16 v)
{
    x &= 0x3fff;
    if (a->watch_dm[x])
        logerror("[WATCH] dm w %04x=%04x pc=%04x ov=%u cyc=%llu\n", x, v,
                 (unsigned)(a->pc & 0x3fff), (unsigned)a->dmovlay,
                 (unsigned long long)a->cycles);
    if (x < 0x2000 && a->dmovlay >= 1 && a->dmovlay <= 2)
        a->data_overlay[a->dmovlay - 1][x] = v;
    else
        a->data[x] = v;
}
INLINE UINT16 RWORD_IO(adsp2100_state *a, UINT32 x) { return a->io[x & 0x7ff]; }
INLINE void WWORD_IO(adsp2100_state *a, UINT32 x, UINT16 v) { a->io[x & 0x7ff] = v; }
INLINE UINT32 RWORD_PGM(adsp2100_state *a, UINT32 x)
{
    x &= 0x3fff;
    if (x >= 0x2000 && a->pmovlay >= 1 && a->pmovlay <= 2)
        return a->program_overlay[a->pmovlay - 1][x - 0x2000] & 0xffffff;
    return a->program[x] & 0xffffff;
}
INLINE void WWORD_PGM(adsp2100_state *a, UINT32 x, UINT32 v)
{
    x &= 0x3fff;
    if (x >= 0x2000 && a->pmovlay >= 1 && a->pmovlay <= 2)
        a->program_overlay[a->pmovlay - 1][x - 0x2000] = v & 0xffffff;
    else
        a->program[x] = v & 0xffffff;
}
#define ROPCODE(a) RWORD_PGM((a), (a)->pc)

#include "2100ops.inc"

static int generate_irq(adsp2100_state *adsp, int which, int priority)
{
    if (!(adsp->imask & (0x200 >> priority)))
        return 0;
    if (adsp->watch_irqs)
        logerror("[IRQ] take=%d priority=%d from=%04x cyc=%llu imask=%03x icntl=%02x pcsp=%u statsp=%u\n",
                 which, priority, (unsigned)(adsp->pc & 0x3fff),
                 (unsigned long long)adsp->cycles, (unsigned)adsp->imask,
                 (unsigned)adsp->icntl, (unsigned)adsp->pc_sp,
                 (unsigned)adsp->stat_sp);
    adsp->irq_latch[which] = 0;
    pc_stack_push(adsp);
    stat_stack_push(adsp);
    adsp->pc = 0x04 + priority * 4;
    adsp->idle = 0;
    if (adsp->icntl & 0x10)
        adsp->imask &= ~(0x3ff >> priority);
    else
        adsp->imask &= ~0x3ff;
    return 1;
}

static void check_irqs(adsp2100_state *adsp)
{
    UINT8 check;
#define TRY_IRQ(which, priority, expression) \
    do { check = (expression); if (check && generate_irq(adsp, (which), (priority))) return; } while (0)
    TRY_IRQ(ADSP2181_IRQ2,      0, (adsp->icntl & 4) ? adsp->irq_latch[ADSP2181_IRQ2] : adsp->irq_state[ADSP2181_IRQ2]);
    TRY_IRQ(ADSP2181_IRQL1,     1, adsp->irq_state[ADSP2181_IRQL1]);
    TRY_IRQ(ADSP2181_IRQL2,     2, adsp->irq_state[ADSP2181_IRQL2]);
    TRY_IRQ(ADSP2181_SPORT0_TX, 3, adsp->irq_latch[ADSP2181_SPORT0_TX]);
    TRY_IRQ(ADSP2181_SPORT0_RX, 4, adsp->irq_latch[ADSP2181_SPORT0_RX]);
    TRY_IRQ(ADSP2181_IRQE,      5, adsp->irq_latch[ADSP2181_IRQE]);
    TRY_IRQ(ADSP2181_IRQ1,      7, (adsp->icntl & 2) ? adsp->irq_latch[ADSP2181_IRQ1] : adsp->irq_state[ADSP2181_IRQ1]);
    TRY_IRQ(ADSP2181_IRQ0,      8, (adsp->icntl & 1) ? adsp->irq_latch[ADSP2181_IRQ0] : adsp->irq_state[ADSP2181_IRQ0]);
    TRY_IRQ(ADSP2181_TIMER,     9, adsp->irq_latch[ADSP2181_TIMER]);
#undef TRY_IRQ
}

static void execute(adsp2100_state *adsp)
{

	check_irqs(adsp);

	do
	{
		UINT32 temp;
		UINT32 op;

		/* debugging */
		adsp->ppc = adsp->pc;	/* copy PC to previous PC */

#if TRACK_HOTSPOTS
		pcbucket[adsp->pc & 0x3fff]++;
#endif

		/* instruction fetch */
		op = ROPCODE(adsp);

        if (adsp->watch_exec[adsp->pc & 0x3fff]) {
            unsigned ret = adsp->pc_sp ? pc_stack_top(adsp) & 0x3fff : 0xffff;
            /* ax1/ar/mr1 carry the control-channel correlator magnitude at the
             * PM 0x3515 decision seam: PM 0x350b puts |MR1| in AR, PM 0x350d
             * copies it to AX1, and the bit in DM(0x060f) is that magnitude
             * thresholded at 0x0578.  Neither value is ever stored to DM. */
            /* pmovlay and the fetched word are logged together because a PM
             * address at or above 0x2000 means a different instruction on
             * each overlay page: the pair says which page actually ran. */
            logerror("[EXEC] pc=%04x ret=%04x pmovlay=%u dmovlay=%u op=%06x "
                     "cyc=%llu cntr=%04x "
                     "i0=%04x i1=%04x m1=%04x m3=%04x "
                     "ax1=%04x ar=%04x mr0=%04x mr1=%04x "
                     "state=%04x event=%04x span=%04x count=%04x stride=%04x "
                     "istate=%04x analysis=%04x\n",
                     (unsigned)(adsp->pc & 0x3fff), ret,
                     (unsigned)adsp->pmovlay, (unsigned)adsp->dmovlay,
                     (unsigned)op,
                     (unsigned long long)adsp->cycles, (unsigned)(adsp->cntr & 0x3fff),
                     adsp->i[0] & 0x3fff, adsp->i[1] & 0x3fff,
                     adsp->m[1] & 0x3fff, adsp->m[3] & 0x3fff,
                     adsp->core.ax1.u & 0xffff, adsp->core.ar.u & 0xffff,
                     /* mr0 carries the candidate record pointer the sequencer
                      * loads from DM(0x1692..0x1695) before testing its
                      * condition; at PM 0x334d it is the record selected. */
                     adsp->core.mr.mrx.mr0.u & 0xffff,
                     adsp->core.mr.mrx.mr1.u & 0xffff,
                     adsp->data[0x16bd], adsp->data[0x198e], adsp->data[0x16c5],
                     adsp->data[0x16c6], adsp->data[0x16c7],
                     /* the INFO sequencer's internal state and the analysis
                      * counter its record conditions compare against */
                     adsp->data[0x1652], adsp->data[0x06e6]);
        }

		if (adsp->trace_budget > 0) {
			adsp->trace_budget--;
			logerror("[TRACE] pc=%04x op=%06x ar=%04x sr0=%04x sr1=%04x "
				 "i4=%04x i5=%04x i6=%04x i7=%04x cyc=%llu\n",
				 (unsigned)(adsp->pc & 0x3fff), op,
				 adsp->core.ar.u & 0xffff,
				 adsp->core.sr.srx.sr0.u & 0xffff,
				 adsp->core.sr.srx.sr1.u & 0xffff,
				 adsp->i[4] & 0x3fff, adsp->i[5] & 0x3fff,
				 adsp->i[6] & 0x3fff, adsp->i[7] & 0x3fff,
				 (unsigned long long)adsp->cycles);
		}

		/* advance to the next instruction */
		if (adsp->pc != adsp->loop)
			adsp->pc++;

		/* handle looping */
		else
		{
			/* condition not met, keep looping */
			if (CONDITION(adsp, adsp->loop_condition))
				adsp->pc = pc_stack_top(adsp);

			/* condition met; pop the PC and loop stacks and fall through */
			else
			{
				loop_stack_pop(adsp);
				pc_stack_pop_val(adsp);
				adsp->pc++;
			}
		}

		/* parse the instruction */
		switch (op >> 16)
		{
			case 0x00:
				/* 00000000 00000000 00000000  NOP */
				break;
			case 0x01:
				/* 00000001 0xxxxxxx xxxxxxxx  dst = IO(x) */
				/* 00000001 1xxxxxxx xxxxxxxx  IO(x) = dst */
				/* ADSP-218x only */
				if (adsp->chip_type >= CHIP_TYPE_ADSP2181)
				{
					if ((op & 0x008000) == 0x000000)
						WRITE_REG(adsp, 0, op & 15, RWORD_IO(adsp, (op >> 4) & 0x7ff));
					else
						WWORD_IO(adsp, (op >> 4) & 0x7ff, READ_REG(adsp, 0, op & 15));
				}
				break;
			case 0x02:
				/* 00000010 0000xxxx xxxxxxxx  modify flag out */
				/* 00000010 10000000 00000000  idle */
				/* 00000010 10000000 0000xxxx  idle (n) */
				if (op & 0x008000)
				{
					adsp->idle = 1;
					adsp->icount = 0;
				}
				else
				{
					if (CONDITION(adsp, op & 15))
					{
						if (op & 0x020) adsp->flagout = 0;
						if (op & 0x010) adsp->flagout ^= 1;
						if (adsp->chip_type >= CHIP_TYPE_ADSP2101)
						{
							if (op & 0x080) adsp->fl0 = 0;
							if (op & 0x040) adsp->fl0 ^= 1;
							if (op & 0x200) adsp->fl1 = 0;
							if (op & 0x100) adsp->fl1 ^= 1;
							if (op & 0x800) adsp->fl2 = 0;
							if (op & 0x400) adsp->fl2 ^= 1;
						}
					}
				}
				break;
			case 0x03:
				/* 00000011 xxxxxxxx xxxxxxxx  call or jump on flag in */
				if (op & 0x000002)
				{
					if (adsp->flagin)
					{
						if (op & 0x000001)
							pc_stack_push(adsp);
						adsp->pc = ((op >> 4) & 0x0fff) | ((op << 10) & 0x3000);
					}
				}
				else
				{
					if (!adsp->flagin)
					{
						if (op & 0x000001)
							pc_stack_push(adsp);
						adsp->pc = ((op >> 4) & 0x0fff) | ((op << 10) & 0x3000);
					}
				}
				break;
			case 0x04:
				/* 00000100 00000000 000xxxxx  stack control */
				if (op & 0x000010) pc_stack_pop_val(adsp);
				if (op & 0x000008) loop_stack_pop(adsp);
				if (op & 0x000004) cntr_stack_pop(adsp);
				if (op & 0x000002)
				{
					if (op & 0x000001) stat_stack_pop(adsp);
					else stat_stack_push(adsp);
				}
				break;
			case 0x05:
				/* 00000101 00000000 00000000  saturate MR */
				if (GET_MV)
				{
					if (adsp->core.mr.mrx.mr2.u & 0x80)
						adsp->core.mr.mrx.mr2.u = 0xffff, adsp->core.mr.mrx.mr1.u = 0x8000, adsp->core.mr.mrx.mr0.u = 0x0000;
					else
						adsp->core.mr.mrx.mr2.u = 0x0000, adsp->core.mr.mrx.mr1.u = 0x7fff, adsp->core.mr.mrx.mr0.u = 0xffff;
				}
				break;
			case 0x06:
				/* 00000110 000xxxxx 00000000  DIVS */
				{
					int xop = (op >> 8) & 7;
					int yop = (op >> 11) & 3;

					xop = ALU_GETXREG_UNSIGNED(adsp, xop);
					yop = ALU_GETYREG_UNSIGNED(adsp, yop);

					temp = xop ^ yop;
					adsp->astat = (adsp->astat & ~QFLAG) | ((temp >> 10) & QFLAG);
					adsp->core.af.u = (yop << 1) | (adsp->core.ay0.u >> 15);
					adsp->core.ay0.u = (adsp->core.ay0.u << 1) | (temp >> 15);
				}
				break;
			case 0x07:
				/* 00000111 00010xxx 00000000  DIVQ */
				{
					int xop = (op >> 8) & 7;
					int res;

					xop = ALU_GETXREG_UNSIGNED(adsp, xop);

					if (GET_Q)
						res = adsp->core.af.u + xop;
					else
						res = adsp->core.af.u - xop;

					temp = res ^ xop;
					adsp->astat = (adsp->astat & ~QFLAG) | ((temp >> 10) & QFLAG);
					adsp->core.af.u = (res << 1) | (adsp->core.ay0.u >> 15);
					adsp->core.ay0.u = (adsp->core.ay0.u << 1) | ((~temp >> 15) & 0x0001);
				}
				break;
			case 0x08:
				/* 00001000 00000000 0000xxxx  reserved */
				break;
			case 0x09:
				/* 00001001 00000000 000xxxxx  modify address register */
				temp = (op >> 2) & 4;
				modify_address(adsp, temp + ((op >> 2) & 3), temp + (op & 3));
				break;
			case 0x0a:
				/* 00001010 00000000 000xxxxx  conditional return */
				if (CONDITION(adsp, op & 15))
				{
					pc_stack_pop(adsp);

					/* RTI case */
					if (op & 0x000010)
						stat_stack_pop(adsp);
				}
				break;
			case 0x0b:
				/* 00001011 00000000 xxxxxxxx  conditional jump (indirect address) */
				if (CONDITION(adsp, op & 15))
				{
					if (op & 0x000010)
						pc_stack_push(adsp);
					adsp->pc = adsp->i[4 + ((op >> 6) & 3)] & 0x3fff;
				}
				break;
			case 0x0c:
				/* 00001100 xxxxxxxx xxxxxxxx  mode control */
				if (adsp->chip_type >= CHIP_TYPE_ADSP2101)
				{
					if (op & 0x000008) adsp->mstat = (adsp->mstat & ~MSTAT_GOMODE) | ((op << 5) & MSTAT_GOMODE);
					if (op & 0x002000) adsp->mstat = (adsp->mstat & ~MSTAT_INTEGER) | ((op >> 8) & MSTAT_INTEGER);
					if (op & 0x008000) adsp->mstat = (adsp->mstat & ~MSTAT_TIMER) | ((op >> 9) & MSTAT_TIMER);
				}
				if (op & 0x000020) adsp->mstat = (adsp->mstat & ~MSTAT_BANK) | ((op >> 4) & MSTAT_BANK);
				if (op & 0x000080) adsp->mstat = (adsp->mstat & ~MSTAT_REVERSE) | ((op >> 5) & MSTAT_REVERSE);
				if (op & 0x000200) adsp->mstat = (adsp->mstat & ~MSTAT_STICKYV) | ((op >> 6) & MSTAT_STICKYV);
				if (op & 0x000800) adsp->mstat = (adsp->mstat & ~MSTAT_SATURATE) | ((op >> 7) & MSTAT_SATURATE);
				update_mstat(adsp);
				break;
			case 0x0d:
				/* 00001101 0000xxxx xxxxxxxx  internal data move */
				WRITE_REG(adsp, (op >> 10) & 3, (op >> 4) & 15, READ_REG(adsp, (op >> 8) & 3, op & 15));
				break;
			case 0x0e:
				/* 00001110 0xxxxxxx xxxxxxxx  conditional shift */
				if (CONDITION(adsp, op & 15)) shift_op(adsp, op);
				break;
			case 0x0f:
				/* 00001111 0xxxxxxx xxxxxxxx  shift immediate */
				shift_op_imm(adsp, op);
				break;
			case 0x10:
				/* 00010000 0xxxxxxx xxxxxxxx  shift with internal data register move.
                 * Parallel-move sources are sampled before either destination
                 * is written.  This matters when the move reads SR while the
                 * shift writes SR (INFO PM 0x25fc). */
				temp = READ_REG(adsp, 0, op & 15);
				shift_op(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, temp);
				break;
			case 0x11:
				/* 00010001 xxxxxxxx xxxxxxxx  shift with pgm memory read/write */
				if (op & 0x8000)
				{
					pgm_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
					shift_op(adsp, op);
				}
				else
				{
					shift_op(adsp, op);
					WRITE_REG(adsp, 0, (op >> 4) & 15, pgm_read_dag2(adsp, op));
				}
				break;
			case 0x12:
				/* 00010010 xxxxxxxx xxxxxxxx  shift with data memory read/write DAG1 */
				if (op & 0x8000)
				{
					data_write_dag1(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
					shift_op(adsp, op);
				}
				else
				{
					shift_op(adsp, op);
					WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag1(adsp, op));
				}
				break;
			case 0x13:
				/* 00010011 xxxxxxxx xxxxxxxx  shift with data memory read/write DAG2 */
				if (op & 0x8000)
				{
					data_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
					shift_op(adsp, op);
				}
				else
				{
					shift_op(adsp, op);
					WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag2(adsp, op));
				}
				break;
			case 0x14: case 0x15: case 0x16: case 0x17:
				/* 000101xx xxxxxxxx xxxxxxxx  do until */
				loop_stack_push(adsp, op & 0x3ffff);
				pc_stack_push(adsp);
				break;
			case 0x18: case 0x19: case 0x1a: case 0x1b:
				/* 000110xx xxxxxxxx xxxxxxxx  conditional jump (immediate addr) */
				if (CONDITION(adsp, op & 15))
				{
					adsp->pc = (op >> 4) & 0x3fff;
					/* check for a busy loop */
					if (adsp->pc == adsp->ppc)
						adsp->icount = 0;
				}
				break;
			case 0x1c: case 0x1d: case 0x1e: case 0x1f:
				/* 000111xx xxxxxxxx xxxxxxxx  conditional call (immediate addr) */
				if (CONDITION(adsp, op & 15))
				{
					pc_stack_push(adsp);
					adsp->pc = (op >> 4) & 0x3fff;
				}
				break;
			case 0x20: case 0x21:
				/* 0010000x xxxxxxxx xxxxxxxx  conditional MAC to MR */
				if (CONDITION(adsp, op & 15))
				{
					if (adsp->chip_type >= CHIP_TYPE_ADSP2181 && (op & 0x0018f0) == 0x000010)
						mac_op_mr_xop(adsp, op);
					else
						mac_op_mr(adsp, op);
				}
				break;
			case 0x22: case 0x23:
				/* 0010001x xxxxxxxx xxxxxxxx  conditional ALU to AR */
				if (CONDITION(adsp, op & 15))
				{
					if (adsp->chip_type >= CHIP_TYPE_ADSP2181 && (op & 0x000010) == 0x000010)
						alu_op_ar_const(adsp, op);
					else
						alu_op_ar(adsp, op);
				}
				break;
			case 0x24: case 0x25:
				/* 0010010x xxxxxxxx xxxxxxxx  conditional MAC to MF */
				if (CONDITION(adsp, op & 15))
				{
					if (adsp->chip_type >= CHIP_TYPE_ADSP2181 && (op & 0x0018f0) == 0x000010)
						mac_op_mf_xop(adsp, op);
					else
						mac_op_mf(adsp, op);
				}
				break;
			case 0x26: case 0x27:
				/* 0010011x xxxxxxxx xxxxxxxx  conditional ALU to AF */
				if (CONDITION(adsp, op & 15))
				{
					if (adsp->chip_type >= CHIP_TYPE_ADSP2181 && (op & 0x000010) == 0x000010)
						alu_op_af_const(adsp, op);
					else
						alu_op_af(adsp, op);
				}
				break;
			case 0x28: case 0x29:
				/* 0010100x xxxxxxxx xxxxxxxx  MAC to MR with internal data register move */
				temp = READ_REG(adsp, 0, op & 15);
				mac_op_mr(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, temp);
				break;
			case 0x2a: case 0x2b:
				/* 0010101x xxxxxxxx xxxxxxxx  ALU to AR with internal data register move */
				if (adsp->chip_type >= CHIP_TYPE_ADSP2181 && (op & 0x0000ff) == 0x0000aa)
					alu_op_none(adsp, op);
				else
				{
					temp = READ_REG(adsp, 0, op & 15);
					alu_op_ar(adsp, op);
					WRITE_REG(adsp, 0, (op >> 4) & 15, temp);
				}
				break;
			case 0x2c: case 0x2d:
				/* 0010110x xxxxxxxx xxxxxxxx  MAC to MF with internal data register move */
				temp = READ_REG(adsp, 0, op & 15);
				mac_op_mf(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, temp);
				break;
			case 0x2e: case 0x2f:
				/* 0010111x xxxxxxxx xxxxxxxx  ALU to AF with internal data register move */
				temp = READ_REG(adsp, 0, op & 15);
				alu_op_af(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, temp);
				break;
			case 0x30: case 0x31: case 0x32: case 0x33:
				/* 001100xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 0) */
				WRITE_REG(adsp, 0, op & 15, (INT32)(op << 14) >> 18);
				break;
			case 0x34: case 0x35: case 0x36: case 0x37:
				/* 001101xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 1) */
				WRITE_REG(adsp, 1, op & 15, (INT32)(op << 14) >> 18);
				break;
			case 0x38: case 0x39: case 0x3a: case 0x3b:
				/* 001110xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 2) */
				WRITE_REG(adsp, 2, op & 15, (INT32)(op << 14) >> 18);
				break;
			case 0x3c: case 0x3d: case 0x3e: case 0x3f:
				/* 001111xx xxxxxxxx xxxxxxxx  load non-data register immediate (group 3) */
				WRITE_REG(adsp, 3, op & 15, (INT32)(op << 14) >> 18);
				break;
			case 0x40: case 0x41: case 0x42: case 0x43: case 0x44: case 0x45: case 0x46: case 0x47:
			case 0x48: case 0x49: case 0x4a: case 0x4b: case 0x4c: case 0x4d: case 0x4e: case 0x4f:
				/* 0100xxxx xxxxxxxx xxxxxxxx  load data register immediate */
				WRITE_REG(adsp, 0, op & 15, (op >> 4) & 0xffff);
				break;
			case 0x50: case 0x51:
				/* 0101000x xxxxxxxx xxxxxxxx  MAC to MR with pgm memory read */
				mac_op_mr(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, pgm_read_dag2(adsp, op));
				break;
			case 0x52: case 0x53:
				/* 0101001x xxxxxxxx xxxxxxxx  ALU to AR with pgm memory read */
				alu_op_ar(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, pgm_read_dag2(adsp, op));
				break;
			case 0x54: case 0x55:
				/* 0101010x xxxxxxxx xxxxxxxx  MAC to MF with pgm memory read */
				mac_op_mf(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, pgm_read_dag2(adsp, op));
				break;
			case 0x56: case 0x57:
				/* 0101011x xxxxxxxx xxxxxxxx  ALU to AF with pgm memory read */
				alu_op_af(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, pgm_read_dag2(adsp, op));
				break;
			case 0x58: case 0x59:
				/* 0101100x xxxxxxxx xxxxxxxx  MAC to MR with pgm memory write */
				pgm_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				mac_op_mr(adsp, op);
				break;
			case 0x5a: case 0x5b:
				/* 0101101x xxxxxxxx xxxxxxxx  ALU to AR with pgm memory write */
				pgm_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				alu_op_ar(adsp, op);
				break;
			case 0x5c: case 0x5d:
				/* 0101110x xxxxxxxx xxxxxxxx  ALU to MR with pgm memory write */
				pgm_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				mac_op_mf(adsp, op);
				break;
			case 0x5e: case 0x5f:
				/* 0101111x xxxxxxxx xxxxxxxx  ALU to MF with pgm memory write */
				pgm_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				alu_op_af(adsp, op);
				break;
			case 0x60: case 0x61:
				/* 0110000x xxxxxxxx xxxxxxxx  MAC to MR with data memory read DAG1 */
				mac_op_mr(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag1(adsp, op));
				break;
			case 0x62: case 0x63:
				/* 0110001x xxxxxxxx xxxxxxxx  ALU to AR with data memory read DAG1 */
				alu_op_ar(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag1(adsp, op));
				break;
			case 0x64: case 0x65:
				/* 0110010x xxxxxxxx xxxxxxxx  MAC to MF with data memory read DAG1 */
				mac_op_mf(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag1(adsp, op));
				break;
			case 0x66: case 0x67:
				/* 0110011x xxxxxxxx xxxxxxxx  ALU to AF with data memory read DAG1 */
				alu_op_af(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag1(adsp, op));
				break;
			case 0x68: case 0x69:
				/* 0110100x xxxxxxxx xxxxxxxx  MAC to MR with data memory write DAG1 */
				data_write_dag1(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				mac_op_mr(adsp, op);
				break;
			case 0x6a: case 0x6b:
				/* 0110101x xxxxxxxx xxxxxxxx  ALU to AR with data memory write DAG1 */
				data_write_dag1(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				alu_op_ar(adsp, op);
				break;
			case 0x6c: case 0x6d:
				/* 0111110x xxxxxxxx xxxxxxxx  MAC to MF with data memory write DAG1 */
				data_write_dag1(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				mac_op_mf(adsp, op);
				break;
			case 0x6e: case 0x6f:
				/* 0111111x xxxxxxxx xxxxxxxx  ALU to AF with data memory write DAG1 */
				data_write_dag1(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				alu_op_af(adsp, op);
				break;
			case 0x70: case 0x71:
				/* 0111000x xxxxxxxx xxxxxxxx  MAC to MR with data memory read DAG2 */
				mac_op_mr(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag2(adsp, op));
				break;
			case 0x72: case 0x73:
				/* 0111001x xxxxxxxx xxxxxxxx  ALU to AR with data memory read DAG2 */
				alu_op_ar(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag2(adsp, op));
				break;
			case 0x74: case 0x75:
				/* 0111010x xxxxxxxx xxxxxxxx  MAC to MF with data memory read DAG2 */
				mac_op_mf(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag2(adsp, op));
				break;
			case 0x76: case 0x77:
				/* 0111011x xxxxxxxx xxxxxxxx  ALU to AF with data memory read DAG2 */
				alu_op_af(adsp, op);
				WRITE_REG(adsp, 0, (op >> 4) & 15, data_read_dag2(adsp, op));
				break;
			case 0x78: case 0x79:
				/* 0111100x xxxxxxxx xxxxxxxx  MAC to MR with data memory write DAG2 */
				data_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				mac_op_mr(adsp, op);
				break;
			case 0x7a: case 0x7b:
				/* 0111101x xxxxxxxx xxxxxxxx  ALU to AR with data memory write DAG2 */
				data_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				alu_op_ar(adsp, op);
				break;
			case 0x7c: case 0x7d:
				/* 0111110x xxxxxxxx xxxxxxxx  MAC to MF with data memory write DAG2 */
				data_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				mac_op_mf(adsp, op);
				break;
			case 0x7e: case 0x7f:
				/* 0111111x xxxxxxxx xxxxxxxx  ALU to AF with data memory write DAG2 */
				data_write_dag2(adsp, op, READ_REG(adsp, 0, (op >> 4) & 15));
				alu_op_af(adsp, op);
				break;
			case 0x80: case 0x81: case 0x82: case 0x83:
				/* 100000xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 0 */
				WRITE_REG(adsp, 0, op & 15, RWORD_DATA(adsp, (op >> 4) & 0x3fff));
				break;
			case 0x84: case 0x85: case 0x86: case 0x87:
				/* 100001xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 1 */
				WRITE_REG(adsp, 1, op & 15, RWORD_DATA(adsp, (op >> 4) & 0x3fff));
				break;
			case 0x88: case 0x89: case 0x8a: case 0x8b:
				/* 100010xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 2 */
				WRITE_REG(adsp, 2, op & 15, RWORD_DATA(adsp, (op >> 4) & 0x3fff));
				break;
			case 0x8c: case 0x8d: case 0x8e: case 0x8f:
				/* 100011xx xxxxxxxx xxxxxxxx  read data memory (immediate addr) to reg group 3 */
				WRITE_REG(adsp, 3, op & 15, RWORD_DATA(adsp, (op >> 4) & 0x3fff));
				break;
			case 0x90: case 0x91: case 0x92: case 0x93:
				/* 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 0 */
				WWORD_DATA(adsp, (op >> 4) & 0x3fff, READ_REG(adsp, 0, op & 15));
				break;
			case 0x94: case 0x95: case 0x96: case 0x97:
				/* 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 1 */
				WWORD_DATA(adsp, (op >> 4) & 0x3fff, READ_REG(adsp, 1, op & 15));
				break;
			case 0x98: case 0x99: case 0x9a: case 0x9b:
				/* 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 2 */
				WWORD_DATA(adsp, (op >> 4) & 0x3fff, READ_REG(adsp, 2, op & 15));
				break;
			case 0x9c: case 0x9d: case 0x9e: case 0x9f:
				/* 1001xxxx xxxxxxxx xxxxxxxx  write data memory (immediate addr) from reg group 3 */
				WWORD_DATA(adsp, (op >> 4) & 0x3fff, READ_REG(adsp, 3, op & 15));
				break;
			case 0xa0: case 0xa1: case 0xa2: case 0xa3: case 0xa4: case 0xa5: case 0xa6: case 0xa7:
			case 0xa8: case 0xa9: case 0xaa: case 0xab: case 0xac: case 0xad: case 0xae: case 0xaf:
				/* 1010xxxx xxxxxxxx xxxxxxxx  data memory write (immediate) DAG1 */
				data_write_dag1(adsp, op, (op >> 4) & 0xffff);
				break;
			case 0xb0: case 0xb1: case 0xb2: case 0xb3: case 0xb4: case 0xb5: case 0xb6: case 0xb7:
			case 0xb8: case 0xb9: case 0xba: case 0xbb: case 0xbc: case 0xbd: case 0xbe: case 0xbf:
				/* 1011xxxx xxxxxxxx xxxxxxxx  data memory write (immediate) DAG2 */
				data_write_dag2(adsp, op, (op >> 4) & 0xffff);
				break;
			case 0xc0: case 0xc1:
				/* 1100000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to AY0 */
				mac_op_mr(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xc2: case 0xc3:
				/* 1100001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to AY0 */
				alu_op_ar(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xc4: case 0xc5:
				/* 1100010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to AY0 */
				mac_op_mr(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xc6: case 0xc7:
				/* 1100011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to AY0 */
				alu_op_ar(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xc8: case 0xc9:
				/* 1100100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to AY0 */
				mac_op_mr(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xca: case 0xcb:
				/* 1100101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to AY0 */
				alu_op_ar(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xcc: case 0xcd:
				/* 1100110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to AY0 */
				mac_op_mr(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xce: case 0xcf:
				/* 1100111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to AY0 */
				alu_op_ar(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.ay0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xd0: case 0xd1:
				/* 1101000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to AY1 */
				mac_op_mr(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xd2: case 0xd3:
				/* 1101001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to AY1 */
				alu_op_ar(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xd4: case 0xd5:
				/* 1101010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to AY1 */
				mac_op_mr(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xd6: case 0xd7:
				/* 1101011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to AY1 */
				alu_op_ar(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xd8: case 0xd9:
				/* 1101100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to AY1 */
				mac_op_mr(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xda: case 0xdb:
				/* 1101101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to AY1 */
				alu_op_ar(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xdc: case 0xdd:
				/* 1101110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to AY1 */
				mac_op_mr(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xde: case 0xdf:
				/* 1101111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to AY1 */
				alu_op_ar(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.ay1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xe0: case 0xe1:
				/* 1110000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to MY0 */
				mac_op_mr(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xe2: case 0xe3:
				/* 1110001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to MY0 */
				alu_op_ar(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xe4: case 0xe5:
				/* 1110010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to MY0 */
				mac_op_mr(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xe6: case 0xe7:
				/* 1110011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to MY0 */
				alu_op_ar(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xe8: case 0xe9:
				/* 1110100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to MY0 */
				mac_op_mr(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xea: case 0xeb:
				/* 1110101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to MY0 */
				alu_op_ar(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xec: case 0xed:
				/* 1110110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to MY0 */
				mac_op_mr(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xee: case 0xef:
				/* 1110111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to MY0 */
				alu_op_ar(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.my0.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xf0: case 0xf1:
				/* 1111000x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX0 & pgm read to MY1 */
				mac_op_mr(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xf2: case 0xf3:
				/* 1111001x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX0 & pgm read to MY1 */
				alu_op_ar(adsp, op);
				adsp->core.ax0.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xf4: case 0xf5:
				/* 1111010x xxxxxxxx xxxxxxxx  MAC to MR with data read to AX1 & pgm read to MY1 */
				mac_op_mr(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xf6: case 0xf7:
				/* 1111011x xxxxxxxx xxxxxxxx  ALU to AR with data read to AX1 & pgm read to MY1 */
				alu_op_ar(adsp, op);
				adsp->core.ax1.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xf8: case 0xf9:
				/* 1111100x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX0 & pgm read to MY1 */
				mac_op_mr(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xfa: case 0xfb:
				/* 1111101x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX0 & pgm read to MY1 */
				alu_op_ar(adsp, op);
				adsp->core.mx0.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xfc: case 0xfd:
				/* 1111110x xxxxxxxx xxxxxxxx  MAC to MR with data read to MX1 & pgm read to MY1 */
				mac_op_mr(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
			case 0xfe: case 0xff:
				/* 1111111x xxxxxxxx xxxxxxxx  ALU to AR with data read to MX1 & pgm read to MY1 */
				alu_op_ar(adsp, op);
				adsp->core.mx1.u = data_read_dag1(adsp, op);
				adsp->core.my1.u = pgm_read_dag2(adsp, op >> 4);
				break;
		}

		adsp->icount--;
		adsp->cycles++;
	} while (adsp->icount > 0);
}

static int create_tables(void)
{
    if (reverse_table) return 1;
    reverse_table = (UINT16 *)calloc(0x4000, sizeof(*reverse_table));
    mask_table = (UINT16 *)calloc(0x4000, sizeof(*mask_table));
    condition_table = (UINT8 *)calloc(0x1000, sizeof(*condition_table));
    if (!reverse_table || !mask_table || !condition_table) return 0;
    for (int i = 0; i < 0x4000; i++) {
        UINT16 r = 0;
        for (int b = 0; b < 14; b++) r |= ((i >> b) & 1) << (13-b);
        reverse_table[i] = r;
        if      (i > 0x2000) mask_table[i] = 0x0000;
        else if (i > 0x1000) mask_table[i] = 0x2000;
        else if (i > 0x0800) mask_table[i] = 0x3000;
        else if (i > 0x0400) mask_table[i] = 0x3800;
        else if (i > 0x0200) mask_table[i] = 0x3c00;
        else if (i > 0x0100) mask_table[i] = 0x3e00;
        else if (i > 0x0080) mask_table[i] = 0x3f00;
        else if (i > 0x0040) mask_table[i] = 0x3f80;
        else if (i > 0x0020) mask_table[i] = 0x3fc0;
        else if (i > 0x0010) mask_table[i] = 0x3fe0;
        else if (i > 0x0008) mask_table[i] = 0x3ff0;
        else if (i > 0x0004) mask_table[i] = 0x3ff8;
        else if (i > 0x0002) mask_table[i] = 0x3ffc;
        else if (i > 0x0001) mask_table[i] = 0x3ffe;
        else                 mask_table[i] = 0x3fff;
    }
    for (int i = 0; i < 0x100; i++) {
        int az = !!(i & ZFLAG), an = !!(i & NFLAG), av = !!(i & VFLAG);
        int ac = !!(i & CFLAG), mv = !!(i & MVFLAG), as = !!(i & SFLAG);
        condition_table[i | 0x000] = az;
        condition_table[i | 0x100] = !az;
        condition_table[i | 0x200] = !((an ^ av) | az);
        condition_table[i | 0x300] = (an ^ av) | az;
        condition_table[i | 0x400] = an ^ av;
        condition_table[i | 0x500] = !(an ^ av);
        condition_table[i | 0x600] = av;
        condition_table[i | 0x700] = !av;
        condition_table[i | 0x800] = ac;
        condition_table[i | 0x900] = !ac;
        condition_table[i | 0xa00] = as;
        condition_table[i | 0xb00] = !as;
        condition_table[i | 0xc00] = mv;
        condition_table[i | 0xd00] = !mv;
        condition_table[i | 0xf00] = 1;
    }
    return 1;
}

static void setup_register_maps(adsp2100_state *a)
{
#define P(arr,n,x) (arr)[n] = &(x)
    P(a->alu_xregs,0,a->core.ax0); P(a->alu_xregs,1,a->core.ax1); P(a->alu_xregs,2,a->core.ar);
    P(a->alu_xregs,3,a->core.mr.mrx.mr0); P(a->alu_xregs,4,a->core.mr.mrx.mr1); P(a->alu_xregs,5,a->core.mr.mrx.mr2);
    P(a->alu_xregs,6,a->core.sr.srx.sr0); P(a->alu_xregs,7,a->core.sr.srx.sr1);
    P(a->alu_yregs,0,a->core.ay0); P(a->alu_yregs,1,a->core.ay1); P(a->alu_yregs,2,a->core.af); P(a->alu_yregs,3,a->core.zero);
    P(a->mac_xregs,0,a->core.mx0); P(a->mac_xregs,1,a->core.mx1); P(a->mac_xregs,2,a->core.ar);
    P(a->mac_xregs,3,a->core.mr.mrx.mr0); P(a->mac_xregs,4,a->core.mr.mrx.mr1); P(a->mac_xregs,5,a->core.mr.mrx.mr2);
    P(a->mac_xregs,6,a->core.sr.srx.sr0); P(a->mac_xregs,7,a->core.sr.srx.sr1);
    P(a->mac_yregs,0,a->core.my0); P(a->mac_yregs,1,a->core.my1); P(a->mac_yregs,2,a->core.mf); P(a->mac_yregs,3,a->core.zero);
    P(a->shift_xregs,0,a->core.si); P(a->shift_xregs,1,a->core.si); P(a->shift_xregs,2,a->core.ar);
    P(a->shift_xregs,3,a->core.mr.mrx.mr0); P(a->shift_xregs,4,a->core.mr.mrx.mr1); P(a->shift_xregs,5,a->core.mr.mrx.mr2);
    P(a->shift_xregs,6,a->core.sr.srx.sr0); P(a->shift_xregs,7,a->core.sr.srx.sr1);
#undef P
}

adsp2181_t *adsp2181_create(void)
{
    adsp2100_state *a = (adsp2100_state *)calloc(1, sizeof(*a));
    if (!a || !create_tables()) { free(a); return NULL; }
    a->chip_type = CHIP_TYPE_ADSP2181; a->mstat_mask = 0x7f; a->imask_mask = 0x3ff;
    setup_register_maps(a); adsp2181_reset(a); return a;
}
void adsp2181_destroy(adsp2181_t *a) { free(a); }
void adsp2181_reset(adsp2181_t *a)
{
    a->core.zero.u = a->alt.zero.u = 0;
    wr_l0(a, a->l[0]); wr_i0(a, a->i[0]);
    wr_l1(a, a->l[1]); wr_i1(a, a->i[1]);
    wr_l2(a, a->l[2]); wr_i2(a, a->i[2]);
    wr_l3(a, a->l[3]); wr_i3(a, a->i[3]);
    wr_l4(a, a->l[4]); wr_i4(a, a->i[4]);
    wr_l5(a, a->l[5]); wr_i5(a, a->i[5]);
    wr_l6(a, a->l[6]); wr_i6(a, a->i[6]);
    wr_l7(a, a->l[7]); wr_i7(a, a->i[7]);
    a->pc=0; a->ppc=0xffffffff; a->loop=0xffff; a->loop_condition=0;
    a->astat_clear=~(CFLAG|VFLAG|NFLAG|ZFLAG); a->mstat=0; a->sstat=0x55; a->idle=0;
    a->pmovlay=0; a->dmovlay=0;
    memset(a->sport_rx, 0, sizeof(a->sport_rx));
    memset(a->sport_tx, 0, sizeof(a->sport_tx));
    memset(a->sport_tx_written, 0, sizeof(a->sport_tx_written));
    update_mstat(a);
    a->pc_sp=a->cntr_sp=a->stat_sp=a->loop_sp=0; a->imask=0; a->icntl=0;
    memset(a->irq_state, 0, sizeof(a->irq_state));
    memset(a->irq_latch, 0, sizeof(a->irq_latch));
}
int adsp2181_run(adsp2181_t *a, int cycles) { if (!a || cycles<=0 || a->idma_boot_hold) return 0; a->icount=cycles; execute(a); return cycles-a->icount; }
/* Put the core in IDMA boot hold: it executes nothing until an IDMA write
 * commits a word to program memory location 0, which starts it at PM 0.
 * This is what keeps a DSP from running its own half-overwritten image
 * while the host streams a download into it. */
void adsp2181_set_idma_boot_hold(adsp2181_t *a, int on)
{
    if (!a) return;
    a->idma_boot_hold = on != 0;
    if (on) a->idma_boot_mode = 1;
}
int adsp2181_idma_boot_held(const adsp2181_t *a) { return a ? a->idma_boot_hold : 0; }
void adsp2181_set_callbacks(adsp2181_t *a, adsp2181_rx_cb r, adsp2181_tx_cb t, adsp2181_timer_cb f) { a->sport_rx_callback=r; a->sport_tx_callback=t; a->timer_fired=f; }
uint32_t *adsp2181_pm(adsp2181_t *a) { return a->program; }
uint16_t *adsp2181_dm(adsp2181_t *a) { return a->data; }
uint16_t *adsp2181_io(adsp2181_t *a) { return a->io; }
uint32_t *adsp2181_pm_overlay(adsp2181_t *a, int overlay)
{
    return a && overlay >= 1 && overlay <= 2 ? a->program_overlay[overlay - 1] : NULL;
}
uint16_t *adsp2181_dm_overlay(adsp2181_t *a, int overlay)
{
    return a && overlay >= 1 && overlay <= 2 ? a->data_overlay[overlay - 1] : NULL;
}
/* IDMA address bit 14 is the ADSP-2181 "destination type" the datasheet
 * describes ("a 14-bit address and 1-bit destination type"): 0 selects the
 * 24-bit program memory, 1 selects the 16-bit data memory.
 *
 * Three independent facts in the shipping Eicon firmware fix this polarity:
 *
 *  - the host-port helpers (te_dmlt.pm 0x80082950 write / 0x80082920 read)
 *    use the two-access form for addresses *below* 0x4000 and a single
 *    access at or above it (both use `bnel`/`beqz` on `addr < 0x4000`), and
 *    a 24-bit PM word is exactly what needs two 16-bit accesses.  The
 *    unconditional 24-bit accessors at 0x80082974 / 0x80082994 confirm the
 *    two-access form is `(hi << 8) | (lo & 0xff)`.
 *  - the symbol resolver (0x800a6204) adds 0x4000 when the target memory
 *    block's `memory_type & 1` is clear, and the combifile memory-block
 *    tables give type 0 to the DM blocks (kernel block 0 @0x0000, block 2
 *    @0x2f80) and type 1 to the PM blocks (block 1 @0x0900, block 3
 *    @0x0580).  So DM gets the flag, PM does not.
 *  - the same resolver's fixed-segment path adds 0x4000 for segments 0 and
 *    2 and not for 1 and 3, matching those blocks' DM/PM split.
 *
 * A previous revision had this inverted, which is why a single PM write
 * needed a commit-on-address-change workaround to make the DSP presence
 * check pass: the check writes DM, and DM needs no workaround.
 */
void adsp2181_idma_addr_write(adsp2181_t *a, uint16_t address)
{
    /* A PM word takes two data accesses.  If only the first *write* half
     * has arrived, commit it as value<<8 before the address changes; the
     * helper's second access supplies a zero pad byte anyway.  A dangling
     * read half carries no data and must not write anything back. */
    if (!(a->idma_addr & 0x4000) && a->idma_offs && a->idma_pending_write)
        WWORD_PGM(a, a->idma_addr & 0x3fff, (UINT32)a->idma_cache << 8);
    a->idma_addr = address; a->idma_offs = 0; a->idma_pending_write = 0;
}
uint16_t adsp2181_idma_addr_read(const adsp2181_t *a) { return a->idma_addr; }
void adsp2181_idma_data_write(adsp2181_t *a, uint16_t value)
{
    if (a->idma_addr & 0x4000) {
        WWORD_DATA(a, a->idma_addr++ & 0x3fff, value);
    } else if (!a->idma_offs) {
        a->idma_cache = value; a->idma_offs = 1; a->idma_pending_write = 1;
    } else {
        uint16_t pm_addr = a->idma_addr & 0x3fff;
        WWORD_PGM(a, pm_addr, (a->idma_cache << 8) | (value & 0xff));
        a->idma_addr++;
        a->idma_offs = 0; a->idma_pending_write = 0;
        /* IDMA boot (BMODE=1, MMAP=0): "Program execution is held off until
         * on-chip program memory location 0 is written to."  The Eicon
         * download streams the image from PM 0x0001 up and releases the core
         * with a final write to PM 0.
         *
         * Any other program-memory write re-arms the hold: it means a code
         * download is under way, and a core executing its own half-replaced
         * image corrupts the transfer and then runs wild.  Data memory is
         * left alone, so mailboxes and command rings can be written to a
         * running DSP as usual.
         *
         * Only cores put in IDMA boot mode behave this way; a core whose
         * image was staged directly (the single-DSP harnesses) keeps
         * running through host PM writes. */
        if (a->idma_boot_mode) {
            if (pm_addr == 0) {
                if (a->idma_boot_hold) {
                    a->idma_boot_hold = 0;
                    a->pc = 0; a->ppc = 0xffffffff;
                }
            } else {
                a->idma_boot_hold = 1;
            }
        }
    }
}
uint16_t adsp2181_idma_data_read(adsp2181_t *a)
{
    uint16_t result;
    if (a->idma_addr & 0x4000) {
        result = RWORD_DATA(a, a->idma_addr++ & 0x3fff);
    } else if (!a->idma_offs) {
        result = RWORD_PGM(a, a->idma_addr & 0x3fff) >> 8;
        a->idma_offs = 1; a->idma_pending_write = 0;
    } else {
        result = RWORD_PGM(a, a->idma_addr++ & 0x3fff) & 0xff;
        a->idma_offs = 0;
    }
    return result;
}
/* Whole-word host-port access with the Eicon helper semantics: bit 14 set
 * is one 16-bit DM access; bit 14 clear is a 24-bit PM word carried by two
 * accesses, of which the helper supplies the value first and a zero pad
 * byte second. */
void adsp2181_host_write(adsp2181_t *a, uint16_t addr, uint16_t value)
{
    if (addr & 0x4000)
        WWORD_DATA(a, addr & 0x3fff, value);
    else
        WWORD_PGM(a, addr & 0x3fff, (UINT32)value << 8);
}
uint16_t adsp2181_host_read(adsp2181_t *a, uint16_t addr)
{
    if (addr & 0x4000)
        return RWORD_DATA(a, addr & 0x3fff);
    return (uint16_t)(RWORD_PGM(a, addr & 0x3fff) >> 8);
}
void adsp2181_watch_dm(adsp2181_t *a, uint16_t addr, int on)
{
    if (a) a->watch_dm[addr & 0x3fff] = on != 0;
}
void adsp2181_watch_pm(adsp2181_t *a, uint16_t addr, int on)
{
    if (a) a->watch_pm[addr & 0x3fff] = on != 0;
}

void adsp2181_watch_exec(adsp2181_t *a, uint16_t addr, int on)
{
    if (a) a->watch_exec[addr & 0x3fff] = on != 0;
}

void adsp2181_watch_irqs(adsp2181_t *a, int on)
{
    if (a) a->watch_irqs = on != 0;
}

/* The PMOVLAY/DMOVLAY page selectors.  Written by the firmware through the
 * register map (2100ops.inc wr_pmovlay/wr_dmovlay), so a caller that wants
 * to know which page a PM address above 0x2000 resolved to has to read them
 * at the same instant as the fetch, not afterwards. */
/* Did the firmware drive the SPORT0 transmit latch during the last
 * adsp2181_sport0_tdm_frame()?  A caller acting as the line side needs this
 * to tell a real transmit sample from a stale latch left by an earlier page. */
int adsp2181_sport0_tx_written(const adsp2181_t *a)
{
    return a ? a->sport_tx_written[0] : 0;
}

uint16_t adsp2181_pmovlay(const adsp2181_t *a) { return a ? a->pmovlay : 0; }
uint16_t adsp2181_dmovlay(const adsp2181_t *a) { return a ? a->dmovlay : 0; }

/* Read a PM word the way the core would right now: resolved through the
 * current PMOVLAY, not out of the resident image. */
uint32_t adsp2181_read_pm(adsp2181_t *a, uint16_t addr)
{
    return a ? RWORD_PGM(a, addr) : 0;
}
uint64_t adsp2181_cycles(const adsp2181_t *a) { return a ? a->cycles : 0; }
void adsp2181_trace_budget(adsp2181_t *a, int64_t n) { if (a) a->trace_budget = n; }
uint16_t adsp2181_pc(const adsp2181_t *a) { return a->pc & 0x3fff; }
void adsp2181_set_pc(adsp2181_t *a, uint16_t pc) { a->pc = pc & 0x3fff; a->idle = 0; }
void adsp2181_call(adsp2181_t *a, uint16_t entry, uint16_t return_pc)
{
    if (!a) return;
    pc_stack_push_val(a, return_pc & 0x3fff);
    a->pc = entry & 0x3fff;
    a->idle = 0;
}
void adsp2181_set_irq(adsp2181_t *a, int irq, int asserted)
{
    if (!a || irq < 0 || irq >= ADSP2181_IRQ_COUNT) return;
    if (asserted && !a->irq_state[irq]) {
        a->irq_latch[irq] = 1;
        if (a->sport_rx_callback) {
            if (irq == ADSP2181_SPORT0_RX)
                a->sport_rx[0] = (UINT16)a->sport_rx_callback(a, 0);
            else if (irq == ADSP2181_SPORT1_RX)
                a->sport_rx[1] = (UINT16)a->sport_rx_callback(a, 1);
        }
    }
    a->irq_state[irq] = asserted != 0;
    check_irqs(a);
}

uint16_t adsp2181_sport0_tdm_frame(adsp2181_t *a, int active_slot,
                                   int dispatch_slot, uint16_t active_word,
                                   uint16_t idle_word, int cycles_per_slot)
{
    if (!a || active_slot < 0 || active_slot >= 32 || dispatch_slot < 0 ||
        dispatch_slot >= 32 || cycles_per_slot <= 0)
        return 0;
    /* PM 02b9 is the kernel foreground dispatch slot. TIKRNL patches it to
     * CALL its continuation. Keep that call only on the assigned PRI
     * timeslot; the other 31 slots run the ordinary empty host dispatcher. */
    const UINT32 task_dispatch = RWORD_PGM(a, 0x02b9);
    const UINT32 task_isr = RWORD_PGM(a, 0x00b5);
    uint16_t selected_tx = 0;
    /* The closed MIPS channel assignment filters the 32-slot TDM stream
     * before this task dispatch: one call receives one 8 kHz slot, not 32
     * task invocations. Model that selected descriptor directly. */
    WWORD_PGM(a, 0x02b9, task_dispatch);
    WWORD_PGM(a, 0x00b5, task_isr);
    a->sport_rx[0] = active_word;
    a->sport_tx_written[0] = 0;
    for (UINT16 address = 0x2e00; address < 0x2e40; ++address)
        WWORD_DATA(a, address, active_word);
    a->irq_latch[ADSP2181_SPORT0_RX] = 1;
    a->irq_state[ADSP2181_SPORT0_RX] = 1;
    check_irqs(a);
    a->irq_state[ADSP2181_SPORT0_RX] = 0;
    a->icount = cycles_per_slot;
    execute(a);
    selected_tx = a->sport_tx[0];
    WWORD_PGM(a, 0x02b9, task_dispatch);
    WWORD_PGM(a, 0x00b5, task_isr);
    return selected_tx;
}
uint16_t adsp2181_imask(const adsp2181_t *a) { return a->imask; }
void adsp2181_set_imask(adsp2181_t *a, uint16_t imask) { if (a) a->imask = imask & 0x3ff; }
void adsp2181_set_flagin(adsp2181_t *a, int asserted) { if (a) a->flagin = asserted ? 1 : 0; }
int adsp2181_flagin(const adsp2181_t *a) { return a ? a->flagin : 0; }
uint16_t adsp2181_icntl(const adsp2181_t *a) { return a->icntl; }
int adsp2181_idle(const adsp2181_t *a) { return a->idle != 0; }
void adsp2181_set_ar(adsp2181_t *a, uint16_t value)
{
    if (a) a->core.ar.u = value;
}
uint16_t adsp2181_sr0(const adsp2181_t *a)
{
    return a ? a->core.sr.srx.sr0.u : 0;
}
uint16_t adsp2181_sr1(const adsp2181_t *a)
{
    return a ? a->core.sr.srx.sr1.u : 0;
}
