"""Regression test for courier-emu: XC that skips the LAST instruction of an RPTB body must not end the block loop.
Run with courier-emu's interpreter from the courier-emu directory:  .venv/bin/python <this file>
Fails (ACC = 1) on the core before courier-emu-xc-rptb.patch, passes (ACC = 3) after it.
SPRU056D: a false XC executes NOPs in place of the skipped words, so the fetch of the block's last word still
triggers the block repeat. Found as the cause of MICA's V.32bis trellis receiver outputting noise."""
import struct
from courier_emu.dsp import NativeC5x

def run(words, steps):
    with NativeC5x.from_program(0, struct.pack('<%dH' % len(words), *words), rebuild=True) as core:
        core.step(steps)
        return core.state()

LDP0, LACL2, SAMM_BRCR, CLRC_C, LACL0 = 0xBC00, 0xB902, 0x8809, 0xBE4E, 0xB900
RPTB, END, ADD1, XC1_C, ADD16, NOP = 0xBEC6, 0x0009, 0xB801, 0xE711, 0xB810, 0x8B00
# BRCR = 2 (three passes); C clear so XC 1,C skips ADD #16, the loop's last instruction
prog = [LDP0, LACL2, SAMM_BRCR, CLRC_C, LACL0, RPTB, END, ADD1, XC1_C, ADD16, NOP, NOP, NOP, NOP]
acc = run(prog, 40)['acc'] & 0xFFFFFFFF
assert acc == 3, f'XC skipped the last RPTB instruction and the loop ended early: ACC = {acc}, expected 3'
print('ok: ACC =', acc)
