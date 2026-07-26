#!/usr/bin/env python3
"""MIPS-side driver shim for the Eicon ADSP emulator.

Runs the real te_dmlt.pm firmware routines (host-port helpers, script
sender, database record builder/commit, request parser) under Unicorn and
connects their host-port transactions to the ADSP-2181 emulator (ctypes),
so the firmware's own code drives the DSP exactly as on real hardware.

Current scope: proof of concept for the command-script sender
(0x800896a4) with a fabricated request struct, against a live
kernel+TIKRNL ADSP instance.
"""

from __future__ import annotations

import argparse
import ctypes
import struct
import sys
from pathlib import Path

from unicorn import Uc, UC_ARCH_MIPS, UC_MODE_LITTLE_ENDIAN, UC_MODE_32
from unicorn import UC_HOOK_CODE

BIAS = 0x80011000
# Unicorn's MIPS translates kseg0 (0x8xxxxxxx) to physical by clearing the
# top three bits, so all mappings use the physical equivalents.
PHYS_BIAS = 0x00011000
IMAGE_SIZE = 0x100000  # covers code (0x80011000) and data (0x8010xxxx)
RAM_VIRT = 0x80800000
RAM_BASE = 0x00800000
RAM_SIZE = 0x100000
STUB_VIRT = 0x80900000
STUB_BASE = 0x00900000

HOST_WRITE = BIAS + 0x71950  # 0x80082950
HOST_READ = BIAS + 0x71920   # 0x80082920
SCRIPT_SENDER = BIAS + 0x786A4  # 0x800896a4

# ---------------------------------------------------------------- ADSP side

ADSP = ctypes.CDLL(str(Path(__file__).resolve().parent /
                     "adsp2181emu" / "libadsp2181.dylib"))
ADSP.adsp2181_create.restype = ctypes.c_void_p
ADSP.adsp2181_destroy.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_reset.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pm.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_idle.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_call.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
ADSP.adsp2181_run.argtypes = [ctypes.c_void_p, ctypes.c_int]
ADSP.adsp2181_set_irq.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
ADSP.adsp2181_host_write.argtypes = [ctypes.c_void_p, ctypes.c_uint16,
                                     ctypes.c_uint16]
ADSP.adsp2181_host_read.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_host_read.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int


def load_pm_words(cpu, path: Path) -> None:
    data = path.read_bytes()
    pm = ADSP.adsp2181_pm(cpu)
    for a in range(0x4000):
        pm[a] = data[a * 3] | (data[a * 3 + 1] << 8) | (data[a * 3 + 2] << 16)


def load_dm_words(cpu, path: Path) -> None:
    data = path.read_bytes()
    dm = ADSP.adsp2181_dm(cpu)
    for a in range(0x4000):
        dm[a] = data[a * 2] | (data[a * 2 + 1] << 8)


def apply_word_map(cpu, path: Path) -> None:
    for line in path.read_text().splitlines():
        addr_s, value_s = line.split()
        ADSP.adsp2181_host_write(cpu, int(addr_s, 16), int(value_s, 16))


# ---------------------------------------------------------------- MIPS side

class MipsShim:
    def __init__(self, image: Path, cpu, log: bool = False):
        self.cpu = cpu
        self.log = log
        self.uc = Uc(UC_ARCH_MIPS, UC_MODE_LITTLE_ENDIAN | UC_MODE_32)
        self.uc.mem_map(PHYS_BIAS, IMAGE_SIZE)
        self.uc.mem_write(PHYS_BIAS, image.read_bytes())
        self.uc.mem_map(RAM_BASE, RAM_SIZE)
        self.uc.mem_map(STUB_BASE, 0x1000)
        # stub function: jr ra; nop (two copies: entry and terminator)
        stub = struct.pack("<II", 0x03E00008, 0)
        self.uc.mem_write(STUB_BASE, stub)
        self.uc.mem_write(STUB_BASE + 0x20, stub)
        self.stub_returns = 0
        # trace printf pointer lives at gp + 0x1a7b; gp is set per-run
        self.uc.hook_add(UC_HOOK_CODE, self._hook)
        from unicorn import UC_HOOK_MEM_FETCH_UNMAPPED
        self.uc.hook_add(UC_HOOK_MEM_FETCH_UNMAPPED, self._unmapped)
        self.trace_log = []

    def _unmapped(self, uc, access, address, size, value, user):
        pc = uc.reg_read(15) if False else None
        from unicorn.mips_const import UC_MIPS_REG_PC, UC_MIPS_REG_RA
        print(f"[mips] unmapped fetch {address:08x} from pc="
              f"{uc.reg_read(UC_MIPS_REG_PC):08x} ra={uc.reg_read(UC_MIPS_REG_RA):08x}")
        for entry in self.trace_log[-12:]:
            print(f"   {entry}")
        return False

    def _hook(self, uc, address, size, user):
        self.trace_log.append(f"{address:08x}")
        from unicorn.mips_const import (UC_MIPS_REG_PC, UC_MIPS_REG_RA,
                                        UC_MIPS_REG_A1, UC_MIPS_REG_A2,
                                        UC_MIPS_REG_V0)
        if address == HOST_WRITE:
            a1 = uc.reg_read(UC_MIPS_REG_A1) & 0xFFFF
            a2 = uc.reg_read(UC_MIPS_REG_A2) & 0xFFFF
            if self.log:
                print(f"[mips] host_write {a1:04x} = {a2:04x}")
            ADSP.adsp2181_host_write(self.cpu, a1, a2)
            uc.reg_write(UC_MIPS_REG_PC, uc.reg_read(UC_MIPS_REG_RA))
        elif address == HOST_READ:
            a1 = uc.reg_read(UC_MIPS_REG_A1) & 0xFFFF
            value = ADSP.adsp2181_host_read(self.cpu, a1)
            if self.log:
                print(f"[mips] host_read {a1:04x} -> {value:04x}")
            uc.reg_write(UC_MIPS_REG_V0, value)
            uc.reg_write(UC_MIPS_REG_PC, uc.reg_read(UC_MIPS_REG_RA))
        elif address == STUB_VIRT:
            self.stub_returns += 1
            uc.reg_write(UC_MIPS_REG_PC, uc.reg_read(UC_MIPS_REG_RA))

    def call(self, entry: int, args: list[int], gp: int, sp: int,
             max_insns: int = 200000) -> int:
        from unicorn.mips_const import (UC_MIPS_REG_A0, UC_MIPS_REG_A1,
                                        UC_MIPS_REG_A2, UC_MIPS_REG_A3,
                                        UC_MIPS_REG_SP, UC_MIPS_REG_GP,
                                        UC_MIPS_REG_RA, UC_MIPS_REG_V0)
        uc = self.uc
        uc.reg_write(UC_MIPS_REG_SP, sp)
        uc.reg_write(UC_MIPS_REG_GP, gp)
        uc.reg_write(UC_MIPS_REG_RA, STUB_VIRT + 0x20)
        for i, value in enumerate(args[:4]):
            uc.reg_write([UC_MIPS_REG_A0, UC_MIPS_REG_A1,
                          UC_MIPS_REG_A2, UC_MIPS_REG_A3][i], value)
        uc.emu_start(entry, STUB_VIRT + 0x20, count=max_insns)
        return uc.reg_read(UC_MIPS_REG_V0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kernel", type=Path, required=True)
    parser.add_argument("--tikrnl", type=Path, required=True)
    parser.add_argument("--image", type=Path,
                        default=Path("docs/firmware/te_dmlt.pm"))
    parser.add_argument("--mode", type=int, default=0)
    parser.add_argument("--code", type=int, default=66,
                        help="command script code (66 = 0x0258 switch-on)")
    parser.add_argument("--selector", type=int, default=0x0001)
    parser.add_argument("--words", type=int, default=200,
                        help="8 kHz host words to pump after the commit")
    parser.add_argument("--log", action="store_true")
    args = parser.parse_args()

    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    load_pm_words(cpu, args.kernel / "pm.bin")
    load_dm_words(cpu, args.kernel / "dm.bin")
    ADSP.adsp2181_run(cpu, 1000)  # boot to IDLE
    # stage TIKRNL and run its initializer, as in eicon_adsp_run
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for line in (args.tikrnl / "pm.words").read_text().splitlines():
        a, v = line.split()
        pm[int(a, 16)] = int(v, 16)
    for line in (args.tikrnl / "dm.words").read_text().splitlines():
        a, v = line.split()
        dm[int(a, 16)] = int(v, 16)
    ADSP.adsp2181_call(cpu, 0x672, 0x02A8)
    ADSP.adsp2181_run(cpu, 1000000)
    print(f"[adsp] staged: idle={ADSP.adsp2181_idle(cpu)}")

    shim = MipsShim(args.image, cpu, log=args.log)

    # Fabricate the request struct (see dsp_assign 0x0258 tail + sender).
    req = RAM_VIRT + 0x1000  # guest-visible pointer
    buf = bytearray(0x60)
    struct.pack_into("<II", buf, 0x00, 0xDEAD0000, 0xDEAD0004)  # host regs
    struct.pack_into("<HH", buf, 0x08, 0x3310, 0x3338)  # symbol13/14 addrs
    buf[0x0C] = 1                     # active
    buf[0x10] = args.code             # script code (< 75)
    buf[0x11] = args.mode             # script mode (< 2)
    struct.pack_into("<H", buf, 0x12, args.selector)    # command selector
    struct.pack_into("<H", buf, 0x14, 0)                # script pc
    struct.pack_into("<H", buf, 0x1C, 0)                # form 0 = script
    struct.pack_into("<H", buf, 0x3E, 0x0020)           # control word
    shim.uc.mem_write(RAM_BASE + 0x1000, bytes(buf))  # API uses physical

    shim.call(SCRIPT_SENDER, [req, 0], gp=0x80108000, sp=RAM_VIRT + 0x8000)

    # pump the 8 kHz host loop so the DSP consumes the command
    for _ in range(args.words):
        ADSP.adsp2181_set_irq(cpu, 3, 1)  # SPORT0_RX
        ADSP.adsp2181_set_irq(cpu, 3, 0)
        ADSP.adsp2181_run(cpu, 10000)
    print(f"[adsp] selector DM3310={ADSP.adsp2181_host_read(cpu, 0x3310):04x}"
          f" producer DM3315={ADSP.adsp2181_host_read(cpu, 0x3315):04x}"
          f" consumer DM3316={ADSP.adsp2181_host_read(cpu, 0x3316):04x}"
          f" resp DM3338={ADSP.adsp2181_host_read(cpu, 0x3338):04x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
