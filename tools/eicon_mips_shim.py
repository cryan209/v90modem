#!/usr/bin/env python3
"""MIPS-side driver shim for the Eicon ADSP emulator.

Runs the real te_dmlt.pm firmware routines (host-port helpers, script
sender, database record builder/commit, request parser, and the modem
service-assign entry) under Unicorn and connects their host-port
transactions to the ADSP-2181 emulator (ctypes), so the firmware's own code
drives the DSP exactly as on real hardware.

Scope:
  * default: request_parser -> script_sender command-script commit
    (code 66 reproduces the script-66 PM-ring commit).
  * --assign: the service-assign entry 0x80096980 (service-driver table
    slot 1) with a synthesized TIKRNL download/task struct, performing the
    switch-on database commit through the real firmware path.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import struct
import sys
from pathlib import Path

from unicorn import Uc, UC_ARCH_MIPS, UC_MODE_LITTLE_ENDIAN, UC_MODE_32
from unicorn import UC_HOOK_CODE

sys.path.insert(0, str(Path(__file__).resolve().parent))

from eicon_dsp_stage import (CARDTYPE_DIVASRV_P_30M_PCI,
                             OFFS_DSP_CODE_BASE_ADDR, build_dsp_code_image,
                             protocol_end_addr)

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
REQUEST_PARSER = BIAS + 0x78138  # 0x80089138
# Service-driver table slot 1: the modem service assign entry (file 0x85980).
# Performs the switch-on database commit for task 0x0258 (TIKRNL81.F34),
# reached through the table at file 0xeaec4 rather than by a direct jal.
SERVICE_ASSIGN = BIAS + 0x85980  # 0x80096980

# The MIPS image's .data/.bss for the protocol task lives at 0x80200000
# (physical 0x200000).  The te_dmlt.pm file only covers 0x11000..0x100230,
# so this segment is zero-initialized; map it writable and auto-map any
# neighbouring .bss page the firmware touches during assign.
DATA_VIRT = 0x80200000
DATA_BASE = 0x200000
DATA_SIZE = 0x100000
# The protocol image's real runtime stack/heap (sp = 0x80338700, set at the
# image entry) and the database-record buffers (e.g. 0x80331c12) live in the
# 0x80300000 segment.  Map it writable; the auto-map hook covers neighbours.
STACK_VIRT = 0x80300000
STACK_BASE = 0x300000
STACK_SIZE = 0x100000
STACK_TOP = 0x80338700  # sp value set at the image entry (file 0x477c..0x4788)
AUTO_PAGE = 0x10000

# Shared RAM (PR_RAM) lives at physical 0x1000 in DRAM (MP_SHARED_RAM_OFFSET).
# The MIPS accesses it via the uncached segment 0xa0001000.  The boot/config
# area (struct mp_load) is at physical 0x0.  Map the full 0x0..0x11000 range
# (shared RAM + boot area); the protocol image at 0x11000 is mapped separately.
SHARED_RAM_BASE = 0x00000
SHARED_RAM_SIZE = 0x11000
PR_RAM_PHYS = 0x1000       # physical base of PR_RAM
PR_RAM_VIRT = 0xa0001000   # MIPS uncached address (stored at gp+0x5e93)

# MIPS firmware entry points for the request-queue path.
MIPS_INIT = BIAS + 0x72130     # 0x80083130: store PR_RAM base, clear 0x4d20 bytes
MIPS_MAINLOOP = BIAS + 0x16970  # 0x80027970: poll PR_RAM, dispatch requests
# The real firmware entry point (boot loader jumps here after copying config
# and clearing .bss).  Reads card config from shared RAM, initialises DSP
# resources, waits for boot->cmd==3, then enters the main loop.
MIPS_ENTRY = BIAS + 0x71f90     # 0x80082f90

# PR_RAM structure (from kernel/pr_pc.h).  Offsets within the PR_RAM region.
PR_NextReq = 0x00   # word: host write pointer (offset into B[])
PR_NextRc = 0x02    # word: MIPS response write pointer
PR_NextInd = 0x04   # word: MIPS indication write pointer
PR_ReqInput = 0x06  # byte: count of requests submitted by host
PR_ReqOutput = 0x07 # byte: count of requests processed by MIPS
PR_Int = 0x09       # byte: interrupt flag
PR_ReadyInt = 0x10  # byte: host pokes this to request a ready interrupt
PR_Signature = 0x1e # word: MIPS writes 0x5858 (not ready) or valid sig
PR_B = 0x20         # start of the REQ/RC/IND buffer area

# REQ structure (from kernel/pr_pc.h).  Each REQ buffer in B[].
REQ_SIZE = 0x120    # next(2)+Req(1)+ReqId(1)+ReqCh(1)+Res1(1)+Ref(2)+Res[8]+XBuffer(2+270)
REQ_NEXT = 0x00     # word: offset of next free REQ in B[]
REQ_REQ = 0x02      # byte: request code (ASSIGN=0x01, etc.)
REQ_REQID = 0x03    # byte: entity ID (NL_ID=0x01, etc.)
REQ_REQCH = 0x04    # byte: channel number
REQ_XBUFFER = 0x10  # PBUFFER: word length + byte[270] data
REQ_XDATA = 0x12    # start of the 270-byte data payload

# IDI request codes (from the driver).
ASSIGN = 0x01
NL_ID = 0x01      # network layer entity
REMOVE = 0x04

# DSP CAI modem hardware types (from kernel/mdm_msg.h).
DSP_CAI_HARDWARE_MODEM_ASYNC = 0x11

# The protocol image sets $gp = 0x8010.0000 - 0x5c4b = 0x800fa3b5 at its entry
# (file 0x4774/0x4764c).  All gp-relative globals live off this value: the
# trace-printf pointer is at gp+0x1a7b (0x800fbe30, file-backed = 0x80083180)
# and the service-driver table at gp+0x1b0f (0x800fbec4).  Using any other gp
# leaves the printf pointer NULL and jalr faults on the first trace call.
GP = 0x800fa3b5

# ---------------------------------------------------------------- ADSP side

# ADSP2181_LIB overrides the emulator library, e.g. to load the
# AddressSanitizer build (make -C tools/adsp2181emu libadsp2181_asan.dylib).
ADSP = ctypes.CDLL(os.environ.get(
    "ADSP2181_LIB",
    str(Path(__file__).resolve().parent / "adsp2181emu" / "libadsp2181.dylib")))
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
ADSP.adsp2181_idma_addr_write.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_idma_data_write.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_idma_data_read.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_idma_data_read.restype = ctypes.c_uint16
ADSP.adsp2181_set_idma_boot_hold.argtypes = [ctypes.c_void_p, ctypes.c_int]
ADSP.adsp2181_idma_boot_held.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_idma_boot_held.restype = ctypes.c_int
ADSP.adsp2181_watch_dm.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int]
ADSP.adsp2181_watch_pm.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int]
ADSP.adsp2181_trace_budget.argtypes = [ctypes.c_void_p, ctypes.c_int64]


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
        # Zero-initialized .data/.bss segment the assign routine reads
        # (lookup tables at 0x80272c90 etc.).  Auto-map neighbours on touch.
        self.uc.mem_map(DATA_BASE, DATA_SIZE)
        self.uc.mem_map(STACK_BASE, STACK_SIZE)
        # Shared RAM + boot/config area (physical 0x0..0x11000).  The MIPS
        # accesses this via uncached 0xa0001000; Unicorn translates kseg1 to
        # physical by clearing the top 3 bits.
        self.uc.mem_map(SHARED_RAM_BASE, SHARED_RAM_SIZE)
        self.mapped_pages = {DATA_BASE, STACK_BASE, SHARED_RAM_BASE}  # phys pages
        # stub function: jr ra; nop (two copies: entry and terminator)
        stub = struct.pack("<II", 0x03E00008, 0)
        self.uc.mem_write(STUB_BASE, stub)
        self.uc.mem_write(STUB_BASE + 0x20, stub)
        self.stub_returns = 0
        # trace printf pointer lives at gp + 0x1a7b; gp is set per-run
        self.host_writes: list[tuple[int, int]] = []
        # MIPS instructions between ADSP time slices (0 disables the pump).
        # A DSP that runs while the MIPS is streaming an IDMA download will
        # execute the half-replaced image and clobber it, so the download
        # paths hold the core instead (see adsp2181_set_idma_boot_hold).
        self.pump_every = 256
        # One emulated ADSP per DSP register block (see core_for).  Off by
        # default so the single-DSP harness paths keep using self.cpu.
        self.multi_dsp = False
        self.cores: dict[int, object] = {}
        self._idma_addrs: dict[int, int] = {}
        # MIPS instruction count of the last IDMA *write* to each core.  A
        # core is not run while a download is streaming into it: the real
        # card holds the DSP in reset for the transfer, and a core executing
        # its own half-replaced image corrupts it (and then runs wild).
        # Reads do not defer execution, so the boot-handshake poll still
        # lets the DSP it is polling make progress.
        self._core_last_write: dict[int, int] = {}
        self.dsp_write_quiet = 512
        self._active_block = -1
        self._idma_addr = 0
        # Hook memory-mapped writes to the host register block.  The single
        # host_write helper (0x80082950) is intercepted at the function level
        # (PC hook), but the bulk-write helper (0x80082a38) writes directly to
        # the register block: +0x80 = IDMA address port, +0x00 = IDMA data
        # port (auto-incrementing).  Route both to the ADSP IDMA interface.
        from unicorn import UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ
        self.uc.hook_add(UC_HOOK_MEM_WRITE, self._hostreg_write,
                         begin=RAM_BASE + 0x5000, end=RAM_BASE + 0x5084)
        # The card init (0x80081de0) builds the DSP register bases itself:
        # 0xbc000800 + row_offset + dsp_index*8 for the 30 module DSPs, plus
        # 0xbc000008 / 0xbc000020 for the two on-board ones.  kseg1 maps
        # 0xbc000000 to physical 0x1c000000.  Within each block, +0x80 is the
        # IDMA address port and +0x00 the (auto-incrementing) data port.
        #
        # All 30 blocks alias onto the one emulated ADSP: the port decode
        # only looks at the low byte, so whichever DSP the firmware talks to
        # reaches the same core.  That is what makes the 30-DSP card init
        # work against a single emulated DSP.
        # The range must stop after the last DSP block: the two on-board DSPs
        # are at +0x08/+0x20, the module rows at +0x800..+0x870 and
        # +0x1000..+0x1070, and each block's address port is 0x80 above it,
        # so the highest byte in use is 0x10f8.  The card's own control
        # registers live just past that (the card object's +0x80 holds
        # 0xbc001800); hooking them too would route a plain register write
        # into the IDMA path and spawn phantom DSPs.
        self._dsp_base = 0x1c000000
        self._dsp_limit = self._dsp_base + 0x1100
        self.uc.hook_add(UC_HOOK_MEM_READ, self._dsp_read,
                         begin=self._dsp_base, end=self._dsp_limit)
        self.uc.hook_add(UC_HOOK_MEM_WRITE, self._dsp_write,
                         begin=self._dsp_base, end=self._dsp_limit)
        self.uc.hook_add(UC_HOOK_CODE, self._hook)
        from unicorn import (UC_HOOK_MEM_FETCH_UNMAPPED,
                             UC_HOOK_MEM_READ_UNMAPPED,
                             UC_HOOK_MEM_WRITE_UNMAPPED)
        self.uc.hook_add(UC_HOOK_MEM_FETCH_UNMAPPED, self._unmapped)
        self.uc.hook_add(UC_HOOK_MEM_READ_UNMAPPED, self._unmapped)
        self.uc.hook_add(UC_HOOK_MEM_WRITE_UNMAPPED, self._unmapped)
        self.trace_log = []

    def _set_load_result(self, uc, val, size):
        """Decode the MIPS load instruction at current PC and write val
        into the destination register.  Needed because the Python Unicorn
        bindings do not allow a UC_HOOK_MEM_READ callback to override the
        value the emulated instruction sees for the current access."""
        from unicorn.mips_const import UC_MIPS_REG_PC, UC_MIPS_REG_0
        pc = uc.reg_read(UC_MIPS_REG_PC)
        try:
            insn = struct.unpack("<I", uc.mem_read(pc, 4))[0]
        except Exception:
            return
        opcode = (insn >> 26) & 0x3F
        # MIPS I-type load opcodes: lb/lh/lwl/lw/lbu/lhu/lwr
        if 0x20 <= opcode <= 0x27:
            rt = (insn >> 16) & 0x1F
            if size == 1:
                if opcode == 0x20:   # lb  – sign-extend byte
                    val = val if (val & 0x80) == 0 else val - 0x100
                else:                # lbu – zero-extend byte
                    val = val & 0xFF
            elif size == 2:
                if opcode == 0x21:   # lh  – sign-extend halfword
                    val = val if (val & 0x8000) == 0 else val - 0x10000
                else:                # lhu/lw/lwl/lwr – zero-extend halfword
                    val = val & 0xFFFF
            uc.reg_write(UC_MIPS_REG_0 + rt, val)

    def core_for(self, block: int):
        """The emulated DSP behind one host register block.

        Each of the card's DSPs has its own 8-byte register block and its own
        ADSP-2181, so they need separate cores: with a single shared core the
        second DSP's download lands in the first DSP's running image.  Cores
        are created on demand, held in IDMA boot mode until their own
        download writes PM 0.  `block` may be a kseg address.
        """
        if not self.multi_dsp:
            return self.cpu
        block &= 0x1fffffff
        self._active_block = block
        core = self.cores.get(block)
        if core is None:
            core = ADSP.adsp2181_create()
            ADSP.adsp2181_reset(core)
            ADSP.adsp2181_set_idma_boot_hold(core, 1)
            self.cores[block] = core
            self._idma_addrs[block] = 0
            if self.log:
                print(f"[mips] new DSP core for register block 0x{block:08x}")
        return core

    def _dsp_ports(self, address):
        """Split a DSP register access into (block base, port)."""
        port = address & 0xFF
        if port >= 0x80:
            return address - 0x80, port
        return address, port

    def _dsp_read(self, uc, access, address, size, value, user):
        # DSP register block: +0x80 = IDMA address port, +0x00 = data port.
        block, port = self._dsp_ports(address)
        if port >= 0x80:
            # Address port reads are rare; let the auto-mapped zero page
            # return 0.
            return True
        # Data port read — return IDMA data and force it into the
        # destination register so the read-back verify sees the value.
        val = ADSP.adsp2181_idma_data_read(self.core_for(block))
        if self.log:
            idma = self._idma_addrs.get(block & 0x1fffffff, self._idma_addr)
            tag = "DM" if idma & 0x4000 else "PM"
            print(f"[mips] dsp_read {tag} 0x{idma & 0x3fff:04x} -> 0x{val:04x}")
        # Patch the mapped page (fallback if Unicorn re-reads), and
        # directly set the destination register.
        uc.mem_write(address, struct.pack("<H", val))
        self._set_load_result(uc, val, size)
        return True

    def _dsp_write(self, uc, access, address, size, value, user):
        block, port = self._dsp_ports(address)
        core = self.core_for(block)
        value &= 0xFFFF
        if port >= 0x80:
            self._idma_addr = value
            self._idma_addrs[block & 0x1fffffff] = value
            ADSP.adsp2181_idma_addr_write(core, value)
            if self.log:
                print(f"[mips] dsp_addr 0x{value:04x}")
        else:
            ADSP.adsp2181_idma_data_write(core, value)
            self._core_last_write[block & 0x1fffffff] = self._insn_count
            idma = self._idma_addrs.get(block & 0x1fffffff, self._idma_addr)
            if self.log:
                tag = "DM" if idma & 0x4000 else "PM"
                print(f"[mips] dsp_write {tag} 0x{idma & 0x3fff:04x} = 0x{value:04x}")
            self.host_writes.append((idma, value))
        return True

    def _hostreg_write(self, uc, access, address, size, value, user):
        # Memory-mapped write to the host register block (hostreg_v region).
        # +0x80 = IDMA address port, +0x00 = IDMA data port (auto-increment).
        off = address - (RAM_BASE + 0x5000)
        if off == 0x80:
            self._idma_addr = value & 0xFFFF
            ADSP.adsp2181_idma_addr_write(self.cpu, value & 0xFFFF)
        elif off == 0x00:
            ADSP.adsp2181_idma_data_write(self.cpu, value & 0xFFFF)
            if self.log:
                tag = "DM" if self._idma_addr & 0x4000 else "PM"
                print(f"[mips] idma_write {tag} 0x{self._idma_addr & 0x7fff:04x} = 0x{value:04x}")
            self.host_writes.append((self._idma_addr, value))
        return True

    def _unmapped(self, uc, access, address, size, value, user):
        from unicorn import UC_MEM_READ_UNMAPPED, UC_MEM_WRITE_UNMAPPED
        from unicorn.mips_const import UC_MIPS_REG_PC, UC_MIPS_REG_RA
        pc = uc.reg_read(UC_MIPS_REG_PC)
        # Translate kseg0 (0x8xxx) / kseg1 (0xaxxx) to physical by clearing
        # the top 3 bits, then auto-map a zero page for data accesses.
        phys = address & 0x1fffffff
        page = phys & ~(AUTO_PAGE - 1)
        if access in (UC_MEM_READ_UNMAPPED, UC_MEM_WRITE_UNMAPPED):
            # Catch-all: auto-map any unmapped physical page.  The firmware
            # init accesses hardware registers (PCI config, interrupt
            # controller) at various addresses; mapping them as zero lets
            # the init proceed past hardware probes.
            if page not in self.mapped_pages:
                if self.log:
                    print(f"[mips] auto-map page phys=0x{page:06x} "
                          f"(touch @0x{address:08x} pc=0x{pc:08x})")
                # ensure_mapped consults the live region list: a bare
                # mem_map here throws when the page overlaps one of the
                # larger fixed mappings, which surfaces as an unmapped-access
                # error rather than the auto-map it is meant to perform.
                self.ensure_mapped(page, AUTO_PAGE)
            return True
        print(f"[mips] unmapped access {access} {address:08x} (phys 0x{phys:08x}) "
              f"sz={size} pc=0x{pc:08x} ra=0x{uc.reg_read(UC_MIPS_REG_RA):08x}")
        for entry in self.trace_log[-12:]:
            print(f"   {entry}")
        return False

    def _hook(self, uc, address, size, user):
        self.trace_log.append(f"{address:08x}")
        # Pump the ADSP in lockstep with the MIPS: every ADSP_PUMP_EVERY
        # MIPS instructions, run the ADSP a few cycles so the DSP can boot,
        # acknowledge, and process commands the MIPS downloads. Without this
        # the MIPS init hangs polling for a DSP boot-acknowledge that never
        # comes (the DSP never runs).
        self._insn_count = getattr(self, '_insn_count', 0) + 1
        if self.pump_every and self._insn_count % self.pump_every == 0:
            # Strobe IRQE (irq 6) to wake the DSP foreground from IDLE so it
            # runs code the MIPS just downloaded and writes the boot-ack.
            # Cores still in IDMA boot hold ignore the run and stay stopped.
            if self.multi_dsp:
                # Only the DSP the firmware is currently talking to runs: the
                # card brings its DSPs up one at a time, and every other core
                # is either still being downloaded into or waiting its turn.
                block = self._active_block
                core = self.cores.get(block)
                quiet = (self._insn_count -
                         self._core_last_write.get(block, -1 << 30))
                runnable = [core] if core is not None and quiet > self.dsp_write_quiet else []
            else:
                runnable = [self.cpu]
            for core in runnable:
                ADSP.adsp2181_set_irq(core, 6, 1)
                ADSP.adsp2181_run(core, 2000)
                ADSP.adsp2181_set_irq(core, 6, 0)
                ADSP.adsp2181_run(core, 1000)
        from unicorn.mips_const import (UC_MIPS_REG_PC, UC_MIPS_REG_RA,
                                        UC_MIPS_REG_A0, UC_MIPS_REG_A1,
                                        UC_MIPS_REG_A2, UC_MIPS_REG_V0)
        if address == HOST_WRITE:
            a0 = uc.reg_read(UC_MIPS_REG_A0)
            a1 = uc.reg_read(UC_MIPS_REG_A1) & 0xFFFF
            a2 = uc.reg_read(UC_MIPS_REG_A2) & 0xFFFF
            if self.log:
                print(f"[mips] host_write [0x{a0:08x}] {a1:04x} = {a2:04x}")
            self.host_writes.append((a1, a2))
            ADSP.adsp2181_host_write(self.core_for(a0), a1, a2)
            self._core_last_write[a0 & 0x1fffffff] = self._insn_count
            uc.reg_write(UC_MIPS_REG_PC, uc.reg_read(UC_MIPS_REG_RA))
        elif address == HOST_READ:
            a0 = uc.reg_read(UC_MIPS_REG_A0)
            a1 = uc.reg_read(UC_MIPS_REG_A1) & 0xFFFF
            value = ADSP.adsp2181_host_read(self.core_for(a0), a1)
            if self.log:
                print(f"[mips] host_read [0x{a0:08x}] {a1:04x} -> {value:04x}")
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
        self.host_writes = []
        uc.emu_start(entry, STUB_VIRT + 0x20, count=max_insns)
        return uc.reg_read(UC_MIPS_REG_V0)

    def write32(self, virt: int, value: int) -> None:
        self.uc.mem_write(virt & 0x1fffffff, struct.pack("<I", value & 0xffffffff))

    def write16(self, virt: int, value: int) -> None:
        self.uc.mem_write(virt & 0x1fffffff, struct.pack("<H", value & 0xffff))

    def write8(self, virt: int, value: int) -> None:
        self.uc.mem_write(virt & 0x1fffffff, bytes([value & 0xff]))

    def ensure_mapped(self, virt: int, size: int) -> None:
        """Map every auto-page the range [virt, virt+size) falls in.

        The fixed mappings are larger than AUTO_PAGE, so membership in
        `mapped_pages` is not enough — check the live region list.
        """
        # Map at Unicorn's 4K granularity: a 64K unit can straddle the edge
        # of an existing region, and mem_map rejects any overlap.
        page_size = 0x1000
        phys = virt & 0x1fffffff
        regions = [(begin, end) for begin, end, _perms in self.uc.mem_regions()]
        first = phys & ~(page_size - 1)
        last = (phys + size - 1) & ~(page_size - 1)
        for page in range(first, last + page_size, page_size):
            if any(begin <= page <= end for begin, end in regions):
                continue
            self.uc.mem_map(page, page_size)
            self.mapped_pages.add(page)
            regions.append((page, page + page_size - 1))

    def write_bytes(self, virt: int, data: bytes) -> None:
        self.ensure_mapped(virt, len(data))
        self.uc.mem_write(virt & 0x1fffffff, data)

    def alloc(self, virt_base: int, words: int) -> int:
        """Reserve a zeroed guest block; returns the guest-visible pointer."""
        self.uc.mem_write(virt_base & 0x1fffffff, bytes(words * 4))
        return virt_base


def symbol_address(metadata: dict, index: int) -> int:
    """Resolve a download symbol to its bare DSP address (no IDMA type bit)."""
    symbol = metadata["symbols"][index]
    segment = symbol["segment"]
    if segment < 4:
        return symbol["offset"]
    seg = next(s for s in metadata["segments"] if s["number"] == segment)
    return seg["base"] + symbol["offset"]


def symbol_host_address(metadata: dict, index: int) -> int:
    """Resolve a download symbol to a host-port address, as 0x800a6204 does.

    The firmware ORs in 0x4000 for data memory and leaves it clear for
    program memory, which is the ADSP-2181 IDMA "destination type" bit.  For
    the fixed segments 0-3 that split is by segment number (0/2 = DM, 1/3 =
    PM, matching memory blocks 0-3); for relocatable segments it comes from
    the target memory block's `memory_type & 1` (1 = PM).
    """
    symbol = metadata["symbols"][index]
    segment = symbol["segment"]
    if segment < 4:
        is_pm = segment in (1, 3)
        return symbol["offset"] | (0 if is_pm else 0x4000)
    seg = next(s for s in metadata["segments"] if s["number"] == segment)
    block = next(b for b in metadata["memory_blocks"]
                 if b["number"] == seg["memory_block"])
    is_pm = bool(block["type"] & 1)
    return (seg["base"] + symbol["offset"]) | (0 if is_pm else 0x4000)


def run_assign(shim: "MipsShim", args) -> None:
    """Synthesize the TIKRNL download/task struct and run the service-assign
    entry (0x80096980) so the real firmware performs the switch-on database
    commit through the hooked host port.

    The struct layout follows the field accesses in the 0x80096980 prologue:
      a0 (s3): assign request
        +0x00 -> base (s2); per-channel state is s1 = s2 + 0x200
        +0x04 -> resource struct (s0); NULL here returns immediately
        +0x08 -> existing mailbox; 0 for a fresh assign
        +0x18 -> channel/timeslot byte
      s0 (resource):
        +0x04 -> download/task descriptor; *(desc+0) = download id (0x0258)
        +0x40 -> task id halfword (0x0258)
        +0x140 -> mailbox state (0 to take the fresh-assign branch)
      desc (download descriptor):
        +0x00 -> download id (0x0258)
        +0x24 -> relocation/symbol table pointer
    The descriptor and per-channel state are large; zero-initialize them and
    let the auto-map hook supply .bss lookups.  Firmware host_write calls
    during assign are the switch-on database commit.
    """
    import json
    metadata = json.loads((args.tikrnl / "metadata.json").read_text())
    write13 = symbol_host_address(metadata, 13)  # 0x3310 host->TIKRNL mailbox
    write14 = symbol_host_address(metadata, 14)  # 0x3338 TIKRNL->host mailbox

    # Guest RAM layout (RAM_VIRT + offset; API uses physical = RAM_BASE + off):
    #   0x4000 assign request (s3)        0x60 bytes
    #   0x4100 resource struct (s0)       0x200 bytes
    #   0x4400 base block (s2) / s1=+0x200 0x400 bytes
    #   0x4900 download descriptor         0x100 bytes
    #   0x4a00 relocation/symbol table     0x200 bytes
    #   0x4d00 channel context (chctx)     0x100 bytes
    #   0x4e00 channel descriptor (desc2)  0x200 bytes
    #   0x5000 host register block         0x40 bytes
    #   0x5100 scratch words               0x100 bytes
    gp = GP
    sp = STACK_TOP
    base_v = RAM_VIRT + 0x4400
    res_v = RAM_VIRT + 0x4100
    desc_v = RAM_VIRT + 0x4900
    reloc_v = RAM_VIRT + 0x4a00
    chctx_v = RAM_VIRT + 0x4d00
    desc2_v = RAM_VIRT + 0x4e00
    hostreg_v = RAM_VIRT + 0x5000
    scratch_v = RAM_VIRT + 0x5100
    shim.alloc(RAM_VIRT + 0x4000, 0x2000)

    # Download descriptor: id 0x0258, relocation table at reloc_v.
    shim.write16(desc_v + 0x00, 0x0258)        # download id (low half)
    shim.write32(desc_v + 0x24, reloc_v)        # relocation/symbol table

    # Populate the relocation/symbol table from metadata.json.  Each entry is
    # 8 bytes; db_record_append reads the resolved DM address as a halfword at
    # entry+4 (reloc_table + symbol_index*8 + 4).  Only DM-resident symbols
    # (segments in memory_block 2 = DM) are meaningful here.
    for sym in metadata["symbols"]:
        seg = next((s for s in metadata["segments"]
                    if s["number"] == sym["segment"]), None)
        if sym["segment"] < 4:
            dm_addr = sym["offset"]
        elif seg and seg["memory_block"] == 2:  # DM
            dm_addr = seg["base"] + sym["offset"]
        else:
            continue  # PM symbol — not a DM database target
        shim.write16(reloc_v + sym["id"] * 8 + 4, dm_addr)

    # Resource struct: +4 -> descriptor, +0x40 = task id, +0x140 = 0 (fresh).
    shim.write32(res_v + 0x04, desc_v)
    shim.write16(res_v + 0x40, 0x0258)

    # Base (s2): the card/channel struct the firmware assumes already exists.
    #   +0x0c -> channel context (chctx)
    #   +0x10 -> host register block (also written to s1+0 and the mailbox)
    # chctx+0x24 -> the download symbol table (db_record_append a2!=0 path
    # resolves symbol index -> DM address at entry+4).  Point this at the
    # populated TIKRNL symbol table (reloc_v) so records resolve correctly.
    shim.write32(base_v + 0x0c, chctx_v)
    shim.write32(base_v + 0x10, hostreg_v)
    shim.write32(chctx_v + 0x24, reloc_v)
    shim.write16(desc2_v + 0x106, 0x0000)

    # Per-channel state s1 = base + 0x200.  s1+0x6c must point to a writable
    # word (the routine does `sh zero, ($v1)` through it).
    shim.write32(base_v + 0x200 + 0x6c, scratch_v)
    shim.write32(base_v + 0x200 + 0x70, scratch_v + 4)
    # s1+0x04 -> resource struct, so db_record_append's a2==0 path resolves
    # *(s1+4) -> res -> +4 -> desc -> +0x24 -> relocation/symbol table.
    shim.write32(base_v + 0x200 + 0x04, res_v)

    # Per-channel command mailbox at s1+0x24 (= base+0x224).  0x80093d14 calls
    # 0x80086af8 (DSP handshake wait) on it before the db commit; for a fresh
    # assign the active flag (+0x10) makes it return nonzero immediately so
    # the switch-on database commit (0x80090e58) actually runs.
    #   +0x08 -> host register block; +0x0c -> symbol-13 mailbox descriptor;
    #   +0x10 (byte) active = 1; +0x12 halfword = 0; +0x14 -> ring descriptor.
    mbox_v = base_v + 0x224
    shim.write32(mbox_v + 0x08, hostreg_v)
    shim.write32(mbox_v + 0x0c, reloc_v)   # any valid descriptor; type 0 path
    shim.write8(mbox_v + 0x10, 1)           # active -> 0x80086af8 returns nonzero
    shim.write16(mbox_v + 0x12, 0x0000)
    shim.write32(mbox_v + 0x14, reloc_v)

    # Database ring descriptor in the channel state (s6/s3 = s1 = base+0x200).
    # db_ring_commit (0x8008dd14) reads the DSP producer via host_read at
    # *(s3+0xc)+1, then validates (producer - base) < length before writing.
    #   s3+0x0c = producer address - 1, already carrying the IDMA type bit
    #             (db_ring_commit host_reads s3+0xc+1 with no further masking)
    #   s3+0x10 (byte) = ring memory select: zero makes 0x8008dda8 OR in
    #             0x4000, i.e. a DM ring; nonzero leaves the address in PM
    #   s3+0x12 = bare ring base address;  s3+0x14 = ring length
    # The TIKRNL symbol-13 command ring is in DM at 0x3327 (16 words) with its
    # producer pointer at DM 0x3315 (initialised to 0x3327 by the TIKRNL
    # initializer at PM 0x672).  The switch-on database commit targets this
    # ring: the DSP's TIKRNL consumer polls DM 0x3315/0x3316 and processes
    # records from DM 0x3327.
    ring_v = base_v + 0x200
    shim.write16(ring_v + 0x0c, write13 + 0x05 - 1)  # producer addr - 1 = 0x7314
    shim.write8(ring_v + 0x10, 0x00)                  # zero -> DM ring (+0x4000)
    shim.write16(ring_v + 0x12, symbol_address(metadata, 13) + 0x17)  # bare 0x3327
    shim.write16(ring_v + 0x14, 0x0010)               # ring length (16)

    # Assign request (s3): +0 -> base, +4 -> resource, +8 = 0, +0x18 = channel.
    shim.write32(RAM_VIRT + 0x4000 + 0x00, base_v)
    shim.write32(RAM_VIRT + 0x4000 + 0x04, res_v)
    shim.write32(RAM_VIRT + 0x4000 + 0x08, 0)
    shim.write8(RAM_VIRT + 0x4000 + 0x18, args.channel)

    print(f"[assign] calling 0x{SERVICE_ASSIGN:08x} req=0x{RAM_VIRT+0x4000:08x} "
          f"base=0x{base_v:08x} res=0x{res_v:08x} desc=0x{desc_v:08x} "
          f"ch={args.channel} mb13=0x{write13:04x} mb14=0x{write14:04x}")
    try:
        v0 = shim.call(SERVICE_ASSIGN, [RAM_VIRT + 0x4000], gp=gp, sp=sp,
                       max_insns=4000000)
    except Exception as exc:
        from unicorn.mips_const import UC_MIPS_REG_PC, UC_MIPS_REG_RA
        pc = shim.uc.reg_read(UC_MIPS_REG_PC)
        ra = shim.uc.reg_read(UC_MIPS_REG_RA)
        print(f"[assign] fault: {exc} pc=0x{pc:08x} ra=0x{ra:08x}")
        print("  recent:")
        for e in shim.trace_log[-16:]:
            print(f"    {e}")
        return
    print(f"[assign] returned v0=0x{v0:08x} host_writes={len(shim.host_writes)}")
    if args.log:
        # Show the last PCs in the assign call to see which return path fired.
        print("  assign trace tail:")
        for e in shim.trace_log[-24:]:
            print(f"    {e}")
    # Read back the database record buffer (0x80331c12) to see what
    # db_record_append produced.
    buf = shim.uc.mem_read(0x00331c12, 0x80)
    nonzero = next((i for i, b in enumerate(buf) if b), None)
    if nonzero is not None:
        print(f"  db buffer @0x80331c12 has data from offset {nonzero}: "
              + " ".join(f"{b:02x}" for b in buf[:64]))
    else:
        print("  db buffer @0x80331c12 is empty (db_record_append produced nothing)")
    if shim.log and shim.host_writes:
        for addr, val in shim.host_writes[:64]:
            tag = "DM" if addr & 0x4000 else "PM"
            print(f"  host_write {tag} 0x{addr & 0x7fff:04x} = 0x{val:04x}")


def stage_dsp_code(shim: "MipsShim", args) -> int:
    """Write the DSP download image to card RAM and point the header at it.

    Reproduces pri_telindus_load (kernel/s_pri.c): the image goes at
    DspCodeBaseAddr — the protocol image's own OFFS_PROTOCOL_END_ADDR,
    dword-aligned — and that address is written back into the image header
    at OFFS_DSP_CODE_BASE_ADDR, which is where the MIPS entry reads it from
    (`lw $s1, 0x106c($s1)` with the image based at 0xa0011000).
    """
    base = args.dsp_code_base
    if base is None:
        base = protocol_end_addr(args.image)
    image = build_dsp_code_image(args.dsp_combifile, args.card_type, base)
    shim.write_bytes(base, image.data)
    # The header field is read through the uncached image alias; write it via
    # the physical address the image was loaded at.
    shim.write32(BIAS + OFFS_DSP_CODE_BASE_ADDR, base)
    print(f"[mainloop] DSP code staged at 0x{base:08x}..0x{image.end_addr:08x} "
          f"({len(image.data)} bytes, {len(image.downloads)} downloads, "
          f"card type {image.card_type} -> file set {image.file_set})")
    for entry in image.downloads:
        if entry.download_id in (0x0258, 0x0261, 0x026A, 0x025F):
            print(f"           id=0x{entry.download_id:04x} @0x{entry.address:08x} "
                  f"{entry.description}")
    return base


DSP_BOOT_MAILBOX = 0x3FFF   # kernel symbol 0: PM 0x3fff
DSP_BOOT_PROBE = 0x5A5A     # written by 0x800a77e0 before the download
DSP_BOOT_ACK = 0xA5A5       # polled for by 0x800a78d0 afterwards


def report_dsp_boot(shim: "MipsShim", cycles: int = 200000) -> int:
    """Run each downloaded DSP and report the boot handshake result.

    The validator (0x80082130 -> 0x800a77e0) writes 0x5a5a to the download's
    symbol 0, streams the kernel in, releases the core, then polls that word
    for 0xa5a5.  This runs the cores the firmware has finished downloading
    and reports how many produce the acknowledgement, which is what the
    handshake is waiting on.
    """
    if not shim.cores:
        return 0
    held = acked = 0
    for block, core in sorted(shim.cores.items()):
        if ADSP.adsp2181_idma_boot_held(core):
            held += 1
            continue
        ADSP.adsp2181_run(core, cycles)
        if ADSP.adsp2181_host_read(core, DSP_BOOT_MAILBOX) == DSP_BOOT_ACK:
            acked += 1
        elif shim.log:
            pm = ADSP.adsp2181_pm(core)
            print(f"[dsp] block 0x{block:08x} no ack: "
                  f"pm[0x{DSP_BOOT_MAILBOX:04x}]=0x{pm[DSP_BOOT_MAILBOX]:06x}")
    print(f"[dsp] {len(shim.cores)} cores: {acked} answered the boot handshake "
          f"with 0x{DSP_BOOT_ACK:04x}, {held} still held (no download)")
    return acked


def run_mainloop(shim: "MipsShim", args) -> None:
    """Drive the MIPS via its native PR_RAM request queue (the real host
    interface).  Maps shared RAM, runs the MIPS init, sets up the PR_RAM
    buffer chain, writes a modem ASSIGN request, and calls the main loop.

    This is the path the Linux driver uses: the host writes IDI requests
    to PR_RAM, the MIPS polls and dispatches them, calling dsp_assign
    and downloading DSP overlays internally.
    """
    import json
    metadata = json.loads((args.tikrnl / "metadata.json").read_text())
    write13 = symbol_host_address(metadata, 13)
    write14 = symbol_host_address(metadata, 14)

    gp = GP
    sp = STACK_TOP

    # 0. Stage the DSP code image, as the host driver's pri_telindus_load
    #    does, and publish its address in the protocol image header.  The
    #    firmware entry reads the count at DspCodeBaseAddr and the descriptor
    #    table right after it; with no image the count is 0, every DSP object
    #    is built with an empty code table and no overlay can be assigned.
    # Each DSP register block gets its own emulated ADSP, all held in IDMA
    # boot mode: the firmware downloads a kernel into every one of them and
    # a shared core would see each download land in the previous DSP's
    # running image.
    shim.multi_dsp = True
    # Downloads dominate this path, and the card holds a DSP for the whole
    # transfer.  Interleaving DSP execution with it is still unstable (a core
    # that starts on a partial image runs wild), so the pump is off by
    # default here; --dsp-pump N turns it back on.
    shim.pump_every = args.dsp_pump

    if args.dsp_combifile is not None:
        stage_dsp_code(shim, args)

    # 1. Write the card config and boot command the firmware entry reads
    #    during init.  The request queue is NOT set up here: the firmware
    #    initialises PR_RAM itself as it boots and publishes its signature
    #    when ready, so a request written beforehand is overwritten.
    sr = PR_RAM_PHYS
    shim.write8(sr + 0x08, 0)      # TEI (0 = auto)
    shim.write8(sr + 0x10, 1)      # ForceLaw = a-law (E1)
    shim.write8(sr + 0x16, 0x80)    # DSPInfo = DSP code loaded
    shim.write8(sr + 0x1a, 12)     # CardType = CARD_MAEP (PRI 30M)
    shim.write8(sr + 0xe0, 0)      # PCINIT_END_OF_LIST
    shim.write32(0x00, 3)          # boot->cmd = 3 (start)
    shim.write32(0x04, 0xa0011000) # boot->addr
    print("[mainloop] card config + boot command written")

    # 2. Call the firmware entry (0x80082f90) to store the PR_RAM pointer
    #    and run basic init.  It reaches a self-loop waiting for a hardware
    #    interrupt; the instruction count stops it there.  The PR_RAM
    #    pointer (gp+0x5e93) is now set.
    print("[mainloop] running firmware entry (basic init)...")
    try:
        shim.call(MIPS_ENTRY, [], gp=gp, sp=sp, max_insns=5000000)
    except Exception as exc:
        print(f"[mainloop] entry stopped at self-loop: {exc}")

    # 3. The self-loop at 0x800830ec waits for a hardware interrupt.  In real
    #    hardware the host triggers this after writing config + boot command.
    #    Skip it and call the post-wait init + main loop directly.
    #    0x80083100 calls 0x80083d10 (init), 0x8002a534 (init), then loops
    #    on 0x80027970 (main loop).  Call the two init functions, then the
    #    main loop separately.
    MIPS_POST_INIT1 = BIAS + 0x72d10   # 0x80083d10
    MIPS_POST_INIT2 = BIAS + 0x1a534   # 0x8002a534
    print("[mainloop] running post-wait init functions...")
    try:
        shim.call(MIPS_POST_INIT1, [], gp=gp, sp=sp, max_insns=2000000)
    except Exception as exc:
        print(f"[mainloop] init1 fault: {exc}")
    try:
        shim.call(MIPS_POST_INIT2, [], gp=gp, sp=sp, max_insns=2000000)
    except Exception as exc:
        print(f"[mainloop] init2 fault: {exc}")

    # Check if DSP resources were registered
    dsp_table = struct.unpack_from("<H",
        shim.uc.mem_read((gp + 0x5eb9) & 0x1fffffff, 2))[0]
    init_state = struct.unpack_from("<H",
        shim.uc.mem_read((gp + 0x5e81) & 0x1fffffff, 2))[0]
    print(f"[mainloop] after init: gp+0x5e81={init_state:#06x} "
          f"gp+0x5eb9={dsp_table:#06x}")
    report_dsp_boot(shim)

    # 3b. Post the modem ASSIGN now that the firmware has initialised PR_RAM,
    #     following pr_out() in the Linux driver (kernel/di.c): fill the REQ
    #     at B[NextReq], advance NextReq to REQ->next, bump ReqInput.
    sig = struct.unpack_from("<H", shim.uc.mem_read(sr + PR_Signature, 2))[0]
    next_req = struct.unpack_from("<H", shim.uc.mem_read(sr + PR_NextReq, 2))[0]
    cai = bytearray(26)
    cai[0] = 25; cai[1] = DSP_CAI_HARDWARE_MODEM_ASYNC; cai[8] = 0x04
    struct.pack_into("<H", cai, 15, 56000)
    struct.pack_into("<H", cai, 19, 56000)
    rb = sr + PR_B + next_req
    req_next = struct.unpack_from("<H", shim.uc.mem_read(rb + REQ_NEXT, 2))[0]
    shim.write8(rb + REQ_REQ, ASSIGN)
    shim.write8(rb + REQ_REQID, NL_ID)
    shim.write8(rb + REQ_REQCH, args.channel)
    shim.write16(rb + REQ_XBUFFER, len(cai))
    shim.uc.mem_write(rb + REQ_XDATA, bytes(cai))
    shim.write16(sr + PR_NextReq, req_next)
    # ReqInput/ReqOutput are free-running byte counters and the firmware owns
    # ReqOutput; the host's counter starts level with it, then one increment
    # per posted request (pr_out()).  Syncing first is what makes the main
    # loop see exactly this one request.
    req_out = shim.uc.mem_read(sr + PR_ReqOutput, 1)[0]
    shim.write8(sr + PR_ReqInput, (req_out + 1) & 0xff)
    print(f"[mainloop] ASSIGN posted: Sig=0x{sig:04x} NextReq=0x{next_req:04x} "
          f"-> 0x{req_next:04x} ReqInput {req_out}->{(req_out + 1) & 0xff}")

    # 4. Run the main loop.
    for i in range(args.words if args.words > 0 else 50):
        try:
            v0 = shim.call(MIPS_MAINLOOP, [], gp=gp, sp=sp, max_insns=500000)
        except Exception as exc:
            from unicorn.mips_const import UC_MIPS_REG_PC, UC_MIPS_REG_RA
            pc = shim.uc.reg_read(UC_MIPS_REG_PC)
            ra = shim.uc.reg_read(UC_MIPS_REG_RA)
            print(f"[mainloop] fault at iteration {i}: {exc} "
                  f"pc=0x{pc:08x} ra=0x{ra:08x}")
            print("  recent:")
            for e in shim.trace_log[-16:]:
                print(f"    {e}")
            return
        ri = shim.uc.mem_read(PR_RAM_PHYS + PR_ReqInput, 1)[0]
        ro = shim.uc.mem_read(PR_RAM_PHYS + PR_ReqOutput, 1)[0]
        sig = struct.unpack_from("<H", shim.uc.mem_read(PR_RAM_PHYS + PR_Signature, 2))[0]
        if i < 5 or i % 10 == 0 or ri != ro:
            print(f"[mainloop] iter {i}: v0=0x{v0:08x} ReqIn={ri} ReqOut={ro} "
                  f"Sig=0x{sig:04x} host_writes={len(shim.host_writes)}")
        if ri == ro and i > 0:
            # Request consumed; check DSP state
            break

    print(f"[mainloop] done: host_writes={len(shim.host_writes)}")
    if shim.log and shim.host_writes:
        for addr, val in shim.host_writes[:32]:
            tag = "DM" if addr & 0x4000 else "PM"
            print(f"  host_write {tag} 0x{addr & 0x7fff:04x} = 0x{val:04x}")


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
    parser.add_argument("--assign", action="store_true",
                        help="run the service assign entry (0x80096980) to "
                             "perform the switch-on database commit before "
                             "the command-script path")
    parser.add_argument("--mainloop", action="store_true",
                        help="drive the MIPS via its native PR_RAM request "
                             "queue (the real host interface), writing a modem "
                             "ASSIGN request and running the main loop")
    parser.add_argument("--channel", type=int, default=1,
                        help="E1 timeslot / channel byte written at req+0x18")
    parser.add_argument("--dsp-combifile", type=Path,
                        default=Path("docs/firmware/dspdload.bin"),
                        help="DSP download combifile staged in card RAM for "
                             "--mainloop (pass an empty value to skip)")
    parser.add_argument("--card-type", type=lambda s: int(s, 0),
                        default=CARDTYPE_DIVASRV_P_30M_PCI,
                        help="CARDTYPE_* number selecting the combifile's "
                             "required download set (23 = PRI 30M PCI)")
    parser.add_argument("--dsp-pump", type=int, default=0,
                        help="MIPS instructions between DSP time slices during "
                             "--mainloop (0 = hold the DSPs for the download, "
                             "the default; they are run afterwards)")
    parser.add_argument("--dsp-code-base", type=lambda s: int(s, 0), default=None,
                        help="override DspCodeBaseAddr (default: the protocol "
                             "image's OFFS_PROTOCOL_END_ADDR)")
    parser.add_argument("--log", action="store_true")
    args = parser.parse_args()
    if args.dsp_combifile is not None and not str(args.dsp_combifile):
        args.dsp_combifile = None

    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    if args.mainloop:
        # The firmware downloads the DSP's own kernel over IDMA, so nothing is
        # pre-staged here.  Hold the core in IDMA boot mode until that
        # download writes PM 0; a DSP left running would execute its
        # half-replaced image and corrupt the transfer (the download's own
        # read-back verify catches it).
        ADSP.adsp2181_set_idma_boot_hold(cpu, 1)
        print("[adsp] held in IDMA boot mode; firmware downloads the kernel")
    else:
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

    # The firmware's trace-printf pointer (gp+0x1a7b = 0x800fbe30) is
    # file-backed and points at the real printf (0x80083180), which writes to
    # the hardware trace buffer at 0xa0005d20.  Redirect it to the no-op stub
    # so trace calls return immediately instead of faulting on that buffer.
    shim.write32(0x800fbe30, STUB_VIRT)

    if args.mainloop:
        run_mainloop(shim, args)
        return 0

    if args.assign:
        run_assign(shim, args)

    # Fabricate the request struct (see dsp_assign 0x0258 tail + sender).
    req = RAM_VIRT + 0x1000  # guest-visible pointer
    buf = bytearray(0x60)
    struct.pack_into("<II", buf, 0x00, 0xDEAD0000, 0xDEAD0004)  # host regs
    # symbol 13/14 mailbox addresses, in the form the firmware's own resolver
    # (0x800a6204) produces: 0x4000 selects data memory.
    struct.pack_into("<HH", buf, 0x08, 0x4000 | 0x3310, 0x4000 | 0x3338)
    buf[0x0C] = 1                     # active
    struct.pack_into("<H", buf, 0x12, args.selector)    # command selector
    struct.pack_into("<H", buf, 0x3E, 0x0020)           # control word
    shim.uc.mem_write(RAM_BASE + 0x1000, bytes(buf))  # API uses physical

    # Top-level byte request: [len, ?, form, code, mode] selects a script.
    outer = bytes([4, 0, 0, args.code, args.mode])
    shim.uc.mem_write(RAM_BASE + 0x2000, outer)
    # Context struct: +0x20 = pointer to the byte request.
    ctx = bytearray(0x40)
    struct.pack_into("<I", ctx, 0x20, RAM_VIRT + 0x2000)
    shim.uc.mem_write(RAM_BASE + 0x3000, bytes(ctx))

    # parser(a0=request, a1=context) then sender(a0=request, a1=context)
    v0 = shim.call(REQUEST_PARSER, [req, RAM_VIRT + 0x3000],
                   gp=GP, sp=STACK_TOP)
    if args.log:
        print(f"[mips] parser -> {v0:#x}")
    if v0:
        shim.call(SCRIPT_SENDER, [req, RAM_VIRT + 0x3000],
                  gp=GP, sp=STACK_TOP)

    # pump the 8 kHz host loop so the DSP consumes the command
    for addr in (0x3315, 0x3316, 0x3310, 0x3338, 0x2f28, 0x2e44, 0x2e45):
        ADSP.adsp2181_watch_dm(cpu, addr, 1)
    ADSP.adsp2181_watch_pm(cpu, 0x3327, 1)
    for _ in range(args.words):
        ADSP.adsp2181_set_irq(cpu, 3, 1)  # SPORT0_RX
        ADSP.adsp2181_set_irq(cpu, 3, 0)
        # IRQE (irq 6) wakes the kernel foreground from IDLE so it runs
        # the command-queue processor (analysis session 3/4: the host
        # doorbell is IRQE, vector 0x18). Without it TIKRNL's consumer
        # never advances past 0x3327.
        ADSP.adsp2181_set_irq(cpu, 6, 1)  # IRQE doorbell
        ADSP.adsp2181_run(cpu, 5000)
        ADSP.adsp2181_set_irq(cpu, 6, 0)
        ADSP.adsp2181_run(cpu, 5000)
        # Call TIKRNL's frame handler (PM 0x64A) directly so its command
        # consumer (0x1810, reads DM 0x3309 -> command ring) runs and
        # advances the consumer pointer at DM 0x3316. Without this the
        # dispatch loop is circular: TIKRNL only runs when dispatched,
        # but dispatch needs TIKRNL to have registered.
        ADSP.adsp2181_call(cpu, 0x64A, 0x02A8)
        ADSP.adsp2181_run(cpu, 20000)
    # Host-port reads need the 0x4000 data-memory select; a bare address
    # selects program memory (see symbol_host_address).
    dm_sel = 0x4000
    print(f"[adsp] selector DM3310={ADSP.adsp2181_host_read(cpu, dm_sel | 0x3310):04x}"
          f" producer DM3315={ADSP.adsp2181_host_read(cpu, dm_sel | 0x3315):04x}"
          f" consumer DM3316={ADSP.adsp2181_host_read(cpu, dm_sel | 0x3316):04x}"
          f" resp DM3338={ADSP.adsp2181_host_read(cpu, dm_sel | 0x3338):04x}")
    # dump the channel table and free-list to see if the descriptor got hooked
    dm = ADSP.adsp2181_dm(cpu)
    print("[adsp] channel table: 2E44=%04x 2E45=%04x  queue 2F08=%04x 2F09=%04x"
          % (dm[0x2E44], dm[0x2E45], dm[0x2F08], dm[0x2F09]))
    print("[adsp] free-list 2F27=%04x 2F28=%04x" % (dm[0x2F27], dm[0x2F28]))
    print("[adsp] DM 2E00-2E10: " + " ".join("%04x" % dm[a] for a in range(0x2E00, 0x2E11)))
    print("[adsp] DM 2F00-2F10: " + " ".join("%04x" % dm[a] for a in range(0x2F00, 0x2F11)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
