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
import json
import math
import os
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

from unicorn import Uc, UC_ARCH_MIPS, UC_MODE_LITTLE_ENDIAN, UC_MODE_32
from unicorn import UC_HOOK_CODE

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dial_tikrnl_drive import sport_rx_word
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

# See the page-14 overlay-load site below. Default keeps the diagnostic that
# reaches outer state 0x0080; EICON_V90D_BULK_ADAPTER=1 restores the adapter.
V90D_BULK_ADAPTER_DISABLED = os.environ.get("EICON_V90D_BULK_ADAPTER", "0") != "1"
# Hold the six-word mapping-frame block across the resident kernel's per-frame
# clear; see the page-14 continuation site below. EICON_V90D_TX_BLOCK_HOLD=0
# restores the old behaviour (one downstream sample in six).
V90D_HOLD_TX_BLOCK = os.environ.get("EICON_V90D_TX_BLOCK_HOLD", "1") != "0"

HOST_WRITE = BIAS + 0x71950  # 0x80082950
HOST_READ = BIAS + 0x71920   # 0x80082920
HOST_WRITE_DM_BLOCK = BIAS + 0x71A38  # 0x80082a38
HOST_WRITE_PM_BLOCK = BIAS + 0x71B8C  # 0x80082b8c
SCRIPT_SENDER = BIAS + 0x786A4  # 0x800896a4
REQUEST_PARSER = BIAS + 0x78138  # 0x80089138
# Service-driver table slot 1: the modem service assign entry (file 0x85980).
# Performs the switch-on database commit for task 0x0258 (TIKRNL81.F34),
# reached through the table at file 0xeaec4 rather than by a direct jal.
SERVICE_ASSIGN = BIAS + 0x85980  # 0x80096980
SWITCH_ON = BIAS + 0x7FE58       # 0x80090e58, publish initial task command
DSP_DOWNLOAD = BIAS + 0x75AF8    # 0x80086af8, native block/relocation loader

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
PR_ReqReserved = 0x08 # byte: Req buffers reserved
PR_XLock = 0x0a     # byte: arbitration lock
PR_RcOutput = 0x0b  # byte: count of RC buffers the MIPS has returned
PR_IndOutput = 0x0c # byte: count of IND buffers the MIPS has returned
PR_IMask = 0x0d     # byte: interrupt mask flag
PR_ReadyInt = 0x10  # byte: host pokes this to request a ready interrupt
PR_Signature = 0x1e # word: MIPS writes 0x5858 (not ready) or valid sig
PR_B = 0x20         # start of the REQ/RC/IND buffer area

# RC structure (kernel/pr_pc.h): next(2) Rc(1) RcId(1) RcCh(1) Res(1) Ref(2)
RC_RC = 0x02
RC_RCID = 0x03
RC_RCCH = 0x04
RC_REFERENCE = 0x06
IND_IND = 0x02
IND_ID = 0x03
IND_CH = 0x04
IND_REFERENCE = 0x08
IND_RBUFFER = 0x10
IND_RDATA = 0x12
# Return codes (kernel/pc.h).
ASSIGN_RC = 0xe0    # ASSIGN acknowledgement class
ASSIGN_OK = 0xef    # ASSIGN succeeded
RC_OK = 0xff        # command accepted

# REQ structure (from kernel/pr_pc.h).  Each REQ buffer in B[].
REQ_SIZE = 0x120    # next(2)+Req(1)+ReqId(1)+ReqCh(1)+Res1(1)+Ref(2)+Res[8]+XBuffer(2+270)
REQ_NEXT = 0x00     # word: offset of next free REQ in B[]
REQ_REQ = 0x02      # byte: request code (ASSIGN=0x01, etc.)
REQ_REQID = 0x03    # byte: global entity id (DSIG_ID, NL_ID, ...)
REQ_REQCH = 0x04    # byte: channel number
REQ_REFERENCE = 0x06  # word: host cookie (0 = signalling, 1 = network)
REQ_XBUFFER = 0x10  # PBUFFER: word length + byte[270] data
REQ_XDATA = 0x12    # start of the 270-byte data payload

# IDI request codes and global entity ids (kernel/pc.h).
ASSIGN = 0x01
LISTEN_REQ = 0x02
N_CONNECT = 0x02
INDICATE_REQ = 0x0a
CALL_RES = 0x0b
DSIG_ID = 0x00    # D-channel signalling
NL_ID = 0x20      # network-layer access (B or D channel)
BLLC_ID = 0x60    # B-channel link level access
TASK_ID = 0x80    # dynamic user tasks
MAN_ID = 0xe0     # management
REMOVE = 0xff

# Signalling-controller object vector used by the common IDI dispatcher.
# gp+0x5eb9 holds the count; entries are firmware runtime pointers and must
# be accessed through Unicorn's physical mirror.
ENTITY_TABLE = 0x80299928
# Complete incoming signalling-message parser.  Do not confuse this with
# 0x800172a8: that address is only the delay slot of a jal to the IE-copy
# helper at the end of the preceding function.
CALL_INGRESS_PARSER = 0x800172c0
SYNTH_CALL_OBJECT = RAM_VIRT + 0x7000
SYNTH_INGRESS_MESSAGE = RAM_VIRT + 0x7800

# DSP CAI modem hardware types (kernel/mdm_msg.h).  add_b1()'s resource[]
# table maps B1 protocol 7/8 (MODEM_ALL_NEGOTIATE / MODEM_ASYNC) to 17 and
# protocol 9 (MODEM_SYNC_HDLC) to 18, so a modem call's B1 resource is one
# of these.
DSP_CAI_HARDWARE_MODEM_ASYNC = 0x11
DSP_CAI_HARDWARE_MODEM_SYNC = 0x12

# IDI parameter codes (kernel/pc.h).
IDI_BC = 0x04     # bearer capability
IDI_CAI = 0x10    # call identity: the B1/DSP configuration
IDI_LLI = 0x19    # logical link id
IDI_DLC = 0x20    # data link layer configuration
IDI_UID = 0x2d    # user id
IDI_LLC = 0x7c    # low layer compatibility

# DLC modem protocol negotiation flags (kernel/mdm_msg.h).
DLC_MODEMPROT_DISABLE_V42_V42BIS = 0x01
DLC_MODEMPROT_DISABLE_MNP_MNP5 = 0x02
DLC_MODEMPROT_REQUIRE_PROTOCOL = 0x04
DLC_MODEMPROT_DISABLE_V42_DETECT = 0x08
DLC_MODEMPROT_DISABLE_COMPRESSION = 0x10
DLC_MODEMPROT_DISABLE_SDLC = 0x40

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
ADSP.adsp2181_pc.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pc.restype = ctypes.c_uint16
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
ADSP.adsp2181_watch_exec.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int]
ADSP.adsp2181_sport0_tx_written.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_sport0_tx_written.restype = ctypes.c_int
ADSP.adsp2181_pmovlay.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pmovlay.restype = ctypes.c_uint16
ADSP.adsp2181_dmovlay.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_dmovlay.restype = ctypes.c_uint16
ADSP.adsp2181_read_pm.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_read_pm.restype = ctypes.c_uint32
ADSP.adsp2181_trace_budget.argtypes = [ctypes.c_void_p, ctypes.c_int64]
ADSP.adsp2181_coverage_clear.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_coverage_count.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_coverage_count.restype = ctypes.c_uint64
ADSP.adsp2181_set_callbacks.argtypes = [ctypes.c_void_p] * 4
ADSP.adsp2181_sport0_tdm_frame.argtypes = [
    ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint16,
    ctypes.c_uint16, ctypes.c_int]
ADSP.adsp2181_sport0_tdm_frame.restype = ctypes.c_uint16
ADSP.adsp2181_modem_sample.argtypes = [
    ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16, ctypes.c_int,
    ctypes.c_uint16, ctypes.c_uint16]
ADSP.adsp2181_modem_sample.restype = ctypes.c_uint16

RX_CB = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p, ctypes.c_int)
TX_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int32)
TIM_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)

SAMPLE_RATE = 8000
DM_COUPLED_BUFFER_MODE = 0x32F0
DM_RX_BUFFER_POINTER = 0x3F0F
DM_TX_BUFFER_POINTER = 0x3FB4
DM_RX_BUFFER = 0x2B00
DM_TX_BUFFER = 0x2B01
DM_TDM_OUTPUT_LATCH = 0x2E52

# V.22FC's common 8 kHz line-side TX adapter (overlay 0x0271).  The overlay
# publishes DM_PAGE_TX_SAMPLE through DM_TX_BUFFER_POINTER.  PM 0x1d06 fills
# the 20-word circular queue and PM 0x1d46 consumes one sample per frame.
DM_PAGE_TX_COUNT = 0x3761
DM_PAGE_TX_SAMPLE = 0x3764
DM_PAGE_TX_WRITE_POINTER = 0x3765
DM_PAGE_TX_READ_POINTER = 0x3768
DM_PAGE_TX_RING = 0x36E0
DM_PAGE_TX_RING_WORDS = 0x14


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


def load_sparse_pm_words(cpu, image: Path) -> None:
    """Apply only the PM blocks actually present in an extracted image."""
    import json
    metadata = json.loads((image / "metadata.json").read_text())
    data = (image / "pm.bin").read_bytes()
    pm = ADSP.adsp2181_pm(cpu)
    for block in metadata["pm_blocks"]:
        start = block["address"]
        for address in range(start, start + block["words"]):
            off = address * 3
            pm[address] = (data[off] | (data[off + 1] << 8) |
                           (data[off + 2] << 16))


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
        self.host_reads: list[tuple[int, int, int]] = []
        self.trace_host_reads = False
        self.preserve_host_writes = False
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
        self.call_trace: list[tuple[str, int, int]] = []
        self.trace_calls = False
        self.phase = "boot"
        self.service_assign_pending = False
        self.intercept_bulk_writes = False
        self.bulk_write_calls: list[tuple[int, int, int]] = []
        self.service_assign_block: int | None = None
        self.native_task_started: set[int] = set()
        self.native_bearer_activation = False
        self.native_kernel: Path | None = None
        self.native_tikrnl: Path | None = None
        self.native_service_assign_return: int | None = None
        self.native_setup_frames = 0
        self.native_connected_driver = False

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

    def _start_native_selected_task(self, block: int, core) -> None:
        block &= 0x1fffffff
        if (not self.native_bearer_activation or
                block != self.service_assign_block or
                block in self.native_task_started):
            return
        # SERVICE_ASSIGN has now finished the firmware's genuine segmented
        # download. It already contains the resident kernel plus the relocated
        # TIKRNL task (source PM 06fc is runtime PM 0703). Do not overlay the
        # extracted source-address image here: that destroys those relocations.
        ADSP.adsp2181_set_idma_boot_hold(core, 0)
        ADSP.adsp2181_call(core, 0x0679, 0x02A8)
        ADSP.adsp2181_run(core, 2_000_000)
        if not ADSP.adsp2181_idle(core):
            raise RuntimeError(
                f"native selected-task initializer stopped at "
                f"PM 0x{ADSP.adsp2181_pc(core):04x}")
        self.native_task_started.add(block)

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
        if self.service_assign_pending and self.service_assign_block is None:
            self.service_assign_block = block & 0x1fffffff
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
        if (self.native_bearer_activation and
                self.native_service_assign_return == address and
                self.service_assign_block is not None):
            block = self.service_assign_block
            self._start_native_selected_task(block, self.core_for(block))
            self.native_service_assign_return = None
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
                if (self.native_bearer_activation and
                        self.native_connected_driver and
                        self.native_setup_frames < 4 and
                        self.service_assign_block in self.native_task_started and
                        core is self.cores.get(self.service_assign_block)):
                    # PRI SPORT never stops while the MIPS handles call setup.
                    # The selected task consumes its connected command on that
                    # clock; IRQE alone is masked after TIKRNL initialization.
                    pm = ADSP.adsp2181_pm(core)
                    saved_isr = pm[0x00B5]
                    pm[0x00B5] = 0x1C000F | (0x0586 << 4)
                    ADSP.adsp2181_modem_sample(
                        core, 0x00FF, 0x00FF, 3000, 0x02A9, 0x02A8)
                    if ADSP.adsp2181_idle(core):
                        ADSP.adsp2181_call(core, 0x06C8, 0x02A8)
                        ADSP.adsp2181_run(core, 3000)
                    pm[0x00B5] = saved_isr
                    self.native_setup_frames += 1
                else:
                    ADSP.adsp2181_set_irq(core, 6, 1)
                    ADSP.adsp2181_run(core, 2000)
                    ADSP.adsp2181_set_irq(core, 6, 0)
                    ADSP.adsp2181_run(core, 1000)
        from unicorn.mips_const import (UC_MIPS_REG_PC, UC_MIPS_REG_RA,
                                        UC_MIPS_REG_A0, UC_MIPS_REG_A1,
                                        UC_MIPS_REG_A2, UC_MIPS_REG_A3,
                                        UC_MIPS_REG_V0,
                                        UC_MIPS_REG_0)
        if self.trace_calls:
            try:
                insn = struct.unpack("<I", uc.mem_read(address & 0x1fffffff, 4))[0]
            except Exception:
                insn = 0
            opcode = (insn >> 26) & 0x3F
            target = None
            if opcode == 0x03:  # jal
                target = ((address + 4) & 0xF0000000) | ((insn & 0x03FFFFFF) << 2)
            elif opcode == 0x00 and (insn & 0x3F) == 0x09:  # jalr
                rs = (insn >> 21) & 0x1F
                target = uc.reg_read(UC_MIPS_REG_0 + rs)
            if target is not None:
                self.call_trace.append((self.phase, address, target))
        if address == 0x800951D4 and self.native_bearer_activation:
            # The connected driver is publishing its control toggle now. PRI
            # clocks before this call cannot service that not-yet-live command.
            self.native_connected_driver = True
            self.native_setup_frames = 0
        if address == SERVICE_ASSIGN:
            self.service_assign_pending = True
            if self.native_bearer_activation:
                # The service object points at the task object at +0; the
                # task owns its DSP host-register base at +0x10. Capture the
                # caller now, then initialize only after SERVICE_ASSIGN has
                # completed its sparse download and before the next DSP pump.
                service = uc.reg_read(UC_MIPS_REG_A0)
                task = struct.unpack(
                    "<I", bytes(uc.mem_read(service & 0x1fffffff, 4)))[0]
                block = struct.unpack(
                    "<I", bytes(uc.mem_read((task + 0x10) & 0x1fffffff, 4)))[0]
                self.service_assign_block = block & 0x1fffffff
                self.native_service_assign_return = uc.reg_read(UC_MIPS_REG_RA)
        if (self.intercept_bulk_writes
                and address in (HOST_WRITE_DM_BLOCK, HOST_WRITE_PM_BLOCK)):
            a0 = uc.reg_read(UC_MIPS_REG_A0)
            dest = uc.reg_read(UC_MIPS_REG_A1) & 0x3FFF
            source = uc.reg_read(UC_MIPS_REG_A2)
            count = uc.reg_read(UC_MIPS_REG_A3) & 0xFFFF
            core = self.core_for(a0)
            self.bulk_write_calls.append((address, dest, count))
            raw = bytes(uc.mem_read(source & 0x1FFFFFFF,
                                    count * (2 if address == HOST_WRITE_DM_BLOCK else 4)))
            if address == HOST_WRITE_DM_BLOCK:
                dm = ADSP.adsp2181_dm(core)
                for index in range(count):
                    value = struct.unpack_from("<H", raw, index * 2)[0]
                    dm[(dest + index) & 0x3FFF] = value
                    self.host_writes.append((0x4000 | ((dest + index) & 0x3FFF),
                                             value))
            else:
                pm = ADSP.adsp2181_pm(core)
                for index in range(count):
                    high, low = struct.unpack_from("<HH", raw, index * 4)
                    value = ((high << 8) | (low & 0xFF)) & 0xFFFFFF
                    pm[(dest + index) & 0x3FFF] = value
                    self.host_writes.append(((dest + index) & 0x3FFF, value))
            uc.reg_write(UC_MIPS_REG_V0, 1)
            uc.reg_write(UC_MIPS_REG_PC, uc.reg_read(UC_MIPS_REG_RA))
        elif address == HOST_WRITE:
            a0 = uc.reg_read(UC_MIPS_REG_A0)
            if self.service_assign_pending and self.service_assign_block is None:
                self.service_assign_block = a0 & 0x1fffffff
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
            if self.trace_host_reads:
                self.host_reads.append((a0 & 0x1FFFFFFF, a1, value))
            if self.log:
                print(f"[mips] host_read [0x{a0:08x}] {a1:04x} -> {value:04x}")
            uc.reg_write(UC_MIPS_REG_V0, value)
            uc.reg_write(UC_MIPS_REG_PC, uc.reg_read(UC_MIPS_REG_RA))
        elif address == STUB_VIRT:
            self.stub_returns += 1
            uc.reg_write(UC_MIPS_REG_PC, uc.reg_read(UC_MIPS_REG_RA))

    def call(self, entry: int, args: list[int], gp: int, sp: int,
             max_insns: int = 200000,
             extra_regs: "dict[int, int] | None" = None) -> int:
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
        if extra_regs:
            for reg, value in extra_regs.items():
                uc.reg_write(reg, value)
        if not self.preserve_host_writes:
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


def mips_runtime_addr(addr: int) -> int:
    """Normalize Unicorn's physical MIPS PCs/targets to firmware kseg0 addrs."""
    phys = addr & 0x1fffffff
    if PHYS_BIAS <= phys < PHYS_BIAS + IMAGE_SIZE:
        return phys | 0x80000000
    return addr


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
    # DM 0x3327..0x3337 is 17 words.  The switch-on record occupies exactly
    # 16 words; declaring a 16-word ring wraps the producer to the consumer
    # and makes the full ring indistinguishable from empty.
    shim.write16(ring_v + 0x14, 0x0011)               # ring length (17)

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
    adsp_dm = ADSP.adsp2181_dm(shim.cpu)
    print("[assign] TIKRNL command ring DM3327..3336: "
          + " ".join(f"{adsp_dm[a]:04x}"
                     for a in range(0x3327, 0x3337)))
    print("[assign] TIKRNL control DM3310..3316: "
          + " ".join(f"{adsp_dm[a]:04x}" for a in range(0x3310, 0x3317)))
    print("[assign] host writes: "
          + " ".join(f"{addr:04x}={val:04x}"
                     for addr, val in shim.host_writes))
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
    interesting_downloads = {
        0x0007,  # DIVA Server PRI 2M TX Kernel
        0x0008,  # DIVA Server PRI 2M RX Kernel
        0x000B,  # DIVA Server PRI 2M TX SIG Kernel
        0x000C,  # DIVA Server PRI 2M RX SIG Kernel
        0x0208,  # SIG.MDM Task
        0x0209,  # SIGPRTX Task
        0x020A,  # SIGPRRX Task
        0x0258,  # TIKRNL81.F34 Task
        0x025F,  # V.8 overlay
        0x0261,  # V.34 overlay
        0x026A,  # V.90 DPCM overlay
        0x0270,  # SIG overlay loaded before negative TIKRNL pages
    }
    for entry in image.downloads:
        if entry.download_id in interesting_downloads:
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


def idi_parameters(*params: "tuple[int, bytes]") -> bytes:
    """Encode an IDI request payload the way add_ie() (message.c) does.

    Each parameter is a {code, length, data} triple and the list ends with a
    single zero code byte — add_ie() writes a terminating 0 after every
    parameter and backs over it when the next one is appended.
    """
    out = bytearray()
    for code, data in params:
        out += bytes((code, len(data)))
        out += data
    out.append(0)
    return bytes(out)


def modem_cai(max_bit_rate: int = 56000,
              b1_resource: int = DSP_CAI_HARDWARE_MODEM_ASYNC,
              b1_options: int = 0) -> bytes:
    """The 26-byte CAI add_b1() builds for a modem B1 protocol.

    Offsets follow the driver's cai[] array, whose [0] is the length byte
    add_p() strips off, so data[i] here is the driver's cai[i+1].
    """
    cai = bytearray(26)
    cai[0] = b1_resource & 0xFF          # cai[1]: B1 resource, low
    cai[1] = (b1_resource >> 8) & 0xFF   # cai[2]: B1 resource, high
    cai[2] = 0                           # cai[3]: async framing (8N1)
    cai[3] = b1_options                  # cai[4]: B1 options
    cai[6] = 0                           # cai[7]: line taking options
    cai[7] = 0                           # cai[8]: modem negotiation options
    struct.pack_into("<H", cai, 12, 0)             # cai[13]: min Tx speed
    struct.pack_into("<H", cai, 14, max_bit_rate)  # cai[15]: max Tx speed
    struct.pack_into("<H", cai, 16, 0)             # cai[17]: min Rx speed
    struct.pack_into("<H", cai, 18, max_bit_rate)  # cai[19]: max Rx speed
    return bytes(cai)


def modem_sig_assign_payload(max_bit_rate: int = 56000) -> bytes:
    """Signalling-entity ASSIGN payload: the CAI, as add_b1() attaches it."""
    return idi_parameters((IDI_CAI, modem_cai(max_bit_rate)),
                          (IDI_UID, b"Capi20"))


def modem_call_res_payload(max_bit_rate: int = 56000) -> bytes:
    """CAPI20 ``connect_res()`` modem payload produced by ``add_b1()``.

    The old i4l IDI compatibility path used a six-byte CAI, but Eicon's
    CAPI20 hardware path attaches the complete 26-byte modem descriptor to
    CALL_RES.  This is the transaction whose private DSP effects the native
    ingress experiment needs to preserve.
    """
    return idi_parameters((IDI_CAI, modem_cai(max_bit_rate)))


def modem_nl_assign_payload(max_data_length: int = 1024,
                            answering: bool = True,
                            signaling_id: int | None = None) -> bytes:
    """Network-layer ASSIGN payload, as add_modem_b23()/send_req() build it.

    The modem configuration is LLI/LLC/DLC. On the first global NL request,
    send_req() prefixes a one-byte CAI containing the parent signalling ID.
    This is the plain B2_TRANSPARENT branch (no error correction/compression
    negotiation block).
    """
    lli = bytes((1,))                            # driver lli[1]
    llc = bytes((9 if answering else 10, 4))     # V42_IN / V42, L3 transparent
    dlc = bytearray(struct.pack("<H", max_data_length))
    dlc += bytes((3,     # Addr A
                  1,     # Addr B
                  7,     # modulo mode
                  7,     # window size
                  0, 0,  # XID length
                  DLC_MODEMPROT_DISABLE_V42_V42BIS
                  | DLC_MODEMPROT_DISABLE_MNP_MNP5
                  | DLC_MODEMPROT_DISABLE_SDLC))
    parameters = []
    if signaling_id is not None:
        # message.c send_req(): the first NL request for a PLCI is global
        # (Id=NL_ID), and is prefixed with CAI[0] = the already assigned
        # signalling entity. This is the call-parent link; omitting it makes
        # the firmware reject the otherwise valid modem ASSIGN with 0xe6.
        parameters.append((IDI_CAI, bytes((signaling_id & 0xFF,))))
    parameters.extend(((IDI_LLI, lli), (IDI_LLC, llc), (IDI_DLC, bytes(dlc))))
    return idi_parameters(*parameters)


def rc_name(rc: int) -> str:
    # isdn_rc() (kernel/di.c) treats any Rc & 0xf0 == ASSIGN_RC as an assign
    # acknowledgement carrying the assigned Id, but only ASSIGN_OK means the
    # assign succeeded.
    if rc == ASSIGN_OK:
        return "ASSIGN_OK"
    if rc == RC_OK:
        return "OK"
    if rc & 0xF0 == ASSIGN_RC:
        return f"assign rejected (0x{rc:02x})"
    return "?"


def clear_host_doorbell(shim: "MipsShim") -> None:
    """Acknowledge the card->host notification, as a host ISR would.

    The main loop only calls the RC/IND flush (0x80029774) when RcOutput and
    IndOutput are both zero *and* the byte pointed at by gp+0x5eaf is zero
    (0x80027d84).  The flush sets that byte to 1 on its way out
    (0x8002989c), so leaving it set stops the firmware publishing any further
    return codes: the RC sits queued in card RAM with gp+0x5e9d stuck at 1.
    """
    ptr = struct.unpack_from("<I",
        shim.uc.mem_read((GP + 0x5eaf) & 0x1fffffff, 4))[0]
    if ptr:
        shim.write8(ptr, 0)


def drain_return_codes(shim: "MipsShim", sr: int) -> "list[tuple[int, int, int, int]]":
    """Consume the queued return codes the way pr_rc() (kernel/di.c) does.

    Walk `RcOutput` entries from `B[NextRc]` along the chain, zero each Rc
    field as it is taken, then clear `RcOutput`.  Returns
    (Rc, RcId, RcCh, Reference) tuples.
    """
    rc_out = shim.uc.mem_read(sr + PR_RcOutput, 1)[0]
    if not rc_out:
        return []
    clear_host_doorbell(shim)
    off = struct.unpack_from("<H", shim.uc.mem_read(sr + PR_NextRc, 2))[0]
    out = []
    for _ in range(rc_out):
        rb = sr + PR_B + off
        rc = shim.uc.mem_read(rb + RC_RC, 1)[0]
        if rc:
            out.append((rc,
                        shim.uc.mem_read(rb + RC_RCID, 1)[0],
                        shim.uc.mem_read(rb + RC_RCCH, 1)[0],
                        struct.unpack_from("<H",
                            shim.uc.mem_read(rb + RC_REFERENCE, 2))[0]))
            shim.write8(rb + RC_RC, 0)
        off = struct.unpack_from("<H", shim.uc.mem_read(rb, 2))[0]
    shim.write8(sr + PR_RcOutput, 0)
    return out


def drain_indications(shim: "MipsShim", sr: int) -> list[tuple[int, int, int, int, bytes]]:
    """Consume card indications using the PR_RAM IND chain."""
    count = shim.uc.mem_read(sr + PR_IndOutput, 1)[0]
    if not count:
        return []
    clear_host_doorbell(shim)
    off = struct.unpack_from("<H", shim.uc.mem_read(sr + PR_NextInd, 2))[0]
    out = []
    for _ in range(count):
        rb = sr + PR_B + off
        length = struct.unpack_from("<H",
            shim.uc.mem_read(rb + IND_RBUFFER, 2))[0]
        out.append((
            shim.uc.mem_read(rb + IND_IND, 1)[0],
            shim.uc.mem_read(rb + IND_ID, 1)[0],
            shim.uc.mem_read(rb + IND_CH, 1)[0],
            struct.unpack_from("<H",
                shim.uc.mem_read(rb + IND_REFERENCE, 2))[0],
            bytes(shim.uc.mem_read(rb + IND_RDATA, length)),
        ))
        shim.write8(rb + IND_IND, 0)
        off = struct.unpack_from("<H", shim.uc.mem_read(rb, 2))[0]
    shim.write8(sr + PR_IndOutput, 0)
    return out


def post_request(shim: "MipsShim", sr: int, req: int, req_id: int,
                 req_ch: int, payload: bytes, reference: int = 0) -> int:
    """Put one IDI request in the queue, as pr_out() (kernel/di.c) does.

    Fill the REQ at B[NextReq], advance NextReq to REQ->next, and increment
    the host-owned ReqInput counter.  Returns the buffer offset used.
    """
    off = struct.unpack_from("<H", shim.uc.mem_read(sr + PR_NextReq, 2))[0]
    rb = sr + PR_B + off
    req_next = struct.unpack_from("<H", shim.uc.mem_read(rb + REQ_NEXT, 2))[0]
    shim.write8(rb + REQ_REQ, req)
    shim.write8(rb + REQ_REQID, req_id)
    shim.write8(rb + REQ_REQCH, req_ch)
    shim.write16(rb + REQ_REFERENCE, reference)
    shim.write16(rb + REQ_XBUFFER, len(payload))
    shim.uc.mem_write(rb + REQ_XDATA, payload)
    shim.write16(sr + PR_NextReq, req_next)
    req_in = shim.uc.mem_read(sr + PR_ReqInput, 1)[0]
    shim.write8(sr + PR_ReqInput, (req_in + 1) & 0xFF)
    return off


def run_until_rc(shim: "MipsShim", sr: int, gp: int, sp: int,
                 iterations: int = 32, phase: str = "mainloop") -> "list[tuple[int, int, int, int]]":
    """Spin the main loop until the firmware queues a return code."""
    for _ in range(iterations):
        try:
            shim.phase = phase
            shim.call(MIPS_MAINLOOP, [], gp=gp, sp=sp, max_insns=500000)
        except Exception as exc:
            print(f"[mainloop] fault: {exc}")
            break
        if shim.uc.mem_read(sr + PR_RcOutput, 1)[0]:
            break
    return drain_return_codes(shim, sr)


def assign_entity(shim: "MipsShim", sr: int, gp: int, sp: int, label: str,
                  req_id: int, req_ch: int, payload: bytes) -> "int | None":
    """Post one ASSIGN and report its return code.

    Returns the local entity id the card assigned on ASSIGN_OK, else None.
    """
    off = post_request(shim, sr, ASSIGN, req_id, req_ch, payload,
                       reference=1 if label == "nl" else 0)
    print(f"[{label}] ASSIGN Id=0x{req_id:02x} Ch=0x{req_ch:02x} "
          f"@B[0x{off:04x}] payload={payload.hex()}")
    codes = run_until_rc(shim, sr, gp, sp, phase=f"{label}-assign")
    if not codes:
        print(f"[{label}] no return code")
        return None
    assigned = None
    for rc, rc_id, rc_ch, ref in codes:
        print(f"[{label}] RC 0x{rc:02x} ({rc_name(rc)}) Id=0x{rc_id:02x} "
              f"Ch=0x{rc_ch:02x} Ref=0x{ref:04x}")
        if rc == ASSIGN_OK and assigned is None:
            assigned = rc_id
    return assigned


def issue_listen_request(shim: "MipsShim", sr: int, gp: int, sp: int,
                         sig_id: int, legacy_req_id: bool = False) -> None:
    """Put the assigned signalling entity into incoming-call listening state.

    The old i4l driver names this host operation INDICATE_REQ even though the
    firmware-side CAPI state machine talks about LISTEN_REQ.  Its payload is a
    one-byte zero parameter block (idi_put_req()), and it must happen before a
    CALL_IND can exist for CALL_RES to answer.
    """
    req_id = 1 if legacy_req_id else sig_id
    off = post_request(shim, sr, INDICATE_REQ, req_id, 0, b"\x00", reference=0)
    print(f"[listen] INDICATE_REQ/LISTEN Id=0x{req_id:02x} "
          f"(sig=0x{sig_id:02x}) Ch=0x00 @B[0x{off:04x}]")
    for rc, rc_id, rc_ch, ref in run_until_rc(shim, sr, gp, sp,
                                              phase="listen-req"):
        print(f"[listen] RC 0x{rc:02x} ({rc_name(rc)}) Id=0x{rc_id:02x} "
              f"Ch=0x{rc_ch:02x} Ref=0x{ref:04x}")
    for ind, ind_id, ind_ch, ref, payload in drain_indications(shim, sr):
        print(f"[listen] IND 0x{ind:02x} Id=0x{ind_id:02x} "
              f"Ch=0x{ind_ch:02x} Ref=0x{ref:04x} payload={payload.hex()}")


def read_runtime32(shim: "MipsShim", addr: int) -> int:
    return struct.unpack_from("<I",
        shim.uc.mem_read(addr & 0x1fffffff, 4))[0]


def dump_entities(shim: "MipsShim", gp: int, limit: int = 16) -> None:
    count = struct.unpack_from("<H",
        shim.uc.mem_read((gp + 0x5eb9) & 0x1fffffff, 2))[0]
    print(f"[entities] count={count} table=0x{ENTITY_TABLE:08x}")
    for idx in range(min(count, limit)):
        ptr = read_runtime32(shim, ENTITY_TABLE + idx * 4)
        if ptr == 0:
            continue
        words = []
        for off in range(0, 0x30, 4):
            words.append(read_runtime32(shim, ptr + off))
        formatted = " ".join(f"+{i * 4:02x}={word:08x}"
                             for i, word in enumerate(words))
        print(f"[entities] {idx:02x}: ptr=0x{ptr:08x} {formatted}")
        call = read_runtime32(shim, ptr + 0x1c)
        if call:
            sig_fields = bytes(shim.uc.mem_read((ptr + 0x340) & 0x1fffffff, 0x1f0))
            call_fields = bytes(shim.uc.mem_read(call & 0x1fffffff, 0x240))
            print(f"[entities] {idx:02x}: sig+340..52f={sig_fields.hex()}")
            print(f"[entities] {idx:02x}: call[0..23f]={call_fields.hex()}")


def inject_call_ingress(shim: "MipsShim", gp: int, sp: int,
                        slot: int = 0) -> None:
    """Inject a network-originated SETUP into the real signalling parser.

    ``0x800172c0`` obtains the current message type from ``gp+0x5e87`` and,
    when ``gp+0x5e88`` is zero, parses the length/data block selected by
    ``gp+0x5ecf``.  This is the same interface the lower PRI/SIG dispatcher
    establishes before calling the controller object's handler.  Event
    ``0x17`` is the no-call-state jump-table entry that allocates the incoming
    call object; event 2 only updates bearer-status flags.
    """
    sig_obj = read_runtime32(shim, ENTITY_TABLE + slot * 4)
    if sig_obj == 0:
        print(f"[ingress] no signalling object in slot {slot}")
        return

    # IDI/Q.931 information elements consumed by the parser: 3.1-kHz audio
    # bearer capability, V.42 low-layer compatibility, and a minimal channel
    # identification.  The parser uses the ordinary code,length,data form.
    payload = idi_parameters(
        (IDI_BC, bytes((0x90, 0x90, 0xa3))),
        (IDI_LLC, bytes((0x88, 0x90, 0x21))),
        (0x18, bytes((0xa1, 0x83))),
    )
    message = SYNTH_INGRESS_MESSAGE + slot * 0x100
    shim.alloc(message, 0x100)
    for off in range(0, 0x100, 4):
        shim.write32(message + off, 0)
    shim.write16(message + 0x10, len(payload))
    shim.write_bytes(message + 0x12, payload)

    entity_id = shim.uc.mem_read((sig_obj + 0x14) & 0x1fffffff, 1)[0]
    shim.write32(gp + 0x5ecf, message)
    shim.write8(gp + 0x5e88, 0x00)  # use the gp+0x5ecf message block
    shim.write8(gp + 0x5eab, entity_id)

    # The PRI dispatcher reports a new call first (0x17), causing state 0 to
    # allocate the call object, then delivers SETUP indication 0x0b to the new
    # call-state handler.  Calling only the first event leaves a correctly
    # allocated but never indicated call.
    for event, label in ((0x17, "allocate"), (0x0b, "SETUP")):
        shim.write8(gp + 0x5e87, event)
        before = read_runtime32(shim, sig_obj + 0x1c)
        print(f"[ingress] {label} event 0x{event:02x} on controller slot "
              f"{slot} obj=0x{sig_obj:08x} entity=0x{entity_id:02x} "
              f"message=0x{message:08x} payload={payload.hex()} "
              f"before +1c=0x{before:08x}")
        try:
            shim.phase = f"call-ingress-{label.lower()}"
            shim.call(CALL_INGRESS_PARSER, [sig_obj], gp=gp, sp=sp,
                      max_insns=2000000)
        except Exception as exc:
            print(f"[ingress] firmware {label} parser stopped: {exc}")
        after = read_runtime32(shim, sig_obj + 0x1c)
        state = (shim.uc.mem_read((after + 0x2c) & 0x1fffffff, 1)[0]
                 if after else 0)
        print(f"[ingress] controller slot {slot} after {label}: "
              f"+1c=0x{after:08x} call_state=0x{state:02x}")
        if event == 0x17 and after:
            # The lower D-channel dispatcher advances the newly allocated
            # controller from allocation state 1 to pending-incoming state 2
            # before delivering the decoded SETUP.  0x8002a89c treats state 2
            # as an already network-owned call; leaving state 1 makes it try
            # to allocate an outgoing B-channel and reject the SETUP.
            shim.write16(sig_obj + 0x24, 2)
            shim.write16(sig_obj + 0x26, 1)
            flags = read_runtime32(shim, sig_obj + 0x20)
            shim.write32(sig_obj + 0x20, flags | 0x00400000)


def synthesize_call_ingress(shim: "MipsShim", slot: int = 0) -> None:
    """Fabricate the minimum incoming-call object needed before CALL_RES.

    This mirrors the field writes in the 0x800172a8 allocation branch without
    depending on the surrounding Q.931 dispatcher frame: the listening SIG
    entity gains a call object at +0x1c, enters pending-call state, and the
    call object points back at its owning SIG entity.
    """
    sig_obj = read_runtime32(shim, ENTITY_TABLE + slot * 4)
    if sig_obj == 0:
        print(f"[ingress] no entity object in slot {slot}")
        return
    call_obj = SYNTH_CALL_OBJECT + slot * 0x100
    shim.alloc(call_obj, 0x100)
    for off in range(0, 0x100, 4):
        shim.write32(call_obj + off, 0)
    flags = read_runtime32(shim, sig_obj + 0x20) & 0xfffeffff
    shim.write8(call_obj + 0x2f, 1)
    shim.write32(call_obj + 0x28, sig_obj)
    # The allocation branch stores state 1 first; the real SETUP parser then
    # progresses the PLCI into the pending incoming-call state before CALL_RES.
    shim.write16(sig_obj + 0x24, 2)
    shim.write8(sig_obj + 0x12a, 1)
    shim.write32(sig_obj + 0x1c, call_obj)
    shim.write32(sig_obj + 0x20, flags)
    # Parsed SETUP fields used by the incoming answer path.  These are
    # length-prefixed internal copies of the Q.931 BC/LLC/HLC/channel IEs.
    # BC 90 90 a3 = 3.1 kHz audio, 64 kbit/s, G.711 A-law; LLC 88 90 21
    # selects V.42/modem-style low-layer handling in the IDI firmware.
    shim.write_bytes(sig_obj + 0x365, bytes((4, 0x90, 0x90, 0xa3, 0x00)))
    shim.write_bytes(sig_obj + 0x37d, bytes((4, 0x88, 0x90, 0x21, 0x00)))
    shim.write_bytes(sig_obj + 0x395, bytes((1, 0x80)))
    shim.write8(sig_obj + 0x51f, 0xff)
    shim.write8(sig_obj + 0x520, 0x11)
    print(f"[ingress] synthetic call object 0x{call_obj:08x} "
          f"linked to entity slot {slot} obj=0x{sig_obj:08x}")


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
    # The DSPs have to run in line with the MIPS: the validator writes 0x5a5a
    # to a DSP, downloads its kernel, releases it, then polls for 0xa5a5
    # within one call, and without that acknowledgement no DSP resources are
    # registered.  IDMA boot hold keeps each core stopped for its own
    # download, so interleaving is safe.
    shim.pump_every = args.dsp_pump

    if args.dsp_combifile is not None:
        stage_dsp_code(shim, args)

    # 1. Write the card config and boot command the firmware entry reads
    #    during init.  The request queue is NOT set up here: the firmware
    #    initialises PR_RAM itself as it boots and publishes its signature
    #    when ready, so a request written beforehand is overwritten.
    sr = PR_RAM_PHYS
    shim.write8(sr + 0x08, 0)      # TEI (0 = auto)
    shim.write8(sr + 0x10, args.force_law)  # 1=A-law, 2=mu-law
    shim.write8(sr + 0x16, 0x80)    # DSPInfo = DSP code loaded
    # The protocol image and staged combifile must agree on card identity.
    # Hardcoding legacy value 12 while selecting the PRI-30M file set (23)
    # lets basic signalling run but bypasses the matching DSP resource path.
    shim.write8(sr + 0x1a, args.card_type)
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
        shim.phase = "entry"
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
        shim.phase = "post-init1"
        shim.call(MIPS_POST_INIT1, [], gp=gp, sp=sp, max_insns=2000000)
    except Exception as exc:
        print(f"[mainloop] init1 fault: {exc}")
    try:
        shim.phase = "post-init2"
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

    # 3b. Assign the entities now that the firmware has initialised PR_RAM.
    #     The driver's order is signalling first (sig_req(plci, ASSIGN,
    #     DSIG_ID), carrying the CAI from add_b1()), then the network layer
    #     (nl_req_ncci(plci, ASSIGN, 0), carrying LLI/LLC/DLC from
    #     add_modem_b23()).  Each ASSIGN is answered with an ASSIGN_RC
    #     carrying the local entity id the card allocated.
    sig = struct.unpack_from("<H", shim.uc.mem_read(sr + PR_Signature, 2))[0]
    print(f"[mainloop] card ready: Sig=0x{sig:04x}")
    # From this point onward report the whole call lifecycle. MipsShim.call()
    # normally resets this diagnostic per helper invocation, which made a
    # connected call incorrectly finish with "host_writes=0" whenever its
    # final main-loop iteration happened to be idle.
    shim.host_writes = []
    shim.preserve_host_writes = True

    assigned = {}
    if args.entity in ("sig", "both"):
        steps = (("sig", DSIG_ID, modem_sig_assign_payload()),)
    else:
        steps = ()
    for label, req_id, payload in steps:
        entity_id = assign_entity(shim, sr, gp, sp, label, req_id,
                                  args.channel, payload)
        if entity_id is None:
            print(f"[{label}] assign did not succeed; stopping the sequence")
            break
        assigned[label] = entity_id
        print(f"[{label}] entity id 0x{entity_id:02x} assigned "
              f"(host_writes={len(shim.host_writes)})")

    defer_nl_assign = (
        args.fake_call_ingress
        and args.call_direction == "answering"
        and args.entity in ("nl", "both")
    )

    if args.entity in ("nl", "both") and not defer_nl_assign:
        signaling_id = assigned.get("sig")
        payload = modem_nl_assign_payload(
            answering=args.call_direction == "answering",
            signaling_id=signaling_id)
        entity_id = assign_entity(shim, sr, gp, sp, "nl", NL_ID,
                                  args.channel, payload)
        if entity_id is None:
            print("[nl] assign did not succeed; stopping the sequence")
        else:
            assigned["nl"] = entity_id
            print(f"[nl] entity id 0x{entity_id:02x} assigned "
                  f"(host_writes={len(shim.host_writes)})")

    call_channel = 0
    if args.fake_call_ingress and "sig" in assigned:
        issue_listen_request(shim, sr, gp, sp, assigned["sig"],
                             legacy_req_id=args.legacy_sig_req_id)
        if args.inject_call_ingress:
            inject_call_ingress(shim, gp, sp, args.ingress_entity_slot)
            # The real SETUP parser emits CALL_IND through PR_RAM.  Its Ch is
            # the per-call selector that must be echoed by CALL_RES; Ch=0
            # answers the listener and bypasses the allocated call object.
            for ind, ind_id, ind_ch, ref, payload in drain_indications(shim, sr):
                print(f"[ingress] IND 0x{ind:02x} Id=0x{ind_id:02x} "
                      f"Ch=0x{ind_ch:02x} Ref=0x{ref:04x} "
                      f"payload={payload.hex()}")
                if ind == 0x02:
                    call_channel = ind_ch
        if args.synthesize_call_ingress:
            synthesize_call_ingress(shim, args.ingress_entity_slot)
        if args.dump_entities:
            dump_entities(shim, gp, args.dump_entity_limit)

    if defer_nl_assign:
        signaling_id = assigned.get("sig")
        payload = modem_nl_assign_payload(
            answering=True,
            signaling_id=signaling_id)
        entity_id = assign_entity(shim, sr, gp, sp, "nl", NL_ID,
                                  args.channel, payload)
        if entity_id is None:
            print("[nl] assign did not succeed after fake ingress")
        else:
            assigned["nl"] = entity_id
            print(f"[nl] entity id 0x{entity_id:02x} assigned after ingress "
                  f"(host_writes={len(shim.host_writes)})")
            if args.dump_entities:
                dump_entities(shim, gp, args.dump_entity_limit)

    if defer_nl_assign:
        # NL ASSIGN gives the asynchronous lower SETUP path enough main-loop
        # turns to publish CALL_IND.  Consume it before CALL_RES, matching the
        # real host ordering and releasing PR_RAM indication flow control.
        for ind, ind_id, ind_ch, ref, payload in drain_indications(shim, sr):
            print(f"[ingress] IND 0x{ind:02x} Id=0x{ind_id:02x} "
                  f"Ch=0x{ind_ch:02x} Ref=0x{ref:04x} "
                  f"payload={payload.hex()}")
            if ind == 0x02:
                call_channel = ind_ch

    if args.connect and "nl" in assigned:
        bearer_disconnected = False
        if args.call_direction == "answering" and "sig" in assigned:
            # message.c connect_res(): add_b1() appends the modem CAI to the
            # CALL_RES itself. The initial SIG ASSIGN only creates the PLCI;
            # an empty CALL_RES answers signalling but never allocates the
            # 0x0258 modem DSP service.
            call_payload = modem_call_res_payload()
            off = post_request(shim, sr, CALL_RES, assigned["sig"],
                               call_channel, call_payload, reference=0)
            print(f"[call] CALL_RES Id=0x{assigned['sig']:02x} "
                  f"Ch=0x{call_channel:02x} @B[0x{off:04x}]")
            for rc, rc_id, rc_ch, ref in run_until_rc(shim, sr, gp, sp,
                                                      phase="call-res"):
                print(f"[call] RC 0x{rc:02x} ({rc_name(rc)}) "
                      f"Id=0x{rc_id:02x} Ch=0x{rc_ch:02x} Ref=0x{ref:04x}")
            if (args.inject_call_ingress and
                    getattr(args, "native_bearer_activation", False)):
                # SETUP allocation is followed by a distinct lower-PRI
                # connected event after an answering CALL_RES. It publishes
                # CONNECT_ACTIVE and installs the selected bearer state.
                sig_obj = read_runtime32(shim, ENTITY_TABLE +
                                          args.ingress_entity_slot * 4)
                shim.write8(gp + 0x5e87, 0x03)
                shim.phase = "call-ingress-connected"
                shim.call(CALL_INGRESS_PARSER, [sig_obj], gp=gp, sp=sp,
                          max_insns=2_000_000)
                print("[ingress] delivered post-CALL_RES event 0x03")
            if args.dump_entities:
                dump_entities(shim, gp, args.dump_entity_limit)
        off = post_request(shim, sr, N_CONNECT, assigned["nl"], 0,
                           b"\x00", reference=1)
        print(f"[call] N_CONNECT Id=0x{assigned['nl']:02x} Ch=0x00 "
              f"@B[0x{off:04x}]")
        for rc, rc_id, rc_ch, ref in run_until_rc(shim, sr, gp, sp,
                                                  phase="n-connect"):
            print(f"[call] RC 0x{rc:02x} ({rc_name(rc)}) "
                  f"Id=0x{rc_id:02x} Ch=0x{rc_ch:02x} Ref=0x{ref:04x}")
        if args.dump_entities:
            dump_entities(shim, gp, args.dump_entity_limit)
        if args.force_modem_dsp_assign:
            force_modem_dsp_assign(shim, args)
        for _ in range(args.call_steps):
            shim.phase = "call-pump"
            shim.call(MIPS_MAINLOOP, [], gp=gp, sp=sp, max_insns=500000)
            for ind, ind_id, ind_ch, ref, payload in drain_indications(shim, sr):
                print(f"[call] IND 0x{ind:02x} Id=0x{ind_id:02x} "
                      f"Ch=0x{ind_ch:02x} Ref=0x{ref:04x} "
                      f"payload={payload.hex()}")
                if ind == 0x04:
                    bearer_disconnected = True
        dsp_assigned = f"{SERVICE_ASSIGN:08x}" in shim.trace_log
        if bearer_disconnected:
            bearer_state = "DISCONNECTED"
        elif dsp_assigned:
            bearer_state = "ACTIVE (modem DSP assigned)"
        else:
            bearer_state = "SIGNALLING ACTIVE, DSP UNASSIGNED"
        print(f"[call] simulated B-channel: {bearer_state}")

    if assigned:
        print("[mainloop] assigned: " +
              ", ".join(f"{k}=0x{v:02x}" for k, v in assigned.items()))

    print(f"[mainloop] done: host_writes={len(shim.host_writes)}")
    print("[mainloop] modem DSP path: service_assign=%d switch_on=%d"
          % (shim.trace_log.count(f"{SERVICE_ASSIGN:08x}"),
             shim.trace_log.count("80090e58")))
    if shim.service_assign_block is not None:
        block = shim.service_assign_block
        core = shim.cores.get(block)
        if core is not None:
            dm = ADSP.adsp2181_dm(core)
            print(f"[mainloop] native modem core block=0x{block:08x} "
                  f"ring=DM{dm[0x3316]:04x}..DM{dm[0x3315]:04x} "
                  f"page=0x{dm[0x3fb0]:04x} "
                  f"mode=0x{dm[0x3f94]:04x}")
            if args.native_dm_out is not None:
                args.native_dm_out.parent.mkdir(parents=True, exist_ok=True)
                args.native_dm_out.write_bytes(struct.pack(
                    "<16384H", *(dm[i] for i in range(0x4000))))
                print(f"[mainloop] native modem DM snapshot: "
                      f"{args.native_dm_out}")
    if args.trace_calls:
        from collections import Counter, defaultdict
        phases: dict[str, Counter[int]] = defaultdict(Counter)
        for phase, _src, target in shim.call_trace:
            target = mips_runtime_addr(target)
            if BIAS <= target < BIAS + len(args.image.read_bytes()):
                phases[phase][target] += 1
        print("[trace] firmware call targets by phase:")
        marked = {SERVICE_ASSIGN, BIAS + 0x7FE58}
        for index, (phase, src, target) in enumerate(shim.call_trace):
            runtime_target = mips_runtime_addr(target)
            if runtime_target not in marked:
                continue
            lo = max(0, index - 8)
            hi = min(len(shim.call_trace), index + 9)
            print(f"[trace] ordered window around 0x{runtime_target:08x} "
                  f"in {phase}:")
            for item_phase, item_src, item_target in shim.call_trace[lo:hi]:
                print(f"    [{item_phase}] 0x{mips_runtime_addr(item_src):08x} "
                      f"-> 0x{mips_runtime_addr(item_target):08x}")
        printed = False
        for phase in sorted(phases):
            top = phases[phase].most_common(args.trace_call_limit)
            if not top:
                continue
            printed = True
            print(f"  [{phase}]")
            for target, count in top:
                mark = ""
                if target == SERVICE_ASSIGN:
                    mark = " SERVICE_ASSIGN"
                elif target == BIAS + 0x7fe58:
                    mark = " SWITCH_ON"
                print(f"    0x{target:08x} count={count}{mark}")
        if not printed:
            raw = Counter((phase, src, target) for phase, src, target in shim.call_trace)
            print(f"  no file-backed targets decoded; raw_calls={sum(raw.values())}")
            for (phase, src, target), count in raw.most_common(args.trace_call_limit):
                print(f"    [{phase}] src=0x{src:08x} target=0x{target:08x} count={count}")
    if shim.log and shim.host_writes:
        for addr, val in shim.host_writes[:32]:
            tag = "DM" if addr & 0x4000 else "PM"
            print(f"  host_write {tag} 0x{addr & 0x7fff:04x} = 0x{val:04x}")


class NativeMipsModem:
    """SIP-facing view of the modem core assigned by the real MIPS firmware.

    The MIPS remains live as the host supervisor.  RTP's 8 kHz clock drives
    the selected ADSP core one PRI frame at a time, while one MIPS main-loop
    pass per RTP packet handles database commands and overlay downloads.
    """

    def __init__(self, shim: MipsShim, core, law: str, dsp_block: int,
                 download_descriptors: dict[int, int],
                 force_info_after_v8: bool = False,
                 tx_prbs: bool = False,
                 prime_v90d_bulk_cursor: bool = False,
                 native_bearer_activation: bool = False,
                 mips_interval: int = 160, adsp_budget: int = 20000):
        self.shim = shim
        self.cpu = core
        self.dm = ADSP.adsp2181_dm(core)
        self.law = law
        self.dsp_block = dsp_block
        self.download_descriptors = download_descriptors
        self.force_info_after_v8 = force_info_after_v8
        self._media_samples = 0
        self.silence = 0xD5 if law == "pcma" else 0xFF
        self.mips_interval = max(1, mips_interval)
        self.adsp_budget = adsp_budget
        self.switches: list[tuple[int, int, int]] = []
        self.overlays: dict[int, tuple[object, str]] = {}
        self.forced_info_samples: list[int] = []
        self.l1l2_forced_samples: list[int] = []
        self.resident = 0x0258
        self._mips_fault_reported = False
        self._private_line_active = False
        self.tx_prbs = tx_prbs
        self.prime_v90d_bulk_cursor = prime_v90d_bulk_cursor
        self._v90d_bulk_cursor_primed = False
        self._v90d_saved_clear = None
        self._direct_selected_dispatch = False
        self.native_bearer_activation = native_bearer_activation
        self.tx_requests = 0
        self.tx_accepted = 0
        self.tx_first_sample: int | None = None
        self._tx_pending = False
        self._tx_lfsr = 0x6D2B79F5

    def _sport_rx_word(self, code: int) -> int:
        """Expand a DS0 octet as the T1/E1 SPORT compander does."""
        return sport_rx_word(code, self.law)

    def start_native_task(self) -> None:
        """Release the assigned core and run TIKRNL's relocated initializer."""
        if self.dsp_block in self.shim.native_task_started:
            print("[native-mips] TIKRNL initialized before native SWITCH_ON")
            return
        if ADSP.adsp2181_idma_boot_held(self.cpu):
            ADSP.adsp2181_set_idma_boot_hold(self.cpu, 0)
        ADSP.adsp2181_call(self.cpu, 0x0679, 0x02A8)
        ADSP.adsp2181_run(self.cpu, 2_000_000)
        if not ADSP.adsp2181_idle(self.cpu):
            raise RuntimeError(
                f"native TIKRNL initializer stopped at PM "
                f"0x{ADSP.adsp2181_pc(self.cpu):04x}")
        self.shim.native_task_started.add(self.dsp_block)
        print("[native-mips] released assigned DSP and initialized TIKRNL")

    def load_native_overlay(self, download_id: int) -> None:
        """Run the firmware's real segmented/relocating ADSP loader."""
        descriptor = self.download_descriptors.get(download_id)
        if descriptor is None:
            raise RuntimeError(f"download 0x{download_id:04x} is not staged")
        # 0x80086af8 consumes this 0x1c-byte transfer state.  Its segment-base
        # pointer is biased by eight bytes: relocation segment N is read at
        # table + N*2 - 8.  Native TIKRNL allocated modem DM segment 4 at
        # 0x32f0 and movable PM export segment 5 at 0x0580.
        state = RAM_VIRT + 0xA000
        bases = RAM_VIRT + 0xA100
        self.shim.alloc(state, 0x200)
        self.shim.write_bytes(state, bytes(0x40))
        self.shim.write_bytes(bases, bytes(0x40))
        self.shim.write32(state + 0x00, self.dsp_block)
        self.shim.write32(state + 0x08, descriptor)
        self.shim.write32(state + 0x0C, bases + 8)
        dm_blocks = struct.unpack(
            "<I", self.shim.uc.mem_read((descriptor + 0x28) & 0x1FFFFFFF, 4))[0]
        self.shim.write32(state + 0x14, dm_blocks)
        self.shim.write16(bases + 4 * 2, 0x32F0)
        self.shim.write16(bases + 5 * 2, 0x0580)
        before = len(self.shim.host_writes)
        self.shim.intercept_bulk_writes = True
        try:
            result = self.shim.call(DSP_DOWNLOAD, [state, 0xFFFF, 0],
                                    gp=GP, sp=STACK_TOP, max_insns=8_000_000)
        finally:
            self.shim.intercept_bulk_writes = False
        active = self.shim.uc.mem_read((state + 0x10) & 0x1FFFFFFF, 1)[0]
        block_index = struct.unpack(
            "<H", self.shim.uc.mem_read((state + 0x12) & 0x1FFFFFFF, 2))[0]
        if result != 1 or not active:
            raise RuntimeError(
                f"native loader did not complete 0x{download_id:04x}: "
                f"result={result} active={active} block={block_index} "
                f"bulk={self.shim.bulk_write_calls[-4:]}")
        # Eicon's PRI kernel publishes the relocated selected-channel state
        # at DM2f86. Generic modem overlays dereference the compatibility word
        # DM32f6 instead; portable overlay DM blocks reset it to zero. Bridge
        # the private descriptor after every download before page init runs.
        if self.dm[0x2F86]:
            self.dm[0x32F6] = self.dm[0x2F86]
        self.resident = download_id
        print(f"[native-mips] loaded 0x{download_id:04x} through MIPS "
              f"({len(self.shim.host_writes) - before} host writes)")

    def attach_connected_bearer(self) -> None:
        # This is the exact result of SIG.MDM's private bearer-connected
        # notification; use the card loader rather than copying extracted
        # fixed-address word maps.
        for download_id in (0x026D, 0x025C, 0x0262):
            self.load_native_overlay(download_id)
        self.dm[0x2F22] = 0x3C27 if self.law == "pcmu" else 0x3C07
        self.dm[0x32F0] = 0x0004
        self.dm[0x3F0F] = 0x2B00
        self.dm[0x3FB4] = 0x2B01
        # ADDSP V.90 guide §5.4.1 Table 12, followed in a distinct host
        # communication cycle by answer-mode Tables 13 and 15.
        initial = {
            0x00: 0x00C4, 0x01: 0x0040, 0x02: 0x0000, 0x03: 0x0000,
            0x07: 0xF0FD, 0x08: 0x0006, 0x09: 0x0006, 0x0A: 0x00FF,
            0x0B: 0x0030, 0x0C: 0x0000, 0x24: 0x000C,
            0x2C: 0x0003, 0x2D: 0x0003,
        }
        for offset, value in initial.items():
            self.dm[0x3EE0 + offset] = value
        self.dm[0x3EEE] = 0x2000
        initial_frames = 0
        for initial_frames in range(1, 4097):
            self._frame_core(self.silence)
            if not (self.dm[0x3EEE] & 0x2000):
                break
        if self.dm[0x3EEE] & 0x2000:
            raise RuntimeError(
                "native TIKRNL did not consume initial WDB: "
                f"3131={self.dm[0x3131]:04x} 3137={self.dm[0x3137]:04x} "
                f"3138={self.dm[0x3138]:04x} 3141={self.dm[0x3141]:04x}")
        final = {
            0x01: 0x0484, 0x02: 0x0030, 0x04: 0x6000,
            0x0F: 0x0001, 0x10: 0x0100, 0x28: 0x0001,
            0x29: 0x8100, 0x2A: 0x001F, 0x2B: 0xFF00,
            0x79: 0x003F, 0x7A: 0xFFFF, 0x7B: 0x03B7,
            0x7C: 0x000E, 0x7D: 0x0015, 0x7E: 0x000E, 0x7F: 0x0015,
        }
        for offset, value in final.items():
            self.dm[0x3EE0 + offset] = value
        self.dm[0x3EEE] = 0x2000
        if self.resident != 0x0262:
            self.load_native_overlay(0x0262)
        for entry, budget in ((0x0581, 200000), (0x13CC, 1000000)):
            ADSP.adsp2181_call(self.cpu, entry, 0x02A8)
            ADSP.adsp2181_run(self.cpu, budget)
            if not ADSP.adsp2181_idle(self.cpu):
                raise RuntimeError(f"native DIAL setup PM {entry:04x} did not return")
        # PM 0581 imports the native CAI defaults, including NORM_H=0x00ff.
        # The documented answer-mode WDB is the following communication cycle;
        # publish it after that import so V.8 sees NORM_H=1 (negotiate).
        for offset, value in final.items():
            self.dm[0x3EE0 + offset] = value
        self.dm[0x3EEE] = 0x2000
        answer_frames = 0
        for answer_frames in range(1, 4097):
            self._frame_core(self.silence)
            if not (self.dm[0x3EEE] & 0x2000):
                break
        if self.dm[0x3EEE] & 0x2000:
            raise RuntimeError("native TIKRNL did not consume answer WDB")
        print("[native-mips] connected bearer activated through DIAL "
              f"(WDB frames {initial_frames}+{answer_frames})")

    def complete_native_answer(self) -> None:
        """Finish ADDSP answer setup after native task attachment.

        Event 0x03 remains the sole TIKRNL attachment owner. The existing
        compatibility routine is used only for its documented DIAL pages and
        two WDB communication cycles; the exact one-call selected-channel
        media adapter is restored before media starts.
        """
        native = self.native_bearer_activation
        self.native_bearer_activation = False
        try:
            self.attach_connected_bearer()
        finally:
            self.native_bearer_activation = native
        # Task attachment and retained DM are native. Media uses the existing
        # one-call selected descriptor adapter because page downloads replace
        # the kernel's private dispatch records; it invokes relocated PM06c8
        # exactly once per line sample.
        self._direct_selected_dispatch = True

    def _next_tx_words(self) -> tuple[int, int, int]:
        """Generate 48 deterministic bits for one V.90D datagram request."""
        words = []
        for _ in range(3):
            value = 0
            for bit in range(16):
                # x^32 + x^22 + x^2 + x + 1, non-zero deterministic seed.
                lsb = self._tx_lfsr & 1
                self._tx_lfsr = ((self._tx_lfsr >> 1) ^
                                 (0x80200003 if lsb else 0)) & 0xFFFFFFFF
                value |= lsb << bit
            words.append(value)
        return words[0], words[1], words[2]

    def _service_tx_request(self) -> None:
        """Supply the polling data interface described by ADDSP guide §5.3.1.

        In V90D, TXD0 bit 0 is oldest and a datagram spans TXD0..TXD2. The
        negotiated packet uses only 21..42 of these bits. The DSP owns and
        clears DI_control bit F after consuming the packet.
        """
        if (not self.tx_prbs or self.resident != 0x026A or self._tx_pending or
                not (self.dm[0x3FAD] & 0x8000)):
            return
        words = self._next_tx_words()
        self.dm[0x3F05], self.dm[0x3F06], self.dm[0x3F07] = words
        self.tx_requests += 1
        self._tx_pending = True
        if self.tx_first_sample is None:
            self.tx_first_sample = self._media_samples
            print(f"[native-mips] supplied first V90D TX datagram at sample "
                  f"{self._media_samples}: "
                  f"{words[0]:04x}/{words[1]:04x}/{words[2]:04x}")

    def _frame_core(self, code: int) -> None:
        # A request raised by the preceding sample is answered before the DSP
        # receives the next SPORT clock, matching an IDMA host polling cycle.
        self._service_tx_request()
        self._media_samples += 1
        sport_word = code & 0xFF
        # The hardware PRI descriptor calls TIKRNL's registered continuation
        # only for this selected channel.  The generic SPORT frame walks the
        # kernel queue but cannot reconstruct that private callback.
        # DIAL activation still consumes this reconstructed line word. Once
        # V.8 is resident, however, DM3f08 is processed status owned by the
        # page; raw G.711 reaches it through the selected SPORT descriptor.
        if not self._private_line_active:
            self.dm[0x3F08] = code & 0xFF
        else:
            # The private descriptor supplies a processed line-status word
            # separately from the raw G.711 ring. Fixed dispatch holds 0x21
            # here during normal V.8 media; storing the octet itself corrupts
            # the result bits, while leaving it at 1 stalls the RX action.
            self.dm[0x3F08] = 0x0021
            # ADDSP V.90 User's Guide §3.3 specifies SPORT companding for the
            # T1/E1 interface. The private descriptor publishes the expanded
            # signed sample, not the compressed DS0 octet, to the page RX word.
            sport_word = self._sport_rx_word(code)
            self.dm[0x3763] = sport_word
        # Native TIKRNL registers PM 0x0586 as the selected-channel ISR and
        # PM 0x0703 as its continuation. Model the private descriptor without
        # permanently replacing either global kernel dispatch slot.
        pm = ADSP.adsp2181_pm(self.cpu)
        saved_isr = pm[0x00B5]
        pm[0x00B5] = 0x1C000F | (0x0586 << 4)
        try:
            # The returned SPORT0 latch is deliberately discarded.  It is not a
            # transmit source: it carries the kernel's TDM slot mirror, i.e. the
            # received word delayed one frame.  Measured on run13 after the
            # bootpage 14 handoff, TX[t+1] == RX[t] for 16000/16000 samples, so
            # publishing it would echo the peer to itself.  The modem's own
            # transmit sample reaches the line only through DM(0x3fb4).
            # MIPS has already consumed the private command mailbox. Run the
            # relocated no-host continuation (source 06c1+7) if the selected
            # channel ISR yields, in the same C call to avoid per-sample FFI.
            if (self.native_bearer_activation and
                    not self._direct_selected_dispatch):
                # Keep the resident kernel foreground at PM 0x02a9 live. It
                # observes DM2f08 != DM2f09 and calls PM 0x01c1 to install the
                # selected task vectors. The compatibility path skips that
                # owner and resumes TIKRNL directly at PM 0x06c8.
                ADSP.adsp2181_modem_sample(
                    self.cpu, sport_word, self.silence, self.adsp_budget,
                    0x02A9, 0x02A8)
                if ADSP.adsp2181_idle(self.cpu):
                    # The tail of this continuation zeroes the six-word V.90
                    # mapping-frame block DM(0x3fa7..0x3fac) at PM
                    # 0x06ca..0x06cd (6 writes every frame, reached through the
                    # 0x04f8 call and its RTS). The page-14 generator refills
                    # that block once per 1333 Hz mapping frame at PM 0x2a52
                    # (`CALL (I4)`, AX0 = 0x3fa7) while the serializer at PM
                    # 0x2eed..0x2ef2 walks cursor DM(0x20de) across it one slot
                    # per 8 kHz frame, so the block has to survive six frames.
                    # Without the block surviving, five of every six downstream
                    # samples read zero and the line carries an impulse train
                    # instead of Sd. The clear runs inside the ISR above rather
                    # than in this continuation, so it is suppressed at the
                    # store itself when page 14 loads, not snapshotted here.
                    ADSP.adsp2181_call(self.cpu, 0x06C8, 0x02A8)
                    ADSP.adsp2181_run(self.cpu, self.adsp_budget)
            else:
                ADSP.adsp2181_modem_sample(
                    self.cpu, sport_word, self.silence, self.adsp_budget,
                    0x06C8, 0x02A8)
        finally:
            pm[0x00B5] = saved_isr
        wanted = self.dm[0x3132] & 0xFFFF
        if (self.force_info_after_v8 and self.resident == 0x025F
                and wanted != 0x0260 and self.dm[0x3FB0] not in (6, 7)):
            if self._media_samples < 12000:
                return
            wanted = 0x0260
            self.dm[0x3FB0] = 7
            self.dm[0x3132] = wanted
            self.forced_info_samples.append(self._media_samples)
            print(f"[native-mips] diagnostic post-V.8 fallback -> INFO "
                  f"at sample {self._media_samples}")
        page_ready = bool(self.dm[0x3FC1] & 0x0100)
        if (self.native_bearer_activation and self.dm[0x3137]
                and wanted != self.resident):
            page_ready = True
        if (page_ready and self.dm[0x3131]
                and wanted in self.download_descriptors):
            previous = self.resident
            if wanted != self.resident:
                self.load_native_overlay(wanted)
                if wanted == 0x026A and V90D_HOLD_TX_BLOCK:
                    # PM 0x06cd is the six-count store that zeroes the V.90
                    # mapping-frame block DM(0x3fa7..0x3fac) every frame in the
                    # resident kernel's frame tail. The page-14 generator
                    # refills the block once per 1333 Hz mapping frame while
                    # the serializer walks it one slot per 8 kHz frame, so the
                    # clear has to stop for the block to survive its six reads.
                    pm_words = ADSP.adsp2181_pm(self.cpu)
                    self._v90d_saved_clear = pm_words[0x06CD]
                    pm_words[0x06CD] = 0x000000
                    print("[native-mips] diagnostic: suppressed per-frame "
                          "clear of the V90D mapping-frame block")
                elif (self._v90d_saved_clear is not None
                        and self.resident == 0x026A):
                    ADSP.adsp2181_pm(self.cpu)[0x06CD] = self._v90d_saved_clear
                    self._v90d_saved_clear = None
                if wanted == 0x026A and V90D_BULK_ADAPTER_DISABLED:
                    # Diagnostic: RTS out the tail of the 0x1900..0x19c8
                    # near/far echo bulk-delay adapter. With the adapter live
                    # the outer state machine stalls before 0x0080 (session
                    # 65's delayed bulk-cursor collision); with it disabled the
                    # machine reaches 0x0080 and transmits. Set
                    # EICON_V90D_BULK_ADAPTER=1 to keep the adapter running.
                    ADSP.adsp2181_pm(self.cpu)[0x19C8] = 0x0A000F
                    print("[native-mips] diagnostic: disabled V90D bulk adapter")
                self.switches.append(
                    (self._media_samples, self.dm[0x3FB0], wanted))
            if wanted == 0x025F:
                self._private_line_active = True
                # PM 2025 snapshots DIAL's processed line status before the
                # first callback. Raw PCMU idle 0xff has result bits 5-6 set;
                # hardware/direct dispatch presents idle status 1 here.
                self.dm[0x3F08] = 0x0001
            self.dm[0x3EEE] = 0x1000
            resume = self.dm[0x3143] & 0x3FFF
            if resume:
                ADSP.adsp2181_call(self.cpu, resume, 0x02A8)
                ADSP.adsp2181_run(self.cpu, self.adsp_budget)
            if wanted == 0x025F:
                # The movable V.8 init leaves its temporary DM image in the
                # runtime TX word and zeroes the two disabled-timer sentinels.
                # Fixed-layout dispatch has completed this shared-state seam
                # with -1 timers and an empty adapter output before frame 1.
                self.dm[0x3995] = 0xFFFF
                self.dm[0x3999] = 0xFFFF
                self.dm[0x3764] = 0x0000
            self.dm[0x3EEE] &= ~0x1000
            print(f"[native-mips] page request 0x{wanted:04x} "
                  f"(from 0x{previous:04x}) resumed at PM 0x{resume:04x}")
        # Diagnostic seam: PM 0x1982 preserves the far-bulk cursor in DM4,
        # but the portable V90D image initializes it to zero. A real selected
        # channel is expected to publish the first valid delay-line address.
        # Prime it once at activation to distinguish that missing publication
        # from a DSP state-machine or codec failure.
        if (self.prime_v90d_bulk_cursor and not self._v90d_bulk_cursor_primed
                and self.resident == 0x026A and self.dm[0x1FF7] == 0x0060):
            self.dm[4] = self.dm[0]
            self._v90d_bulk_cursor_primed = True
            print(f"[native-mips] diagnostic V90D bulk cursor DM4 "
                  f"primed to DM0=0x{self.dm[0]:04x}")
        if self._tx_pending and not (self.dm[0x3FAD] & 0x8000):
            self.tx_accepted += 1
            self._tx_pending = False
        # V.8 FFT work can span more than one execution budget. Preserve a
        # live page context and continue it on the next exact SPORT frame.

    def boot(self) -> None:
        """Compatibility with ``Card``/``LiveKernelModem``; already booted."""

    def configure_modem(self, role: str, law: str = "pcmu") -> None:
        if role != "answer":
            raise ValueError("native MIPS SIP backend currently answers calls only")
        if law != self.law:
            raise ValueError(f"native core booted for {self.law}, not {law}")

    def _step_mips(self) -> None:
        try:
            self.shim.phase = "native-sip"
            self.shim.call(MIPS_MAINLOOP, [], gp=GP, sp=STACK_TOP,
                           max_insns=500000)
            # Act as the host consumer so a long call cannot fill PR_RAM with
            # status indications.  Data-plane delivery will be attached to
            # the NL entity separately; signalling diagnostics are printed.
            for ind, ind_id, ind_ch, ref, payload in drain_indications(
                    self.shim, PR_RAM_PHYS):
                if ind not in (N_CONNECT, 3):
                    print(f"[native-mips] IND 0x{ind:02x} "
                          f"Id=0x{ind_id:02x} Ch=0x{ind_ch:02x} "
                          f"Ref=0x{ref:04x} payload={payload.hex()}")
            drain_return_codes(self.shim, PR_RAM_PHYS)
        except Exception as exc:
            if not self._mips_fault_reported:
                print(f"[native-mips] runtime supervisor stopped: {exc}")
                self._mips_fault_reported = True

    def frame_fast(self, code: int, sample_index: int) -> int:
        self._frame_core(code)
        if (sample_index + 1) % self.mips_interval == 0:
            self._step_mips()
        if self.resident == 0x026A:
            if V90D_HOLD_TX_BLOCK and self.dm[0x20DE] == 0x3FAD:
                # The mapping-frame block is held across the resident kernel's
                # per-frame clear so the serializer can walk all six slots, but
                # it must not outlive its cycle: zero it once the cursor has
                # read the sixth slot (post-frame DM(0x20de) = 0x3fad, its wrap
                # value). The generator refills at the start of the next mapping
                # frame; if it has stopped, the next cycle reads silence instead
                # of re-emitting the last block for ever. Suppressing the clear
                # without this froze run35's second call on codeword 148
                # (linear -13948, near full scale) for its last 7 seconds, from
                # the instant the state machine reached 0x00b3.
                for address in range(0x3FA7, 0x3FAD):
                    self.dm[address] = 0
            # Page 14 publishes the sample itself in DM(0x3fb4): PM 0x19ee
            # re-primes the generic pointer 0x3764 every frame and PM 0x1a1e
            # then overwrites it with the word the V90D serializer left in
            # DM(0x3fa7). Nothing writes DM(0x3764) at all while V90D
            # transmits, so there is nothing for the generic indirection to
            # dereference here; applying it turns each sample into whatever
            # unrelated word lives at that address.
            value = self.dm[0x3FB4]
        else:
            pointer = self.dm[0x3FB4] & 0x3FFF
            value = self.dm[pointer] if pointer else 0
        return value - 0x10000 if value & 0x8000 else value


def create_native_mips_modem(kernel: Path, tikrnl: Path, law: str = "pcmu",
                             image: Path = Path("docs/firmware/te_dmlt.pm"),
                             dsp_combifile: Path = Path("docs/firmware/dspdload.bin"),
                             channel: int = 1, call_steps: int = 2,
                             dsp_pump: int = 256,
                             force_info_after_v8: bool = False,
                             tx_prbs: bool = False,
                             prime_v90d_bulk_cursor: bool = False,
                             native_bearer_activation: bool = False) -> NativeMipsModem:
    """Boot the real card firmware and return its naturally assigned modem."""
    if law not in ("pcmu", "pcma"):
        raise ValueError("native MIPS backend supports only pcmu or pcma")
    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    ADSP.adsp2181_set_idma_boot_hold(cpu, 1)
    shim = MipsShim(image, cpu)
    shim.native_bearer_activation = native_bearer_activation
    shim.native_kernel = kernel
    shim.native_tikrnl = tikrnl
    shim.write32(0x800fbe30, STUB_VIRT)
    base = protocol_end_addr(image)
    staged = build_dsp_code_image(
        dsp_combifile, CARDTYPE_DIVASRV_P_30M_PCI, base)
    descriptors = {entry.download_id: base + 4 + index * 0x30
                   for index, entry in enumerate(staged.downloads)}
    args = SimpleNamespace(
        image=image, tikrnl=tikrnl, dsp_combifile=dsp_combifile,
        dsp_code_base=None, card_type=CARDTYPE_DIVASRV_P_30M_PCI,
        force_law=2 if law == "pcmu" else 1,
        dsp_pump=dsp_pump, entity="both", channel=channel,
        call_direction="answering", fake_call_ingress=True,
        inject_call_ingress=True, synthesize_call_ingress=False,
        ingress_entity_slot=0, legacy_sig_req_id=False,
        connect=True, force_modem_dsp_assign=False, call_steps=call_steps,
        dump_entities=False, dump_entity_limit=0, native_dm_out=None,
        trace_calls=False, trace_call_limit=0,
        native_bearer_activation=native_bearer_activation)
    run_mainloop(shim, args)
    block = shim.service_assign_block
    core = shim.cores.get(block) if block is not None else None
    if core is None:
        raise RuntimeError("native incoming call did not assign a modem DSP core")
    print(f"[native-mips] SIP media attached to DSP block 0x{block:08x} "
          f"using {law}")
    modem = NativeMipsModem(
        shim, core, law, block, descriptors,
        force_info_after_v8=force_info_after_v8, tx_prbs=tx_prbs,
        prime_v90d_bulk_cursor=prime_v90d_bulk_cursor,
        native_bearer_activation=native_bearer_activation)
    if not native_bearer_activation:
        modem.start_native_task()
    if native_bearer_activation:
        # SERVICE_ASSIGN already installed the resident kernel and relocated
        # task. Reapplying extracted source-address PM here would undo that
        # relocation after the connected-task command has run.
        ADSP.adsp2181_set_idma_boot_hold(core, 0)
        modem.complete_native_answer()
        print("[native-mips] using native lower-PRI task attachment with "
              "ADDSP answer WDB completion")
    else:
        modem.attach_connected_bearer()
    return modem


def stage_direct_tikrnl_core(args):
    """Create a standalone PRI-kernel+TIKRNL core for forced call assignment.

    The PR_RAM mainloop emulates every card DSP as a separate core while the
    MIPS firmware boots the card.  The direct service-assign helper, however,
    talks through the synthetic host register block at RAM+0x5000, which is
    wired to shim.cpu.  Use a deliberately staged TIKRNL core there so the
    switch-on command ring lands in a modem task that can consume it.
    """
    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    load_pm_words(cpu, args.kernel / "pm.bin")
    load_dm_words(cpu, args.kernel / "dm.bin")
    ADSP.adsp2181_run(cpu, 1000)
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for line in (args.tikrnl / "pm.words").read_text().splitlines():
        a, v = line.split()
        pm[int(a, 16)] = int(v, 16)
    for line in (args.tikrnl / "dm.words").read_text().splitlines():
        a, v = line.split()
        dm[int(a, 16)] = int(v, 16)
    ADSP.adsp2181_call(cpu, 0x0672, 0x02A8)
    ADSP.adsp2181_run(cpu, 1000000)
    return cpu


def load_adsp_module(cpu, module: Path) -> None:
    """Layer one extracted ADSP download directory onto an existing core."""
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for line in (module / "pm.words").read_text().splitlines():
        a, v = line.split()
        pm[int(a, 16)] = int(v, 16)
    for line in (module / "dm.words").read_text().splitlines():
        a, v = line.split()
        dm[int(a, 16)] = int(v, 16)


def find_extracted_download(download_id: int) -> Path | None:
    roots = (
        Path("artifacts/eicon-dsp/overlays"),
        Path("artifacts/eicon-dsp/sig-path"),
        Path("artifacts/eicon-dsp/build-117-926/tikrnl"),
    )
    for root in roots:
        if not root.is_dir():
            continue
        for entry in root.iterdir():
            meta = entry / "metadata.json"
            if not meta.is_file():
                continue
            try:
                if json.loads(meta.read_text()).get("download_id") == download_id:
                    return entry
            except (OSError, json.JSONDecodeError):
                continue
    return None


def pump_direct_tikrnl_core(cpu, words: int) -> None:
    """Let TIKRNL consume the switch-on database ring written by run_assign."""
    for _ in range(words):
        dm = ADSP.adsp2181_dm(cpu)
        if dm[0x3315] != dm[0x3316]:
            ADSP.adsp2181_host_write(cpu, 0x7310, 0x0001)
        ADSP.adsp2181_set_irq(cpu, 3, 1)  # SPORT0_RX
        ADSP.adsp2181_set_irq(cpu, 3, 0)
        ADSP.adsp2181_set_irq(cpu, 6, 1)  # IRQE doorbell
        ADSP.adsp2181_run(cpu, 5000)
        ADSP.adsp2181_set_irq(cpu, 6, 0)
        ADSP.adsp2181_run(cpu, 5000)
        ADSP.adsp2181_call(cpu, 0x064A, 0x02A8)
        ADSP.adsp2181_run(cpu, 20000)
        service_vector = ADSP.adsp2181_dm(cpu)[0x3308]
        if service_vector:
            ADSP.adsp2181_call(cpu, service_vector, 0x02A8)
            ADSP.adsp2181_run(cpu, 20000)
        ADSP.adsp2181_call(cpu, 0x06BB, 0x02A8)
        ADSP.adsp2181_run(cpu, 20000)


def linear_to_mulaw(sample: int) -> int:
    sample = max(-32768, min(32767, sample))
    sign = 0x80 if sample < 0 else 0
    if sample < 0:
        sample = -sample - 1
    sample += 0x84
    if sample > 0x7FFF:
        sample = 0x7FFF
    segment = 0
    shifted = sample >> 5
    while shifted and segment < 8:
        shifted >>= 1
        segment += 1
    if segment >= 8:
        return (sign | 0x7F) ^ 0xFF
    return (sign | (segment << 4) | ((sample >> (segment + 3)) & 0xF)) ^ 0xFF


def make_g711_stimulus(kind: str, samples: int, code: int,
                       freq: float = 2100.0, amp: int = 20000) -> list[int]:
    """Build an 8 kHz u-law G.711 stimulus for the forced DSP RX path."""
    if samples <= 0:
        return []
    if kind == "constant":
        return [code & 0xFF] * samples
    if kind == "silence" or not freq:
        return [0xFF] * samples
    result = []
    phase_offset = 0.0
    reversal_samples = int(0.450 * SAMPLE_RATE)
    for index in range(samples):
        if kind == "ansam" and index and index % reversal_samples == 0:
            phase_offset += math.pi
        envelope = (1.0 + 0.2 * math.sin(2 * math.pi * 15 * index / SAMPLE_RATE)
                    if kind == "ansam" else 1.0)
        linear = int(amp * envelope
                     * math.sin(2 * math.pi * freq * index / SAMPLE_RATE
                                + phase_offset))
        result.append(linear_to_mulaw(linear))
    return result


def restore_direct_pcm_pointers(cpu) -> None:
    """Restore the one-line pointer-mode PCM buffers used by TIKRNL pages."""
    dm = ADSP.adsp2181_dm(cpu)
    dm[DM_COUPLED_BUFFER_MODE] = 0x0004
    dm[DM_RX_BUFFER_POINTER] = DM_RX_BUFFER
    dm[DM_TX_BUFFER_POINTER] = DM_TX_BUFFER


def probe_direct_tikrnl_g711(cpu, samples: int, code: int,
                             stimulus_kind: str = "constant",
                             stimulus_freq: float = 2100.0,
                             stimulus_amp: int = 20000,
                             bridge_tx: bool = False,
                             restore_pcm_pointers: bool = False) -> None:
    """Feed raw G.711 codewords into the assigned direct TIKRNL core.

    It writes u-law octets to the data-pump line words that DIAL/V.8 consume
    (`DM 0x3f08`/`0x3f09`) and runs TIKRNL's frame entry.  With bridge_tx set,
    it also copies the task pointer-mode TX buffer to the kernel TDM output
    latch (`DM 0x2e52`) before strobing SPORT0_RX so TX0 emits it.
    """
    dm = ADSP.adsp2181_dm(cpu)
    stimulus = make_g711_stimulus(stimulus_kind, samples, code,
                                  stimulus_freq, stimulus_amp)
    tx_words: list[int] = []
    natural_tx_words: list[int] = []
    bridge_tx_words: list[int] = []
    bridged_words: list[int] = []
    rx_index = 0

    def rx_cb(_cpu, port):
        if port != 0:
            return 0
        if not stimulus:
            return code & 0xFF
        return stimulus[min(rx_index, len(stimulus) - 1)] & 0xFF

    def tx_cb(_cpu, port, value):
        if port == 0:
            tx_words.append(value & 0xFFFF)

    def timer_cb(_cpu, _enabled):
        return None

    callbacks = (RX_CB(rx_cb), TX_CB(tx_cb), TIM_CB(timer_cb))
    ADSP.adsp2181_set_callbacks(cpu, *callbacks)
    changes = 0
    prev = None
    page_tx_seen = False
    page_tx_counts: list[int] = []
    page_tx_samples: list[int] = []
    page_tx_ring_nonzero_max = 0
    for index in range(samples):
        rx_index = index
        rx_code = stimulus[index] if stimulus else (code & 0xFF)
        dm[0x3F08] = rx_code & 0xFF
        dm[0x3F09] = rx_code & 0xFF
        # Exercise both paths: the direct frame entry makes page progress
        # visible, while SPORT0 RX/TX strobes give the kernel bridge a chance
        # to move line samples to and from the serial port callbacks.
        tx_mark = len(tx_words)
        ADSP.adsp2181_set_irq(cpu, 3, 1)  # SPORT0_RX
        ADSP.adsp2181_set_irq(cpu, 3, 0)
        ADSP.adsp2181_run(cpu, 50000)
        natural_tx_words.extend(tx_words[tx_mark:])
        ADSP.adsp2181_call(cpu, 0x06BB, 0x02A8)
        ADSP.adsp2181_run(cpu, 50000)
        ADSP.adsp2181_set_irq(cpu, 4, 1)  # SPORT0_TX
        ADSP.adsp2181_set_irq(cpu, 4, 0)
        ADSP.adsp2181_run(cpu, 50000)
        wanted = dm[0x31AA]
        if dm[0x31A9] and wanted:
            module = find_extracted_download(wanted)
            if module is not None:
                load_adsp_module(cpu, module)
                if restore_pcm_pointers:
                    restore_direct_pcm_pointers(cpu)
                dm[0x3EEE] = 0x1000  # BOOTFINISHED; mirrors host overlay ack
                resume = dm[0x31BB]
                if resume:
                    ADSP.adsp2181_call(cpu, resume, 0x02A8)
                    ADSP.adsp2181_run(cpu, 100000)
                print(f"[g711] served requested overlay 0x{wanted:04x} "
                      f"from {module.name}")
            else:
                print(f"[g711] requested overlay 0x{wanted:04x}, "
                      "but no extracted image is available")
        if bridge_tx:
            if restore_pcm_pointers:
                restore_direct_pcm_pointers(cpu)
            tx_ptr = dm[DM_TX_BUFFER_POINTER] & 0x3FFF
            tx_value = dm[tx_ptr] if tx_ptr else dm[0x3F09]
            bridged_words.append(tx_value & 0xFFFF)
            dm[DM_TDM_OUTPUT_LATCH] = tx_value & 0xFFFF
            tx_mark = len(tx_words)
            ADSP.adsp2181_set_irq(cpu, 3, 1)  # SPORT0_RX drives TDM TX0
            ADSP.adsp2181_set_irq(cpu, 3, 0)
            ADSP.adsp2181_run(cpu, 20000)
            bridge_tx_words.extend(tx_words[tx_mark:])
        tx_pointer = dm[DM_TX_BUFFER_POINTER] & 0x3FFF
        if tx_pointer == DM_PAGE_TX_SAMPLE:
            page_tx_seen = True
            page_tx_counts.append(dm[DM_PAGE_TX_COUNT])
            page_tx_samples.append(dm[DM_PAGE_TX_SAMPLE])
            page_tx_ring_nonzero_max = max(
                page_tx_ring_nonzero_max,
                sum(dm[DM_PAGE_TX_RING + n] != 0
                    for n in range(DM_PAGE_TX_RING_WORDS)))
        now = (dm[0x3F08], dm[0x3F09], dm[0x3FB0], dm[0x3FB2],
               dm[0x3FB3], dm[0x3FC1], dm[0x31A9], dm[0x31AA],
               dm[DM_TX_BUFFER_POINTER], dm[tx_pointer] if tx_pointer else 0)
        if now != prev:
            changes += 1
            if changes <= 12:
                print(f"[g711] sample {index:04d}: "
                      f"3F08={now[0]:04x} 3F09={now[1]:04x} "
                      f"3FB0={now[2]:04x} 3FB2={now[3]:04x} "
                      f"3FB3={now[4]:04x} 3FC1={now[5]:04x} "
                      f"31A9={now[6]:04x} 31AA={now[7]:04x} "
                      f"3FB4={now[8]:04x} TXPTR={now[9]:04x}")
            prev = now
    if stimulus_kind == "constant":
        source_desc = f"raw G.711 octets 0x{code & 0xff:02x}"
    elif stimulus_kind == "silence":
        source_desc = "u-law silence"
    else:
        source_desc = (f"{stimulus_kind} {stimulus_freq:g}Hz "
                       f"amp={stimulus_amp}")
    print(f"[g711] fed {samples} {source_desc}; line-state changes={changes}")
    if page_tx_seen:
        from collections import Counter
        sample_counts = Counter(page_tx_samples)
        queue_min = min(page_tx_counts) if page_tx_counts else 0
        queue_max = max(page_tx_counts) if page_tx_counts else 0
        print("[g711] V.22FC page TX adapter DM3764: "
              f"frames={len(page_tx_samples)} "
              f"nonzero={sum(value != 0 for value in page_tx_samples)} "
              f"queue-count={queue_min}..{queue_max} "
              f"write=DM{dm[DM_PAGE_TX_WRITE_POINTER] & 0x3fff:04x} "
              f"read=DM{dm[DM_PAGE_TX_READ_POINTER] & 0x3fff:04x} "
              f"ring-nonzero-max={page_tx_ring_nonzero_max}/{DM_PAGE_TX_RING_WORDS} "
              f"top={','.join(f'{value:04x}:{count}' for value, count in sample_counts.most_common(8))}")
    if bridged_words:
        from collections import Counter
        counts = Counter(bridged_words)
        non_idle = sum(value not in (0x0000, 0x00ff, 0x0400)
                       for value in bridged_words)
        print(f"[g711] bridged task TX DM[3FB4]->DM{DM_TDM_OUTPUT_LATCH:04x}: "
              f"words={len(bridged_words)} unique={len(counts)} non_idle={non_idle} "
              f"top={','.join(f'{value:04x}:{count}' for value, count in counts.most_common(8))} "
              f"first16={' '.join(f'{value:04x}' for value in bridged_words[:16])}")
        bridge_counts = Counter(bridge_tx_words)
        bridge_non_idle = sum(value not in (0x0000, 0x00ff, 0x0400)
                              for value in bridge_tx_words)
        print(f"[g711] SPORT0 TX0 bridged captures: words={len(bridge_tx_words)} "
              f"unique={len(bridge_counts)} non_idle={bridge_non_idle} "
              f"top={','.join(f'{value:04x}:{count}' for value, count in bridge_counts.most_common(8))} "
              f"first16={' '.join(f'{value:04x}' for value in bridge_tx_words[:16])}")
    if natural_tx_words:
        from collections import Counter
        counts = Counter(natural_tx_words)
        non_idle = sum(value not in (0x0000, 0x00ff, 0x0400)
                       for value in natural_tx_words)
        print(f"[g711] SPORT0 TX0 natural captures: words={len(natural_tx_words)} "
              f"unique={len(counts)} non_idle={non_idle} "
              f"top={','.join(f'{value:04x}:{count}' for value, count in counts.most_common(8))} "
              f"first16={' '.join(f'{value:04x}' for value in natural_tx_words[:16])}")
    if tx_words:
        from collections import Counter
        counts = Counter(tx_words)
        non_idle = sum(value not in (0x0000, 0x00ff, 0x0400)
                       for value in tx_words)
        print(f"[g711] SPORT0 TX0 captured: words={len(tx_words)} "
              f"unique={len(counts)} non_idle={non_idle} "
              f"top={','.join(f'{value:04x}:{count}' for value, count in counts.most_common(8))} "
              f"first16={' '.join(f'{value:04x}' for value in tx_words[:16])}")
    else:
        print("[g711] SPORT0 TX0 captured: words=0")


def scan_direct_tikrnl_tx_source(cpu, marker: int = 0x0055,
                                 start: int | None = None,
                                 end: int | None = None) -> None:
    """Poke likely TX buffers and see which one appears on SPORT0 TX0."""
    dm = ADSP.adsp2181_dm(cpu)
    hits: list[tuple[str, int, int]] = []
    tx_words: list[int] = []

    def rx_cb(_cpu, _port):
        return 0xFF

    def tx_cb(_cpu, port, value):
        if port == 0:
            tx_words.append(value & 0xFFFF)

    def timer_cb(_cpu, _enabled):
        return None

    callbacks = (RX_CB(rx_cb), TX_CB(tx_cb), TIM_CB(timer_cb))
    ADSP.adsp2181_set_callbacks(cpu, *callbacks)
    if start is not None or end is not None:
        lo = 0 if start is None else start
        hi = 0x4000 if end is None else end
        ordered_candidates = list(range(max(0, lo), min(0x4000, hi)))
    else:
        candidates = (
            list(range(0x2E00, 0x2E60))
            + list(range(0x2B00, 0x2B10))
            + [0x2E52, 0x3F08, 0x3F09, 0x3F0F, 0x3FB4]
        )
        seen = set()
        ordered_candidates = []
        for addr in candidates:
            if addr not in seen:
                seen.add(addr)
                ordered_candidates.append(addr)
    for addr in ordered_candidates:
        saved = dm[addr]
        tx_words.clear()
        dm[addr] = marker & 0xFFFF
        ADSP.adsp2181_set_irq(cpu, 3, 1)  # SPORT0_RX drives the TDM walk
        ADSP.adsp2181_set_irq(cpu, 3, 0)
        ADSP.adsp2181_run(cpu, 20000)
        if tx_words and tx_words[-1] == (marker & 0xFFFF):
            hits.append(("rx", addr, tx_words[-1]))
        tx_words.clear()
        ADSP.adsp2181_set_irq(cpu, 4, 1)  # SPORT0_TX
        ADSP.adsp2181_set_irq(cpu, 4, 0)
        ADSP.adsp2181_run(cpu, 20000)
        dm[addr] = saved
        if tx_words and tx_words[-1] == (marker & 0xFFFF):
            hits.append(("tx", addr, tx_words[-1]))
    print("[txscan] marker 0x%04x source hits: %s" % (
        marker & 0xFFFF,
        " ".join(f"{irq}:DM{addr:04x}->{value:04x}"
                 for irq, addr, value in hits)
        or "none"))


def force_direct_tikrnl_tx(cpu, samples: int, code: int,
                           source: int = 0x2E52) -> None:
    """Preload the kernel TDM output latch and capture forced SPORT0 TX0."""
    dm = ADSP.adsp2181_dm(cpu)
    tx_words: list[int] = []

    def rx_cb(_cpu, port):
        return 0xFF if port == 0 else 0

    def tx_cb(_cpu, port, value):
        if port == 0:
            tx_words.append(value & 0xFFFF)

    def timer_cb(_cpu, _enabled):
        return None

    callbacks = (RX_CB(rx_cb), TX_CB(tx_cb), TIM_CB(timer_cb))
    ADSP.adsp2181_set_callbacks(cpu, *callbacks)
    for _ in range(samples):
        dm[source & 0x3FFF] = code & 0xFFFF
        ADSP.adsp2181_set_irq(cpu, 3, 1)  # SPORT0_RX runs the TDM TX walk
        ADSP.adsp2181_set_irq(cpu, 3, 0)
        ADSP.adsp2181_run(cpu, 20000)
    if tx_words:
        from collections import Counter
        counts = Counter(tx_words)
        forced = sum(value == (code & 0xFFFF) for value in tx_words)
        print(f"[force-tx] source DM{source & 0x3fff:04x}=0x{code & 0xffff:04x}: "
              f"captured={len(tx_words)} forced={forced} "
              f"top={','.join(f'{value:04x}:{count}' for value, count in counts.most_common(8))} "
              f"first16={' '.join(f'{value:04x}' for value in tx_words[:16])}")
    else:
        print(f"[force-tx] source DM{source & 0x3fff:04x}: captured=0")


def force_modem_dsp_assign(shim: "MipsShim", args) -> None:
    """Force the recovered TIKRNL service assignment during fake call setup."""
    print("[force] staging direct TIKRNL core for modem DSP assignment")
    forced_cpu = stage_direct_tikrnl_core(args)
    old_cpu = shim.cpu
    old_multi = shim.multi_dsp
    try:
        shim.cpu = forced_cpu
        shim.multi_dsp = False
        run_assign(shim, args)
        pump_direct_tikrnl_core(forced_cpu, args.words)
        dm = ADSP.adsp2181_dm(forced_cpu)
        print("[force] after TIKRNL pump: DM3310..3316 "
              + " ".join(f"{dm[a]:04x}" for a in range(0x3310, 0x3317))
              + " DM3327..3336 "
              + " ".join(f"{dm[a]:04x}" for a in range(0x3327, 0x3337)))
        if args.g711_probe_samples:
            probe_direct_tikrnl_g711(forced_cpu, args.g711_probe_samples,
                                     args.g711_probe_code,
                                     args.g711_probe_stimulus,
                                     args.g711_probe_freq,
                                     args.g711_probe_amp,
                                     args.bridge_task_tx,
                                     args.restore_pcm_pointers)
        if args.tx_source_scan:
            scan_direct_tikrnl_tx_source(forced_cpu, args.tx_source_marker,
                                         args.tx_scan_start, args.tx_scan_end)
        if args.force_tx_samples:
            force_direct_tikrnl_tx(forced_cpu, args.force_tx_samples,
                                   args.force_tx_code, args.force_tx_source)
    finally:
        shim.cpu = old_cpu
        shim.multi_dsp = old_multi
    shim.forced_modem_cpu = forced_cpu


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
    parser.add_argument("--force-law", type=int, choices=(1, 2), default=1,
                        help="native card companding: 1=A-law (default), "
                             "2=mu-law")
    parser.add_argument("--entity", choices=("sig", "nl", "both"),
                        default="both",
                        help="which entities --mainloop assigns: the signalling "
                             "entity (DSIG_ID, carries the CAI), the network "
                             "layer (NL_ID, carries LLI/LLC/DLC), or both in "
                             "the driver's order (default)")
    parser.add_argument("--connect", action="store_true",
                        help="after linked SIG+NL assignment, submit the "
                             "network N_CONNECT that activates the bearer")
    parser.add_argument("--call-direction", choices=("calling", "answering"),
                        default="calling",
                        help="select outgoing V42 or incoming V42_IN bearer "
                             "semantics for the modem NL entity")
    parser.add_argument("--simulate-b-channel", action="store_true",
                        help="simulate an answered incoming call through the "
                             "native SETUP/CALL_IND, CALL_RES, NL activation, "
                             "and TIKRNL service-assignment path")
    parser.add_argument("--force-modem-dsp-assign", action="store_true",
                        help="after N_CONNECT, run the recovered "
                             "SERVICE_ASSIGN path against a directly staged "
                             "TIKRNL core so the modem DSP receives a real "
                             "switch-on database commit")
    parser.add_argument("--g711-probe-samples", type=int, default=0,
                        help="after forced modem DSP assignment, feed this "
                             "many raw G.711 octets into the direct TIKRNL "
                             "core's line words (RX-side probe only)")
    parser.add_argument("--g711-probe-code", type=lambda s: int(s, 0),
                        default=0xff,
                        help="raw G.711 octet used by --g711-probe-samples "
                             "(default 0xff)")
    parser.add_argument("--g711-probe-stimulus",
                        choices=("constant", "tone", "ansam", "silence"),
                        default="constant",
                        help="stimulus used by --g711-probe-samples "
                             "(default constant)")
    parser.add_argument("--g711-probe-freq", type=float, default=2100.0,
                        help="tone/ANSam carrier frequency for "
                             "--g711-probe-stimulus (default 2100)")
    parser.add_argument("--g711-probe-amp", type=int, default=20000,
                        help="linear PCM amplitude before u-law encoding for "
                             "tone/ANSam stimulus (default 20000)")
    parser.add_argument("--bridge-task-tx", action="store_true",
                        help="during --g711-probe-samples, copy the "
                             "pointer-mode task TX buffer to the kernel "
                             "SPORT0 TX latch before each TDM strobe")
    parser.add_argument("--restore-pcm-pointers", action="store_true",
                        help="during --g711-probe-samples, restore the old "
                             "one-line pointer-mode PCM block "
                             "(3F0F->2B00, 3FB4->2B01) after overlays")
    parser.add_argument("--tx-source-scan", action="store_true",
                        help="after forced modem DSP assignment, poke likely "
                             "DM TX buffers with --tx-source-marker and report "
                             "whether SPORT0 TX0 emits the marker")
    parser.add_argument("--tx-source-marker", type=lambda s: int(s, 0),
                        default=0x0055,
                        help="16-bit marker used by --tx-source-scan "
                             "(default 0x0055)")
    parser.add_argument("--tx-scan-start", type=lambda s: int(s, 0),
                        default=None,
                        help="optional inclusive DM start address for a wider "
                             "--tx-source-scan")
    parser.add_argument("--tx-scan-end", type=lambda s: int(s, 0),
                        default=None,
                        help="optional exclusive DM end address for a wider "
                             "--tx-source-scan")
    parser.add_argument("--force-tx-samples", type=int, default=0,
                        help="after forced modem DSP assignment, preload the "
                             "kernel TDM TX source and capture this many "
                             "SPORT0 TX0 words")
    parser.add_argument("--force-tx-code", type=lambda s: int(s, 0),
                        default=0x0055,
                        help="G.711/codeword marker used by --force-tx-samples "
                             "(default 0x0055)")
    parser.add_argument("--force-tx-source", type=lambda s: int(s, 0),
                        default=0x2E52,
                        help="DM source address used by --force-tx-samples "
                             "(default 0x2E52, kernel TDM output latch)")
    parser.add_argument("--fake-call-ingress", action="store_true",
                        help="drive the incoming-call host sequence before "
                             "answering: put the assigned signalling entity "
                             "into LISTEN/INDICATE state, then use the normal "
                             "CALL_RES + N_CONNECT path")
    parser.add_argument("--legacy-sig-req-id", action="store_true",
                        help="use ReqId=1 for simple signalling requests, "
                             "matching the old i4l idi_put_req() helper")
    parser.add_argument("--inject-call-ingress", action="store_true",
                        help="after LISTEN/INDICATE, run the internal "
                             "firmware branch that allocates the incoming "
                             "per-call object before CALL_RES")
    parser.add_argument("--synthesize-call-ingress", action="store_true",
                        help="after LISTEN/INDICATE, fabricate the minimum "
                             "incoming call object expected by CALL_RES")
    parser.add_argument("--ingress-entity-slot", type=int, default=0,
                        help="entity table slot whose listener object receives "
                             "the fake ingress")
    parser.add_argument("--call-steps", type=int, default=64,
                        help="MIPS main-loop iterations to run after N_CONNECT")
    parser.add_argument("--dsp-pump", type=int, default=256,
                        help="MIPS instructions between DSP time slices during "
                             "--mainloop; the DSPs must run in line with the "
                             "MIPS for the boot handshake to complete.  0 holds "
                             "them for the whole run instead.")
    parser.add_argument("--dsp-code-base", type=lambda s: int(s, 0), default=None,
                        help="override DspCodeBaseAddr (default: the protocol "
                             "image's OFFS_PROTOCOL_END_ADDR)")
    parser.add_argument("--log", action="store_true")
    parser.add_argument("--trace-calls", action="store_true",
                        help="record MIPS jal/jalr call targets per harness phase")
    parser.add_argument("--trace-call-limit", type=int, default=24,
                        help="number of hot call targets to print per phase")
    parser.add_argument("--dump-entities", action="store_true",
                        help="dump the firmware entity pointer table after "
                             "incoming-call state transitions")
    parser.add_argument("--dump-entity-limit", type=int, default=16,
                        help="maximum number of entity table slots to dump")
    parser.add_argument("--native-dm-out", type=Path,
                        help="write the naturally assigned modem core's full "
                             "0x4000-word DM image after --mainloop")
    args = parser.parse_args()
    if args.simulate_b_channel:
        args.connect = True
        args.call_direction = "answering"
        args.fake_call_ingress = True
        args.inject_call_ingress = True
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
    shim.trace_calls = args.trace_calls

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
        if ADSP.adsp2181_dm(cpu)[0x3315] != ADSP.adsp2181_dm(cpu)[0x3316]:
            ADSP.adsp2181_host_write(cpu, 0x7310, 0x0001)
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
        # TIKRNL init publishes 0x05B1 as its command service vector at
        # DM 0x3308.  The frame initializer above does not consume the
        # host->task database ring by itself; invoke the published service
        # vector to break the assignment/dispatch bootstrap cycle.
        service_vector = ADSP.adsp2181_dm(cpu)[0x3308]
        ADSP.adsp2181_call(cpu, service_vector, 0x02A8)
        ADSP.adsp2181_run(cpu, 20000)
        ADSP.adsp2181_call(cpu, 0x06BB, 0x02A8)
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
