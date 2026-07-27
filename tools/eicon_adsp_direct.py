#!/usr/bin/env python3
"""Direct-drive the Eicon ADSP-2181 without the MIPS firmware.

Compose a flat kernel+overlay image, load into the ADSP emulator, and
manually initialize the modem using the ADDSP data-pump database layout
(DM 0x3EE0–0x3F7F).  No Unicorn, no MIPS.

Usage:
    python3 tools/eicon_adsp_direct.py \
        artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel \
        artifacts/eicon-dsp/build-117-926/tikrnl/0258-tikrnl81.f34-task \
        [overlay_dir ...] \
        --entry 0x672
"""

from __future__ import annotations

import argparse
import ctypes
import json
import struct
import sys
from pathlib import Path

ADSP = ctypes.CDLL(str(Path(__file__).resolve().parent /
                     "adsp2181emu" / "libadsp2181.dylib"))

ADSP.adsp2181_create.restype = ctypes.c_void_p
ADSP.adsp2181_destroy.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_reset.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pm.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_run.argtypes = [ctypes.c_void_p, ctypes.c_int]
ADSP.adsp2181_call.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
ADSP.adsp2181_set_pc.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_idle.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_idle.restype = ctypes.c_int
ADSP.adsp2181_host_write.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
ADSP.adsp2181_host_read.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_host_read.restype = ctypes.c_uint16


def read_words(path: Path) -> dict[int, int]:
    words: dict[int, int] = {}
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) == 2:
            words[int(fields[0], 16)] = int(fields[1], 16)
    return words


def build_sparse_image(modules: list[Path], allow_conflicts: bool = True) -> tuple[bytes, bytes]:
    """Compose PM/DM binary images from ordered module directories."""
    pm: dict[int, int] = {}
    dm: dict[int, int] = {}
    for module in modules:
        for addr, val in read_words(module / "pm.words").items():
            pm[addr] = val
        for addr, val in read_words(module / "dm.words").items():
            dm[addr] = val

    pm_bin = bytearray(0x10000 * 3)
    for addr, val in pm.items():
        pm_bin[addr * 3:(addr + 1) * 3] = val.to_bytes(3, "little")

    dm_bin = bytearray(0x10000 * 2)
    for addr, val in dm.items():
        dm_bin[addr * 2:(addr + 1) * 2] = val.to_bytes(2, "little")

    return bytes(pm_bin), bytes(dm_bin)


def load_bins(cpu, pm_bin: bytes, dm_bin: bytes) -> None:
    pm = ADSP.adsp2181_pm(cpu)
    for a in range(0x4000):
        pm[a] = pm_bin[a * 3] | (pm_bin[a * 3 + 1] << 8) | (pm_bin[a * 3 + 2] << 16)
    dm = ADSP.adsp2181_dm(cpu)
    for a in range(0x4000):
        dm[a] = dm_bin[a * 2] | (dm_bin[a * 2 + 1] << 8)


def poke_db(cpu, addr: int, value: int) -> None:
    """Write a word to the data-pump database (DM address)."""
    ADSP.adsp2181_host_write(cpu, addr & 0x7FFF, value & 0xFFFF)


def read_db(cpu, addr: int) -> int:
    return ADSP.adsp2181_host_read(cpu, addr & 0x7FFF)


def print_db(cpu, base: int = 0x3EE0, count: int = 128) -> None:
    for a in range(base, base + count, 8):
        vals = [read_db(cpu, a + i) for i in range(8)]
        print(f"  DM{a:04x}: " + " ".join(f"{v:04x}" for v in vals))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("modules", nargs="+", type=Path,
                        help="ordered module directories (kernel first, overlays after)")
    parser.add_argument("--entry", type=lambda x: int(x, 0), default=None,
                        help="entry point PC to call after reset (e.g. 0x672 for TIKRNL init)")
    parser.add_argument("--init", action="store_true",
                        help="run database initialization sequence (answer-mode V.90)")
    parser.add_argument("--no-dial", action="store_true",
                        help="skip the DIAL overlay; load kernel only")
    parser.add_argument("--dump-db", action="store_true",
                        help="dump DM database after init")
    parser.add_argument("--log", action="store_true")
    parser.add_argument("--cycles", type=int, default=100000)
    args = parser.parse_args()

    pm_bin, dm_bin = build_sparse_image(args.modules)

    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    load_bins(cpu, pm_bin, dm_bin)

    # Run to IDLE (boot code goes to idle loop)
    ADSP.adsp2181_run(cpu, 1000)
    print(f"[direct] boot pc={ADSP.adsp2181_pc(cpu):04x} idle={ADSP.adsp2181_idle(cpu)}")

    # Call the TIKRNL initializer (or whatever entry point the module expects)
    if args.entry is not None:
        ADSP.adsp2181_call(cpu, args.entry & 0x3FFF, 0x02A8)
        ran = ADSP.adsp2181_run(cpu, 1000000)
        print(f"[direct] after entry=0x{args.entry:04x} pc={ADSP.adsp2181_pc(cpu):04x} "
              f"idle={ADSP.adsp2181_idle(cpu)} ran={ran}")

    if args.init:
        # ADDSP §5.4.1 Table 12: data-pump initialisation sequence
        init_writes = [
            (0x3EE0, 0x00C4),  # GEN_SETUP0
            (0x3EE1, 0x0040),  # GEN_SETUP1
            (0x3EE2, 0x0000),  # GEN_SETUP2
            (0x3EE3, 0xF0FD),  # INFO0_SETUP
            (0x3EE4, 0x0006),  # TD
            (0x3EE5, 0x0006),  # TA
            (0x3EE6, 0x00FF),  # TX_LEVEL_TUNE
            (0x3EE7, 0x0030),  # DCD_OFF
            (0x3EE8, 0x0000),  # DCD_HYST
            (0x3EE9, 0x0003),  # MINTIMER
            (0x3EEA, 0x0003),  # MAXTIMER
            (0x3F00, 0x2000),  # WSTATUS (bit D set to activate)
        ]
        for addr, val in init_writes:
            poke_db(cpu, addr, val)
        print(f"[direct] wrote {len(init_writes)} init words")

        # ADDSP §5.4.1 Table 13: modulation selection
        # V.90 = 0x8000 in Norm_L
        norm_writes = [
            (0x3EF0, 0x8000),  # Norm_L (V.90)
            (0x3EEF, 0x0001),  # Norm_H (V8)
            (0x3F00, 0x2000),  # WSTATUS.activate
        ]
        for addr, val in norm_writes:
            poke_db(cpu, addr, val)
        print(f"[direct] wrote norm selection (V.90)")

    if args.dump_db:
        print("[direct] database dump (DM 0x3EE0-0x3F7F):")
        print_db(cpu)

    # Pump cycles to let the DSP process
    ran = ADSP.adsp2181_run(cpu, args.cycles)
    print(f"[direct] final pc={ADSP.adsp2181_pc(cpu):04x} idle={ADSP.adsp2181_idle(cpu)} ran={ran}")
    ADSP.adsp2181_destroy(cpu)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
