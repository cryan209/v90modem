#!/usr/bin/env python3
"""Probe Eicon PRI/SIG ADSP task registration and doorbell state.

This is a narrow companion to the MIPS shim.  It loads one ADSP kernel plus
one extracted SIG task, calls the task's download entry point, and dumps the
kernel patch slots and SIG state words that tell us how the PRI/E1 signalling
DSP path normally connects to the host-facing firmware.
"""

from __future__ import annotations

import argparse
import ctypes
import os
from pathlib import Path


REPO = Path(__file__).resolve().parent.parent
ADSP = ctypes.CDLL(os.environ.get(
    "ADSP2181_LIB",
    str(REPO / "tools/adsp2181emu/libadsp2181.dylib")))

ADSP.adsp2181_create.restype = ctypes.c_void_p
ADSP.adsp2181_destroy.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_reset.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pm.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_run.argtypes = [ctypes.c_void_p, ctypes.c_int]
ADSP.adsp2181_call.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]
ADSP.adsp2181_pc.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_idle.restype = ctypes.c_int
ADSP.adsp2181_watch_dm.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int]
ADSP.adsp2181_watch_pm.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int]
ADSP.adsp2181_trace_budget.argtypes = [ctypes.c_void_p, ctypes.c_int64]


KERNEL_IDLE = 0x02A8
RING_POINTERS = {
    0x2F27: 0x2F21,
    0x2F28: 0x2F00,
    0x2F29: 0x2F0E,
    0x2F2A: 0x2F42,
    0x2F2B: 0x2F4E,
}
PATCH_SLOTS = (0x02B9, 0x00B5)

TASKS = {
    "sigmdm": {
        "kernel": "artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel",
        "task": "artifacts/eicon-dsp/sig-path/0208-sig.mdm-task",
        "entry": 0x0980,
        "state": (0x0000, 0x0001, 0x008A, 0x0090, 0x009B, 0x009C, 0x009D,
                  0x061A, 0x0660, 0x0672, 0x0673, 0x0674, 0x0675, 0x0676,
                  0x0680, 0x0681, 0x0682, 0x0683, 0x0689, 0x068A, 0x068B,
                  0x068C, 0x068D, 0x068E, 0x068F, 0x0690, 0x0691, 0x0692,
                  0x0693, 0x06AB, 0x06AC, 0x06D3),
    },
    "sigprtx": {
        "kernel": "artifacts/eicon-dsp/sig-path/000b-diva-server-pri-2m-tx-sig-kernel",
        "task": "artifacts/eicon-dsp/sig-path/0209-sigprtx-task",
        "entry": 0x3900,
        "state": (0x0A01, 0x0A0B, 0x0A0C, 0x0A0D, 0x0A0E, 0x0A0F,
                  0x0A19, 0x0A1A, 0x0A1B),
    },
    "sigprrx": {
        "kernel": "artifacts/eicon-dsp/sig-path/000c-diva-server-pri-2m-rx-sig-kernel",
        "task": "artifacts/eicon-dsp/sig-path/020a-sigprrx-task",
        "entry": 0x3900,
        "state": (0x0A01, 0x0A0B, 0x0A0C, 0x0A0D, 0x0A0E, 0x0A0F,
                  0x0A12, 0x0A16, 0x0A17, 0x0A1B, 0x0A1C, 0x0A1D,
                  0x0A22, 0x0A23, 0x0A25, 0x0A27, 0x0A2A, 0x0A2B),
    },
}


def read_words(path: Path) -> dict[int, int]:
    words: dict[int, int] = {}
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) == 2:
            words[int(fields[0], 16)] = int(fields[1], 16)
    return words


def load_module(pm, dm, module: Path) -> set[int]:
    loaded_pm: set[int] = set()
    for addr, value in read_words(module / "pm.words").items():
        pm[addr] = value
        loaded_pm.add(addr)
    for addr, value in read_words(module / "dm.words").items():
        dm[addr] = value
    return loaded_pm


def fmt_pm(word: int) -> str:
    return f"{word & 0xffffff:06x}"


def run_until_idle(cpu, loaded_pm: set[int], budget: int) -> dict[int, int]:
    hist: dict[int, int] = {}
    for _ in range(budget):
        pc = int(ADSP.adsp2181_pc(cpu))
        hist[pc] = hist.get(pc, 0) + 1
        if pc not in loaded_pm and pc != KERNEL_IDLE:
            print(f"[probe] stopped at unpopulated PM 0x{pc:04x}")
            break
        ADSP.adsp2181_run(cpu, 1)
        if ADSP.adsp2181_idle(cpu) or int(ADSP.adsp2181_pc(cpu)) == KERNEL_IDLE:
            break
    return hist


def changed_pm(pm, before: list[int], loaded_pm: set[int]) -> list[tuple[int, int, int]]:
    changes: list[tuple[int, int, int]] = []
    for addr in sorted(loaded_pm):
        old = before[addr] & 0xffffff
        new = pm[addr] & 0xffffff
        if old != new:
            changes.append((addr, old, new))
    return changes


def dump_words(dm, addresses: tuple[int, ...], label: str) -> None:
    print(f"[probe] {label}:")
    for i in range(0, len(addresses), 8):
        chunk = addresses[i:i + 8]
        print("  " + " ".join(f"{addr:04x}={dm[addr]:04x}" for addr in chunk))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", choices=sorted(TASKS))
    parser.add_argument("--entry", type=lambda s: int(s, 0),
                        help="override task entry point")
    parser.add_argument("--cycles", type=int, default=50000)
    parser.add_argument("--trace", type=int, default=0,
                        help="ADSP emulator PM/DM trace budget")
    args = parser.parse_args()

    cfg = TASKS[args.task]
    kernel = REPO / cfg["kernel"]
    task = REPO / cfg["task"]
    entry = args.entry if args.entry is not None else cfg["entry"]

    cpu = ADSP.adsp2181_create()
    try:
        ADSP.adsp2181_reset(cpu)
        if args.trace:
            ADSP.adsp2181_trace_budget(cpu, args.trace)
            for addr in (*PATCH_SLOTS, 0x000A, 0x0017, 0x0019):
                ADSP.adsp2181_watch_pm(cpu, addr, 1)
            for addr in (*RING_POINTERS, *cfg["state"]):
                ADSP.adsp2181_watch_dm(cpu, addr, 1)

        pm = ADSP.adsp2181_pm(cpu)
        dm = ADSP.adsp2181_dm(cpu)
        loaded_pm = load_module(pm, dm, kernel)
        ADSP.adsp2181_run(cpu, 5000)
        print(f"[probe] kernel={kernel.name} pc=0x{int(ADSP.adsp2181_pc(cpu)):04x} "
              f"idle={int(ADSP.adsp2181_idle(cpu))}")

        for addr, value in RING_POINTERS.items():
            dm[addr] = value
        print("[probe] planted kernel ring pointers: "
              + " ".join(f"{addr:04x}={value:04x}"
                         for addr, value in RING_POINTERS.items()))

        loaded_pm |= load_module(pm, dm, task)
        before_pm = [pm[addr] for addr in range(0x4000)]
        before_slots = {addr: pm[addr] for addr in PATCH_SLOTS}
        print(f"[probe] task={task.name} entry=0x{entry:04x}")
        print("[probe] patch slots before: "
              + " ".join(f"PM{addr:04x}={fmt_pm(value)}"
                         for addr, value in before_slots.items()))

        ADSP.adsp2181_call(cpu, entry & 0x3fff, KERNEL_IDLE)
        hist = run_until_idle(cpu, loaded_pm, args.cycles)
        after_slots = {addr: pm[addr] for addr in PATCH_SLOTS}
        print(f"[probe] after entry pc=0x{int(ADSP.adsp2181_pc(cpu)):04x} "
              f"idle={int(ADSP.adsp2181_idle(cpu))} visited={len(hist)}")
        print("[probe] patch slots after:  "
              + " ".join(f"PM{addr:04x}={fmt_pm(value)}"
                         for addr, value in after_slots.items()))
        changes = changed_pm(pm, before_pm, loaded_pm)
        print(f"[probe] PM changes: {len(changes)}")
        for addr, old, new in changes[:32]:
            print(f"  PM{addr:04x}: {old:06x} -> {new:06x}")
        dump_words(dm, tuple(cfg["state"]), "task state")

        hot = sorted(hist.items(), key=lambda item: item[1], reverse=True)[:12]
        print("[probe] hot PCs: "
              + " ".join(f"{pc:04x}:{count}" for pc, count in hot))
    finally:
        ADSP.adsp2181_destroy(cpu)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
