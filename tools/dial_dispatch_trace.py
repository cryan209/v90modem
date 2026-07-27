#!/usr/bin/env python3
"""Trace the kernel foreground dispatcher to find the channel-descriptor
fields that produce the I4 task vector.

Boots kernel+DIAL, then forces the queue non-empty (0x2E44 != 0x2E45) and
single-steps the foreground from 0x02ad, logging every DM/PM read/write at
the dispatcher-relevant addresses so we can see which fields feed I4.
"""
import ctypes
from pathlib import Path

ADSP = ctypes.CDLL(str(Path(__file__).resolve().parent /
                       "adsp2181emu" / "libadsp2181.dylib"))
ADSP.adsp2181_create.restype = ctypes.c_void_p
for n, a in [('reset', [ctypes.c_void_p]), ('pm', [ctypes.c_void_p]),
             ('dm', [ctypes.c_void_p]), ('run', [ctypes.c_void_p, ctypes.c_int]),
             ('pc', [ctypes.c_void_p]), ('idle', [ctypes.c_void_p]),
             ('set_pc', [ctypes.c_void_p, ctypes.c_uint16])]:
    getattr(ADSP, 'adsp2181_' + n).argtypes = a
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int


def rw(p):
    w = {}
    for line in Path(p).read_text().splitlines():
        f = line.split()
        if len(f) == 2:
            w[int(f[0], 16)] = int(f[1], 16)
    return w


def main():
    repo = Path(__file__).resolve().parent.parent
    K = str(repo / 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel')
    D = str(repo / 'artifacts/eicon-dsp/dial/0262-dial-fsk-fax.f34-overlay')
    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for a, v in rw(K + '/pm.words').items():
        pm[a] = v
    for a, v in rw(K + '/dm.words').items():
        dm[a] = v
    ADSP.adsp2181_run(cpu, 2000)
    for a, v in rw(D + '/pm.words').items():
        pm[a] = v
    for a, v in rw(D + '/dm.words').items():
        dm[a] = v

    # force queue non-empty: head != tail
    dm[0x2E44] = 0x2E40      # head -> a channel descriptor
    dm[0x2E45] = 0x2E48      # tail -> different
    # plant a plausible descriptor at 0x2E40 with a vector field
    # (we'll see which offset the dispatcher reads as the vector)
    dm[0x2E40] = 0x1B9C      # candidate: DIAL dispatcher
    dm[0x2E41] = 0x08F0      # candidate: DIAL entry
    dm[0x2E42] = 0x1B9C
    dm[0x2E43] = 0x08F0
    dm[0x2E46] = 0x1B9C
    dm[0x2E47] = 0x08F0

    print(f'planted: 2E44={dm[0x2E44]:04x} 2E45={dm[0x2E45]:04x}')
    ADSP.adsp2181_set_pc(cpu, 0x02a9)   # re-enter idle check
    # step and log
    watched = set(range(0x2E40, 0x2E60)) | set(range(0x2F00, 0x2F30)) | {0x2E78, 0x2E79}
    for i in range(120):
        pc = ADSP.adsp2181_pc(cpu)
        op = pm[pc] & 0xffffff
        # crude disasm of DM-touching ops
        note = ''
        top = op >> 16
        if 0x80 <= top <= 0x87:   # DM read to reg
            addr = (op >> 4) & 0x3fff
            if addr in watched:
                note = f'  R DM[{addr:04x}]={dm[addr]:04x}'
        elif 0x90 <= top <= 0x97:  # DM write from reg
            addr = (op >> 4) & 0x3fff
            if addr in watched:
                note = f'  W DM[{addr:04x}]={dm[addr]:04x}'
        elif 0x8a <= top <= 0x8f:  # reg = DM (I-reg load etc)
            addr = (op >> 4) & 0x3fff
            if addr in watched:
                note = f'  R DM[{addr:04x}]={dm[addr]:04x}'
        elif 0x9a <= top <= 0x9f:
            addr = (op >> 4) & 0x3fff
            if addr in watched:
                note = f'  W DM[{addr:04x}]={dm[addr]:04x}'
        print(f'{i:3d} pc={pc:04x} op={op:06x}{note}')
        ADSP.adsp2181_run(cpu, 1)
        if ADSP.adsp2181_idle(cpu):
            print('  IDLE reached')
            break
        npc = ADSP.adsp2181_pc(cpu)
        if npc == 0x1B9C or npc == 0x08F0 or npc == 0x1BBD:
            print(f'  *** reached DIAL entry {npc:04x} ***')
            break


if __name__ == '__main__':
    main()
