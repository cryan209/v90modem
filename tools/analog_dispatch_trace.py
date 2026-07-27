#!/usr/bin/env python3
"""Reverse the Analog kernel's channel-descriptor format by tracing the
service routine 0x01C4 with planted descriptors.

The Analog kernel (id 0x000d) is a single-channel analog modem kernel, far
simpler than the PRI 30M. Its idle loop (0x02A6) checks 0x2E00==0x2E01
(queue head/tail); the service routine 0x01C4 walks a free-list at
0x2E7A/0x2E7B, reads descriptors, and does CALL (I4) where I4 comes from a
descriptor field.

We plant a free-list entry pointing at a descriptor with a candidate I4
vector at each offset, make the queue non-empty, and trace which offset
becomes I4 (i.e. which makes the dispatcher CALL our target).
"""
import ctypes
from pathlib import Path

ADSP = ctypes.CDLL(str(Path(__file__).resolve().parent /
                       "adsp2181emu" / "libadsp2181.dylib"))
ADSP.adsp2181_create.restype = ctypes.c_void_p
for n, a in [('reset', [ctypes.c_void_p]), ('pm', [ctypes.c_void_p]),
             ('dm', [ctypes.c_void_p]), ('run', [ctypes.c_void_p, ctypes.c_int]),
             ('pc', [ctypes.c_void_p]), ('idle', [ctypes.c_void_p]),
             ('set_pc', [ctypes.c_void_p, ctypes.c_uint16]),
             ('set_flagin', [ctypes.c_void_p, ctypes.c_int])]:
    getattr(ADSP, 'adsp2181_' + n).argtypes = a
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int

TARGET = 0x1FF4  # a distinctive V.8-ish address to detect the CALL landing


def rw(p):
    w = {}
    for line in Path(p).read_text().splitlines():
        f = line.split()
        if len(f) == 2:
            w[int(f[0], 16)] = int(f[1], 16)
    return w


def main():
    repo = Path(__file__).resolve().parent.parent
    A = str(repo / 'artifacts/eicon-dsp/kernels/000d-diva-server-analog-kernel')
    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for a, v in rw(A + '/pm.words').items():
        pm[a] = v
    for a, v in rw(A + '/dm.words').items():
        dm[a] = v
    ADSP.adsp2181_run(cpu, 3000)  # boot to IDLE
    print(f'[trace] booted: pc={ADSP.adsp2181_pc(cpu):04x} idle={ADSP.adsp2181_idle(cpu)}')

    # The free-list head is 0x2E7A. Plant a single descriptor at 0x0E40
    # and link it: 0x2E7A -> 0x0E40 (one entry), and make queue non-empty.
    DESC = 0x0E40
    # Plant the candidate vector at each field of the descriptor and see
    # which one the dispatcher picks as I4.
    for off in range(0, 12):
        # fresh reload
        for a in range(0x4000):
            pm[a] = 0
        for a, v in rw(A + '/pm.words').items():
            pm[a] = v
        for a in range(0x4000):
            dm[a] = 0
        for a, v in rw(A + '/dm.words').items():
            dm[a] = v
        ADSP.adsp2181_reset(cpu)
        ADSP.adsp2181_run(cpu, 3000)
        # plant: free-list head -> DESC, with TARGET at DESC+off
        dm[0x2E7A] = DESC
        dm[0x2E7B] = DESC
        dm[DESC + off] = TARGET
        # make queue non-empty so dispatcher runs
        dm[0x2E00] = DESC
        dm[0x2E01] = DESC + 8
        ADSP.adsp2181_set_pc(cpu, 0x02A6)  # idle loop entry
        hit = None
        for i in range(200):
            ADSP.adsp2181_run(cpu, 1)
            pc = ADSP.adsp2181_pc(cpu)
            if pc == TARGET:
                hit = off
                break
            if ADSP.adsp2181_idle(cpu):
                break
        print(f'  DESC+0x{off:02x} = TARGET: hit={hit}')
        if hit is not None:
            print(f'  *** descriptor field +0x{off:02x} is the I4 dispatch vector ***')
            return


if __name__ == '__main__':
    main()
