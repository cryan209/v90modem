#!/usr/bin/env python3
"""Find the channel-descriptor vector field by planting candidate vectors
and checking whether the kernel foreground dispatcher reaches DIAL.

The dispatcher at PM 0x02a4 does IF EQ CALL (I4) where I4 = SR0, set by the
service routine 0x00D8 reading the per-frame descriptor at DM 0x2F00 and
the channel table at 0x2E40+. We poke candidate DIAL entry addresses into
each plausible field and run, watching whether PC reaches a DIAL address
(0x08F0, 0x1B9C, 0x1BBD).
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

DIAL_ENTRIES = {0x08F0, 0x1B9C, 0x1BBD, 0x1BCE}


def rw(p):
    w = {}
    for line in Path(p).read_text().splitlines():
        f = line.split()
        if len(f) == 2:
            w[int(f[0], 16)] = int(f[1], 16)
    return w


def fresh(cpu, pm, dm, K, D):
    ADSP.adsp2181_reset(cpu)
    for a, v in rw(K + '/pm.words').items():
        pm[a] = v
    for a, v in rw(K + '/dm.words').items():
        dm[a] = v
    ADSP.adsp2181_run(cpu, 2000)
    for a, v in rw(D + '/pm.words').items():
        pm[a] = v
    for a, v in rw(D + '/dm.words').items():
        dm[a] = v


def try_poke(cpu, pm, dm, K, D, label, pokes):
    fresh(cpu, pm, dm, K, D)
    for addr, val in pokes:
        dm[addr] = val & 0xFFFF
    ADSP.adsp2181_set_pc(cpu, 0x02a9)
    reached = None
    for i in range(200):
        ADSP.adsp2181_run(cpu, 1)
        pc = ADSP.adsp2181_pc(cpu)
        if pc in DIAL_ENTRIES:
            reached = pc
            break
        if ADSP.adsp2181_idle(cpu):
            break
    print(f'{label}: reached={reached}')
    return reached


def main():
    repo = Path(__file__).resolve().parent.parent
    K = str(repo / 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel')
    D = str(repo / 'artifacts/eicon-dsp/dial/0262-dial-fsk-fax.f34-overlay')
    cpu = ADSP.adsp2181_create()
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)

    V = 0x1B9C  # DIAL dispatcher
    # 1. just make queue non-empty
    try_poke(cpu, pm, dm, K, D, 'queue-nonempty only',
             [(0x2E44, 0x2E40), (0x2E45, 0x2E48)])
    # 2. per-frame descriptor vector field +1 (the 0x2800 slot)
    try_poke(cpu, pm, dm, K, D, '2F01=vector',
             [(0x2E44, 0x2E40), (0x2E45, 0x2E48), (0x2F01, V)])
    # 3. channel descriptor at 0x2E40, various offsets
    for off in range(0, 8):
        try_poke(cpu, pm, dm, K, D, f'2E40+{off}=vector',
                 [(0x2E44, 0x2E40), (0x2E45, 0x2E48),
                  (0x2E40 + off, V)])
    # 4. 2F00 field +0
    try_poke(cpu, pm, dm, K, D, '2F00=vector',
             [(0x2E44, 0x2E40), (0x2E45, 0x2E48), (0x2F00, V)])
    # 5. service slot 2F27/2F28
    try_poke(cpu, pm, dm, K, D, '2F27=vector',
             [(0x2E44, 0x2E40), (0x2E45, 0x2E48), (0x2F27, V)])
    try_poke(cpu, pm, dm, K, D, '2F28=vector',
             [(0x2E44, 0x2E40), (0x2E45, 0x2E48), (0x2F28, V)])


if __name__ == '__main__':
    main()
