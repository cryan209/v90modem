#!/usr/bin/env python3
"""Drive DIAL standalone, bypassing the kernel task dispatcher.

DIAL reads its line/RX samples from DM 0x3F08/0x3F09 (the data-pump database
line registers), NOT from the kernel's 0x2E52 timeslot sink. The kernel
normally bridges 0x2E52 -> 0x3F08/0x3F09 per frame, but only when a task is
assigned. We skip that: boot kernel+DIAL, then enter DIAL's line handler
(0x1BBD) directly and feed μ-law samples into 0x3F08/0x3F09 ourselves,
strobing the DIAL state dispatcher (0x1B9C) and watching 0x3FB0.

DIAL's two entry points:
  0x1B9C - state dispatcher (reads 0x3FB0, picks action)
  0x1BBD - line/input handler (reads 0x3F08/0x3F09, updates state)

We alternate: drive a sample into 0x3F08/0x3F09, run the line handler, then
run the state dispatcher, and watch whether the state machine advances.
"""
import argparse, ctypes, math
from pathlib import Path

ADSP = ctypes.CDLL(str(Path(__file__).resolve().parent /
                       "adsp2181emu" / "libadsp2181.dylib"))
ADSP.adsp2181_create.restype = ctypes.c_void_p
for n, a in [('reset', [ctypes.c_void_p]), ('pm', [ctypes.c_void_p]),
             ('dm', [ctypes.c_void_p]), ('run', [ctypes.c_void_p, ctypes.c_int]),
             ('pc', [ctypes.c_void_p]), ('idle', [ctypes.c_void_p]),
             ('set_pc', [ctypes.c_void_p, ctypes.c_uint16]),
             ('call', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16])]:
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


MULAW_BIAS = 0x84


def linear_to_mulaw(s):
    s = max(-32768, min(32767, s))
    sign = 0x80 if s < 0 else 0
    if s < 0:
        s = -s - 1
    s += MULAW_BIAS
    if s > 0x7FFF:
        s = 0x7FFF
    seg = 0
    t = s >> 5
    while t and seg < 8:
        t >>= 1
        seg += 1
    if seg >= 8:
        return sign | 0x7F ^ 0xFF
    mant = (s >> (seg + 3)) & 0xF
    return (sign | (seg << 4) | mant) ^ 0xFF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=400)
    ap.add_argument('--freq', type=int, default=440)
    ap.add_argument('--amp', type=int, default=20000)
    args = ap.parse_args()

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

    # build 1s of μ-law tone
    samples = [linear_to_mulaw(int(args.amp * math.sin(
        2 * math.pi * args.freq * i / 8000))) for i in range(8000)]

    print(f'[standalone] DIAL loaded; freq={args.freq}Hz frames={args.frames}')
    print(f'[standalone] initial: 3F08={dm[0x3F08]:04x} 3F09={dm[0x3F09]:04x} '
          f'3FB0={dm[0x3FB0]:04x}')

    prev = (dm[0x3F08], dm[0x3F09], dm[0x3FB0], dm[0x3FB2], dm[0x3FB3])
    for f in range(args.frames):
        # feed a sample into the line registers
        s = samples[f % len(samples)]
        dm[0x3F08] = s
        dm[0x3F09] = s
        # run DIAL's line handler (0x1BBD) as a called subroutine
        ADSP.adsp2181_call(cpu, 0x1BBD, 0x02A8)
        ADSP.adsp2181_run(cpu, 5000)
        # run DIAL's state dispatcher (0x1B9C)
        ADSP.adsp2181_call(cpu, 0x1B9C, 0x02A8)
        ADSP.adsp2181_run(cpu, 5000)
        now = (dm[0x3F08], dm[0x3F09], dm[0x3FB0], dm[0x3FB2], dm[0x3FB3])
        if now != prev:
            print(f'  frame {f:4d}: 3F08={now[0]:04x} 3F09={now[1]:04x} '
                  f'3FB0={now[2]:04x} 3FB2={now[3]:04x} 3FB3={now[4]:04x}')
            prev = now
    print(f'[standalone] final: 3F08={dm[0x3F08]:04x} 3F09={dm[0x3F09]:04x} '
          f'3FB0={dm[0x3FB0]:04x}')


if __name__ == '__main__':
    main()
