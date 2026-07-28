#!/usr/bin/env python3
"""Host-supervisor model: DIAL -> V.8 page transition + V.8 output capture.

Runs DIAL through its REAL entry path (PM 0x08f0 with FLAG_IN strobed), so the
full task loop executes -- including the NORM-mode handler (0x13CC) that
consumes the GEN_SETUP1 NORM bit and updates TrnProgress.

The host supervisor (this script, modeling guide §5.4.2) polls TrnProgress
(DM 0x3FAD) and RSTATUS (0x3FA5). When DIAL signals training-start, the host
"loads" the V.8 bootpage: overlays the V.8 PM/DM image (replacing DIAL),
sets BOOTFINISHED (bit 0xC) in WSTATUS (0x3EEE), and continues running V.8
through its 0x08f0 entry.

Captures the TX output: samples DIAL/V.8 write to the line TX registers
(0x3F08/0x3F09 are RX; TX is the same registers in the opposite direction
per the kernel bridge, or the TXD locations). We sample 0x3F08/0x3F09 each
frame after the DSP runs -- the DSP reads RX there and writes TX there.
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
             ('call', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]),
             ('host_write', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]),
             ('set_flagin', [ctypes.c_void_p, ctypes.c_int])]:
    getattr(ADSP, 'adsp2181_' + n).argtypes = a
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int

DB = 0x3EE0
BOOTPAGE_NR = 0x3FB0
RSTATUS = 0x3FA5
TRNPROG = 0x3FAD
WSTATUS = 0x3EEE
LINE_RX = 0x3F09
LINE_RX2 = 0x3F08


def rw(p):
    w = {}
    for line in Path(p).read_text().splitlines():
        f = line.split()
        if len(f) == 2:
            w[int(f[0], 16)] = int(f[1], 16)
    return w


def load_overlay(pm, dm, dirp):
    for a, v in rw(dirp + '/pm.words').items():
        pm[a] = v
    for a, v in rw(dirp + '/dm.words').items():
        dm[a] = v


def linear_to_mulaw(s):
    s = max(-32768, min(32767, s))
    sign = 0x80 if s < 0 else 0
    if s < 0:
        s = -s - 1
    s += 0x84
    if s > 0x7FFF:
        s = 0x7FFF
    seg = 0
    t = s >> 5
    while t and seg < 8:
        t >>= 1
        seg += 1
    if seg >= 8:
        return (sign | 0x7F) ^ 0xFF
    return (sign | (seg << 4) | ((s >> (seg + 3)) & 0xF)) ^ 0xFF


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=1200)
    ap.add_argument('--mode', choices=['calling', 'answer'], default='answer')
    ap.add_argument('--freq', type=int, default=2100,
                    help='input tone Hz (2100 = answer tone, 440, 0=silence)')
    ap.add_argument('--tx-out', default='')
    args = ap.parse_args()

    repo = Path(__file__).resolve().parent.parent
    K = str(repo / 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel')
    DIAL = str(repo / 'artifacts/eicon-dsp/dial/0262-dial-fsk-fax.f34-overlay')
    V8 = str(repo / 'artifacts/eicon-dsp/v8/025f-v8.f34-overlay')

    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    # boot kernel
    for a, v in rw(K + '/pm.words').items():
        pm[a] = v
    for a, v in rw(K + '/dm.words').items():
        dm[a] = v
    ADSP.adsp2181_run(cpu, 2000)
    # load DIAL
    load_overlay(pm, dm, DIAL)

    # Program data-pump init (guide §5.4.1)
    ADSP.adsp2181_host_write(cpu, DB + 0x00, 0x00C4)  # GEN_SETUP0
    if args.mode == 'calling':
        ADSP.adsp2181_host_write(cpu, DB + 0x01, 0x048C)  # calling
    else:
        ADSP.adsp2181_host_write(cpu, DB + 0x01, 0x0484)  # answer
    ADSP.adsp2181_host_write(cpu, DB + 0x02, 0x0030)
    ADSP.adsp2181_host_write(cpu, DB + 0x03, 0xF0FD)
    ADSP.adsp2181_host_write(cpu, DB + 0x0E, 0x2000)  # WSTATUS activate
    ADSP.adsp2181_host_write(cpu, DB + 0x0F, 0x0001)  # Norm_H = V.8
    ADSP.adsp2181_host_write(cpu, DB + 0x10, 0x0100)  # Norm_L = V.34+V8

    samples = ([linear_to_mulaw(int(20000 * math.sin(
        2 * math.pi * args.freq * i / 8000))) for i in range(8000)]
        if args.freq > 0 else [0x80] * 8000)

    tx_samples = []
    page = 'DIAL'
    prev = None
    v8_loaded = False
    ADSP.adsp2181_set_pc(cpu, 0x08F0)
    print(f'[sup] mode={args.mode} freq={args.freq}Hz frames={args.frames}')
    print(f'[sup] init: trnprog={dm[TRNPROG]:04x} RSTATUS={dm[RSTATUS]:04x} '
          f'bootpage={dm[BOOTPAGE_NR]:04x} GEN1={dm[DB+1]:04x}')

    for f in range(args.frames):
        # feed RX
        s = samples[f % len(samples)]
        dm[LINE_RX] = s
        dm[LINE_RX2] = s
        # strobe FLAG_IN and run DIAL's real entry (0x08f0 dispatches the
        # full task loop when FLAG_IN is set)
        ADSP.adsp2181_set_flagin(cpu, 1)
        ADSP.adsp2181_run(cpu, 8000)
        ADSP.adsp2181_set_flagin(cpu, 0)
        ADSP.adsp2181_run(cpu, 2000)
        # capture TX (what the DSP left in the line regs / TXD)
        tx_samples.append(dm[LINE_RX] & 0xFF)
        tx_samples.append(dm[LINE_RX2] & 0xFF)

        # host supervisor: watch TrnProgress / bootpage_nr
        tp = dm[TRNPROG]
        bp = dm[BOOTPAGE_NR]
        gen1 = dm[DB + 1]
        now = (page, tp, bp, gen1)
        if now != prev:
            print(f'  f{f:4d} [{page}] trnprog={tp:04x} bootpage={bp:04x} '
                  f'GEN1={gen1:04x}')
            prev = now
        # V.8 load trigger: DIAL consumed the NORM bit (GEN1 bit 7 cleared)
        # and/or set bootpage_nr to a V.8 indicator and trnprogress advanced.
        # Per guide the host loads V.8 when the data-pump signals training
        # start. We model: NORM bit consumed (GEN1 & 0x80 went 0-> was set).
        if page == 'DIAL' and not (gen1 & 0x80) and not v8_loaded:
            print(f'[sup] f{f}: DIAL consumed NORM bit -> loading V.8 page')
            load_overlay(pm, dm, V8)
            ADSP.adsp2181_host_write(cpu, WSTATUS, 0x1000)  # BOOTFINISHED
            ADSP.adsp2181_set_pc(cpu, 0x08F0)
            page = 'V8'
            v8_loaded = True
            print(f'[sup] V.8 loaded; pc reset to 0x08f0')

    print(f'[sup] final page={page} trnprog={dm[TRNPROG]:04x} '
          f'bootpage={dm[BOOTPAGE_NR]:04x}')
    print(f'[sup] TX captured: {len(tx_samples)} samples')
    if args.tx_out:
        out = Path(args.tx_out)
        out.write_bytes(bytes(tx_samples))
        print(f'[sup] TX written to {out}')
    else:
        # show first 32 TX samples
        print(f'[sup] first TX: {[hex(x) for x in tx_samples[:32]]}')


if __name__ == '__main__':
    main()
