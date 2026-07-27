#!/usr/bin/env python3
"""Run the V.8 overlay standalone and capture its TX output.

V.8 (bootpage 6) is loaded directly, bypassing the DIAL->V.8 transition
(which needs the kernel task dispatcher / MIPS supervisor). V.8 has the
same structure as DIAL: a state dispatcher (PM 0x1FF4) and a line handler
(PM 0x2000) that reads the line RX registers DM 0x3F08/0x3F09.

We feed μ-law into 0x3F08/0x3F09 and call V.8's handlers each frame, then
capture what V.8 writes back to the line registers (its TX output).
"""
import argparse, ctypes, math, wave
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
ADSP.adsp2181_set_callbacks.argtypes = [ctypes.c_void_p] * 4
RX_CB = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p, ctypes.c_int)
TX_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int32)
TIM_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)

DB = 0x3EE0
TRNPROG = 0x3FAD
BOOTPAGE = 0x3FB0
LINE = 0x3F09
LINE2 = 0x3F08


def rw(p):
    w = {}
    for line in Path(p).read_text().splitlines():
        f = line.split()
        if len(f) == 2:
            w[int(f[0], 16)] = int(f[1], 16)
    return w


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


def mulaw_to_linear(u):
    u = ~u & 0xFF
    sign = u & 0x80
    seg = (u >> 4) & 7
    mant = u & 0xF
    s = ((mant << 3) + 0x84) << seg
    s -= 0x84
    return -s if sign else s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=8000,
                    help='frames = samples (8000 = 1 second)')
    ap.add_argument('--freq', type=int, default=2100)
    ap.add_argument('--amp', type=int, default=20000)
    ap.add_argument('--tx-out', default='')
    ap.add_argument('--mode', choices=['calling', 'answer'], default='answer')
    args = ap.parse_args()

    repo = Path(__file__).resolve().parent.parent
    K = str(repo / 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel')
    V8 = str(repo / 'artifacts/eicon-dsp/v8/025f-v8.f34-overlay')

    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for a, v in rw(K + '/pm.words').items():
        pm[a] = v
    for a, v in rw(K + '/dm.words').items():
        dm[a] = v
    ADSP.adsp2181_run(cpu, 2000)
    for a, v in rw(V8 + '/pm.words').items():
        pm[a] = v
    for a, v in rw(V8 + '/dm.words').items():
        dm[a] = v

    # V.8 init: GEN_SETUP for answer mode, Norm = V.8
    ADSP.adsp2181_host_write(cpu, DB + 0x00, 0x00C4)
    if args.mode == 'calling':
        ADSP.adsp2181_host_write(cpu, DB + 0x01, 0x048C)
    else:
        ADSP.adsp2181_host_write(cpu, DB + 0x01, 0x068C)
    ADSP.adsp2181_host_write(cpu, DB + 0x02, 0x0030)
    ADSP.adsp2181_host_write(cpu, DB + 0x0E, 0x2000)
    ADSP.adsp2181_host_write(cpu, DB + 0x0F, 0x0001)  # Norm_H V.8
    ADSP.adsp2181_host_write(cpu, DB + 0x10, 0x0100)  # Norm_L

    samples = ([linear_to_mulaw(int(args.amp * math.sin(
        2 * math.pi * args.freq * i / 8000))) for i in range(8000)]
        if args.freq > 0 else [0x80] * 8000)

    # SPORT0 callbacks: capture TX0 writes, feed RX0 reads
    tx_captures = []
    rx_feed = [samples]

    def rx_cb(c, p):
        if p == 0:
            f = len(tx_captures)
            return rx_feed[0][f % len(rx_feed[0])]
        return 0

    def tx_cb(c, p, v):
        if p == 0:
            tx_captures.append(v & 0xFFFF)

    def tim_cb(c, e):
        pass

    ADSP.adsp2181_set_callbacks(cpu, RX_CB(rx_cb), TX_CB(tx_cb), TIM_CB(tim_cb))

    print(f'[v8] mode={args.mode} freq={args.freq}Hz frames={args.frames}')
    print(f'[v8] init: trnprog={dm[TRNPROG]:04x} bootpage={dm[BOOTPAGE]:04x} '
          f'GEN1={dm[DB+1]:04x}')

    tx = []
    prev = None
    for f in range(args.frames):
        dm[LINE] = samples[f % len(samples)]
        dm[LINE2] = samples[f % len(samples)]
        ADSP.adsp2181_call(cpu, 0x2000, 0x02A8)   # V.8 line handler
        ADSP.adsp2181_run(cpu, 8000)
        ADSP.adsp2181_call(cpu, 0x1FF4, 0x02A8)   # V.8 state dispatcher
        ADSP.adsp2181_run(cpu, 8000)
        tx.append(dm[LINE] & 0xFF)
        now = (dm[TRNPROG], dm[BOOTPAGE], dm[DB + 1])
        if now != prev:
            print(f'  f{f:5d}: trnprog={now[0]:04x} bootpage={now[1]:04x} '
                  f'GEN1={now[2]:04x}')
            prev = now

    print(f'[v8] final: trnprog={dm[TRNPROG]:04x} bootpage={dm[BOOTPAGE]:04x}')
    print(f'[v8] DM-line TX captured: {len(tx)} samples')
    print(f'[v8] SPORT0 TX0 writes captured: {len(tx_captures)} samples')
    use_tx = tx_captures if len(tx_captures) > len(tx) // 2 else tx

    # convert TX μ-law to linear and write a wav
    out = Path(args.tx_out) if args.tx_out else Path('/tmp/v8_tx.wav')
    linear = [mulaw_to_linear(u) for u in use_tx]
    with wave.open(str(out), 'w') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(8000)
        w.writeframes(b''.join(int(s).to_bytes(2, 'little', signed=True)
                               for s in linear))
    print(f'[v8] TX wav written to {out} (source: {"SPORT0" if use_tx is tx_captures else "DM-line"})')
    nz = sum(1 for u in use_tx if u != 0x80)
    print(f'[v8] non-idle TX samples: {nz}/{len(use_tx)} ({100*nz//max(1,len(use_tx))}%)')


if __name__ == '__main__':
    main()
