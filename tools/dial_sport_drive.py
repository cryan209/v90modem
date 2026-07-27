#!/usr/bin/env python3
"""Drive DIAL with a μ-law T1/E1 stream on SPORT0.

Boots the PRI kernel to IDLE, overlays the DIAL bootpage, programs the
data-pump SPORT0 multichannel control registers (guide §5.3.1: Sp0CntrlReg
@ 0x3F50, word enables @ 0x3F51-0x3F54), then feeds a μ-law sample stream
as 32 timeslots per 8 kHz frame via the SPORT0 RX callback, strobing the
SPORT0_RX interrupt once per frame.

Watches the kernel's PCM sink (DM 0x2E52), DIAL's line registers
(DM 0x3F08/0x3F09) and DIAL state (DM 0x3FB0) to see whether DIAL reacts.

The kernel's SPORT0 RX ISR (PM 0x0072) reads RX0 and stores it to DM 0x2E52,
walking the 32 timeslots via the DM 0x2E50 countdown. We model the
multichannel stream at the sample level: each SPORT0_RX strobe delivers one
timeslot word; after 32 strobes the frame repeats.

Usage:
    python3 tools/dial_sport_drive.py [--frames N] [--channel C] [--tone]
"""
import argparse, ctypes, struct, sys
from pathlib import Path

ADSP = ctypes.CDLL(str(Path(__file__).resolve().parent /
                       "adsp2181emu" / "libadsp2181.dylib"))
ADSP.adsp2181_create.restype = ctypes.c_void_p
for n, a in [('reset', [ctypes.c_void_p]), ('pm', [ctypes.c_void_p]),
             ('dm', [ctypes.c_void_p]), ('run', [ctypes.c_void_p, ctypes.c_int]),
             ('pc', [ctypes.c_void_p]), ('idle', [ctypes.c_void_p]),
             ('set_pc', [ctypes.c_void_p, ctypes.c_uint16]),
             ('set_callbacks', [ctypes.c_void_p] * 4),
             ('set_irq', [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]),
             ('imask', [ctypes.c_void_p]),
             ('set_imask', [ctypes.c_void_p, ctypes.c_uint16]),
             ('host_write', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]),
             ('host_read', [ctypes.c_void_p])]:
    getattr(ADSP, 'adsp2181_' + n).argtypes = a
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int
ADSP.adsp2181_imask.restype = ctypes.c_uint16
ADSP.adsp2181_host_read.restype = ctypes.c_uint16

SPORT0_RX = 3
SPORT0_TX = 4


def rw(p):
    w = {}
    for line in Path(p).read_text().splitlines():
        f = line.split()
        if len(f) == 2:
            w[int(f[0], 16)] = int(f[1], 16)
    return w


# --- μ-law <-> linear (G.711) ---
MULAW_BIAS = 0x84
MULAW_SEG = 8


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
    while t and seg < MULAW_SEG:
        t >>= 1
        seg += 1
    if seg >= MULAW_SEG:
        return sign | 0x7F
    mantissa = (s >> (seg + 3)) & 0xF
    return sign | (seg << 4) | mantissa ^ 0xFF if False else (sign | (seg << 4) | mantissa) ^ 0xFF


def mulaw_to_linear(u):
    u = ~u & 0xFF
    sign = u & 0x80
    seg = (u >> 4) & 7
    mant = u & 0xF
    s = ((mant << 3) + MULAW_BIAS) << seg
    s -= MULAW_BIAS
    return -s if sign else s


# --- SPORT0 RX callback: deliver 32 timeslots per frame ---
RX_CB = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p, ctypes.c_int)
TX_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int32)
TIM_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)

g_frame = 0
g_slot = 0
g_channel = 0
g_samples = None
g_tx = []


def rx_cb(cpu, port):
    global g_slot
    if port != 0:
        return 0
    # deliver the next timeslot; our channel gets a tone, others idle
    if g_slot == g_channel and g_samples is not None:
        v = g_samples[g_frame % len(g_samples)]
    else:
        v = 0xFF  # idle μ-law code (silence)
    g_slot = (g_slot + 1) % 32
    return v


def tx_cb(cpu, port, val):
    if port == 0:
        g_tx.append(val & 0xFF)


def tim_cb(cpu, en):
    pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--frames', type=int, default=200)
    ap.add_argument('--channel', type=int, default=0, help='T1/E1 timeslot 0..31')
    ap.add_argument('--freq', type=int, default=440, help='tone Hz (0=silence)')
    ap.add_argument('--amplitude', type=int, default=20000)
    ap.add_argument('--dial-state', type=lambda x: int(x, 0), default=None,
                    help='pre-poke DIAL state 0x3FB0')
    ap.add_argument('--gen-setup', action='store_true',
                    help='program data-pump GEN_SETUP + SPORT0 multichannel regs')
    args = ap.parse_args()

    repo = Path(__file__).resolve().parent.parent
    kernel = str(repo / 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel')
    dial = str(repo / 'artifacts/eicon-dsp/dial/0262-dial-fsk-fax.f34-overlay')

    global g_channel, g_samples
    g_channel = args.channel
    # 8000 samples/sec; build 1 sec of tone
    if args.freq > 0:
        g_samples = [linear_to_mulaw(int(args.amplitude * math_sin(2 * 3.14159265 * args.freq * i / 8000)))
                     for i in range(8000)]
    else:
        g_samples = [0xFF] * 8000

    cpu = ADSP.adsp2181_create()
    ADSP.adsp2181_reset(cpu)
    pm = ADSP.adsp2181_pm(cpu)
    dm = ADSP.adsp2181_dm(cpu)
    for a, v in rw(kernel + '/pm.words').items():
        pm[a] = v
    for a, v in rw(kernel + '/dm.words').items():
        dm[a] = v
    ADSP.adsp2181_run(cpu, 2000)  # boot kernel to IDLE
    for a, v in rw(dial + '/pm.words').items():
        pm[a] = v
    for a, v in rw(dial + '/dm.words').items():
        dm[a] = v

    # attach SPORT callbacks
    rx = RX_CB(rx_cb)
    tx = TX_CB(tx_cb)
    tim = TIM_CB(tim_cb)
    ADSP.adsp2181_set_callbacks(cpu, rx, tx, tim)

    print(f'[drive] kernel+DIAL loaded; pc={ADSP.adsp2181_pc(cpu):04x} '
          f'imask=0x{ADSP.adsp2181_imask(cpu):03x}')
    print(f'[drive] channel={args.channel} freq={args.freq}Hz frames={args.frames}')

    # Program the data-pump SPORT0 multichannel control registers (guide §5.3.1).
    # Sp0CntrlReg @0x3F50: enable multichannel, companding, etc. Use the
    # ADDSP default-ish value: multichannel on, μ-law, 8-bit, internally
    # generated frame sync. The kernel ISR already runs the timeslot walk;
    # these shadow registers are what the data-pump reads to configure SPORT0.
    if args.gen_setup:
        # Sp0CntrlReg: MCM (multichannel) + μ-law companding + word len 8
        # Bits per ADSP-2181 STCTL0/SRCTL0: we set a plausible enable word.
        ADSP.adsp2181_host_write(cpu, 0x3F50, 0x000F)
        # enable receive/transmit on all 32 channels (word enables)
        ADSP.adsp2181_host_write(cpu, 0x3F51, 0xFFFF)  # Sp0MCRecL (slots 0-15)
        ADSP.adsp2181_host_write(cpu, 0x3F52, 0xFFFF)  # Sp0MCRecM (slots 16-31)
        ADSP.adsp2181_host_write(cpu, 0x3F53, 0xFFFF)  # Sp0MCTXL
        ADSP.adsp2181_host_write(cpu, 0x3F54, 0xFFFF)  # Sp0MCTXM
        # GEN_SETUP0/1/2 + WSTATUS activate (guide §5.4.1 Table 12)
        ADSP.adsp2181_host_write(cpu, 0x3EE0, 0x00C4)  # GEN_SETUP0
        ADSP.adsp2181_host_write(cpu, 0x3EE1, 0x0040)  # GEN_SETUP1
        ADSP.adsp2181_host_write(cpu, 0x3EE2, 0x0000)  # GEN_SETUP2
        ADSP.adsp2181_host_write(cpu, 0x3EEE, 0x2000)  # WSTATUS.activate (bit D)
        print('[drive] programmed SPORT0 multichannel + GEN_SETUP')

    if args.dial_state is not None:
        dm[0x3FB0] = args.dial_state & 0xFFFF
        print(f'[drive] poked DIAL state 0x3FB0=0x{args.dial_state:04x}')

    # Feed frames: each frame = 32 timeslots, each strobed by SPORT0_RX.
    print('[drive] feeding frames...')
    prev_2e52 = prev_3f08 = prev_3f09 = prev_3fb0 = None
    for f in range(args.frames):
        for slot in range(32):
            ADSP.adsp2181_set_irq(cpu, SPORT0_RX, 1)
            ADSP.adsp2181_set_irq(cpu, SPORT0_RX, 0)
            ADSP.adsp2181_run(cpu, 2000)
        # report changes
        v2e52 = dm[0x2E52]
        v3f08 = dm[0x3F08]
        v3f09 = dm[0x3F09]
        v3fb0 = dm[0x3FB0]
        if (v2e52 != prev_2e52 or v3f08 != prev_3f08 or
                v3f09 != prev_3f09 or v3fb0 != prev_3fb0):
            print(f'  frame {f:4d}: 2E52={v2e52:04x} 3F08={v3f08:04x} '
                  f'3F09={v3f09:04x} 3FB0={v3fb0:04x} '
                  f'(pc={ADSP.adsp2181_pc(cpu):04x})')
            prev_2e52, prev_3f08, prev_3f09, prev_3fb0 = v2e52, v3f08, v3f09, v3fb0
    print(f'[drive] done. TX words captured: {len(g_tx)}')
    if g_tx:
        print(f'[drive] first TX: {[hex(x) for x in g_tx[:16]]}')


def math_sin(x):
    import math
    return math.sin(x)


if __name__ == '__main__':
    main()
