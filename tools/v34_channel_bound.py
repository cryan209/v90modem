#!/usr/bin/env python3
"""How good could ANY linear receiver be on this recording?

The question "is the rate ceiling the wire or our receiver?" cannot be
answered from inside the receiver, because every metric it has rests on its
own decisions.  This answers it from outside: given a recorded G.711 tap and
the symbols the receiver decided, fit the best possible fractionally-spaced
least-squares equalizer -- non-causal, as long as you like, solved exactly
rather than adapted -- and report the residual.  That is an upper bound on
every linear receiver, ours included.

If our receiver is within a dB of it, there is nothing left to win at our
end and the ceiling is the wire.  If it is far below, the receiver is the
problem.  Point it at the sender's own pre-resampler tap (the d-modem rig
writes /tmp/dm_from_dsp_9600.raw) and the same number says whether the wire
or the far transmitter is where the noise is made.

The decided symbols are the truth only where the call is actually decoding;
check `distance to grid` first and pick a window inside a healthy stretch.

  v34_channel_bound.py <tap> <frames.answer> [--baud 3000] [--carrier 1800]
                       [--rate 8000] [--alaw] [--raw16] [--from-symbol 2000]
                       [--symbols 1024] [--taps 41,81,161]
"""
import argparse
import sys

import numpy as np


def ulaw2lin(b):
    # int32 first: the shifts below overflow a uint8 silently, which reads as
    # a plausible-looking waveform that will not fit any equalizer.
    b = (~b.astype(np.int32)) & 0xFF
    s = b & 0x80
    e = (b >> 4) & 7
    m = b & 0xF
    v = (((m << 1) + 33) << e) - 33
    return np.where(s != 0, -v, v).astype(float)


def alaw2lin(b):
    b = (b.astype(np.int32) ^ 0x55)
    s = b & 0x80
    e = (b >> 4) & 7
    m = b & 0xF
    v = np.where(e == 0, (m << 4) + 8, ((m << 4) + 0x108) << (e - 1))
    return np.where(s != 0, -v, v).astype(float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tap")
    ap.add_argument("frames")
    ap.add_argument("--baud", type=float, default=3000.0)
    ap.add_argument("--carrier", type=float, default=1800.0)
    ap.add_argument("--rate", type=float, default=8000.0)
    ap.add_argument("--alaw", action="store_true")
    ap.add_argument("--raw16", action="store_true",
                    help="tap is linear int16, not G.711 (the 9600 Hz DSP tap)")
    ap.add_argument("--from-symbol", type=int, default=2000)
    ap.add_argument("--symbols", type=int, default=1024)
    ap.add_argument("--taps", default="41,81,161")
    a = ap.parse_args()

    if a.raw16:
        x = np.fromfile(a.tap, dtype="<i2").astype(float)
    else:
        raw = np.frombuffer(open(a.tap, "rb").read(), dtype=np.uint8)
        x = alaw2lin(raw) if a.alaw else ulaw2lin(raw)
    d = np.fromfile(a.frames, dtype="<i2").astype(float)/128.0
    z = d[0::2] + 1j*d[1::2]
    # 9.x puts every constellation point on odd integers, so the receiver's
    # decision is the nearest one.  Inside a healthy stretch it is the truth.
    sym = (2*np.floor(z.real/2) + 1) + 1j*(2*np.floor(z.imag/2) + 1)
    sym = sym[a.from_symbol:a.from_symbol + a.symbols]
    if len(sym) < a.symbols:
        sys.exit("not enough symbols in %s" % a.frames)

    n = np.arange(len(x))
    bb = x*np.exp(-2j*np.pi*a.carrier*n/a.rate)
    K = 161
    t = np.arange(K) - K//2
    h = np.sinc(2*(a.baud*0.6)/a.rate*t)*np.hamming(K)
    bb = np.convolve(bb, h/h.sum(), "same")

    L = 12
    win = np.hamming(2*L + 1)

    def interp(pos):
        out = np.zeros(len(pos), complex)
        i0 = np.floor(pos).astype(int)
        for k in range(-L, L + 1):
            idx = np.clip(i0 + k, 0, len(bb) - 1)
            out += bb[idx]*np.sinc(pos - (i0 + k))*win[k + L]
        return out

    taps = [int(v) for v in a.taps.split(",")]

    def fit(delay, nt):
        pos = delay + (np.arange(len(sym)*2 + nt)/(2*a.baud))*a.rate
        if pos[-1] >= len(bb) - 20 or pos[0] < 20:
            return None
        y = interp(pos)
        X = np.stack([y[2*i:2*i + nt] for i in range(len(sym))], 0)
        c, *_ = np.linalg.lstsq(X, sym, rcond=None)
        return float(np.mean(np.abs(sym - X@c)**2))

    nt0 = taps[0]
    best = (1e18, None)
    for delay in range(200, len(x) - int(len(sym)*2.5*a.rate/a.baud), 80):
        r = fit(delay, nt0)
        if r is not None and r < best[0]:
            best = (r, delay)
    if best[1] is None:
        sys.exit("no alignment found -- is this the right tap/baud/carrier?")
    for delay in range(best[1] - 90, best[1] + 90):
        r = fit(delay, nt0)
        if r is not None and r < best[0]:
            best = (r, delay)
    power = float(np.mean(np.abs(sym)**2))
    print("%s: aligned at sample %d, %d symbols from %d"
          % (a.tap, best[1], len(sym), a.from_symbol))
    for nt in taps:
        r = fit(best[1], nt)
        if r is None:
            continue
        print("  best possible linear receiver, %3d T/2 taps: %5.1f dB"
              % (nt, 10*np.log10(power/r)))
    # The bias from fitting nt complex taps to len(sym) observations, so a
    # short window cannot be mistaken for a better channel.
    print("  (overfitting bias at %d taps / %d symbols: %.2f dB)"
          % (taps[-1], len(sym),
             -10*np.log10(max(1e-9, 1.0 - taps[-1]/float(len(sym))))))


if __name__ == "__main__":
    main()
