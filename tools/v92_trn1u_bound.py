#!/usr/bin/env python3
"""How well could ANY linear receiver recover V.92 TRN1u from this recording?

V.92 8.5.7: TRN1u's signs are the GPA scrambler (6.3, taps 5/23) fed binary
ones, "initialized to zero prior to the transmission of TRN1u", output 0 ->
+L_U and 1 -> -L_U.  So the whole TRN1u sign sequence is KNOWN from its first
symbol -- no decisions, no descrambler, no error multiplication.  Given a
digital-side G.711 receive tap and a rough TRN1u start, this sweeps the start
offset, fits a symbol-spaced least-squares equalizer to that known sequence on
the first two thirds of the window and reports the held-out residual and sign
error.  It bounds every linear T-spaced receiver, ours included.

  v92_trn1u_bound.py <live-rx.g711> <approx-trn1u-sample>
                     [--alaw] [--len 1800] [--taps 1,5,11,21,41] [--span 60]

Measured on artifacts/apple-v92-sip-r4 (Apple analogue modem on a 2-wire loop
-> VG224 -> SIP -> our digital side), rough start 85323 where v92_p3_rx
entered TRN1u: 1 tap 14% sign error at offset -31 (i.e. TRN1u really began
~30 symbols earlier than the receiver thought); 21 taps 0.7%; 41 taps 0.2%;
21 taps over 600 symbols 0.0% -- shorter windows fit better, which is the
163 ppm clock offset between the two codecs.  See docs/v92_p3_rx_line_plan.md.

Needs numpy (`python3 -m venv` + `pip install numpy`).
"""
import argparse
import numpy as np


def ulaw2lin(b):
    c = (~b.astype(np.int32)) & 0xFF
    s, e, m = c & 0x80, (c >> 4) & 7, c & 15
    v = (((m << 3) + 0x84) << e) - 0x84
    return np.where(s, -v, v).astype(float)


def alaw2lin(b):
    c = b.astype(np.int32) ^ 0x55
    s, e, m = c & 0x80, (c >> 4) & 7, c & 15
    v = np.where(e == 0, (m << 4) + 8, ((m << 4) + 0x108) << np.maximum(e - 1, 0))
    return np.where(s, v, -v).astype(float)


def trn1u_reference(n):
    reg = [0] * 23
    out = np.empty(n)
    for i in range(n):
        b = 1 ^ reg[4] ^ reg[22]
        reg = [b] + reg[:-1]
        out[i] = 1.0 if b == 0 else -1.0
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tap")
    ap.add_argument("start", type=int)
    ap.add_argument("--alaw", action="store_true")
    ap.add_argument("--len", type=int, default=1800)
    ap.add_argument("--taps", default="1,5,11,21,41")
    ap.add_argument("--span", type=int, default=60)
    a = ap.parse_args()

    raw = np.fromfile(a.tap, dtype=np.uint8)
    x = alaw2lin(raw) if a.alaw else ulaw2lin(raw)
    ref = trn1u_reference(a.len)
    train = a.len * 2 // 3

    for nt in [int(t) for t in a.taps.split(",")]:
        half = nt // 2
        results = []
        for d in range(-a.span, a.span + 1):
            s = a.start + d
            if s - half < 0 or s + a.len + half >= len(x):
                continue
            A = np.array([x[s + n - half:s + n + half + 1] for n in range(a.len)])
            w, *_ = np.linalg.lstsq(A[:train], ref[:train], rcond=None)
            z = A[train:] @ w
            r2 = 1 - np.mean((ref[train:] - z) ** 2) / np.mean(ref[train:] ** 2)
            ser = np.mean(np.sign(z) != ref[train:])
            raw_err = np.mean(np.sign(A[:, half]) != ref)
            results.append((r2, d, ser, raw_err))
        results.sort(reverse=True)
        r2, d, ser, raw_err = results[0]
        snr = 10 * np.log10(max(r2, 1e-9) / max(1 - r2, 1e-9))
        print("taps %2d: offset %+d  R2 %.3f  SNR %.1f dB  held-out sign err %.4f"
              "  raw sign err %.3f" % (nt, d, r2, snr, ser, raw_err))


if __name__ == "__main__":
    main()
