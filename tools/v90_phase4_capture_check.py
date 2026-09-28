#!/usr/bin/env python3
"""Locate CRC-valid V.90 CP frames and measure TX echo on recorded DS0 taps.

All reported times are offsets in the named RX file, NOT engine wall times or
TX offsets. Demodulation is the independent T/2 CMA frontend in
v90_phase4_upstream_grade. V.34 10.1.3.3 uses clockwise differential rotation;
V.90 8.5.2 uses GPA and Table 14 framing. A CRC-valid frame, not a majority
zero-bit match, anchors the report. This is an offline diagnostic only.

Optional echo fitting searches TX against the first half of the requested
window, fits a short FIR there, and reports power removal on the second half.
The two taps need not share an origin: the reported offset includes that
unknown difference and MUST NOT be described as a physical round-trip delay.
No G.711 files are modified. NumPy is the only external dependency.
"""
import argparse
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from v90_phase4_upstream_grade import FS, ulaw_lin, demod
from v90_cp_frame import scan


def cp_report(x, start, end, baud, carrier, label):
    s = demod(x, start, end, baud, carrier, skip=0)
    angles = np.angle(s[1:] * np.conj(s[:-1]))
    steps = np.rint(angles / (np.pi / 2)).astype(int)
    q = (-steps) % 4
    b = np.empty(2 * len(q), dtype=np.uint8)
    b[::2], b[1::2] = q & 1, q >> 1
    decoded = b.copy()
    decoded[23:] = b[23:] ^ b[18:-5] ^ b[:-23]
    frames = scan(decoded.tolist())
    # 41 T/2 taps: the central input is 10 symbols after the window start.
    # Differential decision zero belongs to output symbol one.
    origin = start + 11 / baud
    valid = [f for f in frames if f['crc_ok']]
    print(f'{label}: {len(valid)} CRC-valid frames')
    for f in frames:
        print(f"  RX {origin + f['off']/(2*baud):.6f}s {f['kind']} "
              f"bits={f['flen']} CRC={'OK' if f['crc_ok'] else 'BAD'}")
    if not valid:
        print('  No CRC anchor; do not infer protocol identity from ones scores.')
        return
    anchor = valid[0]['off'] // 2
    width = round(baud * .125)
    for k in range(anchor, len(q) - width + 1, width):
        a = angles[k:k+width]
        e = np.degrees(a - np.rint(a/(np.pi/2))*(np.pi/2))
        h = np.bincount(q[k:k+width], minlength=4) / width
        print(f'  RX {origin+k/baud:.6f}s diff_rms_deg={np.sqrt(np.mean(e*e)):.2f}'
              f' GPA_ones={decoded[2*k:2*(k+width)].mean():.4f}'
              f' max_dibit_fraction={h.max():.3f}')


def echo_fit(rx, tx, start, end, taps):
    a, b = round(start * FS), round(end * FS)
    middle = (a + b) // 2
    train = rx[a:middle]
    nfft = 1 << (len(tx) + len(train) - 1).bit_length()
    corr = np.fft.irfft(np.fft.rfft(tx, nfft)
                       * np.conj(np.fft.rfft(train, nfft)), nfft)
    power = np.r_[0., np.cumsum(tx*tx)]
    power = power[len(train):] - power[:-len(train)]
    rho = corr[:len(power)] / np.sqrt(np.maximum(power, 1) * np.dot(train, train))
    # Leave enough TX history for both training and held-out evaluation.
    half = taps // 2
    lo, hi = half, len(tx) - (b-a) - half
    if hi <= lo:
        raise ValueError('TX tap too short for an independent echo test')
    at = lo + np.argmax(np.abs(rho[lo:hi]))
    offset = a - at
    matrix = sliding_window_view(tx, taps)[at-half:at-half+(b-a)]
    split = middle - a
    h = np.linalg.lstsq(matrix[:split], train, rcond=None)[0]
    residual = rx[a:b] - matrix @ h
    print(f'echo: RX-minus-TX file offset={offset} samples ({offset/FS:.6f}s), '
          f'train correlation={rho[at]:.4f}; not a physical delay measurement')
    for name, sl in [('train', slice(0, split)), ('held-out', slice(split, None))]:
        original = rx[a:b][sl]
        removed = 1 - np.mean(residual[sl]**2) / np.mean(original**2)
        print(f'  {name}: power removed={100*removed:.3f}%')
    return offset, h


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('rx')
    p.add_argument('start', type=float)
    p.add_argument('end', type=float)
    p.add_argument('--baud', type=float, default=3200)
    p.add_argument('--carrier', type=float)
    p.add_argument('--tx')
    p.add_argument('--echo-window', nargs=2, type=float, metavar=('START', 'END'))
    p.add_argument('--echo-taps', type=int, default=129)
    args = p.parse_args()
    x = ulaw_lin(np.fromfile(args.rx, dtype=np.uint8))
    if not 0 <= args.start < args.end <= len(x)/FS:
        p.error('demodulation window must be inside RX tap')
    cp_report(x, args.start, args.end, args.baud,
              args.carrier or args.baud*8/14, 'raw')
    if args.tx:
        if not args.echo_window or args.echo_taps < 1 or args.echo_taps % 2 != 1:
            p.error('--tx requires --echo-window and an odd positive --echo-taps')
        begin, end = args.echo_window
        if not 0 <= begin < end <= len(x)/FS:
            p.error('echo window must be inside RX tap')
        tx = ulaw_lin(np.fromfile(args.tx, dtype=np.uint8))
        echo_fit(x, tx, begin, end, args.echo_taps)


if __name__ == '__main__':
    main()
