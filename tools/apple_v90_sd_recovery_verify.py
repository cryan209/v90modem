#!/usr/bin/env python3
"""Grade the preserved Apple r7 analogue Phase 3, independently by stage.

Needs numpy and the local, gitignored capture (never silently skips it).
The RX and TX taps have different time origins. Their Sd onsets, not their
file lengths, anchor the comparison. The TX signs are used ONLY for grading.
"""
import argparse
import pathlib
import re
import subprocess
import tempfile

import numpy as np


def resample(rx):
    """Offline equivalent of the coupler's causal DC blocker and 5/3 FIR."""
    a = np.exp(-2 * np.pi * 40 / 9600)
    filtered = np.empty(len(rx))
    previous = last = 0.0
    for i, sample in enumerate(rx):
        last = sample - previous + a * last
        previous = sample
        filtered[i] = last
    n = 240
    h = np.sinc((np.arange(n) - (n - 1) / 2) / 5) * np.hamming(n)
    for phase in range(5):
        h[phase::5] /= h[phase::5].sum()
    up = np.zeros(len(rx) * 5)
    up[::5] = filtered
    return np.clip(np.convolve(up, h)[:len(up):3], -32768, 32767).astype('<i2')


def jd_bits(signs, start):
    """V.90 §5.3 GPC, then §8.4.2 differential transport, TX-side reference."""
    reg = 0
    for bit in signs[start - 23:start]:
        reg = ((reg << 1) | int(bit)) & 0x7fffff
    previous = int(signs[start - 1])
    bits = []
    for sign in signs[start:start + 72]:
        scrambled = int(sign) ^ previous
        previous = int(sign)
        bits.append(scrambled ^ ((reg >> 17) & 1) ^ ((reg >> 22) & 1))
        reg = ((reg << 1) | scrambled) & 0x7fffff
    return ''.join(map(str, bits))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=pathlib.Path,
                        default=pathlib.Path('artifacts/apple-v90-sip-r7'))
    parser.add_argument('--receiver', default='./v90_analogue_rx_test')
    # These are measured anchors for r7, not a shared RX/TX time origin.
    parser.add_argument('--rx-start', type=float, default=19.4)
    parser.add_argument('--rx-end', type=float, default=23.8)
    parser.add_argument('--tx-sd', type=int, default=89120)
    args = parser.parse_args()
    rx_path = args.capture / 'call/rx-tap.s16'
    tx_path = args.capture / 'server/live-tx.g711'
    for path in (rx_path, tx_path):
        if not path.is_file():
            parser.error(f'missing preserved capture: {path}')
    rx = resample(np.fromfile(rx_path, dtype='<i2').astype(float))
    rx = rx[round(args.rx_start * 16000):round(args.rx_end * 16000)]
    tx = np.fromfile(tx_path, dtype='u1')
    signs = tx >> 7  # u-law positive sign, independently of the C slicer
    with tempfile.TemporaryDirectory(prefix='apple-v90-grade-') as tmp:
        source = pathlib.Path(tmp) / 'line.s16'
        dump = pathlib.Path(tmp) / 'symbols.raw'
        rx.tofile(source)
        run = subprocess.run([args.receiver, '--line-trace', str(source), '78', str(dump)],
                             capture_output=True, text=True, check=True)
        print(run.stdout, end='')
        out = np.fromfile(dump, dtype=[('at', '<u8'), ('y', '<f4'), ('stage', 'u1')])
    # Measure reversal independently: a single-cycle coherent projection,
    # rather than the C detector's two-cycle magnitude minimum.
    reversal = int(re.search(r'measured reversal (\d+)', run.stdout)[1])
    positions = np.arange(len(rx))
    mixed = rx * np.exp(-2j * np.pi * positions / 12)
    reference = mixed[reversal - 200:reversal - 100].mean()
    projected = np.real(np.convolve(mixed, np.ones(12) / 12, mode='valid')
                        * reference.conjugate())
    crossings = np.where((projected[:-1] >= 0) & (projected[1:] < 0))[0]
    measured = min(crossings + 6, key=lambda at: abs(at - reversal))
    assert abs(measured - reversal) <= 4, (measured, reversal)
    print(f'PASS reversal: tracker {reversal}, independent projection {measured} (16 kHz samples)')
    # Sd/S-bar-d take 432T on TX. They cannot train a loop equaliser.
    tx_trn = args.tx_sd + 432
    trained = out[out['stage'] == 3]  # V90A_RX_TRN1D
    errors, raw_errors, lags = [], [], []
    # Short windows avoid folding the two free-running clocks into one fit.
    # These are high-entropy TRN1d signs, never repeated Jd as a channel score.
    for first in range(2500, 18500, 1000):
        block = trained[first:first + 1000]
        assert len(block) == 1000
        decided = block['y'] >= 0
        raw = rx[block['at'].astype(int)] >= 0
        def best_error(candidate):
            return min((min(np.mean(candidate != signs[tx_trn + first + lag:
                                                       tx_trn + first + lag + 1000]),
                            np.mean(candidate == signs[tx_trn + first + lag:
                                                       tx_trn + first + lag + 1000])), lag)
                       for lag in range(-260, 261))
        error, lag = best_error(decided)
        raw_error, raw_lag = best_error(raw)
        assert abs(lag) < 260 and abs(raw_lag) < 260, 'lag sweep clipped'
        errors.append(error); raw_errors.append(raw_error); lags.append(lag)
    assert max(errors) < 0.03 and np.mean(errors) < np.mean(raw_errors) / 2
    print(f'PASS CMA: 16000 TX-graded TRN1d signs, mean {100*np.mean(errors):.2f}% errors, '
          f'worst window {100*max(errors):.2f}%, raw mean {100*np.mean(raw_errors):.2f}%, '
          f'lags {min(lags)}..{max(lags)} (interior)')
    # TX TRN1d is 20004T on this preserved call. Decode its first Table 13
    # independently and require every accepted RX frame to have those fields.
    expected = jd_bits(signs, tx_trn + 20004)
    frames = re.findall(r'Jd bits: ([01]{72})', run.stdout)
    assert len(frames) >= 2 and all(frame == expected for frame in frames)
    ends = list(map(int, re.findall(r'CRC-valid Jd at input sample (\d+)', run.stdout)))
    assert all((end - ends[0]) % 144 == 0 for end in ends)
    print(f'PASS Jd: {len(frames)} CRC-valid frames match independent TX bits, '
          '72-symbol frame / six-slot grid agrees across repetitions')


if __name__ == '__main__':
    main()
