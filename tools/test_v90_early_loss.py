#!/usr/bin/env python3
"""V.90 9.5.1.1/9.6: detect sustained loss even before baseline learning.

Synthetic fault stimuli use a retained acquisition prefix followed by
deterministic noise. They are not hardware recordings or interop evidence.
"""
import argparse
import os
from pathlib import Path
import random
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay', default='./v90_upstream_replay')
    args = parser.parse_args()
    clean = Path('reference-data/artifacts/apple-direct-v90-b1/late-e-rx.ulaw').read_bytes()
    rng = random.Random(9511)
    noise = bytes(rng.randrange(64, 256) for _ in range(32000))
    cases = [
        ('clean', clean, 0),
        ('early-loss', clean[:6400] + noise, 1),
        ('late-loss', clean[:16000] + noise, 1),
        ('loss-then-clean', clean[:6400] + noise + clean[38400:], 1),
        ('short-burst', clean[:6400] + noise[:320], 0),
    ]
    with tempfile.TemporaryDirectory(prefix='v90-early-loss-') as scratch:
        for name, data, lost in cases:
            path = Path(scratch) / (name + '.ulaw')
            path.write_bytes(data)
            result = subprocess.run([
                args.replay, str(path), 'ulaw', '3200', '9600',
                '0.2', '0.2', 'cp-prime',
            ], env=dict(os.environ, V90_REPLAY_QUIET='1'),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                timeout=30)
            if (result.returncode or 'replay: acquired=1\n' not in result.stdout
                    or f'replay: carrier_lost={lost}\n' not in result.stdout):
                print(result.stdout)
                raise SystemExit(f'{name}: expected acquired receiver, carrier_lost={lost}')
            if name == 'early-loss' and 'baseline ready=0' not in result.stdout:
                raise SystemExit('early-loss: did not exercise unfinished baseline')
            if lost and 'replay: carrier loss acknowledged' not in result.stdout:
                raise SystemExit(f'{name}: loss was not clearable through the public API')
            print(f'PASS {name}: acquired=1 carrier_lost={lost}')


if __name__ == '__main__':
    main()
