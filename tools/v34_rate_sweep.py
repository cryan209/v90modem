#!/usr/bin/env python3
"""Measure forced V.34 rates against training estimates and actual PRBS errors.

V.34 sections 11.4 and 12.5.2. A failed forced-rate row is a measurement,
not a runner failure. Compare identical rows from separately built binaries.
"""
import argparse
import concurrent.futures
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=ROOT / 'v34_duplex_test')
    parser.add_argument('--initial-rate', type=int, default=33600)
    parser.add_argument('--rates', default='24000,26400,28800,31200,33600')
    parser.add_argument('--delays', default='0,1,7,23')
    parser.add_argument('--noise', default='clean,40,36,30')
    parser.add_argument('--laws', default='ulaw,alaw')
    parser.add_argument('--bits', type=int, default=100000)
    parser.add_argument('--blocks', type=int, default=2000)
    parser.add_argument('-j', type=int, default=4)
    args = parser.parse_args()
    binary = args.binary.resolve()
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    rows = itertools.product(map(int, args.rates.split(',')),
                             args.laws.split(','),
                             map(int, args.delays.split(',')),
                             args.noise.split(','))

    def run(row):
        rate, law, delay, noise = row
        # Isolate every run as tools/run_tests.py does. Never inherit another
        # diagnostic's delay, clock, echo, renegotiation or payload settings.
        env = {k: v for k, v in os.environ.items()
               if not k.startswith('V34_DUPLEX_')}
        limits = f'0,{rate},0,{rate}'
        env.update(V34_DUPLEX_LOG='1', V34_DUPLEX_CALL_LIMITS=limits,
                   V34_DUPLEX_ANSWER_LIMITS=limits,
                   V34_DUPLEX_EXPECT_RATES=f'{rate},{rate}',
                   V34_DUPLEX_DELAY=str(delay),
                   V34_DUPLEX_PAYLOAD=str(args.bits),
                   V34_DUPLEX_BLOCKS=str(args.blocks))
        if noise != 'clean':
            env['V34_DUPLEX_NOISE_DB'] = noise
        with tempfile.TemporaryDirectory(prefix='v34-rate-') as scratch:
            env.update(TMPDIR=scratch, ME_DUMP_DIR=scratch)
            result = subprocess.run(
                [str(binary), '3429', str(args.initial_rate), law],
                cwd=ROOT, env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, timeout=60)
        markers = ('Tx MP receive-rate:', 'receive SNR', 'B1 acquired:',
                   'V.34 duplex ', 'MP rates:', 'FAIL: MP rates')
        logs = [line for line in result.stdout.splitlines()
                if any(marker in line for marker in markers)]
        return dict(rate=rate, initial_rate=args.initial_rate, law=law,
                    delay=delay, noise=noise, bits=args.bits,
                    blocks=args.blocks, rc=result.returncode,
                    binary_sha256=digest, logs=logs)

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.j) as pool:
        for result in pool.map(run, rows):
            print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
