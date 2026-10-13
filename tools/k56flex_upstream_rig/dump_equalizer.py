#!/usr/bin/env python3
"""Dump MICA's 6910 equalizer coefficients (C53 DM 0B80..0C3F) at call times.

usage: dump_equalizer.py COMMAND.json OUTDIR T [T...]

Reruns a saved mica_trace command (the rerun is deterministic) with the
capture options removed and `mica_trace --dump T` added, one run per T, all
outputs in OUTDIR. Then prints the coefficient vector's size and its change
between consecutive dumps.
"""
import json, math, os, struct, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

LO, HI = 0xB80, 0xC40
DROP = {'--capture-word', '--watch', '--writers', '--writes-to',
        '--capture-pc', '--from', '--to', '--report'}


def run(cmdfile, outdir, t):
    c = json.load(open(cmdfile))
    old = os.path.dirname(cmdfile)
    out, i, a = [], 0, c['command']
    while i < len(a):
        if a[i] in DROP:
            i += 2
            continue
        if a[i] == '--' and '--dump' not in out:
            out += ['--dump', t, f'{outdir}/d{t}']
        out.append(a[i].replace(old, outdir) if not a[i].endswith('.txt') else a[i])
        i += 1
    env = {**os.environ, **c['environment'], 'V34_DATA_TX_DUMP': f'{outdir}/tx{t}'}
    out = [f'{outdir}/dte{t}' if x.endswith('-dte') else x for x in out]
    os.makedirs(f'{outdir}/dte{t}', exist_ok=True)
    subprocess.run(out, env=env, cwd=c['cwd'], capture_output=True)
    return struct.unpack('<65536h', open(f'{outdir}/d{t}.data', 'rb').read())[LO:HI]


def main():
    cmdfile, outdir, *times = sys.argv[1:]
    times.sort(key=float)
    os.makedirs(outdir, exist_ok=True)
    with ThreadPoolExecutor() as ex:
        vecs = list(ex.map(lambda t: run(cmdfile, outdir, t), times))
    final = vecs[-1]
    norm = math.sqrt(sum(x * x for x in final))
    prev = None
    for t, v in zip(times, vecs):
        d = lambda a, b: math.sqrt(sum((p - q) ** 2 for p, q in zip(a, b))) / norm
        print(f'{t:>7} rms {math.sqrt(sum(x*x for x in v)/len(v)):6.0f} '
              f'to-last {d(v, final):.3f} step {"-" if prev is None else f"{d(v, prev):.3f}"}')
        prev = v


if __name__ == '__main__':
    main()
