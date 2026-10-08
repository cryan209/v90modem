#!/usr/bin/env python3
"""Replay a foreign DIL terminator; no claim of live V.90 interoperation.

V.90 9.3.2.9 permits silence during DIL. The following 128T S + 16T S-bar
in 9.3.2.10 must survive the receive gates and end DIL (9.3.1.6).
Use the preserved eicon-v90a-dil2-20261009/live-rx.g711 capture.
"""
import argparse
import os
import re
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--alternation-capture', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env.update(ME_MODE='v90', ME_V90_ROLE='digital',
               ME_V90_DIL_AUTOTERMINATE_CYCLES='0')
    env.pop('ME_V34_SPAN_FLOW_LOG', None)
    env.pop('ME_V90_DIL_S_ACTIVE_FRACTION', None)

    def replay(settings, capture=args.capture):
        result = subprocess.run(
            [str(root / 'v90_engine_replay'), str(capture.resolve()),
             'ulaw', '--fast', '--split'], env=settings, cwd=root,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=30, check=True)
        return result.stdout

    old = replay(dict(env, ME_V90_DIL_S_ACTIVE_FRACTION='0.5'))
    fixed = replay(env)
    boundary = 'DIL termination requested, completed segment'
    if 'REJECTED as dead line during DIL' not in old or boundary in old:
        raise SystemExit('capture did not reproduce the old DIL receive rejection')
    if boundary not in fixed or 'REJECTED as dead line during DIL' in fixed:
        raise SystemExit('default receive gates did not release the valid terminator')
    if not re.search(r'kind=CPt[^\n]*accepted=1', fixed):
        raise SystemExit('replay did not proceed to CPt reception')
    if args.alternation_capture:
        alternating = replay(env, args.alternation_capture)
        if boundary not in alternating or not re.search(
                r'kind=CPt[^\n]*accepted=1', alternating):
            raise SystemExit('confirmed alternation did not release DIL and reach CPt')
    print('PASS: foreign S/S-bar burst ends DIL; old 300 ms activity gate rejects it')


if __name__ == '__main__':
    main()
