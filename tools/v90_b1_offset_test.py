#!/usr/bin/env python3
"""V.90 upstream: a B1 placed from the 4-point signal's end is searched to the
symbol (v34rx.c, v90_t3_sym_search).

The Agere MT9234ZBA-USB on v90ras (7803, 2026-11-09) starts B1 seven symbols
after its 4-point CP'/E, not the nine the MT5634SMI does; at the old fixed gap
every superframe phase decoded white and no upstream byte ever arrived.  The
fixture is the first 26 s of that call's receive tap, 3200 baud 16800 bit/s
high carrier, E at 22.34 s.  With the search the upstream decodes to the
peer's continuous HDLC flags.

A second fixture holds the search to its own evidence: a window that shows the
flags keeps its offset (see below).
"""
import os
import re
import subprocess
import tempfile
from pathlib import Path

root = Path(__file__).resolve().parent.parent


def decode(fixture, bps, e_seconds, high_carrier):
    with tempfile.TemporaryDirectory(prefix='v90-b1-offset-') as temp:
        bits = Path(temp) / 'bits.txt'
        env = dict(os.environ, V90_REPLAY_BITS_OUT=str(bits))
        if high_carrier:
            env['V90_REPLAY_HIGH_CARRIER'] = '1'
        replay = subprocess.run([str(root / 'v90_upstream_replay'),
                                 str(root / fixture / 'rx.ulaw'), 'ulaw',
                                 '3200', str(bps), str(e_seconds), '14'],
                                env=env, capture_output=True, text=True,
                                timeout=60)
        text = replay.stdout + replay.stderr
        assert replay.returncode == 0, text[-4000:]
        chosen = re.search(r'symbol offset \+(\d+) chosen', text)
        assert chosen and chosen.group(1) != '0', text[-4000:]
        stream = ''.join(c for c in bits.read_text() if c in '01')
        flags = len(re.findall('01111110', stream))
        print(f'{fixture}: offset +{chosen.group(1)}, {flags} HDLC flags '
              f'in {len(stream)} bits')
        return flags


# white data matches 01111110 once in 256 bits (~240 here)
assert decode('reference-data/artifacts/mt9234-v90ras-b1-offset-20261109',
              16800, 22.34, True) >= 2000

# The same modem dialling 2904 on 2026-11-11 (14400 low carrier, E at 24.83 s):
# the right offset, +5, read 63% ones in its window (XID frames between the
# flags), short of the 70% that keeps an offset at once, so the search stepped
# to +6 -- and the frame-phase lock then fired on that same window's flags
# (90 per mille) and ended the search one symbol out.  Eye 0.009, bits white
# (395 flags in 74196 bits), no LAPM, peer retrain 10.5 s into data.  With the
# window's flags keeping the offset: 8344 flags.
assert decode('reference-data/artifacts/mt9234-v90-offset-lock-20261112',
              14400, 24.83, False) >= 5000
