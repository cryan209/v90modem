#!/usr/bin/env python3
"""Regression: MT9234 upstream adaptation must remain in the input frame.

The fixture ends before the live server first requests resynchronization.
It contains acquisition damage, so zero HDLC errors is not its acceptance
criterion; require the six real CRC-valid frames, bounded damaged segments,
and a final symbol error below 0.15. Before the correction the same replay
has 30 damaged segments and rises to 0.297.
"""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parent.parent
fixture = root / 'reference-data/artifacts/mt9234-v90-gradient'
with tempfile.TemporaryDirectory(prefix='mt9234-gradient-') as temp:
    bits = Path(temp) / 'bits.txt'
    env = dict(os.environ, V90_REPLAY_QUIET='1', V90_REPLAY_BITS_OUT=str(bits),
               V90_REPLAY_ECHO_TX=str(fixture / 'tx.ulaw'),
               V90_REPLAY_ECHO_WINDOW='12,15')
    replay = subprocess.run([str(root / 'v90_upstream_replay'),
                             str(fixture / 'rx.ulaw'), 'ulaw', '3200',
                             '26400', '24.321', '14'], env=env,
                            capture_output=True, text=True, timeout=30)
    text = replay.stdout + replay.stderr
    assert replay.returncode == 0, text
    assert 'replay: acquired=1' in text and 'replay: carrier_lost=0' in text, text
    errors = re.findall(r'sym err ([0-9.]+)', text)
    assert errors and float(errors[-1]) < .15, text
    frames = subprocess.run([sys.executable, str(root / 'tools/hdlc_bit_check.py'),
                             str(bits)], capture_output=True, text=True,
                            timeout=10)
    stats = json.loads(frames.stdout)
    assert stats['valid_complete_frames'] >= 6, stats
    assert stats['bad_complete_frames'] <= 3, stats
    assert stats['aborted_segments'] <= 1, stats
    print(json.dumps(dict(stats, final_symbol_error=float(errors[-1]))))
