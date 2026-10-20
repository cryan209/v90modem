#!/usr/bin/env python3
"""V.32bis 5.2.1/6.1: recover S and R1 from three foreign-modem calls.

The original failures grade only the first R1: our TX diverges after it.
The echo fixture was recorded after the first fix and grades through R3,
where the second fix first changes our TX. Neither grades later events.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / 'reference-data/v32bis-rf-s'

def main():
    env = dict(os.environ, ME_MODE='v32bis', V32BIS_TRACE='1')
    with tempfile.TemporaryDirectory(prefix='v32bis-rf-') as tmp:
        for ext in ('6311', '6312', '6313', '6311-echo'):
            result = subprocess.run([
                str(ROOT / 'v90_engine_replay'), str(FIXTURES / f'{ext}-rx.g711'),
                'ulaw', '--dial', '--sync', '--split', '--sip-answer',
                '--pty-link', str(Path(tmp) / 'pty'),
            ], cwd=ROOT, env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, timeout=30)
            log = result.stdout
            s = re.search(r'\[V32BIS call  \] incoming S at rx symbol (\d+)', log)
            r1 = re.search(r'\[V32BIS call  \] rate signal 0x([0-9a-f]+) at rx symbol (\d+)', log)
            if (result.returncode or not s or not r1
                    or int(r1[1], 16) != 0x1660
                    or int(r1[2]) <= int(s[1])):
                raise AssertionError(f'{ext}: failed S/R1 acquisition\n{log}')
            if ext == '6311-echo':
                # Our recorded TX agrees through R3 here. The answerer's
                # real second S follows our R2; an earlier S is our echo.
                rates = re.findall(r'rate signal 0x([0-9a-f]+) at rx symbol (\d+)', log)
                if len(rates) < 2 or int(rates[1][0], 16) != 0x1000:
                    raise AssertionError(f'{ext}: missed R3 after our own S echo\n{log}')
                fit = re.search(r'bulk delay (\d+) samples .*?\(([\d.]+) dB\), in force', log)
                if not fit or not 1500 <= int(fit[1]) <= 2200 or float(fit[2]) < 15:
                    raise AssertionError(f'{ext}: delayed echo was not removed\n{log}')
                print(f'{ext}: bulk echo lag {fit[1]}, ERLE {fit[2]} dB')
                print(f'{ext}: R3=0x{rates[1][0]} at symbol {rates[1][1]}')
            print(f'{ext}: S at symbol {s[1]}, R1=0x{r1[1]} at {r1[2]}')
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
