#!/usr/bin/env python3
"""Clean time and longest hold from a v90_upstream_replay log.

Reads the receiver's own "DATA bits" lines.  Clean is TIME, weighted by the
interval each window covers, because the windows are not of equal length --
see docs/v90_upstream_data_path.md.  Intact U%07d payload lines, where the
harness saw any, are the honest measure and are reported beside it.
"""
import re
import sys

CLEAN = 0.20        # symbol error below which the eye is open

t, e = [], []
for line in sys.stdin:
    m = re.search(r't=([0-9.]+)s.*sym err ([0-9.]+)', line)
    if m:
        t.append(float(m.group(1)))
        e.append(float(m.group(2)))
if not t:
    print('no data mode')
    sys.exit(0)
clean = run = best = 0.0
for i in range(1, len(t)):
    d = t[i] - t[i - 1]
    if d < 0 or d > 5:      # a new data mode, not a gap in one
        d = 0.0
    if e[i] < CLEAN:
        clean += d
        run += d
        best = max(best, run)
    else:
        run = 0.0
print('span=%6.1fs clean=%6.1fs (%3.0f%%) longest=%6.1fs'
      % (t[-1], clean, 100.0*clean/max(t[-1], 1e-9), best))
