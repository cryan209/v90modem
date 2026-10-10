#!/usr/bin/env python3
# Sample per-thread CPU of the sip_v90_modem whose cwd is ROOT, every PERIOD s.
# Output TSV: wall_s  tid  comm  utime_ticks  stime_ticks
import os, sys, time
root, out, period = sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
hz = os.sysconf('SC_CLK_TCK')

def find():
    for p in os.listdir('/proc'):
        if not p.isdigit():
            continue
        try:
            if open(f'/proc/{p}/comm').read().strip() != 'sip_v90_modem':
                continue
            if os.readlink(f'/proc/{p}/cwd') == root:
                return p
        except OSError:
            pass
    return None

pid = None
for _ in range(120):
    pid = find()
    if pid:
        break
    time.sleep(0.5)
if not pid:
    sys.exit('no server process found')
t0 = time.monotonic()
with open(out, 'w') as f:
    f.write(f'# pid {pid} hz {hz} start_epoch {time.time():.3f}\n')
    while os.path.exists(f'/proc/{pid}/stat'):
        now = time.monotonic() - t0
        try:
            for tid in os.listdir(f'/proc/{pid}/task'):
                s = open(f'/proc/{pid}/task/{tid}/stat').read()
                comm = s[s.index('(') + 1:s.rindex(')')].replace(' ', '_')
                fl = s[s.rindex(')') + 2:].split()
                f.write(f'{now:.3f}\t{tid}\t{comm}\t{fl[11]}\t{fl[12]}\n')
        except OSError:
            break
        f.flush()
        time.sleep(period)
