#!/usr/bin/env python3
"""V.8 timeline audit of a call's two G.711 taps against V.8 (11/2000) clause 8.

Per call, from the TX tap (what we sent) and RX tap (what arrived):
  ansam_on/off  2100 Hz burst in RX (7.2: 5 +/- 1 s if no CM heard, 8.2.2)
  ci_in_ansam   seconds of our V.21(L) CI transmitted while ANSam is up
                (8.1.1: the call signal "shall be stopped" on ANSam detection)
  te            silence between our last CI burst and CM start (8.1.1: >= 0.5 s,
                >= 1 s for V.25 echo-canceller disabling)
  cm_on         CM start relative to ANSam onset
  cm_dbm0       CM level (mu-law tap, 0 dBm0 = RMS 15767 on a 16-bit scale, since mu-law overload is +3.17 dBm0)
  status        V.8 status from server.log (2 = OK, 4 = failed)
Timing is read in one tap's own sample clock; TX and RX taps are started
together by the engine, and every event here is >= 20 ms, so the per-call
offset between them (tens of ms at most) does not change a verdict.
Usage: v8_timeline_audit.py <call dir>...
"""
import sys, os, re, numpy as np

def ulaw(b):
    u = ~np.frombuffer(b, np.uint8).astype(np.int32) & 0xff
    s, e, m = u & 0x80, (u >> 4) & 7, u & 15
    x = (((m << 3) + 0x84) << e) - 0x84
    return np.where(s, -x, x).astype(float)

def bands(x, freqs, blk=160):
    n = len(x) // blk
    x = x[:n*blk].reshape(n, blk) * np.hanning(blk)
    X = np.abs(np.fft.rfft(x, axis=1))**2
    f = np.fft.rfftfreq(blk, 1/8000)
    tot = X.sum(1) + 1e-9
    out = {k: X[:, (f >= lo) & (f <= hi)].sum(1) / tot for k, (lo, hi) in freqs.items()}
    rms = np.sqrt((x**2).mean(1) / 0.375)
    return out, rms, blk / 8000

def runs(mask, dt, minlen):
    r, start = [], None
    for i, v in enumerate(list(mask) + [False]):
        if v and start is None: start = i
        if not v and start is not None:
            if (i - start) * dt >= minlen: r.append((start*dt, i*dt))
            start = None
    return r

def audit(d):
    rx = ulaw(open(os.path.join(d, 'live-rx.g711'), 'rb').read())
    tx = ulaw(open(os.path.join(d, 'live-tx.g711'), 'rb').read())
    log = open(os.path.join(d, 'server.log'), errors='replace').read()
    m = re.search(r'V\.8 result: status=(\d+)', log)
    status = m.group(1) if m else '?'
    rb, rrms, dt = bands(rx, {'ans': (2050, 2150)})
    ansam = runs((rb['ans'] > 0.6) & (rrms > 50), dt, 1.0)
    if not ansam: return (d, status, None)
    a0, a1 = ansam[0]
    tb, trms, _ = bands(tx, {'v21l': (900, 1260)})
    v21 = (tb['v21l'] > 0.6) & (trms > 50)
    bursts = runs(v21, dt, 0.06)
    # CM = first burst starting after ANSam onset lasting >= 0.4 s (CM streams)
    cm = next((b for b in bursts if b[0] > a0 and b[1] - b[0] >= 0.4), None)
    if cm is None: return (d, status, (a0, a1, None))
    ci_before = [b for b in bursts if b[1] <= cm[0] + 1e-9 and b != cm]
    ci_overlap = sum(max(0, min(b[1], a1) - max(b[0], a0)) for b in ci_before)
    te = cm[0] - ci_before[-1][1] if ci_before else None
    i0, i1 = int(cm[0]/dt), int(min(cm[1], cm[0]+0.5)/dt)
    lvl = 20*np.log10(np.sqrt(np.mean(trms[i0:i1]**2)) / 15767 + 1e-12) if i1 > i0 else float('nan')
    return (d, status, (a0, a1, cm[0], ci_overlap, te, lvl))

print(f"{'call':48s} st  ansam_on dur   ci_in_ansam te     cm-ansam cm_dBm0")
for d in sys.argv[1:]:
    try: r = audit(d)
    except Exception as e: print(f"{d:48s} error {e}"); continue
    d, st, v = r
    name = d.replace('artifacts/', '')[-48:]
    if v is None: print(f"{name:48s} {st:2s}  no ANSam"); continue
    if v[2] is None: print(f"{name:48s} {st:2s}  {v[0]:6.2f} {v[1]-v[0]:5.2f} no CM"); continue
    a0, a1, c0, ov, te, lvl = v
    tes = f"{te:5.2f}" if te is not None else "  -  "
    print(f"{name:48s} {st:2s}  {a0:6.2f} {a1-a0:5.2f} {ov:6.2f}      {tes}  {c0-a0:6.2f}  {lvl:6.1f}")
