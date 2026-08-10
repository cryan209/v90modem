"""Segment a V.90 downstream capture into Phase 3 stages.

Classifies each 50 ms window by Ucode-0 density, distinct magnitudes and
6-symbol sign periodicity, then merges runs.  Enough to tell Sd (1/3 Ucode 0)
from TRN1d (constant magnitude, scrambled signs) from the multi-level data
stages, without assuming where any of them start.

    python3 tools/v90_phase3_segment.py artifacts/eicon-digital-downstream/*.ulaw

Raw G.711 mu-law codewords at 8 kHz, no header.

CAUTION when diffing two captures: only Sd, S-bar and TRN1d are sent
unconditionally (§9.3.1.3).  Jd runs until the analogue modem's S arrives, so
on a call that failed, every stage from Jd onward has a duration set by the
failure rather than by our transmitter.  See docs/eicon_downstream_comparison.md.
"""

import collections, sys, pathlib

def load(p):
    d = pathlib.Path(p).read_bytes()
    return [(((b ^ 0xFF) >> 7) & 1, (b ^ 0xFF) & 0x7F) for b in d]

def sign_period(signs, p=6):
    """fraction of p-groups equal to the modal group, best alignment"""
    best = 0.0
    for off in range(p):
        g = [tuple(signs[i:i+p]) for i in range(off, len(signs)-p, p)]
        if not g: continue
        c = collections.Counter(g).most_common(1)[0][1]
        best = max(best, c/len(g))
    return best

def profile(sym, win_ms=50):
    """(start_ms, %zero, n_distinct_mag, modal_mag, sign_p6)"""
    w = win_ms*8
    out = []
    for s in range(0, len(sym)-w, w):
        seg = sym[s:s+w]
        mags = [m for _, m in seg]
        z = sum(1 for m in mags if m == 0)/len(mags)
        nz = [m for m in mags if m]
        modal = collections.Counter(nz).most_common(1)[0][0] if nz else 0
        out.append((s/8, z, len(set(mags)), modal, sign_period([g for g,_ in seg])))
    return out

def classify(z, ndist, modal, p6):
    if z > 0.97:                      return "silence"
    if ndist <= 2 and z < 0.05:
        return "const-mag periodic" if p6 > 0.9 else "const-mag scrambled"
    if 0.25 < z < 0.45 and ndist <= 3: return "Sd-like (1/3 zero)"
    if z > 0.7:                        return "sparse (5/6 zero)"
    if ndist > 8:                      return "multi-level (data/DIL)"
    return f"other(nd={ndist})"

def segments(sym):
    rows = profile(sym)
    out, cur = [], None
    for t, z, nd, modal, p6 in rows:
        c = classify(z, nd, modal, p6)
        key = (c, modal if 'const' in c else None)
        if cur and cur[0] == key:
            cur[2] = t+50; cur[3].append(p6)
        else:
            if cur: out.append(cur)
            cur = [key, t, t+50, [p6], modal]
    if cur: out.append(cur)
    return out

for path in sys.argv[1:]:
    sym = load(path)
    print(f"\n===== {pathlib.Path(path).name}  ({len(sym)/8:.0f} ms) =====")
    print(f"{'start':>9} {'end':>9} {'dur ms':>9} {'symbols':>8}  {'p6':>4}  class")
    for (c, _), t0, t1, p6s, modal in segments(sym):
        dur = t1-t0
        if dur < 100: continue
        tag = c + (f" U={modal}" if 'const' in c else "")
        print(f"{t0:9.1f} {t1:9.1f} {dur:9.1f} {int(dur*8):8d}  {sum(p6s)/len(p6s):4.2f}  {tag}")
