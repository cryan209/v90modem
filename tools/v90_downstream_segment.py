#!/usr/bin/env python3
"""Reference segmentation of a V.90 digital-side downstream DS0 capture.

Deliberately independent of the C receive path. `tools/eicon_rx_conformance.py`
checks `vpcm_decode` against exact expected offsets; those offsets have to come
from somewhere that cannot share a bug with the thing under test, and this is
that somewhere. It reimplements only what §8.4 defines, from the Recommendation
rather than from our source:

  µ-law    positive codeword = 0xFF - Ucode, bit 7 = polarity (1 = positive)
  §8.4.4   Sd      = 64 reps of {+W, +0, +W, -W, -0, -W}, W = Ucode(16 + UINFO)
           S-bar_d = 8 reps of the sign-inverted frame
  §8.4.5   TRN1d   = constant Ucode UINFO, signs = binary ones through the §5.3
                     scrambler, initialized to zero
  §5.3     GPC     = 1 + x^-18 + x^-23

Zero-slot polarity is not checked. §8.4.4 writes "-0", but G.711 Ucode 0 has two
codewords for one level and a working digital modem sends +0 in both slots --
see docs/eicon_downstream_comparison.md, Finding 3(a).

    python3 tools/v90_downstream_segment.py artifacts/eicon-digital-downstream/*.ulaw
"""

import argparse
import sys


def decode(path):
    data = open(path, 'rb').read()
    sgn = bytearray(len(data))   # 1 = positive
    uc = bytearray(len(data))
    for i, b in enumerate(data):
        sgn[i] = 1 if (b & 0x80) else 0
        uc[i] = 0x7F - (b & 0x7F)
    return sgn, uc


def gpc_ones(n):
    """n TRN1d sign bits: binary ones through GPC, scrambler zeroed (§8.4.5)."""
    sr = 0
    out = bytearray(n)
    for i in range(n):
        fb = ((sr >> 22) ^ (sr >> 17)) & 1
        ob = 1 ^ fb
        sr = ((sr << 1) | ob) & 0x7FFFFF
        out[i] = ob
    return out


def constant_runs(uc, min_len):
    """Maximal runs of one Ucode, as (start, end, ucode)."""
    i, n = 0, len(uc)
    while i < n:
        j = i
        while j < n and uc[j] == uc[i]:
            j += 1
        if j - i >= min_len:
            yield i, j, uc[i]
        i = j


def sd_frame(w):
    """§8.4.4 Sd, as (sign, ucode) with None where polarity is not checked."""
    return [(1, w), (None, 0), (1, w), (0, w), (None, 0), (0, w)]


def match_frame(sgn, uc, at, frame, rot, n):
    for k in range(n):
        s, u = frame[(rot + k) % 6]
        if uc[at + k] != u:
            return False
        if s is not None and sgn[at + k] != s:
            return False
    return True


def find_sd(sgn, uc, w):
    """First offset where Sd holds for 8 frames, and its rotation."""
    frame = sd_frame(w)
    for start in range(len(uc) - 48):
        for rot in range(6):
            if match_frame(sgn, uc, start, frame, rot, 48):
                return start, rot
    return None, None


def segment(path, min_run):
    sgn, uc = decode(path)
    n = len(uc)
    print(f"\n=== {path}  ({n} symbols, {n / 8.0:.1f} ms) ===")

    runs = [(a, b, u) for a, b, u in constant_runs(uc, min_run) if u != 0]
    if not runs:
        print("  no constant-Ucode runs -- not a digital-side downstream?")
        return

    print("  constant-Ucode runs:")
    for a, b, u in runs:
        print(f"    T{a:<7d} - T{b:<7d} {b - a:>7d}T {(b - a) / 8.0:9.1f} ms  Ucode {u}")

    # The longest constant run is TRN1d + Jd, which fixes UINFO.
    u_info = max(runs, key=lambda r: r[1] - r[0])[2]
    w = 16 + u_info
    print(f"\n  UINFO = {u_info}  ->  W = Ucode {w} (§8.4.4)")

    sd, rot = find_sd(sgn, uc, w)
    if sd is None:
        print("  Sd: NOT FOUND")
        return

    frame = sd_frame(w)
    k = 0
    while sd + k < n and match_frame(sgn, uc, sd + k, frame, rot + k, 1):
        k += 1
    print(f"  Sd:     T{sd:<7d} {k:>7d}T {k / 8.0:9.1f} ms  ({k // 6} reps)")

    # S-bar_d is the same frame with the W-slot polarities inverted.
    inv = [(s if s is None else 1 - s, u) for s, u in frame]
    j, m = sd + k, 0
    while j + m < n and match_frame(sgn, uc, j + m, inv, rot + k + m, 1):
        m += 1
    print(f"  S̄d:     T{j:<7d} {m:>7d}T {m / 8.0:9.1f} ms  ({m // 6} reps)")

    # TRN1d is the prefix of the following U_INFO run that descrambles to ones;
    # Jd is the rest of that run (§8.4.5, §8.4.2).
    start = j + m
    run_end = start
    while run_end < n and uc[run_end] == u_info:
        run_end += 1
    ones = gpc_ones(run_end - start)
    lock = 0
    while lock < run_end - start and sgn[start + lock] == ones[lock]:
        lock += 1
    print(f"  TRN1d:  T{start:<7d} {lock:>7d}T {lock / 8.0:9.1f} ms  (GPC-locked)")
    jd = run_end - start - lock
    print(f"  Jd:     T{start + lock:<7d} {jd:>7d}T {jd / 8.0:9.1f} ms")

    # DIL runs from the end of Jd to the next constant-Ucode run, which is Ri.
    later = [r for r in runs if r[0] >= run_end]
    if later:
        ri_start, ri_end, _ = later[0]
        print(f"  DIL:    T{run_end:<7d} {ri_start - run_end:>7d}T "
              f"{(ri_start - run_end) / 8.0:9.1f} ms")
        print(f"  Ri:     T{ri_start:<7d} {ri_end - ri_start:>7d}T "
              f"{(ri_end - ri_start) / 8.0:9.1f} ms  (§8.6.4)")
        print(f"  data:   T{ri_end:<7d} {n - ri_end:>7d}T "
              f"{(n - ri_end) / 8.0:9.1f} ms")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("captures", nargs="+", help="raw µ-law DS0 captures")
    ap.add_argument("--min-run", type=int, default=200,
                    help="shortest constant-Ucode run to report (default 200T)")
    args = ap.parse_args()
    for path in args.captures:
        segment(path, args.min_run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
