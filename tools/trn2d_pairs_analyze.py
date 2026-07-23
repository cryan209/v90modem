#!/usr/bin/env python3
"""Analyze SmartLink TRN2d [received, reference] int16 pair capture.

reference = predicted_sign * table[(idx-1) mod 6][ulaw(|received|)]
Levels come from the peer's findPadGain table; the SIGN is DATA-AIDED —
the round-3 live pairs disproved the disassembly's sign(received) reading
(50% mismatch against our deterministic +++--- Ri), so the peer regenerates
the transmitter's 5.4.5 shaper sign sequence.

Reports, pure python (no numpy):
  - windowed mean (rx-ref)^2  -> compare against peer's logged Error Energy
  - per-|ref|-level residual  -> level offsets (pad/resampler gain shape)
  - sign agreement            -> THE metric-convention read-out on a real
                                 (post-DIL) TRN2d window: ~0% mismatch =
                                 our 5.4.5.6 shaper metric matches the
                                 peer's prediction; ~31% = codec/transmit
                                 fork; ~50% = degenerate no-DIL window
                                 (peer demodding Ri) or miswired capture
  - biggest-error symbols     -> slicer decision flips / transients
"""
import struct
import sys
from collections import defaultdict

def main(path, win=655):
    data = open(path, "rb").read()
    n = len(data) // 4
    pairs = struct.unpack("<%dh" % (n * 2), data[: n * 4])
    rx = pairs[0::2]
    ref = pairs[1::2]
    print("pairs: %d (%.2f s at 8000 sym/s)" % (n, n / 8000.0))

    # sign agreement: data-aided reference vs received — the shaper-metric
    # convention read-out (~0% good / ~31% convention fork / ~50% degenerate)
    signdiff = sum(1 for a, b in zip(rx, ref) if (a < 0) != (b < 0) and b != 0)
    print("sign disagreements: %d / %d (%.1f%%)"
          % (signdiff, n, 100.0 * signdiff / n if n else 0.0))

    # windowed error energy: peer logs Error Energy every ~80 ms (~640 syms)
    print("\nwindowed mean (rx-ref)^2, window=%d syms (~%.0f ms):" % (win, win / 8.0))
    for w in range(0, n, win):
        seg = [(rx[i] - ref[i]) ** 2 for i in range(w, min(w + win, n))]
        if len(seg) < win // 2:
            break
        print("  %7.3f s  mean_err2 = %12.1f  rms_err = %8.1f"
              % (w / 8000.0, sum(seg) / len(seg), (sum(seg) / len(seg)) ** 0.5))

    # per-reference-level residual: group by |ref| (the designed level)
    by_level = defaultdict(list)
    for a, b in zip(rx, ref):
        by_level[abs(b)].append(abs(a) - abs(b))
    print("\nper-designed-level residual (|rx|-|ref|), levels with >=100 hits:")
    print("  %6s %8s %10s %10s %10s" % ("level", "hits", "mean", "rms", "mean/lvl%"))
    for lvl in sorted(by_level):
        r = by_level[lvl]
        if len(r) < 100:
            continue
        mean = sum(r) / len(r)
        rms = (sum(x * x for x in r) / len(r)) ** 0.5
        pct = 100.0 * mean / lvl if lvl else float("nan")
        print("  %6d %8d %10.2f %10.2f %9.2f%%" % (lvl, len(r), mean, rms, pct))

    # top error symbols
    errs = sorted(((rx[i] - ref[i]) ** 2, i) for i in range(n))[-10:]
    print("\nworst symbols (err2, idx, rx, ref):")
    for e, i in reversed(errs):
        print("  %10d  %8d  %6d  %6d" % (e, i, rx[i], ref[i]))

if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 655)
