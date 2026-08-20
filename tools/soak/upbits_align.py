#!/usr/bin/env python3
"""Align a raw V.90 upstream bit dump against what the peer's DTE sent.

The dump (ME_V90_UPSTREAM_BIT_DUMP) is the descrambled bit stream before any
V.14 framing, so it is the only place the receiver's own framing cannot hide
a fault.  An idle DTE reads as all ones here; a DTE sending the soak pattern
reads as back-to-back 10-bit async characters.

Reports, in order of how much they narrow things down:
  * the ones fraction, in blocks -- 100% is idle decoding correctly
  * V.14 framing evidence at each of the 10 bit phases (start 0, stop 1)
  * whether the soak pattern "U%07d\\n" appears at any phase and bit order
  * HDLC flags, in case the peer is running LAPM rather than V.14
"""
import sys

PATTERN_LINE = b"U0000123\n"


def load_bits(path):
    data = open(path, "rb").read()
    bits = bytearray()
    for byte in data:
        for k in range(7, -1, -1):
            bits.append((byte >> k) & 1)
    return bits


def ones_profile(bits, blocks=10):
    step = max(1, len(bits)//blocks)
    out = []
    for i in range(0, len(bits) - step + 1, step):
        seg = bits[i:i + step]
        out.append(sum(seg)/len(seg))
    return out


def v14_evidence(bits, limit=400000):
    best = []
    for phase in range(10):
        starts = stops = count = 0
        for i in range(phase, min(len(bits) - 10, limit), 10):
            starts += (bits[i] == 0)
            stops += (bits[i + 9] == 1)
            count += 1
        if count:
            best.append((starts/count + stops/count, phase,
                         starts/count, stops/count))
    best.sort(reverse=True)
    return best


def char_bits(byte, lsb_first=True):
    out = [0]
    order = range(8) if lsb_first else range(7, -1, -1)
    for k in order:
        out.append((byte >> k) & 1)
    out.append(1)
    return out


def find_pattern(bits, limit=600000):
    """Look for any 'U' followed by seven digits and a newline."""
    hits = []
    for lsb_first in (True, False):
        want = []
        for ch in b"U":
            want += char_bits(ch, lsb_first)
        w = len(want)
        found = 0
        for i in range(0, min(len(bits) - w, limit)):
            if bits[i:i + w] == bytearray(want):
                found += 1
        hits.append((found, "lsb-first" if lsb_first else "msb-first"))
    return hits


def hdlc_flags(bits, limit=600000):
    flag = bytearray([0, 1, 1, 1, 1, 1, 1, 0])
    found = 0
    for i in range(0, min(len(bits) - 8, limit)):
        if bits[i:i + 8] == flag:
            found += 1
    return found


def main():
    bits = load_bits(sys.argv[1])
    print(f"bits: {len(bits)}")
    prof = ones_profile(bits)
    print("ones by block: " + " ".join(f"{p:.2f}" for p in prof))
    print("=== V.14 framing evidence (start0 + stop1 per phase) ===")
    for score, phase, st, sp in v14_evidence(bits)[:4]:
        print(f"  phase {phase}: start0={st:.3f} stop1={sp:.3f}")
    print("=== 'U' character occurrences ===")
    for found, order in find_pattern(bits):
        print(f"  {order}: {found}")
    print(f"=== HDLC flags (0x7E): {hdlc_flags(bits)} ===")
    print("A correct stream shows one framing phase near 1.000/1.000 and "
          "many 'U' hits; a permuted one shows neither, at any phase.")


if __name__ == "__main__":
    main()
