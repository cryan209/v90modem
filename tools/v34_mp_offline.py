#!/usr/bin/env python3
"""Decide the V.34 Phase 4 MP interpretation offline, against the frame CRC.

10.1.3.9 leaves nothing to search once J has given the constellation: MP is
differentially encoded, scrambled by clause 7, and I1n is the first bit in
time.  But neither of the things a live receiver can check early -- a TRN of
scrambled ones, or MP's own 17-bit all-ones frame sync -- can tell one bit
order from the other, so a preamble-only lock can settle on the wrong one and
produce a valid-looking preamble over a garbage body.  The frame CRC is the
only oracle, so this tries every interpretation against it.

Input: the V34_MP_RX_DUMP file (duration, differential dibit, absolute dibit,
magnitude per Phase 4 symbol).
"""
import sys, itertools

def crc16(bits):
    """V.34 10.1.2.3.2: x^16 + x^12 + x^5 + 1, register preset to all ones."""
    reg = 0xffff
    for b in bits:
        fb = ((reg >> 15) & 1) ^ (b & 1)
        reg = ((reg << 1) & 0xffff)
        if fb:
            reg ^= (1 << 12) | (1 << 5) | (1 << 0)
    return reg

def descramble(bits, taps):
    """Self-synchronising descrambler: out(n) = r(n) ^ r(n-a) ^ r(n-b)."""
    a, b = taps
    out = []
    for n in range(len(bits)):
        v = bits[n]
        if n >= a: v ^= bits[n-a]
        if n >= b: v ^= bits[n-b]
        out.append(v)
    return out

def frames(bits, want_type=None):
    """Yield (start, type, ok) for every 17-ones frame sync in the stream."""
    n = len(bits)
    i = 0
    while i < n - 200:
        if all(bits[i+k] for k in range(17)) and bits[i+17] == 0:
            t = bits[i+18]
            total = 188 if t == 1 else 88
            if i + total <= n:
                # The CRC covers every information bit, i.e. everything except
                # the frame sync, the start bits and the fill bits (10.1.2.3.2).
                if t == 0:
                    # Table 20: starts at 17, 34, 51, 68; CRC 69:84; fill 85:87.
                    body = (bits[i+18:i+34] + bits[i+35:i+51] + bits[i+52:i+68])
                    crc = bits[i+69:i+85]
                else:
                    # Table 21: starts at 17, 34, 51, 68, 85, 102, 119, 136,
                    # 153, 170; CRC 171:186; fill 187.
                    body = (bits[i+18:i+34] + bits[i+35:i+51]
                            + bits[i+52:i+68] + bits[i+69:i+85]
                            + bits[i+86:i+102] + bits[i+103:i+119]
                            + bits[i+120:i+136] + bits[i+137:i+153]
                            + bits[i+154:i+170])
                    crc = bits[i+171:i+187]
                want = crc16(body)
                got = 0
                for k, b in enumerate(crc):
                    got |= (b & 1) << k
                yield (i, t, want == got, want, got)
            i += 1
        else:
            i += 1

def main():
    rows = [l.split() for l in open(sys.argv[1]) if l.strip()]
    diff = [int(r[1]) for r in rows]
    absd = [int(r[2]) for r in rows]
    print("symbols: %d" % len(rows))
    best = None
    for domain_name, dibits in (("diff", diff), ("abs", absd)):
        for order in (0, 1):
            bits = []
            for d in dibits:
                i1, i2 = d & 1, (d >> 1) & 1
                bits.extend((i1, i2) if order == 0 else (i2, i1))
            for taps in ((18, 23), (5, 23)):
                ds = descramble(bits, taps)
                syncs = 0; good = 0
                for (i, t, ok, w, g) in frames(ds):
                    syncs += 1
                    if ok: good += 1
                print("  domain=%-4s order=%s taps=%s: syncs=%4d crc_ok=%d"
                      % (domain_name, "I1,I2" if order == 0 else "I2,I1",
                         "x^-%d,x^-%d" % taps, syncs, good))
                if best is None or good > best[0]:
                    best = (good, domain_name, order, taps)
    print("best: crc_ok=%d domain=%s order=%s taps=%s"
          % (best[0], best[1], "I1,I2" if best[2] == 0 else "I2,I1", best[3]))

main()
