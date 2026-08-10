#!/usr/bin/env python3
"""Decode the Jd frame from a V.90 downstream DS0 capture.

Jd (§8.4.2) is the constant-Ucode-U_INFO stretch immediately after TRN1d, with
its bits scrambled, differentially encoded, and carried in the codeword signs.
Recovering it needs three things the Recommendation states but does not spell
out together:

  * TRN1d must be located first, because §8.4.2 initializes the differential
    encoder with "the final symbol of the transmitted TRN1d".  We find TRN1d by
    generating the §5.3 GPC all-ones sequence and sliding it over the capture --
    it locks at 100%, which also fixes the sign polarity.
  * The scrambler is NOT reinitialized at the Jd boundary, so the descrambler's
    23-bit history at Jd's first bit is TRN1d's own tail.  TRN1d carries no
    differential encoding, so its signs are its scrambled bits directly.
  * The CRC is the one piece §8.4.2 delegates ("described in 10.1.2.3.2/V.34")
    without restating.  Verified against a working implementation:

        CRC-16-CCITT, poly 0x1021, init 0xFFFF,
        over the two 16-bit information groups (bits 18:33 and 35:50),
        transmitted MSB-first in bits 52:67.

Validated on two independent Eicon Diva Server PRI calls that a USR Courier
answered with CONNECT: 72-bit period at 98%, all three of Table 13's fixed
fields exact, CRC valid, and the identical frame on both.

    python3 tools/v90_jd_decode.py artifacts/eicon-digital-downstream/*.ulaw

Raw G.711 mu-law codewords at 8 kHz, no header.
"""

import pathlib
import sys


def gpc_ones(n, taps=(18, 23)):
    """§5.3 GPC scrambler, all-ones input, zero-initialized."""
    s = []
    for i in range(n):
        v = 1
        for t in taps:
            v ^= s[i - t] if i - t >= 0 else 0
        s.append(v)
    return s


def crc16_ccitt(bits, poly=0x1021, init=0xFFFF):
    reg = init
    for b in bits:
        fb = ((reg >> 15) & 1) ^ b
        reg = (reg << 1) & 0xFFFF
        if fb:
            reg ^= poly
    return reg


def find_trn1d(signs, probe=512):
    """Locate TRN1d by GPC lock.  Returns (offset, polarity, length)."""
    packed = 0
    for i, s in enumerate(signs):
        if s:
            packed |= 1 << i
    mask = (1 << probe) - 1
    ref = 0
    for i, v in enumerate(gpc_ones(probe)):
        if v:
            ref |= 1 << i

    best = (probe + 1, -1, 0)
    for pol, r in ((0, ref), (1, ref ^ mask)):
        for off in range(len(signs) - probe):
            m = (((packed >> off) & mask) ^ r).bit_count()
            if m < best[0]:
                best = (m, off, pol)
    mism, off, pol = best
    if mism > probe // 8:
        return None
    run = 0
    for i, r in enumerate(gpc_ones(len(signs) - off)):
        if signs[off + i] != (r ^ pol):
            break
        run += 1
    return off, pol, run


def decode(path):
    data = pathlib.Path(path).read_bytes()
    signs = [((b ^ 0xFF) >> 7) & 1 for b in data]
    mags = [(b ^ 0xFF) & 0x7F for b in data]

    found = find_trn1d(signs)
    print(f"\n===== {pathlib.Path(path).name} =====")
    if not found:
        print("  no TRN1d found (no GPC lock) -- cannot seed the Jd decode")
        return
    off, pol, run = found
    u_info = mags[off]
    print(f"  TRN1d: {off/8:.1f} ms, {run}T ({run/8:.1f} ms), U_INFO={u_info}")

    jd = off + run
    n = 0
    while jd + n < len(mags) and mags[jd + n] == u_info:
        n += 1
    if n < 72:
        print(f"  Jd: only {n} constant-U{u_info} symbols after TRN1d -- too short")
        return
    print(f"  Jd:    {jd/8:.1f} ms, {n}T ({n/8:.1f} ms), {n/72:.1f} frames of 72 bits")

    s = [x ^ pol for x in signs]
    scr = list(s[jd - 23:jd])          # TRN1d tail: signs are the scrambled bits
    prev = s[jd - 1]                   # §8.4.2 differential seed
    for i in range(n):
        scr.append(s[jd + i] ^ prev)
        prev = s[jd + i]
    bits = "".join(str(scr[i] ^ scr[i - 18] ^ scr[i - 23])
                   for i in range(23, len(scr)))

    period = sum(1 for i in range(len(bits) - 72) if bits[i] == bits[i + 72])
    print(f"  72-bit periodicity: {100*period/(len(bits)-72):.1f}%")

    sync = bits.find("1" * 17)
    if sync < 0:
        print("  no 17-bit frame sync found")
        return
    f = bits[sync:sync + 72]
    if len(f) < 72:
        print("  frame truncated")
        return

    crc_calc = crc16_ccitt([int(c) for c in f[18:34]] + [int(c) for c in f[35:51]])
    crc_tx = int(f[52:68], 2)
    lookahead = int(f[49]) + 2 * int(f[50])

    print(f"  frame: {f}")
    print(f"    0:16  sync              {f[0:17]} {'OK' if f[0:17]=='1'*17 else 'BAD'}")
    print(f"    17    start bit         {f[17]} {'OK' if f[17]=='0' else 'BAD'}")
    print(f"    18:33 rate mask         {f[18:34]}")
    print(f"    35:46 rate mask (contd) {f[35:47]}")
    print(f"    47    training constln  {f[47]} ({'16-point' if f[47]=='1' else '4-point'})")
    print(f"    48    reneg constln     {f[48]} ({'16-point' if f[48]=='1' else '4-point'})")
    print(f"    49:50 shaping lookahead {f[49]}{f[50]} -> {lookahead}")
    print(f"    52:67 CRC               0x{crc_tx:04x} "
          f"{'VALID' if crc_calc == crc_tx else f'INVALID (computed 0x{crc_calc:04x})'}")
    print(f"    68:71 fill              {f[68:72]} {'OK' if f[68:72]=='0000' else 'BAD'}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for p in sys.argv[1:]:
        decode(p)
