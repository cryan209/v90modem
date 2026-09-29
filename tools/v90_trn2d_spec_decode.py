#!/usr/bin/env python3
"""Decode OUR OWN V.90 Phase 4 downstream (TRN2d then MP) off a transmit tap,
with a receiver written from the Recommendation text alone.

Every other check of our TRN2d/MP (v90_analogue_rx_test --phase4-trace, the
analogue role) shares code or conventions with the transmitter in v90.c, so
it cannot catch a wrong-but-self-consistent convention, and a lenient peer
(SmartLink) reaching data mode does not prove conformance either.  This file
imports nothing from the tree:

  5.4.4  labels in DESCENDING Ucode order (label 0 = largest PCM code)
  5.4.3  K bits -> R0 = b0 + 2*b1 + ...; Ki = Ri mod Mi, interval 0 least
         significant
  5.4.2  d0..dS-1 are the sign inputs s0.., dS..dD-1 are b0..bK-1
  5.4.5.2 / Tables 3, 4 and Figure 2 (Sr = 1 only): the shaper's rule is
         removed without knowing it.  With z_j(k) = $_j(k) xor $_j-1(k), the
         even inversion component of consecutive rules is z_j(0) (because
         p'(0) = 0), and
            s0 = z(1) ^ z_j-1(5) ^ z(0)   s1 = z(2) ^ z(0)   s2 = z(3) ^ z(1)
            s3 = z(4) ^ z(0)              s4 = z(5) ^ z(3)
  5.3    GPC descrambler (V.34 7-1), state zero at TRN2d per 8.6.5
  Table 16 MP framing; CRC per V.34 10.1.2.3.2 in the reflected form that
         validates the analogue modem's own CPt (0x8408, init 0xFFFF, start
         bits and frame sync excluded, field LSB first)

A conformant TRN2d descrambles to ALL ONES (8.6.5: scrambled binary ones)
and every MP must pass its CRC.

  v90_trn2d_spec_decode.py <live-tx.g711> <trn2d_start_sample> <trn2d_symbols>
                           <ucode,ucode,...> [ulaw|alaw]

trn2d_start_sample is the first TRN2d symbol in the tap: find Ri (U_INFO,
+++--- signs), its 4 x ---+++ barred repetitions, and take the next sample.
The mask is the CPt TRANSMIT mask (Table 14 bits 136.., not the bit-128
codec-output set) and must be the same in all six intervals.
"""
import sys
import numpy as np


def main():
    if len(sys.argv) < 5:
        print(__doc__)
        sys.exit(1)
    tap = sys.argv[1]
    start = int(sys.argv[2])
    ntrn = int(sys.argv[3])
    mask = sorted((int(u) for u in sys.argv[4].split(',')), reverse=True)
    law = sys.argv[5] if len(sys.argv) > 5 else 'ulaw'
    m = len(mask)
    k_bits = 6 * (m.bit_length() - 1)
    if (1 << (k_bits // 6)) != m:
        sys.exit("only power-of-two constellations (K = 6*log2 M) are handled")
    s_bits = 5
    d_bits = k_bits + s_bits

    x = np.frombuffer(open(tap, 'rb').read(), dtype=np.uint8).astype(int)
    if law == 'alaw':
        ucode = (x ^ 0x55) & 0x7F
    else:
        ucode = 0x7F - (x & 0x7F)
    sign = ((x & 0x80) != 0).astype(int)     # 5.4.6: 1 = positive
    label = {u: i for i, u in enumerate(mask)}

    bits = []
    prev = [0] * 6
    prev2 = [0] * 6
    stop = None
    for j in range((len(x) - start) // 6):
        a = start + 6 * j
        us = ucode[a:a + 6]
        sg = list(sign[a:a + 6])
        if any(int(u) not in label for u in us):
            stop = j
            break
        r = 0
        for i in reversed(range(6)):
            r = r * m + label[int(us[i])]
        b = [(r >> i) & 1 for i in range(k_bits)]
        z = [sg[k] ^ prev[k] for k in range(6)]
        zp5 = prev[5] ^ prev2[5]
        s = [z[1] ^ zp5 ^ z[0], z[2] ^ z[0], z[3] ^ z[1], z[4] ^ z[0],
             z[5] ^ z[3]]
        bits += s + b
        prev2, prev = prev, sg

    d = np.array(bits, dtype=int)
    o = d.copy()
    for n in range(len(d)):
        o[n] = d[n] ^ (d[n - 18] if n >= 18 else 0) ^ (d[n - 23] if n >= 23 else 0)
    print("D=%d K=%d S=%d M=%d; %d frames decoded%s"
          % (d_bits, k_bits, s_bits, m, len(bits) // d_bits,
             "" if stop is None else
             ", stopped at frame %d (symbol outside the mask)" % stop))
    tb = ntrn // 6 * d_bits
    trn = o[:tb]
    print("TRN2d: %.4f ones over %d bits%s"
          % (trn.mean(), tb, "" if trn.min() else "  <-- NOT all ones"))

    def crc16(v):
        c = 0xFFFF
        for bit in v:
            c = (c >> 1) ^ 0x8408 if ((bit ^ c) & 1) else c >> 1
        return c

    rest = o[tb:]
    total = good = 0
    i = 0
    while i < len(rest) - 86:
        if rest[i:i + 17].all() and rest[i + 17] == 0:
            f = rest[i:i + 86]
            body = [f[k] for k in range(18, 68) if k not in (34, 51)]
            field = sum(int(f[69 + k]) << k for k in range(16))
            ok = crc16(body) == field
            total += 1
            good += ok
            if total <= 2:
                print("MP at +%d: type %d drn %d ack %d mask 0x%04x crc %s"
                      % (i, f[18], sum(int(f[24 + k]) << k for k in range(4)),
                         f[33], sum(int(f[36 + k]) << k for k in range(13)),
                         "ok" if ok else "BAD"))
            i += 86
        else:
            i += 1
    print("MP frames %d, CRC valid %d" % (total, good))


if __name__ == "__main__":
    main()
