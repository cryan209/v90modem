#!/usr/bin/env python3
"""List the MNP (V.42 Annex A) frames in a DS_TX_BIT_DUMP / DS_RX_BIT_DUMP.

Receiver-independent: it reads the bit stream the data stack saw (one ASCII
'0'/'1' per bit) and finds both framings itself -- start-stop octet-oriented
frames (A.3: SYN DLE STX ... DLE ETX, CRC-16 x^16+x^15+x^2+1) and
bit-oriented ones (A.4: flags, zero insertion, the 8.1.1.6.1 FCS) -- so it
shows what the peer actually sent, not what mnp.c made of it.

    tools/mnp_frames.py rx-bits.txt 2400

Each line: bit offset, seconds at the given line rate, A(sync)/S(ync), FCS
ok/BAD, frame type, LI, the header after LI and type, and the start of any
LT data.  Merge a TX and an RX listing by time to read an exchange; the two
dumps start at data-mode entry, so their clocks agree to a frame or so.
"""
import sys

NAMES = {1: 'LR', 2: 'LD', 4: 'LT', 5: 'LA', 6: 'LN', 7: 'LNA'}


def crc16_arc(data, crc=0):
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def fcs16_ok(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc == 0xF0B8


def describe(f):
    if len(f) < 2:
        return f.hex()
    li, t = f[0], f[1]
    s = '%s li=%d hdr=%s' % (NAMES.get(t, '?%d' % t), li, f[2:li + 1].hex())
    data = f[li + 1:]
    if data:
        s += ' data(%d)=%r' % (len(data), bytes(data[:64]))
    return s


def async_frames(bits):
    octs, i = [], 0
    while i + 10 <= len(bits):
        if bits[i] == 0:
            octs.append((i, sum(bits[i + 1 + k] << k for k in range(8))))
            i += 10
        else:
            i += 1
    out, j = [], 0
    while j < len(octs) - 3:
        if [o[1] for o in octs[j:j + 3]] == [0x16, 0x10, 0x02]:
            body, k = [], j + 3
            while k < len(octs) - 3:
                c = octs[k][1]
                if c == 0x10 and octs[k + 1][1] == 0x10:
                    body.append(0x10)
                    k += 2
                    continue
                if c == 0x10 and octs[k + 1][1] == 0x03:
                    crc = octs[k + 2][1] | (octs[k + 3][1] << 8)
                    out.append((octs[j][0], 'A', crc16_arc(bytes(body) + b'\x03') == crc, bytes(body)))
                    break
                body.append(c)
                k += 1
            j = k
        j += 1
    return out


def sync_frames(bits):
    out, ones, start, fbits = [], 0, 0, []
    for i, b in enumerate(bits):
        if b:
            ones += 1
            fbits.append(1)
            continue
        if ones == 6:                   # a flag: drop its leading 0 and six 1s
            fbits = fbits[:-7]
            if len(fbits) >= 32 and len(fbits) % 8 == 0:
                octs = bytes(sum(fbits[n * 8 + k] << k for k in range(8))
                             for n in range(len(fbits) // 8))
                out.append((start, 'S', fcs16_ok(octs), octs[:-2]))
            fbits, start = [], i + 1
        elif ones > 6:                  # abort / idle
            fbits, start = [], i + 1
        elif ones != 5:                 # 5 ones then 0 is a stuffed bit
            fbits.append(0)
        ones = 0
    return out


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    bits = [1 if c == '1' else 0 for c in open(sys.argv[1]).read() if c in '01']
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 2400.0
    for pos, kind, ok, body in sorted(async_frames(bits) + sync_frames(bits)):
        print('%9d %8.3fs %s %s %s' % (pos, pos / rate, kind, 'ok ' if ok else 'BAD', describe(body)))


if __name__ == '__main__':
    main()
