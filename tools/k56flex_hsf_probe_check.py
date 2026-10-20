#!/usr/bin/env python3
"""Compare captured server PCMU with the recovered legacy HSF TRN1 references.

K56flex Draft 0.23 4.10: PCM probing. Reference equations come from original
K56.VXD 12022500/120225d0, called by 12022ec0/12023120; they do not import
our probe generator or tables. This checks transmitted probes, not reception,
carrier, upstream signalling or payload. Profile: normal-power mu-law.
"""
import argparse
import hashlib
import json
from pathlib import Path

P1_SAMPLES = 0x3ff0
P2_SAMPLES = 0x2100


def ulaw(octet):
    value = (~octet) & 255
    magnitude = (((value & 15) << 3) + 132) << ((value >> 4) & 7)
    return 132 - magnitude if value & 128 else magnitude - 132


def reference():
    # 12050460: 5/23, initial history 0, constant-one source. 12022500
    # chooses magnitude from the bit 23 positions behind the new PN bit.
    history, bits = 0, []
    for _ in range(P1_SAMPLES + 3*P2_SAMPLES):
        bit = (1 ^ (history >> 4) ^ (history >> 22)) & 1
        bits.append(bit)
        history = ((history << 1) | bit) & 0x7fffff
    p1 = []
    for i in range(P1_SAMPLES):
        old = bits[i - 23] if i >= 23 else 0
        magnitude = 4*(975 if old else 943)  # original 12009a54, mode 1
        p1.append(-magnitude if bits[i] else magnitude)
    # 120225d0 reads the delayed bit AFTER writing the first two PN bits,
    # hence k-21, not k-23. Odd symbols oppose running DC; it carries from P1.
    levels = (495, 527, 847, 879, 1023, 1087, 1023, 1087)  # 12009b34
    dc, p2 = sum(p1), []
    for i in range(P2_SAMPLES):
        k = P1_SAMPLES + 3*i
        index = sum(bits[k + j] << j for j in range(3))
        value = 4*levels[index]
        if (i % 2 == 0 and bits[k - 21]) or (i % 2 == 1 and dc > 0):
            value = -value
        p2.append(value)
        dc += value
    return p1, p2


def check(path):
    raw = path.read_bytes()
    pcm = [ulaw(octet) for octet in raw]
    # The identifying prefix uses different levels. Find the first complete
    # 100-sample P1 alphabet run without supplying the stage boundary.
    run, start = 0, None
    for i, value in enumerate(pcm):
        run = run + 1 if abs(value) in (3772, 3900) else 0
        if run == 100:
            start = i - 99
            break
    result = {"capture": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
              "p1_start_sample": start, "verified_connection": False}
    if start is None:
        result.update(passed=False, reason="no P1 alphabet run")
        return result
    offset = start
    for name, expected in zip(("p1", "p2"), reference()):
        actual = pcm[offset:offset + len(expected)]
        bad = [i for i, (a, b) in enumerate(zip(actual, expected)) if a != b]
        result[name] = {"samples": len(actual), "required": len(expected),
                        "mismatches": len(bad), "first_mismatch": bad[0] if bad else None,
                        "negative": sum(x < 0 for x in actual),
                        "positive": sum(x > 0 for x in actual),
                        "zero": sum(x == 0 for x in actual)}
        offset += len(expected)
    result["passed"] = all(result[name]["samples"] == result[name]["required"]
                           and result[name]["mismatches"] == 0 for name in ("p1", "p2"))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    result = check(args.capture)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
