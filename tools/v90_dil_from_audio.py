#!/usr/bin/env python3
"""Turn an audio clip into a V.90 DIL training-Ucode ladder.

A DIL descriptor is, at heart, a list of training Ucodes: which PCM level the
digital modem should probe in each segment (§8.4.1). Nothing requires that
list to be a tidy ramp, so any sequence of levels will do -- including one
taken from the envelope of a sound clip.

This is mostly a toy, and it is also a genuinely awkward input for the
measurement path, which is what makes it useful: a clip with a narrow dynamic
range produces a ladder clustered in a few chords, and
`v90_dil_desc_validate()` should say so rather than the measurement quietly
reporting a small constellation later.

    tools/v90_dil_from_audio.py clip.wav --segments 60

Prints a C array ready for `v90_dil_desc_from_ucodes()`, plus what the ladder
covers. 8-bit and 16-bit PCM WAV, any sample rate, mono or stereo.
"""

import argparse
import struct
import sys
import wave


def read_samples(path):
    with wave.open(path, "rb") as w:
        n, width, chans = w.getnframes(), w.getsampwidth(), w.getnchannels()
        raw = w.readframes(n)
    if width == 1:                      # unsigned 8-bit
        vals = [(b - 128) * 256 for b in raw]
    elif width == 2:
        vals = list(struct.unpack("<%dh" % (len(raw) // 2), raw))
    else:
        raise SystemExit(f"{path}: {width * 8}-bit WAV not supported (use 8 or 16)")
    if chans > 1:                       # mix to mono
        vals = [sum(vals[i:i + chans]) // chans
                for i in range(0, len(vals) - chans + 1, chans)]
    return vals


def linear_to_ucode(v):
    """Nearest Ucode for a linear magnitude, via mu-law's own quantisation."""
    v = min(abs(int(v)), 32635)
    v += 132                            # mu-law bias
    exp = 7
    while exp > 0 and v < (1 << (exp + 7)):
        exp -= 1
    man = (v >> (exp + 3)) & 0x0F
    ucode = 0x7F - ((exp << 4) | man)
    return max(0, min(127, ucode))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("wav")
    ap.add_argument("--segments", type=int, default=60,
                    help="DIL segments to produce (default 60, max 255)")
    ap.add_argument("--hc", type=int, default=10,
                    help="Hc for every chord; segment length is (hc+1)*6 "
                         "(default 10 -> 66T)")
    args = ap.parse_args()

    if not 1 <= args.segments <= 255:
        raise SystemExit("--segments must be 1..255")

    samples = read_samples(args.wav)
    if not samples:
        raise SystemExit(f"{args.wav}: no audio")

    # One Ucode per segment, from the peak of that slice of the clip: peaks
    # track the envelope and keep the ladder spread out, where a mean would
    # collapse it toward the middle of the range.
    step = len(samples) / args.segments
    ladder = []
    for i in range(args.segments):
        lo = int(i * step)
        hi = max(lo + 1, int((i + 1) * step))
        peak = max(abs(s) for s in samples[lo:hi])
        u = linear_to_ucode(peak)
        ladder.append(max(1, u))        # Ucode 0 is REFc; it probes nothing

    print(f"/* DIL ladder from {args.wav}: {args.segments} segments, "
          f"Hc={args.hc} ({(args.hc + 1) * 6}T each, "
          f"{args.segments * (args.hc + 1) * 6 / 8.0:.0f} ms cycle) */")
    print("static const uint8_t dil_ladder[] = {")
    for i in range(0, len(ladder), 12):
        print("    " + " ".join(f"{u:3d}," for u in ladder[i:i + 12]))
    print("};")

    chords = sorted({u >> 4 for u in ladder})
    print(f"\n/* Ucodes {min(ladder)}..{max(ladder)}, "
          f"{len(set(ladder))} distinct, chords {chords}.", file=sys.stderr)
    if len(chords) < 3:
        print(" * Narrow: this clip's dynamic range leaves most of the ladder\n"
              " * unprobed, so the measurement will only see part of the range.",
              file=sys.stderr)
    print(" * Feed to v90_dil_desc_from_ucodes(), then v90_dil_desc_validate(). */",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
