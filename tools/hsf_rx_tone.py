#!/usr/bin/env python3
"""Is the HSF RX stream real line audio, and does our TX reach the line?

Every health check in this investigation counts bytes.  None of them checks
that the bytes are audio, so "RX works" has been an assumption throughout.

Reads a --rx-out capture as 4-byte frames (one signed 16-bit sample plus a
second slot, per the 5b 01 00 00 pattern), and runs a Goertzel over the tones
that settle it:

  350 + 440 Hz   dial tone -- present only if the DAA is really off-hook on a
                 live line, so this is what proves the analogue path exists
  697 + 1336 Hz  DTMF '5', the digit the probe transmits -- if this appears in
                 RECEIVE while we are transmitting it, our TX reached the line,
                 whatever the FIFO byte counter says
"""
import sys, math, struct

RATE = 10666.667          # provisional; the notes derive it from the byte rate

def goertzel(x, f, rate=RATE):
    w = 2.0 * math.pi * f / rate
    c = 2.0 * math.cos(w)
    s1 = s2 = 0.0
    for v in x:
        s0 = v + c * s1 - s2
        s2, s1 = s1, s0
    return math.sqrt(max(s1*s1 + s2*s2 - c*s1*s2, 0.0)) / max(len(x), 1)

def load(path, slot=0):
    """The capture is a plain signed-16 LE stream.  It is NOT four-byte frames:
    a capture reads b702 ba02 c402 ... = 695, 698, 708, consecutive samples.
    The 5b 01 00 00 pattern in the notes came from an earlier build and taking
    it as the frame layout made this tool read every other sample."""
    d = open(path, "rb").read()
    n = len(d) // 2
    return list(struct.unpack("<%dh" % n, d[:n * 2]))

def main():
    for path in sys.argv[1:]:
        x = load(path)
        if not x:
            print(f"{path}: empty"); continue
        dc = sum(x) / len(x)
        x = [v - dc for v in x]
        rms = math.sqrt(sum(v * v for v in x) / len(x))
        tones = {"350": 350, "440": 440, "697": 697, "1336": 1336,
                 "1000(ref)": 1000, "2500(ref)": 2500}
        mags = {k: goertzel(x, f) for k, f in tones.items()}
        floor = max(mags["1000(ref)"], mags["2500(ref)"], 1e-9)
        print(f"{path}: {len(x)} frames, DC {dc:8.1f}, RMS {rms:8.1f}")
        for k in ("350", "440", "697", "1336"):
            print(f"    {k:>4} Hz  {mags[k]:9.2f}  ({mags[k]/floor:6.2f}x the "
                  f"off-tone floor)")
main()
