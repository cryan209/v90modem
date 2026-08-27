#!/usr/bin/env python3
"""Check OUR OWN transmitted Rd against V.90 §9.6.1.1, out of a live TX tap.

Why this exists.  ME_V90_RENEG is default off because "this rig's analogue
modem answers 384T of Rd with nothing at all".  That was inferred from two
calls in which the peer retrained and its log said SILENCERETRAIN -- which is
the name of the state where the peer TRANSMITS silence before its own Tone A,
not a report that it heard silence from us.  Before believing the peer is at
fault, demodulate what we actually put on the DS0 and check it against the
clause.  That method -- read your own tap against the spec, not either
modem's log -- is what found the S-bar rotation, the receive-band notch and
the echo canceller in this project.

§9.6.1.1 / §8.6.4: Rd is the six-symbol sign pattern + + + - - - repeated at
the U_INFO magnitude, unscrambled and not differentially encoded, for exactly
384T; the barred Rd that terminates it is four repetitions of - - - + + +
(24T).  The downstream is the DS0 itself, so one symbol is one G.711
codeword: 384T = 384 bytes = 48 ms.

  v90_rd_verify.py <live-tx.g711> [ulaw|alaw]
"""
import sys, collections

def ucode_of(law, b):
    """Return (sign, ucode) for a G.711 codeword as this modem composes it:
    bit 7 is polarity, the low 7 bits are the Ucode's positive PCM code."""
    sign = 1 if (b & 0x80) else 0
    pos = b & 0x7F
    if law == 'ulaw':
        # ucode_to_pcm_positive: pcm = 0xFF - ucode, then masked to 7 bits
        return sign, (0xFF - pos) & 0x7F
    return sign, ALAW_TO_UCODE.get(pos, -1)

# a-law table is built lazily from the same mapping the modem uses
ALAW_TO_UCODE = {}

def load(path):
    with open(path, 'rb') as f:
        return f.read()

RD_PATTERN = [1,1,1,0,0,0]

def scan(data, law):
    """Find maximal runs where sign follows +++--- at one constant magnitude."""
    n = len(data)
    runs = []
    i = 0
    while i < n:
        s0, u0 = ucode_of(law, data[i])
        # try each of the six phases; a run must start on a pattern boundary
        best = 0
        for ph in range(6):
            j = i
            while j < n:
                s, u = ucode_of(law, data[j])
                if u != u0:
                    break
                if s != RD_PATTERN[(j - i + ph) % 6]:
                    break
                j += 1
            best = max(best, j - i)
        if best >= 24:
            runs.append((i, best, u0))
            i += best
        else:
            i += 1
    return runs

def main():
    if len(sys.argv) < 2:
        print(__doc__); return 2
    path = sys.argv[1]
    law = sys.argv[2] if len(sys.argv) > 2 else 'ulaw'
    data = load(path)
    print(f"{path}: {len(data)} codewords ({len(data)/8000.0:.1f} s of DS0), {law}")

    runs = scan(data, law)
    if not runs:
        print("\nNo +++--- run of 24T or more anywhere in this tap.")
        print("=> We never transmitted anything with Rd's structure.")
        return 1

    print(f"\n{len(runs)} run(s) of the §8.6.4 +++--- structure:\n")
    print(f"  {'at (s)':>9}  {'length':>7}  {'Ucode':>5}  verdict")
    for off, ln, u in runs:
        t = off / 8000.0
        if ln >= 384:
            v = "Rd (>=384T per §9.6.1.1)"
        elif ln >= 192:
            v = "Ri (§9.4.1.1 startup, >=192T)"
        else:
            v = f"short -- {ln}T"
        print(f"  {t:9.3f}  {ln:6d}T  {u:5d}  {v}")

    longest = max(r[1] for r in runs)
    print()
    if longest >= 384:
        print("VERDICT: a conformant 384T Rd IS present in our transmit tap.")
        print("         If the peer did not answer, the fault is not the")
        print("         duration or the sign pattern -- look at the level")
        print("         (Ucode above) and at what follows the barred Rd.")
    else:
        print("VERDICT: NO 384T Rd in our transmit tap -- the longest run is")
        print(f"         {longest}T.  §9.6.1.1 requires 384T.  The peer was")
        print("         never given the signal it is required to detect, so")
        print("         'the peer does not answer Rd' is not established by")
        print("         any call whose tap looks like this.")
    return 0

sys.exit(main())
