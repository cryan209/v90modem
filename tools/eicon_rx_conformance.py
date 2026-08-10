#!/usr/bin/env python3
"""Receive-path conformance against a downstream we did not generate.

Every other receive test in this tree is fed by our own transmitter --
`v90_dil_generate_codewords`, `v90_tx_codewords`, `v90_tx_data_frame_codewords`,
`v90_generate_trn2d_codewords` in `vpcm_loopback_test.c`. A convention error
that is wrong but *self-consistent* -- a scrambler seed, tap or polarity that
our transmitter and receiver both get the same way -- is invisible to that
harness by construction: both ends agree, the test goes green, and the peer
still fails.

`artifacts/eicon-digital-downstream/` holds DS0 captures of the Eicon card's
own shipped V.90 firmware transmitting to a USR Courier that connected. This
harness points our analogue-side receive path at them.

Each fixture is checked twice, and the order matters:

  1. POSITIVE CONTROL -- a codeword-exact DIL run and an Sd/S-bar boundary must
     be recovered. These prove codeword recovery is working and the harness is
     actually armed. A watch that never fires and a watch that was never armed
     look identical; this is what tells them apart. If the control fails, the
     harness is broken and the result below means nothing.

  2. THE ASSERTION -- at least 48 TRN1d symbols must be descrambled, which is
     what the decoder itself states it needs for a full decode.

Step 2 FAILS TODAY. That is the correct initial state, and the point of the
test: it is the only check in the tree that can catch this class of defect at
all. See docs/eicon_downstream_comparison.md, Finding 2.

The cause is *not* a scrambler disagreement -- the card's TRN1d is GPC,
zero-initialized, and matches the Recommendation bit-for-bit (Finding 1). Our
decoder mislabels the card's constant-magnitude TRN1d as "Sd" and then searches
for TRN1d after it, where there is none.

Caveat on the control: its Sd arm currently accepts that mislabel, because it
only asks whether *an* Sd was reported. §8.4.4 makes Sd one-third Ucode 0, so it
should additionally require Ucode-0 density near 1/3 -- which the card's TRN1d
would fail, as it should. Until then the Sd arm proves less than it appears to;
the DIL arm is what genuinely establishes that codeword recovery works.

Not wired into `make test` -- it encodes a known-open defect, and a suite that
is red by default stops being read. Run it deliberately:

    make eicon-rx-test
"""

import argparse
import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
FIXTURE_DIR = REPO / "artifacts" / "eicon-digital-downstream"

# The decoder states this threshold itself: "need >=48 for full decode".
MIN_TRN1D_SYMBOLS = 48

TRN1D_RE = re.compile(r"V\.90 TRN1d[^:]*:\s*(\d+)\s*symbols")
DESCRAMBLER_FAILED_RE = re.compile(r"sign-bit descrambler found (\d+) TRN1d symbols")
DIL_RUN_RE = re.compile(r"Codeword-exact DIL run:\s*(\S+)\s*-\s*(\S+)\s*ms")
SD_RE = re.compile(r"V\.90 Sd[^:]*:.*?(\d+) reps")


def decode(binary, path):
    proc = subprocess.run(
        [str(binary), "--g711", "--law", "ulaw", "--v90", "--dil-scan", str(path)],
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        raise SystemExit(
            f"{binary} exited {proc.returncode} on {path.name}:\n{proc.stderr}"
        )
    return proc.stdout


def check(binary, path):
    """Return (control_ok, assertion_ok, notes)."""
    out = decode(binary, path)
    notes = []

    dil = DIL_RUN_RE.search(out)
    sd = SD_RE.search(out)
    control_ok = bool(dil) and bool(sd)

    if dil:
        notes.append(f"control: codeword-exact DIL run {dil.group(1)}-{dil.group(2)} ms")
    else:
        notes.append("control: NO codeword-exact DIL run -- codeword recovery is broken")
    if sd:
        notes.append(f"control: Sd recovered, {sd.group(1)} reps")
    else:
        notes.append("control: NO Sd recovered")

    trn = TRN1D_RE.search(out)
    if trn:
        count = int(trn.group(1))
        assertion_ok = count >= MIN_TRN1D_SYMBOLS
        notes.append(f"TRN1d: {count} symbols descrambled")
    else:
        failed = DESCRAMBLER_FAILED_RE.search(out)
        count = int(failed.group(1)) if failed else 0
        assertion_ok = False
        notes.append(f"TRN1d: {count} symbols descrambled (need >={MIN_TRN1D_SYMBOLS})")

    return control_ok, assertion_ok, notes


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--binary", default=str(REPO / "vpcm_decode"),
                    help="path to vpcm_decode (default: ./vpcm_decode)")
    ap.add_argument("--expect-failure", action="store_true",
                    help="exit 0 while the known defect stands, but fail loudly "
                         "if the positive control breaks or the defect is fixed")
    args = ap.parse_args()

    # resolve(): pathlib drops a leading "./", which would leave subprocess with
    # a bare name and send it to a PATH lookup instead of the local build.
    binary = pathlib.Path(args.binary).resolve()
    if not binary.exists():
        raise SystemExit(f"{binary} not found -- run `make vpcm_decode` first")

    fixtures = sorted(FIXTURE_DIR.glob("*.ulaw"))
    if not fixtures:
        raise SystemExit(f"no fixtures in {FIXTURE_DIR}")

    control_failures = []
    assertion_failures = []

    for path in fixtures:
        control_ok, assertion_ok, notes = check(binary, path)
        status = "PASS" if (control_ok and assertion_ok) else "FAIL"
        print(f"[{status}] {path.name}")
        for note in notes:
            print(f"         {note}")
        if not control_ok:
            control_failures.append(path.name)
        if not assertion_ok:
            assertion_failures.append(path.name)

    print()
    if control_failures:
        print("POSITIVE CONTROL FAILED on: " + ", ".join(control_failures))
        print("The harness is not armed. The TRN1d result above means nothing --")
        print("fix codeword recovery before reading it.")
        return 2

    if not assertion_failures:
        print("All fixtures decoded. If this is the first green run, the")
        print("convention defect in docs/eicon_downstream_comparison.md is fixed --")
        print("update that doc and move this into `make test`.")
        return 1 if args.expect_failure else 0

    print("TRN1d recovery failed on: " + ", ".join(assertion_failures))
    print("Positive control passed, so codeword recovery works and this is a real")
    print("receive-path failure.  It is NOT a scrambler disagreement: the card's")
    print("TRN1d is GPC, zero-initialized, and matches the Recommendation")
    print("bit-for-bit.  We mislabel that constant-magnitude TRN1d as Sd and then")
    print("search for TRN1d after it, where there is none.")
    print("Known-open defect: docs/eicon_downstream_comparison.md, Finding 2.")
    return 0 if args.expect_failure else 1


if __name__ == "__main__":
    sys.exit(main())
