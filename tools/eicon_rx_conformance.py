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

Each fixture is checked in two parts, and the order matters:

  1. PHASE 3 CHAIN -- Sd, S-bar_d, TRN1d and Jd must be recovered at the
     symbol offsets an independent segmentation of the capture puts them at.
     These are exact expected values, not thresholds: they were derived from
     the fixtures without using our decoder (mu-law Ucode decomposition plus
     the 5.3 GPC generator) and then reproduced by it. Any drift is a
     regression.

  2. THE ASSERTION -- the codeword-exact DIL decoder must recover the DIL that
     starts where Jd ends.

Step 2 FAILS TODAY, and step 1 is what makes that failure meaningful: the
Phase 3 chain landing exactly right proves codeword recovery works, so a DIL
no-decode is a real receive-path gap and not a broken harness.

The cause is structural, in `v90_dil_rx.c`. That decoder recovers DIL by
finding an exactly-periodic run and fitting a descriptor to one cycle. But
8.4.1 gives each of the N DIL-segments its own training Ucode, so a real DIL
is not periodic at the segment scale -- only at the full N-segment cycle. The
card sends that cycle roughly once (~15.7 kT, 1.97 s, 132T segments) before
the Courier terminates it, so there is no second cycle to lock onto, and the
scan degenerates to fitting short accidental fragments where adjacent segments
happen to share a Ucode. Our own transmitter repeats DIL cycles, which is why
no loopback test sees this.

See docs/eicon_downstream_comparison.md, Finding 4.

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

SD_RE    = re.compile(r"V\.90 Sd: W_UCODE=(\d+) \(U_INFO=(\d+)\), (\d+) reps")
SBAR_RE  = re.compile(r"V\.90 S̄d: (\d+) reps")
TRN1D_RE = re.compile(r"V\.90 TRN1d: (\d+) symbols .* descrambled to ones")
JD_RE    = re.compile(r"V\.90 Jd\+J'd: (\d+) symbols")
DIL_RE   = re.compile(r"Codeword-exact DIL run:\s*(\S+)\s*-\s*(\S+)\s*ms")

# Independently derived from the captures; see the module docstring.  8.4.4
# fixes Sd at 64 reps and S-bar_d at 8, and both calls spend 30005T on TRN1d --
# 3750.6 ms, 94% of the 9.3.1.5 budget.
#
# dil_start_ms / dil_len_ms bound the region between the end of Jd and the
# start of Ri -- the constant-U_INFO run that follows DIL -- again measured
# without our decoder.
EXPECTED = {
    "call1-connect-32000.ulaw": {
        "u_info": 48, "w_ucode": 64,
        "sd_reps": 64, "sbar_reps": 8,
        "trn1d_symbols": 30005, "jd_symbols": 943,
        "dil_start_ms": 12289.4, "dil_len_ms": 1971.8,
    },
    "call3-connect-42666.ulaw": {
        "u_info": 48, "w_ucode": 64,
        "sd_reps": 64, "sbar_reps": 8,
        "trn1d_symbols": 30005, "jd_symbols": 1015,
        "dil_start_ms": 12337.9, "dil_len_ms": 1980.0,
    },
}

# How close a claimed DIL run has to come to the real one to count.  Generous
# on purpose: the point is to reject fragments, not to pin down a boundary.
DIL_START_TOLERANCE_MS = 50.0
DIL_MIN_COVERAGE = 0.80


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
    """Return (chain_ok, assertion_ok, notes)."""
    out = decode(binary, path)
    want = EXPECTED.get(path.name)
    notes = []

    if want is None:
        return False, False, [f"no expected values recorded for {path.name}"]

    chain_ok = True

    def expect(label, got, exp):
        nonlocal chain_ok
        if got == exp:
            notes.append(f"chain: {label} = {got}")
        else:
            chain_ok = False
            notes.append(f"chain: {label} = {got}, EXPECTED {exp}")

    sd = SD_RE.search(out)
    if sd:
        expect("W_UCODE", int(sd.group(1)), want["w_ucode"])
        expect("U_INFO", int(sd.group(2)), want["u_info"])
        expect("Sd reps", int(sd.group(3)), want["sd_reps"])
    else:
        chain_ok = False
        notes.append("chain: NO Sd recovered")

    sbar = SBAR_RE.search(out)
    if sbar:
        expect("S̄d reps", int(sbar.group(1)), want["sbar_reps"])
    else:
        chain_ok = False
        notes.append("chain: NO S̄d recovered")

    trn = TRN1D_RE.search(out)
    if trn:
        expect("TRN1d symbols", int(trn.group(1)), want["trn1d_symbols"])
    else:
        chain_ok = False
        notes.append("chain: NO descrambled TRN1d -- sign recovery is broken")

    jd = JD_RE.search(out)
    if jd:
        expect("Jd+J'd symbols", int(jd.group(1)), want["jd_symbols"])
    else:
        chain_ok = False
        notes.append("chain: NO Jd recovered")

    #
    # A decode only counts if it lands on the DIL region.  Without this the
    # arm passes on debris: on call3 the scan currently reports a 35 ms,
    # 280-symbol fragment from the middle of DIL, which is an accidental
    # period-132 match between two adjacent segments, not a recovered DIL.
    dil = DIL_RE.search(out)
    if dil:
        got_start = float(dil.group(1))
        got_len = float(dil.group(2)) - got_start
        coverage = got_len / want["dil_len_ms"]
        near_start = abs(got_start - want["dil_start_ms"]) <= DIL_START_TOLERANCE_MS
        assertion_ok = near_start and coverage >= DIL_MIN_COVERAGE
        notes.append(
            f"DIL: run {got_start:.1f}-{float(dil.group(2)):.1f} ms "
            f"({got_len:.1f} ms, {100.0 * coverage:.0f}% of the region; "
            f"region starts {want['dil_start_ms']:.1f} ms)"
        )
        if not assertion_ok:
            notes.append("DIL: rejected -- a fragment, not the DIL region")
    else:
        assertion_ok = False
        notes.append("DIL: no codeword-exact run recovered")

    return chain_ok, assertion_ok, notes


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--binary", default=str(REPO / "vpcm_decode"),
                    help="path to vpcm_decode (default: ./vpcm_decode)")
    ap.add_argument("--expect-failure", action="store_true",
                    help="exit 0 while the known-open DIL defect stands, but "
                         "fail loudly if the Phase 3 chain breaks or the defect "
                         "is fixed")
    args = ap.parse_args()

    # resolve(): pathlib drops a leading "./", which would leave subprocess with
    # a bare name and send it to a PATH lookup instead of the local build.
    binary = pathlib.Path(args.binary).resolve()
    if not binary.exists():
        raise SystemExit(f"{binary} not found -- run `make vpcm_decode` first")

    fixtures = sorted(FIXTURE_DIR.glob("*.ulaw"))
    if not fixtures:
        raise SystemExit(f"no fixtures in {FIXTURE_DIR}")

    chain_failures = []
    assertion_failures = []

    for path in fixtures:
        chain_ok, assertion_ok, notes = check(binary, path)
        status = "PASS" if (chain_ok and assertion_ok) else "FAIL"
        print(f"[{status}] {path.name}")
        for note in notes:
            print(f"         {note}")
        if not chain_ok:
            chain_failures.append(path.name)
        if not assertion_ok:
            assertion_failures.append(path.name)

    print()
    if chain_failures:
        print("PHASE 3 CHAIN FAILED on: " + ", ".join(chain_failures))
        print("This is a regression: these offsets were reproduced from the")
        print("captures independently of our decoder.  Fix the chain before")
        print("reading the DIL result above.")
        return 2

    if not assertion_failures:
        print("All fixtures decoded, DIL included.  If this is the first green")
        print("run, the defect in docs/eicon_downstream_comparison.md Finding 4")
        print("is fixed -- update that doc and move this into `make test`.")
        return 1 if args.expect_failure else 0

    print("DIL recovery failed on: " + ", ".join(assertion_failures))
    print("The Phase 3 chain landed on its expected offsets, so codeword")
    print("recovery works and this is a real receive-path gap: v90_dil_rx.c")
    print("locks onto an exactly-periodic run, and 8.4.1 gives every")
    print("DIL-segment its own training Ucode, so a real one-pass DIL is")
    print("periodic only at the full N-segment cycle -- which the card sends")
    print("about once before the peer terminates it.")
    print("Known-open defect: docs/eicon_downstream_comparison.md, Finding 4.")
    return 0 if args.expect_failure else 1


if __name__ == "__main__":
    sys.exit(main())
