#!/usr/bin/env python3
"""Design and independently measure V.56bis Table A.10/A.11 channel FIRs.

Standard library only. The input is the specified table cells, not SpanDSP's
disabled EDD generator. --check checks the committed coefficients too.
"""
from __future__ import annotations

import argparse
import re
import cmath
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "tools/v56bis/reference.json"
HEADER = ROOT / "v56bis_filters.h"
SIZE = 8192
TAPS = 513
CENTER = (TAPS - 1) // 2
FS = 8000


def fft(values: list[complex], inverse: bool = False) -> list[complex]:
    """Radix-2 FFT; explicit convention so phase/delay signs are reviewable."""
    a = list(values)
    n = len(a)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j ^= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    length = 2
    while length <= n:
        wlen = cmath.exp((2j if inverse else -2j) * math.pi / length)
        for start in range(0, n, length):
            w = 1 + 0j
            for k in range(length // 2):
                u, v = a[start+k], a[start+k+length//2] * w
                a[start+k], a[start+k+length//2] = u+v, u-v
                w *= wlen
        length *= 2
    return [v / n for v in a] if inverse else a


def curve(rows: list[list], column: int) -> list[tuple[float, float]]:
    return [(r[0], r[column+1]) for r in rows if r[column+1] is not None]


def interpolate(points: list[tuple[float, float]], f: float) -> float:
    if f <= points[0][0]:
        return points[0][1]
    for (f0, v0), (f1, v1) in zip(points, points[1:]):
        if f <= f1:
            return v0 + (v1-v0)*(f-f0)/(f1-f0)
    return points[-1][1]


def integral(points: list[tuple[float, float]], f: float) -> float:
    """Integral of piecewise-linear milliseconds against frequency in Hz."""
    p = [(0., points[0][1]), *points]
    total = 0.
    for (f0, v0), (f1, v1) in zip(p, p[1:]):
        upper = min(f, f1)
        if upper > f0:
            width = upper-f0
            total += v0*width + .5*(v1-v0)*width*width/(f1-f0)
        if f <= f1:
            return total
    return total + (f-p[-1][0])*p[-1][1]


def design(ad: list[tuple[float, float]], edd: list[tuple[float, float]]) -> list[float]:
    spectrum = [0j]*SIZE
    # Out-of-table engineering extensions: smoothly stop at DC/Nyquist.
    # AD is linear in dB between published knots. EDD is linear in ms;
    # phi(f)=-2*pi*integral(tau(f),df), NOT -2*pi*f*tau(f).
    for k in range(1, SIZE//2):
        f = k*FS/SIZE
        attenuation = interpolate(ad, f)
        taper = .5*(1-math.cos(math.pi*f/200)) if f < 200 else 1.
        if f > 3900:
            taper *= .5*(1+math.cos(math.pi*(f-3900)/100))
        phase = -2*math.pi*(integral(edd, f)/1000 + f*CENTER/FS)
        value = taper*10**(-attenuation/20)*cmath.exp(1j*phase)
        spectrum[k], spectrum[-k] = value, value.conjugate()
    impulse = fft(spectrum, inverse=True)
    # Rectangular truncation retains the designed phase. Normalize to the
    # spec's 1 kHz reference, not total impulse-response energy.
    h = [v.real for v in impulse[:TAPS]]
    gain = abs(sum(v*cmath.exp(-2j*math.pi*1000*i/FS) for i,v in enumerate(h)))
    # Match the actual committed C coefficients (double, 12 significant digits).
    return [float(f"{v/gain:.12g}") for v in h]


def response(h: list[float], f: float) -> tuple[float, float]:
    """Direct DTFT and analytic derivative; independent of synthesis FFT."""
    terms = [v*cmath.exp(-2j*math.pi*f*i/FS) for i,v in enumerate(h)]
    z = sum(terms)
    delay_ms = (sum(i*v for i,v in enumerate(terms))/z).real*1000/FS
    return -20*math.log10(abs(z)), delay_ms


def ad_tolerance(f: float) -> float:
    if f <= 300: return 2.
    if f <= 400: return 1.
    if f <= 3000: return .5
    if f <= 3300: return 1.
    if f <= 3600: return 2.
    return 3.


def validate(h: list[float], ad: list[tuple[float,float]], edd: list[tuple[float,float]]) -> dict:
    # Tables are relative to 1 kHz loss and 1.8 kHz delay. Check all specified
    # cells, including endpoints, plus a dense interpolated interior grid.
    gain_ref, _ = response(h, 1000)
    _, delay_ref = response(h, 1800)
    worst_ad = worst_edd = 0.
    failures = []
    freqs = sorted(set([f for f,_ in ad] + list(range(200,3901,25))))
    for f in freqs:
        loss,_ = response(h,f)
        delta = loss-gain_ref-interpolate(ad,f)
        worst_ad = max(worst_ad,abs(delta))
        if abs(delta) > ad_tolerance(f):
            failures.append(f"AD at {f} Hz: error {delta:.4f} dB")
    # Starred cells have no numeric target; do not validate our extensions as
    # if they were published values. A dense grid checks interior synthesis.
    freqs = sorted(set([f for f,_ in edd] + list(range(int(edd[0][0]),int(edd[-1][0])+1,25))))
    for f in freqs:
        _,delay = response(h,f)
        delta = delay-delay_ref-interpolate(edd,f)
        worst_edd = max(worst_edd,abs(delta))
        lo,hi = (-.2,.5) if f <= 400 or f >= 3200 else (-.1,.1)
        if not lo <= delta <= hi:
            failures.append(f"EDD at {f} Hz: error {delta:.4f} ms")
    return {"max_ad_error_db":worst_ad,"max_edd_error_ms":worst_edd,
            "reference_delay_samples":delay_ref*FS/1000,"failures":failures}


def same_header(a: str, b: str) -> bool:
    """Equal text, with every number allowed 1e-9 relative drift: the design
    uses floating point whose last digits differ between platforms/libm."""
    num=re.compile(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?")
    if num.sub("#",a)!=num.sub("#",b):
        return False
    xa=[float(x) for x in num.findall(a)]
    xb=[float(x) for x in num.findall(b)]
    return len(xa)==len(xb) and all(
        abs(x-y)<=1e-9*max(abs(x),abs(y))+1e-18 for x,y in zip(xa,xb))


def main() -> int:
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--check",action="store_true")
    p.add_argument("--report",type=Path)
    args=p.parse_args()
    ref=json.loads(REFERENCE.read_text())
    reports=[]
    output=["/* Generated by tools/v56bis_filters.py. Do not hand-edit.\n"
            " * ITU-T V.56bis (08/1995), Tables A.10/A.11; see reference.json.\n"
            " * EDD is integrated to phase; each FIR adds 256 samples nominal delay. */\n"
            "#ifndef V56BIS_FILTERS_H\n#define V56BIS_FILTERS_H\n"
            f"#define V56BIS_FILTER_TAPS {TAPS}\n"]
    for ai,aname in enumerate(ref['ad_names']):
        ad=curve(ref['ad_db'],ai)
        for ei,ename in enumerate(ref['edd_names']):
            edd=curve(ref['edd_ms'],ei)
            h=design(ad,edd)
            report={"ad":aname,"edd":ename,**validate(h,ad,edd)}
            reports.append(report)
            print(f"AD-{aname}/EDD-{ename}: AD error {report['max_ad_error_db']:.4f} dB; "
                  f"EDD error {report['max_edd_error_ms']:.4f} ms; failures={len(report['failures'])}")
            output.append(f"static const double v56bis_ad{aname}_edd{ename}[V56BIS_FILTER_TAPS] = {{\n")
            for i in range(0,len(h),4):
                output.append("    "+", ".join(f"{v:.12g}" for v in h[i:i+4])+",\n")
            output.append("};\n")
    output.append("static const int v56bis_ad_numbers[] = {1, 5, 6, 7, 8, 9};\n")
    output.append("static const double *const v56bis_filter_bank[6][3] = {\n")
    for aname in ref['ad_names']:
        output.append("    {"+", ".join(f"v56bis_ad{aname}_edd{e}" for e in ref['edd_names'])+"},\n")
    output.append("};\n#endif\n")
    if args.report:
        args.report.write_text(json.dumps(reports,indent=2)+"\n")
    if any(r['failures'] for r in reports):
        print("Filter calibration failed; header not written")
        return 1
    content="".join(output)
    if args.check:
        if not HEADER.is_file() or not same_header(HEADER.read_text(),content):
            print("Generated header is missing or stale")
            return 1
    else:
        HEADER.write_text(content)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
