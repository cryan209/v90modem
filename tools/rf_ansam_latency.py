#!/usr/bin/env python3
"""How long the far end holds ANSam before answering — its CM-detection latency.

V.8 has the answering modem stop ANSam once it has detected CM, so the length
of the 2100 Hz burst in the receive tap is a direct, continuous measure of how
quickly the peer heard us.  Measured against the RasFinder on 2026-09-29 it
separates the outcomes cleanly: 2.1-3.2 s on the calls whose V.8 succeeded,
5.3-5.5 s on four that failed, every one of those then having a pure 2250 Hz
tone replace JM at 10.9 s.  Unlike a pass/fail count it yields a number from
every call, which is what makes a small A/B on the transmit level readable.

Usage: rf_ansam_latency.py <dir-of-call-dirs-or-a-call-dir> ...

NumPy is required and the system python3 on this host does not have it, so run
this from a venv.  A soak script that pipes this through a bare `python3` with
stderr discarded prints nothing and looks like a metric that did not fire --
the taps are still there, so score afterwards rather than re-running the calls.
"""
import sys, pathlib, numpy as np

FS = 8000
BLK = 800  # 100 ms


def ulaw(raw):
    b = np.frombuffer(raw, dtype=np.uint8).astype(np.int32) ^ 0xFF
    sign, exp, man = b & 0x80, (b >> 4) & 7, b & 0xF
    mag = ((man * 2 + 33) << exp) - 33
    return np.where(sign != 0, -mag, mag).astype(np.float64)


def tone_run(x, freq, min_frac=0.7, min_rms=300.0):
    n = len(x) // BLK
    if n == 0:
        return None
    blk = x[: n * BLK].reshape(n, BLK)
    k = 2 * np.pi * freq / FS
    idx = np.arange(BLK)
    g = (blk @ np.cos(k * idx)) ** 2 + (blk @ np.sin(k * idx)) ** 2
    tot = (blk ** 2).sum(1) + 1e-9
    frac = 2 * g / (tot * BLK)
    rms = np.sqrt((blk ** 2).mean(1))
    hit = [i for i in range(n) if frac[i] > min_frac and rms[i] > min_rms]
    if not hit:
        return None
    return hit[0] * BLK / FS, hit[-1] * BLK / FS


def report(d):
    tap = d / "live-rx.g711"
    if not tap.is_file() or tap.stat().st_size == 0:
        print(f"{d.name:<18} (no receive tap)")
        return
    rx = ulaw(tap.read_bytes())
    ans = tone_run(rx, 2100)
    hijack = tone_run(rx, 2250, min_frac=0.6, min_rms=200.0)
    log = d / "server.log"
    status = ""
    if log.is_file():
        for line in log.read_text(errors="replace").splitlines():
            if "V.8 result: status=" in line:
                status = line.split("status=")[1].split(",")[0]
                break
    if ans is None:
        print(f"{d.name:<18} no ANSam found          v8_status={status or '-'}")
        return
    a, b = ans
    print(f"{d.name:<18} ANSam {a:5.1f}-{b:5.1f}s = {b - a + 0.1:4.1f}s"
          f"   2250Hz {'at %.1fs' % hijack[0] if hijack else 'absent  '}"
          f"   v8_status={status or '-'}")


def main(paths):
    for p in paths:
        p = pathlib.Path(p)
        kids = sorted(c for c in p.iterdir() if c.is_dir()) if p.is_dir() else []
        if kids and any((c / "live-rx.g711").exists() for c in kids):
            print(f"== {p}")
            for c in kids:
                report(c)
        else:
            report(p)


if __name__ == "__main__":
    main(sys.argv[1:] or ["."])
