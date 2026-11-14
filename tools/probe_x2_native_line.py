#!/usr/bin/env python3
"""The native x2 control: Ie030002 (server) and the Courier 403 through a line.

courier-emu's tools/probe_imodem_analog_pair.py joins the two original
firmwares over its "digital line", which hands the I-modem's codewords to the
Courier's codec model one for one.  This wrapper runs that tool unchanged but
filters the I-modem's downstream on the way: the same static impairments
tools/probe_x2_host_courier.py puts between OUR server and the same Courier
(--clock-phase, --codec-cutoff, --codec-highpass), so the two servers can be
compared on one line.  Everything after `--` goes to the courier-emu tool.

  probe_x2_native_line.py [--phase 0.5] [--cutoff 3900] [--highpass 250]
                          [--ppm 10] [--linear]
                          [--save-downstream FILE] -- <pair tool arguments>

--ppm resamples the downstream so the network clock runs that many ppm fast
of the Courier (as --clock-ppm --line-down-only does for our server), and
--linear hands the Courier linear samples instead of requantising to PCMU.

--save-downstream keeps the I-modem's downstream PCMU as it left the server
(before the line), for a sample-level comparison with our own.
"""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
EMU = Path(os.environ.get('X2_HOST_EMU_ROOT', ROOT.parent / 'courier-emu'))


def line_filter(phase, cutoff, highpass, half=63):
    """One FIR for the whole line: fractional delay, low-pass, high-pass."""
    def sinc_taps(fc, shift):
        out = []
        for k in range(-half, half + 1):
            x = k - shift
            weight = 1.0 if abs(x) < 1e-9 else math.sin(math.pi * 2 * fc * x) / (math.pi * 2 * fc * x)
            out.append(2 * fc * weight * (0.54 + 0.46 * math.cos(math.pi * x / (half + 1))))
        return out
    taps = sinc_taps((cutoff or 4000) / 8000, phase)
    if highpass:
        taps = [a - b for a, b in zip(taps, sinc_taps(highpass / 8000, phase))]
    return taps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--phase', type=float, default=0.0)
    ap.add_argument('--cutoff', type=float, default=0.0)
    ap.add_argument('--highpass', type=float, default=0.0)
    ap.add_argument('--ppm', type=float, default=0.0)
    ap.add_argument('--linear', action='store_true')
    ap.add_argument('--save-downstream', type=Path)
    ap.add_argument('rest', nargs=argparse.REMAINDER)
    a = ap.parse_args()
    rest = a.rest[1:] if a.rest[:1] == ['--'] else a.rest

    os.environ.setdefault('COURIER_LINE_FRAME_MS', '20')
    sys.path.insert(0, str(EMU))
    spec = importlib.util.spec_from_file_location('native_pair', EMU / 'tools/probe_imodem_analog_pair.py')
    pair = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pair)
    from courier_emu.sip import linear_to_ulaw, ulaw_to_linear

    impaired = a.phase or a.cutoff or a.highpass
    taps = line_filter(a.phase, a.cutoff, a.highpass) if impaired else None
    saved = a.save_downstream.open('wb') if a.save_downstream else None
    ratio = 1 + a.ppm * 1e-6
    half = 96
    if a.linear:
        import courier_emu.bearer_line as bearer_line
        decode = bearer_line.ulaw_to_linear
        bearer_line.ulaw_to_linear = lambda v: v if isinstance(v, float) else decode(v)

    def interpolate(history, position):
        base = int(position)
        total = 0.0
        for k in range(-half + 1, half + 1):
            x = k - (position - base)
            weight = 1.0 if abs(x) < 1e-9 else math.sin(math.pi * x) / (math.pi * x)
            total += history[base + k] * weight * (0.54 + 0.46 * math.cos(math.pi * x / half))
        return total

    class LineBearer(pair.BearerLineLink):
        def _exchange(self, dsc, channel, octets):
            if saved:
                saved.write(octets)
            if not (taps or a.ppm or a.linear):
                return super()._exchange(dsc, channel, octets)
            if not hasattr(self, 'line_state'):
                self.line_state = [0.0] * len(taps or [0])
                self.down_line = [0.0] * (2 * half + 400)
                self.down_at = float(half)
            for v in octets:
                y = float(ulaw_to_linear(v))
                if taps:
                    self.line_state.append(y)
                    del self.line_state[0]
                    y = sum(x * t for x, t in zip(self.line_state, taps))
                self.down_line.append(y)
            out = []
            for _ in octets:
                out.append(interpolate(self.down_line, self.down_at) if a.ppm else self.down_line[int(self.down_at)])
                self.down_at += ratio
            drop = int(self.down_at) - 4 * half
            if drop > 4096:
                del self.down_line[:drop]
                self.down_at -= drop
            if not a.linear:
                out = bytes(linear_to_ulaw(max(-32768, min(32767, round(y)))) for y in out)
            return super()._exchange(dsc, channel, out)

    pair.BearerLineLink = LineBearer
    sys.argv = [str(EMU / 'tools/probe_imodem_analog_pair.py'), *rest]
    try:
        return pair.main()
    finally:
        if saved:
            saved.close()


if __name__ == '__main__':
    raise SystemExit(main())
