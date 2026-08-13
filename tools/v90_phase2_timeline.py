#!/usr/bin/env python3
"""Label a V.90 Phase 2 capture: what was on the line, and what stage we were in.

The RX dump (/tmp/v34_rx.raw, s16le 8 kHz mono) is opened once per process and
appended across every call, with no timestamps of its own.  Classifying it
spectrally says which tones are present but not which handshake stage was
running, and that pairing is the whole question -- "the peer is sending Tone A
at the deadline" is a claim about both at once.  modem_engine.c emits

    [ME] RX dump mark: sample=N rx_stage=R tx_stage=T mod=M

every 100 ms of dumped audio, which is what ties the two together.

Usage:
  v90_phase2_timeline.py <log> [--audio FILE] [--from S] [--to S]
  v90_phase2_timeline.py <log> --stalls          # find INFO1a stalls
  v90_phase2_timeline.py <log> --wav OUT --from S --to S

Signal signatures (V.34 11.2 / V.90 9.2):
  Tone A          single tone 2400 Hz      (analogue modem)
  Tone B          single tone 1200 Hz      (digital modem -- us)
  INFO / marks    DPSK 600 baud on the tone: carrier plus sidebands +-600 Hz
  L1 / L2         line probe, many tones 150-3750 Hz at once
"""
import argparse
import re
import struct
import sys
import wave

import numpy as np

FS = 8000
WIN = 0.10

RX_STAGES = {
    0: "IDLE", 1: "INFO0", 2: "INFO1", 3: "INFOH", 4: "INFO1A", 5: "TONE_A",
    6: "TONE_B", 7: "L1_L2", 10: "PHASE3_WAIT_S", 11: "PHASE3_TRAINING",
}

def mark_re(which):
    return re.compile(
        r"%s dump mark: sample=(\d+) rx_stage=(-?\d+) tx_stage=(-?\d+) mod=(-?\d+)"
        % which)


def load_marks(log_path, which):
    """Marks for one dump.

    RX and TX are written from different callbacks at different rates and the
    files end up different lengths for the same call, so each has its own
    counter and they are NOT interchangeable.  Indexing the TX audio with RX
    marks reads the wrong instant entirely.
    """
    rx = mark_re(which)
    marks = []
    with open(log_path, errors="replace") as fp:
        for line in fp:
            m = rx.search(line)
            if m:
                marks.append(tuple(int(g) for g in m.groups()))
    return marks


def stage_at(marks, sample):
    """Stage in effect at a sample offset (marks are monotonic in sample)."""
    lo, hi, best = 0, len(marks) - 1, None
    while lo <= hi:
        mid = (lo + hi) // 2
        if marks[mid][0] <= sample:
            best = marks[mid]
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def classify(seg):
    """Return (verdict, peaks) for one window of audio."""
    rms = float(np.sqrt(np.mean(seg ** 2)))
    if rms < 50:
        return "silence", [], rms
    w = seg * np.hanning(len(seg))
    sp = np.abs(np.fft.rfft(w))
    fr = np.fft.rfftfreq(len(seg), 1.0 / FS)
    band = (fr > 100) & (fr < 3900)
    sp, fr = sp[band], fr[band]
    sp = sp / sp.max()
    peaks = []
    for i in np.argsort(sp)[::-1]:
        f = float(fr[i])
        if sp[i] < 0.20:
            break
        if any(abs(f - p) < 120 for p in peaks):
            continue
        peaks.append(f)
        if len(peaks) >= 10:
            break
    peaks.sort()
    if len(peaks) >= 6:
        return "LINE PROBE L1/L2 (multi-tone)", peaks, rms
    if len(peaks) == 1:
        f = peaks[0]
        if abs(f - 2400) < 150:
            return "Tone A (2400 Hz, steady)", peaks, rms
        if abs(f - 1200) < 150:
            return "Tone B (1200 Hz, steady)", peaks, rms
        return "single tone %.0f Hz" % f, peaks, rms
    # DPSK shows carrier +- 600 Hz. Look for that spacing.
    for i in range(len(peaks)):
        for j in range(i + 1, len(peaks)):
            if abs((peaks[j] - peaks[i]) - 600) < 130:
                mid = (peaks[i] + peaks[j]) / 2.0
                name = "Tone A" if abs(mid - 2400) < 250 else (
                    "Tone B" if abs(mid - 1200) < 250 else "%.0f Hz" % mid)
                return "DPSK on %s (INFO/marks, 600 baud)" % name, peaks, rms
    return "%d tones" % len(peaks), peaks, rms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--audio", default="/tmp/v34_rx.raw")
    ap.add_argument("--from", dest="t0", type=float, default=0.0)
    ap.add_argument("--to", dest="t1", type=float, default=1e9)
    ap.add_argument("--wav", help="write the selected window to this WAV file")
    ap.add_argument("--stalls", action="store_true",
                    help="list windows spent waiting for INFO1a and stop")
    args = ap.parse_args()

    which = "TX" if "tx" in args.audio.rsplit("/", 1)[-1] else "RX"
    marks = load_marks(args.log, which)
    if not marks:
        print("No '%s dump mark' lines in %s -- the build predates them, or "
              "VPCM_ME_VERBOSE was not set." % (which, args.log), file=sys.stderr)
        return 2
    print("using %s marks for %s" % (which, args.audio))
    d = np.fromfile(args.audio, dtype=np.int16).astype(np.float64)
    print("audio: %d samples (%.1f s), %d marks, last mark at sample %d"
          % (d.size, d.size / FS, len(marks), marks[-1][0]))

    # tx_stage 16 is V34_TX_STAGE_V90_WAIT_INFO1A -- the stall we care about.
    if args.stalls:
        runs, cur = [], None
        for sample, rxs, txs, _mod in marks:
            if txs == 16:
                if cur is None:
                    cur = [sample, sample]
                else:
                    cur[1] = sample
            elif cur is not None:
                runs.append(cur)
                cur = None
        if cur is not None:
            runs.append(cur)
        if not runs:
            print("No WAIT_INFO1A windows in this log.")
            return 0
        print("\nWAIT_INFO1A windows (the Phase 2 stall):")
        for a, b in runs:
            print("  %7.2f s -> %7.2f s   (%.2f s)   samples %d..%d"
                  % (a / FS, b / FS, (b - a) / FS, a, b))
        return 0

    n0 = max(0, int(args.t0 * FS))
    n1 = min(len(d), int(args.t1 * FS))
    if n1 <= n0:
        print("empty window", file=sys.stderr)
        return 2

    if args.wav:
        seg = d[n0:n1].astype(np.int16)
        with wave.open(args.wav, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(FS)
            w.writeframes(struct.pack("<%dh" % seg.size, *seg))
        print("wrote %s (%.2f s)" % (args.wav, seg.size / float(FS)))
        return 0

    print("\n%-9s %-16s %-6s %-34s %s"
          % ("t(s)", "stage(rx/tx)", "rms", "peaks (Hz)", "what it is"))
    step = int(WIN * FS)
    last = None
    for i in range(n0, n1 - step, step):
        verdict, peaks, rms = classify(d[i:i + step])
        mk = stage_at(marks, i)
        stage = "%s/%d" % (RX_STAGES.get(mk[1], str(mk[1])), mk[2]) if mk else "?"
        line = (stage, verdict)
        # Collapse runs of identical stage+verdict so the phases stand out.
        if line == last:
            continue
        last = line
        ptxt = " ".join("%.0f" % p for p in peaks[:6])
        print("%-9.2f %-16s %-6.0f %-34s %s"
              % (i / float(FS), stage, rms, ptxt, verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
