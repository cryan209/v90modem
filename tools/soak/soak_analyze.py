#!/usr/bin/env python3
"""Analyze soak test captures: throughput per phase + sequence integrity.

rx_sock.bin should contain the downstream "D%07d\n" stream (sent from PTY),
rx_pty.bin the upstream "U%07d\n" stream (sent from socket side).
"""
import os, re, sys

OUTDIR = sys.argv[1]
PHASES = {"A (downstream only)": (0, 35), "B (upstream only)": (35, 70),
          "C (bidirectional)": (70, 105)}

def phase_throughput(idx_path):
    rates = {name: 0 for name in PHASES}
    if not os.path.exists(idx_path):
        return rates
    for line in open(idx_path):
        try:
            t, n = float(line.split()[0]), int(line.split()[1])
        except (ValueError, IndexError):
            continue
        for name, (a, b) in PHASES.items():
            if a <= t < b:
                rates[name] += n
    return rates

def rev8(b):
    b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4)
    b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2)
    b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1)
    return b

REVTAB = bytes(rev8(i) for i in range(256))

def seq_check(bin_path, letter):
    """Check LETTER%07d\n stream integrity; tolerate leading/trailing junk.
    Also detects a per-byte bit-reversed stream and says so."""
    if not os.path.exists(bin_path):
        return "missing file"
    data = open(bin_path, "rb").read()
    pat = re.compile(re.escape(letter.encode()) + rb"(\d{7})\n")
    seqs = [int(m.group(1)) for m in pat.finditer(data)]
    note = ""
    if not seqs and data:
        rdata = data.translate(REVTAB)
        rseqs = [int(m.group(1)) for m in pat.finditer(rdata)]
        if rseqs:
            note = " [STREAM IS PER-BYTE BIT-REVERSED]"
            data, seqs = rdata, rseqs
    if not seqs:
        return f"0 pattern lines in {len(data)} bytes (even bit-reversed)"
    good_bytes = len(seqs) * 9
    gaps = dupes = ooo = 0
    for prev, cur in zip(seqs, seqs[1:]):
        if cur == prev + 1:
            continue
        elif cur > prev + 1:
            gaps += cur - prev - 1
        elif cur == prev:
            dupes += 1
        else:
            ooo += 1
    covered = seqs[-1] - seqs[0] + 1
    return (f"{len(seqs)} lines rx (seq {seqs[0]}..{seqs[-1]}), "
            f"missing={gaps} dupes={dupes} out-of-order={ooo}, "
            f"pattern bytes {good_bytes}/{len(data)} "
            f"({100.0*good_bytes/len(data):.1f}% clean)" + note
            + ("" if covered == len(seqs) - gaps + dupes else " [count mismatch]"))

print("=== throughput (received bytes per phase) ===")
for side, idx in (("sock (downstream rx)", "rx_sock.idx"),
                  ("pty  (upstream rx)", "rx_pty.idx")):
    rates = phase_throughput(os.path.join(OUTDIR, idx))
    parts = ", ".join(f"{name}: {n} B = {n*8/35:.0f} bps" for name, n in rates.items())
    print(f"{side}: {parts}")

print("=== integrity ===")
print("downstream (D lines at sock):", seq_check(os.path.join(OUTDIR, "rx_sock.bin"), "D"))
print("upstream   (U lines at pty): ", seq_check(os.path.join(OUTDIR, "rx_pty.bin"), "U"))
