#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
import wave
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from recover_ja_descriptor import (
    DilDesc,
    block_match,
    consensus_block,
    descramble_bits,
    descriptor_nearness,
    parse_dil_descriptor,
)


# V.90 upstream symbol-rate/carrier set (§6.2/V.90).
V90_CARRIERS: list[tuple[float, float]] = [
    (3000.0, 1800.0),
    (3000.0, 2000.0),
    (3200.0, 1829.0),
    (3200.0, 1920.0),
    (3429.0, 1959.0),
]


def ulaw_byte_to_linear(u: int) -> int:
    u = (~u) & 0xFF
    sign = u & 0x80
    exponent = (u >> 4) & 0x07
    mantissa = u & 0x0F
    sample = ((mantissa << 3) + 0x84) << exponent
    sample -= 0x84
    return -sample if sign else sample


def interp(xs: np.ndarray, t: float) -> float:
    if t < 0 or t >= len(xs) - 1:
        return 0.0
    i = int(t)
    f = t - i
    return float(xs[i] * (1.0 - f) + xs[i + 1] * f)


def sample_carrier_symbol(xs: np.ndarray, t_center: float, carrier_hz: float, fs: int) -> complex:
    acc = 0j
    for k in range(-2, 3):
        t = t_center + k
        acc += interp(xs, t) * np.exp(-1j * 2.0 * math.pi * carrier_hz * (t / fs))
    return acc


def frange(start: float, stop: float, step: float) -> list[float]:
    out = []
    v = start
    while v <= stop + 1e-12:
        out.append(round(v, 6))
        v += step
    return out


def chunk_bits(bits: str, width: int = 64) -> str:
    return "\n".join(bits[i:i + width] for i in range(0, len(bits), width))


def invert_bits(bits: str) -> str:
    return "".join("0" if b == "1" else "1" for b in bits)


def swap_dibit_order(bits: str) -> str:
    out: list[str] = []
    for i in range(0, len(bits) - 1, 2):
        out.append(bits[i + 1])
        out.append(bits[i])
    if len(bits) & 1:
        out.append(bits[-1])
    return "".join(out)


def bit_streams_from_diffs(diffs: np.ndarray) -> dict[str, str]:
    nat_bits = "".join(f"{int(v):02b}" for v in diffs)
    gray_map = {0: "00", 1: "01", 2: "11", 3: "10"}
    gray_bits = "".join(gray_map[int(v)] for v in diffs)

    out = {
        "nat": nat_bits,
        "gray": gray_bits,
    }

    base_items = list(out.items())
    for name, bits in base_items:
        out[f"{name}_inv"] = invert_bits(bits)
        out[f"{name}_swap"] = swap_dibit_order(bits)
        out[f"{name}_swap_inv"] = invert_bits(out[f"{name}_swap"])

    rev_diffs = (-diffs) & 3
    nat_rev = "".join(f"{int(v):02b}" for v in rev_diffs)
    gray_map = {0: "00", 1: "01", 2: "11", 3: "10"}
    gray_rev = "".join(gray_map[int(v)] for v in rev_diffs)
    out["nat_rev"] = nat_rev
    out["gray_rev"] = gray_rev
    for name in ("nat_rev", "gray_rev"):
        bits = out[name]
        out[f"{name}_inv"] = invert_bits(bits)
        out[f"{name}_swap"] = swap_dibit_order(bits)
        out[f"{name}_swap_inv"] = invert_bits(out[f"{name}_swap"])

    raw_items = list(out.items())
    for name, bits in raw_items:
        out[f"{name}_d4"] = descramble_bits(bits, 4)
        out[f"{name}_d17"] = descramble_bits(bits, 17)

    return out


V90_STREAM_GROUPS: dict[str, list[str]] = {
    "basic": ["gray", "gray_inv", "nat", "nat_inv"],
    "rev": ["gray_rev", "gray_rev_inv", "nat_rev", "nat_rev_inv"],
    "swap": ["gray_swap", "gray_swap_inv", "nat_swap", "nat_swap_inv"],
    "rev_swap": [
        "gray_rev_swap",
        "gray_rev_swap_inv",
        "nat_rev_swap",
        "nat_rev_swap_inv",
    ],
    "descr": [
        "gray_d4", "gray_d17", "gray_inv_d4", "gray_inv_d17",
        "nat_d4", "nat_d17", "nat_inv_d4", "nat_inv_d17",
    ],
    "all_v90": [
        "gray", "gray_inv", "nat", "nat_inv",
        "gray_rev", "gray_rev_inv", "nat_rev", "nat_rev_inv",
        "gray_swap", "gray_swap_inv", "nat_swap", "nat_swap_inv",
        "gray_rev_swap", "gray_rev_swap_inv", "nat_rev_swap", "nat_rev_swap_inv",
    ],
    "all_v90_descr": [
        "gray", "gray_inv", "nat", "nat_inv",
        "gray_rev", "gray_rev_inv", "nat_rev", "nat_rev_inv",
        "gray_swap", "gray_swap_inv", "nat_swap", "nat_swap_inv",
        "gray_rev_swap", "gray_rev_swap_inv", "nat_rev_swap", "nat_rev_swap_inv",
        "gray_d4", "gray_d17", "gray_inv_d4", "gray_inv_d17",
        "nat_d4", "nat_d17", "nat_inv_d4", "nat_inv_d17",
        "gray_rev_d4", "gray_rev_d17", "gray_rev_inv_d4", "gray_rev_inv_d17",
        "nat_rev_d4", "nat_rev_d17", "nat_rev_inv_d4", "nat_rev_inv_d17",
        "gray_swap_d4", "gray_swap_d17", "gray_swap_inv_d4", "gray_swap_inv_d17",
        "nat_swap_d4", "nat_swap_d17", "nat_swap_inv_d4", "nat_swap_inv_d17",
        "gray_rev_swap_d4", "gray_rev_swap_d17",
        "gray_rev_swap_inv_d4", "gray_rev_swap_inv_d17",
        "nat_rev_swap_d4", "nat_rev_swap_d17",
        "nat_rev_swap_inv_d4", "nat_rev_swap_inv_d17",
    ],
}


def qpsk_bit_streams(xs: np.ndarray,
                     fs: int,
                     start_ms: float,
                     offset_samples: float,
                     carrier_hz: float,
                     sym_rate: float,
                     symbols: int) -> dict[str, str]:
    step = fs / sym_rate
    start = start_ms * fs / 1000.0 + offset_samples
    syms = np.zeros(symbols, dtype=np.complex128)

    for n in range(symbols):
        syms[n] = sample_carrier_symbol(xs, start + n * step, carrier_hz, fs)

    states = (((np.angle(syms) + math.pi) / (math.pi / 2.0)).astype(int)) & 3
    diffs = (states[1:] - states[:-1]) & 3
    return bit_streams_from_diffs(diffs)


def qpsk_bit_streams_from_symbols(symbols: np.ndarray,
                                  prev_symbol: complex | None = None) -> dict[str, str]:
    if prev_symbol is not None:
        full = np.empty(len(symbols) + 1, dtype=np.complex128)
        full[0] = prev_symbol
        full[1:] = symbols
    else:
        full = symbols

    states = (((np.angle(full) + math.pi) / (math.pi / 2.0)).astype(int)) & 3
    diffs = (states[1:] - states[:-1]) & 3
    return bit_streams_from_diffs(diffs)


def bitdump_text(wav: Path,
                 start_ms: float,
                 offset_samples: float,
                 carrier_hz: float,
                 sym_rate: float,
                 symbols: int,
                 bit_streams: dict[str, str]) -> str:
    lines = [
        f"file={wav}",
        f"start_ms={start_ms:.3f} mode=carrier-qpsk carrier_hz={carrier_hz:.3f} "
        f"sym_rate={sym_rate:.3f} offset_samples={offset_samples:.3f}",
        f"symbols={symbols}",
    ]

    label_aliases = {
        "nat": "nat_bits",
        "gray": "gray_bits",
        "nat_inv": "nat_bits_inverted",
        "gray_inv": "gray_bits_inverted",
    }

    preferred = [
        "nat", "gray", "nat_inv", "gray_inv",
        "nat_rev", "gray_rev", "nat_rev_inv", "gray_rev_inv",
        "nat_swap", "gray_swap", "nat_swap_inv", "gray_swap_inv",
        "nat_rev_swap", "gray_rev_swap", "nat_rev_swap_inv", "gray_rev_swap_inv",
        "nat_d4", "gray_d4", "nat_inv_d4", "gray_inv_d4",
        "nat_d17", "gray_d17", "nat_inv_d17", "gray_inv_d17",
        "nat_rev_d4", "gray_rev_d4", "nat_rev_inv_d4", "gray_rev_inv_d4",
        "nat_rev_d17", "gray_rev_d17", "nat_rev_inv_d17", "gray_rev_inv_d17",
        "nat_swap_d4", "gray_swap_d4", "nat_swap_inv_d4", "gray_swap_inv_d4",
        "nat_swap_d17", "gray_swap_d17", "nat_swap_inv_d17", "gray_swap_inv_d17",
        "nat_rev_swap_d4", "gray_rev_swap_d4", "nat_rev_swap_inv_d4", "gray_rev_swap_inv_d4",
        "nat_rev_swap_d17", "gray_rev_swap_d17", "nat_rev_swap_inv_d17", "gray_rev_swap_inv_d17",
    ]
    written: set[str] = set()
    for key in preferred:
        if key in bit_streams and key not in written:
            label = label_aliases.get(key, f"{key}_bits")
            bits = bit_streams[key]
            lines.append(f"{label}({len(bits)}):")
            lines.append(chunk_bits(bits))
            written.add(key)

    for key, bits in bit_streams.items():
        if key in written:
            continue
        lines.append(f"{key}_bits({len(bits)}):")
        lines.append(chunk_bits(bits))

    return "\n".join(lines) + "\n"


@dataclass
class Candidate:
    score: float
    start_ms: float
    offset_samples: float
    slip_symbols: int
    carrier_hz: float
    sym_rate: float
    stream: str
    descriptor_start: int
    block_len: int
    blocks: int
    rep: float
    sync_hd: int
    zero_viol: int
    frame17_viol: int
    crc90_hd: int
    crc92_hd: int
    fs12_pos: int
    parsed: DilDesc | None


def framing_zero_viol(bits: str, start: int = 51, step: int = 17, count: int = 40) -> int:
    v = 0
    for k in range(count):
        i = start + k * step
        if i >= len(bits):
            break
        if bits[i] != "0":
            v += 1
    return v


def score_candidate(bits: str,
                    ds: int,
                    block_len: int,
                    blocks: int) -> tuple[float, float, DilDesc | None, int, int, int, int, int, int]:
    rep = block_match(bits, ds, block_len, blocks)
    cons = consensus_block(bits, ds, block_len, blocks)
    parsed = parse_dil_descriptor(cons)
    near = descriptor_nearness(cons)
    if near is None:
        sync_hd, zero_viol, crc90_hd, crc92_hd, fs12_pos = (18, 12, 16, 16, -1)
    else:
        sync_hd, zero_viol, crc90_hd, crc92_hd, fs12_pos = near
    frame17_viol = framing_zero_viol(cons, 51, 17, 40)

    # Prefer V.90 descriptor structure over raw periodicity.
    score = rep * 500.0
    score -= 35.0 * sync_hd
    score -= 25.0 * zero_viol
    score -= 18.0 * frame17_viol
    score -= 8.0 * crc90_hd
    score -= 2.0 * crc92_hd
    if fs12_pos >= 0:
        score -= 80.0
    if parsed is not None:
        score += 2500.0 if parsed.variant == "v90" else 500.0

    return score, rep, parsed, sync_hd, zero_viol, frame17_viol, crc90_hd, crc92_hd, fs12_pos


def parse_pairs(text: str) -> list[tuple[float, float]]:
    out = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        rate_s, carrier_s = item.split("/")
        out.append((float(rate_s), float(carrier_s)))
    return out


def load_wav_channel(path: Path, channel: int) -> tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as w:
        fs = w.getframerate()
        nc = w.getnchannels()
        sw = w.getsampwidth()
        frames = w.readframes(w.getnframes())

    if channel < 0 or channel >= nc:
        raise ValueError(f"channel {channel} out of range for {nc}-channel WAV")

    if sw == 1:
        xs = np.frombuffer(frames, dtype=np.uint8).astype(np.float64)
        xs -= 128.0
    elif sw == 2:
        xs = np.frombuffer(frames, dtype="<i2").astype(np.float64)
    else:
        raise ValueError(f"unsupported sample width {sw} bytes")

    if nc > 1:
        xs = xs.reshape(-1, nc)[:, channel]

    return xs, fs


def parse_bitdump_anchor(path: Path) -> dict[str, float | str]:
    txt = path.read_text(encoding="ascii", errors="ignore")
    meta: dict[str, float | str] = {}

    m = re.search(
        r"window_s=([0-9.]+)-([0-9.]+)\s+protocol=(\S+)\s+carrier_hz=([0-9.]+)\s+sym_rate=([0-9.]+)",
        txt,
    )
    if m:
        meta["window_t0_s"] = float(m.group(1))
        meta["window_t1_s"] = float(m.group(2))
        meta["protocol"] = m.group(3)
        meta["carrier_hz"] = float(m.group(4))
        meta["sym_rate"] = float(m.group(5))

    m = re.search(
        r"trn_run_start_sym=(\d+)\s+trn_run_end_sym=(\d+)\s+"
        r"trn_run_start_ms=([0-9.]+)\s+trn_run_end_ms=([0-9.]+)\s+"
        r"trn_run_start_abs_ms=([0-9.]+)\s+trn_run_end_abs_ms=([0-9.]+)",
        txt,
    )
    if m:
        meta["trn_run_start_sym"] = float(m.group(1))
        meta["trn_run_end_sym"] = float(m.group(2))
        meta["trn_run_start_ms"] = float(m.group(3))
        meta["trn_run_end_ms"] = float(m.group(4))
        meta["trn_run_start_abs_ms"] = float(m.group(5))
        meta["trn_run_end_abs_ms"] = float(m.group(6))

    return meta


def load_iq_dump(path: Path) -> tuple[np.ndarray, dict[str, float]]:
    symbols: list[complex] = []
    meta: dict[str, float] = {}
    for line in path.read_text(encoding="ascii", errors="ignore").splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            m = re.search(r"sym_rate=([0-9.]+)\s+fc=([0-9.]+)\s+t0=([0-9.]+)\s+t1=([0-9.]+)", line)
            if m:
                meta["sym_rate"] = float(m.group(1))
                meta["carrier_hz"] = float(m.group(2))
                meta["window_t0_s"] = float(m.group(3))
                meta["window_t1_s"] = float(m.group(4))
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        symbols.append(complex(float(parts[2]), float(parts[3])))

    if not symbols:
        raise ValueError(f"no IQ symbols found in {path}")
    return np.asarray(symbols, dtype=np.complex128), meta


def expand_stream_names(stream_text: str) -> list[str]:
    out: list[str] = []
    seen: set[str] = set()
    for item in stream_text.split(","):
        name = item.strip()
        if not name:
            continue
        names = V90_STREAM_GROUPS.get(name, [name])
        for expanded in names:
            if expanded not in seen:
                seen.add(expanded)
                out.append(expanded)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Search for V.90 §8.3.1 Ja candidates")
    ap.add_argument("path", type=Path)
    ap.add_argument("--channel", type=int, default=1)
    ap.add_argument("--start-ms", type=float, default=None)
    ap.add_argument("--start-span-ms", type=float, default=150.0)
    ap.add_argument("--start-step-ms", type=float, default=10.0)
    ap.add_argument("--bitdump-anchor", type=Path, default=None,
                    help="Use TRN-run metadata from a v34_phase3_demod bit dump")
    ap.add_argument("--iq-dump", type=Path, default=None,
                    help="Use equalized symbols from a v34_phase3_demod IQ dump")
    ap.add_argument("--after-trn-ms-min", type=float, default=0.0,
                    help="Search start relative to TRN end when --bitdump-anchor is used")
    ap.add_argument("--after-trn-ms-max", type=float, default=250.0,
                    help="Search stop relative to TRN end when --bitdump-anchor is used")
    ap.add_argument("--after-trn-ms-step", type=float, default=10.0,
                    help="Search step relative to TRN end when --bitdump-anchor is used")
    ap.add_argument("--pairs", type=str, default="",
                    help="Comma-separated rate/carrier pairs, e.g. 3000/1800,3200/1829")
    ap.add_argument("--offset-min", type=float, default=0.0)
    ap.add_argument("--offset-max", type=float, default=1.0)
    ap.add_argument("--offset-step", type=float, default=0.25)
    ap.add_argument("--symbol-slip-min", type=int, default=0,
                    help="Additional integer symbol slips to test around each IQ start")
    ap.add_argument("--symbol-slip-max", type=int, default=0,
                    help="Additional integer symbol slips to test around each IQ start")
    ap.add_argument("--symbols", type=int, default=1536)
    ap.add_argument("--descriptor-start-min", type=int, default=0)
    ap.add_argument("--descriptor-start-max", type=int, default=64)
    ap.add_argument("--len-min", type=int, default=206)
    ap.add_argument("--len-max", type=int, default=320)
    ap.add_argument("--min-blocks", type=int, default=3)
    ap.add_argument("--max-blocks", type=int, default=4)
    ap.add_argument("--streams", type=str, default="basic",
                    help="Comma-separated stream names or groups: basic, rev, swap, rev_swap, descr, all_v90, all_v90_descr")
    ap.add_argument("--top", type=int, default=20)
    ap.add_argument("--save-best", type=Path, default=None)
    args = ap.parse_args()

    anchor_meta: dict[str, float | str] = {}
    if args.bitdump_anchor is not None:
        anchor_meta = parse_bitdump_anchor(args.bitdump_anchor)
        if "trn_run_end_abs_ms" not in anchor_meta:
            raise SystemExit(f"bit dump lacks TRN anchor metadata: {args.bitdump_anchor}")

    if args.start_ms is None and args.bitdump_anchor is None:
        raise SystemExit("pass --start-ms or --bitdump-anchor")

    if args.pairs:
        pairs = parse_pairs(args.pairs)
    elif "sym_rate" in anchor_meta and "carrier_hz" in anchor_meta:
        pairs = [(float(anchor_meta["sym_rate"]), float(anchor_meta["carrier_hz"]))]
    else:
        pairs = V90_CARRIERS

    streams = expand_stream_names(args.streams)

    iq_symbols: np.ndarray | None = None
    iq_meta: dict[str, float] = {}
    xs: np.ndarray | None = None
    fs: int | None = None
    if args.iq_dump is not None:
        iq_symbols, iq_meta = load_iq_dump(args.iq_dump)
        if not pairs and "sym_rate" in iq_meta and "carrier_hz" in iq_meta:
            pairs = [(float(iq_meta["sym_rate"]), float(iq_meta["carrier_hz"]))]
    else:
        xs, fs = load_wav_channel(args.path, args.channel)

    if args.bitdump_anchor is not None:
        trn_end_abs_ms = float(anchor_meta["trn_run_end_abs_ms"])
        starts = frange(trn_end_abs_ms + args.after_trn_ms_min,
                        trn_end_abs_ms + args.after_trn_ms_max,
                        args.after_trn_ms_step)
        print(
            f"# anchor bitdump={args.bitdump_anchor} "
            f"trn_end_abs_ms={trn_end_abs_ms:.3f} "
            f"search_after_trn_ms={args.after_trn_ms_min:.3f}..{args.after_trn_ms_max:.3f} "
            f"step={args.after_trn_ms_step:.3f}"
        )
    else:
        assert args.start_ms is not None
        starts = frange(args.start_ms - args.start_span_ms,
                        args.start_ms + args.start_span_ms,
                        args.start_step_ms)

    offsets = frange(args.offset_min, args.offset_max, args.offset_step)
    symbol_slips = list(range(args.symbol_slip_min, args.symbol_slip_max + 1))

    candidates: list[Candidate] = []

    if iq_symbols is not None:
        if "window_t0_s" not in iq_meta or "sym_rate" not in iq_meta:
            raise SystemExit(f"IQ dump lacks sym_rate/window metadata: {args.iq_dump}")
        iq_t0_ms = float(iq_meta["window_t0_s"]) * 1000.0
        iq_sym_rate = float(iq_meta["sym_rate"])
        iq_carrier = float(iq_meta.get("carrier_hz", pairs[0][1]))

        for start_ms in starts:
            base_start_sym = int(round((start_ms - iq_t0_ms) * iq_sym_rate / 1000.0))
            for slip in symbol_slips:
                start_sym = base_start_sym + slip
                if start_sym <= 0:
                    continue
                stop_sym = start_sym + args.symbols
                if stop_sym > len(iq_symbols):
                    continue
                syms = iq_symbols[start_sym:stop_sym]
                prev_symbol = complex(iq_symbols[start_sym - 1])
                bit_streams = qpsk_bit_streams_from_symbols(syms, prev_symbol=prev_symbol)
                actual_start_ms = iq_t0_ms + (1000.0 * start_sym / iq_sym_rate)
                for stream_name in streams:
                    bits = bit_streams[stream_name]
                    for ds in range(args.descriptor_start_min, args.descriptor_start_max + 1):
                        avail = len(bits) - ds
                        for block_len in range(args.len_min, args.len_max + 1):
                            blocks = min(args.max_blocks, avail // block_len)
                            if blocks < args.min_blocks:
                                continue
                            score, rep, parsed, sync_hd, zero_viol, frame17_viol, crc90_hd, crc92_hd, fs12_pos = (
                                score_candidate(bits, ds, block_len, blocks)
                            )
                            candidates.append(
                                Candidate(
                                    score=score,
                                    start_ms=actual_start_ms,
                                    offset_samples=0.0,
                                    slip_symbols=slip,
                                    carrier_hz=iq_carrier,
                                    sym_rate=iq_sym_rate,
                                    stream=stream_name,
                                    descriptor_start=ds,
                                    block_len=block_len,
                                    blocks=blocks,
                                    rep=rep,
                                    sync_hd=sync_hd,
                                    zero_viol=zero_viol,
                                    frame17_viol=frame17_viol,
                                    crc90_hd=crc90_hd,
                                    crc92_hd=crc92_hd,
                                    fs12_pos=fs12_pos,
                                    parsed=parsed,
                                )
                            )
    else:
        assert xs is not None and fs is not None
        for start_ms in starts:
            for sym_rate, carrier_hz in pairs:
                for off in offsets:
                    bit_streams = qpsk_bit_streams(xs, fs, start_ms, off, carrier_hz, sym_rate, args.symbols)
                    for stream_name in streams:
                        bits = bit_streams[stream_name]
                        for ds in range(args.descriptor_start_min, args.descriptor_start_max + 1):
                            avail = len(bits) - ds
                            for block_len in range(args.len_min, args.len_max + 1):
                                blocks = min(args.max_blocks, avail // block_len)
                                if blocks < args.min_blocks:
                                    continue
                                score, rep, parsed, sync_hd, zero_viol, frame17_viol, crc90_hd, crc92_hd, fs12_pos = (
                                    score_candidate(bits, ds, block_len, blocks)
                                )
                                candidates.append(
                                    Candidate(
                                        score=score,
                                        start_ms=start_ms,
                                        offset_samples=off,
                                        slip_symbols=0,
                                        carrier_hz=carrier_hz,
                                        sym_rate=sym_rate,
                                        stream=stream_name,
                                        descriptor_start=ds,
                                        block_len=block_len,
                                        blocks=blocks,
                                        rep=rep,
                                        sync_hd=sync_hd,
                                        zero_viol=zero_viol,
                                        frame17_viol=frame17_viol,
                                        crc90_hd=crc90_hd,
                                        crc92_hd=crc92_hd,
                                        fs12_pos=fs12_pos,
                                        parsed=parsed,
                                    )
                                )

    candidates.sort(key=lambda c: c.score, reverse=True)
    top = candidates[: max(1, args.top)]

    print("rank\tscore\tstart_ms\toffset\tslip\trate\tcarrier\tstream\tds\tL\tblocks\trep\tparse\tvar\tsync_hd\tzviol\tf17\tcrc90\tcrc92\tfs12")
    for i, c in enumerate(top, 1):
        if c.parsed is None:
            print(
                f"{i}\t{c.score:.1f}\t{c.start_ms:.2f}\t{c.offset_samples:.2f}\t{c.slip_symbols:+d}\t{c.sym_rate:.0f}\t{c.carrier_hz:.0f}\t"
                f"{c.stream}\t{c.descriptor_start}\t{c.block_len}\t{c.blocks}\t{c.rep:.4f}\t0\t-\t"
                f"{c.sync_hd}\t{c.zero_viol}\t{c.frame17_viol}\t{c.crc90_hd}\t{c.crc92_hd}\t{c.fs12_pos}"
            )
        else:
            print(
                f"{i}\t{c.score:.1f}\t{c.start_ms:.2f}\t{c.offset_samples:.2f}\t{c.slip_symbols:+d}\t{c.sym_rate:.0f}\t{c.carrier_hz:.0f}\t"
                f"{c.stream}\t{c.descriptor_start}\t{c.block_len}\t{c.blocks}\t{c.rep:.4f}\t1\t{c.parsed.variant}\t"
                f"{c.sync_hd}\t{c.zero_viol}\t{c.frame17_viol}\t{c.crc90_hd}\t{c.crc92_hd}\t{c.fs12_pos}"
            )

    if args.save_best is not None and top:
        best = top[0]
        if iq_symbols is not None:
            iq_t0_ms = float(iq_meta["window_t0_s"]) * 1000.0
            start_sym = int(round((best.start_ms - iq_t0_ms) * best.sym_rate / 1000.0))
            syms = iq_symbols[start_sym:start_sym + args.symbols]
            prev_symbol = complex(iq_symbols[start_sym - 1]) if start_sym > 0 else None
            streams_for_best = qpsk_bit_streams_from_symbols(syms, prev_symbol=prev_symbol)
        else:
            assert xs is not None and fs is not None
            streams_for_best = qpsk_bit_streams(xs, fs, best.start_ms, best.offset_samples,
                                                best.carrier_hz, best.sym_rate, args.symbols)
        args.save_best.parent.mkdir(parents=True, exist_ok=True)
        args.save_best.write_text(
            bitdump_text(args.path, best.start_ms, best.offset_samples,
                         best.carrier_hz, best.sym_rate, args.symbols,
                         streams_for_best),
            encoding="ascii",
        )
        print(f"best_dump={args.save_best}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
