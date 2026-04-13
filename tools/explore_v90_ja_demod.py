#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from search_v90_ja import load_iq_dump, parse_bitdump_anchor


QPSK_REFS = np.asarray(
    [
        (1.0 + 1.0j) / math.sqrt(2.0),
        (1.0 - 1.0j) / math.sqrt(2.0),
        (-1.0 + 1.0j) / math.sqrt(2.0),
        (-1.0 - 1.0j) / math.sqrt(2.0),
    ],
    dtype=np.complex128,
)

DIFF_REFS = np.asarray([1.0 + 0.0j, 0.0 + 1.0j, -1.0 + 0.0j, 0.0 - 1.0j], dtype=np.complex128)


@dataclass
class Candidate:
    score: float
    start_ms: float
    slip_symbols: int
    start_sym: int
    n_symbols: int
    mag_ratio: float
    sym_evm: float
    diff_evm: float
    origin_frac: float
    quad_balance: float
    diff_balance: float
    i_mean: float
    q_mean: float


def frange(start: float, stop: float, step: float) -> list[float]:
    out = []
    v = start
    while v <= stop + 1e-12:
        out.append(round(v, 6))
        v += step
    return out


def rms_normalize(symbols: np.ndarray) -> np.ndarray:
    rms = float(np.sqrt(np.mean(np.abs(symbols) ** 2)))
    if rms <= 1e-12:
        return symbols.copy()
    return symbols / rms


def nearest_evm(points: np.ndarray, refs: np.ndarray) -> float:
    if len(points) == 0:
        return 99.0
    d2 = np.abs(points[:, None] - refs[None, :]) ** 2
    return float(np.sqrt(np.mean(np.min(d2, axis=1))))


def quadrant_balance(points: np.ndarray) -> float:
    counts = np.zeros(4, dtype=np.float64)
    for s in points:
        idx = (0 if s.real >= 0 else 2) + (0 if s.imag >= 0 else 1)
        counts[idx] += 1.0
    if np.sum(counts) <= 0:
        return 0.0
    probs = counts / np.sum(counts)
    ideal = 0.25
    return float(np.sum(np.abs(probs - ideal)))


def diff_symbols(symbols: np.ndarray, prev_symbol: complex) -> np.ndarray:
    prevs = np.empty(len(symbols), dtype=np.complex128)
    prevs[0] = prev_symbol
    if len(symbols) > 1:
        prevs[1:] = symbols[:-1]
    return symbols * np.conj(prevs)


def render_svg(points: np.ndarray, refs: np.ndarray, title: str, out: Path) -> None:
    w = 860
    h = 860
    pad = 90
    plot_w = w - 2 * pad
    plot_h = h - 2 * pad
    lim = max(1.5, float(np.max(np.abs(np.concatenate([points.real, points.imag, refs.real, refs.imag])))) * 1.15)

    def sx(x: float) -> float:
        return pad + (x + lim) * plot_w / (2 * lim)

    def sy(y: float) -> float:
        return pad + (lim - y) * plot_h / (2 * lim)

    lines: list[str] = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">')
    lines.append('<rect width="100%" height="100%" fill="#ffffff"/>')
    lines.append(f'<text x="{w/2:.1f}" y="42" text-anchor="middle" font-size="28" font-family="Arial" font-weight="700">{title}</text>')
    lines.append(f'<rect x="{pad}" y="{pad}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#333" stroke-width="2"/>')
    lines.append(f'<line x1="{sx(0):.2f}" y1="{pad}" x2="{sx(0):.2f}" y2="{h-pad}" stroke="#666" stroke-width="1.5"/>')
    lines.append(f'<line x1="{pad}" y1="{sy(0):.2f}" x2="{w-pad}" y2="{sy(0):.2f}" stroke="#666" stroke-width="1.5"/>')
    for t in (-1.0, -0.5, 0.5, 1.0):
        lines.append(f'<line x1="{sx(t):.2f}" y1="{pad}" x2="{sx(t):.2f}" y2="{h-pad}" stroke="#e5e7eb" stroke-width="1"/>')
        lines.append(f'<line x1="{pad}" y1="{sy(t):.2f}" x2="{w-pad}" y2="{sy(t):.2f}" stroke="#e5e7eb" stroke-width="1"/>')
    for r in refs:
        lines.append(f'<circle cx="{sx(float(r.real)):.2f}" cy="{sy(float(r.imag)):.2f}" r="6.0" fill="#111" opacity="0.55"/>')
    for s in points:
        lines.append(f'<circle cx="{sx(float(s.real)):.2f}" cy="{sy(float(s.imag)):.2f}" r="3.2" fill="#0b66d0" fill-opacity="0.50"/>')
    lines.append(f'<text x="{w/2:.1f}" y="{h-20}" text-anchor="middle" font-size="24" font-family="Arial" font-weight="700">In-phase</text>')
    lines.append(f'<text x="30" y="{h/2:.1f}" text-anchor="middle" font-size="24" font-family="Arial" font-weight="700" transform="rotate(-90 30 {h/2:.1f})">Quadrature</text>')
    lines.append("</svg>")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="ascii")


def candidate_metrics(iq_symbols: np.ndarray,
                      start_sym: int,
                      n_symbols: int,
                      trn_mag_ref: float,
                      sym_rate: float,
                      t0_ms: float,
                      slip: int) -> Candidate:
    syms = iq_symbols[start_sym:start_sym + n_symbols]
    prev_symbol = complex(iq_symbols[start_sym - 1])
    dsyms = diff_symbols(syms, prev_symbol)

    syms_n = rms_normalize(syms)
    dsyms_n = rms_normalize(dsyms)

    mag = np.abs(syms)
    mag_mean = float(np.mean(mag))
    mag_ratio = mag_mean / max(trn_mag_ref, 1e-12)
    origin_frac = float(np.mean(mag < (0.35 * trn_mag_ref)))
    sym_evm = nearest_evm(syms_n, QPSK_REFS)
    diff_evm = nearest_evm(dsyms_n, DIFF_REFS)
    quad_bal = quadrant_balance(syms_n)
    diff_bal = quadrant_balance(dsyms_n)
    i_mean = float(np.mean(syms.real))
    q_mean = float(np.mean(syms.imag))

    score = 250.0 * mag_ratio
    score -= 120.0 * sym_evm
    score -= 110.0 * diff_evm
    score -= 140.0 * origin_frac
    score -= 80.0 * quad_bal
    score -= 55.0 * diff_bal
    score -= 20.0 * abs(i_mean)
    score -= 20.0 * abs(q_mean)

    start_ms = t0_ms + (1000.0 * start_sym / sym_rate)
    return Candidate(
        score=score,
        start_ms=start_ms,
        slip_symbols=slip,
        start_sym=start_sym,
        n_symbols=n_symbols,
        mag_ratio=mag_ratio,
        sym_evm=sym_evm,
        diff_evm=diff_evm,
        origin_frac=origin_frac,
        quad_balance=quad_bal,
        diff_balance=diff_bal,
        i_mean=i_mean,
        q_mean=q_mean,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Explore V.90 J_a demod quality from anchored IQ symbols")
    ap.add_argument("--bitdump-anchor", type=Path, required=True)
    ap.add_argument("--iq-dump", type=Path, required=True)
    ap.add_argument("--after-trn-ms-min", type=float, default=0.0)
    ap.add_argument("--after-trn-ms-max", type=float, default=250.0)
    ap.add_argument("--after-trn-ms-step", type=float, default=5.0)
    ap.add_argument("--symbol-slip-min", type=int, default=-2)
    ap.add_argument("--symbol-slip-max", type=int, default=2)
    ap.add_argument("--symbols", type=int, default=256)
    ap.add_argument("--trn-ref-symbols", type=int, default=256)
    ap.add_argument("--top", type=int, default=12)
    ap.add_argument("--out-dir", type=Path, default=Path("tools/dumps"))
    args = ap.parse_args()

    anchor = parse_bitdump_anchor(args.bitdump_anchor)
    if "trn_run_end_sym" not in anchor or "trn_run_end_abs_ms" not in anchor:
        raise SystemExit(f"bit dump lacks TRN metadata: {args.bitdump_anchor}")

    iq_symbols, iq_meta = load_iq_dump(args.iq_dump)
    if "window_t0_s" not in iq_meta or "sym_rate" not in iq_meta:
        raise SystemExit(f"IQ dump lacks metadata: {args.iq_dump}")

    t0_ms = float(iq_meta["window_t0_s"]) * 1000.0
    sym_rate = float(iq_meta["sym_rate"])
    trn_end_sym = int(round(float(anchor["trn_run_end_sym"])))
    trn_ref_start = max(0, trn_end_sym - args.trn_ref_symbols)
    trn_ref = iq_symbols[trn_ref_start:trn_end_sym]
    if len(trn_ref) == 0:
        raise SystemExit("empty TRN reference window")
    trn_mag_ref = float(np.mean(np.abs(trn_ref)))

    starts = frange(float(anchor["trn_run_end_abs_ms"]) + args.after_trn_ms_min,
                    float(anchor["trn_run_end_abs_ms"]) + args.after_trn_ms_max,
                    args.after_trn_ms_step)

    candidates: list[Candidate] = []
    for start_ms in starts:
        base_start_sym = int(round((start_ms - t0_ms) * sym_rate / 1000.0))
        for slip in range(args.symbol_slip_min, args.symbol_slip_max + 1):
            start_sym = base_start_sym + slip
            if start_sym <= 0 or start_sym + args.symbols > len(iq_symbols):
                continue
            candidates.append(
                candidate_metrics(
                    iq_symbols,
                    start_sym,
                    args.symbols,
                    trn_mag_ref,
                    sym_rate,
                    t0_ms,
                    slip,
                )
            )

    candidates.sort(key=lambda c: c.score, reverse=True)
    top = candidates[: max(1, args.top)]

    print(
        "rank\tscore\tstart_ms\tslip\tstart_sym\tn\tmag_ratio\tsym_evm\tdiff_evm\torigin_frac\tquad_bal\tdiff_bal\ti_mean\tq_mean"
    )
    for i, c in enumerate(top, 1):
        print(
            f"{i}\t{c.score:.1f}\t{c.start_ms:.3f}\t{c.slip_symbols:+d}\t{c.start_sym}\t{c.n_symbols}\t"
            f"{c.mag_ratio:.3f}\t{c.sym_evm:.3f}\t{c.diff_evm:.3f}\t{c.origin_frac:.3f}\t"
            f"{c.quad_balance:.3f}\t{c.diff_balance:.3f}\t{c.i_mean:+.3f}\t{c.q_mean:+.3f}"
        )

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for idx, cand in enumerate(top[:3], 1):
        syms = iq_symbols[cand.start_sym:cand.start_sym + cand.n_symbols]
        prev = complex(iq_symbols[cand.start_sym - 1])
        dsyms = diff_symbols(syms, prev)
        stem = (
            f"v90_demod_rank{idx}_{int(round(cand.start_ms))}ms"
            f"_slip{cand.slip_symbols:+d}".replace("+", "p").replace("-", "m")
        )
        render_svg(rms_normalize(syms), QPSK_REFS, f"Rank {idx} Symbol Constellation", args.out_dir / f"{stem}_sym.svg")
        render_svg(rms_normalize(dsyms), DIFF_REFS, f"Rank {idx} Differential Constellation", args.out_dir / f"{stem}_diff.svg")
        txt = args.out_dir / f"{stem}.txt"
        lines = [
            f"start_ms={cand.start_ms:.6f}",
            f"slip_symbols={cand.slip_symbols}",
            f"start_sym={cand.start_sym}",
            f"symbols={cand.n_symbols}",
            f"mag_ratio={cand.mag_ratio:.6f}",
            f"sym_evm={cand.sym_evm:.6f}",
            f"diff_evm={cand.diff_evm:.6f}",
            f"origin_frac={cand.origin_frac:.6f}",
            f"quad_balance={cand.quad_balance:.6f}",
            f"diff_balance={cand.diff_balance:.6f}",
            f"i_mean={cand.i_mean:+.6f}",
            f"q_mean={cand.q_mean:+.6f}",
            "idx\tI\tQ\tmag\tphase_deg\tdiff_phase_deg",
        ]
        prevs = np.empty(len(syms), dtype=np.complex128)
        prevs[0] = prev
        if len(syms) > 1:
            prevs[1:] = syms[:-1]
        dph = np.angle(syms * np.conj(prevs), deg=True)
        ph = np.angle(syms, deg=True)
        mag = np.abs(syms)
        for j in range(len(syms)):
            lines.append(f"{cand.start_sym + j}\t{syms[j].real:+.6f}\t{syms[j].imag:+.6f}\t{mag[j]:.6f}\t{ph[j]:+.3f}\t{dph[j]:+.3f}")
        txt.write_text("\n".join(lines) + "\n", encoding="ascii")
        print(f"rank{idx}_sym_svg={args.out_dir / f'{stem}_sym.svg'}")
        print(f"rank{idx}_diff_svg={args.out_dir / f'{stem}_diff.svg'}")
        print(f"rank{idx}_txt={txt}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
