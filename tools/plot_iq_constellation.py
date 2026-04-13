#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass
class IqPoint:
    sym_idx: int
    time_ms: float
    i: float
    q: float
    s_score: float
    pp_score: float
    label: str


def load_iq_dump(path: Path) -> tuple[list[IqPoint], dict[str, float]]:
    points: list[IqPoint] = []
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
                meta["t0_s"] = float(m.group(3))
                meta["t1_s"] = float(m.group(4))
            continue
        parts = line.split()
        if len(parts) < 7:
            continue
        points.append(
            IqPoint(
                sym_idx=int(parts[0]),
                time_ms=float(parts[1]),
                i=float(parts[2]),
                q=float(parts[3]),
                s_score=float(parts[4]),
                pp_score=float(parts[5]),
                label=parts[6],
            )
        )
    if not points:
        raise ValueError(f"no IQ points found in {path}")
    return points, meta


def select_points(points: list[IqPoint],
                  sym_start: int | None,
                  sym_end: int | None,
                  label: str | None) -> list[IqPoint]:
    out = points
    if sym_start is not None:
        out = [p for p in out if p.sym_idx >= sym_start]
    if sym_end is not None:
        out = [p for p in out if p.sym_idx < sym_end]
    if label is not None:
        out = [p for p in out if p.label == label]
    return out


def mean_std(vals: list[float]) -> tuple[float, float]:
    if not vals:
        return 0.0, 0.0
    mu = sum(vals) / len(vals)
    var = sum((v - mu) ** 2 for v in vals) / len(vals)
    return mu, math.sqrt(var)


def render_svg(points: list[IqPoint],
               title: str,
               out: Path,
               meta: dict[str, float]) -> None:
    w = 900
    h = 900
    pad = 90
    plot_w = w - 2 * pad
    plot_h = h - 2 * pad

    lim = max(max(abs(p.i), abs(p.q)) for p in points)
    lim = max(lim * 1.1, 1.8)

    def sx(x: float) -> float:
        return pad + (x + lim) * plot_w / (2 * lim)

    def sy(y: float) -> float:
        return pad + (lim - y) * plot_h / (2 * lim)

    colors = {"T": "#1166cc", "P": "#cc5500", "S": "#2b8a3e", "?": "#777777"}

    lines: list[str] = []
    lines.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">')
    lines.append('<rect width="100%" height="100%" fill="#ffffff"/>')
    lines.append(f'<text x="{w/2:.1f}" y="42" text-anchor="middle" font-size="28" font-family="Arial" font-weight="700">{title}</text>')
    if points:
        p0 = points[0]
        p1 = points[-1]
        subtitle = (
            f"symbols {p0.sym_idx}-{p1.sym_idx}   "
            f"time {p0.time_ms:.1f}-{p1.time_ms:.1f} ms   "
            f"count {len(points)}"
        )
        lines.append(f'<text x="{w/2:.1f}" y="72" text-anchor="middle" font-size="16" font-family="Arial">{subtitle}</text>')

    # grid
    for t in (-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5):
        if abs(t) > lim:
            continue
        x = sx(t)
        y = sy(t)
        lines.append(f'<line x1="{x:.2f}" y1="{pad}" x2="{x:.2f}" y2="{h-pad}" stroke="#e4e7eb" stroke-width="1"/>')
        lines.append(f'<line x1="{pad}" y1="{y:.2f}" x2="{w-pad}" y2="{y:.2f}" stroke="#e4e7eb" stroke-width="1"/>')

    # axes
    lines.append(f'<rect x="{pad}" y="{pad}" width="{plot_w}" height="{plot_h}" fill="none" stroke="#333" stroke-width="2"/>')
    lines.append(f'<line x1="{sx(0):.2f}" y1="{pad}" x2="{sx(0):.2f}" y2="{h-pad}" stroke="#666" stroke-width="1.5"/>')
    lines.append(f'<line x1="{pad}" y1="{sy(0):.2f}" x2="{w-pad}" y2="{sy(0):.2f}" stroke="#666" stroke-width="1.5"/>')

    # ideal QPSK reference points
    q = 1.0 / math.sqrt(2.0)
    for xi in (-q, q):
        for yi in (-q, q):
            lines.append(f'<circle cx="{sx(xi):.2f}" cy="{sy(yi):.2f}" r="6" fill="#111" opacity="0.55"/>')

    # points
    for p in points:
        color = colors.get(p.label, "#7f7f7f")
        lines.append(f'<circle cx="{sx(p.i):.2f}" cy="{sy(p.q):.2f}" r="3.2" fill="{color}" fill-opacity="0.55"/>')

    # labels
    lines.append(f'<text x="{w/2:.1f}" y="{h-18}" text-anchor="middle" font-size="24" font-family="Arial" font-weight="700">In-phase</text>')
    lines.append(f'<text x="28" y="{h/2:.1f}" text-anchor="middle" font-size="24" font-family="Arial" font-weight="700" transform="rotate(-90 28 {h/2:.1f})">Quadrature</text>')

    # stats
    mu_i, sd_i = mean_std([p.i for p in points])
    mu_q, sd_q = mean_std([p.q for p in points])
    mu_mag, sd_mag = mean_std([math.hypot(p.i, p.q) for p in points])
    stats = [
        f"I mean={mu_i:+.3f} sd={sd_i:.3f}",
        f"Q mean={mu_q:+.3f} sd={sd_q:.3f}",
        f"|s| mean={mu_mag:.3f} sd={sd_mag:.3f}",
        f"sym_rate={meta.get('sym_rate', 0):.0f} fc={meta.get('carrier_hz', 0):.0f}",
    ]
    y0 = h - pad + 28
    for idx, line in enumerate(stats):
        lines.append(f'<text x="{pad}" y="{y0 + idx*20:.1f}" font-size="15" font-family="Courier New">{line}</text>')

    # legend
    lx = w - pad - 160
    ly = pad + 20
    for idx, (name, color) in enumerate((("T/TRN", "#1166cc"), ("P/PP-ish", "#cc5500"), ("S", "#2b8a3e"))):
        y = ly + idx * 22
        lines.append(f'<circle cx="{lx:.2f}" cy="{y:.2f}" r="5" fill="{color}" fill-opacity="0.75"/>')
        lines.append(f'<text x="{lx + 12:.2f}" y="{y + 5:.2f}" font-size="14" font-family="Arial">{name}</text>')

    lines.append("</svg>")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="ascii")


def main() -> int:
    ap = argparse.ArgumentParser(description="Render IQ dump constellation as SVG")
    ap.add_argument("iq_dump", type=Path)
    ap.add_argument("--sym-start", type=int, default=None)
    ap.add_argument("--sym-end", type=int, default=None)
    ap.add_argument("--label", type=str, default=None)
    ap.add_argument("--title", type=str, default="IQ Constellation")
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    points, meta = load_iq_dump(args.iq_dump)
    sel = select_points(points, args.sym_start, args.sym_end, args.label)
    if not sel:
        raise SystemExit("no points selected")
    render_svg(sel, args.title, args.out, meta)
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
