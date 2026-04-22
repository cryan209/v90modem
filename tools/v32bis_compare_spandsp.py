#!/usr/bin/env python3
"""Compare the local Python V.32bis reference against the bundled SpanDSP tree.

This is intentionally an offline reference harness, not an interop test. It
compares static implementation artefacts that are available locally today:

- SpanDSP constellation tables in ``spandsp-master/src/v17_v32bis_tx_constellation_maps.h``
- SpanDSP V.32bis scrambler-direction assignments in ``spandsp-master/src/v32bis.c``

The tool prints a human-readable summary by default and can emit JSON for
automation. It exits successfully unless an explicit ``--require-*`` option is
used.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.v32bis_ref.scrambler import scrambler_tap
from tools.v32bis_tcm import _CONST_12000, _CONST_14400, _CONST_4800, _CONST_7200, _CONST_9600


SUPPORTED_RATES = (14400, 12000, 9600, 7200, 4800)


def _extract_brace_block(text: str, start_index: int) -> str:
    """Return the contents of the first top-level ``{...}`` block after *start_index*."""
    open_index = text.find("{", start_index)
    if open_index < 0:
        raise ValueError("opening brace not found")
    depth = 0
    for index in range(open_index, len(text)):
        char = text[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[open_index + 1:index]
    raise ValueError("unterminated brace block")


def parse_spandsp_constellation_maps(header_path: Path) -> dict[int, list[tuple[float, float]]]:
    """Parse SpanDSP's V.32bis TX constellation tables."""
    text = header_path.read_text()
    parsed: dict[int, list[tuple[float, float]]] = {}
    point_pattern = re.compile(
        r"\{FP_CONSTELLATION_SCALE\(\s*([-0-9.]+)f\),\s*FP_CONSTELLATION_SCALE\(\s*([-0-9.]+)f\)\}"
    )
    for rate in SUPPORTED_RATES:
        marker = f"v17_v32bis_{rate}_constellation["
        start = text.find(marker)
        if start < 0:
            raise ValueError(f"could not find {marker} in {header_path}")
        body = _extract_brace_block(text, start)
        points = [(float(i_text), float(q_text)) for i_text, q_text in point_pattern.findall(body)]
        parsed[rate] = points
    return parsed


def parse_spandsp_scrambler_taps(c_path: Path) -> dict[str, int]:
    """Parse SpanDSP's caller/answerer scrambler-tap assignments."""
    text = c_path.read_text()
    patterns = {
        "calling_tx": r"s->tx\.scrambler_tap\s*=\s*(\d+);",
        "calling_rx": r"s->rx\.scrambler_tap\s*=\s*(\d+);",
    }
    if_match = re.search(
        r"if\s*\(s->calling_party\)\s*\{(?P<if_body>.*?)\}\s*else\s*\{(?P<else_body>.*?)\}",
        text,
        re.S,
    )
    if if_match is None:
        raise ValueError(f"could not find calling_party scrambler assignment in {c_path}")

    def _extract(body: str, pattern: str) -> int:
        match = re.search(pattern, body)
        if match is None:
            raise ValueError(f"could not parse scrambler tap from body: {pattern}")
        return int(match.group(1))

    if_body = if_match.group("if_body")
    else_body = if_match.group("else_body")
    return {
        "calling_tx": _extract(if_body, patterns["calling_tx"]),
        "calling_rx": _extract(if_body, patterns["calling_rx"]),
        "answering_tx": _extract(else_body, patterns["calling_tx"]),
        "answering_rx": _extract(else_body, patterns["calling_rx"]),
    }


def python_constellation_maps() -> dict[int, list[tuple[float, float]]]:
    """Return the Python constellation tables in ascending codeword order."""
    tables = {
        14400: _CONST_14400,
        12000: _CONST_12000,
        9600: _CONST_9600,
        7200: _CONST_7200,
        4800: _CONST_4800,
    }
    return {
        rate: [tuple(map(float, table[index])) for index in sorted(table)]
        for rate, table in tables.items()
    }


def python_scrambler_taps() -> dict[str, int]:
    """Return the Python caller/answerer scrambler-direction assignments."""
    return {
        "calling_tx": scrambler_tap(calling_party=True, transmit=True),
        "calling_rx": scrambler_tap(calling_party=True, transmit=False),
        "answering_tx": scrambler_tap(calling_party=False, transmit=True),
        "answering_rx": scrambler_tap(calling_party=False, transmit=False),
    }


def compare_constellations(
    spandsp_maps: dict[int, list[tuple[float, float]]],
    python_maps: dict[int, list[tuple[float, float]]],
) -> list[dict[str, object]]:
    """Build a structured per-rate constellation comparison."""
    rows: list[dict[str, object]] = []
    for rate in SUPPORTED_RATES:
        sp_points = spandsp_maps[rate]
        py_points = python_maps[rate]
        sp_set = set(sp_points)
        py_set = set(py_points)
        rows.append(
            {
                "rate": rate,
                "spandsp_count": len(sp_points),
                "python_count": len(py_points),
                "same_order": sp_points == py_points,
                "same_set": sp_set == py_set,
                "python_only_examples": sorted(py_set - sp_set)[:8],
                "spandsp_only_examples": sorted(sp_set - py_set)[:8],
                "python_first_points": py_points[:8],
                "spandsp_first_points": sp_points[:8],
            }
        )
    return rows


def compare_scrambler_assignments(
    spandsp_taps: dict[str, int],
    python_taps: dict[str, int],
) -> dict[str, object]:
    """Compare caller/answerer scrambler-direction assignments."""
    mismatches = {
        key: {"spandsp": spandsp_taps[key], "python": python_taps[key]}
        for key in spandsp_taps
        if spandsp_taps[key] != python_taps[key]
    }
    return {
        "same": not mismatches,
        "spandsp": spandsp_taps,
        "python": python_taps,
        "mismatches": mismatches,
    }


def build_report(header_path: Path, c_path: Path) -> dict[str, object]:
    """Build the complete SpanDSP comparison report."""
    sp_maps = parse_spandsp_constellation_maps(header_path)
    py_maps = python_constellation_maps()
    sp_taps = parse_spandsp_scrambler_taps(c_path)
    py_taps = python_scrambler_taps()
    constellation_rows = compare_constellations(sp_maps, py_maps)
    return {
        "spandsp_header": str(header_path),
        "spandsp_c_file": str(c_path),
        "constellations": constellation_rows,
        "scrambler_taps": compare_scrambler_assignments(sp_taps, py_taps),
        "all_constellation_sets_match": all(row["same_set"] for row in constellation_rows),
        "all_constellation_orders_match": all(row["same_order"] for row in constellation_rows),
    }


def _format_text_report(report: dict[str, object]) -> str:
    lines = [
        "# V.32bis Python vs SpanDSP",
        f"header: {report['spandsp_header']}",
        f"c_file: {report['spandsp_c_file']}",
        "",
        "## Scrambler taps",
    ]
    scrambler = report["scrambler_taps"]
    lines.append(f"same: {scrambler['same']}")
    for key in ("calling_tx", "calling_rx", "answering_tx", "answering_rx"):
        lines.append(
            f"{key}: spandsp={scrambler['spandsp'][key]} python={scrambler['python'][key]}"
        )
    lines.append("")
    lines.append("## Constellations")
    for row in report["constellations"]:
        lines.append(
            f"{row['rate']}: same_order={row['same_order']} same_set={row['same_set']} "
            f"spandsp_count={row['spandsp_count']} python_count={row['python_count']}"
        )
        if not row["same_set"]:
            lines.append(f"  python_only_examples={row['python_only_examples']}")
            lines.append(f"  spandsp_only_examples={row['spandsp_only_examples']}")
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Compare the Python V.32bis reference against local SpanDSP artefacts")
    parser.add_argument(
        "--spandsp-header",
        type=Path,
        default=REPO_ROOT / "spandsp-master/src/v17_v32bis_tx_constellation_maps.h",
    )
    parser.add_argument(
        "--spandsp-c",
        type=Path,
        default=REPO_ROOT / "spandsp-master/src/v32bis.c",
    )
    parser.add_argument("--json", action="store_true", help="emit JSON instead of text")
    parser.add_argument(
        "--require-constellation-set-match",
        action="store_true",
        help="exit non-zero if any constellation point set differs",
    )
    parser.add_argument(
        "--require-scrambler-match",
        action="store_true",
        help="exit non-zero if scrambler direction assignments differ",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    report = build_report(args.spandsp_header, args.spandsp_c)

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(_format_text_report(report))

    if args.require_constellation_set_match and not report["all_constellation_sets_match"]:
        return 1
    if args.require_scrambler_match and not report["scrambler_taps"]["same"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
