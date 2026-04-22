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
import random
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


def _make_deterministic_bits(count: int, seed: int) -> list[int]:
    rng = random.Random(seed)
    return [rng.randint(0, 1) for _ in range(count)]


def simulate_spandsp_tx_symbols(
    bit_rate: int,
    bits: list[int],
    *,
    calling_party: bool,
    spandsp_maps: dict[int, list[tuple[float, float]]],
) -> list[tuple[float, float]]:
    """Simulate SpanDSP's V.32bis/V.17 TX symbol mapping for user-data mode only."""
    bits_per_symbol = {14400: 6, 12000: 5, 9600: 4, 7200: 3, 4800: 2}[bit_rate]
    if len(bits) % bits_per_symbol != 0:
        raise ValueError("bit count must be a multiple of bits_per_symbol")

    v32bis_4800_differential_encoder = (
        (2, 3, 0, 1),
        (0, 2, 1, 3),
        (3, 1, 2, 0),
        (1, 0, 3, 2),
    )
    v17_differential_encoder = (
        (0, 1, 2, 3),
        (1, 2, 3, 0),
        (2, 3, 0, 1),
        (3, 0, 1, 2),
    )
    v17_convolutional_encoder = (
        (0, 2, 3, 1),
        (4, 7, 5, 6),
        (1, 3, 2, 0),
        (7, 4, 6, 5),
        (2, 0, 1, 3),
        (6, 5, 7, 4),
        (3, 1, 0, 2),
        (5, 6, 4, 7),
    )

    scrambler_state = 0x2ECDD5
    scrambler_tap_index = 17 if calling_party else 4
    diff = 1
    convolution = 0
    constellation = spandsp_maps[bit_rate]
    emitted: list[tuple[float, float]] = []

    def _scramble(input_bit: int) -> int:
        nonlocal scrambler_state
        out_bit = (
            input_bit
            ^ ((scrambler_state >> scrambler_tap_index) & 1)
            ^ ((scrambler_state >> (23 - 1)) & 1)
        ) & 1
        scrambler_state = ((scrambler_state << 1) | out_bit) & ((1 << 23) - 1)
        return out_bit

    for offset in range(0, len(bits), bits_per_symbol):
        group = bits[offset:offset + bits_per_symbol]
        q = 0
        for index, bit in enumerate(group):
            q |= _scramble(bit) << index  # SpanDSP packs bits LSB-first.

        if bits_per_symbol == 2:
            diff = v32bis_4800_differential_encoder[diff][q & 0x03]
            codeword = diff
        else:
            diff = v17_differential_encoder[diff][q & 0x03]
            convolution = v17_convolutional_encoder[convolution][diff]
            codeword = ((q << 1) & 0x78) | (diff << 1) | ((convolution >> 2) & 1)
        emitted.append(constellation[codeword])
    return emitted


def simulate_python_tx_symbols(
    bit_rate: int,
    bits: list[int],
    *,
    calling_party: bool,
) -> list[tuple[float, float]]:
    """Run the Python reference data path on the same deterministic bit stream."""
    tap = scrambler_tap(calling_party=calling_party, transmit=True)
    from tools.v32bis_ref.scrambler import Scrambler
    from tools.v32bis_ref.data_mode import DataModeEncoder

    scrambled = Scrambler(tap).process_bits(bits)
    encoder = DataModeEncoder(bit_rate)
    return [tuple(map(float, point)) for point in encoder.encode(scrambled)]


def simulate_python_variant_symbols(
    bit_rate: int,
    bits: list[int],
    *,
    calling_party: bool,
    scrambler_register: int,
    prev_y_state: tuple[int, int],
    conv_state: int = 0,
) -> list[tuple[float, float]]:
    """Run the Python encoder with explicit startup state overrides."""
    from tools.v32bis_ref.scrambler import Scrambler
    from tools.v32bis_tcm import TrellisEncoder

    tap = scrambler_tap(calling_party=calling_party, transmit=True)
    scrambled = Scrambler(tap, register=scrambler_register).process_bits(bits)
    encoder = TrellisEncoder(bit_rate)
    encoder._prev_y1, encoder._prev_y2 = prev_y_state
    encoder._conv_state = conv_state
    return [tuple(map(float, point)) for point in encoder.encode_bits(scrambled)]


def _matching_prefix_length(left: list[tuple[float, float]], right: list[tuple[float, float]]) -> int:
    count = 0
    for lval, rval in zip(left, right):
        if lval != rval:
            break
        count += 1
    return count


def diagnose_emitted_symbol_mismatch(
    spandsp_maps: dict[int, list[tuple[float, float]]],
    *,
    symbol_count: int = 32,
    seed: int = 7,
    calling_party: bool = True,
) -> list[dict[str, object]]:
    """Try simple Python startup-state variants and report the closest match."""
    rows: list[dict[str, object]] = []
    variant_specs = [
        {"name": "python_default", "scrambler_register": 0, "prev_y_state": (0, 0), "conv_state": 0},
        {"name": "spandsp_scrambler_seed", "scrambler_register": 0x2ECDD5, "prev_y_state": (0, 0), "conv_state": 0},
        {"name": "spandsp_seed_diff00", "scrambler_register": 0x2ECDD5, "prev_y_state": (0, 0), "conv_state": 0},
        {"name": "spandsp_seed_diff01", "scrambler_register": 0x2ECDD5, "prev_y_state": (0, 1), "conv_state": 0},
        {"name": "spandsp_seed_diff10", "scrambler_register": 0x2ECDD5, "prev_y_state": (1, 0), "conv_state": 0},
        {"name": "spandsp_seed_diff11", "scrambler_register": 0x2ECDD5, "prev_y_state": (1, 1), "conv_state": 0},
    ]

    for rate in SUPPORTED_RATES:
        bits_per_symbol = {14400: 6, 12000: 5, 9600: 4, 7200: 3, 4800: 2}[rate]
        bits = _make_deterministic_bits(symbol_count * bits_per_symbol, seed)
        target = simulate_spandsp_tx_symbols(
            rate,
            bits,
            calling_party=calling_party,
            spandsp_maps=spandsp_maps,
        )
        best_variant: dict[str, object] | None = None
        for variant in variant_specs:
            trial = simulate_python_variant_symbols(
                rate,
                bits,
                calling_party=calling_party,
                scrambler_register=variant["scrambler_register"],
                prev_y_state=variant["prev_y_state"],
                conv_state=variant["conv_state"],
            )
            exact_matches = sum(sp == py for sp, py in zip(target, trial))
            prefix_matches = _matching_prefix_length(target, trial)
            result = {
                "name": variant["name"],
                "scrambler_register": variant["scrambler_register"],
                "prev_y_state": variant["prev_y_state"],
                "conv_state": variant["conv_state"],
                "exact_match_count": exact_matches,
                "prefix_match_count": prefix_matches,
                "first_symbol": trial[0] if trial else None,
            }
            if best_variant is None or (
                result["exact_match_count"],
                result["prefix_match_count"],
            ) > (
                best_variant["exact_match_count"],
                best_variant["prefix_match_count"],
            ):
                best_variant = result
        rows.append(
            {
                "rate": rate,
                "best_variant": best_variant,
            }
        )
    return rows


def compare_emitted_symbol_streams(
    spandsp_maps: dict[int, list[tuple[float, float]]],
    *,
    symbol_count: int = 32,
    seed: int = 7,
    calling_party: bool = True,
) -> list[dict[str, object]]:
    """Compare deterministic emitted symbol streams for user-data mode."""
    rows: list[dict[str, object]] = []
    for rate in SUPPORTED_RATES:
        bits_per_symbol = {14400: 6, 12000: 5, 9600: 4, 7200: 3, 4800: 2}[rate]
        bits = _make_deterministic_bits(symbol_count * bits_per_symbol, seed)
        spandsp_symbols = simulate_spandsp_tx_symbols(
            rate,
            bits,
            calling_party=calling_party,
            spandsp_maps=spandsp_maps,
        )
        python_symbols = simulate_python_tx_symbols(rate, bits, calling_party=calling_party)
        first_difference = None
        for index, (sp_point, py_point) in enumerate(zip(spandsp_symbols, python_symbols)):
            if sp_point != py_point:
                first_difference = {
                    "symbol_index": index,
                    "spandsp": sp_point,
                    "python": py_point,
                }
                break
        rows.append(
            {
                "rate": rate,
                "symbol_count": symbol_count,
                "same_stream": spandsp_symbols == python_symbols,
                "spandsp_first_symbols": spandsp_symbols[:8],
                "python_first_symbols": python_symbols[:8],
                "first_difference": first_difference,
            }
        )
    return rows


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
        "emitted_symbols": compare_emitted_symbol_streams(sp_maps),
        "emitted_symbol_diagnostics": diagnose_emitted_symbol_mismatch(sp_maps),
        "scrambler_taps": compare_scrambler_assignments(sp_taps, py_taps),
        "all_constellation_sets_match": all(row["same_set"] for row in constellation_rows),
        "all_constellation_orders_match": all(row["same_order"] for row in constellation_rows),
        "all_emitted_streams_match": all(row["same_stream"] for row in compare_emitted_symbol_streams(sp_maps)),
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
    lines.append("")
    lines.append("## Emitted Symbols")
    for row in report["emitted_symbols"]:
        lines.append(f"{row['rate']}: same_stream={row['same_stream']} symbol_count={row['symbol_count']}")
        if not row["same_stream"]:
            lines.append(f"  first_difference={row['first_difference']}")
    lines.append("")
    lines.append("## Emitted Symbol Diagnostics")
    for row in report["emitted_symbol_diagnostics"]:
        lines.append(f"{row['rate']}: best_variant={row['best_variant']}")
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
