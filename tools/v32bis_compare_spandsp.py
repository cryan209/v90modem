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
from tools.v32bis_ref.spec_policy import (
    POST_E_INITIAL_CONVOLUTION_STATE,
    startup_diff_state_from_final_trn_symbol,
    startup_scrambler_register_from_trn,
)
from tools.v32bis_ref.training import generate_conditioning_signal
from tools.v32bis_tcm import (
    DIFF_TABLE1,
    DIFF_TABLE2,
    _CONST_12000,
    _CONST_14400,
    _CONST_4800,
    _CONST_7200,
    _CONST_9600,
    _NEXT_STATE_TABLE,
)


SUPPORTED_RATES = (14400, 12000, 9600, 7200, 4800)
BITS_PER_SYMBOL = {14400: 6, 12000: 5, 9600: 4, 7200: 3, 4800: 2}
_Y_STATE_TO_INDEX = {(0, 0): 0, (0, 1): 1, (1, 0): 2, (1, 1): 3}
_INDEX_TO_Y_STATE = {value: key for key, value in _Y_STATE_TO_INDEX.items()}


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


def compare_encoder_state_tables() -> dict[str, object]:
    """Compare Python and SpanDSP differential/convolution transition tables."""
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

    diff_rows: list[dict[str, object]] = []
    for prev_diff in range(4):
        for q_dibit in range(4):
            sp_diff = v17_differential_encoder[prev_diff][q_dibit]
            y_prev = _INDEX_TO_Y_STATE[prev_diff]
            q1 = q_dibit & 0x01
            q2 = (q_dibit >> 1) & 0x01
            py_y = DIFF_TABLE1[(q1, q2, y_prev[0], y_prev[1])]
            py_diff = _Y_STATE_TO_INDEX[py_y]
            diff_rows.append(
                {
                    "prev_diff": prev_diff,
                    "q_dibit": q_dibit,
                    "spandsp_diff": sp_diff,
                    "python_diff": py_diff,
                    "matches": sp_diff == py_diff,
                }
            )

    conv_rows: list[dict[str, object]] = []
    for prev_conv in range(8):
        for diff_state in range(4):
            sp_conv = v17_convolutional_encoder[prev_conv][diff_state]
            y_state = _INDEX_TO_Y_STATE[diff_state]
            py_conv, py_y0 = _NEXT_STATE_TABLE[(prev_conv & 0x03, y_state[0], y_state[1])]
            conv_rows.append(
                {
                    "prev_convolution": prev_conv,
                    "diff_state": diff_state,
                    "spandsp_convolution": sp_conv,
                    "python_convolution": py_conv,
                    "python_y0": py_y0,
                    "matches": sp_conv == py_conv,
                }
            )

    return {
        "differential": {
            "all_match": all(row["matches"] for row in diff_rows),
            "mismatches": [row for row in diff_rows if not row["matches"]],
        },
        "convolution": {
            "all_match": all(row["matches"] for row in conv_rows),
            "mismatches": [row for row in conv_rows if not row["matches"]],
        },
    }


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
    initial_scrambler_register: int = 0x2ECDD5,
    initial_diff_state: int = 1,
    initial_convolution_state: int = 0,
) -> list[tuple[float, float]]:
    """Simulate SpanDSP's V.32bis/V.17 TX symbol mapping for user-data mode only."""
    bits_per_symbol = BITS_PER_SYMBOL[bit_rate]
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

    scrambler_state = initial_scrambler_register
    scrambler_tap_index = 17 if calling_party else 4
    diff = initial_diff_state
    convolution = initial_convolution_state
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


def trace_spandsp_tx_stages(
    bit_rate: int,
    bits: list[int],
    *,
    calling_party: bool,
    spandsp_maps: dict[int, list[tuple[float, float]]],
    initial_scrambler_register: int = 0x2ECDD5,
    initial_diff_state: int = 1,
    initial_convolution_state: int = 0,
) -> list[dict[str, object]]:
    """Trace SpanDSP's user-data TX stages symbol by symbol."""
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
    bits_per_symbol = BITS_PER_SYMBOL[bit_rate]
    scrambler_state = initial_scrambler_register
    scrambler_tap_index = 17 if calling_party else 4
    diff = initial_diff_state
    convolution = initial_convolution_state
    rows: list[dict[str, object]] = []

    def _scramble(input_bit: int) -> int:
        nonlocal scrambler_state
        out_bit = (
            input_bit
            ^ ((scrambler_state >> scrambler_tap_index) & 1)
            ^ ((scrambler_state >> (23 - 1)) & 1)
        ) & 1
        scrambler_state = ((scrambler_state << 1) | out_bit) & ((1 << 23) - 1)
        return out_bit

    for symbol_index, offset in enumerate(range(0, len(bits), bits_per_symbol)):
        input_bits = bits[offset:offset + bits_per_symbol]
        scrambled_bits = [_scramble(bit) for bit in input_bits]
        q = sum(bit << index for index, bit in enumerate(scrambled_bits))
        diff_in = diff
        if bits_per_symbol == 2:
            diff = v32bis_4800_differential_encoder[diff][q & 0x03]
            codeword = diff
            convolution_out = convolution
        else:
            diff = v17_differential_encoder[diff][q & 0x03]
            convolution = v17_convolutional_encoder[convolution][diff]
            convolution_out = convolution
            codeword = ((q << 1) & 0x78) | (diff << 1) | ((convolution >> 2) & 1)
        y_state = _INDEX_TO_Y_STATE[diff & 0x03]
        rows.append(
            {
                "symbol_index": symbol_index,
                "input_bits": input_bits,
                "scrambled_bits": scrambled_bits,
                "q": q,
                "diff_in": diff_in,
                "diff_out": diff,
                "y_state": y_state,
                "convolution_out": convolution_out,
                "codeword": codeword,
                "codeword_prefix": {
                    "y0": (codeword >> (bits_per_symbol - 1)) & 0x01 if bit_rate != 4800 else None,
                    "y1": y_state[0],
                    "y2": y_state[1],
                },
                "point": spandsp_maps[bit_rate][codeword],
            }
        )
    return rows


def simulate_spandsp_post_training_state(
    bit_rate: int,
    *,
    calling_party: bool,
    short_train: bool = False,
) -> dict[str, object]:
    """Simulate SpanDSP's transmitter state at the first real data symbol.

    This follows the control flow in ``v17_tx_restart``/``training_get``/``getbaud``
    closely enough to recover the scrambler, differential, and convolutional
    states that feed the first real user-data symbol after training.
    """
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
    v32bis_4800_differential_encoder = (
        (2, 3, 0, 1),
        (0, 2, 1, 3),
        (3, 1, 2, 0),
        (1, 0, 3, 2),
    )
    cdba_to_abcd = (2, 3, 1, 0)
    dibit_to_step = (1, 0, 2, 3)

    seg_tep_a = 0
    seg_tep_b = seg_tep_a + 480
    seg_1 = seg_tep_b + 48
    seg_2 = seg_1 + 256
    seg_3 = seg_2 + 2976
    seg_4 = seg_3 + 64
    seg_short_4 = seg_2 + 38
    seg_end = seg_4 + 48
    bridge_word = 0x8880

    bits_per_symbol = BITS_PER_SYMBOL[bit_rate]
    scrambler_state = 0x2ECDD5
    scrambler_tap_index = 17 if calling_party else 4
    diff = 0 if short_train else 1
    convolution = 0
    constellation_state = 0
    in_training = True
    training_step = seg_1

    def _scramble(input_bit: int) -> int:
        nonlocal scrambler_state
        out_bit = (
            input_bit
            ^ ((scrambler_state >> scrambler_tap_index) & 1)
            ^ ((scrambler_state >> (23 - 1)) & 1)
        ) & 1
        scrambler_state = ((scrambler_state << 1) | out_bit) & ((1 << 23) - 1)
        return out_bit

    while in_training:
        if training_step <= seg_end:
            if training_step < seg_4:
                training_step += 1
                if training_step <= seg_3:
                    if training_step <= seg_2:
                        if training_step <= seg_1:
                            continue
                        bits = _scramble(1)
                        bits = (bits << 1) | _scramble(1)
                        constellation_state = cdba_to_abcd[bits]
                        if short_train and training_step == seg_short_4:
                            training_step = seg_4
                        continue
                    shift = ((training_step - seg_3 - 1) & 0x7) << 1
                    bits = _scramble((bridge_word >> shift) & 1)
                    bits = (bits << 1) | _scramble((bridge_word >> (shift + 1)) & 1)
                    constellation_state = (constellation_state + dibit_to_step[bits]) & 3
                    continue
            training_step += 1
            if training_step > seg_end:
                in_training = False
                break
            # Segment 4 sends fake_get_bit()=1 through the normal data path.
            q = 0
            for bit_index in range(bits_per_symbol):
                q |= _scramble(1) << bit_index
            if bits_per_symbol == 2:
                diff = v32bis_4800_differential_encoder[diff][q & 0x03]
            else:
                diff = v17_differential_encoder[diff][q & 0x03]
                convolution = v17_convolutional_encoder[convolution][diff]
        else:
            break

    return {
        "scrambler_register": scrambler_state,
        "diff": diff,
        "convolution": convolution,
        "constellation_state": constellation_state,
        "bits_per_symbol": bits_per_symbol,
    }


def trace_python_tx_stages(
    bit_rate: int,
    bits: list[int],
    *,
    calling_party: bool,
    scrambler_register: int = 0,
    prev_y_state: tuple[int, int] = (0, 0),
    conv_state: int = 0,
) -> list[dict[str, object]]:
    """Trace the Python user-data TX stages symbol by symbol."""
    from tools.v32bis_ref.scrambler import Scrambler

    const_map = {
        14400: _CONST_14400,
        12000: _CONST_12000,
        9600: _CONST_9600,
        7200: _CONST_7200,
        4800: _CONST_4800,
    }[bit_rate]
    bits_per_symbol = BITS_PER_SYMBOL[bit_rate]
    scrambler = Scrambler(scrambler_tap(calling_party=calling_party, transmit=True), register=scrambler_register)
    prev_y1, prev_y2 = prev_y_state
    rows: list[dict[str, object]] = []

    for symbol_index, offset in enumerate(range(0, len(bits), bits_per_symbol)):
        input_bits = bits[offset:offset + bits_per_symbol]
        scrambled_bits = [scrambler.process_bit(bit) for bit in input_bits]
        if bit_rate == 4800:
            q1, q2 = scrambled_bits[0], scrambled_bits[1]
            diff_in = _Y_STATE_TO_INDEX[(prev_y1, prev_y2)]
            y1, y2 = DIFF_TABLE2[(q1, q2, prev_y1, prev_y2)]
            prev_y1, prev_y2 = y1, y2
            diff_out = _Y_STATE_TO_INDEX[(y1, y2)]
            codeword = (y1 << 1) | y2
            convolution_out = conv_state
            y0 = None
        else:
            q1, q2 = scrambled_bits[0], scrambled_bits[1]
            q_extra = scrambled_bits[2:]
            diff_in = _Y_STATE_TO_INDEX[(prev_y1, prev_y2)]
            y1, y2 = DIFF_TABLE1[(q1, q2, prev_y1, prev_y2)]
            prev_y1, prev_y2 = y1, y2
            diff_out = _Y_STATE_TO_INDEX[(y1, y2)]
            conv_state, y0 = _NEXT_STATE_TABLE[(conv_state, y1, y2)]
            convolution_out = conv_state
            n_bits = 1 + 2 + len(q_extra)
            codeword = (y0 << (n_bits - 1)) | (y1 << (n_bits - 2)) | (y2 << (n_bits - 3))
            for index, qbit in enumerate(q_extra):
                codeword |= qbit << (n_bits - 4 - index)
        q_value_lsb = sum(bit << index for index, bit in enumerate(scrambled_bits))
        q_value_msb = sum(bit << (bits_per_symbol - 1 - index) for index, bit in enumerate(scrambled_bits))
        spandsp_style_codeword = codeword
        if bit_rate != 4800:
            spandsp_style_codeword = ((q_value_lsb << 1) & 0x78) | (diff_out << 1) | ((convolution_out >> 2) & 0x01)
        rows.append(
            {
                "symbol_index": symbol_index,
                "input_bits": input_bits,
                "scrambled_bits": scrambled_bits,
                "q": q_value_msb,
                "q_lsb": q_value_lsb,
                "q_msb": q_value_msb,
                "diff_in": diff_in,
                "diff_out": diff_out,
                "y_state": (y1, y2),
                "convolution_out": convolution_out,
                "codeword": codeword,
                "spandsp_style_codeword": spandsp_style_codeword,
                "codeword_prefix": {
                    "y0": y0,
                    "y1": y1,
                    "y2": y2,
                },
                "point": tuple(map(float, const_map[codeword])),
            }
        )
    return rows


def build_stage_diagnostics(
    spandsp_maps: dict[int, list[tuple[float, float]]],
    *,
    rate: int = 12000,
    symbol_count: int = 8,
    seed: int = 7,
    calling_party: bool = True,
) -> dict[str, object]:
    """Build a stage-by-stage diagnostic trace for the easiest narrowing target."""
    bits = _make_deterministic_bits(symbol_count * BITS_PER_SYMBOL[rate], seed)
    sp_rows = trace_spandsp_tx_stages(rate, bits, calling_party=calling_party, spandsp_maps=spandsp_maps)
    py_rows = trace_python_tx_stages(rate, bits, calling_party=calling_party)
    post_training = simulate_spandsp_post_training_state(rate, calling_party=calling_party)
    py_post_rows = trace_python_tx_stages(
        rate,
        bits,
        calling_party=calling_party,
        scrambler_register=int(post_training["scrambler_register"]),
        prev_y_state=_INDEX_TO_Y_STATE[int(post_training["diff"]) & 0x03],
        conv_state=0,
    )
    stage_names = ("input_bits", "scrambled_bits", "q", "diff_in", "diff_out", "convolution_out", "codeword", "point")
    def _first_divergence(compare_rows: list[dict[str, object]]) -> dict[str, object] | None:
        for sp_row, py_row in zip(sp_rows, compare_rows):
            for stage_name in stage_names:
                if sp_row[stage_name] != py_row[stage_name]:
                    return {
                        "symbol_index": sp_row["symbol_index"],
                        "stage": stage_name,
                        "spandsp": sp_row[stage_name],
                        "python": py_row[stage_name],
                    }
        return None

    first_stage_divergence = _first_divergence(py_rows)
    first_stage_divergence_post_training = _first_divergence(py_post_rows)
    return {
        "rate": rate,
        "symbol_count": symbol_count,
        "first_stage_divergence": first_stage_divergence,
        "spandsp_post_training_state": post_training,
        "first_stage_divergence_with_post_training_seed": first_stage_divergence_post_training,
        "spandsp_rows": sp_rows,
        "python_rows": py_rows,
        "python_post_training_seed_rows": py_post_rows,
    }


def build_startup_handoff_diagnostics(
    spandsp_maps: dict[int, list[tuple[float, float]]],
    *,
    rate: int = 12000,
    symbol_count: int = 8,
    calling_party: bool = True,
    trn_length: int = 1280,
) -> dict[str, object]:
    """Compare post-training/post-E handoff seeds and first scrambled-ones symbols."""
    conditioning = generate_conditioning_signal(calling_party, trn_length)
    python_spec_state = {
        "scrambler_register": startup_scrambler_register_from_trn(conditioning.trn_final_scrambler_register),
        "diff": startup_diff_state_from_final_trn_symbol(conditioning.final_trn_symbol),
        "convolution": POST_E_INITIAL_CONVOLUTION_STATE,
        "final_trn_symbol": conditioning.final_trn_symbol,
        "bits_per_symbol": BITS_PER_SYMBOL[rate],
    }
    spandsp_post_training_state = simulate_spandsp_post_training_state(rate, calling_party=calling_party)
    ones_bits = [1] * (symbol_count * BITS_PER_SYMBOL[rate])

    sp_rows = trace_spandsp_tx_stages(
        rate,
        ones_bits,
        calling_party=calling_party,
        spandsp_maps=spandsp_maps,
        initial_scrambler_register=int(spandsp_post_training_state["scrambler_register"]),
        initial_diff_state=int(spandsp_post_training_state["diff"]),
        initial_convolution_state=int(spandsp_post_training_state["convolution"]),
    )
    py_spec_rows = trace_python_tx_stages(
        rate,
        ones_bits,
        calling_party=calling_party,
        scrambler_register=int(python_spec_state["scrambler_register"]),
        prev_y_state=_INDEX_TO_Y_STATE[int(python_spec_state["diff"]) & 0x03],
        conv_state=int(python_spec_state["convolution"]),
    )
    py_spandsp_rows = trace_python_tx_stages(
        rate,
        ones_bits,
        calling_party=calling_party,
        scrambler_register=int(spandsp_post_training_state["scrambler_register"]),
        prev_y_state=_INDEX_TO_Y_STATE[int(spandsp_post_training_state["diff"]) & 0x03],
        conv_state=0,
    )

    stage_names = ("scrambled_bits", "q", "diff_in", "diff_out", "convolution_out", "codeword", "point")

    def _first_divergence(compare_rows: list[dict[str, object]]) -> dict[str, object] | None:
        for sp_row, py_row in zip(sp_rows, compare_rows):
            for stage_name in stage_names:
                if sp_row[stage_name] != py_row[stage_name]:
                    return {
                        "symbol_index": sp_row["symbol_index"],
                        "stage": stage_name,
                        "spandsp": sp_row[stage_name],
                        "python": py_row[stage_name],
                    }
        return None

    grouping_diagnostics: list[dict[str, object]] = []
    for sp_row, py_row in zip(sp_rows, py_spandsp_rows):
        grouping_diagnostics.append(
            {
                "symbol_index": sp_row["symbol_index"],
                "scrambled_bits_match": sp_row["scrambled_bits"] == py_row["scrambled_bits"],
                "spandsp_q": sp_row["q"],
                "python_q_lsb": py_row["q_lsb"],
                "python_q_msb": py_row["q_msb"],
                "q_matches_python_lsb": sp_row["q"] == py_row["q_lsb"],
                "q_matches_python_msb": sp_row["q"] == py_row["q_msb"],
                "spandsp_y_state": sp_row["y_state"],
                "python_y_state": py_row["y_state"],
                "y_state_matches": sp_row["y_state"] == py_row["y_state"],
                "spandsp_diff_in": sp_row["diff_in"],
                "python_diff_in": py_row["diff_in"],
                "diff_in_matches": sp_row["diff_in"] == py_row["diff_in"],
                "spandsp_diff_out": sp_row["diff_out"],
                "python_diff_out": py_row["diff_out"],
                "diff_out_matches": sp_row["diff_out"] == py_row["diff_out"],
                "spandsp_convolution_out": sp_row["convolution_out"],
                "python_convolution_out": py_row["convolution_out"],
                "convolution_out_matches": sp_row["convolution_out"] == py_row["convolution_out"],
                "spandsp_codeword_prefix": sp_row["codeword_prefix"],
                "python_codeword_prefix": py_row["codeword_prefix"],
                "codeword_prefix_matches": sp_row["codeword_prefix"] == py_row["codeword_prefix"],
                "spandsp_codeword": sp_row["codeword"],
                "python_codeword": py_row["codeword"],
                "python_spandsp_style_codeword": py_row["spandsp_style_codeword"],
                "codeword_matches_python_native": sp_row["codeword"] == py_row["codeword"],
                "codeword_matches_python_spandsp_style": sp_row["codeword"] == py_row["spandsp_style_codeword"],
            }
        )

    return {
        "rate": rate,
        "symbol_count": symbol_count,
        "python_spec_state": python_spec_state,
        "spandsp_post_training_state": spandsp_post_training_state,
        "first_divergence_python_spec_seed": _first_divergence(py_spec_rows),
        "first_divergence_python_spandsp_seed": _first_divergence(py_spandsp_rows),
        "grouping_diagnostics_python_spandsp_seed": grouping_diagnostics,
        "spandsp_rows": sp_rows,
        "python_spec_rows": py_spec_rows,
        "python_spandsp_seed_rows": py_spandsp_rows,
    }


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
        "encoder_state_tables": compare_encoder_state_tables(),
        "emitted_symbols": compare_emitted_symbol_streams(sp_maps),
        "emitted_symbol_diagnostics": diagnose_emitted_symbol_mismatch(sp_maps),
        "stage_diagnostics": build_stage_diagnostics(sp_maps),
        "startup_handoff_diagnostics": build_startup_handoff_diagnostics(sp_maps),
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
    lines.append("## Encoder State Tables")
    encoder_tables = report["encoder_state_tables"]
    lines.append(
        f"differential_all_match={encoder_tables['differential']['all_match']} "
        f"convolution_all_match={encoder_tables['convolution']['all_match']}"
    )
    if not encoder_tables["differential"]["all_match"]:
        lines.append(f"  differential_mismatches={encoder_tables['differential']['mismatches'][:8]}")
    if not encoder_tables["convolution"]["all_match"]:
        lines.append(f"  convolution_mismatches={encoder_tables['convolution']['mismatches'][:8]}")
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
    lines.append("")
    lines.append("## Stage Diagnostics")
    stage = report["stage_diagnostics"]
    lines.append(
        f"rate={stage['rate']} symbol_count={stage['symbol_count']} "
        f"first_stage_divergence={stage['first_stage_divergence']}"
    )
    lines.append(
        f"spandsp_post_training_state={stage['spandsp_post_training_state']} "
        f"first_stage_divergence_with_post_training_seed={stage['first_stage_divergence_with_post_training_seed']}"
    )
    lines.append("")
    lines.append("## Startup Handoff Diagnostics")
    handoff = report["startup_handoff_diagnostics"]
    lines.append(
        f"rate={handoff['rate']} symbol_count={handoff['symbol_count']} "
        f"python_spec_state={handoff['python_spec_state']}"
    )
    lines.append(
        f"spandsp_post_training_state={handoff['spandsp_post_training_state']} "
        f"first_divergence_python_spec_seed={handoff['first_divergence_python_spec_seed']} "
        f"first_divergence_python_spandsp_seed={handoff['first_divergence_python_spandsp_seed']}"
    )
    for row in handoff["grouping_diagnostics_python_spandsp_seed"][:3]:
        lines.append(
            "  "
            f"symbol={row['symbol_index']} "
            f"q(sp)={row['spandsp_q']} q(py_lsb)={row['python_q_lsb']} q(py_msb)={row['python_q_msb']} "
            f"q_matches_lsb={row['q_matches_python_lsb']} q_matches_msb={row['q_matches_python_msb']} "
            f"diff_in(sp)={row['spandsp_diff_in']} diff_in(py)={row['python_diff_in']} "
            f"diff_in_match={row['diff_in_matches']} "
            f"diff_out(sp)={row['spandsp_diff_out']} diff_out(py)={row['python_diff_out']} "
            f"diff_out_match={row['diff_out_matches']} "
            f"conv(sp)={row['spandsp_convolution_out']} conv(py)={row['python_convolution_out']} "
            f"conv_match={row['convolution_out_matches']} "
            f"y(sp)={row['spandsp_y_state']} y(py)={row['python_y_state']} "
            f"y_match={row['y_state_matches']} "
            f"prefix(sp)={row['spandsp_codeword_prefix']} prefix(py)={row['python_codeword_prefix']} "
            f"prefix_match={row['codeword_prefix_matches']} "
            f"cw(sp)={row['spandsp_codeword']} cw(py)={row['python_codeword']} "
            f"cw(py_sp_style)={row['python_spandsp_style_codeword']} "
            f"cw_match_py={row['codeword_matches_python_native']} "
            f"cw_match_py_sp_style={row['codeword_matches_python_spandsp_style']}"
        )
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
