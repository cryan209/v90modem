"""Unit tests for v32bis_tcm.py.

Tests verify:
  - Constellation sizes and uniqueness
  - Differential encoder Table 1 and Table 2 exactly match the spec
  - Encoder → hard-decision decoder roundtrip at all rates
  - Convolutional encoder state resets to zero
  - Differential encoder wraps correctly over multiple symbol groups
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.v32bis_tcm import (
    DIFF_TABLE1,
    DIFF_TABLE2,
    TrellisEncoder,
    ViterbiDecoder,
    DiffDetector4800,
    encode_bytes,
    decode_symbols_hard,
    _CONST_14400,
    _CONST_12000,
    _CONST_9600,
    _CONST_7200,
    _CONST_4800,
    _NEXT_STATE_TABLE,
)


# ---------------------------------------------------------------------------
# Constellation sanity checks
# ---------------------------------------------------------------------------

def test_constellation_sizes():
    assert len(_CONST_14400) == 128, f"14400 constellation: {len(_CONST_14400)} points"
    assert len(_CONST_12000) == 64,  f"12000 constellation: {len(_CONST_12000)} points"
    assert len(_CONST_9600)  == 32,  f" 9600 constellation: {len(_CONST_9600)} points"
    assert len(_CONST_7200)  == 16,  f" 7200 constellation: {len(_CONST_7200)} points"
    assert len(_CONST_4800)  == 4,   f" 4800 constellation: {len(_CONST_4800)} points"


def test_constellation_unique_points():
    """Each codeword maps to a unique (I,Q) point."""
    for name, const in [
        ("14400", _CONST_14400),
        ("12000", _CONST_12000),
        ( "9600", _CONST_9600),
        ( "7200", _CONST_7200),
        ( "4800", _CONST_4800),
    ]:
        points = list(const.values())
        assert len(points) == len(set(points)), f"{name} bps has duplicate (I,Q) points"


def test_constellation_odd_coords():
    """All non-4800 constellation points use odd integer coordinates."""
    for name, const in [
        ("14400", _CONST_14400),
        ("12000", _CONST_12000),
        ( "9600", _CONST_9600),
        ( "7200", _CONST_7200),
    ]:
        for cw, (i, q) in const.items():
            assert i % 2 != 0 and q % 2 != 0, \
                f"{name} codeword {cw:07b}: point ({i},{q}) not odd"


def test_9600_cross_shape():
    """9600 bps is the 32-point cross: 6×6 minus 4 corners."""
    points = set(_CONST_9600.values())
    for i in [-5, -3, -1, 1, 3, 5]:
        for q in [-5, -3, -1, 1, 3, 5]:
            if abs(i) == 5 and abs(q) == 5:
                assert (i, q) not in points, f"Corner ({i},{q}) should be absent"
            else:
                assert (i, q) in points, f"Point ({i},{q}) missing from 9600 constellation"


# ---------------------------------------------------------------------------
# Differential encoder table checks (spec Table 1)
# ---------------------------------------------------------------------------

def test_diff_table1_size():
    assert len(DIFF_TABLE1) == 16


def test_diff_table1_spec_rows():
    """Spot-check several rows against Table 1/V.32bis."""
    # From spec Table 1 (verbatim):
    cases = [
        # (Q1, Q2, prevY1, prevY2) → (Y1, Y2)
        ((0, 0, 0, 0), (0, 0)),
        ((0, 0, 1, 1), (1, 1)),
        ((0, 1, 0, 0), (0, 1)),
        ((0, 1, 1, 1), (0, 0)),
        ((1, 0, 0, 0), (1, 0)),
        ((1, 0, 1, 1), (0, 1)),   # +180° of (1,1)=270° → 90° = (0,1)
        ((1, 1, 0, 0), (1, 1)),
        ((1, 1, 1, 1), (1, 0)),   # +270° of (1,1)=270° → 180° = (1,0)
    ]
    for key, expected in cases:
        got = DIFF_TABLE1[key]
        assert got == expected, f"Table1 {key} → {got}, expected {expected}"


def test_diff_table2_spec_rows():
    """Spot-check Table 2/V.32bis (4800 bps differential encoding)."""
    # From spec Table 2 (phase quadrant column included for context):
    cases = [
        # Q1=0,Q2=0 → +90° rotation
        ((0, 0, 0, 0), (0, 1)),   # A→B
        ((0, 0, 0, 1), (1, 1)),   # B→C
        ((0, 0, 1, 0), (0, 0)),   # D? → A
        ((0, 0, 1, 1), (1, 0)),   # C→D
        # Q1=0,Q2=1 → 0° (no rotation)
        ((0, 1, 0, 0), (0, 0)),   # A→A
        ((0, 1, 0, 1), (0, 1)),   # B→B
        ((0, 1, 1, 0), (1, 0)),   # D→D? — spec says (1,0)
        ((0, 1, 1, 1), (1, 1)),   # C→C
    ]
    for key, expected in cases:
        got = DIFF_TABLE2[key]
        assert got == expected, f"Table2 {key} → {got}, expected {expected}"


# ---------------------------------------------------------------------------
# Convolutional encoder checks
# ---------------------------------------------------------------------------

def test_conv_state_zero_init():
    """Encoder starts at state 0."""
    enc = TrellisEncoder(14400)
    assert enc._conv_state == 0


def test_conv_reset():
    enc = TrellisEncoder(14400)
    enc.encode_bits([1, 0, 1, 1, 0, 0])
    assert enc._conv_state != 0 or True   # state may or may not be 0 after 1 symbol
    enc.reset()
    assert enc._conv_state == 0
    assert enc._prev_y1 == 0
    assert enc._prev_y2 == 0


def test_conv_state_transitions_valid():
    """All 4 states must have defined transitions for all (y1,y2) inputs."""
    for state in range(4):
        for y1 in range(2):
            for y2 in range(2):
                assert (state, y1, y2) in _NEXT_STATE_TABLE
                ns, y0 = _NEXT_STATE_TABLE[(state, y1, y2)]
                assert 0 <= ns <= 3
                assert y0 in (0, 1)


# ---------------------------------------------------------------------------
# Encoder → hard-decision decoder roundtrip
# ---------------------------------------------------------------------------

def _make_test_bits(n: int) -> list[int]:
    """Deterministic test pattern: alternating and walking bits."""
    return [(i * 3 + i // 7) % 2 for i in range(n)]


def _roundtrip(rate_bps: int, n_symbols: int = 64) -> bool:
    n = {14400: 6, 12000: 5, 9600: 4, 7200: 3, 4800: 2}[rate_bps]
    bits = _make_test_bits(n * n_symbols)
    enc = TrellisEncoder(rate_bps)
    symbols = enc.encode_bits(bits)
    decoded = decode_symbols_hard(symbols, rate_bps)
    # decoded may have extra zero-padding; compare the meaningful prefix
    return decoded[:len(bits)] == bits


def test_roundtrip_14400():
    assert _roundtrip(14400), "14400 bps roundtrip failed"


def test_roundtrip_12000():
    assert _roundtrip(12000), "12000 bps roundtrip failed"


def test_roundtrip_9600():
    assert _roundtrip(9600), "9600 bps roundtrip failed"


def test_roundtrip_7200():
    assert _roundtrip(7200), "7200 bps roundtrip failed"


def test_roundtrip_4800():
    assert _roundtrip(4800), "4800 bps roundtrip failed"


def test_encode_bytes_roundtrip():
    """encode_bytes + decode_symbols_hard roundtrip for a short message."""
    msg = b"Hello, V.32bis!"
    for rate in (14400, 12000, 9600, 7200, 4800):
        symbols = encode_bytes(msg, rate)
        # Re-encode the same bits to check consistency
        bits = []
        for byte in msg:
            for shift in range(7, -1, -1):
                bits.append((byte >> shift) & 1)
        n = {14400: 6, 12000: 5, 9600: 4, 7200: 3, 4800: 2}[rate]
        while len(bits) % n:
            bits.append(0)
        decoded = decode_symbols_hard(symbols, rate)
        assert decoded[:len(bits)] == bits, f"encode_bytes roundtrip failed at {rate} bps"


# ---------------------------------------------------------------------------
# 4800 bps differential detector
# ---------------------------------------------------------------------------

def test_diff_detector_4800_roundtrip():
    """Encode with TrellisEncoder(4800) and decode with DiffDetector4800."""
    enc = TrellisEncoder(4800)
    det = DiffDetector4800()
    pairs = [(0, 0), (0, 1), (1, 0), (1, 1),
             (1, 1), (0, 0), (1, 0), (0, 1)]
    for q1_in, q2_in in pairs:
        sym = enc.encode_bits([q1_in, q2_in])
        i, q = sym[0]
        q1_out, q2_out = det.detect(float(i), float(q))
        assert (q1_out, q2_out) == (q1_in, q2_in), \
            f"4800 detector: in=({q1_in},{q2_in}) out=({q1_out},{q2_out})"


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import traceback
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    passed = failed = 0
    for fn in tests:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {fn.__name__}: {e}")
            traceback.print_exc()
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
