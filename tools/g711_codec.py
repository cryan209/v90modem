#!/usr/bin/env python3
"""Standalone G.711 A-law/mu-law codec (numpy, table-based).

Python 3.13 dropped the stdlib `audioop` module this project used to lean on
for quick A-law/mu-law conversions; this reimplements the standard ITU-T
G.711 reference algorithm so tools/voice_pcm_fidelity.py doesn't depend on it.
"""
from __future__ import annotations

import numpy as np

CLIP = 32635


def _nearest_codeword_encoder(decode_table: np.ndarray):
    """Build a vectorized encoder as the inverse of a (correct) decode table:
    for each input sample, pick the codeword whose decoded value is closest.
    Avoids re-deriving the segment/bias encode algorithm by hand."""
    order = np.argsort(decode_table)
    sorted_values = decode_table[order].astype(np.int32)
    sorted_codes = order.astype(np.uint8)
    midpoints = (sorted_values[:-1].astype(np.int64) + sorted_values[1:].astype(np.int64)) // 2

    def encode(samples: np.ndarray) -> np.ndarray:
        pcm = np.clip(samples.astype(np.int32), -32768, 32767)
        idx = np.searchsorted(midpoints, pcm)
        return sorted_codes[idx]

    return encode


def _alaw_decode_table() -> np.ndarray:
    table = np.zeros(256, dtype=np.int16)
    for i in range(256):
        a_val = i ^ 0x55
        t = (a_val & 0x0F) << 4
        seg = (a_val & 0x70) >> 4
        if seg == 0:
            t += 8
        elif seg == 1:
            t += 0x108
        else:
            t += 0x108
            t <<= seg - 1
        table[i] = t if (a_val & 0x80) else -t
    return table


_ALAW_DECODE = _alaw_decode_table()
linear_to_alaw = _nearest_codeword_encoder(_ALAW_DECODE)


def alaw_to_linear(codewords: np.ndarray) -> np.ndarray:
    """A-law octets -> int16 linear PCM (numpy, table lookup)."""
    return _ALAW_DECODE[codewords.astype(np.uint8)]


def _ulaw_decode_table() -> np.ndarray:
    table = np.zeros(256, dtype=np.int16)
    for i in range(256):
        u_val = (~i) & 0xFF
        t = ((u_val & 0x0F) << 3) + 0x84
        t <<= (u_val & 0x70) >> 4
        t -= 0x84
        table[i] = -t if (u_val & 0x80) else t
    return table


_ULAW_DECODE = _ulaw_decode_table()


def ulaw_to_linear(codewords: np.ndarray) -> np.ndarray:
    return _ULAW_DECODE[codewords.astype(np.uint8)]


linear_to_ulaw = _nearest_codeword_encoder(_ULAW_DECODE)


def self_test() -> None:
    ramp = np.arange(-32768, 32768, 17, dtype=np.int16)
    a = linear_to_alaw(ramp)
    back = alaw_to_linear(a)
    err = np.abs(ramp.astype(np.int32) - back.astype(np.int32))
    assert np.max(err) < 1200, f"A-law round-trip error too large: {np.max(err)}"
    u = linear_to_ulaw(ramp)
    backu = ulaw_to_linear(u)
    erru = np.abs(ramp.astype(np.int32) - backu.astype(np.int32))
    assert np.max(erru) < 1200, f"u-law round-trip error too large: {np.max(erru)}"
    print("g711_codec self-test OK")


if __name__ == "__main__":
    self_test()
