"""V.32bis data-mode TCM encoder and Viterbi decoder.

This module wires the spec-faithful ``tools.v32bis_tcm`` implementation into
the ``v32bis_ref`` package, exposing a consistent interface alongside the
startup-phase coding helpers in :mod:`.coding`.

Key differences from :mod:`.coding`:
- Uses the spec's odd-integer (I,Q) coordinate grid (±1, ±3, … ±11), not the
  SpanDSP even-float grid used by the startup/4800-DPSK path.
- Differential encoder follows V.32bis Table 1/2 phase-rotation convention
  directly (not the modular-addition state-machine in ``coding.py``).
- Convolutional encoder is the 4-state circuit from Figure 1.
- Viterbi decoder provides soft-decision decoding for the trellis rates.

All five data rates are supported:
  14400 bps — 128-QAM TCM  (6 data bits per symbol)
  12000 bps —  64-QAM TCM  (5 data bits per symbol)
   9600 bps —  32-QAM TCM  (4 data bits per symbol)
   7200 bps —  16-QAM TCM  (3 data bits per symbol)
   4800 bps —   4-DPSK      (2 data bits per symbol, no trellis)
"""

from __future__ import annotations

import sys
import os

# Allow importing tcm from the tools/ directory when the package is used
# directly (without installing).
_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)

from tools.v32bis_tcm import (  # noqa: E402
    TrellisEncoder as _TrellisEncoder,
    ViterbiDecoder as _ViterbiDecoder,
    DiffDetector4800 as _DiffDetector4800,
    decode_symbols_hard as _decode_symbols_hard,
    encode_bytes as _encode_bytes,
    DIFF_TABLE1,
    DIFF_TABLE2,
    _RATE_INFO,
    _NEXT_STATE_TABLE,
)


__all__ = [
    "SUPPORTED_DATA_RATES",
    "DataModeEncoder",
    "DataModeDecoder",
    "encode_data_bits",
    "decode_data_symbols_hard",
    "encode_data_bytes",
    "DIFF_TABLE1",
    "DIFF_TABLE2",
]

SUPPORTED_DATA_RATES = (4800, 7200, 9600, 12000, 14400)


class DataModeEncoder:
    """Stateful V.32bis data-mode trellis encoder.

    Wraps :class:`tools.v32bis_tcm.TrellisEncoder` with a ref-package-
    compatible interface.

    Parameters
    ----------
    bit_rate:
        One of 4800, 7200, 9600, 12000, or 14400 bps.

    Examples
    --------
    >>> enc = DataModeEncoder(9600)
    >>> symbols = enc.encode([0, 1, 0, 1, 1, 0, 0, 1])  # 2 groups of 4
    >>> len(symbols)
    2

    Each symbol is a ``(I, Q)`` tuple of odd integers (spec grid).
    """

    def __init__(self, bit_rate: int = 14400) -> None:
        if bit_rate not in SUPPORTED_DATA_RATES:
            raise ValueError(
                f"unsupported data rate {bit_rate}; valid: {SUPPORTED_DATA_RATES}"
            )
        self.bit_rate = bit_rate
        self._bits_per_group: int = _RATE_INFO[bit_rate][0]
        self._enc = _TrellisEncoder(bit_rate)

    @property
    def bits_per_group(self) -> int:
        """Number of data bits consumed per transmitted symbol."""
        return self._bits_per_group

    def reset(self) -> None:
        """Reset encoder state (call before each new data burst per §6.1)."""
        self._enc.reset()

    def encode(self, bits: list[int]) -> list[tuple[int, int]]:
        """Encode *bits* to a list of ``(I, Q)`` constellation points.

        ``len(bits)`` must be a non-zero multiple of :attr:`bits_per_group`.
        Returns one ``(I, Q)`` tuple per symbol group.
        """
        return self._enc.encode_bits(bits)


class DataModeDecoder:
    """V.32bis data-mode decoder (hard-decision or Viterbi).

    For 4800 bps uses a differential QPSK detector.
    For 7200–14400 bps offers both hard-decision (for testing) and
    soft-decision Viterbi decoding.

    Parameters
    ----------
    bit_rate:
        One of 4800, 7200, 9600, 12000, or 14400 bps.
    soft:
        If *True* and bit_rate > 4800, use the Viterbi decoder.
        If *False* (default), use hard-decision nearest-neighbour decode.
    """

    def __init__(self, bit_rate: int = 14400, *, soft: bool = False) -> None:
        if bit_rate not in SUPPORTED_DATA_RATES:
            raise ValueError(
                f"unsupported data rate {bit_rate}; valid: {SUPPORTED_DATA_RATES}"
            )
        self.bit_rate = bit_rate
        self._soft = soft and (bit_rate != 4800)
        self._bits_per_group: int = _RATE_INFO[bit_rate][0]

        if bit_rate == 4800:
            self._det = _DiffDetector4800()
        elif soft:
            self._viterbi = _ViterbiDecoder(bit_rate)
        # hard-decision path is stateless (each call to decode_hard is independent)

    @property
    def bits_per_group(self) -> int:
        return self._bits_per_group

    def reset(self) -> None:
        """Reset decoder state."""
        if self.bit_rate == 4800:
            self._det.reset()
        elif self._soft:
            self._viterbi.reset()

    def decode_hard(
        self, symbols: list[tuple[float, float]]
    ) -> list[int]:
        """Hard-decision decode a block of symbols.

        Returns a flat list of recovered data bits
        (``bits_per_group`` bits per input symbol).
        This is a stateless block operation — each call starts from
        the zero differential-state.  Use :meth:`decode_symbol` for
        streaming Viterbi decoding.
        """
        return _decode_symbols_hard(symbols, self.bit_rate)

    def decode_symbol(self, i_rx: float, q_rx: float) -> list[int] | None:
        """Streaming Viterbi / differential decode of one sample.

        For 4800 bps: immediately returns ``[Q1, Q2]``.
        For 7200–14400 bps (Viterbi mode): returns decoded bits after
        the traceback delay is filled, otherwise returns *None*.

        .. note::
            ``soft=True`` must be passed to the constructor to enable
            Viterbi mode for trellis rates.  If *soft* is False and
            ``bit_rate > 4800`` this method raises :exc:`RuntimeError`.
        """
        if self.bit_rate == 4800:
            q1, q2 = self._det.detect(i_rx, q_rx)
            return [q1, q2]
        if not self._soft:
            raise RuntimeError(
                "decode_symbol() requires soft=True for trellis rates; "
                "use decode_hard() instead"
            )
        return self._viterbi.decode_symbol(i_rx, q_rx)


def encode_data_bits(
    bits: list[int],
    bit_rate: int = 14400,
) -> list[tuple[int, int]]:
    """Encode a flat bit list to ``(I, Q)`` symbols at *bit_rate*.

    Convenience wrapper around :class:`DataModeEncoder`.  A fresh encoder
    (zero state) is used for each call.

    Parameters
    ----------
    bits:
        List of 0/1 integers.  Length must be a multiple of
        ``bits_per_group(bit_rate)``.
    bit_rate:
        One of 4800, 7200, 9600, 12000, 14400 bps.
    """
    enc = DataModeEncoder(bit_rate)
    return enc.encode(bits)


def decode_data_symbols_hard(
    symbols: list[tuple[float, float]],
    bit_rate: int = 14400,
) -> list[int]:
    """Hard-decision decode ``(I, Q)`` symbols to a flat bit list.

    Convenience wrapper around :class:`DataModeDecoder`.  A fresh decoder
    (zero state) is used for each call.

    Parameters
    ----------
    symbols:
        List of ``(I, Q)`` pairs (floats or ints).
    bit_rate:
        One of 4800, 7200, 9600, 12000, 14400 bps.
    """
    return _decode_symbols_hard(symbols, bit_rate)


def encode_data_bytes(
    data: bytes,
    bit_rate: int = 14400,
) -> list[tuple[int, int]]:
    """Encode a byte string to ``(I, Q)`` symbols.

    Bits are packed MSB-first.  The last symbol group is zero-padded if
    needed.
    """
    return _encode_bytes(data, bit_rate)
