"""Logical receive-side event detection for V.32 startup symbol streams.

Detects the following events from an ObservableSymbol stream:

  AA   — carrier state A repeated (≥ 32 symbols)
  CC   — carrier state C repeated (≥ 32 symbols)
  AC   — alternating A/C pattern  (≥ 64 symbols)
  CA   — alternating C/A pattern  (≥ 64 symbols)
  S    — ABAB receiver-conditioning sequence (≥ 256 symbols)
  R1/R2/R3 — V.32 16-bit rate sequences (sync: B0-3=0, B7=1, B11=1, B15=1)
  E    — V.32 16-bit E sequence      (sync: B0-3=1, B7=1, B11=1, B15=1)
  B1   — scrambled-ones run (≥ 24 symbols)

The AA/CC/AC/CA detection tracks transitions symbol-by-symbol; events fire
once when the minimum run length is first reached.  New runs reset the
fired flag so repeated segments (e.g. two AC bursts separated by CA) each
produce their own event.

V.32 R and E sequences use the same GPC/GPA scrambler and 4800-baud
differential encoder as V.32bis; only the 16-bit sync positions differ.
Rate-event payloads use the V32_CAP_* bitmask constants from v32_rate_signal.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .negotiation import detect_s_sequence
from .stream import ObservableSymbol
from .training import STATE_A, STATE_B, STATE_C, STATE_D
from .v32_rate_signal import (
    V32_CAP_4800,
    V32_CAP_9600,
    V32_CAP_CLEARDOWN,
    V32_CAP_TRELLIS,
    decode_v32_e_sequence,
    decode_v32_rate_sequence_symbols,
    decode_v32_rate_signal,
    is_v32_e_sequence_bits,
    is_v32_rate_signal_bits,
)


@dataclass(frozen=True)
class V32DetectedEvent:
    """A protocol event detected from a V.32 startup symbol stream.

    Fields
    ------
    name:
        "AA", "CC", "AC", "CA", "S", "R1"/"R2"/"R3", "E", or "B1".
    rate_mask:
        For R events: bitmask of V32_CAP_* flags decoded from the R sequence.
        For E events: V32_CAP_TRELLIS set if trellis coding was selected.
        None for all other events.
    selected_rate:
        For E and B1 events: the negotiated data rate (4800 or 9600).
        None otherwise.
    repetitions:
        Symbol count at the time the event fired, for pattern events.
    """

    name: str
    rate_mask: int | None = None
    selected_rate: int | None = None
    repetitions: int | None = None


class V32LogicalReceiver:
    """Consume a V.32 startup symbol stream and emit logical protocol events."""

    _MIN_AA = 32
    _MIN_CC = 32
    _MIN_AC = 64
    _MIN_CA = 64
    _MIN_B1 = 24
    _S_LENGTH = 256

    def __init__(self) -> None:
        # AA / CC / AC / CA pattern tracking
        self._prev_ac: str | None = None      # last A-or-C symbol seen
        self._run: str | None = None          # "AA", "CC", "AC", "CA"
        self._run_len: int = 0
        self._run_emitted: bool = False
        self._alt_expect: str | None = None   # expected next symbol in AC/CA

        # S (ABAB) detection — only A and B symbols contribute
        self._s_window: deque[str] = deque(maxlen=self._S_LENGTH)
        self._in_s: bool = False

        # Q-symbol run for V.32 R / E decoding
        self._q_run: deque[str] = deque()
        self._rate_seqs: list[list[int]] = []
        self._seen_e: bool = False
        self._rate_count: int = 0

        # B1 run
        self._b1_len: int = 0
        self._b1_emitted: bool = False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _reset_q_run(self) -> None:
        self._q_run.clear()
        self._rate_seqs.clear()
        self._seen_e = False

    def _end_ac_pattern(self) -> None:
        self._run = None
        self._run_len = 0
        self._run_emitted = False
        self._alt_expect = None

    def _rate_event_name(self, source_name: str) -> str:
        if source_name in {"R1", "R2", "R3"}:
            return source_name
        self._rate_count += 1
        return {1: "R1", 2: "R2", 3: "R3"}.get(self._rate_count, f"R{self._rate_count}")

    # ------------------------------------------------------------------
    # Per-symbol ingestion
    # ------------------------------------------------------------------

    def ingest(self, observable: ObservableSymbol) -> list[V32DetectedEvent]:  # noqa: C901
        symbol = observable.symbol
        events: list[V32DetectedEvent] = []

        # ── A / B / C / D ──────────────────────────────────────────────
        if symbol in (STATE_A, STATE_B, STATE_C, STATE_D):
            self._reset_q_run()
            self._b1_len = 0
            self._b1_emitted = False

            # S detection: only A and B go into the ABAB window
            if symbol in (STATE_A, STATE_B):
                self._s_window.append(symbol)
                if len(self._s_window) == self._S_LENGTH:
                    if detect_s_sequence(list(self._s_window)):
                        if not self._in_s:
                            self._in_s = True
                            events.append(V32DetectedEvent(name="S"))
                    else:
                        self._in_s = False
            else:
                # C or D breaks the ABAB window
                self._s_window.clear()
                self._in_s = False

            # AA / CC / AC / CA pattern detection (A and C only)
            if symbol in (STATE_A, STATE_C):
                events.extend(self._advance_ac_pattern(symbol))
            else:
                # B or D ends any ongoing AA/CC/AC/CA run
                self._end_ac_pattern()

            return events

        # ── Q symbols (rate / E sequences) ─────────────────────────────
        if symbol.startswith("Q"):
            self._end_ac_pattern()
            self._prev_ac = None
            self._s_window.clear()
            self._in_s = False
            self._b1_len = 0
            self._b1_emitted = False
            events.extend(
                self._process_q(symbol, observable.tx_calling_party, observable.source_name)
            )
            return events

        # ── B1 (scrambled ones) ────────────────────────────────────────
        if symbol == "B1":
            self._end_ac_pattern()
            self._reset_q_run()
            self._prev_ac = None
            self._s_window.clear()
            self._in_s = False
            self._b1_len += 1
            if self._b1_len >= self._MIN_B1 and not self._b1_emitted:
                self._b1_emitted = True
                events.append(
                    V32DetectedEvent(
                        name="B1",
                        selected_rate=observable.selected_rate,
                        repetitions=self._b1_len,
                    )
                )
            return events

        # ── Unknown symbol ─────────────────────────────────────────────
        self._end_ac_pattern()
        self._reset_q_run()
        self._prev_ac = None
        self._s_window.clear()
        self._in_s = False
        self._b1_len = 0
        self._b1_emitted = False
        return events

    def _advance_ac_pattern(self, symbol: str) -> list[V32DetectedEvent]:
        """Update the AA/CC/AC/CA run state for one A-or-C symbol."""
        events: list[V32DetectedEvent] = []
        prev = self._prev_ac

        # Determine whether the symbol continues the current run
        continues = False
        if self._run == "AA" and symbol == STATE_A:
            continues = True
        elif self._run == "CC" and symbol == STATE_C:
            continues = True
        elif self._run in ("AC", "CA") and symbol == self._alt_expect:
            continues = True
            self._alt_expect = STATE_A if symbol == STATE_C else STATE_C

        if continues:
            self._run_len += 1
        else:
            # Start a new run based on the current symbol and its predecessor
            self._run_emitted = False
            if symbol == STATE_A:
                if prev == STATE_C:
                    # C→A transition: start CA (both symbols already seen → len=2)
                    self._run = "CA"
                    self._run_len = 2
                    self._alt_expect = STATE_C
                else:
                    self._run = "AA"
                    self._run_len = 1
                    self._alt_expect = None
            else:  # STATE_C
                if prev == STATE_A:
                    # A→C transition: start AC (both symbols already seen → len=2)
                    self._run = "AC"
                    self._run_len = 2
                    self._alt_expect = STATE_A
                else:
                    self._run = "CC"
                    self._run_len = 1
                    self._alt_expect = None

        # Fire event once when minimum run length is first reached
        if self._run and not self._run_emitted:
            min_len = {
                "AA": self._MIN_AA,
                "CC": self._MIN_CC,
                "AC": self._MIN_AC,
                "CA": self._MIN_CA,
            }[self._run]
            if self._run_len >= min_len:
                self._run_emitted = True
                events.append(
                    V32DetectedEvent(name=self._run, repetitions=self._run_len)
                )

        self._prev_ac = symbol
        return events

    def _process_q(
        self, symbol: str, calling_party: bool, source_name: str
    ) -> list[V32DetectedEvent]:
        """Decode Q-symbol run; detect V.32 R and E sequences."""
        events: list[V32DetectedEvent] = []
        self._q_run.append(symbol)

        while len(self._q_run) >= 8:
            candidate = list(self._q_run)[:8]
            try:
                decoded = decode_v32_rate_sequence_symbols(
                    candidate,
                    calling_party=calling_party,
                    initial_diff_state=1,
                )
            except ValueError:
                self._q_run.popleft()
                continue

            if is_v32_e_sequence_bits(decoded) and not self._seen_e:
                for _ in range(8):
                    self._q_run.popleft()
                self._seen_e = True
                try:
                    rate, trellis = decode_v32_e_sequence(decoded)
                    mask = V32_CAP_TRELLIS if trellis else 0
                    events.append(
                        V32DetectedEvent(name="E", rate_mask=mask, selected_rate=rate)
                    )
                except ValueError:
                    pass
                break

            if is_v32_rate_signal_bits(decoded):
                for _ in range(8):
                    self._q_run.popleft()
                self._rate_seqs.append(decoded)
                if len(self._rate_seqs) >= 2 and self._rate_seqs[-2] == self._rate_seqs[-1]:
                    try:
                        s4800, s9600, tr, cd = decode_v32_rate_signal(decoded)
                        mask = 0
                        if s4800:
                            mask |= V32_CAP_4800
                        if s9600:
                            mask |= V32_CAP_9600
                        if tr:
                            mask |= V32_CAP_TRELLIS
                        if cd:
                            mask |= V32_CAP_CLEARDOWN
                        name = self._rate_event_name(source_name)
                        self._rate_seqs.clear()
                        events.append(V32DetectedEvent(name=name, rate_mask=mask))
                    except ValueError:
                        pass
                continue

            # Neither R nor E: resync by dropping one symbol
            self._q_run.popleft()

        return events

    def ingest_all(self, stream: list[ObservableSymbol]) -> list[V32DetectedEvent]:
        events: list[V32DetectedEvent] = []
        for obs in stream:
            events.extend(self.ingest(obs))
        return events
