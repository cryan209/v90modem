"""Reference-oracle comparison helpers for the datapump."""

from __future__ import annotations

from dataclasses import dataclass

from tools.v32bis_ref.receiver import DetectedEvent


EXPECTED_STARTUP_EVENTS = ("S", "R1", "S", "R3", "E", "B1")


@dataclass(frozen=True)
class OracleComparison:
    expected: list[str]
    actual: list[str]
    matches: bool


def expected_startup_events() -> list[str]:
    return list(EXPECTED_STARTUP_EVENTS)


def compare_events(events: list[DetectedEvent]) -> OracleComparison:
    actual = [event.name for event in events]
    expected = expected_startup_events()
    return OracleComparison(
        expected=expected,
        actual=actual,
        matches=actual == expected,
    )
