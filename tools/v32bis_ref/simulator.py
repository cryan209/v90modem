"""Offline logical startup simulator for the V.32bis reference model."""

from __future__ import annotations

from dataclasses import dataclass

from .negotiation import negotiate_startup_rate
from .receiver import V32bisLogicalReceiver
from .startup import generate_answer_startup_trace, generate_call_startup_trace


@dataclass(frozen=True)
class StartupSimulationResult:
    answer_r1_mask: int
    call_r2_mask: int
    answer_r3_mask: int
    selected_rate: int | None
    caller_e_rate: int | None
    answerer_e_rate: int | None
    cleardown: bool


def simulate_startup(
    *,
    answer_supported_mask: int,
    call_supported_mask: int,
    answer_selected_mask: int | None = None,
) -> StartupSimulationResult:
    """Run a logical R1 -> R2 -> R3 -> E startup exchange.

    This follows the specification's negotiation structure, but it is purely a
    bit-level/event-level simulator. It assumes ideal detection of R and E
    sequences and does not model round-trip timing, carrier segments, or DSP
    convergence.
    """
    answer_receiver = V32bisLogicalReceiver()
    call_receiver = V32bisLogicalReceiver()

    provisional_r3_mask = answer_supported_mask & call_supported_mask
    if answer_selected_mask is None:
        answer_selected_mask = provisional_r3_mask
    else:
        answer_selected_mask &= provisional_r3_mask

    answer_trace = generate_answer_startup_trace(
        r1_mask=answer_supported_mask,
        r2_mask=answer_selected_mask,
        r3_selected_rate=4800 if answer_selected_mask == 0 else max(
            rate for rate, mask in (
                (14400, 0x1000),
                (12000, 0x0400),
                (9600, 0x0200),
                (7200, 0x0040),
                (4800, 0x0020),
            ) if answer_selected_mask & mask
        ),
    )

    answer_events = []
    for segment in answer_trace:
        answer_events.extend(answer_receiver.detect(segment))

    r1_event = next(event for event in answer_events if event.name == "R1")
    r1_mask = r1_event.rate_mask or 0
    r2_mask = r1_mask & call_supported_mask

    negotiation_preview = negotiate_startup_rate(r2_mask, answer_selected_mask)
    selected_rate = negotiation_preview.agreed_rate if not negotiation_preview.cleardown else 4800

    call_trace = generate_call_startup_trace(
        r1_mask=r1_mask,
        r2_mask=r2_mask,
        r3_selected_rate=selected_rate,
    )
    call_events = []
    for segment in call_trace:
        call_events.extend(call_receiver.detect(segment))

    r2_event = next(event for event in call_events if event.name == "R2")
    decoded_r2_mask = r2_event.rate_mask or 0

    # Rebuild the answer trace with the final selected rate if one exists.
    if negotiation_preview.cleardown or negotiation_preview.agreed_rate is None:
        decoded_r3_mask = answer_selected_mask
        return StartupSimulationResult(
            answer_r1_mask=r1_mask,
            call_r2_mask=decoded_r2_mask,
            answer_r3_mask=decoded_r3_mask,
            selected_rate=None,
            caller_e_rate=None,
            answerer_e_rate=None,
            cleardown=True,
        )

    selected_rate = negotiation_preview.agreed_rate
    answer_trace = generate_answer_startup_trace(
        r1_mask=answer_supported_mask,
        r2_mask=answer_selected_mask,
        r3_selected_rate=selected_rate,
    )
    answer_events = []
    for segment in answer_trace:
        answer_events.extend(answer_receiver.detect(segment))

    r3_event = next(event for event in answer_events if event.name == "R3")
    answer_e_event = next(event for event in answer_events if event.name == "E")
    caller_e_event = next(event for event in call_events if event.name == "E")
    decoded_r3_mask = r3_event.rate_mask or 0

    return StartupSimulationResult(
        answer_r1_mask=r1_mask,
        call_r2_mask=decoded_r2_mask,
        answer_r3_mask=decoded_r3_mask,
        selected_rate=selected_rate,
        caller_e_rate=caller_e_event.selected_rate,
        answerer_e_rate=answer_e_event.selected_rate,
        cleardown=False,
    )
