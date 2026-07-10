#!/usr/bin/env python3
"""Summarize live sip_v90_modem verbose logs into a compact modem timeline."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


ME_TRACE_RE = re.compile(
    r"ME trace \((?P<reason>[^)]+)\): "
    r"state=(?P<state>\S+) "
    r"mod=(?P<mod>\S+) "
    r"law=(?P<law>\S+) "
    r"role=(?P<role>\S+) "
    r"media=(?P<media>\S+) "
    r"phase_ms=(?P<phase_ms>\d+) "
    r"v34_rx=(?P<v34_rx>-?\d+) "
    r"v34_tx=(?P<v34_tx>-?\d+) "
    r"v90_rx=(?P<v90_rx>-?\d+) "
    r"v90_tx=(?P<v90_tx>-?\d+) "
    r"v90_event=(?P<v90_event>-?\d+) "
    r"phase3=(?P<phase3>\d+) "
    r"s_events=(?P<s_events>\d+) "
    r"dil=(?P<dil>\d+)"
)

ENGINE_TRACE_RE = re.compile(r"\[TRACE \+(?P<elapsed_ms>\d+)ms\] (?P<message>.*)")


def _first_failure_point(
    me_events: list[dict[str, object]],
    summary: dict[str, object],
) -> dict[str, object] | None:
    """Infer the first obvious failure point when a run does not reach DATA."""
    if summary["saw_data"]:
        return None
    if not me_events:
        return {
            "stage": "no_modem_trace",
            "detail": "No 'ME trace' lines were found in the log.",
            "line_number": None,
        }

    last_me = me_events[-1]
    state = str(last_me["state"])
    media_up = any(str(event["media"]) == "up" for event in me_events)

    if not media_up:
        return {
            "stage": "pre_media",
            "detail": "The modem never reached an active media path.",
            "line_number": last_me["line_number"],
        }
    if not summary["saw_v8"]:
        return {
            "stage": "pre_v8",
            "detail": "Media came up but the modem never entered V8.",
            "line_number": last_me["line_number"],
        }
    if state == "V8":
        return {
            "stage": "v8",
            "detail": "The run stopped in V8 before training started.",
            "line_number": last_me["line_number"],
        }
    if state == "TRAINING":
        if not summary["saw_v90_phase3"]:
            return {
                "stage": "training_pre_phase3",
                "detail": "Training started but never reached the logged V.90 Phase 3 path.",
                "line_number": last_me["line_number"],
            }
        if not summary["saw_v90_dil"]:
            return {
                "stage": "training_no_dil",
                "detail": "Training reached the V.90 bridge but no DIL descriptor was logged before the run stopped.",
                "line_number": last_me["line_number"],
            }
        return {
            "stage": "training",
            "detail": "Training progressed but never transitioned into DATA.",
            "line_number": last_me["line_number"],
        }
    if state == "HANGUP":
        return {
            "stage": "hangup",
            "detail": "The modem reached HANGUP before DATA.",
            "line_number": last_me["line_number"],
        }
    if state == "IDLE":
        return {
            "stage": "idle_return",
            "detail": "The modem returned to IDLE before reaching DATA.",
            "line_number": last_me["line_number"],
        }
    return {
        "stage": state.lower(),
        "detail": f"The last modem state before failure was {state}.",
        "line_number": last_me["line_number"],
    }


def parse_log_lines(lines: list[str]) -> dict[str, object]:
    """Parse modem trace lines from a sip_v90_modem verbose log."""
    me_events: list[dict[str, object]] = []
    engine_events: list[dict[str, object]] = []

    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.rstrip("\n")

        me_match = ME_TRACE_RE.search(line)
        if me_match:
            me_events.append(
                {
                    "line_number": line_number,
                    "reason": me_match.group("reason"),
                    "state": me_match.group("state"),
                    "modulation": me_match.group("mod"),
                    "law": me_match.group("law"),
                    "role": me_match.group("role"),
                    "media": me_match.group("media"),
                    "phase_ms": int(me_match.group("phase_ms")),
                    "v34_rx_stage": int(me_match.group("v34_rx")),
                    "v34_tx_stage": int(me_match.group("v34_tx")),
                    "v90_rx_stage": int(me_match.group("v90_rx")),
                    "v90_tx_stage": int(me_match.group("v90_tx")),
                    "v90_event": int(me_match.group("v90_event")),
                    "v90_phase3_started": int(me_match.group("phase3")),
                    "v90_phase3_s_events": int(me_match.group("s_events")),
                    "v90_dil_valid": int(me_match.group("dil")),
                    "raw": line,
                }
            )
            continue

        engine_match = ENGINE_TRACE_RE.search(line)
        if engine_match:
            engine_events.append(
                {
                    "line_number": line_number,
                    "elapsed_ms": int(engine_match.group("elapsed_ms")),
                    "message": engine_match.group("message"),
                    "raw": line,
                }
            )

    state_sequence: list[str] = []
    for event in me_events:
        state = str(event["state"])
        if not state_sequence or state_sequence[-1] != state:
            state_sequence.append(state)

    summary = {
        "me_event_count": len(me_events),
        "engine_event_count": len(engine_events),
        "state_sequence": state_sequence,
        "first_me_event": me_events[0] if me_events else None,
        "last_me_event": me_events[-1] if me_events else None,
        "first_engine_event": engine_events[0] if engine_events else None,
        "last_engine_event": engine_events[-1] if engine_events else None,
        "saw_v8": any(event["state"] == "V8" for event in me_events),
        "saw_training": any(event["state"] == "TRAINING" for event in me_events),
        "saw_data": any(event["state"] == "DATA" for event in me_events),
        "saw_hangup": any(event["state"] == "HANGUP" for event in me_events),
        "saw_v90_phase3": any(event["v90_phase3_started"] for event in me_events),
        "saw_v90_dil": any(event["v90_dil_valid"] for event in me_events),
    }
    summary["first_failure_point"] = _first_failure_point(me_events, summary)

    return {
        "summary": summary,
        "me_events": me_events,
        "engine_events": engine_events,
    }


def format_text_report(report: dict[str, object]) -> str:
    """Render a compact human-readable trace summary."""
    summary = report["summary"]
    me_events = report["me_events"]
    engine_events = report["engine_events"]

    lines = [
        "# Modem Trace Summary",
        f"me_event_count: {summary['me_event_count']}",
        f"engine_event_count: {summary['engine_event_count']}",
        f"state_sequence: {' -> '.join(summary['state_sequence']) if summary['state_sequence'] else 'none'}",
        (
            "milestones: "
            f"V8={summary['saw_v8']} "
            f"TRAINING={summary['saw_training']} "
            f"DATA={summary['saw_data']} "
            f"HANGUP={summary['saw_hangup']} "
            f"V90_PHASE3={summary['saw_v90_phase3']} "
            f"V90_DIL={summary['saw_v90_dil']}"
        ),
    ]
    if summary["first_failure_point"] is None:
        lines.append("first_failure_point: none")
    else:
        failure = summary["first_failure_point"]
        lines.append(
            "first_failure_point: "
            f"stage={failure['stage']} "
            f"line={failure['line_number']} "
            f"detail={failure['detail']}"
        )
    lines.extend(["", "## ME Timeline"])

    for event in me_events[:20]:
        lines.append(
            f"L{event['line_number']}: "
            f"reason={event['reason']} state={event['state']} mod={event['modulation']} "
            f"law={event['law']} role={event['role']} media={event['media']} "
            f"phase_ms={event['phase_ms']} v34=({event['v34_rx_stage']},{event['v34_tx_stage']}) "
            f"v90=({event['v90_rx_stage']},{event['v90_tx_stage']},event={event['v90_event']}) "
            f"phase3={event['v90_phase3_started']} s_events={event['v90_phase3_s_events']} "
            f"dil={event['v90_dil_valid']}"
        )
    if len(me_events) > 20:
        lines.append(f"... {len(me_events) - 20} more ME events")

    lines.append("")
    lines.append("## Engine Timeline")
    for event in engine_events[:20]:
        lines.append(f"L{event['line_number']}: +{event['elapsed_ms']}ms {event['message']}")
    if len(engine_events) > 20:
        lines.append(f"... {len(engine_events) - 20} more engine trace events")

    return "\n".join(lines)


def _read_input(path: Path | None) -> list[str]:
    if path is None:
        return sys.stdin.read().splitlines()
    return path.read_text().splitlines()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Summarize verbose sip_v90_modem logs")
    parser.add_argument("logfile", nargs="?", type=Path, help="path to a captured verbose log")
    parser.add_argument("--json", action="store_true", help="emit JSON instead of text")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    report = parse_log_lines(_read_input(args.logfile))
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(format_text_report(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
