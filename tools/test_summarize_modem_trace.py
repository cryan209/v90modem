from __future__ import annotations

import unittest

from tools.summarize_modem_trace import format_text_report, parse_log_lines


SAMPLE_LOG = [
    "Random startup line",
    "[TRACE +0ms] SIP media connected: role=caller",
    "[TRACE +1ms] enter V8: advertised mods=V90|V34|V22",
    "00:00:01 sip_modem ME trace (startup): state=IDLE mod=NONE law=ULAW role=answerer media=down phase_ms=0 v34_rx=0 v34_tx=0 v90_rx=-1 v90_tx=-1 v90_event=-1 phase3=0 j=0 dil=0",
    "00:00:02 sip_modem ME trace (media-connected): state=V8 mod=NONE law=ULAW role=caller media=up phase_ms=12 v34_rx=0 v34_tx=0 v90_rx=-1 v90_tx=-1 v90_event=-1 phase3=0 j=0 dil=0",
    "[TRACE +250ms] V8 selected V90",
    "00:00:03 sip_modem ME trace (state-change): state=TRAINING mod=V90 law=ULAW role=caller media=up phase_ms=250 v34_rx=10 v34_tx=34 v90_rx=10 v90_tx=34 v90_event=0 phase3=1 j=0 dil=0",
    "[TRACE +1750ms] V34 stage: rx=PHASE4_TRN(15) tx=PHASE4_TRN(44)",
    "00:00:04 sip_modem ME trace (state-change): state=DATA mod=V90 law=ULAW role=caller media=up phase_ms=1750 v34_rx=16 v34_tx=44 v90_rx=16 v90_tx=44 v90_event=15 phase3=1 j=1 dil=1",
]

FAILED_TRAINING_LOG = [
    "[TRACE +0ms] SIP media connected: role=caller",
    "00:00:02 sip_modem ME trace (media-connected): state=V8 mod=NONE law=ULAW role=caller media=up phase_ms=12 v34_rx=0 v34_tx=0 v90_rx=-1 v90_tx=-1 v90_event=-1 phase3=0 j=0 dil=0",
    "[TRACE +250ms] V8 selected V90",
    "00:00:03 sip_modem ME trace (state-change): state=TRAINING mod=V90 law=ULAW role=caller media=up phase_ms=250 v34_rx=10 v34_tx=34 v90_rx=10 v90_tx=34 v90_event=0 phase3=1 j=0 dil=0",
]


class SummarizeModemTraceTests(unittest.TestCase):
    def test_parse_log_lines_extracts_me_and_engine_events(self) -> None:
        report = parse_log_lines(SAMPLE_LOG)

        self.assertEqual(report["summary"]["me_event_count"], 4)
        self.assertEqual(report["summary"]["engine_event_count"], 4)
        self.assertEqual(report["summary"]["state_sequence"], ["IDLE", "V8", "TRAINING", "DATA"])
        self.assertTrue(report["summary"]["saw_v8"])
        self.assertTrue(report["summary"]["saw_training"])
        self.assertTrue(report["summary"]["saw_data"])
        self.assertTrue(report["summary"]["saw_v90_phase3"])
        self.assertTrue(report["summary"]["saw_v90_dil"])
        self.assertIsNone(report["summary"]["first_failure_point"])

    def test_text_report_contains_timeline_sections(self) -> None:
        report = parse_log_lines(SAMPLE_LOG)
        text = format_text_report(report)

        self.assertIn("# Modem Trace Summary", text)
        self.assertIn("state_sequence: IDLE -> V8 -> TRAINING -> DATA", text)
        self.assertIn("first_failure_point: none", text)
        self.assertIn("## ME Timeline", text)
        self.assertIn("## Engine Timeline", text)
        self.assertIn("reason=state-change state=DATA mod=V90", text)
        self.assertIn("+1750ms V34 stage: rx=PHASE4_TRN(15) tx=PHASE4_TRN(44)", text)

    def test_failed_training_reports_first_failure_point(self) -> None:
        report = parse_log_lines(FAILED_TRAINING_LOG)

        self.assertFalse(report["summary"]["saw_data"])
        self.assertEqual(report["summary"]["first_failure_point"]["stage"], "training_no_dil")
        self.assertIn(
            "no DIL descriptor was logged",
            report["summary"]["first_failure_point"]["detail"],
        )


if __name__ == "__main__":
    unittest.main()
