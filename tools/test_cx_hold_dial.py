#!/usr/bin/env python3
"""Exercise failed-dial cleanup without opening hardware or waiting 90 s."""
import unittest
from unittest.mock import Mock, patch

import cx_hold_dial


class DialCleanupTest(unittest.TestCase):
    def run_call(self, response, interrupt=False):
        elapsed = 0.0
        port = Mock()
        events = []

        def read(duration):
            nonlocal elapsed
            elapsed += duration
            if interrupt and duration == 0.5:
                raise KeyboardInterrupt
            return response if duration == 0.5 else b''

        def command(text, duration=2):
            events.append(text)
            return b'OK\r\n'

        with patch.object(cx_hold_dial.time, 'monotonic', side_effect=lambda: elapsed), \
                patch.object(cx_hold_dial.time, 'sleep') as sleep:
            if interrupt:
                with self.assertRaises(KeyboardInterrupt):
                    cx_hold_dial.dial_and_hold(port, '2904', 0, [], read, command)
                result = None
            else:
                result = cx_hold_dial.dial_and_hold(port, '2904', 0, [], read, command)
            sleep.assert_called_once_with(1)
        self.assertEqual(events[-2:], ['ATH', 'AT&V1'])
        self.assertEqual([call.args[0] for call in port.write.call_args_list],
                         [b'ATDT2904\r', b'+++', b'\r'])
        return result

    def test_failed_results(self):
        for result in (b'NO CARRIER', b'BUSY', b'NO DIALTONE', b''):
            with self.subTest(result=result):
                self.assertEqual(self.run_call(result), 1)

    def test_connected_cleanup(self):
        self.assertEqual(self.run_call(b'CONNECT 50666'), 0)

    def test_interrupted_dial_cleanup(self):
        self.run_call(b'', interrupt=True)


if __name__ == '__main__':
    unittest.main()
