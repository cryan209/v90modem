#!/usr/bin/env python3
"""Hardware-free checks of fragmented pager/result handling and call cleanup."""
import io
import csv
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mt5634mu_diag import Port, analyze_cdr, diagnose


class DiagnosticTest(unittest.TestCase):
    def test_answered_voicemail_is_not_answered_modem(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'Master.csv'
            with path.open('w', newline='') as stream:
                csv.writer(stream).writerows([
                    ['', '6004', '6311', 'default', '', '', '', 'Dial', '',
                     'start', 'answer', 'end', '10', '9', 'ANSWERED', 'DOCUMENTATION', 'old', ''],
                    ['', '6004', '6311', 'default', '', '', '', 'Dial', '',
                     'start', '', 'end', '0', '0', 'BUSY', 'DOCUMENTATION', 'failed', ''],
                    ['', '6004', '6311', 'default', '', '', '', 'VoiceMail', '6311,u',
                     'start', 'answer', 'end', '50', '50', 'ANSWERED', 'DOCUMENTATION', 'failed', ''],
                    ['', '6004', '6312', 'default', '', '', '', 'Dial', '',
                     'start', 'answer', 'end', '36', '35', 'ANSWERED', 'DOCUMENTATION', 'connected', '']])
            failed = analyze_cdr(path, '6311')
            self.assertEqual(failed['route_result'], 'voicemail_after_busy')
            self.assertEqual(len(failed['legs']), 2)
            self.assertEqual(analyze_cdr(path, '6312')['route_result'], 'modem_route_answered')

    def test_fragmented_pager_does_not_eat_next_command(self):
        chunks = [b'ATI11\r\nPress an', b'y key to continue; ESC to quit.',
                  b'\r\nTermination cause 3\r\nPress any key',
                  b' to continue; ESC to quit.', b'\r\nOK\r\n']
        port = Port(42, io.StringIO())
        with patch('mt5634mu_diag.select.select', return_value=([42], [], [])), \
             patch('mt5634mu_diag.os.read', side_effect=chunks), \
             patch.object(port, 'write') as write:
            data, result = port.read(5, pager=True)
        self.assertEqual(result, 'OK')
        self.assertIn(b'Termination cause 3', data)
        self.assertEqual([c.args[0] for c in write.call_args_list], [b' ', b' '])

    def test_profile_text_is_not_a_result(self):
        chunks = [b'No Carrier Disc 2000 msec\r\nOK option enabled\r\n',
                  b'\r\nNO CAR', b'RIER\r\n']
        port = Port(42, io.StringIO())
        with patch('mt5634mu_diag.select.select', return_value=([42], [], [])), \
             patch('mt5634mu_diag.os.read', side_effect=chunks):
            _, result = port.read(5)
        self.assertEqual(result, 'NO CARRIER')

    def run_call(self, replies):
        class Peer:
            def __init__(self):
                self.sent = []

            def command(self, cmd, pager=False):
                self.sent.append((cmd, pager))
                return 'MT5634MU' if cmd == 'ATI3' else 'OK'

            def write(self, data):
                self.sent.append(data)

            def read(self, *args, **kwargs):
                return next(replies)

        peer = Peer()
        with patch('mt5634mu_diag.time.sleep'):
            summary = diagnose(peer, '6311', 80, 5)
        return peer.sent, summary

    def test_failed_call_keeps_diagnostics_without_reset_or_escape(self):
        sent, summary = self.run_call(iter([(b'\r\nNO CARRIER\r\n', 'NO CARRIER')]))
        self.assertEqual(summary['result'], 'NO CARRIER')
        self.assertEqual(sent, [('AT', False), ('ATE1V1Q0', False), ('ATM0', False),
                                ('ATI3', False), ('AT+GMR', False), ('AT&V', True),
                                b'ATDT6311\r', ('ATI11', True)])

    def test_connected_call_hangs_up_before_ati11_and_keeps_inline_banner(self):
        sent, summary = self.run_call(iter([(b'CONNECT 33600\r\nlogin:', 'CONNECT 33600'),
                                           (b' ', None), (b'OK\r\n', 'OK')]))
        self.assertIn('login:', summary['data_preview'])
        self.assertEqual(sent[-3:], [b'+++', ('ATH', False), ('ATI11', True)])

    def test_timeout_aborts_training_before_ath(self):
        sent, summary = self.run_call(iter([(b'', None), (b'NO CARRIER\r\n', 'NO CARRIER')]))
        self.assertEqual(summary['result'], 'TIMEOUT')
        self.assertEqual(sent[-3:], [b'\r', ('ATH', False), ('ATI11', True)])

    def test_unconfirmed_escape_stops_commands(self):
        with self.assertRaisesRegex(RuntimeError, 'escape unconfirmed'):
            self.run_call(iter([(b'CONNECT 33600\r\n', 'CONNECT 33600'),
                                (b'', None), (b'', None)]))


if __name__ == '__main__':
    unittest.main()
