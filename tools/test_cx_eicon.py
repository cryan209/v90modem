#!/usr/bin/env python3
"""Hardware-free serial call lifecycle checks, including failed escape cleanup."""
import contextlib
import io
import itertools
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cx_eicon_test as target


class Port:
    def __init__(self, menu=False, fail_escape=False, fail_recovery=False):
        self.menu = menu
        self.fail_escape = fail_escape
        self.fail_recovery = fail_recovery
        self.data = bytearray()
        self.decoder = target.Decoder()
        self.closed = False
        self.dropped = False
        self._dtr = True
        self.commands = []

    @property
    def dtr(self):
        return self._dtr

    @dtr.setter
    def dtr(self, value):
        self._dtr = value
        if not value:
            self.dropped = True

    def write(self, data):
        if data.startswith(b'AT'):
            self.commands.append(data)
            if data.startswith(b'ATDT'):
                self.data.extend(b'\r\nCONNECT 115200/V34/LAPM/V42B/21600\r\n')
                self.data.extend(b'Select: ' if self.menu else
                                 target.frame(target.lcp(1, 9, b'\x05\x06PEER')))
            elif data == b'ATH0\r' and (self.fail_recovery or (self.fail_escape and not self.dropped)):
                self.data.extend(b'\r\nERROR\r\n')
            elif data == b'AT&V1\r':
                assert not self.fail_escape or self.dropped
                self.data.extend(b'\r\nLAST TX rate................ 21600 BPS\r\nLAST RX rate................ 21600 BPS\r\nOK\r\n')
            else:
                self.data.extend(b'\r\nOK\r\n')
        elif data == b'1\r':
            self.data.extend(b'/menu exits.\r\n')
        elif data.startswith(b'CX-'):
            self.data.extend(b'ECHO: '+data[:-1]+b'\r\n')
        elif data.startswith(b'\x7e'):
            for body in self.decoder.feed(data):
                packet = body[4:]
                code, ident, length = struct.unpack('!BBH', packet[:4])
                if code == 1:
                    self.data.extend(target.frame(target.lcp(2, ident, packet[4:length])))
                elif code == 9:
                    self.data.extend(target.frame(target.lcp(10, ident, b'PEER'+packet[8:length])))
        return len(data)

    def read(self, count):
        data = bytes(self.data[:count])
        del self.data[:count]
        return data

    def flush(self):
        pass

    def close(self):
        self.closed = True


class SerialCallTests(unittest.TestCase):
    def exercise(self, port, cleanup_ok=True):
        clock = itertools.count(0, .1)
        with tempfile.TemporaryDirectory(prefix='cx-eicon-test-') as directory:
            args = SimpleNamespace(device='fake', output=Path(directory)/'call', number='7910',
                                   mode='V34', max_rate=21600, startup=75, data_seconds=35)
            fake_serial = SimpleNamespace(Serial=lambda *a, **kw: port)
            with patch.dict(sys.modules, serial=fake_serial), \
                 patch.object(target.time, 'monotonic', side_effect=lambda: next(clock)), \
                 patch.object(target.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
                result = target.call(args)
            self.assertTrue(port.closed)
            self.assertEqual(result['hangup_confirmed'], cleanup_ok)
            self.assertEqual(result['profile_restored'], cleanup_ok)
            if cleanup_ok:
                self.assertEqual(result['last_tx_bps'], 21600)
            self.assertTrue((args.output/'summary.json').exists())
            self.assertIn(b'AT+MS=V34,0,300,21600,300,21600\r', port.commands)
            return result

    def test_ppp_data_and_dtr_fallback(self):
        port = Port(fail_escape=True)
        result = self.exercise(port)
        self.assertTrue(port.dropped)
        self.assertEqual(result['exact_lcp_echoes'], 3)
        self.assertEqual(result['bad_ppp_fcs'], 0)

    def test_text_echo_service(self):
        result = self.exercise(Port(menu=True))
        self.assertEqual(result['exact_text_echoes'], 3)
        self.assertEqual(result['exact_lcp_echoes'], 0)

    def test_unconfirmed_cleanup_is_not_success(self):
        port = Port(fail_recovery=True)
        result = self.exercise(port, cleanup_ok=False)
        self.assertTrue(result['connected'])
        self.assertEqual(result['exact_lcp_echoes'], 3)
        self.assertTrue(port.dropped)
        self.assertNotIn(b'AT&V1\r', port.commands)
        self.assertEqual(port.commands.count(b'ATZ\r'), 1)

    def test_protocol_selftest(self):
        with contextlib.redirect_stdout(io.StringIO()):
            target.selftest()


if __name__ == '__main__':
    unittest.main()
