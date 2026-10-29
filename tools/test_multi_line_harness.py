#!/usr/bin/env python3
"""Deterministic checks for the concurrent local SIP test harness."""
import errno
import os
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import multi_line_test as harness


class HarnessTest(unittest.TestCase):
    def test_short_writes_and_would_block(self):
        sent = bytearray()
        calls = 0

        def write(fd, data):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise BlockingIOError()
            count = min(2, len(data))
            sent.extend(data[:count])
            return count

        with patch.object(harness.select, 'select', return_value=([], [9], [])), \
             patch.object(harness.os, 'write', side_effect=write):
            harness.write_all(9, b'abcde')
        self.assertEqual(sent, b'abcde')

    def test_write_deadline(self):
        with self.assertRaises(TimeoutError):
            harness.write_all(9, b'x', timeout=0)

    def test_zero_write_is_failure(self):
        with patch.object(harness.select, 'select', return_value=([], [9], [])), \
             patch.object(harness.os, 'write', return_value=0), \
             self.assertRaises(OSError):
            harness.write_all(9, b'x')

    def test_reopen_dead_pty_even_when_symlink_exists(self):
        stop = threading.Event()
        reads = {10: 0, 11: 0}
        output = []

        def read(fd, count):
            reads[fd] += 1
            if reads[fd] == 1:
                return b'CONNECT 56000\r\n'
            if fd == 10:
                if reads[fd] > 8:
                    stop.set()  # Bound the negative arm's old retry loop.
                raise OSError(errno.EIO, 'old PTY master closed')
            return b'own-line-payload'

        def write(fd, data):
            output.append((fd, data))
            stop.set()

        with patch.object(harness, 'open_pty', side_effect=[10, 11]) as opened, \
             patch.object(harness.os, 'read', side_effect=read), \
             patch.object(harness.os, 'close') as closed, \
             patch.object(harness.os.path, 'exists', return_value=True), \
             patch.object(harness.time, 'sleep'), \
             patch.object(harness, 'write_all', side_effect=write):
            harness.line_echo('/unused/line.0', stop, {}, 0)
        self.assertEqual(opened.call_count, 2)
        self.assertEqual(output, [(11, b'own-line-payload')])
        self.assertEqual([c.args[0] for c in closed.call_args_list], [10, 11])

    def test_supervisor_help_does_not_spawn_workers(self):
        result = subprocess.run([harness.BINARY, '--lines', '64', '--help'],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0)
        self.assertIn('--lines', result.stdout)
        self.assertNotIn('worker pid', result.stdout)

    def test_bad_pool_configuration_fails_before_fork(self):
        for extra in [['--local-port', '4000', '--rtp-port', '4000'],
                      ['--rtp-port', '65500'], ['--local-port', 'garbage'],
                      ['--control-link', '/unused/ctl'], ['--data-link', '/unused/data']]:
            with self.subTest(extra=extra):
                result = subprocess.run([harness.BINARY, '--lines', '2', *extra],
                                        capture_output=True, text=True, timeout=5)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn('worker pid', result.stdout)

    def test_one_worker_crash_does_not_restart_other_line(self):
        binary = str(harness.ROOT / 'line_supervisor_test')
        proc = subprocess.Popen([binary, '--fixture', '--lines', '2'],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        pump = harness.Pump(proc)
        pump.start()

        def wait_for(predicate):
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                if predicate():
                    return
                if proc.poll() is not None:
                    break
                time.sleep(.02)
            self.fail('fixture deadline: ' + pump.text())

        try:
            wait_for(lambda: pump.text().count('fixture ready line=') == 2)
            before = dict(harness.worker_pids(pump.text()))
            os.kill(int(before['0']), signal.SIGKILL)
            wait_for(lambda: len(harness.worker_pids(pump.text())) >= 3
                     and pump.text().count('fixture ready line=') >= 3)
            after = dict(harness.worker_pids(pump.text()))
            self.assertNotEqual(before['0'], after['0'])
            self.assertEqual(before['1'], after['1'])
        finally:
            proc.terminate()
            try:
                proc.wait(10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(5)
            pump.join(timeout=2)
            survivors = []
            for _, pid in harness.worker_pids(pump.text()):
                try:
                    os.kill(int(pid), 0)
                except ProcessLookupError:
                    continue
                survivors.append(int(pid))
            for pid in survivors:  # Only this fixture's recorded children.
                os.kill(pid, signal.SIGKILL)
            self.assertEqual(survivors, [], 'fixture workers survived shutdown')
            proc.stdout.close()

    def test_spawn_failure_aborts_other_callers_barrier(self):
        args = SimpleNamespace(mode='v90', io_schedule=False)
        barrier = threading.Barrier(2)
        results = {}
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(harness, 'free_port_block', return_value=12345), \
             patch.object(harness.subprocess, 'Popen', side_effect=OSError('no descriptors')):
            harness.one_call(0, args, Path(tmp), 20000, 30000, results, barrier)
        self.assertTrue(barrier.broken)
        self.assertFalse(results[0]['ok'])
        self.assertIn('no descriptors', results[0]['why'])


if __name__ == '__main__':
    unittest.main()
