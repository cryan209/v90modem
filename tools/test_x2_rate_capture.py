#!/usr/bin/env python3
"""Read-only rate capture regression; no emulator firmware is required."""
import atexit
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class RateCaptureTest(unittest.TestCase):
    def test_native_ring_is_drained_at_close_without_wrapping_execution(self):
        instances, callbacks = [], []

        class Core:
            def __init__(self):
                self._handle = 1
                self.close_count = 0
                instances.append(self)

            def set_pc_capture(self, pc, addresses):
                self.pc, self.addresses = pc, addresses

            def pc_captures(self):
                return [[100, 200, 18048, 6, 1]]

            def step(self, count):
                return count

            step_cycles = service_step = step

            def close(self):
                self.close_count += bool(self._handle)
                self._handle = None

        original_step = Core.step
        package = types.ModuleType('courier_emu')
        package.__path__ = []
        dsp = types.ModuleType('courier_emu.dsp')
        dsp.NativeC5x = Core
        worker = types.ModuleType('courier_emu.worker')

        def main():
            core = Core()
            self.assertIs(Core.step, original_step)
            self.assertIs(Core.step_cycles, original_step)
            self.assertIs(Core.service_step, original_step)
            self.assertEqual(core.step(17), 17)
            core.close()
            core.close()
            return 0

        worker.main = main
        modules = {'courier_emu': package, 'courier_emu.dsp': dsp,
                   'courier_emu.worker': worker}
        with tempfile.TemporaryDirectory(prefix='x2-rate-capture-') as tmp:
            output = Path(tmp) / 'capture.json'
            env = {'X2_HOST_CAPTURE_RATE': '1', 'X2_HOST_NATIVE_CAPTURE': str(output)}
            with patch.dict(os.environ, env, clear=True), patch.dict(sys.modules, modules), \
                 patch.object(sys, 'argv', ['x2_courier_capture.py', 'worker']), \
                 patch.object(sys, 'path', sys.path.copy()), \
                 patch.object(atexit, 'register', side_effect=lambda f: callbacks.append(f) or f):
                with self.assertRaises(SystemExit) as exit_status:
                    runpy.run_path(str(ROOT / 'tools/x2_courier_capture.py'))
                self.assertEqual(exit_status.exception.code, 0)
                for callback in callbacks:
                    callback()
            result = json.loads(output.read_text())
        self.assertEqual(result['pc'], 0xe2ec)
        self.assertIn(0xfff3, result['addresses'])
        self.assertIn(0xfffb, result['addresses'])
        self.assertIn(0xcf3a, result['addresses'])
        self.assertEqual(result['captures'], [[100, 200, 18048, 6, 1]])
        self.assertEqual(instances[0].close_count, 1)

    def test_conflicting_capture_modes_fail_before_creating_output(self):
        with tempfile.TemporaryDirectory(prefix='x2-rate-args-') as tmp:
            output = Path(tmp) / 'not-created'
            for other in ('--capture-native', '--capture-receiver'):
                result = subprocess.run(
                    [sys.executable, str(ROOT / 'tools/probe_x2_host_courier.py'),
                     '--output', str(output), '--capture-rate', other],
                    capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertIn('cannot be combined', result.stderr)
                self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
