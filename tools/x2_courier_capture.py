#!/usr/bin/env python3
"""Read-only native upstream mapper capture for probe_x2_host_courier.py."""
import atexit
import json
import os
from pathlib import Path
import sys

EMU = Path(__file__).resolve().parents[1].parent / 'courier-emu'
sys.path.insert(0, str(EMU))
mode = sys.argv.pop(1)
if mode == 'cli':
    from courier_emu import cli
    original = cli._worker_command
    def command(*a, **kw):
        args = original(*a, **kw)
        assert args[1:3] == ['-m', 'courier_emu.worker']
        return [args[0], str(Path(__file__).resolve()), 'worker', *args[3:]]
    cli._worker_command = command
    raise SystemExit(cli.main())
if mode != 'worker':
    raise SystemExit('expected cli or worker')
from courier_emu.dsp import NativeC5x
original_step = NativeC5x.step
rows = []
# Courier 403 B08C: raw mapper point, source words and rate/frame state.
addresses = [0x3c8, 0x3ca, 0x3f8, 0x3f9, 0x39f, 0x6f, 0x340,
             *range(0x3a2, 0x3ae), 0x4b6, 0x4de, 0x367]
receiver = os.environ.get('X2_HOST_CAPTURE_RECEIVER') == '1'
if receiver:
    addresses = [0x39f, 0x6f, 0x351, 0x364, 0x343, 0x340, 0x341,
                 *range(0x940, 0x944), 0x31c, 0x37d, 0x36e,
                 *range(0x320, 0x333), *range(0x350, 0x36b)]
def step(self, *a, **kw):
    if not getattr(self, '_x2_host_capture', False):
        self.set_pc_capture(0xe11c if receiver else 0xb08c, addresses)
        self._x2_host_capture = True
    result = original_step(self, *a, **kw)
    rows.extend(r for r in self.pc_captures() if len(rows) < 100000)
    self.clear_pc_captures()
    return result
NativeC5x.step = step
@atexit.register
def save():
    Path(os.environ['X2_HOST_NATIVE_CAPTURE']).write_text(
        json.dumps({'addresses': addresses, 'captures': rows}) + '\n')
from courier_emu.worker import main
raise SystemExit(main())
