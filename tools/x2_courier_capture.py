#!/usr/bin/env python3
"""Read-only native upstream mapper capture for probe_x2_host_courier.py."""
import atexit
import json
import os
import struct
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
codec_capture = os.environ.get('X2_HOST_CAPTURE_CODEC') == '1'
codec_samples = bytearray()
codec_generations = []
codec_cursor = 0
codec_events = []
# Courier 403 B08C: raw mapper point, source words and rate/frame state.
addresses = [0x3c8, 0x3ca, 0x3f8, 0x3f9, 0x39f, 0x6f, 0x340,
             *range(0x3a2, 0x3ae), 0x4b6, 0x4de, 0x367]
receiver = os.environ.get('X2_HOST_CAPTURE_RECEIVER') == '1'
if receiver:
    addresses = [0x39f, 0x6f, 0x351, 0x364, 0x343, 0x340, 0x341,
                 *range(0x940, 0x944), 0x31c, 0x37d, 0x36e,
                 *range(0x320, 0x333), *range(0x350, 0x36b)]
def step(self, *a, **kw):
    global codec_cursor, codec_events
    if not getattr(self, '_x2_host_capture', False):
        self.set_pc_capture(0xe11c if receiver else 0xb08c, addresses)
        self._x2_host_capture = True
    result = original_step(self, *a, **kw)
    if codec_capture:
        count = self.serial_state()['line_tx_writes']
        if count < codec_cursor or not codec_generations:
            codec_generations.append({'offset_samples': len(codec_samples)//2})
            codec_cursor = 0
        fresh = self.line_tx_samples(codec_cursor)
        codec_samples.extend(struct.pack('<' + 'h'*len(fresh), *fresh))
        codec_cursor = count
        codec_events = self.line_tx_clock_events()
        codec_generations[-1]['clock_events'] = codec_events
        codec_generations[-1]['samples'] = count
    rows.extend(r for r in self.pc_captures() if len(rows) < 100000)
    self.clear_pc_captures()
    return result
NativeC5x.step = step
@atexit.register
def save():
    Path(os.environ['X2_HOST_NATIVE_CAPTURE']).write_text(
        json.dumps({'addresses': addresses, 'captures': rows}) + '\n')
    if codec_capture:
        target = Path(os.environ['X2_HOST_NATIVE_CAPTURE']).parent
        (target / 'native-codec.s16').write_bytes(codec_samples)
        (target / 'native-codec.json').write_text(json.dumps(codec_generations) + '\n')
from courier_emu.worker import main
raise SystemExit(main())
