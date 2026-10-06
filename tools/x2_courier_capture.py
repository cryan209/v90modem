#!/usr/bin/env python3
"""Native mapper/DAC capture for probe_x2_host_courier.py.

Capture is read-only by default. X2_HOST_DIAGNOSTIC_TX_GAIN explicitly changes
runtime gain for a causal experiment; its result is not unmodified interop.
"""
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
if os.environ.get('X2_HOST_CAPTURE_LIBRARY'):
    from courier_emu import dsp
    dsp.LIBRARY = Path(os.environ['X2_HOST_CAPTURE_LIBRARY'])
original_step = NativeC5x.step
rows = []
codec_capture = os.environ.get('X2_HOST_CAPTURE_CODEC') == '1'
codec_samples = bytearray()
codec_generations = []
codec_cursor = 0
codec_events = []
tx_writes = os.environ.get('X2_HOST_CAPTURE_TX_WRITES') == '1'
pulse_capture = os.environ.get('X2_HOST_CAPTURE_PULSE') == '1'
gain_capture = os.environ.get('X2_HOST_CAPTURE_GAIN') == '1'
write_events = []
diagnostic_gain = os.environ.get('X2_HOST_DIAGNOSTIC_TX_GAIN')
gain_changes = []
if diagnostic_gain:
    diagnostic_gain = int(diagnostic_gain)
    if not 1 <= diagnostic_gain <= 31999:
        raise SystemExit('diagnostic gain must be between 1 and 31999')
    if not codec_capture:
        raise SystemExit('diagnostic gain requires X2_HOST_CAPTURE_CODEC=1')
# Courier 403 B08C: raw mapper point, source words and rate/frame state.
addresses = [0x3c8, 0x3ca, 0x3f8, 0x3f9, 0x39f, 0x6f, 0x340,
             *range(0x3a2, 0x3ae), 0x4b6, 0x4de, 0x367,
             0x3e7, 0x3f5]
receiver = os.environ.get('X2_HOST_CAPTURE_RECEIVER') == '1'
if receiver:
    addresses = [0x39f, 0x6f, 0x351, 0x364, 0x343, 0x340, 0x341,
                 *range(0x940, 0x944), 0x31c, 0x37d, 0x36e,
                 *range(0x320, 0x333), *range(0x350, 0x36b)]
if tx_writes:
    addresses = [0x17, 0x390, 0x391, 0x340, 0x398, 0x399, 0x366, 0x367]
    if pulse_capture:
        addresses += [0x3c7, 0x392, 0x3fb, 0x3c6, 0x39a, 0x39b]
    if gain_capture:
        addresses += [0x3c7, 0x392, 0x3fb, 0x39f, 0x214]
capture_pc = 0xe11c if receiver else 0xb08c
if tx_writes:
    if not codec_capture:
        raise SystemExit('TX write capture requires X2_HOST_CAPTURE_CODEC=1')
    capture_pc = 0x80e1 if gain_capture else 0xb461 if pulse_capture else 0x818f
def step(self, *a, **kw):
    global codec_cursor, codec_events
    if not getattr(self, '_x2_host_capture', False):
        self.set_pc_capture(capture_pc, addresses)
        if tx_writes:
            self.trace_data_writes()
            if gain_capture:
                self.set_data_trace_filter(0x392)
            elif pulse_capture:
                self.set_data_trace_filter(0x3c7)
            else:
                self.set_data_trace_range(0x0b00, 0x0cff)
        self._x2_host_capture = True
    result = original_step(self, *a, **kw)
    # Explicit causal experiment only: change the native runtime level before
    # its fractional multiply/store, never repair already wrapped line PCM.
    # This is not an unmodified-firmware interop qualification.
    if diagnostic_gain and codec_cursor >= 180000 and not gain_changes:
        value = diagnostic_gain
        old = self.data(0x392)
        self.set_data(0x392, value)
        self.set_data(0xfff0, value)
        gain_changes.append({'codec_sample': codec_cursor, 'old': old, 'new': value})
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
    keep = not tx_writes or 180000 <= codec_cursor <= 200000
    if keep:
        rows.extend(r for r in self.pc_captures() if len(rows) < 100000)
    if tx_writes:
        if keep or gain_capture:
            write_events.extend(self.data_events())
        self.trace_data_writes(clear=True)
    self.clear_pc_captures()
    return result
NativeC5x.step = step
@atexit.register
def save():
    Path(os.environ['X2_HOST_NATIVE_CAPTURE']).write_text(
        json.dumps({'pc': capture_pc, 'addresses': addresses, 'captures': rows,
                    'diagnostic_library': os.environ.get('X2_HOST_CAPTURE_LIBRARY'),
                    'diagnostic_gain_changes': gain_changes}) + '\n')
    if codec_capture:
        target = Path(os.environ['X2_HOST_NATIVE_CAPTURE']).parent
        (target / 'native-codec.s16').write_bytes(codec_samples)
        (target / 'native-codec.json').write_text(json.dumps(codec_generations) + '\n')
        if tx_writes:
            (target / 'native-tx-writes.json').write_text(json.dumps(write_events) + '\n')
from courier_emu.worker import main
raise SystemExit(main())
