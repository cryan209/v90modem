#!/usr/bin/env python3
"""Compare production resampling with original MICA 51CE/4580/5219.

Supplied lane ring, slip and gain; overflow excluded. No serial dispatcher,
clock acquisition or raw PCM-to-report claim is made.
"""
import argparse
import ctypes
import hashlib
from pathlib import Path
import subprocess
import tempfile


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mica', type=Path, default=Path.home() / 'MicaEmu')
    ap.add_argument('--saturation', action='store_true', help='Exercise signed gain saturation at extreme gains')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    class State(ctypes.Structure):
        _fields_ = [('source_cursor', ctypes.c_uint), ('output_cursor', ctypes.c_uint),
                    ('output_available', ctypes.c_uint), ('slip', ctypes.c_int)]
    with tempfile.TemporaryDirectory(prefix='flex-resampler-') as tmp:
        library = Path(tmp) / 'flex.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-I', str(repo), str(repo/'mica_qam.c'), '-o', str(library)], check=True)
        native = ctypes.CDLL(str(library))
        ptr = ctypes.POINTER(ctypes.c_int16)
        native.mica_qam_resample.argtypes = [ctypes.POINTER(State), ptr, ctypes.c_uint,
             ctypes.c_uint, ctypes.c_uint, ctypes.c_int16, ctypes.c_int16, ptr]
        native.mica_qam_resample.restype = ctypes.c_int
        def compare(samples, source, destination, old_count, slip, available, phase, shift, gain, bias, expected):
            state = State(source, destination, old_count, slip)
            output = (ctypes.c_int16*256)()
            assert native.mica_qam_resample(ctypes.byref(state), (ctypes.c_int16*128)(*samples),
                available, phase, shift, gain, bias, output) == 0
            assert [output[(destination+i)&255] & 65535 for i in range(len(expected))] == expected
            assert (state.source_cursor, state.output_cursor, state.output_available, state.slip) == (
                (source+2*available)&127, (destination+len(expected))&255,
                old_count+available+slip, 0)
        fixture = args.mica/'artifacts/k56flex-recovery-20261004/verify_receive_resampler.py'
        source = fixture.read_text().replace('run(0x7210)\n', '''compare(samples,source_phase,output_phase,initial_count,slip,available,table_phase,shift,gain,bias,expected)
    run(0x7210)
''')
        if args.saturation:
            source = source.replace('gain=512 if available==2 else 1024', 'gain=32767 if available==2 else -32768')
            source = source.replace('full=65536*bias+32768+64*raw*gain', """product=(16*raw*gain)&0xffffffff
      delta=product if product<0x80000000 else product-0x100000000
      full=65536*bias+32768
      for _ in range(4):full=max(-0x80000000,min(0x7fffffff,full+delta))""")
            source = source.replace("overflow excluded before OVM gain stores", "gain stores include signed accumulator saturation")
        source = source.replace("(P/'receive-resampler-verification.json').write_text", 'output_path.write_text')
        source = source.replace('output_path.write_text', 'report.update(production_source_sha256=production_hash, production_tables_sha256=tables_hash, production_cases=len(cases))\noutput_path.write_text')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        exec(compile(source,str(fixture),'exec'), dict(__file__=str(fixture), compare=compare,
             output_path=args.output, production_hash=hashlib.sha256((repo/'mica_qam.c').read_bytes()).hexdigest(),
             tables_hash=hashlib.sha256((repo/'mica_qam_tables.h').read_bytes()).hexdigest()))


if __name__ == '__main__':
    main()
