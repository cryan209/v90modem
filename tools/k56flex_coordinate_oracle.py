#!/usr/bin/env python3
"""Compare production feedback slicer with original MICA coordinates fixture.

Runs the retained original-instruction fixture without modifying sibling files.
Its inputs are equalized coordinates; no PCM or receiver mode routing is claimed.
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
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='flex-coordinates-') as tmp:
        library = Path(tmp) / 'flex.so'
        subprocess.run(['cc', '-shared', '-fPIC', '-I', str(repo),
                        str(repo / 'k56flex.c'), '-o', str(library)], check=True)
        native = ctypes.CDLL(str(library))
        native.k56flex_feedback_slice.argtypes = [ctypes.c_int16, ctypes.c_int16]
        native.k56flex_feedback_slice.restype = ctypes.c_uint
        native.k56flex_feedback_dibit.argtypes = [ctypes.POINTER(ctypes.c_uint), ctypes.c_uint]
        native.k56flex_feedback_dibit.restype = ctypes.c_uint
        c_previous = ctypes.c_uint(0)
        fixture = args.mica / 'artifacts/k56flex-recovery-20261004/verify_report_coordinates.py'
        source = fixture.read_text()
        source = source.replace("assert got==expected,(a,b,got,expected)", """assert got==expected,(a,b,got,expected)
   if mode == 'bank8e':
    assert [native.k56flex_feedback_slice(a,b), native.k56flex_feedback_slice(b,a)] == got
    production_counts['sliced_coordinates'] += 2""")
        source = source.replace("first=(0x8990", "c_previous.value = 0\n    first=(0x8990")
        source = source.replace("run(0x7210)\n     wanted=", """if mode == 'bank8e':
      c_decisions = [native.k56flex_feedback_dibit(ctypes.byref(c_previous), native.k56flex_feedback_slice(*pair)) for pair in pairs]
     run(0x7210)
     if mode == 'bank8e':
      assert c_decisions == [c.data(0x8d26), c.data(0x8d27)]
      production_counts['differential_decisions'] += 2
     wanted=""")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        source = source.replace("(P/'report-coordinate-verification.json').write_text", "output_path.write_text")
        source = source.replace("output_path.write_text", "report.update(production_counts=production_counts, production_source_sha256=production_hash)\noutput_path.write_text")
        context = dict(__file__=str(fixture), native=native, c_previous=c_previous,
                       ctypes=ctypes, output_path=args.output,
                       production_counts=dict(sliced_coordinates=0, differential_decisions=0),
                       production_hash=hashlib.sha256((repo / 'k56flex.c').read_bytes()).hexdigest())
        exec(compile(source, str(fixture), 'exec'), context)


if __name__ == '__main__':
    main()
