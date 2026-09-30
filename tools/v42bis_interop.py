#!/usr/bin/env python3
"""Cross-check SpanDSP against modem-dsp-emu's independent V.42bis codec.
Run after building SpanDSP: python3 tools/v42bis_interop.py --reference ../modem-dsp-emu
"""
import argparse
import ctypes
from pathlib import Path
import random
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--reference', type=Path, required=True)
parser.add_argument('--library', type=Path)
args = parser.parse_args()
sys.path.insert(0, str(args.reference.resolve() / 'tools'))
from v42bis import V42bisEncoder, V42bisDecoder
root = Path(__file__).resolve().parents[1]
suffix = 'dylib' if sys.platform == 'darwin' else 'so'
library = args.library or root / f'spandsp-master/src/.libs/libspandsp.{suffix}'
l = ctypes.CDLL(str(library.resolve()))
callback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int)
l.v42bis_init.restype = ctypes.c_void_p
l.v42bis_init.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                          callback, ctypes.c_void_p, ctypes.c_int,
                          callback, ctypes.c_void_p, ctypes.c_int]
for name in ['compress', 'decompress']:
    getattr(l, 'v42bis_' + name).argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    getattr(l, 'v42bis_' + name + '_flush').argtypes = [ctypes.c_void_p]
l.v42bis_free.argtypes = [ctypes.c_void_p]
l.v42bis_compression_control.argtypes = [ctypes.c_void_p, ctypes.c_int]


def codec(p1, p2):
    enc, dec = bytearray(), bytearray()
    ec = callback(lambda _, p, n: enc.extend(ctypes.string_at(p, n)))
    dc = callback(lambda _, p, n: dec.extend(ctypes.string_at(p, n)))
    state = l.v42bis_init(None, 3, p1, p2, ec, None, 256, dc, None, 256)
    assert state
    return state, enc, dec, ec, dc


count = 0
rng = random.Random(42)
for p1 in [512, 768, 1024, 2048]:
    for p2 in [6, 32, 250]:
        for data in [b'hello', b'ABCDEFGH' * 256, bytes(range(256)) * 32,
                     bytes((0, 51, 102, 153, 204, 255)) * 200,
                     rng.randbytes(8192)]:
            s, enc, dec, ec, dc = codec(p1, p2)
            try:
                py_decoder = V42bisDecoder(p1, p2)
                py_encoder = V42bisEncoder(p1, p2)
                recovered = bytearray()
                for start in range(0, len(data), 73):
                    block = data[start:start + 73]
                    assert l.v42bis_compress(s, block, len(block)) == 0
                    assert l.v42bis_compress_flush(s) == 0
                    for octet in enc:
                        recovered.extend(py_decoder.feed(bytes((octet,))))
                    enc.clear()
                    wire = py_encoder.feed(block) + py_encoder.flush()
                    for octet in wire:
                        one = bytes((octet,))
                        assert l.v42bis_decompress(s, one, 1) == 0
                        l.v42bis_decompress_flush(s)
                assert recovered == data, (p1, p2, 'C->Python')
                assert dec == data, (p1, p2, 'Python->C')
                count += 1
            finally:
                l.v42bis_free(s)

# Exercise the codec's maximum supported dictionary and 12-bit codewords.
s, enc, dec, ec, dc = codec(4096, 6)
try:
    data = rng.randbytes(48000)
    l.v42bis_compression_control(s, 1)
    l.v42bis_compress(s, data, len(data))
    l.v42bis_compress_flush(s)
    assert V42bisDecoder(4096, 6).feed(bytes(enc)) == data
    count += 1
finally:
    l.v42bis_free(s)

# Exercise dynamic mode changes against the independent decoder.
s, enc, dec, ec, dc = codec(1024, 32)
try:
    py_decoder = V42bisDecoder(1024, 32)
    recovered = bytearray()
    expected = bytearray()
    for mode, block in [(1, b'ABCD' * 300), (2, bytes(range(256)) * 3),
                        (1, b'ABCD' * 300)]:
        l.v42bis_compression_control(s, mode)
        l.v42bis_compress(s, block, len(block))
        l.v42bis_compress_flush(s)
        recovered.extend(py_decoder.feed(bytes(enc)))
        expected.extend(block)
        enc.clear()
    assert recovered == expected
    count += 1
finally:
    l.v42bis_free(s)

# RESET must restore escape zero and fresh dictionaries across fragmented input.
s, enc, dec, ec, dc = codec(512, 32)
try:
    prefix = b'abcabc'
    wire = prefix + b'\x00\x02' + b'hello'
    for octet in wire:
        l.v42bis_decompress(s, bytes((octet,)), 1)
        l.v42bis_decompress_flush(s)
    assert dec == prefix + b'hello'
    count += 1
finally:
    l.v42bis_free(s)

# Force ECM then references absent from a fresh dictionary; do not output zeros.
for wire in [b'\x00\x03', b'\x00\x00' + (291).to_bytes(2, 'little'),
             b'\x00\x00' + (2).to_bytes(2, 'little')]:
    s, enc, dec, ec, dc = codec(512, 32)
    try:
        assert l.v42bis_decompress(s, wire, len(wire)) == -1
        assert not dec
        count += 1
    finally:
        l.v42bis_free(s)
ec = callback(lambda *_: None)
assert not l.v42bis_init(None, 3, 65535, 6, ec, None, 256, ec, None, 256)
count += 1
print(f'V.42bis independent interop: {count} cases passed')
