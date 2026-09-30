#!/usr/bin/env python3
"""Check the C V.44 port against modem-dsp-emu's independent Python codec.
Build v44.c as a shared library and pass --library and --reference.
"""
import argparse
import ctypes
from pathlib import Path
import random
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--reference', type=Path, required=True)
parser.add_argument('--library', type=Path, required=True)
args = parser.parse_args()
sys.path.insert(0, str(args.reference.resolve() / 'tools'))
from v44 import V44Encoder, V44Decoder, _BitWriter
lib = ctypes.CDLL(str(args.library.resolve()))
callback = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int)
for side in ['encoder', 'decoder']:
    init = getattr(lib, f'v44_{side}_init')
    init.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, callback, ctypes.c_void_p]
    init.restype = ctypes.c_void_p
    getattr(lib, f'v44_{side}_feed').argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
    getattr(lib, f'v44_{side}_free').argtypes = [ctypes.c_void_p]
lib.v44_encoder_flush.argtypes = [ctypes.c_void_p]

count = 0
rng = random.Random(44)
for codewords, maximum, history in [(256,32,512), (512,32,1024), (768,48,2048),
                                   (2048,64,4096), (4096,255,12000)]:
    for data in [b'hello', b'ABCDEFGH'*256, bytes(range(256))*20,
                 b'abc'*4000, rng.randbytes(4096)]:
        cwire, cplain = bytearray(), bytearray()
        encode_cb = callback(lambda _,p,n: cwire.extend(ctypes.string_at(p,n)))
        decode_cb = callback(lambda _,p,n: cplain.extend(ctypes.string_at(p,n)))
        enc = lib.v44_encoder_init(codewords, maximum, history, encode_cb, None)
        dec = lib.v44_decoder_init(codewords, maximum, history, decode_cb, None)
        assert enc and dec
        try:
            pyenc = V44Encoder(codewords, maximum, history)
            pydec = V44Decoder(codewords, maximum, history)
            recovered = bytearray()
            for start in range(0,len(data),73):
                block = data[start:start+73]
                assert lib.v44_encoder_feed(enc,block,len(block)) == 0
                assert lib.v44_encoder_flush(enc) == 0
                reference = pyenc.feed(block) + pyenc.flush()
                assert cwire == reference, (codewords, maximum, history, start, 'wire mismatch')
                for octet in cwire:
                    recovered.extend(pydec.feed(bytes((octet,))))
                cwire.clear()
                for octet in reference:
                    b=bytes((octet,))
                    assert lib.v44_decoder_feed(dec,b,1) == 0
            assert recovered == data and cplain == data
            count += 1
        finally:
            lib.v44_encoder_free(enc)
            lib.v44_decoder_free(dec)

# Real CX93001 stream: includes overlapping string extension and a split flush.
plain=bytearray()
cb=callback(lambda _,p,n: plain.extend(ctypes.string_at(p,n)))
s=lib.v44_decoder_init(512,32,1024,cb,None)
try:
    wire=bytes.fromhex('c6f05aec68685a8217316632994c2693c96432994c267311a106c500')
    for b in wire:
        assert lib.v44_decoder_feed(s,bytes((b,)),1) == 0
    assert plain == b'cx-v44-'+b'A'*512+b'\r\n'
    count+=1
finally:lib.v44_decoder_free(s)

# Transparent escape cycling, followed by ECM with a fresh compressed dictionary.
plain=bytearray();s=lib.v44_decoder_init(512,32,1024,cb,None)
try:
    w=_BitWriter();w.put(1,1);w.put(0,6);w.align()
    initial=w.take()+b'hi\x00\x01\x33\x01\x66\x00'
    enc=V44Encoder();wire=initial+enc.feed(b'after ECM')+enc.flush()
    for b in wire: assert lib.v44_decoder_feed(s,bytes((b,)),1) == 0
    assert plain == b'hi\x00\x33after ECM'
    count+=1
finally:lib.v44_decoder_free(s)
print(f'V.44 independent interop: {count} cases passed')
