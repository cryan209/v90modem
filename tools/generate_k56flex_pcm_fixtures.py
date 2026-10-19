#!/usr/bin/env python3
"""Generate research PCM fixtures, NOT hardware recordings.

Usage: python3 tools/generate_k56flex_pcm_fixtures.py ../MicaEmu
The sibling transmitter's points are checked by verify_k56flex_training_tx.py
against resident 5903/5961/1D5A. Its pulse shaper is vendored SpanDSP.
"""
import ctypes as ct
import hashlib
import json
import os
from pathlib import Path
import struct
import sys


def main():
    mica = Path(sys.argv[1]).resolve()
    sys.path.insert(0, str(mica / "tools"))
    from build_k56flex_training_tx import build
    library = build()
    # Do not inherit the sibling's optional report-amplitude experiment.
    os.environ["K56_REPORT_GRID_SCALE"] = "1"
    lib = ct.CDLL(str(library))
    lib.mica_k56_training_new.argtypes = [ct.c_uint] * 6
    lib.mica_k56_training_new.restype = ct.c_void_p
    lib.mica_k56_training_report_length.argtypes = [ct.c_void_p, ct.c_uint]
    lib.mica_k56_training_block.argtypes = [ct.c_void_p, ct.POINTER(ct.c_int16)]
    lib.mica_k56_training_free.argtypes = [ct.c_void_p]
    out = Path(__file__).resolve().parents[1] / "reference-data/k56flex-upstream"
    out.mkdir(parents=True, exist_ok=True)
    manifest = {"source": "MicaEmu research transmitter; not hardware evidence",
                "build": json.loads(library.with_suffix(".json").read_text()),
                "sample_rate": 8000, "format": "signed 16-bit little endian",
                "tap": 18, "high_carrier": 0, "s_symbols": 128,
                "trn_symbols": 512, "report_symbols": 384,
                "report": "8990", "extension": "63", "files": {}}
    for baud in (3200, 2400):
        tx = lib.mica_k56_training_new(baud, 0, 18, 128, 512, 0x638990)
        if not tx:
            raise RuntimeError("transmitter init failed")
        try:
            if lib.mica_k56_training_report_length(tx, 384):
                raise RuntimeError("report length failed")
            block = (ct.c_int16 * 160)()
            pcm = bytearray()
            for _ in range(32):
                if lib.mica_k56_training_block(tx, block) != 160:
                    raise RuntimeError("short PCM block")
                pcm.extend(struct.pack("<160h", *block))
            name = f"{baud}.s16le"
            (out / name).write_bytes(pcm)
            manifest["files"][name] = hashlib.sha256(pcm).hexdigest()
        finally:
            lib.mica_k56_training_free(tx)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(out)


if __name__ == "__main__":
    main()
