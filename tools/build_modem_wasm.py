#!/usr/bin/env python3
"""Build the browser engine from vendored sources, without touching native objects.
Requires Emscripten 4.0.16 (EMCC may specify its executable).
"""
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / 'web/modem-lab'
BUILD = ROOT / 'reference-data/artifacts/wasm-build'
EMCC = os.environ.get('EMCC', shutil.which('emcc') or '/private/tmp/v90modem-emsdk/upstream/emscripten/emcc')

def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    (WEB / 'dist').mkdir(exist_ok=True)
    engine = re.search(r'^SRCS\s*= (.+)$', (ROOT / 'makefile').read_text(), re.M)[1].split()
    engine = [s for s in engine if s not in ('sip_modem.c', 'data_interface.c', 'fax_class2.c')]
    # Only modem/data DSP; no image, fax, TIFF, JPEG, or host libraries.
    names = '''alloc async bit_operations bitstream complex_filters complex_vector_float
    complex_vector_int crc dds_float dds_int echo fsk g711 godard hdlc logging
    math_fixed modem_echo modem_connect_tones power_meter queue silence_gen
    v17rx v17tx v22bis_rx v22bis_tx v27ter_rx v27ter_tx v29rx v29tx v32bis
    v34rx v34rx_data v34rx_phase3 v34rx_phase4_trn v34tx v34_logging v42 v42bis v8
    vector_float vector_int'''.split()
    sources = [ROOT / s for s in engine] + [ROOT / 'spandsp-master/src' / (s+'.c') for s in names]
    sources += [WEB / 'wasm/browser_peer.c']
    flags = ['-O0', '-g0', '-I'+str(WEB/'wasm'), '-I'+str(ROOT), '-I'+str(ROOT/'spandsp-master/src'),
             '-I'+str(ROOT/'port'), '-include', str(WEB/'wasm/config.h'),
             '-Wno-unused-command-line-argument']
    # Rebuild after any relevant header changes; never reuse native objects.
    headers = sorted(ROOT.glob('*.h')) + sorted((ROOT/'spandsp-master/src').rglob('*.h')) + sorted((WEB/'wasm').glob('*.h'))
    digest = hashlib.sha256(' '.join(flags).encode())
    for p in headers:
        digest.update(p.read_bytes())
    header_key = digest.hexdigest()
    def compile_one(source):
        obj = BUILD / (str(source.relative_to(ROOT)).replace('/', '_')+'.o')
        key = hashlib.sha256(header_key.encode()+source.read_bytes()).hexdigest()
        stamp = obj.with_suffix('.key')
        if not obj.exists() or not stamp.exists() or stamp.read_text()!=key:
            result = subprocess.run([EMCC, *flags, '-c', str(source), '-o', str(obj)], capture_output=True, text=True)
            if result.returncode:
                raise RuntimeError(str(source)+'\n'+result.stderr)
            stamp.write_text(key)
        return str(obj)
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        objects = list(pool.map(compile_one, sources))
    subprocess.run([EMCC, *objects, '-O0', '-sMODULARIZE=1', '-sEXPORT_ES6=1',
                    '-sEXPORT_NAME=createModem', '-sALLOW_MEMORY_GROWTH=1',
                    '-sINITIAL_MEMORY=67108864', '-sSTACK_SIZE=4194304',
                    '-sEXPORTED_FUNCTIONS=["_malloc","_free"]',
                    '-sEXPORTED_RUNTIME_METHODS=["HEAPU8","UTF8ToString"]',
                    '-sENVIRONMENT=web,worker,node', '-o', str(WEB/'dist/modem-engine.mjs')], check=True)
    loader = WEB/'dist/modem-engine.mjs'
    loader.write_text('\n'.join(line.rstrip() for line in loader.read_text().splitlines()).rstrip()+'\n')
    provenance = {
        'compiler': subprocess.check_output([EMCC, '--version'], text=True).splitlines()[0],
        'flags': [f.replace(str(ROOT), '$REPO') for f in flags],
        'sources': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        'headers_sha256': header_key,
    }
    (WEB/'dist/build-info.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print('Built '+str(WEB/'dist/modem-engine.wasm'))

if __name__ == '__main__':
    main()
