#!/usr/bin/env python3
"""Build a separate read-only native capture variant; never edit courier-emu.

Changes only host diagnostic retention, optionally stopping capture on the
guest DATA flag. No guest instructions, registers or samples are changed.
The output manifest records all input hashes and the precise diagnostic patch.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--emulator', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--training-only', action='store_true')
    args = p.parse_args()
    source = args.emulator.resolve() / 'native'
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copytree(source, out / 'native')
    core = out / 'native/c5x_core.cpp'
    text = core.read_text()
    patches = [('if (m_pc_captures.size() >= 64)', 'if (m_pc_captures.size() >= 65536)')]
    if args.training_only:
        patches.append(('if (!m_capture_addresses.empty() && previous_pc == m_capture_pc) {',
                        'if (!m_capture_addresses.empty() && previous_pc == m_capture_pc'
                        ' && !(data(0x006f) & 0x0004)) {'))
    for old, new in patches:
        if text.count(old) != 1: raise ValueError('capture hook source profile changed')
        text = text.replace(old, new)
    core.write_text(text)
    library = out / ('libvfc_capture.dylib' if sys.platform == 'darwin' else 'libvfc_capture.so')
    flags = ['-dynamiclib'] if sys.platform == 'darwin' else ['-shared', '-fPIC']
    command = ['c++', '-std=c++17', '-O3', '-Wall', '-Wextra', '-Wpedantic',
               '-fno-semantic-interposition', *flags, str(core),
               str(out / 'native/c5x_capi.cpp'), '-o', str(library)]
    subprocess.run(command, check=True)
    manifest = {'scope': 'read-only diagnostic retention; no guest or bearer changes',
                'patches': patches, 'build_command': command,
                'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in source.iterdir() if p.is_file()},
                'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest()}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(library)


if __name__ == '__main__':
    main()
