#!/usr/bin/env python3
"""Verify native audio sample-clock accounting with an offline audio engine."""
from pathlib import Path
import subprocess
import tempfile

with tempfile.TemporaryDirectory(prefix='modem-gui-audio-test-') as directory:
    root = Path(directory)
    subprocess.run(['swiftc', '-module-cache-path', str(root/'cache'),
        str(Path(__file__).with_suffix('.swift')), '-o', str(root/'test')], check=True)
    subprocess.run([str(root/'test')], check=True)
