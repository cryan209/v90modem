#!/usr/bin/env python3
"""Extract ROM_IMAGE from Conexant's c2firmware.h into a flat binary.

The HSF USB modem (0572:1300) comes up in bootloader state on every plug and
wants ~7.4 KB of controller firmware before it will answer anything else.
hsf_fxo.c loads that from a binary file rather than embedding it, so the
Conexant image stays in its original header and is not redistributed here.

  ./tools/hsf_extract_rom.py ~/hsfmodem-src/hsfmodem-7.80.02.06oem/modules/imported/include/c2firmware.h
"""
import argparse
import re
import sys


def extract(path):
    with open(path) as f:
        text = f.read()
    m = re.search(r"ROM_IMAGE\s*\[\s*\]\s*=\s*\{(.*?)\}\s*;", text, re.S)
    if not m:
        sys.exit(f"{path}: no ROM_IMAGE array found")
    return bytes(int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{1,2})", m.group(1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("header", help="path to c2firmware.h")
    ap.add_argument("-o", "--output", default="hsf_rom_image.bin")
    args = ap.parse_args()

    rom = extract(args.header)
    with open(args.output, "wb") as f:
        f.write(rom)

    # 7399 bytes is what 7.80.02.06 ships; a different size is not an error,
    # but it is worth noticing before you push it at the hardware.
    note = "" if len(rom) == 7399 else "  (expected 7399 for hsfmodem-7.80.02.06)"
    print(f"{args.output}: {len(rom)} bytes{note}")
    print(f"first 16: {rom[:16].hex()}")


if __name__ == "__main__":
    main()
