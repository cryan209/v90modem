#!/usr/bin/env python3
"""Load a patched firmware image, insisting the device is in BOOTLOADER state.

hsf_fxo_probe --wait --load succeeds trivially when firmware is already running,
because loading is a no-op then.  That silently ran a whole patched-firmware
experiment against the STOCK firmware once.  This waits for family 01
(bootloader, wants firmware), loads, and then requires the family to have
changed to 03 -- so "loaded" means the upload actually happened.
"""
import re, subprocess, sys, time

PROBE = "./hsf_fxo_probe"


def info():
    r = subprocess.run([PROBE], capture_output=True, text=True, timeout=30)
    m = re.search(r"info = ([0-9a-f ]+)", r.stdout + r.stderr)
    return [int(x, 16) for x in m.group(1).split()] if m else None


def main():
    rom = sys.argv[1] if len(sys.argv) > 1 else "hsf_rom_patched.bin"
    secs = int(sys.argv[2]) if len(sys.argv) > 2 else 900
    print(f"waiting for BOOTLOADER state (replug required) -- {rom}", flush=True)
    t0 = time.time()
    while time.time() - t0 < secs:
        i = info()
        if i and i[2] == 1:
            break
        time.sleep(0.2)
    else:
        sys.exit("never saw bootloader state; the device must be replugged")

    r = subprocess.run([PROBE, "--load", "--rom", rom],
                       capture_output=True, text=True, timeout=60)
    out = r.stdout + r.stderr
    print(out.strip(), flush=True)
    i = info()
    if not i or i[2] != 3:
        sys.exit("upload did not take: family is not 03")
    print(f"patched firmware is running ({rom})", flush=True)


if __name__ == "__main__":
    main()
