#!/usr/bin/env python3
"""Locate the script interpreter's result byte in the controller's internal RAM.

Requires the firmware patched by hsf_patch_fw.py --find-result, which repoints
opcode 0x23 to a stub that writes 0xC3 into a different internal RAM address on
each call, starting at the sweep base and advancing by one.

The probe body is

    27 AA | 23 | 26 | 27 01 | 28 | 36

so 0x27 puts 0xAA in the result byte, our stub writes 0xC3 somewhere, and 0x26
appends the result byte.  Every call reports AA AA 01 -- except the one where
the address the stub just wrote IS the result byte, which reports AA C3 01.
That call names the address, since the sweep base and the call number give it.
"""
import re, subprocess, sys, time

PROBE = "./hsf_fxo_probe"
BODY = "27aa" + "23" + "26" + "2701" + "28" + "36"


def call(secs=2, tries=2):
    for _ in range(tries):
        r = subprocess.run([PROBE, "--raw", BODY, "--stream", str(secs)],
                           capture_output=True, text=True, timeout=60)
        o = r.stdout + r.stderr
        if "NOT RESPONDING" in o or "GET_INFROMATION failed" in o:
            return None, False
        for l in o.splitlines():
            if "data=" in l and "wValue=0x0001" in l:
                m = re.search(r"data=([0-9a-f]+)", l)
                if m and m.group(1).startswith("aa"):
                    return m.group(1), True
    return None, True


def main():
    base = int(sys.argv[1], 0) if len(sys.argv) > 1 else 0x20
    n = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x40
    for i in range(n):
        addr = base + i
        p, alive = call()
        if not alive:
            print(f"  0x{addr:02x}: DEVICE WEDGED -- replug, re-patch with "
                  f"--find-result 0x{addr + 1:02x}")
            return
        hit = p is not None and "c3" in p
        print(f"  wrote 0x{addr:02x}: {p}{'   <<< RESULT BYTE' if hit else ''}",
              flush=True)
        if hit:
            print(f"\nresult byte is internal RAM 0x{addr:02x}")
            return
        time.sleep(0.3)


if __name__ == "__main__":
    main()
